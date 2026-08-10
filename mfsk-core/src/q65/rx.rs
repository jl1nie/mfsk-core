// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65 receiver: aligned audio → 64-tone energies per data symbol →
//! intrinsic probability distributions → QRA belief propagation →
//! 77-bit Wsjt77 message.
//!
//! Mirrors the data-flow stages in WSJT-X `lib/q65_decode.f90`:
//! 1. For each of the 85 symbol slots, run an NSPS-length FFT at the
//!    aligned start sample.
//! 2. Skip the 22 sync slots; for the 63 data slots, snapshot the 64
//!    data-tone bin energies (`base_bin + 1 ..= base_bin + 64`).
//! 3. Convert energies → per-symbol probability distributions over
//!    GF(64) via [`crate::fec::qra::QraCode::mfsk_bessel_metric`].
//! 4. Run [`Q65Codec::decode`] (BP + CRC verify) — recover the 13
//!    info symbols.
//! 5. Re-pack to 77 bits and unpack via the Wsjt77 message codec.
//!
//! Three decoder strategies are wired in:
//! - **Plain AWGN**: Bessel-I0 metric → BP. The default and most
//!   common path.
//! - **AWGN + AP hint**: BP biased by a single
//!   [`crate::msg::ApHint`] (~2 dB threshold gain when the hint is
//!   correct).
//! - **Fast-fading metric**: replaces the Bessel front end with the
//!   Doppler-spread-aware metric required for microwave EME.
//! - **AP list**: BP-free template matching against a pre-encoded
//!   candidate set (e.g. every standard exchange a known callsign
//!   pair could produce). Mirrors `q65_decode_fullaplist`.

use num_complex::Complex;
use rustfft::FftPlanner;

use crate::engine::{DecodeContext, MessageCodec, ModulationParams};
use crate::fec::qra::{FadingModel, Q65Codec, intrinsics_fast_fading};
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::ApHint;
use crate::msg::Q65Message;
use crate::msg::q65::{ap_hint_to_q65_mask, unpack_symbols_to_bits77};

#[cfg(test)]
use super::Q65a30;
use super::sync_pattern::Q65_SYNC_POSITIONS;

/// Es/No metric used by the Q65 intrinsic-probability front end.
///
/// Matches the `EbNodBMetric = 2.8 dB` convention from
/// `q65_init` in `lib/qra/q65/q65.c`. Stored linearised (i.e.
/// `10^(2.8/10) ≈ 1.905`) and scaled by `nm * R = 6 * 15/65` to land
/// on the C reference's `decoderEsNoMetric` value.
fn default_es_no_metric() -> f32 {
    let eb_no_db = 2.8_f32;
    let eb_no = 10.0_f32.powf(eb_no_db / 10.0);
    // BITS_PER_SYMBOL is 6 for every Q65 sub-mode.
    let nm = 6.0_f32;
    let rate = 15.0 / 65.0;
    nm * rate * eb_no
}

/// Extract a `M=64 × N=63` matrix of squared FFT-bin amplitudes for
/// the data symbols of an aligned Q65 frame in sub-mode `P`.
///
/// Layout: `out[64 * k + t]` is the squared amplitude observed for
/// data tone `t` (0..64) at data-symbol position `k` (0..63).
/// Returns `None` if `audio` does not span the full 85-symbol frame
/// at the requested `(start_sample, base_freq_hz)`.
fn extract_data_energies<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Vec<f32>> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let base_bin = (base_freq_hz / df).round() as usize;
    // Sub-mode tone-spacing multiplier in FFT bins. For sub-mode A
    // tone spacing == bin width so this is 1; for B/C/D/E it is
    // 2/4/8/16. The bin-to-tone mapping below scales accordingly.
    let bins_per_tone = (P::TONE_SPACING_HZ / df).round() as usize;

    let highest_bin = base_bin + 64 * bins_per_tone;
    if start_sample + 85 * nsps > audio.len() || highest_bin >= nsps / 2 {
        return None;
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];

    let mut energies = vec![0.0_f32; 64 * 63];
    let mut sync_iter = Q65_SYNC_POSITIONS.iter().peekable();
    let mut k = 0usize;

    for sym_idx in 0..85u32 {
        if sync_iter.peek().is_some_and(|&&p| p == sym_idx) {
            sync_iter.next();
            continue;
        }
        let sym_start = start_sample + sym_idx as usize * nsps;
        for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
            *slot = Complex::new(s, 0.0);
        }
        fft.process_with_scratch(&mut buf, &mut scratch);
        // Q65 data tones are 1..=64 (tone 0 is reserved for sync).
        // The 6-bit symbol value `s` is on bin
        // `base_bin + (s + 1) * bins_per_tone`.
        let row = &mut energies[64 * k..64 * (k + 1)];
        for (tone, slot) in row.iter_mut().enumerate() {
            let bin = base_bin + (tone + 1) * bins_per_tone;
            *slot = buf[bin].norm_sqr();
        }
        k += 1;
    }
    debug_assert_eq!(k, 63);
    Some(energies)
}

/// Extract a wide-energy spectrogram suitable for the fast-fading
/// metric: per data symbol, capture `64 * (2 + bins_per_tone)`
/// consecutive FFT bins centred on the data tones, with `nM = 64`
/// bins of leading and trailing pad to give the spread-weighting
/// window's tails room to roam.
///
/// Output layout matches `q65_intrinsics_fastfading`'s expectation
/// in `lib/qra/q65/q65.c`: row-major `n_data × nBinsPerSymbol`, where
/// `nBinsPerSymbol = 64 * (2 + bins_per_tone)` and the central bin of
/// data tone 0 in symbol 0 lives at offset `64`.
///
/// Returns `None` if the audio is too short, the FFT placement falls
/// off the spectrum, or the wide window cannot fit at the requested
/// `(start_sample, base_freq_hz)` for the sub-mode.
fn extract_data_energies_wide<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Vec<f32>> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let base_bin = (base_freq_hz / df).round() as usize;
    let bins_per_tone = (P::TONE_SPACING_HZ / df).round() as usize;

    // Central bin of data tone 0 (data tones are FFT tones 1..=64; tone 0 is sync).
    if base_bin + bins_per_tone < 64 {
        return None;
    }
    let central_data_tone0 = base_bin + bins_per_tone;
    let wide_start = central_data_tone0 - 64; // 64 bins of leading pad
    let bins_per_symbol = 64 * (2 + bins_per_tone);
    let wide_end_exclusive = wide_start + bins_per_symbol;
    if start_sample + 85 * nsps > audio.len() || wide_end_exclusive > nsps / 2 {
        return None;
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];

    let mut energies = vec![0.0_f32; bins_per_symbol * 63];
    let mut sync_iter = Q65_SYNC_POSITIONS.iter().peekable();
    let mut k = 0usize;

    for sym_idx in 0..85u32 {
        if sync_iter.peek().is_some_and(|&&p| p == sym_idx) {
            sync_iter.next();
            continue;
        }
        let sym_start = start_sample + sym_idx as usize * nsps;
        for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
            *slot = Complex::new(s, 0.0);
        }
        fft.process_with_scratch(&mut buf, &mut scratch);
        let row = &mut energies[bins_per_symbol * k..bins_per_symbol * (k + 1)];
        for (i, slot) in row.iter_mut().enumerate() {
            *slot = buf[wide_start + i].norm_sqr();
        }
        k += 1;
    }
    debug_assert_eq!(k, 63);
    Some(energies)
}

/// Submode index (0..=4 ⇒ A..E) inferred from `P::TONE_SPACING_HZ`
/// over the FFT bin spacing. Equivalent to `log2(bins_per_tone)`.
fn submode_index_from_params<P: ModulationParams>() -> u8 {
    // bins_per_tone for a Q65 sub-mode is always 1, 2, 4, 8, or 16,
    // and equals `2^(letter - 1)`. The FFT length is `NSPS` so bin
    // spacing == 1 baud == TONE_SPACING_HZ for sub-mode A.
    let bpt = (P::TONE_SPACING_HZ / (12_000.0 / P::NSPS as f32)).round() as u32;
    bpt.trailing_zeros() as u8
}

/// One successful Q65 decode with its alignment metadata.
#[derive(Clone, Debug)]
pub struct Q65Result {
    /// Decoded human-readable Wsjt77 message.
    pub message: String,
    /// Tone-0 frequency in Hz.
    pub freq_hz: f32,
    /// Sample index where the frame's symbol 0 begins.
    pub start_sample: usize,
    /// BP iterations consumed by the QRA decoder.
    pub iterations: u32,
    /// Decode-side SNR estimate in dB (WSJT-X 2500 Hz reference
    /// bandwidth convention), from the per-symbol signal-tone vs.
    /// other-tones power ratio (`snr_db_narrow`/`snr_db_wide` in
    /// `q65::rx`). Closes issue #226.
    pub snr_db: f32,
}

/// Shared floor/ratio/dB-conversion step for [`snr_db_narrow`] and
/// [`snr_db_wide`]: `10·log10(xsig/xnoi − 1) − bw_offset_db`, clamped
/// to a −24 dB floor — the same shape as
/// [`crate::engine::llr::compute_snr_db_generic`] (FT8/FT4/FST4) and
/// [`crate::jt65::rx::demodulate_aligned_with_confidence_and_snr`].
fn snr_db_from_sig_noi(xsig: f32, xnoi: f32, bw_offset_db: f32) -> f32 {
    const SNR_FLOOR_DB: f32 = -24.0;
    // WSJT-X's own display convention ceiling (see e.g. WebFT8's
    // `_autoReport()` clamp cited in issue #226). Also serves as this
    // function's answer when `xnoi` is (near) exactly zero: for a
    // perfectly clean synthetic signal sampled with an integer number
    // of cycles per FFT window, DFT orthogonality can leave *zero*
    // measurable leakage in the non-signal bins — that means "no
    // measurable noise", the best case, not the worst. Reporting the
    // floor there (an earlier version of this function did) is
    // backwards, caught by `q65::rx::tests` decoding a noiseless
    // synth and reading `-24 dB` instead of a very clean number.
    const SNR_CEIL_DB: f32 = 49.0;
    if xnoi < f32::EPSILON {
        return if xsig < f32::EPSILON {
            SNR_FLOOR_DB
        } else {
            SNR_CEIL_DB
        };
    }
    let ratio = xsig / xnoi - 1.0;
    if ratio <= 0.001 {
        return SNR_FLOOR_DB;
    }
    (10.0 * ratio.log10() - bw_offset_db).clamp(SNR_FLOOR_DB, SNR_CEIL_DB)
}

/// Bandwidth-normalisation offset to WSJT-X's 2500 Hz reference:
/// `10·log10(2500/df)` where `df` is the per-tone FFT bin bandwidth —
/// same derivation as JT65's estimate (cross-checked against FT8's
/// literal `-27 dB` @ 6.25 Hz and wsprd's literal `-26.3 dB` @ ~5.1 Hz,
/// both within ~1 dB of this formula). Valid for Q65 because
/// [`extract_data_energies`] puts each tone in exactly one per-symbol
/// FFT bin, same as JT65's single-FFT demod — unlike JT9's multi-stage
/// AGC/IFFT/coherent-sum pipeline, where this shape does *not* hold
/// (see `jt9::softsym::symspec2_from_ss2`).
fn q65_bw_offset_db<P: ModulationParams>() -> f32 {
    10.0 * (2500.0 / P::TONE_SPACING_HZ).log10()
}

/// Decode-side SNR from **narrow** per-symbol energies
/// ([`extract_data_energies`] / [`averaged_data_energies`] layout,
/// `energies[64*k + t]`): signal = power at each data symbol's
/// decoded tone (from `codeword`, the 63-symbol channel codeword —
/// either re-encoded via [`fec::qra::Q65Codec::encode`] from the
/// recovered info symbols, or the winning AP-list candidate), noise =
/// mean power of the other 63 tones in the same symbol slot.
fn snr_db_narrow<P: ModulationParams>(energies: &[f32], codeword: &[i32]) -> f32 {
    let mut xsig = 0.0f32;
    let mut xnoi = 0.0f32;
    for (k, &sym) in codeword.iter().enumerate() {
        let t = sym as usize;
        if t >= 64 || 64 * (k + 1) > energies.len() {
            continue;
        }
        let row = &energies[64 * k..64 * (k + 1)];
        let sig = row[t];
        let total: f32 = row.iter().sum();
        xsig += sig;
        xnoi += (total - sig) / 63.0;
    }
    snr_db_from_sig_noi(xsig, xnoi, q65_bw_offset_db::<P>())
}

/// Like [`snr_db_narrow`] but for the **wide** per-symbol energies
/// layout from [`extract_data_energies_wide`] /
/// [`averaged_data_energies_wide`] (used by every fast-fading-metric
/// decode path). Each tone's bin sits at row offset `64 + tone *
/// bins_per_tone` within the `64 * (2 + bins_per_tone)`-wide window —
/// see [`extract_data_energies_wide`]'s doc comment for the layout
/// derivation.
fn snr_db_wide<P: ModulationParams>(energies: &[f32], sample_rate: u32, codeword: &[i32]) -> f32 {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let bins_per_tone = (P::TONE_SPACING_HZ / df).round().max(1.0) as usize;
    let bins_per_symbol = 64 * (2 + bins_per_tone);
    let mut xsig = 0.0f32;
    let mut xnoi = 0.0f32;
    for (k, &sym) in codeword.iter().enumerate() {
        let t = sym as usize;
        if t >= 64 || bins_per_symbol * (k + 1) > energies.len() {
            continue;
        }
        let row = &energies[bins_per_symbol * k..bins_per_symbol * (k + 1)];
        let mut sig = 0.0f32;
        let mut total = 0.0f32;
        // `bins_per_tone >= 1` (clamped above), so `step_by` is safe;
        // the slice naturally runs out at `row.len()`, matching the
        // original manually-indexed loop's `idx >= row.len()` skip.
        let tail = row.get(64..).unwrap_or(&[]);
        for (tone, &v) in tail.iter().step_by(bins_per_tone).take(64).enumerate() {
            total += v;
            if tone == t {
                sig = v;
            }
        }
        xsig += sig;
        xnoi += (total - sig) / 63.0;
    }
    snr_db_from_sig_noi(xsig, xnoi, q65_bw_offset_db::<P>())
}

/// Decode a Q65 signal at a known `(start_sample, base_freq_hz)`
/// for sub-mode `P`.
///
/// Performs FFT-per-symbol, builds intrinsic probability distributions
/// via the Bessel metric, runs QRA belief propagation, verifies the
/// CRC-12, and unpacks the recovered 77-bit Wsjt77 message. Returns
/// `None` if the buffer is too short, BP fails to converge, the CRC
/// rejects the result, or the unpack fails.
pub(crate) fn decode_at_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    decode_at_inner::<P>(audio, sample_rate, start_sample, base_freq_hz, None, ctx)
}

/// Like [`decode_at_for`] but biases the QRA decoder with an AP
/// hint — typically a known callsign pair or "CQ" expectation.
///
/// Empirically gains 2–4 dB at threshold for Q65-30A and is the
/// dominant mechanism that makes 6 m / 70 cm EME workable. The
/// hint is converted to the Q65-specific 13-symbol GF(64) mask via
/// [`ap_hint_to_q65_mask`] and applied to the depunctured intrinsics
/// before BP.
pub(crate) fn decode_at_with_ap_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ap_hint: &ApHint,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    decode_at_inner::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        Some(ap_hint),
        ctx,
    )
}

fn decode_at_inner<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ap_hint: Option<&ApHint>,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    let energies = extract_data_energies::<P>(audio, sample_rate, start_sample, base_freq_hz)?;

    // Energies → intrinsic probability distributions over GF(64).
    let mut intrinsics = vec![0.0_f32; 64 * 63];
    QRA15_65_64_IRR_E23.mfsk_bessel_metric(&mut intrinsics, &energies, 63, default_es_no_metric());

    // QRA + CRC decode, optionally biased by the AP hint.
    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut info_syms = [0_i32; 13];
    let iterations = match ap_hint {
        Some(hint) if hint.has_info() => {
            let (mask, syms) = ap_hint_to_q65_mask(hint);
            codec
                .decode_with_ap(&intrinsics, &mut info_syms, 50, &mask, &syms)
                .ok()?
        }
        _ => codec.decode(&intrinsics, &mut info_syms, 50).ok()?,
    };

    // 13 GF(64) symbols → 77-bit Wsjt77 → human-readable.
    let bits77 = unpack_symbols_to_bits77(&info_syms);
    let text = Q65Message.unpack(&bits77, ctx)?;

    let mut codeword = [0_i32; 63];
    codec.encode(&info_syms, &mut codeword);
    let fallback = snr_db_narrow::<P>(&energies, &codeword);
    let snr_db = super::snr::q65_snr_db::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        &codeword,
        fallback,
    );

    Some(Q65Result {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        iterations,
        snr_db,
    })
}

/// Decode a Q65 signal at a known `(start_sample, base_freq_hz)`
/// using the **fast-fading metric**, optionally biased by an AP
/// hint.
///
/// Targets channels with significant Doppler spread — microwave EME,
/// fast aircraft scatter, and ionoscatter near the noise floor.
/// `b90_ts` is the spread bandwidth × symbol period (dimensionless);
/// typical values: 0.05 for near-AWGN, 0.5 for moderate spread, 2.0+
/// for severe spread (24 GHz EME). `model` selects between Gaussian
/// (libration-limited EME, default) and Lorentzian (heavier-tail
/// scattering channels) calibration shapes.
///
/// Returns `None` for the same reasons as [`decode_at_for`]: short
/// buffer, BP failure, CRC failure, or message-codec rejection.
pub(crate) fn decode_at_fading_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    b90_ts: f32,
    model: FadingModel,
    ap_hint: Option<&ApHint>,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    let energies = extract_data_energies_wide::<P>(audio, sample_rate, start_sample, base_freq_hz)?;

    // Wide energies → fast-fading intrinsic distributions over GF(64).
    let mut intrinsics = vec![0.0_f32; 64 * 63];
    let _state = intrinsics_fast_fading(
        &QRA15_65_64_IRR_E23,
        &mut intrinsics,
        &energies,
        submode_index_from_params::<P>(),
        b90_ts,
        model,
        default_es_no_metric(),
    );

    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut info_syms = [0_i32; 13];
    let iterations = match ap_hint {
        Some(hint) if hint.has_info() => {
            let (mask, syms) = ap_hint_to_q65_mask(hint);
            codec
                .decode_with_ap(&intrinsics, &mut info_syms, 50, &mask, &syms)
                .ok()?
        }
        _ => codec.decode(&intrinsics, &mut info_syms, 50).ok()?,
    };

    let bits77 = unpack_symbols_to_bits77(&info_syms);
    let text = Q65Message.unpack(&bits77, ctx)?;

    let mut codeword = [0_i32; 63];
    codec.encode(&info_syms, &mut codeword);
    let fallback = snr_db_wide::<P>(&energies, sample_rate, &codeword);
    let snr_db = super::snr::q65_snr_db::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        &codeword,
        fallback,
    );

    Some(Q65Result {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        iterations,
        snr_db,
    })
}

/// Scan an audio buffer for Q65 frames in sub-mode `P` using the
/// fast-fading metric. Mirrors [`decode_scan_for`] but routes each
/// candidate through [`decode_at_fading_for`] with the supplied
/// spread parameters.
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_scan_fading_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    b90_ts: f32,
    model: FadingModel,
    ap_hint: Option<&ApHint>,
    on_result: Option<&(dyn Fn(&Q65Result) + Sync)>,
    ctx: &DecodeContext,
) -> Vec<Q65Result> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let cands =
        super::search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Q65Result> = Vec::new();
    for c in cands {
        let Some(decode) = decode_at_fading_for::<P>(
            audio,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            b90_ts,
            model,
            ap_hint,
            ctx,
        ) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == decode.message
                && (prev.freq_hz - decode.freq_hz).abs() <= 4.0
                && (prev.start_sample as i64 - decode.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            if let Some(cb) = on_result {
                cb(&decode);
            }
            seen.push(decode);
        }
    }
    seen
}

/// Decode a Q65 signal at a known `(start_sample, base_freq_hz)`
/// using **AP-list (template-matching) decoding** instead of belief
/// propagation.
///
/// `candidates` is a slice of pre-encoded 63-symbol GF(64) channel
/// codewords — typically built with
/// [`super::ap_list::standard_qso_codewords`] when the application
/// has a known callsign pair but no QSO context. The decoder picks
/// the candidate whose intrinsic log-likelihood exceeds the
/// list-size-adjusted [`crate::fec::qra::Q65_LLH_THRESHOLD`] and
/// has the highest score.
///
/// Returns `None` when no candidate clears the threshold (the most
/// common outcome on weak signals or when the true message is not
/// in the list) or the buffer / FFT placement is out of range.
pub(crate) fn decode_at_with_ap_list_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    candidates: &[[i32; 63]],
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    if candidates.is_empty() {
        return None;
    }

    let energies = extract_data_energies::<P>(audio, sample_rate, start_sample, base_freq_hz)?;

    let mut intrinsics = vec![0.0_f32; 64 * 63];
    QRA15_65_64_IRR_E23.mfsk_bessel_metric(&mut intrinsics, &energies, 63, default_es_no_metric());

    let codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let (idx, info_syms) = codec.decode_with_codeword_list(&intrinsics, candidates)?;

    let bits77 = unpack_symbols_to_bits77(&info_syms);
    let text = Q65Message.unpack(&bits77, ctx)?;

    let fallback = snr_db_narrow::<P>(&energies, &candidates[idx]);
    let snr_db = super::snr::q65_snr_db::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        &candidates[idx],
        fallback,
    );

    Some(Q65Result {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        // The list path does not run BP; report 0 iterations so
        // callers can still distinguish "decoded via templates" from
        // "decoded via BP" if they care.
        iterations: 0,
        snr_db,
    })
}

/// Scan an audio buffer for Q65 frames in sub-mode `P` using
/// AP-list decoding on every coarse-search candidate. Mirrors
/// [`decode_scan_for`] but routes each candidate through
/// [`decode_at_with_ap_list_for`].
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_scan_with_ap_list_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    candidates: &[[i32; 63]],
    on_result: Option<&(dyn Fn(&Q65Result) + Sync)>,
    ctx: &DecodeContext,
) -> Vec<Q65Result> {
    if candidates.is_empty() {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let cands =
        super::search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Q65Result> = Vec::new();
    for c in cands {
        let Some(decode) = decode_at_with_ap_list_for::<P>(
            audio,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            candidates,
            ctx,
        ) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == decode.message
                && (prev.freq_hz - decode.freq_hz).abs() <= 4.0
                && (prev.start_sample as i64 - decode.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            if let Some(cb) = on_result {
                cb(&decode);
            }
            seen.push(decode);
        }
    }
    seen
}

/// Scan an audio buffer for Q65 frames in sub-mode `P` within the
/// search window: runs [`super::search::coarse_search_for`] and tries
/// [`decode_at_for`] on each candidate in score order, collapsing
/// duplicate decodes (same message, frequency within ±4 Hz, start
/// sample within ±1 symbol).
pub(crate) fn decode_scan_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    on_result: Option<&(dyn Fn(&Q65Result) + Sync)>,
    ctx: &DecodeContext,
) -> Vec<Q65Result> {
    decode_scan_inner::<P>(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        None,
        on_result,
        ctx,
    )
}

/// AP-hint variant of [`decode_scan_for`]. Same coarse search; each
/// candidate is decoded with the AP hint applied, which lifts the
/// effective decode threshold by 2–4 dB on Q65-30A and is essential
/// for EME on 6 m and above.
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_scan_with_ap_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    ap_hint: &ApHint,
    on_result: Option<&(dyn Fn(&Q65Result) + Sync)>,
    ctx: &DecodeContext,
) -> Vec<Q65Result> {
    decode_scan_inner::<P>(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Some(ap_hint),
        on_result,
        ctx,
    )
}

#[allow(clippy::too_many_arguments)]
fn decode_scan_inner<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    ap_hint: Option<&ApHint>,
    on_result: Option<&(dyn Fn(&Q65Result) + Sync)>,
    ctx: &DecodeContext,
) -> Vec<Q65Result> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let cands =
        super::search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Q65Result> = Vec::new();
    for c in cands {
        let Some(decode) = decode_at_with_fine_timing_for::<P>(
            audio,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            nsps,
            ap_hint,
            ctx,
        ) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == decode.message
                && (prev.freq_hz - decode.freq_hz).abs() <= 4.0
                && (prev.start_sample as i64 - decode.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            if let Some(cb) = on_result {
                cb(&decode);
            }
            seen.push(decode);
        }
    }
    seen
}

/// Decode depth for the internal `(Δf, Δt, b90)` grid search — mirrors WSJT-X
/// `q65_loops.f90`'s `ndepth` bit field (`iand(ndepth,3)`).
///
/// `Fast` still sweeps the full `b90` range (WSJT-X never skips that
/// dimension, even at its shallowest depth) but tries only the
/// unperturbed `(Δf, Δt) = (0, 0)` cell — no retry. `Normal` matches
/// WSJT-X's typical automatic-scan depth (`ndepth&3==2`). `Deep`
/// matches WSJT-X's "Decode Again" depth (`ndepth&3==3`,
/// `lib/q65_decode.f90:112`: `if(lagain) ndepth=ior(ndepth,3)` —
/// explicitly commented "Use 'Deep' for manual Q65 decodes", i.e.
/// never WSJT-X's own automatic per-slot default).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GridDepth {
    Fast,
    Normal,
    Deep,
}

impl GridDepth {
    /// `(idfmax, idtmax, maxdist)` — `q65_loops.f90:28-41`.
    fn params(self) -> (i32, i32, i32) {
        match self {
            GridDepth::Fast => (1, 1, 4),
            GridDepth::Normal => (3, 3, 5),
            GridDepth::Deep => (5, 5, 5),
        }
    }
}

/// Submode-specific `b90` sweep lower bound (`ibwa`), matching the
/// table at `lib/q65_decode.f90:168-178`. `ibwb = min(15, ibwa+6)`.
fn ibwa_for_submode(submode: u8) -> i32 {
    match submode {
        0 => 1, // A
        1 => 3, // B
        _ => 8, // C, D, E
    }
}

/// Zigzag index → signed offset: `1→0, 2→-1, 3→1, 4→-2, 5→2, ...` —
/// matches `q65_loops.f90`'s `ndf=idf/2; if(mod(idf,2).eq.0) ndf=-ndf`
/// (1-indexed Fortran integer division), so the center cell is always
/// tried first and neighbours alternate ± outward.
fn zigzag_offset(idx1: i32) -> i32 {
    let n = idx1 / 2;
    if idx1 % 2 == 0 { -n } else { n }
}

/// WSJT-X-faithful `(Δf, Δt, b90)` grid search around a coarse
/// candidate — port of `lib/qra/q65/q65_loops.f90`.
///
/// Replaces the previous narrow-window, AWGN-only Bessel metric
/// wrapped in a time-only ±3-step retry: WSJT-X has **no** separate
/// "plain BP" code path for Q65 at all — `q65_dec2` always calls the
/// fast-fading intrinsics (`q65_intrinsics_ff`), swept over a
/// submode-specific `b90` range, combined with a distance-pruned
/// `(Δf, Δt, b90)` grid. `coarse_search_for`'s timing estimate is
/// measurably imprecise at low SNR (issue #171: can land up to ~1/5 of
/// a symbol period off), which is exactly the role WSJT-X's own `idt`
/// retry loop (steps of `nsps/16`) plays — this searches the same
/// space, plus the `Δf` and `b90` dimensions WSJT-X's own decoder
/// never omits.
///
/// The extraction (FFT) is computed once per `(Δf, Δt)` cell and
/// reused across the whole `b90` sub-sweep for that cell — mirroring
/// `q65_loops.f90`'s own structure (`spec64` inside the `idt` loop,
/// `q65_dec2` inside the nested `ibw` loop) and the same
/// extract-once-reuse-many pattern already applied to
/// `decode_multi_period_for`'s fading sweep.
#[allow(clippy::too_many_arguments)]
fn decode_at_grid_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    depth: GridDepth,
    ap_hint: Option<&ApHint>,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let baud = 1.0 / P::SYMBOL_DT;
    let dt_step = (nsps / 16).max(1) as i64;

    let (idfmax, idtmax, maxdist) = depth.params();
    let submode = submode_index_from_params::<P>();
    let ibwa = ibwa_for_submode(submode);
    let ibwb = (ibwa + 6).min(15);
    let ibw0 = (ibwa + ibwb) / 2;

    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut info_syms = [0_i32; 13];
    let mut intrinsics = vec![0.0_f32; 64 * 63];
    let es_no = default_es_no_metric();

    for idf in 1..=idfmax {
        let ndf = zigzag_offset(idf);
        let freq_shift = base_freq_hz + 0.5 * baud * ndf as f32;
        for idt in 1..=idtmax {
            let ndt = zigzag_offset(idt);
            let ndist_ft = ndf * ndf + ndt * ndt;
            if ndist_ft > maxdist {
                // Even the closest b90 (distance 0) can't satisfy the
                // bound at this (Δf,Δt) — skip the FFT extraction
                // entirely rather than computing it for nothing.
                continue;
            }
            let dt_offset = ndt as i64 * dt_step;
            let Ok(shifted_start) = usize::try_from(start_sample as i64 + dt_offset) else {
                continue;
            };
            let Some(energies) =
                extract_data_energies_wide::<P>(audio, sample_rate, shifted_start, freq_shift)
            else {
                continue;
            };

            for ibw in ibwa..=ibwb {
                // At the unperturbed (Δf,Δt)=(0,0) cell, WSJT-X always
                // runs a full, UNPRUNED ibwa..=ibwb sweep first —
                // `q65_dec_q012` (`lib/qra/q65/q65.f90:381`), called
                // from `q65_dec0` before `q65_loops` ever runs. Only
                // once that full-range attempt fails does `q65_loops`
                // itself run, and *it* prunes by `maxdist` at every
                // (Δf,Δt) including (0,0) — but by then ibwa..ibwb at
                // (0,0) is already known to have failed, so the
                // pruning there is redundant, not restrictive. Pruning
                // it here too (as an earlier port did) silently drops
                // the low-ibw end for wide-ibwa submodes (C/D/E) that
                // matters most for near-zero-fading signals, producing
                // a measured ~4 dB sensitivity regression vs real jt9
                // (`-d 1`, `docs/notes/Q65_BENCHMARK.md`).
                let ndist = ndist_ft + (ibw - ibw0) * (ibw - ibw0);
                if (ndf != 0 || ndt != 0) && ndist > maxdist {
                    continue;
                }
                let b90 = 1.72_f32.powi(ibw);
                // `q65_loops.f90:73` caps b90 at 345 Hz for the (Δf,Δt)
                // retry cells; `q65_dec_q012`'s full (0,0) sweep has no
                // such cap, so only apply it off-center.
                if (ndf != 0 || ndt != 0) && b90 > 345.0 {
                    continue;
                }
                let b90_ts = b90 / baud;

                // `q65_dec1`/`q65_dec2` (`q65.f90:598,627`) both
                // hardcode `nFadingModel=1` — WSJT-X's own automatic
                // Q65 decode always uses Lorentzian here, never
                // Gaussian (the Gaussian/Lorentzian choice only varies
                // in the multi-period fading sweep,
                // `decode_multi_period_for`, which faithfully tries
                // both).
                let _state = intrinsics_fast_fading(
                    &QRA15_65_64_IRR_E23,
                    &mut intrinsics,
                    &energies,
                    submode,
                    b90_ts,
                    FadingModel::Lorentzian,
                    es_no,
                );

                let result = match ap_hint {
                    Some(hint) if hint.has_info() => {
                        let (mask, syms) = ap_hint_to_q65_mask(hint);
                        codec.decode_with_ap(&intrinsics, &mut info_syms, 50, &mask, &syms)
                    }
                    _ => codec.decode(&intrinsics, &mut info_syms, 50),
                };
                let Ok(iterations) = result else { continue };

                let bits77 = unpack_symbols_to_bits77(&info_syms);
                let Some(text) = Q65Message.unpack(&bits77, ctx) else {
                    continue;
                };
                let mut codeword = [0_i32; 63];
                codec.encode(&info_syms, &mut codeword);
                let fallback = snr_db_wide::<P>(&energies, sample_rate, &codeword);
                let snr_db = super::snr::q65_snr_db::<P>(
                    audio,
                    sample_rate,
                    shifted_start,
                    freq_shift,
                    &codeword,
                    fallback,
                );
                return Some(Q65Result {
                    message: text,
                    freq_hz: freq_shift,
                    start_sample: shifted_start,
                    iterations,
                    snr_db,
                });
            }
        }
    }
    None
}

/// Try decoding at a coarse candidate's reported alignment via the
/// WSJT-X-faithful `(Δf, Δt, b90)` grid search — see
/// [`decode_at_grid_for`]. Uses `GridDepth::Fast`, matching WSJT-X's
/// own automatic per-slot decode depth (confirmed against `jt9`'s CLI
/// default, `-d 1`).
#[allow(clippy::too_many_arguments)]
fn decode_at_with_fine_timing_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    _nsps: usize,
    ap_hint: Option<&ApHint>,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    decode_at_grid_for::<P>(
        audio,
        sample_rate,
        start_sample,
        freq_hz,
        GridDepth::Fast,
        ap_hint,
        ctx,
    )
}

// ──────────────────────────────────────────────────────────────────────────
// Multi-period averaging — WSJT-X `iavg=1,2` parity.
//
// Mirrors `lib/q65_decode.f90`'s averaged decode flow: maintain an
// EMA over the per-slot spectrogram, run coarse search on the
// running average, and on each candidate try the AP-list / fading /
// plain BP ladder against energies averaged across the slots seen so
// far. Stateless (caller owns the slot buffer) — see the docstring
// on `decode_multi_period_for` for the call shape.
// ──────────────────────────────────────────────────────────────────────────

/// Average per-symbol FFT energies (`extract_data_energies` output)
/// element-wise across `audio_slots[..=current]` at the candidate
/// `(start_sample, base_freq_hz)`. Returns `None` if no slot yields
/// usable energies.
fn averaged_data_energies<P: ModulationParams>(
    audio_slots: &[&[f32]],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Vec<f32>> {
    let mut accum: Option<Vec<f32>> = None;
    let mut count = 0_usize;
    for &audio in audio_slots {
        let Some(e) = extract_data_energies::<P>(audio, sample_rate, start_sample, base_freq_hz)
        else {
            continue;
        };
        match accum.as_mut() {
            Some(a) => {
                for (slot, v) in a.iter_mut().zip(&e) {
                    *slot += *v;
                }
            }
            None => accum = Some(e),
        }
        count += 1;
    }
    let mut accum = accum?;
    if count > 1 {
        let inv = 1.0_f32 / count as f32;
        for v in &mut accum {
            *v *= inv;
        }
    }
    Some(accum)
}

/// Wide-spectrogram variant of [`averaged_data_energies`] for the
/// fast-fading metric path.
fn averaged_data_energies_wide<P: ModulationParams>(
    audio_slots: &[&[f32]],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Vec<f32>> {
    let mut accum: Option<Vec<f32>> = None;
    let mut count = 0_usize;
    for &audio in audio_slots {
        let Some(e) =
            extract_data_energies_wide::<P>(audio, sample_rate, start_sample, base_freq_hz)
        else {
            continue;
        };
        match accum.as_mut() {
            Some(a) => {
                for (slot, v) in a.iter_mut().zip(&e) {
                    *slot += *v;
                }
            }
            None => accum = Some(e),
        }
        count += 1;
    }
    let mut accum = accum?;
    if count > 1 {
        let inv = 1.0_f32 / count as f32;
        for v in &mut accum {
            *v *= inv;
        }
    }
    Some(accum)
}

/// Run the AP-list decoder against averaged narrow energies.
///
/// `energies` is the caller's already-extracted
/// [`averaged_data_energies`] output — see [`decode_multi_period_for`]'s
/// candidate loop, which extracts it once per candidate and passes the
/// same slice into this function and [`decode_averaged_plain_for`]
/// (Stage B and Stage C-plain used to each call `averaged_data_energies`
/// independently with identical arguments, extracting the same FFT
/// energies twice per candidate that reaches Stage C-plain).
#[allow(clippy::too_many_arguments)]
fn decode_averaged_ap_list_for<P: ModulationParams>(
    energies: &[f32],
    history: &[&[f32]],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    candidates: &[[i32; 63]],
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    if candidates.is_empty() {
        return None;
    }

    let mut intrinsics = vec![0.0_f32; 64 * 63];
    QRA15_65_64_IRR_E23.mfsk_bessel_metric(&mut intrinsics, energies, 63, default_es_no_metric());

    let codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let (idx, info_syms) = codec.decode_with_codeword_list(&intrinsics, candidates)?;

    let bits77 = unpack_symbols_to_bits77(&info_syms);
    let text = Q65Message.unpack(&bits77, ctx)?;

    let fallback = snr_db_narrow::<P>(energies, &candidates[idx]);
    let snr_db = super::snr::q65_snr_db_averaged::<P>(
        history,
        sample_rate,
        start_sample,
        base_freq_hz,
        &candidates[idx],
        fallback,
    );

    Some(Q65Result {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        iterations: 0,
        snr_db,
    })
}

/// Run the fast-fading metric BP decoder against averaged wide
/// energies **already extracted** by the caller.
///
/// Split out of the former `decode_averaged_fading_for` so
/// [`decode_multi_period_for`]'s `b90 × model` sweep can call
/// [`averaged_data_energies_wide`] once per candidate and reuse the
/// same energies buffer across all 6 combinations, instead of paying
/// for the FFT-based extraction (and slot-averaging) redundantly on
/// every sweep step — the extraction depends only on
/// `(audio_slots, start_sample, base_freq_hz)`, never on `b90_ts`/
/// `model`, so those 6 calls were doing bit-identical extraction work
/// 6 times over.
///
/// `codec`/`intrinsics` are allocated fresh per call — **deliberately
/// not** hoisted/reused across the `b90 × model` sweep the way
/// [`decode_at_grid_for`]'s own `(Δf,Δt,ibw)` sweep hoists its codec.
/// That hoist was tried (perf-review follow-up to the FT8/FST4 pass)
/// and measured a real, reproducible ~8% regression on the Q65-30A
/// golden test (`q65_multi_period_speed_diag`) despite `Q65Codec::
/// decode`/`intrinsics_fast_fading` both fully overwriting their
/// output buffers every call — reusing the same codec across 6 calls
/// with similar-but-different intrinsics apparently perturbs
/// `Q65Codec`'s internal BP scratch/extrinsic-message state enough to
/// change the *convergence path* (not the final answer — recall tests
/// stayed byte-identical) in a way that costs more than the ~32KB
/// allocation it was meant to save. Not investigated further: BP
/// iteration cost dominating over allocation cost is the same
/// conclusion yesterday's FT8/FST4 pass reached for its own BP-scratch
/// item, just here the reuse made things *worse* instead of neutral.
#[allow(clippy::too_many_arguments)]
fn decode_fading_with_energies<P: ModulationParams>(
    energies: &[f32],
    history: &[&[f32]],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    b90_ts: f32,
    model: FadingModel,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    let mut intrinsics = vec![0.0_f32; 64 * 63];
    let _state = intrinsics_fast_fading(
        &QRA15_65_64_IRR_E23,
        &mut intrinsics,
        energies,
        submode_index_from_params::<P>(),
        b90_ts,
        model,
        default_es_no_metric(),
    );

    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut info_syms = [0_i32; 13];
    let iterations = codec.decode(&intrinsics, &mut info_syms, 50).ok()?;

    let bits77 = unpack_symbols_to_bits77(&info_syms);
    let text = Q65Message.unpack(&bits77, ctx)?;

    let mut codeword = [0_i32; 63];
    codec.encode(&info_syms, &mut codeword);
    let fallback = snr_db_wide::<P>(energies, sample_rate, &codeword);
    let snr_db = super::snr::q65_snr_db_averaged::<P>(
        history,
        sample_rate,
        start_sample,
        base_freq_hz,
        &codeword,
        fallback,
    );

    Some(Q65Result {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        iterations,
        snr_db,
    })
}

/// Run plain Bessel-metric BP against averaged narrow energies.
///
/// `energies` — see [`decode_averaged_ap_list_for`]'s doc comment;
/// same already-extracted-by-the-caller convention.
fn decode_averaged_plain_for<P: ModulationParams>(
    energies: &[f32],
    history: &[&[f32]],
    sample_rate: u32,
    base_freq_hz: f32,
    start_sample: usize,
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    let mut intrinsics = vec![0.0_f32; 64 * 63];
    QRA15_65_64_IRR_E23.mfsk_bessel_metric(&mut intrinsics, energies, 63, default_es_no_metric());

    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut info_syms = [0_i32; 13];
    let iterations = codec.decode(&intrinsics, &mut info_syms, 50).ok()?;

    let bits77 = unpack_symbols_to_bits77(&info_syms);
    let text = Q65Message.unpack(&bits77, ctx)?;

    let mut codeword = [0_i32; 63];
    codec.encode(&info_syms, &mut codeword);
    let fallback = snr_db_narrow::<P>(energies, &codeword);
    let snr_db = super::snr::q65_snr_db_averaged::<P>(
        history,
        sample_rate,
        start_sample,
        base_freq_hz,
        &codeword,
        fallback,
    );

    Some(Q65Result {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        iterations,
        snr_db,
    })
}

/// Multi-period averaging Q65 decode for sub-mode `P`. Mirrors WSJT-X's
/// `iavg=1`/`iavg=2` averaged-decode path from
/// [`q65_decode.f90`](https://sourceforge.net/p/wsjt/wsjtx/ci/main/tree/lib/q65_decode.f90)
/// — the strategy that lets ionoscatter and weak EME signals decode
/// when single-period BP/fading cannot.
///
/// The function processes the slots in order, maintaining an
/// **exponential moving average** of the per-slot spectrogram with
/// time constant `min(navg, 4)` (`u = 1.0 / min(i+1, 4)`, matching the
/// `lib/qra/q65/q65.f90:300-304` accumulator). At each slot the
/// running-average spectrogram drives a coarse sync search, and for
/// every surviving candidate a 3-stage decode ladder is tried
/// against energies averaged across all slots seen so far:
///
/// 1. **AP-list** — when `ap_codewords.is_some()`. Mirrors `iavg=1`'s
///    q3 path. Cheap relative to the rest, included when caller has a
///    plausible call/grid pair (see [`super::ap_list::standard_qso_codewords`]).
/// 2. **Fast-fading metric BP** — sweeps `b90·Ts ∈ {3, 8, 15}` ×
///    `{Gaussian, Lorentzian}`. Covers the realistic ionoscatter +
///    EME spread regimes.
/// 3. **Plain Bessel BP** — last-resort AWGN-only fallback.
///
/// Returns at most one decode per slot (the first one that succeeds
/// at any stage), deduped by `(message, ±4 Hz freq)` so a stable QSO
/// call only counts once. Single-period decodes are *not* re-run
/// inside this function — callers who want them should call
/// [`decode_scan_for`] / [`decode_scan_fading_for`] separately.
///
/// Empty `audio_slots` returns an empty Vec.
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_multi_period_for<P: ModulationParams>(
    audio_slots: &[&[f32]],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    ap_codewords: Option<&[[i32; 63]]>,
    on_result: Option<&(dyn Fn(&Q65Result) + Sync)>,
    ctx: &DecodeContext,
) -> Vec<Q65Result> {
    use super::search::{Spectrogram, coarse_search_on_spec_for};

    let mut output: Vec<Q65Result> = Vec::new();
    if audio_slots.is_empty() {
        return output;
    }

    // Initialise EMA from slot 0.
    let mut ema_spec = Spectrogram::build_for::<P>(audio_slots[0], sample_rate);
    if ema_spec.n_time == 0 {
        return output;
    }

    let b90_ladder = [3.0_f32, 8.0, 15.0];
    let fading_models = [FadingModel::Gaussian, FadingModel::Lorentzian];

    for (i, &audio) in audio_slots.iter().enumerate() {
        if i > 0 {
            let slot_spec = Spectrogram::build_for::<P>(audio, sample_rate);
            // EMA update: weight = 1 / min(navg, 4) — matches WSJT-X's
            // `ntc = min(navg, 4); u = 1.0/ntc` accumulator. After the
            // 4th slot the time constant saturates, so older history
            // decays at a fixed rate (~25%/slot).
            if slot_spec.n_time == ema_spec.n_time
                && slot_spec.n_freq == ema_spec.n_freq
                && slot_spec.mags_sqr.len() == ema_spec.mags_sqr.len()
            {
                let weight = 1.0_f32 / ((i + 1).min(4) as f32);
                let one_minus = 1.0 - weight;
                for (e, s) in ema_spec.mags_sqr.iter_mut().zip(&slot_spec.mags_sqr) {
                    *e = weight * *s + one_minus * *e;
                }
                ema_spec.noise_per_bin =
                    weight * slot_spec.noise_per_bin + one_minus * ema_spec.noise_per_bin;
            }
            // (else: dimension mismatch, keep prior EMA — this slot
            // still contributes to per-candidate energy averaging.)
        }

        let candidates =
            coarse_search_on_spec_for::<P>(&ema_spec, sample_rate, nominal_start_sample, params);

        let history = &audio_slots[..=i];
        let mut slot_decode: Option<Q65Result> = None;

        'candidate_loop: for cand in candidates {
            // Narrow energies feed both Stage B and Stage C-plain below
            // — extraction depends only on `(history, cand.start_sample,
            // cand.freq_hz)`, never on `ap_codewords`/the BP metric used,
            // so it's shared instead of each stage extracting
            // independently (was 2 redundant extractions per candidate
            // that reached Stage C-plain, mirroring the wide/fading-path
            // fix directly below). Lazy (`Option<Option<Vec<f32>>>` via
            // `get_or_insert_with`), not eager: Stage B is skipped
            // entirely when `ap_codewords` is `None`, and Stage C-plain
            // is never reached when Stage B or Stage C-fading already
            // succeeded — an eager extraction here would pay for work
            // neither stage ends up needing in either of those common
            // cases (caught by a same-session A/B measurement showing a
            // small but consistent regression vs. computing it eagerly).
            // Outer `Option` tracks "extracted yet?", inner tracks
            // "did extraction succeed?" — plain lazy-init, no closure
            // (a closure returning a borrow of its own captured state,
            // called from two different points in this loop body, runs
            // into exactly the streaming-iterator borrow-checker
            // friction Rust is known for; inlining the two call sites
            // is simpler than working around it).
            let mut energies_narrow: Option<Option<Vec<f32>>> = None;

            // Stage B — AP-list decode on averaged narrow energies.
            if let Some(codewords) = ap_codewords
                && let Some(energies) = energies_narrow
                    .get_or_insert_with(|| {
                        averaged_data_energies::<P>(
                            history,
                            sample_rate,
                            cand.start_sample,
                            cand.freq_hz,
                        )
                    })
                    .as_deref()
                && let Some(d) = decode_averaged_ap_list_for::<P>(
                    energies,
                    history,
                    sample_rate,
                    cand.start_sample,
                    cand.freq_hz,
                    codewords,
                    ctx,
                )
            {
                slot_decode = Some(d);
                break 'candidate_loop;
            }

            // Stage C-fading — fast-fading metric BP, b90 × model sweep.
            // Extraction (FFT + slot-averaging) depends only on
            // `(history, cand.start_sample, cand.freq_hz)`, not on
            // `b90`/`model` — computed once here and reused across all
            // 6 sweep combinations (was 6 redundant extractions).
            // `codec`/`intrinsics` are allocated fresh inside
            // `decode_fading_with_energies` on every one of the 6 sweep
            // calls below — see that function's doc comment for why
            // this is deliberate, not an oversight.
            if let Some(energies) = averaged_data_energies_wide::<P>(
                history,
                sample_rate,
                cand.start_sample,
                cand.freq_hz,
            ) {
                for &b90 in &b90_ladder {
                    for &model in &fading_models {
                        if let Some(d) = decode_fading_with_energies::<P>(
                            &energies,
                            history,
                            sample_rate,
                            cand.start_sample,
                            cand.freq_hz,
                            b90,
                            model,
                            ctx,
                        ) {
                            slot_decode = Some(d);
                            break 'candidate_loop;
                        }
                    }
                }
            }

            // Stage C-plain — Bessel-metric BP fallback. Reuses Stage
            // B's extraction if it already ran (`ap_codewords: Some`),
            // or computes it fresh here on first use otherwise (never
            // computed at all if Stage B or Stage C-fading already
            // succeeded above, since this line is then unreached).
            if let Some(energies) = energies_narrow
                .get_or_insert_with(|| {
                    averaged_data_energies::<P>(
                        history,
                        sample_rate,
                        cand.start_sample,
                        cand.freq_hz,
                    )
                })
                .as_deref()
                && let Some(d) = decode_averaged_plain_for::<P>(
                    energies,
                    history,
                    sample_rate,
                    cand.freq_hz,
                    cand.start_sample,
                    ctx,
                )
            {
                slot_decode = Some(d);
                break 'candidate_loop;
            }
        }

        if let Some(d) = slot_decode {
            let dup = output
                .iter()
                .any(|prev| prev.message == d.message && (prev.freq_hz - d.freq_hz).abs() <= 4.0);
            if !dup {
                if let Some(cb) = on_result {
                    cb(&d);
                }
                output.push(d);
            }
        }
    }

    output
}

#[cfg(test)]
mod tests {
    use super::super::decode_request::DecodeRequest;
    use super::super::tx::synthesize_standard;
    use super::*;

    #[test]
    fn aligned_decode_recovers_clean_message() {
        let freq = 1500.0;
        let audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("pack + synth");
        let result = DecodeRequest::<Q65a30>::sniper(&audio, 12_000, 0, freq)
            .decode()
            .expect("clean aligned decode must succeed");
        assert_eq!(result.message, "CQ K1ABC FN42");
        assert_eq!(result.start_sample, 0);
        assert!((result.freq_hz - freq).abs() < 0.001);
    }

    #[test]
    fn scan_recovers_clean_message_without_alignment_hint() {
        let freq = 1500.0;
        let audio =
            synthesize_standard("CQ", "JA1ABC", "PM95", 12_000, freq, 0.3).expect("pack + synth");
        let decodes =
            DecodeRequest::<Q65a30>::new(&audio, 12_000, 0, super::super::SearchParams::default())
                .decode();
        assert!(!decodes.is_empty(), "scan must find a clean signal");
        assert_eq!(decodes[0].message, "CQ JA1ABC PM95");
    }

    #[test]
    fn scan_with_no_signal_returns_empty() {
        // Pure silence (well, low noise) must not produce false decodes.
        let audio = vec![0.0_f32; 12_000 * 30];
        let decodes =
            DecodeRequest::<Q65a30>::new(&audio, 12_000, 0, super::super::SearchParams::default())
                .decode();
        assert!(
            decodes.is_empty(),
            "got false decodes from silence: {decodes:#?}"
        );
    }

    /// `.hash_table(..)` — proves it actually reaches
    /// `Q65Message.unpack`'s hash-table-aware dispatch, not just that
    /// it compiles. Same differential-test shape used for MSK144's
    /// `decode_slot_with_hash_table` and, earlier the same day, for
    /// `DecodeRequest::fft_cache`: a same-audio round-trip can't
    /// distinguish "resolved" from "left unresolved" when there's
    /// nothing to resolve, so this pins a message with a genuinely
    /// hashed callsign and checks both the without- and with-table
    /// outcomes.
    #[test]
    fn sniper_hash_table_resolves_hashed_callsign() {
        use super::super::tx::{encode_channel_symbols, synthesize_audio_for};
        use crate::msg::hash_table::CallsignHashTable;
        use crate::msg::wsjt77::pack77_type4;
        use alloc::sync::Arc;

        // Type 4: non-standard call "JL1NIE/1" + hashed standard call
        // "JA1ABC" — same recipe as msg::wsjt77's own
        // `type4_hash_register_then_resolve` and MSK144's
        // `decode_slot_with_hash_table_resolves_hashed_callsign`.
        let bits77 = pack77_type4("JL1NIE/1", "JA1ABC", "", false).expect("pack77_type4 failed");
        let tones = encode_channel_symbols(&bits77);
        let freq = 1500.0;
        let audio = synthesize_audio_for::<Q65a30>(&tones, 12_000, freq, 0.3);

        // Without a hash table: unresolved placeholder.
        let no_ht = DecodeRequest::<Q65a30>::sniper(&audio, 12_000, 0, freq)
            .decode()
            .expect("clean aligned decode must succeed");
        assert!(
            no_ht.message.contains("JL1NIE/1") && no_ht.message.contains("<...>"),
            "expected an unresolved '<...>' decode without a hash table: {no_ht:?}"
        );

        // With a hash table pre-seeded with the standard call: resolved.
        let mut ht = CallsignHashTable::new();
        ht.insert("JA1ABC");
        let with_ht = DecodeRequest::<Q65a30>::sniper(&audio, 12_000, 0, freq)
            .hash_table(Arc::new(ht))
            .decode()
            .expect("clean aligned decode must succeed");
        assert!(
            with_ht.message.contains("JL1NIE/1") && with_ht.message.contains("<JA1ABC>"),
            "expected the hashed callsign to resolve via the supplied table: {with_ht:?}"
        );
    }
}
