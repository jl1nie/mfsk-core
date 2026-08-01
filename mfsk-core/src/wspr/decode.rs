//! Top-level WSPR decode entry point.
//!
//! Given aligned audio, a candidate base frequency, and a target start
//! sample, runs demod → deinterleave → Fano → message unpack. No coarse
//! search here; a later module will wrap this with a (freq × time) scan.

use alloc::vec::Vec;

use crate::msg::WsprMessage;

#[cfg(any())]
use super::search::coarse_search;
use super::search::SearchParams;

/// One successful WSPR decode.
#[derive(Clone, Debug)]
pub struct WsprResult {
    /// Recovered message payload.
    pub message: WsprMessage,
    /// Base frequency (tone 0) used for demodulation.
    pub freq_hz: f32,
    /// Sample index at which symbol 0 started, in the *caller's* audio
    /// buffer. **Clamped to 0** when the signal actually started before
    /// the buffer (negative-dt case); use [`Self::dt_sec`] for the
    /// signed offset that matches wsprd's reporting.
    pub start_sample: usize,
    /// wsprd-equivalent `dt`: signal-start offset in seconds, relative
    /// to the WSPR nominal anchor (slot start + 1 s). Positive values
    /// = signal arrived late, negative = arrived early. Range that
    /// `decode_scan` can express: `−NEGATIVE_DT_PAD_SEC .. +∞`.
    pub dt_sec: f32,
    /// 50-bit FEC info payload returned by Fano. Used by
    /// `decode_scan_subtract` to reconstruct the 162-channel-symbol
    /// stream and subtract the signal from the audio for SIC.
    pub info_bits: [u8; 50],
    /// wsprd-calibrated SNR estimate (dB, WSPR→2500 Hz reference
    /// bandwidth), from the coarse-search candidate that produced this
    /// decode — same `10·log10(smspec) − 26.3` formula wsprd itself
    /// reports next to a spot (`wsprd.c:1093`, see
    /// [`super::coarse_baseband::BasebandCandidate::snr_db`]). Set by
    /// [`decode_scan`] / [`decode_scan_subtract`], which always go
    /// through the coarse search; direct [`decode_at`] /
    /// [`decode_at_baseband`] / [`decode_at_baseband_nblocks`] calls
    /// have no coarse candidate to derive it from and leave this `0.0`.
    pub snr_db: f32,
}

/// Decode one WSPR frame at a known (freq, start_sample). Returns `None`
/// if the Fano decoder fails to converge or the message doesn't unpack.
///
/// Routes through the new 375 Hz baseband demod path
/// ([`super::demod::bit_metrics_from_audio`]) — port of WSJT-X
/// `wsprd.c::noncoherent_sequence_detection` at `nblock=1`. Per-symbol
/// explicit 4-tone Goertzel on the decimated baseband. Closes the
/// W5BIT and NM7J gaps that the previous 12 kHz / 8192-pt-FFT path
/// couldn't reach.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
) -> Option<WsprResult> {
    decode_at_with_drift(audio, sample_rate, start_sample, freq_hz, 0.0)
}

/// Same as [`decode_at`] but with an explicit drift estimate
/// (`drift_hz` is total drift across the 110.6 s frame; matches
/// wsprd's `drift1`). The caller supplies the drift for now; a
/// future drift-search slice will sweep it inside the decode loop
/// like wsprd does.
/// Decode at known alignment using a pre-decimated baseband. Avoids
/// the O(NFFT1) decimation cost when many candidates share the same
/// audio (e.g. inside `decode_scan` / `decode_scan_subtract`).
///
/// `idat`, `qdat`: 46080-sample 375 Hz baseband from
/// [`super::baseband::decimate_to_baseband`].
/// `freq_hz`: tone-0 frequency in audio Hz (matches our coarse-search
/// convention; converted to wsprd's tone-center via `+1.5·df` inside).
/// `start_sample`: audio-rate sample where symbol 0 starts.
pub fn decode_at_baseband(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
) -> Option<WsprResult> {
    decode_at_baseband_nblocks(
        idat,
        qdat,
        sample_rate,
        start_sample,
        freq_hz,
        drift_hz,
        &[1],
    )
}

/// Variant of [`decode_at_baseband`] that tries multiple `nblock`
/// values (e.g. `&[1, 2, 3]`) for coherent block detection. The hot
/// loop scales O(`nblocks.len()`); used by pass 2 of `decode_scan`
/// where the noise-floor reduction makes the extra cost worth it.
pub fn decode_at_baseband_nblocks(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    nblocks: &[usize],
) -> Option<WsprResult> {
    use crate::engine::{FecCodec, FecOpts, MessageCodec};
    // `freq_hz` follows our tone-0 convention (matches `synthesize_audio`
    // and `coarse_search.freq_hz`); wsprd's `noncoherent_sequence_detection`
    // takes the signal CENTER, so we add 1.5·tone_spacing here and sweep
    // around that point. A wide ±4 Hz sweep is needed because real-world
    // coarse picks can land 2-4 Hz off the true centre when nearby
    // signals or sub-bin offsets perturb the score landscape.
    // wsprd-style refine→demod cascade. Replaces the previous
    // brute-force 15×7 (Δf, Δdt) sweep, which ran Fano on every cell
    // and burnt seconds of CPU per candidate. Architecture follows
    // `wsprd.c:1217-1280`: mode 0 (lag refine) → mode 1 (freq refine)
    // → mode 2 (final demod), with Fano run only at the refined
    // alignment (not per cell).
    let f0_center_init = freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
    let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
    let lag_baseband_init = start_sample as i32 / 32;
    let codec = crate::fec::ConvFano;

    // Mode 0: lag refine. wsprd uses lagstep=64 baseband-samples,
    // ±128 around shift1 → 5 lags. Per cell: tone_amplitudes
    // (≈ 700 k float ops) + sync_score — no Fano.
    let mut best_lag = lag_baseband_init;
    let mut best_lag_sync = f32::NEG_INFINITY;
    let mut best_lag_isqs = None;
    for &dlag in &[-128i32, -64, 0, 64, 128] {
        let lag = lag_baseband_init + dlag;
        let isqs = super::demod::tone_amplitudes(idat, qdat, f0_baseband_init, lag, drift_hz);
        let sync = super::demod::sync_score_isqs(&isqs);
        if sync > best_lag_sync {
            best_lag_sync = sync;
            best_lag = lag;
            best_lag_isqs = Some(isqs);
        }
    }

    // Mode 1: freq refine at the best lag. wsprd uses fstep=0.25 Hz
    // ± 2 → 5 freqs; we use ±1.0 Hz at 0.5 Hz step (slightly wider
    // since coarse's freq grid is 0.73 Hz/bin). 4 new evals (+1 reuse
    // at df=0 from mode 0).
    let mut best_freq = f0_baseband_init;
    let mut best_freq_sync = best_lag_sync;
    let mut best_isqs = best_lag_isqs.expect("at least one lag eval succeeded");
    for &df in &[-1.0f32, -0.5, 0.5, 1.0] {
        let f = f0_baseband_init + df;
        let isqs = super::demod::tone_amplitudes(idat, qdat, f, best_lag, drift_hz);
        let sync = super::demod::sync_score_isqs(&isqs);
        if sync > best_freq_sync {
            best_freq_sync = sync;
            best_freq = f;
            best_isqs = isqs;
        }
    }

    // Mode 2: bit metrics + Fano at the refined alignment. Try each
    // requested nblock value (caller controls; pass 1 = [1], pass 2 =
    // [1, 2, 3] for coherent-block gain). IsQs is reused across
    // nblocks — only `nblock_bit_metrics` (cheap, no oscillator
    // build) runs per variant.
    let mut best_type1: Option<(u32, WsprResult)> = None;
    let mut best_other: Option<(u32, WsprResult)> = None;
    for &nblock in nblocks {
        let bm = super::demod::nblock_bit_metrics(&best_isqs, nblock);
        let mut llrs = bm;
        deinterleave_llrs(&mut llrs);
        // Fano first; if it fails to converge, fall back to OSD-1
        // (Ordered-Statistics Decoding, port of `osdwspr.f90`). OSD
        // can recover signals at -27 dB SNR (e.g. W3BI on the WSJT-X
        // golden) where Fano alone hits the convergence threshold.
        let (info_bits, hard_errors) =
            if let Some(fec_res) = codec.decode_soft(&llrs, &FecOpts::default()) {
                let mut info = [0u8; 50];
                info.copy_from_slice(&fec_res.info);
                (info, fec_res.hard_errors)
            } else if let Some((info, nhardmin)) = super::osd::osd_decode(&llrs) {
                // OSD-2 will synthesise a valid codeword for *any* input;
                // gate by:
                //   1. `nhardmin ≤ 44` — at -27 dB the real signal lands
                //      around 35-40 hard errors after pass-2 subtract;
                //      pure noise produces ≥ 50.
                //   2. Reject Type-3 (hashed-callsign) messages — they're
                //      the dominant phantom class because the 13-bit hash
                //      space produces a nominally valid message for ~15 %
                //      of all 50-bit info vectors. We have no hash table
                //      so any Type-3 that pops out of OSD is overwhelmingly
                //      likely to be garbage.
                const OSD_HARD_ERR_MAX: u32 = 44;
                if nhardmin > OSD_HARD_ERR_MAX {
                    continue;
                }
                let Some(msg) = crate::msg::Wspr50Message
                    .unpack(&info, &crate::engine::DecodeContext::default())
                else {
                    continue;
                };
                if matches!(msg, crate::msg::WsprMessage::Type3 { .. }) {
                    continue;
                }
                (info, nhardmin)
            } else {
                continue;
            };
        let Some(message) =
            crate::msg::Wspr50Message.unpack(&info_bits, &crate::engine::DecodeContext::default())
        else {
            continue;
        };
        let lag_audio = best_lag * 32;
        let dt_sec = lag_audio as f32 / sample_rate as f32 - 1.0;
        let candidate = WsprResult {
            message: message.clone(),
            freq_hz: best_freq + super::baseband::CENTER_HZ - 1.5 * super::demod::TONE_SPACING_HZ,
            start_sample: lag_audio.max(0) as usize,
            dt_sec,
            info_bits,
            snr_db: 0.0,
        };
        let he = hard_errors;
        match message {
            crate::msg::WsprMessage::Type1 { .. } | crate::msg::WsprMessage::Type2 { .. } => {
                if best_type1.as_ref().is_none_or(|(b, _)| he < *b) {
                    best_type1 = Some((he, candidate));
                }
            }
            crate::msg::WsprMessage::Type3 { .. } => {
                if best_other.as_ref().is_none_or(|(b, _)| he < *b) {
                    best_other = Some((he, candidate));
                }
            }
        }
    }
    best_type1.map(|(_, d)| d).or(best_other.map(|(_, d)| d))
}

pub fn decode_at_with_drift(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
) -> Option<WsprResult> {
    let (idat, qdat) = super::baseband::decimate_to_baseband(audio);
    decode_at_baseband(&idat, &qdat, sample_rate, start_sample, freq_hz, drift_hz)
}

/// Scan an audio buffer for any number of WSPR frames, returning all
/// successful decodes. Runs a coarse (freq, time) search with the given
/// [`SearchParams`], then attempts [`decode_at`] on each candidate in
/// score-descending order. Duplicate decodes (same message within ±5 Hz
/// and ±1 symbol) are collapsed to the single earliest-candidate hit,
/// so each transmission appears at most once in the output.
/// Half-window (in seconds) of front-side zero padding added before
/// the search runs. WSPR transmissions can start up to ~2 s **before**
/// the nominal slot anchor (wsprd reports such cases as `dt < -1.0`);
/// the missing pre-roll samples are not in the recording, but with
/// front padding the demodulator still aligns the rest of the frame
/// and Fano can recover from ~1–2 missing leading symbols. Mirrors
/// wsprd's `wspr_decode.f90` which prepends a configurable buffer
/// for the same reason.
const NEGATIVE_DT_PAD_SEC: f32 = 3.0;

pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprResult> {
    // Prepend zeros so signals that started before audio[0] (negative
    // dt) become reachable. Internal `start_sample`s are shifted by
    // `pad`; we subtract `pad` back out before returning so callers
    // see the original time base.
    let pad = (NEGATIVE_DT_PAD_SEC * sample_rate as f32) as usize;
    let mut padded = alloc::vec![0f32; pad + audio.len()];
    padded[pad..].copy_from_slice(audio);
    let nominal_shifted = nominal_start_sample + pad;
    // Decimate ONCE up-front; the wsprd-equivalent coarse and the
    // demod both consume the same baseband buffer, so we save 32×
    // FFT work vs running each separately.
    let (idat, qdat) = super::baseband::decimate_to_baseband(&padded);
    // wsprd-equivalent coarse: 512-pt windowed FFT on the 375 Hz
    // baseband, time-averaged spectrum + 30 th-percentile noise
    // floor, peak detection on smspec, 3-D (freq, time, drift)
    // refinement. Lifts the coarse score landscape to actually peak
    // at the right (freq, dt) for weak signals next to strong ones
    // (W5BIT, W3BI). See `coarse_baseband.rs`.
    let max_drift = 4i32;
    let bb_cands = super::coarse_baseband::coarse_baseband(
        &idat,
        &qdat,
        pad,
        params.max_candidates,
        max_drift,
    );
    // Use the wsprd-equivalent coarse only. Legacy `coarse_search`
    // (12 kHz spectrogram) costs ~30 s of recall-test runtime on a
    // 122 s slot and brings nothing the new coarse doesn't already
    // find — every golden hit on the WSJT-X reference WAV (ND6P,
    // WD4LHT, NM7J, KI7CI, DJ6OL, W3HH, W5BIT) comes through the
    // baseband path. Kept the legacy module for synth round-trip
    // tests; not invoked here.
    let _ = nominal_shifted;
    // `coarse_baseband` already returns candidates sorted by sync score
    // descending — no remap/re-sort needed; keeping `BasebandCandidate`
    // itself (rather than the legacy `SyncCandidate`) preserves each
    // candidate's `snr_db` through to the decode below.
    let mut cands = bb_cands;
    cands.truncate(params.max_candidates);
    let _audio = &padded[..]; // shadow so all downstream reads use padded buffer
    let mut seen: Vec<WsprResult> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192; // one WSPR symbol at 12 kHz
    // 2-D refinement: WSPR's Fano (K=32 convolutional, no CRC) is
    // sensitive to *both* sub-bin freq and sub-t_step time mis-
    // alignment. Coarse-search rounds to 1.46 Hz / 170 ms; this
    // pass refines:
    //   time : ±170 ms / 43 ms step ⇒ 9 points
    //   freq : ±2 Hz   / 0.5 Hz step ⇒ 9 points
    // ≈ 81 sync_score evals × candidate.
    //
    // Going finer in time (e.g. 10 ms) actually *hurts* recall on
    // weak signals: WSPR has no CRC, so the highest-sync_score
    // alignment is often a noise-pattern Fano ghost rather than the
    // true signal. 43 ms preserves true peaks while the coarser
    // grid keeps us out of the ghost-attractor region. Better
    // long-term fix is a Fano-metric / callsign-sanity gate; until
    // then, 43 ms is the empirical optimum on the WSJT-X golden.
    // Tightened grid (3 × 3 = 9 evals/cand, was 9 × 9 = 81): wsprd's
    // entire 3-pass SIC runs in seconds; our refine cost dominated
    // total wall time. The small grid still recovers the same 5/8
    // goldens against the WSJT-X reference WAV — most of the recall
    // win came from sub-bin demod (slice 1 of issue #17), not from
    // the dense refine grid.
    const REFINE_FREQ_RADIUS_HZ: f32 = 1.0;
    const REFINE_FREQ_STEP_HZ: f32 = 1.0;
    let nsps = (sample_rate as f32 * <super::Wspr as crate::engine::ModulationParams>::SYMBOL_DT)
        .round() as i64;
    let refine_time_radius = nsps / 8; // ≈85 ms half-window
    let refine_time_step = nsps / 8; // 1 step at radius → 3 cells in time
    // (idat, qdat) computed above and shared with coarse_baseband.
    // Suppress the unused-warnings for refine_align knobs — they
    // belong to the legacy 12 kHz path that the new baseband
    // demod has retired (decode_at_baseband does its own ±1.5 Hz /
    // ±0.1 s sweep around the coarse pick).
    let _ = (
        REFINE_FREQ_RADIUS_HZ,
        REFINE_FREQ_STEP_HZ,
        refine_time_radius,
        refine_time_step,
    );
    // pass-1 decodes carry their padded-buffer alignment so we can
    // subtract them from the baseband for pass 2.
    let mut pass1: Vec<(WsprResult, usize)> = Vec::new();
    for c in &cands {
        let Some(mut d) =
            decode_at_baseband(&idat, &qdat, sample_rate, c.start_sample, c.freq_hz, 0.0)
        else {
            continue;
        };
        let start_refined = d.start_sample;
        d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
        d.start_sample = start_refined.saturating_sub(pad);
        d.snr_db = c.snr_db;
        let dup = seen.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= TIME_DEDUP_SAMPLES
        });
        if !dup {
            pass1.push((d.clone(), start_refined));
            seen.push(d);
        }
    }

    // Pass 2 — wsprd's 3rd pass equivalent. Subtract every pass-1
    // decode from the baseband (port of `subtract_signal2`,
    // `wsprd.c:541-705`), then re-run the coarse search on the cleaned
    // residual. This exposes signals that were buried under stronger
    // neighbours' noise floor — the only path that recovers W3BI on
    // the WSJT-X golden (-27 dB SNR, hidden by ND6P / KI7CI / etc.).
    if !pass1.is_empty() {
        let mut idat2 = idat.clone();
        let mut qdat2 = qdat.clone();
        for (d, start_refined) in &pass1 {
            let symbols = super::encode_channel_symbols(&d.info_bits);
            let f0_audio = d.freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
            let shift_baseband = (*start_refined as i32) / 32;
            super::subtract::subtract_signal_baseband(
                &mut idat2,
                &mut qdat2,
                f0_audio,
                shift_baseband,
                0.0,
                &symbols,
            );
        }
        // Re-run coarse on the cleaned baseband. Skip the legacy
        // 12 kHz coarse here — pass 2 runs against an already-decimated
        // residual buffer, and reconstructing 12 kHz from baseband is
        // pointless for the same coarse_search call.
        let bb_cands2 = super::coarse_baseband::coarse_baseband(
            &idat2,
            &qdat2,
            pad,
            params.max_candidates,
            max_drift,
        );
        for c in bb_cands2 {
            // Pass 2 uses nblock = 1, 2, 3 (coherent block detection)
            // for the +3..+4.8 dB margin needed to decode signals like
            // W3BI at -27 dB SNR. The strong-signal subtract above has
            // exposed them in the spectrum, but they still need the
            // coherent gain to clear the Fano convergence threshold.
            let Some(mut d) = decode_at_baseband_nblocks(
                &idat2,
                &qdat2,
                sample_rate,
                c.start_sample,
                c.freq_hz,
                c.drift_hz,
                &[1, 2, 3],
            ) else {
                continue;
            };
            let start_refined = d.start_sample;
            d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
            d.start_sample = start_refined.saturating_sub(pad);
            d.snr_db = c.snr_db;
            let dup = seen.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
            if !dup {
                seen.push(d);
            }
        }
    }

    seen
}

/// Convenience: scan using [`SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<WsprResult> {
    decode_scan(audio, sample_rate, 0, &SearchParams::default())
}

/// WSPR subtract configuration (continuous-phase 4-FSK). Mirrors WSJT-X
/// `subtract_signal2` in `wsprd.c`: tone spacing 1.4648 Hz, 8192
/// samples/symbol at 12 kHz, no GFSK shaping (WSPR is plain CPFSK).
const WSPR_SUBTRACT: crate::engine::dsp::subtract::SubtractCfg =
    crate::engine::dsp::subtract::SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 1.4648,
        samples_per_symbol: 8192,
        // WSPR's nominal symbol-0 start is 1.0 s into the slot; our
        // `start_sample` is already absolute, so the subtract layer
        // sees `dt_sec` as `(start - 1.0*FS) / FS`. `base_offset_s = 1.0`
        // matches the convention used by `WsprResult::dt_sec`.
        base_offset_s: 1.0,
        gfsk: None,
    };

/// LPF kernel half-width for the channel-aware subtract (currently
/// unused — see comment in [`decode_scan_subtract`] for why we use
/// the constant-amplitude path instead).
#[allow(dead_code)]
const WSPR_SUBTRACT_LPF_HALF: usize = 600;

/// Multi-pass WSPR decode with successive interference cancellation.
///
/// Mirrors WSJT-X `wsprd.c` `npasses=3` SIC loop (`wsprd.c:998-1438`):
/// each pass runs `decode_scan` on the current residual audio, decodes
/// every signal that survives Fano + (eventually) callsign sanity,
/// reconstructs the on-air channel-symbol sequence via
/// [`super::encode_channel_symbols`], and subtracts a channel-aware
/// LPF reference from the audio so subsequent passes can expose
/// previously-masked weak signals.
///
/// Returns deduplicated decodes from all passes.
pub fn decode_scan_subtract(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprResult> {
    use crate::engine::dsp::subtract::subtract_tones;

    // The subtract helper takes `&mut [i16]`; convert once, mutate
    // across passes, work on `f32` for `decode_scan` per pass.
    let mut residual_i16: Vec<i16> = audio
        .iter()
        .map(|&x| (x * 32767.0).clamp(-32768.0, 32767.0) as i16)
        .collect();

    let mut all: Vec<WsprResult> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192;
    // wsprd uses 3 passes. Our `decode_scan` is expensive (~30 s on
    // a 120-s WSPR slot due to the 2-D refine grid), so we cap at 2
    // — empirically the bulk of the SIC benefit lands on pass 2 once
    // the strong KB0VHA-class signals have been removed.
    const NPASSES: usize = 2;

    for _pass in 0..NPASSES {
        // Re-convert residual back to f32 for decode_scan (it expects
        // unit-scale samples).
        let residual_f32: Vec<f32> = residual_i16.iter().map(|&s| s as f32 / 32_768.0).collect();
        let new_decodes = decode_scan(&residual_f32, sample_rate, nominal_start_sample, params);
        if new_decodes.is_empty() {
            break;
        }
        let mut added = 0usize;
        for d in new_decodes {
            let dup = all.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
            if dup {
                continue;
            }
            // Reconstruct the on-air channel symbols (162 4-FSK tones)
            // from the recovered 50-bit info, and subtract from the
            // residual at the decoded (freq, dt). Mirrors wsprd.c:1432-
            // 1437 `subtract_signal2(idat, qdat, …, channel_symbols)`.
            let symbols = super::encode_channel_symbols(&d.info_bits);
            // Constant-amplitude LS subtract; ignores QSB / drift but
            // is O(N) and avoids the multi-second convolution that
            // `subtract_tones_lpf` (direct conv, kernel ~600 samples)
            // would cost on a 1.4 M-sample WSPR slot. WSJT-X uses the
            // LPF version on a decimated signal — we'll match once we
            // have FFT-conv or a decimate-process-interpolate path.
            subtract_tones(
                &mut residual_i16,
                &symbols,
                d.freq_hz,
                d.dt_sec,
                1.0,
                &WSPR_SUBTRACT,
            );
            all.push(d);
            added += 1;
        }
        if added == 0 {
            break;
        }
    }
    all
}

/// Deinterleave 162 LLRs in place (same permutation as [`deinterleave`]
/// but for `f32` values).
fn deinterleave_llrs(llrs: &mut [f32; 162]) {
    let mut tmp = [0f32; 162];
    let mut p = 0u8;
    let mut i = 0u8;
    while p < 162 {
        // Inline the bit-reverse-8 to avoid exposing a pub helper.
        let i64 = i as u64;
        let j = ((((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64))
            >> 32) as u8 as usize;
        if j < 162 {
            tmp[p as usize] = llrs[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *llrs = tmp;
}

#[cfg(test)]
mod tests {
    use super::super::search::SearchParams;
    use super::super::synthesize_type1;
    use super::*;
    use crate::msg::WsprMessage;

    #[test]
    fn synth_decode_roundtrip_k1abc_fn42_37() {
        let freq = 1500.0;
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("valid message");
        let r = decode_at(&audio, 12_000, 0, freq).expect("decode");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn scan_recovers_message_without_freq_hint() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let decodes = decode_scan(
            &audio,
            12_000,
            0,
            &SearchParams {
                freq_min_hz: 1450.0,
                freq_max_hz: 1550.0,
                ..SearchParams::default()
            },
        );
        assert!(!decodes.is_empty(), "at least one decode");
        let d = decodes.into_iter().next().unwrap();
        assert_eq!(
            d.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
        assert!((d.freq_hz - 1500.0).abs() <= 2.0);
    }

    #[test]
    fn survives_moderate_awgn() {
        use std::f32::consts::PI;

        let freq = 1500.0;
        let mut audio =
            synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.5).expect("valid message");

        // Deterministic "noise": superposition of a handful of off-tone
        // sinusoids plus a pseudorandom dither. This is a cheap AWGN
        // stand-in that keeps the test free of rand dependencies.
        let mut seed: u32 = 0x1234_5678;
        for (i, s) in audio.iter_mut().enumerate() {
            // Linear congruential pseudorandom for reproducible noise.
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let rnd = ((seed >> 16) as f32 / 32768.0 - 1.0) * 0.10;
            let off = 0.05 * (2.0 * PI * 2345.7 * i as f32 / 12_000.0).sin();
            *s += rnd + off;
        }

        let r = decode_at(&audio, 12_000, 0, freq).expect("decode under noise");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K9AN".into(),
                grid: "EN50".into(),
                power_dbm: 33,
            }
        );
    }
}
