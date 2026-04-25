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
//!    GF(64) via [`QraCode::mfsk_bessel_metric`].
//! 4. Run [`Q65Codec::decode`] (BP + CRC verify) — recover the 13
//!    info symbols.
//! 5. Re-pack to 77 bits and unpack via the Wsjt77 message codec.
//!
//! AP / fast-fading paths from the C reference are not yet ported;
//! this module covers the AWGN path used by every Q65 decode before
//! AP refinement kicks in.

use num_complex::Complex;
use rustfft::FftPlanner;

use crate::core::ModulationParams;
use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::ApHint;
use crate::msg::q65::{ap_hint_to_q65_mask, unpack_symbols_to_bits77};

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
        for tone in 0..64 {
            let bin = base_bin + (tone + 1) * bins_per_tone;
            row[tone] = buf[bin].norm_sqr();
        }
        k += 1;
    }
    debug_assert_eq!(k, 63);
    Some(energies)
}

/// One successful Q65 decode with its alignment metadata.
#[derive(Clone, Debug)]
pub struct Q65Decode {
    /// Decoded human-readable Wsjt77 message.
    pub message: String,
    /// Tone-0 frequency in Hz.
    pub freq_hz: f32,
    /// Sample index where the frame's symbol 0 begins.
    pub start_sample: usize,
    /// BP iterations consumed by the QRA decoder.
    pub iterations: u32,
}

/// Decode a Q65 signal at a known `(start_sample, base_freq_hz)`
/// for sub-mode `P`.
///
/// Performs FFT-per-symbol, builds intrinsic probability distributions
/// via the Bessel metric, runs QRA belief propagation, verifies the
/// CRC-12, and unpacks the recovered 77-bit Wsjt77 message. Returns
/// `None` if the buffer is too short, BP fails to converge, the CRC
/// rejects the result, or the unpack fails.
pub fn decode_at_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Q65Decode> {
    decode_at_inner::<P>(audio, sample_rate, start_sample, base_freq_hz, None)
}

/// Like [`decode_at_for`] but biases the QRA decoder with an AP
/// hint — typically a known callsign pair or "CQ" expectation.
///
/// Empirically gains 2–4 dB at threshold for Q65-30A and is the
/// dominant mechanism that makes 6 m / 70 cm EME workable. The
/// hint is converted to the Q65-specific 13-symbol GF(64) mask via
/// [`ap_hint_to_q65_mask`] and applied to the depunctured intrinsics
/// before BP.
pub fn decode_at_with_ap_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ap_hint: &ApHint,
) -> Option<Q65Decode> {
    decode_at_inner::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        Some(ap_hint),
    )
}

fn decode_at_inner<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ap_hint: Option<&ApHint>,
) -> Option<Q65Decode> {
    use crate::core::{DecodeContext, MessageCodec};
    use crate::msg::Q65Message;

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
    let text = Q65Message.unpack(&bits77, &DecodeContext::default())?;

    Some(Q65Decode {
        message: text,
        freq_hz: base_freq_hz,
        start_sample,
        iterations,
    })
}

/// Q65-30A convenience wrapper for [`decode_at_for`].
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Q65Decode> {
    decode_at_for::<Q65a30>(audio, sample_rate, start_sample, base_freq_hz)
}

/// Q65-30A convenience wrapper for [`decode_at_with_ap_for`].
pub fn decode_at_with_ap(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ap_hint: &ApHint,
) -> Option<Q65Decode> {
    decode_at_with_ap_for::<Q65a30>(audio, sample_rate, start_sample, base_freq_hz, ap_hint)
}

/// Scan an audio buffer for Q65 frames in sub-mode `P` within the
/// search window: runs [`super::search::coarse_search_for`] and tries
/// [`decode_at_for`] on each candidate in score order, collapsing
/// duplicate decodes (same message, frequency within ±4 Hz, start
/// sample within ±1 symbol).
pub fn decode_scan_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
) -> Vec<Q65Decode> {
    decode_scan_inner::<P>(audio, sample_rate, nominal_start_sample, params, None)
}

/// AP-biased version of [`decode_scan_for`]. Same coarse search; each
/// candidate is decoded with the AP hint applied, which lifts the
/// effective decode threshold by 2–4 dB on Q65-30A and is essential
/// for EME on 6 m and above.
pub fn decode_scan_with_ap_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    ap_hint: &ApHint,
) -> Vec<Q65Decode> {
    decode_scan_inner::<P>(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Some(ap_hint),
    )
}

fn decode_scan_inner<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    ap_hint: Option<&ApHint>,
) -> Vec<Q65Decode> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let cands =
        super::search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Q65Decode> = Vec::new();
    for c in cands {
        let decode = match ap_hint {
            Some(hint) if hint.has_info() => {
                decode_at_with_ap_for::<P>(audio, sample_rate, c.start_sample, c.freq_hz, hint)
            }
            _ => decode_at_for::<P>(audio, sample_rate, c.start_sample, c.freq_hz),
        };
        let Some(decode) = decode else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == decode.message
                && (prev.freq_hz - decode.freq_hz).abs() <= 4.0
                && (prev.start_sample as i64 - decode.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            seen.push(decode);
        }
    }
    seen
}

/// Q65-30A convenience wrapper for [`decode_scan_for`].
pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
) -> Vec<Q65Decode> {
    decode_scan_for::<Q65a30>(audio, sample_rate, nominal_start_sample, params)
}

/// Q65-30A convenience wrapper for [`decode_scan_with_ap_for`].
pub fn decode_scan_with_ap(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &super::search::SearchParams,
    ap_hint: &ApHint,
) -> Vec<Q65Decode> {
    decode_scan_with_ap_for::<Q65a30>(audio, sample_rate, nominal_start_sample, params, ap_hint)
}

/// Convenience: scan for Q65-30A with default search parameters
/// from sample 0.
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Q65Decode> {
    decode_scan(
        audio,
        sample_rate,
        0,
        &super::search::SearchParams::default(),
    )
}

#[cfg(test)]
mod tests {
    use super::super::tx::synthesize_standard;
    use super::*;

    #[test]
    fn aligned_decode_recovers_clean_message() {
        let freq = 1500.0;
        let audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("pack + synth");
        let result = decode_at(&audio, 12_000, 0, freq).expect("clean aligned decode must succeed");
        assert_eq!(result.message, "CQ K1ABC FN42");
        assert_eq!(result.start_sample, 0);
        assert!((result.freq_hz - freq).abs() < 0.001);
    }

    #[test]
    fn scan_recovers_clean_message_without_alignment_hint() {
        let freq = 1500.0;
        let audio =
            synthesize_standard("CQ", "JA1ABC", "PM95", 12_000, freq, 0.3).expect("pack + synth");
        let decodes = decode_scan_default(&audio, 12_000);
        assert!(!decodes.is_empty(), "scan must find a clean signal");
        assert_eq!(decodes[0].message, "CQ JA1ABC PM95");
    }

    #[test]
    fn scan_with_no_signal_returns_empty() {
        // Pure silence (well, low noise) must not produce false decodes.
        let audio = vec![0.0_f32; 12_000 * 30];
        let decodes = decode_scan_default(&audio, 12_000);
        assert!(
            decodes.is_empty(),
            "got false decodes from silence: {decodes:#?}"
        );
    }
}
