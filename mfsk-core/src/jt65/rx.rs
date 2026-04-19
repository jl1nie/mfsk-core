//! JT65 receiver: audio → 63 hard-decision RS symbols → message.
//!
//! JT65 demodulation is hard-decision (unlike FT8/FT4/FST4/WSPR's
//! bit-LLR path): for each of the 63 data positions we run a
//! symbol-length FFT and take the argmax across the 64 data-tone
//! bins. The resulting symbols are de-Gray'd, de-interleaved, and
//! fed straight to [`crate::fec::Rs63_12::decode_jt65`].
//!
//! Geometry: NSPS = 4460 samples at 12 kHz gives bin width ≈
//! 2.6906 Hz = one JT65A tone spacing.

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::gray::inv_gray6;
use super::interleave::deinterleave;
use super::sync_pattern::JT65_NPRC;
use super::Jt65;

/// Demodulate 63 data symbols from aligned audio. Returns the 63
/// hard-decision symbols in **RS codeword order** (Gray-decoded and
/// de-interleaved), ready for [`Rs63_12::decode_jt65`].
pub fn demodulate_aligned(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<[u8; 63]> {
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32; // ≡ TONE_SPACING_HZ
    let base_bin = (base_freq_hz / df).round() as usize;

    // Sanity bounds.
    if start_sample + 126 * nsps > audio.len() || base_bin + 66 >= nsps / 2 {
        return None;
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];

    let (syms, _conf) = demodulate_aligned_with_confidence_inner(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        nsps,
        base_bin,
        &mut buf,
        &mut scratch,
        &*fft,
    )?;
    Some(syms)
}

/// Demodulate 63 data symbols AND return per-symbol confidence:
/// `(best_power - second_best_power) / best_power`. Confidence is in
/// `[0, 1]`; 1 means the winning tone dominates, 0 means the top two
/// tones are tied (coin-flip).
///
/// Returned in RS codeword order — already Gray-decoded and
/// de-interleaved, ready for `Rs63_12::decode_jt65_erasures`.
pub fn demodulate_aligned_with_confidence(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<([u8; 63], [f32; 63])> {
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let base_bin = (base_freq_hz / df).round() as usize;
    if start_sample + 126 * nsps > audio.len() || base_bin + 66 >= nsps / 2 {
        return None;
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];
    demodulate_aligned_with_confidence_inner(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        nsps,
        base_bin,
        &mut buf,
        &mut scratch,
        &*fft,
    )
}

fn demodulate_aligned_with_confidence_inner(
    audio: &[f32],
    _sample_rate: u32,
    start_sample: usize,
    _base_freq_hz: f32,
    nsps: usize,
    base_bin: usize,
    buf: &mut Vec<Complex<f32>>,
    scratch: &mut Vec<Complex<f32>>,
    fft: &dyn rustfft::Fft<f32>,
) -> Option<([u8; 63], [f32; 63])> {
    // Walk 126 symbol windows. Data positions (NPRC[i] == 0) each get
    // argmax of 64 data-tone magnitudes (+ runner-up for confidence).
    let mut symbols = [0u8; 63];
    let mut conf = [0f32; 63];
    let mut k = 0usize;
    for sym_idx in 0..126 {
        let sym_start = start_sample + sym_idx * nsps;
        for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
            *slot = Complex::new(s, 0.0);
        }
        fft.process_with_scratch(buf, scratch);
        if JT65_NPRC[sym_idx] == 1 {
            continue;
        }
        let mut best_tone = 0u8;
        let mut best_pwr = f32::NEG_INFINITY;
        let mut second_pwr = f32::NEG_INFINITY;
        for tone in 0u8..64 {
            let bin = base_bin + 2 + tone as usize;
            let p = buf[bin].norm_sqr();
            if p > best_pwr {
                second_pwr = best_pwr;
                best_pwr = p;
                best_tone = tone;
            } else if p > second_pwr {
                second_pwr = p;
            }
        }
        symbols[k] = inv_gray6(best_tone);
        conf[k] = if best_pwr > 0.0 {
            ((best_pwr - second_pwr.max(0.0)) / best_pwr).clamp(0.0, 1.0)
        } else {
            0.0
        };
        k += 1;
    }
    debug_assert_eq!(k, 63);
    deinterleave(&mut symbols);
    // Apply the same permutation to confidence so positions line up.
    let mut conf_perm = [0f32; 63];
    {
        // Re-run the 7×9 transpose on the confidence array with the
        // same pattern `deinterleave` uses. Since deinterleave is
        // i_native = j*7 + i_inner, j*9+i_inner mapping, reapply:
        for i in 0..7 {
            for j in 0..9 {
                conf_perm[i * 9 + j] = conf[j * 7 + i];
            }
        }
    }
    Some((symbols, conf_perm))
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::tx::synthesize_standard;
    use crate::core::{DecodeContext, MessageCodec};
    use crate::fec::Rs63_12;
    use crate::msg::{Jt72Codec, Jt72Message};

    #[test]
    fn synth_decode_roundtrip_cq_k1abc_fn42() {
        let freq = 1270.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3)
            .expect("pack+synth");
        let received = demodulate_aligned(&audio, 12_000, 0, freq).expect("demod");
        let rs = Rs63_12::new();
        let (info, nerr) = rs.decode_jt65(&received).expect("clean decode");
        assert_eq!(nerr, 0, "clean synth should have zero errors");

        // Pack 12 × 6-bit words into 72 MSB-first bits, then unpack
        // via Jt72 codec.
        let mut payload = [0u8; 72];
        for (i, bit) in payload.iter_mut().enumerate() {
            let word = info[i / 6];
            let shift = 5 - (i % 6);
            *bit = (word >> shift) & 1;
        }
        let msg = Jt72Codec::default()
            .unpack(&payload, &DecodeContext::default())
            .expect("unpack");
        match msg {
            Jt72Message::Standard { call1, call2, grid_or_report } => {
                assert_eq!(call1, "CQ");
                assert_eq!(call2, "K1ABC");
                assert_eq!(grid_or_report, "FN42");
            }
            other => panic!("expected Standard, got {:?}", other),
        }
    }
}
