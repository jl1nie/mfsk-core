//! JT9 receiver: audio → per-symbol 9-tone magnitudes → 206 bit LLRs.
//!
//! Geometry: one-symbol FFT of NSPS = 6912 samples at 12 kHz gives
//! bin width 12 000 / 6912 ≈ 1.7361 Hz, exactly one JT9 tone spacing.
//! So each of the 9 tones lands on its own bin without leakage.
//!
//! Stages:
//! 1. Per-symbol FFT (85 total), extract 9 tone magnitudes at
//!    `base_bin + 0..=8`.
//! 2. Skip the 16 sync positions (they don't carry data).
//! 3. For each of the 69 data symbols, convert the 8 data-tone
//!    magnitudes to 3 bit LLRs using a max-log-MAP approximation,
//!    accounting for the Gray map so Fano receives **pre-Gray** bits.
//! 4. Concatenate → 207 bits (the 207th is padding), drop the last
//!    bit, and run the 206-bit de-interleaver.
//!
//! The output is 206 bit LLRs suitable for
//! `crate::fec::ConvFano232::decode_soft`.

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::interleave::deinterleave_llrs;
use super::sync_pattern::JT9_ISYNC;
use super::Jt9;

/// Inverse Gray code on 3-bit values.
#[inline]
fn inv_gray3(g: u8) -> u8 {
    let mut n = g & 0x7;
    n ^= n >> 1;
    n ^= n >> 2;
    n & 0x7
}

/// LLR clamp, mirroring WSPR's `mags_to_llrs`. Keeps integer-metric
/// Fano decoder in range.
const LLR_CLAMP: f32 = 20.0;

/// Demodulate 85 channel symbols from aligned audio and produce 206
/// deinterleaved bit LLRs ready for
/// [`ConvFano232::decode_soft`](crate::fec::ConvFano232).
///
/// Convention: positive LLR ⇒ bit 0 is more likely.
pub fn demodulate_aligned(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> [f32; 206] {
    let nsps = (sample_rate as f32 * <Jt9 as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32; // = TONE_SPACING_HZ by construction
    let base_bin = (base_freq_hz / df).round() as usize;

    // Guard — if the caller asked for a window that doesn't fit, return
    // zero LLRs (decode will fail gracefully via Fano non-convergence).
    if start_sample + 85 * nsps > audio.len() || base_bin + 9 >= nsps / 2 {
        return [0f32; 206];
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];

    // Accumulate the 69 data-symbol LLR triples plus a noise reference.
    let mut llrs207 = [0f32; 207];
    let mut noise_acc = 0.0f32;
    let mut noise_count = 0u32;
    let mut j = 0; // data-symbol index within the 69 data slots

    for sym_idx in 0..85 {
        let sym_start = start_sample + sym_idx * nsps;
        for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
            *slot = Complex::new(s, 0.0);
        }
        fft.process_with_scratch(&mut buf, &mut scratch);

        // Noise reference from bins just above the 9-tone passband.
        for k in 9..14 {
            let bin = base_bin + k;
            if bin < nsps / 2 {
                noise_acc += buf[bin].norm_sqr();
                noise_count += 1;
            }
        }

        if JT9_ISYNC[sym_idx] == 1 {
            continue; // sync symbol, not a data carrier
        }

        // Eight data-tone magnitudes (tones 1..=8 in the tone index).
        let mut mags = [0f32; 8];
        for t in 0..8 {
            mags[t] = buf[base_bin + 1 + t].norm();
        }

        // Max-log-MAP bit LLRs. For each of 3 bits, the LLR is
        // max |a|² over tones where bit == 0  —  max |a|² over bit == 1.
        // Tone index post-Gray is 0..=7; the pre-Gray 3-bit payload
        // is `inv_gray3(tone_index)`. Bit order: MSB first (to match
        // the TX `packbits(...,3,...)` layout).
        let mut llr3 = [0f32; 3];
        for bit_pos in 0..3 {
            let mask = 1u8 << (2 - bit_pos); // MSB first
            let mut max0 = f32::NEG_INFINITY;
            let mut max1 = f32::NEG_INFINITY;
            for tone in 0u8..8 {
                let data_bits = inv_gray3(tone);
                let p = mags[tone as usize] * mags[tone as usize];
                if data_bits & mask == 0 {
                    if p > max0 { max0 = p; }
                } else {
                    if p > max1 { max1 = p; }
                }
            }
            llr3[bit_pos] = max0 - max1; // normalised below
        }

        // Place at indices 3j..3j+3 in the 207-bit frame.
        llrs207[3 * j]     = llr3[0];
        llrs207[3 * j + 1] = llr3[1];
        llrs207[3 * j + 2] = llr3[2];
        j += 1;
    }
    debug_assert_eq!(j, 69);

    // Noise-normalise + clamp to keep Fano's i32 metric table in range.
    let noise_var = if noise_count > 0 {
        (noise_acc / noise_count as f32).max(1e-6)
    } else {
        1.0
    };
    // Match WSPR's scale: divide by noise σ² and clamp to ±20.
    let mut out = [0f32; 206];
    for i in 0..206 {
        let raw = llrs207[i] / noise_var;
        out[i] = raw.clamp(-LLR_CLAMP, LLR_CLAMP);
    }
    // llrs207[206] is padding — discarded.

    // De-interleave so bits come out in the encoder's original order.
    deinterleave_llrs(&mut out);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::tx::synthesize_standard;
    use crate::core::{DecodeContext, FecOpts, MessageCodec};
    use crate::fec::{ConvFano232, FecCodec};
    use crate::msg::{Jt72Codec, Jt72Message};

    #[test]
    fn inv_gray_roundtrip() {
        for n in 0u8..8 {
            let g = n ^ (n >> 1);
            assert_eq!(inv_gray3(g), n, "n={n} → gray={g} → inv={}", inv_gray3(g));
        }
    }

    #[test]
    fn synth_decode_roundtrip_cq_k1abc_fn42() {
        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3)
            .expect("pack+synth");
        let llrs = demodulate_aligned(&audio, 12_000, 0, freq);

        let codec = ConvFano232;
        let res = codec
            .decode_soft(&llrs, &FecOpts::default())
            .expect("Fano must converge on clean synth");
        assert_eq!(res.info.len(), 72);

        // Pack the 72 bits back into 12 × 6-bit words and unpack.
        let mut payload = [0u8; 72];
        payload.copy_from_slice(&res.info);
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
