//! 3840-pt sc16 mixed-radix FFT — host software port of embedded
//! `embedded-shared::esp_dsp_fft::MixedRadix3840Sc16Fft`.
//!
//! Bit-exact match: same 256×15 Cooley-Tukey decomposition, same
//! inner 256-pt sc16 radix-2 (via [`super::fft_sc16_r2`]), same
//! f32 PFA outer stage, same round+clamp back to i16.
//!
//! Host callers (`RustFftPlanner16` in `engine::fft`) use this for the
//! NFFT_SPEC=3840 case so `compute_spectrogram` (fixed-point feature
//! on host) produces byte-identical output to the embedded path —
//! enables host simulation of embedded recall behaviour.
//!
//! Total gain: inner 256-pt FFT applies `/256` (= per-stage /2 over
//! log2(256)=8 stages); PFA outer stage runs in f32 with no /15
//! division (matches embedded). So net 3840-pt gain = `1/256`,
//! NOT `1/N=1/3840`.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::{Complex, Complex32};
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::fft_15::fft_15;
use super::fft_sc16_r2::{bit_rev_sc16, fft2r_sc16, gen_w_r2_sc16};

const N: usize = 3840;
const N1: usize = 256;
const N2: usize = 15;

/// Precomputed twiddle tables for the 3840-pt sc16 mixed-radix FFT.
/// Build once at planner construction, reuse across calls.
pub struct Plan3840Sc16 {
    /// 256-pt radix-2 twiddles (bit-reversed sc16, length N1/2=128).
    w_inner: Vec<Complex<i16>>,
    /// Outer 256×15 mixed-radix twiddles `ω_3840^(n2·k1)` (f32, length N).
    twiddles_outer: Box<[Complex32; N]>,
}

impl Plan3840Sc16 {
    pub fn new() -> Self {
        Self {
            w_inner: gen_w_r2_sc16(N1),
            twiddles_outer: super::fft_mixed_3840::build_twiddles(),
        }
    }

    /// In-place forward 3840-pt sc16 FFT. Matches
    /// `embedded-shared::esp_dsp_fft::MixedRadix3840Sc16Fft::process`
    /// byte-for-byte (modulo i16 rounding equivalence between
    /// software xtfixed_bf_{1..4} and the Xtensa AE32 asm).
    pub fn process(&self, buf: &mut [Complex<i16>]) {
        assert_eq!(buf.len(), N, "Plan3840Sc16: input must be 3840 samples");

        // ── Step 1+2: reshape to 15 rows × 256 cols (i16), run 256-pt
        //              sc16 FFT on each row (+ bit-reverse).
        let mut rows: Vec<Complex<i16>> = vec![Complex::new(0i16, 0); N];
        for n1 in 0..N1 {
            for n2 in 0..N2 {
                rows[n2 * N1 + n1] = buf[15 * n1 + n2];
            }
        }
        for n2 in 0..N2 {
            let row = &mut rows[n2 * N1..(n2 + 1) * N1];
            fft2r_sc16(row, &self.w_inner);
            bit_rev_sc16(row);
        }

        // ── Step 3+4: convert to f32, twiddle multiply, 15-pt PFA per col.
        let mut m: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); N];
        for (i, c) in rows.iter().enumerate() {
            m[i] = Complex32::new(c.re as f32, c.im as f32) * self.twiddles_outer[i];
        }
        let mut col = [Complex32::new(0.0, 0.0); N2];
        for k1 in 0..N1 {
            for k2 in 0..N2 {
                col[k2] = m[k2 * N1 + k1];
            }
            fft_15(&mut col);
            for k2 in 0..N2 {
                m[k2 * N1 + k1] = col[k2];
            }
        }

        // ── Step 5: clamp to i16 and write back in natural index order.
        for k2 in 0..N2 {
            for k1 in 0..N1 {
                let c = m[k2 * N1 + k1];
                let re = c.re.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
                let im = c.im.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
                buf[N1 * k2 + k1] = Complex::new(re, im);
            }
        }
    }
}

impl Default for Plan3840Sc16 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Single-tone in → single peak at expected bin. Validates the
    /// 3840-pt mixed-radix sc16 FFT picks up FT8-style tones (= integer
    /// bins at the 3.125 Hz / 6.25 Hz tone spacing alignment).
    #[test]
    fn sc16_3840_pure_tone() {
        let plan = Plan3840Sc16::new();
        let k_bin = 40; // matches some hypothetical FT8 tone
        let mut buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0); N];
        // Pre-shifted tone so /256 final gain lands magnitude ~ amp
        // (= 16000 / scale_loss in the i16 range).
        for i in 0..N {
            let phase = core::f32::consts::TAU * (k_bin as f32) * (i as f32) / (N as f32);
            buf[i] = Complex::new((8000.0 * phase.cos()) as i16, (8000.0 * phase.sin()) as i16);
        }
        plan.process(&mut buf);
        // Peak at k_bin (after natural-order output of mixed-radix).
        let peak_mag2 = (buf[k_bin].re as i32).pow(2) + (buf[k_bin].im as i32).pow(2);
        // Expected ~ (8000 * N/256)² = (8000 * 15)² = 120000² ≈ 1.4e10,
        // but clamped to i16 (32767) so mag² ≈ 32767² × 2 ≈ 2.1e9.
        // (Either way it's the largest bin by far; just check it
        // dominates the average.)
        let mut total_mag2: u64 = 0;
        for c in buf.iter() {
            total_mag2 += ((c.re as i32).pow(2) + (c.im as i32).pow(2)) as u64;
        }
        assert!(
            (peak_mag2 as u64) > total_mag2 / 100,
            "peak {peak_mag2} not dominating (sum={total_mag2})"
        );
    }
}
