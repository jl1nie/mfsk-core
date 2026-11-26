// SPDX-License-Identifier: GPL-3.0-only
//! 2560-pt sc16 mixed-radix FFT: FT8's spectrogram FFT at **8 kHz** (`Ft8Rate::Hz8000`: two symbols of 1280 samples, the
//! tone step 2 bins, as 3840 is at 12 kHz).
//!
//! The same shape as [`Plan3840Sc16`](super::fft_mixed_3840_sc16::Plan3840Sc16), so the two share what they can:
//! `2560 = 256 x 10`, the inner stage is the **same 256-point radix-2 sc16 kernel** (and the same twiddle table: the
//! board's one global table is for 256 either way), and the outer stage is f32 (a 10-point DFT per column, no division
//! by 10). The total gain is therefore `1/256` at either length, which is what `FP_SPEC_SHIFT` and the spectrogram's
//! gain were calibrated on; `512 x 5` would have made it `1/512` and re-initialised the table on every size switch.
//!
//! Index maps (Cooley-Tukey, as for 3840): input `n = 10 n1 + n2`, rows `n2` of 256 over `n1`; after the inner FFT
//! (`k1`) and the twiddle `exp(-j 2 pi n2 k1 / 2560)`, a 10-point DFT over `n2` gives `k2`; the output is
//! `X[k1 + 256 k2]`.
//!
//! This is the host port. The board's esp-dsp version (`MixedRadix2560Sc16Fft`) would be its twin; nothing pins the
//! two to the same bits (no test does for 3840 either), only that each is the DFT to a few LSB.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::{Complex, Complex32};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::fft_15::fft_5;
use super::fft_sc16_r2::{bit_rev_sc16, fft2r_sc16, gen_w_r2_sc16};

/// Transform length.
pub const N: usize = 2560;
const N1: usize = 256;
const N2: usize = 10;

/// Forward 10-point complex FFT, in place: `X[k] = sum_n x[n] exp(-j 2 pi n k / 10)`.
///
/// `10 = 2 x 5`: the even and the odd samples each go through [`fft_5`], and `X[k] = E[k mod 5] + w^k O[k mod 5]` with
/// `w = exp(-j 2 pi / 10)`. No division, as every outer step of these plans.
pub fn fft_10(x: &mut [Complex32; 10]) {
    let mut e = [x[0], x[2], x[4], x[6], x[8]];
    let mut o = [x[1], x[3], x[5], x[7], x[9]];
    fft_5(&mut e);
    fft_5(&mut o);
    for k in 0..10 {
        let phi = -core::f32::consts::TAU * k as f32 / 10.0;
        let w = Complex32::new(phi.cos(), phi.sin());
        x[k] = e[k % 5] + w * o[k % 5];
    }
}

/// The inter-stage twiddles: `t[n2 * 256 + k1] = exp(-j 2 pi n2 k1 / 2560)`.
pub fn build_twiddles() -> Box<[Complex32; N]> {
    let mut t = vec![Complex32::new(0.0, 0.0); N].into_boxed_slice();
    for n2 in 0..N2 {
        for k1 in 0..N1 {
            let phi = -core::f32::consts::TAU * (n2 as f32) * (k1 as f32) / (N as f32);
            t[n2 * N1 + k1] = Complex32::new(phi.cos(), phi.sin());
        }
    }
    // Box<[T]> to Box<[T; N]>: same layout, the length is N
    let raw = Box::into_raw(t) as *mut [Complex32; N];
    unsafe { Box::from_raw(raw) }
}

/// Precomputed tables of the 2560-pt sc16 mixed-radix FFT: build once, reuse across calls.
pub struct Plan2560Sc16 {
    /// The 256-point radix-2 twiddles (bit-reversed sc16), the 3840 plan's.
    w_inner: Vec<Complex<i16>>,
    twiddles_outer: Box<[Complex32; N]>,
}

impl core::fmt::Debug for Plan2560Sc16 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Plan2560Sc16").finish_non_exhaustive()
    }
}

impl Plan2560Sc16 {
    pub fn new() -> Self {
        Self {
            w_inner: gen_w_r2_sc16(N1),
            twiddles_outer: build_twiddles(),
        }
    }

    /// In-place forward 2560-pt sc16 FFT, natural order in and out, total gain `1/256`.
    pub fn process(&self, buf: &mut [Complex<i16>]) {
        assert_eq!(buf.len(), N, "Plan2560Sc16: input must be 2560 samples");

        // 10 rows of 256 (i16): the 256-point sc16 FFT on each, with its bit reversal
        let mut rows: Vec<Complex<i16>> = vec![Complex::new(0i16, 0); N];
        for n1 in 0..N1 {
            for n2 in 0..N2 {
                rows[n2 * N1 + n1] = buf[N2 * n1 + n2];
            }
        }
        for n2 in 0..N2 {
            let row = &mut rows[n2 * N1..(n2 + 1) * N1];
            fft2r_sc16(row, &self.w_inner);
            bit_rev_sc16(row);
        }

        // f32: the twiddle, then a 10-point DFT down each column
        let mut m: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); N];
        for (i, c) in rows.iter().enumerate() {
            m[i] = Complex32::new(c.re as f32, c.im as f32) * self.twiddles_outer[i];
        }
        let mut col = [Complex32::new(0.0, 0.0); N2];
        for k1 in 0..N1 {
            for k2 in 0..N2 {
                col[k2] = m[k2 * N1 + k1];
            }
            fft_10(&mut col);
            for k2 in 0..N2 {
                m[k2 * N1 + k1] = col[k2];
            }
        }

        // round and clamp to i16, natural order `k = 256 k2 + k1`
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

impl Default for Plan2560Sc16 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic pseudo-random values in `-1..1`.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        }
    }

    /// `fft_10` is the DFT: against the double-precision sum, on random input. Fails if the 2 x 5 split, the `w^k`
    /// twiddle or the output order is wrong (any of them moves most of the ten outputs by O(1)).
    #[test]
    fn fft_10_is_the_dft() {
        let mut r = Lcg(7);
        for _ in 0..20 {
            let x: Vec<(f64, f64)> = (0..10).map(|_| (r.next(), r.next())).collect();
            let mut y = [Complex32::new(0.0, 0.0); 10];
            for (i, v) in x.iter().enumerate() {
                y[i] = Complex32::new(v.0 as f32, v.1 as f32);
            }
            fft_10(&mut y);
            for k in 0..10 {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (n, v) in x.iter().enumerate() {
                    let a = -core::f64::consts::TAU * (n * k) as f64 / 10.0;
                    re += v.0 * a.cos() - v.1 * a.sin();
                    im += v.0 * a.sin() + v.1 * a.cos();
                }
                assert!(
                    (y[k].re as f64 - re).abs() < 1e-5 && (y[k].im as f64 - im).abs() < 1e-5,
                    "k {k}"
                );
            }
        }
    }

    /// A complex tone at bin `k` lands in bin `k` with magnitude `amp * 2560 / 256` and nowhere else (to the i16
    /// rounding): for `k` in every residue of the index maps (`k1 = k mod 256`, `k2 = k / 256`), the low, the high and
    /// the ends. Fails if the input reshape, the outer twiddle, the 10-point step or the output order is wrong, or the
    /// total gain is not `1/256`.
    #[test]
    fn sc16_2560_pure_tone() {
        let plan = Plan2560Sc16::new();
        for k in [0usize, 1, 40, 255, 256, 257, 777, 1280, 2000, 2303, 2559] {
            let amp = 1000.0f64;
            let mut buf: Vec<Complex<i16>> = (0..N)
                .map(|n| {
                    let ph = core::f64::consts::TAU * ((k * n) % N) as f64 / N as f64;
                    Complex::new(
                        (amp * ph.cos()).round() as i16,
                        (amp * ph.sin()).round() as i16,
                    )
                })
                .collect();
            plan.process(&mut buf);
            let mag = |c: &Complex<i16>| ((c.re as f64).powi(2) + (c.im as f64).powi(2)).sqrt();
            let want = amp * N as f64 / 256.0;
            assert!(
                (mag(&buf[k]) - want).abs() < 0.02 * want,
                "k {k}: {} against {want}",
                mag(&buf[k])
            );
            let worst = buf
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != k)
                .map(|(_, c)| mag(c))
                .fold(0.0, f64::max);
            assert!(
                worst < 0.01 * want,
                "k {k}: the strongest other bin is {worst}"
            );
        }
    }

    /// Every bin of a random input against the double-precision DFT x 1/256, within a few i16 LSB: the inner stage
    /// rounds eight times, the outer one sums ten of them. Fails on any index error in the 2 x 5 outer step or a wrong
    /// twiddle (an O(amp) error in some bins).
    #[test]
    fn sc16_2560_is_the_dft_to_a_few_lsb() {
        let plan = Plan2560Sc16::new();
        let mut r = Lcg(11);
        let x: Vec<(i16, i16)> = (0..N)
            .map(|_| ((r.next() * 800.0) as i16, (r.next() * 800.0) as i16))
            .collect();
        let mut buf: Vec<Complex<i16>> = x.iter().map(|&(a, b)| Complex::new(a, b)).collect();
        plan.process(&mut buf);
        // the reference: X[k] = sum_n x[n] exp(-j 2 pi n k / N) / 256, in f64
        let mut worst = 0.0f64;
        for k in (0..N).step_by(7).chain([N - 1]) {
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (n, &(a, b)) in x.iter().enumerate() {
                let ph = -core::f64::consts::TAU * ((n * k) % N) as f64 / N as f64;
                let (c, s) = (ph.cos(), ph.sin());
                re += a as f64 * c - b as f64 * s;
                im += a as f64 * s + b as f64 * c;
            }
            let (re, im) = (re / 256.0, im / 256.0);
            worst = worst
                .max((buf[k].re as f64 - re).abs())
                .max((buf[k].im as f64 - im).abs());
        }
        assert!(worst < 8.0, "worst error {worst} LSB");
    }
}
