//! Radix-2 i16-complex FFT — software port of esp-dsp's
//! `dsps_fft2r_sc16_ansi` (Apache 2.0, Espressif).
//!
//! Bit-exact match to the embedded asm path:
//! `embedded-shared::esp_dsp_fft::EspDspFftPlanner16` uses
//! `dsps_fft2r_sc16_ae32_` (Xtensa AE32 asm) on hardware; this
//! module provides the **identical numerical behaviour** for host
//! simulation (compute_spectrogram fixed-point) so host
//! `decode_block_into` etc. reproduce S3 wav_sim recall byte-for-byte.
//!
//! The previous host FFT (`RustFftPlanner16` doing a single
//! rustfft 3840-pt + flat `1/N` scaling) is mathematically the same
//! transform but a completely different implementation, with
//! different per-stage i16 rounding error — that mismatch was the
//! root cause of the host vs embedded recall gap (host 3 vs S3
//! wav_sim 7 on qso3_busy.wav, Phase 1.7.7a investigation).
//!
//! ## Algorithm (= `dsps_fft2r_sc16_ansi`)
//!
//! Decimation-in-frequency radix-2 iterative FFT with per-stage
//! `>>1` scaling (total `1/N` gain). Output is in **bit-reversed**
//! order; call [`bit_rev_sc16`] to natural-order.
//!
//! Each butterfly per stage:
//! ```text
//!   a = data[ia]                      // even index
//!   m = data[ia + N2]                 // odd index
//!   w = w_table[j]                    // twiddle exp(-j·2π·j/N), Q15
//!   t = (w * m) >> 15                 // complex multiply (Q15·Q0 → Q15)
//!   data[ia]      = (a + t) >> 1      // butterfly, per-stage /2
//!   data[ia + N2] = (a - t) >> 1
//! ```
//!
//! esp-dsp's inline `xtfixed_bf_{1..4}` fuses the multiply-add and
//! the shift in a single i32 expression with `+0x7fff` rounding to
//! reduce DC bias; we replicate the exact arithmetic.
//!
//! Twiddle table layout: same as `dsps_gen_w_r2_sc16` followed by
//! `dsps_bit_rev_sc16_ansi` — see [`gen_w_r2_sc16`].

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::TAU;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Generate the radix-2 sc16 twiddle table for an `N`-pt FFT.
///
/// Matches `dsps_gen_w_r2_sc16` + `dsps_bit_rev_sc16_ansi` (in-place
/// bit-reverse) from esp-dsp 1.8.2. Length = N/2 complex entries
/// (= N i16 values × 2 per `Complex<i16>`).
///
/// Generated in natural order, then bit-reversed in place — same as
/// `dsps_fft2r_init_sc16`.
pub fn gen_w_r2_sc16(n: usize) -> Vec<Complex<i16>> {
    assert!(n.is_power_of_two(), "gen_w_r2_sc16: N must be power of 2");
    let half = n / 2;
    let mut w: Vec<Complex<i16>> = vec![Complex::new(0i16, 0); half];
    let e = TAU / n as f32;
    for i in 0..half {
        // dsps_gen_w_r2_sc16 uses INT16_MAX (= 32767), NOT 32768.
        let re = (i16::MAX as f32 * (i as f32 * e).cos()) as i16;
        let im = (i16::MAX as f32 * (i as f32 * e).sin()) as i16;
        w[i] = Complex::new(re, im);
    }
    bit_rev_sc16(&mut w);
    w
}

/// In-place bit-reverse permutation. Matches `dsps_bit_rev_sc16_ansi`.
pub fn bit_rev_sc16(data: &mut [Complex<i16>]) {
    let n = data.len();
    assert!(n.is_power_of_two(), "bit_rev_sc16: N must be power of 2");
    let mut j = 0usize;
    for i in 1..(n - 1) {
        let mut k = n >> 1;
        while k <= j {
            j -= k;
            k >>= 1;
        }
        j += k;
        if i < j {
            data.swap(i, j);
        }
    }
}

// Constants from dsps_fft2r_sc16_ansi.c:
//   add_round_mult  = 0x7fff (= 32767, rounding constant before >>16)
//   mult_shift_const = 0x7fff (= 32767, ≈ Q15 unity for the a0 term)
const MULT_SHIFT_CONST: i32 = 0x7fff;
const ADD_ROUND_MULT: i32 = 0x7fff;

#[inline]
fn xtfixed_bf_1(a0: i16, a1: i16, a2: i16, a3: i16, a4: i16) -> i16 {
    // (a0 * 0x7fff - a1*a2 - a3*a4 + 0x7fff) >> 16
    let mut result: i32 = a0 as i32 * MULT_SHIFT_CONST;
    result -= a1 as i32 * a2 as i32 + a3 as i32 * a4 as i32;
    result += ADD_ROUND_MULT;
    (result >> 16) as i16
}

#[inline]
fn xtfixed_bf_2(a0: i16, a1: i16, a2: i16, a3: i16, a4: i16) -> i16 {
    // (a0 * 0x7fff - (a1*a2 - a3*a4) + 0x7fff) >> 16
    let mut result: i32 = a0 as i32 * MULT_SHIFT_CONST;
    result -= a1 as i32 * a2 as i32 - a3 as i32 * a4 as i32;
    result += ADD_ROUND_MULT;
    (result >> 16) as i16
}

#[inline]
fn xtfixed_bf_3(a0: i16, a1: i16, a2: i16, a3: i16, a4: i16) -> i16 {
    // (a0 * 0x7fff + a1*a2 + a3*a4 + 0x7fff) >> 16
    let mut result: i32 = a0 as i32 * MULT_SHIFT_CONST;
    result += a1 as i32 * a2 as i32 + a3 as i32 * a4 as i32;
    result += ADD_ROUND_MULT;
    (result >> 16) as i16
}

#[inline]
fn xtfixed_bf_4(a0: i16, a1: i16, a2: i16, a3: i16, a4: i16) -> i16 {
    // (a0 * 0x7fff + (a1*a2 - a3*a4) + 0x7fff) >> 16
    let mut result: i32 = a0 as i32 * MULT_SHIFT_CONST;
    result += a1 as i32 * a2 as i32 - a3 as i32 * a4 as i32;
    result += ADD_ROUND_MULT;
    (result >> 16) as i16
}

/// In-place radix-2 sc16 forward FFT. Output is **bit-reversed**;
/// call [`bit_rev_sc16`] separately on the result to natural order
/// (same as the embedded `dsps_fft2r_sc16_ae32_` + `dsps_bit_rev_sc16_ansi`
/// pair).
///
/// `w` must be the bit-reversed twiddle table from [`gen_w_r2_sc16`]
/// for the same N.
pub fn fft2r_sc16(data: &mut [Complex<i16>], w: &[Complex<i16>]) {
    let n = data.len();
    assert!(n.is_power_of_two(), "fft2r_sc16: N must be power of 2");
    assert_eq!(w.len(), n / 2, "fft2r_sc16: w table must be N/2 entries");

    let mut ie: usize = 1;
    let mut n2 = n / 2;
    while n2 > 0 {
        let mut ia: usize = 0;
        for j in 0..ie {
            let cs = w[j];
            for _ in 0..n2 {
                let m = ia + n2;
                let m_data = data[m];
                let a_data = data[ia];

                // m1 = (a - w*m) / 2
                let m1_re = xtfixed_bf_1(a_data.re, cs.re, m_data.re, cs.im, m_data.im);
                let m1_im = xtfixed_bf_2(a_data.im, cs.re, m_data.im, cs.im, m_data.re);
                data[m] = Complex::new(m1_re, m1_im);

                // m2 = (a + w*m) / 2
                let m2_re = xtfixed_bf_3(a_data.re, cs.re, m_data.re, cs.im, m_data.im);
                let m2_im = xtfixed_bf_4(a_data.im, cs.re, m_data.im, cs.im, m_data.re);
                data[ia] = Complex::new(m2_re, m2_im);

                ia += 1;
            }
            ia += n2;
        }
        ie <<= 1;
        n2 >>= 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip a single impulse through the radix-2 sc16 FFT and
    /// verify the output is roughly constant magnitude across bins
    /// (= the FFT of an impulse is constant). Uses N=256 for the
    /// FT8 mixed-radix inner kernel.
    #[test]
    fn sc16_impulse_256() {
        let n = 256;
        let w = gen_w_r2_sc16(n);
        let mut buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0); n];
        // Single impulse at index 0 with i16-MAX amplitude.
        buf[0] = Complex::new(i16::MAX, 0);
        fft2r_sc16(&mut buf, &w);
        bit_rev_sc16(&mut buf);
        // After /2 per stage × log2(N)=8 stages = /256 total gain,
        // impulse output magnitude ≈ INT16_MAX / 256 = 128.
        for c in buf.iter() {
            let mag2 = (c.re as i32) * (c.re as i32) + (c.im as i32) * (c.im as i32);
            // mag² ≈ 128² = 16384, allow generous slack for i16 rounding.
            assert!(
                (4000..50_000).contains(&mag2),
                "impulse bin mag²={mag2} out of range"
            );
        }
    }

    /// Pure tone at bin 32 (= integer-bin so all energy concentrated)
    /// should produce a single peak in the FFT output.
    #[test]
    fn sc16_pure_tone_256() {
        let n = 256;
        let k_bin = 32;
        let w = gen_w_r2_sc16(n);
        let mut buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0); n];
        for i in 0..n {
            let phase = TAU * (k_bin as f32) * (i as f32) / (n as f32);
            // Use ~half i16 range so the /N=/256 final magnitude fits
            // i16 without saturating.
            let re = (16000.0 * phase.cos()) as i16;
            let im = (16000.0 * phase.sin()) as i16;
            buf[i] = Complex::new(re, im);
        }
        fft2r_sc16(&mut buf, &w);
        bit_rev_sc16(&mut buf);
        // Expected peak at bin k_bin, magnitude ≈ 16000 (= input
        // amplitude after /N scaling of N×amp = N×16000 → 16000).
        let peak_mag2 = (buf[k_bin].re as i32).pow(2) + (buf[k_bin].im as i32).pow(2);
        assert!(
            peak_mag2 > 200_000_000,
            "peak bin mag²={peak_mag2} too low (expected ~16k²=256M)"
        );
        // Other bins should be near zero (small).
        for (i, c) in buf.iter().enumerate() {
            if i == k_bin {
                continue;
            }
            let mag2 = (c.re as i32).pow(2) + (c.im as i32).pow(2);
            assert!(
                mag2 < 1_000_000,
                "off-bin {i} leaked mag²={mag2} (expected near 0)"
            );
        }
    }
}
