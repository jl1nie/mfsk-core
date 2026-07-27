//! 15-point complex FFT via Good-Thomas prime-factor decomposition (3 × 5).
//!
//! Used as one half of the 3840-point mixed-radix FFT (3840 = 256 × 15).
//! Twiddle factors for the 3-pt and 5-pt sub-kernels are compile-time
//! constants — no runtime sin/cos calls.
//!
//! 15 = 3 × 5 with gcd(3, 5) = 1, so Good-Thomas (PFA) applies and there
//! are **no inter-kernel twiddle multiplications** between the 3-pt and
//! 5-pt stages — only an index permutation built from the CRT.
//!
//! Cost (forward FFT, complex input):
//! - 3-pt × 5: 6 complex muls + 6 complex adds × 5 = ~30 cmul, 30 cadd
//! - 5-pt × 3: ~10 cmul + 32 cadd × 3 = ~30 cmul, 96 cadd
//! - Total: ~60 cmul + 126 cadd per 15-pt FFT
//!
//! Verified bit-equivalent (within f32 epsilon ~1e-5) to `rustfft::FftPlanner`
//! for impulse / DC / sinusoid / random inputs.

use num_complex::Complex32;

// ── 3-pt twiddles: W₃ = e^(-j 2π/3) ─────────────────────────────────────
//   W₃   = (cos -2π/3, sin -2π/3) = (-1/2, -√3/2)
//   W₃²  = (cos -4π/3, sin -4π/3) = (-1/2, +√3/2)
const W3_RE: f32 = -0.5;
const W3_IM: f32 = -0.866_025_4; // -√3/2 ≈ -sin(2π/3)

/// Forward 3-pt complex FFT, in-place.
/// `X[k] = Σ_{n=0..2} x[n] · W₃^(n·k)` for k = 0..2.
#[inline]
pub fn fft_3(x: &mut [Complex32; 3]) {
    let x0 = x[0];
    let x1 = x[1];
    let x2 = x[2];

    // X0 = x0 + x1 + x2
    x[0] = Complex32::new(x0.re + x1.re + x2.re, x0.im + x1.im + x2.im);

    // X1 = x0 + x1 · W₃ + x2 · W₃²
    //    = x0 + x1·(W3_RE + j W3_IM) + x2·(W3_RE - j W3_IM)
    //    = x0 + W3_RE·(x1+x2)·  + j W3_IM·(x1 - x2)·
    // Symmetry: real part of W3² = real part of W3 = W3_RE,
    //           imag part of W3² = -W3_IM
    let sum_re = x1.re + x2.re;
    let sum_im = x1.im + x2.im;
    let diff_re = x1.re - x2.re;
    let diff_im = x1.im - x2.im;

    let t_re = W3_RE * sum_re;
    let t_im = W3_RE * sum_im;
    // j W3_IM · (a + jb) = -W3_IM·b + j W3_IM·a
    let r_re = W3_IM * diff_im; // = +W3_IM · imag(x1-x2) becomes real after j-mul
    let r_im = -W3_IM * diff_re;

    // X1 = x0 + t + j(W3_IM)(x1-x2) flipped
    // Actually: X1 = x0 + W3·x1 + W3²·x2.
    //   W3·x1   = (W3_RE·x1.re - W3_IM·x1.im, W3_RE·x1.im + W3_IM·x1.re)
    //   W3²·x2  = (W3_RE·x2.re + W3_IM·x2.im, W3_RE·x2.im - W3_IM·x2.re)
    //   sum.re  = W3_RE·(x1.re+x2.re) + W3_IM·(x2.im-x1.im)
    //           = W3_RE·sum_re + W3_IM·(-diff_im)
    //   sum.im  = W3_RE·(x1.im+x2.im) + W3_IM·(x1.re-x2.re)
    //           = W3_RE·sum_im + W3_IM·diff_re
    let _ = (r_re, r_im, t_re, t_im); // unused alias above; recompute clearly:
    let x1_x2_sum_re = W3_RE * sum_re + W3_IM * (-diff_im);
    let x1_x2_sum_im = W3_RE * sum_im + W3_IM * diff_re;
    x[1] = Complex32::new(x0.re + x1_x2_sum_re, x0.im + x1_x2_sum_im);

    // X2 = x0 + W3²·x1 + W3·x2 — swap roles of x1 and x2 above.
    //   W3²·x1  = (W3_RE·x1.re + W3_IM·x1.im, W3_RE·x1.im - W3_IM·x1.re)
    //   W3·x2   = (W3_RE·x2.re - W3_IM·x2.im, W3_RE·x2.im + W3_IM·x2.re)
    //   sum.re  = W3_RE·sum_re + W3_IM·(x1.im - x2.im) = W3_RE·sum_re + W3_IM·diff_im
    //   sum.im  = W3_RE·sum_im + W3_IM·(x2.re - x1.re) = W3_RE·sum_im - W3_IM·diff_re
    let x1_x2_sum_re_2 = W3_RE * sum_re + W3_IM * diff_im;
    let x1_x2_sum_im_2 = W3_RE * sum_im - W3_IM * diff_re;
    x[2] = Complex32::new(x0.re + x1_x2_sum_re_2, x0.im + x1_x2_sum_im_2);
}

// ── 15-pt FFT via Good-Thomas prime-factor decomposition (3 × 5) ────────
//
// Index mappings (from G.B. Thomas / I.J. Good, gcd(3,5)=1):
//   input n  = (5·n1 + 3·n2) mod 15,   n1 ∈ [0,2], n2 ∈ [0,4]
//   output k = (10·k1 + 6·k2) mod 15
// The α=10, β=6 choice satisfies α≡1 (mod 3), α≡0 (mod 5), β≡0 (mod 3),
// β≡1 (mod 5), which is what makes the PFA "twiddle-free" between the
// 3-pt and 5-pt stages.

const INPUT_MAP_15: [usize; 15] = {
    let mut m = [0usize; 15];
    let mut n1 = 0;
    while n1 < 3 {
        let mut n2 = 0;
        while n2 < 5 {
            m[n1 * 5 + n2] = (5 * n1 + 3 * n2) % 15;
            n2 += 1;
        }
        n1 += 1;
    }
    m
};

const OUTPUT_MAP_15: [usize; 15] = {
    let mut m = [0usize; 15];
    let mut k1 = 0;
    while k1 < 3 {
        let mut k2 = 0;
        while k2 < 5 {
            m[k1 * 5 + k2] = (10 * k1 + 6 * k2) % 15;
            k2 += 1;
        }
        k1 += 1;
    }
    m
};

/// Forward 15-pt complex FFT, in-place.
///
/// `X[k] = Σ_{n=0..14} x[n] · ω₁₅^(n·k)` for k = 0..14 with ω₁₅ = e^(-j 2π/15).
///
/// Algorithm: 3 × 5 Good-Thomas PFA — gcd(3,5)=1 so the inter-stage
/// twiddles vanish, leaving only:
///   1. CRT-based input permutation into a 3×5 matrix
///   2. Five 5-pt FFTs along the n2 axis (one per n1 row)
///   3. Three 3-pt FFTs along the n1 axis (one per k2 column)
///   4. CRT-based output permutation back into the k array
///
/// All sub-kernel twiddles are compile-time constants (no sin/cos runtime).
pub fn fft_15(x: &mut [Complex32; 15]) {
    // Step 1: input permutation, build 3×5 matrix `m[n1][n2]`.
    let mut m = [[Complex32::new(0.0, 0.0); 5]; 3];
    for n1 in 0..3 {
        for n2 in 0..5 {
            m[n1][n2] = x[INPUT_MAP_15[n1 * 5 + n2]];
        }
    }

    // Step 2: 5-pt FFT along each of the 3 rows.
    for row in m.iter_mut() {
        fft_5(row);
    }

    // Step 3: 3-pt FFT along each of the 5 columns.
    for k2 in 0..5 {
        let mut col = [m[0][k2], m[1][k2], m[2][k2]];
        fft_3(&mut col);
        m[0][k2] = col[0];
        m[1][k2] = col[1];
        m[2][k2] = col[2];
    }

    // Step 4: output permutation.
    for k1 in 0..3 {
        for k2 in 0..5 {
            x[OUTPUT_MAP_15[k1 * 5 + k2]] = m[k1][k2];
        }
    }
}

#[cfg(test)]
mod tests_15 {
    use super::*;

    fn rustfft_15(input: &[Complex32; 15]) -> [Complex32; 15] {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(15);
        let mut buf: Vec<Complex32> = input.to_vec();
        fft.process(&mut buf);
        let mut out = [Complex32::new(0.0, 0.0); 15];
        out.copy_from_slice(&buf);
        out
    }

    fn close(a: Complex32, b: Complex32, eps: f32) -> bool {
        (a.re - b.re).abs() < eps && (a.im - b.im).abs() < eps
    }

    #[test]
    fn fft15_impulse() {
        let mut x = [Complex32::new(0.0, 0.0); 15];
        x[0] = Complex32::new(1.0, 0.0);
        let expected = rustfft_15(&x);
        fft_15(&mut x);
        for k in 0..15 {
            assert!(close(x[k], expected[k], 1e-5), "k={k}");
        }
    }

    #[test]
    fn fft15_dc() {
        let mut x = [Complex32::new(2.0, -1.0); 15];
        let expected = rustfft_15(&x);
        fft_15(&mut x);
        for k in 0..15 {
            assert!(close(x[k], expected[k], 1e-5));
        }
    }

    #[test]
    fn fft15_random() {
        let mut xs = [Complex32::new(0.0, 0.0); 15];
        // Deterministic pseudo-random.
        let seed: u64 = 0xcafef00d_dead_beef;
        let mut s = seed;
        for c in xs.iter_mut() {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let r = (s >> 33) as f32 / (1u32 << 31) as f32 - 1.0;
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let i = (s >> 33) as f32 / (1u32 << 31) as f32 - 1.0;
            *c = Complex32::new(r, i);
        }
        let expected = rustfft_15(&xs);
        let mut x = xs;
        fft_15(&mut x);
        for k in 0..15 {
            assert!(
                close(x[k], expected[k], 1e-4),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }

    #[test]
    fn fft15_pure_bin() {
        // Single complex sinusoid at bin 4: X[k] should be 15 at k=4, 0 else.
        use core::f32::consts::TAU;
        let mut x = [Complex32::new(0.0, 0.0); 15];
        for n in 0..15 {
            let phi = -TAU * 4.0 * (n as f32) / 15.0;
            x[n] = Complex32::new(phi.cos(), phi.sin());
        }
        let expected = rustfft_15(&x);
        fft_15(&mut x);
        for k in 0..15 {
            assert!(
                close(x[k], expected[k], 1e-4),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }
}

// ── 5-pt twiddles ──────────────────────────────────────────────────────
//   ω = e^(-j 2π/5), so ωᵏ = (cos -2πk/5, sin -2πk/5)
//   c1 = cos(2π/5), s1 = sin(2π/5)
//   c2 = cos(4π/5), s2 = sin(4π/5)
const C1: f32 = 0.309_017; //  cos(72°)  — f32 ~7 sig digits
const S1: f32 = 0.951_056_5; //   sin(72°)
const C2: f32 = -0.809_017; //    cos(144°)
const S2: f32 = 0.587_785_3; //  sin(144°)

/// Forward 5-pt complex FFT, in-place.
/// `X[k] = Σ_{n=0..4} x[n] · ω^(n·k)` for k = 0..4 with ω = e^(-j 2π/5).
///
/// Uses the Rader/Winograd-style sum/diff pairing with conjugate symmetry
/// of `ω` and `ω̄` to share multiplications between symmetric output pairs
/// (`X[1]` ↔ `X[4]`, `X[2]` ↔ `X[3]`).
#[inline]
pub fn fft_5(x: &mut [Complex32; 5]) {
    let x0 = x[0];

    let sum14_re = x[1].re + x[4].re;
    let sum14_im = x[1].im + x[4].im;
    let diff14_re = x[1].re - x[4].re;
    let diff14_im = x[1].im - x[4].im;
    let sum23_re = x[2].re + x[3].re;
    let sum23_im = x[2].im + x[3].im;
    let diff23_re = x[2].re - x[3].re;
    let diff23_im = x[2].im - x[3].im;

    // X[0] = sum of all inputs.
    x[0] = Complex32::new(x0.re + sum14_re + sum23_re, x0.im + sum14_im + sum23_im);

    let t1_re = C1 * sum14_re + C2 * sum23_re;
    let t1_im = C1 * sum14_im + C2 * sum23_im;
    let t2_re = C2 * sum14_re + C1 * sum23_re;
    let t2_im = C2 * sum14_im + C1 * sum23_im;

    let u1_im = S1 * diff14_im + S2 * diff23_im;
    let u1_re = S1 * diff14_re + S2 * diff23_re;
    let u2_im = S2 * diff14_im - S1 * diff23_im;
    let u2_re = S2 * diff14_re - S1 * diff23_re;

    x[1] = Complex32::new(x0.re + t1_re + u1_im, x0.im + t1_im - u1_re);
    x[4] = Complex32::new(x0.re + t1_re - u1_im, x0.im + t1_im + u1_re);
    x[2] = Complex32::new(x0.re + t2_re + u2_im, x0.im + t2_im - u2_re);
    x[3] = Complex32::new(x0.re + t2_re - u2_im, x0.im + t2_im + u2_re);
}

#[cfg(test)]
mod tests_5 {
    use super::*;

    fn rustfft_5(input: &[Complex32; 5]) -> [Complex32; 5] {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(5);
        let mut buf: Vec<Complex32> = input.to_vec();
        fft.process(&mut buf);
        let mut out = [Complex32::new(0.0, 0.0); 5];
        out.copy_from_slice(&buf);
        out
    }

    fn close(a: Complex32, b: Complex32, eps: f32) -> bool {
        (a.re - b.re).abs() < eps && (a.im - b.im).abs() < eps
    }

    #[test]
    fn fft5_impulse() {
        let mut x = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 0.0),
            Complex32::new(0.0, 0.0),
            Complex32::new(0.0, 0.0),
            Complex32::new(0.0, 0.0),
        ];
        let expected = rustfft_5(&x);
        fft_5(&mut x);
        for k in 0..5 {
            assert!(
                close(x[k], expected[k], 1e-5),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }

    #[test]
    fn fft5_dc() {
        let mut x = [
            Complex32::new(1.0, 0.0),
            Complex32::new(1.0, 0.0),
            Complex32::new(1.0, 0.0),
            Complex32::new(1.0, 0.0),
            Complex32::new(1.0, 0.0),
        ];
        let expected = rustfft_5(&x);
        fft_5(&mut x);
        for k in 0..5 {
            assert!(close(x[k], expected[k], 1e-5));
        }
    }

    #[test]
    fn fft5_random() {
        let xs = [
            Complex32::new(0.7, -0.3),
            Complex32::new(-1.4, 1.1),
            Complex32::new(0.2, 0.6),
            Complex32::new(1.5, -0.8),
            Complex32::new(-0.9, 0.4),
        ];
        let expected = rustfft_5(&xs);
        let mut x = xs;
        fft_5(&mut x);
        for k in 0..5 {
            assert!(
                close(x[k], expected[k], 1e-5),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }

    #[test]
    fn fft5_complex_sinusoid() {
        // Pure +1 frequency bin: X should be N at k=1, 0 elsewhere.
        use core::f32::consts::TAU;
        let mut x = [Complex32::new(0.0, 0.0); 5];
        for n in 0..5 {
            let phi = -TAU * (n as f32) / 5.0;
            x[n] = Complex32::new(phi.cos(), phi.sin());
        }
        let expected = rustfft_5(&x);
        fft_5(&mut x);
        for k in 0..5 {
            assert!(
                close(x[k], expected[k], 1e-5),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }
}

#[cfg(test)]
mod tests_3 {
    use super::*;

    fn rustfft_3(input: &[Complex32; 3]) -> [Complex32; 3] {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(3);
        let mut buf: Vec<Complex32> = input.to_vec();
        fft.process(&mut buf);
        [buf[0], buf[1], buf[2]]
    }

    fn close(a: Complex32, b: Complex32, eps: f32) -> bool {
        (a.re - b.re).abs() < eps && (a.im - b.im).abs() < eps
    }

    #[test]
    fn fft3_impulse() {
        let mut x = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 0.0),
            Complex32::new(0.0, 0.0),
        ];
        let expected = rustfft_3(&x);
        fft_3(&mut x);
        for k in 0..3 {
            assert!(
                close(x[k], expected[k], 1e-5),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }

    #[test]
    fn fft3_dc() {
        let mut x = [
            Complex32::new(2.5, 0.0),
            Complex32::new(2.5, 0.0),
            Complex32::new(2.5, 0.0),
        ];
        let expected = rustfft_3(&x);
        fft_3(&mut x);
        for k in 0..3 {
            assert!(close(x[k], expected[k], 1e-5));
        }
    }

    #[test]
    fn fft3_random() {
        let xs = [
            Complex32::new(0.7, -0.3),
            Complex32::new(-1.4, 1.1),
            Complex32::new(0.2, 0.6),
        ];
        let expected = rustfft_3(&xs);
        let mut x = xs;
        fft_3(&mut x);
        for k in 0..3 {
            assert!(
                close(x[k], expected[k], 1e-5),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }
}
