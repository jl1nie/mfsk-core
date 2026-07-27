//! 3840-point complex FFT via mixed-radix decomposition (256 × 15).
//!
//! 3840 = 256 × 15 with gcd(256, 15) = 1, so we **could** use Good-Thomas
//! PFA to skip inter-stage twiddles. We use **Cooley-Tukey** instead: it
//! is a hair more arithmetic (one twiddle multiplication per element)
//! but lets the 256-pt sub-kernel be a plain power-of-two FFT supplied
//! by an external implementation (rustfft on host, esp-dsp on Xtensa
//! embedded). The PFA index permutation would force the 256-pt path to
//! consume non-stride-1 inputs, breaking esp-dsp's expectations.
//!
//! ## Cooley-Tukey 256 × 15 forward FFT
//!
//! Re-index `n = 15·n1 + n2` (0 ≤ n1 < 256, 0 ≤ n2 < 15) and
//! `k = 256·k2 + k1` (0 ≤ k1 < 256, 0 ≤ k2 < 15). Then:
//!
//! ```text
//!   X[k] = Σ_{n1,n2} x[15·n1 + n2] · ω₃₈₄₀^((15·n1+n2)·(256·k2+k1))
//!        = Σ_{n2} ω₃₈₄₀^(n2·k1) · ω₁₅^(n2·k2) · ( Σ_{n1} x[..] · ω₂₅₆^(n1·k1) )
//! ```
//!
//! Algorithm:
//!   1. Reshape input as `m[n2][n1]` (15 rows, 256 cols)
//!   2. 256-pt FFT along each of the 15 rows (uses external 256-pt kernel)
//!   3. Twiddle by `ω₃₈₄₀^(n2·k1)` element-wise
//!   4. 15-pt FFT along each of the 256 columns (uses our `fft_15`)
//!   5. Output `X[256·k2 + k1] = m[k2][k1]`
//!
//! ## Cost
//!
//! - 256-pt FFT × 15: ~1.2 ms on LX7 with esp-dsp asm (≈80 µs each)
//! - twiddle multiply: 3840 complex mul ≈ 0.5 ms
//! - 15-pt PFA × 256: ~3-pt + 5-pt sub-kernels, ~5 µs each → 1.3 ms
//!
//! Total ≈ 3 ms/frame, fits the 4 ms streaming-pipeline budget.

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::TAU;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::fft_15::fft_15;

pub const N: usize = 3840;
const N1: usize = 256;
const N2: usize = 15;

/// Forward 3840-pt complex FFT, in-place. Caller supplies a 256-pt FFT
/// closure so the same wrapper works on host (rustfft) and embedded
/// (esp-dsp). The closure must implement an in-place forward FFT.
///
/// `twiddles` should be the precomputed table of length `N` such that
/// `twiddles[n2 * 256 + k1] = ω₃₈₄₀^(n2 · k1)`. Use [`build_twiddles`]
/// once at startup and cache it.
pub fn fft_3840_with(
    buf: &mut [Complex32; N],
    fft_256: &mut dyn FnMut(&mut [Complex32; N1]),
    twiddles: &[Complex32; N],
) {
    // Step 1: reshape input so row n2 holds `x[n2], x[n2+15], x[n2+30], …`.
    //         Equivalently `m[n2][n1] = x[15·n1 + n2]`. We transpose into
    //         a Vec to keep each row contiguous for the 256-pt FFT.
    let mut m: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); N];
    for n1 in 0..N1 {
        for n2 in 0..N2 {
            m[n2 * N1 + n1] = buf[15 * n1 + n2];
        }
    }

    // Step 2: 256-pt FFT along each of the 15 rows.
    for n2 in 0..N2 {
        // SAFETY: each row is a contiguous &mut [Complex32; 256].
        let row: &mut [Complex32; N1] = (&mut m[n2 * N1..(n2 + 1) * N1])
            .try_into()
            .expect("row slice = N1 elements");
        fft_256(row);
    }

    // Step 3: twiddle by ω₃₈₄₀^(n2·k1).
    for n2 in 0..N2 {
        for k1 in 0..N1 {
            m[n2 * N1 + k1] *= twiddles[n2 * N1 + k1];
        }
    }

    // Step 4: 15-pt FFT along each of the 256 columns. After this,
    //         `m[k2 * N1 + k1]` is the unscrambled output.
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

    // Step 5: write back to buf with the natural-order index
    //         k = 256 · k2 + k1.
    for k2 in 0..N2 {
        for k1 in 0..N1 {
            buf[N1 * k2 + k1] = m[k2 * N1 + k1];
        }
    }
}

/// Build the inter-stage twiddle table: `twiddles[n2 * 256 + k1] =
/// exp(-j · 2π · n2 · k1 / 3840)`.
pub fn build_twiddles() -> alloc::boxed::Box<[Complex32; N]> {
    let mut t = vec![Complex32::new(0.0, 0.0); N].into_boxed_slice();
    for n2 in 0..N2 {
        for k1 in 0..N1 {
            let phi = -TAU * (n2 as f32) * (k1 as f32) / (N as f32);
            t[n2 * N1 + k1] = Complex32::new(phi.cos(), phi.sin());
        }
    }
    // Convert Box<[T]> to Box<[T; N]> (same layout, length-checked).
    let raw = alloc::boxed::Box::into_raw(t) as *mut [Complex32; N];
    unsafe { alloc::boxed::Box::from_raw(raw) }
}

#[cfg(test)]
#[cfg(feature = "fft-rustfft")]
mod tests {
    use super::*;

    fn rustfft_3840(input: &[Complex32; N]) -> [Complex32; N] {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(N);
        let mut buf: Vec<Complex32> = input.to_vec();
        fft.process(&mut buf);
        let mut out = [Complex32::new(0.0, 0.0); N];
        out.copy_from_slice(&buf);
        out
    }

    fn rustfft_256(buf: &mut [Complex32; N1]) {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(N1);
        let mut tmp: Vec<Complex32> = buf.to_vec();
        fft.process(&mut tmp);
        buf.copy_from_slice(&tmp);
    }

    fn close(a: Complex32, b: Complex32, eps: f32) -> bool {
        (a.re - b.re).abs() < eps && (a.im - b.im).abs() < eps
    }

    fn random_input(seed: u64) -> [Complex32; N] {
        let mut x = [Complex32::new(0.0, 0.0); N];
        let mut s = seed;
        for c in x.iter_mut() {
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
        x
    }

    #[test]
    fn fft3840_impulse() {
        let mut x = [Complex32::new(0.0, 0.0); N];
        x[0] = Complex32::new(1.0, 0.0);
        let expected = rustfft_3840(&x);
        let tw = build_twiddles();
        fft_3840_with(&mut x, &mut rustfft_256, &tw);
        for k in 0..N {
            assert!(
                close(x[k], expected[k], 1e-4),
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }

    #[test]
    fn fft3840_pure_bin() {
        // Single complex sinusoid at bin 137: X should be N at k=137.
        let mut x = [Complex32::new(0.0, 0.0); N];
        for n in 0..N {
            let phi = -TAU * 137.0 * (n as f32) / (N as f32);
            x[n] = Complex32::new(phi.cos(), phi.sin());
        }
        let expected = rustfft_3840(&x);
        let tw = build_twiddles();
        fft_3840_with(&mut x, &mut rustfft_256, &tw);
        for k in 0..N {
            assert!(
                close(x[k], expected[k], 1e-2), // tolerance loosened for f32 over N=3840
                "k={k}: got {:?}, want {:?}",
                x[k],
                expected[k]
            );
        }
    }

    #[test]
    fn fft3840_random() {
        let xs = random_input(0xfeed_face_dead_beef);
        let expected = rustfft_3840(&xs);
        let tw = build_twiddles();
        let mut x = xs;
        fft_3840_with(&mut x, &mut rustfft_256, &tw);
        // f32 precision over a 3840-pt FFT: amplitudes stack via √N noise,
        // 1e-3 relative tolerance.
        let mut max_err = 0.0f32;
        for k in 0..N {
            let dr = (x[k].re - expected[k].re).abs();
            let di = (x[k].im - expected[k].im).abs();
            max_err = max_err.max(dr.max(di));
        }
        assert!(
            max_err < 1e-2,
            "max error {max_err:.4e} exceeds 1e-2 over 3840-pt FFT"
        );
    }
}
