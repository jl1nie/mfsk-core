//! 5120-point complex FFT via mixed-radix decomposition (1024 × 5).
//!
//! Same Cooley-Tukey shape as [`super::fft_mixed_3840`] (256 × 15), for
//! the same reason: the inner factor is a power of two so an external
//! implementation (rustfft on host, esp-dsp's radix-2 asm on Xtensa)
//! can serve it, and only the small odd factor is hand-rolled here.
//! The 5-pt sub-kernel is [`super::fft_15::fft_5`], already present
//! and unit-tested as one half of the 3840 path's PFA.
//!
//! **Why this length exists**: `FT4_DOWNSAMPLE.fft2_size` is 5120
//! (`ft4/decode.rs`), so `downsample_cached`'s inverse transform — run
//! once per FT4 candidate — is a 5120-point inverse FFT. 5120 =
//! 2¹⁰ · 5 is not a power of two, which is exactly the wall
//! `embedded-shared::esp_dsp_fft` documents for FT4
//! (`dsps_fft2r_fc32_ae32` is radix-2 only). Without this wrapper an
//! FT4 candidate cannot be decoded on ESP32-S3 at all, even when the
//! wideband `fft1_size = 92_160` transform is supplied pre-baked
//! through `decode_frame`'s `precomputed_fft` seam.
//!
//! ## Cooley-Tukey 1024 × 5 forward FFT
//!
//! Re-index `n = 5·n1 + n2` (0 ≤ n1 < 1024, 0 ≤ n2 < 5) and
//! `k = 1024·k2 + k1` (0 ≤ k1 < 1024, 0 ≤ k2 < 5). Then:
//!
//! ```text
//!   X[k] = Σ_{n1,n2} x[5·n1 + n2] · ω₅₁₂₀^((5·n1+n2)·(1024·k2+k1))
//!        = Σ_{n2} ω₅₁₂₀^(n2·k1) · ω₅^(n2·k2) · ( Σ_{n1} x[..] · ω₁₀₂₄^(n1·k1) )
//! ```
//!
//! ## Inverse
//!
//! [`ifft_5120_with`] uses the conjugation identity
//! `IDFT(x) = conj(DFT(conj(x)))` rather than a second derivation, so
//! it inherits the forward path's correctness exactly and needs no
//! separate twiddle table. Unnormalised, matching `rustfft`'s
//! `plan_fft_inverse` (and therefore matching what
//! `downsample::downsample_cached` already expects — it applies its
//! own `1/sqrt(fft1·fft2)` scaling afterwards).

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::TAU;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::fft_15::fft_5;

pub const N: usize = 5120;
const N1: usize = 1024;
const N2: usize = 5;

/// Forward 5120-pt complex FFT, in-place. Caller supplies a 1024-pt
/// in-place forward FFT closure so the same wrapper works on host
/// (rustfft) and embedded (esp-dsp).
///
/// `twiddles` must be [`build_twiddles`]'s table, of length `N`, with
/// `twiddles[n2 * 1024 + k1] = ω₅₁₂₀^(n2 · k1)`. Build it once at
/// startup and cache it.
pub fn fft_5120_with(
    buf: &mut [Complex32; N],
    fft_1024: &mut dyn FnMut(&mut [Complex32; N1]),
    twiddles: &[Complex32; N],
) {
    // Step 1: `m[n2][n1] = x[5·n1 + n2]`, transposed into a Vec so each
    //         row is contiguous for the 1024-pt FFT.
    let mut m: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); N];
    for n1 in 0..N1 {
        for n2 in 0..N2 {
            m[n2 * N1 + n1] = buf[N2 * n1 + n2];
        }
    }

    // Step 2: 1024-pt FFT along each of the 5 rows.
    for n2 in 0..N2 {
        let row: &mut [Complex32; N1] = (&mut m[n2 * N1..(n2 + 1) * N1])
            .try_into()
            .expect("row slice = N1 elements");
        fft_1024(row);
    }

    // Step 3: twiddle by ω₅₁₂₀^(n2·k1).
    for n2 in 0..N2 {
        for k1 in 0..N1 {
            m[n2 * N1 + k1] *= twiddles[n2 * N1 + k1];
        }
    }

    // Step 4: 5-pt FFT along each of the 1024 columns.
    let mut col = [Complex32::new(0.0, 0.0); N2];
    for k1 in 0..N1 {
        for k2 in 0..N2 {
            col[k2] = m[k2 * N1 + k1];
        }
        fft_5(&mut col);
        for k2 in 0..N2 {
            m[k2 * N1 + k1] = col[k2];
        }
    }

    // Step 5: natural-order write-back, `k = 1024·k2 + k1`.
    for k2 in 0..N2 {
        for k1 in 0..N1 {
            buf[N1 * k2 + k1] = m[k2 * N1 + k1];
        }
    }
}

/// Unnormalised inverse 5120-pt complex FFT, in-place, via
/// `IDFT(x) = conj(DFT(conj(x)))`.
///
/// `fft_1024` must still be a **forward** 1024-pt FFT — the conjugation
/// wraps the whole 5120-point transform, so every stage inside runs
/// forward. Same scaling convention as `rustfft::plan_fft_inverse`
/// (no `1/N`).
pub fn ifft_5120_with(
    buf: &mut [Complex32; N],
    fft_1024: &mut dyn FnMut(&mut [Complex32; N1]),
    twiddles: &[Complex32; N],
) {
    for c in buf.iter_mut() {
        c.im = -c.im;
    }
    fft_5120_with(buf, fft_1024, twiddles);
    for c in buf.iter_mut() {
        c.im = -c.im;
    }
}

/// Build the inter-stage twiddle table: `twiddles[n2 * 1024 + k1] =
/// exp(-j · 2π · n2 · k1 / 5120)`.
pub fn build_twiddles() -> alloc::boxed::Box<[Complex32; N]> {
    let mut t = vec![Complex32::new(0.0, 0.0); N].into_boxed_slice();
    for n2 in 0..N2 {
        for k1 in 0..N1 {
            let phi = -TAU * (n2 as f32) * (k1 as f32) / (N as f32);
            t[n2 * N1 + k1] = Complex32::new(phi.cos(), phi.sin());
        }
    }
    let raw = alloc::boxed::Box::into_raw(t) as *mut [Complex32; N];
    unsafe { alloc::boxed::Box::from_raw(raw) }
}

#[cfg(test)]
#[cfg(feature = "fft-rustfft")]
mod tests {
    use super::*;

    fn rustfft_5120(input: &[Complex32; N], forward: bool) -> [Complex32; N] {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = if forward {
            planner.plan_fft_forward(N)
        } else {
            planner.plan_fft_inverse(N)
        };
        let mut buf: Vec<Complex32> = input.to_vec();
        fft.process(&mut buf);
        let mut out = [Complex32::new(0.0, 0.0); N];
        out.copy_from_slice(&buf);
        out
    }

    fn rustfft_1024(buf: &mut [Complex32; N1]) {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(N1);
        let mut tmp: Vec<Complex32> = buf.to_vec();
        fft.process(&mut tmp);
        buf.copy_from_slice(&tmp);
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

    /// Absolute tolerance scaled to the transform's own magnitude:
    /// a 5120-point f32 FFT of unit-ish input has outputs of order
    /// sqrt(N) ≈ 72, so ~1e-3 is f32 round-off, not a wrong answer.
    fn assert_close(got: &[Complex32; N], want: &[Complex32; N], eps: f32, what: &str) {
        for (k, (a, b)) in got.iter().zip(want.iter()).enumerate() {
            assert!(
                (a.re - b.re).abs() < eps && (a.im - b.im).abs() < eps,
                "{what}: bin {k} got {a:?} want {b:?}"
            );
        }
    }

    #[test]
    fn fft5120_impulse_is_flat() {
        let mut x = [Complex32::new(0.0, 0.0); N];
        x[0] = Complex32::new(1.0, 0.0);
        let tw = build_twiddles();
        fft_5120_with(&mut x, &mut rustfft_1024, &tw);
        for (k, c) in x.iter().enumerate() {
            assert!(
                (c.re - 1.0).abs() < 1e-5 && c.im.abs() < 1e-5,
                "bin {k}: {c:?}"
            );
        }
    }

    #[test]
    fn fft5120_matches_rustfft_forward() {
        let tw = build_twiddles();
        for seed in [1u64, 0xdead_beef, 42] {
            let x = random_input(seed);
            let want = rustfft_5120(&x, true);
            let mut got = x;
            fft_5120_with(&mut got, &mut rustfft_1024, &tw);
            assert_close(&got, &want, 1e-2, "forward");
        }
    }

    #[test]
    fn ifft5120_matches_rustfft_inverse() {
        let tw = build_twiddles();
        for seed in [7u64, 0x0bad_c0de] {
            let x = random_input(seed);
            let want = rustfft_5120(&x, false);
            let mut got = x;
            ifft_5120_with(&mut got, &mut rustfft_1024, &tw);
            assert_close(&got, &want, 1e-2, "inverse");
        }
    }

    /// Forward then inverse returns `N ×` the input — the unnormalised
    /// convention this module and `rustfft` share.
    #[test]
    fn fft5120_roundtrip_scales_by_n() {
        let tw = build_twiddles();
        let x = random_input(99);
        let mut got = x;
        fft_5120_with(&mut got, &mut rustfft_1024, &tw);
        ifft_5120_with(&mut got, &mut rustfft_1024, &tw);
        let scaled: [Complex32; N] = core::array::from_fn(|i| x[i] * N as f32);
        assert_close(&got, &scaled, 5e-2, "roundtrip");
    }
}
