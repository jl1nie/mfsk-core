//! 2304-point complex FFT via mixed-radix decomposition (256 × 9).
//!
//! Third member of the same family as [`super::fft_mixed_3840`]
//! (256 × 15) and [`super::fft_mixed_5120`] (1024 × 5), built the same
//! way and for the same reason: keep the large factor a power of two so
//! an external implementation (rustfft on host, esp-dsp's radix-2 asm
//! on Xtensa) serves it, and hand-roll only the small odd factor.
//!
//! **Why this length exists**: `engine::ft4_coarse::ft4_coarse_sync` —
//! the port of WSJT-X `getcandidates4.f90` — runs a Nuttall-windowed
//! `NFFT1 = 4·NSPS = 2304`-point transform over every one-symbol step
//! of the slot (~152 of them). 2304 = 2⁸·3² is not a power of two, so
//! it is the *last* length standing between FT4 and a self-contained
//! embedded receiver. `fft_mixed_5120` removed the per-candidate one
//! and `ft4::ddc` removed the 92 160-point slot one; until this, the
//! `ft4-bench` on a CoreS3 still had to ship a candidate *list* baked
//! on a host (`embedded-poc/assets/ft4_golden_candidates.bin`).
//!
//! ## Cooley-Tukey 256 × 9
//!
//! Re-index `n = 9·n1 + n2` (0 ≤ n1 < 256, 0 ≤ n2 < 9) and
//! `k = 256·k2 + k1` (0 ≤ k1 < 256, 0 ≤ k2 < 9):
//!
//! ```text
//!   X[k] = Σ_{n2} ω₂₃₀₄^(n2·k1) · ω₉^(n2·k2) · ( Σ_{n1} x[9·n1+n2] · ω₂₅₆^(n1·k1) )
//! ```
//!
//! The 9-point sub-kernel is itself one more level of the same
//! decomposition, 3 × 3 over [`super::fft_15::fft_3`] — 6 three-point
//! transforms and 4 non-trivial twiddles, against the 81 complex
//! multiplies a direct 9-point DFT would cost at every one of the 256
//! columns.
//!
//! ## Inverse
//!
//! [`ifft_2304_with`] uses `IDFT(x) = conj(DFT(conj(x)))`, as
//! `fft_mixed_5120` does, so it inherits the forward path's
//! correctness and needs no second twiddle table. Unnormalised,
//! matching `rustfft::plan_fft_inverse`. No caller needs it today —
//! `ft4_coarse_sync` is forward-only — but the two-line identity costs
//! less than explaining its absence.

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::TAU;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::fft_15::fft_3;

pub const N: usize = 2304;
const N1: usize = 256;
const N2: usize = 9;

/// ω₉ = e^(-j2π/9) and its square, the only non-trivial twiddles the
/// 3 × 3 decomposition of [`fft_9`] needs (`ω₉^(n2·k1)` for
/// `n2, k1 ∈ {1, 2}`; every other combination is 1 or a power of ω₉
/// reachable by squaring one of these two).
const W9_1: Complex32 = Complex32::new(0.766_044_43, -0.642_787_6); // e^(-j2π/9)
const W9_2: Complex32 = Complex32::new(0.173_648_18, -0.984_807_7); // e^(-j4π/9)
const W9_4: Complex32 = Complex32::new(-0.939_692_6, -0.342_020_15); // e^(-j8π/9)

/// Forward 9-pt complex FFT, in-place, as Cooley-Tukey 3 × 3 over
/// [`fft_3`].
///
/// `n = 3·n1 + n2`, `k = 3·k2 + k1`, so
/// `X[k] = Σ_{n2} ω₉^(n2·k1) · ω₃^(n2·k2) · (Σ_{n1} x[3n1+n2] ω₃^(n1·k1))`.
#[inline]
pub fn fft_9(x: &mut [Complex32; 9]) {
    // Rows: `m[n2][n1] = x[3·n1 + n2]`, one 3-pt FFT each.
    let mut m = [[Complex32::new(0.0, 0.0); 3]; 3];
    for (n2, row) in m.iter_mut().enumerate() {
        for (n1, cell) in row.iter_mut().enumerate() {
            *cell = x[3 * n1 + n2];
        }
        fft_3(row);
    }

    // Twiddle by ω₉^(n2·k1). Row 0 and column 0 are unity; the rest are
    // ω₉, ω₉², ω₉², ω₉⁴ — the products `n2·k1` for `n2, k1 ∈ {1, 2}`.
    m[1][1] *= W9_1;
    m[1][2] *= W9_2;
    m[2][1] *= W9_2;
    m[2][2] *= W9_4;

    // Columns: 3-pt FFT down each `k1`, then natural order `k = 3·k2 + k1`.
    for k1 in 0..3 {
        let mut col = [m[0][k1], m[1][k1], m[2][k1]];
        fft_3(&mut col);
        for (k2, &v) in col.iter().enumerate() {
            x[3 * k2 + k1] = v;
        }
    }
}

/// Forward 2304-pt complex FFT, in-place. Caller supplies a 256-pt
/// in-place forward FFT closure so the same wrapper works on host
/// (rustfft) and embedded (esp-dsp).
///
/// `twiddles` must be [`build_twiddles`]'s table, of length `N`, with
/// `twiddles[n2 * 256 + k1] = ω₂₃₀₄^(n2 · k1)`. Build it once at
/// startup and cache it.
pub fn fft_2304_with(
    buf: &mut [Complex32; N],
    fft_256: &mut dyn FnMut(&mut [Complex32; N1]),
    twiddles: &[Complex32; N],
) {
    // Step 1: `m[n2][n1] = x[9·n1 + n2]`, transposed so each row is
    //         contiguous for the 256-pt FFT.
    let mut m: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); N];
    for n1 in 0..N1 {
        for n2 in 0..N2 {
            m[n2 * N1 + n1] = buf[N2 * n1 + n2];
        }
    }

    // Step 2: 256-pt FFT along each of the 9 rows.
    for n2 in 0..N2 {
        let row: &mut [Complex32; N1] = (&mut m[n2 * N1..(n2 + 1) * N1])
            .try_into()
            .expect("row slice = N1 elements");
        fft_256(row);
    }

    // Step 3: twiddle by ω₂₃₀₄^(n2·k1).
    for n2 in 0..N2 {
        for k1 in 0..N1 {
            m[n2 * N1 + k1] *= twiddles[n2 * N1 + k1];
        }
    }

    // Step 4: 9-pt FFT along each of the 256 columns.
    let mut col = [Complex32::new(0.0, 0.0); N2];
    for k1 in 0..N1 {
        for k2 in 0..N2 {
            col[k2] = m[k2 * N1 + k1];
        }
        fft_9(&mut col);
        for k2 in 0..N2 {
            m[k2 * N1 + k1] = col[k2];
        }
    }

    // Step 5: natural-order write-back, `k = 256·k2 + k1`.
    for k2 in 0..N2 {
        for k1 in 0..N1 {
            buf[N1 * k2 + k1] = m[k2 * N1 + k1];
        }
    }
}

/// Unnormalised inverse 2304-pt complex FFT, in-place, via
/// `IDFT(x) = conj(DFT(conj(x)))`.
///
/// `fft_256` must still be a **forward** 256-pt FFT — the conjugation
/// wraps the whole transform, so every stage inside runs forward.
pub fn ifft_2304_with(
    buf: &mut [Complex32; N],
    fft_256: &mut dyn FnMut(&mut [Complex32; N1]),
    twiddles: &[Complex32; N],
) {
    for c in buf.iter_mut() {
        c.im = -c.im;
    }
    fft_2304_with(buf, fft_256, twiddles);
    for c in buf.iter_mut() {
        c.im = -c.im;
    }
}

/// Build the inter-stage twiddle table: `twiddles[n2 * 256 + k1] =
/// exp(-j · 2π · n2 · k1 / 2304)`.
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

    fn rustfft_n(input: &[Complex32], forward: bool) -> Vec<Complex32> {
        use rustfft::FftPlanner;
        let mut planner = FftPlanner::<f32>::new();
        let fft = if forward {
            planner.plan_fft_forward(input.len())
        } else {
            planner.plan_fft_inverse(input.len())
        };
        let mut buf: Vec<Complex32> = input.to_vec();
        fft.process(&mut buf);
        buf
    }

    fn rustfft_256(buf: &mut [Complex32; N1]) {
        let out = rustfft_n(buf, true);
        buf.copy_from_slice(&out);
    }

    fn random_input(seed: u64, n: usize) -> Vec<Complex32> {
        let mut s = seed;
        let mut next = || {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (s >> 33) as f32 / (1u32 << 31) as f32 - 1.0
        };
        (0..n).map(|_| Complex32::new(next(), next())).collect()
    }

    fn assert_close(got: &[Complex32], want: &[Complex32], eps: f32, what: &str) {
        for (k, (a, b)) in got.iter().zip(want.iter()).enumerate() {
            assert!(
                (a.re - b.re).abs() < eps && (a.im - b.im).abs() < eps,
                "{what}: bin {k} got {a:?} want {b:?}"
            );
        }
    }

    /// The 9-pt sub-kernel on its own, against rustfft — the piece the
    /// 2304 path adds over `fft_mixed_5120`, and the only genuinely new
    /// arithmetic in this module.
    #[test]
    fn fft9_matches_rustfft() {
        for seed in [3u64, 0xfeed_face, 11] {
            let x = random_input(seed, 9);
            let want = rustfft_n(&x, true);
            let mut got = [Complex32::new(0.0, 0.0); 9];
            got.copy_from_slice(&x);
            fft_9(&mut got);
            assert_close(&got, &want, 1e-5, "fft_9");
        }
    }

    #[test]
    fn fft2304_impulse_is_flat() {
        let mut x = [Complex32::new(0.0, 0.0); N];
        x[0] = Complex32::new(1.0, 0.0);
        let tw = build_twiddles();
        fft_2304_with(&mut x, &mut rustfft_256, &tw);
        for (k, c) in x.iter().enumerate() {
            assert!(
                (c.re - 1.0).abs() < 1e-5 && c.im.abs() < 1e-5,
                "bin {k}: {c:?}"
            );
        }
    }

    #[test]
    fn fft2304_matches_rustfft_forward() {
        let tw = build_twiddles();
        for seed in [1u64, 0xdead_beef, 42] {
            let x = random_input(seed, N);
            let want = rustfft_n(&x, true);
            let mut got = [Complex32::new(0.0, 0.0); N];
            got.copy_from_slice(&x);
            fft_2304_with(&mut got, &mut rustfft_256, &tw);
            assert_close(&got, &want, 1e-2, "forward");
        }
    }

    #[test]
    fn ifft2304_matches_rustfft_inverse() {
        let tw = build_twiddles();
        for seed in [7u64, 0x0bad_c0de] {
            let x = random_input(seed, N);
            let want = rustfft_n(&x, false);
            let mut got = [Complex32::new(0.0, 0.0); N];
            got.copy_from_slice(&x);
            ifft_2304_with(&mut got, &mut rustfft_256, &tw);
            assert_close(&got, &want, 1e-2, "inverse");
        }
    }

    /// A single tone lands in exactly one bin — the property
    /// `ft4_coarse_sync`'s periodogram actually depends on, and the one
    /// a transposition bug in step 1 or step 5 would break while
    /// leaving the impulse test above green.
    #[test]
    fn fft2304_puts_a_tone_in_one_bin() {
        let tw = build_twiddles();
        for bin in [1usize, 7, 255, 256, 257, 1000, 2303] {
            let mut x = [Complex32::new(0.0, 0.0); N];
            for (n, c) in x.iter_mut().enumerate() {
                let phi = TAU * (bin as f32) * (n as f32) / (N as f32);
                *c = Complex32::new(phi.cos(), phi.sin());
            }
            fft_2304_with(&mut x, &mut rustfft_256, &tw);
            let peak = x
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.norm_sqr().total_cmp(&b.1.norm_sqr()))
                .map(|(k, _)| k)
                .unwrap();
            assert_eq!(peak, bin, "tone at bin {bin} peaked at {peak}");
            assert!(
                (x[bin].norm() - N as f32).abs() < 1.0,
                "bin {bin} magnitude {} != {N}",
                x[bin].norm()
            );
        }
    }
}
