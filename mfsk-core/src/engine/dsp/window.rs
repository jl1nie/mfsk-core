//! Cosine-sum windows, one place for the coefficient tables and the two ways
//! of laying the phase out (#425).
//!
//! A window here is `Σ_j c_j · cos(j · 2π · k / D)` with `D = n − 1`
//! ([`Form::Symmetric`]) or `D = n` ([`Form::Periodic`]). The form is not a free
//! choice: it is whatever the upstream routine the call site ports uses.
//! `getcandidates4.f90` / `nuttal_window.f90` take the symmetric Nuttall window;
//! `get_spectrum_baseline.f90` takes the periodic one, so a spectrum with no
//! leakage at the first bin gets the same shape as its neighbours.
//!
//! The two forms round differently, and each is written to reproduce the
//! expression its first call site used, bit for bit: a window feeds a spectrum
//! whose peak ranks candidates, and a last-bit change there is a decode-output
//! change to be measured, not a refactor.
//!
//! **Not here, on purpose.** Three windows in the crate are cosine sums too, and
//! each lays its phase out a third way, so moving them onto this function
//! changes last bits (measured 2026-10-09: 70 of 512 entries for FST4's
//! Blackman-Harris in `fst4::baseline`, by one f32 step; up to 3e-7 for the
//! Blackman in `fir_decimate::design_lowpass`, which sets the DDC filters'
//! taps): they feed a noise-floor estimate whose divisor is a literal, and a
//! filter design pinned by equivalence tests. The two raised-cosine edges
//! (`downsample`'s taper, `msk144::spd`'s `rcw`) are mirror images of each other
//! with different rounding. Each keeps its own few lines, and this note.

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::TAU;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// Which end the period closes at.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Form {
    /// First and last sample are the window's two zeros: `D = n − 1`.
    Symmetric,
    /// The window is one period of a longer sequence: `D = n`.
    // Only `ft8::baseline` asks for it, and that is not compiled in every
    // feature set that has this module (the matrix's `alloc ft8 fft-extern`
    // row, for one).
    #[allow(dead_code)]
    Periodic,
}

/// Nuttall 4-term (continuous first derivative), with the sign of each cosine
/// folded into its coefficient, as `nuttal_window.f90` lists them.
pub(crate) const NUTTALL4: [f32; 4] = [0.363_581_9, -0.489_177_5, 0.136_599_5, -0.010_641_1];

/// `Σ_j coeffs[j] · cos(j · 2π · k / D)` for `k = 0..n`.
///
/// `n == 0` is empty. `n == 1` is `[1.0]` for the symmetric form (its
/// denominator `n − 1` would be zero); the periodic form has no such problem
/// and gives the formula's value at `k = 0`, as its ported loop does.
pub(crate) fn cosine_sum(n: usize, coeffs: &[f32], form: Form) -> Vec<f32> {
    let mut w = vec![0.0f32; n];
    if form == Form::Symmetric && n < 2 {
        if n == 1 {
            w[0] = 1.0;
        }
        return w;
    }
    let denom = match form {
        Form::Symmetric => (n - 1) as f32,
        Form::Periodic => n as f32,
    };
    for (k, slot) in w.iter_mut().enumerate() {
        let mut acc = coeffs[0];
        for (j, c) in coeffs.iter().enumerate().skip(1) {
            let jtau = j as f32 * TAU;
            let phase = match form {
                // `x = k / (n−1)` first: how `engine::sync` has always spelled it.
                Form::Symmetric => jtau * (k as f32 / denom),
                // `(j·2π·k) / n`: how `get_spectrum_baseline.f90` is ported.
                Form::Periodic => jtau * k as f32 / denom,
            };
            acc += c * phase.cos();
        }
        *slot = acc;
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    fn legacy_symmetric_nuttall(n: usize) -> Vec<f32> {
        const A0: f32 = 0.3635819;
        const A1: f32 = 0.4891775;
        const A2: f32 = 0.1365995;
        const A3: f32 = 0.0106411;
        let mut w = vec![0.0f32; n];
        let two_pi = 2.0 * core::f32::consts::PI;
        let denom = (n - 1) as f32;
        for (k, slot) in w.iter_mut().enumerate() {
            let x = k as f32 / denom;
            *slot = A0 - A1 * (two_pi * x).cos() + A2 * (2.0 * two_pi * x).cos()
                - A3 * (3.0 * two_pi * x).cos();
        }
        w
    }

    fn legacy_periodic_nuttall(n: usize) -> Vec<f32> {
        let (a0, a1, a2, a3) = (0.3635819f32, -0.4891775f32, 0.1365995f32, -0.0106411f32);
        let nf = n as f32;
        let two_pi = core::f32::consts::PI * 2.0;
        (0..n)
            .map(|i| {
                let x = i as f32;
                a0 + a1 * (two_pi * x / nf).cos()
                    + a2 * (2.0 * two_pi * x / nf).cos()
                    + a3 * (3.0 * two_pi * x / nf).cos()
            })
            .collect()
    }

    fn differing(a: &[f32], b: &[f32]) -> (usize, f32) {
        let mut d = 0;
        let mut worst = 0.0f32;
        for (x, y) in a.iter().zip(b) {
            if x.to_bits() != y.to_bits() {
                d += 1;
                worst = worst.max((x - y).abs());
            }
        }
        (d, worst)
    }

    /// The reason this module exists as it is: both Nuttall call sites got
    /// exactly the bits their own loops produced, at every length either of
    /// them uses (`NSPS` 576 / 1920 / 3840 and the FT4 coarse-sync `NFFT1` 2304
    /// among them).
    #[test]
    fn nuttall_forms_are_bit_identical_to_the_loops_they_replaced() {
        for n in [1usize, 2, 4, 144, 512, 576, 1920, 2304, 3840, 4096, 92_160] {
            if n >= 2 {
                assert_eq!(
                    differing(
                        &cosine_sum(n, &NUTTALL4, Form::Symmetric),
                        &legacy_symmetric_nuttall(n)
                    ),
                    (0, 0.0),
                    "symmetric, n = {n}"
                );
            }
            assert_eq!(
                differing(
                    &cosine_sum(n, &NUTTALL4, Form::Periodic),
                    &legacy_periodic_nuttall(n)
                ),
                (0, 0.0),
                "periodic, n = {n}"
            );
        }
    }

    #[test]
    fn degenerate_lengths() {
        assert!(cosine_sum(0, &NUTTALL4, Form::Symmetric).is_empty());
        assert_eq!(cosine_sum(1, &NUTTALL4, Form::Symmetric), vec![1.0]);
        // One point of a periodic window is its value at k = 0, not 1.
        assert_eq!(
            cosine_sum(1, &NUTTALL4, Form::Periodic),
            vec![NUTTALL4[0] + NUTTALL4[1] + NUTTALL4[2] + NUTTALL4[3]]
        );
    }
}
