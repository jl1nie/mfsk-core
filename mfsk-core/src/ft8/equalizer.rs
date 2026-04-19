// SPDX-License-Identifier: GPL-3.0-or-later
//! Adaptive equaliser — thin wrapper over [`crate::core::equalize`].
//!
//! Preserves the pre-refactor `equalize_local(&mut [[Complex;8];79])`
//! signature expected by `decode`. The underlying algorithm (Wiener
//! regularisation driven by per-tone Costas observations) is protocol-
//! agnostic; FT8's tone-7 extrapolation becomes a degenerate linear
//! extrapolation when any tone is unobserved.

use super::Ft8;
use num_complex::Complex;

pub use crate::core::equalize::EqMode;

/// Apply FT8 local equalisation in place.
#[inline]
pub fn equalize_local(cs: &mut [[Complex<f32>; 8]; 79]) {
    // Flatten → generic apply → inflate back.
    let mut flat: Vec<Complex<f32>> = cs.iter().flatten().copied().collect();
    crate::core::equalize::equalize_local::<Ft8>(&mut flat);
    for (k, row) in cs.iter_mut().enumerate() {
        for t in 0..8 {
            row[t] = flat[k * 8 + t];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::params::{COSTAS, COSTAS_POS};
    use std::f32::consts::PI;

    #[test]
    fn flat_channel_is_noop() {
        let mut cs = [[Complex::new(0.0f32, 0.0); 8]; 79];
        for &offset in &COSTAS_POS {
            for (k, &tone) in COSTAS.iter().enumerate() {
                cs[offset + k][tone] = Complex::new(1.0, 0.0);
            }
        }
        for sym in 0..79 {
            for t in 0..8 {
                if cs[sym][t] == Complex::new(0.0, 0.0) {
                    cs[sym][t] = Complex::new(1.0, 0.0);
                }
            }
        }
        let orig = cs;
        equalize_local(&mut cs);
        for sym in 0..79 {
            for t in 0..8 {
                let ratio = cs[sym][t].norm() / orig[sym][t].norm().max(1e-10);
                assert!((ratio - 1.0).abs() < 0.1, "sym={sym} t={t}: ratio={ratio:.3}");
            }
        }
    }

    #[test]
    fn edge_attenuation_corrected() {
        let mut cs = [[Complex::new(0.0f32, 0.0); 8]; 79];
        let h: [f32; 8] = [1.0, 1.0, 1.0, 1.0, 1.0, 0.7, 0.5, 0.3];
        for sym in 0..79 {
            for t in 0..8 {
                cs[sym][t] = Complex::new(h[t], 0.0);
            }
        }
        let mags_before: Vec<f32> = (0..8).map(|t| cs[40][t].norm()).collect();
        let mean_before = mags_before.iter().sum::<f32>() / 8.0;
        let cv_before = {
            let v = mags_before.iter().map(|&m| (m - mean_before).powi(2)).sum::<f32>() / 8.0;
            v.sqrt() / mean_before
        };
        equalize_local(&mut cs);
        let mags_after: Vec<f32> = (0..8).map(|t| cs[40][t].norm()).collect();
        let mean_after = mags_after.iter().sum::<f32>() / 8.0;
        let cv_after = {
            let v = mags_after.iter().map(|&m| (m - mean_after).powi(2)).sum::<f32>() / 8.0;
            v.sqrt() / mean_after
        };
        assert!(cv_after < cv_before);
    }

    #[test]
    fn phase_distortion_corrected() {
        let mut cs = [[Complex::new(0.0f32, 0.0); 8]; 79];
        let phases: [f32; 8] = [0.0, 0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 1.6];
        for sym in 0..79 {
            for t in 0..8 {
                let mag = 1.0;
                cs[sym][t] = Complex::new(mag * phases[t].cos(), mag * phases[t].sin());
            }
        }
        equalize_local(&mut cs);
        let ref_phase = cs[40][0].arg();
        for t in 1..7 {
            let phase_diff = (cs[40][t].arg() - ref_phase).abs();
            let phase_diff = phase_diff.min(2.0 * PI - phase_diff);
            assert!(phase_diff < 0.15, "tone {t}: phase diff={phase_diff:.3} rad");
        }
    }
}
