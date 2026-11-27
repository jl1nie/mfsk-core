// SPDX-License-Identifier: GPL-3.0-only
//! A gate before belief propagation for the cells flow: from the statistics of a candidate's first LLR vector (`llra`) and its search score and hard-sync count, a logistic
//! model says whether the candidate is worth a BP run at all; a candidate under the threshold is not decoded. **A deliberate divergence from WSJT-X** (`ft8b` runs BP on every
//! candidate it keeps), like the cells flow itself, **and a learned one**: the coefficients and the threshold are fitted to data (see [`CELLS_GATE`]).
//!
//! It only skips candidates: it cannot create a decode (or a phantom), and a skipped candidate costs one LLR and the statistics instead of up to four BP calls
//! (about 40 000 host instructions a pass, 20-30 passes a failing call). It is meant for flows **without OSD** (the board's): OSD rescues candidates BP fails on, which the
//! fit never saw, so [`BpGate::passes`] is not consulted when the depth has OSD on.

use crate::engine::scalar::LlrScalar;

/// The candidates' mean, standard deviation and maximum of `|llra|` and the model on them: `logit = bias + sum(coefficient * input)` over
/// `[mean, std, max, mean / std, search score, hard sync]`; a candidate is decoded only if `logit >= min_logit`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BpGate {
    /// The logit's constant term.
    pub bias: f32,
    /// The coefficients of `mean(|llra|)`, `std(|llra|)`, `max(|llra|)`, `mean / std` (`std` floored at 0.001), the candidate's search score and its hard-sync count.
    pub coef: [f32; 6],
    /// The lowest logit that is still decoded.
    pub min_logit: f32,
}

impl BpGate {
    /// The model's logit for a candidate: `llra` is its first LLR vector (any length: 174 for FT8), `score` the search score, `hs` the hard-sync count (of 21).
    pub fn logit<T: LlrScalar>(&self, llra: &[T], score: f32, hs: u8) -> f32 {
        let n = llra.len() as f32;
        let (mut sum, mut max) = (0.0f32, 0.0f32);
        for &v in llra {
            let a = v.to_f32().abs();
            sum += a;
            if a > max {
                max = a;
            }
        }
        let mean = sum / n;
        let mut var = 0.0f32;
        for &v in llra {
            let d = v.to_f32().abs() - mean;
            var += d * d;
        }
        let std = (var / n).sqrt();
        let x = [mean, std, max, mean / std.max(1e-3), score, hs as f32];
        let mut z = self.bias;
        for (c, v) in self.coef.iter().zip(x) {
            z += c * v;
        }
        z
    }

    /// Whether the candidate goes on to BP.
    pub fn passes<T: LlrScalar>(&self, llra: &[T], score: f32, hs: u8) -> bool {
        self.logit(llra, score, hs) >= self.min_logit
    }
}

/// The cells flow's gate, fitted by `scripts/cands-gate-analysis.py` (the cheap model: no sort) on the candidates of 4 720 generated slots (`busy` 420, `dense` 200, `sweep` 1 040,
/// `itu` 3 060: single-signal SNR ladders and busy slots with known messages; the first LLR vector of each candidate that reached BP, 106 654 of them, 6 931 decoding) with the 60 real slots
/// (22 recordings against `jt9`, 38 `ft8_lib` `20m_busy` against their lists) **held out**, the threshold being the lowest logit of any decode in the generated sets **minus 0.5**. On the
/// real slots: 0 of 974 decodes lost, 24.4 % of the BP passes saved; on the generated ones 79.7 %. Leave-one-family-out (a family held out of the fit): `busy` 2 of 1 677 lost at the
/// zero-margin threshold, `dense` / `sweep` / `itu` none. The fit is a 6-input logistic regression, on the data of the day it was made (2026-11-27); **refit it if the search, the
/// LLR or the staircase changes**.
pub const CELLS_GATE: BpGate = BpGate {
    bias: 6.18511,
    coef: [-3.08844, -4.3783, -0.467705, -0.143884, 0.0699579, 0.670358],
    min_logit: -5.6599 - 0.5,
};

#[cfg(test)]
mod tests {
    use super::*;

    /// The logit is `bias + coef . [mean, std, max, mean / std, score, hs]` over the magnitudes of the LLRs. **Fails if** a feature is computed on the wrong quantity (the
    /// signed LLR instead of its magnitude, a sample instead of a population standard deviation, `std` not floored) or the coefficients are applied to the wrong inputs.
    #[test]
    fn the_logit_is_the_model_on_the_llr_statistics() {
        // 4 values: magnitudes 1, 3, 1, 3 (signs mixed): mean 2, population std 1, max 3, mean / std 2
        let llra = [-1.0f32, 3.0, 1.0, -3.0];
        let g = BpGate {
            bias: 0.5,
            coef: [1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0],
            min_logit: 0.0,
        };
        let z = g.logit(&llra, 7.0, 3);
        assert!(
            (z - (0.5 + 2.0 + 10.0 + 300.0 + 2000.0 + 70000.0 + 300000.0)).abs() < 1e-2,
            "{z}"
        );
        // all-equal magnitudes: std 0 is floored (mean / 0.001), not divided by zero
        let flat = [2.0f32; 8];
        let z = BpGate {
            bias: 0.0,
            coef: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            min_logit: 0.0,
        }
        .logit(&flat, 0.0, 0);
        assert!((z - 2000.0).abs() < 1.0, "{z}");
        assert!(
            g.passes(&llra, 7.0, 3)
                && !BpGate {
                    min_logit: 1e9,
                    ..g
                }
                .passes(&llra, 7.0, 3)
        );
    }
}
