//! FT8 synchronisation — fine-sync helpers for the host decode
//! pipeline (`decode_frame*` in `decode.rs`).
//!
//! `coarse_sync` no longer lives here — as of v0.6.0 the FT8 coarse-
//! sync stage is owned exclusively by [`crate::ft8::decode_block`]
//! (the WSJT-X `sync8.f90`-faithful port that already serves the
//! embedded path). The legacy thin wrapper that forwarded to
//! [`crate::core::sync::coarse_sync<crate::ft8::Ft8>`] was removed in
//! #48 step B; `decode.rs` now calls
//! [`crate::ft8::decode_block::compute_spectrogram`] +
//! [`crate::ft8::decode_block::coarse_sync`] directly.
//!
//! The remaining fine-sync helpers are still thin wrappers over the
//! protocol-generic [`crate::core::sync`] module — they have not
//! shown the busy-band recall gap that motivated the coarse-sync
//! consolidation, so the wrappers stay until either #49 cat B
//! decides to delete them or option A ([`Protocol::Sync`] associated
//! type, post-0.6.0) absorbs them.

use super::Ft8;
use num_complex::Complex;

pub use crate::core::sync::{
    FineSyncDetail as GenericFineSyncDetail, SyncCandidate, make_costas_ref, parabolic_peak,
    score_costas_block,
};

/// Per-array FT8 fine-sync detail. Matches the pre-refactor field set (three
/// fixed Costas arrays) by projecting the generic per-block scores.
#[derive(Debug, Clone)]
pub struct FineSyncDetail {
    pub candidate: SyncCandidate,
    pub score_a: f32,
    pub score_b: f32,
    pub score_c: f32,
    pub drift_dt_sec: f32,
}

impl From<GenericFineSyncDetail> for FineSyncDetail {
    fn from(g: GenericFineSyncDetail) -> Self {
        let mut it = g.per_block_scores.into_iter();
        Self {
            candidate: g.candidate,
            score_a: it.next().unwrap_or(0.0),
            score_b: it.next().unwrap_or(0.0),
            score_c: it.next().unwrap_or(0.0),
            drift_dt_sec: g.drift_dt_sec,
        }
    }
}

#[inline]
pub fn compute_spectra(audio: &[i16]) -> crate::core::sync::Spectrogram {
    crate::core::sync::compute_spectra::<Ft8>(audio)
}

#[inline]
pub fn fine_sync_power(cd0: &[Complex<f32>], i0: usize) -> f32 {
    crate::core::sync::fine_sync_power::<Ft8>(cd0, i0)
}

/// Backwards-compatible tuple form: (array_1, array_2, array_3).
#[inline]
pub fn fine_sync_power_split(cd0: &[Complex<f32>], i0: usize) -> (f32, f32, f32) {
    let scores = crate::core::sync::fine_sync_power_per_block::<Ft8>(cd0, i0);
    (
        scores.first().copied().unwrap_or(0.0),
        scores.get(1).copied().unwrap_or(0.0),
        scores.get(2).copied().unwrap_or(0.0),
    )
}

#[inline]
pub fn refine_candidate(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    search_steps: i32,
) -> SyncCandidate {
    crate::core::sync::refine_candidate::<Ft8>(cd0, candidate, search_steps)
}

#[inline]
pub fn refine_candidate_double(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    search_steps: i32,
) -> FineSyncDetail {
    crate::core::sync::refine_candidate_double::<Ft8>(cd0, candidate, search_steps).into()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parabolic_peak_at_center() {
        let (offset, _) = parabolic_peak(1.0, 2.0, 1.0);
        assert!(offset.abs() < 1e-6);
    }

    #[test]
    fn parabolic_peak_offset_right() {
        let (offset, _) = parabolic_peak(0.5, 1.5, 2.0);
        assert!(offset > 0.0);
    }

    #[test]
    fn fine_sync_silence_is_zero() {
        let cd0 = vec![Complex::new(0.0f32, 0.0); 3200];
        let sync = fine_sync_power(&cd0, 0);
        assert_eq!(sync, 0.0);
    }

    #[test]
    fn fine_sync_split_silence_is_zero() {
        let cd0 = vec![Complex::new(0.0f32, 0.0); 3200];
        let (sa, sb, sc) = fine_sync_power_split(&cd0, 0);
        assert_eq!(sa, 0.0);
        assert_eq!(sb, 0.0);
        assert_eq!(sc, 0.0);
    }

    #[test]
    fn fine_sync_split_sum_equals_total() {
        let mut cd0 = vec![Complex::new(0.0f32, 0.0); 3200];
        for (i, c) in cd0.iter_mut().enumerate() {
            let t = i as f32 / 200.0;
            c.re = (2.0 * std::f32::consts::PI * 50.0 * t).cos() * 100.0;
        }
        let total = fine_sync_power(&cd0, 100);
        let (sa, sb, sc) = fine_sync_power_split(&cd0, 100);
        let diff = (total - (sa + sb + sc)).abs();
        assert!(diff < 1e-3);
    }

    #[test]
    fn refine_candidate_double_silence_no_panic() {
        let cd0 = vec![Complex::new(0.0f32, 0.0); 3200];
        let cand = SyncCandidate {
            freq_hz: 1000.0,
            dt_sec: 0.0,
            score: 1.0,
        };
        let detail = refine_candidate_double(&cd0, &cand, 5);
        assert!(detail.drift_dt_sec.is_finite());
    }
}
