//! FT8 synchronisation — public entry points for the host decode
//! pipeline (`decode_frame*` in `decode.rs`).
//!
//! Signatures preserve the pre-refactor shape so out-of-tree callers
//! keep working unchanged. Internals route through the WSJT-X-faithful
//! [`crate::ft8::decode_block`] port for [`coarse_sync`] (closes #40
//! — the protocol-generic [`crate::core::sync`] coarse-sync mis-
//! estimates the noise floor on busy FT8 bands), and through
//! [`crate::core::sync`] for the remaining fine-sync helpers.

use alloc::vec::Vec;

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
pub fn coarse_sync(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    _freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    // Route through decode_block's WSJT-X-faithful sync8.f90 port
    // (closes #40). The protocol-generic core::sync::coarse_sync<Ft8>
    // computed the noise reference from same-time-slot non-Costas
    // tones, which over-estimates the floor on busy bands
    // (qso3_busy.wav lost 3 of 8 goldens including CQ F5RXL IN94 at
    // -3 dB SNR per #40) and under-estimates above 2 kHz (3 phantoms
    // above 2 kHz on the same WAV). decode_block uses the WSJT-X
    // 16-bin sliding-window allsum estimator that fixes both, at the
    // cost of building a Spectrogram first (one extra NFFT_SPEC=3840
    // FFT pass — same NFFT as before, just consolidated into a
    // reusable spec instead of recomputed per-candidate scoring loop).
    //
    // `freq_hint` is currently dropped — decode_block::coarse_sync
    // doesn't honour candidate-score promotion. The sniper paths that
    // use it (decode_sniper_ap) already restrict freq_min/max to a
    // ±250 Hz band around target_freq, so the loss is small. Tracked
    // as a follow-up.
    let spec = crate::ft8::decode_block::compute_spectrogram(audio, freq_max);
    crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand)
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
    fn coarse_sync_on_silence_returns_empty_or_low() {
        let audio = vec![0i16; 15 * 12000];
        let cands = coarse_sync(&audio, 200.0, 2800.0, 1.0, None, 100);
        assert!(cands.len() <= 100);
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
