//! Coarse (frequency, time) sync search.
//!
//! Scans a grid of candidate alignments and ranks them by how well the
//! per-symbol tone powers match the known WSPR sync vector. Candidates
//! above a threshold are handed to the Fano decoder one by one; the
//! first one that unpacks to a plausible message wins.
//!
//! ## Strategy
//!
//! 1. Step the starting sample in **quarter-symbol** increments (NSPS/4).
//!    At 12 kHz that's 2048 samples = ~171 ms — enough resolution for
//!    WSPR's 683 ms symbol window without going quadratic.
//! 2. Step the base frequency in **one-bin** increments (`tone_spacing`
//!    = 1.4648 Hz at 12 kHz). Finer than that buys nothing with the
//!    single-symbol FFT we use downstream.
//! 3. For each (t, f), compute `sync_score` (see [`super::rx`]); keep
//!    the top N.
//! 4. Return candidates sorted by score descending.
//!
//! A future refinement will promote top-K candidates to a fine search
//! (sub-bin freq + sub-quarter-symbol time via parabolic interpolation
//! on the score grid). Not required for typical crowded-band decoding.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::ModulationParams;

use super::Wspr;
use super::spectrogram::{Spectrogram, score_candidate};

pub use crate::engine::search::{DEFAULT_SCORE_THRESHOLD, SearchParams, SyncCandidate};
use crate::engine::search::{SearchWindow, rank_and_truncate};

/// WSPR's own coarse-search defaults.
///
/// Real WSPR TX starts ~1 s into the 120-s slot (and can drift). The
/// signal is 110.6 s long, leaving ≈ 9.4 s of slack. 8 symbols ≈ 5.5 s
/// covers the common case without blowing up the candidate count.
///
/// WSPR counted that tolerance in **symbols** until #394, the last
/// mode still doing so; it is seconds now like every other mode, and
/// `8 × SYMBOL_DT` is the same window to the row (WSPR's spectrogram
/// steps at NSPS/4, so 8 symbols is exactly 32 rows either way).
///
/// `max_candidates` is wsprd's own cap (`wsprd.c:1088`, `npk < 200`).
/// 16 was far too tight for a busy band: the coarse search ranks by
/// sync, and on an 8-signal recording the strong stations plus noise
/// peaks fill the list long before a -23 dB signal gets a look in.
pub fn default_search_params() -> SearchParams {
    SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1600.0,
        time_tolerance_early_sec: WSPR_TIME_TOLERANCE_SEC,
        time_tolerance_late_sec: WSPR_TIME_TOLERANCE_SEC,
        score_threshold: DEFAULT_SCORE_THRESHOLD,
        max_candidates: 200,
    }
}

/// 8 symbols, the window WSPR searched when it counted symbols.
const WSPR_TIME_TOLERANCE_SEC: f32 = 8.0 * <Wspr as ModulationParams>::SYMBOL_DT;

/// Sweep (freq, time) grid and return top-ranked candidates.
///
/// Builds a single quarter-symbol spectrogram (~700 FFTs for a 120-s
/// slot) and scores each (time_row, base_bin) in O(162) lookups, so
/// total work is ~FFT_build + grid_size, independent of how fine the
/// search grid is. Empty or below-threshold alignments are dropped.
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = Spectrogram::build(audio, sample_rate);
    coarse_search_on_spec(&spec, sample_rate, nominal_start_sample, params)
}

/// Variant that reuses a pre-built spectrogram. Useful when a caller
/// decodes multiple slots of the same audio pipeline or wants to share
/// the FFT cost across additional post-processing (waterfall display,
/// etc).
pub fn coarse_search_on_spec(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * <Wspr as ModulationParams>::SYMBOL_DT).round() as usize;
    let w = SearchWindow::new(spec.t_step, sample_rate, nominal_start_sample, nsps, params);
    let df = w.df;
    let rows_per_symbol = 4usize;

    let mut out: Vec<SyncCandidate> = Vec::new();

    for row in w.row_min..=w.row_max {
        if row < 0 {
            continue;
        }
        let row = row as usize;
        // Need room for 162 symbols → 161 * 4 rows of lookahead.
        if row + 161 * rows_per_symbol >= spec.n_time {
            continue;
        }

        for fb in w.fmin_bin..=w.fmax_bin {
            if fb < 0 {
                continue;
            }
            let base_bin = fb as usize;
            if base_bin + 4 > spec.n_freq {
                continue;
            }
            let score = score_candidate(spec, row, base_bin);
            if score >= params.score_threshold {
                out.push(SyncCandidate {
                    start_sample: row * spec.t_step,
                    freq_hz: fb as f32 * df,
                    score,
                });
            }
        }
    }

    rank_and_truncate(&mut out, params.max_candidates);
    out.truncate(params.max_candidates);
    out
}

#[cfg(test)]
mod tests {
    use super::super::synthesize_type1;
    use super::*;

    #[test]
    fn finds_aligned_tone_at_nominal_anchor() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let params = default_search_params();
        let cands = coarse_search(&audio, 12_000, 0, &params);
        assert!(!cands.is_empty(), "should find at least one candidate");
        let best = cands[0];
        // The freq-bin rounding at 12 kHz / 8192 bins = 1.4648 Hz; the
        // true 1500 Hz lands between bin 1023 (=1499.5 Hz) and 1024 (=1500.9 Hz).
        // Either is acceptable.
        assert!(
            (best.freq_hz - 1500.0).abs() <= 2.0,
            "best freq {} should be near 1500 Hz",
            best.freq_hz
        );
        assert_eq!(best.start_sample, 0, "alignment should land exactly at t=0");
        assert!(best.score > 0.9, "clean synthesis should score near 1.0");
    }

    #[test]
    fn finds_offset_start_within_tolerance() {
        // Synthesise a full WSPR frame plus 3 symbols of leading silence.
        let freq = 1500.0;
        let mut audio = vec![0f32; 3 * 8192];
        let body = synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.3).expect("synth");
        audio.extend_from_slice(&body);

        let params = default_search_params();
        // Nominal anchor at 0; search tolerance ±4 symbols covers +3.
        let cands = coarse_search(&audio, 12_000, 0, &params);
        assert!(!cands.is_empty(), "expected candidates with offset signal");
        let best = cands[0];
        assert_eq!(
            best.start_sample,
            3 * 8192,
            "best candidate should land at 3-symbol offset"
        );
    }
}
