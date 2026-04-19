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

use crate::core::ModulationParams;

use super::spectrogram::{score_candidate, Spectrogram};
use super::Wspr;

/// A candidate WSPR alignment, ranked by its sync-vector correlation.
#[derive(Clone, Copy, Debug)]
pub struct SyncCandidate {
    pub start_sample: usize,
    pub freq_hz: f32,
    pub score: f32,
}

/// Default sync-score threshold for [`super::rx::sync_score`]. Pure
/// noise scores ≈ 0; misaligned candidates land near 0 or negative;
/// an aligned frame at +3 dB SNR scores ≈ 0.3 and climbs toward 1.0 as
/// SNR rises. 0.1 leaves headroom for low-SNR real recordings while
/// still filtering out clearly-empty candidates.
pub const DEFAULT_SCORE_THRESHOLD: f32 = 0.1;

/// Search space + ranking controls. All fields have sensible defaults
/// pushed in via `Default`.
#[derive(Clone, Copy, Debug)]
pub struct SearchParams {
    /// Inclusive lower bound of the base-frequency sweep (Hz).
    pub freq_min_hz: f32,
    /// Inclusive upper bound of the base-frequency sweep (Hz).
    pub freq_max_hz: f32,
    /// How far to push the start_sample around a nominal t=0 anchor, in
    /// symbols. WSJT-X tolerates ±2 s → ~3 symbols at the edges.
    pub time_tolerance_symbols: u32,
    /// Minimum `sync_score` to accept. See [`DEFAULT_SCORE_THRESHOLD`].
    pub score_threshold: f32,
    /// Upper bound on candidates returned (top-N by score).
    pub max_candidates: usize,
}

impl Default for SearchParams {
    fn default() -> Self {
        Self {
            freq_min_hz: 1400.0,
            freq_max_hz: 1600.0,
            // Real WSPR TX starts ~1 s into the 120-s slot (and can drift).
            // The signal is 110.6 s long, leaving ≈ 9.4 s of slack. 8
            // symbols ≈ 5.5 s covers the common case without blowing up
            // the candidate count.
            time_tolerance_symbols: 8,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates: 16,
        }
    }
}

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
    let df = sample_rate as f32 / nsps as f32;
    let rows_per_symbol = 4usize;

    let t_span_rows = params.time_tolerance_symbols as i64 * rows_per_symbol as i64;
    let nominal_row = (nominal_start_sample / spec.t_step) as i64;
    let row_min = (nominal_row - t_span_rows).max(0);
    let row_max = nominal_row + t_span_rows;

    let fmin_bin = (params.freq_min_hz / df).floor() as i64;
    let fmax_bin = (params.freq_max_hz / df).ceil() as i64;

    let mut out: Vec<SyncCandidate> = Vec::new();

    for row in row_min..=row_max {
        if row < 0 {
            continue;
        }
        let row = row as usize;
        // Need room for 162 symbols → 161 * 4 rows of lookahead.
        if row + 161 * rows_per_symbol >= spec.n_time {
            continue;
        }

        for fb in fmin_bin..=fmax_bin {
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

    out.sort_unstable_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
    out.truncate(params.max_candidates);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::synthesize_type1;

    #[test]
    fn finds_aligned_tone_at_nominal_anchor() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3)
            .expect("synth");
        let params = SearchParams::default();
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
        let body =
            synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.3).expect("synth");
        audio.extend_from_slice(&body);

        let params = SearchParams::default();
        // Nominal anchor at 0; search tolerance ±4 symbols covers +3.
        let cands = coarse_search(&audio, 12_000, 0, &params);
        assert!(!cands.is_empty(), "expected candidates with offset signal");
        let best = cands[0];
        assert_eq!(
            best.start_sample, 3 * 8192,
            "best candidate should land at 3-symbol offset"
        );
    }
}
