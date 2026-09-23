//! Coarse candidate search, shared by the sample-indexed modes.
//!
//! JT9, JT65, Q65 and WSPR all find their frames the same way: build a
//! spectrogram, sweep a (time × frequency) window scoring a sync
//! pattern, then rank what survives and keep the best few. Before #394
//! each of the four carried its own copy of the candidate type, the
//! parameter block, the window arithmetic and the ranking tail — four
//! literal copies of each, drifting independently.
//!
//! What is *not* here is the part that genuinely differs. Each mode
//! still owns its own selection policy, because each mirrors a
//! different upstream routine:
//!
//! | mode | time collapse | admission | frequency |
//! |---|---|---|---|
//! | JT9 | best lag per bin (`sync9`'s `ccfred`) | fixed floor | log-power refined |
//! | JT65 | every cell | fixed floor | log-power refined |
//! | Q65 | best lag per bin (`q65_ccf_22`) | fixed floor **or** an adaptive percentile gate, then frequency local-max suppression | bin centre |
//! | WSPR | every cell | fixed floor | bin centre |
//!
//! Folding those four into one configuration-driven function was
//! considered and rejected: it needs four independent policy axes, and
//! a body switching on all four reads worse than the four straight-line
//! bodies it replaces. The clones are gone; the differences stay
//! visible, each next to the upstream line it ports.
//!
//! [`crate::engine::sync::SyncCandidate`] is a different type on
//! purpose. FT8/FT4/FST4 reach their candidates through a lag grid and
//! carry the time offset as `dt_sec`; the modes here index samples
//! directly. Two representations, not two copies — converting FT8's
//! path to samples would touch its hot loop for no behavioural gain.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

/// Default sync-score threshold, shared by every mode that scores
/// against [`crate::engine::spectrogram::score_candidate`].
///
/// Pure noise scores ≈ 0; a clean aligned frame scores ≈ 1 at high SNR.
/// 0.1 is a safely-loose prefilter that still drops most garbage. All
/// four modes had independently settled on this same value.
pub const DEFAULT_SCORE_THRESHOLD: f32 = 0.1;

/// One coarse-search candidate: where a frame might start, and how
/// strongly it scored.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SyncCandidate {
    /// Absolute sample index of symbol 0.
    pub start_sample: usize,
    /// Frequency of tone 0 — the sync tone, the low end of the
    /// constellation. Bin centre or sub-bin refined depending on the
    /// mode; see the table in this module's docs.
    pub freq_hz: f32,
    /// Normalised sync score; higher is better.
    pub score: f32,
}

impl SyncCandidate {
    /// Time offset from `nominal_start_sample`, in seconds — the
    /// convention [`crate::msg::decoded::Decoded`] reports.
    pub fn dt_sec(&self, nominal_start_sample: usize, sample_rate: u32) -> f32 {
        (self.start_sample as f32 - nominal_start_sample as f32) / sample_rate as f32
    }
}

/// Coarse-search parameter block.
///
/// The time tolerance is an early/late pair because Q65's window is
/// asymmetric (it reaches much further late than early). Symmetric
/// modes set both to the same value.
///
/// Seconds rather than symbols (issue #282): the symbol-denominated
/// form is what let Q65's equivalent window silently become a
/// different span per sub-mode. WSPR was the last mode still counting
/// symbols and converts in its `Default` (#394).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SearchParams {
    pub freq_min_hz: f32,
    pub freq_max_hz: f32,
    /// How far *before* the nominal start to look, in seconds.
    pub time_tolerance_early_sec: f32,
    /// How far *after* the nominal start to look, in seconds.
    pub time_tolerance_late_sec: f32,
    pub score_threshold: f32,
    pub max_candidates: usize,
}

impl SearchParams {
    /// Symmetric window of `tolerance_sec` either side of nominal.
    pub const fn symmetric(
        freq_min_hz: f32,
        freq_max_hz: f32,
        tolerance_sec: f32,
        max_candidates: usize,
    ) -> Self {
        Self {
            freq_min_hz,
            freq_max_hz,
            time_tolerance_early_sec: tolerance_sec,
            time_tolerance_late_sec: tolerance_sec,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates,
        }
    }
}

/// The (row × bin) rectangle a coarse search sweeps, plus the bin
/// width it was computed with.
///
/// Every mode derived these five numbers the same way from its params
/// and spectrogram. `row_max` is deliberately *not* clamped to
/// `n_time`: the caller's own lookahead bound (how many rows the frame
/// needs past `row_min`) depends on its symbol count, so it stays with
/// the caller.
#[derive(Clone, Copy, Debug)]
pub struct SearchWindow {
    /// First spectrogram row to try (clamped at 0).
    pub row_min: i64,
    /// Last spectrogram row to try, inclusive.
    pub row_max: i64,
    /// First frequency bin to try; may be negative, callers skip those.
    pub fmin_bin: i64,
    /// Last frequency bin to try, inclusive.
    pub fmax_bin: i64,
    /// Spectrogram bin width in Hz.
    pub df: f32,
}

impl SearchWindow {
    /// Derive the window from a params block.
    ///
    /// `nsps` is the mode's samples-per-symbol at `sample_rate`; `df`
    /// follows from it as `sample_rate / nsps`.
    /// `t_step` is the spectrogram's sample stride per row. It is
    /// passed rather than the spectrogram itself because WSPR carries
    /// its own `Spectrogram` type (it adds the per-bin baseline the
    /// wsprd-style scorer needs), and this arithmetic does not care.
    pub fn new(
        t_step: usize,
        sample_rate: u32,
        nominal_start_sample: usize,
        nsps: usize,
        params: &SearchParams,
    ) -> Self {
        let df = sample_rate as f32 / nsps as f32;
        let rows_per_sec = sample_rate as f32 / t_step.max(1) as f32;
        let early_rows = (params.time_tolerance_early_sec.max(0.0) * rows_per_sec).round() as i64;
        let late_rows = (params.time_tolerance_late_sec.max(0.0) * rows_per_sec).round() as i64;
        let nominal_row = (nominal_start_sample / t_step.max(1)) as i64;
        Self {
            row_min: (nominal_row - early_rows).max(0),
            row_max: nominal_row + late_rows,
            fmin_bin: (params.freq_min_hz / df).floor() as i64,
            fmax_bin: (params.freq_max_hz / df).ceil() as i64,
            df,
        }
    }
}

/// Highest-scoring lag for one frequency bin, over `window`'s rows.
///
/// The time-collapse JT9 and Q65 both do — JT9 mirroring `sync9`'s
/// `ccfred(i) = max over lags`, Q65 mirroring `q65_ccf_22`'s
/// `ccfmax = max over lag,idrift`. `row_fits` is the caller's own
/// lookahead bound, which depends on its symbol count.
///
/// Returns `None` when no row in the window fits.
///
/// `#[inline]` is load-bearing, not decoration: without it Q65's golden
/// decode ran 1.35 s -> 1.69 s (+25%, medians of 7, ranges disjoint) when
/// this replaced its inline loop. Q65 has by far the largest search
/// rectangle of the four callers, so an un-inlined `score_at` call per
/// (row, bin) cell shows up there first. JT9, whose rectangle is much
/// smaller, moved +2.9% — inside the noise.
#[inline]
pub fn best_lag_in_bin(
    window: &SearchWindow,
    bin: usize,
    row_fits: impl Fn(usize) -> bool,
    score_at: impl Fn(usize, usize) -> f32,
) -> Option<(usize, f32)> {
    let mut best: Option<(usize, f32)> = None;
    for row in window.row_min..=window.row_max {
        if row < 0 {
            continue;
        }
        let row = row as usize;
        if !row_fits(row) {
            continue;
        }
        let score = score_at(row, bin);
        if best.is_none_or(|(_, best_score)| score > best_score) {
            best = Some((row, score));
        }
    }
    best
}

/// Rank by score descending and keep at most `max_candidates` — the
/// tail every coarse search ends with.
///
/// `sort_unstable_by` rather than a stable sort: the comparator is the
/// score alone, and equal scores are interchangeable here. NaN compares
/// as equal rather than panicking.
pub fn rank_and_truncate(out: &mut Vec<SyncCandidate>, max_candidates: usize) {
    out.sort_unstable_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    out.truncate(max_candidates);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dt_sec_is_signed_against_nominal() {
        // One second late at 12 kHz is 12 000 samples past nominal.
        let c = SyncCandidate {
            start_sample: 24_000,
            freq_hz: 1500.0,
            score: 0.5,
        };
        assert!((c.dt_sec(12_000, 12_000) - 1.0).abs() < 1e-6);
        let early = SyncCandidate {
            start_sample: 6_000,
            ..c
        };
        assert!((early.dt_sec(12_000, 12_000) + 0.5).abs() < 1e-6);
    }

    #[test]
    fn rank_and_truncate_keeps_the_best() {
        let mk = |score| SyncCandidate {
            start_sample: 0,
            freq_hz: 0.0,
            score,
        };
        let mut v = alloc::vec![mk(0.1), mk(0.9), mk(0.5)];
        rank_and_truncate(&mut v, 2);
        assert_eq!(v.len(), 2);
        assert!((v[0].score - 0.9).abs() < 1e-6);
        assert!((v[1].score - 0.5).abs() < 1e-6);
    }

    #[test]
    fn rank_and_truncate_survives_nan() {
        let mk = |score| SyncCandidate {
            start_sample: 0,
            freq_hz: 0.0,
            score,
        };
        let mut v = alloc::vec![mk(f32::NAN), mk(0.5)];
        rank_and_truncate(&mut v, 2);
        assert_eq!(v.len(), 2);
    }

    #[test]
    fn symmetric_sets_both_tolerances() {
        let p = SearchParams::symmetric(200.0, 4000.0, 1.728, 8);
        assert_eq!(p.time_tolerance_early_sec, p.time_tolerance_late_sec);
        assert_eq!(p.score_threshold, DEFAULT_SCORE_THRESHOLD);
    }
}
