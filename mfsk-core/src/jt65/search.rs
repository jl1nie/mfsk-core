//! Coarse (frequency × time) search for JT65.
//!
//! JT65 carries its sync tone (tone 0) at 63 positions determined by
//! a pseudo-random 126-bit pattern (`JT65_NPRC`). The coarse search
//! builds an NSPS-sized FFT spectrogram at quarter-symbol steps and
//! scores each candidate (`start_row`, `base_bin`) by summing the
//! FFT-bin power at `base_bin` across the 63 sync-position rows.
//!
//! The spectrogram build and per-candidate scoring are literally
//! shared with `crate::jt9::search` and `crate::q65::search` (see
//! [`crate::engine::spectrogram`]) — only the sync-positions list and
//! the candidate-selection loop below are this protocol's own.

use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use crate::engine::ModulationParams;
use crate::engine::spectrogram;
use crate::engine::sync::refine_freq_hz_log_power;

use super::Jt65;
use super::sync_pattern::JT65_SYNC_POSITIONS;

/// Precomputed per-time-step FFT magnitudes-squared used by the
/// coarse (freq × time) search. Thin alias — see
/// [`crate::engine::spectrogram::Spectrogram`] for the shared
/// implementation (extracted 2026-08-14, code-sharing audit; was a
/// byte-identical copy of `jt9::search::Spectrogram`).
pub type Spectrogram = spectrogram::Spectrogram;

/// Quarter-symbol time step, matching WSJT-X's own coarse-search
/// resolution for this protocol family.
const NSTEP_PER_SYMBOL: usize = 4;

fn build_spectrogram(audio: &[f32], sample_rate: u32) -> Spectrogram {
    Spectrogram::build_for::<Jt65>(audio, sample_rate, NSTEP_PER_SYMBOL)
}

pub use crate::engine::search::{DEFAULT_SCORE_THRESHOLD, SearchParams, SyncCandidate};
use crate::engine::search::{SearchWindow, rank_and_truncate};

/// JT65's own coarse-search defaults.
///
/// 1000-2000 Hz is where JT65 activity actually sits; ±7.62 s is the
/// window WSJT-X's own search covers.
pub fn default_search_params() -> SearchParams {
    SearchParams::symmetric(1000.0, 2000.0, 7.62, 8)
}

/// Row step between consecutive symbols in a [`Spectrogram`] built at
/// [`NSTEP_PER_SYMBOL`] steps.
const ROWS_PER_SYMBOL: usize = NSTEP_PER_SYMBOL;

fn sync_power_at_bin(spec: &Spectrogram, start_row: usize, bin: usize) -> f32 {
    spectrogram::sync_power_at_bin(spec, start_row, bin, &JT65_SYNC_POSITIONS, ROWS_PER_SYMBOL)
}

/// Score one candidate (start_row, base_bin). Sums FFT-bin power at
/// `base_bin` over the 63 sync-position rows; normalises against the
/// noise floor.
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    spectrogram::score_candidate(
        spec,
        start_row,
        base_bin,
        &JT65_SYNC_POSITIONS,
        ROWS_PER_SYMBOL,
    )
}

/// Refine a candidate's frequency to sub-bin precision — see
/// [`refine_freq_hz_log_power`] for the estimator itself (shared with
/// `jt9::search`, issue #169's original fix).
///
/// `coarse_search`'s frequency grid is one bin wide (`df` ≈ 2.69 Hz at
/// JT65A's NSPS/rate) — a signal landing between two bins pays a real
/// rectangular-window "scalloping loss" (up to ≈3.9 dB worst-case, at
/// exactly half a bin off), which this crate's own AWGN sweep
/// happened to hit on every trial (the golden test frequency, 1500 Hz,
/// sits at *exactly* bin 557.5 — the worst possible case). This
/// estimator recovers the sub-bin offset from the already-computed
/// [`Spectrogram`] (no extra FFTs) so
/// [`crate::jt65::rx::demodulate_aligned`]'s residual-NCO
/// correction has a genuinely fractional frequency to act on — a
/// plain bin-multiple `freq_hz` is a no-op there. See
/// `docs/notes/BENCHMARKS.md`'s JT65 section for the measured effect
/// (this is not specific to the synthetic sweep corpus — any on-air
/// signal not landing exactly on a bin center pays some fraction of
/// the same loss).
fn refine_freq_hz(spec: &Spectrogram, start_row: usize, base_bin: usize, df: f32) -> f32 {
    refine_freq_hz_log_power(base_bin, spec.n_freq, df, |bin| {
        sync_power_at_bin(spec, start_row, bin)
    })
}

/// Build a spectrogram of `audio` and return the best-scoring
/// (start_sample, freq_hz) candidates for a JT65 frame within
/// the search window specified by `params`.
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = build_spectrogram(audio, sample_rate);
    coarse_search_on_spec(&spec, sample_rate, nominal_start_sample, params)
}

/// Like [`coarse_search`] but takes a pre-built [`Spectrogram`] —
/// useful when the same audio buffer is scanned under multiple
/// parameter sets.
pub fn coarse_search_on_spec(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let w = SearchWindow::new(spec.t_step, sample_rate, nominal_start_sample, nsps, params);
    let df = w.df;

    let mut out: Vec<SyncCandidate> = Vec::new();
    for row in w.row_min..=w.row_max {
        if row < 0 {
            continue;
        }
        let row = row as usize;
        if row + 125 * ROWS_PER_SYMBOL >= spec.n_time {
            continue;
        }
        for fb in w.fmin_bin..=w.fmax_bin {
            if fb < 0 || (fb as usize) + 66 > spec.n_freq {
                continue;
            }
            let score = score_candidate(spec, row, fb as usize);
            if score >= params.score_threshold {
                out.push(SyncCandidate {
                    start_sample: row * spec.t_step,
                    freq_hz: refine_freq_hz(spec, row, fb as usize, df),
                    score,
                });
            }
        }
    }
    rank_and_truncate(&mut out, params.max_candidates);
    out
}

#[cfg(test)]
mod tests {
    use super::super::synthesize_standard;
    use super::*;

    #[test]
    fn coarse_search_finds_clean_signal() {
        let freq = 1270.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let cands = coarse_search(&audio, 12_000, 0, &default_search_params());
        assert!(!cands.is_empty());
        let best = cands[0];
        assert!(
            (best.freq_hz - 1270.0).abs() <= 4.0,
            "best freq {} should be near 1270 Hz",
            best.freq_hz
        );
        assert_eq!(best.start_sample, 0);
        assert!(best.score > 0.5);
    }
}
