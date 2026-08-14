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

/// One candidate surviving the coarse (freq × time) sync search.
#[derive(Clone, Copy, Debug)]
pub struct SyncCandidate {
    /// Sample index where symbol 0 is estimated to start.
    pub start_sample: usize,
    /// Base-tone (tone 0) frequency in Hz.
    pub freq_hz: f32,
    /// Normalised sync score, `sync_pwr / (sync_pwr + noise_floor)`.
    /// Larger = better; [`DEFAULT_SCORE_THRESHOLD`] is the minimum
    /// worth handing to a decode attempt.
    pub score: f32,
}

/// Conservative minimum sync score below which candidates are
/// unlikely to yield a successful decode.
pub const DEFAULT_SCORE_THRESHOLD: f32 = 0.1;

/// Search-window parameters for [`coarse_search`].
#[derive(Clone, Copy, Debug)]
pub struct SearchParams {
    /// Lower edge of the frequency search band, Hz.
    pub freq_min_hz: f32,
    /// Upper edge of the frequency search band, Hz.
    pub freq_max_hz: f32,
    /// ±Δt search window around the nominal start sample, **in
    /// seconds**. Seconds rather than symbols for the same reason
    /// Q65's is (issue #282): symbol counts do not transfer between
    /// protocols or sub-modes, and the reference decoder states this
    /// window in time.
    pub time_tolerance_sec: f32,
    /// Minimum candidate score (normalised); see [`SyncCandidate::score`].
    pub score_threshold: f32,
    /// Maximum number of candidates returned, best-score first.
    pub max_candidates: usize,
}

impl Default for SearchParams {
    fn default() -> Self {
        Self {
            // JT65 typically lives 1000–2000 Hz on the dial.
            freq_min_hz: 1000.0,
            freq_max_hz: 2000.0,
            // WSJT-X `sync65.f90:29-30` searches `lag1=-32 .. lag2=82`
            // in units of `1024/11025` s = 92.9 ms, i.e. **−2.97 s to
            // +7.62 s** — deliberately asymmetric, because a JT65
            // frame is 46.8 s inside a 60 s slot and a late start has
            // far more room than an early one.
            //
            // Searched symmetrically here at the wider (late) half.
            // `row_min` clamps at row 0 for any realistic nominal
            // start (~1 s), so the extra negative span costs nothing
            // reachable; the alternative — carrying two fields to
            // mirror the asymmetry exactly — buys no behaviour.
            //
            // Was `time_tolerance_symbols: 3` (±1.11 s) until issue
            // #282. Measured against real `jt9 -6 -p 60 -d 3` over a
            // `jt65sim -t` Δt sweep at −10 dB: `jt9` decoded out to
            // Δt = +5.0 s; this crate stopped at Δt = 0.0.
            time_tolerance_sec: 7.62,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            // Stays at 8, *not* raised toward `sync65.f90:3`'s
            // `MAXCAND=300` (issue #282 proposed that; measurement
            // withdrew it). The two numbers are not comparable:
            // WSJT-X's is an array bound for candidates emitted only
            // at local maxima above `thresh0`, while this one
            // truncates a ranked list in which *every* entry costs a
            // full decode attempt.
            //
            // Measured on ten `jt65sim` files at −22 dB (the SNR
            // where recall is partial, so a cap change can show):
            // 8 → 5/10 in 0.4 s, 300 → 5/10 in 11.0 s. Identical
            // recall, 27× the time. `tests/jt65_sweep.rs` alone went
            // 7.4 s → 189 s with the raise, on a test that gates
            // every PR.
            max_candidates: 8,
        }
    }
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
/// [`crate::jt65::rx::demodulate_aligned_with_runnerup`]'s residual-NCO
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
    let df = sample_rate as f32 / nsps as f32;

    let t_span_rows =
        (params.time_tolerance_sec * sample_rate as f32 / spec.t_step.max(1) as f32).round() as i64;
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
        if row + 125 * ROWS_PER_SYMBOL >= spec.n_time {
            continue;
        }
        for fb in fmin_bin..=fmax_bin {
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
    out.sort_unstable_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    out.truncate(params.max_candidates);
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
        let cands = coarse_search(&audio, 12_000, 0, &SearchParams::default());
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
