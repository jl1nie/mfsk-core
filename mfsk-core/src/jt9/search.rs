//! Coarse (frequency × time) search for JT9.
//!
//! JT9's 16 sync symbols all sit at a single tone (tone 0, one spacing
//! below the 8 data tones). We build a symbol-length-FFT spectrogram
//! at quarter-symbol steps, and for each candidate (`start_row`,
//! `base_bin`) sum the FFT-bin power at `base_bin` for every
//! `JT9_SYNC_POSITIONS`-indexed row. The candidate with the most
//! concentrated sync-tone energy wins.
//!
//! This lets us decode WAV files where the transmitter's start time
//! and carrier frequency aren't known — the common real-world case.
//! The aligned `decode_at` remains available for callers that already
//! know both.

use crate::engine::ModulationParams;
use crate::engine::spectrogram;
use crate::engine::sync::refine_freq_hz_log_power;

use super::Jt9;
use super::sync_pattern::JT9_SYNC_POSITIONS;

/// One-symbol-FFT spectrogram, reusable across many candidate scores.
/// Thin alias — see [`crate::engine::spectrogram::Spectrogram`] for
/// the shared implementation (extracted 2026-08-14, code-sharing
/// audit; was a byte-identical copy of `jt65::search::Spectrogram`).
pub type Spectrogram = spectrogram::Spectrogram;

/// Quarter-symbol time step, matching WSJT-X's own coarse-search
/// resolution for this protocol family.
const NSTEP_PER_SYMBOL: usize = 4;

fn build_spectrogram(audio: &[f32], sample_rate: u32) -> Spectrogram {
    Spectrogram::build_for::<Jt9>(audio, sample_rate, NSTEP_PER_SYMBOL)
}

/// A candidate JT9 alignment, ranked by sync-tone score.
#[derive(Clone, Copy, Debug)]
pub struct SyncCandidate {
    /// Absolute sample index of symbol 0.
    pub start_sample: usize,
    /// Frequency of tone 0 (the sync tone, i.e. the low end of the
    /// 9-tone constellation).
    pub freq_hz: f32,
    /// Normalised score; higher is better.
    pub score: f32,
}

/// Default sync-score threshold. Pure noise scores ≈ 0; a clean
/// aligned frame scores ≈ 1 for high SNR. 0.1 is a safely-loose
/// prefilter that still drops most garbage candidates.
pub const DEFAULT_SCORE_THRESHOLD: f32 = 0.1;

/// JT9 coarse-search parameter block.
#[derive(Clone, Copy, Debug)]
pub struct SearchParams {
    pub freq_min_hz: f32,
    pub freq_max_hz: f32,
    /// ±Δt search window around `nominal_start_sample`, **in
    /// seconds**. Seconds rather than symbols (issue #282) — the
    /// symbol-denominated form is what let Q65's equivalent window
    /// silently become a different span per sub-mode.
    pub time_tolerance_sec: f32,
    pub score_threshold: f32,
    pub max_candidates: usize,
}

impl Default for SearchParams {
    fn default() -> Self {
        Self {
            // `jt9`'s own CLI defaults (`--lowest` 200, `--highest`
            // 4007). Was 1400-1600 Hz, which is narrower than any
            // real JT9 sub-band and **could not decode this crate's
            // own JT9 golden recording**: `tests/jt9_wsjtx_samples.rs`
            // had to override it, with a comment saying the default
            // "excludes every" golden decode.
            //
            // Measured on `130418_1742.wav` (issue #282 follow-up):
            // 1400-1600 Hz found 2 decodes, every wider band found 5,
            // and the wall clock was flat at ~60 ms from 200 Hz wide
            // to 3800 Hz wide — the coarse search's cost is the
            // whole-buffer spectrogram build, not the per-bin scan,
            // so the narrow band was buying nothing at all.
            freq_min_hz: 200.0,
            freq_max_hz: 4000.0,
            // 1.728 s — numerically identical to the previous
            // `time_tolerance_symbols: 3` (JT9 symbols are 0.576 s),
            // so this is a unit change, not a behaviour change.
            //
            // `jt9_decode.f90:69-70` reads `lag1=-2.5/tstep`,
            // `lag2=+5.0/tstep`, which looks far wider. Measured
            // (issue #282), real `jt9 -9 -p 60 -d 3` on a shifted
            // `jt9sim` sweep decodes only out to Δt ≈ +0.6 s — so
            // the source's apparent late reach is not usable reach,
            // and this window already covers what the reference
            // actually achieves. Guarded by
            // `tests/dt_window.rs::jt9_window_reaches_reference_late_edge`.
            time_tolerance_sec: 1.728,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates: 8,
        }
    }
}

/// Row step between consecutive symbols in a [`Spectrogram`] built at
/// [`NSTEP_PER_SYMBOL`] steps.
const ROWS_PER_SYMBOL: usize = NSTEP_PER_SYMBOL;

fn sync_power_at_bin(spec: &Spectrogram, start_row: usize, bin: usize) -> f32 {
    spectrogram::sync_power_at_bin(spec, start_row, bin, &JT9_SYNC_POSITIONS, ROWS_PER_SYMBOL)
}

/// Score one candidate using the precomputed spectrogram.
///
/// `start_row` is the spectrogram row index of symbol 0. Because the
/// spectrogram step is NSPS/4, consecutive symbols are 4 rows apart.
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    spectrogram::score_candidate(
        spec,
        start_row,
        base_bin,
        &JT9_SYNC_POSITIONS,
        ROWS_PER_SYMBOL,
    )
}

/// Refine a candidate's frequency to sub-bin precision — see
/// [`refine_freq_hz_log_power`] for the estimator itself (shared with
/// `jt65::search`).
///
/// `coarse_search`'s frequency grid is one bin wide (`df` ≈ 1.736 Hz,
/// exactly the tone spacing) — the true signal frequency can land
/// anywhere within that bin, but unlike `downsam9`'s later big-FFT
/// extraction (~0.018 Hz resolution), nothing before this refinement
/// step narrows it down. Measured (task #24, `jt9_awgn_m26_09.wav`):
/// the coarse candidate landed at 1399.3 Hz, which never converges in
/// Fano at any [`super::Jt9Depth`] tier, while frequencies just
/// 0.3-1.2 Hz away (1399.0 Hz, 1400.5 Hz) converge easily — the same
/// "coarse bin center isn't close enough to the true frequency, and
/// nothing downstream fully recovers from it" shape as JT65's own
/// scalloping-loss fix (issue #169, originally ported here as its own
/// copy of the estimator before the two were unified into
/// [`refine_freq_hz_log_power`]).
fn refine_freq_hz(spec: &Spectrogram, start_row: usize, base_bin: usize, df: f32) -> f32 {
    refine_freq_hz_log_power(base_bin, spec.n_freq, df, |bin| {
        sync_power_at_bin(spec, start_row, bin)
    })
}

/// Sweep (freq × time) and return top-scored candidates.
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = build_spectrogram(audio, sample_rate);
    coarse_search_on_spec(&spec, sample_rate, nominal_start_sample, params)
}

/// Same as [`coarse_search`] but reuses a pre-built spectrogram.
pub fn coarse_search_on_spec(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * <Jt9 as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;

    let t_span_rows =
        (params.time_tolerance_sec * sample_rate as f32 / spec.t_step.max(1) as f32).round() as i64;
    let nominal_row = (nominal_start_sample / spec.t_step) as i64;
    let row_min = (nominal_row - t_span_rows).max(0);
    let row_max = nominal_row + t_span_rows;

    let fmin_bin = (params.freq_min_hz / df).floor() as i64;
    let fmax_bin = (params.freq_max_hz / df).ceil() as i64;

    // For each freq bin, keep ONLY the best-scoring time alignment —
    // mirrors WSJT-X `sync9` `ccfred(i)=max over lags of sum`. Without
    // this collapse, a single strong signal's many time variants
    // would crowd out lower-scoring real signals at other carriers
    // when we apply `max_candidates`.
    let mut out: Vec<SyncCandidate> = Vec::new();
    for fb in fmin_bin..=fmax_bin {
        if fb < 0 || (fb as usize) + 9 > spec.n_freq {
            continue;
        }
        let mut best_row: i64 = -1;
        let mut best_score = f32::NEG_INFINITY;
        for row in row_min..=row_max {
            if row < 0 {
                continue;
            }
            let row_u = row as usize;
            if row_u + 84 * ROWS_PER_SYMBOL >= spec.n_time {
                continue;
            }
            let score = score_candidate(spec, row_u, fb as usize);
            if score > best_score {
                best_score = score;
                best_row = row;
            }
        }
        if best_row >= 0 && best_score >= params.score_threshold {
            out.push(SyncCandidate {
                start_sample: best_row as usize * spec.t_step,
                freq_hz: refine_freq_hz(spec, best_row as usize, fb as usize, df),
                score: best_score,
            });
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
        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let cands = coarse_search(&audio, 12_000, 0, &SearchParams::default());
        assert!(!cands.is_empty(), "expected at least one candidate");
        let best = cands[0];
        assert!(
            (best.freq_hz - 1500.0).abs() <= 3.0,
            "best freq {} should be near 1500 Hz",
            best.freq_hz
        );
        assert_eq!(best.start_sample, 0);
        assert!(best.score > 0.5, "clean score was {}", best.score);
    }
}

#[cfg(test)]
mod diag_tests {
    use super::*;
    use std::path::Path;

    #[test]
    #[ignore]
    fn jt9_coarse_diag() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        if !path.exists() {
            eprintln!("WAV not found");
            return;
        }
        let bytes = std::fs::read(path).unwrap();
        let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let data = &bytes[44..44 + data_len];
        let audio: Vec<f32> = data
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
            .collect();
        let params = SearchParams {
            freq_min_hz: 1050.0,
            freq_max_hz: 1500.0,
            time_tolerance_sec: 1.728,
            score_threshold: 0.001,
            max_candidates: 5000,
        };
        let cands = coarse_search(&audio, 12_000, 0, &params);
        eprintln!("Total candidates above 0.001: {}", cands.len());
        for golden_hz in &[1119.0f32, 1186.0, 1224.0, 1290.0, 1346.0] {
            let near: Vec<_> = cands
                .iter()
                .filter(|c| (c.freq_hz - golden_hz).abs() < 5.0)
                .collect();
            eprintln!("Near {} Hz: {} cands", golden_hz, near.len());
            for c in near.iter().take(3) {
                eprintln!(
                    "  freq={:.1} start_s={:.2} score={:.4}",
                    c.freq_hz,
                    c.start_sample as f32 / 12000.0,
                    c.score
                );
            }
        }
        eprintln!("Top 20:");
        for c in cands.iter().take(20) {
            eprintln!(
                "  freq={:.1} start_s={:.2} score={:.4}",
                c.freq_hz,
                c.start_sample as f32 / 12000.0,
                c.score
            );
        }
    }
}
