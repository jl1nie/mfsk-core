// SPDX-License-Identifier: GPL-3.0-or-later
//! Coarse (frequency × time) sync search for Q65.
//!
//! Q65's distributed sync (22 symbols all on tone 0) is the
//! cleanest correlation target across the WSJT family — every sync
//! symbol carries the full symbol energy on the same frequency bin.
//! The spectrogram build and per-candidate scoring are literally
//! shared with `crate::jt9::search` and `crate::jt65::search` (see
//! [`crate::engine::spectrogram`]), built here at `nsps/8`-symbol time
//! steps (matching WSJT-X's own `NSTEP=8` sync resolution,
//! `lib/qra/q65/q65.f90:3`) instead of their `nsps/4` — only the
//! sync-positions list and the candidate-selection loop below are
//! this protocol's own.

use crate::engine::ModulationParams;
use crate::engine::spectrogram;

use super::sync_pattern::Q65_SYNC_POSITIONS;

/// FFT-bin spectrogram covering the audio buffer at `nsps/8`-symbol
/// time steps. Thin alias — see
/// [`crate::engine::spectrogram::Spectrogram`] for the shared
/// implementation (extracted 2026-08-14, code-sharing audit; was
/// structurally identical to `jt9::search::Spectrogram` /
/// `jt65::search::Spectrogram`, differing only in the time-step
/// divisor — [`NSTEP_PER_SYMBOL`] here vs. their fixed 4).
pub type Spectrogram = spectrogram::Spectrogram;

/// Time step matching WSJT-X's own sync spectrogram resolution
/// (`NSTEP=8`, `lib/qra/q65/q65.f90:3`, "Number of time bins per
/// symbol in s1, s1a, s1b") — an earlier `nsps/2` here under-resolved
/// the sync search 4× relative to `q65_ccf_22`'s own lag search, which
/// was root-caused (verified against real jt9) as the source of a
/// multi-dB AWGN sensitivity gap at `GridDepth::Fast`'s single-shot
/// decode (no further Δt retry to compensate); see
/// `docs/notes/Q65_BENCHMARK.md`.
pub const NSTEP_PER_SYMBOL: usize = 8;

/// Build a spectrogram for Q65 sub-mode `P`. Frequency resolution =
/// `sample_rate / nsps` ≈ tone spacing.
///
/// The previous `Spectrogram::build` Q65-30A convenience wrapper
/// (calling this with `P = Q65a30` hardcoded) had zero callers
/// anywhere in the crate or its tests — dropped rather than carried
/// through this extraction, predating the crate's now-10-wired-
/// sub-mode reality (issue tracking session, 2026-08-14).
pub fn build_spectrogram<P: ModulationParams>(audio: &[f32], sample_rate: u32) -> Spectrogram {
    Spectrogram::build_for::<P>(audio, sample_rate, NSTEP_PER_SYMBOL)
}

/// One candidate surviving the coarse sync search.
#[derive(Clone, Copy, Debug)]
pub struct SyncCandidate {
    /// Sample index where symbol 0 is estimated to start.
    pub start_sample: usize,
    /// Tone-0 (sync) frequency in Hz.
    pub freq_hz: f32,
    /// Normalised score in `[0, 1]`: `sync_pwr / (sync_pwr + noise_floor)`.
    pub score: f32,
}

/// Default minimum sync score for handing a candidate to a decode attempt.
pub const DEFAULT_SCORE_THRESHOLD: f32 = 0.1;

#[derive(Clone, Copy, Debug)]
pub struct SearchParams {
    pub freq_min_hz: f32,
    pub freq_max_hz: f32,
    /// How far *before* `nominal_start_sample` to search, in seconds.
    ///
    /// Seconds, not symbols, because that is the unit WSJT-X uses and
    /// the two are not interchangeable across sub-modes: Q65 symbol
    /// length ranges 0.15 s (Q65-15) to 3.456 s (Q65-300), so a
    /// symbol-denominated tolerance silently becomes a different
    /// window per sub-mode.
    pub time_tolerance_early_sec: f32,
    /// How far *after* `nominal_start_sample` to search, in seconds.
    ///
    /// Separate from [`Self::time_tolerance_early_sec`] because the
    /// reference window is **not symmetric** — see [`Self::default`].
    pub time_tolerance_late_sec: f32,
    pub score_threshold: f32,
    pub max_candidates: usize,
}

impl Default for SearchParams {
    fn default() -> Self {
        Self {
            // Default Q65 dial range: 200 Hz .. 3000 Hz inside the
            // SSB passband. Callers can narrow this further.
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            // The reference Q65 window is **asymmetric**. Measured by
            // running real `jt9 -3 -d 3` over `q65sim` Δt sweeps:
            //
            //   Q65-15A (nsps=1800)   -1.0 .. +1.0 s
            //   Q65-30A (nsps=3600)   -1.0 .. +1.0 s
            //   Q65-60A (nsps=7200)   -1.0 .. +5.5 s
            //
            // `q65.f90:127-129` sets `lag1=-1.0/dtstep`,
            // `lag2=1.0/dtstep`, and extends `lag2` to `5.5/dtstep`
            // when `nsps >= 3600 .and. emedelay > 0`. The measurement
            // says that extension is live for TR>=60 and not for
            // TR=30, which the `nsps >= 3600` half alone does not
            // explain (Q65-30A *is* nsps=3600) — the `emedelay` half
            // is not observable from outside, so the constants here
            // follow the measurement rather than the source, per
            // `tests/dt_window.rs`'s own doctrine.
            //
            // +5.5 s is applied to every sub-mode rather than gated on
            // NSPS: on the short sub-modes the extra span is
            // geometrically self-limiting (a Q65-15 frame placed +5.5 s
            // late does not fit in a 15 s slot at all, so those rows
            // are rejected by the frame-fits guard for the cost of a
            // scan), and a uniform value cannot silently under-search a
            // sub-mode the way the old symbol-denominated one did.
            //
            // History, because this default has now been wrong twice:
            // it was `time_tolerance_symbols: 5` until issue #282
            // (±0.75 s on Q65-15 — narrower than the reference), then
            // a symmetric `time_tolerance_sec: 1.0`, which fixed
            // Q65-15 but cut Q65-60A's late reach from +3.0 to +1.0 s
            // against a reference that goes to +5.5 s. Both slipped
            // through because every in-tree Q65 test passes explicit
            // tolerances and none exercised the default.
            time_tolerance_early_sec: 1.0,
            time_tolerance_late_sec: 5.5,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates: 8,
        }
    }
}

/// Score one `(start_row, base_bin)`: sum tone-0 power across the
/// 22 sync positions, divide by `(sum + noise_floor)`.
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    let rows_per_symbol = (spec.nsps / spec.t_step).max(1);
    spectrogram::score_candidate(
        spec,
        start_row,
        base_bin,
        &Q65_SYNC_POSITIONS,
        rows_per_symbol,
    )
}

/// Build a spectrogram for Q65 sub-mode `P` and find the top sync
/// candidates inside the search window.
pub fn coarse_search_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = build_spectrogram::<P>(audio, sample_rate);
    coarse_search_on_spec_for::<P>(&spec, sample_rate, nominal_start_sample, params)
}

/// Q65-30A convenience wrapper for [`coarse_search_for`].
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    coarse_search_for::<super::Q65a30>(audio, sample_rate, nominal_start_sample, params)
}

/// Same as [`coarse_search_for`] but accepts a pre-built spectrogram
/// — useful when the same audio is scanned under multiple parameter
/// sets.
pub fn coarse_search_on_spec_for<P: ModulationParams>(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let rows_per_symbol = (nsps / spec.t_step.max(1)).max(1);
    // For wider sub-modes (B/C/D/E) the highest data tone sits
    // 64 × bins_per_tone above the sync bin instead of just 64
    // bins; we need that much headroom in the spectrogram before
    // we will accept a candidate base bin.
    let bins_per_tone = (P::TONE_SPACING_HZ / df).round() as usize;

    let rows_per_sec = sample_rate as f32 / spec.t_step.max(1) as f32;
    let early_rows = (params.time_tolerance_early_sec.max(0.0) * rows_per_sec).round() as i64;
    let late_rows = (params.time_tolerance_late_sec.max(0.0) * rows_per_sec).round() as i64;
    let nominal_row = (nominal_start_sample / spec.t_step) as i64;
    let row_min = (nominal_row - early_rows).max(0);
    let row_max = nominal_row + late_rows;

    let fmin_bin = (params.freq_min_hz / df).floor() as i64;
    let fmax_bin = (params.freq_max_hz / df).ceil() as i64;

    // Collapse over time first, per frequency bin — mirrors
    // `q65_ccf_22`'s own structure (`lib/qra/q65/q65.f90:506-538`):
    // for each frequency it keeps only the single best-scoring lag
    // (`ccfmax = max over lag,idrift`), then ranks candidates across
    // frequencies from that already-time-collapsed curve. Emitting
    // one `(row, freq)` candidate per cell instead — as an earlier
    // version of this function did — let a finer time step (`t_step`
    // was widened 4× here to match WSJT-X's `NSTEP=8`) flood the
    // `max_candidates`-truncated list with near-duplicate rows all
    // describing the same true peak's neighbourhood, crowding out
    // distinct weaker signals (regression caught by
    // `ionoscatter_6m_120e_decodes_with_fading_metric`, a real-off-air
    // multi-signal recording).
    let fb_lo = fmin_bin.max(0) as usize;
    let fb_hi = fmax_bin.max(fmin_bin) as usize;
    let mut curve: Vec<f32> = vec![0.0; fb_hi.saturating_sub(fb_lo) + 1];
    let mut rows: Vec<usize> = vec![0; curve.len()];
    for fb in fb_lo..=fb_hi {
        // Tone 64 (highest data tone) sits at base_bin + 64 *
        // bins_per_tone for the active sub-mode.
        if fb + 64 * bins_per_tone + 1 > spec.n_freq {
            continue;
        }
        let mut best: Option<(usize, f32)> = None;
        for row in row_min..=row_max {
            if row < 0 {
                continue;
            }
            let row = row as usize;
            // Need room for the last data symbol (84) + the 64 data
            // tones above the sync bin.
            if row + 84 * rows_per_symbol >= spec.n_time {
                continue;
            }
            let score = score_candidate(spec, row, fb);
            if best.is_none_or(|(_, best_score)| score > best_score) {
                best = Some((row, score));
            }
        }
        if let Some((row, score)) = best {
            let idx = fb - fb_lo;
            curve[idx] = score;
            rows[idx] = row;
        }
    }

    // Noise-adaptive admission threshold — `q65_ccf_22`'s own
    // candidate-selection logic (`lib/qra/q65/q65.f90:553-574`):
    // `ave` = 50th percentile, `base` = 84th percentile of the whole
    // per-frequency score curve (≈ mean+1σ for a roughly-Gaussian
    // noise floor), `rms = base - ave`, admit only candidates with
    // `(score-ave)/rms >= 6.0`. This adapts to each recording's own
    // noise spread instead of a fixed absolute `score_threshold`,
    // which — verified against real jt9 on a pure-AWGN sweep,
    // `docs/notes/Q65_BENCHMARK.md` — was rejecting genuine weak
    // signals outright at low SNR well before `max_candidates`
    // truncation ever mattered.
    let ave = percentile(&curve, 50);
    let base = percentile(&curve, 84);
    let rms = base - ave;
    let use_adaptive = rms.is_finite() && rms > 1e-6;
    const SNR_ADMIT: f32 = 6.0;

    // Frequency-domain local-max suppression — `i3=i-mode_q65,
    // i4=i+mode_q65; if(ccf2(i).ne.biggest) cycle`
    // (`lib/qra/q65/q65.f90:563-566`) — `mode_q65` there is exactly
    // our `bins_per_tone` (`nBinsPerTone = 1<<submode`, `q65.c:351`).
    let mut out: Vec<SyncCandidate> = Vec::new();
    for (idx, &score) in curve.iter().enumerate() {
        if score <= 0.0 {
            continue;
        }
        // OR, not replace: the adaptive gate rescues weak-but-real
        // signals the fixed floor would reject outright (the AWGN
        // case this was added for), but a clean/strong signal must
        // still be admitted on its own absolute score even if the
        // curve's noise estimate is itself distorted (e.g. broadband
        // transients from a hard on/off edge in a synthetic test
        // buffer inflating `rms` well above what a continuous-noise
        // real recording would show).
        let admitted =
            score >= params.score_threshold || (use_adaptive && (score - ave) / rms >= SNR_ADMIT);
        if !admitted {
            continue;
        }
        let lo = idx.saturating_sub(bins_per_tone);
        let hi = (idx + bins_per_tone).min(curve.len() - 1);
        let is_local_max = curve[lo..=hi].iter().all(|&other| other <= score);
        if !is_local_max {
            continue;
        }
        out.push(SyncCandidate {
            start_sample: rows[idx] * spec.t_step,
            freq_hz: (fb_lo + idx) as f32 * df,
            score,
        });
    }
    out.sort_unstable_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    out.truncate(params.max_candidates);
    out
}

/// Nearest-rank percentile — matches WSJT-X's own `pctile`
/// (`lib/pctile.f90`): sort ascending, take element at
/// `round(n * pct/100)` (1-indexed, clamped to `[1, n]`).
fn percentile(values: &[f32], pct: u32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f32> = values.to_vec();
    sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = sorted.len();
    let j = ((n as f32 * 0.01 * pct as f32).round() as usize)
        .max(1)
        .min(n);
    sorted[j - 1]
}

/// Q65-30A convenience wrapper for [`coarse_search_on_spec_for`].
pub fn coarse_search_on_spec(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    coarse_search_on_spec_for::<super::Q65a30>(spec, sample_rate, nominal_start_sample, params)
}

#[cfg(test)]
mod tests {
    use super::super::tx::synthesize_standard;
    use super::*;

    #[test]
    fn coarse_search_finds_clean_signal() {
        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let cands = coarse_search(&audio, 12_000, 0, &SearchParams::default());
        assert!(!cands.is_empty(), "search should find a clean signal");
        let best = cands[0];
        // Frequency bin width is 12000/3600 ≈ 3.33 Hz, so ±4 Hz is
        // within one bin tolerance.
        assert!(
            (best.freq_hz - freq).abs() <= 4.0,
            "best freq {} should be near {freq} Hz",
            best.freq_hz
        );
        assert_eq!(best.start_sample, 0, "clean synth starts at sample 0");
        assert!(best.score > 0.5, "clean signal should score > 0.5");
    }
}
