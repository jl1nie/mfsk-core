// SPDX-License-Identifier: GPL-3.0-or-later
//! Coarse (frequency × time) sync search for Q65.
//!
//! Q65's distributed sync (22 symbols all on tone 0) is the
//! cleanest correlation target across the WSJT family — every sync
//! symbol carries the full symbol energy on the same frequency bin.
//! This module mirrors the structure of [`crate::jt65::search`]:
//! build an NSPS-sized spectrogram at `nsps/8`-symbol time steps
//! (matching WSJT-X's own `NSTEP=8` sync resolution,
//! `lib/qra/q65/q65.f90:3`), score each candidate `(start_row,
//! base_bin)` by summing the tone-0 power across the 22 sync
//! positions, and return the top-scoring candidates.

use crate::engine::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::Q65a30;
use super::sync_pattern::Q65_SYNC_POSITIONS;

/// FFT-bin spectrogram covering the audio buffer at `nsps/8`-symbol
/// time steps. `mags_sqr[t * n_freq + f]` is `|FFT[f]|²` at time `t`.
pub struct Spectrogram {
    pub mags_sqr: Vec<f32>,
    pub n_time: usize,
    pub n_freq: usize,
    pub t_step: usize,
    pub nsps: usize,
    pub df: f32,
    pub noise_per_bin: f32,
}

impl Spectrogram {
    /// Build the spectrogram from `audio` for Q65 sub-mode `P`. Time
    /// step = `nsps / NSTEP_PER_SYMBOL`, matching WSJT-X's own sync
    /// spectrogram resolution (`NSTEP=8`, `lib/qra/q65/q65.f90:3`,
    /// "Number of time bins per symbol in s1, s1a, s1b") — an earlier
    /// `nsps/2` here under-resolved the sync search 4× relative to
    /// `q65_ccf_22`'s own lag search, which was root-caused (verified
    /// against real jt9) as the source of a multi-dB AWGN sensitivity
    /// gap at `GridDepth::Fast`'s single-shot decode (no further Δt
    /// retry to compensate); see `docs/notes/Q65_BENCHMARK.md`.
    /// Frequency resolution = `sample_rate / nsps` ≈ tone spacing.
    pub fn build_for<P: ModulationParams>(audio: &[f32], sample_rate: u32) -> Self {
        const NSTEP_PER_SYMBOL: usize = 8;
        let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
        let t_step = (nsps / NSTEP_PER_SYMBOL).max(1);
        let n_freq = nsps / 2;
        if audio.len() < nsps || t_step == 0 {
            return Self {
                mags_sqr: Vec::new(),
                n_time: 0,
                n_freq: 0,
                t_step: 0,
                nsps,
                df: sample_rate as f32 / nsps as f32,
                noise_per_bin: 1.0,
            };
        }
        let n_time = (audio.len() - nsps) / t_step + 1;
        let mut mags_sqr = vec![0f32; n_time * n_freq];
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(nsps);
        let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
        let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];

        for t in 0..n_time {
            let start = t * t_step;
            for (slot, &s) in buf.iter_mut().zip(&audio[start..start + nsps]) {
                *slot = Complex::new(s, 0.0);
            }
            fft.process_with_scratch(&mut buf, &mut scratch);
            let row = &mut mags_sqr[t * n_freq..(t + 1) * n_freq];
            for (slot, c) in row.iter_mut().zip(buf.iter().take(n_freq)) {
                *slot = c.norm_sqr();
            }
        }

        // Noise floor estimate: trimmed-mean of the bottom 95 % of
        // FFT magnitudes (rejects strong narrow-band signals). Only
        // the *set* of bottom-95% values (order within that set is
        // irrelevant, we just sum them) is needed, not a full
        // ascending order — `select_nth_unstable_by` partitions in
        // O(n) average instead of `sort_unstable_by`'s O(n log n),
        // same fix already applied to FT8's `xsnr2_db_simple` noise
        // median. Measured as 64% of `decode_multi_period_for`'s
        // wall-clock on Q65-60B (short-slot, `build_for` called once
        // per audio slot).
        let mut sorted = mags_sqr.clone();
        let keep = (sorted.len() as f32 * 0.95) as usize;
        let noise_per_bin = if keep > 0 {
            sorted.select_nth_unstable_by(keep - 1, |a, b| {
                a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
            });
            sorted[..keep].iter().sum::<f32>() / keep as f32
        } else {
            1.0
        };

        Self {
            mags_sqr,
            n_time,
            n_freq,
            t_step,
            nsps,
            df: sample_rate as f32 / nsps as f32,
            noise_per_bin: noise_per_bin.max(1e-6),
        }
    }

    /// Q65-30A convenience wrapper for [`Self::build_for`].
    pub fn build(audio: &[f32], sample_rate: u32) -> Self {
        Self::build_for::<Q65a30>(audio, sample_rate)
    }

    #[inline]
    pub fn get(&self, t: usize, f: usize) -> f32 {
        self.mags_sqr[t * self.n_freq + f]
    }
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
    pub time_tolerance_symbols: u32,
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
            // Symbols are 0.3 s; ±5 symbols covers ±1.5 s of timing
            // error, comfortably more than typical PC-clock drift.
            time_tolerance_symbols: 5,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates: 8,
        }
    }
}

/// Score one `(start_row, base_bin)`: sum tone-0 power across the
/// 22 sync positions, divide by `(sum + noise_floor)`.
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    let rows_per_symbol = (spec.nsps / spec.t_step).max(1);
    let last_row = start_row + (Q65_SYNC_POSITIONS[21] as usize) * rows_per_symbol;
    if last_row >= spec.n_time || base_bin >= spec.n_freq {
        return 0.0;
    }
    let mut sync_pwr = 0.0_f32;
    for &sym_idx in &Q65_SYNC_POSITIONS {
        let row = start_row + (sym_idx as usize) * rows_per_symbol;
        sync_pwr += spec.get(row, base_bin);
    }
    let noise_floor = spec.noise_per_bin * Q65_SYNC_POSITIONS.len() as f32;
    sync_pwr / (sync_pwr + noise_floor)
}

/// Build a spectrogram for Q65 sub-mode `P` and find the top sync
/// candidates inside the search window.
pub fn coarse_search_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = Spectrogram::build_for::<P>(audio, sample_rate);
    coarse_search_on_spec_for::<P>(&spec, sample_rate, nominal_start_sample, params)
}

/// Q65-30A convenience wrapper for [`coarse_search_for`].
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    coarse_search_for::<Q65a30>(audio, sample_rate, nominal_start_sample, params)
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

    let t_span_rows = params.time_tolerance_symbols as i64 * rows_per_symbol as i64;
    let nominal_row = (nominal_start_sample / spec.t_step) as i64;
    let row_min = (nominal_row - t_span_rows).max(0);
    let row_max = nominal_row + t_span_rows;

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
    coarse_search_on_spec_for::<Q65a30>(spec, sample_rate, nominal_start_sample, params)
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
