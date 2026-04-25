// SPDX-License-Identifier: GPL-3.0-or-later
//! Coarse (frequency × time) sync search for Q65.
//!
//! Q65's distributed sync (22 symbols all on tone 0) is the
//! cleanest correlation target across the WSJT family — every sync
//! symbol carries the full symbol energy on the same frequency bin.
//! This module mirrors the structure of [`crate::jt65::search`]:
//! build an NSPS-sized spectrogram at half-symbol time steps,
//! score each candidate `(start_row, base_bin)` by summing the
//! tone-0 power across the 22 sync positions, and return the
//! top-scoring candidates.

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::Q65a30;
use super::sync_pattern::Q65_SYNC_POSITIONS;

/// FFT-bin spectrogram covering the audio buffer at half-symbol
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
    /// Build the spectrogram from `audio`. Time step = `nsps / 2`
    /// (match Q65a30's `NSTEP_PER_SYMBOL = 2`); frequency resolution
    /// = `sample_rate / nsps` ≈ tone spacing.
    pub fn build(audio: &[f32], sample_rate: u32) -> Self {
        let nsps = (sample_rate as f32 * <Q65a30 as ModulationParams>::SYMBOL_DT).round() as usize;
        let t_step = nsps / 2;
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
        // FFT magnitudes (rejects strong narrow-band signals).
        let mut sorted = mags_sqr.clone();
        sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let keep = (sorted.len() as f32 * 0.95) as usize;
        let noise_per_bin = if keep > 0 {
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
    const ROWS_PER_SYMBOL: usize = 2;
    let last_row = start_row + (Q65_SYNC_POSITIONS[21] as usize) * ROWS_PER_SYMBOL;
    if last_row >= spec.n_time || base_bin >= spec.n_freq {
        return 0.0;
    }
    let mut sync_pwr = 0.0_f32;
    for &sym_idx in &Q65_SYNC_POSITIONS {
        let row = start_row + (sym_idx as usize) * ROWS_PER_SYMBOL;
        sync_pwr += spec.get(row, base_bin);
    }
    let noise_floor = spec.noise_per_bin * Q65_SYNC_POSITIONS.len() as f32;
    sync_pwr / (sync_pwr + noise_floor)
}

/// Build a spectrogram and find the top sync candidates inside the
/// search window.
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = Spectrogram::build(audio, sample_rate);
    coarse_search_on_spec(&spec, sample_rate, nominal_start_sample, params)
}

/// Same as [`coarse_search`] but accepts a pre-built spectrogram —
/// useful when the same audio is scanned under multiple parameter
/// sets.
pub fn coarse_search_on_spec(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * <Q65a30 as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let rows_per_symbol = 2_usize;

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
        // Need room for the last data symbol (84) + the 64 data
        // tones above the sync bin.
        if row + 84 * rows_per_symbol >= spec.n_time {
            continue;
        }
        for fb in fmin_bin..=fmax_bin {
            // Tone 64 (highest data tone) sits at base_bin + 64.
            if fb < 0 || (fb as usize) + 65 > spec.n_freq {
                continue;
            }
            let score = score_candidate(spec, row, fb as usize);
            if score >= params.score_threshold {
                out.push(SyncCandidate {
                    start_sample: row * spec.t_step,
                    freq_hz: fb as f32 * df,
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
