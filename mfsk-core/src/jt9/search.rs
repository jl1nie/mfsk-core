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

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::Jt9;
use super::sync_pattern::JT9_SYNC_POSITIONS;

/// One-symbol-FFT spectrogram, reusable across many candidate scores.
pub struct Spectrogram {
    /// Row-major `|FFT|²` table: `mags_sqr[row * n_freq + bin]`.
    pub mags_sqr: Vec<f32>,
    pub n_time: usize,
    pub n_freq: usize,
    /// Samples per spectrogram row.
    pub t_step: usize,
    /// FFT window size (= NSPS).
    pub nsps: usize,
    /// Hz per bin (= tone spacing by construction).
    pub df: f32,
    /// Rough noise-floor estimate (mean of the lower 95 % of all cells).
    pub noise_per_bin: f32,
}

impl Spectrogram {
    /// Build a quarter-symbol spectrogram for JT9. Returns an empty
    /// shell if the audio is shorter than one symbol.
    pub fn build(audio: &[f32], sample_rate: u32) -> Self {
        let nsps = (sample_rate as f32 * <Jt9 as ModulationParams>::SYMBOL_DT).round() as usize;
        let t_step = nsps / 4;
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

        // Noise reference: drop the top 5 % (strong bins) and average
        // the rest. Cheap median-ish estimator.
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
    /// ± symbols around `nominal_start_sample`. JT9 tx offset is ≤ 1 s
    /// (~1.7 symbols); default 3 covers the common drift cases.
    pub time_tolerance_symbols: u32,
    pub score_threshold: f32,
    pub max_candidates: usize,
}

impl Default for SearchParams {
    fn default() -> Self {
        Self {
            freq_min_hz: 1400.0,
            freq_max_hz: 1600.0,
            time_tolerance_symbols: 3,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates: 8,
        }
    }
}

/// Score one candidate using the precomputed spectrogram.
///
/// `start_row` is the spectrogram row index of symbol 0. Because the
/// spectrogram step is NSPS/4, consecutive symbols are 4 rows apart.
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    const ROWS_PER_SYMBOL: usize = 4;
    // Last sync position uses row offset SYNC_POSITIONS[15] * 4.
    let last_row = start_row + (JT9_SYNC_POSITIONS[15] as usize) * ROWS_PER_SYMBOL;
    if last_row >= spec.n_time || base_bin >= spec.n_freq {
        return 0.0;
    }
    let mut sync_pwr = 0.0f32;
    for &sym_idx in &JT9_SYNC_POSITIONS {
        let row = start_row + (sym_idx as usize) * ROWS_PER_SYMBOL;
        sync_pwr += spec.get(row, base_bin);
    }
    // Normalise against the expected noise floor at tone 0 over
    // 16 sync symbols. Score saturates near 1 for clean signals.
    let noise_floor = spec.noise_per_bin * JT9_SYNC_POSITIONS.len() as f32;
    sync_pwr / (sync_pwr + noise_floor)
}

/// Sweep (freq × time) and return top-scored candidates.
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = Spectrogram::build(audio, sample_rate);
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
        // Need row + 84 symbols × 4 rows/symbol room.
        if row + 84 * rows_per_symbol >= spec.n_time {
            continue;
        }
        for fb in fmin_bin..=fmax_bin {
            if fb < 0 || (fb as usize) + 9 > spec.n_freq {
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
