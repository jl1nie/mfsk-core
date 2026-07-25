//! Coarse (frequency × time) search for JT65.
//!
//! JT65 carries its sync tone (tone 0) at 63 positions determined by
//! a pseudo-random 126-bit pattern (`JT65_NPRC`). The coarse search
//! builds an NSPS-sized FFT spectrogram at quarter-symbol steps and
//! scores each candidate (`start_row`, `base_bin`) by summing the
//! FFT-bin power at `base_bin` across the 63 sync-position rows.
//!
//! Reuses the same shape as `crate::jt9::search` — the only differences
//! are the sync-positions list and the per-frame symbol count.

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::sync_pattern::{JT65_NPRC, JT65_SYNC_POSITIONS};
use super::{Jt65, Jt65Shorthand, SyncPolarity};

/// Precomputed per-time-step FFT magnitudes-squared used by the
/// coarse (freq × time) search. Each entry `mags_sqr[t * n_freq + f]`
/// is the squared magnitude of FFT bin `f` at time step `t`.
pub struct Spectrogram {
    /// Row-major `(n_time, n_freq)` grid of `|FFT[k]|²` values.
    pub mags_sqr: Vec<f32>,
    /// Number of time steps (rows).
    pub n_time: usize,
    /// Number of frequency bins per row (`nsps / 2`).
    pub n_freq: usize,
    /// Step size between rows, in samples (`nsps / 4`).
    pub t_step: usize,
    /// FFT window size in samples.
    pub nsps: usize,
    /// Frequency resolution in Hz per bin.
    pub df: f32,
    /// Average noise power per bin — used to normalise scores.
    pub noise_per_bin: f32,
}

impl Spectrogram {
    /// Build the spectrogram from `audio` at quarter-symbol time steps.
    pub fn build(audio: &[f32], sample_rate: u32) -> Self {
        Self::build_for::<Jt65>(audio, sample_rate)
    }

    /// Build the spectrogram for JT65 sub-mode `P`.
    pub fn build_for<P: ModulationParams>(audio: &[f32], sample_rate: u32) -> Self {
        let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
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

    /// Fetch the squared FFT magnitude at time step `t`, bin `f`.
    #[inline]
    pub fn get(&self, t: usize, f: usize) -> f32 {
        self.mags_sqr[t * self.n_freq + f]
    }
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
    /// Inverted sync is the JT65 `OOO` report convention.
    pub polarity: SyncPolarity,
}

/// One candidate for the JT65 two-tone EME shorthands.
#[derive(Clone, Copy, Debug)]
pub struct ShorthandCandidate {
    pub start_sample: usize,
    /// Frequency of shorthand tone 0.
    pub freq_hz: f32,
    pub message: Jt65Shorthand,
    /// Correct-pattern contrast multiplied by low/high-tone balance.
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
    /// ± this many symbol times around the nominal start sample are
    /// tested as candidate frame starts.
    pub time_tolerance_symbols: u32,
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
            // Symbols are long (683 ms); ±3 symbols covers ~2 s drift.
            time_tolerance_symbols: 3,
            score_threshold: DEFAULT_SCORE_THRESHOLD,
            max_candidates: 8,
        }
    }
}

/// Score one candidate using the complete 126-symbol pseudorandom pattern.
///
/// A sum over sync positions alone has broad timing plateaus: shifting a
/// candidate several symbols still overlaps many of the 63 sync tones.
/// Pinned WSJT-X `xcor.f90` instead correlates base-tone power with
/// `2*nprc-1`, so base-tone energy in a data position counts against the
/// alignment. This normalised power contrast is the equivalent statistic.
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    score_candidate_signed(spec, start_row, base_bin).abs()
}

fn score_candidate_signed(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    const ROWS_PER_SYMBOL: usize = 4;
    let last_row = start_row + (JT65_NPRC.len() - 1) * ROWS_PER_SYMBOL;
    if last_row >= spec.n_time || base_bin >= spec.n_freq {
        return 0.0;
    }
    let mut sync_pwr = 0.0f32;
    let mut data_pwr = 0.0f32;
    for (symbol, &is_sync) in JT65_NPRC.iter().enumerate() {
        let power = spec.get(start_row + symbol * ROWS_PER_SYMBOL, base_bin);
        if is_sync == 1 {
            sync_pwr += power;
        } else {
            data_pwr += power;
        }
    }
    let noise_floor = spec.noise_per_bin * JT65_SYNC_POSITIONS.len() as f32;
    ((sync_pwr - data_pwr) / (sync_pwr + data_pwr + noise_floor)).clamp(-1.0, 1.0)
}

fn parabolic_peak_offset(left: f32, center: f32, right: f32) -> f32 {
    let denominator = left - 2.0 * center + right;
    if denominator.abs() <= f32::EPSILON {
        0.0
    } else {
        (0.5 * (left - right) / denominator).clamp(-0.5, 0.5)
    }
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
    coarse_search_for::<Jt65>(audio, sample_rate, nominal_start_sample, params)
}

/// Search for one JT65 sub-mode selected by protocol marker `P`.
pub fn coarse_search_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = Spectrogram::build_for::<P>(audio, sample_rate);
    coarse_search_on_spec_for::<P>(&spec, sample_rate, nominal_start_sample, params)
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
    coarse_search_on_spec_for::<Jt65>(spec, sample_rate, nominal_start_sample, params)
}

/// Search a pre-built spectrogram for one JT65 sub-mode.
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
    let spacing_bins = (P::TONE_SPACING_HZ / df).round() as usize;
    if spacing_bins == 0 {
        return Vec::new();
    }
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
        if row + 125 * rows_per_symbol >= spec.n_time {
            continue;
        }
        for fb in fmin_bin..=fmax_bin {
            if fb < 0 || (fb as usize) + 66 * spacing_bins > spec.n_freq {
                continue;
            }
            let signed_score = score_candidate_signed(spec, row, fb as usize);
            let score = signed_score.abs();
            if score >= params.score_threshold {
                let base_bin = fb as usize;
                let frequency_offset_bins = if base_bin > 0 && base_bin + 1 < spec.n_freq {
                    parabolic_peak_offset(
                        score_candidate_signed(spec, row, base_bin - 1).abs(),
                        score,
                        score_candidate_signed(spec, row, base_bin + 1).abs(),
                    )
                } else {
                    0.0
                };
                let time_offset_rows = if row > 0 && row + 1 + 125 * rows_per_symbol < spec.n_time {
                    parabolic_peak_offset(
                        score_candidate_signed(spec, row - 1, base_bin).abs(),
                        score,
                        score_candidate_signed(spec, row + 1, base_bin).abs(),
                    )
                } else {
                    0.0
                };
                out.push(SyncCandidate {
                    start_sample: ((row as f32 + time_offset_rows) * spec.t_step as f32)
                        .round()
                        .max(0.0) as usize,
                    freq_hz: (fb as f32 + frequency_offset_bins) * df,
                    score,
                    polarity: if signed_score >= 0.0 {
                        SyncPolarity::Normal
                    } else {
                        SyncPolarity::Inverted
                    },
                });
            }
        }
    }
    out.sort_unstable_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut distinct: Vec<SyncCandidate> = Vec::new();
    for candidate in out {
        let duplicate = distinct.iter().any(|previous| {
            previous.polarity == candidate.polarity
                && (previous.freq_hz - candidate.freq_hz).abs() <= 1.5 * df
                && previous.start_sample.abs_diff(candidate.start_sample) <= nsps
        });
        if !duplicate {
            distinct.push(candidate);
            if distinct.len() == params.max_candidates {
                break;
            }
        }
    }
    distinct
}

fn score_shorthand_candidate(
    spec: &Spectrogram,
    start_row: usize,
    base_bin: usize,
    upper_bin: usize,
) -> f32 {
    const ROWS_PER_SYMBOL: usize = 4;
    let last_row = start_row + 125 * ROWS_PER_SYMBOL;
    if last_row >= spec.n_time || upper_bin >= spec.n_freq {
        return 0.0;
    }

    let mut expected_low = 0.0f32;
    let mut expected_high = 0.0f32;
    let mut wrong_low = 0.0f32;
    let mut wrong_high = 0.0f32;
    for symbol in 0..126 {
        let row = start_row + symbol * ROWS_PER_SYMBOL;
        let low = spec.get(row, base_bin);
        let high = spec.get(row, upper_bin);
        if (symbol / 4) % 2 == 0 {
            expected_low += low;
            wrong_high += high;
        } else {
            expected_high += high;
            wrong_low += low;
        }
    }
    let expected = expected_low + expected_high;
    let wrong = wrong_low + wrong_high;
    let noise = 126.0 * spec.noise_per_bin;
    let contrast = ((expected - wrong) / (expected + wrong + noise)).clamp(0.0, 1.0);
    // Merely finding the common lower tone is insufficient to identify
    // which shorthand was sent. Require comparable energy in both tones.
    let balance = 2.0 * expected_low.min(expected_high) / expected.max(f32::EPSILON);
    contrast * balance.clamp(0.0, 1.0)
}

/// Search a pre-built JT65 spectrogram for two-tone EME shorthands.
pub fn coarse_search_shorthand_on_spec_for<P: ModulationParams>(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<ShorthandCandidate> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let spacing_bins = (P::TONE_SPACING_HZ / df).round() as usize;
    if spacing_bins == 0 {
        return Vec::new();
    }

    const ROWS_PER_SYMBOL: usize = 4;
    let tolerance_rows = params.time_tolerance_symbols as i64 * ROWS_PER_SYMBOL as i64;
    let nominal_row = (nominal_start_sample / spec.t_step) as i64;
    let row_min = (nominal_row - tolerance_rows).max(0);
    let row_max = nominal_row + tolerance_rows;
    let minimum_bin = (params.freq_min_hz / df).floor().max(0.0) as usize;
    let maximum_bin = (params.freq_max_hz / df).ceil().max(0.0) as usize;
    let threshold = params.score_threshold.max(0.2);

    let mut candidates = Vec::new();
    for row in row_min..=row_max {
        let row = row as usize;
        if row + 125 * ROWS_PER_SYMBOL >= spec.n_time {
            continue;
        }
        for base_bin in minimum_bin..=maximum_bin.min(spec.n_freq.saturating_sub(1)) {
            for message in [
                Jt65Shorthand::Ro,
                Jt65Shorthand::Rrr,
                Jt65Shorthand::SeventyThree,
            ] {
                let upper_bin = base_bin + message.code() as usize * 10 * spacing_bins;
                let score = score_shorthand_candidate(spec, row, base_bin, upper_bin);
                if score >= threshold {
                    candidates.push(ShorthandCandidate {
                        start_sample: row * spec.t_step,
                        freq_hz: base_bin as f32 * df,
                        message,
                        score,
                    });
                }
            }
        }
    }
    candidates.sort_unstable_by(|left, right| {
        right
            .score
            .partial_cmp(&left.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    candidates.truncate(params.max_candidates);
    candidates
}

/// Build a spectrogram and search for two-tone EME shorthands.
pub fn coarse_search_shorthand_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<ShorthandCandidate> {
    let spec = Spectrogram::build_for::<P>(audio, sample_rate);
    coarse_search_shorthand_on_spec_for::<P>(&spec, sample_rate, nominal_start_sample, params)
}

#[cfg(test)]
mod tests {
    use super::super::{Jt65Shorthand, synthesize_shorthand_for, synthesize_standard};
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

    #[test]
    fn shorthand_search_identifies_all_three_messages() {
        for message in [
            Jt65Shorthand::Ro,
            Jt65Shorthand::Rrr,
            Jt65Shorthand::SeventyThree,
        ] {
            let audio = synthesize_shorthand_for::<Jt65>(message, 12_000, 1_270.0, 0.3);
            let candidates =
                coarse_search_shorthand_for::<Jt65>(&audio, 12_000, 0, &SearchParams::default());
            let best = candidates.first().expect("clean shorthand candidate");
            assert_eq!(best.message, message);
            assert!((best.freq_hz - 1_270.0).abs() <= 4.0);
            assert_eq!(best.start_sample, 0);
            assert!(best.score > 0.8, "{message}: score={}", best.score);
        }
    }
}
