//! Precomputed quarter-symbol spectrogram.
//!
//! One overlapping FFT per quarter-symbol time step, cached as a flat
//! row-major `|FFT|²` table. Coarse search then scores candidates in
//! O(162) lookups instead of O(162) FFTs — a ~1000× speedup for the
//! 120-s WSPR slot over the naive candidate-grid loop.
//!
//! Shape at 12 kHz sample rate:
//! - NSPS = 8192, t_step = NSPS/4 = 2048 samples (~171 ms)
//! - n_time ≈ audio_len / 2048 (≈ 700 rows for a full slot)
//! - n_freq = NSPS/2 = 4096 bins (1.4648 Hz each, Nyquist at 6 kHz)
//! - Storage: ~700 × 4096 × 4 bytes ≈ 11 MB per slot.

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::Wspr;

/// Precomputed spectrogram of an audio slot.
pub struct Spectrogram {
    /// Row-major `|FFT|²` table: `mags_sqr[t * n_freq + f]`.
    pub mags_sqr: Vec<f32>,
    pub n_time: usize,
    pub n_freq: usize,
    /// Samples between consecutive time rows.
    pub t_step: usize,
    /// FFT window size (samples).
    pub nsps: usize,
    /// Frequency resolution (Hz per bin).
    pub df: f32,
    /// Mean squared-magnitude of "noise" bins (rough σ² estimator).
    pub noise_per_bin: f32,
}

impl Spectrogram {
    /// Build a quarter-symbol spectrogram matching WSPR's geometry at
    /// `sample_rate`. Empty if the audio is shorter than one symbol.
    pub fn build(audio: &[f32], sample_rate: u32) -> Self {
        let nsps = (sample_rate as f32 * <Wspr as ModulationParams>::SYMBOL_DT).round() as usize;
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

        // Noise reference: mean power across all bins and times,
        // discarding the top 5 % to avoid strong signals dragging the
        // estimate up. Cheap approximation of median-filter noise floor.
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

/// Score a candidate alignment using precomputed spectrogram rows.
/// `t_row` is the spectrogram row of symbol 0; consecutive symbols are
/// four rows apart (because `t_step = nsps/4`). `base_bin` is the FFT
/// bin of tone 0. Returns a score on the same scale as
/// [`super::rx::sync_score`]: ≈ 1.0 at clean alignment, ≈ 0 for empty
/// windows, negative when signal lands in the sync-inconsistent tones.
pub fn score_candidate(spec: &Spectrogram, t_row: usize, base_bin: usize) -> f32 {
    use super::WSPR_SYNC_VECTOR;
    const ROWS_PER_SYMBOL: usize = 4;
    let last_row = t_row + 161 * ROWS_PER_SYMBOL;
    if last_row >= spec.n_time || base_bin + 4 > spec.n_freq {
        return 0.0;
    }
    let mut sync_pwr = 0.0f32;
    let mut off_pwr = 0.0f32;
    for i in 0..162 {
        let t = t_row + i * ROWS_PER_SYMBOL;
        let m0 = spec.get(t, base_bin);
        let m1 = spec.get(t, base_bin + 1);
        let m2 = spec.get(t, base_bin + 2);
        let m3 = spec.get(t, base_bin + 3);
        if WSPR_SYNC_VECTOR[i] == 0 {
            sync_pwr += m0 + m2;
            off_pwr += m1 + m3;
        } else {
            sync_pwr += m1 + m3;
            off_pwr += m0 + m2;
        }
    }
    let noise_floor = spec.noise_per_bin * 162.0;
    let denom = sync_pwr + off_pwr + noise_floor;
    if denom > 0.0 {
        (sync_pwr - off_pwr) / denom
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::super::synthesize_type1;
    use super::*;

    #[test]
    fn spec_matches_direct_demod() {
        // Sanity: score_candidate on the spectrogram should pick the
        // same alignment as the per-candidate FFT loop. We just check
        // that clean synthesis scores highest at the true alignment
        // (bin ≈ 1024, t_row = 0) among a small neighbourhood.
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let spec = Spectrogram::build(&audio, 12_000);
        assert!(spec.n_time > 0);
        assert_eq!(spec.n_freq, 4096);

        let true_bin = 1024;
        let true_t = 0usize;
        let best_score = score_candidate(&spec, true_t, true_bin);
        // Nearby neighbours should all score lower.
        for dt in [-2i32, -1, 1, 2] {
            if let Some(t) = (true_t as i32 + dt).try_into().ok() {
                let s = score_candidate(&spec, t, true_bin);
                assert!(s < best_score, "dt={} scored {} >= {}", dt, s, best_score);
            }
        }
        for df in [-2i32, -1, 1, 2] {
            let b = (true_bin as i32 + df) as usize;
            let s = score_candidate(&spec, true_t, b);
            assert!(s < best_score, "df={} scored {} >= {}", df, s, best_score);
        }
    }
}
