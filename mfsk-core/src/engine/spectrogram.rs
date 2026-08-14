// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared FFT-based spectrogram builder + tone-0 sync scorer, for
//! protocols whose sync tone sits at a fixed set of symbol positions
//! rather than a block-Costas pattern (JT9, JT65, Q65).
//!
//! Extracted (2026-08-14, code-sharing audit) from three independent,
//! near-identical copies — `jt9::search::Spectrogram`,
//! `jt65::search::Spectrogram`, `q65::search::Spectrogram` — whose own
//! doc comments already cross-referenced each other as "structurally
//! identical" without anyone unifying them. `WSPR` has the same shape
//! FFT loop but is deliberately *not* folded in here: it uses the
//! crate's fixed-point-capable `engine::fft` abstraction rather than
//! raw `rustfft` (WSPR alone among these needs that), and its
//! `score_candidate` normalises against a fitted per-bin baseline
//! (`sbase_linear`) rather than a single flat noise floor — a real
//! algorithmic difference, not just a naming one.
//!
//! `nstep_per_symbol` and the sync-position list are the only axes
//! that actually vary between JT9/JT65/Q65: JT9/JT65 both step at
//! `NSPS/4` (quarter-symbol), Q65 at `NSPS/8` (matching WSJT-X's own
//! `NSTEP=8` sync resolution, `lib/qra/q65/q65.f90:3`); each protocol
//! sums FFT-bin power over its own fixed sync-position list. Both are
//! now explicit parameters instead of a hardcoded constant per module.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
use rustfft::FftPlanner;

use super::ModulationParams;

/// FFT-bin spectrogram covering an audio buffer at `nsps /
/// nstep_per_symbol`-sample time steps. `mags_sqr[t * n_freq + f]` is
/// `|FFT[f]|²` at time step `t`.
pub struct Spectrogram {
    pub mags_sqr: Vec<f32>,
    pub n_time: usize,
    pub n_freq: usize,
    /// Samples between consecutive time rows.
    pub t_step: usize,
    /// FFT window size (samples).
    pub nsps: usize,
    /// Frequency resolution (Hz per bin).
    pub df: f32,
    /// Rough noise-floor estimate (mean of the lower 95% of all cells).
    pub noise_per_bin: f32,
}

impl Spectrogram {
    /// Build a spectrogram for protocol `P` at `nsps / nstep_per_symbol`
    /// time steps. Returns an empty shell if `audio` is shorter than
    /// one symbol.
    pub fn build_for<P: ModulationParams>(
        audio: &[f32],
        sample_rate: u32,
        nstep_per_symbol: usize,
    ) -> Self {
        let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
        let t_step = (nsps / nstep_per_symbol).max(1);
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

        // Noise reference: drop the top 5% (strong bins) and average
        // the rest. Cheap median-ish estimator. Only the *set* of
        // bottom-95% values is needed (order within that set doesn't
        // matter, we just sum them), not a full ascending order —
        // `select_nth_unstable_by` partitions in O(n) average instead
        // of `sort_unstable_by`'s O(n log n).
        let mut sorted = mags_sqr.clone();
        let keep = (sorted.len() as f32 * 0.95) as usize;
        let noise_per_bin = if keep > 0 {
            sorted.select_nth_unstable_by(keep - 1, |a, b| {
                a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)
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

    #[inline]
    pub fn get(&self, t: usize, f: usize) -> f32 {
        self.mags_sqr[t * self.n_freq + f]
    }
}

/// Sum of sync-tone (tone 0) FFT-bin power at `bin`, across
/// `sync_positions`' rows starting at spectrogram row `start_row`.
/// The un-normalised quantity [`score_candidate`] sums before dividing
/// by the noise floor — factored out so a frequency-refine step (e.g.
/// `jt65::search::refine_freq_hz` / `jt9::search::refine_freq_hz`) can
/// read it at the neighbouring bins too.
pub fn sync_power_at_bin(
    spec: &Spectrogram,
    start_row: usize,
    bin: usize,
    sync_positions: &[u32],
    rows_per_symbol: usize,
) -> f32 {
    if bin >= spec.n_freq {
        return 0.0;
    }
    let mut sync_pwr = 0.0f32;
    for &sym_idx in sync_positions {
        let row = start_row + (sym_idx as usize) * rows_per_symbol;
        sync_pwr += spec.get(row, bin);
    }
    sync_pwr
}

/// Score one candidate `(start_row, base_bin)`: sum tone-0 power
/// across `sync_positions`' rows, normalise against the noise floor.
/// `sync_positions` must be sorted ascending (its last element is used
/// to bounds-check the frame against `spec.n_time`).
pub fn score_candidate(
    spec: &Spectrogram,
    start_row: usize,
    base_bin: usize,
    sync_positions: &[u32],
    rows_per_symbol: usize,
) -> f32 {
    let Some(&last_pos) = sync_positions.last() else {
        return 0.0;
    };
    let last_row = start_row + (last_pos as usize) * rows_per_symbol;
    if last_row >= spec.n_time || base_bin >= spec.n_freq {
        return 0.0;
    }
    let sync_pwr = sync_power_at_bin(spec, start_row, base_bin, sync_positions, rows_per_symbol);
    let noise_floor = spec.noise_per_bin * sync_positions.len() as f32;
    sync_pwr / (sync_pwr + noise_floor)
}
