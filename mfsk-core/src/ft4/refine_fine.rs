//! FT4 fine sync — port of WSJT-X `sync4d.f90`.
//!
//! Two-pass refinement of a coarse-sync candidate:
//!
//! - **Coarse**: ±12 Hz / 3 Hz step (9 values) × ±N samples / 4-sample
//!   step. Picks the (Δf, Δt) cell with peak Costas-correlation power
//!   across the four FT4 sync blocks.
//! - **Fine**: ±4 Hz / 1 Hz step × ±5 samples / 1-sample step around
//!   the coarse winner.
//!
//! The output is a tuple of `(refined_freq_hz, refined_i0)` plus the
//! peak score; downstream `symbol_spectra` is invoked on a freq-twiddled
//! `cd0` so per-symbol FFT bins land on the correct tones for the
//! refined carrier.

use alloc::vec::Vec;
use core::f32::consts::PI;

use num_complex::Complex;

use crate::core::Protocol;
use crate::core::sync::{SyncCandidate, SyncDims, make_costas_ref, score_costas_block};

/// Output of [`sync4d_refine`].
#[derive(Clone, Debug)]
pub struct Sync4dResult {
    /// Refined carrier frequency in Hz (= initial + Δf_best).
    pub freq_hz: f32,
    /// Refined symbol-0 sample offset in `cd0` (signed; negative means
    /// the frame nominally started before sample 0 of the baseband).
    pub i0: i32,
    /// Peak sync power summed across all Costas blocks at (Δf, Δt)_best.
    pub score: f32,
}

/// Apply a complex-phasor freq twiddle to a Costas reference vector.
/// `df_hz` is the offset *added to the carrier hypothesis*, so the
/// reference rotates at `+df_hz` and the cross-correlation against
/// `cd0` (mixed at the original carrier) effectively shifts the
/// hypothesis to `f0 + df_hz`.
fn twiddle_ref(csync: &[Vec<Complex<f32>>], df_hz: f32, ds_rate: f32) -> Vec<Vec<Complex<f32>>> {
    if df_hz.abs() < f32::EPSILON {
        return csync.to_vec();
    }
    let omega = 2.0 * PI * df_hz / ds_rate;
    csync
        .iter()
        .enumerate()
        .map(|(k, tone_wave)| {
            // Phase continuity across tone-symbol boundaries: each
            // tone in the reference occupies `ds_spb` samples, so
            // sample index within block is `k * ds_spb + j`.
            let ds_spb = tone_wave.len();
            let mut out = vec![Complex::new(0.0f32, 0.0); ds_spb];
            for (j, slot) in out.iter_mut().enumerate() {
                let n = (k * ds_spb + j) as f32;
                let p = omega * n;
                let twid = Complex::new(p.cos(), p.sin());
                *slot = tone_wave[j] * twid;
            }
            out
        })
        .collect()
}

/// Sum-of-blocks Costas correlation power at (i0, df).
fn score_at<P: Protocol>(
    cd0: &[Complex<f32>],
    blocks_costas: &[(u32, Vec<Vec<Complex<f32>>>)],
    df_hz: f32,
    i0: i32,
    ds_spb: usize,
    ds_rate: f32,
) -> f32 {
    let _ = P::NTONES; // silence unused-bound warning
    let mut total = 0.0f32;
    for (start_sym, csync) in blocks_costas {
        let twiddled = twiddle_ref(csync, df_hz, ds_rate);
        let off = i0 + (*start_sym as i32) * ds_spb as i32;
        // `score_costas_block` zero-fills any block whose samples fall
        // outside cd0 (matching WSJT-X `sync8d.f90:43-45`), so negative
        // `off` is safe to pass through.
        total += score_costas_block(cd0, &twiddled, ds_spb, off);
    }
    total
}

/// Two-pass (Δf, Δt) refinement around an initial coarse candidate.
///
/// Mirrors WSJT-X `sync4d.f90` + the search loop in
/// `ft4_decode.f90:265-275`.
pub fn sync4d_refine<P: Protocol>(cd0: &[Complex<f32>], candidate: &SyncCandidate) -> Sync4dResult {
    let d = SyncDims::of::<P>();
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    let init_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;

    let blocks_costas: Vec<(u32, Vec<Vec<Complex<f32>>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| (b.start_symbol, make_costas_ref(b.pattern, ds_spb)))
        .collect();

    // Pass 1 — coarse grid.
    const COARSE_DF_RADIUS: f32 = 12.0;
    const COARSE_DF_STEP: f32 = 3.0;
    const COARSE_T_RADIUS: i32 = 20; // ds samples; ≈30 ms at 666.67 Hz
    const COARSE_T_STEP: i32 = 4;

    let mut best_df = 0.0f32;
    let mut best_i0 = init_i0;
    let mut best_score = f32::NEG_INFINITY;

    let mut df = -COARSE_DF_RADIUS;
    while df <= COARSE_DF_RADIUS + 1e-3 {
        let mut di = -COARSE_T_RADIUS;
        while di <= COARSE_T_RADIUS {
            let i0 = init_i0 + di;
            let s = score_at::<P>(cd0, &blocks_costas, df, i0, ds_spb, ds_rate);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += COARSE_T_STEP;
        }
        df += COARSE_DF_STEP;
    }

    // Pass 2 — fine grid around best.
    const FINE_DF_RADIUS: f32 = 4.0;
    const FINE_DF_STEP: f32 = 1.0;
    const FINE_T_RADIUS: i32 = 5;
    const FINE_T_STEP: i32 = 1;

    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;

    let mut df = coarse_winner_df - FINE_DF_RADIUS;
    while df <= coarse_winner_df + FINE_DF_RADIUS + 1e-3 {
        let mut di = -FINE_T_RADIUS;
        while di <= FINE_T_RADIUS {
            let i0 = coarse_winner_i0 + di;
            let s = score_at::<P>(cd0, &blocks_costas, df, i0, ds_spb, ds_rate);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += FINE_T_STEP;
        }
        df += FINE_DF_STEP;
    }

    Sync4dResult {
        freq_hz: candidate.freq_hz + best_df,
        i0: best_i0,
        score: best_score,
    }
}

/// Apply a complex-phasor freq shift to `cd0`. Used by callers that
/// take the [`Sync4dResult::freq_hz`] from this module and want to
/// run [`crate::core::llr::symbol_spectra`] on a baseband whose
/// carrier sits at the refined freq.
pub fn freq_shift_cd0(cd0: &[Complex<f32>], df_hz: f32, ds_rate: f32) -> Vec<Complex<f32>> {
    if df_hz.abs() < f32::EPSILON {
        return cd0.to_vec();
    }
    let omega = -2.0 * PI * df_hz / ds_rate;
    cd0.iter()
        .enumerate()
        .map(|(n, &c)| {
            let p = omega * n as f32;
            c * Complex::new(p.cos(), p.sin())
        })
        .collect()
}
