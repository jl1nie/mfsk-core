//! Generic 2-D (Δf, Δt) fine-sync refine — port of WSJT-X `sync4d.f90`,
//! generalised so it isn't FT4-specific.
//!
//! Two-pass refinement of a coarse-sync candidate:
//!
//! - **Coarse**: ±`coarse_df_radius` Hz / `coarse_df_step` Hz-step ×
//!   ±`coarse_t_radius` / `coarse_t_step`-sample window. Picks the
//!   (Δf, Δt) cell with peak Costas-correlation power across the
//!   protocol's sync blocks.
//! - **Fine**: ±`fine_df_radius` Hz / `fine_df_step` Hz-step ×
//!   ±`fine_t_radius` / `fine_t_step`-sample window around the coarse
//!   winner.
//!
//! The radius/step constants are derived from `P::TONE_SPACING_HZ`
//! and `SyncDims::ds_spb` rather than hardcoded absolutes, so the same
//! code serves every T/R-period sub-mode of a chained-frame protocol
//! (FST4's tone spacing spans 0.56–16.67 Hz across its five sub-modes)
//! as well as fixed-geometry protocols like FT4. The ratios are
//! calibrated against FT4's original hand-tuned constants (12 Hz /
//! 3 Hz / 20 samples / 4 samples coarse, 4 Hz / 1 Hz / 5 samples / 1
//! sample fine, at FT4's TONE_SPACING_HZ=20.833 / ds_spb=32) so FT4's
//! behaviour is bit-identical to before this became generic — see
//! `ratio` constants below.
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

/// Output of [`sync2d_refine`].
#[derive(Clone, Debug)]
pub struct Sync2dResult {
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
            let mut out = alloc::vec![Complex::new(0.0f32, 0.0); ds_spb];
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

/// FT4-calibrated ratios (see module doc): coarse/fine Δf radius and
/// step as a fraction of `P::TONE_SPACING_HZ`, and coarse/fine Δt
/// radius/step as a fraction of `SyncDims::ds_spb`. Reproduces FT4's
/// original hardcoded constants exactly (TONE_SPACING_HZ=20.833,
/// ds_spb=32): 12/3/4Hz-radii-and-steps, 20/4/5/1-sample windows.
mod ratio {
    pub const COARSE_DF_RADIUS: f32 = 12.0 / 20.833;
    pub const COARSE_DF_STEP: f32 = 3.0 / 20.833;
    pub const COARSE_T_RADIUS: f32 = 20.0 / 32.0;
    pub const COARSE_T_STEP: f32 = 4.0 / 32.0;
    pub const FINE_DF_RADIUS: f32 = 4.0 / 20.833;
    pub const FINE_DF_STEP: f32 = 1.0 / 20.833;
    pub const FINE_T_RADIUS: f32 = 5.0 / 32.0;
}

/// Two-pass (Δf, Δt) refinement around an initial coarse candidate.
///
/// Mirrors WSJT-X `sync4d.f90` + the search loop in
/// `ft4_decode.f90:265-275` (FT4) / `fst4_sync_search` in
/// `fst4_decode.f90:879-925` (FST4) — both are the same two-level
/// local (Δf, Δt) search shape, scaled to each protocol's own tone
/// spacing / symbol duration.
pub fn sync2d_refine<P: Protocol>(cd0: &[Complex<f32>], candidate: &SyncCandidate) -> Sync2dResult {
    let d = SyncDims::of::<P>();
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    let tone_spacing = P::TONE_SPACING_HZ;
    let init_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;

    let blocks_costas: Vec<(u32, Vec<Vec<Complex<f32>>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| (b.start_symbol, make_costas_ref(b.pattern, ds_spb)))
        .collect();

    // Pass 1 — coarse grid.
    let coarse_df_radius = ratio::COARSE_DF_RADIUS * tone_spacing;
    let coarse_df_step = ratio::COARSE_DF_STEP * tone_spacing;
    let coarse_t_radius = (ratio::COARSE_T_RADIUS * ds_spb as f32).round() as i32;
    let coarse_t_step = ((ratio::COARSE_T_STEP * ds_spb as f32).round() as i32).max(1);

    let mut best_df = 0.0f32;
    let mut best_i0 = init_i0;
    let mut best_score = f32::NEG_INFINITY;

    let mut df = -coarse_df_radius;
    while df <= coarse_df_radius + 1e-3 {
        let mut di = -coarse_t_radius;
        while di <= coarse_t_radius {
            let i0 = init_i0 + di;
            let s = score_at::<P>(cd0, &blocks_costas, df, i0, ds_spb, ds_rate);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += coarse_t_step;
        }
        df += coarse_df_step;
    }

    // Pass 2 — fine grid around best.
    let fine_df_radius = ratio::FINE_DF_RADIUS * tone_spacing;
    let fine_df_step = ratio::FINE_DF_STEP * tone_spacing;
    let fine_t_radius = (ratio::FINE_T_RADIUS * ds_spb as f32).round() as i32;
    const FINE_T_STEP: i32 = 1;

    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;

    let mut df = coarse_winner_df - fine_df_radius;
    while df <= coarse_winner_df + fine_df_radius + 1e-3 {
        let mut di = -fine_t_radius;
        while di <= fine_t_radius {
            let i0 = coarse_winner_i0 + di;
            let s = score_at::<P>(cd0, &blocks_costas, df, i0, ds_spb, ds_rate);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += FINE_T_STEP;
        }
        df += fine_df_step;
    }

    Sync2dResult {
        freq_hz: candidate.freq_hz + best_df,
        i0: best_i0,
        score: best_score,
    }
}

/// Apply a complex-phasor freq shift to `cd0`. Used by callers that
/// take the [`Sync2dResult::freq_hz`] from this module and want to
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
