//! Generic 2-D (Δf, Δt) fine-sync refine — port of WSJT-X `sync4d.f90`
//! (FT4) and `fst4_sync_search` (FST4).
//!
//! **FT4** uses [`Sync2dConfig`] / [`sync2d_refine`]: two-pass local
//! refinement around the coarse-sync candidate.
//!
//! **FST4** uses [`fst4_sync_search`]: faithful port of WSJT-X
//! `fst4_decode.f90:879-925`.  Coarse pass sweeps ±1.5 s of the full
//! slot (not just the local coarse_sync window) so the winner is always
//! near the true peak, then a fine pass ±7 × 0.02·baud × ±4 samples
//! locks in.  The local-window approach (`sync2d_refine` with
//! `Sync2dConfig::for_fst4`) caused regression because a noise peak at
//! the edge of the ±10-sample coarse window displaced the fine pass
//! outside the reach of the true position.
//!
//! The output is a [`Sync2dResult`] with refined `(freq_hz, i0, score)`;
//! downstream `symbol_spectra` is invoked on a freq-twiddled `cd0` so
//! per-symbol FFT bins land on the correct tones for the refined carrier.

use alloc::vec::Vec;
use core::f32::consts::PI;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::core::Protocol;
use crate::core::sync::{SyncCandidate, SyncDims, make_costas_ref, score_costas_block};

/// Parameters for [`sync2d_refine`].
///
/// Use [`Sync2dConfig::for_ft4`] rather than constructing directly.
#[derive(Clone, Debug)]
pub struct Sync2dConfig {
    /// Coarse Δf search: `if = -n..=n`, step `coarse_df_step_hz`.
    pub coarse_df_half_steps: i32,
    pub coarse_df_step_hz: f32,
    /// Coarse Δt search: ±`coarse_t_radius` downsampled samples, step `coarse_t_step`.
    pub coarse_t_radius: i32,
    pub coarse_t_step: i32,
    /// Fine Δf search: `if = -n..=n`, step `fine_df_step_hz`.
    pub fine_df_half_steps: i32,
    pub fine_df_step_hz: f32,
    /// Fine Δt search: ±`fine_t_radius` downsampled samples, step 1.
    pub fine_t_radius: i32,
}

impl Sync2dConfig {
    /// FT4 constants from `sync4d.f90`: coarse ±12 Hz/3 Hz × ±20/4 samples;
    /// fine ±4 Hz/1 Hz × ±5 samples.  Bit-identical to the hard-coded
    /// values that were in `sync2d_refine` before this struct was introduced.
    pub fn for_ft4() -> Self {
        Self {
            coarse_df_half_steps: 4, // 12 Hz / 3 Hz
            coarse_df_step_hz: 3.0,
            coarse_t_radius: 20,
            coarse_t_step: 4,
            fine_df_half_steps: 4, // 4 Hz / 1 Hz
            fine_df_step_hz: 1.0,
            fine_t_radius: 5,
        }
    }
}

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

/// Sum-of-blocks Costas correlation power at (i0, df) — twiddled on the fly.
/// Used by `sync2d_refine` where each (df, i0) combination may have a distinct df.
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
        total += score_costas_block(cd0, &twiddled, ds_spb, off);
    }
    total
}

/// Two-pass (Δf, Δt) refinement around an initial coarse candidate.
///
/// Mirrors WSJT-X `sync4d.f90` + `ft4_decode.f90:265-275` (FT4) and
/// `fst4_sync_search` in `fst4_decode.f90:879-925` (FST4).
/// Pass the appropriate [`Sync2dConfig`] for the calling protocol.
pub fn sync2d_refine<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    cfg: &Sync2dConfig,
) -> Sync2dResult {
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
    let mut best_df = 0.0f32;
    let mut best_i0 = init_i0;
    let mut best_score = f32::NEG_INFINITY;

    for si in -cfg.coarse_df_half_steps..=cfg.coarse_df_half_steps {
        let df = si as f32 * cfg.coarse_df_step_hz;
        let mut di = -cfg.coarse_t_radius;
        while di <= cfg.coarse_t_radius {
            let i0 = init_i0 + di;
            let s = score_at::<P>(cd0, &blocks_costas, df, i0, ds_spb, ds_rate);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += cfg.coarse_t_step;
        }
    }

    // Pass 2 — fine grid around coarse winner.
    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;

    for si in -cfg.fine_df_half_steps..=cfg.fine_df_half_steps {
        let df = coarse_winner_df + si as f32 * cfg.fine_df_step_hz;
        let mut di = -cfg.fine_t_radius;
        while di <= cfg.fine_t_radius {
            let i0 = coarse_winner_i0 + di;
            let s = score_at::<P>(cd0, &blocks_costas, df, i0, ds_spb, ds_rate);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += 1;
        }
        // Note: fine Δt step is always 1 (= 1*hmod with hmod=1).
    }

    Sync2dResult {
        freq_hz: candidate.freq_hz + best_df,
        i0: best_i0,
        score: best_score,
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Coherent block correlator — FST4-specific
//
// WSJT-X `sync_fst4` (fst4_decode.f90:657) computes the sync score by:
//   1. Building a phase-continuous FSK reference (`csync1`/`csync2`) for
//      the entire 8-symbol Costas block with continuous phase accumulation
//      across symbol boundaries.
//   2. Computing ONE coherent inner product over all 8*nss samples:
//      z = sum(cd0 * conjg(csynct1))
//   3. Score = |z| / nz   (AMPLITUDE, not power)
//
// Our previous `score_costas_block` computed the SUM OF PER-TONE POWERS:
//   sum_k |inner_product_k|²  (8 separate dot-products, then sum of powers)
// which gives ~3 dB worse SNR discrimination in the sync score vs the
// coherent approach, causing noise peaks to win at near-threshold SNR even
// with the full-slot time search.
// ──────────────────────────────────────────────────────────────────────────

/// Create a phase-continuous FSK reference for one Costas block.
///
/// Unlike `make_costas_ref` (per-tone, phase reset to 0 each symbol),
/// this accumulates phase continuously — matching the actual phase-
/// continuous FSK modulation of the FST4 signal.  Returned length is
/// `pattern.len() * ds_spb`.
fn make_costas_ref_continuous(pattern: &[u8], ds_spb: usize) -> Vec<Complex<f32>> {
    let mut out = Vec::with_capacity(pattern.len() * ds_spb);
    let mut phi = 0.0f64;
    for &tone in pattern {
        let dphi = core::f64::consts::TAU * (tone as f64) / (ds_spb as f64);
        for _ in 0..ds_spb {
            out.push(Complex::new(phi.cos() as f32, phi.sin() as f32));
            phi += dphi;
        }
    }
    out
}

/// Apply a carrier twiddle to a flat continuous reference.
/// Returns a new Vec; if `df_hz ≈ 0` returns the input unchanged.
fn twiddle_flat_ref(flat_ref: &[Complex<f32>], df_hz: f32, ds_rate: f32) -> Vec<Complex<f32>> {
    if df_hz.abs() < f32::EPSILON {
        return flat_ref.to_vec();
    }
    let omega = 2.0 * PI * df_hz / ds_rate;
    flat_ref
        .iter()
        .enumerate()
        .map(|(n, &r)| {
            let p = omega * n as f32;
            r * Complex::new(p.cos(), p.sin())
        })
        .collect()
}

/// Coherent inner product for one Costas block: returns amplitude |z|.
/// Matches WSJT-X: `abs(sum(cd0 * conjg(csynct))) / nz`
/// (normalization by block length omitted here — comparison is relative).
/// Returns 0.0 if the block's samples fall outside `cd0`.
fn score_flat_coherent(cd0: &[Complex<f32>], flat_ref: &[Complex<f32>], cd0_start: i32) -> f32 {
    let np = cd0.len() as i32;
    let len = flat_ref.len() as i32;
    if cd0_start < 0 || cd0_start + len > np {
        return 0.0;
    }
    let s0 = cd0_start as usize;
    let z: Complex<f32> = cd0[s0..s0 + len as usize]
        .iter()
        .zip(flat_ref.iter())
        .map(|(&c, &r)| c * r.conj())
        .sum();
    z.norm()
}

/// FST4-specific sync: faithful port of WSJT-X `fst4_sync_search`
/// (`fst4_decode.f90:879-925`), with the coherent amplitude scorer matching
/// `sync_fst4` (`fst4_decode.f90:657`, `nsyncoh=8`).
///
/// **Scorer**: phase-continuous FSK reference, one inner product per Costas
/// block (8×ds_spb samples), return amplitude |z| and sum across 5 blocks.
/// This is `~3 dB` better SNR discrimination than the per-tone power-sum
/// (`score_costas_block`) at near-threshold SNR.
///
/// **Coarse pass**: ±12 × 0.1·baud Hz × ±1.5 s time range (step 4).
/// Pre-twiddled once per freq offset to avoid per-cell re-allocation.
///
/// **Fine pass**: ±7 × 0.02·baud Hz × ±4 samples around coarse winner,
/// step 1.  `sbest` reset to 0.0 before fine pass (WSJT-X convention).
pub fn fst4_sync_search<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
) -> Sync2dResult {
    let d = SyncDims::of::<P>();
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    let baud = P::TONE_SPACING_HZ;
    let init_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;

    // WSJT-X: ishw = 1.5 * floor(fs2) samples.
    let ishw = (1.5 * ds_rate as f64).floor() as i32;

    // Pre-build flat phase-continuous references for each Costas block.
    // (start_sample_offset, flat_ref)
    let flat_blocks: Vec<(i32, Vec<Complex<f32>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| {
            let off = b.start_symbol as i32 * ds_spb as i32;
            let flat = make_costas_ref_continuous(b.pattern, ds_spb);
            (off, flat)
        })
        .collect();

    // Helper: score all 5 blocks with pre-twiddled refs at given i0.
    let score_flat = |twiddled: &Vec<(i32, Vec<Complex<f32>>)>, i0: i32| -> f32 {
        twiddled
            .iter()
            .map(|(off, flat)| score_flat_coherent(cd0, flat, i0 + off))
            .sum::<f32>()
    };

    // Coarse pass: sweep ±ishw time × ±12×0.1·baud freq.
    let mut best_df = 0.0f32;
    let mut best_i0 = init_i0;
    let mut best_score = f32::NEG_INFINITY;

    for si in -12i32..=12 {
        let df = si as f32 * 0.1 * baud;
        let twiddled: Vec<(i32, Vec<Complex<f32>>)> = flat_blocks
            .iter()
            .map(|(off, flat)| (*off, twiddle_flat_ref(flat, df, ds_rate)))
            .collect();

        let mut di = -ishw;
        while di <= ishw {
            let i0 = init_i0 + di;
            let s = score_flat(&twiddled, i0);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += 4;
        }
    }

    // Fine pass: ±7×0.02·baud Hz × ±4 samples.  WSJT-X resets sbest=0.0.
    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;
    best_score = 0.0;

    for si in -7i32..=7 {
        let df = coarse_winner_df + si as f32 * 0.02 * baud;
        let twiddled: Vec<(i32, Vec<Complex<f32>>)> = flat_blocks
            .iter()
            .map(|(off, flat)| (*off, twiddle_flat_ref(flat, df, ds_rate)))
            .collect();

        for di in -4i32..=4 {
            let i0 = coarse_winner_i0 + di;
            let s = score_flat(&twiddled, i0);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
        }
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
