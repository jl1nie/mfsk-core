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

/// FT4-specific sync: coherent full-slot Δt search, faithful port of
/// WSJT-X `ft4_decode.f90`'s `isync=1`/`isync=2` loop (`sync4d.f90` scorer)
/// — added 2026-07-18 after a diagnostic
/// (`tests/ft4_coherent_wide_search_diag.rs`) confirmed the hypothesis:
/// `core::sync::coarse_sync`'s non-coherent (power-spectrogram) Δt
/// estimate can be wrong by more than a second under CCIR fading, and
/// the previous local `sync2d_refine` (`Sync2dConfig::for_ft4`, ±20
/// downsampled samples ≈ ±30 ms) could never recover from an error that
/// large — even though the true peak's *coherent* score was consistently
/// higher than whatever the non-coherent stage picked instead.
///
/// **Scorer**: one coherent dot product per FT4 Costas block (4 blocks:
/// symbols 0, 33, 66, 99), magnitude-summed across blocks — matches
/// `sync4d.f90`'s `sync = p(z1)+p(z2)+p(z3)+p(z4)` (`p(z)=|z*fac|`,
/// magnitude not power) where each `z_k` is itself ONE coherent dot
/// product spanning all 4 symbols of block k
/// (`z1=sum(cd0(i1:i1+4*NSS-1:2)*conjg(csync2))`, `sync4d.f90:64`) — i.e.
/// coherent *within* each block, magnitude-summed (non-coherent) *across*
/// the 4 blocks, the same combining style [`fst4_sync_search`] already
/// uses. [`ft4_sync_search_window`] twiddles each candidate `(Δf, Δt)`
/// cell's dot product inline rather than calling `score_flat_coherent`
/// on a pre-twiddled reference (as an earlier revision did) — same
/// arithmetic, but avoids re-allocating a twiddled `Vec<Complex<f32>>`
/// per block on every one of the ~19,900 grid cells the coarse+fine
/// passes evaluate. **Originally shipped using `score_costas_block`**
/// (per-symbol power-sum, correct for FT8's `sync8d.f90` — verified
/// against `/home/minoru/src/WSJT-X/lib/ft8/sync8d.f90`, which really
/// does non-coherent per-symbol power summing) — a ~3 dB-class
/// discrimination gap at near-threshold SNR (same mechanism as the
/// FST4 fix, issue #146), caught during the issue #72 AWGN-gap
/// diagnostic (`docs/notes/FT4_BENCHMARK.md` section 9) by reading
/// `sync4d.f90`'s inner `z1=sum(...)` line rather than stopping at the
/// outer `sync=p(z1)+...` formula that (correctly) matched at a glance.
///
/// **Coarse pass**: ±12 Hz / 3 Hz step (`ft4_decode.f90` isync=1:
/// `idfmin=-12,idfmax=12,idfstp=3`) × a *fixed absolute* Δt window,
/// step 4 downsampled samples (`ibstp=4`). The absolute window
/// `[-344, 1012]` downsampled samples is WSJT-X's combined 3-segment
/// coverage (`iseg=1..3`, `ibmin`/`ibmax` per segment) collapsed into
/// one pass — deliberately centred on the *nominal* frame position
/// (`i0` for `dt_sec=0`), not on `candidate.dt_sec`, since that
/// non-coherent estimate is exactly what this function exists to
/// override.
///
/// **Fine pass**: ±4 Hz / 1 Hz × ±5 samples step 1 around the coarse
/// winner (`ft4_decode.f90` isync=2).
pub fn ft4_sync_search<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
) -> Sync2dResult {
    // WSJT-X `ft4_decode.f90`: iseg=1 ibmin=108/ibmax=560, iseg=2
    // ibmin=560/ibmax=1012, iseg=3 ibmin=-344/ibmax=108 — union is
    // [-344, 1012], an absolute downsampled-sample range independent of
    // any candidate dt guess. Collapsed into one pass here (see module
    // doc above `ft4_sync_search`) rather than WSJT-X's literal 3-segment
    // loop with a per-segment decode attempt — [`ft4_sync_search_window`]
    // exposes the windowed search directly for diagnosing whether that
    // collapse loses anything (issue #72, `FT4_BENCHMARK.md` section 11).
    ft4_sync_search_window::<P>(cd0, candidate, -344, 1012)
}

/// Same coherent full-slot Δt search as [`ft4_sync_search`], but over an
/// explicit `[ib_min, ib_max]` downsampled-sample window instead of the
/// hardcoded full-union range. Lets callers (tests, diagnostics) replicate
/// WSJT-X's literal per-segment search — `ft4_decode.f90`'s `iseg=1,2,3`
/// loop, each with its own `ibmin`/`ibmax` — to check whether the
/// collapsed single-pass search in [`ft4_sync_search`] ever misses a
/// position that a per-segment search plus a per-segment decode attempt
/// would have found.
pub fn ft4_sync_search_window<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    ib_min: i32,
    ib_max: i32,
) -> Sync2dResult {
    let d = SyncDims::of::<P>();
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    const COARSE_DT_STEP: i32 = 4;

    let blocks_ref: Vec<(i32, Vec<Complex<f32>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| {
            let off = b.start_symbol as i32 * ds_spb as i32;
            (off, make_costas_ref_continuous(b.pattern, ds_spb))
        })
        .collect();

    // Twiddle on the fly inside the score rather than materialising a
    // fresh `Vec<Complex<f32>>` per block per (Δf, Δt) grid cell — the
    // coarse+fine passes below evaluate ~19,900 cells, so the old
    // per-cell `twiddle_flat_ref` allocation added up to ~90 heap
    // allocations per `ft4_sync_search_window` call (Gemini PR review).
    // Mathematically identical: `score_flat_coherent` on a pre-twiddled
    // reference is the same dot product as twiddling each sample of
    // `cd0` in place here.
    let score_at = |i0: i32, df: f32| -> f32 {
        blocks_ref
            .iter()
            .map(|(off, flat)| {
                let cd0_start = i0 + off;
                let np = cd0.len() as i32;
                let len = flat.len() as i32;
                if cd0_start < 0 || cd0_start + len > np {
                    return 0.0;
                }
                let s0 = cd0_start as usize;
                if df.abs() < f32::EPSILON {
                    let z: Complex<f32> = cd0[s0..s0 + len as usize]
                        .iter()
                        .zip(flat.iter())
                        .map(|(&c, &r)| c * r.conj())
                        .sum();
                    z.norm()
                } else {
                    let omega = -2.0 * PI * df / ds_rate;
                    let z: Complex<f32> = cd0[s0..s0 + len as usize]
                        .iter()
                        .zip(flat.iter())
                        .enumerate()
                        .map(|(n, (&c, &r))| {
                            let p = omega * n as f32;
                            let twid = Complex::new(p.cos(), p.sin());
                            c * r.conj() * twid
                        })
                        .sum();
                    z.norm()
                }
            })
            .sum::<f32>()
    };

    let mut best_df = 0.0f32;
    let mut best_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;
    let mut best_score = f32::NEG_INFINITY;

    let mut idf = -12i32;
    while idf <= 12 {
        let df = idf as f32;
        let mut i0 = ib_min;
        while i0 <= ib_max {
            let s = score_at(i0, df);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            i0 += COARSE_DT_STEP;
        }
        idf += 3;
    }

    // Fine pass around the coarse winner.
    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;
    best_score = f32::NEG_INFINITY;

    for si in -4i32..=4 {
        let df = coarse_winner_df + si as f32;
        for di in -5i32..=5 {
            let i0 = coarse_winner_i0 + di;
            let s = score_at(i0, df);
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
