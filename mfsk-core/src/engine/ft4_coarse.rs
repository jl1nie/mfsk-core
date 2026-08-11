//! FT4-specific coarse-candidate stage — faithful port of WSJT-X
//! `getcandidates4.f90` + `ft4_baseline.f90`.
//!
//! Lives in `engine`, not `ft4`, alongside `engine::sync2d::ft4_sync_search`
//! for the same reason that function does: `engine` is compiled regardless
//! of which protocol features are enabled, and `engine::pipeline` needs to
//! call this unconditionally (gated only by a runtime `P::ID == Ft4`
//! check, matching the existing `ft4_sync_search` wiring) — a
//! `crate::ft4::*` reference from `engine` would force the `ft4` feature
//! on for every build.
//!
//! `engine::sync::coarse_sync` (used by every other protocol still on the
//! generic path) is a 2-D (freq × lag) Costas-array correlation search —
//! correct for FT8/FST4/etc, but a structurally different algorithm from
//! what WSJT-X actually does for FT4. `getcandidates4.f90` never searches
//! a lag/Δt dimension at all: it's a pure frequency-domain periodogram —
//! Nuttall-windowed FFT over *overlapping* `NFFT1 = 4×NSPS`-sample
//! segments stepped by one full symbol (`NSTEP=NSPS`, `ft4_params.f90:14`
//! — not the generic function's `NSTEP_PER_SYMBOL`-based fractional
//! step), time-averaged into `savg`, 15-bin boxcar-smoothed into `savsm`,
//! normalised by a 5-term-polynomial baseline (`ft4_baseline.f90`,
//! already a faithful, unit-tested port at
//! [`crate::engine::baseline::fit_baseline`]), and reduced to **one
//! candidate per local-max frequency peak** (parabolic sub-bin
//! interpolation). FT4's actual Δt determination happens entirely later,
//! in the already-faithful [`crate::engine::sync2d::ft4_sync_search`]
//! (an *absolute* full-window coherent search that ignores whatever
//! `dt_sec` a coarse candidate carries).
//!
//! Because of that, the generic function's up-to-8 lag-distinct
//! candidates per frequency bin are functionally redundant downstream for
//! FT4: each independently pays the full `ft4_sync_search` + LLR + BP +
//! OSD cost in `process_candidate_basic` and — for a real signal —
//! converges on the same refined position and decode outcome. Measured
//! on the WSJT-X FT4 golden WAV (`ft4_diag_candidate_cost_split`,
//! `tests/ft4_sweep.rs`): the generic search emits 2000 candidates across
//! only 440 distinct frequencies (4.5× redundancy), and the dominant
//! per-candidate cost by far is `ft4_sync_search` itself (5.09 s summed,
//! vs 1.46 s inferred LLR+BP+OSD, out of ~6.65 s total) — so a faithful
//! one-candidate-per-frequency port should cut wall-clock roughly by that
//! redundancy factor. See `~/.claude/plans/dapper-soaring-nest.md`.
//!
//! Note: `Ft4::SPECTRUM_WINDOW` is `Rectangular` (`ft4/mod.rs`) with a
//! doc comment explaining *why* — Nuttall was tried on the *generic*
//! `coarse_sync` (paired with its crude 40th-percentile floor, not the
//! real polynomial baseline) and reverted for misranking signal bins
//! against sidelobes on the synth-roundtrip tests. That's the
//! mismatched-pairing trap this module avoids: Nuttall windowing here is
//! paired with its *actual* WSJT-X counterpart, `fit_baseline`, not the
//! generic function's unrelated floor estimator.
//!
//! Deviation from WSJT-X, deliberate: `getcandidates4.f90` collects
//! candidates in frequency-scan order and stops once `maxcand` array
//! slots fill (a Fortran fixed-array convenience, not an intentional
//! ranking), then reorders only by nfqso-proximity. This module instead
//! sorts by score (falling back to `freq_hint` proximity, matching
//! `engine::sync::coarse_sync`'s own convention) before truncating to
//! `max_cand` — consistent with how every other caller in this pipeline
//! (dedup, sniper mode) already treats `SyncCandidate::score` as a
//! genuine ranking, not an FFI-array-fill leftover.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::baseline::fit_baseline;
use super::dsp::downsample::with_default_planner;
use super::sync::{SyncCandidate, nuttall_window, parabolic_peak};

/// `NSPS` (samples/symbol at 12 kHz) — `ft4_params.f90:9`.
const NSPS: usize = 576;
/// `NFFT1 = 4×NSPS` — `ft4_params.f90:13`. Four-symbol-wide analysis
/// window (75% overlap at `NSTEP=NSPS`), giving 4× frequency
/// oversampling vs a single-symbol FFT.
const NFFT1: usize = NSPS * 4;
/// `NH1 = NFFT1/2` — positive-frequency bin count.
const NH1: usize = NFFT1 / 2;
/// `NSTEP=NSPS` — full-symbol coarse step (`ft4_params.f90:14`).
/// Deliberately *not* `Ft4::NSTEP_PER_SYMBOL` (that constant belongs to
/// the generic Costas-lag search this module replaces for FT4).
const NSTEP: usize = NSPS;
/// `df = 12000/NFFT1` — `getcandidates4.f90:27`.
const DF_HZ: f32 = 12_000.0 / NFFT1 as f32;
/// `f_offset = -1.5*12000/NSPS` — `getcandidates4.f90:51`.
const F_OFFSET_HZ: f32 = -1.5 * 12_000.0 / NSPS as f32;
/// WSJT-X hardcodes these regardless of caller `fa`/`fb`
/// (`getcandidates4.f90:45,47`).
const FREQ_HARD_MIN_HZ: f32 = 200.0;
const FREQ_HARD_MAX_HZ: f32 = 4910.0;

/// Time-averaged linear-power periodogram, length `NH1`. Matches
/// `getcandidates4.f90:29-38`: Nuttall-windowed `NFFT1`-point FFT over
/// `NSTEP`-strided, `NFFT1`-wide (overlapping) segments of raw 12 kHz
/// PCM, averaged across all segments.
fn symbol_spectra_avg(audio: &[i16]) -> Vec<f32> {
    let window = nuttall_window(NFFT1);
    let fac = 1.0f32 / 300.0;
    let fft = with_default_planner(|planner| planner.plan_forward(NFFT1));

    let nhsym = if audio.len() > NFFT1 {
        (audio.len() - NFFT1) / NSTEP
    } else {
        0
    };

    let mut savg = vec![0.0f32; NH1];
    let mut buf = vec![Complex::new(0.0f32, 0.0); NFFT1];
    let mut count = 0usize;
    for j in 0..nhsym {
        let ia = j * NSTEP;
        if ia + NFFT1 > audio.len() {
            break;
        }
        for k in 0..NFFT1 {
            let sample = audio[ia + k] as f32 * fac * window[k];
            buf[k] = Complex::new(sample, 0.0);
        }
        fft.process(&mut buf);
        for i in 0..NH1 {
            savg[i] += buf[i].norm_sqr();
        }
        count += 1;
    }
    if count > 0 {
        let inv = 1.0 / count as f32;
        for s in savg.iter_mut() {
            *s *= inv;
        }
    }
    savg
}

/// FT4 coarse-candidate stage: one candidate per frequency-domain local
/// peak, faithful to `getcandidates4.f90`. Signature matches
/// `engine::sync::coarse_sync::<Ft4>` — a drop-in replacement at call
/// sites.
///
/// `dt_sec` on every returned candidate is `0.0` — genuinely unused
/// downstream: `engine::sync2d::ft4_sync_search` (called next in
/// `process_candidate_basic` for `P::ID == Ft4`) searches the absolute
/// time window regardless of the candidate's own `dt_sec`.
pub fn ft4_coarse_sync(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let savg = symbol_spectra_avg(audio);

    // 15-bin boxcar smooth (`getcandidates4.f90:39-42`: `savsm(i) =
    // sum(savg(i-7:i+7))/15` for 1-indexed `i` in `[8, NH1-7]`; the
    // 0-indexed equivalent valid range is `[7, NH1-8]`).
    let mut savsm = vec![0.0f32; NH1];
    if NH1 > 14 {
        for i in 7..=(NH1 - 8) {
            savsm[i] = savg[i - 7..=i + 7].iter().sum::<f32>() / 15.0;
        }
    }

    // `nfa`/`nfb`: caller's [freq_min, freq_max] intersected with
    // WSJT-X's own hard 200-4910 Hz bounds (`getcandidates4.f90:44-47`).
    // Clamped to [8, NH1-9] so the smoothing/peak-search neighbour
    // indices (`i-1`, `i+1`) are always in the smoothed range above.
    let nfa = ((freq_min / DF_HZ).round().max(0.0) as usize)
        .max((FREQ_HARD_MIN_HZ / DF_HZ).round() as usize)
        .max(8);
    let nfb = ((freq_max / DF_HZ).round().max(0.0) as usize)
        .min((FREQ_HARD_MAX_HZ / DF_HZ).round() as usize)
        .min(NH1.saturating_sub(9));
    if nfb <= nfa {
        return Vec::new();
    }

    // Baseline fit (`ft4_baseline.f90`, ported at
    // `engine::baseline::fit_baseline`) operates on the raw (unsmoothed)
    // `savg`, same as WSJT-X's `call ft4_baseline(savg,nfa,nfb,sbase)`.
    // `fit_baseline` returns dB (its own documented contract); convert
    // back to linear power to match `ft4_baseline.f90:46`'s
    // `sbase(i)=10**(sbase(i)/10.0)` before dividing `savsm` by it
    // (`getcandidates4.f90:50`).
    let sbase_db = fit_baseline(&savg, nfa, nfb);
    for (k, i) in (nfa..=nfb).enumerate() {
        let sbase_lin = 10f32.powf(sbase_db[k] / 10.0);
        savsm[i] = if sbase_lin > f32::EPSILON {
            savsm[i] / sbase_lin
        } else {
            0.0
        };
    }

    // Local-max peak detection with parabolic sub-bin interpolation
    // (`getcandidates4.f90:54-68`).
    let mut out: Vec<SyncCandidate> = Vec::new();
    for i in (nfa + 1)..nfb {
        let y0 = savsm[i];
        if y0 < savsm[i - 1] || y0 < savsm[i + 1] || y0 < sync_min {
            continue;
        }
        let (del, speak) = parabolic_peak(savsm[i - 1], y0, savsm[i + 1]);
        let fpeak = (i as f32 + del) * DF_HZ + F_OFFSET_HZ;
        if !(FREQ_HARD_MIN_HZ..=FREQ_HARD_MAX_HZ).contains(&fpeak) {
            continue;
        }
        out.push(SyncCandidate {
            freq_hz: fpeak,
            dt_sec: 0.0,
            score: speak,
        });
    }

    // Score-sort (freq_hint-priority first), then truncate — see the
    // module-doc "Deviation from WSJT-X" note. Shares
    // `engine::sync::rank_candidates` with the generic `coarse_sync`
    // so both paths get the same non-starving hint policy (issue
    // #257); this port emits one candidate per frequency peak rather
    // than up to 8 lag peaks per bin, so it could not hit #257's
    // annulus as hard, but there is no reason for the two to disagree.
    crate::engine::sync::rank_candidates(out, freq_hint, max_cand)
}
