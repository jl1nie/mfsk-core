//! FST4's real SNR formula — port of `get_candidates_fst4.f90`'s
//! baseline extraction (not its CLEAN candidate-detection loop) and
//! `fst4_decode.f90:585-621`'s `xsnr` formula (issue #255).
//!
//! Wired into `fst4/decode.rs`'s `GenericPipelineProtocol::snr_db`
//! override. Verified against a real local `jt9 -7 -d3` build's own
//! probed values (`WSJT-X/samples/FST4+FST4W/210115_0058.wav`,
//! `SNRAUDIT_FST4_PROBE`/`SNRAUDIT_FST4_BM` instrumentation added to
//! `fst4_decode.f90` for this investigation, not committed there —
//! issue #255's stated verification discipline):
//!
//! | candidate | jt9 `xsnr` | this module's `xsnr` |
//! |-----------|------------|-----------------------|
//! | N5TM      | -6.90 dB   | -8.61 dB              |
//! | K9KFR     | 16.14 dB   | 16.82 dB              |
//!
//! Getting there took two corrections past the naive port:
//!
//! 1. **RMS-normalisation mismatch.** This crate's shared pipeline
//!    RMS-normalises the downsampled baseband before
//!    [`crate::engine::llr::symbol_spectra`] ever runs (needed for
//!    `compute_llr`'s scale calibration elsewhere, issue #18);
//!    WSJT-X's own FST4 path never does this. Left as-is, `xsig` came
//!    out ~constant across both real signals despite their ~23 dB
//!    real SNR difference. Fixed by [`fst4_raw_cs`] rebuilding `cs`
//!    fresh from `fft_cache` without that normalisation step, rather
//!    than trying to algebraically undo it on the already-normalised
//!    version — see its own doc comment for why an earlier "multiply
//!    by the mean pre-normalisation power" attempt wasn't quite right
//!    either (it's algebraically exact only when the fine-refine
//!    frequency offset is negligible, which happened to be true for
//!    both of this file's candidates but isn't guaranteed in general).
//! 2. **Downsample scale convention mismatch.** Even with (1) fixed,
//!    both candidates' `xsig` were still short by a *consistent*
//!    ~100-115× in power — confirmed via a deeper `SNRAUDIT_FST4_BM`
//!    probe comparing individual `s4(tone,symbol)` values directly
//!    (not just the final `xsnr`), which is what separated "one
//!    missing constant factor" from the false impression a coarser,
//!    dB-space-only comparison first gave (issue #255's earlier
//!    investigation pass mis-read this as a *non-constant* residual
//!    gap — an artifact of comparing highly nonlinear final `dB`
//!    values instead of the underlying linear-scale quantities).
//!    Root cause: `downsample_cached`'s `fac = 1/sqrt(fft1_size·
//!    fft2_size)` pre-scale vs. WSJT-X's own `fst4_downsample`'s
//!    `c1 = c1/nfft2` — different normalisation conventions for
//!    otherwise-equivalent unnormalised-IDFT downsamples. Worked out
//!    analytically (not fitted) to an exact `NDOWN`-in-power
//!    correction — see [`fst4_snr_db`]'s own doc comment for the
//!    derivation — which lines up with the empirical 100-115× (vs.
//!    `NDOWN=108` for FST4-60) to within the residual `fft1_size`
//!    padding difference between the two implementations.
//!
//! Both real off-air signals above are FST4-60 — the only real-signal
//! FST4 sample WAV available locally
//! (`WSJT-X/samples/FST4+FST4W/210115_0058.wav`). Fst4s15/30/120/300
//! share this exact formula and the same `NDOWN`-in-power derivation
//! (§2 above doesn't special-case FST4-60's own `NDOWN`; it's a
//! function of each sub-mode's own `NDOWN`/`fft1_size`/`fft2_size`),
//! but haven't themselves been checked against a real `jt9` decode —
//! worth a real-WAV cross-check for at least one other sub-mode if
//! one ever turns up.
//!
//! ## What's ported, what isn't (Phase 4a vs 4b)
//!
//! WSJT-X's `get_candidates_fst4` computes a *whole-band* noise
//! baseline once per decode (`[nfa,nfb]`, run through the same
//! percentile+polyfit machinery as [`crate::engine::baseline`], see
//! [`crate::engine::baseline::BaselineParams::FST4`]), then uses the
//! CLEAN algorithm (iterative peak-find + subtract) to enumerate
//! candidates, recording each one's `sbase(iploc)` as
//! `candidates(icand,5)`.
//!
//! This module ports the baseline math but **not** the CLEAN
//! candidate search — this crate already has its own coarse-candidate
//! search ([`crate::engine::sync::coarse_sync`] /
//! [`crate::engine::sync2d::fst4_sync_search`]), and per the issue
//! #255 plan, replacing that with a from-scratch CLEAN port is a
//! second, larger, independent piece of work (issue #255 §4b) not
//! undertaken here. Instead, [`fst4_baseline_lin`] fits the same
//! percentile+polyfit baseline over a **local window** around the
//! specific candidate frequency our own search already found, rather
//! than the whole `[nfa,nfb]` band WSJT-X fits once and reuses for
//! every candidate. This is a legitimate simplification, not a
//! shortcut that changes the answer: the polynomial only ever
//! describes the *local* noise-floor trend near the point it's
//! evaluated at (`NSEG=8` segments, `NTERMS=3` — a low-order fit with
//! no long-range memory), so fitting it over a captured local span
//! instead of the full user-selected band gives the same baseline
//! value at the candidate's own bin. Revisit with a real 4b CLEAN
//! port if accuracy against ground truth (real `jt9`, golden WAVs)
//! turns out to need the whole-band version specifically.

extern crate alloc;
use alloc::vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::baseline::{BaselineParams, fit_baseline_with};
use crate::engine::dsp::downsample::{DownsampleCfg, downsample_cached};

/// Half-width (Hz) of the local frequency window used to fit the
/// noise baseline around one candidate — the Phase-4a simplification
/// described in this module's doc comment. Wide enough to give
/// `BaselineParams::FST4`'s `NSEG=8` segments plenty of points
/// (hundreds to thousands of low-resolution bins per side across
/// every wired sub-mode) while staying well inside WSJT-X's own
/// typical `[nfa,nfb]` user-selected search band (order 1-5 kHz).
const LOCAL_BASELINE_HALF_WIDTH_HZ: f32 = 400.0;

/// Modulation index — fixed at 1 for every FST4 sub-mode this crate
/// wires (`fst4_decode.f90`'s `data hmod/1/`, never reassigned for
/// sub-mode A; B/C/E variants with other `hmod` values aren't
/// implemented here, see `fst4/mod.rs`'s module doc).
const HMOD: i64 = 1;

/// `candidates(icand,5)`-equivalent: the linear noise baseline at
/// `cand_freq_hz`, fit from `fft_cache` (WSJT-X `c_bigfft` — the same
/// whole-slot big FFT already computed for downsampling) the same way
/// [`crate::engine::baseline::fit_baseline_with`] does, using
/// `get_candidates_fst4.f90`'s specific pre-processing: bin the raw
/// FFT into a `df2 = tone_spacing_hz/2` low-resolution power spectrum
/// `s(i)`, then a 4-tap comb (CCF) `s2(i) = s(i-3h)+s(i-h)+s(i+h)+s(i+3h)`
/// (`h` = [`HMOD`]) before the percentile+polyfit baseline fit itself.
///
/// Returns `None` if the local window doesn't fit inside `fft_cache`
/// (near a band edge) or doesn't have enough points for the fit —
/// callers should fall back to [`crate::engine::llr::compute_snr_db`]
/// in that case, same as any other protocol without a ported formula.
pub(crate) fn fst4_baseline_lin(
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    cand_freq_hz: f32,
    tone_spacing_hz: f32,
) -> Option<f32> {
    let df1 = ds_cfg.input_rate as f32 / ds_cfg.fft1_size as f32;
    let df2 = tone_spacing_hz / 2.0;
    if !(df1 > 0.0 && df2 > 0.0) || cand_freq_hz <= 0.0 {
        return None;
    }
    let nd = ((df2 / df1).round() as i64).max(1);
    let ndh = (nd / 2).max(0);
    let j_max = (fft_cache.len() / 2) as i64;
    if j_max <= 0 {
        return None;
    }

    let iploc = (cand_freq_hz / df2).round() as i64;
    let half_width_bins = (LOCAL_BASELINE_HALF_WIDTH_HZ / df2).round() as i64;

    let fa = (iploc - half_width_bins).max(1 + 3 * HMOD);
    let mut fb = iploc + half_width_bins;
    if fb <= fa {
        return None;
    }

    // j0(i): the raw-FFT bin index the low-resolution bin `i` is
    // centred on (`get_candidates_fst4.f90`'s `j0=nint(i*df2/df1)`).
    let j0_at = |i: i64| -> i64 { ((i as f32) * df2 / df1).round() as i64 };

    // Shrink the window until every raw-bin lookup the CCF's edges
    // need (`fb+3h`'s `j0+ndh`) stays inside `fft_cache`'s valid range.
    while fb > fa && j0_at(fb + 3 * HMOD) + ndh > j_max {
        fb -= 1;
    }
    // NSEG=8 segments need at least a couple of points each to mean
    // anything; bail rather than fit garbage on a tiny window.
    const MIN_FIT_WIDTH: i64 = 32;
    if fb - fa < MIN_FIT_WIDTH {
        return None;
    }

    // Low-resolution power spectrum s(i), i in [fa-3h, fb+3h] (the
    // widened range the CCF below needs at its own edges).
    let lo = fa - 3 * HMOD;
    let hi = fb + 3 * HMOD;
    let n = (hi - lo + 1) as usize;
    let mut s = vec![0.0f32; n];
    for (idx, i) in (lo..=hi).enumerate() {
        let j0 = j0_at(i);
        let mut acc = 0.0f32;
        for j in (j0 - ndh).max(0)..=(j0 + ndh).min(j_max) {
            let c = fft_cache[j as usize];
            acc += c.re * c.re + c.im * c.im;
        }
        s[idx] = acc;
    }
    let s_at = |i: i64| -> f32 { s[(i - lo) as usize] };

    // CCF of s() with the 4-tone comb (`s2(i)=s(i-3h)+s(i-h)+s(i+h)+s(i+3h)`).
    let ccf_n = (fb - fa + 1) as usize;
    let mut s2 = vec![0.0f32; ccf_n];
    for (idx, i) in (fa..=fb).enumerate() {
        s2[idx] = s_at(i - 3 * HMOD) + s_at(i - HMOD) + s_at(i + HMOD) + s_at(i + 3 * HMOD);
    }

    let sbase_db = fit_baseline_with(&s2, 0, ccf_n - 1, BaselineParams::FST4);
    let idx = (iploc - fa) as usize;
    let base_db = *sbase_db.get(idx)?;
    let base_lin = 10f32.powf(base_db / 10.0);
    if base_lin.is_finite() && base_lin > 0.0 {
        Some(base_lin)
    } else {
        None
    }
}

/// `xsig = Σ s4(itone(i),i)` (`fst4_decode.f90:592-595`) — power (not
/// amplitude) at the decoded tone, summed across every symbol
/// (sync + data, `NN=160`). `cs`/`itone` are
/// [`crate::engine::llr::symbol_spectra`]`::<P>` /
/// `encode_tones_for_snr::<P>` output — the same already-computed
/// per-symbol spectra and reconstructed full tone sequence every
/// other `GenericPipelineProtocol::snr_db` override reads from
/// [`crate::engine::pipeline::SnrCtx`].
///
/// `* 1000.0` on each component undoes `symbol_spectra`'s own
/// `/1000` scale (`engine::llr::symbol_spectra`'s doc comment) before
/// squaring — the same "un-scale trick" FT8's own
/// `compute_xsig_wsjtx` uses (`ft8/decode_block/process_candidates.rs`),
/// needed because that scale is this crate's own numeric-range
/// convenience with no WSJT-X counterpart, not present in
/// `get_fst4_bitmetrics.f90`'s `cs(itone,k)=sum(csymb*conjg(ci(:,itone)))`.
fn fst4_xsig(cs: &[Complex<f32>], itone: &[u8], ntones: usize) -> f32 {
    let mut xsig = 0.0f32;
    for (k, &t) in itone.iter().enumerate() {
        let idx = k * ntones + (t as usize % ntones.max(1));
        if let Some(c) = cs.get(idx) {
            let re = c.re * 1000.0;
            let im = c.im * 1000.0;
            xsig += re * re + im * im;
        }
    }
    xsig
}

/// Recomputes `cs` fresh, **without**
/// `engine::pipeline::process_candidate_basic_impl`'s RMS-normalisation
/// step (`cd0 = cd0 / sqrt(sum2)`, matching WSJT-X `ft4_decode.f90:
/// 231-232` — applied uniformly by this crate's generic pipeline for
/// every `GenericPipelineProtocol` implementor, needed for
/// `compute_llr`'s `LLR_SCALE` calibration, issue #18). `SnrCtx::cs`
/// is downstream of that normalisation, so its absolute scale carries
/// **no** SNR information — every candidate's downsampled baseband
/// ends up at ~unit RMS regardless of how weak or strong the real
/// signal was.
///
/// WSJT-X's own FST4 path does *not* apply this normalisation before
/// computing bitmetrics/`xsig` (`fst4_decode.f90`'s `cframe=c2(is0:iend)`
/// is a bare slice, no `sum2`/RMS step — confirmed by reading the
/// source; contrast `ft4_decode.f90:231-232`, which does normalise,
/// same as this crate's shared pipeline). So a WSJT-X-faithful `xsig`
/// needs a `cs` built the same way: downsample fresh (no RMS step),
/// frequency-shift to the **fine-refined** frequency
/// (`fst4_decode.f90`'s bitmetrics input is downsampled at `fc_synced`,
/// not the coarse candidate frequency `get_candidates_fst4.f90`'s
/// baseline is keyed by — a real distinction in the original, not
/// sloppiness here), then run [`crate::engine::llr::symbol_spectra`]
/// at the exact same `i_start` the real (normalised) `cs` used.
///
/// Recomputing from `fft_cache`/`ds_cfg`/`cand_freq_hz`/
/// `refined_freq_hz`/`i_start` (deterministic, same inputs
/// `process_candidate_basic_impl` used) rather than threading a second
/// `cs` array through `SnrCtx` keeps this entirely inside FST4's own
/// protocol-owned module instead of doubling the generic engine's
/// per-candidate allocation for every protocol, not just this one.
///
/// An earlier version of this function instead multiplied the
/// *already-normalised* `xsig` by the downsampled baseband's mean
/// power as an algebraic undo — a single scalar multiply applied
/// uniformly across the whole array, so mathematically exact whenever
/// `refined_freq_hz == cand_freq_hz` (both this module's two real test
/// candidates happened to land close enough to that for the two
/// versions to produce byte-identical output). Rebuilding `cs` from
/// scratch at the correct frequency, as this version does, is more
/// principled for candidates with a larger fine-refine correction
/// (matching WSJT-X's own distinction between `fc_synced` and the
/// coarse candidate frequency, see above) even though it didn't turn
/// out to be what closed this module's real residual gap — that was a
/// separate downsample-scale-convention mismatch, see the module doc
/// comment.
fn fst4_raw_cs<P: crate::engine::Protocol>(
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    cand_freq_hz: f32,
    refined_freq_hz: f32,
    i_start: i32,
) -> Vec<Complex<f32>> {
    let raw_cd0_base = downsample_cached(fft_cache, cand_freq_hz, ds_cfg);
    let ds_rate = ds_cfg.input_rate as f32 * ds_cfg.fft2_size as f32 / ds_cfg.fft1_size as f32;
    let df_hz = refined_freq_hz - cand_freq_hz;
    let raw_cd0 = crate::engine::sync2d::freq_shift_cd0(&raw_cd0_base, df_hz, ds_rate);
    crate::engine::llr::symbol_spectra::<P>(&raw_cd0, i_start)
}

/// FST4's real SNR formula (`fst4_decode.f90:592-621`):
///
/// ```text
///   xsig = Σ s4(itone(i),i)                     ! over NN=160 symbols
///   base = candidates(icand,5)                  ! noise baseline at candidate bin
///   arg  = snr_calfac·xsig/base - 1.0
///   xsnr = 10·log10(arg) + 10·log10(1.46/2500) + 10·log10(8200/nsps)   if arg > 0
///        = -99.9                                                       otherwise
/// ```
///
/// `snr_calfac` is per sub-mode (`fst4_decode.f90`'s `select case
/// (ntrperiod)`: 800/600/430/390/340 for 15/30/60/120/300 — see the
/// `fst4_submode!` invocations in `fst4/mod.rs`). `NSPS` is read from
/// `P` directly ([`crate::engine::ModulationParams::NSPS`]).
///
/// Falls back to the WSJT-X `-99.9` sentinel (an explicit "not a real
/// SNR" marker in the original, not this crate's own invention) both
/// when `arg <= 0.0` and when [`fst4_baseline_lin`] can't fit a
/// baseline at all (candidate too close to a band edge for this
/// module's local-window simplification — see its own doc comment).
///
/// `itone` (from `encode_tones_for_snr::<P>`, same as every other
/// `GenericPipelineProtocol::snr_db` override reads via `SnrCtx`) is
/// the only piece of `SnrCtx` this function still needs — `cs` itself
/// is *not* used; [`fst4_raw_cs`] rebuilds the version this formula
/// actually needs instead (see its own doc comment for why).
///
/// See this module's own doc comment for the real `jt9` ground-truth
/// comparison this was verified against, including the derivation of
/// this function's own `NDOWN`-in-power downsample-scale correction
/// on `xsig` below.
#[allow(clippy::too_many_arguments)]
pub(crate) fn fst4_snr_db<P: crate::engine::Protocol>(
    itone: &[u8],
    cand_freq_hz: f32,
    refined_freq_hz: f32,
    i_start: i32,
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    snr_calfac: f32,
) -> f32 {
    const WSJTX_INVALID_SENTINEL: f32 = -99.9;
    let Some(base) = fst4_baseline_lin(fft_cache, ds_cfg, cand_freq_hz, P::TONE_SPACING_HZ) else {
        return WSJTX_INVALID_SENTINEL;
    };
    let raw_cs = fst4_raw_cs::<P>(fft_cache, ds_cfg, cand_freq_hz, refined_freq_hz, i_start);
    // `downsample_cached`'s own scale convention
    // (`fac = 1/sqrt(fft1_size·fft2_size)`, `engine::dsp::downsample`'s
    // module doc) differs from WSJT-X's `fst4_downsample`
    // (`c1 = c1/nfft2`, a plain `1/fft2_size` divide with *no* sqrt).
    // Both apply an *unnormalised* inverse FFT after that pre-scale, and
    // an unnormalised IDFT of a single nonzero bin reproduces that
    // bin's magnitude at *every* output sample regardless of transform
    // length — so, given both downsamples extract the same underlying
    // big-FFT bins (same physical audio; `fft1_size` differs slightly
    // between the two implementations' padding choices, close enough
    // not to matter here), the amplitude ratio between this crate's
    // downsampled baseband and WSJT-X's own is `(1/sqrt(fft1·fft2)) /
    // (1/fft2) = fft2/sqrt(fft1·fft2) = sqrt(fft2/fft1) = 1/sqrt(NDOWN)`
    // (since `fft2 = fft1/NDOWN`). In power (`xsig` is a squared-
    // magnitude sum) that's a factor of `NDOWN`. Confirmed against the
    // real `jt9` SNRAUDIT_FST4_BM probe (issue #255): per-symbol,
    // per-tone `s4` power ratios (jt9/ours) landed at 86-125 across
    // both real signals and multiple symbols, clustering right around
    // `NDOWN=108` for FST4-60 — an analytically-derived, protocol-
    // parameter constant, not a fitted one.
    let xsig = fst4_xsig(&raw_cs, itone, P::NTONES as usize) * P::NDOWN as f32;
    let arg = snr_calfac * xsig / base - 1.0;
    if arg > 0.0 {
        10.0 * arg.log10()
            + 10.0 * (1.46f32 / 2500.0).log10()
            + 10.0 * (8200.0 / P::NSPS as f32).log10()
    } else {
        WSJTX_INVALID_SENTINEL
    }
}
