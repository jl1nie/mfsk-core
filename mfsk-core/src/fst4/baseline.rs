//! FST4's real SNR formula — port of `get_candidates_fst4.f90`'s
//! baseline extraction (not its CLEAN candidate-detection loop) and
//! `fst4_decode.f90:585-621`'s `xsnr` formula (issue #255).
//!
//! `#![allow(dead_code)]`: every item here is currently unreachable —
//! see below for why this isn't wired into `fst4/decode.rs` yet.
//!
//! **Not wired in yet** (`fst4/decode.rs`'s `impl_frame_decodable!`
//! deliberately does *not* override `GenericPipelineProtocol::snr_db`
//! with [`fst4_snr_db`] — FST4 still reports SNR via the trait's
//! generic `compute_snr_db` default). Verified against a real local
//! `jt9 -7 -d3` build's own probed values
//! (`WSJT-X/samples/FST4+FST4W/210115_0058.wav`, `SNRAUDIT_FST4_PROBE`
//! instrumentation added to `fst4_decode.f90` for this investigation,
//! not committed — issue #255's stated verification discipline):
//!
//! | candidate | jt9 `xsig` | jt9 `base` | jt9 `arg` | jt9 `xsnr` | this module's `xsnr` |
//! |-----------|------------|------------|-----------|------------|----------------------|
//! | N5TM      | 1.621e13   | 4.180e13   | 165.7     | -6.90 dB   | -42.53 dB            |
//! | K9KFR     | 3.836e15   | 4.942e13   | 3.337e4   | 16.14 dB   | -3.53 dB             |
//!
//! [`fst4_baseline_lin`]'s `base` matched jt9's within ~30-50% on the
//! first attempt — that part of the port holds up. [`fst4_xsig`]
//! didn't: initially near-*constant* across both real signals despite
//! their ~23 dB real SNR difference (root-caused to this crate's
//! shared pipeline RMS-normalising the downsampled baseband before
//! [`crate::engine::llr::symbol_spectra`] ever runs, destroying the
//! absolute-power information `xsig` needs — see [`cd0_rms_pow`]'s
//! doc comment). Correcting for that produced the *directionally*
//! right answer (K9KFR now reports higher than N5TM) but the
//! **residual gap is not a constant offset** — 35.6 dB for N5TM,
//! 19.7 dB for K9KFR — so there is no honest single fudge factor left
//! to calibrate away; something else in the `xsig` derivation (most
//! likely a remaining scale or windowing difference between this
//! crate's `downsample_cached`/`symbol_spectra` and WSJT-X's own
//! `fst4_downsample`/`get_fst4_bitmetrics` — e.g. WSJT-X's raised-
//! cosine-free rectangular window vs. this crate's `edge_taper_bins`
//! taper, or per-symbol correlation-gain differences) is still
//! unaccounted for. Left in place, `#[allow(dead_code)]`, as verified
//! groundwork for whoever picks this back up — re-deriving the
//! `base`-side match and the RMS-normalisation root cause from scratch
//! would waste real effort. Do **not** wire this in as-is: for real
//! signals it currently reports *worse* numbers than the existing
//! generic-heuristic default it would replace.
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

#![allow(dead_code)]

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

/// Mean power of the downsampled baseband **before**
/// `engine::pipeline::process_candidate_basic_impl`'s RMS-normalisation
/// (`cd0 = cd0 / sqrt(sum2)`, matching WSJT-X `ft4_decode.f90:231-232` —
/// applied uniformly by this crate's generic pipeline for every
/// `GenericPipelineProtocol` implementor, needed for `compute_llr`'s
/// `LLR_SCALE` calibration, issue #18). `SnrCtx::cs` is downstream of
/// that normalisation, so its absolute scale carries **no** SNR
/// information — every candidate's downsampled baseband ends up at
/// ~unit RMS regardless of how weak or strong the real signal was.
///
/// WSJT-X's own FST4 path does *not* apply this normalisation before
/// computing bitmetrics/`xsig` (`fst4_decode.f90`'s `cframe=c2(is0:iend)`
/// is a bare slice, no `sum2`/RMS step — confirmed by reading the
/// source; contrast `ft4_decode.f90:231-232`, which does normalise,
/// same as this crate's shared pipeline). So `fst4_xsig`'s output needs
/// re-scaling by this function's returned `sum2` (`xsig_absolute =
/// xsig_normalised · sum2`) to land back on the same absolute-power
/// scale WSJT-X's real `xsig`/`snr_calfac` were calibrated against —
/// discovered via the real `jt9 -7` SNRAUDIT probe this function's
/// caller doc references (issue #255): without this correction, `xsig`
/// came out ~constant across wildly different real-SNR candidates
/// (both `WSJT-X/samples/FST4+FST4W/210115_0058.wav` decodes landed
/// within 4% of each other despite a real ~23 dB SNR difference),
/// while `base` (computed straight from the un-normalised `fft_cache`)
/// was already on the right scale — exactly the asymmetry this
/// corrects.
///
/// Recomputes the downsample fresh from `fft_cache`/`ds_cfg`/
/// `cand_freq_hz` (deterministic, same inputs `process_candidate_basic_impl`
/// used) rather than threading the value through `SnrCtx` — keeps this
/// fix entirely inside FST4's own protocol-owned module instead of
/// touching the generic engine's hot per-candidate path.
fn cd0_rms_pow(fft_cache: &[Complex<f32>], ds_cfg: &DownsampleCfg, cand_freq_hz: f32) -> f32 {
    let cd0 = downsample_cached(fft_cache, cand_freq_hz, ds_cfg);
    if cd0.is_empty() {
        return 1.0;
    }
    cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32
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
/// `fst4_submode!` invocations in `fst4/mod.rs`). `nsps` is the
/// sub-mode's un-downsampled samples-per-symbol
/// ([`crate::engine::ModulationParams::NSPS`]).
///
/// Falls back to the WSJT-X `-99.9` sentinel (an explicit "not a real
/// SNR" marker in the original, not this crate's own invention) both
/// when `arg <= 0.0` and when [`fst4_baseline_lin`] can't fit a
/// baseline at all (candidate too close to a band edge for this
/// module's local-window simplification — see its own doc comment).
///
/// **Not wired in** — see this module's own doc comment for the real
/// `jt9` ground-truth comparison and the residual gap that's kept it
/// out of `fst4/decode.rs`'s `GenericPipelineProtocol::snr_db`
/// override so far.
#[allow(clippy::too_many_arguments)]
pub(crate) fn fst4_snr_db(
    cs: &[Complex<f32>],
    itone: &[u8],
    cand_freq_hz: f32,
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    ntones: usize,
    tone_spacing_hz: f32,
    nsps: u32,
    snr_calfac: f32,
) -> f32 {
    const WSJTX_INVALID_SENTINEL: f32 = -99.9;
    let Some(base) = fst4_baseline_lin(fft_cache, ds_cfg, cand_freq_hz, tone_spacing_hz) else {
        return WSJTX_INVALID_SENTINEL;
    };
    let rms_pow = cd0_rms_pow(fft_cache, ds_cfg, cand_freq_hz);
    let xsig = fst4_xsig(cs, itone, ntones) * rms_pow;
    let arg = snr_calfac * xsig / base - 1.0;
    if arg > 0.0 {
        10.0 * arg.log10()
            + 10.0 * (1.46f32 / 2500.0).log10()
            + 10.0 * (8200.0 / nsps as f32).log10()
    } else {
        WSJTX_INVALID_SENTINEL
    }
}
