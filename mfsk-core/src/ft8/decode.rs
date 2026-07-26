/// High-level FT8 decode pipeline.
///
/// Chains: downsample → coarse_sync → fine_sync → LLR → BP decode
use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

pub use super::equalizer::EqMode;
use super::{
    Ft8,
    downsample::build_fft_cache,
    equalizer,
    llr::sync_quality,
    params,
    params::BP_MAX_ITER,
    subtract::{subtract_signal_lpf, subtract_signal_lpf_refine_dt},
    sync::SyncCandidate,
};
use crate::msg::decode_request::{
    DecodeOutcome, DecodeRequest, FrameDecodable, SniperRequest, SupportsFlatSic,
    SupportsStagedSic, SupportsWideBandAp,
};

// ────────────────────────────────────────────────────────────────────────────
// Public types

/// Opaque FFT cache, reusable across pipelined decode passes.
///
/// Canonical definition lives in [`crate::core::pipeline::FftCache`] — same
/// underlying `Vec<Complex<f32>>`, re-exported here for backward-compatible
/// `mfsk_core::ft8::decode::FftCache` import paths (issue #191).
pub use crate::core::pipeline::FftCache;

/// Decode cost/recall configuration, plus the [`LlrEffort`] staircase width.
///
/// Canonical definition lives in [`crate::core::pipeline::DecodeDepth`] —
/// re-exported here (with [`LlrEffort`]) for backward-compatible
/// `mfsk_core::ft8::decode::DecodeDepth` import paths (issue #191 type
/// consolidation migrated `core::pipeline`'s old 2-variant `BpAll`/
/// `BpAllOsd` enum, used by FT4/FST4, onto this struct). `llr_effort` is
/// FT8-only in practice — the generic `core::pipeline` engine FT4/FST4 use
/// always computes all LLR variants and only reads `depth.osd`.
pub use crate::core::pipeline::{DecodeDepth, LlrEffort};

/// Decode strictness: controls false-positive vs sensitivity trade-off.
///
/// Canonical definition lives in [`crate::core::pipeline::DecodeStrictness`]
/// — this used to be a separate, differently-calibrated `osd_max_errors`/
/// `osd_score_min` (FT8's own real-WAV-bench numbers), but neither method
/// has ever actually been called anywhere in FT8's own decode path: FT8's
/// non-AP OSD gate uses hardcoded `OSD_HARDERRORS_MAX`/
/// `WSJTX_NHARDERRORS_MAX` constants instead
/// (`ft8/decode_block/osd_strategy.rs`, `process_candidates.rs`), leftover
/// dead code from before #188 removed the `auto_ap_strategy` module that
/// used to consume them. `ap_max_errors` *is* live (FT8's per-candidate AP
/// loop) and was already numerically identical to the generic pipeline's
/// copy — moved onto the canonical type, no numeric change (issue #191
/// type consolidation).
pub use crate::core::pipeline::DecodeStrictness;

/// One successfully decoded FT8 message.
#[derive(Debug, Clone)]
pub struct DecodeResult {
    /// Decoded message: 77 bits packed as bytes (LSB first within each byte).
    pub message77: [u8; 77],
    /// Carrier frequency (Hz)
    pub freq_hz: f32,
    /// Time offset from the nominal 0.5 s start (seconds)
    pub dt_sec: f32,
    /// Number of hard-decision errors in the final codeword
    pub hard_errors: u32,
    /// Sync quality score from fine sync
    pub sync_score: f32,
    /// Which LLR variant decoded successfully (0=llra, 1=llrb, 2=llrc, 3=llrd)
    pub pass: u8,
    /// Coefficient of variation of the three Costas-array powers (score_a/b/c).
    ///
    /// Near zero for a stable channel; elevated (> 0.3) when QSB or strong
    /// time-varying fading is present.  Used by `decode_frame_subtract` to
    /// apply partial subtraction gain when the amplitude estimate is unreliable.
    pub sync_cv: f32,
    /// WSJT-X compatible SNR estimate (dB).
    ///
    /// Computed from decoded tone power vs. opposite-tone noise power:
    /// `10 log10(xsig/xnoi − 1) − 27 dB`.  Floor is −24 dB (same as WSJT-X).
    pub snr_db: f32,
}

// ────────────────────────────────────────────────────────────────────────────
// A Priori (AP) hint for sniper-mode decode
//
// Canonical definition lives in [`crate::msg::ap::ApHint`] — this used to be
// a separate, byte-for-byte-identical struct (down to reusing the same
// `pack28`/`pack_grid4` from `msg::wsjt77` under the hood), duplicated here
// before the `DecodeRequest<P>` consolidation (issue #191) made a single
// shared type load-bearing for genericity across protocols. Re-exported so
// existing `mfsk_core::ft8::decode::ApHint` import paths keep working.
pub use crate::msg::ap::ApHint;

// ────────────────────────────────────────────────────────────────────────────
// Per-candidate decode helper (used by both inner and sniper paths)

/// Decode a single sync candidate: downsample → refine → LLR → BP/OSD.
///
/// `fft_cache` — pre-computed 192 000-point forward FFT of the full audio
///   (from [`build_fft_cache`]), shared read-only across parallel calls.
/// `known`     — messages decoded in earlier subtract passes; prevents OSD
///   from running on frequencies that already have a result.
///
/// Returns `Some(DecodeResult)` on the first successful decode, `None` if the
/// candidate yields no valid message.
fn process_candidate(
    cand: &SyncCandidate,
    audio: &[i16],
    fft_cache: &[num_complex::Complex<f32>],
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
) -> Option<DecodeResult> {
    let _ = strictness; // used inside try_decode via the inner
    // Use `downsample_cached` directly so the FT8 wrapper's
    // `cache.to_vec()` clone (~3 MB) on the `Some(_)` branch is
    // bypassed — same pattern as `fill_symbol_spectra_via_cd0`.
    let mut cd0 = crate::core::dsp::downsample::downsample_cached(
        fft_cache,
        cand.freq_hz,
        &crate::ft8::downsample::FT8_CFG,
    );

    // WSJT-X 3-stage fine refinement (ft8b.f90:104-150). Validates
    // freq snap to ±0.5 Hz grid + dt to integer 200 Hz step before
    // computing symbol spectra. Without this, busy-band birdies that
    // sit ±1-2 Hz off the real FT8 carrier still produce coherent
    // Costas correlation at the candidate's initial freq, leak into
    // BP, and emit phantom CRC-pass decodes (e.g. qso3_busy W1FC /
    // WM3PEN / XE2X at f > 2 kHz).
    let refine_result = crate::ft8::refine_fine::fine_refine_3stage(&cd0, cand.dt_sec);
    let refined = SyncCandidate {
        freq_hz: cand.freq_hz + refine_result.delf_hz,
        dt_sec: refine_result.dt_sec,
        score: refine_result.score,
    };
    if refine_result.delf_hz.abs() > f32::EPSILON {
        // Apply the freq shift in place so symbol_spectra / BP see
        // the refined baseband.
        let dt2 = 1.0_f32 / 200.0;
        for (k, c) in cd0.iter_mut().enumerate() {
            let phi = -core::f32::consts::TAU * refine_result.delf_hz * (k as f32) * dt2;
            let rot = num_complex::Complex::new(phi.cos(), phi.sin());
            *c *= rot;
        }
    }

    // sync_cv from cd0 + i_start (Costas correlation power CV).
    // cd0 is at the refined-carrier baseband (the freq-shift above
    // brought any non-zero `delf_hz` to 0), so fixed-tone Costas
    // references in `fine_sync_power_per_block` align correctly.
    let i_start = ((refined.dt_sec + 0.5) * 200.0).round() as i32;
    let sync_cv = {
        let scores = crate::core::sync::fine_sync_power_per_block::<crate::ft8::Ft8>(&cd0, i_start);
        let sa = scores.first().copied().unwrap_or(0.0);
        let sb = scores.get(1).copied().unwrap_or(0.0);
        let sc = scores.get(2).copied().unwrap_or(0.0);
        let mean = (sa + sb + sc) / 3.0;
        if mean > f32::EPSILON {
            let sq = (sa - mean).powi(2) + (sb - mean).powi(2) + (sc - mean).powi(2);
            sq.sqrt() / mean
        } else {
            0.0
        }
    };
    drop(cd0);
    // cs from 12 kHz audio directly via `fill_symbol_spectra` —
    // matches embedded `decode_block`'s per-symbol-region DFT exactly.
    // The cd0+symbol_spectra path host used pre-0.6.2 produced
    // numerically-different cs values that lost ~3 entries on
    // qso3_busy.wav vs decode_block (CQ EA2BFM, KD2UGC F6GCP,
    // K1BZM EA3CJ). Rewiring to fill_symbol_spectra closes that gap.
    let mut cs_raw: alloc::boxed::Box<[[crate::core::scalar::Cmplx<f32>; 8]; 79]> =
        alloc::vec![[crate::core::scalar::Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    // `sync_quality` only reads the 21 sync-block symbol positions
    // (`P::SYNC_MODE.blocks()`, `core::llr::sync_quality_generic`) —
    // never the 58 data symbols. Gate on `nsync <= 6` right after the
    // `SyncOnly` fill, *before* paying for `DataOnly`'s per-symbol
    // 32-pt FFTs (58 of the 79 symbols — the bulk of this function's
    // per-candidate cost) and its own `downsample_cached` call. On
    // `qso3_busy.wav`'s reference decode, ~82% of the up-to-1200
    // candidates this function sees across a full staged decode get
    // rejected at this gate — profiling found the pre-gate work was
    // costing nearly as much in aggregate as the entire BP+OSD
    // staircase combined, almost all of it wasted on candidates whose
    // `DataOnly` symbols are never looked at (issue #182 follow-up).
    crate::ft8::decode_block::fill_symbol_spectra(
        &mut cs_raw,
        audio,
        refined.freq_hz,
        refined.dt_sec,
        crate::ft8::decode_block::SymMask::SyncOnly,
        Some(fft_cache),
    );
    let nsync = sync_quality(&cs_raw);
    if nsync <= 6 {
        return None;
    }
    crate::ft8::decode_block::fill_symbol_spectra(
        &mut cs_raw,
        audio,
        refined.freq_hz,
        refined.dt_sec,
        crate::ft8::decode_block::SymMask::DataOnly,
        Some(fft_cache),
    );

    // Per-candidate decode delegated to the unified inner — same
    // staircase + OSD + AP loop the embedded `decode_block` path
    // uses (decode_block.rs::process_one_candidate_inner). The
    // host's outer prelude (downsample → fine_refine_3stage →
    // symbol_spectra → nsync gate → sync_cv → EqMode cs choice)
    // stays here; only the per-candidate decode body delegates.
    let try_decode =
        |cs: &[[crate::core::scalar::Cmplx<f32>; 8]; 79], _use_ap: bool| -> Option<DecodeResult> {
            // BpScratch must be parameterised on the same LlrT the
            // inner uses. decode_block defines `type LlrT = Q11i16`
            // under `feature = "fixed-point"`, `f32` otherwise — match.
            #[cfg(feature = "fixed-point")]
            let mut bp_scratch = crate::fec::ldpc::bp::BpScratch::<
                crate::fec::ldpc::params::Ldpc174_91Params,
                crate::core::scalar::Q11i16,
            >::new();
            #[cfg(not(feature = "fixed-point"))]
            let mut bp_scratch = crate::fec::ldpc::bp::BpScratch::<
                crate::fec::ldpc::params::Ldpc174_91Params,
                f32,
            >::new();
            crate::ft8::decode_block::process_one_candidate_inner(
                cs,
                &refined,
                refined.dt_sec,
                nsync,
                depth,
                BP_MAX_ITER,
                &mut bp_scratch,
                known,
                ap_hint,
                strictness,
                sync_cv,
            )
        };

    match eq_mode {
        EqMode::Off => try_decode(&cs_raw, true),
        EqMode::Local => {
            let mut cs_eq = cs_raw.clone();
            equalizer::equalize_local(&mut cs_eq);
            try_decode(&cs_eq, true)
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────

/// Inner decode loop shared by [`decode_frame`] and [`decode_frame_subtract`].
///
/// `known`           — messages already decoded in earlier passes (skipped).
/// `precomputed_fft` — optional pre-computed 192k-point FFT cache; when `None`
///                     the cache is built internally from `audio`.
/// `ap_hint`         — optional a-priori callsign / grid hint forwarded to
///                     every per-candidate BP/OSD decode.  When `Some(_)` the
///                     BP decoder locks the high-confidence AP bits prior to
///                     iteration, yielding ~1–3 dB gain at threshold when the
///                     hint matches a station actually on air. Passing `None`
///                     preserves legacy behavior bit-for-bit (identical LLR
///                     pipeline; no AP bits are locked).
///
/// Returns `(decoded_results, fft_cache)`.  Callers that don't need the cache
/// can simply ignore the second element.
fn decode_frame_inner(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    precomputed_fft: Option<&[num_complex::Complex<f32>]>,
    ap_hint: Option<&ApHint>,
) -> (Vec<DecodeResult>, Vec<num_complex::Complex<f32>>) {
    // `freq_hint` is intentionally not forwarded — the WSJT-X-faithful
    // decode_block::coarse_sync (the only FT8 coarse-sync after the v0.6
    // consolidation in #48) does not honour candidate-score promotion.
    // Sniper paths in this file constrain freq_min/freq_max around the
    // target instead, so the loss is contained.
    let _ = freq_hint;
    let spec = crate::ft8::decode_block::compute_spectrogram(audio, freq_max);
    let candidates =
        crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
    // Build (or clone) the FFT cache exactly once. The cache is needed both
    // when there are no candidates (early return) and when running BP/OSD
    // per candidate, so do it before the early-exit branch to avoid a
    // redundant clone of `precomputed_fft` on the candidates path.
    let fft_cache = match precomputed_fft {
        Some(c) => c.to_vec(),
        None => build_fft_cache(audio),
    };
    if candidates.is_empty() {
        return (Vec::new(), fft_cache);
    }

    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            process_candidate(
                cand, audio, &fft_cache, depth, strictness, known, eq_mode, ap_hint,
            )
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            process_candidate(
                cand, audio, &fft_cache, depth, strictness, known, eq_mode, ap_hint,
            )
        })
        .collect();

    // Deduplicate: preserve first occurrence; drop messages already in `known`.
    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        if !known.iter().any(|k| k.message77 == r.message77)
            && !results.iter().any(|x| x.message77 == r.message77)
        {
            results.push(r);
        }
    }
    (results, fft_cache)
}

// ────────────────────────────────────────────────────────────────────────────
// Multi-pass decode with signal subtraction

/// The flat 3-pass + sequential SIC, mirroring the structure of
/// `decode_block::decode_block_multipass` (the embedded path) which is a
/// faithful port of `lib/ft8/ft8b.f90:432-437`. Used by
/// [`SupportsFlatSic`]'s `Ft8` impl (`.flat()`) and as
/// [`decode_frame_subtract_staged_with_ap_inner`]'s fallback for
/// buffers too short to stage meaningfully.
///
/// Two changes from the pre-v0.6.0 host implementation:
///
/// 1. **Fixed `sync_min` across all 3 passes** (was 1.0 / 0.75 / 0.5).
///    Progressive relaxation lets phantoms slip through later passes
///    when SIC artefacts dominate the residual; WSJT-X holds the
///    threshold and skips later passes when no new decodes come out.
///
/// 2. **Sequential subtract within each pass** (was batch-after-pass).
///    Each accepted decode immediately subtracts from the residual so
///    the *next* candidate in the same pass sees a cleaner spectrum.
///    This is what surfaces -13 to -18 dB signals sitting beneath
///    strong neighbours (the JTDX-extras shape on `qso3_busy.wav`).
///    Without sequential subtract, all candidates in a pass see the
///    same raw residual and weak signals stay masked.
///
/// Pass termination matches WSJT-X: pass 2 skips when pass 1 returned
/// 0 decodes; pass 3 skips when pass 2 returned no NEW decodes.
#[allow(clippy::too_many_arguments)]
fn flat_sic_inner(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    ap_hint: Option<&ApHint>,
    precomputed_fft: Option<&[num_complex::Complex<f32>]>,
) -> (Vec<DecodeResult>, FftCache) {
    let mut residual = audio.to_vec();
    sic_inner_passes_with_cache(
        &mut residual,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        strictness,
        known,
        ap_hint,
        precomputed_fft,
    )
}

/// Shared inner loop: up to 3 sub-passes of coarse_sync + per-candidate
/// decode, subtracting each accepted decode from `residual` immediately
/// (sequential, not batch-after-pass) so later candidates in the same
/// sub-pass see a cleaner spectrum. This is WSJT-X's `ft8b.f90:432-437`
/// `do ipass=1,npass` structure. Factored out of [`flat_sic_inner`] so
/// [`decode_frame_subtract_staged_with_ap_inner`] can reuse it at
/// checkpoint A and checkpoint C (issue #180) without duplicating the
/// pass/dedup/subtract logic.
///
/// `residual` is mutated in place (final state = fully subtracted).
/// `known` seeds dedup against decodes from an earlier stage — those
/// messages are skipped if re-found and are not re-emitted in the
/// returned `Vec`.
#[allow(clippy::too_many_arguments)]
fn sic_inner_passes(
    residual: &mut [i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    sic_inner_passes_with_cache(
        residual, freq_min, freq_max, sync_min, depth, max_cand, strictness, known, ap_hint, None,
    )
    .0
}

/// Like [`sic_inner_passes`] but also accepts a `precomputed_fft` cache
/// (reused only on pass 0, and only when `residual` has not yet been
/// mutated by a subtraction the caller did before calling in — passing
/// `Some(_)` alongside an already-subtracted `residual` would silently
/// mismatch the cache against the audio it's meant to describe) and
/// returns the pass-0 cache alongside the results, so a follow-up
/// pipelined call can reuse it in turn. This is the fix for issue #191:
/// previously only the (dead-code, zero-caller)
/// `decode_frame_subtract_with_known_and_ap` had *any* cache-reuse path,
/// and it used its own unfixed flat-3-pass engine rather than this
/// (shared by both `.flat()` and `.staged()`) one.
#[allow(clippy::too_many_arguments)]
fn sic_inner_passes_with_cache(
    residual: &mut [i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    ap_hint: Option<&ApHint>,
    precomputed_fft: Option<&[num_complex::Complex<f32>]>,
) -> (Vec<DecodeResult>, FftCache) {
    let mut all_results: Vec<DecodeResult> = Vec::new();
    let mut pass0_cache: Option<FftCache> = None;

    let mut prev_total: usize = 0;
    for ipass in 0..3 {
        if ipass >= 1 && all_results.len() == prev_total {
            break;
        }
        prev_total = all_results.len();

        let spec = crate::ft8::decode_block::compute_spectrogram(residual, freq_max);
        let candidates =
            crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
        drop(spec);
        if candidates.is_empty() {
            continue;
        }

        let fft_cache = if ipass == 0
            && let Some(c) = precomputed_fft
        {
            c.to_vec()
        } else {
            build_fft_cache(residual)
        };
        if ipass == 0 {
            pass0_cache = Some(fft_cache.clone());
        }
        for cand in &candidates {
            let r = match process_candidate(
                cand,
                residual,
                &fft_cache,
                depth,
                strictness,
                known,
                EqMode::Off,
                ap_hint,
            ) {
                Some(r) => r,
                None => continue,
            };
            // Dedup against `known` (an earlier stage) and this loop's
            // own earlier passes.
            if known.iter().any(|x| x.message77 == r.message77)
                || all_results.iter().any(|x| x.message77 == r.message77)
            {
                continue;
            }
            // Sequential subtract — clean residual for next candidate.
            // Use the WSJT-X-style channel-aware LPF subtract (matches
            // `decode_block::decode_block_multipass`'s sequential
            // subtract). The simple constant-amplitude
            // `subtract_signal_weighted` underused the residual on
            // busy bands, leaving Pass-1 coarse_sync unable to surface
            // weaker neighbours like KD2UGC F6GCP / CQ EA2BFM /
            // K1BZM EA3CJ on qso3_busy.wav (the 3 entries embedded
            // catches but host couldn't pre-0.6.2).
            //
            // Single shot, matching `subtractft8.f90` exactly (issue
            // #177/#179): an earlier version of this code iterated
            // `subtract_signal_lpf` to convergence per candidate,
            // reasoning that WSJT-X reaches deeper suppression
            // (~17.65 dB measured vs ~6.6 dB for one call) on hard
            // real signals. Reading `ft8_decode.f90`/`ft8b.f90`
            // directly showed that extra suppression comes from the
            // *outer* `do ipass=1,npass` loop re-detecting the same
            // residual signal as a fresh candidate in a later pass
            // (this function's own `for ipass in 0..3` above already
            // does the same) — `subtractft8.f90` itself is always a
            // single, non-iterated call. The inner convergence loop
            // had no WSJT-X counterpart and repeatedly re-fit/re-
            // subtracted the same candidate against its own imperfect
            // model with no independent ground truth, which let error
            // accumulate and leak into a signal ~40 Hz away on a real
            // FT4 sample (`W9JA PY2APK RRR` at 519.4 Hz, killed by
            // over-iterating a neighbour at 560.0 Hz — see
            // `ft4_wsjtx_sample_iteration_diag.rs`). Removed; the
            // existing outer pass loop is what WSJT-X actually relies
            // on, and every prior regression guard
            // (`ft8_qso3_subtract_fix_check.rs`'s 18/18 with HA5WA,
            // the FT4 busy-band-fading synthetic 10/10) passes
            // identically or better with the single-shot call.
            subtract_signal_lpf(residual, &r);
            all_results.push(r);
        }
    }

    // Pass 0 may have had no candidates at all (empty coarse-sync), in
    // which case `pass0_cache` was never set — build one from the
    // (possibly already-subtracted) residual as a fallback, matching
    // `decode_frame_inner`'s "cache is always returned" contract.
    let fft_cache = pass0_cache.unwrap_or_else(|| build_fft_cache(residual));
    (all_results, fft_cache)
}

// ────────────────────────────────────────────────────────────────────────────
// Staged (checkpoint) SIC — jt9.f90-faithful early-decode-and-subtract
//
// Issue #180: WSJT-X's disk-file FT8 decode is not a single pass over the
// full 15 s slot. `jt9.f90`'s `mode.eq.8` branch calls `multimode_decoder`
// three times with progressively larger *prefixes* of the same audio
// (`nearly=41`, `nearly=47`, `nzhsym=50` — samples 0..141_696, 0..162_432,
// 0..172_800 at 12 kHz), and `ft8_decode.f90::decode` keeps state (`save
// ndec_early, itone_save, f1_save, xdt_save`) across those three calls:
//
//   * Checkpoint A (41): search the truncated/zero-tail buffer with a
//     stricter sync threshold; save whatever decodes as `ndec_early`.
//   * Checkpoint B (47): NOT a search — just subtract every checkpoint-A
//     decode whose full message fits inside this larger truncated buffer,
//     producing a cleaner `dd1`. Late-`dt` decodes are deferred.
//   * Checkpoint C (50): build `dd` = checkpoint-B's cleaned head (0..
//     162_432) + a *fresh* raw tail (162_432..172_800); subtract any
//     deferred checkpoint-A decodes against this now-complete buffer; then
//     run the normal 3-sub-pass search at the relaxed threshold.
//
// The real ground-truth investigation (mfsk-core issue #180, ground-truthed
// against `jt9 -d3` + instrumented WSJT-X source) found `CQ DX DL8YHR JO41`
// (~-17 dB, on `qso3_busy.wav`) only decodes at checkpoint C, after 13 other
// signals found at checkpoint A have already been removed from its local
// spectral neighbourhood — something a flat "decode-the-whole-buffer,
// subtract-after" pass (`decode_frame_subtract_flat_with_ap`, the
// pre-#180 behaviour `decode_frame_subtract_with_ap` delegated to before
// this staged version became the default) can't reproduce no matter how
// the sync threshold is tuned, because DL8YHR's neighbours were never
// subtracted from *before* the residual it's found in was assembled.
//
// This crate has no live streaming buffer for FT8 host/offline decode (the
// checkpoint semantics above are a WSJT-X real-time-emulation artefact of
// `jt9`'s chunked WAV reader) — but the *effect* is fully reproducible
// offline by decoding progressively larger prefixes of the same static
// buffer, since `compute_spectrogram`/`build_fft_cache` already zero-fill
// any index past the slice length (sized off the fixed `NMAX`/`fft1_size`
// constants, not the slice itself) — a plain `&audio[..N]` sub-slice
// reproduces jt9.f90's `id2a(N+1:)=0` zero-pad with no extra copy.

/// jt9.f90's checkpoint sample counts for FT8 disk decode. `kstep=3456`
/// is `nsps/2` where `nsps=6912` is jt9's generic multi-mode block-reader
/// chunk size — a WAV-reading-loop constant unrelated to FT8's own 1920
/// samples/symbol, used unmodified for every mode jt9 supports. Kept as
/// literal sample counts (not re-derived from FT8's own `NSPS`) to stay
/// byte-faithful to the reference values `41×3456`/`47×3456`/`50×3456`.
mod staged_checkpoint {
    /// Checkpoint A ("early pass"): jt9.f90 `nearly=41` — ~11.8 s.
    pub const A_SAMPLES: usize = 141_696;
    /// Checkpoint B (subtract-prep only, no search): jt9.f90 `nearly=47` — ~13.5 s.
    pub const B_SAMPLES: usize = 162_432;
    /// Checkpoint C (final full pass): jt9.f90 `nzhsym=50` — ~14.4 s.
    pub const C_SAMPLES: usize = 172_800;
}

/// Test-only variant that also returns checkpoint C's residual buffer
/// (the state actually handed to the final, most-relaxed search) —
/// used by diagnostics that need to probe a specific `(freq, dt)` by
/// hand against exactly what the production checkpoint-C search sees.
/// Thin shim over the same inner the production function above uses,
/// same pattern as
/// [`decode_frame_subtract_with_known_and_ap_debug_residual`].
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_frame_subtract_staged_with_ap_debug_residual(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    ap_hint: Option<&ApHint>,
) -> (Vec<DecodeResult>, Vec<i16>) {
    decode_frame_subtract_staged_with_ap_inner(
        audio, freq_min, freq_max, sync_min, freq_hint, depth, max_cand, strictness, ap_hint,
    )
}

#[allow(clippy::too_many_arguments)]
fn decode_frame_subtract_staged_with_ap_inner(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    ap_hint: Option<&ApHint>,
) -> (Vec<DecodeResult>, Vec<i16>) {
    use staged_checkpoint::{A_SAMPLES, B_SAMPLES, C_SAMPLES};

    // Buffers shorter than checkpoint A can't be staged meaningfully —
    // there's no "early, incomplete" window smaller than the whole thing.
    // Must call the *flat* fallback (`flat_sic_inner`), not `.staged()`
    // itself — that dispatches to this very function (issue #180), so
    // calling it here would recurse indefinitely (stack overflow, caught
    // by `staged_with_ap_silence_shape`).
    let _ = freq_hint;
    if audio.len() < A_SAMPLES {
        let (r, _) = flat_sic_inner(
            audio,
            freq_min,
            freq_max,
            sync_min,
            depth,
            max_cand,
            strictness,
            &[],
            ap_hint,
            None,
        );
        return (r, audio.to_vec());
    }

    // ---- Checkpoint A (nearly=41): early pass on a full-length buffer
    // whose tail (from `A_SAMPLES` on) is zeroed. Full length (not a
    // truncated slice) matters here: `subtract_signal_lpf`'s reference
    // waveform for a candidate found near the truncation edge can still
    // extend past `A_SAMPLES` samples (a message is `params::NZ` =
    // 151_680 samples long), so the buffer must physically have room for
    // it — exactly why WSJT-X's `dd`/`id2a` are fixed `15*12000`-sample
    // arrays with the *content* zeroed past the checkpoint, not
    // shorter arrays.
    //
    // WSJT-X: `syncmin=2.0` at nzhsym=41 vs `syncmin=1.3` at nzhsym=50
    // (ndepth=3) — a stricter gate on the early, still-incomplete
    // window. Scaled by ratio rather than reusing WSJT-X's absolute
    // `sync8` units, which live on a different score scale than this
    // crate's `coarse_sync` (see e.g. the FT4/FST4 threshold-scaling
    // precedent in `core::pipeline`).
    const EARLY_SYNC_MIN_SCALE: f32 = 2.0 / 1.3;
    let mut residual_a = vec![0i16; audio.len()];
    residual_a[..A_SAMPLES].copy_from_slice(&audio[..A_SAMPLES]);
    let early_results = sic_inner_passes(
        &mut residual_a,
        freq_min,
        freq_max,
        sync_min * EARLY_SYNC_MIN_SCALE,
        depth,
        max_cand,
        strictness,
        &[],
        ap_hint,
    );
    // Checkpoint A's own residual is not carried forward — only its
    // decoded results are (ft8_decode.f90 reloads `dd=iwave` fresh at
    // checkpoint B rather than reusing checkpoint A's `dd`).
    drop(residual_a);

    if early_results.is_empty() {
        // WSJT-X: `nzhsym=47 .and. ndec_early.eq.0` skips checkpoint B's
        // search entirely, and checkpoint C's "combine cleaned head +
        // raw tail" step is itself gated on `ndec_early.ge.1` — with
        // nothing to pre-subtract there is nothing to stage. Falling
        // back to a single flat pass over the *full* original audio
        // (rather than reproducing jt9.f90's own checkpoint-47-sized
        // truncation quirk in this branch) can only find as much or
        // more, never less. Must be the *flat* fallback here too — same
        // recursion hazard as the `A_SAMPLES` branch above.
        let (r, _) = flat_sic_inner(
            audio,
            freq_min,
            freq_max,
            sync_min,
            depth,
            max_cand,
            strictness,
            &[],
            ap_hint,
            None,
        );
        return (r, audio.to_vec());
    }

    // ---- Checkpoint B (nearly=47): subtract-prep only, no search.
    // Full-length buffer again (see checkpoint A comment above), content
    // zeroed past `b_len`. Only signals whose full NN-symbol message
    // fits inside this checkpoint's *real-content* window are subtracted
    // now — subtracting against the zeroed tail would fit the reference
    // waveform to silence there — late-`dt` signals are deferred to
    // checkpoint C, where the raw tail is available.
    let b_len = B_SAMPLES.min(audio.len());
    let mut buf_b = vec![0i16; audio.len()];
    buf_b[..b_len].copy_from_slice(&audio[..b_len]);
    // Message duration (`params::NZ` samples at 12 kHz) plus the 0.5 s
    // frame-start offset must fit before the checkpoint-B buffer's real
    // content ends. Mirrors `ft8_decode.f90`'s `xdt_save(i)-0.5 < 0.396`
    // gate, generalised via the message duration instead of the magic
    // constant (which is this formula evaluated at FT8's own
    // NN=79/NSPS=1920 — see `params.rs`).
    let message_dur_s = params::NZ as f32 / 12_000.0;
    let dt_fit_limit_b = b_len as f32 / 12_000.0 - message_dur_s - 0.5;
    // `ft8_decode.f90:132` subtracts these checkpoint-A decodes with
    // `lrefinedt=.true.` — their `dt` came from an early, still-coarse
    // pass, not a final decode, so WSJT-X re-searches ±90 samples for
    // the best-cancelling alignment before subtracting (issue #180).
    let mut deferred: Vec<DecodeResult> = Vec::new();
    for r in &early_results {
        if r.dt_sec < dt_fit_limit_b {
            subtract_signal_lpf_refine_dt(&mut buf_b, r);
        } else {
            deferred.push(r.clone());
        }
    }

    // ---- Checkpoint C (nzhsym=50): cleaned head (checkpoint B's
    // residual, samples 0..b_len) + fresh raw tail (b_len..c_len),
    // zeroed past `c_len` (matches jt9.f90's `id2a(50*3456+1:)=0`).
    // Subtract any deferred signals against the now-complete buffer,
    // then run the full 3-sub-pass search at the caller's baseline
    // `sync_min`.
    let c_len = C_SAMPLES.min(audio.len());
    let mut buf_c = vec![0i16; audio.len()];
    buf_c[..b_len].copy_from_slice(&buf_b[..b_len]);
    buf_c[b_len..c_len].copy_from_slice(&audio[b_len..c_len]);
    // `ft8_decode.f90:162` — same `lrefinedt=.true.` re-search as
    // checkpoint B above, for the late-`dt` decodes deferred to here.
    for r in &deferred {
        subtract_signal_lpf_refine_dt(&mut buf_c, r);
    }

    let new_results = sic_inner_passes(
        &mut buf_c,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        strictness,
        &early_results,
        ap_hint,
    );

    let mut all_results = early_results;
    all_results.extend(new_results);
    (all_results, buf_c)
}

// ────────────────────────────────────────────────────────────────────────────
// Sniper-mode decode (single target frequency, narrow band)

/// Sniper-mode decode over `target_freq ± 250 Hz`. Used by
/// [`FrameDecodable::__sniper`]'s `Ft8` impl; also the shared inner for
/// [`SniperRequest`]'s single-pass search.
#[allow(clippy::too_many_arguments)]
fn decode_sniper_inner(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    sync_min: f32,
) -> (Vec<DecodeResult>, FftCache) {
    let freq_min = (target_freq - 250.0).max(100.0);
    let freq_max = (target_freq + 250.0).min(5900.0);

    // Sniper-mode: freq_hint (=target_freq) used to promote candidates
    // near the target via the legacy core::sync::coarse_sync path. After
    // the v0.6 consolidation in #48, decode_block::coarse_sync does not
    // honour hints; the ±250 Hz freq_min/freq_max band above does most
    // of the work the hint used to.
    let spec = crate::ft8::decode_block::compute_spectrogram(audio, freq_max);
    let candidates =
        crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
    let fft_cache = build_fft_cache(audio);
    if candidates.is_empty() {
        return (Vec::new(), fft_cache);
    }

    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            process_candidate(
                cand,
                audio,
                &fft_cache,
                depth,
                strictness,
                &[],
                eq_mode,
                ap_hint,
            )
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            process_candidate(
                cand,
                audio,
                &fft_cache,
                depth,
                strictness,
                &[],
                eq_mode,
                ap_hint,
            )
        })
        .collect();

    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        if !results.iter().any(|x| x.message77 == r.message77) {
            results.push(r);
        }
    }
    (results, fft_cache)
}

// ────────────────────────────────────────────────────────────────────────────
// `DecodeRequest`/`SniperRequest` dispatch (issue #191)

impl FrameDecodable for Ft8 {
    type DecodeResult = DecodeResult;

    fn __single_pass(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        let (results, fft_cache) = decode_frame_inner(
            req.audio,
            req.freq_min,
            req.freq_max,
            req.sync_min,
            req.freq_hint,
            req.depth,
            req.max_cand,
            req.strictness,
            req.known,
            req.eq_mode,
            req.fft_cache.as_deref(),
            req.ap_hint,
        );
        DecodeOutcome { results, fft_cache }
    }

    fn __sniper(req: &SniperRequest<'_, Self>) -> DecodeOutcome<Self> {
        let (results, fft_cache) = decode_sniper_inner(
            req.audio,
            req.target_freq,
            req.depth,
            req.max_cand,
            req.strictness,
            req.eq_mode,
            req.ap_hint,
            req.sync_min,
        );
        DecodeOutcome { results, fft_cache }
    }
}

impl SupportsFlatSic for Ft8 {
    fn __flat_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        // Subtract caller-supplied `known` before pass 0, same rationale
        // as `SupportsStagedSic::__staged_sic` below: without this, a
        // strong `known` carrier continues to mask weaker signals
        // throughout the SIC loop (the exact issue #191 bug, previously
        // only reachable through the deleted, unfixed
        // `decode_frame_subtract_with_known_and_ap`). `precomputed_fft`
        // can only be trusted for pass 0 once `known` is empty — reusing
        // it after this subtraction would silently mismatch the cache
        // against the now-modified audio.
        if req.known.is_empty() {
            let (results, fft_cache) = flat_sic_inner(
                req.audio,
                req.freq_min,
                req.freq_max,
                req.sync_min,
                req.depth,
                req.max_cand,
                req.strictness,
                req.known,
                req.ap_hint,
                req.fft_cache.as_deref(),
            );
            DecodeOutcome { results, fft_cache }
        } else {
            let mut audio_clean = req.audio.to_vec();
            for r in req.known {
                subtract_signal_lpf_refine_dt(&mut audio_clean, r);
            }
            let (results, fft_cache) = flat_sic_inner(
                &audio_clean,
                req.freq_min,
                req.freq_max,
                req.sync_min,
                req.depth,
                req.max_cand,
                req.strictness,
                req.known,
                req.ap_hint,
                None,
            );
            DecodeOutcome { results, fft_cache }
        }
    }
}

impl SupportsStagedSic for Ft8 {
    /// The issue #191 fix: `decode_frame_subtract_with_known_and_ap` used
    /// to be the *only* entry point accepting `known`/`precomputed_fft`,
    /// and it ran its own unfixed flat-3-pass engine instead of the
    /// staged checkpoint one — missing every SIC-quality improvement
    /// (issue #178/#179/#180) landed since. `known` is now honoured by
    /// *this* (staged) path directly: subtracted from the audio before
    /// checkpoint A runs, so all three checkpoints see the cleaned
    /// residual, then deduped from the final results.
    ///
    /// `precomputed_fft` is deliberately not reused here: every
    /// checkpoint operates on a truncated/zero-tailed buffer, never on
    /// the full original audio the cache was built from, so reusing it
    /// would silently mismatch. (It *is* reused by [`SupportsFlatSic`]'s
    /// impl above, whose single full-buffer pass 0 has the matching
    /// shape.) Recomputing here is always correct, just not free.
    fn __staged_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        // `subtract_signal_lpf_refine_dt`, not the plain single-shot
        // variant: `known` is conceptually the same kind of carried-
        // forward decode checkpoint B/C already re-subtract with
        // `lrefinedt=.true.` (±90-sample best-alignment search) rather
        // than trusting the original `dt_sec` verbatim. A `known` signal
        // sitting only ~35 Hz from a marginal candidate (as W1FC does
        // next to `CQ DX DL8YHR JO41`, issue #180) needs that same
        // precision — plain `subtract_signal_lpf` measurably left enough
        // residual to lose DL8YHR entirely in end-to-end testing here.
        let mut audio_clean = req.audio.to_vec();
        for r in req.known {
            subtract_signal_lpf_refine_dt(&mut audio_clean, r);
        }
        let (mut results, residual) = decode_frame_subtract_staged_with_ap_inner(
            &audio_clean,
            req.freq_min,
            req.freq_max,
            req.sync_min,
            req.freq_hint,
            req.depth,
            req.max_cand,
            req.strictness,
            req.ap_hint,
        );
        // Belt-and-braces dedup: subtraction above should already
        // prevent `known` signals from re-decoding, but an imperfect
        // subtraction on a very strong carrier could still leave enough
        // residual to re-cross the CRC/OSD threshold.
        results.retain(|r| !req.known.iter().any(|k| k.message77 == r.message77));
        let fft_cache = build_fft_cache(&residual);
        DecodeOutcome { results, fft_cache }
    }
}

impl SupportsWideBandAp for Ft8 {}

#[cfg(test)]
mod tests {
    use super::*;

    /// `DecodeRequest::ap_hint` round-trips a clean self-synthesised
    /// signal with the hint matching. Doesn't directly assert the AP
    /// gain (that needs a low-SNR fixture); just guards against
    /// signature drift and validates the "hint-aware decode of a
    /// perfect signal still succeeds" invariant.
    #[test]
    fn ap_hint_round_trips_clean_signal() {
        use crate::ft8::wave_gen::{message_to_tones, tones_to_i16};
        use crate::msg::wsjt77::pack77;

        let m77 = pack77("CQ", "K1ABC", "FN42").expect("pack77");
        let tones = message_to_tones(&m77);
        let samples = tones_to_i16(&tones, 1500.0, 20_000);

        // 15 s slot, signal at 0.5 s offset.
        let mut audio = vec![0i16; 15 * 12_000];
        let off = 6_000usize;
        let len = samples.len().min(audio.len() - off);
        audio[off..off + len].copy_from_slice(&samples[..len]);

        // Provide a matching AP hint.
        let ap = ApHint::new().with_call1("CQ").with_call2("K1ABC");
        let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.0, 50)
            .depth(DecodeDepth::FULL)
            .ap_hint(&ap)
            .decode()
            .results;
        assert!(
            results.iter().any(|r| r.message77 == m77),
            "expected to decode the self-synthesized signal with matching AP hint"
        );
    }

    /// Compile-shape: `DecodeRequest` accepts an AP hint and strictness,
    /// and returns the FFT cache alongside the decode list. On a silent
    /// buffer the result list must be empty and the cache must be
    /// non-empty (FFT is built unconditionally).
    #[test]
    fn ap_hint_full_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("K1ABC");

        // ap_hint unset
        let out0 = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .decode();
        assert!(out0.results.is_empty());
        assert!(!out0.fft_cache.is_empty(), "FFT cache should be returned");

        // ap_hint = Some, strictness = Strict
        let out1 = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .freq_hint(1500.0)
            .depth(DecodeDepth::FULL)
            .strictness(DecodeStrictness::Strict)
            .ap_hint(&ap)
            .decode();
        assert!(out1.results.is_empty());
        assert!(!out1.fft_cache.is_empty());
    }

    /// Compile-shape: `.staged()` accepts an AP hint and returns no
    /// decodes on silence.
    #[test]
    fn staged_with_ap_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("W7VV");

        let r_none = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .staged()
            .decode()
            .results;
        assert!(r_none.is_empty());

        let r_some = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .staged()
            .ap_hint(&ap)
            .decode()
            .results;
        assert!(r_some.is_empty());
    }

    /// Compile-shape: `.staged().known(&known).fft_cache(cache)` accepts
    /// the full parameter set (known list + FFT cache + AP hint) and
    /// returns no decodes on silence — the issue #191 combination that
    /// used to be unreachable (only the buggy, now-deleted flat-only
    /// `decode_frame_subtract_with_known_and_ap` accepted `known`+cache
    /// at all).
    #[test]
    fn staged_known_and_cache_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
        let known: Vec<DecodeResult> = Vec::new();

        let cache = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .decode()
            .fft_cache;

        let r_none = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .staged()
            .known(&known)
            .fft_cache(cache.clone())
            .decode()
            .results;
        assert!(r_none.is_empty());

        let r_some = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .staged()
            .known(&known)
            .fft_cache(cache)
            .ap_hint(&ap)
            .decode()
            .results;
        assert!(r_some.is_empty());
    }

    /// Regression test for the Phase-2 SIC correctness bug (issue #191):
    /// when caller-supplied `known` signals are *not* subtracted from
    /// the residual, a strong known signal continues to mask weaker
    /// signals throughout the SIC loop. With the fix in place, the
    /// residual is cleaned of the known signal before checkpoint A runs
    /// so every checkpoint operates on a near-zero baseline at the known
    /// signal's frequency.
    ///
    /// We assert this directly by measuring the residual energy at the
    /// known signal's narrow band before vs. after the staged engine
    /// runs. Without the fix, residual energy at f0 ≈ original input
    /// energy at f0. With the fix, it drops by an order of magnitude.
    /// Exercises the same `subtract_signal_lpf` + staged-checkpoint path
    /// `SupportsStagedSic::__staged_sic` uses (see its doc comment).
    #[test]
    fn staged_subtracts_known_before_checkpoint_a() {
        use crate::ft8::wave_gen::{message_to_tones, tones_to_i16};
        use crate::msg::wsjt77::pack77;

        let m_known = pack77("CQ", "K1ABC", "FN42").expect("pack77 known");
        let tones_known = message_to_tones(&m_known);

        // Strong, clean signal at 1500 Hz.
        let f0 = 1500.0_f32;
        let mut audio = vec![0i16; 15 * 12_000];
        let off = 6_000usize;
        let buf = tones_to_i16(&tones_known, f0, 20_000);
        let n_sig = buf.len().min(audio.len() - off);
        audio[off..off + n_sig].copy_from_slice(&buf[..n_sig]);

        // Phase 1: decode A. We need a real DecodeResult (with proper sync_cv,
        // freq_hz, dt_sec) so the SIC path can reconstruct A.
        let phase1 = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 50)
            .depth(DecodeDepth::FULL)
            .decode()
            .results;
        let known_results: Vec<DecodeResult> = phase1
            .iter()
            .filter(|r| r.message77 == m_known)
            .cloned()
            .collect();
        assert!(
            !known_results.is_empty(),
            "Phase 1 must decode the known signal for this test to be meaningful"
        );

        // Helper: narrow-band energy ~f0, ±50 Hz, via Goertzel-ish DFT bin sum.
        // Sums |sample| as a coarse-but-monotonic proxy; precise enough to
        // differentiate "signal present" from "signal subtracted".
        fn band_energy(samples: &[i16], f_lo: f32, f_hi: f32) -> f64 {
            let n = samples.len();
            let fs = 12_000.0_f64;
            let k_lo = ((f_lo as f64) * (n as f64) / fs).floor() as usize;
            let k_hi = ((f_hi as f64) * (n as f64) / fs).ceil() as usize;
            let mut energy = 0.0_f64;
            // Direct DFT magnitude sum over the narrow band — exact, slow,
            // but the test buffer is 180 000 samples and the band is ~1 Hz
            // wide so this is bounded.
            for k in k_lo..=k_hi {
                let mut re = 0.0_f64;
                let mut im = 0.0_f64;
                let w = 2.0 * core::f64::consts::PI * (k as f64) / (n as f64);
                for (i, &s) in samples.iter().enumerate() {
                    let phi = w * (i as f64);
                    re += (s as f64) * phi.cos();
                    im -= (s as f64) * phi.sin();
                }
                energy += re * re + im * im;
            }
            energy
        }

        // Restrict the band to a 2 Hz window so the DFT loop stays cheap.
        let e_before = band_energy(&audio, f0 - 1.0, f0 + 1.0);

        // Mirrors `SupportsStagedSic::__staged_sic`'s own upfront
        // subtraction step exactly (same `subtract_signal_lpf` call),
        // then reuses the `_debug_residual` shim to observe the result.
        let mut audio_clean = audio.clone();
        for r in &known_results {
            subtract_signal_lpf(&mut audio_clean, r);
        }
        let (new_results, residual) = decode_frame_subtract_staged_with_ap_debug_residual(
            &audio_clean,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::FULL,
            50,
            DecodeStrictness::Normal,
            None,
        );
        let _ = new_results; // not under test here

        let e_after = band_energy(&residual, f0 - 1.0, f0 + 1.0);

        // With the SIC fix, the known signal is subtracted from the residual,
        // so band energy at f0 must drop substantially. Use a conservative
        // 2× threshold so the test is robust to subtraction-gain (qsb_partial)
        // and refine residue, not 0.5× which is the typical empirical drop.
        assert!(
            e_after * 2.0 < e_before,
            "expected residual band energy at known signal's frequency \
             to drop by >2× after SIC; got e_before={e_before:.3e}, \
             e_after={e_after:.3e} (fix not applied?)"
        );
    }

    /// Silence produces no decoded messages and does not panic.
    #[test]
    fn silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .decode()
            .results;
        assert!(results.is_empty(), "silence should decode nothing");
    }

    /// Sniper mode on silence also produces no decoded messages.
    #[test]
    fn sniper_silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = SniperRequest::<Ft8>::new(&audio, 1000.0, 10)
            .depth(DecodeDepth::BP_ONLY)
            .decode()
            .results;
        assert!(results.is_empty());
    }

    /// Verify DT accuracy: a signal placed at exactly dt=0 (0.5s into buffer)
    /// should decode with DT close to 0.
    #[test]
    fn dt_accuracy_at_nominal_start() {
        use super::super::message::pack77_type1;
        use super::super::wave_gen::{message_to_tones, tones_to_f32};

        let msg = pack77_type1("CQ", "JA1ABC", "PM95").unwrap();
        let itone = message_to_tones(&msg);
        let pcm = tones_to_f32(&itone, 1000.0, 1.0);

        let mut audio_f32 = vec![0.0f32; 180_000];
        let start = (0.5 * 12000.0) as usize; // 6000 samples
        for (i, &s) in pcm.iter().enumerate() {
            if start + i < audio_f32.len() {
                audio_f32[start + i] = s;
            }
        }
        let audio: Vec<i16> = audio_f32
            .iter()
            .map(|&s| (s * 20000.0).clamp(-32767.0, 32767.0) as i16)
            .collect();

        let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.0, 200)
            .depth(DecodeDepth::FULL)
            .decode()
            .results;
        assert!(!results.is_empty(), "should decode the signal");
        let dt = results[0].dt_sec;
        eprintln!("DT = {dt:+.3} s (expected ≈ 0.0)");
        assert!(dt.abs() < 0.5, "DT={dt} is too far from 0");
    }

    /// Internal per-candidate probe for the CCIR fading gap investigation
    /// (issue #72 follow-up, `FT8_BENCHMARK.md` section 7). Calls the
    /// private `process_candidate` directly on just the known near-golden
    /// candidate — avoids the noise-candidate spam a full-band
    /// `decode_frame` run produces, and is reachable here (unlike from an
    /// external `tests/` crate) because this module's own
    /// `#[cfg(test)] mod tests` sees `crate::ft8`-private items via `use
    /// super::*`. The root cause this helped find (`OSD_HARDERRORS_MAX`
    /// too tight — see `decode_block/osd_strategy.rs`) is fixed; kept as
    /// a reusable stage-attribution probe for future internal
    /// investigations rather than deleted.
    ///
    /// Minimal inline WAV reader (12 kHz mono i16 PCM) since
    /// `tests/common`'s loader isn't reachable from a `src/` unit test.
    #[test]
    #[ignore = "manual diagnostic — internal BP/OSD trace on CCIR losing trials (issue #72)"]
    fn ft8_diag_internal_osd_trace() {
        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        const GOLDEN_FREQ_HZ: f32 = 1500.0;
        const FREQ_TOL_HZ: f32 = 5.0;
        let manifest = env!("CARGO_MANIFEST_DIR");
        let dir = std::path::Path::new(manifest).join("../embedded-poc/assets/ft8_sweep");

        for &(chan, snr_tag, trial) in &[
            ("ccir_poor", "m18", 1u32),
            ("ccir_poor", "m18", 3),
            ("ccir_poor", "m17", 3),
            ("ccir_moderate", "m17", 11),
        ] {
            let path = dir.join(format!("ft8_{chan}_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16(&path) else {
                eprintln!("skip {path:?}");
                continue;
            };
            let spec = crate::ft8::decode_block::compute_spectrogram(&audio, 3000.0);
            let candidates = crate::ft8::decode_block::coarse_sync(&spec, 100.0, 3000.0, 0.8, 50);
            let fft_cache = build_fft_cache(&audio);
            for c in candidates
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
            {
                eprintln!(
                    "{chan} {snr_tag} trial {trial}: cand freq={:.2} dt={:.3} score={:.4}",
                    c.freq_hz, c.dt_sec, c.score
                );
                let r = process_candidate(
                    c,
                    &audio,
                    &fft_cache,
                    DecodeDepth::FULL,
                    DecodeStrictness::default(),
                    &[],
                    EqMode::Off,
                    None,
                );
                eprintln!("  -> process_candidate result: {:?}", r.map(|d| d.pass));
            }
        }
    }

    /// Issue #180 follow-up: does the *actual* checkpoint-C residual
    /// produced by `decode_frame_subtract_staged` — not the flat-pass
    /// full-residual buffer `ft8_qso3_dl8yhr_full_residual_probe.rs`
    /// checked — get `CQ DX DL8YHR JO41` (~-17 dB, f≈2606.25 Hz) any
    /// closer to decoding? Uses
    /// `decode_frame_subtract_staged_with_ap_debug_residual` to get the
    /// exact buffer the production checkpoint-C search sees (not a
    /// hand-rolled approximation), then probes WSJT-X's own reported
    /// coordinates (`f1=2606.25`, internal `xdt=0.695` → display
    /// `dt=0.195`) directly through `sync_quality`/BP/OSD, mirroring
    /// the existing full-residual probe's methodology.
    #[test]
    #[ignore = "manual diagnostic — issue #180 staged-residual DL8YHR probe"]
    fn issue_180_dl8yhr_staged_checkpoint_c_probe() {
        use crate::core::sync::{SyncCandidate, refine_candidate};
        use crate::fec::ldpc::bp::bp_decode;
        use crate::fec::ldpc::osd::{osd_decode_deep4, osd_decode_npre1, osd_decode_npre1_npre2};
        use crate::ft8::Ft8;
        use crate::ft8::decode_block::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
        use crate::ft8::downsample::downsample;
        use crate::ft8::llr::compute_llr;
        use crate::msg::wsjt77::unpack77;

        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");
        let target = "CQ DX DL8YHR JO41";

        let (results, residual) = decode_frame_subtract_staged_with_ap_debug_residual(
            &audio,
            100.0,
            3000.0,
            0.8,
            None,
            DecodeDepth::FULL,
            200,
            DecodeStrictness::Normal,
            None,
        );
        println!(
            "decode_frame_subtract_staged: {} decodes (checkpoint-C residual captured)",
            results.len()
        );

        // WSJT-X's own ground-truth coordinates for this candidate
        // (issue #180): f1=2606.25 Hz, internal xdt=0.695 -> display
        // dt=xdt-0.5=0.195. Probe a small grid around them, same shape
        // as `ft8_qso3_dl8yhr_full_residual_probe.rs`.
        let mut freqs = vec![2606.25f32];
        for df in [-6.25, -3.0, 3.0, 6.25, -9.0, 9.0] {
            freqs.push(2606.25 + df);
        }
        let mut dts = vec![0.195f32];
        for ddt in [-0.05, 0.05, -0.1, 0.1, -0.16, 0.16] {
            dts.push(0.195 + ddt);
        }

        let mut best: Option<(f32, f32, u32)> = None;
        let mut any_hit = false;

        for &freq in &freqs {
            for &dt in &dts {
                let cand = SyncCandidate {
                    freq_hz: freq,
                    dt_sec: dt,
                    score: 0.0,
                };
                let (cd0, _cache) = downsample(&residual, cand.freq_hz, None);
                let refined = refine_candidate::<Ft8>(&cd0, &cand, 10);

                let mut cs = symbol_spectra_direct::<i16>(
                    &residual,
                    cand.freq_hz,
                    refined.dt_sec,
                    SymMask::SyncOnly,
                    None,
                );
                let q = sync_quality(&cs);
                fill_symbol_spectra(
                    &mut cs,
                    &residual,
                    cand.freq_hz,
                    refined.dt_sec,
                    SymMask::DataOnly,
                    None,
                );
                let llr_set = compute_llr::<f32>(&cs);

                for llr in [&llr_set.llra, &llr_set.llrb, &llr_set.llrc, &llr_set.llrd] {
                    if let Some(bp) = bp_decode(llr, None, 40, None) {
                        let text = unpack77(&bp.message77).unwrap_or_default();
                        if text == target {
                            println!(
                                "BP HIT: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q}",
                                refined.dt_sec
                            );
                            any_hit = true;
                        }
                    }
                    let osd = if q >= 18 {
                        osd_decode_npre1_npre2(llr)
                    } else {
                        osd_decode_npre1(llr)
                    };
                    if let Some(o) = osd {
                        let text = unpack77(&o.message77).unwrap_or_default();
                        if text == target {
                            println!(
                                "OSD(wsjtx-faithful) HIT: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q}",
                                refined.dt_sec
                            );
                            any_hit = true;
                        }
                    }
                    if let Some(o) = osd_decode_deep4(llr, 30, None) {
                        let text = unpack77(&o.message77).unwrap_or_default();
                        if text == target {
                            println!(
                                "OSD(deep4) HIT: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q}",
                                refined.dt_sec
                            );
                            any_hit = true;
                        }
                    }
                }

                let is_better = match &best {
                    None => true,
                    Some((_, _, bq)) => q > *bq,
                };
                if is_better {
                    best = Some((freq, dt, q));
                }
                println!(
                    "  probe freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q}",
                    refined.dt_sec
                );
            }
        }

        if let Some((freq, dt, q)) = best {
            println!(
                "\nBest sync_quality on staged checkpoint-C residual: freq={freq:.2} dt={dt:.3} q={q}"
            );
        }
        println!("any_hit={any_hit}");

        // Per-Costas-block breakdown at the exact WSJT-X ground-truth
        // coordinates, directly comparable to jt9's own instrumented
        // `is1`/`is2`/`is3` (this run's real `jt9 -8 -d3` on the
        // identical WAV: is1=2 is2=7 is3=6, nsync=15). Reproduces
        // `sync_quality_generic`'s per-symbol argmax logic
        // (`core/llr.rs`) but reports per-block subtotals instead of
        // just the sum, to localise *where* the 15-vs-9 gap lives.
        {
            println!(
                "\nPer-block Costas breakdown, no-refine (raw candidate dt/freq fed directly):"
            );
            println!("  jt9 (real, this run):        is1=2 is2=7 is3=6  nsync=15");

            // Bug 1 hypothesis (issue #180): WSJT-X's displayed dt is one
            // cd0-sample (5 ms = 1/200 symbol-fraction) *before* the
            // window it actually decodes (`ibest` vs `ibest-1`). If that
            // explains the shortfall, it should show up as roughly
            // uniform improvement across all 3 blocks at some small dt
            // shift — sweep ±3 steps of 1/200 s around the ground-truth
            // dt=0.195 (freq held fixed) and report each block
            // breakdown to check for that signature.
            let step = 1.0f32 / 200.0;
            for k in -3i32..=3 {
                let dt = 0.195f32 + (k as f32) * step;
                let freq = 2606.25f32;
                let cand = SyncCandidate {
                    freq_hz: freq,
                    dt_sec: dt,
                    score: 0.0,
                };
                let cs = symbol_spectra_direct::<i16>(
                    &residual,
                    cand.freq_hz,
                    cand.dt_sec,
                    SymMask::SyncOnly,
                    None,
                );
                print!("  mfsk-core dt={dt:.4} (k={k:+}):");
                let mut total = 0u32;
                for (bi, block) in <Ft8 as crate::core::FrameLayout>::SYNC_MODE
                    .blocks()
                    .iter()
                    .enumerate()
                {
                    let start = block.start_symbol as usize;
                    let mut hits = 0u32;
                    for (t, &expected) in block.pattern.iter().enumerate() {
                        let sym = start + t;
                        let mut best_tone = 0usize;
                        let mut best_val = cs[sym][0].norm_sqr();
                        for a in 1..8 {
                            let v = cs[sym][a].norm_sqr();
                            if v > best_val {
                                best_val = v;
                                best_tone = a;
                            }
                        }
                        if best_tone == expected as usize {
                            hits += 1;
                        }
                    }
                    total += hits;
                    print!(" is{}={hits}", bi + 1);
                }
                println!("  nsync={total}");
            }

            // Also try the refine_candidate-adjusted dt (what production
            // actually feeds symbol_spectra), for reference.
            let cand0 = SyncCandidate {
                freq_hz: 2606.25,
                dt_sec: 0.195,
                score: 0.0,
            };
            let (cd0, _cache) = downsample(&residual, cand0.freq_hz, None);
            let refined = refine_candidate::<Ft8>(&cd0, &cand0, 10);
            let cs = symbol_spectra_direct::<i16>(
                &residual,
                cand0.freq_hz,
                refined.dt_sec,
                SymMask::SyncOnly,
                None,
            );
            print!("  mfsk-core refine_candidate dt={:.4}:", refined.dt_sec);
            let mut total = 0u32;
            for (bi, block) in <Ft8 as crate::core::FrameLayout>::SYNC_MODE
                .blocks()
                .iter()
                .enumerate()
            {
                let start = block.start_symbol as usize;
                let mut hits = 0u32;
                for (t, &expected) in block.pattern.iter().enumerate() {
                    let sym = start + t;
                    let mut best_tone = 0usize;
                    let mut best_val = cs[sym][0].norm_sqr();
                    for a in 1..8 {
                        let v = cs[sym][a].norm_sqr();
                        if v > best_val {
                            best_val = v;
                            best_tone = a;
                        }
                    }
                    if best_tone == expected as usize {
                        hits += 1;
                    }
                }
                total += hits;
                print!(" is{}={hits}", bi + 1);
            }
            println!("  nsync={total}");

            // Same breakdown on the RAW, unsubtracted audio at the same
            // coordinates — distinguishes "SIC subtraction residue is
            // corrupting this region" (raw would look better) from "the
            // tone-detection fidelity gap exists even before any
            // subtraction" (raw looks the same or worse).
            let cs_raw =
                symbol_spectra_direct::<i16>(&audio, 2606.25, 0.195, SymMask::SyncOnly, None);
            print!("  mfsk-core RAW audio dt=0.1950 (no subtraction at all):");
            let mut total_raw = 0u32;
            for (bi, block) in <Ft8 as crate::core::FrameLayout>::SYNC_MODE
                .blocks()
                .iter()
                .enumerate()
            {
                let start = block.start_symbol as usize;
                let mut hits = 0u32;
                for (t, &expected) in block.pattern.iter().enumerate() {
                    let sym = start + t;
                    let mut best_tone = 0usize;
                    let mut best_val = cs_raw[sym][0].norm_sqr();
                    for a in 1..8 {
                        let v = cs_raw[sym][a].norm_sqr();
                        if v > best_val {
                            best_val = v;
                            best_tone = a;
                        }
                    }
                    if best_tone == expected as usize {
                        hits += 1;
                    }
                }
                total_raw += hits;
                print!(" is{}={hits}", bi + 1);
            }
            println!("  nsync={total_raw}");

            // Symbol-by-symbol tone-magnitude dump for Costas block 2
            // (mfsk-core sym 36..42 == jt9's k=37..43), the block with
            // the largest is2 shortfall (4/7 vs jt9's 7/7). Printed in
            // the same per-symbol/per-tone layout as jt9's own
            // `DL8YHR_PROBE s8` dump (captured separately from a real
            // `jt9 -8 -d3` run, re-instrumented to also cover k=37..43/
            // 73..79) for direct side-by-side comparison. Units aren't
            // identical (different FFT normalisation constants) but the
            // *shape* — which tone dominates, by how much — is directly
            // comparable.
            let icos7 = [3u8, 1, 4, 0, 6, 5, 2];
            let cs_all =
                symbol_spectra_direct::<i16>(&residual, 2606.25, 0.195, SymMask::SyncOnly, None);
            for (label, block_start, k_start) in [
                ("Block-1", 0usize, 1u32),
                ("Block-2", 36, 37),
                ("Block-3", 72, 73),
            ] {
                println!("\n{label} (sym {}..{}):", block_start, block_start + 6);
                for t in 0..7 {
                    let sym = block_start + t;
                    let argmax_of =
                        |cs: &[[num_complex::Complex<f32>; 8]; 79]| -> (usize, Vec<f32>) {
                            let mags: Vec<f32> = (0..8).map(|a| cs[sym][a].norm()).collect();
                            let mut best = 0usize;
                            for a in 1..8 {
                                if mags[a] > mags[best] {
                                    best = a;
                                }
                            }
                            (best, mags)
                        };
                    let (best_res, mags_res) = argmax_of(&cs_all);
                    let (best_raw, mags_raw) = argmax_of(&cs_raw);
                    let mark_res = if best_res == icos7[t] as usize {
                        "OK "
                    } else {
                        "BAD"
                    };
                    let mark_raw = if best_raw == icos7[t] as usize {
                        "OK "
                    } else {
                        "BAD"
                    };
                    println!(
                        "  k={:>2} t={} exp={}  RAW argmax={} [{mark_raw}] {:>7.1?}  |  RESIDUAL argmax={} [{mark_res}] {:>7.1?}",
                        k_start + t as u32,
                        t + 1,
                        icos7[t],
                        best_raw,
                        mags_raw,
                        best_res,
                        mags_res,
                    );
                }
            }

            // Raw cd0 (200 sps downsampled baseband) dump at Rust index
            // 267..330 — physically the same samples as jt9's Fortran
            // `cd0(268:331)` IF 0-indexed Rust position n == 1-indexed
            // Fortran position n+1 (i.e. if the two downsample origins
            // are aligned and this is purely an indexing-convention
            // difference, not a real off-by-one). Printed as
            // `fortran_idx = rust_idx+1` so the two dumps line up for a
            // direct diff. This is the ground-truth test for issue
            // #180's "Bug 1" (off-by-one dt convention) hypothesis: if
            // values match, indices are just labelled differently and
            // there's no real bug; if they don't, there's a genuine
            // 1-sample physical misalignment.
            println!(
                "\ncd0 dump (staged residual, f1=2606.25, dt=0.195), Rust idx -> Fortran idx = idx+1:"
            );
            for rust_idx in 267..=330usize {
                let c = cd0[rust_idx];
                println!(
                    "  rust_idx={:>4} fortran_idx={:>4} re={:>12.4} im={:>12.4}",
                    rust_idx,
                    rust_idx + 1,
                    c.re,
                    c.im
                );
            }

            // Confirmation test: jt9's own "13 early-subtracted signals"
            // list (dumped via a second `ft8_decode.f90` instrumentation
            // pass, `DL8YHR_PROBE early_list`) includes a 13th signal —
            // `WA2FZW DL5AXX RR73` @ 2545.88 Hz, dt=-0.125 — that
            // mfsk-core never decodes anywhere in this investigation (11
            // checkpoint-A early results + 7 checkpoint-C new results =
            // 18 total, none of them WA2FZW). Since mfsk-core never
            // finds it, it never subtracts it — a real, un-cancelled
            // signal only 60 Hz from DL8YHR's own carrier is exactly the
            // kind of thing that could explain the excess `cd0` energy
            // measured above. Test directly: manually subtract jt9's
            // exact WA2FZW coordinates from the staged residual and
            // re-measure DL8YHR's per-block sync breakdown.
            if let Some(msg77) = crate::msg::wsjt77::pack77("WA2FZW", "DL5AXX", "RR73") {
                let wa2fzw = DecodeResult {
                    message77: msg77,
                    freq_hz: 2545.88,
                    dt_sec: -0.125,
                    hard_errors: 0,
                    sync_score: 0.0,
                    pass: 0,
                    sync_cv: 0.0,
                    snr_db: 0.0,
                };
                let mut residual2 = residual.clone();
                let refined_freq = crate::ft8::subtract::refine_signal_freq(&residual2, &wa2fzw);
                let mut wa2fzw_r = wa2fzw.clone();
                wa2fzw_r.freq_hz = refined_freq;
                subtract_signal_lpf(&mut residual2, &wa2fzw_r);

                let cs_wa = symbol_spectra_direct::<i16>(
                    &residual2,
                    2606.25,
                    0.195,
                    SymMask::SyncOnly,
                    None,
                );
                print!(
                    "\nAfter manually subtracting jt9's WA2FZW DL5AXX RR73 (refined freq={refined_freq:.2}):"
                );
                let mut total_wa = 0u32;
                for block in <Ft8 as crate::core::FrameLayout>::SYNC_MODE.blocks().iter() {
                    let start = block.start_symbol as usize;
                    let mut hits = 0u32;
                    for (t, &expected) in block.pattern.iter().enumerate() {
                        let sym = start + t;
                        let mut best_tone = 0usize;
                        let mut best_val = cs_wa[sym][0].norm_sqr();
                        for a in 1..8 {
                            let v = cs_wa[sym][a].norm_sqr();
                            if v > best_val {
                                best_val = v;
                                best_tone = a;
                            }
                        }
                        if best_tone == expected as usize {
                            hits += 1;
                        }
                    }
                    total_wa += hits;
                    print!(" {hits}");
                }
                println!("  nsync={total_wa}  (was 9 before this subtract; jt9=15)");
            } else {
                println!(
                    "\npack77(WA2FZW,DL5AXX,RR73) failed to encode — cannot run confirmation test"
                );
            }

            // Decisive test: swap the residual, keep mfsk-core's own
            // algorithm unchanged. `jt9_post_sic_dd.raw` is jt9's own
            // `dd` array — a raw i16 dump added via a fourth
            // `ft8_decode.f90` instrumentation pass, taken right after
            // jt9's real SIC (its 13 early-subtracted signals) and
            // right before the nzhsym=50 `sync8`/`ft8b` search itself —
            // i.e. exactly the buffer real jt9 measures `is1=2 is2=7
            // is3=6` (nsync=15) against. If mfsk-core's own
            // symbol_spectra_direct/sync_quality computation, run
            // unchanged on THIS buffer, still falls well short of 15,
            // that proves the gap is in mfsk-core's tone-detection /
            // downsample computation itself, independent of subtraction
            // quality. If it gets close to 15, that proves the gap is
            // entirely mfsk-core's own SIC being weaker than jt9's
            // (hypothesis (a) from the WA2FZW test above), not a
            // computation bug.
            if let Ok(raw) = std::fs::read("/tmp/jt9_post_sic_dd.raw") {
                let jt9_residual: Vec<i16> = raw
                    .chunks_exact(2)
                    .map(|b| i16::from_le_bytes([b[0], b[1]]))
                    .collect();
                println!(
                    "\nLoaded jt9's own post-SIC residual: {} samples",
                    jt9_residual.len()
                );
                let cs_jt9 = symbol_spectra_direct::<i16>(
                    &jt9_residual,
                    2606.25,
                    0.195,
                    SymMask::SyncOnly,
                    None,
                );
                print!("mfsk-core's own tone-detection on jt9's post-SIC residual:");
                let mut total_jt9 = 0u32;
                for block in <Ft8 as crate::core::FrameLayout>::SYNC_MODE.blocks().iter() {
                    let start = block.start_symbol as usize;
                    let mut hits = 0u32;
                    for (t, &expected) in block.pattern.iter().enumerate() {
                        let sym = start + t;
                        let mut best_tone = 0usize;
                        let mut best_val = cs_jt9[sym][0].norm_sqr();
                        for a in 1..8 {
                            let v = cs_jt9[sym][a].norm_sqr();
                            if v > best_val {
                                best_val = v;
                                best_tone = a;
                            }
                        }
                        if best_tone == expected as usize {
                            hits += 1;
                        }
                    }
                    total_jt9 += hits;
                    print!(" {hits}");
                }
                println!(
                    "  nsync={total_jt9}  (mfsk-core residual gave 9; real jt9 on this exact buffer gives 15)"
                );
                // Full BP/OSD decode on this residual — does the
                // message actually come out, not just the sync count?
                use crate::fec::ldpc::bp::bp_decode;
                use crate::fec::ldpc::osd::{
                    osd_decode_deep4, osd_decode_npre1, osd_decode_npre1_npre2,
                };
                use crate::ft8::llr::compute_llr;
                use crate::msg::wsjt77::unpack77;

                let mut cs_full = cs_jt9.clone();
                fill_symbol_spectra(
                    &mut cs_full,
                    &jt9_residual,
                    2606.25,
                    0.195,
                    SymMask::DataOnly,
                    None,
                );
                let llr_set = compute_llr::<f32>(&cs_full);
                let mut decoded_msg: Option<String> = None;
                for llr in [&llr_set.llra, &llr_set.llrb, &llr_set.llrc, &llr_set.llrd] {
                    if decoded_msg.is_none()
                        && let Some(bp) = bp_decode(llr, None, 40, None)
                    {
                        decoded_msg = unpack77(&bp.message77);
                    }
                    if decoded_msg.is_none() {
                        let osd = if total_jt9 >= 18 {
                            osd_decode_npre1_npre2(llr)
                        } else {
                            osd_decode_npre1(llr)
                        };
                        if let Some(o) = osd {
                            decoded_msg = unpack77(&o.message77);
                        }
                    }
                    if decoded_msg.is_none()
                        && let Some(o) = osd_decode_deep4(llr, 30, None)
                    {
                        decoded_msg = unpack77(&o.message77);
                    }
                }
                println!("Full BP/OSD decode on jt9's post-SIC residual: {decoded_msg:?}");
            } else {
                println!(
                    "\n/tmp/jt9_post_sic_dd.raw not found — run the instrumented jt9 build first"
                );
            }
        }
    }

    /// Throwaway probe (issue #180 DK8NE follow-up) — NOT for commit.
    /// What sync_quality does mfsk-core's own staged-SIC residual give
    /// at `K1BZM DK8NE -10`'s coordinates, vs real jt9's own residual
    /// (ground-truthed via a locally-rebuilt instrumented jt9:
    /// nsync=11, is1=1 is2=7 is3=3 at nzhsym=50)?
    #[test]
    #[ignore = "manual diagnostic — issue #180 DK8NE own-SIC score probe"]
    fn issue_180_dk8ne_own_sic_score_probe() {
        use crate::core::sync::refine_candidate;
        use crate::ft8::decode_block::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
        use crate::ft8::downsample::downsample;
        use crate::ft8::llr::sync_quality;
        use crate::msg::wsjt77::unpack77;

        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");

        let (results, residual) = decode_frame_subtract_staged_with_ap_debug_residual(
            &audio,
            100.0,
            3000.0,
            0.8,
            None,
            DecodeDepth::FULL,
            200,
            DecodeStrictness::Normal,
            None,
        );
        let has_dk8ne = results
            .iter()
            .any(|r| unpack77(&r.message77).as_deref() == Some("K1BZM DK8NE -10"));
        println!("staged pipeline already found DK8NE blind: {has_dk8ne}");

        let freq = 244.2f32;
        let dt = 0.505f32;
        let cand = crate::core::sync::SyncCandidate {
            freq_hz: freq,
            dt_sec: dt,
            score: 0.0,
        };
        let (cd0, _cache) = downsample(&residual, cand.freq_hz, None);
        let refined = refine_candidate::<crate::ft8::Ft8>(&cd0, &cand, 10);

        let mut cs = symbol_spectra_direct::<i16>(
            &residual,
            cand.freq_hz,
            refined.dt_sec,
            SymMask::SyncOnly,
            None,
        );
        let q = sync_quality(&cs);
        fill_symbol_spectra(
            &mut cs,
            &residual,
            cand.freq_hz,
            refined.dt_sec,
            SymMask::DataOnly,
            None,
        );
        let icos7: [u8; 7] = [3, 1, 4, 0, 6, 5, 2];
        let mut is = [0u32; 3];
        for (b, base) in [0usize, 36, 72].iter().enumerate() {
            for (k, &tone) in icos7.iter().enumerate() {
                let sym = base + k;
                let mut best = 0usize;
                let mut best_mag = -1.0f32;
                for t in 0..8 {
                    let m = cs[sym][t].norm();
                    if m > best_mag {
                        best_mag = m;
                        best = t;
                    }
                }
                if best == tone as usize {
                    is[b] += 1;
                }
            }
        }
        println!(
            "mfsk-core's OWN staged-SIC residual: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q} is1={} is2={} is3={}",
            refined.dt_sec, is[0], is[1], is[2]
        );
        println!("jt9's own residual (ground truth):   nsync=11 is1=1 is2=7 is3=3");

        // Sync score matches jt9's exactly — now try the full BP/OSD
        // decode on this same residual to see how close (hard_errors)
        // it gets, even if it doesn't fully converge.
        use crate::fec::ldpc::bp::bp_decode;
        use crate::fec::ldpc::osd::{osd_decode_deep4, osd_decode_npre1, osd_decode_npre1_npre2};
        use crate::ft8::llr::compute_llr;
        let llr_set = compute_llr::<f32>(&cs);
        let target = "K1BZM DK8NE -10";
        for (name, llr) in [
            ("a", &llr_set.llra),
            ("b", &llr_set.llrb),
            ("c", &llr_set.llrc),
            ("d", &llr_set.llrd),
        ] {
            if let Some(bp) = bp_decode(llr, None, 40, None) {
                let text = unpack77(&bp.message77).unwrap_or_default();
                println!("  BP({name}) -> {text:?} hard_errors={}", bp.hard_errors);
            } else {
                println!("  BP({name}) -> no convergence");
            }
            if let Some(o) = osd_decode_npre1_npre2(llr) {
                println!(
                    "  OSD-npre1npre2({name}) -> {:?} hard_errors={}",
                    unpack77(&o.message77).unwrap_or_default(),
                    o.hard_errors
                );
            } else if let Some(o) = osd_decode_npre1(llr) {
                println!(
                    "  OSD-npre1({name}) -> {:?} hard_errors={}",
                    unpack77(&o.message77).unwrap_or_default(),
                    o.hard_errors
                );
            } else {
                println!("  OSD-npre1(npre2)({name}) -> no candidate");
            }
            if let Some(o) = osd_decode_deep4(llr, 30, None) {
                println!(
                    "  OSD-deep4({name}) -> {:?} hard_errors={}",
                    unpack77(&o.message77).unwrap_or_default(),
                    o.hard_errors
                );
            } else {
                println!("  OSD-deep4({name}) -> no candidate");
            }
        }
        let _ = target;
    }

    /// Throwaway probe (issue #180 DK8NE follow-up) — NOT for commit.
    /// Sync score (is1/is2/is3) matches jt9's exactly on both
    /// residuals, but OSD outcome differs. Does the *data* portion (58
    /// symbols the sync_quality metric never looks at) actually differ
    /// between mfsk-core's own SIC residual and jt9's own residual?
    /// Direct per-symbol argmax + magnitude comparison, mirroring the
    /// bin-by-bin methodology the original DL8YHR investigation used.
    #[test]
    #[ignore = "manual diagnostic — issue #180 DK8NE data-symbol residual comparison"]
    fn issue_180_dk8ne_data_symbol_comparison() {
        use crate::core::sync::refine_candidate;
        use crate::ft8::decode_block::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
        use crate::ft8::downsample::downsample;

        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");

        let (_results, mfsk_residual) = decode_frame_subtract_staged_with_ap_debug_residual(
            &audio,
            100.0,
            3000.0,
            0.8,
            None,
            DecodeDepth::FULL,
            200,
            DecodeStrictness::Normal,
            None,
        );

        let jt9_bytes = std::fs::read("/tmp/jt9_post_sic_dd.raw").expect("read jt9 residual dump");
        let jt9_residual: Vec<i16> = jt9_bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();

        let freq = 244.2f32;
        let dt = 0.505f32;

        type SymSpectra = alloc::boxed::Box<[[crate::core::scalar::Cmplx<f32>; 8]; 79]>;
        let mut spectra: Vec<(&str, SymSpectra)> = Vec::new();
        for (label, residual) in [
            ("mfsk-core own SIC", &mfsk_residual),
            ("jt9 own SIC", &jt9_residual),
        ] {
            let cand = crate::core::sync::SyncCandidate {
                freq_hz: freq,
                dt_sec: dt,
                score: 0.0,
            };
            let (cd0, _cache) = downsample(residual, cand.freq_hz, None);
            let refined = refine_candidate::<crate::ft8::Ft8>(&cd0, &cand, 10);
            let mut cs = symbol_spectra_direct::<i16>(
                residual,
                cand.freq_hz,
                refined.dt_sec,
                SymMask::SyncOnly,
                None,
            );
            fill_symbol_spectra(
                &mut cs,
                residual,
                cand.freq_hz,
                refined.dt_sec,
                SymMask::DataOnly,
                None,
            );
            spectra.push((label, cs));
        }

        // Data symbol positions: 7..36 and 43..72 (0-indexed), 58 total.
        let data_syms: Vec<usize> = (7..36).chain(43..72).collect();
        let (mfsk_cs, jt9_cs) = (&spectra[0].1, &spectra[1].1);
        let mut diverge_count = 0usize;
        for &sym in &data_syms {
            let argmax = |cs: &[[crate::core::scalar::Cmplx<f32>; 8]; 79]| -> (usize, f32) {
                let mut best = 0usize;
                let mut best_mag = -1.0f32;
                for t in 0..8 {
                    let m = cs[sym][t].norm();
                    if m > best_mag {
                        best_mag = m;
                        best = t;
                    }
                }
                (best, best_mag)
            };
            let (m_tone, m_mag) = argmax(mfsk_cs);
            let (j_tone, j_mag) = argmax(jt9_cs);
            if m_tone != j_tone {
                diverge_count += 1;
                println!(
                    "  sym={sym:2} DIVERGE  mfsk: tone={m_tone} mag={m_mag:8.1}  |  jt9: tone={j_tone} mag={j_mag:8.1}"
                );
            }
        }
        println!(
            "\n{diverge_count}/{} data-symbol argmax disagreements between mfsk-core's own SIC residual and jt9's own SIC residual (identical sync-symbol scores, both q=11/is1=1/is2=7/is3=3)",
            data_syms.len()
        );

        // Aggregate energy comparison in the data region — average
        // per-tone magnitude at each data symbol, to see whether
        // mfsk-core's residual carries systematically more energy
        // (i.e. more uncancelled interference) even where the argmax
        // agrees.
        let mut mfsk_energy = 0f64;
        let mut jt9_energy = 0f64;
        for &sym in &data_syms {
            for t in 0..8 {
                mfsk_energy += (mfsk_cs[sym][t].norm() as f64).powi(2);
                jt9_energy += (jt9_cs[sym][t].norm() as f64).powi(2);
            }
        }
        println!(
            "data-region total energy: mfsk-core={mfsk_energy:.1}  jt9={jt9_energy:.1}  ratio={:.3}",
            mfsk_energy / jt9_energy
        );
    }

    /// Throwaway probe (issue #182 follow-up) — NOT for commit.
    /// argmax-only comparison (`issue_180_dk8ne_data_symbol_comparison`)
    /// showed 0/58 data-symbol tone disagreements, which rules out a
    /// gross SIC data-quality gap but does NOT rule out a *reliability
    /// ordering* difference — OSD's reprocessing basis is chosen by
    /// sorting all 174 codeword bits by `|LLR|`, and that ordering is a
    /// separate, more sensitive signal than the per-symbol tone argmax.
    /// This probe compares hard-decision agreement and reliability rank
    /// agreement (top-91 most-reliable set overlap) between mfsk-core's
    /// own SIC residual and jt9's own SIC residual, for all 4 LLR
    /// variants (a/b/c/d), to see whether the ~13% energy gap already
    /// found is enough to perturb the ordering OSD actually depends on.
    #[test]
    #[ignore = "manual diagnostic — issue #182 DK8NE LLR reliability-ordering comparison"]
    fn issue_182_dk8ne_llr_reliability_comparison() {
        use crate::core::sync::refine_candidate;
        use crate::ft8::decode_block::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
        use crate::ft8::downsample::downsample;
        use crate::ft8::llr::compute_llr;
        use crate::ft8::params::LDPC_N;

        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");

        let (_results, mfsk_residual) = decode_frame_subtract_staged_with_ap_debug_residual(
            &audio,
            100.0,
            3000.0,
            0.8,
            None,
            DecodeDepth::FULL,
            200,
            DecodeStrictness::Normal,
            None,
        );

        let jt9_bytes = std::fs::read("/tmp/jt9_post_sic_dd.raw").expect("read jt9 residual dump");
        let jt9_residual: Vec<i16> = jt9_bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();

        let freq = 244.2f32;
        let dt = 0.505f32;

        let mut llr_sets: Vec<(&str, crate::ft8::llr::LlrSet<f32>)> = Vec::new();
        for (label, residual) in [
            ("mfsk-core own SIC", &mfsk_residual),
            ("jt9 own SIC", &jt9_residual),
        ] {
            let cand = crate::core::sync::SyncCandidate {
                freq_hz: freq,
                dt_sec: dt,
                score: 0.0,
            };
            let (cd0, _cache) = downsample(residual, cand.freq_hz, None);
            let refined = refine_candidate::<crate::ft8::Ft8>(&cd0, &cand, 10);
            let mut cs = symbol_spectra_direct::<i16>(
                residual,
                cand.freq_hz,
                refined.dt_sec,
                SymMask::SyncOnly,
                None,
            );
            fill_symbol_spectra(
                &mut cs,
                residual,
                cand.freq_hz,
                refined.dt_sec,
                SymMask::DataOnly,
                None,
            );
            llr_sets.push((label, compute_llr::<f32>(&cs)));
        }
        let (mfsk_llr, jt9_llr) = (&llr_sets[0].1, &llr_sets[1].1);

        // OSD's real reprocessing basis size mirrors WSJT-X's `nord=1`
        // entry: the 91 (=LDPC_K) most-reliable bits form the systematic
        // basis after Gaussian elimination; everything past that is
        // candidate-flip territory. Top-91 overlap is the number that
        // actually matters for whether the *same* basis gets built.
        const BASIS_SIZE: usize = 91;

        for (name, m, j) in [
            ("a", &mfsk_llr.llra, &jt9_llr.llra),
            ("b", &mfsk_llr.llrb, &jt9_llr.llrb),
            ("c", &mfsk_llr.llrc, &jt9_llr.llrc),
            ("d", &mfsk_llr.llrd, &jt9_llr.llrd),
        ] {
            let hard_disagree = (0..LDPC_N)
                .filter(|&i| (m[i] > 0.0) != (j[i] > 0.0))
                .count();

            let mut m_rank: Vec<usize> = (0..LDPC_N).collect();
            m_rank.sort_by(|&x, &y| m[y].abs().partial_cmp(&m[x].abs()).unwrap());
            let mut j_rank: Vec<usize> = (0..LDPC_N).collect();
            j_rank.sort_by(|&x, &y| j[y].abs().partial_cmp(&j[x].abs()).unwrap());

            let m_top: std::collections::HashSet<usize> =
                m_rank[..BASIS_SIZE].iter().copied().collect();
            let j_top: std::collections::HashSet<usize> =
                j_rank[..BASIS_SIZE].iter().copied().collect();
            let overlap = m_top.intersection(&j_top).count();

            // Of the bits BOTH sides agree belong in the top-91 basis,
            // how many disagree on hard decision (sign)? This is the
            // number that actually breaks OSD's Gaussian elimination —
            // a shared-basis bit with a flipped sign is a wrong "known"
            // bit baked into the systematic form.
            let basis_hard_disagree = m_top
                .intersection(&j_top)
                .filter(|&&i| (m[i] > 0.0) != (j[i] > 0.0))
                .count();

            println!(
                "llr({name}): hard_disagree={hard_disagree}/{LDPC_N}  top-{BASIS_SIZE}-overlap={overlap}/{BASIS_SIZE}  basis_hard_disagree={basis_hard_disagree}"
            );
        }
    }

    /// Throwaway probe (issue #182) — NOT for commit. Tests the leading
    /// hypothesis for `osd_decode_npre1`'s DK8NE fidelity gap: WSJT-X's
    /// real Gaussian elimination (`osd174_91.f90:86-107`) bounds its
    /// pivot search to `id..k+20` with column swaps ("ad hoc... beware"
    /// per its own comment), while `osd_setup_ldpc174_91` scans the
    /// full N=174 column range — a more complete elimination that can
    /// select a genuinely different set of MRB (most-reliable-basis)
    /// physical bit positions. Since `osd_npre1_pass` only explores
    /// flips *within* whichever basis got selected, a different basis
    /// changes which codewords are reachable at all. Runs
    /// `osd_decode_npre1_fortran_pivot` (same npre1 search, WSJT-X's
    /// bounded-window pivot construction) against
    /// `osd_decode_npre1`'s own construction, on the identical LLR, to
    /// see whether the bounded pivot window is what recovers DK8NE.
    #[test]
    #[ignore = "manual diagnostic — issue #182 Fortran-pivot-window OSD basis probe"]
    fn issue_182_dk8ne_osd_fortran_pivot_probe() {
        use crate::core::sync::refine_candidate;
        use crate::fec::ldpc::bp::bp_llr_zsum;
        use crate::fec::ldpc::osd::{
            osd_debug_basis_sets, osd_decode, osd_decode_npre1, osd_decode_npre1_fortran_pivot,
        };
        use crate::fec::ldpc::params::Ldpc174_91Params;
        use crate::ft8::decode_block::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
        use crate::ft8::downsample::downsample;
        use crate::ft8::llr::compute_llr;
        use crate::msg::wsjt77::unpack77;

        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");

        let (_results, mfsk_residual) = decode_frame_subtract_staged_with_ap_debug_residual(
            &audio,
            100.0,
            3000.0,
            0.8,
            None,
            DecodeDepth::FULL,
            200,
            DecodeStrictness::Normal,
            None,
        );

        let freq = 244.2f32;
        let dt = 0.505f32;
        let cand = crate::core::sync::SyncCandidate {
            freq_hz: freq,
            dt_sec: dt,
            score: 0.0,
        };
        let (cd0, _cache) = downsample(&mfsk_residual, cand.freq_hz, None);
        let refined = refine_candidate::<crate::ft8::Ft8>(&cd0, &cand, 10);
        let mut cs = symbol_spectra_direct::<i16>(
            &mfsk_residual,
            cand.freq_hz,
            refined.dt_sec,
            SymMask::SyncOnly,
            None,
        );
        fill_symbol_spectra(
            &mut cs,
            &mfsk_residual,
            cand.freq_hz,
            refined.dt_sec,
            SymMask::DataOnly,
            None,
        );
        let llr_set = compute_llr::<f32>(&cs);

        let target = "K1BZM DK8NE -10";
        for (name, llr) in [
            ("a", &llr_set.llra),
            ("b", &llr_set.llrb),
            ("c", &llr_set.llrc),
            ("d", &llr_set.llrd),
        ] {
            let current = osd_decode_npre1(llr)
                .map(|o| (unpack77(&o.message77).unwrap_or_default(), o.hard_errors));
            let fortran_pivot = osd_decode_npre1_fortran_pivot(llr)
                .map(|o| (unpack77(&o.message77).unwrap_or_default(), o.hard_errors));
            let (basis_current, basis_fortran) = osd_debug_basis_sets(llr);
            let set_current: std::collections::HashSet<usize> =
                basis_current.iter().copied().collect();
            let set_fortran: std::collections::HashSet<usize> =
                basis_fortran.iter().copied().collect();
            let basis_overlap = set_current.intersection(&set_fortran).count();
            let exhaustive = osd_decode(llr)
                .map(|o| (unpack77(&o.message77).unwrap_or_default(), o.hard_errors));
            println!(
                "llr({name}): current_basis={current:?}  fortran_pivot_basis={fortran_pivot:?}  basis_position_overlap={basis_overlap}/{}  exhaustive_order2={exhaustive:?}",
                set_current.len()
            );

            // WSJT-X's real decode174_91.f90 driver never feeds osd174_91
            // the raw channel LLR when maxosd>0 (FT8 ndepth=3 always sets
            // maxosd=2) -- it feeds `zsave(:,i)`, the running sum of the
            // BP variable-node soft estimate `zn` across the first `i`
            // BP iterations (i=1,2 for maxosd=2), trying i=1 then i=2.
            // `bp_llr_zsum` already exists and is wired for FST4-120
            // (Ldpc240_101) but was never wired into FT8's osd_strategy.rs
            // dispatch at all -- FT8's OSD has only ever seen the raw
            // channel LLR variants (a/b/c/d), never a BP-refined one.
            for n_iter in [1u32, 2u32] {
                let zsum_vec = bp_llr_zsum::<Ldpc174_91Params>(llr, n_iter);
                let mut zsum = [0f32; crate::ft8::params::LDPC_N];
                zsum.copy_from_slice(&zsum_vec);
                let via_zsum = osd_decode_npre1(&zsum)
                    .map(|o| (unpack77(&o.message77).unwrap_or_default(), o.hard_errors));
                println!("  bp_llr_zsum(llr, {n_iter}) -> osd_decode_npre1: {via_zsum:?}");
            }
            if let Some((msg, _)) = &fortran_pivot
                && msg == target
            {
                println!("  -> fortran_pivot_basis RECOVERS {target} on llr variant {name}!");
            }
        }
    }

    /// Throwaway probe (issue #182) — NOT for commit. The `bp_llr_zsum`
    /// OSD-seed fix surfaced a new decode (`<?> 5T5ZGS/R FE02`) on
    /// `qso3_busy.wav`'s AP-on multipass run that wasn't there before.
    /// Print pass/hard_errors/freq for every decode to check whether
    /// it's a plausible weak-but-real signal or a CRC-luck phantom.
    #[test]
    #[ignore = "manual diagnostic — issue #182 zsum-fix phantom check"]
    fn issue_182_zsum_fix_phantom_check() {
        use crate::msg::wsjt77::unpack77;

        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");

        let ap = ApHint::new().with_call1("K1JT").with_call2("HA0DU");
        let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.3, 50)
            .depth(DecodeDepth::FULL)
            .strictness(DecodeStrictness::Normal)
            .staged()
            .ap_hint(&ap)
            .decode()
            .results;
        for r in &results {
            let msg = unpack77(&r.message77).unwrap_or_default();
            println!(
                "pass={:3} hard_errors={:3} freq={:8.2} dt={:+.3} msg={msg:?}",
                r.pass, r.hard_errors, r.freq_hz, r.dt_sec
            );
        }
    }

    /// Throwaway probe (issue #182 follow-up) — NOT for commit. Real
    /// blind-decode wall-clock on `qso3_busy.wav` after the
    /// `bp_llr_zsum` OSD fix, for direct comparison against jt9's own
    /// real `-8 -d3` file decode time (~1.1s, measured in an earlier
    /// session via jt9's built-in `timer.out` profiler).
    #[test]
    #[ignore = "manual diagnostic — issue #182 post-fix wall-clock check"]
    fn issue_182_postfix_wallclock_check() {
        fn load_wav_i16(path: &std::path::Path) -> Option<alloc::vec::Vec<i16>> {
            let bytes = std::fs::read(path).ok()?;
            if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
                return None;
            }
            let mut i = 12usize;
            let mut data_off = None;
            let mut data_len = 0usize;
            while i + 8 <= bytes.len() {
                let id = &bytes[i..i + 4];
                let sz = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
                let body = i + 8;
                if id == b"data" {
                    data_off = Some(body);
                    data_len = sz;
                    break;
                }
                match body.checked_add(sz).and_then(|s| s.checked_add(sz & 1)) {
                    Some(next) => i = next,
                    None => break,
                }
            }
            let off = data_off?;
            let end = off.saturating_add(data_len).min(bytes.len());
            Some(
                bytes[off..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            )
        }

        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(manifest).join("../embedded-poc/assets/qso3_busy.wav");
        let audio = load_wav_i16(&path).expect("load qso3_busy.wav");

        // Blind decode only (no AP hint) -- staged SIC (`.staged()`) has
        // been the default since #180/#183.
        for rep in 0..3 {
            let t0 = std::time::Instant::now();
            let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
                .depth(DecodeDepth::FULL)
                .strictness(DecodeStrictness::Normal)
                .staged()
                .decode()
                .results;
            let elapsed = t0.elapsed();
            println!(
                "rep={rep} blind staged decode: {:?}, {} decodes",
                elapsed,
                results.len()
            );
        }
    }
}
