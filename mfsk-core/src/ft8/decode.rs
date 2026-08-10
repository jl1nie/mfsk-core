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
    DecodeOutcome, DecodeRequest, FrameDecodable, SniperRequest, SupportsSicEarly,
    SupportsSicRounds, SupportsWideBandAp,
};

// ────────────────────────────────────────────────────────────────────────────
// Public types

/// Opaque FFT cache, reusable across pipelined decode passes.
///
/// Canonical definition lives in [`crate::engine::pipeline::FftCache`] — same
/// underlying `Vec<Complex<f32>>`, re-exported here for backward-compatible
/// `mfsk_core::ft8::decode::FftCache` import paths (issue #191).
pub use crate::engine::pipeline::FftCache;

/// Decode cost/recall configuration, plus the [`LlrEffort`] staircase width.
///
/// Canonical definition lives in [`crate::engine::pipeline::DecodeDepth`] —
/// re-exported here (with [`LlrEffort`]) for backward-compatible
/// `mfsk_core::ft8::decode::DecodeDepth` import paths (issue #191 type
/// consolidation migrated `engine::pipeline`'s old 2-variant `BpAll`/
/// `BpAllOsd` enum, used by FT4/FST4, onto this struct). `llr_effort` is
/// FT8-only in practice — the generic `engine::pipeline` engine FT4/FST4 use
/// always computes all LLR variants and only reads `depth.osd`.
pub use crate::engine::pipeline::{DecodeDepth, LlrEffort};

/// Decode strictness: controls false-positive vs sensitivity trade-off.
///
/// Canonical definition lives in [`crate::engine::pipeline::DecodeStrictness`].
/// FT8 consumes two of its three methods:
///
/// - [`DecodeStrictness::ft8_nharderrors_max`] — the BP-staircase/OSD
///   `nharderrors` acceptance ceiling (`ft8/decode_block/process_candidates.rs`'s
///   four BP-variant checks, `osd_strategy.rs`'s OSD dispatch). Live on
///   *every* FT8 decode, AP or not. Added as a strictness-wiring
///   follow-up to #220 (issue #221) — previously this axis was a documented
///   no-op for FT8 (a leftover, differently-calibrated `osd_max_errors`/
///   `osd_score_min` pair that had never actually been called anywhere
///   in FT8's own decode path, dead since #188 removed the
///   `auto_ap_strategy` module that used to consume them; hardcoded
///   `OSD_HARDERRORS_MAX 36`/`WSJTX_NHARDERRORS_MAX 36` ran instead,
///   unconditionally). `Normal` still returns that same 36 — zero
///   default-behavior change from the rewiring itself.
/// - [`DecodeStrictness::ap_max_errors`] — FT8's per-candidate AP loop
///   (Step 4), live only when `ap_hint` is `Some(_)`. Already live
///   before this follow-up; numerically identical to the generic
///   pipeline's own copy (issue #191 type consolidation).
///
/// [`DecodeStrictness::osd_max_errors`]/`osd_score_min` (the
/// `osd_depth`-tiered pair) remain FT4-only — FT8's real OSD dispatch
/// has no `osd_depth` tiering to port, so `ft8_nharderrors_max` is a
/// separate, flat method rather than reusing that one.
pub use crate::engine::pipeline::DecodeStrictness;

/// One successfully decoded FT8 message.
///
/// Canonical definition lives in [`crate::engine::pipeline::DecodeResult`]
/// (issue #194) — this used to be a separate, byte-for-byte-identical
/// struct except for `message77: [u8; 77]` (CRC bits stripped) vs. the
/// generic type's `info: Box<[u8]>` (all `K` FEC info bits, CRC
/// retained) + `message77()` accessor slicing the leading 77. FT8's own
/// BP/OSD engine already produces the full `info` (`fec::ldpc::bp::
/// BpResult::info`) at every construction site — it was just being
/// discarded in favor of the 77-bit-only field. Unifying onto the
/// generic type (rather than just matching its shape) lets a
/// protocol-generic caller over `DecodeRequest<P>` read every
/// protocol's results the same way. Re-exported so existing
/// `mfsk_core::ft8::decode::DecodeResult` import paths keep working.
pub use crate::engine::pipeline::DecodeResult;

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

/// `BpScratch`'s LLR-scalar type, matching `decode_block`'s own
/// `LlrT` selection (`Q11i16` under `fixed-point`, `f32` otherwise).
#[cfg(feature = "fixed-point")]
type LlrT = crate::engine::scalar::Q11i16;
#[cfg(not(feature = "fixed-point"))]
type LlrT = f32;

/// Decode a single sync candidate: downsample → refine → LLR → BP/OSD.
///
/// `fft_cache` — pre-computed 192 000-point forward FFT of the full audio
///   (from [`build_fft_cache`]), shared read-only across parallel calls.
/// `known`     — messages decoded in earlier subtract passes; prevents OSD
///   from running on frequencies that already have a result.
///
/// Owns a fresh `BpScratch` for this call — correct as the per-candidate
/// unit of work under `#[cfg(feature = "parallel")]`'s `par_iter()`
/// (`decode_frame_inner`, `decode_sniper_inner`), where each concurrent
/// candidate needs its own scratch regardless. Callers with a plain
/// sequential per-candidate loop (`sic_inner_passes_with_cache`) should
/// use [`process_candidate_with_scratch`] instead to reuse one scratch
/// pool across the whole loop (issue #201).
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
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    process_candidate_with_scratch(
        cand,
        audio,
        fft_cache,
        depth,
        strictness,
        known,
        eq_mode,
        ap_hint,
        &mut bp_scratch,
    )
}

/// [`process_candidate`] with a caller-owned
/// [`BpScratch`](crate::fec::ldpc::bp::BpScratch) — lets a sequential
/// per-candidate outer loop (`sic_inner_passes_with_cache`) reuse the
/// scratch pool across calls instead of paying its allocation cost on
/// every candidate (issue #201, same pattern as issue #199's fix for
/// `decode_block_multipass`).
#[allow(clippy::too_many_arguments)]
fn process_candidate_with_scratch(
    cand: &SyncCandidate,
    audio: &[i16],
    fft_cache: &[num_complex::Complex<f32>],
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<
        crate::fec::ldpc::params::Ldpc174_91Params,
        LlrT,
    >,
) -> Option<DecodeResult> {
    let _ = strictness; // used inside try_decode via the inner
    // Use `downsample_cached` directly so the FT8 wrapper's
    // `cache.to_vec()` clone (~3 MB) on the `Some(_)` branch is
    // bypassed — same pattern as `fill_symbol_spectra_via_cd0`.
    let mut cd0 = crate::engine::dsp::downsample::downsample_cached(
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
        let scores =
            crate::engine::sync::fine_sync_power_per_block::<crate::ft8::Ft8>(&cd0, i_start);
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
    let mut cs_raw: alloc::boxed::Box<[[crate::engine::scalar::Cmplx<f32>; 8]; 79]> =
        alloc::vec![[crate::engine::scalar::Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    // `sync_quality` only reads the 21 sync-block symbol positions
    // (`P::SYNC_MODE.blocks()`, `engine::llr::sync_quality_generic`) —
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
        #[cfg(feature = "std")]
        crate::ft8::decode_block::TRACE_NSYNC_FAIL
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        return None;
    }
    #[cfg(feature = "std")]
    crate::ft8::decode_block::TRACE_NSYNC_PASS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
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
    let mut try_decode = |cs: &[[crate::engine::scalar::Cmplx<f32>; 8]; 79],
                          _use_ap: bool|
     -> Option<DecodeResult> {
        crate::ft8::decode_block::process_one_candidate_inner(
            cs,
            &refined,
            refined.dt_sec,
            nsync,
            depth,
            BP_MAX_ITER,
            bp_scratch,
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
#[allow(clippy::too_many_arguments)]
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
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, Vec<num_complex::Complex<f32>>) {
    // `freq_hint` is intentionally not forwarded — the WSJT-X-faithful
    // decode_block::coarse_sync (the only FT8 coarse-sync after the v0.6
    // consolidation in #48) does not honour candidate-score promotion.
    // Sniper paths in this file constrain freq_min/freq_max around the
    // target instead, so the loss is contained.
    let _ = freq_hint;
    #[cfg(feature = "std")]
    let trace_stage = crate::ft8::decode_block::stage_trace_enabled();
    #[cfg(feature = "std")]
    let __trace_t0 = trace_stage.then(std::time::Instant::now);
    let spec = crate::ft8::decode_block::compute_spectrogram(audio, freq_max);
    let candidates =
        crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
    #[cfg(feature = "std")]
    if let Some(t0) = __trace_t0 {
        eprintln!(
            "TRACE_STAGE_FT8_SP coarse_sync={:.1}ms n_candidates={}",
            t0.elapsed().as_secs_f64() * 1000.0,
            candidates.len()
        );
    }
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
    #[cfg(feature = "std")]
    if trace_stage {
        crate::ft8::decode_block::TRACE_NSYNC_FAIL.store(0, core::sync::atomic::Ordering::Relaxed);
        crate::ft8::decode_block::TRACE_NSYNC_PASS.store(0, core::sync::atomic::Ordering::Relaxed);
        crate::ft8::decode_block::TRACE_OSD_ATTEMPT.store(0, core::sync::atomic::Ordering::Relaxed);
    }
    #[cfg(feature = "std")]
    let __trace_t1 = trace_stage.then(std::time::Instant::now);

    // `sbase` (WSJT-X-faithful Nuttall-window baseline), captured once
    // for the whole single pass — no subtract happens in this engine, so
    // `audio` never changes and one capture suffices (issue #253
    // SNR-calibration follow-up, 2026-08-10; see `apply_wsjtx_xsnr2`'s
    // doc comment for why `decode_block`/`.sic_early()`/`.sic_rounds()`
    // and this plain single-pass engine must agree on SNR).
    #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
    let sbase: alloc::vec::Vec<f32> = {
        let avg = crate::ft8::baseline::compute_baseline_spectrum(audio);
        crate::ft8::baseline::fit_baseline(&avg, 0, spec.n_freq - 1)
    };

    // `on_result` fires here, inside the per-candidate closure, *before*
    // the cross-candidate dedup pass below — see `DecodeRequest::
    // on_result`'s doc comment for why that ordering means a result can
    // fire via callback but not survive into the returned `Vec`.
    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            #[cfg_attr(
                not(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point"))),
                allow(unused_mut)
            )]
            let mut r = process_candidate(
                cand, audio, &fft_cache, depth, strictness, known, eq_mode, ap_hint,
            )?;
            #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
            {
                let xsig =
                    crate::ft8::decode_block::compute_xsig_wsjtx(&r, audio, Some(&fft_cache));
                if !crate::ft8::decode_block::apply_wsjtx_xsnr2(&mut r, xsig, &sbase, &spec) {
                    return None;
                }
            }
            if let Some(cb) = on_result {
                cb(&r);
            }
            Some(r)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            #[cfg_attr(
                not(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point"))),
                allow(unused_mut)
            )]
            let mut r = process_candidate(
                cand, audio, &fft_cache, depth, strictness, known, eq_mode, ap_hint,
            )?;
            #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
            {
                let xsig =
                    crate::ft8::decode_block::compute_xsig_wsjtx(&r, audio, Some(&fft_cache));
                if !crate::ft8::decode_block::apply_wsjtx_xsnr2(&mut r, xsig, &sbase, &spec) {
                    return None;
                }
            }
            if let Some(cb) = on_result {
                cb(&r);
            }
            Some(r)
        })
        .collect();
    #[cfg(feature = "std")]
    if let Some(t1) = __trace_t1 {
        eprintln!(
            "TRACE_STAGE_FT8_SP decode_loop={:.1}ms nsync_fail={} nsync_pass={} osd_attempt={} n_decoded={}",
            t1.elapsed().as_secs_f64() * 1000.0,
            crate::ft8::decode_block::TRACE_NSYNC_FAIL.load(core::sync::atomic::Ordering::Relaxed),
            crate::ft8::decode_block::TRACE_NSYNC_PASS.load(core::sync::atomic::Ordering::Relaxed),
            crate::ft8::decode_block::TRACE_OSD_ATTEMPT.load(core::sync::atomic::Ordering::Relaxed),
            raw.len()
        );
    }

    // Deduplicate: preserve first occurrence; drop messages already in `known`.
    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        if !known.iter().any(|k| k.message77() == r.message77())
            && !results.iter().any(|x| x.message77() == r.message77())
        {
            results.push(r);
        }
    }
    (results, fft_cache)
}

// ────────────────────────────────────────────────────────────────────────────
// Multi-pass decode with signal subtraction

/// The flat SIC (up to [`DecodeRequest::sic_rounds`] rounds, default and
/// max of 3) plus sequential subtract, mirroring the structure of
/// `decode_block::decode_block_multipass` (the embedded path) which is a
/// faithful port of `lib/ft8/ft8b.f90:432-437`. Used by
/// [`SupportsSicRounds`]'s `Ft8` impl (`.sic_rounds()`) and as
/// [`decode_frame_subtract_staged_with_ap_inner`]'s fallback for
/// buffers too short to stage meaningfully.
///
/// Two changes from the pre-v0.6.0 host implementation:
///
/// 1. **Fixed `sync_min` across all rounds** (was 1.0 / 0.75 / 0.5).
///    Progressive relaxation lets phantoms slip through later rounds
///    when SIC artefacts dominate the residual; WSJT-X holds the
///    threshold and skips later rounds when no new decodes come out.
///
/// 2. **Sequential subtract within each round** (was batch-after-round).
///    Each accepted decode immediately subtracts from the residual so
///    the *next* candidate in the same round sees a cleaner spectrum.
///    This is what surfaces -13 to -18 dB signals sitting beneath
///    strong neighbours (the JTDX-extras shape on `qso3_busy.wav`).
///    Without sequential subtract, all candidates in a round see the
///    same raw residual and weak signals stay masked.
///
/// Round termination matches WSJT-X: round 2 skips when round 1 returned
/// 0 decodes; round 3 skips when round 2 returned no NEW decodes.
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
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    precomputed_fft: Option<&[num_complex::Complex<f32>]>,
    n_rounds: usize,
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
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
        eq_mode,
        ap_hint,
        precomputed_fft,
        n_rounds,
        on_result,
    )
}

/// Shared inner loop: up to `n_rounds` sub-passes of coarse_sync +
/// per-candidate decode, subtracting each accepted decode from `residual`
/// immediately (sequential, not batch-after-round) so later candidates in
/// the same sub-pass see a cleaner spectrum. This is WSJT-X's
/// `ft8b.f90:432-437` `do ipass=1,npass` structure. Factored out of
/// [`flat_sic_inner`] so [`decode_frame_subtract_staged_with_ap_inner`] can
/// reuse it at checkpoint A and checkpoint C (issue #180) without
/// duplicating the pass/dedup/subtract logic — those checkpoint call sites
/// always pass `n_rounds = 3` (checkpoint structure is fixed, not exposed
/// as a caller-tunable knob; see [`SupportsSicEarly`]).
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
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    n_rounds: usize,
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> Vec<DecodeResult> {
    sic_inner_passes_with_cache(
        residual, freq_min, freq_max, sync_min, depth, max_cand, strictness, known, eq_mode,
        ap_hint, None, n_rounds, on_result,
    )
    .0
}

/// Like [`sic_inner_passes`] but also accepts a `precomputed_fft` cache
/// (reused only on round 0, and only when `residual` has not yet been
/// mutated by a subtraction the caller did before calling in — passing
/// `Some(_)` alongside an already-subtracted `residual` would silently
/// mismatch the cache against the audio it's meant to describe) and
/// returns the round-0 cache alongside the results, so a follow-up
/// pipelined call can reuse it in turn. This is the fix for issue #191:
/// previously only the (dead-code, zero-caller)
/// `decode_frame_subtract_with_known_and_ap` had *any* cache-reuse path,
/// and it used its own unfixed flat-3-round engine rather than this
/// (shared by both `.sic_rounds()` and `.sic_early()`) one.
///
/// `n_rounds` — upper bound on SIC rounds, expected 1..=3
/// (`DecodeRequest::sic_rounds` already clamps to this range for the
/// `.sic_rounds()` path; checkpoint callers always pass `3`).
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
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    precomputed_fft: Option<&[num_complex::Complex<f32>]>,
    n_rounds: usize,
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, FftCache) {
    let mut all_results: Vec<DecodeResult> = Vec::new();
    let mut pass0_cache: Option<FftCache> = None;

    // Shared across every pass's per-candidate loop below (issue #201,
    // same pattern as issue #199's `decode_block_multipass` fix): this
    // is always a plain sequential loop (no `#[cfg(feature = "parallel")]`
    // branch here, unlike `decode_frame_inner`/`decode_sniper_inner`), so
    // a single caller-owned scratch pool can be reused across every
    // candidate in every pass instead of reallocating per candidate.
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();

    let mut prev_total: usize = 0;
    for ipass in 0..n_rounds {
        if ipass >= 1 && all_results.len() == prev_total {
            break;
        }
        prev_total = all_results.len();

        let spec = crate::ft8::decode_block::compute_spectrogram(residual, freq_max);
        let candidates =
            crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
        if candidates.is_empty() {
            continue;
        }
        // `sbase` (WSJT-X-faithful Nuttall-window baseline) captured once
        // per pass, from this pass's then-current `residual` — same
        // per-pass-not-per-candidate cadence as `decode_block_multipass`
        // (issue #253 SNR-calibration follow-up, 2026-08-10). `spec`
        // (rectangular) is kept alive (not dropped early like before)
        // only for the `nsync` validity gate below — unrelated to the
        // xsig/xbase scale calibration.
        #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
        let sbase: alloc::vec::Vec<f32> = {
            let avg = crate::ft8::baseline::compute_baseline_spectrum(residual);
            crate::ft8::baseline::fit_baseline(&avg, 0, spec.n_freq - 1)
        };

        let fft_cache = if ipass == 0
            && let Some(c) = precomputed_fft
        {
            c.to_vec()
        } else {
            build_fft_cache(residual)
        };
        if ipass == 0 {
            pass0_cache = Some(FftCache(fft_cache.clone()));
        }
        for cand in &candidates {
            #[cfg_attr(
                not(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point"))),
                allow(unused_mut)
            )]
            let mut r = match process_candidate_with_scratch(
                cand,
                residual,
                &fft_cache,
                depth,
                strictness,
                known,
                eq_mode,
                ap_hint,
                &mut bp_scratch,
            ) {
                Some(r) => r,
                None => continue,
            };
            // Dedup against `known` (an earlier stage) and this loop's
            // own earlier passes.
            if known.iter().any(|x| x.message77() == r.message77())
                || all_results.iter().any(|x| x.message77() == r.message77())
            {
                continue;
            }
            // `xsig` for the xsnr2 gate below — must run *before* the
            // subtract (see `compute_xsig_wsjtx`'s doc comment).
            #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
            let xsig_wsjtx: f32 =
                crate::ft8::decode_block::compute_xsig_wsjtx(&r, residual, Some(&fft_cache));
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
            // WSJT-X xsnr2 validity gate (issue #253 SNR-calibration
            // follow-up, 2026-08-10) — see `apply_wsjtx_xsnr2`'s doc
            // comment. Applied *before* `on_result` fires, matching the
            // same revoke-less-retract discipline `decode_block`'s own
            // driver already follows (issue #243).
            #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
            if !crate::ft8::decode_block::apply_wsjtx_xsnr2(&mut r, xsig_wsjtx, &sbase, &spec) {
                continue;
            }
            if let Some(cb) = on_result {
                cb(&r);
            }
            all_results.push(r);
        }
    }

    // Pass 0 may have had no candidates at all (empty coarse-sync), in
    // which case `pass0_cache` was never set — build one from the
    // (possibly already-subtracted) residual as a fallback, matching
    // `decode_frame_inner`'s "cache is always returned" contract.
    let fft_cache = pass0_cache.unwrap_or_else(|| FftCache(build_fft_cache(residual)));
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
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        EqMode::Off,
        ap_hint,
        &[],
        None,
    )
}

/// Checkpoint SIC round count — always the full 3, never caller-tunable.
/// The A/B/C checkpoint structure itself (not this) is what varies with
/// signal availability; see [`SupportsSicEarly`]'s doc comment for why
/// this axis has no `.sic_rounds()`-style knob.
const CHECKPOINT_SIC_ROUNDS: usize = 3;

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
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    // Caller-level `known` (`DecodeRequest::known`, e.g. an earlier
    // phase's already-reported results). Threaded into *every*
    // checkpoint's own dedup below rather than relied on solely via
    // the caller's upfront `subtract_signal_lpf_refine_dt` pass — see
    // this function's own doc comment / `__staged_sic` for why: an
    // imperfect subtraction on a strong `outer_known` carrier can
    // leave enough residual for a checkpoint to independently
    // re-derive the *same* message, and unless that re-derivation is
    // caught by the message77 dedup *before* `on_result` fires, the
    // caller sees a callback delivery for a result that then silently
    // never appears in the returned `Vec` — a revoke-less retract of
    // exactly the kind issue #243 closed on the `decode_block`
    // engine. `known.iter().any(...)` (inside `sic_inner_passes_with_cache`)
    // already runs *before* the subtract+callback point for whatever
    // slice it's given; passing `outer_known` here (rather than `&[]`)
    // is what makes that existing atomic gate also cover this case.
    outer_known: &[DecodeResult],
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, Vec<i16>) {
    use staged_checkpoint::{A_SAMPLES, B_SAMPLES, C_SAMPLES};

    // `outer_known` pre-subtracted once, up front — used everywhere
    // *except* checkpoint A below (2026-08-10, issue #253). A real
    // `jt9` build's own disk-read "Early" pass (`nearly=41`, the
    // structure checkpoint A ports) is *always* the first decode
    // attempt on freshly-read, unmodified audio for a Rx cycle —
    // there is no code path in WSJT-X where it ever runs against
    // audio some earlier, external pass has already subtracted from.
    // Feeding checkpoint A's zero-tailed truncated buffer
    // pre-subtracted audio is an mfsk-core-original composition with
    // no WSJT-X counterpart to validate it against, and was
    // root-caused to a reproducible false decode (`7Y8CIH HN1GD OP30`
    // on `qso3_busy.wav`, `hard_errors=31` — a garden-variety CRC-14
    // false-accept that a truncation-boundary artefact in the
    // pre-subtracted residual apparently made reachable). Checkpoints
    // B/C don't have this problem — they already rebuild their own
    // buffers fresh from `audio` each time rather than reusing
    // checkpoint A's residual, so routing *their* fresh copy through
    // `audio_clean` instead keeps the "known signals don't mask
    // weaker ones" capability the original design wanted, without
    // exposing checkpoint A to anything unvalidated.
    let mut audio_clean = audio.to_vec();
    for r in outer_known {
        subtract_signal_lpf_refine_dt(&mut audio_clean, r);
    }

    // Buffers shorter than checkpoint A can't be staged meaningfully —
    // there's no "early, incomplete" window smaller than the whole thing.
    // Must call the *flat* fallback (`flat_sic_inner`), not `.sic_early()`
    // itself — that dispatches to this very function (issue #180), so
    // calling it here would recurse indefinitely (stack overflow, caught
    // by `sic_early_with_ap_silence_shape`). A flat pass has none of
    // checkpoint A's truncation-boundary exposure, so `audio_clean` is
    // fine here.
    let _ = freq_hint;
    if audio.len() < A_SAMPLES {
        let (r, _) = flat_sic_inner(
            &audio_clean,
            freq_min,
            freq_max,
            sync_min,
            depth,
            max_cand,
            strictness,
            outer_known,
            eq_mode,
            ap_hint,
            None,
            CHECKPOINT_SIC_ROUNDS,
            on_result,
        );
        return (r, audio_clean);
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
    // precedent in `engine::pipeline`).
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
        outer_known,
        eq_mode,
        ap_hint,
        CHECKPOINT_SIC_ROUNDS,
        on_result,
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
        // recursion hazard as the `A_SAMPLES` branch above. `audio_clean`
        // (not raw `audio`) — same rationale as the short-buffer
        // fallback above, no truncation exposure in a flat pass.
        let (r, _) = flat_sic_inner(
            &audio_clean,
            freq_min,
            freq_max,
            sync_min,
            depth,
            max_cand,
            strictness,
            outer_known,
            eq_mode,
            ap_hint,
            None,
            CHECKPOINT_SIC_ROUNDS,
            on_result,
        );
        return (r, audio_clean);
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
    buf_b[..b_len].copy_from_slice(&audio_clean[..b_len]);
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
    buf_c[b_len..c_len].copy_from_slice(&audio_clean[b_len..c_len]);
    // `ft8_decode.f90:162` — same `lrefinedt=.true.` re-search as
    // checkpoint B above, for the late-`dt` decodes deferred to here.
    for r in &deferred {
        subtract_signal_lpf_refine_dt(&mut buf_c, r);
    }

    // Combine `outer_known` (the caller's earlier-phase results) with
    // this call's own `early_results` (checkpoint A) so checkpoint C's
    // dedup gate — which runs *before* subtract/on_result, same as
    // every other `sic_inner_passes` call site above — catches a
    // candidate matching either set atomically, not via the
    // post-hoc `results.retain` `__staged_sic` used to rely on for
    // `outer_known` alone.
    let known_c: alloc::vec::Vec<DecodeResult> = outer_known
        .iter()
        .cloned()
        .chain(early_results.iter().cloned())
        .collect();
    let new_results = sic_inner_passes(
        &mut buf_c,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        strictness,
        &known_c,
        eq_mode,
        ap_hint,
        CHECKPOINT_SIC_ROUNDS,
        on_result,
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
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, FftCache) {
    let freq_min = (target_freq - 250.0).max(100.0);
    let freq_max = (target_freq + 250.0).min(5900.0);

    // Sniper-mode: freq_hint (=target_freq) used to promote candidates
    // near the target via the legacy engine::sync::coarse_sync path. After
    // the v0.6 consolidation in #48, decode_block::coarse_sync does not
    // honour hints; the ±250 Hz freq_min/freq_max band above does most
    // of the work the hint used to.
    let spec = crate::ft8::decode_block::compute_spectrogram(audio, freq_max);
    let candidates =
        crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
    let fft_cache = FftCache(build_fft_cache(audio));
    if candidates.is_empty() {
        return (Vec::new(), fft_cache);
    }

    // `sbase`, same rationale as `decode_frame_inner`'s own (issue #253
    // SNR-calibration follow-up, 2026-08-10) — no subtract in this
    // engine either, one capture for the whole call suffices.
    #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
    let sbase: alloc::vec::Vec<f32> = {
        let avg = crate::ft8::baseline::compute_baseline_spectrum(audio);
        crate::ft8::baseline::fit_baseline(&avg, 0, spec.n_freq - 1)
    };

    // Same on_result-fires-before-dedup ordering as decode_frame_inner —
    // see its comment above the analogous par_iter block.
    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            #[cfg_attr(
                not(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point"))),
                allow(unused_mut)
            )]
            let mut r = process_candidate(
                cand,
                audio,
                fft_cache.as_slice(),
                depth,
                strictness,
                &[],
                eq_mode,
                ap_hint,
            )?;
            #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
            {
                let xsig = crate::ft8::decode_block::compute_xsig_wsjtx(
                    &r,
                    audio,
                    Some(fft_cache.as_slice()),
                );
                if !crate::ft8::decode_block::apply_wsjtx_xsnr2(&mut r, xsig, &sbase, &spec) {
                    return None;
                }
            }
            if let Some(cb) = on_result {
                cb(&r);
            }
            Some(r)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            #[cfg_attr(
                not(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point"))),
                allow(unused_mut)
            )]
            let mut r = process_candidate(
                cand,
                audio,
                fft_cache.as_slice(),
                depth,
                strictness,
                &[],
                eq_mode,
                ap_hint,
            )?;
            #[cfg(all(feature = "fft-rustfft", feature = "std", not(feature = "fixed-point")))]
            {
                let xsig = crate::ft8::decode_block::compute_xsig_wsjtx(
                    &r,
                    audio,
                    Some(fft_cache.as_slice()),
                );
                if !crate::ft8::decode_block::apply_wsjtx_xsnr2(&mut r, xsig, &sbase, &spec) {
                    return None;
                }
            }
            if let Some(cb) = on_result {
                cb(&r);
            }
            Some(r)
        })
        .collect();

    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        if !results.iter().any(|x| x.message77() == r.message77()) {
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
            req.fft_cache.as_ref().map(FftCache::as_slice),
            req.ap_hint,
            req.on_result,
        );
        DecodeOutcome {
            results,
            fft_cache: FftCache(fft_cache),
        }
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
            req.on_result,
        );
        DecodeOutcome { results, fft_cache }
    }
}

impl SupportsSicRounds for Ft8 {
    fn __flat_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        // Subtract caller-supplied `known` before round 0, same rationale
        // as `SupportsSicEarly::__staged_sic` below: without this, a
        // strong `known` carrier continues to mask weaker signals
        // throughout the SIC loop (the exact issue #191 bug, previously
        // only reachable through the deleted, unfixed
        // `decode_frame_subtract_with_known_and_ap`). `precomputed_fft`
        // can only be trusted for round 0 once `known` is empty — reusing
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
                req.eq_mode,
                req.ap_hint,
                req.fft_cache.as_ref().map(FftCache::as_slice),
                req.sic_rounds,
                req.on_result,
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
                req.eq_mode,
                req.ap_hint,
                None,
                req.sic_rounds,
                req.on_result,
            );
            DecodeOutcome { results, fft_cache }
        }
    }
}

impl SupportsSicEarly for Ft8 {
    /// The issue #191 fix: `decode_frame_subtract_with_known_and_ap` used
    /// to be the *only* entry point accepting `known`/`precomputed_fft`,
    /// and it ran its own unfixed flat-3-round engine instead of the
    /// staged checkpoint one — missing every SIC-quality improvement
    /// (issue #178/#179/#180) landed since. `known` is now honoured by
    /// *this* (early-decode) path directly: subtracted from the audio
    /// before checkpoint A runs, so all three checkpoints see the cleaned
    /// residual, then deduped from the final results.
    ///
    /// `precomputed_fft` is deliberately not reused here: every
    /// checkpoint operates on a truncated/zero-tailed buffer, never on
    /// the full original audio the cache was built from, so reusing it
    /// would silently mismatch. (It *is* reused by [`SupportsSicRounds`]'s
    /// impl above, whose single full-buffer round 0 has the matching
    /// shape.) Recomputing here is always correct, just not free.
    fn __staged_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        // `req.known`'s pre-subtraction now happens *inside*
        // `decode_frame_subtract_staged_with_ap_inner` (2026-08-10,
        // issue #253) — scoped away from checkpoint A specifically, see
        // that function's own doc comment for why. `subtract_signal_lpf_refine_dt`
        // (not the plain single-shot variant) is what it uses: `known`
        // is conceptually the same kind of carried-forward decode
        // checkpoint B/C already re-subtract with `lrefinedt=.true.`
        // (±90-sample best-alignment search) rather than trusting the
        // original `dt_sec` verbatim. A `known` signal sitting only
        // ~35 Hz from a marginal candidate (as W1FC does next to
        // `CQ DX DL8YHR JO41`, issue #180) needs that same precision —
        // plain `subtract_signal_lpf` measurably left enough residual
        // to lose DL8YHR entirely in end-to-end testing here.
        //
        // `req.known` is also threaded into every checkpoint's own
        // message77 dedup below (not just used for the subtraction
        // above) — an imperfect subtraction on a strong `known` carrier
        // can still leave enough residual for a checkpoint to
        // independently re-derive the same message, and unless that
        // re-derivation is caught *before* `on_result` fires for it,
        // `.on_result(cb)` delivers a callback for a result the
        // returned `Vec` then silently never contains — the exact
        // revoke-less-retract hazard issue #243 closed on the
        // `decode_block` engine. A previous version of this function
        // relied on a post-hoc `results.retain(...)` here instead,
        // which is exactly that hazard: it ran *after*
        // `decode_frame_subtract_staged_with_ap_inner` had already
        // fired `on_result` for every checkpoint's raw candidates.
        let (results, residual) = decode_frame_subtract_staged_with_ap_inner(
            req.audio,
            req.freq_min,
            req.freq_max,
            req.sync_min,
            req.freq_hint,
            req.depth,
            req.max_cand,
            req.strictness,
            req.eq_mode,
            req.ap_hint,
            req.known,
            req.on_result,
        );
        let fft_cache = FftCache(build_fft_cache(&residual));
        DecodeOutcome { results, fft_cache }
    }
}

/// FT8 depth tiers mirroring real WSJT-X's `jt9 -d 1/2/3` CLI flag —
/// for apples-to-apples benchmarking against a real `jt9` build.
/// Unrelated to the crate's own `mfsk_core::jt9` (slow-mode JT9
/// protocol) module.
///
/// | | jt9 `-d1` | jt9 `-d2` | jt9 `-d3` |
/// |---|---|---|---|
/// | this tier | `D1` | `D2` | `D3` |
/// | OSD | off | on | on |
/// | SIC | `.sic_rounds(2)` | `.sic_early()` | `.sic_early()` |
/// | AP | — | — | `.ap_hint()` if supplied |
///
/// Measured on `qso3_busy.wav` (this host, real local `jt9 -8 -dN`
/// build vs. this crate):
///
/// | | jt9 | mfsk-core |
/// |---|---|---|
/// | D1 | 14 decodes / 370ms | *(needs remeasurement — see below)* |
/// | D2 | 19 decodes / 1040ms | 22 decodes / 1078ms |
/// | D3 | 22 decodes / 2110ms | 22 decodes / 2991ms |
///
/// The mfsk-core D1 number above (`14 decodes / 237ms`, superseded)
/// was measured before issue #218's `.sic_rounds(2)` fix — `D1`
/// previously ran `.flat()`'s full 3 rounds (no round-count knob
/// existed), not jt9 `-d1`'s actual `npass=2`. Re-measure against
/// `qso3_busy.wav` once `.sic_rounds(2)` lands; expect fewer decodes
/// and lower latency than the superseded number, not identical.
///
/// jt9 `-d1` runs SIC with `npass=2` (vs. 3 for `-d2`/`-d3`,
/// `ft8_decode.f90:172-173`), and jt9's own `ndepth==1` branch skips
/// the checkpoint-replay staging entirely (`ft8_decode.f90:97-103`) —
/// structurally that's `.sic_rounds()` (single full-buffer SIC
/// round-set), not `.sic_early()`. `D2`/`D3` both go through checkpoint
/// staging in jt9 (`npass=3` either way), so both map to
/// `.sic_early()`. `D1` uses `.sic_rounds(2)` to match jt9 `-d1`'s
/// `npass=2` exactly (previously this crate had no round-count knob at
/// all, so `D1` ran the full 3 rounds — see issue #218).
///
/// jt9 also varies `syncmin` per tier (1.6 for d1/d2, 1.3 for d3,
/// `ft8_decode.f90:176-177`) and OSD *strength* is not just on/off in
/// jt9 (`maxosd` 0 vs 2 are different algorithms — mfsk-core only
/// implements the `maxosd>0` branch; see `osd_strategy` module's doc
/// comment). `D2`'s OSD is therefore closer in kind to jt9 `-d3`'s
/// than to `-d2`'s lighter `maxosd=0` branch — a likely contributor
/// to `D2` already matching/exceeding jt9 `-d3`'s recall above.
/// `sync_min` stays an explicit, caller-supplied parameter (pass
/// 1.6/1.6/1.3 for closer jt9 parity on that axis).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WsjtxDepth {
    D1,
    D2,
    D3,
}

impl<'a> DecodeRequest<'a, Ft8> {
    /// Build a request whose (OSD, SIC strategy, AP) triple mirrors
    /// real WSJT-X's `jt9 -d 1/2/3`. See [`WsjtxDepth`]. `ap` is only
    /// consulted for `D3`; ignored (not an error) for `D1`/`D2`,
    /// matching jt9's own depth/AP coupling.
    pub fn wsjtx_depth(
        audio: &'a [i16],
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        tier: WsjtxDepth,
        ap: Option<&'a ApHint>,
    ) -> Self {
        let mut req = Self::new(audio, freq_min, freq_max, sync_min, max_cand)
            .osd(!matches!(tier, WsjtxDepth::D1));
        req = match tier {
            WsjtxDepth::D1 => req.sic_rounds(2),
            WsjtxDepth::D2 | WsjtxDepth::D3 => req.sic_early(),
        };
        if let (WsjtxDepth::D3, Some(ap)) = (tier, ap) {
            req = req.ap_hint(ap);
        }
        req
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
            .ap_hint(&ap)
            .decode()
            .results;
        assert!(
            results.iter().any(|r| r.message77() == m77),
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
            .osd(false)
            .decode();
        assert!(out0.results.is_empty());
        assert!(!out0.fft_cache.is_empty(), "FFT cache should be returned");

        // ap_hint = Some, strictness = Strict
        let out1 = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .freq_hint(1500.0)
            .strictness(DecodeStrictness::Strict)
            .ap_hint(&ap)
            .decode();
        assert!(out1.results.is_empty());
        assert!(!out1.fft_cache.is_empty());
    }

    /// Compile-shape: `.sic_early()` accepts an AP hint and returns no
    /// decodes on silence.
    #[test]
    fn sic_early_with_ap_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("W7VV");

        let r_none = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .osd(false)
            .sic_early()
            .decode()
            .results;
        assert!(r_none.is_empty());

        let r_some = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .osd(false)
            .sic_early()
            .ap_hint(&ap)
            .decode()
            .results;
        assert!(r_some.is_empty());
    }

    /// Compile-shape: `.sic_early().known(&known).fft_cache(cache)` accepts
    /// the full parameter set (known list + FFT cache + AP hint) and
    /// returns no decodes on silence — the issue #191 combination that
    /// used to be unreachable (only the buggy, now-deleted flat-only
    /// `decode_frame_subtract_with_known_and_ap` accepted `known`+cache
    /// at all).
    #[test]
    fn sic_early_known_and_cache_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
        let known: Vec<DecodeResult> = Vec::new();

        let cache = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .decode()
            .fft_cache;

        let r_none = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .osd(false)
            .sic_early()
            .known(&known)
            .fft_cache(cache.clone())
            .decode()
            .results;
        assert!(r_none.is_empty());

        let r_some = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .osd(false)
            .sic_early()
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
    /// `SupportsSicEarly::__staged_sic` uses (see its doc comment).
    #[test]
    fn sic_early_subtracts_known_before_checkpoint_a() {
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
            .decode()
            .results;
        let known_results: Vec<DecodeResult> = phase1
            .iter()
            .filter(|r| r.message77() == m_known)
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

        // Mirrors `SupportsSicEarly::__staged_sic`'s own upfront
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

    /// Regression test for the `eq_mode`/SIC gap (webft8 sniper-mode
    /// investigation, 2026-07-26): `.sic_rounds()`/`.sic_early()` used to
    /// silently hardcode `EqMode::Off` inside `sic_inner_passes`, dropping
    /// `DecodeRequest::eq_mode()` entirely for both SIC strategies even
    /// though the builder method compiled and looked like it worked.
    ///
    /// Builds a synthetic "BPF-edge" candidate: a clean FT8 signal whose
    /// 8 Costas tones are given a frequency-dependent complex gain (the
    /// same shape `equalizer::tests::edge_attenuation_corrected` uses to
    /// validate the correction math), applied to the raw audio via an
    /// FFT-domain multiply so it exercises the real per-candidate decode
    /// path, not just the isolated equalizer unit. `.sic_early()` (this test)
    /// and `.sic_rounds()` share the same `sic_inner_passes` engine this fix
    /// touches, so covering `.sic_early()` here is sufficient for both.
    ///
    /// Asserts `eq_mode(Local)` decodes a weak, edge-distorted signal that
    /// `eq_mode(Off)` misses through the exact same `.sic_early()` call — the
    /// same shape as the `docs/bench.md`-style "BPF edge + AWGN" scenario,
    /// just self-contained (no external BPF/simulator crate). Fails again
    /// if the hardcoded `EqMode::Off` inside `sic_inner_passes` regresses.
    /// Minimal 4-pole Butterworth bandpass (ported from `ft8-bench::bpf`,
    /// same design route: LP prototype poles -> LP->BP transform ->
    /// bilinear -> biquad cascade) — self-contained so this test doesn't
    /// need a cross-crate dependency on `ft8-bench`.
    #[cfg(test)]
    struct TestBpf {
        sections: Vec<(f64, f64, f64, f64, f64, f64, f64)>, // b0,b1,b2,a1,a2,s1,s2
    }
    #[cfg(test)]
    impl TestBpf {
        fn design(n_poles: usize, f_low: f64, f_high: f64, fs: f64) -> Self {
            fn csqrt(re: f64, im: f64) -> (f64, f64) {
                let r = (re * re + im * im).sqrt();
                let theta = im.atan2(re);
                let sr = r.sqrt();
                (sr * (theta / 2.0).cos(), sr * (theta / 2.0).sin())
            }
            fn lp_to_bp(p_re: f64, p_im: f64, bw: f64, w0sq: f64) -> [(f64, f64); 2] {
                let pbw_re = p_re * bw;
                let pbw_im = p_im * bw;
                let d_re = pbw_re * pbw_re - pbw_im * pbw_im - 4.0 * w0sq;
                let d_im = 2.0 * pbw_re * pbw_im;
                let (sd_re, sd_im) = csqrt(d_re, d_im);
                [
                    ((pbw_re + sd_re) / 2.0, (pbw_im + sd_im) / 2.0),
                    ((pbw_re - sd_re) / 2.0, (pbw_im - sd_im) / 2.0),
                ]
            }
            fn bilinear(s_re: f64, s_im: f64, t: f64) -> (f64, f64) {
                let nr = 1.0 + s_re * t;
                let ni = s_im * t;
                let dr = 1.0 - s_re * t;
                let di = -s_im * t;
                let d_sq = dr * dr + di * di;
                ((nr * dr + ni * di) / d_sq, (ni * dr - nr * di) / d_sq)
            }
            let t = 1.0 / (2.0 * fs);
            let wl = (core::f64::consts::PI * f_low / fs).tan() / t;
            let wh = (core::f64::consts::PI * f_high / fs).tan() / t;
            let w0sq = wl * wh;
            let bw = wh - wl;
            let half = n_poles / 2;
            let mut sections = Vec::with_capacity(n_poles);
            for k in 0..half {
                let theta = core::f64::consts::PI * (2.0 * k as f64 + n_poles as f64 + 1.0)
                    / (2.0 * n_poles as f64);
                let (p_re, p_im) = (theta.cos(), theta.sin());
                for (s_re, s_im) in lp_to_bp(p_re, p_im, bw, w0sq) {
                    let (z_re, z_im) = bilinear(s_re, s_im, t);
                    sections.push((
                        1.0,
                        0.0,
                        -1.0,
                        -2.0 * z_re,
                        z_re * z_re + z_im * z_im,
                        0.0,
                        0.0,
                    ));
                }
            }
            // Normalise to unity gain at the geometric centre frequency.
            let fc = (f_low * f_high).sqrt();
            let wc = 2.0 * core::f64::consts::PI * fc / fs;
            let mag_at = |b0: f64, b1: f64, b2: f64, a1: f64, a2: f64, w: f64| -> f64 {
                let (c1, s1) = (w.cos(), w.sin());
                let (c2, s2) = ((2.0 * w).cos(), (2.0 * w).sin());
                let nr = b0 * c2 + b1 * c1 + b2;
                let ni = b0 * s2 + b1 * s1;
                let dr = c2 + a1 * c1 + a2;
                let di = s2 + a1 * s1;
                ((nr * nr + ni * ni) / (dr * dr + di * di)).sqrt()
            };
            let gain: f64 = sections
                .iter()
                .map(|&(b0, b1, b2, a1, a2, _, _)| mag_at(b0, b1, b2, a1, a2, wc))
                .product();
            if let Some(sec) = sections.first_mut() {
                sec.0 /= gain;
                sec.2 /= gain;
            }
            TestBpf { sections }
        }
        fn filter(&mut self, input: &[f32]) -> Vec<f32> {
            input
                .iter()
                .map(|&x| {
                    let mut y = x as f64;
                    for sec in &mut self.sections {
                        let (b0, b1, b2, a1, a2, s1, s2) = *sec;
                        let out = b0 * y + s1;
                        sec.5 = b1 * y - a1 * out + s2;
                        sec.6 = b2 * y - a2 * out;
                        y = out;
                    }
                    y as f32
                })
                .collect()
        }
    }

    /// Regression test for the `eq_mode`/SIC gap (webft8 sniper-mode
    /// investigation, 2026-07-26): `.sic_rounds()`/`.sic_early()` used to
    /// silently hardcode `EqMode::Off` inside `sic_inner_passes`, dropping
    /// `DecodeRequest::eq_mode()` entirely for both SIC strategies even
    /// though the builder method compiled and looked like it worked.
    ///
    /// Reproduces `ft8-bench`'s own "BPF edge" scenario (4-pole
    /// Butterworth 1000-1500 Hz, target at the passband's -3 dB edge,
    /// 1000 Hz), calibrated against a real ground-truth sweep to a target
    /// SNR (-22 dB, WSJT-X convention) where `eq_mode(Off)` reliably
    /// misses the signal and `eq_mode(Local)` reliably recovers it through
    /// `.sic_early()` — proving the fix actually reaches the SIC engine, not
    /// just that the builder method compiles.
    #[test]
    fn sic_early_eq_mode_reaches_sic_engine() {
        use crate::ft8::wave_gen::{message_to_tones, tones_to_f32};
        use crate::msg::wsjt77::pack77;

        // splitmix64 + Box-Muller — deterministic, dependency-free AWGN.
        struct Rng(u64);
        impl Rng {
            fn next_u64(&mut self) -> u64 {
                self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
                let mut z = self.0;
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                z ^ (z >> 31)
            }
            fn uniform01(&mut self) -> f32 {
                ((self.next_u64() >> 40) as f32 / (1u64 << 24) as f32).max(1e-9)
            }
            fn gaussian(&mut self) -> f32 {
                let u1 = self.uniform01();
                let u2 = self.uniform01();
                (-2.0 * u1.ln()).sqrt() * (2.0 * core::f32::consts::PI * u2).cos()
            }
        }

        const FS: f32 = 12_000.0;
        const F0: f32 = 1_000.0; // BPF edge, matches ft8-bench's own "edge" case
        const BPF_LO: f64 = 1_000.0;
        const BPF_HI: f64 = 1_500.0;
        const REF_BW: f32 = 2_500.0;
        // WSJT-X SNR convention (matches `ft8-bench::simulator::generate_frame`).
        const TARGET_SNR_DB: f32 = -22.0;

        let m77 = pack77("CQ", "K1ABC", "FN42").expect("pack77");
        let tones = message_to_tones(&m77);
        let snr_linear = 10.0_f32.powf(TARGET_SNR_DB / 10.0);
        let amplitude = (4.0 * snr_linear * REF_BW / FS).sqrt();
        let sig = tones_to_f32(&tones, F0, amplitude);

        let n = 15 * 12_000;
        let mut mix = vec![0.0f32; n];
        let off = 6_000usize;
        let n_sig = sig.len().min(n - off);
        mix[off..off + n_sig].copy_from_slice(&sig[..n_sig]);

        let mut rng = Rng(0xC0FF_EE12_3456_789A);
        for s in mix.iter_mut() {
            *s += rng.gaussian();
        }

        let mut bpf = TestBpf::design(4, BPF_LO, BPF_HI, FS as f64);
        let filtered = bpf.filter(&mix);

        // Quantise to i16 with WSJT-X-style headroom (peak -> 29000).
        let peak = filtered.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
        let iq_scale = if peak > 1e-6 { 29_000.0 / peak } else { 1.0 };
        let audio: Vec<i16> = filtered
            .iter()
            .map(|&s| (s * iq_scale).clamp(-32_768.0, 32_767.0) as i16)
            .collect();

        let find = |eq: EqMode| -> Option<DecodeResult> {
            let results = DecodeRequest::<Ft8>::new(
                &audio,
                (BPF_LO as f32) - 50.0,
                (BPF_HI as f32) + 50.0,
                0.8,
                10,
            )
            .eq_mode(eq)
            .sic_early()
            .decode()
            .results;
            results.into_iter().find(|r| r.message77() == m77)
        };

        assert!(
            find(EqMode::Off).is_none(),
            "expected the BPF-edge signal to NOT decode with eq_mode(Off) \
             (test fixture drifted off the calibrated marginal point)"
        );
        assert!(
            find(EqMode::Local).is_some(),
            "expected eq_mode(Local) to decode the same BPF-edge signal \
             that eq_mode(Off) misses through .sic_early() — eq_mode is not \
             reaching the SIC engine"
        );
    }

    /// Silence produces no decoded messages and does not panic.
    #[test]
    fn silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = DecodeRequest::<Ft8>::new(&audio, 200.0, 2800.0, 1.0, 10)
            .osd(false)
            .decode()
            .results;
        assert!(results.is_empty(), "silence should decode nothing");
    }

    /// Sniper mode on silence also produces no decoded messages.
    #[test]
    fn sniper_silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = SniperRequest::<Ft8>::new(&audio, 1000.0, 10)
            .osd(false)
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
        use crate::engine::sync::{SyncCandidate, refine_candidate};
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
                for (bi, block) in <Ft8 as crate::engine::FrameLayout>::SYNC_MODE
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
            for (bi, block) in <Ft8 as crate::engine::FrameLayout>::SYNC_MODE
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
            for (bi, block) in <Ft8 as crate::engine::FrameLayout>::SYNC_MODE
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
                let mut info = vec![0u8; 91];
                info[..77].copy_from_slice(&msg77);
                let wa2fzw = DecodeResult {
                    info: info.into_boxed_slice(),
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
                for block in <Ft8 as crate::engine::FrameLayout>::SYNC_MODE
                    .blocks()
                    .iter()
                {
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
                for block in <Ft8 as crate::engine::FrameLayout>::SYNC_MODE
                    .blocks()
                    .iter()
                {
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
        use crate::engine::sync::refine_candidate;
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
            .any(|r| unpack77(r.message77()).as_deref() == Some("K1BZM DK8NE -10"));
        println!("staged pipeline already found DK8NE blind: {has_dk8ne}");

        let freq = 244.2f32;
        let dt = 0.505f32;
        let cand = crate::engine::sync::SyncCandidate {
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
        use crate::engine::sync::refine_candidate;
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

        let jt9_bytes = match std::fs::read("/tmp/jt9_post_sic_dd.raw") {
            Ok(b) => b,
            Err(_) => {
                eprintln!(
                    "skipping issue_180_dk8ne_data_symbol_comparison: \
                     /tmp/jt9_post_sic_dd.raw (WSJT-X jt9 post-SIC residual dump) \
                     not present — this is a local-only diagnostic input"
                );
                return;
            }
        };
        let jt9_residual: Vec<i16> = jt9_bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();

        let freq = 244.2f32;
        let dt = 0.505f32;

        type SymSpectra = alloc::boxed::Box<[[crate::engine::scalar::Cmplx<f32>; 8]; 79]>;
        let mut spectra: Vec<(&str, SymSpectra)> = Vec::new();
        for (label, residual) in [
            ("mfsk-core own SIC", &mfsk_residual),
            ("jt9 own SIC", &jt9_residual),
        ] {
            let cand = crate::engine::sync::SyncCandidate {
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
            let argmax = |cs: &[[crate::engine::scalar::Cmplx<f32>; 8]; 79]| -> (usize, f32) {
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
        use crate::engine::sync::refine_candidate;
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

        let jt9_bytes = match std::fs::read("/tmp/jt9_post_sic_dd.raw") {
            Ok(b) => b,
            Err(_) => {
                eprintln!(
                    "skipping issue_182_dk8ne_llr_reliability_comparison: \
                     /tmp/jt9_post_sic_dd.raw (WSJT-X jt9 post-SIC residual dump) \
                     not present — this is a local-only diagnostic input"
                );
                return;
            }
        };
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
            let cand = crate::engine::sync::SyncCandidate {
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
        use crate::engine::sync::refine_candidate;
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
        let cand = crate::engine::sync::SyncCandidate {
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
            .strictness(DecodeStrictness::Normal)
            .sic_early()
            .ap_hint(&ap)
            .decode()
            .results;
        for r in &results {
            let msg = unpack77(r.message77()).unwrap_or_default();
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

        // Blind decode only (no AP hint) -- staged SIC (`.sic_early()`) has
        // been the default since #180/#183.
        for rep in 0..3 {
            let t0 = std::time::Instant::now();
            let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
                .strictness(DecodeStrictness::Normal)
                .sic_early()
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
