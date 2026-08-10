//! Per-candidate decode loop + facade entry functions.
//!
//! The engine room of FT8 decoding. Contains the multipass driver,
//! refine / fine_refine passes, the per-candidate processing loop
//! that drives LLR → BP → OSD → AP, and `process_one_candidate_inner`
//! (still home to the OSD dispatch block ε.6 lifts into
//! `osd_strategy.rs`).
//!
//! Public-facade entries (`decode_block`, `decode_block_tuned`,
//! `decode_block_with_ap[_tuned]`, `decode_block_into[_tuned]`) live
//! here too, since the file body owns the `decode_block_multipass`
//! they wrap. The parent module (`decode_block.rs`) re-exports them
//! at the same `mfsk_core::ft8::decode_block::*` paths external
//! callers (`mfsk-ffi-ft8`, integration tests, `super::decode::*`)
//! already use.
//!
//! ε.5 of the `docs/CLEANUP_2026_05.md` `decode_block` split.

use alloc::boxed::Box;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::decode::{ApHint, DecodeDepth, DecodeResult, DecodeStrictness, LlrEffort};
use super::super::llr::sync_quality;
use super::super::message::unpack77;
use super::super::params::{COSTAS, DEFAULT_BP_MAX_ITER, LDPC_N, NSPS, NTONES};
use super::super::wave_gen::message_to_tones;
use super::coarse_sync::coarse_sync;
#[cfg(all(feature = "fixed-point", not(feature = "fft-rustfft")))]
use super::fill_symbol_spectra::fill_symbol_spectra_goertzel;
use super::fill_symbol_spectra::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
use super::spectrogram::{Spectrogram, compute_spectrogram};
use super::types::{
    AudioSample, DEFAULT_Q_THRESH, NFFT_SPEC, NMS_ALPHA, NSTEP, SAMPLE_RATE_HZ, TONE_SPACING_HZ,
    TX_START_OFFSET_S,
};
use crate::engine::scalar::{Cmplx, ComplexSpec};
use crate::engine::sync::SyncCandidate;
use crate::fec::ldpc::bp::check_crc14;
#[cfg(feature = "fft-rustfft")]
use crate::fec::ldpc::osd::osd_decode_deep;
use num_complex::Complex;

// ── Stage-timing trace (host diagnostic only) ───────────────────────────────
//
// `MFSK_TRACE_STAGE_FT8` env var — same idiom as this file's own
// `MFSK_TRACE_PHANTOM` and `engine::pipeline`'s
// `MFSK_TRACE_STAGE_FT4`/`_FST4`: zero cost when unset, `eprintln!`s
// per-stage wall-clock + candidate counts. Covers *both* FT8 host
// engines — `ft8::decode::decode_frame_inner` (default/parallel
// strategy) and `decode_block_multipass` below (`.sic_rounds()`/
// `.sic_early()`) — since both funnel through `process_one_candidate_inner`
// for the OSD-attempt count. `pub(in crate::ft8)` so `ft8::decode`'s
// own nsync-gate call site can share the same counters. See
// `~/.claude/plans/moonlit-snuggling-puzzle.md`'s phase-wise benchmark
// plan.
#[cfg(feature = "std")]
pub(in crate::ft8) static TRACE_NSYNC_FAIL: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "std")]
pub(in crate::ft8) static TRACE_NSYNC_PASS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "std")]
pub(in crate::ft8) static TRACE_OSD_ATTEMPT: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

#[cfg(feature = "std")]
pub(in crate::ft8) fn stage_trace_enabled() -> bool {
    std::env::var("MFSK_TRACE_STAGE_FT8").is_ok()
}

// Phase 1.7.7-Stick: both `refine_candidates_into` (pass-2) and the
// embedded branch of `process_candidates_into_with_cs_scratch_tuned`
// (stage-3) call `fill_symbol_spectra_goertzel` (zero scratch). The
// legacy BASIS per-symbol DFT path was removed in 0.8.0 (issue #162).

// ── Public entry ────────────────────────────────────────────────────────────

/// Embedded FT8 decode for one 15-s slot.
///
/// Runs the same algorithm shape as the host
/// [`DecodeRequest`](crate::msg::decode_request::DecodeRequest) single-pass
/// path but talks only to power-of-two FFTs (via the
/// [`crate::engine::fft::FftPlanner`] trait) and uses the min-sum LDPC
/// kernel to skip per-iteration `tanh` / `atanh`. No
/// `decode_sniper*` paths are involved; no wide-band 192 k FFT cache.
///
/// Sensitivity vs `decode_frame` is characterised on host AWGN
/// sweeps before any embedded port — see
/// `tests/ft8_decode_block_snr_sweep.rs`.
///
/// # Arguments
/// * `audio`     — 12 kHz i16 PCM, length up to NMAX = 180 000.
/// * `freq_min`  — lower edge of carrier search (Hz).
/// * `freq_max`  — upper edge of carrier search (Hz).
/// * `sync_min`  — minimum normalised Costas score (typical 1.0–2.0).
/// * `depth`     — `Bp` / `BpAll` / `BpAllOsd`.
/// * `max_cand`  — cap on Costas candidates evaluated.
pub fn decode_block<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        None,
        DecodeStrictness::Normal,
        None,
    )
}

/// [`decode_block`] with a per-candidate streaming callback — fires
/// `on_result` once per accepted candidate, in candidate-processing
/// order, immediately before that candidate's result is pushed into
/// the returned `Vec` (this path is always sequential — no
/// `parallel`/rayon on embedded — so every callback delivery is
/// guaranteed to also appear in the returned `Vec`, unlike the host
/// `DecodeRequest::on_result`'s single-pass/sniper strategies, which
/// are parallelized and can fire on a same-slot duplicate that's later
/// excluded — see that method's doc comment for the full contrast).
///
/// Embedded (`not(fft-rustfft)`) build — see [`decode_block_streaming`]
/// below for the host `fft-rustfft` sibling with the same signature and
/// contract, now that issue #243 made the multipass driver's SNR gate
/// atomic per-candidate.
#[cfg(not(feature = "fft-rustfft"))]
pub fn decode_block_streaming<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    on_result: &mut dyn FnMut(&DecodeResult),
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass1 = fine_refine_pass1(audio, pass1);
    let pass2 = refine_candidates(audio, pass1, max_cand, None);
    process_candidates_tuned_streaming(
        audio,
        pass2,
        depth,
        DEFAULT_Q_THRESH,
        DEFAULT_BP_MAX_ITER,
        on_result,
    )
}

/// [`decode_block`] with a per-candidate streaming callback — host
/// `fft-rustfft` sibling of the embedded `decode_block_streaming`
/// above, same signature, same contract: `on_result` fires once per
/// accepted candidate, in processing order, immediately before that
/// candidate is pushed into the returned `Vec`.
///
/// Until issue #243 this driver's `xsnr2` SNR validity gate
/// (`ft8b.f90:456`) ran as a post-hoc batch *after* an entire pass (or,
/// in the very first cut of the fix, an entire multipass loop)
/// finished — a candidate could be streamed and then later dropped or
/// have its `snr_db` rewritten by that batch gate, with no
/// revise/retract event this callback could deliver. The gate now runs
/// inline, immediately per candidate — right after that candidate's
/// signal is subtracted from the working buffer and before it is
/// pushed into `all` — so a result reaching `on_result` here is exactly
/// as final as one reaching the returned `Vec`, matching WSJT-X's own
/// `ft8b.f90` one-candidate-per-call decode→gate→return atomicity.
///
/// Runs `depth`/`max_cand` with `ap_hint = None`, `strictness =
/// DecodeStrictness::Normal`, `bp_max_iter = DEFAULT_BP_MAX_ITER` —
/// mirrors plain [`decode_block`], not the AP-aware or tuned variants.
/// Use `decode_block_with_ap`/`decode_block_with_ap_tuned` (currently
/// non-streaming) if AP rescue or custom tuning is needed; those can
/// grow their own `_streaming` siblings the same way if a caller needs
/// it — ask before assuming this one covers it.
#[cfg(feature = "fft-rustfft")]
pub fn decode_block_streaming<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    on_result: &mut dyn FnMut(&DecodeResult),
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        None,
        DecodeStrictness::Normal,
        Some(on_result),
    )
}

/// Variant of [`decode_block`] that accepts a runtime `bp_max_iter`
/// (= per-LLR-variant BP iteration cap, default
/// [`DEFAULT_BP_MAX_ITER`]). Useful on time-budgeted targets.
#[allow(clippy::too_many_arguments)]
pub fn decode_block_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
        None,
    )
}

/// AP-aware variant of [`decode_block`]. Mirrors `decode_block`'s
/// behaviour exactly when `ap_hint = None`; with `Some(&ap)` runs
/// the full WSJT-X iaptype loop (5..12) per candidate after Steps 1-3
/// (BP staircase + OSD) all fail. Host `fft-rustfft` build only —
/// embedded fixed-point keeps its existing iaptype-1-only path.
///
/// Used by host `decode_frame_with_ap` after the v0.6.1 redirect, and
/// by mountain-top apps that want full AP rescue from a single entry
/// point. New in 0.6.1.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_with_ap<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        ap_hint,
        DecodeStrictness::Normal,
        None,
    )
}

/// Variant of [`decode_block_with_ap`] that accepts runtime
/// `bp_max_iter` and `strictness`. New in 0.6.1.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_with_ap_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        bp_max_iter,
        ap_hint,
        strictness,
        None,
    )
}

/// WSJT-X `ft8_decode.f90:172-236` 3-pass loop driver. Each pass:
/// coarse_sync on the (subtracted) audio, fine refine, decode, then
/// LPF-subtract every fresh CRC-passing decode for the next pass.
///
/// Pass termination matches WSJT-X exactly:
/// - pass 1 always runs;
/// - pass 2 skips when pass 1 returned 0 decodes;
/// - pass 3 skips when pass 2 returned no NEW decodes.
///
/// On host (`fft-rustfft`) the audio is cloned to a working `Vec<i16>`
/// (subtract operates on i16 samples). Embedded targets compile through
/// the same path; the clone cost is dominated by the BP work it enables.
///
/// `on_result`, when `Some`, fires once per candidate immediately after
/// that candidate clears the xsnr2 validity gate below and right before
/// it is pushed into `all` — see [`decode_block_streaming`]'s doc
/// comment (issue #243) for why this is safe to expose as a streaming
/// callback.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
fn decode_block_multipass<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    mut on_result: Option<&mut dyn FnMut(&DecodeResult)>,
) -> Vec<DecodeResult> {
    use alloc::vec::Vec as AllocVec;
    #[cfg(feature = "std")]
    let trace_stage = stage_trace_enabled();
    let mut work: AllocVec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let mut all: AllocVec<DecodeResult> = AllocVec::new();
    let mut prev_total: usize = 0;
    // Shared across every pass's per-candidate loop below (issue #199):
    // this driver calls the per-candidate decode entry once per
    // candidate (1-element `vec![cand]`), so without a caller-owned
    // scratch here it would otherwise be reallocated on every
    // iteration, defeating the reuse `process_candidates_with_ap`'s
    // scratch pool exists for.
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    for ipass in 0..3 {
        if ipass >= 1 && all.len() == prev_total {
            // Pass 2 skips on zero from pass 1; pass 3 on zero new.
            break;
        }
        prev_total = all.len();

        let spec = compute_spectrogram(work.as_slice(), freq_max);
        // Capture *this pass's own* spectrogram + per-bin baseline for
        // WSJT-X-faithful xsnr2 SNR (issue #243). `ft8_decode.f90:217`
        // calls `sync8` (which produces `sbase`) *inside* the pass
        // loop, once per pass, against that pass's then-current audio
        // — already reflecting whatever subtraction the *prior*
        // pass(es) did. An earlier version of this port captured this
        // only once, at `ipass == 0`, and reused that stale
        // pre-subtraction snapshot for every later pass's candidates
        // too.
        //
        // `sbase` comes from `compute_baseline_spectrum`
        // (`get_spectrum_baseline.f90`-faithful: Nuttall window, 50%
        // overlap), *not* `avg_spectrum` over this pass's own `spec`
        // (rectangular window) — see that function's doc comment for
        // why reusing `spec` under-reports SNR on busy/crowded bands
        // (issue #253 follow-up, 2026-08-10, verified against real
        // `jt9` ground truth). `spec` (rectangular) is kept only for
        // `recompute_nsync`'s sync-quality gate below — unrelated to
        // the xsig/xbase scale calibration. `xsig` no longer comes
        // from `spec` either (see the per-candidate block below) — it
        // now reads the same WSJT-X `cd0`/per-symbol-FFT pipeline
        // `fill_symbol_spectra` already computes for LLR, per
        // `compute_baseline_spectrum`'s doc comment on why the two
        // must be paired.
        #[cfg(not(feature = "fixed-point"))]
        let sbase_and_spec: Option<(AllocVec<f32>, Spectrogram)> = {
            let avg = crate::ft8::baseline::compute_baseline_spectrum(work.as_slice());
            let sbase_v = crate::ft8::baseline::fit_baseline(&avg, 0, spec.n_freq - 1);
            let spec_clone = Spectrogram {
                n_freq: spec.n_freq,
                n_time: spec.n_time,
                data: spec.data.clone(),
            };
            Some((sbase_v, spec_clone))
        };
        #[cfg(feature = "std")]
        let __trace_t0 = trace_stage.then(std::time::Instant::now);
        let cands = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
        drop(spec);
        let cands = fine_refine_pass1(work.as_slice(), cands);
        #[cfg(feature = "std")]
        if let Some(t0) = __trace_t0 {
            eprintln!(
                "TRACE_STAGE_FT8_MP pass={} coarse_sync+fine_refine={:.1}ms n_candidates={}",
                ipass,
                t0.elapsed().as_secs_f64() * 1000.0,
                cands.len()
            );
        }
        // Wide-band 192k-FFT cache, shared across every `refine_candidates`
        // / `process_candidates_tuned_with_ap` call in this pass — valid
        // as long as `work` hasn't changed. Rebuilt lazily (only when the
        // next candidate is actually processed) rather than eagerly after
        // every subtract, since most candidates in a pass don't decode.
        let mut fft_cache: Option<alloc::vec::Vec<Complex<f32>>> =
            Some(crate::ft8::downsample::build_fft_cache(work.as_slice()));
        #[cfg(feature = "std")]
        let __trace_t1 = trace_stage.then(std::time::Instant::now);
        let pass2 = refine_candidates(work.as_slice(), cands, max_cand, fft_cache.as_deref());
        #[cfg(feature = "std")]
        if let Some(t1) = __trace_t1 {
            eprintln!(
                "TRACE_STAGE_FT8_MP pass={} refine_candidates={:.1}ms n_after={}",
                ipass,
                t1.elapsed().as_secs_f64() * 1000.0,
                pass2.len()
            );
        }
        #[cfg(feature = "std")]
        if trace_stage {
            TRACE_NSYNC_FAIL.store(0, core::sync::atomic::Ordering::Relaxed);
            TRACE_NSYNC_PASS.store(0, core::sync::atomic::Ordering::Relaxed);
            TRACE_OSD_ATTEMPT.store(0, core::sync::atomic::Ordering::Relaxed);
        }
        #[cfg(feature = "std")]
        let __trace_t2 = trace_stage.then(std::time::Instant::now);
        #[cfg(feature = "std")]
        let __trace_pass_start_len = all.len();

        // **WSJT-X ft8b.f90:432-437 sequential subtract**: each
        // accepted decode immediately subtracts from `work` so the
        // NEXT candidate in this same pass sees a cleaner residual.
        // Without this, all candidates in a pass see the same raw
        // audio — strong real signals at one freq leak Costas-aligned
        // energy into nearby phantom candidates' bins, allowing
        // CRC-pass garbage to decode there. The driver's outer pass
        // loop is for OSD/AP differences (ndepth-dependent), not for
        // the subtract cadence.
        #[cfg(feature = "std")]
        let trace = std::env::var("MFSK_TRACE_PHANTOM").is_ok();
        #[cfg(not(feature = "std"))]
        let trace = false;
        for cand in pass2 {
            if fft_cache.is_none() {
                fft_cache = Some(crate::ft8::downsample::build_fft_cache(work.as_slice()));
            }
            let single_results = process_candidates_tuned_with_ap_scratch(
                work.as_slice(),
                alloc::vec![cand],
                depth,
                DEFAULT_Q_THRESH,
                bp_max_iter,
                ap_hint,
                strictness,
                fft_cache.as_deref(),
                &mut bp_scratch,
                None,
            );
            #[cfg_attr(feature = "fixed-point", allow(unused_mut))]
            for mut r in single_results {
                if all.iter().any(|x| x.message77() == r.message77()) {
                    continue;
                }
                if trace {
                    #[cfg(feature = "std")]
                    if let Some(text) = crate::msg::wsjt77::unpack77(r.message77()) {
                        eprintln!(
                            "  TRACE pass={} freq={:>7.2} dt={:+.4} e={:>2} '{}'",
                            ipass, r.freq_hz, r.dt_sec, r.hard_errors, text,
                        );
                    }
                }
                // `xsig` for the xsnr2 gate below (issue #253 follow-up,
                // 2026-08-10) — must run *before* the subtract: WSJT-X
                // fills `s8` early in `ft8b`, well before `subtractft8`
                // runs for this candidate, so it reflects audio with
                // every *earlier* candidate in this pass already
                // subtracted (sequential SIC) but not this one's own
                // signal yet. See `compute_xsig_wsjtx`'s doc comment for
                // why this needs the `cd0`/per-symbol-FFT pipeline rather
                // than `compute_spectrogram`'s rectangular one.
                #[cfg(not(feature = "fixed-point"))]
                let xsig_wsjtx: f32 = compute_xsig_wsjtx(&r, work.as_slice(), fft_cache.as_deref());

                // WSJT-X `ft8b.f90:432-437` subtracts *before* its own
                // xsnr2 gate check (below) runs — matched here too, not
                // just for residual cleanliness: this crate's own
                // sequential-subtract design already relied on it. This
                // ordering only matters for computing `xsig_wsjtx` above
                // (which needs pre-subtract audio); `apply_wsjtx_xsnr2`
                // below consumes the already-captured value, so it's
                // order-independent relative to the subtract itself.
                crate::ft8::subtract::subtract_signal_lpf(work.as_mut_slice(), &r);
                fft_cache = None; // `work` changed — cache is stale.

                // WSJT-X xsnr2 validity gate (issue #243, ft8b.f90:456),
                // applied *immediately*, per candidate — not deferred to
                // a later batch pass. This is what makes `on_result`
                // streaming safe to wire for this driver (see
                // `decode_block_streaming`'s own doc comment): a result
                // is fully finalised, accepted or dropped, in this same
                // iteration, before the next candidate is even looked
                // at — matching `ft8b.f90`'s own one-candidate-per-call,
                // decode→gate→return atomicity exactly, not just
                // approximating it at pass granularity.
                //
                // xsnr2/xbase post-process is f32-only. Fixed-point
                // Spectrogram cells are quantised post `>> FP_SPEC_SHIFT`,
                // putting many noise cells at u16 zero — `fit_baseline`'s
                // `log10(p.max(1e-30))` then produces sbase ≈ -250 dB and
                // xsnr2 explodes. The original adjacent-tone SNR from
                // `process_candidates_into` (compute_snr_db) is already
                // on a sensible scale, so leave it untouched on the
                // fixed-point path.
                #[cfg(not(feature = "fixed-point"))]
                if let Some((sbase, spec)) = &sbase_and_spec
                    && !apply_wsjtx_xsnr2(&mut r, xsig_wsjtx, sbase, spec)
                {
                    continue;
                }

                if let Some(cb) = on_result.as_deref_mut() {
                    cb(&r);
                }
                all.push(r);
            }
        }
        #[cfg(feature = "std")]
        if let Some(t2) = __trace_t2 {
            eprintln!(
                "TRACE_STAGE_FT8_MP pass={} decode_loop={:.1}ms nsync_fail={} nsync_pass={} osd_attempt={} n_new={}",
                ipass,
                t2.elapsed().as_secs_f64() * 1000.0,
                TRACE_NSYNC_FAIL.load(core::sync::atomic::Ordering::Relaxed),
                TRACE_NSYNC_PASS.load(core::sync::atomic::Ordering::Relaxed),
                TRACE_OSD_ATTEMPT.load(core::sync::atomic::Ordering::Relaxed),
                all.len() - __trace_pass_start_len
            );
        }
    }
    all
}

/// Hard-decision sync count (= WSJT-X `ft8b.f90:163-176` nsync) read
/// from the pass-1 spectrogram at the result's refined (freq, dt).
/// 21-bit upper bound (3 sync blocks × 7 Costas positions).
///
/// Only the host f32 build calls this — `recompute_snr_xsnr2` /
/// `recompute_nsync` use the `xsnr2/xbase` formulation which is
/// f32-only (see the `#[cfg(not(feature = "fixed-point"))]` caller
/// at line ~1877).
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
fn recompute_nsync(
    result: &DecodeResult,
    spec: &Spectrogram,
    df: f32,
    tstep: f32,
    nsps_steps: f32,
) -> u32 {
    use crate::ft8::params::COSTAS;
    const NTONES: usize = 8;
    let carrier_bin_f = result.freq_hz / df;
    let tone_step = TONE_SPACING_HZ / df; // = 2.0 at NFFT=3840
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;
    // Costas blocks at symbol indices 0, 36, 72 (each 7 symbols long).
    let mut count = 0u32;
    for &block_off in &[0_usize, 36, 72] {
        for (sym_in_block, &expected) in COSTAS.iter().enumerate() {
            let k = block_off + sym_in_block;
            let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
            if m_bin < 0 || m_bin as usize >= spec.n_time {
                continue;
            }
            let m_bin = m_bin as usize;
            let mut best_t = 0;
            let mut best_p = f32::MIN;
            for t in 0..NTONES {
                let f_bin = (carrier_bin_f + (t as f32) * tone_step).round() as i32;
                if f_bin < 0 || f_bin as usize >= spec.n_freq {
                    continue;
                }
                let p = spec.power_acc(f_bin as usize, m_bin);
                if p > best_p {
                    best_p = p;
                    best_t = t;
                }
            }
            if best_t == expected {
                count += 1;
            }
        }
    }
    count
}

/// Slot-baseline xsnr2 SNR for any [`Spectrogram`] — `std`-free
/// alternative to `recompute_snr_xsnr2` for callers that haven't
/// run `baseline::fit_baseline`'s polynomial smoother (= the
/// embedded path: `mfsk-core` is built `default-features = false`
/// + `alloc` only there, so the polynomial fit is unavailable).
///
/// Computes the per-frequency baseline as the mean across time over
/// a ±50-bin window centred on the decode's carrier — same idea as
/// `avg_spectrum` but localised so it works incrementally and stays
/// cheap on Xtensa LX6/LX7. Then evaluates WSJT-X `ft8b.f90:449-454`:
///
/// ```text
///   xbase = mean_over_time(spec[carrier_window]) * cell_scale
///   xsig  = sum_over_79_decoded_tones(spec[tone_bin, m]) * cell_scale
///   xsnr2 = xsig / xbase / 3e6 - 1
///   snr_db = 10·log10(xsnr2) - 27        (clamped at -24 dB on degeneracy)
/// ```
///
/// `cell_scale = 1.0` for an `f32` spectrogram, `2^FP_SPEC_SHIFT`
/// (`= 4096.0` with the default fixed-point shift) for an embedded
/// u16 spectrogram so xsig and xbase land in the same WSJT-X
/// calibration regime.
///
/// Vs the per-Costas-block adjacent-tone SNR `compute_snr_db` returns
/// from `process_candidates*`: that ratio is preserved under the
/// `fill_symbol_spectra` per-block auto-gain so it's *internally*
/// consistent, but its absolute number drifts ~0–15 dB between
/// signals because each block's gain factor is signal-dependent.
/// This function reads `xsig`/`xbase` from the *single* sync8
/// spectrogram (uniform `FP_SPEC_SHIFT` auto-gain across the slot),
/// so the result is comparable across signals AND comparable to
/// WSJT-X / JTDX SNR reports.
pub fn xsnr2_db_simple(spec: &Spectrogram, result: &DecodeResult, cell_scale: f32) -> f32 {
    use crate::ft8::params::NN;
    use crate::ft8::wave_gen::message_to_tones;

    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let nsps_steps = (NSPS / NSTEP) as f32;
    let tone_step = TONE_SPACING_HZ / df;

    if spec.n_freq == 0 || spec.n_time == 0 {
        return -24.0;
    }

    // Per-freq baseline — **median** (P50) of cell values inside a
    // local window around the decode's carrier. A plain mean is
    // dragged upward by the very signal we're trying to measure, so
    // xbase tracks xsig and the ratio collapses (verified
    // empirically: W1FC at 0 dB had xbase ≈ 4.8 M while N1JFU at
    // -14 dB had ≈ 0.15 M before this change). Median ignores the
    // signal-bin outliers and reads true noise floor.
    //
    // Window: ±50 freq bins (~156 Hz at NFFT=3840) × time-decimated
    // by `n_time / 50` so the sort touches ~2 500 samples. Sort cost
    // on f32 ~25 k comparisons ~100 µs at LX7 240 MHz — well inside
    // the post-SlotEnd budget.
    let carrier_bin = (result.freq_hz / df)
        .round()
        .clamp(0.0, (spec.n_freq - 1) as f32) as usize;
    let f_lo = carrier_bin.saturating_sub(50);
    let f_hi = (carrier_bin + 50).min(spec.n_freq - 1);
    let t_stride = spec.n_time.div_ceil(50).max(1);
    let mut samples: alloc::vec::Vec<f32> =
        alloc::vec::Vec::with_capacity((f_hi - f_lo + 1) * spec.n_time.div_ceil(t_stride) + 4);
    for f in f_lo..=f_hi {
        let mut t = 0usize;
        while t < spec.n_time {
            samples.push(spec.power_acc(f, t));
            t += t_stride;
        }
    }
    // Median via O(N) `select_nth_unstable_by` instead of a full
    // O(N log N) sort — only the middle element matters here, the
    // pivot ordering inside the partition is irrelevant. Gemini PR
    // #81 review.
    let median = if samples.is_empty() {
        0.0
    } else {
        let mid = samples.len() / 2;
        samples.select_nth_unstable_by(mid, |a, b| {
            a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)
        });
        samples[mid]
    };
    let xbase = median * cell_scale;
    if xbase <= 0.0 || !xbase.is_finite() {
        return -24.0;
    }

    // xsig at the 79 decoded-tone (freq, m) positions.
    let itone = message_to_tones(result.message77());
    let carrier_bin_f = result.freq_hz / df;
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;
    let mut xsig: f32 = 0.0;
    for k in 0..NN {
        let t = itone[k] as f32;
        let f_bin = (carrier_bin_f + t * tone_step).round() as i32;
        let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
        if f_bin < 0 || f_bin as usize >= spec.n_freq || m_bin < 0 || m_bin as usize >= spec.n_time
        {
            continue;
        }
        xsig += spec.power_acc(f_bin as usize, m_bin as usize);
    }
    xsig *= cell_scale;

    // Empirical calibration constant — pair-matched against the
    // JTDX qso3_busy reference on real M5StickS3 silicon
    // (2026-05-05). With the median-of-window noise floor above the
    // raw `xsig/xbase` ratio spans ~1100 (-14 dB SNR) → ~100 000
    // (0 dB SNR) on the embedded u16 spectrogram; the value below
    // collapses that to JTDX-comparable dB within ±3 dB across
    // weak / mid / strong signals.
    //
    // (WSJT-X's `ft8b.f90:451` value `3e6` is calibrated to their
    // f32 spectrogram amplitude scale and doesn't carry over here.)
    const XSNR2_CAL_DB: f32 = 46.0;

    let ratio = xsig / xbase;
    if ratio <= 1.0 {
        return -24.0;
    }
    let snr = 10.0 * ratio.log10() - XSNR2_CAL_DB;
    snr.max(-24.0)
}

/// WSJT-X `ft8b.f90:449-454` xsnr2 SNR formula:
///
/// ```text
///   xbase = 10^((sbase[round(f1/df)] - 40) / 10)
///   xsnr2 = xsig / xbase / 3e6 - 1
///   xsnr2_db = 10·log10(xsnr2) - 27
/// ```
///
/// `xsig` (the caller-supplied per-candidate signal power) and `sbase`
/// (from [`crate::ft8::baseline::compute_baseline_spectrum`]) must both
/// come from their WSJT-X-faithful source pipelines — the `cd0`/
/// per-symbol-FFT chain for `xsig`, the Nuttall-window
/// `get_spectrum_baseline.f90` chain for `sbase` — for the `/3e6 - 27`
/// calibration to map onto a WSJT-X-compatible dB number. Pairing
/// either with `compute_spectrogram`'s rectangular-window spectrum
/// instead measurably breaks this (verified 2026-08-10, issue #253
/// follow-up): the two pipelines' absolute gains don't match by
/// construction (different windows), so an inconsistent pairing adds a
/// spurious offset rather than cancelling one out.
///
/// Falls back to `-24 dB` if the ratio degenerates.
///
/// f32-only — fixed-point spectrograms quantise to u16, putting noise
/// cells at zero and breaking the `log10` baseline; see the comment
/// block at the `retain_mut` caller for the full rationale.
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
fn recompute_snr_xsnr2(freq_hz: f32, xsig: f32, sbase: &[f32], df: f32) -> f32 {
    let bin = (freq_hz / df).round() as i32;
    let bin = bin.clamp(0, sbase.len() as i32 - 1) as usize;
    let xbase = 10f32.powf(0.1 * (sbase[bin] - 40.0));
    let arg = xsig / xbase / 3.0e6 - 1.0;
    // WSJT-X `ft8b.f90:445-454`: `xsnr2 = max(0.001, xsig/xbase/3e6 - 1)`
    // then `xsnr2_db = 10·log10(xsnr2) - 27` → floors at -57 dB.
    // Caller (`retain_mut` in `decode_block_multipass`) applies the
    // `xsnr < -24` gate against this raw value BEFORE clamping it to
    // the -24 dB display floor; the previous `snr.max(-24.0)` here
    // pre-clamped and made the gate fire only for arithmetic
    // underflow, not for "degenerate signal" cases.
    //
    // The previous form `if arg > 0.1 { arg } else { 0.001 }` was an
    // mfsk-core-specific deviation: at `arg = 0.1` the result jumped
    // from ~-37 dB straight to -57 dB (Gemini PR #88 review). WSJT-X
    // is continuous from -57 dB upward via the simple `max`.
    let xsnr2 = arg.max(0.001);
    10.0 * xsnr2.log10() - 27.0
}

/// `xsig` (WSJT-X `ft8b.f90:154-161`'s `s8`) via the real `cd0`/
/// per-symbol-FFT pipeline (`fill_symbol_spectra`) — the same one this
/// codebase already uses for LLR, and the same one real WSJT-X's own
/// `xsig` comes from. **Not** `compute_spectrogram`'s rectangular
/// coarse-sync spectrum (the previous approach): see
/// [`crate::ft8::baseline::compute_baseline_spectrum`]'s doc comment
/// for why that doesn't calibrate against
/// [`recompute_snr_xsnr2`]/`sbase`.
///
/// `audio` must be the state *before* this candidate's own signal is
/// subtracted from it (matches WSJT-X: `s8` is filled early in `ft8b`,
/// well before `subtractft8` runs for the same candidate).
///
/// Shared by every FT8 entry point (`decode_block`, `.sic_rounds()`,
/// `.sic_early()`, the plain single-pass `.decode()`, `.sniper()`) so
/// they all report the same SNR for the same signal — before this
/// (issue #253 SNR-calibration follow-up, 2026-08-10) `decode_block`
/// and `DecodeRequest` used two different metrics entirely (this one
/// vs. the adjacent-tone `compute_snr_db` in `super::super::llr`),
/// confirmed by a real production consumer (WebFT8) seeing visibly
/// different SNR for the same signal depending on which entry point it
/// called.
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
pub(crate) fn compute_xsig_wsjtx(
    result: &DecodeResult,
    audio: &[i16],
    fft_cache: Option<&[Complex<f32>]>,
) -> f32 {
    let itone = crate::ft8::wave_gen::message_to_tones(result.message77());
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> = alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
        .try_into()
        .unwrap();
    fill_symbol_spectra(
        &mut cs,
        audio,
        result.freq_hz,
        result.dt_sec,
        SymMask::SyncOnly,
        fft_cache,
    );
    fill_symbol_spectra(
        &mut cs,
        audio,
        result.freq_hz,
        result.dt_sec,
        SymMask::DataOnly,
        fft_cache,
    );
    let mut xsig = 0.0f32;
    for (k, tones) in cs.iter().enumerate() {
        // Undo `fill_symbol_spectra`'s `CS_SCALE = 1/1000`
        // (`ft8b.f90:159`'s `cs = csymb/1e3`) — `xsig` needs the raw,
        // unscaled `s8 = abs(csymb)`.
        let c = tones[itone[k] as usize];
        let re = c.re * 1000.0;
        let im = c.im * 1000.0;
        xsig += re * re + im * im;
    }
    xsig
}

/// Replaces `result.snr_db` with WSJT-X's `xsnr2` and applies its
/// validity gate — the shared finishing step every FT8 entry point
/// runs on an already-CRC-passed candidate. `xsig` must come from
/// [`compute_xsig_wsjtx`] called *before* the candidate's own
/// subtract; `sbase`/`spec` are this pass's already-captured baseline
/// and coarse-sync spectrum (order-independent relative to the
/// subtract — both are frozen snapshots by the time this runs).
///
/// Returns `false` if the candidate fails WSJT-X's `nsync <= 10 &&
/// xsnr < -24.0 dB` bail-out (`ft8b.f90:456`) — caller should drop it.
/// `result.snr_db` is updated (clamped to -24 dB floor) whenever this
/// returns `true`.
///
/// **WSJT-X post-decode validity gates (#63).** Mirrors
/// `ft8b.f90:422-459`'s `nsync <= 10 && xsnr < -24.0` bail-out (line
/// 456) specifically — the msg-type `i3`/`n3` validity (lines
/// 425-428) and `unpack77` success (line 430) gates are intentionally
/// omitted here: every `result` this is called with already came
/// through `process_one_candidate_inner`, which already rejects via
/// `unpack77(&bp.message77)?`. `unpack77` returns `None` for all-zero
/// messages (i3=0 n3=0 → free text → empty string → None) and for
/// every invalid i3/n3 combination, so repeating those checks here
/// would just pay for a second `unpack77` call per result (Gemini PR
/// #88 review).
///
/// The gate runs on the RAW (un-clamped) `xsnr2` because both WSJT-X
/// (line 460) and an earlier version of this port clamp the value to
/// -24 dB for display *after* the gate — clamping first would collapse
/// every "below floor" result to exactly -24, dead-letter the `xsnr <
/// -24.0` test, and let exactly the phantoms the gate was designed to
/// catch slip through (the qso3_busy phantoms named in issue #63's
/// body).
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
pub(crate) fn apply_wsjtx_xsnr2(
    result: &mut DecodeResult,
    xsig: f32,
    sbase: &[f32],
    spec: &Spectrogram,
) -> bool {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let nsps_steps = (NSPS / NSTEP) as f32;

    let raw_snr = recompute_snr_xsnr2(result.freq_hz, xsig, sbase, df);
    let nsync = recompute_nsync(result, spec, df, tstep, nsps_steps);
    if nsync <= 10 && raw_snr < -24.0 {
        return false;
    }
    result.snr_db = raw_snr.max(-24.0);
    true
}

/// Embedded path: single-pass `decode_block` (matches the previous
/// production behaviour, no subtract). Host-only `fft-rustfft` adds
/// the multipass driver. `_ap_hint` and `_strictness` parameters are
/// accepted for signature parity with the host variant but ignored
/// here — embedded fixed-point keeps its existing iaptype-1-only
/// hardcoded plumbing (full AP deferred to 0.7.x).
#[cfg(not(feature = "fft-rustfft"))]
#[allow(clippy::too_many_arguments)]
fn decode_block_multipass<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    _ap_hint: Option<&ApHint>,
    _strictness: DecodeStrictness,
    _on_result: Option<&mut dyn FnMut(&DecodeResult)>,
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass1 = fine_refine_pass1(audio, pass1);
    let pass2 = refine_candidates(audio, pass1, max_cand, None);
    process_candidates_tuned(audio, pass2, depth, DEFAULT_Q_THRESH, bp_max_iter)
}

/// Per-candidate WSJT-X-style 3-stage fine refine. Builds the
/// 192k-FFT cache once and downsamples per candidate. Host-only;
/// embedded paths skip this for compute reasons (cache is 1.5 MB,
/// 192k FFT is not in our embedded planner).
#[cfg(feature = "fft-rustfft")]
pub(super) fn fine_refine_pass1<S: AudioSample>(
    audio: &[S],
    cands: alloc::vec::Vec<crate::engine::sync::SyncCandidate>,
) -> alloc::vec::Vec<crate::engine::sync::SyncCandidate> {
    if cands.is_empty() {
        return cands;
    }
    // Convert audio → Vec<i16> for the downsampler (no-op when S=i16).
    let audio_i16: alloc::vec::Vec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let fft_cache = crate::ft8::downsample::build_fft_cache(&audio_i16);
    cands
        .into_iter()
        .map(|c| {
            // Use `downsample_cached` directly so the FT8 wrapper's
            // `cache.to_vec()` clone (~1.5 MB) on the `Some(_)` branch is
            // bypassed — same pattern as `decode.rs::process_candidate_with_scratch`
            // (that fix's scope never covered this sibling call site).
            let cd0 = crate::engine::dsp::downsample::downsample_cached(
                &fft_cache,
                c.freq_hz,
                &crate::ft8::downsample::FT8_CFG,
            );
            let r = crate::ft8::refine_fine::fine_refine_3stage(&cd0, c.dt_sec);
            crate::engine::sync::SyncCandidate {
                freq_hz: c.freq_hz + r.delf_hz,
                dt_sec: r.dt_sec,
                score: c.score,
            }
        })
        .collect()
}

/// Embedded build path — preserve the original (no fine refine) shape.
///
/// Attempted in 0.6.3 via `fill_symbol_spectra` iteration at
/// 41 (freq, dt) probe points per candidate, but the per-symbol
/// DFT cost (~6900 DFTs/cand × 30 cand × 1920-sample DFT ≈ 200k
/// DFTs/slot) tripped the FreeRTOS task watchdog after ~5 s of
/// uninterrupted compute on S3 LX7 — fundamentally too heavy
/// without the host's 192k FFT shortcut. Reverted to NO-OP. A
/// proper embedded fine_refine needs cd0 built via FIR decimate
/// (3:1 → 4:1 → 5:1, integer ratios, ~30 ms total) followed by
/// `refine_fine_3stage` on the 200 Hz baseband — deferred to a
/// future patch (estimate: ~150 lines for the FIR decimator).
#[cfg(not(feature = "fft-rustfft"))]
pub(super) fn fine_refine_pass1<S: AudioSample>(
    _audio: &[S],
    cands: alloc::vec::Vec<crate::engine::sync::SyncCandidate>,
) -> alloc::vec::Vec<crate::engine::sync::SyncCandidate> {
    cands
}

/// Variant of [`decode_block`] used by embedded fixed-point callers.
/// Same recall / depth / staircase as `decode_block`; kept as a
/// distinct name for API stability with existing embedded callers
/// (`mfsk-ffi-ft8`, `embedded-shared::dual_core`).
#[cfg(feature = "fixed-point")]
pub fn decode_block_into<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_block_into_tuned(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
    )
}

/// Variant of [`decode_block_into`] that accepts a runtime
/// `bp_max_iter`. The recommended top-level entry point for embedded
/// LX6 / LX7 callers — `bp_max_iter` is the dominant time-scaling
/// knob in the post-SlotEnd budget.
#[cfg(feature = "fixed-point")]
pub fn decode_block_into_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass2 = refine_candidates_into(audio, pass1, max_cand);
    process_candidates_into_tuned(audio, pass2, depth, DEFAULT_Q_THRESH, bp_max_iter)
}

/// Pass-1 candidate cap — coarse_sync emits at most this many
/// candidates regardless of `max_cand`. Pass 2 re-ranks by
/// `sync_quality` (the same metric stage 3 uses to gate decode
/// attempts — much sharper than the per-bin power ratio) and
/// truncates to caller's `max_cand` for stage 3.
///
/// Sweep on real-QSO WAVs (host fp i16, BpAll, with the regularised
/// coarse_sync ratio in `RATIO_EPS_DEFAULT`) showed:
/// - PASS1 ∈ {30, 50}: 14/22 truth (drops one weak qso1 signal)
/// - PASS1 ∈ {75, 100}: 15/22 truth (full recall ceiling)
/// - PASS1=200: same 15/22 (no further gain — qso3's remaining gap
///   is at coarse_sync rank 100+, beyond Pass 2's reach).
///
/// 75 is the smallest PASS1 that keeps the full recall ceiling.
/// 30 is the smallest PASS1 that keeps the qso3 (busy band) truth
/// ceiling — it loses one borderline -17 dB qso1 signal (OH3NIV).
/// Core2 ships with 30 (speed-priority — Pass 2 cost ≈ 0.4 s vs
/// 1.0 s at PASS1=75). Override per-call via `MFSK_PASS1_LIMIT`
/// when std is enabled.
const PASS1_LIMIT_DEFAULT: usize = 30;
pub(super) fn pass1_limit() -> usize {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_PASS1_LIMIT")
            && let Ok(v) = s.parse::<usize>()
        {
            return v;
        }
    }
    PASS1_LIMIT_DEFAULT
}

/// One Pass-2 output: the original candidate, its 79×8 Costas-only
/// spectrum (filled in stage 3 with the data-symbol DFT), and its
/// `sync_quality` score for ranking.
pub type RefinedCandidate = (SyncCandidate, Box<[[Cmplx<f32>; 8]; 79]>, u32);

/// Per-candidate Costas-block-0 DFT + sync_quality_block0 re-rank.
/// Keeps the top `max_cand` by Pass-2 score; **the cs spectrum is
/// retained** (block 0 only at this point) and stage 3 fills the
/// remaining 72 symbols via [`SymMask::NotBlock0`].
///
/// Cost: 7 sync symbols × 8 tones = 56 DFT per candidate vs
/// `SyncOnly`'s 168 — 1/3 the work. On Core2 ~13 ms/cand with the
/// asm dot product. PASS1=75 → Pass 2 ≈ 1.0 s.
///
/// The retained `q` is the **block-0** score (range 0..=7, expected
/// ~6-7 for real signals, ~0.875 for noise). Stage 3 recomputes the
/// full 21-symbol `sync_quality` after filling blocks 1, 2 and uses
/// that for its `q > 6` gate; the per-cand sort here is just for
/// truncating to `max_cand`.
pub(super) fn refine_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    fft_cache: Option<&[Complex<f32>]>,
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct(audio, c.freq_hz, c.dt_sec, SymMask::SyncBlock0, fft_cache)
    })
}

/// Variant of [`refine_candidates`] that builds its per-candidate cs
/// via [`fill_symbol_spectra_goertzel`] (zero scratch) instead of the
/// dispatcher `symbol_spectra_direct` uses.
///
/// Public as of v0.6 (#49 cat C): used by the embedded
/// `embedded-shared::dual_core` worker.
#[cfg(feature = "fixed-point")]
pub fn refine_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
) -> Vec<RefinedCandidate> {
    use super::super::params::NN;
    use super::fill_symbol_spectra::fill_symbol_spectra_goertzel;
    // NB (Gemini PR #103 MEDIUM): one cs Box per cand-iter = one 5 KB
    // heap alloc per pass1 candidate (up to 30/slot on embedded).
    // Total transient PSRAM use is ~150 KB, not catastrophic, but the
    // alloc churn fragments the TLSF allocator in long-running
    // sessions. Reusing a single Box across the closure invocations
    // needs `refine_candidates_with` to expose a scratch parameter —
    // the heap is no longer the bottleneck so the win is marginal vs
    // the API churn.
    refine_candidates_with(cands, max_cand, |c| {
        let mut cs: alloc::boxed::Box<[[Cmplx<f32>; 8]; NN]> =
            alloc::vec![[Cmplx::<f32>::default(); 8]; NN]
                .try_into()
                .unwrap();
        fill_symbol_spectra_goertzel(&mut cs, audio, c.freq_hz, c.dt_sec, SymMask::SyncBlock0);
        cs
    })
}

/// Common min-heap selection logic used by both `refine_candidates`
/// (heap-allocated basis per call) and `refine_candidates_into`
/// (caller-provided basis scratch). The closure abstracts how each
/// candidate's cs Box is produced.
fn refine_candidates_with<F>(
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    mut cs_for: F,
) -> Vec<RefinedCandidate>
where
    F: FnMut(&SyncCandidate) -> Box<[[Cmplx<f32>; 8]; 79]>,
{
    use alloc::collections::BinaryHeap;
    use core::cmp::{Ordering, Reverse};

    // Min-heap on q so the smallest survivor is at the top — replace
    // it whenever a stronger candidate arrives. Bounds the live heap
    // footprint at `max_cand × cs Box` regardless of PASS1_LIMIT,
    // which is the heap-fragmentation fix Task #2 was opened for.
    // Old code collected all PASS1=30 cs Boxes (240 KB) before the
    // truncate; new code never holds more than max_cand=15 Boxes
    // (120 KB peak).
    //
    // The heap stores (q, cand_idx, RefinedCandidate); cand_idx
    // breaks ties deterministically (insertion order) so the
    // truncation result is reproducible across runs.
    struct Slot {
        q: u32,
        idx: u32,
        cand: SyncCandidate,
        cs: Box<[[Cmplx<f32>; 8]; 79]>,
    }
    impl PartialEq for Slot {
        fn eq(&self, other: &Self) -> bool {
            self.q == other.q && self.idx == other.idx
        }
    }
    impl Eq for Slot {}
    impl Ord for Slot {
        fn cmp(&self, other: &Self) -> Ordering {
            self.q.cmp(&other.q).then_with(|| self.idx.cmp(&other.idx))
        }
    }
    impl PartialOrd for Slot {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    let mut heap: BinaryHeap<Reverse<Slot>> = BinaryHeap::with_capacity(max_cand + 1);
    for (idx, c) in cands.into_iter().enumerate() {
        let cs = cs_for(&c);
        let q = sync_quality_block0(&cs);
        let slot = Slot {
            q,
            idx: idx as u32,
            cand: c,
            cs,
        };
        if heap.len() < max_cand {
            heap.push(Reverse(slot));
        } else if let Some(Reverse(top)) = heap.peek()
            && slot.q > top.q
        {
            heap.pop();
            heap.push(Reverse(slot));
            // else branch: drop slot.cs immediately when it leaves scope
        }
    }
    let mut out: Vec<RefinedCandidate> = heap
        .into_iter()
        .map(|r| {
            let s = r.0;
            (s.cand, s.cs, s.q)
        })
        .collect();
    out.sort_by_key(|r| core::cmp::Reverse(r.2));
    out
}

/// Hard-decision sync quality on Costas **block 0 only** (symbols
/// 0..7). Cheaper variant of [`sync_quality`] for Pass 2 — checks
/// only one of the three Costas blocks. Range 0..=7.
///
/// Pub-but-doc-hidden so embedded callers (e.g. the m5stack-core2
/// PoC's manual Pass 2) can re-rank coarse_sync candidates by this
/// metric without pulling in the full `decode_block` D-pattern.
#[doc(hidden)]
pub fn sync_quality_block0<S: crate::engine::scalar::SpecScalar>(cs: &[[Cmplx<S>; 8]; 79]) -> u32
where
    S::Wide: PartialOrd,
{
    let mut count = 0u32;
    for (t, &expected) in COSTAS.iter().enumerate() {
        let sym = t; // block 0 starts at symbol 0
        let best = (0..NTONES)
            .max_by(|&a, &b| {
                let na = cs[sym][a].norm_sqr_wide();
                let nb = cs[sym][b].norm_sqr_wide();
                na.partial_cmp(&nb).unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap_or(0);
        if best == expected {
            count += 1;
        }
    }
    count
}

/// LLR / BP scalar for the hot loop. `Q11i16` (i16, ±16 range,
/// 1/2048 resolution, ~12 KB BP scratch on FT8 LDPC(174,91)) under
/// `fixed-point` (embedded integer pipeline); `f32` otherwise
/// (host / FPU targets). Both go through the same generic NMS
/// implementation in `fec::ldpc::bp`.
///
/// LlrT history:
/// - 0.5.x: `Q3i8` (i8, ±16, ~1/8 LSB resolution, ~6 KB BP scratch).
///   Issue #15 Phase 1 host-only sweep (2026-05-03) initially read
///   as recall-equivalent to `Q11i16`.
/// - 0.6.2 / 0.6.3: switched to `Q11i16`. The wider real-silicon
///   LX7 sweep showed the Q3i8 quantization step (~0.875 LLR units
///   between codes) was the dominant recall ceiling on Xtensa
///   builds — pre-0.6.3 host fixed-point + rustfft hit 16/18 with
///   f32 but only 9/18 with Q3i8 on `qso3_busy.wav` (the host f32
///   number later dropped to 13/18 in 0.6.3 when OSD tightening
///   removed 3 CRC-luck phantoms; the Q3i8-vs-f32 gap that
///   motivated the widening was measured before that). `Q11i16`'s
///   1/2048 resolution closes the LLR-resolution gap fully on
///   host (host fixed-point reaches f32-equivalent recall), but
///   on real silicon the embedded gain is only 1 entry — embedded
///   recall went 6/18 → 6/18 + 1 bonus = 7 total (XE2X HA2NP RR73),
///   not the ~10/18 the host sweep had projected. The remaining
///   headroom is blocked by other parts of the embedded pipeline
///   (NSTEP-half, coarse-sync simplifications, no `fine_refine_pass1`),
///   not by the LLR scalar itself. Cost: BP scratch doubles from
///   ~6 KB to ~12 KB, still inside the S3 / Core2 internal-DRAM
///   budget.
///
/// `Q3i8` stays in `engine::scalar` for the comparison path.
#[cfg(feature = "fixed-point")]
type LlrT = crate::engine::scalar::Q11i16;
#[cfg(not(feature = "fixed-point"))]
type LlrT = f32;

/// BP-kind switch (host-only). **Default `tanh`** (= WSJT-X
/// `bpdecode174_91.f90` log-domain tanh-product, our
/// `BpKind::SumProduct`) — that's the golden reference. The embedded
/// ship path keeps `NormalizedMinSum` (α=0.75) for speed; on host we
/// pay the tanh / atanh per-iteration cost in exchange for
/// numerically-correct convergence. The env var
/// `MFSK_BP_KIND=nms` opts into the approximation for A/B
/// comparison, but is NOT for production use — NMS appears to "find
/// more decodes" only because its approximation error sometimes
/// happens to land on a CRC-passing codeword that tanh would
/// (correctly) reject.
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
#[inline]
fn bp_step_select<T: crate::engine::scalar::LlrScalar>(
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<crate::fec::ldpc::Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<crate::fec::ldpc::bp::BpResult> {
    // `std::env::var` is a locked, allocating lookup — reading it fresh
    // on every call (up to 4x per candidate: Steps 1/d/b/c) measurably
    // added up across the hundreds of candidates a busy-band decode
    // processes. Cache the one-time result; this env var is a debug
    // A/B-comparison switch (see doc comment above), never expected to
    // change mid-process. `once_cell` isn't a dependency here — a
    // `std::sync::OnceLock<bool>` is sufficient and std-only.
    static USE_NMS: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *USE_NMS.get_or_init(|| std::env::var("MFSK_BP_KIND").as_deref() == Ok("nms")) {
        return crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<T>(
            bp_scratch, llr, None, max_iter, verify, NMS_ALPHA,
        );
    }
    let llr_f32: [f32; LDPC_N] = core::array::from_fn(|i| llr[i].to_f32());
    crate::fec::ldpc::bp::bp_decode(&llr_f32, None, max_iter, verify)
}

#[cfg(any(not(feature = "fft-rustfft"), feature = "fixed-point"))]
#[inline]
fn bp_step_select<T: crate::engine::scalar::LlrScalar>(
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<crate::fec::ldpc::Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<crate::fec::ldpc::bp::BpResult> {
    crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<T>(
        bp_scratch, llr, None, max_iter, verify, NMS_ALPHA,
    )
}

/// Stage 3: take Pass-2 refined candidates (cand + Costas-only cs +
/// sync_quality), fill in the data-symbol spectra, run LLR + BP/OSD
/// staircase. The Costas DFT was already done in Pass 2 — we only
/// add the data-symbol DFT here.
///
/// `q_thresh` is the post-fill `sync_quality` early-reject threshold
/// (see [`DEFAULT_Q_THRESH`]).
///
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn process_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
) -> Vec<DecodeResult> {
    process_candidates_tuned(audio, cands, depth, q_thresh, DEFAULT_BP_MAX_ITER)
}

/// Variant of [`process_candidates`] that accepts a runtime
/// `bp_max_iter` (= per-LLR-variant BP iteration cap). Embedded
/// LX6 / LX7 callers tune this to trade weak-signal recall for
/// post-SlotEnd time budget.
#[doc(hidden)]
pub fn process_candidates_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    process_candidates_tuned_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
        None,
        None,
    )
}

/// [`process_candidates_tuned`] with a per-candidate streaming
/// callback — fires `on_result` once per accepted candidate, in
/// candidate-processing order (this loop is always sequential, no
/// `parallel`/rayon involved on the embedded path), immediately after
/// the existing `results.push(r)` site inside
/// [`process_candidates_with_ap`] — so, unlike the host's parallel
/// `par_iter()` sites, every callback delivery here is guaranteed to
/// also appear in the returned `Vec`, in the same order.
///
/// `#[cfg(not(feature = "fft-rustfft"))]`: this exists solely to back
/// [`decode_block_streaming`], which is gated the same way — see that
/// function's doc comment for why.
#[cfg(not(feature = "fft-rustfft"))]
pub(crate) fn process_candidates_tuned_streaming<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    on_result: &mut dyn FnMut(&DecodeResult),
) -> Vec<DecodeResult> {
    process_candidates_tuned_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
        None,
        Some(on_result),
    )
}

/// AP-aware variant of [`process_candidates_tuned`]. When
/// `ap_hint = None` and `strictness = Normal`, behaviour is bit-
/// identical to `process_candidates_tuned`. Internal helper for the
/// `decode_block_with_ap` driver and host's redirected
/// `process_candidate`.
///
/// Owns a fresh [`BpScratch`](crate::fec::ldpc::bp::BpScratch) for
/// this call. Callers that invoke this once per candidate in a loop
/// (e.g. `decode_block_multipass`) should use
/// [`process_candidates_tuned_with_ap_scratch`] instead so the
/// scratch pool is reused across iterations (issue #199).
#[allow(clippy::too_many_arguments)]
pub(super) fn process_candidates_tuned_with_ap<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    fft_cache: Option<&[Complex<f32>]>,
    on_result: Option<&mut dyn FnMut(&DecodeResult)>,
) -> Vec<DecodeResult> {
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    process_candidates_tuned_with_ap_scratch(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        ap_hint,
        strictness,
        fft_cache,
        &mut bp_scratch,
        on_result,
    )
}

/// [`process_candidates_tuned_with_ap`] with a caller-owned
/// [`BpScratch`](crate::fec::ldpc::bp::BpScratch) — lets a per-candidate
/// outer loop (`decode_block_multipass`) reuse the scratch pool across
/// calls instead of paying its allocation cost on every candidate
/// (issue #199).
#[allow(clippy::too_many_arguments)]
pub(super) fn process_candidates_tuned_with_ap_scratch<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    fft_cache: Option<&[Complex<f32>]>,
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<
        crate::fec::ldpc::params::Ldpc174_91Params,
        LlrT,
    >,
    on_result: Option<&mut dyn FnMut(&DecodeResult)>,
) -> Vec<DecodeResult> {
    let mut cs_scratch: alloc::boxed::Box<[[Cmplx<f32>; 8]; 79]> =
        alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    process_candidates_with_ap(
        audio,
        &cands,
        depth,
        q_thresh,
        bp_max_iter,
        &mut cs_scratch,
        bp_scratch,
        |cs, cand, mask| {
            fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask, fft_cache);
        },
        ap_hint,
        strictness,
        on_result,
    )
}

/// Variant of [`process_candidates`] used by embedded fixed-point
/// callers.
///
/// **Pub for benchmarking + manually-staged callers** (e.g.
/// m5stack-core2 main.rs which logs per-stage wall-clock).
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
pub fn process_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
) -> Vec<DecodeResult> {
    process_candidates_into_tuned(audio, cands, depth, q_thresh, DEFAULT_BP_MAX_ITER)
}

/// Variant of [`process_candidates_into`] that accepts a runtime
/// `bp_max_iter`. Same role as [`process_candidates_tuned`] but for
/// the embedded fixed-point path.
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
pub fn process_candidates_into_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    let mut cs_scratch: alloc::boxed::Box<[[Cmplx<f32>; 8]; 79]> =
        alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    process_candidates_into_with_cs_scratch_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        &mut cs_scratch,
    )
}

/// Variant of [`process_candidates_into`] that also accepts a
/// caller-provided per-symbol-spectra scratch (`cs_scratch`, 5 KB =
/// `[[Cmplx<f32>; 8]; 79]`). Each candidate's PSRAM-resident `cs Box`
/// is copied into this scratch before LLR / BP run on it, then
/// dropped — letting the BP / LLR hot loops read `cs` from internal
/// DRAM (~5–10× faster than PSRAM on Xtensa LX6/LX7). Provide a
/// `static mut` array in `.bss` for max win.
///
/// Public as of v0.6 (#49 cat C): used by the embedded
/// `embedded-shared::dual_core::stage3_split` worker.
///
/// `#[doc(hidden)]`: embedded-internal scratch-reuse API, not part of
/// the supported host decode surface (issue #203) — stays `pub`
/// rather than `pub(crate)` because `embedded-poc` depends on this
/// crate as an external path dependency, not a workspace member.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn process_candidates_into_with_cs_scratch<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    process_candidates_into_with_cs_scratch_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        DEFAULT_BP_MAX_ITER,
        cs_scratch,
    )
}

/// Variant of [`process_candidates_into_with_cs_scratch`] that accepts
/// a runtime `bp_max_iter`. This is the entry point used by the
/// embedded `dual_core::stage3_split` worker so LX6 / LX7 binaries
/// can dial the BP cap without rebuilding `mfsk-core`.
///
/// Public as of v0.6 (#49 cat C).
///
/// `#[doc(hidden)]`: see [`process_candidates_into_with_cs_scratch`]
/// (issue #203) — stays `pub` since `embedded-shared::dual_core` (an
/// external path dependency) calls this directly.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn process_candidates_into_with_cs_scratch_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    process_candidates_with_ap(
        audio,
        &cands,
        depth,
        q_thresh,
        bp_max_iter,
        cs_scratch,
        &mut bp_scratch,
        |cs, cand, mask| {
            // Host fft-rustfft: cd0-based 32-pt FFT cs builder (=
            // ft8b.f90:154-161). WSJT-X-faithful; out-of-band signals
            // suppressed by the downsample's edge-tapered filter.
            #[cfg(feature = "fft-rustfft")]
            {
                // fft_cache=None here because process_candidates_into_tuned
                // is the embedded entry, not the slot-cache pipeline.
                fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask, None);
            }
            // Embedded (no fft-rustfft): f32 Goertzel recursion with
            // sample-outer / tone-inner loop ordering so the 8 per-tone
            // dependent chains run independently through the Xtensa
            // FPU pipeline (see `fill_symbol_spectra_goertzel` doc).
            // Zero scratch needed.
            #[cfg(not(feature = "fft-rustfft"))]
            fill_symbol_spectra_goertzel(cs, audio, cand.freq_hz, cand.dt_sec, mask);
        },
        None,
        DecodeStrictness::Normal,
        None,
    )
}

/// Caller-provided **fill closure** variant of
/// `process_candidates_into_with_cs_scratch_tuned`. The closure
/// builds the per-candidate per-mask `cs_scratch` entries instead of
/// the default `fill_symbol_spectra` path. Used by host research /
/// regression tests to compare alternative pass-2 fill algorithms.
///
/// The closure signature is
/// `FnMut(&mut [[Cmplx<f32>; 8]; 79], &SyncCandidate, SymMask)` —
/// fill only the cells matching `mask` (block-0 sync, block-1/2 sync,
/// data symbols) using the caller's preferred algorithm.
///
/// `#[doc(hidden)]`: see [`process_candidates_into_with_cs_scratch`]
/// (issue #203).
#[doc(hidden)]
#[allow(clippy::too_many_arguments)]
pub fn process_candidates_into_with_cs_scratch_tuned_with_fill<S, F>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
    fill: F,
) -> Vec<DecodeResult>
where
    S: AudioSample,
    F: FnMut(&mut [[Cmplx<f32>; 8]; 79], &SyncCandidate, SymMask),
{
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    process_candidates_with_ap(
        audio,
        &cands,
        depth,
        q_thresh,
        bp_max_iter,
        cs_scratch,
        &mut bp_scratch,
        fill,
        None,
        DecodeStrictness::Normal,
        None,
    )
}

/// Common body of `process_candidates` / `process_candidates_into`
/// / `process_candidates_into_with_cs_scratch` — the BP staircase
/// logic is identical between them; only the per-candidate
/// `fill_symbol_spectra(_into)` call differs (heap-allocated basis vs
/// caller-provided). `cs_scratch` is the per-symbol-spectra working
/// buffer that hot loops (BP / LLR / sync_quality) read from — see
/// [`process_candidates_into_with_cs_scratch`] for the rationale.
/// Per-candidate driver — dt is already parabolically refined by
/// coarse_sync. AP-aware: pass `ap_hint = None` for the legacy
/// non-AP behaviour (bit-identical to the pre-0.6.1 shape).
#[allow(clippy::too_many_arguments)]
fn process_candidates_with_ap<S: AudioSample, F>(
    _audio: &[S],
    cands: &[RefinedCandidate],
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<
        crate::fec::ldpc::params::Ldpc174_91Params,
        LlrT,
    >,
    mut fill: F,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    mut on_result: Option<&mut dyn FnMut(&DecodeResult)>,
) -> Vec<DecodeResult>
where
    F: FnMut(&mut [[Cmplx<f32>; 8]; 79], &SyncCandidate, SymMask),
{
    // dt is already parabolically refined by coarse_sync; no grid here.

    let mut results: Vec<DecodeResult> = Vec::new();
    // BP scratch pool — caller-owned (issue #199) so it can be reused
    // not just across this call's `cands` × all 5 BP calls per
    // candidate, but also across an outer per-candidate loop like
    // `decode_block_multipass`'s (which calls
    // `process_candidates_tuned_with_ap_scratch` once per candidate
    // with a 1-element cand list — see that function's caller for the
    // scratch's actual lifetime). Eliminates the ~12 KB-per-BP-call
    // `tlsf_malloc` traffic that dominated stage-3 non-DFT cost on
    // Core2 (~50–100 ms / qso). See `mfsk_core::fec::ldpc::bp::BpScratch`.
    for (cand, cs_box, _q_block0) in cands.iter() {
        // Stage cs into the caller's scratch (typically internal DRAM
        // on Xtensa) so the LLR / BP / sync_quality hot loops below
        // read from fast memory. Cost: ~60 µs (5 KB at ~80 MB/s OCT
        // PSRAM read on S3) vs many-hundred-µs gains in BP iter loop.
        // Slice-borrow form (PR #118 round-2): caller keeps ownership
        // of `cs_box`, enabling auto-AP to reuse the same refined
        // candidates across multiple callsigns without per-candidate
        // Box reallocation.
        *cs_scratch = **cs_box;
        // Two-step fill: sync blocks first, gate by full sync_quality,
        // then fill data symbols only for survivors. Saves the 58 ×
        // 8 = 464 data-symbol DFTs on every candidate that fails the
        // q gate (typically half of `max_cand`). `SyncBlocks12`
        // (instead of `SyncOnly`) skips re-filling block 0 — Pass 2
        // already populated it on `SyncBlock0`, and that data
        // survives in `cs_scratch` here. Saves an additional
        // 56 DFTs / candidate.
        fill(cs_scratch, cand, SymMask::SyncBlocks12);
        let q = sync_quality(cs_scratch);
        if q <= q_thresh {
            #[cfg(feature = "std")]
            TRACE_NSYNC_FAIL.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            continue;
        }
        #[cfg(feature = "std")]
        TRACE_NSYNC_PASS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        fill(cs_scratch, cand, SymMask::DataOnly);
        if let Some(r) = process_one_candidate_inner(
            cs_scratch,
            cand,
            cand.dt_sec,
            q,
            depth,
            bp_max_iter,
            bp_scratch,
            &results,
            ap_hint,
            strictness,
            0.0,
        ) {
            if let Some(cb) = on_result.as_mut() {
                cb(&r);
            }
            results.push(r);
        }
    }

    results
}

// WSJT-X-faithful nharderrors gate (ft8b.f90:422), formerly a flat
// `WSJTX_NHARDERRORS_MAX = 36` constant — now
// `DecodeStrictness::ft8_nharderrors_max` (issue #221, strictness-wiring
// follow-up to #220), consumed directly at each of the four BP-acceptance
// call sites below and at `osd_strategy::try_fallback`'s OSD gate.
// `Normal` still returns the same 36.

/// Minimum `sync_quality` (nsync, 0..=21) required to attempt the
/// always-on blind-CQ AP pass (Step 4 "Pass 12", issue #190). Real
/// WSJT-X's `sync8`'s own `syncmin` pre-filter keeps the candidate
/// count that ever reaches `ft8b`'s (always-on, no `ap_hint` needed)
/// iaptype-1 pass much smaller than mfsk-core's own coarse-sync +
/// wide `max_cand` lets through; without an equivalent filter here,
/// `decode_frame_subtract_staged`'s 3-checkpoint scan of
/// `qso3_busy.wav` sends 188 candidates through this pass (measured),
/// nearly all of them (140/188) sitting at nsync 7-9 — and **none**
/// of them, at any nsync value observed on that file, actually
/// produced a decode via this pass (all 22 decodes came from earlier
/// steps). The 3 trials issue #190 traced this fix against
/// (`ccir_moderate_m19_{01,05,14}.wav`) needed nsync 14/16/18 to
/// succeed. Gating at 12 — comfortably below that observed floor,
/// comfortably above the noise-dominated 7-9 cluster — cuts
/// `qso3_busy.wav`'s Step-4 candidate count from 188 to 34 (measured)
/// with zero recall change on any golden test, while leaving the
/// CCIR-sweep fix fully intact.
///
/// **FT4/FST4 analog**: this Pass 12 blind-CQ pass is FT8's own
/// bespoke equivalent of [`crate::msg::pipeline_ap::ap_passes`]'s
/// `pass 7` (CQ + DX call), which FT4/FST4 reach via
/// `msg::pipeline_ap`. Independent implementations, independently
/// tuned — review both when adjusting either (issue #192).
#[cfg(feature = "fft-rustfft")]
const BLIND_CQ_MIN_NSYNC: u32 = 12;

/// Per-candidate decode core — runs the LLR-staircase, OSD fallback,
/// and AP iaptype loop on a *fully-filled* `cs_scratch`. Shared
/// between the embedded `process_candidates_with` driver (above) and
/// the host `process_candidate` redirect (decode.rs, post-0.6.1).
/// Returns `Some(DecodeResult)` on first success (Step 1, 2, 3, or 4
/// in order), `None` on full failure.
///
/// Caller responsibilities:
/// - `cs_scratch` filled with all 79 symbols × 8 tones (data + sync).
/// - `q = sync_quality(cs_scratch)` already computed and gated.
/// - `refined_dt` carries the parabolically-refined `dt_sec`.
/// - `known` is the dedup list (in-progress + prior-pass results).
/// - `ap_hint` is `None` for AP-off (embedded default); `Some(_)` to
///   activate Step 4 AP iaptype loop. `#[cfg(feature = "fft-rustfft")]`
///   only — embedded fixed-point builds always pass `None`.
/// - `strictness` controls the BP-staircase/OSD `nharderrors` ceiling
///   ([`DecodeStrictness::ft8_nharderrors_max`], Steps 1-3, always
///   live) and the per-iaptype AP cap ([`DecodeStrictness::ap_max_errors`],
///   Step 4, live only when `ap_hint` is `Some(_)`).
/// - `sync_cv` is the Costas-array power CV (host computes it for QSB
///   gain attenuation; embedded passes 0.0).
#[allow(clippy::too_many_arguments)]
#[allow(unused_variables)] // ap_hint only used under #[cfg(feature = "fft-rustfft")]
#[allow(unused_assignments)] // llrb_arr/llrc_arr only read back under #[cfg(feature = "fft-rustfft")]
pub(in crate::ft8) fn process_one_candidate_inner(
    cs_scratch: &[[Cmplx<f32>; 8]; 79],
    cand: &SyncCandidate,
    refined_dt: f32,
    q: u32,
    depth: DecodeDepth,
    bp_max_iter: u32,
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<
        crate::fec::ldpc::params::Ldpc174_91Params,
        LlrT,
    >,
    known: &[DecodeResult],
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    sync_cv: f32,
) -> Option<DecodeResult> {
    // ── Staircase: cheap → deeper → OSD ─────────────────────────
    //
    // 1) Bp(llra) on the fast nsym=1 LLR. Most candidates that
    //    decode at all decode here; the rest fall through.
    // 2) Full compute_llr (nsym=1+2+3) → Bp on all 4 variants
    //    (a/b/c/d).
    // 3) OSD-1 / OSD-3 fallback gated on sync_quality.
    //
    // `BpAll` and `BpAllOsd` enable the deeper stages; plain
    // `Bp` stops after step 1.
    let mut accepted: Option<(crate::fec::ldpc::bp::BpResult, u8)> = None;

    // Step 1: fast llra. The LLR / BP scalar is selected at compile
    // time via `fixed-point` (Q11i16) or default (f32) — see the
    // `LlrT` definition above. Both go through the *same* generic
    // NMS implementation, bit-identical AWGN behaviour by design.
    let llr_a_fast: super::super::llr::LlrSet<LlrT> =
        super::super::llr::compute_llr_fast(cs_scratch);
    let bp_step1 =
        bp_step_select::<LlrT>(bp_scratch, &llr_a_fast.llra, bp_max_iter, Some(check_crc14));
    if let Some(bp) = bp_step1
        && bp.hard_errors <= strictness.ft8_nharderrors_max()
    {
        accepted = Some((bp, 0));
    }

    // Step 2: deeper-LLR variants. Lazy + LLR-shared with Step 1.
    //
    // **Variant a is skipped** — Step 1 already ran BP on the
    // identical input (`compute_llr_fast` and `compute_llr`
    // produce bit-identical `llra`, since nsym=1 work doesn't
    // depend on `max_nsym`). Re-running it would be guaranteed
    // failure.
    //
    // **Variant d reuses** Step 1's `llr_a_fast.llrd` — same
    // nsym=1 derivation, costs zero LLR work.
    //
    // **Variants b / c are lazy-computed**: only pay the nsym=2
    // work if d failed, and only pay the heavy nsym=3 work
    // (~80 % of `compute_llr`) if both d and b also failed.
    // Order chosen by ascending compute cost — same number of BP
    // calls as the old variant loop in the worst case, far fewer
    // in the typical case where any earlier variant decodes.
    // Per-variant gates. `d` is cheap (Step-1 LLR re-use, BP only) and
    // unconditional — every `LlrEffort` tier ran it even before this
    // struct existed, so there was never a real third state here.
    // `b`/`c` add nsym=2 / nsym=3 LLR work on top of the BP.
    let run_d = true;
    let (run_b, run_c) = match depth.llr_effort {
        LlrEffort::Minimal => (false, false),
        LlrEffort::Full => (true, true),
    };

    if accepted.is_none() && run_d {
        // Variant d: free reuse of Step 1's llrd.
        let bp_d =
            bp_step_select::<LlrT>(bp_scratch, &llr_a_fast.llrd, bp_max_iter, Some(check_crc14));
        if let Some(bp) = bp_d
            && bp.hard_errors <= strictness.ft8_nharderrors_max()
        {
            accepted = Some((bp, 3));
        }
    }
    // Variant b: lazy nsym=2 only. Kept outside the `if` (as
    // `Option`) so a `BpAllOsd` candidate that falls through to Step 3
    // can reuse it — see the `prefetched_llr` assembly below. Only
    // read back under `fft-rustfft` (that assembly is gated on it);
    // `fft-extern`/no_std builds compute and discard it — see the
    // function's own `#[allow(unused_assignments)]`.
    let mut llrb_arr: Option<[LlrT; LDPC_N]> = None;
    if accepted.is_none() && run_b {
        let arr: [LlrT; LDPC_N] = super::super::llr::compute_llr_partial::<LlrT>(cs_scratch, 2);
        let bp_b = bp_step_select::<LlrT>(bp_scratch, &arr, bp_max_iter, Some(check_crc14));
        if let Some(bp) = bp_b
            && bp.hard_errors <= strictness.ft8_nharderrors_max()
        {
            accepted = Some((bp, 1));
        }
        llrb_arr = Some(arr);
    }
    // Variant c: lazy nsym=3 (the expensive one). Gated to
    // `LlrEffort::Full` — `LlrEffort::Minimal` skips it as the
    // embedded post-SlotEnd dominant cost (~5× variant `b`) on
    // busy-band reference WAVs (see `LlrEffort`'s doc comment). Same
    // `Option`-hoist (and `fft-rustfft`-only readback) as `llrb_arr`
    // above, for the same reason.
    let mut llrc_arr: Option<[LlrT; LDPC_N]> = None;
    if accepted.is_none() && run_c {
        let arr: [LlrT; LDPC_N] = super::super::llr::compute_llr_partial::<LlrT>(cs_scratch, 3);
        let bp_c = bp_step_select::<LlrT>(bp_scratch, &arr, bp_max_iter, Some(check_crc14));
        if let Some(bp) = bp_c
            && bp.hard_errors <= strictness.ft8_nharderrors_max()
        {
            accepted = Some((bp, 2));
        }
        llrc_arr = Some(arr);
    }

    // Pre-compute the f32 `LlrSet` OSD needs, reusing Steps 1-2's own
    // llra/llrd/llrb/llrc instead of a fresh `compute_llr(cs_scratch)`
    // call whenever possible (issue #182 follow-up). For `BpAllOsd`
    // depth, `run_b`/`run_c` are unconditionally `true`, so if we've
    // reached this point with `accepted.is_none()` then *both*
    // `llrb_arr` and `llrc_arr` above were computed — nothing here was
    // ever skipped by an earlier success (that would have short-
    // circuited `accepted` to `Some` already). On the non-`fixed-point`
    // host build `LlrT = f32`, so `llr_a_fast`/`llrb_arr`/`llrc_arr`
    // are *already* the exact `f32` values OSD needs — no recompute.
    //
    // Previously this only ever fired `if ap_hint.is_some()` (Gemini PR
    // #81 review — avoiding a *second* `compute_llr` when both OSD and
    // the AP loop would otherwise each compute their own), which meant
    // blind decode (`ap_hint = None`, the common case) always paid for
    // a full redundant `compute_llr(cs_scratch)` recompute inside
    // `try_fallback` itself — nsym=3 alone is ~80% of that call's cost,
    // paid twice (once here piecewise, once again wholesale) for every
    // single candidate that reached OSD. Measured on `qso3_busy.wav`
    // blind decode: this was a large fraction of OSD's real-world cost
    // that a synthetic-LLR microbenchmark (which calls `osd_setup`/
    // `osd_npre1_pass` directly, bypassing this recompute entirely)
    // couldn't see.
    #[cfg(feature = "fft-rustfft")]
    let prefetched_llr: Option<super::super::llr::LlrSet<f32>> =
        if accepted.is_none() && depth.osd && q > 6 {
            #[cfg(not(feature = "fixed-point"))]
            {
                match (llrb_arr, llrc_arr) {
                    (Some(llrb), Some(llrc)) => Some(super::super::llr::LlrSet {
                        llra: llr_a_fast.llra,
                        llrb,
                        llrc,
                        llrd: llr_a_fast.llrd,
                    }),
                    // Defensive fallback — shouldn't happen when
                    // `depth.osd` (see reasoning above), but a fresh
                    // compute is still correct if it ever does.
                    _ => Some(super::super::llr::compute_llr(cs_scratch)),
                }
            }
            #[cfg(feature = "fixed-point")]
            {
                // Steps 1-2's llrb_arr/llrc_arr are Q11i16 here, not
                // directly reusable as the f32 LlrSet OSD needs —
                // still must recompute.
                Some(super::super::llr::compute_llr(cs_scratch))
            }
        } else {
            None
        };
    #[cfg(not(feature = "fft-rustfft"))]
    let prefetched_llr: Option<super::super::llr::LlrSet<f32>> = None;

    // Step 3: OSD fallback (sync_quality gated; only when `depth.osd`).
    // The actual dispatch — including the WSJT-X-faithfulness
    // deviation #63 tracks restoring — lives in `super::osd_strategy`
    // so #63 can patch it without touching the rest of this function.
    // Host-only: `osd_strategy` itself is `#[cfg(feature =
    // "fft-rustfft")]`-gated (`DecodeDepth::osd` is a no-op on
    // embedded — see its doc comment in `ft8::decode`), so the call
    // is gated the same way here rather than relying on `depth.osd`
    // alone to keep it unreachable.
    #[cfg(feature = "fft-rustfft")]
    if accepted.is_none() {
        #[cfg(feature = "std")]
        if depth.osd && q > 6 {
            TRACE_OSD_ATTEMPT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        accepted = super::osd_strategy::try_fallback(
            cs_scratch,
            prefetched_llr.as_ref(),
            depth,
            q,
            strictness,
        );
    }

    // Step 4: AP iaptype loop (host f32 only; gated on fft-rustfft).
    // Mirrors WSJT-X ft8b.f90 ipass 5..12 — runs only if Steps 1-3 all
    // failed. Embedded fixed-point builds always pass `ap_hint = None`
    // and never reach here (no `fft-rustfft`), so this block is a
    // host-only cost.
    //
    // Pass 12 (blind CQ, WSJT-X iaptype 1) always runs, independent of
    // `ap_hint` — this is a faithful port, not an mfsk-core extension:
    // `ft8b.f90`'s `naptypes(0,1:4)=(1,2,0,0)` tries iaptype=1 for
    // *every* idle-state (`nQSOProgress=0`) candidate, using the fixed
    // 29-bit "CQ" callsign pattern (`mcq`) as its AP prior — no
    // operator-supplied mycall/hiscall needed, unlike iaptype≥2. Real
    // `jt9 -8 -d3` (`lib/jt9.f90:302`, `lft8apon=.true.` hardcoded in
    // the CLI tool) always includes this pass; prior to this fix,
    // mfsk-core's equivalent only fired when the *caller* supplied an
    // `ap_hint`, so a plain blind `decode_frame` call (`ap_hint: None`,
    // e.g. `tests/ft8_sweep.rs`'s SNR-crossing measurement) never
    // attempted it. Traced directly against a real jt9 build on the 3
    // `ccir_moderate_m19_{01,05,14}.wav` trials from issue #190: nsync
    // matches jt9 almost exactly (14/16/18 vs jt9's own 14/16/18) and
    // ipass 1-4 (blind BP/OSD) fail identically on both sides — the
    // decode only succeeds via jt9's ipass 5 (iaptype=1, nharderrors
    // 27-35, well above the blind BP/OSD ceiling but within
    // `decode174_91`'s `nharderrors.le.36` AP acceptance). The
    // CCIR-moderate/poor sensitivity gap issue #190 measured is this
    // missing pass, not a numerical LLR/sync precision deficiency —
    // AWGN/CCIR-good were unaffected because blind decode alone already
    // clears those (higher-SNR) thresholds.
    //
    // Iaptype priority (deepest-first, mirroring decode.rs:634-684):
    //   9/10/11 (call1+call2+RRR/RR73/73 → full 77-bit lock)
    //   7 (CQ + call2)
    //   8 (call1+call2 → ~61 bits)
    //   6 (ap as-supplied → ~33 bits)
    //   5 (mycall only → ~32 bits, WSJT-X iaptype 2)
    //   12 (blind CQ → always tried last, WSJT-X iaptype 1)
    #[cfg(feature = "fft-rustfft")]
    if accepted.is_none() {
        // Reuse the pre-computed LLR from above if it ran; otherwise
        // compute fresh. The unwrap_or_else only fires when the
        // pre-compute gate was `false` (BpAll with no OSD) but AP
        // still ran somehow — defensive, but not the dominant path.
        let llr_full_f32: super::super::llr::LlrSet<f32> =
            prefetched_llr.unwrap_or_else(|| super::super::llr::compute_llr(cs_scratch));
        let apmag = llr_full_f32
            .llra
            .iter()
            .map(|v| v.abs())
            .fold(0.0f32, f32::max)
            * 1.01;
        let llr_variants: [&[f32; LDPC_N]; 4] = [
            &llr_full_f32.llra,
            &llr_full_f32.llrb,
            &llr_full_f32.llrc,
            &llr_full_f32.llrd,
        ];

        let mut ap_passes: alloc::vec::Vec<(ApHint, u8)> = alloc::vec::Vec::new();
        if let Some(ap) = ap_hint
            && ap.has_info()
        {
            if ap.call1.is_some() && ap.call2.is_some() {
                for (rpt, pid) in [("RRR", 9u8), ("RR73", 10), ("73", 11)] {
                    let ap_full = ap.clone().with_report(rpt);
                    ap_passes.push((ap_full, pid));
                }
            }
            if ap.call2.is_some() && ap.call1.is_none() {
                let ap7 = ap.clone().with_call1("CQ");
                ap_passes.push((ap7, 7));
            }
            if ap.call1.is_some() && ap.call2.is_some() {
                ap_passes.push((ap.clone(), 8));
            }
            ap_passes.push((ap.clone(), 6));
            if ap.call1.is_some() {
                let mut ap5 = ApHint::new();
                if let Some(ref c1) = ap.call1 {
                    ap5 = ap5.with_call1(c1);
                }
                ap_passes.push((ap5, 5));
            }
        }
        // Pass 12: blind-CQ (WSJT-X iaptype 1) — tried regardless of
        // `ap_hint` (see the module comment above), gated on nsync to
        // bound the cost of scanning it over every failing candidate
        // in a busy-band multipass decode (see `BLIND_CQ_MIN_NSYNC`'s
        // doc comment).
        if q >= BLIND_CQ_MIN_NSYNC {
            ap_passes.push((ApHint::new().with_call1("CQ"), 12));
        }

        'ap_outer: for (ap_cfg, pass_id) in &ap_passes {
            // `ApHint::build_bits` (canonical, `crate::msg::ap`) returns
            // dynamically-sized `Vec<u8>` mask/value bits rather than the
            // old FT8-local `build_ap`'s `[bool; LDPC_N]`/`[f32; LDPC_N]`
            // arrays with the ±apmag magnitude already baked in — apply
            // that mapping here instead. Same underlying `pack28`/
            // `pack_grid4` and bit layout, so this is behavior-preserving.
            let (ap_mask_bits, ap_values) = ap_cfg.build_bits(LDPC_N);
            let mut ap_mask = [false; LDPC_N];
            for (dst, &src) in ap_mask.iter_mut().zip(ap_mask_bits.iter()) {
                *dst = src != 0;
            }
            let locked_bits = ap_mask.iter().filter(|&&m| m).count();
            let max_errors: u32 = strictness.ap_max_errors(locked_bits);

            for &base_llr in &llr_variants {
                let mut llr_ap = *base_llr;
                // Iterator form (issue #208-style — same shape as
                // `fill_bmet_for_nsym`'s max-reduction fix) instead of
                // three separate bounds-checked `[i]` indexes; run up
                // to ~28x per Step-4 candidate (4 LLR variants × up to
                // ~7 AP passes).
                for ((dst, &locked), &val) in
                    llr_ap.iter_mut().zip(ap_mask.iter()).zip(ap_values.iter())
                {
                    if locked {
                        *dst = if val == 1 { apmag } else { -apmag };
                    }
                }
                // Inline AP-result validator: hard-error gate, unpack,
                // plausibility, locked-call substring check.
                let validate = |msg77: [u8; 77], hard_errors: u32| -> bool {
                    if hard_errors >= max_errors {
                        return false;
                    }
                    let Some(text) = unpack77(&msg77) else {
                        return false;
                    };
                    if !crate::msg::wsjt77::is_plausible_message(&text) {
                        return false;
                    }
                    let upper = text.to_uppercase();
                    if let Some(ref c1) = ap_cfg.call1
                        && !upper.contains(&c1.to_uppercase())
                    {
                        return false;
                    }
                    if let Some(ref c2) = ap_cfg.call2
                        && !upper.contains(&c2.to_uppercase())
                    {
                        return false;
                    }
                    true
                };

                // AP + BP
                if let Some(bp) = crate::fec::ldpc::bp::bp_decode(
                    &llr_ap,
                    Some(&ap_mask),
                    bp_max_iter,
                    Some(check_crc14),
                ) && validate(bp.message77, bp.hard_errors)
                {
                    accepted = Some((bp, *pass_id));
                    break 'ap_outer;
                }
                // AP + OSD-Deep fallback (`depth.osd` only)
                if depth.osd
                    && let Some(osd) = osd_decode_deep(&llr_ap, 2, Some(check_crc14))
                    && validate(osd.message77, osd.hard_errors)
                {
                    // Reuse `osd.codeword` — `OsdResult` already
                    // carries the decoded bits; the previous
                    // `vec![0; LDPC_N]` was both wasteful and dropped
                    // the real codeword on the floor (Gemini PR #86
                    // review).
                    let bp = crate::fec::ldpc::bp::BpResult {
                        message77: osd.message77,
                        info: osd.info,
                        codeword: osd.codeword,
                        hard_errors: osd.hard_errors,
                        iterations: 0,
                    };
                    accepted = Some((bp, *pass_id));
                    break 'ap_outer;
                }
            }
        }
    }

    let (bp, pass_id) = accepted?;
    let text = unpack77(&bp.message77)?;
    // Plausibility filter — reject CRC-passing-but-garbage
    // messages. With max_cand=200 × 4 LLR variants × OSD,
    // CRC-14's 1/16384 false-positive rate produces ~1-2 random
    // strings per slot. Same filter the host wide-band path
    // uses (`decode_frame::process_candidate`).
    if !crate::msg::wsjt77::is_plausible_message(&text) {
        return None;
    }
    if known.iter().any(|r| r.message77() == bp.message77) {
        return None;
    }
    let itone = message_to_tones(&bp.message77);
    let snr_db = super::super::llr::compute_snr_db(cs_scratch, &itone);
    Some(DecodeResult {
        info: bp.info.into_boxed_slice(),
        freq_hz: cand.freq_hz,
        dt_sec: refined_dt,
        hard_errors: bp.hard_errors,
        sync_score: cand.score,
        pass: pass_id,
        sync_cv,
        snr_db,
    })
}
