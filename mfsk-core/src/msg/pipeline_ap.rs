//! AP-assisted decode pipeline for WSJT 77-bit-family protocols.
//!
//! Builds on `mfsk-engine::pipeline` to add multi-pass AP (a-priori) hints: the
//! caller supplies known portions of the expected message (callsigns, grid,
//! response) and the decoder tries several configurations with those bits
//! clamped to high-confidence LLRs. Because the 77-bit bit layout is shared
//! across FT8 / FT4 / FT2 / FST4, this code is protocol-agnostic via the
//! `P: Protocol` bound plus the `P::Msg = Wsjt77Message` convention.
//!
//! Typical threshold improvement is 2–4 dB when both call1 and call2 are
//! known (CQ + DX scenario) and can exceed that when a specific response
//! token (RRR / RR73 / 73) is also locked.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::dsp::downsample::{DownsampleCfg, build_fft_cache};
use crate::engine::equalize::{EqMode, equalize_local};
use crate::engine::llr::{compute_llr_fast, compute_llr_partial, symbol_spectra, sync_quality};
use crate::engine::pipeline::{
    DecodeDepth, DecodeResult, DecodeStrictness, GenericPipelineProtocol, SnrCtx,
    refine_candidate_position,
};
use crate::engine::sync::{SyncCandidate, coarse_sync, fine_sync_power_per_block};
use crate::engine::tx::codeword_to_itone;
use crate::engine::{FecCodec, FecOpts, Protocol};

use super::ap::{ApHint, WsjtApCompatible};
use super::wsjt77::{is_plausible_message, unpack77};

/// Build one AP configuration: derive the mask/values bit vectors from a
/// hint for this protocol's codeword length. Convenience for callers that
/// want to try several hint shapes (full lock, partial lock, …).
///
/// Bound on [`WsjtApCompatible`] keeps callers honest: the hint encodes
/// callsign / grid / report at fixed Wsjt77 bit positions and is meaningless
/// for protocols whose info layout differs (e.g. byte-oriented codecs).
pub(crate) fn ap_bits_for<P: Protocol>(hint: &ApHint) -> (Vec<u8>, Vec<u8>)
where
    P::Msg: WsjtApCompatible,
{
    hint.build_bits(P::Fec::N)
}

/// Enumerate the multi-pass AP configurations WSJT-X cycles through in
/// sniper mode — the `u8` is a pass-id tag for diagnostics.
///
/// - 9/10/11: full 77-bit lock with `RRR` / `RR73` / `73` (QSO in progress).
/// - 7:       CQ + DX call (expected "CQ DXCALL GRID").
/// - 8:       my-call + DX call (directed message).
/// - 6:       DX call only (partial lock, fallback).
///
/// **FT8 analog for pass 7**: `ft8::decode_block::process_candidates`'s
/// own blind-CQ `Pass 12` (gated by `BLIND_CQ_MIN_NSYNC`) is FT8's
/// bespoke equivalent, independently implemented and tuned — review
/// both when adjusting either (issue #192).
pub(crate) fn ap_passes(base: &ApHint) -> Vec<(ApHint, u8)> {
    let mut passes = Vec::new();
    if base.call1.is_some() && base.call2.is_some() {
        for (rpt, pid) in [("RRR", 9u8), ("RR73", 10), ("73", 11)] {
            passes.push((base.clone().with_report(rpt), pid));
        }
    }
    if base.call2.is_some() && base.call1.is_none() {
        passes.push((base.clone().with_call1("CQ"), 7));
    }
    if base.call1.is_some() && base.call2.is_some() {
        passes.push((base.clone(), 8));
    }
    passes.push((base.clone(), 6));
    passes
}

/// Decode a single candidate with AP hints. Returns the first successful
/// AP pass, or falls back to a plain BP/OSD decode (no AP) to catch
/// already-clear signals.
///
/// `P::Msg: WsjtApCompatible` gates this function to protocols whose
/// 77-bit message layout matches the Wsjt77 family — `ApHint` writes
/// call1/call2/grid bits at hardcoded positions that would be nonsense
/// for a different layout.
pub(crate) fn process_candidate_ap<P: GenericPipelineProtocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
    ap_hint: Option<&ApHint>,
) -> Option<DecodeResult>
where
    P::Fec: crate::engine::protocol::BpPooledFec,
    P::Msg: WsjtApCompatible,
{
    let ds_rate = 12_000.0 / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;
    let _ = refine_steps; // superseded by refine_candidate_position's own P-specific search below

    // FT4/FST4's own 2-D (frequency + time) coherent refine
    // (`refine_candidate_position`, `engine::sync2d::ft4_sync_search`/
    // `fst4_sync_search`), matching `engine::pipeline::
    // process_candidate_basic_impl`'s wide-band path exactly (issue
    // #255 follow-up, WebFT8 downstream report). This AP/sniper path
    // previously used the generic, time-only `refine_candidate` (no
    // frequency correction at all) plus a raw, non-RMS-normalised
    // `cd0` — this closes both gaps.
    //
    // A companion fix (swapping this path's *coarse* search from
    // generic `coarse_sync` to `ft4_coarse_sync`, matching wide-band's
    // own `getcandidates4.f90`-faithful mechanism, so `ft4_snr_db`
    // would receive the score formula it actually expects) was tried
    // and reverted: `ft4_coarse_sync`'s coarse-frequency estimate is
    // only accurate to roughly its own 15-bin (~78 Hz) smoothing
    // width — fine for wide-band search, where a real signal usually
    // has *some* candidate close enough for this function's own
    // ±12 Hz fine-frequency refine to lock onto, but demonstrably not
    // for a narrow, single-target sniper search: a real synthetic
    // regression (`ft4_streaming_sniper_matches_batch_exactly`) showed
    // `ft4_coarse_sync` locking a clean 1000 Hz signal's only
    // candidates 60+ Hz away, outside that refine range, losing the
    // decode entirely. `cand.score` reaching `SnrCtx` in this path is
    // therefore still `coarse_sync`'s Costas-correlation score, not
    // `getcandidates4.f90`'s own value `ft4_snr_db` was written
    // against — a known, deliberately-not-fixed SNR-accuracy gap on
    // this path specifically (issue #255 follow-up discussion).
    let (cd0, refined_freq_hz, i_start, refined_score) =
        refine_candidate_position::<P>(cand, fft_cache, ds_cfg);
    let refined = SyncCandidate {
        freq_hz: refined_freq_hz,
        dt_sec: (i_start as f32) / ds_rate - tx_start,
        score: refined_score,
    };
    let cs_raw = symbol_spectra::<P>(&cd0, i_start);
    let nsync = sync_quality::<P>(&cs_raw);
    if nsync <= sync_q_min {
        return None;
    }

    let per_block = fine_sync_power_per_block::<P>(&cd0, i_start);
    let sync_cv = if !per_block.is_empty() {
        let n = per_block.len() as f32;
        let mean = per_block.iter().sum::<f32>() / n;
        if mean > f32::EPSILON {
            (per_block.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n).sqrt() / mean
        } else {
            0.0
        }
    } else {
        0.0
    };

    let fec = P::Fec::default();

    // Prepare EQ / non-EQ views of the symbol spectra. The AP-list
    // path is EQ-only (matches FT8's historical single-path approach);
    // the non-EQ fallback (~1/20 extra decodes at -18 dB) was retired
    // because it doubled per-candidate cost for marginal gain.
    let cs_eq = {
        let mut v = cs_raw.clone();
        equalize_local::<P>(&mut v);
        v
    };
    let try_order: &[(&[Complex<f32>], bool)] = match eq_mode {
        EqMode::Off => &[(&cs_raw, false)],
        EqMode::Local => &[(&cs_eq, true)],
    };

    for (cs_ref, _used_eq) in try_order {
        let cs_ref: &[Complex<f32>] = cs_ref;

        // Lazy nsym staircase (issue #199 follow-up): build each LLR
        // variant only as this loop reaches it, instead of eagerly
        // computing the whole `LlrSet` (nsym=1, 2, `LLR_NSYM_MAX`) up
        // front regardless of whether a cheap variant already lets
        // plain BP succeed. Mirrors the lazy staircase
        // `engine::pipeline::process_candidate_basic` has had since
        // commit `4801722` (issue #197 item 2) — this AP-path sibling
        // never got the same port despite sharing the exact same
        // motivation (FST4's `LLR_NSYM_MAX=8` rung enumerates
        // `4^8=65536` tone-combination hypotheses per group, 128-256x
        // FT8/FT4's own deepest rung). If every plain-BP attempt below
        // fails (the common case whenever an AP-assisted pass is what
        // actually succeeds), `llr_set` ends up fully populated exactly
        // once by the time the AP loop runs — same total cost as the
        // previous eager version, so this is a pure win with no
        // regression on the AP-success path.
        let mut llr_set = compute_llr_fast::<P, f32>(cs_ref);
        macro_rules! try_plain_bp {
            ($llr:expr, $pass_id:expr) => {
                let bp_opts = FecOpts {
                    bp_max_iter: 30,
                    osd_depth: 0,
                    ap_mask: None,
                    verify_info: Some(<P::Msg as crate::engine::MessageCodec>::verify_info),
                    ..FecOpts::default()
                };
                if let Some(r) = fec.decode_soft($llr, &bp_opts)
                    && let Some(res) = finalise_result::<P>(
                        &r, cand, &refined, sync_cv, $pass_id, cs_ref, None, &fec, fft_cache,
                        ds_cfg, i_start,
                    )
                {
                    return Some(res);
                }
            };
        }
        try_plain_bp!(&llr_set.llra, 0u8);
        llr_set.llrb = compute_llr_partial::<P, f32, f32>(cs_ref, 2);
        try_plain_bp!(&llr_set.llrb, 1u8);
        llr_set.llrc = compute_llr_partial::<P, f32, f32>(cs_ref, P::LLR_NSYM_MAX as usize);
        try_plain_bp!(&llr_set.llrc, 2u8);
        try_plain_bp!(&llr_set.llrd, 3u8);

        let variants = [
            (&llr_set.llra, 0u8),
            (&llr_set.llrb, 1),
            (&llr_set.llrc, 2),
            (&llr_set.llrd, 3),
        ];

        // ── AP-assisted passes ─────────────────────────────────────────
        //
        // Integer-timing retry (±2 downsampled samples around the
        // refined peak) was measured to deliver zero threshold
        // improvement at 5× runtime — the -18 dB floor is LLR-dominated,
        // not timing-dominated. See snr_sweep bench history 2026-04-18.
        if let Some(hint) = ap_hint
            && hint.has_info()
        {
            for (ap_cfg, pass_id) in ap_passes(hint) {
                let (mask, values) = ap_bits_for::<P>(&ap_cfg);
                let locked = mask.iter().filter(|&&m| m != 0).count();
                let max_errors = strictness.ap_max_errors(locked);

                for (llr, _) in &variants {
                    let ap_opts = FecOpts {
                        bp_max_iter: 30,
                        osd_depth: 0,
                        ap_mask: Some((&mask, &values)),
                        verify_info: Some(<P::Msg as crate::engine::MessageCodec>::verify_info),
                        ..FecOpts::default()
                    };
                    if let Some(r) = fec.decode_soft(llr, &ap_opts)
                        && r.hard_errors < max_errors
                        && let Some(res) = finalise_result::<P>(
                            &r,
                            cand,
                            &refined,
                            sync_cv,
                            pass_id,
                            cs_ref,
                            Some(&ap_cfg),
                            &fec,
                            fft_cache,
                            ds_cfg,
                            i_start,
                        )
                    {
                        return Some(res);
                    }
                    if depth.osd {
                        // OSD depth-2 only (matches FT8's AP path).
                        let depths: &[u32] = &[2];
                        let _ = locked;
                        for &od in depths {
                            let osd_opts = FecOpts {
                                bp_max_iter: 30,
                                osd_depth: od,
                                ap_mask: Some((&mask, &values)),
                                verify_info: Some(
                                    <P::Msg as crate::engine::MessageCodec>::verify_info,
                                ),
                                ..FecOpts::default()
                            };
                            if let Some(r) = fec.decode_soft(llr, &osd_opts)
                                && r.hard_errors < max_errors
                                && let Some(res) = finalise_result::<P>(
                                    &r,
                                    cand,
                                    &refined,
                                    sync_cv,
                                    pass_id,
                                    cs_ref,
                                    Some(&ap_cfg),
                                    &fec,
                                    fft_cache,
                                    ds_cfg,
                                    i_start,
                                )
                            {
                                return Some(res);
                            }
                        }
                    }
                }
            }
        }
    }

    None
}

fn finalise_result<P: GenericPipelineProtocol>(
    fec_result: &crate::engine::FecResult,
    cand: &SyncCandidate,
    refined: &SyncCandidate,
    sync_cv: f32,
    pass_id: u8,
    cs: &[Complex<f32>],
    ap_cfg: Option<&ApHint>,
    fec: &P::Fec,
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    i_start: i32,
) -> Option<DecodeResult>
where
    P::Fec: crate::engine::protocol::BpPooledFec,
{
    // FT4 pre-LDPC scramble (WSJT-X `genft4.f90:64`): undo the rvec
    // XOR on the 77 message bits before unpacking text. SNR re-
    // encode below still uses the *scrambled* `fec_result.info`
    // because that's what the on-air codeword carried.
    let mut info_unscrambled = fec_result.info.clone();
    crate::engine::llr::descramble_info::<P>(&mut info_unscrambled);
    let msg77: [u8; 77] = info_unscrambled[..77].try_into().ok()?;
    let text = unpack77(&msg77)?;
    if text.is_empty() || !is_plausible_message(&text) {
        return None;
    }
    // If this result came from an AP pass, verify the locked callsigns
    // actually appear in the decoded text — guards against spurious decodes
    // where the FEC happened to accept with the bits clamped.
    if let Some(ap) = ap_cfg {
        let upper = text.to_uppercase();
        if let Some(ref c1) = ap.call1
            && !upper.contains(&c1.to_uppercase())
        {
            return None;
        }
        if let Some(ref c2) = ap.call2
            && !upper.contains(&c2.to_uppercase())
        {
            return None;
        }
    }

    // Re-encode to compute a WSJT-X compatible SNR. After Phase A the
    // FEC's `r.info` already carries the K-bit info (message + CRC bits
    // that `MessageCodec::verify_info` already accepted), so feeding it
    // straight back through `fec.encode` reproduces the same codeword as
    // the previous "extract msg77 → recompute CRC → encode" path —
    // bit-identical because verifier acceptance enforces
    // `info[77..K] == crc(info[..77])` at the moment of acceptance.
    let mut cw = vec![0u8; P::Fec::N];
    fec.encode(&fec_result.info, &mut cw);
    let itone = codeword_to_itone::<P>(&cw);
    let snr_db = P::snr_db(SnrCtx {
        cs,
        itone: &itone,
        cand_score: cand.score,
        cand_freq_hz: cand.freq_hz,
        fft_cache,
        ds_cfg,
        refined_freq_hz: refined.freq_hz,
        i_start,
    });

    Some(DecodeResult {
        info: info_unscrambled.into_boxed_slice(),
        freq_hz: cand.freq_hz,
        dt_sec: refined.dt_sec,
        hard_errors: fec_result.hard_errors,
        sync_score: refined.score,
        pass: pass_id,
        sync_cv,
        snr_db,
    })
}

/// Sniper-mode decode with AP hints: search within `±search_hz` of
/// `target_freq`, with optional AP bit-locking applied per candidate.
///
/// `P::Msg: WsjtApCompatible` mirrors [`process_candidate_ap`]'s bound:
/// the underlying AP path writes to Wsjt77 bit positions and only makes
/// sense for protocols whose 77-bit message field shares that layout.
///
/// **`search_hz` is load-bearing for reported SNR, not just for
/// recall.** Both callers pass `250.0`, so the search spans 500 Hz —
/// and `coarse_sync` draws its 40th-percentile noise reference from
/// exactly this window, which is what `SyncCandidate::score`, and
/// therefore `Ft4`'s `snr_db`, is normalised against. 500 Hz centred
/// on the operator's aim point is also the roofing-filter passband a
/// sniper deployment is premised on (the operator tuned the rig so the
/// target sits in it), so the noise floor gets estimated over real
/// noise rather than over filter stopband. Widening this would start
/// averaging in stopband and bias SNR high; narrowing it would let
/// FT4's own 83.3 Hz occupied bandwidth contaminate the percentile.
/// Measured both directions in `docs/notes/SNR_FORMULAS.md`
/// ("Band-limited (roofing-filtered) input") — change it only with
/// that table re-measured.
#[allow(clippy::too_many_arguments)]
// Only `ft4::decode`/`fst4::decode` call this (issue #203's pub(crate)
// demotion made that reachability-dependent-on-feature visible to
// rustc): dead code under any feature combination that excludes both
// `ft4` and `fst4` (`jt9`/`jt65`/`q65`-only, etc). `#[allow(dead_code)]`
// here also covers this function's own private callees
// (`ap_bits_for`/`ap_passes`/`process_candidate_ap`/`finalise_result`),
// which would otherwise separately warn once this entry point is
// unreachable.
#[allow(dead_code)]
pub(crate) fn decode_sniper_ap<P: GenericPipelineProtocol>(
    audio: &[i16],
    ds_cfg: &DownsampleCfg,
    target_freq: f32,
    search_hz: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
    ap_hint: Option<&ApHint>,
    // Fires once per accepted result, right before it's pushed into
    // `results` — sequential, exact-match contract (same shape as
    // FT8's sniper strategy). 0-or-1+ calls depending on `has_ap`'s
    // early exit below.
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> Vec<DecodeResult>
where
    P::Fec: crate::engine::protocol::BpPooledFec,
    P::Msg: WsjtApCompatible,
{
    let freq_min = (target_freq - search_hz).max(100.0);
    let freq_max = (target_freq + search_hz).min(5_900.0);
    let candidates = coarse_sync::<P>(
        audio,
        freq_min,
        freq_max,
        sync_min,
        Some(target_freq),
        max_cand,
    );
    if candidates.is_empty() {
        return Vec::new();
    }
    let has_ap = ap_hint.is_some_and(|h| h.has_info());
    let fft_cache = build_fft_cache(audio, ds_cfg);

    let mut results: Vec<DecodeResult> = Vec::new();
    for cand in &candidates {
        if let Some(r) = process_candidate_ap::<P>(
            cand,
            &fft_cache,
            ds_cfg,
            depth,
            strictness,
            eq_mode,
            refine_steps,
            sync_q_min,
            ap_hint,
        ) {
            let new = !results.iter().any(|x| x.info == r.info);
            if new {
                if let Some(cb) = on_result {
                    cb(&r);
                }
                results.push(r);
                // Early-exit: in sniper+AP mode we're hunting ONE target.
                // Once any AP-verified decode lands, further candidates are
                // almost certainly spurious — cut the remaining work.
                if has_ap {
                    break;
                }
            }
        }
    }
    results
}
