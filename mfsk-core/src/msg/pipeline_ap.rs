//! AP-assisted decode pipeline for WSJT 77-bit-family protocols.
//!
//! Builds on `mfsk-core::pipeline` to add multi-pass AP (a-priori) hints: the
//! caller supplies known portions of the expected message (callsigns, grid,
//! response) and the decoder tries several configurations with those bits
//! clamped to high-confidence LLRs. Because the 77-bit bit layout is shared
//! across FT8 / FT4 / FT2 / FST4, this code is protocol-agnostic via the
//! `P: Protocol` bound plus the `P::Msg = Wsjt77Message` convention.
//!
//! Typical threshold improvement is 2–4 dB when both call1 and call2 are
//! known (CQ + DX scenario) and can exceed that when a specific response
//! token (RRR / RR73 / 73) is also locked.

use crate::core::dsp::downsample::{DownsampleCfg, build_fft_cache, downsample_cached};
use crate::core::equalize::{EqMode, equalize_local};
use crate::core::llr::{compute_llr, compute_snr_db, symbol_spectra, sync_quality};
use crate::core::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness};
use crate::core::sync::{SyncCandidate, coarse_sync, fine_sync_power_per_block, refine_candidate};
use crate::core::tx::codeword_to_itone;
use crate::core::{FecCodec, FecOpts, Protocol};
use num_complex::Complex;

use super::ap::ApHint;
use super::wsjt77::{is_plausible_message, unpack77};

/// Upper bound on hard_errors for AP-assisted decodes, graded by the number
/// of locked bits (heavier locks → tighter threshold, since random bits
/// flipping to agree with the lock is increasingly unlikely).
fn ap_max_errors(strictness: DecodeStrictness, locked_bits: usize) -> u32 {
    match (strictness, locked_bits >= 55) {
        (DecodeStrictness::Strict, true) => 20,
        (DecodeStrictness::Strict, false) => 24,
        (DecodeStrictness::Normal, true) => 25,
        (DecodeStrictness::Normal, false) => 30,
        (DecodeStrictness::Deep, true) => 30,
        (DecodeStrictness::Deep, false) => 36,
    }
}

/// Build one AP configuration: derive the mask/values bit vectors from a
/// hint for this protocol's codeword length. Convenience for callers that
/// want to try several hint shapes (full lock, partial lock, …).
pub fn ap_bits_for<P: Protocol>(hint: &ApHint) -> (Vec<u8>, Vec<u8>) {
    hint.build_bits(P::Fec::N)
}

/// Enumerate the multi-pass AP configurations WSJT-X cycles through in
/// sniper mode — the `u8` is a pass-id tag for diagnostics.
///
/// - 9/10/11: full 77-bit lock with `RRR` / `RR73` / `73` (QSO in progress).
/// - 7:       CQ + DX call (expected "CQ DXCALL GRID").
/// - 8:       my-call + DX call (directed message).
/// - 6:       DX call only (partial lock, fallback).
pub fn ap_passes(base: &ApHint) -> Vec<(ApHint, u8)> {
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
pub fn process_candidate_ap<P: Protocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    ds_cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
    ap_hint: Option<&ApHint>,
) -> Option<DecodeResult> {
    let ds_rate = 12_000.0 / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;

    let cd0 = downsample_cached(fft_cache, cand.freq_hz, ds_cfg);
    let refined = refine_candidate::<P>(&cd0, cand, refine_steps);
    let i_start = ((refined.dt_sec + tx_start) * ds_rate).round() as usize;
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

    // Prepare EQ / non-EQ views of the symbol spectra. The non-EQ fallback
    // inside `EqMode::Adaptive` doubles per-candidate cost for only a
    // marginal gain (~1/20 extra decodes at -18 dB), so it is feature-
    // gated behind `eq-fallback`. Default Adaptive behaviour is
    // "EQ-only" — matches FT8's historical single-path approach.
    let cs_eq = {
        let mut v = cs_raw.clone();
        equalize_local::<P>(&mut v);
        v
    };
    #[cfg(feature = "eq-fallback")]
    let try_order: &[(&[Complex<f32>], bool)] = match eq_mode {
        EqMode::Off => &[(&cs_raw, false)],
        EqMode::Local => &[(&cs_eq, true)],
        EqMode::Adaptive => &[(&cs_eq, true), (&cs_raw, false)],
    };
    #[cfg(not(feature = "eq-fallback"))]
    let try_order: &[(&[Complex<f32>], bool)] = match eq_mode {
        EqMode::Off => &[(&cs_raw, false)],
        EqMode::Local | EqMode::Adaptive => &[(&cs_eq, true)],
    };

    for (cs_ref, _used_eq) in try_order {
        let cs_ref: &[Complex<f32>] = cs_ref;
        let llr_set = compute_llr::<P>(cs_ref);
        let variants: Vec<(&Vec<f32>, u8)> = match depth {
            DecodeDepth::Bp => vec![(&llr_set.llra, 0)],
            DecodeDepth::BpAll | DecodeDepth::BpAllOsd => vec![
                (&llr_set.llra, 0),
                (&llr_set.llrb, 1),
                (&llr_set.llrc, 2),
                (&llr_set.llrd, 3),
            ],
        };

        // ── Plain BP first, in case the signal is already clear ────────
        for (llr, pass_id) in &variants {
            let bp_opts = FecOpts { bp_max_iter: 30, osd_depth: 0, ap_mask: None };
            if let Some(r) = fec.decode_soft(llr, &bp_opts) {
                if let Some(res) = finalise_result::<P>(
                    &r, cand, &refined, sync_cv, *pass_id, cs_ref, None, &fec,
                ) {
                    return Some(res);
                }
            }
        }

        // ── AP-assisted passes ─────────────────────────────────────────
        //
        // Integer-timing retry (±2 downsampled samples around the
        // refined peak) was measured to deliver zero threshold
        // improvement at 5× runtime — the -18 dB floor is LLR-dominated,
        // not timing-dominated. See snr_sweep bench history 2026-04-18.
        if let Some(hint) = ap_hint {
            if hint.has_info() {
                for (ap_cfg, pass_id) in ap_passes(hint) {
                    let (mask, values) = ap_bits_for::<P>(&ap_cfg);
                    let locked = mask.iter().filter(|&&m| m != 0).count();
                    let max_errors = ap_max_errors(strictness, locked);

                    for (llr, _) in &variants {
                        let ap_opts = FecOpts {
                            bp_max_iter: 30,
                            osd_depth: 0,
                            ap_mask: Some((&mask, &values)),
                        };
                        if let Some(r) = fec.decode_soft(llr, &ap_opts) {
                            if r.hard_errors < max_errors {
                                if let Some(res) = finalise_result::<P>(
                                    &r, cand, &refined, sync_cv, pass_id, cs_ref,
                                    Some(&ap_cfg), &fec,
                                ) {
                                    return Some(res);
                                }
                            }
                        }
                        if depth == DecodeDepth::BpAllOsd {
                            // Default is depth-2 only (matches FT8's AP path).
                            // `osd-deep` feature enables the depth-3 fallback
                            // under heavy AP locks — ~0.5 dB threshold gain
                            // at ~25% extra runtime.
                            #[cfg(feature = "osd-deep")]
                            let depths: &[u32] = if locked >= 55 { &[2, 3] } else { &[2] };
                            #[cfg(not(feature = "osd-deep"))]
                            let depths: &[u32] = &[2];
                            let _ = locked;
                            for &od in depths {
                                let osd_opts = FecOpts {
                                    bp_max_iter: 30,
                                    osd_depth: od,
                                    ap_mask: Some((&mask, &values)),
                                };
                                if let Some(r) = fec.decode_soft(llr, &osd_opts) {
                                    if r.hard_errors < max_errors {
                                        if let Some(res) = finalise_result::<P>(
                                            &r, cand, &refined, sync_cv, pass_id,
                                            cs_ref, Some(&ap_cfg), &fec,
                                        ) {
                                            return Some(res);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    None
}

fn finalise_result<P: Protocol>(
    fec_result: &crate::core::FecResult,
    cand: &SyncCandidate,
    refined: &SyncCandidate,
    sync_cv: f32,
    pass_id: u8,
    cs: &[Complex<f32>],
    ap_cfg: Option<&ApHint>,
    fec: &P::Fec,
) -> Option<DecodeResult> {
    let msg77: [u8; 77] = fec_result.info[..77].try_into().ok()?;
    let text = unpack77(&msg77)?;
    if text.is_empty() || !is_plausible_message(&text) {
        return None;
    }
    // If this result came from an AP pass, verify the locked callsigns
    // actually appear in the decoded text — guards against spurious decodes
    // where the FEC happened to accept with the bits clamped.
    if let Some(ap) = ap_cfg {
        let upper = text.to_uppercase();
        if let Some(ref c1) = ap.call1 {
            if !upper.contains(&c1.to_uppercase()) {
                return None;
            }
        }
        if let Some(ref c2) = ap.call2 {
            if !upper.contains(&c2.to_uppercase()) {
                return None;
            }
        }
    }

    // Re-encode to compute a WSJT-X compatible SNR.
    let mut info = vec![0u8; 91];
    info[..77].copy_from_slice(&msg77);
    let mut bytes = [0u8; 12];
    for (i, &bit) in msg77.iter().enumerate() {
        bytes[i / 8] |= (bit & 1) << (7 - i % 8);
    }
    let crc = crc14_local(&bytes);
    for i in 0..14 {
        info[77 + i] = ((crc >> (13 - i)) & 1) as u8;
    }
    let mut cw = vec![0u8; P::Fec::N];
    fec.encode(&info, &mut cw);
    let itone = codeword_to_itone::<P>(&cw);
    let snr_db = compute_snr_db::<P>(cs, &itone);

    Some(DecodeResult {
        message77: msg77,
        freq_hz: cand.freq_hz,
        dt_sec: refined.dt_sec,
        hard_errors: fec_result.hard_errors,
        sync_score: refined.score,
        pass: pass_id,
        sync_cv,
        snr_db,
    })
}

/// CRC-14 (poly 0x2757) — duplicated locally to avoid a cross-crate dep on
/// `mfsk-fec::ldpc::crc14`. Kept small to stay inline-friendly.
fn crc14_local(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let msb = (crc >> 13) & 1;
            crc = ((crc << 1) | bit as u16) & 0x3FFF;
            if msb != 0 {
                crc ^= 0x2757;
            }
        }
    }
    crc
}

/// Sniper-mode decode with AP hints: search within `±search_hz` of
/// `target_freq`, with optional AP bit-locking applied per candidate.
#[allow(clippy::too_many_arguments)]
pub fn decode_sniper_ap<P: Protocol>(
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
) -> Vec<DecodeResult> {
    let freq_min = (target_freq - search_hz).max(100.0);
    let freq_max = (target_freq + search_hz).min(5_900.0);
    let candidates =
        coarse_sync::<P>(audio, freq_min, freq_max, sync_min, Some(target_freq), max_cand);
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
            let new = !results.iter().any(|x| x.message77 == r.message77);
            if new {
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
