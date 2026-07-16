//! Protocol-agnostic decode pipeline (basic path, no AP hints).
//!
//! Generic versions of `decode_frame` and `decode_frame_subtract` that drive
//! sync → downsample → LLR → FEC for any `P: Protocol`. AP-assisted decoding
//! (which depends on the 77-bit WSJT message bit layout) lives in
//! protocol-specific crates.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::dsp::downsample::{DownsampleCfg, build_fft_cache, downsample_cached};
use super::dsp::subtract::{SubtractCfg, subtract_tones};
use super::equalize::{EqMode, equalize_local};
use super::llr::{compute_llr, compute_snr_db, descramble_info, symbol_spectra, sync_quality};
use super::sync::{SyncCandidate, coarse_sync, fine_sync_power_per_block, refine_candidate};
use super::tx::codeword_to_itone;
use super::{FecCodec, FecOpts, MessageCodec, Protocol};

/// FFT cache for the initial large forward transform; reusable across passes.
pub type FftCache = Vec<Complex<f32>>;

/// Decoding depth: which LLR variants to attempt and whether to use OSD.
///
/// The single-variant `Bp` rung (llra-only, no all-variants pass) was retired
/// in 0.7.0 — no production caller was found by issue #74, and the cheapest
/// staircase step never functioned as a power-budget escape hatch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecodeDepth {
    /// BP across all four LLR variants (a, b, c, d).
    BpAll,
    /// BP on all variants, then OSD fallback when BP fails.
    BpAllOsd,
}

/// Decode strictness: trades off sensitivity vs false-positive rate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DecodeStrictness {
    Strict,
    #[default]
    Normal,
    Deep,
}

impl DecodeStrictness {
    /// Upper bound on `hard_errors` for non-AP OSD decode. Same calibration
    /// as the FT8 implementation; FT4/FST4 can re-tune later.
    pub fn osd_max_errors(self, osd_depth: u8) -> u32 {
        match (self, osd_depth) {
            (Self::Strict, 3) => 20,
            (Self::Strict, 4) => 24,
            (Self::Strict, _) => 22,
            (Self::Normal, 3) => 26,
            (Self::Normal, 4) => 30,
            (Self::Normal, _) => 29,
            (Self::Deep, 3) => 30,
            (Self::Deep, 4) => 36,
            (Self::Deep, _) => 40,
        }
    }

    /// Minimum coarse-sync score to enter OSD fallback.
    pub fn osd_score_min(self) -> f32 {
        match self {
            Self::Strict => 3.0,
            Self::Normal => 2.2,
            Self::Deep => 2.0,
        }
    }
}

/// One successfully decoded message. Protocol-agnostic.
///
/// `info` carries the FEC's K information bits — for LDPC(174,91) that's 91
/// bits (77 message + 14 CRC for Wsjt77-family), for LDPC(240,101) that's 101
/// bits (77 message + 24 CRC for FST4), for uvpacket it's 91 bits with the
/// `PacketBytesMessage` layout (4-bit length + 80-bit payload + 7-bit CRC-7).
/// The pipeline is agnostic to the layout; `MessageCodec::unpack` /
/// `MessageCodec::verify_info` interpret it per-protocol.
#[derive(Debug, Clone)]
pub struct DecodeResult {
    /// FEC-decoded information bits; length = `<P::Fec as FecCodec>::K`.
    pub info: Box<[u8]>,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub hard_errors: u32,
    pub sync_score: f32,
    pub pass: u8,
    /// Coefficient of variation of the per-block Costas powers — near 0 for
    /// stable channels, elevated under QSB or fading.
    pub sync_cv: f32,
    pub snr_db: f32,
}

impl DecodeResult {
    /// Slice the leading 77 message bits — the convention shared by every
    /// Wsjt77-family protocol (FT8 / FT4 / FT2 / FST4 / Q65). For uvpacket
    /// this still returns a 77-bit slice, but its interpretation is
    /// uvpacket-specific (length code + bytes + CRC fragment).
    ///
    /// Panics if `info` is shorter than 77 bits.
    pub fn message77(&self) -> &[u8] {
        &self.info[..77]
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Per-candidate processing
// ──────────────────────────────────────────────────────────────────────────

/// Decode a single sync candidate through the basic pipeline.
///
/// `fft_cache` must match the protocol's [`DownsampleCfg`]. `known` is used
/// to prevent redundant OSD work on frequencies with an existing decode.
pub fn process_candidate_basic<P: Protocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
) -> Option<DecodeResult> {
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let ds_rate = 12_000.0 / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;

    let mut cd0 = downsample_cached(fft_cache, cand.freq_hz, cfg);
    // RMS-normalise the downsampled baseband to unit power.
    // Matches WSJT-X `ft4_decode.f90:231-232`:
    //   sum2 = sum(|cd2|²) / (NMAX/NDOWN)
    //   cd2  = cd2 / sqrt(sum2)
    // The LLR_SCALE=2.83 used by `compute_llr` is calibrated against
    // unit-RMS input; without this normalisation the per-tone
    // magnitudes feeding `tanh(llr/2)` inside BP land at the wrong
    // scale and the decoder converges on systematically wrong
    // codewords that just happen to satisfy CRC-14 (the 4-CRC-false-
    // positive symptom on the FT4 reference WAV — issue #18).
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }

    // FT4 uses the WSJT-X-faithful 2-D `sync2d_refine` (`sync4d.f90`
    // constants: coarse ±12 Hz/3 Hz × ±20/4 samples; fine ±4 Hz/1 Hz ×
    // ±5 samples).  Bit-identical to the old `sync4d_refine`; critical for
    // sub-bin signals like W9JA (520.0 Hz, midway between our 5.21-Hz bins)
    // in the WSJT-X golden WAV.
    //
    // FST4 uses `fst4_sync_search`: faithful port of WSJT-X
    // `fst4_decode.f90:879-925`.  Coarse pass sweeps ±1.5 s (full slot)
    // so the winner is always near the true peak; fine pass ±7×0.02·baud ×
    // ±4 samples locks in.  Previous local-window approach (Sync2dConfig
    // ±10 samples) caused regression because noise peaks at the window edge
    // displaced the fine pass outside reach of the true position.
    //
    // FT8 keeps the generic time-only `refine_candidate` path; it has its
    // own 3-stage refine wired separately in `ft8/decode.rs`.
    let (refined, i_start): (SyncCandidate, i32) =
        if P::ID == super::ProtocolId::Ft4 || P::ID == super::ProtocolId::Fst4 {
            let s2 = if P::ID == super::ProtocolId::Fst4 {
                super::sync2d::fst4_sync_search::<P>(&cd0, cand)
            } else {
                let cfg = super::sync2d::Sync2dConfig::for_ft4();
                super::sync2d::sync2d_refine::<P>(&cd0, cand, &cfg)
            };
            let df_hz = s2.freq_hz - cand.freq_hz;
            cd0 = super::sync2d::freq_shift_cd0(&cd0, df_hz, ds_rate);
            let i_start: i32 = s2.i0;
            let refined = SyncCandidate {
                freq_hz: s2.freq_hz,
                dt_sec: (s2.i0 as f32) / ds_rate - tx_start,
                score: s2.score,
            };
            (refined, i_start)
        } else {
            let refined = refine_candidate::<P>(&cd0, cand, refine_steps);
            let i_start = ((refined.dt_sec + tx_start) * ds_rate).round() as i32;
            (refined, i_start)
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
            let var = per_block.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n;
            var.sqrt() / mean
        } else {
            0.0
        }
    } else {
        0.0
    };

    let _ = ntones;
    let _ = n_sym;
    let decode = |cs: &[Complex<f32>]| -> Option<DecodeResult> {
        let mut llr_set = compute_llr::<P, f32>(cs);
        // RX half of the optional bit interleaver: if the protocol
        // declares an interleave table, permute each LLR vector from
        // channel-bit order into codeword-bit order before BP/OSD.
        // No-op for protocols with `CODEWORD_INTERLEAVE = None`
        // (FT4/FT8/FST4/etc) — same call site, byte-identical result.
        deinterleave_llr_set::<P>(&mut llr_set);
        // llre (nsym=P::LLR_NSYM_MID, e.g. FST4's nsym=4 rung — see
        // `ModulationParams::LLR_NSYM_MID`) is empty for every protocol
        // that doesn't set LLR_NSYM_MID, so this is a Vec instead of a
        // fixed array only to make that slot conditional; no behaviour
        // change for FT8/FT4/etc.
        let mut variants: Vec<(&Vec<f32>, u8)> = Vec::with_capacity(5);
        variants.push((&llr_set.llra, 0u8));
        variants.push((&llr_set.llrb, 1));
        if !llr_set.llre.is_empty() {
            variants.push((&llr_set.llre, 6));
        }
        variants.push((&llr_set.llrc, 2));
        variants.push((&llr_set.llrd, 3));

        let fec = P::Fec::default();
        let bp_opts = FecOpts {
            bp_max_iter: 30,
            osd_depth: 0,
            ap_mask: None,
            // Thread the protocol's message-codec verifier so CRC-bearing
            // protocols (FT8/FT4/FST4 → Wsjt77 → CRC-14) keep their
            // existing reject-on-CRC-fail behaviour. uvpacket-style
            // codecs that override `verify_info = |_| true` accept any
            // parity-converged candidate.
            verify_info: Some(<P::Msg as MessageCodec>::verify_info),
            ..FecOpts::default()
        };

        for (llr, pass_id) in &variants {
            if let Some(mut r) = fec.decode_soft(llr, &bp_opts) {
                let itone = encode_tones_for_snr::<P>(&r.info, &fec);
                let snr_db = compute_snr_db::<P>(cs, &itone);
                // FT4 pre-LDPC scramble (WSJT-X `genft4.f90:64`): undo
                // the rvec XOR before presenting the 77-bit payload.
                descramble_info::<P>(&mut r.info);
                return Some(DecodeResult {
                    info: r.info.into_boxed_slice(),
                    freq_hz: cand.freq_hz,
                    dt_sec: refined.dt_sec,
                    hard_errors: r.hard_errors,
                    sync_score: refined.score,
                    pass: *pass_id,
                    sync_cv,
                    snr_db,
                });
            }
        }

        // WSJT-X's own FST4 decoder (`fst4_decode.f90`) has neither of
        // the gates below: `decode240_101` is called unconditionally
        // after BP fails (no coarse-sync-score pre-filter), and its
        // only acceptance test is `nharderrors.ge.0 .and.
        // unpk77_success` (`fst4_decode.f90:570`) — i.e. "OSD
        // converged to a CRC-24-verified codeword", full stop, no
        // upper bound on how many bits OSD had to flip to get there.
        // `osd_score_min`/`osd_max_errors` are FT8-calibrated
        // (doc'd as "can re-tune later", issue #72) and were never
        // re-tuned for FST4: near its own sensitivity threshold,
        // every real candidate's coarse-sync score sat below
        // `osd_score_min` (blocking OSD entirely) and every OSD
        // result that *did* run had a CRC-verified hard-error count
        // above `osd_max_errors` (rejected despite being provably
        // correct) — issue #146. Bypass both for FST4 to match
        // WSJT-X: attempt OSD whenever plain BP fails (still gated on
        // `nsync` the same as every other protocol), and trust the
        // CRC-24 verification inside `decode_soft` alone.
        let is_fst4 = P::ID == super::ProtocolId::Fst4;
        if depth == DecodeDepth::BpAllOsd
            && nsync >= 12
            && (is_fst4 || cand.score >= strictness.osd_score_min())
        {
            let freq_dup = known
                .iter()
                .any(|r| (r.freq_hz - cand.freq_hz).abs() < 20.0);
            if !freq_dup {
                let osd_depth: u8 = if nsync >= 18 { 3 } else { 2 };
                let osd_opts = FecOpts {
                    bp_max_iter: 30,
                    osd_depth: osd_depth as u32,
                    ap_mask: None,
                    verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                    ..FecOpts::default()
                };
                for (llr, _) in &variants {
                    if let Some(mut r) = fec.decode_soft(llr, &osd_opts) {
                        if !is_fst4 && r.hard_errors >= strictness.osd_max_errors(osd_depth) {
                            continue;
                        }
                        let itone = encode_tones_for_snr::<P>(&r.info, &fec);
                        let snr_db = compute_snr_db::<P>(cs, &itone);
                        descramble_info::<P>(&mut r.info);
                        return Some(DecodeResult {
                            info: r.info.into_boxed_slice(),
                            freq_hz: cand.freq_hz,
                            dt_sec: refined.dt_sec,
                            hard_errors: r.hard_errors,
                            sync_score: refined.score,
                            pass: if osd_depth == 3 { 5 } else { 4 },
                            sync_cv,
                            snr_db,
                        });
                    }
                }
                // OSD depth-4 Top-K pruning gated on high sync quality.
                if nsync >= 18 {
                    let osd4_opts = FecOpts {
                        bp_max_iter: 30,
                        osd_depth: 4,
                        ap_mask: None,
                        verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                        ..FecOpts::default()
                    };
                    for (llr, _) in &variants {
                        if let Some(mut r) = fec.decode_soft(llr, &osd4_opts) {
                            if !is_fst4 && r.hard_errors >= strictness.osd_max_errors(4) {
                                continue;
                            }
                            let itone = encode_tones_for_snr::<P>(&r.info, &fec);
                            let snr_db = compute_snr_db::<P>(cs, &itone);
                            descramble_info::<P>(&mut r.info);
                            return Some(DecodeResult {
                                info: r.info.into_boxed_slice(),
                                freq_hz: cand.freq_hz,
                                dt_sec: refined.dt_sec,
                                hard_errors: r.hard_errors,
                                sync_score: refined.score,
                                pass: 13,
                                sync_cv,
                                snr_db,
                            });
                        }
                    }
                }
            }
        }

        None
    };

    match eq_mode {
        EqMode::Off => decode(&cs_raw),
        EqMode::Local => {
            let mut cs_eq = cs_raw.clone();
            equalize_local::<P>(&mut cs_eq);
            decode(&cs_eq)
        }
    }
}

/// Deinterleave each of the four LLR variants from channel-bit order to
/// codeword-bit order, in place. No-op when
/// [`P::CODEWORD_INTERLEAVE`](crate::core::FrameLayout::CODEWORD_INTERLEAVE)
/// is `None` — every existing protocol stays bit-identical.
fn deinterleave_llr_set<P: Protocol>(set: &mut crate::core::llr::LlrSet) {
    if let Some(table) = P::CODEWORD_INTERLEAVE {
        deinterleave_llr_vec(&mut set.llra, table);
        deinterleave_llr_vec(&mut set.llrb, table);
        deinterleave_llr_vec(&mut set.llrc, table);
        deinterleave_llr_vec(&mut set.llrd, table);
    }
}

/// `llr[INTERLEAVE[j]] = channel_llr[j]` — inverse of the TX-side
/// permutation. Allocates one temporary `Vec<f32>` per call (per LLR
/// variant); the cost is tiny next to BP/OSD.
fn deinterleave_llr_vec(llr: &mut [f32], table: &[u16]) {
    debug_assert_eq!(
        llr.len(),
        table.len(),
        "interleave table length must match LLR length"
    );
    let original: Vec<f32> = llr.to_vec();
    for j in 0..llr.len() {
        llr[table[j] as usize] = original[j];
    }
}

/// Re-encode FEC info bits back into tones for SNR estimation.
///
/// Phase A reduced this to a 3-line helper: `r.info[..]` already
/// carries the K-bit info the FEC produced, including any CRC bits
/// that `MessageCodec::verify_info` already accepted. Feeding it
/// straight back into `fec.encode` reproduces the same codeword as
/// the previous "extract msg77 → recompute CRC → encode" path —
/// bit-identical because verifier acceptance enforces
/// `info[77..K] == crc(info[..77])` at the moment of acceptance.
fn encode_tones_for_snr<P: Protocol>(info: &[u8], fec: &P::Fec) -> Vec<u8> {
    let mut cw = vec![0u8; P::Fec::N];
    fec.encode(info, &mut cw);
    codeword_to_itone::<P>(&cw)
}

// ──────────────────────────────────────────────────────────────────────────
// Frame-level entry points
// ──────────────────────────────────────────────────────────────────────────

/// Decode one slot of audio: coarse sync → candidates → BP/OSD per candidate.
pub fn decode_frame<P: Protocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
) -> (Vec<DecodeResult>, FftCache) {
    let candidates = coarse_sync::<P>(audio, freq_min, freq_max, sync_min, freq_hint, max_cand);
    let fft_cache = build_fft_cache(audio, cfg);
    if candidates.is_empty() {
        return (Vec::new(), fft_cache);
    }

    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            process_candidate_basic::<P>(
                cand,
                &fft_cache,
                cfg,
                depth,
                strictness,
                &[],
                eq_mode,
                refine_steps,
                sync_q_min,
            )
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            process_candidate_basic::<P>(
                cand,
                &fft_cache,
                cfg,
                depth,
                strictness,
                &[],
                eq_mode,
                refine_steps,
                sync_q_min,
            )
        })
        .collect();

    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        if !results.iter().any(|x| x.info == r.info) {
            results.push(r);
        }
    }
    (results, fft_cache)
}

/// Multi-pass decode with successive signal subtraction. Each pass decodes
/// the residual audio; decoded signals are reconstructed and subtracted so
/// subsequent passes can expose previously-masked weak signals.
pub fn decode_frame_subtract<P: Protocol>(
    audio: &[i16],
    ds_cfg: &DownsampleCfg,
    sub_cfg: &SubtractCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    refine_steps: i32,
    sync_q_min: u32,
) -> Vec<DecodeResult> {
    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = Vec::new();
    let passes: &[f32] = &[1.0, 0.75, 0.5];
    let fec = P::Fec::default();

    for &factor in passes {
        let candidates = coarse_sync::<P>(
            &residual,
            freq_min,
            freq_max,
            sync_min * factor,
            freq_hint,
            max_cand,
        );
        if candidates.is_empty() {
            continue;
        }
        let fft_cache = build_fft_cache(&residual, ds_cfg);

        #[cfg(feature = "parallel")]
        let new: Vec<DecodeResult> = candidates
            .par_iter()
            .filter_map(|cand| {
                process_candidate_basic::<P>(
                    cand,
                    &fft_cache,
                    ds_cfg,
                    depth,
                    strictness,
                    &all_results,
                    EqMode::Off,
                    refine_steps,
                    sync_q_min,
                )
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let new: Vec<DecodeResult> = candidates
            .iter()
            .filter_map(|cand| {
                process_candidate_basic::<P>(
                    cand,
                    &fft_cache,
                    ds_cfg,
                    depth,
                    strictness,
                    &all_results,
                    EqMode::Off,
                    refine_steps,
                    sync_q_min,
                )
            })
            .collect();

        let mut deduped: Vec<DecodeResult> = Vec::new();
        for r in new {
            if !all_results.iter().any(|k| k.info == r.info)
                && !deduped.iter().any(|x| x.info == r.info)
            {
                deduped.push(r);
            }
        }

        for r in &deduped {
            // Arithmetic form (1.0 - 0.5*qsb) instead of the obvious
            // `if cond { 0.5 } else { 1.0 }` to dodge an Xtensa Rust
            // 1.95.0.0 LLVM instruction-selection SIGSEGV on the
            // `[2 x float] [1.0, 0.5]` constant pool the select form
            // generates. The FT8 SIC path used the same workaround
            // until 0.6.2 retired the FT8 implementation of
            // `qsb_partial_gain` and `subtract_signal_weighted` (the
            // FT4 versions are still live in this generic pipeline).
            let qsb = (r.sync_cv > 0.3) as u32 as f32;
            let gain = 1.0 - 0.5 * qsb;
            // `r.info` is post-descramble (FT4 only); re-apply the rvec
            // XOR before re-encoding so the subtracted tones match what
            // was actually on the air. XOR is its own inverse, so calling
            // `descramble_info` here scrambles back to the wire form.
            let mut info_for_tx = r.info.to_vec();
            descramble_info::<P>(&mut info_for_tx);
            let tones = encode_tones_for_snr::<P>(&info_for_tx, &fec);
            subtract_tones(&mut residual, &tones, r.freq_hz, r.dt_sec, gain, sub_cfg);
        }
        all_results.extend(deduped);
    }

    all_results
}
