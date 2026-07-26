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
use super::dsp::subtract::SubtractCfg;
use super::equalize::{EqMode, equalize_local};
use super::llr::{compute_llr, compute_snr_db, descramble_info, symbol_spectra, sync_quality};
use super::sync::{SyncCandidate, coarse_sync, fine_sync_power_per_block, refine_candidate};
use super::tx::codeword_to_itone;
use super::{FecCodec, FecOpts, MessageCodec, Protocol};

/// FFT cache for the initial large forward transform; reusable across passes.
pub type FftCache = Vec<Complex<f32>>;

/// How much extra work the BP staircase does per candidate before falling
/// back to more expensive strategies. The only axis embedded targets ever
/// configure — see [`DecodeDepth::osd`] for the (host-only) OSD escalation
/// axis.
///
/// Each bit's log-likelihood ratio (LLR) can be estimated by looking at
/// just its own symbol, or jointly across 2 or 3 *adjacent* symbols — a
/// wider joint estimate is a more reliable LLR (correlated symbol-decision
/// errors partially cancel) but costs proportionally more to compute, and
/// BP is tried again from scratch each time a wider estimate is added.
/// `LlrEffort` picks how wide this staircase climbs before giving up on a
/// candidate.
///
/// FT8-only in practice: `process_candidate_basic` below (the engine
/// FT4/FST4 share) always computes all LLR variants unconditionally and
/// never reads this field — only FT8's own `ft8::decode_block` engine has
/// an actual `Minimal`/`Full` staircase. Kept on the shared type (rather
/// than an FT8-local field) so [`DecodeDepth`] has one shape across every
/// protocol using [`crate::msg::decode_request::DecodeRequest`] (issue #191).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LlrEffort {
    /// Only the two cheap 1-symbol LLR estimates. ESP32 ship default — the
    /// 2-symbol/3-symbol estimates empirically add zero extra decodes on
    /// power-budgeted busy-band references (S3 log 2026-05-21; host
    /// re-measurement 2026-07-26: +8ms, 0 extra decodes on `qso3_busy.wav`).
    Minimal,
    /// All four LLR estimates, up to the 3-symbol joint one. Host default —
    /// full recall.
    Full,
}

/// Decode cost/recall configuration: [`LlrEffort`] plus whether to escalate
/// to OSD when the BP staircase fails.
///
/// `osd` is host-only: the OSD dispatch code is compiled out of
/// non-`fft-rustfft` builds entirely, so `osd: true` is a silent no-op on
/// embedded rather than a footgun. OSD has never shipped on an ESP32 target
/// and there is no plan to add it there — this isn't a current tuning
/// choice, it's a permanent architectural boundary.
///
/// Redesigned in 0.8.0 (issue #182 follow-up, then issue #191) from
/// FT8-local 3-/4-variant enums (`BpAll`/`BpAllOsd`/…) into this single
/// orthogonal struct shared by every protocol. The single-variant `Bp` rung
/// (llra-only, no all-variants pass) was retired in 0.7.0 — no production
/// caller was found by issue #74, and the cheapest staircase step never
/// functioned as a power-budget escape hatch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodeDepth {
    pub llr_effort: LlrEffort,
    pub osd: bool,
}

impl DecodeDepth {
    /// ESP32 ship config: cheapest LLR effort, OSD off.
    pub const EMBEDDED: Self = Self {
        llr_effort: LlrEffort::Minimal,
        osd: false,
    };
    /// Full LLR effort, no OSD — host "fast" baseline (was `BpAll`).
    pub const BP_ONLY: Self = Self {
        llr_effort: LlrEffort::Full,
        osd: false,
    };
    /// Full LLR effort + OSD fallback — host default (was `BpAllOsd`).
    pub const FULL: Self = Self {
        llr_effort: LlrEffort::Full,
        osd: true,
    };
}

/// Decode strictness: trades off sensitivity vs false-positive rate.
///
/// `process_candidate_basic` bypasses both `osd_score_min` and
/// `osd_max_errors` for FST4 (see the `is_fst4` gate below — issue #146:
/// WSJT-X's own FST4 decoder has no such gates), so in practice these
/// numbers are FT4-exclusive. `Normal` (FT4's hardcoded strictness,
/// issue #72) was retuned 2026-07-18 against a `ft4sim` AWGN/CCIR sweep
/// (`docs/notes/FT4_BENCHMARK.md`) — no longer a placeholder copy of the
/// FT8 calibration. `Strict`/`Deep` are unused by any current caller but
/// kept for the API shape; their numbers are the original FT8-copied
/// values, unverified for FT4.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DecodeStrictness {
    Strict,
    #[default]
    Normal,
    Deep,
}

impl DecodeStrictness {
    /// Upper bound on `hard_errors` for non-AP OSD decode.
    ///
    /// `Normal`'s values were retuned for FT4 (issue #72, 2026-07-18) by
    /// sweeping against `ft4sim`-generated AWGN/CCIR WAVs and picking the
    /// loosest thresholds that gained real (golden-message) recall without
    /// also growing false-accepts (any CRC-passing decode beyond the golden
    /// one) — see the `ft4_strictness_probe` test and
    /// `docs/notes/FT4_BENCHMARK.md` section 5 for the measurements.
    /// `Strict`/`Deep` remain the original FT8-copied placeholders.
    pub fn osd_max_errors(self, osd_depth: u8) -> u32 {
        match (self, osd_depth) {
            (Self::Strict, 3) => 20,
            (Self::Strict, 4) => 24,
            (Self::Strict, _) => 22,
            (Self::Normal, 3) => 28,
            (Self::Normal, 4) => 30,
            (Self::Normal, _) => 31,
            (Self::Deep, 3) => 30,
            (Self::Deep, 4) => 36,
            (Self::Deep, _) => 40,
        }
    }

    /// Minimum coarse-sync score to enter OSD fallback.
    ///
    /// `Normal` retuned alongside `osd_max_errors` above (issue #72).
    pub fn osd_score_min(self) -> f32 {
        match self {
            Self::Strict => 3.0,
            Self::Normal => 1.8,
            Self::Deep => 2.0,
        }
    }

    /// Upper bound on `hard_errors` for AP-assisted decode passes, graded by
    /// the number of locked bits (heavier locks → tighter threshold, since
    /// random bits flipping to agree with the lock is increasingly
    /// unlikely). Calibrated from a synthetic QSO scenario (REPORT AP at
    /// -18 dB: 15% FP rate with old thresholds 30/36) — shared by FT8's
    /// per-candidate AP loop and [`crate::msg::pipeline_ap`]'s generic
    /// sniper (issue #191 type consolidation; previously duplicated
    /// byte-for-byte in both places).
    pub fn ap_max_errors(self, locked_bits: usize) -> u32 {
        match (self, locked_bits >= 55) {
            (Self::Strict, true) => 20,
            (Self::Strict, false) => 24,
            (Self::Normal, true) => 25,
            (Self::Normal, false) => 30,
            (Self::Deep, true) => 30,
            (Self::Deep, false) => 36,
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

    let _ = ntones;
    let _ = n_sym;
    // BP iteration budget: WSJT-X's `ft8b.f90:96` and `fst4/decode240_101.f90:27`
    // both use `max_iterations=30`, but `ft4_decode.f90:194` uses 40 — FT4 is
    // the outlier, not the other two. Scoped to `P::ID == Ft4` (issue #72,
    // discovered while checking whether BP/OSD strength explains the residual
    // AWGN gap after `docs/notes/FT4_BENCHMARK.md` section 9) so FT8/FST4 stay
    // byte-identical.
    let bp_max_iter: u32 = if P::ID == super::ProtocolId::Ft4 {
        40
    } else {
        30
    };
    let cd0_base = cd0;

    // Attempt a full decode (symbol_spectra -> nsync gate -> BP -> OSD) at
    // one explicit `(freq_hz, i0, score)` position. Factored out of the
    // single-position call below so FT4 can retry it at up to 3 positions
    // (see the segment loop further down) without duplicating the LLR/BP/
    // OSD logic.
    let try_position = |freq_hz: f32, i0: i32, score: f32| -> Option<DecodeResult> {
        let df_hz = freq_hz - cand.freq_hz;
        let cd0 = super::sync2d::freq_shift_cd0(&cd0_base, df_hz, ds_rate);
        let refined = SyncCandidate {
            freq_hz,
            dt_sec: (i0 as f32) / ds_rate - tx_start,
            score,
        };

        let cs_raw = symbol_spectra::<P>(&cd0, i0);
        let nsync = sync_quality::<P>(&cs_raw);
        if nsync <= sync_q_min {
            return None;
        }

        let per_block = fine_sync_power_per_block::<P>(&cd0, i0);
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
                bp_max_iter,
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
                        freq_hz: refined.freq_hz,
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
            // FT4 hit the identical `osd_score_min` symptom FST4 already
            // worked around: `cand.score` is `coarse_sync`'s non-coherent
            // score (unrelated to the coherent `ft4_sync_search` score
            // computed above), and `1.8` was tuned against it back when
            // that was the only score in play (section 5/6). Empirically
            // confirmed (`ft4_diag_weak_trials`, issue #72,
            // `docs/notes/FT4_BENCHMARK.md` section 12): 13/17 currently-
            // failing near-crossing AWGN candidates have `cand.score <
            // 1.8` and so never even attempt OSD, despite all 17 clearing
            // WSJT-X's own `syncmin=1.2` easily (real signals, not
            // noise). WSJT-X's own FT4 decoder has no OSD-attempt score
            // gate at all either (`decode174_91` runs BP+OSD together,
            // governed by `ndepth`, not by a score check) — bypass
            // `osd_score_min` for FT4 too, same as FST4, keeping
            // `osd_max_errors` (a real hard-error ceiling, not a stale-
            // quantity gate) as the false-accept safety net.
            let bypass_osd_score_min = is_fst4 || P::ID == super::ProtocolId::Ft4;
            // The `12`/`18` OSD depth-escalation gates below were calibrated
            // against FT8's `N_SYNC=21` (3 blocks x 7-symbol Costas): 12/21
            // ~ attempt-OSD-at-all, 18/21 ~ escalate to depth-3/depth-4.
            // FT4's `N_SYNC=16` (4 blocks x 4-symbol Costas) is smaller —
            // `nsync` can never reach 18 there (empirically confirmed via
            // `ft4_diag_weak_trials`, issue #72: even -14dB AWGN decodes
            // topped out around 15/16), so depth-3 OSD and the depth-4 Top-K
            // rescue were silently dead code for every FT4 candidate. Scale
            // by the same ratio the FT8 numbers imply, applied to FT4's own
            // `N_SYNC` (16 * 12/21 ~ 9, 16 * 18/21 ~ 14) — reproduces 12/18
            // exactly for FT8 (`P::N_SYNC == 21`).
            //
            // FST4's `N_SYNC=40` (5 blocks x 8-symbol Costas) is the
            // opposite problem: 18/40=45% is a far *looser* bar than FT8's
            // 18/21=86%, so roughly half of all real candidates cleared it
            // regardless of actual signal quality — not dead code, but the
            // wrong kind of live code. `Ldpc240_101::decode_soft` tries OSD
            // twice per LLR variant at whatever depth is requested (raw LLR,
            // then WSJT-X's `zsave`-style running-BP-sum retry,
            // `fec/ldpc240_101/mod.rs:148-197` — both genuinely needed,
            // issue #146), across up to 5 LLR variants
            // (`llra/llrb/llre/llrc/llrd`) — so escalating unnecessarily is
            // expensive: `fst4_60_diag_osd_escalation`
            // (`tests/fst4_sweep.rs`) measured the WSJT-X FST4-60 golden WAV
            // at the unscaled gates: 24 of 50 candidates attempted OSD
            // depth-2/3 (only 1 succeeded), for 3.7 s combined, vs 2
            // escalating further to depth-4 for another 2.1 s — on a WAV
            // whose real signals were all found well under either threshold.
            // The comment this replaced claimed FST4's depth-escalation gate
            // "was already tuned separately" (issue #146) — checked
            // `CHANGELOG.md` for that tuning and found none: issue #146's
            // FST4 work tuned `bypass_osd_score_min` (directly above) and
            // AWGN sensitivity, never the `(12, 18)` pair itself, which
            // appears to have been an untouched FT8 inheritance all along.
            //
            // Unlike FT4, reusing the same `N_SYNC`-scaled formula for FST4
            // (→ 23/34) is NOT safe: a controlled A/B (`FST4_BENCHMARK.md`
            // section 8) measured a real ~0.5 dB AWGN sensitivity regression
            // — some real FST4 signals' `nsync` genuinely falls in [18, 34),
            // unlike FT4 where the scaled threshold only ever unlocked
            // previously-dead code. `osd_attempt_min` stays the shared `12`
            // (raising it was most of that 0.5 dB loss); `osd_depth3_min=20`
            // is a hand-calibrated value verified directly against the real
            // `fst4_snr_sweep` AWGN/CCIR sweep (not the `N_SYNC` formula) —
            // matches the documented pre-fix baseline within sampling noise
            // on all 4 channels, plus FST4-120/300 AWGN spot-checks.
            // Integer round-to-nearest (`(A + B/2) / B`) instead of the
            // f32 `.round()` this originally used — same result for
            // FT4's `N_SYNC=16` (9/14 either way), no float ops on a
            // path embedded/no_std builds also compile (Gemini PR
            // review).
            let (osd_attempt_min, osd_depth3_min) = if P::ID == super::ProtocolId::Ft4 {
                ((12 * P::N_SYNC + 10) / 21, (18 * P::N_SYNC + 10) / 21)
            } else if P::ID == super::ProtocolId::Fst4 {
                (12, 20)
            } else {
                (12, 18)
            };
            if depth.osd
                && nsync >= osd_attempt_min
                && (bypass_osd_score_min || cand.score >= strictness.osd_score_min())
            {
                let freq_dup = known
                    .iter()
                    .any(|r| (r.freq_hz - cand.freq_hz).abs() < 20.0);
                if !freq_dup {
                    let osd_depth: u8 = if nsync >= osd_depth3_min { 3 } else { 2 };
                    let osd_opts = FecOpts {
                        bp_max_iter,
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
                                freq_hz: refined.freq_hz,
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
                    if nsync >= osd_depth3_min {
                        let osd4_opts = FecOpts {
                            bp_max_iter,
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
                                    freq_hz: refined.freq_hz,
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
    };

    // FT4 uses `ft4_sync_search`: a coherent full-slot Δt search (WSJT-X
    // `ft4_decode.f90` isync=1/2 + `sync4d.f90` scorer). A literal port of
    // WSJT-X's `iseg=1,2,3` per-segment retry structure (try up to 3
    // different Δt positions, not just the single global best) was
    // implemented and measured here — empirically ruled out, not just
    // unimplemented: `ft4_diag_segment_retry` (`tests/ft4_sweep.rs`,
    // issue #72, `docs/notes/FT4_BENCHMARK.md` section 11) found 0/17
    // rescues once the diagnostic was corrected to apply the same
    // `hard_errors >= osd_max_errors` gate and golden-message check
    // production does — an earlier uncorrected pass had over-reported
    // 10/17 by skipping that gate. Reverted to the single collapsed pass
    // to avoid 3x the search/decode cost for zero measured benefit.
    //
    // FST4 uses `fst4_sync_search`: faithful port of WSJT-X
    // `fst4_decode.f90:879-925`. Coarse pass sweeps ±1.5 s (full slot) so
    // the winner is always near the true peak; fine pass ±7×0.02·baud ×
    // ±4 samples locks in. Previous local-window approach (Sync2dConfig
    // ±10 samples) caused regression because noise peaks at the window
    // edge displaced the fine pass outside reach of the true position.
    //
    // FT8 (and everything else) keeps the generic time-only
    // `refine_candidate` path; FT8 has its own 3-stage refine wired
    // separately in `ft8/decode.rs`.
    let (freq_hz, i0, score) = if P::ID == super::ProtocolId::Ft4 {
        let s2 = super::sync2d::ft4_sync_search::<P>(&cd0_base, cand);
        (s2.freq_hz, s2.i0, s2.score)
    } else if P::ID == super::ProtocolId::Fst4 {
        let s2 = super::sync2d::fst4_sync_search::<P>(&cd0_base, cand);
        (s2.freq_hz, s2.i0, s2.score)
    } else {
        let refined = refine_candidate::<P>(&cd0_base, cand, refine_steps);
        let i_start = ((refined.dt_sec + tx_start) * ds_rate).round() as i32;
        (refined.freq_hz, i_start, refined.score)
    };

    // A WSJT-X-style `smax` early exit (`ft4_decode.f90:279`:
    // `if(smax.lt.1.2) cycle`) was implemented and measured here — using
    // `ft4_sync_search`'s own coherent score, not `cand.score` — and
    // reverted for negligible benefit (dapper-soaring-nest plan Phase 4,
    // `FT4_BENCHMARK.md` section 15): a safely-margined cutoff only
    // filtered 0.5% of non-golden candidates in the calibration sweep
    // (`ft4_diag_smax_calibration`, `tests/ft4_sweep.rs`) — junk scores
    // cluster tightly just below the golden-succeeding floor rather than
    // spread far below it, so there's no safe gap wide enough to filter
    // much without risking a real signal.

    try_position(freq_hz, i0, score)
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
    // FT4's own coarse-candidate stage (`core::ft4_coarse::ft4_coarse_sync`,
    // a faithful `getcandidates4.f90` port) replaces the generic 2-D
    // (freq × lag) Costas-correlation search: WSJT-X's FT4 candidate
    // finder has no lag dimension at all, and the generic search's
    // up-to-8 lag-distinct candidates per frequency are redundant
    // downstream for FT4 — `ft4_sync_search` (below) already searches
    // Δt absolutely, ignoring each candidate's own `dt_sec`. See
    // `core::ft4_coarse` module doc / `~/.claude/plans/dapper-soaring-nest.md`.
    let candidates = if P::ID == super::ProtocolId::Ft4 {
        super::ft4_coarse::ft4_coarse_sync(audio, freq_min, freq_max, sync_min, freq_hint, max_cand)
    } else {
        coarse_sync::<P>(audio, freq_min, freq_max, sync_min, freq_hint, max_cand)
    };
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

    // Dedup by decoded message, keeping the candidate with the highest
    // `sync_score` (the post-refine coherent Costas correlation) rather
    // than the first-processed one. `coarse_sync`'s NMS can keep more
    // than one (freq, dt) candidate per frequency bin, and more than one
    // can independently reach a self-consistent Costas lock on the same
    // real signal (not noise — both land in the same place after
    // `ft4_sync_search`'s refine). This is now mostly cosmetic
    // (`DecodeResult.freq_hz`/`dt_sec` come from the *refined* position,
    // not the raw candidate, so duplicates converge on nearly the same
    // reported values) but keeps the tie-break meaningful for the rare
    // case where refinement doesn't fully converge.
    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        match results.iter_mut().find(|x| x.info == r.info) {
            Some(existing) if r.sync_score > existing.sync_score => *existing = r,
            Some(_) => {}
            None => results.push(r),
        }
    }
    (results, fft_cache)
}

/// Multi-pass decode with successive signal subtraction. Each pass decodes
/// the residual audio; decoded signals are reconstructed and subtracted so
/// subsequent passes can expose previously-masked weak signals.
#[allow(clippy::too_many_arguments)]
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
    // Channel-aware LPF subtract tuning (issue #178/#179 FT4 port).
    // Protocol-specific — mirrors WSJT-X's per-protocol `NFILT`/
    // end-correction choice (`subtractft8.f90` vs `subtractft4.f90`).
    // Passed in rather than derived from `P` to avoid growing the
    // `Protocol` trait for a single generic-pipeline caller (FT4, as
    // of this writing).
    lpf_half: usize,
    lpf_endcorrection: bool,
    refine_freq_radius_hz: f32,
) -> Vec<DecodeResult> {
    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = Vec::new();
    let passes: &[f32] = &[1.0, 0.75, 0.5];
    let fec = P::Fec::default();

    for &factor in passes {
        // See the identical `P::ID == Ft4` branch in `decode_frame` above.
        let candidates = if P::ID == super::ProtocolId::Ft4 {
            super::ft4_coarse::ft4_coarse_sync(
                &residual,
                freq_min,
                freq_max,
                sync_min * factor,
                freq_hint,
                max_cand,
            )
        } else {
            coarse_sync::<P>(
                &residual,
                freq_min,
                freq_max,
                sync_min * factor,
                freq_hint,
                max_cand,
            )
        };
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
            // `r.info` is post-descramble (FT4 only); re-apply the rvec
            // XOR before re-encoding so the subtracted tones match what
            // was actually on the air. XOR is its own inverse, so calling
            // `descramble_info` here scrambles back to the wire form.
            let mut info_for_tx = r.info.to_vec();
            descramble_info::<P>(&mut info_for_tx);
            let tones = encode_tones_for_snr::<P>(&info_for_tx, &fec);
            // WSJT-X-faithful channel-aware LPF subtract, single shot
            // (issue #177/#178/#179): the old constant-amplitude
            // `subtract_tones` + coarse binary QSB gain (0.5 / 1.0 on
            // `sync_cv > 0.3`) is FT8's pre-0.6.2 design, never
            // migrated here when FT8 moved to `subtract_tones_lpf`. On
            // a synthetic busy-band scenario with a strong
            // Rayleigh-faded interferer 40 Hz from a weak target
            // (`ft4_busy_band_fading_probe.rs`), the old path recovered
            // the target 0/10 seeds; migrating to `subtract_tones_lpf`
            // (this call) recovers it reliably, 10/10.
            //
            // An intermediate version of this code iterated
            // `subtract_tones_lpf` to convergence per candidate (up to
            // 6, later 20, re-fits) — reading `ft4_decode.f90` /
            // `subtractft4.f90` directly showed WSJT-X never does
            // this: `subtractft4` is always a single call, and deeper
            // suppression of a persistent signal comes from the
            // *outer* multi-pass loop above (`for &factor in passes`)
            // re-detecting it as a fresh candidate in a later pass —
            // which this function already does independently of any
            // inner iteration. The inner convergence loop had no
            // WSJT-X counterpart and, once its iteration cap was
            // raised, repeatedly re-fit/re-subtracted the same
            // candidate against its own imperfect model with no
            // independent ground truth: on the real WSJT-X FT4 sample
            // this leaked distortion from `CQ RU AB5XS EM12` (560.0 Hz)
            // into `W9JA PY2APK RRR` (519.4 Hz, ~40 Hz away) and lost
            // that decode (`ft4_wsjtx_sample_iteration_diag.rs`).
            // Removed — the single-shot call here matches every
            // regression guard that previously seemed to require
            // convergence (this synthetic scenario 10/10, the real
            // sample 6/6, FT8's `qso3_busy.wav` 18/18) identically or
            // better.
            //
            // Also refines the carrier frequency first (`refine_freq`'s
            // own doc comment recommends this for real-signal input;
            // wasn't being called here either).
            let refined_freq = super::dsp::subtract::refine_freq(
                &residual,
                &tones,
                r.freq_hz,
                r.dt_sec,
                sub_cfg,
                refine_freq_radius_hz,
                0.1,
            );
            super::dsp::subtract::subtract_tones_lpf(
                &mut residual,
                &tones,
                refined_freq,
                r.dt_sec,
                sub_cfg,
                lpf_half,
                lpf_endcorrection,
            );
        }
        all_results.extend(deduped);
    }

    all_results
}
