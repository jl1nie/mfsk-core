/// High-level FT8 decode pipeline.
///
/// Chains: downsample → coarse_sync → fine_sync → LLR → BP decode
#[cfg(feature = "parallel")]
use rayon::prelude::*;

pub use super::equalizer::EqMode;
use super::{
    downsample::{build_fft_cache, downsample},
    equalizer,
    ldpc::{
        bp::bp_decode,
        osd::{osd_decode, osd_decode_deep, osd_decode_deep4},
    },
    llr::{compute_llr, compute_snr_db, symbol_spectra, sync_quality},
    message::pack28,
    params::{BP_MAX_ITER, LDPC_N},
    subtract::subtract_signal_weighted,
    sync::{SyncCandidate, coarse_sync, fine_sync_power_split, refine_candidate},
    wave_gen::message_to_tones,
};

// ────────────────────────────────────────────────────────────────────────────
// Public types

/// Opaque FFT cache produced by [`decode_frame_with_cache`] (Phase 1),
/// consumed by [`decode_frame_subtract_with_known`] (Phase 2).
pub type FftCache = Vec<num_complex::Complex<f32>>;

/// Decoding depth: which LLR sets and passes to attempt.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecodeDepth {
    /// Belief-propagation only, using nsym=1 metrics (fast).
    Bp,
    /// BP with all four metric variants (a, b, c, d).
    BpAll,
    /// BP (all four variants) then OSD order-1 fallback when BP fails.
    BpAllOsd,
}

/// Decode strictness: controls false-positive vs sensitivity trade-off.
///
/// Adjusts OSD hard_errors thresholds, AP hard_errors thresholds, and
/// the minimum sync score required for OSD fallback entry.
/// Actual numeric values are placeholders pending benchmark calibration.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DecodeStrictness {
    /// Minimise false positives — tighter thresholds.
    Strict,
    /// Balanced (current behaviour).
    #[default]
    Normal,
    /// Maximum sensitivity — looser thresholds, more FP risk.
    Deep,
}

impl DecodeStrictness {
    /// Maximum hard_errors for non-AP OSD decode.
    ///
    /// Calibrated from real WAV bench (2026-04-07):
    ///   - BP pass 0: errors 0–8 (all clean)
    ///   - OSD real signals: errors 19, 23
    ///   - OSD false positive: errors 29
    pub fn osd_max_errors(self, osd_depth: u8) -> u32 {
        match (self, osd_depth) {
            // Strict: high-confidence OSD (e19 real → keep, e23+ → cut)
            (Self::Strict, 3) => 20,
            (Self::Strict, 4) => 24,
            (Self::Strict, _) => 22,
            // Normal: catches errors=29 FP, keeps errors=23 real decode
            (Self::Normal, 3) => 26,
            (Self::Normal, 4) => 30,
            (Self::Normal, _) => 29,
            // Deep: previous defaults — maximum sensitivity
            (Self::Deep, 3) => 30,
            (Self::Deep, 4) => 36,
            (Self::Deep, _) => 40,
        }
    }

    /// Maximum hard_errors for AP decode passes.
    ///
    /// Calibrated from synthetic QSO scenario:
    ///   - REPORT AP at -18 dB: 15% FP rate with old thresholds (30/36)
    pub fn ap_max_errors(self, locked_bits: usize) -> u32 {
        match (self, locked_bits >= 55) {
            (Self::Strict, true) => 20,
            (Self::Strict, false) => 24,
            (Self::Normal, true) => 25,
            (Self::Normal, false) => 30,
            // Deep: previous defaults
            (Self::Deep, true) => 30,
            (Self::Deep, false) => 36,
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

/// A Priori information for assisted decoding.
///
/// Known callsigns are converted to 28-bit packed tokens and injected as
/// high-confidence LLR values into the BP decoder, effectively reducing the
/// number of unknown bits.  This lowers the decode threshold by several dB.
///
/// # Example
/// ```
/// use mfsk_core::ft8::decode::ApHint;
/// // "I'm calling 3Y0Z, expecting a reply to my CQ"
/// let ap = ApHint::new().with_call1("CQ").with_call2("3Y0Z");
/// ```
#[derive(Debug, Clone, Default)]
pub struct ApHint {
    /// Known first callsign (e.g. "CQ", "JA1ABC").
    /// Locks message bits 0–28 (28-bit call + 1-bit flag).
    pub call1: Option<String>,
    /// Known second callsign (e.g. "3Y0Z").
    /// Locks message bits 29–57 (28-bit call + 1-bit flag).
    pub call2: Option<String>,
    /// Known grid locator (e.g. "JD34").
    /// Locks message bits 58 (ir=0) + 59–73 (15-bit grid).
    pub grid: Option<String>,
    /// Known report/response token (e.g. "RRR", "RR73", "73").
    /// Locks bits 58–73 (ir flag + 15-bit report field) for full 77-bit lock.
    pub report: Option<String>,
}

impl ApHint {
    /// Construct an empty `ApHint` — no fields pre-filled.
    pub fn new() -> Self {
        Self::default()
    }
    /// Pre-fill the first callsign (`CALL1` in a standard FT8 message).
    pub fn with_call1(mut self, call: &str) -> Self {
        self.call1 = Some(call.to_string());
        self
    }
    /// Pre-fill the second callsign (`CALL2`).
    pub fn with_call2(mut self, call: &str) -> Self {
        self.call2 = Some(call.to_string());
        self
    }
    /// Pre-fill the 4-character Maidenhead grid.
    pub fn with_grid(mut self, grid: &str) -> Self {
        self.grid = Some(grid.to_string());
        self
    }
    /// Pre-fill the signal report (e.g. `"-12"`, `"R+05"`, `"73"`).
    pub fn with_report(mut self, rpt: &str) -> Self {
        self.report = Some(rpt.to_string());
        self
    }

    /// Returns true if any a-priori information is available.
    pub fn has_info(&self) -> bool {
        self.call1.is_some() || self.call2.is_some()
    }

    /// Build AP mask and LLR overrides for the 174-bit LDPC codeword.
    ///
    /// `apmag` — magnitude to assign to known bits (typically `max(|llr|) * 1.01`).
    ///
    /// Returns `(ap_mask, ap_llr)` where:
    /// - `ap_mask[i] = true` means bit `i` is a-priori known (frozen in BP)
    /// - `ap_llr[i]` is the LLR override for known bits (±apmag)
    pub fn build_ap(&self, apmag: f32) -> ([bool; LDPC_N], [f32; LDPC_N]) {
        let mut mask = [false; LDPC_N];
        let mut ap_llr = [0.0f32; LDPC_N];

        // Helper: write 28-bit packed call + 1-bit flag (=0) into AP arrays
        let mut set_call_bits = |call: &str, start: usize| {
            if let Some(n28) = pack28(call) {
                // Write 28 bits of the packed callsign
                for i in 0..28 {
                    let bit = ((n28 >> (27 - i)) & 1) as u8;
                    mask[start + i] = true;
                    ap_llr[start + i] = if bit == 1 { apmag } else { -apmag };
                }
                // Flag bit (ipa/ipb) = 0 for standard calls
                mask[start + 28] = true;
                ap_llr[start + 28] = -apmag; // bit=0 → negative LLR
            }
        };

        if let Some(ref c1) = self.call1 {
            set_call_bits(c1, 0); // bits 0–28
        }
        if let Some(ref c2) = self.call2 {
            set_call_bits(c2, 29); // bits 29–57
        }

        // Lock grid field (bits 58–73: ir=0 + 15-bit grid) if known
        if let Some(ref grid) = self.grid
            && let Some(igrid) = super::message::pack_grid4(grid)
        {
            mask[58] = true;
            ap_llr[58] = -apmag; // ir=0
            for i in 0..15 {
                let bit = ((igrid >> (14 - i)) & 1) as u8;
                mask[59 + i] = true;
                ap_llr[59 + i] = if bit == 1 { apmag } else { -apmag };
            }
        }

        // Lock report field (bits 58–73) for known responses: RRR, RR73, 73
        if let Some(ref rpt) = self.report {
            // Type 1: igrid values for special responses
            let igrid_val: Option<u32> = match rpt.as_str() {
                "RRR" => Some(32_400 + 2),
                "RR73" => Some(32_400 + 3),
                "73" => Some(32_400 + 4),
                _ => None,
            };
            if let Some(igrid) = igrid_val {
                mask[58] = true;
                ap_llr[58] = -apmag; // ir=0
                for i in 0..15 {
                    let bit = ((igrid >> (14 - i)) & 1) as u8;
                    mask[59 + i] = true;
                    ap_llr[59 + i] = if bit == 1 { apmag } else { -apmag };
                }
            }
        }

        // Lock message type i3=1 (Type 1 standard) if any call is known
        if self.has_info() {
            // bits 74-76 = i3 = 001 (Type 1)
            mask[74] = true;
            ap_llr[74] = -apmag; // bit=0
            mask[75] = true;
            ap_llr[75] = -apmag; // bit=0
            mask[76] = true;
            ap_llr[76] = apmag; // bit=1
        }

        (mask, ap_llr)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Main decode entry point

/// Decode one 15-second FT8 audio frame.
///
/// # Arguments
/// * `audio`      — 16-bit PCM samples at 12 000 Hz, length ≤ 180 000
/// * `freq_min`   — lower edge of search band (Hz)
/// * `freq_max`   — upper edge of search band (Hz)
/// * `sync_min`   — minimum coarse-sync score (typical: 1.0–2.0)
/// * `freq_hint`  — optional preferred frequency; matching candidates are tried first
/// * `depth`      — decoding depth
/// * `max_cand`   — maximum number of sync candidates to evaluate
///
/// Returns all successfully decoded messages (deduplicated by `message77`).
pub fn decode_frame(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_frame_inner(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        None,
    )
    .0
}

/// Like [`decode_frame`] but also returns the 192k-point FFT cache for
/// reuse by a subsequent [`decode_frame_subtract_with_known`] call.
///
/// This is the Phase 1 entry point for pipelined decoding.
pub fn decode_frame_with_cache(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    decode_frame_inner(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        None,
    )
}

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
    let osd_score_min = strictness.osd_score_min();
    let (cd0, _) = downsample(audio, cand.freq_hz, Some(fft_cache));

    let refined = refine_candidate(&cd0, cand, 10);
    let i_start = ((refined.dt_sec + 0.5) * 200.0).round() as usize;
    let cs_raw = symbol_spectra(&cd0, i_start);
    let nsync = sync_quality(&cs_raw);
    if nsync <= 6 {
        return None;
    }

    let sync_cv = {
        let (sa, sb, sc) = fine_sync_power_split(&cd0, i_start);
        let mean = (sa + sb + sc) / 3.0;
        if mean > f32::EPSILON {
            let sq = (sa - mean).powi(2) + (sb - mean).powi(2) + (sc - mean).powi(2);
            sq.sqrt() / mean
        } else {
            0.0
        }
    };

    let try_decode = |cs: &[[num_complex::Complex<f32>; 8]; 79],
                      use_ap: bool|
     -> Option<DecodeResult> {
        let llr_set = compute_llr(cs);

        let llr_variants: &[(&[f32; LDPC_N], u8)] = match depth {
            DecodeDepth::Bp => &[(&llr_set.llra, 0)],
            DecodeDepth::BpAll | DecodeDepth::BpAllOsd => &[
                (&llr_set.llra, 0),
                (&llr_set.llrb, 1),
                (&llr_set.llrc, 2),
                (&llr_set.llrd, 3),
            ],
        };

        // BP decode (no AP)
        for &(llr, pass_id) in llr_variants {
            if let Some(bp) = bp_decode(llr, None, BP_MAX_ITER) {
                let itone = message_to_tones(&bp.message77);
                let snr_db = compute_snr_db(cs, &itone);
                return Some(DecodeResult {
                    message77: bp.message77,
                    freq_hz: cand.freq_hz,
                    dt_sec: refined.dt_sec,
                    hard_errors: bp.hard_errors,
                    sync_score: refined.score,
                    pass: pass_id,
                    sync_cv,
                    snr_db,
                });
            }
        }

        // OSD fallback
        if depth == DecodeDepth::BpAllOsd && nsync >= 12 && cand.score >= osd_score_min {
            let freq_dup = known
                .iter()
                .any(|r| (r.freq_hz - cand.freq_hz).abs() < 20.0);
            if !freq_dup {
                let osd_depth: u8 = if nsync >= 18 { 3 } else { 2 };
                for llr_osd in [&llr_set.llra, &llr_set.llrb, &llr_set.llrc, &llr_set.llrd] {
                    let osd_result = if osd_depth == 3 {
                        osd_decode_deep(llr_osd, 3)
                    } else {
                        osd_decode(llr_osd)
                    };
                    if let Some(osd) = osd_result {
                        let max_errors = strictness.osd_max_errors(osd_depth);
                        if osd.hard_errors >= max_errors {
                            continue;
                        }
                        let itone = message_to_tones(&osd.message77);
                        let snr_db = compute_snr_db(cs, &itone);
                        return Some(DecodeResult {
                            message77: osd.message77,
                            freq_hz: cand.freq_hz,
                            dt_sec: refined.dt_sec,
                            hard_errors: osd.hard_errors,
                            sync_score: refined.score,
                            pass: if osd_depth == 3 { 5 } else { 4 },
                            sync_cv,
                            snr_db,
                        });
                    }
                }
                // OSD depth-4 (Top-K pruning): same sync gate as depth-3.
                // k4_limit=30 → C(30,4)=27,405 extra candidates at depth-3 cost.
                if nsync >= 18 {
                    for llr_osd in [&llr_set.llra, &llr_set.llrb, &llr_set.llrc, &llr_set.llrd] {
                        if let Some(osd4) = osd_decode_deep4(llr_osd, 30) {
                            let max_errors = strictness.osd_max_errors(4);
                            if osd4.hard_errors >= max_errors {
                                continue;
                            }
                            let itone = message_to_tones(&osd4.message77);
                            let snr_db = compute_snr_db(cs, &itone);
                            return Some(DecodeResult {
                                message77: osd4.message77,
                                freq_hz: cand.freq_hz,
                                dt_sec: refined.dt_sec,
                                hard_errors: osd4.hard_errors,
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

        // Multi-pass AP (similar to WSJT-X a1..a7)
        // Try progressively deeper AP configurations:
        //   pass 6: call2 only (original)
        //   pass 7: CQ + call2 (locks ~61 bits for CQ messages)
        //   pass 8: call1 + call2 (locks ~61 bits for directed messages)
        if use_ap
            && let Some(ap) = ap_hint
            && ap.has_info()
        {
            let apmag = llr_set.llra.iter().map(|v| v.abs()).fold(0.0f32, f32::max) * 1.01;

            // Build multiple AP configurations (deepest first)
            let mut ap_passes: Vec<(ApHint, u8)> = Vec::new();

            // Pass 9/10/11: full 77-bit lock (call1+call2+response)
            // Equivalent to WSJT-X a4/a5/a6 for QSO in progress
            if ap.call1.is_some() && ap.call2.is_some() {
                for (rpt, pid) in [("RRR", 9u8), ("RR73", 10), ("73", 11)] {
                    let ap_full = ap.clone().with_report(rpt);
                    ap_passes.push((ap_full, pid));
                }
            }

            // Pass 7: CQ + call2 (expect "CQ DXCALL GRID", ~61 bits)
            if ap.call2.is_some() && ap.call1.is_none() {
                let ap7 = ap.clone().with_call1("CQ");
                ap_passes.push((ap7, 7));
            }

            // Pass 8: mycall + call2 (~61 bits)
            if ap.call1.is_some() && ap.call2.is_some() {
                ap_passes.push((ap.clone(), 8));
            }

            // Pass 6: call2 only (~33 bits, fallback)
            ap_passes.push((ap.clone(), 6));

            for (ap_cfg, pass_id) in &ap_passes {
                let (ap_mask, ap_llr_override) = ap_cfg.build_ap(apmag);
                let locked_bits = ap_mask.iter().filter(|&&m| m).count();
                let max_errors: u32 = strictness.ap_max_errors(locked_bits);

                for &(base_llr, _) in llr_variants {
                    let mut llr_ap = *base_llr;
                    for i in 0..LDPC_N {
                        if ap_mask[i] {
                            llr_ap[i] = ap_llr_override[i];
                        }
                    }

                    // Helper: validate AP decode result
                    let check_result =
                        |msg77: [u8; 77], hard_errors: u32| -> Option<DecodeResult> {
                            if hard_errors >= max_errors {
                                return None;
                            }
                            let text = super::message::unpack77(&msg77)?;
                            if !super::message::is_plausible_message(&text) {
                                return None;
                            }
                            // Verify AP-locked callsigns appear in decoded message
                            let upper = text.to_uppercase();
                            if let Some(ref c1) = ap_cfg.call1
                                && !upper.contains(&c1.to_uppercase())
                            {
                                return None;
                            }
                            if let Some(ref c2) = ap_cfg.call2
                                && !upper.contains(&c2.to_uppercase())
                            {
                                return None;
                            }
                            let itone = message_to_tones(&msg77);
                            let snr_db = compute_snr_db(cs, &itone);
                            Some(DecodeResult {
                                message77: msg77,
                                freq_hz: cand.freq_hz,
                                dt_sec: refined.dt_sec,
                                hard_errors,
                                sync_score: refined.score,
                                pass: *pass_id,
                                sync_cv,
                                snr_db,
                            })
                        };

                    // AP + BP
                    if let Some(bp) = bp_decode(&llr_ap, Some(&ap_mask), BP_MAX_ITER)
                        && let Some(r) = check_result(bp.message77, bp.hard_errors)
                    {
                        return Some(r);
                    }
                    // AP + OSD fallback
                    if depth == DecodeDepth::BpAllOsd
                        && let Some(osd) = osd_decode_deep(&llr_ap, 2)
                        && let Some(r) = check_result(osd.message77, osd.hard_errors)
                    {
                        return Some(r);
                    }
                }
            }
        }

        None
    };

    match eq_mode {
        EqMode::Off => try_decode(&cs_raw, true),
        EqMode::Local => {
            let mut cs_eq = cs_raw.clone();
            equalizer::equalize_local(&mut cs_eq);
            try_decode(&cs_eq, true)
        }
        EqMode::Adaptive => {
            let mut cs_eq = cs_raw.clone();
            equalizer::equalize_local(&mut cs_eq);
            if let Some(r) = try_decode(&cs_eq, true) {
                return Some(r);
            }
            try_decode(&cs_raw, true)
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────

/// Inner decode loop shared by [`decode_frame`] and [`decode_frame_subtract`].
///
/// `known`           — messages already decoded in earlier passes (skipped).
/// `precomputed_fft` — optional pre-computed 192k-point FFT cache; when `None`
///                     the cache is built internally from `audio`.
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
) -> (Vec<DecodeResult>, Vec<num_complex::Complex<f32>>) {
    let candidates = coarse_sync(audio, freq_min, freq_max, sync_min, freq_hint, max_cand);
    if candidates.is_empty() {
        let fft_cache = match precomputed_fft {
            Some(c) => c.to_vec(),
            None => build_fft_cache(audio),
        };
        return (Vec::new(), fft_cache);
    }

    let fft_cache = match precomputed_fft {
        Some(c) => c.to_vec(),
        None => build_fft_cache(audio),
    };

    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            process_candidate(
                cand, audio, &fft_cache, depth, strictness, known, eq_mode, None,
            )
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            process_candidate(
                cand, audio, &fft_cache, depth, strictness, known, eq_mode, None,
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

/// Decode a 15-second FT8 frame using successive signal subtraction.
///
/// Runs three decode passes with decreasing sync thresholds.  After each
/// pass every newly decoded signal is subtracted from the residual audio,
/// revealing weaker signals that were previously hidden.
///
/// | Pass | sync_min factor | OSD score min | Purpose |
/// |------|----------------|---------------|---------|
/// | 1    | 1.0×           | 2.5           | Strong signals (BP + OSD) |
/// | 2    | 0.75×          | 2.5           | Medium signals on residual |
/// | 3    | 0.5×           | 2.0           | Weak / spurious signals |
///
/// Pass 3 uses a lower OSD score threshold (`2.0` vs the normal `2.5`) to
/// also subtract signals that are marginal but have valid CRC — even if they
/// were questionable in the original audio, subtracting their reconstructed
/// waveform from the already-cleaned residual does more good than harm.
pub fn decode_frame_subtract(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = Vec::new();

    let passes: &[f32] = &[1.0, 0.75, 0.5];

    for &factor in passes {
        let (new, _) = decode_frame_inner(
            &residual,
            freq_min,
            freq_max,
            sync_min * factor,
            freq_hint,
            depth,
            max_cand,
            strictness,
            &all_results,
            EqMode::Off,
            None,
        );

        for r in &new {
            // QSB gate: if Costas-array power CV > 0.3 the channel is time-varying
            // and the amplitude estimate is less accurate — use half gain to avoid
            // over-subtraction artefacts that would corrupt later passes.
            let sub_gain = if r.sync_cv > 0.3 { 0.5 } else { 1.0 };
            subtract_signal_weighted(&mut residual, r, sub_gain);
        }
        all_results.extend(new);
    }

    all_results
}

/// Phase-2 subtract decode: accepts Phase-1 results as `known` and an
/// optional pre-computed FFT cache for the first pass.
///
/// Internally runs three subtract passes (sync_min × 1.0 / 0.75 / 0.5).
/// The first pass reuses `precomputed_fft` when available; subsequent
/// passes recompute the FFT from the post-subtraction residual.
///
/// Returns only **newly** decoded messages (those not in `known`).
pub fn decode_frame_subtract_with_known(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    precomputed_fft: Option<FftCache>,
) -> Vec<DecodeResult> {
    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = known.to_vec();
    let known_count = known.len();

    let passes: &[f32] = &[1.0, 0.75, 0.5];

    for (i, &factor) in passes.iter().enumerate() {
        // Reuse the pre-computed FFT cache only for the first pass
        // (the audio hasn't been modified yet).
        let fft = if i == 0 {
            precomputed_fft.as_deref()
        } else {
            None
        };

        let (new, _) = decode_frame_inner(
            &residual,
            freq_min,
            freq_max,
            sync_min * factor,
            freq_hint,
            depth,
            max_cand,
            strictness,
            &all_results,
            EqMode::Off,
            fft,
        );

        for r in &new {
            let sub_gain = if r.sync_cv > 0.3 { 0.5 } else { 1.0 };
            subtract_signal_weighted(&mut residual, r, sub_gain);
        }
        all_results.extend(new);
    }

    // Return only the newly decoded messages (exclude `known`).
    all_results.split_off(known_count)
}

// ────────────────────────────────────────────────────────────────────────────
// Convenience: sniper-mode decode (single target frequency, narrow band)

/// Sniper-mode decode: search only within ±250 Hz of `target_freq`.
///
/// Intended for use after a 500 Hz hardware BPF.  The search band is
/// narrowed to `target_freq ± 250 Hz` and `sync_min` is lowered to 0.8
/// because the BPF removes strong adjacent signals that would otherwise
/// raise the noise floor.
///
/// `sync_cv` (Costas-array power coefficient of variation) is computed for
/// each decoded result and can be used downstream as a channel-quality
/// indicator for the Phase 3 adaptive equaliser.
pub fn decode_sniper(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_sniper_eq(audio, target_freq, depth, max_cand, EqMode::Off)
}

/// Sniper-mode decode with configurable equalizer.
///
/// Same as [`decode_sniper`] but allows enabling the adaptive equalizer
/// to correct BPF edge distortion.
pub fn decode_sniper_eq(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
    max_cand: usize,
    eq_mode: EqMode,
) -> Vec<DecodeResult> {
    decode_sniper_ap(audio, target_freq, depth, max_cand, eq_mode, None)
}

/// Sniper-mode decode with equalizer and A Priori hints.
///
/// The full sniper pipeline: hardware BPF simulation + adaptive EQ +
/// AP-assisted BP decode.  When `ap_hint` provides known callsigns,
/// the BP decoder locks those bits at high confidence, effectively
/// reducing the number of unknown bits and lowering the decode threshold.
///
/// # Example
/// ```ignore
/// let ap = ApHint::new().with_call1("CQ").with_call2("3Y0Z");
/// let results = decode_sniper_ap(
///     &audio, 1000.0, DecodeDepth::BpAllOsd, 20,
///     EqMode::Adaptive, Some(&ap),
/// );
/// ```
pub fn decode_sniper_ap(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
    max_cand: usize,
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    decode_sniper_inner(audio, target_freq, depth, max_cand, eq_mode, ap_hint, 0.8)
}

/// Sniper-mode decode with in-band Successive Interference Cancellation (SIC).
///
/// Pass 1 decodes all signals in ±250 Hz.  Any decoded signal more than 25 Hz
/// away from `target_freq` is subtracted from the audio.  Pass 2 then
/// re-decodes the residual with a relaxed sync threshold, recovering targets
/// that were masked by in-band interferers.
///
/// This is particularly effective when 2–3 stronger stations reside within the
/// 500 Hz BPF window alongside the target.  Falls back to a single-pass result
/// when no interferers are found (zero extra cost).
pub fn decode_sniper_sic(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
    max_cand: usize,
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    // Pass 1: decode everything in ±250 Hz at normal sync threshold.
    let pass1 = decode_sniper_inner(audio, target_freq, depth, max_cand, eq_mode, ap_hint, 0.8);

    // Subtract non-target signals (those > 25 Hz away from target_freq).
    let mut residual: Vec<i16> = audio.to_vec();
    let mut subtracted = false;
    for r in &pass1 {
        if (r.freq_hz - target_freq).abs() > 25.0 {
            // QSB gate: partial subtraction for time-varying channels.
            let gain = if r.sync_cv > 0.3 { 0.5 } else { 1.0 };
            subtract_signal_weighted(&mut residual, r, gain);
            subtracted = true;
        }
    }

    if !subtracted {
        return pass1;
    }

    // Pass 2: re-decode residual with relaxed sync_min to catch the target.
    let pass2 = decode_sniper_inner(
        &residual,
        target_freq,
        depth,
        max_cand,
        eq_mode,
        ap_hint,
        0.6,
    );

    // Merge, deduplicating by message77.
    let mut results = pass1;
    for r in pass2 {
        if !results.iter().any(|x| x.message77 == r.message77) {
            results.push(r);
        }
    }
    results
}

fn decode_sniper_inner(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
    max_cand: usize,
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
    sync_min: f32,
) -> Vec<DecodeResult> {
    let freq_min = (target_freq - 250.0).max(100.0);
    let freq_max = (target_freq + 250.0).min(5900.0);

    let candidates = coarse_sync(
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

    let fft_cache = build_fft_cache(audio);

    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            process_candidate(
                cand,
                audio,
                &fft_cache,
                depth,
                DecodeStrictness::Normal,
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
                DecodeStrictness::Normal,
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
    results
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Silence produces no decoded messages and does not panic.
    #[test]
    fn silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = decode_frame(&audio, 200.0, 2800.0, 1.0, None, DecodeDepth::Bp, 10);
        assert!(results.is_empty(), "silence should decode nothing");
    }

    /// Sniper mode on silence also produces no decoded messages.
    #[test]
    fn sniper_silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = decode_sniper(&audio, 1000.0, DecodeDepth::Bp, 10);
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

        let results = decode_frame(&audio, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200);
        assert!(!results.is_empty(), "should decode the signal");
        let dt = results[0].dt_sec;
        eprintln!("DT = {dt:+.3} s (expected ≈ 0.0)");
        assert!(dt.abs() < 0.5, "DT={dt} is too far from 0");
    }
}
