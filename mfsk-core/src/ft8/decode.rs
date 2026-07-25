/// High-level FT8 decode pipeline.
///
/// Chains: downsample → coarse_sync → fine_sync → LLR → BP decode
use alloc::string::{String, ToString};
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

pub use super::equalizer::EqMode;
use super::{
    downsample::build_fft_cache,
    equalizer,
    llr::sync_quality,
    message::pack28,
    params,
    params::{BP_MAX_ITER, LDPC_N},
    subtract::subtract_signal_lpf,
    sync::SyncCandidate,
};

// ────────────────────────────────────────────────────────────────────────────
// Public types

/// Opaque FFT cache produced by [`decode_frame_with_cache`] (Phase 1),
/// consumed by [`decode_frame_subtract_with_known`] (Phase 2).
pub type FftCache = Vec<num_complex::Complex<f32>>;

/// Decoding depth: which LLR sets and passes to attempt.
///
/// `Bp` (single-metric, no all-variants pass) was retired in 0.7.0 —
/// no production consumer was found by issue #74, and the cheapest
/// staircase rung wasn't a power-budget escape hatch (embedded ship
/// runs `BpAll` end-to-end inside the slot budget already).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecodeDepth {
    /// BP with all four metric variants (a, b, c, d).
    BpAll,
    /// BP (all four variants) then OSD order-1 fallback when BP fails.
    BpAllOsd,
    /// BP across `a`, `d`, `b` only — skips the heavy nsym=3 `c`
    /// variant. Used by the embedded port to bypass the
    /// `compute_llr_partial(3)` call (~5× cost of variant `b`) for
    /// failed candidates. Empirically (S3 qso3_busy log,
    /// 2026-05-21) every decode lands at `pass=0` (variant `a`) on
    /// busy-band reference WAVs, so `c` contributes no extra
    /// decodes and is pure overhead on a power-budgeted target.
    ///
    /// The lighter variants `d` (nsym=1 bit-normalised, free reuse
    /// of step-1 LLR) and `b` (nsym=2, ~1.5× variant `a` cost) are
    /// retained as safety nets for marginal SNR / fading cases.
    BpAllNoNsym3,
    /// BP across `a` (Step 1) and `d` (free-reuse bit-normalised
    /// nsym=1) only — drops both the nsym=2 `b` and nsym=3 `c`
    /// LLR computations + their BP attempts. Host A/B sweep
    /// (`ft8_no_nsym3_sweep`, 5 in-repo WAVs) showed `b` and `c`
    /// contribute zero extra decodes vs `a` alone; the lighter
    /// `BpAllNoNsym3` only dropped `c`, this one drops `b` too.
    ///
    /// Embedded ship target — on a power-budgeted MCU each failed
    /// candidate now costs only the Step-1 BP plus a free-LLR
    /// `d` re-BP (~2×T1 instead of ~8.5×T1 under `BpAll`).
    BpVariantsAd,
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
        None,
    )
    .0
}

/// Wide-band decode with an a-priori (AP) callsign / grid hint applied
/// to every candidate.
///
/// Unlike [`decode_sniper_ap`] which scans only ±250 Hz around a target
/// frequency, this function scans the full `freq_min..freq_max` band and
/// attempts AP-assisted decoding on each sync candidate. Useful when an
/// operator has an active QSO and wants the whole band searched with
/// the known callsigns biasing FEC LLRs:
///
/// ```ignore
/// use mfsk_core::ft8::decode::{decode_frame_with_ap, ApHint, DecodeDepth};
/// let ap = ApHint::new().with_call1("CQ").with_call2("K1ABC");
/// let results = decode_frame_with_ap(
///     &audio, 100.0, 3000.0, 1.0, None,
///     DecodeDepth::BpAllOsd, 50, Some(&ap),
/// );
/// ```
///
/// AP gain is typically 1–3 dB at the FT8 decode threshold when at
/// least one of `call1` / `call2` matches a station actually on air.
/// When the hint is wrong, decode quality degrades only slightly
/// because the AP path is gated behind sync-quality and BP score
/// checks; spurious AP-locked decodes are caught by the post-FEC CRC.
pub fn decode_frame_with_ap(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    decode_frame_with_ap_full(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        DecodeStrictness::Normal,
        max_cand,
        ap_hint,
    )
    .0
}

/// Wide-band decode with AP hint plus configurable strictness, returning
/// the FFT cache for downstream pipelined passes.
///
/// This is the "full" form of [`decode_frame_with_ap`]: it exposes the
/// [`DecodeStrictness`] knob and returns the 192 k-point FFT cache so a
/// follow-up [`decode_frame_subtract_with_known`] (or
/// [`decode_frame_subtract_with_known_and_ap`]) call can reuse it without
/// recomputing.
///
/// `ap_hint = None` reproduces the legacy [`decode_frame_with_cache`]
/// pipeline bit-for-bit (no AP bits locked).
pub fn decode_frame_with_ap_full(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    max_cand: usize,
    ap_hint: Option<&ApHint>,
) -> (Vec<DecodeResult>, FftCache) {
    decode_frame_inner(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        &[],
        EqMode::Off,
        None,
        ap_hint,
    )
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
    crate::ft8::decode_block::fill_symbol_spectra(
        &mut cs_raw,
        audio,
        refined.freq_hz,
        refined.dt_sec,
        crate::ft8::decode_block::SymMask::SyncOnly,
        Some(fft_cache),
    );
    crate::ft8::decode_block::fill_symbol_spectra(
        &mut cs_raw,
        audio,
        refined.freq_hz,
        refined.dt_sec,
        crate::ft8::decode_block::SymMask::DataOnly,
        Some(fft_cache),
    );
    let nsync = sync_quality(&cs_raw);
    if nsync <= 6 {
        return None;
    }

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
    decode_frame_subtract_with_ap(
        audio, freq_min, freq_max, sync_min, freq_hint, depth, max_cand, strictness, None,
    )
}

/// Like [`decode_frame_subtract`] but forwards an `ap_hint` to every
/// per-candidate decode in every subtract pass.
///
/// `ap_hint = None` reproduces [`decode_frame_subtract`] bit-for-bit.
pub fn decode_frame_subtract_with_ap(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    // **WSJT-X-faithful** 3-pass + sequential SIC, mirroring the
    // structure of `decode_block::decode_block_multipass` (the embedded
    // path) which is a faithful port of `lib/ft8/ft8b.f90:432-437`.
    //
    // Two changes from the pre-v0.6.0 host implementation:
    //
    // 1. **Fixed `sync_min` across all 3 passes** (was 1.0 / 0.75 / 0.5).
    //    Progressive relaxation lets phantoms slip through later passes
    //    when SIC artefacts dominate the residual; WSJT-X holds the
    //    threshold and skips later passes when no new decodes come out.
    //
    // 2. **Sequential subtract within each pass** (was batch-after-pass).
    //    Each accepted decode immediately subtracts from the residual so
    //    the *next* candidate in the same pass sees a cleaner spectrum.
    //    This is what surfaces -13 to -18 dB signals sitting beneath
    //    strong neighbours (the JTDX-extras shape on `qso3_busy.wav`).
    //    Without sequential subtract, all candidates in a pass see the
    //    same raw residual and weak signals stay masked.
    //
    // Pass termination matches WSJT-X: pass 2 skips when pass 1 returned
    // 0 decodes; pass 3 skips when pass 2 returned no NEW decodes.
    let _ = freq_hint;

    let mut residual = audio.to_vec();
    sic_inner_passes(
        &mut residual,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        strictness,
        &[],
        ap_hint,
    )
}

/// Shared inner loop: up to 3 sub-passes of coarse_sync + per-candidate
/// decode, subtracting each accepted decode from `residual` immediately
/// (sequential, not batch-after-pass) so later candidates in the same
/// sub-pass see a cleaner spectrum. This is WSJT-X's `ft8b.f90:432-437`
/// `do ipass=1,npass` structure. Factored out of
/// [`decode_frame_subtract_with_ap`] so [`decode_frame_subtract_staged_with_ap`]
/// can reuse it at checkpoint A and checkpoint C (issue #180) without
/// duplicating the pass/dedup/subtract logic.
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
    let mut all_results: Vec<DecodeResult> = Vec::new();

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

        let fft_cache = build_fft_cache(residual);
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

    all_results
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
// subtract-after" pass (`decode_frame_subtract_with_ap` above) can't
// reproduce no matter how the sync threshold is tuned, because DL8YHR's
// neighbours were never subtracted from *before* the residual it's found
// in was assembled.
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

/// Like [`decode_frame_subtract`] but structured as WSJT-X's disk-decode
/// checkpoint architecture (see the module doc above) instead of a flat
/// 3-pass SIC over the whole buffer. `audio` should be a full 15 s / 12 kHz
/// slot (≤ 180_000 samples) for the staging to have any effect; shorter
/// buffers (e.g. synthetic test fixtures) fall back to
/// [`decode_frame_subtract`] unchanged.
pub fn decode_frame_subtract_staged(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    decode_frame_subtract_staged_with_ap(
        audio, freq_min, freq_max, sync_min, freq_hint, depth, max_cand, strictness, None,
    )
}

/// Like [`decode_frame_subtract_staged`] but forwards an `ap_hint` to
/// every per-candidate decode in every checkpoint.
#[allow(clippy::too_many_arguments)]
pub fn decode_frame_subtract_staged_with_ap(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    decode_frame_subtract_staged_with_ap_inner(
        audio, freq_min, freq_max, sync_min, freq_hint, depth, max_cand, strictness, ap_hint,
    )
    .0
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
    if audio.len() < A_SAMPLES {
        let r = decode_frame_subtract_with_ap(
            audio, freq_min, freq_max, sync_min, freq_hint, depth, max_cand, strictness, ap_hint,
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
        // more, never less.
        let r = decode_frame_subtract_with_ap(
            audio, freq_min, freq_max, sync_min, freq_hint, depth, max_cand, strictness, ap_hint,
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
    let mut deferred: Vec<DecodeResult> = Vec::new();
    for r in &early_results {
        if r.dt_sec < dt_fit_limit_b {
            subtract_signal_lpf(&mut buf_b, r);
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
    for r in &deferred {
        subtract_signal_lpf(&mut buf_c, r);
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

/// Phase-2 subtract decode: accepts Phase-1 results as `known` and an
/// optional pre-computed FFT cache for the first pass.
///
/// Internally runs three subtract passes (sync_min × 1.0 / 0.75 / 0.5).
/// The first pass reuses `precomputed_fft` when available; subsequent
/// passes recompute the FFT from the post-subtraction residual.
///
/// Caller-supplied `known` signals are subtracted from the working
/// buffer after Pass 0 (before Pass 1 / Pass 2 begin), so subsequent
/// passes see a residual with all known signals removed. Without this,
/// strong known carriers would re-decode in later passes and crowd out
/// the new candidates this function exists to surface.
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
    decode_frame_subtract_with_known_and_ap(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        known,
        precomputed_fft,
        None,
    )
}

/// Like [`decode_frame_subtract_with_known`] but also forwards an
/// `ap_hint` to every per-candidate decode in every subtract pass.
///
/// `ap_hint = None` reproduces [`decode_frame_subtract_with_known`]
/// bit-for-bit.
///
/// Caller-supplied `known` signals are subtracted from the working
/// buffer after Pass 0 (before Pass 1 / Pass 2 begin), so subsequent
/// passes see a residual with all known signals removed. This applies
/// regardless of whether `ap_hint` is supplied.
#[allow(clippy::too_many_arguments)]
pub fn decode_frame_subtract_with_known_and_ap(
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
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    let (results, _residual) = decode_frame_subtract_with_known_and_ap_inner(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        known,
        precomputed_fft,
        ap_hint,
    );
    results
}

/// Shared SIC loop for `decode_frame_subtract_with_known_and_ap` and its
/// `#[cfg(test)]` debug counterpart. Returns the newly-decoded messages
/// (excluding `known`) plus the residual buffer after all passes
/// complete.
///
/// Pass 0 uses the pre-computed FFT (built from the *original* audio) to
/// discover any signals missed in Phase 1; only after that do we subtract
/// both the caller-supplied `known` signals and the newly discovered
/// signals from the residual. Subtracting before Pass 0 would require
/// either (a) recomputing the FFT cache against the modified residual
/// or (b) using a stale cache that no longer matches the audio. Either
/// would defeat the cache-reuse optimization.
#[allow(clippy::too_many_arguments)]
fn decode_frame_subtract_with_known_and_ap_inner(
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
    ap_hint: Option<&ApHint>,
) -> (Vec<DecodeResult>, Vec<i16>) {
    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = known.to_vec();
    let known_count = known.len();

    let passes: &[f32] = &[1.0, 0.75, 0.5];
    let mut residual_dirty = false;

    for (i, &factor) in passes.iter().enumerate() {
        // Reuse the pre-computed FFT cache only on Pass 0, and only while
        // `residual` has not yet been modified by any subtraction step.
        let fft = if i == 0 && !residual_dirty {
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
            ap_hint,
        );

        // After Pass 0, also subtract every `known` signal supplied by
        // the caller (typically Phase 1 results). Without this, those
        // strong known signals continue to mask weaker ones in Pass 1
        // and Pass 2 of the SIC loop, defeating the whole purpose of
        // successive interference cancellation.
        if i == 0 {
            for r in known {
                subtract_signal_lpf(&mut residual, r);
            }
            if !known.is_empty() {
                residual_dirty = true;
            }
        }

        for r in &new {
            subtract_signal_lpf(&mut residual, r);
        }
        if !new.is_empty() {
            residual_dirty = true;
        }
        all_results.extend(new);
    }

    // Return only the newly decoded messages (exclude `known`).
    (all_results.split_off(known_count), residual)
}

/// Test-only variant that returns the residual buffer after all
/// subtraction passes complete, alongside the decoded messages. Used
/// by regression tests that need to verify successive-interference-
/// cancellation actually cancels the caller-supplied `known` signals
/// from the residual.
///
/// Implemented as a thin shim over the same private inner that the
/// production [`decode_frame_subtract_with_known_and_ap`] uses, so the
/// SIC pass structure, gain schedule, FFT-cache validity logic, and
/// `known`-subtraction placement can never drift between the two. The
/// only difference is that this variant exposes the residual buffer
/// the production function discards.
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_frame_subtract_with_known_and_ap_debug_residual(
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
    ap_hint: Option<&ApHint>,
) -> (Vec<DecodeResult>, Vec<i16>) {
    decode_frame_subtract_with_known_and_ap_inner(
        audio,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        known,
        precomputed_fft,
        ap_hint,
    )
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
///     EqMode::Local, Some(&ap),
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
            subtract_signal_lpf(&mut residual, r);
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

    // Sniper-mode: freq_hint (=target_freq) used to promote candidates
    // near the target via the legacy core::sync::coarse_sync path. After
    // the v0.6 consolidation in #48, decode_block::coarse_sync does not
    // honour hints; the ±250 Hz freq_min/freq_max band above does most
    // of the work the hint used to.
    let spec = crate::ft8::decode_block::compute_spectrogram(audio, freq_max);
    let candidates =
        crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
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

    /// `decode_frame_with_ap` accepts the AP hint and round-trips a
    /// clean self-synthesised signal with the hint matching. Doesn't
    /// directly assert the AP gain (that needs a low-SNR fixture);
    /// just guards against signature drift and validates the
    /// "hint-aware decode of a perfect signal still succeeds" invariant.
    #[test]
    fn decode_frame_with_ap_round_trips_clean_signal() {
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
        let results = decode_frame_with_ap(
            &audio,
            100.0,
            3000.0,
            1.0,
            None,
            DecodeDepth::BpAllOsd,
            50,
            Some(&ap),
        );
        assert!(
            results.iter().any(|r| r.message77 == m77),
            "expected to decode the self-synthesized signal with matching AP hint"
        );
    }

    /// `decode_frame_with_ap` with `ap_hint = None` should produce
    /// exactly the same results as `decode_frame` (legacy path).
    #[test]
    fn decode_frame_with_ap_none_matches_legacy() {
        use crate::ft8::wave_gen::{message_to_tones, tones_to_i16};
        use crate::msg::wsjt77::pack77;

        let m77 = pack77("CQ", "W7VV", "CN87").expect("pack77");
        let tones = message_to_tones(&m77);
        let samples = tones_to_i16(&tones, 1200.0, 18_000);

        let mut audio = vec![0i16; 15 * 12_000];
        let off = 6_000usize;
        let len = samples.len().min(audio.len() - off);
        audio[off..off + len].copy_from_slice(&samples[..len]);

        let r_legacy = decode_frame(&audio, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 50);
        let r_ap_none = decode_frame_with_ap(
            &audio,
            100.0,
            3000.0,
            1.0,
            None,
            DecodeDepth::BpAllOsd,
            50,
            None,
        );
        assert_eq!(
            r_legacy.iter().map(|r| r.message77).collect::<Vec<_>>(),
            r_ap_none.iter().map(|r| r.message77).collect::<Vec<_>>(),
            "ap_hint=None must match legacy decode_frame exactly"
        );
    }

    /// Compile-shape: `decode_frame_with_ap_full` accepts all parameter
    /// combinations and returns the FFT cache alongside the decode list.
    /// On a silent buffer the result list must be empty and the cache
    /// must be non-empty (FFT is built unconditionally).
    #[test]
    fn decode_frame_with_ap_full_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("K1ABC");

        // ap_hint = None
        let (r0, c0) = decode_frame_with_ap_full(
            &audio,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::BpAll,
            DecodeStrictness::Normal,
            10,
            None,
        );
        assert!(r0.is_empty());
        assert!(!c0.is_empty(), "FFT cache should be returned");

        // ap_hint = Some, strictness = Strict
        let (r1, c1) = decode_frame_with_ap_full(
            &audio,
            200.0,
            2800.0,
            1.0,
            Some(1500.0),
            DecodeDepth::BpAllOsd,
            DecodeStrictness::Strict,
            10,
            Some(&ap),
        );
        assert!(r1.is_empty());
        assert!(!c1.is_empty());
    }

    /// Compile-shape: `decode_frame_subtract_with_ap` accepts an AP hint
    /// and returns no decodes on silence.
    #[test]
    fn decode_frame_subtract_with_ap_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("W7VV");

        let r_none = decode_frame_subtract_with_ap(
            &audio,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::BpAll,
            10,
            DecodeStrictness::Normal,
            None,
        );
        assert!(r_none.is_empty());

        let r_some = decode_frame_subtract_with_ap(
            &audio,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::BpAll,
            10,
            DecodeStrictness::Normal,
            Some(&ap),
        );
        assert!(r_some.is_empty());
    }

    /// Compile-shape: `decode_frame_subtract_with_known_and_ap` accepts
    /// the full parameter set (known list + FFT cache + AP hint) and
    /// returns no decodes on silence.
    #[test]
    fn decode_frame_subtract_with_known_and_ap_silence_shape() {
        let audio = vec![0i16; 15 * 12_000];
        let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
        let known: Vec<DecodeResult> = Vec::new();

        let r_none = decode_frame_subtract_with_known_and_ap(
            &audio,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::BpAll,
            10,
            DecodeStrictness::Normal,
            &known,
            None,
            None,
        );
        assert!(r_none.is_empty());

        let r_some = decode_frame_subtract_with_known_and_ap(
            &audio,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::BpAll,
            10,
            DecodeStrictness::Normal,
            &known,
            None,
            Some(&ap),
        );
        assert!(r_some.is_empty());
    }

    /// Regression test for the Phase-2 SIC correctness bug: when caller-supplied
    /// `known` signals are *not* subtracted from the residual, a strong known
    /// signal continues to mask weaker signals throughout the SIC loop. With
    /// the fix in place, the residual is cleaned of the known signal after
    /// Pass 0 so subsequent passes operate on a near-zero baseline at the
    /// known signal's frequency.
    ///
    /// We assert this directly by measuring the residual energy at the known
    /// signal's narrow band before vs. after the function runs. Without the
    /// fix, residual energy at f0 ≈ original input energy at f0. With the
    /// fix, it drops by an order of magnitude.
    #[test]
    fn decode_frame_subtract_with_known_and_ap_subtracts_known_before_phase2() {
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
        let phase1 = decode_frame(&audio, 200.0, 2800.0, 1.0, None, DecodeDepth::BpAllOsd, 50);
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

        let (new_results, residual) = decode_frame_subtract_with_known_and_ap_debug_residual(
            &audio,
            200.0,
            2800.0,
            1.0,
            None,
            DecodeDepth::BpAllOsd,
            50,
            DecodeStrictness::Normal,
            &known_results,
            None,
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
        let results = decode_frame(&audio, 200.0, 2800.0, 1.0, None, DecodeDepth::BpAll, 10);
        assert!(results.is_empty(), "silence should decode nothing");
    }

    /// Sniper mode on silence also produces no decoded messages.
    #[test]
    fn sniper_silence_no_decode() {
        let audio = vec![0i16; 15 * 12_000];
        let results = decode_sniper(&audio, 1000.0, DecodeDepth::BpAll, 10);
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
                    DecodeDepth::BpAllOsd,
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
            DecodeDepth::BpAllOsd,
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
}
