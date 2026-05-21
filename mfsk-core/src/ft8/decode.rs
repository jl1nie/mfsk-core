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
    let mut all_results: Vec<DecodeResult> = Vec::new();

    let mut prev_total: usize = 0;
    for ipass in 0..3 {
        if ipass >= 1 && all_results.len() == prev_total {
            break;
        }
        prev_total = all_results.len();

        let spec = crate::ft8::decode_block::compute_spectrogram(&residual, freq_max);
        let candidates =
            crate::ft8::decode_block::coarse_sync(&spec, freq_min, freq_max, sync_min, max_cand);
        drop(spec);
        if candidates.is_empty() {
            continue;
        }

        let fft_cache = build_fft_cache(&residual);
        for cand in &candidates {
            let r = match process_candidate(
                cand,
                &residual,
                &fft_cache,
                depth,
                strictness,
                &all_results,
                EqMode::Off,
                ap_hint,
            ) {
                Some(r) => r,
                None => continue,
            };
            // Dedup against earlier passes' accepted decodes.
            if all_results.iter().any(|x| x.message77 == r.message77) {
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
            subtract_signal_lpf(&mut residual, &r);
            all_results.push(r);
        }
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
}
