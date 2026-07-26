//! Shared types, constants, and runtime tunables for the
//! `decode_block` pipeline.
//!
//! ε.1 of the `docs/CLEANUP_2026_05.md` decode_block split. Items here
//! are the cross-cutting bits referenced by ≥2 of the per-stage
//! submodules (spectrogram, coarse_sync, fill_symbol_spectra,
//! process_candidates) plus the public-facade entry functions. The
//! later ε.N PRs move per-stage code into siblings of this file under
//! `mfsk-core/src/ft8/decode_block/`.
//!
//! Re-exported by the parent module (`decode_block.rs`) so external
//! callers see the same paths (`mfsk_core::ft8::decode_block::AudioSample`,
//! `…::NFFT_SPEC`, …) as before the move.

use super::super::params::NSPS;

// ── Audio sample trait ──────────────────────────────────────────────────────

/// Trait for audio sample types accepted by `decode_block`. Lets the
/// caller hand in either `i16` (the canonical FT8 PCM) or `i8`
/// (half-storage, ~45 dB SQNR — plenty for FT8's -24 dB threshold —
/// useful when the slot needs to fit in scarce internal SRAM on
/// embedded targets where PSRAM access is the bottleneck).
///
/// The `to_f32` implementation must produce values on the same
/// amplitude scale as `i16` so the LLR computation downstream keeps
/// its calibration; for `i8` we therefore multiply by 256.
///
/// `Copy` only — no `Sync` supertrait. An earlier version added `+
/// Sync` here so `&[S]` could cross a `rayon` task boundary in the
/// old auto-AP per-callsign parallel loop (issue #117); that whole
/// mechanism was removed in the 0.8.0 `DecodeDepth` redesign (issue
/// #182 follow-up — it was firing unconditionally on every `depth.osd`
/// call regardless of AP usage, for zero measured recall benefit once
/// the OSD `bp_llr_zsum` fix landed), and no other `AudioSample`-
/// generic function crosses a thread boundary. Re-add `+ Sync` only
/// alongside a real generic-over-`S` parallel caller, not preemptively.
pub trait AudioSample: Copy {
    fn to_f32(self) -> f32;
    /// Promote to i16 range. i8 → i16 via `<<8`; i16 → i16
    /// identity. Used by the fixed-point FFT input path.
    fn to_i16(self) -> i16;
}

impl AudioSample for i16 {
    #[inline]
    fn to_f32(self) -> f32 {
        self as f32
    }
    #[inline]
    fn to_i16(self) -> i16 {
        self
    }
}

impl AudioSample for i8 {
    #[inline]
    fn to_f32(self) -> f32 {
        // Match i16 amplitude scale (multiply by 2^8). LLR
        // calibration (LLR_SCALE in ft8::params) thus stays
        // valid without per-sample-type rescaling.
        (self as i32 * 256) as f32
    }
    #[inline]
    fn to_i16(self) -> i16 {
        (self as i16) << 8
    }
}

// ── Tunables ────────────────────────────────────────────────────────────────

/// Per-symbol spectrogram FFT length. Power of two.
///
/// Caps differ by backend:
/// - **fc32 (f32 path)**: 4096, limited by esp-dsp's bit-rev
///   lookup tables shipped only at sizes 16..4096
///   (`dsps_fft2r_bitrev_tables_fc32.c`). Requesting 8192 corrupts
///   the rev-table array inside `dsps_fft2r_fc32_ae32_`.
/// - **sc16 (`fixed-point` feature)**: no cap up to 32768; sc16
///   has no rev-table dependency and generates twiddles on the fly.
///
/// **NFFT=3840 = 2*NSPS**, matching WSJT-X `sync8.f90`'s `NFFT1`.
///
/// - `tone_step_bins = TONE_SPACING_HZ / df = 6.25 / (12000/3840) = 2.0`
///   exactly (integer), so each FT8 tone falls on a single FFT bin and
///   the rectangular-window sidelobes do not leak onto adjacent tones.
/// - Numerically identical scale to WSJT-X — `savg`, `sbase`, `xsig`,
///   `xsnr2` and the Costas-correlation score can be compared bin-for-bin
///   against WSJT-X reference output when debugging false decodes / SNR
///   reporting (no calibration constants required).
/// - Rectangular window throughout (no Hann); the previously needed
///   Hann compensation, multi-bin tone sum, and Hann-coherent-gain
///   pre-shift have all been removed.
///
/// Embedded (Xtensa, `fixed-point` feature) gets the same NFFT via a
/// 256 × 15 mixed-radix wrapper around esp-dsp's radix-2 256-pt FFT
/// (see `embedded-shared::esp_dsp_fft::MixedRadix3840Fft`). The 15-pt
/// PFA factor is in `mfsk-core/src/core/dsp/fft_15.rs` with hardcoded
/// 3-pt and 5-pt twiddles.
pub const NFFT_SPEC: usize = 3840;

/// Coarse-sync slide step (samples). **Quarter-symbol** (NSPS/4=480,
/// 40 ms, 372 frames per slot) — matches WSJT-X `ft8_params.f90`
/// `NSTEP=NSPS/4` exactly. The earlier setting NSPS/2 (=960, 184
/// frames) had half the dt resolution and was the dominant blocker
/// of `decode_block` parity with WSJT-X on busy slots: low-band
/// candidates (e.g. W0RSJ @400 Hz, N1PJT @466 Hz, KD2UGC @472 Hz on
/// qso3_busy) were either missed or the dt accuracy left BP unable
/// to lock. The previous comment claimed halving to NSPS killed
/// AWGN sensitivity — but that was vs NSPS, not NSPS/4 (the WSJT-X
/// choice), which had not been benchmarked.
// Mirrors `crate::ft8::params::NSTEP` — gated on `nstep-half` so
// embedded targets pick the NSPS/2 (= 960) variant the embedded
// `stage1_inc` builds spec at, instead of the WSJT-faithful NSPS/4
// (= 480) used on host. Both consts must agree because the score
// loop in `coarse_sync_inner` derives `m_base` from them.
#[cfg(not(feature = "nstep-half"))]
pub(super) const NSTEP: usize = NSPS / 4;
#[cfg(feature = "nstep-half")]
pub(super) const NSTEP: usize = NSPS / 2;

/// Steps per symbol — used to map symbol-index to time-step lag.
pub(super) const NSSY: i32 = (NSPS / NSTEP) as i32;

/// FT8 tone spacing (Hz).
pub(super) const TONE_SPACING_HZ: f32 = 6.25;

/// Regulariser added to `mean_others` in coarse_sync's ratio metric
/// `t / (mean_others + ε)`. On the fp path the u16 spectrogram
/// quantises noise bins to 0; on phantom carriers where the 7
/// non-Costas tones happen to quantise to 0 the bare ratio explodes
/// 100-1000× over real-signal scores and buries busy-band truth in
/// coarse_sync's top-N. ε ≈ a fraction of one u16 LSB at
/// `FP_SPEC_SHIFT=12` keeps the ratio finite without depressing
/// genuine weak-signal scores (AWGN -17.5 dB threshold preserved).
///
/// 0.5 was picked from a host sweep over real-QSO WAVs: ε ∈ {0.1,
/// 0.25, 0.5, 1.0, 2.0} — 0.25 and 0.5 both gave 8/13 truth in
/// top-30 on busy-band qso3 (was 4/13 with bare ratio); 0.5 had
/// slightly tighter top ranks. ε > 1.0 starts losing borderline
/// weak signals; ε < 0.25 leaks phantom inflation back in.
///
/// On the f32 path `mean_others` never quantises to 0 so ε is
/// dwarfed by typical t0_ref values and has no measurable effect.
const RATIO_EPS_DEFAULT: f32 = 0.5;
pub(super) fn ratio_eps() -> f32 {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_RATIO_EPS")
            && let Ok(v) = s.parse::<f32>()
        {
            return v;
        }
    }
    RATIO_EPS_DEFAULT
}

/// 12 kHz fixed sample rate.
pub(super) const SAMPLE_RATE_HZ: f32 = 12_000.0;

/// Slot start offset (FT8 transmits 0.5 s into the slot).
pub(super) const TX_START_OFFSET_S: f32 = 0.5;

/// Coarse-sync ±lag search window (s).
///
/// WSJT-X uses ±2.5 s — covers operators with sloppy slot timing
/// or slow rigs. Embedded targets running on a synced clock (NTP /
/// GPS) live well within ±1 s; halving the lag range cuts
/// `coarse_sync` work by ~60 % (linear in `n_lag`). If the live
/// timing source is loose, raise this back to 2.5.
const SYNC_LAG_S_DEFAULT: f32 = 1.0;
pub(super) fn sync_lag_s() -> f32 {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_SYNC_LAG_S")
            && let Ok(v) = s.parse::<f32>()
        {
            return v;
        }
    }
    SYNC_LAG_S_DEFAULT
}

/// Same NMS α as the bench-tuned default in `mfsk-core/src/fec/ldpc/bp.rs`.
pub(super) const NMS_ALPHA: f32 = 0.75;

/// `process_candidates` early-rejects cands whose full-21-symbol
/// `sync_quality` is at or below this threshold. Matches WSJT-X
/// `ft8b.f90:177` — `nsync ≤ 6 → bail`. Slower MCUs may raise this
/// at the cost of a few weak-signal decodes (the previous default
/// of 12 saved ~12-21 % stage-3 wall-clock); pass via the
/// `q_thresh` parameter on `process_candidates_into` /
/// `process_candidates_into_with_cs_scratch`.
pub const DEFAULT_Q_THRESH: u32 = 6;
