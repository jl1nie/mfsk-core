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
pub(super) use crate::engine::protocol::SAMPLE_RATE_HZ;

/// LLR / BP scalar for the hot loop. `Q11i16` (i16, ±16 range,
/// 1/2048 resolution, ~12 KB BP scratch on FT8 LDPC(174,91)) under
/// `fixed-point-llr`; `f32` otherwise (host / FPU targets). Both go
/// through the same generic NMS implementation in `fec::ldpc::bp`.
///
/// Shared by `process_candidates` and `decode.rs`'s host driver, which
/// must agree: `process_one_candidate_inner`'s signature is written
/// against this alias (one definition since #420; the two used to be
/// kept in sync by hand).
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
///
/// - 0.10.x: `fixed-point` stopped implying it (issue #349). On the
///   board this was built for, `Q11i16` BP measures **0.85x f32** —
///   22 813 us against 19 455 on a CoreS3, same LLRs and same
///   `max_iter` — because the LX7 has an f32 FPU and the saturating
///   i16 helpers cost more than the narrower loads save. The scalar
///   is now `fixed-point-llr`, off by default; `fixed-point` keeps
///   the u16 spectrogram, which is where its 351 KB actually comes
///   from.
#[cfg(feature = "fixed-point-llr")]
pub(in crate::ft8) type LlrT = crate::engine::scalar::Q11i16;
#[cfg(not(feature = "fixed-point-llr"))]
pub(in crate::ft8) type LlrT = f32;

/// Which of WSJT-X's decode passes a candidate is being decoded in
/// (`ft8_decode.f90`'s `do ipass=1,npass`, v3.0.0 onward), carried down
/// to [`process_one_candidate_inner`](super::process_candidates) so a
/// pass can change how the candidate is decoded, not only which
/// candidates it sees (#439).
///
/// `imetric` is `ft8b.f90`'s argument of that name: 1 = the metric
/// `|cs|` (passes 1), 2 = `|cs|²` (passes 2 and 3). Every caller that has
/// no pass structure — the embedded driver, the single-pass host path,
/// sniper — decodes as pass 1, as WSJT-X's `-d1` first pass does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::ft8) struct PassCtx {
    pub imetric: u8,
    /// `ndepth <= 2` (`jt9 -d1/-d2`, [`WsjtxDepth::D1`](crate::ft8::decode::WsjtxDepth)
    /// and `D2`): a higher nsync floor, see [`PassCtx::nsync_floor`].
    pub low_depth: bool,
    /// `ncontest != 0`: a contest is being worked, so `ft8b.f90` does not
    /// drop `/R` and `TU; ` messages (see [`Self::FIRST`]'s neighbours in
    /// `process_candidates::wsjtx_quirky`). Off by default, as `ncontest=0`.
    pub contest: bool,
}

impl PassCtx {
    /// Pass 1: `imetric` 1.
    pub(in crate::ft8) const FIRST: PassCtx = PassCtx {
        imetric: 1,
        low_depth: false,
        contest: false,
    };

    /// This context with `ncontest != 0` when `on` is set.
    pub(in crate::ft8) const fn contest(self, on: bool) -> PassCtx {
        PassCtx {
            contest: on,
            ..self
        }
    }

    /// This context for a `jt9 -d1/-d2` tier (`ndepth <= 2`) when `low` is set.
    pub(in crate::ft8) const fn low_depth(self, low: bool) -> PassCtx {
        PassCtx {
            low_depth: low,
            ..self
        }
    }

    /// `ft8b.f90`'s hard-sync gate: a candidate whose Costas hard-decision
    /// count `nsync` is at or below this is dropped before any decode
    /// (`syncmin=6; if(imetric.eq.2) syncmin=7; if(ndepth.le.2) syncmin=8`,
    /// v3.0.0 onward). The squared metric of passes 2 and 3 gets a floor one
    /// higher than pass 1's, and the `-d1/-d2` tiers a floor of 8 whatever the
    /// pass.
    pub(in crate::ft8) const fn nsync_floor(self) -> u32 {
        if self.low_depth {
            8
        } else if self.imetric == 2 {
            7
        } else {
            6
        }
    }

    /// `imetric` 2: the squared metric (`ft8b.f90`: `s2=s2**2`).
    pub(in crate::ft8) const fn squared(self) -> bool {
        self.imetric == 2
    }

    /// The context of 0-based decode round `round` (`ipass - 1`) of a
    /// decode whose first pass is `self`: `ft8_decode.f90` v3.0.0 sets
    /// `imetric=1` for pass 1 and `imetric=2` for passes 2 and 3; the tier
    /// (`low_depth`) carries over.
    pub(in crate::ft8) const fn round(self, round: usize) -> PassCtx {
        PassCtx {
            imetric: if round == 0 { 1 } else { 2 },
            ..self
        }
    }

    /// Whether 0-based round `round` runs, given `decodes_so_far` — every
    /// message decoded before it, including an earlier stage's
    /// (`ndecodes`, which `ft8_decode.f90` seeds with `ndec_early`).
    ///
    /// v3.0.0: pass 1 and pass 2 always run; pass 3 runs only if there is
    /// at least one decode (`if(ndecodes.eq.0) cycle`). Before 3.0.0 pass 2
    /// also needed a decode and pass 3 needed a *new* one; that skip
    /// rule is gone upstream (#439).
    pub(in crate::ft8) const fn round_runs(round: usize, decodes_so_far: usize) -> bool {
        round < 2 || decodes_so_far > 0
    }
}

/// Slot start offset (FT8 transmits 0.5 s into the slot).
pub(super) const TX_START_OFFSET_S: f32 = 0.5;

/// Coarse-sync ±lag search window (s) — WSJT-X's own `sync8.f90`
/// `JZ=62` window, i.e. ±2.5 s relative to the 0.5 s TX start.
/// Covers operators with sloppy slot timing or slow rigs.
///
/// Was 1.0 until issue #280. The narrower window was an embedded
/// compute trade (`coarse_sync` is linear in `n_lag`: 51 lag steps
/// at 1.0 s vs 127 at 2.5 s), taken on the assumption that recall
/// was otherwise unaffected. It wasn't: at 2.5 s, `qso3_busy.wav`
/// lost `K1BZM DK8NE -10` and `K1JT HA5WA 73`, which made the
/// narrow window quietly load-bearing for golden recall rather than
/// a pure speed knob.
///
/// That coupling is gone. Probing the real `jt9` binary
/// (`sync8.f90` + `ft8_decode.f90`, issue #280) showed WSJT-X finds
/// `K1BZM DK8NE` only through its *secondary* (full-`±JZ`) channel
/// at `jpeak2=14` — its fixed-`±mlag=10` primary (upstream moved to 13
/// in 3.0, which this port has not adopted — see `coarse_sync::MLAG`;
/// the probe was against 2.7) scores 0.95, far
/// under `syncmin` — and only on its second subtraction pass
/// (`nzhsym=50, ipass=2`), from candidate rank 35 of 337 with no
/// pass-1 truncation at all (`MAXCAND=600`). The equivalent
/// candidate is present in our list at every window width; what
/// dropped it at 2.5 s was `PASS1_LIMIT_DEFAULT=30` truncating
/// before it climbed the ranks. See
/// [`pass1_limit_for`](super::process_candidates::pass1_limit_for).
///
/// Embedded ship config (`max_cand ≤ 15`) keeps the 30-candidate
/// pass-1 cap unchanged, so the cost of this default is the wider
/// `sync2d` scan alone. Override per-call with `MFSK_SYNC_LAG_S`
/// when std is enabled — set it to 1.0 to recover the old window on
/// a tightly clock-synced deployment.
const SYNC_LAG_S_DEFAULT: f32 = 2.5;
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
