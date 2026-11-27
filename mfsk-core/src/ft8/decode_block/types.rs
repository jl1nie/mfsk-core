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

// Defined with the audio sources (`engine::source`): the sample type of a slot, whichever mode reads it.
pub use crate::engine::source::AudioSample;

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

/// The sample rate of the audio the **low-memory** decoders ([`decode_block_low_mem`](super::decode_block_low_mem),
/// [`coarse_sync_gated`](super::coarse_sync_gated)) read.
///
/// An enum and not a number: the rates this is built for are those whose symbol (0.16 s) is a whole number of
/// samples at a rate the 200 Hz baseband divides, and whose filters have been designed and measured. Anything else
/// is resampled to one of them by the caller.
///
/// **A deliberate divergence from WSJT-X**, which decodes FT8 at 12 kHz only: `Hz8000` is for a radio whose audio
/// is 8 kHz (the ICAP), so that it need not resample to 12 kHz and pay 1.5x the samples. The analysis grid is the
/// same (3.125 Hz a bin, 0.04 s a step, the tone step 2 bins: `NFFT` and `NSPS` scale with the rate), only the
/// first stage of the 200 Hz cascade differs. `Hz12000` is every other path's rate and is bit-for-bit what these
/// decoders did before the rate was an argument.
///
/// Not in the `fixed-point` build at `Hz8000` (it has no 2560-point 16-bit plan, and `FP_SPEC_SHIFT` is calibrated
/// for 3840): the spectrogram panics there. With `fft-extern`, the binary's planner must give a **2560-point** real
/// FFT at `Hz8000` (a planner that has 3840 only, as the ESP32 one does, panics at the first 8 kHz decode).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Ft8Rate {
    /// 12 000 Hz: `NSPS` 1920, FFT 3840.
    Hz12000,
    /// 8 000 Hz: `NSPS` 1280, FFT 2560.
    Hz8000,
}

impl Ft8Rate {
    /// The rate for `hz`, if there is one: `12_000` and `8_000`. Anything else is resampled to one of them first
    /// (`engine::dsp::resample`).
    pub const fn from_hz(hz: u32) -> Option<Self> {
        match hz {
            12_000 => Some(Ft8Rate::Hz12000),
            8_000 => Some(Ft8Rate::Hz8000),
            _ => None,
        }
    }

    /// The rate, Hz.
    pub const fn hz(self) -> u32 {
        match self {
            Ft8Rate::Hz12000 => 12_000,
            Ft8Rate::Hz8000 => 8_000,
        }
    }

    /// Samples per symbol (0.16 s).
    pub(super) const fn nsps(self) -> usize {
        self.hz() as usize * 4 / 25
    }

    /// The spectrogram FFT: two symbols (WSJT-X `sync8.f90`'s `NFFT1`), so a tone is 2 bins at any rate.
    pub(super) const fn nfft(self) -> usize {
        2 * self.nsps()
    }

    /// Whether the **fixed-point** coarse spectrogram weights its frames with a Hann window over two symbols (`nfft` samples
    /// centred on the frame's instant) instead of leaving them rectangular over one symbol.
    ///
    /// **A deliberate divergence from WSJT-X** (`sync8.f90:35-37`: `x(1:NSPS)=fac*dd(ia:ib); x(NSPS+1:)=0`, one symbol,
    /// rectangular, zero-padded to `NFFT1`), at `Hz8000` only: the rate it was measured at. 60 real slots (the 38 of
    /// `ft8_lib/test/wav/20m_busy` and 22 recordings, 1 338 `jt9` decodes; `ft8dec`'s 1 026 messages as the yardstick), the S3
    /// flow, 8 kHz, fixed-point, host: at **60 candidates** 879 -> 905 of `ft8dec`'s (hits 896 -> 922, extras 19 -> 15), at
    /// **30** 877 -> 905 (hits 893 -> 921), **at 15 candidates -10 hits (710 -> 700) and at 10 -13**: the window changes the order
    /// of the first candidates by hard sync before it improves the rest, so it pays from about 20 candidates and not below.
    /// Instructions per slot (12 slots): +0.4 M at 60 candidates (607.4 -> 607.8 M), +25 M at 30, +17 M at 15. Why it helps:
    /// the hard-sync count at `ft8dec`'s own place for the signals the shipped frame loses is 10 of 21 rectangular, 17 Hann,
    /// because the rectangular frame's sidelobes bury the Costas array of a signal that is not on the bin grid (a strong tone
    /// leaves 260 of 30 000 in the cell 25 Hz away, a Hann frame 0). `docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md`. It also moves
    /// the public `decode_block_low_mem` / `coarse_sync_gated` at `Hz8000` in this build (22 recordings, 30 candidates: 280 ->
    /// 284 of 402, one more decode `jt9` lacks, a copy of a signal 100 Hz away, attested in `profile::ATTESTED_EXTRAS`). `Hz12000` and the f32 build are
    /// not measured and keep the rectangular frame.
    #[cfg(feature = "fixed-point")]
    pub(super) const fn coarse_hann_2t(self) -> bool {
        matches!(self, Ft8Rate::Hz8000)
    }

    /// Whether the audio-source gated search (`coarse_sync_gated`) scores the slot with **`ft8_lib`'s statistic** on 0.5 dB cells of the
    /// demux's 32-bit power (`coarse_lib`) instead of `sync8`'s: `Hz8000` in the `fixed-point` build, the rate and build of the two
    /// windows ([`coarse_hann_2t`](Self::coarse_hann_2t), [`symbol_hann_2t`](Self::symbol_hann_2t)), whose frame it reads.
    ///
    /// **A deliberate divergence from WSJT-X** (`sync8.f90`). On 60 real slots (38 `20m_busy` + the 22 recordings, 1 338 `jt9` decodes;
    /// `ft8dec`'s 1 026 messages as the yardstick) it takes the S3 flow with both windows from 982 to 1 001 of those at 60 candidates (hits 997 -> 1 011; 15 candidates 737 -> 761); see
    /// `docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md` for the measurement and the cost. `Hz12000`, the f32 build and the streamed
    /// spectrogram's gate (`coarse_sync_gated_spec`) keep `sync8`.
    #[cfg(feature = "fixed-point")]
    pub(super) const fn coarse_ft8lib(self) -> bool {
        matches!(self, Ft8Rate::Hz8000)
    }

    /// Whether the **stage-3 symbol DFT** (the Goertzel of `fill_symbol_spectra_goertzel_at`) weights its window with a Hann window
    /// over two symbols (`2 x nsps` samples centred on the symbol) instead of leaving it rectangular over the symbol: `Hz8000` in
    /// the `fixed-point` build only (the numeric path the boards ship, and the one it was measured in), the same rate and build as
    /// [`coarse_hann_2t`](Self::coarse_hann_2t), whose frame it matches.
    ///
    /// **A deliberate divergence from WSJT-X** (`ft8b.f90:153-161`: `cs(0:7,k)=csymb(1:8)/1e3`, the symbol's 32 samples of the 200 Hz
    /// baseband through a rectangular 32-point FFT). Doubles the stage-3 DFT work (`2 x nsps` samples a symbol, 8 tones), which is
    /// about 63 % of a candidate's cost at the DFT alone; the whole S3 flow measured +25..+29 % instructions per slot (198 -> 255 M at
    /// 15 candidates). The Goertzel output is rotated back by `e^{-j w nsps/2}` to the rectangular kernel's phase reference: without
    /// it every odd tone's cell had the opposite sign and the coherent metrics (`llrb`, `llrc`, `llre`, which **OSD computes whatever
    /// the LLR effort**, so the measured S3 flow reads them) added cells of different tones with the wrong sign.
    ///
    /// What is **not** validated: the coherent metrics with **overlapping windows** (adjacent symbols' DFTs share half a window, so
    /// they are not independent) beyond the recall of the corpora in `FT8_WINDOW_AND_COARSE_DESIGN.md`; and the SNR read from
    /// these cells: a tone of one symbol in the window has 0.82 of its rectangular amplitude (power 0.67) and noise 0.75 of its
    /// power, which would put it about 0.5 dB low (**computed, not measured**; the neighbours' tones leak in and move it).
    pub(super) const fn symbol_hann_2t(self) -> bool {
        cfg!(feature = "fixed-point") && matches!(self, Ft8Rate::Hz8000)
    }

    /// The spectrogram's step between frames, samples (`NSPS / 4`, or `NSPS / 2` under `nstep-half`).
    pub(super) const fn nstep(self) -> usize {
        #[cfg(not(feature = "nstep-half"))]
        {
            self.nsps() / 4
        }
        #[cfg(feature = "nstep-half")]
        {
            self.nsps() / 2
        }
    }

    /// The input gain of the fixed-point spectrogram as a ratio `num / den`: `12 000 / rate`. A frame of `nsps` samples gives a
    /// tone `amp x nsps`, so scaling the samples by this makes a cell the same at either rate for the same signal.
    ///
    /// **Known limit**: the samples are `i16`, so a slot whose raw peak is above `32767 / 1.5 = 21845` (the auto-gain only ever raises
    /// a peak to 7680, never lowers it) clips its few strongest samples at 8 kHz: 2 to 254 samples of a slot in the 9 loud recordings
    /// of the corpus (raw peak 21 886 to 32 351), none in the 13 quiet ones. Scaling the cells by 9/4 after the transform instead
    /// avoids it and was **worse** (the 22 recordings, mean difference to 12 kHz over 15 candidate counts -1.7 against -0.4:
    /// an input of 2/3 the size is rounded 1.5x coarser), and the loud recordings lose no more than the quiet ones
    /// (-0.22 and -0.23 decodes a file at 15 candidates).
    #[cfg(feature = "fixed-point")]
    pub(super) const fn input_gain(self) -> (i32, i32) {
        match self {
            Ft8Rate::Hz12000 => (1, 1),
            Ft8Rate::Hz8000 => (3, 2),
        }
    }

    /// Input samples per 200 Hz sample (`rate / 200`): 60 at 12 kHz, 40 at 8 kHz.
    pub(super) const fn bin(self) -> usize {
        self.hz() as usize / 200
    }

    /// The slot length the spectrogram covers, samples (15 s).
    pub(super) const fn nmax(self) -> usize {
        15 * self.hz() as usize
    }
}

// The 12 kHz values are the crate's constants: the rate being an argument moves nothing at 12 kHz.
const _: () = {
    assert!(Ft8Rate::Hz12000.nsps() == NSPS);
    assert!(Ft8Rate::Hz12000.nfft() == NFFT_SPEC);
    assert!(Ft8Rate::Hz12000.nstep() == NSTEP);
    assert!(Ft8Rate::Hz12000.nmax() == super::super::params::NMAX);
    // the same analysis grid at every rate: bins of 3.125 Hz and steps of 0.04 s (0.08 s), a tone 2 bins
    assert!(Ft8Rate::Hz8000.hz() as usize * 1000 / Ft8Rate::Hz8000.nfft() == 3125);
    assert!(Ft8Rate::Hz8000.nstep() * 12_000 == Ft8Rate::Hz12000.nstep() * 8_000);
};

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
    /// Whether the AP passes may run. `ft8_decode.f90` sets `npasses=5`
    /// (blind passes only) while `nzhsym < 50`, i.e. in the early
    /// checkpoint; the final one and everything without a checkpoint
    /// structure run AP.
    pub ap: bool,
    /// Skip the staircase's variant `d` (`llrd` at one symbol, run after `llra` fails): the cells flow sets it for a candidate whose search score is under
    /// `CellsWorker`'s floor ([`CELLS_D_MIN_SCORE`](super::CELLS_D_MIN_SCORE)). Off in every WSJT-X-shaped path.
    pub skip_d: bool,
    /// The OSD fallback's runs ([`OsdSeeds`]); `Faithful` everywhere but a cells worker that asks otherwise.
    pub osd_seeds: OsdSeeds,
    /// When the OSD fallback runs ([`OsdMode`]).
    pub osd_mode: OsdMode,
    /// The candidate's hard-sync count from the search (of 21), for `OsdRank`; 0 where unknown.
    pub hs: u8,
    /// The cells flow's gate before BP (`BpGate`) and the candidate's hard-sync count it reads; `None` everywhere else.
    #[cfg(feature = "fixed-point")]
    pub gate: Option<GateCtx>,
}

/// Which of the OSD fallback's ten runs a candidate gets: `ft8b.f90` seeds `osd174_91` from five LLR variants (`a` to `e`) at two BP snapshots each, in that order, and
/// returns on the first CRC pass. Measured on the cells flow (4 780 slots, 143 400 candidates, 600 returns): **only variants `a` and `d` ever return** (`b`, `c`, `e`: none, in
/// 6 runs of the ten), so [`AD`](Self::AD) tries those two (four runs) and loses no return on that corpus. A divergence from WSJT-X, off by default.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum OsdSeeds {
    /// `ft8b.f90`'s: variants `a`, `b`, `c`, `d`, `e`, each at snapshots 1 and 2 (ten runs).
    #[default]
    Faithful,
    /// Variants `a` and `d` only, each at snapshots 1 and 2 (four runs; the LLRs are the one-symbol ones the staircase made already).
    AD,
}

/// When the OSD fallback of an attempt runs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum OsdMode {
    /// At once, when the BP staircase has failed (`ft8b.f90`'s order).
    #[default]
    Inline,
    /// Not run: the attempt is queued with its `OsdRank` score (`CellsWorker::osd_pending`), to be run by
    /// `CellsWorker::osd_pass` after the slot's BP work, the best `k` of the slot only.
    Defer,
    /// An attempt queued by `Defer`, run again at its place for the OSD alone (the BP staircase is given no iterations).
    Only,
}

/// A gate and the hard-sync count of the candidate it is asked about. Equal when they are the same gate (by address) and the same count.
#[cfg(feature = "fixed-point")]
#[derive(Clone, Copy, Debug)]
pub(in crate::ft8) struct GateCtx {
    pub gate: &'static super::pre_bp_gate::BpGate,
    pub hs: u8,
}

#[cfg(feature = "fixed-point")]
impl PartialEq for GateCtx {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.gate, other.gate) && self.hs == other.hs
    }
}
#[cfg(feature = "fixed-point")]
impl Eq for GateCtx {}

impl PassCtx {
    /// Pass 1: `imetric` 1.
    pub(in crate::ft8) const FIRST: PassCtx = PassCtx {
        imetric: 1,
        low_depth: false,
        contest: false,
        ap: true,
        skip_d: false,
        osd_seeds: OsdSeeds::Faithful,
        osd_mode: OsdMode::Inline,
        hs: 0,
        #[cfg(feature = "fixed-point")]
        gate: None,
    };

    /// This context with the gate before BP (the cells flow).
    #[cfg(feature = "fixed-point")]
    pub(in crate::ft8) const fn with_gate(self, gate: Option<GateCtx>) -> PassCtx {
        PassCtx { gate, ..self }
    }

    /// This context with the OSD fallback's timing and the candidate's search hard-sync count set.
    #[cfg(feature = "fixed-point")]
    pub(in crate::ft8) const fn with_osd_mode(self, osd_mode: OsdMode, hs: u8) -> PassCtx {
        PassCtx {
            osd_mode,
            hs,
            ..self
        }
    }

    /// This context with the OSD fallback's runs chosen.
    #[cfg(feature = "fixed-point")]
    pub(in crate::ft8) const fn with_osd_seeds(self, osd_seeds: OsdSeeds) -> PassCtx {
        PassCtx { osd_seeds, ..self }
    }

    /// This context without the staircase's variant `d` when `skip` is set.
    #[cfg(feature = "fixed-point")]
    pub(in crate::ft8) const fn skip_d(self, skip: bool) -> PassCtx {
        PassCtx {
            skip_d: skip,
            ..self
        }
    }

    /// This context without the AP passes (the early checkpoint).
    pub(in crate::ft8) const fn without_ap(self) -> PassCtx {
        PassCtx { ap: false, ..self }
    }

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
/// in 3.0, adopted here in #439 — see `coarse_sync::MLAG`;
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
