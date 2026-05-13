//! Embedded-friendly FT8 decode (esp-dsp pow-of-2 FFT only).
//!
//! Mirrors the host `decode_frame` pipeline but skips the 192_000-pt
//! wide-band FFT cache and the 3_840-pt per-symbol FFT — both of
//! which are non-power-of-two. Uses an 8192-pt per-symbol FFT for
//! the spectrogram (1920-sample input zero-padded) and a brute-force
//! per-tone DFT for the per-candidate LLR pass. Calls `bp_decode_kind`
//! with `BpKind::NormalizedMinSum` so the BP step skips the
//! `tanh`/`atanh` cache.
//!
//! Because the FFT bin width (≈ 1.465 Hz at 8192-pt) does not divide
//! the 6.25 Hz tone spacing evenly, Costas tone positions are computed
//! at fractional bins and rounded to the nearest integer. The
//! resulting bin-alignment jitter (≤ 0.7 Hz) is below FT8's
//! frequency-search tolerance.
//!
//! Same FFT trait, same compute path on host (rustfft) and on target
//! (esp-dsp) — sensitivity sweeps run on host and the result transfers
//! directly to hardware.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::decode::{ApHint, DecodeDepth, DecodeResult, DecodeStrictness};
use super::llr::sync_quality;
use super::message::unpack77;
use super::params::{COSTAS, COSTAS_POS, DEFAULT_BP_MAX_ITER, LDPC_N, NMAX, NN, NSPS, NTONES};
use super::wave_gen::message_to_tones;
#[cfg(not(feature = "fixed-point"))]
use crate::core::fft::default_planner;
use crate::core::scalar::Cmplx;
use crate::core::sync::SyncCandidate;
use crate::fec::ldpc::bp::check_crc14;
use crate::fec::ldpc::osd::{osd_decode, osd_decode_deep};

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
const NSTEP: usize = NSPS / 4;
#[cfg(feature = "nstep-half")]
const NSTEP: usize = NSPS / 2;

/// Steps per symbol — used to map symbol-index to time-step lag.
const NSSY: i32 = (NSPS / NSTEP) as i32;

/// FT8 tone spacing (Hz).
const TONE_SPACING_HZ: f32 = 6.25;

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
fn ratio_eps() -> f32 {
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
const SAMPLE_RATE_HZ: f32 = 12_000.0;

/// Slot start offset (FT8 transmits 0.5 s into the slot).
const TX_START_OFFSET_S: f32 = 0.5;

/// Coarse-sync ±lag search window (s).
///
/// WSJT-X uses ±2.5 s — covers operators with sloppy slot timing
/// or slow rigs. Embedded targets running on a synced clock (NTP /
/// GPS) live well within ±1 s; halving the lag range cuts
/// `coarse_sync` work by ~60 % (linear in `n_lag`). If the live
/// timing source is loose, raise this back to 2.5.
const SYNC_LAG_S_DEFAULT: f32 = 1.0;
fn sync_lag_s() -> f32 {
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
const NMS_ALPHA: f32 = 0.75;

/// `process_candidates` early-rejects cands whose full-21-symbol
/// `sync_quality` is at or below this threshold. Matches WSJT-X
/// `ft8b.f90:177` — `nsync ≤ 6 → bail`. Slower MCUs may raise this
/// at the cost of a few weak-signal decodes (the previous default
/// of 12 saved ~12-21 % stage-3 wall-clock); pass via the
/// `q_thresh` parameter on `process_candidates_into` /
/// `process_candidates_into_with_cs_scratch`.
pub const DEFAULT_Q_THRESH: u32 = 6;

// ── Spectrogram ─────────────────────────────────────────────────────────────

/// Spectrogram cell type. f32 (4 bytes) by default; u16 (2 bytes)
/// under `fixed-point` — magnitude squared right-shifted by
/// `FP_SPEC_SHIFT` to fit u16. `Spectrogram::power` returns f32 in
/// either case so downstream code (coarse_sync score division,
/// allsum) stays uniform. **Halves PSRAM bandwidth in stage 2.**
#[cfg(not(feature = "fixed-point"))]
pub type SpecCell = f32;
#[cfg(feature = "fixed-point")]
pub type SpecCell = u16;

/// Right-shift applied to `(re² + im²)` before storing as u16.
///
/// 12, not 16: with the host stub matching esp-dsp's `1/N` total
/// scaling, AWGN noise bins at typical recording levels (σ ≈ 5800
/// at peak 29000 input) yield mag² ≈ 8200 — `>>16` quantises that
/// to zero and breaks coarse_sync ratios. `>>12` keeps it at ~2,
/// preserving the noise floor.
///
/// Headroom check: max single-tone bin (peak input 29000, /N) is
/// 14500 → mag² = 2.1×10⁸ → `>>12` = 51 200, fits u16. Two coincident
/// FT8 tones at the same bin peaks at ≈ 8.4×10⁸ → `>>12` = 205 000,
/// **overflows u16** — extremely rare in practice (independent stations
/// virtually never align both freq and dt-grid bin), but watch for
/// truncation if the busy-band recall regresses.
#[cfg(feature = "fixed-point")]
const FP_SPEC_SHIFT: u32 = 12;

/// Power spectrogram emitted by [`compute_spectrogram`] and consumed
/// by [`coarse_sync`] / [`coarse_sync_with_allsum`]. The struct is
/// public so embedded consumers (`embedded-shared::dual_core` and
/// the M5Stack apps) can hold a [`Spectrogram`] across the
/// stage-1/stage-2 boundary; layout fields are publicly readable.
///
/// Public as of v0.6 (#49 cat C, was previously `#[doc(hidden)]`).
pub struct Spectrogram {
    /// Number of positive-frequency bins kept. Always ≤ NFFT_SPEC/2.
    /// We crop above the band of interest so a 8192-pt spectrogram on
    /// PSRAM-light targets (ESP32 Core2: 4 MB mapped) doesn't blow
    /// the heap (full 4096 × ~370 × 4 B = ~6 MB).
    pub n_freq: usize,
    /// Number of time slices.
    pub n_time: usize,
    /// **Column-major** (time major): `data[time * n_freq + freq]`.
    /// Picked so the inner Costas-correlation loop, which fixes
    /// time `m` and walks several frequency bins around a carrier
    /// candidate, does sequential PSRAM reads. Row-major would
    /// stride by `n_time × 4 ≈ 4 KB` per read — disaster on the
    /// ESP32's small PSRAM cache. Column-major keeps the working
    /// set of one time slice (`n_freq × 4 ≈ 4 KB`) in cache for
    /// the duration of all `(fi, lag)` cells touching it.
    pub data: Vec<SpecCell>,
}

impl Spectrogram {
    /// Build a `Spectrogram` from caller-provided parts. Used by the
    /// embedded Phase-E PoC to plumb an incrementally-computed
    /// spectrogram into the decode pipeline (bin builds the buffer
    /// during slot capture, then constructs a `Spectrogram` to feed
    /// `coarse_sync` directly, skipping `compute_spectrogram`).
    /// Layout must match `compute_spectrogram`: column-major,
    /// `data[time * n_freq + freq]`, length `n_time * n_freq`.
    pub fn from_parts(n_freq: usize, n_time: usize, data: Vec<SpecCell>) -> Self {
        assert_eq!(
            data.len(),
            n_freq * n_time,
            "Spectrogram::from_parts: data length must be n_freq * n_time"
        );
        Self {
            n_freq,
            n_time,
            data,
        }
    }
}

/// coarse_sync inner-loop accumulator. f32 — overlaps on the FPU with
/// integer index arithmetic on the ALU on ESP32 LX6/LX7; the integer
/// `i32` variant (formerly `fixed-point-coarse-i32`) serialises both
/// onto the ALU and costs ~25 % stage-2 wall-clock on Core2, so it
/// was retired. If an FPU-less target (RP2040, Cortex-M0+, Hazard3)
/// is added later, reintroduce the i32 alias on a per-target cfg.
type CoarseAcc = f32;

impl Spectrogram {
    /// Power-cell read in `CoarseAcc` (i32 under fixed-point, f32 otherwise).
    /// Used by `coarse_sync` to keep the precompute + score loop integer-pure
    /// on the embedded path — no per-cell u16→f32 promotion in the hot loop.
    /// Under host f32 the cast collapses to a no-op.
    #[inline]
    fn power_acc(&self, freq_bin: usize, time_idx: usize) -> CoarseAcc {
        debug_assert!(freq_bin < self.n_freq);
        debug_assert!(time_idx < self.n_time);
        #[allow(clippy::unnecessary_cast)]
        let v = self.data[time_idx * self.n_freq + freq_bin] as CoarseAcc;
        v
    }
}

/// Build the per-symbol power spectrogram via NFFT_SPEC-pt FFTs.
/// Each time slice is `NSPS = 1920` samples of Hann-windowed audio
/// zero-padded to `NFFT_SPEC`.
///
/// `max_freq_hz` is the upper edge of the carrier search; we keep
/// bins covering up to `max_freq_hz + 7 × tone_spacing + ε` so the
/// top Costas tone of a candidate at `max_freq_hz` is still in
/// range. Bins above that are discarded — saves ~half the heap on
/// ESP32 (4 MB PSRAM ceiling).
///
/// Public as of v0.6 (#48): `compute_spectrogram` is the canonical
/// FT8 spectrogram builder for both the embedded path and the host
/// `decode_frame*` pipeline (the latter routes coarse-sync through
/// this module after #46 / #48 step B).
#[cfg(not(feature = "fixed-point"))]
pub fn compute_spectrogram<S: AudioSample>(audio: &[S], max_freq_hz: f32) -> Spectrogram {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;
    let scale = 1.0f32 / 300.0;

    let mut planner = default_planner();
    let fft = planner.plan_forward(NFFT_SPEC);

    // Rectangular window — matches WSJT-X `sync8.f90`. With NFFT=3840,
    // `tone_step_bins = 6.25 / (12000/3840) = 2.0` exactly, so the
    // rectangular sidelobes do not bleed onto adjacent FT8 tones.
    // (The fixed-point path keeps Hann to mitigate the wider sidelobes
    // that come with NFFT=4096's fractional 2.13 bins/tone.)
    let mut data = vec![0.0f32; n_freq * n_time];
    let mut buf = vec![Complex::new(0.0f32, 0.0); NFFT_SPEC];

    for j in 0..n_time {
        let ia = j * NSTEP;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < NSPS {
                let sample = if ia + k < audio.len() {
                    audio[ia + k].to_f32() * scale
                } else {
                    0.0
                };
                Complex::new(sample, 0.0)
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        fft.process(&mut buf);
        // Column-major write — `data[j * n_freq + i]` keeps each
        // time slice contiguous in memory (good PSRAM locality
        // for downstream coarse_sync).
        let row_base = j * n_freq;
        for i in 0..n_freq {
            data[row_base + i] = buf[i].norm_sqr();
        }
    }

    Spectrogram {
        n_freq,
        n_time,
        data,
    }
}

/// Fixed-point variant: `Vec<u32>` magnitude squared from an i16
/// complex FFT. Halves spectrogram heap on PSRAM-light targets and
/// is the only viable backend on FPU-less MCUs.
///
/// **Auto-gain**: esp-dsp's `dsps_fft2r_sc16` divides by 2 at each
/// of `log2(N)` butterfly stages (12 stages at NFFT=4096) to keep
/// the i16 working set from overflowing. That's a total `/4096`
/// scale-down of the output. A real-world FT8 recording with peaks
/// well below i16 max (e.g. WSJT-X reference WAVs at 5 % of full
/// scale) gets quantised to zero by stage 6 of the FFT and produces
/// an empty spectrogram. We compute the slot's peak once and shift
/// the i16 input left enough to reach ~ ¼ of i16 range, leaving
/// headroom for FFT growth in tone-rich slots.
#[cfg(feature = "fixed-point")]
pub fn compute_spectrogram<S: AudioSample>(audio: &[S], max_freq_hz: f32) -> Spectrogram {
    use crate::core::fft::default_planner_16;

    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;

    // Pre-scale gain: shift left so the audio peak lands at
    // `2 × NFFT` (so after `log2(NFFT)` stages of /2 the post-FFT
    // peak still has a usable mantissa of ~2).
    let target_peak: i32 = (NFFT_SPEC * 2) as i32;
    let mut peak_abs: i32 = 1;
    let n_scan = audio.len().min(NMAX);
    for k in 0..n_scan {
        let v = audio[k].to_i16() as i32;
        let a = v.unsigned_abs() as i32;
        if a > peak_abs {
            peak_abs = a;
        }
    }
    let mut shift: u32 = 0;
    while peak_abs << shift < target_peak && shift < 8 {
        shift += 1;
    }
    // +1 extra shift: the Hann window's coherent gain is 0.5
    // (single-bin signal amplitude halved → bin power ÷ 4). Pre-
    // shifting input by +1 bit doubles amplitude so the post-FFT bin
    // amplitude lands back where the rectangular-window auto-gain
    // plan expected. Peak input samples near the window centre
    // (where Hann ≈ 1) may saturate to i16_MAX after this shift,
    // but the centre is also where CG is highest — clamping a few
    // (Hann window and the +1 pre-shift that compensated its 0.5
    // coherent gain were both removed when the spectrogram switched
    // to NFFT=3840 — at integer tone alignment the rectangular-window
    // sidelobes don't leak onto adjacent FT8 tones, so the window
    // costs more than it saves.)

    let mut planner = default_planner_16();
    let fft = planner.plan_forward(NFFT_SPEC);

    let mut data: Vec<u16> = vec![0u16; n_freq * n_time];
    let mut buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0i16); NFFT_SPEC];

    // **Two-for-one real-FFT trick**. Each frame's audio is real; we
    // pack two consecutive frames as `re = windowed(frame_a)`,
    // `im = windowed(frame_b)` into a single complex FFT. From
    //     Y[k] = X_a[k] + j·X_b[k]
    // and the real-input conjugate symmetry `X_•[N-k] = conj(X_•[k])`,
    // we recover the per-frame spectra via post-butterfly:
    //     X_a[k] = (Y[k] + conj(Y[N-k])) / 2
    //     X_b[k] = -j · (Y[k] - conj(Y[N-k])) / 2
    // i.e.
    //     A_re = (Y[k].re + Y[N-k].re) / 2,
    //     A_im = (Y[k].im - Y[N-k].im) / 2
    //     B_re = (Y[k].im + Y[N-k].im) / 2,
    //     B_im = (Y[N-k].re - Y[k].re) / 2
    // Halves the FFT count (184 → 92 on the standard FT8 slot) — the
    // dominant stage-1 cost on Core2. Demux is O(n_freq) per pair,
    // negligible vs an N=4096 FFT. Magnitude scaling matches the
    // single-frame path (the >>1 in demux exactly cancels the √2
    // amplitude headroom that |Y[k]|² = |X_a[k]|² + |X_b[k]|² would
    // otherwise give); auto-gain `shift` is reused unchanged.
    let n_pairs = n_time / 2;
    let n_odd = n_time & 1;

    // Hann window (Q15). Even though NFFT=3840 makes tone_step_bins
    // an exact 2.0, leaving the bare i16 spectrum un-windowed lost
    // ~half the WSJT-X-golden recall in regression — the rect-window
    // -13 dB sidelobes apparently leak between the *adjacent slot's*
    // signals, not just the cross-tone neighbours we predicted, and
    // the auto-gain shift on i16 amplifies that leakage. Keep Hann
    // here (it's also what stage1_inc already does on embedded).
    let mut hann = [0i16; NSPS];
    for n in 0..NSPS {
        let phase = 2.0 * core::f32::consts::PI * (n as f32) / (NSPS as f32);
        let w = 0.5 - 0.5 * phase.cos();
        hann[n] = (w * 32767.0) as i16;
    }
    let pack = |buf: &mut [Complex<i16>], ia_a: usize, ia_b: Option<usize>| {
        for (k, c) in buf.iter_mut().enumerate() {
            let re = if k < NSPS && ia_a + k < audio.len() {
                let raw = audio[ia_a + k].to_i16() as i32;
                let scaled = (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32);
                ((scaled * hann[k] as i32) >> 15) as i16
            } else {
                0
            };
            let im = match ia_b {
                Some(ia_b) if k < NSPS && ia_b + k < audio.len() => {
                    let raw = audio[ia_b + k].to_i16() as i32;
                    let scaled = (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32);
                    ((scaled * hann[k] as i32) >> 15) as i16
                }
                _ => 0,
            };
            *c = Complex::new(re, im);
        }
    };

    for jj in 0..n_pairs {
        let j_a = 2 * jj;
        let j_b = j_a + 1;
        pack(&mut buf, j_a * NSTEP, Some(j_b * NSTEP));
        fft.process(&mut buf);

        let row_a = j_a * n_freq;
        let row_b = j_b * n_freq;
        // Demux. Modular wrap (NFFT_SPEC=3840 isn't a power of two so
        // `& (NFFT-1)` would alias the high bins). `kn = (NFFT-k) mod
        // NFFT` collapses k=0 to 0 — DC bin is real, as expected for
        // real input.
        for k in 0..n_freq {
            let kn = if k == 0 { 0 } else { NFFT_SPEC - k };
            let yk_re = buf[k].re as i32;
            let yk_im = buf[k].im as i32;
            let yn_re = buf[kn].re as i32;
            let yn_im = buf[kn].im as i32;
            let a_re = (yk_re + yn_re) >> 1;
            let a_im = (yk_im - yn_im) >> 1;
            let b_re = (yk_im + yn_im) >> 1;
            let b_im = (yn_re - yk_re) >> 1;
            let mag2_a = ((a_re * a_re + a_im * a_im) as u32) >> FP_SPEC_SHIFT;
            let mag2_b = ((b_re * b_re + b_im * b_im) as u32) >> FP_SPEC_SHIFT;
            data[row_a + k] = mag2_a as u16;
            data[row_b + k] = mag2_b as u16;
        }
    }

    // Odd-frame fallback: single-frame FFT for the trailing slice when
    // n_time is odd. n_time=184 on the standard FT8 slot so this path
    // is exercised only by truncated inputs / regression-test fixtures.
    if n_odd != 0 {
        let j = 2 * n_pairs;
        pack(&mut buf, j * NSTEP, None);
        fft.process(&mut buf);
        let row_base = j * n_freq;
        for i in 0..n_freq {
            let re = buf[i].re as i32;
            let im = buf[i].im as i32;
            let mag2 = ((re * re + im * im) as u32) >> FP_SPEC_SHIFT;
            data[row_base + i] = mag2 as u16;
        }
    }

    Spectrogram {
        n_freq,
        n_time,
        data,
    }
}

// ── Coarse sync ─────────────────────────────────────────────────────────────

/// Costas-array correlation search across the spectrogram. Mirrors
/// WSJT-X `lib/ft8/sync8.f90` — 16-bin sliding-window allsum noise
/// estimator with `tone_step_bins = 2.0` exactly at NFFT_SPEC=3840.
///
/// Public as of v0.6 (#48): this is now the canonical FT8 coarse-sync
/// for both the embedded port and the host `decode_frame*` pipeline.
/// `core::sync::coarse_sync<Ft8>` is no longer reachable from FT8 code
/// paths; that generic function stays in place for FT4 / FST4 / JT9 /
/// Q65 / WSPR / uvpacket.
pub fn coarse_sync(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(spec, freq_min, freq_max, sync_min, max_cand, None)
}

/// Phase-E2 entry point — like [`coarse_sync`] but consumes a
/// caller-built allsum table instead of recomputing it. Saves the
/// 280-300 ms allsum precompute on Core2 by hiding it under the
/// 15 s capture window in the embedded port (`stage1_inc` builds
/// the allsum incrementally as new spectrogram rows arrive).
///
/// `allsum` must match exactly what
/// [`precompute_coarse_allsum`] produces for the same `spec`,
/// `freq_min`, `freq_max` triple — same layout
/// (`data[fi * spec.n_time + m]`), same length
/// ([`coarse_allsum_len`]).
///
/// Public as of v0.6 (#49 cat C): the embedded port (`m5stack-core2`,
/// `m5stack-s3`, `m5stack-s3-app`) and `mfsk-ffi-ft8` both build the
/// allsum incrementally during slot capture and pass it back here, so
/// this is a stable public surface — not a benchmark-only escape hatch.
pub fn coarse_sync_with_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum: &[CoarseAcc],
) -> Vec<SyncCandidate> {
    coarse_sync_inner(spec, freq_min, freq_max, sync_min, max_cand, Some(allsum))
}

/// Length of the allsum buffer that [`coarse_sync_with_allsum`]
/// expects for the given `spec` + `freq_min..=freq_max` band.
/// Returns 0 when the band has no candidates (then
/// `coarse_sync_with_allsum` would return an empty vec immediately
/// regardless of `allsum`).
pub fn coarse_allsum_len(
    spec_n_freq: usize,
    spec_n_time: usize,
    freq_min: f32,
    freq_max: f32,
) -> usize {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tone_step_bins = TONE_SPACING_HZ / df;
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(spec_n_freq.saturating_sub(max_tone_off));
    if ib < ia {
        return 0;
    }
    let n_freq = ib - ia + 1;
    n_freq * spec_n_time
}

/// One-shot allsum builder. The embedded port can build the allsum
/// incrementally instead and pass it to [`coarse_sync_with_allsum`];
/// host callers use this. Layout matches [`coarse_sync_with_allsum`]'s
/// expected input.
pub fn precompute_coarse_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
) -> Vec<CoarseAcc> {
    let mut buf =
        vec![CoarseAcc::default(); coarse_allsum_len(spec.n_freq, spec.n_time, freq_min, freq_max)];
    if !buf.is_empty() {
        precompute_coarse_allsum_into(spec, freq_min, freq_max, &mut buf);
    }
    buf
}

/// In-place variant of [`precompute_coarse_allsum`]. `dst` must have
/// length [`coarse_allsum_len`]. No-op if the band is empty.
pub fn precompute_coarse_allsum_into(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    dst: &mut [CoarseAcc],
) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tone_step_bins = TONE_SPACING_HZ / df;
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let nh1 = spec.n_freq;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
    if ib < ia {
        return;
    }
    let n_freq = ib - ia + 1;
    debug_assert_eq!(
        dst.len(),
        n_freq * spec.n_time,
        "allsum buffer length mismatch"
    );
    fill_coarse_allsum(spec, ia, ib, n_freq, dst);
}

/// Build the 16-bin sliding-window allsum for the carrier-bin range
/// `[ia..=ib]` into the caller buffer `dst` (length
/// `n_freq * spec.n_time`, layout `dst[fi * n_time + m]`).
///
/// Mirrors the inline precompute inside [`coarse_sync_inner`] —
/// kept as a separate helper so the embedded port can replicate it
/// row-by-row in [`stage1_inc`] without depending on private
/// internals. Computes for **every m**, not just `needed_m`, so
/// callers can populate cells incrementally with single-row updates
/// in any order.
fn fill_coarse_allsum(
    spec: &Spectrogram,
    ia: usize,
    ib: usize,
    _n_freq: usize,
    dst: &mut [CoarseAcc],
) {
    let nh1 = spec.n_freq;
    // WSJT-X `sync8.f90:66` `t0a = sum(s(i:i+nfos*6:nfos, m))` — sums
    // **7** tones (k=0..6), NOT 8. The Costas array uses `icos7 =
    // [3,1,4,0,6,5,2]` ⊂ {0..6}; tone 7 is data-only and never a
    // Costas position. Including tone 7 in the reference sum dilutes
    // the discriminator with data/noise energy that has no parallel
    // in WSJT-X — it shifts our normalised sync score relative to
    // theirs and was a logical implementation diff. NFFT=3840 →
    // tone_step_bins=2.0 exactly, so single-bin gather per tone.
    for (fi, i_carrier) in (ia..=ib).enumerate() {
        let row_off = fi * spec.n_time;
        for m in 0..spec.n_time {
            let mut s: CoarseAcc = CoarseAcc::default();
            for k in 0..(NTONES - 1) {
                let bin = (i_carrier + 2 * k).min(nh1 - 1);
                s += spec.power_acc(bin, m);
            }
            dst[row_off + m] = s;
        }
    }
}

fn coarse_sync_inner(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    external_allsum: Option<&[CoarseAcc]>,
) -> Vec<SyncCandidate> {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let jstrt = (TX_START_OFFSET_S / tstep).round() as i32;
    let jz = (sync_lag_s() / tstep).round() as i32;
    let tone_step_bins = TONE_SPACING_HZ / df;

    // Carrier-bin search range. Reserve room above the carrier for the
    // top tone (round(7 * tone_step_bins)).
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let nh1 = spec.n_freq;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
    if ib < ia {
        return Vec::new();
    }
    let n_freq = ib - ia + 1;
    let n_lag = (2 * jz + 1) as usize;
    let mut sync2d = vec![0.0f32; n_freq * n_lag];
    let idx = |fi: usize, lag: i32| fi * n_lag + (lag + jz) as usize;
    let ratio_eps = ratio_eps();
    #[cfg(feature = "std")]
    let prof = cfg!(feature = "profile-coarse") || std::env::var("MFSK_PROFILE_COARSE").is_ok();
    #[cfg(feature = "std")]
    let t_setup = std::time::Instant::now();

    // **Single-bin tone gather**. NFFT=3840 → tone_step_bins = 2.0
    // exactly, so the 8 FT8 tones sit on integer bins {0, 2, 4, …, 14}
    // relative to the carrier. With the rectangular pre-FFT window
    // each tone's mainlobe is one bin wide and contains the full tone
    // energy, so single-bin reads are loss-less. The Plan-A multi-bin
    // sum (floor-bin + floor-bin+1) was a fractional-alignment +
    // Hann-mainlobe compensation for the NFFT=4096 era and was
    // dropped at the 3840 migration; do NOT re-add it without also
    // restoring Hann (the two were always paired).
    let mut tone_bin_lo = [0usize; NTONES];
    for k in 0..NTONES {
        tone_bin_lo[k] = (k as f32 * tone_step_bins).floor() as usize;
    }

    // Pre-compute the (bk, n) → m_base table. m for the inner iter is
    // `m_base[bk][n] + lag`; m_base depends only on the Costas pattern
    // and `jstrt` (constants of the slot), not on (fi, lag). Hoists
    // 21 mul/add chains out of the n_freq × n_lag × 21 inner loop.
    let m_base: [[i32; COSTAS.len()]; COSTAS_POS.len()] = {
        let mut t = [[0i32; COSTAS.len()]; COSTAS_POS.len()];
        for (bk, &start_sym) in COSTAS_POS.iter().enumerate() {
            let block_offset = NSSY * start_sym as i32;
            for (n, _) in COSTAS.iter().enumerate() {
                t[bk][n] = jstrt + block_offset + NSSY * n as i32;
            }
        }
        t
    };
    // Pre-compute Costas-tone bin offsets in COSTAS-order (i.e.
    // `tone_bin_lo[COSTAS[n]]`). Saves an indirect lookup per inner
    // iteration.
    let costas_off: [usize; COSTAS.len()] = {
        let mut t = [0usize; COSTAS.len()];
        for (n, &costas_n) in COSTAS.iter().enumerate() {
            t[n] = tone_bin_lo[costas_n];
        }
        t
    };

    // Pre-compute the set of `m` time-indices that the score loop
    // will read. Only Costas-symbol positions ± lag count — typically
    // 3 contiguous bands totalling ~110 of the 184 frames, so the
    // naïve "compute allsum for every m" wastes ~40 % of its work
    // on cells the score loop never reads.
    let needed_m: alloc::vec::Vec<usize> = {
        let mut mark = alloc::vec![false; spec.n_time];
        for bk in 0..COSTAS_POS.len() {
            let lo = m_base[bk][0] - jz;
            let hi = m_base[bk][COSTAS.len() - 1] + jz;
            let lo_u = lo.max(0) as usize;
            let hi_u = (hi.min(spec.n_time as i32 - 1)) as usize;
            if lo_u <= hi_u {
                #[allow(clippy::needless_range_loop)]
                for m in lo_u..=hi_u {
                    mark[m] = true;
                }
            }
        }
        (0..spec.n_time).filter(|&m| mark[m]).collect()
    };

    // Pre-compute `Σ_{k=0..NTONES-1} spec[i_carrier + 2*k, m]` for every
    // (fi, m ∈ needed_m). NFFT=3840 → tone_step_bins=2.0 exactly →
    // 8-bin every-other gather per carrier (matches WSJT-X `sync8.f90`'s
    // single-bin `t0a` accumulator). No sliding-window reuse since
    // adjacent fi share zero bins under this pattern.
    //
    // Phase-E2: caller can pass a pre-built allsum (built incrementally
    // during slot capture by stage1_inc). When provided, skip the
    // precompute entirely. Otherwise build it inline.
    let owned_allsum: Vec<CoarseAcc>;
    let allsum: &[CoarseAcc] = if let Some(ext) = external_allsum {
        debug_assert_eq!(
            ext.len(),
            n_freq * spec.n_time,
            "external allsum length mismatch (expected n_freq * spec.n_time)"
        );
        ext
    } else {
        owned_allsum = {
            // Sum 7 Costas-tone bins (k=0..NTONES-1; tone 7 is data-only,
            // never a Costas position). Matches WSJT-X `sync8.f90:66` and
            // the public `fill_coarse_allsum` helper, so the score
            // formula's `(t0 - t) / (NTONES - 2)` divisor is calibrated:
            // t0 sums 7 tones, t = 1 Costas-tone bin energy, t0-t = 6
            // non-Costas tones, divisor = 6 ✓. Pre-fix this loop ran
            // `0..NTONES` (= 8 tones), which made owned_allsum and
            // fill_coarse_allsum disagree numerically and broke the
            // `coarse_sync_with_allsum_matches_internal` /
            // `coarse_sync_split_with_allsum_*` consistency tests.
            let mut buf = vec![CoarseAcc::default(); n_freq * spec.n_time];
            for (fi, i_carrier) in (ia..=ib).enumerate() {
                let row_off = fi * spec.n_time;
                for &m in &needed_m {
                    let mut s: CoarseAcc = CoarseAcc::default();
                    for k in 0..(NTONES - 1) {
                        let bin = (i_carrier + 2 * k).min(nh1 - 1);
                        s += spec.power_acc(bin, m);
                    }
                    buf[row_off + m] = s;
                }
            }
            buf
        };
        &owned_allsum
    };
    #[cfg(feature = "std")]
    let t_allsum = std::time::Instant::now();

    // Three identical Costas arrays at symbol positions 0, 36, 72.
    //
    // **Bounds-check hoist**. For the standard FT8 slot:
    //   block 0: m = lag + jstrt + NSSY*n   ∈ [-7..31] over (lag, n)
    //   block 1: m = lag + jstrt + 144 + NSSY*n  ∈ [65..103]
    //   block 2: m = lag + jstrt + 288 + NSSY*n  ∈ [137..175]
    //
    // Only block 0 can dip below 0 (and only at large negative lag).
    // Compute the smallest valid `n` per lag once, then iterate
    // n_start..NTONES_C without per-iter checks. Blocks 1 and 2 are
    // always fully in range. Sanity-check the upper bound at function
    // entry so the unchecked-as-usize is safe.
    debug_assert!(
        m_base[2][COSTAS.len() - 1] + jz < spec.n_time as i32,
        "n_time too small for SYNC_LAG_S/jstrt"
    );
    let n_time = spec.n_time;
    // Per-fi: tbin_lo[n] = i_carrier + costas_off[n]. Hoist out of
    // the lag loop — saves an addition per inner iteration.
    let mut tbin_lo_arr = [0usize; COSTAS.len()];
    for (fi, i_carrier) in (ia..=ib).enumerate() {
        for n in 0..COSTAS.len() {
            tbin_lo_arr[n] = i_carrier + costas_off[n];
        }
        let allsum_row = &allsum[fi * n_time..(fi + 1) * n_time];
        for lag in -jz..=jz {
            let mut t_blocks: [CoarseAcc; 3] = [CoarseAcc::default(); 3];
            let mut t0_blocks: [CoarseAcc; 3] = [CoarseAcc::default(); 3];

            // Block 0: smallest valid n where m_base[0][n] + lag >= 0.
            // m_base[0][n] = jstrt + NSSY*n. Solve for n:
            //   jstrt + NSSY*n + lag >= 0 ⇒ n >= ceil((-jstrt - lag) / NSSY)
            let bk0_n_start = {
                let needed = -jstrt - lag;
                if needed <= 0 {
                    0usize
                } else {
                    ((needed + NSSY - 1) / NSSY).min(COSTAS.len() as i32) as usize
                }
            };
            // NFFT=3840 → tone_step_bins = 2.0 exactly, signal at bin
            // `tbin_lo` only (single bin gather, matching WSJT-X's
            // `s(i+nfos*icos7(n), m)`). The legacy `+ spec[tbin_lo + 1]`
            // half was a fractional-alignment compensation for NFFT=4096
            // and is removed.
            for n in bk0_n_start..COSTAS.len() {
                let m_u = (m_base[0][n] + lag) as usize;
                let tbin_lo = tbin_lo_arr[n];
                t_blocks[0] += spec.power_acc(tbin_lo, m_u);
                t0_blocks[0] += allsum_row[m_u];
            }
            // Blocks 1, 2: always fully in range — no per-iter check.
            for bk in 1..COSTAS_POS.len() {
                for n in 0..COSTAS.len() {
                    let m_u = (m_base[bk][n] + lag) as usize;
                    let tbin_lo = tbin_lo_arr[n];
                    t_blocks[bk] += spec.power_acc(tbin_lo, m_u);
                    t0_blocks[bk] += allsum_row[m_u];
                }
            }

            // Regularised ratio `t / (mean_others + ε)`.
            // ε prevents the u16-quantised fp path from blowing up
            // when phantom carriers happen to land where 7 of 8 tone
            // bins quantise to 0; `t0_ref → 0` would otherwise inflate
            // ratio scores by 100-1000× over real signals.
            //
            // Phase 3: t/t0 sums stay in CoarseAcc; convert to f32 only
            // at the score-division boundary so sync2d / red / base
            // (downstream sort + percentile) remain f32.
            let t_all: CoarseAcc = t_blocks[0] + t_blocks[1] + t_blocks[2];
            let t0_all: CoarseAcc = t0_blocks[0] + t0_blocks[1] + t0_blocks[2];
            // `as f32` is a real promotion under fixed-point (CoarseAcc=i32)
            // and a no-op when CoarseAcc=f32 — silence the host clippy.
            #[allow(clippy::unnecessary_cast)]
            let t_all_f = t_all as f32;
            #[allow(clippy::unnecessary_cast)]
            let t0_all_f = t0_all as f32;
            let t0_ref = (t0_all_f - t_all_f) / (NTONES as f32 - 2.0);
            let sync_all = t_all_f / (t0_ref + ratio_eps);

            // Trailing-2-blocks score (drop block 0 — late-start tolerance).
            #[allow(clippy::unnecessary_cast)]
            let t_tail_f = (t_blocks[1] + t_blocks[2]) as f32;
            #[allow(clippy::unnecessary_cast)]
            let t0_tail_f = (t0_blocks[1] + t0_blocks[2]) as f32;
            let t0_tail_ref = (t0_tail_f - t_tail_f) / (NTONES as f32 - 2.0);
            let sync_tail = t_tail_f / (t0_tail_ref + ratio_eps);

            sync2d[idx(fi, lag)] = sync_all.max(sync_tail);
        }
    }
    #[cfg(feature = "std")]
    let t_score = std::time::Instant::now();

    const MLAG: i32 = 10;

    // Per-bin peak + 40-percentile noise floor.
    let mut red = vec![0.0f32; n_freq];
    for fi in 0..n_freq {
        red[fi] = (-jz..=jz)
            .map(|lag| sync2d[idx(fi, lag)])
            .fold(0.0f32, f32::max);
    }
    let base = {
        let mut sorted = red.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct_idx = (0.40 * n_freq as f32) as usize;
        sorted[pct_idx.min(n_freq - 1)].max(f32::EPSILON)
    };

    let mut cands: Vec<SyncCandidate> = Vec::new();
    for fi in 0..n_freq {
        let i_carrier = ia + fi;
        let freq_hz = i_carrier as f32 * df;

        let mut peaks: Vec<(i32, f32)> = (-jz..=jz)
            .filter_map(|lag| {
                let raw = sync2d[idx(fi, lag)];
                let norm = raw / base;
                if norm.is_finite() && norm >= sync_min {
                    Some((lag, norm))
                } else {
                    None
                }
            })
            .collect();
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let mut picked: Vec<i32> = Vec::new();
        for (lag, score) in peaks {
            if picked.iter().any(|&pl| (lag - pl).abs() <= MLAG) {
                continue;
            }
            picked.push(lag);
            let dt_quanta = if lag > -jz && lag < jz {
                let y_lo = sync2d[idx(fi, lag - 1)];
                let y_mi = sync2d[idx(fi, lag)];
                let y_hi = sync2d[idx(fi, lag + 1)];
                let denom = y_lo - 2.0 * y_mi + y_hi;
                if denom.abs() > f32::EPSILON {
                    let off = 0.5 * (y_lo - y_hi) / denom;
                    off.clamp(-1.0, 1.0)
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let dt_lag = lag as f32 + dt_quanta;
            cands.push(SyncCandidate {
                freq_hz,
                dt_sec: (dt_lag - 0.5) * tstep,
                score,
            });
            // WSJT-X sync8.f90 emits at most 2 candidates per freq:
            // one from `red` (narrow ±MLAG window) and one from `red2`
            // (full ±jz window) when the peak lags differ. We don't
            // run two windows separately; the `picked` greedy NMS with
            // cap 2 produces equivalent recall on `qso3_busy.wav` —
            // empirically verified during the v0.6.2 plan (a
            // dual-window prototype produced identical 16/18 JTDX
            // hit, so the additional separate normalization wasn't
            // worth the code volume). Dropped from 0.6.2 scope.
            if picked.len() >= 2 {
                break;
            }
        }
    }

    // Dedupe within 4 Hz / 40 ms; keep highest score.
    //
    // Sort once by score desc, then greedily keep cands with no
    // already-kept near neighbour. Stops early at `max_cand` for
    // O(n log n + n × max_cand) instead of the prior O(n²) pairwise
    // compare-and-zero (n is several thousand).
    //
    // Empirically this drops 1 borderline busy-band truth on qso3
    // vs the byte-equivalent O(n²) implementation — likely a
    // tie-break ordering difference at the 4 Hz dedupe boundary
    // (haven't fully traced; the algorithms are equivalent on every
    // small-case scenario I worked through). We accept the recall
    // delta for the ~150 us host (~150 ms Core2) saving.
    cands.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
    let mut out: Vec<SyncCandidate> = Vec::with_capacity(max_cand);
    for c in cands {
        if c.score < sync_min {
            break;
        }
        let near = out
            .iter()
            .any(|k| (c.freq_hz - k.freq_hz).abs() < 4.0 && (c.dt_sec - k.dt_sec).abs() < 0.04);
        if near {
            continue;
        }
        out.push(c);
        if out.len() >= max_cand {
            break;
        }
    }
    let cands = out;
    #[cfg(feature = "std")]
    if prof {
        let t_end = std::time::Instant::now();
        let allsum_us = (t_allsum - t_setup).as_micros();
        let score_us = (t_score - t_allsum).as_micros();
        let post_us = (t_end - t_score).as_micros();
        let total_us = (t_end - t_setup).as_micros();
        eprintln!(
            "[coarse_sync prof] n_freq={n_freq} n_lag={n_lag}  allsum={allsum_us} score={score_us} dedupe+sort={post_us}  total={total_us} us"
        );
    }
    cands
}

// ── Per-symbol direct DFT (no FFT cache) ────────────────────────────────────

/// Compute the 79 × 8 complex tone spectra for one candidate by
/// direct DFT at the exact tone frequencies. Bypasses the wide-band
/// FFT cache entirely.
///
/// **Phase-rotator recursion.** Naïve per-sample `cos/sin` would be
/// ~25 M libm calls per `decode_block` invocation (8 candidates × 5
/// dt offsets × 79 symbols × 8 tones × 1920 samples) — minutes on
/// LX6. We replace it with one cos/sin pair per (symbol, tone) and
/// a single complex multiply per sample.
///
/// **PSRAM-aware access pattern.** The audio buffer (360 KB) lives
/// in PSRAM on Core2 (40 MHz quad, ~5× slower than internal RAM).
/// A naïve "for tone × for sample" loop would re-read each audio
/// sample 8 times across PSRAM. Instead we copy each 1920-sample
/// symbol into a stack-local f32 buffer once, then run all 8 tone
/// integrations over that internal-RAM copy. Reduces audio reads
/// from PSRAM by 8× — the dominant cost on LX6.
///
/// Numerical error: each rotation is a unit-magnitude multiply with
/// f32 round-off ≈ 6e-8; over 1920 samples the cumulative magnitude
/// drift stays below 0.012 % — negligible for LLR computation.
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn symbol_spectra_direct<S: AudioSample>(
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    sym_mask: SymMask,
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra(&mut out, audio, freq_hz, dt_sec, sym_mask, None);
    out
}

/// Which subset of the 79 symbols to compute. Used for the
/// Costas-first early-reject in `process_candidates`: the first
/// pass fills only Costas tone positions (21 symbols, 27 % of
/// full DFT cost) for the `sync_quality` gate; only on a hit do
/// we go back and fill the data-symbol positions.
///
/// **Pub for benchmarking only.**
#[doc(hidden)]
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum SymMask {
    /// Costas symbols only — all three blocks (positions 0-6, 36-42,
    /// 72-78). 21 symbols. Used for full-precision sync_quality
    /// gating in stage 3.
    SyncOnly,
    /// Costas block 0 only (positions 0-6). 7 symbols — 1/3 the cost
    /// of `SyncOnly`. Used for Pass 2 sync_quality re-rank where the
    /// finer ranking precision of all three blocks is unnecessary.
    SyncBlock0,
    /// Everything except Costas block 0 — fills positions 7-78
    /// (data symbols + Costas blocks 1, 2). 72 symbols. Used in
    /// stage 3 to "top up" a `SyncBlock0`-filled spectrum.
    NotBlock0,
    /// Data symbols only (positions 7-35, 43-71). Skips the 21 sync
    /// positions — used to "top up" a `SyncOnly`-filled spectrum.
    DataOnly,
    /// Costas blocks 1 and 2 only (positions 36-42, 72-78). 14
    /// symbols — 2/3 the cost of `SyncOnly`. Used in stage 3 to top
    /// up a `SyncBlock0`-filled cs (Pass 2 output) into a full
    /// `SyncOnly`-equivalent without redoing block 0.
    SyncBlocks12,
}

#[inline]
fn sym_in_mask(sym: usize, mask: SymMask) -> bool {
    let (in_block_a, in_block_b, in_block_c) = (
        sym < COSTAS.len(),                                         // 0..7
        sym >= COSTAS_POS[1] && sym < COSTAS_POS[1] + COSTAS.len(), // 36..43
        sym >= COSTAS_POS[2] && sym < COSTAS_POS[2] + COSTAS.len(), // 72..79
    );
    let is_sync = in_block_a || in_block_b || in_block_c;
    match mask {
        SymMask::SyncOnly => is_sync,
        SymMask::SyncBlock0 => in_block_a,
        SymMask::NotBlock0 => !in_block_a,
        SymMask::DataOnly => !is_sync,
        SymMask::SyncBlocks12 => in_block_b || in_block_c,
    }
}

/// **Pub for benchmarking only — do not depend on it.**
///
/// f32 wrapper. **WSJT-X-faithful** when `fft-rustfft` is enabled:
/// routes through the `ft8_downsample` chain (192k FFT → tapered LPF
/// → 200 sps cd0) + per-symbol 32-pt FFT, matching
/// `lib/ft8/ft8b.f90:154-161` exactly. Out-of-band signals (broadband
/// birdies, sidelobes) are suppressed by the downsample's
/// edge-tapered filter, instead of leaking into per-tone DFT
/// sidelobes as they would in a rectangular-window per-tone DFT.
/// Used by both host f32 and host fixed-point builds.
#[doc(hidden)]
#[cfg(feature = "fft-rustfft")]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    fill_symbol_spectra_via_cd0(out, audio, freq_hz, dt_sec, mask, fft_cache);
}

/// Embedded fallback (no `fft-rustfft` available — Xtensa cannot run
/// the 192k cd0 FFT). Reverts to the rectangular-window per-tone DFT
/// for non-fixed-point builds; the fixed-point variant has its own
/// basis-precompute path in `fill_symbol_spectra_into`.
///
/// The `fft_cache` parameter is accepted for API parity with the
/// `fft-rustfft` variant but ignored — there is no 192k FFT to skip
/// on this path.
#[doc(hidden)]
#[cfg(all(not(feature = "fft-rustfft"), not(feature = "fixed-point")))]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    let _ = fft_cache;
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// WSJT-X cd0-based per-symbol FFT. Mirrors `ft8b.f90:154-161`:
/// ```fortran
/// call ft8_downsample(dd, newdat, f1, cd0)
/// do k=1,NN
///   i1 = ibest + (k-1)*32
///   csymb = cd0(i1:i1+31)
///   call four2a(csymb, 32, 1, -1, 1)   ! 32-pt FFT
///   cs(0:7,k) = csymb(1:8) / 1e3
/// enddo
/// ```
/// Per-call cost: 79 × 32-pt FFT + (one 192k forward FFT only if the
/// caller did not supply `fft_cache`) + one 3.2k inverse FFT.
/// Passing `Some(cache)` from the multipass driver (the cache built
/// once per slot in `decode_frame_inner`) skips the 192k forward FFT
/// per candidate — ~5 ms saved × 30+ candidates × 3 passes.
#[cfg(feature = "fft-rustfft")]
fn fill_symbol_spectra_via_cd0<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    use rustfft::FftPlanner;
    extern crate alloc;

    // S → i16 conversion (no-op when S=i16 already). Per-call alloc
    // — still wasteful when cand-loop calls this 30+ times per slot.
    // See #55 (perf-B) for the caller-buffer hoist. Skipped entirely
    // when `fft_cache` is supplied (the slot-cache path doesn't need
    // the audio bytes — only the precomputed forward FFT).
    let cd0 = match fft_cache {
        Some(cache) => crate::core::dsp::downsample::downsample_cached(
            cache,
            freq_hz,
            &crate::ft8::downsample::FT8_CFG,
        ),
        None => {
            let audio_i16: alloc::vec::Vec<i16> = audio.iter().map(|s| s.to_i16()).collect();
            crate::ft8::downsample::downsample(&audio_i16, freq_hz, None).0
        }
    };

    // ibest in cd0 sample units (200 sps). dt_sec is offset from
    // TX_START_OFFSET_S = 0.5 s; cd0[0] corresponds to slot t=0,
    // so the first symbol starts at sample (0.5 + dt) × 200.
    let ibest = ((TX_START_OFFSET_S + dt_sec) * 200.0).round() as i32;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(32);
    let mut buf = [Complex::new(0.0_f32, 0.0); 32];

    // WSJT-X scales `cs = csymb / 1e3` (ft8b.f90:159). The /1e3 is
    // absorbed by `normalize_bmet` in the LLR pipeline, but we keep
    // it for traceable parity with WSJT-X numerics.
    const CS_SCALE: f32 = 1.0 / 1000.0;

    let np2 = cd0.len() as i32;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let i1 = ibest + (sym as i32) * 32;
        // WSJT-X ft8b.f90:155-157 — *all-or-nothing* boundary check:
        //   csymb = cmplx(0.0, 0.0)
        //   if( i1.ge.0 .and. i1+31 .le. NP2-1 ) csymb = cd0(i1:i1+31)
        // i.e. when ANY of the 32 samples falls outside cd0, the whole
        // window is set to zero. Per-element fill (= use partial cd0
        // data) was an incorrect simplification that pulled extra
        // signal energy into edge symbols and shifted the LLR sign
        // pattern away from WSJT-X's.
        if i1 >= 0 && i1 + 31 < np2 {
            for j in 0..32 {
                buf[j] = cd0[(i1 + j as i32) as usize];
            }
        } else {
            for j in 0..32 {
                buf[j] = Complex::new(0.0, 0.0);
            }
        }
        fft.process(&mut buf);
        // csymb(1:8) (Fortran) = bins 0..7 (0-based) = tones 0..7.
        for tone in 0..NTONES {
            out[sym][tone] = Cmplx {
                re: buf[tone].re * CS_SCALE,
                im: buf[tone].im * CS_SCALE,
            };
        }
    }
}

/// Generic per-symbol DFT — writes `Cmplx<Sc>` for any spec scalar
/// `Sc: SpecScalar`. For `Sc = f32` (`NEEDS_AUTOGAIN = false`) the
/// inner loop writes f32 components directly — byte-identical to the
/// pre-Phase-2.6 implementation. For fixed-point `Sc` (`NEEDS_AUTOGAIN
/// = true`) the function runs a 2-pass scan-and-scale: compute all
/// 79 × 8 Complex<f32> entries into a stack tmp buffer (~5 KB), find
/// the peak |re|/|im| across the active mask, then write
/// `Sc::from_f32_scaled(value, scale)` with `scale = i16::MAX × 0.95
/// / peak` so the i16 range is fully utilised without saturation.
#[doc(hidden)]
#[cfg(not(feature = "fixed-point"))]
pub fn fill_symbol_spectra_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;

    let mut rotators = [Complex::new(0.0f32, 0.0); NTONES];
    for tone in 0..NTONES {
        let tone_freq = freq_hz + tone as f32 * TONE_SPACING_HZ;
        let dphi = -two_pi_over_fs * tone_freq;
        rotators[tone] = Complex::new(dphi.cos(), dphi.sin());
    }

    let mut sym_buf = [0.0f32; NSPS];

    if !Sc::NEEDS_AUTOGAIN {
        // Sc = f32: inline write via `Sc::from_f32` (no-op for f32).
        // Const dispatch — LLVM eliminates the `else` branch when
        // monomorphised for `Sc = f32`.
        for sym in 0..NN {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            let sym_start = i0 + (sym as i64) * (NSPS as i64);
            for k in 0..NSPS {
                let idx = sym_start + k as i64;
                sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                    audio[idx as usize].to_f32()
                } else {
                    0.0
                };
            }
            for tone in 0..NTONES {
                let rotator = rotators[tone];
                let mut osc = Complex::new(1.0f32, 0.0);
                let mut acc = Complex::new(0.0f32, 0.0);
                for &s in sym_buf.iter() {
                    acc.re += s * osc.re;
                    acc.im += s * osc.im;
                    osc *= rotator;
                }
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32(acc.re),
                    im: Sc::from_f32(acc.im),
                };
            }
        }
        return;
    }

    // Fixed-point path: 2-pass with auto-gain.
    let mut tmp = [[Complex::new(0.0f32, 0.0); 8]; 79];
    let mut peak: f32 = 0.0;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        for k in 0..NSPS {
            let idx = sym_start + k as i64;
            sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                audio[idx as usize].to_f32()
            } else {
                0.0
            };
        }
        for tone in 0..NTONES {
            let rotator = rotators[tone];
            let mut osc = Complex::new(1.0f32, 0.0);
            let mut acc = Complex::new(0.0f32, 0.0);
            for &s in sym_buf.iter() {
                acc.re += s * osc.re;
                acc.im += s * osc.im;
                osc *= rotator;
            }
            tmp[sym][tone] = acc;
            peak = peak.max(acc.re.abs()).max(acc.im.abs());
        }
    }
    let scale = if peak > 1e-9 {
        (i16::MAX as f32 * 0.95) / peak
    } else {
        0.0
    };
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        for tone in 0..NTONES {
            let c = tmp[sym][tone];
            out[sym][tone] = Cmplx {
                re: Sc::from_f32_scaled(c.re, scale),
                im: Sc::from_f32_scaled(c.im, scale),
            };
        }
    }
}

/// Required scratch length for [`fill_symbol_spectra_into`] — one
/// flat array per axis (cos / sin), `NTONES × NSPS = 15 360` i16.
/// Caller must provide two slices of at least this length.
#[cfg(feature = "fixed-point")]
pub const BASIS_SCRATCH_LEN: usize = NTONES * NSPS;

/// Fixed-point per-symbol DFT — basis-precompute + dot-product
/// kernel. Drop-in heap-allocating wrapper around
/// [`fill_symbol_spectra_into`]: allocates 60 KB × 2 of basis scratch
/// from the default heap on every call. Convenient for host use; on
/// embedded targets the scratch typically lands in PSRAM (slow reads
/// in the dot-product inner loop), so callers that care about Core2
/// throughput should pre-allocate scratch in **internal RAM**
/// (`static [i16; BASIS_SCRATCH_LEN]` in `.bss`, or
/// `heap_caps_malloc(MALLOC_CAP_INTERNAL)`) and call
/// [`fill_symbol_spectra_into`] directly.
// Fixed-point host with `fft-rustfft` uses the cd0-based
// `fill_symbol_spectra` defined above. Pure embedded fixed-point
// (no fft-rustfft) goes through `fill_symbol_spectra_into` instead
// — this short heap-allocating wrapper is only kept for the
// `(fixed-point, !fft-rustfft)` build, which is the only one that
// would have called this entry.
#[doc(hidden)]
#[cfg(all(feature = "fixed-point", not(feature = "fft-rustfft")))]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    let _ = fft_cache;
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// Generic fixed-point fill — writes `Cmplx<Sc>`. f32 wrapper above.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let mut basis_re: Vec<i16> = alloc::vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im: Vec<i16> = alloc::vec![0i16; BASIS_SCRATCH_LEN];
    fill_symbol_spectra_into_generic::<Sc, S>(
        out,
        audio,
        freq_hz,
        dt_sec,
        mask,
        &mut basis_re,
        &mut basis_im,
    );
}

/// Fixed-point per-symbol DFT with caller-provided basis scratch.
///
/// Two phases per call:
/// 1. **Basis precompute** (in `basis_re` / `basis_im`) — 8 tones ×
///    {cos, sin} = 16 vectors of NSPS=1920 i16 samples, generated by
///    a Q15 rotator (one cos+sin pair per tone, then 1920 complex
///    multiplies to fill the vector).
/// 2. **Per-symbol dot products** — for each symbol in `mask`,
///    16 calls to [`crate::core::dotprod::dot_q15_i32`] against the
///    basis. Default is a Rust loop; embedded targets can override
///    via `mfsk_core_dot_q15_i32` to bridge to chip-native asm
///    (e.g. esp-dsp `dsps_dotprod_s16_ae32` on Xtensa LX6).
///
/// **Why caller-provided scratch?** On Core2 the basis is the inner
/// loop's hot data — esp-dsp's asm dot product runs at 1 cycle/sample
/// only when the basis lives in fast internal RAM. Default heap on
/// ESP32 with `CONFIG_SPIRAM_USE_MALLOC` puts a 60 KB allocation in
/// PSRAM (~5–10 cycles/sample read latency), which kills the asm
/// kernel's advantage. Pre-allocating scratch in `.bss` (static
/// arrays land in internal DRAM) lets the dot product reach its
/// theoretical speed.
///
/// Both `basis_re` and `basis_im` must be at least
/// [`BASIS_SCRATCH_LEN`] long — debug-asserted; longer is fine
/// (only the prefix is used).
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_into<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) {
    fill_symbol_spectra_into_generic::<f32, S>(
        out, audio, freq_hz, dt_sec, mask, basis_re, basis_im,
    );
}

/// Generic version of [`fill_symbol_spectra_into`] — writes
/// `Cmplx<Sc>` for any spec scalar. f32 wrapper above.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_into_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) {
    use crate::core::dotprod::dot_q15_i32;
    debug_assert!(basis_re.len() >= BASIS_SCRATCH_LEN);
    debug_assert!(basis_im.len() >= BASIS_SCRATCH_LEN);
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;

    // ── Phase 1: precompute Q15 basis vectors (cos, sin × 8 tones).
    for tone in 0..NTONES {
        let tone_freq = freq_hz + tone as f32 * TONE_SPACING_HZ;
        let dphi = -two_pi_over_fs * tone_freq;
        let rot_re = (dphi.cos() * 32767.0).round() as i32;
        let rot_im = (dphi.sin() * 32767.0).round() as i32;
        let mut osc_re: i32 = 32767;
        let mut osc_im: i32 = 0;
        let base = tone * NSPS;
        for k in 0..NSPS {
            basis_re[base + k] = osc_re as i16;
            basis_im[base + k] = osc_im as i16;
            let new_re = ((osc_re * rot_re) - (osc_im * rot_im)) >> 15;
            let new_im = ((osc_re * rot_im) + (osc_im * rot_re)) >> 15;
            osc_re = new_re;
            osc_im = new_im;
        }
    }

    // Stack buffer: one symbol of audio as i16. 1920 × 2 = 3.8 KB.
    let mut sym_buf = [0i16; NSPS];

    // ── Phase 2: per-symbol dot products (audio × basis).
    //
    // For `Sc = f32` (no autogain) we write each cell straight into
    // `out` via `Sc::from_f32` (no-op cast). For fixed-point types
    // we collect i32 accumulators into a stack tmp buffer
    // (~40 KB i32×8×79×2), find the peak, and write
    // `Sc::from_f32_scaled(acc as f32, scale)` so the i16 output
    // range is fully utilised. The scratch is a stack array — fits
    // the 32 KB Core2 main task stack with the existing 16 KB used
    // by basis scratch references and the spec.
    let mut tmp_re = [[0i32; 8]; 79];
    let mut tmp_im = [[0i32; 8]; 79];
    let mut peak: i32 = 0;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        for k in 0..NSPS {
            let idx = sym_start + k as i64;
            sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                audio[idx as usize].to_i16()
            } else {
                0
            };
        }
        for tone in 0..NTONES {
            let base = tone * NSPS;
            let basis_re_tone = &basis_re[base..base + NSPS];
            let basis_im_tone = &basis_im[base..base + NSPS];
            let acc_re = dot_q15_i32(&sym_buf, basis_re_tone);
            let acc_im = dot_q15_i32(&sym_buf, basis_im_tone);
            if !Sc::NEEDS_AUTOGAIN {
                // f32 fast path — direct write, identical to the
                // pre-2.6 cast.
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32(acc_re as f32),
                    im: Sc::from_f32(acc_im as f32),
                };
            } else {
                tmp_re[sym][tone] = acc_re;
                tmp_im[sym][tone] = acc_im;
                peak = peak.max(acc_re.unsigned_abs() as i32);
                peak = peak.max(acc_im.unsigned_abs() as i32);
            }
        }
    }

    // 2-pass auto-gain: scale i32 accumulators into i16 range,
    // saturating safe, peak ≈ 95 % of i16::MAX. Skipped on the f32
    // fast path (`out` is already populated above).
    if Sc::NEEDS_AUTOGAIN {
        let scale = if peak > 0 {
            (i16::MAX as f32 * 0.95) / peak as f32
        } else {
            0.0
        };
        for sym in 0..NN {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            for tone in 0..NTONES {
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32_scaled(tmp_re[sym][tone] as f32, scale),
                    im: Sc::from_f32_scaled(tmp_im[sym][tone] as f32, scale),
                };
            }
        }
    }
}

/// Heap-allocating sibling of [`symbol_spectra_direct`] that uses a
/// caller-provided basis scratch (passed through to
/// [`fill_symbol_spectra_into`]). Only the fixed-point variant is
/// exposed — host f32 path doesn't need a scratch (`fill_symbol_spectra`
/// f32 has no basis precompute step).
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn symbol_spectra_direct_into<S: AudioSample>(
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    sym_mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra_into(
        &mut out, audio, freq_hz, dt_sec, sym_mask, basis_re, basis_im,
    );
    out
}

// ── Public entry ────────────────────────────────────────────────────────────

/// Embedded FT8 decode for one 15-s slot.
///
/// Runs the same algorithm shape as [`decode_frame`](super::decode::decode_frame)
/// but talks only to power-of-two FFTs (via the
/// [`crate::core::fft::FftPlanner`] trait) and uses the min-sum LDPC
/// kernel to skip per-iteration `tanh` / `atanh`. No
/// `decode_sniper*` paths are involved; no wide-band 192 k FFT cache.
///
/// Sensitivity vs `decode_frame` is characterised on host AWGN
/// sweeps before any embedded port — see
/// `tests/ft8_decode_block_snr_sweep.rs`.
///
/// # Arguments
/// * `audio`     — 12 kHz i16 PCM, length up to NMAX = 180 000.
/// * `freq_min`  — lower edge of carrier search (Hz).
/// * `freq_max`  — upper edge of carrier search (Hz).
/// * `sync_min`  — minimum normalised Costas score (typical 1.0–2.0).
/// * `depth`     — `Bp` / `BpAll` / `BpAllOsd`.
/// * `max_cand`  — cap on Costas candidates evaluated.
pub fn decode_block<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        None,
        DecodeStrictness::Normal,
    )
}

/// Variant of [`decode_block`] that accepts a runtime `bp_max_iter`
/// (= per-LLR-variant BP iteration cap, default
/// [`DEFAULT_BP_MAX_ITER`]). Useful on time-budgeted targets.
#[allow(clippy::too_many_arguments)]
pub fn decode_block_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
    )
}

/// AP-aware variant of [`decode_block`]. Mirrors `decode_block`'s
/// behaviour exactly when `ap_hint = None`; with `Some(&ap)` runs
/// the full WSJT-X iaptype loop (5..12) per candidate after Steps 1-3
/// (BP staircase + OSD) all fail. Host `fft-rustfft` build only —
/// embedded fixed-point keeps its existing iaptype-1-only path.
///
/// Used by host `decode_frame_with_ap` after the v0.6.1 redirect, and
/// by mountain-top apps that want full AP rescue from a single entry
/// point. New in 0.6.1.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_with_ap<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        ap_hint,
        DecodeStrictness::Normal,
    )
}

/// Variant of [`decode_block_with_ap`] that accepts runtime
/// `bp_max_iter` and `strictness`. New in 0.6.1.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_with_ap_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        bp_max_iter,
        ap_hint,
        strictness,
    )
}

/// WSJT-X `ft8_decode.f90:172-236` 3-pass loop driver. Each pass:
/// coarse_sync on the (subtracted) audio, fine refine, decode, then
/// LPF-subtract every fresh CRC-passing decode for the next pass.
///
/// Pass termination matches WSJT-X exactly:
/// - pass 1 always runs;
/// - pass 2 skips when pass 1 returned 0 decodes;
/// - pass 3 skips when pass 2 returned no NEW decodes.
///
/// On host (`fft-rustfft`) the audio is cloned to a working `Vec<i16>`
/// (subtract operates on i16 samples). Embedded targets compile through
/// the same path; the clone cost is dominated by the BP work it enables.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
fn decode_block_multipass<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    use alloc::vec::Vec as AllocVec;
    let mut work: AllocVec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let mut all: AllocVec<DecodeResult> = AllocVec::new();
    let mut prev_total: usize = 0;
    let mut sbase_and_spec: Option<(AllocVec<f32>, Spectrogram)> = None;
    for ipass in 0..3 {
        if ipass >= 1 && all.len() == prev_total {
            // Pass 2 skips on zero from pass 1; pass 3 on zero new.
            break;
        }
        prev_total = all.len();

        let spec = compute_spectrogram(work.as_slice(), freq_max);
        // Capture pass 1's spectrogram + per-bin baseline (= ORIGINAL
        // audio, before any subtract) for WSJT-X-faithful xsnr2 SNR.
        // ft8b.f90:449 reads sbase from the pre-subtract baseline so
        // xsnr2 stays stable across passes; xsig is also read from
        // the same spectrogram so the two share an absolute scale.
        if ipass == 0 {
            let mut avg = alloc::vec![0.0_f32; spec.n_freq];
            crate::ft8::baseline::avg_spectrum(&spec, &mut avg);
            let sbase_v = crate::ft8::baseline::fit_baseline(&avg, 0, spec.n_freq - 1);
            let spec_clone = Spectrogram {
                n_freq: spec.n_freq,
                n_time: spec.n_time,
                data: spec.data.clone(),
            };
            sbase_and_spec = Some((sbase_v, spec_clone));
        }
        let cands = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
        drop(spec);
        let cands = fine_refine_pass1(work.as_slice(), cands);
        let pass2 = refine_candidates(work.as_slice(), cands, max_cand);

        // **WSJT-X ft8b.f90:432-437 sequential subtract**: each
        // accepted decode immediately subtracts from `work` so the
        // NEXT candidate in this same pass sees a cleaner residual.
        // Without this, all candidates in a pass see the same raw
        // audio — strong real signals at one freq leak Costas-aligned
        // energy into nearby phantom candidates' bins, allowing
        // CRC-pass garbage to decode there. The driver's outer pass
        // loop is for OSD/AP differences (ndepth-dependent), not for
        // the subtract cadence.
        #[cfg(feature = "std")]
        let trace = std::env::var("MFSK_TRACE_PHANTOM").is_ok();
        #[cfg(not(feature = "std"))]
        let trace = false;
        for cand in pass2 {
            let single_results = process_candidates_tuned_with_ap(
                work.as_slice(),
                alloc::vec![cand],
                depth,
                DEFAULT_Q_THRESH,
                bp_max_iter,
                ap_hint,
                strictness,
            );
            for r in single_results {
                if all.iter().any(|x| x.message77 == r.message77) {
                    continue;
                }
                if trace {
                    #[cfg(feature = "std")]
                    if let Some(text) = crate::msg::wsjt77::unpack77(&r.message77) {
                        eprintln!(
                            "  TRACE pass={} freq={:>7.2} dt={:+.4} e={:>2} '{}'",
                            ipass, r.freq_hz, r.dt_sec, r.hard_errors, text,
                        );
                    }
                }
                crate::ft8::subtract::subtract_signal_lpf(work.as_mut_slice(), &r);
                all.push(r);
            }
        }
    }

    // Replace each result's snr_db with WSJT-X xsnr2 (pre-subtract
    // spectrogram + baseline). `xsig` is read directly from the
    // captured pass-1 spectrogram at each tone position, so xsig
    // and xbase share the exact same absolute scale (= the original
    // sync8 spectrogram). This is critical: feeding xsig from a
    // different chain (e.g. cd0 downsampled per-symbol DFT) loses
    // the calibration.
    // xsnr2/xbase post-process is f32-only. Fixed-point Spectrogram
    // cells are quantised post `>> FP_SPEC_SHIFT`, putting many noise
    // cells at u16 zero — `fit_baseline`'s `log10(p.max(1e-30))` then
    // produces sbase ≈ -250 dB and xsnr2 explodes. The original
    // adjacent-tone SNR from `process_candidates_into` (compute_snr_db)
    // is already on a sensible scale, so leave it untouched on the
    // fixed-point path. late bail (`xsnr<-24 && nsync≤10`) likewise
    // f32-only here; on qso3_busy it had no effect in any case.
    #[cfg(not(feature = "fixed-point"))]
    if let Some((sbase, spec)) = sbase_and_spec {
        let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
        let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
        let nsps_steps = (NSPS / NSTEP) as f32;
        all.retain_mut(|r| {
            r.snr_db = recompute_snr_xsnr2(r, &spec, &sbase, df, tstep, nsps_steps, 1.0);
            let nsync = recompute_nsync(r, &spec, df, tstep, nsps_steps);
            !(nsync <= 10 && r.snr_db < -24.0)
        });
    }
    #[cfg(feature = "fixed-point")]
    let _ = sbase_and_spec;
    all
}

/// Hard-decision sync count (= WSJT-X `ft8b.f90:163-176` nsync) read
/// from the pass-1 spectrogram at the result's refined (freq, dt).
/// 21-bit upper bound (3 sync blocks × 7 Costas positions).
#[cfg(feature = "fft-rustfft")]
fn recompute_nsync(
    result: &DecodeResult,
    spec: &Spectrogram,
    df: f32,
    tstep: f32,
    nsps_steps: f32,
) -> u32 {
    use crate::ft8::params::COSTAS;
    const NTONES: usize = 8;
    let carrier_bin_f = result.freq_hz / df;
    let tone_step = TONE_SPACING_HZ / df; // = 2.0 at NFFT=3840
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;
    // Costas blocks at symbol indices 0, 36, 72 (each 7 symbols long).
    let mut count = 0u32;
    for &block_off in &[0_usize, 36, 72] {
        for (sym_in_block, &expected) in COSTAS.iter().enumerate() {
            let k = block_off + sym_in_block;
            let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
            if m_bin < 0 || m_bin as usize >= spec.n_time {
                continue;
            }
            let m_bin = m_bin as usize;
            let mut best_t = 0;
            let mut best_p = f32::MIN;
            for t in 0..NTONES {
                let f_bin = (carrier_bin_f + (t as f32) * tone_step).round() as i32;
                if f_bin < 0 || f_bin as usize >= spec.n_freq {
                    continue;
                }
                let p = spec.power_acc(f_bin as usize, m_bin);
                if p > best_p {
                    best_p = p;
                    best_t = t;
                }
            }
            if best_t == expected {
                count += 1;
            }
        }
    }
    count
}

/// Slot-baseline xsnr2 SNR for any [`Spectrogram`] — `std`-free
/// alternative to `recompute_snr_xsnr2` for callers that haven't
/// run `baseline::fit_baseline`'s polynomial smoother (= the
/// embedded path: `mfsk-core` is built `default-features = false`
/// + `alloc` only there, so the polynomial fit is unavailable).
///
/// Computes the per-frequency baseline as the mean across time over
/// a ±50-bin window centred on the decode's carrier — same idea as
/// `avg_spectrum` but localised so it works incrementally and stays
/// cheap on Xtensa LX6/LX7. Then evaluates WSJT-X `ft8b.f90:449-454`:
///
/// ```text
///   xbase = mean_over_time(spec[carrier_window]) * cell_scale
///   xsig  = sum_over_79_decoded_tones(spec[tone_bin, m]) * cell_scale
///   xsnr2 = xsig / xbase / 3e6 - 1
///   snr_db = 10·log10(xsnr2) - 27        (clamped at -24 dB on degeneracy)
/// ```
///
/// `cell_scale = 1.0` for an `f32` spectrogram, `2^FP_SPEC_SHIFT`
/// (`= 4096.0` with the default fixed-point shift) for an embedded
/// u16 spectrogram so xsig and xbase land in the same WSJT-X
/// calibration regime.
///
/// Vs the per-Costas-block adjacent-tone SNR `compute_snr_db` returns
/// from `process_candidates*`: that ratio is preserved under the
/// `fill_symbol_spectra` per-block auto-gain so it's *internally*
/// consistent, but its absolute number drifts ~0–15 dB between
/// signals because each block's gain factor is signal-dependent.
/// This function reads `xsig`/`xbase` from the *single* sync8
/// spectrogram (uniform `FP_SPEC_SHIFT` auto-gain across the slot),
/// so the result is comparable across signals AND comparable to
/// WSJT-X / JTDX SNR reports.
pub fn xsnr2_db_simple(spec: &Spectrogram, result: &DecodeResult, cell_scale: f32) -> f32 {
    use crate::ft8::params::NN;
    use crate::ft8::wave_gen::message_to_tones;

    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let nsps_steps = (NSPS / NSTEP) as f32;
    let tone_step = TONE_SPACING_HZ / df;

    if spec.n_freq == 0 || spec.n_time == 0 {
        return -24.0;
    }

    // Per-freq baseline — **median** (P50) of cell values inside a
    // local window around the decode's carrier. A plain mean is
    // dragged upward by the very signal we're trying to measure, so
    // xbase tracks xsig and the ratio collapses (verified
    // empirically: W1FC at 0 dB had xbase ≈ 4.8 M while N1JFU at
    // -14 dB had ≈ 0.15 M before this change). Median ignores the
    // signal-bin outliers and reads true noise floor.
    //
    // Window: ±50 freq bins (~156 Hz at NFFT=3840) × time-decimated
    // by `n_time / 50` so the sort touches ~2 500 samples. Sort cost
    // on f32 ~25 k comparisons ~100 µs at LX7 240 MHz — well inside
    // the post-SlotEnd budget.
    let carrier_bin = (result.freq_hz / df)
        .round()
        .clamp(0.0, (spec.n_freq - 1) as f32) as usize;
    let f_lo = carrier_bin.saturating_sub(50);
    let f_hi = (carrier_bin + 50).min(spec.n_freq - 1);
    let t_stride = spec.n_time.div_ceil(50).max(1);
    let mut samples: alloc::vec::Vec<f32> =
        alloc::vec::Vec::with_capacity((f_hi - f_lo + 1) * spec.n_time.div_ceil(t_stride) + 4);
    for f in f_lo..=f_hi {
        let mut t = 0usize;
        while t < spec.n_time {
            samples.push(spec.power_acc(f, t));
            t += t_stride;
        }
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let median = if samples.is_empty() {
        0.0
    } else {
        samples[samples.len() / 2]
    };
    let xbase = median * cell_scale;
    if xbase <= 0.0 || !xbase.is_finite() {
        return -24.0;
    }

    // xsig at the 79 decoded-tone (freq, m) positions.
    let itone = message_to_tones(&result.message77);
    let carrier_bin_f = result.freq_hz / df;
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;
    let mut xsig: f32 = 0.0;
    for k in 0..NN {
        let t = itone[k] as f32;
        let f_bin = (carrier_bin_f + t * tone_step).round() as i32;
        let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
        if f_bin < 0 || f_bin as usize >= spec.n_freq || m_bin < 0 || m_bin as usize >= spec.n_time
        {
            continue;
        }
        xsig += spec.power_acc(f_bin as usize, m_bin as usize);
    }
    xsig *= cell_scale;

    // Empirical calibration constant — pair-matched against the
    // JTDX qso3_busy reference on real M5StickS3 silicon
    // (2026-05-05). With the median-of-window noise floor above the
    // raw `xsig/xbase` ratio spans ~1100 (-14 dB SNR) → ~100 000
    // (0 dB SNR) on the embedded u16 spectrogram; the value below
    // collapses that to JTDX-comparable dB within ±3 dB across
    // weak / mid / strong signals.
    //
    // (WSJT-X's `ft8b.f90:451` value `3e6` is calibrated to their
    // f32 spectrogram amplitude scale and doesn't carry over here.)
    const XSNR2_CAL_DB: f32 = 46.0;

    let ratio = xsig / xbase;
    if ratio <= 1.0 {
        return -24.0;
    }
    let snr = 10.0 * ratio.log10() - XSNR2_CAL_DB;
    snr.max(-24.0)
}

/// WSJT-X `ft8b.f90:449-454` xsnr2 SNR formula. Reads the signal's
/// per-symbol power from the pass-1 spectrogram (= pre-subtract,
/// matching WSJT-X's sync8 spectrogram convention) at each of the
/// 79 expected tones, then divides by the per-frequency baseline
/// `sbase`:
///
/// ```text
///   xbase = 10^((sbase[round(f1/df)] - 40) / 10)
///   xsnr2 = xsig / xbase / 3e6 - 1
///   xsnr2_db = 10·log10(xsnr2) - 27
/// ```
///
/// Both `xsig` and `xbase` are on the same spectrogram scale, so the
/// formula's `/3e6 - 27` calibration (WSJT-X's "2.5 kHz reference"
/// convention) maps directly into a WSJT-X-compatible dB number.
///
/// Falls back to `-24 dB` if the ratio degenerates.
#[cfg(feature = "fft-rustfft")]
fn recompute_snr_xsnr2(
    result: &DecodeResult,
    spec: &Spectrogram,
    sbase: &[f32],
    df: f32,
    tstep: f32,
    nsps_steps: f32,
    cell_scale: f32,
) -> f32 {
    let itone = crate::ft8::wave_gen::message_to_tones(&result.message77);
    let carrier_bin_f = result.freq_hz / df;
    let tone_step = TONE_SPACING_HZ / df;
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;

    let mut xsig = 0.0_f32;
    for k in 0..79_usize {
        let t = itone[k] as f32;
        let f_bin = (carrier_bin_f + t * tone_step).round() as i32;
        let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
        if f_bin < 0 || f_bin as usize >= spec.n_freq || m_bin < 0 || m_bin as usize >= spec.n_time
        {
            continue;
        }
        xsig += spec.power_acc(f_bin as usize, m_bin as usize);
    }
    // `cell_scale` reverts the fixed-point spectrogram's
    // `>> FP_SPEC_SHIFT` so xsig and xbase live in WSJT-X's calibration
    // regime. For the f32 spectrogram cell_scale=1.0 (no-op).
    xsig *= cell_scale;

    let bin = carrier_bin_f.round() as i32;
    let bin = bin.clamp(0, sbase.len() as i32 - 1) as usize;
    // Same compensation on xbase: sbase_db came from `fit_baseline` of
    // post-shift cells, so its log10 reads ~36 dB low in fixed-point.
    let sbase_db_compensated = sbase[bin] + 10.0 * cell_scale.log10();
    let xbase = 10f32.powf(0.1 * (sbase_db_compensated - 40.0));
    let arg = xsig / xbase / 3.0e6 - 1.0;
    if arg <= 0.1 {
        return -24.0;
    }
    let snr = 10.0 * arg.log10() - 27.0;
    snr.max(-24.0)
}

/// Embedded path: single-pass `decode_block` (matches the previous
/// production behaviour, no subtract). Host-only `fft-rustfft` adds
/// the multipass driver. `_ap_hint` and `_strictness` parameters are
/// accepted for signature parity with the host variant but ignored
/// here — embedded fixed-point keeps its existing iaptype-1-only
/// hardcoded plumbing (full AP deferred to 0.7.x).
#[cfg(not(feature = "fft-rustfft"))]
#[allow(clippy::too_many_arguments)]
fn decode_block_multipass<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    _ap_hint: Option<&ApHint>,
    _strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass1 = fine_refine_pass1(audio, pass1);
    let pass2 = refine_candidates(audio, pass1, max_cand);
    process_candidates_tuned(audio, pass2, depth, DEFAULT_Q_THRESH, bp_max_iter)
}

/// Per-candidate WSJT-X-style 3-stage fine refine. Builds the
/// 192k-FFT cache once and downsamples per candidate. Host-only;
/// embedded paths skip this for compute reasons (cache is 1.5 MB,
/// 192k FFT is not in our embedded planner).
#[cfg(feature = "fft-rustfft")]
fn fine_refine_pass1<S: AudioSample>(
    audio: &[S],
    cands: alloc::vec::Vec<crate::core::sync::SyncCandidate>,
) -> alloc::vec::Vec<crate::core::sync::SyncCandidate> {
    if cands.is_empty() {
        return cands;
    }
    // Convert audio → Vec<i16> for the downsampler (no-op when S=i16).
    let audio_i16: alloc::vec::Vec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let fft_cache = crate::ft8::downsample::build_fft_cache(&audio_i16);
    cands
        .into_iter()
        .map(|c| {
            let (cd0, _) =
                crate::ft8::downsample::downsample(&audio_i16, c.freq_hz, Some(&fft_cache));
            let r = crate::ft8::refine_fine::fine_refine_3stage(&cd0, c.dt_sec);
            crate::core::sync::SyncCandidate {
                freq_hz: c.freq_hz + r.delf_hz,
                dt_sec: r.dt_sec,
                score: c.score,
            }
        })
        .collect()
}

/// Embedded build path — preserve the original (no fine refine) shape.
///
/// Attempted in 0.6.3 via `fill_symbol_spectra` iteration at
/// 41 (freq, dt) probe points per candidate, but the per-symbol
/// DFT cost (~6900 DFTs/cand × 30 cand × 1920-sample DFT ≈ 200k
/// DFTs/slot) tripped the FreeRTOS task watchdog after ~5 s of
/// uninterrupted compute on S3 LX7 — fundamentally too heavy
/// without the host's 192k FFT shortcut. Reverted to NO-OP. A
/// proper embedded fine_refine needs cd0 built via FIR decimate
/// (3:1 → 4:1 → 5:1, integer ratios, ~30 ms total) followed by
/// `refine_fine_3stage` on the 200 Hz baseband — deferred to a
/// future patch (estimate: ~150 lines for the FIR decimator).
#[cfg(not(feature = "fft-rustfft"))]
fn fine_refine_pass1<S: AudioSample>(
    _audio: &[S],
    cands: alloc::vec::Vec<crate::core::sync::SyncCandidate>,
) -> alloc::vec::Vec<crate::core::sync::SyncCandidate> {
    cands
}

/// Variant of [`decode_block`] that accepts caller-provided
/// basis scratch — required on Core2 / Cortex-M for the asm dot
/// product to reach its theoretical 1-cycle/sample speed (default
/// heap allocation routes the 60 KB × 2 basis to PSRAM /
/// non-cacheable RAM and erases the kernel's advantage).
///
/// Both `basis_re` and `basis_im` must be at least
/// [`BASIS_SCRATCH_LEN`] long; place them in **internal-RAM `.bss`**
/// (`static [i16; BASIS_SCRATCH_LEN]` arrays) for max throughput.
/// Same recall / depth / staircase as `decode_block`, just no
/// per-candidate allocation cycles.
#[cfg(feature = "fixed-point")]
pub fn decode_block_into<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    decode_block_into_tuned(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        basis_re,
        basis_im,
    )
}

/// Variant of [`decode_block_into`] that accepts a runtime
/// `bp_max_iter`. The recommended top-level entry point for embedded
/// LX6 / LX7 callers — `bp_max_iter` is the dominant time-scaling
/// knob in the post-SlotEnd budget.
#[cfg(feature = "fixed-point")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_into_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass2 = refine_candidates_into(audio, pass1, max_cand, basis_re, basis_im);
    process_candidates_into_tuned(
        audio,
        pass2,
        depth,
        DEFAULT_Q_THRESH,
        bp_max_iter,
        basis_re,
        basis_im,
    )
}

/// Pass-1 candidate cap — coarse_sync emits at most this many
/// candidates regardless of `max_cand`. Pass 2 re-ranks by
/// `sync_quality` (the same metric stage 3 uses to gate decode
/// attempts — much sharper than the per-bin power ratio) and
/// truncates to caller's `max_cand` for stage 3.
///
/// Sweep on real-QSO WAVs (host fp i16, BpAll, with the regularised
/// coarse_sync ratio in `RATIO_EPS_DEFAULT`) showed:
/// - PASS1 ∈ {30, 50}: 14/22 truth (drops one weak qso1 signal)
/// - PASS1 ∈ {75, 100}: 15/22 truth (full recall ceiling)
/// - PASS1=200: same 15/22 (no further gain — qso3's remaining gap
///   is at coarse_sync rank 100+, beyond Pass 2's reach).
///
/// 75 is the smallest PASS1 that keeps the full recall ceiling.
/// 30 is the smallest PASS1 that keeps the qso3 (busy band) truth
/// ceiling — it loses one borderline -17 dB qso1 signal (OH3NIV).
/// Core2 ships with 30 (speed-priority — Pass 2 cost ≈ 0.4 s vs
/// 1.0 s at PASS1=75). Override per-call via `MFSK_PASS1_LIMIT`
/// when std is enabled.
const PASS1_LIMIT_DEFAULT: usize = 30;
fn pass1_limit() -> usize {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_PASS1_LIMIT")
            && let Ok(v) = s.parse::<usize>()
        {
            return v;
        }
    }
    PASS1_LIMIT_DEFAULT
}

/// One Pass-2 output: the original candidate, its 79×8 Costas-only
/// spectrum (filled in stage 3 with the data-symbol DFT), and its
/// `sync_quality` score for ranking.
pub type RefinedCandidate = (SyncCandidate, Box<[[Cmplx<f32>; 8]; 79]>, u32);

/// Per-candidate Costas-block-0 DFT + sync_quality_block0 re-rank.
/// Keeps the top `max_cand` by Pass-2 score; **the cs spectrum is
/// retained** (block 0 only at this point) and stage 3 fills the
/// remaining 72 symbols via [`SymMask::NotBlock0`].
///
/// Cost: 7 sync symbols × 8 tones = 56 DFT per candidate vs
/// `SyncOnly`'s 168 — 1/3 the work. On Core2 ~13 ms/cand with the
/// asm dot product. PASS1=75 → Pass 2 ≈ 1.0 s.
///
/// The retained `q` is the **block-0** score (range 0..=7, expected
/// ~6-7 for real signals, ~0.875 for noise). Stage 3 recomputes the
/// full 21-symbol `sync_quality` after filling blocks 1, 2 and uses
/// that for its `q > 6` gate; the per-cand sort here is just for
/// truncating to `max_cand`.
fn refine_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct(audio, c.freq_hz, c.dt_sec, SymMask::SyncBlock0)
    })
}

/// Variant of [`refine_candidates`] that uses caller-provided basis
/// scratch (forwards to `symbol_spectra_direct_into`).
///
/// Public as of v0.6 (#49 cat C): used by the embedded
/// `embedded-shared::dual_core` worker to keep the basis scratch
/// allocated in internal DRAM across stages.
#[cfg(feature = "fixed-point")]
pub fn refine_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct_into(
            audio,
            c.freq_hz,
            c.dt_sec,
            SymMask::SyncBlock0,
            basis_re,
            basis_im,
        )
    })
}

/// Common min-heap selection logic used by both `refine_candidates`
/// (heap-allocated basis per call) and `refine_candidates_into`
/// (caller-provided basis scratch). The closure abstracts how each
/// candidate's cs Box is produced.
fn refine_candidates_with<F>(
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    mut cs_for: F,
) -> Vec<RefinedCandidate>
where
    F: FnMut(&SyncCandidate) -> Box<[[Cmplx<f32>; 8]; 79]>,
{
    use alloc::collections::BinaryHeap;
    use core::cmp::{Ordering, Reverse};

    // Min-heap on q so the smallest survivor is at the top — replace
    // it whenever a stronger candidate arrives. Bounds the live heap
    // footprint at `max_cand × cs Box` regardless of PASS1_LIMIT,
    // which is the heap-fragmentation fix Task #2 was opened for.
    // Old code collected all PASS1=30 cs Boxes (240 KB) before the
    // truncate; new code never holds more than max_cand=15 Boxes
    // (120 KB peak).
    //
    // The heap stores (q, cand_idx, RefinedCandidate); cand_idx
    // breaks ties deterministically (insertion order) so the
    // truncation result is reproducible across runs.
    struct Slot {
        q: u32,
        idx: u32,
        cand: SyncCandidate,
        cs: Box<[[Cmplx<f32>; 8]; 79]>,
    }
    impl PartialEq for Slot {
        fn eq(&self, other: &Self) -> bool {
            self.q == other.q && self.idx == other.idx
        }
    }
    impl Eq for Slot {}
    impl Ord for Slot {
        fn cmp(&self, other: &Self) -> Ordering {
            self.q.cmp(&other.q).then_with(|| self.idx.cmp(&other.idx))
        }
    }
    impl PartialOrd for Slot {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    let mut heap: BinaryHeap<Reverse<Slot>> = BinaryHeap::with_capacity(max_cand + 1);
    for (idx, c) in cands.into_iter().enumerate() {
        let cs = cs_for(&c);
        let q = sync_quality_block0(&cs);
        let slot = Slot {
            q,
            idx: idx as u32,
            cand: c,
            cs,
        };
        if heap.len() < max_cand {
            heap.push(Reverse(slot));
        } else if let Some(Reverse(top)) = heap.peek()
            && slot.q > top.q
        {
            heap.pop();
            heap.push(Reverse(slot));
            // else branch: drop slot.cs immediately when it leaves scope
        }
    }
    let mut out: Vec<RefinedCandidate> = heap
        .into_iter()
        .map(|r| {
            let s = r.0;
            (s.cand, s.cs, s.q)
        })
        .collect();
    out.sort_by_key(|r| core::cmp::Reverse(r.2));
    out
}

/// Hard-decision sync quality on Costas **block 0 only** (symbols
/// 0..7). Cheaper variant of [`sync_quality`] for Pass 2 — checks
/// only one of the three Costas blocks. Range 0..=7.
///
/// Pub-but-doc-hidden so embedded callers (e.g. the m5stack-core2
/// PoC's manual Pass 2) can re-rank coarse_sync candidates by this
/// metric without pulling in the full `decode_block` D-pattern.
#[doc(hidden)]
pub fn sync_quality_block0<S: crate::core::scalar::SpecScalar>(cs: &[[Cmplx<S>; 8]; 79]) -> u32
where
    S::Wide: PartialOrd,
{
    let mut count = 0u32;
    for (t, &expected) in COSTAS.iter().enumerate() {
        let sym = t; // block 0 starts at symbol 0
        let best = (0..NTONES)
            .max_by(|&a, &b| {
                let na = cs[sym][a].norm_sqr_wide();
                let nb = cs[sym][b].norm_sqr_wide();
                na.partial_cmp(&nb).unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap_or(0);
        if best == expected {
            count += 1;
        }
    }
    count
}

/// LLR / BP scalar for the hot loop. `Q3i8` under `fixed-point`
/// (embedded integer pipeline; recall-equivalent to Q11i16 with half
/// the BP scratch — Issue #15 Phase 1 validated 2026-05-03), `f32`
/// otherwise (host / FPU targets). Both go through the same generic
/// NMS implementation in `fec::ldpc::bp`.
///
/// **0.6.3**: switched embedded LlrT from `Q3i8` (i8, ±16 range, 1/8
/// resolution) to `Q11i16` (i16, ±16 range, 1/2048 resolution). The
/// Q3i8 quantization step (~0.875 LLR units between codes) was the
/// dominant recall ceiling on Xtensa builds — host fixed-point +
/// rustfft hit 16/18 with f32, 9/18 with Q3i8, on `qso3_busy.wav`.
/// Q11i16's 1/2048 resolution recovers most of that gap (target
/// embedded recall: 6/18 → ~10/18). Cost: BP scratch doubles from
/// ~6 KB to ~12 KB, still inside S3 / Core2 internal-DRAM budget.
#[cfg(feature = "fixed-point")]
type LlrT = crate::core::scalar::Q11i16;
#[cfg(not(feature = "fixed-point"))]
type LlrT = f32;

/// BP-kind switch (host-only). **Default `tanh`** (= WSJT-X
/// `bpdecode174_91.f90` log-domain tanh-product, our
/// `BpKind::SumProduct`) — that's the golden reference. The embedded
/// ship path keeps `NormalizedMinSum` (α=0.75) for speed; on host we
/// pay the tanh / atanh per-iteration cost in exchange for
/// numerically-correct convergence. The env var
/// `MFSK_BP_KIND=nms` opts into the approximation for A/B
/// comparison, but is NOT for production use — NMS appears to "find
/// more decodes" only because its approximation error sometimes
/// happens to land on a CRC-passing codeword that tanh would
/// (correctly) reject.
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
#[inline]
fn bp_step_select<T: crate::core::scalar::LlrScalar>(
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<crate::fec::ldpc::Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<crate::fec::ldpc::bp::BpResult> {
    if std::env::var("MFSK_BP_KIND").as_deref() == Ok("nms") {
        return crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<T>(
            bp_scratch, llr, None, max_iter, verify, NMS_ALPHA,
        );
    }
    let llr_f32: [f32; LDPC_N] = core::array::from_fn(|i| llr[i].to_f32());
    crate::fec::ldpc::bp::bp_decode(&llr_f32, None, max_iter, verify)
}

#[cfg(any(not(feature = "fft-rustfft"), feature = "fixed-point"))]
#[inline]
fn bp_step_select<T: crate::core::scalar::LlrScalar>(
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<crate::fec::ldpc::Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<crate::fec::ldpc::bp::BpResult> {
    crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<T>(
        bp_scratch, llr, None, max_iter, verify, NMS_ALPHA,
    )
}

/// Stage 3: take Pass-2 refined candidates (cand + Costas-only cs +
/// sync_quality), fill in the data-symbol spectra, run LLR + BP/OSD
/// staircase. The Costas DFT was already done in Pass 2 — we only
/// add the data-symbol DFT here.
///
/// `q_thresh` is the post-fill `sync_quality` early-reject threshold
/// (see [`DEFAULT_Q_THRESH`]).
///
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn process_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
) -> Vec<DecodeResult> {
    process_candidates_tuned(audio, cands, depth, q_thresh, DEFAULT_BP_MAX_ITER)
}

/// Variant of [`process_candidates`] that accepts a runtime
/// `bp_max_iter` (= per-LLR-variant BP iteration cap). Embedded
/// LX6 / LX7 callers tune this to trade weak-signal recall for
/// post-SlotEnd time budget.
#[doc(hidden)]
pub fn process_candidates_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    process_candidates_tuned_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
    )
}

/// AP-aware variant of [`process_candidates_tuned`]. When
/// `ap_hint = None` and `strictness = Normal`, behaviour is bit-
/// identical to `process_candidates_tuned`. Internal helper for the
/// `decode_block_with_ap` driver and host's redirected
/// `process_candidate`.
#[allow(clippy::too_many_arguments)]
fn process_candidates_tuned_with_ap<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    let mut cs_scratch: alloc::boxed::Box<[[Cmplx<f32>; 8]; 79]> =
        alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    process_candidates_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        &mut cs_scratch,
        |cs, cand, mask| {
            fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask, None);
        },
        ap_hint,
        strictness,
    )
}

/// Variant of [`process_candidates`] that uses caller-provided basis
/// scratch — required on Core2 / Cortex-M for the asm dot product
/// to reach its theoretical 1-cycle/sample speed (default heap
/// allocation routes the 60 KB × 2 basis to PSRAM /
/// non-cacheable RAM and erases the kernel's advantage).
///
/// **Pub for benchmarking + manually-staged callers** (e.g.
/// m5stack-core2 main.rs which logs per-stage wall-clock).
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
pub fn process_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    process_candidates_into_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        DEFAULT_BP_MAX_ITER,
        basis_re,
        basis_im,
    )
}

/// Variant of [`process_candidates_into`] that accepts a runtime
/// `bp_max_iter`. Same role as [`process_candidates_tuned`] but for
/// the caller-provided-basis path.
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
#[allow(clippy::too_many_arguments)]
pub fn process_candidates_into_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    let mut cs_scratch: alloc::boxed::Box<[[Cmplx<f32>; 8]; 79]> =
        alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    process_candidates_into_with_cs_scratch_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        basis_re,
        basis_im,
        &mut cs_scratch,
    )
}

/// Variant of [`process_candidates_into`] that also accepts a
/// caller-provided per-symbol-spectra scratch (`cs_scratch`, 5 KB =
/// `[[Cmplx<f32>; 8]; 79]`). Each candidate's PSRAM-resident `cs Box`
/// is copied into this scratch before `fill_symbol_spectra_into` /
/// LLR / BP run on it, then dropped — letting the BP / LLR hot loops
/// read `cs` from internal DRAM (~5–10× faster than PSRAM on Xtensa
/// LX6/LX7). Provide a `static mut` array in `.bss` for max win.
///
/// Public as of v0.6 (#49 cat C): used by the embedded
/// `embedded-shared::dual_core::stage3_split` worker.
#[cfg(feature = "fixed-point")]
pub fn process_candidates_into_with_cs_scratch<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    process_candidates_into_with_cs_scratch_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        DEFAULT_BP_MAX_ITER,
        basis_re,
        basis_im,
        cs_scratch,
    )
}

/// Variant of [`process_candidates_into_with_cs_scratch`] that accepts
/// a runtime `bp_max_iter`. This is the entry point used by the
/// embedded `dual_core::stage3_split` worker so LX6 / LX7 binaries
/// can dial the BP cap without rebuilding `mfsk-core`.
///
/// Public as of v0.6 (#49 cat C).
#[cfg(feature = "fixed-point")]
#[allow(clippy::too_many_arguments)]
pub fn process_candidates_into_with_cs_scratch_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    process_candidates_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        cs_scratch,
        |cs, cand, mask| {
            // Host fft-rustfft: use the cd0-based 32-pt FFT cs builder
            // (= ft8b.f90:154-161). Same path as host f32. fixed-point
            // with rustfft (= host) gets WSJT-X-faithful cs construction
            // identical to the f32 build.
            #[cfg(feature = "fft-rustfft")]
            {
                // basis_{re,im} unused on this path — borrow nothing.
                // fft_cache=None here because process_candidates_into_tuned
                // is the embedded entry, not the slot-cache pipeline.
                fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask, None);
            }
            // Embedded fixed-point (no fft-rustfft): keep the
            // basis-precompute + dot-product path — no 192k FFT
            // available on Xtensa.
            #[cfg(not(feature = "fft-rustfft"))]
            fill_symbol_spectra_into(
                cs,
                audio,
                cand.freq_hz,
                cand.dt_sec,
                mask,
                basis_re,
                basis_im,
            );
        },
        None,
        DecodeStrictness::Normal,
    )
}

/// Common body of `process_candidates` / `process_candidates_into`
/// / `process_candidates_into_with_cs_scratch` — the BP staircase
/// logic is identical between them; only the per-candidate
/// `fill_symbol_spectra(_into)` call differs (heap-allocated basis vs
/// caller-provided). `cs_scratch` is the per-symbol-spectra working
/// buffer that hot loops (BP / LLR / sync_quality) read from — see
/// [`process_candidates_into_with_cs_scratch`] for the rationale.
/// Per-candidate driver — dt is already parabolically refined by
/// coarse_sync. AP-aware: pass `ap_hint = None` for the legacy
/// non-AP behaviour (bit-identical to the pre-0.6.1 shape).
#[allow(clippy::too_many_arguments)]
fn process_candidates_with_ap<S: AudioSample, F>(
    _audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
    mut fill: F,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult>
where
    F: FnMut(&mut [[Cmplx<f32>; 8]; 79], &SyncCandidate, SymMask),
{
    // dt is already parabolically refined by coarse_sync; no grid here.

    let mut results: Vec<DecodeResult> = Vec::new();
    // BP scratch pool — instantiated once and reused across all
    // candidates × all 5 BP calls per candidate. Eliminates the
    // ~12 KB-per-call `tlsf_malloc` traffic that dominated stage-3
    // non-DFT cost on Core2 (~50–100 ms / qso). See
    // `mfsk_core::fec::ldpc::bp::BpScratch`.
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    for (cand, cs_box, _q_block0) in cands {
        // Stage cs into the caller's scratch (typically internal DRAM
        // on Xtensa) so the LLR / BP / sync_quality hot loops below
        // read from fast memory. The PSRAM-resident `cs_box` is
        // dropped immediately after the copy. Cost: ~60 µs (5 KB at
        // ~80 MB/s OCT PSRAM read on S3) vs many-hundred-µs gains in
        // BP iter loop.
        *cs_scratch = *cs_box;
        drop(cs_box);
        // Two-step fill: sync blocks first, gate by full sync_quality,
        // then fill data symbols only for survivors. Saves the 58 ×
        // 8 = 464 data-symbol DFTs on every candidate that fails the
        // q gate (typically half of `max_cand`). `SyncBlocks12`
        // (instead of `SyncOnly`) skips re-filling block 0 — Pass 2
        // already populated it via `symbol_spectra_direct_into` on
        // `SyncBlock0`, and that data survives in `cs_scratch` here.
        // Saves an additional 56 DFTs / candidate.
        fill(cs_scratch, &cand, SymMask::SyncBlocks12);
        let q = sync_quality(cs_scratch);
        if q <= q_thresh {
            continue;
        }
        fill(cs_scratch, &cand, SymMask::DataOnly);
        if let Some(r) = process_one_candidate_inner(
            cs_scratch,
            &cand,
            cand.dt_sec,
            q,
            depth,
            bp_max_iter,
            &mut bp_scratch,
            &results,
            ap_hint,
            strictness,
            0.0,
        ) {
            results.push(r);
        }
    }

    results
}

/// WSJT-X-faithful nharderrors gate (ft8b.f90:422). High hard-error
/// CRC-pass decodes are the dominant phantom source on busy bands;
/// reject above this and fall through to the next BP variant.
const WSJTX_NHARDERRORS_MAX: u32 = 36;

/// Per-candidate decode core — runs the LLR-staircase, OSD fallback,
/// and AP iaptype loop on a *fully-filled* `cs_scratch`. Shared
/// between the embedded `process_candidates_with` driver (above) and
/// the host `process_candidate` redirect (decode.rs, post-0.6.1).
/// Returns `Some(DecodeResult)` on first success (Step 1, 2, 3, or 4
/// in order), `None` on full failure.
///
/// Caller responsibilities:
/// - `cs_scratch` filled with all 79 symbols × 8 tones (data + sync).
/// - `q = sync_quality(cs_scratch)` already computed and gated.
/// - `refined_dt` carries the parabolically-refined `dt_sec`.
/// - `known` is the dedup list (in-progress + prior-pass results).
/// - `ap_hint` is `None` for AP-off (embedded default); `Some(_)` to
///   activate Step 4 AP iaptype loop. `#[cfg(feature = "fft-rustfft")]`
///   only — embedded fixed-point builds always pass `None`.
/// - `strictness` controls the per-iaptype `nharderrors` cap; ignored
///   when `ap_hint` is `None`.
/// - `sync_cv` is the Costas-array power CV (host computes it for QSB
///   gain attenuation; embedded passes 0.0).
#[allow(clippy::too_many_arguments)]
#[allow(unused_variables)] // ap_hint / strictness only used under #[cfg(feature = "fft-rustfft")]
pub(super) fn process_one_candidate_inner(
    cs_scratch: &[[Cmplx<f32>; 8]; 79],
    cand: &SyncCandidate,
    refined_dt: f32,
    q: u32,
    depth: DecodeDepth,
    bp_max_iter: u32,
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<
        crate::fec::ldpc::params::Ldpc174_91Params,
        LlrT,
    >,
    known: &[DecodeResult],
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    sync_cv: f32,
) -> Option<DecodeResult> {
    // ── Staircase: cheap → deeper → OSD ─────────────────────────
    //
    // 1) Bp(llra) on the fast nsym=1 LLR. Most candidates that
    //    decode at all decode here; the rest fall through.
    // 2) Full compute_llr (nsym=1+2+3) → Bp on all 4 variants
    //    (a/b/c/d).
    // 3) OSD-1 / OSD-3 fallback gated on sync_quality.
    //
    // `BpAll` and `BpAllOsd` enable the deeper stages; plain
    // `Bp` stops after step 1.
    let mut accepted: Option<(crate::fec::ldpc::bp::BpResult, u8)> = None;

    // Step 1: fast llra. The LLR / BP scalar is selected at compile
    // time via `fixed-point` (Q3i8) or default (f32) — see the
    // `LlrT` definition above. Both go through the *same* generic
    // NMS implementation, bit-identical AWGN behaviour by design.
    let llr_a_fast: super::llr::LlrSet<LlrT> = super::llr::compute_llr_fast(cs_scratch);
    let bp_step1 =
        bp_step_select::<LlrT>(bp_scratch, &llr_a_fast.llra, bp_max_iter, Some(check_crc14));
    if let Some(bp) = bp_step1
        && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
    {
        accepted = Some((bp, 0));
    }

    // Step 2: deeper-LLR variants. Lazy + LLR-shared with Step 1.
    //
    // **Variant a is skipped** — Step 1 already ran BP on the
    // identical input (`compute_llr_fast` and `compute_llr`
    // produce bit-identical `llra`, since nsym=1 work doesn't
    // depend on `max_nsym`). Re-running it would be guaranteed
    // failure.
    //
    // **Variant d reuses** Step 1's `llr_a_fast.llrd` — same
    // nsym=1 derivation, costs zero LLR work.
    //
    // **Variants b / c are lazy-computed**: only pay the nsym=2
    // work if d failed, and only pay the heavy nsym=3 work
    // (~80 % of `compute_llr`) if both d and b also failed.
    // Order chosen by ascending compute cost — same number of BP
    // calls as the old variant loop in the worst case, far fewer
    // in the typical case where any earlier variant decodes.
    if accepted.is_none() && matches!(depth, DecodeDepth::BpAll | DecodeDepth::BpAllOsd) {
        // Variant d: free reuse of Step 1's llrd.
        let bp_d =
            bp_step_select::<LlrT>(bp_scratch, &llr_a_fast.llrd, bp_max_iter, Some(check_crc14));
        if let Some(bp) = bp_d
            && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
        {
            accepted = Some((bp, 3));
        }
        // Variant b: lazy nsym=2 only.
        if accepted.is_none() {
            let llrb_arr: [LlrT; LDPC_N] = super::llr::compute_llr_partial::<LlrT>(cs_scratch, 2);
            let bp_b =
                bp_step_select::<LlrT>(bp_scratch, &llrb_arr, bp_max_iter, Some(check_crc14));
            if let Some(bp) = bp_b
                && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
            {
                accepted = Some((bp, 1));
            }
        }
        // Variant c: lazy nsym=3 (the expensive one).
        if accepted.is_none() {
            let llrc_arr: [LlrT; LDPC_N] = super::llr::compute_llr_partial::<LlrT>(cs_scratch, 3);
            let bp_c =
                bp_step_select::<LlrT>(bp_scratch, &llrc_arr, bp_max_iter, Some(check_crc14));
            if let Some(bp) = bp_c
                && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
            {
                accepted = Some((bp, 2));
            }
        }
    }

    // Step 3: OSD fallback (sync_quality gated; only for BpAllOsd).
    // OSD operates on `&[f32]` directly — independent of the
    // `LlrT` choice — so we compute a fresh f32 LLR bundle here.
    // Only fires when Steps 1+2 BP failed and `q >= 12`, so the
    // extra compute_llr is cheap relative to the OSD work itself.
    if accepted.is_none() && matches!(depth, DecodeDepth::BpAllOsd) && q >= 12 {
        let llr_full_f32: super::llr::LlrSet<f32> = super::llr::compute_llr(cs_scratch);
        // Pass-ID space (post-0.6.1): BP variants 0..3, AP iaptypes
        // 5..12 (mirroring WSJT-X ipass 5..12), host OSD-Deep 13,
        // embedded OSD a/b/c/d 14/15/16/17. The +10 shift on OSD
        // frees 5..12 for the AP loop being plumbed into this same
        // function in 0.6.1 (decode_block_with_ap path).
        for (llr, pid) in [
            (&llr_full_f32.llra, 14u8),
            (&llr_full_f32.llrb, 15),
            (&llr_full_f32.llrc, 16),
            (&llr_full_f32.llrd, 17),
        ] {
            // OSD depth dispatch — mfsk-core terminology.
            //
            // `osd_decode_deep(_, N, _)` tries MRB error patterns of
            // weight 0..=N (inclusive). `osd_decode` is a thin wrapper for
            // `osd_decode_generic(_, 2, _, _)` (ndeep=2). The
            // ndeep=2/3 split has been the design since f118a64.
            //
            // **WSJT-X-faithfulness deviation (see #63).** ft8b.f90
            // always calls `osd174_91(..., norder=2, ...)` for FT8,
            // which dispatches to `nord=1 + npre1=1` — order-1 MRB
            // search **plus** a precoding rule (test error patterns
            // derived from G matrix columns; osd174_91.f90:230-289).
            // mfsk-core has no precoding implementation; the bumped
            // ndeep here was chosen as a simpler stand-in. Whether
            // pattern-count compensation actually covers the
            // structurally-distinct patterns WSJT-X's precoding
            // generates is an open question. #63 tracks restoring
            // strict WSJT-X behaviour (nord=1 + npre1).
            let osd = if q >= 18 {
                osd_decode_deep(llr, 3, Some(check_crc14))
            } else {
                osd_decode(llr)
            };
            if let Some(osd) = osd {
                // Mirror the BP-variant `nharderrors > 36` cycle
                // gate (ft8b.f90:422) on the OSD path. WSJT-X's
                // OSD itself returns CRC-pass codewords with
                // negated `nhardmin` on CRC fail (osd174_91:290),
                // but we apply the same upper bound to the
                // hard-error count so high-error CRC-luck
                // codewords don't pass through OSD either.
                if osd.hard_errors > WSJTX_NHARDERRORS_MAX {
                    continue;
                }
                let bp = crate::fec::ldpc::bp::BpResult {
                    message77: osd.message77,
                    info: osd.info,
                    codeword: vec![0u8; LDPC_N],
                    hard_errors: osd.hard_errors,
                    iterations: 0,
                };
                accepted = Some((bp, pid));
                break;
            }
        }
    }

    // Step 4: AP iaptype loop (host f32 only; gated on fft-rustfft).
    // Mirrors WSJT-X ft8b.f90 ipass 5..12 — runs only if Steps 1-3 all
    // failed and the caller supplied an `ap_hint`. Embedded fixed-point
    // builds always pass `ap_hint = None` so this entire block
    // constant-folds away.
    //
    // Iaptype priority (deepest-first, mirroring decode.rs:634-684):
    //   9/10/11 (call1+call2+RRR/RR73/73 → full 77-bit lock)
    //   7 (CQ + call2)
    //   8 (call1+call2 → ~61 bits)
    //   6 (ap as-supplied → ~33 bits)
    //   5 (mycall only → ~32 bits, WSJT-X iaptype 2)
    //   12 (blind CQ → always tried last, WSJT-X iaptype 1)
    #[cfg(feature = "fft-rustfft")]
    if accepted.is_none()
        && let Some(ap) = ap_hint
    {
        let llr_full_f32: super::llr::LlrSet<f32> = super::llr::compute_llr(cs_scratch);
        let apmag = llr_full_f32
            .llra
            .iter()
            .map(|v| v.abs())
            .fold(0.0f32, f32::max)
            * 1.01;
        let llr_variants: [&[f32; LDPC_N]; 4] = [
            &llr_full_f32.llra,
            &llr_full_f32.llrb,
            &llr_full_f32.llrc,
            &llr_full_f32.llrd,
        ];

        let mut ap_passes: alloc::vec::Vec<(ApHint, u8)> = alloc::vec::Vec::new();
        if ap.has_info() {
            if ap.call1.is_some() && ap.call2.is_some() {
                for (rpt, pid) in [("RRR", 9u8), ("RR73", 10), ("73", 11)] {
                    let ap_full = ap.clone().with_report(rpt);
                    ap_passes.push((ap_full, pid));
                }
            }
            if ap.call2.is_some() && ap.call1.is_none() {
                let ap7 = ap.clone().with_call1("CQ");
                ap_passes.push((ap7, 7));
            }
            if ap.call1.is_some() && ap.call2.is_some() {
                ap_passes.push((ap.clone(), 8));
            }
            ap_passes.push((ap.clone(), 6));
            if ap.call1.is_some() {
                let mut ap5 = ApHint::new();
                if let Some(ref c1) = ap.call1 {
                    ap5 = ap5.with_call1(c1);
                }
                ap_passes.push((ap5, 5));
            }
        }
        // Pass 12: blind-CQ (WSJT-X iaptype 1) — always tried.
        ap_passes.push((ApHint::new().with_call1("CQ"), 12));

        'ap_outer: for (ap_cfg, pass_id) in &ap_passes {
            let (ap_mask, ap_llr_override) = ap_cfg.build_ap(apmag);
            let locked_bits = ap_mask.iter().filter(|&&m| m).count();
            let max_errors: u32 = strictness.ap_max_errors(locked_bits);

            for &base_llr in &llr_variants {
                let mut llr_ap = *base_llr;
                for i in 0..LDPC_N {
                    if ap_mask[i] {
                        llr_ap[i] = ap_llr_override[i];
                    }
                }
                // Inline AP-result validator: hard-error gate, unpack,
                // plausibility, locked-call substring check.
                let validate = |msg77: [u8; 77], hard_errors: u32| -> bool {
                    if hard_errors >= max_errors {
                        return false;
                    }
                    let Some(text) = unpack77(&msg77) else {
                        return false;
                    };
                    if !crate::msg::wsjt77::is_plausible_message(&text) {
                        return false;
                    }
                    let upper = text.to_uppercase();
                    if let Some(ref c1) = ap_cfg.call1
                        && !upper.contains(&c1.to_uppercase())
                    {
                        return false;
                    }
                    if let Some(ref c2) = ap_cfg.call2
                        && !upper.contains(&c2.to_uppercase())
                    {
                        return false;
                    }
                    true
                };

                // AP + BP
                if let Some(bp) = crate::fec::ldpc::bp::bp_decode(
                    &llr_ap,
                    Some(&ap_mask),
                    bp_max_iter,
                    Some(check_crc14),
                ) && validate(bp.message77, bp.hard_errors)
                {
                    accepted = Some((bp, *pass_id));
                    break 'ap_outer;
                }
                // AP + OSD-Deep fallback (BpAllOsd only)
                if depth == DecodeDepth::BpAllOsd
                    && let Some(osd) = osd_decode_deep(&llr_ap, 2, Some(check_crc14))
                    && validate(osd.message77, osd.hard_errors)
                {
                    let bp = crate::fec::ldpc::bp::BpResult {
                        message77: osd.message77,
                        info: osd.info,
                        codeword: vec![0u8; LDPC_N],
                        hard_errors: osd.hard_errors,
                        iterations: 0,
                    };
                    accepted = Some((bp, *pass_id));
                    break 'ap_outer;
                }
            }
        }
    }

    let (bp, pass_id) = accepted?;
    let text = unpack77(&bp.message77)?;
    // Plausibility filter — reject CRC-passing-but-garbage
    // messages. With max_cand=200 × 4 LLR variants × OSD,
    // CRC-14's 1/16384 false-positive rate produces ~1-2 random
    // strings per slot. Same filter the host wide-band path
    // uses (`decode_frame::process_candidate`).
    if !crate::msg::wsjt77::is_plausible_message(&text) {
        return None;
    }
    if known.iter().any(|r| r.message77 == bp.message77) {
        return None;
    }
    let itone = message_to_tones(&bp.message77);
    let snr_db = super::llr::compute_snr_db(cs_scratch, &itone);
    Some(DecodeResult {
        message77: bp.message77,
        freq_hz: cand.freq_hz,
        dt_sec: refined_dt,
        hard_errors: bp.hard_errors,
        sync_score: cand.score,
        pass: pass_id,
        sync_cv,
        snr_db,
    })
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{MessageCodec, MessageFields};
    use crate::ft8::wave_gen::{message_to_tones, tones_to_f32};
    use crate::msg::Wsjt77Message;

    fn pack_cq() -> [u8; 77] {
        let bits = Wsjt77Message
            .pack(&MessageFields {
                call1: Some("CQ".into()),
                call2: Some("JA1ABC".into()),
                grid: Some("PM95".into()),
                ..Default::default()
            })
            .unwrap();
        let mut out = [0u8; 77];
        out.copy_from_slice(&bits);
        out
    }

    fn synth_clean(msg77: &[u8; 77], freq_hz: f32) -> Vec<i16> {
        let itone = message_to_tones(msg77);
        let pcm = tones_to_f32(&itone, freq_hz, 0.5);
        let mut slot = vec![0.0f32; NMAX];
        let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize;
        let n = pcm.len().min(NMAX - start);
        slot[start..start + n].copy_from_slice(&pcm[..n]);
        slot.iter()
            .map(|&s| (s * 25_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect()
    }

    /// Reproduce the dual_core::coarse_sync_split_with_allsum
    /// pattern on host: build a full-band allsum, slice it for the
    /// head [100, mid] and tail [mid, freq_max] sub-bands, run
    /// coarse_sync_with_allsum on each, compare against running
    /// coarse_sync on each sub-band without allsum. Synthesises 5
    /// signals across the band so both halves have real candidates
    /// (mirrors the qso3 busy-band failure case).
    /// Verify that filling the allsum **column-by-column** (mirroring
    /// what `embedded-poc/m5stack-core2::stage1_inc::update_one_half`
    /// does as new spectrogram rows arrive) produces the same buffer
    /// as the one-shot `precompute_coarse_allsum`.
    /// **Test mirrors only the FT8 contiguous (16-bin) path.**
    #[test]
    fn fill_coarse_allsum_column_by_column_matches_oneshot() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax) = (1550.0_f32, 3000.0_f32);
        let oneshot = precompute_coarse_allsum(&spec, fmin, fmax);

        let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
        let tone_step_bins = TONE_SPACING_HZ / df;
        let ia = (fmin / df).round() as usize;
        let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
        let nh1 = spec.n_freq;
        let ib_unbounded = (fmax / df).round() as usize;
        let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
        let n_freq = ib - ia + 1;
        let n_time = spec.n_time;

        // Column-by-column: simulate incremental fill, walking m
        // outer, fi inner. NFFT=3840 → 7-tone every-other gather per
        // carrier (k=0..NTONES-1; tone 7 is data-only, never a Costas
        // position — matches WSJT-X sync8.f90:66 and stage1_inc's
        // `update_one_half`).
        let mut col: Vec<f32> = vec![0.0; n_freq * n_time];
        for m in 0..n_time {
            for fi in 0..n_freq {
                let i_carrier = ia + fi;
                let mut s = 0.0_f32;
                for k in 0..(NTONES - 1) {
                    let bin = (i_carrier + 2 * k).min(nh1 - 1);
                    s += spec.power_acc(bin, m);
                }
                col[fi * n_time + m] = s;
            }
        }

        for fi in 0..n_freq {
            for m in 0..n_time {
                let a = oneshot[fi * n_time + m];
                let b = col[fi * n_time + m];
                assert!(
                    (a - b).abs() < 1e-3,
                    "fi={fi} m={m}: oneshot={a} col-by-col={b}"
                );
            }
        }
    }

    /// Same as `coarse_sync_with_allsum_matches_internal` but for a
    /// non-default sub-band (tail of [1550, 3000]) — isolates whether
    /// the coarse_sync_with_allsum path itself works for arbitrary
    /// freq ranges, independent of slicing logic.
    #[test]
    fn coarse_sync_with_allsum_tail_band_only() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 2000.0); // signal in tail band
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax, smin, ncand) = (1550.0_f32, 3000.0_f32, 1.0_f32, 30usize);

        let a = coarse_sync(&spec, fmin, fmax, smin, ncand);
        let allsum = precompute_coarse_allsum(&spec, fmin, fmax);
        let b = coarse_sync_with_allsum(&spec, fmin, fmax, smin, ncand, &allsum);

        assert_eq!(
            a.len(),
            b.len(),
            "tail band candidate count A={} B={}",
            a.len(),
            b.len()
        );
        for (ai, bi) in a.iter().zip(b.iter()) {
            assert!(
                (ai.freq_hz - bi.freq_hz).abs() < 1e-3 && (ai.score - bi.score).abs() < 1e-4,
                "tail-band mismatch: A={ai:?} B={bi:?}"
            );
        }
    }

    /// Reproduce the dual_core::coarse_sync_split_with_allsum
    /// pattern using per-half allsums (the API supports any sub-band
    /// directly). Synthesises 5 signals across the band so both
    /// halves have real candidates (mirrors qso3 busy band).
    /// **Per-half precompute avoids sliding-window f32 drift across
    /// the full band that breaks slice-based approaches.**
    #[test]
    fn coarse_sync_split_with_allsum_per_half_busy_band() {
        let msg = pack_cq();
        let freqs = [400.0_f32, 1100.0, 1700.0, 2200.0, 2700.0];
        let mut mix = vec![0.0f32; NMAX];
        for (i, &f) in freqs.iter().enumerate() {
            let itone = message_to_tones(&msg);
            let pcm = tones_to_f32(&itone, f, 0.5);
            let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize + i * 100;
            let n = pcm.len().min(NMAX - start);
            for k in 0..n {
                mix[start + k] += pcm[k];
            }
        }
        let audio: Vec<i16> = mix
            .iter()
            .map(|&s| (s * 5_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax, smin, ncand) = (100.0_f32, 3_000.0_f32, 1.0_f32, 30usize);
        let mid = 0.5 * (fmin + fmax);

        // Path A: baseline coarse_sync per half.
        let head_a = coarse_sync(&spec, fmin, mid, smin, ncand);
        let tail_a = coarse_sync(&spec, mid, fmax, smin, ncand);

        // Path B: per-half precompute_coarse_allsum.
        let allsum_head = precompute_coarse_allsum(&spec, fmin, mid);
        let allsum_tail = precompute_coarse_allsum(&spec, mid, fmax);
        let head_b = coarse_sync_with_allsum(&spec, fmin, mid, smin, ncand, &allsum_head);
        let tail_b = coarse_sync_with_allsum(&spec, mid, fmax, smin, ncand, &allsum_tail);

        assert_eq!(head_a.len(), head_b.len());
        assert_eq!(tail_a.len(), tail_b.len());
        for (a, b) in head_a.iter().zip(head_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3 && (a.score - b.score).abs() < 1e-4,
                "head per-half mismatch: A={a:?} B={b:?}"
            );
        }
        for (a, b) in tail_a.iter().zip(tail_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3 && (a.score - b.score).abs() < 1e-4,
                "tail per-half mismatch: A={a:?} B={b:?}"
            );
        }
    }

    /// **Documents a known limitation, not a regression**: building a
    /// full-band allsum and slicing it for sub-band consumption
    /// Full-band allsum sliced for each half MUST match per-half
    /// precompute. With NFFT=3840 the 8-bin every-other gather (no
    /// sliding window) is f32-stable, so slicing a full-band allsum
    /// reproduces the per-half result bit-for-bit. The previous
    /// `#[should_panic]` documented an f32 sliding-window drift that
    /// existed at NFFT=4096 + 16-bin sliding sum and is now gone.
    #[test]
    fn coarse_sync_split_with_allsum_busy_band() {
        let msg = pack_cq();
        // 5 signals across [100, 3000] Hz band — 2 in head [100, 1550]
        // and 3 in tail [1550, 3000].
        let freqs = [400.0_f32, 1100.0, 1700.0, 2200.0, 2700.0];
        let mut mix = vec![0.0f32; NMAX];
        for (i, &f) in freqs.iter().enumerate() {
            let itone = message_to_tones(&msg);
            let pcm = tones_to_f32(&itone, f, 0.5);
            let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize + i * 100;
            let n = pcm.len().min(NMAX - start);
            for k in 0..n {
                mix[start + k] += pcm[k];
            }
        }
        let audio: Vec<i16> = mix
            .iter()
            .map(|&s| (s * 5_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let spec = compute_spectrogram(&audio, 3_000.0);

        let (fmin, fmax, smin, ncand) = (100.0_f32, 3_000.0_f32, 1.0_f32, 30usize);
        let mid = 0.5 * (fmin + fmax);
        let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;

        // Path A: baseline — coarse_sync per half.
        let head_a = coarse_sync(&spec, fmin, mid, smin, ncand);
        let tail_a = coarse_sync(&spec, mid, fmax, smin, ncand);

        // Path B: precompute full-band allsum, slice, run coarse_sync_with_allsum.
        let allsum_full = precompute_coarse_allsum(&spec, fmin, fmax);
        let allsum_ia = (fmin / df).round() as usize;
        let head_len = coarse_allsum_len(spec.n_freq, spec.n_time, fmin, mid);
        let tail_len = coarse_allsum_len(spec.n_freq, spec.n_time, mid, fmax);
        let ia_head = (fmin / df).round() as usize;
        let ia_tail = (mid / df).round() as usize;
        let head_off = (ia_head - allsum_ia) * spec.n_time;
        let tail_off = (ia_tail - allsum_ia) * spec.n_time;
        let head_slice = &allsum_full[head_off..head_off + head_len];
        let tail_slice = &allsum_full[tail_off..tail_off + tail_len];
        let head_b = coarse_sync_with_allsum(&spec, fmin, mid, smin, ncand, head_slice);
        let tail_b = coarse_sync_with_allsum(&spec, mid, fmax, smin, ncand, tail_slice);

        assert_eq!(
            head_a.len(),
            head_b.len(),
            "head candidate count differs: A={} B={}",
            head_a.len(),
            head_b.len()
        );
        assert_eq!(
            tail_a.len(),
            tail_b.len(),
            "tail candidate count differs: A={} B={}",
            tail_a.len(),
            tail_b.len()
        );
        for (a, b) in head_a.iter().zip(head_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3
                    && (a.dt_sec - b.dt_sec).abs() < 1e-6
                    && (a.score - b.score).abs() < 1e-4,
                "head mismatch: A={a:?} B={b:?}"
            );
        }
        for (a, b) in tail_a.iter().zip(tail_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3
                    && (a.dt_sec - b.dt_sec).abs() < 1e-6
                    && (a.score - b.score).abs() < 1e-4,
                "tail mismatch: A={a:?} B={b:?}"
            );
        }
    }

    #[test]
    fn coarse_sync_with_allsum_matches_internal() {
        // Phase-E2 sister API equivalence: compute the same allsum that
        // the internal precompute would produce, feed it via
        // `coarse_sync_with_allsum`, and verify identical candidate output.
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax, smin, ncand) = (100.0_f32, 3000.0_f32, 1.0_f32, 30usize);
        let internal = coarse_sync(&spec, fmin, fmax, smin, ncand);
        let allsum = precompute_coarse_allsum(&spec, fmin, fmax);
        assert_eq!(
            allsum.len(),
            coarse_allsum_len(spec.n_freq, spec.n_time, fmin, fmax),
            "allsum length must match coarse_allsum_len"
        );
        let external = coarse_sync_with_allsum(&spec, fmin, fmax, smin, ncand, &allsum);
        assert_eq!(internal.len(), external.len(), "candidate count differs");
        for (a, b) in internal.iter().zip(external.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3
                    && (a.dt_sec - b.dt_sec).abs() < 1e-6
                    && (a.score - b.score).abs() < 1e-4,
                "candidate mismatch: internal={a:?} external={b:?}"
            );
        }
    }

    #[test]
    fn roundtrip_clean_signal() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let results = decode_block(&audio, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 30);
        assert!(
            results.iter().any(|r| r.message77 == msg),
            "decode_block should recover clean CQ; got {} results",
            results.len()
        );
    }

    /// Fill a clean CQ signal with `Sc = Q14i16` cs storage and
    /// verify the autogain produces non-saturated output that the
    /// generic `sync_quality_block0` recognises as a Costas hit.
    /// Exercises the Phase 2.6 fill / autogain path end-to-end.
    #[test]
    fn fill_q14i16_autogain_recovers_costas() {
        use crate::core::scalar::Q14i16;
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);

        let mut cs_q14: alloc::boxed::Box<[[Cmplx<Q14i16>; 8]; 79]> =
            alloc::vec![[Cmplx::<Q14i16>::default(); 8]; 79]
                .try_into()
                .unwrap();
        // Use the host f32 fill on the f32 path; under fixed-point
        // the i16 fill (with autogain) is exercised. Both must yield
        // a Costas-perfect block 0 sync_quality.
        #[cfg(not(feature = "fixed-point"))]
        fill_symbol_spectra_generic::<Q14i16, i16>(
            &mut cs_q14,
            &audio,
            1500.0,
            0.0,
            SymMask::SyncBlock0,
        );
        #[cfg(feature = "fixed-point")]
        fill_symbol_spectra_generic::<Q14i16, i16>(
            &mut cs_q14,
            &audio,
            1500.0,
            0.0,
            SymMask::SyncBlock0,
        );

        // Spot-check: non-zero entries (autogain didn't kill the signal).
        let nonzero = cs_q14.iter().flatten().any(|c| c.re.0 != 0 || c.im.0 != 0);
        assert!(nonzero, "Q14 autogain produced all-zero cs");

        // sync_quality_block0 generic with S=Q14i16 should recognise
        // the Costas pattern at full strength on a clean signal.
        let q = sync_quality_block0(&cs_q14);
        assert_eq!(q, 7, "expected perfect Costas block-0 hit, got q={q}");
    }
}
