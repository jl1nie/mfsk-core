//! Power-spectrogram stage: `Spectrogram` container + the two
//! `compute_spectrogram` backends (host `fft-rustfft` and embedded
//! `fixed-point`).
//!
//! ε.2 of the `docs/CLEANUP_2026_05.md` `decode_block` split. The
//! parent (`decode_block.rs`) re-exports `Spectrogram`, `SpecCell`,
//! and `compute_spectrogram` so external callers
//! (`embedded-shared::pipeline::Stage1Output`, the m5stack apps,
//! integration tests) see the same `mfsk_core::ft8::decode_block::*`
//! paths as before the split. The remaining ex-private items
//! (`CoarseAcc`, `Spectrogram::power_acc`) are surfaced as
//! `pub(super)` for the coarse-sync code that still lives in the
//! parent — they'll move to siblings of this file in ε.3.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::params::NTONES;
// the 12 kHz constants: `compute_spectrogram_f32_timed` and the fixed-point gain are still in them; the builders take
// everything else from their `Ft8Rate`
#[cfg(any(feature = "fixed-point", feature = "internal-testing"))]
use super::super::params::NMAX;
#[cfg(feature = "internal-testing")]
use super::super::params::NSPS;
#[cfg(any(feature = "fixed-point", feature = "internal-testing"))]
use super::types::NFFT_SPEC;
#[cfg(feature = "internal-testing")]
use super::types::NSTEP;
#[cfg(any(not(feature = "fixed-point"), feature = "internal-testing"))]
use super::types::SAMPLE_RATE_HZ;
use super::types::{AudioSample, Ft8Rate, TONE_SPACING_HZ};
use crate::engine::source::{PcmSource, SliceSource, Window};

// Needed by the f32 builder below, which exists in a `fixed-point`
// build too — `compute_spectrogram_f32_timed` is how such a build
// measures what it would otherwise have paid.
#[cfg(any(not(feature = "fixed-point"), feature = "internal-testing"))]
use crate::engine::fft::default_planner;

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
pub(super) const FP_SPEC_SHIFT: u32 = 12;

/// Power spectrogram emitted by [`compute_spectrogram`] and consumed
/// by [`super::coarse_sync`] / [`super::coarse_sync_with_allsum`].
/// The struct is public so embedded consumers
/// (`embedded-shared::dual_core` and the M5Stack apps) can hold a
/// [`Spectrogram`] across the stage-1/stage-2 boundary; layout
/// fields are publicly readable.
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

impl core::fmt::Debug for Spectrogram {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Spectrogram")
            .field("n_freq", &self.n_freq)
            .field("n_time", &self.n_time)
            .finish_non_exhaustive()
    }
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

/// Rows of bins a spectrogram needs for carriers up to `max_freq_hz`: the carrier band plus the room the
/// top tone of a carrier at its upper edge reads (`max_freq_hz + NTONES * tone spacing`).
pub(super) fn spectrogram_n_freq(max_freq_hz: f32, rate: Ft8Rate) -> usize {
    // 3.125 Hz at every rate (`NFFT` is two symbols): the bins of a frequency do not move with the rate
    let df = rate.hz() as f32 / rate.nfft() as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = rate.nfft() / 2;
    ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full)
}

/// coarse_sync inner-loop accumulator. f32 — overlaps on the FPU with
/// integer index arithmetic on the ALU on ESP32 LX6/LX7; the integer
/// `i32` variant (formerly `fixed-point-coarse-i32`) serialises both
/// onto the ALU and costs ~25 % stage-2 wall-clock on Core2, so it
/// was retired. If an FPU-less target (RP2040, Cortex-M0+, Hazard3)
/// is added later, reintroduce the i32 alias on a per-target cfg.
pub(super) type CoarseAcc = f32;

impl Spectrogram {
    /// Power-cell read in `CoarseAcc` (i32 under fixed-point, f32 otherwise).
    /// Used by `coarse_sync` to keep the precompute + score loop integer-pure
    /// on the embedded path — no per-cell u16→f32 promotion in the hot loop.
    /// Under host f32 the cast collapses to a no-op.
    #[inline]
    pub(super) fn power_acc(&self, freq_bin: usize, time_idx: usize) -> CoarseAcc {
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
/// The f32 spectrogram build, timed rather than used — available in
/// **every** build so a `fixed-point` binary can measure what it would
/// have cost.
///
/// `fixed-point` exists to halve this stage's PSRAM bandwidth, and on
/// a CoreS3 the stage is 51 % of an FT8 slot
/// (`docs/historical/FT4_BENCHMARK_LOG.md` §35.1). Whether the halving actually
/// buys anything had never been measured either way: the two builders
/// are `#[cfg]` alternatives, so no binary ever contained both, and
/// the feature cannot simply be turned off — `stage1_inc` needs the
/// `Fft16` planner that only exists under it.
///
/// Returns `(n_freq, n_time, data)` rather than a [`Spectrogram`],
/// whose `data` is `Vec<SpecCell>` and therefore `Vec<u16>` in the
/// build that most wants to call this. Nothing consumes the result;
/// the point is the wall-clock.
///
/// **Not a pure cell-type comparison.** The two paths differ in their
/// window as well — rectangular here, Hann in the fixed-point builder
/// — and the fixed-point one additionally scans the slot for a peak to
/// pick its shift. What this measures is what each path costs, which
/// is the question, but it is not "u16 versus f32" in isolation.
#[cfg(feature = "internal-testing")]
pub fn compute_spectrogram_f32_timed<S: AudioSample>(
    audio: &[S],
    max_freq_hz: f32,
) -> (usize, usize, Vec<f32>) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;
    let scale = 1.0f32 / 300.0;

    // A real-input FFT, as `sync8.f90`'s `four2a(x,NFFT1,1,-1,0)` is: half the work of the
    // complex FFT of a zero-imaginary signal this ran until #761, the same bins up to rounding.
    let mut planner = default_planner();
    let fft = planner.plan_real_forward(NFFT_SPEC);

    let mut data = vec![0.0f32; n_freq * n_time];
    let mut input = vec![0.0f32; NFFT_SPEC];
    let mut buf = vec![Complex::new(0.0f32, 0.0); NFFT_SPEC / 2 + 1];

    for j in 0..n_time {
        let ia = j * NSTEP;
        // `process` may use `input` as scratch, so it is rewritten whole for every row.
        for (k, c) in input.iter_mut().enumerate() {
            *c = if k < NSPS && ia + k < audio.len() {
                audio[ia + k].to_f32() * scale
            } else {
                0.0
            };
        }
        fft.process(&mut input, &mut buf);
        let row_base = j * n_freq;
        for i in 0..n_freq {
            data[row_base + i] = buf[i].norm_sqr();
        }
    }

    (n_freq, n_time, data)
}

/// FT8 spectrogram builder for both the embedded path and the host
/// `decode_frame*` pipeline (the latter routes coarse-sync through
/// this module after #46 / #48 step B).
#[cfg(not(feature = "fixed-point"))]
pub fn compute_spectrogram<S: AudioSample>(audio: &[S], max_freq_hz: f32) -> Spectrogram {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::Spectrogram);
    compute_spectrogram_bins(audio, 0, spectrogram_n_freq(max_freq_hz, Ft8Rate::Hz12000))
}

/// [`compute_spectrogram`] for the bins `bin_lo..bin_hi` only (`n_freq = bin_hi - bin_lo`; bin `j` of
/// the result is bin `bin_lo + j` of the full one). Every cell is the one the full spectrogram holds: the
/// frame's FFT is the same, only the rows kept differ. This is what lets coarse sync run a band at a
/// time (`coarse_sync_banded`) with the spectrogram's memory cut to the band.
#[cfg(not(feature = "fixed-point"))]
pub(super) fn compute_spectrogram_bins<S: AudioSample>(
    audio: &[S],
    bin_lo: usize,
    bin_hi: usize,
) -> Spectrogram {
    compute_spectrogram_bins_src(
        &mut SliceSource(audio),
        Ft8Rate::Hz12000,
        bin_lo,
        bin_hi,
        (),
    )
}

/// What a band's spectrogram needs from the whole slot before it can be built: nothing in the f32 build.
#[cfg(not(feature = "fixed-point"))]
pub(super) type Gain = ();

/// The slot-wide gain of the spectrogram (the fixed-point build scans the slot for its peak).
#[cfg(not(feature = "fixed-point"))]
pub(super) fn spectrogram_gain<A: PcmSource>(_src: &mut A) -> Gain {}

/// [`compute_spectrogram_bins`] from an [`PcmSource`]: the frames are read forward, one window at a time.
#[cfg(not(feature = "fixed-point"))]
pub(super) fn compute_spectrogram_bins_src<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    bin_lo: usize,
    bin_hi: usize,
    _gain: Gain,
) -> Spectrogram {
    // The frame, the FFT and the step in samples of `rate`; the bins (3.125 Hz) and the rows (0.04 s) are the
    // same at every rate, so what reads the spectrogram does not know the rate.
    let (nsps, nfft, nstep, nmax) = (rate.nsps(), rate.nfft(), rate.nstep(), rate.nmax());
    assert!(
        bin_lo <= bin_hi && bin_hi <= nfft / 2 + 1,
        "spectrogram bins out of range"
    );
    let n_freq = bin_hi - bin_lo;
    let n_time = nmax / nstep - 3;
    // A cell is `|FFT|^2` of a frame of `nsps` samples, so it goes as `nsps^2`: scaling the amplitude by the ratio of the
    // rates makes a cell the same at either rate for the same signal and noise (what reads it is ratios, but
    // `ratio_eps` is absolute). `x 1.0` at 12 kHz: the same bits as before the rate was an argument.
    let scale = 1.0f32 / 300.0 * (SAMPLE_RATE_HZ / rate.hz() as f32);

    // A real-input FFT, as `sync8.f90`'s `four2a(x,NFFT1,1,-1,0)` is: half the work of the
    // complex FFT of a zero-imaginary signal this ran until #761, the same bins up to rounding.
    let mut planner = default_planner();
    let fft = planner.plan_real_forward(nfft);

    // Rectangular window — matches WSJT-X `sync8.f90`. With NFFT=3840,
    // `tone_step_bins = 6.25 / (12000/3840) = 2.0` exactly, so the
    // rectangular sidelobes do not bleed onto adjacent FT8 tones.
    // (The fixed-point path keeps Hann to mitigate the wider sidelobes
    // that come with NFFT=4096's fractional 2.13 bins/tone.)
    let mut data = vec![0.0f32; n_freq * n_time];
    let mut input = vec![0.0f32; nfft];
    let mut buf = vec![Complex::new(0.0f32, 0.0); nfft / 2 + 1];

    let mut win = Window::new(src);
    let mut frame = vec![0i16; nsps];
    for j in 0..n_time {
        let ia = j * nstep;
        win.read_into(ia as i64, &mut frame);
        // `process` may use `input` as scratch, so it is rewritten whole for every row.
        for (k, c) in input.iter_mut().enumerate() {
            *c = if k < nsps {
                frame[k] as f32 * scale
            } else {
                0.0
            };
        }
        fft.process(&mut input, &mut buf);
        // Column-major write — `data[j * n_freq + i]` keeps each
        // time slice contiguous in memory (good PSRAM locality
        // for downstream coarse_sync).
        let row_base = j * n_freq;
        for i in bin_lo..bin_hi {
            data[row_base + (i - bin_lo)] = buf[i].norm_sqr();
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
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::Spectrogram);
    compute_spectrogram_bins(audio, 0, spectrogram_n_freq(max_freq_hz, Ft8Rate::Hz12000))
}

/// [`compute_spectrogram`] for the bins `lo..hi` only (`n_freq = hi - lo`; bin `j` of the result is bin
/// `lo + j` of the full one). Every cell is the one the full spectrogram holds: the auto-gain `shift` is
/// taken from the whole slot as before, the frame's FFT is the same, only the rows kept differ. This is
/// what lets coarse sync run a band at a time (`coarse_sync_banded`).
#[cfg(feature = "fixed-point")]
pub(super) fn compute_spectrogram_bins<S: AudioSample>(
    audio: &[S],
    lo: usize,
    hi: usize,
) -> Spectrogram {
    let mut src = SliceSource(audio);
    let gain = spectrogram_gain(&mut src);
    compute_spectrogram_bins_src(&mut src, Ft8Rate::Hz12000, lo, hi, gain)
}

/// The slot-wide gain of the fixed-point spectrogram: the left shift that brings the slot's peak to
/// `2 x NFFT` (see [`compute_spectrogram_bins_src`]).
#[cfg(feature = "fixed-point")]
pub(super) type Gain = u32;

/// Scan the slot (the first `NMAX` samples) for its peak and return the shift. One pass over the source.
#[cfg(feature = "fixed-point")]
pub(super) fn spectrogram_gain<A: PcmSource>(src: &mut A) -> Gain {
    let mut peak_abs: i32 = 1;
    let n_scan = src.len().min(NMAX);
    let mut block = vec![0i16; 1024];
    let mut at = 0;
    while at < n_scan {
        let n = (n_scan - at).min(block.len());
        src.read(at as i64, &mut block[..n]);
        for &v in &block[..n] {
            peak_abs = peak_abs.max((v as i32).abs());
        }
        at += n;
    }
    gain_for_peak(peak_abs)
}

/// The left shift that brings a slot peak of `peak_abs` to `2 x NFFT` (at most 8). The peak is the one thing
/// the gain needs from the slot, so a capture that tracks it can hand it over instead of a read.
#[cfg(feature = "fixed-point")]
pub(super) fn gain_for_peak(peak_abs: i32) -> Gain {
    let target_peak: i32 = (NFFT_SPEC * 2) as i32;
    let mut shift: u32 = 0;
    while peak_abs.max(1) << shift < target_peak && shift < 8 {
        shift += 1;
    }
    shift
}

/// The periodic Hann window over `n` samples, `sin^2(pi k / n)`, in Q15 (peak 32767 at `n / 2`, 0 at `k = 0`). Symmetric by
/// construction (`w[k] == w[n - k]`: the first half is computed, the second mirrors it), so an `f32` `sin` that rounds
/// differently on another target moves a pair of entries together, never one.
#[cfg(feature = "fixed-point")]
fn hann_q15(n: usize) -> Vec<i16> {
    (0..n)
        .map(|k| {
            let k = k.min(n - k);
            let s = (core::f32::consts::PI * k as f32 / n as f32).sin();
            (s * s * 32767.0 + 0.5) as i16
        })
        .collect()
}

/// [`compute_spectrogram_bins`] from an [`PcmSource`], with the slot's gain given: the frames are read
/// forward, one window at a time.
#[cfg(feature = "fixed-point")]
pub(super) fn compute_spectrogram_bins_src<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    lo: usize,
    hi: usize,
    shift: Gain,
) -> Spectrogram {
    use crate::engine::fft::default_planner_16;

    // The frame, the FFT and the step in samples of `rate` (the 2560-point plan at 8 kHz has the 3840 one's inner kernel and
    // gain `1/256`, so `FP_SPEC_SHIFT` is the same); the bins and rows are the same at either rate.
    let (nsps, nfft, nstep, nmax) = (rate.nsps(), rate.nfft(), rate.nstep(), rate.nmax());
    // A frame of `nsps` samples has a tone `amp x nsps` strong, so 8 kHz frames give cells 4/9 of 12 kHz's at the same
    // amplitude. The input is scaled by 12000 / rate (3/2 at 8 kHz) after the shift, which makes a cell the same at either
    // rate for the same signal (and, at the same noise density, for noise): `FP_SPEC_SHIFT`, `ratio_eps` and the SNR's
    // `cell_scale` carry over, and the u16 cells keep the resolution they have at 12 kHz. 1 / 1 at 12 kHz.
    let (gain_num, gain_den) = rate.input_gain();

    assert!(
        lo <= hi && hi <= nfft / 2 + 1,
        "spectrogram bins out of range"
    );
    let n_freq = hi - lo;
    let n_time = nmax / nstep - 3;

    // The frame's weighting. 12 kHz: rectangular over one symbol (`nsps` samples, zero-padded to the FFT), as it always was
    // and as `sync8.f90:35-37` does. 8 kHz: **a Hann window over two symbols** (`nfft` samples) centred on the same instant,
    // `sin^2` in Q15, **a deliberate divergence from WSJT-X** (see `Ft8Rate::coarse_hann_2t` for why, and for what it costs
    // at 15 candidates). A row's frame starts `nsps / 2` earlier and is `nsps` longer, so its centre is the rectangular
    // frame's (`row x nstep + nsps / 2`): the rows mean the same time as before.
    //
    // Cells against the rectangular frame: a **steady** tone has the same cell (`sum w = nfft / 2 = nsps`), so the gain and
    // `FP_SPEC_SHIFT` carry over; a tone of **one symbol** centred in the window has amplitude `0.5 + 1/pi` = 0.82 of it (power
    // 0.67, -1.7 dB: the window's middle half); noise has 0.75 (-1.25 dB: `sum w^2 = 0.375 nfft` against `nsps`). So a
    // signal's Costas and data cells are about 0.5 dB weaker against the noise than they were, and `xsnr2_db_simple`, which
    // reads them against a noise baseline and is not rate-aware, would read about 0.5 dB low at 8 kHz (not measured, not
    // compensated).
    let hann = rate.coarse_hann_2t();
    let (flen, lead) = if hann { (nfft, nsps / 2) } else { (nsps, 0) };
    let win_q15: Vec<i16> = if hann { hann_q15(nfft) } else { Vec::new() };

    // The pre-scale gain (`shift`, from `spectrogram_gain`): the slot's peak lands at `2 x NFFT`, so that after
    // `log2(NFFT)` stages of /2 the post-FFT peak still has a usable mantissa of ~2. The scan covers the full
    // slot (the ESP32's `stage1_inc` peaks only the first second, since it cannot wait for the tail; the two
    // agree on real recordings: `qso3_busy.wav`'s peak is in its first 12 k samples). The Hann window and its
    // +1 pre-shift were removed when the spectrogram went to NFFT=3840: at integer tone alignment the
    // rectangular window's sidelobes do not leak onto adjacent FT8 tones. **That holds for a tone on the grid, not for
    // a real signal**: at 8 kHz the weighting is back, now as a Hann over two symbols (`docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md`).

    let mut planner = default_planner_16();
    let fft = planner.plan_forward(nfft);

    let mut data: Vec<u16> = vec![0u16; n_freq * n_time];
    let mut buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0i16); nfft];

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

    // **Rectangular window at 12 kHz** — matches `embedded-shared::stage1_inc`
    // verbatim (at 8 kHz the frame is the Hann above, which `stage1_inc` does not build). At NFFT=3840 the FT8 tone spacing falls on integer
    // bins (`tone_step_bins = 6.25 / (12000/3840) = 2.0`), so the
    // rectangular sidelobes don't leak onto adjacent tones; Hann's
    // coherent gain 0.5 (≈ -6 dB SNR penalty) + 2-bin mainlobe spread
    // would be pure cost.
    let mut win = Window::new(src);
    let (mut fa, mut fb) = (vec![0i16; flen], vec![0i16; flen]);
    let mut pack = |buf: &mut [Complex<i16>], ia_a: usize, ia_b: Option<usize>| {
        win.read_into(ia_a as i64 - lead as i64, &mut fa);
        if let Some(ia_b) = ia_b {
            win.read_into(ia_b as i64 - lead as i64, &mut fb);
        }
        let scale = |raw: i16, k: usize| {
            let v = (raw as i32) << shift;
            let v = if hann {
                ((v as i64 * win_q15[k] as i64) >> 15) as i32
            } else {
                v
            };
            // `gain_num == gain_den` at 12 kHz: the sample is as it was
            let v = if gain_num == gain_den {
                v
            } else {
                v * gain_num / gain_den
            };
            v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
        };
        for (k, c) in buf.iter_mut().enumerate() {
            let re = if k < flen { scale(fa[k], k) } else { 0 };
            let im = match ia_b {
                Some(_) if k < flen => scale(fb[k], k),
                _ => 0,
            };
            *c = Complex::new(re, im);
        }
    };

    for jj in 0..n_pairs {
        let j_a = 2 * jj;
        let j_b = j_a + 1;
        pack(&mut buf, j_a * nstep, Some(j_b * nstep));
        fft.process(&mut buf);

        let row_a = j_a * n_freq;
        let row_b = j_b * n_freq;
        // Demux. Modular wrap (NFFT_SPEC=3840 (2560 at 8 kHz) isn't a power of two so
        // `& (NFFT-1)` would alias the high bins). `kn = (NFFT-k) mod
        // NFFT` collapses k=0 to 0 — DC bin is real.
        //
        // D4: DC bin hoisted; main loop 4× unrolled to expose
        // independent multiply chains at opt-level=1. Per-square u32
        // cast: each square ≤ 2^30 fits in i32; cast before adding so
        // the sum ≤ 2^31 stays within u32 (avoids i32 panic in debug
        // for the ±32768 edge case that `unsigned_abs` was guarding).

        // k=0: DC bin (kn=0 → yn=yk)
        if lo == 0 && hi > 0 {
            let r0 = buf[0].re as i32;
            let i0 = buf[0].im as i32;
            data[row_a] = ((r0 * r0) as u32 >> FP_SPEC_SHIFT).min(u16::MAX as u32) as u16;
            data[row_b] = ((i0 * i0) as u32 >> FP_SPEC_SHIFT).min(u16::MAX as u32) as u16;
        }

        let mut k = lo.max(1);
        while k + 4 <= hi {
            let yk0 = buf[k];
            let yk1 = buf[k + 1];
            let yk2 = buf[k + 2];
            let yk3 = buf[k + 3];
            let yn0 = buf[nfft - k];
            let yn1 = buf[nfft - k - 1];
            let yn2 = buf[nfft - k - 2];
            let yn3 = buf[nfft - k - 3];

            let (a0r, a0i) = (
                (yk0.re as i32 + yn0.re as i32) >> 1,
                (yk0.im as i32 - yn0.im as i32) >> 1,
            );
            let (b0r, b0i) = (
                (yk0.im as i32 + yn0.im as i32) >> 1,
                (yn0.re as i32 - yk0.re as i32) >> 1,
            );
            let (a1r, a1i) = (
                (yk1.re as i32 + yn1.re as i32) >> 1,
                (yk1.im as i32 - yn1.im as i32) >> 1,
            );
            let (b1r, b1i) = (
                (yk1.im as i32 + yn1.im as i32) >> 1,
                (yn1.re as i32 - yk1.re as i32) >> 1,
            );
            let (a2r, a2i) = (
                (yk2.re as i32 + yn2.re as i32) >> 1,
                (yk2.im as i32 - yn2.im as i32) >> 1,
            );
            let (b2r, b2i) = (
                (yk2.im as i32 + yn2.im as i32) >> 1,
                (yn2.re as i32 - yk2.re as i32) >> 1,
            );
            let (a3r, a3i) = (
                (yk3.re as i32 + yn3.re as i32) >> 1,
                (yk3.im as i32 - yn3.im as i32) >> 1,
            );
            let (b3r, b3i) = (
                (yk3.im as i32 + yn3.im as i32) >> 1,
                (yn3.re as i32 - yk3.re as i32) >> 1,
            );

            data[row_a + (k - lo)] = (((a0r * a0r) as u32 + (a0i * a0i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_b + (k - lo)] = (((b0r * b0r) as u32 + (b0i * b0i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_a + (k - lo) + 1] = (((a1r * a1r) as u32 + (a1i * a1i) as u32)
                >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_b + (k - lo) + 1] = (((b1r * b1r) as u32 + (b1i * b1i) as u32)
                >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_a + (k - lo) + 2] = (((a2r * a2r) as u32 + (a2i * a2i) as u32)
                >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_b + (k - lo) + 2] = (((b2r * b2r) as u32 + (b2i * b2i) as u32)
                >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_a + (k - lo) + 3] = (((a3r * a3r) as u32 + (a3i * a3i) as u32)
                >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_b + (k - lo) + 3] = (((b3r * b3r) as u32 + (b3i * b3i) as u32)
                >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;

            k += 4;
        }
        while k < hi {
            let yn = buf[nfft - k];
            let yk = buf[k];
            let (ar, ai) = (
                (yk.re as i32 + yn.re as i32) >> 1,
                (yk.im as i32 - yn.im as i32) >> 1,
            );
            let (br, bi) = (
                (yk.im as i32 + yn.im as i32) >> 1,
                (yn.re as i32 - yk.re as i32) >> 1,
            );
            data[row_a + (k - lo)] = (((ar * ar) as u32 + (ai * ai) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            data[row_b + (k - lo)] = (((br * br) as u32 + (bi * bi) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            k += 1;
        }
    }

    // Odd-frame fallback: single-frame FFT for the trailing slice when
    // n_time is odd. n_time=184 on the standard FT8 slot so this path
    // is exercised only by truncated inputs / regression-test fixtures.
    if n_odd != 0 {
        let j = 2 * n_pairs;
        pack(&mut buf, j * nstep, None);
        fft.process(&mut buf);
        let row_base = j * n_freq;
        for i in lo..hi {
            let re = buf[i].re as i32;
            let im = buf[i].im as i32;
            let mag2 =
                (((re * re) as u32 + (im * im) as u32) >> FP_SPEC_SHIFT).min(u16::MAX as u32);
            data[row_base + (i - lo)] = mag2 as u16;
        }
    }

    Spectrogram {
        n_freq,
        n_time,
        data,
    }
}

#[cfg(all(test, not(feature = "fixed-point")))]
mod tests {
    use super::*;

    /// A full slot holding a steady tone of `f` Hz at input rate `rate`.
    fn tone(rate: Ft8Rate, f: f32) -> Vec<i16> {
        let hz = rate.hz() as f32;
        (0..rate.nmax())
            .map(|n| (8000.0 * (core::f32::consts::TAU * f * n as f32 / hz).cos()) as i16)
            .collect()
    }

    /// The spectrogram of either rate is the same grid: 3.125 Hz bins, 0.04 s (0.08 s under `nstep-half`) rows, the
    /// same number of each, a tone in the same bin. What reads the spectrogram (`sync_grid`, the score, the gate)
    /// does not know the rate, so a frame, FFT, step or slot length not scaled with it moves this: the rows beyond
    /// the slot are silence, the bins are the wrong frequencies. Tones on the bin centres (a multiple of 3.125 Hz).
    #[test]
    fn the_spectrogram_of_either_rate_is_the_same_grid() {
        let build = |rate: Ft8Rate, f: f32| {
            let slot = tone(rate, f);
            let n_freq = spectrogram_n_freq(3000.0, rate);
            let s = compute_spectrogram_bins_src(&mut SliceSource(&slot), rate, 0, n_freq, ());
            (n_freq, s)
        };
        for f in [500.0f32, 1000.0, 2500.0] {
            let (n12, a) = build(Ft8Rate::Hz12000, f);
            let (n8, b) = build(Ft8Rate::Hz8000, f);
            assert_eq!(n12, n8, "bins up to 3 kHz + the tones");
            assert_eq!((a.n_freq, a.n_time), (b.n_freq, b.n_time), "{f} Hz");
            // a mid-slot row: the tone's bin is the strongest, and the same one
            let peak = |s: &Spectrogram| {
                let row = &s.data[(s.n_time / 2) * s.n_freq..][..s.n_freq];
                row.iter()
                    .enumerate()
                    .max_by(|x, y| x.1.total_cmp(y.1))
                    .unwrap()
                    .0
            };
            assert_eq!(peak(&a), (f / 3.125) as usize, "12 kHz, {f} Hz");
            assert_eq!(peak(&b), (f / 3.125) as usize, "8 kHz, {f} Hz");
            // the last row still holds the tone, against a mid-slot row of the same spectrogram: the slot, not a
            // longer one, is what the rows cover (a cell's size is the frame's, `NSPS^2`, so rates are compared
            // within themselves)
            let cell = |s: &Spectrogram, j: usize| {
                s.data[j * s.n_freq..][..s.n_freq]
                    .iter()
                    .cloned()
                    .fold(0.0, f32::max)
            };
            for (name, s) in [("12 kHz", &a), ("8 kHz", &b)] {
                let (last, mid) = (cell(s, s.n_time - 1), cell(s, s.n_time / 2));
                assert!(
                    last > 0.5 * mid,
                    "{name} {f} Hz: last row {last} against {mid}"
                );
            }
        }
    }
}

#[cfg(all(test, feature = "fixed-point"))]
mod tests_fixed {
    use super::*;

    /// A full slot holding a steady tone of `f` Hz at amplitude `amp`, at input rate `rate`, and one spike of 20 000 in the
    /// first frame: the auto-gain follows the slot's peak, so a bare tone would be brought to the top of the u16 range (a
    /// real slot's strongest signal is well below its peak sample); with the spike the shift is 0 and the tone keeps its size.
    fn tone(rate: Ft8Rate, f: f32, amp: f32) -> Vec<i16> {
        let hz = rate.hz() as f32;
        let mut v: Vec<i16> = (0..rate.nmax())
            .map(|n| (amp * (core::f32::consts::TAU * f * n as f32 / hz).cos()) as i16)
            .collect();
        v[5] = 20_000;
        v
    }

    fn build(rate: Ft8Rate, slot: &[i16]) -> Spectrogram {
        let mut src = SliceSource(slot);
        let gain = spectrogram_gain(&mut src);
        compute_spectrogram_bins_src(&mut src, rate, 0, spectrogram_n_freq(3000.0, rate), gain)
    }

    /// The Q15 table is the periodic Hann: 0 at the first sample, 32767 at the middle, symmetric about it (`w[k] == w[n - k]`),
    /// `sin^2` to within one LSB, and it sums to half its length (the property the cell scale rests on: `sum w = nfft / 2`).
    #[test]
    fn hann_q15_is_the_periodic_hann() {
        for n in [2560usize, 3840] {
            let w = hann_q15(n);
            assert_eq!((w[0], w[n / 2]), (0, 32767), "{n}: ends and peak");
            for k in 1..n {
                assert_eq!(w[k], w[n - k], "{n}: symmetric at {k}");
            }
            for k in (0..n).step_by(37) {
                let want = (core::f64::consts::PI * k as f64 / n as f64).sin().powi(2) * 32767.0;
                assert!((w[k] as f64 - want).abs() <= 1.0, "{n}: sin^2 at {k}");
            }
            let sum: f64 = w.iter().map(|&v| v as f64).sum();
            assert!(
                (sum / (n as f64 / 2.0 * 32767.0) - 1.0).abs() < 1e-3,
                "{n}: the window sums to {sum}"
            );
        }
    }

    /// The 8 kHz frame is a Hann window over two symbols (`Ft8Rate::coarse_hann_2t`): a strong tone does not leak into the
    /// bins around it. A tone of 3 000 (cell about 30 000) at 1 000 Hz leaves the cells 25 Hz and 50 Hz away at 0 at 8 kHz;
    /// the rectangular frame (12 kHz, 260 of 30 000 at 25 Hz: -21 dB in power) leaves 260 and 57, on the grid and half a bin off it. Fails if the
    /// 8 kHz frame is rectangular again (the cells at 25 Hz are then about 260); the window's shape is pinned by `hann_q15_is_the_periodic_hann`
    /// and the cell tests (a half-sine passes this one).
    #[test]
    fn the_8_khz_frame_does_not_leak_a_strong_tone_into_the_bins_around_it() {
        for off in [0.0f32, 1.5625] {
            let f = 1000.0 + off;
            let pk = (f / 3.125) as usize;
            let mid = 92;
            let a = build(Ft8Rate::Hz12000, &tone(Ft8Rate::Hz12000, f, 3000.0));
            let b = build(Ft8Rate::Hz8000, &tone(Ft8Rate::Hz8000, f, 3000.0));
            let band = |s: &Spectrogram, from: usize| {
                *s.data[mid * s.n_freq..][..s.n_freq][pk + from..pk + from + 4]
                    .iter()
                    .max()
                    .unwrap()
            };
            let peak = |s: &Spectrogram| {
                *s.data[mid * s.n_freq..][..s.n_freq][pk.saturating_sub(1)..pk + 3]
                    .iter()
                    .max()
                    .unwrap()
            };
            assert!(
                peak(&b) > 20_000 && peak(&a) > 20_000,
                "{off}: peaks {} {}",
                peak(&b),
                peak(&a)
            );
            for from in [8usize, 16, 32] {
                assert!(
                    band(&a, from) >= if from == 32 { 4 } else { 20 },
                    "{off} +{from} bins: the rectangular frame leaks {} (the premise of this test)",
                    band(&a, from)
                );
                assert!(
                    band(&b, from) <= 1,
                    "{off} +{from} bins: the 8 kHz frame leaks {} of {}",
                    band(&b, from),
                    peak(&b)
                );
            }
        }
    }

    /// The 8 kHz row means the same time as the rectangular one's: a burst of one symbol that starts on a row's first sample
    /// is strongest in **that** row (the Hann frame is `nsps / 2` earlier and `nsps` longer, so its centre is the rectangular
    /// frame's). Fails if the frame is not shifted by `nsps / 2` (the burst is then strongest a row early) or the window is
    /// not symmetric about its centre.
    #[test]
    fn the_8_khz_rows_keep_the_time_of_the_rectangular_ones() {
        let rate = Ft8Rate::Hz8000;
        let (nsps, nstep) = (rate.nsps(), rate.nstep());
        for row in [40usize, 61, 100] {
            let mut v = vec![0i16; rate.nmax()];
            let f = 1500.0f32;
            for n in 0..nsps {
                let t = (row * nstep + n) as f32;
                v[row * nstep + n] =
                    (3000.0 * (core::f32::consts::TAU * f * t / 8000.0).cos()) as i16;
            }
            v[5] = 20_000;
            let sp = build(rate, &v);
            let bin = (f / 3.125) as usize;
            let cell = |j: usize| sp.data[j * sp.n_freq + bin];
            let best = (0..sp.n_time).max_by_key(|&j| cell(j)).unwrap();
            assert_eq!(
                best,
                row,
                "burst at row {row}: rows around it {:?}",
                (row - 2..row + 3).map(cell).collect::<Vec<_>>()
            );
        }
    }

    /// The fixed-point spectrogram at 8 kHz is the 12 kHz one's grid **and cells**: the same bins and rows, a tone in the same
    /// bin, and the same u16 value in it (to the rounding of the 15-bit pre-scale and the `>> FP_SPEC_SHIFT`), because the
    /// input is scaled by 12000 / rate. Fails if the 2560-point plan's gain is not `1/256`, the frame, FFT or step is not the
    /// rate's, the input gain is missing (the cell would be 4/9 of it) or the demux index is the 12 kHz one.
    #[test]
    fn the_fixed_point_spectrogram_of_either_rate_has_the_same_cells() {
        for f in [500.0f32, 1000.0, 2500.0] {
            let a = build(Ft8Rate::Hz12000, &tone(Ft8Rate::Hz12000, f, 500.0));
            let b = build(Ft8Rate::Hz8000, &tone(Ft8Rate::Hz8000, f, 500.0));
            assert_eq!((a.n_freq, a.n_time), (b.n_freq, b.n_time), "{f} Hz");
            let bin = (f / 3.125) as usize;
            let mid = a.n_time / 2;
            let cell = |s: &Spectrogram, i: usize| s.data[mid * s.n_freq + i] as f32;
            let (ca, cb) = (cell(&a, bin), cell(&b, bin));
            assert!(ca > 500.0, "{f} Hz: the 12 kHz cell is {ca}, no signal");
            assert!(
                (cb / ca - 1.0).abs() < 0.03,
                "{f} Hz: 8 kHz cell {cb} against 12 kHz {ca}"
            );
            // the tone's bin is the strongest of the row at either rate
            for (name, s) in [("12k", &a), ("8k", &b)] {
                let row = &s.data[mid * s.n_freq..][..s.n_freq];
                let peak = row.iter().enumerate().max_by_key(|x| *x.1).unwrap().0;
                assert_eq!(
                    peak,
                    bin,
                    "{name} {f} Hz row {:?}",
                    &row[bin.saturating_sub(3)..bin + 4]
                );
            }
            // and the slot, not a longer one, is what the rows cover: the last row still holds the tone
            let last = |s: &Spectrogram| s.data[(s.n_time - 1) * s.n_freq + bin] as f32;
            assert!(
                last(&b) > 0.5 * cell(&b, bin),
                "{f} Hz: last row {}",
                last(&b)
            );
        }
    }

    /// The same noise **density** gives the noise cells of the frame's own bandwidth at either rate (the rate's
    /// `sqrt(rate / 12000)` on the sample deviation): 12 kHz's rectangular frame is the reference; the 8 kHz Hann frame over two
    /// symbols has `sum w^2 = 0.375 nfft` against the rectangular `nsps`, i.e. **0.75 of it (-1.25 dB)** while a steady tone's cell is
    /// the same (`sum w = nsps`, the test above; a one-symbol tone's is 0.67). Fails if the input gain is not `12000 / rate`, or the 8 kHz frame is
    /// rectangular again (the ratio is then 1.0).
    #[test]
    fn noise_of_the_same_density_gives_the_same_cells() {
        let noise = |rate: Ft8Rate| -> Vec<i16> {
            let sigma = 1500.0 * (rate.hz() as f32 / 12_000.0).sqrt();
            let mut s = 0x1234_5678_9abc_def0u64;
            (0..rate.nmax())
                .map(|_| {
                    let mut a = 0.0f32;
                    for _ in 0..12 {
                        s = s
                            .wrapping_mul(6364136223846793005)
                            .wrapping_add(1442695040888963407);
                        a += ((s >> 40) as f32) / (1u64 << 24) as f32;
                    }
                    ((a - 6.0) * sigma) as i16
                })
                .collect()
        };
        let mean = |rate: Ft8Rate| -> f64 {
            let sp = build(rate, &noise(rate));
            sp.data.iter().map(|&v| v as f64).sum::<f64>() / sp.data.len() as f64
        };
        let (a, b) = (mean(Ft8Rate::Hz12000), mean(Ft8Rate::Hz8000));
        assert!(a > 50.0, "12 kHz mean cell {a}");
        assert!(
            (b / a - 0.75).abs() < 0.04,
            "8 kHz mean cell {b} against 12 kHz {a} (expected 0.75 of it)"
        );
    }
}
