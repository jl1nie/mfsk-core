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

use super::super::params::{NMAX, NSPS, NTONES};
use super::source::{AudioSource, SliceSource, Window};
use super::types::{AudioSample, NFFT_SPEC, NSTEP, SAMPLE_RATE_HZ, TONE_SPACING_HZ};

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
pub(super) fn spectrogram_n_freq(max_freq_hz: f32) -> usize {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
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
    compute_spectrogram_bins(audio, 0, spectrogram_n_freq(max_freq_hz))
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
    compute_spectrogram_bins_src(&mut SliceSource(audio), bin_lo, bin_hi, ())
}

/// What a band's spectrogram needs from the whole slot before it can be built: nothing in the f32 build.
#[cfg(not(feature = "fixed-point"))]
pub(super) type Gain = ();

/// The slot-wide gain of the spectrogram (the fixed-point build scans the slot for its peak).
#[cfg(not(feature = "fixed-point"))]
pub(super) fn spectrogram_gain<A: AudioSource>(_src: &mut A) -> Gain {}

/// [`compute_spectrogram_bins`] from an [`AudioSource`]: the frames are read forward, one window at a time.
#[cfg(not(feature = "fixed-point"))]
pub(super) fn compute_spectrogram_bins_src<A: AudioSource>(
    src: &mut A,
    bin_lo: usize,
    bin_hi: usize,
    _gain: Gain,
) -> Spectrogram {
    assert!(
        bin_lo <= bin_hi && bin_hi <= NFFT_SPEC / 2 + 1,
        "spectrogram bins out of range"
    );
    let n_freq = bin_hi - bin_lo;
    let n_time = NMAX / NSTEP - 3;
    let scale = 1.0f32 / 300.0;

    // A real-input FFT, as `sync8.f90`'s `four2a(x,NFFT1,1,-1,0)` is: half the work of the
    // complex FFT of a zero-imaginary signal this ran until #761, the same bins up to rounding.
    let mut planner = default_planner();
    let fft = planner.plan_real_forward(NFFT_SPEC);

    // Rectangular window — matches WSJT-X `sync8.f90`. With NFFT=3840,
    // `tone_step_bins = 6.25 / (12000/3840) = 2.0` exactly, so the
    // rectangular sidelobes do not bleed onto adjacent FT8 tones.
    // (The fixed-point path keeps Hann to mitigate the wider sidelobes
    // that come with NFFT=4096's fractional 2.13 bins/tone.)
    let mut data = vec![0.0f32; n_freq * n_time];
    let mut input = vec![0.0f32; NFFT_SPEC];
    let mut buf = vec![Complex::new(0.0f32, 0.0); NFFT_SPEC / 2 + 1];

    let mut win = Window::new(src);
    let mut frame = vec![0i16; NSPS];
    for j in 0..n_time {
        let ia = j * NSTEP;
        win.read_into(ia as i64, &mut frame);
        // `process` may use `input` as scratch, so it is rewritten whole for every row.
        for (k, c) in input.iter_mut().enumerate() {
            *c = if k < NSPS {
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
    compute_spectrogram_bins(audio, 0, spectrogram_n_freq(max_freq_hz))
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
    compute_spectrogram_bins_src(&mut src, lo, hi, gain)
}

/// The slot-wide gain of the fixed-point spectrogram: the left shift that brings the slot's peak to
/// `2 x NFFT` (see [`compute_spectrogram_bins_src`]).
#[cfg(feature = "fixed-point")]
pub(super) type Gain = u32;

/// Scan the slot (the first `NMAX` samples) for its peak and return the shift. One pass over the source.
#[cfg(feature = "fixed-point")]
pub(super) fn spectrogram_gain<A: AudioSource>(src: &mut A) -> Gain {
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

/// [`compute_spectrogram_bins`] from an [`AudioSource`], with the slot's gain given: the frames are read
/// forward, one window at a time.
#[cfg(feature = "fixed-point")]
pub(super) fn compute_spectrogram_bins_src<A: AudioSource>(
    src: &mut A,
    lo: usize,
    hi: usize,
    shift: Gain,
) -> Spectrogram {
    use crate::engine::fft::default_planner_16;

    assert!(
        lo <= hi && hi <= NFFT_SPEC / 2 + 1,
        "spectrogram bins out of range"
    );
    let n_freq = hi - lo;
    let n_time = NMAX / NSTEP - 3;

    // The pre-scale gain (`shift`, from `spectrogram_gain`): the slot's peak lands at `2 x NFFT`, so that after
    // `log2(NFFT)` stages of /2 the post-FFT peak still has a usable mantissa of ~2. The scan covers the full
    // slot (the ESP32's `stage1_inc` peaks only the first second, since it cannot wait for the tail; the two
    // agree on real recordings: `qso3_busy.wav`'s peak is in its first 12 k samples). The Hann window and its
    // +1 pre-shift were removed when the spectrogram went to NFFT=3840: at integer tone alignment the
    // rectangular window's sidelobes do not leak onto adjacent FT8 tones.

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

    // **Rectangular window** — matches `embedded-shared::stage1_inc`
    // verbatim. At NFFT=3840 the FT8 tone spacing falls on integer
    // bins (`tone_step_bins = 6.25 / (12000/3840) = 2.0`), so the
    // rectangular sidelobes don't leak onto adjacent tones; Hann's
    // coherent gain 0.5 (≈ -6 dB SNR penalty) + 2-bin mainlobe spread
    // would be pure cost.
    let mut win = Window::new(src);
    let (mut fa, mut fb) = (vec![0i16; NSPS], vec![0i16; NSPS]);
    let mut pack = |buf: &mut [Complex<i16>], ia_a: usize, ia_b: Option<usize>| {
        win.read_into(ia_a as i64, &mut fa);
        if let Some(ia_b) = ia_b {
            win.read_into(ia_b as i64, &mut fb);
        }
        let scale =
            |raw: i16| (((raw as i32) << shift).clamp(i16::MIN as i32, i16::MAX as i32)) as i16;
        for (k, c) in buf.iter_mut().enumerate() {
            let re = if k < NSPS { scale(fa[k]) } else { 0 };
            let im = match ia_b {
                Some(_) if k < NSPS => scale(fb[k]),
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
            let yn0 = buf[NFFT_SPEC - k];
            let yn1 = buf[NFFT_SPEC - k - 1];
            let yn2 = buf[NFFT_SPEC - k - 2];
            let yn3 = buf[NFFT_SPEC - k - 3];

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
            let yn = buf[NFFT_SPEC - k];
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
        pack(&mut buf, j * NSTEP, None);
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
