//! Power-spectrogram stage: `Spectrogram` container + the two
//! `compute_spectrogram` backends (host `fft-rustfft` and embedded
//! `fixed-point`).
//!
//! ε.2 of the `docs/CLEANUP_2026_05.md` `decode_block` split. The
//! parent (`decode_block.rs`) re-exports `Spectrogram`, `SpecCell`,
//! and `compute_spectrogram` so external callers (`mfsk-ffi-ft8`,
//! `embedded-shared::pipeline::Stage1Output`, the m5stack apps,
//! integration tests) see the same `mfsk_core::ft8::decode_block::*`
//! paths as before the split. The remaining ex-private items
//! (`CoarseAcc`, `Spectrogram::power_acc`) are surfaced as
//! `pub(super)` for the coarse-sync code that still lives in the
//! parent — they'll move to siblings of this file in ε.3.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::params::{NMAX, NSPS, NTONES};
use super::types::{AudioSample, NFFT_SPEC, NSTEP, SAMPLE_RATE_HZ, TONE_SPACING_HZ};

#[cfg(not(feature = "fixed-point"))]
use crate::core::fft::default_planner;

// Hann window table for the fixed-point `compute_spectrogram`.
// Cached at first use — the table is constant in shape and length
// (NSPS = 1920), so recomputing 1920 `cos()` per call (= per
// multipass pass) was wasted work on every decode. Gemini PR #83
// review. `OnceLock` requires `std`; `fixed-point` builds also
// enable `std` (the embedded path uses `stage1_inc` which has its
// own cached table), so this is safe.
#[cfg(all(feature = "fixed-point", feature = "std"))]
use std::sync::OnceLock;
#[cfg(all(feature = "fixed-point", feature = "std"))]
static HANN_NSPS: OnceLock<[i16; NSPS]> = OnceLock::new();

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
    //
    // Cached in `HANN_NSPS` on `std` builds: the window is constant
    // in shape + length (NSPS = 1920), so recomputing the 1920
    // `cos()` calls per `compute_spectrogram` (= per multipass pass)
    // burned cycles on every decode. `OnceLock` is std-only so the
    // no_std fixed-point path keeps the per-call recompute — that
    // path runs on Xtensa anyway, and the embedded production
    // pipeline routes via `stage1_inc` which has its own cached
    // Hann (per the comment immediately above). Gemini PR #83 + #101
    // review.
    #[cfg(feature = "std")]
    let hann_cached = HANN_NSPS.get_or_init(|| {
        let mut table = [0i16; NSPS];
        for n in 0..NSPS {
            let phase = 2.0 * core::f32::consts::PI * (n as f32) / (NSPS as f32);
            let w = 0.5 - 0.5 * phase.cos();
            table[n] = (w * 32767.0) as i16;
        }
        table
    });
    #[cfg(not(feature = "std"))]
    let hann_local = {
        let mut table = [0i16; NSPS];
        for n in 0..NSPS {
            let phase = 2.0 * core::f32::consts::PI * (n as f32) / (NSPS as f32);
            let w = 0.5 - 0.5 * phase.cos();
            table[n] = (w * 32767.0) as i16;
        }
        table
    };
    #[cfg(feature = "std")]
    let hann: &[i16; NSPS] = hann_cached;
    #[cfg(not(feature = "std"))]
    let hann: &[i16; NSPS] = &hann_local;
    // **Rectangular window** — matches `embedded-shared::stage1_inc`
    // verbatim. Earlier the host path windowed with Hann (cached in
    // `HANN_NSPS` above) "for parity with stage1_inc" but stage1_inc
    // had already switched to rectangular at the NFFT=3840 migration
    // (see `embedded-shared/src/stage1_inc.rs:338`). The drift caused
    // host `decode_block_into` to lose 2 weak decodes that the
    // embedded S3 wav_sim baseline finds (XE2X, K1JT on qso3_busy.wav).
    // The Hann coherent gain 0.5 ≈ -6 dB SNR penalty + 2-bin mainlobe
    // spread is what cost the recall — at integer tone alignment
    // (tone_step_bins = 2.0 exactly) the rectangular sidelobes don't
    // leak onto adjacent FT8 tones, so the window provides no benefit.
    let _ = hann; // silence unused warning when std cache is built
    let pack = |buf: &mut [Complex<i16>], ia_a: usize, ia_b: Option<usize>| {
        for (k, c) in buf.iter_mut().enumerate() {
            let re = if k < NSPS && ia_a + k < audio.len() {
                let raw = audio[ia_a + k].to_i16() as i32;
                (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            } else {
                0
            };
            let im = match ia_b {
                Some(ia_b) if k < NSPS && ia_b + k < audio.len() => {
                    let raw = audio[ia_b + k].to_i16() as i32;
                    (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
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
            // Square via `unsigned_abs` to dodge the `i32` overflow at
            // `(-32768)² + (-32768)² = 2^31`, which doesn't fit in i32
            // (panic in debug, wrap to negative in release). Squaring as
            // `u32` keeps the sum in 2^31 with one bit to spare —
            // `>> FP_SPEC_SHIFT` (= 12) lands in u32.
            // `.min(u16::MAX as u32)` then saturates the u16 cast so
            // extremely-strong signals (two coincident tones at the
            // same bin per the FP_SPEC_SHIFT comment) cap at u16::MAX
            // instead of wrapping to a tiny value. Gemini PR #83 + #100
            // review.
            let aru = a_re.unsigned_abs();
            let aiu = a_im.unsigned_abs();
            let bru = b_re.unsigned_abs();
            let biu = b_im.unsigned_abs();
            let mag2_a = ((aru * aru + aiu * aiu) >> FP_SPEC_SHIFT).min(u16::MAX as u32);
            let mag2_b = ((bru * bru + biu * biu) >> FP_SPEC_SHIFT).min(u16::MAX as u32);
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
            // Same `unsigned_abs` square + saturating `.min(u16::MAX
            // as u32)` pattern as the demux loop above. Gemini PR #83
            // + #100 review.
            let ru = re.unsigned_abs();
            let iu = im.unsigned_abs();
            let mag2 = ((ru * ru + iu * iu) >> FP_SPEC_SHIFT).min(u16::MAX as u32);
            data[row_base + i] = mag2 as u16;
        }
    }

    Spectrogram {
        n_freq,
        n_time,
        data,
    }
}
