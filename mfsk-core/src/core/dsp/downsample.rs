//! FFT-based downconversion / decimation.
//!
//! Ported from WSJT-X `ft8_downsample.f90` and generalised to arbitrary MFSK
//! signals. The algorithm is:
//!
//! 1. Zero-pad the real input to [`DownsampleCfg::fft1_size`] samples and take
//!    a forward FFT.
//! 2. Extract the positive-frequency bins covering
//!    `[f0 - leading_pad·Δf, f0 + trailing_pad·Δf]`, where the pads are
//!    expressed in tone-spacing units.
//! 3. Hann-taper [`DownsampleCfg::edge_taper_bins`] on each side of the
//!    extracted block to suppress FFT ringing.
//! 4. Cyclic-rotate so that `f0` lands at DC.
//! 5. Inverse FFT of size [`DownsampleCfg::fft2_size`] gives the decimated
//!    complex baseband.
//! 6. Scale by `1 / sqrt(fft1_size · fft2_size)`.
//!
//! The output sample rate is `input_rate · fft2_size / fft1_size`. For the FT8
//! tuning (12 000 → 200 Hz) it is 200 Hz; FT4 will pick a different
//! `fft2_size` to keep a wider baseband.
//!
//! ## Why a config struct rather than `<P>` ?
//!
//! `fft1_size` is chosen per protocol for FFT efficiency (highly-composite
//! numbers close to the slot length). It is not simply derived from
//! `SLOT_S · sample_rate` — it is a tunable tied to the FFT backend's
//! strengths. Keeping it in a runtime struct lets callers express values that
//! are awkward to pin to associated constants.

use num_complex::Complex;
use rustfft::FftPlanner;

/// Runtime parameters shared by [`downsample`], [`downsample_cached`], and
/// [`build_fft_cache`]. Callers typically keep one instance per protocol.
#[derive(Clone, Copy, Debug)]
pub struct DownsampleCfg {
    /// Input sample rate in Hz (12 000 for the WSJT pipeline).
    pub input_rate: u32,

    /// Zero-padded forward-FFT length.
    pub fft1_size: usize,

    /// Inverse-FFT length = output sample count per call.
    pub fft2_size: usize,

    /// Tone spacing of the modulation in Hz (passed as the bandwidth unit so
    /// the extracted band scales with the protocol).
    pub tone_spacing_hz: f32,

    /// Bins of headroom below `f0` in tone-spacing units (FT8 uses 1.5).
    pub leading_pad_tones: f32,

    /// Bins of headroom above `f0 + (ntones-1)·tone_spacing` in tone-spacing
    /// units (FT8 uses 1.5 past tone 7 → 8.5 total).
    pub trailing_pad_tones: f32,

    /// Number of data tones; used together with `trailing_pad_tones` to
    /// determine the upper edge of the extracted band.
    pub ntones: u32,

    /// Length of the raised-cosine taper applied to each edge (typically 101).
    pub edge_taper_bins: usize,
}

impl DownsampleCfg {
    #[inline]
    fn bin_hz(&self) -> f32 {
        self.input_rate as f32 / self.fft1_size as f32
    }
}

/// Compute only the large forward-FFT cache. Subsequent calls to
/// [`downsample_cached`] reuse it across all candidate frequencies, which is
/// the expensive operation amortised by `ft8-core::decode::process_candidate`.
#[inline]
pub fn build_fft_cache(audio: &[i16], cfg: &DownsampleCfg) -> Vec<Complex<f32>> {
    let mut x: Vec<Complex<f32>> = audio
        .iter()
        .map(|&s| Complex::new(s as f32, 0.0))
        .chain(std::iter::repeat(Complex::new(0.0, 0.0)))
        .take(cfg.fft1_size)
        .collect();
    let mut planner = FftPlanner::<f32>::new();
    planner.plan_fft_forward(cfg.fft1_size).process(&mut x);
    x
}

/// Downconvert `audio` to a complex baseband centred on `f0`.
///
/// Returns the decimated signal plus the forward-FFT cache so the caller can
/// feed it to subsequent frequency-shifted calls without recomputing the
/// 192 k-point transform.
#[inline]
pub fn downsample(
    audio: &[i16],
    f0: f32,
    cfg: &DownsampleCfg,
) -> (Vec<Complex<f32>>, Vec<Complex<f32>>) {
    let cache = build_fft_cache(audio, cfg);
    let out = downsample_cached(&cache, f0, cfg);
    (out, cache)
}

/// Downconvert using a pre-computed forward-FFT cache.
#[inline]
pub fn downsample_cached(
    fft_cache: &[Complex<f32>],
    f0: f32,
    cfg: &DownsampleCfg,
) -> Vec<Complex<f32>> {
    debug_assert_eq!(fft_cache.len(), cfg.fft1_size);
    let mut planner = FftPlanner::<f32>::new();

    let df = cfg.bin_hz();
    let baud = cfg.tone_spacing_hz;

    let i0 = (f0 / df).round() as usize;
    let ft = f0 + (cfg.ntones as f32 - 1.0 + cfg.trailing_pad_tones) * baud;
    let fb = f0 - cfg.leading_pad_tones * baud;
    let it = ((ft / df).round() as usize).min(cfg.fft1_size / 2);
    let ib = ((fb / df).round() as usize).max(1);
    let k = it.saturating_sub(ib) + 1;

    let mut c1 = vec![Complex::new(0.0f32, 0.0); cfg.fft2_size];
    for (dst, src) in c1[..k.min(cfg.fft2_size)]
        .iter_mut()
        .zip(fft_cache[ib..=it].iter())
    {
        *dst = *src;
    }

    // Raised-cosine taper on leading and trailing edges.
    let et = cfg.edge_taper_bins;
    if et > 1 {
        let n = et - 1;
        let taper: Vec<f32> = (0..et)
            .map(|i| 0.5 * (1.0 + (i as f32 * std::f32::consts::PI / n as f32).cos()))
            .collect();
        for i in 0..et.min(k) {
            c1[i] *= taper[n - i];
        }
        if k > et {
            for i in 0..et {
                c1[k - et + i] *= taper[i];
            }
        }
    }

    // Cyclic shift so f0 lands on DC.
    let shift = i0.saturating_sub(ib) % cfg.fft2_size;
    c1.rotate_left(shift);

    // Inverse FFT.
    planner.plan_fft_inverse(cfg.fft2_size).process(&mut c1);

    // Combined scale factor.
    let fac = 1.0 / ((cfg.fft1_size as f32) * (cfg.fft2_size as f32)).sqrt();
    for s in c1.iter_mut() {
        *s *= fac;
    }

    c1
}
