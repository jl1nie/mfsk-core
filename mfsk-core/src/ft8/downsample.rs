//! FT8-tuned wrapper around the generic downsampler in
//! [`crate::core::dsp::downsample`].
//!
//! Keeps the pre-existing `crate::ft8::downsample::{downsample, downsample_simple,
//! build_fft_cache}` signatures so existing callers (`decode.rs`, WASM glue,
//! benchmarks) continue working without change. The heavy lifting lives in
//! `mfsk-core` so FT4 and future LDPC-family modes reuse it.

use crate::core::dsp::downsample::{self as g, DownsampleCfg};
use num_complex::Complex;

/// FT8 downsample configuration: 12 kHz → 200 Hz, 8 tones spaced 6.25 Hz apart.
pub const FT8_CFG: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 192_000,
    fft2_size: 3_200,
    tone_spacing_hz: 6.25,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 8,
    edge_taper_bins: 101,
};

/// Downconvert and decimate `audio` to a complex baseband at 200 Hz centred
/// on `f0`. Matches the pre-refactor signature: returns the result plus the
/// forward-FFT cache so candidate loops can avoid recomputing it.
#[inline]
pub fn downsample(
    audio: &[i16],
    f0: f32,
    fft_cache: Option<&[Complex<f32>]>,
) -> (Vec<Complex<f32>>, Vec<Complex<f32>>) {
    match fft_cache {
        Some(cache) => (g::downsample_cached(cache, f0, &FT8_CFG), cache.to_vec()),
        None => g::downsample(audio, f0, &FT8_CFG),
    }
}

/// Compute only the forward FFT cache (192 000-point) — expensive, shared
/// across all subsequent downsample calls for the same audio block.
#[inline]
pub fn build_fft_cache(audio: &[i16]) -> Vec<Complex<f32>> {
    g::build_fft_cache(audio, &FT8_CFG)
}

/// No-cache convenience: returns only the 3200-sample baseband.
#[inline]
pub fn downsample_simple(audio: &[i16], f0: f32) -> Vec<Complex<f32>> {
    downsample(audio, f0, None).0
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::params::NMAX;

    const NFFT2: usize = 3_200;

    #[test]
    fn sine_at_f0_energy_at_dc() {
        let f0 = 1000.0f32;
        let audio: Vec<i16> = (0..NMAX)
            .map(|n| {
                let t = n as f32 / 12_000.0;
                (10_000.0 * (2.0 * std::f32::consts::PI * f0 * t).sin()) as i16
            })
            .collect();

        let out = downsample_simple(&audio, f0);

        let mut spectrum = out.clone();
        let mut planner = rustfft::FftPlanner::<f32>::new();
        planner.plan_fft_forward(NFFT2).process(&mut spectrum);

        let energy_near_dc: f32 = spectrum[..=10]
            .iter()
            .chain(spectrum[NFFT2 - 10..].iter())
            .map(|c| c.norm_sqr())
            .sum();
        let total_energy: f32 = spectrum.iter().map(|c| c.norm_sqr()).sum();

        assert!(total_energy > 0.0);
        let frac = energy_near_dc / total_energy;
        assert!(frac > 0.5, "energy near DC fraction = {frac:.3}");
    }

    #[test]
    fn sine_offset_from_f0_not_at_dc() {
        let f0 = 1000.0f32;
        let audio: Vec<i16> = (0..NMAX)
            .map(|n| {
                let t = n as f32 / 12_000.0;
                (10_000.0 * (2.0 * std::f32::consts::PI * (f0 + 100.0) * t).sin()) as i16
            })
            .collect();

        let out = downsample_simple(&audio, f0);

        let mut spectrum = out.clone();
        let mut planner = rustfft::FftPlanner::<f32>::new();
        planner.plan_fft_forward(NFFT2).process(&mut spectrum);

        let energy_near_dc: f32 = spectrum[..=2].iter().map(|c| c.norm_sqr()).sum();
        let total_energy: f32 = spectrum.iter().map(|c| c.norm_sqr()).sum();

        let frac = energy_near_dc / total_energy;
        assert!(frac < 0.1, "energy at DC fraction = {frac:.3}");
    }

    #[test]
    fn output_length() {
        let audio = vec![0i16; NMAX];
        let out = downsample_simple(&audio, 1000.0);
        assert_eq!(out.len(), NFFT2);
    }

    #[test]
    fn silence_gives_zero_output() {
        let audio = vec![0i16; NMAX];
        let out = downsample_simple(&audio, 1500.0);
        let max_abs = out.iter().map(|c| c.norm()).fold(0.0f32, f32::max);
        assert!(max_abs < 1e-10);
    }
}
