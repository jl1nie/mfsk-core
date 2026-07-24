//! Spectrum baseline estimator — historical FT8 entry point.
//!
//! The polynomial-fit implementation moved to
//! [`crate::core::baseline`] in 2026-05 (slice 1 of issue #18) so that
//! FT4's `coarse_sync` can normalise its candidate spectrum the same
//! way WSJT-X does (`ft4_baseline.f90`). The algorithm is identical
//! across FT8 / FT4 / FST4 in WSJT-X — keeping a single Rust port
//! avoids drift.
//!
//! This module is now a thin re-export plus the FT8-specific
//! [`avg_spectrum`] helper that consumes the embedded
//! [`crate::ft8::decode_block::Spectrogram`].

#![cfg(feature = "std")]

pub use crate::core::baseline::fit_baseline;

/// Compute the average linear power per FFT bin from a `Spectrogram`.
/// `out.len()` must equal `spec.n_freq`. FT8-specific because it
/// targets the embedded `decode_block::Spectrogram` layout.
pub fn avg_spectrum(spec: &crate::ft8::decode_block::Spectrogram, out: &mut [f32]) {
    debug_assert_eq!(out.len(), spec.n_freq);
    out.fill(0.0);
    for t in 0..spec.n_time {
        for f in 0..spec.n_freq {
            #[allow(clippy::unnecessary_cast)]
            let v = spec.data[t * spec.n_freq + f] as f32;
            out[f] += v;
        }
    }
    let inv = 1.0 / spec.n_time as f32;
    for v in out.iter_mut() {
        *v *= inv;
    }
}

/// WSJT-X `get_spectrum_baseline` for FT8 SNR validation.
///
/// This is deliberately separate from the short rectangular-window
/// spectrogram used by coarse synchronization. WSJT-X derives `xbase`
/// from 3840-sample Nuttall-windowed FFTs stepped by 1920 samples across
/// the original slot. Mixing the two spectrum domains miscalibrates the
/// post-decode weak-signal gate.
#[cfg(feature = "fft-rustfft")]
pub fn spectrum_baseline<S: crate::ft8::decode_block::AudioSample>(
    audio: &[S],
    mut freq_min_hz: f32,
    mut freq_max_hz: f32,
) -> Vec<f32> {
    use num_complex::Complex;

    const NFFT: usize = 3840;
    const STEP: usize = NFFT / 2;
    const SAMPLE_RATE: f32 = 12_000.0;
    const NSPS: f32 = 1920.0;

    let mut window = vec![0.0_f32; NFFT];
    for (i, value) in window.iter_mut().enumerate() {
        let phase = core::f32::consts::TAU * i as f32 / NFFT as f32;
        *value = 0.363_581_9 - 0.489_177_5 * phase.cos() + 0.136_599_5 * (2.0 * phase).cos()
            - 0.010_641_1 * (3.0 * phase).cos();
    }
    let window_scale = NSPS * 2.0 / 300.0 / window.iter().sum::<f32>();
    for value in &mut window {
        *value *= window_scale;
    }

    let mut planner = crate::core::fft::default_planner();
    let fft = planner.plan_forward(NFFT);
    let mut buffer = vec![Complex::new(0.0_f32, 0.0); NFFT];
    let mut summed_power = vec![0.0_f32; NFFT / 2 + 1];
    for start in (0..audio.len()).step_by(STEP) {
        if start + NFFT > audio.len() {
            break;
        }
        for i in 0..NFFT {
            buffer[i] = Complex::new(audio[start + i].to_f32() * window[i], 0.0);
        }
        fft.process(&mut buffer);
        for bin in 1..=NFFT / 2 {
            summed_power[bin] += buffer[bin].norm_sqr();
        }
    }

    let original_width = freq_max_hz - freq_min_hz;
    if freq_min_hz < 100.0 {
        freq_min_hz = 100.0;
        if original_width < 100.0 {
            freq_max_hz = freq_min_hz + original_width;
        }
    }
    if freq_max_hz > 4910.0 {
        freq_max_hz = 4910.0;
        if original_width < 100.0 {
            freq_min_hz = freq_max_hz - original_width;
        }
    }

    let df = SAMPLE_RATE / NFFT as f32;
    let first = ((freq_min_hz / df).round() as usize).max(1);
    let last = (freq_max_hz / df).round() as usize;
    let fitted = fit_baseline(&summed_power, first, last);
    let mut out = vec![0.0_f32; summed_power.len()];
    out[first..first + fitted.len()].copy_from_slice(&fitted);
    out
}
