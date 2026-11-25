// SPDX-License-Identifier: GPL-3.0-only
//! Windowed-sinc rate conversion (Hann, 48 taps each side), double precision, deterministic.
//!
//! ICAP delivers 8 kHz audio (`icap_audio_read`), mfsk-core's FT8 is built on 12 kHz. This is the
//! conversion the profile puts between them: 12 -> 8 kHz models what the radio hands the app (the band
//! stays inside 0.2-3.5 kHz), 8 -> 12 kHz is what a decode path on mfsk-core's rate would have to add. It
//! is this crate's own filter, not the radio's, so the figures it gives are a model of the input, not of
//! the radio's DSP.

use core::f64::consts::PI;

const HW: i64 = 48;

pub fn resample(x: &[i16], from_hz: u32, to_hz: u32) -> Vec<i16> {
    if from_hz == to_hz {
        return x.to_vec();
    }
    let r = from_hz as f64 / to_hz as f64; // input samples per output sample
    let n_out = (x.len() as f64 / r) as usize;
    // cutoff in cycles per input sample: 0.47 of the lower Nyquist
    let fc = 0.47 * (to_hz as f64 / from_hz as f64).min(1.0);
    (0..n_out)
        .map(|i| {
            let t = i as f64 * r;
            let k0 = t.floor() as i64 - HW + 1;
            let mut a = 0.0;
            for k in k0..k0 + 2 * HW {
                if k < 0 || k as usize >= x.len() {
                    continue;
                }
                let d = t - k as f64;
                let w = 0.5 + 0.5 * (PI * d / HW as f64).cos();
                let s = if d.abs() < 1e-9 {
                    2.0 * fc
                } else {
                    (2.0 * PI * fc * d).sin() / (PI * d)
                };
                a += x[k as usize] as f64 * s * w;
            }
            a.round().clamp(i16::MIN as f64, i16::MAX as f64) as i16
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn length_and_tone_survive_a_round_trip() {
        let n = 12_000;
        let x: Vec<i16> = (0..n)
            .map(|i| (8000.0 * (2.0 * PI * 1000.0 * i as f64 / 12_000.0).sin()) as i16)
            .collect();
        let y = resample(&resample(&x, 12_000, 8_000), 8_000, 12_000);
        assert_eq!(y.len(), x.len());
        // a 1 kHz tone is well inside both bands: the amplitude must come back within 2 %
        let rms = |v: &[i16]| {
            (v[1000..n - 1000]
                .iter()
                .map(|&s| (s as f64).powi(2))
                .sum::<f64>()
                / (n - 2000) as f64)
                .sqrt()
        };
        assert!(
            (rms(&y) / rms(&x) - 1.0).abs() < 0.02,
            "{} vs {}",
            rms(&y),
            rms(&x)
        );
    }
}
