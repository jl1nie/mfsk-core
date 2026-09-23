//! JT9 baseband decimation: 12 kHz → 1500 Hz complex.
//!
//! Mirrors WSJT-X `lib/downsam9.f90`: multiply by `e^{-j2π f n / Fs}`
//! then box-car integrate ÷8.  At 1500 Hz the 9-tone constellation
//! (tones 0–8, spacing ≈ 1.736 Hz) occupies only ~15 Hz — far below
//! the 750 Hz Nyquist — so box-car is adequate.
//!
//! NSPS_BB = 6912 / 8 = **864** baseband samples per symbol.

use alloc::vec;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use core::f32::consts::TAU;

/// Baseband samples per symbol (1500 Hz, NSPS = 6912 at 12 kHz).
pub const NSPS_BB: usize = 864;

/// Mix `audio` (12 kHz) to complex baseband centered at `freq_hz`,
/// then box-car integrate ÷8 to reach 1500 Hz.
///
/// Returns `(idat, qdat)`, each of length `⌊audio.len() / 8⌋`.
pub fn mix_to_baseband(audio: &[f32], freq_hz: f32) -> (Vec<f32>, Vec<f32>) {
    let n_bb = audio.len() / 8;
    let mut idat = vec![0.0f32; n_bb];
    let mut qdat = vec![0.0f32; n_bb];
    let phase_step = TAU * freq_hz / 12_000.0;
    for k in 0..n_bb {
        let mut si = 0.0f32;
        let mut sq = 0.0f32;
        for j in 0..8usize {
            let n = k * 8 + j;
            let theta = phase_step * n as f32;
            let (sin_t, cos_t) = theta.sin_cos();
            si += audio[n] * cos_t;
            sq -= audio[n] * sin_t;
        }
        idat[k] = si;
        qdat[k] = sq;
    }
    (idat, qdat)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn length_is_audio_div_8() {
        let audio = vec![0.0f32; 720_000]; // 60 s @ 12 kHz
        let (i, q) = mix_to_baseband(&audio, 1000.0);
        assert_eq!(i.len(), 90_000);
        assert_eq!(q.len(), 90_000);
    }

    #[test]
    fn dc_tone_concentrates_at_dc() {
        // Pure cosine at freq_hz → should give large idat, near-zero qdat
        let freq = 1200.0f32;
        let n = 8 * NSPS_BB; // one symbol worth
        let audio: Vec<f32> = (0..n)
            .map(|k| (TAU * freq * k as f32 / 12_000.0).cos())
            .collect();
        let (idat, qdat) = mix_to_baseband(&audio, freq);
        let energy_i: f32 = idat.iter().map(|x| x * x).sum();
        let energy_q: f32 = qdat.iter().map(|x| x * x).sum();
        assert!(
            energy_i > energy_q * 10.0,
            "expected I >> Q for cosine at mix freq: ei={energy_i:.1} eq={energy_q:.1}"
        );
    }
}
