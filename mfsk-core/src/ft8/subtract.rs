// SPDX-License-Identifier: GPL-3.0-or-later
//! FT8 signal subtraction (successive interference cancellation).
//!
//! Thin FT8-tuned wrapper around the protocol-agnostic
//! [`crate::core::dsp::subtract`] implementation. Given a decoded message and
//! its time/frequency coordinates, reconstructs the ideal 8-GFSK waveform and
//! subtracts it in place so weaker signals become decodable.

use super::{decode::DecodeResult, wave_gen::message_to_tones};
use crate::core::dsp::subtract::{SubtractCfg, subtract_tones};

/// FT8 subtract configuration: 12 kHz sample rate, 6.25 Hz tone spacing,
/// 1920 samples/symbol, frame origin at 0.5 s.
const FT8_CFG: SubtractCfg = SubtractCfg {
    sample_rate: 12_000.0,
    tone_spacing_hz: 6.25,
    samples_per_symbol: 1920,
    base_offset_s: 0.5,
};

/// Subtract a decoded FT8 signal from `audio` in-place (full amplitude).
#[inline]
pub fn subtract_signal(audio: &mut [i16], result: &DecodeResult) {
    subtract_signal_weighted(audio, result, 1.0);
}

/// Subtract a decoded FT8 signal with a fractional gain. `gain = 1.0` is full
/// subtraction; `gain < 1.0` partial subtraction to hedge against channel
/// variation that would otherwise leave a negative residual.
#[inline]
pub fn subtract_signal_weighted(audio: &mut [i16], result: &DecodeResult, gain: f32) {
    let tones = message_to_tones(&result.message77);
    subtract_tones(audio, &tones, result.freq_hz, result.dt_sec, gain, &FT8_CFG);
}

#[cfg(test)]
mod tests {
    use super::super::decode::{DecodeDepth, DecodeStrictness};
    use super::super::wave_gen::{message_to_tones, tones_to_i16};
    use super::*;

    #[test]
    fn subtract_reduces_power() {
        let msg = [0u8; 77];
        let itone = message_to_tones(&msg);
        let samples = tones_to_i16(&itone, 1000.0, 20_000);

        let mut audio = vec![0i16; 180_000];
        let offset = 6_000usize;
        let len = samples.len().min(180_000 - offset);
        audio[offset..offset + len].copy_from_slice(&samples[..len]);

        let power_before: f32 =
            audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>() / audio.len() as f32;

        let result = DecodeResult {
            message77: msg,
            freq_hz: 1000.0,
            dt_sec: 0.0,
            hard_errors: 0,
            sync_score: 10.0,
            pass: 0,
            sync_cv: 0.0,
            snr_db: 0.0,
        };

        subtract_signal(&mut audio, &result);

        let power_after: f32 =
            audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>() / audio.len() as f32;

        assert!(
            power_after < power_before * 0.10,
            "power before={power_before:.1} after={power_after:.1}"
        );
    }

    #[test]
    fn subtract_with_exact_timing_near_zero() {
        let msg = [1u8; 77];
        let itone = message_to_tones(&msg);
        let samples = tones_to_i16(&itone, 1000.0, 20_000);

        let mut audio = vec![0i16; 180_000];
        let offset = 6_000usize;
        let len = samples.len().min(180_000 - offset);
        audio[offset..offset + len].copy_from_slice(&samples[..len]);

        let power_before: f32 = audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>();

        let result = DecodeResult {
            message77: msg,
            freq_hz: 1000.0,
            dt_sec: 0.0,
            hard_errors: 0,
            sync_score: 10.0,
            pass: 0,
            sync_cv: 0.0,
            snr_db: 0.0,
        };
        subtract_signal(&mut audio, &result);

        let power_after: f32 = audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>();
        assert!(
            power_after < power_before * 0.02,
            "power before={power_before:.0} after={power_after:.0}"
        );
    }

    #[test]
    fn subtract_reveals_hidden_signal() {
        use super::super::decode::decode_frame_subtract;

        let msg_strong = [0u8; 77];
        let itone_s = message_to_tones(&msg_strong);
        let strong = tones_to_i16(&itone_s, 1000.0, 20_000);

        let msg_weak = [1u8; 77];
        let itone_w = message_to_tones(&msg_weak);
        let weak = tones_to_i16(&itone_w, 1500.0, 3_000);

        let mut audio = vec![0i16; 180_000];
        let off = 6_000usize;
        let len = strong.len().min(180_000 - off);
        for i in 0..len {
            let v = strong[i] as i32 + weak[i] as i32;
            audio[off + i] = v.clamp(-32_768, 32_767) as i16;
        }

        let results = decode_frame_subtract(
            &audio,
            800.0,
            1700.0,
            1.0,
            None,
            DecodeDepth::BpAll,
            50,
            DecodeStrictness::Normal,
        );
        let found_strong = results.iter().any(|r| r.message77 == msg_strong);
        let found_weak = results.iter().any(|r| r.message77 == msg_weak);
        assert!(found_strong, "strong signal not decoded");
        assert!(found_weak, "weak signal not decoded after subtract");
    }
}
