//! FT8-specific integration tests for the resampler.
//!
//! The resampler itself lives in `mfsk-core::dsp::resample` (pure DSP, no
//! protocol knowledge). These tests exercise the end-to-end path
//! `arbitrary-rate PCM → resample → FT8 decoder` which can only be expressed
//! in a crate that depends on `ft8-core::decode`.

use mfsk_core::ft8::decode::{DecodeDepth, decode_frame};
use mfsk_core::ft8::params::{MSG_BITS, NMAX};
use mfsk_core::ft8::resample::{resample_f32_to_12k, resample_to_12k};
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_f32};

/// Generate a 12 kHz FT8 frame with signal + AWGN noise.
fn make_noisy_frame(msg: &[u8; 77], freq: f32, snr_db: f32) -> Vec<i16> {
    let _ = MSG_BITS;
    let itone = message_to_tones(msg);
    let pcm = tones_to_f32(&itone, freq, 1.0);

    let pad = 6000usize;
    let mut audio = vec![0.0f32; NMAX];
    for (i, &s) in pcm.iter().enumerate() {
        if pad + i < NMAX {
            audio[pad + i] = s;
        }
    }

    let noise_std = (0.707 * 10.0_f64.powf(-snr_db as f64 / 20.0)) as f32;
    let mut rng_state = 0x12345678u64;
    for s in audio.iter_mut() {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u1 = (rng_state >> 33) as f32 / (1u64 << 31) as f32;
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u2 = (rng_state >> 33) as f32 / (1u64 << 31) as f32;
        let u1c = u1.max(1e-10);
        let gauss = (-2.0 * u1c.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos();
        *s += noise_std * gauss;
    }

    audio
        .iter()
        .map(|&s| (s * 20000.0).clamp(-32768.0, 32767.0) as i16)
        .collect()
}

/// Linear upsampler used to stage inputs at arbitrary rates before handing them
/// to the production resampler. Not a production codepath; test-only.
fn upsample(audio_12k: &[i16], target_rate: u32) -> Vec<i16> {
    let ratio = target_rate as f64 / 12000.0;
    let out_len = (audio_12k.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;
        if idx + 1 < audio_12k.len() {
            let v =
                audio_12k[idx] as f64 + (audio_12k[idx + 1] as f64 - audio_12k[idx] as f64) * frac;
            out.push(v.round() as i16);
        } else if idx < audio_12k.len() {
            out.push(audio_12k[idx]);
        }
    }
    out
}

#[test]
fn resample_decode_48k_weak_signal() {
    let msg = [1u8; MSG_BITS];
    let audio_12k = make_noisy_frame(&msg, 1000.0, -18.0);

    let audio_48k = upsample(&audio_12k, 48000);
    let resampled = resample_to_12k(&audio_48k, 48000);
    assert!((resampled.len() as i32 - NMAX as i32).abs() <= 1);

    let results = decode_frame(
        &resampled,
        800.0,
        1200.0,
        1.0,
        None,
        DecodeDepth::BpAllOsd,
        50,
    );
    assert!(
        !results.is_empty(),
        "resample 48k decode failed at -18 dB SNR"
    );
    assert_eq!(results[0].message77, msg);
}

#[test]
fn resample_f32_decode_48k_weak_signal() {
    let msg = [1u8; MSG_BITS];
    let audio_12k_i16 = make_noisy_frame(&msg, 1000.0, -18.0);
    let audio_48k_i16 = upsample(&audio_12k_i16, 48000);
    let audio_48k_f32: Vec<f32> = audio_48k_i16.iter().map(|&s| s as f32 / 32768.0).collect();

    let resampled = resample_f32_to_12k(&audio_48k_f32, 48000);
    assert!((resampled.len() as i32 - NMAX as i32).abs() <= 1);

    let results = decode_frame(
        &resampled,
        800.0,
        1200.0,
        1.0,
        None,
        DecodeDepth::BpAllOsd,
        50,
    );
    assert!(
        !results.is_empty(),
        "f32 resample 48k decode failed at -18 dB SNR"
    );
    assert_eq!(results[0].message77, msg);
}

#[test]
fn resample_decode_44100_weak_signal() {
    let msg = [1u8; MSG_BITS];
    let audio_12k = make_noisy_frame(&msg, 1000.0, -18.0);

    let audio_44k = upsample(&audio_12k, 44100);
    let resampled = resample_to_12k(&audio_44k, 44100);
    assert!((resampled.len() as i32 - NMAX as i32).abs() <= 2);

    let results = decode_frame(
        &resampled,
        800.0,
        1200.0,
        1.0,
        None,
        DecodeDepth::BpAllOsd,
        50,
    );
    assert!(
        !results.is_empty(),
        "resample 44100 decode failed at -18 dB SNR"
    );
    assert_eq!(results[0].message77, msg);
}
