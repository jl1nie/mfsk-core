//! FT8-specific integration tests for the resampler.
//!
//! The resampler itself lives in `mfsk-engine::dsp::resample` (pure DSP, no
//! protocol knowledge). These tests exercise the end-to-end path
//! `arbitrary-rate PCM → resample → FT8 decoder` which can only be expressed
//! in a crate that depends on `ft8-core::decode`.

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::params::{MSG_BITS, NMAX};
use mfsk_core::ft8::resample::{resample_f32_to_12k, resample_to_12k};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::pack77;

/// Valid FT8 standard message used by all resample round-trip tests.
/// Post v0.6.1 the host pipeline routes through
/// `decode_block::process_one_candidate_inner` which gates on a
/// successful unpack77 + plausibility check, so an arbitrary
/// `[1u8; 77]` no longer round-trips. Use a real message instead.
fn test_msg() -> [u8; 77] {
    pack77("CQ", "JA1ABC", "PM95").expect("pack77")
}

/// Generate a 12 kHz FT8 frame with signal + AWGN noise.
fn make_noisy_frame(msg: &[u8; 77], freq: f32, snr_db: f32) -> Vec<i16> {
    let _ = MSG_BITS;
    let itone = mfsk_core::engine::tx::message_to_tones::<mfsk_core::ft8::Ft8>(msg);
    let pcm = mfsk_core::engine::tx::synthesize::<mfsk_core::ft8::Ft8>(&itone, 12_000, freq, 1.0);

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
    let _ = MSG_BITS;
    let msg = test_msg();
    let audio_12k = make_noisy_frame(&msg, 1000.0, -18.0);

    let audio_48k = upsample(&audio_12k, 48000);
    let resampled = resample_to_12k(&audio_48k, 48000);
    assert!((resampled.len() as i32 - NMAX as i32).abs() <= 1);

    let results = DecodeRequest::<Ft8>::new(&resampled, 800.0, 1200.0, 1.0, 50)
        .decode()
        .results;
    assert!(
        !results.is_empty(),
        "resample 48k decode failed at -18 dB SNR"
    );
    assert_eq!(*results[0].message77(), msg);
}

#[test]
fn resample_f32_decode_48k_weak_signal() {
    let _ = MSG_BITS;
    let msg = test_msg();
    let audio_12k_i16 = make_noisy_frame(&msg, 1000.0, -18.0);
    let audio_48k_i16 = upsample(&audio_12k_i16, 48000);
    let audio_48k_f32: Vec<f32> = audio_48k_i16.iter().map(|&s| s as f32 / 32768.0).collect();

    let resampled = resample_f32_to_12k(&audio_48k_f32, 48000);
    assert!((resampled.len() as i32 - NMAX as i32).abs() <= 1);

    let results = DecodeRequest::<Ft8>::new(&resampled, 800.0, 1200.0, 1.0, 50)
        .decode()
        .results;
    assert!(
        !results.is_empty(),
        "f32 resample 48k decode failed at -18 dB SNR"
    );
    assert_eq!(*results[0].message77(), msg);
}

#[test]
fn resample_decode_44100_weak_signal() {
    let _ = MSG_BITS;
    let msg = test_msg();
    let audio_12k = make_noisy_frame(&msg, 1000.0, -18.0);

    let audio_44k = upsample(&audio_12k, 44100);
    let resampled = resample_to_12k(&audio_44k, 44100);
    assert!((resampled.len() as i32 - NMAX as i32).abs() <= 2);

    let results = DecodeRequest::<Ft8>::new(&resampled, 800.0, 1200.0, 1.0, 50)
        .decode()
        .results;
    assert!(
        !results.is_empty(),
        "resample 44100 decode failed at -18 dB SNR"
    );
    assert_eq!(*results[0].message77(), msg);
}

/// #576. The tests above build the signal and the noise at 12 kHz and only
/// then raise the rate, so the input has nothing above 6 kHz to fold, and
/// they passed while 48 kHz → 12 kHz was a plain decimation by 4. Here the
/// noise is white over the whole 48 kHz band, as from a microphone or a
/// sound card, at -19 dB in 2500 Hz. Measured over 16 trials a cell, the
/// unfiltered decimation decoded 0 of 16 at -19 dB (50 % at -15.3 dB) and
/// `fil4` 16 of 16 (50 % at -21.1 dB; -21.0 dB with signal and noise
/// generated at 12 kHz).
#[test]
fn resample_48k_full_band_noise_does_not_fold_into_the_band() {
    use mfsk_core::decoder::{Decoder, SlotInput};
    use mfsk_core::engine::tx::{message_to_tones, synthesize};

    struct Rng(u64);
    impl Rng {
        fn u(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        }
        fn gauss(&mut self) -> f64 {
            let (a, b) = (self.u(), self.u());
            (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
        }
    }

    let msg = test_msg();
    let tones = message_to_tones::<Ft8>(&msg);
    let want = mfsk_core::msg::wsjt77::unpack77(&msg).unwrap();
    // White noise of std `sigma` over 0..24 kHz puts sigma^2 * 2500/24000 in
    // 2500 Hz; a constant-envelope frame of amplitude `amp` carries amp^2/2.
    let sigma = 1_000.0f64;
    let snr_db = -19.0f64;
    let amp = (2.0 * sigma * sigma * 2_500.0 / 24_000.0 * 10f64.powf(snr_db / 10.0)).sqrt();

    let trials = 4;
    let mut decoded = 0;
    for t in 0..trials {
        let f0 = 1_000.0 + 61.0 * t as f32;
        let frame = synthesize::<Ft8>(&tones, 48_000, f0, amp as f32);
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ (t as u64 + 1));
        let x48: Vec<i16> = (0..48_000usize * 15)
            .map(|i| {
                let s = i
                    .checked_sub(24_000)
                    .and_then(|j| frame.get(j))
                    .map_or(0.0, |&v| v as f64);
                (s + sigma * rng.gauss()).round().clamp(-32_768.0, 32_767.0) as i16
            })
            .collect();
        let audio = resample_to_12k(&x48, 48_000);
        let rows = Decoder::<Ft8>::with_defaults()
            .decode(&SlotInput::i16(&audio))
            .rows;
        decoded += rows.iter().any(|r| r.decoded.text == want) as u32;
    }
    assert!(
        decoded >= 3,
        "{decoded} of {trials} decoded at {snr_db} dB with full-band noise at 48 kHz"
    );
}
