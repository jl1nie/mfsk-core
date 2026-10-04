// SPDX-License-Identifier: GPL-3.0-only
//! FT8's GFSK configuration — what `Ft8`'s
//! [`crate::engine::tx::FskWaveform`] impl points at. The transmit chain
//! itself is generic since #391 ([`crate::engine::tx::message_to_tones`],
//! [`crate::engine::tx::synthesize`]) and mirrors WSJT-X
//! `genft8.f90` / `encode174_91.f90`:
//!
//! ```text
//! message77  →  CRC-14  →  info91
//!            →  LDPC encode  →  codeword174
//!            →  Gray-map 3 bits/symbol  →  itone[79]
//!            →  phase accumulation  →  PCM f32 / i16
//! ```
//!
//! ## Encoder-only example
//!
//! This module has no FFT dependency and no `std` requirement — it's the
//! TX-only path a `no_std + alloc` embedded transmitter links against
//! (decode needs an [`FftPlanner`](crate::engine::fft::FftPlanner) impl via
//! `fft-rustfft` or `fft-extern`; encode needs neither):
//!
//! ```
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
//! use mfsk_core::ft8::Ft8;
//! use mfsk_core::msg::wsjt77::pack77;
//!
//! let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack");
//! let tones = message_to_tones::<Ft8>(&msg77); // 79 Costas + data symbols
//! let pcm = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1500.0, /* amp */ 20_000);
//! assert_eq!(pcm.len(), tones.len() * 1920); // NSPS samples/symbol @ 12 kHz
//! # }
//! ```

/// FT8 GFSK configuration: 12 kHz sample rate, 1920 samples/symbol (= 6.25 Hz
/// tone spacing), BT=2.0, modulation index 1.0, 240-sample raised-cosine ramp.
/// Public so a transmitter can build a
/// [`GfskStream`](crate::engine::dsp::gfsk::GfskStream) with the same
/// configuration the batch entry points use — a streaming caller has
/// to name the config, and there must be exactly one FT8 answer to
/// what it is.
pub const FT8_GFSK: crate::engine::dsp::gfsk::GfskCfg = crate::engine::dsp::gfsk::GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 1920,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 1920 / 8,
};

// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::super::params::NSPS;

    /// Round-trip: generate a waveform and verify it decodes back to the same
    /// tone sequence (structural smoke-test only — no full decode).
    #[test]
    fn tone_sequence_length() {
        let msg = [0u8; 77];
        let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(&msg);
        assert_eq!(itone.len(), super::super::params::NN);
    }

    #[test]
    fn all_tones_in_range() {
        let msg = [1u8; 77]; // arbitrary non-zero message
        let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(&msg);
        for &t in itone.iter() {
            assert!(t < 8, "tone {t} out of range");
        }
    }

    #[test]
    fn costas_positions_correct() {
        use super::super::params::COSTAS;
        let msg = [0u8; 77];
        let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(&msg);
        for offset in [0usize, 36, 72] {
            for (i, &c) in COSTAS.iter().enumerate() {
                assert_eq!(
                    itone[offset + i],
                    c as u8,
                    "Costas mismatch at symbol {}",
                    offset + i
                );
            }
        }
    }

    #[test]
    fn waveform_length() {
        let msg = [0u8; 77];
        let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(&msg);
        let pcm = crate::engine::tx::synthesize::<crate::ft8::Ft8>(&itone, 12_000, 1000.0, 1.0);
        assert_eq!(pcm.len(), super::super::params::NN * NSPS);
    }

    /// Encode → decode round-trip via the full ft8-core pipeline (raw bits).
    /// Uses a valid FT8 standard message so the unpack77 + plausibility
    /// gate inside `process_one_candidate_inner` accepts the decoded
    /// codeword. (The old host pipeline emitted any CRC-converged
    /// codeword without unpack/plausibility checks; the inner unifies
    /// host with embedded by tightening to embedded's behaviour, so an
    /// arbitrary `[1u8; 77]` payload no longer round-trips.)
    #[test]
    fn encode_decode_roundtrip() {
        use super::super::Ft8;

        use super::super::message::pack77;
        use crate::msg::decode_request::DecodeRequest;

        // Build a valid FT8 standard message ("CQ JA1ABC PM95").
        let msg = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(&msg);

        // Strong noiseless signal at 1000 Hz.
        let pcm_f32 = crate::engine::tx::synthesize::<crate::ft8::Ft8>(&itone, 12_000, 1000.0, 1.0);

        // Start at nominal 0.5 s into the frame — pad with 0.5 s of silence.
        let pad = vec![0.0f32; 6000];
        let signal: Vec<f32> = pad.iter().chain(pcm_f32.iter()).cloned().collect();
        let samples: Vec<i16> = signal.iter().map(|&s| (s * 20000.0) as i16).collect();

        // Pad to 180 000 samples.
        let mut audio = vec![0i16; 180_000];
        let len = samples.len().min(audio.len());
        audio[..len].copy_from_slice(&samples[..len]);

        let results = DecodeRequest::<Ft8>::new(&audio, 800.0, 1200.0, 1.0, 50)
            .osd(false)
            .decode()
            .results;
        assert!(
            !results.is_empty(),
            "round-trip decode failed — no message found"
        );
        // The decoded message77 bits should match.
        assert_eq!(
            *results[0].message77(),
            msg,
            "decoded message77 does not match input"
        );
    }

    /// Full encode → decode round-trip with real FT8 callsigns.
    ///
    /// Tests the complete pipeline:
    ///   pack77 → message_to_tones → synthesize → decode_frame → unpack77
    ///
    /// Catches bugs in pack77/unpack77 that the raw-bit round-trip test misses,
    /// and verifies WSJT-X CRC compatibility (77-bit CRC, not 96-bit).
    #[test]
    fn callsign_roundtrip() {
        use super::super::Ft8;

        use super::super::message::{pack77, unpack77};
        use crate::msg::decode_request::DecodeRequest;

        let cases: &[(&str, &str, &str, &str)] = &[
            ("CQ", "JA1ABC", "PM95", "CQ JA1ABC PM95"),
            ("JA1ABC", "W1AW", "-15", "JA1ABC W1AW -15"),
            ("W1AW", "JA1ABC", "R-15", "W1AW JA1ABC R-15"),
            ("JA1ABC", "W1AW", "RR73", "JA1ABC W1AW RR73"),
            ("W1AW", "JA1ABC", "73", "W1AW JA1ABC 73"),
            ("CQ", "3Y0Z", "JD34", "CQ 3Y0Z JD34"),
        ];

        for &(call1, call2, report, expected) in cases {
            // 1. Pack message
            let msg77 = pack77(call1, call2, report)
                .unwrap_or_else(|| panic!("pack77 failed: {call1} {call2} {report}"));

            // 2. Verify pack → unpack consistency (no audio)
            let text =
                unpack77(&msg77).unwrap_or_else(|| panic!("unpack77 failed for: {expected}"));
            assert_eq!(text, expected, "pack/unpack mismatch");

            // 3. Full encode → decode with audio
            let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(&msg77);
            let pcm_f32 =
                crate::engine::tx::synthesize::<crate::ft8::Ft8>(&itone, 12_000, 1000.0, 1.0);
            let pad = vec![0.0f32; 6000];
            let signal: Vec<f32> = pad.iter().chain(pcm_f32.iter()).cloned().collect();
            let samples: Vec<i16> = signal.iter().map(|&s| (s * 20000.0) as i16).collect();
            let mut audio = vec![0i16; 180_000];
            let n = samples.len().min(audio.len());
            audio[..n].copy_from_slice(&samples[..n]);

            let results = DecodeRequest::<Ft8>::new(&audio, 800.0, 1200.0, 1.0, 50)
                .osd(false)
                .decode()
                .results;
            assert!(!results.is_empty(), "decode found nothing for: {expected}");

            let decoded = unpack77(results[0].message77())
                .unwrap_or_else(|| panic!("unpack decoded bits failed for: {expected}"));
            assert_eq!(
                decoded, expected,
                "full roundtrip mismatch for: {call1} {call2} {report}"
            );
        }
    }
}
