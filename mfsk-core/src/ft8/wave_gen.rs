// SPDX-License-Identifier: GPL-3.0-or-later
//! FT8 waveform generator.
//!
//! Encodes a 77-bit message into an 8-FSK baseband waveform at 12 000 Hz.
//! The pipeline mirrors WSJT-X `genft8.f90` / `encode174_91.f90`:
//!
//! ```text
//! message77  →  CRC-14  →  info91
//!            →  LDPC encode  →  codeword174
//!            →  Gray-map 3 bits/symbol  →  itone[79]
//!            →  phase accumulation  →  PCM f32 / i16
//! ```
use super::Ft8;
use super::{
    ldpc::osd::ldpc_encode,
    params::{LDPC_K, MSG_BITS, NN},
};

/// Append 14 CRC bits to a 77-bit message, producing 91 info bits. Uses the
/// shared CRC-14 implementation from mfsk-fec.
fn append_crc14(message77: &[u8; MSG_BITS]) -> [u8; LDPC_K] {
    let mut bytes = [0u8; 12];
    for (i, &bit) in message77.iter().enumerate() {
        bytes[i / 8] |= (bit & 1) << (7 - i % 8);
    }
    let crc = crate::fec::ldpc::crc14(&bytes);

    let mut info = [0u8; LDPC_K];
    info[..MSG_BITS].copy_from_slice(message77);
    for i in 0..14 {
        info[MSG_BITS + i] = ((crc >> (13 - i)) & 1) as u8;
    }
    info
}

/// Encode a 77-bit message into a 79-symbol FT8 tone sequence.
pub fn message_to_tones(message77: &[u8; MSG_BITS]) -> [u8; NN] {
    let info = append_crc14(message77);
    let cw = ldpc_encode(&info);
    let generic = crate::core::tx::codeword_to_itone::<Ft8>(&cw);
    let mut out = [0u8; NN];
    out.copy_from_slice(&generic);
    out
}

/// FT8 GFSK configuration: 12 kHz sample rate, 1920 samples/symbol (= 6.25 Hz
/// tone spacing), BT=2.0, modulation index 1.0, 240-sample raised-cosine ramp.
const FT8_GFSK: crate::core::dsp::gfsk::GfskCfg = crate::core::dsp::gfsk::GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 1920,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 1920 / 8,
};

/// Synthesise a 12 000 Hz f32 PCM waveform from an FT8 tone sequence.
///
/// Matches WSJT-X `gen_ft8wave.f90`: 3-symbol Gaussian pulse shape with
/// BT=2.0, dummy ramp-in/out symbols, and a half-cosine envelope on the
/// outermost `nsps/8` samples. Output length is `79 × 1920 = 151 680`.
#[inline]
pub fn tones_to_f32(itone: &[u8; NN], f0: f32, amplitude: f32) -> Vec<f32> {
    crate::core::dsp::gfsk::synth_f32(itone, f0, amplitude, &FT8_GFSK)
}

/// Synthesise and return a 16-bit PCM waveform. Peak value of the returned
/// signal equals `amplitude_i16` (0..32767).
#[inline]
pub fn tones_to_i16(itone: &[u8; NN], f0: f32, amplitude_i16: i16) -> Vec<i16> {
    crate::core::dsp::gfsk::synth_i16(itone, f0, amplitude_i16, &FT8_GFSK)
}

// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::super::params::NSPS;
    use super::*;

    /// Round-trip: generate a waveform and verify it decodes back to the same
    /// tone sequence (structural smoke-test only — no full decode).
    #[test]
    fn tone_sequence_length() {
        let msg = [0u8; MSG_BITS];
        let itone = message_to_tones(&msg);
        assert_eq!(itone.len(), NN);
    }

    #[test]
    fn all_tones_in_range() {
        let msg = [1u8; MSG_BITS]; // arbitrary non-zero message
        let itone = message_to_tones(&msg);
        for &t in itone.iter() {
            assert!(t < 8, "tone {t} out of range");
        }
    }

    #[test]
    fn costas_positions_correct() {
        use super::super::params::COSTAS;
        let msg = [0u8; MSG_BITS];
        let itone = message_to_tones(&msg);
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
        let msg = [0u8; MSG_BITS];
        let itone = message_to_tones(&msg);
        let pcm = tones_to_f32(&itone, 1000.0, 1.0);
        assert_eq!(pcm.len(), NN * NSPS);
    }

    /// Encode → decode round-trip via the full ft8-core pipeline (raw bits).
    #[test]
    fn encode_decode_roundtrip() {
        use super::super::decode::{DecodeDepth, decode_frame};

        // Build a known message (all bits = 1 is unlikely to collide with anything).
        let msg = [1u8; MSG_BITS];
        let itone = message_to_tones(&msg);

        // Strong noiseless signal at 1000 Hz.
        let pcm_f32 = tones_to_f32(&itone, 1000.0, 1.0);

        // Start at nominal 0.5 s into the frame — pad with 0.5 s of silence.
        let pad = vec![0.0f32; 6000];
        let signal: Vec<f32> = pad.iter().chain(pcm_f32.iter()).cloned().collect();
        let samples: Vec<i16> = signal.iter().map(|&s| (s * 20000.0) as i16).collect();

        // Pad to 180 000 samples.
        let mut audio = vec![0i16; 180_000];
        let len = samples.len().min(audio.len());
        audio[..len].copy_from_slice(&samples[..len]);

        let results = decode_frame(&audio, 800.0, 1200.0, 1.0, None, DecodeDepth::BpAll, 50);
        assert!(
            !results.is_empty(),
            "round-trip decode failed — no message found"
        );
        // The decoded message77 bits should match.
        assert_eq!(
            results[0].message77, msg,
            "decoded message77 does not match input"
        );
    }

    /// Full encode → decode round-trip with real FT8 callsigns.
    ///
    /// Tests the complete pipeline:
    ///   pack77 → message_to_tones → tones_to_f32 → decode_frame → unpack77
    ///
    /// Catches bugs in pack77/unpack77 that the raw-bit round-trip test misses,
    /// and verifies WSJT-X CRC compatibility (77-bit CRC, not 96-bit).
    #[test]
    fn callsign_roundtrip() {
        use super::super::decode::{DecodeDepth, decode_frame};
        use super::super::message::{pack77, unpack77};

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
            let itone = message_to_tones(&msg77);
            let pcm_f32 = tones_to_f32(&itone, 1000.0, 1.0);
            let pad = vec![0.0f32; 6000];
            let signal: Vec<f32> = pad.iter().chain(pcm_f32.iter()).cloned().collect();
            let samples: Vec<i16> = signal.iter().map(|&s| (s * 20000.0) as i16).collect();
            let mut audio = vec![0i16; 180_000];
            let n = samples.len().min(audio.len());
            audio[..n].copy_from_slice(&samples[..n]);

            let results = decode_frame(&audio, 800.0, 1200.0, 1.0, None, DecodeDepth::BpAll, 50);
            assert!(!results.is_empty(), "decode found nothing for: {expected}");

            let decoded = unpack77(&results[0].message77)
                .unwrap_or_else(|| panic!("unpack decoded bits failed for: {expected}"));
            assert_eq!(
                decoded, expected,
                "full roundtrip mismatch for: {call1} {call2} {report}"
            );
        }
    }
}
