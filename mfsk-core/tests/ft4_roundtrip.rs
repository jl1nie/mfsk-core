//! FT4 encode → decode round-trip on noiseless synthetic audio.
//!
//! Builds a known 77-bit message, synthesises the FT4 waveform, drops it
//! into a 7.5 s slot at a moderate amplitude, then decodes. If the trait
//! wiring (modulation params, frame layout, GFSK shaping, sync patterns,
//! LDPC, message codec) is correct end-to-end, the first decoded message
//! should equal the input bit-for-bit.

use mfsk_core::core::{FrameLayout, MessageCodec, MessageFields, ModulationParams};
use mfsk_core::ft4::{Ft4, decode, encode};

const NSPS: usize = <Ft4 as ModulationParams>::NSPS as usize; // 576
const NN: usize = <Ft4 as FrameLayout>::N_SYMBOLS as usize; // 103

/// 7.5 s slot @ 12 kHz = 90 000 samples.
const SLOT_SAMPLES: usize = 90_000;

/// Pack a standard CQ message via the WSJT 77-bit codec so the payload
/// unpacks to a valid string (arbitrary 1/0 bits fail `unpack77`).
fn pack_cq(call: &str, grid: &str) -> [u8; 77] {
    let codec = mfsk_core::msg::Wsjt77Message;
    let bits = codec
        .pack(&MessageFields {
            call1: Some("CQ".into()),
            call2: Some(call.into()),
            grid: Some(grid.into()),
            ..MessageFields::default()
        })
        .expect("pack_cq succeeds");
    let mut out = [0u8; 77];
    out.copy_from_slice(&bits);
    out
}

fn build_slot(msg77: &[u8; 77], freq_hz: f32, peak_i16: i16) -> Vec<i16> {
    let itone = encode::message_to_tones(msg77);
    assert_eq!(itone.len(), NN);
    let pcm = encode::tones_to_i16(&itone, freq_hz, peak_i16);
    assert_eq!(pcm.len(), NN * NSPS);

    // Place at the nominal 0.5 s frame-start offset inside a 7.5 s slot.
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize; // 6000
    let len = pcm.len().min(audio.len() - pad);
    audio[pad..pad + len].copy_from_slice(&pcm[..len]);
    audio
}

#[test]
fn encode_decode_clean_signal_1000hz() {
    let msg = pack_cq("JA1ABC", "PM95");
    let audio = build_slot(&msg, 1000.0, 20_000);
    let results = decode::decode_frame(&audio, 800.0, 1200.0, 1.0, 50);
    assert!(
        !results.is_empty(),
        "FT4 decode produced no results for clean 1000 Hz signal"
    );
    let got = results
        .iter()
        .find(|r| r.message77() == msg)
        .expect("no result matches transmitted payload");
    // Verify the decoded payload also unpacks to the expected human-readable
    // text — confirms the full trait chain (FEC → MessageCodec::unpack).
    let codec = mfsk_core::msg::Wsjt77Message;
    let ctx = mfsk_core::core::DecodeContext::default();
    let text = codec
        .unpack(got.message77(), &ctx)
        .expect("unpack returns a valid text");
    assert!(
        text.contains("CQ") && text.contains("JA1ABC"),
        "text = '{text}'"
    );
}

#[test]
fn encode_decode_mid_band_1500hz() {
    let msg = pack_cq("W1AW", "FN42");
    let audio = build_slot(&msg, 1500.0, 20_000);
    let results = decode::decode_frame(&audio, 1200.0, 1800.0, 1.0, 50);
    assert!(!results.is_empty());
    assert!(results.iter().any(|r| r.message77() == msg));
}

#[test]
fn tone_sequence_length_matches_frame() {
    let msg = pack_cq("JA1ABC", "PM95");
    let itone = encode::message_to_tones(&msg);
    assert_eq!(itone.len(), NN);
    for &t in itone.iter() {
        assert!(t < 4, "FT4 tone {t} must be 0..=3");
    }
}

#[test]
fn costas_patterns_correct_in_emitted_tones() {
    let msg = pack_cq("JA1ABC", "PM95");
    let itone = encode::message_to_tones(&msg);
    let cases = [
        (0usize, [0u8, 1, 3, 2]),
        (33, [1, 0, 2, 3]),
        (66, [2, 3, 1, 0]),
        (99, [3, 2, 0, 1]),
    ];
    for (start, pattern) in cases {
        for (i, &expected) in pattern.iter().enumerate() {
            assert_eq!(
                itone[start + i],
                expected,
                "Costas mismatch at symbol {}",
                start + i
            );
        }
    }
}
