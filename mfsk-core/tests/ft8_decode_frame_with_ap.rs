//! Integration test for `DecodeRequest::ap_hint` (issue #191 rewrite of
//! the original PR #22 test, which covered `ft8::decode::decode_frame_with_ap`
//! before that function family was consolidated into `DecodeRequest`).
//!
//! Covers (a) clean self-synth round-trip with a matching hint and (b) a
//! deliberately *wrong* AP hint does not corrupt a clean-signal decode —
//! the post-FEC CRC must catch any AP-locked spurious convergence, which
//! is the safety claim made in the original PR description.

use mfsk_core::engine::{MessageCodec, MessageFields};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::ApHint;
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::{Wsjt77Message, wsjt77};

fn pack_msg(call1: &str, call2: &str, grid: &str) -> [u8; 77] {
    let bits = Wsjt77Message
        .pack(&MessageFields {
            call1: Some(call1.into()),
            call2: Some(call2.into()),
            grid: Some(grid.into()),
            ..MessageFields::default()
        })
        .expect("pack succeeds");
    let mut out = [0u8; 77];
    out.copy_from_slice(&bits);
    out
}

fn synth_slot(msg77: &[u8; 77], freq_hz: f32, peak_i16: i16) -> Vec<i16> {
    let itone = message_to_tones(msg77);
    let pcm = tones_to_i16(&itone, freq_hz, peak_i16);
    let mut audio = vec![0i16; 180_000];
    let offset = 6_000usize;
    let len = pcm.len().min(audio.len() - offset);
    audio[offset..offset + len].copy_from_slice(&pcm[..len]);
    audio
}

fn first_text_at(
    results: &[mfsk_core::ft8::decode::DecodeResult],
    target: [u8; 77],
) -> Option<String> {
    let r = results.iter().find(|r| r.message77() == target)?;
    wsjt77::unpack77(r.message77())
}

#[test]
fn matching_ap_hint_decodes_clean_signal() {
    let msg = pack_msg("CQ", "K1ABC", "FN42");
    let audio = synth_slot(&msg, 1500.0, 25_000);
    let ap = ApHint::new().with_call1("CQ").with_call2("K1ABC");

    let results = DecodeRequest::<Ft8>::new(&audio, 300.0, 2700.0, 1.5, 15)
        .ap_hint(&ap)
        .decode()
        .results;
    assert!(
        first_text_at(&results, msg)
            .unwrap_or_default()
            .contains("K1ABC"),
        "matching AP hint did not produce the expected decode"
    );
}

#[test]
fn wrong_ap_hint_does_not_corrupt_clean_decode() {
    // PR claim (paraphrased): "When the hint is wrong, decode quality
    // degrades only slightly because the AP path is gated behind
    // sync-quality + BP score checks; spurious AP-locked decodes are
    // caught by the post-FEC CRC." → wrong hint must not produce a
    // CRC-passing wrong message in place of the right one.
    let msg = pack_msg("CQ", "K1ABC", "FN42");
    let audio = synth_slot(&msg, 1500.0, 25_000);

    // Hint targets a totally unrelated callsign pair.
    let wrong = ApHint::new().with_call1("CQ").with_call2("3Y0Z");

    let results_wrong = DecodeRequest::<Ft8>::new(&audio, 300.0, 2700.0, 1.5, 15)
        .ap_hint(&wrong)
        .decode()
        .results;

    // Either the right message survives, or no decode happens at the
    // target frequency — but we must NOT see a "K1ABC" frame mutate
    // into a "3Y0Z" CRC-passing decode.
    let bad_3y0z = results_wrong.iter().any(|r| {
        wsjt77::unpack77(r.message77())
            .map(|t| t.contains("3Y0Z"))
            .unwrap_or(false)
    });
    assert!(
        !bad_3y0z,
        "wrong AP hint induced a CRC-passing 3Y0Z decode where K1ABC was transmitted"
    );
}
