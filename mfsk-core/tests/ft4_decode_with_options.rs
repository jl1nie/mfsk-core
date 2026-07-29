//! Integration test for `DecodeRequest<Ft4>::osd` (issue #191 rewrite
//! of the original PR #21 test, which covered
//! `ft4::decode::decode_frame_with_options` before that function family
//! was consolidated into `DecodeRequest`).
//!
//! The PR ships an inline unit test that iterates `DecodeDepth` rungs
//! against a *silent* buffer, which catches signature drift but doesn't
//! validate that the parameter actually flows through the decode
//! pipeline. This test synthesises a clean FT4 signal and asserts every
//! rung decodes it back, so a future refactor that silently drops
//! `osd` on the floor would break the assertion.

use mfsk_core::engine::{FrameLayout, MessageCodec, MessageFields};
use mfsk_core::ft4::{Ft4, encode};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::{Wsjt77Message, wsjt77};

const NN: usize = <Ft4 as FrameLayout>::N_SYMBOLS as usize;
const SLOT_SAMPLES: usize = 90_000;

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
    let itone = encode::message_to_tones(msg77);
    assert_eq!(itone.len(), NN);
    let pcm = encode::tones_to_i16(&itone, freq_hz, peak_i16);
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let len = pcm.len().min(audio.len() - pad);
    audio[pad..pad + len].copy_from_slice(&pcm[..len]);
    audio
}

#[test]
fn every_depth_decodes_clean_signal() {
    let msg = pack_msg("CQ", "K1ABC", "FN42");
    let audio = synth_slot(&msg, 1500.0, 25_000);

    let mut decoded_text = None;
    for osd in [false, true] {
        let results = DecodeRequest::<Ft4>::new(&audio, 100.0, 3000.0, 0.6, 5)
            .osd(osd)
            .decode()
            .results;
        let hit = results
            .iter()
            .find(|r| r.message77() == msg)
            .unwrap_or_else(|| {
                panic!(
                    "no clean-signal decode for osd={:?} (got {} results)",
                    osd,
                    results.len()
                )
            });
        // Every rung on a clean signal should produce a CRC-valid
        // payload that unpacks to the input string.
        let m77: [u8; 77] = hit.message77().try_into().expect("message77 is 77 bits");
        let text = wsjt77::unpack77(&m77).unwrap_or_default();
        if decoded_text.is_none() {
            decoded_text = Some(text.clone());
        }
        assert_eq!(
            text,
            decoded_text.as_deref().unwrap(),
            "osd={:?} produced a different decode",
            osd
        );
    }
    assert!(decoded_text.unwrap_or_default().contains("K1ABC"));
}
