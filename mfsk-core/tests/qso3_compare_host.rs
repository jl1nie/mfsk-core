//! Entry point of `scripts/qso3-compare.py`: `qso3_busy.wav` through the host decoders (`Decoder<Ft8>` at the three depths with AP off and full, and `decode_block` at the ship,
//! 30-candidate, BP-only and full-parity configurations), one line a decode, `H|config|message`, to the file named by `ZZ_OUT`. Asserts nothing: the recall floors of these
//! configurations are `ft8_qso3_apoff_recall` and `ft8_qso3_full_parity_recall`.
#![cfg(feature = "fft-rustfft")]
use std::path::Path;

use mfsk_core::decoder::{ApMode, DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;
#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

#[test]
#[ignore = "entry point of scripts/qso3-compare.py: asserts nothing"]
fn qso3_compare_host() {
    let out = std::env::var("ZZ_OUT").expect("ZZ_OUT: the file to write");
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let mut s = String::new();
    for (dn, d) in [
        ("d1", Depth::Fast),
        ("d2", Depth::Normal),
        ("d3", Depth::Deep),
    ] {
        for (an, ap) in [("apoff", ApMode::Off), ("apfull", ApMode::Full)] {
            let params = DecodeParams::for_band((200.0, 3000.0)).depth(d).ap(ap);
            for r in Decoder::<Ft8>::new(params)
                .decode(&SlotInput::i16(&slot))
                .rows
            {
                s += &format!("H|Decoder<Ft8> {dn} {an}|{}\n", r.decoded.text);
            }
        }
    }
    for (name, depth, sync_min, max_cand) in [
        (
            "decode_block EMBEDDED 1.3/15 (ship)",
            DecodeDepth::EMBEDDED,
            1.3f32,
            15usize,
        ),
        (
            "decode_block EMBEDDED 1.3/30",
            DecodeDepth::EMBEDDED,
            1.3,
            30,
        ),
        ("decode_block BP_ONLY 0.8/60", DecodeDepth::BP_ONLY, 0.8, 60),
        ("decode_block FULL 0.8/60 (OSD)", DecodeDepth::FULL, 0.8, 60),
    ] {
        for d in decode_block(&slot, 200.0, 3000.0, sync_min, depth, max_cand) {
            if let Some(t) = unpack77(d.message77()) {
                s += &format!("H|{name}|{t}\n");
            }
        }
    }
    std::fs::write(out, s).unwrap();
}
