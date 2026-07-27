//! Direct before/after check for the issue #177 subtract fix
//! (freq refine + 3x iterative subtract in the staged SIC engine).
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeStrictness};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn check_decode_frame_subtract_full_qso3() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
        .depth(DecodeDepth::FULL)
        .strictness(DecodeStrictness::Normal)
        .staged()
        .decode()
        .results;
    let msgs: BTreeSet<String> = results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    println!("staged decode(qso3_busy.wav): {} decodes", msgs.len());
    for m in &msgs {
        println!("  {m}");
    }
}
