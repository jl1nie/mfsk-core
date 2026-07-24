//! Exact FT8 parity regression for the official WSJT-X sample.
//!
//! Reference:
//! - WSJT-X 3.0.0
//! - `jt9 -8 -d 3 210703_133430.wav`
//! - fixture SHA-256:
//!   `9feb99c275770a6618538026da7decc6b09eb6cf63121e5168fa86dcdf00c2f5`
//!
//! The fixture at `embedded-poc/assets/qso3_busy.wav` is byte-identical
//! to WSJT-X `samples/FT8/210703_133430.wav`.

#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

const WSJTX_3_MESSAGES: &[&str] = &[
    "A92EE F5PSR -14",
    "CQ DX DL8YHR JO41",
    "CQ EA2BFM IN83",
    "CQ F5RXL IN94",
    "K1BZM DK8NE -10",
    "K1BZM EA3CJ JN01",
    "K1BZM EA3GP -9",
    "K1JT EA3AGB -15",
    "K1JT HA0DU KN07",
    "K1JT HA5WA 73",
    "KD2UGC F6GCP R-23",
    "N1API F2VX 73",
    "N1API HA6FQ -23",
    "N1JFU EA6EE R-7",
    "N1PJT HB9CQK -10",
    "W0RSJ EA3BMU RR73",
    "W1DIG SV9CVY -14",
    "W1FC F5BZB -8",
    "WA2FZW DL5AXX RR73",
    "WM3PEN EA6VQ -9",
    "XE2X HA2NP RR73",
];

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "full parity regression is intended for --release"
)]
fn official_wsjtx_v3_sample_matches_all_21_messages() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let decoded = decode_block(&slot, 100.0, 3_000.0, 1.0, DecodeDepth::BpAllOsd, 200);
    let actual: BTreeSet<String> = decoded
        .iter()
        .filter_map(|result| unpack77(&result.message77))
        .collect();
    let expected: BTreeSet<String> = WSJTX_3_MESSAGES
        .iter()
        .map(|message| (*message).to_owned())
        .collect();

    let missing: Vec<&String> = expected.difference(&actual).collect();
    let unexpected: Vec<&String> = actual.difference(&expected).collect();
    assert!(
        missing.is_empty() && unexpected.is_empty(),
        "WSJT-X 3.0.0 parity mismatch: missing={missing:?}, unexpected={unexpected:?}"
    );
    assert_eq!(decoded.len(), 21, "decoder returned duplicate messages");
}
