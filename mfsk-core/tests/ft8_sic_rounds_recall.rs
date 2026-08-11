//! Regression tests for issue #218's `.sic_rounds(n)`/`.sic_early()`
//! rename (was `.flat()`/`.staged()`) on the reference WAV
//! (`samples/FT8/210703_133430.wav` = `embedded-poc/assets/qso3_busy.wav`).
//!
//! 1. **`sic_rounds_3_matches_pre_refactor_flat_golden`** — `.sic_rounds(3)`
//!    (the new default-equivalent) must decode exactly the same 20-message
//!    set the pre-#218 `.flat()` did. Verified directly against a clean
//!    `main` worktree checkout at the commit this branch forked from
//!    (`1ef7ecf`) before this golden was written — not just asserted.
//! 2. **`sic_rounds_recall_is_monotonic`** — fewer rounds must never surface
//!    a decode a higher round count misses: `sic_rounds(1)`'s message set
//!    is a subset of `sic_rounds(2)`'s, which is a subset of
//!    `sic_rounds(3)`'s. Guards the round-count knob itself, independent of
//!    the exact golden list above (which will drift as unrelated SIC-quality
//!    work lands; the subset relationship should not).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_sic_rounds_recall -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;

use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Captured 2026-07-29 from both a pre-#218 `main` worktree's `.flat()`
/// and this branch's `.sic_rounds(3)` on the same WAV/params — identical
/// 20-entry sets confirmed byte-for-byte before this golden was written.
const SIC_ROUNDS_3_GOLDEN: &[&str] = &[
    "A92EE F5PSR -14",
    "CQ EA2BFM IN83",
    "CQ F5RXL IN94",
    "K1BZM DK8NE -10",
    "K1BZM EA3CJ JN01",
    "K1BZM EA3GP -09",
    "K1JT EA3AGB -15",
    "K1JT HA0DU KN07",
    "K1JT HA5WA 73",
    "KD2UGC F6GCP R-23",
    "N1API F2VX 73",
    "N1API HA6FQ -23",
    "N1JFU EA6EE R-07",
    "N1PJT HB9CQK -10",
    "W0RSJ EA3BMU RR73",
    "W1DIG SV9CVY -14",
    "W1FC F5BZB -08",
    "WA2FZW DL5AXX RR73",
    "WM3PEN EA6VQ -09",
    "XE2X HA2NP RR73",
];

fn decode_messages(audio: &[i16], n_rounds: usize) -> BTreeSet<String> {
    DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 60)
        .sic_rounds(n_rounds)
        .decode()
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect()
}

#[test]
fn sic_rounds_3_matches_pre_refactor_flat_golden() {
    let audio = load_wav_i16(QSO3_PATH);
    let got = decode_messages(&audio, 3);
    let want: BTreeSet<String> = SIC_ROUNDS_3_GOLDEN.iter().map(|s| s.to_string()).collect();
    assert_eq!(
        got, want,
        "sic_rounds(3) diverged from the pre-#218 .flat() golden"
    );
}

#[test]
fn sic_rounds_recall_is_monotonic() {
    let audio = load_wav_i16(QSO3_PATH);
    let r1 = decode_messages(&audio, 1);
    let r2 = decode_messages(&audio, 2);
    let r3 = decode_messages(&audio, 3);

    assert!(
        r1.is_subset(&r2),
        "sic_rounds(1) found a decode sic_rounds(2) missed: {:?}",
        r1.difference(&r2).collect::<Vec<_>>()
    );
    assert!(
        r2.is_subset(&r3),
        "sic_rounds(2) found a decode sic_rounds(3) missed: {:?}",
        r2.difference(&r3).collect::<Vec<_>>()
    );
    assert!(
        r1.len() <= r2.len() && r2.len() <= r3.len(),
        "recall not monotonic: r1={} r2={} r3={}",
        r1.len(),
        r2.len(),
        r3.len()
    );
}
