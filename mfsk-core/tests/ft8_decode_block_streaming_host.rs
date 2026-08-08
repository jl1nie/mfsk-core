//! `ft8::decode_block::decode_block_streaming` — host `fft-rustfft`
//! sibling of `tests/ft8_decode_block_streaming.rs`'s embedded
//! no_std variant. Exercises `decode_block_multipass` (the same
//! 3-pass driver `decode_block`/`decode_block_with_ap` use) with an
//! `on_result` callback threaded through.
//!
//! Prior to issue #243's fix this driver could not safely expose a
//! streaming callback: the `xsnr2` SNR validity gate ran as a
//! post-hoc batch (first across the whole 3-pass loop, then —
//! mid-fix — per pass) *after* results had already been accepted
//! into the working set, so a result delivered via callback could
//! later be dropped or have its `snr_db` rewritten with no
//! revise/retract event to tell the callback so. The gate now runs
//! inline, immediately per candidate, before that candidate is ever
//! considered final — so this test asserts *exact* equality (same
//! contract as the embedded test), not merely a superset, matching
//! `decode_block_streaming`'s own doc comment.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full,internal-testing \
//!     --test ft8_decode_block_streaming_host -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{decode_block, decode_block_streaming};

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

#[test]
fn decode_block_streaming_matches_decode_block_exactly() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let batch = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::FULL, 50);

    let mut streamed = Vec::new();
    let streamed_count =
        decode_block_streaming(&slot, 100.0, 3000.0, 1.3, DecodeDepth::FULL, 50, &mut |r| {
            streamed.push(r.message77().to_vec())
        })
        .len();

    println!(
        "batch: {} decode(s), streaming return: {} decode(s), callback firings: {}",
        batch.len(),
        streamed_count,
        streamed.len()
    );

    // `decode_block_multipass` is fully sequential (no rayon inside
    // this function — unlike `DecodeRequest`'s single-pass/sniper
    // strategies) and the xsnr2 gate is now atomic per candidate
    // (issue #243), so callback firings must exactly match the
    // returned Vec, in the same order, and both must exactly match
    // plain `decode_block`'s own output.
    let batch_msgs: Vec<Vec<u8>> = batch.iter().map(|r| r.message77().to_vec()).collect();
    assert_eq!(
        streamed, batch_msgs,
        "streamed callback order/content must exactly match decode_block's own output"
    );
    assert_eq!(streamed_count, batch.len());
    assert!(!batch.is_empty(), "expected real decodes on qso3_busy.wav");
}

/// SNR values delivered via callback must be the *final* post-gate
/// `xsnr2` values, not a pre-gate/pre-clamp intermediate — i.e. the
/// callback must fire after the gate has already run, not before.
/// Cross-checks against the batch result's own `snr_db` for the same
/// message.
#[test]
fn decode_block_streaming_snr_matches_batch() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let batch = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::FULL, 50);
    let batch_snr: std::collections::BTreeMap<Vec<u8>, f32> = batch
        .iter()
        .map(|r| (r.message77().to_vec(), r.snr_db))
        .collect();

    let mut streamed_snr: std::collections::BTreeMap<Vec<u8>, f32> =
        std::collections::BTreeMap::new();
    decode_block_streaming(&slot, 100.0, 3000.0, 1.3, DecodeDepth::FULL, 50, &mut |r| {
        streamed_snr.insert(r.message77().to_vec(), r.snr_db);
    });

    assert_eq!(
        streamed_snr, batch_snr,
        "streamed snr_db must match batch's"
    );
    assert!(
        !batch_snr.is_empty(),
        "expected real decodes on qso3_busy.wav"
    );
}
