//! `DecodeRequest::on_result` — streaming decode-callback verification
//! for FST4-60A, against the real WSJT-X golden WAV
//! (`WSJT-X/samples/FST4+FST4W/210115_0058.wav`).
//!
//! Companion to `ft8_streaming_decode.rs`/`ft4_streaming_decode.rs` —
//! see `ft4_streaming_decode.rs`'s module doc for why this coverage
//! gap existed (FST4 shares the same `DecodeRequest<P>::on_result`
//! field FT4 does, and was silently a no-op the same way).
//!
//! FST4 has no SIC path (issue #193 — no `SubtractCfg` exists yet),
//! so only the single contract applies here: the default
//! `par_iter()`-parallel single-pass strategy, callback fires inside
//! each candidate's closure before the cross-candidate dedup pass —
//! a documented possible superset of the batch result.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::Mutex;

use mfsk_core::fst4::Fst4s60;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    )
}

#[test]
fn fst4_60_streaming_single_pass_superset_of_batch() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FST4 sample not found at \
             ../../WSJT-X/samples/FST4+FST4W/210115_0058.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::fst4::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let outcome = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.0, 50)
        .on_result(&on_result)
        .decode();

    let batch: BTreeSet<String> = outcome
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    let streamed: BTreeSet<String> = streamed.into_inner().unwrap().into_iter().collect();

    println!(
        "batch: {} decode(s), streamed: {} callback(s)",
        batch.len(),
        streamed.len()
    );

    let missing_from_stream: Vec<&String> = batch.difference(&streamed).collect();
    assert!(
        missing_from_stream.is_empty(),
        "every batch result must have fired via callback at least once: missing {:?}",
        missing_from_stream
    );
    assert!(
        !batch.is_empty(),
        "expected real decodes on the FST4-60A golden WAV"
    );
}
