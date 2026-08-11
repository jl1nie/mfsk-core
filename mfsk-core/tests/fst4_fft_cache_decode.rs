//! `DecodeRequest::fft_cache` — FFT-cache-reuse verification for
//! FST4-60A. See `ft4_fft_cache_decode.rs`'s module doc for the full
//! rationale (same root cause, same reason a same-audio round-trip
//! can't prove reuse, same differential-test strategy).
//!
//! FST4 has no SIC path (issue #193), so only the single-pass strategy
//! applies here.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use mfsk_core::fst4::Fst4s60;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const GOLDEN_MSG: &str = "CQ N5TM EL29"; // see fst4_wsjtx_samples.rs

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    )
}

fn recovers_golden(results: &[mfsk_core::fst4::decode::DecodeResult]) -> bool {
    results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .any(|m| m == GOLDEN_MSG)
}

#[test]
fn fst4_60_fft_cache_override_actually_used() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FST4 sample not found at \
             ../../WSJT-X/samples/FST4+FST4W/210115_0058.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let plain = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.0, 50)
        .decode()
        .results;
    assert!(
        recovers_golden(&plain),
        "expected the uncached decode to recover '{GOLDEN_MSG}'"
    );

    // A real (correctly-shaped) but *wrong* FftCache, built from silence
    // the same length as the golden WAV.
    let silence = vec![0i16; audio.len()];
    let wrong_cache = DecodeRequest::<Fst4s60>::new(&silence, 100.0, 3000.0, 1.0, 50)
        .decode()
        .fft_cache;

    let poisoned = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.0, 50)
        .fft_cache(wrong_cache)
        .decode()
        .results;
    assert!(
        !recovers_golden(&poisoned),
        "'.fft_cache(wrong_cache)' must actually be consumed: decoding real audio \
         against a cache built from silence still recovered '{GOLDEN_MSG}', which \
         means the supplied cache was silently ignored. decoded: {:?}",
        poisoned
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .collect::<Vec<_>>()
    );
}
