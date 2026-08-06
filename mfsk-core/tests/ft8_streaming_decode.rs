//! `DecodeRequest::on_result`/`SniperRequest::on_result` — streaming
//! decode-callback verification against the real WSJT-X golden WAV
//! (`qso3_busy.wav`), per `docs/reference/LIBRARY.md`'s "public decode
//! entry point" section and the design doc comment on
//! `DecodeRequest::on_result` itself.
//!
//! Two contracts to check, matching the two different delivery
//! guarantees `on_result`'s own doc comment documents:
//!
//! 1. **Sequential SIC path** (`.sic_rounds(n)`): callback-delivered
//!    messages must be *exactly* the batch result, same set, no more
//!    no less — the push point inside `sic_inner_passes_with_cache`
//!    *is* the final-acceptance point, so there's no divergence
//!    mechanism at all.
//! 2. **Parallel single-pass path** (`DecodeRequest::new(...)` with no
//!    `.sic_rounds()`/`.sic_early()`): callback-delivered messages must
//!    be a *superset* of the batch result — the callback fires inside
//!    each candidate's `par_iter()` closure, before the later
//!    cross-candidate dedup pass, so a same-message duplicate found by
//!    two different sync candidates could in principle fire twice via
//!    callback while only one survives into the final `Vec`. On the
//!    real golden WAV this test doesn't actually hit that duplicate
//!    case (verified equal, not just superset, below) — the assertion
//!    is a superset check because that's the documented contract, not
//!    because equality is expected to fail here.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8 \
//!     --test ft8_streaming_decode -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;
use std::sync::Mutex;

use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

#[test]
fn ft8_streaming_sic_rounds_matches_batch_exactly() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft8::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let outcome = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.3, 50)
        .sic_rounds(3)
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

    assert_eq!(
        streamed, batch,
        "sequential SIC path: streamed callback deliveries must exactly \
         match the batch result (no divergence mechanism exists on this path)"
    );
    // Sanity: this is a real decode, not a vacuously-true empty-set match.
    assert!(!batch.is_empty(), "expected real decodes on qso3_busy.wav");
}

#[test]
fn ft8_streaming_single_pass_superset_of_batch() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft8::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    // No .sic_rounds()/.sic_early() — default single-pass strategy,
    // parallelized via par_iter() under feature = "parallel".
    let outcome = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.3, 50)
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
    assert!(!batch.is_empty(), "expected real decodes on qso3_busy.wav");
    // On this golden WAV the parallel path happens not to hit the
    // documented same-message-two-candidates duplicate case — verify
    // that explicitly rather than silently allowing the weaker
    // superset check to mask a real regression elsewhere.
    assert_eq!(
        streamed, batch,
        "no duplicate-delivery case expected on this golden WAV \
         (if this starts failing, the superset contract is what's \
         guaranteed, not this equality — see this test's doc comment)"
    );
}
