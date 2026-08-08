//! `DecodeRequest::on_result`/`SniperRequest::on_result` — streaming
//! decode-callback verification against the real WSJT-X golden WAV
//! (`qso3_busy.wav`), per `docs/reference/LIBRARY.md`'s "public decode
//! entry point" section and the design doc comment on
//! `DecodeRequest::on_result` itself.
//!
//! Two contracts to check, matching the two different delivery
//! guarantees `on_result`'s own doc comment documents:
//!
//! 1. **Sequential SIC path** (`.sic_rounds(n)`/`.sic_early()`):
//!    callback-delivered messages must be *exactly* the batch result,
//!    same set, no more no less — the push point inside
//!    `sic_inner_passes_with_cache` *is* the final-acceptance point,
//!    so there's no divergence mechanism at all. This *used* to be
//!    false for `.sic_early()` when combined with `.known(...)` — see
//!    `ft8_streaming_sic_early_with_known_matches_batch_exactly`
//!    below (issue #243 follow-up: a *different* post-hoc gate than
//!    the one #243 itself fixed, in `SupportsSicEarly::__staged_sic`
//!    rather than `decode_block::decode_block_multipass`).
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
fn ft8_streaming_sic_early_matches_batch_exactly() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft8::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let outcome = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.3, 50)
        .sic_early()
        .on_result(&on_result)
        .decode();

    let batch: BTreeSet<String> = outcome
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    let streamed: BTreeSet<String> = streamed.into_inner().unwrap().into_iter().collect();

    assert_eq!(
        streamed, batch,
        "sic_early: streamed callback deliveries must exactly match the batch result"
    );
    assert!(!batch.is_empty(), "expected real decodes on qso3_busy.wav");
}

/// Regression for the bug reported against b087902: `.sic_early()`
/// combined with `.known(...)` (a plausible two-phase pipeline shape —
/// re-decode the same audio seeded with an earlier phase's results,
/// looking for more in the residual) could fire `on_result` for a
/// candidate that then never appeared in the returned `Vec`.
///
/// Root cause: `SupportsSicEarly::__staged_sic` subtracted `known`
/// from the audio up front, but never threaded `known` into any
/// checkpoint's own message77 dedup — instead relying on a *post-hoc*
/// `results.retain(...)` after `decode_frame_subtract_staged_with_ap_inner`
/// had already fired `on_result` for every checkpoint's raw
/// candidates. When subtraction of a `known` signal left enough
/// residual for a checkpoint to independently re-decode that same
/// message (real for marginal/high-hard-error signals — this WAV's
/// `K1BZM DK8NE -10` and `XE2X HA2NP RR73` both reproduced it, at
/// `hard_errors` 20 and 16 respectively), the callback fired and the
/// retain then silently dropped the result — a revoke-less retract of
/// exactly the kind issue #243 closed on the `decode_block` engine,
/// just in a different function. Fixed by threading `known` into
/// every checkpoint's dedup so it gates *before* the subtract/
/// callback point, same as every other `sic_inner_passes` caller.
#[test]
fn ft8_streaming_sic_early_with_known_matches_batch_exactly() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // Phase 1: plain decode, no callback — its results seed phase 2's
    // `known`, reproducing a "second pass looking for more" shape.
    let phase1 = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.3, 50)
        .sic_early()
        .decode();
    assert!(
        !phase1.results.is_empty(),
        "expected real decodes on qso3_busy.wav"
    );

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft8::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let phase2 = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.3, 50)
        .sic_early()
        .known(&phase1.results)
        .on_result(&on_result)
        .decode();

    let batch: BTreeSet<String> = phase2
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    let streamed: BTreeSet<String> = streamed.into_inner().unwrap().into_iter().collect();

    println!(
        "phase2 batch: {} decode(s), streamed: {} callback(s)",
        batch.len(),
        streamed.len()
    );

    assert_eq!(
        streamed, batch,
        "sic_early + known: streamed callback deliveries must exactly \
         match the batch result — no candidate should fire on_result \
         and then be silently dropped by known-dedup"
    );
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
