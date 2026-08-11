//! `DecodeRequest::on_result` — streaming decode-callback verification
//! for FT4, against the real WSJT-X golden WAV
//! (`WSJT-X/samples/FT4/000000_000002.wav`).
//!
//! Companion to `ft8_streaming_decode.rs`. Exists because
//! `on_result` is a field on the *shared*
//! `DecodeRequest`/`SniperRequest<P>` structs (issue #191's generic
//! builder), so `.on_result(cb)` type-checks and compiles for
//! `DecodeRequest<Ft4>` even before any FT4-specific wiring exists —
//! and for a while (0.9.0-dev, between PR #237 and its FT4/FST4
//! follow-up) it silently compiled to a no-op there: `Ft4`'s
//! `FrameDecodable::__single_pass`/`SupportsSicRounds::__flat_sic`
//! never threaded `req.on_result` down into
//! `engine::pipeline::decode_frame`/`decode_frame_subtract`, so `cb`
//! was never invoked, with no compile error and no test to catch it
//! (`ft8_streaming_decode.rs`/`q65_wsjtx_samples.rs` were the only
//! `.on_result(` call sites in the whole test suite). This file closes
//! that coverage gap for FT4, mirroring FT8's own two contracts:
//!
//! 1. **Sequential SIC path** (`.sic_rounds(n)`): callback-delivered
//!    messages must exactly match the batch result — the push point
//!    inside `decode_frame_subtract`'s outer pass loop *is* the
//!    final-acceptance point.
//! 2. **Parallel single-pass path** (no `.sic_rounds()`): callback
//!    fires inside each candidate's `par_iter()` closure, before the
//!    later cross-candidate dedup — documented as a possible superset
//!    (a same-message duplicate found by two sync candidates could
//!    fire twice via callback while only one survives the dedup).
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path.

#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::Mutex;

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s × 12 kHz

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream("ft4/000000_000002.wav", Some("FT4/000000_000002.wav"))
}

fn load_slot() -> Option<Vec<i16>> {
    let path = sample_path()?;
    let raw = read_wsjtx_wav_i16(&path)?;
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

#[test]
fn ft4_streaming_sic_rounds_matches_batch_exactly() {
    let Some(audio) = load_slot() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft4::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let outcome = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
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
    assert!(
        !batch.is_empty(),
        "expected real decodes on the FT4 golden WAV"
    );
}

#[test]
fn ft4_streaming_single_pass_superset_of_batch() {
    let Some(audio) = load_slot() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft4::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    // No .sic_rounds() — default single-pass strategy, parallelized
    // via par_iter() under feature = "parallel".
    let outcome = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
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
        "expected real decodes on the FT4 golden WAV"
    );
}

/// Regression for the FT4/FST4 sibling of the `.sic_early()`+`.known()`
/// bug found on FT8 (issue #243 follow-up): `Ft4`'s `__single_pass`/
/// `__flat_sic` called `dedup_known` on the *returned* `Vec` only,
/// after `req.on_result` had already been threaded straight into
/// `engine::pipeline::decode_frame`/`decode_frame_subtract` and fired
/// there — so a candidate matching a caller-supplied `.known(...)`
/// entry could fire via callback and then be silently absent from the
/// returned `Vec`. Fixed via `pipeline::known_filtered_on_result`,
/// which wraps the callback itself so a `known`-duplicate never
/// reaches it in the first place. Exercises `.sic_rounds()`
/// specifically since that strategy is held to `on_result`'s
/// exact-match contract (unlike the parallel single-pass strategy's
/// weaker superset contract).
#[test]
fn ft4_streaming_sic_rounds_with_known_matches_batch_exactly() {
    let Some(audio) = load_slot() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    // Phase 1: plain decode, no callback — seeds phase 2's `known`.
    let phase1 = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(3)
        .decode();
    assert!(
        !phase1.results.is_empty(),
        "expected real decodes on the FT4 golden WAV"
    );

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft4::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let phase2 = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(3)
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
        "sic_rounds + known: streamed callback deliveries must exactly \
         match the batch result — no candidate should fire on_result \
         and then be silently dropped by known-dedup"
    );
}

/// `SniperRequest::on_result` — the third call site `pipeline_ap::
/// decode_sniper_ap` threads `on_result` through, shared by both FT4
/// and FST4's `__sniper`. No existing test anywhere in the suite
/// exercised `SniperRequest::on_result` for *any* protocol before this
/// one (including FT8's), so this also closes that broader gap, not
/// just the FT4-specific no-op.
///
/// Uses a clean synthetic signal (not the real golden WAV) —
/// `decode_sniper_ap` calls the *generic* `coarse_sync::<P>`, not
/// FT4's own `ft4_coarse_sync` (only `decode_frame`/
/// `decode_frame_subtract` branch on `P::ID == Ft4` for that), so
/// real-signal sniper recall on this particular WAV isn't guaranteed
/// the way the wide-band/SIC paths above are — a pre-existing
/// characteristic of `decode_sniper_ap`, not something this session's
/// `on_result` wiring changed. A clean synthetic keeps this test
/// focused on the callback contract, not on that separate question.
#[test]
fn ft4_streaming_sniper_matches_batch_exactly() {
    use mfsk_core::engine::{MessageCodec, MessageFields};
    use mfsk_core::ft4::encode::{message_to_tones, tones_to_i16};
    use mfsk_core::msg::Wsjt77Message;

    let msg77 = {
        let codec = Wsjt77Message;
        let bits = codec
            .pack(&MessageFields {
                call1: Some("CQ".into()),
                call2: Some("JA1ABC".into()),
                grid: Some("PM95".into()),
                ..Default::default()
            })
            .expect("pack CQ JA1ABC PM95");
        let mut out = [0u8; 77];
        out.copy_from_slice(&bits);
        out
    };
    let itone = message_to_tones(&msg77);
    let tone_audio = tones_to_i16(&itone, 1000.0, 20_000);

    let mut audio = vec![0i16; SLOT_SAMPLES];
    let start = (0.5 * 12_000.0) as usize;
    let n = tone_audio.len().min(SLOT_SAMPLES - start);
    audio[start..start + n].copy_from_slice(&tone_audio[..n]);

    let streamed: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::ft4::decode::DecodeResult| {
        if let Some(text) = unpack77(r.message77()) {
            streamed.lock().unwrap().push(text);
        }
    };

    let outcome = DecodeRequest::<Ft4>::sniper(&audio, 1000.0, 30)
        .on_result(&on_result)
        .decode();

    let batch: BTreeSet<String> = outcome
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    let streamed: BTreeSet<String> = streamed.into_inner().unwrap().into_iter().collect();

    println!(
        "sniper batch: {} decode(s), streamed: {} callback(s)",
        batch.len(),
        streamed.len()
    );

    assert_eq!(
        streamed, batch,
        "sniper path: streamed callback deliveries must exactly match \
         the batch result (sequential, push-point-is-final-acceptance)"
    );
    let phantoms: Vec<&String> = batch.iter().filter(|m| *m != "CQ JA1ABC PM95").collect();
    assert!(
        phantoms.is_empty(),
        "clean single-signal synth produced phantom decode(s): {phantoms:?}"
    );
    assert!(
        batch.iter().any(|m| m == "CQ JA1ABC PM95"),
        "expected the sniper to recover the clean synthetic 'CQ JA1ABC PM95', got {:?}",
        batch
    );
}
