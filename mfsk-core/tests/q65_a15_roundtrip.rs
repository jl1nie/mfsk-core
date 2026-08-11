//! Q65-15A end-to-end synthesis + decode integration test.
//!
//! Q65-15A is the fastest wired Q65 sub-mode (15 s T/R period,
//! NSPS=1800). Mirrors `tests/q65_eme_submodes.rs`'s pattern for the
//! EME sub-modes: verifies the generic `synthesize_standard_for<P>` /
//! `DecodeRequest<P>`/`SniperRequest<P>` paths correctly propagate
//! Q65a15's NSPS/tone-spacing constants through the entire tx → rx
//! pipeline, including the sync-search path at a non-zero offset
//! inside the (short) 15 s slot.

#![cfg(feature = "q65")]

use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::tx::synthesize_standard_for;
use mfsk_core::q65::{DecodeRequest, Q65a15};

const FS: u32 = 12_000;

#[test]
fn q65_15a_aligned_decode_recovers_message() {
    let freq = 1500.0;
    let audio = synthesize_standard_for::<Q65a15>("CQ", "K1ABC", "FN42", FS, freq, 0.3)
        .expect("Q65-15A: pack + synth must succeed");
    let r = DecodeRequest::<Q65a15>::sniper(&audio, FS, 0, freq)
        .decode()
        .expect("Q65-15A: aligned decode must succeed");
    assert_eq!(r.message, "CQ K1ABC FN42");
    assert_eq!(r.start_sample, 0);
    assert!((r.freq_hz - freq).abs() < 0.1);
}

#[test]
fn q65_15a_scan_recovers_at_offset() {
    // WSJT-X's nominal Q65 start is 1.0 s into the slot; verify the
    // scan can locate it without an alignment hint even in the short
    // 15 s slot (NSPS=1800, symbol length 0.15 s).
    let freq = 1500.0;
    let audio = synthesize_standard_for::<Q65a15>("CQ", "JA1ABC", "PM95", FS, freq, 0.3)
        .expect("Q65-15A: synth must succeed");
    let mut slot = vec![0.0_f32; (FS as usize) * 15];
    let start = FS as usize; // 1 s offset
    let n = audio.len().min(slot.len() - start);
    slot[start..start + n].copy_from_slice(&audio[..n]);

    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 5,
        score_threshold: 0.1,
        max_candidates: 8,
    };
    let decodes = DecodeRequest::<Q65a15>::new(&slot, FS, start, params).decode();
    // Precision, not just recall: one signal went into a clean synth
    // slot, so any other message coming out is a phantom. A
    // recall-only `any(...)` cannot see them.
    let phantoms: Vec<&str> = decodes
        .iter()
        .map(|d| d.message.as_str())
        .filter(|m| *m != "CQ JA1ABC PM95")
        .collect();
    assert!(
        phantoms.is_empty(),
        "clean single-signal synth produced phantom decode(s): {phantoms:?}"
    );
    assert!(
        decodes.iter().any(|d| d.message == "CQ JA1ABC PM95"),
        "Q65-15A scan must find offset signal, got {decodes:#?}"
    );
}
