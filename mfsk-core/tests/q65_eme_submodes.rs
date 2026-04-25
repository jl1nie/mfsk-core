//! Q65 EME sub-mode end-to-end roundtrip tests.
//!
//! Verifies that the generic `synthesize_standard_for<P>` /
//! `decode_at_for<P>` / `decode_scan_for<P>` paths work with each of
//! the wired EME sub-modes (Q65-60A through Q65-60E). One synthetic
//! frame per sub-mode is enough to prove the type parameter cleanly
//! propagates NSPS and tone-spacing constants through the entire
//! tx → rx pipeline.

#![cfg(feature = "q65")]

use mfsk_core::core::ModulationParams;
use mfsk_core::q65::rx::{decode_at_for, decode_scan_for};
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::tx::synthesize_standard_for;
use mfsk_core::q65::{Q65a60, Q65b60, Q65c60, Q65d60, Q65e60};

const FS: u32 = 12_000;

fn roundtrip_aligned<P: ModulationParams>(label: &str) {
    let freq = 1500.0;
    let audio = synthesize_standard_for::<P>("CQ", "K1ABC", "FN42", FS, freq, 0.3)
        .unwrap_or_else(|| panic!("{label}: pack + synth must succeed"));
    let r = decode_at_for::<P>(&audio, FS, 0, freq)
        .unwrap_or_else(|| panic!("{label}: aligned decode must succeed"));
    assert_eq!(r.message, "CQ K1ABC FN42", "{label}: message text");
    assert_eq!(r.start_sample, 0, "{label}: start_sample");
    assert!((r.freq_hz - freq).abs() < 0.1, "{label}: freq_hz");
}

#[test]
fn q65_60a_roundtrip() {
    roundtrip_aligned::<Q65a60>("Q65-60A");
}

#[test]
fn q65_60b_roundtrip() {
    roundtrip_aligned::<Q65b60>("Q65-60B");
}

#[test]
fn q65_60c_roundtrip() {
    roundtrip_aligned::<Q65c60>("Q65-60C");
}

#[test]
fn q65_60d_roundtrip() {
    roundtrip_aligned::<Q65d60>("Q65-60D");
}

#[test]
fn q65_60e_roundtrip() {
    roundtrip_aligned::<Q65e60>("Q65-60E");
}

#[test]
fn q65_60a_scan_recovers_at_offset() {
    // For Q65-60A (the most common EME mode), also check the
    // sync-search path with a non-zero start sample inside a 60 s
    // slot. ±1 s of mis-alignment from PC clock drift is realistic.
    let freq = 1500.0;
    let audio = synthesize_standard_for::<Q65a60>("CQ", "JA1ABC", "PM95", FS, freq, 0.3)
        .expect("synth must succeed");
    let mut slot = vec![0.0_f32; (FS as usize) * 60];
    let start = FS as usize; // 1 s offset
    let n = audio.len().min(slot.len() - start);
    slot[start..start + n].copy_from_slice(&audio[..n]);

    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        // Q65-60A symbols are 0.6 s; a few-symbol search around
        // sample 0 would miss the 1 s offset. Cover comfortably.
        time_tolerance_symbols: 5,
        score_threshold: 0.1,
        max_candidates: 8,
    };
    let decodes = decode_scan_for::<Q65a60>(&slot, FS, start, &params);
    assert!(
        decodes.iter().any(|d| d.message == "CQ JA1ABC PM95"),
        "Q65-60A scan must find offset signal, got {decodes:#?}"
    );
}
