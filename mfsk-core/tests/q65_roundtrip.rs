//! Q65-30A end-to-end synthesis + decode integration tests.
//!
//! These exercise the full tx → audio buffer → search → decode →
//! unpack pipeline on synthesised signals (no FEC noise in the
//! channel) under conditions slightly more realistic than the unit
//! tests:
//! - the signal lives at a non-trivial dial frequency (1500 Hz),
//! - it can start at any sample within the slot — the receiver
//!   should still find it via [`mfsk_core::q65::decode_scan_default`],
//! - non-standard messages (free text, varying callsigns / grids)
//!   round-trip through the message codec.
//!
//! Failures here indicate a regression somewhere along the chain
//! that the unit tests didn't cover.

#![cfg(feature = "q65")]

use mfsk_core::q65::{decode_at, decode_scan_default, synthesize_standard};

const FS: u32 = 12_000;
/// 30-second Q65-30A slot at 12 kHz.
const SLOT_SAMPLES: usize = 12_000 * 30;

/// Drop a Q65 frame into a `SLOT_SAMPLES`-long zeroed buffer at
/// `start_sample`. Returns a vector ready to feed any of the
/// receive-side helpers.
fn make_slot(audio: &[f32], start_sample: usize) -> Vec<f32> {
    let mut slot = vec![0.0_f32; SLOT_SAMPLES];
    let n = audio.len().min(SLOT_SAMPLES.saturating_sub(start_sample));
    slot[start_sample..start_sample + n].copy_from_slice(&audio[..n]);
    slot
}

#[test]
fn aligned_decode_at_dial_frequency_recovers_message() {
    let freq = 1500.0;
    let audio = synthesize_standard("CQ", "K1ABC", "FN42", FS, freq, 0.3).expect("pack + synth");
    let result = decode_at(&audio, FS, 0, freq).expect("aligned decode must succeed");
    assert_eq!(result.message, "CQ K1ABC FN42");
    assert!((result.freq_hz - freq).abs() < 0.1);
}

#[test]
fn scan_finds_signal_with_one_second_offset() {
    // WSJT-X's nominal Q65-30A start is 1.0 s into the 30 s slot;
    // verify the scan can locate it without an alignment hint.
    let freq = 1500.0;
    let audio = synthesize_standard("CQ", "JA1ABC", "PM95", FS, freq, 0.3).expect("pack + synth");
    let slot = make_slot(&audio, FS as usize); // 1.0 s offset
    let decodes = decode_scan_default(&slot, FS);
    assert!(!decodes.is_empty(), "scan must find an offset signal");
    let hit = decodes
        .iter()
        .find(|d| d.message == "CQ JA1ABC PM95")
        .expect("expected message text not found");
    // Frequency bin width is 12000/3600 ≈ 3.33 Hz, so ±4 Hz is
    // within one bin tolerance.
    assert!(
        (hit.freq_hz - freq).abs() <= 4.0,
        "best freq {} should be near {freq} Hz",
        hit.freq_hz
    );
    // Time tolerance: ±1 symbol (3600 samples = 0.3 s) is the
    // worst-case mis-alignment from half-symbol search granularity.
    let nominal_start = FS as i64;
    let drift = (hit.start_sample as i64 - nominal_start).abs();
    assert!(
        drift <= 3600,
        "start_sample drift {drift} exceeds one symbol"
    );
}

#[test]
fn scan_finds_signal_at_low_dial_frequency() {
    // Q65-30A occupies only ~217 Hz of bandwidth, so it can sit
    // very close to the lower passband edge. 400 Hz is comfortably
    // inside the search default of 200 Hz.
    let freq = 400.0;
    let audio = synthesize_standard("CQ", "K1ABC", "FN42", FS, freq, 0.3).expect("pack + synth");
    let slot = make_slot(&audio, FS as usize);
    let decodes = decode_scan_default(&slot, FS);
    assert!(
        decodes.iter().any(|d| d.message == "CQ K1ABC FN42"),
        "expected CQ K1ABC FN42 not in {decodes:#?}"
    );
}

#[test]
fn scan_finds_signal_at_high_dial_frequency() {
    // Stress the upper edge of the default search band as well.
    let freq = 2700.0;
    let audio = synthesize_standard("W1AW", "JA1XYZ", "QM06", FS, freq, 0.3).expect("pack + synth");
    let slot = make_slot(&audio, FS as usize);
    let decodes = decode_scan_default(&slot, FS);
    assert!(
        decodes.iter().any(|d| d.message == "W1AW JA1XYZ QM06"),
        "expected W1AW JA1XYZ QM06 not in {decodes:#?}"
    );
}

#[test]
fn duplicate_decodes_are_collapsed() {
    // The same frame inserted at the same alignment should yield
    // exactly one decode in the scan output (the dedup logic in
    // `decode_scan` collapses near-duplicates).
    let freq = 1500.0;
    let audio = synthesize_standard("CQ", "K1ABC", "FN42", FS, freq, 0.3).expect("pack + synth");
    let slot = make_slot(&audio, FS as usize);
    let decodes = decode_scan_default(&slot, FS);
    let count = decodes
        .iter()
        .filter(|d| d.message == "CQ K1ABC FN42")
        .count();
    assert_eq!(
        count, 1,
        "expected exactly one CQ K1ABC FN42 decode, got {count}"
    );
}
