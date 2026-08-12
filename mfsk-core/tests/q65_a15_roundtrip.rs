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
        time_tolerance_early_sec: 0.75,
        time_tolerance_late_sec: 0.75,
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

/// Δt search window parity with WSJT-X (issue #282).
///
/// `q65.f90:127-128` sets `lag1 = -1.0/dtstep`, `lag2 = 1.0/dtstep`,
/// i.e. **±1.0 s for every sub-mode**. This crate expressed the same
/// window as `time_tolerance_symbols: 5`, which is ±0.75 s on
/// Q65-15A (0.15 s symbols) and ±17.3 s on Q65-300A — narrower than
/// the reference exactly where the symbols are shortest.
///
/// Measured against real `jt9 -3 -p 15 -b A -d 3` on a `q65sim` Δt
/// sweep at −20 dB: `jt9` decoded Δt ∈ [−1.00, +1.00]; this crate
/// decoded [−0.50, +0.50] before the fix and reaches +1.00 after it.
/// This test pins the positive edge, which is the half that is not
/// also bounded by the start of the slot buffer.
///
/// Deliberately at the *default* `SearchParams`, since the whole
/// defect was that the default's unit made it sub-mode dependent.
#[test]
fn q65_15a_default_window_covers_wsjtx_plus_one_second() {
    let freq = 1500.0;
    let audio = synthesize_standard_for::<Q65a15>("CQ", "JA1ABC", "PM95", FS, freq, 0.3)
        .expect("Q65-15A: synth must succeed");

    // Nominal Q65-15 frame start is 0.5 s into the slot (`q65sim.f90`
    // places at `t = xdt + 0.5` for TR ≤ 30). Place the signal a
    // further +1.0 s late — the outer edge of WSJT-X's own window.
    let nominal = FS as usize / 2;
    let start = nominal + FS as usize;
    let mut slot = vec![0.0_f32; (FS as usize) * 15];
    let n = audio.len().min(slot.len() - start);
    slot[start..start + n].copy_from_slice(&audio[..n]);

    let decodes =
        DecodeRequest::<Q65a15>::new(&slot, FS, nominal, SearchParams::default()).decode();
    assert!(
        decodes.iter().any(|d| d.message == "CQ JA1ABC PM95"),
        "Q65-15A must decode at Δt = +1.0 s, WSJT-X's own lag2 \
         (q65.f90:128) — got {decodes:#?}"
    );
}
