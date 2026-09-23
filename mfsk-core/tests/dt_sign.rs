// SPDX-License-Identifier: GPL-3.0-or-later
//! A signal that starts *before* the nominal position must report a
//! negative dt, all the way out to [`Decoded`].
//!
//! ## Why this test exists
//!
//! Every mode here stores the frame start as a `usize` sample index,
//! and every one of them clamps it at zero on the way out:
//! `start_sample.saturating_sub(pad)` in Q65, JT65 and WSPR,
//! `c2_offset.max(0)` inside `jt9::decode::lag_to_audio_sample`. An
//! early signal's offset is negative and a `usize` cannot hold it.
//!
//! Q65, JT65 and WSPR keep the truth in a parallel `dt_sec: f32`,
//! which is exactly what that field is for — `q65::decode_request`
//! says so where it writes it: "`start_sample` has nowhere to put a
//! negative, so it saturates and `dt_sec` carries the truth." Then
//! three of the four `to_decoded` impls threw it away again, deriving
//! dt from the clamped `start_sample` instead of reading the field,
//! and handed the caller 0.0 for every early decode (#397).
//!
//! The searches look for these deliberately — Q65's default window
//! reaches 1.0 s early — so this is not a corner nobody visits.
//!
//! Per-trial sweep CSVs cannot see any of this: they record pass/fail,
//! and an early signal that decodes with the wrong dt still passes.
//! Hence a test that asserts the sign.

#[allow(dead_code)]
mod common;

const FS: u32 = 12_000;

/// Lay `signal` into a slot starting at `start_sec`.
fn place(signal: &[f32], start_sec: f32, slot_sec: f32) -> Vec<f32> {
    let mut slot = vec![0.0f32; (FS as f32 * slot_sec) as usize];
    let start = (start_sec * FS as f32).round() as usize;
    let n = signal.len().min(slot.len().saturating_sub(start));
    slot[start..start + n].copy_from_slice(&signal[..n]);
    slot
}

/// Q65-30A placed 0.5 s before the nominal start.
///
/// The nominal start for a 30 s period is 0.5 s into the slot
/// (`TX_START_OFFSET_S`, #399), so a signal laid at t=0 is half a
/// second early and dt must come back at about −0.5 s.
#[test]
fn q65_early_signal_reports_negative_dt() {
    use mfsk_core::q65::{Q65a30, decode_request::DecodeRequest, tx::synthesize_standard_for};

    let signal = synthesize_standard_for::<Q65a30>("CQ", "JL1NIE", "PM95", FS, 1500.0, 0.5)
        .expect("Q65-30A synth must succeed");

    let nominal_sec = 0.5f32;
    let early_sec = 0.0f32;
    let slot = place(&signal, early_sec, 30.0);
    let nominal_start = (nominal_sec * FS as f32) as usize;

    let out = DecodeRequest::<Q65a30>::new(
        &slot,
        FS,
        nominal_start,
        mfsk_core::q65::search::default_search_params(),
    )
    .decode();

    let Some(r) = out.first() else {
        panic!("Q65-30A must decode its own clean synthesis placed 0.5 s early");
    };

    let want = early_sec - nominal_sec; // −0.5 s
    assert!(
        (r.dt_sec - want).abs() < 0.15,
        "Q65Result::dt_sec should be about {want:+.2} s, got {:+.3} s \
         (start_sample = {} — clamped at zero, which is why the field exists)",
        r.dt_sec,
        r.start_sample
    );

    // And the value must survive the conversion the UI actually calls.
    let d = r.to_decoded();
    assert!(
        (d.dt_sec - want).abs() < 0.15,
        "Decoded::dt_sec should be about {want:+.2} s, got {:+.3} s — \
         to_decoded must read the stored dt_sec, not re-derive it from \
         the clamped start_sample",
        d.dt_sec
    );
}

/// JT65 placed 0.5 s early. Its nominal start is 0.0 s, so the signal
/// goes into a padded slot and the decoder's own early-frame padding
/// is what brings it back.
#[test]
fn jt65_early_signal_reports_negative_dt() {
    use mfsk_core::jt65::{decode_scan, search::default_search_params, tx::synthesize_standard};

    let signal = synthesize_standard("CQ", "JL1NIE", "PM95", FS, 1270.0, 0.5).expect("JT65 synth");

    // Nominal start 1.0 s in, signal at 0.5 s → dt = −0.5 s.
    let nominal_sec = 1.0f32;
    let early_sec = 0.5f32;
    let slot = place(&signal, early_sec, 60.0);
    let nominal_start = (nominal_sec * FS as f32) as usize;

    let out = decode_scan(&slot, FS, nominal_start, &default_search_params());
    let Some(r) = out.first() else {
        panic!("JT65 must decode its own clean synthesis placed 0.5 s early");
    };

    let want = early_sec - nominal_sec;
    assert!(
        (r.dt_sec - want).abs() < 0.3,
        "Jt65Result::dt_sec should be about {want:+.2} s, got {:+.3} s (start_sample = {})",
        r.dt_sec,
        r.start_sample
    );

    let d = r.to_decoded();
    assert!(
        (d.dt_sec - want).abs() < 0.3,
        "Decoded::dt_sec should be about {want:+.2} s, got {:+.3} s",
        d.dt_sec
    );
}
