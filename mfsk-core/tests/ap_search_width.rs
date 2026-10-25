//! The search window is a parameter, and an AP hint no longer ends the
//! search.
//!
//! Two things are pinned here, both of which were structural
//! assumptions before they were parameters.
//!
//! **1. `search_hz` is plumbed.** It was a `250.0` literal at each
//! dispatch site, which is why the candidate-population question raised
//! on issue #306 could not be measured: on FST4 the embedded runtime is
//! dominated by false survivors reaching deep decoding — a real decode
//! costs ~55 ms against ~14 s for a pathological false one — and
//! narrowing the band is the one multiplier that the per-candidate cost
//! work never touched. Whether it *does* reduce the false-survivor
//! count is still unmeasured, because this path also runs a halved sync
//! gate that may offset the narrower band. This test only establishes
//! that the knob exists and behaves monotonically; the measurement is
//! separate work.
//!
//! **2. The default is unchanged.** `.search_hz()` unset must reproduce
//! the old hardcoded 500 Hz span exactly, or every sniper number this
//! crate has published moves silently.
#![cfg(all(feature = "ft8", feature = "fft-rustfft"))]

use mfsk_core::__internal::msg::decode_request::DecodeRequest;
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

const FS: f32 = 12_000.0;

/// One clean FT8 slot carrying `msg` at `f0`, plus a second signal well
/// outside a ±250 Hz window around it. Two signals is what makes the
/// early-exit question observable at all.
fn ft8_two_signal_slot(a: &[u8; 77], fa: f32, b: &[u8; 77], fb: f32) -> Vec<i16> {
    let mut slot = vec![0i16; 15 * FS as usize];
    for (msg, f0) in [(a, fa), (b, fb)] {
        let tones = mfsk_core::engine::tx::message_to_tones::<mfsk_core::ft8::Ft8>(msg);
        let wave =
            mfsk_core::engine::tx::synthesize_i16::<mfsk_core::ft8::Ft8>(&tones, 12_000, f0, 8_000);
        let start = (0.5 * FS) as usize;
        for (i, s) in wave.iter().enumerate() {
            if let Some(d) = slot.get_mut(start + i) {
                *d = d.saturating_add(*s);
            }
        }
    }
    slot
}

fn texts(results: &[mfsk_core::engine::frame::DecodeResult]) -> Vec<String> {
    let mut v: Vec<String> = results
        .iter()
        .map(|r| unpack77(r.message77()).unwrap_or_default())
        .collect();
    v.sort();
    v
}

#[test]
fn default_search_width_is_the_old_hardcoded_500_hz_span() {
    let a = pack77("CQ", "JA1ABC", "PM95").expect("pack a");
    let b = pack77("CQ", "VK3NV", "QF22").expect("pack b");
    // 1500 Hz and 2100 Hz: 600 Hz apart, so the second sits outside a
    // ±250 Hz window and inside a ±400 Hz one.
    let slot = ft8_two_signal_slot(&a, 1500.0, &b, 2100.0);

    let implicit = DecodeRequest::<Ft8>::sniper(&slot, 1500.0, 20)
        .decode()
        .results;
    let explicit = DecodeRequest::<Ft8>::sniper(&slot, 1500.0, 20)
        .search_hz(250.0)
        .decode()
        .results;
    assert_eq!(
        texts(&implicit),
        texts(&explicit),
        "the default must reproduce the 250.0 literal the dispatch sites used to carry"
    );
    assert!(
        !texts(&implicit).is_empty(),
        "fixture produced no decodes at all — the test cannot say anything"
    );
}

#[test]
fn widening_the_window_admits_a_signal_outside_it() {
    let a = pack77("CQ", "JA1ABC", "PM95").expect("pack a");
    let b = pack77("CQ", "VK3NV", "QF22").expect("pack b");
    let slot = ft8_two_signal_slot(&a, 1500.0, &b, 2100.0);

    let narrow = texts(
        &DecodeRequest::<Ft8>::sniper(&slot, 1500.0, 20)
            .decode()
            .results,
    );
    let wide = texts(
        &DecodeRequest::<Ft8>::sniper(&slot, 1500.0, 20)
            .search_hz(700.0)
            .decode()
            .results,
    );

    assert!(
        narrow.iter().any(|t| t.contains("JA1ABC")),
        "the in-window signal should decode at the default width: {narrow:?}"
    );
    assert!(
        !narrow.iter().any(|t| t.contains("VK3NV")),
        "a signal 600 Hz away must be outside the ±250 Hz default: {narrow:?}"
    );
    assert!(
        wide.iter().any(|t| t.contains("VK3NV")),
        "±700 Hz must reach it: {wide:?}"
    );
}
