//! `SlotInput::budget` is honoured by every mode, and `BudgetReport::exhausted`
//! says so (#593). Before it, WSPR, JT9, JT65 and Q65 ignored the budget and
//! reported "not reached" whatever was passed.
//!
//! Through the public `decode`, one recording per mode. For each: a budget that
//! never stops changes nothing; one spent from the start gets no row and says
//! `exhausted`; one that stops after a few polls gets fewer rows than the full
//! decode, so the poll is inside the candidate loop and not only in front of it.
#![cfg(feature = "full")]

#[allow(dead_code)]
mod common;

use std::sync::atomic::{AtomicUsize, Ordering};

use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, SlotInput};

/// Decode `path` as `mode` three ways and check what the budget did.
/// `at_least`: the full decode must have this many rows, so that a cut after
/// the first poll has something to cut.
fn check(mode: Mode, path: &str, at_least: usize) {
    let Some(a) = common::load_wav_f32_opt(path) else {
        common::skip_or_fail(path);
        return;
    };
    let name = mode.name();
    let run = |budget: Option<&(dyn Fn() -> bool + Sync)>| {
        let mut d = AnyDecoder::with_defaults(mode);
        let slot = SlotInput::f32(&a).period(100);
        let slot = match budget {
            Some(b) => slot.budget(b),
            None => slot,
        };
        d.decode(&slot)
    };

    let full = run(None);
    assert!(
        full.rows.len() >= at_least,
        "{name}: the recording gave {} rows, the test needs {at_least}",
        full.rows.len()
    );
    assert!(
        !full.budget.exhausted,
        "{name}: no budget, nothing to exhaust"
    );

    // A budget that never stops is the same decode.
    let always = || true;
    let same = run(Some(&always));
    assert_eq!(same.rows, full.rows, "{name}: a budget that never stops");
    assert!(!same.budget.exhausted, "{name}");

    // Spent from the start: nothing decoded, and it says why.
    let spent = || false;
    let none = run(Some(&spent));
    assert!(
        none.budget.exhausted,
        "{name}: a spent budget must be reported"
    );
    assert!(
        none.rows.is_empty(),
        "{name}: a spent budget still decoded {:?}",
        none.rows
    );

    // Stopping after the first poll: fewer rows than the full decode, so the
    // poll is inside the candidate loop.
    let polls = AtomicUsize::new(0);
    let once = || polls.fetch_add(1, Ordering::Relaxed) < 1;
    let cut = run(Some(&once));
    assert!(cut.budget.exhausted, "{name}: cut after one poll");
    assert!(
        cut.rows.len() < full.rows.len(),
        "{name}: cut after one poll gave {} of {} rows",
        cut.rows.len(),
        full.rows.len()
    );
}

#[test]
fn wspr_honours_the_budget() {
    check(Mode::Wspr, asset_path!("golden/wspr/150426_0918.wav"), 2);
}

#[test]
fn jt9_honours_the_budget() {
    check(Mode::Jt9, asset_path!("130418_1742.wav"), 2);
}

#[test]
fn jt65_honours_the_budget() {
    check(Mode::Jt65, asset_path!("golden/jt65/jt65a_5sig_m18.wav"), 2);
}

#[test]
fn ft8_still_honours_it() {
    check(Mode::Ft8, asset_path!("qso3_busy.wav"), 2);
}

/// Q65 needs the search window of `decoder_depth.rs`; typed, like the key test.
#[test]
fn q65_honours_the_budget() {
    use mfsk_core::decoder::{DecodeParams, Decoder, Q65Extras};
    use mfsk_core::q65::Q65d60;
    let Some(path) = common::corpus::golden_path("q65/60D_EME_10GHz/201212_1838.wav") else {
        common::skip_or_fail("Q65 60D golden");
        return;
    };
    let a = common::load_wav_f32_opt(&path).unwrap();
    let make = || {
        let mut e = Q65Extras::default();
        e.search.time_tolerance_early_sec = Some(7.0);
        e.search.time_tolerance_late_sec = Some(5.0);
        e.search.score_threshold = Some(0.05);
        e.search.max_candidates = Some(8);
        e.fading = Some((mfsk_core::fec::qra::FadingModel::Gaussian, 10.0));
        Decoder::<Q65d60>::new(DecodeParams::for_band((200.0, 3000.0))).with_extras(e)
    };
    let full = make().decode(&SlotInput::f32(&a));
    assert!(!full.rows.is_empty() && !full.budget.exhausted);

    let always = || true;
    let same = make().decode(&SlotInput::f32(&a).budget(&always));
    assert_eq!(same.rows.len(), full.rows.len());
    assert!(!same.budget.exhausted);

    let spent = || false;
    let none = make().decode(&SlotInput::f32(&a).budget(&spent));
    assert!(
        none.budget.exhausted,
        "Q65: a spent budget must be reported"
    );
    assert!(none.rows.is_empty(), "Q65: {:?}", none.rows);
}
