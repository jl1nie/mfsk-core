//! The three strategies the capability word advertised and no C entry
//! point could reach: `MFSK_CAP_BUDGET`, `MFSK_CAP_KNOWN_FILTER` /
//! `_KNOWN_SUBTRACT`, and `MFSK_CAP_FFT_CACHE`.
//!
//! A published capability that cannot be exercised is worse than an
//! absent one: a consumer reads the bit, believes the mode does it, and
//! finds nothing to call. These tests are written the way that consumer
//! would have to — set the option through the ABI, then assert on what
//! the decode did differently.

mod common;

use std::sync::atomic::{AtomicU32, Ordering};

use common::*;
use mfsk::*;

/// The predicate is a plain C function: a counter in `user_data`, and a
/// cut-off after `n` polls. `Sync` is the caller's problem by contract,
/// which an atomic satisfies.
struct Poller {
    polls: AtomicU32,
    allow: u32,
}

unsafe extern "C" fn poll(user: *mut std::ffi::c_void) -> bool {
    let p = unsafe { &*(user as *const Poller) };
    p.polls.fetch_add(1, Ordering::Relaxed) < p.allow
}

fn budget_report(dec: *const MfskDecodeSession) -> MfskBudgetReport {
    let mut r = std::mem::MaybeUninit::<MfskBudgetReport>::zeroed();
    assert_eq!(
        unsafe { mfsk_session_last_budget(dec, r.as_mut_ptr()) },
        MfskStatus::Ok
    );
    unsafe { r.assume_init() }
}

/// Two stations in one slot, so there is something for a budget to cut
/// and something for a known list to remove.
fn two_station_slot() -> Vec<i16> {
    let a = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1200.0);
    let b = synth_slot_i16(MfskMode::Ft8, "CQ", "VK3NV", "QF22", 1800.0);
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| x.saturating_add(*y))
        .collect()
}

#[test]
fn without_a_budget_the_report_is_all_zero() {
    let dec = open(MfskMode::Ft8, None);
    let rows = decode_i16(dec, &two_station_slot());
    assert!(!rows.is_empty(), "the fixture should decode");

    let r = budget_report(dec);
    assert!(
        !r.exhausted,
        "nothing was cut, so nothing should say it was"
    );
    assert_eq!(r.candidates_skipped, 0);
    assert_eq!(r.cut_at_sync, -1, "absent is -1, because 0 is a real count");
    assert!(
        r.cut_at_score.is_nan(),
        "and absent is NaN, because 0.0 is a real score"
    );
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn a_budget_that_refuses_everything_cuts_the_search_and_says_so() {
    let audio = two_station_slot();
    let dec = open(MfskMode::Ft8, None);
    let full = decode_i16(dec, &audio).len();
    assert!(full > 0);

    let p = Poller {
        polls: AtomicU32::new(0),
        allow: 0,
    };
    assert_eq!(
        unsafe {
            mfsk_session_set_budget(
                dec,
                Some(poll),
                &p as *const Poller as *mut std::ffi::c_void,
            )
        },
        MfskStatus::Ok
    );
    let cut = decode_i16(dec, &audio);
    assert!(
        p.polls.load(Ordering::Relaxed) > 0,
        "the predicate was never polled"
    );
    assert!(
        cut.len() < full,
        "a budget that refuses every candidate should find less than the unbudgeted call \
         (got {} of {full})",
        cut.len()
    );

    let r = budget_report(dec);
    assert!(
        r.exhausted,
        "work was left undone and the report must say so"
    );
    assert!(r.candidates_skipped > 0);
    assert!(
        r.cut_at_sync >= 0,
        "FT8 ranks by Costas sync, so the cut point is knowable there"
    );

    // And removing it restores the full search on the same handle.
    assert_eq!(
        unsafe { mfsk_session_set_budget(dec, None, std::ptr::null_mut()) },
        MfskStatus::Ok
    );
    assert_eq!(decode_i16(dec, &audio).len(), full);
    assert!(!budget_report(dec).exhausted);
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn a_generous_budget_changes_nothing() {
    let audio = two_station_slot();
    let dec = open(MfskMode::Ft8, None);
    let full = decode_i16(dec, &audio).len();

    let p = Poller {
        polls: AtomicU32::new(0),
        allow: u32::MAX,
    };
    unsafe {
        mfsk_session_set_budget(
            dec,
            Some(poll),
            &p as *const Poller as *mut std::ffi::c_void,
        )
    };
    assert_eq!(decode_i16(dec, &audio).len(), full);
    assert!(!budget_report(dec).exhausted);
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn known_signals_are_not_reported_twice() {
    let audio = two_station_slot();
    let dec = open(MfskMode::Ft8, None);
    assert_eq!(
        unsafe { mfsk_session_keep_known(dec, true) },
        MfskStatus::Ok
    );
    assert_eq!(
        mfsk_session_known_count(dec),
        0,
        "nothing is known before the first decode"
    );

    let first = decode_i16(dec, &audio);
    assert!(any_contains(&first, "JA1ABC"));
    assert_eq!(
        mfsk_session_known_count(dec),
        first.len(),
        "every row of the first decode should now be known"
    );

    // Same audio, same session: everything in it is already known.
    let second = decode_i16(dec, &audio);
    assert!(
        second.is_empty(),
        "a second pass over the same slot re-reported {:?}",
        texts(&second)
    );

    // Turning it off both stops carrying and drops what was held.
    assert_eq!(
        unsafe { mfsk_session_keep_known(dec, false) },
        MfskStatus::Ok
    );
    assert_eq!(mfsk_session_known_count(dec), 0);
    assert_eq!(decode_i16(dec, &audio).len(), first.len());
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn the_fft_cache_does_not_change_the_answer() {
    let audio = two_station_slot();
    let plain = {
        let dec = open(MfskMode::Ft8, None);
        let rows = texts(&decode_i16(dec, &audio));
        unsafe { mfsk_session_close(dec) };
        rows
    };

    let dec = open(MfskMode::Ft8, None);
    assert_eq!(
        unsafe { mfsk_session_keep_fft_cache(dec, true) },
        MfskStatus::Ok
    );
    let first = texts(&decode_i16(dec, &audio));
    let second = texts(&decode_i16(dec, &audio)); // this one reuses the cache
    assert_eq!(first, plain);
    assert_eq!(
        second, plain,
        "the reused transform must decode the same slot the same way"
    );
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn a_cache_is_not_reused_against_different_audio() {
    // The failure this guards is silent: a slot transform of *other*
    // audio produces a confident wrong answer. The fingerprint means
    // the second decode rebuilds instead.
    let first_audio = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1200.0);
    let second_audio = synth_slot_i16(MfskMode::Ft8, "CQ", "VK3NV", "QF22", 1800.0);

    let dec = open(MfskMode::Ft8, None);
    unsafe { mfsk_session_keep_fft_cache(dec, true) };
    let a = decode_i16(dec, &first_audio);
    assert!(any_contains(&a, "JA1ABC"));
    let b = decode_i16(dec, &second_audio);
    assert!(
        any_contains(&b, "VK3NV"),
        "the second slot decoded as {:?} — a stale cache would do exactly this",
        texts(&b)
    );
    assert!(
        !any_contains(&b, "JA1ABC"),
        "and it must not report the first slot's station"
    );
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn every_handle_mode_publishes_what_it_now_accepts() {
    // The bits and the entry points have to agree: this is the check
    // that the four capability bits are no longer decorative.
    for i in 0..mfsk_mode_count() {
        let mut m = MfskMode::Ft8;
        assert_eq!(unsafe { mfsk_mode_at(i, &mut m) }, MfskStatus::Ok);
        let caps = mfsk_mode_caps(m as u32);
        if caps & MFSK_CAP_DECODE_HANDLE == 0 {
            continue;
        }
        let dec = open(m, None);
        let want_budget = caps & MFSK_CAP_BUDGET != 0;
        let got = unsafe { mfsk_session_set_budget(dec, Some(poll), std::ptr::null_mut()) };
        assert_eq!(
            got == MfskStatus::Ok,
            want_budget,
            "{m:?}: MFSK_CAP_BUDGET says {want_budget}, set_budget says {got:?}"
        );
        unsafe { mfsk_session_set_budget(dec, None, std::ptr::null_mut()) };

        let want_known = caps & MFSK_CAP_KNOWN_FILTER != 0;
        let got = unsafe { mfsk_session_keep_known(dec, true) };
        assert_eq!(got == MfskStatus::Ok, want_known, "{m:?}: known");

        let want_cache = caps & MFSK_CAP_FFT_CACHE != 0;
        let got = unsafe { mfsk_session_keep_fft_cache(dec, true) };
        assert_eq!(got == MfskStatus::Ok, want_cache, "{m:?}: fft cache");
        unsafe { mfsk_session_close(dec) };
    }
}
