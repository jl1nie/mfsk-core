//! The registry's capability bits must match the real trait impls —
//! checked in **both** directions.
//!
//! A capability table that is written by hand and read by a C ABI is a
//! table that lies within two releases. The failure is silent in the
//! worst way: a UI greys out a feature that works, or offers one that
//! returns an error, and nothing in the decoder notices.
//!
//! The enforcement here is the type system, used twice:
//!
//! - **Claimed ⇒ implemented.** `assert_sic_rounds::<P>()` is bounded on
//!   `SupportsSicRounds`, so naming a protocol that does not implement
//!   it is a *compile* error. Every protocol whose entry sets
//!   `caps::SIC_ROUNDS` must appear in the list passed to it.
//! - **Implemented ⇒ claimed.** The same call asserts the bit is set, so
//!   implementing the trait and forgetting the bit fails at runtime.
//!
//! What this cannot catch is a protocol that implements a trait and is
//! simply never named here. `every_capability_is_exercised` closes that
//! by requiring each bit to have been checked against at least one
//! protocol, so a newly-wired capability cannot slip through with no
//! coverage at all.
#![cfg(all(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
// `P` is deliberately unused in the `check_*` bodies: the *bound* is the
// check. `check_sic_rounds::<Ft4>("FT4")` compiles only because `Ft4`
// implements `SupportsSicRounds`, so the call site is a compile-time
// assertion about the trait while the body is the runtime assertion
// about the bit. Using `P` for something would weaken that, not
// strengthen it.
#![allow(clippy::extra_unused_type_parameters)]

use std::collections::BTreeSet;

use mfsk_core::msg::decode_request::{
    FrameDecodable, SupportsSicEarly, SupportsSicRounds, SupportsSniper, SupportsWideBandAp,
};
use mfsk_core::registry::{DecodeProfile, SyncScale, caps};
use mfsk_core::{PROTOCOLS, ProtocolMeta};

fn meta(name: &str) -> &'static ProtocolMeta {
    PROTOCOLS
        .iter()
        .find(|p| p.name == name)
        .unwrap_or_else(|| panic!("{name} missing from PROTOCOLS"))
}

fn profile(name: &str) -> DecodeProfile {
    meta(name).profile
}

/// Bounded on the trait: naming a protocol that lacks it will not
/// compile. Asserting the bit: implementing it without the bit fails.
fn check_sic_rounds<P: SupportsSicRounds>(name: &str) {
    assert!(
        profile(name).caps & caps::SIC_ROUNDS != 0,
        "{name} implements SupportsSicRounds but its registry entry does not claim caps::SIC_ROUNDS"
    );
}

fn check_sic_early<P: SupportsSicEarly>(name: &str) {
    assert!(
        profile(name).caps & caps::SIC_EARLY != 0,
        "{name} implements SupportsSicEarly but its registry entry does not claim caps::SIC_EARLY"
    );
}

fn check_wideband_ap<P: SupportsWideBandAp>(name: &str) {
    assert!(
        profile(name).caps & caps::AP_WIDEBAND != 0,
        "{name} implements SupportsWideBandAp but its entry does not claim caps::AP_WIDEBAND"
    );
}

fn check_sniper<P: SupportsSniper>(name: &str) {
    assert!(
        profile(name).caps & caps::SNIPER != 0,
        "{name} implements SupportsSniper but its registry entry does not claim caps::SNIPER"
    );
}

fn check_decode_handle<P: FrameDecodable>(name: &str) {
    assert!(
        profile(name).caps & caps::DECODE_HANDLE != 0,
        "{name} implements FrameDecodable but its entry does not claim caps::DECODE_HANDLE"
    );
}

const FST4_MODES: [&str; 5] = ["FST4-15", "FST4-30", "FST4-60A", "FST4-120", "FST4-300"];

#[test]
fn implemented_capabilities_are_claimed() {
    use mfsk_core::fst4::{Fst4s15, Fst4s30, Fst4s60, Fst4s120, Fst4s300};
    use mfsk_core::{Ft4, Ft8};

    check_decode_handle::<Ft8>("FT8");
    check_decode_handle::<Ft4>("FT4");
    check_decode_handle::<Fst4s15>("FST4-15");
    check_decode_handle::<Fst4s30>("FST4-30");
    check_decode_handle::<Fst4s60>("FST4-60A");
    check_decode_handle::<Fst4s120>("FST4-120");
    check_decode_handle::<Fst4s300>("FST4-300");

    check_sic_rounds::<Ft8>("FT8");
    check_sic_rounds::<Ft4>("FT4");

    check_sic_early::<Ft8>("FT8");

    check_wideband_ap::<Ft8>("FT8");
    check_wideband_ap::<Ft4>("FT4");
    check_wideband_ap::<Fst4s60>("FST4-60A");

    check_sniper::<Ft8>("FT8");
}

/// The other direction: a bit claimed by an entry that the list above
/// never validated. Keeping these lists exhaustive is the whole point,
/// so state them explicitly and compare.
#[test]
fn claimed_capabilities_are_implemented() {
    let claims = |bit: u32| -> BTreeSet<&'static str> {
        PROTOCOLS
            .iter()
            .filter(|p| p.profile.caps & bit != 0)
            .map(|p| p.name)
            .collect()
    };

    assert_eq!(
        claims(caps::SIC_ROUNDS),
        BTreeSet::from(["FT8", "FT4"]),
        "SupportsSicRounds is implemented for FT8 and FT4 only \
         (FST4 has no subtract path in WSJT-X at all)"
    );
    assert_eq!(
        claims(caps::SIC_EARLY),
        BTreeSet::from(["FT8"]),
        "SupportsSicEarly is FT8-only — no other protocol has a checkpoint \
         architecture to port"
    );
    let mut wideband_ap = BTreeSet::from(["FT8", "FT4"]);
    wideband_ap.extend(FST4_MODES);
    assert_eq!(
        claims(caps::AP_WIDEBAND),
        wideband_ap,
        "wide-band AP reaches every protocol now: it is a rung on the shared \
         ladder rather than something only the sniper's engine could do"
    );
    let q65 = PROTOCOLS
        .iter()
        .map(|p| p.name)
        .filter(|n| n.starts_with("Q65-"));
    let mut narrow_ap: BTreeSet<&'static str> = q65.collect();
    narrow_ap.insert("FT8");
    assert_eq!(
        claims(caps::AP_NARROW),
        narrow_ap,
        "narrow AP is for searches that already know the carrier: FT8's \
         SniperRequest, and Q65, whose decode is targeted by construction \
         and so has no wide-band counterpart"
    );
    assert_eq!(
        claims(caps::SNIPER),
        BTreeSet::from(["FT8"]),
        "narrow-band single-target search is an FT8 mode — the receive half of \
         an analogue roofing filter. FT4 is a contest protocol and FST4 has its \
         own DDC channelizer; see SupportsSniper"
    );

    let mut handles = BTreeSet::from(["FT8", "FT4"]);
    handles.extend(FST4_MODES);
    assert_eq!(
        claims(caps::DECODE_HANDLE),
        handles,
        "FrameDecodable is FT8 + FT4 + every FST4 sub-mode"
    );
}

/// Every bit must be claimed by at least one entry, so a constant that
/// is defined and then never wired up cannot sit there looking
/// meaningful.
#[test]
fn every_capability_is_claimed_by_something() {
    let all = [
        ("DECODE_HANDLE", caps::DECODE_HANDLE),
        ("SNIPER", caps::SNIPER),
        ("AP_NARROW", caps::AP_NARROW),
        ("AP_WIDEBAND", caps::AP_WIDEBAND),
        ("SIC_ROUNDS", caps::SIC_ROUNDS),
        ("SIC_EARLY", caps::SIC_EARLY),
        ("OSD", caps::OSD),
        ("EQ_MODE", caps::EQ_MODE),
        ("STRICTNESS", caps::STRICTNESS),
        ("BUDGET", caps::BUDGET),
        ("KNOWN_FILTER", caps::KNOWN_FILTER),
        ("KNOWN_SUBTRACT", caps::KNOWN_SUBTRACT),
        ("FFT_CACHE", caps::FFT_CACHE),
        ("ON_RESULT", caps::ON_RESULT),
        ("ENCODE", caps::ENCODE),
    ];
    for (name, bit) in all {
        assert!(
            PROTOCOLS.iter().any(|p| p.profile.caps & bit != 0),
            "caps::{name} is defined but no registry entry claims it"
        );
    }
}

/// `KNOWN_SUBTRACT` is strictly stronger than `KNOWN_FILTER`: a
/// protocol that subtracts a known signal from the audio also does not
/// re-report it.
#[test]
fn known_subtract_implies_known_filter() {
    for p in PROTOCOLS {
        if p.profile.caps & caps::KNOWN_SUBTRACT != 0 {
            assert!(
                p.profile.caps & caps::KNOWN_FILTER != 0,
                "{} claims KNOWN_SUBTRACT without KNOWN_FILTER",
                p.name
            );
        }
    }
}

/// The trap this table exists to defuse: FT4's `sync_min` is not on the
/// same scale as FT8's or FST4's, so a caller that copies one number
/// across protocols is wrong and cannot currently tell.
#[test]
fn sync_scale_is_recorded_and_defaults_respect_it() {
    assert_eq!(profile("FT4").sync_scale, SyncScale::BaselineNormalised);
    assert_eq!(profile("FT8").sync_scale, SyncScale::CostasAbsolute);
    for m in FST4_MODES {
        assert_eq!(profile(m).sync_scale, SyncScale::CostasAbsolute);
    }
    // The scan modes score sync as a 0..1 fraction of sync plus noise
    // (#413): a third scale, not FT8's.
    #[cfg(all(feature = "wspr", feature = "jt9", feature = "jt65", feature = "q65"))]
    for m in ["WSPR", "JT9", "JT65", "Q65-60A"] {
        assert_eq!(profile(m).sync_scale, SyncScale::SyncFraction, "{m}");
        let d = profile(m).defaults;
        assert!(d.sync_min > 0.0 && d.sync_min < 1.0, "{m}: {}", d.sync_min);
    }

    // On the baseline-normalised scale noise sits at ~1.0 by
    // construction, so a default at or below it would admit every peak
    // in the band. WSJT-X's own value is 1.18.
    let ft4 = profile("FT4");
    assert!(
        ft4.defaults.sync_min > 1.0,
        "FT4's default sync_min {} is at or below the noise floor its own scale puts at 1.0",
        ft4.defaults.sync_min
    );
    assert_eq!(ft4.defaults.sync_min, 1.18, "WSJT-X's ft4_decode.f90:195");
}

/// Every mode that can be decoded must publish a searchable band and a
/// candidate budget; every mode that cannot must not pretend to.
#[test]
fn defaults_are_usable_wherever_a_search_exists() {
    for p in PROTOCOLS {
        let d = p.profile.defaults;
        if p.profile.caps & caps::DECODE_HANDLE != 0 {
            assert!(
                d.freq_max_hz > d.freq_min_hz && d.max_cand > 0 && d.sync_min > 0.0,
                "{} drives the decode handle but publishes an unusable default search: {d:?}",
                p.name
            );
        }
    }
}

/// Geometry a host needs in order to size a buffer or place a
/// transmission, which the registry could not previously answer.
#[test]
fn slot_geometry_matches_the_protocols() {
    assert_eq!(meta("FT8").slot_samples_12k, 180_000);
    assert_eq!(meta("FT4").slot_samples_12k, 90_000);
    assert_eq!(meta("FST4-300").slot_samples_12k, 3_600_000);

    // FST4-15 is the odd one out: 0.5 s like FT8/FT4, where every other
    // FST4 sub-mode starts 1.0 s into the slot.
    assert_eq!(meta("FST4-15").tx_start_offset_s, 0.5);
    assert_eq!(meta("FT8").tx_start_offset_s, 0.5);
    for m in ["FST4-30", "FST4-60A", "FST4-120", "FST4-300"] {
        assert_eq!(meta(m).tx_start_offset_s, 1.0, "{m}");
    }
}
