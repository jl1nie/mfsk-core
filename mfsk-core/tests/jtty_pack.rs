// SPDX-License-Identifier: GPL-3.0-or-later
//! The JTTY text packer against WSJT-X's own `pack_jtty` (tier B), phase P5.
//!
//! `embedded-poc/assets/golden/jtty/pack_cases.tsv` is what upstream's
//! `pack_jtty` (v3.2.0-rc1, through `scripts/jttysim/jtty_pack_oracle.f90`) makes of
//! 3 500 messages under each exchange profile: every atom kind, the profile
//! differences, the grammar's edges, seeded random compositions and random text.
//! Each must come out as the very same frames — same count, same words, same
//! end-of-message flag — or as the same refusal.

#![cfg(feature = "jtty")]

#[allow(dead_code)]
mod common;

use mfsk_core::jtty::pack::{self, ExchangeProfile, PackError};
use mfsk_core::jtty::source::{Atom, CallAction, render_message};
use mfsk_core::jtty::tx;

fn profile(n: u8) -> ExchangeProfile {
    match n {
        0 => ExchangeProfile::Unknown,
        1 => ExchangeProfile::FieldDay,
        _ => ExchangeProfile::RttyRoundup,
    }
}

/// A payload as nine hex digits, the bits as an integer, first bit most significant.
fn hex(p: &mfsk_core::jtty::Payload) -> String {
    let v = p.iter().fold(0u64, |v, &b| v << 1 | u64::from(b));
    format!("{v:09X}")
}

#[test]
fn pack_matches_upstream_pack_jtty_on_every_case() {
    let path = common::corpus::golden_path("jtty/pack_cases.tsv").expect("vendored golden");
    let text = std::fs::read_to_string(path).unwrap();
    let mut n = 0;
    let mut refused = 0;
    for row in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = row.split('\t').collect();
        let (prof, msg, nframes, frames) = (f[0].parse::<u8>().unwrap(), f[1], f[2], f[3]);
        let want: i32 = nframes.parse().unwrap();
        let got = pack::pack(msg, profile(prof));
        match (want, got) {
            (-1, Err(_)) => refused += 1,
            (-1, Ok(a)) => panic!("{prof} {msg:?}: upstream refuses, we pack {a:?}"),
            (_, Err(e)) => panic!("{prof} {msg:?}: upstream packs {nframes} frames, we: {e}"),
            (0, Ok(atoms)) => assert!(atoms.is_empty(), "{prof} {msg:?}"),
            (k, Ok(atoms)) => {
                assert_eq!(atoms.len(), k as usize, "{prof} {msg:?}: {atoms:?}");
                let payloads = tx::payloads(&atoms).expect("packed atoms encode");
                let ours: Vec<String> = payloads.iter().map(hex).collect();
                assert_eq!(ours.join(" "), frames, "{prof} {msg:?}: {atoms:?}");
            }
        }
        n += 1;
    }
    assert!(n >= 3000, "only {n} cases");
    assert!(refused >= 1);
}

#[test]
fn what_is_packed_reads_back_as_what_was_typed() {
    for (typed, back) in [
        ("cq k1abc cq", "CQ K1ABC CQ"),
        ("  Tu   nOW  ja1xyz ", "TU NOW JA1XYZ"),
        ("599 fn42", "599 FN42"),
        ("3a ema", "3A EMA"),
        ("hello~world", "HELLO WORLD"),
    ] {
        let atoms = pack::pack(typed, ExchangeProfile::Unknown).unwrap();
        assert_eq!(render_message(&atoms), back, "{typed:?}");
    }
}

#[test]
fn the_choices_are_the_fewest_frames_and_structured_ones_win_ties() {
    // "CQ K1ABC CQ" is one frame as a call atom; as text it would be three.
    let a = pack::pack("CQ K1ABC CQ", ExchangeProfile::Unknown).unwrap();
    assert_eq!(a, [Atom::call(CallAction::Cq, "K1ABC")]);
    // "TU" alone: the control phrase (5·100 + 2·12) beats TEXT5 at equal cost
    let a = pack::pack("TU", ExchangeProfile::Unknown).unwrap();
    assert_eq!(a, [Atom::Control(12)]);
    // eighty characters of text fill sixteen frames exactly
    let long = "X".repeat(80);
    assert_eq!(
        pack::pack(&long, ExchangeProfile::Unknown).unwrap().len(),
        16
    );
}

#[test]
fn only_rtty_roundup_reads_a_serial_and_a_state() {
    use mfsk_core::jtty::source::{LocationKind, NumberKind, Role};
    let serial = |p| pack::pack("599 5", p).unwrap();
    // Unknown: 599 then a number is one generic-number utterance … unless it renders otherwise
    let plain = serial(ExchangeProfile::Unknown);
    let rtty = serial(ExchangeProfile::RttyRoundup);
    assert_eq!(plain.len(), 1);
    assert!(
        matches!(
            plain[0],
            Atom::Number {
                kind: NumberKind::Generic,
                ..
            }
        ),
        "{plain:?}"
    );
    assert!(
        matches!(
            rtty[0],
            Atom::Number {
                kind: NumberKind::Serial,
                role: Role::Full,
                value: 5
            }
        ),
        "{rtty:?}"
    );
    // and the state token
    let state = pack::pack("599 CA", ExchangeProfile::RttyRoundup).unwrap();
    assert!(
        matches!(
            &state[0],
            Atom::Location {
                kind: LocationKind::StateProvince,
                ..
            }
        ),
        "{state:?}"
    );
}

#[test]
fn what_cannot_be_packed_is_refused_not_truncated() {
    assert_eq!(
        pack::pack(&"A".repeat(81), ExchangeProfile::Unknown),
        Err(PackError::TooLong)
    );
    assert_eq!(pack::pack("", ExchangeProfile::Unknown), Ok(Vec::new()));
    // RTTY's serial rewrite lengthens `599 5` to `599 005`: thirteen of them overflow eighty
    let long = "599 5 ".repeat(13);
    assert_eq!(
        pack::pack(long.trim(), ExchangeProfile::RttyRoundup),
        Err(PackError::Serial)
    );
    // and the same text is fine under a profile that does not rewrite it
    assert!(pack::pack(long.trim(), ExchangeProfile::Unknown).is_ok());
}

#[test]
fn packed_text_goes_on_to_tones() {
    let t = pack::tones("CQ K1ABC CQ", ExchangeProfile::Unknown)
        .unwrap()
        .unwrap();
    assert_eq!(t.len(), 59);
    assert_eq!(pack::tones("", ExchangeProfile::Unknown), Ok(None));
}
