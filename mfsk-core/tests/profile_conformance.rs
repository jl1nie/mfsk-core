// SPDX-License-Identifier: GPL-3.0-only
//! Tier B: each profile's decode of ft8_lib's 22 real recordings against what `jt9` decodes (`tests/fixtures/ft8_lib_jt9_beta6.tsv`, the
//! fixture of `ft8_real_recordings_vs_jt9.rs`: 402 decodes), through the profile's own `DecodeParams`.
//!
//! **Fails if** a profile's decode drifts from what its numbers promise: P2 (`Depth::Deep`) must find every `jt9` decode, P1 (`Depth::Normal`)
//! at least [`P1_FLOOR`] of the 402; and neither may print a message that is neither `jt9`'s nor one of the attested extras (a phantom). The
//! floor and the ceiling are measured: the table in `docs/notes/PI3_JT9_GAP.md` section 20.
#![cfg(feature = "fft-rustfft")]

#[allow(dead_code)]
mod common;

use common::load_wav_i16;
use mfsk_core::decoder::{DecodeParams, Decoder, Profile, SlotInput};
use mfsk_core::ft8::Ft8;

const REFERENCE: &str = include_str!("fixtures/ft8_lib_jt9_beta6.tsv");

/// Decodes `jt9` lacks that are real signals (the list of `ft8_real_recordings_vs_jt9.rs`: keep the two in step).
const ATTESTED: &[(&str, &str)] = &[
    ("191111_110215", "SP7NHS YC2FCE -15"),
    ("websdr_test1", "CQ EA5OL IM99"),
    ("websdr_test2", "UA9SIX GM0LIR R-07"),
    ("websdr_test4", "CQ CU2DX HM77"),
    ("websdr_test7", "CU2DX R2DQA KO96"),
    ("websdr_test7", "EA8PP JH0INP PM96"),
];

/// P1's recall floor over the 402: 387 measured on x86-64 (2026-11-28), two under it for a platform's rounding.
const P1_FLOOR: usize = 385;

fn files() -> Vec<&'static str> {
    let mut f: Vec<&str> = REFERENCE
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
        .map(|l| l.split('\t').next().unwrap())
        .collect();
    f.dedup();
    f
}

/// (hits of jt9's, jt9's total, the messages that are neither jt9's nor attested) over the 22 recordings.
fn run(profile: Profile) -> (usize, usize, Vec<String>) {
    let (mut hits, mut total, mut phantoms) = (0, 0, Vec::new());
    for file in files() {
        let truth: Vec<&str> = REFERENCE
            .lines()
            .filter(|l| !l.starts_with('#') && !l.is_empty())
            .filter_map(|l| {
                let f: Vec<&str> = l.split('\t').collect();
                (f[0] == file).then_some(f[4])
            })
            .collect();
        let path = format!(
            "{}/../embedded-poc/assets/{file}.wav",
            env!("CARGO_MANIFEST_DIR")
        );
        let audio = load_wav_i16(path);
        let params = profile.apply(DecodeParams::for_band((200.0, 3000.0)));
        let rows = Decoder::<Ft8>::new(params)
            .decode(&SlotInput::i16(&audio))
            .rows;
        let got: std::collections::BTreeSet<String> =
            rows.iter().map(|r| r.decoded.text.clone()).collect();
        total += truth.len();
        hits += truth.iter().filter(|t| got.contains(**t)).count();
        for m in &got {
            if !truth.contains(&m.as_str()) && !ATTESTED.contains(&(file, m.as_str())) {
                phantoms.push(format!("{file}: {m}"));
            }
        }
    }
    (hits, total, phantoms)
}

#[test]
fn p2_finds_every_jt9_decode_and_nothing_unattested() {
    let (hits, total, phantoms) = run(Profile::P2);
    eprintln!("P2: {hits} of {total}, unattested extras {phantoms:?}");
    assert_eq!(total, 402);
    assert_eq!(hits, total);
    assert!(phantoms.is_empty(), "{phantoms:?}");
}

#[test]
fn p1_meets_its_floor_and_prints_nothing_unattested() {
    let (hits, total, phantoms) = run(Profile::P1);
    eprintln!("P1: {hits} of {total}, unattested extras {phantoms:?}");
    assert_eq!(total, 402);
    assert!(hits >= P1_FLOOR, "{hits} of {total}");
    assert!(phantoms.is_empty(), "{phantoms:?}");
}
