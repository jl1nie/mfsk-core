// SPDX-License-Identifier: GPL-3.0-or-later
//! JTTY wire level against WSJT-X's own `sjtty` / `rjtty` (tier A/B).
//!
//! The fixtures under `embedded-poc/assets/golden/jtty/sim/` come from
//! `scripts/gen_jtty_vectors.sh` (WSJT-X `v3.2.0-rc1`, built by
//! `scripts/build_jttysim.sh`):
//!
//! - `MANIFEST.tsv` — for each message, the tone sequence `sjtty` prints, which
//!   is `genjtty`'s output. The messages are turned into atoms **by hand** here
//!   (upstream's text packer is phase P5), so this checks the encoder — source
//!   word, CRC-12, tail-biting code, sync — against upstream, not the packer.
//! - `clean_*.wav` — `sjtty` at SNR > 90, i.e. the noiseless waveform,
//!   peak-normalised. This checks the GFSK synthesis. It says nothing about
//!   sensitivity: a noiseless synthetic fixture is not an instrument for that.

#![cfg(feature = "jtty")]

#[allow(dead_code)]
mod common;

use mfsk_core::jtty::source::{
    Atom, CallAction, LocationKind, NumberKind, Role, decode_payload, render_message,
};
use mfsk_core::jtty::{NSPS, SAMPLE_RATE, tx};

struct Row {
    name: String,
    message: String,
    f0: f32,
    dt: f32,
    tones: Vec<u8>,
    wav: Option<String>,
    rjtty_text: Option<String>,
}

fn manifest() -> Option<Vec<Row>> {
    let path = common::corpus::golden_path("jtty/sim/MANIFEST.tsv")?;
    let text = std::fs::read_to_string(path).expect("read MANIFEST.tsv");
    Some(
        text.lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .map(|l| {
                let f: Vec<&str> = l.split('\t').collect();
                let opt = |s: &str| (s != "-").then(|| s.to_string());
                Row {
                    name: f[0].into(),
                    message: f[2].into(),
                    f0: f[3].parse().unwrap(),
                    dt: f[4].parse().unwrap(),
                    tones: f[7]
                        .split_whitespace()
                        .map(|t| t.parse().unwrap())
                        .collect(),
                    wav: opt(f[8]),
                    rjtty_text: f.get(11).and_then(|s| opt(s)),
                }
            })
            .collect(),
    )
}

/// The atoms `pack_jtty` chooses for each manifest message, written out by hand.
fn atoms_for(name: &str) -> Vec<Atom> {
    let call = |a, c| Atom::call(a, c);
    let text = Atom::text5;
    match name {
        "cq_1frame" | "clean_cq_1frame" => vec![call(CallAction::Cq, "K1ABC")],
        "call_exch_2frames" | "clean_call_exch_2frames" => vec![
            call(CallAction::Call, "WB9XYZ"),
            Atom::Number {
                role: Role::Full,
                kind: NumberKind::Generic,
                value: 123,
            },
        ],
        "text5_3frames" => vec![text("HELLO"), text(" WORL"), text("D 73")],
        "class_section" => vec![Atom::class_section(1, 'D', "EMA").unwrap()],
        "rtty_serial" => vec![
            call(CallAction::Call, "K1ABC"),
            Atom::Number {
                role: Role::Full,
                kind: NumberKind::Serial,
                value: 1,
            },
        ],
        "tu_call" => vec![call(CallAction::TuCq, "K1ABC")],
        "call_tu" => vec![call(CallAction::CallTu, "K1ABC")],
        "call_agn" => vec![call(CallAction::CallAgn, "W9XYZ")],
        "tu_now" => vec![call(CallAction::TuNow, "W7UVW")],
        "generic_qth" => vec![Atom::location(Role::Full, LocationKind::Qth, "MA")],
        "grid4_full" => vec![Atom::grid4(Role::Full, "FN42")],
        "control_qsl_tu" => vec![Atom::Control(11)],
        "numeric" => vec![Atom::Number {
            role: Role::FieldOnly,
            kind: NumberKind::Generic,
            value: 123,
        }],
        "call_with_slash" => vec![text("K1ABC"), text("/P")],
        "lowercase_punct" => vec![text("CQ K1"), text("ABC, "), text("HI!")],
        other => panic!("no atoms written for manifest row {other}"),
    }
}

#[test]
fn tones_match_sjtty_for_every_manifest_message() {
    let Some(rows) = manifest() else { return };
    assert!(rows.len() >= 15, "manifest rows: {}", rows.len());
    for r in &rows {
        let atoms = atoms_for(&r.name);
        let got = tx::tones(&atoms).unwrap_or_else(|| panic!("{}: not encodable", r.name));
        assert_eq!(got, r.tones, "{} ({:?})", r.name, r.message);
    }
}

#[test]
fn our_rendering_matches_what_rjtty_prints() {
    let Some(rows) = manifest() else { return };
    let mut checked = 0;
    for r in rows.iter().filter(|r| r.rjtty_text.is_some()) {
        let text = r.rjtty_text.as_deref().unwrap();
        // rjtty prints the message with `~` shown as a space and no trailing blanks
        assert_eq!(render_message(&atoms_for(&r.name)), text, "{}", r.name);
        checked += 1;
    }
    assert!(checked >= 5);
}

#[test]
fn every_transmitted_frame_decodes_back_to_its_atom() {
    // Atom -> payload -> atom, for the atoms of every manifest message, with EOM
    // on the last frame only (the receive side of the wire level).
    let Some(rows) = manifest() else { return };
    for r in &rows {
        let atoms = atoms_for(&r.name);
        let payloads = tx::payloads(&atoms).unwrap();
        for (i, (a, p)) in atoms.iter().zip(&payloads).enumerate() {
            let (back, eom) = decode_payload(p).unwrap_or_else(|| panic!("{}", r.name));
            assert_eq!(&back.word(), &a.word(), "{} frame {i}", r.name);
            assert_eq!(eom, i + 1 == atoms.len(), "{} frame {i}", r.name);
        }
    }
}

/// `sjtty` peak-normalises its noiseless waveform to 32766.9; normalise both
/// sides to their own peak and compare sample by sample over the transmission.
///
/// The residual is upstream's, not ours: `gen_jttywave` integrates the phase in
/// single precision (`real`), whose error grows with the length of the
/// transmission (6e-4 for one frame, 1e-3 for two, measured); ours is carried
/// in `f64`. A one-sample time shift is off by ~0.8 and a one-sample error in the
/// Gaussian pulse index (#482) by 4.9e-2, so 2e-3 still discriminates both.
#[test]
fn waveform_matches_sjtty_noiseless_output() {
    let Some(rows) = manifest() else { return };
    let mut checked = 0;
    for r in rows.iter().filter(|r| r.name.starts_with("clean_")) {
        let path = common::corpus::golden_path(&format!("jtty/sim/{}", r.wav.as_ref().unwrap()))
            .expect("clean wav");
        let theirs = common::load_wav_i16(path);
        let tones = tx::tones(&atoms_for(&r.name)).unwrap();
        let ours = tx::synth_f32(&tones, r.f0, 1.0);
        let offset = (r.dt * SAMPLE_RATE).round() as usize;
        assert!(
            theirs.len() >= offset + ours.len(),
            "{}: wav too short",
            r.name
        );
        assert_eq!(ours.len() % NSPS, 0);

        let peak_theirs = theirs
            .iter()
            .map(|&x| f32::from(x).abs())
            .fold(0f32, f32::max);
        let peak_ours = ours.iter().fold(0f32, |m, &x| m.max(x.abs()));
        let worst_at = |shift: i64| {
            ours.iter()
                .enumerate()
                .filter(|(i, _)| *i as i64 + shift >= 0)
                .map(|(i, &x)| {
                    let t = theirs[(offset as i64 + i as i64 + shift) as usize];
                    (x / peak_ours - f32::from(t) / peak_theirs).abs()
                })
                .fold(0f32, f32::max)
        };
        let worst = worst_at(0);
        eprintln!(
            "{}: worst normalised sample error {worst:.2e} over {} samples",
            r.name,
            ours.len()
        );
        assert!(worst < 2e-3, "{}: worst error {worst}", r.name);
        assert!(
            worst_at(1) > 0.1 && worst_at(-1) > 0.1,
            "{}: alignment",
            r.name
        );
        // before the transmission the file is silent
        assert!(
            theirs[..offset].iter().all(|&x| x == 0),
            "{}: leading silence",
            r.name
        );
        checked += 1;
    }
    assert_eq!(checked, 2);
}
