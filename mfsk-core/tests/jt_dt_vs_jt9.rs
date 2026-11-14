// SPDX-License-Identifier: GPL-3.0-only
//! JT9 and JT65 report DT on WSJT-X's scale through the public `Decoder` (#726).
//!
//! WSJT-X starts JT9 and JT65 audio 1.0 s into the minute (`Modulator.cpp:163` `delay_ms=1000`,
//! `jt9sim.f90:124`, `jt65sim.f90:217`) and its decoders report DT from there (`jt65_decode.f90:425`
//! `dtx-1.0`). Until #726 both protocols had `TX_START_OFFSET_S = 0.0`, so `Decoder` reported every DT
//! 1.0 s higher than `jt9` does on the same file, and nothing failed: the golden tests compared
//! `start_sample` against a hand-added offset, and the snapshot tests only pinned this crate's own
//! numbers. This file compares with `jt9` directly.
//!
//! Expected DTs are real `jt9` output, WSJT-X v3.3.0-beta6 (`scripts/build_jt9_upstream.sh`, 2026-11-14):
//! `jt9 -9 -p 60 -L 1050 -H 1550 -d 3 130418_1742.wav` and `jt9 -6 -p 60 -b A jt65a_5sig_m18.wav`.
//! `jt9` prints DT to 0.1 s, so the tolerance is 0.1. Before `jt9::sync9` (#726) two JT9 rows were
//! 0.2 s off (`K1JT N5KDV EM41` -0.10, `G7CNF N4HFA EL89` +0.01): the old scan decoded them at other bins.

#![cfg(all(
    feature = "jt9",
    feature = "jt65",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

#[allow(dead_code)]
mod common;

use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::{Jt9, Jt65};

const DT_TOL_SEC: f32 = 0.1;

fn assert_dt(mode: &str, rows: &[mfsk_core::msg::Decoded], want: &[(&str, f32)]) {
    for &(msg, dt) in want {
        let got: Vec<f32> = rows
            .iter()
            .filter(|r| r.text == msg)
            .map(|r| r.dt_sec)
            .collect();
        assert!(!got.is_empty(), "{mode}: `{msg}` not decoded");
        for g in got {
            assert!(
                (g - dt).abs() <= DT_TOL_SEC,
                "{mode}: `{msg}` DT {g:+.3}, jt9 reports {dt:+.1}"
            );
        }
    }
}

#[test]
fn jt9_decoder_dt_matches_jt9() {
    let Some(a) = common::load_wav_f32_opt(asset_path!("130418_1742.wav")) else {
        common::skip_or_fail("JT9 recording");
        return;
    };
    let rows: Vec<_> = Decoder::<Jt9>::new(DecodeParams::for_band((1050.0, 1550.0)))
        .decode(&SlotInput::f32(&a))
        .rows
        .into_iter()
        .map(|r| r.decoded)
        .collect();
    assert_dt(
        "JT9",
        &rows,
        &[
            ("JA1KAU PD0JAC -23", 1.2),
            ("TF3G N7MQ CN84", 0.0),
            ("K1JT KF4RWA 73", 0.0),
            ("CQ M0WAY IO82", 0.1),
            ("K1JT N5KDV EM41", 0.1),
            ("G7CNF N4HFA EL89", -0.2),
        ],
    );
}

#[test]
fn jt65_decoder_dt_matches_jt9() {
    let Some(path) = common::corpus::golden_path("jt65/jt65a_5sig_m18.wav") else {
        common::skip_or_fail("JT65 golden recording");
        return;
    };
    let Some(a) = common::load_wav_f32_opt(&path) else {
        return;
    };
    let rows: Vec<_> = Decoder::<Jt65>::new(DecodeParams::for_band((300.0, 2700.0)))
        .decode(&SlotInput::f32(&a))
        .rows
        .into_iter()
        .map(|r| r.decoded)
        .collect();
    assert_dt("JT65", &rows, &[("K1ABC W9XYZ EN37", 0.0)]);
}
