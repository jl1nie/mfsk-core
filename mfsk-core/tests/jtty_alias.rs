// SPDX-License-Identifier: GPL-3.0-only
//! Alias rejection in the JTTY frame decoder (tier B, a signal made here), as WSJT-X v3.3.0-beta6's
//! `tests/unit/jtty/test_jtty_alias_decode.f90` (PR #702) builds it.
//!
//! A strong transmission decoded in channel 0 used to leave repeatable false frames such as `IG4QXF TU`:
//! a sync candidate that lands a whole number of symbols inside an already decoded frame of the same
//! signal matches 7 of 13 sync tones by chance and still yields a TBCC codeword. `Work::is_frame_alias`
//! (`jtty/rx.rs`, upstream `is_frame_alias`) refuses them. Two cases pin the trade-off:
//!
//! 1. the fox message at -3 dB in 2 500 Hz, 24 seeded noise trials: every decoded frame must be a
//!    fragment of the transmitted text (`max_extra: 0`), and the whole message must come out in all but
//!    one trial;
//! 2. a second station of the same level 30 Hz away (inside the alias window) and 18.5 symbols later: both
//!    must decode completely. A real station overlapping a strong one does not match its tones.

#![cfg(all(feature = "jtty", any(feature = "fft-rustfft", feature = "fft-extern")))]

use mfsk_core::jtty::pack::{self, ExchangeProfile};
use mfsk_core::jtty::rx::{FrameDecode, Params, Receiver};
use mfsk_core::jtty::source::{Atom, render_message};
use mfsk_core::jtty::tx;

const FOX: &str = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG";
const WEAK: &str = "CQ DL3WDG JN68";
const NSPS: usize = 384;

/// Park-Miller LCG and Box-Muller, as the upstream test (deterministic, no overflow).
struct Noise(u64);
impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 = self.0 * 16_807 % 2_147_483_647;
        self.0 as f64 / 2_147_483_647.0
    }
    fn gauss(&mut self) -> f32 {
        let (u1, u2) = (self.uniform(), self.uniform());
        ((-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()) as f32
    }
}

fn tones(text: &str) -> Vec<u8> {
    pack::tones(text, ExchangeProfile::Unknown)
        .expect("packs")
        .expect("not empty")
}

/// Frames of one transmission, as `tests/jtty_rx.rs` joins them: on one frequency, one frame period apart.
fn messages(frames: &[FrameDecode]) -> Vec<String> {
    let mut sorted: Vec<&FrameDecode> = frames.iter().collect();
    sorted.sort_by(|a, b| a.tsync_s.total_cmp(&b.tsync_s));
    let mut groups: Vec<Vec<&FrameDecode>> = Vec::new();
    for f in sorted {
        let fits = |g: &Vec<&FrameDecode>| {
            let last = g.last().unwrap();
            !last.eom
                && (f.f1_hz - last.f1_hz).abs() < 12.0
                && (f.tsync_s - last.tsync_s - 1.888).abs() < 0.1
        };
        match groups.iter_mut().find(|g| fits(g)) {
            Some(g) => g.push(f),
            None => groups.push(vec![f]),
        }
    }
    groups
        .into_iter()
        .map(|g| {
            let atoms: Vec<Atom> = g.iter().map(|f| f.atom.clone()).collect();
            render_message(&atoms)
        })
        .collect()
}

fn fox_in_noise(snr_db: f32, seed: u64) -> Vec<i16> {
    let wave = tx::synth_f32(&tones(FOX), 1500.0, 1.0);
    let start = 6000; // 0.5 s
    let total = start + wave.len() + 2 * 59 * NSPS;
    // unit-variance noise, 2 500 Hz reference: signal power A^2 / 2 against sigma^2 * 2500 / 6000
    let sig = (2.0f32 * 2500.0 / 6000.0).sqrt() * 10f32.powf(0.05 * snr_db);
    let mut noise = Noise(seed);
    (0..total)
        .map(|i| {
            let s = i
                .checked_sub(start)
                .and_then(|k| wave.get(k))
                .copied()
                .unwrap_or(0.0);
            (100.0 * (noise.gauss() + sig * s))
                .round()
                .clamp(-32767.0, 32767.0) as i16
        })
        .collect()
}

#[test]
fn a_strong_signal_leaves_no_alias_frames() {
    let rx = Receiver::new();
    let (trials, mut full, mut false_frames) = (24, 0, 0);
    for trial in 1..=trials {
        let audio = fox_in_noise(-3.0, 1000 + trial);
        let frames = rx.scan(&audio, &Params::default());
        for f in &frames {
            let t = f.atom.render();
            if !FOX.contains(t.trim()) {
                false_frames += 1;
                eprintln!("false frame: seed {trial}: {t:?}");
            }
        }
        full += usize::from(messages(&frames).iter().any(|m| m == FOX));
    }
    eprintln!("trials {trials}, full {full}, false frames {false_frames}");
    assert_eq!(
        false_frames, 0,
        "no false (alias) frames from a strong signal"
    );
    assert!(
        full + 1 >= trials as usize,
        "the message decodes in all but one trial: {full}"
    );
}

#[test]
fn a_station_at_a_fractional_symbol_offset_survives() {
    let (fox, weak) = (
        tx::synth_f32(&tones(FOX), 1500.0, 14_000.0),
        tx::synth_f32(&tones(WEAK), 1530.0, 12_000.0),
    );
    let fox_start = 2400;
    let weak_start = fox_start + 37 * NSPS / 2; // 18.5 symbols later
    let total = (fox_start + fox.len()).max(weak_start + weak.len()) + 2 * 59 * NSPS;
    let audio: Vec<i16> = (0..total)
        .map(|i| {
            let a = i
                .checked_sub(fox_start)
                .and_then(|k| fox.get(k))
                .copied()
                .unwrap_or(0.0);
            let b = i
                .checked_sub(weak_start)
                .and_then(|k| weak.get(k))
                .copied()
                .unwrap_or(0.0);
            (a + b).round().clamp(-32767.0, 32767.0) as i16
        })
        .collect();
    let msgs = messages(&Receiver::new().scan(&audio, &Params::default()));
    assert!(
        msgs.iter().any(|m| m == FOX),
        "the strong station: {msgs:?}"
    );
    assert!(msgs.iter().any(|m| m == WEAK), "the weaker one: {msgs:?}");
}
