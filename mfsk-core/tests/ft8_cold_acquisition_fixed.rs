//! Cold acquisition on the numeric path the board actually runs (#358).
//!
//! `ft8_cold_acquisition.rs` is gated `not(feature = "fixed-point")`,
//! so every measurement in it — including the one that concluded the
//! shipped estimator is fine — was taken in f32. The receiver is a
//! `fixed-point` build, and #280 already recorded that the two differ
//! for exactly the phenomenon at issue here: under `u16` quantisation
//! the far-lag ghosts do not climb into the top-5 the way they do in
//! f32, which is why one of that issue's tests carries a
//! `cfg(not(fixed-point))` of its own.
//!
//! On hardware, acquisition answered `-5.75 s` at `R 0.93` for a grid
//! 1.2 s out. In f32 the same estimator picks a phase that decodes
//! 15-16 messages at every offset tried. One of those is not measuring
//! the shipped receiver, and this file is the one that is.
//!
//! ```sh
//! cargo test -p mfsk-core --features full,fixed-point --release \
//!     --test ft8_cold_acquisition_fixed -- --ignored --nocapture
//! ```
#![cfg(feature = "fixed-point")]

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phase};
use mfsk_core::msg::decode_request::DecodeRequest;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;
use std::path::Path;

const SR: usize = 12_000;
const SLOT: usize = 180_000;
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
const MAX_CAND: usize = 200;

fn roll(audio: &[i16], n: i64) -> Vec<i16> {
    let len = audio.len() as i64;
    (0..audio.len())
        .map(|k| audio[((k as i64 + n).rem_euclid(len)) as usize])
        .collect()
}

fn acquire_tiled(slot: &[i16]) -> Option<(f32, f32)> {
    let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
    while long.len() < REQUIRED_SAMPLES {
        long.extend_from_slice(slot);
    }
    acquire_slot_phase(&long, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 5)
}

/// Decodes obtainable once the grid sits at phase `p`.
fn decodes_at(slot: &[i16], p: f32) -> usize {
    let shifted = roll(slot, (p * SR as f32).round() as i64);
    DecodeRequest::<Ft8>::new(&shifted, FREQ_MIN, FREQ_MAX, SYNC_MIN, 200)
        .decode()
        .results
        .len()
}

/// Does the phase acquisition returns actually decode, on the numeric
/// path the receiver runs?
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn phase_quality_by_decodes_fixed_point() {
    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    assert_eq!(audio.len(), SLOT);

    println!("\n roll | acquired dt /    R | dec | best");
    println!("{:-<44}", "");
    let (mut tot, mut best_tot, mut zero) = (0usize, 0usize, 0usize);
    for i in 0..20 {
        let off_s = i as f32 * 0.75;
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        let est = acquire_tiled(&rolled);
        let d = est.map(|(p, _)| decodes_at(&rolled, p)).unwrap_or(0);
        let b = decodes_at(&rolled, -off_s);
        match est {
            Some((p, r)) => println!("{off_s:5.2} | {p:+11.2} / {r:.2} | {d:3} | {b:3}"),
            None => println!("{off_s:5.2} |        none        | {d:3} | {b:3}"),
        }
        tot += d;
        best_tot += b;
        if d == 0 {
            zero += 1;
        }
    }
    println!(
        "\nacquired: {tot} decodes total, {zero}/20 rolls decoded nothing\n\
         best:     {best_tot} decodes total"
    );
}
