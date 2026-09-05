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

fn acquire_tiled_k(slot: &[i16], top_k: usize) -> Option<(f32, f32)> {
    let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
    while long.len() < REQUIRED_SAMPLES {
        long.extend_from_slice(slot);
    }
    acquire_slot_phase(&long, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, top_k)
}

#[allow(dead_code)]
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

/// Does narrowing the estimate to the *strongest* candidates help?
///
/// The ghosts that poison the top-5 are weaker than the signal they
/// come from — on `qso3_busy` the real peak scored 229.5 against
/// ghosts at 112/78/64/57 — so a smaller `top_k` should stop averaging
/// them in. This costs nothing: `top_k` is already a parameter, and no
/// BP runs either way.
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn top_k_sweep_fixed_point() {
    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    println!("\ntop_k | decodes | rolls with none | R(ok) min..max | R(bad) min..max");
    println!("{:-<74}", "");
    for &k in &[1usize, 2, 3, 5, 8, 12, 20] {
        let (mut tot, mut zero) = (0usize, 0usize);
        let (mut okmin, mut okmax) = (2.0f32, -1.0f32);
        let (mut bmin, mut bmax) = (2.0f32, -1.0f32);
        for i in 0..40 {
            let off_s = i as f32 * 0.375;
            let rolled = roll(&audio, (off_s * SR as f32) as i64);
            match acquire_tiled_k(&rolled, k) {
                Some((p, r)) => {
                    let d = decodes_at(&rolled, p);
                    tot += d;
                    if d == 0 {
                        zero += 1;
                        bmin = bmin.min(r);
                        bmax = bmax.max(r);
                    } else {
                        okmin = okmin.min(r);
                        okmax = okmax.max(r);
                    }
                }
                None => zero += 1,
            }
        }
        println!(
            "{k:5} | {tot:7} | {zero:15} | {okmin:.2}..{okmax:.2}      | {bmin:.2}..{bmax:.2}"
        );
    }
    println!("\n(40 offsets at 0.375 s; best achievable is ~600 decodes, 0 rolls with none)");
}

/// With one candidate per tile the mean-resultant `R` is degenerate
/// (a lone angle always agrees with itself), so this asks whether the
/// candidate's own coarse **score** works as the confidence instead.
///
/// It measures a different thing: `R` asks whether candidates agree,
/// the score asks whether this one is a signal — which is the question
/// the gate actually needs answered, and the one every agreement-based
/// statistic tried so far has failed (#358).
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn top1_score_as_confidence() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Strongest candidate across the tiles, with its score.
    let top1 = |slot: &[i16]| -> Option<(f32, f32)> {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        let mut best: Option<SyncCandidate> = None;
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let spec = compute_spectrogram(&long[start..start + SLOT], FREQ_MAX);
            for c in coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5) {
                if best.as_ref().is_none_or(|b| c.score > b.score) {
                    best = Some(SyncCandidate {
                        freq_hz: c.freq_hz,
                        dt_sec: c.dt_sec + w,
                        score: c.score,
                    });
                }
            }
        }
        best.map(|c| (c.dt_sec, c.score))
    };

    println!("\n roll |     dt |  score | decodes");
    println!("{:-<40}", "");
    let (mut ok, mut bad): (Vec<f32>, Vec<f32>) = (Vec::new(), Vec::new());
    for i in 0..40 {
        let off_s = i as f32 * 0.375;
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        if let Some((dt, sc)) = top1(&rolled) {
            let d = decodes_at(&rolled, dt);
            println!("{off_s:5.2} | {dt:+6.2} | {sc:6.1} | {d:3}");
            if d == 0 { bad.push(sc) } else { ok.push(sc) }
        }
    }
    let stat = |v: &mut Vec<f32>| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        (
            v.first().copied().unwrap_or(0.0),
            v.last().copied().unwrap_or(0.0),
        )
    };
    let (okmin, okmax) = stat(&mut ok);
    let (bmin, bmax) = stat(&mut bad);
    println!(
        "\ndecoding rolls   (n={}): score {okmin:.1}..{okmax:.1}\n\
         non-decoding     (n={}): score {bmin:.1}..{bmax:.1}",
        ok.len(),
        bad.len()
    );
}

/// Is an absolute score gate safe on a weak band? (#358)
///
/// The worry was that a threshold tuned on this fixture would close
/// permanently where signals are weaker. But a band whose best signal
/// cannot be decoded has no grid to acquire, so the gate closing there
/// is correct rather than harmful — *if* score and decodability fall
/// together. This adds noise to the real recording (rather than
/// attenuating it, which leaves SNR untouched, or synthesising a clean
/// one, which is not a trustworthy instrument) and watches both.
#[test]
#[ignore = "diagnostic, decodes at every noise level — slow"]
fn score_tracks_decodability_as_snr_falls() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

    let clean = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Deterministic white noise, amplitude `amp` in i16 counts.
    let noisy = |amp: i32| -> Vec<i16> {
        let mut x: u32 = 0x2545_F491;
        clean
            .iter()
            .map(|&s| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                // Two draws summed: closer to Gaussian than one.
                let n = ((x >> 16) as i32 & 0xff) + ((x >> 8) as i32 & 0xff) - 255;
                (s as i32 + n * amp / 255).clamp(-32768, 32767) as i16
            })
            .collect()
    };

    // Top-1 score, and the same relative to the median candidate in
    // its own tile — a scale-free version that should not care how
    // loud the band is, only how far the winner stands above it.
    let top1_score = |slot: &[i16]| -> (f32, f32) {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        let mut best: Option<SyncCandidate> = None;
        let mut best_ratio = 0.0f32;
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let spec = compute_spectrogram(&long[start..start + SLOT], FREQ_MAX);
            let cs = coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5);
            if cs.is_empty() {
                continue;
            }
            let mut sc: Vec<f32> = cs.iter().map(|c| c.score).collect();
            sc.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = sc[sc.len() / 2].max(1e-6);
            for c in cs {
                if best.as_ref().is_none_or(|b| c.score > b.score) {
                    best_ratio = c.score / med;
                    best = Some(c);
                }
            }
        }
        (best.map(|c| c.score).unwrap_or(0.0), best_ratio)
    };

    println!("\n noise amp | top-1 score | top1/median | decodes at the true grid");
    println!("{:-<66}", "");
    for &amp in &[0i32, 200, 400, 800, 1600, 3200, 6400] {
        let a = noisy(amp);
        let (sc, ratio) = top1_score(&a);
        println!(
            "{amp:10} | {sc:11.1} | {ratio:11.2} | {:3}",
            decodes_at(&a, 0.0)
        );
    }
    println!(
        "\nIf the two columns fall together, a band too weak to gate through\n\
         is also a band with nothing to acquire, and an absolute gate is safe."
    );
}
