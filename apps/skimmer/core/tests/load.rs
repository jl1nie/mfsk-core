// SPDX-License-Identifier: GPL-3.0-or-later
//! How long N FT8 decoders take when every slot ends at the same instant, as
//! they do on a 15 s grid: `cargo test --release -p skimmer-core --test load -- --ignored --nocapture`.
//! Measures the decode only (no IQ, no channelizer), on a busy-band recording.

use std::sync::{Arc, Barrier};
use std::time::Instant;

use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, SlotInput};

fn recording() -> Vec<i16> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../embedded-poc/assets/qso3_busy.wav"
    );
    let b = std::fs::read(path).expect("qso3_busy.wav");
    // 16-bit mono 12 kHz after a 44-byte header.
    b[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| i16::from_le_bytes(*c))
        .collect()
}

fn round(n: usize, audio: &Arc<Vec<i16>>) -> (f64, f64, usize) {
    let go = Arc::new(Barrier::new(n + 1));
    let handles: Vec<_> = (0..n)
        .map(|_| {
            let (go, audio) = (go.clone(), audio.clone());
            std::thread::spawn(move || {
                let mut d = AnyDecoder::with_defaults(Mode::Ft8);
                go.wait();
                let t = Instant::now();
                let rows = d.decode(&SlotInput::i16(&audio)).rows.len();
                (t.elapsed().as_secs_f64(), rows)
            })
        })
        .collect();
    go.wait();
    let t = Instant::now();
    let r: Vec<(f64, usize)> = handles.into_iter().map(|h| h.join().unwrap()).collect();
    let wall = t.elapsed().as_secs_f64();
    let slowest = r.iter().map(|x| x.0).fold(0.0, f64::max);
    (wall, slowest, r[0].1)
}

#[test]
#[ignore]
fn n_decoders_at_once() {
    let audio = Arc::new(recording());
    let cores = std::thread::available_parallelism().map_or(0, |n| n.get());
    println!(
        "{} s of audio, {cores} threads available",
        audio.len() as f64 / 12_000.0
    );
    for n in [1usize, 4, 8, 16, 24, 40, 64] {
        let _ = round(n, &audio); // warm up: plans, caches
        let runs: Vec<_> = (0..3).map(|_| round(n, &audio)).collect();
        let wall = runs.iter().map(|r| r.0).fold(f64::MAX, f64::min);
        let slow = runs.iter().map(|r| r.1).fold(f64::MAX, f64::min);
        println!(
            "{n:>3} decoders: all done in {wall:>6.2} s (slowest {slow:>6.2} s), {} decodes each",
            runs[0].2
        );
    }
}

/// 40 slots ending together, but only `k` allowed to decode at a time.
fn gated(jobs: usize, k: usize, audio: &Arc<Vec<i16>>) -> f64 {
    let next = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let t = Instant::now();
    let hs: Vec<_> = (0..k)
        .map(|_| {
            let (next, audio) = (next.clone(), audio.clone());
            std::thread::spawn(move || {
                let mut d = AnyDecoder::with_defaults(Mode::Ft8);
                while next.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < jobs {
                    let _ = d.decode(&SlotInput::i16(&audio));
                }
            })
        })
        .collect();
    for h in hs {
        h.join().unwrap();
    }
    t.elapsed().as_secs_f64()
}

#[test]
#[ignore]
fn forty_slots_with_a_gate() {
    let audio = Arc::new(recording());
    for k in [2usize, 3, 4, 6, 8, 12, 24, 40] {
        let _ = gated(k, k, &audio);
        let best = (0..2)
            .map(|_| gated(40, k, &audio))
            .fold(f64::MAX, f64::min);
        println!("40 slots, {k:>2} at a time: {best:>6.2} s");
    }
}
