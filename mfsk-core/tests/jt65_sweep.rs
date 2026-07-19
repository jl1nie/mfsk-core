//! JT65A AWGN SNR sweep against `jt65sim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually when investigating JT65
//! sensitivity. `jt65sim` is WSJT-X's canonical JT65 signal generator
//! (Fortran, `WSJT-X/lib/jt65sim.f90`); this crate has no from-source
//! build script for it (unlike `ft8sim`/`ft4sim`/`fst4sim` — `jt65sim`
//! links the full `wsjt_fort`/`wsjt_cxx` libraries per WSJT-X's
//! `CMakeLists.txt`, not a small standalone subset), so build it via
//! the full WSJT-X CMake build:
//!
//! ```sh
//! cmake --build ~/wsjtx-build --target jt65sim -j"$(nproc)"
//! ```
//!
//! Then:
//!
//! ```sh
//! # 1. Generate WAVs (once, or when widening the SNR grid):
//! scripts/gen_jt65_sweep_wavs.sh ~/wsjtx-build/jt65sim
//!
//! # 2. Run the sweep:
//! cargo test --test jt65_sweep --release --features jt65,fft-rustfft \
//!   -- --ignored --nocapture
//! ```
//!
//! (`MFSK_JT65_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/jt65_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! ## Provenance (2026-07-19, issue #24)
//!
//! This sweep was built as the closing step of the JT65 decode-chain
//! bug hunt: `mfsk-core`'s JT65 `interleave`/`deinterleave` (the 7×9
//! transpose `interleave63.f90` implements) were swapped relative to
//! WSJT-X's TX/RX convention (`gen65.f90`/`jt65sim.f90` call
//! `interleave63(sent, idir=1)` at TX, `extract.f90` calls
//! `interleave63(mrsym, idir=-1)` at RX) — self-consistent (so
//! self-roundtrip tests passed 100%) but not matching the real channel
//! symbol order, so every `jt65sim`-generated signal failed to decode
//! regardless of SNR. Structurally identical to the JT9 encoder bug
//! (#19). Fixed by swapping the two functions' bodies.
//!
//! One false alarm during this investigation, worth recording: passing
//! `-s 0` to `jt65sim` does **not** mean 0 dB — `jt65sim.f90:167-170`
//! special-cases `snrdb.eq.0.0` and substitutes a built-in ~-19 to
//! -21 dB weak-signal default, while still echoing "0.0" in its own
//! console readout. A `0 dB` grid point that looked like a decode gap
//! was actually a `~-20 dB` signal. `scripts/gen_jt65_sweep_wavs.sh`
//! uses `0.01` instead of `0` for the near-0 dB grid point.
//!
//! Output is a recall table — no assertions, statistics only.

#![cfg(all(feature = "jt65", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_f32_opt;
use mfsk_core::jt65::decode_scan_default;

const GOLDEN_CALL1: &str = "CQ";
const GOLDEN_CALL2: &str = "JL1NIE";
const GOLDEN_GRID: &str = "PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_JT65_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/jt65_sweep")
        .to_path_buf()
}

/// Parse `jt65_awgn_<snr_tag>_<trial>.wav`. snr_tag: `m20` = -20, `p10` = +10.
fn parse_snr_tag(tag: &str) -> Option<i32> {
    if let Some(rest) = tag.strip_prefix('m') {
        rest.parse::<i32>().ok().map(|v| -v)
    } else if let Some(rest) = tag.strip_prefix('p') {
        rest.parse::<i32>().ok()
    } else {
        None
    }
}

fn decode_wav_jt65(audio: &[f32]) -> bool {
    decode_scan_default(audio, 12_000).iter().any(|d| {
        matches!(
            &d.message,
            mfsk_core::msg::Jt72Message::Standard { call1, call2, grid_or_report }
                if call1 == GOLDEN_CALL1 && call2 == GOLDEN_CALL2 && grid_or_report == GOLDEN_GRID
        ) && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
    })
}

#[test]
#[ignore]
fn jt65_awgn_snr_sweep() {
    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!(
            "skipping jt65_awgn_snr_sweep: corpus dir not found at {:?}\n\
             Run scripts/gen_jt65_sweep_wavs.sh ~/wsjtx-build/jt65sim",
            dir
        );
        return;
    };

    // snr -> (hits, trials)
    let mut cells: std::collections::BTreeMap<i32, (u32, u32)> = std::collections::BTreeMap::new();

    for entry in entries.flatten() {
        let path = entry.path();
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let Some(rest) = stem.strip_prefix("jt65_awgn_") else {
            continue;
        };
        let Some((tag, _trial)) = rest.split_once('_') else {
            continue;
        };
        let Some(snr) = parse_snr_tag(tag) else {
            continue;
        };
        let Some(audio) = load_wav_f32_opt(&path) else {
            continue;
        };
        let hit = decode_wav_jt65(&audio);
        let cell = cells.entry(snr).or_insert((0, 0));
        cell.1 += 1;
        if hit {
            cell.0 += 1;
        }
    }

    if cells.is_empty() {
        eprintln!(
            "skipping jt65_awgn_snr_sweep: no jt65_awgn_*.wav files found in {:?}",
            dir
        );
        return;
    }

    println!("JT65A AWGN SNR sweep — {:?}", dir);
    println!("{:>6}  {:>10}  {:>6}", "SNR", "hits/trials", "pct");
    for (snr, (hits, trials)) in &cells {
        let pct = *hits as f32 / *trials as f32 * 100.0;
        println!(
            "{:>+5}dB  {:>10}  {:5.1}%",
            snr,
            format!("{hits}/{trials}"),
            pct
        );
    }
}
