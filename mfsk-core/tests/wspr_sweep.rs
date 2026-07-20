//! WSPR AWGN SNR sweep against `wsprsim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually when investigating WSPR
//! sensitivity. WSJT-X ships *two* WSPR simulators; CMakeLists.txt
//! only wires up `lib/wsprd/wsprsim.c`, which writes `.c2`
//! complex-baseband files meant for `wsprd`, not WAV audio this
//! crate's decode path can consume. The other one,
//! `lib/wsprd/wsprsimf.f90` (Fortran, no CMake target — same
//! "orphaned" situation as `jt9sim`), has a `nwav=1` branch that
//! writes real 12 kHz PCM16 WAV via `wspr_wav.f90`, matching every
//! other `*sim` tool's output format. Build it with:
//!
//! ```sh
//! scripts/build_wsprsim.sh
//! ```
//!
//! Then:
//!
//! ```sh
//! # 1. Generate WAVs (once, or when widening the SNR grid):
//! scripts/gen_wspr_sweep_wavs.sh
//!
//! # 2. Run the sweep:
//! cargo test --test wspr_sweep --release --features wspr,fft-rustfft,parallel \
//!   -- --ignored --nocapture
//! ```
//!
//! (`MFSK_WSPR_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/wspr_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! Before this test, WSPR's only objective validation was
//! `wspr_wsjtx_samples.rs` (a single real-world WAV, 8/8 golden
//! recall at whatever SNR that recording happens to carry) — unlike
//! every other supported protocol (FT8/FT4/JT9/JT65/Q65/MSK144/FST4),
//! which each have a `*sim`-driven AWGN sweep giving a real
//! SNR-vs-recall curve. This closes that gap.
//!
//! Output is a recall table — no assertions, statistics only.

#![cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_f32_opt;
use mfsk_core::wspr::decode::decode_scan_default;

const GOLDEN_MSG: &str = "JL1NIE PM95 37";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 4.0;

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_WSPR_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/wspr_sweep")
        .to_path_buf()
}

/// Parse `wspr_awgn_<snr_tag>_<trial>.wav`. snr_tag: `m28` = -28, `p05` = +5.
fn parse_snr_tag(tag: &str) -> Option<i32> {
    if let Some(rest) = tag.strip_prefix('m') {
        rest.parse::<i32>().ok().map(|v| -v)
    } else if let Some(rest) = tag.strip_prefix('p') {
        rest.parse::<i32>().ok()
    } else {
        None
    }
}

fn decode_wav_wspr(audio: &[f32]) -> bool {
    decode_scan_default(audio, 12_000).iter().any(|d| {
        d.message.to_string() == GOLDEN_MSG && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
    })
}

struct Job {
    snr: i32,
    path: PathBuf,
}

#[test]
#[ignore]
fn wspr_awgn_snr_sweep() {
    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!(
            "skipping wspr_awgn_snr_sweep: corpus dir not found at {:?}\n\
             Run scripts/build_wsprsim.sh then scripts/gen_wspr_sweep_wavs.sh",
            dir
        );
        return;
    };

    let mut jobs = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let Some(rest) = stem.strip_prefix("wspr_awgn_") else {
            continue;
        };
        let Some((tag, _trial)) = rest.split_once('_') else {
            continue;
        };
        let Some(snr) = parse_snr_tag(tag) else {
            continue;
        };
        jobs.push(Job { snr, path });
    }

    // Corpus is small enough that a sequential run finishes in
    // seconds, but the load+decode step parallelizes for free with
    // the same rayon pattern the larger sweeps use (`ft8_sweep.rs`,
    // `q65_sim_sweep.rs`).
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    #[cfg(feature = "parallel")]
    let results: Vec<(i32, bool)> = jobs
        .par_iter()
        .filter_map(|job| {
            load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav_wspr(&audio)))
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let results: Vec<(i32, bool)> = jobs
        .iter()
        .filter_map(|job| {
            load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav_wspr(&audio)))
        })
        .collect();

    // snr -> (hits, trials)
    let mut cells: std::collections::BTreeMap<i32, (u32, u32)> = std::collections::BTreeMap::new();
    for (snr, hit) in results {
        let cell = cells.entry(snr).or_insert((0, 0));
        cell.1 += 1;
        if hit {
            cell.0 += 1;
        }
    }

    if cells.is_empty() {
        eprintln!(
            "skipping wspr_awgn_snr_sweep: no wspr_awgn_*.wav files found in {:?}",
            dir
        );
        return;
    }

    println!("WSPR AWGN SNR sweep — {:?}", dir);
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
