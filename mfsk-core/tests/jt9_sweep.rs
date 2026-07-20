//! JT9 AWGN SNR sweep against `jt9sim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually when investigating JT9
//! sensitivity. `jt9sim` is WSJT-X's canonical JT9 signal generator
//! (Fortran, `WSJT-X/lib/jt9sim.f90`); unlike `ft8sim`/`ft4sim`/
//! `fst4sim`/`jt65sim`, it has **no CMake target at all** in WSJT-X's
//! own build — not linked against the full `wsjt_fort` library either.
//! `scripts/build_jt9sim.sh` assembles just its actual dependency
//! closure (`gen9` → `packjt`/`entail`/`encode232`/`interleave9`/
//! `graycode`, plus `jt9fano`/`fano232` for jt9sim's own internal
//! self-verify decode step) as a standalone binary, mirroring how
//! `build_fst4sim.sh` does the same for FST4.
//!
//! ```sh
//! # 1. Build jt9sim (once):
//! scripts/build_jt9sim.sh ~/src/WSJT-X
//!
//! # 2. Generate WAVs (once, or when widening the SNR grid):
//! scripts/gen_jt9_sweep_wavs.sh
//!
//! # 3. Run the sweep:
//! cargo test --test jt9_sweep --release --features jt9,fft-rustfft,uvpacket,parallel \
//!   -- --ignored --nocapture
//! ```
//!
//! (`MFSK_JT9_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/jt9_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! ## Provenance (2026-07-19)
//!
//! Built as the JT9 counterpart to `tests/jt65_sweep.rs`, itself the
//! closing step of the JT65 decode-chain bug hunt (#24, fixed in
//! #168). Cross-checked `decode_scan_default` against a freshly
//! `jt9sim`-generated signal ("CQ JL1NIE PM95" @ 1400 Hz, jt9sim's
//! hardcoded center frequency) as an independent-reference sanity
//! check beyond the existing golden-WAV test
//! (`tests/jt9_wsjtx_samples.rs`, one real recording with only 5
//! entries and band congestion) — decodes cleanly, matching WSJT-X's
//! own `jt9 -9` output exactly. This also re-confirms the #19 encoder
//! fix (`pack_grid4_plain`/`unpack_grid`,
//! [[project_jt9_encoder_bug_19_root_cause]]) holds against a second,
//! independently-built reference encoder, not just the one real WAV
//! recording used when that bug was originally closed.
//!
//! Unlike `jt65sim`, `jt9sim` has no `-s 0` sentinel gotcha — its
//! `snrdb` argument only special-cases `>= 90` (noiseless), so `0` in
//! the grid below is genuinely 0 dB.
//!
//! Output is a recall table — no assertions, statistics only.

#![cfg(all(feature = "jt9", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_f32_opt;
use mfsk_core::jt9::decode_scan_default;

const GOLDEN_CALL1: &str = "CQ";
const GOLDEN_CALL2: &str = "JL1NIE";
const GOLDEN_GRID: &str = "PM95";
const GOLDEN_FREQ_HZ: f32 = 1400.0;
const FREQ_TOL_HZ: f32 = 5.0;

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_JT9_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/jt9_sweep")
        .to_path_buf()
}

/// Parse `jt9_awgn_<snr_tag>_<trial>.wav`. snr_tag: `m20` = -20, `p10` = +10.
fn parse_snr_tag(tag: &str) -> Option<i32> {
    if let Some(rest) = tag.strip_prefix('m') {
        rest.parse::<i32>().ok().map(|v| -v)
    } else if let Some(rest) = tag.strip_prefix('p') {
        rest.parse::<i32>().ok()
    } else {
        None
    }
}

fn decode_wav_jt9(audio: &[f32]) -> bool {
    decode_scan_default(audio, 12_000).iter().any(|d| {
        matches!(
            &d.message,
            mfsk_core::msg::Jt72Message::Standard { call1, call2, grid_or_report }
                if call1 == GOLDEN_CALL1 && call2 == GOLDEN_CALL2 && grid_or_report == GOLDEN_GRID
        ) && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
    })
}

struct Job {
    snr: i32,
    path: PathBuf,
}

#[test]
#[ignore]
fn jt9_awgn_snr_sweep() {
    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!(
            "skipping jt9_awgn_snr_sweep: corpus dir not found at {:?}\n\
             Run scripts/build_jt9sim.sh ~/src/WSJT-X && scripts/gen_jt9_sweep_wavs.sh",
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
        let Some(rest) = stem.strip_prefix("jt9_awgn_") else {
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
            load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav_jt9(&audio)))
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let results: Vec<(i32, bool)> = jobs
        .iter()
        .filter_map(|job| {
            load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav_jt9(&audio)))
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
            "skipping jt9_awgn_snr_sweep: no jt9_awgn_*.wav files found in {:?}",
            dir
        );
        return;
    }

    println!("JT9 AWGN SNR sweep — {:?}", dir);
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
