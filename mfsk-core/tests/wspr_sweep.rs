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
#[cfg(feature = "internal-testing")]
use mfsk_core::wspr::decode::{WsprCallsignTable, decode_scan_with_table};

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

/// Harvest every LLR vector that reaches `osd_decode` across the AWGN
/// sweep — real inputs for differential-testing a future bit-packing
/// rewrite of its generator matrix (issue #260, the OSD stack audit
/// in `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`).
///
/// `decode_scan_default` (the sweep's normal driver) never lets OSD
/// fire: OSD only runs when the candidate's callsign is *already*
/// Fano-confirmed, and each sweep trial is a lone, independent file.
/// Seeding a [`WsprCallsignTable`] with the known transmitted
/// callsign+grid *before* decoding — simulating "we've heard this
/// station on an earlier, easier slot", exactly the scenario the
/// cross-slot table exists for — opens that gate legitimately, not as
/// a shortcut: it's the same mechanism `wspr_carried_table_across_
/// slots_adds_no_phantoms` exercises, just applied across an SNR
/// sweep instead of a handful of hand-picked slots.
///
/// Prints the harvested count per SNR cell — that count *is* the
/// answer to "how much real coverage would a differential test have":
/// large and this is a strong validation harness, thin and it isn't.
#[test]
#[ignore]
#[cfg(feature = "internal-testing")]
fn wspr_osd_input_harvest() {
    use mfsk_core::msg::WsprMessage;

    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("skipping wspr_osd_input_harvest: corpus dir not found at {dir:?}");
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
    if jobs.is_empty() {
        eprintln!("skipping wspr_osd_input_harvest: no wspr_awgn_*.wav files found in {dir:?}");
        return;
    }

    mfsk_core::wspr::osd::capture::reset();

    // Callsign/grid parsed straight out of GOLDEN_MSG ("JL1NIE PM95
    // 37") rather than hand-copied, so this can't drift out of sync
    // with the sweep's own transmitted message.
    let (callsign, grid) = {
        let mut parts = GOLDEN_MSG.split_whitespace();
        (
            parts.next().unwrap().to_string(),
            parts.next().unwrap().to_string(),
        )
    };

    let params = mfsk_core::wspr::SearchParams::default();
    let mut per_snr_osd: std::collections::BTreeMap<i32, (u32, u32, u32)> =
        std::collections::BTreeMap::new(); // snr -> (trials, osd_attempts_delta, fano_hits)

    for job in &jobs {
        let Some(audio) = load_wav_f32_opt(&job.path) else {
            continue;
        };
        // Fresh table per trial, seeded with the known station —
        // "we already confirmed this callsign", not "this trial
        // confirmed itself" (which would need Fano to already have
        // succeeded, at which point OSD is moot for that trial).
        let mut table = WsprCallsignTable::new();
        table.record(&WsprMessage::Type1 {
            callsign: callsign.clone(),
            grid: grid.clone(),
            power_dbm: 37,
        });
        let before = mfsk_core::wspr::osd::capture::snapshot().len();
        let results = decode_scan_with_table(&audio, 12_000, 0, &params, &mut table);
        let after = mfsk_core::wspr::osd::capture::snapshot().len();
        let fano_hit = results.iter().any(|d| d.message.to_string() == GOLDEN_MSG);

        let cell = per_snr_osd.entry(job.snr).or_insert((0, 0, 0));
        cell.0 += 1;
        cell.1 += (after - before) as u32;
        if fano_hit {
            cell.2 += 1;
        }
    }

    println!("WSPR OSD-input harvest — {dir:?}");
    println!(
        "{:>6}  {:>7}  {:>14}  {:>9}",
        "SNR", "trials", "osd calls/trial", "fano hit"
    );
    for (snr, (trials, osd_calls, fano_hits)) in &per_snr_osd {
        println!(
            "{:>+5}dB  {:>7}  {:>14.1}  {:>8}/{}",
            snr,
            trials,
            *osd_calls as f32 / *trials as f32,
            fano_hits,
            trials,
        );
    }

    let total = mfsk_core::wspr::osd::capture::snapshot().len();
    println!("\ntotal osd_decode() inputs harvested: {total}");
}
