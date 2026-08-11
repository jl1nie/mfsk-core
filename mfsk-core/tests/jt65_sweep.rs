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
//! cargo test --test jt65_sweep --release --features jt65,fft-rustfft,parallel \
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
use mfsk_core::jt65::search::SearchParams;
use mfsk_core::jt65::{decode_scan_chase_default, decode_scan_default};

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

fn is_golden(d: &mfsk_core::jt65::Jt65Result) -> bool {
    matches!(
        &d.message,
        mfsk_core::msg::Jt72Message::Standard { call1, call2, grid_or_report }
            if call1 == GOLDEN_CALL1 && call2 == GOLDEN_CALL2 && grid_or_report == GOLDEN_GRID
    ) && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
}

fn decode_wav_jt65(audio: &[f32]) -> bool {
    decode_scan_default(audio, 12_000).iter().any(is_golden)
}

/// Like [`decode_wav_jt65`] but via the stochastic Chase decoder
/// (`decode_scan_chase_default`) — see `jt65_chase_awgn_snr_sweep`.
fn decode_wav_jt65_chase(audio: &[f32]) -> bool {
    decode_scan_chase_default(audio, 12_000)
        .iter()
        .any(is_golden)
}

struct Job {
    snr: i32,
    path: PathBuf,
}

fn collect_jobs(dir: &Path) -> Option<Vec<Job>> {
    let entries = std::fs::read_dir(dir).ok()?;
    let mut jobs = Vec::new();
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
        jobs.push(Job { snr, path });
    }
    Some(jobs)
}

/// Runs the AWGN corpus through `decode_wav` and prints a recall
/// table, `label`-tagged. Shared by [`jt65_awgn_snr_sweep`] (plain
/// zero-erasure hard decision, via `decode_scan_default`) and
/// [`jt65_chase_awgn_snr_sweep`] (stochastic Chase decoder, via
/// `decode_scan_chase_default`) so both run identical methodology
/// against the same corpus — the only difference is `decode_wav`.
fn run_sweep(label: &str, decode_wav: impl Fn(&[f32]) -> bool + Sync) {
    let dir = sweep_dir();
    let Some(jobs) = collect_jobs(&dir) else {
        eprintln!(
            "skipping {label}: corpus dir not found at {:?}\n\
             Run scripts/gen_jt65_sweep_wavs.sh ~/wsjtx-build/jt65sim",
            dir
        );
        return;
    };

    // Corpus is small enough that a sequential run finishes in
    // seconds, but the load+decode step parallelizes for free with
    // the same rayon pattern the larger sweeps use (`ft8_sweep.rs`,
    // `q65_sim_sweep.rs`).
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    #[cfg(feature = "parallel")]
    let results: Vec<(i32, bool)> = jobs
        .par_iter()
        .filter_map(|job| load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav(&audio))))
        .collect();

    #[cfg(not(feature = "parallel"))]
    let results: Vec<(i32, bool)> = jobs
        .iter()
        .filter_map(|job| load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav(&audio))))
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
            "skipping {label}: no jt65_awgn_*.wav files found in {:?}",
            dir
        );
        return;
    }

    println!("{label} — {:?}", dir);
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

#[test]
#[ignore]
fn jt65_awgn_snr_sweep() {
    run_sweep(
        "JT65A AWGN SNR sweep (decode_scan_default)",
        decode_wav_jt65,
    );
}

/// Same corpus, same methodology as [`jt65_awgn_snr_sweep`], but via
/// the stochastic Chase decoder (issue #169 port —
/// `mfsk_core::jt65::chase`). Run both and compare tables directly to
/// measure the real recall improvement (or lack thereof) — see that
/// module's doc comment for why this is an honest-measurement
/// deliverable, not an assumed one.
#[test]
#[ignore]
fn jt65_chase_awgn_snr_sweep() {
    run_sweep(
        "JT65A AWGN SNR sweep (decode_scan_chase_default)",
        decode_wav_jt65_chase,
    );
}

/// Reported SNR must track the injected SNR the corpus was generated
/// at (issue #255).
///
/// Unlike the recall sweeps above this one **asserts**, because it is
/// guarding a number that was measured against real `jt9` and can
/// silently drift. Not `#[ignore]`d for the same reason — but it
/// skips cleanly when the (gitignored, regenerable) corpus is absent,
/// exactly like they do.
///
/// Provenance of the reference: `jt65sim`'s `-s` is an S/N in WSJT-X's
/// 2500 Hz reference bandwidth, and a real local `jt9 -6 -b A` build
/// reports within ~1 dB of it across this range (measured 2026-08-11:
/// injected -8/-12/-16/-20/-23 came back -9/-13/-17/-20/-24). So
/// asserting against the injected value is asserting against real
/// `jt9`, to within that ~1 dB.
///
/// **Restricted to -22…-10 dB on purpose.** Outside it the injected
/// value stops being a usable reference:
///
/// - Above ~-5 dB real `jt9` *saturates* — `jt65_decode.f90:254-255`
///   clamps its display to `-1`, and a +10 dB and a +5 dB signal both
///   come back `-1`. Nothing to track there.
/// - Below -22 dB decodes get too sparse for a stable per-cell mean.
///
/// That -22…-10 window is where JT65 actually operates, and where
/// `jt65::rx::demodulate_aligned_with_confidence_and_snr`'s estimator
/// (a signal/noise power ratio with a `10·log10(2500/TONE_SPACING_HZ)`
/// bandwidth offset — *not* the generic `engine::llr::compute_snr_db`
/// heuristic issue #255 assumed) lands within ~0.7 dB of real `jt9`.
#[test]
fn jt65_reported_snr_tracks_injected() {
    /// Per-cell mean error budget. Measured spread is -0.35…-1.64 dB
    /// across the window, so this catches a real break (the JT9
    /// sibling bug was ~32 dB) with room for corpus-regeneration
    /// noise, without being loose enough to be meaningless.
    const MEAN_ERR_TOL_DB: f32 = 2.5;
    const MIN_DECODES_PER_CELL: usize = 5;

    let jobs = collect_jobs(&sweep_dir()).unwrap_or_default();
    if jobs.is_empty() {
        eprintln!(
            "skipping jt65_reported_snr_tracks_injected: no jt65_awgn_*.wav in {:?} \
             (regenerate with scripts/gen_jt65_sweep_wavs.sh)",
            sweep_dir()
        );
        return;
    }

    let params = SearchParams {
        freq_min_hz: GOLDEN_FREQ_HZ - 100.0,
        freq_max_hz: GOLDEN_FREQ_HZ + 100.0,
        ..SearchParams::default()
    };

    let mut cells: std::collections::BTreeMap<i32, Vec<f32>> = std::collections::BTreeMap::new();
    for job in &jobs {
        if !(-22..=-10).contains(&job.snr) {
            continue;
        }
        let Some(audio) = load_wav_f32_opt(&job.path) else {
            continue;
        };
        let decodes = mfsk_core::jt65::decode_scan(&audio, 12_000, 0, &params);
        if let Some(d) = decodes
            .iter()
            .find(|d| (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
        {
            cells
                .entry(job.snr)
                .or_default()
                .push(d.snr_db - job.snr as f32);
        }
    }

    assert!(
        !cells.is_empty(),
        "corpus present but produced no JT65 decodes in -22..-10 dB — recall regressed, \
         so this test could not measure what it exists to measure"
    );

    let mut worst: Option<(i32, f32)> = None;
    for (snr, errs) in &cells {
        if errs.len() < MIN_DECODES_PER_CELL {
            continue;
        }
        let mean = errs.iter().sum::<f32>() / errs.len() as f32;
        eprintln!(
            "  injected {snr:>4} dB  n={:<3} mean err {mean:+.2} dB",
            errs.len()
        );
        if worst.is_none_or(|(_, w)| mean.abs() > w.abs()) {
            worst = Some((*snr, mean));
        }
    }

    let Some((snr, mean)) = worst else {
        panic!("no -22..-10 dB cell reached {MIN_DECODES_PER_CELL} decodes");
    };
    assert!(
        mean.abs() <= MEAN_ERR_TOL_DB,
        "JT65 reported SNR at injected {snr} dB is off by {mean:+.2} dB on average \
         (tolerance ±{MEAN_ERR_TOL_DB}) — check \
         `jt65::rx::demodulate_aligned_with_confidence_and_snr`'s bandwidth offset \
         and the WSJT-X [-30,-1] display clamp (jt65_decode.f90:254-255)"
    );
}
