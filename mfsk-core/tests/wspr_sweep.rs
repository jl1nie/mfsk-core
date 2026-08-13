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
//! Output is a recall **and precision** table — no assertions,
//! statistics only. Every corpus file holds exactly one transmitted
//! message, so any other decode is a phantom by construction.

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

/// `(found the transmitted signal, number of decodes that are not it)`.
///
/// Each corpus file contains exactly one transmitted message, so every
/// other decode is a phantom by construction — there is no ambiguity
/// about ground truth the way there is on a real off-air recording.
/// Recall alone cannot see a decoder setting that buys hits by
/// accepting more false codewords, which is exactly the failure mode
/// the Fano cycle budget has at the top of its range (issue #260: at
/// 500 000 cycles/bit the golden file emits four CRC-passing garbage
/// decodes). A recall-only sweep reports that as a *win* right up
/// until recall itself collapses.
fn decode_wav_wspr(audio: &[f32]) -> (bool, u32) {
    let decodes = decode_scan_default(audio, 12_000);
    let mut hit = false;
    let mut phantoms = 0;
    for d in &decodes {
        if d.message.to_string() == GOLDEN_MSG && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
        {
            hit = true;
        } else {
            phantoms += 1;
        }
    }
    (hit, phantoms)
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
    let results: Vec<(i32, (bool, u32))> = jobs
        .par_iter()
        .filter_map(|job| {
            load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav_wspr(&audio)))
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let results: Vec<(i32, (bool, u32))> = jobs
        .iter()
        .filter_map(|job| {
            load_wav_f32_opt(&job.path).map(|audio| (job.snr, decode_wav_wspr(&audio)))
        })
        .collect();

    // snr -> (hits, trials, phantoms, trials_with_a_phantom)
    let mut cells: std::collections::BTreeMap<i32, (u32, u32, u32, u32)> =
        std::collections::BTreeMap::new();
    for (snr, (hit, phantoms)) in results {
        let cell = cells.entry(snr).or_insert((0, 0, 0, 0));
        cell.1 += 1;
        if hit {
            cell.0 += 1;
        }
        cell.2 += phantoms;
        if phantoms > 0 {
            cell.3 += 1;
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
    println!(
        "{:>6}  {:>10}  {:>6}  {:>9}  {:>10}",
        "SNR", "hits/trials", "pct", "phantoms", "dirty/tr"
    );
    let (mut tot_ph, mut tot_dirty, mut tot_tr) = (0u32, 0u32, 0u32);
    for (snr, (hits, trials, phantoms, dirty)) in &cells {
        let pct = *hits as f32 / *trials as f32 * 100.0;
        println!(
            "{:>+5}dB  {:>10}  {:5.1}%  {:>9}  {:>10}",
            snr,
            format!("{hits}/{trials}"),
            pct,
            phantoms,
            format!("{dirty}/{trials}"),
        );
        tot_ph += phantoms;
        tot_dirty += dirty;
        tot_tr += trials;
    }
    // Every corpus file holds one transmitted message, so this is a
    // true false-decode count, not an estimate.
    println!("  TOTAL phantoms {tot_ph} across {tot_tr} trials ({tot_dirty} trial(s) affected)");
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
/// Shared by [`wspr_osd_input_harvest`] and
/// [`wspr_osd_packed_matches_unpacked`]: resets the capture buffer,
/// decodes the whole sweep with a pre-seeded callsign table (opening
/// `osd_decode`'s gate legitimately — see the module doc comment
/// above), and returns per-SNR-cell `(trials, osd_calls, fano_hits)`.
/// The harvested inputs themselves are left in `capture::snapshot()`
/// for the caller to read afterward.
#[cfg(feature = "internal-testing")]
fn harvest_osd_inputs() -> Option<std::collections::BTreeMap<i32, (u32, u32, u32)>> {
    use mfsk_core::msg::WsprMessage;

    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("corpus dir not found at {dir:?}");
        return None;
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
        eprintln!("no wspr_awgn_*.wav files found in {dir:?}");
        return None;
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

    Some(per_snr_osd)
}

#[test]
#[ignore]
#[cfg(feature = "internal-testing")]
fn wspr_osd_input_harvest() {
    let dir = sweep_dir();
    let Some(per_snr_osd) = harvest_osd_inputs() else {
        eprintln!("skipping wspr_osd_input_harvest");
        return;
    };

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

/// The differential test the harvest exists for: every real LLR
/// vector `osd_decode` was actually handed during the sweep must
/// produce a bit-identical result from `osd_decode_packed` — same
/// `Option` variant, same `info`, same `nhardmin`.
///
/// This is the bar the bit-packed rewrite (issue #260's OSD stack
/// audit) has to clear before it can replace the original: not "looks
/// right on a hand-picked case", but "identical on every input the
/// reference implementation was actually asked to solve across an
/// SNR sweep".
#[test]
#[ignore]
#[cfg(feature = "internal-testing")]
fn wspr_osd_packed_matches_unpacked() {
    let Some(_) = harvest_osd_inputs() else {
        eprintln!("skipping wspr_osd_packed_matches_unpacked");
        return;
    };
    let inputs = mfsk_core::wspr::osd::capture::snapshot();
    assert!(!inputs.is_empty(), "harvested zero osd_decode inputs");

    let mut mismatches = 0usize;
    for (i, llrs) in inputs.iter().enumerate() {
        let a = mfsk_core::wspr::osd::osd_decode(llrs);
        let b = mfsk_core::wspr::osd::osd_decode_packed(llrs);
        if a != b {
            mismatches += 1;
            eprintln!("mismatch at input {i}: unpacked={a:?} packed={b:?}");
        }
    }
    println!(
        "checked {} harvested inputs, {mismatches} mismatches",
        inputs.len()
    );
    assert_eq!(mismatches, 0, "osd_decode_packed diverged from osd_decode");
}

/// Where does the real transmitted signal rank among *all* coarse
/// candidates, by refined (post-cascade) sync — across the whole AWGN
/// sweep, not just the one golden file's G8VDQ/W3BI cases?
///
/// Directly extends `wspr_diag_g8vdq_rank_after_minsync2` /
/// `wspr_diag_rank_by_pass` (`wspr_wsjtx_samples.rs`) with real
/// coverage across the marginal-SNR range those two single-file
/// diagnostics couldn't provide — the concern issue #260 kept raising
/// about this corpus applies here too (single-signal, no residual
/// SIC context), so this brackets pass 2's *isolated-weak-candidate*
/// case rather than reproducing pass 2 exactly. `coarse_baseband`'s
/// `maxdrift = 0` matches pass 2's own config either way.
///
/// Also settles a question the golden file alone couldn't: OSD's
/// `osd_ok` counter reads 0 in every device measurement so far
/// (`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s Fano/OSD
/// split) — never seen to contribute a single decode on that one
/// file. `real_decoded_via` records whether each SNR cell's hits come
/// from Fano or OSD, so this checks whether that's a one-file fluke
/// or holds across the sweep.
#[test]
#[ignore]
#[cfg(feature = "internal-testing")]
fn wspr_rank_sweep() {
    use mfsk_core::msg::WsprMessage;
    use mfsk_core::wspr::baseband::decimate_to_baseband;
    use mfsk_core::wspr::coarse_baseband::coarse_baseband;
    use mfsk_core::wspr::decode::{
        WsprCallsignTable, debug_refined_sync, decode_at_baseband_nblocks_gated_drift,
    };
    use mfsk_core::wspr::instrument;

    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("skipping wspr_rank_sweep: corpus dir not found at {dir:?}");
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
        eprintln!("skipping wspr_rank_sweep: no wspr_awgn_*.wav files found in {dir:?}");
        return;
    }

    // Callsign/grid for the OSD-seeded table, parsed from GOLDEN_MSG.
    let (callsign, grid) = {
        let mut parts = GOLDEN_MSG.split_whitespace();
        (
            parts.next().unwrap().to_string(),
            parts.next().unwrap().to_string(),
        )
    };

    const MINSYNC2_FINAL: f32 = 0.10;
    const PAD_SEC: f32 = 3.0;
    let sample_rate = 12_000u32;

    // snr -> Vec<(rank, n_candidates, kept_count, via)>, via: "fano"/"osd"/"none"
    type RankCell<'a> = (Option<usize>, usize, usize, &'a str);
    let mut cells: std::collections::BTreeMap<i32, Vec<RankCell<'_>>> =
        std::collections::BTreeMap::new();

    for job in &jobs {
        let Some(audio) = load_wav_f32_opt(&job.path) else {
            continue;
        };
        let pad = (PAD_SEC * sample_rate as f32) as usize;
        let mut padded = vec![0.0f32; pad + audio.len()];
        padded[pad..].copy_from_slice(&audio);
        let (idat, qdat) = decimate_to_baseband(&padded);

        // Pass-2 config: maxdrift = 0, and a pre-confirmed table —
        // this models a station already heard on an earlier, easier
        // slot, the scenario OSD exists for (same seeding
        // `wspr_osd_input_harvest` uses).
        let mut table = WsprCallsignTable::new();
        table.record(&WsprMessage::Type1 {
            callsign: callsign.clone(),
            grid: grid.clone(),
            power_dbm: 37,
        });

        let mut cands = coarse_baseband(&idat, &qdat, pad, 100, 0);
        cands.truncate(100);

        let mut ranked: Vec<(f32, bool, &'static str)> = Vec::new(); // (sync, kept, via)
        for c in &cands {
            let sync =
                debug_refined_sync(&idat, &qdat, c.start_sample, c.freq_hz, c.drift_hz, false);
            let kept = sync > MINSYNC2_FINAL;
            let before = instrument::snapshot();
            let msg = decode_at_baseband_nblocks_gated_drift(
                &idat,
                &qdat,
                sample_rate,
                c.start_sample,
                c.freq_hz,
                c.drift_hz,
                &[1, 2, 3, 0],
                Some(&table),
                false,
            )
            .filter(|d| d.message.to_string() == GOLDEN_MSG);
            let after = instrument::snapshot();
            let via = if msg.is_none() {
                "none"
            } else if after.osd_ok > before.osd_ok {
                "osd"
            } else {
                "fano"
            };
            ranked.push((sync, kept, via));
        }
        ranked.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

        let n_kept = ranked.iter().filter(|r| r.1).count();
        let hit = ranked
            .iter()
            .enumerate()
            .find(|(_, (_, _, via))| *via != "none");
        let entry = match hit {
            Some((rank, (_, _, via))) => (Some(rank), ranked.len(), n_kept, *via),
            None => (None, ranked.len(), n_kept, "none"),
        };
        cells.entry(job.snr).or_default().push(entry);
    }

    println!("WSPR pass-2-style rank sweep — {dir:?}");
    println!(
        "{:>6}  {:>7}  {:>10}  {:>10}  {:>10}  {:>8}",
        "SNR", "trials", "found", "rank<=1", "rank<=2", "via fano/osd/none"
    );
    for (snr, entries) in &cells {
        let trials = entries.len();
        let found = entries.iter().filter(|(r, ..)| r.is_some()).count();
        let rank_le1 = entries
            .iter()
            .filter(|(r, ..)| r.is_some_and(|r| r <= 1))
            .count();
        let rank_le2 = entries
            .iter()
            .filter(|(r, ..)| r.is_some_and(|r| r <= 2))
            .count();
        let fano = entries.iter().filter(|(.., v)| *v == "fano").count();
        let osd = entries.iter().filter(|(.., v)| *v == "osd").count();
        let none = entries.iter().filter(|(.., v)| *v == "none").count();
        println!(
            "{:>+5}dB  {:>7}  {:>10}  {:>10}  {:>10}  {fano:>4}/{osd:>4}/{none:>4}",
            snr, trials, found, rank_le1, rank_le2,
        );
        for (rank, n_cand, n_kept, via) in entries {
            if let Some(r) = rank
                && *r > 1
            {
                println!("    ^ rank {r} outlier: {n_cand} candidates, {n_kept} kept, via={via}");
            }
        }
    }
}

/// How many DT peak-up x nblocks ladder positions a *real, converging*
/// signal needs, across the AWGN corpus — the number
/// `PASS2_CANDIDATE_TIME_BUDGET_US` (`embedded-shared`'s wspr_bench)
/// needs to safely exceed. Companion to `wspr_rank_sweep`, which
/// proved a real signal always ranks <= 1 among `minsync2` survivors
/// but says nothing about how *long* its own ladder search can take
/// once it's there — that's what this sweep measures, on the exact
/// same corpus and the exact same `rank_pass2_candidates` +
/// `deep_decode_pass2_candidate` pair production uses (`--features
/// wspr-pass2-topn`), not `decode_at_baseband_nblocks_gated_drift`'s
/// unbudgeted path.
///
/// Position count, not wall-clock: portable across host and S3 (the
/// number of `(nblock, idt)` cells visited before Fano/OSD converges
/// is a property of the signal and the search order, not the CPU) —
/// converting to an S3 time budget is a separate, deliberately
/// manual step against S3's own measured per-position cost (see
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`).
///
/// Uses a counting budget (`Fn`, always returns `true`) rather than
/// `None`, so the exact position where convergence happens is
/// visible — matches this session's `wspr_ladder_budget_*` unit
/// test's own technique (`wspr_wsjtx_samples.rs`), just run across
/// the whole corpus instead of one golden file's one candidate.
#[test]
#[ignore = "manual sweep — WSPR pass-2 ladder position count for a converging signal, needs the AWGN corpus"]
#[cfg(feature = "wspr-pass2-topn")]
fn wspr_pass2_ladder_position_sweep() {
    use core::sync::atomic::{AtomicU32, Ordering};

    use mfsk_core::msg::WsprMessage;
    use mfsk_core::wspr::baseband::decimate_to_baseband;
    use mfsk_core::wspr::coarse_baseband::coarse_baseband;
    use mfsk_core::wspr::decode::{
        WsprCallsignTable, deep_decode_pass2_candidate, rank_pass2_candidates,
    };

    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("skipping wspr_pass2_ladder_position_sweep: corpus dir not found at {dir:?}");
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
        eprintln!(
            "skipping wspr_pass2_ladder_position_sweep: no wspr_awgn_*.wav files found in {dir:?}"
        );
        return;
    }

    let (callsign, grid) = {
        let mut parts = GOLDEN_MSG.split_whitespace();
        (
            parts.next().unwrap().to_string(),
            parts.next().unwrap().to_string(),
        )
    };

    const PAD_SEC: f32 = 3.0;
    let sample_rate = 12_000u32;

    // snr -> Vec<Option<positions>> (None = this trial's real signal
    // never converged inside the top-N survivors at all — a rank
    // failure, not a ladder-length one; excluded from the max below,
    // reported separately).
    let mut cells: std::collections::BTreeMap<i32, Vec<Option<u32>>> =
        std::collections::BTreeMap::new();

    for job in &jobs {
        let Some(audio) = load_wav_f32_opt(&job.path) else {
            continue;
        };
        let pad = (PAD_SEC * sample_rate as f32) as usize;
        let mut padded = vec![0.0f32; pad + audio.len()];
        padded[pad..].copy_from_slice(&audio);
        let (idat, qdat) = decimate_to_baseband(&padded);

        let mut table = WsprCallsignTable::new();
        table.record(&WsprMessage::Type1 {
            callsign: callsign.clone(),
            grid: grid.clone(),
            power_dbm: 37,
        });

        let mut cands = coarse_baseband(&idat, &qdat, pad, 100, 0);
        cands.truncate(100);
        let ranked = rank_pass2_candidates(&idat, &qdat, &cands);

        let mut found: Option<u32> = None;
        for r in &ranked {
            let calls = AtomicU32::new(0);
            let counting: &(dyn Fn() -> bool + Sync) = &|| {
                calls.fetch_add(1, Ordering::Relaxed);
                true
            };
            let d = deep_decode_pass2_candidate(
                &idat,
                &qdat,
                sample_rate,
                pad,
                &table,
                r,
                Some(counting),
            );
            if d.is_some_and(|d| d.message.to_string() == GOLDEN_MSG) {
                found = Some(calls.load(Ordering::Relaxed));
                break;
            }
        }
        cells.entry(job.snr).or_default().push(found);
    }

    println!("WSPR pass-2 ladder position count for a converging signal — {dir:?}");
    println!(
        "{:>6}  {:>7}  {:>10}  {:>12}  {:>12}  {:>12}",
        "SNR", "trials", "found", "max pos", "mean pos", "p90 pos"
    );
    let mut overall_max = 0u32;
    for (snr, entries) in &cells {
        let trials = entries.len();
        let mut positions: Vec<u32> = entries.iter().filter_map(|e| *e).collect();
        let found = positions.len();
        positions.sort_unstable();
        let max_pos = positions.last().copied().unwrap_or(0);
        let mean_pos = if found > 0 {
            positions.iter().sum::<u32>() as f64 / found as f64
        } else {
            0.0
        };
        let p90_pos = if found > 0 {
            positions[(found * 9 / 10).min(found - 1)]
        } else {
            0
        };
        overall_max = overall_max.max(max_pos);
        println!(
            "{:>+5}dB  {:>7}  {:>10}  {:>12}  {:>12.1}  {:>12}",
            snr, trials, found, max_pos, mean_pos, p90_pos,
        );
    }
    println!(
        "\noverall max ladder positions across every SNR level, every converging trial: {overall_max}"
    );
    println!(
        "(4 nblocks x 17 idt = 68 total positions possible; a real signal needing all 68 \
         would mean no time budget below the unbudgeted default is ever safe)"
    );
}
