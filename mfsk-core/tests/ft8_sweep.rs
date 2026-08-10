//! FT8 SNR sweep + fading benchmark against `ft8sim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually when investigating FT8
//! sensitivity. What this sweep gives FT8: a true Watterson-fading
//! AWGN/CCIR corpus generated from WSJT-X's own `ft8sim`, as opposed to the
//! existing CI "ft8 characterization" suite (`ft8_decode_block_snr_sweep`
//! and friends), which is homegrown LCG-noise synthesis — not validated
//! against any WSJT-X-native ground truth.
//!
//! **Update (2026-08-10, issue #253)**: the claim that used to sit here —
//! that FT8's OSD gate isn't reachable from an external probe because
//! `process_candidate` isn't `pub` — was wrong for the `DecodeRequest`
//! entry point specifically: `.strictness(s)` on `DecodeRequest<Ft8>`
//! reaches the exact same shared gate (`DecodeStrictness::ft8_nharderrors_max`,
//! called from `ft8::decode_block::process_candidates`/`osd_strategy`,
//! which `ft8::decode::decode_frame_inner` also routes through). See
//! `ft8_strictness_probe` below, added after a reproducible false decode
//! (`7Y8CIH HN1GD OP30` on `qso3_busy.wav` via WebFT8's `Deep` +
//! `.sic_early()` phase-2 pipeline, `hard_errors=31` under `Deep`'s
//! `ft8_nharderrors_max=40` — a ceiling the type's own doc comment already
//! flagged as "not yet swept against a fading corpus").
//!
//! ```sh
//! # 1. Generate WAVs (once, or when widening the SNR grid):
//! scripts/build_ft8sim.sh
//! scripts/gen_ft8_sweep_wavs.sh
//!
//! # 2. Run the sweep:
//! cargo test --test ft8_sweep --release --features ft8,fft-rustfft,parallel,uvpacket \
//!   -- --ignored --nocapture
//! ```
//!
//! (`uvpacket` is only required because `tests/common/channel.rs`, pulled in
//! via `mod common`, unconditionally imports `mfsk_core::uvpacket` — unrelated
//! to FT8 itself. `MFSK_FT8_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/ft8_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! Output is a recall table — no assertions, statistics only. Set
//! `MFSK_FT8_SWEEP_CSV=/path/out.csv` to also dump raw per-trial pass/fail
//! rows for bootstrap-CI analysis of the 50%-crossing estimate. See
//! `docs/notes/FST4_BENCHMARK.md` for the shared methodology this mirrors —
//! FT8 has no sub-modes, so there's one grid instead of one per T/R period.

#![cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;
use mfsk_core::msg::wsjt77::unpack77;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;
const DT_TOL_SEC: f32 = 0.6;

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_FT8_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/ft8_sweep")
        .to_path_buf()
}

// ── Channel conditions (must match gen_ft8_sweep_wavs.sh CHANNELS) ─────────
#[allow(dead_code)]
const CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];

fn decode_wav_ft8(audio: &[i16]) -> bool {
    use mfsk_core::ft8::Ft8;

    use mfsk_core::msg::decode_request::DecodeRequest;
    DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50)
        .decode()
        .results
        .iter()
        .any(|d| {
            unpack77(d.message77()).as_deref() == Some(GOLDEN_MSG)
                && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
                && d.dt_sec.abs() <= DT_TOL_SEC
        })
}

// ── Filename parsing ─────────────────────────────────────────────────────────

/// Parse `ft8_<channel>_<snr_tag>_<trial>.wav`.
/// snr_tag: `m05` = -5, `p05` = +5.
fn parse_snr_tag(tag: &str) -> Option<i32> {
    if let Some(rest) = tag.strip_prefix('m') {
        rest.parse::<i32>().ok().map(|v| -v)
    } else if let Some(rest) = tag.strip_prefix('p') {
        rest.parse::<i32>().ok()
    } else {
        None
    }
}

struct WavMeta {
    channel: String,
    snr_db: i32,
    trial: u32,
    path: PathBuf,
}

fn collect_wavs(dir: &Path) -> Vec<WavMeta> {
    let mut out = Vec::new();
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return out,
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_string();
        // ft8_awgn_m05_01
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() < 4 || parts[0] != "ft8" {
            continue;
        }
        let trial: u32 = match parts.last().and_then(|s| s.parse().ok()) {
            Some(v) => v,
            None => continue,
        };
        let snr_tag = parts[parts.len() - 2];
        let snr_db = match parse_snr_tag(snr_tag) {
            Some(v) => v,
            None => continue,
        };
        // channel = everything between "ft8" and snr_tag
        let channel = parts[1..parts.len() - 2].join("_");
        out.push(WavMeta {
            channel,
            snr_db,
            trial,
            path,
        });
    }
    // Sort: channel → snr desc → trial
    out.sort_by_key(|m| (m.channel.clone(), std::cmp::Reverse(m.snr_db), m.trial));
    out
}

// ── Main sweep test ──────────────────────────────────────────────────────────

#[test]
#[ignore = "manual pre-merge benchmark — run with --ignored --nocapture"]
fn ft8_snr_sweep() {
    let dir = sweep_dir();
    let all_wavs = collect_wavs(&dir);

    if all_wavs.is_empty() {
        eprintln!(
            "No WAVs found in {:?}\n\
             Run: scripts/build_ft8sim.sh && scripts/gen_ft8_sweep_wavs.sh",
            dir
        );
        return;
    }

    // Optional env-var filters — narrow the sweep to the region of interest.
    // MFSK_FT8_SWEEP_CHANNELS=awgn       (comma-separated channel names)
    // MFSK_FT8_SWEEP_SNR_MIN=-24         (inclusive lower bound, dB)
    // MFSK_FT8_SWEEP_SNR_MAX=-17         (inclusive upper bound, dB)
    // MFSK_FT8_SWEEP_CSV=/path/out.csv   (optional: dump raw per-trial
    //   pass/fail rows — channel,snr_db,trial,pass)
    let chan_filter: Option<Vec<String>> = std::env::var("MFSK_FT8_SWEEP_CHANNELS")
        .ok()
        .map(|s| s.split(',').map(|v| v.trim().to_string()).collect());
    let snr_min: Option<i32> = std::env::var("MFSK_FT8_SWEEP_SNR_MIN")
        .ok()
        .and_then(|s| s.trim().parse().ok());
    let snr_max: Option<i32> = std::env::var("MFSK_FT8_SWEEP_SNR_MAX")
        .ok()
        .and_then(|s| s.trim().parse().ok());

    let wavs: Vec<WavMeta> = all_wavs
        .into_iter()
        .filter(|w| {
            chan_filter
                .as_ref()
                .is_none_or(|f| f.iter().any(|c| c == &w.channel))
        })
        .filter(|w| snr_min.is_none_or(|m| w.snr_db >= m))
        .filter(|w| snr_max.is_none_or(|m| w.snr_db <= m))
        .collect();

    eprintln!("\n{:-<64}", "");
    eprintln!(
        "  {:<14} {:>7}   {:>6}  Bar",
        "Channel", "SNR(dB)", "Recall"
    );
    eprintln!("{:-<64}", "");

    let mut csv = std::env::var("MFSK_FT8_SWEEP_CSV").ok().map(|path| {
        let mut f = std::fs::File::create(&path)
            .unwrap_or_else(|e| panic!("MFSK_FT8_SWEEP_CSV={path}: {e}"));
        writeln!(f, "channel,snr_db,trial,pass").unwrap();
        f
    });

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;
    use std::io::Write;

    let mut groups: BTreeMap<(String, i32), Vec<&WavMeta>> = BTreeMap::new();
    for wav in &wavs {
        groups
            .entry((wav.channel.clone(), wav.snr_db))
            .or_default()
            .push(wav);
    }

    let mut last_chan: Option<String> = None;
    for ((chan, snr), wav_group) in &groups {
        #[cfg(feature = "parallel")]
        let results: Vec<(u32, bool)> = wav_group
            .par_iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_wav_ft8(&audio)))
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<(u32, bool)> = wav_group
            .iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_wav_ft8(&audio)))
            })
            .collect();

        let trials = results.len() as u32;
        if trials == 0 {
            continue;
        }
        let hits = results.iter().filter(|&(_, h)| *h).count() as u32;

        if let Some(f) = csv.as_mut() {
            for &(trial, pass) in &results {
                writeln!(f, "{chan},{snr},{trial},{}", pass as u8).unwrap();
            }
        }

        if Some(chan) != last_chan.as_ref() {
            eprintln!("{:-<64}", "");
            last_chan = Some(chan.clone());
        }
        let pct = hits as f32 / trials as f32 * 100.0;
        let bar_len = (hits as usize * 20).div_ceil(trials as usize);
        let bar = format!("{}{}", "#".repeat(bar_len), ".".repeat(20 - bar_len));
        eprintln!(
            "  {:<14}  {:>4} dB   {:>2}/{:<2}  [{}]  {:4.0}%",
            chan, snr, hits, trials, bar, pct
        );
    }
    eprintln!("{:-<64}", "");
    eprintln!(
        "\nChannels (ITU-R Watterson): awgn=no fading | \
         ccir_good=fdop 0.1Hz/del 0.5ms | \
         ccir_moderate=0.5/1.0 | ccir_poor=1.0/2.0"
    );
}

/// Per-trial stage attribution for CCIR moderate/poor losing trials
/// (`FT8_BENCHMARK.md` CCIR fading gap investigation, issue #72
/// follow-up, 2026-07-18), mirroring `ft4_diag_weak_trials`/
/// `fst4_diag_weak_trials`. `process_candidate`/`process_one_candidate_inner`
/// (`src/ft8/decode.rs`, `src/ft8/decode_block/process_candidates.rs`)
/// are not `pub` outside `crate::ft8`, so this replicates
/// `process_candidate`'s prefix (coarse_sync -> fine_refine_3stage ->
/// nsync gate) directly against the public building blocks it itself
/// calls, and uses the real `decode_frame` (production entry point, no
/// sniper mode / no EqMode substitution — those are a hardware-roofing-
/// filter-specific accommodation, not a valid general stand-in, per
/// correction) as an opaque black box for the final BP/OSD stage — same
/// limitation `fst4_diag_weak_trials` accepted for its own black-box
/// decode call.
#[test]
#[ignore = "manual diagnostic — CCIR fading stage attribution (issue #72 follow-up)"]
fn ft8_diag_weak_trials() {
    use mfsk_core::engine::dsp::downsample::downsample_cached;
    use mfsk_core::engine::sync::fine_sync_power_per_block;
    use mfsk_core::ft8::Ft8;
    use mfsk_core::ft8::decode_block::{coarse_sync, compute_spectrogram};
    use mfsk_core::ft8::downsample::{FT8_CFG, build_fft_cache};
    use mfsk_core::ft8::llr::sync_quality;
    use mfsk_core::ft8::refine_fine::fine_refine_3stage;

    let dir = sweep_dir();
    for &(chan, snr_tag) in &[
        ("ccir_moderate", "m18"),
        ("ccir_moderate", "m17"),
        ("ccir_poor", "m18"),
        ("ccir_poor", "m17"),
    ] {
        for trial in 1..=20u32 {
            let path = dir.join(format!("ft8_{chan}_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            if decode_wav_ft8(&audio) {
                continue; // only trace losing trials
            }

            let spec = compute_spectrogram(&audio, 3000.0);
            let candidates = coarse_sync(&spec, 100.0, 3000.0, 0.8, 50);
            let near: Vec<_> = candidates
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            eprintln!(
                "{chan} {snr_tag} trial {trial}: {} candidates total, {} near golden freq",
                candidates.len(),
                near.len()
            );
            if near.is_empty() {
                continue;
            }
            let fft_cache = build_fft_cache(&audio);
            for c in &near {
                let cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT8_CFG);
                let refine = fine_refine_3stage(&cd0, c.dt_sec);
                let refined_freq = c.freq_hz + refine.delf_hz;
                let i_start = ((refine.dt_sec + 0.5) * 200.0).round() as i32;
                let shifted =
                    mfsk_core::engine::sync2d::freq_shift_cd0(&cd0, refine.delf_hz, 200.0);
                let scores = fine_sync_power_per_block::<Ft8>(&shifted, i_start);
                let mean = scores.iter().sum::<f32>() / scores.len().max(1) as f32;
                let sync_cv = if mean > f32::EPSILON {
                    (scores.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / scores.len() as f32)
                        .sqrt()
                        / mean
                } else {
                    0.0
                };
                let mut cs_raw: [[mfsk_core::engine::scalar::Cmplx<f32>; 8]; 79] =
                    [[Default::default(); 8]; 79];
                mfsk_core::ft8::decode_block::fill_symbol_spectra(
                    &mut cs_raw,
                    &audio,
                    refined_freq,
                    refine.dt_sec,
                    mfsk_core::ft8::decode_block::SymMask::SyncOnly,
                    Some(&fft_cache),
                );
                mfsk_core::ft8::decode_block::fill_symbol_spectra(
                    &mut cs_raw,
                    &audio,
                    refined_freq,
                    refine.dt_sec,
                    mfsk_core::ft8::decode_block::SymMask::DataOnly,
                    Some(&fft_cache),
                );
                let nsync = sync_quality(&cs_raw);
                eprintln!(
                    "  cand freq={:.2} dt={:.3} coarse_score={:.4} refined_freq={:.2} \
                     refined_dt={:.3} refine_score={:.3} sync_cv={:.3} nsync={}/21 (gate>6={})",
                    c.freq_hz,
                    c.dt_sec,
                    c.score,
                    refined_freq,
                    refine.dt_sec,
                    refine.score,
                    sync_cv,
                    nsync,
                    nsync > 6
                );
            }
            // Final BP/OSD stage: opaque black box (see doc comment). Already
            // known false (filtered above), restated for readability.
            eprintln!("  -> full pipeline (decode_frame) decode: false");
        }
    }
}

/// `DecodeStrictness` calibration probe (issue #253, prompted by a
/// reproducible false decode found via WebFT8's `Deep` + `.sic_early()`
/// phase-2 pipeline on `qso3_busy.wav`: `7Y8CIH HN1GD OP30` @509 Hz,
/// `hard_errors=31`, admitted by `Deep`'s `ft8_nharderrors_max=40` —
/// a ceiling documented as "not yet swept against a fading corpus"
/// since it was introduced). Mirrors `ft4_strictness_probe`'s
/// methodology: drives `DecodeRequest<Ft8>` with each of
/// `Strict`/`Normal`/`Deep`, across **both** the plain single-pass
/// strategy and `.sic_early()` (the false-decode above only
/// reproduced under SIC — plain single-pass may not show the same
/// false-accept growth, since only `.sic_early()`'s later passes
/// search a subtraction *residual* rather than the raw trial).
///
/// Every trial in this corpus encodes exactly one real signal
/// (`GOLDEN_MSG` at `GOLDEN_FREQ_HZ`), so any additional distinct
/// decoded message is a false accept by construction — no ambiguity
/// about whether a "phantom" is secretly a second real signal, unlike
/// `qso3_busy.wav`'s own multi-station busy band.
///
/// ```sh
/// cargo test --test ft8_sweep --release --features ft8,fft-rustfft,parallel,uvpacket \
///   ft8_strictness_probe -- --ignored --nocapture
/// ```
#[test]
#[ignore = "manual calibration probe — run with --ignored --nocapture"]
fn ft8_strictness_probe() {
    use mfsk_core::engine::pipeline::DecodeStrictness;
    use mfsk_core::ft8::Ft8;
    use mfsk_core::msg::decode_request::DecodeRequest;

    let dir = sweep_dir();
    let all_wavs = collect_wavs(&dir);
    if all_wavs.is_empty() {
        eprintln!(
            "No WAVs found in {:?}\n\
             Run: scripts/build_ft8sim.sh && scripts/gen_ft8_sweep_wavs.sh",
            dir
        );
        return;
    }

    // Cells: near/below the AWGN/CCIR ~-20..-22 dB 50% crossings
    // (docs/notes/BENCHMARKS.md) plus deep-noise cells where the real
    // signal essentially never decodes — any positive result there is
    // almost certainly a false accept, isolating Deep's risk cleanly
    // from its real recall gain.
    const CELLS: &[i32] = &[-19, -21, -24, -26];
    let wavs: Vec<&WavMeta> = all_wavs
        .iter()
        .filter(|w| CELLS.contains(&w.snr_db))
        .collect();

    #[derive(Default, Clone, Copy)]
    struct Cell {
        trials: u32,
        golden: u32,
        false_accept: u32,
    }

    use std::collections::BTreeMap;
    // (channel, snr, strategy, strictness) -> Cell
    let mut table: BTreeMap<(String, i32, &'static str, &'static str), Cell> = BTreeMap::new();

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    let strategies: &[(&str, bool)] = &[("single_pass", false), ("sic_early", true)];
    let levels: &[(&str, DecodeStrictness)] = &[
        ("Strict", DecodeStrictness::Strict),
        ("Normal", DecodeStrictness::Normal),
        ("Deep", DecodeStrictness::Deep),
    ];

    type TrialRow = (
        (String, i32),
        Vec<((&'static str, &'static str), (bool, bool))>,
    );

    #[cfg(feature = "parallel")]
    let per_trial: Vec<TrialRow> = wavs
        .par_iter()
        .filter_map(|wav| {
            let audio = load_wav_i16_opt(&wav.path)?;
            let mut row = Vec::with_capacity(strategies.len() * levels.len());
            for &(strat_name, use_sic) in strategies {
                for &(strict_name, strictness) in levels {
                    let req = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 50)
                        .strictness(strictness);
                    let results = if use_sic {
                        req.sic_early().decode().results
                    } else {
                        req.decode().results
                    };
                    let mut golden = false;
                    let mut false_accept = false;
                    for r in &results {
                        let Some(text) = unpack77(r.message77()) else {
                            continue;
                        };
                        let is_golden = text == GOLDEN_MSG
                            && (r.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
                            && r.dt_sec.abs() <= DT_TOL_SEC;
                        if is_golden {
                            golden = true;
                        } else {
                            false_accept = true;
                        }
                    }
                    row.push(((strat_name, strict_name), (golden, false_accept)));
                }
            }
            Some(((wav.channel.clone(), wav.snr_db), row))
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let per_trial: Vec<TrialRow> = wavs
        .iter()
        .filter_map(|wav| {
            let audio = load_wav_i16_opt(&wav.path)?;
            let mut row = Vec::with_capacity(strategies.len() * levels.len());
            for &(strat_name, use_sic) in strategies {
                for &(strict_name, strictness) in levels {
                    let req = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 50)
                        .strictness(strictness);
                    let results = if use_sic {
                        req.sic_early().decode().results
                    } else {
                        req.decode().results
                    };
                    let mut golden = false;
                    let mut false_accept = false;
                    for r in &results {
                        let Some(text) = unpack77(r.message77()) else {
                            continue;
                        };
                        let is_golden = text == GOLDEN_MSG
                            && (r.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
                            && r.dt_sec.abs() <= DT_TOL_SEC;
                        if is_golden {
                            golden = true;
                        } else {
                            false_accept = true;
                        }
                    }
                    row.push(((strat_name, strict_name), (golden, false_accept)));
                }
            }
            Some(((wav.channel.clone(), wav.snr_db), row))
        })
        .collect();

    for ((chan, snr), row) in per_trial {
        for ((strat_name, strict_name), (golden, false_accept)) in row {
            let cell = table
                .entry((chan.clone(), snr, strat_name, strict_name))
                .or_default();
            cell.trials += 1;
            cell.golden += golden as u32;
            cell.false_accept += false_accept as u32;
        }
    }

    eprintln!("\n{:-<86}", "");
    eprintln!(
        "  {:<14} {:>4} {:<11} {:<7} {:>10} {:>14}",
        "Channel", "SNR", "Strategy", "Level", "golden", "false_accept"
    );
    eprintln!("{:-<86}", "");
    let mut last_key: Option<(String, i32)> = None;
    for ((chan, snr, strat_name, strict_name), cell) in &table {
        if last_key.as_ref() != Some(&(chan.clone(), *snr)) {
            eprintln!("{:-<86}", "");
            last_key = Some((chan.clone(), *snr));
        }
        eprintln!(
            "  {:<14} {:>3}dB {:<11} {:<7} {:>7}/{:<2} {:>10}/{:<2}",
            chan,
            snr,
            strat_name,
            strict_name,
            cell.golden,
            cell.trials,
            cell.false_accept,
            cell.trials,
        );
    }
    eprintln!("{:-<86}", "");
    eprintln!(
        "\nfalse_accept = trials with a CRC-passing decode that is NOT the golden \
         message — every trial here encodes exactly one real signal, so this is \
         an unambiguous false-accept count, not a suspected-phantom guess."
    );
}
