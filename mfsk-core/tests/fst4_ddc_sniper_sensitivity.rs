//! FST4-60 sniper coarse-sync: real-path vs. DDC-path score, by SNR.
//!
//! Answers the question `docs/notes/FST4_DDC_DESIGN.md`'s own stage 3
//! end-to-end test left open (see its doc comment): the DDC path's
//! `coarse_sync` score measured ~1/3 of the real path's on a single
//! *clean* synthetic signal — plausible extra processing-margin cost
//! from routing through a mixer + 2-stage filter cascade instead of
//! one direct FFT, but that test cannot say how many dB of real
//! sensitivity that margin costs, because a noiseless fixture can't
//! (`feedback_noiseless_synth_fixture_trap`). This one uses the real
//! `fst4sim`-generated AWGN corpus instead.
//!
//! **Not a detection-recall sweep** — an earlier version of this file
//! was, and the result was uninformative: `coarse_sync`'s own
//! admission gate (`sync_min = 0.8`, plus the FST4 full-slot OR-gate)
//! is loose *by design* — this decoder's real signal/noise selectivity
//! happens downstream, in LDPC/CRC, not at the coarse-sync layer.
//! "Was a candidate found near the golden position" turned out to be
//! ~100% for both paths from -5 dB all the way down to synthetic
//! -60 dB, which is not a real result, just evidence that binary
//! detection isn't a useful metric for this decoder's architecture.
//! What *does* move with SNR, meaningfully, is the golden candidate's
//! own `.score` — this measures that instead, and reports it as a
//! ratio (DDC/real) per SNR bucket, which is the number that
//! generalises the single-clean-signal ~1/3 figure the stage 3 test
//! measured.
//!
//! `#[ignore]` — tier C, needs the generated corpus:
//!
//! ```sh
//! scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh
//! cargo test --test fst4_ddc_sniper_sensitivity --release \
//!   --features fst4,fft-rustfft,parallel,internal-testing \
//!   -- --ignored --nocapture
//! ```
//!
//! Both paths search the same ±250 Hz sniper window around the known
//! golden frequency (a real `SniperRequest` would do the same once it
//! had a target hint), so this is specifically the sniper cascade's
//! cost — the wideband cascade (heavier, ~10K taps) is not measured
//! here.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use mfsk_core::engine::dsp::ddc::ddc_block;
use mfsk_core::engine::sync::{AudioSource, RxGrid, coarse_sync};
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{grid_for, sniper_cascade};

const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;
const DT_TOL_SEC: f32 = 0.6;
const SEARCH_HALF_WIDTH_HZ: f32 = 250.0; // matches sniper_cascade's own ±250 Hz shape

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_FST4_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf()
}

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
    snr_db: i32,
    trial: u32,
    path: PathBuf,
}

/// Collects only `fst4_60_awgn_*` — the channel/mode this test scopes
/// to (see the module doc comment).
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
        // fst4_60_awgn_m26_01
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() != 5 || parts[0] != "fst4" || parts[1] != "60" || parts[2] != "awgn" {
            continue;
        }
        let trial: u32 = match parts[4].parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let snr_db = match parse_snr_tag(parts[3]) {
            Some(v) => v,
            None => continue,
        };
        out.push(WavMeta {
            snr_db,
            trial,
            path,
        });
    }
    out.sort_by_key(|m| (-m.snr_db, m.trial));
    out
}

/// The golden candidate's own `.score` if `coarse_sync` returned one
/// near the known position, `None` otherwise (rare at these SNRs —
/// see the module doc comment on why binary detection isn't the
/// metric here; `None` trials are excluded from the mean rather than
/// counted as 0, so a handful of misses don't silently pull the mean
/// toward a number that looks like "signal" when it's actually "no
/// candidate returned at all").
fn golden_score(cands: &[mfsk_core::engine::sync::SyncCandidate]) -> Option<f32> {
    cands
        .iter()
        .find(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ && c.dt_sec.abs() <= DT_TOL_SEC)
        .map(|c| c.score)
}

fn real_path_score(audio: &[i16]) -> Option<f32> {
    let cands = coarse_sync::<Fst4s60>(
        AudioSource::Real(audio),
        GOLDEN_FREQ_HZ - SEARCH_HALF_WIDTH_HZ,
        GOLDEN_FREQ_HZ + SEARCH_HALF_WIDTH_HZ,
        // f32::NEG_INFINITY, not the production 0.8 — this test wants
        // the golden candidate's raw score whether or not it would
        // have cleared the admission gate (see module doc comment),
        // not to replicate the gate itself.
        f32::NEG_INFINITY,
        Some(GOLDEN_FREQ_HZ),
        50,
        RxGrid::real(12_000.0),
    );
    golden_score(&cands)
}

fn ddc_path_score(audio: &[i16]) -> Option<f32> {
    let cfg = sniper_cascade(GOLDEN_FREQ_HZ);
    let (ai, aq) = ddc_block(audio, &cfg);
    // `sniper_cascade` hardcodes the 600 Hz / K=256 grid internally
    // (design doc §4.3's sniper row) — reproduced here since it
    // doesn't expose K/Fs_c itself.
    let grid = grid_for(600.0);
    let cands = coarse_sync::<Fst4s60>(
        AudioSource::Complex(&ai, &aq),
        GOLDEN_FREQ_HZ - SEARCH_HALF_WIDTH_HZ,
        GOLDEN_FREQ_HZ + SEARCH_HALF_WIDTH_HZ,
        f32::NEG_INFINITY,
        Some(GOLDEN_FREQ_HZ),
        50,
        RxGrid::complex(grid.fs_c, GOLDEN_FREQ_HZ),
    );
    golden_score(&cands)
}

#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_sniper_ddc_vs_real_score_by_snr() {
    let dir = sweep_dir();
    let wavs = collect_wavs(&dir);
    if wavs.is_empty() {
        eprintln!(
            "No fst4_60_awgn_*.wav found in {dir:?}\n\
             Run: scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh"
        );
        return;
    }

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;
    use std::io::Write;

    let mut groups: BTreeMap<i32, Vec<&WavMeta>> = BTreeMap::new();
    for w in &wavs {
        groups.entry(w.snr_db).or_default().push(w);
    }

    let mut csv = common::sweep_csv_writer(
        "MFSK_FST4_DDC_SNIPER_SWEEP_CSV",
        "snr_db,trial,real_score,ddc_score",
    );

    eprintln!("\n{:-<78}", "");
    eprintln!(
        "  {:>7}  {:>6}  {:>12}  {:>12}  {:>10}",
        "SNR(dB)", "n", "real (mean)", "ddc (mean)", "ddc/real"
    );
    eprintln!("{:-<78}", "");

    for (snr, group) in &groups {
        #[cfg(feature = "parallel")]
        let results: Vec<(u32, Option<f32>, Option<f32>)> = group
            .par_iter()
            .filter_map(|w| {
                load_wav_i16_opt(&w.path)
                    .map(|audio| (w.trial, real_path_score(&audio), ddc_path_score(&audio)))
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<(u32, Option<f32>, Option<f32>)> = group
            .iter()
            .filter_map(|w| {
                load_wav_i16_opt(&w.path)
                    .map(|audio| (w.trial, real_path_score(&audio), ddc_path_score(&audio)))
            })
            .collect();

        let n = results.len() as u32;
        if n == 0 {
            continue;
        }

        if let Some(f) = csv.as_mut() {
            for &(trial, r, d) in &results {
                writeln!(
                    f,
                    "{snr},{trial},{},{}",
                    r.map(|v| v.to_string()).unwrap_or_default(),
                    d.map(|v| v.to_string()).unwrap_or_default()
                )
                .unwrap();
            }
        }

        let real_vals: Vec<f32> = results.iter().filter_map(|&(_, r, _)| r).collect();
        let ddc_vals: Vec<f32> = results.iter().filter_map(|&(_, _, d)| d).collect();
        let mean = |v: &[f32]| -> Option<f32> {
            if v.is_empty() {
                None
            } else {
                Some(v.iter().sum::<f32>() / v.len() as f32)
            }
        };
        let real_mean = mean(&real_vals);
        let ddc_mean = mean(&ddc_vals);

        let fmt = |v: Option<f32>, hit_n: usize| match v {
            Some(m) => format!("{m:>8.2} ({hit_n}/{n})"),
            None => format!("{:>8}", "n/a"),
        };
        let ratio = match (real_mean, ddc_mean) {
            (Some(r), Some(d)) if r > 0.0 => format!("{:.2}", d / r),
            _ => "n/a".to_string(),
        };

        eprintln!(
            "  {snr:>7}  {n:>6}  {}  {}  {ratio:>10}",
            fmt(real_mean, real_vals.len()),
            fmt(ddc_mean, ddc_vals.len()),
        );
    }
    eprintln!("{:-<78}", "");
}
