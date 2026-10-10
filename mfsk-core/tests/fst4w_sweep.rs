//! FST4W SNR sweep + fading benchmark against `fst4sim`-generated signals (#649).
//!
//! `#[ignore]`: a tier-C measurement, not a check. Run it through
//! `scripts/run-sensitivity-sweeps.sh fst4w`, or by hand:
//!
//! ```sh
//! scripts/build_fst4sim.sh
//! scripts/gen_fst4w_sweep_wavs.sh          # 120 s and 300 s, 4 channels, 20 trials a cell
//! MFSK_FST4W_SWEEP_CSV=/tmp/fst4w.csv cargo test -p mfsk-core --release \
//!   --features full,internal-testing --test fst4w_sweep -- --ignored --nocapture
//! ```
//!
//! The task is `scripts/upstream_tasks.json`'s `fst4w/t1`: what the FST4W GUI
//! does with no known calls, `jt9 -W -p <T> -d 3 -f 1500 -F 100`. The crate's
//! side is `Decoder::<Fst4wN>` with `rx_freq_hz` 1500, `tol_hz` 100, `Depth::Deep`
//! (Keff 50 on, empty known-call list) — the list is fresh for every file, as
//! `jt9`'s is.
//!
//! No assertions. Output is a recall table; `MFSK_FST4W_SWEEP_CSV` also dumps
//! `mode,channel,snr_db,trial,pass,extra` per trial.
//! `MFSK_FST4W_SWEEP_DIR` overrides the corpus directory.

#![cfg(all(
    feature = "fst4w",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::{load_wav_i16_opt, parse_snr_tag};
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::fst4w::{Fst4w120, Fst4w300};

/// What `fst4sim "<msg>" ... T` sent (`gen_fst4w_sweep_wavs.sh`).
const GOLDEN_MSG: &str = "JL1NIE PM95 37";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;
const DT_TOL_SEC: f32 = 0.6;

fn sweep_dir() -> PathBuf {
    common::sweep_dir("MFSK_FST4W_SWEEP_DIR", "fst4w_sweep")
}

fn params() -> DecodeParams {
    DecodeParams::for_band((1400.0, 1600.0))
        .rx_freq(1500.0)
        .tol(100.0)
}

/// `(pass, extra)`: whether the injected message came out at the right
/// frequency and time, and how many *other* distinct messages came out.
fn decode_wav(nsec: u32, audio: &[i16]) -> (bool, u32) {
    let slot = SlotInput::i16(audio);
    let rows = match nsec {
        120 => Decoder::<Fst4w120>::new(params()).decode(&slot).rows,
        300 => Decoder::<Fst4w300>::new(params()).decode(&slot).rows,
        other => panic!("no FST4W-{other} in this sweep"),
    };
    let pass = rows.iter().any(|r| {
        r.decoded.text == GOLDEN_MSG
            && (r.decoded.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
            && r.decoded.dt_sec.abs() <= DT_TOL_SEC
    });
    let texts: Vec<String> = rows.iter().map(|r| r.decoded.text.clone()).collect();
    (pass, common::distinct_extras(&texts, GOLDEN_MSG))
}

struct WavMeta {
    nsec: u32,
    channel: String,
    snr_db: i32,
    trial: u32,
    path: PathBuf,
}

/// `fst4w_<nsec>_<channel>_<snr_tag>_<trial>.wav`
fn collect_wavs(dir: &Path) -> Vec<WavMeta> {
    let mut out = Vec::new();
    let Ok(entries) = std::fs::read_dir(dir) else {
        return out;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_string();
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() < 5 || parts[0] != "fst4w" {
            continue;
        }
        let Ok(nsec) = parts[1].parse::<u32>() else {
            continue;
        };
        let Some(trial) = parts.last().and_then(|s| s.parse::<u32>().ok()) else {
            continue;
        };
        let Some(snr_db) = parse_snr_tag(parts[parts.len() - 2]) else {
            continue;
        };
        let channel = parts[2..parts.len() - 2].join("_");
        out.push(WavMeta {
            nsec,
            channel,
            snr_db,
            trial,
            path,
        });
    }
    out.sort_by_key(|m| (m.nsec, m.channel.clone(), -m.snr_db, m.trial));
    out
}

#[test]
#[ignore = "tier C — run with --ignored --nocapture, or scripts/run-sensitivity-sweeps.sh fst4w"]
fn fst4w_snr_sweep() {
    use std::collections::BTreeMap;
    use std::io::Write;

    let dir = sweep_dir();
    let wavs = collect_wavs(&dir);
    if wavs.is_empty() {
        eprintln!(
            "No WAVs found in {dir:?}\nRun: scripts/build_fst4sim.sh && scripts/gen_fst4w_sweep_wavs.sh"
        );
        return;
    }
    let mode_filter: Option<Vec<u32>> = std::env::var("MFSK_FST4W_SWEEP_MODES")
        .ok()
        .map(|s| s.split(',').filter_map(|v| v.trim().parse().ok()).collect());

    let mut csv = common::sweep_csv_writer(
        "MFSK_FST4W_SWEEP_CSV",
        "mode,channel,snr_db,trial,pass,extra",
    );

    let mut groups: BTreeMap<(u32, String, i32), Vec<&WavMeta>> = BTreeMap::new();
    for w in &wavs {
        if mode_filter.as_ref().is_none_or(|f| f.contains(&w.nsec)) {
            groups
                .entry((w.nsec, w.channel.clone(), w.snr_db))
                .or_default()
                .push(w);
        }
    }

    eprintln!("\n{:-<72}", "");
    eprintln!(
        "  {:<11} {:<14} {:>7}   {:>6}  {:<22}       Extra",
        "Mode", "Channel", "SNR(dB)", "Recall", "Bar"
    );
    let mut last: Option<(u32, String)> = None;
    for ((nsec, chan, snr), group) in &groups {
        let results: Vec<(u32, (bool, u32))> = common::par_map(group, |w| {
            load_wav_i16_opt(&w.path).map(|a| (w.trial, decode_wav(w.nsec, &a)))
        })
        .into_iter()
        .flatten()
        .collect();
        let trials = results.len() as u32;
        if trials == 0 {
            continue;
        }
        let hits = results.iter().filter(|&&(_, (h, _))| h).count() as u32;
        let extras: u32 = results.iter().map(|&(_, (_, e))| e).sum();
        if let Some(f) = csv.as_mut() {
            for &(trial, (pass, extra)) in &results {
                writeln!(f, "{nsec},{chan},{snr},{trial},{},{extra}", pass as u8).unwrap();
            }
        }
        if last.as_ref() != Some(&(*nsec, chan.clone())) {
            eprintln!("{:-<72}", "");
            last = Some((*nsec, chan.clone()));
        }
        let bar_len = (hits as usize * 20).div_ceil(trials as usize);
        eprintln!(
            "  FST4W-{:<4}  {:<14}  {:>4} dB   {:>2}/{:<2}  [{}{}]  {:4.0}%   {:>3}",
            nsec,
            chan,
            snr,
            hits,
            trials,
            "#".repeat(bar_len),
            ".".repeat(20 - bar_len),
            hits as f32 / trials as f32 * 100.0,
            extras
        );
    }
    eprintln!("{:-<72}", "");
}
