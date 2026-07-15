//! FST4 SNR sweep + fading benchmark against `fst4sim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually before merging a new sub-mode:
//!
//! ```sh
//! # 1. Generate WAVs (once, or when adding a new mode/channel):
//! scripts/build_fst4sim.sh
//! scripts/gen_fst4_sweep_wavs.sh
//!
//! # 2. Run the sweep (all currently-wired modes):
//! MFSK_FST4_SWEEP_DIR=embedded-poc/assets/fst4_sweep \
//!   cargo test --test fst4_sweep --release --features fst4,fft-rustfft,parallel \
//!   -- --ignored --nocapture
//! ```
//!
//! Output is a recall table — no assertions, statistics only.
//! Add a new sub-mode by:
//!   1. Implementing `Fst4sNNN` + `decode_frameNNN` in `mfsk_core::fst4`.
//!   2. Adding a `SweepMode` entry to `MODES` below.
//!   3. Uncommenting `decode_frameNNN` in the dispatch block.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

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
    if let Ok(d) = std::env::var("MFSK_FST4_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf()
}

// ── Channel conditions (must match gen_fst4_sweep_wavs.sh CHANNELS) ─────────
#[allow(dead_code)]
const CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];

// ── Per-mode decode dispatch ─────────────────────────────────────────────────

fn decode_wav_fst4<P>(audio: &[i16], cfg: &mfsk_core::core::dsp::downsample::DownsampleCfg) -> bool
where
    P: mfsk_core::core::Protocol,
{
    use mfsk_core::fst4::decode::decode_frame_for;
    decode_frame_for::<P>(audio, cfg, 100.0, 3000.0, 0.8, 50)
        .iter()
        .any(|d| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(d.message77());
            unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
                && d.dt_sec.abs() <= DT_TOL_SEC
        })
}

fn decode_wav_fst4_15(audio: &[i16]) -> bool {
    use mfsk_core::fst4::{Fst4s15, decode::FST4_15_DOWNSAMPLE};
    decode_wav_fst4::<Fst4s15>(audio, &FST4_15_DOWNSAMPLE)
}
fn decode_wav_fst4_30(audio: &[i16]) -> bool {
    use mfsk_core::fst4::{Fst4s30, decode::FST4_30_DOWNSAMPLE};
    decode_wav_fst4::<Fst4s30>(audio, &FST4_30_DOWNSAMPLE)
}
fn decode_wav_fst4_60(audio: &[i16]) -> bool {
    use mfsk_core::fst4::{Fst4s60, decode::FST4_60A_DOWNSAMPLE};
    decode_wav_fst4::<Fst4s60>(audio, &FST4_60A_DOWNSAMPLE)
}
fn decode_wav_fst4_120(audio: &[i16]) -> bool {
    use mfsk_core::fst4::{Fst4s120, decode::FST4_120_DOWNSAMPLE};
    decode_wav_fst4::<Fst4s120>(audio, &FST4_120_DOWNSAMPLE)
}
fn decode_wav_fst4_300(audio: &[i16]) -> bool {
    use mfsk_core::fst4::{Fst4s300, decode::FST4_300_DOWNSAMPLE};
    decode_wav_fst4::<Fst4s300>(audio, &FST4_300_DOWNSAMPLE)
}

// ── Mode table ───────────────────────────────────────────────────────────────

struct SweepMode {
    nsec: u32,
    decode: fn(&[i16]) -> bool,
    enabled: bool,
}

const MODES: &[SweepMode] = &[
    SweepMode { nsec: 15,  decode: decode_wav_fst4_15,  enabled: true },
    SweepMode { nsec: 30,  decode: decode_wav_fst4_30,  enabled: true },
    SweepMode { nsec: 60,  decode: decode_wav_fst4_60,  enabled: true },
    SweepMode { nsec: 120, decode: decode_wav_fst4_120, enabled: true },
    SweepMode { nsec: 300, decode: decode_wav_fst4_300, enabled: true },
];

// ── Filename parsing ─────────────────────────────────────────────────────────

/// Parse `fst4_<nsec>_<channel>_<snr_tag>_<trial>.wav`.
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
    nsec: u32,
    channel: String,
    snr_db: i32,
    #[allow(dead_code)]
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
        // fst4_60_awgn_m05_01
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() < 5 || parts[0] != "fst4" {
            continue;
        }
        let nsec: u32 = match parts[1].parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        // channel may contain underscore: hf_quiet → parts[2]_parts[3]
        // We parse from the right: last field = trial, second-to-last = snr_tag
        let trial: u32 = match parts.last().and_then(|s| s.parse().ok()) {
            Some(v) => v,
            None => continue,
        };
        let snr_tag = parts[parts.len() - 2];
        let snr_db = match parse_snr_tag(snr_tag) {
            Some(v) => v,
            None => continue,
        };
        // channel = everything between nsec and snr_tag
        let channel = parts[2..parts.len() - 2].join("_");
        out.push(WavMeta { nsec, channel, snr_db, trial, path });
    }
    // Sort: mode → channel → snr desc → trial
    out.sort_by_key(|m| (m.nsec, m.channel.clone(), -m.snr_db, m.trial));
    out
}

// ── Main sweep test ──────────────────────────────────────────────────────────

#[test]
#[ignore = "manual pre-merge benchmark — run with --ignored --nocapture"]
fn fst4_snr_sweep() {
    let dir = sweep_dir();
    let wavs = collect_wavs(&dir);

    if wavs.is_empty() {
        eprintln!(
            "No WAVs found in {:?}\n\
             Run: scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh",
            dir
        );
        return;
    }

    eprintln!("\n{:-<72}", "");
    eprintln!(
        "  {:<10} {:<14} {:>7}   {:>6}  {}",
        "Mode", "Channel", "SNR(dB)", "Recall", "Bar"
    );
    eprintln!("{:-<72}", "");

    // Group WAVs by (nsec, channel, snr_db) so we can parallelise within each
    // group and print each row immediately when the group finishes.
    use std::collections::BTreeMap;
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    let mut groups: BTreeMap<(u32, String, i32), Vec<&WavMeta>> = BTreeMap::new();
    for wav in &wavs {
        if MODES.iter().any(|m| m.nsec == wav.nsec && m.enabled) {
            groups
                .entry((wav.nsec, wav.channel.clone(), wav.snr_db))
                .or_default()
                .push(wav);
        }
    }

    let mut last_mode_chan: Option<(u32, String)> = None;
    for ((nsec, chan, snr), wav_group) in &groups {
        let decode_fn = MODES
            .iter()
            .find(|m| m.nsec == *nsec && m.enabled)
            .map(|m| m.decode)
            .unwrap(); // safe: we filtered above

        #[cfg(feature = "parallel")]
        let results: Vec<bool> = wav_group
            .par_iter()
            .filter_map(|wav| load_wav_i16_opt(&wav.path).map(|audio| decode_fn(&audio)))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<bool> = wav_group
            .iter()
            .filter_map(|wav| load_wav_i16_opt(&wav.path).map(|audio| decode_fn(&audio)))
            .collect();

        let trials = results.len() as u32;
        if trials == 0 {
            continue;
        }
        let hits = results.iter().filter(|&&h| h).count() as u32;

        let mode_chan = (*nsec, chan.clone());
        if Some(&mode_chan) != last_mode_chan.as_ref() {
            eprintln!("{:-<72}", "");
            last_mode_chan = Some(mode_chan);
        }
        let pct = hits as f32 / trials as f32 * 100.0;
        let bar_len = (hits as usize * 20).div_ceil(trials as usize);
        let bar = format!("{}{}", "#".repeat(bar_len), ".".repeat(20 - bar_len));
        eprintln!(
            "  FST4-{:<4}  {:<14}  {:>4} dB   {:>2}/{:<2}  [{}]  {:4.0}%",
            nsec, chan, snr, hits, trials, bar, pct
        );
    }
    eprintln!("{:-<72}", "");
    eprintln!(
        "\nChannels (ITU-R Watterson): awgn=no fading | \
         ccir_good=fdop 0.1Hz/del 0.5ms | \
         ccir_moderate=0.5/1.0 | ccir_poor=1.0/2.0"
    );
    eprintln!("(Disabled modes show no rows — enable by wiring decode fn in MODES[])\n");
}
