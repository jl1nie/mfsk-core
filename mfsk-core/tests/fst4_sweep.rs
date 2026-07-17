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
//! cargo test --test fst4_sweep --release --features fst4,fft-rustfft,parallel,uvpacket \
//!   -- --ignored --nocapture
//! ```
//!
//! (`uvpacket` is only required because `tests/common/channel.rs`, pulled in
//! via `mod common`, unconditionally imports `mfsk_core::uvpacket` — unrelated
//! to FST4 itself. `MFSK_FST4_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/fst4_sweep`, relative to `CARGO_MANIFEST_DIR` —
//! i.e. absolute, or relative to the repo root, not the crate root cargo
//! actually runs tests from.)
//!
//! Output is a recall table — no assertions, statistics only. Set
//! `MFSK_FST4_SWEEP_CSV=/path/out.csv` to also dump raw per-trial pass/fail
//! rows for bootstrap-CI analysis of the 50%-crossing estimate (see the
//! env-var doc block in `fst4_snr_sweep` below).
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
    SweepMode {
        nsec: 15,
        decode: decode_wav_fst4_15,
        enabled: true,
    },
    SweepMode {
        nsec: 30,
        decode: decode_wav_fst4_30,
        enabled: true,
    },
    SweepMode {
        nsec: 60,
        decode: decode_wav_fst4_60,
        enabled: true,
    },
    SweepMode {
        nsec: 120,
        decode: decode_wav_fst4_120,
        enabled: true,
    },
    SweepMode {
        nsec: 300,
        decode: decode_wav_fst4_300,
        enabled: true,
    },
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
        out.push(WavMeta {
            nsec,
            channel,
            snr_db,
            trial,
            path,
        });
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
    let all_wavs = collect_wavs(&dir);

    if all_wavs.is_empty() {
        eprintln!(
            "No WAVs found in {:?}\n\
             Run: scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh",
            dir
        );
        return;
    }

    // Optional env-var filters — narrow the sweep to the region of interest.
    // MFSK_FST4_SWEEP_MODES=30,300        (comma-separated T/R periods)
    // MFSK_FST4_SWEEP_CHANNELS=awgn       (comma-separated channel names)
    // MFSK_FST4_SWEEP_SNR_MIN=-25         (inclusive lower bound, dB)
    // MFSK_FST4_SWEEP_SNR_MAX=-20         (inclusive upper bound, dB)
    // MFSK_FST4_SWEEP_CSV=/path/out.csv   (optional: dump raw per-trial
    //   pass/fail rows — mode,channel,snr_db,trial,pass — alongside the
    //   printed aggregate table. Only the aggregate hits/trials was
    //   available before; a 50%-crossing interpolation's confidence
    //   interval needs the per-trial outcomes to bootstrap, e.g. to tell
    //   apart a genuine sub-mode-specific recall deficit from 20-trial
    //   sampling noise — issue #146.)
    let mode_filter: Option<Vec<u32>> = std::env::var("MFSK_FST4_SWEEP_MODES")
        .ok()
        .map(|s| s.split(',').filter_map(|v| v.trim().parse().ok()).collect());
    let chan_filter: Option<Vec<String>> = std::env::var("MFSK_FST4_SWEEP_CHANNELS")
        .ok()
        .map(|s| s.split(',').map(|v| v.trim().to_string()).collect());
    let snr_min: Option<i32> = std::env::var("MFSK_FST4_SWEEP_SNR_MIN")
        .ok()
        .and_then(|s| s.trim().parse().ok());
    let snr_max: Option<i32> = std::env::var("MFSK_FST4_SWEEP_SNR_MAX")
        .ok()
        .and_then(|s| s.trim().parse().ok());

    let wavs: Vec<WavMeta> = all_wavs
        .into_iter()
        .filter(|w| mode_filter.as_ref().is_none_or(|f| f.contains(&w.nsec)))
        .filter(|w| {
            chan_filter
                .as_ref()
                .is_none_or(|f| f.iter().any(|c| c == &w.channel))
        })
        .filter(|w| snr_min.is_none_or(|m| w.snr_db >= m))
        .filter(|w| snr_max.is_none_or(|m| w.snr_db <= m))
        .collect();

    eprintln!("\n{:-<72}", "");
    eprintln!(
        "  {:<10} {:<14} {:>7}   {:>6}  Bar",
        "Mode", "Channel", "SNR(dB)", "Recall"
    );
    eprintln!("{:-<72}", "");

    let mut csv = std::env::var("MFSK_FST4_SWEEP_CSV").ok().map(|path| {
        let mut f = std::fs::File::create(&path)
            .unwrap_or_else(|e| panic!("MFSK_FST4_SWEEP_CSV={path}: {e}"));
        writeln!(f, "mode,channel,snr_db,trial,pass").unwrap();
        f
    });

    // Group WAVs by (nsec, channel, snr_db) so we can parallelise within each
    // group and print each row immediately when the group finishes.
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;
    use std::io::Write;

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
        let results: Vec<(u32, bool)> = wav_group
            .par_iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_fn(&audio)))
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<(u32, bool)> = wav_group
            .iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_fn(&audio)))
            })
            .collect();

        let trials = results.len() as u32;
        if trials == 0 {
            continue;
        }
        let hits = results.iter().filter(|&(_, h)| *h).count() as u32;

        if let Some(f) = csv.as_mut() {
            for &(trial, pass) in &results {
                writeln!(f, "{nsec},{chan},{snr},{trial},{}", pass as u8).unwrap();
            }
        }

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

/// Diagnostic probe (issue #146) — pinpoint where a known-failing AWGN
/// trial actually breaks: coarse_sync candidate presence/score near the
/// golden (freq, dt), vs. downstream decode (LLR/BP/OSD). Used to
/// disprove the "coarse-sync candidate crowding" hypothesis (the real
/// candidate was found in every trial, well above `sync_min`) and point
/// at the post-candidate pipeline instead — kept for future regressions
/// in this area.
///
/// Deliberately only exercises FST4-30 and FST4-300 (opposite ends of
/// the sub-mode range) rather than all five — the measured gap is flat
/// across periods (task #146), so two modes bracketing the range are
/// enough to confirm a mechanism generalizes without paying for a full
/// 5-mode diagnostic pass. Set `MFSK_DEBUG_TRACE=1` to also get
/// per-candidate nsync/OSD-gate/hard-error tracing from
/// `core::pipeline::process_candidate_basic`.
#[test]
#[ignore = "manual diagnostic, not a recall gate"]
fn fst4_diag_weak_trials() {
    use mfsk_core::core::equalize::EqMode;
    use mfsk_core::core::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::core::sync::coarse_sync;
    use mfsk_core::fst4::decode::{FST4_30_DOWNSAMPLE, FST4_300_DOWNSAMPLE};
    use mfsk_core::fst4::{Fst4s30, Fst4s300};

    fn probe<P: mfsk_core::core::Protocol>(
        dir: &std::path::Path,
        file_prefix: &str,
        cfg: &mfsk_core::core::dsp::downsample::DownsampleCfg,
    ) {
        for trial in 1..=5 {
            let path = dir.join(format!("{file_prefix}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                eprintln!("skip {path:?}");
                continue;
            };
            let cands = coarse_sync::<P>(&audio, 100.0, 3000.0, 0.8, None, 50);
            let near: Vec<_> = cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            eprintln!(
                "{file_prefix} trial {trial}: {} candidates total, {} near golden freq",
                cands.len(),
                near.len()
            );
            for c in &near {
                eprintln!(
                    "  cand freq={:.2} dt={:.3} score={:.4}",
                    c.freq_hz, c.dt_sec, c.score
                );
            }
            let fft_cache = mfsk_core::core::dsp::downsample::build_fft_cache(&audio, cfg);
            for c in &near {
                let r = process_candidate_basic::<P>(
                    c,
                    &fft_cache,
                    cfg,
                    DecodeDepth::BpAllOsd,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    40,
                    10,
                );
                eprintln!("  -> decode result: {:?}", r.map(|d| d.sync_score));
            }
        }
    }

    let dir = sweep_dir();
    for snr_tag in ["m20", "m22", "m23", "m24", "m25"] {
        probe::<Fst4s30>(
            &dir,
            &format!("fst4_30_awgn_{snr_tag}"),
            &FST4_30_DOWNSAMPLE,
        );
    }
    probe::<Fst4s300>(&dir, "fst4_300_awgn_m33", &FST4_300_DOWNSAMPLE);
}

/// Diagnostic (issue #146, VK3NV's 2026-07-16 comment) — quantify whether
/// adding a genuine `nsym=4` LLR variant (the depth WSJT-X's
/// `get_fst4_bitmetrics.f90` includes in its 1/2/4/8 ladder but that
/// `LlrSet`'s 4 fixed slots currently skip — FST4's `LLR_NSYM_MAX=8` runs
/// `nsym ∈ {1, 2, 8}`, never 4) would recover any additional trials, BEFORE
/// paying for the structural change (a 5th `LlrSet` slot touching the
/// shared BP staircase in `core::pipeline.rs`).
///
/// For every trial in the near-threshold SNR range of FST4-30/FST4-300,
/// runs the real production pipeline (`process_candidate_basic`, i.e.
/// nsym ∈ {1, 2, 8, bit-normalised} + OSD escalation) as the baseline, then
/// separately computes a standalone nsym=4 LLR variant
/// (`compute_llr_generic(cs, 4)`, whose `llrc` slot holds the nsym=4
/// metrics) and runs it through the same BP → OSD escalation the
/// production path uses (mirroring `pipeline.rs:246-350`, including the
/// FST4-specific "CRC-24-verified accepts unconditionally, no
/// `osd_max_errors` gate" bypass). Tallies the 2×2 outcome so the value of
/// a standalone nsym=4 pass is visible before touching `LlrSet`.
#[test]
#[ignore = "manual diagnostic, not a recall gate"]
fn fst4_diag_nsym4_ladder() {
    use mfsk_core::core::dsp::downsample::{DownsampleCfg, build_fft_cache, downsample_cached};
    use mfsk_core::core::equalize::EqMode;
    use mfsk_core::core::llr::{compute_llr_generic, symbol_spectra, sync_quality};
    use mfsk_core::core::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::core::sync::coarse_sync;
    use mfsk_core::core::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::core::{FecCodec, FecOpts, MessageCodec};
    use mfsk_core::fst4::decode::{FST4_30_DOWNSAMPLE, FST4_300_DOWNSAMPLE};
    use mfsk_core::fst4::{Fst4s30, Fst4s300};

    #[derive(Default)]
    struct Tally {
        total: u32,
        both_fail: u32,
        both_pass: u32,
        /// nsym=4 alone recovers a trial the real pipeline (nsym 1/2/8/d)
        /// misses — the number that decides whether the 5th-slot change
        /// in task #2 is worth it.
        nsym4_unique_win: u32,
        /// Sanity-check bucket: should stay ~0 (nsym=4 is a strict addition
        /// to the ladder, not a replacement for 1/2/8/d).
        baseline_only: u32,
    }

    fn probe<P>(
        dir: &Path,
        file_prefix: &str,
        cfg: &DownsampleCfg,
        trials: std::ops::RangeInclusive<u32>,
        tally: &mut Tally,
    ) where
        P: mfsk_core::core::Protocol,
        P::Fec: FecCodec,
        P::Msg: MessageCodec,
    {
        for trial in trials {
            let path = dir.join(format!("{file_prefix}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                eprintln!("skip {path:?}");
                continue;
            };

            let cands = coarse_sync::<P>(&audio, 100.0, 3000.0, 0.8, None, 50);
            let Some(cand) = cands
                .iter()
                .find(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
            else {
                eprintln!("{file_prefix} trial {trial}: no candidate near golden freq");
                continue;
            };

            let fft_cache = build_fft_cache(&audio, cfg);

            // Baseline: real production pipeline (nsym in {1,2,8,d} + OSD escalation).
            let baseline_ok = process_candidate_basic::<P>(
                cand,
                &fft_cache,
                cfg,
                DecodeDepth::BpAllOsd,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
                40,
                10,
            )
            .is_some();

            // Standalone nsym=4 pass: replicate process_candidate_basic's
            // sync/downsample/normalise path (pipeline.rs:138-193), then
            // compute *only* the nsym=4 LLR variant instead of the
            // production {1,2,8,d} set.
            let mut cd0 = downsample_cached(&fft_cache, cand.freq_hz, cfg);
            let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for c in cd0.iter_mut() {
                    *c *= inv;
                }
            }
            let s2 = fst4_sync_search::<P>(&cd0, cand);
            let ds_rate = 12_000.0 / P::NDOWN as f32;
            let df_hz = s2.freq_hz - cand.freq_hz;
            cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
            let i_start = s2.i0;

            let cs = symbol_spectra::<P>(&cd0, i_start);
            let nsync = sync_quality::<P>(&cs);
            let nsym4_ok = if nsync <= 10 {
                false
            } else {
                let llr4 = compute_llr_generic::<P, f32, f32>(&cs, 4);
                let fec = P::Fec::default();
                let bp_opts = FecOpts {
                    bp_max_iter: 30,
                    osd_depth: 0,
                    ap_mask: None,
                    verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                    ..FecOpts::default()
                };
                let mut ok = fec.decode_soft(&llr4.llrc, &bp_opts).is_some();
                if !ok && nsync >= 12 {
                    // Mirror pipeline.rs:284-320's FST4 bypass: OSD accepts
                    // any CRC-24-verified codeword unconditionally, no
                    // `osd_max_errors` gate.
                    let osd_depth: u32 = if nsync >= 18 { 3 } else { 2 };
                    let osd_opts = FecOpts {
                        bp_max_iter: 30,
                        osd_depth,
                        ap_mask: None,
                        verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                        ..FecOpts::default()
                    };
                    ok = fec.decode_soft(&llr4.llrc, &osd_opts).is_some();
                }
                ok
            };

            tally.total += 1;
            match (baseline_ok, nsym4_ok) {
                (false, false) => tally.both_fail += 1,
                (true, true) => tally.both_pass += 1,
                (false, true) => tally.nsym4_unique_win += 1,
                (true, false) => tally.baseline_only += 1,
            }
            eprintln!(
                "{file_prefix} trial {trial}: baseline={baseline_ok} nsym4_only={nsym4_ok} nsync={nsync}"
            );
        }
    }

    let dir = sweep_dir();
    let mut tally30 = Tally::default();
    for snr_tag in ["m20", "m21", "m22", "m23", "m24", "m25", "m26"] {
        probe::<Fst4s30>(
            &dir,
            &format!("fst4_30_awgn_{snr_tag}"),
            &FST4_30_DOWNSAMPLE,
            1..=20,
            &mut tally30,
        );
    }
    eprintln!(
        "\nFST4-30 (near-threshold m20-m26, {} trials): both_pass={} both_fail={} nsym4_unique_win={} baseline_only={}",
        tally30.total,
        tally30.both_pass,
        tally30.both_fail,
        tally30.nsym4_unique_win,
        tally30.baseline_only
    );

    let mut tally300 = Tally::default();
    for snr_tag in ["m31", "m32", "m33", "m34", "m35", "m36", "m37"] {
        probe::<Fst4s300>(
            &dir,
            &format!("fst4_300_awgn_{snr_tag}"),
            &FST4_300_DOWNSAMPLE,
            1..=20,
            &mut tally300,
        );
    }
    eprintln!(
        "FST4-300 (near-threshold m31-m37, {} trials): both_pass={} both_fail={} nsym4_unique_win={} baseline_only={}",
        tally300.total,
        tally300.both_pass,
        tally300.both_fail,
        tally300.nsym4_unique_win,
        tally300.baseline_only
    );
}

/// Diagnostic (issue #146) — quantify whether feeding OSD the running
/// accumulated sum of BP's early-iteration soft output (WSJT-X
/// `decode240_101.f90`'s `zsave` scheme — see
/// [`mfsk_core::fec::ldpc::bp::bp_llr_zsum`]'s doc comment) instead of
/// the raw channel LLR recovers any of the residual sensitivity gap.
/// Scoped to FST4-120 only — the sub-mode with the largest measured gap
/// vs WSJT-X's published threshold (1.3 dB, vs 0.5-0.8 dB for the other
/// four) after the nsym=4 ladder fix, and the user asked to skip a full
/// 5-mode run given how long that takes.
///
/// For each of BP's 4 main LLR variants (a=nsym1, b=nsym2, e=nsym4,
/// c=nsym8 — d/bit-normalised is skipped since it has no WSJT-X FST4
/// counterpart at all, see the `LLR_NSYM_MID` doc comment), on trials
/// where plain BP fails, compares `osd_decode_generic` fed the raw
/// channel LLR (what `Ldpc240_101::decode_soft` does today) against the
/// same OSD fed `bp_llr_zsum(llr, 2)` instead (matching WSJT-X's
/// `maxosd=2` case). Tallies the 2×2 outcome per (trial, variant) pair.
#[test]
#[ignore = "manual diagnostic, not a recall gate"]
fn fst4_diag_zsum_osd() {
    use mfsk_core::core::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::core::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::core::sync::coarse_sync;
    use mfsk_core::core::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::core::{MessageCodec, ModulationParams, Protocol};
    use mfsk_core::fec::ldpc::bp::{bp_decode_generic, bp_llr_zsum};
    use mfsk_core::fec::ldpc::osd::osd_decode_generic;
    use mfsk_core::fec::ldpc::params::Ldpc240_101Params;
    use mfsk_core::fec::ldpc240_101::LDPC_K;
    use mfsk_core::fst4::Fst4s120;
    use mfsk_core::fst4::decode::FST4_120_DOWNSAMPLE;

    #[derive(Default)]
    struct Tally {
        total_variant_attempts: u32,
        both_fail: u32,
        both_pass: u32,
        /// zsum-as-OSD-input recovers a variant plain-OSD-on-channel-LLR
        /// misses — the number that answers the actual question.
        zsum_unique_win: u32,
        /// Sanity-check bucket: should stay ~0 (zsum is not expected to
        /// be strictly worse than the raw channel LLR).
        raw_only: u32,
    }

    let dir = sweep_dir();
    let verify: Option<fn(&[u8]) -> bool> =
        Some(<<Fst4s120 as Protocol>::Msg as MessageCodec>::verify_info);
    let mut tally = Tally::default();

    for snr_tag in ["m27", "m28", "m29", "m30", "m31", "m32", "m33"] {
        for trial in 1..=20u32 {
            let path = dir.join(format!("fst4_120_awgn_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                eprintln!("skip {path:?}");
                continue;
            };

            let cands = coarse_sync::<Fst4s120>(&audio, 100.0, 3000.0, 0.8, None, 50);
            let Some(cand) = cands
                .iter()
                .find(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
            else {
                continue;
            };

            let fft_cache = build_fft_cache(&audio, &FST4_120_DOWNSAMPLE);
            let mut cd0 = downsample_cached(&fft_cache, cand.freq_hz, &FST4_120_DOWNSAMPLE);
            let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for c in cd0.iter_mut() {
                    *c *= inv;
                }
            }
            let s2 = fst4_sync_search::<Fst4s120>(&cd0, cand);
            let ds_rate = 12_000.0 / Fst4s120::NDOWN as f32;
            let df_hz = s2.freq_hz - cand.freq_hz;
            cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);

            let cs = symbol_spectra::<Fst4s120>(&cd0, s2.i0);
            let nsync = sync_quality::<Fst4s120>(&cs);
            if nsync <= 10 {
                continue;
            }

            let llr_set = compute_llr::<Fst4s120, f32>(&cs);
            for (name, llr) in [
                ("a", &llr_set.llra),
                ("b", &llr_set.llrb),
                ("e", &llr_set.llre),
                ("c", &llr_set.llrc),
            ] {
                if llr.is_empty() {
                    continue;
                }
                if bp_decode_generic::<Ldpc240_101Params>(llr, None, 30, verify).is_some() {
                    continue; // BP already succeeds — not an OSD-input question for this variant.
                }
                let raw_ok =
                    osd_decode_generic::<Ldpc240_101Params>(llr, 3, LDPC_K, verify).is_some();
                let zsum = bp_llr_zsum::<Ldpc240_101Params>(llr, 2);
                let zsum_ok =
                    osd_decode_generic::<Ldpc240_101Params>(&zsum, 3, LDPC_K, verify).is_some();

                tally.total_variant_attempts += 1;
                match (raw_ok, zsum_ok) {
                    (false, false) => tally.both_fail += 1,
                    (true, true) => tally.both_pass += 1,
                    (false, true) => tally.zsum_unique_win += 1,
                    (true, false) => tally.raw_only += 1,
                }
                eprintln!(
                    "fst4_120_awgn_{snr_tag}_{trial:02} variant={name}: raw_osd={raw_ok} zsum_osd={zsum_ok} nsync={nsync}"
                );
            }
        }
    }

    eprintln!(
        "\nFST4-120 zsum-vs-raw OSD input ({} variant-attempts): both_pass={} both_fail={} zsum_unique_win={} raw_only={}",
        tally.total_variant_attempts,
        tally.both_pass,
        tally.both_fail,
        tally.zsum_unique_win,
        tally.raw_only
    );
}

/// Throwaway diagnostic (issue #148, VK3NV's blind-paired FST4-120x2
/// proposal) — before claiming an LLR-combining scheme should recover
/// "close to the ideal ~3dB gain" in AWGN, check whether near-threshold
/// failures are actually decode failures (BP/OSD couldn't correct given a
/// found candidate — the case LLR combining helps) or sync failures (no
/// candidate near the golden freq at all — LLR combining does nothing for
/// these, since there's no per-slot LLR vector to combine if the
/// candidate was never found). If a meaningful fraction of near-threshold
/// failures are sync failures, the achievable gain from LLR-only
/// combining is capped well below the naive 3dB even in clean AWGN.
#[test]
#[ignore = "manual diagnostic, not a recall gate"]
fn fst4_120_diag_sync_vs_decode_failure() {
    use mfsk_core::core::equalize::EqMode;
    use mfsk_core::core::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::core::sync::coarse_sync;
    use mfsk_core::fst4::Fst4s120;
    use mfsk_core::fst4::decode::FST4_120_DOWNSAMPLE;

    let dir = sweep_dir();
    for snr_tag in ["m29", "m30", "m31", "m32"] {
        let mut n_total = 0;
        let mut n_no_candidate = 0;
        let mut n_candidate_decode_fail = 0;
        let mut n_decode_ok = 0;
        for trial in 1..=20 {
            let path = dir.join(format!("fst4_120_awgn_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            n_total += 1;
            let cands = coarse_sync::<Fst4s120>(&audio, 100.0, 3000.0, 0.8, None, 50);
            let near: Vec<_> = cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            if near.is_empty() {
                n_no_candidate += 1;
                eprintln!("fst4_120_awgn_{snr_tag}_{trial:02}: NO candidate near golden freq");
                continue;
            }
            let fft_cache =
                mfsk_core::core::dsp::downsample::build_fft_cache(&audio, &FST4_120_DOWNSAMPLE);
            let mut ok = false;
            for c in &near {
                if let Some(d) = process_candidate_basic::<Fst4s120>(
                    c,
                    &fft_cache,
                    &FST4_120_DOWNSAMPLE,
                    DecodeDepth::BpAllOsd,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    40,
                    10,
                ) {
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(d.message77());
                    if unpack77(&m77).as_deref() == Some(GOLDEN_MSG) {
                        ok = true;
                        break;
                    }
                }
            }
            if ok {
                n_decode_ok += 1;
            } else {
                n_candidate_decode_fail += 1;
                eprintln!(
                    "fst4_120_awgn_{snr_tag}_{trial:02}: candidate found ({} near) but decode FAILED",
                    near.len()
                );
            }
        }
        eprintln!(
            "== fst4_120_awgn_{snr_tag}: total={n_total} decode_ok={n_decode_ok} candidate_found_decode_fail={n_candidate_decode_fail} no_candidate={n_no_candidate} =="
        );
    }
}
