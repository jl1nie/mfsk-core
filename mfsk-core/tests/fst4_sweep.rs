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
//! cargo test --test fst4_sweep --release \
//!   --features fst4,fft-rustfft,parallel,uvpacket,internal-testing \
//!   -- --ignored --nocapture
//! ```
//!
//! (`uvpacket` is only required because `tests/common/channel.rs`, pulled in
//! via `mod common`, unconditionally imports `mfsk_core::uvpacket` — unrelated
//! to FST4 itself. `internal-testing` (issue #203) is required because this
//! file calls `engine::pipeline::{process_candidate_basic, osd_escalation_gates,
//! GenericPipelineProtocol}` directly, which are `pub(crate)` on the default
//! feature set. `MFSK_FST4_SWEEP_DIR` overrides the default corpus location
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

fn decode_wav_fst4<P>(audio: &[i16]) -> bool
where
    P: mfsk_core::msg::decode_request::FrameDecodable<
            DecodeResult = mfsk_core::fst4::decode::DecodeResult,
        >,
{
    use mfsk_core::msg::decode_request::DecodeRequest;
    DecodeRequest::<P>::new(audio, 100.0, 3000.0, 0.8, 50)
        .decode()
        .results
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
    use mfsk_core::fst4::Fst4s15;
    decode_wav_fst4::<Fst4s15>(audio)
}
fn decode_wav_fst4_30(audio: &[i16]) -> bool {
    use mfsk_core::fst4::Fst4s30;
    decode_wav_fst4::<Fst4s30>(audio)
}
fn decode_wav_fst4_60(audio: &[i16]) -> bool {
    use mfsk_core::fst4::Fst4s60;
    decode_wav_fst4::<Fst4s60>(audio)
}
fn decode_wav_fst4_120(audio: &[i16]) -> bool {
    use mfsk_core::fst4::Fst4s120;
    decode_wav_fst4::<Fst4s120>(audio)
}
fn decode_wav_fst4_300(audio: &[i16]) -> bool {
    use mfsk_core::fst4::Fst4s300;
    decode_wav_fst4::<Fst4s300>(audio)
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
/// `engine::pipeline::process_candidate_basic`.
#[test]
#[ignore = "manual diagnostic, not a recall gate"]
fn fst4_diag_weak_trials() {
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::fst4::decode::{FST4_30_DOWNSAMPLE, FST4_300_DOWNSAMPLE};
    use mfsk_core::fst4::{Fst4s30, Fst4s300};

    fn probe<P: mfsk_core::engine::pipeline::GenericPipelineProtocol>(
        dir: &std::path::Path,
        file_prefix: &str,
        cfg: &mfsk_core::engine::dsp::downsample::DownsampleCfg,
    ) where
        P::Fec: mfsk_core::engine::protocol::BpPooledFec,
    {
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
            let fft_cache = mfsk_core::engine::dsp::downsample::build_fft_cache(&audio, cfg);
            for c in &near {
                let r = process_candidate_basic::<P>(
                    c,
                    &fft_cache,
                    cfg,
                    DecodeDepth::FULL,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
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
/// shared BP staircase in `engine::pipeline.rs`).
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
    use mfsk_core::engine::dsp::downsample::{DownsampleCfg, build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::llr::{compute_llr_generic, symbol_spectra, sync_quality};
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec};
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
        P: mfsk_core::engine::pipeline::GenericPipelineProtocol,
        P::Fec: FecCodec + mfsk_core::engine::protocol::BpPooledFec,
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
                DecodeDepth::FULL,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
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
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{MessageCodec, ModulationParams, Protocol};
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
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::coarse_sync;
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
                mfsk_core::engine::dsp::downsample::build_fft_cache(&audio, &FST4_120_DOWNSAMPLE);
            let mut ok = false;
            for c in &near {
                if let Some(d) = process_candidate_basic::<Fst4s120>(
                    c,
                    &fft_cache,
                    &FST4_120_DOWNSAMPLE,
                    DecodeDepth::FULL,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
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

/// Phase 0 diagnostic (new investigation, mirrors the FT4
/// `ft4_diag_candidate_cost_split` methodology from
/// `~/.claude/plans/dapper-soaring-nest.md`): is FST4-60A slow
/// (`BENCHMARKS.md`: 2.60 s, slowest golden-WAV decode of any protocol)
/// for the same structural reason FT4 was — a generic 2-D (freq × lag)
/// Costas-correlation `engine::sync::coarse_sync` computed for a protocol
/// whose WSJT-X reference candidate finder (`get_candidates_fst4` in
/// `fst4_decode.f90:802-877`) has no lag dimension at all (a CLEAN-style
/// iterative-peak periodogram, single wideband FFT, no per-lag grid)?
///
/// Measures, on the real WSJT-X FST4-60 golden WAV: `coarse_sync`
/// candidate count / distinct-frequency count, and the wall-clock split
/// between `coarse_sync` itself, `fst4_sync_search`'s coherent full-slot
/// Δt search, and everything after it (symbol_spectra + LLR + BP + OSD)
/// — plus the real production `decode_frame` wall-clock for direct
/// comparison against the `BENCHMARKS.md` figure. Measurement only, no
/// assertions.
#[test]
#[ignore = "manual diagnostic — FST4-60A coarse-sync cost split (new investigation, mirrors dapper-soaring-nest plan Phase 0)"]
fn fst4_60_diag_candidate_cost_split() {
    use std::time::{Duration, Instant};

    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::{SyncCandidate, coarse_sync};
    use mfsk_core::engine::sync2d::fst4_sync_search;
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    // Was `10`, not production's real `16` (`fst4/decode.rs::SYNC_Q_MIN`)
    // — a real diagnostic/production mismatch found during the issue
    // #244/#245 investigation: this test's own looser gate let more
    // candidates through to the expensive LLR/BP/OSD stages than
    // `decode_frame` actually does, so its per-stage cost breakdown
    // didn't reproduce production's real cost distribution. Fixed to
    // match; see `~/.claude/plans/moonlit-snuggling-puzzle.md` and
    // `engine::pipeline::decode_frame_impl`'s `MFSK_TRACE_STAGE_FST4`
    // env var for the now-authoritative real-production-path measurement
    // this standalone loop was meant to approximate.
    const SYNC_Q_MIN: u32 = 16;

    fn freq_bucket_count(cands: &[SyncCandidate]) -> usize {
        let mut buckets: Vec<i32> = cands
            .iter()
            .map(|c| (c.freq_hz / 4.0).round() as i32)
            .collect();
        buckets.sort_unstable();
        buckets.dedup();
        buckets.len()
    }

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let golden_path = Path::new(&manifest).join("../../WSJT-X/samples/FST4+FST4W/210115_0058.wav");
    let Ok(golden_path) = golden_path.canonicalize() else {
        eprintln!("skipping: WSJT-X FST4 sample not found (sibling checkout)");
        return;
    };
    let Some(audio) = load_wav_i16_opt(&golden_path) else {
        eprintln!("skipping: WAV not 12 kHz mono PCM-16");
        return;
    };

    let t0 = Instant::now();
    let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.0, None, 50);
    let coarse_dt = t0.elapsed();

    eprintln!(
        "FST4-60A golden: {} candidates, {} distinct freq buckets (avg {:.1} cand/freq), coarse_sync={:.1}ms",
        cands.len(),
        freq_bucket_count(&cands),
        cands.len() as f32 / freq_bucket_count(&cands).max(1) as f32,
        coarse_dt.as_secs_f64() * 1000.0
    );

    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let mut total_downsample = Duration::ZERO;
    let mut total_sync_search = Duration::ZERO;
    let mut total_process = Duration::ZERO;
    let mut n_decoded = 0usize;
    for c in &cands {
        let t0 = Instant::now();
        let cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
        total_downsample += t0.elapsed();

        let t0 = Instant::now();
        let _ = fst4_sync_search::<Fst4s60>(&cd0, c);
        total_sync_search += t0.elapsed();

        let t0 = Instant::now();
        let r = process_candidate_basic::<Fst4s60>(
            c,
            &fft_cache,
            &FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
        );
        total_process += t0.elapsed();
        if r.is_some() {
            n_decoded += 1;
        }
    }
    let llr_bp_osd = total_process
        .saturating_sub(total_downsample)
        .saturating_sub(total_sync_search);
    eprintln!(
        "  per-cand[downsample={:.1}ms sync_search={:.1}ms llr+bp+osd(inferred)={:.1}ms] decoded={n_decoded}",
        total_downsample.as_secs_f64() * 1000.0,
        total_sync_search.as_secs_f64() * 1000.0,
        llr_bp_osd.as_secs_f64() * 1000.0
    );

    // Real production entry point, for direct comparison against
    // `BENCHMARKS.md`'s "Decode speed" table (2.60 s as of this
    // investigation's start).
    let t0 = Instant::now();
    let decodes = mfsk_core::msg::decode_request::DecodeRequest::<mfsk_core::fst4::Fst4s60>::new(
        &audio, 100.0, 3000.0, 1.0, 50,
    )
    .decode()
    .results;
    let dt = t0.elapsed();
    eprintln!(
        "FST4-60A golden: decode_frame wall-clock = {:.1} ms, {} decode(s)",
        dt.as_secs_f64() * 1000.0,
        decodes.len()
    );
}

/// Same cost-split methodology as [`fst4_60_diag_candidate_cost_split`],
/// applied to FST4-300 (5-min slot, longest sub-mode) — issue
/// mfsk-core#perf-review Phase 0: `fst4_60_diag_candidate_cost_split`'s
/// original investigation (FST4_BENCHMARK.md §8) only ever profiled
/// FST4-60A; no per-stage breakdown exists for the long sub-modes,
/// which have no comparable prior perf-tuning pass either. Uses the
/// first slot of the WSJT-X sample `201230_0300.wav`, same asset
/// `bench_qso3_busy_timing.rs::timing_fst4_300` already depends on
/// (sibling `../../WSJT-X` checkout — skips cleanly if absent).
#[test]
#[ignore = "manual diagnostic — FST4-300 coarse-sync cost split (perf-review Phase 0)"]
fn fst4_300_diag_candidate_cost_split() {
    use std::time::{Duration, Instant};

    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::{SyncCandidate, coarse_sync};
    use mfsk_core::engine::sync2d::fst4_sync_search;
    use mfsk_core::fst4::Fst4s300;
    use mfsk_core::fst4::decode::FST4_300_DOWNSAMPLE;

    // Was `10`, not production's real `16` (`fst4/decode.rs::SYNC_Q_MIN`)
    // — a real diagnostic/production mismatch found during the issue
    // #244/#245 investigation: this test's own looser gate let more
    // candidates through to the expensive LLR/BP/OSD stages than
    // `decode_frame` actually does, so its per-stage cost breakdown
    // didn't reproduce production's real cost distribution. Fixed to
    // match; see `~/.claude/plans/moonlit-snuggling-puzzle.md` and
    // `engine::pipeline::decode_frame_impl`'s `MFSK_TRACE_STAGE_FST4`
    // env var for the now-authoritative real-production-path measurement
    // this standalone loop was meant to approximate.
    const SYNC_Q_MIN: u32 = 16;

    fn freq_bucket_count(cands: &[SyncCandidate]) -> usize {
        let mut buckets: Vec<i32> = cands
            .iter()
            .map(|c| (c.freq_hz / 4.0).round() as i32)
            .collect();
        buckets.sort_unstable();
        buckets.dedup();
        buckets.len()
    }

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let golden_path = Path::new(&manifest).join("../../WSJT-X/samples/FST4+FST4W/201230_0300.wav");
    let Ok(golden_path) = golden_path.canonicalize() else {
        eprintln!("skipping: WSJT-X FST4 sample not found (sibling checkout)");
        return;
    };
    let Some(full) = load_wav_i16_opt(&golden_path) else {
        eprintln!("skipping: WAV not 12 kHz mono PCM-16");
        return;
    };
    let audio: Vec<i16> = full.iter().take(300 * 12_000).copied().collect();

    let t0 = Instant::now();
    let cands = coarse_sync::<Fst4s300>(&audio, 100.0, 3000.0, 1.0, None, 50);
    let coarse_dt = t0.elapsed();

    eprintln!(
        "FST4-300 golden (slot 0): {} candidates, {} distinct freq buckets (avg {:.1} cand/freq), coarse_sync={:.1}ms",
        cands.len(),
        freq_bucket_count(&cands),
        cands.len() as f32 / freq_bucket_count(&cands).max(1) as f32,
        coarse_dt.as_secs_f64() * 1000.0
    );

    let fft_cache = build_fft_cache(&audio, &FST4_300_DOWNSAMPLE);
    let mut total_downsample = Duration::ZERO;
    let mut total_sync_search = Duration::ZERO;
    let mut total_process = Duration::ZERO;
    let mut n_decoded = 0usize;
    for c in &cands {
        let t0 = Instant::now();
        let cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_300_DOWNSAMPLE);
        total_downsample += t0.elapsed();

        let t0 = Instant::now();
        let _ = fst4_sync_search::<Fst4s300>(&cd0, c);
        total_sync_search += t0.elapsed();

        let t0 = Instant::now();
        let r = process_candidate_basic::<Fst4s300>(
            c,
            &fft_cache,
            &FST4_300_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
        );
        total_process += t0.elapsed();
        if r.is_some() {
            n_decoded += 1;
        }
    }
    let llr_bp_osd = total_process
        .saturating_sub(total_downsample)
        .saturating_sub(total_sync_search);
    eprintln!(
        "  per-cand[downsample={:.1}ms sync_search={:.1}ms llr+bp+osd(inferred)={:.1}ms] decoded={n_decoded}",
        total_downsample.as_secs_f64() * 1000.0,
        total_sync_search.as_secs_f64() * 1000.0,
        llr_bp_osd.as_secs_f64() * 1000.0
    );

    let t0 = Instant::now();
    let decodes = mfsk_core::msg::decode_request::DecodeRequest::<mfsk_core::fst4::Fst4s300>::new(
        &audio, 100.0, 3000.0, 1.0, 50,
    )
    .decode()
    .results;
    let dt = t0.elapsed();
    eprintln!(
        "FST4-300 golden: decode_frame wall-clock = {:.1} ms, {} decode(s)",
        dt.as_secs_f64() * 1000.0,
        decodes.len()
    );
}

/// Phase 0 follow-up: `fst4_60_diag_candidate_cost_split` found the
/// 8+ second cost is almost entirely inside `process_candidate_basic`
/// (LLR + BP + OSD), not `coarse_sync`/`fst4_sync_search` — the
/// opposite of what drove FT4's slowness. Hypothesis: `nsync` gates
/// `osd_attempt_min`/`osd_depth3_min` are hardcoded `(12, 18)`
/// (`core/pipeline.rs`), calibrated against FT8's `N_SYNC=21` — but
/// FST4-60's `N_SYNC=40` (`fst4/mod.rs`), so 18/40=45% is a far looser
/// bar than FT8's 18/21=86%, letting most candidates escalate into the
/// most expensive OSD depth-4 (+ `Ldpc240_101`'s raw-then-zsum
/// *double* OSD attempt per depth, `fec/ldpc240_101/mod.rs:148-197`)
/// tier even when they have no real chance of succeeding.
///
/// Measures, per candidate: `nsync` (out of 40), whether plain BP alone
/// succeeds, and — for BP failures — how much wall-clock the OSD
/// escalation (depth-2/3, then depth-4 if `nsync>=18`) burns before
/// giving up. No assertions.
#[test]
#[ignore = "manual diagnostic — FST4-60A OSD-escalation gate hypothesis (new investigation)"]
fn fst4_60_diag_osd_escalation() {
    use std::time::Instant;

    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    const BP_MAX_ITER: u32 = 30;

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let golden_path = Path::new(&manifest).join("../../WSJT-X/samples/FST4+FST4W/210115_0058.wav");
    let Ok(golden_path) = golden_path.canonicalize() else {
        eprintln!("skipping: WSJT-X FST4 sample not found (sibling checkout)");
        return;
    };
    let Some(audio) = load_wav_i16_opt(&golden_path) else {
        eprintln!("skipping: WAV not 12 kHz mono PCM-16");
        return;
    };

    let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.0, None, 50);
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
    let fec = <Fst4s60 as Protocol>::Fec::default();
    let verify_info =
        Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

    let mut n_bp_ok = 0usize;
    let mut n_no_osd_attempt = 0usize;
    let mut n_osd_2_3_only = 0usize;
    let mut n_osd_4_escalated = 0usize;
    let mut t_llr = std::time::Duration::ZERO;
    let mut t_bp = std::time::Duration::ZERO;
    let mut t_osd_2_3 = std::time::Duration::ZERO;
    let mut t_osd_4 = std::time::Duration::ZERO;
    // Real production gate — see `osd_escalation_gates`'s doc comment in
    // `core/pipeline.rs` (FST4 uses (12, 20), not FT8's (12, 18)).
    let (osd_attempt_min, osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    for c in &cands {
        let cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
        let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
        let df_hz = s2.freq_hz - c.freq_hz;
        let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
        let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
        let nsync = sync_quality::<Fst4s60>(&cs);

        let t0 = Instant::now();
        let llr_set = compute_llr::<Fst4s60, f32>(&cs);
        t_llr += t0.elapsed();
        let variants: Vec<&Vec<f32>> = [
            Some(&llr_set.llra),
            Some(&llr_set.llrb),
            if llr_set.llre.is_empty() {
                None
            } else {
                Some(&llr_set.llre)
            },
            Some(&llr_set.llrc),
            Some(&llr_set.llrd),
        ]
        .into_iter()
        .flatten()
        .collect();

        let bp_opts = FecOpts {
            bp_max_iter: BP_MAX_ITER,
            osd_depth: 0,
            ap_mask: None,
            verify_info,
            ..FecOpts::default()
        };
        let t0 = Instant::now();
        let bp_ok = variants
            .iter()
            .any(|llr| fec.decode_soft(llr, &bp_opts).is_some());
        t_bp += t0.elapsed();
        if bp_ok {
            n_bp_ok += 1;
            continue;
        }

        if nsync < osd_attempt_min {
            n_no_osd_attempt += 1;
            continue;
        }
        let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
        let t0 = Instant::now();
        let osd_opts = FecOpts {
            bp_max_iter: BP_MAX_ITER,
            osd_depth,
            ap_mask: None,
            verify_info,
            ..FecOpts::default()
        };
        let ok_2_3 = variants
            .iter()
            .any(|llr| fec.decode_soft(llr, &osd_opts).is_some());
        t_osd_2_3 += t0.elapsed();
        if ok_2_3 {
            n_osd_2_3_only += 1;
            continue;
        }

        if nsync >= osd_depth3_min {
            n_osd_4_escalated += 1;
            let t0 = Instant::now();
            let osd4_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 4,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let _ = variants
                .iter()
                .any(|llr| fec.decode_soft(llr, &osd4_opts).is_some());
            t_osd_4 += t0.elapsed();
        }
    }

    eprintln!(
        "FST4-60A golden ({} candidates): bp_ok={n_bp_ok} \
         no_osd_attempt(nsync<{osd_attempt_min})={n_no_osd_attempt} \
         osd_2_3_only={n_osd_2_3_only} \
         osd_4_escalated(nsync>={osd_depth3_min})={n_osd_4_escalated}",
        cands.len()
    );
    eprintln!(
        "  wall-clock: llr={:.1}ms bp={:.1}ms osd_depth_2_3={:.1}ms osd_depth_4={:.1}ms",
        t_llr.as_secs_f64() * 1000.0,
        t_bp.as_secs_f64() * 1000.0,
        t_osd_2_3.as_secs_f64() * 1000.0,
        t_osd_4.as_secs_f64() * 1000.0
    );
}

/// Calibration diagnostic: the naive fix (reusing FT4's exact
/// `N_SYNC`-scaled ratio — `40 * 12/21 ~ 23`, `40 * 18/21 ~ 34` — for
/// FST4 too) measured as a real ~0.5 dB AWGN sensitivity *regression*
/// (`fst4_snr_sweep`, AWGN, controlled A/B on identical code:
/// pre-fix ≈-27.6dB, post-fix ≈-27.1dB) — unlike FT4, where the
/// analogous scaling only ever *raised* an unreachable threshold.
/// Measures the actual `nsync` distribution of candidates that only
/// succeed via OSD depth-3/4 (not plain BP, not depth-2) across the
/// FST4-60 AWGN near-crossing region, so a safe cutoff can be picked
/// from real data instead of a cross-protocol ratio.
#[test]
#[ignore = "manual diagnostic — FST4 OSD-escalation gate recalibration (new investigation)"]
fn fst4_60_diag_osd_depth34_nsync_floor() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    const BP_MAX_ITER: u32 = 30;

    let dir = sweep_dir();
    let mut work: Vec<(&str, u32)> = Vec::new();
    for snr_tag in ["m29", "m28", "m27", "m26"] {
        for trial in 1..=20u32 {
            work.push((snr_tag, trial));
        }
    }

    let process_one = |&(snr_tag, trial): &(&str, u32)| -> Vec<(u32, u8)> {
        let path = dir.join(format!("fst4_60_awgn_{snr_tag}_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            return Vec::new();
        };
        let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.0, None, 50);
        let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
        let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
        let fec = <Fst4s60 as Protocol>::Fec::default();
        let verify_info =
            Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

        let mut out = Vec::new();
        for c in &cands {
            if (c.freq_hz - GOLDEN_FREQ_HZ).abs() > FREQ_TOL_HZ {
                continue;
            }
            let cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            let df_hz = s2.freq_hz - c.freq_hz;
            let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
            let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
            let nsync = sync_quality::<Fst4s60>(&cs);

            let llr_set = compute_llr::<Fst4s60, f32>(&cs);
            let variants: Vec<&Vec<f32>> = [
                Some(&llr_set.llra),
                Some(&llr_set.llrb),
                if llr_set.llre.is_empty() {
                    None
                } else {
                    Some(&llr_set.llre)
                },
                Some(&llr_set.llrc),
                Some(&llr_set.llrd),
            ]
            .into_iter()
            .flatten()
            .collect();

            let bp_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 0,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let is_golden = |llr: &Vec<f32>, opts: &FecOpts| -> bool {
                fec.decode_soft(llr, opts).is_some_and(|r| {
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(&r.info[..77]);
                    unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                })
            };

            let bp_ok = variants.iter().any(|llr| is_golden(llr, &bp_opts));
            if bp_ok {
                continue; // not an OSD-rescue case
            }

            let osd2_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 2,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let ok_depth2 = variants.iter().any(|llr| is_golden(llr, &osd2_opts));
            if ok_depth2 {
                out.push((nsync, 2));
                continue;
            }

            let osd3_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 3,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let ok_depth3 = variants.iter().any(|llr| is_golden(llr, &osd3_opts));
            if ok_depth3 {
                out.push((nsync, 3));
                continue;
            }

            let osd4_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 4,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let ok_depth4 = variants.iter().any(|llr| is_golden(llr, &osd4_opts));
            if ok_depth4 {
                out.push((nsync, 4));
            }
        }
        out
    };

    #[cfg(feature = "parallel")]
    let per_file: Vec<Vec<(u32, u8)>> = work.par_iter().map(process_one).collect();
    #[cfg(not(feature = "parallel"))]
    let per_file: Vec<Vec<(u32, u8)>> = work.iter().map(process_one).collect();

    let mut rescues: Vec<(u32, u8)> = per_file.into_iter().flatten().collect();
    rescues.sort_by_key(|&(nsync, _)| nsync);

    eprintln!(
        "OSD depth-2/3/4 rescues (BP failed, OSD succeeded), near-crossing AWGN (-29..-26 dB): n={}",
        rescues.len()
    );
    eprintln!("(nsync, depth) sorted by nsync: {rescues:?}");
    if let Some(&(min_nsync, _)) = rescues.first() {
        eprintln!("minimum nsync among all OSD rescues: {min_nsync} (out of N_SYNC=40)");
    }
    let depth3_or_4: Vec<u32> = rescues
        .iter()
        .filter(|&&(_, d)| d >= 3)
        .map(|&(n, _)| n)
        .collect();
    eprintln!(
        "depth-3/4-specifically rescues: n={}, min nsync={:?}",
        depth3_or_4.len(),
        depth3_or_4.iter().min()
    );
}

/// Reported SNR must track the injected SNR, **for every sub-mode**
/// (issue #255 §4 follow-up).
///
/// `fst4::baseline::fst4_snr_db` ports `fst4_decode.f90:592-621`, whose
/// calibration is per sub-mode (`snr_calfac` = 800/600/430/390/340 for
/// 15/30/60/120/300, plus a `10·log10(8200/nsps)` term). When it
/// shipped, only FST4-60 had been checked — the only sub-mode with a
/// real off-air recording available locally — and the other four were
/// left as "share the same formula/derivation but aren't individually
/// confirmed". This closes that gap using the `fst4sim` corpus.
///
/// Why injected SNR is a valid reference: a real local `jt9 -7` build
/// reports within ~1 dB of the injected value on this same corpus
/// across all five sub-modes (measured 2026-08-11 — FST4-15 m10→-10,
/// m18→-18; FST4-30 m15→-14, m22→-21; FST4-60 m15→-15, m25→-25;
/// FST4-120 m20→-20, m28→-28; FST4-300 m24→-24, m32→-32).
///
/// Measured mean error over the AWGN corpus (3 trials/cell):
///
/// | sub-mode | 15 | 30 | 60 | 120 | 300 |
/// |---|---:|---:|---:|---:|---:|
/// | mean err | -0.45 | +0.43 | -0.01 | -0.19 | **-1.26** |
///
/// FST4-300 carries a real, SNR-independent ~1.3 dB offset (~1.9 dB
/// under CCIR-moderate fading) that the other four don't. It is *not*
/// a wrong parameter — `nsps`/`ndown`/`snr_calfac` were each checked
/// against `fst4_decode.f90:182-214,597-613` and all match exactly.
/// The likely origin is `fst4_snr_db`'s `xsig · NDOWN` scale
/// correction, which was derived and confirmed on FST4-60 (note that
/// sub-mode's -0.01 dB here). Left as a measured, documented residual
/// rather than absorbed into a per-sub-mode fudge factor.
///
/// **Message-matching is load-bearing.** Taking `results.first()`
/// instead of the decode that carries the corpus message silently
/// admits spurious low-SNR decodes: doing so inflated FST4-15's
/// `max |err|` from 1.45 dB to 6.43 dB and produced a fake
/// "FST4-300 is -5 dB off under fading" signal that was entirely an
/// artifact of a constant-valued false decode.
#[test]
fn fst4_reported_snr_tracks_injected_all_submodes() {
    /// Per-sub-mode mean error budget. Wide enough for FST4-300's
    /// known ~1.3 dB residual plus corpus-regeneration noise, tight
    /// enough that losing the formula entirely (the pre-`e1200b6`
    /// state was ~2 dB out, the generic heuristic far more) fails.
    const MEAN_ERR_TOL_DB: f32 = 2.5;
    /// Keep the default `cargo test` run bounded — FST4-300 files are
    /// 300 s of audio each.
    const MAX_FILES_PER_SUBMODE: usize = 4;
    const EXPECT_MSG: &str = "CQ JL1NIE PM95";

    let wavs = collect_wavs(&sweep_dir());
    if wavs.is_empty() {
        eprintln!(
            "skipping fst4_reported_snr_tracks_injected_all_submodes: no fst4_*.wav in {:?} \
             (regenerate with scripts/gen_fst4_sweep_wavs.sh)",
            sweep_dir()
        );
        return;
    }

    macro_rules! check {
        ($proto:ty, $nsec:expr) => {{
            let mut picked: Vec<&WavMeta> = wavs
                .iter()
                .filter(|w| {
                    w.nsec == $nsec
                        && w.channel == "awgn"
                        && w.trial == 1
                        // Mid-range cells: strong enough to decode
                        // reliably, weak enough to be a real test.
                        && (-30..=-10).contains(&w.snr_db)
                })
                .collect();
            picked.sort_by_key(|w| w.snr_db);
            let step = (picked.len() / MAX_FILES_PER_SUBMODE).max(1);
            let picked: Vec<&&WavMeta> = picked
                .iter()
                .step_by(step)
                .take(MAX_FILES_PER_SUBMODE)
                .collect();

            let mut errs = Vec::new();
            for w in &picked {
                let Some(audio) = load_wav_i16_opt(&w.path) else {
                    continue;
                };
                let out = mfsk_core::msg::decode_request::DecodeRequest::<$proto>::new(
                    &audio, 100.0, 3000.0, 1.2, 50,
                )
                .decode();
                if let Some(d) = out.results.iter().find(|d| {
                    mfsk_core::msg::wsjt77::unpack77(d.message77()).as_deref() == Some(EXPECT_MSG)
                }) {
                    errs.push(d.snr_db - w.snr_db as f32);
                }
            }

            if errs.is_empty() {
                eprintln!("  FST4-{}: no decodes in the sampled cells — skipped", $nsec);
            } else {
                let mean = errs.iter().sum::<f32>() / errs.len() as f32;
                eprintln!(
                    "  FST4-{:<3} n={:<2} mean err {mean:+.2} dB",
                    $nsec,
                    errs.len()
                );
                assert!(
                    mean.abs() <= MEAN_ERR_TOL_DB,
                    "FST4-{} reported SNR is off by {mean:+.2} dB on average \
                     (tolerance ±{MEAN_ERR_TOL_DB}) — check `fst4::baseline::fst4_snr_db`'s \
                     `snr_calfac` for this sub-mode and its `xsig · NDOWN` scale correction",
                    $nsec
                );
            }
        }};
    }

    check!(mfsk_core::fst4::Fst4s15, 15);
    check!(mfsk_core::fst4::Fst4s30, 30);
    check!(mfsk_core::fst4::Fst4s60, 60);
    check!(mfsk_core::fst4::Fst4s120, 120);
    check!(mfsk_core::fst4::Fst4s300, 300);
}
