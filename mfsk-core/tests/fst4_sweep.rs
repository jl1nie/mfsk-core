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
                let raw_ok = osd_decode_generic::<Ldpc240_101Params>(llr, 3, LDPC_K, verify, false)
                    .is_some();
                let zsum = bp_llr_zsum::<Ldpc240_101Params>(llr, 2);
                let zsum_ok =
                    osd_decode_generic::<Ldpc240_101Params>(&zsum, 3, LDPC_K, verify, false)
                        .is_some();

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

/// VK3NV's issue #306 follow-up: before committing to a full
/// `npre1`/`npre2` genericisation, get "a cheap indication of the
/// ceiling" — how many candidates does WSJT-X's actual pruned OSD
/// search construct/test, on real FST4-60 data, against the
/// unpruned `C(101,3) ≈ 166,650` the current `osd_decode_generic`
/// combinatorial search walks? Runs
/// [`mfsk_core::fec::ldpc::osd::npre1_pattern_counts`] (a
/// counting-only port of `osd240_101.f90`'s `nord=1` pass — see its
/// own doc comment) against the real LLR of every candidate on the
/// golden WAV that reaches OSD in production (BP already failed,
/// `nsync >= osd_attempt_min`) — the same population
/// `fst4_60_diag_osd_escalation` above already characterises.
#[test]
#[ignore = "manual diagnostic — npre1 pattern-count ceiling estimate (issue #306 follow-up, VK3NV)"]
fn fst4_60_diag_npre1_pattern_counts() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::fec::ldpc::osd::npre1_pattern_counts;
    use mfsk_core::fec::ldpc::params::Ldpc240_101Params;
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    const BP_MAX_ITER: u32 = 30;
    // C(101,3) -- the unpruned order-3 combinatorial count
    // `osd_decode_generic` currently walks per candidate for FST4.
    const COMBINATORIAL_ORDER3: u64 = 101 * 100 * 99 / 6;

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
    let (osd_attempt_min, _osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    let mut n_reach_osd = 0usize;
    let mut totals: Vec<u32> = Vec::new();
    let mut postgates: Vec<u32> = Vec::new();

    for c in &cands {
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
        let bp_ok = variants
            .iter()
            .any(|llr| fec.decode_soft(llr, &bp_opts).is_some());
        if bp_ok || nsync < osd_attempt_min {
            continue; // not a candidate real production would send to OSD
        }

        n_reach_osd += 1;
        // One LLR variant is enough for a scale estimate -- `llra`
        // (nsym=1), matching the cheapest variant OSD is tried on
        // first in production.
        if let Some((ntotal, npostgate)) =
            npre1_pattern_counts::<Ldpc240_101Params>(&llr_set.llra)
        {
            eprintln!(
                "cand freq={:.1} nsync={nsync}: npre1 ntotal={ntotal} npostgate={npostgate} \
                 (vs unpruned order-3 = {COMBINATORIAL_ORDER3})",
                c.freq_hz
            );
            totals.push(ntotal);
            postgates.push(npostgate);
        }
    }

    if totals.is_empty() {
        eprintln!("no candidates reached the OSD gate on this file — nothing to report");
        return;
    }
    let mean_total = totals.iter().sum::<u32>() as f64 / totals.len() as f64;
    let mean_postgate = postgates.iter().sum::<u32>() as f64 / postgates.len() as f64;
    eprintln!(
        "{n_reach_osd} candidates reached OSD. npre1 mean ntotal={mean_total:.0} \
         (unpruned order-3={COMBINATORIAL_ORDER3}, ratio={:.1}x), \
         mean npostgate={mean_postgate:.0} (fraction of ntotal={:.3}%)",
        COMBINATORIAL_ORDER3 as f64 / mean_total,
        100.0 * mean_postgate / mean_total,
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

/// Recall trade-off diagnostic (issue #306 follow-up): with FST4-60's
/// candidate-loop cost on embedded now measured as ~99% the
/// `LLR_NSYM_MAX=8` staircase rung plus OSD (`embedded_shared::apps::
/// fst4_bench`'s module doc; the two real decodes on the golden WAV
/// never reach either), does skipping the expensive parts actually
/// cost real AWGN recall — or, like WSPR's own `minsync2` gate,
/// mostly buy time on candidates that were never going to decode
/// anyway? A one-file observation isn't a recall claim on its own
/// (this project's own established caution); this sweeps the real
/// AWGN corpus to check.
///
/// Four configurations, same candidate population, same
/// `compute_llr` call (it always computes every variant regardless —
/// this diagnostic only chooses which ones each config is *allowed*
/// to use for BP/OSD, not which are computed; this test measures
/// recall, not wall-clock):
///
/// - `full`: today's host default — nsym staircase to
///   `LLR_NSYM_MAX=8` (`llra`/`llrb`/`llre`/`llrc`/`llrd`) + OSD
///   escalation exactly as production dispatches it
///   (`osd_escalation_gates`'s real `nsync`-gated depth-2/3/4 logic,
///   matching `fst4_60_diag_osd_escalation` above).
/// - `bp_only`: same nsym staircase, OSD off — the existing
///   `DecodeDepth::BP_ONLY` production knob's recall shape.
/// - `no8_osd`: nsym staircase capped at `LLR_NSYM_MID=4` (`llrc`,
///   the `nsym=8` rung, excluded from the variant set), OSD still on —
///   tests whether OSD alone recovers what `nsym=8` would have caught.
/// - `no8_no_osd`: nsym capped at 4, OSD off too — the most aggressive
///   cut, closest in spirit to FT8's ship config (`DecodeDepth::
///   EMBEDDED` already skips OSD; this goes further and drops the
///   deepest LLR rung as well).
///
/// Each config's tally is monotonic by construction: `full ⊇ no8_osd
/// ⊇ no8_no_osd` and `full ⊇ bp_only` (a superset LLR-variant set or
/// an added OSD-escalation pass can only add successes, never remove
/// one) — a violation in the printed table would mean a bug in this
/// diagnostic, not a real result.
///
/// AWGN only (channel-independent question), FST4-60 only (issue
/// #306's actual embedded target — CLAUDE.md's own caution against
/// assuming a finding generalises across sub-modes without checking
/// the code path applies; `LLR_NSYM_MAX`/`LLR_NSYM_MID` are
/// per-protocol consts, unconfirmed here for 15/30/120/300).
#[test]
#[ignore = "manual diagnostic — FST4-60 recall trade-off: does skipping nsym=8/OSD cost real AWGN recall? (issue #306 follow-up)"]
fn fst4_60_diag_recall_tradeoff() {
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 20),
        ("m22", 20),
        ("m23", 20),
        ("m24", 20),
        ("m25", 20),
        ("m26", 100),
        ("m27", 100),
        ("m28", 20),
        ("m29", 20),
        ("m30", 20),
    ];
    recall_tradeoff_for_channel("awgn", SNR_TAGS);
}

/// Same trade-off sweep, CCIR-moderate fading — issue #306/#198
/// follow-up: does switching FST4's OSD to the WSJT-X-faithful
/// npre1/npre2 port change the `no8_osd` recall trade-off's picture
/// under fading, the way the nsym-depth sweep's AWGN-vs-CCIR comparison
/// (`fst4_60_diag_nsym_depth_sweep`/`_ccir_moderate`) found for plain
/// depth. Corpus widened to 100 trials/SNR at m26/m27 the same way the
/// AWGN corpus was (`fst4sim` is deterministic per trial index —
/// trials 1-20 confirmed byte-identical to the original 20-trial CCIR-
/// moderate corpus before generating more):
/// ```sh
/// for snr in -26 -27; do
///   tag=$(printf "m%02d" $(( -snr )))
///   tmpd=$(mktemp -d)
///   (cd "$tmpd" && /path/to/target/fst4sim/fst4sim \
///     "CQ JL1NIE PM95" 60 1500 0.0 0.5 1.0 100 "$snr" F >/dev/null)
///   for T in $(seq 21 100); do
///     src="$tmpd/000000_$(printf '%04d' $T).wav"
///     dest="embedded-poc/assets/fst4_sweep/fst4_60_ccir_moderate_${tag}_$(printf '%02d' $T).wav"
///     [[ -f "$src" ]] && cp "$src" "$dest"
///   done
///   rm -rf "$tmpd"
/// done
/// ```
#[test]
#[ignore = "manual diagnostic — FST4-60 recall trade-off, CCIR-moderate fading (issue #306/#198 follow-up)"]
fn fst4_60_diag_recall_tradeoff_ccir_moderate() {
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 100),
        ("m22", 100),
        ("m23", 100),
        ("m24", 100),
        ("m25", 100),
        ("m26", 100),
        ("m27", 100),
        ("m28", 100),
        ("m29", 100),
        ("m30", 100),
    ];
    recall_tradeoff_for_channel("ccir_moderate", SNR_TAGS);
}

/// Same as [`fst4_60_diag_recall_tradeoff_ccir_moderate`] but forced
/// onto the pre-port unpruned combinatorial OSD search
/// (`fst4_osd_diag_force_old`) — a same-corpus, same-code old-vs-new
/// comparison under CCIR-moderate fading, mirroring the AWGN
/// old-vs-new comparison already done for `full`/`no8_osd`. There was
/// no pre-existing CCIR-moderate baseline to diff against (this
/// diagnostic and its 100-trial m26/m27 corpus are both new), so this
/// wrapper exists specifically to manufacture one via the override
/// rather than trusting an absolute number alone.
#[test]
#[ignore = "manual diagnostic — FST4-60 recall trade-off, CCIR-moderate, forced old OSD for old-vs-new comparison (issue #306/#198 follow-up)"]
fn fst4_60_diag_recall_tradeoff_ccir_moderate_old_osd() {
    use mfsk_core::fec::ldpc240_101::fst4_osd_diag_force_old;
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 100),
        ("m22", 100),
        ("m23", 100),
        ("m24", 100),
        ("m25", 100),
        ("m26", 100),
        ("m27", 100),
        ("m28", 100),
        ("m29", 100),
        ("m30", 100),
    ];
    fst4_osd_diag_force_old(true);
    recall_tradeoff_for_channel("ccir_moderate", SNR_TAGS);
    fst4_osd_diag_force_old(false);
}

/// Shared body for [`fst4_60_diag_recall_tradeoff`] /
/// [`fst4_60_diag_recall_tradeoff_ccir_moderate`] — everything except
/// the channel name is identical.
fn recall_tradeoff_for_channel(channel: &str, snr_tags: &'static [(&'static str, u32)]) {
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
    // Dense near the known ~-27.6 dB FST4-60 AWGN crossing, plus one
    // point well above it as a sanity ceiling (all four configs should
    // read ~20/20 there). m26/m27 use the wider 100-trial corpus
    // generated for VK3NV's statistical-power follow-up (the original
    // 7/20 vs 10/20 `no8_osd`-vs-`bp_only` read at -27 dB was "pretty
    // under-powered," per VK3NV's own comment) — trials 1-20 are
    // byte-identical to the original 20-trial corpus (`fst4sim` is
    // deterministic per trial index), so this only adds statistical
    // power, it doesn't change what was already measured.
    let dir = sweep_dir();
    let (osd_attempt_min, osd_depth3_min) = mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    #[derive(Default, Clone, Copy)]
    struct Tally {
        full: u32,
        bp_only: u32,
        no8_osd: u32,
        no8_no_osd: u32,
        total: u32,
    }

    let mut work: Vec<(&str, u32)> = Vec::new();
    for &(snr_tag, trials) in snr_tags {
        for trial in 1..=trials {
            work.push((snr_tag, trial));
        }
    }

    let process_one = |&(snr_tag, trial): &(&str, u32)| -> (usize, u32, bool, bool, bool, bool) {
        let snr_idx = snr_tags.iter().position(|&(t, _)| t == snr_tag).unwrap();
        let path = dir.join(format!("fst4_60_{channel}_{snr_tag}_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            return (snr_idx, trial, false, false, false, false);
        };
        // sync_min=0.8, matching production's own `DecodeRequest::new`
        // call (`decode_wav_fst4` above) — `fst4_60_diag_osd_escalation`'s
        // 1.0 was tuned against the strong real-world golden WAV only,
        // untested against weak AWGN-sweep candidates.
        let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.8, None, 50);
        let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
        let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
        let fec = <Fst4s60 as Protocol>::Fec::default();
        let verify_info =
            Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

        let mut ok_full = false;
        let mut ok_bp_only = false;
        let mut ok_no8_osd = false;
        let mut ok_no8_no_osd = false;

        for c in &cands {
            // Freq-only pre-filter, matching both `fst4_60_diag_
            // osd_escalation` and `_osd_depth34_nsync_floor` above — no
            // dt filter at the coarse-candidate stage in either
            // reference.
            if (c.freq_hz - GOLDEN_FREQ_HZ).abs() > FREQ_TOL_HZ {
                continue;
            }
            let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
            // RMS-normalise, matching `refine_candidate_position_impl`
            // (`engine::pipeline`) exactly — omitted from both
            // `fst4_60_diag_osd_escalation`/`_osd_depth34_nsync_floor`
            // above (neither noticed: the first never checks the
            // decoded message at all, the second only ran against the
            // strong real golden WAV, where an unnormalised scale
            // doesn't visibly break sync/BP). Missing it here read as
            // 0/20 recall at every SNR, including m20 — confirming the
            // step is load-bearing, not cosmetic, for `fst4_sync_search`
            // and everything downstream of it on real (non-strong)
            // signals.
            let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for z in cd0.iter_mut() {
                    *z *= inv;
                }
            }
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            let df_hz = s2.freq_hz - c.freq_hz;
            let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
            let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
            let nsync = sync_quality::<Fst4s60>(&cs);

            let llr_set = compute_llr::<Fst4s60, f32>(&cs);
            let variants_full: Vec<&Vec<f32>> = [
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
            let variants_no8: Vec<&Vec<f32>> = [
                Some(&llr_set.llra),
                Some(&llr_set.llrb),
                if llr_set.llre.is_empty() {
                    None
                } else {
                    Some(&llr_set.llre)
                },
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
            // FST4 defines `INFO_SCRAMBLE_RVEC` (`fst4::mod::FST4_RVEC`)
            // -- `decode_soft`'s CRC check passes on the still-scrambled
            // bits (CRC is computed the same way on the TX side, over
            // scrambled bits), but `unpack77` expects the raw 77-bit
            // payload. Production's `process_candidate_basic_impl`
            // calls `engine::llr::descramble_info::<P>(&mut r.info)`
            // after every successful `decode_soft`, before ever reading
            // `r.info` as a message -- omitted here first, which
            // produced consistent, perfect-`nsync`, CRC-valid-but-wrong
            // decodes (e.g. "CQ 2Q299KDY0GK" for every LLR variant, same
            // candidate) that made this diagnostic read as 0/20 recall
            // even at m20, where production gets 20/20. Confirms the
            // scramble is load-bearing before unpack77, not just a
            // display nicety.
            let is_golden = |llr: &Vec<f32>, opts: &FecOpts| -> bool {
                fec.decode_soft(llr, opts).is_some_and(|mut r| {
                    mfsk_core::engine::llr::descramble_info::<Fst4s60>(&mut r.info);
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(&r.info[..77]);
                    unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                })
            };

            let bp_full_ok = variants_full.iter().any(|llr| is_golden(llr, &bp_opts));
            let bp_no8_ok = variants_no8.iter().any(|llr| is_golden(llr, &bp_opts));
            if bp_full_ok {
                ok_full = true;
                ok_bp_only = true;
            }
            if bp_no8_ok {
                ok_no8_osd = true;
                ok_no8_no_osd = true;
            }

            // OSD can only add successes, never remove one (production's
            // own dispatch shape) -- only spend it where the matching
            // BP attempt already failed.
            if !bp_full_ok && nsync >= osd_attempt_min {
                let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
                let osd_opts = FecOpts {
                    bp_max_iter: BP_MAX_ITER,
                    osd_depth,
                    ap_mask: None,
                    verify_info,
                    ..FecOpts::default()
                };
                if variants_full.iter().any(|llr| is_golden(llr, &osd_opts)) {
                    ok_full = true;
                }
            }
            if !bp_no8_ok && nsync >= osd_attempt_min {
                let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
                let osd_opts = FecOpts {
                    bp_max_iter: BP_MAX_ITER,
                    osd_depth,
                    ap_mask: None,
                    verify_info,
                    ..FecOpts::default()
                };
                if variants_no8.iter().any(|llr| is_golden(llr, &osd_opts)) {
                    ok_no8_osd = true;
                }
            }
        }

        (snr_idx, trial, ok_full, ok_bp_only, ok_no8_osd, ok_no8_no_osd)
    };

    #[cfg(feature = "parallel")]
    let results: Vec<(usize, u32, bool, bool, bool, bool)> =
        work.par_iter().map(process_one).collect();
    #[cfg(not(feature = "parallel"))]
    let results: Vec<(usize, u32, bool, bool, bool, bool)> =
        work.iter().map(process_one).collect();

    // Sanity: `full ⊇ no8_osd ⊇ no8_no_osd` and `full ⊇ bp_only` must hold
    // per-*trial*, not just in the aggregate counts — a bookkeeping bug
    // could easily preserve monotonic totals while crossing individual
    // trials' flags. `no8_osd` vs `bp_only` has NO such guarantee either
    // direction (different variant sets *and* different OSD on/off, not
    // a superset relationship) -- reported, not asserted.
    let mut violations = 0u32;
    let mut no8_osd_beats_bp_only = 0u32;
    let mut bp_only_beats_no8_osd = 0u32;
    for &(idx, trial, full, bp_only, no8_osd, no8_no_osd) in &results {
        let tag = snr_tags[idx].0;
        if !full && (bp_only || no8_osd || no8_no_osd) {
            eprintln!("MONOTONICITY VIOLATION {tag}_{trial:02}: full=false but a reduced config succeeded");
            violations += 1;
        }
        if bp_only && !full {
            eprintln!("MONOTONICITY VIOLATION {tag}_{trial:02}: bp_only=true but full=false");
            violations += 1;
        }
        if no8_osd && !full {
            eprintln!("MONOTONICITY VIOLATION {tag}_{trial:02}: no8_osd=true but full=false");
            violations += 1;
        }
        if no8_no_osd && !no8_osd {
            eprintln!("MONOTONICITY VIOLATION {tag}_{trial:02}: no8_no_osd=true but no8_osd=false");
            violations += 1;
        }
        if no8_osd && !bp_only {
            no8_osd_beats_bp_only += 1;
        }
        if bp_only && !no8_osd {
            bp_only_beats_no8_osd += 1;
        }
    }
    eprintln!(
        "sanity: {violations} monotonicity violations (must be 0). \
         no8_osd vs bp_only have no superset relationship either \
         direction (different variant sets *and* different OSD on/off): \
         no8_osd-only wins {no8_osd_beats_bp_only} trials, \
         bp_only-only wins {bp_only_beats_no8_osd} trials — both nonzero \
         confirms they're catching genuinely different candidates, not \
         one dominating the other."
    );

    let mut tallies = vec![Tally::default(); snr_tags.len()];
    for (idx, _trial, full, bp_only, no8_osd, no8_no_osd) in results {
        let t = &mut tallies[idx];
        t.total += 1;
        t.full += full as u32;
        t.bp_only += bp_only as u32;
        t.no8_osd += no8_osd as u32;
        t.no8_no_osd += no8_no_osd as u32;
    }

    eprintln!("FST4-60 {channel} recall trade-off (n=20/SNR, n=100 at m26/m27):");
    eprintln!(
        "{:>6} {:>6} {:>8} {:>8} {:>10} {:>4}",
        "SNR", "full", "bp_only", "no8+osd", "no8_noosd", "n"
    );
    for (&(tag, _), t) in snr_tags.iter().zip(&tallies) {
        eprintln!(
            "{:>6} {:>6} {:>8} {:>8} {:>10} {:>4}",
            tag, t.full, t.bp_only, t.no8_osd, t.no8_no_osd, t.total
        );
    }
}

/// VK3NV's follow-up (issue #306): rather than the binary "nsym=8 or
/// nsym≤4" choice `fst4_60_diag_recall_tradeoff` above tested, is
/// there a better middle ground? `compute_llr_partial` already accepts
/// any `nsym` — `LLR_NSYM_MAX=8`/`LLR_NSYM_MID=4` are `Fst4s60`'s own
/// protocol consts, not a limit on the underlying LLR machinery — so
/// `nsym=5`/`6` (both divide FST4-60's 120 data symbols cleanly,
/// unlike odd depths in general) are free to test as a pure
/// diagnostic, no production code change. VK3NV's own visit-count
/// derivation: `(120/n groups) × (2n bit positions) × 4^n hypotheses =
/// 240 × 4^n` — nsym=8: 15.73M, nsym=6: 983k (16× less), nsym=5: 246k
/// (64× less), nsym=4: 61.4k (256× less).
///
/// Also answers VK3NV's statistical-power concern on `no8_osd` vs
/// `bp_only`'s -26/-27 dB asymmetry in the previous diagnostic (20
/// trials/SNR was "pretty under-powered" for a 7/20 vs 10/20 read) —
/// this sweep runs those two SNRs against a wider 100-trial corpus
/// generated for this follow-up
/// (`embedded-poc/assets/fst4_sweep/fst4_60_awgn_m26/m27_*.wav`,
/// trials 21-100 added; trials 1-20 confirmed byte-identical to the
/// original 20-trial corpus before generating more — `fst4sim` is
/// deterministic per trial index, so this extends rather than
/// replaces the existing data). `embedded-poc/assets/fst4_sweep/` is
/// gitignored (regenerated locally, not checked in — same as the rest
/// of the sweep corpora); reproduce the extra 80 trials/SNR with:
/// ```sh
/// for snr in -26 -27; do
///   tag=$(printf "m%02d" $(( -snr )))
///   tmpd=$(mktemp -d)
///   (cd "$tmpd" && /path/to/target/fst4sim/fst4sim \
///     "CQ JL1NIE PM95" 60 1500 0.0 0.0 0.0 100 "$snr" F >/dev/null)
///   for T in $(seq 1 100); do
///     src="$tmpd/000000_$(printf '%04d' $T).wav"
///     dest="embedded-poc/assets/fst4_sweep/fst4_60_awgn_${tag}_$(printf '%02d' $T).wav"
///     [[ -f "$src" && ! -f "$dest" ]] && cp "$src" "$dest"
///   done
///   rm -rf "$tmpd"
/// done
/// ```
///
/// Each `cap` in `{4, 5, 6, 8}` is tested as `{nsym=1, nsym=2,
/// nsym=cap, normalised nsym=1} + OSD` — the same shape as `full`
/// (cap=8) and `no8_osd` (cap=4) in the diagnostic above, generalised
/// to the two new intermediate depths. Unlike that diagnostic's
/// `full`/`no8_osd` pair (which only ever *added* a variant, so
/// `full ⊇ no8_osd` held by construction), different `cap`s here
/// *swap* which deep rung is tried — there is no subset relationship
/// between them, so no monotonicity is asserted or expected across
/// `cap` values (only across the same `cap`'s BP-then-OSD escalation,
/// which mirrors the earlier diagnostic's already-verified shape).
///
/// AWGN here; see [`fst4_60_diag_nsym_depth_sweep_ccir_moderate`] for the
/// CCIR-fading confirmation pass VK3NV's own caution recommended
/// before calling any AWGN-derived depth an embedded profile —
/// FST4-60's 8-symbol coherent integration spans ~2.6 s, so "deeper
/// correlation" isn't necessarily equivalent to "more AWGN SNR" once
/// channel variation over that interval matters.
#[test]
#[ignore = "manual diagnostic — FST4-60 intermediate nsym depths (issue #306 follow-up, VK3NV)"]
fn fst4_60_diag_nsym_depth_sweep() {
    // m26/m27 use the wider 100-trial corpus generated for this
    // follow-up; the rest keep the original 20.
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 20),
        ("m22", 20),
        ("m23", 20),
        ("m24", 20),
        ("m25", 20),
        ("m26", 100),
        ("m27", 100),
        ("m28", 20),
        ("m29", 20),
        ("m30", 20),
    ];
    nsym_depth_sweep_for_channel("awgn", SNR_TAGS);
}

/// Same sweep, CCIR-moderate fading channel — VK3NV's own caution on
/// the AWGN-only version above: FST4-60's 8-symbol coherent
/// integration spans ~2.6 s, so "deeper correlation" isn't necessarily
/// equivalent to "more AWGN SNR" once channel variation over that
/// interval matters; recommended confirming any promising AWGN depth
/// against at least the existing moderate CCIR fading corpus before
/// calling it an embedded profile. Same 20 trials/SNR as the standard
/// corpus (`scripts/gen_fst4_sweep_wavs.sh`'s default) — not widened
/// like AWGN's m26/m27, since this run's purpose is checking whether
/// the AWGN-derived ranking/shape survives fading at all, not
/// re-deriving a precise crossing point under fading.
#[test]
#[ignore = "manual diagnostic — FST4-60 intermediate nsym depths, CCIR-moderate fading (issue #306 follow-up, VK3NV)"]
fn fst4_60_diag_nsym_depth_sweep_ccir_moderate() {
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 20),
        ("m22", 20),
        ("m23", 20),
        ("m24", 20),
        ("m25", 20),
        ("m26", 20),
        ("m27", 20),
        ("m28", 20),
        ("m29", 20),
        ("m30", 20),
    ];
    nsym_depth_sweep_for_channel("ccir_moderate", SNR_TAGS);
}

/// Recall verification for issue #306/#198's `osd_decode_npre_generic`
/// port: FST4's OSD dispatch (`fec/ldpc240_101/mod.rs::fst4_osd_decode`)
/// now runs WSJT-X-faithful `npre1`/`npre1+npre2` (ntheta=12, ntau=14)
/// instead of `osd_decode_generic`'s unpruned k1/k2/k3 combinatorial
/// search for ndeep=2/3 — a genuinely different search strategy, so
/// unlike the earlier bit-packing/verify-gate-reorder follow-ups in this
/// file, this one is **not** bit-exact with what it replaces and needs
/// its own recall check rather than a differential test.
///
/// Runs the full production path (`DecodeRequest::<Fst4s60>::decode()`,
/// same entry point `decode_wav_fst4_60` above uses) against the m26/m27
/// 100-trial AWGN corpus generated for the nsym-depth-sweep follow-up —
/// the same corpus, same crossing-adjacent SNRs, as the pre-port
/// baseline recorded in this investigation's #306 comments:
///
/// | SNR | pre-port (old combinatorial OSD) | n |
/// |---|---:|---:|
/// | -26 dB | 97% | 100 |
/// | -27 dB | 77% | 100 |
///
/// No hard assertion (this is the crossing region — a few trials'
/// difference either way is within the sampling noise this
/// investigation already characterised, see the statistical-power
/// follow-up above) — printed for direct comparison against the table.
/// A large regression (not a couple of trials) would indicate the port
/// dropped candidates the unpruned search still reaches; a match or
/// improvement is the expected outcome given `npre1_pattern_counts`'
/// measured ~32× reduction in unpruned search *volume* — the `ntheta`/
/// `ntau` gates are WSJT-X's own tuned thresholds, not a truncation of
/// mfsk-core's own search depth.
#[test]
#[ignore = "manual verification — FST4-60 npre1/npre2 OSD port recall check (issue #306/#198, VK3NV)"]
fn fst4_60_diag_npre_osd_recall_verify() {
    let dir = sweep_dir();
    const SNR_TAGS: &[(&str, u32)] = &[("m26", 100), ("m27", 100)];
    for &(snr_tag, trials) in SNR_TAGS {
        let mut pass = 0u32;
        let mut total = 0u32;
        for trial in 1..=trials {
            let path = dir.join(format!("fst4_60_awgn_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            total += 1;
            if decode_wav_fst4_60(&audio) {
                pass += 1;
            }
        }
        println!("{snr_tag}: {pass}/{total} ({:.0}%)", 100.0 * pass as f64 / total.max(1) as f64);
    }
}

/// Bug-hunt for [`fst4_60_diag_npre_osd_recall_verify`]'s m27 gap
/// (97→96 at m26, 77→70 at m27 — noise-plausible at m26, not at m27):
/// find concrete trials where the old unpruned combinatorial OSD search
/// decodes but the new npre1/npre2 port doesn't, by running the *exact*
/// production entry point (`decode_wav_fst4_60`) twice per trial via
/// [`mfsk_core::fec::Ldpc240_101::fst4_osd_diag_force_old`] — once as
/// shipped (npre1/npre2), once forced back to the old search — rather
/// than reimplementing `decode_soft`'s BP/zsum control flow separately.
///
/// `#[test]` (not `--ignored`) would need the corpus present; kept
/// `#[ignore]` like every other corpus-dependent diagnostic here.
/// Single-threaded by construction (`fst4_osd_diag_force_old` is a
/// process-global toggle) — this function alone in the process is fine,
/// but don't run it concurrently with another FST4-decoding test.
#[test]
#[ignore = "manual bug hunt — FST4-60 npre1/npre2 OSD port m27 recall gap (issue #306/#198)"]
fn fst4_60_diag_npre_osd_bug_hunt() {
    use mfsk_core::fec::ldpc240_101::fst4_osd_diag_force_old;

    let dir = sweep_dir();
    const SNR_TAGS: &[(&str, u32)] = &[("m26", 100), ("m27", 100)];
    let mut old_only: Vec<(&str, u32)> = Vec::new();
    let mut new_only: Vec<(&str, u32)> = Vec::new();
    let mut both = 0u32;
    let mut neither = 0u32;

    for &(snr_tag, trials) in SNR_TAGS {
        for trial in 1..=trials {
            let path = dir.join(format!("fst4_60_awgn_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            fst4_osd_diag_force_old(false);
            let ok_new = decode_wav_fst4_60(&audio);
            fst4_osd_diag_force_old(true);
            let ok_old = decode_wav_fst4_60(&audio);
            fst4_osd_diag_force_old(false); // restore default for anything else in-process

            match (ok_new, ok_old) {
                (true, true) => both += 1,
                (false, false) => neither += 1,
                (false, true) => old_only.push((snr_tag, trial)),
                (true, false) => new_only.push((snr_tag, trial)),
            }
        }
    }

    println!("both: {both}, neither: {neither}");
    println!("old-only ({} trials): {:?}", old_only.len(), old_only);
    println!("new-only ({} trials): {:?}", new_only.len(), new_only);
}

/// Digs into one concrete "old succeeds, new fails" trial from
/// [`fst4_60_diag_npre_osd_bug_hunt_ccir_moderate`] (CCIR-moderate m26,
/// trial 2) to characterise *why*: does the old combinatorial search's
/// success come from an order (1/2) `npre1` should also reach (a real
/// port bug) or genuinely from order-3 (WSJT-X's own npre1+npre2
/// architecture's known limitation — the `npre2` hash table only
/// surfaces weight-3 patterns whose partial-parity signature collides
/// with another MRB pair within the `ntau`-bit window, not every
/// possible weight-3 combination the way an unpruned search does)?
#[test]
#[ignore = "manual root-cause probe — one CCIR-moderate old-only trial (issue #306/#198)"]
fn fst4_60_diag_npre_osd_ccir_trial_probe() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::fec::ldpc::osd::{osd_decode_generic, osd_decode_npre_generic};
    use mfsk_core::fec::ldpc::params::Ldpc240_101Params;
    use mfsk_core::engine::{MessageCodec, Protocol};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    let dir = sweep_dir();
    let path = dir.join("fst4_60_ccir_moderate_m26_02.wav");
    let audio = load_wav_i16_opt(&path).expect("trial 2 must exist");

    let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.8, None, 50);
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
    let verify_info =
        Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

    for c in cands.iter().filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ) {
        let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
        let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
        if sum2 > f32::EPSILON {
            let inv = 1.0 / sum2.sqrt();
            for z in cd0.iter_mut() {
                *z *= inv;
            }
        }
        let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
        let df_hz = s2.freq_hz - c.freq_hz;
        let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
        let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
        let nsync = sync_quality::<Fst4s60>(&cs);
        let llr_set = compute_llr::<Fst4s60, f32>(&cs);

        for (label, llr) in [
            ("llra", &llr_set.llra),
            ("llrb", &llr_set.llrb),
            ("llrc", &llr_set.llrc),
            ("llrd", &llr_set.llrd),
        ] {
            let ord1 = osd_decode_generic::<Ldpc240_101Params>(llr, 1, 101, verify_info, false);
            let ord2 = osd_decode_generic::<Ldpc240_101Params>(llr, 2, 101, verify_info, false);
            let ord3 = osd_decode_generic::<Ldpc240_101Params>(llr, 3, 101, verify_info, false);
            let npre1 = osd_decode_npre_generic::<Ldpc240_101Params>(llr, 12, 0, false, verify_info);
            let npre12 =
                osd_decode_npre_generic::<Ldpc240_101Params>(llr, 12, 14, true, verify_info);
            println!(
                "freq={:.1} nsync={nsync} variant={label}: ord1={} ord2={} ord3={} npre1={} npre1+2={}",
                c.freq_hz,
                ord1.is_some(),
                ord2.is_some(),
                ord3.is_some(),
                npre1.is_some(),
                npre12.is_some(),
            );
            if let Some(r) = &ord3 {
                println!("  ord3 hard_errors={}", r.hard_errors);
            }
        }
    }
}

/// Root-cause bug hunt for the CCIR-moderate recall gap
/// [`fst4_60_diag_recall_tradeoff_ccir_moderate`] found (m26:
/// 24/100→18/100, ~25% relative) — same shape as
/// [`fst4_60_diag_npre_osd_bug_hunt`] but through the true production
/// entry point (`decode_wav_fst4_60`, not `recall_tradeoff_for_channel`'s
/// own hand-rolled pipeline) on the m26 CCIR-moderate corpus, to confirm
/// the gap isn't an artifact of that diagnostic's own candidate-
/// generation path (which has previously had real, independent bugs —
/// see `fst4_60_diag_recall_tradeoff`'s doc comments on `sync_min` and
/// `descramble_info`).
#[test]
#[ignore = "manual bug hunt — FST4-60 npre1/npre2 OSD port CCIR-moderate recall gap (issue #306/#198)"]
fn fst4_60_diag_npre_osd_bug_hunt_ccir_moderate() {
    use mfsk_core::fec::ldpc240_101::fst4_osd_diag_force_old;

    let dir = sweep_dir();
    let mut old_only: Vec<u32> = Vec::new();
    let mut new_only: Vec<u32> = Vec::new();
    let mut both = 0u32;
    let mut neither = 0u32;

    for trial in 1..=100u32 {
        let path = dir.join(format!("fst4_60_ccir_moderate_m26_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            continue;
        };
        fst4_osd_diag_force_old(false);
        let ok_new = decode_wav_fst4_60(&audio);
        fst4_osd_diag_force_old(true);
        let ok_old = decode_wav_fst4_60(&audio);
        fst4_osd_diag_force_old(false);

        match (ok_new, ok_old) {
            (true, true) => both += 1,
            (false, false) => neither += 1,
            (false, true) => old_only.push(trial),
            (true, false) => new_only.push(trial),
        }
    }

    println!("both: {both}, neither: {neither}");
    println!("old-only ({} trials): {:?}", old_only.len(), old_only);
    println!("new-only ({} trials): {:?}", new_only.len(), new_only);
}

/// Shared body for [`fst4_60_diag_nsym_depth_sweep`] /
/// [`fst4_60_diag_nsym_depth_sweep_ccir_moderate`] — everything except
/// the channel name and the (SNR tag, trial count) grid is identical.
fn nsym_depth_sweep_for_channel(channel: &str, snr_tags: &[(&str, u32)]) {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr_fast, compute_llr_partial, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    const BP_MAX_ITER: u32 = 30;
    // VK3NV's item 2: "caps something like 1/2/4/6/8". `llra`/`llrb`
    // (nsym=1/2) and `llrd` (normalised nsym=1) are already tried as
    // fixed baseline variants alongside whichever `cap` swaps the deep
    // rung — cap=1/2 here just make the swapped-in deep rung redundant
    // with an existing baseline variant, which is exactly "no rung
    // beyond nsym=1/2" the way cap=4/5/6/8 already meant "no rung beyond
    // nsym=cap". No special-casing needed, same shape throughout.
    const CAPS: [usize; 5] = [1, 2, 4, 6, 8];

    let dir = sweep_dir();
    let (osd_attempt_min, osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    let mut work: Vec<(&str, u32)> = Vec::new();
    for &(snr_tag, trials) in snr_tags {
        for trial in 1..=trials {
            work.push((snr_tag, trial));
        }
    }

    // 0 = miss, 1 = correct (golden message), 2 = false positive
    // (CRC-valid but a different message) -- VK3NV's item 2 ask: for a
    // monitoring application, a wrong-but-valid decode is worse than a
    // missed marginal one, so it's tracked separately rather than
    // folded into "miss" the way a plain recall sweep would.
    const MISS: u8 = 0;
    const CORRECT: u8 = 1;
    const FALSE_POSITIVE: u8 = 2;

    // Returns (snr_idx, per-cap outcome in CAPS order).
    let process_one = |&(snr_tag, trial): &(&str, u32)| -> (usize, [u8; 5]) {
        let snr_idx = snr_tags.iter().position(|&(t, _)| t == snr_tag).unwrap();
        let path = dir.join(format!("fst4_60_{channel}_{snr_tag}_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            return (snr_idx, [MISS; 5]);
        };
        let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.8, None, 50);
        let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
        let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
        let fec = <Fst4s60 as Protocol>::Fec::default();
        let verify_info =
            Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

        let mut outcome = [MISS; 5];

        for c in cands.iter().filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ) {
            let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
            let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for z in cd0.iter_mut() {
                    *z *= inv;
                }
            }
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            let df_hz = s2.freq_hz - c.freq_hz;
            let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
            let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
            let nsync = sync_quality::<Fst4s60>(&cs);

            let llr_set = compute_llr_fast::<Fst4s60, f32>(&cs); // nsym=1 -> llra, llrd
            let llrb = compute_llr_partial::<Fst4s60, f32, f32>(&cs, 2);

            let bp_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 0,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            // Returns MISS / CORRECT / FALSE_POSITIVE for this one
            // decode attempt (CRC failure vs. wrong-message-but-CRC-
            // valid vs. the golden message).
            let classify = |llr: &Vec<f32>, opts: &FecOpts| -> u8 {
                fec.decode_soft(llr, opts).map_or(MISS, |mut r| {
                    mfsk_core::engine::llr::descramble_info::<Fst4s60>(&mut r.info);
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(&r.info[..77]);
                    if unpack77(&m77).as_deref() == Some(GOLDEN_MSG) {
                        CORRECT
                    } else {
                        FALSE_POSITIVE
                    }
                })
            };
            // Best outcome across a variant set: CORRECT beats
            // FALSE_POSITIVE beats MISS -- if any variant lands the
            // golden message that's the outcome to report, even if a
            // different variant landed a false positive first.
            let best = |variants: &[&Vec<f32>], opts: &FecOpts| -> u8 {
                variants.iter().map(|llr| classify(llr, opts)).max().unwrap_or(MISS)
            };

            for (i, &cap) in CAPS.iter().enumerate() {
                if outcome[i] == CORRECT {
                    continue; // already golden via an earlier candidate this file
                }
                let llr_cap = compute_llr_partial::<Fst4s60, f32, f32>(&cs, cap);
                let variants: [&Vec<f32>; 4] = [&llr_set.llra, &llrb, &llr_cap, &llr_set.llrd];
                let bp_result = best(&variants, &bp_opts);
                if bp_result != MISS {
                    outcome[i] = outcome[i].max(bp_result);
                    if outcome[i] == CORRECT {
                        continue;
                    }
                }
                if nsync >= osd_attempt_min {
                    let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
                    let osd_opts = FecOpts {
                        bp_max_iter: BP_MAX_ITER,
                        osd_depth,
                        ap_mask: None,
                        verify_info,
                        ..FecOpts::default()
                    };
                    let osd_result = best(&variants, &osd_opts);
                    outcome[i] = outcome[i].max(osd_result);
                }
            }
        }

        (snr_idx, outcome)
    };

    #[cfg(feature = "parallel")]
    let results: Vec<(usize, [u8; 5])> = work.par_iter().map(process_one).collect();
    #[cfg(not(feature = "parallel"))]
    let results: Vec<(usize, [u8; 5])> = work.iter().map(process_one).collect();

    #[derive(Default, Clone, Copy)]
    struct Tally {
        correct: [u32; 5],
        false_pos: [u32; 5],
        total: u32,
    }
    let mut tallies = vec![Tally::default(); snr_tags.len()];
    for (idx, outcome) in &results {
        let t = &mut tallies[*idx];
        t.total += 1;
        for (i, &o) in outcome.iter().enumerate() {
            match o {
                CORRECT => t.correct[i] += 1,
                FALSE_POSITIVE => t.false_pos[i] += 1,
                _ => {}
            }
        }
    }

    eprintln!("FST4-60 {channel} nsym-depth sweep (cap in {{1,2,4,6,8}}, OSD on for all) -- correct/false-positive:");
    eprintln!(
        "{:>6} {:>10} {:>10} {:>10} {:>10} {:>10} {:>5}",
        "SNR", "cap=1", "cap=2", "cap=4", "cap=6", "cap=8", "n"
    );
    for (&(tag, _), t) in snr_tags.iter().zip(&tallies) {
        eprintln!(
            "{:>6} {:>10} {:>10} {:>10} {:>10} {:>10} {:>5}",
            tag,
            format!("{}/{}", t.correct[0], t.false_pos[0]),
            format!("{}/{}", t.correct[1], t.false_pos[1]),
            format!("{}/{}", t.correct[2], t.false_pos[2]),
            format!("{}/{}", t.correct[3], t.false_pos[3]),
            format!("{}/{}", t.correct[4], t.false_pos[4]),
            t.total
        );
    }
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

/// VK3NV's item 2, ccir_good: coarse recon (n=20/SNR) found its
/// 50%-crossing sits around m25-m26 (cap=8: 15/20, 7/20) — one SNR
/// step milder than ccir_moderate's, as expected for a gentler fading
/// profile. Widened to n=100 at m25/m26 for the depth/cap comparison
/// (`fst4sim` determinism confirmed as usual before extending).
#[test]
#[ignore = "manual diagnostic — FST4-60 nsym-depth + false-positive sweep, ccir_good near crossing (issue #306 item 2)"]
fn fst4_60_diag_nsym_depth_sweep_ccir_good() {
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 20),
        ("m23", 20),
        ("m24", 20),
        ("m25", 100),
        ("m26", 100),
        ("m27", 20),
        ("m28", 20),
    ];
    nsym_depth_sweep_for_channel("ccir_good", SNR_TAGS);
}

/// VK3NV's item 2, ccir_poor: coarse recon found its 50%-crossing sits
/// around m24-m25 (cap=8: 12/20, 3/20) — one SNR step harsher than
/// ccir_moderate's. Widened to n=100 at m24/m25.
#[test]
#[ignore = "manual diagnostic — FST4-60 nsym-depth + false-positive sweep, ccir_poor near crossing (issue #306 item 2)"]
fn fst4_60_diag_nsym_depth_sweep_ccir_poor() {
    const SNR_TAGS: &[(&str, u32)] = &[
        ("m20", 20),
        ("m22", 20),
        ("m23", 20),
        ("m24", 100),
        ("m25", 100),
        ("m26", 20),
        ("m27", 20),
    ];
    nsym_depth_sweep_for_channel("ccir_poor", SNR_TAGS);
}

/// VK3NV's item 2: "ideally some noise-only trials" — for WhisperQSO,
/// a CRC-valid-but-wrong decode is worse than a missed marginal one, so
/// the false-positive *rate itself* (not just its presence relative to
/// correct decodes) matters on its own. `fst4sim` doesn't have a
/// dedicated "no signal" mode; `-60 dB` (100 trials) is used as a
/// practical proxy — 60 dB below the AWGN floor is indistinguishable
/// from pure noise for anything this decoder's LLR/BP/OSD stages can
/// resolve (the signal is still nominally present in the generated
/// WAV, but no test elsewhere in this corpus, even down to m30/m37,
/// gets anywhere close to reading it out). Uses
/// [`nsym_depth_sweep_for_channel`] unchanged — its "correct" column
/// should read ~0 here by construction (the signal is unrecoverable);
/// the number to actually look at is "false_pos", broken out per `cap`
/// exactly like the fading sweeps above, at n=100.
#[test]
#[ignore = "manual diagnostic — FST4-60 false-positive rate on noise-only trials (issue #306 item 2, VK3NV)"]
fn fst4_60_diag_false_positive_rate_noise_only() {
    const SNR_TAGS: &[(&str, u32)] = &[("m60", 100)];
    nsym_depth_sweep_for_channel("noise", SNR_TAGS);
}

/// VK3NV's item 3: rung-major (breadth-first-by-depth) scheduling —
/// "running nsym=1 across all candidates before any single one gets to
/// nsym=2/4/8 turns the decoder into an anytime process." Two-step plan
/// (issue #306): this is step 1+2, host-only — verify reordering
/// doesn't change *what* gets decoded, then project the "anytime"
/// benefit using host-measured per-rung costs scaled to the real
/// embedded totals already measured this session, before committing to
/// an actual `process_candidate_basic_impl` control-flow rewrite (step 3,
/// not started).
///
/// Uses the real off-air FST4-60 golden WAV (`210115_0058.wav`, the
/// same recording `fst4_bench`'s 41 baked candidates come from) rather
/// than synthetic AWGN — the point here is candidate *order*, and only
/// a real multi-signal recording has a realistic distribution of "hard
/// candidate at low index, easy real signal at high index" the way
/// `fst4_bench`'s own logs showed (both real decodes are candidates
/// #37/#38 of ~41, i.e. *last*).
///
/// Depth-first (current production order): for each candidate in
/// coarse_sync's order, try nsym=1, then 2, then `LLR_NSYM_MID`, then
/// `LLR_NSYM_MAX`+OSD, stopping at the first success — a candidate that
/// never decodes pays for every rung before the loop moves on.
///
/// Rung-major: for each rung in the same 1→2→mid→max+OSD order, try
/// that rung across *every still-undecided* candidate before moving to
/// the next rung.
///
/// Host `Instant` timing isn't embedded-calibrated in absolute terms
/// (different CPU entirely), so this projects rather than measures: the
/// host-measured "rung 1+2+mid+max" total (no OSD, i.e. depth-first's
/// `BP_ONLY` shape) is scaled by a single factor so it matches this
/// session's real hardware `BP_ONLY` figure (31.023 s, `docs/reference/
/// EMBEDDED.md`'s Sixteenth attempt) — then that same factor is applied
/// to project both orderings' full cumulative-decode-vs-time curves
/// (OSD included) into embedded-equivalent seconds.
#[test]
#[ignore = "manual diagnostic — rung-major scheduling: correctness + anytime-curve projection (issue #306 item 3, VK3NV)"]
fn fst4_60_diag_rung_major_scheduling() {
    use std::time::{Duration, Instant};

    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr_fast, compute_llr_partial, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    const BP_MAX_ITER: u32 = 30;
    const LLR_NSYM_MID: usize = 4;
    const LLR_NSYM_MAX: usize = 8;
    // Real embedded ground truth, current build (docs/reference/
    // EMBEDDED.md "Sixteenth attempt") -- the calibration target.
    const REAL_BP_ONLY_SECS: f64 = 31.023;

    let Some(path) = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    ) else {
        eprintln!("skip: real FST4-60 golden WAV not vendored/found");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV must load");
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);

    // Replicates `fst4_bake_golden_refined_candidates`'s exact recipe
    // (sync_min=1.2, refine + dedup_refined_candidates' rule, stable
    // index sort) — this is the same 41-candidate set/order
    // `fst4_bench`'s real-hardware logs are keyed against (both real
    // decodes land at candidate #37/#38 of ~41 there). A plain
    // `coarse_sync` call at production's general sync_min=0.8 pulls in
    // far more raw/duplicate candidates on this strong recording and
    // gives a materially different order — not what's being scheduled
    // on real hardware, so not what this diagnostic should model.
    let raw_candidates = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.2, None, 50);
    let refined: Vec<_> = raw_candidates
        .iter()
        .map(|c| mfsk_core::engine::pipeline::refine_candidate_position::<Fst4s60>(c, &fft_cache, &FST4_60A_DOWNSAMPLE))
        .collect();
    let freq_tol = 0.10 * FST4_60A_DOWNSAMPLE.tone_spacing_hz;
    const I0_TOL: i32 = 2;
    let mut order: Vec<usize> = (0..raw_candidates.len()).collect();
    order.sort_by(|&a, &b| {
        refined[b].3.partial_cmp(&refined[a].3).unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut kept_positions: Vec<(f32, i32)> = Vec::new();
    let mut survivors: Vec<usize> = Vec::new();
    for idx in order {
        let (_, f, i0, _) = &refined[idx];
        let dup = kept_positions
            .iter()
            .any(|&(kf, ki)| (f - kf).abs() < freq_tol && (i0 - ki).abs() <= I0_TOL);
        if !dup {
            kept_positions.push((*f, *i0));
            survivors.push(idx);
        }
    }
    survivors.sort_unstable();
    let cands: Vec<_> = survivors.iter().map(|&i| raw_candidates[i].clone()).collect();
    eprintln!(
        "{} raw candidates -> {} refined+deduped (matches fst4_bench's baked asset)",
        raw_candidates.len(),
        cands.len()
    );
    let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
    let fec = <Fst4s60 as Protocol>::Fec::default();
    let verify_info =
        Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);
    let (osd_attempt_min, osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    struct CandTiming {
        idx: usize,
        freq_hz: f32,
        // Per-rung (setup, nsym1, nsym2, mid, max+osd) host durations
        // and whether that rung alone (given all earlier rungs already
        // failed) decodes the golden message.
        t_setup: Duration,
        t: [Duration; 4],
        ok: [bool; 4],
    }

    let mut timings: Vec<CandTiming> = Vec::new();
    for (idx, c) in cands.iter().enumerate() {
        let t_setup_start = Instant::now();
        let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
        let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
        if sum2 > f32::EPSILON {
            let inv = 1.0 / sum2.sqrt();
            for z in cd0.iter_mut() {
                *z *= inv;
            }
        }
        let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
        let df_hz = s2.freq_hz - c.freq_hz;
        let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
        let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
        let nsync = sync_quality::<Fst4s60>(&cs);
        let t_setup = t_setup_start.elapsed();

        let bp_opts = FecOpts {
            bp_max_iter: BP_MAX_ITER,
            osd_depth: 0,
            ap_mask: None,
            verify_info,
            ..FecOpts::default()
        };
        // Unlike the fst4sim-corpus diagnostics elsewhere in this file
        // (single injected `GOLDEN_MSG`), this real off-air recording
        // carries two distinct real QSOs — `fst4_bench`'s own log
        // confirms both ("CQ N5TM EL29" candidate #37, "CQ K9KFR EN71"
        // #38 of ~41 refined candidates).
        const REAL_GOLDEN_MSGS: [&str; 2] = ["CQ N5TM EL29", "CQ K9KFR EN71"];
        let is_golden = |llr: &Vec<f32>, opts: &FecOpts| -> bool {
            fec.decode_soft(llr, opts).is_some_and(|mut r| {
                mfsk_core::engine::llr::descramble_info::<Fst4s60>(&mut r.info);
                let mut m77 = [0u8; 77];
                m77.copy_from_slice(&r.info[..77]);
                unpack77(&m77)
                    .as_deref()
                    .is_some_and(|m| REAL_GOLDEN_MSGS.contains(&m))
            })
        };

        let mut t = [Duration::ZERO; 4];
        let mut ok = [false; 4];

        // Rung 0: nsym=1 (llra) + normalised nsym=1 (llrd).
        let t0 = Instant::now();
        let llr1 = compute_llr_fast::<Fst4s60, f32>(&cs);
        let r0 = is_golden(&llr1.llra, &bp_opts) || is_golden(&llr1.llrd, &bp_opts);
        t[0] = t0.elapsed();
        ok[0] = r0;

        // Rung 1: nsym=2 (llrb).
        let t1 = Instant::now();
        let llrb = compute_llr_partial::<Fst4s60, f32, f32>(&cs, 2);
        let r1 = is_golden(&llrb, &bp_opts);
        t[1] = t1.elapsed();
        ok[1] = r1;

        // Rung 2: nsym=LLR_NSYM_MID.
        let t2 = Instant::now();
        let llr_mid = compute_llr_partial::<Fst4s60, f32, f32>(&cs, LLR_NSYM_MID);
        let r2 = is_golden(&llr_mid, &bp_opts);
        t[2] = t2.elapsed();
        ok[2] = r2;

        // Rung 3: nsym=LLR_NSYM_MAX, then OSD (production npre1/npre2)
        // if BP still hasn't found it and nsync clears the gate.
        let t3 = Instant::now();
        let llr_max = compute_llr_partial::<Fst4s60, f32, f32>(&cs, LLR_NSYM_MAX);
        let mut r3 = is_golden(&llr_max, &bp_opts);
        if !r3 && nsync >= osd_attempt_min {
            let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
            let osd_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            r3 = is_golden(&llr_max, &osd_opts) || is_golden(&llrb, &osd_opts);
        }
        t[3] = t3.elapsed();
        ok[3] = r3;

        timings.push(CandTiming {
            idx,
            freq_hz: c.freq_hz,
            t_setup,
            t,
            ok,
        });
    }

    // ---- Correctness: final decode set must match regardless of order ----
    let depth_first_decoded: std::collections::BTreeSet<usize> = timings
        .iter()
        .filter(|c| c.ok.iter().any(|&o| o))
        .map(|c| c.idx)
        .collect();
    // Rung-major visits the same rungs for the same candidates, just
    // reordered -- the decoded SET is trivially the same set of `ok`
    // flags read a different way, so this is really asserting the
    // harness computed `ok` consistently, not a deep property. Kept as
    // an explicit sanity check anyway.
    let rung_major_decoded: std::collections::BTreeSet<usize> = depth_first_decoded.clone();
    assert_eq!(
        depth_first_decoded, rung_major_decoded,
        "reordering must not change which candidates decode"
    );
    eprintln!(
        "correctness: {} candidates, {} decoded (set identical under both orderings by construction)",
        timings.len(),
        depth_first_decoded.len()
    );

    // ---- Calibration: host BP_ONLY total -> real embedded 31.023 s ----
    let host_bp_only_total: Duration = timings
        .iter()
        .map(|c| c.t_setup + c.t[0] + c.t[1] + c.t[2] + c.t[3])
        .sum();
    let scale = REAL_BP_ONLY_SECS / host_bp_only_total.as_secs_f64();
    eprintln!(
        "calibration: host BP_ONLY-shape total = {:.3}s over {} candidates -> scale factor {:.2}x to match real {:.3}s",
        host_bp_only_total.as_secs_f64(),
        timings.len(),
        scale,
        REAL_BP_ONLY_SECS
    );

    // ---- Depth-first cumulative timeline ----
    eprintln!("\ndepth-first order (current production order):");
    let mut cum = 0.0f64;
    for c in &timings {
        let mut decoded_at: Option<f64> = None;
        cum += c.t_setup.as_secs_f64() * scale;
        for r in 0..4 {
            cum += c.t[r].as_secs_f64() * scale;
            if c.ok[r] {
                decoded_at = Some(cum);
                break;
            }
        }
        if let Some(t) = decoded_at {
            eprintln!(
                "  candidate #{:2} ({:7.1} Hz) decoded at t={:7.3}s (cumulative)",
                c.idx, c.freq_hz, t
            );
        }
    }
    eprintln!("  depth-first total: {cum:.3}s");

    // ---- Depth-first, worst-case candidate order ----
    // The recording's *actual* candidate order happens to put both real
    // decodes first (frequency-ascending happens to favour them here) --
    // that's a property of this one recording, not something a general
    // monitoring receiver can rely on. This computes the same
    // depth-first schedule with the two decoding candidates moved to
    // the *end* of the list instead, as the honest worst case
    // depth-first's own ordering-dependence can produce.
    let mut worst_order: Vec<&CandTiming> = timings.iter().filter(|c| !c.ok.iter().any(|&o| o)).collect();
    worst_order.extend(timings.iter().filter(|c| c.ok.iter().any(|&o| o)));
    eprintln!("\ndepth-first order, worst case (decoding candidates moved last):");
    let mut cum = 0.0f64;
    for c in &worst_order {
        let mut decoded_at: Option<f64> = None;
        cum += c.t_setup.as_secs_f64() * scale;
        for r in 0..4 {
            cum += c.t[r].as_secs_f64() * scale;
            if c.ok[r] {
                decoded_at = Some(cum);
                break;
            }
        }
        if let Some(t) = decoded_at {
            eprintln!(
                "  candidate #{:2} ({:7.1} Hz) decoded at t={:7.3}s (cumulative)",
                c.idx, c.freq_hz, t
            );
        }
    }
    eprintln!("  worst-case depth-first total: {cum:.3}s (same total work, all decodes deferred to the end)");

    // ---- Rung-major cumulative timeline ----
    eprintln!("\nrung-major order (nsym=1 across all, then nsym=2 across all, ...):");
    let mut cum = 0.0f64;
    let mut decided = vec![false; timings.len()];
    // Setup (symbol_spectra etc.) has to happen once per candidate
    // before any rung can run -- charged up front, same as depth-first.
    for c in &timings {
        cum += c.t_setup.as_secs_f64() * scale;
    }
    for r in 0..4 {
        let mut newly_decoded: Vec<(usize, f32)> = Vec::new();
        for (i, c) in timings.iter().enumerate() {
            if decided[i] {
                continue;
            }
            cum += c.t[r].as_secs_f64() * scale;
            if c.ok[r] {
                decided[i] = true;
                newly_decoded.push((c.idx, c.freq_hz));
            }
        }
        for (idx, freq_hz) in newly_decoded {
            eprintln!("  candidate #{idx:2} ({freq_hz:7.1} Hz) decoded at t={cum:7.3}s (cumulative, end of rung {r})");
        }
    }
    eprintln!("  rung-major total: {cum:.3}s (must equal depth-first total -- same total work, different order)");
}

/// Ablation: does `llrd` (normalised nsym=1 — tried *last* in
/// production's BP staircase, and as the last OSD variant too) ever
/// decode something `llra`/`llrb`/`llre`(mid)/`llrc`(max) + OSD on
/// those four wouldn't have found on their own? Prompted directly by
/// issue #306 item 3 (VK3NV, rung-major scheduling): before committing
/// to a "6-stage" rung-major design (BP: llra/llrb/llre/llrc/llrd, then
/// OSD on all 5), check whether the embedded config even needs all six
/// stages — a stage that never contributes shouldn't be scheduled at
/// all, regardless of order.
///
/// Tests all four `(keep llrc, keep llrd)` combinations against the
/// same real FST4-60 AWGN corpus already used for the `no8_osd`
/// trade-off (m26/m27, n=100) so results are directly comparable to
/// that earlier measurement.
#[test]
#[ignore = "manual diagnostic — FST4-60 llrd/llrc stage ablation for rung-major design (issue #306 item 3, VK3NV)"]
fn fst4_60_diag_stage_ablation() {
    stage_ablation_for_channel("awgn");
}

/// Same ablation, CCIR-moderate — `llrc` (nsym=8) is fading-sensitive
/// (this investigation's own earlier finding), so before concluding
/// `llrd` is droppable in general (not just under AWGN), check whether
/// it behaves differently under fading too.
#[test]
#[ignore = "manual diagnostic — FST4-60 llrd/llrc stage ablation, CCIR-moderate (issue #306 item 3, VK3NV)"]
fn fst4_60_diag_stage_ablation_ccir_moderate() {
    stage_ablation_for_channel("ccir_moderate");
}

fn stage_ablation_for_channel(channel: &str) {
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
    const SNR_TAGS: &[(&str, u32)] = &[("m26", 100), ("m27", 100)];
    // (keep_llrc, keep_llrd) -- llra/llrb/llre are always kept, matching
    // every config this investigation has considered so far (nobody
    // proposed dropping the cheap rungs).
    const CONFIGS: [(bool, bool); 4] = [
        (true, true),   // full (production's actual 5-variant set)
        (true, false),  // full minus llrd
        (false, true),  // no8_osd (production's skip_llr_nsym_max shape)
        (false, false), // no8_osd minus llrd
    ];

    let dir = sweep_dir();
    let (osd_attempt_min, osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    let mut work: Vec<(&str, u32)> = Vec::new();
    for &(snr_tag, trials) in SNR_TAGS {
        for trial in 1..=trials {
            work.push((snr_tag, trial));
        }
    }

    let process_one = |&(snr_tag, trial): &(&str, u32)| -> (usize, [bool; 4]) {
        let snr_idx = SNR_TAGS.iter().position(|&(t, _)| t == snr_tag).unwrap();
        let path = dir.join(format!("fst4_60_{channel}_{snr_tag}_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            return (snr_idx, [false; 4]);
        };
        let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.8, None, 50);
        let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
        let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;
        let fec = <Fst4s60 as Protocol>::Fec::default();
        let verify_info =
            Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

        let mut ok = [false; 4];

        for c in cands.iter().filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ) {
            let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
            let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for z in cd0.iter_mut() {
                    *z *= inv;
                }
            }
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            let df_hz = s2.freq_hz - c.freq_hz;
            let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);
            let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
            let nsync = sync_quality::<Fst4s60>(&cs);
            let llr_set = compute_llr::<Fst4s60, f32>(&cs); // llra/llrb/llrc(=nsym8)/llrd/llre(=nsym4)

            let bp_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 0,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let is_golden = |llr: &Vec<f32>, opts: &FecOpts| -> bool {
                fec.decode_soft(llr, opts).is_some_and(|mut r| {
                    mfsk_core::engine::llr::descramble_info::<Fst4s60>(&mut r.info);
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(&r.info[..77]);
                    unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                })
            };

            for (i, &(keep_llrc, keep_llrd)) in CONFIGS.iter().enumerate() {
                if ok[i] {
                    continue;
                }
                let mut variants: Vec<&Vec<f32>> = vec![&llr_set.llra, &llr_set.llrb, &llr_set.llre];
                if keep_llrc {
                    variants.push(&llr_set.llrc);
                }
                if keep_llrd {
                    variants.push(&llr_set.llrd);
                }
                if variants.iter().any(|llr| is_golden(llr, &bp_opts)) {
                    ok[i] = true;
                    continue;
                }
                if nsync >= osd_attempt_min {
                    let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
                    let osd_opts = FecOpts {
                        bp_max_iter: BP_MAX_ITER,
                        osd_depth,
                        ap_mask: None,
                        verify_info,
                        ..FecOpts::default()
                    };
                    if variants.iter().any(|llr| is_golden(llr, &osd_opts)) {
                        ok[i] = true;
                    }
                }
            }
        }

        (snr_idx, ok)
    };

    #[cfg(feature = "parallel")]
    let results: Vec<(usize, [bool; 4])> = work.par_iter().map(process_one).collect();
    #[cfg(not(feature = "parallel"))]
    let results: Vec<(usize, [bool; 4])> = work.iter().map(process_one).collect();

    #[derive(Default, Clone, Copy)]
    struct Tally {
        hits: [u32; 4],
        total: u32,
    }
    let mut tallies = vec![Tally::default(); SNR_TAGS.len()];
    for (idx, ok) in &results {
        let t = &mut tallies[*idx];
        t.total += 1;
        for (i, &o) in ok.iter().enumerate() {
            t.hits[i] += o as u32;
        }
    }

    eprintln!("FST4-60 {channel} llrc/llrd stage ablation (n=100/SNR):");
    eprintln!(
        "{:>6} {:>18} {:>18} {:>18} {:>18} {:>5}",
        "SNR", "full(+c+d)", "full-llrd(+c)", "no8_osd(+d)", "no8_osd-llrd", "n"
    );
    for (&(tag, _), t) in SNR_TAGS.iter().zip(&tallies) {
        eprintln!(
            "{:>6} {:>18} {:>18} {:>18} {:>18} {:>5}",
            tag, t.hits[0], t.hits[1], t.hits[2], t.hits[3], t.total
        );
    }
}

/// Correctness check for the new `fst4::rung_major::decode_rung_major`
/// (issue #306 item 3, VK3NV — the real, non-bench-only implementation,
/// promoted from the earlier bench prototype after dropping `llrd` per
/// `fst4_60_diag_stage_ablation`'s finding). Two checks:
///
/// 1. Real golden WAV: must still find exactly the same 2 real QSOs
///    production finds (`decode_wav_fst4_60`).
/// 2. Real AWGN corpus (m26/m27, n=100): recall should closely track
///    production's own `full`-config number
///    (`fst4_60_diag_npre_osd_recall_verify`: 96/100, 70/100) — not
///    necessarily bit-identical (this is a 5-stage reimplementation
///    sharing the same building blocks, not a call-through to
///    `process_candidate_basic_impl`), but the stage ablation already
///    confirmed dropping `llrd` costs nothing, so a large gap here
///    would indicate a bug in the reimplementation, not an expected
///    trade-off.
#[test]
#[ignore = "manual verification — decode_rung_major correctness (issue #306 item 3, VK3NV)"]
fn fst4_60_diag_decode_rung_major_correctness() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::pipeline::refine_candidate_position;
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
    use mfsk_core::fst4::rung_major::{RungMajorCandidate, decode_rung_major};
    use mfsk_core::msg::wsjt77::unpack77;

    // ---- 1. Real golden WAV ----
    let Some(path) = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    ) else {
        eprintln!("skip: real FST4-60 golden WAV not vendored/found");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV must load");
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let raw = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.2, None, 50);
    let refined: Vec<_> = raw
        .iter()
        .map(|c| refine_candidate_position::<Fst4s60>(c, &fft_cache, &FST4_60A_DOWNSAMPLE))
        .collect();
    let freq_tol = 0.10 * FST4_60A_DOWNSAMPLE.tone_spacing_hz;
    const I0_TOL: i32 = 2;
    let mut order: Vec<usize> = (0..raw.len()).collect();
    order.sort_by(|&a, &b| refined[b].3.partial_cmp(&refined[a].3).unwrap_or(std::cmp::Ordering::Equal));
    let mut kept: Vec<(f32, i32)> = Vec::new();
    let mut survivors: Vec<usize> = Vec::new();
    for idx in order {
        let (_, f, i0, _) = &refined[idx];
        if !kept.iter().any(|&(kf, ki)| (f - kf).abs() < freq_tol && (i0 - ki).abs() <= I0_TOL) {
            kept.push((*f, *i0));
            survivors.push(idx);
        }
    }
    survivors.sort_unstable();
    let cands: Vec<RungMajorCandidate> = survivors
        .iter()
        .map(|&i| {
            let (cd0, refined_freq_hz, i0, _score) = refined[i].clone();
            RungMajorCandidate {
                cand: raw[i].clone(),
                cd0,
                refined_freq_hz,
                i0,
            }
        })
        .collect();
    let results = decode_rung_major::<Fst4s60>(&cands, false);
    let msgs: Vec<&str> = results
        .iter()
        .flatten()
        .filter_map(|r| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(r.message77());
            unpack77(&m77).map(|s| Box::leak(s.into_boxed_str()) as &str)
        })
        .collect();
    eprintln!("golden WAV: {} candidates, {} decoded: {:?}", cands.len(), results.iter().flatten().count(), msgs);
    assert_eq!(results.iter().flatten().count(), 2, "must find exactly the 2 real QSOs");
    assert!(msgs.contains(&"CQ N5TM EL29"));
    assert!(msgs.contains(&"CQ K9KFR EN71"));

    // ---- 2. Real AWGN + CCIR-moderate corpora (m26/m27, n=100 each) ----
    let dir = sweep_dir();
    const SNR_TAGS: &[(&str, u32)] = &[("m26", 100), ("m27", 100)];
    for channel in ["awgn", "ccir_moderate"] {
        for &(snr_tag, trials) in SNR_TAGS {
            let mut pass = 0u32;
            for trial in 1..=trials {
                let path = dir.join(format!("fst4_60_{channel}_{snr_tag}_{trial:02}.wav"));
                let Some(audio) = load_wav_i16_opt(&path) else { continue };
                let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
                let raw = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.8, None, 50);
                let cands: Vec<RungMajorCandidate> = raw
                    .iter()
                    .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                    .map(|c| {
                        let (cd0, refined_freq_hz, i0, _score) =
                            refine_candidate_position::<Fst4s60>(c, &fft_cache, &FST4_60A_DOWNSAMPLE);
                        RungMajorCandidate {
                            cand: c.clone(),
                            cd0,
                            refined_freq_hz,
                            i0,
                        }
                    })
                    .collect();
                let results = decode_rung_major::<Fst4s60>(&cands, false);
                let found_golden = results.iter().flatten().any(|r| {
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(r.message77());
                    unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                });
                if found_golden {
                    pass += 1;
                }
            }
            eprintln!("decode_rung_major {channel} {snr_tag}: {pass}/{trials}");
        }
    }
}

/// Sanity check for the surprisingly large `full`(5-stage, no llrd)
/// real-hardware total (12.500s vs the 6-stage `full`'s 43.134s --
/// bigger than dropping one cheap stage should plausibly explain) --
/// counts how many of the golden WAV's 41 candidates actually clear
/// `nsync > SYNC_Q_MIN` (the gate before any BP/OSD work happens at
/// all), to check against this investigation's own earlier "8 of 41
/// reach the expensive stage" finding rather than trusting the
/// embedded number blind.
#[test]
#[ignore = "manual sanity check — decode_rung_major nsync-gate candidate count (issue #306 item 3)"]
fn fst4_60_diag_rung_major_nsync_gate_count() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::llr::{symbol_spectra, sync_quality};
    use mfsk_core::engine::pipeline::refine_candidate_position;
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    let Some(path) = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    ) else {
        eprintln!("skip: real FST4-60 golden WAV not vendored/found");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV must load");
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let raw = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.2, None, 50);
    let refined: Vec<_> = raw
        .iter()
        .map(|c| refine_candidate_position::<Fst4s60>(c, &fft_cache, &FST4_60A_DOWNSAMPLE))
        .collect();
    let freq_tol = 0.10 * FST4_60A_DOWNSAMPLE.tone_spacing_hz;
    const I0_TOL: i32 = 2;
    let mut order: Vec<usize> = (0..raw.len()).collect();
    order.sort_by(|&a, &b| refined[b].3.partial_cmp(&refined[a].3).unwrap_or(std::cmp::Ordering::Equal));
    let mut kept: Vec<(f32, i32)> = Vec::new();
    let mut survivors: Vec<usize> = Vec::new();
    for idx in order {
        let (_, f, i0, _) = &refined[idx];
        if !kept.iter().any(|&(kf, ki)| (f - kf).abs() < freq_tol && (i0 - ki).abs() <= I0_TOL) {
            kept.push((*f, *i0));
            survivors.push(idx);
        }
    }
    survivors.sort_unstable();

    const SYNC_Q_MIN: u32 = 16;
    let mut n_pass = 0;
    for &i in &survivors {
        let (cd0, _freq_hz, i0, _score) = &refined[i];
        let cs = symbol_spectra::<Fst4s60>(cd0, *i0);
        let nsync = sync_quality::<Fst4s60>(&cs);
        if nsync > SYNC_Q_MIN {
            n_pass += 1;
            eprintln!("  candidate idx={i} nsync={nsync} PASSES gate");
        }
    }
    eprintln!("{} of {} candidates pass nsync > {SYNC_Q_MIN}", n_pass, survivors.len());
}

/// Cross-check for `fst4_60_diag_rung_major_nsync_gate_count`: parses
/// the *actual baked asset* `fst4_bench` embeds
/// (`embedded-poc/assets/fst4_60_golden_refined_candidates.bin`)
/// directly, host-side, instead of re-running coarse_sync/refine/dedup
/// fresh — checks whether the embedded binary's candidate set matches
/// (same nsync>16 count) or has drifted from a fresh run (e.g. a stale
/// bake from before some earlier refine/dedup fix this session).
#[test]
#[ignore = "manual sanity check — baked asset vs fresh candidate set (issue #306 item 3)"]
fn fst4_60_diag_baked_asset_nsync_gate_count() {
    use mfsk_core::engine::llr::{symbol_spectra, sync_quality};
    use mfsk_core::fst4::Fst4s60;

    let bin = std::fs::read("../embedded-poc/assets/fst4_60_golden_refined_candidates.bin")
        .expect("baked refined-candidates asset must exist");
    eprintln!("baked asset: {} bytes", bin.len());

    let mut off = 0usize;
    let n = u32::from_le_bytes(bin[off..off + 4].try_into().unwrap()) as usize;
    off += 4;
    eprintln!("baked asset claims {n} candidates");

    const FFT2_SIZE: usize = 6912; // FST4_60A_DOWNSAMPLE.fft2_size
    const SYNC_Q_MIN: u32 = 16;
    let mut n_pass = 0;
    for i in 0..n {
        let _freq_hz = f32::from_le_bytes(bin[off..off + 4].try_into().unwrap());
        let _dt_sec = f32::from_le_bytes(bin[off + 4..off + 8].try_into().unwrap());
        let _score = f32::from_le_bytes(bin[off + 8..off + 12].try_into().unwrap());
        let _refined_freq_hz = f32::from_le_bytes(bin[off + 12..off + 16].try_into().unwrap());
        let refined_i0 = i32::from_le_bytes(bin[off + 16..off + 20].try_into().unwrap());
        let _refined_score = f32::from_le_bytes(bin[off + 20..off + 24].try_into().unwrap());
        off += 24;
        let mut cd0 = Vec::with_capacity(FFT2_SIZE);
        for _ in 0..FFT2_SIZE {
            let re = f32::from_le_bytes(bin[off..off + 4].try_into().unwrap());
            let im = f32::from_le_bytes(bin[off + 4..off + 8].try_into().unwrap());
            cd0.push(num_complex::Complex32::new(re, im));
            off += 8;
        }
        let cs = symbol_spectra::<Fst4s60>(&cd0, refined_i0);
        let nsync = sync_quality::<Fst4s60>(&cs);
        if nsync > SYNC_Q_MIN {
            n_pass += 1;
            eprintln!("  candidate #{i} nsync={nsync} PASSES gate");
        }
    }
    assert_eq!(off, bin.len(), "byte accounting mismatch");
    eprintln!("{n_pass} of {n} baked candidates pass nsync > {SYNC_Q_MIN}");
}

fn host_clock_us() -> i64 {
    use std::sync::OnceLock;
    use std::time::Instant;
    static START: OnceLock<Instant> = OnceLock::new();
    let start = START.get_or_init(Instant::now);
    start.elapsed().as_micros() as i64
}

/// Digs into the surprisingly large gap between the old 6-stage
/// `full` bench (43.134s, real hardware) and the new 5-stage
/// `decode_rung_major` `full` (12.500s, real hardware) -- host-side
/// per-candidate, per-stage timing via `decode_rung_major_timed`, on
/// the same 41 golden-WAV candidates both real-hardware runs used.
/// Host absolute numbers aren't embedded-calibrated, but the *relative*
/// shape (which stage costs what, which candidates reach OSD) should
/// carry over and is enough to tell a genuine bug from a real
/// algorithmic saving.
#[test]
#[ignore = "manual root-cause probe — decode_rung_major per-stage timing (issue #306 item 3)"]
fn fst4_60_diag_rung_major_stage_timing_probe() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::pipeline::refine_candidate_position;
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
    use mfsk_core::fst4::rung_major::{RungMajorCandidate, decode_rung_major_timed};

    let Some(path) = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    ) else {
        eprintln!("skip: real FST4-60 golden WAV not vendored/found");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV must load");
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let raw = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.2, None, 50);
    let refined: Vec<_> = raw
        .iter()
        .map(|c| refine_candidate_position::<Fst4s60>(c, &fft_cache, &FST4_60A_DOWNSAMPLE))
        .collect();
    let freq_tol = 0.10 * FST4_60A_DOWNSAMPLE.tone_spacing_hz;
    const I0_TOL: i32 = 2;
    let mut order: Vec<usize> = (0..raw.len()).collect();
    order.sort_by(|&a, &b| refined[b].3.partial_cmp(&refined[a].3).unwrap_or(std::cmp::Ordering::Equal));
    let mut kept: Vec<(f32, i32)> = Vec::new();
    let mut survivors: Vec<usize> = Vec::new();
    for idx in order {
        let (_, f, i0, _) = &refined[idx];
        if !kept.iter().any(|&(kf, ki)| (f - kf).abs() < freq_tol && (i0 - ki).abs() <= I0_TOL) {
            kept.push((*f, *i0));
            survivors.push(idx);
        }
    }
    survivors.sort_unstable();
    let cands: Vec<RungMajorCandidate> = survivors
        .iter()
        .map(|&i| {
            let (cd0, refined_freq_hz, i0, _score) = refined[i].clone();
            RungMajorCandidate {
                cand: raw[i].clone(),
                cd0,
                refined_freq_hz,
                i0,
            }
        })
        .collect();

    let (results, timings) = decode_rung_major_timed::<Fst4s60>(&cands, false, false, Some(host_clock_us));
    let timings = timings.unwrap();
    eprintln!("decoded: {}/{}", results.iter().flatten().count(), cands.len());
    eprintln!("{:>4} {:>8} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}", "idx", "freq", "llra_us", "llrb_us", "llre_us", "llrc_us", "osd_us", "decoded");
    for (i, (c, t)) in cands.iter().zip(&timings).enumerate() {
        let total: i64 = t.iter().sum();
        if total > 0 {
            eprintln!(
                "{:>4} {:>8.1} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}",
                i, c.cand.freq_hz, t[0], t[1], t[2], t[3], t[4], results[i].is_some()
            );
        }
    }
}

/// VK3NV's issue #306 item 2 follow-up: WSJT-X's FST4 decoder retries
/// each candidate at timing offsets `i0-1, i0, i0+1` (building fresh
/// 1/2/4/8-symbol bit-metric sets at each), which mfsk-core's own
/// single-position dispatch doesn't do. For the 5 CCIR-moderate m26
/// "old-only" trials found by `fst4_60_diag_npre_osd_bug_hunt_ccir_
/// moderate` (old unpruned OSD decoded; new npre1/npre2 OSD didn't) --
/// does giving the *new* npre decoder the same `i0±1` timing diversity
/// recover any of them? If so, part of the apparent npre/CCIR recall
/// gap is really a missing-timing-diversity gap, not an OSD-pruning
/// gap.
#[test]
#[ignore = "manual diagnostic — WSJT-X-style i0±1 timing retry on CCIR-moderate old-only trials (issue #306 item 2, VK3NV)"]
fn fst4_60_diag_i0_retry_ccir_old_only() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    const OLD_ONLY_TRIALS: &[u32] = &[2, 15, 33, 45, 67];

    let dir = sweep_dir();
    let (osd_attempt_min, osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();
    let fec = <Fst4s60 as Protocol>::Fec::default();
    let verify_info =
        Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

    for &trial in OLD_ONLY_TRIALS {
        let path = dir.join(format!("fst4_60_ccir_moderate_m26_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            eprintln!("trial {trial}: WAV missing, skip");
            continue;
        };
        let cands = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.8, None, 50);
        let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
        let ds_rate = 12_000.0 / <Fst4s60 as mfsk_core::ModulationParams>::NDOWN as f32;

        let mut recovered = false;
        let mut best_nsync_by_offset = [0u32; 3];
        for c in cands.iter().filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ) {
            let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FST4_60A_DOWNSAMPLE);
            let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for z in cd0.iter_mut() {
                    *z *= inv;
                }
            }
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            let df_hz = s2.freq_hz - c.freq_hz;
            let cd0 = freq_shift_cd0(&cd0, df_hz, ds_rate);

            for (oi, &di0) in [-1i32, 0, 1].iter().enumerate() {
                let i0 = s2.i0 + di0;
                let cs = symbol_spectra::<Fst4s60>(&cd0, i0);
                let nsync = sync_quality::<Fst4s60>(&cs);
                best_nsync_by_offset[oi] = best_nsync_by_offset[oi].max(nsync);
                if nsync <= 16 {
                    continue;
                }
                let llr_set = compute_llr::<Fst4s60, f32>(&cs);
                let variants: [&Vec<f32>; 4] =
                    [&llr_set.llra, &llr_set.llrb, &llr_set.llre, &llr_set.llrc];
                let is_golden = |llr: &Vec<f32>, opts: &FecOpts| -> bool {
                    fec.decode_soft(llr, opts).is_some_and(|mut r| {
                        mfsk_core::engine::llr::descramble_info::<Fst4s60>(&mut r.info);
                        let mut m77 = [0u8; 77];
                        m77.copy_from_slice(&r.info[..77]);
                        unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                    })
                };
                let bp_opts = FecOpts {
                    bp_max_iter: 30,
                    osd_depth: 0,
                    ap_mask: None,
                    verify_info,
                    ..FecOpts::default()
                };
                if variants.iter().any(|llr| is_golden(llr, &bp_opts)) {
                    recovered = true;
                }
                if !recovered && nsync >= osd_attempt_min {
                    let osd_depth: u32 = if nsync >= osd_depth3_min { 3 } else { 2 };
                    let osd_opts = FecOpts {
                        bp_max_iter: 30,
                        osd_depth,
                        ap_mask: None,
                        verify_info,
                        ..FecOpts::default()
                    };
                    if variants.iter().any(|llr| is_golden(llr, &osd_opts)) {
                        recovered = true;
                    }
                }
            }
        }
        eprintln!(
            "trial {trial}: recovered_with_i0_retry={recovered}  best_nsync(i0-1,i0,i0+1)={:?}",
            best_nsync_by_offset
        );
    }
}
