//! FT4 SNR sweep + fading benchmark against `ft4sim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually when investigating FT4
//! sensitivity / `DecodeStrictness` calibration (issue #72).
//!
//! ```sh
//! # 1. Generate WAVs (once, or when widening the SNR grid):
//! scripts/build_ft4sim.sh
//! scripts/gen_ft4_sweep_wavs.sh
//!
//! # 2. Run the sweep:
//! cargo test --test ft4_sweep --release \
//!   --features ft4,fft-rustfft,parallel,uvpacket,internal-testing \
//!   -- --ignored --nocapture
//! ```
//!
//! (`uvpacket` is only required because `tests/common/channel.rs`, pulled in
//! via `mod common`, unconditionally imports `mfsk_core::uvpacket` — unrelated
//! to FT4 itself. `internal-testing` (issue #203) is required because this
//! file calls `engine::pipeline::{decode_frame, process_candidate_basic}`
//! directly, which are `pub(crate)` on the default feature set.
//! `MFSK_FT4_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/ft4_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! Output is a recall table — no assertions, statistics only. Set
//! `MFSK_FT4_SWEEP_CSV=/path/out.csv` to also dump raw per-trial pass/fail
//! rows for bootstrap-CI analysis of the 50%-crossing estimate. See
//! `docs/notes/FST4_BENCHMARK.md` for the shared methodology this mirrors —
//! FT4 has no sub-modes, so there's one grid instead of one per T/R period.

#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

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
    if let Ok(d) = std::env::var("MFSK_FT4_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/ft4_sweep")
        .to_path_buf()
}

// ── Channel conditions (must match gen_ft4_sweep_wavs.sh CHANNELS) ─────────
#[allow(dead_code)]
const CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];

fn decode_wav_ft4(audio: &[i16]) -> bool {
    mfsk_core::msg::decode_request::DecodeRequest::<mfsk_core::ft4::Ft4>::new(
        audio, 100.0, 3000.0, 0.8, 50,
    )
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

// ── Filename parsing ─────────────────────────────────────────────────────────

/// Parse `ft4_<channel>_<snr_tag>_<trial>.wav`.
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
        // ft4_awgn_m05_01
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() < 4 || parts[0] != "ft4" {
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
        // channel = everything between "ft4" and snr_tag
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
fn ft4_snr_sweep() {
    let dir = sweep_dir();
    let all_wavs = collect_wavs(&dir);

    if all_wavs.is_empty() {
        eprintln!(
            "No WAVs found in {:?}\n\
             Run: scripts/build_ft4sim.sh && scripts/gen_ft4_sweep_wavs.sh",
            dir
        );
        return;
    }

    // Optional env-var filters — narrow the sweep to the region of interest.
    // MFSK_FT4_SWEEP_CHANNELS=awgn       (comma-separated channel names)
    // MFSK_FT4_SWEEP_SNR_MIN=-20         (inclusive lower bound, dB)
    // MFSK_FT4_SWEEP_SNR_MAX=-14         (inclusive upper bound, dB)
    // MFSK_FT4_SWEEP_CSV=/path/out.csv   (optional: dump raw per-trial
    //   pass/fail rows — channel,snr_db,trial,pass)
    let chan_filter: Option<Vec<String>> = std::env::var("MFSK_FT4_SWEEP_CHANNELS")
        .ok()
        .map(|s| s.split(',').map(|v| v.trim().to_string()).collect());
    let snr_min: Option<i32> = std::env::var("MFSK_FT4_SWEEP_SNR_MIN")
        .ok()
        .and_then(|s| s.trim().parse().ok());
    let snr_max: Option<i32> = std::env::var("MFSK_FT4_SWEEP_SNR_MAX")
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

    let mut csv = std::env::var("MFSK_FT4_SWEEP_CSV").ok().map(|path| {
        let mut f = std::fs::File::create(&path)
            .unwrap_or_else(|e| panic!("MFSK_FT4_SWEEP_CSV={path}: {e}"));
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
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_wav_ft4(&audio)))
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<(u32, bool)> = wav_group
            .iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_wav_ft4(&audio)))
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

/// `DecodeStrictness` calibration probe (issue #72) — does loosening past
/// `Normal` actually recover any of the near-threshold trials that
/// `ft4_snr_sweep` shows as misses, or is FT4's SNR floor set entirely by
/// coarse-sync / BP and the OSD-gate knob copied from FT8 is a no-op for
/// this protocol? Run once a full sweep (above) has identified the
/// partial-recall band per channel.
///
/// `SYNC_Q_MIN = 8` below mirrors the private constant in
/// `src/ft4/decode.rs` — duplicated here because integration tests only
/// see `pub` items.
#[test]
#[ignore = "manual diagnostic — DecodeStrictness calibration probe (issue #72)"]
fn ft4_strictness_probe() {
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, decode_frame};
    use mfsk_core::ft4::Ft4;
    use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;

    const SYNC_Q_MIN: u32 = 8;

    fn snr_tag(snr: i32) -> String {
        if snr < 0 {
            format!("m{:02}", -snr)
        } else {
            format!("p{:02}", snr)
        }
    }

    let dir = sweep_dir();
    // Partial-recall band identified by a prior `ft4_snr_sweep` run
    // (2026-07-18, this corpus/seed): awgn -16..-13, ccir_moderate -16..-13,
    // ccir_poor -16..-13. Narrow window — this is a diagnostic, not a
    // full re-sweep (see FST4_BENCHMARK.md section 7).
    let cells: &[(&str, i32)] = &[
        ("awgn", -16),
        ("awgn", -15),
        ("ccir_moderate", -16),
        ("ccir_moderate", -15),
        ("ccir_moderate", -14),
        ("ccir_poor", -15),
        ("ccir_poor", -14),
        ("ccir_poor", -13),
    ];
    let strictness_levels = [
        DecodeStrictness::Strict,
        DecodeStrictness::Normal,
        DecodeStrictness::Deep,
    ];

    eprintln!("\n{:-<72}", "");
    eprintln!(
        "  {:<14} {:>7}  {:<8}   {:>10}  {:>10}",
        "Channel", "SNR(dB)", "Level", "Golden", "Any-msg"
    );
    eprintln!("{:-<72}", "");
    for &(chan, snr) in cells {
        let tag = snr_tag(snr);
        for &strictness in &strictness_levels {
            let mut golden_hits = 0u32;
            let mut any_hits = 0u32;
            let mut trials = 0u32;
            for t in 1..=20u32 {
                let path = dir.join(format!("ft4_{chan}_{tag}_{t:02}.wav"));
                let Some(audio) = load_wav_i16_opt(&path) else {
                    continue;
                };
                trials += 1;
                let (results, _) = decode_frame::<Ft4>(
                    &audio,
                    &FT4_DOWNSAMPLE,
                    100.0,
                    3000.0,
                    0.8,
                    None,
                    DecodeDepth::FULL,
                    50,
                    strictness,
                    EqMode::Off,
                    SYNC_Q_MIN,
                    None,
                    None,
                );
                any_hits += !results.is_empty() as u32;
                let is_golden = results.iter().any(|d| {
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(d.message77());
                    unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
                        && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
                        && d.dt_sec.abs() <= DT_TOL_SEC
                });
                golden_hits += is_golden as u32;
            }
            if trials == 0 {
                continue;
            }
            eprintln!(
                "  {:<14} {:>4} dB  {:<8?}   {:>4}/{:<4}   {:>4}/{:<4}",
                chan, snr, strictness, golden_hits, trials, any_hits, trials
            );
        }
        eprintln!();
    }
    eprintln!("{:-<72}", "");
    eprintln!(
        "Golden = matches the known transmitted message (recall).\n\
         Any-msg = any CRC-passing decode at all (recall + false-accepts).\n\
         If Golden is flat across Strict/Normal/Deep at a given cell, the\n\
         OSD-gate knob isn't the bottleneck there — look at coarse-sync/BP\n\
         instead. If Any-msg grows faster than Golden as strictness loosens,\n\
         that's false-accept inflation, not real sensitivity gain."
    );
}

/// Per-trial diagnostic (issue #72 / `FT4_BENCHMARK.md` AWGN gap
/// investigation, 2026-07-18) mirroring `fst4_diag_weak_trials` in
/// `fst4_sweep.rs`. Traces `coarse_sync` + `process_candidate_basic`
/// directly on individual AWGN trials straddling the current ≈-15.7 dB
/// 50% crossing, to determine — per trial — whether a losing decode is
/// because `coarse_sync` never finds a near-golden-freq candidate at all,
/// or because a found candidate fails somewhere inside
/// `process_candidate_basic` (fine-refine / sync-quality gate / LLR /
/// BP / OSD). This is read manually, not asserted — see `FST4_BENCHMARK.md`
/// section 6 for why this stage-attribution step comes before any fix.
///
/// `SYNC_Q_MIN = 8` mirrors the private constant in `src/ft4/decode.rs`
/// (integration tests only see `pub` items).
#[test]
#[ignore = "manual diagnostic — AWGN sensitivity gap stage attribution (issue #72)"]
fn ft4_diag_weak_trials() {
    use mfsk_core::ModulationParams;
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::llr::{symbol_spectra, sync_quality};
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::{SyncCandidate, coarse_sync};
    use mfsk_core::engine::sync2d::ft4_sync_search;
    use mfsk_core::ft4::Ft4;
    use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;

    const SYNC_Q_MIN: u32 = 8;

    // Mirrors `process_candidate_basic`'s downsample -> RMS-normalize ->
    // `ft4_sync_search` -> `symbol_spectra` -> `sync_quality` prefix
    // (`core/pipeline.rs:157-217`), stopping right before the `nsync`
    // gate so we can print the raw value instead of just pass/fail.
    fn nsync_for(
        cand: &SyncCandidate,
        fft_cache: &[num_complex::Complex<f32>],
        cfg: &mfsk_core::engine::dsp::downsample::DownsampleCfg,
    ) -> u32 {
        let mut cd0 = downsample_cached(fft_cache, cand.freq_hz, cfg);
        let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
        if sum2 > f32::EPSILON {
            let inv = 1.0 / sum2.sqrt();
            for c in cd0.iter_mut() {
                *c *= inv;
            }
        }
        let s2 = ft4_sync_search::<Ft4>(&cd0, cand);
        let df_hz = s2.freq_hz - cand.freq_hz;
        let ds_rate = 12_000.0 / Ft4::NDOWN as f32;
        let cd0 = mfsk_core::engine::sync2d::freq_shift_cd0(&cd0, df_hz, ds_rate);
        let cs_raw = symbol_spectra::<Ft4>(&cd0, s2.i0);
        sync_quality::<Ft4>(&cs_raw)
    }

    let dir = sweep_dir();
    // Straddles the post-section-9 AWGN 50% crossing (≈-16.7dB,
    // FT4_BENCHMARK.md section 9) — deliberately narrow, this is a
    // stage-attribution probe, not a re-sweep.
    for snr_tag in ["m18", "m17", "m16", "m15"] {
        for trial in 1..=20u32 {
            let path = dir.join(format!("ft4_awgn_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            let cands = coarse_sync::<Ft4>(&audio, 100.0, 3000.0, 0.8, None, 50);
            let near: Vec<_> = cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            eprintln!(
                "awgn {snr_tag} trial {trial}: {} candidates total, {} near golden freq",
                cands.len(),
                near.len()
            );
            if near.is_empty() {
                continue;
            }
            let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
            for c in &near {
                // N_SYNC=16 for FT4 (4 blocks x 4-symbol Costas); the
                // OSD-attempt/depth-3 gates are 9/14 post section-8 fix
                // (core/pipeline.rs), not the FT8-scale 12/18.
                let nsync = nsync_for(c, &fft_cache, &FT4_DOWNSAMPLE);
                eprintln!(
                    "  cand freq={:.2} dt={:.3} score={:.4} nsync={}/16 (osd@9={} osd3@14={})",
                    c.freq_hz,
                    c.dt_sec,
                    c.score,
                    nsync,
                    nsync >= 9,
                    nsync >= 14
                );
            }
            for c in &near {
                let r = process_candidate_basic::<Ft4>(
                    c,
                    &fft_cache,
                    &FT4_DOWNSAMPLE,
                    DecodeDepth::FULL,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    SYNC_Q_MIN,
                );
                // pass: 0/1/2/3=BP variants a/b/c/d, 6=llre (BP succeeded
                // without OSD); 4=OSD depth-2, 5=OSD depth-3 (BP needed
                // rescuing). hard_errors is the LDPC decoder's own residual
                // error count on the winning codeword.
                eprintln!(
                    "  -> decode result: {:?}",
                    r.map(|d| (d.pass, d.hard_errors, d.sync_score))
                );
            }
        }
    }
}

/// Hypothesis 2 from `FT4_BENCHMARK.md` section 8's closing notes: does
/// `ft4_sync_search`'s collapsed single-pass Δt search (one global best
/// over the full `[-344,1012]` union) ever lose recall relative to
/// WSJT-X's literal 3-segment structure (`ft4_decode.f90`'s `iseg=1,2,3`
/// loop), which can attempt a decode at up to 3 different positions per
/// candidate (segment 1 `[108,560]`, then segment 2 `[560,1012]` / segment
/// 3 `[-344,108]` only if that segment's own `smax` clears both the
/// absolute floor 1.2 and segment 1's `smax`)?
///
/// For every currently-failing near-golden candidate in the AWGN
/// crossing band: run the collapsed search first (skip if it already
/// succeeds — only interested in today's failures), then replicate the
/// literal 3-segment search via
/// [`mfsk_core::engine::sync2d::ft4_sync_search_window`] and attempt a
/// decode at every segment whose gate passes, via a local
/// `try_decode_at` that mirrors `process_candidate_basic`'s LLR/BP/OSD
/// tail (symbol_spectra -> sync_quality -> BP variants -> OSD) for an
/// explicit `(freq_hz, i0)` rather than re-running the full search — the
/// gate is intentionally permissive (no pre-OSD score check — matches
/// production since issue #230 removed the `osd_score_min` gate this
/// diagnostic used to skip around) to give a segment-2/3 rescue its best
/// possible chance. A rescue here means WSJT-X's literal per-segment
/// retry structure is worth porting; a clean floor rules it out for pure
/// AWGN.
#[test]
#[ignore = "manual diagnostic — 3-segment Δt-search retry hypothesis (issue #72)"]
fn ft4_diag_segment_retry() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::engine::sync2d::{freq_shift_cd0, ft4_sync_search_window};
    use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
    use mfsk_core::ft4::Ft4;
    use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;

    const SYNC_Q_MIN: u32 = 8;
    const BP_MAX_ITER: u32 = 40; // FT4's own value, section 10.
    const OSD_ATTEMPT_MIN: u32 = 9; // section 8's FT4-scaled gate.
    const OSD_DEPTH3_MIN: u32 = 14;
    // WSJT-X `ft4_decode.f90:240-251` segment boundaries.
    const SEGMENTS: [(i32, i32); 3] = [(108, 560), (560, 1012), (-344, 108)];
    const SYNCMIN: f32 = 1.2;

    // Mirrors `process_candidate_basic`'s LLR/BP/OSD tail for one explicit
    // `(freq_hz, i0)` position instead of re-deriving it via
    // `ft4_sync_search` — INCLUDING the `hard_errors >= osd_max_errors`
    // rejection gate real OSD results go through (first attempt at this
    // diagnostic omitted it and over-reported rescues — see
    // `FT4_BENCHMARK.md` section 11 for the retraction). Returns
    // `Some((hard_errors, message77))` on a decode that would actually
    // survive production's gates.
    fn try_decode_at(
        cd0_norm: &[num_complex::Complex<f32>],
        freq_hz: f32,
        cand_freq_hz: f32,
        i0: i32,
        ds_rate: f32,
        allow_osd: bool,
    ) -> Option<(u32, [u8; 77])> {
        let df_hz = freq_hz - cand_freq_hz;
        let shifted = freq_shift_cd0(cd0_norm, df_hz, ds_rate);
        let cs = symbol_spectra::<Ft4>(&shifted, i0);
        let nsync = sync_quality::<Ft4>(&cs);
        if nsync <= SYNC_Q_MIN {
            return None;
        }
        // No `deinterleave_llr_set` call: FT4's `CODEWORD_INTERLEAVE` is
        // `None`, and `core/pipeline.rs` documents that call as a no-op
        // for FT4/FT8/FST4 — same result either way.
        let llr_set = compute_llr::<Ft4, f32>(&cs);
        let variants = [&llr_set.llra, &llr_set.llrb, &llr_set.llrc, &llr_set.llrd];
        let fec = <<Ft4 as Protocol>::Fec>::default();
        let verify_info =
            Some(<<Ft4 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);
        let bp_opts = FecOpts {
            bp_max_iter: BP_MAX_ITER,
            osd_depth: 0,
            ap_mask: None,
            verify_info,
            ..FecOpts::default()
        };
        let msg77 = |info: &[u8]| -> [u8; 77] {
            let mut m = [0u8; 77];
            m.copy_from_slice(&info[..77]);
            m
        };
        for llr in variants {
            if let Some(r) = fec.decode_soft(llr, &bp_opts) {
                return Some((r.hard_errors, msg77(&r.info)));
            }
        }
        if !allow_osd || nsync < OSD_ATTEMPT_MIN {
            return None;
        }
        let osd_depth: u32 = if nsync >= OSD_DEPTH3_MIN { 3 } else { 2 };
        let osd_opts = FecOpts {
            bp_max_iter: BP_MAX_ITER,
            osd_depth,
            ap_mask: None,
            verify_info,
            ..FecOpts::default()
        };
        let strictness = DecodeStrictness::Normal;
        for llr in variants {
            if let Some(r) = fec.decode_soft(llr, &osd_opts) {
                if r.hard_errors >= strictness.osd_max_errors(osd_depth as u8) {
                    continue;
                }
                return Some((r.hard_errors, msg77(&r.info)));
            }
        }
        if nsync >= OSD_DEPTH3_MIN {
            let osd4_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: 4,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            for llr in variants {
                if let Some(r) = fec.decode_soft(llr, &osd4_opts) {
                    if r.hard_errors >= strictness.osd_max_errors(4) {
                        continue;
                    }
                    return Some((r.hard_errors, msg77(&r.info)));
                }
            }
        }
        None
    }

    let dir = sweep_dir();
    let mut candidates_checked = 0u32;
    let mut rescued = 0u32;

    for snr_tag in ["m18", "m17", "m16", "m15"] {
        for trial in 1..=20u32 {
            let path = dir.join(format!("ft4_awgn_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            let cands = coarse_sync::<Ft4>(&audio, 100.0, 3000.0, 0.8, None, 50);
            let near: Vec<_> = cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            if near.is_empty() {
                continue;
            }
            let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);

            for c in &near {
                let baseline = process_candidate_basic::<Ft4>(
                    c,
                    &fft_cache,
                    &FT4_DOWNSAMPLE,
                    DecodeDepth::FULL,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    SYNC_Q_MIN,
                );
                if baseline.is_some() {
                    continue;
                }
                candidates_checked += 1;

                let ds_rate = 12_000.0 / <Ft4 as mfsk_core::ModulationParams>::NDOWN as f32;
                let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT4_DOWNSAMPLE);
                let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
                if sum2 > f32::EPSILON {
                    let inv = 1.0 / sum2.sqrt();
                    for z in cd0.iter_mut() {
                        *z *= inv;
                    }
                }

                let mut smax1 = f32::NEG_INFINITY;
                let mut seg_rescue = false;
                for (seg_idx, &(ib_min, ib_max)) in SEGMENTS.iter().enumerate() {
                    let s2 = ft4_sync_search_window::<Ft4>(&cd0, c, ib_min, ib_max);
                    if seg_idx == 0 {
                        smax1 = s2.score;
                    }
                    if s2.score < SYNCMIN {
                        continue;
                    }
                    if seg_idx > 0 && s2.score < smax1 {
                        continue;
                    }
                    // Production has no pre-OSD score gate at all since
                    // issue #230 (`osd_score_min` removed — it had no
                    // live caller). Always attempt OSD once BP fails,
                    // matching production; BP itself is still attempted
                    // unconditionally either way.
                    let allow_osd = true;
                    if let Some((hard_errors, msg77)) =
                        try_decode_at(&cd0, s2.freq_hz, c.freq_hz, s2.i0, ds_rate, allow_osd)
                    {
                        let is_golden = unpack77(&msg77).as_deref() == Some(GOLDEN_MSG);
                        eprintln!(
                            "  {} {snr_tag} trial {trial}: segment {seg_idx} \
                             (ib={ib_min}..{ib_max}) freq={:.2} i0={} score={:.3} \
                             hard_errors={hard_errors} golden={is_golden}",
                            if is_golden { "RESCUE" } else { "FALSE-ACCEPT" },
                            s2.freq_hz,
                            s2.i0,
                            s2.score
                        );
                        if is_golden {
                            seg_rescue = true;
                        }
                    }
                }
                if seg_rescue {
                    rescued += 1;
                }
            }
        }
    }
    eprintln!(
        "\nfailing candidates checked={candidates_checked} rescued-by-segment-retry={rescued}"
    );
}

/// Phase 0 diagnostic for the coarse_sync-precision / BP-OSD-speed plan
/// (see `~/.claude/plans/dapper-soaring-nest.md`): measures whether FT4's
/// generic `engine::sync::coarse_sync` — a 2-D (freq × lag) Costas-array
/// correlation search, unlike WSJT-X's `getcandidates4.f90` (a
/// frequency-only periodogram with no lag dimension at all) — produces
/// multiple lag-distinct candidates per real signal frequency, and how
/// much of `process_candidate_basic`'s per-candidate wall-clock is spent
/// in `ft4_sync_search`'s coherent grid search vs. everything after it
/// (symbol_spectra + LLR + BP + OSD).
///
/// `ft4_sync_search` already does an *absolute* full-window Δt search
/// that ignores each candidate's own `dt_sec` (FT4_BENCHMARK.md section
/// 9) — so if a real signal's frequency does produce several lag-distinct
/// coarse_sync candidates, each pays this diagnostic's full per-candidate
/// cost independently for what should be a redundant, identical result.
///
/// Run against the real WSJT-X FT4 golden WAV (skipped if the sibling
/// WSJT-X checkout isn't present) and a handful of representative sweep
/// cells. This is a measurement tool, not a pass/fail gate — no
/// assertions, `--nocapture` output only.
#[test]
#[ignore = "manual diagnostic — coarse_sync candidate redundancy / cost split (dapper-soaring-nest plan, Phase 0)"]
fn ft4_diag_candidate_cost_split() {
    use std::time::{Duration, Instant};

    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::{SyncCandidate, coarse_sync};
    use mfsk_core::engine::sync2d::ft4_sync_search;
    use mfsk_core::ft4::Ft4;
    use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;

    const SYNC_Q_MIN: u32 = 8;

    #[derive(Default)]
    struct CostSplit {
        n_candidates: usize,
        n_freq_buckets: usize,
        coarse_sync: Duration,
        downsample: Duration,
        sync_search: Duration,
        process_total: Duration,
        n_decoded: usize,
    }

    /// Bucket width matches `coarse_sync`'s own within-frequency dedup
    /// radius (`sync.rs`: `fdiff < 4.0`) — candidates in the same 4 Hz
    /// bucket are, by that same logic, the same underlying signal.
    fn freq_bucket_count(cands: &[SyncCandidate]) -> usize {
        let mut buckets: Vec<i32> = cands
            .iter()
            .map(|c| (c.freq_hz / 4.0).round() as i32)
            .collect();
        buckets.sort_unstable();
        buckets.dedup();
        buckets.len()
    }

    fn measure(
        label: &str,
        audio: &[i16],
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        use_new: bool,
    ) {
        let t0 = Instant::now();
        let cands = if use_new {
            ft4_coarse_sync(audio, freq_min, freq_max, sync_min, None, max_cand)
        } else {
            coarse_sync::<Ft4>(audio, freq_min, freq_max, sync_min, None, max_cand)
        };
        let coarse_dt = t0.elapsed();

        let mut split = CostSplit {
            n_candidates: cands.len(),
            n_freq_buckets: freq_bucket_count(&cands),
            coarse_sync: coarse_dt,
            ..Default::default()
        };

        let fft_cache = build_fft_cache(audio, &FT4_DOWNSAMPLE);
        let ds_rate = 12_000.0 / <Ft4 as mfsk_core::ModulationParams>::NDOWN as f32;

        for c in &cands {
            let t0 = Instant::now();
            let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT4_DOWNSAMPLE);
            let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for z in cd0.iter_mut() {
                    *z *= inv;
                }
            }
            split.downsample += t0.elapsed();

            let t0 = Instant::now();
            let _ = ft4_sync_search::<Ft4>(&cd0, c);
            split.sync_search += t0.elapsed();
            let _ = ds_rate;

            let t0 = Instant::now();
            let r = process_candidate_basic::<Ft4>(
                c,
                &fft_cache,
                &FT4_DOWNSAMPLE,
                DecodeDepth::FULL,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
                SYNC_Q_MIN,
            );
            split.process_total += t0.elapsed();
            if r.is_some() {
                split.n_decoded += 1;
            }
        }

        let llr_bp_osd = split
            .process_total
            .saturating_sub(split.downsample)
            .saturating_sub(split.sync_search);
        eprintln!(
            "{label}: {} candidates, {} distinct freq buckets (avg {:.1} cand/freq)",
            split.n_candidates,
            split.n_freq_buckets,
            split.n_candidates as f32 / split.n_freq_buckets.max(1) as f32
        );
        eprintln!(
            "  coarse_sync={:>7.1}ms  per-cand[downsample={:>7.1}ms sync_search={:>7.1}ms \
             llr+bp+osd(inferred)={:>7.1}ms]  decoded={}",
            split.coarse_sync.as_secs_f64() * 1000.0,
            split.downsample.as_secs_f64() * 1000.0,
            split.sync_search.as_secs_f64() * 1000.0,
            llr_bp_osd.as_secs_f64() * 1000.0,
            split.n_decoded
        );
    }

    // ── Golden WAV (matches `ft4_wsjtx_samples.rs`'s exact search params) ──
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let golden_path = Path::new(&manifest).join("../../WSJT-X/samples/FT4/000000_000002.wav");
    if let Ok(golden_path) = golden_path.canonicalize() {
        if let Some(raw) = load_wav_i16_opt(&golden_path) {
            const SLOT_SAMPLES: usize = 90_000;
            let mut audio = vec![0i16; SLOT_SAMPLES];
            let copy = raw.len().min(SLOT_SAMPLES);
            audio[..copy].copy_from_slice(&raw[..copy]);
            measure(
                "golden 000000_000002.wav [OLD generic coarse_sync]",
                &audio,
                100.0,
                2700.0,
                0.05,
                2000,
                false,
            );
            measure(
                "golden 000000_000002.wav [NEW ft4_coarse_sync]",
                &audio,
                100.0,
                2700.0,
                0.05,
                2000,
                true,
            );

            // Real production entry point (rayon-parallel, 3 subtract
            // passes) — the number directly comparable to
            // `BENCHMARKS.md`'s "Decode speed" table.
            let t0 = Instant::now();
            let decodes =
                mfsk_core::msg::decode_request::DecodeRequest::<mfsk_core::ft4::Ft4>::new(
                    &audio, 100.0, 2700.0, 0.05, 2000,
                )
                .sic_rounds(3)
                .decode()
                .results;
            let dt = t0.elapsed();
            eprintln!(
                "golden 000000_000002.wav: flat SIC wall-clock = {:.1} ms, {} decode(s)",
                dt.as_secs_f64() * 1000.0,
                decodes.len()
            );
        }
    } else {
        eprintln!("skipping golden WAV: WSJT-X sample not found (sibling checkout)");
    }

    // ── Representative sweep cells (near the measured 50%-crossing SNRs,
    //    FT4_BENCHMARK.md / BENCHMARKS.md) — a handful of trials per
    //    channel, not a full re-sweep. Search params match `ft4_snr_sweep`
    //    / `decode_wav_ft4` above (100..3000 Hz, sync_min=0.8, max_cand=50).
    let dir = sweep_dir();
    for (channel, snr_tag) in [
        ("awgn", "m17"),
        ("ccir_good", "m17"),
        ("ccir_moderate", "m15"),
        ("ccir_poor", "m16"),
    ] {
        for trial in 1..=5u32 {
            let path = dir.join(format!("ft4_{channel}_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            measure(
                &format!("{channel} {snr_tag} trial {trial} [OLD]"),
                &audio,
                100.0,
                3000.0,
                0.8,
                50,
                false,
            );
            measure(
                &format!("{channel} {snr_tag} trial {trial} [NEW]"),
                &audio,
                100.0,
                3000.0,
                0.8,
                50,
                true,
            );
        }
    }
}

/// Phase 4 calibration diagnostic (dapper-soaring-nest plan): gathers the
/// `ft4_sync_search` coherent `score` distribution (WSJT-X's `smax`
/// equivalent) alongside decode outcome, across the AWGN/CCIR
/// near-crossing region, using the *current production* candidate
/// generator (`ft4_coarse_sync`) — not the retired generic
/// `engine::sync::coarse_sync`. Goal: find a `score` cutoff low enough
/// that zero golden-succeeding candidates ever fall below it (so a
/// WSJT-X-style `if(smax < cutoff) cycle` early-exit before
/// symbol_spectra/LLR/BP/OSD costs nothing in recall), following the
/// same "measure before picking a number" discipline as
/// `FT4_BENCHMARK.md` section 6 rather than guessing a translated
/// constant from WSJT-X's own `1.2`.
#[test]
#[ignore = "manual diagnostic — Phase 4 smax early-reject calibration (dapper-soaring-nest plan)"]
fn ft4_diag_smax_calibration() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync2d::ft4_sync_search;
    use mfsk_core::ft4::Ft4;
    use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    const SYNC_Q_MIN: u32 = 8;

    let dir = sweep_dir();

    // Flatten (channel, snr_tag, trial) into one work list so the whole
    // grid — not just the per-candidate loop inside a trial — fans out
    // across cores. Each entry is independent (its own WAV, own
    // candidate list), so this is embarrassingly parallel; the same
    // `#[cfg(feature = "parallel")]` / rayon `par_iter` convention
    // `ft4_snr_sweep` already uses above.
    let mut work: Vec<(&str, &str, u32)> = Vec::new();
    for (channel, snr_tags) in [
        ("awgn", &["m19", "m18", "m17", "m16", "m15"][..]),
        ("ccir_good", &["m19", "m18", "m17", "m16"][..]),
        ("ccir_moderate", &["m18", "m17", "m16", "m15"][..]),
        ("ccir_poor", &["m18", "m17", "m16", "m15"][..]),
    ] {
        for &snr_tag in snr_tags {
            for trial in 1..=20u32 {
                work.push((channel, snr_tag, trial));
            }
        }
    }

    let process_one = |&(channel, snr_tag, trial): &(&str, &str, u32)| -> (Vec<f32>, Vec<f32>) {
        let path = dir.join(format!("ft4_{channel}_{snr_tag}_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            return (Vec::new(), Vec::new());
        };
        let cands = ft4_coarse_sync(&audio, 100.0, 3000.0, 0.8, None, 50);
        if cands.is_empty() {
            return (Vec::new(), Vec::new());
        }
        let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
        let mut golden = Vec::new();
        let mut other = Vec::new();
        for c in &cands {
            let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT4_DOWNSAMPLE);
            let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for z in cd0.iter_mut() {
                    *z *= inv;
                }
            }
            let s2 = ft4_sync_search::<Ft4>(&cd0, c);

            let r = process_candidate_basic::<Ft4>(
                c,
                &fft_cache,
                &FT4_DOWNSAMPLE,
                DecodeDepth::FULL,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
                SYNC_Q_MIN,
            );
            let is_golden = r.as_ref().is_some_and(|d| {
                let mut m77 = [0u8; 77];
                m77.copy_from_slice(&d.info[..77]);
                unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
            });
            if is_golden {
                golden.push(s2.score);
            } else {
                other.push(s2.score);
            }
        }
        (golden, other)
    };

    #[cfg(feature = "parallel")]
    let per_file: Vec<(Vec<f32>, Vec<f32>)> = work.par_iter().map(process_one).collect();
    #[cfg(not(feature = "parallel"))]
    let per_file: Vec<(Vec<f32>, Vec<f32>)> = work.iter().map(process_one).collect();

    let mut golden_scores: Vec<f32> = Vec::new();
    let mut other_scores: Vec<f32> = Vec::new();
    for (g, o) in per_file {
        golden_scores.extend(g);
        other_scores.extend(o);
    }

    golden_scores.sort_by(|a, b| a.partial_cmp(b).unwrap());
    other_scores.sort_by(|a, b| a.partial_cmp(b).unwrap());
    eprintln!(
        "golden-succeeding candidates: n={}, min score={:.4}, 1st%={:.4}, median={:.4}",
        golden_scores.len(),
        golden_scores.first().copied().unwrap_or(f32::NAN),
        golden_scores
            .get(golden_scores.len() / 100)
            .copied()
            .unwrap_or(f32::NAN),
        golden_scores
            .get(golden_scores.len() / 2)
            .copied()
            .unwrap_or(f32::NAN),
    );
    eprintln!(
        "non-golden candidates: n={}, below golden-min: {}",
        other_scores.len(),
        other_scores
            .iter()
            .filter(|&&s| s < golden_scores.first().copied().unwrap_or(f32::NEG_INFINITY))
            .count()
    );
    // Histogram of the lowest 30 golden scores — the tail that determines
    // how tight a safe cutoff can be.
    eprintln!(
        "lowest 30 golden scores: {:?}",
        &golden_scores[..30.min(golden_scores.len())]
    );

    let below_200 = other_scores.iter().filter(|&&s| s < 200.0).count();
    eprintln!(
        "non-golden scores below FT4_SMAX_MIN=200.0: {}/{} ({:.1}%)",
        below_200,
        other_scores.len(),
        100.0 * below_200 as f32 / other_scores.len() as f32
    );
}
