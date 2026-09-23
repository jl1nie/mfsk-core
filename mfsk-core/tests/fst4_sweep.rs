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

fn decode_wav_fst4_60(audio: &[i16]) -> bool {
    use mfsk_core::fst4::Fst4s60;
    decode_wav_fst4::<Fst4s60>(audio)
}

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

    let mut csv = common::sweep_csv_writer("MFSK_FST4_SWEEP_CSV", "mode,channel,snr_db,trial,pass");

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

/// Regression: `coarse_sync`'s de-duplication decision must be final.
///
/// The dedup loop marks the losing near-duplicate (within 4 Hz / 40 ms)
/// with `score = 0.0`, but the `retain` that follows admits any candidate
/// satisfying `stage1_pass(fi)` *regardless of score* — and `stage1_norm`
/// is only populated for FST4. So on FST4 a candidate the dedup had just
/// rejected could come back, sort to the bottom of its group, and still
/// occupy a slot once `max_cand` truncated the list (issue #312, VK3NV).
///
/// Measured before the fix: 3 of the 50 slots at the production
/// `max_cand = 50` on the golden's K9KFR target, all three inside
/// `rank_candidates`' reserved near-`freq_hint` group; and across 80
/// near-threshold sweep trials, **not one of them ever decoded**
/// (`fst4_60_diag_dedup_zero_score_recall_effect`).
///
/// A `score` of exactly `0.0` in `coarse_sync`'s output can only come
/// from that dedup assignment — real scores are normalised sync ratios —
/// so this asserts the observable property directly.
#[test]
fn fst4_coarse_sync_output_has_no_deduped_candidates() {
    use mfsk_core::engine::sync::{AudioSource, RxGrid, coarse_sync};
    use mfsk_core::fst4::Fst4s60;

    let Some(path) = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    ) else {
        common::skip_or_fail("skip: FST4-60 golden not vendored");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV must load");

    // Both shapes: sniper (freq_hint set, where a zero could take a
    // reserved slot) and wideband (no hint, what `DecodeRequest` uses).
    for (name, lo, hi, hint) in [
        ("sniper/K9KFR", 1081.0, 1581.0, Some(1331.0)),
        ("wideband", 100.0, 3000.0, None),
    ] {
        for cap in [4usize, 16, 50, 200] {
            let out = coarse_sync::<Fst4s60>(
                AudioSource::Real(&audio),
                lo,
                hi,
                0.8,
                hint,
                cap,
                RxGrid::real(12_000.0),
            );
            let zeros = out.iter().filter(|c| c.score == 0.0).count();
            assert_eq!(
                zeros,
                0,
                "{name} @max_cand={cap}: {zeros} de-duplicated candidate(s) survived \
                 into the capped list of {} — coarse_sync's dedup decision must be \
                 final (issue #312)",
                out.len()
            );
        }
    }
}

/// The two schedules must return **exactly the same decodes**.
///
/// [`Schedule::RungMajor`] and [`Schedule::PhaseSplit`] visit the same
/// set of (candidate, offset, sub-stage) units and do the same total
/// work — they differ only in the order, and therefore only under a
/// budget. With no budget gate, any difference in the result is a bug in
/// the split, not a property of it.
///
/// This is the guard that makes the phase split safe to adopt: the
/// scheduling change is supposed to move *when* decodes appear, never
/// *whether* they do.
///
/// Checked on the real FST4-60 golden and across several offset
/// configurations, since the phase boundary (`offset_idx == 0 &&
/// substage == 0` is Phase A) is exactly where an off-by-one would hide.
#[test]
fn fst4_phase_split_matches_rung_major_without_a_budget() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::pipeline::refine_candidate_position;
    use mfsk_core::engine::sync::{AudioSource, RxGrid, coarse_sync};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
    use mfsk_core::fst4::rung_major::{
        RungMajorCandidate, decode_phase_split_timed, decode_rung_major_timed,
    };

    let Some(path) = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    ) else {
        common::skip_or_fail("skip: real FST4-60 golden WAV not vendored/found");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV must load");
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    let raw = coarse_sync::<Fst4s60>(
        AudioSource::Real(&audio),
        100.0,
        3000.0,
        1.2,
        None,
        50,
        RxGrid::real(12_000.0),
    );
    let cands: Vec<RungMajorCandidate> = raw
        .iter()
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
    assert!(!cands.is_empty(), "golden must produce candidates");

    for offsets in [&[0i32][..], &[0, -1][..], &[0, 1, -1][..]] {
        for skip_llrc in [false, true] {
            let (a, _) = decode_rung_major_timed::<Fst4s60>(
                &cands, skip_llrc, false, offsets, None, 12_000.0,
            );
            let (b, _) = decode_phase_split_timed::<Fst4s60>(
                &cands, skip_llrc, false, offsets, None, None, 12_000.0,
            );
            assert_eq!(
                a.len(),
                b.len(),
                "offsets={offsets:?} skip_llrc={skip_llrc}: result length differs"
            );
            for (i, (x, y)) in a.iter().zip(&b).enumerate() {
                match (x, y) {
                    (None, None) => {}
                    (Some(x), Some(y)) => assert_eq!(
                        x.info, y.info,
                        "offsets={offsets:?} skip_llrc={skip_llrc} cand {i}: \
                         different message decoded"
                    ),
                    _ => panic!(
                        "offsets={offsets:?} skip_llrc={skip_llrc} cand {i}: one schedule \
                         decoded and the other did not ({} vs {})",
                        x.is_some(),
                        y.is_some()
                    ),
                }
            }
        }
    }
}
