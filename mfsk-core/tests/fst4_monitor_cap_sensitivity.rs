//! What does the FST4 monitor's per-candidate decode cap cost at
//! threshold? — issue #307, tier C.
//!
//! The embedded wideband monitor (`fst4_ddc_bench`,
//! `MFSK_FST4_DDC_BENCH_MODE=monitor`) walks `coarse_sync`'s candidates
//! in score order and bounds each one's LLR/BP/OSD ladder with
//! `decode_phase_split_timed`'s `budget_ok`. On real hardware that took
//! band coverage from 13 to 39 candidates in a 45 s deadline at zero
//! cost to the two decodes in that recording.
//!
//! But **that recording carries two strong signals**, so it cannot show
//! what the cap costs a *weak* one — and #310 measured that OSD, the
//! last and most expensive rung, is where most decodes come from. An
//! aggressive cap trades exactly that. This test is the missing half.
//!
//! ## Why the budget is counted in stages, not milliseconds
//!
//! The shipped cap is wall-clock (`MFSK_FST4_DDC_BENCH_CAP_MS`). A host
//! is ~50x faster than the CoreS3, so a host wall-clock cap would
//! measure nothing that transfers. `budget_ok` is polled exactly once
//! before each Phase B/C stage, so counting *stages allowed* is the
//! same knob expressed in a device-independent unit:
//!
//! | budget | ladder reached |
//! |---|---|
//! | 0 | `llra` only (Phase A is never gated — that is the latency invariant) |
//! | 1 | + `llrb` |
//! | 2 | + `llre` |
//! | 3 | + `llrc` |
//! | 4 | + OSD (i.e. the full ladder, equivalent to no cap) |
//!
//! Reading the result: budget 4 is the uncapped reference. Any recall
//! gap between it and a lower budget is what a cap of that depth costs
//! at that SNR. Picking a production `CAP_MS` means choosing a row of
//! this table and then measuring, on hardware, which millisecond value
//! reaches that depth for a real signal.
//!
//! `#[ignore]` — tier C, needs the generated corpus:
//!
//! ```sh
//! scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!   --test fst4_monitor_cap_sensitivity -- --ignored --nocapture
//! ```

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::cell::Cell;
use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use num_complex::Complex;

use mfsk_core::engine::dsp::ddc::StreamingComplexDdc;
use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{
    REFINE_DS_RATE_HZ, grid_for, wideband_cascade, wideband_refine_recenter,
};
use mfsk_core::fst4::rung_major::{RungMajorCandidate, decode_phase_split_timed};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

/// The message the `fst4_60_awgn_*` corpus encodes — matches
/// `tests/fst4_sweep.rs`'s own `GOLDEN_MSG`.
const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
/// Wideband production search band, and the DDC grid that spans it —
/// the same 100-3000 Hz the embedded monitor runs.
const CENTER_HZ: f32 = 1550.0;
const HALF_WIDTH_HZ: f32 = 1450.0;
const GRID_BW_HZ: f32 = 2900.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 50;

/// Stage budgets to sweep. 4 is the uncapped reference.
const BUDGETS: [i32; 5] = [0, 1, 2, 3, 4];

thread_local! {
    /// Remaining Phase B/C stages this candidate may run. A
    /// `thread_local` rather than a global because the trial loop is
    /// rayon-parallel and `budget_ok` must be a plain `fn` pointer
    /// (`decode_phase_split_timed` takes no closure).
    static STAGE_BUDGET: Cell<i32> = const { Cell::new(i32::MAX) };
}

fn budget_ok() -> bool {
    STAGE_BUDGET.with(|b| {
        let v = b.get();
        if v > 0 {
            b.set(v - 1);
            true
        } else {
            false
        }
    })
}

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_FST4_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf()
}

struct WavMeta {
    snr_db: i32,
    path: PathBuf,
}

/// `fst4_60_<channel>_m<snr>_<trial>.wav`. Matching on the whole
/// `fst4_60_<channel>_m` prefix rather than splitting on `_` is what
/// lets one parser serve both `awgn` and the two-word fading channels
/// (`ccir_moderate`).
fn collect_wavs(dir: &Path, channel: &str) -> Vec<WavMeta> {
    let prefix = format!("fst4_60_{channel}_m");
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
        let Some(rest) = stem.strip_prefix(&prefix) else {
            continue;
        };
        let Some((snr, _trial)) = rest.split_once('_') else {
            continue;
        };
        let Ok(v) = snr.parse::<i32>() else { continue };
        out.push(WavMeta { snr_db: -v, path });
    }
    out.sort_by_key(|m| -m.snr_db);
    out
}

/// Mirrors `embedded_shared::apps::fst4_ddc_bench::ddc_refine` — the
/// wideband recentre this measurement is about.
fn ddc_refine(
    cand: &SyncCandidate,
    coarse_i: &[f32],
    coarse_q: &[f32],
    delay: usize,
) -> Vec<Complex<f32>> {
    let mut refine = wideband_refine_recenter(cand.freq_hz, CENTER_HZ);
    let mut cd0_i = Vec::new();
    let mut cd0_q = Vec::new();
    refine.push(coarse_i, coarse_q, &mut cd0_i, &mut cd0_q);
    refine.flush(&mut cd0_i, &mut cd0_q);
    let total_delay_out = (delay as f32 * REFINE_DS_RATE_HZ / 12_000.0).round() as usize
        + refine.group_delay_output_samples();
    let trim = total_delay_out.min(cd0_i.len());

    let mut cd0: Vec<Complex<f32>> = cd0_i[trim..]
        .iter()
        .zip(cd0_q[trim..].iter())
        .map(|(&i, &q)| Complex::new(i, q))
        .collect();
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
    cd0
}

struct TrialOutcome {
    /// Production wide-band path, for reference.
    baseline: bool,
    /// Per budget: did the golden message decode, and at what
    /// coarse-score rank (0-based) if so.
    per_budget: Vec<(bool, Option<usize>)>,
    /// Rank the golden signal sat at in `coarse_sync`'s output, if it
    /// ever decoded at any budget — the ordering half of the question.
    golden_rank: Option<usize>,
}

fn run_trial(audio: &[i16]) -> TrialOutcome {
    let baseline = DecodeRequest::<Fst4s60>::new(audio, 100.0, 3000.0, SYNC_MIN, MAX_CAND)
        .decode()
        .results
        .iter()
        .any(|r| {
            r.message77()
                .try_into()
                .ok()
                .and_then(|m: &[u8; 77]| unpack77(m))
                .as_deref()
                == Some(GOLDEN_MSG)
        });

    let cfg = wideband_cascade(CENTER_HZ);
    let mut ddc = StreamingComplexDdc::new(&cfg);
    let (mut ci, mut cq) = (Vec::new(), Vec::new());
    ddc.push_i16(audio, &mut ci, &mut cq);
    ddc.flush(&mut ci, &mut cq);
    let delay = ddc.group_delay_input_samples();

    let grid = grid_for(GRID_BW_HZ);
    let cands = coarse_sync::<Fst4s60>(
        AudioSource::Complex(&ci, &cq),
        CENTER_HZ - HALF_WIDTH_HZ,
        CENTER_HZ + HALF_WIDTH_HZ,
        SYNC_MIN,
        None,
        MAX_CAND,
        RxGrid::complex(grid.fs_c, CENTER_HZ),
    );

    // Refine every candidate once and reuse across budgets — refine is
    // the expensive half and is budget-independent.
    let refined: Vec<RungMajorCandidate> = cands
        .iter()
        .map(|c| {
            let cd0 = ddc_refine(c, &ci, &cq, delay);
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            RungMajorCandidate {
                cand: c.clone(),
                cd0,
                refined_freq_hz: s2.freq_hz,
                i0: s2.i0,
            }
        })
        .collect();

    let mut per_budget = Vec::with_capacity(BUDGETS.len());
    let mut golden_rank = None;
    for &b in &BUDGETS {
        let mut hit = None;
        for (rank, rc) in refined.iter().enumerate() {
            STAGE_BUDGET.with(|c| c.set(b));
            let inputs = [RungMajorCandidate {
                cand: rc.cand.clone(),
                cd0: rc.cd0.clone(),
                refined_freq_hz: rc.refined_freq_hz,
                i0: rc.i0,
            }];
            let (out, _) = decode_phase_split_timed::<Fst4s60>(
                &inputs,
                false,
                false,
                &[0],
                None,
                Some(budget_ok),
                12_000.0,
            );
            let decoded = out.first().and_then(|o| o.as_ref()).and_then(|r| {
                r.message77()
                    .try_into()
                    .ok()
                    .and_then(|m: &[u8; 77]| unpack77(m))
            });
            if decoded.as_deref() == Some(GOLDEN_MSG) {
                hit = Some(rank);
                golden_rank.get_or_insert(rank);
                break;
            }
        }
        per_budget.push((hit.is_some(), hit));
    }

    TrialOutcome {
        baseline,
        per_budget,
        golden_rank,
    }
}

/// AWGN, the channel every monitor design decision was taken on.
#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_monitor_cap_recall_vs_stage_budget() {
    // Threshold region only — above -24 dB everything decodes at every
    // budget and the rows carry no information.
    run_budget_sweep("awgn", i32::MIN, -23);
}

/// The same measurement under CCIR-moderate fading (0.5 Hz Doppler
/// spread, 1.0 ms delay spread), which is where a real HF path lives
/// and where none of the monitor work had been measured.
///
/// Its threshold sits ~2.5 dB above AWGN's, so the SNR window differs.
/// Measured production recall on this corpus (`fst4_sweep`,
/// `MFSK_FST4_SWEEP_CHANNELS=ccir_moderate`, 100 trials/cell): -27 dB
/// 1%, -26 dB 20%, -25 dB 56%, -24 dB 85%, -23 dB 96%. So -26..-24 is
/// this channel's threshold region, the way -28..-26 is AWGN's; at
/// -27 dB every row would read zero.
#[test]
#[ignore = "tier C — needs the fst4_60_ccir_moderate_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_monitor_cap_recall_vs_stage_budget_ccir_moderate() {
    run_budget_sweep("ccir_moderate", -26, -24);
}

/// `snr_lo`/`snr_hi` are inclusive dB bounds on which corpus cells to
/// run — the threshold region for that channel, since above it every
/// budget decodes everything and below it none of them decode anything.
fn run_budget_sweep(channel: &str, snr_lo: i32, snr_hi: i32) {
    let dir = sweep_dir();
    let wavs = collect_wavs(&dir, channel);
    if wavs.is_empty() {
        eprintln!(
            "No fst4_60_{channel}_*.wav in {dir:?}\n\
             Run: scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh"
        );
        return;
    }

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;

    let mut groups: BTreeMap<i32, Vec<&WavMeta>> = BTreeMap::new();
    for w in &wavs {
        if (snr_lo..=snr_hi).contains(&w.snr_db) {
            groups.entry(w.snr_db).or_default().push(w);
        }
    }

    eprintln!(
        "\nFST4-60 {} — monitor recall vs per-candidate stage budget",
        channel.to_uppercase()
    );
    eprintln!("budget 4 = uncapped reference; 0 = llra only\n");
    eprintln!(
        "  {:>7}  {:>5}  {:>9}  {:>7} {:>7} {:>7} {:>7} {:>7}  {:>10}",
        "SNR(dB)", "n", "baseline", "b=0", "b=1", "b=2", "b=3", "b=4", "median rank"
    );
    eprintln!("{:-<86}", "");

    for (snr, group) in &groups {
        #[cfg(feature = "parallel")]
        let outcomes: Vec<TrialOutcome> = group
            .par_iter()
            .filter_map(|w| load_wav_i16_opt(&w.path).map(|a| run_trial(&a)))
            .collect();
        #[cfg(not(feature = "parallel"))]
        let outcomes: Vec<TrialOutcome> = group
            .iter()
            .filter_map(|w| load_wav_i16_opt(&w.path).map(|a| run_trial(&a)))
            .collect();

        let n = outcomes.len();
        if n == 0 {
            continue;
        }
        let base = outcomes.iter().filter(|o| o.baseline).count();
        let per: Vec<usize> = (0..BUDGETS.len())
            .map(|i| outcomes.iter().filter(|o| o.per_budget[i].0).count())
            .collect();

        let mut ranks: Vec<usize> = outcomes.iter().filter_map(|o| o.golden_rank).collect();
        ranks.sort_unstable();
        let median_rank = if ranks.is_empty() {
            String::from("-")
        } else {
            format!("{}", ranks[ranks.len() / 2] + 1)
        };

        eprintln!(
            "  {snr:>7}  {n:>5}  {:>4}/{:<4}  {:>3}/{:<3} {:>3}/{:<3} {:>3}/{:<3} {:>3}/{:<3} {:>3}/{:<3}  {:>10}",
            base, n, per[0], n, per[1], n, per[2], n, per[3], n, per[4], n, median_rank,
        );
    }
    eprintln!("{:-<86}", "");
}
