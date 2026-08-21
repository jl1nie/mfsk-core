//! Is `max_cand = 50` the wideband monitor's recall gap? — issue #307,
//! tier C.
//!
//! `fst4_monitor_cap_sensitivity` measured the wideband DDC monitor at
//! 65/100 at −27 dB against an 80/100 production baseline, and
//! `fst4_ddc_sniper_full_decode` had already isolated the DDC front end
//! itself at that SNR (77 vs 80 real). So the DDC explains ~3 of the
//! ~15, and the leading hypothesis for the rest is **candidate-slot
//! competition**: the sniper searched ±250 Hz, the monitor searches
//! ±1450 Hz — 5.8× the band for the same 50 slots — so a weak signal
//! has to out-score far more noise to survive `rank_candidates`'
//! truncation at all.
//!
//! Rather than sweep `max_cand` and re-run the decode for each value,
//! this walks a **deliberately over-long** candidate list once per
//! trial and records the rank at which the golden message first
//! decodes. Recall at any truncation point is then just a count over
//! that rank distribution, so one pass answers the whole curve — and
//! the distribution itself says *why*, which a recall-only sweep would
//! not.
//!
//! Reading the result: if truncation is the cause, ranks pile up beyond
//! 50 and recall keeps climbing past `max_cand = 50`. If instead the
//! golden simply never decodes at any rank, the gap is upstream — the
//! DDC front end or the candidate *scoring*, not the slot count.
//!
//! `#[ignore]` — tier C, needs the generated corpus:
//!
//! ```sh
//! scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!   --test fst4_wideband_max_cand -- --ignored --nocapture
//! ```

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use num_complex::Complex;

use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{
    REFINE_DS_RATE_HZ, grid_for, wideband_cascade, wideband_refine_recenter,
};
use mfsk_core::fst4::rung_major::{RungMajorCandidate, decode_phase_split_timed};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const CENTER_HZ: f32 = 1550.0;
const HALF_WIDTH_HZ: f32 = 1450.0;
const GRID_BW_HZ: f32 = 2900.0;
const SYNC_MIN: f32 = 0.8;

/// The shipped value, and the one under suspicion.
const PROD_MAX_CAND: usize = 50;
/// Deliberately over-long, so the rank distribution is visible past the
/// production cutoff rather than censored by it.
const PROBE_MAX_CAND: usize = 200;
/// Truncation points the recall curve is reported at.
const CUTOFFS: [usize; 6] = [1, 2, 4, 8, 50, 200];

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
        if parts.len() != 5 || parts[0] != "fst4" || parts[1] != "60" || parts[2] != "awgn" {
            continue;
        }
        let Some(rest) = parts[3].strip_prefix('m') else {
            continue;
        };
        let Ok(v) = rest.parse::<i32>() else { continue };
        out.push(WavMeta { snr_db: -v, path });
    }
    out.sort_by_key(|m| -m.snr_db);
    out
}

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

struct Trial {
    baseline: bool,
    /// Rank (0-based) at which the golden message first decoded when
    /// walking the over-long list, or `None` if it never did.
    golden_rank: Option<usize>,
    /// How many candidates `coarse_sync` actually returned — so a
    /// "rank beyond the cutoff" reading can be distinguished from
    /// "there were never that many candidates".
    n_cands: usize,
}

fn run_trial(audio: &[i16]) -> Trial {
    let baseline = DecodeRequest::<Fst4s60>::new(audio, 100.0, 3000.0, SYNC_MIN, PROD_MAX_CAND)
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
    let mut ddc = mfsk_core::engine::dsp::ddc::StreamingComplexDdc::new(&cfg);
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
        PROBE_MAX_CAND,
        RxGrid::complex(grid.fs_c, CENTER_HZ),
    );
    let n_cands = cands.len();

    let mut golden_rank = None;
    for (rank, c) in cands.iter().enumerate() {
        let cd0 = ddc_refine(c, &ci, &cq, delay);
        let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
        let inputs = [RungMajorCandidate {
            cand: c.clone(),
            cd0,
            refined_freq_hz: s2.freq_hz,
            i0: s2.i0,
        }];
        // Uncapped: this measures slot competition, not ladder depth —
        // `fst4_monitor_cap_sensitivity` already covers the latter.
        let (out, _) =
            decode_phase_split_timed::<Fst4s60>(&inputs, false, false, &[0], None, None, 12_000.0);
        let decoded = out.first().and_then(|o| o.as_ref()).and_then(|r| {
            r.message77()
                .try_into()
                .ok()
                .and_then(|m: &[u8; 77]| unpack77(m))
        });
        if decoded.as_deref() == Some(GOLDEN_MSG) {
            golden_rank = Some(rank);
            break;
        }
    }

    Trial {
        baseline,
        golden_rank,
        n_cands,
    }
}

#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_wideband_recall_vs_max_cand() {
    let dir = sweep_dir();
    let wavs = collect_wavs(&dir);
    if wavs.is_empty() {
        eprintln!(
            "No fst4_60_awgn_*.wav in {dir:?}\n\
             Run: scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh"
        );
        return;
    }

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;

    let mut groups: BTreeMap<i32, Vec<&WavMeta>> = BTreeMap::new();
    for w in &wavs {
        // Threshold region only — the question is about weak signals
        // losing slot competitions.
        if (-28..=-26).contains(&w.snr_db) {
            groups.entry(w.snr_db).or_default().push(w);
        }
    }

    eprintln!("\nFST4-60 AWGN wideband — recall vs max_cand truncation point");
    eprintln!("walking a {PROBE_MAX_CAND}-deep list; production is {PROD_MAX_CAND}\n");
    eprintln!(
        "  {:>7}  {:>5}  {:>9}  {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}  {:>12}  {:>9}",
        "SNR(dB)",
        "n",
        "baseline",
        "<=10",
        "<=25",
        "<=50",
        "<=75",
        "<=100",
        "<=200",
        "median rank",
        "med #cand"
    );
    eprintln!("{:-<108}", "");

    for (snr, group) in &groups {
        #[cfg(feature = "parallel")]
        let trials: Vec<Trial> = group
            .par_iter()
            .filter_map(|w| load_wav_i16_opt(&w.path).map(|a| run_trial(&a)))
            .collect();
        #[cfg(not(feature = "parallel"))]
        let trials: Vec<Trial> = group
            .iter()
            .filter_map(|w| load_wav_i16_opt(&w.path).map(|a| run_trial(&a)))
            .collect();

        let n = trials.len();
        if n == 0 {
            continue;
        }
        let base = trials.iter().filter(|t| t.baseline).count();
        let at: Vec<usize> = CUTOFFS
            .iter()
            .map(|&c| {
                trials
                    .iter()
                    .filter(|t| t.golden_rank.is_some_and(|r| r < c))
                    .count()
            })
            .collect();

        let mut ranks: Vec<usize> = trials.iter().filter_map(|t| t.golden_rank).collect();
        ranks.sort_unstable();
        let med_rank = if ranks.is_empty() {
            String::from("-")
        } else {
            format!("{}", ranks[ranks.len() / 2] + 1)
        };
        let mut ns: Vec<usize> = trials.iter().map(|t| t.n_cands).collect();
        ns.sort_unstable();
        let med_n = ns[ns.len() / 2];

        eprintln!(
            "  {snr:>7}  {n:>5}  {:>4}/{:<4}  {:>3}/{:<2} {:>3}/{:<2} {:>3}/{:<2} {:>3}/{:<2} {:>3}/{:<2} {:>3}/{:<2}  {:>12}  {:>9}",
            base, n, at[0], n, at[1], n, at[2], n, at[3], n, at[4], n, at[5], n, med_rank, med_n,
        );
    }
    eprintln!("{:-<108}", "");
    eprintln!(
        "\nIf recall keeps climbing past <=50, slot truncation is the gap.\n\
         If it plateaus at <=50 well under baseline, the gap is upstream\n\
         (front end or candidate scoring), not the slot count."
    );
}
