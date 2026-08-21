//! Does `llrd` contribute recall on the **DDC** path? — issue #307,
//! tier C.
//!
//! `fst4::rung_major` omits `llrd` outright, on the strength of
//! `fst4_sweep.rs::fst4_60_diag_stage_ablation[_ccir_moderate]` finding
//! it "never contributes additional recall on real AWGN or
//! CCIR-moderate data". `fst4_rung_major_recall_gap` then measured the
//! wideband DDC monitor at 65/100 at −27 dB through `rung_major`
//! against 75/100 through the production ladder on the *same candidate
//! list* — a ~10/100 gap that the omitted stage is the obvious
//! suspect for.
//!
//! But "obvious suspect" is not a measurement, and `rung_major`
//! reimplements the whole stage sequence, so the gap could equally be
//! ordering, the escalation gates, or something else entirely. This
//! isolates `llrd` specifically.
//!
//! ## Why a second ablation rather than trusting the first
//!
//! The existing ablation runs on the **real 12 kHz path**
//! (`AudioSource::Real` + `downsample_cached`). The monitor runs on a
//! DDC baseband: a different front end, a different resampling
//! history, and therefore different LLR statistics feeding the same
//! decoder. A stage that is redundant on one need not be on the other,
//! which is exactly the kind of assumption this investigation has
//! already been wrong about three times (#333 PSRAM, #338 `max_cand`,
//! #339 the cascade response).
//!
//! Method mirrors `stage_ablation_for_channel` deliberately — same
//! four `(keep_llrc, keep_llrd)` configs, same BP-then-OSD escalation
//! against the same gates, same oracle filter to the golden candidate
//! so candidate *selection* cannot contaminate a stage question. The
//! only thing swapped is the front end.
//!
//! `#[ignore]` — tier C, needs the generated corpus:
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!   --test fst4_llrd_ablation_ddc -- --ignored --nocapture
//! ```

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use num_complex::Complex;

use mfsk_core::engine::llr::{compute_llr, symbol_spectra, sync_quality};
use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::{freq_shift_cd0, fst4_sync_search};
use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{
    REFINE_DS_RATE_HZ, grid_for, wideband_cascade, wideband_refine_recenter,
};
use mfsk_core::msg::wsjt77::unpack77;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
/// Oracle window, matching `fst4_sweep`'s own `FREQ_TOL_HZ` usage in
/// the ablation this mirrors.
const FREQ_TOL_HZ: f32 = 5.0;
const CENTER_HZ: f32 = 1550.0;
const HALF_WIDTH_HZ: f32 = 1450.0;
const GRID_BW_HZ: f32 = 2900.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 50;
const BP_MAX_ITER: u32 = 30;

const SNR_TAGS: &[(&str, u32)] = &[("m26", 100), ("m27", 100)];

/// `llra`/`llrb`/`llre` are always kept — no one has proposed dropping
/// the cheap rungs, and the existing ablation makes the same choice.
const CONFIGS: [(bool, bool); 4] = [
    (true, true),   // full: production's actual 5-variant set
    (true, false),  // full minus llrd  <- what rung_major runs
    (false, true),  // no8_osd shape
    (false, false), // no8_osd minus llrd
];
const CONFIG_NAMES: [&str; 4] = ["full", "full-llrd", "no8_osd", "no8_osd-llrd"];

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_FST4_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf()
}

fn ddc_refine(c: &SyncCandidate, ci: &[f32], cq: &[f32], delay: usize) -> Vec<Complex<f32>> {
    let mut r = wideband_refine_recenter(c.freq_hz, CENTER_HZ);
    let (mut a, mut b) = (Vec::new(), Vec::new());
    r.push(ci, cq, &mut a, &mut b);
    r.flush(&mut a, &mut b);
    let trim = ((delay as f32 * REFINE_DS_RATE_HZ / 12_000.0).round() as usize
        + r.group_delay_output_samples())
    .min(a.len());
    let mut cd0: Vec<Complex<f32>> = a[trim..]
        .iter()
        .zip(b[trim..].iter())
        .map(|(&i, &q)| Complex::new(i, q))
        .collect();
    let s2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if s2 > f32::EPSILON {
        let inv = 1.0 / s2.sqrt();
        for z in cd0.iter_mut() {
            *z *= inv;
        }
    }
    cd0
}

#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_llrd_ablation_on_the_ddc_path() {
    let dir = sweep_dir();
    let (osd_attempt_min, osd_depth3_min) =
        mfsk_core::engine::pipeline::osd_escalation_gates::<Fst4s60>();

    let mut work: Vec<(usize, &str, u32)> = Vec::new();
    for (i, &(tag, trials)) in SNR_TAGS.iter().enumerate() {
        for t in 1..=trials {
            work.push((i, tag, t));
        }
    }

    let process_one = |&(snr_idx, tag, trial): &(usize, &str, u32)| -> (usize, [bool; 4]) {
        let path = dir.join(format!("fst4_60_awgn_{tag}_{trial:02}.wav"));
        let Some(audio) = load_wav_i16_opt(&path) else {
            return (snr_idx, [false; 4]);
        };

        let cfg = wideband_cascade(CENTER_HZ);
        let mut ddc = mfsk_core::engine::dsp::ddc::StreamingComplexDdc::new(&cfg);
        let (mut ci, mut cq) = (Vec::new(), Vec::new());
        ddc.push_i16(&audio, &mut ci, &mut cq);
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

        let fec = <Fst4s60 as Protocol>::Fec::default();
        let verify_info =
            Some(<<Fst4s60 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);
        let mut ok = [false; 4];

        for c in cands
            .iter()
            .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
        {
            let cd0 = ddc_refine(c, &ci, &cq, delay);
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
            let cd0 = freq_shift_cd0(&cd0, s2.freq_hz - c.freq_hz, REFINE_DS_RATE_HZ);
            let cs = symbol_spectra::<Fst4s60>(&cd0, s2.i0);
            let nsync = sync_quality::<Fst4s60>(&cs);
            let llr_set = compute_llr::<Fst4s60, f32>(&cs);

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
                let mut variants: Vec<&Vec<f32>> =
                    vec![&llr_set.llra, &llr_set.llrb, &llr_set.llre];
                if keep_llrc {
                    variants.push(&llr_set.llrc);
                }
                if keep_llrd {
                    variants.push(&llr_set.llrd);
                }
                if variants.iter().any(|l| is_golden(l, &bp_opts)) {
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
                    if variants.iter().any(|l| is_golden(l, &osd_opts)) {
                        ok[i] = true;
                    }
                }
            }
        }
        (snr_idx, ok)
    };

    #[cfg(feature = "parallel")]
    let results: Vec<(usize, [bool; 4])> = {
        use rayon::prelude::*;
        work.par_iter().map(process_one).collect()
    };
    #[cfg(not(feature = "parallel"))]
    let results: Vec<(usize, [bool; 4])> = work.iter().map(process_one).collect();

    eprintln!("\nFST4-60 AWGN — llrd/llrc stage ablation on the **DDC** path");
    eprintln!("(oracle-filtered to the golden candidate, so this is a stage question only)\n");
    eprintln!(
        "  {:>7}  {:>5}  {:>8} {:>10} {:>8} {:>13}",
        "SNR(dB)", "n", CONFIG_NAMES[0], CONFIG_NAMES[1], CONFIG_NAMES[2], CONFIG_NAMES[3]
    );
    eprintln!("{:-<62}", "");
    for (i, &(tag, _)) in SNR_TAGS.iter().enumerate() {
        let rows: Vec<&(usize, [bool; 4])> = results.iter().filter(|(s, _)| *s == i).collect();
        let n = rows.len();
        if n == 0 {
            continue;
        }
        let hit = |k: usize| rows.iter().filter(|(_, ok)| ok[k]).count();
        eprintln!(
            "  {:>7}  {:>5}  {:>4}/{:<3} {:>6}/{:<3} {:>4}/{:<3} {:>9}/{:<3}",
            tag,
            n,
            hit(0),
            n,
            hit(1),
            n,
            hit(2),
            n,
            hit(3),
            n
        );
    }
    eprintln!("{:-<62}", "");
    eprintln!(
        "\n`full` vs `full-llrd` is the question: a gap means dropping llrd\n\
         costs recall on this path, and `rung_major`'s omission is not free."
    );
}
