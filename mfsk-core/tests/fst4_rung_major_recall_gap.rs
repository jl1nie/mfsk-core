//! Does `decode_phase_split_timed` recall as well as the production
//! ladder at threshold? — issue #307, tier C.
//!
//! `fst4_wideband_max_cand` measured the wideband DDC monitor at 65/100
//! at −27 dB against an 80/100 production baseline and attributed the
//! gap to the wideband front end. **That conclusion was wrong**, and
//! this test is what corrects it: `fst4_ddc_cascade_response` then
//! showed the wideband cascade is flatter through its search band than
//! the sniper's (−0.23 dB at the edge vs −2.87 dB) with ≥40 dB alias
//! rejection, so the filter could not account for it.
//!
//! The real confound was in the comparison itself. The monitor decodes
//! through `fst4::rung_major::decode_phase_split_timed`, which
//! **omits `llrd`** (that module's doc comment says so: five stages
//! llra/llrb/llre/llrc/OSD, not six) on the strength of an ablation
//! finding it "never contributes additional recall on real AWGN or
//! CCIR-moderate data". The baseline runs the production six-variant
//! ladder. So the two differed by a stage, not just by a front end.
//!
//! Holding the candidate list fixed and swapping only the decode path
//! separates them.

#![allow(clippy::type_complexity)]
#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use num_complex::Complex;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{
    REFINE_DS_RATE_HZ, grid_for, wideband_cascade, wideband_refine_recenter,
};
use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
use mfsk_core::fst4::rung_major::{RungMajorCandidate, decode_phase_split_timed};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const CENTER_HZ: f32 = 1550.0;
const HALF: f32 = 1450.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 50;
const SYNC_Q_MIN: u32 = 16;

fn ddc_refine(c: &SyncCandidate, ci: &[f32], cq: &[f32], d: usize) -> Vec<Complex<f32>> {
    let mut r = wideband_refine_recenter(c.freq_hz, CENTER_HZ);
    let (mut a, mut b) = (Vec::new(), Vec::new());
    r.push(ci, cq, &mut a, &mut b);
    r.flush(&mut a, &mut b);
    let t = ((d as f32 * REFINE_DS_RATE_HZ / 12_000.0).round() as usize
        + r.group_delay_output_samples())
    .min(a.len());
    let mut cd0: Vec<Complex<f32>> = a[t..]
        .iter()
        .zip(b[t..].iter())
        .map(|(&i, &q)| Complex::new(i, q))
        .collect();
    let s2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if s2 > f32::EPSILON {
        let inv = 1.0 / s2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
    cd0
}

fn is_golden(m: &[u8]) -> bool {
    m.try_into()
        .ok()
        .and_then(|a: &[u8; 77]| unpack77(a))
        .as_deref()
        == Some(GOLDEN_MSG)
}

/// Measured 2026-08-22, wideband DDC candidates, 100 trials per cell:
///
/// | SNR | baseline | via `rung_major` | via production ladder |
/// |---|---:|---:|---:|
/// | −27 dB | 80/100 | **65** | **75** |
/// | −26 dB | 99/100 | 96 | **100** |
///
/// So at threshold the omitted stage is worth ~10/100, and the DDC
/// front end only ~5 — the latter matching
/// `fst4_ddc_sniper_full_decode`'s independent 77-vs-80. At −26 dB the
/// wideband path on the production ladder matches or beats the real
/// 12 kHz baseline outright.
///
/// This does not by itself prove `llrd` is the responsible stage —
/// `rung_major` reimplements the whole sequence — but it does show the
/// ablation behind that omission does not hold at this operating point,
/// and the embedded monitor pays the difference whenever its cap is on.
#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_rung_major_vs_production_ladder_at_threshold() {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let dir: PathBuf = Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf();

    use rayon::prelude::*;
    for snr in ["m27", "m26"] {
        let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.file_stem()
                    .and_then(|s| s.to_str())
                    .is_some_and(|s| s.starts_with(&format!("fst4_60_awgn_{snr}_")))
            })
            .collect();
        paths.sort();

        let res: Vec<(bool, bool, bool)> = paths
            .par_iter()
            .filter_map(|p| {
                let audio = load_wav_i16_opt(p)?;
                let baseline =
                    DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, SYNC_MIN, MAX_CAND)
                        .decode()
                        .results
                        .iter()
                        .any(|r| is_golden(r.message77()));

                let cfg = wideband_cascade(CENTER_HZ);
                let mut ddc = mfsk_core::engine::dsp::ddc::StreamingComplexDdc::new(&cfg);
                let (mut ci, mut cq) = (Vec::new(), Vec::new());
                ddc.push_i16(&audio, &mut ci, &mut cq);
                ddc.flush(&mut ci, &mut cq);
                let delay = ddc.group_delay_input_samples();
                let grid = grid_for(2900.0);
                let cands = coarse_sync::<Fst4s60>(
                    AudioSource::Complex(&ci, &cq),
                    CENTER_HZ - HALF,
                    CENTER_HZ + HALF,
                    SYNC_MIN,
                    None,
                    MAX_CAND,
                    RxGrid::complex(grid.fs_c, CENTER_HZ),
                );

                let (mut via_rung, mut via_prod) = (false, false);
                for c in cands.iter() {
                    let cd0 = ddc_refine(c, &ci, &cq, delay);
                    let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);

                    if !via_rung {
                        let inputs = [RungMajorCandidate {
                            cand: c.clone(),
                            cd0: cd0.clone(),
                            refined_freq_hz: s2.freq_hz,
                            i0: s2.i0,
                        }];
                        let (o, _) = decode_phase_split_timed::<Fst4s60>(
                            &inputs,
                            false,
                            false,
                            &[0],
                            None,
                            None,
                            12_000.0,
                        );
                        if o.first()
                            .and_then(|x| x.as_ref())
                            .is_some_and(|r| is_golden(r.message77()))
                        {
                            via_rung = true;
                        }
                    }
                    if !via_prod {
                        let pre = (cd0, s2.freq_hz, s2.i0, s2.score);
                        if process_candidate_precomputed::<Fst4s60>(
                            c,
                            &[],
                            &FST4_60A_DOWNSAMPLE,
                            DecodeDepth::FULL,
                            DecodeStrictness::Normal,
                            &[],
                            EqMode::Off,
                            SYNC_Q_MIN,
                            pre,
                            true,
                            false,
                        )
                        .is_some_and(|r| is_golden(r.message77()))
                        {
                            via_prod = true;
                        }
                    }
                    if via_rung && via_prod {
                        break;
                    }
                }
                Some((baseline, via_rung, via_prod))
            })
            .collect();

        let n = res.len();
        eprintln!(
            "{snr}: n={n}  baseline(real 12k)={}  wideband via rung_major(no llrd)={}  wideband via production ladder={}",
            res.iter().filter(|r| r.0).count(),
            res.iter().filter(|r| r.1).count(),
            res.iter().filter(|r| r.2).count(),
        );
    }
}
