//! Where the wideband monitor's threshold recall actually goes —
//! issue #307/#308/#310, tier C.
//!
//! Chases one number to ground through three wrong answers, which is
//! why it exists as a test rather than a note: **the monitor's 65/100
//! at −27 dB against an 80/100 production baseline is the `i0 ± 1`
//! timing retry, and nothing else.**
//!
//! The eliminations, in order:
//!
//! 1. **`max_cand` truncation** (`fst4_wideband_max_cand`) — recall
//!    plateaus by rank 25; 200 slots buy three decodes over 50.
//! 2. **The wideband front end** (`fst4_ddc_cascade_response`) — that
//!    cascade is *flatter* through its search band than the sniper's
//!    (−0.23 dB vs −2.87 dB at the respective edges), with ≥40 dB alias
//!    rejection.
//! 3. **The omitted `llrd` stage** (`fst4_llrd_ablation_ddc`) — the
//!    obvious suspect, since `rung_major` drops it. Ablated directly on
//!    the DDC path: identical with and without, 96/96 and 65/65, just
//!    as the pre-existing real-path ablation found.
//!
//! What is left is timing diversity. `engine::pipeline`'s
//! `process_candidate_basic_impl` retries `i0 + 1` and `i0 - 1`
//! whenever OSD depth is enabled (`pipeline.rs:1236`), ported by #308.
//! `rung_major` makes that the caller's choice, and **every embedded
//! caller passes `&[0]`** — which is exactly the gap #310 names when it
//! says "embedded currently gets zero timing diversity".
//!
//! Measured here, wideband DDC candidates, 100 trials per cell:
//!
//! | SNR | baseline | `rung_major` `[0]` | `rung_major` `[0,1,-1]` | production |
//! |---|---:|---:|---:|---:|
//! | −27 dB | 80/100 | **65** | **75** | **75** |
//! | −26 dB | 99/100 | 96 | **100** | **100** |
//!
//! With timing diversity restored, `rung_major` matches the production
//! ladder exactly at both SNRs — so the scheduler is faithful and only
//! its `offsets` argument was costing recall. It also independently
//! reproduces `rung_major`'s own documented `&[0]` → `&[0, 1, -1]`
//! table (74 → 82 on AWGN m27 there; 65 → 75 here on the DDC path).
//!
//! The remaining ~5 at −27 dB (75 vs 80) is the DDC front end, matching
//! `fst4_ddc_sniper_full_decode`'s independent 77-vs-80. At −26 dB the
//! DDC path matches the real 12 kHz baseline outright.
//!
//! ## What this costs the monitor
//!
//! `rung_major`'s own cost table puts `&[0, 1, -1]` at 2.83× host and
//! 2.51–3.02× on real hardware. At the monitor's measured 627 ms per
//! candidate that is ~1.6–1.9 s, so covering all 50 candidates would go
//! 31.4 s → ~85 s and no longer fit a 60 s slot period.
//!
//! But it does not have to be uniform. `fst4_monitor_cap_sensitivity`
//! and `fst4_wideband_max_cand` both measured the golden signal at
//! **median coarse-score rank 1** at every SNR, so timing diversity is
//! only needed where the signal actually is: full `&[0, 1, -1]` for the
//! first few candidates, `&[0]` for the tail. The first decode would
//! then land at ~4039 + ~1900 ms ≈ 5.9 s post-slot — still inside
//! FST4-60's 7.2 s guard.

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

/// See the module doc comment for the full elimination chain and the
/// measured table.
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

        let res: Vec<(bool, bool, bool, bool)> = paths
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

                let (mut via_rung, mut via_prod, mut via_rung_off) = (false, false, false);
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
                        let pre = (cd0.clone(), s2.freq_hz, s2.i0, s2.score);
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
                    if !via_rung_off {
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
                            &[0, 1, -1],
                            None,
                            None,
                            12_000.0,
                        );
                        if o.first()
                            .and_then(|x| x.as_ref())
                            .is_some_and(|r| is_golden(r.message77()))
                        {
                            via_rung_off = true;
                        }
                    }
                    if via_rung && via_prod && via_rung_off {
                        break;
                    }
                }
                Some((baseline, via_rung, via_prod, via_rung_off))
            })
            .collect();

        let n = res.len();
        eprintln!(
            "{snr}: n={n}  baseline(real 12k)={}  rung_major offsets=[0]={}  \
             rung_major offsets=[0,1,-1]={}  production ladder={}",
            res.iter().filter(|r| r.0).count(),
            res.iter().filter(|r| r.1).count(),
            res.iter().filter(|r| r.3).count(),
            res.iter().filter(|r| r.2).count(),
        );
    }
}

/// One trial, four arms, on one shared DDC candidate list: the
/// production baseline on real 12 kHz audio, `rung_major` at
/// `offsets = &[0]` (what every embedded caller runs) and at
/// `&[0, 1, -1]`, and the production ladder on the same candidates.
///
/// Extracted so
/// [`fst4_60_ccir_moderate_timing_x_pruning`] can run it twice per
/// cell, once per OSD search.
fn trial_arms(audio: &[i16]) -> (bool, bool, bool, bool) {
    let baseline = DecodeRequest::<Fst4s60>::new(audio, 100.0, 3000.0, SYNC_MIN, MAX_CAND)
        .decode()
        .results
        .iter()
        .any(|r| is_golden(r.message77()));

    let cfg = wideband_cascade(CENTER_HZ);
    let mut ddc = mfsk_core::engine::dsp::ddc::StreamingComplexDdc::new(&cfg);
    let (mut ci, mut cq) = (Vec::new(), Vec::new());
    ddc.push_i16(audio, &mut ci, &mut cq);
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

    let (mut via_rung, mut via_prod, mut via_rung_off) = (false, false, false);
    for c in cands.iter() {
        let cd0 = ddc_refine(c, &ci, &cq, delay);
        let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
        let mk = || RungMajorCandidate {
            cand: c.clone(),
            cd0: cd0.clone(),
            refined_freq_hz: s2.freq_hz,
            i0: s2.i0,
        };

        if !via_rung {
            let (o, _) = decode_phase_split_timed::<Fst4s60>(
                &[mk()],
                false,
                false,
                &[0],
                None,
                None,
                12_000.0,
            );
            via_rung = o
                .first()
                .and_then(|x| x.as_ref())
                .is_some_and(|r| is_golden(r.message77()));
        }
        if !via_rung_off {
            let (o, _) = decode_phase_split_timed::<Fst4s60>(
                &[mk()],
                false,
                false,
                &[0, 1, -1],
                None,
                None,
                12_000.0,
            );
            via_rung_off = o
                .first()
                .and_then(|x| x.as_ref())
                .is_some_and(|r| is_golden(r.message77()));
        }
        if !via_prod {
            let pre = (cd0.clone(), s2.freq_hz, s2.i0, s2.score);
            via_prod = process_candidate_precomputed::<Fst4s60>(
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
            .is_some_and(|r| is_golden(r.message77()));
        }
        if via_rung && via_prod && via_rung_off {
            break;
        }
    }
    (baseline, via_rung, via_rung_off, via_prod)
}

/// Do the monitor's fading recall gap and #311's unexplained
/// `npre2`-pruning misses share a mechanism? — issue #311/#343.
///
/// The two are different by construction — #311's residual 3 of 5 is
/// *defined* as what timing diversity failed to explain, and the
/// monitor's `&[0]` gap is timing diversity — but that only rules out
/// identity, not a common cause. The version worth testing is whether
/// they interact: if fading works by making success depend on getting
/// enough independent tries, then every mechanism that removes tries
/// (one `i0`, a pruned OSD pattern set) should cost more here than on
/// AWGN, and the two should partly substitute for each other.
///
/// So: the 2x2, on the monitor's own DDC path, at 100 trials per cell.
/// `fst4_osd_diag_force_old` is a process-global toggle, so each cell
/// runs as two rayon-parallel passes with the flag set *between* them
/// rather than per trial — which is also why this cannot share a
/// process with another FST4-decoding test.
///
/// This is #311's own ablation at the n it asked for. That one ran at
/// n=20 ("the n=100 run that would settle it needs `fst4sim`, which
/// isn't installed on the machine this ran on") and reported
/// single-digit rescue counts it called directional only; the
/// CCIR-moderate corpus here carries 100 trials at every cell of this
/// channel's threshold region.
///
/// ## Measured (100 trials/cell, paired)
///
/// | cell | arm | npre | unpruned | npre-only | unpruned-only |
/// |---|---|---:|---:|---:|---:|
/// | m26 | baseline | 20 | 26 | 0 | 6 |
/// | m26 | `[0]` | 12 | 14 | 0 | 2 |
/// | m26 | `[0,1,-1]` | 20 | 23 | 1 | 4 |
/// | m25 | baseline | 56 | 57 | 1 | 2 |
/// | m25 | `[0]` | 43 | 46 | 2 | 5 |
/// | m25 | `[0,1,-1]` | 56 | 58 | 3 | 5 |
/// | m24 | baseline | 85 | 88 | 1 | 4 |
/// | m24 | `[0]` | 75 | 84 | 0 | 9 |
/// | m24 | `[0,1,-1]` | 84 | 89 | 0 | 5 |
///
/// **The two are separate mechanisms, and the monitor's gap is
/// entirely the first one.** Timing diversity takes `rung_major` from
/// 12/43/75 to 20/56/84 with **zero** trials lost, matching the real
/// 12 kHz baseline exactly at m26 and m25 (20/20, 56/56) and within one
/// at m24 — so under fading the monitor path is timing-limited and
/// nothing else. On AWGN a residual 5 remained at −27 dB and was
/// attributed to the DDC front end; here not even that shows.
///
/// The `npre` pruning cost is real, independent and much smaller:
/// 2 npre-only against 12 unpruned-only pooled across cells on the
/// production baseline (McNemar exact, two-sided, p = 0.013), i.e.
/// 1–6 trials per 100. That is #311's floor, quantified at n=100 —
/// with the caveat that its n=20 claim "`npre` never decodes a trial
/// the unpruned search misses" is *nearly* but not exactly right here:
/// npre-only is 1 at both m25 and m24.
///
/// **Substitutable or additive is cell-dependent, and the trend runs
/// with SNR.** From the shared `[0]`+npre floor: m26 12 → timing 20,
/// pruning 14, both 23 where additivity predicts 22 (super-additive —
/// #311's "rescues trials neither rescues alone", reproduced at n=100);
/// m25 43 → 56 / 46 / 58 against 59 (additive); m24 75 → 84 / 84 / 89
/// against 93 (strongly overlapping — 4 of 9 rescued by either alone).
/// Near threshold the two fix different failures; higher up the
/// marginal trials just need one more try, whichever kind.
///
/// One consequence for the embedded ladder (#310): the pruning cost is
/// nearly twice as large without timing diversity as with it (9 vs 5
/// unpruned-only at m24), so the two are not independent knobs, and
/// restoring timing is the one that pays.
#[test]
#[ignore = "tier C — needs the fst4_60_ccir_moderate_* corpus; process-global OSD toggle, run alone"]
fn fst4_60_ccir_moderate_timing_x_pruning() {
    use mfsk_core::fec::ldpc240_101::fst4_osd_diag_force_old;
    use rayon::prelude::*;

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let dir: PathBuf = Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf();

    eprintln!("\nFST4-60 CCIR-moderate — timing diversity x OSD search, DDC monitor path");
    eprintln!("npre = production npre1/npre2; unpruned = pre-#198 combinatorial search");
    eprintln!("trials are paired (same waveform both passes), so the discordant counts");
    eprintln!("are what the aggregate difference is actually made of.\n");

    /// Discordant pair counts for one arm across the two OSD passes:
    /// (both, npre-only, unpruned-only, neither).
    fn pairs(
        a: &[(bool, bool, bool, bool)],
        b: &[(bool, bool, bool, bool)],
        f: fn(&(bool, bool, bool, bool)) -> bool,
    ) -> (usize, usize, usize, usize) {
        let mut c = (0, 0, 0, 0);
        for (x, y) in a.iter().zip(b.iter()) {
            match (f(x), f(y)) {
                (true, true) => c.0 += 1,
                (true, false) => c.1 += 1,
                (false, true) => c.2 += 1,
                (false, false) => c.3 += 1,
            }
        }
        c
    }

    for snr in ["m26", "m25", "m24"] {
        let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.file_stem()
                    .and_then(|s| s.to_str())
                    .is_some_and(|s| s.starts_with(&format!("fst4_60_ccir_moderate_{snr}_")))
            })
            .collect();
        paths.sort();

        let mut passes: Vec<Vec<(bool, bool, bool, bool)>> = Vec::new();
        for force_old in [false, true] {
            fst4_osd_diag_force_old(force_old);
            passes.push(
                paths
                    .par_iter()
                    .filter_map(|p| load_wav_i16_opt(p).map(|a| trial_arms(&a)))
                    .collect(),
            );
            fst4_osd_diag_force_old(false);
        }
        let (np, un) = (&passes[0], &passes[1]);
        let n = np.len();

        eprintln!("{snr} (n={n})");
        eprintln!(
            "  {:<22} {:>6} {:>9}   {:>9} {:>13}",
            "arm", "npre", "unpruned", "npre-only", "unpruned-only"
        );
        for (name, f) in [
            (
                "baseline (real 12k)",
                (|r: &(bool, bool, bool, bool)| r.0) as fn(&(bool, bool, bool, bool)) -> bool,
            ),
            ("rung_major [0]", |r| r.1),
            ("rung_major [0,1,-1]", |r| r.2),
            ("production ladder", |r| r.3),
        ] {
            let (_, npre_only, unpruned_only, _) = pairs(np, un, f);
            eprintln!(
                "  {name:<22} {:>6} {:>9}   {:>9} {:>13}",
                np.iter().filter(|r| f(r)).count(),
                un.iter().filter(|r| f(r)).count(),
                npre_only,
                unpruned_only,
            );
        }

        // The timing factor is within-trial in both passes, so its
        // discordance comes straight out of one pass's tuples.
        for (label, pass) in [("npre", np), ("unpruned", un)] {
            let gained = pass.iter().filter(|r| !r.1 && r.2).count();
            let lost = pass.iter().filter(|r| r.1 && !r.2).count();
            eprintln!(
                "  timing [0] -> [0,1,-1] under {label:<9}: +{gained} -{lost}  \
                 (rescued by timing alone: {})",
                pass.iter().filter(|r| !r.1 && r.2).count(),
            );
        }
        // Substitutable or additive: what each factor rescues from the
        // shared [0]+npre floor, and whether the two sets overlap.
        let floor = np.iter().filter(|r| r.1).count();
        let timing_only = np.iter().filter(|r| r.2).count();
        let pruning_only = un.iter().filter(|r| r.1).count();
        let both = un.iter().filter(|r| r.2).count();
        eprintln!(
            "  factorial: floor([0],npre)={floor}  timing={timing_only}  \
             pruning={pruning_only}  both={both}  \
             (additive would predict {})",
            floor + (timing_only - floor) + (pruning_only - floor),
        );
        eprintln!();
    }
}
