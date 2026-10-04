// SPDX-License-Identifier: GPL-3.0-only
//! Can a cold acquisition find the grid from less than 25 s?
//!
//! The capture is the larger half of the ~40 s of dark band an
//! acquisition costs (`docs/notes/CORES3_FT8_SLOT_BUDGET.md` §8), so a
//! shorter one is worth real outage. `acquire_slot_phases` takes three
//! ±2.5 s tiles at 0 / 5 / 10 s, which covers the 15 s period exactly
//! and needs `10 + SLOT` = 25 s of audio.
//!
//! Fewer tiles means wider ones, and a wider tile spends more of its
//! range on lags where a Costas block is truncated — which is what
//! made the wide grid probe unusable. Whether that matters *here* is
//! the question: acquisition resolves its shortlist by trying decodes,
//! so it can tolerate a list that a one-shot estimator could not.
//!
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_acquire_capture_length -- --ignored --nocapture
//! ```
#![cfg(all(feature = "ft8", not(feature = "fft-rustfft")))]

use mfsk_core::engine::sync::{SyncCandidate, circular_dt_clusters};
use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

const SLOT: usize = 180_000;
const SR: usize = 12_000;
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
/// `ACQUIRE_MAX_CAND` on the board.
const MAX_CAND: usize = 200;
/// `CLUSTER_KERNEL_S` in `ft8::acquire`.
const KERNEL_S: f32 = 0.5;
/// `ACQUIRE_MAX_TRIALS` — the shortlist the board actually tries.
const MAX_OUT: usize = 5;

/// One tiling: how many windows, how far apart, and how wide each
/// searches. Capture length follows: `(n - 1) * spacing + SLOT`.
struct Tiling {
    name: &'static str,
    n: usize,
    spacing_s: f32,
    lag_s: f32,
}

impl Tiling {
    fn capture_s(&self) -> f32 {
        (self.n as f32 - 1.0) * self.spacing_s + 15.0
    }
    /// Period covered: the tiles' span plus each end's reach.
    fn coverage_s(&self) -> f32 {
        (self.n as f32 - 1.0) * self.spacing_s + 2.0 * self.lag_s
    }
}

fn wrap(mut d: f32) -> f32 {
    while d > 7.5 {
        d -= 15.0;
    }
    while d <= -7.5 {
        d += 15.0;
    }
    d
}

/// `acquire_slot_phases`, with the tiling as a parameter.
fn phases(audio: &[i16], t: &Tiling) -> Vec<(f32, f32)> {
    let mut cands: Vec<SyncCandidate> = Vec::new();
    for k in 0..t.n {
        let off_s = k as f32 * t.spacing_s;
        let off = (off_s * SR as f32) as usize;
        if audio.len() < off + SLOT {
            continue;
        }
        let spec = compute_spectrogram(&audio[off..off + SLOT], FREQ_MAX);
        for c in coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, t.lag_s) {
            cands.push(SyncCandidate {
                freq_hz: c.freq_hz,
                dt_sec: c.dt_sec + off_s,
                score: c.score,
            });
        }
    }
    circular_dt_clusters(&cands, KERNEL_S, 15.0, MAX_OUT)
}

/// For each capture phase, is the true grid phase on the shortlist?
///
/// The board tries each cluster with a real decode and keeps the best,
/// so "on the shortlist at all" is the property that matters — a
/// cluster ranked fifth still gets tried.
///
/// The tolerance is what the board actually accepts, not what a
/// point estimate would want: the trial corrects the cluster centre by
/// the median DT of what it decoded, and `acquire`'s own note says a
/// ±2.5 s trial with that correction "accepts nothing outside ±1.0 s".
/// `cluster_then_decode_acquisition` scores itself the same way.
/// `MFSK_ACQ_TOL` overrides it.
#[test]
#[ignore = "diagnostic — capture length against acquisition coverage"]
fn a_shorter_capture_still_finds_the_grid() {
    let name = std::env::var("MFSK_ACQ_WAV").unwrap_or_else(|_| "qso3_busy.wav".into());
    let path = format!(
        "{}/../embedded-poc/assets/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut slot = load_wav_i16(std::path::Path::new(&path));
    slot.truncate(SLOT);

    let tilings = [
        Tiling {
            name: "3 x ±2.5 @5s  (shipped)",
            n: 3,
            spacing_s: 5.0,
            lag_s: 2.5,
        },
        Tiling {
            name: "2 x ±3.75 @7.5s        ",
            n: 2,
            spacing_s: 7.5,
            lag_s: 3.75,
        },
        Tiling {
            name: "2 x ±4.5 @6s           ",
            n: 2,
            spacing_s: 6.0,
            lag_s: 4.5,
        },
        Tiling {
            name: "2 x ±5.0 @5s           ",
            n: 2,
            spacing_s: 5.0,
            lag_s: 5.0,
        },
        Tiling {
            name: "1 x ±6.24 (max lag)    ",
            n: 1,
            spacing_s: 0.0,
            lag_s: 6.24,
        },
    ];

    // 40 capture phases across the period, as `ft8_cold_acquisition_fixed`
    // uses — the capture starts wherever `arm_acquisition` happened to fire.
    const N_PHASE: usize = 40;
    let tol: f32 = std::env::var("MFSK_ACQ_TOL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1.0);
    println!("\n  {name}\n");
    println!(
        "  {:24} {:>8} {:>9} {:>10} {:>9} {:>8}",
        "tiling", "capture", "coverage", "on list", "rank 1", "worst"
    );
    println!("  {:-<74}", "");

    for t in &tilings {
        let (mut on_list, mut first, mut worst) = (0usize, 0usize, 0.0f32);
        for i in 0..N_PHASE {
            let off_s = i as f32 * (15.0 / N_PHASE as f32);
            // The capture begins `off_s` into the period, so the grid
            // sits at `-off_s` relative to the capture's first sample.
            let truth = wrap(-off_s);
            let need = (t.capture_s() * SR as f32) as usize;
            let mut long: Vec<i16> = Vec::with_capacity(need + SLOT);
            let start = (off_s * SR as f32) as usize;
            while long.len() < need {
                let take = (SLOT - (start + long.len()) % SLOT).min(need - long.len());
                let from = (start + long.len()) % SLOT;
                long.extend_from_slice(&slot[from..from + take]);
            }
            let list = phases(&long, t);
            let best = list
                .iter()
                .map(|(dt, _)| wrap(dt - truth).abs())
                .fold(f32::INFINITY, f32::min);
            if best <= tol {
                on_list += 1;
                if let Some((dt, _)) = list.first() {
                    if wrap(dt - truth).abs() <= tol {
                        first += 1;
                    }
                }
            } else if best.is_finite() {
                worst = worst.max(best);
            }
        }
        println!(
            "  {:24} {:7.1}s {:8.1}s {:7}/{} {:8}/{} {:7.2}s",
            t.name,
            t.capture_s(),
            t.coverage_s(),
            on_list,
            N_PHASE,
            first,
            N_PHASE,
            worst
        );
    }
    println!(
        "\n  \"on list\" = the true phase is within 0.5 s of one of the {MAX_OUT} clusters the\n  \
         board would try at tolerance {tol}; \"rank 1\" = it is the first one.\n           A capture of 25 s costs\n  \
         25 s of dark band before the 10-15 s of compute even starts."
    );
}

/// **Progressive acquisition: does the second stage rescue the first's
/// failures, or fail on the same slots?**
///
/// The capture-length table says shortening is a wash — every tiling
/// loses as much success as it saves time, so expected outage stays
/// near 40 s. What it also says is that **one tile over the first 15 s
/// already succeeds two thirds of the time**, and those 15 s are
/// collected before the shipped acquisition has even finished
/// listening.
///
/// So the question is not how long to capture; it is what order to do
/// the work in. Try one tile as soon as a slot exists, and keep
/// capturing only if it misses. That is only worth anything if the
/// three-tile stage succeeds *on the cases the one-tile stage failed*
/// — if the two fail together, the fast path buys nothing and costs a
/// wasted search.
#[test]
#[ignore = "diagnostic — is stage 2 independent of stage 1's failures"]
fn a_second_stage_rescues_the_first_stages_misses() {
    const N_PHASE: usize = 40;
    let tol: f32 = std::env::var("MFSK_ACQ_TOL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1.0);
    let one = Tiling {
        name: "",
        n: 1,
        spacing_s: 0.0,
        lag_s: 6.24,
    };
    let three = Tiling {
        name: "",
        n: 3,
        spacing_s: 5.0,
        lag_s: 2.5,
    };

    println!(
        "\n  {:14} {:>10} {:>10} {:>12} {:>14}",
        "recording", "1 tile", "3 tiles", "3 | 1 missed", "E[outage]"
    );
    println!("  {:-<64}", "");
    for name in ["qso3_busy.wav", "qso1.wav", "qso2.wav"] {
        let path = format!(
            "{}/../embedded-poc/assets/{name}",
            env!("CARGO_MANIFEST_DIR")
        );
        let mut slot = load_wav_i16(std::path::Path::new(&path));
        slot.truncate(SLOT);

        let (mut p1, mut p3, mut rescued, mut missed) = (0usize, 0usize, 0usize, 0usize);
        for i in 0..N_PHASE {
            let off_s = i as f32 * (15.0 / N_PHASE as f32);
            let truth = wrap(-off_s);
            let hit = |t: &Tiling| {
                let need = (t.capture_s() * SR as f32) as usize;
                let start = (off_s * SR as f32) as usize;
                let mut long: Vec<i16> = Vec::with_capacity(need + SLOT);
                while long.len() < need {
                    let from = (start + long.len()) % SLOT;
                    let take = (SLOT - from).min(need - long.len());
                    long.extend_from_slice(&slot[from..from + take]);
                }
                phases(&long, t)
                    .iter()
                    .any(|(dt, _)| wrap(dt - truth).abs() <= tol)
            };
            let a = hit(&one);
            let b = hit(&three);
            if a {
                p1 += 1;
            }
            if b {
                p3 += 1;
            }
            if !a {
                missed += 1;
                if b {
                    rescued += 1;
                }
            }
        }
        // Stage 1: 15 s capture + ~5 s compute. Stage 2 adds 10 s of
        // capture and the two further tiles, ~18 s. Shipped is ~37 s.
        let (t1, t2, t_ship) = (20.0f32, 38.0f32, 37.0f32);
        let pa = p1 as f32 / N_PHASE as f32;
        let pb = if missed > 0 {
            rescued as f32 / missed as f32
        } else {
            1.0
        };
        // Expected time to a successful acquisition, retrying the whole
        // thing on failure.
        let p_total = pa + (1.0 - pa) * pb;
        let e_prog = (pa * t1 + (1.0 - pa) * t2) / p_total.max(1e-3);
        let e_ship = t_ship / (p3 as f32 / N_PHASE as f32).max(1e-3);
        println!(
            "  {name:14} {p1:6}/{N_PHASE} {p3:6}/{N_PHASE} {rescued:8}/{missed:<3} {:6.0}s vs {:.0}s",
            e_prog, e_ship
        );
    }
    println!(
        "\n  \"3 | 1 missed\" is the number that decides it: a second stage that fails\n           on the same slots as the first buys nothing. E[outage] retries on failure."
    );
}

/// **On the list is not the same as confirmable.**
///
/// A trial cuts a whole slot out of the capture starting at the
/// candidate phase, so a capture of `C` seconds can only cut at
/// offsets `0 ..= C - 15`. `decode_pipeline`'s clamp
/// (`d4c014b3`) maps a centre circularly onto that band and lets the
/// trial's own ±2.5 s search plus its median-DT correction cover the
/// difference — which works while the centre is within 2.5 s of the
/// band, and not otherwise.
///
/// So the reachable share of the period is
/// `(max_off + 5) / 15`, and a 15 s capture reaches a third of it
/// however good the search is. "One tile at 15 s finds the grid
/// two thirds of the time" is therefore not a statement about what a
/// 15 s stage could *confirm*, and a progressive acquisition has to be
/// designed against this table rather than that one.
#[test]
#[ignore = "diagnostic — what a capture can trial, not just find"]
fn what_a_short_capture_can_actually_confirm() {
    const N_PHASE: usize = 40;
    let tol: f32 = std::env::var("MFSK_ACQ_TOL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1.0);
    /// `decode_block`'s own coarse window, which is what a trial searches.
    const TRIAL_LAG_S: f32 = 2.5;

    let tilings = [
        (
            "3 x ±2.5 @5s (shipped)",
            Tiling {
                name: "",
                n: 3,
                spacing_s: 5.0,
                lag_s: 2.5,
            },
        ),
        (
            "2 x ±5.0 @5s          ",
            Tiling {
                name: "",
                n: 2,
                spacing_s: 5.0,
                lag_s: 5.0,
            },
        ),
        (
            "1 x ±6.24 @17.5s      ",
            Tiling {
                name: "",
                n: 1,
                spacing_s: 2.5,
                lag_s: 6.24,
            },
        ),
        (
            "1 x ±6.24 @15s        ",
            Tiling {
                name: "",
                n: 1,
                spacing_s: 0.0,
                lag_s: 6.24,
            },
        ),
    ];

    println!(
        "\n  {:24} {:>8} {:>10} {:>10} {:>12}",
        "tiling", "capture", "reachable", "on list", "confirmable"
    );
    println!("  {:-<68}", "");
    for name in ["qso3_busy.wav", "qso1.wav", "qso2.wav"] {
        let path = format!(
            "{}/../embedded-poc/assets/{name}",
            env!("CARGO_MANIFEST_DIR")
        );
        let mut slot = load_wav_i16(std::path::Path::new(&path));
        slot.truncate(SLOT);
        println!("  {name}");
        for (label, t) in &tilings {
            // `capture_s` is (n-1)*spacing + 15, so a one-tile entry
            // with a spacing carries extra audio for the trial to cut
            // into without adding a window.
            let max_off = t.capture_s() - 15.0;
            let reach = ((max_off + 2.0 * TRIAL_LAG_S) / 15.0).min(1.0);
            let (mut on, mut conf) = (0usize, 0usize);
            for i in 0..N_PHASE {
                let off_s = i as f32 * (15.0 / N_PHASE as f32);
                let truth = wrap(-off_s);
                let need = (t.capture_s() * SR as f32) as usize;
                let start = (off_s * SR as f32) as usize;
                let mut long: Vec<i16> = Vec::with_capacity(need + SLOT);
                while long.len() < need {
                    let from = (start + long.len()) % SLOT;
                    let take = (SLOT - from).min(need - long.len());
                    long.extend_from_slice(&slot[from..from + take]);
                }
                let list = phases(&long, t);
                let hit = list.iter().any(|(dt, _)| wrap(dt - truth).abs() <= tol);
                if hit {
                    on += 1;
                    // Reachable: the true phase, folded into [0, 15),
                    // is within a trial's search of some legal offset.
                    let want = truth.rem_euclid(15.0);
                    let nearest = want.min(max_off.max(0.0)).max(0.0);
                    let d = (want - nearest).abs().min(15.0 - want);
                    if d <= TRIAL_LAG_S {
                        conf += 1;
                    }
                }
            }
            println!(
                "  {label:24} {:7.1}s {:9.0}% {:7}/{} {:9}/{}",
                t.capture_s(),
                100.0 * reach,
                on,
                N_PHASE,
                conf,
                N_PHASE
            );
        }
    }
    println!(
        "\n  \"reachable\" is geometry — (max_off + 5) / 15. \"confirmable\" is a phase that\n           is both on the shortlist and inside a trial's reach, which is what a stage can end on."
    );
}
