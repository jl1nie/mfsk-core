//! What a slot-grid phase error costs the **board's** driver, measured
//! on the host.
//!
//! The CoreS3 receiver sets its slot grid once — NTP/RTC, or a cold
//! acquisition — and holds it (`m5stack-cores3-app`'s decode pipeline,
//! "lock and hold"). Where that grid lands is therefore a decision
//! taken once per session, and on 2026-09-16 three boots of one image
//! against one baked slot settled at `dec` 8, 6 and 5, flat for every
//! slot of each capture. Deciding what to *do* about that needs the
//! shape of `dec` against phase, and taking it on hardware costs a
//! flash and several minutes per point.
//!
//! `decode_block_multipass` has two bodies split on `fft-rustfft` —
//! three passes with subtraction between them under it, one pass with
//! none without — so a host build with `fft-extern,fixed-point` runs
//! the board's single-pass driver, and scores 7 of the 20-entry fixed
//! golden against the board's own `dec=7` on the same recording
//! (`ft8_embedded_driver_recall.rs` pins that).
//!
//! ## This is an approximation, and it has misled once
//!
//! Matching one number at one phase is not matching the path. Two
//! differences remain, and both matter for phase:
//!
//! - **Search width.** `decode_block` runs FT8's default ±2.5 s coarse
//!   search. The board's per-slot path runs `stage1_inc`'s
//!   `SYNC_LAG_S = 1.0`. A phase whose signals sit at DT +1.5 s decodes
//!   well here and not at all there. The `in1s` column (decodes at
//!   |dt| <= 1.0) is the correction, and it is the column to read.
//! - **Candidate flow.** `decode_block` has its own pass-1 limit and
//!   refine. The board goes `coarse_sync_with_lag` → early/late split
//!   → `refine_candidates_into` → `process_candidates_into_…`
//!   (`embedded-shared/src/dual_core.rs`). This file does not model
//!   that, and its `in1s` ceiling is 7 where the board has run at 8.
//!
//! On 2026-09-16 the first bullet was missed: scored on the ±2.5 s
//! count, "align the grid to the earliest decode" beat the median rule
//! 6.30 to 5.50, was implemented, and on `in1s` scores **1.00** against
//! **4.70**. It was reverted before it reached a board. Treat anything
//! this file says about *which phase is best* as provisional until
//! `ft8_embedded_pipeline_mirror.rs`, which follows the board's own
//! candidate flow, agrees.
//!
//! The SIM harness loops **one whole slot**, so a grid at phase `phi`
//! sees a window that wraps — the tail of one copy followed by the head
//! of the next. `rotate_left` reproduces that exactly, which matters:
//! signals straddling the seam are the ones a phase error costs.
//!
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_embedded_grid_phase -- --ignored --nocapture
//! ```
//! Drop `fixed-point` for the f32 twin. Diagnostic: it prints a table
//! and asserts nothing about the shape, because the shape is what is
//! being looked at.
//!
//! Refs #356, #358.
#![cfg(not(feature = "fft-rustfft"))]

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// One FT8 slot at 12 kHz.
const SLOT: usize = 180_000;

/// The board's own decode arguments (`decode_pipeline.rs`): the audio
/// band it searches, its baseline-normalised sync floor, the ship
/// depth, and the refined-candidate cap.
const BAND: (f32, f32) = (200.0, 3_000.0);
const SYNC_MIN: f32 = 1.0;
const MAX_CAND: usize = 15;

/// `phi` in seconds, positive meaning the grid opens later into the
/// slot than the recording's own boundary.
fn decode_at_phase(slot: &[i16], phi_s: f32) -> Vec<mfsk_core::engine::frame::DecodeResult> {
    let k = ((phi_s * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
    let mut rotated = slot.to_vec();
    rotated.rotate_left(k);
    decode_block(
        &rotated,
        BAND.0,
        BAND.1,
        SYNC_MIN,
        DecodeDepth::EMBEDDED,
        MAX_CAND,
    )
}

#[test]
#[ignore = "diagnostic — prints the phase response of the shipped embedded driver"]
fn embedded_driver_phase_response() {
    let mut slot = load_wav_i16(std::path::Path::new(QSO3_PATH));
    // The recording is 180 101 samples; the harness on the board loops
    // a whole number of slots for phase continuity, so take the same
    // 180 000 it does rather than the file's own length.
    slot.truncate(SLOT);

    println!(
        "driver: single-pass, no subtraction (cfg(not(fft-rustfft)))  numeric: {}",
        if cfg!(feature = "fixed-point") {
            "fixed-point"
        } else {
            "f32"
        }
    );
    // `dec` is what the *block* driver finds with its +-2.5 s coarse
    // search. `in1s` counts only decodes at |dt| <= 1.0, which is what
    // the board's per-slot path can reach at all: `stage1_inc` runs
    // `SYNC_LAG_S = 1.0` (jz = 13, `n_lag=27` in its profile line),
    // deliberately narrower than the acquisition trial's. A phase that
    // scores well on `dec` and badly on `in1s` is one the steady loop
    // cannot use.
    println!("  phi(s)  dec  in1s   min dt   med dt   max dt");

    // `MFSK_GRID_PHASE=start,end,step` (seconds) overrides the default
    // 0.1 s comb over +-2.5 s. A finer comb is what shows that the
    // response inside the plateau is noise rather than a peak: adjacent
    // 0.1 s points already swing by 2 decodes.
    let (lo, hi, dphi) = match std::env::var("MFSK_GRID_PHASE") {
        Ok(v) => {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            assert_eq!(f.len(), 3, "MFSK_GRID_PHASE wants start,end,step");
            (f[0], f[1], f[2])
        }
        Err(_) => (-2.5, 2.5, 0.1),
    };
    let mut best = (0usize, 0.0f32);
    let mut step = 0i32;
    while (lo + step as f32 * dphi) <= hi + 1e-6 {
        let phi = lo + step as f32 * dphi;
        let got = decode_at_phase(&slot, phi);
        let in1s = got.iter().filter(|r| r.dt_sec.abs() <= 1.0).count();
        let mut dts: Vec<f32> = got.iter().map(|r| r.dt_sec).collect();
        dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let mut msgs: Vec<String> = got.iter().filter_map(|r| unpack77(r.message77())).collect();
        msgs.sort();
        msgs.dedup();
        if in1s > best.0 {
            best = (in1s, phi);
        }
        if dts.is_empty() {
            println!("  {phi:+6.2}   0    0        -        -        -");
        } else {
            println!(
                "  {phi:+6.2}  {:2}   {in1s:2}   {:+6.3}   {:+6.3}   {:+6.3}",
                got.len(),
                dts[0],
                dts[dts.len() / 2],
                dts[dts.len() - 1],
            );
        }
        step += 1;
    }
    println!(
        "  best in1s={} at phi={:+.2} s — the plateau's width and height is the \
         number the lock policy has to live with",
        best.0, best.1
    );
}

/// Median or earliest? The acquisition correction rule, measured.
///
/// `m5stack-cores3-app` corrects the cluster centre it accepts by the
/// **median DT of what that trial decoded** — "so the decoded median
/// lands near zero". The competing intuition is that the grid should
/// instead open before the *earliest* signal, since a frame that
/// starts before the window is clipped and one that ends early is not:
/// over the comb above, phases with `min dt > 0` average 5.27 decodes
/// against 4.67 for `min dt <= 0` (91 vs 165 points, 2026-09-16), and
/// both of the two phases that reach 8 are in the first group.
///
/// +0.6 decodes of mean is worth an experiment, not an assumption, so
/// this runs the whole acquisition — `acquire_slot_phases`, the
/// best-of-N trial choice the app makes, then each rule's correction —
/// from a comb of capture phases and reports where each rule's grid
/// lands and what it decodes there.
#[test]
#[ignore = "diagnostic — compares the acquisition's DT correction rules"]
fn acquisition_correction_rules() {
    use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phases};

    let mut slot = load_wav_i16(std::path::Path::new(QSO3_PATH));
    slot.truncate(SLOT);
    let mut loopbuf = Vec::with_capacity(SLOT * 4);
    for _ in 0..4 {
        loopbuf.extend_from_slice(&slot);
    }

    // What the board passes: 200 coarse candidates per tile, up to 5
    // cluster centres to try.
    const ACQ_MAX_CAND: usize = 200;
    const ACQ_MAX_TRIALS: usize = 5;
    /// Where the earliest rule puts the earliest decoded signal,
    /// seconds after the window opens. Zero is the edge itself; the two
    /// phases that reach 8 on the +-2.5 s metric sat at +0.45 and
    /// +0.61 — and both score 1 on the metric that matters, which is
    /// the point this test exists to make.
    const EARLIEST_TARGET_S: f32 = 0.45;
    let earliest_target: f32 = std::env::var("MFSK_EARLIEST_TARGET")
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(EARLIEST_TARGET_S);

    let wrap = |mut dt: f32| {
        while dt > 7.5 {
            dt -= 15.0;
        }
        while dt <= -7.5 {
            dt += 15.0;
        }
        dt
    };
    // Two numbers per phase: what the block driver finds with its
    // +-2.5 s coarse search, and what is left inside the +-1.0 s the
    // per-slot path searches (`stage1_inc`'s `SYNC_LAG_S`). Only the
    // second is reachable on the board — scoring by the first is what
    // made the earliest rule look like an improvement.
    let dec_at = |phi: f32| -> (usize, usize) {
        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let got = decode_block(
            &loopbuf[k..k + SLOT],
            BAND.0,
            BAND.1,
            SYNC_MIN,
            DecodeDepth::EMBEDDED,
            MAX_CAND,
        );
        let in1s = got.iter().filter(|r| r.dt_sec.abs() <= 1.0).count();
        (got.len(), in1s)
    };

    println!("  capture   trial(dec)   median-rule -> in1s(dec)   earliest-rule -> in1s(dec)");
    let (mut sum_a, mut sum_b, mut n) = (0usize, 0usize, 0usize);
    let mut step = 0;
    while step < 30 {
        let cap_phi = step as f32 * 0.5;
        let cap_k = (cap_phi * 12_000.0) as usize;
        let audio = &loopbuf[cap_k..cap_k + REQUIRED_SAMPLES];
        let phases = acquire_slot_phases(
            audio,
            100.0,
            3_000.0,
            SYNC_MIN,
            ACQ_MAX_CAND,
            ACQ_MAX_TRIALS,
        );
        // Best of the trials, which is what the app does since
        // 2026-09-16.
        let mut best: Option<(usize, f32, f32, f32)> = None; // (dec, centre, med, min)
        for &(centre, _) in phases.iter() {
            let off = ((centre * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
            let got = decode_block(
                &loopbuf[cap_k + off..cap_k + off + SLOT],
                100.0,
                3_000.0,
                SYNC_MIN,
                DecodeDepth::EMBEDDED,
                MAX_CAND,
            );
            if got.is_empty() {
                continue;
            }
            let mut dts: Vec<f32> = got.iter().map(|r| r.dt_sec).collect();
            dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            if best.is_none_or(|(n, _, _, _)| got.len() > n) {
                best = Some((got.len(), centre, dts[dts.len() / 2], dts[0]));
            }
        }
        let Some((dec_trial, centre, med, min_dt)) = best else {
            println!("  {cap_phi:6.2}   (nothing decoded)");
            step += 1;
            continue;
        };
        let a = (cap_phi + wrap(centre + med)).rem_euclid(15.0);
        let b = (cap_phi + wrap(centre + min_dt - earliest_target)).rem_euclid(15.0);
        let ((da, ia), (db, ib)) = (dec_at(a), dec_at(b));
        sum_a += ia;
        sum_b += ib;
        n += 1;
        println!(
            "  {cap_phi:6.2}      {dec_trial:2}       phi={a:5.2} -> {ia:2} ({da:2})       phi={b:5.2} -> {ib:2} ({db:2})"
        );
        step += 1;
    }
    println!(
        "  mean in1s over {n} captures: median-rule {:.2}, earliest-rule {:.2}",
        sum_a as f32 / n as f32,
        sum_b as f32 / n as f32
    );
}

/// What the ceiling is made of, once the phase is right.
///
/// The phase response above tops out at 8 and sits at 7 for most of
/// the plateau, against 20 known real signals in this recording. Phase
/// is therefore not what is missing — this walks the two knobs the
/// board actually chose (`MAX_CAND`, and `DecodeDepth`'s OSD) at the
/// best phase and at the one the median rule used to pick, so the
/// remaining gap is attributed rather than guessed.
#[test]
#[ignore = "diagnostic — what raising the embedded caps would buy at a fixed phase"]
fn ceiling_at_a_fixed_phase() {
    let mut slot = load_wav_i16(std::path::Path::new(QSO3_PATH));
    slot.truncate(SLOT);

    println!("  phi      depth      max_cand   dec");
    for phi in [-1.26f32, 0.22, 0.0] {
        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let mut rotated = slot.clone();
        rotated.rotate_left(k);
        for (name, depth) in [
            ("EMBEDDED", DecodeDepth::EMBEDDED),
            ("BP_ONLY ", DecodeDepth::BP_ONLY),
            ("FULL    ", DecodeDepth::FULL),
        ] {
            for max_cand in [15usize, 30, 60, 120] {
                let got = decode_block(&rotated, BAND.0, BAND.1, SYNC_MIN, depth, max_cand);
                println!("  {phi:+5.2}   {name}   {max_cand:8}   {:2}", got.len());
            }
        }
    }
}

/// Estimate the phase, or search for it?
///
/// The correction rules above are estimators: they read a statistic off
/// the trial's decodes and move the grid by it. Both land inside the
/// plateau, and neither can do better than the plateau's own noise —
/// over a 0.02 s comb `dec` swings by up to 3 between adjacent points,
/// and the earliest rule's own landings differ by 10-40 ms from capture
/// to capture and decode 5 to 8 for it.
///
/// No DT statistic resolves that. What does is the objective itself:
/// acquisition already decodes whole trial slots, so it can try a few
/// offsets around its estimate and keep whichever decoded most. This
/// measures what that buys, and what it costs in extra trial decodes —
/// the only currency that matters here, ~1.1 s each on the board, paid
/// once per acquisition and never in the slot loop.
#[test]
#[ignore = "diagnostic — fine search around the estimate vs the estimate alone"]
fn fine_search_around_the_estimate() {
    use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phases};

    let mut slot = load_wav_i16(std::path::Path::new(QSO3_PATH));
    slot.truncate(SLOT);
    let mut loopbuf = Vec::with_capacity(SLOT * 4);
    for _ in 0..4 {
        loopbuf.extend_from_slice(&slot);
    }
    const ACQ_MAX_CAND: usize = 200;
    const ACQ_MAX_TRIALS: usize = 5;

    // `span,step` in seconds for the fine search, both sides.
    let (span, fine_step) = match std::env::var("MFSK_FINE_SEARCH") {
        Ok(v) => {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            assert_eq!(f.len(), 2, "MFSK_FINE_SEARCH wants span,step");
            (f[0], f[1])
        }
        Err(_) => (0.15, 0.03),
    };

    // Both numbers: what the block driver finds with its +-2.5 s
    // search, and what survives inside the +-1.0 s the per-slot path
    // actually searches. Only the second is reachable on the board, and
    // measuring the first is what made the earliest rule look good.
    let dec_at = |phi: f32| -> (usize, usize) {
        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let got = decode_block(
            &loopbuf[k..k + SLOT],
            BAND.0,
            BAND.1,
            SYNC_MIN,
            DecodeDepth::EMBEDDED,
            MAX_CAND,
        );
        let in1s = got.iter().filter(|r| r.dt_sec.abs() <= 1.0).count();
        (got.len(), in1s)
    };
    let wrap = |mut dt: f32| {
        while dt > 7.5 {
            dt -= 15.0;
        }
        while dt <= -7.5 {
            dt += 15.0;
        }
        dt
    };

    println!("  fine search: +-{span:.2} s in {fine_step:.3} s steps");
    println!("  capture   estimate -> dec   searched -> dec   extra decodes");
    let (mut sum_e, mut sum_s, mut sum_cost, mut n) = (0usize, 0usize, 0usize, 0usize);
    let mut step = 0;
    while step < 30 {
        let cap_phi = step as f32 * 0.5;
        let cap_k = (cap_phi * 12_000.0) as usize;
        let audio = &loopbuf[cap_k..cap_k + REQUIRED_SAMPLES];
        let phases = acquire_slot_phases(
            audio,
            100.0,
            3_000.0,
            SYNC_MIN,
            ACQ_MAX_CAND,
            ACQ_MAX_TRIALS,
        );
        let mut best: Option<(usize, f32, f32)> = None; // (dec, centre, earliest)
        for &(centre, _) in phases.iter() {
            let off = ((centre * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
            let got = decode_block(
                &loopbuf[cap_k + off..cap_k + off + SLOT],
                100.0,
                3_000.0,
                SYNC_MIN,
                DecodeDepth::EMBEDDED,
                MAX_CAND,
            );
            if got.is_empty() {
                continue;
            }
            let mut dts: Vec<f32> = got.iter().map(|r| r.dt_sec).collect();
            dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            if best.is_none_or(|(n, _, _)| got.len() > n) {
                best = Some((got.len(), centre, dts[dts.len() / 2]));
            }
        }
        let Some((_, centre, med)) = best else {
            step += 1;
            continue;
        };
        let est = (cap_phi + wrap(centre + med)).rem_euclid(15.0);
        let de = dec_at(est).1;
        // The search: the estimate plus a comb around it, best kept.
        let mut searched = (de, est);
        let mut cost = 0usize;
        let mut i = 1;
        while (i as f32) * fine_step <= span + 1e-6 {
            for sign in [-1.0f32, 1.0] {
                let phi = (est + sign * i as f32 * fine_step).rem_euclid(15.0);
                let d = dec_at(phi).1;
                cost += 1;
                if d > searched.0 {
                    searched = (d, phi);
                }
            }
            i += 1;
        }
        sum_e += de;
        sum_s += searched.0;
        sum_cost += cost;
        n += 1;
        println!(
            "  {cap_phi:6.2}    phi={est:5.2} -> {de:2}      phi={:5.2} -> {:2}      {cost:2}",
            searched.1, searched.0
        );
        step += 1;
    }
    println!(
        "  mean over {n} captures: estimate {:.2}, searched {:.2}, extra decodes {:.1}",
        sum_e as f32 / n as f32,
        sum_s as f32 / n as f32,
        sum_cost as f32 / n as f32
    );
}
