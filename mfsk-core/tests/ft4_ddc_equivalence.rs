//! Does the FT4 DDC front end decode the golden recording the way the
//! FFT front end does? — `ft4::ddc`, the embedded FT4 feasibility line
//! (`docs/notes/FT4_BENCHMARK.md` §17-19).
//!
//! `ft4::ddc::candidate_baseband` exists to replace
//! `downsample_cached`'s per-candidate `cd0` — the stage that needs a
//! 92 160-point forward FFT no embedded backend can run, and that cost
//! 2 251 ms of an 8 642 ms slot on a CoreS3. Its unit tests pin the
//! filter's *response* (passband flatness, out-of-band rejection, noise
//! bandwidth against the reference band). This file pins the only thing
//! that actually decides whether the swap is safe: the decodes.
//!
//! Method — one candidate list, two front ends, everything downstream
//! held identical:
//!
//! ```text
//!   ft4_coarse_sync (shared)
//!     ├─ arm A: process_candidate_basic       → downsample_cached cd0
//!     └─ arm B: candidate_baseband + RMS-norm + ft4_sync_search
//!                → process_candidate_precomputed
//! ```
//!
//! Arm A is the production path, already held to 11 decodes / zero
//! phantoms against real `jt9` by
//! `ft4_wsjtx_samples::ft4_wsjtx_sample_precision_vs_reference_decoder`.
//! Asserting arm B reproduces arm A's *set* therefore inherits that
//! recall-and-precision statement rather than restating it — and a
//! divergence points at the front end, because nothing else differs.
//!
//! Search parameters mirror `ft4_wsjtx_samples::bench_assets`, i.e.
//! what the on-device `ft4-bench` actually runs, so a number here is
//! comparable to a number from the board.
//!
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core \
//!   --features full,internal-testing --release \
//!   --test ft4_ddc_equivalence -- --nocapture
//! ```

#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use num_complex::Complex;

use mfsk_core::engine::dsp::downsample::build_fft_cache;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
use mfsk_core::engine::pipeline::{
    DecodeDepth, DecodeResult, DecodeStrictness, process_candidate_basic,
    process_candidate_precomputed,
};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::engine::sync2d::ft4_sync_search;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::ddc::candidate_baseband;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000;

/// `bench_assets` in `ft4_wsjtx_samples.rs`, mirrored — the search the
/// baked device assets were generated for.
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 2700.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 100;
/// `ft4::decode`'s own private `SYNC_Q_MIN`.
const SYNC_Q_MIN: u32 = 8;

fn slot_audio() -> Option<Vec<i16>> {
    let path = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

/// The RMS normalisation `process_candidate_basic_impl` and
/// `refine_candidate_position` both apply (WSJT-X
/// `ft4_decode.f90:231-232`). `candidate_baseband` deliberately does
/// not, so every caller of it — this test, and any future embedded one
/// — has to do it here.
fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

/// Arm B: DDC baseband → refined position → decode.
///
/// Returns the refined `(freq_hz, i0)` alongside the result so a
/// divergence can be attributed to the sync search rather than to the
/// decoder.
fn ddc_arm(
    cand: &SyncCandidate,
    audio: &[i16],
    fft_cache: &[Complex<f32>],
    depth: DecodeDepth,
) -> (Option<DecodeResult>, f32, i32) {
    let mut cd0 = candidate_baseband(audio, cand.freq_hz);
    rms_normalise(&mut cd0);
    let s2 = ft4_sync_search::<Ft4>(&cd0, cand);
    let (freq_hz, i0) = (s2.freq_hz, s2.i0);
    let r = process_candidate_precomputed::<Ft4>(
        cand,
        // Still required by the signature; FT4's `snr_db` reads the
        // *coarse* candidate score (`pipeline::ft4_snr_db`), not this
        // cache, so nothing on this arm touches the 92 160-point
        // transform — which is the point.
        fft_cache,
        &FT4_DOWNSAMPLE,
        depth,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        SYNC_Q_MIN,
        (cd0, freq_hz, i0, s2.score),
        false,
        false,
    );
    (r, freq_hz, i0)
}

fn msg_of(d: &DecodeResult) -> String {
    unpack77(d.message77()).unwrap_or_default()
}

fn sorted_messages(results: &[DecodeResult]) -> Vec<String> {
    let mut v: Vec<String> = results.iter().map(msg_of).collect();
    v.sort();
    v.dedup();
    v
}

/// The assertion this file exists for: same candidates, same decodes.
///
/// Run at both depths the device bench reports (`EMBEDDED` and `FULL`,
/// which agree with each other on this recording — OSD buys nothing
/// here), because a front-end change that only survives the more
/// expensive ladder would be a sensitivity loss hidden behind OSD.
#[test]
fn ft4_ddc_baseband_decodes_the_golden_like_the_fft_path() {
    let Some(audio) = slot_audio() else {
        if std::env::var("MFSK_REQUIRE_CORPUS").is_ok() {
            panic!("MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing");
        }
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };

    let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    assert!(!cands.is_empty(), "coarse stage found nothing");
    let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);

    for (depth_name, depth) in [
        ("EMBEDDED", DecodeDepth::EMBEDDED),
        ("FULL", DecodeDepth::FULL),
    ] {
        let mut fft_results: Vec<DecodeResult> = Vec::new();
        let mut ddc_results: Vec<DecodeResult> = Vec::new();
        // Refined-position disagreement, over candidates *both* arms
        // decode — the diagnostic that separates "the filter moved the
        // sync peak" from "the decoder gave up".
        let mut max_dfreq = 0.0f32;
        let mut max_di0 = 0i32;
        let mut freq_steps_off = 0usize;
        let mut both = 0usize;

        for c in &cands {
            let fft_r = process_candidate_basic::<Ft4>(
                c,
                &fft_cache,
                &FT4_DOWNSAMPLE,
                depth,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
                SYNC_Q_MIN,
            );
            let (ddc_r, ddc_freq, ddc_i0) = ddc_arm(c, &audio, &fft_cache, depth);

            if let (Some(a), Some(_)) = (&fft_r, &ddc_r) {
                both += 1;
                let dfreq = (a.freq_hz - ddc_freq).abs();
                if dfreq > 0.5 {
                    freq_steps_off += 1;
                }
                max_dfreq = max_dfreq.max(dfreq);
                // `DecodeResult` carries `dt_sec`, not `i0`; convert
                // back through the same `ds_rate`/`TX_START_OFFSET_S`
                // relation `process_candidate_basic_impl` uses.
                let ds_rate = 12_000.0 / 18.0;
                let a_i0 = ((a.dt_sec + 0.5) * ds_rate).round() as i32;
                max_di0 = max_di0.max((a_i0 - ddc_i0).abs());
            }
            if let Some(r) = fft_r {
                fft_results.push(r);
            }
            if let Some(r) = ddc_r {
                ddc_results.push(r);
            }
        }

        let fft_msgs = sorted_messages(&fft_results);
        let ddc_msgs = sorted_messages(&ddc_results);

        eprintln!(
            "[{depth_name}] {} candidates | FFT path {} distinct | DDC path {} distinct | \
             max Δfreq {max_dfreq:.2} Hz ({freq_steps_off}/{both} candidates one step off), \
             max Δi0 {max_di0}",
            cands.len(),
            fft_msgs.len(),
            ddc_msgs.len()
        );
        for m in &fft_msgs {
            let mark = if ddc_msgs.contains(m) { "  " } else { "!!" };
            eprintln!("  {mark} {m}");
        }
        for m in &ddc_msgs {
            if !fft_msgs.contains(m) {
                eprintln!("  ++ {m}   (DDC only)");
            }
        }

        // Arm A is the path the golden test already holds to 11 decodes
        // / zero phantoms; if it moved, this file is measuring the
        // wrong thing and the failure belongs to `ft4_wsjtx_samples`.
        assert_eq!(
            fft_msgs.len(),
            11,
            "[{depth_name}] the FFT reference arm changed — fix that first"
        );
        assert_eq!(
            ddc_msgs, fft_msgs,
            "[{depth_name}] DDC front end decoded a different set than the FFT front end"
        );
        // The two front ends are different filters, so the refined
        // position may land one search cell away — but only one.
        // `ft4_sync_search_window` steps `df` in whole Hz (`df = idf as
        // f32`, `engine/sync2d.rs`), so 1.00 Hz *is* the quantum, not a
        // tolerance chosen to fit: measured 2026-08-30, one candidate of
        // eleven lands one step off at both depths, every other one
        // agrees exactly, and Δi0 is 0 across the board. A move of two
        // steps would mean the filter is dragging the sync peak rather
        // than the peak sitting between two equally good cells.
        assert!(
            max_dfreq <= 1.0,
            "[{depth_name}] refined frequency moved {max_dfreq:.2} Hz — more than \
             ft4_sync_search's own 1 Hz grid step — between front ends"
        );
        assert!(
            max_di0 <= 2,
            "[{depth_name}] refined sync position moved {max_di0} samples between front ends"
        );
    }
}

// ── Tier C: does the front-end swap cost sensitivity? ────────────────────

/// Golden signal in the generated corpus (`scripts/gen_ft4_sweep_wavs.sh`),
/// same constants `ft4_sweep.rs` uses.
const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;

/// Channels and SNR tags straddling every channel's own 50% crossing:
/// `docs/notes/sweep-baseline.json` puts them at −16.9 (awgn), −17.5
/// (ccir_good), −15.7 (ccir_moderate), −16.0 (ccir_poor) dB.
const SWEEP_CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];
const SWEEP_TAGS: &[&str] = &["m14", "m15", "m16", "m17", "m18", "m19", "m20"];
const SWEEP_TRIALS: u32 = 20;
/// The sweep's own coarse search (`ft4_sweep::decode_wav_ft4`), not the
/// golden test's — a recall number is only comparable to the baseline
/// if the candidate stage saw the same parameters.
const SWEEP_SYNC_MIN: f32 = 0.8;
const SWEEP_MAX_CAND: usize = 50;
const SWEEP_FREQ_MAX_HZ: f32 = 3000.0;

fn sweep_dir() -> std::path::PathBuf {
    if let Ok(d) = std::env::var("MFSK_FT4_SWEEP_DIR") {
        return std::path::PathBuf::from(d);
    }
    std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default())
        .join("../embedded-poc/assets/ft4_sweep")
        .to_path_buf()
}

fn is_golden(d: &DecodeResult) -> bool {
    msg_of(d) == GOLDEN_MSG && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
}

/// Paired recall, FFT front end vs DDC front end, on the *same* noise
/// realisations across the 50% crossing — the sensitivity statement the
/// single golden recording cannot make.
///
/// Paired on purpose: both arms decode the same file from the same
/// candidate list, so the noise realisation cancels and a difference of
/// one trial is a real difference rather than sampling scatter (which
/// 20 trials per cell would otherwise be far too few to resolve — see
/// the `feedback_sparse_snr_sampling_looks_like_bug` lesson).
///
/// Candidates are filtered to the golden frequency before decoding.
/// That deliberately measures **recall only**: candidate selection is
/// shared between the arms and cannot differ, and precision is what the
/// golden-recording test above pins (11 decodes, zero extras, on a file
/// with 14 real signals in it).
///
/// Measured 2026-08-30, 560 files (4 channels × 7 tags × 20 trials):
/// **FFT 237 decodes, DDC 238**, five disagreements — three in the
/// DDC's favour, two against, every one of them in a cell already
/// sitting on its own 50% crossing. That is the coin-flip a paired
/// comparison is supposed to leave behind once a systematic difference
/// is absent; the cost of the front-end swap is 0.0 dB.
///
/// ```text
/// channel        tag   FFT   DDC        channel        tag   FFT   DDC
/// awgn           m16    18    18        ccir_moderate  m15    14    15
/// awgn           m17     9    10        ccir_moderate  m17     3     4
/// awgn           m18     5     4        ccir_good      m17    15    14
/// ```
///
/// `#[ignore]` — tier C, needs the ~2 GB generated FT4 corpus:
///
/// ```sh
/// cargo test -p mfsk-core --features full,internal-testing --release \
///   --test ft4_ddc_equivalence -- --ignored --nocapture
/// ```
#[test]
#[ignore = "tier C — needs the ft4_sweep corpus; run with --ignored --nocapture"]
fn ft4_ddc_recall_matches_the_fft_path_across_the_crossing() {
    use rayon::prelude::*;

    let dir = sweep_dir();
    let mut work: Vec<(&str, &str, u32)> = Vec::new();
    for &ch in SWEEP_CHANNELS {
        for &tag in SWEEP_TAGS {
            for trial in 1..=SWEEP_TRIALS {
                work.push((ch, tag, trial));
            }
        }
    }

    // `(fft_hit, ddc_hit, present)` per file.
    let per_file: Vec<(bool, bool, bool)> = work
        .par_iter()
        .map(|&(ch, tag, trial)| {
            let path = dir.join(format!("ft4_{ch}_{tag}_{trial:02}.wav"));
            let Some(audio) = read_wsjtx_wav_i16(&path) else {
                return (false, false, false);
            };
            let cands = ft4_coarse_sync(
                &audio,
                FREQ_MIN_HZ,
                SWEEP_FREQ_MAX_HZ,
                SWEEP_SYNC_MIN,
                None,
                SWEEP_MAX_CAND,
            );
            let near: Vec<&SyncCandidate> = cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            if near.is_empty() {
                return (false, false, true);
            }
            let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
            let mut fft_hit = false;
            let mut ddc_hit = false;
            for c in near {
                if !fft_hit {
                    fft_hit = process_candidate_basic::<Ft4>(
                        c,
                        &fft_cache,
                        &FT4_DOWNSAMPLE,
                        DecodeDepth::FULL,
                        DecodeStrictness::Normal,
                        &[],
                        EqMode::Off,
                        SYNC_Q_MIN,
                    )
                    .as_ref()
                    .is_some_and(is_golden);
                }
                if !ddc_hit {
                    ddc_hit = ddc_arm(c, &audio, &fft_cache, DecodeDepth::FULL)
                        .0
                        .as_ref()
                        .is_some_and(is_golden);
                }
                if fft_hit && ddc_hit {
                    break;
                }
            }
            (fft_hit, ddc_hit, true)
        })
        .collect();

    let present = per_file.iter().filter(|f| f.2).count();
    if present == 0 {
        eprintln!("skipping: no FT4 sweep corpus under {}", dir.display());
        return;
    }
    assert_eq!(
        present,
        work.len(),
        "corpus is incomplete ({present}/{}) — a partial run would compare cells that are not there",
        work.len()
    );

    eprintln!("channel        tag   FFT   DDC   Δ   (of {SWEEP_TRIALS})");
    let (mut tot_fft, mut tot_ddc, mut tot_disagree) = (0usize, 0usize, 0usize);
    for (ci, &ch) in SWEEP_CHANNELS.iter().enumerate() {
        for (ti, &tag) in SWEEP_TAGS.iter().enumerate() {
            let base = (ci * SWEEP_TAGS.len() + ti) * SWEEP_TRIALS as usize;
            let cell = &per_file[base..base + SWEEP_TRIALS as usize];
            let f = cell.iter().filter(|c| c.0).count();
            let d = cell.iter().filter(|c| c.1).count();
            let dis = cell.iter().filter(|c| c.0 != c.1).count();
            tot_fft += f;
            tot_ddc += d;
            tot_disagree += dis;
            eprintln!(
                "{ch:<14} {tag:<4} {f:>4} {d:>5} {:>+4}   ({dis} disagree)",
                d as i32 - f as i32
            );
        }
    }
    eprintln!("TOTAL: FFT {tot_fft}, DDC {tot_ddc}, disagreements {tot_disagree} of {present}");

    // A front end that loses sensitivity shows up as a one-sided
    // deficit. Paired trials make the threshold tight: over 560 files
    // straddling four crossings, more than a 2% net loss is a curve
    // shift, not scatter.
    let floor = tot_fft.saturating_sub(tot_fft / 50).saturating_sub(1);
    assert!(
        tot_ddc >= floor,
        "DDC recall {tot_ddc} against FFT {tot_fft} — below the {floor} floor"
    );
}

// ── What the swap is actually worth on embedded ──────────────────────────

/// The DDC arm decodes the golden with **no wideband cache at all**.
///
/// This is the claim the whole embedded FT4 line rests on, and it is
/// not the same claim as
/// `ft4_ddc_baseband_decodes_the_golden_like_the_fft_path` above. That
/// one shows the DDC's `cd0` is as good as `downsample_cached`'s; it
/// still hands `process_candidate_precomputed` a real `fft_cache`,
/// because the parameter is not optional. If anything downstream reads
/// that slice, the 92 160-point transform is still on the critical path
/// and the device still needs the 737 280-byte baked asset — the DDC
/// would have removed a cost and not a dependency.
///
/// Reading the code says it does not: `precomputed_refine = Some(..)`
/// takes the branch that skips `downsample_cached`
/// (`engine/pipeline.rs`), and the only other consumer is
/// `P::snr_db(SnrCtx { fft_cache, .. })`, which for FT4 is
/// `pipeline::ft4_snr_db(ctx.cand_score)` — a closed form over the
/// *coarse* candidate score (`ft4_decode.f90:226,452-457`, issue #255)
/// that never touches the spectrum. FST4 is the counter-example that
/// makes this worth pinning rather than assuming: `fst4_snr_db` calls
/// `downsample_cached` a second time on purpose, which is why
/// `process_candidate_precomputed` grew its `skip_snr` flag for issue
/// #306 in the first place.
///
/// So this test passes an **empty** slice. Reading it would panic on
/// the first index rather than quietly return something plausible, and
/// the results are compared field-by-field against the same arm run
/// with the real cache — a difference either way means FT4 does depend
/// on it and the embedded bench must keep baking it.
#[test]
fn ft4_ddc_arm_never_reads_the_wideband_cache() {
    let Some(audio) = slot_audio() else {
        if std::env::var("MFSK_REQUIRE_CORPUS").is_ok() {
            panic!("MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing");
        }
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };

    let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    assert!(!cands.is_empty(), "coarse stage found nothing");
    let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
    let empty: [Complex<f32>; 0] = [];

    for (depth_name, depth) in [
        ("EMBEDDED", DecodeDepth::EMBEDDED),
        ("FULL", DecodeDepth::FULL),
    ] {
        let mut with_cache: Vec<DecodeResult> = Vec::new();
        let mut without_cache: Vec<DecodeResult> = Vec::new();
        for c in &cands {
            if let (Some(r), _, _) = ddc_arm(c, &audio, &fft_cache, depth) {
                with_cache.push(r);
            }
            if let (Some(r), _, _) = ddc_arm(c, &audio, &empty, depth) {
                without_cache.push(r);
            }
        }

        eprintln!(
            "[{depth_name}] DDC arm: {} decodes with the wideband cache, {} with an empty slice",
            with_cache.len(),
            without_cache.len()
        );
        assert_eq!(
            with_cache.len(),
            without_cache.len(),
            "[{depth_name}] dropping the wideband cache changed how many candidates decoded"
        );
        for (a, b) in with_cache.iter().zip(&without_cache) {
            assert_eq!(msg_of(a), msg_of(b), "[{depth_name}] message differs");
            assert_eq!(a.freq_hz, b.freq_hz, "[{depth_name}] freq differs");
            assert_eq!(a.dt_sec, b.dt_sec, "[{depth_name}] dt differs");
            // Not `assert_eq!` on the whole struct: `snr_db` is the
            // field that would move if `ft4_snr_db` ever started
            // reading the spectrum, so it is named explicitly.
            assert_eq!(a.snr_db, b.snr_db, "[{depth_name}] snr_db differs");
        }
        // Guards against the whole test passing vacuously if the coarse
        // stage or the DDC front end silently stopped producing
        // anything — 11 is what the arm-A/arm-B test above pins.
        assert_eq!(
            sorted_messages(&without_cache).len(),
            11,
            "[{depth_name}] cacheless DDC arm should reach the same 11 distinct decodes"
        );
    }
}
