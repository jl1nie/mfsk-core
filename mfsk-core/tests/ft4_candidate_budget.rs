//! How many coarse candidates does FT4 actually need? — the embedded
//! FT4 budget line (`docs/notes/FT4_BENCHMARK.md` §17-20).
//!
//! Every stage after `ft4_coarse_sync` is per-candidate:
//! `ft4_sync_search`, the LLR ladder, BP, OSD. On a CoreS3 the measured
//! slot is 31 candidates × (80 ms search + 63 ms LLR/BP) against a
//! 1 960 ms budget, so the candidate count is not one lever among
//! several — it multiplies every other one. Nothing had measured it.
//!
//! `ft4_coarse_sync` ranks by score and truncates to `max_cand`
//! (`engine::sync::rank_candidates`, with `freq_hint = None` here so
//! the order is score alone), which makes the question answerable in
//! one pass: decode candidates **in rank order** and record the rank at
//! which each decode first appears. Recall at any smaller `max_cand` is
//! then read straight off those ranks, with no re-run per budget.
//! `sync_min` only filters and never reorders, so the same pass answers
//! it too.
//!
//! ## What it found (2026-08-30)
//!
//! **`sync_min` is the lever, and the value that works is WSJT-X's
//! own.** `ft4_decode.f90:195` sets `syncmin = 1.2`; the bench and the
//! sweep harness had been passing 0.05 and 0.8. Those are not merely
//! looser — they are *below the noise floor*, because
//! `getcandidates4.f90` divides the smoothed spectrum by a fitted
//! baseline, which puts noise at ~1.0 by construction. Every peak in
//! the band therefore qualifies:
//!
//! | `sync_min` | mean candidates (560 weak files) | golden (14 signals) | recall |
//! |---:|---:|---:|---|
//! | 0.05 – 0.80 | 67.1 | 31 | full |
//! | 1.00 | 54.8 | 28 | full |
//! | 1.10 | 12.1 | 12 | full |
//! | **1.20** (upstream) | **1.6** | **12** | **full** |
//! | 1.30 | 1.0 | 12 | full |
//! | 1.40 | 0.8 | 12 | −2/237 |
//! | 1.70 | 0.4 | 11 | −63/237 |
//!
//! So the faithful value is also 2.6× cheaper on a crowded real
//! recording and 42× cheaper on a sparse one, at zero measured recall
//! cost on either — and the knee is close enough (1.4 starts costing)
//! that it should not be pushed further without re-running this.
//!
//! `max_cand` is not a lever on this evidence: on the golden the
//! deepest decoding candidate is rank 11 of 31, and on the sweep corpus
//! every decode is rank 0 — but that second number is a property of the
//! fixture (one signal per file), not a finding, so `max_cand` is
//! bounded only by the golden's 12.
//!
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core \
//!   --features full,internal-testing --release \
//!   --test ft4_candidate_budget -- --nocapture
//! ```

#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::engine::dsp::downsample::build_fft_cache;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
use mfsk_core::engine::pipeline::{
    DecodeDepth, DecodeResult, DecodeStrictness, process_candidate_basic,
};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const SLOT_SAMPLES: usize = 90_000;
/// `ft4_wsjtx_samples::bench_assets`, mirrored — the search the
/// on-device `ft4-bench` runs.
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 2700.0;
/// Deliberately *below* production's `sync_min` (WSJT-X's 1.2, now
/// `bench_assets::SYNC_MIN`): the ranking question — where do real
/// decodes sit among all peaks — is what bounds `max_cand`, and it is
/// answered on the unfiltered list. The threshold itself is the subject
/// of `ft4_sync_min_trades_candidate_count_against_nothing_until_it_bites`.
const SYNC_MIN: f32 = 0.05;
const MAX_CAND: usize = 100;
const SYNC_Q_MIN: u32 = 8;

fn golden_audio() -> Option<Vec<i16>> {
    let path = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = load_wav_i16_opt(&path)?;
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

fn decode_at(
    cand: &mfsk_core::engine::sync::SyncCandidate,
    fft_cache: &[num_complex::Complex<f32>],
) -> Option<DecodeResult> {
    decode_at_depth(cand, fft_cache, DecodeDepth::EMBEDDED)
}

fn decode_at_depth(
    cand: &mfsk_core::engine::sync::SyncCandidate,
    fft_cache: &[num_complex::Complex<f32>],
    depth: DecodeDepth,
) -> Option<DecodeResult> {
    process_candidate_basic::<Ft4>(
        cand,
        fft_cache,
        &FT4_DOWNSAMPLE,
        depth,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        SYNC_Q_MIN,
    )
}

/// Where in the score ranking the real decodes actually live, on the
/// WSJT-X golden — and therefore what `max_cand` would cost.
///
/// Written as a ratchet, not a diagnostic: the number printed is also
/// asserted, so a change that pushes a true signal further down the
/// coarse ranking fails here instead of silently raising the candidate
/// budget an embedded caller has to pay.
#[test]
fn ft4_golden_decodes_live_near_the_top_of_the_ranking() {
    let Some(audio) = golden_audio() else {
        if std::env::var("MFSK_REQUIRE_CORPUS").is_ok() {
            panic!("MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing");
        }
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };

    let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);

    // Rank at which each distinct message first decodes.
    let mut first_rank: Vec<(usize, String, f32, f32)> = Vec::new();
    for (rank, c) in cands.iter().enumerate() {
        let Some(d) = decode_at(c, &fft_cache) else {
            continue;
        };
        let msg = unpack77(d.message77()).unwrap_or_default();
        if !first_rank.iter().any(|(_, m, _, _)| *m == msg) {
            first_rank.push((rank, msg, c.freq_hz, c.score));
        }
    }

    eprintln!(
        "{} candidates, {} distinct decodes",
        cands.len(),
        first_rank.len()
    );
    for (rank, msg, freq, score) in &first_rank {
        eprintln!("  rank {rank:>2}  {freq:>7.1} Hz  score {score:>7.3}  {msg}");
    }

    let deepest = first_rank.iter().map(|(r, ..)| *r).max().unwrap_or(0);
    eprintln!(
        "deepest rank carrying a decode: {deepest} -> max_cand = {} suffices \
         ({} of {} candidates are dead weight)",
        deepest + 1,
        cands.len() - (deepest + 1),
        cands.len()
    );

    assert_eq!(
        first_rank.len(),
        11,
        "single-pass golden recall moved — fix `ft4_wsjtx_samples` first"
    );
    // Measured 2026-08-30: the deepest decoding candidate is rank 11,
    // so `max_cand = 12` reproduces every single-pass decode this
    // recording yields and **19 of the 31 candidates — 61 % — pay full
    // search + LLR + BP cost for nothing**. Ratchet, not a target: if a
    // true signal starts ranking deeper than this, the coarse stage's
    // ordering has regressed and an embedded caller's budget grows with
    // it silently. Slack of 3 so an unrelated scoring tweak does not
    // trip it on noise.
    assert!(
        deepest <= 14,
        "deepest decoding candidate is rank {deepest}, was 11 — the coarse ranking regressed"
    );
}

/// `sync_min` is the other half of the budget: it gates peaks before
/// ranking, so it changes how many candidates exist at all rather than
/// how many survive truncation. Printed as a table because the right
/// value is a judgement about reach, not a threshold to assert — the
/// only assertion is that the production value does not throw decodes
/// away.
#[test]
fn ft4_sync_min_trades_candidate_count_against_nothing_until_it_bites() {
    let Some(audio) = golden_audio() else {
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);

    eprintln!("sync_min  candidates  distinct decodes");
    let mut at_production = 0usize;
    for &sync_min in &[0.05f32, 0.8, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.7, 2.0, 4.0] {
        let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, sync_min, None, MAX_CAND);
        let mut msgs: Vec<String> = Vec::new();
        for c in &cands {
            if let Some(d) = decode_at(c, &fft_cache) {
                let m = unpack77(d.message77()).unwrap_or_default();
                if !msgs.contains(&m) {
                    msgs.push(m);
                }
            }
        }
        eprintln!("{sync_min:>7.2}  {:>10}  {:>16}", cands.len(), msgs.len());
        if sync_min == SYNC_MIN {
            at_production = msgs.len();
        }
    }
    assert_eq!(at_production, 11, "production sync_min lost decodes");
}

// ── Tier C: does a weak signal need the deep candidates? ─────────────────

/// Golden signal in the generated corpus, same constants `ft4_sweep.rs`
/// and `ft4_ddc_equivalence.rs` use.
const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;

const SWEEP_CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];
const SWEEP_TAGS: &[&str] = &["m14", "m15", "m16", "m17", "m18", "m19", "m20"];
const SWEEP_TRIALS: u32 = 20;
const SWEEP_FREQ_MAX_HZ: f32 = 3000.0;
/// Candidate budgets to read off the ranking.
const BUDGETS: &[usize] = &[4, 8, 12, 16, 20, 24, 31, 50, 100];
/// `sync_min` values to re-gate the coarse stage with.
const SYNC_MINS: &[f32] = &[0.05, 0.8, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.7, 2.0];

fn sweep_dir() -> std::path::PathBuf {
    if let Ok(d) = std::env::var("MFSK_FT4_SWEEP_DIR") {
        return std::path::PathBuf::from(d);
    }
    std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default())
        .join("../embedded-poc/assets/ft4_sweep")
        .to_path_buf()
}

/// The golden-recording answer above is one file of mostly comfortable
/// signals. The budget question that decides an embedded search is the
/// weak one: **near the 50 % crossing, how deep in the ranking does the
/// true signal sit?** A candidate budget that is free at −5 dB and
/// costs a dB at −17 would be a sensitivity regression sold as an
/// optimisation.
///
/// One pass per file yields both curves at once. Candidates are ranked
/// by score and `max_cand` only truncates, so the rank at which the
/// golden message first decodes gives recall at *every* budget; and
/// `sync_min` only filters (it never reorders), so re-running the
/// coarse stage at each threshold and testing whether that same
/// candidate frequency survives gives the `sync_min` curve exactly,
/// with no second decode.
///
/// Only near-golden candidates are decoded — the others cannot produce
/// the golden message — but the **rank is taken from the full list**,
/// which is what an embedded caller would actually pay for.
///
/// `#[ignore]` — tier C, needs the ft4_sweep corpus:
///
/// ```sh
/// cargo test -p mfsk-core --features full,internal-testing --release \
///   --test ft4_candidate_budget -- --ignored --nocapture
/// ```
#[test]
#[ignore = "tier C — needs the ft4_sweep corpus; run with --ignored --nocapture"]
fn ft4_candidate_budget_across_the_crossing() {
    use rayon::prelude::*;

    let dir = sweep_dir();
    let mut work: Vec<(usize, &str, &str, u32)> = Vec::new();
    for (ci, &ch) in SWEEP_CHANNELS.iter().enumerate() {
        for &tag in SWEEP_TAGS {
            for trial in 1..=SWEEP_TRIALS {
                work.push((ci, ch, tag, trial));
            }
        }
    }

    /// Per file: `(present, n_cands, first decoding rank at `FULL`,
    /// same at `EMBEDDED`, sync_min survival, candidate count at each
    /// sync_min)`.
    type Row = (bool, usize, Option<usize>, bool, Vec<bool>, Vec<usize>);

    let per_file: Vec<Row> = work
        .par_iter()
        .map(|&(_, ch, tag, trial)| {
            let path = dir.join(format!("ft4_{ch}_{tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                return (
                    false,
                    0,
                    None,
                    false,
                    vec![false; SYNC_MINS.len()],
                    vec![0; SYNC_MINS.len()],
                );
            };
            let cands = ft4_coarse_sync(
                &audio,
                FREQ_MIN_HZ,
                SWEEP_FREQ_MAX_HZ,
                SYNC_MINS[0],
                None,
                MAX_CAND,
            );
            let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);

            let is_golden = |d: DecodeResult| -> bool {
                unpack77(d.message77()).as_deref() == Some(GOLDEN_MSG)
                    && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
            };
            let mut hit: Option<(usize, f32)> = None;
            let mut hit_embedded = false;
            for (rank, c) in cands.iter().enumerate() {
                if (c.freq_hz - GOLDEN_FREQ_HZ).abs() > FREQ_TOL_HZ {
                    continue;
                }
                if !hit_embedded {
                    hit_embedded = decode_at_depth(c, &fft_cache, DecodeDepth::EMBEDDED)
                        .is_some_and(is_golden);
                }
                if hit.is_none()
                    && decode_at_depth(c, &fft_cache, DecodeDepth::FULL).is_some_and(is_golden)
                {
                    hit = Some((rank, c.freq_hz));
                }
                if hit.is_some() && hit_embedded {
                    break;
                }
            }

            // `sync_min` only filters, so the winning candidate either
            // survives a given threshold or does not — and the count of
            // what is left is the cost side of the same curve.
            let mut survives = Vec::with_capacity(SYNC_MINS.len());
            let mut counts = Vec::with_capacity(SYNC_MINS.len());
            for &s in SYNC_MINS {
                let filtered =
                    ft4_coarse_sync(&audio, FREQ_MIN_HZ, SWEEP_FREQ_MAX_HZ, s, None, MAX_CAND);
                survives.push(hit.is_some_and(|(_, freq)| {
                    filtered.iter().any(|c| (c.freq_hz - freq).abs() < 0.01)
                }));
                counts.push(filtered.len());
            }

            (
                true,
                cands.len(),
                hit.map(|(r, _)| r),
                hit_embedded,
                survives,
                counts,
            )
        })
        .collect();

    let present = per_file.iter().filter(|r| r.0).count();
    if present == 0 {
        eprintln!("skipping: no FT4 sweep corpus under {}", dir.display());
        return;
    }
    assert_eq!(
        present,
        work.len(),
        "corpus is incomplete ({present}/{})",
        work.len()
    );

    let mean_cands = per_file.iter().map(|r| r.1).sum::<usize>() as f32 / per_file.len() as f32;
    let hits: Vec<usize> = per_file.iter().filter_map(|r| r.2).collect();
    eprintln!(
        "{} files, mean {mean_cands:.1} candidates each, {} decode the golden signal",
        per_file.len(),
        hits.len()
    );

    eprintln!("\nmax_cand   decodes   vs full");
    let full = hits.len();
    for &k in BUDGETS {
        let n = hits.iter().filter(|&&r| r < k).count();
        eprintln!("{k:>8}   {n:>7}   {:>+6}", n as i32 - full as i32);
    }

    eprintln!("\nsync_min   mean cands   decodes   vs full");
    for (i, &s) in SYNC_MINS.iter().enumerate() {
        let n = per_file.iter().filter(|r| r.4[i]).count();
        let mean = per_file.iter().map(|r| r.5[i]).sum::<usize>() as f32 / per_file.len() as f32;
        eprintln!(
            "{s:>8.2}   {mean:>10.1}   {n:>7}   {:>+6}",
            n as i32 - full as i32
        );
    }

    // Depth is not the subject here, but the same pass answers it for
    // free, and the golden recording's "EMBEDDED == FULL" does not
    // survive contact with weak data.
    let n_embedded = per_file.iter().filter(|r| r.3).count();
    eprintln!(
        "\nDecodeDepth::FULL {full} vs EMBEDDED {n_embedded} \
         ({:+} — OSD near the crossing, unlike on the golden)",
        n_embedded as i32 - full as i32
    );

    // Rank histogram — where the weak signal actually sits.
    eprintln!("\nrank of the decoding candidate:");
    for (lo, hi) in [
        (0usize, 0usize),
        (1, 3),
        (4, 7),
        (8, 15),
        (16, 31),
        (32, 999),
    ] {
        let n = hits.iter().filter(|&&r| r >= lo && r <= hi).count();
        if n > 0 {
            eprintln!("  {lo:>3}..{hi:<3}  {n:>4}");
        }
    }

    assert!(
        full > 0,
        "no file decoded at all — the corpus or the search is wrong"
    );
}
