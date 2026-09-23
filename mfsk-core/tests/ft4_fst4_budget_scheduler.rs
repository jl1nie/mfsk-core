//! `DecodeRequest::budget` on the shared generic engine — FT4 and FST4.
//!
//! The FT8 counterpart (`ft8_budget_scheduler.rs`) carries the full
//! argument for why the budget is counted in candidates rather than
//! milliseconds, and what each assertion is for. This file is the same
//! properties on the engine FT4 and FST4 share, where the priority key
//! is different for each and neither is FT8's `nsync`:
//!
//! - **FT4** needs no reordering. `ft4_coarse_sync` already ranks its
//!   candidates by sync score (`engine::sync::rank_candidates`), so
//!   polling in the existing order declines the weakest rather than the
//!   top of the band.
//! - **FST4** reorders. `dedup_refined_candidates` has already run
//!   `fst4_sync_search` over every candidate to suppress near-duplicates,
//!   so a *refined* score — sharper than the coarse one, and free — is
//!   in hand for all survivors, but they come back in candidate order.
//!
//! Both engines' cross-candidate dedup keeps the highest `sync_score`
//! per message rather than the first-processed one, so unlike FT8 the
//! reordering cannot change which candidate's measurements survive.
//! That is why there is no re-sort here and no `freq_hz` identity check:
//! the dedup rule already makes the result order-independent.
//!
//! Run:
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test --release -p mfsk-core \
//!     --features full,internal-testing \
//!     --test ft4_fst4_budget_scheduler -- --nocapture
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "ft4", feature = "fst4"))]

use std::path::PathBuf;
use std::sync::atomic::{AtomicI64, Ordering};

use mfsk_core::engine::pipeline::DecodeResult;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::{BudgetReport, DecodeRequest};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const FT4_SLOT_SAMPLES: usize = 90_000; // 7.5 s x 12 kHz

fn ft4_sample() -> Option<Vec<i16>> {
    let path: PathBuf = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = load_wav_i16_opt(&path)?;
    let mut audio = vec![0i16; FT4_SLOT_SAMPLES];
    let copy = raw.len().min(FT4_SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

fn fst4_sample() -> Option<Vec<i16>> {
    let path: PathBuf = common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    )?;
    load_wav_i16_opt(&path)
}

/// Allows exactly `n` units of work, then stops. See the FT8 file for
/// why a counter and not a clock.
struct CandidateBudget(AtomicI64);

impl CandidateBudget {
    fn new(n: i64) -> Self {
        Self(AtomicI64::new(n))
    }
    fn check(&self) -> bool {
        self.0.fetch_sub(1, Ordering::SeqCst) > 0
    }
}

fn messages(results: &[DecodeResult]) -> Vec<String> {
    let mut v: Vec<String> = results
        .iter()
        .map(|d| unpack77(d.message77()).unwrap_or_default())
        .collect();
    v.sort();
    v
}

fn ft4_decode(
    audio: &[i16],
    budget: Option<&CandidateBudget>,
) -> (Vec<DecodeResult>, BudgetReport) {
    let req = DecodeRequest::<Ft4>::new(audio, 300.0, 2700.0, 1.2, 50);
    let check;
    let req = match budget {
        None => req,
        Some(b) => {
            check = move || b.check();
            req.budget(&check)
        }
    };
    let out = req.decode();
    (out.results, out.budget)
}

fn fst4_decode(
    audio: &[i16],
    budget: Option<&CandidateBudget>,
) -> (Vec<DecodeResult>, BudgetReport) {
    let req = DecodeRequest::<Fst4s60>::new(audio, 100.0, 3000.0, 0.8, 50);
    let check;
    let req = match budget {
        None => req,
        Some(b) => {
            check = move || b.check();
            req.budget(&check)
        }
    };
    let out = req.decode();
    (out.results, out.budget)
}

#[test]
fn ft4_budget_that_never_fires_reproduces_the_unbudgeted_decode() {
    let Some(audio) = ft4_sample() else {
        common::skip_or_fail("FT4 golden not present");
        return;
    };
    let (plain, plain_report) = ft4_decode(&audio, None);
    assert_eq!(plain_report, BudgetReport::default());

    let generous = CandidateBudget::new(10_000);
    let (budgeted, report) = ft4_decode(&audio, Some(&generous));
    assert!(!report.exhausted);
    assert_eq!(report.candidates_skipped, 0);
    assert!(report.stages_run > 0);
    assert_eq!(messages(&budgeted), messages(&plain));
}

#[test]
fn ft4_recall_is_monotone_and_never_invents_a_decode() {
    let Some(audio) = ft4_sample() else {
        common::skip_or_fail("FT4 golden not present");
        return;
    };
    let full = messages(&ft4_decode(&audio, None).0);

    let mut prev = 0usize;
    for n in [0i64, 1, 2, 4, 8, 16, 32, 64] {
        let budget = CandidateBudget::new(n);
        let (results, report) = ft4_decode(&audio, Some(&budget));
        let got = messages(&results);
        println!(
            "FT4 budget {n:>3} -> {:>2} decodes, ran={}, skipped={}, cut_at_score={:?}",
            got.len(),
            report.stages_run,
            report.candidates_skipped,
            report.cut_at_score
        );
        assert!(report.stages_run as i64 <= n);
        for m in &got {
            assert!(
                full.contains(m),
                "FT4 budget {n} produced {m:?}, which the unbudgeted decode does not"
            );
        }
        assert!(got.len() >= prev, "FT4 budget {n} went backwards");
        prev = got.len();
    }
    assert_eq!(prev, full.len(), "a large budget should reach the full set");
}

#[test]
fn ft4_sic_rounds_are_declined_whole_and_never_repeat_a_message() {
    let Some(audio) = ft4_sample() else {
        common::skip_or_fail("FT4 golden not present");
        return;
    };
    // The generic SIC engine subtracts a whole round's accepted decodes
    // as one batch, so a round is the granularity it can decline — the
    // budget must never cut inside one.
    for n in [0i64, 1, 2, 3, 8] {
        let budget = CandidateBudget::new(n);
        let check = || budget.check();
        let out = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50)
            .sic_rounds(3)
            .budget(&check)
            .decode();
        let msgs = messages(&out.results);
        let mut dedup = msgs.clone();
        dedup.dedup();
        assert_eq!(
            msgs.len(),
            dedup.len(),
            "FT4 SIC budget {n} duplicated a message"
        );
        assert!(
            out.budget.stages_run as i64 <= n.min(3),
            "FT4 SIC ran {} rounds on a budget of {n}",
            out.budget.stages_run
        );
        println!(
            "FT4 .sic_rounds(3) budget {n} -> {} decodes, rounds={}",
            msgs.len(),
            out.budget.stages_run
        );
    }
}

#[test]
fn fst4_budget_orders_by_refined_score_and_never_invents_a_decode() {
    let Some(audio) = fst4_sample() else {
        common::skip_or_fail("FST4 golden not present");
        return;
    };
    let (plain, plain_report) = fst4_decode(&audio, None);
    assert_eq!(plain_report, BudgetReport::default());
    let full = messages(&plain);

    let generous = CandidateBudget::new(10_000);
    let (budgeted, report) = fst4_decode(&audio, Some(&generous));
    assert!(!report.exhausted);
    assert_eq!(
        messages(&budgeted),
        full,
        "reordering by refined score changed the FST4 decode set"
    );

    let mut prev = 0usize;
    for n in [0i64, 1, 2, 4, 8, 16, 64] {
        let budget = CandidateBudget::new(n);
        let (results, report) = fst4_decode(&audio, Some(&budget));
        let got = messages(&results);
        println!(
            "FST4-60 budget {n:>3} -> {:>2} decodes, ran={}, skipped={}, cut_at_score={:?}",
            got.len(),
            report.stages_run,
            report.candidates_skipped,
            report.cut_at_score
        );
        assert!(report.stages_run as i64 <= n);
        for m in &got {
            assert!(
                full.contains(m),
                "FST4 budget {n} produced {m:?}, which the unbudgeted decode does not"
            );
        }
        assert!(got.len() >= prev, "FST4 budget {n} went backwards");
        prev = got.len();
    }
    assert_eq!(prev, full.len());
}
