//! `DecodeRequest::<Ft8>::budget` — the cheapest-first scheduler, and
//! what it is allowed to do when the budget runs out.
//!
//! ## Why the budget is counted in candidates, not milliseconds
//!
//! The shipped knob is wall-clock: a caller closes over a deadline and
//! the predicate compares a clock against it. A test cannot use that.
//! This host is ~50× a CoreS3, CI runners are shared, and a
//! millisecond threshold that means "cut about half the candidates"
//! here means "cut nothing" or "cut everything" somewhere else — the
//! test would assert on the machine, not on the scheduler.
//!
//! Because the API takes a *closure* rather than a clock, the
//! device-independent unit is free: a predicate backed by an
//! `AtomicI64` counter returns `false` after exactly N calls, and the
//! scheduler polls exactly once per candidate it is about to start. N
//! is therefore "candidates allowed", deterministic on every machine
//! and every thread count. (`fst4_monitor_cap_sensitivity` had to
//! reach for the same trick through a global, because
//! `rung_major`'s budget is a bare `fn` pointer that cannot capture.)
//!
//! ## What is asserted
//!
//! 1. A budget that never fires reproduces the unbudgeted decode
//!    exactly — same messages, same freq/dt/SNR. This is the one that
//!    catches a scheduler that reorders the first-wins dedup, which
//!    would silently change a decode's reported frequency rather than
//!    its presence.
//! 2. Recall is monotone in the budget: more candidates allowed never
//!    decodes fewer messages, and every budgeted result set is a subset
//!    of the unbudgeted one.
//! 3. **Precision at every budget**: a truncated ladder may only lose
//!    decodes, never invent one. `CONTRIBUTING.md`'s rule that a new
//!    strategy ships with its precision guard — both false-decode bugs
//!    this crate has shipped were in non-default strategies.
//! 4. The SIC strategies never emit the same message twice under a cut.
//!    That is the direct test for polling *before* a candidate rather
//!    than between accepting a decode and subtracting it: a cut in that
//!    window leaves an unsubtracted signal in the residual for the next
//!    round's `coarse_sync` to re-find.
//!
//! Run:
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test --release -p mfsk-core \
//!     --features full,internal-testing \
//!     --test ft8_budget_scheduler -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;
use std::sync::atomic::{AtomicI64, Ordering};

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::ft8_qso3::{DF_TOL_HZ, QSO3_KNOWN_REAL_SIGNALS, SNR_TOL_DB};
use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Host research config, the same one `ft8_qso3_jtdx_recall` and
/// `ft8_sweep` drive `DecodeRequest` with.
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3000.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 60;

/// A budget that allows exactly `n` candidates and then stops.
///
/// One `false` is enough — the scheduler latches `exhausted` — but this
/// keeps returning `false` so the test does not depend on that detail.
struct CandidateBudget(AtomicI64);

impl CandidateBudget {
    fn new(n: i64) -> Self {
        Self(AtomicI64::new(n))
    }
    fn check(&self) -> bool {
        self.0.fetch_sub(1, Ordering::SeqCst) > 0
    }
}

fn decode_with_budget(
    audio: &[i16],
    budget: Option<&CandidateBudget>,
) -> (
    Vec<DecodeResult>,
    mfsk_core::msg::decode_request::BudgetReport,
) {
    let req =
        DecodeRequest::<Ft8>::new(audio, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND).single_pass();
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

fn messages(results: &[DecodeResult]) -> Vec<String> {
    results
        .iter()
        .map(|d| unpack77(d.message77()).unwrap_or_default())
        .collect()
}

fn view(d: &DecodeResult) -> DecodeView {
    DecodeView {
        msg: unpack77(d.message77()).unwrap_or_default(),
        freq_hz: d.freq_hz,
        dt_sec: d.dt_sec,
        snr_db: Some(d.snr_db),
    }
}

#[test]
fn budget_that_never_fires_reproduces_the_unbudgeted_decode() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let (plain, plain_report) = decode_with_budget(&slot, None);
    assert_eq!(
        plain_report,
        Default::default(),
        "an unbudgeted decode must report nothing"
    );

    // Far more than `MAX_CAND`, so the predicate is never the reason
    // anything stops.
    let generous = CandidateBudget::new(10_000);
    let (budgeted, report) = decode_with_budget(&slot, Some(&generous));

    assert!(!report.exhausted, "budget of 10 000 candidates was hit");
    assert_eq!(report.candidates_skipped, 0);
    assert!(report.stages_run > 0, "scheduler ran no candidates at all");

    assert_eq!(
        messages(&budgeted),
        messages(&plain),
        "scheduled path changed the decode set (or its order)"
    );
    // Not just the messages: a reordered first-wins dedup keeps the
    // same message while changing which candidate's measurements
    // survive, which is the failure mode the re-sort exists to prevent.
    for (b, p) in budgeted.iter().zip(plain.iter()) {
        assert_eq!(b.freq_hz, p.freq_hz, "{}", unpack77(p.message77()).unwrap());
        assert_eq!(b.dt_sec, p.dt_sec, "{}", unpack77(p.message77()).unwrap());
        assert_eq!(b.snr_db, p.snr_db, "{}", unpack77(p.message77()).unwrap());
    }
}

#[test]
fn recall_is_monotone_in_the_budget_and_never_invents_a_decode() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let (plain, _) = decode_with_budget(&slot, None);
    let full: Vec<String> = messages(&plain);

    let mut prev = 0usize;
    for n in [0i64, 1, 2, 4, 8, 16, 32, 64] {
        let budget = CandidateBudget::new(n);
        let (results, report) = decode_with_budget(&slot, Some(&budget));
        let got = messages(&results);

        println!(
            "budget {n:>3} candidates -> {:>2} decodes, stages_run={}, skipped={}, cut_at_sync={:?}",
            got.len(),
            report.stages_run,
            report.candidates_skipped,
            report.cut_at_sync
        );

        assert!(
            report.stages_run as i64 <= n,
            "ran {} candidates on a budget of {n}",
            report.stages_run
        );
        for m in &got {
            assert!(
                full.contains(m),
                "budget {n} produced {m:?}, which the unbudgeted decode does not — \
                 a truncated ladder may only lose decodes, never invent one"
            );
        }
        assert!(
            got.len() >= prev,
            "budget {n} decoded {} messages, fewer than the {prev} a smaller budget did",
            got.len()
        );
        prev = got.len();
    }
    assert_eq!(
        prev,
        full.len(),
        "a budget of 64 candidates (> max_cand {MAX_CAND}) should reach the full set"
    );
}

#[test]
fn a_cut_budget_still_meets_the_precision_bar() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    // Deliberately mid-range: enough to decode the strong stations,
    // not enough to finish the list.
    let budget = CandidateBudget::new(8);
    let (results, report) = decode_with_budget(&slot, Some(&budget));
    assert!(report.exhausted, "budget of 8 candidates was not reached");

    assert_golden(
        &results,
        &GoldenSet {
            name: "FT8 qso3_busy.wav, budget = 8 candidates",
            expected: QSO3_KNOWN_REAL_SIGNALS,
            // Recall is not the property here — the budget is *meant*
            // to cost decodes. `max_extra: 0` is: whatever it does
            // return must be real.
            min_hits: 1,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: DF_TOL_HZ,
            dt_sec: Tolerances::default().dt_sec,
            snr_db: SNR_TOL_DB,
        },
        view,
    );
}

#[test]
fn an_exhausted_budget_reports_what_it_cut() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // Zero candidates: the triage sweep still runs in full — it is what
    // produces the ordering, and gating it would make the schedule
    // depend on frequency order — but no ladder is started, so FT8
    // returns nothing. (FST4's phase A includes a BP attempt and so
    // decodes at budget zero; FT8's cheapest boundary is before any BP.
    // The divergence is deliberate.)
    let budget = CandidateBudget::new(0);
    let (results, report) = decode_with_budget(&slot, Some(&budget));

    assert!(
        results.is_empty(),
        "budget of 0 candidates decoded something"
    );
    assert!(report.exhausted);
    assert_eq!(report.stages_run, 0);
    assert!(
        report.candidates_skipped > 0,
        "nothing was reported as skipped"
    );
    let cut_sync = report.cut_at_sync.expect("no cut_at_sync reported");
    assert!(
        cut_sync > 6,
        "the first candidate cut had nsync {cut_sync}, which the triage gate should have rejected"
    );
    assert!(
        report.cut_at_score.is_some_and(|s| s >= SYNC_MIN),
        "cut_at_score {:?} is below the search's own sync_min",
        report.cut_at_score
    );
    println!(
        "budget 0: skipped {} candidates, best was nsync={cut_sync} score={:?}",
        report.candidates_skipped, report.cut_at_score
    );
}

#[test]
fn sic_strategies_never_repeat_a_message_when_cut() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // A cut between accepting a decode and subtracting it would leave
    // that signal in the residual for the next round to re-find. Sweep
    // the cut point across the whole range so it lands inside a round,
    // at a round boundary, and past the end.
    for n in [0i64, 1, 3, 7, 15, 31, 63, 127] {
        for early in [false, true] {
            let budget = CandidateBudget::new(n);
            let check = || budget.check();
            let req = DecodeRequest::<Ft8>::new(&slot, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND)
                .budget(&check);
            let out = if early {
                req.sic_early().decode()
            } else {
                req.sic_rounds(3).decode()
            };
            let msgs = messages(&out.results);
            let mut sorted = msgs.clone();
            sorted.sort();
            let before = sorted.len();
            sorted.dedup();
            assert_eq!(
                before,
                sorted.len(),
                "budget {n} on {} produced a duplicate message: {msgs:?}",
                if early {
                    ".sic_early()"
                } else {
                    ".sic_rounds(3)"
                }
            );
            assert!(
                n > 0 || out.results.is_empty(),
                "budget of 0 candidates decoded something"
            );
        }
    }
}
