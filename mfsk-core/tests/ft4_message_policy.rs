//! The message-acceptance policy on the **generic** pipeline
//! (issue #383, step 5a).
//!
//! FT8 reaches its text stage inside its own bespoke engine; FT4 and
//! every FST4 sub-mode reach theirs through
//! `engine::pipeline::InfoAccept`, a seam that exists because `engine`
//! cannot depend on `msg` and so cannot unpack a codeword itself. This
//! file holds that seam down from the outside:
//!
//! 1. **A no-op policy is bit-identical to the default**, which for
//!    FST4 is a stronger claim than for FT8: `MESSAGE_FILTER_DEFAULT`
//!    is `false` there, so the seam must not even build the message
//!    string. `PolicyAccept` folds itself away on two compile-time
//!    constants; the behavioural half is checked here. FT4 joined FT8
//!    in running the verdict by default on 2026-09-21 — see
//!    `Ft4::MESSAGE_FILTER_DEFAULT` for the 720-slot measurement.
//! 2. **`.message_filter(|_| false)` reaches every rung.** Not a
//!    tautology: the pipeline accepts candidates at four separate
//!    points (BP ladder, OSD-2/3, OSD-4 Top-K, a-priori), and an
//!    unwired one would leak rows.
//! 3. **`.also_accept()` can only widen.** Over a verdict that already
//!    passes a row it is a no-op, which is what lets `.also_accept(f)`
//!    mean "the usual filter, plus this" on every protocol, whether or
//!    not that protocol filters by default.
//! 4. **`.sic_rounds()` carries the policy too.** FT4's SIC path runs
//!    a second engine; before this it called the `AcceptAll` wrapper
//!    and would have ignored the caller silently.
//!
//! Run:
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test --release -p mfsk-core \
//!     --features full,internal-testing --test ft4_message_policy
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "ft4"))]

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::{DecodeOutcome, DecodeRequest};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s @ 12 kHz

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

fn req(audio: &[i16]) -> DecodeRequest<'_, Ft4> {
    DecodeRequest::<Ft4>::new(audio, 300.0, 2800.0, 1.2, 100)
}

fn rows(out: &DecodeOutcome<Ft4>) -> Vec<String> {
    let mut v: Vec<String> = out
        .results
        .iter()
        .map(|r| {
            format!(
                "{}|{:.3}|{:.3}|{}",
                unpack77(r.message77()).unwrap_or_default(),
                r.freq_hz,
                r.dt_sec,
                r.pass
            )
        })
        .collect();
    v.sort();
    v
}

#[test]
fn a_no_op_policy_changes_nothing() {
    let Some(a) = slot_audio() else {
        common::corpus::missing("ft4_message_policy", "ft4/000000_000002.wav");
        return;
    };
    let base = rows(&req(&a).decode());
    assert!(!base.is_empty(), "the golden recording must decode");

    // FT4 runs the verdict by default now, so asking for it explicitly
    // is the default.
    let verdict = rows(&req(&a).codec_filter().decode());
    assert_eq!(base, verdict, "codec_filter is not FT4's default");

    // And the verdict must still cost this recording nothing — half of
    // it is ARRL RTTY Roundup, which the verdict refused outright until
    // issue #383 gave the type a structural rule. Turning the filter
    // *off* is what shows that: every row the default returns has to
    // survive here too, and on this recording there is nothing extra to
    // find, because a clean WSJT-X sample has no CRC-14 false positives
    // in it.
    let all = rows(&req(&a).message_filter(|_| true).decode());
    assert_eq!(base, all, "the codec verdict dropped an FT4 decode");

    let widened = rows(&req(&a).also_accept(|_| false).decode());
    assert_eq!(
        base, widened,
        "also_accept over a passing verdict is a no-op"
    );
}

/// Four acceptance points in `process_candidate_basic_impl`; an unwired
/// one leaks rows here.
#[test]
fn a_rejecting_filter_reaches_every_rung() {
    let Some(a) = slot_audio() else {
        common::corpus::missing("ft4_message_policy", "ft4/000000_000002.wav");
        return;
    };
    let none = req(&a).message_filter(|_| false).decode();
    assert!(
        none.results.is_empty(),
        "a rung is not wired: {} rows survived a reject-all filter",
        none.results.len()
    );

    // The SIC strategy runs its own engine — it used to call the
    // `AcceptAll` wrapper and would have ignored the caller.
    let none_sic = req(&a).message_filter(|_| false).sic_rounds(2).decode();
    assert!(
        none_sic.results.is_empty(),
        "the SIC path ignored the policy: {} rows",
        none_sic.results.len()
    );
}

#[test]
fn a_permissive_filter_can_only_add() {
    let Some(a) = slot_audio() else {
        common::corpus::missing("ft4_message_policy", "ft4/000000_000002.wav");
        return;
    };
    let base = rows(&req(&a).decode());
    let all = rows(&req(&a).message_filter(|_| true).decode());
    for r in &base {
        assert!(all.contains(r), "removing the filter lost {r}");
    }
    assert!(all.len() >= base.len());
}
