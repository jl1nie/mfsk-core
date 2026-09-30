//! The message-acceptance policy hook (issue #383, step 4).
//!
//! Three things need holding down, and only the first is about
//! behaviour a caller asked for:
//!
//! 1. **The default is bit-identical.** A request that never names a
//!    policy carries [`DefaultPolicy`], which is zero-sized and inlines
//!    to the codec's own verdict — exactly what
//!    `process_one_candidate_inner` did unconditionally before the hook
//!    existed. Asserted by decoding the reference recording both ways
//!    and comparing the full result rows, not just the count.
//! 2. **`.also_accept()` only widens.** It cannot lose a decode the
//!    default would have made, whatever the caller's predicate says —
//!    including a predicate that rejects everything.
//! 3. **`.message_filter()` replaces.** A predicate that rejects
//!    everything yields nothing; one that accepts everything yields at
//!    least what the default did, since it removes a filter rather than
//!    adding one.
//!
//! The strategy tag is exercised too: `.sic_early()` before and after
//! `.also_accept()` has to reach the same strategy, which is what
//! `SupportsMessageFilter::__strategy_for` exists to guarantee.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8 \
//!     --test ft8_message_policy
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "ft8"))]

use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

fn audio() -> Vec<i16> {
    load_wav_i16(std::path::Path::new(QSO3_PATH))
}

fn req(audio: &[i16]) -> DecodeRequest<'_, Ft8> {
    DecodeRequest::<Ft8>::new(audio, 200.0, 3000.0, 1.5, 200)
}

/// Every field a caller can see, so "unchanged" means unchanged rather
/// than "same number of rows".
fn rows(out: &mfsk_core::msg::decode_request::DecodeOutcome<Ft8>) -> Vec<String> {
    let mut v: Vec<String> = out
        .results
        .iter()
        .map(|r| {
            format!(
                "{}|{:.3}|{:.3}|{}|{:.2}",
                unpack77(r.message77()).unwrap_or_default(),
                r.freq_hz,
                r.dt_sec,
                r.pass,
                r.snr_db
            )
        })
        .collect();
    v.sort();
    v
}

/// FT8 applies its codec verdict by default, so naming it explicitly
/// and widening it with a predicate that never fires must both be
/// no-ops.
#[test]
fn the_default_policy_is_the_codec_verdict() {
    let a = audio();
    let base = rows(&req(&a).decode());
    assert!(!base.is_empty(), "the reference recording must decode");

    let named = rows(&req(&a).codec_filter().decode());
    assert_eq!(base, named, "codec_filter() must reproduce the default");

    let widened = rows(&req(&a).also_accept(|_| false).decode());
    assert_eq!(base, widened, "also_accept(|_| false) must be a no-op");
}

#[test]
fn also_accept_can_only_widen() {
    let a = audio();
    let base = rows(&req(&a).decode());
    let wide = rows(&req(&a).also_accept(|_| true).decode());
    for r in &base {
        assert!(
            wide.contains(r),
            "also_accept dropped a default decode: {r}"
        );
    }
    assert!(wide.len() >= base.len());
}

#[test]
fn message_filter_replaces_the_verdict() {
    let a = audio();
    let base = rows(&req(&a).decode());

    let none = req(&a).message_filter(|_| false).decode();
    assert!(
        none.results.is_empty(),
        "a filter that accepts nothing must yield nothing, got {}",
        none.results.len()
    );

    // Opting out entirely is how a caller gets WSJT-X's own acceptance
    // rule (`nbadcrc` + `nharderrors <= 36`) with nothing on top.
    let all = rows(&req(&a).message_filter(|_| true).decode());
    for r in &base {
        assert!(all.contains(r), "message_filter(|_| true) lost {r}");
    }
    assert!(all.len() >= base.len());
}

/// `.also_accept()` rebuilds the strategy function pointer for the new
/// policy type. If it rebuilt the *wrong* one, a `.sic_early()` request
/// would quietly fall back to single-pass — so the two orderings have
/// to agree, and both have to differ from single-pass.
#[test]
fn the_strategy_survives_a_policy_change_in_either_order() {
    let a = audio();
    let single = rows(&req(&a).single_pass().also_accept(|_| false).decode());
    let before = rows(&req(&a).also_accept(|_| false).sic_early().decode());
    let after = rows(&req(&a).sic_early().also_accept(|_| false).decode());

    assert_eq!(before, after, "policy order must not change the strategy");
    assert_ne!(
        single, before,
        "sic_early must still be reaching its own engine, not single-pass"
    );
}
