//! Durable exercise of [`WsjtxDepth`]'s `D1`/`D2`/`D3` tiers against
//! `qso3_busy.wav`, replacing the throwaway `adhoc_*_probe.rs` files
//! used to derive them.
//!
//! Real `jt9` isn't available in CI, so this can't diff against it
//! directly — the table below records a real local `jt9 -8 -d1/-d2/-d3`
//! build's output on this file (this session, this host) as a
//! human-comparable reference, and each tier asserts only a soft
//! recall floor (not exact-match) against it.
//!
//! Reference (`jt9 -8 -dN qso3_busy.wav`):
//!
//! | | jt9 | mfsk-core (this test) |
//! |---|---|---|
//! | D1 | 14 decodes / 370ms | 14 decodes / 237ms |
//! | D2 | 19 decodes / 1040ms | 22 decodes / 1078ms |
//! | D3 | 22 decodes / 2110ms | 22 decodes / 2991ms |
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_wsjtx_depth_ladder -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{ApHint, WsjtxDepth};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Operator context for `D3`'s AP hint — same convention as
/// `ft8_qso3_apon_recall.rs` (K1JT is the "called side" of the AP-off
/// golden entry `K1JT HA0DU KN07`).
const MYCALL: &str = "K1JT";
const HISCALL: &str = "HA0DU";

/// Soft recall floors — well below the reference table above, just
/// enough to catch a catastrophic regression (e.g. a future change
/// accidentally severing `.sic_rounds()`/`.sic_early()`/`.ap_hint()` from
/// `wsjtx_depth`). Not a golden-exact-match test.
const MIN_DECODES: [usize; 3] = [10, 15, 15];

fn run_tier(slot: &[i16], tier: WsjtxDepth, ap: Option<&ApHint>) -> (usize, f64, Vec<String>) {
    let t0 = Instant::now();
    let decoded = DecodeRequest::<Ft8>::wsjtx_depth(slot, 100.0, 3000.0, 1.3, 50, tier, ap)
        .decode()
        .results;
    let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let mut msgs: Vec<String> = decoded
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    msgs.sort();
    (decoded.len(), elapsed_ms, msgs)
}

#[test]
fn wsjtx_depth_ladder_on_qso3_busy() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let ap = ApHint::new().with_call1(MYCALL).with_call2(HISCALL);

    for (label, tier, floor) in [
        ("D1", WsjtxDepth::D1, MIN_DECODES[0]),
        ("D2", WsjtxDepth::D2, MIN_DECODES[1]),
        ("D3", WsjtxDepth::D3, MIN_DECODES[2]),
    ] {
        let (n, ms, msgs) = run_tier(&slot, tier, Some(&ap));
        println!("\n{label}: {n} decodes, {ms:.1} ms");
        for m in &msgs {
            println!("    {m}");
        }
        assert!(
            n >= floor,
            "{label} recall regressed: {n} decodes, floor is {floor}"
        );
    }
}
