//! Regression check for `decode_frame_subtract_staged` (issue #180):
//! WSJT-X-faithful checkpoint SIC (`jt9.f90` `nearly=41/47/50`) vs the
//! existing flat 3-pass `decode_frame_subtract`.
//!
//! # Background
//!
//! Issue #180's ground-truth trace of WSJT-X's *actual* disk-decode path
//! (instrumented `jt9.f90` / `ft8_decode.f90`, not just its exported
//! `.wav` → message behaviour) found that `jt9` never runs a single pass
//! over the whole 15 s slot. It decodes progressively larger buffer
//! *prefixes* at three checkpoints (`nearly=41` → 141_696 samples,
//! `nearly=47` → 162_432 samples subtract-prep only, `nzhsym=50` →
//! 172_800 samples for the final full pass), carrying decoded signals
//! forward across checkpoints and subtracting them from the residual
//! *before* the harder, marginal candidates at the final checkpoint are
//! attempted. `decode_frame_subtract_staged` (`src/ft8/decode.rs`) ports
//! this structure; see its module doc for the full architecture.
//!
//! `CQ DX DL8YHR JO41` (~-17 dB @ 2606 Hz) only decodes in WSJT-X at
//! that final checkpoint, after 13 stronger neighbours have already been
//! removed — something the pre-existing flat pass structurally cannot
//! reproduce, no matter how its sync threshold is tuned, because nothing
//! is ever subtracted from the buffer *before* the residual DL8YHR is
//! found in gets assembled.
//!
//! # Status
//!
//! Verified (2026-07-25) against this real recording:
//! - The staged path finds the same 18/18 golden messages as the flat
//!   `decode_frame_subtract` baseline — no regression from restructuring
//!   the pipeline into checkpoints.
//! - Checkpoint A found 11 signals early (including `W1FC F5BZB`, only
//!   ~35 Hz from DL8YHR's carrier); checkpoint C found the remaining 7
//!   against that cleaned residual.
//! - `DL8YHR` still does **not** decode. This matches issue #180's own
//!   finding that even subtracting *all 18* golden decodes from the raw
//!   residual (a superset of WSJT-X's 13) only reached `sync_quality=9`,
//!   short of `jt9`'s `nsync=15` — i.e. the architecture fix alone does
//!   not close the separately-documented ~1-2 dB sync/downsample
//!   sensitivity gap (issue #180's "Bug 1" off-by-one dt convention /
//!   numeric fidelity discussion). That remains open, tracked in #180.
//!
//! This test therefore asserts the *no-regression* floor, not a DL8YHR
//! hit — flip `assert!(has_dl8yhr, ...)` on once the sensitivity gap is
//! independently closed.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_staged_sic_check -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::decode::{DecodeDepth, DecodeStrictness, decode_frame_subtract_staged};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// The 18-message set `decode_frame_subtract` (flat 3-pass SIC) already
/// finds on `qso3_busy.wav` — see `ft8_qso3_subtract_fix_check.rs`. The
/// staged path must not lose any of these.
const FLAT_PASS_GOLDEN: &[&str] = &[
    "A92EE F5PSR -14",
    "CQ EA2BFM IN83",
    "CQ F5RXL IN94",
    "K1BZM EA3CJ JN01",
    "K1BZM EA3GP -9",
    "K1JT EA3AGB -15",
    "K1JT HA0DU KN07",
    "K1JT HA5WA 73",
    "KD2UGC F6GCP R-23",
    "N1API F2VX 73",
    "N1API HA6FQ -23",
    "N1JFU EA6EE R-7",
    "N1PJT HB9CQK -10",
    "W0RSJ EA3BMU RR73",
    "W1DIG SV9CVY -14",
    "W1FC F5BZB -8",
    "WM3PEN EA6VQ -9",
    "XE2X HA2NP RR73",
];

use common::load_wav_i16;

#[test]
fn staged_sic_matches_flat_pass_golden_floor() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let results = decode_frame_subtract_staged(
        &audio,
        100.0,
        3000.0,
        0.8,
        None,
        DecodeDepth::BpAllOsd,
        200,
        DecodeStrictness::Normal,
    );
    let msgs: BTreeSet<String> = results
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect();

    println!(
        "decode_frame_subtract_staged(qso3_busy.wav): {} decodes",
        msgs.len()
    );
    let mut golden_hits = 0usize;
    for g in FLAT_PASS_GOLDEN {
        let hit = msgs.contains(*g);
        println!("  {} {}", if hit { "✓" } else { "✗" }, g);
        if hit {
            golden_hits += 1;
        }
    }
    let has_dl8yhr = msgs.iter().any(|m| m.contains("DL8YHR"));
    println!("has_dl8yhr={has_dl8yhr} (see module doc — known open gap, issue #180)");

    assert_eq!(
        golden_hits,
        FLAT_PASS_GOLDEN.len(),
        "staged SIC regressed vs the flat-pass golden floor: {}/{}",
        golden_hits,
        FLAT_PASS_GOLDEN.len(),
    );
}
