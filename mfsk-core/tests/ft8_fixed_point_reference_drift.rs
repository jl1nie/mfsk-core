//! Where the `fixed-point` matrix loses messages that f32 keeps, on
//! the three real-QSO recordings.
//!
//! `ft8_decode_block_real_qso.rs` computes `truth` with `DecodeRequest`
//! in the same build as the ship config it scores, so the obvious
//! suspicion for its `fixed-point` failure is that the reference moved
//! too. **It does not**: the `truth` sets are identical by name in both
//! matrices, 6 / 5 / 15, so the whole deficit is in `decode_block`.
//!
//! What it is, measured with the two knobs below:
//!
//! | matrix | lag | `max_cand` | qso1 | qso2 | qso3_busy |
//! |---|---|---|---|---|---|
//! | f32 | 1.0 | 15 | 4 | 5 | 12 |
//! | f32 | 2.5 | 15 | 4 | 5 | 12 |
//! | fixed-point | 1.0 | 15 | 4 | 5 | 12 |
//! | fixed-point | 2.0 | 15 | 4 | 5 | 11 |
//! | fixed-point | 2.5 | 15 | **3** | 5 | **10** |
//! | fixed-point | 2.5 | 25 | 3 | 5 | 12 |
//! | fixed-point | 2.5 | 40 | 4 | 5 | 13 |
//!
//! The wider window issue #280 restored costs the quantised path three
//! messages and the f32 path none, and raising `max_cand` buys them
//! back — so it is candidate-budget displacement by far-lag ghosts,
//! which u16 quantisation ranks differently, not a decode failure. The
//! board itself is unaffected: `dual_core` calls `coarse_sync_with_lag`
//! with `EMBEDDED_SYNC_LAG_S = 1.0` and never sees `SYNC_LAG_S_DEFAULT`.
//!
//! This test does not assert — it prints the sets by name. `MFSK_SYNC_LAG_S`
//! and `MFSK_SHIP_MAX_CAND` are the sweep knobs:
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test ft8_fixed_point_reference_drift -- --ignored --nocapture
//! cargo test -p mfsk-core --features full,fixed-point,internal-testing --release \
//!     --test ft8_fixed_point_reference_drift -- --ignored --nocapture
//! ```
//!
//! Refs #359, #280.

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

use common::load_wav_i16;

const WAVS: &[(&str, &str)] = &[
    ("qso1", asset_path!("qso1.wav")),
    ("qso2", asset_path!("qso2.wav")),
    ("qso3_busy", asset_path!("qso3_busy.wav")),
];

/// The build this is running in, for the reader of the two outputs.
fn matrix() -> &'static str {
    if cfg!(feature = "fixed-point") {
        "fixed-point (implies nstep-half: N_TIME 184, NSSY 2)"
    } else {
        "f32 (quarter-step grid: N_TIME 372, NSSY 4)"
    }
}

#[test]
#[ignore = "diagnostic: prints message sets for cross-matrix diffing (#359)"]
fn reference_and_ship_sets_by_name() {
    println!("\nmatrix: {}\n", matrix());

    for (label, path) in WAVS {
        let slot = load_wav_i16(Path::new(path));

        // Same two configurations `ft8_decode_block_real_qso.rs` uses.
        let truth: BTreeSet<String> = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.0, 200)
            .decode()
            .results
            .iter()
            .filter_map(|x| unpack77(x.message77()))
            .collect();
        let max_cand: usize = std::env::var("MFSK_SHIP_MAX_CAND")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(15);
        let ship: BTreeSet<String> =
            decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::BP_ONLY, max_cand)
                .iter()
                .filter_map(|x| unpack77(x.message77()))
                .collect();

        println!(
            "== {label}: truth={} ship={} hit={} extra={}",
            truth.len(),
            ship.len(),
            ship.intersection(&truth).count(),
            ship.difference(&truth).count(),
        );
        for m in &truth {
            println!("   truth  {m}");
        }
        for m in &ship {
            println!("   ship   {m}");
        }
        println!();
    }
}
