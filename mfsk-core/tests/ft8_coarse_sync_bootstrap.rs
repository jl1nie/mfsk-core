//! Bootstrap-from-coarse-sync DT median feasibility + correctness gate.
//!
//! Issue context: embedded `decode_pipeline.rs` currently feeds
//! confirmed-decode DT median into `set_bootstrap_slot_shift_12k`,
//! so cold-start with 0 confirmed decodes leaves the slot mis-aligned
//! ("BtnA required"). WebFT8 wants to do better — bootstrap slot
//! alignment from raw `coarse_sync` candidate DTs while
//! N_confirmed = 0.
//!
//! `engine::sync::bootstrap_dt_median(&cands, K=5)` is the published
//! helper both consumers use. This test gates K=5 |Δ| < 100 ms vs
//! the confirmed-decode DT median on three reference recordings —
//! one busy band (15 decodes) and two sparse JTDX captures (4-5
//! decodes). The K=5 cutoff is empirical: K=10+ washes out under
//! false-candidate noise (see informational K=10/20 print-outs).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_coarse_sync_bootstrap -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::engine::sync::{SyncCandidate, bootstrap_dt_median};
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{coarse_sync, compute_spectrogram, decode_block};

#[allow(dead_code)]
mod common;

use common::load_wav_i16;

fn median(mut xs: Vec<f32>) -> Option<f32> {
    if xs.is_empty() {
        return None;
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = xs.len();
    Some(if n % 2 == 1 {
        xs[n / 2]
    } else {
        0.5 * (xs[n / 2 - 1] + xs[n / 2])
    })
}

/// K=5 |Δ| budget vs confirmed-decode median. Empirical worst case on
/// the three reference WAVs is 90 ms (host f32); 100 ms gives a small
/// safety margin without admitting K=10's ~225-275 ms misses.
#[cfg(not(feature = "fixed-point"))]
const BOOTSTRAP_MAX_DELTA_SEC: f32 = 0.100;

/// `fixed-point`'s `coarse_sync` quantises candidate magnitudes
/// through `SpecCell = u16` / `CoarseAcc = i32` (see
/// `decode_block/spectrogram.rs` and `decode_block/coarse_sync.rs`)
/// rather than host f32's direct sum — this measurably reorders which
/// candidates land in the top-5 by score, without indicating any
/// decode-correctness problem (`ft8_qso3_apoff_recall`'s
/// fixed-point run gets byte-identical golden hits + phantom set to
/// f32 on the same fixture). Empirical worst case here is 116 ms
/// (`qso3_busy.wav`); 130 ms keeps a similar small margin to the f32
/// budget above.
#[cfg(feature = "fixed-point")]
const BOOTSTRAP_MAX_DELTA_SEC: f32 = 0.130;

fn check_one(label: &str, wav_path: &str) {
    let audio = load_wav_i16(Path::new(wav_path));
    let spec = compute_spectrogram(&audio, 3_000.0);

    let cands: Vec<SyncCandidate> = coarse_sync(&spec, 100.0, 3_000.0, 1.0, 200);
    let decodes = decode_block(&audio, 100.0, 3_000.0, 1.0, DecodeDepth::FULL, 30);

    let dt_conf = median(decodes.iter().map(|r| r.dt_sec).collect())
        .expect("reference WAVs all have ≥1 confirmed decode");
    let dt_boot_k5 = bootstrap_dt_median(&cands, 5).expect("coarse_sync returned no candidates");
    let dt_boot_k10 = bootstrap_dt_median(&cands, 10);
    let dt_boot_k20 = bootstrap_dt_median(&cands, 20);

    let diff = (dt_conf - dt_boot_k5).abs();
    println!(
        "{label:<22} N_conf={:>2}  conf_dt={:+.3}s  K5={:+.3}s  |Δ|={:.3}s  \
         (K10={:+.3?} K20={:+.3?})",
        decodes.len(),
        dt_conf,
        dt_boot_k5,
        diff,
        dt_boot_k10,
        dt_boot_k20,
    );

    assert!(
        diff < BOOTSTRAP_MAX_DELTA_SEC,
        "{label}: K=5 bootstrap DT off by {:.3}s vs confirmed (budget {:.3}s) — \
         coarse_sync top-5 no longer correlates with truth on this fixture",
        diff,
        BOOTSTRAP_MAX_DELTA_SEC
    );
}

#[test]
fn bootstrap_dt_median_top5_matches_confirmed() {
    check_one("qso3_busy.wav", asset_path!("qso3_busy.wav"));
    check_one("191111_110130.wav", asset_path!("191111_110130.wav"));
    check_one("191111_110200.wav", asset_path!("191111_110200.wav"));
}

#[test]
fn bootstrap_dt_median_handles_empty_and_zero_k() {
    assert!(bootstrap_dt_median(&[], 5).is_none());
    let dummy = vec![SyncCandidate {
        freq_hz: 1000.0,
        dt_sec: 0.3,
        score: 2.0,
    }];
    assert!(bootstrap_dt_median(&dummy, 0).is_none());
    assert_eq!(bootstrap_dt_median(&dummy, 5).unwrap(), 0.3);
}
