// SPDX-License-Identifier: GPL-3.0-only
//! Stage 0 of the S3 plan: what the ESP32 boards' own pieces reach with the hard-sync gate and OSD added, and
//! no FIR baseband. The boards run Goertzel for the candidates' symbol spectra; the FIR cascade (`cd0_fir`)
//! was tried on the S3 in 0.6.2 and given up (`ROADMAP.md`, "Embedded fine_refine attempt postmortem": PSRAM
//! bandwidth), and costs about twice the instructions of the Goertzel path on the host (596-614 M against
//! 283 M for one slot of `qso3_busy`). So the question is how much of the FIR path's recall the cheaper pieces
//! keep. `V9` is: the first `max_cand` candidates by hard sync (`coarse_sync_gated`), the boards'
//! `fine_sync_12k`, the pass-2 symbol spectra (`refine_candidates_into`), BP and OSD
//! (`process_candidates_into`).
//!
//! Fixed-point only (the per-candidate entry points are `fixed-point` code, the numeric path the boards ship).
//! Run: `cargo test --release -p icap-app-hosttest --features fixed-point --test s3_gate_osd`.
#![cfg(feature = "fixed-point")]

use std::collections::BTreeMap;

use icap_app_hosttest::{fft_shim, profile, profile::Variant, resample::resample, wav};

#[path = "../../../mfsk-core/tests/common/ft8_qso3.rs"]
#[allow(dead_code)]
mod ft8_qso3;
#[path = "../../../mfsk-core/tests/common/golden.rs"]
#[allow(dead_code)]
mod golden;

use ft8_qso3::QSO3_KNOWN_REAL_SIGNALS;

fn slots() -> Vec<(String, Vec<i16>)> {
    profile::stems()
        .into_iter()
        .map(|stem| {
            let slot12 = wav::load_i16(profile::asset_path(&stem));
            let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
            (stem, slot)
        })
        .collect()
}

fn hits(v: Variant, max_cand: usize) -> (profile::Score, usize) {
    let mut runs = BTreeMap::new();
    for (stem, slot) in slots() {
        runs.insert(stem, profile::run_slot_variant(&slot, v, max_cand).decodes);
    }
    assert!(
        fft_shim::factory_calls() > 0,
        "built with fft-rustfft: this ran the host driver (run `-p icap-app-hosttest` alone)"
    );
    let s = profile::score(&runs);
    let h = s.hits;
    (s, h)
}

fn qso3(v: Variant, max_cand: usize) -> usize {
    let slot12 = wav::load_i16(profile::asset_path("qso3_busy"));
    let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
    let msgs: Vec<String> = profile::run_slot_variant(&slot, v, max_cand)
        .decodes
        .into_iter()
        .map(|d| d.0)
        .collect();
    QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .filter(|g| msgs.contains(&profile::norm(g.msg)))
        .count()
}

/// Measured 2026-11-25 (8 kHz input, 22 recordings of `jt9`'s 402; `qso3_busy` of 20), fixed-point:
///
/// | | 15 cand. | 30 cand. | `qso3_busy` 15 / 30 |
/// |---|---:|---:|---:|
/// | `V0` ESP32 driver (score order, no fine, no OSD) | 131 | 144 | 7 / 7 |
/// | `V10` gate order + `fine_sync_12k`, no OSD | 211 | 248 | 10 / 10 |
/// | **`V9`** + OSD | **221** | **267** | **13 / 14** |
/// | `V4` the FIR baseband path (the ICAP profile) | 227 | 279 | 13 / 14 |
///
/// So `V9` has 94 % / 91 % of `V4`'s recall gain over `V0` on the recordings and all of it on the busy slot.
#[test]
fn gate_fine_sync_and_osd_reach_the_fir_paths_recall() {
    for (max_cand, floor9, floor10, q9) in
        [(15usize, 221usize, 211usize, 13usize), (30, 267, 248, 14)]
    {
        let (s9, h9) = hits(Variant::V9, max_cand);
        let (_, h10) = hits(Variant::V10, max_cand);
        let (_, h0) = hits(Variant::V0, max_cand);
        let (_, h4) = hits(Variant::V4, max_cand);
        println!(
            "max_cand {max_cand}: V0 {h0}, V10 {h10}, V9 {h9}, V4 {h4}; qso3 V9 {}",
            qso3(Variant::V9, max_cand)
        );
        assert!(h9 >= floor9, "V9 {h9} < {floor9}");
        assert!(h10 >= floor10, "V10 {h10} < {floor10}");
        assert_eq!(
            s9.unexplained, 0,
            "V9: a decode jt9 lacks that is not attested"
        );
        // the finding itself: the cheap pieces keep 9 tenths of the FIR path's gain over the ESP32 driver
        assert!(
            (h9 - h0) * 10 >= (h4 - h0) * 9,
            "V9 gains {} of V4's {}",
            h9 - h0,
            h4 - h0
        );
        assert!(qso3(Variant::V9, max_cand) >= q9, "qso3_busy");
    }
}
