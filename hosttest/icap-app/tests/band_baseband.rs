// SPDX-License-Identifier: GPL-3.0-only
//! Method C of `docs/notes/MIDRANGE_EMBEDDED_EVAL.md`: the whole band as one complex baseband (`band_bb`), built
//! in **one** forward read of the audio, from which the coarse search, every candidate's 200 Hz baseband and the
//! subtraction between passes all work. Floors on ft8_lib's 22 recordings (against `jt9`'s 402) and on
//! `qso3_busy.wav` (WSJT-X's and JTDX's 20), at the radio's 8 kHz input.
//!
//! A floor is the measured figure: a change that lowers it fails, one that raises it asks for the constant to
//! be raised with it. The decodes `jt9` lacks must be attested (`profile::ATTESTED_EXTRAS`) or counted in
//! `MAX_UNEXPLAINED`, which is documented debt: two decodes no other decoder lists (below).
//!
//! Run by package, f32 and `--features fixed-point`: `cargo test --release -p icap-app-hosttest --test
//! band_baseband`.

use std::collections::BTreeMap;

use icap_app_hosttest::{fft_shim, profile, resample::resample, wav};
use mfsk_core::ft8::decode_block::band_bb::SubMode;

#[path = "../../../mfsk-core/tests/common/ft8_qso3.rs"]
#[allow(dead_code)]
mod ft8_qso3;
#[path = "../../../mfsk-core/tests/common/golden.rs"]
#[allow(dead_code)]
mod golden;

use ft8_qso3::QSO3_KNOWN_REAL_SIGNALS;

/// One pass (the coarse search and the candidates on the unsubtracted baseband), 30 candidates. Measured
/// 2026-11-25: f32 272, fixed-point 279 of 402; the decode of the audio that re-reads it per group
/// (`decode_block_low_mem_src`) has 274 and 279.
#[cfg(not(feature = "fixed-point"))]
const ONE_PASS_30: usize = 272;
#[cfg(feature = "fixed-point")]
const ONE_PASS_30: usize = 279;

/// Three passes with the subtraction on the baseband between them, 15 candidates (the ship setting) and 30.
/// Measured 2026-11-25: f32 334 / 353, fixed-point 345 / 350 of 402; ICAP's `ft8dec` (three passes, exact
/// subtraction in its own way) has 333.
#[cfg(not(feature = "fixed-point"))]
const THREE_PASS_15: usize = 334;
#[cfg(feature = "fixed-point")]
const THREE_PASS_15: usize = 345;
#[cfg(not(feature = "fixed-point"))]
const THREE_PASS_30: usize = 353;
#[cfg(feature = "fixed-point")]
const THREE_PASS_30: usize = 350;

/// Decodes `jt9` lacks, no other decoder lists, and nothing attests: `EA8PP JH0INP PM96` (websdr_test7, f32)
/// and `EA8ESB JM5WSV OP12` (websdr_test5, fixed-point), each at about 940 Hz, found in the subtraction passes.
/// Possible phantoms (a CRC-14 match is 1 in 16 384 per attempt); not investigated further. One per build.
const MAX_UNEXPLAINED: usize = 1;

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

fn assert_embedded_path() {
    assert!(
        fft_shim::factory_calls() > 0,
        "mfsk-core never asked for the extern FFT planner: it was built with fft-rustfft (run `-p icap-app-hosttest` alone)"
    );
}

/// One pass, the coarse search from the baseband: the audio is read **once**, forward, whole (one sweep, no
/// sample skipped, no restart), and recall is the audio-reading path's.
#[test]
fn one_read_of_the_audio_meets_the_floor() {
    let mut runs = BTreeMap::new();
    for (stem, slot) in slots() {
        let r = profile::run_slot_band(&slot, 30, 4, 4, true);
        assert_eq!(r.cost.restarts, 0, "{stem}: the audio was read again");
        assert_eq!(r.cost.samples_skipped, 0, "{stem}: samples thrown away");
        assert_eq!(
            r.cost.samples_read as usize,
            slot.len(),
            "{stem}: not exactly one sweep ({:?})",
            r.cost
        );
        runs.insert(stem, r.run.decodes);
    }
    assert_embedded_path();
    let s = profile::score(&runs);
    println!(
        "one pass, 30 candidates: recall {}/{} extras {} (unexplained {})",
        s.hits, s.reference, s.extras, s.unexplained
    );
    assert!(s.hits >= ONE_PASS_30, "recall {} < {ONE_PASS_30}", s.hits);
    assert!(
        s.unexplained <= MAX_UNEXPLAINED,
        "{} unexplained",
        s.unexplained
    );
}

/// Three passes with the subtraction on the baseband: more of `jt9`'s decodes than ICAP's own decoder finds
/// (333), from one read of the audio.
#[test]
fn subtraction_on_the_baseband_meets_the_floor() {
    for (max_cand, floor) in [(15usize, THREE_PASS_15), (30, THREE_PASS_30)] {
        let mut runs = BTreeMap::new();
        for (stem, slot) in slots() {
            let r = profile::run_slot_band_sic(&slot, max_cand, 4, 4, 3);
            assert_eq!(r.cost.restarts, 0, "{stem}: the audio was read again");
            runs.insert(stem, r.run.decodes);
        }
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "three passes, {max_cand} candidates: recall {}/{} extras {} (unexplained {})",
            s.hits, s.reference, s.extras, s.unexplained
        );
        assert!(
            s.hits >= floor,
            "max_cand {max_cand}: recall {} < {floor}",
            s.hits
        );
        assert!(
            s.unexplained <= MAX_UNEXPLAINED,
            "max_cand {max_cand}: {} unexplained",
            s.unexplained
        );
    }
}

/// The busy slot: 19 of WSJT-X's and JTDX's 20, none outside them, at 15 and at 30 candidates. The one missed,
/// `WA2FZW DL5AXX RR73`, is the entry the golden marks as possibly a JTDX false positive; the exact audio-path
/// subtraction (`run_slot_sic`) misses the same one. ICAP's `ft8dec` decodes 16.
#[test]
fn the_busy_slot_gives_19_of_20() {
    let slot12 = wav::load_i16(profile::asset_path("qso3_busy"));
    let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
    for max_cand in [15usize, 30] {
        let r = profile::run_slot_band_sic(&slot, max_cand, 4, 4, 3);
        let msgs: Vec<String> = r.run.decodes.iter().map(|d| d.0.clone()).collect();
        let known: Vec<String> = QSO3_KNOWN_REAL_SIGNALS
            .iter()
            .map(|g| profile::norm(g.msg))
            .collect();
        let hits = known.iter().filter(|k| msgs.contains(k)).count();
        let extra: Vec<&String> = msgs.iter().filter(|m| !known.contains(m)).collect();
        let missing: Vec<&String> = known.iter().filter(|k| !msgs.contains(k)).collect();
        println!("max_cand {max_cand}: {hits}/20, missing {missing:?}, extra {extra:?}");
        assert!(
            hits >= 19,
            "max_cand {max_cand}: {hits}/20, missing {missing:?}"
        );
        assert!(
            extra.is_empty(),
            "max_cand {max_cand}: not in the golden: {extra:?}"
        );
    }
}

/// The subtraction with no `f64` (`SubMode::F32`, for a target whose `f64` is software) gives the `f64` one's
/// recall: at the ship setting the same floor holds on the 22 recordings (measured 2026-11-25: f32 335, fixed-point
/// 345, against 334 and 345 with `f64`), and the busy slot is still 19 of 20.
#[test]
fn subtraction_without_f64_keeps_the_recall() {
    let mut runs = BTreeMap::new();
    for (stem, slot) in slots() {
        let r = profile::run_slot_band_sic_with(&slot, 15, 4, 4, 3, SubMode::F32);
        runs.insert(stem, r.run.decodes);
    }
    assert_embedded_path();
    let s = profile::score(&runs);
    println!(
        "f32 subtraction, 15 candidates: recall {}/{} unexplained {}",
        s.hits, s.reference, s.unexplained
    );
    assert!(
        s.hits >= THREE_PASS_15,
        "recall {} < {THREE_PASS_15}",
        s.hits
    );
    assert!(
        s.unexplained <= MAX_UNEXPLAINED,
        "{} unexplained",
        s.unexplained
    );

    let slot12 = wav::load_i16(profile::asset_path("qso3_busy"));
    let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
    let r = profile::run_slot_band_sic_with(&slot, 15, 4, 4, 3, SubMode::F32);
    let msgs: Vec<String> = r.run.decodes.iter().map(|d| d.0.clone()).collect();
    let hits = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .filter(|g| msgs.contains(&profile::norm(g.msg)))
        .count();
    assert!(hits >= 19, "qso3_busy: {hits}/20");
}
