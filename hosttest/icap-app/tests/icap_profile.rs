// SPDX-License-Identifier: GPL-3.0-only
//! The ICAP profile's floors: mfsk-core's embedded FT8 path (no `fft-rustfft`) on ft8_lib's 22 real
//! recordings at the radio's 8 kHz input rate, against `jt9`'s 402 decodes (the tier-B reference).
//!
//! Two paths, two floors, both regression floors rather than targets:
//! - `decode_block`, the ESP32 driver: single pass, no subtraction, no OSD (both are `fft-rustfft` code),
//!   candidates by coarse score cut to 30 then 15 by block-0 sync. About a third of `jt9`'s decodes, no extras.
//! - `decode_block_low_mem`, the ICAP profile: candidates by hard sync, refined, a band of the spectrogram at
//!   a time. About half, and 60 % at 30 candidates; its extras must each be an attested real signal.
//!
//! A floor is the measured figure: a change that lowers it fails, one that raises it asks for the constant
//! to be raised with it. `banding_the_low_memory_profile_changes_no_decode` pins that the bands change memory
//! and time only.
//!
//! Run by package: `cargo test --release -p icap-app-hosttest` (and again with `--features fixed-point`). A
//! workspace-wide run unifies `fft-rustfft` into mfsk-core and the tests say so instead of measuring the
//! wrong driver.

use std::collections::BTreeMap;

use icap_app_hosttest::{fft_shim, profile, resample::resample, wav};

/// Measured 2026-11-24 (`icap-profile --rate 8000 --max-cand 15`): f32 143 of 402, fixed-point 131 of 402
/// (what the ESP32 boards ship), no extras either way; f32 gives the same figure at 12 kHz input, so the
/// 8 kHz path costs nothing here.
#[cfg(not(feature = "fixed-point"))]
const MIN_HITS: usize = 143;
#[cfg(feature = "fixed-point")]
const MIN_HITS: usize = 131;
const MAX_EXTRAS: usize = 0;

#[cfg(not(feature = "fixed-point"))]
const LOW_MEM_FLOOR_15: usize = 225;
#[cfg(feature = "fixed-point")]
const LOW_MEM_FLOOR_15: usize = 227;
#[cfg(not(feature = "fixed-point"))]
const LOW_MEM_FLOOR_30: usize = 274;
#[cfg(feature = "fixed-point")]
const LOW_MEM_FLOOR_30: usize = 279;

/// Every recording through the embedded path at 8 kHz (`low_mem`: `decode_block_low_mem` with that many
/// coarse bands, or `decode_block`), `max_cand` candidates: what each file decoded, in the order found.
fn decode_all(low_mem: Option<usize>, max_cand: usize) -> BTreeMap<String, Vec<(String, f32)>> {
    let mut runs = BTreeMap::new();
    for stem in profile::stems() {
        let slot12 = wav::load_i16(profile::asset_path(&stem));
        let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
        runs.insert(stem, profile::run_slot(&slot, max_cand, low_mem).decodes);
    }
    runs
}

fn assert_embedded_path() {
    assert!(
        fft_shim::factory_calls() > 0,
        "mfsk-core never asked for the extern FFT planner: it was built with fft-rustfft (run `-p icap-app-hosttest` alone), so this ran the host driver"
    );
}

#[test]
fn embedded_path_at_8khz_meets_its_floor() {
    let runs = decode_all(None, 15);
    assert_embedded_path();
    let s = profile::score(&runs);
    println!(
        "decode_block: recall {}/{} extras {}",
        s.hits, s.reference, s.extras
    );
    assert!(s.hits >= MIN_HITS, "recall {} < floor {MIN_HITS}", s.hits);
    assert_eq!(
        s.extras, MAX_EXTRAS,
        "outputs jt9 lacks: the ceiling is {MAX_EXTRAS}"
    );
}

/// The low-memory profile (`decode_block_low_mem`: candidates by hard sync, refined, a band of the
/// spectrogram at a time, tone spectra from a 200 Hz baseband, OSD) on the same 22 recordings, at the ship
/// setting (15 candidates) and at 30. Measured 2026-11-25 on the streaming pipeline: f32 225 and 274 of 402,
/// fixed-point 227 and 279,
/// against `decode_block`'s 143 and 131. Its extras must each be a signal `jt9` lacks that
/// `profile::ATTESTED_EXTRAS` lists with its evidence; anything else is a phantom.
#[test]
fn the_low_memory_profile_meets_its_floor() {
    for (max_cand, floor) in [(15usize, LOW_MEM_FLOOR_15), (30, LOW_MEM_FLOOR_30)] {
        let runs = decode_all(Some(4), max_cand);
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "low_mem max_cand {max_cand}: recall {}/{} extras {} (unexplained {})",
            s.hits, s.reference, s.extras, s.unexplained
        );
        assert!(
            s.hits >= floor,
            "max_cand {max_cand}: recall {} < floor {floor}",
            s.hits
        );
        assert_eq!(
            s.unexplained, 0,
            "max_cand {max_cand}: a decode jt9 lacks that is not attested"
        );
    }
}

/// The number of bands changes memory and time, never the decodes: the 22 recordings decode to the same
/// messages, frequencies and order with the spectrogram in 1 band as in 4.
#[test]
fn banding_the_low_memory_profile_changes_no_decode() {
    assert_eq!(decode_all(Some(1), 15), decode_all(Some(4), 15));
}
