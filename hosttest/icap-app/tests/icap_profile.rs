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
use mfsk_core::ft8::decode_block::Ft8Rate;

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

/// The same, at 8 kHz **read as it is** (`Ft8Rate::Hz8000`): the recordings are resampled to 8 kHz once and nothing
/// converts them back. Measured 2026-11-26. **f32**: 224 and 273 of 402 against the round trip's 225 and 274, no extras; one
/// signal is lost at 30 candidates (`websdr_test6`, `OM7AZA SV8EUB -11`, 348.5 Hz); at 45 and 60 the two recalls are equal
/// (277 and 277, 278 and 278): a candidate on the edge of the list. **Fixed-point** (the coarse spectrogram's frame is a Hann
/// window over two symbols at 8 kHz, `Ft8Rate::coarse_hann_2t`): 221 and 284 of 402 with the window, 221 and 280 without (and 229 and 287 with the `ft8_lib` search of `Ft8Rate::coarse_ft8lib`, which this path's
/// `coarse_sync_gated` takes too); with
/// the window one more decode `jt9` lacks: `websdr_test6`'s `SM2EKA UT7IS -06` at 1615.6 Hz, which `jt9` lists at 1716 Hz; the
/// message's tones read 0 of 58 wrong at both places (the audio holds the signal and a copy of it 100 Hz below), so it is
/// attested (`profile::ATTESTED_EXTRAS`) like the copies of strong signals already there; the S3 flow gives the same message
/// without the window, at 30 candidates.
#[cfg(not(feature = "fixed-point"))]
const NATIVE_8K_FLOORS: [(usize, usize); 2] = [(15, 224), (30, 273)];
#[cfg(feature = "fixed-point")]
const NATIVE_8K_FLOORS: [(usize, usize); 2] = [(15, 229), (30, 287)];

#[test]
fn the_low_memory_profile_at_native_8khz_meets_its_floor() {
    for (max_cand, floor) in NATIVE_8K_FLOORS {
        let mut runs = BTreeMap::new();
        for stem in profile::stems() {
            let slot12 = wav::load_i16(profile::asset_path(&stem));
            let slot8 = resample(&slot12, 12_000, 8_000);
            runs.insert(
                stem,
                profile::run_slot_at(&slot8, Ft8Rate::Hz8000, max_cand, Some(4)).decodes,
            );
        }
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "native 8 kHz max_cand {max_cand}: recall {}/{} extras {} (unexplained {:?})",
            s.hits, s.reference, s.extras, s.unexplained_list
        );
        assert!(
            s.hits >= floor,
            "max_cand {max_cand}: recall {} < floor {floor}",
            s.hits
        );
        #[cfg(not(feature = "fixed-point"))]
        {
            // not meaningfully below what the resample to 12 kHz and back gets
            let round_trip_floor = if max_cand == 15 {
                LOW_MEM_FLOOR_15
            } else {
                LOW_MEM_FLOOR_30
            };
            assert!(
                s.hits + 3 >= round_trip_floor,
                "max_cand {max_cand}: {} against the round trip's floor {round_trip_floor}",
                s.hits
            );
        }
        assert!(
            s.unexplained_list.is_empty(),
            "max_cand {max_cand}: a decode jt9 lacks that is not attested: {:?}",
            s.unexplained_list
        );
    }
}

/// The number of bands changes memory and time, never the decodes: the 22 recordings decode to the same
/// messages, frequencies and order with the spectrogram in 1 band as in 4.
#[test]
fn banding_the_low_memory_profile_changes_no_decode() {
    assert_eq!(decode_all(Some(1), 15), decode_all(Some(4), 15));
}

/// The S3's own flow (`profile::run_slot_s3`: the gate by hard sync, `fine_sync_at`, block-0 refine, stage 3 with OSD, in the
/// fixed-point numeric path the boards run, no FIR baseband) on the same 22 recordings **read at 8 kHz as they are**
/// (`Ft8Rate::Hz8000`), with the windows and the search of `docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md` on: the coarse spectrogram's
/// Hann window over two symbols (`Ft8Rate::coarse_hann_2t`), the symbol DFT's (`Ft8Rate::symbol_hann_2t`) and the candidate search on
/// `ft8_lib`'s statistic (`Ft8Rate::coarse_ft8lib`).
///
/// Measured 2026-11-26 against `jt9`'s 402: **15 candidates 230, 30 candidates 289**; with the two windows and the `sync8` search 224 and
/// 287, with the coarse window alone 216 and 272, with neither 216 and 268. Extras 5 and 7, every one attested
/// (`profile::ATTESTED_EXTRAS`) but `EY8MM YB1BZV OI33`, which is pinned as debt (`S3_DEBT`). On the 60 slots of the design note (the 38 busy ones of `ft8_lib/test/wav/20m_busy` added) the three are
/// +51 hits at 15 candidates and +116 at 30 against neither window. The floors are the measured figures; every extra must be attested
/// or pinned debt, or it is a phantom.
#[cfg(feature = "fixed-point")]
#[test]
fn the_s3_flow_at_native_8khz_meets_its_floor() {
    for (max_cand, floor) in [(15usize, S3_FLOOR_15), (30, S3_FLOOR_30)] {
        let mut runs = BTreeMap::new();
        for stem in profile::stems() {
            let slot12 = wav::load_i16(profile::asset_path(&stem));
            let slot = resample(&slot12, 12_000, 8_000);
            runs.insert(
                stem,
                profile::run_slot_s3(&slot, Ft8Rate::Hz8000, max_cand).decodes,
            );
        }
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "S3 flow, 8 kHz, {max_cand} candidates: recall {}/{} extras {} unexplained {:?}",
            s.hits, s.reference, s.extras, s.unexplained_list
        );
        assert!(
            s.hits >= floor,
            "{max_cand} candidates: recall {} < floor {floor}",
            s.hits
        );
        assert!(
            s.extras <= S3_MAX_EXTRAS,
            "{max_cand} candidates: {} outputs jt9 lacks, ceiling {S3_MAX_EXTRAS}",
            s.extras
        );
        // the decodes `jt9` lacks that nothing attests are the pinned debt by identity, no others (one of them disappearing is fine)
        for u in &s.unexplained_list {
            assert!(
                S3_DEBT.contains(&(u.0.as_str(), u.1.as_str())),
                "{max_cand} candidates: a decode jt9 lacks that is not attested and not pinned debt: {u:?}"
            );
        }
    }
}

/// The cells flow (`profile::run_slot_s3_cells` = `decode_block_cells`: the symbol spectra read from the half-dB cells of the search, no
/// per-candidate DFT, no fine sync, the second try at the place the Costas tones point to) on the same 22 recordings at 8 kHz as they are.
/// Measured 2026-11-26 against `jt9`'s 402: **15 candidates 226, 30 candidates 286** (the S3 flow: 230 and 289), extras 3 and 5 (`Z33Z`, `KC8MUE` attested
/// in `profile::ATTESTED_EXTRAS`, `EY8MM YB1BZV OI33` pinned debt). The floors are the measured figures; every extra must be attested or pinned debt (`S3_DEBT`).
/// The 22 recordings are the weaker yardstick here: on the 60 slots of the design note, against the union of `jt9`'s and `ft8dec`'s lists, the cells flow finds
/// 765 / 1 035 at 15 / 30 candidates against the S3 flow's 769 / 1 036.
#[cfg(feature = "fixed-point")]
#[test]
fn the_cells_flow_at_native_8khz_meets_its_floor() {
    for (max_cand, floor) in [(15usize, CELLS_FLOOR_15), (30, CELLS_FLOOR_30)] {
        let mut runs = BTreeMap::new();
        for stem in profile::stems() {
            let slot12 = wav::load_i16(profile::asset_path(&stem));
            let slot = resample(&slot12, 12_000, 8_000);
            runs.insert(
                stem,
                profile::run_slot_s3_cells(&slot, Ft8Rate::Hz8000, max_cand).decodes,
            );
        }
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "cells flow, 8 kHz, {max_cand} candidates: recall {}/{} extras {} unexplained {:?}",
            s.hits, s.reference, s.extras, s.unexplained_list
        );
        assert!(
            s.hits >= floor,
            "{max_cand} candidates: recall {} < floor {floor}",
            s.hits
        );
        assert!(
            s.extras <= CELLS_MAX_EXTRAS,
            "{max_cand} candidates: {} outputs jt9 lacks, ceiling {CELLS_MAX_EXTRAS}",
            s.extras
        );
        for u in &s.unexplained_list {
            assert!(
                S3_DEBT.contains(&(u.0.as_str(), u.1.as_str())),
                "{max_cand} candidates: a decode jt9 lacks that is not attested and not pinned debt: {u:?}"
            );
        }
    }
}

#[cfg(feature = "fixed-point")]
const CELLS_FLOOR_15: usize = 226;
#[cfg(feature = "fixed-point")]
const CELLS_FLOOR_30: usize = 286;
#[cfg(feature = "fixed-point")]
const CELLS_MAX_EXTRAS: usize = 5;

/// The cells flow on the 22 recordings **through the radio's USB path**: a 12 kHz recording up-converted to 48 kHz (the model of the IC-705's 48 kHz audio) and brought to
/// 8 kHz by the library's new 6:1 decimator (`mfsk_core::engine::dsp::resample_to_8k`), in place of the 12 -> 8 kHz model of `icap-app/src/resample.rs` (the ICAP radio's own 8 kHz).
/// Measured 2026-11-26: **225 / 285 of `jt9`'s 402 at 15 / 30 candidates, extras 2 and 3, none unexplained**, against the ICAP model's 226 / 286 (extras 3 and 5): one hit less at each, the
/// difference of two filters (a Kaiser low-pass to 3 200 / 4 000 Hz against the model's windowed sinc), no worse than the margin of a hit. The floors are the measured figures.
/// **Fails if** the decimator's band, aliasing or level costs recall against the ICAP model.
#[cfg(feature = "fixed-point")]
#[test]
fn the_cells_flow_through_the_48k_to_8k_path_meets_the_floor() {
    for (max_cand, floor) in [(15usize, 225), (30, 285)] {
        let mut runs = BTreeMap::new();
        for stem in profile::stems() {
            let slot12 = wav::load_i16(profile::asset_path(&stem));
            let slot48 = resample(&slot12, 12_000, 48_000);
            let slot = mfsk_core::engine::dsp::resample_to_8k(&slot48, 48_000);
            runs.insert(
                stem,
                profile::run_slot_s3_cells(&slot, Ft8Rate::Hz8000, max_cand).decodes,
            );
        }
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "cells flow through 48 kHz -> 8 kHz, {max_cand} candidates: recall {}/{} extras {} unexplained {:?}",
            s.hits, s.reference, s.extras, s.unexplained_list
        );
        assert!(
            s.hits >= floor,
            "{max_cand} candidates: recall {} < floor {floor}",
            s.hits
        );
    }
}

/// The cells flow with `ft8dec`'s subtraction passes (`profile::run_slot_s3_sic`, 2 passes: `decode_block_cells_sic`) on the same 22 recordings at 8 kHz. The floors
/// are the measured figures (2026-11-26, `jt9`'s 402: **275 at 15 candidates, 332 at 30**, against the cells flow's 226 and 286; extras 4 and 9); every extra must be
/// attested (`profile::ATTESTED_EXTRAS`) or pinned debt (`S3_DEBT`, by identity; `EA8PP JH0INP PM96`, which the passes make, is attested).
#[cfg(feature = "fixed-point")]
#[test]
fn the_sic_flow_at_native_8khz_meets_its_floor() {
    for (max_cand, floor) in [(15usize, SIC_FLOOR_15), (30, SIC_FLOOR_30)] {
        let mut runs = BTreeMap::new();
        for stem in profile::stems() {
            let slot12 = wav::load_i16(profile::asset_path(&stem));
            let slot = resample(&slot12, 12_000, 8_000);
            runs.insert(
                stem,
                profile::run_slot_s3_sic(&slot, Ft8Rate::Hz8000, max_cand, 2).decodes,
            );
        }
        assert_embedded_path();
        let s = profile::score(&runs);
        println!(
            "sic flow, 8 kHz, {max_cand} candidates, 2 passes: recall {}/{} extras {} unexplained {:?}",
            s.hits, s.reference, s.extras, s.unexplained_list
        );
        assert!(
            s.hits >= floor,
            "{max_cand} candidates: recall {} < floor {floor}",
            s.hits
        );
        assert!(
            s.extras <= SIC_MAX_EXTRAS,
            "{max_cand} candidates: {} outputs jt9 lacks, ceiling {SIC_MAX_EXTRAS}",
            s.extras
        );
        for u in &s.unexplained_list {
            assert!(
                S3_DEBT.contains(&(u.0.as_str(), u.1.as_str())),
                "{max_cand} candidates: a decode jt9 lacks that is not attested and not pinned debt: {u:?}"
            );
        }
    }
}

#[cfg(feature = "fixed-point")]
const SIC_FLOOR_15: usize = 275;
#[cfg(feature = "fixed-point")]
const SIC_FLOOR_30: usize = 332;
#[cfg(feature = "fixed-point")]
const SIC_MAX_EXTRAS: usize = 9;

/// Decodes `jt9` lacks and nothing attests, **by identity** (debt, not a count): `websdr_test3`'s `EY8MM YB1BZV OI33`, which
/// `band_baseband.rs` already counts as a chance match until shown otherwise (`jt9` hears the station in `websdr_test1` with another
/// partner, `EY8MM YB1BML 73`; no decoder lists this message in `websdr_test3`). The S3 flow with the `ft8_lib` search decodes it
/// (tones read 23 of 58 wrong at the decoded place, where noise-only reads 40 or more: not shown by any noise-only run).
#[cfg(feature = "fixed-point")]
const S3_DEBT: [(&str, &str); 1] = [("websdr_test3", "EY8MM YB1BZV OI33")];

#[cfg(feature = "fixed-point")]
const S3_FLOOR_15: usize = 230;
#[cfg(feature = "fixed-point")]
const S3_FLOOR_30: usize = 289;
#[cfg(feature = "fixed-point")]
const S3_MAX_EXTRAS: usize = 7;

/// The CoreS3's two P0 configurations (`CellsOptions::p0_window()`, `p0_late()`; `docs/notes/PROFILES.md` section 10) on the 22 recordings at 8 kHz, against `jt9`'s 402. A strategy that
/// reads weak words (the OSD, seeds `a` and `d`, the gate, the subtraction) is where a phantom comes from, so each ships with its **precision guard**: the floors are the measured
/// figures (2026-11-27), the extras are held to the attested or pinned set (`S3_DEBT`, `profile::ATTESTED_EXTRAS`) and to a ceiling, and `the_p0_configurations_decode_nothing_from_noise` holds 200 slots
/// of noise to **zero** decodes.
#[cfg(feature = "fixed-point")]
fn p0_on_the_recordings(
    options: &mfsk_core::ft8::decode_block::CellsOptions,
    what: &str,
    floor: usize,
    max_extras: usize,
) {
    let mut runs = BTreeMap::new();
    for stem in profile::stems() {
        let slot12 = wav::load_i16(profile::asset_path(&stem));
        let slot = resample(&slot12, 12_000, 8_000);
        runs.insert(
            stem,
            profile::run_slot_s3_options(&slot, Ft8Rate::Hz8000, options).decodes,
        );
    }
    assert_embedded_path();
    let s = profile::score(&runs);
    println!(
        "{what}: recall {}/{} extras {} unexplained {:?}",
        s.hits, s.reference, s.extras, s.unexplained_list
    );
    assert!(s.hits >= floor, "{what}: recall {} < floor {floor}", s.hits);
    assert!(
        s.extras <= max_extras,
        "{what}: {} outputs jt9 lacks, ceiling {max_extras}",
        s.extras
    );
    for u in &s.unexplained_list {
        assert!(
            S3_DEBT.contains(&(u.0.as_str(), u.1.as_str())),
            "{what}: a decode jt9 lacks that is not attested and not pinned debt: {u:?}"
        );
    }
}

#[cfg(feature = "fixed-point")]
#[test]
fn the_p0_window_configuration_meets_its_floor() {
    p0_on_the_recordings(
        &mfsk_core::ft8::decode_block::CellsOptions::p0_window(),
        "P0 window",
        P0_WINDOW_FLOOR,
        P0_WINDOW_MAX_EXTRAS,
    );
}

#[cfg(feature = "fixed-point")]
#[test]
fn the_p0_late_configuration_meets_its_floor() {
    p0_on_the_recordings(
        &mfsk_core::ft8::decode_block::CellsOptions::p0_late(),
        "P0 late",
        P0_LATE_FLOOR,
        P0_LATE_MAX_EXTRAS,
    );
}

/// Measured 2026-11-27, `jt9`'s 402: window 280 (extras 2), late 324 (extras 5). The plain cells flow with the faithful inline OSD gets 286 / 332 (the gate, seeds `a` and `d` and the cap of 6 cost 6 and 8
/// on these 22 recordings: 2 %, against 0.6 % on the 4 780 slots of design note section 29).
#[cfg(feature = "fixed-point")]
const P0_WINDOW_FLOOR: usize = 280;
#[cfg(feature = "fixed-point")]
const P0_WINDOW_MAX_EXTRAS: usize = 2;
#[cfg(feature = "fixed-point")]
const P0_LATE_FLOOR: usize = 324;
#[cfg(feature = "fixed-point")]
const P0_LATE_MAX_EXTRAS: usize = 5;

/// 15 s of white noise at 8 kHz, a function of `seed` alone: the sum of four xorshift uniforms (a normal draw to a good approximation) at standard deviation `sigma` counts.
#[cfg(feature = "fixed-point")]
fn noise_slot(seed: u64, sigma: f32) -> Vec<i16> {
    let mut x = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut next = move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x >> 40) as f32 / (1u64 << 24) as f32
    };
    (0..15 * 8000)
        .map(|_| {
            (((next() + next() + next() + next()) - 2.0) * 1.732 * sigma).clamp(-32000.0, 32000.0)
                as i16
        })
        .collect()
}

/// **The precision guard of the P0 configurations**: 200 slots of noise (50 each at four levels, from a few counts to a loud floor) decode **nothing** with the window path and the late path (the
/// capped OSD reads the best-ranked failed words of a slot, and the subtraction passes read what is left: both are where a CRC-passing phantom would come from). A decode here is a regression
/// or a real discovery about the CRC and the plausibility checks, and the test reports it by slot and message. **Fails if** any slot decodes; try it with the OSD's CRC check removed.
#[cfg(feature = "fixed-point")]
#[test]
fn the_p0_configurations_decode_nothing_from_noise() {
    use mfsk_core::ft8::decode_block::CellsOptions;
    let mut phantoms = Vec::new();
    for (i, (name, options)) in [
        ("window", CellsOptions::p0_window()),
        ("late", CellsOptions::p0_late()),
    ]
    .into_iter()
    .enumerate()
    {
        for n in 0..200u64 {
            let sigma = [30.0, 300.0, 3000.0, 9000.0][(n % 4) as usize];
            let slot = noise_slot(1000 * (i as u64 + 1) + n, sigma);
            let r = profile::run_slot_s3_options(&slot, Ft8Rate::Hz8000, &options);
            for d in r.decodes {
                phantoms.push(format!("{name} slot {n} sigma {sigma}: {d:?}"));
            }
        }
    }
    assert_embedded_path();
    assert!(
        phantoms.is_empty(),
        "decodes from noise alone: {phantoms:#?}"
    );
}
