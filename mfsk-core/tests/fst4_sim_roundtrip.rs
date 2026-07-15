//! FST4 round-trip validation against `fst4sim`-generated AWGN signals.
//!
//! `fst4sim` is the canonical WSJT-X signal generator (Fortran, from
//! `WSJT-X/lib/fst4/fst4sim.f90`).  Each WAV was produced with:
//!
//! ```text
//! fst4sim "CQ JL1NIE PM95" <nsec> 1500 0.0 0.0 0.0 1 -5 F
//! ```
//!
//! (no fading, SNR = -5 dB, f0 = 1500 Hz, DT = 0.0 s)
//!
//! WAVs live in `embedded-poc/assets/fst4_sim/` and are skipped cleanly
//! when absent.  Regenerate them with:
//!
//! ```sh
//! scripts/build_fst4sim.sh   # once, needs gfortran
//! scripts/gen_fst4_sim_wavs.sh
//! ```
//!
//! ## Adding a new sub-mode
//!
//! 1. Add a `Fst4sNNN` struct in `mfsk_core::fst4` with the correct
//!    `NSPS` / `NDOWN` / `TX_START_OFFSET_S` (see constants below).
//! 2. Add a `decode_frameNNN` function (or make the generic pipeline
//!    callable with the new struct).
//! 3. Uncomment / add the corresponding test below.
//!
//! ## Reference WSJT-X parameters for all sub-modes
//!
//! | Mode      | NSPS  | NDOWN | TX_START_OFFSET_S | baud    |
//! |-----------|-------|-------|-------------------|---------|
//! | FST4-15   |   720 |    18 | 0.5               | 16.667  |
//! | FST4-30   |  1680 |    42 | 1.0               |  7.143  |
//! | FST4-60   |  3888 |   108 | 1.0               |  3.086  |
//! | FST4-120  |  8200 |   205 | 1.0               |  1.463  |
//! | FST4-300  | 21504 |   512 | 1.0               |  0.558  |
//!
//! Source: `fst4_decode.f90` (ndown per ntrperiod) and `fst4sim.f90`
//! (TX_START_OFFSET: nsec==15 uses k=xdt+0.5, others k=xdt+1.0).

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
/// fst4sim uses DT=0.0 → signal starts at TX_START_OFFSET_S into the slot.
const GOLDEN_DT_SEC: f32 = 0.0;
const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5;

fn sim_asset_dir() -> PathBuf {
    // Allow override via env; fall back to the in-repo location.
    if let Ok(d) = std::env::var("MFSK_FST4_SIM_ASSETS_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sim")
        .to_path_buf()
}

fn sim_wav(nsec: u32) -> Option<Vec<i16>> {
    let path = sim_asset_dir().join(format!("fst4_{}_sim.wav", nsec));
    load_wav_i16_opt(&path)
}

// ────────────────────────────────────────────────────────────────────────────
// FST4-60 (existing sub-mode, always exercised)
// ────────────────────────────────────────────────────────────────────────────

#[test]
fn fst4_60_sim_roundtrip() {
    use mfsk_core::fst4::decode::decode_frame;
    use mfsk_core::msg::wsjt77::unpack77;

    let Some(audio) = sim_wav(60) else {
        eprintln!(
            "skipping fst4_60_sim_roundtrip: WAV not found in {:?}\n\
             Run scripts/build_fst4sim.sh && scripts/gen_fst4_sim_wavs.sh",
            sim_asset_dir()
        );
        return;
    };

    let decodes = decode_frame(&audio, 100.0, 3000.0, 0.8, 50);
    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .filter_map(|d| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(d.message77());
            unpack77(&m77).map(|s| (s, d.freq_hz, d.dt_sec))
        })
        .collect();

    eprintln!("FST4-60 sim decoded {} message(s):", decoded.len());
    for (m, f, dt) in &decoded {
        eprintln!("  {:7.1} Hz  dt={:+.2} s  {}", f, dt, m);
    }

    let hit = decoded.iter().any(|(m, f, dt)| {
        m == GOLDEN_MSG
            && (f - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
            && (dt - GOLDEN_DT_SEC).abs() <= DT_TOL_SEC
    });
    assert!(
        hit,
        "FST4-60 fst4sim roundtrip: '{}' @ {:.0} Hz not found in {:?}",
        GOLDEN_MSG, GOLDEN_FREQ_HZ, decoded
    );
}

// ────────────────────────────────────────────────────────────────────────────
// FST4-15
// ────────────────────────────────────────────────────────────────────────────

#[test]
fn fst4_15_sim_roundtrip() {
    use mfsk_core::fst4::Fst4s15;
    use mfsk_core::fst4::decode::{FST4_15_DOWNSAMPLE, decode_frame_for};

    let Some(audio) = sim_wav(15) else {
        eprintln!("skipping fst4_15_sim_roundtrip: run scripts/gen_fst4_sim_wavs.sh");
        return;
    };
    let decodes = decode_frame_for::<Fst4s15>(&audio, &FST4_15_DOWNSAMPLE, 100.0, 3000.0, 0.8, 50);
    check_golden_hit(15, &decodes);
}

// ────────────────────────────────────────────────────────────────────────────
// FST4-30
// ────────────────────────────────────────────────────────────────────────────

#[test]
fn fst4_30_sim_roundtrip() {
    use mfsk_core::fst4::Fst4s30;
    use mfsk_core::fst4::decode::{FST4_30_DOWNSAMPLE, decode_frame_for};

    let Some(audio) = sim_wav(30) else {
        eprintln!("skipping fst4_30_sim_roundtrip: run scripts/gen_fst4_sim_wavs.sh");
        return;
    };
    let decodes = decode_frame_for::<Fst4s30>(&audio, &FST4_30_DOWNSAMPLE, 100.0, 3000.0, 0.8, 50);
    check_golden_hit(30, &decodes);
}

// ────────────────────────────────────────────────────────────────────────────
// FST4-120
// ────────────────────────────────────────────────────────────────────────────

#[test]
fn fst4_120_sim_roundtrip() {
    use mfsk_core::fst4::Fst4s120;
    use mfsk_core::fst4::decode::{FST4_120_DOWNSAMPLE, decode_frame_for};

    let Some(audio) = sim_wav(120) else {
        eprintln!("skipping fst4_120_sim_roundtrip: run scripts/gen_fst4_sim_wavs.sh");
        return;
    };
    let decodes =
        decode_frame_for::<Fst4s120>(&audio, &FST4_120_DOWNSAMPLE, 100.0, 3000.0, 0.8, 50);
    check_golden_hit(120, &decodes);
}

// ────────────────────────────────────────────────────────────────────────────
// FST4-300
// ────────────────────────────────────────────────────────────────────────────

#[test]
fn fst4_300_sim_roundtrip() {
    use mfsk_core::fst4::Fst4s300;
    use mfsk_core::fst4::decode::{FST4_300_DOWNSAMPLE, decode_frame_for};

    let Some(audio) = sim_wav(300) else {
        eprintln!("skipping fst4_300_sim_roundtrip: run scripts/gen_fst4_sim_wavs.sh");
        return;
    };
    let decodes =
        decode_frame_for::<Fst4s300>(&audio, &FST4_300_DOWNSAMPLE, 100.0, 3000.0, 0.8, 50);
    check_golden_hit(300, &decodes);
}

// ────────────────────────────────────────────────────────────────────────────
// Shared assertion helper
// ────────────────────────────────────────────────────────────────────────────

fn check_golden_hit(nsec: u32, decodes: &[mfsk_core::fst4::decode::DecodeResult]) {
    use mfsk_core::msg::wsjt77::unpack77;
    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .filter_map(|d| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(d.message77());
            unpack77(&m77).map(|s| (s, d.freq_hz, d.dt_sec))
        })
        .collect();
    eprintln!("FST4-{} sim decoded {} message(s):", nsec, decoded.len());
    for (m, f, dt) in &decoded {
        eprintln!("  {:7.1} Hz  dt={:+.2} s  {}", f, dt, m);
    }
    let hit = decoded.iter().any(|(m, f, dt)| {
        m == GOLDEN_MSG
            && (f - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
            && (dt - GOLDEN_DT_SEC).abs() <= DT_TOL_SEC
    });
    assert!(
        hit,
        "FST4-{} fst4sim roundtrip: '{}' @ {:.0} Hz not found in {:?}",
        nsec, GOLDEN_MSG, GOLDEN_FREQ_HZ, decoded
    );
}
