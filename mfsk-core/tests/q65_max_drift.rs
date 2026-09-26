//! WSJT-X's Q65 **Max Drift** (`q65_ccf_22`'s idrift search and
//! `q65_loops`' `twkfreq` de-chirp; issue #471).
//!
//! The gate synthesises a Q65-30A frame whose tones drift linearly, the
//! way `q65sim`'s `f1` (Hz/min) does, and checks that the plain decode
//! loses it while `.max_drift()` recovers it. `drift_measure` (ignored)
//! decodes `q65sim` WAVs named in `MFSK_Q65_DRIFT_WAVS`.

#![cfg(feature = "q65")]

#[allow(dead_code)]
mod common;

use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{DecodeRequest, Q65a30, encode_channel_symbols};

const FS: u32 = 12_000;
const FREQ: f32 = 1500.0;
const NSPS: usize = 3600;

/// A Q65-30A slot: the frame starts at 0.5 s (the nominal start) and its
/// frequency moves `f1` Hz per minute, measured from the slot's centre as
/// `q65sim.f90` does (`f = f0 + f1*(t-15)/60`).
fn drifting_slot(f1_hz_per_min: f32) -> Vec<f32> {
    let bits = mfsk_core::msg::q65::pack77_q65("K1ABC", "JA1ABC", "-15").unwrap();
    let tones = encode_channel_symbols(&bits);
    let mut a = vec![0f32; 30 * FS as usize];
    let start = FS as usize / 2;
    let baud = FS as f32 / NSPS as f32;
    let mut phi = 0f64;
    for (k, &t) in tones.iter().enumerate() {
        for i in 0..NSPS {
            let n = start + k * NSPS + i;
            let secs = n as f32 / FS as f32;
            let f = FREQ + t as f32 * baud + f1_hz_per_min * (secs - 15.0) / 60.0;
            phi += core::f64::consts::TAU * f as f64 / FS as f64;
            a[n] = 0.3 * phi.sin() as f32;
        }
    }
    a
}

fn params() -> SearchParams {
    // Upstream narrows to nfqso ± ntol when drift is searched.
    SearchParams {
        freq_min_hz: FREQ - 50.0,
        freq_max_hz: FREQ + 50.0,
        ..mfsk_core::q65::search::default_search_params()
    }
}

fn decodes(audio: &[f32], max_drift: u32) -> Vec<String> {
    DecodeRequest::<Q65a30>::new(audio, FS, FS as usize / 2, params())
        .max_drift(max_drift)
        .decode()
        .into_iter()
        .map(|r| r.message)
        .collect()
}

const WANT: &str = "K1ABC JA1ABC -15";

#[test]
fn no_drift_decodes_either_way() {
    let a = drifting_slot(0.0);
    assert_eq!(decodes(&a, 0), [WANT]);
    assert_eq!(decodes(&a, 10), [WANT]);
}

#[test]
fn max_drift_recovers_a_drifting_signal() {
    // 60 Hz/min: 25 Hz across the 25.5 s frame, 7.5 bins at 3.33 Hz.
    let a = drifting_slot(60.0);
    assert!(decodes(&a, 0).is_empty(), "{:?}", decodes(&a, 0));
    assert_eq!(decodes(&a, 10), [WANT]);
}

/// Decode each `q65sim` WAV listed (whitespace-separated) in
/// `MFSK_Q65_DRIFT_WAVS` at Max Drift 0 and `MFSK_Q65_MAX_DRIFT`
/// (default 10), printing the messages and the 78th-bit flag.
#[test]
#[ignore]
fn drift_measure() {
    let Ok(list) = std::env::var("MFSK_Q65_DRIFT_WAVS") else {
        return;
    };
    let md: u32 = std::env::var("MFSK_Q65_MAX_DRIFT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(10);
    for path in list.split_whitespace() {
        let a = common::load_wav_f32(path);
        let run = |d| {
            DecodeRequest::<Q65a30>::new(&a, FS, FS as usize / 2, params())
                .max_drift(d)
                .decode()
                .into_iter()
                .map(|r| format!("{}{}", r.message, if r.copied_last_tx { "#" } else { "" }))
                .collect::<Vec<_>>()
        };
        println!("{path} d0={:?} d{md}={:?}", run(0), run(md));
    }
}
