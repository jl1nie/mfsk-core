//! FST4-60 real-world signal validation against
//! `WSJT-X/samples/FST4+FST4W/210115_0058.wav` (12 kHz mono, 60 s).
//!
//! Skipped when the WSJT-X tree is absent.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

use mfsk_core::fst4::decode::decode_frame;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

fn sample_path() -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let p = Path::new(&manifest)
        .join("../../WSJT-X/samples/FST4+FST4W/210115_0058.wav")
        .canonicalize()
        .ok()?;
    if p.is_file() { Some(p) } else { None }
}

struct Golden {
    msg: &'static str,
    freq_hz: f32,
    dt_sec: f32,
}

const GOLDEN: &[Golden] = &[Golden {
    msg: "CQ N5TM EL29",
    freq_hz: 1101.0,
    dt_sec: 0.3,
}];

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5;

#[test]
#[ignore = "FST4-60 host path currently 0/1 against WSJT-X golden \
            (decode_frame returns 0 messages) — run with \
            `cargo test -- --ignored` to track repair"]
fn fst4_60_wsjtx_sample_recall_vs_golden() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FST4 sample not found at \
             ../../WSJT-X/samples/FST4+FST4W/210115_0058.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let decodes = decode_frame(&audio, 100.0, 3000.0, 1.0, 50);

    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .filter_map(|d| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(d.message77());
            unpack77(&m77).map(|s| (s, d.freq_hz, d.dt_sec))
        })
        .collect();

    eprintln!("FST4-60 sample decoded {} message(s):", decoded.len());
    for (m, f, dt) in &decoded {
        eprintln!("  freq={:6.1} Hz dt={:+.2} s : {}", f, dt, m);
    }

    let mut hits = 0usize;
    for g in GOLDEN {
        let hit = decoded.iter().any(|(m, f, dt)| {
            m == g.msg
                && (f - g.freq_hz).abs() <= FREQ_TOL_HZ
                && (dt - g.dt_sec).abs() <= DT_TOL_SEC
        });
        if hit {
            hits += 1;
        } else {
            eprintln!(
                "  MISSING: '{}' @ {:.1} Hz dt={:+.2}",
                g.msg, g.freq_hz, g.dt_sec
            );
        }
    }
    eprintln!("recall: {}/{} golden FST4-60 decodes", hits, GOLDEN.len());

    assert_eq!(
        hits,
        GOLDEN.len(),
        "FST4-60 WSJT-X sample recall regressed: {}/{}",
        hits,
        GOLDEN.len()
    );
}
