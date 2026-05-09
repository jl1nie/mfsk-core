//! WSPR real-world signal validation against
//! `WSJT-X/samples/WSPR/150426_0918.wav` (12 kHz mono, 120 s = 1
//! WSPR slot).
//!
//! Skipped when the WSJT-X tree is absent.

#![cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

use mfsk_core::wspr::SearchParams;
use mfsk_core::wspr::decode::decode_scan_subtract;

#[allow(dead_code)]
mod common;

use common::load_wav_f32_opt as read_wsjtx_wav_f32;

fn sample_path() -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let p = Path::new(&manifest)
        .join("../../WSJT-X/samples/WSPR/150426_0918.wav")
        .canonicalize()
        .ok()?;
    if p.is_file() { Some(p) } else { None }
}

/// Each golden entry carries the WSPR Type-1 message string in its
/// `Display` form ("call grid pwr") plus the audio carrier in Hz
/// (= wsprd's `freq_MHz × 1e6`).
struct Golden {
    msg: &'static str,
    freq_hz: f32,
    dt_sec: f32,
}

const GOLDEN: &[Golden] = &[
    Golden {
        msg: "ND6P DM04 30",
        freq_hz: 1446.0,
        dt_sec: 1.1,
    },
    Golden {
        msg: "W5BIT EL09 17",
        freq_hz: 1460.0,
        dt_sec: 0.1,
    },
    Golden {
        msg: "WD4LHT EL89 30",
        freq_hz: 1489.0,
        dt_sec: 0.6,
    },
    Golden {
        msg: "NM7J DM26 30",
        freq_hz: 1503.0,
        dt_sec: -0.8,
    },
    Golden {
        msg: "KI7CI DM09 37",
        freq_hz: 1517.0,
        dt_sec: 0.5,
    },
    Golden {
        msg: "DJ6OL JO52 37",
        freq_hz: 1530.0,
        dt_sec: -1.9,
    },
    Golden {
        msg: "W3HH EL89 30",
        freq_hz: 1587.0,
        dt_sec: 0.8,
    },
    Golden {
        msg: "W3BI FN20 30",
        freq_hz: 1594.0,
        dt_sec: 0.7,
    },
];

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5;

#[test]
fn wspr_wsjtx_sample_recall_vs_golden() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X WSPR sample not found at ../../WSJT-X/samples/WSPR/150426_0918.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_f32(&path).expect("WAV must be 12 kHz mono PCM-16");

    // Golden carriers span 1446..1594 Hz — widen well past the
    // ±100-Hz default to keep edges in scope. WSPR slots commonly
    // hold 6–10 transmissions; weak signals score 0.15–0.4 and
    // get crowded out of the default 16-candidate budget by the
    // strong (0.7+) signals' alternate alignments. Bump candidate
    // count so the weak signals have a chance.
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1620.0,
        max_candidates: 100,
        score_threshold: 0.05,
        ..SearchParams::default()
    };

    let decodes = decode_scan_subtract(&audio, 12_000, 0, &params);

    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .map(|d| (d.message.to_string(), d.freq_hz, d.dt_sec))
        .collect();

    eprintln!("WSPR sample decoded {} message(s):", decoded.len());
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
    eprintln!("recall: {}/{} golden WSPR decodes", hits, GOLDEN.len());

    assert_eq!(
        hits,
        GOLDEN.len(),
        "WSPR WSJT-X sample recall regressed: {}/{}",
        hits,
        GOLDEN.len()
    );
}
