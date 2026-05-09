//! FT4 real-world signal validation against the WSJT-X sample
//! recording shipped under `WSJT-X/samples/FT4/000000_000002.wav`.
//!
//! The sample is 12 kHz mono PCM-16, 6.048 s long (shorter than the
//! nominal 7.5 s FT4 slot — we zero-pad to `SLOT_SAMPLES = 90 000`
//! before running the decoder). WSJT-X's own decode of this file
//! yields six FT4 messages — recorded as the golden list below
//! (`reference_ft4_wsjtx_sample_decode.md`).
//!
//! This is the FT4 counterpart to `q65_wsjtx_samples.rs`. It exists
//! to catch regressions in the *generic* DSP path that FT4 shares
//! with FT8: in particular, the `core::dsp::subtract` rewrite that
//! turned on GFSK shaping for FT4 (commit cec9472) had no real-WAV
//! coverage before this test was added.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path so developers cloning only `mfsk-core` won't see
//! a failure they can't fix.

#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

use mfsk_core::ft4::decode::decode_frame_subtract;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s × 12 kHz

fn sample_path() -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let p = Path::new(&manifest)
        .join("../../WSJT-X/samples/FT4/000000_000002.wav")
        .canonicalize()
        .ok()?;
    if p.is_file() { Some(p) } else { None }
}

/// WSJT-X-published golden decode list (see
/// `reference_ft4_wsjtx_sample_decode.md`). `freq_hz` is the WSJT-X
/// reported carrier (Hz); `dt_sec` is its DT column.
struct Golden {
    msg: &'static str,
    freq_hz: f32,
    dt_sec: f32,
}

const GOLDEN: &[Golden] = &[
    Golden {
        msg: "N1TRK N4FKH 569 VA",
        freq_hz: 296.0,
        dt_sec: -0.2,
    },
    Golden {
        msg: "N1TRK KB7RUQ RR73",
        freq_hz: 421.0,
        dt_sec: -0.4,
    },
    Golden {
        msg: "CQ RU AB5XS EM12",
        freq_hz: 560.0,
        dt_sec: -0.1,
    },
    Golden {
        msg: "NZ7P WA7JAY 589 CA",
        freq_hz: 726.0,
        dt_sec: 0.1,
    },
    Golden {
        msg: "KB0VHA KA1YQC R 539 MA",
        freq_hz: 1148.0,
        dt_sec: 0.3,
    },
    Golden {
        msg: "W9JA PY2APK RRR",
        freq_hz: 520.0,
        dt_sec: -0.3,
    },
];

const FREQ_TOL_HZ: f32 = 12.0;
const DT_TOL_SEC: f32 = 0.3;

#[test]
fn ft4_wsjtx_sample_recall_vs_golden() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    // Zero-pad / truncate to the FT4 slot length the decoder expects.
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);

    // Wide search — FT4 audio band is ~100..2700 Hz.
    // `max_cand` matters: synth diagnostics show that at +10 dB SNR
    // with the polynomial-baseline divisor in `coarse_sync` (slice 1
    // of #18), many spurious candidates rank above the true signal
    // — bumping max_cand from 50 → 100 takes recall from 0/10 to
    // 10/10. Real-WAV recordings carry 6+ coexisting signals each
    // contributing several alternate alignments, so 200 is the
    // working budget.
    let decodes = decode_frame_subtract(&audio, 100.0, 2700.0, 0.05, 2000);

    // Enumerate decodes (msg + freq + dt) for diagnostic visibility.
    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .filter_map(|d| {
            let mut msg77 = [0u8; 77];
            msg77.copy_from_slice(d.message77());
            unpack77(&msg77).map(|s| (s, d.freq_hz, d.dt_sec))
        })
        .collect();

    eprintln!("FT4 sample decoded {} message(s):", decoded.len());
    for (m, f, dt) in &decoded {
        eprintln!("  freq={:6.1} Hz dt={:+.2} s : {}", f, dt, m);
    }

    let mut hits = 0usize;
    let mut misses: Vec<&Golden> = Vec::new();
    for g in GOLDEN {
        let hit = decoded.iter().any(|(m, f, dt)| {
            m == g.msg
                && (f - g.freq_hz).abs() <= FREQ_TOL_HZ
                && (dt - g.dt_sec).abs() <= DT_TOL_SEC
        });
        if hit {
            hits += 1;
        } else {
            misses.push(g);
        }
    }
    eprintln!("recall: {}/{} golden FT4 decodes", hits, GOLDEN.len());
    for g in &misses {
        eprintln!(
            "  MISSING: '{}' @ {:.1} Hz dt={:+.2}",
            g.msg, g.freq_hz, g.dt_sec
        );
    }

    // Strict gate: all 6 golden messages must be recovered. WSJT-X
    // itself prints all 6 from this file; we have no excuse to miss
    // any once the FT4 receive chain is healthy. If this drops, the
    // FT4 path (likely subtract / GFSK shaping) has regressed.
    assert_eq!(
        hits,
        GOLDEN.len(),
        "FT4 WSJT-X sample recall regressed: {}/{}",
        hits,
        GOLDEN.len()
    );
}
