//! JT9 real-world signal validation against the WSJT-X sample
//! recording shipped under `WSJT-X/samples/JT9/130418_1742.wav`
//! (12 kHz mono PCM-16, 60 s — one full JT9 slot).
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path.
//!
//! Counterpart to `q65_wsjtx_samples.rs` and `ft4_wsjtx_samples.rs`.

#![cfg(all(feature = "jt9", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use mfsk_core::jt9::decode_scan;
use mfsk_core::jt9::search::SearchParams;

fn read_wsjtx_wav_f32(path: &Path) -> Option<Vec<f32>> {
    let mut file = File::open(path).ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    if bytes.len() < 44 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    if &bytes[12..16] != b"fmt " {
        return None;
    }
    let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
    let sample_rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
    let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
    if channels != 1 || sample_rate != 12_000 || bits != 16 {
        return None;
    }
    if &bytes[36..40] != b"data" {
        return None;
    }
    let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
    let data = &bytes[44..44 + data_len.min(bytes.len() - 44)];
    let mut out = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
        out.push(s as f32 / 32_768.0);
    }
    Some(out)
}

fn sample_path() -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let p = Path::new(&manifest)
        .join("../embedded-poc/assets/130418_1742.wav")
        .canonicalize()
        .ok()?;
    if p.is_file() { Some(p) } else { None }
}

struct Golden {
    msg: &'static str,
    freq_hz: f32,
    dt_sec: f32,
}

// The recording `130418_1742.wav` starts ~0.91 s before the 17:42:00 slot
// boundary.  WSJT-X reports dt relative to the slot; we measure start_sample
// from the beginning of the WAV, so add WAV_PRE_ROLL_SEC to each WSJT-X dt.
const WAV_PRE_ROLL_SEC: f32 = 0.91;

const GOLDEN: &[Golden] = &[
    Golden {
        msg: "CQ GM7GAX IO75",
        freq_hz: 1119.0,
        dt_sec: WAV_PRE_ROLL_SEC + 0.0, // 0.91
    },
    Golden {
        msg: "TF3G N7MQ CN84",
        freq_hz: 1186.0,
        dt_sec: WAV_PRE_ROLL_SEC + 0.0, // 0.91
    },
    Golden {
        msg: "K1JT KF4RWA 73",
        freq_hz: 1224.0,
        dt_sec: WAV_PRE_ROLL_SEC + 0.1, // 1.01  (confirmed from decode output)
    },
    Golden {
        msg: "CQ M0WAY IO82",
        freq_hz: 1290.0,
        dt_sec: WAV_PRE_ROLL_SEC + 0.1, // 1.01
    },
    Golden {
        msg: "K1JT N5KDV EM41",
        freq_hz: 1346.0,
        dt_sec: WAV_PRE_ROLL_SEC + 0.1, // 1.01
    },
    // Two more signals confirmed via JTDX 2026-05-08 (the original 5-entry
    // table understated the busy-band content of this slot).
    Golden {
        msg: "G7CNF N4HFA EL89",
        freq_hz: 1461.0,
        dt_sec: WAV_PRE_ROLL_SEC + 0.0, // 0.91
    },
    Golden {
        msg: "JA1KAU PD0JAC -23",
        freq_hz: 1505.0,
        dt_sec: WAV_PRE_ROLL_SEC + 1.2, // 2.11
    },
];

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5; // ≈ 1 symbol; covers ⅛-sym lag grid + slot-offset uncertainty

// Recall is 7/7 on this golden as of 2026-05-08. Six source-faithful
// fixes under issue #19 lifted recall from 1/5 to 5/5 (see
// memory/project_jt9_wsjtx_recall.md); JTDX cross-check on the same
// WAV (2026-05-08) confirmed two additional real signals beyond the
// original 5-entry table — `G7CNF N4HFA EL89` @ 1461 Hz and
// `JA1KAU PD0JAC -23` @ 1505 Hz — both of which `decode_scan` already
// surfaces. The freq_max in `params` was bumped from 1500 → 1550 Hz
// to bring the 1505 Hz signal inside the search band.
#[test]
fn jt9_wsjtx_sample_recall_vs_golden() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: vendored JT9 sample not found at \
             embedded-poc/assets/130418_1742.wav"
        );
        return;
    };

    let audio = read_wsjtx_wav_f32(&path).expect("WAV must be 12 kHz mono PCM-16");

    // Default SearchParams is 1400..1600 Hz which excludes every
    // golden tone (1119..1346 Hz). Widen the band; everything else
    // stays at default.
    let params = SearchParams {
        freq_min_hz: 1050.0,
        freq_max_hz: 1550.0,
        time_tolerance_symbols: 3,
        score_threshold: 0.05,
        max_candidates: 200,
    };

    // JT9 transmissions start at the top of the slot — `nominal_start_sample = 0`.
    let decodes = decode_scan(&audio, 12_000, 0, &params);

    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .map(|d| {
            let dt = d.start_sample as f32 / 12_000.0;
            (d.message.to_string(), d.freq_hz, dt)
        })
        .collect();

    eprintln!("JT9 sample decoded {} message(s):", decoded.len());
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
    eprintln!("recall: {}/{} golden JT9 decodes", hits, GOLDEN.len());
    for g in &misses {
        eprintln!(
            "  MISSING: '{}' @ {:.1} Hz dt={:+.2}",
            g.msg, g.freq_hz, g.dt_sec
        );
    }

    assert_eq!(
        hits,
        GOLDEN.len(),
        "JT9 WSJT-X sample recall regressed: {}/{}",
        hits,
        GOLDEN.len()
    );
}
