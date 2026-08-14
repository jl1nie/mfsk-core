//! JT9 real-world signal validation against the WSJT-X sample
//! recording vendored under `embedded-poc/assets/130418_1742.wav`
//! (12 kHz mono PCM-16, 60 s — one full JT9 slot).
//!
//! Counterpart to `q65_wsjtx_samples.rs` and `ft4_wsjtx_samples.rs`.

#![cfg(all(feature = "jt9", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::Path;

use mfsk_core::jt9::decode_scan;
use mfsk_core::jt9::search::SearchParams;

#[allow(dead_code)]
mod common;
use common::golden::GoldenEntry;
use common::load_wav_f32 as read_wsjtx_wav_f32;

const SAMPLE_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../embedded-poc/assets/130418_1742.wav"
);

// The recording `130418_1742.wav` starts ~0.91 s before the 17:42:00 slot
// boundary.  WSJT-X reports dt relative to the slot; we measure start_sample
// from the beginning of the WAV, so add WAV_PRE_ROLL_SEC to each WSJT-X dt.
const WAV_PRE_ROLL_SEC: f32 = 0.91;

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5; // ≈ 1 symbol; covers ⅛-sym lag grid + slot-offset uncertainty
const SNR_TOL_DB: f32 = 4.0;

/// All seven real signals on this recording. `CQ GM7GAX IO75` is real
/// WSJT-X GUI output (`reference_jt9_wsjtx_sample_decode.md`, user-
/// provided 2026-05-06); the other six are a local
/// `jt9 -9 -p 60 -L 1050 -H 1550 -d 3` CLI run (2026-08-14) — `TF3G
/// N7MQ CN84` / `K1JT KF4RWA 73` / `CQ M0WAY IO82` / `K1JT N5KDV
/// EM41` match the original GUI table too; `G7CNF N4HFA EL89` and
/// `JA1KAU PD0JAC -23` were previously confirmed via JTDX
/// (2026-05-08) without SNR — the CLI run backfills theirs.
///
/// Six source-faithful fixes under issue #19 lifted recall from 1/5
/// to 5/5 (see `project_jt9_wsjtx_recall.md`); `decode_scan` reaches
/// full 7/7 recall with zero phantoms as of 2026-08-14 — so the
/// budget below is 0, not a looser recall-only floor. The `params`
/// `freq_max_hz` is 1550 Hz (default `SearchParams` is 1400..1600,
/// which excludes every golden tone here) to bring all seven inside
/// the search band, including the 1505 Hz signal.
static JT9_FULL_REFERENCE: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "CQ GM7GAX IO75",
        freq_hz: Some(1119.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 0.0), // 0.91
        snr_db: Some(-25.0),
    },
    GoldenEntry {
        msg: "TF3G N7MQ CN84",
        freq_hz: Some(1186.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 0.0), // 0.91
        snr_db: Some(-18.0),
    },
    GoldenEntry {
        msg: "K1JT KF4RWA 73",
        freq_hz: Some(1224.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 0.1), // 1.01
        snr_db: Some(-19.0),
    },
    GoldenEntry {
        msg: "CQ M0WAY IO82",
        freq_hz: Some(1290.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 0.1), // 1.01
        snr_db: Some(-22.0),
    },
    GoldenEntry {
        msg: "K1JT N5KDV EM41",
        freq_hz: Some(1346.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 0.1), // 1.01
        snr_db: Some(-1.0),
    },
    GoldenEntry {
        msg: "G7CNF N4HFA EL89",
        freq_hz: Some(1461.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 0.0), // 0.91
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "JA1KAU PD0JAC -23",
        freq_hz: Some(1505.0),
        dt_sec: Some(WAV_PRE_ROLL_SEC + 1.2), // 2.11
        snr_db: Some(-5.0),
    },
];

fn jt9_search_params() -> SearchParams {
    SearchParams {
        freq_min_hz: 1050.0,
        freq_max_hz: 1550.0,
        time_tolerance_sec: 1.728,
        score_threshold: 0.05,
        max_candidates: 200,
    }
}

/// Recall, precision, and SNR together, in one `assert_golden` call —
/// see `common::golden`'s module doc for why that is one call and not
/// three. JT9 had no false-decode guard and no SNR check before this;
/// this single test replaces what used to be three separate ones
/// (a message+freq+dt-only recall test, a message-only precision
/// test, and a standalone SNR-vs-jt9 test covering only 4 of the 7
/// signals).
#[test]
fn jt9_wsjtx_sample_recall_precision_and_snr() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    let audio = read_wsjtx_wav_f32(Path::new(SAMPLE_PATH));
    // JT9 transmissions start at the top of the slot — `nominal_start_sample = 0`.
    let decodes = decode_scan(&audio, 12_000, 0, &jt9_search_params());

    assert_golden(
        &decodes,
        &GoldenSet {
            name: "JT9 130418_1742.wav",
            expected: JT9_FULL_REFERENCE,
            min_hits: 7,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: SNR_TOL_DB,
        },
        |d| DecodeView {
            msg: d.message.to_string(),
            freq_hz: d.freq_hz,
            dt_sec: d.start_sample as f32 / 12_000.0,
            snr_db: Some(d.snr_db),
        },
    );
}

/// `decode_scan_streaming`'s `on_result` callback — real-signal
/// verification, mirroring `tests/ft8_streaming_decode.rs`'s
/// sequential-exact-match contract. `decode_scan`'s candidate loop is
/// sequential with no early exit and no parallelism, so the
/// callback-delivered set must exactly equal the batch `Vec`.
#[test]
fn jt9_scan_streaming_matches_batch_exactly() {
    use mfsk_core::jt9::Jt9Result;
    use mfsk_core::jt9::decode_scan_streaming;

    let audio = read_wsjtx_wav_f32(Path::new(SAMPLE_PATH));

    let streamed: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let on_result = |r: &Jt9Result| streamed.lock().unwrap().push(r.message.to_string());
    let batch = decode_scan_streaming(&audio, 12_000, 0, &jt9_search_params(), &on_result);

    let batch_msgs: Vec<String> = batch.iter().map(|d| d.message.to_string()).collect();
    let streamed = streamed.into_inner().unwrap();
    eprintln!(
        "JT9 decode_scan_streaming: batch={} streamed={}",
        batch_msgs.len(),
        streamed.len()
    );
    assert_eq!(
        streamed, batch_msgs,
        "JT9 decode_scan_streaming: streamed callback deliveries must exactly \
         match the batch result, same order (sequential, no early exit, no \
         parallelism — no divergence mechanism exists on this path)"
    );
    assert!(
        !batch.is_empty(),
        "expected real decodes on the JT9 golden WAV"
    );
}
