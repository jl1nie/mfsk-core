//! JT9 real-world signal validation against the WSJT-X sample
//! recording vendored under `embedded-poc/assets/130418_1742.wav`
//! (12 kHz mono PCM-16, 60 s — one full JT9 slot).
//!
//! Counterpart to `q65_wsjtx_samples.rs` and `ft4_wsjtx_samples.rs`.

#![cfg(all(feature = "jt9", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::Path;

use mfsk_core::__internal::jt9::DecodeRequest;
use mfsk_core::jt9::Jt9Depth;
use mfsk_core::jt9::search::SearchParams;

#[allow(dead_code)]
mod common;
use common::golden::GoldenEntry;
use common::load_wav_f32 as read_wsjtx_wav_f32;

const SAMPLE_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../embedded-poc/assets/130418_1742.wav"
);

/// `Jt9::TX_START_OFFSET_S` (1.0 s, #726): the request measures DT from there, as `jt9` does.
const NOMINAL: usize = 12_000;

const FREQ_TOL_HZ: f32 = 1.0;
/// `jt9` prints DT to 0.1 s.
const DT_TOL_SEC: f32 = 0.1;
/// `jt9` prints SNR to 1 dB; this crate is within 0.5 dB of every row below.
const SNR_TOL_DB: f32 = 1.0;

const fn row(msg: &'static str, freq: f32, dt: f32, snr: f32) -> GoldenEntry {
    GoldenEntry {
        msg,
        freq_hz: Some(freq),
        dt_sec: Some(dt),
        snr_db: Some(snr),
    }
}

/// `jt9 -9 -p 60 -L 1050 -H 1550 -d 3 -f 1119 -F 20 130418_1742.wav` (the local `jt9` build of 2026-08-12, run 2026-11-14), in
/// full. The Rx window at 1119 Hz is what decodes `CQ GM7GAX IO75` (the `nqd = 1` pass's lower gates):
/// without `-f 1119` neither `jt9` nor this crate reports it, and it was in this table from the WSJT-X
/// GUI's output (2026-05-06), whose Rx frequency was there.
///
/// Until #726 this crate reached these seven with its own candidate scan, at other bins: the DTs differed
/// from `jt9`'s by up to 0.2 s (tolerance was 0.5) and the SNRs by up to 4 dB. Since `jt9::sync9` the
/// rows are `jt9`'s.
static JT9_DEEP_RX_1119: &[GoldenEntry] = &[
    row("CQ GM7GAX IO75", 1119.0, 0.0, -25.0),
    row("TF3G N7MQ CN84", 1186.0, 0.0, -18.0),
    row("K1JT KF4RWA 73", 1224.0, 0.0, -18.0),
    row("CQ M0WAY IO82", 1290.0, 0.1, -22.0),
    row("K1JT N5KDV EM41", 1346.0, 0.1, -1.0),
    row("G7CNF N4HFA EL89", 1461.0, -0.2, -15.0),
    row("JA1KAU PD0JAC -23", 1505.0, 1.2, -5.0),
];

/// `jt9 -9 -p 60 -L 1050 -H 1550 -d 1` on the same file (same build and date), in full: `ccflim` 3.0
/// keeps `K1JT N5KDV EM41` out (it decodes from the 1344.0 Hz bin, `ccfred` 2.60), and so `G7CNF N4HFA
/// EL89` is decoded at another bin, DT 0.0 and -13 dB instead of -0.2 and -15. `-d 2` prints the same.
static JT9_FAST: &[GoldenEntry] = &[
    row("TF3G N7MQ CN84", 1186.0, 0.0, -17.0),
    row("K1JT KF4RWA 73", 1224.0, 0.0, -18.0),
    row("CQ M0WAY IO82", 1290.0, 0.1, -22.0),
    row("G7CNF N4HFA EL89", 1461.0, 0.0, -13.0),
    row("JA1KAU PD0JAC -23", 1505.0, 1.2, -5.0),
];

fn jt9_search_params() -> SearchParams {
    SearchParams {
        freq_min_hz: 1050.0,
        freq_max_hz: 1550.0,
        ..mfsk_core::jt9::search::default_search_params()
    }
}

fn check(
    name: &'static str,
    decodes: &[mfsk_core::jt9::Jt9Result],
    expected: &'static [GoldenEntry],
) {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};
    assert_golden(
        decodes,
        &GoldenSet {
            name,
            expected,
            min_hits: expected.len(),
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
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}

/// Recall, precision and DT/SNR against `jt9 -d 3` with an Rx window, every row and nothing else.
#[test]
fn jt9_wsjtx_sample_deep_with_rx_window_is_jt9s() {
    let audio = read_wsjtx_wav_f32(Path::new(SAMPLE_PATH));
    let decodes = DecodeRequest::new(&audio, 12_000)
        .nominal_start(NOMINAL)
        .params(jt9_search_params())
        .depth(Jt9Depth::Deep)
        .narrow(1119.0, 20.0)
        .decode();
    check(
        "JT9 130418_1742.wav -d 3 -f 1119 -F 20",
        &decodes,
        JT9_DEEP_RX_1119,
    );
}

/// The same against `jt9 -d 1`: the shallow depth's `ccflim` is `jt9`'s too.
#[test]
fn jt9_wsjtx_sample_fast_is_jt9s() {
    let audio = read_wsjtx_wav_f32(Path::new(SAMPLE_PATH));
    let decodes = DecodeRequest::new(&audio, 12_000)
        .nominal_start(NOMINAL)
        .params(jt9_search_params())
        .depth(Jt9Depth::Fast)
        .decode();
    check("JT9 130418_1742.wav -d 1", &decodes, JT9_FAST);
}

/// `DecodeRequest::on_result` callback — real-signal
/// verification, mirroring `tests/ft8_streaming_decode.rs`'s
/// sequential-exact-match contract. The scan's candidate loop is
/// sequential with no early exit and no parallelism, so the
/// callback-delivered set must exactly equal the batch `Vec`.
#[test]
fn jt9_scan_streaming_matches_batch_exactly() {
    use mfsk_core::jt9::Jt9Result;

    let audio = read_wsjtx_wav_f32(Path::new(SAMPLE_PATH));

    let streamed: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let on_result = |r: &Jt9Result| streamed.lock().unwrap().push(r.message.to_string());
    let batch = DecodeRequest::new(&audio, 12_000)
        .params(jt9_search_params())
        .depth(Jt9Depth::Deep)
        .narrow(1119.0, 20.0)
        .on_result(&on_result)
        .decode();

    let batch_msgs: Vec<String> = batch.iter().map(|d| d.message.to_string()).collect();
    let streamed = streamed.into_inner().unwrap();
    eprintln!(
        "JT9 on_result: batch={} streamed={}",
        batch_msgs.len(),
        streamed.len()
    );
    assert_eq!(
        streamed, batch_msgs,
        "JT9 on_result: streamed callback deliveries must exactly \
         match the batch result, same order (sequential, no early exit, no \
         parallelism — no divergence mechanism exists on this path)"
    );
    assert!(
        !batch.is_empty(),
        "expected real decodes on the JT9 golden WAV"
    );
}
