//! MSK144 real-world signal validation against the two WSJT-X sample
//! recordings shipped under `WSJT-X/samples/MSK144/`.
//!
//! Golden decode lists come from WSJT-X's own decode of these files
//! (`reference_msk144_jt65_wsjtx_sample_decode.md`): `utc snr dt_s
//! freq_hz sync msg`, where `dt_s` is WSJT-X's `tdec` (decode time
//! within the slot — `mskrtd.f90`'s reported timestamp — not a
//! symbol-timing offset the way FT8's `dt` is), matching
//! [`SlotDecode::tsec`] here.
//!
//! Phase 3's burst-detection thresholds were flagged from the start
//! as the highest-risk, most-likely-to-need-iteration part of the
//! whole MSK144 port (hand-tuned WSJT-X magic constants with no
//! principled derivation, validated against synthetic/independent-
//! oracle signals only up to this point) — but this first real-WAV
//! run recovers all 3 golden messages across both files with no
//! tuning needed (freq within a few Hz, `tsec` exact), so this is a
//! strict gate like the other protocols' golden-WAV tests.
//!
//! SNR is gated too, exact-match after the `analytic_signal` fixed
//! bandpass filter fix (`core/dsp/analytic.rs` — WSJT-X's `analytic()`
//! always applies a 1500 Hz-centered raised-cosine bandpass before
//! computing `pmax`/`pnoise`; the initial port omitted it, producing
//! a systematic -1dB bias on all 3 golden decodes). `SNR_TOL_DB`
//! leaves headroom for future incidental changes upstream of the SNR
//! computation (e.g. FFT backend swaps) without making this an
//! exact-bit-match gate.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path so developers cloning only `mfsk-core` won't see a
//! failure they can't fix.

#![cfg(all(
    feature = "msk144",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use std::path::PathBuf;

use mfsk_core::msk144::decode::{Depth, decode_slot};

#[allow(dead_code)]
mod common;
use common::golden::GoldenEntry;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

fn sample_path(name: &str) -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        &format!("msk144/{name}"),
        Some(&format!("MSK144/{name}")),
    )
}

const FREQ_TOL_HZ: f32 = 15.0;
const TSEC_TOL: f32 = 1.0;
const SNR_TOL_DB: f32 = 1.0;

/// WSJT-X-published golden decode for `181211_120500.wav` (see
/// `reference_msk144_jt65_wsjtx_sample_decode.md`).
static MSK144_120500_REFERENCE: &[GoldenEntry] = &[GoldenEntry {
    msg: "K1JT WA4CQG EM72",
    freq_hz: Some(1488.0),
    dt_sec: Some(8.7),
    snr_db: Some(8.0),
}];

/// WSJT-X-published golden decodes for `181211_120800.wav`.
static MSK144_120800_REFERENCE: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "CQ W4IMD EM84",
        freq_hz: Some(1458.0),
        dt_sec: Some(4.6),
        snr_db: Some(5.0),
    },
    GoldenEntry {
        msg: "CQ KD9VV EN71",
        freq_hz: Some(1496.0),
        dt_sec: Some(12.2),
        snr_db: Some(7.0),
    },
];

/// Recall, precision, and SNR together, on both golden recordings —
/// one `assert_golden` call per file, replacing what used to be three
/// separate tests (two bespoke recall-only checks sharing a hand-
/// rolled `Golden`/`check()`, plus a third message-only precision
/// test duplicating the same two file loads).
///
/// Phase 3's burst-detection thresholds were flagged from the start
/// as the highest-risk, most-likely-to-need-iteration part of the
/// whole MSK144 port (hand-tuned WSJT-X magic constants with no
/// principled derivation, validated against synthetic/independent-
/// oracle signals only up to that point) — but the first real-WAV run
/// recovered all 3 golden messages across both files with no tuning
/// needed (freq within a few Hz, `tsec` exact), so this is a strict
/// gate like the other protocols' golden-WAV tests: `max_extra: 0`.
///
/// MSK144 is also the protocol most exposed to phantom decodes,
/// because `msk144sync.f90`-faithful search attempts OSD on the order
/// of a thousand times per file (measured under issue #246:
/// 1044-1116 attempts, 0 successes on these very recordings). Every
/// one of those is an opportunity to synthesise a codeword out of
/// noise, exactly as WSPR's OSD path did.
///
/// SNR is gated too, exact-match after the `analytic_signal` fixed
/// bandpass filter fix (`core/dsp/analytic.rs` — WSJT-X's
/// `analytic()` always applies a 1500 Hz-centered raised-cosine
/// bandpass before computing `pmax`/`pnoise`; the initial port
/// omitted it, producing a systematic -1dB bias on all 3 golden
/// decodes). `SNR_TOL_DB` leaves headroom for future incidental
/// changes upstream of the SNR computation (e.g. FFT backend swaps)
/// without making this an exact-bit-match gate.
#[test]
fn msk144_wsjtx_samples_recall_precision_and_snr() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    for (file, expected, floor) in [
        ("181211_120500.wav", MSK144_120500_REFERENCE, 1usize),
        ("181211_120800.wav", MSK144_120800_REFERENCE, 2),
    ] {
        let Some(path) = sample_path(file) else {
            eprintln!("skipping: MSK144 golden {file} not found");
            continue;
        };
        let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

        // fc/ntol: a single nominal-center-frequency guess wide enough
        // to cover all of this file's signals (1458-1496 Hz observed
        // in the golden list) -- `ntol` here is the coarse
        // squared-signal search tolerance (`detect_burst_candidates`),
        // independent of `msk144_sync`'s own tighter internal
        // fine-search width.
        let decodes = decode_slot(&audio, 1477.0, 60.0, Depth::Deep);

        assert_golden(
            &decodes,
            &GoldenSet {
                name: file,
                expected,
                min_hits: floor,
                max_extra: 0,
            },
            Tolerances {
                freq_hz: FREQ_TOL_HZ,
                dt_sec: TSEC_TOL,
                snr_db: SNR_TOL_DB,
            },
            |d| DecodeView {
                msg: d.message.clone(),
                freq_hz: d.freq_hz,
                dt_sec: d.tsec,
                snr_db: Some(d.snr_db as f32),
            },
        );
    }
}
