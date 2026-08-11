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
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

fn sample_path(name: &str) -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        &format!("msk144/{name}"),
        Some(&format!("MSK144/{name}")),
    )
}

/// WSJT-X-published golden decode (see
/// `reference_msk144_jt65_wsjtx_sample_decode.md`).
struct Golden {
    msg: &'static str,
    freq_hz: f32,
    tsec: f32,
    snr_db: i32,
}

const FREQ_TOL_HZ: f32 = 15.0;
const TSEC_TOL: f32 = 1.0;
const SNR_TOL_DB: i32 = 1;

fn check(name: &str, golden: &[Golden]) {
    let Some(path) = sample_path(name) else {
        eprintln!("skipping: WSJT-X MSK144 sample not found at ../../WSJT-X/samples/MSK144/{name}");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    // fc/ntol: a single nominal-center-frequency guess wide enough to
    // cover all of this file's signals (1458-1496 Hz observed in the
    // golden list) -- `ntol` here is the coarse squared-signal search
    // tolerance (`detect_burst_candidates`), independent of
    // `msk144_sync`'s own tighter internal fine-search width.
    let decodes = decode_slot(&audio, 1477.0, 60.0, Depth::Deep);

    eprintln!("MSK144 {name} decoded {} message(s):", decodes.len());
    for d in &decodes {
        eprintln!(
            "  freq={:6.1} Hz tsec={:5.1} snr={:+3} : {}",
            d.freq_hz, d.tsec, d.snr_db, d.message
        );
    }

    let mut hits = 0usize;
    let mut misses: Vec<&Golden> = Vec::new();
    for g in golden {
        let hit = decodes.iter().any(|d| {
            d.message == g.msg
                && (d.freq_hz - g.freq_hz).abs() <= FREQ_TOL_HZ
                && (d.tsec - g.tsec).abs() <= TSEC_TOL
                && (d.snr_db - g.snr_db).abs() <= SNR_TOL_DB
        });
        if hit {
            hits += 1;
        } else {
            misses.push(g);
        }
    }
    eprintln!(
        "recall: {hits}/{} golden MSK144 decodes for {name}",
        golden.len()
    );
    for g in &misses {
        eprintln!(
            "  MISSING: '{}' @ {:.1} Hz tsec={:.1} snr={:+}",
            g.msg, g.freq_hz, g.tsec, g.snr_db
        );
    }

    // Strict gate: WSJT-X decodes every golden message from these
    // files, and so does this port (verified 2026-07-18) — a drop
    // means the MSK144 receive chain has regressed.
    assert_eq!(
        hits,
        golden.len(),
        "MSK144 WSJT-X sample recall regressed for {name}: {}/{}",
        hits,
        golden.len()
    );
}

#[test]
fn msk144_181211_120500_wsjtx_sample() {
    check(
        "181211_120500.wav",
        &[Golden {
            msg: "K1JT WA4CQG EM72",
            freq_hz: 1488.0,
            tsec: 8.7,
            snr_db: 8,
        }],
    );
}

#[test]
fn msk144_181211_120800_wsjtx_sample() {
    check(
        "181211_120800.wav",
        &[
            Golden {
                msg: "CQ W4IMD EM84",
                freq_hz: 1458.0,
                tsec: 4.6,
                snr_db: 5,
            },
            Golden {
                msg: "CQ KD9VV EN71",
                freq_hz: 1496.0,
                tsec: 12.2,
                snr_db: 7,
            },
        ],
    );
}

/// Precision: nothing beyond the known signals may be emitted, on
/// both recordings.
///
/// MSK144 had no false-decode guard. Its own `check()` above asserts
/// recall only — and MSK144 is the protocol most exposed to this
/// class of bug, because `msk144sync.f90`-faithful search attempts
/// OSD on the order of a thousand times per file (measured under
/// issue #246: 1044-1116 attempts, 0 successes on these very
/// recordings). Every one of those is an opportunity to synthesise a
/// codeword out of noise, exactly as WSPR's OSD path did.
#[test]
fn msk144_wsjtx_samples_precision() {
    use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};

    for (file, expected, floor) in [
        (
            "181211_120500.wav",
            &[GoldenEntry::msg("K1JT WA4CQG EM72")][..],
            1usize,
        ),
        (
            "181211_120800.wav",
            &[
                GoldenEntry::msg("CQ W4IMD EM84"),
                GoldenEntry::msg("CQ KD9VV EN71"),
            ][..],
            2,
        ),
    ] {
        let Some(path) = sample_path(file) else {
            eprintln!("skipping: MSK144 golden {file} not found");
            continue;
        };
        let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
        let decodes = decode_slot(&audio, 1477.0, 60.0, Depth::Deep);

        assert_golden(
            &decodes,
            &GoldenSet {
                name: "MSK144",
                expected: Box::leak(expected.to_vec().into_boxed_slice()),
                min_hits: floor,
                max_extra: 0,
            },
            Tolerances::default(),
            |d| DecodeView {
                msg: d.message.clone(),
                freq_hz: d.freq_hz,
                dt_sec: d.tsec,
                snr_db: None,
            },
        );
    }
}
