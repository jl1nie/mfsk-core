//! FT4 real-world signal validation against the WSJT-X sample
//! recording shipped under `WSJT-X/samples/FT4/000000_000002.wav`.
//!
//! The sample is 12 kHz mono PCM-16, 6.048 s long (shorter than the
//! nominal 7.5 s FT4 slot — we zero-pad to `SLOT_SAMPLES = 90 000`
//! before running the decoder). WSJT-X's own GUI decode of this file
//! yields six FT4 messages (`reference_ft4_wsjtx_sample_decode.md`);
//! a real `jt9 -5 -p 15 -L 300 -H 2700 -d 3` CLI run over the full
//! 300–2700 Hz band finds all 14 signals actually present, with
//! freq/dt/SNR — that fuller set is `FT4_FULL_REFERENCE` below, and
//! is what every test in this file checks against via
//! `common::golden::assert_golden`.
//!
//! This is the FT4 counterpart to `q65_wsjtx_samples.rs`. It exists
//! to catch regressions in the *generic* DSP path that FT4 shares
//! with FT8: in particular, the `engine::dsp::subtract` rewrite that
//! turned on GFSK shaping for FT4 (commit cec9472) had no real-WAV
//! coverage before this test was added.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path so developers cloning only `mfsk-core` won't see
//! a failure they can't fix.

#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::golden::GoldenEntry;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s × 12 kHz

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream("ft4/000000_000002.wav", Some("FT4/000000_000002.wav"))
}

const FREQ_TOL_HZ: f32 = 12.0;
const DT_TOL_SEC: f32 = 0.3;

/// Every decode real `jt9 -5 -p 15 -L 300 -H 2700 -d 3` reports on
/// this recording — freq/dt/SNR captured from that exact run
/// (2026-08-14), not estimated. Previously message-only
/// (`GoldenEntry::msg`): `assert_golden` was checking recall and
/// precision on text alone, so a decode landing on the right
/// message at the *wrong* frequency or time would have passed
/// silently — the same blind spot this file's own SNR test
/// (below) already closed for SNR specifically.
static FT4_FULL_REFERENCE: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "AC6BW KR9A R 559 WI",
        freq_hz: Some(2300.0),
        dt_sec: Some(0.2),
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "CQ RU AB5XS EM12",
        freq_hz: Some(560.0),
        dt_sec: Some(-0.1),
        snr_db: Some(-7.0),
    },
    GoldenEntry {
        msg: "CQ RU N9OY EN43",
        freq_hz: Some(1640.0),
        dt_sec: Some(-0.2),
        snr_db: Some(-4.0),
    },
    GoldenEntry {
        msg: "CQ RU W0FRC DM79",
        freq_hz: Some(2560.0),
        dt_sec: Some(-0.3),
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "K1JT WB4HXE 559 GA",
        freq_hz: Some(1910.0),
        dt_sec: Some(0.2),
        snr_db: Some(-11.0),
    },
    GoldenEntry {
        msg: "KB0VHA KA1YQC R 539 MA",
        freq_hz: Some(1149.0),
        dt_sec: Some(0.3),
        snr_db: Some(16.0),
    },
    GoldenEntry {
        msg: "N1TRK KB7RUQ RR73",
        freq_hz: Some(422.0),
        dt_sec: Some(-0.4),
        snr_db: Some(-10.0),
    },
    GoldenEntry {
        msg: "N1TRK N4FKH 569 VA",
        freq_hz: Some(297.0),
        dt_sec: Some(-0.2),
        snr_db: Some(-10.0),
    },
    GoldenEntry {
        msg: "NI6G W7DRW 569 AZ",
        freq_hz: Some(2567.0),
        dt_sec: Some(-0.3),
        snr_db: Some(-7.0),
    },
    GoldenEntry {
        msg: "NZ7P WA7JAY 589 CA",
        freq_hz: Some(727.0),
        dt_sec: Some(0.1),
        snr_db: Some(-13.0),
    },
    GoldenEntry {
        msg: "VE3LON K7RL R 549 WA",
        freq_hz: Some(2067.0),
        dt_sec: Some(0.3),
        snr_db: Some(5.0),
    },
    GoldenEntry {
        msg: "W7BOB KJ7G RR73",
        freq_hz: Some(2413.0),
        dt_sec: Some(-0.4),
        snr_db: Some(-17.0),
    },
    GoldenEntry {
        msg: "W9JA PY2APK RRR",
        freq_hz: Some(520.0),
        dt_sec: Some(-0.3),
        snr_db: Some(-9.0),
    },
    GoldenEntry {
        msg: "WD9IGY KX1X 73",
        freq_hz: Some(2310.0),
        dt_sec: Some(0.2),
        snr_db: Some(-1.0),
    },
];

/// Wide-band recall: with `sic_rounds(3)` and a 100–2700 Hz search
/// (vs. `ft4_wsjtx_sample_reaches_jt9_parity_with_sic`'s narrower
/// 300–2700 Hz + `sic_rounds(2)`), this reaches the same full 14/14
/// jt9 parity, zero phantoms — verified 2026-08-14, so the budget is
/// `max_extra: 0` rather than a looser recall-only check.
#[test]
fn ft4_wsjtx_sample_recall_vs_golden() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

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
    // `max_cand=100` matches WSJT-X's own `getcandidates4.f90`
    // `MAXCAND=100`: since `engine::ft4_coarse::ft4_coarse_sync`
    // (dapper-soaring-nest plan, Phase 1) emits one candidate per
    // frequency-domain peak instead of the old generic Costas-lag
    // search's up-to-8-per-frequency, this real 6-signal WAV needed
    // only 31 candidates even at the old `max_cand=2000` ceiling — no
    // large safety margin required any more. (The stale comment this
    // replaced described the old search's redundancy-driven budget,
    // now obsolete.)
    let out = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(3)
        .decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FT4 000000_000002.wav (wide band, sic_rounds(3))",
            expected: FT4_FULL_REFERENCE,
            // Full parity with real jt9, same as the sic_rounds(2)
            // narrow-band test — the wider search doesn't cost
            // precision here. If this drops, the FT4 path (likely
            // subtract / GFSK shaping) has regressed.
            min_hits: 14,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: 2.0,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}

/// Precision: the decoder must emit **nothing** the reference decoder
/// doesn't. Also pins reported SNR against real `jt9`'s.
///
/// FT4 had no false-decode guard of any kind. A real local
/// `jt9 -5 -p 15 -L 300 -H 2700 -d 3` run over the same WAV finds the
/// 14 `FT4_FULL_REFERENCE` entries below. Against that set this crate
/// emits **11 decodes, all real, zero phantom** — so the budget is 0,
/// and the recall floor states the 3 it does not reach rather than
/// hiding them.
///
/// FT4 was also, for a while, the only implemented protocol with
/// **no SNR check of any kind** — neither against a reference decoder
/// nor against an injected value. That is the gap issue #255 walked
/// into: FT4's reported SNR was ~6.9 dB low for as long as the crate
/// had existed, because `engine::llr::compute_snr_db`'s generic
/// adjacent-tone heuristic stood in for `ft4_decode.f90`'s real
/// `10·log10(candidate_score − 1) − 14.8`, and nothing measured it.
/// `FT4_FULL_REFERENCE.snr_db` pins the fixed formula
/// (`engine::pipeline::ft4_snr_db`) via the `Tolerances { snr_db: 2.0,
/// .. }` below. Measured agreement across the 11 decodes this crate
/// and `jt9` share, spanning −17…+16 dB: mean error +0.06 dB, max
/// |error| 0.46 dB — the closest SNR agreement of any protocol here.
/// 2 dB is loose enough that a refactor or a rebuilt corpus won't trip
/// it, tight enough that the ~6.9 dB error #255 fixed could never pass
/// again.
///
/// Written through `common::golden::assert_golden`, which asserts
/// recall, precision, and SNR together — see its module doc for why
/// that is one call and not three.
#[test]
fn ft4_wsjtx_sample_precision_vs_reference_decoder() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    let Some(path) = sample_path() else {
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let out = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50).decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FT4 000000_000002.wav",
            expected: FT4_FULL_REFERENCE,
            // 11 of jt9's 14. Raising this is a sensitivity win;
            // lowering it is a regression.
            min_hits: 11,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: 2.0,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}

/// With SIC enabled, FT4 reaches **full parity with real `jt9`** on
/// this recording — 14/14, zero phantoms.
///
/// `ft4_wsjtx_sample_precision_vs_reference_decoder` above asserts
/// 11/14, which is what the *default* single-pass strategy reaches.
/// That is not a decoder deficiency: all three it misses are weak
/// signals sitting inside a stronger neighbour's 83 Hz occupied
/// bandwidth, which is exactly what successive interference
/// cancellation exists to recover:
///
/// | missed | SNR | masked by | Δf |
/// |---|---:|---|---:|
/// | `W9JA PY2APK RRR` @ 520 Hz | -9 | `CQ RU AB5XS EM12` @ 560, -7 | 40 Hz |
/// | `AC6BW KR9A R 559 WI` @ 2300 Hz | -15 | `WD9IGY KX1X 73` @ 2310, -1 | 10 Hz |
/// | `CQ RU W0FRC DM79` @ 2560 Hz | -15 | `NI6G W7DRW 569 AZ` @ 2567, -7 | 7 Hz |
///
/// The comparison against `jt9` was therefore not like-for-like:
/// real `jt9` runs its own multi-pass subtraction by default, and
/// `DecodeRequest`'s default strategy is `__single_pass` (for FT8
/// too — `sic_rounds` is a field default that only takes effect once
/// `.sic_rounds()` switches the strategy). A caller wanting
/// WSJT-X-equivalent recall must ask for it.
///
/// Two rounds suffice; three costs more and finds nothing extra.
/// Measured on this file: 5.0 ms default → 71.3 ms with
/// `.sic_rounds(2)`, against a 7.5 s slot — about 1 % of the slot for
/// a 27 % recall gain.
#[test]
fn ft4_wsjtx_sample_reaches_jt9_parity_with_sic() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    let Some(path) = sample_path() else {
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let out = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50)
        .sic_rounds(2)
        .decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FT4 000000_000002.wav + sic_rounds(2)",
            expected: FT4_FULL_REFERENCE,
            // Full parity with real jt9. This is a hard floor: SIC is
            // the whole point of this test, so 13 is a regression.
            min_hits: 14,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: 2.0,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}
