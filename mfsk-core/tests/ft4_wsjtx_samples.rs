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
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s × 12 kHz

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream("ft4/000000_000002.wav", Some("FT4/000000_000002.wav"))
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
    // `max_cand=100` matches WSJT-X's own `getcandidates4.f90`
    // `MAXCAND=100`: since `engine::ft4_coarse::ft4_coarse_sync`
    // (dapper-soaring-nest plan, Phase 1) emits one candidate per
    // frequency-domain peak instead of the old generic Costas-lag
    // search's up-to-8-per-frequency, this real 6-signal WAV needed
    // only 31 candidates even at the old `max_cand=2000` ceiling — no
    // large safety margin required any more. (The stale comment this
    // replaced described the old search's redundancy-driven budget,
    // now obsolete.)
    let decodes = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(3)
        .decode()
        .results;

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

/// Precision: the decoder must emit **nothing** the reference decoder
/// doesn't.
///
/// FT4 had no false-decode guard of any kind. This file's `GOLDEN`
/// list holds only 6 of the recording's signals, so "extra" had to be
/// measured against something authoritative: a real local
/// `jt9 -5 -p 15 -L 300 -H 2700 -d 3` run over the same WAV finds the
/// 14 below. Against that set this crate emits **11 decodes, all
/// real, zero phantom** — so the budget is 0, and the recall floor
/// states the 3 it does not reach rather than hiding them.
///
/// Written through `common::golden::assert_golden`, which asserts
/// recall and precision together — see its module doc for why that is
/// one call and not two.
#[test]
fn ft4_wsjtx_sample_precision_vs_reference_decoder() {
    use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};

    /// Every decode real `jt9` reports on this recording.
    static REFERENCE: &[GoldenEntry] = &[
        GoldenEntry::msg("AC6BW KR9A R 559 WI"),
        GoldenEntry::msg("CQ RU AB5XS EM12"),
        GoldenEntry::msg("CQ RU N9OY EN43"),
        GoldenEntry::msg("CQ RU W0FRC DM79"),
        GoldenEntry::msg("K1JT WB4HXE 559 GA"),
        GoldenEntry::msg("KB0VHA KA1YQC R 539 MA"),
        GoldenEntry::msg("N1TRK KB7RUQ RR73"),
        GoldenEntry::msg("N1TRK N4FKH 569 VA"),
        GoldenEntry::msg("NI6G W7DRW 569 AZ"),
        GoldenEntry::msg("NZ7P WA7JAY 589 CA"),
        GoldenEntry::msg("VE3LON K7RL R 549 WA"),
        GoldenEntry::msg("W7BOB KJ7G RR73"),
        GoldenEntry::msg("W9JA PY2APK RRR"),
        GoldenEntry::msg("WD9IGY KX1X 73"),
    ];

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
            expected: REFERENCE,
            // 11 of jt9's 14. Raising this is a sensitivity win;
            // lowering it is a regression.
            min_hits: 11,
            max_extra: 0,
        },
        Tolerances::default(),
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: None,
        },
    );
}

/// Reported SNR must track real `jt9`'s on the same recording.
///
/// FT4 was the only implemented protocol with **no SNR check of any
/// kind** — neither against a reference decoder nor against an
/// injected value. That is the gap issue #255 walked into: FT4's
/// reported SNR was ~6.9 dB low for as long as the crate had existed,
/// because `engine::llr::compute_snr_db`'s generic adjacent-tone
/// heuristic stood in for `ft4_decode.f90`'s real
/// `10·log10(candidate_score − 1) − 14.8`, and nothing measured it.
///
/// This pins the fixed formula (`engine::pipeline::ft4_snr_db`).
/// Reference values are real `jt9 -5 -p 15 -L 300 -H 2700 -d 3` on
/// the vendored recording. Measured agreement across the 11 decodes
/// this crate and `jt9` share, spanning −17…+16 dB:
///
/// ```text
///   mean error  +0.06 dB      max |error|  0.46 dB
/// ```
///
/// — the closest SNR agreement of any protocol here. Tolerance is set
/// at 2 dB: loose enough that a refactor or a rebuilt corpus won't
/// trip it, tight enough that the ~6.9 dB error #255 fixed could
/// never pass again.
#[test]
fn ft4_wsjtx_sample_snr_matches_real_jt9() {
    const SNR_TOL_DB: f32 = 2.0;
    /// `(message, jt9's reported SNR)`.
    const REFERENCE: &[(&str, f32)] = &[
        ("N1TRK N4FKH 569 VA", -10.0),
        ("N1TRK KB7RUQ RR73", -10.0),
        ("CQ RU AB5XS EM12", -7.0),
        ("NZ7P WA7JAY 589 CA", -13.0),
        ("KB0VHA KA1YQC R 539 MA", 16.0),
        ("CQ RU N9OY EN43", -4.0),
        ("K1JT WB4HXE 559 GA", -11.0),
        ("VE3LON K7RL R 549 WA", 5.0),
        ("WD9IGY KX1X 73", -1.0),
        ("W7BOB KJ7G RR73", -17.0),
        ("NI6G W7DRW 569 AZ", -7.0),
        ("W9JA PY2APK RRR", -9.0),
        ("AC6BW KR9A R 559 WI", -15.0),
        ("CQ RU W0FRC DM79", -15.0),
    ];

    let Some(path) = sample_path() else {
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let out = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50).decode();

    let mut checked = 0usize;
    let mut bad = Vec::new();
    for d in &out.results {
        let Some(msg) = unpack77(d.message77()) else {
            continue;
        };
        let Some((_, reference)) = REFERENCE.iter().find(|(t, _)| *t == msg) else {
            continue;
        };
        let err = d.snr_db - reference;
        eprintln!(
            "  {msg:<24} ours {:+6.1}  jt9 {reference:+5.1}  err {err:+.1}",
            d.snr_db
        );
        checked += 1;
        if err.abs() > SNR_TOL_DB {
            bad.push(format!(
                "{msg:?}: {:+.1} vs {reference:+.1} (err {err:+.1})",
                d.snr_db
            ));
        }
    }

    assert!(
        checked >= 8,
        "only {checked} decode(s) could be compared against jt9's SNR — recall \
         regressed, so this test could not measure what it exists to measure"
    );
    assert!(
        bad.is_empty(),
        "FT4 reported SNR is more than ±{SNR_TOL_DB} dB from real jt9's on \
         {} decode(s): {bad:#?}. Check `engine::pipeline::ft4_snr_db` and that \
         `SnrCtx::cand_score` still carries `ft4_coarse_sync`'s score.",
        bad.len()
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
    use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};

    static REFERENCE: &[GoldenEntry] = &[
        GoldenEntry::msg("AC6BW KR9A R 559 WI"),
        GoldenEntry::msg("CQ RU AB5XS EM12"),
        GoldenEntry::msg("CQ RU N9OY EN43"),
        GoldenEntry::msg("CQ RU W0FRC DM79"),
        GoldenEntry::msg("K1JT WB4HXE 559 GA"),
        GoldenEntry::msg("KB0VHA KA1YQC R 539 MA"),
        GoldenEntry::msg("N1TRK KB7RUQ RR73"),
        GoldenEntry::msg("N1TRK N4FKH 569 VA"),
        GoldenEntry::msg("NI6G W7DRW 569 AZ"),
        GoldenEntry::msg("NZ7P WA7JAY 589 CA"),
        GoldenEntry::msg("VE3LON K7RL R 549 WA"),
        GoldenEntry::msg("W7BOB KJ7G RR73"),
        GoldenEntry::msg("W9JA PY2APK RRR"),
        GoldenEntry::msg("WD9IGY KX1X 73"),
    ];

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
            expected: REFERENCE,
            // Full parity with real jt9. This is a hard floor: SIC is
            // the whole point of this test, so 13 is a regression.
            min_hits: 14,
            max_extra: 0,
        },
        Tolerances::default(),
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: None,
        },
    );
}
