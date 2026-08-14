//! Q65-30A real-world signal validation against WSJT-X sample
//! recordings.
//!
//! WSJT-X ships off-the-air capture .wav files at
//! `samples/Q65/30A_Ionoscatter_6m/*.wav` (12 kHz mono PCM-16, 30 s
//! each — Joe Taylor's reference dataset). These are real ionoscatter
//! signals on 6 m, captured by K1JT, with all the channel impairments
//! (Doppler, multipath, fading) absent from the synth-only tests.
//!
//! The test is conditionally skipped when the WSJT-X tree is not
//! present at the expected sibling path
//! (`../../WSJT-X/samples/Q65/30A_Ionoscatter_6m/`); developers
//! cloning only `mfsk-core` won't see a failure they can't fix.

#![cfg(feature = "q65")]

use std::path::{Path, PathBuf};

use mfsk_core::fec::qra::FadingModel;
use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{
    DecodeRequest, MultiPeriodRequest, Q65a30, Q65a60, Q65a300, Q65b60, Q65d60, Q65d120, Q65e120,
};

#[allow(dead_code)]
mod common;

use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};
use common::load_wav_f32_opt as read_wsjtx_wav;

/// [`Q65Result`](mfsk_core::q65::Q65Result) → [`DecodeView`], shared by
/// every real-signal test below.
fn q65_view(d: &mfsk_core::q65::Q65Result) -> DecodeView {
    DecodeView {
        msg: d.message.clone(),
        freq_hz: d.freq_hz,
        dt_sec: d.dt_sec,
        snr_db: Some(d.snr_db),
    }
}

fn samples_dir(rel: &str) -> Option<PathBuf> {
    // Tests run from `mfsk-core/mfsk-core/`; the WSJT-X tree is at
    // `mfsk-core/../WSJT-X/`. Use `CARGO_MANIFEST_DIR` so the lookup
    // is independent of the caller's working directory.
    // Only 60A_EME_6m and 60D_EME_10GHz are vendored (the full Q65
    // sample tree is ~25 MB); the rest still resolve from an upstream
    // checkout when one is present. `optional_corpus`-style semantics
    // deliberately: a sub-mode that is not vendored must not fail CI.
    let vendored = common::corpus::golden_dir().join("q65").join(rel);
    if vendored.is_dir() {
        return Some(vendored);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let dir = Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65")
        .join(rel)
        .canonicalize()
        .ok()?;
    if dir.is_dir() { Some(dir) } else { None }
}

/// Strict gate: stack the four ionoscatter recordings into one
/// running EMA via [`mfsk_core::q65::MultiPeriodRequest`] and require at least
/// one decode total.
///
/// Mirrors WSJT-X's `iavg=1`/`iavg=2` averaged-decode flow from
/// `lib/q65_decode.f90` — the path that lets weak ionoscatter
/// signals decode when single-period BP/fading cannot. Our reduced
/// b90 sweep `{3, 8, 15} × {Gaussian, Lorentzian}` plus plain Bessel
/// fallback covers the realistic ionoscatter spread regime.
///
/// Currently no AP-list pair is supplied (the recordings are from
/// an unknown station so we don't know plausible call/grid pairs);
/// the fading + plain BP ladder must reach the threshold on its own.
/// If the expected call/grid pair becomes known, pass it through
/// [`mfsk_core::q65::standard_qso_codewords`] for the AP-list path.
#[test]
fn ionoscatter_6m_full_stack_decodes_via_averaging() {
    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!("skipping: WSJT-X sample tree not found");
        return;
    };
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    // Stable order so the EMA sees slots in chronological order.
    paths.sort();
    assert!(
        !paths.is_empty(),
        "WSJT-X Q65-30A sample dir contains no .wav files"
    );

    let audios: Vec<Vec<f32>> = paths.iter().filter_map(read_wsjtx_wav).collect();
    assert!(
        !audios.is_empty(),
        "no readable WAVs in WSJT-X Q65-30A ionoscatter sample dir"
    );
    let slot_refs: Vec<&[f32]> = audios.iter().map(|v| v.as_slice()).collect();

    // 15 s into the 30 s slot, ±15 s tolerance — covers the full
    // recording so the running-EMA coarse search has the whole slot.
    let nominal_mid = 12_000 * 15;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 15.0,
        time_tolerance_late_sec: 15.0,
        score_threshold: 0.05,
        max_candidates: 8,
    };

    let decodes_no_ap =
        MultiPeriodRequest::<Q65a30>::new(&slot_refs, 12_000, nominal_mid, params).decode();
    eprintln!(
        "[info] ionoscatter multi-period (no AP-list): {} unique decode(s) across {} slot(s)",
        decodes_no_ap.len(),
        slot_refs.len(),
    );
    for d in &decodes_no_ap {
        eprintln!(
            "  → freq={:.1} Hz dt={:.2} s iter={} snr={:.1} dB : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.iterations,
            d.snr_db,
            d.message
        );
    }

    // Try the AP-list path with K1JT / K9AN — the call pair revealed
    // by the no-AP run above. WSJT-X normally builds this list from
    // the user's "Watch list" or last-seen-CQ; here we hard-code the
    // discovered pair to test the AP-list integration.
    use mfsk_core::q65::standard_qso_codewords;
    let ap_codewords = standard_qso_codewords("K1JT", "K9AN", "");
    let decodes_ap = MultiPeriodRequest::<Q65a30>::new(&slot_refs, 12_000, nominal_mid, params)
        .ap_list(&ap_codewords)
        .decode();
    eprintln!(
        "[info] ionoscatter multi-period (AP-list K1JT/K9AN): {} unique decode(s)",
        decodes_ap.len(),
    );
    for d in &decodes_ap {
        eprintln!(
            "  → freq={:.1} Hz dt={:.2} s iter={} snr={:.1} dB : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.iterations,
            d.snr_db,
            d.message
        );
    }

    // Golden: "K1JT K9AN R-16" @ 1010 Hz — both the no-AP and AP-list
    // paths agree on it (see the `[info]` prints above). No independent
    // `jt9` cross-check is possible here: `jt9`'s CLI has no flag to
    // drive the `iavg` multi-period-averaging state real WSJT-X uses
    // for this recording (verified experimentally — feeding it several
    // files in one invocation just runs independent single-slot
    // attempts), so this is the crate's own two independently-computed
    // paths agreeing with each other, not agreement with a reference
    // decoder. `dt_sec`/`snr_db` are left unchecked for the same
    // reason: the no-AP and AP-list runs differ slightly on `snr_db`
    // (-20.0 vs -19.4) with nothing external to say which is "right".
    //
    // Precision matters here as much as recall: both runs' *only*
    // output is this one message, so `max_extra: 0` is not a guess.
    assert_golden(
        &[decodes_no_ap, decodes_ap].concat(),
        &GoldenSet {
            name: "Q65-30A ionoscatter (30A_Ionoscatter_6m)",
            expected: &[GoldenEntry {
                msg: "K1JT K9AN R-16",
                freq_hz: Some(1010.0),
                dt_sec: None,
                snr_db: None,
            }],
            min_hits: 1,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: 5.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

#[test]
fn eme_6m_sample_yields_decode_with_ap() {
    // Q65-60A 6 m EME recording. With the AP path active we
    // should be able to recover at least one of the typical
    // call/CQ patterns even from this real-world weak signal.
    let Some(dir) = samples_dir("60A_EME_6m") else {
        eprintln!("skipping: WSJT-X 6m EME sample tree not found");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    assert!(
        !entries.is_empty(),
        "WSJT-X 6m EME sample dir contains no .wav files"
    );

    // The 60A frame can begin anywhere in a 60 s slot — wide tol.
    let nominal_mid = 12_000 * 30; // 30 s into the 60 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 16,
    };

    // The 210106_1621.wav recording captures W7GJ working multiple
    // stations on 6 m EME (W7GJ is a well-known prolific 6 m EME
    // operator). Try the empty hint plus a hint locking call1 =
    // W7GJ (matching the actual exchange).
    let hints = [
        ("plain", ApHint::new()),
        ("W7GJ ??", ApHint::new().with_call1("W7GJ")),
    ];

    let mut all_decodes: Vec<mfsk_core::q65::Q65Result> = Vec::new();
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        for (label, hint) in &hints {
            let mut req = DecodeRequest::<Q65a60>::new(&audio, 12_000, nominal_mid, params);
            if hint.has_info() {
                req = req.ap_hint(hint);
            }
            let decodes = req.decode();
            let names: Vec<String> = decodes.iter().map(|d| d.message.clone()).collect();
            println!(
                "{} [{label}]: {} decode(s) → {names:?}",
                path.file_name().unwrap().to_string_lossy(),
                decodes.len()
            );
            all_decodes.extend(decodes);
        }
    }

    // 6 m EME has the lowest Doppler spread in the EME band lineup, so
    // the AWGN-only metric already does a respectable job on
    // strong-ish signals — 210106_1621.wav captures W7GJ (a
    // well-known prolific 6 m EME operator) working four other
    // stations. Golden set independently verified against a locally
    // built `jt9 -3 -p 60 -b A -d 3` (2026-08-14): depth 1 (this
    // crate's own default effort) only reaches 3 of the 4 at the CLI's
    // default settings, depth 3 reaches all 4, matching this crate's
    // frequencies to within 1 Hz and SNRs to within 0.3 dB — so all
    // four are real, not phantoms, and the golden set uses `jt9`'s own
    // reported freq/SNR as the reference. The plain and AP-hint passes
    // land on the identical four messages (AP-list adds nothing new
    // for this recording), so both are folded into one assertion
    // rather than two separate weaker ones.
    assert_golden(
        &all_decodes,
        &GoldenSet {
            name: "Q65-60A EME 6m (210106_1621.wav)",
            expected: &[
                GoldenEntry {
                    msg: "W7GJ N8JX EN73",
                    freq_hz: Some(697.0),
                    dt_sec: None,
                    snr_db: Some(-24.0),
                },
                GoldenEntry {
                    msg: "W7GJ N0TB -15",
                    freq_hz: Some(943.0),
                    dt_sec: None,
                    snr_db: Some(-24.0),
                },
                GoldenEntry {
                    msg: "W7GJ W1VD FN31",
                    freq_hz: Some(1420.0),
                    dt_sec: None,
                    snr_db: Some(-20.0),
                },
                GoldenEntry {
                    msg: "W7GJ VE1JF RRR",
                    freq_hz: Some(1620.0),
                    dt_sec: None,
                    snr_db: Some(-19.0),
                },
            ],
            min_hits: 4,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: 4.0,
            snr_db: 3.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

/// Q65-60D 10 GHz EME sample: `WSJT-X/samples/Q65/60D_EME_10GHz/201212_1838.wav`.
///
/// WSJT-X UnitTests.txt golden: "VK7MO K6QPV DM12" / "VK7MO K6QPV -15"
/// near 1000 Hz. At -25 to -30 dB SNR the AWGN-only BP metric fails;
/// the fast-fading metric (b90 = 10 Hz, Gaussian) is required.
///
/// Verified 2026-05-06: fading path decodes cleanly (plain = 0 decodes,
/// which matches WSJT-X's "No_AP" count ≤ 9/14 on the full dataset).
#[test]
fn eme_10ghz_60d_decodes_with_fading_metric() {
    let path = common::corpus::golden_path_or_upstream(
        "q65/60D_EME_10GHz/201212_1838.wav",
        Some("Q65/60D_EME_10GHz/201212_1838.wav"),
    );
    let path = match path {
        Some(p) => p,
        None => {
            eprintln!(
                "skipping: Q65 60D_EME_10GHz sample not found under {}",
                common::corpus::golden_dir().display()
            );
            return;
        }
    };

    let audio = match read_wsjtx_wav(&path) {
        Some(a) => a,
        None => {
            eprintln!("skipping: WAV format not recognised");
            return;
        }
    };

    // Signal is at ~1000 Hz; slot is 60 s.
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 6.0,
        time_tolerance_late_sec: 6.0,
        score_threshold: 0.05,
        max_candidates: 8,
    };

    // Plain BP is known to fail at this SNR (see doc comment above) —
    // not re-verified here since it's diagnostic-only and, with this
    // sub-mode's fine-timing retry, was measured costing seconds of
    // wall time for zero assertion coverage.

    // Fast-fading Gaussian, b90 = 10 Hz: the path that actually works.
    let fading = DecodeRequest::<Q65d60>::new(&audio, 12_000, 0, params)
        .fading(FadingModel::Gaussian, 10.0)
        .decode();
    eprintln!(
        "[info] 10 GHz EME fading b90=10 Gaussian: {} decode(s)",
        fading.len()
    );
    for d in &fading {
        eprintln!(
            "  freq={:.1} Hz dt={:.2} s : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.message
        );
    }

    // Golden "VK7MO K6QPV DM12" @ ~995 Hz, jt9 SNR -15 dB — same
    // reference `q65_snr_matches_jt9_ground_truth` uses. `max_extra: 0`:
    // a real `jt9 -3 -p 60 -b D -d 3` run on this file (2026-08-14)
    // also reports exactly one decode.
    assert_golden(
        &fading,
        &GoldenSet {
            name: "Q65-60D EME 10GHz (201212_1838.wav)",
            expected: &[GoldenEntry {
                msg: "VK7MO K6QPV DM12",
                freq_hz: Some(1000.0),
                dt_sec: None,
                snr_db: Some(-15.0),
            }],
            min_hits: 1,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: 20.0,
            snr_db: 3.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

/// Q65-60B 1296 MHz troposcatter recordings:
/// `WSJT-X/samples/Q65/60B_1296_Troposcatter/*.wav` (3 of the 75 files
/// in WSJT-X's own full test corpus — `WSJT-X/UnitTests.txt`'s
/// "Q65-60B / 60B_1296_Troposcatter" entry documents the golden
/// message).
///
/// WSJT-X UnitTests.txt golden: "VK7MO VK7PD QE38" near 1000 Hz.
///
/// Single-slot plain/AP-hint decode fails on all 3 files (this
/// dataset sits below that threshold, same as
/// `eme_10ghz_60d_decodes_with_fading_metric`'s plain path) — the
/// multi-period EMA-on-spectrogram path
/// (`MultiPeriodRequest`, same mechanism
/// `ionoscatter_6m_full_stack_decodes_via_averaging` uses) recovers
/// it cleanly, both with and without the AP-list.
///
/// Added 2026-07-19: this recording already existed in the vendored
/// WSJT-X samples tree but had no corresponding test — Q65-60B was
/// the only one of the six wired sub-modes with a real off-air
/// recording available and not yet exercised.
#[test]
fn tropo_1296_60b_decodes_via_averaging() {
    let Some(dir) = samples_dir("60B_1296_Troposcatter") else {
        eprintln!("skipping: WSJT-X 60B_1296_Troposcatter sample tree not found");
        return;
    };
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    paths.sort();
    assert!(
        !paths.is_empty(),
        "WSJT-X 60B_1296_Troposcatter sample dir contains no .wav files"
    );

    let audios: Vec<Vec<f32>> = paths.iter().filter_map(read_wsjtx_wav).collect();
    assert!(
        !audios.is_empty(),
        "no readable WAVs in WSJT-X 60B_1296_Troposcatter sample dir"
    );
    let slot_refs: Vec<&[f32]> = audios.iter().map(|v| v.as_slice()).collect();

    let nominal_mid = 12_000 * 30; // 30 s into the 60 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 8,
    };

    let decodes_no_ap =
        MultiPeriodRequest::<Q65b60>::new(&slot_refs, 12_000, nominal_mid, params).decode();
    eprintln!(
        "[info] 1296 troposcatter multi-period (no AP-list): {} unique decode(s) across {} slot(s)",
        decodes_no_ap.len(),
        slot_refs.len(),
    );
    for d in &decodes_no_ap {
        eprintln!(
            "  → freq={:.1} Hz snr={:.1} dB : {}",
            d.freq_hz, d.snr_db, d.message
        );
    }

    use mfsk_core::q65::standard_qso_codewords;
    let ap_codewords = standard_qso_codewords("VK7MO", "VK7PD", "");
    let decodes_ap = MultiPeriodRequest::<Q65b60>::new(&slot_refs, 12_000, nominal_mid, params)
        .ap_list(&ap_codewords)
        .decode();
    eprintln!(
        "[info] 1296 troposcatter multi-period (AP-list VK7MO/VK7PD): {} unique decode(s)",
        decodes_ap.len(),
    );
    for d in &decodes_ap {
        eprintln!(
            "  → freq={:.1} Hz snr={:.1} dB : {}",
            d.freq_hz, d.snr_db, d.message
        );
    }

    // Golden "VK7MO VK7PD QE38" @ ~1002 Hz — both no-AP and AP-list
    // agree (see the `[info]` prints above). No independent `jt9`
    // cross-check is possible here, same `iavg` CLI limitation as
    // `ionoscatter_6m_full_stack_decodes_via_averaging`; this is the
    // crate's own two paths agreeing with each other. Both runs'
    // *only* output is this one message, so `max_extra: 0` reflects
    // what was actually measured, not a guess.
    assert_golden(
        &[decodes_no_ap, decodes_ap].concat(),
        &GoldenSet {
            name: "Q65-60B 1296 troposcatter (60B_1296_Troposcatter)",
            expected: &[GoldenEntry {
                msg: "VK7MO VK7PD QE38",
                freq_hz: Some(1002.0),
                dt_sec: None,
                snr_db: None,
            }],
            min_hits: 1,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: 5.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

/// Q65-120D 10 GHz rainscatter/troposcatter recording:
/// `WSJT-X/samples/Q65/120D_Rainscatter_10_GHz/210117_0920.wav`.
///
/// Golden (confirmed via `jt9 -3 -p 120 -b D`): "VK3WE VK7MO QE37"
/// near 995 Hz. Plain BP fails (same shape as the 60D/300A cases
/// below); the fast-fading metric (b90 ≥ 20 Hz, Gaussian or
/// Lorentzian) recovers it.
#[test]
fn rainscatter_10ghz_120d_decodes_with_fading_metric() {
    let Some(dir) = samples_dir("120D_Rainscatter_10_GHz") else {
        eprintln!("skipping: WSJT-X 120D_Rainscatter_10_GHz sample tree not found");
        return;
    };
    let path = dir.join("210117_0920.wav");
    let Some(audio) = read_wsjtx_wav(&path) else {
        eprintln!("skipping: WAV not found/readable at {}", path.display());
        return;
    };

    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 13.333,
        time_tolerance_late_sec: 13.333,
        score_threshold: 0.0,
        max_candidates: 8,
    };

    // Plain BP is known to fail at this SNR (see doc comment above) —
    // not re-verified here; diagnostic-only and not worth the wall time.

    let fading = DecodeRequest::<Q65d120>::new(&audio, 12_000, 0, params)
        .fading(FadingModel::Gaussian, 20.0)
        .decode();
    eprintln!(
        "[info] 10 GHz rainscatter fading b90=20 Gaussian: {} decode(s)",
        fading.len()
    );
    for d in &fading {
        eprintln!("  freq={:.1} Hz : {}", d.freq_hz, d.message);
    }

    // Golden "VK3WE VK7MO QE37" @ 995 Hz, jt9 SNR -16 dB.
    //
    // `max_extra: 1`, not the usual 0 — **tracked debt, not accepted
    // design.** A real local `jt9 -3 -p 120 -b D -d 3` run on this file
    // (2026-08-14) reports exactly *one* decode at 995 Hz; this crate
    // reports the identical message twice, at two nearby frequencies
    // (~994 Hz and ~1001 Hz, ~7 Hz apart). Root cause: `q65` has no
    // cross-candidate dedup at all (unlike FT8/FT4/FST4, which collapse
    // re-derivations of the same message via SIC-subtraction or a
    // `known`-list filter) — two coarse candidates a few Hz apart both
    // independently lock onto and decode the same real signal, and
    // nothing downstream notices they agree. Filed as issue #287
    // rather than fixed here or silently accepted: this test's job is
    // to make the gap visible, not to design the fix.
    assert_golden(
        &fading,
        &GoldenSet {
            name: "Q65-120D 10GHz rainscatter (210117_0920.wav)",
            expected: &[GoldenEntry {
                msg: "VK3WE VK7MO QE37",
                freq_hz: Some(995.0),
                dt_sec: None,
                snr_db: Some(-16.0),
            }],
            min_hits: 1,
            max_extra: 1,
        },
        Tolerances {
            freq_hz: 15.0,
            snr_db: 3.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

/// Q65-120E 6 m ionoscatter recordings:
/// `WSJT-X/samples/Q65/120E_Ionoscatter_6m/*.wav` (2 files).
///
/// Golden (confirmed via `jt9 -3 -p 120 -b E`): "KB7IJ N0AN 73" near
/// 1799 Hz, recovered from `210130_1442.wav` only — `210130_1438.wav`
/// doesn't decode even via WSJT-X's own reference, so this test
/// doesn't require a hit from every file in the directory, only that
/// at least one recovers the golden message (mirrors how
/// `tests/q65_wsjtx_samples.rs`'s other multi-file tests handle
/// per-file variance).
#[test]
fn ionoscatter_6m_120e_decodes_with_fading_metric() {
    let Some(dir) = samples_dir("120E_Ionoscatter_6m") else {
        eprintln!("skipping: WSJT-X 120E_Ionoscatter_6m sample tree not found");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    assert!(
        !entries.is_empty(),
        "WSJT-X 120E_Ionoscatter_6m sample dir contains no .wav files"
    );

    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 13.333,
        time_tolerance_late_sec: 13.333,
        score_threshold: 0.0,
        max_candidates: 8,
    };

    let mut all_decodes: Vec<mfsk_core::q65::Q65Result> = Vec::new();
    for path in &entries {
        let Some(audio) = read_wsjtx_wav(path) else {
            continue;
        };
        let fading = DecodeRequest::<Q65e120>::new(&audio, 12_000, 0, params)
            .fading(FadingModel::Gaussian, 12.0)
            .decode();
        eprintln!(
            "{}: {} fading decode(s)",
            path.file_name().unwrap().to_string_lossy(),
            fading.len()
        );
        for d in &fading {
            eprintln!("  freq={:.1} Hz : {}", d.freq_hz, d.message);
        }
        all_decodes.extend(fading);
    }

    // Golden "KB7IJ N0AN 73" @ 1799 Hz, jt9 SNR -17 dB, confirmed via a
    // real local `jt9 -3 -p 120 -b E -d 3` run on 210130_1442.wav
    // (2026-08-14): exactly one decode, freq 1799 Hz. `210130_1438.wav`
    // contributes nothing (doesn't decode even via WSJT-X's own
    // reference), so `min_hits: 1` / `max_extra: 0` cover both files
    // together, not per-file.
    assert_golden(
        &all_decodes,
        &GoldenSet {
            name: "Q65-120E 6m ionoscatter (120E_Ionoscatter_6m)",
            expected: &[GoldenEntry {
                msg: "KB7IJ N0AN 73",
                freq_hz: Some(1799.0),
                dt_sec: None,
                snr_db: Some(-17.0),
            }],
            min_hits: 1,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: 15.0,
            snr_db: 3.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

/// Q65-300A optical (laser) scatter recording:
/// `WSJT-X/samples/Q65/300A_Optical_Scatter/201210_0505.wav`.
///
/// Golden (confirmed via `jt9 -3 -p 300 -b A`): "VK7MO VK7PD QE38"
/// near 1002 Hz at SNR ≈ -34 dB — right at Q65-300A's published
/// ~-33.8 dB AWGN threshold, the deepest of any wired Q65 sub-mode.
/// Plain BP fails; the fast-fading metric (b90 = 2-20 Hz, either
/// model) recovers it even though this is a stable-path scatter
/// signal, not classic Doppler-spread EME — the fading metric's
/// robustness appears to help at threshold-adjacent SNR generally,
/// not just under true multipath.
#[test]
fn optical_scatter_300a_decodes_with_fading_metric() {
    let Some(dir) = samples_dir("300A_Optical_Scatter") else {
        eprintln!("skipping: WSJT-X 300A_Optical_Scatter sample tree not found");
        return;
    };
    let path = dir.join("201210_0505.wav");
    let Some(audio) = read_wsjtx_wav(&path) else {
        eprintln!("skipping: WAV not found/readable at {}", path.display());
        return;
    };

    // 300 s slot is long; widen the timing search to cover the whole
    // recording rather than just the first few seconds.
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 207.36,
        time_tolerance_late_sec: 207.36,
        score_threshold: 0.0,
        max_candidates: 20,
    };

    // Plain BP is known to fail at this SNR (see doc comment above) —
    // not re-verified here. With these wide search params (needed to
    // locate an unknown offset in the 293.8 s recording) plus the
    // fine-timing retry, a live run of this diagnostic-only scan was
    // measured costing ~8.5 s of the previous ~9.6 s test wall time
    // for zero assertion coverage (2026-07-20 profiling).

    let fading = DecodeRequest::<Q65a300>::new(&audio, 12_000, 0, params)
        .fading(FadingModel::Gaussian, 5.0)
        .decode();
    eprintln!(
        "[info] optical scatter fading b90=5 Gaussian: {} decode(s)",
        fading.len()
    );
    for d in &fading {
        eprintln!("  freq={:.1} Hz : {}", d.freq_hz, d.message);
    }

    // Golden "VK7MO VK7PD QE38" @ 1002 Hz, jt9 SNR -34 dB — confirmed
    // via a real local `jt9 -3 -p 300 -b A -d 3` run (2026-08-14):
    // exactly one decode, freq 1002 Hz, SNR -34 dB, matching this
    // crate's own output almost exactly.
    assert_golden(
        &fading,
        &GoldenSet {
            name: "Q65-300A optical scatter (201210_0505.wav)",
            expected: &[GoldenEntry {
                msg: "VK7MO VK7PD QE38",
                freq_hz: Some(1002.0),
                dt_sec: None,
                snr_db: Some(-34.0),
            }],
            min_hits: 1,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: 15.0,
            snr_db: 3.0,
            ..Tolerances::default()
        },
        q65_view,
    );
}

/// SNR ground truth for issue #255's Q65 real-formula port
/// (`q65::snr::q65_snr_db`): a real local `jt9 -3 -d3` build's own
/// displayed SNR for four real off-air decodes, one per sub-mode
/// (`SNRAUDIT`-style verification done for this investigation, no
/// Fortran instrumentation needed this time — `jt9`'s own CLI output
/// already prints the SNR column). `±3.0` dB tolerance — generous
/// relative to the `~1` dB gap the investigation actually landed on,
/// since this is a regression gate, not a precision claim.
#[test]
fn q65_snr_matches_jt9_ground_truth() {
    const SNR_TOL_DB: f32 = 3.0;

    // Q65-60D: WSJT-X/samples/Q65/60D_EME_10GHz/201212_1838.wav,
    // "VK7MO K6QPV DM12" — jt9: -15 dB.
    if let Some(dir) = samples_dir("60D_EME_10GHz") {
        let path = dir.join("201212_1838.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            let params = SearchParams {
                freq_min_hz: 200.0,
                freq_max_hz: 3_000.0,
                time_tolerance_early_sec: 6.0,
                time_tolerance_late_sec: 6.0,
                score_threshold: 0.05,
                max_candidates: 8,
            };
            let fading = DecodeRequest::<Q65d60>::new(&audio, 12_000, 0, params)
                .fading(FadingModel::Gaussian, 10.0)
                .decode();
            let hit = fading
                .iter()
                .find(|d| d.message.contains("VK7MO") && d.message.contains("K6QPV"));
            match hit {
                Some(d) => assert_snr_close("Q65-60D VK7MO/K6QPV", d.snr_db, -15.0, SNR_TOL_DB),
                None => panic!(
                    "Q65-60D VK7MO/K6QPV not decoded — see eme_10ghz_60d_decodes_with_fading_metric"
                ),
            }
        }
    }

    // Q65-120D: WSJT-X/samples/Q65/120D_Rainscatter_10_GHz/210117_0920.wav,
    // "VK3WE VK7MO QE37" — jt9: -16 dB.
    if let Some(dir) = samples_dir("120D_Rainscatter_10_GHz") {
        let path = dir.join("210117_0920.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            let params = SearchParams {
                freq_min_hz: 200.0,
                freq_max_hz: 3_000.0,
                time_tolerance_early_sec: 13.333,
                time_tolerance_late_sec: 13.333,
                score_threshold: 0.0,
                max_candidates: 8,
            };
            let fading = DecodeRequest::<Q65d120>::new(&audio, 12_000, 0, params)
                .fading(FadingModel::Gaussian, 20.0)
                .decode();
            let hit = fading
                .iter()
                .find(|d| d.message.contains("VK3WE") && d.message.contains("VK7MO"));
            match hit {
                Some(d) => assert_snr_close("Q65-120D VK3WE/VK7MO", d.snr_db, -16.0, SNR_TOL_DB),
                None => panic!(
                    "Q65-120D VK3WE/VK7MO not decoded — see rainscatter_10ghz_120d_decodes_with_fading_metric"
                ),
            }
        }
    }

    // Q65-120E: WSJT-X/samples/Q65/120E_Ionoscatter_6m/210130_1442.wav,
    // "KB7IJ N0AN 73" — jt9: -17 dB.
    if let Some(dir) = samples_dir("120E_Ionoscatter_6m") {
        let path = dir.join("210130_1442.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            let params = SearchParams {
                freq_min_hz: 200.0,
                freq_max_hz: 3_000.0,
                time_tolerance_early_sec: 13.333,
                time_tolerance_late_sec: 13.333,
                score_threshold: 0.0,
                max_candidates: 8,
            };
            let fading = DecodeRequest::<Q65e120>::new(&audio, 12_000, 0, params)
                .fading(FadingModel::Gaussian, 12.0)
                .decode();
            let hit = fading
                .iter()
                .find(|d| d.message.contains("KB7IJ") && d.message.contains("N0AN"));
            match hit {
                Some(d) => assert_snr_close("Q65-120E KB7IJ/N0AN", d.snr_db, -17.0, SNR_TOL_DB),
                None => panic!(
                    "Q65-120E KB7IJ/N0AN not decoded — see ionoscatter_6m_120e_decodes_with_fading_metric"
                ),
            }
        }
    }

    // Q65-300A: WSJT-X/samples/Q65/300A_Optical_Scatter/201210_0505.wav,
    // "VK7MO VK7PD QE38" — jt9: -34 dB.
    if let Some(dir) = samples_dir("300A_Optical_Scatter") {
        let path = dir.join("201210_0505.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            let params = SearchParams {
                freq_min_hz: 200.0,
                freq_max_hz: 3_000.0,
                time_tolerance_early_sec: 207.36,
                time_tolerance_late_sec: 207.36,
                score_threshold: 0.0,
                max_candidates: 20,
            };
            let fading = DecodeRequest::<Q65a300>::new(&audio, 12_000, 0, params)
                .fading(FadingModel::Gaussian, 5.0)
                .decode();
            let hit = fading
                .iter()
                .find(|d| d.message.contains("VK7MO") && d.message.contains("VK7PD"));
            match hit {
                Some(d) => assert_snr_close("Q65-300A VK7MO/VK7PD", d.snr_db, -34.0, SNR_TOL_DB),
                None => panic!(
                    "Q65-300A VK7MO/VK7PD not decoded — see optical_scatter_300a_decodes_with_fading_metric"
                ),
            }
        }
    }
}

fn assert_snr_close(label: &str, ours: f32, jt9: f32, tol_db: f32) {
    eprintln!(
        "{label}: ours={ours:.2} dB, jt9={jt9:.2} dB, diff={:.2} dB",
        (ours - jt9).abs()
    );
    assert!(
        (ours - jt9).abs() <= tol_db,
        "{label} SNR diverged from jt9 ground truth by {:.2} dB (ours={:.2}, jt9={:.2}) — \
         beyond the {tol_db} dB regression gate",
        (ours - jt9).abs(),
        ours,
        jt9,
    );
}

/// Diagnostic (new investigation, mirrors the FT4/FST4 speed
/// methodology): `BENCHMARKS.md`'s "Decode speed" table has Q65-60A
/// (1.57 s), Q65-60B (1.89 s), Q65-30A (2.55 s) far slower than
/// Q65-120D (0.15 s) / Q65-60D (0.39 s) despite similar or shorter
/// audio — counter-intuitive since 120D/60D use the (presumably more
/// expensive per-candidate) fast-fading metric while 60A uses plain
/// BP. Measures `coarse_search_for`'s candidate count/time separately
/// from the full `DecodeRequest::decode`/`.ap_hint(..).decode()` wall-clock,
/// to see whether the gap is coarse-search grid size (Q65-60A's
/// `SearchParams::time_tolerance_sec=30.0` vs Q65-60D's `6.0` — a 5×
/// larger coarse time-search window, `coarse_search_on_spec_for`,
/// `q65/search.rs:202-236`, O(time_span × freq_span)) or the
/// per-candidate fine-timing retry (`decode_at_with_fine_timing_for`,
/// up to 7 decode attempts per candidate). No assertions.
#[test]
#[ignore = "manual diagnostic — Q65-60A/60B/30A speed investigation (new investigation)"]
fn q65_speed_diag_coarse_vs_finetiming() {
    use std::time::Instant;

    use mfsk_core::q65::search::coarse_search_for;

    fn measure_plain<P: mfsk_core::q65::Q65SubMode>(
        label: &str,
        audio: &[f32],
        nominal_start: usize,
        params: &SearchParams,
    ) {
        let t0 = Instant::now();
        let cands = coarse_search_for::<P>(audio, 12_000, nominal_start, params);
        let coarse_dt = t0.elapsed();

        let t0 = Instant::now();
        let decodes = DecodeRequest::<P>::new(audio, 12_000, nominal_start, *params).decode();
        let plain_dt = t0.elapsed();

        eprintln!(
            "{label}: {} candidates (tol_early={} s, max_cand={}), coarse={:.1}ms, \
             DecodeRequest::decode(plain)={:.1}ms, {} decode(s)",
            cands.len(),
            params.time_tolerance_early_sec,
            params.max_candidates,
            coarse_dt.as_secs_f64() * 1000.0,
            plain_dt.as_secs_f64() * 1000.0,
            decodes.len()
        );
    }

    if let Some(dir) = samples_dir("60A_EME_6m") {
        if let Some(entry) = std::fs::read_dir(&dir).ok().and_then(|it| {
            it.filter_map(|e| e.ok())
                .find(|e| e.path().extension().and_then(|s| s.to_str()) == Some("wav"))
        }) && let Some(audio) = read_wsjtx_wav(entry.path())
        {
            let params = SearchParams {
                freq_min_hz: 200.0,
                freq_max_hz: 3_000.0,
                time_tolerance_early_sec: 30.0,
                time_tolerance_late_sec: 30.0,
                score_threshold: 0.05,
                max_candidates: 16,
            };
            measure_plain::<Q65a60>("Q65-60A (real params)", &audio, 12_000 * 30, &params);

            // Same audio, Q65-60D's much narrower time-tolerance,
            // to isolate the coarse-search-window effect alone.
            let params_narrow = SearchParams {
                time_tolerance_early_sec: 6.0,
                time_tolerance_late_sec: 6.0,
                max_candidates: 100,
                ..params
            };
            measure_plain::<Q65a60>(
                "Q65-60A (60D-style narrow tolerance)",
                &audio,
                12_000 * 30,
                &params_narrow,
            );
        }
    } else {
        eprintln!("skipping Q65-60A: sample dir not found");
    }

    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok().unwrap_or_default();
    let path = std::path::Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65/60D_EME_10GHz/201212_1838.wav");
    if let Ok(path) = path.canonicalize() {
        if let Some(audio) = read_wsjtx_wav(&path) {
            let params = SearchParams {
                freq_min_hz: 200.0,
                freq_max_hz: 3_000.0,
                time_tolerance_early_sec: 6.0,
                time_tolerance_late_sec: 6.0,
                score_threshold: 0.05,
                max_candidates: 100,
            };
            let t0 = Instant::now();
            let cands = coarse_search_for::<Q65d60>(&audio, 12_000, 0, &params);
            let coarse_dt = t0.elapsed();
            eprintln!(
                "Q65-60D (real params): {} candidates, coarse={:.1}ms",
                cands.len(),
                coarse_dt.as_secs_f64() * 1000.0
            );
            let t0 = Instant::now();
            let fading = DecodeRequest::<Q65d60>::new(&audio, 12_000, 0, params)
                .fading(FadingModel::Gaussian, 10.0)
                .decode();
            let fading_dt = t0.elapsed();
            eprintln!(
                "Q65-60D DecodeRequest::fading decode = {:.1}ms, {} decode(s)",
                fading_dt.as_secs_f64() * 1000.0,
                fading.len()
            );
        }
    } else {
        eprintln!("skipping Q65-60D: sample not found");
    }
}

/// Timing-only diagnostic for the `MultiPeriodRequest` redundant-
/// extraction fix (Q65-60B/30A speed investigation, new investigation).
/// Search params match the real golden tests exactly
/// (`tropo_1296_60b_decodes_via_averaging` /
/// `ionoscatter_6m_full_stack_decodes_via_averaging`).
#[test]
#[ignore = "manual diagnostic — Q65-60B/30A MultiPeriodRequest timing (new investigation)"]
fn q65_multi_period_speed_diag() {
    use std::time::Instant;

    if let Some(dir) = samples_dir("60B_1296_Troposcatter") {
        let mut entries: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
            .collect();
        entries.sort();
        let slots: Vec<Vec<f32>> = entries.iter().filter_map(read_wsjtx_wav).collect();
        let slot_refs: Vec<&[f32]> = slots.iter().map(|v| v.as_slice()).collect();
        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: 30.0,
            time_tolerance_late_sec: 30.0,
            score_threshold: 0.05,
            max_candidates: 8,
        };
        let t0 = Instant::now();
        let decodes =
            MultiPeriodRequest::<Q65b60>::new(&slot_refs, 12_000, 12_000 * 30, params).decode();
        let dt = t0.elapsed();
        eprintln!(
            "Q65-60B MultiPeriodRequest::decode = {:.1}ms, {} slot(s), {} decode(s)",
            dt.as_secs_f64() * 1000.0,
            slot_refs.len(),
            decodes.len()
        );
    }

    if let Some(dir) = samples_dir("30A_Ionoscatter_6m") {
        let mut entries: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
            .collect();
        entries.sort();
        let slots: Vec<Vec<f32>> = entries.iter().filter_map(read_wsjtx_wav).collect();
        let slot_refs: Vec<&[f32]> = slots.iter().map(|v| v.as_slice()).collect();
        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: 15.0,
            time_tolerance_late_sec: 15.0,
            score_threshold: 0.05,
            max_candidates: 8,
        };
        let t0 = Instant::now();
        let decodes =
            MultiPeriodRequest::<Q65a30>::new(&slot_refs, 12_000, 12_000 * 15, params).decode();
        let dt = t0.elapsed();
        eprintln!(
            "Q65-30A MultiPeriodRequest::decode = {:.1}ms, {} slot(s), {} decode(s)",
            dt.as_secs_f64() * 1000.0,
            slot_refs.len(),
            decodes.len()
        );
    }
}

/// Candidate-count diagnostic: how many coarse candidates does each
/// slot of the Q65-30A ionoscatter multi-period scan actually
/// produce? `MultiPeriodRequest::decode` pays its 8-stage decode ladder
/// (AP-list + 6-way fading sweep + plain Bessel) once per candidate
/// per slot — if candidate counts are large, that's the real cost
/// driver, not the extraction redundancy already fixed above.
#[test]
#[ignore = "manual diagnostic — Q65-30A/60B candidate-count profiling (new investigation)"]
fn q65_multi_period_candidate_count_diag() {
    use mfsk_core::q65::search::{Spectrogram, coarse_search_on_spec_for};

    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!("skipping: sample tree not found");
        return;
    };
    let mut entries: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    entries.sort();
    let slots: Vec<Vec<f32>> = entries.iter().filter_map(read_wsjtx_wav).collect();

    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 15.0,
        time_tolerance_late_sec: 15.0,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let mut ema_spec = Spectrogram::build_for::<Q65a30>(&slots[0], 12_000);
    for (i, audio) in slots.iter().enumerate() {
        if i > 0 {
            let slot_spec = Spectrogram::build_for::<Q65a30>(audio, 12_000);
            if slot_spec.n_time == ema_spec.n_time && slot_spec.n_freq == ema_spec.n_freq {
                let weight = 1.0_f32 / ((i + 1).min(4) as f32);
                let one_minus = 1.0 - weight;
                for (e, s) in ema_spec.mags_sqr.iter_mut().zip(&slot_spec.mags_sqr) {
                    *e = weight * *s + one_minus * *e;
                }
            }
        }
        let cands = coarse_search_on_spec_for::<Q65a30>(&ema_spec, 12_000, 12_000 * 15, &params);
        eprintln!("slot {i}: {} candidates", cands.len());
    }
}

/// Score-distribution calibration diagnostic: with `max_candidates`
/// raised high enough to see the *full* candidate list (not truncated
/// at 32), where does the real signal's own candidate score fall
/// relative to the rest? Q65-30A's known golden message ("K1JT K9AN
/// R-16" — confirmed via `ionoscatter_6m_full_stack_decodes_via_averaging`)
/// gives a concrete target to check "did truncating to N candidates
/// ever drop the real signal" against.
#[test]
#[ignore = "manual diagnostic — Q65-30A candidate-score calibration (new investigation)"]
fn q65_candidate_score_calibration_diag() {
    use mfsk_core::q65::search::{Spectrogram, coarse_search_on_spec_for};

    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!("skipping: sample tree not found");
        return;
    };
    let mut entries: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    entries.sort();
    let slots: Vec<Vec<f32>> = entries.iter().filter_map(read_wsjtx_wav).collect();

    // No truncation — see the full score distribution above threshold.
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 15.0,
        time_tolerance_late_sec: 15.0,
        score_threshold: 0.05,
        max_candidates: 100_000,
    };

    let mut ema_spec = Spectrogram::build_for::<Q65a30>(&slots[0], 12_000);
    for (i, audio) in slots.iter().enumerate() {
        if i > 0 {
            let slot_spec = Spectrogram::build_for::<Q65a30>(audio, 12_000);
            if slot_spec.n_time == ema_spec.n_time && slot_spec.n_freq == ema_spec.n_freq {
                let weight = 1.0_f32 / ((i + 1).min(4) as f32);
                let one_minus = 1.0 - weight;
                for (e, s) in ema_spec.mags_sqr.iter_mut().zip(&slot_spec.mags_sqr) {
                    *e = weight * *s + one_minus * *e;
                }
            }
        }
        let mut cands =
            coarse_search_on_spec_for::<Q65a30>(&ema_spec, 12_000, 12_000 * 15, &params);
        cands.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        eprintln!(
            "slot {i}: {} total candidates above score_threshold=0.05",
            cands.len()
        );
        eprintln!(
            "  top 40 scores: {:?}",
            cands.iter().take(40).map(|c| c.score).collect::<Vec<_>>()
        );
        // Where would a real ionoscatter signal near 1010 Hz (the
        // decoded freq from the no-AP run) rank? Report every
        // candidate within 30 Hz of that frequency and its rank.
        for (rank, c) in cands.iter().enumerate() {
            if (c.freq_hz - 1010.0).abs() <= 30.0 {
                eprintln!(
                    "  near-1010Hz candidate: rank={rank} freq={:.1} score={:.4}",
                    c.freq_hz, c.score
                );
            }
        }
    }
}

#[test]
#[ignore = "manual diagnostic — Q65-60A candidate-score calibration (2026-07-20 speed follow-up)"]
fn q65_60a_eme6m_candidate_score_calibration_diag() {
    use mfsk_core::q65::search::coarse_search_for;
    use mfsk_core::q65::{Q65a60, SniperRequest};

    let Some(dir) = samples_dir("60A_EME_6m") else {
        eprintln!("skipping: WSJT-X 6m EME sample tree not found");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();

    let nominal_mid = 12_000 * 30;
    let unbounded = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.0,
        max_candidates: 100_000,
    };

    for path in &entries {
        let Some(audio) = read_wsjtx_wav(path) else {
            continue;
        };
        let cands = coarse_search_for::<Q65a60>(&audio, 12_000, nominal_mid, &unbounded);
        println!(
            "== {} — {} total candidates ==",
            path.file_name().unwrap().to_string_lossy(),
            cands.len()
        );
        for (rank, c) in cands.iter().enumerate() {
            let plain =
                SniperRequest::<Q65a60>::new(&audio, 12_000, c.start_sample, c.freq_hz).decode();
            let ap_hint = ApHint::new().with_call1("W7GJ");
            let ap = SniperRequest::<Q65a60>::new(&audio, 12_000, c.start_sample, c.freq_hz)
                .ap_hint(&ap_hint)
                .decode();
            let decoded = plain.as_ref().or(ap.as_ref());
            if rank < 12 || decoded.is_some() {
                println!(
                    "  rank={rank} score={:.4} freq={:.1}Hz -> {:?}",
                    c.score,
                    c.freq_hz,
                    decoded.map(|d| d.message.as_str())
                );
            }
        }
    }
}

/// `DecodeRequest::on_result` streaming callback — real-signal
/// verification, mirroring `tests/ft8_streaming_decode.rs`'s
/// sequential-exact-match contract. Q65's wide-band scan
/// (`decode_scan_for` → `decode_scan_inner`) is sequential and
/// dedup-then-push with no early exit, so — same as FT8's
/// `.sic_rounds()` path — the callback-delivered set must exactly
/// equal the batch `Vec`, no divergence mechanism exists.
#[test]
fn q65_scan_streaming_matches_batch_exactly() {
    let Some(dir) = samples_dir("60A_EME_6m") else {
        eprintln!("skipping: WSJT-X 6m EME sample tree not found");
        return;
    };
    let Some(path) = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .find(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
    else {
        eprintln!("skipping: no .wav files in 60A_EME_6m sample dir");
        return;
    };
    let Some(audio) = read_wsjtx_wav(&path) else {
        eprintln!("skipping: WAV format not recognised");
        return;
    };

    let nominal_mid = 12_000 * 30;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 16,
    };

    let streamed: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::q65::Q65Result| {
        streamed.lock().unwrap().push(r.message.clone());
    };
    let batch = DecodeRequest::<Q65a60>::new(&audio, 12_000, nominal_mid, params)
        .on_result(&on_result)
        .decode();

    let batch_msgs: Vec<String> = batch.iter().map(|d| d.message.clone()).collect();
    let streamed = streamed.into_inner().unwrap();
    eprintln!(
        "{}: batch={} streamed={}",
        path.file_name().unwrap().to_string_lossy(),
        batch_msgs.len(),
        streamed.len()
    );
    assert_eq!(
        streamed, batch_msgs,
        "Q65 wide-band scan: streamed callback deliveries must exactly \
         match the batch result, same order (sequential dedup-then-push, \
         no divergence mechanism exists on this path)"
    );
}

/// `MultiPeriodRequest::on_result` streaming callback — real-signal
/// verification. Unlike the plain wide-band scan, this fires once
/// per *slot* that yields an accepted decode (see the field doc
/// comment on `MultiPeriodRequest::on_result` for why slots are the
/// natural streaming unit here) — still an exact-match contract
/// against the batch `Vec` since `decode_multi_period_for`'s
/// candidate loop is sequential with no cross-thread duplication.
#[test]
fn q65_multi_period_streaming_matches_batch_exactly() {
    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!("skipping: WSJT-X sample tree not found");
        return;
    };
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    paths.sort();
    let audios: Vec<Vec<f32>> = paths.iter().filter_map(read_wsjtx_wav).collect();
    if audios.is_empty() {
        eprintln!("skipping: no readable WAVs in 30A_Ionoscatter_6m sample dir");
        return;
    }
    let slot_refs: Vec<&[f32]> = audios.iter().map(|v| v.as_slice()).collect();

    let nominal_mid = 12_000 * 15;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 15.0,
        time_tolerance_late_sec: 15.0,
        score_threshold: 0.05,
        max_candidates: 8,
    };

    let streamed: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let on_result = |r: &mfsk_core::q65::Q65Result| {
        streamed.lock().unwrap().push(r.message.clone());
    };
    let batch = MultiPeriodRequest::<Q65a30>::new(&slot_refs, 12_000, nominal_mid, params)
        .on_result(&on_result)
        .decode();

    let batch_msgs: Vec<String> = batch.iter().map(|d| d.message.clone()).collect();
    let streamed = streamed.into_inner().unwrap();
    eprintln!(
        "ionoscatter multi-period: batch={} streamed={}",
        batch_msgs.len(),
        streamed.len()
    );
    assert_eq!(
        streamed, batch_msgs,
        "Q65 multi-period scan: per-slot streamed callback deliveries must \
         exactly match the batch result, same order"
    );
}

#[test]
#[ignore = "manual diagnostic — Q65-60D/120D/120E/300A fading-metric candidate-score calibration (2026-07-20 speed follow-up)"]
fn q65_fading_candidate_score_calibration_diag() {
    use mfsk_core::q65::SniperRequest;
    use mfsk_core::q65::search::coarse_search_for;

    struct Golden<'a> {
        want: &'a [&'a str],
        freq_hz: f32,
        tol_hz: f32,
    }

    fn probe<P: mfsk_core::q65::Q65SubMode + Sync>(
        label: &str,
        audio: &[f32],
        time_tolerance_sec: f32,
        b90_ts: f32,
        model: FadingModel,
        golden: Golden,
    ) {
        let unbounded = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: time_tolerance_sec,
            time_tolerance_late_sec: time_tolerance_sec,
            score_threshold: 0.0,
            max_candidates: 100_000,
        };
        let cands = coarse_search_for::<P>(audio, 12_000, 0, &unbounded);
        println!("== {label} — {} total candidates ==", cands.len());

        // Per-candidate decode attempts are independent — parallelize
        // over rayon (same convention as tests/q65_sim_sweep.rs)
        // rather than the sequential loop an earlier version of this
        // diagnostic used; the 300A file alone has 3094 candidates.
        let decode_one = |(rank, c): (usize, &mfsk_core::q65::search::SyncCandidate)| {
            let d = SniperRequest::<P>::new(audio, 12_000, c.start_sample, c.freq_hz)
                .fading(model, b90_ts)
                .decode();
            let is_hit = d.as_ref().is_some_and(|d| {
                golden.want.iter().all(|w| d.message.contains(w))
                    && (d.freq_hz - golden.freq_hz).abs() <= golden.tol_hz
            });
            (rank, c.score, c.freq_hz, d.map(|d| d.message), is_hit)
        };
        #[cfg(feature = "parallel")]
        let results: Vec<_> = {
            use rayon::prelude::*;
            cands.par_iter().enumerate().map(decode_one).collect()
        };
        #[cfg(not(feature = "parallel"))]
        let results: Vec<_> = cands.iter().enumerate().map(decode_one).collect();

        for (rank, score, freq_hz, message, is_hit) in results {
            if rank < 20 || is_hit {
                println!(
                    "  rank={rank} score={score:.4} freq={freq_hz:.1}Hz -> {message:?}{}",
                    if is_hit { "  <-- GOLDEN" } else { "" }
                );
            }
        }
    }

    if let Some(dir) = samples_dir("60D_EME_10GHz") {
        let path = dir.join("201212_1838.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            probe::<Q65d60>(
                "Q65-60D 201212_1838.wav",
                &audio,
                6.0,
                10.0,
                FadingModel::Gaussian,
                Golden {
                    want: &["VK7MO", "K6QPV"],
                    freq_hz: 1000.0,
                    tol_hz: 20.0,
                },
            );
        }
    }

    if let Some(dir) = samples_dir("120D_Rainscatter_10_GHz") {
        let path = dir.join("210117_0920.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            probe::<Q65d120>(
                "Q65-120D 210117_0920.wav",
                &audio,
                13.33,
                20.0,
                FadingModel::Gaussian,
                Golden {
                    want: &["VK3WE", "VK7MO"],
                    freq_hz: 995.0,
                    tol_hz: 15.0,
                },
            );
        }
    }

    if let Some(dir) = samples_dir("120E_Ionoscatter_6m") {
        let entries: Vec<_> = std::fs::read_dir(&dir)
            .into_iter()
            .flatten()
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
            .collect();
        for path in &entries {
            if let Some(audio) = read_wsjtx_wav(path) {
                probe::<Q65e120>(
                    &format!("Q65-120E {}", path.file_name().unwrap().to_string_lossy()),
                    &audio,
                    13.33,
                    12.0,
                    FadingModel::Gaussian,
                    Golden {
                        want: &["KB7IJ", "N0AN"],
                        freq_hz: 1799.0,
                        tol_hz: 15.0,
                    },
                );
            }
        }
    }

    if let Some(dir) = samples_dir("300A_Optical_Scatter") {
        let path = dir.join("201210_0505.wav");
        if let Some(audio) = read_wsjtx_wav(&path) {
            probe::<Q65a300>(
                "Q65-300A 201210_0505.wav",
                &audio,
                207.4,
                5.0,
                FadingModel::Gaussian,
                Golden {
                    want: &["VK7MO", "VK7PD"],
                    freq_hz: 1002.0,
                    tol_hz: 15.0,
                },
            );
        }
    }
}
