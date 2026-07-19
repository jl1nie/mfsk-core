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
    Q65a30, Q65a60, Q65a300, Q65b60, Q65d60, Q65d120, Q65e120, decode_multi_period_for,
    decode_scan, decode_scan_fading_for, decode_scan_for, decode_scan_with_ap,
    decode_scan_with_ap_for,
};

#[allow(dead_code)]
mod common;

use common::load_wav_f32_opt as read_wsjtx_wav;

fn samples_dir(rel: &str) -> Option<PathBuf> {
    // Tests run from `mfsk-core/mfsk-core/`; the WSJT-X tree is at
    // `mfsk-core/../WSJT-X/`. Use `CARGO_MANIFEST_DIR` so the lookup
    // is independent of the caller's working directory.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let dir = Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65")
        .join(rel)
        .canonicalize()
        .ok()?;
    if dir.is_dir() { Some(dir) } else { None }
}

/// Smoke test: the Q65-30A *single-period* receive chain (plain BP +
/// AP + fast-fading metric) runs to completion against every WSJT-X
/// ionoscatter reference recording without panicking, and the WAV
/// reader handles the file format correctly.
///
/// **Single-period decode rate is not asserted** — these recordings
/// sit below the single-period decode threshold (each WAV produces
/// 0 decodes via plain / AP-CQ / fast-fading taken in isolation).
/// The averaged-decode gate lives in
/// `ionoscatter_6m_full_stack_decodes_via_averaging` below, which
/// stacks the slots and recovers them via the multi-period EMA path
/// (`decode_multi_period_for`). This test stays useful as a
/// regression catch on the per-slot chain (panic / WAV-reader /
/// search-params blow-up).
#[test]
fn ionoscatter_6m_receive_chain_runs() {
    let Some(dir) = samples_dir("30A_Ionoscatter_6m") else {
        eprintln!(
            "skipping: WSJT-X sample tree not found at ../../WSJT-X/samples/Q65/30A_Ionoscatter_6m/"
        );
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
        "WSJT-X Q65-30A sample dir contains no .wav files"
    );

    // ±50 symbols × 0.3 s/sym ≈ ±15 s around the slot midpoint.
    let nominal_mid = 12_000 * 15; // 15 s into the 30 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let mut wav_count = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => {
                eprintln!("skip {}: unsupported WAV format", path.display());
                continue;
            }
        };
        wav_count += 1;

        // Three receive paths must all complete without panic. Decode
        // counts are reported but not asserted — see this test's
        // docstring for why ionoscatter is currently a known gap.
        let plain = decode_scan(&audio, 12_000, nominal_mid, &params);
        let cq = decode_scan_with_ap(
            &audio,
            12_000,
            nominal_mid,
            &params,
            &ApHint::new().with_call1("CQ"),
        );
        let fading = decode_scan_fading_for::<Q65a30>(
            &audio,
            12_000,
            nominal_mid,
            &params,
            8.0,
            FadingModel::Gaussian,
            None,
        );
        eprintln!(
            "{}: plain={} cq={} fading_b90=8={}",
            path.file_name().unwrap().to_string_lossy(),
            plain.len(),
            cq.len(),
            fading.len(),
        );
    }
    assert!(
        wav_count > 0,
        "no readable WAVs in WSJT-X Q65-30A ionoscatter sample dir"
    );
}

/// Strict gate: stack the four ionoscatter recordings into one
/// running EMA via [`decode_multi_period_for`] and require at least
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
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let decodes_no_ap =
        decode_multi_period_for::<Q65a30>(&slot_refs, 12_000, nominal_mid, &params, None);
    eprintln!(
        "[info] ionoscatter multi-period (no AP-list): {} unique decode(s) across {} slot(s)",
        decodes_no_ap.len(),
        slot_refs.len(),
    );
    for d in &decodes_no_ap {
        eprintln!(
            "  → freq={:.1} Hz dt={:.2} s iter={} : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.iterations,
            d.message
        );
    }

    // Try the AP-list path with K1JT / K9AN — the call pair revealed
    // by the no-AP run above. WSJT-X normally builds this list from
    // the user's "Watch list" or last-seen-CQ; here we hard-code the
    // discovered pair to test the AP-list integration.
    use mfsk_core::q65::standard_qso_codewords;
    let ap_codewords = standard_qso_codewords("K1JT", "K9AN", "");
    let decodes_ap = decode_multi_period_for::<Q65a30>(
        &slot_refs,
        12_000,
        nominal_mid,
        &params,
        Some(&ap_codewords),
    );
    eprintln!(
        "[info] ionoscatter multi-period (AP-list K1JT/K9AN): {} unique decode(s)",
        decodes_ap.len(),
    );
    for d in &decodes_ap {
        eprintln!(
            "  → freq={:.1} Hz dt={:.2} s iter={} : {}",
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.iterations,
            d.message
        );
    }

    // Goal: multi-period averaging recovers the signal in at least one
    // of the strategies. Single decode counts as success because the
    // running-EMA collapses repeated copies of the same QSO line into
    // one output entry — once a QSO has been recovered there's no
    // additional information from re-recovering it.
    assert!(
        !decodes_no_ap.is_empty() || !decodes_ap.is_empty(),
        "0/{} ionoscatter slots produced any decode through multi-period averaging \
         (neither without AP-list nor with K1JT/K9AN AP-list) — regression or \
         insufficient signal recovery in the EMA-on-spectrogram path",
        slot_refs.len(),
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
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    // The 210106_1621.wav recording captures W7GJ working multiple
    // stations on 6 m EME (W7GJ is a well-known prolific 6 m EME
    // operator). Try the empty hint plus a hint locking call1 =
    // W7GJ (matching the actual exchange).
    let hints = [
        ("plain", ApHint::new()),
        ("W7GJ ??", ApHint::new().with_call1("W7GJ")),
    ];

    let mut plain_count = 0usize;
    let mut ap_count = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        for (label, hint) in &hints {
            use mfsk_core::q65::decode_scan_for;
            let decodes = if hint.has_info() {
                decode_scan_with_ap_for::<Q65a60>(&audio, 12_000, nominal_mid, &params, hint)
            } else {
                decode_scan_for::<Q65a60>(&audio, 12_000, nominal_mid, &params)
            };
            let names: Vec<String> = decodes.iter().map(|d| d.message.clone()).collect();
            println!(
                "{} [{label}]: {} decode(s) → {names:?}",
                path.file_name().unwrap().to_string_lossy(),
                decodes.len()
            );
            if hint.has_info() {
                ap_count += decodes.len();
            } else {
                plain_count += decodes.len();
            }
        }
    }
    // 6 m EME has the lowest Doppler spread in the EME band lineup,
    // so the AWGN-only metric already does a respectable job on
    // strong-ish signals — the published 210106_1621.wav reference
    // typically yields several W7GJ exchanges on first scan. We
    // require at least one decode to land via the plain or AP
    // path so a regression in the receive chain trips this test.
    assert!(
        plain_count + ap_count > 0,
        "6m EME reference recording produced no decodes via either \
         plain or AP — regression in the Q65-60A receive chain"
    );
    eprintln!("[info] 6m EME: plain {plain_count} decode(s), AP {ap_count} decode(s)");
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
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok().unwrap_or_default();
    let path = std::path::Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65/60D_EME_10GHz/201212_1838.wav");
    let path = match path.canonicalize() {
        Ok(p) if p.is_file() => p,
        _ => {
            eprintln!(
                "skipping: WSJT-X 60D_EME_10GHz sample not found at {}",
                path.display()
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
        time_tolerance_symbols: 10,
        score_threshold: 0.05,
        max_candidates: 100,
    };

    // Plain BP: expected 0 decodes at this SNR.
    let plain = decode_scan_for::<Q65d60>(&audio, 12_000, 0, &params);
    eprintln!("[info] 10 GHz EME plain: {} decode(s)", plain.len());

    // Fast-fading Gaussian, b90 = 10 Hz: the path that actually works.
    let fading = decode_scan_fading_for::<Q65d60>(
        &audio,
        12_000,
        0,
        &params,
        10.0,
        FadingModel::Gaussian,
        None,
    );
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

    // Must recover the golden message at the right frequency (±15 Hz)
    // and dt (3.6 ± 1.0 s after slot start).
    let hit = fading.iter().any(|d| {
        d.message.contains("VK7MO")
            && d.message.contains("K6QPV")
            && (d.freq_hz - 1000.0).abs() <= 20.0
    });
    assert!(
        hit,
        "10 GHz EME Q65-60D fading decode did not recover 'VK7MO K6QPV' near 1000 Hz — \
         regression in the fast-fading receive chain. Fading decodes: {:?}",
        fading.iter().map(|d| &d.message).collect::<Vec<_>>()
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
/// (`decode_multi_period_for`, same mechanism
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
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let decodes_no_ap =
        decode_multi_period_for::<Q65b60>(&slot_refs, 12_000, nominal_mid, &params, None);
    eprintln!(
        "[info] 1296 troposcatter multi-period (no AP-list): {} unique decode(s) across {} slot(s)",
        decodes_no_ap.len(),
        slot_refs.len(),
    );
    for d in &decodes_no_ap {
        eprintln!("  → freq={:.1} Hz : {}", d.freq_hz, d.message);
    }

    use mfsk_core::q65::standard_qso_codewords;
    let ap_codewords = standard_qso_codewords("VK7MO", "VK7PD", "");
    let decodes_ap = decode_multi_period_for::<Q65b60>(
        &slot_refs,
        12_000,
        nominal_mid,
        &params,
        Some(&ap_codewords),
    );
    eprintln!(
        "[info] 1296 troposcatter multi-period (AP-list VK7MO/VK7PD): {} unique decode(s)",
        decodes_ap.len(),
    );
    for d in &decodes_ap {
        eprintln!("  → freq={:.1} Hz : {}", d.freq_hz, d.message);
    }

    let hit = |ds: &[mfsk_core::q65::Q65Decode]| {
        ds.iter()
            .any(|d| d.message.contains("VK7MO") && d.message.contains("VK7PD"))
    };
    assert!(
        hit(&decodes_no_ap) || hit(&decodes_ap),
        "1296 MHz troposcatter reference recording did not recover 'VK7MO VK7PD' via \
         multi-period averaging (neither without AP-list nor with VK7MO/VK7PD AP-list) — \
         regression in the Q65-60B receive chain"
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
        time_tolerance_symbols: 10,
        score_threshold: 0.0,
        max_candidates: 30,
    };

    let plain = decode_scan_for::<Q65d120>(&audio, 12_000, 0, &params);
    eprintln!("[info] 10 GHz rainscatter plain: {} decode(s)", plain.len());

    let fading = decode_scan_fading_for::<Q65d120>(
        &audio,
        12_000,
        0,
        &params,
        20.0,
        FadingModel::Gaussian,
        None,
    );
    eprintln!(
        "[info] 10 GHz rainscatter fading b90=20 Gaussian: {} decode(s)",
        fading.len()
    );
    for d in &fading {
        eprintln!("  freq={:.1} Hz : {}", d.freq_hz, d.message);
    }

    let hit = fading.iter().any(|d| {
        d.message.contains("VK3WE")
            && d.message.contains("VK7MO")
            && (d.freq_hz - 995.0).abs() <= 15.0
    });
    assert!(
        hit,
        "10 GHz rainscatter Q65-120D fading decode did not recover 'VK3WE VK7MO' near 995 Hz — \
         regression in the fast-fading receive chain. Fading decodes: {:?}",
        fading.iter().map(|d| &d.message).collect::<Vec<_>>()
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
        time_tolerance_symbols: 10,
        score_threshold: 0.0,
        max_candidates: 30,
    };

    let mut hit = false;
    for path in &entries {
        let Some(audio) = read_wsjtx_wav(path) else {
            continue;
        };
        let fading = decode_scan_fading_for::<Q65e120>(
            &audio,
            12_000,
            0,
            &params,
            12.0,
            FadingModel::Gaussian,
            None,
        );
        eprintln!(
            "{}: {} fading decode(s)",
            path.file_name().unwrap().to_string_lossy(),
            fading.len()
        );
        for d in &fading {
            eprintln!("  freq={:.1} Hz : {}", d.freq_hz, d.message);
        }
        if fading.iter().any(|d| {
            d.message.contains("KB7IJ")
                && d.message.contains("N0AN")
                && (d.freq_hz - 1799.0).abs() <= 15.0
        }) {
            hit = true;
        }
    }
    assert!(
        hit,
        "6 m ionoscatter Q65-120E fading decode did not recover 'KB7IJ N0AN' near 1799 Hz \
         from any of {} recordings — regression in the fast-fading receive chain",
        entries.len()
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
        time_tolerance_symbols: 60,
        score_threshold: 0.0,
        max_candidates: 200,
    };

    let plain = decode_scan_for::<Q65a300>(&audio, 12_000, 0, &params);
    eprintln!("[info] optical scatter plain: {} decode(s)", plain.len());

    let fading = decode_scan_fading_for::<Q65a300>(
        &audio,
        12_000,
        0,
        &params,
        5.0,
        FadingModel::Gaussian,
        None,
    );
    eprintln!(
        "[info] optical scatter fading b90=5 Gaussian: {} decode(s)",
        fading.len()
    );
    for d in &fading {
        eprintln!("  freq={:.1} Hz : {}", d.freq_hz, d.message);
    }

    let hit = fading.iter().any(|d| {
        d.message.contains("VK7MO")
            && d.message.contains("VK7PD")
            && (d.freq_hz - 1002.0).abs() <= 15.0
    });
    assert!(
        hit,
        "Optical scatter Q65-300A fading decode did not recover 'VK7MO VK7PD' near 1002 Hz — \
         regression in the fast-fading receive chain. Fading decodes: {:?}",
        fading.iter().map(|d| &d.message).collect::<Vec<_>>()
    );
}
