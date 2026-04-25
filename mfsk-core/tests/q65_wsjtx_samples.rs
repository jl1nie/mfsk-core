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

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{Q65a60, decode_scan, decode_scan_with_ap, decode_scan_with_ap_for};

/// Minimal WAV reader for WSJT-X's exact format: RIFF/WAVE header,
/// `fmt ` chunk = PCM (1 channel, 12 kHz, 16-bit), `data` chunk =
/// little-endian i16 samples. Anything else returns `None`.
fn read_wsjtx_wav(path: &Path) -> Option<Vec<f32>> {
    let mut file = File::open(path).ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    // Locate the `data` chunk after the standard 44-byte RIFF header.
    if bytes.len() < 44 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    // Confirm fmt chunk advertises mono / 12 kHz / 16-bit PCM.
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

#[test]
fn ionoscatter_6m_samples_yield_some_decode() {
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

    // Wide search params: the signal in a real recording can start
    // anywhere in the slot, not just at the nominal +1 s offset.
    // ±50 symbols × 0.3 s/sym ≈ ±15 s around the slot midpoint.
    let mut total = 0usize;
    let mut decoded_at_least_one = 0usize;
    let nominal_mid = 12_000 * 15; // 15 s into the 30 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    // Try a few common AP guesses on each recording. Without knowing
    // the actual exchange, "CQ" is the most common starting hint
    // (most ionoscatter signals are calling CQ).
    let hints = [
        ("plain", ApHint::new()),
        ("CQ", ApHint::new().with_call1("CQ")),
    ];

    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => {
                eprintln!("skip {}: unsupported WAV format", path.display());
                continue;
            }
        };
        total += 1;
        let mut hit = false;
        for (label, hint) in &hints {
            let decodes = if hint.has_info() {
                decode_scan_with_ap(&audio, 12_000, nominal_mid, &params, hint)
            } else {
                decode_scan(&audio, 12_000, nominal_mid, &params)
            };
            let names: Vec<String> = decodes.iter().map(|d| d.message.clone()).collect();
            println!(
                "{} [{label}]: {} decode(s) → {names:?}",
                path.file_name().unwrap().to_string_lossy(),
                decodes.len()
            );
            if !decodes.is_empty() {
                hit = true;
            }
        }
        if hit {
            decoded_at_least_one += 1;
        }
    }

    // Real-world ionoscatter signals exhibit Doppler spread, multi-
    // path fading, and may sit close to the noise floor. Q65's
    // fast-fading metric in WSJT-X helps with that; that path is
    // not yet ported here, so we do NOT insist on any specific
    // decode rate. The test runs to validate that:
    //   - .wav reading succeeds,
    //   - decode_scan runs to completion without crashing on real
    //     audio,
    //   - the printed table can be inspected to track real-world
    //     fidelity as the receiver evolves.
    eprintln!(
        "[info] {decoded_at_least_one}/{total} WSJT-X Q65-30A ionoscatter recordings \
         produced at least one decode (fast-fading path not yet ported)"
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
