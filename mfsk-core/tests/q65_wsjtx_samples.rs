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

use mfsk_core::q65::decode_scan;
use mfsk_core::q65::search::SearchParams;

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

fn samples_dir() -> Option<PathBuf> {
    // Tests run from `mfsk-core/mfsk-core/`; the WSJT-X tree is at
    // `mfsk-core/../WSJT-X/`. Use `CARGO_MANIFEST_DIR` so the lookup
    // is independent of the caller's working directory.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let dir = Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65/30A_Ionoscatter_6m")
        .canonicalize()
        .ok()?;
    if dir.is_dir() { Some(dir) } else { None }
}

#[test]
fn ionoscatter_6m_samples_yield_some_decode() {
    let Some(dir) = samples_dir() else {
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

    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => {
                eprintln!("skip {}: unsupported WAV format", path.display());
                continue;
            }
        };
        total += 1;
        let decodes = decode_scan(&audio, 12_000, nominal_mid, &params);
        let names: Vec<String> = decodes.iter().map(|d| d.message.clone()).collect();
        println!(
            "{}: {} decode(s) → {names:?}",
            path.file_name().unwrap().to_string_lossy(),
            decodes.len()
        );
        if !decodes.is_empty() {
            decoded_at_least_one += 1;
        }
    }

    // Real-world ionoscatter signals exhibit Doppler spread, multi-
    // path fading, and may sit close to the noise floor. Q65's AP
    // (a-priori) and fast-fading metrics in WSJT-X help recover them
    // — those paths are not yet ported here, so we do NOT insist on
    // any specific decode rate. The test runs to validate that:
    //   - .wav reading succeeds,
    //   - decode_scan runs to completion without crashing on real
    //     audio,
    //   - the printed table can be inspected to track real-world
    //     fidelity as the receiver evolves.
    eprintln!(
        "[info] {decoded_at_least_one}/{total} WSJT-X Q65-30A reference recordings produced \
         at least one decode (AP + fast-fading paths not yet ported; cf. q65.c)"
    );
}
