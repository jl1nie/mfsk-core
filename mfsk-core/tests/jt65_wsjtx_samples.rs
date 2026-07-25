//! JT65B interoperability against the off-air recordings shipped by the
//! pinned WSJT-X source tree.

#![cfg(all(feature = "jt65", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

use mfsk_core::core::dsp::resample::resample_f32_to_12k_f32;
use mfsk_core::jt65::search::SearchParams;
use mfsk_core::jt65::{
    Jt65ApContext, Jt65QsoProgress, Jt65b, decode_multi_period_for,
    decode_multi_period_for_with_ap, decode_scan_for,
};

#[allow(dead_code)]
mod common;
use common::load_wav_f32_opt;

fn wsjtx_source_dir() -> Option<PathBuf> {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    [
        std::env::var_os("WSJTX_SOURCE_DIR").map(PathBuf::from),
        Some(manifest.join("../WSJT-X")),
        Some(manifest.join("../../OpenDigi/.cache/upstream/wsjtx")),
    ]
    .into_iter()
    .flatten()
    .find(|root| root.join("lib/jt65_decode.f90").is_file())
}

fn load_unsigned_pcm8_mono(path: &Path) -> Option<(u32, Vec<f32>)> {
    let bytes = std::fs::read(path).ok()?;
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut offset = 12usize;
    let mut sample_rate = 0u32;
    let mut channels = 0u16;
    let mut bits = 0u16;
    let mut pcm_format = 0u16;
    let mut data = None;
    while offset + 8 <= bytes.len() {
        let chunk_id = &bytes[offset..offset + 4];
        let chunk_len = u32::from_le_bytes(bytes[offset + 4..offset + 8].try_into().ok()?) as usize;
        offset += 8;
        let chunk_end = offset.checked_add(chunk_len)?.min(bytes.len());
        if chunk_id == b"fmt " && chunk_end >= offset + 16 {
            pcm_format = u16::from_le_bytes(bytes[offset..offset + 2].try_into().ok()?);
            channels = u16::from_le_bytes(bytes[offset + 2..offset + 4].try_into().ok()?);
            sample_rate = u32::from_le_bytes(bytes[offset + 4..offset + 8].try_into().ok()?);
            bits = u16::from_le_bytes(bytes[offset + 14..offset + 16].try_into().ok()?);
        } else if chunk_id == b"data" {
            data = Some(&bytes[offset..chunk_end]);
        }
        offset = chunk_end + (chunk_len & 1);
    }
    if pcm_format != 1 || channels != 1 || bits != 8 || sample_rate == 0 {
        return None;
    }
    Some((
        sample_rate,
        data?
            .iter()
            .map(|sample| (*sample as f32 - 128.0) / 128.0)
            .collect(),
    ))
}

#[test]
fn decodes_pinned_wsjtx_jt65b_recordings() {
    let Some(root) = wsjtx_source_dir() else {
        eprintln!(
            "not run: set WSJTX_SOURCE_DIR to the pinned WSJT-X tree \
             to execute JT65B reference-audio interoperability"
        );
        return;
    };
    let sample_dir = root.join("samples/JT65/JT65B");
    let mut paths = std::fs::read_dir(&sample_dir)
        .expect("read pinned JT65B sample directory")
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .and_then(|extension| extension.to_str())
                .is_some_and(|extension| extension.eq_ignore_ascii_case("wav"))
        })
        .collect::<Vec<_>>();
    paths.sort();
    assert_eq!(paths.len(), 8, "pinned WSJT-X JT65B fixture count changed");

    let params = SearchParams {
        freq_min_hz: 100.0,
        freq_max_hz: 2_700.0,
        time_tolerance_symbols: 12,
        score_threshold: 0.05,
        max_candidates: 100,
    };
    let mut averaging_audio = Vec::new();
    for path in &paths {
        let Some(audio) = load_wav_f32_opt(path) else {
            // The historical DL7UAE fixture is 11,025 Hz unsigned PCM-8;
            // it is covered separately once resampled into the browser's
            // canonical 12 kHz DSP rate.
            continue;
        };
        averaging_audio.push(audio);
    }
    assert_eq!(averaging_audio.len(), 7);
    let recordings = averaging_audio
        .iter()
        .map(Vec::as_slice)
        .collect::<Vec<_>>();
    let averaged = decode_multi_period_for::<Jt65b>(&recordings, 12_000, 0, &params);
    let messages = averaged
        .iter()
        .map(|decode| decode.message.to_string())
        .collect::<Vec<_>>();
    assert!(
        messages.iter().any(|message| message == "CQ K1ABC FN42"),
        "three same-sequence official periods must average to CQ K1ABC FN42"
    );
    assert!(
        messages.iter().any(|message| message == "K1ABC G4XYZ IO91"),
        "three opposite-sequence official periods must average to K1ABC G4XYZ IO91"
    );

    let ap_periods = [recordings[1], recordings[3]];
    let without_ap = decode_multi_period_for::<Jt65b>(&ap_periods, 12_000, 0, &params);
    assert!(
        !without_ap
            .iter()
            .any(|decode| decode.message.to_string() == "K1ABC G4XYZ IO91"),
        "the pinned two-period AP exercise unexpectedly decoded without AP"
    );
    let with_ap = decode_multi_period_for_with_ap::<Jt65b>(
        &ap_periods,
        12_000,
        0,
        &params,
        &Jt65ApContext {
            my_call: "K1ABC".to_owned(),
            dx_call: None,
            dx_grid: None,
            progress: Jt65QsoProgress::Tx2,
        },
    );
    assert!(
        with_ap
            .iter()
            .any(|decode| decode.message.to_string() == "K1ABC G4XYZ IO91"),
        "station-aware AP must decode the pinned files 2+4 exercise; got {with_ap:?}"
    );
}

#[test]
fn decodes_legacy_pinned_wsjtx_jt65b_recording_after_resampling() {
    let Some(root) = wsjtx_source_dir() else {
        eprintln!(
            "not run: set WSJTX_SOURCE_DIR to the pinned WSJT-X tree \
             to execute legacy JT65B reference-audio interoperability"
        );
        return;
    };
    let path = root.join("samples/JT65/JT65B/DL7UAE_040308_002400.wav");
    let (sample_rate, audio) =
        load_unsigned_pcm8_mono(&path).expect("load 11,025 Hz unsigned PCM-8 fixture");
    assert_eq!(sample_rate, 11_025);
    let audio = resample_f32_to_12k_f32(&audio, sample_rate);
    let params = SearchParams {
        freq_min_hz: 1_000.0,
        freq_max_hz: 2_000.0,
        time_tolerance_symbols: 16,
        score_threshold: 0.03,
        max_candidates: 8,
    };
    let decodes = decode_scan_for::<Jt65b>(&audio, 12_000, 12_000, &params);
    let messages = decodes
        .iter()
        .map(|decode| decode.message.to_string())
        .collect::<Vec<_>>();
    assert!(
        messages.iter().any(|message| message == "K1JT DL7UAE JO62"),
        "legacy JT65B fixture must decode the WSJT tutorial golden; got {messages:?}"
    );
    assert!(
        messages
            .iter()
            .all(|message| matches!(message.as_str(), "K1JT DL7UAE JO62" | "K1JT SP6GWB JO80")),
        "legacy JT65B fixture produced a false decode: {messages:?}"
    );
}
