#![cfg(feature = "jt4")]

use std::path::{Path, PathBuf};

use mfsk_core::jt4::{Jt4Submode, SearchParams, decode_scan};

fn wsjtx_source_dir() -> Option<PathBuf> {
    if let Some(path) = std::env::var_os("WSJTX_SOURCE_DIR") {
        return Some(PathBuf::from(path));
    }
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    [
        manifest.join("../WSJT-X"),
        manifest.join("../../OpenDigi/.cache/upstream/wsjtx"),
    ]
    .into_iter()
    .find(|candidate| candidate.join("lib/jt4.f90").is_file())
}

fn read_pcm16_wav(path: &Path) -> (u32, Vec<f32>) {
    let bytes = std::fs::read(path).expect("read official WAV");
    assert_eq!(&bytes[0..4], b"RIFF");
    assert_eq!(&bytes[8..12], b"WAVE");
    let mut cursor = 12usize;
    let mut sample_rate = None;
    let mut channels = None;
    let mut bits = None;
    let mut pcm = None;
    while cursor + 8 <= bytes.len() {
        let id = &bytes[cursor..cursor + 4];
        let length = u32::from_le_bytes(
            bytes[cursor + 4..cursor + 8]
                .try_into()
                .expect("chunk length"),
        ) as usize;
        let start = cursor + 8;
        let end = start + length;
        assert!(end <= bytes.len(), "malformed WAV chunk");
        if id == b"fmt " {
            assert_eq!(
                u16::from_le_bytes(bytes[start..start + 2].try_into().unwrap()),
                1,
                "fixture must be PCM",
            );
            channels = Some(u16::from_le_bytes(
                bytes[start + 2..start + 4].try_into().unwrap(),
            ));
            sample_rate = Some(u32::from_le_bytes(
                bytes[start + 4..start + 8].try_into().unwrap(),
            ));
            bits = Some(u16::from_le_bytes(
                bytes[start + 14..start + 16].try_into().unwrap(),
            ));
        } else if id == b"data" {
            pcm = Some(
                bytes[start..end]
                    .chunks_exact(2)
                    .map(|sample| i16::from_le_bytes([sample[0], sample[1]]) as f32 / 32_768.0)
                    .collect::<Vec<_>>(),
            );
        }
        cursor = end + (length & 1);
    }
    assert_eq!(channels, Some(1));
    assert_eq!(bits, Some(16));
    (sample_rate.expect("sample rate"), pcm.expect("data chunk"))
}

#[test]
fn decodes_pinned_wsjtx_jt4a_and_jt4f_recordings() {
    let Some(root) = wsjtx_source_dir() else {
        eprintln!(
            "not run: set WSJTX_SOURCE_DIR to the pinned WSJT-X tree \
             to execute official JT4 WAV interoperability"
        );
        return;
    };
    let cases = [
        (
            "samples/JT4/JT4A/DF2ZC_070926_040700.WAV",
            Jt4Submode::A,
            "K1JT DF2ZC JO30",
        ),
        (
            "samples/JT4/JT4F/OK1KIR_141105_175700.WAV",
            Jt4Submode::F,
            "G3WDG OK1KIR RRR",
        ),
    ];
    for (relative, submode, expected) in cases {
        let (sample_rate, audio) = read_pcm16_wav(&root.join(relative));
        let params = SearchParams {
            max_candidates: 20,
            min_score: 0.015,
            ..SearchParams::default()
        };
        let decodes = decode_scan(&audio, sample_rate, sample_rate as usize, submode, &params);
        assert_eq!(
            decodes
                .first()
                .map(|decode| decode.message.to_string())
                .as_deref(),
            Some(expected),
            "wrong decode for pinned WSJT-X fixture {relative}",
        );
    }
}
