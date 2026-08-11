// SPDX-License-Identifier: GPL-3.0-or-later
//! Common test fixtures: channel models, RNG helpers, asset paths,
//! WAV loaders.

pub mod air_channel;
pub mod channel;
pub mod corpus;
// uvpacket is an experimental example, not a feature of this crate —
// its Eb/N0 helpers are gated so the shared scaffolding stays usable
// without it.
pub mod golden;
#[cfg(feature = "uvpacket")]
pub mod uvpacket_channel;

/// Minimal RIFF/WAVE loader — parses the standard `fmt ` + `data`
/// chunks (plus any `JUNK` / `LIST` / `bext` chunks ahead of `data`)
/// and returns mono i16 samples. Asserts 12 kHz / mono / 16-bit PCM.
/// Source-faithful port of the strictest pre-existing loader
/// (`tests/ft8_decode_block_real_qso.rs::load_wav_i16` before the
/// v0.6.0 consolidation in #49 cat A).
///
/// `dead_code`: each integration test is its own crate, so consumers
/// that don't load WAVs (e.g. uvpacket tests) see this fn as unused.
#[allow(dead_code)]
pub fn load_wav_i16(path: impl AsRef<std::path::Path>) -> Vec<i16> {
    let p = path.as_ref();
    load_wav_i16_opt(p).unwrap_or_else(|| {
        panic!(
            "load WAV failed (missing or non-conforming): {}",
            p.display()
        )
    })
}

/// f32-normalised version of [`load_wav_i16`]. Returns samples in
/// approximately `[-1.0, 1.0)` (i16 / 32_768).
#[allow(dead_code)]
pub fn load_wav_f32(path: impl AsRef<std::path::Path>) -> Vec<f32> {
    load_wav_i16(path)
        .into_iter()
        .map(|s| s as f32 / 32_768.0)
        .collect()
}

/// Soft variant of [`load_wav_i16`] — returns `None` if the file is
/// missing, not a RIFF/WAVE, or not 12 kHz / mono / 16-bit PCM. Used
/// by tests that skip cleanly when the WSJT-X reference tree is not
/// at the expected sibling path.
#[allow(dead_code)]
pub fn load_wav_i16_opt(path: impl AsRef<std::path::Path>) -> Option<Vec<i16>> {
    let bytes = std::fs::read(path).ok()?;
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut i = 12usize;
    let mut sample_rate = 0u32;
    let mut bits = 0u16;
    let mut channels = 0u16;
    let mut data_off = 0usize;
    let mut data_len = 0usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?) as usize;
        i += 8;
        if id == b"fmt " && i + 16 <= bytes.len() {
            channels = u16::from_le_bytes(bytes[i + 2..i + 4].try_into().ok()?);
            sample_rate = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?);
            bits = u16::from_le_bytes(bytes[i + 14..i + 16].try_into().ok()?);
        } else if id == b"data" {
            data_off = i;
            data_len = len.min(bytes.len() - i);
        }
        i += len;
        if len % 2 == 1 {
            i += 1;
        }
    }
    if channels != 1 || sample_rate != 12_000 || bits != 16 || data_off == 0 {
        return None;
    }
    let samples = &bytes[data_off..data_off + data_len];
    Some(
        samples
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect(),
    )
}

/// Soft f32 variant. See [`load_wav_i16_opt`].
#[allow(dead_code)]
pub fn load_wav_f32_opt(path: impl AsRef<std::path::Path>) -> Option<Vec<f32>> {
    Some(
        load_wav_i16_opt(path)?
            .into_iter()
            .map(|s| s as f32 / 32_768.0)
            .collect(),
    )
}

/// Build a path to an asset under `embedded-poc/assets/` that resolves
/// regardless of where the crate is checked out (CI runners, contributor
/// dev boxes, the maintainer's `/home/ubuntu/...` tree). Equivalent to
/// `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/", $asset)`.
#[macro_export]
macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

/// On-air FT8 recordings used by the `decode_block` performance and
/// recall sweeps. Two consecutive 15 s slots from `jl1nie/RustFT8` plus
/// the WSJT-X-distributed reference recording.
///
/// `qso3_busy.wav` is bit-identical to WSJT-X
/// `samples/FT8/210703_133430.wav` (verified via `cmp` 2026-05-04, see
/// the module doc on `ft8_reference_suite_recall.rs`).
//
// `dead_code`: each integration test is its own crate, so consumers
// that only need the channel helpers (e.g. uvpacket tests) see this
// const as unused.
#[allow(dead_code)]
pub const REAL_QSO_WAVS: &[&str] = &[
    asset_path!("191111_110130.wav"),
    asset_path!("191111_110200.wav"),
    asset_path!("qso3_busy.wav"),
];
