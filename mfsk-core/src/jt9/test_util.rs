//! Shared WAV loader for JT9's own `#[ignore]` diagnostic probes
//! (`rx.rs`, `decode.rs`, `search.rs`, `softsym.rs`, and this module's
//! own tests). Chunk-walks the RIFF container instead of assuming
//! `data` starts at byte offset 44 — the ~8 hand-rolled copies this
//! replaced (#421) broke on any WAV with extra chunks (`JUNK`,
//! `LIST`, `bext`, …) ahead of `data`. Source-faithful port of the
//! same walk `tests/common::load_wav_i16_opt` already does for the
//! integration-test side; this one stays `f32`-only and `pub(in
//! crate::jt9)` since every call site here wants normalised samples.

#[allow(dead_code)]
pub(in crate::jt9) fn load_wav_f32(path: impl AsRef<std::path::Path>) -> Vec<f32> {
    let p = path.as_ref();
    load_wav_f32_opt(p).unwrap_or_else(|| panic!("load WAV failed: {}", p.display()))
}

/// Soft variant — `None` on a missing or malformed file, for probes
/// that skip cleanly when a gitignored local corpus isn't present.
#[allow(dead_code)]
pub(in crate::jt9) fn load_wav_f32_opt(path: impl AsRef<std::path::Path>) -> Option<Vec<f32>> {
    let bytes = std::fs::read(path.as_ref()).ok()?;
    let mut i = 12usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?) as usize;
        let start = i + 8;
        if id == b"data" {
            if start + len > bytes.len() {
                return None;
            }
            let samples = &bytes[start..start + len];
            return Some(
                samples
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0)
                    .collect(),
            );
        }
        i = start + len + (len % 2);
    }
    None
}
