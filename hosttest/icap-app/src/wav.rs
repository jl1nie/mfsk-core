// SPDX-License-Identifier: GPL-3.0-only
//! 12 kHz mono 16-bit PCM WAV loader (RIFF chunks walked, so a LIST chunk is fine).

use std::path::Path;

pub fn load_i16(path: impl AsRef<Path>) -> Vec<i16> {
    let p = path.as_ref();
    let b = std::fs::read(p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()));
    assert!(
        b.len() >= 12 && &b[0..4] == b"RIFF" && &b[8..12] == b"WAVE",
        "{}: not a WAV",
        p.display()
    );
    let (mut i, mut fmt_ok, mut data) = (12usize, false, None);
    while i + 8 <= b.len() {
        let id = &b[i..i + 4];
        let n = u32::from_le_bytes(b[i + 4..i + 8].try_into().unwrap()) as usize;
        let body = &b[i + 8..(i + 8 + n).min(b.len())];
        if id == b"fmt " && body.len() >= 16 {
            let ch = u16::from_le_bytes(body[2..4].try_into().unwrap());
            let sr = u32::from_le_bytes(body[4..8].try_into().unwrap());
            let bits = u16::from_le_bytes(body[14..16].try_into().unwrap());
            assert!(
                ch == 1 && sr == 12_000 && bits == 16,
                "{}: need 12 kHz mono 16-bit",
                p.display()
            );
            fmt_ok = true;
        } else if id == b"data" {
            data = Some(body);
        }
        i += 8 + n + (n & 1);
    }
    assert!(fmt_ok, "{}: no fmt chunk", p.display());
    data.expect("no data chunk")
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| i16::from_le_bytes(*c))
        .collect()
}
