// SPDX-License-Identifier: GPL-3.0-only
//! Run mfsk-core and rust-ft8 (crates.io, 0.2.1) on the same 12 kHz / 16-bit WAV files.
//!
//! usage: ft8-compare <out.csv> <wav>...
//!
//! One CSV row per decode: `file,decoder,message,freq_hz,dt_s,snr_db,ms`, where `ms` is the
//! wall time of the whole file's decode (repeated on each of its rows; a file with no decode
//! gets one row with an empty message so its time is not lost). One file at a time; set
//! `RAYON_NUM_THREADS=1` for a single-threaded run on both sides.
//!
//! Decoders (all with the band 200-3000 Hz, matching `jt9 -8 -d 3 -L 200 -H 3000`):
//! - `mfsk`:       `Decoder<Ft8>`, `Depth::Deep`, `ApMode::Full` (task `ft8/t1` in
//!                 scripts/upstream_tasks.json)
//! - `rf8-default`: rust-ft8 `DecoderConfig::default()` with the band set (passes 2)
//! - `rf8-deep`:    rust-ft8 passes 3, `deep_search`, the configuration its own
//!                 `tests/full_diag_report.rs` is measured with (there with band 100-3500)
//!
//! DT: rust-ft8 documents `window_start_offset` as "recording start relative to the true slot
//! window"; its README passes -0.9. DT is reported but not used to match (freq and text are).

use mfsk_core::decoder::{ApMode, DecodeParams, Decoder, Depth, Ft8Extras, Ft8Strategy, SlotInput};
use mfsk_core::ft8::Ft8;
use rust_ft8::{DecoderConfig, decode_audio};
use std::io::Write;
use std::time::Instant;

fn read_wav_i16(path: &str) -> Vec<i16> {
    let b = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    assert!(&b[0..4] == b"RIFF" && &b[8..12] == b"WAVE", "{path}: not a WAV");
    let mut i = 12;
    let (mut fmt_ok, mut data) = (false, None);
    while i + 8 <= b.len() {
        let id = &b[i..i + 4];
        let n = u32::from_le_bytes(b[i + 4..i + 8].try_into().unwrap()) as usize;
        let body = &b[i + 8..(i + 8 + n).min(b.len())];
        if id == b"fmt " {
            let tag = u16::from_le_bytes(body[0..2].try_into().unwrap());
            let ch = u16::from_le_bytes(body[2..4].try_into().unwrap());
            let sr = u32::from_le_bytes(body[4..8].try_into().unwrap());
            let bits = u16::from_le_bytes(body[14..16].try_into().unwrap());
            assert!(tag == 1 && ch == 1 && sr == 12_000 && bits == 16, "{path}: need 12 kHz mono 16-bit PCM");
            fmt_ok = true;
        } else if id == b"data" {
            data = Some(body);
        }
        i += 8 + n + (n & 1);
    }
    assert!(fmt_ok);
    data.expect("no data chunk").chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]])).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (out, wavs) = args.split_first().expect("usage: ft8-compare <out.csv> <wav>...");
    let mut csv = std::io::BufWriter::new(std::fs::File::create(out).unwrap());
    writeln!(csv, "file,decoder,message,freq_hz,dt_s,snr_db,ms").unwrap();

    let rf8_default = DecoderConfig { nfa: 200.0, nfb: 3000.0, ..DecoderConfig::default() };
    let rf8_deep = DecoderConfig { nfa: 200.0, nfb: 3000.0, passes: 3, deep_search: true, ..DecoderConfig::default() };

    for path in wavs {
        let name = std::path::Path::new(path).file_name().unwrap().to_string_lossy().to_string();
        let i16s = read_wav_i16(path);
        let f32s: Vec<f32> = i16s.iter().map(|&s| s as f32 / 32768.0).collect();
        let mut emit = |dec: &str, rows: Vec<(String, f32, f32, f32)>, ms: f64| {
            if rows.is_empty() {
                writeln!(csv, "{name},{dec},,,,,{ms:.1}").unwrap();
            }
            for (m, f, dt, snr) in rows {
                writeln!(csv, "{name},{dec},\"{m}\",{f:.1},{dt:.2},{snr:.0},{ms:.1}").unwrap();
            }
        };

        let only = std::env::var("FT8_COMPARE_ONLY").unwrap_or_default();
        // mfsk-core: a fresh decoder per file (no hash-table or state carried between files).
        if only != "sic3" {
        let t = Instant::now();
        let params = DecodeParams::for_band((200.0, 3000.0)).depth(Depth::Deep).ap(ApMode::Full);
        let res = Decoder::<Ft8>::new(params).decode(&SlotInput::i16(&i16s));
        let ms = t.elapsed().as_secs_f64() * 1e3;
        emit("mfsk", res.rows.iter().map(|r| (r.decoded.text.clone(), r.decoded.freq_hz, r.decoded.dt_sec, r.decoded.snr_db)).collect(), ms);
        }

        // Diagnostic variants of the same request: only the search strategy differs
        // (`Ft8Extras::tuning.strategy`; the default above is `SicEarly`).
        if std::env::var_os("FT8_COMPARE_STRATEGIES").is_some() || only == "sic3" {
            for (label, strat) in [("mfsk-sic3", Ft8Strategy::SicRounds(3)), ("mfsk-single", Ft8Strategy::SinglePass)] {
                if only == "sic3" && label != "mfsk-sic3" {
                    continue;
                }
                let t = Instant::now();
                let params = DecodeParams::for_band((200.0, 3000.0)).depth(Depth::Deep).ap(ApMode::Full);
                let mut ex = Ft8Extras::default();
                ex.tuning.strategy = Some(strat);
                let res = Decoder::<Ft8>::new(params).with_extras(ex).decode(&SlotInput::i16(&i16s));
                let ms = t.elapsed().as_secs_f64() * 1e3;
                emit(label, res.rows.iter().map(|r| (r.decoded.text.clone(), r.decoded.freq_hz, r.decoded.dt_sec, r.decoded.snr_db)).collect(), ms);
            }
        }

        if only == "mfsk" || only == "sic3" {
            continue;
        }
        for (label, cfg) in [("rf8-default", &rf8_default), ("rf8-deep", &rf8_deep)] {
            let t = Instant::now();
            let res = decode_audio(&f32s, cfg, -0.9, 0);
            let ms = t.elapsed().as_secs_f64() * 1e3;
            emit(label, res.iter().map(|m| (m.message.clone(), m.freq, m.dt, m.snr as f32)).collect(), ms);
        }
    }
}
