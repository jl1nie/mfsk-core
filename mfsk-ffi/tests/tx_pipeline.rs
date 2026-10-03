//! Transmit: pack → tones → PCM, each stage into caller memory.
//!
//! The pre-v2 ABI had seven heap-allocating `mfsk_encode_*` functions
//! that accepted only the three-string `call1 call2 report` path, so a
//! caller with any other message had no way in, and FST4 reached 60A
//! alone. `mfsk-ffi-ft8` already had the better design — this
//! generalises it over `MfskMode`.
//!
//! The property that matters is a **round trip**: what the pipeline
//! synthesises must decode back to what was packed, for every mode with
//! a tone stage. A wiring mistake in the middle (wrong GFSK config,
//! wrong symbol count) is invisible in a size check and obvious here.

mod common;

use std::ffi::CString;

use common::*;
use mfsk::*;

fn pack(call1: &str, call2: &str, report: &str) -> [u8; 77] {
    let (a, b, c) = (
        CString::new(call1).unwrap(),
        CString::new(call2).unwrap(),
        CString::new(report).unwrap(),
    );
    let mut m = [0u8; 77];
    assert_eq!(
        unsafe { mfsk_pack77(a.as_ptr(), b.as_ptr(), c.as_ptr(), m.as_mut_ptr()) },
        MfskStatus::Ok
    );
    m
}

/// Run all three stages for `mode` and return i16 PCM at 12 kHz.
fn synth_i16(mode: MfskMode, msg: &[u8; 77], freq_hz: f32) -> Vec<i16> {
    let n_tones = mfsk_symbol_count(mode as u32);
    assert!(n_tones > 0, "{mode:?} should have a tone stage");
    let mut tones = vec![0u8; n_tones];
    let mut got = 0usize;
    assert_eq!(
        unsafe {
            mfsk_message_to_tones(
                mode as u32,
                msg.as_ptr(),
                tones.as_mut_ptr(),
                tones.len(),
                &mut got,
            )
        },
        MfskStatus::Ok,
        "{mode:?}"
    );
    assert_eq!(got, n_tones);

    let n = mfsk_synth_output_len(mode as u32);
    assert!(n > 0);
    let mut pcm = vec![0i16; n];
    let mut wrote = 0usize;
    assert_eq!(
        unsafe {
            mfsk_tones_to_i16(
                mode as u32,
                tones.as_ptr(),
                tones.len(),
                freq_hz,
                8_000,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut wrote,
            )
        },
        MfskStatus::Ok,
        "{mode:?}"
    );
    assert_eq!(wrote, n);
    pcm
}

/// FT8 and FT4 round-trip through all three stages.
#[test]
fn the_pipeline_round_trips() {
    let msg = pack("CQ", "JA1ABC", "PM95");
    for (mode, slot_s) in [(MfskMode::Ft8, 15.0f32), (MfskMode::Ft4, 7.5)] {
        let frame = synth_i16(mode, &msg, 1500.0);
        let mut slot = vec![0i16; (slot_s * FS as f32) as usize];
        let start = (0.5 * FS as f32) as usize;
        for (i, s) in frame.iter().enumerate() {
            if let Some(d) = slot.get_mut(start + i) {
                *d = d.saturating_add(*s);
            }
        }
        let dec = open(mode, None, None);
        let rows = decode_i16(dec, &slot);
        assert!(
            any_contains(&rows, "JA1ABC"),
            "{mode:?} did not round-trip: {:?}",
            texts(&rows)
        );
        unsafe { mfsk_decoder_close(dec) };
    }
}

/// Every FST4 sub-mode has its own geometry, and a constant baked for
/// 60A is silently wrong for the other four — a factor of 30 between
/// the extremes. The pre-v2 ABI could only reach 60A at all.
#[test]
fn every_fst4_submode_has_its_own_geometry() {
    let seen: Vec<(MfskMode, usize, usize)> = [
        MfskMode::Fst4s15,
        MfskMode::Fst4s30,
        MfskMode::Fst4s60,
        MfskMode::Fst4s120,
        MfskMode::Fst4s300,
    ]
    .into_iter()
    .map(|m| {
        (
            m,
            mfsk_symbol_count(m as u32),
            mfsk_synth_output_len(m as u32),
        )
    })
    .collect();

    for (m, sym, pcm) in &seen {
        assert_eq!(*sym, 160, "{m:?}: every FST4 sub-mode has 160 symbols");
        assert!(*pcm > 0, "{m:?} reports no output length");
        assert_eq!(pcm % sym, 0, "{m:?}: samples should divide by symbols");
    }
    let lens: Vec<usize> = seen.iter().map(|(_, _, p)| *p).collect();
    assert_eq!(lens[0], 160 * 720, "FST4-15");
    assert_eq!(lens[4], 160 * 21_504, "FST4-300");
    assert!(
        lens[4] / lens[0] >= 29,
        "the spread is the reason to ask rather than assume"
    );
}

/// FST4-60A round-trips too, which is the one that can be decoded
/// cheaply enough to check here.
///
/// At 1000 Hz, inside the band `mfsk_decoder_open` gives FST4 by default:
/// the GUI's F Low / F High, 600-1400 Hz. This test synthesised at 1500 Hz
/// from the 0.12 ABI, whose FST4 band was wider, and decoded nothing once
/// the decoder took the GUI's defaults; it skips unless
/// `RUN_FST4_ROUNDTRIP` is set, so only CI's `ffi` job saw it fail.
#[test]
fn fst4_round_trips() {
    if std::env::var_os("RUN_FST4_ROUNDTRIP").is_none() {
        eprintln!("skipping (set RUN_FST4_ROUNDTRIP=1)");
        return;
    }
    let msg = pack("CQ", "JA1ABC", "PM95");
    let frame = synth_i16(MfskMode::Fst4s60, &msg, 1000.0);
    let mut slot = vec![0i16; 60 * FS as usize];
    let start = FS as usize;
    for (i, s) in frame.iter().enumerate() {
        if let Some(d) = slot.get_mut(start + i) {
            *d = d.saturating_add(*s);
        }
    }
    let dec = open(MfskMode::Fst4s60, None, None);
    let rows = decode_i16(dec, &slot);
    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));
    unsafe { mfsk_decoder_close(dec) };
}

/// A mode without a tone stage says so rather than producing something.
#[test]
fn modes_without_a_tone_stage_report_zero_and_refuse() {
    for m in [
        MfskMode::Wspr,
        MfskMode::Jt9,
        MfskMode::Jt65,
        MfskMode::Q65a30,
    ] {
        assert_eq!(mfsk_symbol_count(m as u32), 0, "{m:?}");
        assert_eq!(mfsk_synth_output_len(m as u32), 0, "{m:?}");
        let msg = [0u8; 77];
        let mut buf = [0u8; 256];
        let mut n = 0usize;
        assert_eq!(
            unsafe {
                mfsk_message_to_tones(m as u32, msg.as_ptr(), buf.as_mut_ptr(), buf.len(), &mut n)
            },
            MfskStatus::Unsupported,
            "{m:?}"
        );
    }
    // And a mode value that is not a mode at all.
    assert_eq!(mfsk_symbol_count(9999), 0);
    assert_eq!(mfsk_synth_output_len(9999), 0);
}

/// Short buffers report the size needed rather than truncating.
#[test]
fn short_buffers_report_what_they_needed() {
    let msg = pack("CQ", "JA1ABC", "PM95");
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_message_to_tones(
                MfskMode::Ft8 as u32,
                msg.as_ptr(),
                std::ptr::null_mut(),
                0,
                &mut n,
            )
        },
        MfskStatus::InvalidArg
    );
    assert_eq!(n, mfsk_symbol_count(MfskMode::Ft8 as u32));

    let tones = vec![0u8; n];
    let mut m = 0usize;
    assert_eq!(
        unsafe {
            mfsk_tones_to_i16(
                MfskMode::Ft8 as u32,
                tones.as_ptr(),
                tones.len(),
                1500.0,
                8_000,
                std::ptr::null_mut(),
                0,
                &mut m,
            )
        },
        MfskStatus::InvalidArg
    );
    assert_eq!(m, mfsk_synth_output_len(MfskMode::Ft8 as u32));
}

/// A tone count that does not match the mode is an error, not a buffer
/// overrun waiting to happen.
#[test]
fn a_wrong_tone_count_is_refused() {
    let tones = [0u8; 3];
    let mut pcm = vec![0i16; mfsk_synth_output_len(MfskMode::Ft8 as u32)];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_tones_to_i16(
                MfskMode::Ft8 as u32,
                tones.as_ptr(),
                tones.len(),
                1500.0,
                8_000,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut n,
            )
        },
        MfskStatus::InvalidArg
    );
}

/// The four packers and the unpacker, including the hashed form that
/// needs a session to read back.
#[test]
fn the_packers_cover_the_message_types() {
    let mut m = [0u8; 77];
    let txt = |m: &[u8; 77], dec: *const MfskDecoder| -> String {
        let mut buf = [0i8; 64];
        let mut n = 0usize;
        let st = unsafe {
            if dec.is_null() {
                mfsk_unpack77(m.as_ptr(), buf.as_mut_ptr(), buf.len(), &mut n)
            } else {
                mfsk_decoder_unpack77(dec, m.as_ptr(), buf.as_mut_ptr(), buf.len(), &mut n)
            }
        };
        assert_eq!(st, MfskStatus::Ok);
        let b: &[u8] = unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, n - 1) };
        String::from_utf8_lossy(b).into_owned()
    };

    let grid = CString::new("PM95").unwrap();
    let cq = CString::new("CQ").unwrap();
    let ja = CString::new("JA1ABC").unwrap();
    assert_eq!(
        unsafe { mfsk_pack77_type1(cq.as_ptr(), ja.as_ptr(), grid.as_ptr(), m.as_mut_ptr()) },
        MfskStatus::Ok
    );
    assert!(txt(&m, std::ptr::null()).contains("JA1ABC"));

    let free = CString::new("HELLO WORLD").unwrap();
    assert_eq!(
        unsafe { mfsk_pack77_free_text(free.as_ptr(), m.as_mut_ptr()) },
        MfskStatus::Ok
    );
    assert!(txt(&m, std::ptr::null()).contains("HELLO"));

    // Type 4 hashes the standard call, so it reads as `<...>` until a
    // session has seen it.
    let nonstd = CString::new("JA1ABC/QRP").unwrap();
    let vk = CString::new("VK3NV").unwrap();
    assert_eq!(
        unsafe {
            mfsk_pack77_type4(
                nonstd.as_ptr(),
                vk.as_ptr(),
                std::ptr::null(),
                false,
                m.as_mut_ptr(),
            )
        },
        MfskStatus::Ok
    );
    assert!(txt(&m, std::ptr::null()).contains("<...>"));

    let dec = open(MfskMode::Ft8, None, None);
    let c = CString::new("VK3NV").unwrap();
    unsafe { mfsk_decoder_add_callsign(dec, c.as_ptr()) };
    assert!(
        txt(&m, dec).contains("VK3NV"),
        "a session that knows the call should resolve the hash"
    );
    unsafe { mfsk_decoder_close(dec) };

    // A message that does not fit its format is refused.
    let bad = CString::new("XXX").unwrap();
    let worse = CString::new("Y2Z").unwrap();
    assert_eq!(
        unsafe { mfsk_pack77(bad.as_ptr(), worse.as_ptr(), grid.as_ptr(), m.as_mut_ptr()) },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe {
            mfsk_pack77(
                bad.as_ptr(),
                worse.as_ptr(),
                grid.as_ptr(),
                std::ptr::null_mut(),
            )
        },
        MfskStatus::InvalidArg
    );
}
