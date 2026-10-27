//! `mfsk_encode` / `mfsk_encode_info`: one transmit call over the mode.
//!
//! The properties: the generic call agrees with the per-mode `mfsk_encode_ft8`
//! and with `mfsk_core::encoder::encode` (it is a wrapper, so any gap is a
//! marshalling fault), the buffer and status contracts hold, and the calls the
//! message sends as hashes are what `mfsk_decoder_add_callsign` needs for the
//! answer's `<...>` to resolve.

mod common;

use std::ffi::{CStr, CString, c_char};

use common::*;
use mfsk::*;

fn c(s: &str) -> CString {
    CString::new(s).unwrap()
}

fn last_error() -> String {
    unsafe { CStr::from_ptr(mfsk_last_error()) }
        .to_string_lossy()
        .into_owned()
}

fn encode_pcm(mode: u32, text: &str, flags: u32) -> (MfskStatus, Vec<f32>, usize) {
    let t = c(text);
    let mut n = 0usize;
    let st = unsafe {
        mfsk_encode(
            mode,
            t.as_ptr(),
            flags,
            12_000,
            1_500.0,
            std::ptr::null_mut(),
            0,
            &mut n,
        )
    };
    if st != MfskStatus::InvalidArg || n == 0 {
        return (st, Vec::new(), n);
    }
    let mut pcm = vec![0.0f32; n];
    let mut m = 0usize;
    let st = unsafe {
        mfsk_encode(
            mode,
            t.as_ptr(),
            flags,
            12_000,
            1_500.0,
            pcm.as_mut_ptr(),
            pcm.len(),
            &mut m,
        )
    };
    assert_eq!(m, n);
    (st, pcm, n)
}

struct Info {
    tones: Vec<u8>,
    text: String,
    hashed: String,
}

fn encode_info(mode: u32, text: &str, flags: u32) -> Result<Info, MfskStatus> {
    let t = c(text);
    let mut tones = vec![0u8; 512];
    let mut n = 0usize;
    let (mut text_len, mut hashed_len) = (0usize, 0usize);
    let mut text_out = [0 as c_char; 64];
    let mut hashed = [0 as c_char; 64];
    let st = unsafe {
        mfsk_encode_info(
            mode,
            t.as_ptr(),
            flags,
            tones.as_mut_ptr(),
            tones.len(),
            &mut n,
            text_out.as_mut_ptr(),
            text_out.len(),
            &mut text_len,
            hashed.as_mut_ptr(),
            hashed.len(),
            &mut hashed_len,
        )
    };
    if st != MfskStatus::Ok {
        return Err(st);
    }
    tones.truncate(n);
    let s = |b: &[c_char; 64]| {
        unsafe { CStr::from_ptr(b.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    };
    let (text, hashed) = (s(&text_out), s(&hashed));
    assert_eq!((text_len, hashed_len), (text.len() + 1, hashed.len() + 1));
    Ok(Info {
        tones,
        text,
        hashed,
    })
}

#[test]
fn ft8_agrees_with_the_per_mode_call() {
    // The pieces `mfsk_encode_ft8` takes, and the whole text `mfsk_encode` takes. A directed
    // CQ is the case the two once disagreed on: `pack77` left-justified `CQ DX`, the whole-text
    // packer right-justifies it as rc1 does (`tests/pack77_forms.rs`).
    for (c1, c2, rpt) in [
        ("CQ", "JA1ABC", "PM95"),
        ("CQ DX", "K1ABC", "FN42"),
        ("CQ POTA", "K1ABC", "FN42"),
        ("CQ NA", "JA1ABC/P", "PM95"),
        ("CQ 123", "K1ABC", "FN42"),
        ("K1ABC", "JA1ABC", "R-12"),
        ("K1ABC", "JA1ABC", "RR73"),
    ] {
        let text = format!("{c1} {c2} {rpt}");
        let (st, generic, n) = encode_pcm(MfskMode::Ft8 as u32, &text, 0);
        assert_eq!(st, MfskStatus::Ok, "{text}");
        assert_eq!(generic.len(), n);

        let (a, b, d) = (c(c1), c(c2), c(rpt));
        let mut legacy = vec![0.0f32; n];
        let mut m = 0usize;
        assert_eq!(
            unsafe {
                mfsk_encode_ft8(
                    a.as_ptr(),
                    b.as_ptr(),
                    d.as_ptr(),
                    1_500.0,
                    legacy.as_mut_ptr(),
                    legacy.len(),
                    &mut m,
                )
            },
            MfskStatus::Ok,
            "{text}"
        );
        assert_eq!(m, n);
        assert_eq!(
            generic, legacy,
            "{text}: same message, same carrier, same samples"
        );
    }
}

#[test]
fn info_equals_the_rust_encoder() {
    use mfsk_core::{Mode, encoder::encode};
    for (mfsk_mode, mode, text) in [
        (MfskMode::Ft8, Mode::Ft8, "CQ JA1ABC PM95"),
        (MfskMode::Ft8, Mode::Ft8, "<W9XYZ> PJ4/K1ABC RR73"),
        (MfskMode::Ft4, Mode::Ft4, "k1abc  w9xyz  r-12"),
        (MfskMode::Fst4s60, Mode::Fst4S60, "CQ JA1ABC PM95"),
        (MfskMode::Q65a60, Mode::Q65A60, "CQ JA1ABC PM95"),
        (MfskMode::Wspr, Mode::Wspr, "K1ABC FN42 37"),
    ] {
        let want = encode(mode, text).unwrap();
        let got = encode_info(mfsk_mode as u32, text, 0).unwrap();
        assert_eq!(got.tones, want.tones(), "{mfsk_mode:?} {text}");
        assert_eq!(got.text, want.text(), "{mfsk_mode:?} {text}");
        assert_eq!(
            got.hashed,
            want.hashed_calls().join(" "),
            "{mfsk_mode:?} {text}"
        );
    }
    // The tone count is what mfsk_symbol_count says, where the mode has the stage.
    let ft8 = encode_info(MfskMode::Ft8 as u32, "CQ JA1ABC PM95", 0).unwrap();
    assert_eq!(ft8.tones.len(), mfsk_symbol_count(MfskMode::Ft8 as u32));
    assert_eq!(ft8.hashed, "JA1ABC");
}

#[test]
fn a_buffer_too_small_is_invalid_arg_and_says_how_much() {
    let (st, pcm, n) = encode_pcm(MfskMode::Ft8 as u32, "CQ JA1ABC PM95", 0);
    assert_eq!(st, MfskStatus::Ok);
    assert_eq!(n, pcm.len());

    // PCM: one sample short, nothing written, *out_len is the count needed.
    let t = c("CQ JA1ABC PM95");
    let mut short = vec![7.0f32; n - 1];
    let mut got = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode(
                MfskMode::Ft8 as u32,
                t.as_ptr(),
                0,
                12_000,
                1_500.0,
                short.as_mut_ptr(),
                short.len(),
                &mut got,
            )
        },
        MfskStatus::InvalidArg
    );
    assert_eq!(got, n);
    assert!(short.iter().all(|&x| x == 7.0));
    assert!(
        last_error().contains("buffer too small"),
        "{}",
        last_error()
    );

    // Info: each of the three buffers, one byte short; nothing is written.
    let info = encode_info(MfskMode::Ft8 as u32, "CQ JA1ABC PM95", 0).unwrap();
    let (nt, ntext, nh) = (info.tones.len(), info.text.len() + 1, info.hashed.len() + 1);
    for (tc, xc, hc, what) in [
        (nt - 1, ntext, nh, "tones"),
        (nt, ntext - 1, nh, "text"),
        (nt, ntext, nh - 1, "hashed"),
    ] {
        let mut tones = vec![9u8; nt];
        let mut text = vec![9 as c_char; ntext];
        let mut hashed = vec![9 as c_char; nh];
        let (mut len, mut xlen, mut hlen) = (0usize, 0usize, 0usize);
        let st = unsafe {
            mfsk_encode_info(
                MfskMode::Ft8 as u32,
                t.as_ptr(),
                0,
                tones.as_mut_ptr(),
                tc,
                &mut len,
                text.as_mut_ptr(),
                xc,
                &mut xlen,
                hashed.as_mut_ptr(),
                hc,
                &mut hlen,
            )
        };
        assert_eq!(st, MfskStatus::InvalidArg, "{what}");
        // Every length is the size needed, whichever buffer was short.
        assert_eq!((len, xlen, hlen), (nt, ntext, nh), "{what}");
        assert!(last_error().contains(what), "{what}: {}", last_error());
        assert!(tones.iter().all(|&x| x == 9), "{what}: nothing written");
        assert!(text.iter().all(|&x| x == 9), "{what}: nothing written");
        assert!(hashed.iter().all(|&x| x == 9), "{what}: nothing written");
    }

    // Exactly sized works, and a string passed as NULL/0 is skipped.
    let mut tones = vec![0u8; nt];
    let mut text = vec![0 as c_char; ntext];
    let (mut len, mut xlen, mut hlen) = (0usize, 0usize, 0usize);
    assert_eq!(
        unsafe {
            mfsk_encode_info(
                MfskMode::Ft8 as u32,
                t.as_ptr(),
                0,
                tones.as_mut_ptr(),
                nt,
                &mut len,
                text.as_mut_ptr(),
                ntext,
                &mut xlen,
                std::ptr::null_mut(),
                0,
                &mut hlen,
            )
        },
        MfskStatus::Ok
    );
    assert_eq!(tones, info.tones);
    assert_eq!(
        (xlen, hlen),
        (ntext, nh),
        "the skipped string still reports its size"
    );

    // A pure probe: all three buffers NULL/0 reports every size (and is
    // InvalidArg, since the tones did not fit).
    let (mut len, mut xlen, mut hlen) = (0usize, 0usize, 0usize);
    assert_eq!(
        unsafe {
            mfsk_encode_info(
                MfskMode::Ft8 as u32,
                t.as_ptr(),
                0,
                std::ptr::null_mut(),
                0,
                &mut len,
                std::ptr::null_mut(),
                0,
                &mut xlen,
                std::ptr::null_mut(),
                0,
                &mut hlen,
            )
        },
        MfskStatus::InvalidArg
    );
    assert_eq!((len, xlen, hlen), (nt, ntext, nh));
}

#[test]
fn bad_arguments_have_the_decoder_open_statuses() {
    let t = c("CQ JA1ABC PM95");
    let call = |mode: u32, text: *const c_char, flags: u32| {
        let mut n = 0usize;
        unsafe {
            mfsk_encode(
                mode,
                text,
                flags,
                12_000,
                1_500.0,
                std::ptr::null_mut(),
                0,
                &mut n,
            )
        }
    };
    // Reserved bits.
    for flags in [2u32, 0x8000_0000, 3] {
        assert_eq!(
            call(MfskMode::Ft8 as u32, t.as_ptr(), flags),
            MfskStatus::InvalidArg
        );
        assert!(last_error().contains("reserved"), "{}", last_error());
        assert_eq!(
            encode_info(MfskMode::Ft8 as u32, "CQ JA1ABC PM95", flags).err(),
            Some(MfskStatus::InvalidArg)
        );
    }
    // Null text, as mfsk_encode_ft8.
    assert_eq!(
        call(MfskMode::Ft8 as u32, std::ptr::null(), 0),
        MfskStatus::InvalidArg
    );
    // Not UTF-8.
    let bad = CString::new(vec![0xffu8, 0xfe]).unwrap();
    assert_eq!(
        call(MfskMode::Ft8 as u32, bad.as_ptr(), 0),
        MfskStatus::InvalidArg
    );
    // A text no FT8 message can be.
    assert_eq!(
        call(
            MfskMode::Ft8 as u32,
            c("THIS TEXT IS FAR TOO LONG TO SEND").as_ptr(),
            0
        ),
        MfskStatus::InvalidArg
    );
    assert!(!last_error().is_empty());
    // Out-of-range mode numbers: exactly what mfsk_decoder_open says.
    for mode in [9_999u32, u32::MAX] {
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_decoder_open(mode, std::ptr::null(), std::ptr::null(), &mut st) };
        assert!(d.is_null());
        assert_ne!(st, MfskStatus::Ok);
        assert_eq!(call(mode, t.as_ptr(), 0), st, "mode {mode}");
        assert_eq!(encode_info(mode, "CQ JA1ABC PM95", 0).err(), Some(st));
    }
    // A mode with no text encoder: as decoder_open again.
    let mut st = MfskStatus::Ok;
    let d = unsafe {
        mfsk_decoder_open(
            MfskMode::Msk144 as u32,
            std::ptr::null(),
            std::ptr::null(),
            &mut st,
        )
    };
    assert!(d.is_null());
    assert_eq!(call(MfskMode::Msk144 as u32, t.as_ptr(), 0), st);
    // Zero sample rate does not panic across the boundary.
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode(
                MfskMode::Ft8 as u32,
                t.as_ptr(),
                0,
                0,
                1_500.0,
                std::ptr::null_mut(),
                0,
                &mut n,
            )
        },
        MfskStatus::InvalidArg
    );
}

#[test]
fn the_q65_flag_changes_the_tones() {
    let plain = encode_info(MfskMode::Q65a60 as u32, "CQ JA1ABC PM95", 0).unwrap();
    let flagged = encode_info(
        MfskMode::Q65a60 as u32,
        "CQ JA1ABC PM95",
        MFSK_ENCODE_Q65_COPIED_LAST_TX,
    )
    .unwrap();
    assert_eq!(plain.tones.len(), flagged.tones.len());
    assert_ne!(plain.tones, flagged.tones);
    assert_eq!(plain.text, flagged.text);
    // Ignored by FT8.
    let a = encode_info(MfskMode::Ft8 as u32, "CQ JA1ABC PM95", 0).unwrap();
    let b = encode_info(
        MfskMode::Ft8 as u32,
        "CQ JA1ABC PM95",
        MFSK_ENCODE_Q65_COPIED_LAST_TX,
    )
    .unwrap();
    assert_eq!(a.tones, b.tones);
}

#[test]
fn the_hashed_calls_teach_a_decoder_the_answer() {
    // We send "<VK3NV> JA1ABC/QRP RR73": VK3NV goes out as a hash only.
    let text = "<VK3NV> JA1ABC/QRP RR73";
    let info = encode_info(MfskMode::Ft8 as u32, text, 0).unwrap();
    assert_eq!(info.text, text);
    assert!(
        info.hashed.split(' ').any(|c| c == "VK3NV"),
        "{:?}",
        info.hashed
    );

    let (st, pcm, _) = encode_pcm(MfskMode::Ft8 as u32, text, 0);
    assert_eq!(st, MfskStatus::Ok);
    let mut slot = vec![0i16; 180_000];
    for (i, s) in pcm.iter().enumerate() {
        slot[6_000 + i] = (s * 8_000.0) as i16;
    }

    let cold = open(MfskMode::Ft8, None, None);
    let rows = decode_i16(cold, &slot);
    assert!(
        texts(&rows).iter().any(|t| t.contains("<...>")),
        "{:?}",
        texts(&rows)
    );

    let taught = open(MfskMode::Ft8, None, None);
    for call in info.hashed.split(' ') {
        let call = c(call);
        assert_eq!(
            unsafe { mfsk_decoder_add_callsign(taught, call.as_ptr()) },
            MfskStatus::Ok
        );
    }
    let rows = decode_i16(taught, &slot);
    assert!(
        texts(&rows).iter().any(|t| t.contains("<VK3NV>")),
        "{:?}",
        texts(&rows)
    );
    for d in [cold, taught] {
        unsafe { mfsk_decoder_close(d) };
    }
}
