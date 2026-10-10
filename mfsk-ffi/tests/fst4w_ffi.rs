//! FST4W and the long FST4 periods through the C ABI (#649, phase 4): the
//! mode list, the pack -> tones -> PCM -> decode round trip, `hash22` on a row,
//! and the Keff-50 known-call list.

mod common;
use common::*;
use mfsk::*;
use std::ffi::{CStr, CString, c_char};

const W: [(MfskMode, &str, u32); 4] = [
    (MfskMode::Fst4w120, "FST4W-120", 120),
    (MfskMode::Fst4w300, "FST4W-300", 300),
    (MfskMode::Fst4w900, "FST4W-900", 900),
    (MfskMode::Fst4w1800, "FST4W-1800", 1800),
];

#[test]
fn the_new_modes_are_listed_and_described() {
    for (mode, name, period) in W.iter().copied().chain([
        (MfskMode::Fst4s900, "FST4-900", 900),
        (MfskMode::Fst4s1800, "FST4-1800", 1800),
    ]) {
        let n = unsafe { CStr::from_ptr(mfsk_mode_name(mode as u32)) }
            .to_str()
            .unwrap();
        assert_eq!(n, name);
        let mut found = MfskMode::Ft8;
        let c = CString::new(name).unwrap();
        assert_eq!(
            unsafe { mfsk_mode_from_name(c.as_ptr(), &mut found) },
            MfskStatus::Ok
        );
        assert_eq!(found, mode);
        let mut info = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
        assert_eq!(
            unsafe { mfsk_mode_info(mode as u32, info.as_mut_ptr()) },
            MfskStatus::Ok
        );
        let info = unsafe { info.assume_init() };
        assert_eq!(info.t_slot_s, period as f32, "{name}");
        assert_eq!(info.slot_samples_12k, period * 12_000, "{name}");
        assert_eq!(info.n_symbols, 160, "{name}");
        let w = name.starts_with("FST4W");
        assert_eq!(info.fec_k, if w { 74 } else { 101 }, "{name}");
        assert_eq!(info.payload_bits, if w { 50 } else { 77 }, "{name}");
        assert_ne!(info.caps & MFSK_CAP_BUDGET, 0, "{name}");
        assert_eq!(
            mfsk_symbol_count(mode as u32),
            160,
            "{name} has a tone stage"
        );
    }
    // the largest slot transform is FST4W-1800's, as the registry says
    let mut info = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
    unsafe { mfsk_mode_info(MfskMode::Fst4w1800 as u32, info.as_mut_ptr()) };
    assert_eq!(unsafe { info.assume_init() }.decode_fft1_size, 21_591_360);
}

/// A slot of `mode` holding `text` at `freq_hz`, 1 s after the period starts,
/// built only through the C calls.
fn fst4w_slot(mode: MfskMode, period: u32, text: &str, freq_hz: f32) -> Vec<i16> {
    let t = CString::new(text).unwrap();
    let mut msg = [0u8; 77];
    assert_eq!(
        unsafe { mfsk_fst4w_pack(t.as_ptr(), msg.as_mut_ptr()) },
        MfskStatus::Ok,
        "{text:?}"
    );
    let mut tones = vec![0u8; mfsk_symbol_count(mode as u32)];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_message_to_tones(
                mode as u32,
                msg.as_ptr(),
                tones.as_mut_ptr(),
                tones.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    let mut pcm = vec![0i16; mfsk_synth_output_len(mode as u32)];
    let mut w = 0usize;
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
                &mut w,
            )
        },
        MfskStatus::Ok
    );
    let mut slot = vec![0i16; period as usize * 12_000];
    for (d, s) in slot.iter_mut().skip(12_000).zip(pcm.iter().take(w)) {
        *d = *s;
    }
    slot
}

#[test]
fn pack_transmit_decode_round_trip_over_the_c_abi() {
    let slot = fst4w_slot(MfskMode::Fst4w120, 120, "K1ABC FN42 37", 1500.0);
    let d = open(MfskMode::Fst4w120, None, None);
    let rows = decode_i16(d, &slot);
    assert_eq!(texts(&rows), ["K1ABC FN42 37"]);
    let r = &rows[0];
    assert_eq!(r.mode, MfskMode::Fst4w120);
    assert_eq!(r.info_bits, 74);
    assert_eq!(r.key_bits, 50);
    assert_eq!(r.flags & MFSK_DECODE_FLAG_HAS_HASH22, 0);
    assert_eq!(r.hash22, 0);
    let mut info = [0u8; 80];
    let mut n = 0usize;
    assert_eq!(
        unsafe { mfsk_decoder_copy_info(d, 0, info.as_mut_ptr(), info.len(), &mut n) },
        MfskStatus::Ok
    );
    assert_eq!(n, 74);
    unsafe { mfsk_decoder_close(d) };
}

#[test]
fn an_unresolved_hash_reaches_the_row() {
    let slot = fst4w_slot(MfskMode::Fst4w120, 120, "<JA1XYZ> PM95AA", 1500.0);
    let d = open(MfskMode::Fst4w120, None, None);
    let rows = decode_i16(d, &slot);
    assert_eq!(texts(&rows), ["<...> PM95AA"]);
    let r = &rows[0];
    assert_ne!(r.flags & MFSK_DECODE_FLAG_HAS_HASH22, 0);
    // the 22 bits the sender hashed "JA1XYZ" to
    assert_eq!(
        r.hash22,
        mfsk_core::msg::hash_table::ihashcall("JA1XYZ", 22)
    );
    unsafe { mfsk_decoder_close(d) };
}

fn get_wcalls(d: *mut MfskDecoder) -> (MfskStatus, String) {
    let mut buf = vec![0 as c_char; 4096];
    let mut n = 0usize;
    let st = unsafe { mfsk_decoder_get_wcalls(d, buf.as_mut_ptr(), buf.len(), &mut n) };
    let s = unsafe { CStr::from_ptr(buf.as_ptr()) }
        .to_string_lossy()
        .into_owned();
    (
        st,
        if st == MfskStatus::Ok {
            s
        } else {
            String::new()
        },
    )
}

fn set_wcalls(d: *mut MfskDecoder, s: &str) -> MfskStatus {
    let c = CString::new(s).unwrap();
    unsafe { mfsk_decoder_set_wcalls(d, c.as_ptr()) }
}

#[test]
fn the_known_call_list_round_trips_and_is_learned() {
    let d = open(MfskMode::Fst4w120, None, None);
    assert_eq!(get_wcalls(d), (MfskStatus::Ok, String::new()));
    assert_eq!(set_wcalls(d, "JA1XYZ PM95\nVK3NV QF22\n"), MfskStatus::Ok);
    assert_eq!(
        get_wcalls(d),
        (MfskStatus::Ok, "JA1XYZ PM95\nVK3NV QF22".to_string())
    );
    // size query
    let mut n = 0usize;
    assert_eq!(
        unsafe { mfsk_decoder_get_wcalls(d, std::ptr::null_mut(), 0, &mut n) },
        MfskStatus::InvalidArg
    );
    assert_eq!(n, "JA1XYZ PM95\nVK3NV QF22".len() + 1);
    // 101 entries are refused, and the list is untouched
    let many = (0..101)
        .map(|i| format!("C{i}"))
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(set_wcalls(d, &many), MfskStatus::InvalidArg);
    assert_eq!(get_wcalls(d).1, "JA1XYZ PM95\nVK3NV QF22");
    assert_eq!(set_wcalls(d, ""), MfskStatus::Ok);
    assert_eq!(get_wcalls(d).1, "");
    // a Keff-66 decode of a type-1 message teaches it
    let slot = fst4w_slot(MfskMode::Fst4w120, 120, "K1ABC FN42 37", 1500.0);
    assert_eq!(texts(&decode_i16(d, &slot)), ["K1ABC FN42 37"]);
    assert_eq!(get_wcalls(d).1, "K1ABC FN42");
    unsafe { mfsk_decoder_close(d) };

    // other modes have no list
    let ft8 = open(MfskMode::Ft8, None, None);
    assert_eq!(get_wcalls(ft8).0, MfskStatus::Unsupported);
    assert_eq!(set_wcalls(ft8, "K1ABC FN42"), MfskStatus::Unsupported);
    unsafe { mfsk_decoder_close(ft8) };
}

#[test]
fn a_message_fst4w_cannot_send_is_refused() {
    let mut msg = [0u8; 77];
    for bad in ["CQ K1ABC FN42", "hello", "K1ABC FN42 38"] {
        let t = CString::new(bad).unwrap();
        assert_eq!(
            unsafe { mfsk_fst4w_pack(t.as_ptr(), msg.as_mut_ptr()) },
            MfskStatus::DecodeFailed,
            "{bad:?}"
        );
    }
    // a 77-bit message that is not WSPR-type has no FST4W tones
    let (a, b, c) = (
        CString::new("CQ").unwrap(),
        CString::new("K1ABC").unwrap(),
        CString::new("FN42").unwrap(),
    );
    assert_eq!(
        unsafe { mfsk_pack77(a.as_ptr(), b.as_ptr(), c.as_ptr(), msg.as_mut_ptr()) },
        MfskStatus::Ok
    );
    let mut tones = [0u8; 160];
    assert_eq!(
        unsafe {
            mfsk_message_to_tones(
                MfskMode::Fst4w120 as u32,
                msg.as_ptr(),
                tones.as_mut_ptr(),
                160,
                std::ptr::null_mut(),
            )
        },
        MfskStatus::InvalidArg
    );
}

#[test]
fn every_new_mode_opens_a_decoder() {
    for (mode, ..) in W {
        let d = open(mode, None, None);
        unsafe { mfsk_decoder_close(d) };
    }
    for mode in [MfskMode::Fst4s900, MfskMode::Fst4s1800] {
        let d = open(mode, None, None);
        unsafe { mfsk_decoder_close(d) };
    }
}
