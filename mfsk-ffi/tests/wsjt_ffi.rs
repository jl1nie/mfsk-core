//! WSJT-family (non-Q65) FFI surface integration tests.
//!
//! `q65_ffi.rs` covers the dedicated `mfsk_q65_*` function family;
//! this file exercises the rest of the ABI — FT8 / FT4 / WSPR / JT9 /
//! JT65 / FST4-60A through the generic `mfsk_decoder_new` +
//! `mfsk_decode_f32` / `mfsk_decode_i16` dispatch, plus the encode
//! entry points and a few negative/NULL-handling paths. Test vectors
//! mirror `examples/cpp_smoke/main.cpp` (already known-good there)
//! so a break here and a break in the C++ driver should correlate.

use std::ffi::{CString, c_char};
use std::ptr;

use mfsk::{
    MfskProtocol, MfskResultList, MfskSamples, MfskStatus, mfsk_decode_f32, mfsk_decode_i16,
    mfsk_decoder_free, mfsk_decoder_new, mfsk_encode_fst4s60, mfsk_encode_ft4, mfsk_encode_ft8,
    mfsk_encode_jt9, mfsk_encode_jt65, mfsk_encode_wspr, mfsk_result_list_free, mfsk_samples_free,
};

fn empty_samples() -> MfskSamples {
    MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    }
}

fn empty_list() -> MfskResultList {
    MfskResultList {
        items: ptr::null_mut(),
        len: 0,
        _capacity: 0,
    }
}

unsafe fn cstr_to_string(p: *const c_char) -> String {
    if p.is_null() {
        return String::new();
    }
    unsafe { std::ffi::CStr::from_ptr(p).to_string_lossy().into_owned() }
}

unsafe fn list_any_contains(list: &MfskResultList, needle: &str) -> bool {
    if list.items.is_null() || list.len == 0 {
        return false;
    }
    let slice = unsafe { std::slice::from_raw_parts(list.items, list.len) };
    slice
        .iter()
        .any(|m| unsafe { cstr_to_string(m.text.as_ptr()) }.contains(needle))
}

/// Decode `pcm` through the generic handle path and assert every
/// `needles` substring shows up somewhere in the decoded text.
fn decode_and_check(protocol: MfskProtocol, pcm: &MfskSamples, needles: &[&str]) {
    let dec = mfsk_decoder_new(protocol);
    assert!(!dec.is_null(), "mfsk_decoder_new returned null");
    let mut list = empty_list();
    let st = unsafe { mfsk_decode_f32(dec, pcm.samples, pcm.len, 12_000, ptr::null(), &mut list) };
    assert_eq!(st, MfskStatus::Ok, "decode_f32 failed for {protocol:?}");
    for needle in needles {
        assert!(
            unsafe { list_any_contains(&list, needle) },
            "expected '{needle}' in {protocol:?} decode output"
        );
    }
    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decoder_free(dec);
    }
}

#[test]
fn ft8_roundtrip_f32() {
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("JA1ABC").unwrap();
    let r = CString::new("PM95").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_ft8(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    decode_and_check(MfskProtocol::Ft8, &pcm, &["JA1ABC", "PM95"]);
    unsafe {
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn ft8_roundtrip_i16() {
    // Same signal as `ft8_roundtrip_f32`, but pushed through
    // `mfsk_decode_i16` (the path every ADC / WAV-file consumer
    // actually uses) instead of f32.
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("JA1ABC").unwrap();
    let r = CString::new("PM95").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_ft8(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);

    let f32_samples = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) };
    let i16_samples: Vec<i16> = f32_samples
        .iter()
        .map(|&s| (s * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect();

    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    assert!(!dec.is_null());
    let mut list = empty_list();
    let st = unsafe {
        mfsk_decode_i16(
            dec,
            i16_samples.as_ptr(),
            i16_samples.len(),
            12_000,
            ptr::null(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(unsafe { list_any_contains(&list, "JA1ABC") && list_any_contains(&list, "PM95") });
    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decoder_free(dec);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn ft4_roundtrip_f32() {
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("JA1ABC").unwrap();
    let r = CString::new("PM95").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_ft4(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    decode_and_check(MfskProtocol::Ft4, &pcm, &["JA1ABC", "PM95"]);
    unsafe {
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn wspr_roundtrip_f32() {
    let call = CString::new("K1ABC").unwrap();
    let grid = CString::new("FN42").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_wspr(call.as_ptr(), grid.as_ptr(), 37, 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    decode_and_check(MfskProtocol::Wspr, &pcm, &["K1ABC", "FN42"]);
    unsafe {
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn jt9_roundtrip_f32() {
    // `decode_jt9_aligned` in the FFI dispatcher fixes the search at
    // (sample 0, 1500 Hz) — the encoder call below must match.
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("K1ABC").unwrap();
    let g = CString::new("FN42").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_jt9(c1.as_ptr(), c2.as_ptr(), g.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    decode_and_check(MfskProtocol::Jt9, &pcm, &["K1ABC", "FN42"]);
    unsafe {
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn jt65_roundtrip_f32() {
    // `decode_jt65_aligned` fixes the search at (sample 0, 1270 Hz).
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("K1ABC").unwrap();
    let g = CString::new("FN42").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_jt65(c1.as_ptr(), c2.as_ptr(), g.as_ptr(), 1270.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    decode_and_check(MfskProtocol::Jt65, &pcm, &["K1ABC", "FN42"]);
    unsafe {
        mfsk_samples_free(&mut pcm);
    }
}

/// FST4-60A: outer 786 432-pt FFT over a full 60 s slot makes this
/// multi-second even in `--release`. Gated behind the same
/// `RUN_FST4_ROUNDTRIP` env var the C++ smoke driver uses (CI sets
/// it explicitly; local `cargo test` runs skip it by default). See
/// `feedback_fst4_release_build` project convention — always run
/// FST4 tests in `--release`.
#[test]
fn fst4s60_roundtrip_f32() {
    if std::env::var_os("RUN_FST4_ROUNDTRIP").is_none() {
        eprintln!("skipping fst4s60_roundtrip_f32 (set RUN_FST4_ROUNDTRIP=1 to run)");
        return;
    }
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("JA1ABC").unwrap();
    let r = CString::new("PM95").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_fst4s60(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);

    // Pad into a full 60 s slot with 1 s of leading silence, matching
    // `examples/cpp_smoke/main.cpp::test_fst4`.
    const SLOT: usize = 60 * 12_000;
    const OFFSET: usize = 12_000;
    let mut slot = vec![0.0f32; SLOT];
    let frame = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) };
    let copy_len = frame.len().min(SLOT - OFFSET);
    slot[OFFSET..OFFSET + copy_len].copy_from_slice(&frame[..copy_len]);

    let dec = mfsk_decoder_new(MfskProtocol::Fst4s60);
    assert!(!dec.is_null());
    let mut list = empty_list();
    let st = unsafe {
        mfsk_decode_f32(
            dec,
            slot.as_ptr(),
            slot.len(),
            12_000,
            ptr::null(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(unsafe { list_any_contains(&list, "JA1ABC") && list_any_contains(&list, "PM95") });
    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decoder_free(dec);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn decode_f32_null_decoder_returns_invalid_arg() {
    let mut list = empty_list();
    let st =
        unsafe { mfsk_decode_f32(ptr::null(), ptr::null(), 0, 12_000, ptr::null(), &mut list) };
    assert_eq!(st, MfskStatus::InvalidArg);
}

#[test]
fn decode_f32_null_samples_returns_invalid_arg() {
    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    assert!(!dec.is_null());
    let mut list = empty_list();
    let st = unsafe { mfsk_decode_f32(dec, ptr::null(), 0, 12_000, ptr::null(), &mut list) };
    assert_eq!(st, MfskStatus::InvalidArg);
    unsafe {
        mfsk_decoder_free(dec);
    }
}

#[test]
fn encode_ft8_bad_callsign_returns_invalid_arg() {
    // "XXX"/"Y2Z" are not packable by `pack77` — the encoder must
    // report failure rather than silently emitting garbage PCM.
    let c1 = CString::new("XXX").unwrap();
    let c2 = CString::new("Y2Z").unwrap();
    let r = CString::new("FN42").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_ft8(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::InvalidArg);
    assert!(pcm.samples.is_null());
}

#[test]
fn free_null_pointers_is_safe() {
    unsafe {
        mfsk_decoder_free(ptr::null_mut());
        mfsk_result_list_free(ptr::null_mut());
        mfsk_samples_free(ptr::null_mut());
    }
}
