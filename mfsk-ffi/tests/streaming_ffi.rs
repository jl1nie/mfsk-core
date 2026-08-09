//! `mfsk_decode_i16_streaming` integration tests (issue #246 follow-up
//! — `.on_result()` was never exposed across this FFI before this
//! function existed).
//!
//! Mirrors `wsjt_ffi.rs`'s synth-then-decode convention: no WAV
//! fixtures needed, a self-synthesised FT8 signal round-trips through
//! the streaming entry point and is checked against both the
//! callback's own delivery and the batch `MfskResultList` it still
//! populates.

use std::ffi::{CString, c_char, c_void};
use std::ptr;
use std::sync::Mutex;

use mfsk::{
    MfskProtocol, MfskResult, MfskResultList, MfskStatus, mfsk_decode_i16,
    mfsk_decode_i16_streaming, mfsk_decoder_free, mfsk_decoder_new, mfsk_encode_ft8,
    mfsk_result_list_free, mfsk_samples_free,
};

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

/// Synthesise one standard FT8 message, return it as i16 PCM (the
/// `mfsk_decode_i16_streaming`-ready shape).
fn synth_ft8_i16(call1: &str, call2: &str, report: &str, freq_hz: f32) -> Vec<i16> {
    let c1 = CString::new(call1).unwrap();
    let c2 = CString::new(call2).unwrap();
    let r = CString::new(report).unwrap();
    let mut pcm = mfsk::MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    };
    let st = unsafe { mfsk_encode_ft8(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), freq_hz, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    let f32_samples = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) };
    let i16_samples: Vec<i16> = f32_samples
        .iter()
        .map(|&s| (s * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    unsafe { mfsk_samples_free(&mut pcm) };
    i16_samples
}

/// Trampoline: `user_data` points to a `Mutex<Vec<String>>` the test
/// owns on its own stack. Collects the NUL-terminated `text` field as
/// an owned `String` per firing — the `MfskResult` pointer itself is
/// only valid for the duration of this call (documented on
/// `MfskResultCallback`), so nothing borrowed from it may outlive it.
extern "C" fn collect_cb(result: *const MfskResult, user_data: *mut c_void) {
    let collected = unsafe { &*(user_data as *const Mutex<Vec<String>>) };
    let rec = unsafe { &*result };
    let text = unsafe { cstr_to_string(rec.text.as_ptr()) };
    collected.lock().unwrap().push(text);
}

#[test]
fn streaming_callback_fires_and_matches_batch_for_single_signal() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);

    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    assert!(!dec.is_null());
    let mut list = empty_list();
    let collected: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let user_data = &collected as *const Mutex<Vec<String>> as *mut c_void;

    let st = unsafe {
        mfsk_decode_i16_streaming(
            dec,
            samples.as_ptr(),
            samples.len(),
            12_000,
            ptr::null(),
            Some(collect_cb),
            user_data,
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);

    let streamed = collected.into_inner().unwrap();
    assert!(!streamed.is_empty(), "streaming callback never fired");
    assert!(
        streamed.iter().any(|s| s.contains("JA1ABC")),
        "streamed results {streamed:?} missing JA1ABC"
    );
    assert!(unsafe { list_any_contains(&list, "JA1ABC") && list_any_contains(&list, "PM95") });
    // A single, well-separated candidate should deliver an exact match
    // between the streamed count and the batch list — no duplicate
    // firing, no revoke-less retract (STREAMING.md §3's two documented
    // contracts both collapse to this for a single-candidate decode).
    assert_eq!(
        streamed.len(),
        list.len,
        "streamed count should match batch count for one clean candidate"
    );

    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decoder_free(dec);
    }
}

#[test]
fn streaming_null_callback_behaves_like_batch_decode() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1700.0);

    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    assert!(!dec.is_null());
    let mut streaming_list = empty_list();
    let st = unsafe {
        mfsk_decode_i16_streaming(
            dec,
            samples.as_ptr(),
            samples.len(),
            12_000,
            ptr::null(),
            None, // no callback: exercises the `let Some(cb) = callback else { return }` path
            ptr::null_mut(),
            &mut streaming_list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(unsafe { list_any_contains(&streaming_list, "JA1ABC") });

    // Cross-check against the plain batch entry point on the same audio.
    let mut batch_list = empty_list();
    let st = unsafe {
        mfsk_decode_i16(
            dec,
            samples.as_ptr(),
            samples.len(),
            12_000,
            ptr::null(),
            &mut batch_list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert_eq!(streaming_list.len, batch_list.len);

    unsafe {
        mfsk_result_list_free(&mut streaming_list);
        mfsk_result_list_free(&mut batch_list);
        mfsk_decoder_free(dec);
    }
}

#[test]
fn streaming_unsupported_protocol_returns_unknown_protocol() {
    // Streaming is FT8-only so far; any other protocol must report
    // UnknownProtocol rather than silently ignoring `callback`.
    let dummy = [0i16; 16]; // non-null, non-empty: isolates the protocol check
    let dec = mfsk_decoder_new(MfskProtocol::Ft4);
    assert!(!dec.is_null());
    let mut list = empty_list();
    let st = unsafe {
        mfsk_decode_i16_streaming(
            dec,
            dummy.as_ptr(),
            dummy.len(),
            12_000,
            ptr::null(),
            None,
            ptr::null_mut(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::UnknownProtocol);
    unsafe {
        mfsk_decoder_free(dec);
    }
}

#[test]
fn streaming_null_pointers_are_rejected() {
    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    assert!(!dec.is_null());
    let mut list = empty_list();

    // Null decoder handle.
    let st = unsafe {
        mfsk_decode_i16_streaming(
            ptr::null(),
            [0i16; 4].as_ptr(),
            4,
            12_000,
            ptr::null(),
            None,
            ptr::null_mut(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::InvalidArg);

    // Null samples pointer.
    let st = unsafe {
        mfsk_decode_i16_streaming(
            dec,
            ptr::null(),
            0,
            12_000,
            ptr::null(),
            None,
            ptr::null_mut(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::InvalidArg);

    unsafe {
        mfsk_decoder_free(dec);
    }
}
