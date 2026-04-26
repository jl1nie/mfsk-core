//! Q65 FFI surface integration tests.
//!
//! Calls the public `mfsk_*` functions through their Rust signatures
//! (they are `pub extern "C" fn`, so safe to invoke as ordinary
//! functions in-crate) and validates:
//!
//! - `mfsk_encode_q65` × every sub-mode round-trips.
//! - `mfsk_q65_decode` recovers a clean Q65a30 frame.
//! - `mfsk_q65_decode_with_ap` accepts NULL hints and matches the
//!   plain path; with hints, decodes the same clean frame.
//! - `mfsk_q65_decode_fading` decodes a clean frame with tight
//!   spread parameters (Gaussian model, B90·Ts ≈ 0.05).
//! - `mfsk_q65_decode_with_ap_list` decodes a frame whose template
//!   is in the standard candidate set.
//! - The generic-handle path (`mfsk_decoder_new(MFSK_PROTOCOL_Q65A30)`
//!   + `mfsk_decode_f32`) routes Q65a30 traffic correctly.

use std::ffi::{CString, c_char};
use std::ptr;

use mfsk::{
    MfskMessageList, MfskProtocol, MfskQ65FadingModel, MfskQ65SubMode, MfskSamples, MfskStatus,
    mfsk_decode_f32, mfsk_decoder_free, mfsk_decoder_new, mfsk_encode_q65, mfsk_message_list_free,
    mfsk_q65_decode, mfsk_q65_decode_fading, mfsk_q65_decode_with_ap, mfsk_q65_decode_with_ap_list,
    mfsk_samples_free,
};

fn empty_samples() -> MfskSamples {
    MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    }
}

fn empty_list() -> MfskMessageList {
    MfskMessageList {
        items: ptr::null_mut(),
        len: 0,
        _cap: 0,
    }
}

/// Read a NUL-terminated C string from a decoded message into an
/// owned `String`. Caller still owns the underlying allocation
/// (via the parent `MfskMessageList`).
unsafe fn cstr_to_string(p: *const c_char) -> String {
    if p.is_null() {
        return String::new();
    }
    unsafe { std::ffi::CStr::from_ptr(p).to_string_lossy().into_owned() }
}

/// True if any decoded message in `list` contains the given
/// substring. Used to keep the assertions tolerant of FT4-style
/// `<...>` decorations and protocol-level whitespace.
unsafe fn list_any_contains(list: &MfskMessageList, needle: &str) -> bool {
    if list.items.is_null() || list.len == 0 {
        return false;
    }
    let slice = unsafe { std::slice::from_raw_parts(list.items, list.len) };
    slice
        .iter()
        .any(|m| unsafe { cstr_to_string(m.text) }.contains(needle))
}

/// Build a `Q65a30` message via the FFI encoder, returning the
/// resulting samples buffer. Panics if the encoder reports failure
/// (means the test's call/grid are malformed, which is a test-side
/// bug not a library one).
fn encode_q65a30(call1: &str, call2: &str, report: &str) -> MfskSamples {
    let c1 = CString::new(call1).unwrap();
    let c2 = CString::new(call2).unwrap();
    let r = CString::new(report).unwrap();
    let mut out = empty_samples();
    let st = unsafe {
        mfsk_encode_q65(
            MfskQ65SubMode::A30,
            c1.as_ptr(),
            c2.as_ptr(),
            r.as_ptr(),
            1500.0,
            &mut out,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "mfsk_encode_q65 failed");
    out
}

#[test]
fn encode_q65_roundtrips_for_every_submode() {
    let c1 = CString::new("CQ").unwrap();
    let c2 = CString::new("K1ABC").unwrap();
    let g = CString::new("FN42").unwrap();

    for &sm in &[
        MfskQ65SubMode::A30,
        MfskQ65SubMode::A60,
        MfskQ65SubMode::B60,
        MfskQ65SubMode::C60,
        MfskQ65SubMode::D60,
        MfskQ65SubMode::E60,
    ] {
        let mut out = empty_samples();
        let st =
            unsafe { mfsk_encode_q65(sm, c1.as_ptr(), c2.as_ptr(), g.as_ptr(), 1500.0, &mut out) };
        assert_eq!(st, MfskStatus::Ok, "encode failed for sub-mode {sm:?}");
        assert!(!out.samples.is_null(), "null PCM for sub-mode {sm:?}");
        // Q65 frames are 85 symbols × NSPS samples. Q65-30A:
        // NSPS=3600 → 306 000 samples (25.5 s). Q65-60A..E:
        // NSPS=7200 → 612 000 samples (51 s). Encoder returns the
        // frame-length PCM, not the slot-length PCM.
        let expected = match sm {
            MfskQ65SubMode::A30 => 85 * 3_600,
            _ => 85 * 7_200,
        };
        assert_eq!(
            out.len, expected,
            "unexpected PCM length for sub-mode {sm:?}: got {} expected {expected}",
            out.len
        );
        unsafe {
            mfsk_samples_free(&mut out);
        }
    }
}

#[test]
fn q65_plain_decode_recovers_clean_signal() {
    let mut pcm = encode_q65a30("CQ", "K1ABC", "FN42");
    let mut list = empty_list();
    let st =
        unsafe { mfsk_q65_decode(MfskQ65SubMode::A30, pcm.samples, pcm.len, 12_000, &mut list) };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        unsafe { list_any_contains(&list, "K1ABC") } && unsafe { list_any_contains(&list, "FN42") },
        "expected K1ABC + FN42 in plain Q65 decode output"
    );
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn q65_decode_with_ap_handles_null_hints() {
    // All four AP hint strings NULL → must behave like the plain
    // path (no false rejects, no crashes).
    let mut pcm = encode_q65a30("CQ", "JA1ABC", "PM95");
    let mut list = empty_list();
    let st = unsafe {
        mfsk_q65_decode_with_ap(
            MfskQ65SubMode::A30,
            pcm.samples,
            pcm.len,
            12_000,
            ptr::null(),
            ptr::null(),
            ptr::null(),
            ptr::null(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(unsafe { list_any_contains(&list, "JA1ABC") });
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn q65_decode_with_ap_uses_call1_hint() {
    let mut pcm = encode_q65a30("CQ", "JA1ABC", "PM95");
    let mut list = empty_list();
    let cq = CString::new("CQ").unwrap();
    let st = unsafe {
        mfsk_q65_decode_with_ap(
            MfskQ65SubMode::A30,
            pcm.samples,
            pcm.len,
            12_000,
            cq.as_ptr(),
            ptr::null(),
            ptr::null(),
            ptr::null(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(unsafe { list_any_contains(&list, "JA1ABC") });
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn q65_decode_fading_recovers_clean_signal() {
    let mut pcm = encode_q65a30("CQ", "K1ABC", "FN42");
    let mut list = empty_list();
    let st = unsafe {
        mfsk_q65_decode_fading(
            MfskQ65SubMode::A30,
            pcm.samples,
            pcm.len,
            12_000,
            0.05, // tight spread → near-AWGN
            MfskQ65FadingModel::Gaussian,
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        unsafe { list_any_contains(&list, "K1ABC") },
        "fast-fading FFI path must decode a clean signal"
    );
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn q65_decode_with_ap_list_picks_matching_template() {
    // Encode "K1ABC JA1ABC PM95" — that exact template lives in
    // the 206-candidate set generated for (K1ABC, JA1ABC, PM95).
    let mut pcm = encode_q65a30("K1ABC", "JA1ABC", "PM95");
    let mut list = empty_list();
    let mc = CString::new("K1ABC").unwrap();
    let hc = CString::new("JA1ABC").unwrap();
    let hg = CString::new("PM95").unwrap();
    let st = unsafe {
        mfsk_q65_decode_with_ap_list(
            MfskQ65SubMode::A30,
            pcm.samples,
            pcm.len,
            12_000,
            mc.as_ptr(),
            hc.as_ptr(),
            hg.as_ptr(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        unsafe { list_any_contains(&list, "K1ABC JA1ABC PM95") },
        "AP-list FFI path must pick the matching template"
    );
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn q65_decode_with_ap_list_returns_decode_failed_on_bad_calls() {
    // `standard_qso_codewords` rejects garbage callsigns →
    // empty candidate set → DecodeFailed status.
    let mut pcm = encode_q65a30("CQ", "K1ABC", "FN42");
    let mut list = empty_list();
    let bad = CString::new("!!!").unwrap();
    let hc = CString::new("K1ABC").unwrap();
    let st = unsafe {
        mfsk_q65_decode_with_ap_list(
            MfskQ65SubMode::A30,
            pcm.samples,
            pcm.len,
            12_000,
            bad.as_ptr(),
            hc.as_ptr(),
            ptr::null(),
            &mut list,
        )
    };
    assert_eq!(
        st,
        MfskStatus::DecodeFailed,
        "garbage calls should yield DecodeFailed without aborting"
    );
    assert_eq!(list.len, 0);
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_samples_free(&mut pcm);
    }
}

#[test]
fn generic_handle_path_decodes_q65a30() {
    // Confirms MFSK_PROTOCOL_Q65A30 routes through the
    // mfsk_decode_f32 dispatcher to mfsk_core::q65::decode_scan_default.
    let mut pcm = encode_q65a30("CQ", "JA1ABC", "PM95");
    let dec = mfsk_decoder_new(MfskProtocol::Q65a30);
    assert!(!dec.is_null());
    let mut list = empty_list();
    let st = unsafe { mfsk_decode_f32(dec, pcm.samples, pcm.len, 12_000, &mut list) };
    assert_eq!(st, MfskStatus::Ok);
    assert!(unsafe { list_any_contains(&list, "JA1ABC") });
    unsafe {
        mfsk_message_list_free(&mut list);
        mfsk_decoder_free(dec);
        mfsk_samples_free(&mut pcm);
    }
}
