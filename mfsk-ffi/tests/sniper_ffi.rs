//! `mfsk_decode_i16_sniper` / `mfsk_decode_f32_sniper` — the
//! single-frequency-target entry points (issue #249).
//!
//! What these tests are for, in order of what would actually break:
//!
//! 1. **FT4 and FST4 can take an AP hint through C at all.** That is
//!    the reason the issue exists: `mfsk_decode_options_set_ap_hint`
//!    reaches the wide-band decoder for FT8 only, so before this entry
//!    point a C caller had no way to hint FT4/FST4. A wiring mistake
//!    here (hint dropped, wrong protocol arm) is invisible on a clean
//!    signal, so the AP tests assert the hint *reaches* the decoder and
//!    leaves a decodable signal decodable — the 1-3 dB the hint is
//!    worth is `mfsk_core`'s own property, tested there.
//! 2. The options fields that do not apply are ignored rather than
//!    fatal — a caller reusing one handle across both entry points is
//!    the expected shape, not an error.
//! 3. Protocols with no sniper mode say so instead of decoding
//!    something wrong.

use std::ffi::CString;
use std::ptr;

use mfsk::{
    MfskDecodeDepth, MfskProtocol, MfskResultList, MfskSamples, MfskStatus, mfsk_decode_f32_sniper,
    mfsk_decode_i16_sniper, mfsk_decode_options_free, mfsk_decode_options_new,
    mfsk_decode_options_set_ap_hint, mfsk_decode_options_set_freq_hint,
    mfsk_decode_options_set_sic_rounds, mfsk_decoder_free, mfsk_decoder_new, mfsk_encode_ft4,
    mfsk_encode_ft8, mfsk_result_list_free, mfsk_samples_free,
};

fn empty_list() -> MfskResultList {
    MfskResultList {
        items: ptr::null_mut(),
        len: 0,
        _capacity: 0,
    }
}

unsafe fn list_contains(list: &MfskResultList, needle: &str) -> bool {
    if list.items.is_null() || list.len == 0 {
        return false;
    }
    let slice = unsafe { std::slice::from_raw_parts(list.items, list.len) };
    slice.iter().any(|m| {
        let s = unsafe { std::ffi::CStr::from_ptr(m.text.as_ptr()) }.to_string_lossy();
        s.contains(needle)
    })
}

/// `encode`: one of the `mfsk_encode_*` functions, all of which share
/// the `(call1, call2, report, freq, out)` shape and synthesise at
/// amplitude 1.0.
fn synth(
    encode: unsafe extern "C" fn(
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        f32,
        *mut MfskSamples,
    ) -> MfskStatus,
    call1: &str,
    call2: &str,
    report: &str,
    freq_hz: f32,
) -> Vec<i16> {
    let c1 = CString::new(call1).unwrap();
    let c2 = CString::new(call2).unwrap();
    let r = CString::new(report).unwrap();
    let mut pcm = MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    };
    let st = unsafe { encode(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), freq_hz, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    let f32_samples = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) };
    let out: Vec<i16> = f32_samples
        .iter()
        .map(|&s| (s * 32_767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    unsafe { mfsk_samples_free(&mut pcm) };
    out
}

const FREQ: f32 = 1_200.0;

#[test]
fn ft8_sniper_decodes_at_the_target_frequency() {
    let audio = synth(mfsk_encode_ft8, "JL1NIE", "VK3NV", "-12", FREQ);
    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    let mut list = empty_list();
    let st = unsafe {
        mfsk_decode_i16_sniper(
            dec,
            audio.as_ptr(),
            audio.len(),
            12_000,
            FREQ,
            ptr::null(),
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        unsafe { list_contains(&list, "JL1NIE") },
        "sniper at the signal's own frequency should decode it"
    );
    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decoder_free(dec);
    }
}

/// The f32 entry point is the same decode behind a different input
/// convention — same signal, same result.
#[test]
fn f32_and_i16_sniper_agree() {
    let audio = synth(mfsk_encode_ft8, "JL1NIE", "VK3NV", "-12", FREQ);
    let as_f32: Vec<f32> = audio.iter().map(|&s| s as f32 / 32_767.0).collect();
    let dec = mfsk_decoder_new(MfskProtocol::Ft8);

    let mut a = empty_list();
    let mut b = empty_list();
    unsafe {
        assert_eq!(
            mfsk_decode_i16_sniper(
                dec,
                audio.as_ptr(),
                audio.len(),
                12_000,
                FREQ,
                ptr::null(),
                &mut a
            ),
            MfskStatus::Ok
        );
        assert_eq!(
            mfsk_decode_f32_sniper(
                dec,
                as_f32.as_ptr(),
                as_f32.len(),
                12_000,
                FREQ,
                ptr::null(),
                &mut b
            ),
            MfskStatus::Ok
        );
        assert!(list_contains(&a, "JL1NIE"));
        assert_eq!(a.len, b.len, "same audio, same decode count");
        assert!(list_contains(&b, "JL1NIE"));
        mfsk_result_list_free(&mut a);
        mfsk_result_list_free(&mut b);
        mfsk_decoder_free(dec);
    }
}

/// **The point of #249.** `mfsk_decode_options_set_ap_hint` is accepted
/// on an FT4 decode here; through `mfsk_decode_i16` it would be
/// silently dropped, because `SupportsWideBandAp` is FT8-only.
#[test]
fn ft4_sniper_accepts_an_ap_hint() {
    let audio = synth(mfsk_encode_ft4, "JL1NIE", "VK3NV", "-12", FREQ);
    let dec = mfsk_decoder_new(MfskProtocol::Ft4);
    let opts = mfsk_decode_options_new(200.0, 3_000.0, 1.2, 8, MfskDecodeDepth::BpAllOsd);

    let call1 = CString::new("JL1NIE").unwrap();
    let call2 = CString::new("VK3NV").unwrap();
    assert_eq!(
        unsafe {
            mfsk_decode_options_set_ap_hint(
                opts,
                call1.as_ptr(),
                call2.as_ptr(),
                ptr::null(),
                ptr::null(),
            )
        },
        MfskStatus::Ok
    );

    let mut list = empty_list();
    let st = unsafe {
        mfsk_decode_i16_sniper(
            dec,
            audio.as_ptr(),
            audio.len(),
            12_000,
            FREQ,
            opts,
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        unsafe { list_contains(&list, "JL1NIE") },
        "an AP hint naming the station on air must not stop it decoding"
    );
    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decode_options_free(opts);
        mfsk_decoder_free(dec);
    }
}

/// Options built for the wide-band entry point are accepted here with
/// their inapplicable fields ignored — the convention this crate
/// already follows for `sic_early` on FT4.
#[test]
fn wide_band_only_options_are_ignored_not_fatal() {
    let audio = synth(mfsk_encode_ft8, "JL1NIE", "VK3NV", "-12", FREQ);
    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    let opts = mfsk_decode_options_new(200.0, 3_000.0, 2.0, 8, MfskDecodeDepth::BpAllOsd);
    unsafe {
        // Both are `DecodeRequest`-only knobs: a sniper request has no
        // freq_hint (the target is the hint) and no SIC strategy.
        assert_eq!(
            mfsk_decode_options_set_freq_hint(opts, 2_500.0),
            MfskStatus::Ok
        );
        assert_eq!(mfsk_decode_options_set_sic_rounds(opts, 3), MfskStatus::Ok);
    }

    let mut list = empty_list();
    let st = unsafe {
        mfsk_decode_i16_sniper(
            dec,
            audio.as_ptr(),
            audio.len(),
            12_000,
            FREQ,
            opts,
            &mut list,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "inapplicable options must not fail");
    assert!(
        unsafe { list_contains(&list, "JL1NIE") },
        "a freq_hint pointing 1.3 kHz away is ignored, not obeyed"
    );
    unsafe {
        mfsk_result_list_free(&mut list);
        mfsk_decode_options_free(opts);
        mfsk_decoder_free(dec);
    }
}

#[test]
fn protocols_without_a_sniper_mode_are_rejected() {
    let audio = vec![0i16; 120 * 12_000];
    for p in [MfskProtocol::Wspr, MfskProtocol::Jt9, MfskProtocol::Q65a30] {
        let dec = mfsk_decoder_new(p);
        let mut list = empty_list();
        let st = unsafe {
            mfsk_decode_i16_sniper(
                dec,
                audio.as_ptr(),
                audio.len(),
                12_000,
                FREQ,
                ptr::null(),
                &mut list,
            )
        };
        assert_eq!(
            st,
            MfskStatus::UnknownProtocol,
            "{p:?} has no single-frequency mode and should say so"
        );
        unsafe { mfsk_decoder_free(dec) };
    }
}

#[test]
fn null_arguments_are_rejected() {
    let dec = mfsk_decoder_new(MfskProtocol::Ft8);
    let mut list = empty_list();
    let audio = vec![0i16; 15 * 12_000];
    unsafe {
        assert_eq!(
            mfsk_decode_i16_sniper(
                ptr::null(),
                audio.as_ptr(),
                audio.len(),
                12_000,
                FREQ,
                ptr::null(),
                &mut list
            ),
            MfskStatus::InvalidArg
        );
        assert_eq!(
            mfsk_decode_i16_sniper(dec, ptr::null(), 0, 12_000, FREQ, ptr::null(), &mut list),
            MfskStatus::InvalidArg
        );
        assert_eq!(
            mfsk_decode_f32_sniper(dec, ptr::null(), 0, 12_000, FREQ, ptr::null(), &mut list),
            MfskStatus::InvalidArg
        );
        mfsk_decoder_free(dec);
    }
}
