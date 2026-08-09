//! `mfsk_decode_options_set_*` integration tests (FFI builder-parity
//! pass, issue #162 follow-up — mfsk-ffi's `MfskDecodeOptions` had not
//! grown a single setter since its creation, issue #205, despite its
//! own doc comment anticipating exactly that).
//!
//! `strictness`/`eq_mode`/`freq_hint` are already exhaustively tested
//! at the `mfsk_core` level (this pass only wires existing,
//! already-verified Rust builder methods through to C) — the
//! round-trip tests here confirm each setter reaches the real decode
//! call and doesn't break a clean baseline, not a fresh
//! accept/reject-threshold sweep or candidate-promotion proof. A
//! `freq_hint`-specific behavioural proof (promotion under a
//! `max_cand` cap) turned out not to be cheap to construct — see that
//! test's own doc comment for what got in the way.

use std::ffi::CString;
use std::ptr;

use mfsk::{
    MfskDecodeDepth, MfskEqMode, MfskProtocol, MfskResultList, MfskSamples, MfskStatus,
    MfskStrictness, mfsk_decode_i16, mfsk_decode_options_free, mfsk_decode_options_new,
    mfsk_decode_options_set_eq_mode, mfsk_decode_options_set_freq_hint,
    mfsk_decode_options_set_sic_early, mfsk_decode_options_set_sic_rounds,
    mfsk_decode_options_set_strictness, mfsk_decoder_free, mfsk_decoder_new, mfsk_encode_ft8,
    mfsk_result_list_free, mfsk_samples_free,
};

fn empty_list() -> MfskResultList {
    MfskResultList {
        items: ptr::null_mut(),
        len: 0,
        _capacity: 0,
    }
}

unsafe fn cstr_to_string(p: *const std::ffi::c_char) -> String {
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

fn synth_ft8_i16(call1: &str, call2: &str, report: &str, freq_hz: f32) -> Vec<i16> {
    synth_ft8_i16_scaled(call1, call2, report, freq_hz, 32_767.0)
}

/// `mfsk_encode_ft8` synthesises at amplitude 1.0 (`tones_to_f32`'s own
/// full-scale convention) — fine standalone, but summing two full-scale
/// i16 signals in [`mix`] would clip both into unrecoverable noise.
/// `scale` lets [`mix`]-bound callers leave headroom (e.g. `32_767.0 *
/// 0.45` per signal keeps a two-signal sum comfortably inside i16
/// range).
fn synth_ft8_i16_scaled(
    call1: &str,
    call2: &str,
    report: &str,
    freq_hz: f32,
    scale: f32,
) -> Vec<i16> {
    let c1 = CString::new(call1).unwrap();
    let c2 = CString::new(call2).unwrap();
    let r = CString::new(report).unwrap();
    let mut pcm = MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    };
    let st = unsafe { mfsk_encode_ft8(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), freq_hz, &mut pcm) };
    assert_eq!(st, MfskStatus::Ok);
    let f32_samples = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) };
    let i16_samples: Vec<i16> = f32_samples
        .iter()
        .map(|&s| (s * scale).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    unsafe { mfsk_samples_free(&mut pcm) };
    i16_samples
}

/// Mix two synthesised FT8 signals into one buffer (max-hold combine —
/// matches `subtract.rs`'s own test convention of summing i32 then
/// clamping).
fn mix(a: &[i16], b: &[i16]) -> Vec<i16> {
    let len = a.len().max(b.len());
    (0..len)
        .map(|i| {
            let va = *a.get(i).unwrap_or(&0) as i32;
            let vb = *b.get(i).unwrap_or(&0) as i32;
            (va + vb).clamp(-32_768, 32_767) as i16
        })
        .collect()
}

/// `mfsk_decode_options_set_strictness`/`_set_eq_mode` round-trip:
/// every value still decodes the clean baseline signal identically —
/// confirms the setter reaches the real decode call (wrong wiring,
/// e.g. a swapped enum mapping, would show up as *some* combination
/// failing to decode a clean signal that easily clears every
/// strictness tier).
#[test]
fn strictness_and_eq_mode_setters_reach_decode_without_breaking_clean_signal() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);

    for strictness in [
        MfskStrictness::Strict,
        MfskStrictness::Normal,
        MfskStrictness::Deep,
    ] {
        for eq_mode in [MfskEqMode::Off, MfskEqMode::Local] {
            let opts = mfsk_decode_options_new(200.0, 3_000.0, 2.0, 50, MfskDecodeDepth::BpAllOsd);
            assert!(!opts.is_null());
            unsafe {
                assert_eq!(
                    mfsk_decode_options_set_strictness(opts, strictness),
                    MfskStatus::Ok
                );
                assert_eq!(
                    mfsk_decode_options_set_eq_mode(opts, eq_mode),
                    MfskStatus::Ok
                );
            }

            let dec = mfsk_decoder_new(MfskProtocol::Ft8);
            let mut list = empty_list();
            let st = unsafe {
                mfsk_decode_i16(
                    dec,
                    samples.as_ptr(),
                    samples.len(),
                    12_000,
                    opts,
                    &mut list,
                )
            };
            assert_eq!(
                st,
                MfskStatus::Ok,
                "decode failed for strictness={strictness:?} eq_mode={eq_mode:?}"
            );
            assert!(
                unsafe { list_any_contains(&list, "JA1ABC") },
                "clean signal not recovered for strictness={strictness:?} eq_mode={eq_mode:?}"
            );

            unsafe {
                mfsk_result_list_free(&mut list);
                mfsk_decoder_free(dec);
                mfsk_decode_options_free(opts);
            }
        }
    }
}

/// `mfsk_decode_options_set_freq_hint` round-trip: reaches the decode
/// call, doesn't exclude non-matching candidates (it's documented as
/// promotion-only, not a filter), and setting it to a signal's own
/// frequency doesn't break decoding that signal.
///
/// A stronger behavioural proof (freq_hint actually changing *which*
/// candidate survives a `max_cand` cap, mirroring this session's
/// `.sic_early()` recall proof) turned out not to be cheap to
/// construct: `max_cand=1` was found, while writing this test, to
/// drop every candidate outright regardless of hinting (a real, but
/// pre-existing and unrelated property — not investigated further,
/// out of scope for this pass), and a 3-signal mix under `max_cand=2`
/// didn't reliably preserve exactly 2 of the 3 real signals (mixing
/// three full-strength FT8 signals introduces coarse-sync-visible
/// cross-artifacts that can outrank a real signal, unrelated to
/// `freq_hint` itself). Both are `max_cand`/candidate-ranking
/// questions, not `freq_hint` bugs — worth a separate look sometime,
/// not blocking this builder-parity pass.
#[test]
fn freq_hint_setter_reaches_decode_and_does_not_exclude_candidates() {
    let sig_a = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);
    let sig_b = synth_ft8_i16_scaled("CQ", "K1ABC", "FN42", 2200.0, 32_767.0 * 0.5);
    let mixed = mix(&sig_a, &sig_b);

    for hint_freq in [1500.0, 2200.0, 2700.0 /* matches neither */] {
        let opts = mfsk_decode_options_new(200.0, 3_000.0, 2.0, 50, MfskDecodeDepth::BpAllOsd);
        assert!(!opts.is_null());
        unsafe {
            assert_eq!(
                mfsk_decode_options_set_freq_hint(opts, hint_freq),
                MfskStatus::Ok
            );
        }
        let dec = mfsk_decoder_new(MfskProtocol::Ft8);
        let mut list = empty_list();
        let st =
            unsafe { mfsk_decode_i16(dec, mixed.as_ptr(), mixed.len(), 12_000, opts, &mut list) };
        assert_eq!(
            st,
            MfskStatus::Ok,
            "decode failed with freq_hint={hint_freq}"
        );
        assert!(
            unsafe { list_any_contains(&list, "JA1ABC") && list_any_contains(&list, "K1ABC") },
            "freq_hint={hint_freq} should not exclude either candidate (ample max_cand)"
        );
        unsafe {
            mfsk_result_list_free(&mut list);
            mfsk_decoder_free(dec);
            mfsk_decode_options_free(opts);
        }
    }
}

/// `mfsk_decode_options_set_sic_rounds`/`_set_sic_early`'s real
/// behavioural effect: a weak signal masked by a much stronger nearby
/// one is invisible to the default single-pass strategy but recovered
/// once the strong signal is subtracted first. Same amplitude/
/// frequency choices as `mfsk-core`'s own
/// `ft8::subtract::tests::subtract_reveals_hidden_signal` (already
/// proven to reproduce this masking reliably) — reused rather than
/// re-derived.
/// Minimal RIFF/WAVE mono-i16 loader, standalone (mfsk-ffi's test
/// crate can't reach `mfsk-core/tests/common/mod.rs`'s version —
/// different crate). Path convention matches `CLAUDE.md`'s own
/// documented rule for non-`mfsk-core`-internal test code:
/// `CARGO_MANIFEST_DIR`-relative into `embedded-poc/assets/`, never a
/// hardcoded absolute path.
fn load_wav_i16(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    assert!(
        bytes.len() >= 12 && &bytes[0..4] == b"RIFF" && &bytes[8..12] == b"WAVE",
        "{path}: not a RIFF/WAVE file"
    );
    let mut offset = 12usize;
    let mut data: Option<&[u8]> = None;
    while offset + 8 <= bytes.len() {
        let id = &bytes[offset..offset + 4];
        let size = u32::from_le_bytes(bytes[offset + 4..offset + 8].try_into().unwrap()) as usize;
        let body = offset + 8;
        if id == b"data" {
            data = Some(&bytes[body..body + size]);
        }
        offset = body + size + (size % 2);
    }
    let data = data.unwrap_or_else(|| panic!("{path}: missing data chunk"));
    data.chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

/// `mfsk_decode_options_set_sic_early`/`_set_sic_rounds`'s real
/// behavioural effect, using the same real busy-band recording (and
/// the same default-vs-SIC recall gap, ~14-15 vs ~20-22 stations)
/// this session's WASM/streaming investigation (issue #246) measured
/// repeatedly on the Rust side — reused here rather than
/// hand-constructing a synthetic masked-signal scenario, which turned
/// out to be finicky (see git history of this file: a two-signal
/// strong/weak construction either decoded both with or without SIC,
/// or neither, across every amplitude/frequency combination tried;
/// masking a single weak signal behind one strong one apparently
/// isn't as reliably reproducible as this real multi-station
/// recording's own natural overlap).
#[test]
fn sic_rounds_and_sic_early_recover_more_stations_than_default() {
    let audio = load_wav_i16(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/qso3_busy.wav"
    ));

    let decode_with = |configure: Option<fn(*mut mfsk::MfskDecodeOptions)>| -> usize {
        let opts = mfsk_decode_options_new(200.0, 3_000.0, 2.0, 50, MfskDecodeDepth::BpAllOsd);
        assert!(!opts.is_null());
        if let Some(f) = configure {
            f(opts);
        }
        let dec = mfsk_decoder_new(MfskProtocol::Ft8);
        let mut list = empty_list();
        let st =
            unsafe { mfsk_decode_i16(dec, audio.as_ptr(), audio.len(), 12_000, opts, &mut list) };
        assert_eq!(st, MfskStatus::Ok);
        let n = list.len;
        unsafe {
            mfsk_result_list_free(&mut list);
            mfsk_decoder_free(dec);
            mfsk_decode_options_free(opts);
        }
        n
    };

    let default_n = decode_with(None);
    let sic_early_n = decode_with(Some(|opts| unsafe {
        assert_eq!(mfsk_decode_options_set_sic_early(opts), MfskStatus::Ok);
    }));
    let sic_rounds_n = decode_with(Some(|opts| unsafe {
        assert_eq!(mfsk_decode_options_set_sic_rounds(opts, 3), MfskStatus::Ok);
    }));

    assert!(
        sic_early_n > default_n,
        "sic_early ({sic_early_n}) should recover more qso3_busy stations than default ({default_n})"
    );
    assert!(
        sic_rounds_n > default_n,
        "sic_rounds(3) ({sic_rounds_n}) should recover more qso3_busy stations than default ({default_n})"
    );
}
