//! The v2 decode surface: handle, size-versioned params, rows into
//! caller memory.
//!
//! Three things here would each have been a real defect shipped:
//!
//! 1. **Hashed callsigns could never resolve over this ABI.** Every
//!    decode built a fresh empty `CallsignHashTable`, so a `<...>`
//!    reference had nothing to look up — for any protocol, in any call,
//!    since the ABI was written. A table is only worth anything if it
//!    outlives the slot that populated it, which is why it is on the
//!    handle now.
//! 2. **The float entry point lost dynamic range at 12 kHz.** It
//!    normalised before quantising only when a resample was needed, so
//!    a quiet buffer — a USB radio adapter at a low Windows volume,
//!    the case that normalisation exists for — was worse off at the
//!    rate that needed no work.
//! 3. **Unsupported options were dropped in silence.** `ap_hint`
//!    reached FT8 only, `sic_rounds` FT8+FT4, `strictness` was a no-op
//!    on FST4, and `depth` was silently *upgraded* rather than ignored.

use std::ffi::CString;

use mfsk::*;

const FS: u32 = 12_000;

/// A row the library will overwrite. `size` is the caller's contract;
/// everything else is don't-care.
fn blank_row() -> MfskDecode {
    MfskDecode {
        size: std::mem::size_of::<MfskDecode>() as u32,
        mode: MfskMode::Ft8,
        text: [0; MFSK_DECODE_TEXT_LEN],
        freq_hz: 0.0,
        dt_sec: 0.0,
        snr_db: 0.0,
        sync_score: 0.0,
        sync_cv: 0.0,
        hard_errors: 0,
        info_bits: 0,
        pass: 0,
        flags: 0,
    }
}

fn params(mode: MfskMode) -> MfskDecodeParams {
    let mut p = std::mem::MaybeUninit::<MfskDecodeParams>::zeroed();
    assert_eq!(
        unsafe { mfsk_decode_params_init(mode as u32, p.as_mut_ptr()) },
        MfskStatus::Ok
    );
    unsafe { p.assume_init() }
}

fn open(mode: MfskMode, p: Option<&MfskDecodeParams>) -> *mut MfskDecodeSession {
    let mut st = MfskStatus::Internal;
    let d = unsafe {
        mfsk_session_open(
            mode as u32,
            p.map(|p| p as *const _).unwrap_or(std::ptr::null()),
            &mut st,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "{mode:?} failed to open");
    assert!(!d.is_null());
    d
}

fn decode(dec: *mut MfskDecodeSession, audio: &[i16]) -> Vec<String> {
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_i16(
            dec,
            audio.as_ptr(),
            audio.len(),
            FS,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    rows[..n]
        .iter()
        .map(|r| {
            let b: &[u8] =
                unsafe { std::slice::from_raw_parts(r.text.as_ptr() as *const u8, r.text.len()) };
            let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
            String::from_utf8_lossy(&b[..end]).into_owned()
        })
        .collect()
}

fn ft8_slot(msg: &[u8; 77], f0: f32) -> Vec<i16> {
    let mut slot = vec![0i16; 15 * FS as usize];
    let tones = mfsk_core::engine::tx::message_to_tones::<mfsk_core::ft8::Ft8>(msg);
    let wave =
        mfsk_core::engine::tx::synthesize_i16::<mfsk_core::ft8::Ft8>(&tones, 12_000, f0, 8_000);
    let start = (0.5 * FS as f32) as usize;
    for (i, s) in wave.iter().enumerate() {
        if let Some(d) = slot.get_mut(start + i) {
            *d = d.saturating_add(*s);
        }
    }
    slot
}

/// The defect the handle exists for.
///
/// A type-4 message carries one non-standard callsign in full and a
/// **hashed** reference to the standard one — `<...> JA1ABC/QRP`. The
/// bracketed half cannot be expanded without a table that has seen
/// `VK3NV` before, and before the handle no table ever had: every
/// decode call built a fresh empty one.
#[test]
fn a_hashed_callsign_resolves_once_the_handle_has_seen_it() {
    let nonstd = "JA1ABC/QRP";
    let hashed = "VK3NV";
    let msg = mfsk_core::msg::wsjt77::pack77_type4(nonstd, hashed, "", false)
        .expect("pack a type-4 message");
    let audio = ft8_slot(&msg, 1500.0);

    // Cold handle: the reference cannot be resolved, and the row says so.
    let cold = open(MfskMode::Ft8, None);
    let before = decode(cold, &audio);
    assert!(!before.is_empty(), "fixture did not decode at all");
    assert!(
        before.iter().all(|t| !t.contains(hashed)),
        "a cold table should not be able to expand the hash: {before:?}"
    );
    assert!(
        before.iter().any(|t| t.contains("<...>")),
        "the unexpanded form is what a cold table produces: {before:?}"
    );
    unsafe { mfsk_session_close(cold) };

    // Teach the handle the callsign, exactly as a band map would.
    let warm = open(MfskMode::Ft8, None);
    let c = CString::new(hashed).unwrap();
    assert_eq!(
        unsafe { mfsk_session_add_callsign(warm, c.as_ptr()) },
        MfskStatus::Ok
    );
    let after = decode(warm, &audio);
    assert!(
        after.iter().any(|t| t.contains(hashed)),
        "the handle knows {hashed} and still could not resolve it: {after:?}"
    );
    // And the row says the text needed the table.
    unsafe { mfsk_session_close(warm) };
    let warm = open(MfskMode::Ft8, None);
    let c = CString::new(hashed).unwrap();
    unsafe { mfsk_session_add_callsign(warm, c.as_ptr()) };
    let mut rows = [blank_row(); 8];
    let mut n = 0usize;
    unsafe {
        mfsk_session_decode_i16(
            warm,
            audio.as_ptr(),
            audio.len(),
            FS,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert!(n > 0);
    assert_ne!(
        rows[0].flags & MFSK_DECODE_FLAG_HASH_RESOLVED,
        0,
        "the row should flag that the hash table was what made the text readable"
    );
    unsafe { mfsk_session_close(warm) };
}

/// And it learns across slots on its own, which is the half a caller
/// gets for free.
#[test]
fn the_table_carries_from_one_slot_to_the_next() {
    // Slot 1 names VK3NV in full (a standard type-1 exchange) …
    let naming = mfsk_core::msg::wsjt77::pack77("CQ", "VK3NV", "QF22").expect("pack cq");
    // … slot 2 refers to it only by hash.
    let referring =
        mfsk_core::msg::wsjt77::pack77_type4("JA1ABC/QRP", "VK3NV", "", false).expect("pack reply");

    let dec = open(MfskMode::Ft8, None);
    let first = decode(dec, &ft8_slot(&naming, 1500.0));
    assert!(
        first.iter().any(|t| t.contains("VK3NV")),
        "first slot did not decode: {first:?}"
    );
    let second = decode(dec, &ft8_slot(&referring, 1700.0));
    assert!(
        second.iter().any(|t| t.contains("VK3NV")),
        "slot 2 should resolve what slot 1 named: {second:?}"
    );
    unsafe { mfsk_session_close(dec) };
}

/// Asking a mode for something it does not have fails at open, with a
/// message naming the mode — rather than being dropped in silence at
/// decode time, which is what the pre-v2 ABI did with six of its eleven
/// options.
#[test]
fn unsupported_options_are_refused_not_dropped() {
    type Case<'a> = (MfskMode, &'a dyn Fn(&mut MfskDecodeParams), &'a str);
    let cases: [Case; 4] = [
        (MfskMode::Fst4s60, &|p| p.sic_rounds = 2, "SIC on FST4"),
        (MfskMode::Ft4, &|p| p.sic_early = true, "sic_early on FT4"),
        (
            MfskMode::Ft4,
            &|p| p.search_hz = 250.0,
            "a narrow search on FT4",
        ),
        (
            MfskMode::Fst4s15,
            &|p| p.search_hz = 250.0,
            "a narrow search on FST4",
        ),
    ];
    for (mode, mutate, what) in cases {
        let mut p = params(mode);
        mutate(&mut p);
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_session_open(mode as u32, &p, &mut st) };
        assert!(d.is_null(), "{what} should not open a handle");
        assert_eq!(st, MfskStatus::Unsupported, "{what}");
    }
}

/// `tx_freq_hz` is FT8's alone (`MFSK_CAP_TX_FREQ`). The Rust builder
/// accepts the call on every protocol and the others never read it, which
/// is exactly the accepted-and-ignored shape this ABI refuses — so the
/// claim in `mfsk-core/tests/registry_caps.rs`, which has no marker trait
/// behind it, is pinned here by behaviour.
#[test]
fn tx_freq_is_refused_off_ft8() {
    for mode in [MfskMode::Ft4, MfskMode::Fst4s15, MfskMode::Fst4s60] {
        let mut p = params(mode);
        p.tx_freq_hz = 1_500.0;
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_session_open(mode as u32, &p, &mut st) };
        assert!(d.is_null(), "{mode:?} has no transmit frequency to use");
        assert_eq!(st, MfskStatus::Unsupported, "{mode:?}");
    }

    // FT8 takes it on the wide-band search…
    let mut p = params(MfskMode::Ft8);
    assert!(p.tx_freq_hz.is_nan(), "unset is NaN, as freq_hint_hz");
    p.tx_freq_hz = 1_500.0;
    unsafe { mfsk_session_close(open(MfskMode::Ft8, Some(&p))) };

    // …but not on the narrow-band one, which never reads it.
    p.search_hz = 250.0;
    p.freq_hint_hz = 1_500.0;
    let mut st = MfskStatus::Ok;
    assert!(unsafe { mfsk_session_open(MfskMode::Ft8 as u32, &p, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::Unsupported);
}

/// The blanker is FST4's alone, and its numbers are validated rather than
/// reinterpreted: the engine clamps a percentage past 25 and reads any
/// step but 1 or 2 as 5, and a C caller who typed 30 or 3 should be told.
#[test]
fn the_noise_blanker_is_fst4s_and_its_numbers_are_checked() {
    for mode in [MfskMode::Ft8, MfskMode::Ft4] {
        for mutate in [
            (|p: &mut MfskDecodeParams| p.nb_percent = 2) as fn(&mut MfskDecodeParams),
            |p| {
                p.nb_sweep_step = 5;
                p.nb_ftol_hz = 20.0;
            },
        ] {
            let mut p = params(mode);
            mutate(&mut p);
            let mut st = MfskStatus::Ok;
            assert!(unsafe { mfsk_session_open(mode as u32, &p, &mut st) }.is_null());
            assert_eq!(st, MfskStatus::Unsupported, "{mode:?}");
        }
    }

    type Case<'a> = (&'a dyn Fn(&mut MfskDecodeParams), &'a str);
    let bad: [Case; 4] = [
        (&|p| p.nb_percent = 26, "26 % is past the GUI's range"),
        (&|p| p.nb_sweep_step = 3, "a step of 3"),
        (
            &|p| p.nb_sweep_step = 5,
            "a sweep with no window (nb_ftol_hz = 0)",
        ),
        (
            &|p| {
                p.nb_sweep_step = 5;
                p.nb_ftol_hz = f32::NAN;
            },
            "a sweep with a NaN window",
        ),
    ];
    for (mutate, what) in bad {
        let mut p = params(MfskMode::Fst4s15);
        mutate(&mut p);
        let mut st = MfskStatus::Ok;
        assert!(
            unsafe { mfsk_session_open(MfskMode::Fst4s15 as u32, &p, &mut st) }.is_null(),
            "{what} should not open"
        );
        assert_eq!(st, MfskStatus::Unsupported, "{what}");
    }

    // The defaults, and each accepted shape, open on every FST4 sub-mode.
    for mode in [
        MfskMode::Fst4s15,
        MfskMode::Fst4s30,
        MfskMode::Fst4s60,
        MfskMode::Fst4s120,
        MfskMode::Fst4s300,
    ] {
        let p = params(mode);
        assert_eq!((p.nb_percent, p.nb_sweep_step), (0, 0), "off by default");
        unsafe { mfsk_session_close(open(mode, Some(&p))) };
        for pct in [0u8, 2, 25] {
            let mut p = params(mode);
            p.nb_percent = pct;
            unsafe { mfsk_session_close(open(mode, Some(&p))) };
        }
        for step in [1u8, 2, 5] {
            let mut p = params(mode);
            p.nb_sweep_step = step;
            p.nb_ftol_hz = 20.0;
            unsafe { mfsk_session_close(open(mode, Some(&p))) };
        }
    }
}

/// A struct from before these fields existed — its `size` ends at
/// `search_hz` — must read as "unset", not as whatever bytes follow it.
/// The size-versioned contract is what lets the ABI grow at all.
#[test]
fn an_older_params_struct_leaves_the_new_fields_at_their_defaults() {
    use std::mem::{offset_of, size_of};
    let mut p = params(MfskMode::Ft8);
    // The struct as a pre-#466 header declared it.
    p.size = (offset_of!(MfskDecodeParams, search_hz) + size_of::<f32>()) as u32;
    // Bytes past `size` are garbage a short caller never wrote. `nb_percent`
    // is the tell: FT8 refuses a blanker, so if the tail were read the open
    // below would fail.
    p.tx_freq_hz = 1_500.0;
    p.nb_percent = 9;
    let mut st = MfskStatus::Ok;
    let d = unsafe { mfsk_session_open(MfskMode::Ft8 as u32, &p, &mut st) };
    assert!(!d.is_null(), "the garbage tail must not be read: {st:?}");
    assert_eq!(st, MfskStatus::Ok);
    unsafe { mfsk_session_close(d) };
}

/// A capability the mode *does* have must still be accepted, or the
/// check above is just refusing everything.
#[test]
fn supported_options_are_accepted() {
    let mut p = params(MfskMode::Ft8);
    p.sic_early = true;
    unsafe { mfsk_session_close(open(MfskMode::Ft8, Some(&p))) };

    let mut p = params(MfskMode::Ft4);
    p.sic_rounds = 2;
    unsafe { mfsk_session_close(open(MfskMode::Ft4, Some(&p))) };

    // Wide-band AP reaches FT4 and FST4 now, so a hint must open there.
    for mode in [MfskMode::Ft4, MfskMode::Fst4s60] {
        let mut p = params(mode);
        p.has_ap_hint = true;
        p.ap_call1[..2].copy_from_slice(&[b'C' as _, b'Q' as _]);
        unsafe { mfsk_session_close(open(mode, Some(&p))) };
    }

    // FT8's sniper needs a carrier to aim at, not just a width.
    let mut p = params(MfskMode::Ft8);
    p.search_hz = 250.0;
    let mut st = MfskStatus::Ok;
    assert!(
        unsafe { mfsk_session_open(MfskMode::Ft8 as u32, &p, &mut st) }.is_null(),
        "search_hz without freq_hint_hz says how wide but not where"
    );
    assert_eq!(st, MfskStatus::Unsupported);
    p.freq_hint_hz = 1500.0;
    unsafe { mfsk_session_close(open(MfskMode::Ft8, Some(&p))) };
}

/// A `memset`-to-zero params struct — the obvious C idiom — must be
/// rejected with a message, not decode nothing and report success.
///
/// It must also not be *undefined*. A `#[repr(C)]` fieldless enum is an
/// `int` to C, so zeroing the struct writes discriminants Rust may not
/// have; `read_params` validates each one as an integer before the
/// bytes are read as Rust types. `MfskDecodeDepth` gained an explicit
/// `MODE_DEFAULT = 0` for the same reason — 0 was deliberately
/// unassigned, which made the commonest C idiom produce an invalid
/// value.
#[test]
fn a_memset_params_struct_is_rejected_not_undefined() {
    let bytes = vec![0u8; std::mem::size_of::<MfskDecodeParams>()];
    let mut st = MfskStatus::Ok;
    let d = unsafe {
        mfsk_session_open(
            MfskMode::Ft8 as u32,
            bytes.as_ptr() as *const MfskDecodeParams,
            &mut st,
        )
    };
    assert!(d.is_null(), "an all-zero band and max_cand decode nothing");
    assert_eq!(st, MfskStatus::Unsupported);
}

/// An enum field a caller sets out of range is refused rather than
/// transmuted. This is the case that is undefined behaviour if the
/// boundary just copies bytes into a Rust struct.
#[test]
fn an_out_of_range_discriminant_is_refused() {
    let good = params(MfskMode::Ft8);
    let mut bytes = vec![0u8; std::mem::size_of::<MfskDecodeParams>()];
    unsafe {
        std::ptr::copy_nonoverlapping(
            &good as *const MfskDecodeParams as *const u8,
            bytes.as_mut_ptr(),
            bytes.len(),
        );
    }
    let off = std::mem::offset_of!(MfskDecodeParams, strictness);
    bytes[off..off + 4].copy_from_slice(&47i32.to_ne_bytes());

    let mut st = MfskStatus::Ok;
    let d = unsafe {
        mfsk_session_open(
            MfskMode::Ft8 as u32,
            bytes.as_ptr() as *const MfskDecodeParams,
            &mut st,
        )
    };
    assert!(d.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);
    let msg = unsafe { std::ffi::CStr::from_ptr(mfsk_last_error()) }.to_string_lossy();
    assert!(msg.contains("strictness") && msg.contains("47"), "{msg}");
}

/// Every mode with `MFSK_CAP_DECODE_HANDLE` must actually open and
/// decode through it — the bit is a promise.
#[test]
fn every_mode_claiming_the_handle_can_open_one() {
    for i in 0..mfsk_mode_count() {
        let mut m = MfskMode::Ft8;
        assert_eq!(unsafe { mfsk_mode_at(i, &mut m) }, MfskStatus::Ok);
        if mfsk_mode_caps(m as u32) & MFSK_CAP_DECODE_HANDLE == 0 {
            continue;
        }
        unsafe { mfsk_session_close(open(m, None)) };
    }
}

/// A mode without the bit must refuse **at open**, naming the bit.
///
/// Failing later, at decode, would be worse in a specific way: the
/// first thing to go wrong would be a complaint about the search
/// parameters when the real answer is "wrong entry point". Since #413
/// WSPR, JT9 and JT65 publish a default band, so a caller who filled
/// `MfskDecodeParams` from `mfsk_mode_defaults` would not even get that.
#[test]
fn a_mode_without_the_handle_is_refused_at_open() {
    for mode in [
        MfskMode::Wspr,
        MfskMode::Jt9,
        MfskMode::Jt65,
        MfskMode::Q65a30,
    ] {
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_session_open(mode as u32, std::ptr::null(), &mut st) };
        assert!(d.is_null(), "{mode:?} should not open a decode handle");
        assert_eq!(st, MfskStatus::Unsupported, "{mode:?}");
        let msg = unsafe { std::ffi::CStr::from_ptr(mfsk_last_error()) }.to_string_lossy();
        assert!(
            msg.contains("MFSK_CAP_DECODE_HANDLE"),
            "the error should name the bit to check: {msg}"
        );
    }
}

/// A short output buffer reports what was needed instead of silently
/// truncating, so a caller can size and retry without decoding twice.
#[test]
fn a_short_buffer_reports_the_count_it_needed() {
    let a = mfsk_core::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").unwrap();
    let b = mfsk_core::msg::wsjt77::pack77("CQ", "VK3NV", "QF22").unwrap();
    let mut audio = ft8_slot(&a, 1500.0);
    for (i, s) in ft8_slot(&b, 2100.0).iter().enumerate() {
        audio[i] = audio[i].saturating_add(*s);
    }

    let dec = open(MfskMode::Ft8, None);
    let mut one = [blank_row(); 1];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_i16(
            dec,
            audio.as_ptr(),
            audio.len(),
            FS,
            std::ptr::null(),
            one.as_mut_ptr(),
            1,
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::InvalidArg, "two decodes into one slot");
    assert_eq!(
        n, 2,
        "*out_len must be the count needed, not the count written"
    );
    unsafe { mfsk_session_close(dec) };
}

/// The float path must not be worse than the integer one at the rate
/// that needs no resampling — which is exactly where it used to be.
#[test]
fn the_float_path_keeps_dynamic_range_at_12k() {
    let msg = mfsk_core::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").unwrap();
    let loud = ft8_slot(&msg, 1500.0);
    // A quiet capture: peak ~1/64 of full scale, the shape a USB audio
    // adapter at a low system volume produces.
    let quiet: Vec<f32> = loud.iter().map(|&s| s as f32 / 32768.0 / 64.0).collect();

    let dec = open(MfskMode::Ft8, None);
    let mut rows = [blank_row(); 8];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_f32(
            dec,
            quiet.as_ptr(),
            quiet.len(),
            FS,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        n > 0,
        "a quiet float buffer at 12 kHz decoded nothing — the normalisation \
         that every other sample rate got is missing again"
    );
    unsafe { mfsk_session_close(dec) };
}

/// The FEC bits are reachable without the row carrying a pointer.
#[test]
fn info_bits_come_back_through_the_handle() {
    let msg = mfsk_core::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").unwrap();
    let dec = open(MfskMode::Ft8, None);
    let texts = decode(dec, &ft8_slot(&msg, 1500.0));
    assert!(!texts.is_empty());

    let mut need = 0usize;
    assert_eq!(
        unsafe { mfsk_session_copy_info(dec, 0, std::ptr::null_mut(), 0, &mut need) },
        MfskStatus::InvalidArg
    );
    assert_eq!(need, 91, "FT8 is LDPC(174,91)");

    let mut buf = vec![0u8; need];
    let mut got = 0usize;
    assert_eq!(
        unsafe { mfsk_session_copy_info(dec, 0, buf.as_mut_ptr(), buf.len(), &mut got) },
        MfskStatus::Ok
    );
    assert_eq!(got, 91);
    assert!(buf.iter().all(|&b| b <= 1), "info bits are 0/1 bytes");

    assert_eq!(
        unsafe { mfsk_session_copy_info(dec, 99, buf.as_mut_ptr(), buf.len(), &mut got) },
        MfskStatus::InvalidArg
    );
    unsafe { mfsk_session_close(dec) };
}

/// Rows report the concrete sub-mode. `Decoded::protocol` collapses all
/// five FST4 periods onto one id and the C row must not, because the
/// addressing model does not.
#[test]
fn rows_carry_the_concrete_submode() {
    let msg = mfsk_core::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").unwrap();
    let dec = open(MfskMode::Ft8, None);
    let mut rows = [blank_row(); 8];
    let mut n = 0usize;
    let audio = ft8_slot(&msg, 1500.0);
    assert_eq!(
        unsafe {
            mfsk_session_decode_i16(
                dec,
                audio.as_ptr(),
                audio.len(),
                FS,
                std::ptr::null(),
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    assert!(n > 0);
    assert_eq!(rows[0].mode, MfskMode::Ft8);
    assert_eq!(rows[0].info_bits, 91);
    assert!(rows[0].sync_score > 0.0, "sync_score should be reported");
    unsafe { mfsk_session_close(dec) };
}

/// Null handling on every new entry point.
#[test]
fn nulls_are_rejected_everywhere() {
    let m = params(MfskMode::Ft8);
    assert_eq!(
        unsafe { mfsk_decode_params_init(MfskMode::Ft8 as u32, std::ptr::null_mut()) },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe {
            mfsk_session_decode_i16(
                std::ptr::null_mut(),
                std::ptr::null(),
                0,
                FS,
                &m,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
            )
        },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe {
            mfsk_session_copy_info(
                std::ptr::null(),
                0,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
            )
        },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe { mfsk_session_add_callsign(std::ptr::null_mut(), std::ptr::null()) },
        MfskStatus::InvalidArg
    );
    assert!(unsafe { mfsk_session_last_error(std::ptr::null()) }.is_null());
    unsafe { mfsk_session_close(std::ptr::null_mut()) };
}
