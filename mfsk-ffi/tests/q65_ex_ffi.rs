//! `mfsk_q65_decode_ex` and the Q65 handles (`Q65History`, `Q65Callers`), the
//! way WSJT-X 3.2's Q65 settings reach a C, Kotlin or Swift caller (#466).
//!
//! The scenarios are the ones `mfsk-core`'s own tests use for the same
//! settings (`q65_pileup_flag`, `q65_q3`, `q65_max_drift`, `dt_window`), driven
//! through the ABI. Each setting has to *change what is decoded*: a field that
//! is accepted and ignored looks identical to one that works, from C, forever.

mod common;

use std::ffi::{CStr, CString, c_char};
use std::mem::{MaybeUninit, offset_of, size_of};
use std::ptr;

use common::*;
use mfsk::*;

const FS: usize = 12_000;
const FREQ: f32 = 1_500.0;
const WANT: &str = "K1ABC JA1ABC -15";

fn last_error() -> String {
    let p = mfsk_last_error();
    if p.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
}

fn put(dst: &mut [c_char], s: &str) {
    let b = s.as_bytes();
    let n = b.len().min(dst.len() - 1);
    for (d, &c) in dst.iter_mut().zip(&b[..n]) {
        *d = c as c_char;
    }
    dst[n] = 0;
}

fn q65_params(mode: MfskMode) -> MfskQ65Params {
    let mut p = MaybeUninit::<MfskQ65Params>::zeroed();
    assert_eq!(
        unsafe { mfsk_q65_params_init(mode as u32, p.as_mut_ptr()) },
        MfskStatus::Ok
    );
    unsafe { p.assume_init() }
}

/// `message` as Q65-30A audio, `flag` being Pileup's "copied last Tx".
fn frame(message: (&str, &str, &str), flag: bool) -> Vec<f32> {
    let (a, b, c) = (
        CString::new(message.0).unwrap(),
        CString::new(message.1).unwrap(),
        CString::new(message.2).unwrap(),
    );
    let mut pcm = vec![0f32; 40 * FS];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_q65_flagged(
                MfskQ65SubMode::A30 as u32,
                a.as_ptr(),
                b.as_ptr(),
                c.as_ptr(),
                flag as u32,
                FREQ,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    pcm.truncate(n);
    pcm
}

/// A 30 s slot with `sig` starting `start_s` seconds in.
fn slot30(sig: &[f32], start_s: f32) -> Vec<f32> {
    let mut a = vec![0f32; 30 * FS];
    let at = (start_s * FS as f32).round() as usize;
    let n = sig.len().min(a.len() - at);
    a[at..at + n].copy_from_slice(&sig[..n]);
    a
}

fn decode_ex(
    mode: MfskMode,
    audio: &[f32],
    p: Option<&MfskQ65Params>,
    callers: *const MfskQ65Callers,
) -> Result<Vec<MfskDecode>, MfskStatus> {
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_q65_decode_ex(
            mode as u32,
            audio.as_ptr(),
            audio.len(),
            FS as u32,
            p.map_or(ptr::null(), |p| p as *const _),
            callers,
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    if st != MfskStatus::Ok {
        return Err(st);
    }
    rows.truncate(n);
    Ok(rows)
}

fn messages(mode: MfskMode, audio: &[f32], p: &MfskQ65Params) -> Vec<String> {
    texts(&decode_ex(mode, audio, Some(p), ptr::null()).expect("decode"))
}

#[test]
fn the_defaults_are_the_librarys_own_and_say_where_dt_zero_is() {
    let p = q65_params(MfskMode::Q65a30);
    assert_eq!((p.freq_min_hz, p.freq_max_hz), (200.0, 3_000.0));
    // WSJT-X's ±1 s window, not the wide one the older functions scan.
    assert_eq!((p.t_early_s, p.t_late_s), (1.0, 1.0));
    assert_eq!(p.score_threshold, 0.1);
    assert_eq!(p.max_cand, 8);
    assert_eq!(p.nominal_start_s, 0.5, "Q65-30A's frame starts 0.5 s in");
    assert_eq!(q65_params(MfskMode::Q65d120).nominal_start_s, 1.0);
    assert!(
        p.rx_freq_hz.is_nan() && p.fading_b90_ts.is_nan(),
        "unset is NaN"
    );
    assert_eq!(p.ftol_hz, 10.0);
    assert_eq!(
        (p.pileup, p.eme_delay, p.max_drift, p.ap_list, p.has_ap_hint),
        (0, 0, 0, 0, 0)
    );

    // Not a Q65 mode, and not a mode at all.
    let mut q = MaybeUninit::<MfskQ65Params>::zeroed();
    for mode in [MfskMode::Ft8 as u32, MfskMode::Fst4s60 as u32, 9_999] {
        assert_eq!(
            unsafe { mfsk_q65_params_init(mode, q.as_mut_ptr()) },
            MfskStatus::InvalidArg,
            "mode {mode}"
        );
    }
    assert_eq!(
        decode_ex(MfskMode::Ft8, &[0.0; 16], None, ptr::null()).unwrap_err(),
        MfskStatus::InvalidArg
    );
}

/// A combination the engine would quietly not honour is refused, with the
/// reason — not dropped.
#[test]
fn a_setting_that_cannot_be_honoured_is_refused_not_dropped() {
    let mode = MfskMode::Q65a30;
    let audio = vec![0f32; 30 * FS];
    let base = || {
        let mut p = q65_params(mode);
        put(&mut p.list_my_call, "K1ABC");
        put(&mut p.list_his_call, "JA1ABC");
        p
    };
    type Case<'a> = (&'a dyn Fn(&mut MfskQ65Params), &'a str, &'a str);
    let cases: [Case; 14] = [
        (&|p| p.max_cand = 0, "max_cand", "no candidates"),
        (
            &|p| p.freq_max_hz = p.freq_min_hz,
            "search band",
            "an empty band",
        ),
        (
            &|p| p.nominal_start_s = -1.0,
            "nominal_start_s",
            "a negative start",
        ),
        (&|p| p.max_drift = 51, "max_drift", "drift past 50"),
        (&|p| p.ftol_hz = 0.0, "ftol_hz", "a zero F Tol"),
        (&|p| p.ap_list = 3, "ap_list", "an unknown list"),
        (
            &|p| {
                p.ap_list = 1;
                p.list_his_call = [0; 16];
            },
            "list_his_call",
            "a list with no DX call",
        ),
        (
            &|p| p.ap_list = 2,
            "MfskQ65Callers",
            "a contest list with no handle",
        ),
        (
            &|p| {
                p.ap_list = 1;
                p.fading_b90_ts = 1.0;
            },
            "mutually exclusive",
            "a list and fading",
        ),
        (
            &|p| {
                p.fading_b90_ts = 1.0;
                p.fading_model = 7;
            },
            "fading_model",
            "an unknown fading model",
        ),
        (
            &|p| p.rx_freq_hz = 1_500.0,
            "ap_list",
            "an Rx frequency with no list",
        ),
        (&|p| p.pileup = 1, "AP hint", "pileup with no hint"),
        (
            &|p| {
                p.fading_b90_ts = 1.0;
                p.max_drift = 10;
            },
            "fast-fading",
            "drift with fading",
        ),
        (
            &|p| {
                p.ap_list = 1;
                p.max_drift = 10;
            },
            "rx_freq_hz",
            "drift with a list and no Rx frequency",
        ),
    ];
    for (mutate, needle, what) in cases {
        let mut p = base();
        mutate(&mut p);
        assert_eq!(
            decode_ex(mode, &audio, Some(&p), ptr::null()).unwrap_err(),
            MfskStatus::Unsupported,
            "{what} should be refused"
        );
        assert!(
            last_error().contains(needle),
            "{what}: the refusal should name {needle:?}, got {:?}",
            last_error()
        );
    }

    // A callers handle that nothing reads.
    let callers = mfsk_q65_callers_new();
    assert_eq!(
        decode_ex(mode, &audio, Some(&base()), callers).unwrap_err(),
        MfskStatus::Unsupported
    );
    unsafe { mfsk_q65_callers_free(callers) };
}

#[test]
fn dt_is_measured_from_the_nominal_start_and_the_flagged_row_says_so() {
    let mode = MfskMode::Q65a30;
    let p = q65_params(mode);
    for (start, want_dt) in [(0.5f32, 0.0f32), (0.9, 0.4), (0.2, -0.3)] {
        let rows = decode_ex(
            mode,
            &slot30(&frame(("K1ABC", "JA1ABC", "-15"), false), start),
            Some(&p),
            ptr::null(),
        )
        .expect("decode");
        assert_eq!(texts(&rows), [WANT], "start {start}");
        assert!(
            (rows[0].dt_sec - want_dt).abs() < 0.05,
            "dt {}",
            rows[0].dt_sec
        );
        assert_eq!(rows[0].mode, mode);
        assert_eq!(rows[0].flags & MFSK_DECODE_FLAG_COPIED_LAST_TX, 0);
    }

    // The flag reaches the row, on the plain scan and on the older entry
    // points too.
    let flagged = slot30(&frame(("K1ABC", "JA1ABC", "-15"), true), 0.5);
    let rows = decode_ex(mode, &flagged, Some(&p), ptr::null()).expect("decode");
    assert_eq!(texts(&rows), [WANT]);
    assert_ne!(rows[0].flags & MFSK_DECODE_FLAG_COPIED_LAST_TX, 0);

    let mut legacy = vec![blank_row(); 8];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_q65_decode(
                MfskQ65SubMode::A30 as u32,
                flagged.as_ptr(),
                flagged.len(),
                FS as u32,
                ptr::null(),
                legacy.as_mut_ptr(),
                legacy.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    assert_eq!(n, 1);
    assert_ne!(legacy[0].flags & MFSK_DECODE_FLAG_COPIED_LAST_TX, 0);
}

#[test]
fn the_flagged_encode_changes_only_the_codeword() {
    let (a, b, c) = (
        CString::new("K1ABC").unwrap(),
        CString::new("JA1ABC").unwrap(),
        CString::new("-15").unwrap(),
    );
    let run = |f: fn(u32) -> u32, flag: u32| {
        let mut pcm = vec![0f32; 40 * FS];
        let mut n = 0usize;
        assert_eq!(
            unsafe {
                mfsk_encode_q65_flagged(
                    MfskQ65SubMode::A30 as u32,
                    a.as_ptr(),
                    b.as_ptr(),
                    c.as_ptr(),
                    f(flag),
                    FREQ,
                    pcm.as_mut_ptr(),
                    pcm.len(),
                    &mut n,
                )
            },
            MfskStatus::Ok
        );
        pcm.truncate(n);
        pcm
    };
    let unflagged = frame(("K1ABC", "JA1ABC", "-15"), false);
    let mut plain = vec![0f32; 40 * FS];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_q65(
                MfskQ65SubMode::A30 as u32,
                a.as_ptr(),
                b.as_ptr(),
                c.as_ptr(),
                FREQ,
                plain.as_mut_ptr(),
                plain.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    plain.truncate(n);
    assert_eq!(unflagged, plain, "flag 0 is exactly mfsk_encode_q65");
    assert_ne!(run(|f| f, 1), plain, "the flag changes the transmission");
    assert_eq!(run(|f| f, 7), run(|f| f, 1), "any non-zero value is on");
}

/// Q65-30A's `nsps >= 3600` puts the EME late edge at +5.5 s, against the
/// default +1.0 s: a frame 3 s late is found only with it on.
#[test]
fn eme_delay_reaches_a_frame_the_default_window_does_not() {
    let mode = MfskMode::Q65a30;
    let audio = slot30(&frame(("K1ABC", "JA1ABC", "-15"), false), 3.5);
    let mut p = q65_params(mode);
    assert!(messages(mode, &audio, &p).is_empty(), "default +1.0 s late");
    p.eme_delay = 1;
    let rows = decode_ex(mode, &audio, Some(&p), ptr::null()).expect("decode");
    assert_eq!(texts(&rows), [WANT]);
    assert!((rows[0].dt_sec - 3.0).abs() < 0.05, "dt {}", rows[0].dt_sec);
}

/// A tone drift of 60 Hz/min across the frame: the plain decode loses it and
/// `max_drift` recovers it (`mfsk-core/tests/q65_max_drift.rs`'s frame).
#[test]
fn max_drift_recovers_a_drifting_signal() {
    const NSPS: usize = 3600;
    // The tones the C ABI's encoder transmits, from the core encoder it calls
    // (the ABI's encoder emits audio, and this needs to bend the tones).
    let bits = mfsk_core::msg::q65::pack77_q65("K1ABC", "JA1ABC", "-15").unwrap();
    let tones = mfsk_core::q65::encode_channel_symbols(&bits);
    let mut a = vec![0f32; 30 * FS];
    let baud = FS as f32 / NSPS as f32;
    let mut phi = 0f64;
    for (k, &t) in tones.iter().enumerate() {
        for i in 0..NSPS {
            let n = FS / 2 + k * NSPS + i;
            let secs = n as f32 / FS as f32;
            let f = FREQ + t as f32 * baud + 60.0 * (secs - 15.0) / 60.0;
            phi += core::f64::consts::TAU * f as f64 / FS as f64;
            a[n] = 0.3 * phi.sin() as f32;
        }
    }
    let mode = MfskMode::Q65a30;
    let mut p = q65_params(mode);
    // Upstream narrows to nfqso ± ntol when drift is searched.
    p.freq_min_hz = FREQ - 50.0;
    p.freq_max_hz = FREQ + 50.0;
    assert!(
        messages(mode, &a, &p).is_empty(),
        "the plain decode loses it"
    );
    p.max_drift = 10;
    assert_eq!(messages(mode, &a, &p), [WANT]);
}

/// A reply with the "copied last Tx" flag: outside Pileup a MyCall + DxCall
/// hint locks the flag to 0 and cannot match it; under Pileup it does.
#[test]
fn pileup_frees_the_78th_bit_for_a_mycall_dxcall_hint() {
    let mode = MfskMode::Q65a30;
    let flagged = slot30(&frame(("K1ABC", "JA1ABC", "-15"), true), 0.5);
    let mut p = q65_params(mode);
    p.has_ap_hint = 1;
    put(&mut p.ap_call1, "K1ABC");
    put(&mut p.ap_call2, "JA1ABC");

    p.pileup = 1;
    let rows = decode_ex(mode, &flagged, Some(&p), ptr::null()).expect("decode");
    assert_eq!(texts(&rows), [WANT]);
    assert_ne!(rows[0].flags & MFSK_DECODE_FLAG_COPIED_LAST_TX, 0);

    // An unflagged reply decodes either way.
    let plain = slot30(&frame(("K1ABC", "JA1ABC", "-15"), false), 0.5);
    for pileup in [0, 1] {
        p.pileup = pileup;
        assert_eq!(messages(mode, &plain, &p), [WANT], "pileup {pileup}");
    }
}

/// q3: the list decode at the Rx frequency, with a window that holds nothing
/// else, finds a list message the scan could not have (`q65_q3.rs`).
#[test]
fn q3_finds_a_list_message_at_the_rx_frequency() {
    let mode = MfskMode::Q65a30;
    let audio = slot30(&frame(("K1ABC", "JA1ABC", "-15"), false), 0.5);
    let mut p = q65_params(mode);
    p.freq_min_hz = 3_900.0;
    p.freq_max_hz = 3_950.0;
    put(&mut p.list_my_call, "K1ABC");
    put(&mut p.list_his_call, "JA1ABC");
    put(&mut p.list_his_grid, "PM95");

    assert!(
        messages(mode, &audio, &p).is_empty(),
        "the scan window holds nothing"
    );
    p.ap_list = 1;
    p.rx_freq_hz = FREQ;
    assert_eq!(messages(mode, &audio, &p), [WANT], "q3 at the Rx frequency");

    // Off the Rx frequency by more than F Tol: not there to be found.
    p.rx_freq_hz = FREQ + 100.0;
    assert!(messages(mode, &audio, &p).is_empty());
    p.ftol_hz = 150.0;
    assert_eq!(messages(mode, &audio, &p), [WANT], "F Tol reaches it");

    // Noise alone yields nothing.
    let mut noise = vec![0f32; 30 * FS];
    Awgn::new(0.3, 1).apply(&mut noise);
    p.rx_freq_hz = FREQ;
    p.ftol_hz = 10.0;
    assert!(messages(mode, &noise, &p).is_empty());
}

/// The contest list: callers remembered from earlier decodes, decoded
/// together. `K1ABC W9XYZ RR73` is on the list only because W9XYZ called
/// with a grid.
#[test]
fn the_contest_list_decodes_a_caller_it_remembers() {
    let mode = MfskMode::Q65a30;
    let audio = slot30(&frame(("K1ABC", "W9XYZ", "RR73"), false), 0.5);
    let mut p = q65_params(mode);
    p.freq_min_hz = 3_900.0;
    p.freq_max_hz = 3_950.0;
    put(&mut p.list_my_call, "K1ABC");
    p.ap_list = 2;
    p.rx_freq_hz = FREQ;

    let callers = mfsk_q65_callers_new();
    assert!(!callers.is_null());
    // Nobody listed: nothing to find.
    assert!(
        decode_ex(mode, &audio, Some(&p), callers)
            .expect("decode")
            .is_empty()
    );

    let msg = CString::new("K1ABC W9XYZ EN37").unwrap();
    assert_eq!(
        unsafe { mfsk_q65_callers_record(callers, 1_500.0, msg.as_ptr(), 1_000) },
        MfskStatus::Ok
    );
    let rows = decode_ex(mode, &audio, Some(&p), callers).expect("decode");
    assert_eq!(texts(&rows), ["K1ABC W9XYZ RR73"]);
    unsafe { mfsk_q65_callers_free(callers) };
}

#[test]
fn the_caller_list_remembers_expires_and_forgets() {
    let h = mfsk_q65_callers_new();
    let rec = |f: f32, m: &str, now: u64| {
        let m = CString::new(m).unwrap();
        assert_eq!(
            unsafe { mfsk_q65_callers_record(h, f, m.as_ptr(), now) },
            MfskStatus::Ok
        );
    };
    rec(1_500.0, "K1ABC W9XYZ EN37", 100);
    rec(1_510.0, "K1ABC JA1ABC R PM95", 100);
    rec(1_520.0, "K1ABC VK3ABC -15", 100); // no grid: not added
    rec(1_530.0, "K1ABC W9XYZ/R EN37", 100); // compound: ignored
    assert_eq!(unsafe { mfsk_q65_callers_len(h) }, 2);

    let get = |i: usize| -> Option<(String, String, u64, i32)> {
        let mut c = MaybeUninit::<MfskQ65Caller>::zeroed();
        unsafe { (*c.as_mut_ptr()).size = size_of::<MfskQ65Caller>() as u32 };
        if unsafe { mfsk_q65_callers_get(h, i, c.as_mut_ptr()) } != MfskStatus::Ok {
            return None;
        }
        let c = unsafe { c.assume_init() };
        let s = |a: &[c_char]| {
            unsafe { CStr::from_ptr(a.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        };
        Some((s(&c.call), s(&c.grid), c.last_heard, c.freq_hz))
    };
    assert_eq!(get(0), Some(("W9XYZ".into(), "EN37".into(), 100, 1_500)));
    assert_eq!(get(1), Some(("JA1ABC".into(), "PM95".into(), 100, 1_510)));
    assert_eq!(get(2), None, "past the end is an error, not garbage");

    // A known caller is refreshed.
    rec(1_600.0, "K1ABC W9XYZ RR73", 500);
    assert_eq!(get(0), Some(("W9XYZ".into(), "EN37".into(), 500, 1_600)));

    // Twenty-four hours on, only the refreshed one is left.
    assert_eq!(
        unsafe { mfsk_q65_callers_expire(h, 100 + 24 * 3600 + 1) },
        MfskStatus::Ok
    );
    assert_eq!(unsafe { mfsk_q65_callers_len(h) }, 1);
    let w = CString::new("W9XYZ").unwrap();
    assert_eq!(
        unsafe { mfsk_q65_callers_remove(h, w.as_ptr()) },
        MfskStatus::Ok
    );
    assert_eq!(unsafe { mfsk_q65_callers_len(h) }, 0);
    unsafe { mfsk_q65_callers_free(h) };

    assert_eq!(unsafe { mfsk_q65_callers_len(ptr::null()) }, 0);
    unsafe { mfsk_q65_callers_free(ptr::null_mut()) };
}

#[test]
fn the_history_finds_the_dx_station_near_the_rx_frequency() {
    let h = mfsk_q65_history_new();
    let push = |f: f32, m: &str| {
        let m = CString::new(m).unwrap();
        assert_eq!(
            unsafe { mfsk_q65_history_push(h, f, m.as_ptr()) },
            MfskStatus::Ok
        );
    };
    let lookup = |f: f32| -> Option<(String, Option<String>)> {
        let mut dx = MaybeUninit::<MfskQ65Dx>::zeroed();
        unsafe { (*dx.as_mut_ptr()).size = size_of::<MfskQ65Dx>() as u32 };
        if unsafe { mfsk_q65_history_lookup(h, f, dx.as_mut_ptr()) } != MfskStatus::Ok {
            return None;
        }
        let dx = unsafe { dx.assume_init() };
        let s = |a: &[c_char]| {
            unsafe { CStr::from_ptr(a.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        };
        Some((s(&dx.call), (dx.has_grid != 0).then(|| s(&dx.grid))))
    };
    assert_eq!(lookup(1_500.0), None, "an empty history finds nothing");

    push(1_500.0, "K1ABC JA1ABC PM95");
    assert_eq!(
        lookup(1_500.0),
        Some(("JA1ABC".into(), Some("PM95".into())))
    );
    assert_eq!(
        lookup(1_508.0),
        Some(("JA1ABC".into(), Some("PM95".into()))),
        "within 10 Hz"
    );
    assert_eq!(lookup(1_520.0), None, "outside 10 Hz");

    // A later CQ is passed over (its first word is too short) for the older
    // decode, and a message with no grid names the call alone.
    push(1_500.0, "CQ VK3ABC QF22");
    assert_eq!(
        lookup(1_500.0),
        Some(("JA1ABC".into(), Some("PM95".into())))
    );
    push(1_500.0, "K1ABC W9XYZ -15");
    assert_eq!(lookup(1_500.0), Some(("W9XYZ".into(), None)));

    // Rows straight from a decode go in the same way.
    let mut row = blank_row();
    row.freq_hz = 700.0;
    let t = b"K1ABC DL1XYZ JO62\0";
    for (d, &c) in row.text.iter_mut().zip(t) {
        *d = c as c_char;
    }
    assert_eq!(
        unsafe { mfsk_q65_history_record(h, &row, 1) },
        MfskStatus::Ok
    );
    assert_eq!(lookup(700.0), Some(("DL1XYZ".into(), Some("JO62".into()))));
    assert_eq!(unsafe { mfsk_q65_history_len(h) }, 4);

    // Only the 100 most recent are kept.
    for i in 0..120 {
        push(2_000.0 + i as f32, "K1ABC JA1ABC PM95");
    }
    assert_eq!(unsafe { mfsk_q65_history_len(h) }, 100);
    unsafe { mfsk_q65_history_free(h) };
    unsafe { mfsk_q65_history_free(ptr::null_mut()) };
}

/// A caller built against an older header passes a shorter `size`; the tail
/// stays at the defaults instead of being read from beyond what it wrote.
#[test]
fn a_shorter_params_struct_leaves_the_tail_at_its_defaults() {
    let mode = MfskMode::Q65a30;
    let audio = slot30(&frame(("K1ABC", "JA1ABC", "-15"), false), 0.5);
    let mut p = q65_params(mode);
    p.size = offset_of!(MfskQ65Params, pileup) as u32;
    // Garbage past `size`: pileup with no hint would be refused if it were read.
    p.pileup = 1;
    p.max_drift = 50;
    let rows = decode_ex(mode, &audio, Some(&p), ptr::null()).expect("the tail must not be read");
    assert_eq!(texts(&rows), [WANT]);
}

#[test]
fn a_short_output_buffer_reports_the_count_it_needed() {
    let mode = MfskMode::Q65a30;
    let audio = slot30(&frame(("K1ABC", "JA1ABC", "-15"), false), 0.5);
    let mut n = 0usize;
    let st = unsafe {
        mfsk_q65_decode_ex(
            mode as u32,
            audio.as_ptr(),
            audio.len(),
            FS as u32,
            ptr::null(),
            ptr::null(),
            ptr::null(),
            ptr::null_mut(),
            0,
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::InvalidArg);
    assert_eq!(n, 1);
}
