//! The JTTY receiver handle (`mfsk_jtty_*`), fed the way a C, Kotlin or Swift
//! host feeds it: chunks of audio in, `poll` until it says 0.
//!
//! The recording is upstream's own (`golden/jtty/260807_134110.wav`); what
//! `rjtty` reads from it is `RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!`
//! (plus one false decode on channel 2, which the golden test in `mfsk-core`
//! pins; here it shows up as a second, short message).

use mfsk::*;
use std::ffi::CStr;
use std::ptr;

const MESSAGE: &str = "RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!";

fn sample() -> Vec<i16> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/golden/jtty/260807_134110.wav"
    );
    let b = std::fs::read(path).expect("vendored golden recording");
    let at = b.windows(4).position(|w| w == b"data").expect("data chunk") + 8;
    let (pairs, _) = b[at..].as_chunks::<2>();
    pairs.iter().map(|c| i16::from_le_bytes(*c)).collect()
}

fn open(rate: u32) -> *mut MfskJttyReceiver {
    let mut st = MfskStatus::Internal;
    let rx = unsafe { mfsk_jtty_open(rate, ptr::null(), &mut st) };
    assert_eq!(st, MfskStatus::Ok);
    assert!(!rx.is_null());
    rx
}

fn text(u: &MfskJttyUpdate) -> String {
    unsafe { CStr::from_ptr(u.text.as_ptr()) }
        .to_str()
        .unwrap()
        .to_string()
}

/// Everything waiting, oldest first.
fn drain(rx: *mut MfskJttyReceiver) -> Vec<(u64, String, bool, f32)> {
    let mut out = Vec::new();
    loop {
        let mut u = MfskJttyUpdate {
            size: std::mem::size_of::<MfskJttyUpdate>() as u32,
            complete: 0,
            id: 0,
            f1_hz: 0.0,
            start_s: 0.0,
            text: [0; 128],
        };
        match unsafe { mfsk_jtty_poll(rx, &mut u) } {
            1 => out.push((u.id, text(&u), u.complete != 0, u.f1_hz)),
            0 => return out,
            e => panic!("poll returned {e}"),
        }
    }
}

/// Feed `pcm` in `chunk`-sample pieces, polling after each, and keep the last
/// update seen per message id.
fn run(rx: *mut MfskJttyReceiver, pcm: &[i16], chunk: usize) -> Vec<(u64, String, bool, f32)> {
    let mut last: Vec<(u64, String, bool, f32)> = Vec::new();
    for c in pcm.chunks(chunk) {
        assert_eq!(
            unsafe { mfsk_jtty_push_i16(rx, c.as_ptr(), c.len()) },
            MfskStatus::Ok
        );
        for u in drain(rx) {
            match last.iter_mut().find(|l| l.0 == u.0) {
                Some(l) => *l = u,
                None => last.push(u),
            }
        }
    }
    last
}

#[test]
fn jtty_is_addressable_and_describes_its_frame() {
    let mut i = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
    assert_eq!(
        unsafe { mfsk_mode_info(MfskMode::Jtty as u32, i.as_mut_ptr()) },
        MfskStatus::Ok
    );
    let i = unsafe { i.assume_init() };
    assert_eq!(
        unsafe { CStr::from_ptr(i.name.as_ptr()) }.to_str().unwrap(),
        "JTTY"
    );
    assert_eq!((i.ntones, i.n_symbols, i.n_sync, i.n_data), (4, 59, 13, 46));
    assert_eq!(i.slot_samples_12k, 22_656, "one 1.888 s frame at 12 kHz");
    assert_ne!(i.caps & MFSK_CAP_STREAM_RECEIVER, 0);
    assert_eq!(
        i.caps & MFSK_CAP_DECODE_HANDLE,
        0,
        "there is no slot decode handle"
    );
    assert_eq!(
        mfsk_mode_caps(MfskMode::Jtty as u32),
        MFSK_CAP_STREAM_RECEIVER
    );

    let mut m = MfskMode::Ft8;
    let name = std::ffi::CString::new("JTTY").unwrap();
    assert_eq!(
        unsafe { mfsk_mode_from_name(name.as_ptr(), &mut m) },
        MfskStatus::Ok
    );
    assert_eq!(m, MfskMode::Jtty);
}

#[test]
fn the_recording_reads_the_same_whatever_the_chunk_size() {
    let pcm = sample();
    let mut runs = Vec::new();
    for chunk in [1000, 4096, 30_000, pcm.len()] {
        let rx = open(12_000);
        let mut last = run(rx, &pcm, chunk);
        last.sort_by_key(|a| a.0);
        runs.push(last);
        unsafe { mfsk_jtty_close(rx) };
    }
    let msg = runs[0]
        .iter()
        .find(|u| u.1.starts_with("RAN ALL NIGHT"))
        .expect("the message");
    assert_eq!(msg.1, MESSAGE);
    assert!((msg.3 - 1507.0).abs() < 3.0, "{} Hz", msg.3);
    // Polling once per chunk gives, per message, its last update — and that is
    // what the coalescing hands out however the audio was cut. Ids are the
    // same too (they count messages from the start).
    let texts =
        |r: &Vec<(u64, String, bool, f32)>| r.iter().map(|u| u.1.clone()).collect::<Vec<_>>();
    for r in &runs[1..] {
        assert_eq!(texts(r), texts(&runs[0]));
    }
}

#[test]
fn updates_between_polls_coalesce_per_message() {
    let pcm = sample();
    let rx = open(12_000);
    // No polling until the end: the queue holds one row per message, not one
    // per frame (that message alone grew ten times).
    for c in pcm.chunks(4096) {
        assert_eq!(
            unsafe { mfsk_jtty_push_i16(rx, c.as_ptr(), c.len()) },
            MfskStatus::Ok
        );
    }
    let pending = mfsk_jtty_pending(rx);
    let all = drain(rx);
    assert_eq!(all.len(), pending);
    assert!(all.len() <= 3, "{all:?}");
    assert!(all.iter().any(|u| u.1 == MESSAGE), "{all:?}");
    assert_eq!(mfsk_jtty_pending(rx), 0);
    unsafe { mfsk_jtty_close(rx) };
}

#[test]
fn finish_and_reset() {
    let pcm = sample();
    let rx = open(12_000);
    // 12 s of a 22 s message: open, not complete.
    let _ = run(rx, &pcm[..12 * 12_000], 4096);
    assert_eq!(unsafe { mfsk_jtty_finish(rx) }, MfskStatus::Ok);
    let after = drain(rx);
    assert!(
        after
            .iter()
            .any(|u| u.1.starts_with("RAN ALL NIGHT") && !u.2),
        "{after:?}"
    );
    assert_eq!(unsafe { mfsk_jtty_reset(rx) }, MfskStatus::Ok);
    assert_eq!(mfsk_jtty_pending(rx), 0);
    // and it decodes again from the start
    let again = run(rx, &pcm, 8192);
    assert!(again.iter().any(|u| u.1 == MESSAGE && u.2), "{again:?}");
    unsafe { mfsk_jtty_close(rx) };
}

#[test]
fn other_sample_rates_are_resampled() {
    let pcm = sample();
    // 24 kHz by linear interpolation
    let mut up = Vec::with_capacity(pcm.len() * 2);
    for w in pcm.windows(2) {
        up.push(w[0]);
        up.push(((w[0] as i32 + w[1] as i32) / 2) as i16);
    }
    let rx = open(24_000);
    let last = run(rx, &up, 8192);
    assert!(last.iter().any(|u| u.1 == MESSAGE), "{last:?}");
    // f32 goes through the same path
    assert_eq!(unsafe { mfsk_jtty_reset(rx) }, MfskStatus::Ok);
    let f: Vec<f32> = up.iter().map(|&x| x as f32 / 32768.0).collect();
    for c in f.chunks(8192) {
        assert_eq!(
            unsafe { mfsk_jtty_push_f32(rx, c.as_ptr(), c.len()) },
            MfskStatus::Ok
        );
    }
    assert!(drain(rx).iter().any(|u| u.1 == MESSAGE));
    unsafe { mfsk_jtty_close(rx) };
}

#[test]
fn parameters_are_size_versioned_and_checked() {
    let mut p = std::mem::MaybeUninit::<MfskJttyParams>::zeroed();
    assert_eq!(
        unsafe { mfsk_jtty_params_init(p.as_mut_ptr()) },
        MfskStatus::Ok
    );
    let mut p = unsafe { p.assume_init() };
    assert_eq!(p.size as usize, std::mem::size_of::<MfskJttyParams>());
    assert_eq!(
        (p.f0_hz, p.ftol_hz, p.smin_db, p.subtract),
        (1500.0, 50.0, 4.6, 1)
    );

    // Bad values are refused; a NULL pointer means the defaults.
    let mut st = MfskStatus::Ok;
    p.nfb_hz = p.nfa_hz;
    assert!(unsafe { mfsk_jtty_open(12_000, &p, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);
    p.nfb_hz = 2800.0;
    p.f0_hz = f32::NAN;
    assert!(unsafe { mfsk_jtty_open(12_000, &p, &mut st) }.is_null());
    assert!(unsafe { mfsk_jtty_open(0, ptr::null(), &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);

    p.f0_hz = 1000.0;
    let rx = unsafe { mfsk_jtty_open(12_000, &p, &mut st) };
    assert_eq!(st, MfskStatus::Ok);
    assert_eq!(
        unsafe { mfsk_jtty_set_params(rx, ptr::null()) },
        MfskStatus::Ok
    );
    p.ftol_hz = -1.0;
    assert_eq!(
        unsafe { mfsk_jtty_set_params(rx, &p) },
        MfskStatus::InvalidArg
    );
    // an older, shorter struct: only `size` and `subtract` — the rest stays default
    let short = [8u32, 0u32];
    assert_eq!(
        unsafe { mfsk_jtty_set_params(rx, short.as_ptr() as *const MfskJttyParams) },
        MfskStatus::Ok
    );
    unsafe { mfsk_jtty_close(rx) };
}

#[test]
fn null_handles_are_refused_not_dereferenced() {
    let mut u = std::mem::MaybeUninit::<MfskJttyUpdate>::zeroed();
    let z = [0i16; 4];
    unsafe {
        assert_eq!(
            mfsk_jtty_push_i16(ptr::null_mut(), z.as_ptr(), 4),
            MfskStatus::NullPointer
        );
        assert_eq!(mfsk_jtty_finish(ptr::null_mut()), MfskStatus::NullPointer);
        assert_eq!(mfsk_jtty_reset(ptr::null_mut()), MfskStatus::NullPointer);
        assert_eq!(
            mfsk_jtty_poll(ptr::null_mut(), u.as_mut_ptr()),
            MfskStatus::NullPointer as i32
        );
        mfsk_jtty_close(ptr::null_mut());
    }
    assert_eq!(mfsk_jtty_pending(ptr::null_mut()), 0);
    let rx = open(12_000);
    unsafe {
        assert_eq!(
            mfsk_jtty_poll(rx, ptr::null_mut()),
            MfskStatus::NullPointer as i32
        );
        assert_eq!(
            mfsk_jtty_poll(rx, u.as_mut_ptr()),
            0,
            "nothing waiting is 0, not an error"
        );
        assert_eq!(mfsk_jtty_push_i16(rx, ptr::null(), 0), MfskStatus::Ok);
        assert_eq!(
            mfsk_jtty_push_i16(rx, ptr::null(), 4),
            MfskStatus::NullPointer
        );
        mfsk_jtty_close(rx);
    }
}
