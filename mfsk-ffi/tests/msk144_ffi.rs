//! The MSK144 receiver handle (`mfsk_msk144_*`, #497), fed the way a C, Kotlin
//! or Swift host feeds it: chunks of audio in, `poll` until it says 0.
//!
//! The recordings are WSJT-X's own (`golden/msk144/`); WSJT-X decodes
//! `K1JT WA4CQG EM72` from the first and `CQ W4IMD EM84`, `CQ KD9VV EN71` from
//! the second (`mfsk-core/tests/msk144_wsjtx_samples.rs`).

use mfsk::*;
use std::ffi::CStr;
use std::ptr;

const GOLDEN: [(&str, &[&str]); 2] = [
    ("181211_120500.wav", &["K1JT WA4CQG EM72"]),
    ("181211_120800.wav", &["CQ W4IMD EM84", "CQ KD9VV EN71"]),
];

fn sample(name: &str) -> Vec<i16> {
    let path = format!(
        "{}/../embedded-poc/assets/golden/msk144/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let b = std::fs::read(path).expect("vendored golden recording");
    let at = b.windows(4).position(|w| w == b"data").expect("data chunk") + 8;
    let (pairs, _) = b[at..].as_chunks::<2>();
    pairs.iter().map(|c| i16::from_le_bytes(*c)).collect()
}

fn open(rate: u32) -> *mut MfskMsk144Receiver {
    let mut st = MfskStatus::Internal;
    let rx = unsafe { mfsk_msk144_open(rate, 1477.0, 60.0, 3, &mut st) };
    assert_eq!(st, MfskStatus::Ok);
    assert!(!rx.is_null());
    rx
}

fn empty() -> MfskMsk144Decode {
    MfskMsk144Decode {
        size: std::mem::size_of::<MfskMsk144Decode>() as u32,
        snr_db: 0,
        freq_hz: 0.0,
        reserved: 0,
        t_s: 0.0,
        text: [0; 64],
    }
}

fn drain(rx: *mut MfskMsk144Receiver) -> Vec<(String, i32, f32, f64)> {
    let mut out = Vec::new();
    loop {
        let mut d = empty();
        match unsafe { mfsk_msk144_poll(rx, &mut d) } {
            1 => out.push((
                unsafe { CStr::from_ptr(d.text.as_ptr()) }
                    .to_str()
                    .unwrap()
                    .to_string(),
                d.snr_db,
                d.freq_hz,
                d.t_s,
            )),
            0 => return out,
            e => panic!("poll returned {e}"),
        }
    }
}

fn run(rx: *mut MfskMsk144Receiver, pcm: &[i16], chunk: usize) -> Vec<(String, i32, f32, f64)> {
    let mut all = Vec::new();
    for c in pcm.chunks(chunk) {
        assert_eq!(
            unsafe { mfsk_msk144_push_i16(rx, c.as_ptr(), c.len()) },
            MfskStatus::Ok
        );
        all.extend(drain(rx));
    }
    all
}

#[test]
fn msk144_is_a_stream_receiver() {
    let caps = mfsk_mode_caps(MfskMode::Msk144 as u32);
    assert_ne!(caps & MFSK_CAP_STREAM_RECEIVER, 0);
    assert_eq!(caps & MFSK_CAP_DECODE_HANDLE, 0, "no slot decode handle");
    let mut i = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
    assert_eq!(
        unsafe { mfsk_mode_info(MfskMode::Msk144 as u32, i.as_mut_ptr()) },
        MfskStatus::Ok
    );
    assert_eq!(unsafe { i.assume_init() }.caps, caps);
}

#[test]
fn the_goldens_decode_whatever_the_chunk_size() {
    for (name, want) in GOLDEN {
        let pcm = sample(name);
        let mut runs = Vec::new();
        for chunk in [1usize, 480, 3584, 4096, pcm.len()] {
            let rx = open(12_000);
            runs.push(run(rx, &pcm, chunk));
            unsafe { mfsk_msk144_close(rx) };
        }
        for w in want {
            assert!(
                runs[0].iter().any(|d| d.0 == *w),
                "{name}: {w} missing from {:?}",
                runs[0]
            );
        }
        assert!(
            runs.iter().all(|r| r == &runs[0]),
            "{name}: chunk sizes disagree: {runs:?}"
        );
    }
}

/// 48 kHz in (each 12 kHz sample four times): resampled to 12 kHz inside, the
/// messages still come out.
#[test]
fn other_rates_are_resampled() {
    let (name, want) = GOLDEN[1];
    let pcm: Vec<i16> = sample(name).iter().flat_map(|&s| [s; 4]).collect();
    let rx = open(48_000);
    let got = run(rx, &pcm, 4800);
    unsafe { mfsk_msk144_close(rx) };
    for w in want {
        assert!(
            got.iter().any(|d| d.0 == *w),
            "{w} missing at 48 kHz: {got:?}"
        );
    }
}

#[test]
fn new_period_and_reset() {
    let (name, want) = GOLDEN[0];
    let pcm = sample(name);
    let rx = open(12_000);
    let first = run(rx, &pcm, 12_000);
    assert!(first.iter().any(|d| d.0 == want[0]));

    // A new period: the same slot again reports its message again, later.
    assert_eq!(unsafe { mfsk_msk144_new_period(rx) }, MfskStatus::Ok);
    let again = run(rx, &pcm, 12_000);
    let t0 = first.iter().find(|d| d.0 == want[0]).unwrap().3;
    let t1 = again
        .iter()
        .find(|d| d.0 == want[0])
        .expect("repeat after new_period")
        .3;
    assert!(t1 > t0 + 14.0, "times run on: {t0} then {t1}");

    // Reset: times start again.
    assert_eq!(unsafe { mfsk_msk144_reset(rx) }, MfskStatus::Ok);
    assert_eq!(mfsk_msk144_pending(rx), 0);
    let fresh = run(rx, &pcm, 12_000);
    assert_eq!(fresh, first);
    unsafe { mfsk_msk144_close(rx) };
}

/// An old caller's shorter struct gets only the fields it knows.
#[test]
fn poll_is_size_versioned() {
    let (name, _) = GOLDEN[0];
    let pcm = sample(name);
    let rx = open(12_000);
    unsafe { mfsk_msk144_push_i16(rx, pcm.as_ptr(), pcm.len()) };
    let mut d = empty();
    d.size = 12; // size, snr_db, freq_hz
    d.t_s = -1.0;
    assert_eq!(unsafe { mfsk_msk144_poll(rx, &mut d) }, 1);
    assert_eq!(d.t_s, -1.0, "wrote past the declared size");
    assert_ne!(d.freq_hz, 0.0);
    unsafe { mfsk_msk144_close(rx) };
}

#[test]
fn bad_arguments() {
    let mut st = MfskStatus::Ok;
    for (rate, f, tol, depth) in [
        (0, 1500.0, 100.0, 3),
        (12_000, 1500.0, 100.0, 0),
        (12_000, 1500.0, 100.0, 4),
        (12_000, f32::NAN, 100.0, 3),
        (12_000, 1500.0, -1.0, 3),
    ] {
        let rx = unsafe { mfsk_msk144_open(rate, f, tol, depth, &mut st) };
        assert!(rx.is_null());
        assert_eq!(st, MfskStatus::InvalidArg);
    }
    let mut d = empty();
    assert_eq!(
        unsafe { mfsk_msk144_poll(ptr::null_mut(), &mut d) },
        MfskStatus::NullPointer as i32
    );
    let rx = open(12_000);
    assert_eq!(
        unsafe { mfsk_msk144_poll(rx, ptr::null_mut()) },
        MfskStatus::NullPointer as i32
    );
    assert_eq!(
        unsafe { mfsk_msk144_push_i16(rx, ptr::null(), 5) },
        MfskStatus::NullPointer
    );
    assert_eq!(
        unsafe { mfsk_msk144_push_i16(rx, ptr::null(), 0) },
        MfskStatus::Ok
    );
    unsafe { mfsk_msk144_close(rx) };
    unsafe { mfsk_msk144_close(ptr::null_mut()) };
}

/// Transmit then receive: `mfsk_encode_msk144`'s audio through the handle.
#[test]
fn a_transmission_comes_back_through_the_receiver() {
    let c = |s: &str| std::ffi::CString::new(s).unwrap();
    let (c1, c2, rep) = (c("JA1ABC"), c("K1JT"), c("PM95"));
    let mut need = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_msk144(
                c1.as_ptr(),
                c2.as_ptr(),
                rep.as_ptr(),
                15,
                ptr::null_mut(),
                0,
                &mut need,
            )
        },
        MfskStatus::InvalidArg,
        "a size query reports the length through out_len"
    );
    assert_eq!(need, 15 * 12_000 - 6_000);
    let mut pcm = vec![0f32; need];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_msk144(
                c1.as_ptr(),
                c2.as_ptr(),
                rep.as_ptr(),
                15,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    for x in &mut pcm {
        *x *= 0.3;
    }
    let mut st = MfskStatus::Internal;
    let rx = unsafe { mfsk_msk144_open(12_000, 1500.0, 100.0, 3, &mut st) };
    assert_eq!(
        unsafe { mfsk_msk144_push_f32(rx, pcm.as_ptr(), pcm.len()) },
        MfskStatus::Ok
    );
    let got = drain(rx);
    unsafe { mfsk_msk144_close(rx) };
    assert!(
        got.iter()
            .any(|d| d.0 == "JA1ABC K1JT PM95" && (d.2 - 1500.0).abs() < 5.0),
        "{got:?}"
    );

    assert_eq!(
        unsafe {
            mfsk_encode_msk144(
                c1.as_ptr(),
                c2.as_ptr(),
                rep.as_ptr(),
                0,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut n,
            )
        },
        MfskStatus::InvalidArg,
        "period 0"
    );
    let bad = c("NOT A CALL");
    assert_eq!(
        unsafe {
            mfsk_encode_msk144(
                bad.as_ptr(),
                c2.as_ptr(),
                rep.as_ptr(),
                15,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut n,
            )
        },
        MfskStatus::InvalidArg,
        "a message that does not pack"
    );
}
