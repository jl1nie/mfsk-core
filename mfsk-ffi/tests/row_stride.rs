//! #607: a caller's array of `MfskDecode` is stepped by the stride its first
//! row declares (`sizeof(MfskDecode)` as the caller's header has it), not by
//! this library's `sizeof`. A caller built against an older, shorter struct
//! gets every row where it put it and nothing past its buffer; one built
//! against a newer, longer struct keeps each row's tail.

mod common;
use common::*;
use mfsk::*;
use std::mem::{offset_of, size_of};

const GUARD: u8 = 0xAB;
const FULL: usize = size_of::<MfskDecode>();

/// Two stations in one FT8 slot.
fn two_station_slot() -> Vec<i16> {
    slot_of([("CQ", "JA1ABC", "PM95"), ("CQ", "K1JT", "FN20")])
}

fn slot_of(m: [(&str, &str, &str); 2]) -> Vec<i16> {
    let a = synth_slot_i16(MfskMode::Ft8, m[0].0, m[0].1, m[0].2, 1_000.0);
    let b = synth_slot_i16(MfskMode::Ft8, m[1].0, m[1].1, m[1].2, 1_800.0);
    a.iter()
        .zip(&b)
        .map(|(x, y)| x.saturating_add(*y))
        .collect()
}

/// The rows a caller with this library's own header gets.
fn reference(slot: &[i16]) -> Vec<String> {
    let d = open(MfskMode::Ft8, None, None);
    let mut rows = vec![blank_row(); 16];
    let mut n = 0;
    let st = unsafe {
        mfsk_decoder_decode_i16(
            d,
            slot.as_ptr(),
            slot.len(),
            12_000,
            3,
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    unsafe { mfsk_decoder_close(d) };
    rows.truncate(n);
    texts(&rows)
}

/// Decode into a byte buffer of `cap` rows `stride` apart, plus a guard tail,
/// every byte `GUARD` except the first row's `size`.
fn decode_at(
    slot: &[i16],
    stride: usize,
    cap: usize,
    declared: u32,
) -> (MfskStatus, usize, Vec<u8>) {
    let tail = 64;
    let mut buf = vec![GUARD; cap * stride + tail];
    buf[..4].copy_from_slice(&declared.to_ne_bytes());
    let d = open(MfskMode::Ft8, None, None);
    let mut n = 0;
    let st = unsafe {
        mfsk_decoder_decode_i16(
            d,
            slot.as_ptr(),
            slot.len(),
            12_000,
            3,
            buf.as_mut_ptr() as *mut MfskDecode,
            cap,
            &mut n,
        )
    };
    unsafe { mfsk_decoder_close(d) };
    (st, n, buf)
}

fn size_at(buf: &[u8], off: usize) -> u32 {
    u32::from_ne_bytes(buf[off..off + 4].try_into().unwrap())
}

fn text_at(buf: &[u8], off: usize) -> String {
    let t = &buf[off + offset_of!(MfskDecode, text)..][..MFSK_DECODE_TEXT_LEN];
    let end = t.iter().position(|&b| b == 0).unwrap_or(t.len());
    String::from_utf8_lossy(&t[..end]).into_owned()
}

#[test]
fn an_older_callers_shorter_rows_are_written_at_its_stride() {
    let slot = two_station_slot();
    let want = reference(&slot);
    assert!(want.len() >= 2, "{want:?}");
    // A header from before the last appended fields.
    let old = offset_of!(MfskDecode, stage) & !3;
    assert!(old < FULL);
    let cap = want.len();
    let (st, n, buf) = decode_at(&slot, old, cap, old as u32);
    assert_eq!(st, MfskStatus::Ok);
    assert_eq!(n, want.len());
    let got: Vec<String> = (0..n).map(|i| text_at(&buf, i * old)).collect();
    assert_eq!(got, want, "every row where the caller put it");
    for i in 0..n {
        assert_eq!(
            size_at(&buf, i * old),
            old as u32,
            "row {i} says what was written"
        );
    }
    assert!(
        buf[cap * old..].iter().all(|&b| b == GUARD),
        "nothing past the caller's {cap} rows"
    );
}

#[test]
fn a_newer_callers_longer_rows_keep_their_tails() {
    let slot = two_station_slot();
    let want = reference(&slot);
    let new = FULL + 16;
    let cap = want.len();
    let (st, n, buf) = decode_at(&slot, new, cap, new as u32);
    assert_eq!(st, MfskStatus::Ok);
    let got: Vec<String> = (0..n).map(|i| text_at(&buf, i * new)).collect();
    assert_eq!(got, want);
    for i in 0..n {
        assert_eq!(
            size_at(&buf, i * new),
            new as u32,
            "row {i}: the caller's stride, which the next reader of the array steps by (#635)"
        );
        assert!(
            buf[i * new + FULL..(i + 1) * new]
                .iter()
                .all(|&b| b == GUARD),
            "row {i}'s tail is the caller's"
        );
    }
    assert!(buf[cap * new..].iter().all(|&b| b == GUARD));
}

#[test]
fn a_zero_size_is_this_headers_struct_and_a_bad_one_is_refused() {
    let slot = two_station_slot();
    let want = reference(&slot);
    let (st, n, buf) = decode_at(&slot, FULL, want.len(), 0);
    assert_eq!(st, MfskStatus::Ok);
    let got: Vec<String> = (0..n).map(|i| text_at(&buf, i * FULL)).collect();
    assert_eq!(got, want);
    for bad in [2u32, 6, 7] {
        let (st, _, buf) = decode_at(&slot, FULL, want.len(), bad);
        assert_eq!(st, MfskStatus::InvalidArg, "size {bad}");
        assert!(
            buf[4..].iter().all(|&b| b == GUARD),
            "size {bad}: nothing written"
        );
    }
}

#[test]
fn the_q65_history_reads_rows_at_the_callers_stride() {
    // Not CQs: the lookup passes those over.
    let slot = slot_of([("K1JT", "JA1ABC", "PM95"), ("K1ABC", "W9XYZ", "EN37")]);
    let want = reference(&slot);
    assert_eq!(want.len(), 2, "{want:?}");
    let old = offset_of!(MfskDecode, stage) & !3;
    let (st, n, buf) = decode_at(&slot, old, want.len(), old as u32);
    assert_eq!(st, MfskStatus::Ok);
    let h = mfsk_q65_history_new();
    assert_eq!(
        unsafe { mfsk_q65_history_record(h, buf.as_ptr() as *const MfskDecode, n) },
        MfskStatus::Ok
    );
    assert_eq!(
        unsafe { mfsk_q65_history_len(h) },
        n,
        "every row, none from between them"
    );
    // Each station at its own frequency, read from where the caller put it.
    for (hz, call) in [(1_000.0f32, "JA1ABC"), (1_800.0, "W9XYZ")] {
        let mut dx: MfskQ65Dx = unsafe { std::mem::zeroed() };
        dx.size = size_of::<MfskQ65Dx>() as u32;
        assert_eq!(
            unsafe { mfsk_q65_history_lookup(h, hz, &mut dx) },
            MfskStatus::Ok,
            "{hz} Hz"
        );
        let got = unsafe { std::ffi::CStr::from_ptr(dx.call.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        assert_eq!(got, call, "{hz} Hz");
    }
    // A size that stops before the text cannot be read.
    let mut short = buf.clone();
    short[..4].copy_from_slice(&8u32.to_ne_bytes());
    assert_eq!(
        unsafe { mfsk_q65_history_record(h, short.as_ptr() as *const MfskDecode, n) },
        MfskStatus::InvalidArg
    );
    unsafe { mfsk_q65_history_free(h) };
}

/// #635: rows decoded into a newer caller's longer array go straight to the
/// history, which steps by `rows[0].size`: that is the caller's stride, not
/// this library's `sizeof`, or row 1 would be read from inside row 0's tail.
#[test]
fn a_newer_callers_rows_go_straight_to_the_q65_history() {
    let slot = slot_of([("K1JT", "JA1ABC", "PM95"), ("K1ABC", "W9XYZ", "EN37")]);
    let want = reference(&slot);
    assert_eq!(want.len(), 2, "{want:?}");
    let new = FULL + 16;
    let (st, n, buf) = decode_at(&slot, new, want.len(), new as u32);
    assert_eq!(st, MfskStatus::Ok);
    let before = buf.clone();
    let h = mfsk_q65_history_new();
    assert_eq!(
        unsafe { mfsk_q65_history_record(h, buf.as_ptr() as *const MfskDecode, n) },
        MfskStatus::Ok
    );
    assert_eq!(unsafe { mfsk_q65_history_len(h) }, n);
    for (hz, call) in [(1_000.0f32, "JA1ABC"), (1_800.0, "W9XYZ")] {
        let mut dx: MfskQ65Dx = unsafe { std::mem::zeroed() };
        dx.size = size_of::<MfskQ65Dx>() as u32;
        assert_eq!(
            unsafe { mfsk_q65_history_lookup(h, hz, &mut dx) },
            MfskStatus::Ok,
            "{hz} Hz"
        );
        let got = unsafe { std::ffi::CStr::from_ptr(dx.call.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        assert_eq!(got, call, "{hz} Hz");
    }
    assert_eq!(buf, before, "the history only reads");
    for i in 0..n {
        assert!(
            buf[i * new + FULL..(i + 1) * new]
                .iter()
                .all(|&b| b == GUARD),
            "row {i}'s tail is the caller's"
        );
    }
    unsafe { mfsk_q65_history_free(h) };
}
