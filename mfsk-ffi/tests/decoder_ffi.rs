//! The decoder handle through the C ABI: open with the parameter block and
//! the library's options, decode a period, rows streamed as they are found,
//! state kept between periods, options refused for modes that lack them.
//!
//! Where the Rust side has the test of the behaviour (the depth mappings,
//! AP, the snapshot), this is about the boundary: size-versioned structs,
//! status codes, ownership, callbacks.

mod common;
use common::*;
use mfsk::*;
use std::ffi::{CString, c_void};

fn put_wav(frame: &[f32], offset_s: f32, slot_s: usize) -> Vec<f32> {
    let mut slot = vec![0f32; slot_s * 12_000];
    let at = (offset_s * 12_000.0) as usize;
    for (i, v) in frame.iter().enumerate() {
        if let Some(d) = slot.get_mut(at + i) {
            *d += *v;
        }
    }
    slot
}

fn pcm(
    enc: unsafe extern "C" fn(
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        f32,
        *mut f32,
        usize,
        *mut usize,
    ) -> MfskStatus,
    a: &str,
    b: &str,
    c: &str,
) -> Vec<f32> {
    encode_f32(enc, a, b, c, 1_500.0)
}

// ── The structs ───────────────────────────────────────────────────────────

#[test]
fn init_writes_the_modes_defaults() {
    let p = params(MfskMode::Ft8);
    assert_eq!(p.size as usize, std::mem::size_of::<MfskParams>());
    assert_eq!(p.depth, MFSK_DEPTH_DEEP, "the GUI's default");
    assert_eq!(
        p.ap_mode, MFSK_AP_OFF,
        "FT8's Enable AP box starts unchecked"
    );
    assert!(p.rx_freq_hz.is_nan() && p.tol_hz.is_nan() && p.tx_freq_hz.is_nan());
    assert_eq!((p.band_lo_hz, p.band_hi_hz), (200.0, 4000.0));
    assert_eq!(params(MfskMode::Ft4).ap_mode, MFSK_AP_FULL);
    assert_eq!(
        (
            params(MfskMode::Fst4s60).band_lo_hz,
            params(MfskMode::Fst4s60).band_hi_hz
        ),
        (600.0, 1400.0)
    );
    let e = extras();
    assert!(e.sync_min.is_nan() && e.max_cand == 0 && e.osd == -1 && e.strictness == -1);
    // A mode that is not decoded as a slot has no block.
    let mut q = unsafe { std::mem::zeroed::<MfskParams>() };
    assert_eq!(
        unsafe { mfsk_params_init(MfskMode::Msk144 as u32, &mut q) },
        MfskStatus::UnknownProtocol
    );
    assert_eq!(
        unsafe { mfsk_params_init(9_999, &mut q) },
        MfskStatus::InvalidArg
    );
}

#[test]
fn a_bad_block_is_refused_not_clamped() {
    let try_open = |p: &MfskParams| {
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_decoder_open(MfskMode::Ft8 as u32, p, std::ptr::null(), &mut st) };
        if !d.is_null() {
            unsafe { mfsk_decoder_close(d) };
        }
        st
    };
    let mut p = params(MfskMode::Ft8);
    assert_eq!(try_open(&p), MfskStatus::Ok);
    p.depth = 7;
    assert_eq!(try_open(&p), MfskStatus::InvalidArg);
    let mut p = params(MfskMode::Ft8);
    p.band_hi_hz = p.band_lo_hz;
    assert_eq!(try_open(&p), MfskStatus::InvalidArg);
    let mut p = params(MfskMode::Ft8);
    p.band_lo_hz = f32::NAN;
    assert_eq!(try_open(&p), MfskStatus::InvalidArg);
    let mut p = params(MfskMode::Ft8);
    p.contest = 5;
    assert_eq!(try_open(&p), MfskStatus::InvalidArg);
    let mut p = params(MfskMode::Ft8);
    p.qso_progress = 6;
    assert_eq!(try_open(&p), MfskStatus::InvalidArg);
}

#[test]
fn an_option_the_mode_lacks_is_unsupported() {
    let try_open = |mode: MfskMode, e: &MfskExtras| {
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_decoder_open(mode as u32, std::ptr::null(), e, &mut st) };
        if !d.is_null() {
            unsafe { mfsk_decoder_close(d) };
        }
        st
    };
    // FST4 has no subtraction (fst4_decode.f90), FT4 no a7, WSPR no AP.
    let mut e = extras();
    e.strategy = MFSK_STRATEGY_SIC_ROUNDS;
    e.sic_rounds = 2;
    assert_eq!(try_open(MfskMode::Ft8, &e), MfskStatus::Ok);
    assert_eq!(try_open(MfskMode::Ft4, &e), MfskStatus::Ok);
    assert_eq!(try_open(MfskMode::Fst4s60, &e), MfskStatus::Unsupported);
    assert_eq!(try_open(MfskMode::Wspr, &e), MfskStatus::Unsupported);
    let mut e = extras();
    e.a7 = 1;
    assert_eq!(try_open(MfskMode::Ft8, &e), MfskStatus::Ok);
    assert_eq!(try_open(MfskMode::Ft4, &e), MfskStatus::Unsupported);
    let mut e = extras();
    with_ap(&mut e, "CQ", "", "");
    assert_eq!(try_open(MfskMode::Ft8, &e), MfskStatus::Ok);
    assert_eq!(try_open(MfskMode::Wspr, &e), MfskStatus::Unsupported);
    assert_eq!(try_open(MfskMode::Jt9, &e), MfskStatus::Unsupported);
    let mut e = extras();
    e.nb_percent = 5;
    assert_eq!(try_open(MfskMode::Fst4s60, &e), MfskStatus::Ok);
    assert_eq!(try_open(MfskMode::Ft8, &e), MfskStatus::Unsupported);
    // An out-of-range value is the caller's mistake, not a missing option.
    let mut e = extras();
    e.nb_percent = 99;
    assert_eq!(try_open(MfskMode::Fst4s60, &e), MfskStatus::InvalidArg);
    let mut e = extras();
    e.strictness = 9;
    assert_eq!(try_open(MfskMode::Ft8, &e), MfskStatus::InvalidArg);
}

// ── Decoding ──────────────────────────────────────────────────────────────

#[test]
fn ft8_decodes_a_period_and_reports_its_detail() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let dec = open(MfskMode::Ft8, None, None);
    let rows = decode_i16(dec, &slot);
    assert!(any_contains(&rows, "CQ JA1ABC PM95"), "{:?}", texts(&rows));
    let r = &rows[0];
    assert_eq!(r.mode, MfskMode::Ft8);
    assert!((r.freq_hz - 1_500.0).abs() < 2.0);
    assert_eq!(r.info_bits, 91);
    // FT8 reports its sync score, fading measure and error count (#594).
    let all = MFSK_DECODE_FLAG_HAS_SYNC_SCORE
        | MFSK_DECODE_FLAG_HAS_SYNC_CV
        | MFSK_DECODE_FLAG_HAS_HARD_ERRORS;
    assert_eq!(r.flags & all, all, "flags {:#x}", r.flags);
    let mut bits = [0u8; 128];
    let mut n = 0usize;
    assert_eq!(
        unsafe { mfsk_decoder_copy_info(dec, 0, bits.as_mut_ptr(), bits.len(), &mut n) },
        MfskStatus::Ok
    );
    assert_eq!(n, 91);
    // The row's key is the first 77 bits of that block, packed (#592).
    assert_eq!(r.key_bits, 77);
    let mut packed = [0u8; MFSK_DECODE_KEY_LEN];
    for (i, b) in bits[..77].iter().enumerate() {
        packed[i / 8] |= (b & 1) << (7 - i % 8);
    }
    assert_eq!(r.key, packed);
    assert_eq!(
        r.delivery, -1,
        "no callback was set, so no delivery to name"
    );
    assert_eq!(
        unsafe { mfsk_decoder_copy_info(dec, 9, bits.as_mut_ptr(), bits.len(), &mut n) },
        MfskStatus::InvalidArg
    );
    unsafe { mfsk_decoder_close(dec) };
}

#[test]
fn a_short_output_array_reports_the_count_needed() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let dec = open(MfskMode::Ft8, None, None);
    let mut n = 0usize;
    let st = unsafe {
        mfsk_decoder_decode_i16(
            dec,
            slot.as_ptr(),
            slot.len(),
            FS,
            MFSK_PERIOD_NONE,
            std::ptr::null_mut(),
            0,
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::InvalidArg);
    assert!(n >= 1, "{n}");
    unsafe { mfsk_decoder_close(dec) };
}

#[test]
fn f32_audio_reaches_the_engines_at_any_level() {
    // A quiet float buffer, as a radio adapter at a low volume gives.
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let quiet: Vec<f32> = slot.iter().map(|&s| s as f32 / 32_768.0 * 0.002).collect();
    let dec = open(MfskMode::Ft8, None, None);
    assert!(any_contains(&decode_f32(dec, &quiet), "CQ JA1ABC PM95"));
    unsafe { mfsk_decoder_close(dec) };
}

#[test]
fn rows_arrive_through_the_callback_as_they_are_found() {
    extern "C" fn on_row(row: *const MfskDecode, user: *mut c_void) {
        let seen = unsafe { &mut *(user as *mut Vec<(String, i32)>) };
        let row = unsafe { &*row };
        seen.push((text_of(row), row.delivery));
    }
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let dec = open(MfskMode::Ft8, None, None);
    let mut seen: Vec<(String, i32)> = Vec::new();
    assert_eq!(
        unsafe {
            mfsk_decoder_set_on_decode(dec, Some(on_row), &mut seen as *mut _ as *mut c_void)
        },
        MfskStatus::Ok
    );
    let rows = decode_i16(dec, &slot);
    let seen_texts: Vec<String> = seen.iter().map(|s| s.0.clone()).collect();
    assert_eq!(
        seen_texts,
        texts(&rows),
        "the callback saw what the array holds"
    );
    assert!(!seen.is_empty());
    // The callback's rows carry their position; each returned row names the
    // delivery it was (#592).
    for (i, s) in seen.iter().enumerate() {
        assert_eq!(s.1, i as i32);
    }
    for (i, r) in rows.iter().enumerate() {
        assert_eq!(r.delivery, i as i32, "{}", text_of(r));
    }
    unsafe { mfsk_decoder_close(dec) };
}

#[test]
fn a_budget_that_refuses_everything_returns_nothing_and_says_so() {
    extern "C" fn never(_: *mut c_void) -> bool {
        false
    }
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let dec = open(MfskMode::Ft8, None, None);
    assert_eq!(
        unsafe { mfsk_decoder_set_budget(dec, Some(never), std::ptr::null_mut()) },
        MfskStatus::Ok
    );
    assert!(decode_i16(dec, &slot).is_empty());
    let mut rep = unsafe { std::mem::zeroed::<MfskBudgetReport>() };
    assert_eq!(
        unsafe { mfsk_decoder_last_budget(dec, &mut rep) },
        MfskStatus::Ok
    );
    assert!(rep.exhausted);
    unsafe { mfsk_decoder_close(dec) };
}

/// Every mode with a decoder takes a budget and publishes `MFSK_CAP_BUDGET`
/// (#593: WSPR, JT9, JT65 and Q65 poll it too). Each mode's behaviour under
/// one is `mfsk-core/tests/decoder_budget.rs`; this pins the C gate.
#[test]
fn every_mode_with_a_decoder_takes_a_budget() {
    extern "C" fn always(_: *mut c_void) -> bool {
        true
    }
    let mut opened = 0;
    for i in 0..mfsk_mode_count() {
        let mut m = MfskMode::Ft8;
        assert_eq!(unsafe { mfsk_mode_at(i, &mut m) }, MfskStatus::Ok);
        let m = m as u32;
        let mut st = MfskStatus::Ok;
        let d = unsafe { mfsk_decoder_open(m, std::ptr::null(), std::ptr::null(), &mut st) };
        if d.is_null() {
            continue; // uvpacket, MSK144, JTTY: no slot decoder
        }
        opened += 1;
        assert_ne!(mfsk_mode_caps(m) & MFSK_CAP_BUDGET, 0, "mode {m}");
        assert_eq!(
            unsafe { mfsk_decoder_set_budget(d, Some(always), std::ptr::null_mut()) },
            MfskStatus::Ok,
            "mode {m}"
        );
        unsafe { mfsk_decoder_close(d) };
    }
    assert!(opened >= 20, "{opened} decoders opened");
}

/// `rows_subtracted` reaches C: FT8's default (`SicEarly`) subtracts the
/// checkpoint-A row at B, and a budget that never says stop leaves it there.
#[test]
fn the_budget_report_counts_sic_early_subtractions() {
    extern "C" fn always(_: *mut c_void) -> bool {
        true
    }
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let dec = open(MfskMode::Ft8, None, None);
    assert!(
        unsafe { mfsk_decoder_set_budget(dec, Some(always), std::ptr::null_mut()) }
            == MfskStatus::Ok
    );
    let rows = decode_i16(dec, &slot);
    assert_eq!(rows.len(), 1);
    let mut rep = unsafe { std::mem::zeroed::<MfskBudgetReport>() };
    assert_eq!(
        unsafe { mfsk_decoder_last_budget(dec, &mut rep) },
        MfskStatus::Ok
    );
    assert!(!rep.exhausted);
    assert_eq!(rep.rows_subtracted, 1);
    assert_eq!(rep.size as usize, std::mem::size_of::<MfskBudgetReport>());
    unsafe { mfsk_decoder_close(dec) };
}

/// `mfsk_decoder_delivery_is_exact` is `AnyDecoder::delivery_is_exact`: it
/// follows the mode, the depth and the strategy (`STREAMING.md` §3).
#[test]
fn delivery_is_exact_follows_mode_depth_and_strategy() {
    let ft8 = open(MfskMode::Ft8, None, None);
    assert!(
        unsafe { mfsk_decoder_delivery_is_exact(ft8) },
        "FT8 SicEarly is exact"
    );
    let mut e = extras();
    e.strategy = MFSK_STRATEGY_SINGLE_PASS;
    assert_eq!(unsafe { mfsk_decoder_set_extras(ft8, &e) }, MfskStatus::Ok);
    assert!(
        !unsafe { mfsk_decoder_delivery_is_exact(ft8) },
        "FT8 single pass is not"
    );
    unsafe { mfsk_decoder_close(ft8) };

    let mut p = params(MfskMode::Ft4);
    p.depth = MFSK_DEPTH_FAST;
    let ft4 = open(MfskMode::Ft4, Some(&p), None);
    assert!(
        !unsafe { mfsk_decoder_delivery_is_exact(ft4) },
        "FT4 Fast is single pass"
    );
    p.depth = MFSK_DEPTH_DEEP;
    assert_eq!(unsafe { mfsk_decoder_set_params(ft4, &p) }, MfskStatus::Ok);
    assert!(
        unsafe { mfsk_decoder_delivery_is_exact(ft4) },
        "FT4 Deep is SicRounds"
    );
    unsafe { mfsk_decoder_close(ft4) };

    for (m, exact) in [
        (MfskMode::Wspr, false),
        (MfskMode::Jt9, true),
        (MfskMode::Jt65, true),
    ] {
        let d = open(m, None, None);
        assert_eq!(unsafe { mfsk_decoder_delivery_is_exact(d) }, exact, "{m:?}");
        unsafe { mfsk_decoder_close(d) };
    }
    assert!(!unsafe { mfsk_decoder_delivery_is_exact(std::ptr::null()) });
}

/// The decoder's callsign table is its own and outlives the period: a
/// `<...>` that period 0 introduced reads as the call in period 1 — on the
/// same decoder, and not on another.
#[test]
fn a_hashed_call_resolves_in_the_decoder_that_heard_it() {
    // Type 4: a non-standard call, with a hashed standard one.
    let pack4 = |a: &str, b: &str| {
        let (a, b) = (CString::new(a).unwrap(), CString::new(b).unwrap());
        let mut m = [0u8; 77];
        assert_eq!(
            unsafe {
                mfsk_pack77_type4(
                    a.as_ptr(),
                    b.as_ptr(),
                    std::ptr::null(),
                    false,
                    m.as_mut_ptr(),
                )
            },
            MfskStatus::Ok
        );
        m
    };
    let frame = |msg: &[u8; 77]| -> Vec<i16> {
        let mut tones = vec![0u8; mfsk_symbol_count(MfskMode::Ft8 as u32)];
        let mut n = 0usize;
        assert_eq!(
            unsafe {
                mfsk_message_to_tones(
                    MfskMode::Ft8 as u32,
                    msg.as_ptr(),
                    tones.as_mut_ptr(),
                    tones.len(),
                    &mut n,
                )
            },
            MfskStatus::Ok
        );
        let mut pcm = vec![0i16; mfsk_synth_output_len(MfskMode::Ft8 as u32)];
        let mut w = 0usize;
        assert_eq!(
            unsafe {
                mfsk_tones_to_i16(
                    MfskMode::Ft8 as u32,
                    tones.as_ptr(),
                    tones.len(),
                    1_500.0,
                    8_000,
                    pcm.as_mut_ptr(),
                    pcm.len(),
                    &mut w,
                )
            },
            MfskStatus::Ok
        );
        let mut slot = vec![0i16; 180_000];
        for (i, s) in pcm.iter().enumerate() {
            slot[6_000 + i] = *s;
        }
        slot
    };
    // Period 0: the standard call is sent in full (a plain message).
    let heard = synth_slot_i16(MfskMode::Ft8, "CQ", "VK3NV", "QF22", 1_500.0);
    // Period 1: VK3NV is only a hash.
    let hashed = frame(&pack4("JA1ABC/QRP", "VK3NV"));

    let a = open(MfskMode::Ft8, None, None);
    let b = open(MfskMode::Ft8, None, None);
    assert!(any_contains(&decode_i16_at(a, &heard, 10), "VK3NV"));
    let with = decode_i16_at(a, &hashed, 11);
    let without = decode_i16_at(b, &hashed, 11);
    // `a` learned VK3NV in period 10; `b` never heard it.
    assert!(
        texts(&without).iter().any(|t| t.contains("<...>")),
        "{:?}",
        texts(&without)
    );
    assert!(
        texts(&with).iter().any(|t| t.contains("<VK3NV>")),
        "{:?}",
        texts(&with)
    );
    assert!(
        with.iter()
            .any(|r| r.flags & MFSK_DECODE_FLAG_HASH_RESOLVED != 0)
    );
    // Teaching it from outside works too, and clearing forgets.
    let c = open(MfskMode::Ft8, None, None);
    let call = CString::new("VK3NV").unwrap();
    assert_eq!(
        unsafe { mfsk_decoder_add_callsign(c, call.as_ptr()) },
        MfskStatus::Ok
    );
    assert!(
        texts(&decode_i16(c, &hashed))
            .iter()
            .any(|t| t.contains("<VK3NV>"))
    );
    assert_eq!(unsafe { mfsk_decoder_clear(c) }, MfskStatus::Ok);
    assert!(
        texts(&decode_i16(c, &hashed))
            .iter()
            .any(|t| t.contains("<...>"))
    );
    // A mode whose messages carry no hashes says so.
    let w = open(MfskMode::Wspr, None, None);
    assert_eq!(
        unsafe { mfsk_decoder_add_callsign(w, call.as_ptr()) },
        MfskStatus::Unsupported
    );
    for d in [a, b, c, w] {
        unsafe { mfsk_decoder_close(d) };
    }
}

#[test]
fn the_other_modes_decode_through_the_same_handle() {
    // WSPR
    let w = pcm_wspr();
    let dec = open(MfskMode::Wspr, None, None);
    let rows = decode_f32(dec, &put_wav(&w, 1.0, 120));
    assert!(any_contains(&rows, "K1ABC FN42 37"), "{:?}", texts(&rows));
    unsafe { mfsk_decoder_close(dec) };

    // JT9 and JT65: the frame starts the period.
    for (mode, enc, text) in [
        (MfskMode::Jt9, mfsk_encode_jt9 as _, "CQ K1ABC FN42"),
        (MfskMode::Jt65, mfsk_encode_jt65 as _, "CQ K1ABC FN42"),
    ] {
        let frame = pcm(enc, "CQ", "K1ABC", "FN42");
        let dec = open(mode, None, None);
        let rows = decode_f32(dec, &put_wav(&frame, 0.0, 60));
        assert!(any_contains(&rows, text), "{mode:?}: {:?}", texts(&rows));
        unsafe { mfsk_decoder_close(dec) };
    }

    // Q65-30A, with the Pileup flag round-tripped into the row.
    let mut buf = vec![0f32; 600_000];
    let mut n = 0usize;
    let (a, b, c) = (
        CString::new("CQ").unwrap(),
        CString::new("K1ABC").unwrap(),
        CString::new("FN42").unwrap(),
    );
    assert_eq!(
        unsafe {
            mfsk_encode_q65(
                MfskQ65SubMode::A30 as u32,
                a.as_ptr(),
                b.as_ptr(),
                c.as_ptr(),
                1_500.0,
                buf.as_mut_ptr(),
                buf.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    buf.truncate(n);
    let mut e = extras();
    e.t_early_s = 1.0;
    e.t_late_s = 1.0;
    let dec = open(MfskMode::Q65a30, None, Some(&e));
    let rows = decode_f32(dec, &put_wav(&buf, 0.5, 30));
    assert!(any_contains(&rows, "CQ K1ABC FN42"), "{:?}", texts(&rows));
    unsafe { mfsk_decoder_close(dec) };
}

fn pcm_wspr() -> Vec<f32> {
    let (c, g) = (
        CString::new("K1ABC").unwrap(),
        CString::new("FN42").unwrap(),
    );
    let mut buf = vec![0f32; 1_500_000];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_wspr(
                c.as_ptr(),
                g.as_ptr(),
                37,
                1_500.0,
                buf.as_mut_ptr(),
                buf.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    buf.truncate(n);
    buf
}

// ── Streams and the clock ─────────────────────────────────────────────────

/// A stream cuts slots on the UTC grid; the slot comes with its index, and a
/// clock reading is followed, not jumped to.
#[test]
fn a_stream_cuts_on_the_grid_and_the_decoder_reads_the_period() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let mut st = MfskStatus::Internal;
    let s = unsafe { mfsk_stream_open(MfskMode::Ft8 as u32, 12_000, &mut st) };
    assert_eq!(st, MfskStatus::Ok);
    // 15 s boundary + a quarter of a second of lead-in, and the clock says so.
    let t0: i64 = 1_700_000_010 * 1_000_000_000;
    let lead = vec![0i16; 3_000];
    unsafe { mfsk_stream_push_i16(s, lead.as_ptr(), lead.len()) };
    let mut change = -1;
    assert_eq!(
        unsafe { mfsk_stream_set_time(s, t0 - 250_000_000, 0, &mut change) },
        MfskStatus::Ok
    );
    assert_eq!(change, MFSK_CLOCK_FIRST);
    // The recording from the next boundary on, in odd chunks.
    let mut audio = vec![0i16; 0];
    audio.extend_from_slice(&slot);
    audio.extend(std::iter::repeat_n(0i16, 12_000));
    for chunk in audio.chunks(7_777) {
        unsafe { mfsk_stream_push_i16(s, chunk.as_ptr(), chunk.len()) };
    }
    assert!(mfsk_stream_slot_ready(s));
    let dec = open(MfskMode::Ft8, None, None);
    let mut rows = vec![blank_row(); 32];
    let (mut n, mut period, mut utc) = (0usize, 0i64, 0i64);
    assert_eq!(
        unsafe {
            mfsk_decoder_decode_stream(
                dec,
                s,
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
                &mut period,
                &mut utc,
            )
        },
        MfskStatus::Ok
    );
    rows.truncate(n);
    assert!(any_contains(&rows, "CQ JA1ABC PM95"), "{:?}", texts(&rows));
    assert_eq!(
        period,
        1_700_000_010 / 15,
        "the boundary the lead-in ends on"
    );
    assert_eq!(utc, period * 15_000_000_000);
    // Nothing more is ready.
    assert!(!mfsk_stream_slot_ready(s));
    assert_eq!(
        unsafe {
            mfsk_decoder_decode_stream(
                dec,
                s,
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        },
        MfskStatus::Unsupported
    );
    unsafe { mfsk_decoder_close(dec) };
    unsafe { mfsk_stream_close(s) };
}

#[test]
fn a_stream_and_a_decoder_of_different_modes_do_not_mix() {
    let mut st = MfskStatus::Internal;
    let s = unsafe { mfsk_stream_open(MfskMode::Ft4 as u32, 12_000, &mut st) };
    assert_eq!(st, MfskStatus::Ok);
    let dec = open(MfskMode::Ft8, None, None);
    let mut rows = vec![blank_row(); 4];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_decoder_decode_stream(
                dec,
                s,
                rows.as_mut_ptr(),
                4,
                &mut n,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        },
        MfskStatus::InvalidArg
    );
    unsafe { mfsk_decoder_close(dec) };
    unsafe { mfsk_stream_close(s) };
    // A mode that is not cut into slots has no stream.
    let jt = unsafe { mfsk_stream_open(MfskMode::Jtty as u32, 12_000, &mut st) };
    assert!(jt.is_null());
}

/// `jt9fano.f90:88` blanks the all-zero codeword (`000AAA 000AAA RA90`), which
/// Fano reached three times on the strong signal's sidelobes before the port.
#[test]
fn jt9_reports_the_signal_and_no_all_zero_codewords() {
    let frame = pcm(mfsk_encode_jt9 as _, "CQ", "K1ABC", "FN42");
    let dec = open(MfskMode::Jt9, None, None);
    let rows = decode_f32(dec, &put_wav(&frame, 0.0, 60));
    assert_eq!(texts(&rows), vec!["CQ K1ABC FN42".to_string()]);
    unsafe { mfsk_decoder_close(dec) };
}

/// `mfsk_decoder_decode_prefix_i16` (#572): FT8's checkpoint A returns its
/// rows early, B returns none, the whole period returns the same rows a
/// whole-period decode does, and the callback sees each row once.
#[test]
fn prefix_calls_deliver_early_and_end_where_decode_does() {
    extern "C" fn count(_: *const MfskDecode, user: *mut c_void) {
        unsafe { *(user as *mut usize) += 1 };
    }
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    assert_eq!(slot.len(), 180_000);
    let whole = {
        let d = open(MfskMode::Ft8, None, None);
        let r = decode_i16_at(d, &slot, 7);
        unsafe { mfsk_decoder_close(d) };
        r
    };
    assert_eq!(whole.len(), 1);
    assert_eq!(whole[0].stage, MFSK_STAGE_NONE);

    let d = open(MfskMode::Ft8, None, None);
    let mut seen = 0usize;
    assert_eq!(
        unsafe {
            mfsk_decoder_set_on_decode(d, Some(count), &mut seen as *mut usize as *mut c_void)
        },
        MfskStatus::Ok
    );
    let prefix = |len: usize| {
        let mut rows = vec![blank_row(); 16];
        let mut n = 0usize;
        let st = unsafe {
            mfsk_decoder_decode_prefix_i16(
                d,
                slot.as_ptr(),
                len,
                12_000,
                7,
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
            )
        };
        assert_eq!(st, MfskStatus::Ok);
        rows.truncate(n);
        rows
    };
    let a = prefix(141_696);
    assert_eq!(a.len(), 1, "checkpoint A finds the one station");
    assert_eq!(a[0].stage, MFSK_STAGE_EARLY);
    assert!(prefix(162_432).is_empty());
    let end = prefix(180_000);
    assert_eq!(end.len(), 1);
    assert_eq!(end[0].text, whole[0].text);
    assert_eq!(end[0].freq_hz.to_bits(), whole[0].freq_hz.to_bits());
    assert_eq!(end[0].stage, MFSK_STAGE_EARLY, "it was returned early");
    assert_eq!(end[0].delivery, 0, "and pairs with the early delivery");
    assert_eq!(seen, 1, "delivered once across the period");
    unsafe { mfsk_decoder_close(d) };
}

/// A prefix call refused for a short buffer has run its stage; the retry the
/// ABI asks for gets the rows, not an empty answer.
#[test]
fn a_prefix_call_retried_for_a_short_buffer_keeps_its_rows() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1_500.0);
    let d = open(MfskMode::Ft8, None, None);
    let mut n = 0usize;
    let st = unsafe {
        mfsk_decoder_decode_prefix_i16(
            d,
            slot.as_ptr(),
            141_696,
            12_000,
            3,
            std::ptr::null_mut(),
            0,
            &mut n,
        )
    };
    assert_eq!((st, n), (MfskStatus::InvalidArg, 1));
    let mut rows = vec![blank_row(); n];
    let st = unsafe {
        mfsk_decoder_decode_prefix_i16(
            d,
            slot.as_ptr(),
            141_696,
            12_000,
            3,
            rows.as_mut_ptr(),
            n,
            &mut n,
        )
    };
    assert_eq!((st, n), (MfskStatus::Ok, 1));
    assert_eq!(rows[0].stage, MFSK_STAGE_EARLY);
    unsafe { mfsk_decoder_close(d) };
}
