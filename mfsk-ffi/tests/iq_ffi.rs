//! The wideband IQ receiver handle (`mfsk_iq_*`), driven the way a C, Kotlin,
//! Swift or C# host drives it: bytes of IQ in, `poll` until it says 0.
//!
//! One synthetic FT8 signal (`CQ JA1ABC PM95` at audio 1500 Hz, 0.5 s into the
//! slot, in white noise) placed as double-sideband IQ at 48 kS/s, quantised to
//! each of the five wire formats, on a UTC grid anchored on a 15 s boundary.

use mfsk::*;
use mfsk_core::engine::dsp::polyphase::PolyphaseResampler;
use std::ffi::{CStr, c_void};
use std::ptr;

mod common;
use common::Awgn;

const FS: u32 = 48_000;
const CENTER: f64 = 14_077_000.0;
const DIAL: f64 = CENTER + 6_000.0;
const T0_NS: i64 = 1_700_000_100 * 1_000_000_000;
const TEXT: &str = "CQ JA1ABC PM95";

/// A 15 s FT8 slot as complex baseband at 48 kS/s, plus half a second of
/// zeros so the last audio sample clears the front end's filters.
fn scene() -> Vec<(f32, f32)> {
    use mfsk_core::Ft8;
    use mfsk_core::msg::wsjt77::pack77;
    let msg = pack77("CQ", "JA1ABC", "PM95").unwrap();
    let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&msg);
    let sig = mfsk_core::engine::tx::synthesize::<Ft8>(&tones, 12_000, 1_500.0, 1.0);
    let mut a = vec![0f32; 15 * 12_000];
    let off = 6_000;
    let n = sig.len().min(a.len() - off);
    a[off..off + n].copy_from_slice(&sig[..n]);
    Awgn::new(1.5, 0x534).apply(&mut a);

    // 12 kHz real audio -> 48 kHz (L/M = 4/1), then mixed up by the offset
    // of the dial from the centre.
    let mut rs = PolyphaseResampler::new(4, 1, 129, 4096);
    let (mut ri, mut rq) = (Vec::new(), Vec::new());
    for &v in &a {
        rs.push(v, 0.0, &mut ri, &mut rq);
    }
    let skip = rs.group_delay_output();
    let w = std::f64::consts::TAU * (DIAL - CENTER) / FS as f64;
    let mut iq: Vec<(f32, f32)> = ri
        .iter()
        .skip(skip)
        .enumerate()
        .map(|(n, &v)| {
            let p = w * n as f64;
            (v * p.cos() as f32, v * p.sin() as f32)
        })
        .collect();
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));
    let peak = iq
        .iter()
        .fold(0f32, |m, &(i, q)| m.max(i.abs()).max(q.abs()));
    iq.iter()
        .map(|&(i, q)| (i * 0.7 / peak, q * 0.7 / peak))
        .collect()
}

fn encode(iq: &[(f32, f32)], format: u32) -> Vec<u8> {
    let mut b = Vec::new();
    for &(i, q) in iq {
        for v in [i, q] {
            match format {
                MFSK_IQ_FORMAT_CF32 => b.extend(v.to_le_bytes()),
                MFSK_IQ_FORMAT_CS16 => b.extend(((v * 32_768.0) as i16).to_le_bytes()),
                MFSK_IQ_FORMAT_CS8 => {
                    b.push(((v * 128.0).round().clamp(-128.0, 127.0) as i8) as u8)
                }
                MFSK_IQ_FORMAT_CU8 => b.push((v * 128.0 + 128.0).round().clamp(0.0, 255.0) as u8),
                _ => b.extend(&((v * 8_388_608.0).round() as i32).to_le_bytes()[..3]),
            }
        }
    }
    b
}

fn open(format: u32) -> *mut MfskIqReceiver {
    open_with(format, MFSK_IQ_CHANNELIZER_DIRECT)
}

fn open_with(format: u32, channelizer: u32) -> *mut MfskIqReceiver {
    let mut st = MfskStatus::Internal;
    let rx = unsafe { mfsk_iq_open_with(FS, CENTER, format, 0, channelizer, &mut st) };
    assert_eq!(st, MfskStatus::Ok);
    assert!(!rx.is_null());
    rx
}

fn text(d: &MfskIqDecode) -> String {
    unsafe { CStr::from_ptr(d.text.as_ptr()) }
        .to_str()
        .unwrap()
        .to_string()
}

fn drain(rx: *mut MfskIqReceiver) -> Vec<MfskIqDecode> {
    let mut out = Vec::new();
    let mut d: MfskIqDecode = unsafe { std::mem::zeroed() };
    while unsafe { mfsk_iq_poll(rx, &mut d) } == 1 {
        assert_eq!(d.size as usize, std::mem::size_of::<MfskIqDecode>());
        out.push(d);
    }
    out
}

#[test]
fn every_format_decodes_the_injected_message() {
    every_format_through(MFSK_IQ_CHANNELIZER_DIRECT);
}

#[test]
fn every_format_decodes_the_injected_message_through_the_pfb() {
    every_format_through(MFSK_IQ_CHANNELIZER_PFB);
}

fn every_format_through(channelizer: u32) {
    let iq = scene();
    for format in [
        MFSK_IQ_FORMAT_CF32,
        MFSK_IQ_FORMAT_CS16,
        MFSK_IQ_FORMAT_CS8,
        MFSK_IQ_FORMAT_CU8,
        MFSK_IQ_FORMAT_CS24,
    ] {
        let bytes = encode(&iq, format);
        let rx = open_with(format, channelizer);
        let mut ch = u32::MAX;
        assert_eq!(
            unsafe {
                mfsk_iq_add_channel(
                    rx,
                    DIAL,
                    MfskMode::Ft8 as u32,
                    ptr::null(),
                    ptr::null(),
                    &mut ch,
                )
            },
            MfskStatus::Ok
        );
        assert_eq!(
            unsafe { mfsk_iq_set_time(rx, T0_NS, 0, ptr::null_mut()) },
            MfskStatus::Ok
        );
        // Odd chunk sizes, so samples split across calls.
        for chunk in bytes.chunks(65_537) {
            assert_eq!(
                unsafe { mfsk_iq_push(rx, chunk.as_ptr() as *const c_void, chunk.len()) },
                MfskStatus::Ok
            );
        }
        assert_eq!(unsafe { mfsk_iq_samples_in(rx) }, iq.len() as u64);
        let rows = drain(rx);
        assert_eq!(unsafe { mfsk_iq_pending(rx) }, 0);
        let hit = rows
            .iter()
            .find(|d| text(d) == TEXT)
            .unwrap_or_else(|| panic!("format {format}: {TEXT} not decoded: {}", rows.len()));
        assert_eq!(hit.channel, ch);
        assert_eq!(hit.mode, MfskMode::Ft8);
        assert!((hit.freq_hz - 1_500.0).abs() < 2.0, "{}", hit.freq_hz);
        assert!((hit.abs_freq_hz - (DIAL + hit.freq_hz as f64)).abs() < 1e-6);
        assert_eq!(hit.has_utc, 1);
        assert_eq!(hit.slot_start_utc_ns, T0_NS);
        assert_eq!(hit.slot_start_sample, 0);
        // The row's detail travels with it, as on `MfskDecode`: the key is
        // the packed 77-bit message, and FT8's numbers are flagged real.
        let msg = mfsk_core::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").unwrap();
        let mut key = [0u8; MFSK_DECODE_KEY_LEN];
        for (i, &b) in msg.iter().enumerate() {
            key[i / 8] |= (b & 1) << (7 - i % 8);
        }
        assert_eq!((hit.key_bits, hit.key), (77, key), "format {format}");
        let real = MFSK_DECODE_FLAG_HAS_SYNC_SCORE
            | MFSK_DECODE_FLAG_HAS_SYNC_CV
            | MFSK_DECODE_FLAG_HAS_HARD_ERRORS;
        assert_eq!(hit.flags & real, real, "format {format}");
        assert!(hit.sync_score > 0.0);
        assert_eq!(
            hit.delivery, -1,
            "no callback was set on the channel decoder"
        );
        unsafe { mfsk_iq_close(rx) };
    }
}

#[test]
fn free_running_grid_reports_no_utc() {
    let iq = scene();
    let bytes = encode(&iq, MFSK_IQ_FORMAT_CF32);
    let rx = open(MFSK_IQ_FORMAT_CF32);
    unsafe {
        mfsk_iq_add_channel(
            rx,
            DIAL,
            MfskMode::Ft8 as u32,
            ptr::null(),
            ptr::null(),
            ptr::null_mut(),
        )
    };
    unsafe { mfsk_iq_push(rx, bytes.as_ptr() as *const c_void, bytes.len()) };
    let rows = drain(rx);
    let hit = rows.iter().find(|d| text(d) == TEXT).expect("decoded");
    assert_eq!(hit.has_utc, 0);
    unsafe { mfsk_iq_close(rx) };
}

#[test]
fn retune_and_gap_drop_the_open_slot() {
    let iq = scene();
    let bytes = encode(&iq, MFSK_IQ_FORMAT_CF32);
    let w = 8; // bytes per CF32 sample
    let cut = 7 * FS as usize * w;

    for act in ["retune", "gap"] {
        let rx = open(MFSK_IQ_FORMAT_CF32);
        unsafe {
            mfsk_iq_add_channel(
                rx,
                DIAL,
                MfskMode::Ft8 as u32,
                ptr::null(),
                ptr::null(),
                ptr::null_mut(),
            )
        };
        unsafe { mfsk_iq_set_time(rx, T0_NS, 0, ptr::null_mut()) };
        unsafe { mfsk_iq_push(rx, bytes.as_ptr() as *const c_void, cut) };
        match act {
            "retune" => assert_eq!(
                unsafe { mfsk_iq_retune(rx, CENTER, ptr::null_mut(), ptr::null_mut()) },
                MfskStatus::Ok
            ),
            _ => assert_eq!(unsafe { mfsk_iq_gap(rx, 100) }, MfskStatus::Ok),
        }
        let rest = &bytes[cut + if act == "gap" { 100 * w } else { 0 }..];
        unsafe { mfsk_iq_push(rx, rest.as_ptr() as *const c_void, rest.len()) };
        assert!(
            drain(rx).iter().all(|d| text(d) != TEXT),
            "{act}: the slot straddling it must not decode"
        );
        unsafe { mfsk_iq_close(rx) };
    }
}

#[test]
fn errors_are_statuses_not_crashes() {
    let mut st = MfskStatus::Ok;
    // Unknown format, rate below 12 kHz, non-finite centre.
    assert!(unsafe { mfsk_iq_open(FS, CENTER, 99, 0, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);
    assert!(unsafe { mfsk_iq_open(8_000, CENTER, MFSK_IQ_FORMAT_CF32, 0, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);
    assert!(unsafe { mfsk_iq_open(FS, f64::NAN, MFSK_IQ_FORMAT_CF32, 0, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);
    // An unknown channelizer, and the bank at a rate none fits.
    assert!(unsafe { mfsk_iq_open_with(FS, CENTER, MFSK_IQ_FORMAT_CF32, 0, 7, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);
    let pfb = MFSK_IQ_CHANNELIZER_PFB;
    assert!(
        unsafe { mfsk_iq_open_with(30_000, CENTER, MFSK_IQ_FORMAT_CF32, 0, pfb, &mut st) }
            .is_null()
    );
    assert_eq!(st, MfskStatus::InvalidArg);

    let rx = open(MFSK_IQ_FORMAT_CF32);
    let add = |dial: f64, mode: u32| unsafe {
        mfsk_iq_add_channel(rx, dial, mode, ptr::null(), ptr::null(), ptr::null_mut())
    };
    // DC inside the band, past the band edge, a mode the receiver does not
    // carry, a value that is not a mode at all, a non-finite dial.
    assert_eq!(
        add(CENTER - 1_000.0, MfskMode::Ft8 as u32),
        MfskStatus::InvalidArg
    );
    assert_eq!(
        add(CENTER + 40_000.0, MfskMode::Ft8 as u32),
        MfskStatus::InvalidArg
    );
    assert_eq!(add(DIAL, MfskMode::Msk144 as u32), MfskStatus::InvalidArg);
    assert_eq!(add(DIAL, 9_999), MfskStatus::InvalidArg);
    assert_eq!(add(f64::NAN, MfskMode::Ft8 as u32), MfskStatus::InvalidArg);

    let mut ch = 0u32;
    assert_eq!(
        unsafe {
            mfsk_iq_add_channel(
                rx,
                DIAL,
                MfskMode::Wspr as u32,
                ptr::null(),
                ptr::null(),
                &mut ch,
            )
        },
        MfskStatus::Ok
    );
    // A retune that puts the channel outside pauses it; it is not an error.
    let (mut paused, mut resumed) = (0u32, 0u32);
    assert_eq!(
        unsafe { mfsk_iq_retune(rx, CENTER + 200_000.0, &mut paused, &mut resumed) },
        MfskStatus::Ok
    );
    assert_eq!((paused, resumed), (1, 0));
    assert_eq!(
        unsafe { mfsk_iq_channel_state(rx, ch) },
        MFSK_IQ_CHANNEL_PAUSED
    );
    let (mut paused, mut resumed) = (0u32, 0u32);
    assert_eq!(
        unsafe { mfsk_iq_retune(rx, CENTER, &mut paused, &mut resumed) },
        MfskStatus::Ok
    );
    assert_eq!((paused, resumed), (0, 1));
    assert_eq!(
        unsafe { mfsk_iq_channel_state(rx, ch) },
        MFSK_IQ_CHANNEL_ACTIVE
    );
    assert_eq!(unsafe { mfsk_iq_remove_channel(rx, ch) }, MfskStatus::Ok);
    assert_eq!(
        unsafe { mfsk_iq_remove_channel(rx, ch) },
        MfskStatus::InvalidArg
    );

    // Null handles and pointers.
    let mut d: MfskIqDecode = unsafe { std::mem::zeroed() };
    assert!(unsafe { mfsk_iq_poll(ptr::null_mut(), &mut d) } < 0);
    assert!(unsafe { mfsk_iq_poll(rx, ptr::null_mut()) } < 0);
    assert_eq!(unsafe { mfsk_iq_poll(rx, &mut d) }, 0);
    assert_eq!(
        unsafe { mfsk_iq_push(ptr::null_mut(), ptr::null(), 0) },
        MfskStatus::NullPointer
    );
    assert_eq!(
        unsafe { mfsk_iq_push(rx, ptr::null(), 4) },
        MfskStatus::NullPointer
    );
    assert_eq!(unsafe { mfsk_iq_push(rx, ptr::null(), 0) }, MfskStatus::Ok);
    assert_eq!(unsafe { mfsk_iq_samples_in(ptr::null_mut()) }, 0);
    unsafe { mfsk_iq_close(ptr::null_mut()) };
    unsafe { mfsk_iq_close(rx) };
}
