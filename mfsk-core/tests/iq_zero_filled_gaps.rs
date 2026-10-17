//! #686: a server that fills API losses with silence preserves the device's sample
//! time axis. Periodic 60 ms holes are erasures; deleting them accumulates timing slips.
#![cfg(all(feature = "ft8", feature = "ft4", feature = "fft-rustfft"))]

use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::engine::{MessageCodec, MessageFields};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft8::Ft8;
use mfsk_core::iq::{Channelizer, IqReceiver, IqSampleFormat, IqStream};
use mfsk_core::msg::Wsjt77Message;

#[allow(dead_code)]
#[path = "common/iq.rs"]
mod iq;
use iq::{interleave, synth_iq};

const FS: u32 = 48_000;
const CENTER: f64 = 7_000_000.0;
const DIAL: f64 = CENTER + 6_000.0;
const T0: i64 = 1_700_000_010_000_000_000; // on both slot grids
const TEXT: &str = "CQ K1ABC FN42";

fn slot(mode: Mode) -> Vec<i16> {
    let bits = Wsjt77Message
        .pack(&MessageFields {
            call1: Some("CQ".into()),
            call2: Some("K1ABC".into()),
            grid: Some("FN42".into()),
            ..MessageFields::default()
        })
        .unwrap();
    let bits: [u8; 77] = bits.try_into().unwrap();
    let frame = match mode {
        Mode::Ft8 => synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, 1500.0, 25_000),
        Mode::Ft4 => synthesize_i16::<Ft4>(&message_to_tones::<Ft4>(&bits), 12_000, 1500.0, 25_000),
        _ => unreachable!(),
    };
    let len = match mode {
        Mode::Ft8 => 180_000,
        Mode::Ft4 => 90_000,
        _ => unreachable!(),
    };
    let mut out = vec![0; len];
    out[6_000..6_000 + frame.len()].copy_from_slice(&frame);
    out
}

/// Both variants remove the same source samples; the server's repaired stream writes
/// zeros in their exact places, whereas a raw lossy server sends the next sample immediately.
fn holes(iq: &[(f32, f32)], fill: bool) -> Vec<(f32, f32)> {
    let period = (1.85 * FS as f64) as usize;
    let first = (1.05 * FS as f64) as usize;
    let length = (0.060 * FS as f64) as usize;
    iq.iter()
        .enumerate()
        .filter_map(|(k, &v)| {
            if k >= first && (k - first) % period < length {
                fill.then_some((0.0, 0.0))
            } else {
                Some(v)
            }
        })
        .collect()
}

fn receive(
    mode: Mode,
    kind: Channelizer,
    format: IqSampleFormat,
    iq: &[(f32, f32)],
) -> Option<(f32, f32)> {
    let mut rx = IqReceiver::with_channelizer(IqStream::new(FS, CENTER, format), kind).unwrap();
    rx.add_channel(DIAL, mode).unwrap();
    rx.set_time(T0, 0);
    let mut slots = Vec::new();
    for block in iq.chunks(4099) {
        match format {
            IqSampleFormat::Cs16 => {
                let samples: Vec<i16> = block
                    .iter()
                    .flat_map(|&(i, q)| [(i * 32_768.0) as i16, (q * 32_768.0) as i16])
                    .collect();
                rx.push_cs16(&samples, &mut slots);
            }
            IqSampleFormat::Cu8 => {
                let bytes: Vec<u8> = block
                    .iter()
                    .flat_map(|&(i, q)| [i, q])
                    .map(|v| (v * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8)
                    .collect();
                rx.push_bytes(&bytes, &mut slots);
            }
            _ => rx.push_cf32(&interleave(block), &mut slots),
        }
    }
    assert_eq!(rx.samples_in(), iq.len() as u64);
    assert_eq!(rx.rate_offset_ppm(), 0.0);
    assert_eq!(
        rx.utc_of(rx.samples_in()),
        Some(T0 + iq.len() as i64 * 1_000_000_000 / FS as i64)
    );
    let slot = slots
        .iter()
        .find(|s| s.utc_ns == Some(T0))
        .expect("whole slot, including filter tail");
    AnyDecoder::with_defaults(mode)
        .decode(&slot.input())
        .rows
        .into_iter()
        .find(|r| r.decoded.text == TEXT)
        .map(|r| (r.decoded.freq_hz, r.decoded.dt_sec))
}

fn check(kind: Channelizer, format: IqSampleFormat) {
    for mode in [Mode::Ft8, Mode::Ft4] {
        let mut clean = synth_iq(&slot(mode), FS, CENTER, DIAL);
        // Covers the front end's filter latency and the shorter, deleted-sample control.
        clean.resize(clean.len() + FS as usize, (0.0, 0.0));
        let filled = holes(&clean, true);
        let dropped = holes(&clean, false);
        assert_eq!(filled.len(), clean.len());
        assert!(dropped.len() < filled.len());
        let reference = receive(mode, kind, format, &clean).expect("clean stream decodes");
        let restored = receive(mode, kind, format, &filled).expect("zero-filled holes decode");
        assert!(
            (reference.0 - restored.0).abs() < 1.0,
            "{mode:?}: frequency {reference:?} vs {restored:?}"
        );
        assert!(
            (reference.1 - restored.1).abs() < 0.05,
            "{mode:?}: DT {reference:?} vs {restored:?}"
        );
        assert!(
            receive(mode, kind, format, &dropped).is_none(),
            "{mode:?}: deleted holes unexpectedly decoded"
        );
    }
}

#[test]
fn direct_s16_zero_filled_holes_preserve_ft8_and_ft4() {
    check(Channelizer::Direct, IqSampleFormat::Cs16);
}
#[test]
fn pfb_s16_zero_filled_holes_preserve_ft8_and_ft4() {
    check(Channelizer::Pfb, IqSampleFormat::Cs16);
}
#[test]
fn direct_u8_zero_filled_holes_preserve_ft8_and_ft4() {
    check(Channelizer::Direct, IqSampleFormat::Cu8);
}
#[test]
fn pfb_u8_zero_filled_holes_preserve_ft8_and_ft4() {
    check(Channelizer::Pfb, IqSampleFormat::Cu8);
}
