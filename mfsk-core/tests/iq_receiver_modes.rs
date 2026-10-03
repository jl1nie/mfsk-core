//! #534: `IqReceiver` on WSPR, JT9, JT65 and Q65, each a real recording placed as IQ, on a UTC
//! grid anchored so the recording starts on its slot boundary, and pulled
//! back out through the receiver. The contract is the WAV path's own: the
//! same messages at the same audio frequency and DT, plus the absolute
//! frequency the dial makes of it.
#![cfg(all(
    feature = "wspr",
    feature = "jt9",
    feature = "jt65",
    feature = "q65",
    feature = "fft-rustfft"
))]

use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, SlotInput};
use mfsk_core::iq::{Channelizer, CompletedSlot, IqReceiver, IqSampleFormat, IqStream};
use mfsk_core::msg::decoded::Decoded;
use std::collections::BTreeMap;

#[allow(dead_code)]
mod common;
use common::iq::{interleave, synth_iq};

const FS: u32 = 48_000;
const CENTER: f64 = 7_000_000.0;
const DIAL: f64 = CENTER + 6_000.0;
/// A multiple of every period here (7.5 s ... 300 s): of 600 s.
const T0_NS: i64 = 1_700_000_400 * 1_000_000_000;

type Set = BTreeMap<String, (f32, f32)>;

fn set(rows: impl IntoIterator<Item = Decoded>) -> Set {
    rows.into_iter()
        .map(|d| (d.text, (d.freq_hz, d.dt_sec)))
        .collect()
}

/// `wav` (12 kHz f32, zero-padded to `slot_s`) as IQ through a receiver with
/// one `mode` channel; the rows it delivers.
fn through_iq(kind: Channelizer, mode: Mode, wav: &[f32], slot_s: usize) -> Vec<CompletedSlot> {
    let mut pcm: Vec<i16> = wav
        .iter()
        .map(|&v| (v * 32_768.0).round().clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    pcm.resize(slot_s * 12_000, 0);
    let mut iq = synth_iq(&pcm, FS, CENTER, DIAL);
    // The front end drops its group delay, so the last audio sample of the
    // slot comes out a few filter lengths after the last IQ sample.
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));

    let mut rx =
        IqReceiver::with_channelizer(IqStream::new(FS, CENTER, IqSampleFormat::Cf32), kind)
            .unwrap();
    let mut slots = Vec::new();
    rx.add_channel(DIAL, mode).unwrap();
    rx.set_time(T0_NS, 0);
    for chunk in interleave(&iq).chunks(2 * 65_536) {
        rx.push_cf32(chunk, &mut slots);
    }
    slots
}

fn check(kind: Channelizer, mode: Mode, path: &str, slot_s: usize) {
    let full = format!(
        "{}/../embedded-poc/assets/{path}",
        env!("CARGO_MANIFEST_DIR")
    );
    let Some(wav) = common::load_wav_i16_opt(&full) else {
        common::skip_or_fail(path);
        return;
    };
    let wav: Vec<f32> = wav.iter().map(|&s| s as f32 / 32_768.0).collect();
    let want = set(AnyDecoder::with_defaults(mode)
        .decode(&SlotInput::f32(&wav))
        .rows);
    assert!(!want.is_empty(), "{mode:?}: the WAV path decoded nothing");

    let slots = through_iq(kind, mode, &wav, slot_s);
    let mut decoder = AnyDecoder::with_defaults(mode);
    let mut rows = Vec::new();
    for slot in &slots {
        for d in decoder.decode(&slot.input()).rows {
            rows.push((slot.clone(), d));
        }
    }
    let got = set(rows.iter().map(|(_, d)| d.clone()));
    let missing: Vec<_> = want.keys().filter(|k| !got.contains_key(*k)).collect();
    let extra: Vec<_> = got.keys().filter(|k| !want.contains_key(*k)).collect();
    println!(
        "  {mode:?}: wav {} iq {} missing {} extra {}",
        want.len(),
        got.len(),
        missing.len(),
        extra.len()
    );
    assert!(extra.is_empty(), "{mode:?}: phantom {extra:?}");
    // One weak-edge decode may tip either way in a set of ten or more; in a
    // smaller set a loss is a loss.
    assert!(
        missing.len() <= want.len() / 10,
        "{mode:?}: lost {missing:?}"
    );
    for (k, &(f, dt)) in &want {
        if let Some(&(f2, dt2)) = got.get(k) {
            assert!((f - f2).abs() < 2.0, "{mode:?}: freq {f} vs {f2}");
            assert!((dt - dt2).abs() < 0.1, "{mode:?}: dt {dt} vs {dt2}");
        }
    }
    for (slot, d) in &rows {
        assert_eq!(slot.mode, mode);
        assert!((slot.abs_freq_hz(d.freq_hz) - (DIAL + d.freq_hz as f64)).abs() < 1e-6);
        assert_eq!(slot.utc_ns, Some(T0_NS));
    }
}

fn wspr(kind: Channelizer) {
    check(kind, Mode::Wspr, "golden/wspr/150426_0918.wav", 120);
}

fn jt9(kind: Channelizer) {
    check(kind, Mode::Jt9, "130418_1742.wav", 60);
}

fn jt65(kind: Channelizer) {
    check(kind, Mode::Jt65, "golden/jt65/jt65a_5sig_m18.wav", 60);
}

fn q65_120d(kind: Channelizer) {
    check(
        kind,
        Mode::Q65D120,
        "golden/q65/120D_Rainscatter_10_GHz/210117_0920.wav",
        120,
    );
}

fn q65_300a(kind: Channelizer) {
    check(
        kind,
        Mode::Q65A300,
        "golden/q65/300A_Optical_Scatter/201210_0505.wav",
        300,
    );
}

// Every scene above through both paths: same recordings, same expectations.
#[test]
fn wspr_direct() {
    wspr(Channelizer::Direct);
}
#[test]
fn wspr_pfb() {
    wspr(Channelizer::Pfb);
}
#[test]
fn jt9_direct() {
    jt9(Channelizer::Direct);
}
#[test]
fn jt9_pfb() {
    jt9(Channelizer::Pfb);
}
#[test]
fn jt65_direct() {
    jt65(Channelizer::Direct);
}
#[test]
fn jt65_pfb() {
    jt65(Channelizer::Pfb);
}
#[test]
fn q65_120d_direct() {
    q65_120d(Channelizer::Direct);
}
#[test]
fn q65_120d_pfb() {
    q65_120d(Channelizer::Pfb);
}
#[test]
fn q65_300a_direct() {
    q65_300a(Channelizer::Direct);
}
#[test]
fn q65_300a_pfb() {
    q65_300a(Channelizer::Pfb);
}
