//! #534: `IqReceiver` on the modes that are not `DecodeRequest`-generic —
//! WSPR, JT9, JT65 and Q65 — each a real recording placed as IQ, on a UTC
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

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use mfsk_core::iq::{IqDecode, IqMode, IqReceiver, IqSampleFormat, IqStream};
use mfsk_core::msg::decoded::Decoded;

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
fn through_iq(mode: IqMode, wav: &[f32], slot_s: usize) -> Vec<IqDecode> {
    let mut pcm: Vec<i16> = wav
        .iter()
        .map(|&v| (v * 32_768.0).round().clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    pcm.resize(slot_s * 12_000, 0);
    let mut iq = synth_iq(&pcm, FS, CENTER, DIAL);
    // The front end drops its group delay, so the last audio sample of the
    // slot comes out a few filter lengths after the last IQ sample.
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));

    let mut rx = IqReceiver::new(IqStream {
        sample_rate: FS,
        center_hz: CENTER,
        format: IqSampleFormat::Cf32,
        iq_swap: false,
    });
    let rows = Arc::new(Mutex::new(Vec::new()));
    let sink = rows.clone();
    rx.on_decode(move |r| sink.lock().unwrap().push(r.clone()));
    rx.add_channel(DIAL, mode).unwrap();
    rx.set_time_anchor(T0_NS);
    for chunk in interleave(&iq).chunks(2 * 65_536) {
        rx.push_cf32(chunk);
    }
    rows.lock().unwrap().clone()
}

fn check(mode: IqMode, path: &str, slot_s: usize, reference: impl Fn(&[f32]) -> Vec<Decoded>) {
    let full = format!(
        "{}/../embedded-poc/assets/{path}",
        env!("CARGO_MANIFEST_DIR")
    );
    let Some(wav) = common::load_wav_i16_opt(&full) else {
        common::skip_or_fail(path);
        return;
    };
    let wav: Vec<f32> = wav.iter().map(|&s| s as f32 / 32_768.0).collect();
    let want = set(reference(&wav));
    assert!(!want.is_empty(), "{mode:?}: the WAV path decoded nothing");

    let rows = through_iq(mode, &wav, slot_s);
    let got = set(rows.iter().map(|r| r.decoded.clone()));
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
    for r in &rows {
        assert_eq!(r.mode, mode);
        assert!((r.abs_freq_hz - (DIAL + r.decoded.freq_hz as f64)).abs() < 1e-6);
        assert_eq!(r.slot_start_utc_ns, Some(T0_NS));
    }
}

#[test]
fn wspr() {
    check(IqMode::Wspr, "golden/wspr/150426_0918.wav", 120, |a| {
        mfsk_core::wspr::DecodeRequest::new(a, 12_000)
            .nominal_start(12_000)
            .decode()
            .iter()
            .map(|r| r.to_decoded())
            .collect()
    });
}

#[test]
fn jt9() {
    check(IqMode::Jt9, "130418_1742.wav", 60, |a| {
        mfsk_core::jt9::DecodeRequest::new(a, 12_000)
            .nominal_start(0)
            .decode()
            .iter()
            .map(|r| r.to_decoded())
            .collect()
    });
}

#[test]
fn jt65() {
    check(IqMode::Jt65, "golden/jt65/jt65a_5sig_m18.wav", 60, |a| {
        mfsk_core::jt65::DecodeRequest::new(a, 12_000)
            .nominal_start(0)
            .decode()
            .iter()
            .map(|r| r.to_decoded())
            .collect()
    });
}

fn q65<P: mfsk_core::q65::Q65SubMode>(a: &[f32]) -> Vec<Decoded> {
    mfsk_core::q65::DecodeRequest::<P>::new(
        a,
        12_000,
        12_000,
        mfsk_core::q65::search::default_search_params(),
    )
    .decode()
    .iter()
    .map(|r| r.to_decoded())
    .collect()
}

#[test]
fn q65_120d() {
    check(
        IqMode::Q65D120,
        "golden/q65/120D_Rainscatter_10_GHz/210117_0920.wav",
        120,
        q65::<mfsk_core::q65::Q65d120>,
    );
}

#[test]
fn q65_300a() {
    check(
        IqMode::Q65A300,
        "golden/q65/300A_Optical_Scatter/201210_0505.wav",
        300,
        q65::<mfsk_core::q65::Q65a300>,
    );
}
