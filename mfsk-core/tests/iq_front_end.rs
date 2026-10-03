//! Phase 1 of #534: the IQ front end must be transparent to the decoder.
//!
//! A real recording is placed as USB inside wideband IQ (several rates,
//! sample formats, an I/Q-swapped stream, a channel off DC), pulled back out
//! through [`IqToAudio`] and decoded with the request the WAV path uses. The
//! contract is the WAV path's own: the same set of messages, no extra ones,
//! and each at the same audio frequency and time to within the front end's
//! resolution.
//!
//! The synthetic IQ is double-sideband (`a(t)·e^{jΩt}`, i.e. the real audio
//! mixed up): its lower sideband lands below the dial, where the front end
//! must reject it, so a leak would show as phantom decodes or changed SNR.
#![cfg(all(feature = "ft8", feature = "fft-rustfft"))]

use std::collections::BTreeMap;

use mfsk_core::ft8::Ft8;
use mfsk_core::iq::{IqSampleFormat, IqStream, IqToAudio};
use mfsk_core::msg::decode_request::DecodeRequest;

#[allow(dead_code)]
mod common;
use common::iq::synth_iq;

const QSO3: &str = asset_path!("qso3_busy.wav");

/// `message77 -> (freq, dt)` for one decode of `audio`.
fn decode(audio: &[i16]) -> BTreeMap<Vec<u8>, (f32, f32)> {
    DecodeRequest::<Ft8>::new(audio, 200.0, 3000.0, 1.0, 200)
        .decode()
        .results
        .iter()
        .map(|r| (r.message77().to_vec(), (r.freq_hz, r.dt_sec)))
        .collect()
}

fn through_front_end(
    iq: &[(f32, f32)],
    stream: IqStream,
    dial_hz: f64,
    swap_input: bool,
) -> Vec<i16> {
    let mut fe = IqToAudio::new(stream, dial_hz).expect("front end");
    let mut out = Vec::new();
    // Mixed block sizes, including ones that split every internal stage's
    // decimation phase differently.
    let sizes = [4_099usize, 1, 65_536, 777, 30_011];
    let (mut at, mut k) = (0usize, 0usize);
    while at < iq.len() {
        let n = sizes[k % sizes.len()].min(iq.len() - at);
        k += 1;
        let block = &iq[at..at + n];
        at += n;
        match stream.format {
            IqSampleFormat::Cf32 => {
                let v: Vec<f32> = block
                    .iter()
                    .flat_map(|&(i, q)| if swap_input { [q, i] } else { [i, q] })
                    .collect();
                fe.push_cf32(&v, &mut out);
            }
            IqSampleFormat::Cs16 => {
                // Full scale is 1.0 here; the audio peaks well under it.
                let v: Vec<i16> = block
                    .iter()
                    .flat_map(|&(i, q)| {
                        let (i, q) = if swap_input { (q, i) } else { (i, q) };
                        [(i * 32_768.0) as i16, (q * 32_768.0) as i16]
                    })
                    .collect();
                fe.push_cs16(&v, &mut out);
            }
            // Byte-only formats have their own tests (`iq_receiver.rs`).
            other => unreachable!("{other:?} is not driven by this test"),
        }
    }
    assert_eq!(fe.samples_in(), iq.len() as u64);
    // Double-sideband IQ puts half the amplitude in the wanted sideband.
    out.iter()
        .map(|&v| (v * 2.0 * 32_768.0).round().clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

fn check(fs: u32, format: IqSampleFormat, iq_swap: bool, dial_off_hz: f64) {
    let Some(wav) = common::load_wav_i16_opt(QSO3) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let reference = decode(&wav);
    assert!(
        reference.len() >= 10,
        "WAV path decoded {}",
        reference.len()
    );

    let center = 14_200_000.0;
    let dial = center + dial_off_hz;
    let iq = synth_iq(&wav, fs, center, dial);
    let stream = IqStream::new(fs, center, format).iq_swap(iq_swap);
    // A swapped stream is the conjugate one: feed swap(I,Q) and it undoes.
    let audio = through_front_end(&iq, stream, dial, iq_swap);
    let via_iq = decode(&audio);

    let missing: Vec<_> = reference
        .keys()
        .filter(|k| !via_iq.contains_key(*k))
        .collect();
    let extra: Vec<_> = via_iq
        .keys()
        .filter(|k| !reference.contains_key(*k))
        .collect();
    println!(
        "  fs {fs:>8} {format:?} swap={iq_swap} off={dial_off_hz:>8}: wav {} iq {} missing {} extra {}",
        reference.len(),
        via_iq.len(),
        missing.len(),
        extra.len()
    );
    // Recall no worse than one weak-edge decode, precision exact.
    assert!(
        extra.is_empty(),
        "{} phantom decodes through IQ",
        extra.len()
    );
    assert!(
        missing.len() <= 1,
        "{} of {} decodes lost through IQ",
        missing.len(),
        reference.len()
    );
    for (k, &(f, dt)) in &reference {
        if let Some(&(f2, dt2)) = via_iq.get(k) {
            assert!((f - f2).abs() < 1.0, "freq {f} vs {f2}");
            assert!((dt - dt2).abs() < 0.05, "dt {dt} vs {dt2}");
        }
    }
}

#[test]
fn cf32_48k() {
    check(48_000, IqSampleFormat::Cf32, false, 6_000.0);
}

#[test]
fn cf32_192k() {
    check(192_000, IqSampleFormat::Cf32, false, 40_000.0);
}

#[test]
fn cf32_768k_off_dc() {
    check(768_000, IqSampleFormat::Cf32, false, -150_000.0);
}

#[test]
fn cs16_250k() {
    check(250_000, IqSampleFormat::Cs16, false, 20_000.0);
}

#[test]
fn cf32_2_4m_iq_swapped() {
    check(2_400_000, IqSampleFormat::Cf32, true, 300_000.0);
}

/// Selectivity (#534): an interferer anywhere outside the channel's audio
/// −200…6200 Hz comes out at least `REJECT_DB` below a tone inside it. Swept
/// finely just past both window edges, where only the last filter acts, and
/// at points off any grid across the band, where the decimators' aliases
/// fall. Before the Kaiser designs this path gave −73.5 dB at the window's
/// edge (Blackman windows); measured after, −120.6 dB worst over ~550 points
/// at 192 k, 768 k and 2.4 MS/s. One rate here, to keep the gate quick.
#[test]
fn rejects_every_interferer_outside_the_window() {
    use mfsk_core::iq::REJECT_DB;
    const FS: u32 = 768_000;
    let center = 14_200_000.0;
    let dial = center + 96_000.0;
    let rms = |rf: f64| {
        let n = FS as usize / 5;
        let w = std::f64::consts::TAU * (rf - center) / FS as f64;
        let v: Vec<f32> = (0..n)
            .flat_map(|k| {
                let p = w * k as f64;
                [p.cos() as f32, p.sin() as f32]
            })
            .collect();
        let s = IqStream::new(FS, center, IqSampleFormat::Cf32);
        let mut fe = IqToAudio::new(s, dial).unwrap();
        let mut o = Vec::new();
        fe.push_cf32(&v, &mut o);
        let t = &o[o.len() / 2..];
        (t.iter().map(|x| (*x as f64).powi(2)).sum::<f64>() / t.len() as f64).sqrt()
    };
    let wanted = rms(dial + 1_500.0);
    let mut offs = Vec::new();
    let mut x = 0.0;
    while x < 1_500.0 {
        offs.extend([-200.0 - x, 6_200.0 + x]);
        x += 31.3;
    }
    let mut s: u64 = 534;
    while offs.len() < 160 {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let o = ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 0.95 * FS as f64 - (dial - center);
        if !(-200.0..=6_200.0).contains(&o) {
            offs.push(o);
        }
    }
    let (worst, at) = offs
        .iter()
        .map(|&o| (20.0 * (rms(dial + o) / wanted).log10(), o))
        .fold((f64::MIN, 0.0), |a, b| if b.0 > a.0 { b } else { a });
    println!(
        "  worst {worst:.1} dB at {at:.1} Hz from the dial ({} points)",
        offs.len()
    );
    assert!(worst <= -(REJECT_DB - 1.0), "{worst:.1} dB at {at:.1} Hz");
}
