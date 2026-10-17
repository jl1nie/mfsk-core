//! #686: a source whose real sample rate is not the nominal one the stream was built for.
//!
//! The IQ is made at a rate `R` (a real recording placed as double-sideband IQ, as `iq_front_end` does) and
//! the stream is told a nominal rate `N` that is off by up to a few thousand ppm; `set_rate_offset_ppm` says
//! by how much. The contract is the WAV path's, as in `iq_front_end` and `iq_receiver`: the same messages, at
//! the same audio frequency and DT. Without the offset the same stream must *not* meet it (the channel sits
//! 80 kHz from the centre: 3000 ppm of 83 kHz is 250 Hz of mixing error), or these tests would pass for
//! nothing.
#![cfg(all(feature = "ft8", feature = "fft-rustfft"))]

use std::collections::BTreeMap;

use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::iq::{
    Channelizer, CompletedSlot, IqReceiver, IqSampleFormat, IqStream, IqToAudio, plannable_rate,
};

#[allow(dead_code)]
mod common;
use common::iq::{interleave, synth_iq};

const FT8_WAV: &str = asset_path!("qso3_busy.wav");
/// The source's real rate.
const REAL: u32 = 192_000;
const CENTER: f64 = 14_077_000.0;
/// 80 kHz above the centre: the offset's mixing error is large.
const DIAL: f64 = CENTER + 80_000.0;
/// A multiple of 15 s.
const T0_NS: i64 = 1_700_000_010 * 1_000_000_000;

type Set = BTreeMap<String, (f32, f32)>;

/// The nominal rate a client that is `eps` off would give, and the offset that tells the truth.
fn nominal_for(eps: f64) -> (u32, f64) {
    let (n, _) = plannable_rate(REAL as f64 / (1.0 + eps)).expect("a rate near it plans");
    (n, (REAL as f64 / n as f64 - 1.0) * 1e6)
}

fn reference(audio: &[i16]) -> Set {
    AnyDecoder::with_defaults(Mode::Ft8)
        .decode_i16(audio, None)
        .rows
        .into_iter()
        .map(|r| (r.decoded.text, (r.decoded.freq_hz, r.decoded.dt_sec)))
        .collect()
}

fn iq_of(audio: &[i16]) -> Vec<(f32, f32)> {
    let mut iq = synth_iq(audio, REAL, CENTER, DIAL);
    // The slot's last audio comes out a few filter lengths after the last IQ sample.
    iq.resize(iq.len() + REAL as usize / 2, (0.0, 0.0));
    iq
}

fn decode_slots(slots: &[CompletedSlot]) -> Set {
    let mut d = AnyDecoder::with_defaults(Mode::Ft8);
    slots
        .iter()
        .flat_map(|s| d.decode(&s.input()).rows)
        .map(|r| (r.decoded.text, (r.decoded.freq_hz, r.decoded.dt_sec)))
        .collect()
}

/// `got` against `want` as `iq_receiver::same`: nothing extra, at most a tenth lost, frequency and DT close.
fn same(got: &Set, want: &Set, what: &str) {
    let missing: Vec<_> = want.keys().filter(|k| !got.contains_key(*k)).collect();
    let extra: Vec<_> = got.keys().filter(|k| !want.contains_key(*k)).collect();
    println!(
        "  {what}: ref {} got {} missing {} extra {}",
        want.len(),
        got.len(),
        missing.len(),
        extra.len()
    );
    assert!(want.len() >= 10, "{what}: reference of {}", want.len());
    assert!(extra.len() <= 1, "{what}: phantom {extra:?}");
    assert!(missing.len() <= want.len() / 10, "{what}: lost {missing:?}");
    for (k, &(f, dt)) in want {
        if let Some(&(f2, dt2)) = got.get(k) {
            assert!((f - f2).abs() < 2.0, "{what}: freq {f} vs {f2}");
            assert!((dt - dt2).abs() < 0.05, "{what}: dt {dt} vs {dt2}");
        }
    }
}

/// The largest frequency error among the messages both sets have.
fn worst_freq_error(got: &Set, want: &Set) -> f32 {
    want.iter()
        .filter_map(|(k, &(f, _))| got.get(k).map(|&(f2, _)| (f - f2).abs()))
        .fold(0.0, f32::max)
}

fn receive(eps: f64, offset: bool, kind: Channelizer, iq: &[(f32, f32)]) -> (Set, IqReceiver) {
    let (n, ppm) = nominal_for(eps);
    let stream = IqStream::new(n, CENTER, IqSampleFormat::Cf32);
    let mut rx = IqReceiver::with_channelizer(stream, kind).unwrap();
    if offset {
        rx.set_rate_offset_ppm(ppm);
    }
    rx.add_channel(DIAL, Mode::Ft8).unwrap();
    rx.set_time(T0_NS, 0);
    let mut slots = Vec::new();
    // Odd block sizes, so slot ends fall inside a push.
    for chunk in interleave(iq).chunks(2 * 77_777) {
        rx.push_cf32(chunk, &mut slots);
    }
    (decode_slots(&slots), rx)
}

fn receiver_follows_a_source_off_its_nominal_rate(kind: Channelizer) {
    let Some(wav) = common::load_wav_i16_opt(FT8_WAV) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let want = reference(&wav);
    let iq = iq_of(&wav);
    for eps in [3.0e-3, -3.0e-3, 4.0e-4] {
        let (_, ppm) = nominal_for(eps);
        let (got, rx) = receive(eps, true, kind, &iq);
        same(&got, &want, &format!("{kind:?} {ppm:+.0} ppm, corrected"));
        // The clock is on the true time axis: the last sample is at T0 + len / REAL, not len / nominal.
        let t = rx.utc_of(rx.samples_in()).expect("a clock");
        let true_ns = T0_NS + (iq.len() as f64 / REAL as f64 * 1e9) as i64;
        assert!(
            (t - true_ns).abs() < 2_000_000,
            "{ppm} ppm: clock {t} vs {true_ns}"
        );
        if eps.abs() > 1e-3 {
            // The same stream, not told: the mixing is off by tens to hundreds of Hz.
            let (blind, _) = receive(eps, false, kind, &iq);
            let err = worst_freq_error(&blind, &want);
            println!("  {kind:?} {ppm:+.0} ppm, uncorrected: worst frequency error {err:.0} Hz");
            assert!(
                err > 30.0 || blind.len() + want.len() / 4 < want.len(),
                "an uncorrected stream should not meet the contract ({err} Hz)"
            );
        }
    }
}

#[test]
fn direct_receiver_follows_a_source_off_its_nominal_rate() {
    receiver_follows_a_source_off_its_nominal_rate(Channelizer::Direct);
}

#[test]
fn pfb_receiver_follows_a_source_off_its_nominal_rate() {
    receiver_follows_a_source_off_its_nominal_rate(Channelizer::Pfb);
}

/// The front end alone: the audio of a stream 3000 ppm off decodes like the WAV once told.
#[test]
fn front_end_with_an_offset_decodes_like_the_wav() {
    let Some(wav) = common::load_wav_i16_opt(FT8_WAV) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let want = reference(&wav);
    let iq = iq_of(&wav);
    for eps in [3.0e-3, -3.0e-3] {
        let (n, ppm) = nominal_for(eps);
        let mut fe = IqToAudio::new(IqStream::new(n, CENTER, IqSampleFormat::Cf32), DIAL).unwrap();
        fe.set_rate_offset_ppm(ppm);
        assert!((fe.rate_offset_ppm() - ppm).abs() < 1e-9);
        let mut out = Vec::new();
        for chunk in interleave(&iq).chunks(2 * 4_099) {
            fe.push_cf32(chunk, &mut out);
        }
        let audio: Vec<i16> = out
            .iter()
            .map(|&v| (v * 2.0 * 32_768.0).round().clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let got: Set = AnyDecoder::with_defaults(Mode::Ft8)
            .decode_i16(&audio, None)
            .rows
            .into_iter()
            .map(|r| (r.decoded.text, (r.decoded.freq_hz, r.decoded.dt_sec)))
            .collect();
        same(&got, &want, &format!("front end {ppm:+.0} ppm"));
    }
}

/// `plannable_rate` returns a rate the chain takes, within 500 ppm of the real one, with the offset left.
#[test]
fn plannable_rate_picks_a_rate_the_chain_takes() {
    for real in [
        994_271.3,
        1_035_949.0,
        998_444.0,
        2_047_000.0,
        191_424.7,
        48_001.0,
        250_000.0,
        768_153.0,
    ] {
        let (n, ppm) = plannable_rate(real).unwrap_or_else(|| panic!("{real}"));
        assert!(
            IqToAudio::new(
                IqStream::new(n, 14_000_000.0, IqSampleFormat::Cf32),
                14_000_000.0 + 20_000.0
            )
            .is_ok()
                || real < 60_000.0,
            "{real} -> {n}"
        );
        assert!(ppm.abs() <= 500.5, "{real} -> {n}: {ppm}");
        assert!(((real / n as f64 - 1.0) * 1e6 - ppm).abs() < 1e-6);
    }
    assert!(plannable_rate(5_000.0).is_none());
}

/// A change of the offset while running moves the audio continuously: a steady tone keeps its level and
/// no sample steps, through a change in each direction.
#[test]
fn changing_the_offset_while_running_does_not_step_the_audio() {
    let n = 192_000u32;
    let center = 14_100_000.0;
    let dial = center + 20_000.0;
    let mut fe = IqToAudio::new(IqStream::new(n, center, IqSampleFormat::Cf32), dial).unwrap();
    // A tone at dial + 1500 Hz, made at the nominal rate; the offset moves the mixing and resampling a
    // little, the tone's frequency in the audio with it (by 1500 ppm of 3000 Hz: a few Hz).
    let w = std::f64::consts::TAU * (dial + 1_500.0 - center) / n as f64;
    let mut out = Vec::new();
    let total = n as usize * 6;
    for start in (0..total).step_by(4_800) {
        if start == 2 * n as usize {
            fe.set_rate_offset_ppm(1_500.0);
        }
        if start == 4 * n as usize {
            fe.set_rate_offset_ppm(-1_500.0);
        }
        let iq: Vec<f32> = (start..(start + 4_800).min(total))
            .flat_map(|k| {
                let p = w * k as f64;
                [0.5 * p.cos() as f32, 0.5 * p.sin() as f32]
            })
            .collect();
        fe.push_cf32(&iq, &mut out);
    }
    // After the filters have settled, a 1.5 kHz tone steps at most 2 * sin(pi * 1500 / 12000) of its
    // amplitude per sample... the largest step of a real sinusoid of amplitude a at 1.5 kHz is
    // a * 2 sin(pi f / fs) = 0.765 a at the zero crossing.
    let a = out[10_000..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
    let worst_step = (10_000..out.len() - 1)
        .map(|k| (out[k + 1] - out[k]).abs())
        .fold(0.0f32, f32::max);
    assert!(
        worst_step < 0.77 * a * 1.02,
        "a step of {worst_step} in a tone of amplitude {a}"
    );
    // The level is the same before and after each change (windows of 0.5 s).
    let rms = |from: usize, to: usize| {
        (out[from..to].iter().map(|v| v * v).sum::<f32>() / (to - from) as f32).sqrt()
    };
    let (r1, r2, r3) = (
        rms(18_000, 23_000),
        rms(42_000, 47_000),
        rms(66_000, 71_000),
    );
    assert!(
        (r1 / r2 - 1.0).abs() < 0.01 && (r2 / r3 - 1.0).abs() < 0.01,
        "{r1} {r2} {r3}"
    );
}

/// The offset survives a gap (the front ends are rebuilt) and a retune: the audio tone stays where it is.
#[test]
fn the_offset_survives_a_gap_and_a_retune() {
    let (n, ppm) = nominal_for(3.0e-3);
    let center = CENTER;
    let stream = IqStream::new(n, center, IqSampleFormat::Cf32);
    let mut rx = IqReceiver::new(stream);
    rx.set_rate_offset_ppm(ppm);
    let ch = rx.add_audio_channel(DIAL).unwrap();
    // A tone at dial + 1500 Hz in a stream that really runs at REAL.
    let tone = |from: usize, to: usize, centre: f64| -> Vec<f32> {
        let w = std::f64::consts::TAU * (DIAL + 1_500.0 - centre) / REAL as f64;
        (from..to)
            .flat_map(|k| {
                let p = w * k as f64;
                [0.5 * p.cos() as f32, 0.5 * p.sin() as f32]
            })
            .collect()
    };
    let freq_of = |audio: &[f32]| -> f64 {
        // Goertzel scan around 1500 Hz in 0.25 Hz steps.
        let mut best = (0.0f64, 0.0f64);
        let mut f = 1_480.0;
        while f <= 1_520.0 {
            let w = std::f64::consts::TAU * f / 12_000.0;
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (k, &v) in audio.iter().enumerate() {
                re += v as f64 * (w * k as f64).cos();
                im += v as f64 * (w * k as f64).sin();
            }
            let p = re * re + im * im;
            if p > best.1 {
                best = (f, p);
            }
            f += 0.25;
        }
        best.0
    };
    let mut slots = Vec::new();
    let take = |rx: &mut IqReceiver| {
        let mut a = Vec::new();
        while rx.take_audio_from(ch, &mut a).is_some() {}
        a
    };
    rx.push_cf32(&tone(0, 2 * REAL as usize, center), &mut slots);
    let a = take(&mut rx);
    let f1 = freq_of(&a[a.len() - 12_000..]);
    rx.gap(0);
    rx.push_cf32(
        &tone(2 * REAL as usize, 4 * REAL as usize, center),
        &mut slots,
    );
    let a = take(&mut rx);
    let f2 = freq_of(&a[a.len() - 12_000..]);
    // Retune by 10 kHz: the same dial, a different centre.
    let c2 = center + 10_000.0;
    rx.retune(c2);
    rx.push_cf32(&tone(4 * REAL as usize, 6 * REAL as usize, c2), &mut slots);
    let a = take(&mut rx);
    let f3 = freq_of(&a[a.len() - 12_000..]);
    println!("  tone at 1500 Hz read as {f1} / {f2} / {f3} Hz ({ppm:+.0} ppm)");
    for f in [f1, f2, f3] {
        assert!((f - 1_500.0).abs() < 0.6, "{f1} {f2} {f3}");
    }
}

/// Told only the clock, the receiver learns the offset (#686): a source 1500 ppm fast, readings of the
/// stream's UTC every 0.1 s of real time with a few ms of jitter, and nothing else. In 6 minutes the offset
/// is within 30 ppm and the clock was never stepped.
#[test]
fn the_receiver_learns_the_offset_from_the_clock() {
    use mfsk_core::iq::RateTrackConfig;
    use mfsk_core::slotgrid::ClockChange;
    const N: u32 = 192_000;
    let real = N as f64 * 1.0015;
    let mut rx = IqReceiver::new(IqStream::new(N, CENTER, IqSampleFormat::Cf32));
    rx.track_rate(Some(RateTrackConfig::default()));
    let (mut slots, mut steps, mut rng, mut moves) = (Vec::new(), 0, 99u32, 0);
    let chunk = vec![0.0f32; 2 * 19_200];
    let mut s = 0u64;
    for k in 1..=3_600u64 {
        rx.push_cf32(&chunk, &mut slots);
        s += 19_200;
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let jitter = ((rng >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 0.006;
        // The true UTC of sample s: the source's own clock is exact, the readings are a little late or early.
        let utc = T0_NS + ((s as f64 / real + jitter) * 1e9) as i64;
        let before = rx.rate_offset_ppm();
        if let ClockChange::Stepped { .. } = rx.set_time(utc, s) {
            steps += 1;
        }
        // The offset moves only on the 15 s grid: the reading that moved it is the first of its cell.
        if rx.rate_offset_ppm() != before {
            moves += 1;
            let into = utc.rem_euclid(15_000_000_000);
            assert!(
                into < 100_000_000,
                "offset moved {into} ns into a slot (reading {k})"
            );
        }
    }
    let ppm = rx.rate_offset_ppm();
    println!("  learned {ppm:.1} ppm of 1500, {steps} clock steps");
    assert!((ppm - 1_500.0).abs() < 30.0, "{ppm}");
    assert_eq!(steps, 0);
    assert!(moves >= 1, "the offset was never moved");
}

/// What the last stage costs (#686's last line): one channel at 768 kS/s, 30 s of stream, with and
/// without an offset, on the same machine in the same run. Not an assertion; `--ignored --nocapture`.
#[test]
#[ignore = "a measurement"]
fn cost_of_the_offset() {
    let n = 768_000u32;
    let secs = 30usize;
    let iq: Vec<f32> = (0..n as usize * secs * 2)
        .map(|k| ((k * 2654435761usize) >> 8 & 0xffff) as f32 / 65536.0 - 0.5)
        .collect();
    let time = |ppm: f64| {
        let mut fe = IqToAudio::new(
            IqStream::new(n, CENTER, IqSampleFormat::Cf32),
            CENTER + 100_000.0,
        )
        .unwrap();
        if ppm != 0.0 {
            fe.set_rate_offset_ppm(ppm);
        }
        let mut out = Vec::new();
        let t = std::time::Instant::now();
        for chunk in iq.chunks(2 * 16_384) {
            fe.push_cf32(chunk, &mut out);
            out.clear();
        }
        t.elapsed().as_secs_f64()
    };
    let (a, b, c) = (time(0.0), time(300.0), time(0.0));
    let base = (a + c) / 2.0;
    println!(
        "  768 kS/s, 30 s: no offset {base:.3} s ({:.2} % of a core), offset {b:.3} s ({:.2} %), +{:.3} s = {:.3} % of a core",
        base / secs as f64 * 100.0,
        b / secs as f64 * 100.0,
        b - base,
        (b - base) / secs as f64 * 100.0
    );
}

/// A restarted device can come back a few per cent faster (an RSP behind `rsp_tcp`, #686): the receiver
/// takes the offset as far as 5 %. The chain's filters stay those of the nominal rate, so the edges of the
/// channel window move 160 Hz; FT8 sits well inside it. At +-4.5 % the messages still come out.
#[test]
fn the_receiver_follows_a_source_a_few_per_cent_off() {
    let Some(wav) = common::load_wav_i16_opt(FT8_WAV) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let want = reference(&wav);
    let iq = iq_of(&wav);
    for eps in [2.5e-2, -2.5e-2, 4.5e-2, -4.5e-2] {
        // The nominal rate a client would still have: a few per cent off the real one, so a plan it takes.
        let (n, _) = plannable_rate(REAL as f64 / (1.0 + eps)).unwrap();
        let ppm = (REAL as f64 / n as f64 - 1.0) * 1e6;
        assert!(ppm.abs() > 20_000.0 && ppm.abs() < 50_000.0, "{ppm}");
        let stream = IqStream::new(n, CENTER, IqSampleFormat::Cf32);
        let mut rx = IqReceiver::with_channelizer(stream, Channelizer::Direct).unwrap();
        rx.set_rate_offset_ppm(ppm);
        assert!((rx.rate_offset_ppm() - ppm).abs() < 1e-9);
        rx.add_channel(DIAL, Mode::Ft8).unwrap();
        rx.set_time(T0_NS, 0);
        let mut slots = Vec::new();
        for chunk in interleave(&iq).chunks(2 * 77_777) {
            rx.push_cf32(chunk, &mut slots);
        }
        let got = decode_slots(&slots);
        same(&got, &want, &format!("{ppm:+.0} ppm"));
    }
}
