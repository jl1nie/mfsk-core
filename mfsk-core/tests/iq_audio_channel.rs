//! #650: an `IqReceiver` channel that cuts no slots and keeps its continuous
//! audio, for a receiver with no slot (JTTY). Its audio is the slotted
//! channel's, and `take_audio_from` hands it over in runs that break where
//! the audio broke.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::Mode;
use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};

const FS: u32 = 48_000;
const CENTER: f64 = 7_070_000.0;
const DIAL: f64 = 7_078_000.0;
const T0_NS: i64 = 1_700_000_010 * 1_000_000_000;

fn noise(n: usize) -> Vec<f32> {
    let mut x: u32 = 0x2468_ace1;
    (0..2 * n)
        .map(|_| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (x >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        })
        .collect()
}

fn rx() -> IqReceiver {
    IqReceiver::new(IqStream::new(FS, CENTER, IqSampleFormat::Cf32))
}

/// Every run waiting, as (first index, length).
fn runs(rx: &mut IqReceiver, id: mfsk_core::iq::ChannelId) -> Vec<(u64, usize)> {
    let mut v = Vec::new();
    loop {
        let mut buf = Vec::new();
        match rx.take_audio_from(id, &mut buf) {
            Some(k) => v.push((k, buf.len())),
            None => return v,
        }
    }
}

#[test]
fn continuous_audio_is_one_run_from_the_channels_start() {
    let mut r = rx();
    let ch = r.add_audio_channel(DIAL).unwrap();
    let mut slots = Vec::new();
    for c in noise(3 * FS as usize).chunks(2 * 5_000) {
        r.push_cf32(c, &mut slots);
    }
    assert!(slots.is_empty(), "an audio channel cuts no slots");
    let got = runs(&mut r, ch);
    assert_eq!(got.len(), 1, "{got:?}");
    assert_eq!(got[0].0, 0);
    // 3 s at 12 kHz, less the front end's group delay.
    assert!((35_000..=36_000).contains(&got[0].1), "{got:?}");
    assert!(runs(&mut r, ch).is_empty(), "drained");
}

#[test]
fn a_gap_and_a_clock_step_each_start_a_run() {
    let mut r = rx();
    let ch = r.add_audio_channel(DIAL).unwrap();
    r.set_time(T0_NS, 0);
    let iq = noise(FS as usize);
    let mut slots = Vec::new();
    r.push_cf32(&iq, &mut slots);
    // A hole of half a second: the index jumps.
    r.gap(FS as u64 / 2);
    r.push_cf32(&iq, &mut slots);
    // The clock stepped by 5 s: the index runs on, the UTC does not.
    let at = r.samples_in();
    r.set_time(
        T0_NS + 5_000_000_000 + at as i64 * 1_000_000_000 / FS as i64,
        at,
    );
    r.push_cf32(&iq, &mut slots);
    let got = runs(&mut r, ch);
    assert_eq!(got.len(), 3, "{got:?}");
    let end = |(k, n): (u64, usize)| k + n as u64;
    assert!(
        got[1].0 > end(got[0]),
        "after the gap the index jumps: {got:?}"
    );
    assert_eq!(got[2].0, end(got[1]), "after the step it runs on: {got:?}");
}

#[test]
fn take_audio_joins_the_runs_and_the_audio_is_the_slotted_channels() {
    let mut r = rx();
    let audio = r.add_audio_channel(DIAL).unwrap();
    let slotted = r.add_channel(DIAL, Mode::Ft8).unwrap();
    r.tap_audio(slotted, true);
    let mut slots = Vec::new();
    let iq = noise(FS as usize);
    r.push_cf32(&iq, &mut slots);
    r.gap(1_000);
    r.push_cf32(&iq, &mut slots);
    let (mut a, mut b) = (Vec::new(), Vec::new());
    assert!(r.take_audio(audio, &mut a));
    assert!(r.take_audio(slotted, &mut b));
    assert!(!a.is_empty());
    assert_eq!(a, b, "one front end, one audio");
    assert!(
        runs(&mut r, audio).is_empty(),
        "take_audio drained every run"
    );
    assert!(
        !r.set_prefix_points(audio, &[1_000]),
        "an audio channel has no slots"
    );
}

#[test]
fn utc_of_audio_follows_the_clock() {
    let mut r = rx();
    assert_eq!(r.utc_of_audio(12_000), None, "no clock yet");
    r.set_time(T0_NS, 0);
    assert_eq!(r.utc_of_audio(0), Some(T0_NS));
    assert_eq!(r.utc_of_audio(12_000), Some(T0_NS + 1_000_000_000));
}

/// The path the skimmer takes: a JTTY transmission placed as IQ on a dial,
/// its audio channel's runs scaled to `i16`, into `jtty::rx::Stream`. The
/// message and its callsigns come out.
#[cfg(feature = "jtty")]
#[test]
fn jtty_through_an_audio_channel() {
    use mfsk_core::jtty::rx::{Params, Receiver, Stream};
    use mfsk_core::jtty::source::{Atom, CallAction};
    use mfsk_core::jtty::tx;
    use std::sync::Arc;

    #[allow(dead_code)]
    #[path = "common/iq.rs"]
    mod iq;

    let atoms = [
        Atom::call(CallAction::Call, "JA1ABC"),
        Atom::call(CallAction::Call, "K1ABC"),
    ];
    let tones = tx::tones(&atoms).unwrap();
    let mut audio: Vec<i16> = vec![0; 12_000];
    audio.extend(
        tx::synth_f32(&tones, 1500.0, 3000.0)
            .iter()
            .map(|&x| x as i16),
    );
    audio.extend(std::iter::repeat_n(0, 2 * 12_000));
    let mut sig = iq::synth_iq(&audio, FS, CENTER, DIAL);
    // A little noise under it, so the level is the band's, not the signal's.
    for (s, n) in sig.iter_mut().zip(noise(audio.len() * 4).chunks(2)) {
        s.0 += n[0] * 0.02;
        s.1 += n[1] * 0.02;
    }

    let mut r = rx();
    let ch = r.add_audio_channel(DIAL).unwrap();
    let mut slots = Vec::new();
    for c in iq::interleave(&sig).chunks(2 * 4_096) {
        r.push_cf32(c, &mut slots);
    }
    let mut a = Vec::new();
    let k = r.take_audio_from(ch, &mut a).expect("audio");
    assert_eq!(k, 0);
    // One gain for the recording, as a slow AGC would settle on.
    let rms = (a.iter().map(|v| v * v).sum::<f32>() / a.len() as f32).sqrt();
    let g = 2_000.0 / rms;
    let pcm: Vec<i16> = a
        .iter()
        .map(|&v| (v * g).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    let mut stream = Stream::new(Arc::new(Receiver::new()), Params::default());
    let mut ups = Vec::new();
    stream.push(&pcm, &mut |u| ups.push(u));
    stream.finish(&mut |u| ups.push(u));
    let done = ups
        .iter()
        .find(|u| u.complete)
        .unwrap_or_else(|| panic!("{ups:?}"));
    assert_eq!(done.calls, ["JA1ABC", "K1ABC"], "{}", done.text);
    assert!((done.f1_hz - 1500.0).abs() < 3.0, "{}", done.f1_hz);
}
