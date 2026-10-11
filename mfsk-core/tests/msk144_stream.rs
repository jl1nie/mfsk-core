//! `msk144::decode::Stream` (#497) against `decode_slot` on the WSJT-X
//! golden recordings: fed in chunks of any size, from one sample to the
//! whole slot, the stream reports exactly what the whole-slot scan does,
//! and the dupe check behaves as `mskrtd.f90:88` says across periods.

#![cfg(all(
    feature = "msk144",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::msk144::decode::{Depth, SlotDecode, Stream, StreamDecode, decode_slot};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const FILES: [&str; 2] = ["181211_120500.wav", "181211_120800.wav"];

fn golden(name: &str) -> Option<Vec<i16>> {
    let Some(path) = common::corpus::golden_path_or_upstream(
        &format!("msk144/{name}"),
        Some(&format!("MSK144/{name}")),
    ) else {
        common::skip_or_fail(&format!("the MSK144 golden {name}"));
        return None;
    };
    Some(load_wav_i16_opt(&path).expect("WAV must be 12 kHz mono PCM-16"))
}

fn stream_in_chunks(audio: &[i16], chunk: usize) -> Vec<StreamDecode> {
    let mut s = Stream::new(1477.0, 60.0, Depth::Deep, None);
    let mut out = Vec::new();
    for c in audio.chunks(chunk) {
        s.push(c, &mut |d| out.push(d));
    }
    out
}

fn same(a: &[SlotDecode], b: &[StreamDecode]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| {
            x.message == y.message
                && x.snr_db == y.snr_db
                && x.freq_hz == y.freq_hz
                && x.tsec == y.t_s as f32
        })
}

#[test]
fn any_chunk_size_matches_decode_slot() {
    for name in FILES {
        let Some(audio) = golden(name) else { continue };
        let whole = decode_slot(&audio, 1477.0, 60.0, Depth::Deep);
        assert!(!whole.is_empty(), "{name}: the golden decodes nothing");
        // 1 sample, primes that straddle every window boundary, the stride,
        // the window, and the whole slot.
        for chunk in [1, 7, 1000, 3583, 3584, 3585, 7168, 12_000, audio.len()] {
            let got = stream_in_chunks(&audio, chunk);
            assert!(
                same(&whole, &got),
                "{name}, chunk {chunk}: {got:?} != {whole:?}"
            );
        }
    }
}

/// A slot fed twice in one stream: with no period boundary the second copy's
/// repeats are dupes of the first copy's last message; after `new_period` the
/// slot decodes as it did the first time, and the times run on.
#[test]
fn new_period_resets_only_the_dupe_check() {
    let Some(audio) = golden("181211_120500.wav") else {
        return;
    };
    let mut s = Stream::new(1477.0, 60.0, Depth::Deep, None);
    let mut first = Vec::new();
    s.push(&audio, &mut |d| first.push(d));
    assert!(!first.is_empty());

    let mut again = Vec::new();
    s.new_period();
    s.push(&audio, &mut |d| again.push(d));
    let msgs = |v: &[StreamDecode]| v.iter().map(|d| d.message.clone()).collect::<Vec<_>>();
    assert_eq!(msgs(&again), msgs(&first), "the second period lost decodes");
    let slot_s = audio.len() as f64 / 12_000.0;
    // The stream's windows run on across the join, so they sit at the same
    // offsets only when the slot is a whole number of strides; allow one.
    for (a, b) in first.iter().zip(&again) {
        assert!(
            (b.t_s - a.t_s - slot_s).abs() <= 0.3,
            "time did not run on: {a:?} {b:?}"
        );
    }

    let mut s = Stream::new(1477.0, 60.0, Depth::Deep, None);
    s.push(&audio, &mut |_| {});
    let mut no_reset = Vec::new();
    s.push(&audio, &mut |d| no_reset.push(d));
    let last = first.last().unwrap().message.clone();
    assert!(
        no_reset
            .first()
            .is_none_or(|d| d.message != last || d.snr_db > first.last().unwrap().snr_db),
        "a repeat of the last message came through without a new period: {no_reset:?}"
    );
}

/// `reset` puts the stream back to its first sample.
#[test]
fn reset_restarts_times_and_state() {
    let Some(audio) = golden("181211_120800.wav") else {
        return;
    };
    let mut s = Stream::new(1477.0, 60.0, Depth::Deep, None);
    s.push(&audio[..50_000], &mut |_| {});
    s.reset();
    let mut got = Vec::new();
    s.push(&audio, &mut |d| got.push(d));
    let whole = decode_slot(&audio, 1477.0, 60.0, Depth::Deep);
    assert!(same(&whole, &got), "{got:?} != {whole:?}");
}
