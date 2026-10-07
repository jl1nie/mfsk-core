// SPDX-License-Identifier: GPL-3.0-only
//! `Decoder::decode_prefix` (#572, `docs/notes/EARLY_DECODE_DESIGN.md` §6):
//! FT8's checkpoints A (141 696 samples), B (162 432) and the end, called on
//! growing prefixes of one period, against the whole-period `decode` of the
//! same audio.
//!
//! The recording is `qso3_busy.wav` cut to 180 000 samples, one period: the
//! prefix calls pad to that length, and the equivalence is for audio that is
//! exactly a period long (a longer buffer reaches `build_fft_cache`'s
//! 192 000-point window with samples a prefix never had).

#![cfg(feature = "ft8")]

use std::sync::Mutex;

use mfsk_core::decoder::{
    AnyDecoder, Decoder, Ft8Extras, Ft8Strategy, Row, SlotInput, SlotResult, Stage,
};
use mfsk_core::engine::pipeline::DecodeResult;
use mfsk_core::{Ft4, Ft8, Mode};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const A: usize = 141_696;
const B: usize = 162_432;
const FULL: usize = 180_000;

fn period() -> Option<Vec<i16>> {
    let Some(mut a) = load_wav_i16_opt(asset_path!("qso3_busy.wav")) else {
        common::skip_or_fail("qso3_busy.wav");
        return None;
    };
    a.truncate(FULL);
    Some(a)
}

/// What two results must agree on: text, the message bits, and the exact
/// frequency and time, in order.
fn key(r: &SlotResult<DecodeResult>) -> Vec<(String, Vec<u8>, u32, u32)> {
    r.rows
        .iter()
        .map(|row| {
            (
                row.decoded.text.clone(),
                row.detail.info.clone(),
                row.decoded.freq_hz.to_bits(),
                row.decoded.dt_sec.to_bits(),
            )
        })
        .collect()
}

fn calls(
    d: &mut Decoder<Ft8>,
    audio: &[i16],
    at: &[usize],
    n: i64,
) -> Vec<SlotResult<DecodeResult>> {
    at.iter()
        .map(|&len| d.decode_prefix(&SlotInput::i16(&audio[..len]).period(n)))
        .collect()
}

/// Validation 1: calls at A, B and the end give the whole-period `SicEarly`'s
/// rows, in its order; the A call returns rows early and marks them.
#[test]
fn prefixes_at_the_checkpoints_end_where_decode_does() {
    let Some(audio) = period() else { return };
    let whole = Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&audio).period(7));
    assert!(whole.rows.len() >= 20, "{} rows", whole.rows.len());

    let mut d = Decoder::<Ft8>::with_defaults();
    let out = calls(&mut d, &audio, &[A, B, FULL], 7);
    let early = &out[0];
    assert!(!early.rows.is_empty(), "checkpoint A found nothing");
    assert!(
        early
            .rows
            .iter()
            .all(|r| r.detail.stage == Some(Stage::Early))
    );
    assert!(out[1].rows.is_empty(), "checkpoint B searches nothing");
    assert_eq!(
        key(&out[2]),
        key(&whole),
        "the final call is the whole-period decode"
    );

    // In the complete set, A's rows say they came early and the rest at the end.
    let n_early = early.rows.len();
    for (i, row) in out[2].rows.iter().enumerate() {
        let want = if i < n_early {
            Stage::Early
        } else {
            Stage::Final
        };
        assert_eq!(
            row.detail.stage,
            Some(want),
            "row {i}: {}",
            row.decoded.text
        );
    }
    // `CQ DX DL8YHR JO41` is the row only checkpoint C finds.
    assert!(
        out[2].rows[n_early..]
            .iter()
            .any(|r| r.decoded.text == "CQ DX DL8YHR JO41")
    );
}

/// Validations 5 and 6: a skipped middle call, two middle calls, a call
/// between checkpoints, a repeated call, and the final call alone all end
/// with the same rows.
#[test]
fn every_call_sequence_ends_with_the_same_rows() {
    let Some(audio) = period() else { return };
    let whole = key(&Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&audio).period(3)));
    for seq in [
        &[FULL][..],
        &[A, FULL],
        &[B, FULL],
        &[A, A, B, B, FULL],
        &[100_000, A, 150_000, B, 170_000, FULL],
    ] {
        let mut d = Decoder::<Ft8>::with_defaults();
        let out = calls(&mut d, &audio, seq, 3);
        assert_eq!(key(out.last().unwrap()), whole, "{seq:?}");
        // A call before A, between checkpoints, or repeating one returns nothing.
        for (len, r) in seq.iter().zip(&out) {
            if *len < A || (*len != A && *len != B && *len < FULL) {
                assert!(r.rows.is_empty(), "{seq:?}: {len} returned rows");
            }
        }
        let firsts: Vec<_> = seq.iter().zip(&out).filter(|(l, _)| **l == A).collect();
        if firsts.len() == 2 {
            assert!(
                firsts[1].1.rows.is_empty(),
                "a repeated A call returned rows again"
            );
        }
    }
}

/// After the final call the period is done: another call for it returns the
/// complete set again and delivers nothing.
#[test]
fn a_call_after_the_final_one_returns_the_set_without_decoding() {
    let Some(audio) = period() else { return };
    let mut d = Decoder::<Ft8>::with_defaults();
    let first = d.decode_prefix(&SlotInput::i16(&audio).period(1));
    let seen = Mutex::new(0usize);
    let again = d.decode_prefix_with(&SlotInput::i16(&audio).period(1), &|_| {
        *seen.lock().unwrap() += 1
    });
    assert_eq!(key(&again), key(&first));
    assert_eq!(*seen.lock().unwrap(), 0);
}

/// Each row is streamed once across the period's calls, and the final set
/// pairs with what was streamed by `delivery`, earlier calls included.
#[test]
fn deliveries_count_across_the_periods_calls() {
    let Some(audio) = period() else { return };
    let mut d = Decoder::<Ft8>::with_defaults();
    let streamed: Mutex<Vec<Row<DecodeResult>>> = Mutex::new(Vec::new());
    let cb = |r: &Row<DecodeResult>| streamed.lock().unwrap().push(r.clone());
    let mut last = None;
    for len in [A, B, FULL] {
        last = Some(d.decode_prefix_with(&SlotInput::i16(&audio[..len]).period(5), &cb));
    }
    let last = last.unwrap();
    let streamed = streamed.into_inner().unwrap();
    assert_eq!(streamed.len(), last.rows.len(), "each row once");
    for (i, s) in streamed.iter().enumerate() {
        assert_eq!(s.detail.delivery, Some(i as u32));
    }
    for row in &last.rows {
        let at = row
            .detail
            .delivery
            .expect("every returned row was streamed") as usize;
        assert_eq!(streamed[at].detail.info, row.detail.info);
        assert_eq!(streamed[at].detail.stage, row.detail.stage);
    }
}

/// `audio` plus white noise of RMS `sigma`, from a fixed seed.
fn noisy(audio: &[i16], sigma: f32) -> Vec<i16> {
    let mut s = 7u64;
    let mut u = || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s >> 11) as f32 / (1u64 << 53) as f32
    };
    audio
        .iter()
        .map(|&v| {
            let g = (-2.0 * u().max(1e-12).ln()).sqrt() * (std::f32::consts::TAU * u()).cos();
            (v as f32 + sigma * g).round().clamp(-32_768.0, 32_767.0) as i16
        })
        .collect()
}

/// Validation 7: a7 reads a period's rows only when its final call came.
/// Period 12 is the recording under noise of its own RMS, where the search
/// keeps 2 rows and a7, fed period 10, adds 6 (measured).
#[test]
fn a7_remembers_a_period_only_through_its_final_call() {
    let Some(audio) = period() else { return };
    let rms = (audio.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / audio.len() as f64).sqrt();
    let later = noisy(&audio, rms as f32);
    let a7 = || {
        let mut e = Ft8Extras::default();
        e.a7 = true;
        Decoder::<Ft8>::with_defaults().with_extras(e)
    };
    let next = |d: &mut Decoder<Ft8>| key(&d.decode(&SlotInput::i16(&later).period(12)));
    let mut probe = a7();
    probe.decode(&SlotInput::i16(&audio).period(10));
    let (remembered, fresh) = (next(&mut probe), next(&mut a7()));
    assert!(
        remembered.len() > fresh.len(),
        "a7 adds nothing here: {} vs {}",
        remembered.len(),
        fresh.len()
    );

    let mut by_decode = a7();
    by_decode.decode(&SlotInput::i16(&audio).period(10));
    let mut by_prefix = a7();
    calls(&mut by_prefix, &audio, &[A, B, FULL], 10);
    assert_eq!(next(&mut by_prefix), next(&mut by_decode));

    let mut unfinished = a7();
    calls(&mut unfinished, &audio, &[A, B], 10);
    let mut fresh = a7();
    assert_eq!(next(&mut unfinished), next(&mut fresh));
}

/// Validation 8: the strategy is the one the period's first call saw.
#[test]
fn the_first_calls_strategy_holds_for_the_period() {
    let Some(audio) = period() else { return };
    let mut d = Decoder::<Ft8>::with_defaults();
    let whole = key(&Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&audio).period(2)));
    d.decode_prefix(&SlotInput::i16(&audio[..A]).period(2));
    d.extras_mut().tuning.strategy = Some(Ft8Strategy::SinglePass);
    let end = d.decode_prefix(&SlotInput::i16(&audio).period(2));
    assert_eq!(key(&end), whole);
    // The next period starts under the new one: single pass, nothing early.
    assert!(
        d.decode_prefix(&SlotInput::i16(&audio[..A]).period(3))
            .rows
            .is_empty()
    );
}

/// Validation 9: without a period a prefix call is a one-shot `decode` and
/// keeps nothing.
#[test]
fn without_a_period_a_prefix_call_is_a_one_shot_decode() {
    let Some(audio) = period() else { return };
    let mut d = Decoder::<Ft8>::with_defaults();
    let once = key(&d.decode_prefix(&SlotInput::i16(&audio[..A])));
    let twice = key(&d.decode_prefix(&SlotInput::i16(&audio[..A])));
    assert_eq!(once, twice);
    assert_eq!(
        once,
        key(&Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&audio[..A])))
    );
}

/// Validation 4, as far as it can hold: `f32` audio is scaled by the gain of
/// the period's first prefix, so it is not byte-equal to a whole-period
/// decode, but it finds the same messages.
#[test]
fn f32_audio_through_the_prefixes_finds_the_same_messages() {
    let Some(audio) = period() else { return };
    let f: Vec<f32> = audio.iter().map(|&v| v as f32 / 32_768.0 * 0.3).collect();
    let texts = |r: &SlotResult<DecodeResult>| {
        let mut t: Vec<_> = r.rows.iter().map(|r| r.decoded.text.clone()).collect();
        t.sort();
        t
    };
    let whole = texts(&Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&audio)));
    let mut d = Decoder::<Ft8>::with_defaults();
    for len in [A, B] {
        d.decode_prefix(&SlotInput::f32(&f[..len]).period(4));
    }
    let end = d.decode_prefix(&SlotInput::f32(&f).period(4));
    let got = texts(&end);
    let missing: Vec<_> = whole.iter().filter(|t| !got.contains(t)).collect();
    assert!(missing.len() <= 1, "missing {missing:?}");
}

/// A mode with no early decode returns nothing before the end, and the end
/// is `decode`: the mode-generic caller needs no capability check.
#[test]
fn a_mode_with_no_checkpoints_waits_for_the_whole_period() {
    let mut d = Decoder::<Ft4>::with_defaults();
    let slot = vec![0i16; 90_000];
    assert!(
        d.decode_prefix(&SlotInput::i16(&slot[..60_000]).period(1))
            .rows
            .is_empty()
    );
    let mut any = AnyDecoder::with_defaults(Mode::Ft4);
    let r = any.decode_prefix(&SlotInput::i16(&slot).period(1));
    assert!(r.rows.is_empty());
}

/// `AnyDecoder` runs the same sequence.
#[test]
fn any_decoder_runs_the_same_sequence() {
    let Some(audio) = period() else { return };
    let mut typed = Decoder::<Ft8>::with_defaults();
    let want = calls(&mut typed, &audio, &[A, B, FULL], 9);
    let mut any = AnyDecoder::with_defaults(Mode::Ft8);
    for (len, w) in [A, B, FULL].iter().zip(&want) {
        let got = any.decode_prefix(&SlotInput::i16(&audio[..*len]).period(9));
        let texts: Vec<_> = got.rows.iter().map(|r| r.text.clone()).collect();
        let want: Vec<_> = w.rows.iter().map(|r| r.decoded.text.clone()).collect();
        assert_eq!(texts, want, "at {len}");
        assert!(got.details.iter().all(|d| d.stage.is_some()));
    }
}
