//! #600: `IqReceiver` delivers a channel's slots also as prefixes, at the
//! points its decoder asked for, so a wideband consumer reaches
//! `decode_prefix`. Design and the validation list these follow:
//! `docs/notes/IQ_PREFIX_DESIGN.md` §9.
#![cfg(all(feature = "ft8", feature = "ft4", feature = "fft-rustfft"))]

use std::collections::BTreeSet;

use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, Decoder, Depth, Ft8Strategy, Stage};
use mfsk_core::iq::{Channelizer, CompletedSlot, IqReceiver, IqSampleFormat, IqStream};

#[allow(dead_code)]
mod common;
use common::iq::{interleave, synth_iq};

const A: usize = 141_696;
const B: usize = 162_432;
const WHOLE: usize = 180_000;
/// A multiple of 15 s.
const T0_NS: i64 = 1_700_000_010 * 1_000_000_000;

/// Deterministic noise: no decode is wanted from it, only slots.
fn noise_iq(n: usize) -> Vec<f32> {
    let mut x: u32 = 0x1234_5678;
    (0..2 * n)
        .map(|_| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (x >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        })
        .collect()
}

const NFS: u32 = 48_000;
const NCENTER: f64 = 14_064_000.0;
const NDIAL: f64 = NCENTER + 10_000.0;

/// An FT8 channel over 31 s of noise, pushed `chunk` complex samples at a
/// time, with `points`.
fn noise_slots(chunk: usize, points: &[usize]) -> Vec<CompletedSlot> {
    let mut rx = IqReceiver::new(IqStream::new(NFS, NCENTER, IqSampleFormat::Cf32));
    let ch = rx.add_channel(NDIAL, Mode::Ft8).unwrap();
    assert!(rx.set_prefix_points(ch, points));
    rx.set_time(T0_NS, 0);
    let iq = noise_iq(31 * NFS as usize);
    let mut out = Vec::new();
    for c in iq.chunks(2 * chunk) {
        rx.push_cf32(c, &mut out);
    }
    out
}

/// Per period, the lengths delivered, in order.
fn lengths(slots: &[CompletedSlot]) -> Vec<(i64, Vec<usize>)> {
    let mut v: Vec<(i64, Vec<usize>)> = Vec::new();
    for s in slots {
        match v.last_mut() {
            Some((j, l)) if *j == s.period => l.push(s.audio.len()),
            _ => v.push((s.period, vec![s.audio.len()])),
        }
    }
    v
}

/// §9.2 and §9.3: each slot goes out at exactly A and B, then whole, in that
/// order, however the stream is pushed; a prefix's samples are the first of
/// the whole's, bit for bit (the level is pinned at the first prefix).
#[test]
fn prefixes_are_cut_at_the_points_and_share_the_wholes_level() {
    let reference = noise_slots(8_192, &[B, A]); // unsorted: the receiver sorts
    let want = lengths(&reference);
    assert_eq!(want.len(), 2, "{want:?}");
    for (_, l) in &want {
        assert_eq!(l, &[A, B, WHOLE]);
    }
    for chunk in [1, 1_000, 31 * NFS as usize] {
        let got = noise_slots(chunk, &[A, B]);
        assert_eq!(lengths(&got), want, "push of {chunk}");
        for (g, r) in got.iter().zip(&reference) {
            assert_eq!(g.audio, r.audio, "push of {chunk}, period {}", g.period);
        }
    }
    for j in [want[0].0, want[1].0] {
        let parts: Vec<&CompletedSlot> = reference.iter().filter(|s| s.period == j).collect();
        let whole = parts.last().unwrap();
        assert!(whole.is_whole());
        for p in &parts[..parts.len() - 1] {
            assert!(!p.is_whole());
            assert_eq!(p.start_sample, whole.start_sample);
            assert_eq!(p.utc_ns, whole.utc_ns);
            let n = p.audio.len();
            assert!(
                p.audio
                    .iter()
                    .zip(&whole.audio[..n])
                    .all(|(a, b)| a.to_bits() == b.to_bits()),
                "period {j}: prefix of {n} is not the whole's head"
            );
        }
    }
}

/// §9.1: without points a channel delivers whole slots only, at the level of
/// the whole slot, as before #600; and points that are not prefixes (0, a
/// whole slot or more) leave it so.
#[test]
fn without_points_only_whole_slots_go_out() {
    for points in [&[][..], &[0, WHOLE, WHOLE + 1][..]] {
        let got = noise_slots(8_192, points);
        let l = lengths(&got);
        assert_eq!(l.len(), 2);
        assert!(l.iter().all(|(_, l)| l == &[WHOLE]), "{points:?}: {l:?}");
        for s in &got {
            let rms = (s.audio.iter().map(|v| v * v).sum::<f32>() / s.audio.len() as f32).sqrt();
            assert!(
                (rms * 32_768.0 - 2_000.0).abs() < 1.0,
                "rms {}",
                rms * 32_768.0
            );
        }
    }
}

/// §9.6: a clock stepped back across an open slot reopens a period the
/// channel already delivered a prefix of. A channel with points does not
/// deliver it again, prefix or whole; a channel without delivers its whole as
/// it always has.
#[test]
fn a_period_is_not_reopened_after_the_clock_steps_back() {
    for with_points in [true, false] {
        let mut rx = IqReceiver::new(IqStream::new(NFS, NCENTER, IqSampleFormat::Cf32));
        let ch = rx.add_channel(NDIAL, Mode::Ft8).unwrap();
        if with_points {
            rx.set_prefix_points(ch, &[A, B]);
        }
        rx.set_time(T0_NS, 0);
        let iq = noise_iq(45 * NFS as usize);
        let mut out = Vec::new();
        // 12.5 s: past A of the first period, before B.
        let split = 2 * (12.5 * NFS as f64) as usize;
        rx.push_cf32(&iq[..split], &mut out);
        let first = T0_NS / 15_000_000_000;
        if with_points {
            assert_eq!(lengths(&out), vec![(first, vec![A])]);
        } else {
            assert!(out.is_empty());
        }
        // The clock says it is 1 s before that period began: it opens again
        // a second from now, with other audio.
        let at = rx.samples_in();
        rx.set_time(T0_NS - 1_000_000_000, at);
        rx.push_cf32(&iq[split..], &mut out);
        let l = lengths(&out);
        if with_points {
            assert_eq!(
                l[..2],
                [(first, vec![A]), (first + 1, vec![A, B, WHOLE])],
                "the reopened period went out: {l:?}"
            );
        } else {
            assert_eq!(l[0], (first, vec![WHOLE]), "{l:?}");
        }
    }
}

/// §9.5: the points follow the decoder's settings.
#[test]
fn prefix_points_follow_the_decoders_settings() {
    let mut d = AnyDecoder::with_defaults(Mode::Ft8);
    for depth in [Depth::Normal, Depth::Deep] {
        d.params_mut().depth = depth;
        assert_eq!(d.prefix_points(), &[A, B], "{depth:?}");
    }
    d.params_mut().depth = Depth::Fast;
    assert!(d.prefix_points().is_empty());

    let mut d = Decoder::<mfsk_core::Ft8>::with_defaults();
    d.params_mut().depth = Depth::Deep;
    d.extras_mut().tuning.strategy = Some(Ft8Strategy::SinglePass);
    assert!(d.prefix_points().is_empty(), "SinglePass");

    for &m in Mode::ALL {
        if m != Mode::Ft8 {
            assert!(
                AnyDecoder::with_defaults(m).prefix_points().is_empty(),
                "{m:?}"
            );
        }
    }
}

const FS: u32 = 192_000;
const CENTER: f64 = 14_077_000.0;
const DIAL: f64 = CENTER + 20_000.0;

/// §9.4: the FT8 recording through IQ, decoded with `decode_prefix` on every
/// delivery, ends with the messages the whole slot gives `decode`; the first
/// prefix already returns checkpoint A's rows, marked early, and they are in
/// the final set.
fn early_rows_through_iq(kind: Channelizer) {
    let Some(audio) = common::load_wav_i16_opt(asset_path!("qso3_busy.wav")) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let mut iq = synth_iq(&audio[..WHOLE], FS, CENTER, DIAL);
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));
    let iq = interleave(&iq);

    let run = |points: bool| {
        let mut rx =
            IqReceiver::with_channelizer(IqStream::new(FS, CENTER, IqSampleFormat::Cf32), kind)
                .unwrap();
        let ch = rx.add_channel(DIAL, Mode::Ft8).unwrap();
        let mut d = AnyDecoder::with_defaults(Mode::Ft8);
        if points {
            assert!(rx.set_prefix_points(ch, d.prefix_points()));
        }
        rx.set_time(T0_NS, 0);
        let mut slots = Vec::new();
        for c in iq.chunks(2 * 77_777) {
            rx.push_cf32(c, &mut slots);
        }
        slots
            .iter()
            .map(|s| {
                let r = if points {
                    d.decode_prefix(&s.input())
                } else {
                    d.decode(&s.input())
                };
                (s.audio.len(), r)
            })
            .collect::<Vec<_>>()
    };

    let whole = run(false);
    assert_eq!(whole.len(), 1);
    let want: BTreeSet<String> = whole[0].1.rows.iter().map(|r| r.text.clone()).collect();
    assert!(want.len() >= 10, "decode found {}", want.len());

    let calls = run(true);
    let lens: Vec<usize> = calls.iter().map(|(n, _)| *n).collect();
    assert_eq!(lens, [A, B, WHOLE]);
    let early = &calls[0].1;
    assert!(!early.rows.is_empty(), "checkpoint A returned nothing");
    assert!(early.details.iter().all(|d| d.stage == Some(Stage::Early)));
    assert!(calls[1].1.rows.is_empty(), "B returns nothing");
    let fin = &calls[2].1;
    let got: BTreeSet<String> = fin.rows.iter().map(|r| r.text.clone()).collect();
    println!(
        "  {kind:?}: whole {} final {} early {}",
        want.len(),
        got.len(),
        early.rows.len()
    );
    for r in &early.rows {
        assert!(
            got.contains(&r.text),
            "early row {} not in the final set",
            r.text
        );
    }
    // The prefix sequence pins the level at 11.8 s, the whole slot at 15 s:
    // the same signals at a level a few percent apart, so the weakest can
    // tip either way (as `f32` through the prefixes, `decoder_prefix.rs`).
    let known: Vec<&str> = common::ft8_qso3::QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .map(|e| e.msg)
        .collect();
    let extra: Vec<_> = got
        .difference(&want)
        .filter(|t| !known.contains(&t.as_str()))
        .collect();
    let missing: Vec<_> = want.difference(&got).collect();
    assert!(extra.is_empty(), "phantom {extra:?}");
    assert!(missing.len() <= want.len() / 10, "lost {missing:?}");
}

#[test]
fn early_rows_through_iq_direct() {
    early_rows_through_iq(Channelizer::Direct);
}

#[test]
fn early_rows_through_iq_pfb() {
    early_rows_through_iq(Channelizer::Pfb);
}
