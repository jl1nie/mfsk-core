//! `Decoder::<Ft8>::decode_candidates`: the candidates the caller gives in place of the decoder's own coarse search, everything after the search unchanged.
//!
//! **Fails if** (1) the rows of `decode_candidates` on the very candidates `coarse_sync` found differ, in any bit of the message, frequency, DT or SNR, from those of `decode` for `SinglePass`
//! (the injection changed the triage, the ladder or the SNR), or, for `SicRounds(2)` (no re-search of the residual), contain a message `decode` did not find or fewer than 80 % of its rows, on three real recordings; (2) candidates on pure noise decode anything (the entry lets a false
//! decode through: 200 candidates on each of three noise levels); (3) a candidate that is not finite, outside the band or outside the period panics or decodes; (4) `SicEarly` (the default at `Deep`)
//! or a sniper is accepted instead of refused; (6) a dense grid of candidates (9 places round each of the search's first 150) at the default `ApMode::Full` on three busy recordings
//! decodes a message that is neither `jt9`'s nor attested, or fewer than half of `jt9`'s; (7) `eme_delay` does more than shift a row's DT by 2 s; (5) the search's own candidates, **given in the reverse order**, loses a decode that the search order
//! finds (the order is the caller's, the result must not depend on it beyond the subtraction's own order effects: here, none lost).
#![cfg(all(feature = "ft8", feature = "fft-rustfft"))]
#[allow(dead_code)]
mod common;

use mfsk_core::Ft8;
use mfsk_core::decoder::{
    ApMode, CandidatesError, DecodeParams, Decoder, Depth, Ft8Candidate, Ft8Extras, Ft8Strategy,
    SlotInput, Sniper,
};
use mfsk_core::ft8::decode_block::{coarse_sync, compute_spectrogram};

fn audio(name: &str) -> Vec<i16> {
    common::load_wav_i16(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/").to_string() + name + ".wav",
    )
}

fn decoder(strategy: Ft8Strategy) -> Decoder<Ft8> {
    decoder_with(strategy, ApMode::Off, (200.0, 3000.0), false)
}

fn decoder_with(strategy: Ft8Strategy, ap: ApMode, band: (f32, f32), eme: bool) -> Decoder<Ft8> {
    let mut e = Ft8Extras::default();
    e.tuning.strategy = Some(strategy);
    e.tuning.sync_min = Some(1.3);
    e.tuning.max_cand = Some(1000);
    let mut p = DecodeParams::for_band(band).depth(Depth::Deep).ap(ap);
    p.eme_delay = eme;
    Decoder::<Ft8>::new(p).with_extras(e)
}

type Keys = Vec<(String, u32, u32, u32)>;

fn keys(r: &mfsk_core::decoder::SlotResult<mfsk_core::engine::frame::DecodeResult>) -> Keys {
    r.rows
        .iter()
        .map(|x| {
            (
                x.decoded.text.clone(),
                x.decoded.freq_hz.to_bits(),
                x.decoded.dt_sec.to_bits(),
                x.decoded.snr_db.to_bits(),
            )
        })
        .collect()
}

fn search(a: &[i16]) -> Vec<Ft8Candidate> {
    coarse_sync(&compute_spectrogram(a, 3000.0), 200.0, 3000.0, 1.3, 1000)
        .iter()
        .map(|c| Ft8Candidate::new(c.freq_hz, c.dt_sec).score(c.score))
        .collect()
}

#[test]
fn the_searchs_own_candidates_give_the_decodes_of_decode() {
    for name in ["qso3_busy", "191111_110130", "websdr_test4"] {
        let a = audio(name);
        let cands = search(&a);
        assert!(cands.len() > 50, "{name}: {} candidates", cands.len());
        for strategy in [Ft8Strategy::SinglePass, Ft8Strategy::SicRounds(2)] {
            let want = keys(&decoder(strategy).decode(&SlotInput::i16(&a)));
            let got = keys(
                &decoder(strategy)
                    .decode_candidates(&SlotInput::i16(&a), &cands)
                    .expect("supported strategy"),
            );
            assert!(
                !want.is_empty(),
                "{name} {strategy:?}: nothing decoded, the comparison is empty"
            );
            if strategy == Ft8Strategy::SinglePass {
                assert_eq!(got, want, "{name} {strategy:?}");
            } else {
                let t = |k: &Keys| -> std::collections::BTreeSet<String> {
                    k.iter().map(|x| x.0.clone()).collect()
                };
                // given-only: the rounds after the first do not search the residual again, so a decode that only a new candidate finds is absent (measured: 4 of 24 on websdr_test4), and nothing is added
                assert!(
                    t(&got).is_subset(&t(&want)),
                    "{name} {strategy:?}: given-only decoded what decode() did not: {:?}",
                    t(&got).difference(&t(&want)).collect::<Vec<_>>()
                );
                assert!(
                    got.len() * 10 >= want.len() * 8,
                    "{name} {strategy:?}: {} of {}",
                    got.len(),
                    want.len()
                );
            }
        }
    }
}

#[test]
fn candidates_on_noise_decode_nothing() {
    let mut x = 0x9E37_79B9_7F4A_7C15u64;
    let mut unit = move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x >> 40) as f32 / (1u64 << 24) as f32
    };
    for sigma in [50.0f32, 800.0, 6000.0] {
        let noise: Vec<i16> = (0..180_000)
            .map(|_| {
                (((unit() + unit() + unit() + unit()) - 2.0) * 1.732 * sigma)
                    .clamp(-32000.0, 32000.0) as i16
            })
            .collect();
        let cands: Vec<Ft8Candidate> = (0..200)
            .map(|_| Ft8Candidate::new(200.0 + 2800.0 * unit(), -1.0 + 3.0 * unit()))
            .collect();
        for strategy in [Ft8Strategy::SinglePass, Ft8Strategy::SicRounds(3)] {
            let r = decoder(strategy)
                .decode_candidates(&SlotInput::i16(&noise), &cands)
                .unwrap();
            assert!(
                r.rows.is_empty(),
                "sigma {sigma} {strategy:?}: {:?}",
                r.rows
                    .iter()
                    .map(|x| x.decoded.text.clone())
                    .collect::<Vec<_>>()
            );
        }
    }
}

#[test]
fn a_candidate_that_cannot_be_one_is_ignored() {
    let a = audio("qso3_busy");
    // a real signal's own place, given to a decoder whose band excludes it: the band filter is what keeps it out
    let rows = decoder(Ft8Strategy::SinglePass)
        .decode(&SlotInput::i16(&a))
        .rows;
    let strong = rows
        .iter()
        .max_by(|x, y| x.decoded.snr_db.total_cmp(&y.decoded.snr_db))
        .expect("a decode");
    let (f, dt) = (strong.decoded.freq_hz, strong.decoded.dt_sec);
    let real = [Ft8Candidate::new(f, dt)];
    let hit = decoder(Ft8Strategy::SinglePass)
        .decode_candidates(&SlotInput::i16(&a), &real)
        .unwrap();
    assert!(
        hit.rows
            .iter()
            .any(|r| r.decoded.text == strong.decoded.text),
        "the real candidate decodes in the full band"
    );
    let out_of_band = decoder_with(
        Ft8Strategy::SinglePass,
        ApMode::Off,
        (f + 100.0, 3000.0),
        false,
    )
    .decode_candidates(&SlotInput::i16(&a), &real)
    .unwrap();
    assert!(
        out_of_band
            .rows
            .iter()
            .all(|r| r.decoded.text != strong.decoded.text),
        "a candidate outside the band decoded"
    );
    // not finite, or nowhere near a period
    let bad = [
        Ft8Candidate::new(f32::NAN, 0.0),
        Ft8Candidate::new(1000.0, f32::INFINITY),
        Ft8Candidate::new(f, dt + 10.0),
        Ft8Candidate::new(f, dt - 10.0),
    ];
    let r = decoder(Ft8Strategy::SicRounds(2))
        .decode_candidates(&SlotInput::i16(&a), &bad)
        .unwrap();
    assert!(r.rows.is_empty());
    assert!(
        decoder(Ft8Strategy::SinglePass)
            .decode_candidates(&SlotInput::i16(&a), &[])
            .unwrap()
            .rows
            .is_empty()
    );
}

#[test]
fn eme_delay_only_shifts_the_rows_dt_by_two_seconds() {
    // `eme_delay` shifts a row's DT by 2 s and changes nothing else: text and SNR bit for bit. (The next round's exclusion of an already decoded candidate compares in the unshifted frame; a
    // wrong frame only retries decoded candidates, which the rows do not show: it costs time, so no row-level test can tell, and this one does not claim to.)
    let a = audio("qso3_busy");
    let cands = search(&a);
    let run = |eme: bool| {
        decoder_with(Ft8Strategy::SicRounds(3), ApMode::Off, (200.0, 3000.0), eme)
            .decode_candidates(&SlotInput::i16(&a), &cands)
            .unwrap()
    };
    let (off, on) = (run(false), run(true));
    let v = |r: &mfsk_core::decoder::SlotResult<mfsk_core::engine::frame::DecodeResult>| {
        r.rows
            .iter()
            .map(|x| (x.decoded.text.clone(), x.decoded.snr_db.to_bits()))
            .collect::<Vec<_>>()
    };
    assert_eq!(v(&on), v(&off));
    for (x, y) in on.rows.iter().zip(off.rows.iter()) {
        assert!((x.decoded.dt_sec - y.decoded.dt_sec - 2.0).abs() < 1e-3);
    }
}

/// jt9's own decodes of a recording (`fixtures/ft8_lib_jt9_beta6.tsv`) and what is attested beside them.
fn jt9_messages(file: &str) -> std::collections::BTreeSet<String> {
    include_str!("fixtures/ft8_lib_jt9_beta6.tsv")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
        .filter_map(|l| {
            let f: Vec<&str> = l.split('\t').collect();
            (f[0] == file).then(|| f[4].to_string())
        })
        .collect()
}

#[test]
fn a_dense_grid_of_candidates_on_busy_recordings_adds_no_phantom_at_the_default_ap() {
    // Candidates the search would have suppressed (a grid of 9 places round each of its first 150, 1 350 in all) and `ApMode::Full`, the default and the phantom-prone path: every message
    // decoded is one of jt9's or an attested extra. Fails if the entry lets an unattested message through that `decode()` of the same recording never produced.
    for (name, file, attested) in [
        ("websdr_test5", "websdr_test5", &[][..]),
        ("websdr_test4", "websdr_test4", &["CQ CU2DX HM77"][..]),
        (
            "websdr_test7",
            "websdr_test7",
            &["CU2DX R2DQA KO96", "EA8PP JH0INP PM96"][..],
        ),
    ] {
        let a = audio(name);
        let base = search(&a);
        let mut grid = Vec::new();
        for c in base.iter().take(150) {
            for df in [-3.0f32, 0.0, 3.0] {
                for ddt in [-0.04f32, 0.0, 0.04] {
                    grid.push(Ft8Candidate::new(c.freq_hz + df, c.dt_sec + ddt).score(c.score));
                }
            }
        }
        let mut e = Ft8Extras::default();
        e.tuning.strategy = Some(Ft8Strategy::SicRounds(2));
        e.tuning.max_cand = Some(2000);
        let mut d = Decoder::<Ft8>::new(
            DecodeParams::for_band((200.0, 3000.0))
                .depth(Depth::Deep)
                .ap(ApMode::Full),
        )
        .with_extras(e);
        let r = d.decode_candidates(&SlotInput::i16(&a), &grid).unwrap();
        let truth = jt9_messages(file);
        let got: std::collections::BTreeSet<String> =
            r.rows.iter().map(|x| x.decoded.text.clone()).collect();
        let extras: Vec<&String> = got
            .iter()
            .filter(|m| !truth.contains(*m) && !attested.contains(&m.as_str()))
            .collect();
        assert!(
            extras.is_empty(),
            "{name}: not in jt9's list and not attested: {extras:?}"
        );
        assert!(
            got.intersection(&truth).count() * 2 >= truth.len(),
            "{name}: only {} of jt9's {} (the guard would be vacuous)",
            got.intersection(&truth).count(),
            truth.len()
        );
    }
}

#[test]
fn the_strategies_with_a_search_of_their_own_are_refused() {
    let a = audio("qso3_busy");
    let cands = [Ft8Candidate::new(1500.0, 0.0)];
    assert_eq!(
        decoder(Ft8Strategy::SicEarly)
            .decode_candidates(&SlotInput::i16(&a), &cands)
            .unwrap_err(),
        CandidatesError::UnsupportedStrategy
    );
    // the default at `Deep` is `SicEarly`
    let mut d = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3000.0)).depth(Depth::Deep));
    assert_eq!(
        d.decode_candidates(&SlotInput::i16(&a), &cands)
            .unwrap_err(),
        CandidatesError::UnsupportedStrategy
    );
    // a sniper
    let mut e = Ft8Extras::default();
    e.tuning.strategy = Some(Ft8Strategy::SinglePass);
    e.sniper = Some(Sniper::new(100.0));
    let mut p = DecodeParams::for_band((200.0, 3000.0));
    p.rx_freq_hz = Some(1500.0);
    assert_eq!(
        Decoder::<Ft8>::new(p)
            .with_extras(e)
            .decode_candidates(&SlotInput::i16(&a), &cands)
            .unwrap_err(),
        CandidatesError::UnsupportedStrategy
    );
}

#[test]
fn the_order_of_the_given_candidates_loses_no_decode() {
    let a = audio("qso3_busy");
    let cands = search(&a);
    let mut rev = cands.clone();
    rev.reverse();
    let strategy = Ft8Strategy::SicRounds(2);
    let fwd = keys(
        &decoder(strategy)
            .decode_candidates(&SlotInput::i16(&a), &cands)
            .unwrap(),
    );
    let back = keys(
        &decoder(strategy)
            .decode_candidates(&SlotInput::i16(&a), &rev)
            .unwrap(),
    );
    let texts = |k: &Keys| -> std::collections::BTreeSet<String> {
        k.iter().map(|x| x.0.clone()).collect()
    };
    assert!(
        texts(&fwd).is_subset(&texts(&back)),
        "lost with the reversed order: {:?}",
        texts(&fwd).difference(&texts(&back)).collect::<Vec<_>>()
    );
}
