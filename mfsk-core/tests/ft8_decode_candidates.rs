//! `Decoder::<Ft8>::decode_candidates`: the candidates the caller gives in place of the decoder's own coarse search, everything after the search unchanged.
//!
//! **Fails if** (1) the rows of `decode_candidates` on the very candidates `coarse_sync` found differ, in any bit of the message, frequency, DT or SNR, from those of `decode` for `SinglePass`
//! (the injection changed the triage, the ladder or the SNR), or, for `SicRounds(2)` (no re-search of the residual), contain a message `decode` did not find or fewer than 80 % of its rows, on three real recordings; (2) candidates on pure noise decode anything (the entry lets a false
//! decode through: 200 candidates on each of three noise levels); (3) a candidate that is not finite, outside the band or outside the period panics or decodes; (4) `SicEarly` (the default at `Deep`)
//! or a sniper is accepted instead of refused; (5) a candidate list of a real recording's own decodes, **given in the order of a worse search** (reversed), loses a decode that the search order
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
    let mut e = Ft8Extras::default();
    e.tuning.strategy = Some(strategy);
    e.tuning.sync_min = Some(1.3);
    e.tuning.max_cand = Some(1000);
    Decoder::<Ft8>::new(
        DecodeParams::for_band((200.0, 3000.0))
            .depth(Depth::Deep)
            .ap(ApMode::Off),
    )
    .with_extras(e)
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
    let bad = [
        Ft8Candidate::new(f32::NAN, 0.0),
        Ft8Candidate::new(1000.0, f32::INFINITY),
        Ft8Candidate::new(5000.0, 0.0),
        Ft8Candidate::new(100.0, 0.0),
        Ft8Candidate::new(1000.0, 10.0),
        Ft8Candidate::new(1000.0, -10.0),
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
