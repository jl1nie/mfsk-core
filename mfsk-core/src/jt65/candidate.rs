// SPDX-License-Identifier: GPL-3.0-only
//! Decode one JT65 candidate at a known position: [`decode_candidate`].
//!
//! No search and no sync estimate: the aligned demodulator, then Reed-Solomon by the chosen [`Jt65Strategy`]. The message
//! comes back alone, as the point decodes always have: there is no SNR or sync to report.

use crate::engine::alignment::Alignment;
use crate::msg::Jt72Message;

use super::chase::ChaseParams;

/// How Reed-Solomon is run on the candidate.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum Jt65Strategy<'a> {
    /// Hard decisions only.
    Hard,
    /// Flag the least-confident symbols as RS erasures, trying each count in the list in order until one unpacks into a
    /// valid message. Each erasure buys one more correctable symbol (`2·errors + erasures ≤ 51`); `&[0, 8, 16, 24, 32]`
    /// is a reasonable ladder. Counts above 51 are clamped.
    Erasures(&'a [usize]),
    /// WSJT-X's `ftrsdap` stochastic Chase search: randomized erasure trials with its own acceptance gate (see
    /// [`super::chase`]).
    Chase(ChaseParams),
}

/// One JT65 candidate and how to decode it.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Jt65Candidate<'a> {
    pub at: Alignment,
    pub strategy: Jt65Strategy<'a>,
}

impl<'a> Jt65Candidate<'a> {
    /// A candidate at `at`, decoded hard-decision.
    pub const fn new(at: Alignment) -> Self {
        Self {
            at,
            strategy: Jt65Strategy::Hard,
        }
    }

    /// Run RS with erasures: see [`Jt65Strategy::Erasures`].
    pub fn erasures(mut self, attempts: &'a [usize]) -> Self {
        self.strategy = Jt65Strategy::Erasures(attempts);
        self
    }

    /// Run RS with the Chase search: see [`Jt65Strategy::Chase`].
    pub fn chase(mut self, params: ChaseParams) -> Self {
        self.strategy = Jt65Strategy::Chase(params);
        self
    }
}

/// Decode `cand` from 12 kHz-class f32 PCM `audio` at `sample_rate`. `None` when RS does not converge, the acceptance
/// gate rejects it, or the payload does not unpack.
pub fn decode_candidate(
    audio: &[f32],
    sample_rate: u32,
    cand: &Jt65Candidate<'_>,
) -> Option<Jt72Message> {
    let (start, freq) = (cand.at.start_sample, cand.at.freq_hz);
    match &cand.strategy {
        Jt65Strategy::Hard => {
            super::decode_at_with_snr(audio, sample_rate, start, freq).map(|(m, ..)| m)
        }
        Jt65Strategy::Erasures(attempts) => {
            super::decode_at_with_erasures(audio, sample_rate, start, freq, attempts)
        }
        Jt65Strategy::Chase(params) => {
            super::chase::decode_at_with_chase(audio, sample_rate, start, freq, params)
                .map(|(m, ..)| m)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clean(freq: f32) -> Vec<f32> {
        let tx =
            super::super::synthesize_standard("K1ABC", "W9XYZ", "FN42", 12_000, freq, 0.5).unwrap();
        let mut a = vec![0.0f32; 12_000 + tx.len() + 12_000];
        a[12_000..12_000 + tx.len()].copy_from_slice(&tx);
        a
    }

    fn noisy(clean: &[f32], sigma: f32, seed: u32) -> Vec<f32> {
        let mut x = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        clean
            .iter()
            .map(|&v| {
                let mut n = 0.0f32;
                for _ in 0..4 {
                    x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    n += (x >> 8) as f32 / (1u32 << 24) as f32 - 0.5;
                }
                v + n * sigma * 0.866
            })
            .collect()
    }

    const START: usize = 12_000;

    fn at() -> Alignment {
        Alignment::new(START, 1500.0)
    }

    fn decode(audio: &[f32], c: &Jt65Candidate) -> Option<Jt72Message> {
        decode_candidate(audio, 12_000, c)
    }

    /// The candidate's place is what is decoded: at its start and frequency the message comes out; at the slot's start or 3 Hz
    /// off it does not.
    #[test]
    fn the_alignment_places_the_candidate() {
        let audio = clean(1500.0);
        let msg = decode(&audio, &Jt65Candidate::new(at())).expect("aligned decode");
        let text = format!("{msg:?}");
        assert!(text.contains("K1ABC") && text.contains("W9XYZ"), "{text}");
        assert!(decode(&audio, &Jt65Candidate::new(Alignment::new(0, 1500.0))).is_none());
        assert!(decode(&audio, &Jt65Candidate::new(Alignment::new(START, 1503.0))).is_none());
    }

    /// Each strategy and what it is given reach the decoder. A clean signal decodes under all of them, so these are set in
    /// a noisy slot (sigma 16) where they part: seed 0 is lost by hard decisions and by the one-attempt ladder `[0]`
    /// (which is hard decisions), found by the full ladder and by Chase, and lost by Chase when it is allowed no trials;
    /// seed 1 only Chase finds.
    #[test]
    fn every_strategy_reaches_the_decoder() {
        let c = clean(1500.0);
        let want = decode(&c, &Jt65Candidate::new(at())).expect("clean");
        let ladder = [0usize, 8, 16, 24, 32];
        let base = Jt65Candidate::new(at());

        let a = noisy(&c, 16.0, 0);
        assert!(decode(&a, &base).is_none(), "hard decisions lose seed 0");
        assert_eq!(
            decode(&a, &base.clone().erasures(&ladder)),
            Some(want.clone())
        );
        assert!(
            decode(&a, &base.clone().erasures(&[0])).is_none(),
            "the ladder [0] is hard decisions"
        );
        assert!(
            decode(&a, &base.clone().erasures(&[])).is_none(),
            "no attempts, no decode"
        );
        assert_eq!(
            decode(&a, &base.clone().chase(ChaseParams::default())),
            Some(want.clone())
        );
        let no_trials = ChaseParams {
            max_trials: 0,
            ..ChaseParams::default()
        };
        assert!(
            decode(&a, &base.clone().chase(no_trials)).is_none(),
            "Chase with no trials is hard decisions"
        );

        let b = noisy(&c, 16.0, 1);
        assert!(decode(&b, &base.clone().erasures(&ladder)).is_none());
        assert_eq!(decode(&b, &base.chase(ChaseParams::default())), Some(want));
    }
}
