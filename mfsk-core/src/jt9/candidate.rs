// SPDX-License-Identifier: GPL-3.0-only
//! Decode one JT9 candidate at a known position: [`decode_candidate`].
//!
//! The plain aligned demodulator plus Fano at the default cycle budget: no sync search, AFC or SNR estimate, which is why
//! the message comes back alone and not as a [`super::decode`]-style result. There is nothing to configure yet: the candidate
//! is its [`Alignment`], in a struct of its own so that an option can be added later without a breaking change, as the other
//! modes' candidates are.

use crate::engine::alignment::Alignment;
use crate::msg::Jt72Message;

/// One JT9 candidate: where it is.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Jt9Candidate {
    pub at: Alignment,
}

impl Jt9Candidate {
    pub const fn new(at: Alignment) -> Self {
        Self { at }
    }
}

/// Decode `cand` from f32 PCM `audio` at `sample_rate`. `None` when Fano does not converge or the payload does not unpack.
pub fn decode_candidate(
    audio: &[f32],
    sample_rate: u32,
    cand: &Jt9Candidate,
) -> Option<Jt72Message> {
    super::decode_at_inner(audio, sample_rate, cand.at.start_sample, cand.at.freq_hz)
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

    fn at(start: usize, freq: f32) -> Jt9Candidate {
        Jt9Candidate::new(Alignment::new(start, freq))
    }

    /// The candidate's place is what is decoded: at its start and frequency the message comes out; at the slot's start, 3 Hz
    /// off, or read at another sample rate it does not.
    #[test]
    fn the_alignment_places_the_candidate() {
        let audio = clean(1500.0);
        let msg = decode_candidate(&audio, 12_000, &at(12_000, 1500.0)).expect("aligned decode");
        let text = format!("{msg:?}");
        assert!(text.contains("K1ABC") && text.contains("W9XYZ"), "{text}");
        assert!(decode_candidate(&audio, 12_000, &at(0, 1500.0)).is_none());
        assert!(decode_candidate(&audio, 12_000, &at(12_000, 1503.0)).is_none());
        assert!(decode_candidate(&audio, 8_000, &at(12_000, 1500.0)).is_none());
    }
}
