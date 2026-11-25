// SPDX-License-Identifier: GPL-3.0-only
//! Decode one Q65 candidate at a known position: [`decode_candidate`].
//!
//! No coarse search: the caller has the position. What runs is the plain Bessel-metric BP decode, or the options of
//! [`Q65Candidate`]: an AP hint, the fast-fading metric, or a list of candidate codewords.

use alloc::sync::Arc;

use super::Q65Result;
use super::decode_request::{Q65SubMode, ctx_from_hash_table};
use super::rx::Q65Ap;
use crate::engine::alignment::Alignment;
use crate::fec::qra::FadingModel;
use crate::msg::ApHint;
use crate::msg::hash_table::CallsignHashTable;

/// One Q65 candidate and how to decode it.
///
/// **Precedence** (as `SniperRequest`'s was): `ap_list`, then `fading`, then `ap_hint`, then the plain decode. A candidate
/// that sets several is decoded by the first of these that is set: `ap_list` reads none of the others; `fading` also reads
/// `ap_hint` (and its `pileup`); `ap_hint` alone reads `pileup`, which is read only with it.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Q65Candidate<'a> {
    pub at: Alignment,
    /// A-priori information: a known call or "CQ" expectation (a gain of 2-4 dB at threshold on Q65-30A).
    pub ap_hint: Option<&'a ApHint>,
    /// With a MyCall + DxCall hint, leave the spare 78th bit free (`q65_ap.f90`, iaptype 3 under `lq65pileup`). Read only
    /// with `ap_hint`.
    pub pileup: bool,
    /// Full-AP list decoding: template matching of the listed 63-symbol codewords at this carrier, no BP. **A crate-native
    /// extension, not a 1:1 port** (#522): upstream has no point-decode mode to match.
    pub ap_list: Option<&'a [[i32; 63]]>,
    /// The fast-fading metric: the model and `b90 * Ts` (the spread bandwidth times the symbol period, dimensionless: `b90`
    /// in Hz is `b90_ts / SYMBOL_DT`).
    pub fading: Option<(FadingModel, f32)>,
    /// The callsign hash table that resolves a hashed call in the decoded message.
    pub hash_table: Option<Arc<CallsignHashTable>>,
}

impl<'a> Q65Candidate<'a> {
    /// A candidate at `at`, plain decode.
    pub const fn new(at: Alignment) -> Self {
        Self {
            at,
            ap_hint: None,
            pileup: false,
            ap_list: None,
            fading: None,
            hash_table: None,
        }
    }

    /// Set [`ap_hint`](Self::ap_hint).
    pub const fn ap_hint(mut self, hint: &'a ApHint) -> Self {
        self.ap_hint = Some(hint);
        self
    }

    /// Set [`pileup`](Self::pileup).
    pub const fn pileup(mut self, on: bool) -> Self {
        self.pileup = on;
        self
    }

    /// Set [`ap_list`](Self::ap_list).
    pub const fn ap_list(mut self, candidates: &'a [[i32; 63]]) -> Self {
        self.ap_list = Some(candidates);
        self
    }

    /// Set [`fading`](Self::fading).
    pub const fn fading(mut self, model: FadingModel, b90_ts: f32) -> Self {
        self.fading = Some((model, b90_ts));
        self
    }

    /// Set [`hash_table`](Self::hash_table).
    pub fn hash_table(mut self, table: Arc<CallsignHashTable>) -> Self {
        self.hash_table = Some(table);
        self
    }

    fn ap(&self) -> Option<Q65Ap<'a>> {
        self.ap_hint.map(|hint| Q65Ap {
            hint,
            pileup: self.pileup,
        })
    }
}

/// Decode `cand` from f32 PCM `audio` (`i16 / 32768`) for sub-mode `P`. `None` when the buffer is too short, BP fails or the
/// CRC rejects it, or the message does not unpack.
pub fn decode_candidate<P: Q65SubMode>(
    audio: &[f32],
    sample_rate: u32,
    cand: &Q65Candidate<'_>,
) -> Option<Q65Result> {
    let ctx = ctx_from_hash_table(cand.hash_table.as_ref());
    let (start, freq) = (cand.at.start_sample, cand.at.freq_hz);
    if let Some(candidates) = cand.ap_list {
        super::rx::decode_at_with_ap_list_for::<P>(
            audio,
            sample_rate,
            start,
            freq,
            candidates,
            &ctx,
        )
    } else if let Some((model, b90_ts)) = cand.fading {
        super::rx::decode_at_fading_for::<P>(
            audio,
            sample_rate,
            start,
            freq,
            b90_ts,
            model,
            cand.ap(),
            &ctx,
        )
    } else {
        match cand.ap() {
            Some(hint) => {
                super::rx::decode_at_with_ap_for::<P>(audio, sample_rate, start, freq, hint, &ctx)
            }
            None => super::rx::decode_at_for::<P>(audio, sample_rate, start, freq, &ctx),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::q65::{Q65a30, standard_qso_codewords, synthesize_standard};

    fn gauss_noise(n: usize, seed: u64) -> Vec<f32> {
        let mut s = seed.wrapping_add(1);
        let mut next = || {
            s = s
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((s >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
        };
        let mut out = Vec::with_capacity(n);
        while out.len() < n {
            let (u, v) = (next(), next());
            let m = (-2.0 * u.ln()).sqrt();
            out.push(m * (core::f32::consts::TAU * v).cos());
            out.push(m * (core::f32::consts::TAU * v).sin());
        }
        out.truncate(n);
        out
    }

    const SIGMA: f32 = 6.5;

    fn clean() -> Vec<f32> {
        synthesize_standard("K1ABC", "JA1ABC", "PM95", 12_000, 1500.0, 0.3).unwrap()
    }

    fn at() -> Alignment {
        Alignment::new(0, 1500.0)
    }

    fn run(audio: &[f32], c: &Q65Candidate) -> Option<Q65Result> {
        decode_candidate::<Q65a30>(audio, 12_000, c)
    }

    /// The candidate's place is what is decoded: at its start and frequency the message comes out; half a second late or 3 Hz
    /// off it does not (plain, and through the list, which matches at the carrier).
    #[test]
    fn the_alignment_places_the_candidate() {
        let audio = clean();
        let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        let r = run(&audio, &Q65Candidate::new(at())).expect("aligned decode");
        assert_eq!(r.message, "K1ABC JA1ABC PM95");
        assert!(run(&audio, &Q65Candidate::new(Alignment::new(6_000, 1500.0))).is_none());
        assert!(run(&audio, &Q65Candidate::new(Alignment::new(0, 1503.0))).is_none());
        assert!(
            run(
                &audio,
                &Q65Candidate::new(Alignment::new(0, 1503.0)).ap_list(&list)
            )
            .is_none()
        );
    }

    /// A clean signal decodes plain, so the options are set where they decide, in a noisy slot (sigma 6.5, seed 0 of a fixed
    /// generator; plain BP also loses seeds 1-11, found by a sweep): plain BP loses it; the AP hint finds it; the fast-fading
    /// metric alone loses it but finds it with the hint (the hint reaches the fading branch too); the candidate list finds it
    /// without BP (`iterations == 0`).
    #[test]
    fn the_ap_options_reach_the_decoder() {
        let clean = clean();
        let nz = gauss_noise(clean.len(), 0);
        let audio: Vec<f32> = clean.iter().zip(&nz).map(|(c, n)| c + n * SIGMA).collect();
        let hint = ApHint::new().with_call1("K1ABC").with_call2("JA1ABC");
        let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");

        assert!(
            run(&audio, &Q65Candidate::new(at())).is_none(),
            "plain BP loses this slot"
        );
        let hinted = run(&audio, &Q65Candidate::new(at()).ap_hint(&hint)).expect("AP hint");
        assert_eq!(hinted.message, "K1ABC JA1ABC PM95");
        assert!(
            run(
                &audio,
                &Q65Candidate::new(at()).fading(FadingModel::Lorentzian, 0.5)
            )
            .is_none(),
            "fading alone loses it too"
        );
        let faded_hint = run(
            &audio,
            &Q65Candidate::new(at())
                .fading(FadingModel::Lorentzian, 0.5)
                .ap_hint(&hint),
        )
        .expect("fading and the AP hint");
        assert_eq!(faded_hint.message, "K1ABC JA1ABC PM95");
        let listed = run(&audio, &Q65Candidate::new(at()).ap_list(&list)).expect("AP list");
        assert_eq!(listed.message, "K1ABC JA1ABC PM95");
        assert_eq!(listed.iterations, 0, "the list decode runs no BP");
    }

    /// The fading metric's model and `b90 * Ts` reach the decoder: a clean signal decodes plain, and under a Lorentzian of
    /// b90*Ts = 500, but not under the Gaussian of the same width (whose window leaves too little in the tails).
    #[test]
    fn the_fading_model_and_width_reach_the_decoder() {
        let audio = clean();
        assert!(run(&audio, &Q65Candidate::new(at())).is_some());
        assert!(
            run(
                &audio,
                &Q65Candidate::new(at()).fading(FadingModel::Lorentzian, 500.0)
            )
            .is_some()
        );
        assert!(
            run(
                &audio,
                &Q65Candidate::new(at()).fading(FadingModel::Gaussian, 500.0)
            )
            .is_none()
        );
    }

    /// Precedence: a list that does not hold the message is decoded by the list alone, with no fallback to BP and the
    /// fading metric not read: a clean signal that decodes plain, and under the fading metric, is not decoded.
    #[test]
    fn the_list_takes_precedence_over_fading_and_bp() {
        let audio = clean();
        let wrong = standard_qso_codewords("W9XYZ", "N0CALL", "EN52");
        assert!(
            run(
                &audio,
                &Q65Candidate::new(at()).fading(FadingModel::Lorentzian, 0.5)
            )
            .is_some()
        );
        assert!(run(&audio, &Q65Candidate::new(at()).ap_list(&wrong)).is_none());
        assert!(
            run(
                &audio,
                &Q65Candidate::new(at())
                    .ap_list(&wrong)
                    .fading(FadingModel::Lorentzian, 0.5)
            )
            .is_none()
        );
    }
}
