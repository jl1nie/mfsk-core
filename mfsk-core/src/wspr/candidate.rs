// SPDX-License-Identifier: GPL-3.0-only
//! Decode one WSPR candidate at a known position: [`decode_candidate`].
//!
//! No search: the caller has the position (a coarse search, an earlier pass, a spot list) and the audio, or a 375 Hz
//! baseband made once from it. What runs is wsprd's refine cascade around the position, the `minsync2` gate, then Fano
//! and, when [`WsprCandidate::confirmed`] is given, OSD. The embedded pipelines decode each candidate of a slot this way,
//! from one baseband built once (`WsprInput::baseband`), instead of decimating the audio for each.

use super::decode::{WsprCallsignTable, WsprResult};
use crate::engine::alignment::Alignment;

/// The signal a candidate is decoded from.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum WsprInput<'a> {
    /// 12 kHz audio. It is decimated with the reference whole-slot FFT channelizer on **every**
    /// [`decode_candidate`], whatever the `wspr-ddc*` features are: to decode many candidates of one slot, build the
    /// baseband once.
    Audio {
        samples: &'a [f32],
        sample_rate: u32,
    },
    /// A 375 Hz baseband, from [`super::baseband::decimate_to_baseband`] or one of the `wspr::ddc` down-converters:
    /// skips the O(NFFT1) decimation. **`sample_rate` is the rate of the audio the baseband was made from (12 000), not the
    /// baseband's 375 Hz**: it converts the candidate's audio-rate `start_sample` and the decoded lag, so a wrong value
    /// misplaces both.
    Baseband {
        idat: &'a [f32],
        qdat: &'a [f32],
        sample_rate: u32,
    },
}

impl<'a> WsprInput<'a> {
    /// 12 kHz-class audio: see [`WsprInput::Audio`].
    pub const fn audio(samples: &'a [f32], sample_rate: u32) -> Self {
        Self::Audio {
            samples,
            sample_rate,
        }
    }

    /// A baseband made once from audio at `sample_rate`: see [`WsprInput::Baseband`].
    pub const fn baseband(idat: &'a [f32], qdat: &'a [f32], sample_rate: u32) -> Self {
        Self::Baseband {
            idat,
            qdat,
            sample_rate,
        }
    }
}

/// One WSPR candidate and how to decode it. `at.freq_hz` is the tone-0 frequency in audio Hz (this crate's coarse search
/// and synthesiser convention; the decoder converts it to wsprd's tone-centre), `at.start_sample` the audio-rate sample where
/// symbol 0 starts.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct WsprCandidate<'a> {
    pub at: Alignment,
    /// Linear drift estimate, Hz across the 110.6 s frame (wsprd's `drift1`). Default 0. The refine cascade searches ±0.5 Hz
    /// around it while [`refine_drift`](Self::refine_drift) is on.
    pub drift_hz: f32,
    /// Coherent block lengths to try, in order (wsprd's `nblocksize` ladder). Default `&[1]`; the scan's final pass uses
    /// `&[1, 2, 3, 0]`, where `0` is this crate's spelling of wsprd's fourth rung (block size 1 with the alternate bit
    /// metric). The hot loop scales with the length.
    pub nblocks: &'a [usize],
    /// Callsigns an earlier Fano decode already established. An OSD result is accepted only if its callsign is in this table
    /// (wsprd's `hashtab` check, `wsprd.c:1396`); without it every OSD result is rejected, as OSD synthesises a valid codeword
    /// for any input and ungated is a phantom generator (see [`WsprCallsignTable`]).
    pub confirmed: Option<&'a WsprCallsignTable>,
    /// wsprd's per-pass drift switch (`wsprd.c:1236`, `if (ipass < 2)`). On (the default) the cascade tries `drift ± 0.5` and
    /// keeps the better sync, and `minsync2` is 0.12. Off, as on wsprd's final pass, drift is held fixed and `minsync2` drops to
    /// 0.10: drift tracking traded for a lower-variance frequency estimate on weak signals.
    pub refine_drift: bool,
}

impl<'a> WsprCandidate<'a> {
    /// A candidate at `at`: no drift, the one-block ladder, no OSD, drift refined.
    pub const fn new(at: Alignment) -> Self {
        Self {
            at,
            drift_hz: 0.0,
            nblocks: &[1],
            confirmed: None,
            refine_drift: true,
        }
    }

    /// Set [`drift_hz`](Self::drift_hz).
    pub const fn drift_hz(mut self, drift_hz: f32) -> Self {
        self.drift_hz = drift_hz;
        self
    }

    /// Set [`nblocks`](Self::nblocks).
    pub const fn nblocks(mut self, nblocks: &'a [usize]) -> Self {
        self.nblocks = nblocks;
        self
    }

    /// Set [`confirmed`](Self::confirmed).
    pub const fn confirmed(mut self, table: &'a WsprCallsignTable) -> Self {
        self.confirmed = Some(table);
        self
    }

    /// Set [`refine_drift`](Self::refine_drift).
    pub const fn refine_drift(mut self, on: bool) -> Self {
        self.refine_drift = on;
        self
    }
}

/// Decode `cand` from `input`. `None` when the refined sync misses `minsync2`, Fano and OSD both fail, or the payload does not
/// unpack. [`WsprResult::snr_db`] is `0.0`: the SNR comes from a coarse candidate, and a point decode has none.
pub fn decode_candidate(input: WsprInput<'_>, cand: &WsprCandidate<'_>) -> Option<WsprResult> {
    let decimated;
    let (idat, qdat, sample_rate): (&[f32], &[f32], u32) = match input {
        WsprInput::Audio {
            samples,
            sample_rate,
        } => {
            decimated = super::baseband::decimate_to_baseband(samples);
            (&decimated.0, &decimated.1, sample_rate)
        }
        WsprInput::Baseband {
            idat,
            qdat,
            sample_rate,
        } => (idat, qdat, sample_rate),
    };
    super::decode::decode_at_baseband_inner(
        idat,
        qdat,
        sample_rate,
        cand.at.start_sample,
        cand.at.freq_hz,
        cand.drift_hz,
        cand.nblocks,
        cand.confirmed,
        cand.refine_drift,
        super::decode::Ladder::DEFAULT,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slot(freq: f32) -> Vec<f32> {
        // `synthesize_type1` is the frame alone; a WSPR slot starts a second in.
        let tx = super::super::synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.5).unwrap();
        let mut a = vec![0.0f32; 12_000 + tx.len() + 12_000];
        a[12_000..12_000 + tx.len()].copy_from_slice(&tx);
        a
    }

    const START: usize = 12_000;

    fn at() -> Alignment {
        Alignment::new(START, 1500.0)
    }

    fn decodes(audio: &[f32], c: &WsprCandidate) -> Option<WsprResult> {
        decode_candidate(WsprInput::audio(audio, 12_000), c)
    }

    /// `slot` plus noise: a fixed LCG, four uniforms summed (about Gaussian), scaled by `sigma`.
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

    /// The candidate's place is what is decoded: at its start and frequency the message comes out; a second early or 2 Hz
    /// off it does not.
    #[test]
    fn the_alignment_places_the_candidate() {
        let audio = slot(1500.0);
        let r = decodes(&audio, &WsprCandidate::new(at())).expect("aligned decode");
        assert_eq!(
            r.message,
            crate::msg::WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37
            }
        );
        assert!(decodes(&audio, &WsprCandidate::new(Alignment::new(0, 1500.0))).is_none());
        assert!(decodes(&audio, &WsprCandidate::new(Alignment::new(START, 1502.0))).is_none());
    }

    /// A baseband made once and the audio it came from give the same decode.
    #[test]
    fn a_baseband_input_is_the_audio_input() {
        let audio = slot(1500.0);
        let (idat, qdat) = super::super::baseband::decimate_to_baseband(&audio);
        let c = WsprCandidate::new(at());
        let from_audio = decodes(&audio, &c).expect("audio");
        let from_bb =
            decode_candidate(WsprInput::baseband(&idat, &qdat, 12_000), &c).expect("baseband");
        assert_eq!(from_audio.message, from_bb.message);
        assert_eq!(from_audio.freq_hz.to_bits(), from_bb.freq_hz.to_bits());
        assert_eq!(from_audio.dt_sec.to_bits(), from_bb.dt_sec.to_bits());
    }

    /// Every field of a candidate reaches the decoder. A clean signal ignores most knobs, so each is set where it decides:
    /// a drift of 3 Hz is found by the refine cascade (±0.5 Hz around it, and the cascade reaches it) but missed when
    /// `refine_drift` is off and the drift is held at 3 Hz; 5 Hz is out of reach either way; an empty block ladder tries
    /// nothing; and OSD, which finds the message in a noisy slot (sigma 45, seed 0) that Fano alone loses, is accepted only
    /// through a table that holds its callsign.
    #[test]
    fn every_option_reaches_the_decoder() {
        let audio = slot(1500.0);
        let base = WsprCandidate::new(at());
        assert!(
            decodes(&audio, &base.drift_hz(3.0)).is_some(),
            "drift 3 Hz, refined"
        );
        assert!(
            decodes(&audio, &base.drift_hz(3.0).refine_drift(false)).is_none(),
            "drift 3 Hz held fixed misses"
        );
        assert!(
            decodes(&audio, &base.drift_hz(5.0)).is_none(),
            "drift 5 Hz is out of reach"
        );
        assert!(
            decodes(&audio, &base.nblocks(&[])).is_none(),
            "no rung, no decode"
        );

        let noisy_audio = noisy(&audio, 45.0, 0);
        let msg = crate::msg::WsprMessage::Type1 {
            callsign: "K1ABC".into(),
            grid: "FN42".into(),
            power_dbm: 37,
        };
        let mut known = WsprCallsignTable::new();
        known.record(&msg);
        let mut other = WsprCallsignTable::new();
        other.record(&crate::msg::WsprMessage::Type1 {
            callsign: "W9XYZ".into(),
            grid: "EN52".into(),
            power_dbm: 30,
        });
        let osd = base.nblocks(&[1, 2, 3, 0]).refine_drift(false);
        assert!(
            decodes(&noisy_audio, &osd).is_none(),
            "OSD is not accepted without a table"
        );
        assert!(
            decodes(&noisy_audio, &osd.confirmed(&other)).is_none(),
            "nor through a table without the call"
        );
        assert_eq!(
            decodes(&noisy_audio, &osd.confirmed(&known)).map(|r| r.message),
            Some(msg)
        );
    }

    /// The deprecated builder is the function: the same four option cases, set through `SniperRequest`, give the results
    /// above (a dropped option in the shim would flip one of them).
    #[allow(deprecated)]
    #[test]
    fn the_deprecated_builder_is_the_function() {
        use super::super::SniperRequest;
        let audio = slot(1500.0);
        let sn = |a: &'static [f32]| SniperRequest::new(a, 12_000, START, 1500.0);
        let audio: &'static [f32] = Box::leak(audio.into_boxed_slice());
        assert!(sn(audio).drift(3.0).decode().is_some());
        assert!(sn(audio).drift(3.0).refine_drift(false).decode().is_none());
        assert!(sn(audio).nblocks(&[]).decode().is_none());

        let noisy_audio: &'static [f32] = Box::leak(noisy(audio, 45.0, 0).into_boxed_slice());
        let msg = crate::msg::WsprMessage::Type1 {
            callsign: "K1ABC".into(),
            grid: "FN42".into(),
            power_dbm: 37,
        };
        let mut known = WsprCallsignTable::new();
        known.record(&msg);
        let osd = |r: SniperRequest<'static>| r.nblocks(&[1, 2, 3, 0]).refine_drift(false);
        assert!(osd(sn(noisy_audio)).decode().is_none());
        assert_eq!(
            osd(sn(noisy_audio))
                .confirmed(&known)
                .decode()
                .map(|r| r.message),
            Some(msg)
        );
    }
}
