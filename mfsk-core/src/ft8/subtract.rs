// SPDX-License-Identifier: GPL-3.0-or-later
//! FT8 signal subtraction (successive interference cancellation).
//!
//! Thin FT8-tuned wrapper around the protocol-agnostic
//! [`crate::engine::dsp::subtract`] implementation. Given a decoded message and
//! its time/frequency coordinates, reconstructs the ideal 8-GFSK waveform and
//! subtracts it in place so weaker signals become decodable.

use super::{decode::DecodeResult, wave_gen::message_to_tones};
use crate::engine::dsp::subtract::{
    GfskParams, SubtractCfg, subtract_tones_lpf, subtract_tones_lpf_refine_dt,
};

/// FT8 subtract configuration: 12 kHz sample rate, 6.25 Hz tone spacing,
/// 1920 samples/symbol, frame origin at 0.5 s, GFSK pulse shaping
/// matching `wave_gen::tones_to_*` (BT=2.0, hmod=1.0, ramp=nsps/8).
///
/// The GFSK shaping is required for correct subtract: without it, the
/// reference reverts to abrupt phase transitions and only achieves
/// ~-19 dB drop on a perfectly clean self-synthesised signal vs > -100 dB
/// with GFSK. See test `tests/ft8_subtract_self_test.rs`.
const FT8_CFG: SubtractCfg = SubtractCfg {
    sample_rate: 12_000.0,
    tone_spacing_hz: 6.25,
    samples_per_symbol: 1920,
    base_offset_s: 0.5,
    gfsk: Some(GfskParams {
        bt: 2.0,
        hmod: 1.0,
        ramp_samples: 1920 / 8,
    }),
};

/// WSJT-X-style channel-aware subtract for FT8. Wraps
/// [`crate::engine::dsp::subtract::subtract_tones_lpf`] with the FT8 cfg
/// and `lpf_half = 2000` matching WSJT-X NFILT=4000. The canonical
/// FT8 subtract entry point as of v0.6.2 — both the host
/// `decode_frame_subtract*` driver and the embedded
/// `decode_block_multipass` driver call this for every accepted
/// decode in their sequential-subtract loop.
///
/// Pre-v0.6.2 the host path used a constant-amplitude
/// `subtract_signal_weighted` (with QSB-aware partial gain) which
/// underused the residual on busy bands; see the v0.6.2 CHANGELOG
/// for the recall delta this rewire produced on `qso3_busy.wav`.
pub fn subtract_signal_lpf(audio: &mut [i16], result: &DecodeResult) {
    let tones = message_to_tones(result.message77());
    subtract_tones_lpf(
        audio,
        &tones,
        result.freq_hz,
        result.dt_sec,
        &FT8_CFG,
        2000,
        true, // endcorrection: matches subtractft8.f90
    );
}

/// `subtractft8.f90`'s `lrefinedt=.true.` variant of
/// [`subtract_signal_lpf`]: probes `dt` around `result.dt_sec` for the
/// offset that leaves the least residual energy in the signal's own
/// tone band, then subtracts there. Matches `ft8_decode.f90`'s two
/// early-decode-and-subtract checkpoints (issue #180) — candidates
/// found early in a staged/checkpointed SIC pass, whose `dt` hasn't had
/// a final decode pass to lock it down. Use plain [`subtract_signal_lpf`]
/// once a candidate's `dt` is already final.
pub fn subtract_signal_lpf_refine_dt(audio: &mut [i16], result: &DecodeResult) {
    let tones = message_to_tones(result.message77());
    subtract_tones_lpf_refine_dt(
        audio,
        &tones,
        result.freq_hz,
        result.dt_sec,
        &FT8_CFG,
        2000,
        true,
    );
}

/// Refine `result.freq_hz` by grid-searching ±2.5 Hz at 0.1 Hz resolution
/// for the carrier that maximises the LS amplitude of the GFSK reference
/// against `audio`. Returns the refined frequency.
///
/// Use this before [`subtract_signal_lpf`] when the input is a
/// real-WAV decode (not a self-synthesised signal). mfsk-core's
/// coarse_sync reports carriers on a 2.93 Hz bin grid; real signals
/// routinely sit ±0.5..3 Hz off-bin and the resulting phase drift
/// over the 12.7 s frame defeats the LS estimate inside
/// `subtract_tones_lpf`. Empirical: refines CQ F5RXL on qso3_busy
/// from 1198 → 1196.8 Hz, |amp| jumps 3.6 → 16.2 (~4.5×).
///
/// Cost: ~50 GFSK reference builds × ~150 k samples each. On host f32
/// this is a few ms per signal — call once per decoded result rather
/// than per pass-2 candidate.
pub fn refine_signal_freq(audio: &[i16], result: &DecodeResult) -> f32 {
    let tones = message_to_tones(result.message77());
    crate::engine::dsp::subtract::refine_freq(
        audio,
        &tones,
        result.freq_hz,
        result.dt_sec,
        &FT8_CFG,
        2.5,
        0.1,
    )
}

#[cfg(test)]
mod tests {
    use super::super::decode::DecodeStrictness;
    use super::super::wave_gen::{message_to_tones, tones_to_i16};
    use super::*;

    /// Build a 91-bit `info` (K for LDPC174_91) from a 77-bit message,
    /// zero-padding the CRC-14 tail — `message77()` only reads the
    /// leading 77 bits, so the padding is never exercised by these
    /// signal-reconstruction tests.
    fn info91(msg77: [u8; 77]) -> Box<[u8]> {
        let mut info = vec![0u8; 91];
        info[..77].copy_from_slice(&msg77);
        info.into_boxed_slice()
    }

    #[test]
    fn subtract_reduces_power() {
        let msg = [0u8; 77];
        let itone = message_to_tones(&msg);
        let samples = tones_to_i16(&itone, 1000.0, 20_000);

        let mut audio = vec![0i16; 180_000];
        let offset = 6_000usize;
        let len = samples.len().min(180_000 - offset);
        audio[offset..offset + len].copy_from_slice(&samples[..len]);

        let power_before: f32 =
            audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>() / audio.len() as f32;

        let result = DecodeResult {
            info: info91(msg),
            freq_hz: 1000.0,
            dt_sec: 0.0,
            hard_errors: 0,
            sync_score: 10.0,
            pass: 0,
            sync_cv: 0.0,
            snr_db: 0.0,
        };

        subtract_signal_lpf(&mut audio, &result);

        let power_after: f32 =
            audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>() / audio.len() as f32;

        assert!(
            power_after < power_before * 0.10,
            "power before={power_before:.1} after={power_after:.1}"
        );
    }

    #[test]
    fn subtract_with_exact_timing_near_zero() {
        let msg = [1u8; 77];
        let itone = message_to_tones(&msg);
        let samples = tones_to_i16(&itone, 1000.0, 20_000);

        let mut audio = vec![0i16; 180_000];
        let offset = 6_000usize;
        let len = samples.len().min(180_000 - offset);
        audio[offset..offset + len].copy_from_slice(&samples[..len]);

        let power_before: f32 = audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>();

        let result = DecodeResult {
            info: info91(msg),
            freq_hz: 1000.0,
            dt_sec: 0.0,
            hard_errors: 0,
            sync_score: 10.0,
            pass: 0,
            sync_cv: 0.0,
            snr_db: 0.0,
        };
        subtract_signal_lpf(&mut audio, &result);

        let power_after: f32 = audio.iter().map(|&s| (s as f32).powi(2)).sum::<f32>();
        assert!(
            power_after < power_before * 0.02,
            "power before={power_before:.0} after={power_after:.0}"
        );
    }

    #[test]
    fn subtract_reveals_hidden_signal() {
        use super::super::Ft8;
        use super::super::message::pack77;
        use crate::msg::decode_request::DecodeRequest;

        // Two valid FT8 standard messages (the inner's unpack77 +
        // plausibility gate inside `process_one_candidate_inner`
        // requires real-shape codewords; v0.6.1 host redirect through
        // the inner inherits embedded's strictness).
        let msg_strong = pack77("CQ", "JA1ABC", "PM95").expect("pack77 strong");
        let itone_s = message_to_tones(&msg_strong);
        let strong = tones_to_i16(&itone_s, 1000.0, 20_000);

        let msg_weak = pack77("W1AW", "JA1ABC", "73").expect("pack77 weak");
        let itone_w = message_to_tones(&msg_weak);
        let weak = tones_to_i16(&itone_w, 1500.0, 3_000);

        let mut audio = vec![0i16; 180_000];
        let off = 6_000usize;
        let len = strong.len().min(180_000 - off);
        for i in 0..len {
            let v = strong[i] as i32 + weak[i] as i32;
            audio[off + i] = v.clamp(-32_768, 32_767) as i16;
        }

        let results = DecodeRequest::<Ft8>::new(&audio, 800.0, 1700.0, 1.0, 50)
            .osd(false)
            .strictness(DecodeStrictness::Normal)
            .sic_early()
            .decode()
            .results;
        let found_strong = results.iter().any(|r| r.message77() == msg_strong);
        let found_weak = results.iter().any(|r| r.message77() == msg_weak);
        assert!(found_strong, "strong signal not decoded");
        assert!(found_weak, "weak signal not decoded after subtract");
    }
}
