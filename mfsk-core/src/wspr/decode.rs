//! Top-level WSPR decode entry point.
//!
//! Given aligned audio, a candidate base frequency, and a target start
//! sample, runs demod → deinterleave → Fano → message unpack. No coarse
//! search here; a later module will wrap this with a (freq × time) scan.

use crate::msg::WsprMessage;

use super::search::{SearchParams, coarse_search};
use super::{decode_from_deinterleaved_llrs, demodulate_aligned};

/// One successful WSPR decode.
#[derive(Clone, Debug)]
pub struct WsprDecode {
    /// Recovered message payload.
    pub message: WsprMessage,
    /// Base frequency (tone 0) used for demodulation.
    pub freq_hz: f32,
    /// Sample index at which symbol 0 started.
    pub start_sample: usize,
}

/// Decode one WSPR frame at a known (freq, start_sample). Returns `None`
/// if the Fano decoder fails to converge or the message doesn't unpack.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
) -> Option<WsprDecode> {
    let mut llrs = demodulate_aligned(audio, sample_rate, start_sample, freq_hz);
    deinterleave_llrs(&mut llrs);
    let message = decode_from_deinterleaved_llrs(&llrs)?;
    Some(WsprDecode {
        message,
        freq_hz,
        start_sample,
    })
}

/// Scan an audio buffer for any number of WSPR frames, returning all
/// successful decodes. Runs a coarse (freq, time) search with the given
/// [`SearchParams`], then attempts [`decode_at`] on each candidate in
/// score-descending order. Duplicate decodes (same message within ±5 Hz
/// and ±1 symbol) are collapsed to the single earliest-candidate hit,
/// so each transmission appears at most once in the output.
pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprDecode> {
    let cands = coarse_search(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<WsprDecode> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192; // one WSPR symbol at 12 kHz
    for c in cands {
        let Some(d) = decode_at(audio, sample_rate, c.start_sample, c.freq_hz) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= TIME_DEDUP_SAMPLES
        });
        if !dup {
            seen.push(d);
        }
    }
    seen
}

/// Convenience: scan using [`SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<WsprDecode> {
    decode_scan(audio, sample_rate, 0, &SearchParams::default())
}

/// Deinterleave 162 LLRs in place (same permutation as [`deinterleave`]
/// but for `f32` values).
fn deinterleave_llrs(llrs: &mut [f32; 162]) {
    let mut tmp = [0f32; 162];
    let mut p = 0u8;
    let mut i = 0u8;
    while p < 162 {
        // Inline the bit-reverse-8 to avoid exposing a pub helper.
        let i64 = i as u64;
        let j = ((((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64))
            >> 32) as u8 as usize;
        if j < 162 {
            tmp[p as usize] = llrs[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *llrs = tmp;
}

#[cfg(test)]
mod tests {
    use super::super::search::SearchParams;
    use super::super::synthesize_type1;
    use super::*;
    use crate::msg::WsprMessage;

    #[test]
    fn synth_decode_roundtrip_k1abc_fn42_37() {
        let freq = 1500.0;
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("valid message");
        let r = decode_at(&audio, 12_000, 0, freq).expect("decode");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn scan_recovers_message_without_freq_hint() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let decodes = decode_scan(
            &audio,
            12_000,
            0,
            &SearchParams {
                freq_min_hz: 1450.0,
                freq_max_hz: 1550.0,
                ..SearchParams::default()
            },
        );
        assert!(!decodes.is_empty(), "at least one decode");
        let d = decodes.into_iter().next().unwrap();
        assert_eq!(
            d.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
        assert!((d.freq_hz - 1500.0).abs() <= 2.0);
    }

    #[test]
    fn survives_moderate_awgn() {
        use std::f32::consts::PI;

        let freq = 1500.0;
        let mut audio =
            synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.5).expect("valid message");

        // Deterministic "noise": superposition of a handful of off-tone
        // sinusoids plus a pseudorandom dither. This is a cheap AWGN
        // stand-in that keeps the test free of rand dependencies.
        let mut seed: u32 = 0x1234_5678;
        for (i, s) in audio.iter_mut().enumerate() {
            // Linear congruential pseudorandom for reproducible noise.
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let rnd = ((seed >> 16) as f32 / 32768.0 - 1.0) * 0.10;
            let off = 0.05 * (2.0 * PI * 2345.7 * i as f32 / 12_000.0).sin();
            *s += rnd + off;
        }

        let r = decode_at(&audio, 12_000, 0, freq).expect("decode under noise");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K9AN".into(),
                grid: "EN50".into(),
                power_dbm: 33,
            }
        );
    }
}
