//! Aligned JT4 demodulation and decode.
//!
//! The receiver correlates the two data hypotheses allowed by each known
//! sync bit. It uses rounded cumulative boundaries, so the 4.375 baud
//! symbol clock has no cumulative rounding drift at a 12 kHz input rate.

use core::f32::consts::TAU;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::core::{DecodeContext, FecCodec, FecOpts, MessageCodec};
use crate::fec::ConvFano232Jt4;
use crate::msg::{Jt72Codec, Jt72Message};

use super::interleave::deinterleave_llrs;
use super::{JT4_BAUD, JT4_SYNC_VECTOR, Jt4Submode, SyncPolarity};

/// Successful JT4 decode at a caller-provided time/frequency alignment.
#[derive(Clone, Debug, PartialEq)]
pub struct Jt4Decode {
    pub message: Jt72Message,
    pub hard_errors: u32,
    pub polarity: SyncPolarity,
    pub freq_hz: f32,
    pub start_sample: usize,
}

fn correlation_power(
    audio: &[f32],
    sample_rate: u32,
    start: usize,
    end: usize,
    frequency_hz: f32,
) -> f32 {
    if start >= audio.len() {
        return 0.0;
    }
    let step = TAU * frequency_hz / sample_rate as f32;
    let step_cos = step.cos();
    let step_sin = step.sin();
    let mut oscillator_cos = 1.0f32;
    let mut oscillator_sin = 0.0f32;
    let mut real = 0.0f32;
    let mut imag = 0.0f32;
    // WSJT-X keeps demodulating the final symbol when a recording ends a
    // little early and treats unavailable samples as zero. Several pinned
    // JT4 reference WAVs are shorter than the nominal 206-symbol frame by a
    // fraction of a symbol, so requiring a complete trailing slice rejects
    // an otherwise valid transmission.
    for &sample in &audio[start..end.min(audio.len())] {
        real += sample * oscillator_cos;
        imag -= sample * oscillator_sin;
        let next_cos = oscillator_cos * step_cos - oscillator_sin * step_sin;
        oscillator_sin = oscillator_sin * step_cos + oscillator_cos * step_sin;
        oscillator_cos = next_cos;
    }
    real * real + imag * imag
}

fn demodulate_aligned_chips(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    submode: Jt4Submode,
    polarity: SyncPolarity,
    chip_count: usize,
) -> Option<[f32; 206]> {
    if sample_rate == 0 || !base_freq_hz.is_finite() || base_freq_hz < 0.0 || chip_count == 0 {
        return None;
    }
    let samples_per_symbol = sample_rate as f64 / JT4_BAUD as f64;
    if start_sample >= audio.len() {
        return None;
    }

    let spacing = JT4_BAUD * submode.spacing_multiplier() as f32;
    let mut channel_llrs = [0f32; 206];
    for symbol_index in 0..206 {
        let symbol_begin =
            start_sample + (symbol_index as f64 * samples_per_symbol).round() as usize;
        let symbol_end =
            start_sample + ((symbol_index + 1) as f64 * samples_per_symbol).round() as usize;
        let normal_sync = JT4_SYNC_VECTOR[symbol_index];
        let sync = match polarity {
            SyncPolarity::Normal => normal_sync,
            SyncPolarity::Inverted => 1 - normal_sync,
        };
        let frequency0 = base_freq_hz + sync as f32 * spacing;
        let frequency1 = base_freq_hz + (2 + sync) as f32 * spacing;
        let symbol_samples = symbol_end - symbol_begin;
        let mut power0 = 0.0f32;
        let mut power1 = 0.0f32;
        for chip in 0..chip_count {
            let begin = symbol_begin
                + ((chip as f64 * symbol_samples as f64 / chip_count as f64).round() as usize);
            let end = symbol_begin
                + (((chip + 1) as f64 * symbol_samples as f64 / chip_count as f64).round()
                    as usize);
            if end <= begin {
                return None;
            }
            power0 += correlation_power(audio, sample_rate, begin, end, frequency0);
            power1 += correlation_power(audio, sample_rate, begin, end, frequency1);
        }
        // Keep the unnormalised energy difference here. WSJT-X
        // `extract4.f90` normalises the complete 206-symbol vector,
        // rather than normalising each symbol independently.
        channel_llrs[symbol_index] = power0 - power1;
    }
    let mean = channel_llrs.iter().sum::<f32>() / channel_llrs.len() as f32;
    let variance = channel_llrs
        .iter()
        .map(|value| {
            let centred = *value - mean;
            centred * centred
        })
        .sum::<f32>()
        / (channel_llrs.len() - 1) as f32;
    let rms = variance.sqrt();
    if !rms.is_finite() || rms <= f32::EPSILON {
        return None;
    }
    for soft in &mut channel_llrs {
        // mfsk-core LLR convention is positive => bit 0. The scale and
        // ±127 saturation are the `amp=30` quantiser in extract4.
        *soft = (30.0 * (*soft - mean) / rms).clamp(-127.0, 127.0);
    }
    deinterleave_llrs(&mut channel_llrs);
    Some(channel_llrs)
}

/// Produce deinterleaved convolutional-code LLRs for one exact alignment.
///
/// This is the coherent one-chip width. [`decode_aligned`] additionally tries
/// the non-coherent chip widths used by WSJT-X for Doppler-spread signals.
pub fn demodulate_aligned(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    submode: Jt4Submode,
    polarity: SyncPolarity,
) -> Option<[f32; 206]> {
    demodulate_aligned_chips(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        submode,
        polarity,
        1,
    )
}

/// Decode one exact alignment. Both WSJT-X sync polarities are tried.
pub fn decode_aligned(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    submode: Jt4Submode,
) -> Option<Jt4Decode> {
    const CHIP_COUNTS: [usize; 7] = [1, 2, 4, 9, 18, 36, 70];
    let maximum_chips = submode.spacing_multiplier() as usize;
    for chip_count in CHIP_COUNTS
        .into_iter()
        .filter(|count| *count <= maximum_chips)
    {
        for polarity in [SyncPolarity::Normal, SyncPolarity::Inverted] {
            let llrs = demodulate_aligned_chips(
                audio,
                sample_rate,
                start_sample,
                base_freq_hz,
                submode,
                polarity,
                chip_count,
            )?;
            let fec = ConvFano232Jt4.decode_soft(&llrs, &FecOpts::default());
            let Some(fec) = fec else {
                continue;
            };
            let mut payload = [0u8; 72];
            payload.copy_from_slice(&fec.info);
            let Some(message) = Jt72Codec::default().unpack(&payload, &DecodeContext::default())
            else {
                continue;
            };
            return Some(Jt4Decode {
                message,
                hard_errors: fec.hard_errors,
                polarity,
                freq_hz: base_freq_hz,
                start_sample,
            });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jt4::tx::synthesize_standard;

    #[test]
    fn every_submode_clean_audio_round_trips() {
        for submode in Jt4Submode::ALL {
            let audio = synthesize_standard("CQ", "K1ABC", "FN42", submode, 12_000, 1_000.0, 0.35)
                .expect("pack");
            let decoded = decode_aligned(&audio, 12_000, 0, 1_000.0, submode)
                .expect("clean waveform must decode");
            assert_eq!(decoded.message.to_string(), "CQ K1ABC FN42");
            assert_eq!(decoded.polarity, SyncPolarity::Normal);
        }
    }

    #[test]
    fn inverted_report_sync_round_trips() {
        let audio = synthesize_standard(
            "K1ABC",
            "W9XYZ",
            "-23",
            Jt4Submode::A,
            12_000,
            1_500.0,
            0.35,
        )
        .expect("pack");
        let decoded = decode_aligned(&audio, 12_000, 0, 1_500.0, Jt4Submode::A)
            .expect("inverted waveform must decode");
        assert_eq!(decoded.message.to_string(), "K1ABC W9XYZ -23");
        assert_eq!(decoded.polarity, SyncPolarity::Inverted);
    }
}
