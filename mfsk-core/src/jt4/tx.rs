//! JT4 transmitter: 72 information bits to 206 interleaved-sync 4-FSK
//! symbols and phase-continuous audio.

use alloc::vec::Vec;
use core::f32::consts::TAU;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::core::FecCodec;
use crate::fec::ConvFano232Jt4;

use super::interleave::interleave;
use super::{JT4_BAUD, JT4_SYNC_VECTOR, Jt4Submode, SyncPolarity};

/// Encode one JT4 frame exactly as WSJT-X `encode4` + `gen4`.
pub fn encode_channel_symbols(info_bits: &[u8; 72], polarity: SyncPolarity) -> [u8; 206] {
    let mut coded = [0u8; 206];
    ConvFano232Jt4.encode(info_bits, &mut coded);
    interleave(&mut coded);

    core::array::from_fn(|index| {
        let sync = match polarity {
            SyncPolarity::Normal => JT4_SYNC_VECTOR[index],
            SyncPolarity::Inverted => 1 - JT4_SYNC_VECTOR[index],
        };
        2 * coded[index] + sync
    })
}

/// Exact output length at `sample_rate`.
///
/// JT4's 4.375 baud symbol duration is fractional at 12 kHz. The frame
/// length is rounded once, rather than rounding every symbol independently.
pub fn synthesize_audio_len(sample_rate: u32) -> usize {
    (206.0 * sample_rate as f64 / JT4_BAUD as f64).round() as usize
}

/// Synthesize phase-continuous JT4 audio while retaining the fractional
/// symbol boundaries used by WSJT-X `jt4sim.f90`.
pub fn synthesize_audio(
    symbols: &[u8; 206],
    submode: Jt4Submode,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    assert!(sample_rate > 0);
    assert!(base_freq_hz.is_finite() && base_freq_hz >= 0.0);
    assert!(amplitude.is_finite() && (0.0..=1.0).contains(&amplitude));

    let length = synthesize_audio_len(sample_rate);
    let samples_per_symbol = sample_rate as f64 / JT4_BAUD as f64;
    let spacing = JT4_BAUD * submode.spacing_multiplier() as f32;
    let mut output = Vec::with_capacity(length);
    let mut phase = 0.0f32;

    for sample_index in 0..length {
        let symbol_index = ((sample_index as f64 / samples_per_symbol).floor() as usize).min(205);
        let tone = symbols[symbol_index];
        assert!(tone < 4, "JT4 channel symbol must be in 0..=3");
        let frequency = base_freq_hz + tone as f32 * spacing;
        phase += TAU * frequency / sample_rate as f32;
        if phase >= TAU {
            phase -= TAU;
        }
        output.push(amplitude * phase.sin());
    }
    output
}

/// Pack and synthesize a standard JT72 message.
pub fn synthesize_standard(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    submode: Jt4Submode,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let words = crate::msg::jt72::pack_standard(call1, call2, grid_or_report)?;
    let mut info = [0u8; 72];
    for index in 0..72 {
        info[index] = (words[index / 6] >> (5 - index % 6)) & 1;
    }
    let polarity = SyncPolarity::for_standard_report(grid_or_report);
    let symbols = encode_channel_symbols(&info, polarity);
    Some(synthesize_audio(
        &symbols,
        submode,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

/// Pack and synthesize a complete JT4 message, including Type 6 free text.
pub fn synthesize_message(
    message: &str,
    submode: Jt4Submode,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    let normalized = message
        .split_ascii_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_ascii_uppercase();
    let words = crate::msg::jt72::pack_message(&normalized);
    let mut info = [0u8; 72];
    for index in 0..72 {
        info[index] = (words[index / 6] >> (5 - index % 6)) & 1;
    }
    let report = normalized
        .split_ascii_whitespace()
        .nth(2)
        .unwrap_or_default();
    let symbols = encode_channel_symbols(&info, SyncPolarity::for_standard_report(report));
    synthesize_audio(&symbols, submode, sample_rate, base_freq_hz, amplitude)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_lsb_and_fractional_length_match_reference_geometry() {
        let symbols = encode_channel_symbols(&[0u8; 72], SyncPolarity::Normal);
        for (index, symbol) in symbols.iter().enumerate() {
            assert_eq!(symbol & 1, JT4_SYNC_VECTOR[index]);
            assert!(*symbol < 4);
        }
        assert_eq!(synthesize_audio_len(12_000), 565_029);
    }

    #[test]
    fn free_text_roundtrips_through_jt4() {
        let audio = synthesize_message("TNX JOE -14 73", Jt4Submode::A, 12_000, 1_000.0, 0.35);
        let decoded = crate::jt4::rx::decode_aligned(&audio, 12_000, 0, 1_000.0, Jt4Submode::A)
            .expect("decode free-text synth");
        assert_eq!(decoded.message.to_string(), "TNX JOE -14 7");
    }
}
