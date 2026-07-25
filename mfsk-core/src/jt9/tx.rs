//! JT9 transmitter: 72-bit info → 85 channel tones → 12 kHz audio.
//!
//! Mirrors the stages in WSJT-X `gen9.f90`:
//! 1. Convolutional r=½ K=32 encode (72 info + 31 tail → 206 coded bits)
//! 2. Append a padding zero to reach 207 bits
//! 3. Interleave the first 206 bits in place (the padding bit is a
//!    no-op under the 206-bit bit-reversal)
//! 4. Pack into 69 × 3-bit data symbols (MSB-first within each group)
//! 5. Apply Gray coding (`g = n ^ (n >> 1)` on 3-bit values)
//! 6. Add 1 to each data tone and splice the 16 sync symbols
//!    (tone 0 at the sync positions) → 85 tones in the range 0..=8
//! 7. Emit 9-FSK audio at 1.736 Hz tone spacing (plain FSK, no GFSK)

use core::f32::consts::TAU;

use crate::core::{FecCodec, ModulationParams};
use crate::fec::ConvFano232;

use super::interleave::interleave;
use super::sync_pattern::JT9_ISYNC;
use super::{Jt9, Jt9Waveform};

/// Gray-map a 3-bit value: `n ^ (n >> 1)`.
#[inline]
fn gray3(n: u8) -> u8 {
    (n ^ (n >> 1)) & 0x7
}

/// Encode 72 info bits into 85 channel tones (values 0..=8, with the
/// 16 sync positions carrying tone 0 and data symbols carrying
/// `gray(data_bits)+1`).
pub fn encode_channel_symbols(info_bits: &[u8; 72]) -> [u8; 85] {
    let codec = ConvFano232;

    // Step 1–2: convolutional encode to 206 bits; pad to 207.
    let mut cw206 = vec![0u8; 206];
    codec.encode(info_bits, &mut cw206);
    let mut bits207 = [0u8; 207];
    bits207[..206].copy_from_slice(&cw206);
    // bits207[206] = 0 (padding, already zero)

    // Step 3: interleave the first 206 bits.
    let mut interleaved_206 = [0u8; 206];
    interleaved_206.copy_from_slice(&bits207[..206]);
    interleave(&mut interleaved_206);
    bits207[..206].copy_from_slice(&interleaved_206);

    // Step 4–5: pack 3 bits → data symbol, Gray-map.
    let mut data_symbols = [0u8; 69];
    for i in 0..69 {
        let b0 = bits207[3 * i];
        let b1 = bits207[3 * i + 1];
        let b2 = bits207[3 * i + 2];
        let raw = (b0 << 2) | (b1 << 1) | b2;
        data_symbols[i] = gray3(raw);
    }

    // Step 6: splice sync (tone 0) and data (tone = gray+1) into 85 slots.
    let mut tones = [0u8; 85];
    let mut j = 0;
    for (i, slot) in tones.iter_mut().enumerate() {
        if JT9_ISYNC[i] == 1 {
            *slot = 0;
        } else {
            *slot = data_symbols[j] + 1;
            j += 1;
        }
    }
    debug_assert_eq!(j, 69, "sync/data split must fill exactly 69 data symbols");
    tones
}

/// Synthesize JT9 audio: one CPFSK tone per symbol at
/// `base_freq + tone * 1.7361 Hz`. `base_freq` is the frequency of
/// tone 0 (the sync tone, i.e. the low end of the 9-tone set).
pub fn synthesize_audio(
    tones: &[u8; 85],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    let nsps = (sample_rate as f32 * <Jt9 as ModulationParams>::SYMBOL_DT).round() as usize;
    let tone_spacing = <Jt9 as ModulationParams>::TONE_SPACING_HZ;
    let mut out = Vec::with_capacity(nsps * 85);
    let mut phase = 0.0f32;
    for &sym in tones {
        assert!(sym < 9, "JT9 tone must be in 0..=8");
        let freq = base_freq_hz + sym as f32 * tone_spacing;
        let dphi = TAU * freq / sample_rate as f32;
        for _ in 0..nsps {
            out.push(amplitude * phase.cos());
            phase += dphi;
            if phase > TAU {
                phase -= TAU;
            } else if phase < -TAU {
                phase += TAU;
            }
        }
    }
    out
}

/// Synthesize any selectable normal or fast JT9 waveform.
pub fn synthesize_audio_variant(
    tones: &[u8; 85],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
    variant: Jt9Waveform,
) -> Vec<f32> {
    assert!(sample_rate > 0);
    assert!(base_freq_hz.is_finite() && base_freq_hz >= 0.0);
    assert!(amplitude.is_finite() && (0.0..=1.0).contains(&amplitude));
    let symbol_dt = variant.symbol_samples_12k() as f64 / 12_000.0;
    let samples_per_symbol = (sample_rate as f64 * symbol_dt).round() as usize;
    let tone_spacing = variant.tone_spacing_hz();
    let mut out = Vec::with_capacity(samples_per_symbol * tones.len());
    let mut phase = 0.0f32;
    for &tone in tones {
        assert!(tone < 9, "JT9 tone must be in 0..=8");
        let frequency = base_freq_hz + tone as f32 * tone_spacing;
        let step = TAU * frequency / sample_rate as f32;
        for _ in 0..samples_per_symbol {
            out.push(amplitude * phase.cos());
            phase += step;
            if phase >= TAU {
                phase -= TAU;
            }
        }
    }
    out
}

/// Synthesize the complete scheduled transmit interval for a JT9
/// waveform. Normal A-H emit their single 85-symbol frame. Fast E-H
/// repeat that frame phase-continuously and stop 0.5 seconds before the
/// selected 5/10/15/30 second boundary, matching WSJT-X's fast-mode
/// modulator.
pub fn synthesize_audio_variant_period(
    tones: &[u8; 85],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
    variant: Jt9Waveform,
    period_seconds: u32,
) -> Option<Vec<f32>> {
    if !matches!(period_seconds, 5 | 10 | 15 | 30 | 60) {
        return None;
    }
    if matches!(variant, Jt9Waveform::Normal(_)) {
        return (period_seconds == 60).then(|| {
            synthesize_audio_variant(tones, sample_rate, base_freq_hz, amplitude, variant)
        });
    }
    if sample_rate == 0
        || !base_freq_hz.is_finite()
        || base_freq_hz < 0.0
        || !amplitude.is_finite()
        || !(0.0..=1.0).contains(&amplitude)
    {
        return None;
    }
    let symbol_dt = variant.symbol_samples_12k() as f64 / 12_000.0;
    let samples_per_symbol = (sample_rate as f64 * symbol_dt).round() as usize;
    if samples_per_symbol == 0 {
        return None;
    }
    let sample_count = period_seconds as usize * sample_rate as usize - sample_rate as usize / 2;
    let mut output = Vec::with_capacity(sample_count);
    let mut phase = 0.0f32;
    for sample_index in 0..sample_count {
        let symbol = (sample_index / samples_per_symbol) % tones.len();
        let tone = tones[symbol];
        let frequency = base_freq_hz + tone as f32 * variant.tone_spacing_hz();
        phase += TAU * frequency / sample_rate as f32;
        if phase >= TAU {
            phase -= TAU;
        }
        output.push(amplitude * phase.cos());
    }
    Some(output)
}

/// Convenience: pack a standard message via `Jt72` and synthesize.
pub fn synthesize_standard(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let words = crate::msg::jt72::pack_standard(call1, call2, grid_or_report)?;
    // 12 × 6-bit words → 72 MSB-first bits.
    let mut info_bits = [0u8; 72];
    for (i, bit) in info_bits.iter_mut().enumerate() {
        let word = words[i / 6];
        let bit_in_word = 5 - (i % 6);
        *bit = (word >> bit_in_word) & 1;
    }
    let tones = encode_channel_symbols(&info_bits);
    Some(synthesize_audio(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

/// Pack a standard message and synthesize a selected JT9 waveform.
pub fn synthesize_standard_variant(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
    variant: Jt9Waveform,
) -> Option<Vec<f32>> {
    let words = crate::msg::jt72::pack_standard(call1, call2, grid_or_report)?;
    let mut info_bits = [0u8; 72];
    for (index, bit) in info_bits.iter_mut().enumerate() {
        let word = words[index / 6];
        *bit = (word >> (5 - index % 6)) & 1;
    }
    let tones = encode_channel_symbols(&info_bits);
    Some(synthesize_audio_variant(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
        variant,
    ))
}

/// Pack a standard message and synthesize its complete scheduled
/// normal/fast transmit interval.
pub fn synthesize_standard_variant_period(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
    variant: Jt9Waveform,
    period_seconds: u32,
) -> Option<Vec<f32>> {
    let words = crate::msg::jt72::pack_standard(call1, call2, grid_or_report)?;
    let mut info_bits = [0u8; 72];
    for (index, bit) in info_bits.iter_mut().enumerate() {
        *bit = (words[index / 6] >> (5 - index % 6)) & 1;
    }
    let tones = encode_channel_symbols(&info_bits);
    synthesize_audio_variant_period(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
        variant,
        period_seconds,
    )
}

/// Pack a complete JT9 message, including Type 6 free text, and synthesize
/// its complete scheduled normal/fast transmit interval.
pub fn synthesize_message_variant_period(
    message: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
    variant: Jt9Waveform,
    period_seconds: u32,
) -> Option<Vec<f32>> {
    let words = crate::msg::jt72::pack_message(message);
    let mut info_bits = [0u8; 72];
    for (index, bit) in info_bits.iter_mut().enumerate() {
        *bit = (words[index / 6] >> (5 - index % 6)) & 1;
    }
    let tones = encode_channel_symbols(&info_bits);
    synthesize_audio_variant_period(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
        variant,
        period_seconds,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_produces_16_sync_tones() {
        let info = [0u8; 72];
        let tones = encode_channel_symbols(&info);
        let sync_count = tones.iter().filter(|&&t| t == 0).count();
        // At least 16 — data tones are 1..=8 so tone 0 only appears at
        // sync positions. (All-zero info might produce a few accidental
        // tone-0 data symbols — but in practice with Gray mapping and
        // convolutional expansion the count lands on exactly 16.)
        assert!(
            sync_count >= 16,
            "expected >=16 sync tones, got {}",
            sync_count
        );
    }

    #[test]
    fn encode_all_tones_in_range() {
        let info: Vec<u8> = (0..72).map(|i| (i & 1) as u8).collect();
        let mut info72 = [0u8; 72];
        info72.copy_from_slice(&info);
        let tones = encode_channel_symbols(&info72);
        for (i, &t) in tones.iter().enumerate() {
            assert!(t <= 8, "tone at {i} = {t} is out of range");
        }
    }

    #[test]
    fn synthesize_produces_expected_length() {
        let tones = [0u8; 85];
        let audio = synthesize_audio(&tones, 12_000, 1500.0, 0.3);
        assert_eq!(audio.len(), 6912 * 85);
    }

    #[test]
    fn synthesize_standard_message_ok() {
        let audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3).expect("pack + synth");
        assert_eq!(audio.len(), 6912 * 85);
    }

    #[test]
    fn free_text_roundtrips_through_jt9() {
        let variant = Jt9Waveform::Normal(super::super::Jt9Submode::A);
        let audio =
            synthesize_message_variant_period("TNX JOE -14 73", 12_000, 1_500.0, 0.3, variant, 60)
                .expect("free-text synth");
        let decoded = super::super::decode_at_variant(&audio, 12_000, 0, 1_500.0, variant)
            .expect("decode free-text synth");
        assert_eq!(decoded.to_string(), "TNX JOE -14 7");
    }

    #[test]
    fn fast_period_repeats_until_the_wsjt_x_stop_margin() {
        let tones = encode_channel_symbols(&[0; 72]);
        for period in [5, 10, 15, 30] {
            let audio = synthesize_audio_variant_period(
                &tones,
                12_000,
                1_500.0,
                0.3,
                Jt9Waveform::Fast(super::super::Jt9Submode::H),
                period,
            )
            .expect("fast period");
            assert_eq!(audio.len(), period as usize * 12_000 - 6_000,);
        }
    }

    #[test]
    fn every_normal_and_fast_geometry_has_exact_length() {
        let tones = [0u8; 85];
        for submode in super::super::Jt9Submode::ALL {
            let variant = Jt9Waveform::Normal(submode);
            assert_eq!(
                synthesize_audio_variant(&tones, 12_000, 1_500.0, 0.3, variant).len(),
                variant.symbol_samples_12k() as usize * 85,
            );
        }
        for submode in super::super::Jt9Submode::FAST {
            let variant = Jt9Waveform::fast(submode).expect("E-H");
            assert_eq!(
                synthesize_audio_variant(&tones, 12_000, 1_500.0, 0.3, variant).len(),
                variant.symbol_samples_12k() as usize * 85,
            );
        }
    }
}
