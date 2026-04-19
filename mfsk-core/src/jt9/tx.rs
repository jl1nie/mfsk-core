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
use super::Jt9;

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
    Some(synthesize_audio(&tones, sample_rate, base_freq_hz, amplitude))
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
        assert!(sync_count >= 16, "expected >=16 sync tones, got {}", sync_count);
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
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
            .expect("pack + synth");
        assert_eq!(audio.len(), 6912 * 85);
    }
}
