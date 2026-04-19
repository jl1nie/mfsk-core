//! WSPR transmitter path: channel symbols → audio samples.
//!
//! Pragmatic first-pass synthesiser for end-to-end decoder tests. Each
//! symbol emits one continuous-phase sinusoid at
//! `base_freq + symbol * tone_spacing` for `NSPS / sample_rate` seconds.
//! No GFSK shaping yet — plain CPFSK is close enough in the narrowband
//! limit that WSPR's FFT-based demod sees the same spectral peaks.
//!
//! A real over-the-air WSPR transmitter applies a raised-cosine pulse to
//! smooth symbol transitions; adding that here is straightforward follow
//! up (pre-compute a pulse table, convolve symbol-boundary regions) but
//! not required for the decode-roundtrip tests this module enables.

use core::f32::consts::TAU;

use crate::core::ModulationParams;

use super::Wspr;

/// Synthesize a WSPR transmission as mono `f32` audio samples.
///
/// `symbols` must be 162 values in `0..=3`. `base_freq_hz` is the
/// frequency of tone 0; the remaining tones sit at
/// `base_freq_hz + tone * WSPR::TONE_SPACING_HZ`. Phase is continuous
/// across symbol boundaries so the receiver's FFT window can land on
/// any 683 ms stretch without picking up transient spectral spread.
pub fn synthesize_audio(
    symbols: &[u8; 162],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    // NSPS scales by the sample rate — the trait constant is for 12 kHz.
    let nsps = (sample_rate as f32 * <Wspr as ModulationParams>::SYMBOL_DT).round() as usize;
    let tone_spacing = <Wspr as ModulationParams>::TONE_SPACING_HZ;
    let mut out = Vec::with_capacity(nsps * 162);
    let mut phase = 0.0f32;
    for &sym in symbols {
        assert!(sym < 4, "WSPR channel symbol must be in 0..=3");
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

/// Convenience wrapper that packs a message and synthesises in one step.
/// Returns `None` if the message can't fit the Type 1 layout.
pub fn synthesize_type1(
    callsign: &str,
    grid: &str,
    power_dbm: i32,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let info = crate::msg::wspr::pack_type1(callsign, grid, power_dbm)?;
    let symbols = super::encode_channel_symbols(&info);
    Some(synthesize_audio(&symbols, sample_rate, base_freq_hz, amplitude))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthesizes_162_symbol_buffer_at_12k() {
        let symbols = [0u8; 162];
        let audio = synthesize_audio(&symbols, 12_000, 1500.0, 0.5);
        // 8192 samples/symbol × 162 symbols = 1_327_104 samples
        assert_eq!(audio.len(), 8192 * 162);
    }

    #[test]
    fn synthesizes_valid_message() {
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
            .expect("valid message");
        assert_eq!(audio.len(), 8192 * 162);
        // Basic sanity: peak amplitude close to the requested level.
        let peak = audio.iter().cloned().fold(0.0f32, f32::max);
        assert!(peak > 0.28 && peak < 0.32, "peak amplitude out of range: {}", peak);
    }
}
