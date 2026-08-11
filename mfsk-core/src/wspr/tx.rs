//! WSPR transmitter path: channel symbols → audio samples.
//!
//! Pragmatic first-pass synthesiser for end-to-end decoder tests. Each
//! symbol emits one continuous-phase sinusoid at
//! `base_freq + symbol * tone_spacing` for `NSPS / sample_rate` seconds.
//! **No GFSK symbol shaping, deliberately** — WSJT-X does not shape
//! WSPR either. `mainwindow.cpp` passes a *positive* `toneSpacing` for
//! WSPR, selecting `Modulator::modulate`'s plain-CPFSK branch rather
//! than the `toneSpacing < 0` "pre-computed, filtered waveform" branch
//! that FT8/FT4/FST4 use. Adding a raised-cosine frequency pulse here
//! would measurably lower close-in sidelobes (issue #259 measured
//! −53.8 → −85.9 dBc at +25 Hz for a T/8 pulse) but would emit a
//! different waveform than the reference implementation. See
//! `engine::dsp::envelope`'s module doc for the full comparison.
//!
//! The burst envelope *is* ramped — see [`synthesize_audio_into`].

use alloc::vec::Vec;
use core::f32::consts::TAU;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::ModulationParams;
use crate::engine::dsp::envelope;

use super::Wspr;

/// Output sample count for [`synthesize_audio`] /
/// [`synthesize_audio_into`] at the given sample rate. Embedded callers
/// allocate (or claim from a pool) `synthesize_audio_len(sample_rate)`
/// samples and pass them as `out`.
#[inline]
pub fn synthesize_audio_len(sample_rate: u32) -> usize {
    let nsps = (sample_rate as f32 * <Wspr as ModulationParams>::SYMBOL_DT).round() as usize;
    nsps * 162
}

/// Synthesize a WSPR transmission into a caller-provided `f32` buffer.
/// **No allocation** — `out` must be sized to
/// [`synthesize_audio_len`]`(sample_rate)`.
///
/// `symbols` must be 162 values in `0..=3`. `base_freq_hz` is the
/// frequency of tone 0; the remaining tones sit at
/// `base_freq_hz + tone * WSPR::TONE_SPACING_HZ`. Phase is continuous
/// across symbol boundaries so the receiver's FFT window can land on
/// any 683 ms stretch without picking up transient spectral spread.
///
/// # Panics
///
/// Panics if `out.len() != synthesize_audio_len(sample_rate)` or if any
/// symbol is `>= 4`.
pub fn synthesize_audio_into(
    out: &mut [f32],
    symbols: &[u8; 162],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) {
    // NSPS scales by the sample rate — the trait constant is for 12 kHz.
    let nsps = (sample_rate as f32 * <Wspr as ModulationParams>::SYMBOL_DT).round() as usize;
    assert_eq!(
        out.len(),
        nsps * 162,
        "synthesize_audio_into: out.len() must equal synthesize_audio_len()"
    );
    let tone_spacing = <Wspr as ModulationParams>::TONE_SPACING_HZ;
    let mut phase = 0.0f32;
    let mut idx = 0usize;
    for &sym in symbols {
        assert!(sym < 4, "WSPR channel symbol must be in 0..=3");
        let freq = base_freq_hz + sym as f32 * tone_spacing;
        let dphi = TAU * freq / sample_rate as f32;
        for _ in 0..nsps {
            out[idx] = amplitude * phase.cos();
            idx += 1;
            phase += dphi;
            if phase > TAU {
                phase -= TAU;
            } else if phase < -TAU {
                phase += TAU;
            }
        }
    }

    // Transmit-envelope ramp (issue #259). Without it the burst starts
    // and ends on a step discontinuity — a broadband click at both
    // edges, independent of symbol-transition shaping. See
    // `engine::dsp::envelope` for why WSPR gets an envelope ramp but
    // deliberately no GFSK symbol shaping.
    crate::engine::dsp::envelope::apply_ramp(out, envelope::ramp_samples(sample_rate, nsps));
}

/// Synthesize a WSPR transmission as mono `f32` audio samples.
/// Vec-returning convenience wrapper for [`synthesize_audio_into`].
#[inline]
pub fn synthesize_audio(
    symbols: &[u8; 162],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    let mut out = alloc::vec![0.0f32; synthesize_audio_len(sample_rate)];
    synthesize_audio_into(&mut out, symbols, sample_rate, base_freq_hz, amplitude);
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
    Some(synthesize_audio(
        &symbols,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
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
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3).expect("valid message");
        assert_eq!(audio.len(), 8192 * 162);
        // Basic sanity: peak amplitude close to the requested level.
        let peak = audio.iter().cloned().fold(0.0f32, f32::max);
        assert!(
            peak > 0.28 && peak < 0.32,
            "peak amplitude out of range: {}",
            peak
        );
    }
}
