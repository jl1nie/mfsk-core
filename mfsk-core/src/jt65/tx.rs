//! JT65 transmitter: 72-bit message → 126 channel tones → audio.
//!
//! Mirrors the stages in WSJT-X `jt65sim.f90` lines 172–190:
//! 1. Pack the message into 12 × 6-bit symbols (`Jt72` words).
//! 2. Encode with RS(63, 12) using the JT65 byte ordering
//!    (`Rs63_12::encode_jt65`) → 63 codeword symbols.
//! 3. Interleave (7×9 transpose).
//! 4. Gray-code each 6-bit symbol.
//! 5. Splice into the 126-slot frame: sync positions emit tone 0,
//!    data positions emit `gray(sent[k]) + 2`.
//! 6. Emit CPFSK audio at the JT65A baud (≈ 2.69 Hz tone spacing).

use core::f32::consts::TAU;

use crate::core::ModulationParams;
use crate::fec::Rs63_12;

use super::Jt65;
use super::gray::gray6;
use super::interleave::interleave;
use super::sync_pattern::JT65_NPRC;

/// Encode a 12-symbol info payload into 126 channel tones
/// (values 0 or 2..=65 where 0 = sync, 2..=65 = data + 2).
pub fn encode_channel_symbols(info: &[u8; 12]) -> [u8; 126] {
    let rs = Rs63_12::new();
    let mut sent = rs.encode_jt65(info);
    interleave(&mut sent);
    for s in sent.iter_mut() {
        *s = gray6(*s);
    }
    let mut tones = [0u8; 126];
    let mut k = 0usize;
    for i in 0..126 {
        if JT65_NPRC[i] == 1 {
            tones[i] = 0; // sync
        } else {
            // Data tones are the 64 Gray-coded values, offset by +2
            // (WSJT-X `jt65sim.f90` line 186: itone(j)=sent(k)+2).
            tones[i] = sent[k] + 2;
            k += 1;
        }
    }
    debug_assert_eq!(k, 63, "data positions must total 63");
    tones
}

/// Synthesize JT65A audio: one CPFSK tone per symbol at
/// `base_freq + tone * 2.69 Hz`. `base_freq` is the frequency of
/// tone 0 (the sync tone).
pub fn synthesize_audio(
    tones: &[u8; 126],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let tone_spacing = <Jt65 as ModulationParams>::TONE_SPACING_HZ;
    let mut out = Vec::with_capacity(nsps * 126);
    let mut phase = 0.0f32;
    for &sym in tones {
        assert!(sym <= 65, "JT65 tone must be in 0..=65");
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
    let tones = encode_channel_symbols(&words);
    Some(synthesize_audio(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_splits_63_sync_63_data() {
        let info = [0u8; 12];
        let tones = encode_channel_symbols(&info);
        let sync_count = tones.iter().filter(|&&t| t == 0).count();
        assert_eq!(sync_count, 63, "expected exactly 63 sync tones");
        let data_count = tones.iter().filter(|&&t| (2..=65).contains(&t)).count();
        assert_eq!(data_count, 63, "expected exactly 63 data tones");
    }

    #[test]
    fn synthesize_produces_expected_length() {
        let tones = [0u8; 126];
        let audio = synthesize_audio(&tones, 12_000, 1270.0, 0.3);
        assert_eq!(audio.len(), 4460 * 126);
    }

    #[test]
    fn synthesize_standard_message_ok() {
        let audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3).expect("pack + synth");
        assert_eq!(audio.len(), 4460 * 126);
    }
}
