//! FT4 encode: 77-bit message → 103-symbol tone sequence → 12 kHz PCM.
//!
//! Mirrors `ft8-core::wave_gen` but driven by the [`Ft4`] trait impl, so all
//! modulation parameters (tone spacing, samples/symbol, BT) come from
//! compile-time constants.

use alloc::vec::Vec;

use super::Ft4;
use crate::engine::dsp::gfsk::{GfskCfg, synth_f32, synth_f32_into, synth_i16, synth_i16_into};
use crate::engine::{FecCodec, FrameLayout, ModulationParams};
use crate::fec::Ldpc174_91;

/// FT4 GFSK configuration: 12 kHz, 576 samples/symbol, BT=1.0, hmod=1.0,
/// 72-sample (NSPS/8) cosine ramp. BT=1.0 matches WSJT-X
/// `lib/ft4/gen_ft4wave.f90` (`gfsk_pulse(1.0, tt)`).
pub const FT4_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 576,
    bt: 1.0,
    hmod: 1.0,
    ramp_samples: 576 / 8,
};

/// Append CRC-14 to the 77-bit message, producing 91 info bits.
fn append_crc14(message77: &[u8; 77]) -> [u8; 91] {
    let mut bytes = [0u8; 12];
    for (i, &bit) in message77.iter().enumerate() {
        bytes[i / 8] |= (bit & 1) << (7 - i % 8);
    }
    let crc = crate::fec::ldpc::crc14(&bytes);
    let mut info = [0u8; 91];
    info[..77].copy_from_slice(message77);
    for i in 0..14 {
        info[77 + i] = ((crc >> (13 - i)) & 1) as u8;
    }
    info
}

/// Encode a 77-bit message into the 103-symbol FT4 tone sequence.
///
/// XORs the input with [`super::FT4_RVEC`] before CRC + LDPC, matching
/// WSJT-X `genft4.f90:64`. The CRC-14 is then computed over the
/// **scrambled** message bits — same as WSJT-X, so the receive-side
/// CRC verification stays correct.
pub fn message_to_tones(message77: &[u8; 77]) -> Vec<u8> {
    let mut scrambled = *message77;
    for (b, &r) in scrambled.iter_mut().zip(super::FT4_RVEC.iter()) {
        *b = (*b ^ r) & 1;
    }
    let info = append_crc14(&scrambled);
    let codec = Ldpc174_91;
    let mut cw = [0u8; 174];
    codec.encode(&info, &mut cw);
    crate::engine::tx::codeword_to_itone::<Ft4>(&cw)
}

/// Output sample count for FT4 waveform synthesis (103 × 576 = 59 328).
pub const TONES_OUTPUT_LEN: usize = (<Ft4 as FrameLayout>::N_SYMBOLS as usize) * 576;

/// Synthesise into a caller-provided f32 PCM buffer. **No allocation
/// of the output**; `out.len()` must equal [`TONES_OUTPUT_LEN`].
pub fn tones_to_f32_into(out: &mut [f32], itone: &[u8], f0: f32, amplitude: f32) {
    debug_assert_eq!(itone.len(), <Ft4 as FrameLayout>::N_SYMBOLS as usize);
    synth_f32_into(out, itone, f0, amplitude, &FT4_GFSK)
}

/// Synthesise a 12 kHz f32 PCM waveform from an FT4 tone sequence.
/// Vec-returning convenience wrapper for [`tones_to_f32_into`]. Output
/// length is [`TONES_OUTPUT_LEN`] (= 103 × 576 = 59 328) samples.
pub fn tones_to_f32(itone: &[u8], f0: f32, amplitude: f32) -> Vec<f32> {
    debug_assert_eq!(itone.len(), <Ft4 as FrameLayout>::N_SYMBOLS as usize);
    synth_f32(itone, f0, amplitude, &FT4_GFSK)
}

/// Synthesise into a caller-provided i16 PCM buffer. Peak equals
/// `amplitude_i16`; `out.len()` must equal [`TONES_OUTPUT_LEN`].
pub fn tones_to_i16_into(out: &mut [i16], itone: &[u8], f0: f32, amplitude_i16: i16) {
    debug_assert_eq!(itone.len(), <Ft4 as FrameLayout>::N_SYMBOLS as usize);
    synth_i16_into(out, itone, f0, amplitude_i16, &FT4_GFSK)
}

/// Synthesise a 16-bit PCM waveform. Peak equals `amplitude_i16`.
pub fn tones_to_i16(itone: &[u8], f0: f32, amplitude_i16: i16) -> Vec<i16> {
    debug_assert_eq!(itone.len(), <Ft4 as FrameLayout>::N_SYMBOLS as usize);
    synth_i16(itone, f0, amplitude_i16, &FT4_GFSK)
}

// Quiet rust about the unused trait import in release builds that strip debug_assert.
fn _silence() {
    let _ = <Ft4 as ModulationParams>::NTONES;
}
