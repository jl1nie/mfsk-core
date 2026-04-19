//! FST4 encode: 77-bit message → 160-symbol tone sequence → 12 kHz PCM.
//!
//! Mirrors `ft4-core::encode` but for the [`Fst4s60`] geometry:
//! LDPC(240, 101) + CRC-24 over the shared 77-bit WSJT payload,
//! GFSK with BT = 1.0 and 320 ms symbols.

use super::Fst4s60;
use crate::core::dsp::gfsk::{GfskCfg, synth_f32, synth_i16};
use crate::core::{FecCodec, FrameLayout, ModulationParams};
use crate::fec::Ldpc240_101;

/// FST4-60A GFSK configuration: 12 kHz, 3840 samples/symbol, BT=1.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_60A_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 3840,
    bt: 1.0,
    hmod: 1.0,
    ramp_samples: 3840 / 8,
};

/// Append the 24-bit CRC used by FST4 (see `mfsk_core::fec::ldpc240_101::crc24`)
/// to a 77-bit message, producing 101 info bits.
fn append_crc24(message77: &[u8; 77]) -> [u8; 101] {
    let mut info = [0u8; 101];
    info[..77].copy_from_slice(message77);
    // CRC over the 101-bit word with CRC slot zeroed — matches
    // WSJT-X `get_crc24` convention (same scheme as check_crc24).
    let mut with_zero = [0u8; 101];
    with_zero[..77].copy_from_slice(message77);
    let crc = crate::fec::ldpc240_101::crc24(&with_zero);
    for i in 0..24 {
        info[77 + i] = ((crc >> (23 - i)) & 1) as u8;
    }
    info
}

/// Encode a 77-bit message into the 160-symbol FST4-60A tone sequence.
pub fn message_to_tones(message77: &[u8; 77]) -> Vec<u8> {
    let info = append_crc24(message77);
    let codec = Ldpc240_101;
    let mut cw = [0u8; 240];
    codec.encode(&info, &mut cw);
    crate::core::tx::codeword_to_itone::<Fst4s60>(&cw)
}

/// Synthesise a 12 kHz f32 PCM waveform from an FST4 tone sequence.
/// Output length is `N_SYMBOLS × NSPS = 160 × 3840 = 614 400` samples
/// (~51.2 s).
pub fn tones_to_f32(itone: &[u8], f0: f32, amplitude: f32) -> Vec<f32> {
    debug_assert_eq!(itone.len(), <Fst4s60 as FrameLayout>::N_SYMBOLS as usize);
    synth_f32(itone, f0, amplitude, &FST4_60A_GFSK)
}

/// Synthesise a 16-bit PCM waveform. Peak equals `amplitude_i16`.
pub fn tones_to_i16(itone: &[u8], f0: f32, amplitude_i16: i16) -> Vec<i16> {
    debug_assert_eq!(itone.len(), <Fst4s60 as FrameLayout>::N_SYMBOLS as usize);
    synth_i16(itone, f0, amplitude_i16, &FST4_60A_GFSK)
}

fn _silence() {
    let _ = <Fst4s60 as ModulationParams>::NTONES;
}
