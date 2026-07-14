//! FST4 encode: 77-bit message → 160-symbol tone sequence → 12 kHz PCM.
//!
//! Mirrors `ft4-core::encode` but for the [`Fst4s60`] geometry:
//! LDPC(240, 101) + CRC-24 over the shared 77-bit WSJT payload,
//! GFSK with BT = 2.0 and 324 ms symbols.

use super::Fst4s60;
use crate::core::dsp::gfsk::{GfskCfg, synth_f32, synth_i16};
use crate::core::{FecCodec, FrameLayout, ModulationParams};
use crate::fec::Ldpc240_101;

/// FST4-60A GFSK configuration: 12 kHz, 3888 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
///
/// Was `samples_per_symbol: 3840, bt: 1.0` — hardcoded independently
/// of [`ModulationParams`] and never updated to match WSJT-X
/// `fst4_decode.f90`'s `nsps=3888` / `gen_fst4wave.f90`'s
/// `gfsk_pulse(2.0,tt)` for `ntrperiod.eq.60` (issue #23 root cause).
pub const FST4_60A_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 3_888,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 3_888 / 8,
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
///
/// WSJT-X `genfst4.f90:63` XORs the message with `rvec` before CRC-24 +
/// LDPC encode; `message_to_tones` transmits the **scrambled** message
/// bits — same as WSJT-X, so the receive-side CRC verification stays
/// correct (see `super::FST4_RVEC`'s doc comment).
pub fn message_to_tones(message77: &[u8; 77]) -> Vec<u8> {
    let mut scrambled = *message77;
    for (b, &r) in scrambled.iter_mut().zip(super::FST4_RVEC.iter()) {
        *b = (*b ^ r) & 1;
    }
    let info = append_crc24(&scrambled);
    let codec = Ldpc240_101;
    let mut cw = [0u8; 240];
    codec.encode(&info, &mut cw);
    crate::core::tx::codeword_to_itone::<Fst4s60>(&cw)
}

/// Synthesise a 12 kHz f32 PCM waveform from an FST4 tone sequence.
/// Output length is `N_SYMBOLS × NSPS = 160 × 3888 = 622 080` samples
/// (~51.84 s).
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
