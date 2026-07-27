//! FST4 encode: 77-bit message → 160-symbol tone sequence → 12 kHz PCM.
//!
//! Mirrors `ft4-core::encode` but for the FST4 geometry shared by
//! every sub-mode: LDPC(240, 101) + CRC-24 over the shared 77-bit
//! WSJT payload, GFSK with BT = 2.0. [`message_to_tones`] produces
//! the (period-independent) symbol-domain tone sequence; the
//! `FST4_*_GFSK` constants plus [`tones_to_f32_with_gfsk`] /
//! [`tones_to_i16_with_gfsk`] handle the per-sub-mode sample-domain
//! synthesis. `tones_to_f32` / `tones_to_i16` (no suffix) are FST4-60A
//! convenience wrappers kept for backward compatibility.

use super::Fst4s60;
use crate::engine::dsp::gfsk::{GfskCfg, synth_f32, synth_i16};
use crate::engine::{FecCodec, FrameLayout, ModulationParams};
use crate::fec::Ldpc240_101;

/// FST4-15 GFSK configuration: 12 kHz, 720 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_15_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 720,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 720 / 8,
};

/// FST4-30 GFSK configuration: 12 kHz, 1680 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_30_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 1_680,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 1_680 / 8,
};

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

/// FST4-120 GFSK configuration: 12 kHz, 8200 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_120_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 8_200,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 8_200 / 8,
};

/// FST4-300 GFSK configuration: 12 kHz, 21504 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_300_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 21_504,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 21_504 / 8,
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

/// Encode a 77-bit message into the 160-symbol FST4 tone sequence.
///
/// Period-independent: the tone sequence only depends on `NTONES` /
/// `BITS_PER_SYMBOL` / `GRAY_MAP` / the frame layout, which every FST4
/// sub-mode shares. Callers wanting a different sub-mode's *sample*
/// waveform still call this unchanged and pass its tone sequence to
/// [`tones_to_f32_with_gfsk`] / [`tones_to_i16_with_gfsk`] with the
/// matching `FST4_*_GFSK` constant.
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
    crate::engine::tx::codeword_to_itone::<Fst4s60>(&cw)
}

/// Synthesise a 12 kHz f32 PCM waveform from an FST4 tone sequence
/// using an explicit GFSK config — pass the `FST4_*_GFSK` constant
/// matching the sub-mode `itone` was produced for. Output length is
/// `N_SYMBOLS × cfg.samples_per_symbol` (160 × NSPS).
pub fn tones_to_f32_with_gfsk(itone: &[u8], f0: f32, amplitude: f32, cfg: &GfskCfg) -> Vec<f32> {
    debug_assert_eq!(itone.len(), <Fst4s60 as FrameLayout>::N_SYMBOLS as usize);
    synth_f32(itone, f0, amplitude, cfg)
}

/// Synthesise a 16-bit PCM waveform using an explicit GFSK config.
/// Peak equals `amplitude_i16`. See [`tones_to_f32_with_gfsk`].
pub fn tones_to_i16_with_gfsk(
    itone: &[u8],
    f0: f32,
    amplitude_i16: i16,
    cfg: &GfskCfg,
) -> Vec<i16> {
    debug_assert_eq!(itone.len(), <Fst4s60 as FrameLayout>::N_SYMBOLS as usize);
    synth_i16(itone, f0, amplitude_i16, cfg)
}

/// Synthesise a 12 kHz f32 PCM waveform from an FST4-60A tone
/// sequence. Output length is `N_SYMBOLS × NSPS = 160 × 3888 =
/// 622 080` samples (~51.84 s).
///
/// Convenience wrapper over [`tones_to_f32_with_gfsk`] with
/// [`FST4_60A_GFSK`] — kept for backward compatibility. Other
/// sub-modes: call `tones_to_f32_with_gfsk(itone, f0, amplitude,
/// &FST4_120_GFSK)` etc. directly.
pub fn tones_to_f32(itone: &[u8], f0: f32, amplitude: f32) -> Vec<f32> {
    tones_to_f32_with_gfsk(itone, f0, amplitude, &FST4_60A_GFSK)
}

/// FST4-60A convenience wrapper over [`tones_to_i16_with_gfsk`]. Peak
/// equals `amplitude_i16`. See [`tones_to_f32`] for the multi-sub-mode
/// alternative.
pub fn tones_to_i16(itone: &[u8], f0: f32, amplitude_i16: i16) -> Vec<i16> {
    tones_to_i16_with_gfsk(itone, f0, amplitude_i16, &FST4_60A_GFSK)
}

fn _silence() {
    let _ = <Fst4s60 as ModulationParams>::NTONES;
}
