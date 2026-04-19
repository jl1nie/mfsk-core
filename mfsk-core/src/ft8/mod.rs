//! # `ft8` — FT8 decoder and synthesiser
//!
//! FT8 is the most widely-used WSJT-family mode: 15-second slots,
//! 8-GFSK modulation at 6.25 baud (= 160 ms / symbol), LDPC(174, 91)
//! with CRC-14 inside a 77-bit WSJT message, and three Costas-7 sync
//! blocks at positions 0 / 36 / 72.
//!
//! ## Sample rate
//!
//! The internal decode pipeline assumes **12 000 Hz** PCM input.
//! For other sample rates (e.g. 44 100, 48 000 Hz), use
//! [`resample::resample_to_12k`] to convert before calling
//! [`decode::decode_frame`] or [`decode::decode_sniper_ap`].
//!
//! The WASM wrapper (`ft8-web`) accepts a `sample_rate` parameter
//! on each decode function and handles this conversion automatically.
//!
//! ## Protocol trait
//!
//! The zero-sized [`Ft8`] type implements the generic
//! [`crate::core::Protocol`] trait so downstream pipeline code (shared with
//! FT4, FT2, FST4) can dispatch on `P: Protocol` at compile time.

pub mod decode;
pub mod downsample;
pub mod equalizer;
pub mod hash_table;
pub mod ldpc;
pub mod llr;
pub mod message;
pub mod params;
pub mod resample;
pub mod subtract;
pub mod sync;
pub mod wave_gen;

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode};
use crate::fec::Ldpc174_91;
use crate::msg::Wsjt77Message;

/// FT8 protocol marker: 8-GFSK, 79 symbols over a 15 s slot, 6.25 Hz tone
/// spacing, three 7-symbol Costas arrays, LDPC(174,91) FEC, WSJT 77-bit
/// message payload. Carries no data — used as a type-level switch.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ft8;

impl ModulationParams for Ft8 {
    const NTONES: u32 = params::NTONES as u32;
    const BITS_PER_SYMBOL: u32 = 3;
    const NSPS: u32 = params::NSPS as u32;
    const SYMBOL_DT: f32 = params::SYMBOL_DT;
    const TONE_SPACING_HZ: f32 = 6.25;
    const GRAY_MAP: &'static [u8] = &FT8_GRAY_MAP;
    const GFSK_BT: f32 = 2.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 2; // NFFT1 = 2 × NSPS = 3840
    const NSTEP_PER_SYMBOL: u32 = 4; // quarter-symbol coarse-sync step
    const NDOWN: u32 = 60; // 12 000 / 60 = 200 Hz baseband
}

impl FrameLayout for Ft8 {
    const N_DATA: u32 = params::ND as u32;
    const N_SYNC: u32 = params::NS as u32;
    const N_SYMBOLS: u32 = params::NN as u32;
    const N_RAMP: u32 = 0; // ramp is internal to gfsk::synth
    const SYNC_MODE: SyncMode = SyncMode::Block(&FT8_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 15.0;
    const TX_START_OFFSET_S: f32 = 0.5;
}

impl Protocol for Ft8 {
    type Fec = Ldpc174_91;
    type Msg = Wsjt77Message;
    const ID: ProtocolId = ProtocolId::Ft8;
}

// `params::GRAYMAP` / `params::COSTAS` are `[usize; _]` for historical reasons,
// but `ModulationParams::GRAY_MAP` etc. require `&'static [u8]`. Narrow them
// here at compile time.
const FT8_GRAY_MAP: [u8; 8] = {
    let mut out = [0u8; 8];
    let mut i = 0;
    while i < 8 {
        out[i] = params::GRAYMAP[i] as u8;
        i += 1;
    }
    out
};

const FT8_COSTAS: [u8; 7] = {
    let mut out = [0u8; 7];
    let mut i = 0;
    while i < 7 {
        out[i] = params::COSTAS[i] as u8;
        i += 1;
    }
    out
};

/// FT8 has three identical Costas arrays at symbols 0 / 36 / 72.
const FT8_SYNC_BLOCKS: [SyncBlock; 3] = [
    SyncBlock {
        start_symbol: 0,
        pattern: &FT8_COSTAS,
    },
    SyncBlock {
        start_symbol: 36,
        pattern: &FT8_COSTAS,
    },
    SyncBlock {
        start_symbol: 72,
        pattern: &FT8_COSTAS,
    },
];
