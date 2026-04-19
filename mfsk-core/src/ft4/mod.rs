//! # `ft4` — FT4 decoder and synthesiser
//!
//! FT4 shares the LDPC(174, 91) code and WSJT 77-bit message payload with FT8;
//! only the modulation parameters (4-GFSK at 20.833 baud), frame layout (four
//! 4-symbol Costas arrays at symbols 0 / 33 / 66 / 99) and DSP ratios differ.
//! All heavy lifting is delegated to generic code in [`crate::core`]; this
//! module mainly wires the trait impls and provides decode / synth entry
//! points.

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode};
use crate::fec::Ldpc174_91;
use crate::msg::Wsjt77Message;

pub mod decode;
pub mod encode;

/// FT4 protocol marker: 4-GFSK, 103 symbols over 7.5 s slot, 20.833 Hz tone
/// spacing, four different Costas-4 arrays, LDPC(174,91) FEC, WSJT 77-bit
/// message payload.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576; // 48 ms @ 12 kHz
    const SYMBOL_DT: f32 = 0.048;
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 4; // NFFT1 = 4 × NSPS = 2304
    const NSTEP_PER_SYMBOL: u32 = 2; // half-symbol coarse-sync step (24 ms)
    const NDOWN: u32 = 18; // 12 000 / 18 ≈ 666.7 Hz baseband
    // LLR_SCALE tuning (2.0 / 2.83 / 3.5) was measured to give identical
    // threshold curves — BP already converges within that range. Keeping
    // the WSJT-X default.
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16; // 4 × 4-symbol Costas
    const N_SYMBOLS: u32 = 103; // active channel symbols (excludes 2 ramp symbols)
    const N_RAMP: u32 = 2; // 1 each side, NN2 = 105
    const SYNC_MODE: SyncMode = SyncMode::Block(&FT4_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 7.5;
    const TX_START_OFFSET_S: f32 = 0.5;
}

impl Protocol for Ft4 {
    type Fec = Ldpc174_91;
    type Msg = Wsjt77Message;
    const ID: ProtocolId = ProtocolId::Ft4;
}

/// FT4's four Costas arrays — each a distinct permutation of `[0,1,2,3]`.
const FT4_COSTAS_A: [u8; 4] = [0, 1, 3, 2];
const FT4_COSTAS_B: [u8; 4] = [1, 0, 2, 3];
const FT4_COSTAS_C: [u8; 4] = [2, 3, 1, 0];
const FT4_COSTAS_D: [u8; 4] = [3, 2, 0, 1];

const FT4_SYNC_BLOCKS: [SyncBlock; 4] = [
    SyncBlock {
        start_symbol: 0,
        pattern: &FT4_COSTAS_A,
    },
    SyncBlock {
        start_symbol: 33,
        pattern: &FT4_COSTAS_B,
    },
    SyncBlock {
        start_symbol: 66,
        pattern: &FT4_COSTAS_C,
    },
    SyncBlock {
        start_symbol: 99,
        pattern: &FT4_COSTAS_D,
    },
];
