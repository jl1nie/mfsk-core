//! # `ft4` — FT4 decoder and synthesiser
//!
//! FT4 shares the LDPC(174, 91) code and WSJT 77-bit message payload with FT8;
//! only the modulation parameters (4-GFSK at 20.833 baud), frame layout (four
//! 4-symbol Costas arrays at symbols 0 / 33 / 66 / 99) and DSP ratios differ.
//! All heavy lifting is delegated to generic code in [`crate::engine`]; this
//! module mainly wires the trait impls and provides decode / synth entry
//! points.
//!
//! ## Quick example
//!
//! Round-trip a synthesised FT4 frame through the decoder:
//!
//! ```
//! # #[cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))] {
//! use mfsk_core::ft4::{
//!     Ft4,
//!     encode::{message_to_tones, tones_to_i16},
//! };
//! use mfsk_core::msg::decode_request::DecodeRequest;
//! use mfsk_core::msg::wsjt77::{pack77, unpack77};
//!
//! // 1. Pack a standard message and synthesise 12 kHz i16 PCM.
//! //    The synth produces just the transmitted frame; pad to the full
//! //    7.5 s slot with the signal starting at 0.5 s.
//! let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack");
//! let tones = message_to_tones(&msg77);
//! let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);
//!
//! let mut audio = vec![0i16; 90_000]; // 7.5 s @ 12 kHz
//! let start = (0.5 * 12_000.0) as usize;
//! for (i, &s) in frame.iter().enumerate() {
//!     if start + i < audio.len() { audio[start + i] = s; }
//! }
//!
//! // 2. Decode it back across the full FT4 band.
//! let results = DecodeRequest::<Ft4>::new(&audio, 100.0, 3_000.0, 1.0, /* max_cand */ 100)
//!     .decode()
//!     .results;
//! assert!(!results.is_empty(), "roundtrip must decode");
//! let msg77: &[u8; 77] = results[0].message77().try_into().unwrap();
//! let text = unpack77(msg77).expect("unpack");
//! assert_eq!(text, "CQ JA1ABC PM95");
//! # }
//! ```

use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode};
use crate::fec::Ldpc174_91;
use crate::msg::Wsjt77Message;

// Decode pulls `engine::pipeline` (FFT trait); gated on the FFT
// meta-feature. `encode` is FFT-free and always available.
// FT4's own coarse-candidate stage lives in `engine::ft4_coarse`, not
// here — `engine::pipeline` calls it directly, and `engine` is compiled
// regardless of which protocol features are enabled, so a
// `crate::ft4::*` reference from `engine::pipeline` would force the
// `ft4` feature on for every build (same reason `engine::sync2d::
// ft4_sync_search` lives in `engine`, not `ft4`, despite being FT4-only).
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod decode;
pub mod encode;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod subtract;

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

    // Spectrum window: Rectangular preserves the synth-roundtrip
    // tests (clean-signal case where Nuttall + global-pct baseline
    // misranks the signal bin against sidelobes). Nuttall is correct
    // per WSJT-X but needs a more robust noise-floor estimator first.
    const SPECTRUM_WINDOW: crate::engine::SpectrumWindow =
        crate::engine::SpectrumWindow::Rectangular;

    // 4-symbol coherent integration on the 3rd LLR variant (matches
    // WSJT-X `get_ft4_bitmetrics.f90:71` `nsym=4`). Gives ~3 dB SNR
    // boost on stable signals vs the default nsym=3 — exactly what
    // weak FT4 goldens at 0.4–3× noise need to land on the true
    // codeword instead of a CRC-14 false positive.
    const LLR_NSYM_MAX: u32 = 4;

    // 77-bit pre-LDPC scrambler (WSJT-X `genft4.f90:64`).
    const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = Some(&FT4_RVEC);
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

/// FT4-specific 77-bit pre-LDPC scrambler. WSJT-X `genft4.f90:33-35,64`
/// XORs the 77-bit message with this fixed sequence before appending
/// CRC-14 + LDPC parity, and `ft4_decode.f90:430` removes the
/// scrambling after LDPC decode + CRC verify. Without it, our
/// decoder converges on a valid codeword whose payload is the
/// message bits XORed with this vector — passes CRC (because the
/// CRC was committed in the scrambled domain) but unpacks as
/// nonsense.
pub const FT4_RVEC: [u8; 77] = [
    0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0,
    1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1,
];

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
