//! # `jt9` — JT9 decoder and synthesiser
//!
//! JT9 is a 9-FSK mode (8 data tones + 1 sync tone at tone 0) with
//! a 60-second slot, plain FSK shaping, convolutional r=½ K=32 FEC
//! + Fano decoding, and the 72-bit JT message payload shared with
//! JT65. Since the FEC polynomials are identical to WSPR's
//! (`crate::fec::conv::fano::POLY1`/`POLY2`), the Fano decoder body
//! is reused unchanged via [`crate::fec::ConvFano232`] — only the
//! code dimensions differ (72 info + 31 tail → 206 coded bits).
//!
//! Sync is carried by 16 symbols at fixed positions in the 85-symbol
//! frame, each expected on tone 0. That distribution fits the
//! existing [`crate::core::SyncMode::Block`] variant by expressing
//! each sync symbol as a length-1 [`crate::core::SyncBlock`]; no new
//! `SyncMode` variant is required.
//!
//! References:
//! - WSJT-X `lib/jt9_decode.f90`, `lib/jt9sync.f90`, `lib/conv232.f90`,
//!   `lib/fano232.f90`, `lib/interleave9.f90`

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::ConvFano232;
use crate::msg::Jt72Codec;

pub mod interleave;
pub mod rx;
pub mod search;
pub mod sync_pattern;
pub mod tx;

pub use interleave::{deinterleave, deinterleave_llrs, interleave};
pub use rx::demodulate_aligned;
pub use search::{SearchParams, SyncCandidate, coarse_search};
pub use sync_pattern::{JT9_ISYNC, JT9_SYNC_BLOCKS, JT9_SYNC_POSITIONS};
pub use tx::{encode_channel_symbols, synthesize_audio, synthesize_standard};

/// Top-level convenience: decode a JT9 signal at a known (start_sample,
/// base_freq) and return the recovered message if Fano converges.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<crate::msg::Jt72Message> {
    use crate::core::{DecodeContext, FecCodec, FecOpts, MessageCodec};

    let llrs = rx::demodulate_aligned(audio, sample_rate, start_sample, base_freq_hz);
    let codec = ConvFano232;
    let res = codec.decode_soft(&llrs, &FecOpts::default())?;
    let mut payload = [0u8; 72];
    payload.copy_from_slice(&res.info);
    crate::msg::Jt72Codec::default().unpack(&payload, &DecodeContext::default())
}

/// One successful JT9 decode with its alignment info.
#[derive(Clone, Debug)]
pub struct Jt9Decode {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    pub start_sample: usize,
}

/// Scan an audio buffer for any JT9 frames: runs coarse (freq, time)
/// search via [`search::coarse_search`] and tries [`decode_at`] on
/// each candidate in score order, collapsing duplicates that decode
/// to the same message within ±2 Hz / ±1 symbol.
pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> Vec<Jt9Decode> {
    use crate::core::ModulationParams;
    let nsps = (sample_rate as f32 * <Jt9 as ModulationParams>::SYMBOL_DT).round() as usize;
    let cands = search::coarse_search(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Jt9Decode> = Vec::new();
    for c in cands {
        let Some(msg) = decode_at(audio, sample_rate, c.start_sample, c.freq_hz) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == msg
                && (prev.freq_hz - c.freq_hz).abs() <= 2.0
                && (prev.start_sample as i64 - c.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            seen.push(Jt9Decode {
                message: msg,
                freq_hz: c.freq_hz,
                start_sample: c.start_sample,
            });
        }
    }
    seen
}

/// Convenience: scan using [`search::SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Jt9Decode> {
    decode_scan(audio, sample_rate, 0, &search::SearchParams::default())
}

/// JT9 protocol marker.
#[derive(Copy, Clone, Debug, Default)]
pub struct Jt9;

impl ModulationParams for Jt9 {
    const NTONES: u32 = 9;
    const BITS_PER_SYMBOL: u32 = 3; // 8 data tones + 1 sync
    /// Samples per symbol at the 12 kHz pipeline rate. 6912 gives a
    /// baud rate of 12 000 / 6912 ≈ 1.736 Hz, matching WSJT-X.
    const NSPS: u32 = 6912;
    const SYMBOL_DT: f32 = 6912.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 6912.0; // ≈ 1.736 Hz
    /// Data tones are 1..=8; Gray-map the 3 data bits within those
    /// eight tones. Tone 0 is reserved for sync and isn't part of
    /// the data constellation, so the Gray map has 8 entries, not 9.
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2, 6, 7, 5, 4];
    /// No Gaussian shaping — JT9 is plain (square) FSK. Value `0.0`
    /// signals "no GFSK" to TX synthesisers that check the constant.
    const GFSK_BT: f32 = 0.0;
    const GFSK_HMOD: f32 = 1.0;
    /// Two FFTs per symbol window — standard convention (same as FT8).
    const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
    /// Half-symbol coarse-sync step.
    const NSTEP_PER_SYMBOL: u32 = 2;
    /// 12 000 / 8 = 1500 Hz baseband. Adequate for the 9-tone
    /// constellation (9 × 1.736 ≈ 15.6 Hz occupied) plus guard.
    const NDOWN: u32 = 8;
}

impl FrameLayout for Jt9 {
    const N_DATA: u32 = 69;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 85;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Block(&JT9_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 60.0;
    /// JT9 transmissions start at the top of the minute (0 s into the
    /// slot). `tx_start` is 0 rather than WSPR's 1 s.
    const TX_START_OFFSET_S: f32 = 0.0;
}

impl Protocol for Jt9 {
    /// Convolutional r=½ K=32 with Layland-Lushbaugh polynomials —
    /// same as WSPR, different code dimensions (K=72, N=206).
    type Fec = ConvFano232;
    /// 72-bit message payload, shared with JT65.
    type Msg = Jt72Codec;
    const ID: ProtocolId = ProtocolId::Jt9;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::FecCodec;

    #[test]
    fn jt9_trait_surface() {
        assert_eq!(<Jt9 as ModulationParams>::NTONES, 9);
        assert_eq!(<Jt9 as ModulationParams>::BITS_PER_SYMBOL, 3);
        assert_eq!(<Jt9 as ModulationParams>::NSPS, 6912);
        assert!((<Jt9 as ModulationParams>::SYMBOL_DT - 0.576).abs() < 1e-3,);
        assert_eq!(<Jt9 as FrameLayout>::N_SYMBOLS, 85);
        assert_eq!(<Jt9 as FrameLayout>::N_SYNC, 16);
        assert_eq!(<Jt9 as FrameLayout>::N_DATA, 69);
        assert_eq!(<Jt9 as FrameLayout>::T_SLOT_S, 60.0);

        match <Jt9 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 16);
                assert_eq!(blocks[0].start_symbol, 0);
                assert_eq!(blocks[15].start_symbol, 84);
                for b in blocks {
                    assert_eq!(b.pattern, &[0u8]);
                }
            }
            SyncMode::Interleaved { .. } => panic!("JT9 must use Block sync"),
        }

        assert_eq!(<<Jt9 as Protocol>::Fec as FecCodec>::N, 206);
        assert_eq!(<<Jt9 as Protocol>::Fec as FecCodec>::K, 72);
    }
}
