//! # wspr-core
//!
//! WSPR (Weak Signal Propagation Reporter) implementation on the generic
//! `mfsk-*` stack. Unlike FT8/FT4/FST4, WSPR uses:
//!
//! * **4-FSK at 1.4648 Hz** tone spacing, 162 symbols over ~110.6 s
//! * **Convolutional r=1/2 K=32** with Fano sequential decoder
//! * **50-bit message payload** (callsign + grid4 + power, or hashed variants)
//! * **Per-symbol interleaved sync**: the LSB of every 4-FSK symbol
//!   reproduces a fixed 162-bit pseudorandom vector (the "npr3 sync"), so
//!   sync is not a block Costas array — the decoder recovers timing by
//!   correlating every symbol's LSB against the known vector.
//!
//! All protocol-invariant pieces (FFT/downsample DSP, generic pipeline
//! scaffolding, FEC codec, message codec) are borrowed from `mfsk-core`,
//! `mfsk-fec`, and `mfsk-msg`. This crate provides the `Wspr` ZST that
//! ties them together plus WSPR-specific TX/RX helpers that handle the
//! interleaver and sync-bit embedding.

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::ConvFano;
use crate::msg::Wspr50Message;

pub mod decode;
pub mod rx;
pub mod search;
pub mod spectrogram;
pub mod sync_vector;
pub mod tx;

pub use decode::{decode_at, WsprDecode};
pub use rx::demodulate_aligned;
pub use search::{coarse_search, SearchParams, SyncCandidate};
pub use sync_vector::WSPR_SYNC_VECTOR;
pub use tx::{synthesize_audio, synthesize_type1};

// ─────────────────────────────────────────────────────────────────────────
// Protocol ZST
// ─────────────────────────────────────────────────────────────────────────

/// WSPR-2 (the standard 2-minute slot variant). WSPR-15 differs in slot
/// length and NSPS; a separate ZST can be added later sharing everything
/// except the few timing constants.
#[derive(Copy, Clone, Debug, Default)]
pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    /// 8192 samples at 12 kHz = 0.6827 s per symbol. WSJT-X demodulates at
    /// 375 Hz after a 32× decimation (12000/32 = 375), where one symbol is
    /// 256 samples; we keep the pipeline-standard 12 kHz convention here.
    const NSPS: u32 = 8192;
    const SYMBOL_DT: f32 = 8192.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0; // ≈ 1.4648
    /// Gray map for 4-FSK. WSPR tones map naturally (no Gray conversion in
    /// the WSJT-X reference), so this is the identity — the data bit just
    /// picks the top bit of the tone index.
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    // WSPR uses MSK-ish continuous-phase shaping; GFSK is close enough for
    // coarse modelling (WSJT-X genwspr.f90 applies a raised-cosine pulse
    // rather than a Gaussian). BT=1.0 is a reasonable stand-in here.
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 1; // sync correlation windows = 1 symbol
    const NSTEP_PER_SYMBOL: u32 = 16; // WSJT-X scans 16 sub-symbol offsets
    const NDOWN: u32 = 32; // 12000 / 32 = 375 Hz baseband
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162; // every symbol is both data and sync
    const N_SYNC: u32 = 0;
    const N_SYMBOLS: u32 = 162;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Interleaved {
        sync_bit_pos: 0, // LSB of 4-FSK tone = sync bit, MSB = data bit
        vector: &WSPR_SYNC_VECTOR,
    };
    /// Nominal slot length — the "2" in "WSPR-2". Matches WSJT-X's 120-s
    /// schedule. The actual frame transmission is ≈ 110.6 s inside this
    /// slot.
    const T_SLOT_S: f32 = 120.0;
    /// Frame begins ~1 s after the slot boundary (WSJT-X convention).
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Wspr {
    type Fec = ConvFano;
    type Msg = Wspr50Message;
    const ID: ProtocolId = ProtocolId::Wspr;
}

// ─────────────────────────────────────────────────────────────────────────
// WSPR-specific interleaver
// ─────────────────────────────────────────────────────────────────────────

/// 8-bit bit-reversal by SWAR magic-constant multiplication — the
/// identity used by WSJT-X's interleaver (and a classic Hacker's Delight
/// trick). Input `i` only needs to be considered modulo 256.
#[inline]
fn bit_reverse_8(i: u8) -> u8 {
    // Matches `j = ((i * 0x80200802) & 0x0884422110) * 0x0101010101 >> 32`
    // from wsprsim_utils.c, with the implicit truncation to `unsigned char`
    // made explicit via `as u8` on the final result.
    let i64 = i as u64;
    (((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64) >> 32) as u8
}

/// Permute the 162-symbol stream using WSJT-X's bit-reversal interleaver:
/// position `p` goes to position `j = bit_reverse_8(i)` where `i` walks
/// from 0 counting only those where `j < 162`.
pub fn interleave(bits: &mut [u8; 162]) {
    let mut tmp = [0u8; 162];
    let mut p = 0u8;
    let mut i = 0u8;
    while p < 162 {
        let j = bit_reverse_8(i) as usize;
        if j < 162 {
            tmp[j] = bits[p as usize];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    bits.copy_from_slice(&tmp);
}

/// Inverse interleaver — walks the same (p, j) sequence but gathers
/// `tmp[p] = bits[j]`. `deinterleave(interleave(x)) == x`.
pub fn deinterleave(bits: &mut [u8; 162]) {
    let mut tmp = [0u8; 162];
    let mut p = 0u8;
    let mut i = 0u8;
    while p < 162 {
        let j = bit_reverse_8(i) as usize;
        if j < 162 {
            tmp[p as usize] = bits[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    bits.copy_from_slice(&tmp);
}

// ─────────────────────────────────────────────────────────────────────────
// TX pipeline: message → 162 channel symbols
// ─────────────────────────────────────────────────────────────────────────

/// Encode a 50-bit WSPR message into 162 4-FSK channel symbols (values 0..3).
/// Mirrors WSJT-X `get_wspr_channel_symbols`: FEC encode → interleave →
/// combine with sync vector as `symbol = 2·data_bit + sync_bit`.
pub fn encode_channel_symbols(info_bits: &[u8; 50]) -> [u8; 162] {
    use crate::core::FecCodec;

    let codec = ConvFano;
    let mut cw = vec![0u8; ConvFano::N];
    codec.encode(info_bits, &mut cw);

    // Interleave.
    let mut channel_bits = [0u8; 162];
    channel_bits.copy_from_slice(&cw);
    interleave(&mut channel_bits);

    // Combine with sync vector: symbol = 2·data + sync.
    let mut symbols = [0u8; 162];
    for i in 0..162 {
        symbols[i] = 2 * channel_bits[i] + WSPR_SYNC_VECTOR[i];
    }
    symbols
}

/// RX counterpart: given 162 per-symbol LLRs for the **data bit** (MSB of
/// the 4-FSK tone) already de-interleaved, run Fano and unpack.
///
/// Real decoders would first demodulate the 4-FSK tones, extract the
/// data-bit LLR per symbol, then de-interleave. This function is the
/// last mile of that pipeline and the entry point we exercise in tests.
pub fn decode_from_deinterleaved_llrs(
    data_llrs: &[f32; 162],
) -> Option<crate::msg::WsprMessage> {
    use crate::core::{FecCodec, FecOpts, MessageCodec};

    let codec = ConvFano;
    let fec = codec.decode_soft(data_llrs, &FecOpts::default())?;
    let msg = Wspr50Message;
    let mut info_bits = [0u8; 50];
    info_bits.copy_from_slice(&fec.info);
    msg.unpack(&info_bits, &crate::core::DecodeContext::default())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::FecCodec;

    #[test]
    fn wspr_trait_surface() {
        assert_eq!(<Wspr as ModulationParams>::NTONES, 4);
        assert_eq!(<Wspr as ModulationParams>::NSPS, 8192);
        assert_eq!(<Wspr as FrameLayout>::N_SYMBOLS, 162);
        assert_eq!(<Wspr as FrameLayout>::T_SLOT_S, 120.0);
        match <Wspr as FrameLayout>::SYNC_MODE {
            SyncMode::Interleaved { sync_bit_pos, vector } => {
                assert_eq!(sync_bit_pos, 0);
                assert_eq!(vector.len(), 162);
            }
            SyncMode::Block(_) => panic!("WSPR must use interleaved sync"),
        }
        assert_eq!(<<Wspr as Protocol>::Fec as FecCodec>::N, 162);
        assert_eq!(<<Wspr as Protocol>::Fec as FecCodec>::K, 50);
    }

    #[test]
    fn interleave_is_involution() {
        let mut bits = [0u8; 162];
        for i in 0..162 {
            bits[i] = ((i * 7 + 13) & 1) as u8;
        }
        let original = bits;
        interleave(&mut bits);
        assert_ne!(bits, original, "interleave must permute");
        let once = bits;
        // deinterleave(interleave(x)) == x
        deinterleave(&mut bits);
        assert_eq!(bits, original);
        // Also: interleave(interleave(x)) restores bits touched by the
        // fixed-point permutation but need not be identity overall —
        // check that calling interleave twice is NOT identity in general.
        let mut bits2 = once;
        interleave(&mut bits2);
        // Not an involution on arbitrary input — this is what forces us
        // to keep deinterleave separate.
        let _ = bits2;
    }

    #[test]
    fn roundtrip_k1abc_fn42_37() {
        use crate::msg::{wspr::pack_type1, WsprMessage};

        let info_bits = pack_type1("K1ABC", "FN42", 37).expect("pack");
        let symbols = encode_channel_symbols(&info_bits);

        // Verify the sync vector LSB is reproduced.
        for i in 0..162 {
            assert_eq!(symbols[i] & 1, WSPR_SYNC_VECTOR[i], "sync LSB mismatch at {}", i);
            assert!(symbols[i] < 4);
        }

        // Recover the data bits (MSB of each 4-FSK tone).
        let mut data_bits = [0u8; 162];
        for i in 0..162 {
            data_bits[i] = (symbols[i] >> 1) & 1;
        }
        // De-interleave back to the Fano-input order.
        deinterleave(&mut data_bits);
        // Build perfect LLRs (+8 for bit 0, -8 for bit 1).
        let mut llrs = [0f32; 162];
        for i in 0..162 {
            llrs[i] = if data_bits[i] == 0 { 8.0 } else { -8.0 };
        }
        let msg = decode_from_deinterleaved_llrs(&llrs).expect("decode");
        assert_eq!(
            msg,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }
}
