// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65-30A protocol marker + trait wiring.
//!
//! [`Q65a30`] is the first (and currently only) Q65 sub-mode wired up:
//! 30-second T/R period, A tone-spacing (= baud × 1 = 3.333 Hz). Other
//! sub-modes vary along two orthogonal axes — T/R period (15/30/60/
//! 120/300 s, controlling `NSPS`) and tone-spacing letter A–E
//! (multipliers 1/2/4/8/16, controlling `TONE_SPACING_HZ`) — and can be
//! added later by mirroring the constants below.

use crate::core::{
    FecCodec, FecOpts, FecResult, FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode,
};
use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::Q65Message;

use super::sync_pattern::Q65_SYNC_BLOCKS;

/// Q65-30A: 30-second T/R period, sub-mode A (tone spacing = baud).
///
/// - 65-tone FSK (1 sync + 64 data tones), plain (no GFSK shaping).
/// - 3600 samples / symbol at 12 kHz → baud ≈ 3.333 Hz, frame ≈ 25.5 s
///   inside the 30 s slot.
/// - QRA(15, 65) over GF(64) + CRC-12, two CRC symbols punctured →
///   13 user info symbols on transmit, 63 channel symbols on the air.
/// - 22 distributed sync symbols (always tone 0) at fixed positions.
/// - 77-bit WSJT message ([`Q65Message`]) padded with one zero bit
///   to match the 13 × 6 = 78-bit FEC information field.
#[derive(Copy, Clone, Debug, Default)]
pub struct Q65a30;

impl ModulationParams for Q65a30 {
    /// 65 = 1 sync tone (index 0) + 64 data tones (indices 1..=64).
    /// Q65 has no skipped tone (unlike JT65, which leaves index 1
    /// unused and so reports `NTONES = 66`).
    const NTONES: u32 = 65;
    const BITS_PER_SYMBOL: u32 = 6;

    /// 3600 samples/symbol at 12 kHz → 0.300 s/symbol, baud 3.333 Hz.
    /// Per `q65params.f90`, this is the value for the 30 s T/R period.
    const NSPS: u32 = 3600;
    const SYMBOL_DT: f32 = 3600.0 / 12_000.0;
    /// Sub-mode A multiplier is 1× the baud, so spacing == baud.
    const TONE_SPACING_HZ: f32 = 12_000.0 / 3600.0; // ≈ 3.333 Hz

    /// QRA encodes / decodes at the *symbol* level — there is no
    /// channel-side Gray map. An identity table satisfies the trait
    /// invariant `GRAY_MAP.len() == NTONES`.
    const GRAY_MAP: &'static [u8] = &IDENTITY_65;

    /// Plain FSK — Q65 does not Gaussian-shape its tones.
    const GFSK_BT: f32 = 0.0;
    const GFSK_HMOD: f32 = 1.0;

    const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
    const NSTEP_PER_SYMBOL: u32 = 2;
    /// Decimate to 4 kHz baseband — comfortably wider than the
    /// 65 × 3.333 Hz ≈ 217 Hz Q65-30A occupancy.
    const NDOWN: u32 = 3;
}

const IDENTITY_65: [u8; 65] = {
    let mut m = [0u8; 65];
    let mut i = 0usize;
    while i < 65 {
        m[i] = i as u8;
        i += 1;
    }
    m
};

impl FrameLayout for Q65a30 {
    const N_DATA: u32 = 63;
    const N_SYNC: u32 = 22;
    const N_SYMBOLS: u32 = 85;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Block(&Q65_SYNC_BLOCKS);

    /// 30-second slot.
    const T_SLOT_S: f32 = 30.0;
    /// Q65 transmits ~25.5 s into a 30 s slot; the canonical
    /// nominal start offset is 1.0 s (matches WSJT-X's Q65 timing).
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Q65a30 {
    /// QRA-based codec: see [`Q65Fec`] for the trait stub and
    /// [`Q65Codec`] for the real symbol-level decoder used by the
    /// q65 receive pipeline.
    type Fec = Q65Fec;
    /// 77-bit WSJT message (shared with FT8/FT4/FST4); the Q65-
    /// specific 13 × 6 → 78-bit symbol packing happens in the
    /// protocol's tx / rx code, not in the message codec.
    type Msg = Q65Message;
    const ID: ProtocolId = ProtocolId::Q65;
}

/// FecCodec stub for Q65 — present so [`Q65a30`] can satisfy the
/// `Protocol::Fec: FecCodec` bound; the real soft-decision decode
/// path lives in [`Q65Codec`] and is invoked from
/// [`crate::q65::rx`].
///
/// `decode_soft` always returns `None` because the QRA decoder
/// consumes per-symbol probability distributions over GF(64), not
/// bit-level LLRs. `encode` is implemented faithfully (bit-level in,
/// bit-level out) by routing through a transient [`Q65Codec`], so
/// callers that want a quick reference encoding via the generic
/// trait still get the right answer.
#[derive(Copy, Clone, Debug, Default)]
pub struct Q65Fec;

impl FecCodec for Q65Fec {
    /// 63 transmitted channel symbols × 6 bits.
    const N: usize = 63 * 6;
    /// 13 user info symbols × 6 bits = 78 bits (77-bit Wsjt77 +
    /// 1-bit zero padding handled at the message-codec boundary).
    const K: usize = 13 * 6;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), Self::K, "encode: info.len() != K");
        assert_eq!(codeword.len(), Self::N, "encode: codeword.len() != N");

        // bits → 13 GF(64) symbols (MSB-first within each 6-bit group).
        let mut info_syms = [0_i32; 13];
        for (i, slot) in info_syms.iter_mut().enumerate() {
            let mut s = 0_i32;
            for b in 0..6 {
                s = (s << 1) | (info[6 * i + b] & 1) as i32;
            }
            *slot = s;
        }

        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut channel = [0_i32; 63];
        codec.encode(&info_syms, &mut channel);

        // 63 GF(64) symbols → bits (MSB-first within each symbol).
        for (i, &sym) in channel.iter().enumerate() {
            for b in 0..6 {
                codeword[6 * i + b] = ((sym >> (5 - b)) & 1) as u8;
            }
        }
    }

    fn decode_soft(&self, _llr: &[f32], _opts: &FecOpts) -> Option<FecResult> {
        // Bit-LLR soft decoding is not the natural API for Q65's
        // GF(64) belief propagation. Use `Q65Codec::decode` (or the
        // protocol-level helpers in `crate::q65::rx`) instead.
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modulation_constants_match_spec() {
        assert_eq!(<Q65a30 as ModulationParams>::NTONES, 65);
        assert_eq!(<Q65a30 as ModulationParams>::BITS_PER_SYMBOL, 6);
        assert_eq!(<Q65a30 as ModulationParams>::NSPS, 3600);
        assert!(
            (<Q65a30 as ModulationParams>::SYMBOL_DT - 0.3).abs() < 1e-6,
            "SYMBOL_DT must be 0.3 s for the 30-s T/R period"
        );
        let spacing = <Q65a30 as ModulationParams>::TONE_SPACING_HZ;
        assert!(
            (spacing - 12_000.0 / 3600.0).abs() < 1e-3,
            "Q65-30A tone spacing must be 12000/3600 ≈ 3.333 Hz, got {spacing}"
        );
    }

    #[test]
    fn frame_layout_constants_match_spec() {
        assert_eq!(<Q65a30 as FrameLayout>::N_DATA, 63);
        assert_eq!(<Q65a30 as FrameLayout>::N_SYNC, 22);
        assert_eq!(<Q65a30 as FrameLayout>::N_SYMBOLS, 85);
        assert_eq!(<Q65a30 as FrameLayout>::N_RAMP, 0);
        assert_eq!(<Q65a30 as FrameLayout>::T_SLOT_S, 30.0);
        match <Q65a30 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 22, "Q65 has 22 distributed sync symbols");
                for b in blocks {
                    assert_eq!(b.pattern, &[0u8], "every Q65 sync symbol is tone 0");
                }
            }
            SyncMode::Interleaved { .. } => {
                panic!("Q65 must use Block sync, not Interleaved")
            }
        }
    }

    #[test]
    fn protocol_id_is_q65() {
        assert_eq!(<Q65a30 as Protocol>::ID, ProtocolId::Q65);
    }

    #[test]
    fn q65fec_encode_matches_q65codec_direct() {
        // The bit-level FecCodec stub must agree with calling
        // `Q65Codec::encode` directly on the same payload.
        let fec = Q65Fec;
        // Pseudo-random 78-bit info pattern.
        let info: Vec<u8> = (0..Q65Fec::K)
            .map(|i| ((i.wrapping_mul(13) ^ 0x55) & 1) as u8)
            .collect();
        let mut codeword = vec![0u8; Q65Fec::N];
        fec.encode(&info, &mut codeword);

        // Compute the ground truth via Q65Codec on the same 13-symbol
        // info vector.
        let mut info_syms = [0_i32; 13];
        for (i, slot) in info_syms.iter_mut().enumerate() {
            let mut s = 0_i32;
            for b in 0..6 {
                s = (s << 1) | (info[6 * i + b] & 1) as i32;
            }
            *slot = s;
        }
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut expected_channel = [0_i32; 63];
        codec.encode(&info_syms, &mut expected_channel);

        // Reconstruct the bit-level codeword from expected_channel
        // and compare.
        let mut expected_bits = vec![0u8; Q65Fec::N];
        for (i, &sym) in expected_channel.iter().enumerate() {
            for b in 0..6 {
                expected_bits[6 * i + b] = ((sym >> (5 - b)) & 1) as u8;
            }
        }
        assert_eq!(codeword, expected_bits);
    }

    #[test]
    fn decode_soft_is_a_stub() {
        let fec = Q65Fec;
        let llr = vec![0.0_f32; Q65Fec::N];
        assert!(fec.decode_soft(&llr, &FecOpts::default()).is_none());
    }
}
