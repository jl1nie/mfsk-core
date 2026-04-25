// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65 message codec.
//!
//! Q65 reuses the same 77-bit WSJT message format as FT8 / FT4 / FST4
//! (`super::wsjt77`). The only Q65-specific detail at the message
//! layer is the bit-to-GF(64)-symbol packing that feeds the QRA
//! encoder: 77 bits go in as **13 GF(64) symbols** with layout `12 ×
//! 6 bits + 1 × 5 bits`, with the last symbol's LSB zero-padded to
//! complete a 6-bit symbol value.
//!
//! Mirrors the Fortran code in `lib/qra/q65/genq65.f90`:
//!
//! ```text
//! read(c77, '(12b6.6, b5.5)') dgen   ! pack 77 bits into 13 ints (last is 5-bit)
//! dgen(13) = 2 * dgen(13)            ! left-shift the 13th symbol, zero-padding the LSB
//! ```
//!
//! [`Q65Message`] is the [`MessageCodec`] surface; it delegates pack /
//! unpack to the existing 77-bit wsjt77 helpers and just records the
//! correct `CRC_BITS = 12` for Q65 (the CRC-12 lives at the FEC
//! layer; see [`crate::fec::qra::Q65Codec`]).

use super::wsjt77;
use super::{CallsignHashTable, Wsjt77Message};
use crate::core::{DecodeContext, MessageCodec, MessageFields};

/// Pack a 77-bit WSJT message (LSB / MSB convention matching
/// [`super::wsjt77`]: each byte holds one bit in its LSB) into the
/// 13-GF(64)-symbol vector that feeds Q65's QRA encoder.
///
/// Layout: `symbols[0..12]` carry bits `0..72` six at a time
/// (MSB-first within each symbol). `symbols[12]` carries bits
/// `72..77` in its top five bits, with the LSB zero-padded to make
/// it a valid 6-bit GF(64) value.
pub fn pack77_to_symbols(bits77: &[u8; 77]) -> [i32; 13] {
    let mut out = [0_i32; 13];
    for (i, slot) in out.iter_mut().enumerate().take(12) {
        let mut s = 0_i32;
        for b in 0..6 {
            s = (s << 1) | (bits77[6 * i + b] & 1) as i32;
        }
        *slot = s;
    }
    // Last symbol: 5 bits from bits77[72..77], shift left by 1 to
    // zero-pad the LSB into a 6-bit symbol value.
    let mut last = 0_i32;
    for b in 0..5 {
        last = (last << 1) | (bits77[72 + b] & 1) as i32;
    }
    out[12] = last << 1;
    out
}

/// Inverse of [`pack77_to_symbols`]: extract a 77-bit WSJT message
/// from the 13-symbol decoder output. The LSB of `symbols[12]` is
/// discarded (it was zero-padding on the encode side).
pub fn unpack_symbols_to_bits77(symbols: &[i32; 13]) -> [u8; 77] {
    let mut bits = [0_u8; 77];
    for i in 0..12 {
        let s = symbols[i];
        for b in 0..6 {
            bits[6 * i + b] = ((s >> (5 - b)) & 1) as u8;
        }
    }
    // bits 72..77 are the top 5 bits of symbol 12; the LSB is dropped.
    let s = symbols[12];
    for b in 0..5 {
        // Bit positions 5..1 of the 6-bit symbol value (the LSB / bit
        // 0 was the zero-pad).
        bits[72 + b] = ((s >> (5 - b)) & 1) as u8;
    }
    bits
}

/// Q65 [`MessageCodec`] — wire-compatible with [`Wsjt77Message`] at
/// the human-readable level (Q65 transmits standard FT-style
/// callsign / grid / report messages and free text), but advertises
/// the Q65-specific CRC-12 width as metadata.
///
/// The 77-bit ↔ 13-symbol conversion (which is the Q65-specific
/// piece) lives as free functions in this module
/// ([`pack77_to_symbols`] / [`unpack_symbols_to_bits77`]) and is
/// invoked from the protocol's tx / rx paths, not through this trait.
#[derive(Copy, Clone, Debug, Default)]
pub struct Q65Message;

impl MessageCodec for Q65Message {
    type Unpacked = String;
    /// Q65 carries the same 77-bit WSJT payload as FT8 / FT4 / FST4.
    const PAYLOAD_BITS: u32 = 77;
    /// Q65 protects the payload with a CRC-12 (vs the 14-bit CRC
    /// FT8/FT4 use). The CRC sits inside the QRA codec — see
    /// [`crate::fec::qra::Q65Codec`].
    const CRC_BITS: u32 = 12;

    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        // Bit-for-bit identical to FT8/FT4/FST4 — Q65 uses the same
        // 77-bit format. Reuse the existing implementation.
        Wsjt77Message.pack(fields)
    }

    fn unpack(&self, payload: &[u8], ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != 77 {
            return None;
        }
        let mut buf = [0u8; 77];
        buf.copy_from_slice(payload);

        if let Some(any) = ctx.callsign_hash_table.as_ref()
            && let Some(ht) = any.downcast_ref::<CallsignHashTable>()
        {
            return wsjt77::unpack77_with_hash(&buf, ht);
        }
        wsjt77::unpack77(&buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_unpack_roundtrip_random_bits() {
        // Round-trip every distinct bit pattern we care about: zero,
        // all-ones, and a deterministic pseudo-random pattern. The
        // padding bit (LSB of symbol 12) gets discarded on unpack so
        // the original 77 bits must come back unchanged.
        let cases: Vec<[u8; 77]> = vec![
            [0u8; 77],
            [1u8; 77],
            // Pseudo-random bit pattern.
            std::array::from_fn(|i| (((i * 31) ^ 0x55) & 1) as u8),
        ];
        for bits in cases {
            let symbols = pack77_to_symbols(&bits);
            // Each symbol must be a valid GF(64) value (0..64).
            for (k, s) in symbols.iter().enumerate() {
                assert!(*s >= 0 && *s < 64, "symbol[{k}] = {s} out of range");
            }
            let back = unpack_symbols_to_bits77(&symbols);
            assert_eq!(back, bits, "77-bit roundtrip failed");
        }
    }

    #[test]
    fn last_symbol_has_zero_lsb_after_pack() {
        // The pack function must always zero-pad the LSB so that the
        // 13th symbol stays in 0..64 even when bits77[72..77] is all
        // ones.
        let mut bits = [0u8; 77];
        for b in 72..77 {
            bits[b] = 1;
        }
        let symbols = pack77_to_symbols(&bits);
        // Top 5 bits set, LSB zero → 0b111110 = 62.
        assert_eq!(symbols[12], 62);
        assert_eq!(symbols[12] & 1, 0, "LSB padding bit must be 0");
    }

    #[test]
    fn message_codec_pack_matches_wsjt77() {
        // Q65Message must produce byte-identical packed output to
        // Wsjt77Message (Q65 reuses the format unchanged).
        let fields = MessageFields {
            call1: Some("CQ".to_string()),
            call2: Some("JA1ABC".to_string()),
            grid: Some("PM95".to_string()),
            ..Default::default()
        };
        let q65 = Q65Message.pack(&fields).expect("Q65 pack must succeed");
        let wsjt = Wsjt77Message
            .pack(&fields)
            .expect("Wsjt77 pack must succeed");
        assert_eq!(q65, wsjt);
        assert_eq!(q65.len(), 77);
    }

    #[test]
    fn unpack_roundtrip_preserves_message_text() {
        // Pack a standard message → convert to symbols → convert
        // back → unpack: the human-readable string must round-trip.
        let fields = MessageFields {
            call1: Some("CQ".to_string()),
            call2: Some("K1ABC".to_string()),
            grid: Some("FN42".to_string()),
            ..Default::default()
        };
        let bits = Q65Message.pack(&fields).expect("pack");
        let bits77: [u8; 77] = bits.try_into().expect("77-bit length");
        let symbols = pack77_to_symbols(&bits77);
        let back = unpack_symbols_to_bits77(&symbols);
        let text = Q65Message
            .unpack(&back, &DecodeContext::default())
            .expect("unpack");
        assert_eq!(text, "CQ K1ABC FN42");
    }

    #[test]
    fn payload_and_crc_bit_widths() {
        assert_eq!(<Q65Message as MessageCodec>::PAYLOAD_BITS, 77);
        assert_eq!(<Q65Message as MessageCodec>::CRC_BITS, 12);
    }
}
