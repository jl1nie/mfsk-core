// SPDX-License-Identifier: GPL-3.0-or-later
//! `PacketBytesMessage` — variable-length byte-payload message codec.
//!
//! Used by the [`crate::uvpacket`] family. Unlike the WSJT-style
//! codecs ([`crate::msg::Wsjt77Message`], [`crate::msg::Wspr50Message`],
//! [`crate::msg::Jt72Codec`]) which pack callsign / grid / report
//! fields into a fixed-width payload, this codec carries an
//! arbitrary byte slice of length 1..=10 in 91 information bits.
//!
//! ## Bit layout (91 bits)
//!
//! ```text
//! bits  0 ..  4 : length code (4 bits) = (actual_length - 1)
//! bits  4 .. 84 : 10 bytes × 8 = 80 bits, MSB-first per byte
//! bits 84 .. 91 : 7 bits of zero padding (sent as zero, ignored on unpack)
//! ```
//!
//! Length codes 0..=9 encode payload byte counts 1..=10. Codes 10..=15
//! are reserved and cause [`MessageCodec::unpack`] to return `None`.
//!
//! [`MessageCodec::Unpacked = Vec<u8>`] — the codec's `unpack`
//! returns the payload bytes only (length field stripped).

use crate::core::{DecodeContext, MessageCodec, MessageFields};

/// Maximum payload length in bytes per frame.
pub const MAX_PAYLOAD_BYTES: usize = 10;

/// Variable-length byte-payload codec. See module docs for the bit
/// layout.
#[derive(Copy, Clone, Debug, Default)]
pub struct PacketBytesMessage;

impl MessageCodec for PacketBytesMessage {
    type Unpacked = Vec<u8>;

    /// 91 information bits matching `Ldpc174_91`'s K. Of those, 4 bits
    /// are length, 80 bits are up to 10 bytes of payload, and the
    /// final 7 bits are zero padding.
    const PAYLOAD_BITS: u32 = 91;
    /// CRC-12 protects the payload at the FEC layer (handled by the
    /// underlying QRA codec when used with Q65; for `uvpacket` the
    /// frame's LDPC(174,91) FEC + the pack-time length sanity check
    /// take the place of a separate CRC, so this codec advertises 0).
    const CRC_BITS: u32 = 0;

    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        // The codec is byte-oriented: callers pass payload via the
        // `free_text` field (interpreting the bytes as UTF-8 is up
        // to the application — `Vec<u8>` is what comes back out).
        let bytes = fields.free_text.as_ref()?.as_bytes();
        if bytes.is_empty() || bytes.len() > MAX_PAYLOAD_BYTES {
            return None;
        }
        let mut out = vec![0u8; PacketBytesMessage::PAYLOAD_BITS as usize];
        // 4-bit length field (length - 1 in 0..=10, big-endian, MSB first).
        let len_code = (bytes.len() - 1) as u8;
        for i in 0..4 {
            out[i] = (len_code >> (3 - i)) & 1;
        }
        // 88 bits of payload (11 bytes), MSB first per byte. Bytes
        // beyond `len` are zero-padded.
        for byte_idx in 0..MAX_PAYLOAD_BYTES {
            let b = if byte_idx < bytes.len() {
                bytes[byte_idx]
            } else {
                0
            };
            for bit in 0..8 {
                out[4 + byte_idx * 8 + bit] = (b >> (7 - bit)) & 1;
            }
        }
        Some(out)
    }

    fn unpack(&self, payload: &[u8], _ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != Self::PAYLOAD_BITS as usize {
            return None;
        }
        // Length: 4 bits, big-endian, encodes (len - 1) in 0..=10.
        let mut len_code: u8 = 0;
        for i in 0..4 {
            len_code = (len_code << 1) | (payload[i] & 1);
        }
        let len = len_code as usize + 1;
        if len > MAX_PAYLOAD_BYTES {
            return None;
        }
        // Payload bytes: 8 bits each, MSB first.
        let mut out = Vec::with_capacity(len);
        for byte_idx in 0..len {
            let mut b: u8 = 0;
            for bit in 0..8 {
                b = (b << 1) | (payload[4 + byte_idx * 8 + bit] & 1);
            }
            out.push(b);
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pack(bytes: &[u8]) -> Option<Vec<u8>> {
        let fields = MessageFields {
            free_text: Some(unsafe { std::str::from_utf8_unchecked(bytes) }.to_string()),
            ..Default::default()
        };
        PacketBytesMessage.pack(&fields)
    }

    fn unpack(bits: &[u8]) -> Option<Vec<u8>> {
        PacketBytesMessage.unpack(bits, &DecodeContext::default())
    }

    #[test]
    fn pack_then_unpack_roundtrips_short_payload() {
        let payload = b"hello";
        let bits = pack(payload).expect("pack short");
        let out = unpack(&bits).expect("unpack short");
        assert_eq!(out, payload);
    }

    #[test]
    fn pack_then_unpack_roundtrips_max_length() {
        let payload = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a"; // 10 bytes
        assert_eq!(payload.len(), MAX_PAYLOAD_BYTES);
        let bits = pack(payload).expect("pack 10");
        let out = unpack(&bits).expect("unpack 10");
        assert_eq!(out, payload);
    }

    #[test]
    fn pack_then_unpack_roundtrips_single_byte() {
        let payload = b"\x42";
        let bits = pack(payload).expect("pack 1");
        let out = unpack(&bits).expect("unpack 1");
        assert_eq!(out, payload);
    }

    #[test]
    fn pack_rejects_empty_payload() {
        assert!(pack(b"").is_none(), "empty payload must be rejected");
    }

    #[test]
    fn pack_rejects_oversize_payload() {
        let bytes = vec![0x55_u8; 11]; // one byte over MAX_PAYLOAD_BYTES
        let fields = MessageFields {
            free_text: Some(unsafe { String::from_utf8_unchecked(bytes) }),
            ..Default::default()
        };
        assert!(
            PacketBytesMessage.pack(&fields).is_none(),
            "11-byte payload must be rejected"
        );
    }

    #[test]
    fn unpack_rejects_wrong_length_buffer() {
        let bits = vec![0u8; 90]; // off by one
        assert!(unpack(&bits).is_none(), "bit buffer of length 90 rejected");
        let bits = vec![0u8; 92];
        assert!(unpack(&bits).is_none(), "bit buffer of length 92 rejected");
    }

    #[test]
    fn unpack_rejects_invalid_length_code() {
        // 4-bit length code = 10 → decoded length 11 > MAX_PAYLOAD_BYTES.
        let mut bits = vec![0u8; 91];
        bits[0] = 1;
        bits[1] = 0;
        bits[2] = 1;
        bits[3] = 0; // 0b1010 = 10 → length 11
        assert!(
            unpack(&bits).is_none(),
            "length code 10 (→ 11 bytes) must reject"
        );
    }

    #[test]
    fn pack_payload_bits_in_correct_positions() {
        // Sanity-check the bit layout. Single-byte payload 0xAA:
        //   length code = 0 (encodes 1 byte) → 4 bits of 0
        //   byte 0 = 0xAA = 0b10101010 → bits[4..12] = 1,0,1,0,1,0,1,0
        //   remaining 80 bits = zero-padded
        let bits = pack(b"\xAA").expect("pack 0xAA");
        assert_eq!(bits.len(), 91);
        assert_eq!(&bits[0..4], &[0, 0, 0, 0], "length code");
        assert_eq!(&bits[4..12], &[1, 0, 1, 0, 1, 0, 1, 0], "byte 0 bits");
        for &b in &bits[12..] {
            assert_eq!(b, 0, "trailing pad must be zero");
        }
    }
}
