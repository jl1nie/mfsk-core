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
//! bits 84 .. 91 : CRC-7 over bits 0..84 (poly x^7 + x^3 + 1, 0x09)
//! ```
//!
//! Length codes 0..=9 encode payload byte counts 1..=10. Codes 10..=15
//! are reserved and cause [`MessageCodec::unpack`] to return `None`.
//! The CRC-7 occupies the trailing 7 bits and is verified both on
//! [`MessageCodec::unpack`] and as the in-FEC `verify_info` integrity
//! check (BP rejects mid-iteration on CRC-7 fail). Polynomial
//! `x^7 + x^3 + 1` (0x09) — the SD-card standard CRC-7. Hamming
//! distance ≥ 3 over the 84-bit input; combined with LDPC's already-low
//! post-FEC BER this drops false-decode rate by ~2 orders of magnitude
//! versus the naive "always accept" verifier.
//!
//! [`MessageCodec::Unpacked = Vec<u8>`] — the codec's `unpack`
//! returns the payload bytes only (length and CRC fields stripped).

use crate::core::{DecodeContext, MessageCodec, MessageFields};

/// Maximum payload length in bytes per frame.
pub const MAX_PAYLOAD_BYTES: usize = 10;

/// Number of head bits (length + payload) covered by the CRC.
const HEAD_BITS: usize = 84;
/// CRC-7 generator polynomial (`x^7 + x^3 + 1` = 0b1001001 = 0x09 in
/// the standard SD-card form). Uses the leading `x^7` term implicitly;
/// the value below is the 7-bit polynomial without the high bit.
const CRC7_POLY: u8 = 0x09;

/// CRC-7 over `bits` (one bit per byte, LSB), MSB-first bit order.
///
/// Returns the 7-bit CRC value (top 7 bits of the final shift register
/// `<< 1`). Mirrors the canonical WSJT bit-buffer CRC pattern: shift in
/// each input bit at the LSB of an 8-bit register, XOR the polynomial
/// when the bit shifted out at position 7 is set.
fn crc7(bits: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &bit in bits {
        let in_bit = bit & 1;
        let top = (crc >> 6) & 1;
        crc = ((crc << 1) | in_bit) & 0x7F;
        if top ^ in_bit != 0 {
            // Standard CRC-7 step: XOR the 7-bit poly when the bit
            // about to overflow XOR'd with the incoming bit is 1.
            crc ^= CRC7_POLY;
        }
    }
    crc & 0x7F
}

/// Variable-length byte-payload codec. See module docs for the bit
/// layout.
#[derive(Copy, Clone, Debug, Default)]
pub struct PacketBytesMessage;

impl MessageCodec for PacketBytesMessage {
    type Unpacked = Vec<u8>;

    /// 91 information bits matching `Ldpc174_91`'s K. Of those, 4 bits
    /// are length, 80 bits are up to 10 bytes of payload, and the
    /// final 7 bits are a CRC-7 over the head 84 bits.
    const PAYLOAD_BITS: u32 = 91;
    /// CRC-7 trailing the payload — `x^7 + x^3 + 1` over bits 0..84.
    /// The 7-bit CRC sits at info bits 84..91. See [`crc7`] /
    /// [`Self::verify_info`].
    const CRC_BITS: u32 = 7;

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
        // 80 bits of payload (10 bytes max), MSB first per byte. Bytes
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
        // CRC-7 over bits 0..84 in the trailing 7 bits.
        let crc = crc7(&out[..HEAD_BITS]);
        for i in 0..7 {
            out[HEAD_BITS + i] = (crc >> (6 - i)) & 1;
        }
        Some(out)
    }

    fn unpack(&self, payload: &[u8], _ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != Self::PAYLOAD_BITS as usize {
            return None;
        }
        // Verify CRC-7 first — rejects garbage that survived BP parity.
        if !Self::verify_info(payload) {
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

    /// Verify the CRC-7 trailer. Called by the FEC layer (BP / OSD)
    /// to reject parity-converged candidates whose CRC doesn't match —
    /// substantially reduces uvpacket false-decode rate.
    fn verify_info(info: &[u8]) -> bool {
        if info.len() != Self::PAYLOAD_BITS as usize {
            return false;
        }
        let computed = crc7(&info[..HEAD_BITS]);
        let mut received: u8 = 0;
        for &b in &info[HEAD_BITS..(HEAD_BITS + 7)] {
            received = (received << 1) | (b & 1);
        }
        computed == received
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
        //   bits[12..84] = zero-padded payload tail
        //   bits[84..91] = CRC-7 over bits[..84]
        let bits = pack(b"\xAA").expect("pack 0xAA");
        assert_eq!(bits.len(), 91);
        assert_eq!(&bits[0..4], &[0, 0, 0, 0], "length code");
        assert_eq!(&bits[4..12], &[1, 0, 1, 0, 1, 0, 1, 0], "byte 0 bits");
        for &b in &bits[12..84] {
            assert_eq!(b, 0, "payload tail must be zero");
        }
        // CRC-7 of bits[..84] must match what the codec wrote at [84..91].
        let computed = crc7(&bits[..84]);
        let mut stored: u8 = 0;
        for &b in &bits[84..91] {
            stored = (stored << 1) | (b & 1);
        }
        assert_eq!(stored, computed, "trailer must hold the CRC-7 of the head");
    }

    #[test]
    fn unpack_rejects_bit_flip_in_payload() {
        // A single bit flip anywhere in the head (length + payload)
        // must invalidate the CRC-7 and cause unpack to return None,
        // demonstrating the integrity check is wired correctly.
        let mut bits = pack(b"hello").expect("pack");
        // Flip a bit in the middle of the payload.
        bits[20] ^= 1;
        assert!(
            unpack(&bits).is_none(),
            "single bit flip must fail CRC-7 verification"
        );
    }

    #[test]
    fn unpack_rejects_bit_flip_in_crc() {
        // A bit flip in the CRC-7 trailer alone must also fail.
        let mut bits = pack(b"hi").expect("pack");
        bits[88] ^= 1;
        assert!(
            unpack(&bits).is_none(),
            "bit flip in CRC trailer must fail verification"
        );
    }

    #[test]
    fn verify_info_accepts_valid_pack_output() {
        // Every output of `pack` must satisfy `verify_info` — it's the
        // codec's own integrity contract that the FEC layer relies on.
        for payload in [b"x".as_slice(), b"hello", b"\x00\x01\x02\x03\x04"] {
            let bits = pack(payload).expect("pack");
            assert!(
                PacketBytesMessage::verify_info(&bits),
                "verify_info must accept a fresh pack() output for {:?}",
                payload
            );
        }
    }

    #[test]
    fn verify_info_rejects_wrong_length() {
        // The verifier is wired through `FecOpts::verify_info` and
        // sees a slice whose length the FEC controls. Any length other
        // than 91 must reject — guards against accidental misuse from
        // a different FEC.
        assert!(!PacketBytesMessage::verify_info(&[0u8; 90]));
        assert!(!PacketBytesMessage::verify_info(&[0u8; 92]));
        assert!(!PacketBytesMessage::verify_info(&[0u8; 0]));
    }
}
