// SPDX-License-Identifier: GPL-3.0-only
//! Frame header + CRC-16 (CCITT-FALSE) for the redesigned uvpacket.
//!
//! ## On-the-wire frame layout (post 0.4.0 redesign)
//!
//! ```text
//! [ Long preamble (mode-encoded, 127 chips) ]
//! [ Header LDPC block — Robust, fixed, 12 bytes info ]
//! [ Payload LDPC blocks × n_blocks — at the mode the preamble
//!   identified ]
//! ```
//!
//! Mode (Robust / Standard / Fast / Express) is now carried in the
//! **preamble pattern selection** rather than in a header field, so
//! the receiver knows the mode after sync detection — well before
//! it tries to decode any LDPC block. This eliminates the
//! `4 modes × 32 n_blocks = 128` brute-force decode sweep that the
//! prior decoder needed to discover layout, and keeps decode cost
//! at `1 + n_blocks` LDPC operations per frame.
//!
//! ## Header block bit layout
//!
//! The header LDPC block carries 96 information bits, of which the
//! first 32 are the header word + CRC and the remaining 64 are
//! zero-pad (extra coding gain for the most-critical block). The
//! header word + CRC layout matches the prior single-frame header
//! exactly except for the removed `mode` field:
//!
//! ```text
//! Bit:   15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
//! Field: └── blocks ──┘└── app ──┘└── seq ──────┘└rsv─┘
//! ```
//!
//! - `blocks` (5 bits) — payload LDPC block count, encoded as
//!   `count - 1` (`0b00000` = 1 block, `0b11111` = 32 blocks).
//! - `app` (4 bits) — application-layer dispatch tag, 0..=15.
//! - `seq` (5 bits) — ARQ sequence number 0..=31, wraps mod 32.
//! - `rsv` (2 bits) — reserved, must be zero.
//!
//! Bytes 0..2 carry this 16-bit word in big-endian. Bytes 2..4
//! carry the CRC-16/CCITT-FALSE computed over the header word
//! plus the payload bytes (and any trailing zero padding to the
//! `n_blocks × 12-byte` block boundary).

use crc::{CRC_16_IBM_3740, Crc};

use super::puncture::Mode;

/// Total header-word + CRC byte count.
pub const HEADER_BYTES: usize = 4;

/// Information bytes carried per LDPC block (96 bits of the 101
/// `Ldpc240_101` info bits; the 5 trailing bits are zero-padded).
pub const INFO_BYTES_PER_BLOCK: usize = 12;

/// Maximum payload LDPC blocks per frame (5-bit `blocks` field,
/// encoded as `count − 1`).
pub const MAX_BLOCKS_PER_FRAME: usize = 32;

/// Maximum application payload — info-byte budget across all 32
/// payload blocks (the dedicated header block carries no payload).
pub const MAX_PAYLOAD_BYTES: usize = MAX_BLOCKS_PER_FRAME * INFO_BYTES_PER_BLOCK;

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection,
/// no XOR-out.
const CRC16_ALGO: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);

/// Decoded uvpacket frame header. `mode` is included for caller
/// convenience even though it is technically conveyed by the
/// preamble pattern at the modulation layer rather than by any
/// bits in the header word.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FrameHeader {
    pub mode: Mode,
    /// Payload LDPC block count, 1..=32.
    pub block_count: u8,
    /// Application-layer dispatch tag, 0..=15.
    pub app_type: u8,
    /// ARQ sequence number, 0..=31.
    pub sequence: u8,
}

/// Errors returned by header packing.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PackError {
    /// `block_count` was outside `1..=32`.
    InvalidBlockCount(u8),
    /// `app_type` was outside `0..=15`.
    InvalidAppType(u8),
    /// `sequence` was outside `0..=31`.
    InvalidSequence(u8),
    /// `payload.len()` exceeded [`MAX_PAYLOAD_BYTES`].
    PayloadTooLarge(usize),
}

/// Errors returned by header unpacking.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum UnpackError {
    /// Input was shorter than [`HEADER_BYTES`].
    Truncated,
    /// CRC-16 mismatch.
    CrcMismatch { expected: u16, computed: u16 },
    /// Reserved bits were non-zero (forward-compatibility check).
    ReservedNotZero(u8),
}

/// Pack the header word + CRC into the first [`HEADER_BYTES`] of a
/// fresh `Vec<u8>`. The CRC is computed over `header_word ++ payload`,
/// where `payload` is the byte stream that the receiver will recover
/// after concatenating decoded payload-block info bytes.
///
/// Note that the returned bytes are **only the header**, not the
/// header-block info bytes. Callers concatenate
/// `pack_header(...)` plus their own payload to form the post-LDPC
/// byte stream that gets verified end-to-end.
pub fn pack_header(header: &FrameHeader, payload: &[u8]) -> Result<[u8; HEADER_BYTES], PackError> {
    if !(1..=MAX_BLOCKS_PER_FRAME as u8).contains(&header.block_count) {
        return Err(PackError::InvalidBlockCount(header.block_count));
    }
    if header.app_type > 15 {
        return Err(PackError::InvalidAppType(header.app_type));
    }
    if header.sequence > 31 {
        return Err(PackError::InvalidSequence(header.sequence));
    }
    if payload.len() > MAX_PAYLOAD_BYTES {
        return Err(PackError::PayloadTooLarge(payload.len()));
    }
    let blocks_bits = u16::from(header.block_count - 1) & 0x1F;
    let app_bits = u16::from(header.app_type) & 0xF;
    let seq_bits = u16::from(header.sequence) & 0x1F;
    let header_word: u16 = (blocks_bits << 11) | (app_bits << 7) | (seq_bits << 2);

    let mut out = [0u8; HEADER_BYTES];
    out[0..2].copy_from_slice(&header_word.to_be_bytes());
    let mut crc_input = Vec::with_capacity(2 + payload.len());
    crc_input.extend_from_slice(&out[0..2]);
    crc_input.extend_from_slice(payload);
    let crc = crc16(&crc_input);
    out[2..4].copy_from_slice(&crc.to_be_bytes());
    Ok(out)
}

/// Inverse of [`pack_header`]: parse the 4-byte header off the front
/// of `bytes`, verify CRC over `header_word ++ payload`, return
/// `(header, payload_slice)` on success.
///
/// `mode` is supplied externally (the preamble identified it) and
/// passed through into the returned [`FrameHeader`] for caller
/// convenience. The CRC is computed over the bytes following
/// `HEADER_BYTES` exactly as produced by [`pack_header`] — the
/// caller is responsible for trimming any zero-padding before
/// presenting the result to the application layer.
pub fn unpack_header(bytes: &[u8], mode: Mode) -> Result<(FrameHeader, &[u8]), UnpackError> {
    if bytes.len() < HEADER_BYTES {
        return Err(UnpackError::Truncated);
    }
    let header_word = u16::from_be_bytes([bytes[0], bytes[1]]);
    let crc_recv = u16::from_be_bytes([bytes[2], bytes[3]]);
    let payload = &bytes[HEADER_BYTES..];

    let mut crc_input = Vec::with_capacity(2 + payload.len());
    crc_input.extend_from_slice(&bytes[..2]);
    crc_input.extend_from_slice(payload);
    let crc_calc = crc16(&crc_input);
    if crc_calc != crc_recv {
        return Err(UnpackError::CrcMismatch {
            expected: crc_recv,
            computed: crc_calc,
        });
    }

    let blocks_code = ((header_word >> 11) & 0x1F) as u8;
    let app_type = ((header_word >> 7) & 0x0F) as u8;
    let sequence = ((header_word >> 2) & 0x1F) as u8;
    let reserved = (header_word & 0x3) as u8;
    if reserved != 0 {
        return Err(UnpackError::ReservedNotZero(reserved));
    }
    let block_count = blocks_code + 1;

    Ok((
        FrameHeader {
            mode,
            block_count,
            app_type,
            sequence,
        },
        payload,
    ))
}

/// CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, no reflection,
/// no XOR-out). Public so callers can verify checksums against
/// alternative byte slicings.
pub fn crc16(bytes: &[u8]) -> u16 {
    CRC16_ALGO.checksum(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_header() -> FrameHeader {
        FrameHeader {
            mode: Mode::Robust,
            block_count: 5,
            app_type: 1,
            sequence: 7,
        }
    }

    #[test]
    fn pack_unpack_roundtrip_empty_payload() {
        let h = sample_header();
        let bytes = pack_header(&h, &[]).unwrap();
        let mut all = Vec::new();
        all.extend_from_slice(&bytes);
        let (h2, p2) = unpack_header(&all, Mode::Robust).unwrap();
        assert_eq!(h, h2);
        assert!(p2.is_empty());
    }

    #[test]
    fn pack_unpack_roundtrip_with_payload() {
        let h = sample_header();
        let payload: Vec<u8> = (0..60).collect();
        let bytes = pack_header(&h, &payload).unwrap();
        let mut all = Vec::new();
        all.extend_from_slice(&bytes);
        all.extend_from_slice(&payload);
        let (h2, p2) = unpack_header(&all, Mode::Robust).unwrap();
        assert_eq!(h, h2);
        assert_eq!(p2, &payload[..]);
    }

    #[test]
    fn pack_unpack_roundtrip_all_modes() {
        for mode in [
            Mode::Robust,
            Mode::Standard,
            Mode::UltraRobust,
            Mode::Express,
        ] {
            let h = FrameHeader {
                mode,
                block_count: 1,
                app_type: 0,
                sequence: 0,
            };
            let bytes = pack_header(&h, b"hi").unwrap();
            let mut all = Vec::new();
            all.extend_from_slice(&bytes);
            all.extend_from_slice(b"hi");
            let (h2, p2) = unpack_header(&all, mode).unwrap();
            assert_eq!(h, h2);
            assert_eq!(p2, b"hi");
        }
    }

    #[test]
    fn pack_rejects_invalid_block_count() {
        for bad in [0u8, 33, 200] {
            let h = FrameHeader {
                mode: Mode::Robust,
                block_count: bad,
                app_type: 0,
                sequence: 0,
            };
            assert_eq!(
                pack_header(&h, &[]).unwrap_err(),
                PackError::InvalidBlockCount(bad),
            );
        }
    }

    #[test]
    fn unpack_detects_header_bit_flip() {
        let h = sample_header();
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&pack_header(&h, b"hello").unwrap());
        bytes.extend_from_slice(b"hello");
        bytes[0] ^= 0x40;
        match unpack_header(&bytes, Mode::Robust) {
            Err(UnpackError::CrcMismatch { .. }) => {}
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }

    #[test]
    fn unpack_detects_payload_bit_flip() {
        let h = sample_header();
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&pack_header(&h, b"hello").unwrap());
        bytes.extend_from_slice(b"hello");
        let pos = HEADER_BYTES + 2;
        bytes[pos] ^= 0x01;
        match unpack_header(&bytes, Mode::Robust) {
            Err(UnpackError::CrcMismatch { .. }) => {}
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }

    #[test]
    fn unpack_rejects_truncated() {
        for n in 0..HEADER_BYTES {
            let bytes = vec![0u8; n];
            assert_eq!(
                unpack_header(&bytes, Mode::Robust).unwrap_err(),
                UnpackError::Truncated,
            );
        }
    }

    #[test]
    fn crc16_canonical_check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn capacity_constants_consistent() {
        assert_eq!(MAX_BLOCKS_PER_FRAME, 32);
        assert_eq!(INFO_BYTES_PER_BLOCK, 12);
        assert_eq!(MAX_PAYLOAD_BYTES, 32 * 12); // = 384, no header subtraction
    }
}
