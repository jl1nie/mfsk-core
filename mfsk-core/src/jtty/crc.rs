//! CRC-12 over the 34-bit JTTY payload.
//!
//! Ported from WSJT-X `lib/jtty/tbcc.f90` (`encode_crc12`) and
//! `jtty_tbcc_list_decoder.f90` (`jtty_tbcc_crc_valid`); the polynomial is
//! `JTTY_TBCC_PROFILE_1167_1545_80F%outer_polynomial` in
//! `jtty_tbcc_code_profile.f90`.
//!
//! The generator is x¹² + x¹¹ + x³ + x² + x + 1, written `0x80F` with the
//! leading term implicit. It is the **same polynomial** as Q65's CRC-12
//! ([`crate::fec::qra::q65::crc12`]) but a different routine: that one runs
//! LSB-first over 6-bit symbols, this one is a bitwise MSB-first remainder over
//! the payload bits, so the two are not interchangeable.

use super::{INFO_BITS, PAYLOAD_BITS, Payload};

/// Generator polynomial, leading x¹² term implicit.
pub const POLY: u16 = 0x80F;

const TOP: u16 = 0x800;
const MASK: u16 = 0xFFF;

/// The 12-bit register after feeding `bits` (each 0 or 1, first bit first).
///
/// Over a message it is the CRC; over a message *followed by its own CRC* it is
/// zero, which is how [`is_valid`] checks a received word.
pub fn remainder(bits: &[u8]) -> u16 {
    bits.iter().fold(0u16, |reg, &b| {
        let reg = reg ^ (u16::from(b & 1) << 11);
        let reg = if reg & TOP != 0 {
            (reg << 1) ^ POLY
        } else {
            reg << 1
        };
        reg & MASK
    })
}

/// Append the CRC-12 to a payload: 34 payload bits then 12 CRC bits, the CRC's
/// most significant bit first (the encoder's 46 information bits).
pub fn append(payload: &Payload) -> [u8; INFO_BITS] {
    let crc = remainder(payload);
    core::array::from_fn(|i| {
        if i < PAYLOAD_BITS {
            payload[i]
        } else {
            ((crc >> (INFO_BITS - 1 - i)) & 1) as u8
        }
    })
}

/// `true` when the 46 information bits carry a correct CRC-12.
pub fn is_valid(info: &[u8; INFO_BITS]) -> bool {
    remainder(info) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_payload_has_zero_crc() {
        assert_eq!(remainder(&[0u8; PAYLOAD_BITS]), 0);
    }

    #[test]
    fn single_bit_is_the_shifted_polynomial() {
        // A lone 1 in the last payload position is x^0 · x^12 mod g = 0x80F
        // (the register after the bit, before the 12 shifts, is 0x800 -> shifted
        // once with the polynomial folded in).
        let mut p = [0u8; PAYLOAD_BITS];
        p[PAYLOAD_BITS - 1] = 1;
        assert_eq!(remainder(&p), POLY);
    }

    #[test]
    fn append_then_check() {
        for seed in 0u64..200 {
            let mut p = [0u8; PAYLOAD_BITS];
            let mut x = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            for b in p.iter_mut() {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                *b = (x & 1) as u8;
            }
            let info = append(&p);
            assert_eq!(&info[..PAYLOAD_BITS], &p[..]);
            assert!(is_valid(&info), "seed {seed}");
        }
    }

    #[test]
    fn every_single_bit_error_is_caught() {
        let mut p = [0u8; PAYLOAD_BITS];
        for (i, b) in p.iter_mut().enumerate() {
            *b = (i % 3 == 0) as u8;
        }
        let good = append(&p);
        for i in 0..INFO_BITS {
            let mut bad = good;
            bad[i] ^= 1;
            assert!(!is_valid(&bad), "bit {i}");
        }
    }
}
