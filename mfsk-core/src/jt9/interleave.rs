//! JT9 bit-reversal interleaver over 206 coded bits.
//!
//! Same SWAR 8-bit bit-reversal identity WSPR uses (WSJT-X
//! `interleave9.f90`); only the frame length changes from 162 to
//! 206. The permutation is its own inverse-pair: calling
//! [`interleave`] on a buffer and then [`deinterleave`] restores it.
//!
//! Thin wrapper over [`crate::engine::interleave`] (extracted
//! 2026-08-14, code-sharing audit — this module's `bit_reverse_8` was
//! byte-identical to `wspr::bit_reverse_8`, and this module's loop
//! shape was the same permutation walk modulo frame length).

use crate::engine::interleave::{deinterleave_bitrev, interleave_bitrev};

const FRAME: usize = 206;

/// Permute 206 bits: `tmp[bit_reverse_8(i)] = src[p]`, iterating `i`
/// skipping positions whose bit-reverse ≥ 206.
pub fn interleave(bits: &mut [u8; FRAME]) {
    interleave_bitrev(bits);
}

/// Inverse permutation — `tmp[p] = src[bit_reverse_8(i)]`.
pub fn deinterleave(bits: &mut [u8; FRAME]) {
    deinterleave_bitrev(bits);
}

/// f32 variant for LLR arrays.
pub fn deinterleave_llrs(llrs: &mut [f32; FRAME]) {
    deinterleave_bitrev(llrs);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let mut bits = [0u8; FRAME];
        for i in 0..FRAME {
            bits[i] = ((i * 7 + 3) & 1) as u8;
        }
        let original = bits;
        interleave(&mut bits);
        assert_ne!(bits, original, "permutation must change content");
        deinterleave(&mut bits);
        assert_eq!(bits, original, "deinterleave must invert interleave");
    }

    #[test]
    fn llr_round_trip_matches_bits() {
        // LLR sign should track the bit after a round-trip through
        // the f32 deinterleave.
        let mut bits = [0u8; FRAME];
        for i in 0..FRAME {
            bits[i] = ((i * 11 + 5) & 1) as u8;
        }
        let mut llrs = [0f32; FRAME];
        for i in 0..FRAME {
            llrs[i] = if bits[i] == 0 { 4.0 } else { -4.0 };
        }
        interleave(&mut bits);
        // Now deinterleave the LLRs; they should line up with
        // original bits under the same permutation.
        let mut interleaved_llrs = [0f32; FRAME];
        for i in 0..FRAME {
            interleaved_llrs[i] = if bits[i] == 0 { 4.0 } else { -4.0 };
        }
        deinterleave_llrs(&mut interleaved_llrs);
        for i in 0..FRAME {
            let expected = if (((i * 11 + 5) & 1) as u8) == 0 {
                4.0
            } else {
                -4.0
            };
            assert_eq!(interleaved_llrs[i], expected, "pos {i}");
        }
    }
}
