//! JT9 bit-reversal interleaver over 206 coded bits.
//!
//! Same SWAR 8-bit bit-reversal identity WSPR uses (WSJT-X
//! `interleave9.f90`); only the frame length changes from 162 to
//! 206. The permutation is its own inverse-pair: calling
//! [`interleave`] on a buffer and then [`deinterleave`] restores it.

const FRAME: usize = 206;

#[inline]
fn bit_reverse_8(i: u8) -> u8 {
    let i64 = i as u64;
    (((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64) >> 32) as u8
}

/// Permute 206 bits: `tmp[bit_reverse_8(i)] = src[p]`, iterating `i`
/// skipping positions whose bit-reverse ≥ 206.
pub fn interleave(bits: &mut [u8; FRAME]) {
    let mut tmp = [0u8; FRAME];
    let mut p = 0usize;
    let mut i: u32 = 0;
    while p < FRAME {
        let j = bit_reverse_8((i & 0xff) as u8) as usize;
        if j < FRAME {
            tmp[j] = bits[p];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    bits.copy_from_slice(&tmp);
}

/// Inverse permutation — `tmp[p] = src[bit_reverse_8(i)]`.
pub fn deinterleave(bits: &mut [u8; FRAME]) {
    let mut tmp = [0u8; FRAME];
    let mut p = 0usize;
    let mut i: u32 = 0;
    while p < FRAME {
        let j = bit_reverse_8((i & 0xff) as u8) as usize;
        if j < FRAME {
            tmp[p] = bits[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    bits.copy_from_slice(&tmp);
}

/// f32 variant for LLR arrays.
pub fn deinterleave_llrs(llrs: &mut [f32; FRAME]) {
    let mut tmp = [0f32; FRAME];
    let mut p = 0usize;
    let mut i: u32 = 0;
    while p < FRAME {
        let j = bit_reverse_8((i & 0xff) as u8) as usize;
        if j < FRAME {
            tmp[p] = llrs[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *llrs = tmp;
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
            let expected = if (((i * 11 + 5) & 1) as u8) == 0 { 4.0 } else { -4.0 };
            assert_eq!(interleaved_llrs[i], expected, "pos {i}");
        }
    }
}
