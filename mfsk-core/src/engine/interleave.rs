// SPDX-License-Identifier: GPL-3.0-or-later
//! Bit-reversal interleaver — the permutation WSJT-X uses for WSPR
//! (`interleave9.f90`, `wsprsim_utils.c`'s `bit_reverse_8`) and JT9
//! (`interleave9.f90` again — JT9 and WSPR share the identical FEC
//! polynomials and interleave scheme, differing only in frame length).
//!
//! Extracted (2026-08-14, code-sharing audit, DSP-level sweep) from
//! **four** independent copies: `wspr::bit_reverse_8`/`interleave`/
//! `deinterleave` (`wspr/mod.rs`), a *fifth* inline re-derivation of
//! the same magic-constant formula in `wspr::decode::deinterleave_llrs`
//! (its own comment: "avoid exposing a pub helper" — i.e. a
//! deliberate duplicate of a duplicate), and `jt9::interleave`'s
//! `bit_reverse_8`/`interleave`/`deinterleave`/`deinterleave_llrs`.
//! All five were byte-identical modulo frame length (162 for WSPR, 206
//! for JT9) and the loop-counter's integer type (WSPR walks a native
//! wrapping `u8`; JT9 masks a `u32` with `& 0xff` — the same "count
//! mod 256" behaviour either way, already established as an
//! equivalent-not-identical parametrisation axis when
//! `engine::sync::refine_freq_hz_log_power` was extracted the same
//! way).
//!
//! JT65's own `interleave`/`deinterleave` (`jt65/interleave.rs`) is a
//! **different algorithm** — a 7×9 matrix transpose over 63 RS
//! symbols (`interleave63.f90`), not bit-reversal — and is correctly
//! left alone.

/// 8-bit bit-reversal by SWAR magic-constant multiplication — the
/// identity WSJT-X's bit-reversal interleaver uses (`wsprsim_utils.c`
/// `bit_reverse_8`, a classic Hacker's Delight trick). Input `i` only
/// needs to be considered modulo 256.
#[inline]
pub fn bit_reverse_8(i: u8) -> u8 {
    let i64 = i as u64;
    (((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64) >> 32) as u8
}

/// Permute a `FRAME`-length buffer using the bit-reversal interleaver:
/// position `p` goes to position `j = bit_reverse_8(i)`, where `i`
/// walks `0, 1, 2, …` (mod 256, wrapping) counting only those `i`
/// where `j < FRAME`. Inverted by [`deinterleave_bitrev`].
pub fn interleave_bitrev<T: Copy + Default, const FRAME: usize>(buf: &mut [T; FRAME]) {
    let mut tmp = [T::default(); FRAME];
    let mut p = 0usize;
    let mut i: u32 = 0;
    while p < FRAME {
        let j = bit_reverse_8((i & 0xff) as u8) as usize;
        if j < FRAME {
            tmp[j] = buf[p];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *buf = tmp;
}

/// Inverse of [`interleave_bitrev`]: walks the same `(p, j)` sequence
/// but gathers `tmp[p] = buf[j]`.
pub fn deinterleave_bitrev<T: Copy + Default, const FRAME: usize>(buf: &mut [T; FRAME]) {
    let mut tmp = [T::default(); FRAME];
    let mut p = 0usize;
    let mut i: u32 = 0;
    while p < FRAME {
        let j = bit_reverse_8((i & 0xff) as u8) as usize;
        if j < FRAME {
            tmp[p] = buf[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *buf = tmp;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_162() {
        let mut bits = [0u8; 162];
        for (i, b) in bits.iter_mut().enumerate() {
            *b = ((i * 7 + 3) & 1) as u8;
        }
        let original = bits;
        interleave_bitrev(&mut bits);
        assert_ne!(bits, original, "permutation must change content");
        deinterleave_bitrev(&mut bits);
        assert_eq!(bits, original, "deinterleave must invert interleave");
    }

    #[test]
    fn round_trip_206() {
        let mut bits = [0u8; 206];
        for (i, b) in bits.iter_mut().enumerate() {
            *b = ((i * 11 + 5) & 1) as u8;
        }
        let original = bits;
        interleave_bitrev(&mut bits);
        assert_ne!(bits, original, "permutation must change content");
        deinterleave_bitrev(&mut bits);
        assert_eq!(bits, original, "deinterleave must invert interleave");
    }

    #[test]
    fn llr_variant_round_trips() {
        let mut llrs = [0f32; 162];
        for (i, v) in llrs.iter_mut().enumerate() {
            *v = if i % 3 == 0 { 4.0 } else { -4.0 };
        }
        let original = llrs;
        interleave_bitrev(&mut llrs);
        deinterleave_bitrev(&mut llrs);
        assert_eq!(llrs, original);
    }
}
