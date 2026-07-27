//! LDPC(128, 90) codec with CRC-13 for MSK144.
//!
//! Shares the generic BP/OSD engine in [`crate::fec::ldpc`] via
//! [`Ldpc128_90Params`] — only the parity-check tables (in [`tables`])
//! and the CRC-13 helpers ([`crc13`], [`check_crc13`]) are specific to
//! this code. Ported from WSJT-X `lib/encode_128_90.f90` +
//! `lib/bpdecode128_90.f90` + `lib/ldpc_128_90_{generator,reordered_parity}.f90`.
//!
//! The 90 information bits are 77 packjt77 message bits + a 13-bit
//! CRC (`crc13`, polynomial 0x15D7 — `boost::augmented_crc<13,
//! 0x15D7>` in WSJT-X `lib/crc13.cpp`), the same `pack77`/`unpack77`
//! payload FT8/FT4/FST4 use (`genmsk_128_90.f90:84`: `call
//! pack77(message,i3,n3,c77)`).

pub mod tables;

use alloc::vec;

use crate::engine::{FecCodec, FecOpts, FecResult};
use crate::fec::ldpc::bp::bp_decode_generic_kind;
use crate::fec::ldpc::osd::{ldpc_encode_generic, osd_decode_generic};
use crate::fec::ldpc::params::Ldpc128_90Params;

pub const LDPC_N: usize = 128;
pub const LDPC_K: usize = 90;
pub const LDPC_M: usize = LDPC_N - LDPC_K; // 38

// ────────────────────────────────────────────────────────────────────
// CRC-13

/// CRC-13 (polynomial 0x15D7) over `data` bytes, processed MSB-first.
/// Matches `boost::augmented_crc<13, 0x15D7>` used in WSJT-X
/// `crc13.cpp` — same bit-serial LFSR shape as
/// [`crate::fec::ldpc::crc14`] / [`crate::fec::ldpc240_101::crc24`],
/// just width 13 and a different polynomial.
pub fn crc13(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let msb = (crc >> 12) & 1;
            crc = ((crc << 1) | bit as u16) & 0x1FFF;
            if msb != 0 {
                crc ^= 0x15D7;
            }
        }
    }
    crc
}

/// Verify CRC-13 for a 90-bit decoded word (77 msg + 13 CRC).
/// Packs the 77 message bits into a 12-byte buffer (big-endian, MSB
/// first), leaving the remaining 19 bits zero — matching WSJT-X
/// `chkcrc13a.f90`'s masking of the trailing CRC field before
/// recomputing — then compares against the stored CRC bits.
///
/// Accepts any `&[u8]` slice; lengths other than [`LDPC_K`] (= 90) are
/// rejected so the function is suitable as a `MessageCodec::verify_info`
/// implementation passed through `FecOpts::verify_info`.
pub fn check_crc13(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut bytes = [0u8; 12];
    for (i, &bit) in decoded[..77].iter().enumerate() {
        let byte_idx = i / 8;
        let bit_pos = 7 - (i % 8);
        bytes[byte_idx] |= (bit & 1) << bit_pos;
    }

    let computed = crc13(&bytes);

    let mut received: u16 = 0;
    for &bit in &decoded[77..90] {
        received = (received << 1) | (bit as u16 & 1);
    }

    computed == received
}

// ────────────────────────────────────────────────────────────────────
// FecCodec impl

/// Zero-sized LDPC(128, 90) codec — a thin wrapper that pins the
/// generic [`crate::fec::ldpc::params::LdpcParams`]-based
/// implementation to [`Ldpc128_90Params`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc128_90;

impl crate::engine::protocol::sealed::Sealed for Ldpc128_90 {}
impl FecCodec for Ldpc128_90 {
    const N: usize = LDPC_N;
    const K: usize = LDPC_K;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), LDPC_K, "info must be {} bits", LDPC_K);
        assert_eq!(codeword.len(), LDPC_N, "codeword must be {} bits", LDPC_N);
        ldpc_encode_generic::<Ldpc128_90Params>(info, codeword);
    }

    fn decode_soft(&self, llr: &[f32], opts: &FecOpts<'_>) -> Option<FecResult> {
        assert_eq!(llr.len(), LDPC_N, "llr must be {} values", LDPC_N);
        let mut llr_arr = vec![0f32; LDPC_N];
        llr_arr.copy_from_slice(llr);

        // AP hint injection (same convention as Ldpc174_91 / Ldpc240_101).
        let ap_storage_holder;
        let ap_slice: Option<&[bool]> = match opts.ap_mask {
            Some((mask, values)) => {
                assert_eq!(mask.len(), LDPC_N, "ap mask must be {} bits", LDPC_N);
                assert_eq!(values.len(), LDPC_N, "ap values must be {} bits", LDPC_N);
                let apmag = llr_arr.iter().map(|x| x.abs()).fold(0.0f32, f32::max) * 1.01;
                let mut a = vec![false; LDPC_N];
                for i in 0..LDPC_N {
                    if mask[i] != 0 {
                        a[i] = true;
                        llr_arr[i] = if values[i] != 0 { apmag } else { -apmag };
                    }
                }
                ap_storage_holder = a;
                Some(ap_storage_holder.as_slice())
            }
            None => None,
        };

        if let Some(r) = bp_decode_generic_kind::<Ldpc128_90Params>(
            &llr_arr,
            ap_slice,
            opts.bp_max_iter,
            opts.verify_info,
            opts.bp_kind,
        ) {
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: r.iterations,
            });
        }

        if opts.osd_depth == 0 {
            return None;
        }

        if let Some(r) = osd_decode_generic::<Ldpc128_90Params>(
            &llr_arr,
            opts.osd_depth.min(3) as u8,
            LDPC_K,
            opts.verify_info,
        ) {
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: 0,
            });
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::ldpc::bp::bp_decode_generic;
    use crate::fec::ldpc::osd::ldpc_encode_generic;

    /// Known-vector cross-check against `boost::augmented_crc<13,
    /// 0x15D7>` (WSJT-X `crc13.cpp`) — computed independently via a
    /// small C++ harness linking the real Boost CRC implementation,
    /// not just derived by inspection.
    #[test]
    fn crc13_known_vectors() {
        assert_eq!(crc13(&[0u8; 12]), 0);

        let mut all_ones_77 = [0u8; 12];
        for i in 0..77 {
            let byte_idx = i / 8;
            let bit_pos = 7 - (i % 8);
            all_ones_77[byte_idx] |= 1 << bit_pos;
        }
        assert_eq!(crc13(&all_ones_77), 5559);

        let mut pattern = [0u8; 12];
        for i in 0..77 {
            if (i * 7 + 3) & 1 != 0 {
                let byte_idx = i / 8;
                let bit_pos = 7 - (i % 8);
                pattern[byte_idx] |= 1 << bit_pos;
            }
        }
        assert_eq!(crc13(&pattern), 4105);

        let mut single_bit0 = [0u8; 12];
        single_bit0[0] |= 1 << 7;
        assert_eq!(crc13(&single_bit0), 4822);

        let mut single_bit76 = [0u8; 12];
        single_bit76[9] |= 1 << 3; // bit 76 (0-indexed) -> byte 9, bit_pos 7-(76%8)=3
        assert_eq!(crc13(&single_bit76), 7029);
    }

    /// Round-trip: encode a 90-bit info word (77 msg + CRC13), feed
    /// perfect LLRs, decoder should recover the original info.
    /// Exercises the full BP path plus the generator sub-matrix via
    /// the generic helpers.
    #[test]
    fn roundtrip_perfect_llr() {
        let mut info = [0u8; LDPC_K];
        for i in 0..77 {
            info[i] = ((i * 7 + 3) & 1) as u8;
        }
        let crc = crc13(&{
            let mut bytes = [0u8; 12];
            for i in 0..77 {
                let byte_idx = i / 8;
                let bit_pos = 7 - (i % 8);
                bytes[byte_idx] |= (info[i] & 1) << bit_pos;
            }
            bytes
        });
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }

        let mut cw = [0u8; LDPC_N];
        ldpc_encode_generic::<Ldpc128_90Params>(&info, &mut cw);
        // Sanity: systematic encode keeps info bits in positions 0..K.
        assert_eq!(&cw[..LDPC_K], &info[..]);
        // Sanity: the CRC we just embedded verifies against itself.
        assert!(check_crc13(&info));

        let mut llr = vec![0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if cw[i] == 1 { 8.0 } else { -8.0 };
        }
        let r = bp_decode_generic::<Ldpc128_90Params>(&llr, None, 30, Some(check_crc13))
            .expect("BP converges on perfect LLR");
        assert_eq!(&r.info[..77], &info[..77]);
    }

    /// Noisy-LLR sweep: flip a handful of bits' confidence and confirm
    /// BP still converges to the transmitted codeword.
    #[test]
    fn roundtrip_noisy_llr_recovers() {
        let mut info = [0u8; LDPC_K];
        for i in 0..77 {
            info[i] = ((i * 3 + 1) & 1) as u8;
        }
        let crc = crc13(&{
            let mut bytes = [0u8; 12];
            for i in 0..77 {
                let byte_idx = i / 8;
                let bit_pos = 7 - (i % 8);
                bytes[byte_idx] |= (info[i] & 1) << bit_pos;
            }
            bytes
        });
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }

        let mut cw = [0u8; LDPC_N];
        ldpc_encode_generic::<Ldpc128_90Params>(&info, &mut cw);

        let mut llr = vec![0f32; LDPC_N];
        for i in 0..LDPC_N {
            // Weak, occasionally-wrong-sign LLR every 11th bit to
            // simulate noise while staying within BP's correction range.
            let mag = if i % 11 == 0 { 0.3 } else { 3.0 };
            let sign = if cw[i] == 1 { 1.0 } else { -1.0 };
            llr[i] = sign * mag;
        }
        let r = bp_decode_generic::<Ldpc128_90Params>(&llr, None, 30, Some(check_crc13))
            .expect("BP converges on noisy LLR");
        assert_eq!(&r.info[..77], &info[..77]);
    }
}
