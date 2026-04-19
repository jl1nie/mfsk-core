//! LDPC(240, 101) codec with CRC-24 for the WSJT FST4 / FST4W family.
//!
//! Algorithm mirrors [`super::ldpc`] for LDPC(174, 91); this module
//! supplies the larger code's parity-check tables, generator sub-matrix
//! and CRC polynomial. Algorithm-level improvements added to either
//! module should be mirrored in the other — or, better, the shared
//! BP / OSD bodies should be promoted to code-size-generic helpers
//! (a natural follow-up now that a second concrete size exists).

pub mod bp;
pub mod osd;
pub mod tables;

pub use bp::{BpResult, bp_decode, check_crc24, crc24};
pub use osd::{OsdResult, ldpc_encode, osd_decode, osd_decode_deep};

use crate::core::{FecCodec, FecOpts, FecResult};

pub const LDPC_N: usize = 240;
pub const LDPC_K: usize = 101;
pub const LDPC_M: usize = LDPC_N - LDPC_K; // 139

/// Zero-sized LDPC(240, 101) codec.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc240_101;

impl FecCodec for Ldpc240_101 {
    const N: usize = LDPC_N;
    const K: usize = LDPC_K;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), LDPC_K, "info must be {} bits", LDPC_K);
        assert_eq!(codeword.len(), LDPC_N, "codeword must be {} bits", LDPC_N);
        let mut arr = [0u8; LDPC_K];
        arr.copy_from_slice(info);
        let cw = ldpc_encode(&arr);
        codeword.copy_from_slice(&cw);
    }

    fn decode_soft(&self, llr: &[f32], opts: &FecOpts<'_>) -> Option<FecResult> {
        assert_eq!(llr.len(), LDPC_N, "llr must be {} values", LDPC_N);
        let mut llr_arr = [0f32; LDPC_N];
        llr_arr.copy_from_slice(llr);

        // AP hint injection (same convention as Ldpc174_91).
        let ap_storage;
        let ap_mask: Option<&[bool; LDPC_N]> = match opts.ap_mask {
            Some((mask, values)) => {
                assert_eq!(mask.len(), LDPC_N, "ap mask must be {} bits", LDPC_N);
                assert_eq!(values.len(), LDPC_N, "ap values must be {} bits", LDPC_N);
                let apmag = llr_arr
                    .iter()
                    .map(|x| x.abs())
                    .fold(0.0f32, f32::max)
                    * 1.01;
                let mut a = [false; LDPC_N];
                for i in 0..LDPC_N {
                    if mask[i] != 0 {
                        a[i] = true;
                        llr_arr[i] = if values[i] != 0 { apmag } else { -apmag };
                    }
                }
                ap_storage = a;
                Some(&ap_storage)
            }
            None => None,
        };

        if let Some(r) = bp_decode(&llr_arr, ap_mask, opts.bp_max_iter) {
            let mut info = vec![0u8; LDPC_K];
            info[..77].copy_from_slice(&r.message77);
            info[77..].copy_from_slice(&r.codeword[77..LDPC_K]);
            return Some(FecResult {
                info,
                hard_errors: r.hard_errors,
                iterations: r.iterations,
            });
        }

        if opts.osd_depth == 0 {
            return None;
        }

        let r = osd_decode_deep(&llr_arr, opts.osd_depth.min(3) as u8)?;
        let mut info = vec![0u8; LDPC_K];
        info[..77].copy_from_slice(&r.message77);
        info[77..].copy_from_slice(&r.codeword[77..LDPC_K]);
        Some(FecResult {
            info,
            hard_errors: r.hard_errors,
            iterations: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip: encode a 101-bit info word, feed perfect LLRs, decoder
    /// should recover the original info. Exercises the full BP path plus
    /// the generator sub-matrix.
    #[test]
    fn roundtrip_perfect_llr() {
        // Build a 77-bit message + 24-bit CRC = 101-bit info word.
        let mut info = [0u8; LDPC_K];
        for i in 0..77 {
            info[i] = ((i * 7 + 3) & 1) as u8;
        }
        let crc = crc24(&info); // upper 24 bits still zero
        for i in 0..24 {
            info[77 + i] = ((crc >> (23 - i)) & 1) as u8;
        }

        let cw = ldpc_encode(&info);
        // Sanity: systematic encode keeps info bits in positions 0..K.
        assert_eq!(&cw[..LDPC_K], &info[..]);

        // Perfect LLR: ±8 per bit, sign follows the bit.
        let mut llr = [0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if cw[i] == 1 { 8.0 } else { -8.0 };
        }
        let r = bp_decode(&llr, None, 30).expect("BP converges on perfect LLR");
        assert_eq!(&r.message77[..], &info[..77]);
    }
}
