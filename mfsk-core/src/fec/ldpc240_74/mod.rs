//! WSJT-X FST4W LDPC(240, 74) codec (50 payload bits + CRC-24).

pub mod tables;

use alloc::vec;

use crate::core::{FecCodec, FecOpts, FecResult};
use crate::fec::ldpc::bp::bp_decode_generic_kind;
use crate::fec::ldpc::osd::{ldpc_encode_generic, osd_decode_generic};
use crate::fec::ldpc::params::Ldpc240_74Params;

pub const LDPC_N: usize = 240;
pub const LDPC_K: usize = 74;
pub const LDPC_M: usize = 166;

pub fn crc24(bits: &[u8]) -> u32 {
    crate::fec::ldpc240_101::crc24(bits)
}

pub fn check_crc24(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut with_zero = [0u8; LDPC_K];
    with_zero[..50].copy_from_slice(&decoded[..50]);
    let expected = crc24(&with_zero);
    let mut got = 0u32;
    for &bit in &decoded[50..74] {
        got = (got << 1) | u32::from(bit & 1);
    }
    expected == got
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc240_74;

impl FecCodec for Ldpc240_74 {
    const N: usize = LDPC_N;
    const K: usize = LDPC_K;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), LDPC_K, "info must be {LDPC_K} bits");
        assert_eq!(codeword.len(), LDPC_N, "codeword must be {LDPC_N} bits");
        ldpc_encode_generic::<Ldpc240_74Params>(info, codeword);
    }

    fn decode_soft(&self, llr: &[f32], opts: &FecOpts<'_>) -> Option<FecResult> {
        assert_eq!(llr.len(), LDPC_N, "llr must be {LDPC_N} values");
        let mut llr_values = vec![0.0f32; LDPC_N];
        llr_values.copy_from_slice(llr);

        let ap_storage;
        let ap_slice: Option<&[bool]> = match opts.ap_mask {
            Some((mask, values)) => {
                assert_eq!(mask.len(), LDPC_N, "ap mask must be {LDPC_N} bits");
                assert_eq!(values.len(), LDPC_N, "ap values must be {LDPC_N} bits");
                let magnitude = llr_values
                    .iter()
                    .map(|value| value.abs())
                    .fold(0.0f32, f32::max)
                    * 1.01;
                let mut locked = vec![false; LDPC_N];
                for index in 0..LDPC_N {
                    if mask[index] != 0 {
                        locked[index] = true;
                        llr_values[index] = if values[index] != 0 {
                            magnitude
                        } else {
                            -magnitude
                        };
                    }
                }
                ap_storage = locked;
                Some(ap_storage.as_slice())
            }
            None => None,
        };

        if let Some(result) = bp_decode_generic_kind::<Ldpc240_74Params>(
            &llr_values,
            ap_slice,
            opts.bp_max_iter,
            opts.verify_info,
            opts.bp_kind,
        ) {
            return Some(FecResult {
                info: result.info,
                hard_errors: result.hard_errors,
                iterations: result.iterations,
            });
        }

        if opts.osd_depth == 0 {
            return None;
        }
        osd_decode_generic::<Ldpc240_74Params>(
            &llr_values,
            opts.osd_depth.min(3) as u8,
            LDPC_K,
            opts.verify_info,
        )
        .map(|result| FecResult {
            info: result.info,
            hard_errors: result.hard_errors,
            iterations: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_perfect_llr() {
        let mut info = [0u8; LDPC_K];
        for (index, bit) in info[..50].iter_mut().enumerate() {
            *bit = ((index * 5 + 1) & 1) as u8;
        }
        let crc = crc24(&info);
        for index in 0..24 {
            info[50 + index] = ((crc >> (23 - index)) & 1) as u8;
        }
        assert!(check_crc24(&info));

        let codec = Ldpc240_74;
        let mut codeword = [0u8; LDPC_N];
        codec.encode(&info, &mut codeword);
        assert_eq!(&codeword[..LDPC_K], &info);
        let llr: Vec<f32> = codeword
            .iter()
            .map(|bit| if *bit == 1 { 8.0 } else { -8.0 })
            .collect();
        let decoded = codec
            .decode_soft(
                &llr,
                &FecOpts {
                    verify_info: Some(check_crc24),
                    ..FecOpts::default()
                },
            )
            .expect("decode perfect codeword");
        assert_eq!(decoded.info, info);
    }
}
