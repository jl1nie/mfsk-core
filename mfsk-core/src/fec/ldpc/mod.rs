//! LDPC (174, 91) codec with CRC-14 (polynomial 0x2757).
//!
//! This is the Forward Error Correction layer shared by FT8, FT4, FT2 and
//! FST4. The code itself is identical across protocols; different message
//! payloads (always 77 information bits) plus a 14-bit CRC are systematically
//! encoded to 174 codeword bits.
//!
//! ## Organisation
//!
//! | Module        | Role                                              |
//! |---------------|---------------------------------------------------|
//! | [`tables`]    | Parity-check matrix (MN / NM / NRW) — static data |
//! | [`bp`]        | Belief-propagation soft-decision decoder          |
//! | [`osd`]       | Ordered-statistics decoder (order 0..4) fallback  |
//!
//! ## Public surface
//!
//! - [`Ldpc174_91`] — zero-sized type implementing [`crate::core::FecCodec`].
//! - [`bp::bp_decode`] / [`osd::osd_decode_deep`] / [`osd::ldpc_encode`] — raw
//!   functions kept stable for the existing ft8-core callers that integrate
//!   CRC checks and AP hints directly.

pub mod bp;
pub mod osd;
pub mod tables;

pub use bp::{BpResult, bp_decode, check_crc14, crc14};
pub use osd::{OsdResult, ldpc_encode, osd_decode, osd_decode_deep, osd_decode_deep4};

use crate::core::{FecCodec, FecOpts, FecResult};

/// Codeword length of the WSJT LDPC code.
pub const LDPC_N: usize = 174;
/// Information-bit length (77 message bits + 14 CRC).
pub const LDPC_K: usize = 91;
/// Parity-bit count.
pub const LDPC_M: usize = LDPC_N - LDPC_K; // 83

/// Zero-sized codec implementing [`FecCodec`] for the WSJT LDPC(174, 91) code.
///
/// All tables are `const` / `static` so the type carries no data — any
/// concrete protocol (FT8/FT4/FT2/FST4) may share a single instance.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc174_91;

impl FecCodec for Ldpc174_91 {
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

        // Apply AP hint: for every `mask[i] == 1`, clamp LLR to ±apmag
        // according to `values[i]` (1 → +apmag, 0 → −apmag).
        // `apmag = max(|llr|) · 1.01` gives the AP bits a stronger vote than
        // any channel observation, matching WSJT-X convention.
        let ap_storage;
        let ap_mask: Option<&[bool; LDPC_N]> = match opts.ap_mask {
            Some((mask, values)) => {
                assert_eq!(mask.len(), LDPC_N, "ap mask must be {} bits", LDPC_N);
                assert_eq!(values.len(), LDPC_N, "ap values must be {} bits", LDPC_N);
                let apmag = llr_arr.iter().map(|x| x.abs()).fold(0.0f32, f32::max) * 1.01;
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

        let r = if opts.osd_depth >= 4 {
            osd_decode_deep4(&llr_arr, 30)?
        } else {
            osd_decode_deep(&llr_arr, opts.osd_depth.min(3) as u8)?
        };
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
