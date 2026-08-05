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
//! - [`Ldpc174_91`] — zero-sized type implementing [`crate::engine::FecCodec`].
//! - [`bp::bp_decode`] / [`osd::osd_decode_deep`] / [`osd::ldpc_encode`] — raw
//!   functions kept stable for the existing ft8-core callers that integrate
//!   CRC checks and AP hints directly.

pub mod bp;
pub mod osd;
pub mod params;
pub mod tables;

pub use bp::{BpResult, bp_decode, bp_decode_kind, check_crc14, crc14};
pub use osd::{OsdResult, ldpc_encode, osd_decode, osd_decode_deep, osd_decode_deep4};
pub use params::{Ldpc174_91Params, Ldpc240_101Params, LdpcParams};

use crate::engine::protocol::BpPooledFec;
use crate::engine::{FecCodec, FecOpts, FecResult};
use bp::{BpScratch, bp_decode_generic_kind_with_scratch};

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

impl crate::engine::protocol::sealed::Sealed for Ldpc174_91 {}
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
        let (llr_arr, ap_mask) = prepare_ap_llr(llr, opts);

        if let Some(r) = bp_decode_kind(
            &llr_arr,
            ap_mask.as_ref(),
            opts.bp_max_iter,
            opts.verify_info,
            opts.bp_kind,
        ) {
            // Phase 0c-B: BpResult.info is a Vec<u8> of length P::K
            // already, so no copy/reconstruction needed.
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: r.iterations,
            });
        }

        if opts.osd_depth == 0 {
            return None;
        }

        let r = if opts.osd_depth >= 4 {
            osd_decode_deep4(&llr_arr, 30, opts.verify_info)?
        } else {
            osd_decode_deep(&llr_arr, opts.osd_depth.min(3) as u8, opts.verify_info)?
        };
        Some(FecResult {
            info: r.info,
            hard_errors: r.hard_errors,
            iterations: 0,
        })
    }
}

impl BpPooledFec for Ldpc174_91 {
    type Scratch = BpScratch<Ldpc174_91Params, f32>;

    fn decode_soft_pooled(
        &self,
        llr: &[f32],
        opts: &FecOpts<'_>,
        scratch: &mut Self::Scratch,
    ) -> Option<FecResult> {
        let (llr_arr, ap_mask) = prepare_ap_llr(llr, opts);
        let ap_mask_slice: Option<&[bool]> = ap_mask.as_ref().map(|a| a.as_slice());

        if let Some(r) = bp_decode_generic_kind_with_scratch::<Ldpc174_91Params>(
            scratch,
            &llr_arr,
            ap_mask_slice,
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

        // OSD fallback stays unpooled — out of scope for this pass
        // (identical to `decode_soft`'s tail above).
        if opts.osd_depth == 0 {
            return None;
        }

        let r = if opts.osd_depth >= 4 {
            osd_decode_deep4(&llr_arr, 30, opts.verify_info)?
        } else {
            osd_decode_deep(&llr_arr, opts.osd_depth.min(3) as u8, opts.verify_info)?
        };
        Some(FecResult {
            info: r.info,
            hard_errors: r.hard_errors,
            iterations: 0,
        })
    }
}

/// Shared AP-hint LLR preparation for [`Ldpc174_91`]'s pooled and
/// unpooled `decode_soft`: for every `mask[i] == 1`, clamps LLR to
/// ±apmag according to `values[i]` (1 → +apmag, 0 → −apmag);
/// `apmag = max(|llr|) · 1.01` gives the AP bits a stronger vote than
/// any channel observation, matching WSJT-X convention. Returns an
/// owned array (not a borrow tied to `opts`) so both callers can build
/// their own `Option<&[bool; N]>`/`Option<&[bool]>` view over it.
fn prepare_ap_llr(llr: &[f32], opts: &FecOpts<'_>) -> ([f32; LDPC_N], Option<[bool; LDPC_N]>) {
    assert_eq!(llr.len(), LDPC_N, "llr must be {} values", LDPC_N);
    let mut llr_arr = [0f32; LDPC_N];
    llr_arr.copy_from_slice(llr);

    let ap_mask = match opts.ap_mask {
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
            Some(a)
        }
        None => None,
    };
    (llr_arr, ap_mask)
}
