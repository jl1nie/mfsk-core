//! LDPC(240, 101) codec with CRC-24 for the WSJT FST4 / FST4W family.
//!
//! Phase 0c-B unified the implementation: this module no longer carries
//! its own BP / OSD / encode bodies — those live in
//! [`crate::fec::ldpc`] and are parameterised by [`Ldpc240_101Params`].
//! All that remains here are the protocol-specific tables (in [`tables`])
//! and the CRC-24 helpers ([`crc24`], [`check_crc24`]).
//!
//! The public surface is preserved as a type alias:
//! `pub type Ldpc240_101 = LdpcCodec<Ldpc240_101Params>` lives here so
//! callers continue to write `mfsk_core::fec::Ldpc240_101`.

pub mod tables;

use crate::core::{FecCodec, FecOpts, FecResult};
use crate::fec::ldpc::bp::bp_decode_generic;
use crate::fec::ldpc::osd::{ldpc_encode_generic, osd_decode_generic};
use crate::fec::ldpc::params::Ldpc240_101Params;

pub const LDPC_N: usize = 240;
pub const LDPC_K: usize = 101;
pub const LDPC_M: usize = LDPC_N - LDPC_K; // 139

// ────────────────────────────────────────────────────────────────────
// CRC-24

/// CRC-24Q as used by WSJT-X FST4: polynomial 0x100065B, applied bit-
/// serially over the message padded with 24 zeros.
///
/// Matches the `get_crc24` subroutine in WSJT-X `lib/fst4/get_crc24.f90`.
pub fn crc24(bits: &[u8]) -> u32 {
    let mut r = [0u8; 25];
    for (i, slot) in r.iter_mut().enumerate() {
        *slot = if i < bits.len() { bits[i] & 1 } else { 0 };
    }
    const POLY: [u8; 25] = [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1,
    ];
    let n = bits.len().saturating_sub(25);
    for i in 0..=n {
        if i + 25 <= bits.len() {
            r[24] = bits[i + 24] & 1;
        } else {
            r[24] = 0;
        }
        let top = r[0];
        if top != 0 {
            for (rv, pv) in r.iter_mut().zip(POLY.iter()) {
                *rv ^= *pv;
            }
        }
        let first = r[0];
        for k in 0..24 {
            r[k] = r[k + 1];
        }
        r[24] = first;
    }
    let mut v = 0u32;
    for &b in &r[..24] {
        v = (v << 1) | (b as u32);
    }
    v
}

/// Verify CRC-24 for a 101-bit decoded word (77 msg + 24 CRC).
///
/// Accepts any `&[u8]` slice; lengths other than [`LDPC_K`] (= 101) are
/// rejected so the function is suitable as a `MessageCodec::verify_info`
/// implementation passed through `FecOpts::verify_info`.
pub fn check_crc24(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut with_zero = [0u8; LDPC_K];
    with_zero[..77].copy_from_slice(&decoded[..77]);
    let expected = crc24(&with_zero);

    let mut got = 0u32;
    for &b in &decoded[77..101] {
        got = (got << 1) | (b as u32 & 1);
    }
    expected == got
}

// ────────────────────────────────────────────────────────────────────
// FecCodec impl

/// Zero-sized LDPC(240, 101) codec — a thin wrapper that pins the
/// generic [`crate::fec::ldpc::params::LdpcParams`]-based
/// implementation to [`Ldpc240_101Params`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc240_101;

impl FecCodec for Ldpc240_101 {
    const N: usize = LDPC_N;
    const K: usize = LDPC_K;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), LDPC_K, "info must be {} bits", LDPC_K);
        assert_eq!(codeword.len(), LDPC_N, "codeword must be {} bits", LDPC_N);
        ldpc_encode_generic::<Ldpc240_101Params>(info, codeword);
    }

    fn decode_soft(&self, llr: &[f32], opts: &FecOpts<'_>) -> Option<FecResult> {
        assert_eq!(llr.len(), LDPC_N, "llr must be {} values", LDPC_N);
        let mut llr_arr = vec![0f32; LDPC_N];
        llr_arr.copy_from_slice(llr);

        // AP hint injection (same convention as Ldpc174_91): clamp the
        // masked bits to ±apmag where apmag dominates any channel
        // observation, then build a parallel bool mask the BP loop
        // consults to skip variable-node updates on those bits.
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

        if let Some(r) = bp_decode_generic::<Ldpc240_101Params>(
            &llr_arr,
            ap_slice,
            opts.bp_max_iter,
            opts.verify_info,
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

        let r = osd_decode_generic::<Ldpc240_101Params>(
            &llr_arr,
            opts.osd_depth.min(3) as u8,
            LDPC_K,
            opts.verify_info,
        )?;
        Some(FecResult {
            info: r.info,
            hard_errors: r.hard_errors,
            iterations: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::ldpc::bp::bp_decode_generic;
    use crate::fec::ldpc::osd::ldpc_encode_generic;

    /// Round-trip: encode a 101-bit info word, feed perfect LLRs,
    /// decoder should recover the original info. Exercises the full
    /// BP path plus the generator sub-matrix via the generic helpers.
    #[test]
    fn roundtrip_perfect_llr() {
        let mut info = [0u8; LDPC_K];
        for i in 0..77 {
            info[i] = ((i * 7 + 3) & 1) as u8;
        }
        let crc = crc24(&info); // upper 24 bits still zero
        for i in 0..24 {
            info[77 + i] = ((crc >> (23 - i)) & 1) as u8;
        }

        let mut cw = [0u8; LDPC_N];
        ldpc_encode_generic::<Ldpc240_101Params>(&info, &mut cw);
        // Sanity: systematic encode keeps info bits in positions 0..K.
        assert_eq!(&cw[..LDPC_K], &info[..]);

        // Perfect LLR: ±8 per bit, sign follows the bit.
        let mut llr = vec![0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if cw[i] == 1 { 8.0 } else { -8.0 };
        }
        let r = bp_decode_generic::<Ldpc240_101Params>(&llr, None, 30, Some(check_crc24))
            .expect("BP converges on perfect LLR");
        assert_eq!(&r.info[..77], &info[..77]);
    }
}
