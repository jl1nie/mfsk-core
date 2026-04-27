//! Belief-Propagation (log-domain) decoder, generic over [`LdpcParams`].
//!
//! Originally ported from WSJT-X `bpdecode174_91.f90`. Phase 0c-B
//! generalised the algorithm so [`Ldpc174_91`](super::Ldpc174_91) and
//! [`Ldpc240_101`](super::Ldpc240_101) share a single implementation;
//! the matrix shape comes from `P` at compile time.
//!
//! For backward compatibility (FT8's bespoke decode path goes through
//! [`bp_decode`] directly), this module also exposes a non-generic
//! [`bp_decode`] that pins `P = Ldpc174_91Params` — same behaviour as
//! before, just routed through the generic body.
//!
//! [`Ldpc174_91`]: super::Ldpc174_91
//! [`Ldpc240_101`]: super::Ldpc240_101

use super::params::{Ldpc174_91Params, LdpcParams};
use super::{LDPC_K, LDPC_N};

/// Column weight (variable-node degree). Both LDPC codes in this
/// crate are uniform with `NCW = 3`.
const NCW: usize = 3;

/// Clamped atanh to avoid ±∞ near the boundaries.
/// Equivalent to WSJT-X `platanh`.
#[inline]
fn platanh(x: f32) -> f32 {
    if x.abs() > 0.999_999_9 {
        x.signum() * 4.6
    } else {
        x.atanh()
    }
}

/// CRC-14 (polynomial 0x2757) over `data` bytes, processed MSB-first.
/// Matches boost::augmented_crc<14, 0x2757> used in WSJT-X crc14.cpp.
pub fn crc14(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let msb = (crc >> 13) & 1;
            crc = ((crc << 1) | bit as u16) & 0x3FFF;
            if msb != 0 {
                crc ^= 0x2757;
            }
        }
    }
    crc
}

/// Verify CRC-14 for a 91-bit decoded word (77 msg + 14 CRC).
/// Packs bits into 12 bytes (big-endian, MSB first), zeros the CRC field,
/// computes CRC-14, then compares with the stored CRC bits.
///
/// Accepts any `&[u8]` slice; lengths other than 91 are rejected so the
/// function is suitable as a `MessageCodec::verify_info` implementation
/// passed through `FecOpts::verify_info`.
pub fn check_crc14(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut bytes = [0u8; 12];
    for (i, &bit) in decoded[..77].iter().enumerate() {
        let byte_idx = i / 8;
        let bit_pos = 7 - (i % 8);
        bytes[byte_idx] |= (bit & 1) << bit_pos;
    }

    let computed = crc14(&bytes);

    let mut received: u16 = 0;
    for &bit in &decoded[77..91] {
        received = (received << 1) | (bit as u16 & 1);
    }

    computed == received
}

/// Output of a successful BP decode.
///
/// `info` is the systematic prefix (length `P::K`); `codeword` is the
/// full decoded codeword (length `P::N`). Both are heap-allocated so
/// the struct can serve any [`LdpcParams`] without const-generic
/// gymnastics. `message77` exposes the leading 77 bits as a fixed-size
/// array for the Wsjt77-family ergonomics that pre-existing FT8 code
/// relies on; uvpacket-class callers ignore it and read `info`.
pub struct BpResult {
    /// Leading 77 info bits (Wsjt77 message field). Same content as
    /// `info[..77]` — duplicated here for callers that take fixed-size
    /// references.
    pub message77: [u8; 77],
    /// Full systematic info (length `P::K`).
    pub info: Vec<u8>,
    /// Full codeword bits (length `P::N`).
    pub codeword: Vec<u8>,
    /// Number of hard errors (bits where hard decision disagrees with LLR sign).
    pub hard_errors: u32,
    /// Number of BP iterations executed.
    pub iterations: u32,
}

/// Generic log-domain Belief-Propagation decode.
///
/// `llr.len()` and (if present) `ap_mask.len()` must equal `P::N`.
///
/// `verify` is an optional integrity check applied to each parity-
/// converged candidate. When `Some`, BP keeps iterating past a
/// parity-only convergence whose verification fails (mirroring how
/// CRC-aware codecs behave under noise that leaves multiple valid
/// codewords near the LLR estimate). When `None`, BP returns on first
/// parity convergence — appropriate for codecs whose message codec
/// carries no internal integrity field.
pub fn bp_decode_generic<P: LdpcParams>(
    llr: &[f32],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    debug_assert_eq!(llr.len(), P::N, "llr length must equal P::N");
    if let Some(m) = ap_mask {
        debug_assert_eq!(m.len(), P::N, "ap_mask length must equal P::N");
    }

    let n = P::N;
    let m_checks = P::M;
    let k = P::K;
    let max_row = P::MAX_ROW;

    // Heap-allocated working buffers. Sizes:
    //   tov     : N * NCW   (≤ 720 bytes for ldpc240_101)
    //   toc     : M * MAX_ROW
    //   tanhtoc : M * MAX_ROW
    //   zn      : N
    //   cw      : N
    // For both codes the total stays under 8 KB — negligible vs the
    // 30+ BP iterations of inner-loop arithmetic.
    let mut tov = vec![0f32; n * NCW];
    let mut toc = vec![0f32; m_checks * max_row];
    let mut tanhtoc = vec![0f32; m_checks * max_row];
    let mut zn = vec![0f32; n];
    let mut cw = vec![0u8; n];

    // Initial messages: each check node receives the raw LLR for the
    // bits it tests.
    for j in 0..m_checks {
        let nrw_j = P::nrw(j) as usize;
        for i in 0..nrw_j {
            let bit = P::nm(j, i) as usize;
            toc[j * max_row + i] = llr[bit];
        }
    }

    let mut ncnt = 0u32;
    let mut nclast = 0u32;

    for iter in 0..=max_iter {
        // Variable-node update: zn = llr + Σ tov, except AP-locked
        // bits hold their LLR fixed.
        for i in 0..n {
            let ap = ap_mask.is_some_and(|mm| mm[i]);
            if !ap {
                let mut sum = 0.0f32;
                for k_ in 0..NCW {
                    sum += tov[i * NCW + k_];
                }
                zn[i] = llr[i] + sum;
            } else {
                zn[i] = llr[i];
            }
        }

        // Hard decisions.
        for i in 0..n {
            cw[i] = if zn[i] > 0.0 { 1 } else { 0 };
        }

        // Count parity-violating checks.
        let mut ncheck = 0u32;
        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            let mut parity = 0u8;
            for s in 0..nrw_i {
                parity ^= cw[P::nm(i, s) as usize];
            }
            if parity != 0 {
                ncheck += 1;
            }
        }

        if ncheck == 0 {
            let mut decoded = vec![0u8; k];
            decoded.copy_from_slice(&cw[..k]);
            // No verifier → accept any parity-converged candidate.
            // With a verifier (e.g. CRC-14/24 length-dispatched in
            // Wsjt77Message::verify_info) → accept only on true.
            let accept = match verify {
                Some(f) => f(&decoded),
                None => true,
            };
            if accept {
                let mut hard_errors = 0u32;
                for i in 0..n {
                    if (cw[i] == 1) != (llr[i] > 0.0) {
                        hard_errors += 1;
                    }
                }
                let mut message77 = [0u8; 77];
                message77.copy_from_slice(&decoded[..77]);
                return Some(BpResult {
                    message77,
                    info: decoded,
                    codeword: cw,
                    hard_errors,
                    iterations: iter,
                });
            }
        }

        // Stall detector: same heuristic as the WSJT-X reference.
        if iter > 0 {
            if ncheck < nclast {
                ncnt = 0;
            } else {
                ncnt += 1;
            }
            if ncnt >= 5 && iter >= 10 && ncheck > 15 {
                return None;
            }
        }
        nclast = ncheck;

        // Check-to-variable message update (extrinsic info).
        for j in 0..m_checks {
            let nrw_j = P::nrw(j) as usize;
            for i in 0..nrw_j {
                let ibj = P::nm(j, i) as usize;
                let mut msg = zn[ibj];
                let mn_ibj = P::mn(ibj);
                for kk in 0..NCW {
                    if mn_ibj[kk] as usize == j {
                        msg -= tov[ibj * NCW + kk];
                    }
                }
                toc[j * max_row + i] = msg;
            }
        }

        // tanh half-message cache.
        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            for k_ in 0..nrw_i {
                tanhtoc[i * max_row + k_] = (-toc[i * max_row + k_] / 2.0).tanh();
            }
        }

        // Variable-to-check message update.
        for j in 0..n {
            let mn_j = P::mn(j);
            for k_ in 0..NCW {
                let ichk = mn_j[k_] as usize;
                let nrw_ichk = P::nrw(ichk) as usize;
                let mut tmn = 1.0f32;
                for s in 0..nrw_ichk {
                    let bit = P::nm(ichk, s) as usize;
                    if bit != j {
                        tmn *= tanhtoc[ichk * max_row + s];
                    }
                }
                tov[j * NCW + k_] = 2.0 * platanh(-tmn);
            }
        }
    }

    None
}

/// Backward-compatible LDPC(174,91) BP decode — pins
/// [`bp_decode_generic`] to [`Ldpc174_91Params`]. Used by FT8's
/// bespoke decode loop (which still consumes the shared LDPC
/// implementation through `super::ft8::ldpc`'s re-export façade).
///
/// `llr[i]` follows the convention: positive = bit likely 1, negative
/// = bit likely 0. `ap_mask[i] = true` means the bit's LLR is
/// AP-locked and not updated by BP.
pub fn bp_decode(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic::<Ldpc174_91Params>(llr.as_slice(), ap_slice, max_iter, verify)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_perfect_llr_all_zeros() {
        let llr = [10.0f32; 174];
        let _result = bp_decode(&llr, None, 30, None);
    }

    #[test]
    fn crc14_known_vector() {
        assert_eq!(crc14(&[0u8; 12]), 0);
    }
}
