//! Belief-Propagation (log-domain) decoder for the LDPC (174, 91) code.
//! Ported from WSJT-X bpdecode174_91.f90.

use super::tables::{MN, NM, NRW};
use super::{LDPC_K, LDPC_M, LDPC_N};

/// Number of check nodes per bit (constant in this code).
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
/// Accepts any `&[u8]` slice; lengths other than [`LDPC_K`] (= 91) are
/// rejected so the function is suitable as a `MessageCodec::verify_info`
/// implementation passed through `FecOpts::verify_info`.
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
pub struct BpResult {
    /// Decoded 77-bit message payload.
    pub message77: [u8; 77],
    /// Full 174-bit codeword (hard decisions).
    pub codeword: [u8; LDPC_N],
    /// Number of hard errors (bits where hard decision disagrees with LLR sign).
    pub hard_errors: u32,
    /// Number of BP iterations executed.
    pub iterations: u32,
}

/// Log-domain Belief-Propagation decode.
///
/// `llr[i]` follows the convention: positive = bit likely 1, negative
/// = bit likely 0. `ap_mask`: optional slice where `true` means the
/// bit LLR is trusted as-is (a-priori known bit — used for AP-assisted
/// decoding passes in WSJT-X).
///
/// `verify` is an optional integrity check applied to each parity-
/// converged candidate. When `Some`, BP keeps iterating past a
/// parity-only convergence whose verification fails (mirroring how
/// CRC-aware codecs behave under noise that leaves multiple valid
/// codewords near the LLR estimate). When `None`, BP returns on first
/// parity convergence — the right behaviour for codecs whose message
/// codec carries no internal integrity field (e.g.
/// [`crate::msg::PacketBytesMessage`] in `uvpacket`).
///
/// Returns `Some(BpResult)` once parity (and any verifier) accepts a
/// candidate, `None` if the iteration limit is exhausted first.
pub fn bp_decode(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    let mut tov = [[0f32; NCW]; LDPC_N];
    let mut toc = [[0f32; 7]; LDPC_M];
    let mut tanhtoc = [[0f32; 7]; LDPC_M];
    let mut zn = [0f32; LDPC_N];
    let mut cw = [0u8; LDPC_N];

    for j in 0..LDPC_M {
        for i in 0..NRW[j] as usize {
            toc[j][i] = llr[NM[j][i] as usize];
        }
    }

    let mut ncnt = 0u32;
    let mut nclast = 0u32;

    for iter in 0..=max_iter {
        for i in 0..LDPC_N {
            let ap = ap_mask.is_some_and(|m| m[i]);
            if !ap {
                let sum_tov: f32 = tov[i].iter().sum();
                zn[i] = llr[i] + sum_tov;
            } else {
                zn[i] = llr[i];
            }
        }

        for i in 0..LDPC_N {
            cw[i] = if zn[i] > 0.0 { 1 } else { 0 };
        }

        let mut ncheck = 0u32;
        for i in 0..LDPC_M {
            let n = NRW[i] as usize;
            let parity: u8 = NM[i][..n].iter().map(|&b| cw[b as usize]).sum::<u8>() % 2;
            if parity != 0 {
                ncheck += 1;
            }
        }

        if ncheck == 0 {
            let mut decoded = [0u8; LDPC_K];
            decoded.copy_from_slice(&cw[..LDPC_K]);
            // No verifier → accept any parity-converged candidate.
            // With a verifier (e.g. CRC-14 for Wsjt77 protocols) →
            // accept only when the verifier returns true.
            let accept = match verify {
                Some(f) => f(&decoded),
                None => true,
            };
            if accept {
                let hard_errors = cw
                    .iter()
                    .zip(llr.iter())
                    .filter(|&(&b, &l)| (b == 1) != (l > 0.0))
                    .count() as u32;
                let mut message77 = [0u8; 77];
                message77.copy_from_slice(&decoded[..77]);
                return Some(BpResult {
                    message77,
                    codeword: cw,
                    hard_errors,
                    iterations: iter,
                });
            }
        }

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

        for j in 0..LDPC_M {
            for i in 0..NRW[j] as usize {
                let ibj = NM[j][i] as usize;
                let mut msg = zn[ibj];
                for kk in 0..NCW {
                    if MN[ibj][kk] as usize == j {
                        msg -= tov[ibj][kk];
                    }
                }
                toc[j][i] = msg;
            }
        }

        for i in 0..LDPC_M {
            for k in 0..NRW[i] as usize {
                tanhtoc[i][k] = (-toc[i][k] / 2.0).tanh();
            }
        }

        for j in 0..LDPC_N {
            for k in 0..NCW {
                let ichk = MN[j][k] as usize;
                let n = NRW[ichk] as usize;
                let tmn: f32 = NM[ichk][..n]
                    .iter()
                    .zip(tanhtoc[ichk][..n].iter())
                    .filter(|&(&b, _)| b as usize != j)
                    .map(|(_, &t)| t)
                    .product();
                tov[j][k] = 2.0 * platanh(-tmn);
            }
        }
    }

    None
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
