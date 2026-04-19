//! Belief-Propagation decoder for LDPC(240, 101) + CRC-24. Algorithm
//! is identical to the LDPC(174, 91) variant in `super::ldpc::bp`; only
//! the code parameters and row-weight maximum differ.

use super::tables::{MN, NM, NRW};
use super::{LDPC_K, LDPC_M, LDPC_N};

/// Column weight (variable-node degree) — every bit participates in
/// exactly 3 parity checks, same as LDPC(174, 91).
const NCW: usize = 3;
/// Maximum row weight across the 139 check nodes (all NRW entries are
/// either 5 or 6; allocate for the upper bound).
const MAX_ROW: usize = 6;

#[inline]
fn platanh(x: f32) -> f32 {
    if x.abs() > 0.999_999_9 {
        x.signum() * 4.6
    } else {
        x.atanh()
    }
}

/// CRC-24Q as used by WSJT-X FST4: polynomial 0x100065B, applied bit-
/// serially over the message padded with 24 zeros.
///
/// Matches the `get_crc24` subroutine in WSJT-X `lib/fst4/get_crc24.f90`.
pub fn crc24(bits: &[u8]) -> u32 {
    // Working register holds the current 25-bit residual (MSB at index 0).
    let mut r = [0u8; 25];
    // Preload the first 25 bits, padded with zero if the input is shorter.
    for (i, slot) in r.iter_mut().enumerate() {
        *slot = if i < bits.len() { bits[i] & 1 } else { 0 };
    }
    // Polynomial bits (MSB first, x^24..x^0), matching the Fortran array.
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
        // cshift(r, 1): rotate left.
        let first = r[0];
        for k in 0..24 {
            r[k] = r[k + 1];
        }
        r[24] = first;
    }
    // Bits 0..24 of r (MSB first) form the CRC.
    let mut v = 0u32;
    for &b in &r[..24] {
        v = (v << 1) | (b as u32);
    }
    v
}

/// Verify CRC-24 for a 101-bit decoded word (77 msg + 24 CRC).
pub fn check_crc24(decoded: &[u8; LDPC_K]) -> bool {
    // WSJT-X style: run CRC over the full 101-bit word with the CRC slot
    // zeroed. The result should be zero for a consistent receive.
    let mut with_zero = [0u8; LDPC_K];
    with_zero[..77].copy_from_slice(&decoded[..77]);
    // Upper 24 bits (77..101) stay zero.
    let expected = crc24(&with_zero);

    // Compare against the received CRC field (bits 77..101, MSB first).
    let mut got = 0u32;
    for &b in &decoded[77..101] {
        got = (got << 1) | (b as u32 & 1);
    }
    expected == got
}

/// Output of a successful BP decode.
pub struct BpResult {
    pub message77: [u8; 77],
    pub codeword: [u8; LDPC_N],
    pub hard_errors: u32,
    pub iterations: u32,
}

pub fn bp_decode(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
) -> Option<BpResult> {
    let mut tov = [[0f32; NCW]; LDPC_N];
    let mut toc = [[0f32; MAX_ROW]; LDPC_M];
    let mut tanhtoc = [[0f32; MAX_ROW]; LDPC_M];
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
            if check_crc24(&decoded) {
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
            if ncnt >= 5 && iter >= 10 && ncheck > 20 {
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
    fn crc24_zero_bits_returns_zero() {
        // CRC of all-zero input over LDPC_K bits is zero.
        let bits = [0u8; LDPC_K];
        assert_eq!(crc24(&bits), 0);
    }

    #[test]
    fn crc24_round_trip() {
        // Build 77-bit payload + 24-bit CRC, then `check_crc24` accepts.
        let mut msg = [0u8; LDPC_K];
        // Arbitrary nonzero payload.
        for i in 0..77 {
            msg[i] = ((i * 13) & 1) as u8;
        }
        let crc = crc24(&msg); // Upper 24 bits still zero.
        for i in 0..24 {
            msg[77 + i] = ((crc >> (23 - i)) & 1) as u8;
        }
        assert!(check_crc24(&msg));
    }
}
