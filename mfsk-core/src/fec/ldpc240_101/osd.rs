//! Ordered Statistics Decoder (OSD) for LDPC(240, 101). Mirrors the
//! LDPC(174, 91) variant in `super::ldpc::osd`; only the code
//! dimensions, generator sub-matrix and CRC check differ.

use super::bp::check_crc24;
use super::tables::GEN_PARITY;
use super::{LDPC_K, LDPC_N};

/// Systematic encode of 101 info bits into a 240-bit codeword.
pub fn ldpc_encode(info: &[u8; LDPC_K]) -> [u8; LDPC_N] {
    let mut cw = [0u8; LDPC_N];
    cw[..LDPC_K].copy_from_slice(info);
    for (j, row) in GEN_PARITY.iter().enumerate() {
        let mut p = 0u8;
        for (k, &g) in row.iter().enumerate() {
            p ^= g & info[k];
        }
        cw[LDPC_K + j] = p;
    }
    cw
}

pub struct OsdResult {
    pub message77: [u8; 77],
    pub codeword: [u8; LDPC_N],
    pub hard_errors: u32,
}

pub fn osd_decode(llr: &[f32; LDPC_N]) -> Option<OsdResult> {
    osd_decode_impl(llr, 2, LDPC_K)
}

pub fn osd_decode_deep(llr: &[f32; LDPC_N], ndeep: u8) -> Option<OsdResult> {
    osd_decode_impl(llr, ndeep, LDPC_K)
}

fn osd_decode_impl(llr: &[f32; LDPC_N], ndeep: u8, k4_limit: usize) -> Option<OsdResult> {
    // Sort bit indices by |LLR| descending.
    let mut perm = [0usize; LDPC_N];
    for (i, slot) in perm.iter_mut().enumerate() {
        *slot = i;
    }
    perm.sort_unstable_by(|&a, &b| {
        llr[b]
            .abs()
            .partial_cmp(&llr[a].abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Build permuted generator matrix G'[101][240] on the heap (2D boxed
    // slice keeps the 24 kB off the stack).
    let mut g: Box<[[u8; LDPC_N]; LDPC_K]> = vec![[0u8; LDPC_N]; LDPC_K]
        .into_boxed_slice()
        .try_into()
        .ok()?;
    for row in 0..LDPC_K {
        for col in 0..LDPC_N {
            let j = perm[col];
            g[row][col] = if j < LDPC_K {
                (row == j) as u8
            } else {
                GEN_PARITY[j - LDPC_K][row]
            };
        }
    }

    // GF(2) Gaussian elimination to reduced row echelon form.
    let mut pivot_col = [0usize; LDPC_K];
    let mut pivot_row = 0usize;

    for col in 0..LDPC_N {
        if pivot_row >= LDPC_K {
            break;
        }
        let found = (pivot_row..LDPC_K).find(|&r| g[r][col] != 0);
        if let Some(r) = found {
            if r != pivot_row {
                for c in 0..LDPC_N {
                    let tmp = g[r][c];
                    g[r][c] = g[pivot_row][c];
                    g[pivot_row][c] = tmp;
                }
            }
            for r2 in 0..LDPC_K {
                if r2 != pivot_row && g[r2][col] != 0 {
                    for c in 0..LDPC_N {
                        g[r2][c] ^= g[pivot_row][c];
                    }
                }
            }
            pivot_col[pivot_row] = col;
            pivot_row += 1;
        }
    }

    if pivot_row < LDPC_K {
        return None;
    }

    let mut mrb = [0u8; LDPC_K];
    for r in 0..LDPC_K {
        let orig = perm[pivot_col[r]];
        mrb[r] = if llr[orig] > 0.0 { 1 } else { 0 };
    }

    let mut c_perm = [0u8; LDPC_N];
    for r in 0..LDPC_K {
        if mrb[r] == 1 {
            for col in 0..LDPC_N {
                c_perm[col] ^= g[r][col];
            }
        }
    }

    let try_candidate = |cp: &[u8; LDPC_N]| -> Option<([u8; LDPC_K], [u8; LDPC_N], f32)> {
        let mut c = [0u8; LDPC_N];
        for col in 0..LDPC_N {
            c[perm[col]] = cp[col];
        }
        let mut decoded = [0u8; LDPC_K];
        decoded.copy_from_slice(&c[..LDPC_K]);
        if !check_crc24(&decoded) {
            return None;
        }
        let mut wd = 0.0f32;
        for col in 0..LDPC_N {
            let hard = if llr[perm[col]] > 0.0 { 1u8 } else { 0u8 };
            if cp[col] != hard {
                wd += llr[perm[col]].abs();
            }
        }
        Some((decoded, c, wd))
    };

    let mut best: Option<([u8; LDPC_K], [u8; LDPC_N], f32)> = None;

    let mut update_best = |decoded: [u8; LDPC_K], cw: [u8; LDPC_N], wd: f32| {
        let improve = best.as_ref().is_none_or(|(_, _, bd)| wd < *bd);
        if improve {
            best = Some((decoded, cw, wd));
        }
    };

    if let Some((d, cw, wd)) = try_candidate(&c_perm) {
        update_best(d, cw, wd);
    }

    for k1 in 0..LDPC_K {
        let mut c1 = c_perm;
        for col in 0..LDPC_N {
            c1[col] ^= g[k1][col];
        }
        if let Some((d, cw, wd)) = try_candidate(&c1) {
            update_best(d, cw, wd);
        }
        if ndeep < 2 {
            continue;
        }
        for k2 in (k1 + 1)..LDPC_K {
            let mut c2 = c1;
            for col in 0..LDPC_N {
                c2[col] ^= g[k2][col];
            }
            if let Some((d, cw, wd)) = try_candidate(&c2) {
                update_best(d, cw, wd);
            }
            if ndeep < 3 {
                continue;
            }
            for k3 in (k2 + 1)..LDPC_K {
                let mut c3 = c2;
                for col in 0..LDPC_N {
                    c3[col] ^= g[k3][col];
                }
                if let Some((d, cw, wd)) = try_candidate(&c3) {
                    update_best(d, cw, wd);
                }
                if ndeep >= 4 && k3 + 1 < k4_limit {
                    for k4 in (k3 + 1)..k4_limit.min(LDPC_K) {
                        let mut c4 = c3;
                        for col in 0..LDPC_N {
                            c4[col] ^= g[k4][col];
                        }
                        if let Some((d, cw, wd)) = try_candidate(&c4) {
                            update_best(d, cw, wd);
                        }
                    }
                }
            }
        }
    }

    let (decoded, codeword, _) = best?;
    let hard_errors = codeword
        .iter()
        .zip(llr.iter())
        .filter(|&(&b, &l)| (b == 1) != (l > 0.0))
        .count() as u32;
    let mut message77 = [0u8; 77];
    message77.copy_from_slice(&decoded[..77]);
    Some(OsdResult {
        message77,
        hard_errors,
        codeword,
    })
}
