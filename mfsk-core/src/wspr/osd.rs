//! Ordered-Statistics Decoding (OSD) for the WSPR K=32 r=½
//! convolutional code. Last-resort fallback when Fano fails to
//! converge on weak signals (e.g. W3BI at -27 dB on the WSJT-X
//! golden). Faithful port of `wsprd/osdwspr.f90` order-1 path.
//!
//! Algorithm:
//! 1. Hard-decision the received soft codeword; reliability = |LLR|.
//! 2. Sort coded-bit columns by reliability (desc).
//! 3. Gaussian-eliminate the sorted generator matrix `G` so its first
//!    K columns form the identity (the "most reliable basis", MRB).
//! 4. Take the K hardest-decided bits as the order-0 message `m0`.
//!    Encode → order-0 codeword `c0`, distance `d0 = Σ |LLR| · err`.
//! 5. Order-1: flip each of the K MRB bits one at a time, re-encode,
//!    keep the codeword with smallest distance.
//! 6. Convert the recovered codeword's K-bit message back to the 50
//!    info bits via re-running Fano on the hard codeword (zero hard
//!    errors → trivial Fano convergence).
//!
//! We stop at order-1 (no preprocessing rules `npre1`/`npre2`).
//! On the WSJT-X golden sample this is enough to recover W3BI at
//! -27 dB SNR while keeping per-call cost under a few ms.

use alloc::vec;

use crate::engine::{FecCodec, FecOpts};
use crate::fec::ConvFano;

const N: usize = 162;
const K: usize = 50;

/// Generator matrix `G[K][N]`: row `i` = the codeword produced by
/// encoding the unit vector with `1` at position `i`. K=50 info bits
/// → N=162 coded bits via the K=32 r=½ Layland-Lushbaugh polynomial.
///
/// Built lazily on first `osd_decode` call from `ConvFano::encode`,
/// then frozen in a `OnceLock`. Saves recomputing on every call.
fn gen_matrix() -> &'static [[u8; N]; K] {
    use std::sync::OnceLock;
    static GEN: OnceLock<[[u8; N]; K]> = OnceLock::new();
    GEN.get_or_init(|| {
        let codec = ConvFano;
        let mut g = [[0u8; N]; K];
        let mut info = [0u8; K];
        let mut codeword = vec![0u8; N];
        for i in 0..K {
            info.fill(0);
            info[i] = 1;
            codec.encode(&info, &mut codeword);
            g[i].copy_from_slice(&codeword);
        }
        g
    })
}

/// OSD decode: returns the 50-bit info **and** the number of hard
/// errors between the recovered codeword and the input hard
/// decisions. Caller should reject results with too many hard errors
/// (typical threshold: `nhardmin ≤ 36..56` of 162) since OSD will
/// always synthesise *some* valid codeword from any input — the hard-
/// error count is the only signal of how plausible that codeword is.
///
/// `llrs` are deinterleaved soft symbols (positive → bit 0 more
/// likely, matches our Fano sign convention). Returns `None` if
/// Gaussian elimination fails (rare, signals a degenerate reliability
/// ordering) or the info-extraction Fano round-trip fails.
pub fn osd_decode(llrs: &[f32; N]) -> Option<([u8; K], u32)> {
    // Hard decisions and reliability magnitudes. Our LLR sign is
    // "positive = bit 0", so hard bit = (llr < 0) as u8 (bit 1 when
    // negative).
    let mut hard = [0u8; N];
    let mut absllr = [0.0f32; N];
    for i in 0..N {
        hard[i] = if llrs[i] < 0.0 { 1 } else { 0 };
        absllr[i] = llrs[i].abs();
    }

    // Sort indices by reliability descending.
    let mut indices: [usize; N] = core::array::from_fn(|i| i);
    indices.sort_by(|&a, &b| {
        absllr[b]
            .partial_cmp(&absllr[a])
            .unwrap_or(core::cmp::Ordering::Equal)
    });

    // Reorder G's columns and the hard/absllr vectors by `indices`.
    let g0 = gen_matrix();
    let mut g: [[u8; N]; K] = [[0u8; N]; K];
    let mut hard_perm = [0u8; N];
    let mut abs_perm = [0.0f32; N];
    let mut perm = indices;
    for (col_out, &col_src) in indices.iter().enumerate() {
        for row in 0..K {
            g[row][col_out] = g0[row][col_src];
        }
        hard_perm[col_out] = hard[col_src];
        abs_perm[col_out] = absllr[col_src];
    }

    // Gaussian elimination: make the leftmost K columns of `g` the
    // identity matrix (the MRB). If a pivot can't be found in column
    // `id` within columns [id, K+20), bail out — the code is too
    // degenerate at this reliability ordering.
    for id in 0..K {
        let mut pivot_col = None;
        for icol in id..(K + 20).min(N) {
            if g[id][icol] == 1 {
                pivot_col = Some(icol);
                break;
            }
        }
        let icol = pivot_col?;
        if icol != id {
            // Swap columns `id` and `icol` across all rows.
            for row in 0..K {
                g[row].swap(id, icol);
            }
            hard_perm.swap(id, icol);
            abs_perm.swap(id, icol);
            perm.swap(id, icol);
        }
        // Eliminate other rows that have 1 in column `id`.
        for row in 0..K {
            if row != id && g[row][id] == 1 {
                for c in 0..N {
                    g[row][c] ^= g[id][c];
                }
            }
        }
    }

    // Order-0 message = first K bits of permuted hard decisions.
    let mut m0 = [0u8; K];
    m0.copy_from_slice(&hard_perm[..K]);

    let encode = |me: &[u8; K]| -> [u8; N] {
        let mut cw = [0u8; N];
        // After Gaussian elimination, columns 0..K are identity, so
        // cw[0..K] = me. cw[K..N] = Σ_i (me[i] · g[i][K..N]).
        cw[..K].copy_from_slice(me);
        for i in 0..K {
            if me[i] == 1 {
                for c in K..N {
                    cw[c] ^= g[i][c];
                }
            }
        }
        cw
    };

    let distance = |cw: &[u8; N]| -> f32 {
        let mut d = 0.0f32;
        for i in 0..N {
            if cw[i] != hard_perm[i] {
                d += abs_perm[i];
            }
        }
        d
    };

    let c0 = encode(&m0);
    let mut best_d = distance(&c0);
    let mut best_cw = c0;

    // Order-1: flip each of the K MRB bits. K = 50 trials.
    for n1 in 0..K {
        let mut me = m0;
        me[n1] ^= 1;
        let cw = encode(&me);
        let d = distance(&cw);
        if d < best_d {
            best_d = d;
            best_cw = cw;
        }
    }

    // Order-2: flip pairs of MRB bits. C(50,2) = 1225 trials. wsprd's
    // `osdwspr.f90` uses up to nord=3 (with `ndeep=5`); order-2 alone
    // covers most weak-signal recovery on the WSPR golden WAV without
    // the runtime cost of order-3 (≈ 19 600 trials).
    for n1 in 0..K {
        for n2 in (n1 + 1)..K {
            let mut me = m0;
            me[n1] ^= 1;
            me[n2] ^= 1;
            let cw = encode(&me);
            let d = distance(&cw);
            if d < best_d {
                best_d = d;
                best_cw = cw;
            }
        }
    }

    // Un-permute the codeword back to natural coded-bit order.
    let mut cw_natural = [0u8; N];
    for (perm_i, &orig_i) in perm.iter().enumerate() {
        cw_natural[orig_i] = best_cw[perm_i];
    }

    // Recover info bits: build hard-decision LLRs from the codeword
    // (±127 magnitude) and run Fano. Convergence is trivial since
    // the codeword has zero hard errors against itself.
    let mut hard_llrs = [0.0f32; N];
    for i in 0..N {
        hard_llrs[i] = if cw_natural[i] == 0 { 127.0 } else { -127.0 };
    }
    let codec = ConvFano;
    let res = codec.decode_soft(&hard_llrs, &FecOpts::default())?;
    if res.hard_errors > 0 {
        // OSD output didn't match a valid codeword — give up.
        return None;
    }
    let mut info = [0u8; K];
    info.copy_from_slice(&res.info);
    // nhardmin: bits where the recovered codeword disagrees with the
    // original hard decisions. Compute in NATURAL order (cw_natural
    // vs hard) — both vectors are in coded-bit order at this point.
    let mut nhardmin = 0u32;
    for i in 0..N {
        if cw_natural[i] != hard[i] {
            nhardmin += 1;
        }
    }
    Some((info, nhardmin))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn osd_runs_without_panic() {
        // Spot-check that osd_decode handles arbitrary LLRs without
        // panicking. Recall behaviour is exercised by the WSJT-X
        // golden-recall integration test (`wspr_wsjtx_samples`),
        // which is the only practical way to validate end-to-end OSD
        // correctness for a non-systematic convolutional code.
        let llrs = [1.0f32; N];
        let _ = osd_decode(&llrs); // None or Some — both fine
    }
}
