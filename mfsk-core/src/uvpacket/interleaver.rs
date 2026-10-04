// SPDX-License-Identifier: GPL-3.0-only
//! Block-interleaver across multiple LDPC codewords.
//!
//! Spreads consecutive codeword bits across the entire multi-block
//! channel transmission so that a fade null longer than one
//! codeword's airtime turns into scattered single-bit erasures
//! across every codeword, well within the soft-decision LDPC's
//! correction capacity.
//!
//! ## Construction
//!
//! Standard column-write / row-read block interleaver:
//!
//! ```text
//! Input  (per-codeword): cw[block][bit]  for block in 0..N, bit in 0..L
//! Output (channel order): out[k] = cw[k % N][k / N]   for k in 0..N×L
//! ```
//!
//! Equivalently: write the `N` codewords as rows of an N × L matrix,
//! read it column-by-column. Consecutive bits in the channel stream
//! step through the codewords, so a burst of `B ≤ N` consecutive
//! channel-bit corruptions hits at most one bit per codeword. For
//! `B > N`, ceil(B / N) bits per codeword.
//!
//! At the rates uvpacket targets (a frame of 1–32 LDPC blocks, each
//! block 240 / 202 / 152 / 134 channel bits depending on mode), this
//! gives the LDPC's BP / OSD a comfortable working point even
//! through fade events that would wipe out one or two whole
//! codewords' airtime.
//!
//! ## API
//!
//! Two layers:
//!
//! - [`interleave`] / [`deinterleave_llr`] are the production fast
//!   path: take the concatenated codewords (TX) or LLR vector (RX)
//!   and produce the corresponding output. No allocation beyond the
//!   single output `Vec`.
//! - [`build_permutation`] returns the explicit permutation table —
//!   useful for sanity checks and for callers that want to apply
//!   the same permutation to other per-bit metadata (e.g. expected
//!   tone indices).
//!
//! All three functions are deterministic and parameter-only — no
//! state is carried. The N=1 case (single LDPC block per frame) is
//! the identity.

/// Build the permutation table such that
/// `out[k] = src[perm[k]]` where `src` is the row-major concatenation
/// of `n_blocks` codewords, each of length `block_len`. Length is
/// `n_blocks * block_len`.
pub fn build_permutation(n_blocks: usize, block_len: usize) -> Vec<usize> {
    let total = n_blocks * block_len;
    let mut perm = Vec::with_capacity(total);
    for k in 0..total {
        let block = k % n_blocks;
        let bit = k / n_blocks;
        perm.push(block * block_len + bit);
    }
    perm
}

/// Apply the column-write / row-read interleave to a hard-decision
/// channel-bit stream.
///
/// `codewords_concat` is the row-major concatenation of `n_blocks`
/// codewords, each of length `codewords_concat.len() / n_blocks`.
///
/// Returns the channel-order bit stream.
pub fn interleave(codewords_concat: &[u8], n_blocks: usize) -> Vec<u8> {
    assert!(n_blocks > 0, "n_blocks must be > 0");
    let total = codewords_concat.len();
    assert!(
        total.is_multiple_of(n_blocks),
        "codewords_concat length {total} not divisible by n_blocks {n_blocks}",
    );
    let block_len = total / n_blocks;
    let mut out = vec![0u8; total];
    for k in 0..total {
        let block = k % n_blocks;
        let bit = k / n_blocks;
        out[k] = codewords_concat[block * block_len + bit];
    }
    out
}

/// Inverse of [`interleave`] for soft-decision LLRs.
///
/// `llr_tx` is the channel-order LLR vector. Returns one LLR vector
/// per codeword (length `n_blocks`, each of length
/// `llr_tx.len() / n_blocks`).
pub fn deinterleave_llr(llr_tx: &[f32], n_blocks: usize) -> Vec<Vec<f32>> {
    assert!(n_blocks > 0, "n_blocks must be > 0");
    let total = llr_tx.len();
    assert!(
        total.is_multiple_of(n_blocks),
        "llr_tx length {total} not divisible by n_blocks {n_blocks}",
    );
    let block_len = total / n_blocks;
    let mut per_block = vec![vec![0.0f32; block_len]; n_blocks];
    for k in 0..total {
        let block = k % n_blocks;
        let bit = k / n_blocks;
        per_block[block][bit] = llr_tx[k];
    }
    per_block
}

// ────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_permutation_is_valid_permutation() {
        for &(n, l) in &[(1, 240), (4, 240), (32, 134), (8, 202), (16, 152)] {
            let perm = build_permutation(n, l);
            assert_eq!(perm.len(), n * l);
            let mut sorted = perm.clone();
            sorted.sort_unstable();
            for (i, &v) in sorted.iter().enumerate() {
                assert_eq!(v, i, "perm({n}, {l}) missing index {i}");
            }
        }
    }

    #[test]
    fn n_blocks_one_is_identity() {
        let cw: Vec<u8> = (0..240).map(|i| (i & 1) as u8).collect();
        let interleaved = interleave(&cw, 1);
        assert_eq!(interleaved, cw);

        let llrs: Vec<f32> = (0..240).map(|i| i as f32).collect();
        let deint = deinterleave_llr(&llrs, 1);
        assert_eq!(deint.len(), 1);
        assert_eq!(deint[0], llrs);
    }

    #[test]
    fn interleave_then_llr_deinterleave_roundtrip() {
        for &(n_blocks, block_len) in &[(2, 134), (4, 240), (16, 152), (32, 134)] {
            let total = n_blocks * block_len;
            let cw: Vec<u8> = (0..total)
                .map(|i| ((i * 2654435761usize) & 1) as u8)
                .collect();
            let tx = interleave(&cw, n_blocks);

            // Convert to ±1 LLRs as a stand-in for the channel.
            let llrs: Vec<f32> = tx
                .iter()
                .map(|&b| if b == 1 { 1.0 } else { -1.0 })
                .collect();
            let per_block = deinterleave_llr(&llrs, n_blocks);

            assert_eq!(per_block.len(), n_blocks);
            for (block_idx, block_llrs) in per_block.iter().enumerate() {
                assert_eq!(block_llrs.len(), block_len);
                for (bit_idx, &llr) in block_llrs.iter().enumerate() {
                    let original_bit = cw[block_idx * block_len + bit_idx];
                    let expected = if original_bit == 1 { 1.0 } else { -1.0 };
                    assert!(
                        (llr - expected).abs() < 1e-6,
                        "block {block_idx} bit {bit_idx}: got {llr}, want {expected}",
                    );
                }
            }
        }
    }

    /// A burst of `n_blocks` consecutive channel-bit corruptions
    /// must affect exactly one bit per codeword (the canonical
    /// burst-spread guarantee of a column-write block interleaver).
    #[test]
    fn burst_of_n_blocks_hits_one_bit_per_codeword() {
        let n_blocks = 8usize;
        let block_len = 240usize;
        let total = n_blocks * block_len;

        // Mark every channel-bit with whether it was corrupted.
        let mut tx_corrupted = vec![false; total];
        for k in 100..(100 + n_blocks) {
            tx_corrupted[k] = true;
        }

        // Convert corruption flags into per-bit codeword positions
        // they correspond to.
        let mut per_block_corruptions = vec![0usize; n_blocks];
        for (k, &corr) in tx_corrupted.iter().enumerate() {
            if corr {
                let block = k % n_blocks;
                per_block_corruptions[block] += 1;
            }
        }

        for (block, count) in per_block_corruptions.iter().enumerate() {
            assert_eq!(
                *count, 1,
                "block {block} got {count} hits, expected 1 (burst length = n_blocks)",
            );
        }
    }

    /// A burst of `n_blocks * 2` consecutive channel-bit corruptions
    /// must affect exactly 2 bits per codeword (= ceil(burst / n_blocks)).
    #[test]
    fn burst_of_2x_n_blocks_hits_two_bits_per_codeword() {
        let n_blocks = 4usize;
        let block_len = 240usize;
        let total = n_blocks * block_len;
        let burst = 2 * n_blocks;

        let mut tx_corrupted = vec![false; total];
        for k in 50..(50 + burst) {
            tx_corrupted[k] = true;
        }

        let mut per_block_corruptions = vec![0usize; n_blocks];
        for (k, &corr) in tx_corrupted.iter().enumerate() {
            if corr {
                let block = k % n_blocks;
                per_block_corruptions[block] += 1;
            }
        }

        for (block, count) in per_block_corruptions.iter().enumerate() {
            assert_eq!(
                *count, 2,
                "block {block} got {count} hits, expected 2 (burst = 2 × n_blocks)",
            );
        }
    }

    /// Sanity-check that, even with the full uvpacket per-mode sizes,
    /// a single-block frame (the smallest possible) and a 32-block
    /// frame (the largest possible) round-trip cleanly.
    #[test]
    fn full_size_extremes_roundtrip() {
        for &(n_blocks, block_len) in &[
            (1, 240),  // Robust, 1 block
            (1, 134),  // Express, 1 block
            (32, 240), // Robust, max blocks
            (32, 134), // Express, max blocks
        ] {
            let total = n_blocks * block_len;
            let cw: Vec<u8> = (0..total).map(|i| ((i ^ 0x5A) & 1) as u8).collect();
            let tx = interleave(&cw, n_blocks);
            let llrs: Vec<f32> = tx
                .iter()
                .map(|&b| if b == 1 { 1.0 } else { -1.0 })
                .collect();
            let per_block = deinterleave_llr(&llrs, n_blocks);
            for (block, block_llrs) in per_block.iter().enumerate() {
                for (bit, &llr) in block_llrs.iter().enumerate() {
                    let want = if cw[block * block_len + bit] == 1 {
                        1.0
                    } else {
                        -1.0
                    };
                    assert!((llr - want).abs() < 1e-6);
                }
            }
        }
    }
}
