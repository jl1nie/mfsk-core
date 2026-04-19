//! Fano sequential decoder for a rate-1/2, K=32 convolutional code.
//!
//! Ported from WSJT-X `lib/wsprd/fano.c` (Phil Karn KA9Q 1994, minor
//! modifications K1JT). The algorithm is unchanged; data structures use
//! `i32` metrics and `u32` encoder state to match the C implementation
//! precisely.
//!
//! ## Generator polynomials
//!
//! Currently only the Layland–Lushbaugh code is wired (the one WSPR uses):
//! `POLY1 = 0xf2d0_5351`, `POLY2 = 0xe461_3c47`. Both polynomials have odd
//! parity, so the two branch symbols for a given encoder state are always
//! complementary — this is what lets the hot loop XOR `encstate^1` instead
//! of calling the encoder twice.
//!
//! ## Metric table
//!
//! `mettab[bit][sym]` is the log-likelihood of receiving the quantised
//! symbol `sym` (0..=255) when the transmitter sent `bit`. Callers pass in
//! LLRs per coded bit; `build_mettab_from_llrs` shapes them to match.

/// Generator polynomial 1 — Layland–Lushbaugh r=1/2 K=32.
pub const POLY1: u32 = 0xf2d0_5351;
/// Generator polynomial 2 — Layland–Lushbaugh r=1/2 K=32.
pub const POLY2: u32 = 0xe461_3c47;

/// Constraint length: number of bits the encoder remembers.
pub const K_CONSTRAINT: usize = 32;

/// Compute the two output bits (packed `{b_poly1 << 1 | b_poly2}`) for a
/// given encoder state. Mirrors the `ENCODE` macro in WSJT-X `fano.h`.
#[inline]
pub fn encode_step(encstate: u32) -> u32 {
    // Parity of (encstate & POLYn) — XOR the four 8-bit nibbles together
    // and look up the final 8-bit parity.
    let p1 = {
        let mut t = encstate & POLY1;
        t ^= t >> 16;
        t ^= t >> 8;
        (t & 0xff).count_ones() & 1
    };
    let p2 = {
        let mut t = encstate & POLY2;
        t ^= t >> 16;
        t ^= t >> 8;
        (t & 0xff).count_ones() & 1
    };
    (p1 << 1) | p2
}

/// Convolutionally encode `nbits` message bits (MSB-first in `data`) into a
/// `2 * nbits`-long symbol stream (one bit per byte). Matches
/// `fano.c::encode`.
pub fn conv_encode(data: &[u8], nbits: usize, out: &mut [u8]) {
    assert!(out.len() >= 2 * nbits, "output buffer too small");
    let mut encstate: u32 = 0;
    for i in 0..nbits {
        let bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        encstate = (encstate << 1) | bit as u32;
        let sym = encode_step(encstate);
        out[2 * i] = ((sym >> 1) & 1) as u8;
        out[2 * i + 1] = (sym & 1) as u8;
    }
}

/// Build per-coded-bit metric pair `(m_if_sent_0, m_if_sent_1)` from LLRs.
///
/// Convention: positive LLR ⇒ bit 0 is more likely. We use the max-log-MAP
/// approximation `m(b=0) = +llr/2`, `m(b=1) = -llr/2`, subtract a fixed
/// Fano bias, and quantise to i32 with a constant scale so the decoder's
/// threshold stepping has enough resolution.
pub fn build_branch_metrics(llrs: &[f32], bias: f32, scale: f32) -> Vec<[i32; 2]> {
    llrs.iter()
        .map(|&l| {
            let m0 = l * 0.5 - bias;
            let m1 = -l * 0.5 - bias;
            [(m0 * scale).round() as i32, (m1 * scale).round() as i32]
        })
        .collect()
}

struct Node {
    encstate: u32,
    gamma: i64, // path metric, accumulated
    metrics: [i32; 4],
    tm: [i32; 2],
    i: u8, // 0 or 1 — which hypothesis we're currently exploring
}

impl Default for Node {
    fn default() -> Self {
        Self {
            encstate: 0,
            gamma: 0,
            metrics: [0; 4],
            tm: [0; 2],
            i: 0,
        }
    }
}

/// Outcome of a Fano decode.
pub struct FanoDecodeResult {
    /// Recovered message bits, MSB-first. Length = `nbits.div_ceil(8)` bytes
    /// covering `nbits` bits.
    pub data: Vec<u8>,
    /// Final path metric at the accept node (larger is better).
    pub metric: i64,
    /// Cycles consumed. If `cycles > maxcycles * nbits`, decode timed out.
    pub cycles: u64,
    /// Deepest node reached during the search.
    pub max_np: usize,
    /// `true` iff the decoder terminated at the target depth before timeout.
    pub converged: bool,
}

/// Fano sequential decoder. `branch_metrics[i]` is `[m_if_0, m_if_1]` for the
/// i-th coded-bit position (`branch_metrics.len() == 2 * nbits`). The last
/// `K_CONSTRAINT - 1` input bits (the "tail") are assumed to be zero — the
/// decoder exploits that to prune the 1-branch.
pub fn fano_decode(
    branch_metrics: &[[i32; 2]],
    nbits: usize,
    delta: i32,
    max_cycles_per_bit: u64,
) -> FanoDecodeResult {
    assert_eq!(
        branch_metrics.len(),
        2 * nbits,
        "branch_metrics length mismatch"
    );

    let mut nodes: Vec<Node> = (0..=nbits).map(|_| Node::default()).collect();

    // Precompute all 4 branch-metric sums per node position.
    for (k, node) in nodes.iter_mut().take(nbits).enumerate() {
        let a = branch_metrics[2 * k];
        let b = branch_metrics[2 * k + 1];
        node.metrics[0] = a[0] + b[0]; // sent (0,0)
        node.metrics[1] = a[0] + b[1]; // sent (0,1)
        node.metrics[2] = a[1] + b[0]; // sent (1,0)
        node.metrics[3] = a[1] + b[1]; // sent (1,1)
    }

    let last_idx = nbits.saturating_sub(1);
    let tail_idx = nbits.saturating_sub(K_CONSTRAINT - 1);

    // Bootstrap the root node's best/worst branch.
    {
        let lsym = encode_step(0) as usize;
        let m0 = nodes[0].metrics[lsym];
        let m1 = nodes[0].metrics[3 ^ lsym];
        if m0 > m1 {
            nodes[0].tm = [m0, m1];
        } else {
            nodes[0].tm = [m1, m0];
            nodes[0].encstate |= 1;
        }
        nodes[0].i = 0;
        nodes[0].gamma = 0;
    }

    let max_cycles = max_cycles_per_bit * nbits as u64;
    let mut np: usize = 0;
    let mut t: i32 = 0;
    let mut max_np: usize = 0;
    let mut cycles: u64 = 0;
    let mut converged = false;

    while cycles < max_cycles {
        cycles += 1;
        if np > max_np {
            max_np = np;
        }

        // Look forward.
        let ngamma = nodes[np].gamma + nodes[np].tm[nodes[np].i as usize] as i64;
        if ngamma >= t as i64 {
            if nodes[np].gamma < (t as i64) + delta as i64 {
                // First visit — tighten threshold.
                while ngamma >= (t as i64) + delta as i64 {
                    t += delta;
                }
            }
            let new_state = nodes[np].encstate << 1;
            let new_idx = np + 1;
            nodes[new_idx].gamma = ngamma;
            nodes[new_idx].encstate = new_state;
            np = new_idx;
            if np > last_idx {
                converged = true;
                break;
            }
            let lsym = encode_step(nodes[np].encstate) as usize;
            if np >= tail_idx {
                // Tail is all zeros — only the 0-branch is valid.
                nodes[np].tm[0] = nodes[np].metrics[lsym];
                nodes[np].tm[1] = i32::MIN / 2; // never chosen
            } else {
                let m0 = nodes[np].metrics[lsym];
                let m1 = nodes[np].metrics[3 ^ lsym];
                if m0 > m1 {
                    nodes[np].tm = [m0, m1];
                } else {
                    nodes[np].tm = [m1, m0];
                    nodes[np].encstate |= 1;
                }
            }
            nodes[np].i = 0;
            continue;
        }

        // Threshold violated — look backward.
        loop {
            if np == 0 || nodes[np - 1].gamma < t as i64 {
                // Can't back up — relax threshold, stay on best branch.
                t -= delta;
                if nodes[np].i != 0 {
                    nodes[np].i = 0;
                    nodes[np].encstate ^= 1;
                }
                break;
            }
            // Back up one step.
            np -= 1;
            if np < tail_idx && nodes[np].i == 0 {
                // Try the next-best branch at this node.
                nodes[np].i = 1;
                nodes[np].encstate ^= 1;
                break;
            }
            // else: keep backing up
        }
    }

    // Recover the data bits: the encoder was shifted in at bit position k
    // with LSB = data[k], so data[k] = nodes[k+K-1].encstate & 1.
    // WSJT-X copies `nodes[7].encstate` as the first byte and then strides
    // by 8 — that's because it packs 8 input bits back into one byte.
    let nbytes = nbits / 8;
    let mut data = Vec::with_capacity(nbytes);
    for i in 0..nbytes {
        data.push(nodes[8 * i + 7].encstate as u8);
    }

    let final_metric = nodes[np.min(nbits)].gamma;
    FanoDecodeResult {
        data,
        metric: final_metric,
        cycles,
        max_np,
        converged,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_then_decode_noise_free() {
        // 81-bit input (50 message + 31 zero-tail). Use a bit pattern that
        // fills the first 50 positions with alternating 1s/0s.
        let nbits = 81;
        let mut data = [0u8; 11]; // 11 * 8 = 88 > 81
        for i in 0..50 {
            if i % 3 == 0 {
                data[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        let mut coded = vec![0u8; 2 * nbits];
        conv_encode(&data, nbits, &mut coded);

        // Perfect LLRs: +8 for bit=0, -8 for bit=1.
        let llrs: Vec<f32> = coded.iter().map(|&b| if b == 0 { 8.0 } else { -8.0 }).collect();
        let bm = build_branch_metrics(&llrs, 0.0, 16.0);
        let res = fano_decode(&bm, nbits, 17, 10_000);

        assert!(res.converged, "fano should converge on perfect LLRs");
        // Verify the recovered first 50 bits match the input.
        for i in 0..50 {
            let orig = (data[i / 8] >> (7 - (i % 8))) & 1;
            let got = (res.data[i / 8] >> (7 - (i % 8))) & 1;
            assert_eq!(got, orig, "bit {} mismatch", i);
        }
    }

    #[test]
    fn encoder_symmetry_poly1_poly2_odd() {
        // Both POLY1 and POLY2 have odd parity (their LSB = 1), so the
        // branch symbols for state and state^1 are complementary pairs.
        // Verify this for a few states.
        for state in [0u32, 1, 0xaaaa_aaaa, 0x5555_5555, 0xdead_beef] {
            let a = encode_step(state);
            let b = encode_step(state ^ 1);
            assert_eq!(a ^ b, 0b11, "not complementary for state {:#x}", state);
        }
    }
}
