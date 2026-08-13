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

#[cfg(test)]
use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

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

/// wsprd's soft-symbol normalisation + `mettab` lookup — the faithful
/// replacement for [`build_branch_metrics`]'s linear approximation on
/// the WSPR code.
///
/// Two steps, both from `wsprd.c`:
///
/// 1. **Normalise** (`wsprd.c:472-481`). Divide every soft symbol by
///    the RMS `fac = sqrt(f2sum − fsum²)` of the 162-symbol vector,
///    scale by `symfac = 50`, clamp to `[−128, 127]`, offset by
///    `+128`. Note wsprd divides by the RMS *without* subtracting the
///    mean — `fsum` is computed but only ever used inside `fac`. This
///    port keeps that quirk deliberately.
///
///    Normalising here is what makes the metric scale-invariant, which
///    is why the table can be shared across every input level: the
///    caller no longer has to hand us LLRs on a particular scale.
///
/// 2. **Look up** (`wsprd.c:912-913`):
///    `mettab[0][i] = round(10·(T[i] − bias))`,
///    `mettab[1][i] = round(10·(T[255 − i] − bias))`.
///
/// `llrs` arrives in *this crate's* sign convention (positive → bit 0,
/// matching `wspr::demod`'s `p0 − p1`); wsprd's `fsymb` is `xm1 − xm0`,
/// the opposite, so the sign flips on the way in.
pub fn build_branch_metrics_wsprd(llrs: &[f32], bias: f32) -> Vec<[i32; 2]> {
    use super::metric_table::WSPRD_METRIC_TABLE as T;
    wsprd_normalised_symbols(llrs)
        .map(|v| {
            let sym = ((v + 128.0) as usize).min(255);
            let m0 = (10.0 * (T[sym] - bias)).round() as i32;
            let m1 = (10.0 * (T[255 - sym] - bias)).round() as i32;
            [m0, m1]
        })
        .collect()
}

/// `symfac`, `wsprd.c:816`.
const SYMFAC: f32 = 50.0;

/// Step 1 of [`build_branch_metrics_wsprd`] on its own: the soft
/// symbols wsprd's `fsymb` holds after normalisation, in wsprd's sign
/// and on wsprd's `[-128, 127]` scale.
///
/// Split out because `wsprd.c:1338-1345` measures the RMS of exactly
/// this vector before deciding whether a Fano attempt is worth making,
/// and that gate has to see the same numbers the metric does.
pub fn wsprd_normalised_symbols(llrs: &[f32]) -> impl Iterator<Item = f32> + '_ {
    let n = llrs.len() as f32;
    // Work in wsprd's sign so the constants can be read straight off
    // the C. Note wsprd divides by the RMS *without* subtracting the
    // mean — `fsum` is computed but only ever used inside `fac`.
    let fsum: f32 = llrs.iter().map(|l| -l).sum::<f32>() / n;
    let f2sum: f32 = llrs.iter().map(|l| l * l).sum::<f32>() / n;
    let fac = (f2sum - fsum * fsum).max(0.0).sqrt();
    llrs.iter().map(move |&l| {
        if fac > 0.0 {
            (SYMFAC * -l / fac).clamp(-128.0, 127.0)
        } else {
            0.0
        }
    })
}

/// RMS of the normalised soft symbols — `wsprd.c:1338-1345`'s `rms`.
///
/// Normalisation alone would put this at or just above `symfac = 50`;
/// the clamp to `[-128, 127]` is what pulls it down, so a low value
/// means the vector is dominated by a handful of saturating outliers
/// rather than carrying evenly-distributed evidence. wsprd uses it as a
/// cheap plausibility gate before spending a Fano attempt.
pub fn wsprd_soft_symbol_rms(llrs: &[f32]) -> f32 {
    let n = llrs.len() as f32;
    (wsprd_normalised_symbols(llrs).map(|v| v * v).sum::<f32>() / n).sqrt()
}

/// `minrms`, `wsprd.c:808`: `52.0 * (symfac / 64.0)`.
pub const WSPRD_MIN_RMS: f32 = 52.0 * (SYMFAC / 64.0);

#[derive(Default)]
struct Node {
    encstate: u32,
    gamma: i64, // path metric, accumulated
    metrics: [i32; 4],
    tm: [i32; 2],
    i: u8, // 0 or 1 — which hypothesis we're currently exploring
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
///
/// Allocates a fresh [`FanoScratch`] every call — for a hot loop that
/// calls this repeatedly with the same `nbits` (WSPR: once per
/// `nblock` value per candidate, up to ~100 calls/decode), use
/// [`fano_decode_with_scratch`] with a caller-pooled scratch instead.
pub fn fano_decode(
    branch_metrics: &[[i32; 2]],
    nbits: usize,
    delta: i32,
    max_cycles_per_bit: u64,
) -> FanoDecodeResult {
    let mut scratch = FanoScratch::new();
    fano_decode_with_scratch(
        &mut scratch,
        branch_metrics,
        nbits,
        delta,
        max_cycles_per_bit,
    )
}

/// Reusable working memory for [`fano_decode_with_scratch`] — the
/// `Vec<Node>` search-tree buffer, pooled across calls that share the
/// same `nbits` instead of reallocated (~`nbits+1` `Node`s, ~32B each)
/// every time.
#[derive(Default)]
pub struct FanoScratch {
    nodes: Vec<Node>,
}

impl FanoScratch {
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }
}

/// [`fano_decode`] with caller-provided scratch, reused across calls
/// that share `nbits` (constant for a given code — 81 for WSPR,
/// [`crate::fec::ConvFano::NBITS`]) instead of reallocating
/// `Vec<Node>` every call.
///
/// Safe to pool: the precompute loop below unconditionally rewrites
/// `.metrics` for every node index `0..nbits`, and every node's
/// `.gamma`/`.encstate`/`.tm`/`.i` are set on first visit *before* any
/// read, both going forward (the `while cycles < max_cycles` loop's
/// "look forward" branch) and on backtrack (which only revisits nodes
/// already touched earlier *in the same call*) — no field is ever read
/// before this call has written it, so nothing leaks from a prior
/// call's leftover values as long as `nbits` (hence `nodes.len()`)
/// stays constant. `nodes[nbits]` itself is the one exception: its
/// `.metrics` is never rewritten by the precompute loop (which only
/// touches `0..nbits`) *and* never read either (the forward loop
/// breaks with `converged = true` the instant `np` reaches `nbits`,
/// before ever reaching the "look forward" read at the top of the
/// next iteration) — so its stale contents are inert either way.
pub fn fano_decode_with_scratch(
    scratch: &mut FanoScratch,
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

    if scratch.nodes.len() != nbits + 1 {
        scratch.nodes = (0..=nbits).map(|_| Node::default()).collect();
    }
    let nodes = &mut scratch.nodes;

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
    instrument::record(cycles, max_cycles, converged);
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
        let llrs: Vec<f32> = coded
            .iter()
            .map(|&b| if b == 0 { 8.0 } else { -8.0 })
            .collect();
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

/// Cycle-budget histogram for the sequential decoder.
///
/// Written 2026-08-13 for issue
/// [#260](https://github.com/jl1nie/mfsk-core/issues/260), which the S3
/// candidate-loop measurement narrowed to one question: WSPR spends
/// most of its embedded decode time in Fano attempts that never
/// converge, so *how much of the budget does a successful decode
/// actually need, and do the failures run to exhaustion?* If the
/// successes finish early and the failures do not, a progressive
/// budget ladder rejects most of the workload without needing a
/// classifier.
///
/// [`FanoDecodeResult`] already carries `cycles`, `max_np` and
/// `converged`, so nothing new has to be computed — these counters
/// only aggregate what the decoder already returns, at one relaxed
/// atomic add per call.
pub mod instrument {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Cycle sums are accumulated in units of 1024 cycles.
    ///
    /// Xtensa has no 64-bit atomics, and a raw sum would be tight in
    /// `u32`: a WSPR budget is `10_000 × 81 = 810_000` cycles and one
    /// scan makes ~1 200 attempts, so an exact sum reaches ~9.7e8 —
    /// inside `u32` but with under a decade of margin. Shifting by 10
    /// buys three decades and costs nothing a histogram cares about.
    pub const CYCLE_SUM_SHIFT: u32 = 10;

    /// `max_cycles_per_bit × nbits` of the most recent call — the
    /// denominator the deciles below are fractions of.
    pub static BUDGET: AtomicU32 = AtomicU32::new(0);

    pub static OK_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Sum of converged attempts' cycles, in `1 << CYCLE_SUM_SHIFT` units.
    pub static OK_CYCLES_SUM: AtomicU32 = AtomicU32::new(0);
    /// Raw cycles of the most expensive converged attempt.
    pub static OK_CYCLES_MAX: AtomicU32 = AtomicU32::new(0);
    pub static FAIL_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Sum of failed attempts' cycles, same units as [`OK_CYCLES_SUM`].
    pub static FAIL_CYCLES_SUM: AtomicU32 = AtomicU32::new(0);

    /// Converged attempts bucketed by tenth of budget consumed;
    /// `OK_DECILE[0]` is "finished inside the first 10 %".
    pub static OK_DECILE: [AtomicU32; 10] = [const { AtomicU32::new(0) }; 10];
    /// Failed attempts, same bucketing. `FAIL_DECILE[9]` is
    /// "ran essentially to exhaustion".
    pub static FAIL_DECILE: [AtomicU32; 10] = [const { AtomicU32::new(0) }; 10];

    pub(super) fn record(cycles: u64, budget: u64, converged: bool) {
        let cycles32 = u32::try_from(cycles).unwrap_or(u32::MAX);
        let budget32 = u32::try_from(budget).unwrap_or(u32::MAX);
        BUDGET.store(budget32, Ordering::Relaxed);
        let decile = (cycles * 10)
            .checked_div(budget)
            .map_or(0, |d| (d as usize).min(9));
        let scaled = cycles32 >> CYCLE_SUM_SHIFT;
        if converged {
            OK_COUNT.fetch_add(1, Ordering::Relaxed);
            OK_CYCLES_SUM.fetch_add(scaled, Ordering::Relaxed);
            OK_CYCLES_MAX.fetch_max(cycles32, Ordering::Relaxed);
            OK_DECILE[decile].fetch_add(1, Ordering::Relaxed);
        } else {
            FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
            FAIL_CYCLES_SUM.fetch_add(scaled, Ordering::Relaxed);
            FAIL_DECILE[decile].fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Zero every counter.
    pub fn reset() {
        for c in [
            &BUDGET,
            &OK_COUNT,
            &OK_CYCLES_SUM,
            &OK_CYCLES_MAX,
            &FAIL_COUNT,
            &FAIL_CYCLES_SUM,
        ] {
            c.store(0, Ordering::Relaxed);
        }
        for b in OK_DECILE.iter().chain(FAIL_DECILE.iter()) {
            b.store(0, Ordering::Relaxed);
        }
    }

    /// One reading, flattened for logging.
    #[must_use]
    pub fn snapshot() -> Snapshot {
        let mut ok = [0u32; 10];
        let mut fail = [0u32; 10];
        for i in 0..10 {
            ok[i] = OK_DECILE[i].load(Ordering::Relaxed);
            fail[i] = FAIL_DECILE[i].load(Ordering::Relaxed);
        }
        Snapshot {
            budget: BUDGET.load(Ordering::Relaxed),
            ok_count: OK_COUNT.load(Ordering::Relaxed),
            ok_cycles_sum: OK_CYCLES_SUM.load(Ordering::Relaxed),
            ok_cycles_max: OK_CYCLES_MAX.load(Ordering::Relaxed),
            fail_count: FAIL_COUNT.load(Ordering::Relaxed),
            fail_cycles_sum: FAIL_CYCLES_SUM.load(Ordering::Relaxed),
            ok_decile: ok,
            fail_decile: fail,
        }
    }

    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Snapshot {
        pub budget: u32,
        pub ok_count: u32,
        /// In `1 << CYCLE_SUM_SHIFT` cycle units.
        pub ok_cycles_sum: u32,
        pub ok_cycles_max: u32,
        pub fail_count: u32,
        /// In `1 << CYCLE_SUM_SHIFT` cycle units.
        pub fail_cycles_sum: u32,
        pub ok_decile: [u32; 10],
        pub fail_decile: [u32; 10],
    }

    impl Snapshot {
        /// `self - earlier`, for bracketing one pass.
        #[must_use]
        pub fn since(self, e: Snapshot) -> Snapshot {
            let mut ok = [0u32; 10];
            let mut fail = [0u32; 10];
            for i in 0..10 {
                ok[i] = self.ok_decile[i].saturating_sub(e.ok_decile[i]);
                fail[i] = self.fail_decile[i].saturating_sub(e.fail_decile[i]);
            }
            Snapshot {
                budget: self.budget,
                ok_count: self.ok_count.saturating_sub(e.ok_count),
                ok_cycles_sum: self.ok_cycles_sum.saturating_sub(e.ok_cycles_sum),
                // A max cannot be differenced; carry the running value.
                ok_cycles_max: self.ok_cycles_max,
                fail_count: self.fail_count.saturating_sub(e.fail_count),
                fail_cycles_sum: self.fail_cycles_sum.saturating_sub(e.fail_cycles_sum),
                ok_decile: ok,
                fail_decile: fail,
            }
        }

        /// Mean cycles of converged attempts, back in raw units.
        #[must_use]
        pub fn ok_cycles_mean(self) -> u32 {
            self.ok_cycles_sum.checked_div(self.ok_count).unwrap_or(0) << CYCLE_SUM_SHIFT
        }

        /// Mean cycles of failed attempts, back in raw units.
        #[must_use]
        pub fn fail_cycles_mean(self) -> u32 {
            self.fail_cycles_sum
                .checked_div(self.fail_count)
                .unwrap_or(0)
                << CYCLE_SUM_SHIFT
        }
    }
}
