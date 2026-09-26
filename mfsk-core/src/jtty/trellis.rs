//! List decoding of the JTTY tail-biting convolutional code.
//!
//! Ported from WSJT-X `lib/jtty/jtty_tbcc_list_decoder.f90`
//! (`jtty_tbcc_list_wava_optimized`; `..._reference` is the plain form of the
//! same algorithm), tag `v3.2.0-rc1`.
//!
//! The input is not bit LLRs but the receiver's complex correlations with the
//! four tone references at each of the 46 data symbols ([`Correlations`]); the
//! branch metric of a block of `L` symbols and a hypothesised tone sequence is
//!
//! ```text
//! |Σ z[tone_i, symbol_i]|² / L          (L = 1, 2 or 4: the coherent length)
//! ```
//!
//! — non-coherent energy for `L = 1`, and for longer blocks the complex
//! correlations are summed first, i.e. the carrier phase is assumed constant
//! over `L` symbols. (Tone spacing equals the baud, so each tone's phase
//! advances a whole number of turns per symbol, which is what lets them add.)
//!
//! **Algorithm.** A list Wrap-Around Viterbi decoder: every one of the 512
//! states starts alive with metric 0 and itself as origin; the frame is run
//! round the circle twice; each state keeps its best four paths (ordered by
//! metric, then by origin and word). At the start of each pass every path's
//! word and origin are reset to its current state, the metric carried over.
//! Afterwards only *closed* paths (final state = origin) are kept, deduplicated
//! by their 46-bit word, **re-scored with a single-pass metric** (the WAVA
//! metric includes the earlier pass) and sorted by that; the best four are
//! returned. Branches that would set the reserved payload bit (bit 33) are
//! dropped inside the trellis, before any CRC.
//!
//! **Parallelism.** One decode is one small sequential trellis; the work that
//! parallelises is *across* decodes (candidates, rungs of the ladder — see
//! [`super::ladder`]), so this module has no rayon in it. A [`Plan`] is
//! immutable and shared between threads; a decode allocates its own workspace.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use num_complex::Complex32;

use super::tbcc::{STATES, transition};
use super::{INFO_BITS, crc};

/// Complex correlation of each data symbol with each of the four tone
/// references: `[symbol][tone]`.
pub type Correlations = [[Complex32; 4]; INFO_BITS];

/// Paths kept per state (`per_state_width`).
pub const PATHS_PER_STATE: usize = 4;
/// Words returned to the CRC gate (`JTTY_TBCC_MAX_HYPOTHESES`).
pub const HYPOTHESES: usize = 4;
/// Trips round the circle.
pub const WRAPS: usize = 2;
/// The reserved payload bit, 1-based.
pub const RESERVED_BIT: usize = 33;

const VALID: u64 = 1 << 57;
const ORIGIN_SHIFT: u32 = INFO_BITS as u32;
const ORIGIN_MASK: u64 = 0x7FF;
const WORD_MASK: u64 = (1 << INFO_BITS) - 1;
const NEG: f64 = f64::MIN; // -huge, as upstream's NEGATIVE_METRIC

/// One returned word.
#[derive(Clone, Debug, PartialEq)]
pub struct Hypothesis {
    /// The 46 information bits, first bit first.
    pub bits: [u8; INFO_BITS],
    /// The CRC-12 checks out.
    pub crc_valid: bool,
    /// Single-pass metric the list is ordered by.
    pub clean_metric: f64,
    /// Metric of the best path that produced it, history of the first pass
    /// included.
    pub wava_metric: f64,
    /// State the path started (and ended) in.
    pub start_state: u32,
}

/// The best words of one decode and how many distinct closed words there were.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ListResult {
    /// At most [`HYPOTHESES`], best first.
    pub hypotheses: Vec<Hypothesis>,
    /// Distinct closed words found (`pool_count`).
    pub pool: usize,
}

struct Block {
    /// 0-based index of the block's first information bit.
    start: usize,
    len: usize,
    /// Offset of this block's tone-sequence energies in the flat table.
    energy_offset: usize,
    /// `[state * 2^len + word]` → tone-sequence index within the block.
    sequence: Vec<u16>,
    /// `[word]` → the word's bits placed in a survivor key.
    identity: Vec<u64>,
}

/// The trellis geometry for one coherent length, shared by every decode.
pub struct Plan {
    coherent: usize,
    blocks: Vec<Block>,
    energy_count: usize,
}

impl Plan {
    /// A plan for coherent length `coherent` (1, 2 or 4).
    ///
    /// # Panics
    /// If `coherent` is not 1, 2 or 4.
    pub fn new(coherent: usize) -> Self {
        assert!(matches!(coherent, 1 | 2 | 4), "coherent length 1, 2 or 4");
        let mut energy_offset = 0;
        let blocks: Vec<Block> = (0..INFO_BITS)
            .step_by(coherent)
            .map(|start| {
                let len = coherent.min(INFO_BITS - start);
                let words = 1usize << len;
                let mut sequence = alloc::vec![0u16; STATES * words];
                for state in 0..STATES {
                    for word in 0..words {
                        let (mut s, mut seq) = (state as u32, 0u16);
                        for k in 0..len {
                            let bit = ((word >> (len - 1 - k)) & 1) as u8;
                            let (next, tone) = transition(s, bit);
                            seq = seq * 4 + u16::from(tone);
                            s = next;
                        }
                        sequence[state * words + word] = seq;
                    }
                }
                let identity = (0..words)
                    .map(|word| {
                        (0..len)
                            .filter(|k| (word >> (len - 1 - k)) & 1 == 1)
                            .fold(0u64, |acc, k| acc | 1 << (INFO_BITS - (start + k + 1)))
                    })
                    .collect();
                let block = Block {
                    start,
                    len,
                    energy_offset,
                    sequence,
                    identity,
                };
                energy_offset += 1 << (2 * len);
                block
            })
            .collect();
        Self {
            coherent,
            blocks,
            energy_count: energy_offset,
        }
    }

    /// The coherent length this plan was built for.
    pub fn coherent(&self) -> usize {
        self.coherent
    }

    /// `|Σ z|² / len` for every tone sequence of every block (`precompute_sequence_energies`).
    fn energies(&self, z: &Correlations) -> Vec<f64> {
        let mut out = alloc::vec![0f64; self.energy_count];
        for b in &self.blocks {
            for seq in 0..(1usize << (2 * b.len)) {
                let sum = (0..b.len).fold(Complex32::new(0.0, 0.0), |acc, k| {
                    let tone = (seq >> (2 * (b.len - 1 - k))) & 3;
                    acc + z[b.start + k][tone]
                });
                let e = sum.re * sum.re + sum.im * sum.im;
                out[b.energy_offset + seq] = f64::from(e) / b.len as f64;
            }
        }
        out
    }

    /// Single-pass metric of a closed word (`score_planned_identity`).
    fn clean_metric(&self, energies: &[f64], key: u64) -> f64 {
        let start = (key & (STATES as u64 - 1)) as u32; // the last MEMORY bits
        let mut state = start;
        let mut metric = 0.0;
        for b in &self.blocks {
            let word = ((key >> (INFO_BITS - b.start - b.len)) & ((1 << b.len) - 1)) as usize;
            metric += energies
                [b.energy_offset + usize::from(b.sequence[state as usize * (1 << b.len) + word])];
            state = ((state << b.len) | word as u32) & (STATES as u32 - 1);
        }
        debug_assert_eq!(state, start, "a closed path must score as closed");
        metric
    }

    /// List-WAVA decode of `z`.
    ///
    /// With `prune_reserved` set, branches that set the reserved payload bit are
    /// dropped inside the trellis (`prune_reserved_zero`).
    pub fn decode(&self, z: &Correlations, prune_reserved: bool) -> ListResult {
        let energies = self.energies(z);
        let mut prev = alloc::vec![[Surv::EMPTY; PATHS_PER_STATE]; STATES];
        let mut cur = prev.clone();
        prev.iter_mut()
            .enumerate()
            .for_each(|(s, p)| p[0] = Surv::new(s));

        for _ in 0..WRAPS {
            // each pass restarts the word and origin of every live path
            prev.iter_mut().enumerate().for_each(|(state, paths)| {
                paths
                    .iter_mut()
                    .filter(|p| p.valid())
                    .for_each(|p| p.key = VALID | (state as u64) << ORIGIN_SHIFT);
            });
            for (index, b) in self.blocks.iter().enumerate() {
                let prune =
                    prune_reserved && b.start < RESERVED_BIT && RESERVED_BIT <= b.start + b.len; // block holds bit 33 (1-based)
                advance(b, &energies, prune, index == 0, &prev, &mut cur);
                core::mem::swap(&mut prev, &mut cur);
            }
        }

        // closed paths only, one entry per distinct word
        struct Entry {
            key: u64,
            wava: f64,
            start: u32,
        }
        let mut pool: Vec<Entry> = Vec::new();
        let mut index: BTreeMap<u64, usize> = BTreeMap::new();
        for (state, paths) in prev.iter().enumerate() {
            for p in paths.iter().filter(|p| p.valid()) {
                if (p.key >> ORIGIN_SHIFT) & ORIGIN_MASK != state as u64 {
                    continue;
                }
                let key = p.key & WORD_MASK;
                match index.get(&key) {
                    Some(&i) => {
                        let e = &mut pool[i];
                        if p.metric > e.wava || (p.metric >= e.wava && (state as u32) < e.start) {
                            e.wava = p.metric;
                            e.start = state as u32;
                        }
                    }
                    None => {
                        index.insert(key, pool.len());
                        pool.push(Entry {
                            key,
                            wava: p.metric,
                            start: state as u32,
                        });
                    }
                }
            }
        }

        let mut scored: Vec<(f64, u64, &Entry)> = pool
            .iter()
            .map(|e| (self.clean_metric(&energies, e.key), reverse_bits(e.key), e))
            .collect();
        // clean metric descending, then the word (first bit least significant,
        // as upstream's `identity`), then start state, then WAVA metric
        scored.sort_by(|a, b| {
            b.0.partial_cmp(&a.0)
                .unwrap_or(core::cmp::Ordering::Equal)
                .then(a.1.cmp(&b.1))
                .then(a.2.start.cmp(&b.2.start))
                .then(
                    b.2.wava
                        .partial_cmp(&a.2.wava)
                        .unwrap_or(core::cmp::Ordering::Equal),
                )
        });
        let hypotheses = scored
            .iter()
            .take(HYPOTHESES)
            .map(|&(clean, _, e)| {
                let bits: [u8; INFO_BITS] =
                    core::array::from_fn(|i| ((e.key >> (INFO_BITS - 1 - i)) & 1) as u8);
                Hypothesis {
                    crc_valid: crc::is_valid(&bits),
                    bits,
                    clean_metric: clean,
                    wava_metric: e.wava,
                    start_state: e.start,
                }
            })
            .collect();
        ListResult {
            hypotheses,
            pool: pool.len(),
        }
    }
}

/// The word with its first bit in the least significant place (upstream's
/// `identity`); only used to break ties the way it does.
fn reverse_bits(key: u64) -> u64 {
    (0..INFO_BITS).fold(0u64, |acc, i| acc | ((key >> (INFO_BITS - 1 - i)) & 1) << i)
}

/// A surviving path: its metric and a key packing the word so far (first bit
/// most significant), the origin state and a valid flag.
#[derive(Clone, Copy)]
struct Surv {
    metric: f64,
    key: u64,
}

impl Surv {
    const EMPTY: Self = Self {
        metric: NEG,
        key: 0,
    };

    fn new(state: usize) -> Self {
        Self {
            metric: 0.0,
            key: VALID | (state as u64) << ORIGIN_SHIFT,
        }
    }

    fn valid(&self) -> bool {
        self.key & VALID != 0
    }

    /// Better metric first; ties by key (origin, then word).
    fn precedes(&self, other: &Self) -> bool {
        self.metric > other.metric || (self.metric == other.metric && self.key < other.key)
    }
}

/// Insert `cand` into a state's four best, best first. `dedupe`: after a pass
/// restart several ranks of a state share one key, and the better metric wins.
fn insert(sel: &mut [Surv; PATHS_PER_STATE], cand: Surv, dedupe: bool) {
    if dedupe && let Some(slot) = sel.iter().position(|s| s.valid() && s.key == cand.key) {
        if cand.metric <= sel[slot].metric {
            return;
        }
        sel.copy_within(slot + 1.., slot);
        sel[PATHS_PER_STATE - 1] = Surv::EMPTY;
    }
    let Some(at) = sel.iter().position(|s| !s.valid() || cand.precedes(s)) else {
        return;
    };
    sel.copy_within(at..PATHS_PER_STATE - 1, at + 1);
    sel[at] = cand;
}

/// One block of the trellis: for every end state, its best paths.
///
/// The low `len` bits of an end state are the block's input word, and the
/// predecessors are the states whose remaining high bits enumerate every
/// possibility — so no branch table is needed to find them.
fn advance(
    b: &Block,
    energies: &[f64],
    prune: bool,
    first_block: bool,
    prev: &[[Surv; PATHS_PER_STATE]],
    cur: &mut [[Surv; PATHS_PER_STATE]],
) {
    let words = 1usize << b.len;
    let stride = STATES / words;
    let reserved = 1u64 << (INFO_BITS - RESERVED_BIT);
    cur.iter_mut().enumerate().for_each(|(end, out)| {
        *out = [Surv::EMPTY; PATHS_PER_STATE];
        let word = end & (words - 1);
        let identity = b.identity[word];
        if prune && identity & reserved != 0 {
            return;
        }
        for pred in (0..words).map(|k| (end >> b.len) + k * stride) {
            let branch = energies[b.energy_offset + usize::from(b.sequence[pred * words + word])];
            for s in prev[pred].iter().filter(|s| s.valid()) {
                let metric = s.metric + branch;
                // paths are ordered, so once one is too weak the rest are too
                if out[PATHS_PER_STATE - 1].valid() && metric < out[PATHS_PER_STATE - 1].metric {
                    break;
                }
                insert(
                    out,
                    Surv {
                        metric,
                        key: s.key | identity,
                    },
                    first_block,
                );
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jtty::{crc, tbcc};

    fn truth_correlations(info: &[u8; INFO_BITS]) -> Correlations {
        let tones = tbcc::encode(info);
        core::array::from_fn(|t| {
            core::array::from_fn(|k| {
                if usize::from(tones[t]) == k {
                    Complex32::new(1.0, 0.0)
                } else {
                    Complex32::new(0.0, 0.0)
                }
            })
        })
    }

    #[test]
    fn noiseless_correlations_decode_to_the_sent_word_at_every_length() {
        let mut payload = [0u8; 34];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = u8::from(i % 5 == 1 || i % 7 == 3);
        }
        payload[32] = 0;
        let info = crc::append(&payload);
        let z = truth_correlations(&info);
        for l in [1, 2, 4] {
            let r = Plan::new(l).decode(&z, true);
            let best = &r.hypotheses[0];
            assert_eq!(best.bits, info, "L={l}");
            assert!(best.crc_valid, "L={l}");
            assert!(r.hypotheses.len() <= HYPOTHESES);
        }
    }

    #[test]
    fn plan_geometry() {
        let p = Plan::new(4);
        assert_eq!(p.blocks.len(), 12);
        assert_eq!(p.blocks.last().unwrap().len, 2, "46 = 11·4 + 2");
        assert_eq!(Plan::new(1).blocks.len(), 46);
        assert_eq!(Plan::new(2).blocks.len(), 23);
        assert_eq!(crate::jtty::tbcc::MEMORY, 9);
    }
}
