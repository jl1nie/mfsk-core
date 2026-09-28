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
use num_traits::Float;

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

/// The precision the path metrics are carried in. `f64` is what upstream uses and what
/// [`Plan::decode`] does; [`Plan::decode_f32`] carries them in `f32`, which is hardware on
/// an Xtensa LX7 where `f64` is software (`docs/notes/JTTY_UPSTREAM.md`, "E0 results").
/// Words and keys are integers either way; only the metric arithmetic and comparisons
/// change.
trait Metric: Float {}
impl Metric for f32 {}
impl Metric for f64 {}

/// What a decode reports about itself while it runs; `()` reports nothing and costs nothing.
trait Probe {
    /// A candidate extension of a path (a metric was added).
    fn extension(&mut self) {}
    /// A path was placed in an end state's list.
    fn insert_call(&mut self) {}
    /// The merge met a word already in the list (first block of a pass only).
    fn duplicate(&mut self) {}
    /// End of phase `i` (see [`Profile::laps`]).
    fn lap(&mut self, _phase: usize) {}
}
impl Probe for () {}

/// Cycles and counts of one decode, for `jtty-bench` on the LX7 (#499); [`Plan::profile_f32`].
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Profile {
    /// Ticks of the caller's clock per phase: 0 energies, 1 survivor arrays, 2 first wrap, 3
    /// second wrap, 4 closed-word pool, 5 scoring and sort, 6 hypotheses and CRC.
    pub laps: [u32; 7],
    /// Candidate extensions considered.
    pub extensions: u32,
    /// Paths placed in an end state's list.
    pub inserts: u32,
    /// Extensions the merge dropped as a word already in the list (first block only).
    pub duplicates: u32,
    /// Distinct closed words.
    pub pool: usize,
    clock: Option<fn() -> u32>,
    last: u32,
}
impl Probe for Profile {
    fn extension(&mut self) {
        self.extensions += 1;
    }
    fn insert_call(&mut self) {
        self.inserts += 1;
    }
    fn duplicate(&mut self) {
        self.duplicates += 1;
    }
    fn lap(&mut self, phase: usize) {
        if let Some(clock) = self.clock {
            let now = clock();
            self.laps[phase] = self.laps[phase].wrapping_add(now.wrapping_sub(self.last));
            self.last = clock();
        }
    }
}

/// The two survivor arrays of an `f32` decode, 32 KB each, for [`Plan::decode_f32_in`].
pub struct TrellisScratch {
    a: Vec<[Surv<f32>; PATHS_PER_STATE]>,
    b: Vec<[Surv<f32>; PATHS_PER_STATE]>,
}

impl TrellisScratch {
    /// Allocate both (the caller chooses where by when it calls this).
    pub fn new() -> Self {
        let a = alloc::vec![[Surv::<f32>::empty(); PATHS_PER_STATE]; STATES];
        Self { b: a.clone(), a }
    }
}

impl Default for TrellisScratch {
    fn default() -> Self {
        Self::new()
    }
}

/// `-huge`, as upstream's `NEGATIVE_METRIC`.
fn neg<M: Metric>() -> M {
    M::min_value()
}

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
    /// `[state * 2^len + word]` → tone-sequence index within the block. It depends on the
    /// block's length only, so blocks of one length share it: 22 KB for the three plans
    /// instead of 380 KB, small enough for the LX7's internal DRAM (#499).
    sequence: alloc::sync::Arc<[u16]>,
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
        let table = |len: usize| -> alloc::sync::Arc<[u16]> {
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
            sequence.into()
        };
        let mut tables: [Option<alloc::sync::Arc<[u16]>>; 5] = Default::default();
        let mut energy_offset = 0;
        let blocks: Vec<Block> = (0..INFO_BITS)
            .step_by(coherent)
            .map(|start| {
                let len = coherent.min(INFO_BITS - start);
                let words = 1usize << len;
                let sequence = tables[len].get_or_insert_with(|| table(len)).clone();
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
    fn energies<M: Metric>(&self, z: &Correlations) -> Vec<M> {
        let mut out = alloc::vec![M::zero(); self.energy_count];
        for b in &self.blocks {
            for seq in 0..(1usize << (2 * b.len)) {
                let sum = (0..b.len).fold(Complex32::new(0.0, 0.0), |acc, k| {
                    let tone = (seq >> (2 * (b.len - 1 - k))) & 3;
                    acc + z[b.start + k][tone]
                });
                let e = sum.re * sum.re + sum.im * sum.im;
                out[b.energy_offset + seq] =
                    M::from(e).expect("a float") / M::from(b.len).expect("a float");
            }
        }
        out
    }

    /// Single-pass metric of a closed word (`score_planned_identity`).
    fn clean_metric<M: Metric>(&self, energies: &[M], key: u64) -> M {
        let start = (key & (STATES as u64 - 1)) as u32; // the last MEMORY bits
        let mut state = start;
        let mut metric = M::zero();
        for b in &self.blocks {
            let word = ((key >> (INFO_BITS - b.start - b.len)) & ((1 << b.len) - 1)) as usize;
            metric = metric
                + energies[b.energy_offset
                    + usize::from(b.sequence[state as usize * (1 << b.len) + word])];
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
        self.decode_with::<f64>(z, prune_reserved)
    }

    /// [`Self::decode`] with the path metrics carried in `f32`. The words, their order and the
    /// CRC flags agree with the `f64` decode except where two metrics are within `f32`
    /// rounding of each other; `clean_metric` and `wava_metric` are the `f32` values widened.
    pub fn decode_f32(&self, z: &Correlations, prune_reserved: bool) -> ListResult {
        self.decode_with::<f32>(z, prune_reserved)
    }

    /// [`Self::decode_f32`] counting and timing itself with `clock` (ticks; wraps are handled),
    /// for `jtty-bench` (#499). Returns the same list; the list is dropped here.
    #[doc(hidden)]
    pub fn profile_f32(&self, z: &Correlations, clock: fn() -> u32) -> Profile {
        let mut probe = Profile {
            clock: Some(clock),
            last: clock(),
            ..Profile::default()
        };
        let list = self.decode_probe::<f32, Profile>(z, true, &mut probe);
        probe.pool = list.pool;
        probe
    }

    fn decode_with<M: Metric>(&self, z: &Correlations, prune_reserved: bool) -> ListResult {
        self.decode_probe::<M, ()>(z, prune_reserved, &mut ())
    }

    /// [`Self::decode_f32`] in `scratch`'s survivor arrays instead of two it allocates: an
    /// embedded receiver makes them once, in memory it chose (#499).
    pub fn decode_f32_in(
        &self,
        z: &Correlations,
        prune_reserved: bool,
        scratch: &mut TrellisScratch,
    ) -> ListResult {
        let (a, b) = (&mut scratch.a, &mut scratch.b);
        self.decode_in::<f32, ()>(z, prune_reserved, &mut (), a, b)
    }

    fn decode_probe<M: Metric, P: Probe>(
        &self,
        z: &Correlations,
        prune_reserved: bool,
        probe: &mut P,
    ) -> ListResult {
        let mut a = alloc::vec![[Surv::<M>::empty(); PATHS_PER_STATE]; STATES];
        let mut b = a.clone();
        self.decode_in(z, prune_reserved, probe, &mut a, &mut b)
    }

    fn decode_in<M: Metric, P: Probe>(
        &self,
        z: &Correlations,
        prune_reserved: bool,
        probe: &mut P,
        a: &mut [[Surv<M>; PATHS_PER_STATE]],
        b: &mut [[Surv<M>; PATHS_PER_STATE]],
    ) -> ListResult {
        let energies = self.energies::<M>(z);
        probe.lap(0);
        let (mut prev, mut cur) = (a, b);
        prev.iter_mut()
            .for_each(|p| *p = [Surv::empty(); PATHS_PER_STATE]);
        prev.iter_mut()
            .enumerate()
            .for_each(|(s, p)| p[0] = Surv::new(s));
        probe.lap(1);

        for wrap in 0..WRAPS {
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
                if index == 0 {
                    advance::<M, P, true>(b, &energies, prune, prev, cur, probe);
                } else {
                    advance::<M, P, false>(b, &energies, prune, prev, cur, probe);
                }
                core::mem::swap(&mut prev, &mut cur);
            }
            probe.lap(2 + wrap);
        }

        // closed paths only, one entry per distinct word
        struct Entry<M> {
            key: u64,
            wava: M,
            start: u32,
        }
        let mut pool: Vec<Entry<M>> = Vec::new();
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
                        let metric = p.metric;
                        if metric > e.wava || (metric >= e.wava && (state as u32) < e.start) {
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

        probe.lap(4);
        let mut scored: Vec<(M, u64, &Entry<M>)> = pool
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
        probe.lap(5);
        let hypotheses = scored
            .iter()
            .take(HYPOTHESES)
            .map(|&(clean, _, e)| {
                let bits: [u8; INFO_BITS] =
                    core::array::from_fn(|i| ((e.key >> (INFO_BITS - 1 - i)) & 1) as u8);
                Hypothesis {
                    crc_valid: crc::is_valid(&bits),
                    bits,
                    clean_metric: clean.to_f64().expect("a float"),
                    wava_metric: e.wava.to_f64().expect("a float"),
                    start_state: e.start,
                }
            })
            .collect();
        probe.lap(6);
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
///
/// `packed(4)`: with an `f32` metric the default layout is 16 bytes, 4 of them padding
/// before the 8-aligned key, and the two survivor arrays ([`TrellisScratch`]) 32 KB each;
/// packed they are 12 and 24 KB, which on the CoreS3 is internal DRAM the app did not have
/// (`docs/notes/JTTY_CORES3_APP.md` §14). The key is still 4-aligned, which is all a 64-bit
/// load needs on Xtensa (two 32-bit words). Fields are only ever read by value (a reference
/// to the key would be unaligned).
#[derive(Clone, Copy)]
#[repr(C, packed(4))]
struct Surv<M> {
    metric: M,
    key: u64,
}

impl<M: Metric> Surv<M> {
    fn empty() -> Self {
        Self {
            metric: neg(),
            key: 0,
        }
    }

    fn new(state: usize) -> Self {
        Self {
            metric: M::zero(),
            key: VALID | (state as u64) << ORIGIN_SHIFT,
        }
    }

    fn valid(&self) -> bool {
        let key = self.key;
        key & VALID != 0
    }

    /// Better metric first; ties by key (origin, then word).
    fn precedes(&self, other: &Self) -> bool {
        let (m, om, k, ok) = (self.metric, other.metric, self.key, other.key);
        m > om || (m == om && k < ok)
    }
}

/// One block of the trellis: for every end state, its best paths.
///
/// The low `len` bits of an end state are the block's input word, and the
/// predecessors are the states whose remaining high bits enumerate every
/// possibility — so no branch table is needed to find them.
///
/// Each predecessor's paths are already best first, and extending them all by the same
/// branch keeps them so, so an end state's four best are a **merge** of its `2^len` sorted
/// lists: take the best head, four times, extending a path only when it becomes a head.
/// Upstream (and this crate until #499) inserted every extension into the end state's list one
/// at a time — 88 % of 280 000 extensions a decode reached that insert, about 400 cycles each
/// on the LX7; building every list first instead made L=4 slower (614 400 extensions). The
/// survivors are the same: the order is the same total order ([`Surv::precedes`]), and with
/// `DEDUPE` (the first block of a pass, where the paths of a state share one key) the first of
/// a key to come out is its best.
///
/// One case is not a plain merge: two paths ordered by metric can round to the same sum and
/// then be ordered by key, the other way round. Whenever a path becomes a head, the path behind
/// it is checked for that (an equal sum from unequal metrics, with a smaller key); if it
/// happened anywhere in an end state, the state is done again by [`advance_state_sorted`],
/// which sorts each extended list first.
fn advance<M: Metric, P: Probe, const DEDUPE: bool>(
    b: &Block,
    energies: &[M],
    prune: bool,
    prev: &[[Surv<M>; PATHS_PER_STATE]],
    cur: &mut [[Surv<M>; PATHS_PER_STATE]],
    probe: &mut P,
) {
    // one-bit blocks by the small merge: 190 -> 138 ms a rung on the CoreS3; two-bit blocks
    // were slower that way (145 -> 180 ms) and keep the lazy one
    if b.len == 1 {
        return advance_small::<M, P, DEDUPE, 2>(b, energies, prune, prev, cur, probe);
    }
    let words = 1usize << b.len;
    debug_assert!(words <= MAX_WORDS);
    let stride = STATES / words;
    let reserved = 1u64 << (INFO_BITS - RESERVED_BIT);
    let mut preds = [0usize; MAX_WORDS];
    let mut branch = [M::zero(); MAX_WORDS];
    let mut pos = [0usize; MAX_WORDS];
    let mut head = [Surv::<M>::empty(); MAX_WORDS];
    for (end, out) in cur.iter_mut().enumerate() {
        *out = [Surv::empty(); PATHS_PER_STATE];
        let word = end & (words - 1);
        let identity = b.identity[word];
        if prune && identity & reserved != 0 {
            continue;
        }
        // the extension of `prev[preds[k]][i]`, or empty
        let extend =
            |k: usize, i: usize, preds: &[usize; MAX_WORDS], branch: &[M; MAX_WORDS]| match prev
                [preds[k]]
                .get(i)
            {
                Some(s) if s.valid() => Surv {
                    metric: s.metric + branch[k],
                    key: s.key | identity,
                },
                _ => Surv::empty(),
            };
        // the path behind head `i` of list `k` would come out ahead of it
        let reordered =
            |k: usize, i: usize, preds: &[usize; MAX_WORDS], branch: &[M; MAX_WORDS]| {
                let list = &prev[preds[k]];
                i + 1 < PATHS_PER_STATE
                    && list[i + 1].valid()
                    && { list[i].metric } != { list[i + 1].metric }
                    && { list[i].metric } + branch[k] == { list[i + 1].metric } + branch[k]
                    && list[i + 1].key < list[i].key
            };
        let mut inverted = false;
        for k in 0..words {
            let pred = (end >> b.len) + k * stride;
            preds[k] = pred;
            branch[k] = energies[b.energy_offset + usize::from(b.sequence[pred * words + word])];
            pos[k] = 0;
            head[k] = extend(k, 0, &preds, &branch);
            probe.extension();
            inverted |= reordered(k, 0, &preds, &branch);
        }
        let mut filled = 0;
        while !inverted && filled < PATHS_PER_STATE {
            let mut best = MAX_WORDS;
            for k in 0..words {
                if head[k].valid() && (best == MAX_WORDS || head[k].precedes(&head[best])) {
                    best = k;
                }
            }
            if best == MAX_WORDS {
                break;
            }
            let cand = head[best];
            pos[best] += 1;
            head[best] = extend(best, pos[best], &preds, &branch);
            probe.extension();
            if reordered(best, pos[best], &preds, &branch) {
                inverted = true;
                break;
            }
            if DEDUPE && out[..filled].iter().any(|s| s.key == cand.key) {
                probe.duplicate();
                continue;
            }
            probe.insert_call();
            out[filled] = cand;
            filled += 1;
        }
        if inverted {
            advance_state_sorted::<M, P, DEDUPE>(
                &preds[..words],
                &branch,
                identity,
                prev,
                out,
                probe,
            );
        }
    }
}

/// [`advance`] for blocks of one or two bits (`W` = 2 or 4 predecessors): every extension is
/// built — at most 16 — each list checked to be still in order (it is unless an `f32` sum tied two
/// paths the other way round, then [`advance_state_sorted`] takes the state), and the lists merged.
/// Fixed-size arrays and no per-head bookkeeping, for the one-bit blocks of L=1 and the
/// half-symbol rung, most of the ladder's calls: 190 -> 138 ms a rung on the LX7 (#499).
#[inline(always)]
fn advance_small<M: Metric, P: Probe, const DEDUPE: bool, const W: usize>(
    b: &Block,
    energies: &[M],
    prune: bool,
    prev: &[[Surv<M>; PATHS_PER_STATE]],
    cur: &mut [[Surv<M>; PATHS_PER_STATE]],
    probe: &mut P,
) {
    let stride = STATES / W;
    let reserved = 1u64 << (INFO_BITS - RESERVED_BIT);
    let mut lists = [[Surv::<M>::empty(); PATHS_PER_STATE]; W];
    let mut lens = [0usize; W];
    for (end, out) in cur.iter_mut().enumerate() {
        let word = end & (W - 1);
        let identity = b.identity[word];
        if prune && identity & reserved != 0 {
            *out = [Surv::empty(); PATHS_PER_STATE];
            continue;
        }
        let mut sorted = true;
        let mut preds = [0usize; MAX_WORDS];
        let mut branch = [M::zero(); MAX_WORDS];
        for k in 0..W {
            let pred = (end >> b.len) + k * stride;
            let br = energies[b.energy_offset + usize::from(b.sequence[pred * W + word])];
            preds[k] = pred;
            branch[k] = br;
            let src = &prev[pred];
            let mut n = 0;
            while n < PATHS_PER_STATE && src[n].valid() {
                probe.extension();
                lists[k][n] = Surv {
                    metric: src[n].metric + br,
                    key: src[n].key | identity,
                };
                if n > 0 && lists[k][n].precedes(&lists[k][n - 1]) {
                    sorted = false;
                }
                n += 1;
            }
            lens[k] = n;
        }
        if !sorted {
            advance_state_sorted::<M, P, DEDUPE>(&preds[..W], &branch, identity, prev, out, probe);
            continue;
        }
        let mut heads = [0usize; W];
        let mut filled = 0;
        while filled < PATHS_PER_STATE {
            let mut best = W;
            for k in 0..W {
                if heads[k] < lens[k]
                    && (best == W || lists[k][heads[k]].precedes(&lists[best][heads[best]]))
                {
                    best = k;
                }
            }
            if best == W {
                break;
            }
            let cand = lists[best][heads[best]];
            heads[best] += 1;
            if DEDUPE && out[..filled].iter().any(|s| s.key == cand.key) {
                probe.duplicate();
                continue;
            }
            probe.insert_call();
            out[filled] = cand;
            filled += 1;
        }
        for s in &mut out[filled..] {
            *s = Surv::empty();
        }
    }
}

/// Coherent length 4: sixteen predecessors.
const MAX_WORDS: usize = 16;

/// One end state of [`advance`] with each extended list sorted before the merge: the exact form
/// for when `f32` rounding reordered a list.
#[cold]
fn advance_state_sorted<M: Metric, P: Probe, const DEDUPE: bool>(
    preds: &[usize],
    branch: &[M; MAX_WORDS],
    identity: u64,
    prev: &[[Surv<M>; PATHS_PER_STATE]],
    out: &mut [Surv<M>; PATHS_PER_STATE],
    probe: &mut P,
) {
    let mut lists = [[Surv::<M>::empty(); PATHS_PER_STATE]; MAX_WORDS];
    let mut lens = [0usize; MAX_WORDS];
    let mut heads = [0usize; MAX_WORDS];
    for (k, &pred) in preds.iter().enumerate() {
        let mut n = 0;
        for s in prev[pred].iter().take_while(|s| s.valid()) {
            let cand = Surv {
                metric: s.metric + branch[k],
                key: s.key | identity,
            };
            let mut j = n;
            while j > 0 && cand.precedes(&lists[k][j - 1]) {
                lists[k][j] = lists[k][j - 1];
                j -= 1;
            }
            lists[k][j] = cand;
            n += 1;
        }
        lens[k] = n;
    }
    *out = [Surv::empty(); PATHS_PER_STATE];
    let mut filled = 0;
    while filled < PATHS_PER_STATE {
        let mut best = MAX_WORDS;
        for k in 0..preds.len() {
            if heads[k] < lens[k]
                && (best == MAX_WORDS || lists[k][heads[k]].precedes(&lists[best][heads[best]]))
            {
                best = k;
            }
        }
        if best == MAX_WORDS {
            break;
        }
        let cand = lists[best][heads[best]];
        heads[best] += 1;
        if DEDUPE && out[..filled].iter().any(|s| s.key == cand.key) {
            probe.duplicate();
            continue;
        }
        probe.insert_call();
        out[filled] = cand;
        filled += 1;
    }
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

    /// Decoding into reused survivor arrays gives what fresh ones give, decode after decode.
    #[test]
    fn reused_scratch_decodes_as_fresh_arrays() {
        let mut s = 7u64;
        let mut u = move || {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((s >> 40) as f32 / (1u64 << 24) as f32) - 0.5
        };
        let plans = [Plan::new(1), Plan::new(2), Plan::new(4)];
        let mut scratch = TrellisScratch::new();
        for trial in 0..30 {
            let amp = [0.0f32, 1.0, 3.0][trial % 3];
            let z: Correlations = core::array::from_fn(|i| {
                core::array::from_fn(|k| {
                    Complex32::new(u() + if (i * 7 + trial) % 4 == k { amp } else { 0.0 }, u())
                })
            });
            for p in &plans {
                assert_eq!(
                    p.decode_f32_in(&z, true, &mut scratch),
                    p.decode_f32(&z, true)
                );
            }
        }
    }

    #[test]
    fn survivors_are_packed_to_twelve_bytes() {
        // The CoreS3's internal DRAM budget counts on it (§14 of JTTY_CORES3_APP.md).
        assert_eq!(core::mem::size_of::<Surv<f32>>(), 12);
        assert_eq!(
            core::mem::size_of::<[Surv<f32>; PATHS_PER_STATE]>() * STATES,
            24 * 1024
        );
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
