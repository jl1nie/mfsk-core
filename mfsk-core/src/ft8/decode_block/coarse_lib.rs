// SPDX-License-Identifier: GPL-3.0-only
//! The coarse search on `ft8_lib`'s statistic, for the S3's 8 kHz flow (`Ft8Rate::coarse_ft8lib`: `Hz8000`, `fixed-point`).
//!
//! **A deliberate divergence from WSJT-X** (`sync8.f90`: a sum of the Costas-tone power over the mean of the other six tones,
//! one or two peaks a carrier). Ported from `ft8_lib`'s `ft8_sync_score` and `ftx_find_candidates` (`ft8/decode.c`, MIT) as the
//! icap-sdk's `ft8dec.c` (769-830) has them: every (time, frequency) position of the slot is scored by the **mean of the dB
//! contrasts** of each Costas tone against the adjacent tones and the adjacent symbols (integer adds and subtracts, no division
//! but one per position, no noise floor to estimate), and the best positions are kept after a non-maximum suppression.
//!
//! Why: the `sync8` score is a *sum* of powers and one symbol with a lot of energy dominates it. On 60 real slots (38
//! `20m_busy` + the 22 recordings, 1 338 `jt9` decodes) of the 23 signals the windowed coarse still lost, 0 ranked among the
//! first 60 positions of their slot by that score and 22 did by this one; the S3 flow (60 candidates) went from 982 to 1 001 of the
//! 1 026 messages `ft8dec` decodes, hits 997 -> 1 011, and at 15 candidates 737 -> 761 hits (`docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md`, 11).
//!
//! The cells are the demux's power in **0.5 dB units of the 32-bit power** (`spectrogram::half_db`), not the shipped u16
//! cell: its `>> 12` throws the noise floor away, and a contrast of two floor cells is then 0. Measured the same search on
//! the u16 cell: 933-984 of the 1 026 (at best level with the `sync8` order's 982, no gain).
//!
//! Row `j` of the spectrogram is the symbol that starts at `j x nstep` (the Hann frame is centred on it, `Ft8Rate::coarse_hann_2t`),
//! so a candidate whose Costas block 0 starts at row `j0` has `dt = j0 x 0.08 - 0.5` s and symbol `s` is read on row `j0 + 2 s`.
//! The rows searched are `ft8dec`'s: `time_offset` -10..19 in half-symbol steps, DT -2.26..+2.46 s (a wider range finds real signals at
//! DT +2.6 s that WSJT-X's own window does not: they showed up as phantom-looking extras).
//!
//! Memory: one band's cells (u16, at most 193) and `n_carriers x 60` bytes of scores a band; at the S3 flow's one band the peak heap
//! was 425 216 B with the `sync8` search and 425 216..432 292 B with this one.
//!
//! The search is run a band of carriers at a time (the spectrogram of each is built, scored and dropped, as `banded_peaks`
//! does), and its result does not depend on the number of bands: each band keeps its best positions by a total order
//! (score, then carrier, then row), the union is cut to the best `RAW_FACTOR x pool` of that order and only then
//! suppressed, so every band count ends on the same list (`ft8_coarse_banded.rs`'s invariant, here at 8 kHz).

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::params::{COSTAS, NTONES};
use super::coarse_sync::HARD_SYNC_POOL;
use super::spectrogram::{
    ByteCells, Gain, compute_half_db_bytes_into, compute_half_db_bytes_src, spectrogram_gain,
    spectrogram_n_freq,
};
use super::types::{Ft8Rate, SAMPLE_RATE_HZ};
use crate::engine::source::PcmSource;
use crate::engine::sync::SyncCandidate;

/// First row searched: `ft8dec`'s `time_offset` -10 in half-symbol steps, minus the one block `ft8dec` starts its rows before
/// the symbol (`DT -2.26 s`).
const J0_MIN: i32 = -22;
/// Rows searched, `J0_MIN .. J0_MIN + NJ` (DT up to +2.46 s).
const NJ: usize = 60;
/// Lowest score kept (`ft8dec`'s `min_score`, in the same 0.5 dB units).
const MIN_SCORE: i32 = 8;
/// A band keeps, and the merge cuts to, this many positions per candidate asked for: positions beside a strong signal are
/// suppressed, so the list needs more raw positions than candidates.
const RAW_FACTOR: usize = 12;
/// Bins a carrier reads above itself: its seventh tone is 14 bins up, so the band's spectrogram runs to `k + 15` exclusive.
const MAX_TONE_OFF: usize = 2 * (NTONES - 1) + 1;

/// The whole slot's half-dB cells (`spectrogram::half_db`: 0.5 dB units of the 32-bit power, at most 193, so a byte), `n_time` rows of
/// `n_freq` bins of 3.125 Hz from bin `first_bin`, row-major. What `ft8_lib` and `ft8dec` decode from: the search scores them and the
/// symbol spectra of a candidate are read from them, with no DFT per candidate (`FT8_WINDOW_AND_COARSE_DESIGN.md`, 14-20).
///
/// Made by [`coarse_sync_gated_cells`] as bytes by the demux (no `u16` cell in between): at one band the band **is** the store, 172 KB of cells at the
/// full 200-3000 Hz band, 184 rows; with several bands each band's bytes are copied into it, and the overlap between two bands (the 14 bins a
/// carrier's top tone reads above it) is written twice with the same values.
#[derive(Clone, Debug)]
pub struct HalfDbCells {
    n_freq: usize,
    n_time: usize,
    /// Rows written so far (a producer that makes the store a row at a time on the way: [`row_mut`](Self::row_mut), [`mark_filled`](Self::mark_filled)); a row at or past it
    /// is not measured, and neither [`cell`](Self::cell) nor the search reads it.
    filled: usize,
    first_bin: usize,
    rate: Ft8Rate,
    data: Vec<u8>,
}

impl HalfDbCells {
    /// Cells made by hand (the tests of `cells_flow` build the cells they read).
    #[cfg(test)]
    pub(super) fn from_parts(
        n_freq: usize,
        n_time: usize,
        first_bin: usize,
        rate: Ft8Rate,
        data: Vec<u8>,
    ) -> Self {
        assert_eq!(data.len(), n_freq * n_time);
        Self {
            n_freq,
            n_time,
            filled: n_time,
            first_bin,
            rate,
            data,
        }
    }

    /// An empty store (zeroed, **no row filled**) for a producer that writes it a row at a time: `n_time` rows of `n_freq` bins of 3.125 Hz from bin `first_bin`
    /// (172 224 B at 200-3 000 Hz, 184 rows: [`row_len`](Self::row_len) bins from [`carrier_bins`](Self::carrier_bins)'s first to its last plus 14).
    pub fn new(rate: Ft8Rate, first_bin: usize, n_freq: usize, n_time: usize) -> Self {
        Self {
            n_freq,
            n_time,
            filled: 0,
            first_bin,
            rate,
            data: alloc::vec![0u8; n_freq * n_time],
        }
    }

    /// Bins per row.
    pub fn row_len(&self) -> usize {
        self.n_freq
    }

    /// Row `row` for writing: `n_freq` bytes, the half-dB cells of one frame. Rows are written in order and then [`mark_filled`](Self::mark_filled).
    pub fn row_mut(&mut self, row: usize) -> &mut [u8] {
        &mut self.data[row * self.n_freq..(row + 1) * self.n_freq]
    }

    /// Rows `j` and `j + 1` for writing, one after the other (`n_freq` bytes each): the pair one transform makes.
    pub fn row_pair_mut(&mut self, j: usize) -> (&mut [u8], &mut [u8]) {
        let n = self.n_freq;
        self.data[j * n..(j + 2) * n].split_at_mut(n)
    }

    /// Empty the store for the next slot: every cell zero, no row filled (the memory is kept).
    pub fn clear(&mut self) {
        self.data.fill(0);
        self.filled = 0;
    }

    /// Declare the first `rows` rows written (at most `n_time`): from here the search and [`cell`](Self::cell) read them.
    pub fn mark_filled(&mut self, rows: usize) {
        self.filled = rows.min(self.n_time);
    }

    /// Rows written so far.
    pub fn filled_rows(&self) -> usize {
        self.filled
    }

    /// The carrier bins the store can score: a carrier's tone 7 is 14 bins above it, so the last 14 bins of a row only serve as tones.
    pub fn carrier_bins(&self) -> core::ops::Range<usize> {
        self.first_bin
            ..(self.first_bin + self.n_freq)
                .saturating_sub(MAX_TONE_OFF - 1)
                .max(self.first_bin)
    }

    /// Bins per row.
    pub fn n_freq(&self) -> usize {
        self.n_freq
    }

    /// Rows: the symbol that starts at row `j` starts at `j x nstep` samples (`0.08 s` at 8 kHz).
    pub fn n_time(&self) -> usize {
        self.n_time
    }

    /// The spectrogram bin (3.125 Hz) of column 0.
    pub fn first_bin(&self) -> usize {
        self.first_bin
    }

    /// The rate the cells were made at.
    pub fn rate(&self) -> Ft8Rate {
        self.rate
    }

    /// The cell of row `row` at the **spectrogram bin** `bin` (the frequency is `bin x 3.125 Hz`), `None` outside the store.
    pub fn cell(&self, row: usize, bin: usize) -> Option<u8> {
        let col = bin.checked_sub(self.first_bin)?;
        if row >= self.filled || col >= self.n_freq {
            return None;
        }
        Some(self.data[row * self.n_freq + col])
    }

    /// Bytes the cells take.
    pub fn size_bytes(&self) -> usize {
        self.data.len()
    }
}

/// A borrowed view of cells for the scoring: `n_time` is the number of rows **measured** (a row at or past it is "off the slot": `score` stops at it and `hard_sync`
/// skips it, as they do at the end of a slot), `data` row-major.
struct CellsRef<'a> {
    n_freq: usize,
    n_time: usize,
    data: &'a [u8],
}

fn view_of(cells: &ByteCells) -> CellsRef<'_> {
    CellsRef {
        n_freq: cells.n_freq,
        n_time: cells.n_time,
        data: &cells.data,
    }
}

impl HalfDbCells {
    /// The rows filled so far, as a view for the scoring.
    fn view(&self) -> CellsRef<'_> {
        CellsRef {
            n_freq: self.n_freq,
            n_time: self.filled,
            data: &self.data[..self.filled * self.n_freq],
        }
    }
}

/// The order candidates are returned in. `Score`: `ft8_lib`'s (best first). `PoolThenHardSync`: the best `HARD_SYNC_POOL`
/// by score, then by their hard-sync count (the gated search's, `coarse_sync_gated`): see `ORDER`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Order {
    #[allow(dead_code)] // the other order, measured against `ORDER` (and used by the tests)
    Score,
    PoolThenHardSync,
}

/// The best positions by score the hard-sync order is taken over (`coarse_sync_gated`'s `HARD_SYNC_POOL`).
const POOL: usize = HARD_SYNC_POOL;

/// What the flow uses, decided by measurement (60 slots, hits at 10 / 15 / 20 / 30 / 60 candidates): `Score` 514 / 733 / 877 / 1 001 / 1 011,
/// `PoolThenHardSync` 534 / 761 / 919 / 1 009 / 1 011 (the pool of 100 and of 600 the same within 1 hit; 1 010 at 30 candidates with 100): the hard-sync order keeps the
/// short lists good, where `ft8_lib`'s own order is below the `sync8` order at 15 and 20 candidates.
pub(super) const ORDER: Order = Order::PoolThenHardSync;

/// One scored position: the carrier bin of tone 0, the row of Costas symbol 0, its score and its hard-sync count.
#[derive(Clone, Copy, Debug)]
struct Pos {
    score: i32,
    k: usize,
    j: i32,
    hs: u8,
}

/// The total order the selection is made in: best score first, then the lower carrier, then the earlier row.
fn better(a: &Pos, b: &Pos) -> core::cmp::Ordering {
    b.score
        .cmp(&a.score)
        .then(a.k.cmp(&b.k))
        .then(a.j.cmp(&b.j))
}

/// `ft8_sync_score` of the position (carrier bin `rel_k` of `cells`, row `j0`): the mean over the Costas symbols of the
/// Costas tone's dB minus the adjacent tones' and the adjacent symbols' (the previous and the next row of the same
/// tone), integer, divided once by the number of terms (`ft8_lib`'s `num`; a symbol off the slot adds none).
///
/// This is the **reference** the search is checked against (`carrier_scores` must give its values for every position): the search itself scores a carrier at a time.
#[cfg(test)]
#[inline]
fn score(cells: &CellsRef<'_>, rel_k: usize, j0: i32) -> i32 {
    let (nf, rows) = (cells.n_freq, cells.n_time);
    let d = &cells.data;
    let (mut sum, mut num) = (0i32, 0i32);
    for m in 0..3 {
        for (k, &sm) in COSTAS.iter().enumerate() {
            let row = j0 + 2 * (36 * m + k) as i32;
            if row < 0 {
                continue;
            }
            let r = row as usize;
            if r >= rows {
                break;
            }
            let base = r * nf + rel_k + 2 * sm;
            let c = d[base] as i32;
            if sm > 0 {
                sum += c - d[base - 2] as i32;
                num += 1;
            }
            if sm < NTONES - 1 {
                sum += c - d[base + 2] as i32;
                num += 1;
            }
            if k > 0 && r >= 2 {
                sum += c - d[base - 2 * nf] as i32;
                num += 1;
            }
            if k + 1 < COSTAS.len() && r + 2 < rows {
                sum += c - d[base + 2 * nf] as i32;
                num += 1;
            }
        }
    }
    if num > 0 { sum / num } else { sum }
}

/// The number of Costas symbols (of 21) whose strongest of the eight tone cells is the Costas tone, on the rows the slot has:
/// `spec_hard_sync`'s count on these cells.
fn hard_sync(cells: &CellsRef<'_>, rel_k: usize, j0: i32) -> u8 {
    let (nf, rows) = (cells.n_freq, cells.n_time);
    let mut ok = 0u8;
    for m in 0..3 {
        for (k, &sm) in COSTAS.iter().enumerate() {
            let row = j0 + 2 * (36 * m + k) as i32;
            if row < 0 || row as usize >= rows {
                continue;
            }
            let base = row as usize * nf + rel_k;
            let mut best = 0usize;
            for t in 1..NTONES {
                if cells.data[base + 2 * t] > cells.data[base + 2 * best] {
                    best = t;
                }
            }
            if best == sm {
                ok += 1;
            }
        }
    }
    ok
}

/// The terms one Costas symbol adds to a position's score, for the symbol `k` of the pattern read at row `r` of the carrier at bin `rel_k`: `(sum, count)` of `score`'s: the
/// Costas tone's cell minus the cell of the tone below (not for tone 0), of the tone above, of the same tone two rows before (not for the first symbol of a block, nor before row 2) and
/// two rows after (not for the last, nor at the last two rows of the slot). They depend on the carrier, the row and the symbol only, **not on the position's row**, which is what lets
/// a carrier compute them once for every row and read them back 60 times.
#[inline]
fn symbol_terms(d: &[u8], nf: usize, rows: usize, rel_k: usize, r: usize, k: usize) -> (i32, u8) {
    let sm = COSTAS[k];
    let base = r * nf + rel_k + 2 * sm;
    let c = d[base] as i32;
    let (mut sum, mut num) = (0i32, 0u8);
    if sm > 0 {
        sum += c - d[base - 2] as i32;
        num += 1;
    }
    if sm < NTONES - 1 {
        sum += c - d[base + 2] as i32;
        num += 1;
    }
    if k > 0 && r >= 2 {
        sum += c - d[base - 2 * nf] as i32;
        num += 1;
    }
    if k + 1 < COSTAS.len() && r + 2 < rows {
        sum += c - d[base + 2 * nf] as i32;
        num += 1;
    }
    (sum, num)
}

/// The stride of the per-row table of [`carrier_scores`]: seven Costas symbols, padded to eight.
const SYM_STRIDE: usize = 8;
/// A table entry is `COUNT_RADIX * sum + num` (`num` is at most 4 terms a symbol, 84 for 21 symbols): one load and one add a symbol, and `acc >> 7` / `acc & 127` split the total back.
const COUNT_RADIX: i32 = 128;
const _: () = assert!(COUNT_RADIX == 128 && 21 * 4 < COUNT_RADIX);

/// The scores of the [`NJ`] positions of the carrier at bin `rel_k` of `cells` (`out[jj]` is `score(cells, rel_k, J0_MIN + jj)`, **to the integer**: the same terms, the same
/// truncating division, the same positions off the slot skipped), by `ft8_sync_score`'s sum **regrouped**: the terms of a symbol do not depend on the position (`symbol_terms`), so they are made
/// once for every row and symbol of the carrier into `w` (`rows x SYM_STRIDE`), and each position adds up its 21 symbols' entries. A position cost about 1 200 host instructions
/// with 260 conditional branches before (the CoreS3 spends about 4 000 cycles on it: the scoring is CPU-bound, not memory-bound, measured with narrow and whole ranges).
fn carrier_scores(cells: &CellsRef<'_>, rel_k: usize, w: &mut [i32], out: &mut [i32; NJ]) {
    let (nf, rows) = (cells.n_freq, cells.n_time);
    let d = cells.data;
    debug_assert!(w.len() >= rows * SYM_STRIDE);
    // rows 2 ..= rows - 3 have every neighbour row: one 15-byte window a row (tones 0..=7 are 14 bins) and no per-symbol test
    let interior = 2..rows.saturating_sub(2);
    for r in 0..rows {
        if interior.contains(&r) {
            let win = |row: usize| -> &[u8; 15] { d[row * nf + rel_k..][..15].try_into().unwrap() };
            let (cur, back, fwd) = (win(r), win(r - 2), win(r + 2));
            for (k, &sm) in COSTAS.iter().enumerate() {
                let c = cur[2 * sm] as i32;
                let mut sum = 0i32;
                let mut num = 0u8;
                if sm > 0 {
                    sum += c - cur[2 * sm - 2] as i32;
                    num += 1;
                }
                if sm < NTONES - 1 {
                    sum += c - cur[2 * sm + 2] as i32;
                    num += 1;
                }
                if k > 0 {
                    sum += c - back[2 * sm] as i32;
                    num += 1;
                }
                if k + 1 < COSTAS.len() {
                    sum += c - fwd[2 * sm] as i32;
                    num += 1;
                }
                w[r * SYM_STRIDE + k] = sum * COUNT_RADIX + num as i32;
            }
        } else {
            for k in 0..COSTAS.len() {
                let (sum, num) = symbol_terms(d, nf, rows, rel_k, r, k);
                w[r * SYM_STRIDE + k] = sum * COUNT_RADIX + num as i32;
            }
        }
    }
    let rows_i = rows as i32;
    for (jj, o) in out.iter_mut().enumerate() {
        let j0 = J0_MIN + jj as i32;
        let mut acc = 0i32;
        for m in 0..3i32 {
            // the row of the block's first symbol; symbol k is at b + 2k, on the slot while 0 <= b + 2k < rows
            let b = j0 + 72 * m;
            let k_lo = if b >= 0 { 0 } else { (-b + 1) / 2 };
            let k_hi = if rows_i - b <= 0 {
                0
            } else {
                ((rows_i - b + 1) / 2).min(7)
            };
            for k in k_lo..k_hi {
                acc += w[(b + 2 * k) as usize * SYM_STRIDE + k as usize];
            }
        }
        // acc = COUNT_RADIX * sum + num with 0 <= num <= 84 < COUNT_RADIX: the floor division and the remainder give them back
        let (sum, num) = (acc >> 7, acc & (COUNT_RADIX - 1));
        *o = if num > 0 { sum / num } else { sum };
    }
}

/// Score every position of the carriers `k_first..=k_last` (bins of the full spectrogram; `cells` starts at bin `lo`) and
/// return the best `keep_raw` of them by [`better`] (ties at the cut are all kept: the cut is by score, then trimmed by the
/// total order), each with its hard-sync count.
fn band_best(
    cells: &CellsRef<'_>,
    lo: usize,
    k_first: usize,
    k_last: usize,
    keep_raw: usize,
    allowed: Option<&dyn Fn(usize) -> bool>,
) -> Vec<Pos> {
    let n_car = k_last - k_first + 1;
    // scores above 127 are clamped to 127 (a tie of very strong signals is broken by carrier and row, not by the peak: none is that
    // strong in the corpora, where the contrasts against the adjacent symbols keep a clean signal near 30-60)
    let mut scores = alloc::vec![i8::MIN; n_car * NJ];
    // the scores 0..=127 only matter above MIN_SCORE: a histogram finds the cut without sorting every position above the minimum (hundreds a slot)
    let mut hist = [0u32; 128];
    // the per-row terms of one carrier (`carrier_scores`), made once per call and rewritten for each carrier
    let mut w = alloc::vec![0i32; cells.n_time * SYM_STRIDE];
    let mut carrier = [0i32; NJ];
    for ci in 0..n_car {
        // a carrier that is not allowed is not scored (`ft8dec.c` 833-841: after a subtraction only the bins next to a new signal)
        if allowed.is_some_and(|f| !f(k_first + ci)) {
            continue;
        }
        let rel_k = k_first + ci - lo;
        carrier_scores(cells, rel_k, &mut w, &mut carrier);
        for (jj, &s) in carrier.iter().enumerate() {
            if s >= MIN_SCORE {
                let s = s.min(127);
                scores[ci * NJ + jj] = s as i8;
                hist[s as usize] += 1;
            }
        }
    }
    let mut thr = MIN_SCORE;
    let mut cum = 0usize;
    for s in (MIN_SCORE as usize..128).rev() {
        cum += hist[s] as usize;
        if cum >= keep_raw {
            thr = s as i32;
            break;
        }
    }
    let mut out = Vec::new();
    for ci in 0..n_car {
        for jj in 0..NJ {
            let s = scores[ci * NJ + jj];
            if s != i8::MIN && s as i32 >= thr {
                let (k, j) = (k_first + ci, J0_MIN + jj as i32);
                out.push(Pos {
                    score: s as i32,
                    k,
                    j,
                    hs: hard_sync(cells, k - lo, j),
                });
            }
        }
    }
    out.sort_by(better);
    out.truncate(keep_raw);
    out
}

/// The merge of the bands' best positions: the best `keep_raw` of all of them by [`better`]. Each band kept its own best `keep_raw`
/// by the same total order, so this is the best `keep_raw` of every position of the slot whatever the number of bands.
fn merge(mut all: Vec<Pos>, keep_raw: usize) -> Vec<Pos> {
    all.sort_by(better);
    all.truncate(keep_raw);
    all
}

/// Non-maximum suppression, +-2 bins and +-2 rows (`ft8dec` has none: it decodes up to 300 and skips duplicates after a
/// success; a list of 15-60 has no slot for the neighbours of a strong signal): walk `sorted` (best first) and keep a position
/// with no kept one within 2 bins and 2 rows, until `limit` are kept.
fn suppress(sorted: &[Pos], limit: usize) -> Vec<Pos> {
    let mut kept: Vec<Pos> = Vec::with_capacity(limit.min(sorted.len()));
    for p in sorted {
        if kept
            .iter()
            .any(|q| (q.j - p.j).abs() <= 2 && (q.k as i64 - p.k as i64).abs() <= 2)
        {
            continue;
        }
        kept.push(*p);
        if kept.len() >= limit {
            break;
        }
    }
    kept
}

/// The search over `src`: the candidates (tone-0 frequency, DT, the score as `score`) with the hard-sync count, in `order`, at
/// most `keep`. `freq_min .. freq_max` is the carriers' band, `bands` the number of bands the spectrogram is built in.
pub(super) fn search<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    freq_min: f32,
    freq_max: f32,
    keep: usize,
    bands: usize,
    order: Order,
) -> Vec<(SyncCandidate, u8)> {
    search_impl(
        src, rate, freq_min, freq_max, keep, bands, order, false, None, None,
    )
    .0
}

/// The best `keep_raw` of the scored positions of every band, suppressed, in `order`, at most `keep`, as candidates of `rate`: what the search returns once the bands are scored. The
/// same function whether one core scored the whole range or two scored a half each.
fn finish(
    all: Vec<Pos>,
    keep_raw: usize,
    pool: usize,
    keep: usize,
    order: Order,
    rate: Ft8Rate,
) -> Vec<(SyncCandidate, u8)> {
    let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
    let tstep = rate.nstep() as f32 / rate.hz() as f32;
    let all = merge(all, keep_raw);
    let mut kept = suppress(&all, pool);
    if order == Order::PoolThenHardSync {
        // stable: equal counts stay in score order
        kept.sort_by_key(|p| core::cmp::Reverse(p.hs));
    }
    kept.truncate(keep);
    kept.into_iter()
        .map(|p| {
            (
                SyncCandidate {
                    freq_hz: p.k as f32 * df,
                    dt_sec: p.j as f32 * tstep - 0.5,
                    score: p.score as f32,
                },
                p.hs,
            )
        })
        .collect()
}

/// [`search`] with the whole slot's cells kept (`keep_cells`): the same candidates, the cells the search scored in a [`HalfDbCells`].
#[allow(clippy::too_many_arguments)]
/// The carrier bins `ia..=ib` (3.125 Hz) the search of `freq_min..freq_max` scores, `None` for an empty band: the one place the store's shape comes from, for the search and for
/// [`CellsProducer`](super::CellsProducer).
pub(super) fn carrier_bins_of(
    freq_min: f32,
    freq_max: f32,
    rate: Ft8Rate,
) -> Option<(usize, usize)> {
    let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
    let spec_n_freq = spectrogram_n_freq(freq_max, rate);
    let ia = (freq_min / df).round() as usize;
    let ib = ((freq_max / df).round() as usize).min(spec_n_freq.saturating_sub(MAX_TONE_OFF));
    (ib >= ia).then_some((ia, ib))
}

/// Tone bins above a carrier's own that its row must hold (`2 (NTONES - 1) + 1`).
pub(super) const TONE_BINS: usize = MAX_TONE_OFF;

fn search_impl<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    freq_min: f32,
    freq_max: f32,
    keep: usize,
    bands: usize,
    order: Order,
    keep_cells: bool,
    allowed: Option<&dyn Fn(usize) -> bool>,
    gain: Option<Gain>,
) -> (Vec<(SyncCandidate, u8)>, Option<HalfDbCells>) {
    let Some((ia, ib)) = carrier_bins_of(freq_min, freq_max, rate) else {
        return (Vec::new(), None);
    };
    if keep == 0 {
        return (Vec::new(), None);
    }
    let pool = match order {
        Order::Score => keep,
        Order::PoolThenHardSync => POOL.max(keep),
    };
    let keep_raw = RAW_FACTOR * pool;
    // the slot's gain: read from the source unless the caller has it (a residual is not read twice, and its peak is the audio's)
    let gain = gain.unwrap_or_else(|| spectrogram_gain(src));
    let n_car = ib - ia + 1;
    let bands = bands.clamp(1, n_car);
    let mut all: Vec<Pos> = Vec::new();
    let mut store: Option<HalfDbCells> = None;
    for b in 0..bands {
        let a = ia + b * n_car / bands;
        let z = ia + (b + 1) * n_car / bands - 1;
        let cells = compute_half_db_bytes_src(src, rate, a, z + MAX_TONE_OFF, gain);
        all.extend(band_best(&view_of(&cells), a, a, z, keep_raw, allowed));
        if keep_cells {
            let total = ib + MAX_TONE_OFF - ia;
            if bands == 1 {
                // the band is the whole store: no copy
                store = Some(HalfDbCells {
                    n_freq: cells.n_freq,
                    n_time: cells.n_time,
                    filled: cells.n_time,
                    first_bin: ia,
                    rate,
                    data: cells.data,
                });
            } else {
                let store = store.get_or_insert_with(|| HalfDbCells {
                    n_freq: total,
                    n_time: cells.n_time,
                    filled: cells.n_time,
                    first_bin: ia,
                    rate,
                    data: alloc::vec![0u8; total * cells.n_time],
                });
                for r in 0..cells.n_time {
                    let (from, to) = (r * cells.n_freq, r * total + (a - ia));
                    store.data[to..to + cells.n_freq]
                        .copy_from_slice(&cells.data[from..from + cells.n_freq]);
                }
            }
        }
    }
    let found = finish(all, keep_raw, pool, keep, order, rate);
    (found, store)
}

/// [`coarse_sync_gated`](super::coarse_sync_gated)'s candidates at `Hz8000` in the `fixed-point` build (`Ft8Rate::coarse_ft8lib`: the
/// same list, the same order) **and the cells the search scored**, as a [`HalfDbCells`]: what a flow that reads the symbols from the cells
/// instead of a per-candidate DFT needs. `None` at any other rate. The audio is read forward `bands` times (the cells are made a band of
/// carriers at a time) plus once for the slot's peak; the candidates do not depend on `bands`, and neither do the cells.
pub fn coarse_sync_gated_cells<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    search: &super::coarse_sync::GatedSearch,
    bands: usize,
) -> Option<(Vec<(SyncCandidate, u8)>, HalfDbCells)> {
    if !rate.coarse_ft8lib() {
        return None;
    }
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::CoarseSync);
    let (found, store) = search_impl(
        src,
        rate,
        search.freq_min,
        search.freq_max,
        search.keep,
        bands,
        ORDER,
        true,
        None,
        None,
    );
    // no cells were made (an empty band, `freq_max` under `freq_min`, or `keep` of 0): an empty store
    let store = store.unwrap_or(HalfDbCells {
        n_freq: 0,
        n_time: 0,
        filled: 0,
        first_bin: 0,
        rate,
        data: Vec::new(),
    });
    Some((found, store))
}

/// The scored positions of a range of carriers of a store, to be merged with the other ranges' by [`merge_cells_parts`]: one core scores the lower half of the carriers
/// and the other the upper half (the old coarse search splits the same way), the merge is the one that makes the list **the same for any split**.
pub struct CellsPart(Vec<Pos>);

/// Score the carriers `carriers` (absolute bins of 3.125 Hz, clipped to [`HalfDbCells::carrier_bins`]) of `store` **over the rows filled so far**: a row not yet written is
/// not read (the search of a store that is still being made, at the board's early emit). The scores need `(carriers.len()) x 60` bytes while this runs (56 KB for the whole range).
pub fn search_cells(
    store: &HalfDbCells,
    search: &super::coarse_sync::GatedSearch,
    carriers: core::ops::Range<usize>,
) -> CellsPart {
    let range = store.carrier_bins();
    let (lo, hi) = (carriers.start.max(range.start), carriers.end.min(range.end));
    if lo >= hi || store.filled == 0 || search.keep == 0 {
        return CellsPart(Vec::new());
    }
    let keep_raw = RAW_FACTOR * POOL.max(search.keep);
    CellsPart(band_best(
        &store.view(),
        store.first_bin,
        lo,
        hi - 1,
        keep_raw,
        None,
    ))
}

/// The candidates of the parts of one search, in the search's order (`ft8_lib`'s statistic, the hard-sync order over the best pool, at most `search.keep`): **the list
/// [`coarse_sync_gated_cells`] returns for the same store whatever the number of parts and where they are cut** (a test pins it: 1, 2, 3 and 7 parts).
pub fn merge_cells_parts(
    parts: &[CellsPart],
    search: &super::coarse_sync::GatedSearch,
    rate: Ft8Rate,
) -> Vec<(SyncCandidate, u8)> {
    let pool = POOL.max(search.keep);
    let all: Vec<Pos> = parts.iter().flat_map(|p| p.0.iter().copied()).collect();
    finish(all, RAW_FACTOR * pool, pool, search.keep, ORDER, rate)
}

/// Rows a candidate's decode reads past the row of its first symbol: its 79 symbols are every second row (`2 x 78`) and the retry reads the row after the last.
const ROWS_READ: i32 = 2 * 78 + 1;

/// Whether every row the decode of `cand` reads is already written in `store` (`j0 + 157 < filled_rows`): the early / late split of the board's emit, which decodes the
/// candidates whose symbols have all arrived and defers the rest to the slot's end.
pub fn candidate_ready(store: &HalfDbCells, cand: &SyncCandidate) -> bool {
    let tstep = store.rate.nstep() as f32 / store.rate.hz() as f32;
    let j0 = ((cand.dt_sec + 0.5) / tstep).round() as i32;
    j0 + ROWS_READ < store.filled as i32
}

/// The search of a **residual** (the audio minus the signals decoded so far: `ft8dec_sic::ResidualSource`) restricted to the carriers within
/// `radius` cells of one of `near` (Hz): `ft8dec.c` 833-841 (`binok`, +-9 bins of 6.25 Hz = 18 cells of 3.125 Hz) after a subtraction, only where something
/// changed. One band; `gain` is the slot's (the original audio's), so the residual is not read a second time for its peak. The candidates are the best by the
/// ordinary rule among the allowed carriers, and the cells of the residual are written **into the caller's store**: the cells of the residual are written into `store` (the one of the previous pass, which a board keeps: 172 KB, made once)
/// and the candidates are returned. `store` must be the store [`coarse_sync_gated_cells`] made for the same `rate` and `search` (the same carriers and rows: the band is the
/// same), whatever it held: the cells are written whole, and `filled_rows` is the slot's. An empty `store` (a search with nothing to score) gives no candidates.
pub(super) fn search_near_into<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    search: &super::coarse_sync::GatedSearch,
    gain: Gain,
    near: &[f32],
    radius: usize,
    store: &mut HalfDbCells,
) -> Vec<(SyncCandidate, u8)> {
    let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
    let centres: Vec<i64> = near.iter().map(|&f| (f / df).round() as i64).collect();
    let allowed = |k: usize| {
        centres
            .iter()
            .any(|&c| (k as i64 - c).unsigned_abs() as usize <= radius)
    };
    // the same band as `search_impl` makes at one band
    let spec_n_freq = spectrogram_n_freq(search.freq_max, rate);
    let ia = (search.freq_min / df).round() as usize;
    let ib =
        ((search.freq_max / df).round() as usize).min(spec_n_freq.saturating_sub(MAX_TONE_OFF));
    if ib < ia || search.keep == 0 || store.n_freq == 0 {
        return Vec::new();
    }
    assert!(
        store.rate == rate && store.first_bin == ia && store.n_freq == ib + MAX_TONE_OFF - ia,
        "search_near_into: the store is not the one of this search"
    );
    let pool = POOL.max(search.keep);
    let keep_raw = RAW_FACTOR * pool;
    let (n_freq, n_time) =
        compute_half_db_bytes_into(src, rate, ia, ib + MAX_TONE_OFF, gain, &mut store.data);
    debug_assert_eq!((n_freq, n_time), (store.n_freq, store.n_time));
    store.filled = store.n_time;
    let all = band_best(&store.view(), ia, ia, ib, keep_raw, Some(&allowed));
    finish(all, keep_raw, pool, search.keep, ORDER, rate)
}

/// The allocating form of [`search_near_into`] (candidates and a fresh store), kept for the tests: the reference the in-place one must equal.
#[cfg(test)]
pub(super) fn search_near<A: PcmSource>(
    src: &mut A,
    rate: Ft8Rate,
    search: &super::coarse_sync::GatedSearch,
    gain: Gain,
    near: &[f32],
    radius: usize,
) -> (Vec<(SyncCandidate, u8)>, HalfDbCells) {
    let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
    let centres: Vec<i64> = near.iter().map(|&f| (f / df).round() as i64).collect();
    let allowed = |k: usize| {
        centres
            .iter()
            .any(|&c| (k as i64 - c).unsigned_abs() as usize <= radius)
    };
    let (found, store) = search_impl(
        src,
        rate,
        search.freq_min,
        search.freq_max,
        search.keep,
        1,
        ORDER,
        true,
        Some(&allowed),
        Some(gain),
    );
    let store = store.unwrap_or(HalfDbCells {
        n_freq: 0,
        n_time: 0,
        filled: 0,
        first_bin: 0,
        rate,
        data: Vec::new(),
    });
    (found, store)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::source::SliceSource;
    use crate::engine::tx::{message_to_tones, synthesize_i16};
    use crate::ft8::Ft8;
    use crate::msg::wsjt77::pack77;

    /// A slot at 8 kHz with the signals `(call1, call2, report, freq, dt, amplitude)` and LCG noise of sigma 1 500.
    fn slot(signals: &[(&str, &str, &str, f32, f32, i16)], seed: u64) -> Vec<i16> {
        let hz = 8000u32;
        let mut v = alloc::vec![0f32; 15 * hz as usize];
        for &(a, b, c, f, dt, amp) in signals {
            let m77 = pack77(a, b, c).expect("pack");
            let tones = message_to_tones::<Ft8>(&m77);
            let pcm = synthesize_i16::<Ft8>(&tones, hz, f, amp);
            let at = ((0.5 + dt) * hz as f32).round() as i64;
            for (i, &s) in pcm.iter().enumerate() {
                let idx = at + i as i64;
                if idx >= 0 && (idx as usize) < v.len() {
                    v[idx as usize] += s as f32;
                }
            }
        }
        let mut s = seed;
        v.iter()
            .map(|&x| {
                let mut g = 0.0f32;
                for _ in 0..12 {
                    s = s
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    g += ((s >> 40) as f32) / (1u64 << 24) as f32;
                }
                (x + (g - 6.0) * 1500.0).round().clamp(-32768.0, 32767.0) as i16
            })
            .collect()
    }

    fn run(slot: &[i16], keep: usize, bands: usize, order: Order) -> Vec<(SyncCandidate, u8)> {
        search(
            &mut SliceSource(slot),
            Ft8Rate::Hz8000,
            200.0,
            3000.0,
            keep,
            bands,
            order,
        )
    }

    /// The clean signals of a slot are the best candidates, where they are (the tone-0 frequency within one bin, DT within one
    /// row), **with a strong signal 25 Hz away** (the case the rectangular frame's sidelobes lost), and their hard-sync count
    /// is the whole 21. Fails if the score reads the wrong cells (the adjacent-tone or adjacent-symbol term has the wrong
    /// sign or row), the row of a candidate is not `j0 x 0.08 - 0.5`, or the frequency is not `k x df`.
    #[test]
    fn the_signals_of_a_slot_are_the_best_candidates() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 1000.0f32, 0.31f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1025.0, -0.45, 1500),
            ("CQ", "JA1ABC", "PM95", 2100.0, 1.1, 1500),
        ];
        let s = slot(&signals, 3);
        let got = run(&s, 10, 1, Order::Score);
        // no two candidates are neighbours (within 2 bins and 2 rows): the strong signal's skirt is one candidate
        for (i, a) in got.iter().enumerate() {
            for b in &got[i + 1..] {
                let near = (a.0.freq_hz - b.0.freq_hz).abs() <= 2.0 * 3.125 + 0.01
                    && (a.0.dt_sec - b.0.dt_sec).abs() <= 2.0 * 0.08 + 0.001;
                assert!(!near, "{:?} and {:?} are neighbours", a.0, b.0);
            }
        }
        for &(_, _, _, f, dt, amp) in &signals {
            let hit = got
                .iter()
                .position(|(c, _)| (c.freq_hz - f).abs() <= 3.2 && (c.dt_sec - dt).abs() <= 0.09)
                .unwrap_or_else(|| panic!("{f} Hz, dt {dt}: not among {got:?}"));
            assert!(hit < 6, "{f} Hz: rank {hit}");
            // the strong signal reads (nearly) all 21 Costas symbols; the weaker ones, in noise of the same sigma, far more than the
            // 2 or 3 of 21 that noise gives
            let want = if amp >= 6000 { 19 } else { 10 };
            assert!(got[hit].1 >= want, "{f} Hz: hard sync {}", got[hit].1);
        }
    }

    /// The list does not depend on the number of bands, to the bit, at 8 kHz. Fails if a carrier's score reads another band's
    /// cells, or the cut and the suppression depend on how the positions arrive.
    #[test]
    fn the_candidates_do_not_depend_on_the_number_of_bands() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 3000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 2500),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 2000),
        ];
        let s = slot(&signals, 5);
        for order in [Order::Score, Order::PoolThenHardSync] {
            let key = |v: &[(SyncCandidate, u8)]| {
                v.iter()
                    .map(|(c, h)| {
                        (
                            c.freq_hz.to_bits(),
                            c.dt_sec.to_bits(),
                            c.score.to_bits(),
                            *h,
                        )
                    })
                    .collect::<Vec<_>>()
            };
            let one = key(&run(&s, 30, 1, order));
            assert!(!one.is_empty());
            for bands in [2usize, 4, 7] {
                assert_eq!(
                    one,
                    key(&run(&s, 30, bands, order)),
                    "{order:?}, {bands} bands"
                );
            }
        }
    }

    /// The store holds the demux's cells: every cell of `HalfDbCells`, for 1, 2, 4 and 7 bands, is `half_db` of the whole-range
    /// demux's cell for the same frame (a band boundary writes the overlap twice with the same values); the largest cell of the slot is above 100
    /// and at most 193. **Fails if** a band is written at the wrong column or row, the overlap differs, the narrowing clamps below 100 (a
    /// clamp at 180 would pass: both sides are the same byte demux, only the range check guards it), or the store's `first_bin` / `cell` indexing is off by one.
    #[test]
    fn the_store_holds_the_demuxs_cells_for_any_number_of_bands() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 5000),
        ];
        let s = slot(&signals, 11);
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let n_freq = spectrogram_n_freq(3000.0, Ft8Rate::Hz8000);
        let ia = (200.0f32 / df).round() as usize;
        let ib = ((3000.0f32 / df).round() as usize).min(n_freq - MAX_TONE_OFF);
        let mut src = SliceSource(&s);
        let gain = spectrogram_gain(&mut src);
        let whole =
            compute_half_db_bytes_src(&mut src, Ft8Rate::Hz8000, ia, ib + MAX_TONE_OFF, gain);
        let mut most = 0u8;
        for bands in [1usize, 2, 4, 7] {
            let (found, store) = coarse_sync_gated_cells(
                &mut SliceSource(&s),
                Ft8Rate::Hz8000,
                &super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 20),
                bands,
            )
            .expect("8 kHz fixed-point");
            assert!(!found.is_empty());
            assert_eq!(store.first_bin(), ia);
            assert_eq!(store.n_time(), whole.n_time);
            assert_eq!(store.n_freq(), whole.n_freq);
            assert_eq!(store.size_bytes(), whole.data.len());
            for row in 0..whole.n_time {
                for bin in ia..ib + MAX_TONE_OFF {
                    let want = whole.data[row * whole.n_freq + (bin - ia)];
                    most = most.max(want);
                    assert_eq!(
                        store.cell(row, bin),
                        Some(want as u8),
                        "{bands} bands, row {row}, bin {bin}"
                    );
                }
            }
            assert_eq!(store.cell(whole.n_time, ia), None);
            assert_eq!(store.cell(0, ia - 1), None);
            assert_eq!(store.cell(0, ib + MAX_TONE_OFF), None);
        }
        assert!(most > 100 && most <= 193, "largest cell {most}");
    }

    /// `coarse_sync_gated_cells` returns `coarse_sync_gated`'s list at 8 kHz fixed-point, bit for bit (frequency, DT, score, hard-sync
    /// count, order), whatever the number of bands, and `None` at 12 kHz. **Fails if** the cells-keeping path changes a candidate, or
    /// the rate gate is dropped.
    #[test]
    fn the_cells_search_returns_the_gated_candidates() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
        ];
        let s = slot(&signals, 5);
        let gs = super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 15);
        let key = |v: &[(SyncCandidate, u8)]| {
            v.iter()
                .map(|(c, h)| {
                    (
                        c.freq_hz.to_bits(),
                        c.dt_sec.to_bits(),
                        c.score.to_bits(),
                        *h,
                    )
                })
                .collect::<Vec<_>>()
        };
        for bands in [1usize, 4] {
            let gated = super::super::coarse_sync::coarse_sync_gated(
                &mut SliceSource(&s),
                Ft8Rate::Hz8000,
                &gs,
                bands,
            );
            let (cells_found, _) =
                coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, bands).unwrap();
            assert_eq!(key(&gated), key(&cells_found), "{bands} bands");
        }
        assert!(
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz12000, &gs, 1).is_none(),
            "no cells at 12 kHz"
        );
    }

    /// The restricted search of a pass after a subtraction: with `near` at one signal's frequency every candidate is within `radius` cells of it and that
    /// signal is among them; with no `near` there is none; the cells are the same as the unrestricted search's. **Fails if** a carrier outside the radius is
    /// scored, one inside is dropped (a strong signal 200 Hz away is the test of both), or the gain argument is ignored.
    #[test]
    fn the_search_near_a_signal_returns_only_candidates_near_it() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 8000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 8000),
        ];
        let s = slot(&signals, 11);
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let gs = super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 30);
        let gain = spectrogram_gain(&mut SliceSource(&s));
        let (c, cells) = search_near(
            &mut SliceSource(&s),
            Ft8Rate::Hz8000,
            &gs,
            gain,
            &[1500.3],
            18,
        );
        assert!(!c.is_empty());
        for (cand, _) in &c {
            assert!(
                ((cand.freq_hz - 1500.3) / df).abs() <= 18.5,
                "candidate at {} Hz is outside 18 cells of 1500.3 Hz",
                cand.freq_hz
            );
        }
        assert!(
            c.iter()
                .any(|(cand, _)| (cand.freq_hz - 1500.3).abs() < 4.0),
            "the signal itself is among the candidates"
        );
        let (none, _) = search_near(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, gain, &[], 18);
        assert!(none.is_empty());
        let (all, whole) =
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, 1).unwrap();
        assert!(
            all.iter()
                .any(|(cand, _)| (cand.freq_hz - 700.0).abs() < 4.0)
        );
        assert_eq!(cells.size_bytes(), whole.size_bytes());
        assert_eq!(cells.cell(40, 480), whole.cell(40, 480));
    }

    /// The in-place search of a pass equals the allocating one: the same candidates and the same cells, **whatever the store held** (a store of the previous pass, here filled
    /// with a pattern that would show in a cell not written). **Fails if** a cell is left from the previous contents, `filled_rows` is not the slot's, or the candidates differ.
    #[test]
    fn search_near_into_a_used_store_equals_search_near() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 8000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 8000),
        ];
        let s = slot(&signals, 11);
        let gs = super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 30);
        let gain = spectrogram_gain(&mut SliceSource(&s));
        let (_, mut store) =
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, 1).unwrap();
        store.data.fill(0xA5);
        store.filled = 3;
        for near in [&[1500.3f32][..], &[700.0, 1500.3][..], &[][..]] {
            let (want, want_cells) =
                search_near(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, gain, near, 18);
            let got = search_near_into(
                &mut SliceSource(&s),
                Ft8Rate::Hz8000,
                &gs,
                gain,
                near,
                18,
                &mut store,
            );
            assert_eq!(got.len(), want.len(), "near {near:?}");
            for (a, b) in got.iter().zip(&want) {
                assert_eq!(
                    (
                        a.0.freq_hz.to_bits(),
                        a.0.dt_sec.to_bits(),
                        a.0.score.to_bits(),
                        a.1
                    ),
                    (
                        b.0.freq_hz.to_bits(),
                        b.0.dt_sec.to_bits(),
                        b.0.score.to_bits(),
                        b.1
                    ),
                    "near {near:?}"
                );
            }
            assert_eq!(store.filled_rows(), want_cells.filled_rows());
            assert_eq!(store.data, want_cells.data, "near {near:?}: the cells");
            store.data.fill(0x5A); // used again by the next pass
        }
    }

    /// A store written a row at a time is the demux's: the rows of the demux copied into an empty store with `row_mut` and `mark_filled` give the same cells, the same fill and the
    /// same search as the store `coarse_sync_gated_cells` makes; before the last rows are marked a row is not read. **Fails if** a row lands in the wrong place,
    /// `filled_rows` is off by one, or `cell` reads a row that is not filled.
    #[test]
    fn a_store_written_a_row_at_a_time_is_the_demuxs() {
        let s = slot(&[("CQ", "K1ABC", "FN42", 1500.3, 0.9, 8000)], 11);
        let gs = super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 20);
        let (list, whole) =
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, 1).unwrap();
        let mut built = HalfDbCells::new(
            Ft8Rate::Hz8000,
            whole.first_bin(),
            whole.n_freq(),
            whole.n_time(),
        );
        assert_eq!(
            (built.filled_rows(), built.cell(0, whole.first_bin())),
            (0, None)
        );
        for row in 0..whole.n_time() {
            for (c, bin) in built.row_mut(row).iter_mut().zip(whole.first_bin()..) {
                *c = whole.cell(row, bin).unwrap();
            }
            built.mark_filled(row + 1);
            assert_eq!(built.filled_rows(), row + 1);
            assert_eq!(
                built.cell(row + 1, whole.first_bin()),
                None,
                "row {} is not filled",
                row + 1
            );
        }
        for row in 0..whole.n_time() {
            for bin in whole.first_bin()..whole.first_bin() + whole.n_freq() {
                assert_eq!(built.cell(row, bin), whole.cell(row, bin));
            }
        }
        let part = search_cells(&built, &gs, built.carrier_bins());
        let key = |v: &[(SyncCandidate, u8)]| {
            v.iter()
                .map(|(c, h)| {
                    (
                        c.freq_hz.to_bits(),
                        c.dt_sec.to_bits(),
                        c.score.to_bits(),
                        *h,
                    )
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(
            key(&merge_cells_parts(&[part], &gs, Ft8Rate::Hz8000)),
            key(&list)
        );
    }

    /// The carriers split between any number of parts, scored apart and merged, give `coarse_sync_gated_cells`'s list bit for bit (the two-core split of the board).
    /// **Fails if** the merge or the cut of a part changes a candidate or the order, or a carrier is scored twice or not at all.
    #[test]
    fn a_search_split_between_parts_is_the_whole_search() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 5000),
            // on the last carrier of the range (3 000 Hz): a part that drops its last carrier loses it
            ("W9XYZ", "K1ABC", "RR73", 3000.0, 0.5, 8000),
        ];
        let s = slot(&signals, 11);
        let gs = super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 20);
        let (list, store) =
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, 1).unwrap();
        let key = |v: &[(SyncCandidate, u8)]| {
            v.iter()
                .map(|(c, h)| {
                    (
                        c.freq_hz.to_bits(),
                        c.dt_sec.to_bits(),
                        c.score.to_bits(),
                        *h,
                    )
                })
                .collect::<Vec<_>>()
        };
        assert!(
            list.iter().any(|(c, _)| (c.freq_hz - 3000.0).abs() < 1.0),
            "the signal on the last carrier is a candidate"
        );
        let r = store.carrier_bins();
        for n in [1usize, 2, 3, 7] {
            let n_car = r.end - r.start;
            let parts: Vec<CellsPart> = (0..n)
                .map(|i| {
                    let (a, b) = (r.start + i * n_car / n, r.start + (i + 1) * n_car / n);
                    search_cells(&store, &gs, a..b)
                })
                .collect();
            assert_eq!(
                key(&merge_cells_parts(&parts, &gs, Ft8Rate::Hz8000)),
                key(&list),
                "{n} parts"
            );
        }
    }

    /// A search over a store still being made reads only the rows written: two stores filled to the same row, the rest zero in one and **pseudo-random cells** in the other, give the
    /// same list, and a signal whose rows are all written is found while one that has not arrived is not. **Fails if** a row at or past the fill is read (the scoring
    /// would use the slot's length, not the fill), or the fill is off by a row.
    #[test]
    fn a_partly_written_store_is_searched_over_the_rows_it_has() {
        // A at DT 0.2 s (its 79 symbols end at row 8 + 156 = 164), B at DT 2.3 s (rows 35 to 191: not all of them written at row 174)
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 8000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 2.3, 8000),
        ];
        let s = slot(&signals, 11);
        let gs = super::super::coarse_sync::GatedSearch::new(200.0, 3000.0, 1.3, 20);
        let (_, whole) =
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &gs, 1).unwrap();
        let fill = 174; // the board's emit: row pair 87
        // what a store holds past its fill that a search must not read: nothing, or cells that would change every score (random, up to the 193 of a real cell)
        let make = |garbage: bool| {
            let mut st = HalfDbCells::new(
                Ft8Rate::Hz8000,
                whole.first_bin(),
                whole.n_freq(),
                whole.n_time(),
            );
            for row in 0..whole.n_time() {
                for (c, bin) in st.row_mut(row).iter_mut().zip(whole.first_bin()..) {
                    *c = if row < fill {
                        whole.cell(row, bin).unwrap()
                    } else if garbage {
                        ((row * 31 + bin * 17) % 194) as u8
                    } else {
                        0
                    };
                }
            }
            st.mark_filled(fill);
            st
        };
        let (zero, junk) = (make(false), make(true));
        let list = |st: &HalfDbCells| {
            merge_cells_parts(
                &[search_cells(st, &gs, st.carrier_bins())],
                &gs,
                Ft8Rate::Hz8000,
            )
        };
        let key = |v: &[(SyncCandidate, u8)]| {
            v.iter()
                .map(|(c, h)| {
                    (
                        c.freq_hz.to_bits(),
                        c.dt_sec.to_bits(),
                        c.score.to_bits(),
                        *h,
                    )
                })
                .collect::<Vec<_>>()
        };
        let (lz, lj) = (list(&zero), list(&junk));
        assert_eq!(key(&lz), key(&lj), "a row past the fill was read");
        let near =
            |l: &[(SyncCandidate, u8)], f: f32| l.iter().any(|(c, _)| (c.freq_hz - f).abs() < 4.0);
        assert!(
            near(&lz, 700.0),
            "the signal whose rows are all written is found"
        );
        // the early signal's candidate is ready, the late signal's (its true place, DT 2.3 s) is found from the rows there are and is not ready
        let place = |l: &[(SyncCandidate, u8)], f: f32, dt: f32| {
            l.iter()
                .find(|(c, _)| (c.freq_hz - f).abs() < 4.0 && (c.dt_sec - dt).abs() < 0.15)
                .map(|(c, _)| candidate_ready(&zero, c))
        };
        assert_eq!(
            place(&lz, 700.0, 0.2),
            Some(true),
            "the early signal is ready"
        );
        assert_eq!(
            place(&lz, 1500.3, 2.3),
            Some(false),
            "the late signal is found and is not ready at row {fill}"
        );
    }

    /// `candidate_ready` is exact: a candidate whose last row read is `fill - 1` is ready, one row later it is not. **Fails if** the rows a decode reads (its 79 symbols and the retry's
    /// neighbour row) are counted wrong by one either way.
    #[test]
    fn a_candidate_is_ready_when_every_row_it_reads_is_written() {
        let mut st = HalfDbCells::new(Ft8Rate::Hz8000, 64, 960, 184);
        let at = |j0: i32| SyncCandidate {
            freq_hz: 700.0,
            dt_sec: j0 as f32 * 0.08 - 0.5,
            score: 10.0,
        };
        st.mark_filled(174);
        // rows read: j0 + 2 x 78 + 1 = j0 + 157 must be below 174: j0 <= 16
        assert!(candidate_ready(&st, &at(16)));
        assert!(!candidate_ready(&st, &at(17)));
        st.mark_filled(0);
        assert!(!candidate_ready(&st, &at(-22)) || false);
    }

    /// The union of the bands' best positions, merged, is the best positions of the whole range, **for cuts that bind**: the same
    /// positions, in the same order, whether the carriers are scored in 1, 2, 4 or 7 bands, for `keep_raw` of 3, 6, 24 and 96
    /// (far fewer than the positions above `MIN_SCORE` of the slot). The list `search` returns only sees the cut when it starves
    /// (the best positions of a signal are scattered over several neighbourhoods: at `RAW_FACTOR` 12 it never does on a real slot),
    /// so the property is tested here on the stage below it. Fails if the histogram cut drops the ties at the threshold, a band
    /// or the merge does not truncate to the best `keep_raw` by the total order, or the order depends on the arrival of the positions.
    #[test]
    fn the_merged_bands_are_the_best_positions_of_the_whole_range() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 1500.0f32, 0.2f32, 20000i16),
            ("K1ABC", "W9XYZ", "-12", 700.3, 0.9, 3000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 3000),
        ];
        let s = slot(&signals, 11);
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let n_freq = spectrogram_n_freq(3000.0, Ft8Rate::Hz8000);
        let ia = (200.0f32 / df).round() as usize;
        let ib = ((3000.0f32 / df).round() as usize).min(n_freq - MAX_TONE_OFF);
        let mut src = SliceSource(&s);
        let gain = spectrogram_gain(&mut src);
        let bands_of = |bands: usize, keep_raw: usize, src: &mut SliceSource<i16>| {
            let n_car = ib - ia + 1;
            let mut all = Vec::new();
            for b in 0..bands {
                let a = ia + b * n_car / bands;
                let z = ia + (b + 1) * n_car / bands - 1;
                let cells =
                    compute_half_db_bytes_src(src, Ft8Rate::Hz8000, a, z + MAX_TONE_OFF, gain);
                all.extend(band_best(&view_of(&cells), a, a, z, keep_raw, None));
            }
            merge(all, keep_raw)
        };
        let key = |v: &[Pos]| {
            v.iter()
                .map(|p| (p.score, p.k, p.j, p.hs))
                .collect::<Vec<_>>()
        };
        for keep_raw in [3usize, 6, 24, 96] {
            let one = bands_of(1, keep_raw, &mut src);
            assert_eq!(
                one.len(),
                keep_raw,
                "the slot has more than {keep_raw} positions above the minimum"
            );
            for bands in [2usize, 4, 7] {
                assert_eq!(
                    key(&one),
                    key(&bands_of(bands, keep_raw, &mut src)),
                    "keep_raw {keep_raw}, {bands} bands"
                );
            }
        }
    }

    /// The histogram cut, the per-band selection and the merge give the list a plain sort of every position and the suppression
    /// give (`Order::Score`). Fails if the cut keeps too few positions to fill the list after the suppression, or the
    /// total order is not applied.
    #[test]
    fn the_selection_is_the_plain_sort_and_suppression() {
        let s = slot(&[("CQ", "K1ABC", "FN42", 1200.0, 0.0, 4000)], 9);
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let n_freq = spectrogram_n_freq(3000.0, Ft8Rate::Hz8000);
        let ia = (200.0f32 / df).round() as usize;
        let ib = ((3000.0f32 / df).round() as usize).min(n_freq - MAX_TONE_OFF);
        let mut src = SliceSource(&s);
        let gain = spectrogram_gain(&mut src);
        let cells =
            compute_half_db_bytes_src(&mut src, Ft8Rate::Hz8000, ia, ib + MAX_TONE_OFF, gain);
        let mut every = Vec::new();
        for k in ia..=ib {
            for jj in 0..NJ {
                let j = J0_MIN + jj as i32;
                let sc = score(&view_of(&cells), k - ia, j);
                if sc >= MIN_SCORE {
                    every.push(Pos {
                        score: sc.min(127),
                        k,
                        j,
                        hs: 0,
                    });
                }
            }
        }
        every.sort_by(better);
        // the reference written out again, not `suppress`: walk the sorted positions, keep one with no kept neighbour
        let mut want: Vec<Pos> = Vec::new();
        for p in &every {
            let near = want
                .iter()
                .any(|w| (w.k as i64 - p.k as i64).abs() <= 2 && (w.j - p.j).abs() <= 2);
            if !near {
                want.push(*p);
            }
            if want.len() == 30 {
                break;
            }
        }
        let got = run(&s, 30, 1, Order::Score);
        assert_eq!(got.len(), want.len());
        for (g, w) in got.iter().zip(&want) {
            assert_eq!(
                (g.0.freq_hz.to_bits(), g.0.dt_sec.to_bits()),
                (
                    (w.k as f32 * df).to_bits(),
                    (w.j as f32 * 0.08 - 0.5).to_bits()
                )
            );
        }
    }

    /// Every candidate is inside `ft8dec`'s rows and the band asked for: DT -2.26..+2.46 s, the tone-0 frequency within
    /// `freq_min .. freq_max`. Fails if the row range is widened (the signal at DT +2.6 s of `20m_busy` comes back).
    #[test]
    fn the_candidates_stay_inside_the_rows_and_the_band() {
        let s = slot(
            &[
                ("CQ", "K1ABC", "FN42", 1000.0, 2.6, 6000),
                ("K1ABC", "W9XYZ", "-12", 1800.0, -2.4, 6000),
                ("CQ", "JA1ABC", "PM95", 400.0, 0.2, 6000),
            ],
            7,
        );
        let got = search(
            &mut SliceSource(&s),
            Ft8Rate::Hz8000,
            500.0,
            2000.0,
            60,
            3,
            Order::Score,
        );
        assert!(!got.is_empty());
        for (c, _) in &got {
            assert!((-2.27..=2.47).contains(&c.dt_sec), "dt {}", c.dt_sec);
            assert!((499.0..=2001.0).contains(&c.freq_hz), "freq {}", c.freq_hz);
        }
        assert!(
            !got.iter()
                .any(|(c, _)| (c.freq_hz - 1000.0).abs() < 3.2 && c.dt_sec > 2.47),
            "the signal at DT +2.6 s is not a candidate"
        );
    }

    /// `carrier_scores` gives `score`'s value at **every position of every carrier**, on a slot with signals and with rows missing: a store read to 184 rows (the whole slot) and to the rows a store
    /// partly written has (100, 40, 7, 5, 4, 3, 2, 1, 0 of them: the edges of the interior fast path at 2 and `rows - 2`, and the positions whose symbols are all off the slot). Cells of a real slot,
    /// and a second store of arbitrary bytes (0..255, so the sums and the truncating division see negatives and large terms). **Fails if** the regrouped sum differs by one at any position: a
    /// neighbour term missing at a row edge, a block's first or last symbol counted wrongly, a symbol off the slot added, a division by a different count.
    #[test]
    fn carrier_scores_are_the_scores_at_every_position() {
        let s = slot(
            &[
                ("CQ", "K1ABC", "FN42", 1200.0, 0.0, 4000),
                ("K1ABC", "W9XYZ", "-10", 1800.0, 0.5, 3000),
            ],
            5,
        );
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let n_freq = spectrogram_n_freq(3000.0, Ft8Rate::Hz8000);
        let ia = (200.0f32 / df).round() as usize;
        let ib = ((3000.0f32 / df).round() as usize).min(n_freq - MAX_TONE_OFF);
        let mut src = SliceSource(&s);
        let gain = spectrogram_gain(&mut src);
        let real =
            compute_half_db_bytes_src(&mut src, Ft8Rate::Hz8000, ia, ib + MAX_TONE_OFF, gain);
        let mut noise =
            compute_half_db_bytes_src(&mut src, Ft8Rate::Hz8000, ia, ib + MAX_TONE_OFF, gain);
        let mut x = 12345u32;
        for b in noise.data.iter_mut() {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            *b = (x >> 24) as u8;
        }
        for (name, cells) in [("real", &real), ("arbitrary bytes", &noise)] {
            let full = view_of(cells);
            let mut checked = 0usize;
            for rows in [full.n_time, 100, 40, 7, 5, 4, 3, 2, 1, 0] {
                let view = CellsRef {
                    n_time: rows.min(full.n_time),
                    ..full
                };
                let mut w = alloc::vec![0i32; view.n_time * SYM_STRIDE];
                let mut out = [0i32; NJ];
                for rel_k in (0..=(ib - ia)).step_by(if name == "real" { 1 } else { 7 }) {
                    carrier_scores(&view, rel_k, &mut w, &mut out);
                    for (jj, &got) in out.iter().enumerate() {
                        let want = score(&view, rel_k, J0_MIN + jj as i32);
                        assert_eq!(got, want, "{name}: rows {rows}, carrier {rel_k}, lag {jj}");
                        checked += 1;
                    }
                }
            }
            assert!(checked > 50_000, "{name}: {checked} positions checked");
        }
    }
}
