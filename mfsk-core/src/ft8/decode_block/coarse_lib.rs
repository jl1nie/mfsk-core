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
    Spectrogram, compute_half_db_bins_src, spectrogram_gain, spectrogram_n_freq,
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
#[inline]
fn score(cells: &Spectrogram, rel_k: usize, j0: i32) -> i32 {
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
fn hard_sync(cells: &Spectrogram, rel_k: usize, j0: i32) -> u8 {
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

/// Score every position of the carriers `k_first..=k_last` (bins of the full spectrogram; `cells` starts at bin `lo`) and
/// return the best `keep_raw` of them by [`better`] (ties at the cut are all kept: the cut is by score, then trimmed by the
/// total order), each with its hard-sync count.
fn band_best(
    cells: &Spectrogram,
    lo: usize,
    k_first: usize,
    k_last: usize,
    keep_raw: usize,
) -> Vec<Pos> {
    let n_car = k_last - k_first + 1;
    // scores above 127 are clamped to 127 (a tie of very strong signals is broken by carrier and row, not by the peak: none is that
    // strong in the corpora, where the contrasts against the adjacent symbols keep a clean signal near 30-60)
    let mut scores = alloc::vec![i8::MIN; n_car * NJ];
    // the scores 0..=127 only matter above MIN_SCORE: a histogram finds the cut without sorting every position above the minimum (hundreds a slot)
    let mut hist = [0u32; 128];
    for ci in 0..n_car {
        let rel_k = k_first + ci - lo;
        for jj in 0..NJ {
            let s = score(cells, rel_k, J0_MIN + jj as i32);
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
    let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
    let tstep = rate.nstep() as f32 / rate.hz() as f32;
    let spec_n_freq = spectrogram_n_freq(freq_max, rate);
    let ia = (freq_min / df).round() as usize;
    let ib = ((freq_max / df).round() as usize).min(spec_n_freq.saturating_sub(MAX_TONE_OFF));
    if ib < ia || keep == 0 {
        return Vec::new();
    }
    let pool = match order {
        Order::Score => keep,
        Order::PoolThenHardSync => POOL.max(keep),
    };
    let keep_raw = RAW_FACTOR * pool;
    let gain = spectrogram_gain(src);
    let n_car = ib - ia + 1;
    let bands = bands.clamp(1, n_car);
    let mut all: Vec<Pos> = Vec::new();
    for b in 0..bands {
        let a = ia + b * n_car / bands;
        let z = ia + (b + 1) * n_car / bands - 1;
        let cells = compute_half_db_bins_src(src, rate, a, z + MAX_TONE_OFF, gain);
        all.extend(band_best(&cells, a, a, z, keep_raw));
    }
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
                    compute_half_db_bins_src(src, Ft8Rate::Hz8000, a, z + MAX_TONE_OFF, gain);
                all.extend(band_best(&cells, a, a, z, keep_raw));
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
            compute_half_db_bins_src(&mut src, Ft8Rate::Hz8000, ia, ib + MAX_TONE_OFF, gain);
        let mut every = Vec::new();
        for k in ia..=ib {
            for jj in 0..NJ {
                let j = J0_MIN + jj as i32;
                let sc = score(&cells, k - ia, j);
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
}
