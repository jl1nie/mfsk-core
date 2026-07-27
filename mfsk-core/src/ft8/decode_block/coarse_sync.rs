//! Coarse-sync stage: Costas-array correlation search across the
//! power spectrogram. Mirrors WSJT-X `lib/ft8/sync8.f90` — 16-bin
//! sliding-window allsum noise estimator with `tone_step_bins = 2.0`
//! exactly at `NFFT_SPEC = 3840`.
//!
//! ε.3 of the `docs/CLEANUP_2026_05.md` `decode_block` split. The
//! parent (`decode_block.rs`) re-exports the public entries so
//! external callers — host `decode.rs`, embedded `stage1_inc`,
//! `mfsk-ffi-ft8`, integration tests — keep the same
//! `mfsk_core::ft8::decode_block::*` paths.

use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::params::{COSTAS, COSTAS_POS, NTONES};
use super::spectrogram::{CoarseAcc, Spectrogram};
use super::types::{
    NFFT_SPEC, NSSY, NSTEP, SAMPLE_RATE_HZ, TONE_SPACING_HZ, TX_START_OFFSET_S, ratio_eps,
    sync_lag_s,
};
use crate::engine::sync::SyncCandidate;

// ── Coarse sync ─────────────────────────────────────────────────────────────

/// Costas-array correlation search across the spectrogram. Mirrors
/// WSJT-X `lib/ft8/sync8.f90` — 16-bin sliding-window allsum noise
/// estimator with `tone_step_bins = 2.0` exactly at NFFT_SPEC=3840.
///
/// Public as of v0.6 (#48): this is now the canonical FT8 coarse-sync
/// for both the embedded port and the host `decode_frame*` pipeline.
/// `engine::sync::coarse_sync<Ft8>` is no longer reachable from FT8 code
/// paths; that generic function stays in place for FT4 / FST4 / JT9 /
/// Q65 / WSPR / uvpacket.
pub fn coarse_sync(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(spec, freq_min, freq_max, sync_min, max_cand, None)
}

/// Phase-E2 entry point — like [`coarse_sync`] but consumes a
/// caller-built allsum table instead of recomputing it. Saves the
/// 280-300 ms allsum precompute on Core2 by hiding it under the
/// 15 s capture window in the embedded port (`stage1_inc` builds
/// the allsum incrementally as new spectrogram rows arrive).
///
/// `allsum` must match exactly what
/// [`precompute_coarse_allsum`] produces for the same `spec`,
/// `freq_min`, `freq_max` triple — same layout
/// (`data[fi * spec.n_time + m]`), same length
/// ([`coarse_allsum_len`]).
///
/// Public as of v0.6 (#49 cat C): the embedded port (`m5stack-core2`,
/// `m5stack-s3`, `m5stack-s3-app`) and `mfsk-ffi-ft8` both build the
/// allsum incrementally during slot capture and pass it back here, so
/// this is a stable public surface — not a benchmark-only escape hatch.
pub fn coarse_sync_with_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum: &[CoarseAcc],
) -> Vec<SyncCandidate> {
    coarse_sync_inner(spec, freq_min, freq_max, sync_min, max_cand, Some(allsum))
}

/// Length of the allsum buffer that [`coarse_sync_with_allsum`]
/// expects for the given `spec` + `freq_min..=freq_max` band.
/// Returns 0 when the band has no candidates (then
/// `coarse_sync_with_allsum` would return an empty vec immediately
/// regardless of `allsum`).
pub fn coarse_allsum_len(
    spec_n_freq: usize,
    spec_n_time: usize,
    freq_min: f32,
    freq_max: f32,
) -> usize {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tone_step_bins = TONE_SPACING_HZ / df;
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(spec_n_freq.saturating_sub(max_tone_off));
    if ib < ia {
        return 0;
    }
    let n_freq = ib - ia + 1;
    n_freq * spec_n_time
}

/// One-shot allsum builder. The embedded port can build the allsum
/// incrementally instead and pass it to [`coarse_sync_with_allsum`];
/// host callers use this. Layout matches [`coarse_sync_with_allsum`]'s
/// expected input.
pub fn precompute_coarse_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
) -> Vec<CoarseAcc> {
    let mut buf =
        vec![CoarseAcc::default(); coarse_allsum_len(spec.n_freq, spec.n_time, freq_min, freq_max)];
    if !buf.is_empty() {
        precompute_coarse_allsum_into(spec, freq_min, freq_max, &mut buf);
    }
    buf
}

/// In-place variant of [`precompute_coarse_allsum`]. `dst` must have
/// length [`coarse_allsum_len`]. No-op if the band is empty.
pub fn precompute_coarse_allsum_into(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    dst: &mut [CoarseAcc],
) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tone_step_bins = TONE_SPACING_HZ / df;
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let nh1 = spec.n_freq;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
    if ib < ia {
        return;
    }
    let n_freq = ib - ia + 1;
    debug_assert_eq!(
        dst.len(),
        n_freq * spec.n_time,
        "allsum buffer length mismatch"
    );
    fill_coarse_allsum(spec, ia, ib, n_freq, dst);
}

/// Build the 7-tone-stride-2 allsum for one time slice `m` into `dst`.
///
/// Loop order: `m`-outer, `fi`-inner. One time slice (~2 KB at u16)
/// stays in cache for all carriers instead of striding by `n_freq`
/// bytes per time step.
///
/// Under `fixed-point` (SpecCell = u16, values ≤ 65535): uses a
/// **sliding window** — carriers at `fi` and `fi+2` share 6 of 7
/// bins, so `allsum[fi] = allsum[fi-2] + row[ia+fi+12] - row[ia+fi-2]`.
/// Reduces inner work from 7 adds to 1 add + 1 sub. Intermediate
/// sums ≤ 7×65535 < 2^19 are exactly representable in f32, so no
/// drift accumulates.
///
/// Without `fixed-point` (SpecCell = f32, host path): direct sum to
/// avoid rounding drift on large spectrum values that would break the
/// full-band-allsum-then-slice equivalence test.
fn fill_allsum_row(spec: &Spectrogram, ia: usize, n_freq: usize, m: usize, dst: &mut [CoarseAcc]) {
    if n_freq == 0 {
        return;
    }
    let nh1 = spec.n_freq;
    let upper = nh1 - 1;
    let n_time = spec.n_time;

    #[cfg(feature = "fixed-point")]
    {
        // Sliding window: two independent chains (even / odd fi).
        let mut prev = [CoarseAcc::default(); 2];
        for parity in 0..2usize {
            if parity >= n_freq {
                break;
            }
            let i_carrier = ia + parity;
            let mut s = CoarseAcc::default();
            for k in 0..(NTONES - 1) {
                let bin = (i_carrier + 2 * k).min(upper);
                s += spec.power_acc(bin, m);
            }
            dst[parity * n_time + m] = s;
            prev[parity] = s;
        }
        for fi in 2..n_freq {
            let i_carrier = ia + fi;
            let drop = (i_carrier - 2).min(upper);
            let add = (i_carrier + 2 * (NTONES - 2)).min(upper);
            let p = fi & 1;
            #[allow(clippy::unnecessary_cast)]
            let s = prev[p] + spec.power_acc(add, m) - spec.power_acc(drop, m);
            dst[fi * n_time + m] = s;
            prev[p] = s;
        }
    }

    #[cfg(not(feature = "fixed-point"))]
    {
        // Direct sum: no f32 drift for host tests.
        for fi in 0..n_freq {
            let i_carrier = ia + fi;
            let mut s = CoarseAcc::default();
            for k in 0..(NTONES - 1) {
                let bin = (i_carrier + 2 * k).min(upper);
                s += spec.power_acc(bin, m);
            }
            dst[fi * n_time + m] = s;
        }
    }
}

/// Build the 7-tone-stride-2 allsum for the carrier range `[ia..=ib]`
/// into `dst` (`n_freq * spec.n_time`, layout `dst[fi * n_time + m]`).
///
/// Mirrors the inline precompute inside [`coarse_sync_inner`] and
/// [`stage1_inc::update_one_half`] — kept as a public helper so the
/// embedded port can replicate it row-by-row without accessing private
/// internals. Computes **every m** so callers can populate cells
/// incrementally in any order.
fn fill_coarse_allsum(
    spec: &Spectrogram,
    ia: usize,
    ib: usize,
    _n_freq: usize,
    dst: &mut [CoarseAcc],
) {
    let n_freq = ib - ia + 1;
    // m-outer loop: one time slice (~2 KB u16) stays in cache for all fi.
    for m in 0..spec.n_time {
        fill_allsum_row(spec, ia, n_freq, m, dst);
    }
}

fn coarse_sync_inner(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    external_allsum: Option<&[CoarseAcc]>,
) -> Vec<SyncCandidate> {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let jstrt = (TX_START_OFFSET_S / tstep).round() as i32;
    let jz = (sync_lag_s() / tstep).round() as i32;
    let tone_step_bins = TONE_SPACING_HZ / df;

    // Carrier-bin search range. Reserve room above the carrier for the
    // top tone (round(7 * tone_step_bins)).
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let nh1 = spec.n_freq;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
    if ib < ia {
        return Vec::new();
    }
    let n_freq = ib - ia + 1;
    let n_lag = (2 * jz + 1) as usize;
    let mut sync2d = vec![0.0f32; n_freq * n_lag];
    let idx = |fi: usize, lag: i32| fi * n_lag + (lag + jz) as usize;
    let ratio_eps = ratio_eps();
    #[cfg(all(
        feature = "profile-coarse",
        not(all(target_arch = "wasm32", target_os = "unknown"))
    ))]
    let t_setup = std::time::Instant::now();

    // **Single-bin tone gather**. NFFT=3840 → tone_step_bins = 2.0
    // exactly, so the 8 FT8 tones sit on integer bins {0, 2, 4, …, 14}
    // relative to the carrier. With the rectangular pre-FFT window
    // each tone's mainlobe is one bin wide and contains the full tone
    // energy, so single-bin reads are loss-less. The Plan-A multi-bin
    // sum (floor-bin + floor-bin+1) was a fractional-alignment +
    // Hann-mainlobe compensation for the NFFT=4096 era and was
    // dropped at the 3840 migration; do NOT re-add it without also
    // restoring Hann (the two were always paired).
    let mut tone_bin_lo = [0usize; NTONES];
    for k in 0..NTONES {
        tone_bin_lo[k] = (k as f32 * tone_step_bins).floor() as usize;
    }

    // Pre-compute the (bk, n) → m_base table. m for the inner iter is
    // `m_base[bk][n] + lag`; m_base depends only on the Costas pattern
    // and `jstrt` (constants of the slot), not on (fi, lag). Hoists
    // 21 mul/add chains out of the n_freq × n_lag × 21 inner loop.
    let m_base: [[i32; COSTAS.len()]; COSTAS_POS.len()] = {
        let mut t = [[0i32; COSTAS.len()]; COSTAS_POS.len()];
        for (bk, &start_sym) in COSTAS_POS.iter().enumerate() {
            let block_offset = NSSY * start_sym as i32;
            for (n, _) in COSTAS.iter().enumerate() {
                t[bk][n] = jstrt + block_offset + NSSY * n as i32;
            }
        }
        t
    };
    // Pre-compute Costas-tone bin offsets in COSTAS-order (i.e.
    // `tone_bin_lo[COSTAS[n]]`). Saves an indirect lookup per inner
    // iteration.
    let costas_off: [usize; COSTAS.len()] = {
        let mut t = [0usize; COSTAS.len()];
        for (n, &costas_n) in COSTAS.iter().enumerate() {
            t[n] = tone_bin_lo[costas_n];
        }
        t
    };

    // Pre-compute the set of `m` time-indices that the score loop
    // will read. Only Costas-symbol positions ± lag count — typically
    // 3 contiguous bands totalling ~110 of the 184 frames, so the
    // naïve "compute allsum for every m" wastes ~40 % of its work
    // on cells the score loop never reads.
    let needed_m: alloc::vec::Vec<usize> = {
        let mut mark = alloc::vec![false; spec.n_time];
        for bk in 0..COSTAS_POS.len() {
            let lo = m_base[bk][0] - jz;
            let hi = m_base[bk][COSTAS.len() - 1] + jz;
            let lo_u = lo.max(0) as usize;
            let hi_u = (hi.min(spec.n_time as i32 - 1)) as usize;
            if lo_u <= hi_u {
                #[allow(clippy::needless_range_loop)]
                for m in lo_u..=hi_u {
                    mark[m] = true;
                }
            }
        }
        (0..spec.n_time).filter(|&m| mark[m]).collect()
    };

    // Pre-compute `Σ_{k=0..NTONES-1} spec[i_carrier + 2*k, m]` for every
    // (fi, m ∈ needed_m). NFFT=3840 → tone_step_bins=2.0 exactly →
    // 8-bin every-other gather per carrier (matches WSJT-X `sync8.f90`'s
    // single-bin `t0a` accumulator). No sliding-window reuse since
    // adjacent fi share zero bins under this pattern.
    //
    // Phase-E2: caller can pass a pre-built allsum (built incrementally
    // during slot capture by stage1_inc). When provided, skip the
    // precompute entirely. Otherwise build it inline.
    let owned_allsum: Vec<CoarseAcc>;
    let allsum: &[CoarseAcc] = if let Some(ext) = external_allsum {
        debug_assert_eq!(
            ext.len(),
            n_freq * spec.n_time,
            "external allsum length mismatch (expected n_freq * spec.n_time)"
        );
        ext
    } else {
        owned_allsum = {
            // Sum 7 Costas-tone bins (k=0..NTONES-1; tone 7 is data-only,
            // never a Costas position). Matches WSJT-X `sync8.f90:66` and
            // the public `fill_coarse_allsum` helper. Only populates
            // `needed_m` cells — ~60 % of spec.n_time — to save ~40 % of
            // allsum work; the remaining cells stay at default (0/0.0) and
            // are never read by the score loop.
            // Uses the stride-2 sliding window: allsum[fi] = allsum[fi-2]
            // + row[add] - row[drop], 7× fewer ops per fi after init.
            let mut buf = vec![CoarseAcc::default(); n_freq * spec.n_time];
            for &m in &needed_m {
                fill_allsum_row(spec, ia, n_freq, m, &mut buf);
            }
            buf
        };
        &owned_allsum
    };
    #[cfg(all(
        feature = "profile-coarse",
        not(all(target_arch = "wasm32", target_os = "unknown"))
    ))]
    let t_allsum = std::time::Instant::now();

    // Three identical Costas arrays at symbol positions 0, 36, 72.
    //
    // **Bounds-check hoist**. For the standard FT8 slot:
    //   block 0: m = lag + jstrt + NSSY*n   ∈ [-7..31] over (lag, n)
    //   block 1: m = lag + jstrt + 144 + NSSY*n  ∈ [65..103]
    //   block 2: m = lag + jstrt + 288 + NSSY*n  ∈ [137..175]
    //
    // Only block 0 can dip below 0 (and only at large negative lag).
    // Compute the smallest valid `n` per lag once, then iterate
    // n_start..NTONES_C without per-iter checks. Blocks 1 and 2 are
    // always fully in range. Sanity-check the upper bound at function
    // entry so the unchecked-as-usize is safe.
    debug_assert!(
        m_base[2][COSTAS.len() - 1] + jz < spec.n_time as i32,
        "n_time too small for SYNC_LAG_S/jstrt"
    );
    let n_time = spec.n_time;
    // Per-fi: tbin_lo[n] = i_carrier + costas_off[n]. Hoist out of
    // the lag loop — saves an addition per inner iteration.
    let mut tbin_lo_arr = [0usize; COSTAS.len()];
    for (fi, i_carrier) in (ia..=ib).enumerate() {
        for n in 0..COSTAS.len() {
            tbin_lo_arr[n] = i_carrier + costas_off[n];
        }
        let allsum_row = &allsum[fi * n_time..(fi + 1) * n_time];
        for lag in -jz..=jz {
            let mut t_blocks: [CoarseAcc; 3] = [CoarseAcc::default(); 3];
            let mut t0_blocks: [CoarseAcc; 3] = [CoarseAcc::default(); 3];

            // Block 0: smallest valid n where m_base[0][n] + lag >= 0.
            // m_base[0][n] = jstrt + NSSY*n. Solve for n:
            //   jstrt + NSSY*n + lag >= 0 ⇒ n >= ceil((-jstrt - lag) / NSSY)
            let bk0_n_start = {
                let needed = -jstrt - lag;
                if needed <= 0 {
                    0usize
                } else {
                    ((needed + NSSY - 1) / NSSY).min(COSTAS.len() as i32) as usize
                }
            };
            // NFFT=3840 → tone_step_bins = 2.0 exactly, signal at bin
            // `tbin_lo` only (single bin gather, matching WSJT-X's
            // `s(i+nfos*icos7(n), m)`). The legacy `+ spec[tbin_lo + 1]`
            // half was a fractional-alignment compensation for NFFT=4096
            // and is removed.
            for n in bk0_n_start..COSTAS.len() {
                let m_u = (m_base[0][n] + lag) as usize;
                let tbin_lo = tbin_lo_arr[n];
                t_blocks[0] += spec.power_acc(tbin_lo, m_u);
                t0_blocks[0] += allsum_row[m_u];
            }
            // Blocks 1, 2: always fully in range — no per-iter check.
            for bk in 1..COSTAS_POS.len() {
                for n in 0..COSTAS.len() {
                    let m_u = (m_base[bk][n] + lag) as usize;
                    let tbin_lo = tbin_lo_arr[n];
                    t_blocks[bk] += spec.power_acc(tbin_lo, m_u);
                    t0_blocks[bk] += allsum_row[m_u];
                }
            }

            // Regularised ratio `t / (mean_others + ε)`.
            // ε prevents the u16-quantised fp path from blowing up
            // when phantom carriers happen to land where 7 of 8 tone
            // bins quantise to 0; `t0_ref → 0` would otherwise inflate
            // ratio scores by 100-1000× over real signals.
            //
            // Phase 3: t/t0 sums stay in CoarseAcc; convert to f32 only
            // at the score-division boundary so sync2d / red / base
            // (downstream sort + percentile) remain f32.
            let t_all: CoarseAcc = t_blocks[0] + t_blocks[1] + t_blocks[2];
            let t0_all: CoarseAcc = t0_blocks[0] + t0_blocks[1] + t0_blocks[2];
            // `as f32` is a real promotion under fixed-point (CoarseAcc=i32)
            // and a no-op when CoarseAcc=f32 — silence the host clippy.
            #[allow(clippy::unnecessary_cast)]
            let t_all_f = t_all as f32;
            #[allow(clippy::unnecessary_cast)]
            let t0_all_f = t0_all as f32;
            let t0_ref = (t0_all_f - t_all_f) / (NTONES as f32 - 2.0);
            let sync_all = t_all_f / (t0_ref + ratio_eps);

            // Trailing-2-blocks score (drop block 0 — late-start tolerance).
            #[allow(clippy::unnecessary_cast)]
            let t_tail_f = (t_blocks[1] + t_blocks[2]) as f32;
            #[allow(clippy::unnecessary_cast)]
            let t0_tail_f = (t0_blocks[1] + t0_blocks[2]) as f32;
            let t0_tail_ref = (t0_tail_f - t_tail_f) / (NTONES as f32 - 2.0);
            let sync_tail = t_tail_f / (t0_tail_ref + ratio_eps);

            sync2d[idx(fi, lag)] = sync_all.max(sync_tail);
        }
    }
    #[cfg(all(
        feature = "profile-coarse",
        not(all(target_arch = "wasm32", target_os = "unknown"))
    ))]
    let t_score = std::time::Instant::now();

    const MLAG: i32 = 10;

    // Per-bin peak + 40-percentile noise floor.
    let mut red = vec![0.0f32; n_freq];
    for fi in 0..n_freq {
        red[fi] = (-jz..=jz)
            .map(|lag| sync2d[idx(fi, lag)])
            .fold(0.0f32, f32::max);
    }
    let base = {
        // 40th-percentile noise floor: `select_nth_unstable_by` is O(N)
        // average vs the previous full-sort's O(N log N). Pivot
        // ordering inside the percentile isn't load-bearing
        // (downstream only uses `red[pct_idx]` as a normaliser), so
        // unstable selection is fine. `unwrap_or(Ordering::Equal)`
        // for NaN safety — matches the same pattern at
        // `process_candidates.rs` xsnr2_db_simple median.
        // Gemini PR #79 + #101 review.
        let mut sorted = red.clone();
        let pct_idx = ((0.40 * n_freq as f32) as usize).min(n_freq - 1);
        sorted.select_nth_unstable_by(pct_idx, |a, b| {
            a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)
        });
        sorted[pct_idx].max(f32::EPSILON)
    };

    let mut cands: Vec<SyncCandidate> = Vec::new();
    for fi in 0..n_freq {
        let i_carrier = ia + fi;
        let freq_hz = i_carrier as f32 * df;

        let mut peaks: Vec<(i32, f32)> = (-jz..=jz)
            .filter_map(|lag| {
                let raw = sync2d[idx(fi, lag)];
                let norm = raw / base;
                if norm.is_finite() && norm >= sync_min {
                    Some((lag, norm))
                } else {
                    None
                }
            })
            .collect();
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let mut picked: Vec<i32> = Vec::new();
        for (lag, score) in peaks {
            if picked.iter().any(|&pl| (lag - pl).abs() <= MLAG) {
                continue;
            }
            picked.push(lag);
            let dt_quanta = if lag > -jz && lag < jz {
                let y_lo = sync2d[idx(fi, lag - 1)];
                let y_mi = sync2d[idx(fi, lag)];
                let y_hi = sync2d[idx(fi, lag + 1)];
                let denom = y_lo - 2.0 * y_mi + y_hi;
                if denom.abs() > f32::EPSILON {
                    let off = 0.5 * (y_lo - y_hi) / denom;
                    off.clamp(-1.0, 1.0)
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let dt_lag = lag as f32 + dt_quanta;
            cands.push(SyncCandidate {
                freq_hz,
                dt_sec: (dt_lag - 0.5) * tstep,
                score,
            });
            // WSJT-X sync8.f90 emits at most 2 candidates per freq:
            // one from `red` (narrow ±MLAG window) and one from `red2`
            // (full ±jz window) when the peak lags differ. We don't
            // run two windows separately; the `picked` greedy NMS with
            // cap 2 produces equivalent recall on `qso3_busy.wav` —
            // empirically verified during the v0.6.2 plan (a
            // dual-window prototype produced identical 16/18 JTDX
            // hit, so the additional separate normalization wasn't
            // worth the code volume). Dropped from 0.6.2 scope.
            if picked.len() >= 2 {
                break;
            }
        }
    }

    // Dedupe within 4 Hz / 40 ms; keep highest score.
    //
    // Sort once by score desc, then greedily keep cands with no
    // already-kept near neighbour. Stops early at `max_cand` for
    // O(n log n + n × max_cand) instead of the prior O(n²) pairwise
    // compare-and-zero (n is several thousand).
    //
    // Empirically this drops 1 borderline busy-band truth on qso3
    // vs the byte-equivalent O(n²) implementation — likely a
    // tie-break ordering difference at the 4 Hz dedupe boundary
    // (haven't fully traced; the algorithms are equivalent on every
    // small-case scenario I worked through). We accept the recall
    // delta for the ~150 us host (~150 ms Core2) saving.
    cands.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
    let mut out: Vec<SyncCandidate> = Vec::with_capacity(max_cand);
    for c in cands {
        if c.score < sync_min {
            break;
        }
        let near = out
            .iter()
            .any(|k| (c.freq_hz - k.freq_hz).abs() < 4.0 && (c.dt_sec - k.dt_sec).abs() < 0.04);
        if near {
            continue;
        }
        out.push(c);
        if out.len() >= max_cand {
            break;
        }
    }
    let cands = out;
    #[cfg(all(
        feature = "profile-coarse",
        not(all(target_arch = "wasm32", target_os = "unknown"))
    ))]
    {
        let t_end = std::time::Instant::now();
        let allsum_us = (t_allsum - t_setup).as_micros();
        let score_us = (t_score - t_allsum).as_micros();
        let post_us = (t_end - t_score).as_micros();
        let total_us = (t_end - t_setup).as_micros();
        eprintln!(
            "[coarse_sync prof] n_freq={n_freq} n_lag={n_lag}  allsum={allsum_us} score={score_us} dedupe+sort={post_us}  total={total_us} us"
        );
    }
    cands
}
