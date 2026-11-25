//! Coarse-sync stage: Costas-array correlation search across the
//! power spectrogram. Mirrors WSJT-X `lib/ft8/sync8.f90` — 16-bin
//! sliding-window allsum noise estimator with `tone_step_bins = 2.0`
//! exactly at `NFFT_SPEC = 3840`.
//!
//! ε.3 of the `docs/CLEANUP_2026_05.md` `decode_block` split. The
//! parent (`decode_block.rs`) re-exports the public entries so
//! external callers — host `decode.rs`, embedded `stage1_inc`,
//! integration tests — keep the same
//! `mfsk_core::ft8::decode_block::*` paths.

use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::params::{COSTAS, COSTAS_POS, NSPS, NTONES};
use super::source::{AudioSource, SliceSource};
use super::spectrogram::{
    CoarseAcc, Spectrogram, compute_spectrogram_bins_src, spectrogram_gain, spectrogram_n_freq,
};
use super::types::AudioSample;
use super::types::{
    NFFT_SPEC, NSSY, NSTEP, SAMPLE_RATE_HZ, TONE_SPACING_HZ, TX_START_OFFSET_S, ratio_eps,
    sync_lag_s,
};
use crate::engine::sync::SyncCandidate;

// ── Coarse sync ─────────────────────────────────────────────────────────────

/// Smallest candidate list for which the dedupe compares `sync8.f90`'s integer-lag DT.
///
/// `sync8.f90:143` drops a candidate within 4 Hz and `tdiff < 0.04` s of a stronger one, on
/// `(jpeak-0.5)*tstep`. This crate's candidate DT is parabolically interpolated, and deduping on
/// that dropped weak candidates `jt9` keeps (`191111_110615`: 903.125 / 909.375 Hz beside a +22 dB
/// signal, whose residue `jt9` decodes and subtracts; #721). The faithful rule keeps a few more
/// near-duplicates of strong signals, which a lean list has no slot for: at the embedded
/// `PASS1_LIMIT_DEFAULT` of 30 they pushed `OH3NIV ZS6S -03` (`qso1.wav`, score 2.06, rank 27 before)
/// out, 4 -> 3 of 5 in `ft8_decode_block_real_qso`. `jt9` has `MAXCAND` 600; the host configs ask for
/// 60 (research) to 200+ (default), the embedded ones for 30. Below this the interpolated DT is
/// compared, as before.
pub(crate) const FAITHFUL_DEDUPE_MIN_CAND: usize = 60;

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
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::CoarseSync);
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        None,
        None,
        PartialBlock2::Score,
    )
}

/// [`coarse_sync`] with an explicit ±lag search window in seconds,
/// instead of the crate default (`SYNC_LAG_S_DEFAULT` /
/// `MFSK_SYNC_LAG_S`, WSJT-X's own ±2.5 s).
///
/// The window is not purely a speed knob — it changes *which*
/// candidates exist and how they rank, so a caller whose downstream
/// use has its own timing assumptions should state them here rather
/// than inherit the decoder's. Two real cases (issue #280):
///
/// - [`bootstrap_dt_median`](crate::engine::sync::bootstrap_dt_median)
///   needs a window comparable to the timing error it is trying to
///   estimate. At ±2.5 s a strong signal's far-lag ghost (secondary
///   channel, see this module's `coarse_sync_inner`) can outscore its own true
///   peak — verified as WSJT-X behaviour too, by probing real `jt9`'s
///   `sync8` on `qso3_busy.wav` — which invalidates a top-K DT
///   median over the raw list.
/// - Clock-synced deployments (NTP / GPS embedded targets) live well
///   within ±1 s and pay for the wider scan in `n_lag`-linear
///   `sync2d` work for nothing.
pub fn coarse_sync_with_lag(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    sync_lag_s: f32,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        None,
        Some(sync_lag_s),
        PartialBlock2::Score,
    )
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
/// `m5stack-s3`, `m5stack-s3-app`) builds the
/// allsum incrementally during slot capture and passes it back here, so
/// this is a stable public surface — not a benchmark-only escape hatch.
pub fn coarse_sync_with_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum: &[CoarseAcc],
) -> Vec<SyncCandidate> {
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        Some(allsum),
        None,
        PartialBlock2::Score,
    )
}

/// [`coarse_sync_with_allsum`] with an explicit ±lag search window in
/// seconds. See [`coarse_sync_with_lag`] for when a caller should pin
/// the window rather than inherit the crate default.
///
/// `allsum` is indexed by `(fi, m)` only, so the same table is valid
/// at any lag window — no need to rebuild it when pinning.
pub fn coarse_sync_with_allsum_and_lag(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum: &[CoarseAcc],
    sync_lag_s: f32,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        Some(allsum),
        Some(sync_lag_s),
        PartialBlock2::Score,
    )
}

/// Length of the allsum buffer that [`coarse_sync_with_allsum`]
/// expects for the given `spec` + `freq_min..=freq_max` band.
/// Returns 0 when the band has no candidates (then
/// `coarse_sync_with_allsum` would return an empty vec immediately
/// regardless of `allsum`).
/// [`coarse_sync_with_allsum_and_lag`] for a spectrogram only part of
/// which is filled, **scoring only the lags that still have all three
/// Costas blocks**.
///
/// `valid_rows` is how many leading time rows hold a spectrum. A
/// streaming producer emits its bundle before the slot ends and must
/// still declare the full `n_time`, because that is what sets the
/// allsum's stride ([`coarse_allsum_len`]) — so its tail rows are
/// present and zero, and block 2's trailing symbols reach into them at
/// large positive lag. Those rows add nothing to the score's sums, so
/// the score is not wrong; it is computed over fewer symbols and is
/// correspondingly noisier, and a noisy score competes on equal terms
/// with a quiet one.
///
/// This refuses to score such a lag at all. See the gate in
/// `coarse_sync_inner` for why that is a divergence from WSJT-X and
/// why it is opt-in; whole-slot callers use
/// [`coarse_sync_with_allsum_and_lag`] and are unaffected.
/// What to do with a lag whose Costas block 2 is not fully covered by
/// the spectrogram's real rows.
///
/// A streaming producer emits its bundle before the slot ends, so its
/// tail rows are present and zero. Those rows add nothing to either
/// sum, so the score is not *wrong* — `ft8_coarse_partial_blocks`
/// measures a zeroed tail as bit-identical to the same symbols
/// skipped — it is simply computed over fewer Costas symbols.
///
/// **How much fewer, in the shipped configuration: one.** Under
/// `fixed-point` `NSSY = NSPS / NSTEP = 2`, so block 2 runs from
/// `jstrt + 2*72 = 150` to `162` (`nstep-half` keeps the rounded
/// `jstrt` of 6, see `coarse_sync_inner`), the real rows end at 173, and
/// the widest lag the ±1.0 s search reaches is 13 — which leaves
/// `(173 - 163) / 2 + 1 = 6` of 7 symbols. That bounds every
/// "the score is noisier" argument about this population at
/// `sqrt(20/21)`, i.e. 2 %.
///
/// A third variant that shrank a partial score toward the noise floor
/// by exactly that factor was written and measured on 2026-09-20
/// (`mirror_partial_block2_policies`, three recordings, fixed-point):
/// it moved 8 of 77 candidates out of a 30-deep pass-1 list, 2 of 31
/// out of the refined top-15, and **changed no decode count on any
/// fixture at any phase**. It was removed rather than shipped. If this
/// population needs handling, the handle is not its score.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartialBlock2 {
    /// Score it and take the result — what WSJT-X does (`sync8.f90`
    /// guards the read and skips the symbol). Correct for a whole-slot
    /// caller, whose spectrogram ends where its slot does, and the
    /// shipping behaviour of the streaming path too.
    Score,
    /// Refuse to score the lag at all.
    ///
    /// **One-sided**, and that is not a choice — negative lag moves
    /// block 2 *earlier*, where it is always fully covered, so nothing
    /// below `-0.88 s` is ever gated. An earlier version of this doc
    /// claimed the gate costs the 0.26-1.35 % of on-air decodes seen
    /// below `-0.88 s`; it cannot, and the measurement below confirms
    /// the gated arm keeps every distinct message at aligned phases.
    ///
    /// **A no-op at the shipped window, and kept anyway** (decided
    /// 2026-09-20). Since `decode_pipeline` searches exactly the
    /// coverage ceiling, no lag it visits truncates block 2 and this
    /// never fires. It stays because the alternative to narrowing the
    /// window — emitting earlier and gating instead of searching
    /// narrower — is a real experiment, and deleting the gate would
    /// close it. Do not remove it as dead code without that decision
    /// being revisited.
    ///
    /// What it does cost is real stations **above** `+0.88 s`, which
    /// exist whenever the recording's own alignment puts them there.
    /// Measured at grid-locked phases (±0.10 s, fixed-point,
    /// `mirror_partial_block2_policies`): `qso3_busy` 137 → 140
    /// decodes, `qso1` 49 → 36 and one distinct station lost, `qso2`
    /// 45 → 34. **Do not turn this on from one fixture.**
    Gate {
        /// How many leading time rows hold a real spectrum.
        valid_rows: usize,
    },
}

/// [`coarse_sync_with_allsum_and_lag`] for a spectrogram only part of
/// which is filled, **scoring only the lags that still have all three
/// Costas blocks** — i.e. [`PartialBlock2::Gate`], whose doc carries
/// what that costs and what it does not.
///
/// `valid_rows` is how many leading time rows hold a spectrum. A
/// streaming producer emits its bundle before the slot ends and must
/// still declare the full `n_time`, because that is what sets the
/// allsum's stride ([`coarse_allsum_len`]) — so its tail rows are
/// present and zero, and block 2's trailing symbols reach into them at
/// large positive lag.
pub fn coarse_sync_with_allsum_lag_and_rows(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum: &[CoarseAcc],
    sync_lag_s: f32,
    valid_rows: usize,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        Some(allsum),
        Some(sync_lag_s),
        PartialBlock2::Gate { valid_rows },
    )
}

/// [`coarse_sync_with_lag`] with the incomplete-block-2 policy chosen
/// by the caller, building its own allsum.
///
/// The allsum is a precompute and does not change any score, so this
/// and [`coarse_sync_with_allsum_lag_and_partial`] rank identically —
/// which is what lets a host harness compare policies without also
/// reproducing the board's allsum plumbing.
pub fn coarse_sync_with_lag_and_partial(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    sync_lag_s: f32,
    partial: PartialBlock2,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        None,
        Some(sync_lag_s),
        partial,
    )
}

/// [`coarse_sync_with_allsum_lag_and_rows`] with the policy chosen by
/// the caller rather than fixed at [`PartialBlock2::Gate`].
///
/// Exists so the two treatments of an incomplete block 2 can be
/// measured against each other on the same slot. See
/// [`PartialBlock2`]; a whole-slot caller wants
/// [`coarse_sync_with_allsum_and_lag`] and none of this.
pub fn coarse_sync_with_allsum_lag_and_partial(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum: &[CoarseAcc],
    sync_lag_s: f32,
    partial: PartialBlock2,
) -> Vec<SyncCandidate> {
    coarse_sync_inner(
        spec,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
        Some(allsum),
        Some(sync_lag_s),
        partial,
    )
}

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

/// One carrier's allsum row for the score loop: `dst[m] = Σ_{k=0..NTONES-2} spec[i_carrier + 2k, m]` for
/// each `m` in `needed_m` (the other cells are not read, and keep whatever they held), bins clamped to the
/// top row as [`fill_allsum_row`] clamps them.
fn fill_allsum_carrier(
    spec: &Spectrogram,
    i_carrier: usize,
    needed_m: &[usize],
    dst: &mut [CoarseAcc],
) {
    let upper = spec.n_freq - 1;
    for &m in needed_m {
        let mut s = CoarseAcc::default();
        for k in 0..(NTONES - 1) {
            s += spec.power_acc((i_carrier + 2 * k).min(upper), m);
        }
        dst[m] = s;
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

/// The slot-wide constants of one coarse search: the time grid (`jstrt`, `jz`) and the carrier-bin range
/// `ia..=ib` it covers, in full-spectrogram bins. `None` when the lag window or the band is empty.
struct SyncGrid {
    df: f32,
    tstep: f32,
    jstrt: i32,
    jz: i32,
    tone_step_bins: f32,
    ia: usize,
    ib: usize,
    /// Bins above a carrier that its top tone reads (a carrier `i` reads up to `i + max_tone_off - 1`).
    max_tone_off: usize,
}

fn sync_grid(
    n_time: usize,
    spec_n_freq: usize,
    freq_min: f32,
    freq_max: f32,
    sync_lag_override_s: Option<f32>,
) -> Option<SyncGrid> {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    // `sync8.f90`: `jstrt=0.5/tstep` assigns a real to an integer, which
    // truncates (12.5 -> 12 at NSTEP = NSPS/4), and its spectrum index `m`
    // is 1-based where this one is 0-based: lag 0 is row `jstrt - 1`. This
    // used to be `round(0.5/tstep)` = 13 on a 0-based row, which put every
    // lag two steps (80 ms) away from upstream's on identical `sync2d`
    // values: the reported coarse DT was 80 ms early, the `±MLAG` primary
    // window covered a different second, and `ft8b`'s ±10-sample search
    // started from another place (#721: `CQ IZ8IQO JM89`, jt9 lag 29, here
    // 27). `JZ=62` is likewise 2.5/tstep truncated, not rounded to 63.
    //
    // That grid is upstream's at `NSTEP = NSPS/4` only. Under `nstep-half`
    // (the fixed-point embedded build, NSTEP = NSPS/2, 80 ms steps) WSJT-X
    // has no grid to follow, and the same formula would start lag 0 at
    // 0.40 s, a -60 ms coarse-DT bias that the board's driver (no `ft8b`
    // ±10-sample search) cannot absorb: `true_ship_driver_meets_fixed_golden_floor`
    // fell 7 -> 2 of 20 on `qso3_busy.wav`. It keeps the rounded origin
    // (0.48 s, +20 ms) it was tuned on.
    #[cfg(not(feature = "nstep-half"))]
    let (jstrt, requested_jz) = (
        (TX_START_OFFSET_S / tstep) as i32 - 1,
        (sync_lag_override_s.unwrap_or_else(sync_lag_s) / tstep) as i32,
    );
    #[cfg(feature = "nstep-half")]
    let (jstrt, requested_jz) = (
        (TX_START_OFFSET_S / tstep).round() as i32,
        (sync_lag_override_s.unwrap_or_else(sync_lag_s) / tstep).round() as i32,
    );
    let jz = bounded_sync_lag_steps(n_time, jstrt, requested_jz)?;
    let tone_step_bins = TONE_SPACING_HZ / df;

    // Carrier-bin search range. Reserve room above the carrier for the
    // top tone (round(7 * tone_step_bins)).
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let nh1 = spec_n_freq;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
    if ib < ia {
        return None;
    }
    Some(SyncGrid {
        df,
        tstep,
        jstrt,
        jz,
        tone_step_bins,
        ia,
        ib,
        max_tone_off,
    })
}

/// What the score loop keeps of each carrier: its best lag in the fixed `±MLAG` window (primary) and in
/// the whole window (secondary), the score there and the parabola's offset. Everything the candidate
/// selection needs, a few floats per carrier instead of the carrier's whole lag row.
#[derive(Default)]
struct CarrierPeaks {
    red_primary: Vec<f32>,
    jpeak_primary: Vec<i32>,
    quanta_primary: Vec<f32>,
    red_secondary: Vec<f32>,
    jpeak_secondary: Vec<i32>,
    quanta_secondary: Vec<f32>,
    /// [`spec_hard_sync`] at each carrier's primary / secondary peak; empty unless asked for
    /// (`score_carriers`' `hard_sync`).
    nsync_primary: Vec<u8>,
    nsync_secondary: Vec<u8>,
}

impl CarrierPeaks {
    fn append(&mut self, mut o: CarrierPeaks) {
        self.red_primary.append(&mut o.red_primary);
        self.jpeak_primary.append(&mut o.jpeak_primary);
        self.quanta_primary.append(&mut o.quanta_primary);
        self.red_secondary.append(&mut o.red_secondary);
        self.jpeak_secondary.append(&mut o.jpeak_secondary);
        self.quanta_secondary.append(&mut o.quanta_secondary);
        self.nsync_primary.append(&mut o.nsync_primary);
        self.nsync_secondary.append(&mut o.nsync_secondary);
    }
}

/// How many of the 21 Costas symbols of a candidate at tone-0 bin `bin0` (of `spec`) and start `dt_sec` have
/// their strongest of the 8 tone bins on the expected tone: a hard-decision sync count read from the
/// spectrogram alone, with no DFT and no audio. A real signal scores far above anything else on the list
/// (the ghosts of a strong signal, whose tones only partly line up, score low): on `qso3_busy.wav`'s 300
/// best coarse candidates 10 or more of 21 is 20 real signals and 4 others.
///
/// **A deliberate divergence from WSJT-X**, which has no such gate before `ft8b`'s downsample and its own
/// `nsync` (which is the same count on the complex symbol spectra, past `ft8b.f90`'s `nsync <= 6` bail-out).
/// This is the cheap stand-in for ranking a long list without running the per-candidate DFTs on all of it, as
/// `ft8_lib`'s `ft8_find_sync` does with its hard-sync count. Used only by [`coarse_sync_gated`].
fn spec_hard_sync(spec: &Spectrogram, bin0: usize, dt_sec: f32, tstep: f32) -> u8 {
    let mut ok = 0u8;
    for &start in COSTAS_POS.iter() {
        for (n, &expect) in COSTAS.iter().enumerate() {
            let sym = (start + n) as f32;
            let t = TX_START_OFFSET_S + dt_sec + sym * (NSPS as f32 / SAMPLE_RATE_HZ);
            let j = (t / tstep).round();
            if j < 0.0 || j as usize >= spec.n_time {
                continue;
            }
            let j = j as usize;
            let mut best = 0usize;
            let mut best_p = f32::NEG_INFINITY;
            for tone in 0..NTONES {
                // i32 under `fixed-point` (`CoarseAcc`), f32 otherwise
                #[allow(clippy::unnecessary_cast)]
                let v = spec.power_acc(bin0 + 2 * tone, j) as f32;
                if v > best_p {
                    best_p = v;
                    best = tone;
                }
            }
            if best == expect {
                ok += 1;
            }
        }
    }
    ok
}

/// Score the carriers `ia..=ib` (bins **of `spec`**) at every lag. A carrier's score reads only its own
/// bins, `i..=i + 14`, so `spec` may be a band of the full spectrogram (`compute_spectrogram_bins`) that
/// holds those and nothing else: the result is the same as from the full one.
fn score_carriers(
    spec: &Spectrogram,
    ia: usize,
    ib: usize,
    g: &SyncGrid,
    external_allsum: Option<&[CoarseAcc]>,
    partial: PartialBlock2,
    hard_sync: bool,
) -> CarrierPeaks {
    let (jstrt, jz, tone_step_bins) = (g.jstrt, g.jz, g.tone_step_bins);
    let n_freq = ib - ia + 1;
    let n_lag = (2 * jz + 1) as usize;
    // One carrier's scores over every lag, reused. The whole `n_freq x n_lag` grid used to be kept (226 KB
    // fixed-point, 449 KB f32) for two readers only: the per-carrier peak search and the parabola on the
    // peak's two neighbours. Both are done per carrier right after its row is scored, with the same
    // arithmetic in the same order, so what leaves the loop is bit-identical and the grid is gone.
    let mut sync_row = vec![0.0f32; n_lag];
    let idx = |lag: i32| (lag + jz) as usize;
    // `sync8.f90`'s `mlag`: the fixed half-window of the primary channel,
    // in coarse-sync steps (NSTEP = NSPS/4 = 0.04 s). 10 (±0.40 s) through
    // WSJT-X 2.7, 13 (±0.52 s) from 3.0 (read at the `v2.7.0` / `v3.0.0`
    // tags).
    //
    // **13 since #439**, as upstream. It was left at 10 in #438: 13 was a
    // no-op on the f32 tier-C sweep (800 trials, byte-identical) and cost one
    // hit on the fixed-point ship config (`ft8_qso3_apoff_recall` 12/20 →
    // 11/20, because a wider primary window reorders the 15 candidates that
    // config keeps).
    //
    // Re-measured with the rest of the 3.x port in, `qso3_busy`, fixed-point
    // ship shape (`decode_block`, `EMBEDDED`, sync_min 1.3), hits of the 20
    // known signals by `max_cand`:
    //
    // | max_cand | 10-12 | 15-20 | 25-30 | 40 and up |
    // |---|---|---|---|---|
    // | `MLAG` 10 | 11 | 12 | 15 | 16 |
    // | `MLAG` 13 | 11 | 11 | 14 | 16 |
    //
    // One signal fewer at every cap from 15 to 30, the range the ship config
    // can use; equal from 40 up, where upstream's `MAXCAND` of 1000 lives. On
    // the busy-band corpus (DT -0.5..+1.5 s) 13 is neutral to slightly better
    // (.sic_early() unexpected decodes 12 -> 9, recall +0.1 pt) and the ft8
    // sweeps are unchanged. Adopted because this crate follows the reference
    // and nothing measured says the wider window is worse; the ship floor in
    // `ft8_qso3_apoff_recall` is 11 for it.
    const MLAG: i32 = 13;

    // Two independent noise-floor channels, matching WSJT-X `sync8.f90`
    // exactly (issue #280). Collapsing these into a single
    // `±jz`-dependent channel — the pre-#280 shape of this function —
    // let the noise floor `base` scale with the requested lag window:
    // widening `MFSK_SYNC_LAG_S` toward WSJT-X's own `±2.5s` inflated
    // the *only* floor available here (per-bin max over a wider window
    // has a higher expected value under basic extreme-value
    // statistics), silently pushing weak real decodes below `sync_min`
    // even though WSJT-X's actual primary channel — anchored to a
    // fixed `mlag` (10 through 2.7, 13 from 3.0; see `MLAG`) regardless of
    // `JZ` — would still have caught
    // them. `red_primary`/`base_primary` restores that JZ-independent
    // anchor; `red_secondary`/`base_secondary` (the old sole channel)
    // now plays its correct WSJT-X role: a supplementary second
    // candidate per bin, contributed only when its peak lag disagrees
    // with the primary's.
    let primary_half = MLAG.min(jz);
    let mut red_primary = vec![0.0f32; n_freq];
    let mut jpeak_primary = vec![0i32; n_freq];
    let mut quanta_primary = vec![0.0f32; n_freq];
    let mut red_secondary = vec![0.0f32; n_freq];
    let mut jpeak_secondary = vec![0i32; n_freq];
    let mut quanta_secondary = vec![0.0f32; n_freq];
    let mut nsync_primary: Vec<u8> = Vec::new();
    let mut nsync_secondary: Vec<u8> = Vec::new();
    let ratio_eps = ratio_eps();

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

    // `Σ_{k=0..NTONES-2} spec[i_carrier + 2*k, m]` (the seven Costas tones; tone 7 is data-only, as
    // `sync8.f90:66`) for every `m` the score loop will read, **one carrier at a time**.
    //
    // This used to be built for all carriers first (`n_freq * n_time` cells, 660 KB fixed-point, 1.33 MB
    // f32, a fifth of the slot's working set) although the score loop below is carrier-outer and reads
    // one carrier's row at a time. The row is now filled just before it is read, into a buffer of
    // `n_time` cells. The values are the ones the full table held: f32 was already a direct per-cell
    // sum in this order, and the fixed-point sliding window it used is an integer identity of the direct
    // sum (same clamp at both edges), so the scores are bit-identical.
    //
    // Phase-E2: a caller can still pass a pre-built table (built incrementally during slot capture by
    // `stage1_inc`); it is then read as before and nothing is computed here.
    let mut own_row: Vec<CoarseAcc> = Vec::new();
    if let Some(ext) = external_allsum {
        debug_assert_eq!(
            ext.len(),
            n_freq * spec.n_time,
            "external allsum length mismatch (expected n_freq * spec.n_time)"
        );
    } else {
        own_row = vec![CoarseAcc::default(); spec.n_time];
    }

    // Three identical Costas arrays at symbol positions 0, 36, 72.
    //
    // **Bounds-check hoist**. For the standard FT8 slot:
    //   block 0: m = lag + jstrt + NSSY*n   ∈ [-7..31] over (lag, n)
    //   block 1: m = lag + jstrt + 144 + NSSY*n  ∈ [65..103]
    //   block 2: m = lag + jstrt + 288 + NSSY*n  ∈ [137..175]
    //
    // Block 0 can dip below 0 and block 2 can run past `n_time`, exactly
    // the two bounds guarded by WSJT-X `sync8.f90`. Compute the valid
    // symbol ranges once per lag instead of checking every inner read.
    // Block 1 remains fully in range for every accepted lag window.
    debug_assert!(
        m_base[1][0] - jz >= 0 && m_base[1][COSTAS.len() - 1] + jz < spec.n_time as i32,
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
        let allsum_row: &[CoarseAcc] = match external_allsum {
            Some(ext) => &ext[fi * n_time..(fi + 1) * n_time],
            None => {
                fill_allsum_carrier(spec, ia + fi, &needed_m, &mut own_row);
                &own_row
            }
        };
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
            // Block 1 is always fully in range.
            for n in 0..COSTAS.len() {
                let m_u = (m_base[1][n] + lag) as usize;
                let tbin_lo = tbin_lo_arr[n];
                t_blocks[1] += spec.power_acc(tbin_lo, m_u);
                t0_blocks[1] += allsum_row[m_u];
            }
            // Block 2: WSJT-X guards `m + nssy*72 <= NHSYM` because
            // positive lag can push its trailing symbols past the slot.
            let bk2_n_end = valid_trailing_symbol_count(m_base[2][0], lag, n_time);
            // **A lag whose block 2 is incomplete is not scored at
            // all** — when the caller has said how many rows are real.
            //
            // Both scores below contain block 2, so an incomplete one
            // makes both of them a ratio over fewer Costas symbols.
            // That does not *inflate* the score: numerator and
            // denominator are sums over the same symbols and the ratio
            // is self-normalising (`ft8_coarse_partial_blocks` measures
            // it — a zeroed tail scores bit-identically to the same
            // symbols skipped). What it does is raise the *variance*:
            // a noise coincidence over 14 symbols reaches a given
            // ratio far more often than over 21, which is why the
            // ±1.75 s widen filled the shortlist with candidates at
            // −1.64 / +1.72 / +1.08 that scored 20-30 and decoded
            // nothing (radio, 2026-09-19).
            //
            // Negative lag moves block 2 *earlier*, so it is never the
            // truncated one; the asymmetry the measurement asked for
            // (0 % of real decodes past +0.88 s, 0.26-1.35 % below
            // −0.88 s, on 1 382 on-air decodes) falls out without a
            // special case.
            //
            // **A deliberate divergence from WSJT-X**, which scores
            // the truncated block and takes the result (`sync8.f90`
            // guards the read and skips). Opt-in through `valid_rows`
            // for exactly that reason: the host passes `None` and is
            // bit-identical, the streaming embedded pipeline passes
            // its fill point and pays for the divergence. Upstream
            // never needs it, because upstream's spectrogram ends
            // where its slot ends.
            // Costas symbols block 2 actually carries: its trailing
            // ones go at positive lag or past the fill point. Block 1
            // is always whole and block 0 loses only leading symbols
            // at negative lag, which `sync_tail` does not see.
            let n_cov2 = match partial {
                PartialBlock2::Score => bk2_n_end,
                PartialBlock2::Gate { valid_rows } => {
                    valid_trailing_symbol_count(m_base[2][0], lag, valid_rows.min(n_time))
                }
            };
            if matches!(partial, PartialBlock2::Gate { .. }) && n_cov2 < COSTAS.len() {
                sync_row[idx(lag)] = f32::NEG_INFINITY;
                continue;
            }
            for n in 0..bk2_n_end {
                let m_u = (m_base[2][n] + lag) as usize;
                let tbin_lo = tbin_lo_arr[n];
                t_blocks[2] += spec.power_acc(tbin_lo, m_u);
                t0_blocks[2] += allsum_row[m_u];
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

            sync_row[idx(lag)] = sync_all.max(sync_tail);
        }

        // This carrier's peaks: `-MLAG..=MLAG` (primary) and the whole window (secondary), first maximum
        // wins; then the parabola on each peak's neighbours (`sync8.f90`'s `peakup`).
        let mut best_p = f32::NEG_INFINITY;
        let mut lag_p = -primary_half;
        let mut best_s = f32::NEG_INFINITY;
        let mut lag_s = -jz;
        for lag in -jz..=jz {
            let v = sync_row[idx(lag)];
            if v > best_s {
                best_s = v;
                lag_s = lag;
            }
            if (-primary_half..=primary_half).contains(&lag) && v > best_p {
                best_p = v;
                lag_p = lag;
            }
        }
        let quanta_at = |lag: i32| -> f32 {
            if lag > -jz && lag < jz {
                let y_lo = sync_row[idx(lag - 1)];
                let y_mi = sync_row[idx(lag)];
                let y_hi = sync_row[idx(lag + 1)];
                let denom = y_lo - 2.0 * y_mi + y_hi;
                if denom.abs() > f32::EPSILON {
                    (0.5 * (y_lo - y_hi) / denom).clamp(-1.0, 1.0)
                } else {
                    0.0
                }
            } else {
                0.0
            }
        };
        red_primary[fi] = best_p;
        jpeak_primary[fi] = lag_p;
        quanta_primary[fi] = quanta_at(lag_p);
        red_secondary[fi] = best_s;
        jpeak_secondary[fi] = lag_s;
        quanta_secondary[fi] = quanta_at(lag_s);
        if hard_sync {
            // the candidate's start as `select_candidates` reports it: `(lag + parabola - 0.5) * tstep`
            let at = |lag: i32, q: f32| {
                spec_hard_sync(spec, ia + fi, (lag as f32 + q - 0.5) * g.tstep, g.tstep)
            };
            let ns_p = at(lag_p, quanta_primary[fi]);
            nsync_primary.push(ns_p);
            nsync_secondary.push(if lag_s == lag_p {
                ns_p
            } else {
                at(lag_s, quanta_secondary[fi])
            });
        }
    }
    CarrierPeaks {
        red_primary,
        jpeak_primary,
        quanta_primary,
        red_secondary,
        jpeak_secondary,
        quanta_secondary,
        nsync_primary,
        nsync_secondary,
    }
}

/// Rank the carriers' peaks: the 40th-percentile noise floor over **all** carriers, both channels, the
/// 4 Hz / 40 ms dedupe, the `max_cand` cut.
fn select_candidates(
    p: &CarrierPeaks,
    g: &SyncGrid,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    select_candidates_with_sync(p, g, sync_min, max_cand)
        .into_iter()
        .map(|c| c.0)
        .collect()
}

/// [`select_candidates`] with each candidate's [`spec_hard_sync`] count (0 where `p` holds none).
fn select_candidates_with_sync(
    p: &CarrierPeaks,
    g: &SyncGrid,
    sync_min: f32,
    max_cand: usize,
) -> Vec<(SyncCandidate, u8)> {
    let n_freq = p.red_primary.len();
    let nsync_at = |fi: usize, primary: bool| -> u8 {
        let v = if primary {
            &p.nsync_primary
        } else {
            &p.nsync_secondary
        };
        v.get(fi).copied().unwrap_or(0)
    };
    let (df, tstep, ia) = (g.df, g.tstep, g.ia);
    let (red_primary, jpeak_primary, quanta_primary) =
        (&p.red_primary, &p.jpeak_primary, &p.quanta_primary);
    let (red_secondary, jpeak_secondary, quanta_secondary) =
        (&p.red_secondary, &p.jpeak_secondary, &p.quanta_secondary);
    let percentile_floor = |red: &[f32]| -> f32 {
        let mut sorted = red.to_vec();
        let pct_idx = ((0.40 * n_freq as f32) as usize).min(n_freq - 1);
        sorted.select_nth_unstable_by(pct_idx, |a, b| {
            a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)
        });
        sorted[pct_idx].max(f32::EPSILON)
    };
    let base_primary = percentile_floor(red_primary);
    let base_secondary = percentile_floor(red_secondary);

    // Parabolic sub-quantum interpolation around a chosen peak lag —
    // shared by both channels. Only valid strictly inside the searched
    // window (needs both neighbours); falls back to 0 at the edge.
    let mut cands: Vec<(SyncCandidate, f32, u8)> = Vec::new();
    for fi in 0..n_freq {
        let i_carrier = ia + fi;
        let freq_hz = i_carrier as f32 * df;

        let lag_p = jpeak_primary[fi];
        let norm_p = red_primary[fi] / base_primary;
        if norm_p.is_finite() && norm_p >= sync_min {
            let dt_lag = lag_p as f32 + quanta_primary[fi];
            cands.push((
                SyncCandidate {
                    freq_hz,
                    dt_sec: (dt_lag - 0.5) * tstep,
                    score: norm_p,
                },
                (lag_p as f32 - 0.5) * tstep,
                nsync_at(fi, true),
            ));
        }

        // Secondary candidate only when the full-window peak actually
        // disagrees with the fixed-window one — matches `sync8.f90`'s
        // `if (jpeak2(n)==jpeak(n)) cycle`.
        let lag_s = jpeak_secondary[fi];
        if lag_s != lag_p {
            let norm_s = red_secondary[fi] / base_secondary;
            if norm_s.is_finite() && norm_s >= sync_min {
                let dt_lag = lag_s as f32 + quanta_secondary[fi];
                cands.push((
                    SyncCandidate {
                        freq_hz,
                        dt_sec: (dt_lag - 0.5) * tstep,
                        score: norm_s,
                    },
                    (lag_s as f32 - 0.5) * tstep,
                    nsync_at(fi, false),
                ));
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
    cands.sort_by(|a, b| b.0.score.partial_cmp(&a.0.score).unwrap());
    let lag_dedupe = max_cand >= FAITHFUL_DEDUPE_MIN_CAND;
    let mut out: Vec<(SyncCandidate, u8)> = Vec::with_capacity(max_cand);
    let mut out_raw: Vec<(f32, f32)> = Vec::with_capacity(max_cand);
    for (c, raw_dt, ns) in cands {
        if c.score < sync_min {
            break;
        }
        let key_dt = if lag_dedupe { raw_dt } else { c.dt_sec };
        let near = out_raw
            .iter()
            .any(|k| (c.freq_hz - k.0).abs() < 4.0 && (key_dt - k.1).abs() < 0.04);
        if near {
            continue;
        }
        out_raw.push((c.freq_hz, key_dt));
        out.push((c, ns));
        if out.len() >= max_cand {
            break;
        }
    }
    out
}

fn coarse_sync_inner(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    external_allsum: Option<&[CoarseAcc]>,
    sync_lag_override_s: Option<f32>,
    partial: PartialBlock2,
) -> Vec<SyncCandidate> {
    let Some(g) = sync_grid(
        spec.n_time,
        spec.n_freq,
        freq_min,
        freq_max,
        sync_lag_override_s,
    ) else {
        return Vec::new();
    };
    let peaks = score_carriers(spec, g.ia, g.ib, &g, external_allsum, partial, false);
    select_candidates(&peaks, &g, sync_min, max_cand)
}

/// [`coarse_sync`] from the audio, **one band of carriers at a time**: each band's spectrogram
/// (`compute_spectrogram_bins`) holds the band's carriers and the bins their tones read, and is dropped
/// before the next is built, so the spectrogram's memory is that of one band (`1 / n_bands`, plus 14 bins)
/// instead of the whole carrier range. The cost is the frame FFTs, which run once per band.
///
/// Returns exactly what `coarse_sync(&compute_spectrogram(audio, freq_max), ..)` returns, bit for bit: a
/// cell of a band is the cell of the full spectrogram, a carrier's score reads only its own bins, and the
/// per-carrier peaks of all bands are ranked together (the noise floor is a percentile over every
/// carrier). `n_bands` is clamped to `1..=carriers`.
pub fn coarse_sync_banded<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    n_bands: usize,
) -> Vec<SyncCandidate> {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::CoarseSync);
    banded_peaks(&mut SliceSource(audio), freq_min, freq_max, n_bands, false)
        .map_or_else(Vec::new, |(peaks, g)| {
            select_candidates(&peaks, &g, sync_min, max_cand)
        })
}

/// Pool the hard-sync gate ranks: the coarse list is cut to this many by score before [`coarse_sync_gated`]
/// reorders it by hard sync. 300 holds 18 of the 20 known signals of `qso3_busy.wav` and costs a count of
/// 21 x 8 spectrogram cells each; going to 600 adds nothing there.
pub const HARD_SYNC_POOL: usize = 300;

/// The coarse search of the low-memory profile: [`coarse_sync_banded`]'s list of [`HARD_SYNC_POOL`]
/// candidates, **reordered by their hard-sync count** (read from the spectrogram band
/// while it exists) and cut to the `keep` best; ties keep the coarse-score order. Returns each with its
/// count (of 21).
///
/// Why: the coarse score ranks the ghosts of a strong signal above weaker real ones (on `qso3_busy.wav` the
/// first 30 by score hold 9 real signals and 20 ghosts of strong ones; the first 30 by hard sync, 21 real),
/// so a short list by score misses real signals that a short list by hard sync keeps. The per-candidate
/// DFT stages then see 15-30 candidates that are mostly real. **A deliberate divergence from WSJT-X**, see
/// `spec_hard_sync` below; the decodes still pass `ft8b`'s own gates (the 21-symbol `nsync`, BP/CRC).
pub fn coarse_sync_gated<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    keep: usize,
    n_bands: usize,
) -> Vec<(SyncCandidate, u8)> {
    coarse_sync_gated_src(
        &mut SliceSource(audio),
        freq_min,
        freq_max,
        sync_min,
        keep,
        n_bands,
    )
}

/// [`coarse_sync_gated`] from an [`AudioSource`], the form the radio needs: the slot is never held. It reads
/// the source forward `n_bands` times, plus once more in the fixed-point build for the slot's peak (a value
/// the capture could hand over instead). The candidates are the slice version's, bit for bit.
pub fn coarse_sync_gated_src<A: AudioSource>(
    src: &mut A,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    keep: usize,
    n_bands: usize,
) -> Vec<(SyncCandidate, u8)> {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::CoarseSync);
    let Some((peaks, g)) = banded_peaks(src, freq_min, freq_max, n_bands, true) else {
        return Vec::new();
    };
    let mut pool = select_candidates_with_sync(&peaks, &g, sync_min, HARD_SYNC_POOL);
    // stable: equal counts stay in coarse-score order
    pool.sort_by_key(|c| core::cmp::Reverse(c.1));
    pool.truncate(keep);
    pool
}

/// Score every carrier of the band `freq_min..freq_max` a band at a time (the spectrogram of each is built,
/// scored and dropped before the next), with the hard-sync count when `hard_sync`. Reads `src` once for the
/// slot's gain (fixed-point build) and once for each band, forward.
fn banded_peaks<A: AudioSource>(
    src: &mut A,
    freq_min: f32,
    freq_max: f32,
    n_bands: usize,
    hard_sync: bool,
) -> Option<(CarrierPeaks, SyncGrid)> {
    // `()` in the f32 build, which needs nothing from the whole slot; the fixed-point shift otherwise
    #[allow(clippy::let_unit_value)]
    let gain = spectrogram_gain(src);
    banded_peaks_with(freq_min, freq_max, n_bands, hard_sync, |lo, hi| {
        compute_spectrogram_bins_src(src, lo, hi, gain)
    })
}

/// [`banded_peaks`] with the spectrogram of a band of bins from `build(lo, hi)`: from the audio, or from the
/// band baseband (`band_bb`).
fn banded_peaks_with<B: FnMut(usize, usize) -> Spectrogram>(
    freq_min: f32,
    freq_max: f32,
    n_bands: usize,
    hard_sync: bool,
    mut build: B,
) -> Option<(CarrierPeaks, SyncGrid)> {
    let n_time = crate::ft8::params::NMAX / NSTEP - 3;
    let g = sync_grid(
        n_time,
        spectrogram_n_freq(freq_max),
        freq_min,
        freq_max,
        None,
    )?;
    let n_car = g.ib - g.ia + 1;
    let n_bands = n_bands.clamp(1, n_car);
    let mut peaks = CarrierPeaks::default();
    for b in 0..n_bands {
        let a = g.ia + b * n_car / n_bands;
        let z = g.ia + (b + 1) * n_car / n_bands - 1; // last carrier of the band
        let band = build(a, z + g.max_tone_off);
        peaks.append(score_carriers(
            &band,
            0,
            z - a,
            &g,
            None,
            PartialBlock2::Score,
            hard_sync,
        ));
    }
    Some((peaks, g))
}

/// [`coarse_sync_gated`] from the band baseband: the spectrogram of each band of carriers is made from
/// [`BandBaseband`](super::band_bb::BandBaseband)'s samples (`band_bb::spectrogram_bins`), the audio is not
/// read. The candidates are the gated search's up to the baseband's rounding.
pub fn coarse_sync_gated_band(
    bb: &super::band_bb::BandBaseband,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    keep: usize,
    n_bands: usize,
) -> Vec<(SyncCandidate, u8)> {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::CoarseSync);
    let Some((peaks, g)) = banded_peaks_with(freq_min, freq_max, n_bands, true, |lo, hi| {
        bb.spectrogram_bins(lo, hi)
    }) else {
        return Vec::new();
    };
    let mut pool = select_candidates_with_sync(&peaks, &g, sync_min, HARD_SYNC_POOL);
    pool.sort_by_key(|c| core::cmp::Reverse(c.1));
    pool.truncate(keep);
    pool
}

/// Keep the requested symmetric lag window when the always-read middle Costas
/// block fits. Only pathological overrides need clamping; the first and third
/// blocks have the same conditional bounds as WSJT-X `sync8.f90`.
fn bounded_sync_lag_steps(
    spec_n_time: usize,
    start_step: i32,
    requested_steps: i32,
) -> Option<i32> {
    let requested_steps = requested_steps.max(0);
    let second_block_base = start_step + NSSY * COSTAS_POS[1] as i32;
    let second_block_end = second_block_base + NSSY * (COSTAS.len() as i32 - 1);
    let max_positive_steps = (spec_n_time as i32 - 1).checked_sub(second_block_end)?;
    let safe_steps = second_block_base.min(max_positive_steps);
    (safe_steps >= 0).then(|| requested_steps.min(safe_steps))
}

fn valid_trailing_symbol_count(base_step: i32, lag: i32, n_time: usize) -> usize {
    let available = (n_time as i32 - 1) - (base_step + lag);
    if available < 0 {
        0
    } else {
        (available / NSSY + 1).min(COSTAS.len() as i32) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::super::super::params::{COSTAS, COSTAS_POS};
    use super::{
        NSSY, PartialBlock2, Spectrogram, bounded_sync_lag_steps, coarse_sync_inner,
        valid_trailing_symbol_count,
    };

    /// Time rows in a standard 15 s FT8 spectrogram, on whichever grid
    /// is built: 372 quarter-symbol rows by default, 184 half-symbol
    /// ones under `nstep-half` (which `fixed-point` implies, and which
    /// is therefore what every embedded build runs).
    ///
    /// These two tests used to hardcode the quarter-grid numbers, so
    /// they failed for the whole `full,fixed-point` matrix and — worse
    /// — left the arithmetic they cover with **no** coverage on the
    /// grid that ships on hardware. Derive the expectations instead.
    const N_TIME: usize = 372;

    #[test]
    fn coarse_sync_accepts_wsjtx_lag_window_without_out_of_bounds_access() {
        // Before the trailing-block guard was restored, requesting
        // WSJT-X's +/-2.5 s window made coarse_sync index beyond the
        // final row.
        let n_freq = 128;
        let spec =
            Spectrogram::from_parts(n_freq, N_TIME, vec![Default::default(); n_freq * N_TIME]);

        let candidates = coarse_sync_inner(
            &spec,
            200.0,
            300.0,
            1.5,
            10,
            None,
            Some(2.5),
            PartialBlock2::Score,
        );

        assert!(candidates.is_empty());
    }

    /// The score loop builds each carrier's allsum row itself when the caller passes none, and reads a
    /// caller's full table when it does (`stage1_inc`, `precompute_coarse_allsum`). The two must give the
    /// same candidates to the bit: the lazy row replaced a full `n_freq x n_time` table in the one path
    /// (fixed-point: its sliding window is an integer identity of the direct sum, f32: it always was one).
    #[test]
    fn lazy_allsum_equals_the_full_table() {
        use super::super::spectrogram::SpecCell;
        use super::super::types::NSTEP;
        use super::{coarse_sync_with_allsum, precompute_coarse_allsum};

        let n_time = crate::ft8::params::NMAX / NSTEP - 3;
        let n_freq = 977; // 0..3000 Hz + the top tone's room, as `compute_spectrogram(_, 3000)`
        let mut x: u32 = 0x1234_5678;
        let mut next = || {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (x >> 16) as u16
        };
        let data: Vec<SpecCell> = (0..n_freq * n_time)
            .map(|i| {
                // noise, plus a ridge at 1500 Hz so there are real peaks to rank
                let ridge = if (480..482).contains(&(i % n_freq)) {
                    4000
                } else {
                    0
                };
                #[allow(clippy::unnecessary_cast)]
                {
                    ((next() >> 4) + ridge) as SpecCell
                }
            })
            .collect();
        let spec = Spectrogram::from_parts(n_freq, n_time, data);

        let lazy = super::coarse_sync(&spec, 200.0, 3000.0, 0.0, 300);
        let full = precompute_coarse_allsum(&spec, 200.0, 3000.0);
        let table = coarse_sync_with_allsum(&spec, 200.0, 3000.0, 0.0, 300, &full);
        assert!(
            lazy.len() > 50,
            "only {} candidates: the test would prove little",
            lazy.len()
        );
        assert_eq!(lazy.len(), table.len());
        for (a, b) in lazy.iter().zip(&table) {
            assert_eq!(a.freq_hz.to_bits(), b.freq_hz.to_bits());
            assert_eq!(a.dt_sec.to_bits(), b.dt_sec.to_bits());
            assert_eq!(a.score.to_bits(), b.score.to_bits());
        }
    }

    #[test]
    fn sync_lag_preserves_wsjtx_window_when_middle_block_fits() {
        // The bound is how far the middle Costas block can slide before
        // it runs off either end of the spectrogram.
        let base = 13 + NSSY * COSTAS_POS[1] as i32;
        let end = base + NSSY * (COSTAS.len() as i32 - 1);
        let safe = base.min((N_TIME as i32 - 1) - end);

        // A request inside the bound is honoured as asked.
        assert_eq!(bounded_sync_lag_steps(N_TIME, 13, safe - 1), Some(safe - 1));
        // One past it is clamped to the bound, not refused.
        assert_eq!(bounded_sync_lag_steps(N_TIME, 13, safe + 1_000), Some(safe));
        // A negative request is floored at zero, not treated as huge.
        assert_eq!(bounded_sync_lag_steps(N_TIME, 13, -10), Some(0));
        // A spectrogram too short for the middle block at all: no
        // window exists, so the search must be refused outright.
        let too_short = (end as usize).min(N_TIME);
        assert_eq!(bounded_sync_lag_steps(too_short, 13, 25), None);
    }

    #[test]
    fn trailing_block_skips_symbols_beyond_spectrogram() {
        let last = N_TIME as i32 - 1;
        let base = 301;
        // Room for the whole block — saturates at the Costas length on
        // either grid.
        assert_eq!(
            valid_trailing_symbol_count(base, 0, N_TIME),
            COSTAS.len(),
            "a block with room to spare keeps all its symbols"
        );
        // Room for part of it: one symbol per `NSSY` rows that remain,
        // plus the one at the base row itself.
        let lag = last - base - NSSY;
        assert_eq!(
            valid_trailing_symbol_count(base, lag, N_TIME),
            (NSSY / NSSY + 1) as usize,
            "a partly-truncated block keeps what fits"
        );
        // Starting past the end: nothing is readable.
        assert_eq!(
            valid_trailing_symbol_count(base, last - base + 1, N_TIME),
            0
        );
    }
}
