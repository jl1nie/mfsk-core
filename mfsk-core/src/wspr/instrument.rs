//! Free-running counters for the WSPR candidate loop.
//!
//! Written for `docs/notes/WSPR_EMBEDDED_MEASUREMENT_PLAN.md` Phase 1,
//! whose highest-value single number is the `minsync1` pass rate: the
//! refine cascade in
//! [`decode::decode_at_baseband_nblocks_gated_drift`](super::decode::decode_at_baseband_nblocks_gated_drift)
//! runs 1 + 4 + 4 evaluations, then **8 more** only when
//! `best_sync > MINSYNC1`, and each evaluation reads
//! 162 × 256 × 2 × 4 B = 324 KiB of baseband. Whether a candidate costs
//! ~2.9 MiB or ~6.0 MiB is decided entirely by that gate, and it can't
//! be inferred from the outside.
//!
//! These are plain relaxed [`AtomicU32`]s, always compiled in. One
//! relaxed add sits next to a 324 KiB traversal
//! ([`TONE_AMPLITUDES`]) or a Fano/OSD decode — the counters are free
//! at the resolution anything here is measured at, so there is no
//! feature gate to get wrong on the embedded side, where `cfg`
//! mismatches between host and device have cost this repo half-days
//! before.
//!
//! Nothing resets them implicitly. A caller measuring one stage brackets
//! it with [`snapshot`] and subtracts, or calls [`reset`] first.

use core::sync::atomic::{AtomicU32, Ordering};

/// Calls to [`demod::tone_amplitudes`](super::demod::tone_amplitudes) —
/// the 324 KiB traversal, and the unit the whole traffic model is
/// denominated in.
pub static TONE_AMPLITUDES: AtomicU32 = AtomicU32::new(0);

/// Entries to the refine cascade, i.e. candidates actually demodulated.
pub static CANDIDATES: AtomicU32 = AtomicU32::new(0);

/// Candidates whose post-drift-refine `best_sync` cleared `MINSYNC1`
/// and so paid for refine stages 4 and 5.
pub static MINSYNC1_PASS: AtomicU32 = AtomicU32::new(0);

/// Candidates whose fully-refined `best_sync` failed to clear
/// `minsync2` and so never reached Fano/OSD — wsprd's own
/// candidate-list filter (`wsprd.c:1294`).
pub static MINSYNC2_REJECTED: AtomicU32 = AtomicU32::new(0);

/// Fano attempts that survived the `minrms` plausibility gate.
pub static FANO_ATTEMPTS: AtomicU32 = AtomicU32::new(0);

/// Fano attempts that converged.
pub static FANO_OK: AtomicU32 = AtomicU32::new(0);

/// OSD attempts. Reached only when Fano failed *and* a callsign table
/// was supplied, so this is zero for the whole of passes 0 and 1.
pub static OSD_ATTEMPTS: AtomicU32 = AtomicU32::new(0);

/// OSD attempts that both decoded and cleared the callsign-table gate.
pub static OSD_OK: AtomicU32 = AtomicU32::new(0);

/// Microseconds inside `coarse_baseband`'s `build_spectro` — the
/// ~359 windowed 512-point FFTs that fill `ps`.
///
/// Paired with [`COARSE_REFINE_US`] to answer a question the existing
/// Phase-2 bandwidth arm did not: that arm measured `tone_amplitudes`
/// in the *decode* stage and concluded the loop is compute-bound
/// (internal-SRAM placement bought 5 %). `coarse` is a separate 28 % of
/// the scan, and `refine_alignment_top_k` there reads the 735 KB
/// PSRAM-resident `ps` on the order of 10^8 times in a 7-float window
/// per innermost step. Whether that stage is latency-bound the way
/// FT8's `cs Box` was (5-10x from staging into internal DRAM,
/// `embedded-shared/src/internal_pool.rs`) has never been measured;
/// splitting coarse in two is the first thing that has to be known.
///
/// `u32` micros — Xtensa has no 64-bit atomics, and 2^32 us is 71
/// minutes against a stage that runs ~30 s. Only accumulated on `std`
/// builds (`Instant` is
/// the only clock `mfsk-core` can portably reach; the embedded WSPR
/// bench enables `mfsk-core/std`). Zero elsewhere.
pub static COARSE_SPECTRO_US: AtomicU32 = AtomicU32::new(0);

/// Microseconds inside `refine_alignment_top_k`, summed over peaks.
/// See [`COARSE_SPECTRO_US`].
pub static COARSE_REFINE_US: AtomicU32 = AtomicU32::new(0);

/// Add `d` to one of the coarse timers. No-op without `std`.
#[inline]
pub fn add_us(counter: &AtomicU32, us: u32) {
    counter.fetch_add(us, Ordering::Relaxed);
}

/// Times a caller-supplied ladder budget (see
/// `wspr::decode::decode_pass2_top_n`'s own doc comment, behind
/// feature `wspr-pass2-topn`) cut the DT peak-up × nblocks sweep
/// short instead of letting it run to exhaustion. Zero for every
/// existing caller — they all pass `None` — so a nonzero count here
/// is exactly the
/// signal that time-budget control fired on this run.
pub static LADDER_BUDGET_ABORTED: AtomicU32 = AtomicU32::new(0);

const ALL: &[&AtomicU32] = &[
    &TONE_AMPLITUDES,
    &CANDIDATES,
    &MINSYNC1_PASS,
    &MINSYNC2_REJECTED,
    &FANO_ATTEMPTS,
    &FANO_OK,
    &OSD_ATTEMPTS,
    &OSD_OK,
    &LADDER_BUDGET_ABORTED,
];

/// One reading of every counter.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    pub coarse_spectro_us: u32,
    pub coarse_refine_us: u32,
    pub tone_amplitudes: u32,
    pub candidates: u32,
    pub minsync1_pass: u32,
    pub minsync2_rejected: u32,
    pub fano_attempts: u32,
    pub fano_ok: u32,
    pub osd_attempts: u32,
    pub osd_ok: u32,
    pub ladder_budget_aborted: u32,
}

impl Counts {
    /// `self - earlier`, for bracketing a stage. Saturating, so a
    /// counter that wrapped between the two readings reports 0 rather
    /// than a nonsense large delta.
    #[must_use]
    pub fn since(self, earlier: Counts) -> Counts {
        Counts {
            coarse_spectro_us: self
                .coarse_spectro_us
                .saturating_sub(earlier.coarse_spectro_us),
            coarse_refine_us: self
                .coarse_refine_us
                .saturating_sub(earlier.coarse_refine_us),
            tone_amplitudes: self.tone_amplitudes.saturating_sub(earlier.tone_amplitudes),
            candidates: self.candidates.saturating_sub(earlier.candidates),
            minsync1_pass: self.minsync1_pass.saturating_sub(earlier.minsync1_pass),
            minsync2_rejected: self
                .minsync2_rejected
                .saturating_sub(earlier.minsync2_rejected),
            fano_attempts: self.fano_attempts.saturating_sub(earlier.fano_attempts),
            fano_ok: self.fano_ok.saturating_sub(earlier.fano_ok),
            osd_attempts: self.osd_attempts.saturating_sub(earlier.osd_attempts),
            osd_ok: self.osd_ok.saturating_sub(earlier.osd_ok),
            ladder_budget_aborted: self
                .ladder_budget_aborted
                .saturating_sub(earlier.ladder_budget_aborted),
        }
    }

    /// Bytes of baseband read by [`Counts::tone_amplitudes`] calls:
    /// 162 symbols × 256 samples × 2 (I and Q) × 4 B.
    #[must_use]
    pub fn baseband_bytes_read(self) -> u64 {
        const PER_CALL: u64 =
            (super::demod::N_SYMBOLS * super::demod::NSPS_BASEBAND * 2 * 4) as u64;
        u64::from(self.tone_amplitudes) * PER_CALL
    }
}

/// Read every counter.
#[must_use]
pub fn snapshot() -> Counts {
    Counts {
        coarse_spectro_us: COARSE_SPECTRO_US.load(Ordering::Relaxed),
        coarse_refine_us: COARSE_REFINE_US.load(Ordering::Relaxed),
        tone_amplitudes: TONE_AMPLITUDES.load(Ordering::Relaxed),
        candidates: CANDIDATES.load(Ordering::Relaxed),
        minsync1_pass: MINSYNC1_PASS.load(Ordering::Relaxed),
        minsync2_rejected: MINSYNC2_REJECTED.load(Ordering::Relaxed),
        fano_attempts: FANO_ATTEMPTS.load(Ordering::Relaxed),
        fano_ok: FANO_OK.load(Ordering::Relaxed),
        osd_attempts: OSD_ATTEMPTS.load(Ordering::Relaxed),
        osd_ok: OSD_OK.load(Ordering::Relaxed),
        ladder_budget_aborted: LADDER_BUDGET_ABORTED.load(Ordering::Relaxed),
    }
}

/// Zero every counter.
pub fn reset() {
    for c in ALL {
        c.store(0, Ordering::Relaxed);
    }
    COARSE_SPECTRO_US.store(0, Ordering::Relaxed);
    COARSE_REFINE_US.store(0, Ordering::Relaxed);
}

/// Bump a counter by one. Relaxed: these are diagnostics, and no
/// reader orders anything against them.
#[inline]
pub(crate) fn bump(c: &AtomicU32) {
    c.fetch_add(1, Ordering::Relaxed);
}
