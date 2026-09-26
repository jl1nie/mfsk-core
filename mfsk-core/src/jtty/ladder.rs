//! The JTTY decode ladder: from correlations to an accepted payload.
//!
//! Ported from WSJT-X `lib/jtty/jtty_tbcc_decoder.f90` (`decode_ladder`,
//! `decode_rung`), tag `v3.2.0-rc1`.
//!
//! Four rungs are tried in order and the **first that yields an accepted word
//! wins**: the list decoder ([`super::trellis`]) at coherent length 1, 2 and 4 on
//! the full-symbol correlations, then length 1 on the half-symbol energies
//! (which discard phase, so only length 1 means anything there). A rung accepts
//! the first of its four best words whose CRC-12 checks out — except that an
//! all-zero payload ends the scan of that rung instead: it is the one word a
//! zero-filled or dead input produces, and stepping past it would widen the
//! false-accept budget. (The reserved bit is already forced to 0 inside the
//! trellis.) Source-grammar validity is checked by the caller, on the payload.
//!
//! ## Parallel by construction
//!
//! With the `parallel` feature the four rungs are evaluated concurrently and the
//! first accepting one *in rung order* is taken, so the result is exactly the
//! sequential one. This is speculative — a strong signal that L = 1 accepts
//! still pays for the others, on other cores — but candidates that decode
//! nothing, the great majority, run every rung anyway and finish in the time of
//! the slowest instead of the sum.

use super::trellis::{Correlations, ListResult, Plan};
use super::{PAYLOAD_BITS, Payload};

/// Coherent lengths of the three full-symbol rungs.
pub const COHERENT_LENGTHS: [usize; 3] = [1, 2, 4];

/// A payload some rung accepted, and how.
#[derive(Clone, Debug, PartialEq)]
pub struct Accepted {
    /// The 34-bit payload (source word, reserved bit, EOM).
    pub payload: Payload,
    /// 1‥3 for the full-symbol rungs in order, 4 for the half-symbol one.
    pub rung: usize,
    /// Coherent length used (1 for the half-symbol rung).
    pub coherent: usize,
    /// 1-based rank of the accepted word in its rung's list.
    pub rank: usize,
    /// Accepted on the half-symbol energies.
    pub half_symbol: bool,
    /// Distinct closed words the rung found.
    pub pool: usize,
    /// The accepted word's single-pass metric.
    pub metric: f64,
}

/// The trellis plans for every rung; immutable, so one instance serves any
/// number of threads.
pub struct Ladder {
    plans: [Plan; 3],
    /// Carry the path metrics in `f32` (see [`Plan::decode_f32`]).
    f32_metrics: bool,
    /// Rungs run, for `jtty-stats`.
    #[cfg(feature = "jtty-stats")]
    rungs: [core::sync::atomic::AtomicU64; 4],
}

impl Default for Ladder {
    fn default() -> Self {
        Self::new()
    }
}

impl Ladder {
    /// Build the plans (a few milliseconds; do it once).
    pub fn new() -> Self {
        Self {
            plans: COHERENT_LENGTHS.map(Plan::new),
            f32_metrics: false,
            #[cfg(feature = "jtty-stats")]
            rungs: Default::default(),
        }
    }

    /// Rungs run so far: L=1, L=2, L=4, half-symbol L=1 (`jtty-stats`).
    #[cfg(feature = "jtty-stats")]
    pub fn rung_counts(&self) -> [u64; 4] {
        core::array::from_fn(|i| self.rungs[i].load(core::sync::atomic::Ordering::Relaxed))
    }

    /// Zero the rung counts (`jtty-stats`).
    #[cfg(feature = "jtty-stats")]
    pub fn reset_rung_counts(&self) {
        self.rungs
            .iter()
            .for_each(|r| r.store(0, core::sync::atomic::Ordering::Relaxed));
    }

    /// The same ladder with the trellis metrics in `f32`: hardware on a core whose `f64` is
    /// software (`docs/notes/JTTY_UPSTREAM.md`, "E0 results"), and the choice measured
    /// against `f64` by `tests/jtty_f32_metrics.rs`.
    pub fn with_f32_metrics(mut self) -> Self {
        self.f32_metrics = true;
        self
    }

    /// One rung: the list decode and the acceptance rule.
    fn rung(&self, index: usize, zsym: &Correlations, zhalf: &Correlations) -> Option<Accepted> {
        #[cfg(feature = "jtty-stats")]
        self.rungs[index].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        let half = index == 3;
        let plan = &self.plans[if half { 0 } else { index }];
        let z = if half { zhalf } else { zsym };
        let list = if self.f32_metrics {
            plan.decode_f32(z, true)
        } else {
            plan.decode(z, true)
        };
        accept(&list).map(|(rank, hyp, pool)| Accepted {
            payload: core::array::from_fn::<u8, PAYLOAD_BITS, _>(|i| hyp.bits[i]),
            rung: index + 1,
            coherent: plan.coherent(),
            rank,
            half_symbol: half,
            pool,
            metric: hyp.clean_metric,
        })
    }

    /// Run the ladder on the full-symbol correlations `zsym` and the half-symbol
    /// energies `zhalf` (real values in a `Correlations`).
    pub fn decode(&self, zsym: &Correlations, zhalf: &Correlations) -> Option<Accepted> {
        #[cfg(feature = "parallel")]
        {
            let ((a, b), (c, d)) = rayon::join(
                || rayon::join(|| self.rung(0, zsym, zhalf), || self.rung(1, zsym, zhalf)),
                || rayon::join(|| self.rung(2, zsym, zhalf), || self.rung(3, zsym, zhalf)),
            );
            a.or(b).or(c).or(d)
        }
        #[cfg(not(feature = "parallel"))]
        {
            (0..4).find_map(|i| self.rung(i, zsym, zhalf))
        }
    }
}

/// The first CRC-valid word of a list, unless an all-zero payload comes first.
fn accept(list: &ListResult) -> Option<(usize, &super::trellis::Hypothesis, usize)> {
    for (i, h) in list.hypotheses.iter().enumerate() {
        if !h.crc_valid {
            continue;
        }
        if h.bits[..PAYLOAD_BITS].iter().all(|&b| b == 0) {
            return None;
        }
        return Some((i + 1, h, list.pool));
    }
    None
}
