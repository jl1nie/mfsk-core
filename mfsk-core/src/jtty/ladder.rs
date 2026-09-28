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
//! With the `parallel` feature the rungs run in two lanes, L = 1 then the half-symbol
//! rung, and L = 2 then L = 4, and the first accepting one *in rung order* is taken, so
//! the result is exactly the sequential one. A lane's second rung is skipped when an
//! earlier rung in order has already accepted; candidates that decode nothing, the great
//! majority, finish in the time of the slower lane instead of the sum of four.

use super::scratch::Slot;
use super::trellis::{Correlations, ListResult, Plan, TrellisScratch};

use super::{PAYLOAD_BITS, Payload};

/// Coherent lengths of the three full-symbol rungs.
pub const COHERENT_LENGTHS: [usize; 3] = [1, 2, 4];

/// Which rungs of the ladder to try (a receiver setting, `rx::Params::ladder_rungs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rungs {
    /// Coherent length 1, 2 and 4 on the full-symbol correlations.
    pub l1: bool,
    /// Coherent length 2.
    pub l2: bool,
    /// Coherent length 4.
    pub l4: bool,
    /// Length 1 on the half-symbol energies.
    pub half: bool,
}

impl Rungs {
    /// All four, as upstream.
    pub const ALL: Self = Self {
        l1: true,
        l2: true,
        l4: true,
        half: true,
    };
    /// The three full-symbol rungs. On 860 simulated files — AWGN at 1500 Hz and off the bin
    /// grid, ITU LM, MD and LD fading — the half-symbol rung accepted no frame the others missed
    /// and made all four unexpected decodes of the embedded receiver; on the CoreS3 it is 140 ms
    /// of every candidate that fails (#499).
    pub const FULL_SYMBOL: Self = Self {
        half: false,
        ..Self::ALL
    };

    /// L=1 then L=4: what an embedded receiver keeps when speed comes before the last frames.
    /// On the CoreS3 a candidate that fails every rung costs 341 ms, against 483 with L=2 and
    /// 623 with all four; jtty_sweep's AWGN and mid-moderate files decode the same (163 of 360),
    /// ITU LM / MD fading loses 2 + 2 of 240 frames and AWGN off the bin grid 1 of 140 (#499).
    pub const L1_L4: Self = Self {
        l1: true,
        l2: false,
        l4: true,
        half: false,
    };

    fn has(self, i: usize) -> bool {
        [self.l1, self.l2, self.l4, self.half][i]
    }
}

impl Default for Rungs {
    fn default() -> Self {
        Self::ALL
    }
}

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
    /// Survivor arrays made with the `f32` metrics and reused by every decode that finds them
    /// free (one at a time; a concurrent rung allocates its own).
    scratch: Option<Slot<TrellisScratch>>,
    /// Rungs run, for `jtty-stats`.
    #[cfg(feature = "jtty-stats")]
    rungs: [super::stats::Acc; 4],
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
            scratch: None,
            #[cfg(feature = "jtty-stats")]
            rungs: Default::default(),
        }
    }

    /// Rungs run so far: L=1, L=2, L=4, half-symbol L=1 (`jtty-stats`).
    #[cfg(feature = "jtty-stats")]
    pub fn rung_counts(&self) -> [u64; 4] {
        core::array::from_fn(|i| self.rungs[i].get())
    }

    /// Zero the rung counts (`jtty-stats`).
    #[cfg(feature = "jtty-stats")]
    pub fn reset_rung_counts(&self) {
        self.rungs.iter().for_each(super::stats::Acc::clear);
    }

    /// The same ladder with the trellis metrics in `f32`: hardware on a core whose `f64` is
    /// software (`docs/notes/JTTY_UPSTREAM.md`, "E0 results"), and the choice measured
    /// against `f64` by `tests/jtty_f32_metrics.rs`.
    ///
    /// The two survivor arrays (24 KB each) are allocated here, once, and reused by each decode: on
    /// the CoreS3 a receiver built while allocations prefer internal DRAM keeps them there,
    /// where a rung costs 190 ms against 410 ms in PSRAM (#499), and later decodes allocate
    /// nothing whatever the heap then holds.
    pub fn with_f32_metrics(self) -> Self {
        self.with_f32_scratch(TrellisScratch::new())
    }

    /// [`Self::with_f32_metrics`] around survivor arrays the caller already allocated — so
    /// that [`super::rx::Receiver::new_with_f32_metrics`] can take them before anything else.
    pub(crate) fn with_f32_scratch(mut self, scratch: TrellisScratch) -> Self {
        self.f32_metrics = true;
        self.scratch = Some(Slot::new(scratch));
        self
    }

    /// One rung: the list decode and the acceptance rule.
    fn rung(&self, index: usize, zsym: &Correlations, zhalf: &Correlations) -> Option<Accepted> {
        #[cfg(feature = "jtty-stats")]
        self.rungs[index].add(1);
        let half = index == 3;
        let plan = &self.plans[if half { 0 } else { index }];
        let z = if half { zhalf } else { zsym };
        let list = if self.f32_metrics {
            match self.scratch.as_ref().and_then(Slot::take) {
                Some(mut guard) => plan.decode_f32_in(z, true, &mut guard),
                None => plan.decode_f32(z, true),
            }
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
        self.decode_rungs(zsym, zhalf, Rungs::ALL)
    }

    /// [`Self::decode`] trying only the rungs in `rungs`, in ladder order.
    pub fn decode_rungs(
        &self,
        zsym: &Correlations,
        zhalf: &Correlations,
        rungs: Rungs,
    ) -> Option<Accepted> {
        #[cfg(feature = "parallel")]
        {
            // Two lanes, each two rungs in turn: L=1 then the half-symbol rung, L=2 then L=4
            // (about 190+190 and 145+265 ms on the CoreS3). A lane skips its second rung once a
            // rung earlier in ladder order has accepted, so a frame L=1 takes costs one rung's
            // time, and one no rung takes the slower lane's instead of all four (#499). The
            // result is still the first accepting rung in order.
            use core::sync::atomic::{AtomicBool, Ordering::SeqCst};
            let accepted: [AtomicBool; 4] = Default::default();
            let run = |i: usize| {
                if !rungs.has(i) {
                    return None;
                }
                let r = self.rung(i, zsym, zhalf);
                if r.is_some() {
                    accepted[i].store(true, SeqCst);
                }
                r
            };
            let ((a, d), (b, c)) = rayon::join(
                || {
                    let a = run(0);
                    // the half-symbol rung is last in order: skip it if L=1 or L=2 took the frame
                    let d = if a.is_none() && !accepted[1].load(SeqCst) {
                        run(3)
                    } else {
                        None
                    };
                    (a, d)
                },
                || {
                    let b = run(1);
                    let c = if b.is_none() && !accepted[0].load(SeqCst) {
                        run(2)
                    } else {
                        None
                    };
                    (b, c)
                },
            );
            a.or(b).or(c).or(d)
        }
        #[cfg(not(feature = "parallel"))]
        {
            (0..4)
                .filter(|&i| rungs.has(i))
                .find_map(|i| self.rung(i, zsym, zhalf))
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
