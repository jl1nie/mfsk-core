//! A cells-flow configuration as **data**: [`CellsOptions`] names every choice a profile makes (the candidate count, the variant `d` floor, the pre-BP gate, the OSD fallback's flavour and timing, the
//! SIC), [`CellsOptions::p0_window`] and [`CellsOptions::p0_late`] are the CoreS3's two (`docs/notes/PROFILES.md` section 10), and [`decode_block_cells_with`] runs one over a slot's audio.
//! Nothing here is a new code path: every choice maps onto an item that exists ([`CellsWorker`]'s `with_*`, [`decode_round_osd`](CellsWorker::decode_round_osd), [`sic_passes`], [`sic_passes_osd`]), and the
//! tests check each preset against the chain it stands for. A firmware that drives the worker itself (two cores, a producer filling the store a row at a time) builds the worker with
//! [`CellsWorker::with_options`] and runs the rounds as it does today.
//!
//! **The API is not frozen** (Issue #766, `PROFILES.md` D3: the owner has not agreed to defaults): the types are plain values so an application overrides a field, at the price of a field
//! or a variant being a breaking change. Combinations outside the two presets and [`CellsOptions::PLAIN`] (an inline OSD in the SIC, `CELLS_GATE` with an OSD, a `k` of 0) are **not covered
//! by a precision guard**; only `p0_window()` and `p0_late()` are (`hosttest/icap-app/tests/icap_profile.rs`).
//!
//! **Every option here but [`CellsOptions::PLAIN`] is a divergence from WSJT-X** (the cells flow's search and stage 3 above all; the gate, `OsdSeeds::AD` and the capped OSD each have their record in Issue #766).

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use crate::engine::scalar::Cmplx;
use crate::engine::source::{AudioSample, SliceSource};
use crate::ft8::decode::{DecodeDepth, DecodeResult};

use super::cells_flow::{CELLS_D_MIN_SCORE, CellTables, CellsWorker, Found, SnrBaseline};
use super::coarse_lib::coarse_sync_gated_cells;
use super::coarse_sync::GatedSearch;
use super::ft8dec_sic::{SicStorage, sic_passes, sic_passes_osd, sic_signal_capacity};
use super::pre_bp_gate::{BpGate, CELLS_GATE_OSD};
use super::process_candidates::Ft8BpScratch;
use super::types::{Ft8Rate, OsdSeeds};

/// The OSD fallback of a configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OsdOptions {
    /// No OSD: the BP staircase only.
    Off,
    /// The OSD at once for every attempt the staircase fails (`ft8b.f90`'s order; `seeds` chooses its runs).
    Inline { seeds: OsdSeeds },
    /// The OSD **deferred and capped**: the attempts that fail the staircase are queued, ranked ([`OsdRank`](super::OsdRank)) and the best `k` of the slot (of each round in the SIC) run it.
    Capped { seeds: OsdSeeds, k: usize },
}

/// The subtraction passes of a configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SicOptions {
    /// Passes in all (the first is the plain one; 2 is one subtraction pass after it).
    pub passes: usize,
}

/// A cells-flow configuration. See the module doc.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CellsOptions {
    /// Candidates decoded in a round (30 on the CoreS3).
    pub max_cand: usize,
    /// The variant `d` floor (`CELLS_D_MIN_SCORE`, 16; 0 runs `d` for every candidate).
    pub d_min_score: f32,
    /// The pre-BP gate (`CELLS_GATE` for OSD off, `CELLS_GATE_OSD` with an OSD), or none.
    pub gate: Option<&'static BpGate>,
    pub osd: OsdOptions,
    pub sic: Option<SicOptions>,
}

impl CellsOptions {
    /// What [`decode_block_cells`](super::decode_block_cells) does: 30 candidates, the `d` floor, no gate, no OSD, no SIC.
    pub const PLAIN: Self = Self {
        max_cand: 30,
        d_min_score: CELLS_D_MIN_SCORE,
        gate: None,
        osd: OsdOptions::Off,
        sic: None,
    };

    /// The CoreS3's **window path** (decodes before the reply deadline): 30 candidates, the `d` floor, `CELLS_GATE_OSD`, the OSD deferred and capped at 6 of the slot's attempts, seeds `a` and `d`.
    pub const fn p0_window() -> Self {
        Self {
            max_cand: 30,
            d_min_score: CELLS_D_MIN_SCORE,
            gate: Some(&CELLS_GATE_OSD),
            osd: OsdOptions::Capped {
                seeds: OsdSeeds::AD,
                k: 6,
            },
            sic: None,
        }
    }

    /// The CoreS3's **late path** (after the window): the same, with two SIC passes, the OSD capped at 6 in each round and its decodes subtracted for the next pass.
    pub const fn p0_late() -> Self {
        Self {
            sic: Some(SicOptions { passes: 2 }),
            ..Self::p0_window()
        }
    }

    /// The depth the staircase runs at: the board's (`DecodeDepth::EMBEDDED`), with the OSD on when the options have one.
    pub fn depth(&self) -> DecodeDepth {
        DecodeDepth {
            osd: !matches!(self.osd, OsdOptions::Off | OsdOptions::Capped { k: 0, .. }),
            ..DecodeDepth::EMBEDDED
        }
    }

    fn seeds(&self) -> OsdSeeds {
        match self.osd {
            OsdOptions::Off => OsdSeeds::Faithful,
            OsdOptions::Inline { seeds } | OsdOptions::Capped { seeds, .. } => seeds,
        }
    }
}

impl<'a> CellsWorker<'a> {
    /// This worker configured by `options`: the `d` floor, the gate, the OSD's seeds and (for [`OsdOptions::Capped`]) its deferral. The depth to run the rounds at is
    /// [`CellsOptions::depth`]; a capped OSD is run by [`decode_round_osd`](Self::decode_round_osd) / [`osd_pass`](Self::osd_pass) with `k`.
    pub fn with_options(self, options: &CellsOptions) -> Self {
        self.with_d_min_score(options.d_min_score)
            .with_gate(options.gate)
            .with_osd_seeds(options.seeds())
            .with_osd_deferred(matches!(options.osd, OsdOptions::Capped { .. }))
    }
}

/// The cells flow over one slot's audio **as `options` say**, at 8 kHz (`None` unless `rate` is `Hz8000`): the search of `search`, the rounds, the OSD and the SIC of the options, and the SNR of every decode
/// from the cells ([`SnrBaseline`]). The memory is made for the call (a firmware drives [`CellsWorker`] and [`sic_passes_osd`] itself over its own).
pub fn decode_block_cells_with<S: AudioSample>(
    audio: &[S],
    rate: Ft8Rate,
    search: &GatedSearch,
    options: &CellsOptions,
) -> Option<Vec<DecodeResult>> {
    let (cands, mut cells) = coarse_sync_gated_cells(&mut SliceSource(audio), rate, search, 1)?;
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> = vec![[Cmplx::<f32>::default(); 8]; 79]
        .try_into()
        .expect("79 x 8");
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(&mut cs, &mut bp, &tables).with_options(options);
    let depth = options.depth();
    let results = match options.sic {
        None => {
            let mut known: Vec<Found> = Vec::new();
            match options.osd {
                OsdOptions::Capped { k, .. } => {
                    worker.decode_round_osd(
                        &cells,
                        &cands,
                        options.max_cand,
                        depth,
                        super::cells_flow::RETRY_MIN_SHIFT,
                        &mut known,
                        k,
                    );
                }
                _ => {
                    worker.decode_round(
                        &cells,
                        &cands,
                        options.max_cand,
                        depth,
                        super::cells_flow::RETRY_MIN_SHIFT,
                        &mut known,
                    );
                }
            }
            let base = SnrBaseline::new(&cells, &tables);
            for f in &mut known {
                base.apply(&cells, &tables, f);
            }
            known.into_iter().map(|f| f.res).collect()
        }
        Some(sic) => {
            let mut storage =
                SicStorage::new(rate, sic_signal_capacity(options.max_cand, sic.passes));
            match options.osd {
                OsdOptions::Capped { k, .. } => sic_passes_osd(
                    &mut SliceSource(audio),
                    rate,
                    search,
                    options.max_cand,
                    depth,
                    sic.passes,
                    &cands,
                    &mut cells,
                    &tables,
                    &mut worker,
                    &mut storage.scratch(),
                    k,
                ),
                _ => sic_passes(
                    &mut SliceSource(audio),
                    rate,
                    search,
                    options.max_cand,
                    depth,
                    sic.passes,
                    &cands,
                    &mut cells,
                    &tables,
                    &mut worker,
                    &mut storage.scratch(),
                ),
            }
        }
    };
    Some(results)
}
