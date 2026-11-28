// SPDX-License-Identifier: GPL-3.0-only
//! The S3's FT8 flow with the symbol spectra read from the half-dB cells (`HalfDbCells`), not from a DFT of each candidate.
//!
//! **A deliberate divergence from WSJT-X** (`ft8b.f90` takes each candidate's 79 x 8 spectra from the downsampled complex baseband
//! at the refined place). Ported from `ft8dec.c` (icap-sdk, built on `ft8_lib`'s `decode.c`, MIT): a symbol's eight tone powers are read from the
//! waterfall at the candidate's grid cell, and for a candidate that does not decode there, a **second try at the place the parabola on the
//! 3 x 3 energy of the 21 Costas tones points to** (`ft8dec.c` 656-735: `parab`, a bilinear mix of four cells in power). The staircase after the
//! spectra (the LLRs, BP, OSD) is the existing one, `process_candidates_into_with_cs_scratch_tuned_with_fill`.
//!
//! Why: the per-candidate DFT is 234 of the 558 M instructions a slot of the S3 flow at 30 candidates, the fine sync 22 and the block-0
//! refine 23 (`docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md`, 15), and a phase is not needed (12). Measured (60 slots, 8 kHz, host
//! instructions, section 21): 382 against 549 M at 30 candidates and 165 against 295 M at 15, the same decodes of the union of `jt9`'s and
//! `ft8dec`'s lists to within 4 (18-21). The retry is limited to a shift of a quarter cell or more: smaller ones gain nothing (19).
//!
//! A cell is 0.5 dB of the **power**: the amplitude the spectra hold is `10^(h / 40)`, and the mix of the retry is in power (`10^(h / 20)`),
//! then its square root. A cell outside the slot or the store is 0 (no power), as in `ft8dec`.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::decode::{DecodeDepth, DecodeResult};
use super::super::params::{COSTAS, DEFAULT_BP_MAX_ITER};
use super::ap_context::ApContext;
use super::coarse_lib::{HalfDbCells, coarse_sync_gated_cells, merge_cells_parts, search_cells};
use super::coarse_sync::GatedSearch;
use super::fill_symbol_spectra::{SymMask, sym_in_mask};
use super::pre_bp_gate::BpGate;
use super::process_candidates::{Ft8BpScratch, process_candidate_with_scratch_ctx};
use super::types::{AudioSample, DEFAULT_Q_THRESH, Ft8Rate, SAMPLE_RATE_HZ};
use super::types::{GateCtx, PassCtx};
use super::types::{OsdMode, OsdSeeds};
use crate::engine::scalar::Cmplx;
use crate::engine::source::SliceSource;
use crate::engine::sync::SyncCandidate;

/// The retry is made only if the estimated shift of the place is at least this many cells (rows or bins) in one of the two directions.
/// Measured (60 slots, 30 candidates): under 0.15 the retry never decoded anything the first try had not, under 0.25 almost nothing (1
/// of 1 338 `jt9` decodes), and each skipped retry saves its LLRs, BP and OSD (451 -> 400 M instructions at 30 candidates in the experiment of
/// section 19; the landed flow is 382 M).
pub(super) const RETRY_MIN_SHIFT: f32 = 0.25;

/// The lowest search score (`ft8_lib`'s, 8..127, in 0.5 dB units) at which the cells flow runs the staircase's variant `d` (the second BP, on `llrd`) for a candidate: below it a
/// candidate gets `llra` only (at the grid and, if the retry applies, at the retry). Measured over 4 780 slots and the SNR ladders (Issue #765, design note section 27):
/// **-49 % of the BP passes** over the whole set (-24.5 % on the 60 real slots), **4 of 6 935 decodes lost** (1 of 975 real), no slot of the ladders lost. 21 loses 28, 0 (every candidate) is
/// the staircase as it was. **A deliberate divergence from WSJT-X** (`ft8b` tries all the variants), like the cells flow itself. [`CellsWorker::new`] keeps the staircase whole
/// ([`CellsWorker::with_d_min_score`] sets the floor); [`decode_block_cells`] and [`decode_block_cells_sic`] set this one.
pub const CELLS_D_MIN_SCORE: f32 = 16.0;
/// Largest half-dB cell: `20 log10 (2^32 - 1)` in 0.5 dB units of power (`spectrogram::half_db`).
const MAX_CELL: usize = 193;

/// The index of a cell that is not on the slot or the store: no power, as `ft8dec`'s `cellp` returns 0.
const OUTSIDE: usize = MAX_CELL + 1;

/// `10^(h / 40)` and `10^(h / 20)` for every cell value `h`, and 0 for [`OUTSIDE`].
/// The 194-entry amplitude and power tables of the cells (`10^(h / 40)`, `10^(h / 20)`, and 0 for a cell outside): made once, read by every fill. 1.5 KB; a firmware
/// keeps them in a `static` so that they are in internal DRAM.
pub struct CellTables {
    amp: [f32; OUTSIDE + 1],
    pow: [f32; OUTSIDE + 1],
}

impl Default for CellTables {
    fn default() -> Self {
        Self::new()
    }
}

impl CellTables {
    pub fn new() -> Self {
        let mut t = Self {
            amp: [0.0; OUTSIDE + 1],
            pow: [0.0; OUTSIDE + 1],
        };
        for h in 0..=MAX_CELL {
            t.amp[h] = (h as f32 * (core::f32::consts::LN_10 / 40.0)).exp();
            t.pow[h] = (h as f32 * (core::f32::consts::LN_10 / 20.0)).exp();
        }
        t
    }
}

/// Where a candidate sits on the cells: the row of its Costas symbol 0 and the bin of its tone 0.
/// Where a candidate sits on the cells: the row of its Costas symbol 0 and the bin (3.125 Hz) of its tone 0.
#[derive(Clone, Copy, Debug)]
pub struct Grid {
    pub j0: i32,
    pub k: i64,
}

/// A place off the grid: the neighbour row (`dr`, +-1) and bin (`dc`, +-1) and the weight of each neighbour (`wa`, `wb`, 0..0.5).
#[derive(Clone, Copy)]
struct Place {
    dr: i32,
    dc: i32,
    wa: f32,
    wb: f32,
}

impl Place {
    /// The frequency and DT of a decode found at this place, from those of the grid place: moved towards the neighbour the estimate points to by its weight
    /// of a bin (`df` Hz) and of a row (`tstep` s).
    fn moved(&self, freq_hz: f32, dt_sec: f32, df: f32, tstep: f32) -> (f32, f32) {
        (
            freq_hz + self.dc as f32 * self.wb * df,
            dt_sec + self.dr as f32 * self.wa * tstep,
        )
    }

    /// Whether the estimate is far enough from the grid (`min_shift` cells in rows or bins) for a second try to differ from the first.
    fn worth_a_retry(&self, min_shift: f32) -> bool {
        self.wa.max(self.wb) >= min_shift
    }
}

const ON_GRID: Place = Place {
    dr: 0,
    dc: 0,
    wa: 0.0,
    wb: 0.0,
};

pub(super) struct View<'a> {
    pub(super) cells: &'a HalfDbCells,
    pub(super) t: &'a CellTables,
}

impl View<'_> {
    /// The cell of tone `t` of symbol `sym` at the grid place shifted by (`dr` rows, `dc` bins): its value, or [`OUTSIDE`] off the slot or the store.
    #[inline]
    fn h(&self, g: Grid, sym: usize, t: usize, dr: i32, dc: i32) -> usize {
        let row = g.j0 + 2 * sym as i32 + dr;
        let bin = g.k + 2 * t as i64 + dc as i64;
        if row < 0 || bin < 0 {
            return OUTSIDE;
        }
        self.cells
            .cell(row as usize, bin as usize)
            .map_or(OUTSIDE, |c| c as usize)
    }

    /// `ft8dec`'s estimate of where the signal is, from the energy of the 21 Costas tones on the 3 x 3 cells around the grid place: a parabola
    /// through the three rows and through the three bins (`parab`), the weights of the four cells of the quadrant it points to.
    fn place(&self, g: Grid) -> Place {
        let mut e = [[0.0f32; 3]; 3];
        for (a, row) in e.iter_mut().enumerate() {
            for (b, cell) in row.iter_mut().enumerate() {
                let (dr, dc) = (a as i32 - 1, b as i32 - 1);
                let mut sum = 0.0;
                for m in 0..3 {
                    for (i, &t) in COSTAS.iter().enumerate() {
                        sum += self.t.pow[self.h(g, 36 * m + i, t, dr, dc)];
                    }
                }
                *cell = sum;
            }
        }
        let ta = parab(e[0][1], e[1][1], e[2][1]);
        let fb = parab(e[1][0], e[1][1], e[1][2]);
        Place {
            dr: if ta < 0.0 { -1 } else { 1 },
            dc: if fb < 0.0 { -1 } else { 1 },
            wa: ta.abs(),
            wb: fb.abs(),
        }
    }

    /// The place of a **decoded** signal on the cells (`ft8dec.c` 874-882): the energy of its 79 transmitted tones at the grid place and one row /
    /// one cell either side, a parabola through each (`parab`; the time one through `tcal`, whose map is measured for a Hann window of two symbols,
    /// whose energy-against-offset curve is not a parabola). Returns the frequency of tone 0 (Hz) and the sample at which symbol 0 starts (`nstep` samples a
    /// row, `df` Hz a cell): what the subtraction needs and what a decode reports.
    pub(super) fn refine(&self, g: Grid, tones: &[u8; 79], df: f32, nstep: i64) -> (f32, i64) {
        let energy = |dr: i32, dc: i32| -> f32 {
            tones
                .iter()
                .enumerate()
                .map(|(sym, &t)| self.t.pow[self.h(g, sym, t as usize, dr, dc)])
                .sum()
        };
        let e0 = energy(0, 0);
        let dtr = parab(energy(-1, 0), e0, energy(1, 0));
        let dfr = parab(energy(0, -1), e0, energy(0, 1));
        let t0 = g.j0 as f32 * nstep as f32 + tcal(dtr) * nstep as f32;
        let t0 = (t0 + if t0 >= 0.0 { 0.5 } else { -0.5 }) as i64;
        ((g.k as f32 + dfr) * df, t0)
    }

    /// The 79 x 8 spectra of the candidate at `g` moved by `p`: the amplitude of each cell, or the square root of the mix of the four cells'
    /// powers. Phase is not kept (`im` is 0): the metrics that read it are worth at most 3 hits of 1 338 (`FT8_WINDOW_AND_COARSE_DESIGN.md`, 12).
    fn fill_mask(&self, cs: &mut [[Cmplx<f32>; 8]; 79], g: Grid, p: Place, mask: SymMask) {
        let on_grid = p.wa == 0.0 && p.wb == 0.0;
        let (w00, w10, w01, w11) = (
            (1.0 - p.wa) * (1.0 - p.wb),
            p.wa * (1.0 - p.wb),
            (1.0 - p.wa) * p.wb,
            p.wa * p.wb,
        );
        for (sym, row) in cs.iter_mut().enumerate() {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            for (t, c) in row.iter_mut().enumerate() {
                let re = if on_grid {
                    self.t.amp[self.h(g, sym, t, 0, 0)]
                } else {
                    let pw = |dr, dc| self.t.pow[self.h(g, sym, t, dr, dc)];
                    (w00 * pw(0, 0) + w10 * pw(p.dr, 0) + w01 * pw(0, p.dc) + w11 * pw(p.dr, p.dc))
                        .sqrt()
                };
                *c = Cmplx { re, im: 0.0 };
            }
        }
    }

    /// All 79 symbols (the tests' form of [`fill_mask`](Self::fill_mask)).
    #[cfg(test)]
    fn fill(&self, cs: &mut [[Cmplx<f32>; 8]; 79], g: Grid, p: Place) {
        for mask in [
            SymMask::SyncBlock0,
            SymMask::SyncBlocks12,
            SymMask::DataOnly,
        ] {
            self.fill_mask(cs, g, p, mask);
        }
    }
}

/// The vertex of the parabola through the energies `em`, `e0`, `ep` of the neighbours -1, 0, +1, in -0.5..0.5 (`ft8dec.c`'s `parab`: a
/// neighbour better than the centre points half way to it).
fn parab(em: f32, e0: f32, ep: f32) -> f32 {
    let den = em - 2.0 * e0 + ep;
    if !(e0 >= em && e0 >= ep) {
        return if em > ep { -0.5 } else { 0.5 };
    }
    if den >= 0.0 {
        return 0.0;
    }
    (0.5 * (em - ep) / den).clamp(-0.5, 0.5)
}

/// `ft8dec.c`'s `tcal` (720-730): the map from the time parabola's vertex (`|d|`, rows) to the true offset, measured on synthetic signals in 5 ms
/// steps (the energy-against-offset curve of a Hann window of two symbols is not a parabola, and its vertex is biased).
fn tcal(d: f32) -> f32 {
    const TMAP: [f32; 9] = [0.0, 0.04, 0.08, 0.12, 0.165, 0.22, 0.29, 0.375, 0.49];
    let a = d.abs();
    let mut u = 0.5f32;
    for i in 1..9 {
        if a <= TMAP[i] {
            u = 0.0625 * ((i - 1) as f32 + (a - TMAP[i - 1]) / (TMAP[i] - TMAP[i - 1]));
            break;
        }
    }
    let u = u.min(0.5);
    if d < 0.0 { -u } else { u }
}

/// The slot's noise floor on its cells, for the SNR of its decodes: WSJT-X's `baseline.f90` (the lower envelope of the slot's average spectrum: a polynomial through the 10th percentile
/// of each of ten segments, `engine::baseline::fit_baseline`) run over the **cells' own time-averaged power**, so that the signal's power and the floor come from one spectrum (the
/// calibration of `ft8b.f90:451`'s `xsnr2` is fitted to a matching pair, `process_candidates.rs`). Made **once a slot from the complete cells of the first pass** (the subtraction
/// passes overwrite the store with a residual), about 456 columns read over 184 rows; applied to each decode with [`apply`](Self::apply).
///
/// **A deliberate divergence from WSJT-X** (`ft8b.f90:449-454`: `xsnr2 = xsig / xbase / 3e6 - 1`, dB - 27): the baseline is fitted over the half-dB cells, not `get_spectrum_baseline.f90`'s
/// Nuttall spectrum; the signal is the best of 3 x 3 cells; `SNR_K` and `SNR_REF_DB` are **empirical**, fitted on the 22 recordings of `tests/cells_flow_accuracy.rs` against `jt9`. The
/// accuracy quoted for it (mean 0, SD 2.1 dB) is therefore **in-sample**, measured with the OSD off; it is not a held-out figure.
pub struct SnrBaseline {
    first_bin: usize,
    /// dB (of the cells' power units) at each column.
    base_db: Vec<f32>,
}

/// `xsig / xbase` of `ft8b.f90:451` is `xsig / xbase / K - 1` here; `K` and the dB scale are **fitted** to `jt9`'s SNR on the 22 recordings (`tests/cells_flow_accuracy.rs`).
const SNR_K: f32 = 58.0;
const SNR_REF_DB: f32 = 26.12;
use super::process_candidates::FT8_SNR_FLOOR_DB as SNR_FLOOR_DB;

impl SnrBaseline {
    /// The baseline of `cells` (call it on the complete store of a slot, before any subtraction pass).
    pub fn new(cells: &HalfDbCells, tables: &CellTables) -> Self {
        // a partial store gives another baseline than the slot's (the early emit must not use it), a residual store another again
        debug_assert!(
            cells.filled_rows() == cells.n_time(),
            "SnrBaseline::new wants the complete cells of the slot"
        );
        let n_freq = cells.row_len();
        let rows = cells.filled_rows();
        let mut avg = alloc::vec![0.0f32; n_freq];
        let first = cells.carrier_bins().start;
        for row in 0..rows {
            for (col, a) in avg.iter_mut().enumerate() {
                if let Some(c) = cells.cell(row, first + col) {
                    *a += tables.pow[c as usize];
                }
            }
        }
        if rows > 0 {
            for a in avg.iter_mut() {
                *a /= rows as f32;
            }
        }
        let base_db = crate::engine::baseline::fit_baseline(&avg, 0, n_freq.saturating_sub(1));
        Self {
            first_bin: first,
            base_db,
        }
    }

    /// `xsig / xbase`: the power of the 79 transmitted `tones` (the best of the 3 x 3 cells round the grid place `g`: a signal half a cell off loses up to 3 dB) over the baseline at the
    /// carrier. `None` where the baseline does not reach the carrier.
    pub fn ratio(
        &self,
        cells: &HalfDbCells,
        tables: &CellTables,
        g: Grid,
        tones: &[u8; 79],
    ) -> Option<f32> {
        let view = View { cells, t: tables };
        let col = usize::try_from(g.k).ok()?.checked_sub(self.first_bin)?;
        let base_db = *self.base_db.get(col)?;
        let energy = |dr: i32, dc: i32| -> f32 {
            tones
                .iter()
                .enumerate()
                .map(|(sym, &t)| tables.pow[view.h(g, sym, t as usize, dr, dc)])
                .sum()
        };
        let mut xsig = 0.0f32;
        for dr in -1..=1 {
            for dc in -1..=1 {
                xsig = xsig.max(energy(dr, dc));
            }
        }
        let xbase = 10.0f32.powf(base_db / 10.0);
        (xbase > 0.0 && xbase.is_finite()).then_some(xsig / xbase)
    }

    /// The SNR in WSJT-X's reference (dB in 2 500 Hz) of a decode at grid place `g`.
    pub fn snr_db(
        &self,
        cells: &HalfDbCells,
        tables: &CellTables,
        g: Grid,
        tones: &[u8; 79],
    ) -> f32 {
        match self.ratio(cells, tables, g, tones) {
            Some(r) if r / SNR_K > 1.0 => {
                (10.0 * (r / SNR_K - 1.0).log10() - SNR_REF_DB).max(SNR_FLOOR_DB)
            }
            _ => SNR_FLOOR_DB,
        }
    }

    /// Replace `found.res.snr_db` (the staircase's adjacent-tone figure, 0-15 dB off between signals) by [`snr_db`](Self::snr_db).
    pub fn apply(&self, cells: &HalfDbCells, tables: &CellTables, found: &mut Found) {
        let tones: [u8; 79] =
            crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(found.res.message77())
                .try_into()
                .expect("FT8 has 79 symbols");
        found.res.snr_db = self.snr_db(cells, tables, found.grid, &tones);
    }
}

/// A worker of the cells flow over **memory the caller owns**: the staging block of 79 x 8 spectra (internal DRAM on a board: its existing staging pair), the BP scratch
/// (about 12 KB, made once per worker) and the tables. [`decode_candidate`](Self::decode_candidate) allocates nothing for a candidate that does not decode (OSD off: the OSD
/// fallback makes a scratch of its own), so a firmware runs one worker a core and pulls candidates from a queue. `decode_block_cells` and `decode_block_cells_sic` make one
/// worker per call.
pub struct CellsWorker<'a> {
    cs: &'a mut [[Cmplx<f32>; 8]; 79],
    bp: &'a mut Ft8BpScratch,
    tables: &'a CellTables,
    d_min_score: f32,
    gate: Option<&'static BpGate>,
    osd_seeds: OsdSeeds,
    /// `OsdMode::Defer`: an attempt that reaches the OSD fallback is queued in `pending`, not run.
    osd_deferred: bool,
    pending: Vec<OsdAttempt>,
    /// Attempts queued and attempts run by [`osd_pass`](CellsWorker::osd_pass) since [`take_osd_counts`](CellsWorker::take_osd_counts).
    osd_queued: u64,
    osd_ran: u64,
    /// The operator's QSO: the AP rung runs on the candidates near the partner ([`with_ap_context`](CellsWorker::with_ap_context)).
    ap_ctx: Option<ApContext>,
}

/// The half width of the band [`CellsWorker::decode_partner`] searches round the partner's frequency, Hz (8 bins of 3.125 Hz), and how many candidates it keeps.
const PARTNER_HALF_BAND_HZ: f32 = 25.0;
const PARTNER_CANDIDATES: usize = 4;
/// The search keeps this many before the DT window is applied (the window drops the ones that sit far from where the partner was).
const PARTNER_POOL: usize = 12;

/// An attempt queued for the OSD fallback ([`CellsWorker::take_osd_attempts`]): the candidate, the place of the attempt and its `OsdRank` score. Opaque; it can be moved to
/// another worker (another core) and run there by [`CellsWorker::run_osd_attempt`] over the same cells.
#[derive(Clone)]
pub struct OsdAttempt {
    cand: SyncCandidate,
    hs: u8,
    g: Grid,
    /// `None`: the grid place.
    p: Option<Place>,
    logit: f32,
}

impl OsdAttempt {
    /// The attempt's `OsdRank` score: the higher, the likelier that the OSD decodes it.
    pub fn logit(&self) -> f32 {
        self.logit
    }

    /// The order the OSD pass takes the attempts in: best score first; equal scores by the candidate's frequency, DT and place, so that the order does not depend on which
    /// worker queued an attempt.
    fn order(a: &Self, b: &Self) -> core::cmp::Ordering {
        b.logit
            .partial_cmp(&a.logit)
            .unwrap_or(core::cmp::Ordering::Equal)
            .then(
                a.cand
                    .freq_hz
                    .partial_cmp(&b.cand.freq_hz)
                    .unwrap_or(core::cmp::Ordering::Equal),
            )
            .then(
                a.cand
                    .dt_sec
                    .partial_cmp(&b.cand.dt_sec)
                    .unwrap_or(core::cmp::Ordering::Equal),
            )
            .then(a.p.is_some().cmp(&b.p.is_some()))
    }
}

/// The best `k` of `attempts` by score, best first (the rest dropped): what [`CellsWorker::osd_pass`] runs. A firmware with two cores gathers the attempts of both
/// workers, ranks them here and deals the first `k` out between the workers.
pub fn rank_osd_attempts(attempts: &mut Vec<OsdAttempt>, k: usize) {
    attempts.sort_by(OsdAttempt::order);
    attempts.truncate(k);
}

impl<'a> CellsWorker<'a> {
    /// A worker over the caller's staging block, BP scratch and tables.
    pub fn new(
        cs: &'a mut [[Cmplx<f32>; 8]; 79],
        bp: &'a mut Ft8BpScratch,
        tables: &'a CellTables,
    ) -> Self {
        Self {
            cs,
            bp,
            tables,
            d_min_score: 0.0,
            gate: None,
            osd_seeds: OsdSeeds::Faithful,
            osd_deferred: false,
            pending: Vec::new(),
            osd_queued: 0,
            osd_ran: 0,
            ap_ctx: None,
        }
    }

    /// This worker with the gate before BP ([`BpGate`], [`CELLS_GATE`](super::CELLS_GATE) is the fitted one): a candidate the model gives no chance is not decoded, at the grid
    /// or at the retry. Needs the candidate's hard-sync count, so it acts through [`decode_round`](Self::decode_round) and
    /// [`decode_candidate_hs`](Self::decode_candidate_hs), not [`decode_candidate`](Self::decode_candidate). **The gate must be the one fitted for the flow's depth** ([`CELLS_GATE`](super::CELLS_GATE) is for OSD off); the depth does not switch it off.
    pub fn with_gate(mut self, gate: Option<&'static BpGate>) -> Self {
        self.gate = gate;
        self
    }

    /// This worker with the OSD fallback's runs chosen ([`OsdSeeds`]; only matters when the depth has `osd`). The default is `ft8b.f90`'s ten runs.
    pub fn with_osd_seeds(mut self, osd_seeds: OsdSeeds) -> Self {
        self.osd_seeds = osd_seeds;
        self
    }

    /// This worker with the operator's QSO ([`ApContext`]) set, or cleared with `None` (the default: no AP, every decode as before). With one, a candidate within
    /// [`ApContext::WINDOW_HZ`] of the partner that the BP staircase and the OSD did not decode gets the context's AP passes (`ft8b.f90` `ipass` 5..11: the bits of the
    /// message the partner is expected to send locked, then the masked BP and OSD). **A divergence from WSJT-X**, off unless set.
    pub fn with_ap_context(mut self, ap_ctx: Option<ApContext>) -> Self {
        self.ap_ctx = ap_ctx;
        self
    }

    /// [`with_ap_context`](Self::with_ap_context) on a worker that is kept (the application refreshes the context between slots).
    pub fn set_ap_context(&mut self, ap_ctx: Option<ApContext>) {
        self.ap_ctx = ap_ctx;
    }

    /// The partner's own candidates: with a context set, a search of `+-25 Hz` round the partner's frequency over the store (`PARTNER_CANDIDATES` best, the search's own score, **no
    /// minimum**: the partner may be below what the slot's search keeps, section 23 of `docs/notes/PI3_JT9_GAP.md`: in a busy band knowing where it is is worth about 1 dB and the AP
    /// another 1.5-2), each decoded with the AP rung and **without the pre-BP gate** (fitted without AP). A decode already in `known` is not added again. Returns how many
    /// were added; 0 without a context or in a stage with no pass ([`ApStage::Calling`](super::ApStage::Calling)).
    pub fn decode_partner(
        &mut self,
        cells: &HalfDbCells,
        depth: DecodeDepth,
        min_shift: f32,
        known: &mut Vec<Found>,
    ) -> usize {
        let Some(ctx) = self.ap_ctx else { return 0 };
        if ctx.passes().is_empty() {
            return 0;
        }
        let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
        let f = ctx.freq_hz();
        let search = GatedSearch::new(
            f - PARTNER_HALF_BAND_HZ,
            f + PARTNER_HALF_BAND_HZ,
            0.0,
            PARTNER_POOL,
        );
        let lo = (search.freq_min / df).round().max(0.0) as usize;
        let hi = (search.freq_max / df).round() as usize;
        let part = search_cells(cells, &search, lo..hi + 1);
        let mut cands = merge_cells_parts(&[part], &search, cells.rate());
        cands.retain(|(c, _)| (c.dt_sec - ctx.dt_sec()).abs() <= ApContext::WINDOW_DT_S);
        cands.truncate(PARTNER_CANDIDATES);
        let gate = self.gate.take();
        let before = known.len();
        for (cand, hs) in &cands {
            if let Some(f) = self.decode_candidate_hs(cells, cand, *hs, depth, min_shift)
                && !known.iter().any(|o| o.res.message77() == f.res.message77())
            {
                known.push(f);
            }
        }
        self.gate = gate;
        known.len() - before
    }

    /// This worker with the OSD fallback **deferred** ([`OsdMode::Defer`]): an attempt that reaches it (the BP staircase failed, `depth.osd`) is queued with its
    /// [`OsdRank`](super::OsdRank) score instead of run, and [`osd_pass`](Self::osd_pass), called after the slot's BP work, runs the best `k` of them. Without it
    /// (the default) the OSD runs at once, as `ft8b.f90` does. A divergence from WSJT-X (the OSD decodes that the cap drops, and the order of the decodes).
    pub fn with_osd_deferred(mut self, deferred: bool) -> Self {
        self.osd_deferred = deferred;
        self
    }

    /// The attempts [`osd_pass`](Self::osd_pass) has been given and has run since the last call (or since the worker was made), and start counting again: what the cap
    /// kept out of the OSD is the difference.
    pub fn take_osd_counts(&mut self) -> (u64, u64) {
        (
            core::mem::take(&mut self.osd_queued),
            core::mem::take(&mut self.osd_ran),
        )
    }

    /// Make the OSD fallback's memory now (about 17 KB: the OSD's own scratch and the f32 BP scratch of its inputs; the fallback otherwise makes it on the first candidate
    /// that reaches it and keeps it). A firmware that wants it in a particular heap (internal DRAM) makes it at a point of its choosing, before the first decode.
    #[cfg(any(feature = "fft-rustfft", feature = "osd"))]
    pub fn reserve_osd_work(&mut self) {
        let _ = self.bp.osd_work();
    }

    /// How many attempts wait for [`osd_pass`](Self::osd_pass).
    pub fn osd_pending(&self) -> usize {
        self.pending.len()
    }

    /// This worker running the staircase's variant `d` only for a candidate whose search score ([`SyncCandidate::score`]) is `d_min_score` or more
    /// ([`CELLS_D_MIN_SCORE`] is the measured value; the default, 0, runs it for every candidate). Applies to the grid try and to the retry alike.
    pub fn with_d_min_score(mut self, d_min_score: f32) -> Self {
        self.d_min_score = d_min_score;
        self
    }

    /// The belief-propagation work this worker has run **on its own BP scratch** ([`BpStats`](crate::fec::ldpc::bp::BpStats): calls and message-update passes, which is
    /// what BP's time scales with) since it was made or since [`take_bp_stats`](Self::take_bp_stats). Not counted: the host build's blind-CQ AP pass (with `fft-rustfft`
    /// a candidate that does not decode goes on to it, on a scratch of its own; the boards' build has no such pass) and OSD; `decoded` counts BP calls accepted, not
    /// the decodes the worker returns (a duplicate or a validity gate drops some).
    pub fn bp_stats(&self) -> crate::fec::ldpc::bp::BpStats {
        self.bp.stats()
    }

    /// [`bp_stats`](Self::bp_stats), and start counting from zero again.
    pub fn take_bp_stats(&mut self) -> crate::fec::ldpc::bp::BpStats {
        self.bp.take_stats()
    }

    /// One attempt at the staircase for the spectra read at (`g`, `p`): `None` if the candidate does not decode.
    fn attempt(
        &mut self,
        cells: &HalfDbCells,
        cand: &SyncCandidate,
        g: Grid,
        p: Place,
        hs: Option<u8>,
        depth: DecodeDepth,
        mode: OsdMode,
    ) -> Option<DecodeResult> {
        let gate = self.gate.zip(hs).map(|(gate, hs)| GateCtx { gate, hs });
        let view = View {
            cells,
            t: self.tables,
        };
        process_candidate_with_scratch_ctx(
            self.cs,
            self.bp,
            cand,
            |cs, _, mask| view.fill_mask(cs, g, p, mask),
            depth,
            DEFAULT_Q_THRESH,
            // `Only` runs the OSD of an attempt whose BP already failed: the staircase gets one pass, which cannot decode what it did not
            if mode == OsdMode::Only {
                0
            } else {
                DEFAULT_BP_MAX_ITER
            },
            &[],
            PassCtx::FIRST
                .skip_d(cand.score < self.d_min_score)
                .with_osd_seeds(self.osd_seeds)
                .with_osd_mode(mode, hs.unwrap_or(0))
                .with_gate(gate)
                .with_ap_context(self.ap_ctx),
        )
    }

    /// One candidate: decoded at its grid place from `cells`, and, if that fails and the Costas tones point to a place `min_shift` cells off, once more there (the
    /// retry; the decode then reports the place it was found at). `None` if it does not decode. The candidate's place is `cand`'s (the search's grid).
    pub fn decode_candidate(
        &mut self,
        cells: &HalfDbCells,
        cand: &SyncCandidate,
        depth: DecodeDepth,
        min_shift: f32,
    ) -> Option<Found> {
        self.decode_candidate_at(cells, cand, None, depth, min_shift)
    }

    /// [`decode_candidate`](Self::decode_candidate) for a candidate whose hard-sync count `hs` (the search's, of 21) is known, which is what the worker's gate
    /// ([`with_gate`](Self::with_gate)) reads: with a gate set, a candidate it gives no chance is not decoded.
    pub fn decode_candidate_hs(
        &mut self,
        cells: &HalfDbCells,
        cand: &SyncCandidate,
        hs: u8,
        depth: DecodeDepth,
        min_shift: f32,
    ) -> Option<Found> {
        self.decode_candidate_at(cells, cand, Some(hs), depth, min_shift)
    }

    /// After an attempt in [`OsdMode::Defer`]: queue it if it reached the OSD fallback.
    fn queue_if_deferred(
        &mut self,
        cand: &SyncCandidate,
        hs: Option<u8>,
        g: Grid,
        p: Option<Place>,
    ) {
        if !self.osd_deferred {
            return;
        }
        #[cfg(any(feature = "fft-rustfft", feature = "osd"))]
        if let Some(logit) = self.bp.osd_work().deferred.take() {
            self.pending.push(OsdAttempt {
                cand: cand.clone(),
                hs: hs.unwrap_or(0),
                g,
                p,
                logit,
            });
        }
        #[cfg(not(any(feature = "fft-rustfft", feature = "osd")))]
        let _ = (cand, hs, g, p);
    }

    /// [`decode_round`](Self::decode_round) with the OSD fallback deferred and capped: the round's candidates run their BP staircase, the attempts that fail it are
    /// queued, and the best `k` of them ([`osd_pass`](Self::osd_pass)) run the OSD, their decodes pushed to `known` after the round's own (in the order a subtraction
    /// flow reads them). `depth` must have `osd`. `k = 0` is `decode_round` without OSD; unbounded `k` is `decode_round` with the OSD at once, up to the order in
    /// which a retry's BP decode and a grid attempt's OSD decode meet. The worker's own [`with_osd_deferred`](Self::with_osd_deferred) setting is restored. Returns
    /// how many decodes were added.
    pub fn decode_round_osd(
        &mut self,
        cells: &HalfDbCells,
        cands: &[(SyncCandidate, u8)],
        max_cand: usize,
        depth: DecodeDepth,
        min_shift: f32,
        known: &mut Vec<Found>,
        k: usize,
    ) -> usize {
        let was = core::mem::replace(&mut self.osd_deferred, true);
        let a = self.decode_round(cells, cands, max_cand, depth, min_shift, known);
        self.osd_deferred = was;
        a + self.osd_pass(cells, k, depth, known)
    }

    /// The OSD fallback of the best `k` of the queued attempts ([`with_osd_deferred`](Self::with_osd_deferred)), by [`OsdRank`](super::OsdRank) score, each run again at
    /// its place for the OSD alone; the queue is emptied. A decode already in `known` (same 77 bits) is not added again. Returns how many were added. `depth` is the one the
    /// attempts ran with (it must have `osd`).
    pub fn osd_pass(
        &mut self,
        cells: &HalfDbCells,
        k: usize,
        depth: DecodeDepth,
        known: &mut Vec<Found>,
    ) -> usize {
        let mut pend = self.take_osd_attempts();
        self.osd_queued += pend.len() as u64;
        self.osd_ran += pend.len().min(k) as u64;
        rank_osd_attempts(&mut pend, k);
        let before = known.len();
        for a in &pend {
            if let Some(d) = self.run_osd_attempt(cells, a, depth)
                && !known.iter().any(|o| o.res.message77() == d.res.message77())
            {
                known.push(d);
            }
        }
        pend.clear();
        self.pending = pend; // the capacity stays
        known.len() - before
    }

    /// The attempts queued so far ([`with_osd_deferred`](Self::with_osd_deferred)), taken: the queue is empty after it.
    pub fn take_osd_attempts(&mut self) -> Vec<OsdAttempt> {
        core::mem::take(&mut self.pending)
    }

    /// One queued attempt run for the OSD alone, at its place on `cells` (the BP staircase gets one pass): the decode, with the frequency and DT of the place it was
    /// found at. Not checked against any list of decodes. `depth` must have `osd`.
    pub fn run_osd_attempt(
        &mut self,
        cells: &HalfDbCells,
        a: &OsdAttempt,
        depth: DecodeDepth,
    ) -> Option<Found> {
        let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
        let tstep = cells.rate().nstep() as f32 / cells.rate().hz() as f32;
        let place = a.p.unwrap_or(ON_GRID);
        let mut d = self.attempt(cells, &a.cand, a.g, place, Some(a.hs), depth, OsdMode::Only)?;
        if a.p.is_some() {
            (d.freq_hz, d.dt_sec) = place.moved(d.freq_hz, d.dt_sec, df, tstep);
        }
        Some(Found { res: d, grid: a.g })
    }

    fn decode_candidate_at(
        &mut self,
        cells: &HalfDbCells,
        cand: &SyncCandidate,
        hs: Option<u8>,
        depth: DecodeDepth,
        min_shift: f32,
    ) -> Option<Found> {
        let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
        let tstep = cells.rate().nstep() as f32 / cells.rate().hz() as f32;
        let g = Grid {
            j0: ((cand.dt_sec + 0.5) / tstep).round() as i32,
            k: (cand.freq_hz / df).round() as i64,
        };
        let mode = if self.osd_deferred {
            OsdMode::Defer
        } else {
            OsdMode::Inline
        };
        let mut found = self.attempt(cells, cand, g, ON_GRID, hs, depth, mode);
        self.queue_if_deferred(cand, hs, g, None);
        if found.is_none() {
            let p = View {
                cells,
                t: self.tables,
            }
            .place(g);
            if p.worth_a_retry(min_shift) {
                let d = self.attempt(cells, cand, g, p, hs, depth, mode);
                self.queue_if_deferred(cand, hs, g, Some(p));
                if let Some(mut d) = d {
                    (d.freq_hz, d.dt_sec) = p.moved(d.freq_hz, d.dt_sec, df, tstep);
                    found = Some(d);
                }
            }
        }
        found.map(|res| Found { res, grid: g })
    }

    /// The first `max_cand` of `cands` decoded at their grid place, and, for each that did not decode, at the place the Costas tones point to when that is
    /// `min_shift` cells off (the retry); a decode already in `known` (same 77 bits) is not added again. Returns how many were added. The worker's memory is
    /// the caller's and is **reused** by every call (a firmware keeps one worker per core for the slot, and for every pass of a subtraction flow): a call
    /// does not remake the staging block or the BP scratch, which is all it guarantees: the attempts still make small buffers (the LLR step, see
    /// [`process_candidate_with_scratch`](super::process_candidate_with_scratch)) and `known` grows.
    pub fn decode_round(
        &mut self,
        cells: &HalfDbCells,
        cands: &[(SyncCandidate, u8)],
        max_cand: usize,
        depth: DecodeDepth,
        min_shift: f32,
        known: &mut Vec<Found>,
    ) -> usize {
        let before = known.len();
        for (cand, hs) in cands.iter().take(max_cand) {
            if let Some(f) = self.decode_candidate_hs(cells, cand, *hs, depth, min_shift)
                && !known.iter().any(|o| o.res.message77() == f.res.message77())
            {
                known.push(f);
            }
        }
        known.len() - before
    }
}

/// The FT8 flow of the S3 at 8 kHz from the half-dB cells: the search of `search` ([`coarse_sync_gated_cells`]: `ft8_lib`'s statistic, the
/// hard-sync order, `search.keep` candidates), the first `max_cand` of them decoded at their grid place, and a second try, at the place
/// estimated from the Costas tones, for each that did not decode and whose estimate is a quarter cell or more off the grid (`RETRY_MIN_SHIFT`).
/// The audio is read forward once, plus once for the slot's peak.
///
/// `None` unless `rate.coarse_ft8lib()` (`Hz8000`, `fixed-point`). `depth` is the staircase's (`DecodeDepth::EMBEDDED` with `osd` on is
/// the S3's). A decode found on the second try reports the place it was found at (`freq_hz`, `dt_sec` moved by the estimate); one found
/// at the grid, the grid's (3.125 Hz, 80 ms: coarser than a fine sync's). `DecodeResult::sync_score` is the search's score in 0.5 dB units, and
/// `snr_db` is [`SnrBaseline`]'s (against `jt9`'s: mean 0, SD 2.1 dB **in-sample**, OSD off, `tests/cells_flow_accuracy.rs`).
///
/// **A deliberate divergence from WSJT-X** (see this module). Peak memory: the cells (172 KB at 200-3000 Hz, made as bytes by the demux: no
/// `u16` band in between) plus the scores of the band (n_carriers x 60 bytes) and the spectra of one candidate; 258 KB on the host heap meter
/// at 200-3000 Hz, against 422 KB for the `fine_sync_at` flow. Candidates are taken in the search's hard-sync order (no block-0 re-rank).
pub fn decode_block_cells<S: AudioSample>(
    audio: &[S],
    rate: Ft8Rate,
    search: &GatedSearch,
    max_cand: usize,
    depth: DecodeDepth,
) -> Option<Vec<DecodeResult>> {
    decode_cells_inner(audio, rate, search, max_cand, depth, RETRY_MIN_SHIFT)
}

/// [`decode_block_cells`] with the retry's threshold a parameter (the tests move it).
fn decode_cells_inner<S: AudioSample>(
    audio: &[S],
    rate: Ft8Rate,
    search: &GatedSearch,
    max_cand: usize,
    depth: DecodeDepth,
    min_shift: f32,
) -> Option<Vec<DecodeResult>> {
    let (cands, cells) = coarse_sync_gated_cells(&mut SliceSource(audio), rate, search, 1)?;
    let tables = CellTables::new();
    let mut known = Vec::new();
    decode_round(
        &cells, &tables, &cands, max_cand, depth, min_shift, &mut known,
    );
    // the SNR from the complete cells of the slot
    let base = SnrBaseline::new(&cells, &tables);
    for f in &mut known {
        base.apply(&cells, &tables, f);
    }
    Some(known.into_iter().map(|f| f.res).collect())
}

/// A decode with the grid place of the candidate it came from (the place [`View::refine`] starts from).
pub struct Found {
    pub res: DecodeResult,
    pub grid: Grid,
}

/// [`CellsWorker::decode_round`] with a worker of its own (the staging block and the BP scratch made for this one round): what the one-shot flows use.
#[allow(clippy::too_many_arguments)]
pub(super) fn decode_round(
    cells: &HalfDbCells,
    tables: &CellTables,
    cands: &[(SyncCandidate, u8)],
    max_cand: usize,
    depth: DecodeDepth,
    min_shift: f32,
    known: &mut Vec<Found>,
) -> usize {
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> = vec![[Cmplx::<f32>::default(); 8]; 79]
        .try_into()
        .expect("79 x 8");
    let mut bp = Box::new(Ft8BpScratch::new());
    CellsWorker::new(&mut cs, &mut bp, tables)
        .with_d_min_score(CELLS_D_MIN_SCORE)
        .decode_round(cells, cands, max_cand, depth, min_shift, known)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::tx::{message_to_tones, synthesize_i16};
    use crate::ft8::Ft8;
    use crate::msg::wsjt77::{pack77, unpack77};

    /// A slot of 15 s at 8 kHz: the signals `(call1, call2, report, tone-0 Hz, DT s, amplitude)` and Gaussian-ish noise of `noise` (sd, i16
    /// counts), seeded.
    fn slot(signals: &[(&str, &str, &str, f32, f32, i16)], noise: f32, seed: u64) -> Vec<i16> {
        let hz = 8000u32;
        let mut v = vec![0f32; 15 * hz as usize];
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
                (x + (g - 6.0) * noise).round().clamp(-32768.0, 32767.0) as i16
            })
            .collect()
    }

    fn osd_on() -> DecodeDepth {
        DecodeDepth {
            osd: true,
            ..DecodeDepth::EMBEDDED
        }
    }

    fn texts(out: &[DecodeResult]) -> Vec<alloc::string::String> {
        let mut t: Vec<_> = out.iter().filter_map(|d| unpack77(d.message77())).collect();
        t.sort();
        t
    }

    fn search() -> GatedSearch {
        GatedSearch::new(200.0, 3000.0, 1.3, 30)
    }

    /// Clean signals on and off the grid decode from the cells: three stations, at 700.0 Hz (a grid bin), 1500.3 Hz and 2300.7 Hz, DT 0.2, 0.9
    /// and -0.6 s, noise sd 1500 against amplitudes 5000-6000. **Fails if** the flow does not decode a plain signal (the search, the fill and the
    /// staircase together; a fill one row off the grid breaks it, found by mutation). It does **not** catch a one-bin error of the fill alone
    /// (the fill test below does) or a wrong amplitude scale (the tables test does).
    #[test]
    fn clean_signals_decode_from_the_cells() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 5000),
        ];
        let s = slot(&signals, 1500.0, 11);
        let out = decode_block_cells(&s, Ft8Rate::Hz8000, &search(), 15, osd_on()).unwrap();
        assert_eq!(
            texts(&out),
            ["CQ JA1ABC PM95", "CQ K1ABC FN42", "K1ABC W9XYZ -12"],
            "{out:?}"
        );
    }

    /// 12 kHz has no cells: `None`, not a silent empty list. **Fails if** the rate gate is dropped.
    #[test]
    fn no_cells_flow_at_12_khz() {
        let s = slot(&[], 1500.0, 1);
        assert!(decode_block_cells(&s, Ft8Rate::Hz12000, &search(), 15, osd_on()).is_none());
    }

    /// The estimator on a signal **off the grid**: at 1500.0 Hz + 1.2 Hz (0.38 of a 3.125 Hz bin) and DT +0.03 s (0.38 of an 80 ms row)
    /// from where the nearest grid cell is, it points the right way in both directions and lands within 0.25 cell of the truth. **Fails if** the
    /// row or bin direction is flipped, or the 3 x 3 energy reads the wrong rows or bins (a shift inside `View::h` that `place` sees; the fill's own
    /// reads are the fill test's).
    #[test]
    fn the_estimate_points_to_a_signal_off_the_grid() {
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let tstep = 0.08f32;
        let mut checked = 0;
        for (f_true, dt_true) in [
            (1501.2f32, 0.2 + 0.03),
            (1498.7, 0.2 - 0.03),
            (1501.0, 0.2),
            (1500.0, 0.2 - 0.032),
        ] {
            let s = slot(&[("CQ", "K1ABC", "FN42", f_true, dt_true, 6000)], 1500.0, 3);
            let (cands, cells) =
                coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &search(), 1)
                    .unwrap();
            // the candidate nearest to the signal
            let (c, _) = cands
                .iter()
                .min_by(|a, b| {
                    (a.0.freq_hz - f_true)
                        .abs()
                        .total_cmp(&(b.0.freq_hz - f_true).abs())
                })
                .unwrap();
            let t = CellTables::new();
            let v = View {
                cells: &cells,
                t: &t,
            };
            let g = Grid {
                j0: ((c.dt_sec + 0.5) / tstep).round() as i32,
                k: (c.freq_hz / df).round() as i64,
            };
            let p = v.place(g);
            let (want_r, want_c) = ((dt_true - c.dt_sec) / tstep, (f_true - c.freq_hz) / df);
            let (got_r, got_c) = (p.dr as f32 * p.wa, p.dc as f32 * p.wb);
            assert!(
                (got_r - want_r).abs() <= 0.25 && (got_c - want_c).abs() <= 0.25,
                "signal {f_true} Hz {dt_true} s from cell ({}, {}): shift truth ({want_r:.2} rows, {want_c:.2} bins), estimate ({got_r:.2}, {got_c:.2})",
                c.freq_hz,
                c.dt_sec
            );
            checked += 1;
        }
        assert_eq!(checked, 4);
    }

    /// The tables: the amplitude of a cell is `10^(h / 40)` (40 half-dB units a decade of amplitude) and the square root of the power table's
    /// `10^(h / 20)`. **Fails if** either is off by a scale (a decade per 10 or 20 units) or the two disagree.
    #[test]
    fn the_tables_are_the_power_scale() {
        let t = CellTables::new();
        assert_eq!(t.amp[0], 1.0);
        assert_eq!((t.amp[OUTSIDE], t.pow[OUTSIDE]), (0.0, 0.0));
        assert!((t.amp[40] - 10.0).abs() < 1e-3, "{}", t.amp[40]);
        assert!((t.pow[20] - 10.0).abs() < 1e-3, "{}", t.pow[20]);
        for h in 0..=MAX_CELL {
            assert!(
                (t.amp[h] * t.amp[h] / t.pow[h] - 1.0).abs() < 1e-4,
                "cell {h}: amplitude {} power {}",
                t.amp[h],
                t.pow[h]
            );
        }
    }

    /// The parabola's vertex points to the larger neighbour, is 0 for a symmetric peak, and a better neighbour than the centre points half way to it.
    /// **Fails if** the sign is flipped or the neighbour rule is dropped.
    #[test]
    fn the_parabola_points_to_the_larger_neighbour() {
        assert_eq!(parab(1.0, 2.0, 1.0), 0.0);
        assert!(parab(1.0, 4.0, 2.0) > 0.0 && parab(1.0, 4.0, 2.0) < 0.5);
        assert!(parab(2.0, 4.0, 1.0) < 0.0 && parab(2.0, 4.0, 1.0) > -0.5);
        assert_eq!(parab(5.0, 3.0, 1.0), -0.5);
        assert_eq!(parab(1.0, 3.0, 5.0), 0.5);
    }

    /// A decode found at an estimated place reports it: towards the neighbour row / bin by the weight. **Fails if** either sign is flipped or a weight
    /// swapped (the DT sign is not pinned by the decode-level test: its adjustments are small).
    #[test]
    fn a_place_moves_the_reported_frequency_and_dt_towards_the_neighbour() {
        let p = Place {
            dr: 1,
            dc: -1,
            wa: 0.5,
            wb: 0.25,
        };
        let (f, dt) = p.moved(1500.0, 0.2, 3.125, 0.08);
        assert!((f - (1500.0 - 0.25 * 3.125)).abs() < 1e-4, "{f}");
        assert!((dt - (0.2 + 0.5 * 0.08)).abs() < 1e-6, "{dt}");
        let p = Place {
            dr: -1,
            dc: 1,
            wa: 0.1,
            wb: 0.4,
        };
        let (f, dt) = p.moved(1500.0, 0.2, 3.125, 0.08);
        assert!(
            (f - (1500.0 + 0.4 * 3.125)).abs() < 1e-4 && (dt - (0.2 - 0.1 * 0.08)).abs() < 1e-6
        );
    }

    /// The threshold: an estimate a tenth of a cell off the grid is not worth a retry, two fifths is. **Fails if** the threshold is dropped.
    #[test]
    fn a_retry_needs_a_quarter_cell() {
        let p = |wa, wb| Place {
            dr: 1,
            dc: 1,
            wa,
            wb,
        };
        assert!(!p(0.1, 0.1).worth_a_retry(RETRY_MIN_SHIFT));
        assert!(!p(0.24, 0.0).worth_a_retry(RETRY_MIN_SHIFT));
        assert!(p(0.4, 0.0).worth_a_retry(RETRY_MIN_SHIFT));
        assert!(p(0.0, 0.4).worth_a_retry(RETRY_MIN_SHIFT));
    }

    /// The retry gains on signals off the grid: a signal 0.38 cell off in time and frequency (1501.2 Hz, DT 0.23 s), at amplitudes 150-180 against
    /// noise sd 1500, is decoded more often with the retry (threshold 0.25) than with none (threshold 9) over the same 20 seeded slots each: 54
    /// against 46 decodes of 80 slots as it stands (the grid place alone decodes 57 %). **Fails if** the retry is never made (the threshold or the
    /// failure test is wrong) or the second try is made at the same place or the wrong bin side (a flipped `dc` is caught here; a flipped `dr`
    /// only by the estimator test).
    #[test]
    fn the_retry_decodes_what_the_grid_place_does_not() {
        let (mut with, mut without) = (0, 0);
        for amp in [150i16, 160, 170, 180] {
            for seed in 0..20u64 {
                let s = slot(
                    &[("CQ", "K1ABC", "FN42", 1501.2, 0.23, amp)],
                    1500.0,
                    200 + seed,
                );
                let run = |min_shift| {
                    decode_cells_inner(&s, Ft8Rate::Hz8000, &search(), 15, osd_on(), min_shift)
                        .unwrap()
                        .len()
                };
                with += run(RETRY_MIN_SHIFT);
                without += run(9.0);
            }
        }
        assert!(
            with >= without + 5,
            "with the retry {with} decodes, without it {without} (of 80 slots)"
        );
    }

    /// The fill reads the grid cells: a clean strong signal at a grid place (700.0 Hz is bin 224, DT 0.3 s is row 10) has, at all but a few of its 79
    /// symbols, the strongest of the eight tone cells at the transmitted tone. **Fails if** the fill reads the wrong row (`j0 + 2 s`) or bin
    /// (`k + 2 t`), the tone order is wrong, or the grid place is rounded the wrong way (a one-bin shift splits a tone between two cells, a one-row
    /// shift mixes two symbols).
    #[test]
    fn the_fill_reads_the_transmitted_tones() {
        let m77 = pack77("CQ", "K1ABC", "FN42").unwrap();
        let tones = message_to_tones::<Ft8>(&m77);
        let s = slot(&[("CQ", "K1ABC", "FN42", 700.0, 0.3, 8000)], 1500.0, 21);
        let (cands, cells) =
            coarse_sync_gated_cells(&mut SliceSource(&s), Ft8Rate::Hz8000, &search(), 1).unwrap();
        let (c, _) = cands
            .iter()
            .min_by(|a, b| {
                (a.0.freq_hz - 700.0)
                    .abs()
                    .total_cmp(&(b.0.freq_hz - 700.0).abs())
            })
            .unwrap();
        assert_eq!(
            (c.freq_hz, c.dt_sec),
            (700.0, 10.0 * 0.08 - 0.5),
            "the grid place of the signal"
        );
        let df = SAMPLE_RATE_HZ / super::super::types::NFFT_SPEC as f32;
        let g = Grid {
            j0: ((c.dt_sec + 0.5) / 0.08).round() as i32,
            k: (c.freq_hz / df).round() as i64,
        };
        let t = CellTables::new();
        let v = View {
            cells: &cells,
            t: &t,
        };
        let mut cs = [[Cmplx::<f32>::default(); 8]; 79];
        v.fill(&mut cs, g, ON_GRID);
        let right = cs
            .iter()
            .zip(&tones)
            .filter(|(row, tone)| {
                let best = (0..8)
                    .max_by(|&a, &b| row[a].re.total_cmp(&row[b].re))
                    .unwrap();
                best == **tone as usize
            })
            .count();
        assert!(
            right >= 76,
            "{right} of 79 symbols read the transmitted tone"
        );
    }

    /// The retry's mix: the square root of the weighted sum of the four cells' **powers**, and no power outside the slot. Built by hand: the cell at the
    /// grid place is 40 (power 100, amplitude 10) and its row neighbour 60 (power 1 000, amplitude 31.6); at half weight the amplitude is
    /// `sqrt(0.5 x 100 + 0.5 x 1 000)` = 23.45, not the mean of the amplitudes (20.8) or of the powers (550). **Fails if** the mix is in amplitude, the square root is dropped, or the
    /// weights are swapped; and a place off the slot reads 0.
    #[test]
    fn the_retry_mixes_powers_and_the_outside_has_none() {
        let (nf, nt) = (40usize, 60usize);
        let mut data = vec![40u8; nf * nt];
        for b in 0..nf {
            data[6 * nf + b] = 60;
        }
        let cells = HalfDbCells::from_parts(nf, nt, 0, Ft8Rate::Hz8000, data);
        let t = CellTables::new();
        let v = View {
            cells: &cells,
            t: &t,
        };
        let g = Grid { j0: 5, k: 3 };
        let mut cs = [[Cmplx::<f32>::default(); 8]; 79];
        v.fill(
            &mut cs,
            g,
            Place {
                dr: 1,
                dc: 0,
                wa: 0.5,
                wb: 0.0,
            },
        );
        let want = (0.5 * t.pow[40] + 0.5 * t.pow[60]).sqrt();
        assert!(
            (cs[0][0].re / want - 1.0).abs() < 1e-5,
            "{} against {want}",
            cs[0][0].re
        );
        assert!((want - 23.45).abs() < 0.01, "{want}");
        // symbol 0 at row 5 on the grid, row 6 with the shift; a symbol far past the last row has no power
        v.fill(&mut cs, g, ON_GRID);
        assert!((cs[0][0].re - t.amp[40]).abs() < 1e-3);
        assert_eq!(cs[78][0].re, 0.0, "row {} is past the slot", 5 + 156);
        let g = Grid { j0: -200, k: -5 };
        v.fill(&mut cs, g, ON_GRID);
        assert!(cs.iter().all(|r| r.iter().all(|c| c.re == 0.0)));
    }

    /// A decode found on the retry reports the place it was found at: for the seeded slots where the retry decodes a signal the grid place does not,
    /// the mean error of the reported frequency is under 1 Hz (measured 0.41; the grid cell is 1.2 Hz off, a flipped sign leaves it above 1.2) and of the DT under 20 ms (measured 13 ms; the grid row is 30 ms off).
    /// **Fails if** the sign of the frequency or DT adjustment is flipped (each ends up 2.4 Hz / 60 ms off) or it is not applied at the half-weight rule.
    #[test]
    fn a_retry_decode_reports_the_place_it_was_found_at() {
        let (mut checked, mut ferr, mut terr) = (0, 0.0f32, 0.0f32);
        for amp in [150i16, 160, 170] {
            for seed in 0..20u64 {
                let s = slot(
                    &[("CQ", "K1ABC", "FN42", 1501.2, 0.23, amp)],
                    1500.0,
                    200 + seed,
                );
                let run = |min_shift| {
                    decode_cells_inner(&s, Ft8Rate::Hz8000, &search(), 15, osd_on(), min_shift)
                        .unwrap()
                };
                let (with, without) = (run(RETRY_MIN_SHIFT), run(9.0));
                if let (Some(d), None) = (with.first(), without.first()) {
                    ferr += (d.freq_hz - 1501.2).abs();
                    terr += (d.dt_sec - 0.23).abs();
                    checked += 1;
                }
            }
        }
        assert!(
            checked >= 3,
            "only {checked} slots where the retry alone decodes"
        );
        let (ferr, terr) = (ferr / checked as f32, terr / checked as f32);
        assert!(
            ferr <= 1.0 && terr <= 0.02,
            "mean error {ferr} Hz, {terr} s over {checked} decodes"
        );
    }

    /// A fill by mask writes only the symbols of the mask, and the three masks the staircase asks for (`SyncBlock0`, `SyncBlocks12`, `DataOnly`) together make the whole block.
    /// **Fails if** a mask fills another mask's symbols (the staircase would gate on a block it has not read, or overwrite the sync block a failed gate leaves), or a
    /// symbol is in no mask.
    #[test]
    fn a_fill_by_mask_writes_only_its_symbols() {
        let a = slot(&[("CQ", "K1ABC", "FN42", 700.0, 0.3, 8000)], 1500.0, 21);
        let (_, cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search(), 1).unwrap();
        let t = CellTables::new();
        let v = View {
            cells: &cells,
            t: &t,
        };
        let g = Grid { j0: 10, k: 224 };
        let mut all = [[Cmplx::<f32>::default(); 8]; 79];
        v.fill(&mut all, g, ON_GRID);
        let mut staged = [[Cmplx::<f32>::default(); 8]; 79];
        v.fill_mask(&mut staged, g, ON_GRID, SymMask::SyncBlock0);
        for (sym, row) in staged.iter().enumerate() {
            let filled = row.iter().any(|c| c.re != 0.0);
            assert_eq!(filled, sym < 7, "symbol {sym} after SyncBlock0");
        }
        v.fill_mask(&mut staged, g, ON_GRID, SymMask::SyncBlocks12);
        v.fill_mask(&mut staged, g, ON_GRID, SymMask::DataOnly);
        assert!(
            staged
                .iter()
                .zip(&all)
                .all(|(x, y)| x.iter().zip(y).all(|(p, q)| p.re == q.re)),
            "the three masks together are the whole block"
        );
        assert!(all.iter().all(|r| r.iter().any(|c| c.re != 0.0)));
    }

    /// One worker over the caller's memory decodes, candidate by candidate, what a fresh worker decodes for each, and what `decode_block_cells` decodes in all.
    /// **Fails if** state leaks from one candidate into the next (the staging block or the BP scratch is not rewritten), or the per-candidate step changes the
    /// first-try / retry logic.
    #[test]
    fn a_worker_decodes_candidate_by_candidate_what_the_flow_decodes() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 5000),
        ];
        let a = slot(&signals, 1500.0, 11);
        let (cands, cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search(), 1).unwrap();
        let tables = CellTables::new();
        let mut cs = Box::new([[Cmplx::<f32>::default(); 8]; 79]);
        let mut bp = Box::new(Ft8BpScratch::new());
        let mut reused = CellsWorker::new(&mut cs, &mut bp, &tables);
        let mut got = Vec::new();
        for (c, _) in cands.iter().take(15) {
            let one = reused.decode_candidate(&cells, c, osd_on(), RETRY_MIN_SHIFT);
            let mut cs2 = Box::new([[Cmplx::<f32>::default(); 8]; 79]);
            let mut bp2 = Box::new(Ft8BpScratch::new());
            let fresh = CellsWorker::new(&mut cs2, &mut bp2, &tables).decode_candidate(
                &cells,
                c,
                osd_on(),
                RETRY_MIN_SHIFT,
            );
            assert_eq!(
                one.as_ref().map(|f| *f.res.message77()),
                fresh.as_ref().map(|f| *f.res.message77()),
                "candidate at {} Hz: a reused worker and a fresh one differ",
                c.freq_hz
            );
            if let Some(f) = one {
                got.push(f.res);
            }
        }
        let mut want =
            texts(&decode_block_cells(&a, Ft8Rate::Hz8000, &search(), 15, osd_on()).unwrap());
        let mut have = texts(&got);
        have.dedup();
        want.dedup();
        assert_eq!(have, want);
        assert_eq!(have.len(), 3);
    }

    /// Signals synthesised **at 48 kHz** (the radio's USB audio) with white noise decode through the 8 kHz path (the new 6:1 decimator, then the cells flow) and the
    /// 12 kHz path (4:1, then the host decoder): the same three messages. **Fails if** the 8 kHz decimator loses a signal the 12 kHz one finds (a band edge too low, aliasing,
    /// a level change).
    #[test]
    fn signals_at_48_khz_decode_through_the_8_khz_path_as_through_the_12_khz_path() {
        use crate::engine::dsp::{resample_to_8k, resample_to_12k};
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 5000),
        ];
        let hz = 48_000u32;
        let mut v = vec![0f32; 15 * hz as usize];
        for &(a, b, c, f, dt, amp) in &signals {
            let tones = message_to_tones::<Ft8>(&pack77(a, b, c).unwrap());
            let pcm = synthesize_i16::<Ft8>(&tones, hz, f, amp);
            let at = ((0.5 + dt) * hz as f32).round() as i64;
            for (i, &x) in pcm.iter().enumerate() {
                let idx = at + i as i64;
                if idx >= 0 && (idx as usize) < v.len() {
                    v[idx as usize] += x as f32;
                }
            }
        }
        // white noise at 48 kHz with the density of the 8 kHz tests' sd 1 500 (x sqrt 6)
        let mut st = 77u64;
        let x48: Vec<i16> = v
            .iter()
            .map(|&x| {
                let mut g = 0.0f32;
                for _ in 0..12 {
                    st = st
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    g += ((st >> 40) as f32) / (1u64 << 24) as f32;
                }
                (x + (g - 6.0) * 3674.0).round().clamp(-32768.0, 32767.0) as i16
            })
            .collect();
        let want = ["CQ JA1ABC PM95", "CQ K1ABC FN42", "K1ABC W9XYZ -12"];
        let x8 = resample_to_8k(&x48, hz);
        let out8 = decode_block_cells(&x8, Ft8Rate::Hz8000, &search(), 15, osd_on()).unwrap();
        assert_eq!(texts(&out8), want, "the 8 kHz path");
        let x12 = resample_to_12k(&x48, hz);
        let out12 = crate::ft8::decode_block::decode_block(&x12, 100.0, 3000.0, 1.0, osd_on(), 30);
        assert_eq!(texts(&out12), want, "the 12 kHz path");
    }
}
