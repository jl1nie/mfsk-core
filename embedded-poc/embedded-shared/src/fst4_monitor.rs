//! The FST4 wideband monitor pipeline — issue #307/#327.
//!
//! Everything a receiver does to one slot of audio, in the order it
//! does it, with no bench or application policy mixed in:
//!
//! | stage | when | cost (real CoreS3, FST4-60 wideband) |
//! |---|---|---|
//! | [`SlotCapture`] — DDC + spectrogram, block by block | during capture | 5839 ms, 9% duty of a 60 s slot |
//! | [`coarse_search`] — correlation fill (dual-core) + ranking | post-slot | 1009 ms |
//! | [`run_candidate_loop`] — refine + decode, rank-tiered, dual-core | post-slot | first decode +1174 ms, all 50 in 33.1 s |
//!
//! Extracted from `apps::fst4_ddc_bench`, which measured every one of
//! those numbers and remains the thing that re-measures them. The
//! extraction exists because a receiver application needs the same
//! pipeline and the repo has already paid once for the alternative:
//! the code-sharing audit (#290-298) found "80% shared" to be 31.8% in
//! practice, mostly through copies like this one would have been.
//!
//! ## What is policy and what is not
//!
//! [`MonitorConfig`] carries every number a caller might legitimately
//! choose differently — band, cap, tier depth, deadline. What it does
//! *not* carry is anything that changes the answer rather than the
//! schedule: `sync_min` defaults to the production 0.8 and raising it
//! is a bench-only trick that disables issue #146's stage-1 OR-gate
//! (see `apps::fst4_ddc_bench`'s own constant for the full warning).
//!
//! ## Monitor semantics
//!
//! The loop is **breadth-first over coarse score, not depth-first over
//! the ladder**, because the number a monitoring receiver is judged on
//! is time-to-first-decode, not total. Two measurements put the real
//! signal at median coarse rank 1 at every SNR and on both AWGN and
//! CCIR-moderate fading (#337/#343), which is what makes that ordering
//! pay and what [`MonitorConfig::full_depth_ranks`] exploits: the top
//! ranks get the full production ladder — OSD *and* production's
//! automatic `i0 ± 1` timing retry — while the tail is capped. Under
//! fading that tiering lands within 1-2 trials per 100 of the full
//! production decoder (#345).

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use num_complex::Complex32;

use mfsk_core::engine::dsp::ddc::StreamingComplexDdc;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::{
    RxGrid, Spectrogram, SpectrogramBuilder, Sync2dShape, SyncCandidate, coarse_sync_from_sync2d,
    fill_sync2d_row, spectra_crop_for, sync2d_shape,
};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{
    REFINE_DS_RATE_HZ, grid_for, sniper_cascade, sniper_refine_recenter, wideband_cascade,
    wideband_refine_recenter,
};
use mfsk_core::fst4::rung_major::{RungMajorCandidate, decode_phase_split_timed};
use mfsk_core::msg::wsjt77::unpack77;

use crate::fst4_dual_core;

/// Matches `fst4::decode`'s own (private) `SYNC_Q_MIN`.
const SYNC_Q_MIN: u32 = 16;

/// Which DDC cascade the front end runs, and therefore which analysis
/// grid everything downstream sits on.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Band {
    /// L/M 64/243, no integer stage; K=1024, nfft1=2048. The
    /// production 100-3000 Hz search.
    Wide,
    /// 49-tap /3 + L/M 16/81; K=256, nfft1=512. A known-target window,
    /// used by the bench to answer "does the chain work end to end".
    Sniper,
}

/// Everything about a slot's decode that a caller may choose.
#[derive(Copy, Clone, Debug)]
pub struct MonitorConfig {
    pub band: Band,
    /// DDC centre. The cascade's passband is built around it.
    pub center_hz: f32,
    /// Search half-width around [`Self::center_hz`].
    pub half_width_hz: f32,
    /// Bandwidth handed to `grid_for` — drives `K` and `nfft1`.
    pub grid_bandwidth_hz: f32,
    /// **Leave at the production 0.8** unless you have read
    /// `apps::fst4_ddc_bench`'s constant of the same name: this value
    /// doubles as issue #146's stage-1 gate, where a high value
    /// disables ~3 dB of FST4 sensitivity rather than merely tightening
    /// a score filter.
    pub sync_min: f32,
    pub max_cand: usize,
    /// Per-candidate wall-clock decode cap for candidates outside the
    /// tier, in ms; `0` disables it.
    ///
    /// Bounding wall-clock cannot separate an expensive noise candidate
    /// from a weak signal that needs OSD — they are the same work — so
    /// this is a schedule knob, not a quality one, and #343 measured
    /// that under fading a uniform cap removes threshold recall
    /// outright (0/100 without OSD at -26 dB). It is survivable only
    /// because of [`Self::full_depth_ranks`].
    pub cap_ms: i64,
    /// How many top-ranked candidates skip the cap and take the full
    /// production ladder — which is what carries OSD *and* production's
    /// automatic `i0 ± 1` retry. `0` makes the cap uniform.
    pub full_depth_ranks: usize,
    /// Wall-clock bound on the whole candidate loop; a core that
    /// reaches it stops claiming candidates.
    pub deadline_ms: i64,
    /// Spread the correlation fill and the candidate loop across both
    /// cores ([`fst4_dual_core`], which must have been `init()`ed —
    /// before WiFi, per its own ordering constraint).
    pub dual_core: bool,
}

impl MonitorConfig {
    /// The shipped FST4-60 wideband monitor: 100-3000 Hz at production
    /// `sync_min`, 150 ms cap beyond rank 8, dual-core, 45 s deadline.
    /// Every default here is a measured choice — see the module doc and
    /// #337/#341/#343 for which measurement bought which number.
    pub const FST4_60_WIDEBAND: Self = Self {
        band: Band::Wide,
        center_hz: 1550.0,
        half_width_hz: 1450.0,
        grid_bandwidth_hz: 2900.0,
        sync_min: 0.8,
        max_cand: 50,
        cap_ms: 150,
        full_depth_ranks: 8,
        deadline_ms: 45_000,
        dual_core: true,
    };

    fn freq_lo(&self) -> f32 {
        self.center_hz - self.half_width_hz
    }
    fn freq_hi(&self) -> f32 {
        self.center_hz + self.half_width_hz
    }

    /// The analysis grid the DDC output sits on.
    pub fn rx_grid(&self) -> RxGrid {
        RxGrid::complex(grid_for(self.grid_bandwidth_hz).fs_c, self.center_hz)
    }
}

pub fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Absolute `esp_timer` deadline (µs) the [`budget_ok`] `fn` item reads.
///
/// `decode_phase_split_timed` takes `Option<fn() -> bool>` — a plain
/// function pointer, not a closure — so the deadline cannot be captured
/// and has to travel through a global. Xtensa's `max_atomic_width` is
/// 32, so there is no `AtomicI64` to use; this mirrors
/// `wspr_dual_core`'s `DeadlineCell` exactly, including why it is an
/// `UnsafeCell` rather than an atomic.
///
/// **Per core**, not one shared cell: under [`MonitorConfig::dual_core`]
/// both cores sit inside [`monitor_candidate`] on *different*
/// candidates at once, so a single cell would let one core's arming
/// truncate the other's ladder. Indexed by `xTaskGetCoreID`.
struct DeadlineCells(core::cell::UnsafeCell<[i64; 2]>);
// SAFETY: each core writes and reads only its own slot, and only
// between its own candidates. No slot is ever touched by two cores.
unsafe impl Sync for DeadlineCells {}
static DEADLINE_US: DeadlineCells = DeadlineCells(core::cell::UnsafeCell::new([i64::MAX; 2]));

fn core_slot() -> usize {
    (unsafe { esp_idf_svc::sys::xTaskGetCoreID(core::ptr::null_mut()) } as usize) & 1
}

fn set_deadline(abs_us: i64) {
    unsafe { (*DEADLINE_US.0.get())[core_slot()] = abs_us };
}

fn budget_ok() -> bool {
    now_us() < unsafe { (*DEADLINE_US.0.get())[core_slot()] }
}

// ─────────────────────────────────────────────────────────────────────
// Capture-time front end

/// The DDC and the spectrogram, fed block by block as audio arrives.
///
/// This does not make the front end cheaper; it moves it. A receiver
/// has the whole slot to do this work, so what it costs against is a
/// *duty cycle*, not the post-slot guard budget — 5839 ms of a 60 s
/// slot, 9%. WSPR already ships this shape (`wspr_app`'s `ddc_loop` ->
/// `DDC_READY_IDX` -> `scan_loop`).
///
/// The spectrogram is bit-identical at any block size (pinned by
/// `fst4_spectrogram_builder`), so how a caller chunks its audio is a
/// pacing choice, not a numerical one.
pub struct SlotCapture {
    cfg: MonitorConfig,
    ddc: StreamingComplexDdc,
    builder: Option<SpectrogramBuilder>,
    coarse_i: Vec<f32>,
    coarse_q: Vec<f32>,
}

impl SlotCapture {
    /// `with_spectrogram = false` skips the capture-time spectrogram,
    /// leaving [`CapturedSlot::spectra`] empty for a caller that means
    /// to build it after the slot (which is what the bench's batch mode
    /// measures against).
    pub fn new(cfg: MonitorConfig, with_spectrogram: bool) -> Self {
        Self::with_input_hint(cfg, with_spectrogram, 0)
    }

    /// Same, but reserve the baseband buffers up front for a slot of
    /// `input_samples` 12 kHz samples.
    ///
    /// Worth doing on a device: the pair grows by doubling otherwise,
    /// and at 1.5 MB combined a reallocation near the end of a slot
    /// transiently needs 3 MB of an 8 MB PSRAM that is also holding a
    /// spectrogram and, in a receiver, the previous slot.
    pub fn with_input_hint(
        cfg: MonitorConfig,
        with_spectrogram: bool,
        input_samples: usize,
    ) -> Self {
        let cascade = match cfg.band {
            Band::Wide => wideband_cascade(cfg.center_hz),
            Band::Sniper => sniper_cascade(cfg.center_hz),
        };
        let builder = if with_spectrogram {
            spectra_crop_for::<Fst4s60>(cfg.freq_lo(), cfg.freq_hi(), cfg.rx_grid())
                .map(|(lo, hi)| SpectrogramBuilder::new::<Fst4s60>(lo, hi, cfg.rx_grid()))
        } else {
            None
        };
        // The cascades' output rates: wideband decimates by 243/64,
        // sniper by 3 * 81/16. A little slack for `flush`'s tail.
        let cap = match cfg.band {
            Band::Wide => input_samples * 64 / 243 + 64,
            Band::Sniper => input_samples * 16 / 243 + 64,
        };
        Self {
            cfg,
            ddc: StreamingComplexDdc::new(&cascade),
            builder,
            coarse_i: Vec::with_capacity(cap),
            coarse_q: Vec::with_capacity(cap),
        }
    }

    /// Feed the next block of 12 kHz mono audio. Block boundaries do
    /// not affect the result.
    ///
    /// The cascade appends straight into the slot buffer and the
    /// builder is handed the tail that just appeared, rather than both
    /// reading a separate block buffer that is then copied in. That
    /// copy is not free at this size: it cost 80 ms of a 1300 ms DDC
    /// on real hardware when the whole slot arrives as one push.
    pub fn push_i16(&mut self, audio: &[i16]) {
        let from = self.coarse_i.len();
        self.ddc
            .push_i16(audio, &mut self.coarse_i, &mut self.coarse_q);
        if let Some(b) = self.builder.as_mut() {
            b.push(&self.coarse_i[from..], &self.coarse_q[from..]);
        }
    }

    /// End the slot: flush the cascade's tail and finish the
    /// spectrogram.
    /// How many baseband samples have been produced so far. A caller
    /// pacing a slot by sample count is measuring the DDC's output,
    /// not its input, so this is the number that says whether a slot
    /// is actually full.
    pub fn produced(&self) -> usize {
        self.coarse_i.len()
    }

    pub fn finish(mut self) -> CapturedSlot {
        let from = self.coarse_i.len();
        self.ddc.flush(&mut self.coarse_i, &mut self.coarse_q);
        if let Some(b) = self.builder.as_mut() {
            b.push(&self.coarse_i[from..], &self.coarse_q[from..]);
        }
        // Where a non-finite value first appears decides what is
        // broken: the DDC, the spectrogram FFTs, or the correlation
        // that reads them. Checking both here rather than only after
        // the correlation is what separates those three.
        let bad_bb = self
            .coarse_i
            .iter()
            .chain(self.coarse_q.iter())
            .filter(|v| !v.is_finite())
            .count();
        if bad_bb > 0 {
            log::warn!("fst4_monitor: {bad_bb} non-finite baseband samples out of capture");
        }
        CapturedSlot {
            cfg: self.cfg,
            delay: self.ddc.group_delay_input_samples(),
            spectra: self.builder.map(|b| b.finish()),
            coarse_i: self.coarse_i,
            coarse_q: self.coarse_q,
        }
    }
}

/// One slot's baseband, and the spectrogram built alongside it.
pub struct CapturedSlot {
    pub cfg: MonitorConfig,
    pub coarse_i: Vec<f32>,
    pub coarse_q: Vec<f32>,
    /// `None` when the caller asked for no capture-time spectrogram.
    pub spectra: Option<Spectrogram>,
    /// The cascade's group delay, in input samples.
    pub delay: usize,
}

// ─────────────────────────────────────────────────────────────────────
// Coarse search

/// The correlation matrix, as both cores see it while filling it.
/// Rows are disjoint by construction — index `fi` owns
/// `sync2d[fi * n_lag ..][.. n_lag]` and nothing else — which is why
/// this loop needs no lock.
struct Sync2dCtx {
    s: *const Spectrogram,
    shape: *const Sync2dShape,
    sync2d: *mut f32,
    n_lag: usize,
}

/// One row of the correlation matrix. Returns `Option<()>` rather than
/// a result type because the output *is* the matrix; `Vec<()>` is a ZST
/// vector, so `run_split`'s result collection allocates nothing.
fn sync2d_row(ctx: &Sync2dCtx, fi: usize) -> Option<()> {
    // SAFETY: `run_split`/`drain_single` block until both cores are
    // done, so all three pointers outlive this call. The write is to
    // row `fi` alone, and `NEXT_IDX` hands out each `fi` exactly once,
    // so no two cores ever hold overlapping slices.
    let s = unsafe { &*ctx.s };
    let shape = unsafe { &*ctx.shape };
    let row = unsafe { core::slice::from_raw_parts_mut(ctx.sync2d.add(fi * ctx.n_lag), ctx.n_lag) };
    fill_sync2d_row::<Fst4s60>(s, shape, fi, row);
    Some(())
}

/// `coarse_sync_from_spectra` with its correlation fill spread across
/// both cores (issue #327).
///
/// Every row of the fill is a pure function of the finished
/// spectrogram, so the split decides only which core writes which row
/// and the results are **bit-identical** either way; the ranking stage
/// still sees the whole matrix.
///
/// Returns `(candidates, fill_us, rank_us)`. The two costs are worth
/// reporting apart: only the fill parallelises, so the rank half is the
/// floor this can approach — and it was the fill/rank split that found
/// the ranking stage to be 72% of a search everyone had assumed was
/// correlation-bound.
pub fn coarse_search(slot: &CapturedSlot) -> (Vec<SyncCandidate>, i64, i64) {
    let cfg = &slot.cfg;
    let Some(s) = slot.spectra.as_ref() else {
        return (Vec::new(), 0, 0);
    };
    let Some(shape) = sync2d_shape::<Fst4s60>(cfg.freq_lo(), cfg.freq_hi(), cfg.rx_grid()) else {
        return (Vec::new(), 0, 0);
    };
    {
        // `avg_power_per_bin` is the only whole-spectrogram view the
        // public API offers; a non-finite cell anywhere in a bin makes
        // that bin's average non-finite, which is enough to say
        // *whether* the spectrogram is the source and *which* bins.
        let avg = s.avg_power_per_bin();
        let bad = avg.iter().filter(|v| !v.is_finite()).count();
        if bad > 0 {
            let first = avg.iter().position(|v| !v.is_finite()).unwrap_or(0);
            log::warn!(
                "fst4_monitor: {bad} of {} spectrogram bins non-finite (first at bin {first}, \
                 offset {})",
                avg.len(),
                s.freq_offset(),
            );
        }
    }

    let t_fill0 = now_us();
    let mut sync2d = alloc::vec![0.0f32; shape.len()];
    {
        let ctx = Sync2dCtx {
            s: s as *const Spectrogram,
            shape: &shape as *const Sync2dShape,
            sync2d: sync2d.as_mut_ptr(),
            n_lag: shape.n_lag,
        };
        // No deadline: unlike a candidate, a row is not optional — a
        // row left unfilled is a hole in the search band, not merely a
        // decode not attempted.
        if cfg.dual_core {
            fst4_dual_core::run_split(&ctx, shape.n_freq, i64::MAX, sync2d_row);
        } else {
            fst4_dual_core::drain_single(&ctx, shape.n_freq, i64::MAX, sync2d_row);
        }
    }
    let fill_us = now_us() - t_fill0;

    // A non-finite cell here means the front end produced one, and the
    // ranking stage downstream cannot rank what it cannot compare.
    // Counted rather than asserted because a receiver that has been up
    // for hours should say what it saw and keep going — and because
    // real hardware produced this on a second slot after a clean first
    // one, which is exactly the shape a silent check would have hidden.
    let bad = sync2d.iter().filter(|v| !v.is_finite()).count();
    if bad > 0 {
        log::warn!(
            "fst4_monitor: {bad} of {} correlation cells are non-finite — \
             the baseband or spectrogram is corrupt",
            sync2d.len(),
        );
    }

    let t_rank0 = now_us();
    let cands = coarse_sync_from_sync2d::<Fst4s60>(
        s,
        &sync2d,
        &shape,
        cfg.sync_min,
        None,
        cfg.max_cand,
        cfg.rx_grid(),
    );
    (cands, fill_us, now_us() - t_rank0)
}

// ─────────────────────────────────────────────────────────────────────
// Candidate loop

/// Recentre `cand`'s own frequency out of the coarse DDC baseband down
/// to the refine (`ds_rate`) baseband, trim the best-effort combined
/// group delay, RMS-normalise (matching
/// `engine::pipeline::refine_candidate_position_impl`'s own convention,
/// so `process_candidate_precomputed`'s LLR scaling sees what it would
/// from `downsample_cached`). Same recipe
/// `tests/fst4_ddc_sniper_full_decode.rs::ddc_refine_and_decode`
/// verified against `downsample_cached` on host — see that test for the
/// delay-trim derivation this mirrors.
pub fn ddc_refine(
    cfg: &MonitorConfig,
    cand: &SyncCandidate,
    coarse_i: &[f32],
    coarse_q: &[f32],
    coarse_delay_orig: usize,
) -> Vec<Complex32> {
    let mut refine = match cfg.band {
        Band::Wide => wideband_refine_recenter(cand.freq_hz, cfg.center_hz),
        Band::Sniper => sniper_refine_recenter(cand.freq_hz, cfg.center_hz),
    };
    // Sized from the cascade's own fixed L/M ratio (plus a small margin
    // for `flush`'s tail) rather than growing from empty via repeated
    // reallocation — this runs once per candidate, each on a buffer up
    // to ~266 KB. Both refine configs share L=9; only M differs
    // (64 sniper / 256 wideband).
    let cap = match cfg.band {
        Band::Wide => coarse_i.len() * 9 / 256 + 16,
        Band::Sniper => coarse_i.len() * 9 / 64 + 16,
    };
    let mut cd0_i = Vec::with_capacity(cap);
    let mut cd0_q = Vec::with_capacity(cap);
    refine.push(coarse_i, coarse_q, &mut cd0_i, &mut cd0_q);
    refine.flush(&mut cd0_i, &mut cd0_q);

    let total_delay_out = (coarse_delay_orig as f32 * REFINE_DS_RATE_HZ / 12_000.0).round()
        as usize
        + refine.group_delay_output_samples();
    let trim = total_delay_out.min(cd0_i.len());

    let mut cd0: Vec<Complex32> = cd0_i[trim..]
        .iter()
        .zip(cd0_q[trim..].iter())
        .map(|(&i, &q)| Complex32::new(i, q))
        .collect();

    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
    cd0
}

/// One candidate's outcome. Carries `rank` because the dual-core split
/// does not preserve order — two cores complete interleaved.
pub struct MonitorHit {
    /// Position in the coarse-score ordering, 0-based.
    pub rank: usize,
    pub freq_hz: f32,
    pub refined_hz: f32,
    pub cscore: f32,
    /// The polyphase recentre alone — [`ddc_refine`].
    pub recenter_us: i64,
    /// Recentre plus `fst4_sync_search`. The two are split because
    /// they are different code with different memory behaviour, and a
    /// combined number cannot say which of them moved.
    pub refine_us: i64,
    pub decode_us: i64,
    /// Milliseconds after the loop started — the same clock on both
    /// cores, which is the number a monitor is judged on.
    pub t_ms: i64,
    pub msg: Option<String>,
    /// From the decode, when there was one — a receiver displays these,
    /// and both ladder arms return the same `DecodeResult` shape.
    pub snr_db: f32,
    pub dt_sec: f32,
    pub hard_errors: u32,
}

/// Everything one candidate needs that does not vary across the batch,
/// shared read-only by both cores. Raw pointers rather than slices
/// because this crosses a FreeRTOS task boundary; `run_split`'s
/// send → work → blocking-recv discipline is what makes them valid for
/// the whole batch.
struct MonitorCtx {
    cfg: MonitorConfig,
    candidates: *const core::ffi::c_void,
    n: usize,
    coarse_i: *const f32,
    coarse_q: *const f32,
    coarse_len: usize,
    coarse_delay_orig: usize,
    t0_us: i64,
}

/// One candidate's full work: refine, sync-search, decode. A plain `fn`
/// item so [`fst4_dual_core::run_split`] can hand it to the worker task
/// — a closure's captures cannot cross that boundary, which is why
/// every input arrives through [`MonitorCtx`].
fn monitor_candidate(ctx: &MonitorCtx, i: usize) -> Option<MonitorHit> {
    // SAFETY: `run_split` blocks until both cores are done, so these
    // outlive the call. Shared read-only across cores.
    let candidates: &[SyncCandidate] =
        unsafe { core::slice::from_raw_parts(ctx.candidates as *const SyncCandidate, ctx.n) };
    let coarse_i: &[f32] = unsafe { core::slice::from_raw_parts(ctx.coarse_i, ctx.coarse_len) };
    let coarse_q: &[f32] = unsafe { core::slice::from_raw_parts(ctx.coarse_q, ctx.coarse_len) };
    let cand = &candidates[i];

    let t_r = now_us();
    let cd0 = ddc_refine(&ctx.cfg, cand, coarse_i, coarse_q, ctx.coarse_delay_orig);
    let recenter_us = now_us() - t_r;
    let s2 = fst4_sync_search::<Fst4s60>(&cd0, cand);
    let refine_us = now_us() - t_r;

    let t_d = now_us();
    let result = if ctx.cfg.cap_ms > 0 && i >= ctx.cfg.full_depth_ranks {
        set_deadline(now_us() + ctx.cfg.cap_ms * 1000);
        let inputs = [RungMajorCandidate {
            cand: cand.clone(),
            cd0,
            refined_freq_hz: s2.freq_hz,
            i0: s2.i0,
        }];
        let (mut out, _) = decode_phase_split_timed::<Fst4s60>(
            &inputs,
            false,
            false,
            &[0],
            None,
            Some(budget_ok),
            12_000.0,
        );
        set_deadline(i64::MAX);
        out.pop().flatten()
    } else {
        // The full production ladder — which is what brings OSD *and*
        // production's automatic `i0 ± 1` retry (`pipeline.rs:1236`,
        // #308). That retry is the whole of the monitor's measured
        // fading gap, so which candidates land in this arm is the
        // single most consequential line in the loop (#344/#345).
        let precomputed = (cd0, s2.freq_hz, s2.i0, s2.score);
        process_candidate_precomputed::<Fst4s60>(
            cand,
            &[],
            &mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            precomputed,
            // `skip_snr = false`. It was `true` while FST4's only SNR
            // estimator needed the whole-slot FFT this path does not
            // build — the flag's original purpose was avoiding a second
            // `downsample_cached` that would have been pure waste here.
            // With `fst4_ddc_snr_db` reading the noise out of the
            // refined baseband instead, the estimate costs a handful of
            // 512-point FFTs per *decode* and the receiver has a number
            // to show.
            false,
            false,
        )
    };
    let decode_us = now_us() - t_d;

    let msg = result.as_ref().and_then(|r| {
        let m: &[u8; 77] = r.message77().try_into().ok()?;
        unpack77(m)
    });

    Some(MonitorHit {
        rank: i,
        freq_hz: cand.freq_hz,
        refined_hz: s2.freq_hz,
        cscore: cand.score,
        recenter_us,
        refine_us,
        decode_us,
        t_ms: (now_us() - ctx.t0_us) / 1000,
        msg,
        snr_db: result.as_ref().map_or(f32::NAN, |r| r.snr_db),
        dt_sec: result.as_ref().map_or(f32::NAN, |r| r.dt_sec),
        hard_errors: result.as_ref().map_or(0, |r| r.hard_errors),
    })
}

/// Walk `candidates` in coarse-score order, refining and decoding each,
/// until the list or [`MonitorConfig::deadline_ms`] runs out.
///
/// Results come back **sorted by rank**, which the dual-core path has
/// to restore explicitly: two cores complete interleaved.
pub fn run_candidate_loop(slot: &CapturedSlot, candidates: &[SyncCandidate]) -> Vec<MonitorHit> {
    // Alignment of the buffer every candidate's refine streams, logged
    // because it is not a constant of the code: it comes from wherever
    // the allocator put a multi-megabyte PSRAM block, which differs
    // between a bench that grows the buffer by doubling and an app that
    // reserves it up front. `engine::dsp::dotprod`'s esp-dsp PIE path
    // has an alignment requirement, so this is the first thing to look
    // at when the same refine costs different amounts in two binaries.
    let addr = slot.coarse_i.as_ptr() as usize;
    log::info!(
        "fst4_monitor: baseband at {:#x} (align {} B), {} samples",
        addr,
        1usize << addr.trailing_zeros().min(6),
        slot.coarse_i.len(),
    );

    let t0 = now_us();
    let n = candidates.len();
    let ctx = MonitorCtx {
        cfg: slot.cfg,
        candidates: candidates.as_ptr() as *const core::ffi::c_void,
        n,
        coarse_i: slot.coarse_i.as_ptr(),
        coarse_q: slot.coarse_q.as_ptr(),
        coarse_len: slot.coarse_i.len(),
        coarse_delay_orig: slot.delay,
        t0_us: t0,
    };
    let deadline = t0 + slot.cfg.deadline_ms * 1000;

    let mut hits = if slot.cfg.dual_core {
        fst4_dual_core::run_split(&ctx, n, deadline, monitor_candidate)
    } else {
        fst4_dual_core::drain_single(&ctx, n, deadline, monitor_candidate)
    };
    hits.sort_by_key(|h| h.rank);
    hits
}

/// The hits that decoded, one per distinct message, in the order they
/// were first decoded.
///
/// By decoded *message*, not by position: a monitor reports stations,
/// and two coarse cells for one signal are the same station. Cheaper
/// than a dedup barrier ahead of the loop, and it cannot delay a first
/// decode.
pub fn distinct_decodes(hits: &[MonitorHit]) -> Vec<&MonitorHit> {
    let mut out: Vec<&MonitorHit> = Vec::new();
    for h in hits {
        if let Some(text) = &h.msg {
            if !out.iter().any(|o| o.msg.as_deref() == Some(text.as_str())) {
                out.push(h);
            }
        }
    }
    out
}
