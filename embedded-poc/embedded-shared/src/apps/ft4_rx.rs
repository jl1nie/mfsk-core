// SPDX-License-Identifier: GPL-3.0-only
//! FT4 receive pipeline — board-agnostic half.
//!
//! The fourth receiver in this binary, and the first whose coarse stage
//! runs **during** capture rather than after the slot. That is the
//! whole reason it can exist: FT4's slot is 7.5 s and its frame ends at
//! 5.54 s, leaving 1 960 ms to decode in, and the periodogram alone was
//! 758 ms of that (`docs/notes/FT4_BENCHMARK.md` §25-32). Accumulated a
//! block at a time through [`Ft4SavgBuilder`] it costs 6 ms after the
//! slot instead, and the worst single block is 25 % of its own
//! real-time budget (§33) — so it is fed straight from the audio
//! callback with no queue of its own.
//!
//! ## Where the boundary is
//!
//! Everything here is data flow: feed [`SlotAccum`] audio blocks from
//! wherever they come from and it hands back a [`CapturedSlot`] when
//! one is complete; [`decode_slot`] turns that into messages. No
//! threading, no global state, no peripheral — the handoff between an
//! audio callback and a decode task belongs to the board crate, which
//! has `std` and knows which tasks exist. Same shared/board split the
//! rest of this tree uses: data flow crosses it, callbacks do not.
//!
//! Two callers: `m5stack-cores3-app`'s FT4 boot mode (a radio through
//! `uac.rs`'s `AudioSink`) and its `ft4-demo` bin (the vendored WSJT-X
//! golden replayed at 12 kHz — the FT4 equivalent of the FT8
//! controller's `wav_sim`, which FT4 had no answer to until now). Both
//! feed the 256-sample blocks a UAC read produces, so the demo
//! exercises the shipping cadence rather than a friendlier one.
//!
//! ## What it does not do
//!
//! No TX, no QSO state machine, no band control. This is a monitor.
//!
//! [`Ft4SavgBuilder`]: mfsk_core::engine::ft4_coarse::Ft4SavgBuilder

extern crate alloc;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::{ft4_coarse_sync_from_savg, Ft4SavgBuilder};
use mfsk_core::engine::pipeline::{process_candidate_precomputed, DecodeDepth, DecodeStrictness};
use mfsk_core::engine::sync2d::{
    Ft4CoarsePhasors, Ft4CoarseSweep, Ft4SweepScratch, ft4_sync_search_window_binned,
    ft4_sync_search_window_with,
};
use mfsk_core::engine::{FrameLayout, ModulationParams};
use mfsk_core::ft4::ddc::{
    CD0_LEN, CandidateDdc, SlotDecimator, candidate_baseband_boxcar, candidate_baseband_half,
};
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::ft4::Ft4;
use mfsk_core::msg::wsjt77::unpack77;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicUsize, Ordering};
use super::ft4_grid::{Anchor, SlotGrid};
use mfsk_core::engine::sync::SyncCandidate;
use num_complex::Complex;

/// One FT4 slot at 12 kHz: 7.5 s.
pub const SLOT_SAMPLES: usize = 90_000;

/// Where the capture window closes — **not** the end of the slot.
///
/// FT4 exists for fast QSOs, so the deadline this receiver is designed
/// against is the moment it has to key up, and everything is derived
/// backwards from there. Within a slot beginning at 0:
///
/// ```text
///   0.50 s  the other station's transmission starts
///   5.54 s  its frame ends (105 symbols x 48 ms)
///   6.56 s  ...plus the +1.0 s of DT `WSJTX_WINDOW` allows: every
///           sample `ft4_sync_search_window` can read has arrived
///   6.77 s  ...plus the DDC chain's group delay, so those samples are
///           filtered against real history and not against the zero
///           tail  <- CLOSE HERE
///   7.50 s  slot boundary
///   8.00 s  THIS station's transmission must start
/// ```
///
/// Closing at the slot boundary instead — which this receiver did
/// until 2026-09-01 — leaves 0.5 s to decode in before key-up, not the
/// 1.96 s the budget claimed, because the 1.96 s was anchored to the
/// slot end rather than to the transmission that follows it. The last
/// 0.94 s of the slot is audio no candidate can reach: the search tops
/// out at `i0 = 1012` and a frame is 105 x 32 = 3 360 downsampled
/// samples, so 4 372 of a slot's 5 000 — 6.56 s. The close sits 0.22 s
/// past that so those last samples are filtered against real history.
///
/// Measured lossless on the WSJT-X golden — same 12 candidates and the
/// same 11 decodes as the whole slot, both with the periodogram
/// averaged over the shorter span and with the tail zero-filled
/// (`tests/ft4_early_close.rs`). One recording is not a sensitivity
/// statement; the 560-file sweep is the arm that would be, and it has
/// not been run.
pub const CAPTURE_CLOSE_SAMPLES: usize = 81_300;

/// `ft4::decode`'s own private `SYNC_Q_MIN` — 16 sync symbols
/// (4 × Costas-4), at least half correct. Mirrored the same way
/// `ft4_bench` mirrors it, and with the same caveat: if the crate-side
/// value moves this silently stops matching the shipped gate.
const SYNC_Q_MIN: u32 = 8;

/// The coarse search, mirroring `ft4_wsjtx_samples.rs`'s `bench_assets`
/// so a number from this receiver is comparable to a number from
/// `ft4-bench`. `SYNC_MIN` is WSJT-X's own `syncmin = 1.2`
/// (`ft4_decode.f90:195`).
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 2700.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 100;

/// Δt search window — **WSJT-X's own**, `[-344, 1012]` in downsampled
/// samples, i.e. ±1.0 s about `dt = 0` at `i0 = 333`.
///
/// This receiver used to narrow it to `(0, 667)` (±0.5 s), which §18-19
/// measured as lossless on the golden and on an `ft4sim` DT sweep and
/// worth 1.5-1.9x on the search stage: what it gave up was *reach*,
/// not sensitivity. It is restored anyway. WSJT-X is this crate's
/// reference implementation and searches ±1.0 s because it cannot
/// assume the other station has a clock — a receiver that quietly
/// searches half of that decodes a different set of stations on a
/// real band, and "the ones inside ±0.5 s" is not a property anyone
/// operating the radio can see.
///
/// It is not free, and the price turned out to be somewhere other than
/// where §19 measured it. The search stage roughly doubling barely
/// shows — it is a smaller share of a candidate than it was before the
/// shared decimation and the second core — but
/// [`CAPTURE_CLOSE_SAMPLES`] has to extend to cover `i0 = 1012`, and
/// **that** costs 525 ms of budget, which on the 14-signal golden is
/// one to two candidates: 11 decodes become 9-10.
///
/// **Why that trade is still right, and why the fixture cannot show
/// it.** What the deadline cuts is the *weakest* candidates, so the
/// narrow window buys marginal-SNR stations. What the narrow window
/// gives up is every station whose clock is off by more than half a
/// second — and on a real band those are more common than the
/// marginal-SNR ones. The two errors also add: this receiver's own
/// slot alignment is still an open item (#313), so a station well
/// inside ±0.5 s of UTC can sit outside ±0.5 s of *us*.
///
/// The golden recording cannot weigh in on this. Its DTs span
/// -0.44…+0.30 s, so it fits inside the narrow window by
/// construction — which is exactly why §18-19's "lossless" was a
/// statement about that file rather than about the band. The
/// instrument that does speak is §18's `ft4sim` DT sweep, where recall
/// is 100 % inside the window and 0 % outside it: reach is a cliff,
/// not a curve, and every station past the edge is lost outright.
const WSJTX_WINDOW: (i32, i32) = (-344, 1012);

/// Phase error past which [`SlotAccum::anchor_or_reanchor`] moves the
/// grid rather than leaving the wobble to the DT-median trim. 100 ms — a
/// tenth of the ±1.0 s [`WSJTX_WINDOW`], the same ratio `uac.rs`'s
/// `SLOT_DRIFT_REANCHOR_MS` uses against FT8's ±2.5 s. It also has to be
/// larger than the DT search can pull back, so a clock step this side of
/// it is corrected here and anything smaller is left to converge.
const REANCHOR_THRESH_SAMPLES: i32 = 1_200;

/// The slot at 6 kHz, growing while the slot arrives — written by
/// [`SlotAccum`] on the capture task and read, a published prefix at a
/// time, by [`EarlyBasebands`] on the other core.
///
/// **One writer, readers of the prefix only.** The capture side appends
/// and then publishes the new length with `Release`; a reader loads the
/// length with `Acquire` and touches nothing past it. The buffer is
/// sized once, for the whole window, and never reallocates — the
/// writer checks before every append that it cannot, because a
/// reallocation would move the samples out from under a reader that is
/// part-way through them.
///
/// Shared through an `Arc` rather than lent, because the two lifetimes
/// genuinely differ: a grid re-anchor throws the accumulator's window
/// away (`SlotAccum::anchor_or_reanchor`) while a worker may still be
/// reading it, and the `Arc` is what keeps those samples alive until
/// the worker notices it has been orphaned.
pub struct HalfStream {
    buf: UnsafeCell<Vec<f32>>,
    /// The buffer's base, captured once. Never moves — see above.
    base: *const f32,
    len: AtomicUsize,
    closed: AtomicBool,
}
// SAFETY: the only mutation is `SlotAccum::push`, through
// `&mut SlotAccum` (so one writer), appending past the published length;
// readers read only below it. See the struct doc.
unsafe impl Sync for HalfStream {}
unsafe impl Send for HalfStream {}

impl HalfStream {
    fn new() -> Self {
        // The stage's group delay makes the window a little short of
        // exactly half; the margin is so no block split can make it one
        // sample long.
        let buf: Vec<f32> = Vec::with_capacity(CAPTURE_CLOSE_SAMPLES / 2 + 64);
        let base = buf.as_ptr();
        Self {
            buf: UnsafeCell::new(buf),
            base,
            len: AtomicUsize::new(0),
            closed: AtomicBool::new(false),
        }
    }

    /// The published samples. After the window has closed this is the
    /// whole slot at 6 kHz.
    pub fn as_slice(&self) -> &[f32] {
        let n = self.len.load(Ordering::Acquire);
        // SAFETY: `[0, n)` was written before `n` was published and is
        // never written again; the base never moves.
        unsafe { core::slice::from_raw_parts(self.base, n) }
    }

    fn is_closed(&self) -> bool {
        self.closed.load(Ordering::Acquire)
    }
}

/// A finished slot: its audio, and the periodogram accumulated while
/// that audio was arriving.
pub struct CapturedSlot {
    pub audio: Vec<i16>,
    /// Already averaged — [`Ft4SavgBuilder::finish`] has run.
    pub savg: Vec<f32>,
    /// The slot at 6 kHz, decimated *while it arrived* — the shared
    /// half of `ft4::ddc`'s front end, which every candidate then
    /// mixes and filters down from. Bit-identical to
    /// `ft4::ddc::decimate_slot` over the whole buffer whatever block
    /// sizes it was fed in (`slot_decimator_is_block_independent`),
    /// the same property `Ft4SavgBuilder` has and for the same reason:
    /// a `FirStage` carries its own history. Closed; read it through
    /// [`CapturedSlot::half`].
    pub half: Arc<HalfStream>,
    /// `esp_timer` microseconds at the moment the slot closed, so the
    /// decoder can report post-slot latency the way the benches do.
    pub closed_us: i64,
}

impl CapturedSlot {
    /// The whole window at 6 kHz.
    pub fn half(&self) -> &[f32] {
        self.half.as_slice()
    }
}

/// Bin spacing of the coarse periodogram, `12 000 / 2 304` Hz — the
/// grid every carrier the coarse stage reports is built on.
const COARSE_BIN_HZ: f32 = 12_000.0 / 2_304.0;

fn bin_of(freq_hz: f32) -> i32 {
    (freq_hz / COARSE_BIN_HZ).round() as i32
}

/// A carrier snapped to the nearest periodogram bin centre.
///
/// **What makes a baseband buildable before the list is final.** The
/// coarse stage refines each peak off-grid, so the same station reads a
/// slightly different carrier from a partial periodogram than from the
/// finished one, and a baseband mixed at the first cannot be reused
/// for the second. Snapped, both land on one bin. Free on the host
/// mirror: the snapped receiver decodes the golden's eleven, the Δt
/// search's own frequency refinement absorbing the half-bin
/// (`ft4_embedded_pipeline_mirror.rs`).
fn snap_to_bin(freq_hz: f32) -> f32 {
    bin_of(freq_hz) as f32 * COARSE_BIN_HZ
}

/// 12 kHz samples into the window at which the provisional candidate
/// list is read: the end of Costas block D — the last active symbol —
/// of a frame at `dt = 0`. 65 328 samples, 5.444 s.
///
/// A protocol position rather than a tuned time. By then a
/// nominally-timed frame has sent every one of its sync symbols, and on
/// the golden the list read there is the list the finished window
/// gives (`how_early_the_candidate_list_is_known`). A late station
/// still adds power afterwards; what that can change is the ranking,
/// and a candidate the provisional list missed is simply built after
/// the close, as before.
pub fn provisional_samples() -> usize {
    let blocks = Ft4::SYNC_MODE.blocks();
    let d = &blocks[blocks.len() - 1];
    let end_symbol = d.start_symbol as usize + d.pattern.len();
    (Ft4::TX_START_OFFSET_S * 12_000.0) as usize + end_symbol * Ft4::NSPS as usize
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Accumulates one slot of audio and its periodogram together.
///
/// Three things advance in step: the raw samples, the coarse stage's
/// periodogram, and the shared ÷2 the per-candidate DDC reads. None of
/// them can simply be run over the buffer afterwards — that is the
/// 758 ms (coarse) and ~109 ms (decimation) this receiver exists to
/// spend *during* the slot rather than after it, and on FT4 the
/// post-slot budget is candidates and therefore decodes
/// (`docs/notes/FT4_BENCHMARK.md` §32, §37, §42).
///
/// The audio is still kept whole: `snr_db` aside, a caller that wants
/// to re-run anything at 12 kHz needs it, and the replay path reads it.
pub struct SlotAccum {
    audio: Vec<i16>,
    savg: Ft4SavgBuilder,
    decim: SlotDecimator,
    half: Arc<HalfStream>,
    /// Whether [`take_provisional`](Self::take_provisional) has fired
    /// for this window.
    provisional_taken: bool,
    /// Where the window sits on the slot grid: the inter-window skip
    /// that keeps this a *slot* grid after the early close (without it
    /// each window would start 0.725 s earlier than the last and walk
    /// off the transmissions entirely), the pending phase correction,
    /// and whether an external clock has set the phase at all.
    ///
    /// Its own module, because it is integer arithmetic and this one
    /// cannot be compiled off-target — see [`SlotGrid`]'s doc. The
    /// board crate owns the clock and the DT tracker
    /// (`mfsk-app-shared`'s `time_sync`); this crate only moves the grid
    /// when told, the same shared/board split the module doc describes.
    grid: SlotGrid,
}

impl Default for SlotAccum {
    fn default() -> Self {
        Self::new()
    }
}

impl SlotAccum {
    pub fn new() -> Self {
        Self {
            audio: Vec::with_capacity(CAPTURE_CLOSE_SAMPLES),
            savg: Ft4SavgBuilder::new(CAPTURE_CLOSE_SAMPLES),
            decim: SlotDecimator::new(),
            half: Arc::new(HalfStream::new()),
            provisional_taken: false,
            grid: SlotGrid::new(CAPTURE_CLOSE_SAMPLES, SLOT_SAMPLES, REANCHOR_THRESH_SAMPLES),
        }
    }

    /// Whether the grid has been anchored to an external clock.
    pub fn is_aligned(&self) -> bool {
        self.grid.is_aligned()
    }

    /// The signed disagreement between the clock's boundary and the
    /// grid's, in samples, or `None` when it is inside the re-anchor
    /// threshold. Positive means the grid is running early.
    ///
    /// Same frame as [`anchor_or_reanchor`](Self::anchor_or_reanchor) —
    /// counted from the accumulator, not from now. Exposed so a
    /// receiver can log the number it is steering on.
    pub fn phase_error(&self, samples_to_boundary_from_here: usize) -> Option<i32> {
        self.grid.phase_error(samples_to_boundary_from_here)
    }

    /// Set — or, once set, trim — the slot grid's phase from an external
    /// clock.
    ///
    /// `samples_to_boundary` is what the board crate got from
    /// `time_sync::samples_to_next_slot_12k_ms(7_500)`: 12 kHz samples
    /// from now until the next UTC 7.5 s boundary.
    ///
    /// The first call anchors the grid outright — the next window opens
    /// on that boundary, and **a window already part-filled is thrown
    /// away**, because it straddles that boundary: its audio starts
    /// wherever the reader happened to begin, so every DT measured in
    /// it would be off by that much. One slot is the whole cost, and
    /// only on a receiver that had audio before it had a clock —
    /// `time_sync` reports no phase until the RTC or NTP lands, while
    /// UAC audio arrives regardless.
    ///
    /// Later calls move the grid only when the phase has drifted past
    /// [`REANCHOR_THRESH_SAMPLES`], which is what absorbs the clock
    /// stepping when NTP first disciplines an RTC-seeded clock — that
    /// step can be seconds, well past what the DT search could pull
    /// back. Those never discard: the correction lands at the next
    /// window close so windows stay exactly one window long. Jitter
    /// below the threshold is left for
    /// [`shift_next_window`](Self::shift_next_window).
    pub fn anchor_or_reanchor(&mut self, samples_to_boundary_from_here: usize) {
        if self.grid.anchor_or_reanchor(samples_to_boundary_from_here) == Anchor::DiscardPartial {
            // Carry the grid across the reset that drops the buffers,
            // the same way a window close does — the grid is the one
            // piece of state the new window inherits.
            let grid = self.grid;
            *self = Self::new();
            self.grid = grid;
        }
    }

    /// Fold a signed correction into the next inter-window skip — the
    /// DT-median trim from `time_sync::slot_dt_offset`. Positive delays
    /// the next window: the slot opened early, so the decoded signals
    /// sat late in it (WSJT-X's DT sign). Small by construction —
    /// [`anchor_or_reanchor`](Self::anchor_or_reanchor) has the grid
    /// within ~100 ms of UTC before this ever runs.
    pub fn shift_next_window(&mut self, delta_samples: i32) {
        self.grid.shift_next_window(delta_samples);
    }

    /// Decimate `samples` onto the shared stream and publish them.
    fn push_half(&mut self, samples: &[i16]) {
        // SAFETY: `&mut self` makes this the only writer; readers only
        // read below the published length, which is stored after the
        // append.
        let buf = unsafe { &mut *self.half.buf.get() };
        // The ÷2 yields at most one sample per two in, plus one.
        assert!(
            buf.len() + samples.len() / 2 + 1 <= buf.capacity(),
            "ft4_rx: half-rate stream would reallocate under its readers"
        );
        self.decim.push_i16(samples, buf);
        debug_assert_eq!(buf.as_ptr(), self.half.base);
        self.half.len.store(buf.len(), Ordering::Release);
    }

    /// The window's half-rate stream, for a worker to read while it
    /// grows.
    pub fn half_stream(&self) -> Arc<HalfStream> {
        self.half.clone()
    }

    /// The provisional candidate carriers — snapped, one per bin — once
    /// the window has reached [`provisional_samples`], and `None` before
    /// that and after it has fired once for this window.
    ///
    /// Read from a snapshot of the periodogram so far; the builder goes
    /// on accumulating untouched (`ft4_savg_snapshot_leaves_the_builder_alone`).
    pub fn take_provisional(&mut self) -> Option<Vec<f32>> {
        if self.provisional_taken || self.audio.len() < provisional_samples() {
            return None;
        }
        self.provisional_taken = true;
        let cands = ft4_coarse_sync_from_savg(
            &self.savg.snapshot(),
            FREQ_MIN_HZ,
            FREQ_MAX_HZ,
            SYNC_MIN,
            None,
            MAX_CAND,
        );
        let mut carriers: Vec<f32> = Vec::with_capacity(cands.len());
        for c in &cands {
            let f = snap_to_bin(c.freq_hz);
            if !carriers.iter().any(|&g| bin_of(g) == bin_of(f)) {
                carriers.push(f);
            }
        }
        Some(carriers)
    }

    /// Feed the next block. Returns the finished slot on the block that
    /// closes its window, and resets for the next.
    ///
    /// A block straddling the boundary is split, so the caller's block
    /// size never shifts the slot grid — which matters because a UAC
    /// read is not a divisor of 75 000 either.
    ///
    /// The window closes at [`CAPTURE_CLOSE_SAMPLES`] — 6.775 s of a
    /// 7.5 s slot — and the remaining 0.725 s is discarded, because a
    /// QSO-capable receiver has to have answered by 8.0 s and no
    /// candidate can read that audio anyway.
    ///
    /// No waterfall rows from here any more: the panel's waterfall is
    /// fed from the audio itself, the same way in every mode
    /// (`waterfall::WfRowBuilder`).
    pub fn push(&mut self, samples: &[i16]) -> Option<CapturedSlot> {
        let mut rest = samples;
        let mut done = None;
        while !rest.is_empty() {
            let dropped = self.grid.take_skip(rest.len());
            if dropped > 0 {
                rest = &rest[dropped..];
                continue;
            }
            let take = self.grid.room().min(rest.len());
            self.audio.extend_from_slice(&rest[..take]);
            self.savg.push(&rest[..take]);
            self.push_half(&rest[..take]);
            rest = &rest[take..];
            if self.grid.fill(take) {
                // The grid counts what this struct buffers, and the two
                // are advanced by the same `take` — so a disagreement
                // means the slot about to be decoded is not the window
                // the grid anchored, and every DT in it is off by the
                // difference. `stage1_inc::finalize_slot` runs the same
                // cross-check on FT8's reader (`audio_fill` against the
                // reported `total_samples`), which is what kept that
                // line's version of this visible.
                if self.audio.len() != CAPTURE_CLOSE_SAMPLES {
                    log::warn!(
                        "ft4_rx: window closed holding {} samples, grid says {}",
                        self.audio.len(),
                        CAPTURE_CLOSE_SAMPLES,
                    );
                }
                // `fill` has already set the next gap; carry the grid
                // across the reset that moves the buffers out.
                let grid = self.grid;
                let prev = core::mem::replace(self, Self::new());
                self.grid = grid;
                prev.half.closed.store(true, Ordering::Release);
                done = Some(CapturedSlot {
                    audio: prev.audio,
                    savg: prev.savg.finish(),
                    half: prev.half,
                    closed_us: now_us(),
                });
            }
        }
        done
    }
}

/// Each buffer a baseband built during capture owns is allocated at
/// least this big: one byte over `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`
/// (2 048 on the CoreS3), so the allocator puts it in PSRAM.
///
/// Measured on the host mirror with a counting allocator: twelve
/// basebands in flight at once would hold 136 KB below that threshold —
/// four times the largest internal block WiFi leaves — and with this
/// floor they hold 27 888 B, less than the serial receiver's peak
/// (`what_the_pipelined_receiver_asks_of_internal_dram`). The output is
/// bit-identical; only the placement moves.
const PIPELINED_MIN_ALLOC_BYTES: usize = 2_049;

/// Half-rate samples a capture-time worker feeds one baseband per step:
/// ~2 ms of DDC on a CoreS3 (83 ms for the 40 609 of a whole window).
/// With [`SWEEP_SLICE`] it bounds how long the decode waits for the
/// workers to let go at the close, and it is large enough that
/// `push_f32`'s per-call scratch is not the cost.
const EARLY_CHUNK: usize = 2_048;

/// Coarse block-cells a worker scores per step — 16 `i0` positions at
/// each of the nine `df`, ~1.3 ms at the Δt search's measured 2.18
/// cycles per multiply-add (§47). The same bound as [`EARLY_CHUNK`],
/// for the same reason.
const SWEEP_SLICE: usize = 144;

/// Stack for each capture-time worker.
///
/// **4 KB, measured.** It was 8 KB on the reasoning that it has the
/// candidate worker's shape; measured, it is not that deep — the early
/// DDC log's `stack [..] B free of 8192` read 6 688-6 836 B free on
/// both workers over every FT4 SIM run on 2026-09-22
/// (`logs/ft4sim_*_2026-09-22.log`), i.e. ~1.5 KB used. 4 KB keeps
/// ~2.5 KB of headroom and hands 8 KB of internal DRAM back.
const EARLY_STACK: u32 = 4 * 1024;

/// One candidate's baseband and coarse sweep, built while the slot is
/// still arriving.
struct Pipe {
    bin: i32,
    ddc: CandidateDdc,
    out: Vec<Complex<f32>>,
    /// Half-rate samples already fed.
    fed: usize,
    /// Taken by the decode, which finishes it.
    sweep: Option<Ft4CoarseSweep>,
}

impl Pipe {
    /// Feed the rest of the window and flush: the baseband
    /// `candidate_baseband_half` would have built from the whole slot,
    /// bit for bit (`pipelined_ddc_decodes_exactly_what_the_snapped_receiver_does`).
    fn finish(&mut self, half: &[f32]) -> Vec<Complex<f32>> {
        if self.fed < half.len() {
            self.ddc.push_f32(&half[self.fed..], &mut self.out);
            self.fed = half.len();
        }
        let mut cd0 = core::mem::take(&mut self.out);
        self.ddc.flush_to(CD0_LEN, &mut cd0);
        cd0.resize(CD0_LEN, Complex::new(0.0, 0.0));
        cd0
    }
}

/// Where each capture-time worker runs: `(core, priority)`.
///
/// Core 1 has nothing else to do during capture but the `net` task (2).
/// Core 0 carries the capture itself — the UAC reader (6) and the slot
/// task draining it (5) — and above those WiFi and the timers. Neither
/// worker can delay the audio.
///
/// **The core-0 worker is level with the slot task (5).** Run at the
/// panel's priority (1) instead, to keep the screen moving, it shared
/// core 0 with a panel that redraws its waterfall every frame and
/// measured the worse way on both counts: 65-72 % of the sweep done by
/// the close instead of 83-88 %, the loop ending at ~850 ms instead of
/// ~700, and the screen no less still — 0.9-1.5 s between frames,
/// because the decode after the close holds the core as long by itself
/// (2026-09-21, FT4 SIM, with and without the waterfall feed). A timed
/// yield to the panel every 100 ms did not shorten the gap either. The
/// screen's stall is an open item, not solved by priorities alone.
const EARLY_WORKERS: [(i32, u32); 2] = [(1, 4), (0, 5)];

struct EarlyShared {
    stream: Arc<HalfStream>,
    /// Handed between workers by `claimed` — a worker touches pipe `i`
    /// only while it holds `claimed[i]` — and to the decode once every
    /// worker has exited. Never resized after construction.
    pipes: UnsafeCell<Vec<Pipe>>,
    claimed: Vec<AtomicBool>,
    refs: Ft4CoarsePhasors,
    stop: AtomicBool,
    /// Workers still running; the decode waits for zero.
    running: AtomicU32,
    stack_hw_bytes: [AtomicU32; 2],
    started_us: i64,
}
// SAFETY: each pipe is touched by one worker at a time, under its
// `claimed` flag (Acquire on take, Release on give), and by the decode
// only after `running` reaches zero (Release/Acquire). Everything else
// is atomics or immutable.
unsafe impl Sync for EarlyShared {}
unsafe impl Send for EarlyShared {}

/// The per-candidate DDCs **and coarse Δt/Δf sweeps**, run on both
/// cores while the slot is still arriving.
///
/// **Why.** Measured on the board, a candidate after the close was
/// ~83 ms of DDC and ~120 ms of Δt search, and a slot's twelve are what
/// made decodes miss [`REPLY_DEADLINE_MS`]. The capture leaves both
/// cores mostly idle. Every `FirStage` carries its own history, so a
/// baseband fed in pieces is bit-identical to one fed the whole window,
/// and each coarse cell reads only the samples under its Costas blocks,
/// so it can be scored as soon as they exist — all of the ±1.0 s
/// window's cells are readable before the close on a 5.04 s frame
/// (`pipelined_sweep_decodes_exactly_what_the_snapped_receiver_does`).
/// What is left after the close is each baseband's last few percent and
/// flush, and the fine pass.
///
/// Started by the capture side at [`provisional_samples`] from
/// [`SlotAccum::take_provisional`]'s carriers; handed to
/// [`decode_slot_with`], which stops the workers (they let go within one
/// [`EARLY_CHUNK`] or [`SWEEP_SLICE`]), finishes whatever the final list
/// asks for, and builds the rest from scratch as before.
///
/// Dropping it without decoding just tells the workers to stop; each
/// holds its own reference and the last one out frees the state.
pub struct EarlyBasebands {
    shared: Arc<EarlyShared>,
    workers: u32,
}

impl EarlyBasebands {
    /// Allocate one baseband and sweep per carrier and start the
    /// workers, or `None` if none can be created — in which case the
    /// slot simply decodes the way it did before.
    pub fn start(stream: Arc<HalfStream>, carriers: &[f32]) -> Option<Self> {
        let pipes: Vec<Pipe> = carriers
            .iter()
            .map(|&f| Pipe {
                bin: bin_of(f),
                ddc: CandidateDdc::new_half_rate_with_min_alloc(f, PIPELINED_MIN_ALLOC_BYTES),
                out: Vec::with_capacity(CD0_LEN),
                fed: 0,
                sweep: Some(Ft4CoarseSweep::new(
                    WSJTX_WINDOW.0,
                    WSJTX_WINDOW.1,
                    CD0_LEN,
                    PIPELINED_MIN_ALLOC_BYTES,
                )),
            })
            .collect();
        let claimed = (0..pipes.len()).map(|_| AtomicBool::new(false)).collect();
        let shared = Arc::new(EarlyShared {
            stream,
            pipes: UnsafeCell::new(pipes),
            claimed,
            refs: Ft4CoarsePhasors::new::<Ft4>(),
            stop: AtomicBool::new(false),
            running: AtomicU32::new(0),
            stack_hw_bytes: [AtomicU32::new(0), AtomicU32::new(0)],
            started_us: now_us(),
        });
        let mut workers = 0u32;
        for (w, &(core, prio)) in EARLY_WORKERS.iter().enumerate() {
            let arg = Box::into_raw(Box::new((shared.clone(), w)));
            shared.running.fetch_add(1, Ordering::AcqRel);
            let created = unsafe {
                esp_idf_svc::sys::xTaskCreatePinnedToCore(
                    Some(early_worker),
                    c"ft4_early".as_ptr(),
                    EARLY_STACK,
                    arg as *mut core::ffi::c_void,
                    prio,
                    core::ptr::null_mut(),
                    core,
                )
            } == 1;
            if created {
                workers += 1;
            } else {
                shared.running.fetch_sub(1, Ordering::AcqRel);
                // SAFETY: the task never ran, so the box is still ours.
                drop(unsafe { Box::from_raw(arg) });
                log::warn!("ft4_rx: capture-time worker on core {core} not created ({EARLY_STACK} B stack)");
            }
        }
        if workers == 0 {
            return None;
        }
        Some(Self { shared, workers })
    }

    /// Stop the workers and wait for them to let go of the pipes.
    /// Returns the microseconds spent waiting.
    fn join(&self) -> i64 {
        let t0 = now_us();
        self.shared.stop.store(true, Ordering::Release);
        // Yield rather than sleep: the core-0 worker shares this task's
        // priority (see `EARLY_WORKERS`), and a tick is ~10 ms against
        // a slice of ~2.
        while self.shared.running.load(Ordering::Acquire) != 0 {
            unsafe {
                esp_idf_svc::sys::vPortYield();
                esp_idf_svc::sys::esp_rom_delay_us(50);
            }
        }
        now_us() - t0
    }
}

impl Drop for EarlyBasebands {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
    }
}

/// One worker's pass over the pipes. Returns whether it did anything.
/// `None` when told to stop.
fn early_pass(shared: &EarlyShared, scratch: &mut Ft4SweepScratch) -> Option<bool> {
    let half = shared.stream.as_slice();
    // SAFETY: the Vec is never resized; each element is touched only
    // under its `claimed` flag, taken below.
    let base = unsafe { (*shared.pipes.get()).as_mut_ptr() };
    let mut progressed = false;
    for (i, flag) in shared.claimed.iter().enumerate() {
        if shared.stop.load(Ordering::Acquire) {
            return None;
        }
        if flag
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            continue;
        }
        // SAFETY: claimed above; released below on every path.
        let p = unsafe { &mut *base.add(i) };
        let mut stopped = false;
        // Depth-first: the baseband catches up, then its sweep, before
        // the next pipe — so the strongest candidates finish first and
        // the state stays in cache.
        while p.fed < half.len() {
            if shared.stop.load(Ordering::Acquire) {
                stopped = true;
                break;
            }
            let end = (p.fed + EARLY_CHUNK).min(half.len());
            p.ddc.push_f32(&half[p.fed..end], &mut p.out);
            p.fed = end;
            progressed = true;
        }
        while let (false, Some(sw)) = (stopped, p.sweep.as_mut()) {
            if shared.stop.load(Ordering::Acquire) {
                stopped = true;
                break;
            }
            let before = sw.progress().0;
            sw.advance::<Ft4>(&p.out, CD0_LEN, scratch, &shared.refs, SWEEP_SLICE);
            if sw.progress().0 == before {
                break;
            }
            progressed = true;
        }
        flag.store(false, Ordering::Release);
        if stopped {
            return None;
        }
    }
    Some(progressed)
}

extern "C" fn early_worker(arg: *mut core::ffi::c_void) {
    // SAFETY: `start` leaked exactly one box for this task.
    let (shared, w) = *unsafe { Box::from_raw(arg as *mut (Arc<EarlyShared>, usize)) };
    {
        // **In internal DRAM**, unlike the pipes. Placed in PSRAM the
        // first time, the post-close search stage still cost ~115 ms a
        // candidate with only a third of its cells left — the one-shot
        // search's figure for all of them — because every dot product
        // reads these references. The one-shot search holds the same
        // set per core in internal DRAM, so this asks no more than the
        // receiver did.
        let mut scratch = Ft4SweepScratch::new::<Ft4>();
        loop {
            let closed = shared.stream.is_closed();
            match early_pass(&shared, &mut scratch) {
                None => break,
                Some(true) => {}
                Some(false) if closed => break,
                Some(false) => {
                    // Caught up with the capture: one UAC read is ~21 ms.
                    unsafe { esp_idf_svc::sys::vTaskDelay(1) };
                }
            }
        }
    }
    let hw = unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) };
    shared.stack_hw_bytes[w].store(hw, Ordering::Relaxed);
    shared.running.fetch_sub(1, Ordering::AcqRel);
    // Dropped before the task deletes itself — `vTaskDelete` does not
    // unwind, so anything still live here would leak.
    drop(shared);
    unsafe { esp_idf_svc::sys::vTaskDelete(core::ptr::null_mut()) };
}

/// One decoded transmission.
pub struct Ft4Decode {
    pub msg: String,
    /// The raw 77-bit payload, so the caller can resolve hashed
    /// callsigns and register this decode's own calls after the
    /// parallel workers have joined. `decode_candidate` runs on two
    /// cores and cannot hold a `&mut CallsignHashTable`; `msg` above
    /// is therefore the unresolved rendering, and a caller with a
    /// table replaces it.
    pub msg77: [u8; 77],
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
    /// BP hard-error count, as `DecodedRow::hard_errors` wants it —
    /// the shared FT8 screen marks a row borderline at 24 or more.
    pub hard_errors: u32,
}

/// Milliseconds from [`CAPTURE_CLOSE_SAMPLES`] to the moment this
/// station must be transmitting: `8.0 s − 6.775 s`.
///
/// The budget a **transceiver** has. Derived from key-up, not from the
/// slot boundary — see [`CAPTURE_CLOSE_SAMPLES`] for the timeline and
/// for what the old 1 960 ms was actually measuring. A receiver that
/// never transmits has until the next slot instead — see
/// [`decode_slot`]'s `budget_ms` and [`RX_ONLY_BUDGET_MS`].
///
/// Two things narrowed it from the 1 960 ms this used to claim: the
/// anchor moved to key-up (which is what the number always should have
/// meant), and [`WSJTX_WINDOW`] restored WSJT-X's full ±1.0 s DT
/// search, which pushes the capture window out by 0.52 s. Both are
/// deliberate. A QSO-capable build had 500 ms under the old
/// anchoring — the budget did not shrink, it was measured.
///
/// **1 025 ms since 2026-09-21: the reply deadline, [`REPLY_DEADLINE_MS`].**
/// The 300 was settled as a transmit-chain lead, not a contradiction
/// (`docs/reference/EMBEDDED.md`, "FT4: the reply is due when the audio
/// starts"), and the `intime` measurement it waited on came in: with
/// the basebands and the coarse sweep built during capture, all eleven
/// of the golden's decodes land by 1 025 ms on a CoreS3 (loop ends
/// 663-771 ms). At 1 225 the audio went out at 8.0 s, ~200 ms late at
/// the other end. The original note, kept for the reasoning it records:
///
/// **Upstream puts FT4's transmit audio at 0.3 s, not 0.5 s.** `Modulator::start` pads silence to `delay_ms`, and that
/// constant is **300 for FT4** where it is 500 for FT8 and 1000
/// otherwise (`Modulator/Modulator.cpp:71-74`). The FT4 *decoder*
/// nevertheless references 0.5 s — `xdt = ibest/666.67 - 0.5`
/// (`lib/ft4_decode.f90:462`), searching `ibmin=-344 .. ibmax=1012`
/// = ±1.0 s about it (`ft4_decode.f90:244-256`). The two disagree by
/// 0.2 s in upstream itself.
///
/// The timeline above takes 0.5 s for both, so if the modulator's
/// constant is the one that governs when *this* station must be
/// transmitting, this budget is **200 ms optimistic**:
/// `7.80 - 6.775` = 1 025 ms rather than 1 225 ms.
///
/// Left as it is, deliberately. The evidence available does not
/// settle it: the WSJT-X FT4 sample's own six decodes run dt −0.4 to
/// +0.3 (mean ≈ −0.1), which is real stations with real clock error
/// and cannot separate a systematic 0.2 s from the spread. Changing a
/// shipped budget on a reading of one constant is what the key-up
/// re-anchoring already had to undo once. What would settle it: a
/// WSJT-X FT4 transmission recorded against a disciplined clock, or
/// the upstream rationale for the 300.
pub const TX_TURNAROUND_BUDGET_MS: i64 = REPLY_DEADLINE_MS;

/// Milliseconds from [`CAPTURE_CLOSE_SAMPLES`] to the slot boundary:
/// `(90 000 − 81 300) / 12 kHz` = 725 ms. Where WSJT-X's GUI commits
/// its message, and **not** the reply deadline — see
/// [`REPLY_DEADLINE_MS`].
pub const SLOT_BOUNDARY_MS: i64 =
    ((SLOT_SAMPLES - CAPTURE_CLOSE_SAMPLES) as i64 * 1_000) / 12_000;

/// WSJT-X's FT4 audio start, measured from the boundary:
/// `Modulator::start`'s `delay_ms = 300` for FT4
/// (`Modulator/Modulator.cpp:74`).
pub const FT4_AUDIO_START_AFTER_BOUNDARY_MS: i64 = 300;

/// Milliseconds from [`CAPTURE_CLOSE_SAMPLES`] to the moment this
/// station's audio must start: [`SLOT_BOUNDARY_MS`] +
/// [`FT4_AUDIO_START_AFTER_BOUNDARY_MS`] = **1 025 ms**.
///
/// **The reply deadline.** The message has to exist when the waveform
/// starts, and not before: PTT needs no content, and this board keys
/// the IC-705 by VOX on its USB audio, so there is no PTT step at all —
/// the transmitter becomes active the moment the audio does. WSJT-X
/// commits at the boundary only because `guiUpdate` reads the message
/// in the same pass that raises PTT. The derivation, the upstream
/// sources and the ~150 ms transmit-chain lead the 300 carries are in
/// `docs/reference/EMBEDDED.md`, "FT4: the reply is due when the audio
/// starts".
///
/// **Also the cut, since 2026-09-21** — [`TX_TURNAROUND_BUDGET_MS`]
/// is this. WSJT-X never stops a decode for a transmission; this board
/// has a cut only because it decodes on the cores the transmitter
/// needs. `intime` counts the decodes a transmitter could have
/// answered; with the cut here, the two coincide.
pub const REPLY_DEADLINE_MS: i64 = SLOT_BOUNDARY_MS + FT4_AUDIO_START_AFTER_BOUNDARY_MS;

/// A whole FT4 slot, so a receive-only monitor can spend one.
///
/// Nothing is lost by overrunning [`TX_TURNAROUND_BUDGET_MS`] unless
/// the radio has to key up: slot `N + 1`'s audio is accumulating on
/// the capture side while slot `N` decodes, and only running past
/// *this* costs a slot.
///
/// Conservative since the early close: the next window opens 0.725 s
/// after this one closes, and a decode that runs past *that* spends
/// staging rather than the grid — which is the only reason a whole
/// slot is spendable here at all. Left at a slot
/// because the staging buffer a board holds is sized in seconds of
/// audio and 7.5 s is already more than it has (`apps/ft4.rs`'s
/// `STAGING_CAP` is 4 s — a receive-only build that really spent this
/// budget would drop audio, and says so when it does).
pub const RX_ONLY_BUDGET_MS: i64 = 7_500;

/// What one slot's decode produced, and what it cost.
pub struct SlotOutcome {
    pub decodes: Vec<Ft4Decode>,
    /// Candidates the coarse stage produced.
    pub cands: usize,
    /// How many of them the loop actually started before the deadline.
    /// `tried < cands` is the cut.
    pub tried: usize,
    /// Microseconds from slot close to the loop returning — including
    /// the overshoot of the candidate that was already running when
    /// the deadline passed.
    pub elapsed_us: i64,
    /// Coarse score of the first candidate that was skipped, if any.
    /// The number that says what the cut actually gave up: these are
    /// baseline-normalised, so 1.2 is WSJT-X's own threshold and a cut
    /// landing near it dropped almost nothing.
    pub cut_at_score: Option<f32>,
    /// Decodes that finished by [`REPLY_DEADLINE_MS`] after slot close.
    pub intime: usize,
    /// When the last distinct message finished, in ms after slot close.
    pub last_decode_ms: i64,
    /// Microseconds each core spent inside `decode_candidate`, and how
    /// many candidates each took. See `ParShared::busy_us` for what
    /// the pair is for.
    pub busy_us: [i64; 2],
    pub took: [usize; 2],
    /// Wall clock of the candidate loop alone — from the worker being
    /// created to the join. The denominator `busy_us` is a fraction
    /// of; `elapsed_us` is not, because it also carries the coarse
    /// sweep and the reference build.
    pub loop_us: i64,
}

/// Decode one captured slot, stopping when `budget_ms` after the slot
/// closed have passed.
///
/// The deadline is measured from [`CapturedSlot::closed_us`], not from
/// entry: a decoder that started late because it was still finishing
/// the previous slot has correspondingly less time, which is the
/// honest accounting and the one that keeps a receiver current.
///
/// **The cut takes the weakest candidates.** `ft4_coarse_sync` returns
/// them in descending coarse score, so truncation drops the tail —
/// which is both the cheapest thing to lose and, on the golden slot,
/// the candidates that decode last. Same shape as `fst4_monitor`'s
/// `run_candidate_loop`, which checks the deadline before starting each
/// candidate rather than trying to predict whether the next one fits;
/// predicting needs a per-candidate estimate that is itself wrong
/// whenever the band changes, and stopping one candidate early is worse
/// than overshooting by part of one.
///
/// This is the production per-candidate path `ft4-bench`'s SHIP arm
/// measures: DDC baseband, RMS normalise, narrowed Δt search, then
/// `process_candidate_precomputed`. The wideband `fft_cache` argument
/// is an **empty slice** — FT4's `snr_db` is a closed form over the
/// coarse candidate score (`pipeline::ft4_snr_db`) and never reads a
/// spectrum, which is what lets a receiver exist without the
/// 92 160-point transform no embedded backend can serve.
///
/// `depth` is [`DecodeDepth::EMBEDDED`]. On the 560-file sweep corpus
/// `FULL` reaches 237 decodes against `EMBEDDED`'s 179, so this gives
/// up about a quarter of the recall on weak signals — the OSD ladder is
/// what costs, and at 12 candidates it is ~900 ms of a 1 960 ms budget.
/// Revisit when the budget has room, not before.
/// Stack for the core-1 candidate worker.
///
/// **Re-measured 2026-09-22: 5 088 B used** (`ft4_rx: core-1 worker
/// stack 3104 B free of 8192`, FT4 SIM through the real sink), twice
/// the 2 584 B below — the per-candidate path has grown since. 3 KB of
/// headroom; re-measure before shrinking it.
///
/// **8 KB, from a measurement rather than a guess.** The first version
/// asked for 32 KB because the app's decode task did; then
/// `board::log_task_stacks` was made to report every task's headroom
/// instead of only the tight ones, and the task running exactly this
/// code — `ft4-demo`'s feed thread — turned out to use **2 584 B of
/// its 32 KB**. The per-candidate buffers are all heap (`cd0` alone is
/// 40 KB); the stack holds small locals.
///
/// That is not a tidiness point. This stack comes out of **internal
/// DRAM**, which is the board's scarce resource: with WiFi's driver
/// buffers live the largest free internal block measured 31 744 B
/// (2026-09-01, and §30 recorded the same number), against a 32 KB
/// ask. Sizing from the measurement takes the worker off that cliff
/// and hands 24 KB back to the allocations the decoder makes for
/// itself, whose fallback to PSRAM is what an FT4 slot actually pays
/// for (§26.3: 41 % on the 2 304-point workspace).
///
/// [`decode_slot`] still falls back to one core when the task cannot
/// be created — a receiver that decodes fewer candidates is a
/// receiver; one that panics at slot 1 is not.
const WORKER_STACK: u32 = 8 * 1024;

/// Everything the two cores share for one slot.
///
/// Candidates are taken from `next` rather than split in half:
/// `ft4_coarse_sync` returns them in descending score and they are not
/// equal cost, so a fixed split leaves one core idle at the end (§30
/// named this as one of the three reasons its probe reached 1.33x and
/// not 2x). Taking from a cursor also makes the deadline work
/// per-core with no coordination: whichever core notices first simply
/// stops taking.
struct ParShared {
    half: *const f32,
    half_len: usize,
    cands: *const SyncCandidate,
    n: usize,
    /// Per candidate, the index into `pipes` of the baseband built for
    /// it during capture, if any. Each pipe is assigned to at most one
    /// candidate, so a core holding candidate `i` holds its pipe alone.
    pipe_of: *const Option<usize>,
    pipes: *mut Pipe,
    next: AtomicUsize,
    deadline: i64,
    /// One slot per candidate, written by whichever core took that
    /// index. No lock: the cursor hands out each index exactly once,
    /// so the writes are disjoint, and `done`'s Release/Acquire pair
    /// publishes them all at the join.
    res: *const UnsafeCell<Option<Ft4Decode>>,
    /// Per candidate, microseconds after slot close that its decode
    /// finished, or −1 for none. Written by whichever core took the
    /// index, read after the join — the same disjoint-write argument
    /// as `res`. What turns "in time" from a total into a count.
    done_at_us: *const AtomicI32,
    closed_us: i64,
    /// The coarse Δt references, built once for the slot and read by
    /// both cores. Immutable after construction, which is what makes
    /// sharing it sound.
    refs: *const Ft4CoarsePhasors,
    done: AtomicBool,
    stack_hw_bytes: AtomicU32,
    /// Per-core occupancy, indexed by `xPortGetCoreID`. Each core
    /// accumulates on its own stack and stores once on the way out, so
    /// the two cores never share a cache line while they work.
    ///
    /// This is what separates the two ways a second core disappoints:
    /// if `busy[0] + busy[1]` is close to twice the wall clock, both
    /// cores worked the whole slot and the loss is *inside* a
    /// candidate (the FFT guard, the PSRAM bus); if it is well under,
    /// a core sat idle and the loss is the hand-out — the tail, or the
    /// deadline stopping one core early.
    /// Microseconds as `i32`: Xtensa has no 64-bit atomics, and a
    /// slot's busy time is bounded by the deadline (~1.2 s) with three
    /// orders of magnitude to spare.
    busy_us: [AtomicI32; 2],
    /// Candidates each core took off the cursor. Unequal counts with
    /// equal busy time is the cost spread, not an imbalance.
    took: [AtomicUsize; 2],
    /// The same busy time split by stage — `[DDC, Δt search, tail]`,
    /// per core. With occupancy already near 100 %, what is left to
    /// find is *which* stage a second core makes slower, and the
    /// stages differ in what they contend for: the DDC and the Δt
    /// search touch no FFT at all, while the tail's `symbol_spectra`
    /// takes esp-dsp's process-global `Fc32Guard`. A tail that alone
    /// inflates is the lock; all three inflating together is the
    /// memory bus.
    stage_us: [[AtomicI32; 3]; 2],
}
// SAFETY: `half` and `cands` address buffers owned by `decode_slot`'s
// frame, which does not return until `done` is set; every mutation
// goes through the atomics or the mutex.
unsafe impl Sync for ParShared {}

/// The candidate loop, run by both cores against the same cursor.
fn run_candidates(s: &ParShared) {
    // SAFETY: see the `Sync` impl — both slices outlive this call.
    let half = unsafe { core::slice::from_raw_parts(s.half, s.half_len) };
    let cands = unsafe { core::slice::from_raw_parts(s.cands, s.n) };
    // Both accumulators live on this core's stack (16 B) and are
    // published once below: a probe that wrote a shared atomic per
    // candidate would be measuring its own contention.
    let core =
        (unsafe { esp_idf_svc::sys::xTaskGetCoreID(core::ptr::null_mut()) } as usize).min(1);
    let mut busy = 0i64;
    let mut took = 0usize;
    let mut stage = [0i64; 3];
    // The fine pass's references, built on first use by a candidate
    // whose coarse sweep ran during capture, then kept for the slot.
    let mut scratch: Option<Ft4SweepScratch> = None;
    loop {
        if now_us() >= s.deadline {
            break;
        }
        let i = s.next.fetch_add(1, Ordering::AcqRel);
        if i >= cands.len() {
            break;
        }
        // SAFETY: built in `decode_slot`'s frame, which outlives every
        // worker (it waits for `done`), and never mutated after.
        let refs = unsafe { &*s.refs };
        // SAFETY: `pipe_of[i]` is unique to candidate `i`, and index
        // `i` is this core's alone (see `res`); the early worker has
        // exited before either core starts.
        let pipe = unsafe { (*s.pipe_of.add(i)).map(|k| &mut *s.pipes.add(k)) };
        let t0 = now_us();
        let d = decode_candidate(half, &cands[i], refs, pipe, &mut scratch, &mut stage);
        let t1 = now_us();
        busy += t1 - t0;
        took += 1;
        if d.is_some() {
            // SAFETY: as `res` below — index `i` is this core's alone.
            unsafe { &*s.done_at_us.add(i) }
                .store((t1 - s.closed_us) as i32, Ordering::Relaxed);
        }
        // SAFETY: index `i` came from `fetch_add`, so this core is the
        // only one that has it, and nothing reads the slots until
        // `done` is observed.
        unsafe { *(*s.res.add(i)).get() = d };
    }
    s.busy_us[core].store(busy as i32, Ordering::Relaxed);
    s.took[core].store(took, Ordering::Relaxed);
    for (slot, us) in s.stage_us[core].iter().zip(stage) {
        slot.store(us as i32, Ordering::Relaxed);
    }
}

extern "C" fn par_worker(arg: *mut core::ffi::c_void) {
    // SAFETY: `decode_slot` waits for `done` before its frame goes
    // away, so the reference is live for the whole call.
    let s: &ParShared = unsafe { &*(arg as *const ParShared) };
    run_candidates(s);
    let hw = unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) };
    s.stack_hw_bytes.store(hw, Ordering::Relaxed);
    s.done.store(true, Ordering::Release);
    unsafe { esp_idf_svc::sys::vTaskDelete(core::ptr::null_mut()) };
}

/// Charges the tail's time on whichever way the tail ends. It ends at
/// a `?` as often as it ends at a decode, and a timer that only ran to
/// the end would report the successes and drop the failures — which
/// are the expensive ones.
struct TailTimer<'a> {
    stage: &'a mut [i64; 3],
    t_tail: i64,
}
impl Drop for TailTimer<'_> {
    fn drop(&mut self) {
        self.stage[2] += now_us() - self.t_tail;
    }
}

/// One candidate, end to end: half-rate DDC, RMS normalise, narrowed
/// Δt search, decode. No shared mutable state beyond the global FFT
/// planner's own guard, which is what lets two cores run it at once.
///
/// `pipe` is the baseband [`EarlyBasebands`] built for this carrier
/// during capture, when there is one: only the part of the window it
/// has not seen yet is left to filter.
fn decode_candidate(
    half: &[f32],
    cand: &SyncCandidate,
    refs: &Ft4CoarsePhasors,
    pipe: Option<&mut Pipe>,
    scratch: &mut Option<Ft4SweepScratch>,
    stage: &mut [i64; 3],
) -> Option<Ft4Decode> {
    let t_ddc = now_us();
    // A candidate built during capture: its baseband needs its last few
    // percent and its flush, and its coarse sweep whatever cells were
    // not yet readable — against the raw baseband, the one the rest of
    // the sweep scored. Then normalise in place and run the fine pass,
    // which is where the score and position the decoder uses come from.
    if let Some(p) = pipe {
        let mut cd0 = p.finish(half);
        let t1 = now_us();
        // Internal DRAM — see `early_worker`.
        let scratch = scratch.get_or_insert_with(Ft4SweepScratch::new::<Ft4>);
        let s2 = match p.sweep.take() {
            Some(mut sw) => {
                sw.complete::<Ft4>(&cd0, scratch, refs);
                let t2 = now_us();
                rms_normalise(&mut cd0);
                let t3 = now_us();
                let s2 = sw.fine::<Ft4>(&cd0, cand, scratch, refs);
                stage[0] += (t1 - t_ddc) + (t3 - t2);
                stage[1] += (t2 - t1) + (now_us() - t3);
                s2
            }
            None => {
                rms_normalise(&mut cd0);
                let t2 = now_us();
                let s2 = ft4_sync_search_window_with::<Ft4>(
                    &cd0,
                    cand,
                    WSJTX_WINDOW.0,
                    WSJTX_WINDOW.1,
                    refs,
                );
                stage[0] += t2 - t_ddc;
                stage[1] += now_us() - t2;
                s2
            }
        };
        let t_tail = now_us();
        let _tail = TailTimer { stage, t_tail };
        return tail_decode(cand, cd0, s2);
    }
    // `MFSK_FT4_BOXCAR=1` swaps the 101 + 263-tap chain for a mix and
    // a nine-sample boxcar. Measured on the host it costs 0.29 dB of
    // threshold alone and 0.50 dB against a +20 dB neighbour folding
    // onto the band, and takes the golden's eleven decodes to ten —
    // for a seventh of the time *there*, on a machine with no PIE dot
    // product. This knob is what turns that into a number from the
    // board, which is the only one the budget can be spent against.
    let mut cd0 = if option_env!("MFSK_FT4_BOXCAR").is_some() {
        candidate_baseband_boxcar(half, cand.freq_hz)
    } else {
        candidate_baseband_half(half, cand.freq_hz)
    };
    rms_normalise(&mut cd0);
    let t_search = now_us();
    stage[0] += t_search - t_ddc;
    // `MFSK_FT4_BINNED_SEARCH=1` scores the coarse sweep over
    // tone-demodulated bins: a quarter of the multiply-adds, and a
    // different shape of loop. Whether a quarter of the arithmetic is
    // a quarter of the time is exactly what the shipped path makes
    // doubtful — it runs its dots through `dsps_dotprod_f32_aes3` at
    // 2.18 cycles per multiply-add (§47), which a shorter, less
    // predictably aligned inner product will not match.
    let s2 = if option_env!("MFSK_FT4_BINNED_SEARCH").is_some() {
        ft4_sync_search_window_binned::<Ft4>(&cd0, cand, WSJTX_WINDOW.0, WSJTX_WINDOW.1, refs)
    } else {
        ft4_sync_search_window_with::<Ft4>(&cd0, cand, WSJTX_WINDOW.0, WSJTX_WINDOW.1, refs)
    };
    let t_tail = now_us();
    stage[1] += t_tail - t_search;
    let _tail = TailTimer { stage, t_tail };
    tail_decode(cand, cd0, s2)
}

/// LLR, BP, unpack — everything after the Δt/Δf search.
fn tail_decode(
    cand: &SyncCandidate,
    cd0: Vec<Complex<f32>>,
    s2: mfsk_core::engine::sync2d::Sync2dResult,
) -> Option<Ft4Decode> {
    let r = process_candidate_precomputed::<Ft4>(
        cand,
        &[],
        &FT4_DOWNSAMPLE,
        DecodeDepth::EMBEDDED,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        SYNC_Q_MIN,
        (cd0, s2.freq_hz, s2.i0, s2.score),
        false,
        false,
    )?;
    let m77: [u8; 77] = *r.message77();
    // Still unpacked here, because a payload that cannot be rendered
    // is not a decode and this is where that is decided.
    let text = unpack77(&m77)?;
    Some(Ft4Decode {
        msg: text,
        msg77: m77,
        freq_hz: r.freq_hz,
        dt_sec: r.dt_sec,
        snr_db: r.snr_db,
        hard_errors: r.hard_errors,
    })
}

pub fn decode_slot(slot: &CapturedSlot, budget_ms: i64) -> SlotOutcome {
    decode_slot_with(slot, budget_ms, None)
}

/// [`decode_slot`], taking over whatever basebands `early` built during
/// capture.
///
/// Every candidate's carrier is snapped to its periodogram bin whether
/// or not a baseband was built for it — see [`snap_to_bin`] — so a
/// candidate decodes the same way on either path, and the host mirror
/// that pins this (`pipelined_ddc_decodes_exactly_what_the_snapped_receiver_does`)
/// describes both.
pub fn decode_slot_with(
    slot: &CapturedSlot,
    budget_ms: i64,
    early: Option<EarlyBasebands>,
) -> SlotOutcome {
    let deadline = slot.closed_us + budget_ms * 1_000;
    // Before anything else: the worker is on core 1, where the
    // candidate worker is about to go, and it lets go within a chunk.
    let early_wait_us = early.as_ref().map(|e| e.join()).unwrap_or(0);
    let early = early.filter(|e| {
        let ours = Arc::ptr_eq(&e.shared.stream, &slot.half);
        if !ours {
            // A re-anchor threw the window it was reading away.
            log::info!("ft4_rx: early basebands were for a discarded window — not used");
        }
        ours
    });
    let half = slot.half();
    let cands: Vec<SyncCandidate> = ft4_coarse_sync_from_savg(
        &slot.savg,
        FREQ_MIN_HZ,
        FREQ_MAX_HZ,
        SYNC_MIN,
        None,
        MAX_CAND,
    )
    .into_iter()
    .map(|c| SyncCandidate {
        freq_hz: snap_to_bin(c.freq_hz),
        ..c
    })
    .collect();

    // SAFETY: the worker has set `done` (`join` above) and never
    // touches the pipes again; nothing else has them.
    let mut no_pipes: Vec<Pipe> = Vec::new();
    let pipes: &mut Vec<Pipe> = match &early {
        Some(e) => unsafe { &mut *e.shared.pipes.get() },
        None => &mut no_pipes,
    };
    // How far the workers got, before the decode takes the rest.
    let early_fed: usize = pipes.iter().map(|p| p.fed).sum();
    let (sweep_done, sweep_total) = pipes
        .iter()
        .filter_map(|p| p.sweep.as_ref().map(|s| s.progress()))
        .fold((0, 0), |(a, b), (d, t)| (a + d, b + t));
    let mut used = alloc::vec![false; pipes.len()];
    let pipe_of: Vec<Option<usize>> = cands
        .iter()
        .map(|c| {
            let k = pipes.iter().position(|p| p.bin == bin_of(c.freq_hz))?;
            if used[k] {
                return None;
            }
            used[k] = true;
            Some(k)
        })
        .collect();
    let reused = pipe_of.iter().filter(|k| k.is_some()).count();

    // The shared half of the front end costs nothing here: `SlotAccum`
    // decimated the window while it was arriving. `ft4::ddc`'s stage A
    // used to filter all 90 000 samples per candidate at 12 kHz
    // (61.0 ms) and now sees 6.25 s of it at 6 kHz with half the taps
    // (30.8 ms), with the ÷2 that makes that possible off the
    // post-window budget entirely (`docs/notes/FT4_BENCHMARK.md` §37,
    // §42).
    // Built once for the slot: the coarse sweep's nine references do
    // not depend on `cd0` or on the candidate, and rebuilding them per
    // candidate was 30 % of the Δt search (25.7 ms of 86.7 measured on
    // a CoreS3, `docs/notes/FT4_BENCHMARK.md` §47).
    let coarse_refs = Ft4CoarsePhasors::new::<Ft4>();
    let res: Vec<UnsafeCell<Option<Ft4Decode>>> =
        (0..cands.len()).map(|_| UnsafeCell::new(None)).collect();
    let done_at: Vec<AtomicI32> = (0..cands.len()).map(|_| AtomicI32::new(-1)).collect();
    let shared = ParShared {
        half: half.as_ptr(),
        half_len: half.len(),
        cands: cands.as_ptr(),
        n: cands.len(),
        pipe_of: pipe_of.as_ptr(),
        pipes: pipes.as_mut_ptr(),
        next: AtomicUsize::new(0),
        deadline,
        res: res.as_ptr(),
        done_at_us: done_at.as_ptr(),
        closed_us: slot.closed_us,
        refs: &coarse_refs,
        done: AtomicBool::new(false),
        stack_hw_bytes: AtomicU32::new(0),
        busy_us: [AtomicI32::new(0), AtomicI32::new(0)],
        took: [AtomicUsize::new(0), AtomicUsize::new(0)],
        stage_us: [
            [AtomicI32::new(0), AtomicI32::new(0), AtomicI32::new(0)],
            [AtomicI32::new(0), AtomicI32::new(0), AtomicI32::new(0)],
        ],
    };

    // Core 1 takes candidates from the same cursor this core does.
    // Measured on the golden's 12: 1 923 ms on one core, 1 367 on two
    // (1.40x, `ft4-bench`'s dual-core probe), which is what brings the
    // whole list inside `TX_TURNAROUND_BUDGET_MS`. §31.1 had measured
    // the same probe at 1.17x and called dual-core closed; the shared
    // decimation is what changed that, by taking most of the PSRAM
    // streaming out of the per-candidate half.
    // `MFSK_FT4_SINGLE_CORE=1` runs the same loop on one core, which is
    // the only honest baseline for the two-core numbers: it is this
    // binary, this cursor and this cached Δt path, with the worker not
    // created. A serial figure taken from a bench that calls a
    // different entry point cannot be divided into a parallel one.
    let want_two_cores = option_env!("MFSK_FT4_SINGLE_CORE").is_none();
    let created = want_two_cores
        && unsafe {
            esp_idf_svc::sys::xTaskCreatePinnedToCore(
                Some(par_worker),
                c"ft4_cand".as_ptr(),
                WORKER_STACK,
                &shared as *const ParShared as *mut core::ffi::c_void,
                5,
                core::ptr::null_mut(),
                1,
            )
        } == 1;
    if want_two_cores && !created {
        // Not fatal, and not silent: this is the internal-DRAM
        // constraint biting, and the receiver keeps working at the
        // single-core rate with a shorter candidate list.
        log::warn!("ft4_rx: core-1 worker not created ({WORKER_STACK} B stack) — decoding on one core");
    }

    let loop_t0 = now_us();
    let fft0 = crate::esp_dsp_fft::fc32_table_stats();
    let dot0 = crate::esp_dsp_dotprod::dotprod_path_report();
    run_candidates(&shared);

    if created {
        // The frame this `ParShared` lives in must outlive the worker.
        while !shared.done.load(Ordering::Acquire) {
            unsafe { esp_idf_svc::sys::vTaskDelay(1) };
        }
    }

    let loop_us = now_us() - loop_t0;
    let started = shared.next.load(Ordering::Acquire).min(cands.len());
    let cut_at_score = cands.get(started).map(|c| c.score);

    // Back into candidate order — descending coarse score, the order a
    // single core produced and the screen expects — and dedup there,
    // because two candidates can land on one signal and which core
    // finished first must not decide which copy survives.
    let mut out: Vec<Ft4Decode> = Vec::new();
    // Each message's earliest completion: when two candidates land on
    // one signal, the reply could have used whichever finished first,
    // so that is the one "in time" is judged by.
    let mut first_at: Vec<i64> = Vec::new();
    for (i, cell) in res.iter().enumerate() {
        // SAFETY: both cores are finished — the worker through `done`,
        // this one by returning from `run_candidates`.
        let Some(d) = (unsafe { (*cell.get()).take() }) else {
            continue;
        };
        let at = done_at[i].load(Ordering::Relaxed) as i64;
        if let Some(k) = out.iter().position(|k| k.msg == d.msg) {
            first_at[k] = first_at[k].min(at);
            continue;
        }
        out.push(d);
        first_at.push(at);
    }
    let deadline_us = REPLY_DEADLINE_MS * 1_000;
    let intime = first_at.iter().filter(|&&t| t <= deadline_us).count();
    let last_decode_ms = first_at.iter().copied().max().unwrap_or(0) / 1_000;

    // Where a second core's missing speed-up actually goes.
    // `occ` is the two cores' busy time over twice the loop's wall
    // clock: near 100 % means both cores worked the whole loop and
    // whatever was lost was lost *inside* a candidate; well under
    // means a core was idle, and `took` says whether that was the
    // hand-out or the deadline. The per-stage split says which stage
    // pays — see `ParShared::stage_us`.
    let b0 = shared.busy_us[0].load(Ordering::Relaxed) as i64;
    let b1 = shared.busy_us[1].load(Ordering::Relaxed) as i64;
    let t0 = shared.took[0].load(Ordering::Relaxed);
    let t1 = shared.took[1].load(Ordering::Relaxed);
    let cores = if created { 2 } else { 1 };
    let occ = if loop_us > 0 {
        (b0 + b1) * 100 / (cores * loop_us)
    } else {
        0
    };
    let st = |k: usize| {
        (shared.stage_us[0][k].load(Ordering::Relaxed) as i64
            + shared.stage_us[1][k].load(Ordering::Relaxed) as i64)
            / 1000
    };
    let n = (t0 + t1).max(1) as i64;
    // Both should read zero every slot after the first: a table is
    // built once per length, and FT4's decode path takes no lock at
    // all (the only `Fc32Guard` left guards `MIXED_SCRATCH`, and the
    // coarse transform on the audio task is its only user here).
    let fft1 = crate::esp_dsp_fft::fc32_table_stats();
    // The Δt search's dot products take esp-dsp's PIE path only when
    // both operands are 16-byte aligned and the length is a multiple
    // of four. §32.1 measured what losing that costs — 1 049 -> 1 789
    // ms — from an allocation elsewhere shifting the heap, so this
    // belongs next to the timing rather than in a separate probe.
    let dot1 = crate::esp_dsp_dotprod::dotprod_path_report();
    let dot_fast = dot1.0 - dot0.0;
    let dot_slow = (dot1.1 - dot0.1) + (dot1.2 - dot0.2);
    let dot_pct = dot_fast * 100 / (dot_fast + dot_slow).max(1);
    let fft_new = fft1.0 - fft0.0;
    let fft_wait = fft1.1 - fft0.1;
    log::info!(
        "ft4_rx: cores — loop {} ms | core0 {} ms/{t0} cand | core1 {} ms/{t1} cand | occ {occ}% \
         | per cand {} ms = ddc {} + search {} + tail {} | fft tables +{fft_new} waits {fft_wait} | dot PIE {dot_pct}% of {}",
        loop_us / 1000,
        b0 / 1000,
        b1 / 1000,
        (b0 + b1) / 1000 / n,
        st(0) / n,
        st(1) / n,
        st(2) / n,
        dot_fast + dot_slow,
    );

    // How far short of the reply boundary this slot is. `intime` is the
    // FT8 side's figure of the same name: decodes that finished before
    // the boundary, i.e. that a transmitter could have answered.
    // Every distinct decode's completion, sorted, so any deadline can be
    // read off the same capture — the boundary, the audio start and the
    // current cut side by side — rather than re-flashing to move one.
    let mut done_ms: Vec<i64> = first_at.iter().map(|t| t / 1_000).collect();
    done_ms.sort_unstable();
    let by = |ms: i64| done_ms.iter().filter(|&&t| t <= ms).count();
    log::info!(
        "ft4_rx: reply deadline {REPLY_DEADLINE_MS} ms — intime {intime} of {} decodes \
         | by {SLOT_BOUNDARY_MS}: {} by {REPLY_DEADLINE_MS}: {} by {TX_TURNAROUND_BUDGET_MS}: {} \
         | loop ends {} ms | done {:?}",
        out.len(),
        by(SLOT_BOUNDARY_MS),
        by(REPLY_DEADLINE_MS),
        by(TX_TURNAROUND_BUDGET_MS),
        (loop_t0 - slot.closed_us + loop_us) / 1_000,
        done_ms,
    );

    if let Some(e) = &early {
        let internal = unsafe {
            esp_idf_svc::sys::heap_caps_get_largest_free_block(esp_idf_svc::sys::MALLOC_CAP_INTERNAL)
        };
        // `fed` is how much of the DDC work was done before the close:
        // the pipes' samples over what the reused ones needed in all.
        log::info!(
            "ft4_rx: early DDC — {} built from {} ms before close ({} workers), reused {reused} fresh {} wasted {} \
             | {}% fed, sweep {}% at close | wait {} us | stack {:?} B free of {EARLY_STACK} | internal largest {internal} B",
            pipes.len(),
            (slot.closed_us - e.shared.started_us) / 1_000,
            e.workers,
            cands.len() - reused,
            used.iter().filter(|u| !**u).count(),
            early_fed * 100 / (pipes.len() * half.len()).max(1),
            sweep_done * 100 / sweep_total.max(1),
            early_wait_us,
            [
                e.shared.stack_hw_bytes[0].load(Ordering::Relaxed),
                e.shared.stack_hw_bytes[1].load(Ordering::Relaxed),
            ],
        );
    }

    if created {
        let stack_hw = shared.stack_hw_bytes.load(Ordering::Relaxed);
        // At info, not debug: this is the number that justifies
        // `WORKER_STACK`, and a size chosen from a measurement should
        // keep reporting whether it is still true.
        //
        // **Bytes, not words.** ESP-IDF's `uxTaskGetStackHighWaterMark`
        // returns bytes, unlike vanilla FreeRTOS where it is words; the
        // first version of this line multiplied by four and printed
        // "14624 B free of 8192", which is its own proof that the unit
        // was wrong.
        log::info!("ft4_rx: core-1 worker stack {stack_hw} B free of {WORKER_STACK}");
    }

    SlotOutcome {
        decodes: out,
        cands: cands.len(),
        tried: started,
        elapsed_us: now_us() - slot.closed_us,
        cut_at_score,
        intime,
        last_decode_ms,
        busy_us: [
            shared.busy_us[0].load(Ordering::Relaxed) as i64,
            shared.busy_us[1].load(Ordering::Relaxed) as i64,
        ],
        took: [
            shared.took[0].load(Ordering::Relaxed),
            shared.took[1].load(Ordering::Relaxed),
        ],
        loop_us,
    }
}

/// Unit-power normalisation, matching what
/// `process_candidate_basic_impl` applies to its own
/// `downsample_cached` output (WSJT-X `ft4_decode.f90:231-232`).
/// `candidate_baseband` deliberately does not do it, so every caller
/// must — `compute_llr`'s `LLR_SCALE` is calibrated against unit-RMS
/// input.
fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}
