// SPDX-License-Identifier: GPL-3.0-or-later
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

use alloc::string::String;
use alloc::vec::Vec;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::{ft4_coarse_sync_from_savg, Ft4SavgBuilder};
use mfsk_core::engine::pipeline::{process_candidate_precomputed, DecodeDepth, DecodeStrictness};
use mfsk_core::engine::sync2d::ft4_sync_search_window;
use mfsk_core::ft4::ddc::{SlotDecimator, candidate_baseband_half};
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::ft4::Ft4;
use mfsk_core::msg::wsjt77::unpack77;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
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
/// 1.25 s of the slot is audio no candidate can reach: the search tops
/// out at `i0 = 1012` and a frame is 105 x 32 = 3 360 downsampled
/// samples, so 4 372 of a slot's 5 000.
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
    /// a `FirStage` carries its own history.
    pub half: Vec<f32>,
    /// `esp_timer` microseconds at the moment the slot closed, so the
    /// decoder can report post-slot latency the way the benches do.
    pub closed_us: i64,
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
/// to re-run anything at 12 kHz needs it, and the waterfall and the
/// replay path both read it.
pub struct SlotAccum {
    audio: Vec<i16>,
    savg: Ft4SavgBuilder,
    decim: SlotDecimator,
    half: Vec<f32>,
    /// Samples still to be discarded before the next slot's window
    /// opens: the slot grid is 7.5 s and the window is 6.25 s, so the
    /// tail no candidate can reach is dropped rather than accumulated.
    /// This is what keeps the grid a *slot* grid after the early
    /// close — without it each window would start 1.25 s earlier than
    /// the last and walk off the transmissions entirely.
    skip: usize,
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
            // Half the window; the stage's group delay is what makes
            // it a little short of exactly half.
            half: Vec::with_capacity(CAPTURE_CLOSE_SAMPLES / 2),
            skip: 0,
        }
    }

    /// Feed the next block. Returns the finished slot on the block that
    /// closes its window, and resets for the next.
    ///
    /// A block straddling the boundary is split, so the caller's block
    /// size never shifts the slot grid — which matters because a UAC
    /// read is not a divisor of 75 000 either.
    ///
    /// The window closes at [`CAPTURE_CLOSE_SAMPLES`] — 6.25 s of a
    /// 7.5 s slot — and the remaining 1.25 s is discarded, because a
    /// QSO-capable receiver has to have answered by 8.0 s and no
    /// candidate can read that audio anyway.
    pub fn push(&mut self, samples: &[i16]) -> Option<CapturedSlot> {
        self.push_with_rows(samples, &mut |_| {})
    }

    /// [`push`](Self::push), forwarding each completed spectrum row to
    /// `on_row` — see [`Ft4SavgBuilder::push_with_rows`]. This is how a
    /// waterfall gets 152 rows a slot without a transform of its own;
    /// [`wf_row`] turns one into palette indices.
    pub fn push_with_rows(
        &mut self,
        samples: &[i16],
        on_row: &mut dyn FnMut(&[f32]),
    ) -> Option<CapturedSlot> {
        let mut rest = samples;
        let mut done = None;
        while !rest.is_empty() {
            if self.skip > 0 {
                let take = self.skip.min(rest.len());
                self.skip -= take;
                rest = &rest[take..];
                continue;
            }
            let room = CAPTURE_CLOSE_SAMPLES - self.audio.len();
            let take = room.min(rest.len());
            self.audio.extend_from_slice(&rest[..take]);
            self.savg.push_with_rows(&rest[..take], on_row);
            self.decim.push_i16(&rest[..take], &mut self.half);
            rest = &rest[take..];
            if self.audio.len() == CAPTURE_CLOSE_SAMPLES {
                let prev = core::mem::replace(self, Self::new());
                self.skip = SLOT_SAMPLES - CAPTURE_CLOSE_SAMPLES;
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

/// One decoded transmission.
pub struct Ft4Decode {
    pub msg: String,
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
pub const TX_TURNAROUND_BUDGET_MS: i64 = 1_225;

/// A whole FT4 slot, so a receive-only monitor can spend one.
///
/// Nothing is lost by overrunning [`TX_TURNAROUND_BUDGET_MS`] unless
/// the radio has to key up: slot `N + 1`'s audio is accumulating on
/// the capture side while slot `N` decodes, and only running past
/// *this* costs a slot.
///
/// Conservative by 1.25 s since the early close: the next window
/// actually opens at `7.5 + 6.25` after this one did. Left at a slot
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
    next: AtomicUsize,
    deadline: i64,
    /// One slot per candidate, written by whichever core took that
    /// index. No lock: the cursor hands out each index exactly once,
    /// so the writes are disjoint, and `done`'s Release/Acquire pair
    /// publishes them all at the join.
    res: *const UnsafeCell<Option<Ft4Decode>>,
    done: AtomicBool,
    stack_hw_bytes: AtomicU32,
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
    loop {
        if now_us() >= s.deadline {
            return;
        }
        let i = s.next.fetch_add(1, Ordering::AcqRel);
        if i >= cands.len() {
            return;
        }
        let d = decode_candidate(half, &cands[i]);
        // SAFETY: index `i` came from `fetch_add`, so this core is the
        // only one that has it, and nothing reads the slots until
        // `done` is observed.
        unsafe { *(*s.res.add(i)).get() = d };
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

/// One candidate, end to end: half-rate DDC, RMS normalise, narrowed
/// Δt search, decode. No shared mutable state beyond the global FFT
/// planner's own guard, which is what lets two cores run it at once.
fn decode_candidate(half: &[f32], cand: &SyncCandidate) -> Option<Ft4Decode> {
    let mut cd0 = candidate_baseband_half(half, cand.freq_hz);
    rms_normalise(&mut cd0);
    let s2 = ft4_sync_search_window::<Ft4>(&cd0, cand, WSJTX_WINDOW.0, WSJTX_WINDOW.1);
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
    let text = r
        .message77()
        .try_into()
        .ok()
        .and_then(|m77: &[u8; 77]| unpack77(m77))?;
    Some(Ft4Decode {
        msg: text,
        freq_hz: r.freq_hz,
        dt_sec: r.dt_sec,
        snr_db: r.snr_db,
        hard_errors: r.hard_errors,
    })
}

pub fn decode_slot(slot: &CapturedSlot, budget_ms: i64) -> SlotOutcome {
    let deadline = slot.closed_us + budget_ms * 1_000;
    let cands = ft4_coarse_sync_from_savg(
        &slot.savg,
        FREQ_MIN_HZ,
        FREQ_MAX_HZ,
        SYNC_MIN,
        None,
        MAX_CAND,
    );

    // The shared half of the front end costs nothing here: `SlotAccum`
    // decimated the window while it was arriving. `ft4::ddc`'s stage A
    // used to filter all 90 000 samples per candidate at 12 kHz
    // (61.0 ms) and now sees 6.25 s of it at 6 kHz with half the taps
    // (30.8 ms), with the ÷2 that makes that possible off the
    // post-window budget entirely (`docs/notes/FT4_BENCHMARK.md` §37,
    // §42).
    let res: Vec<UnsafeCell<Option<Ft4Decode>>> =
        (0..cands.len()).map(|_| UnsafeCell::new(None)).collect();
    let shared = ParShared {
        half: slot.half.as_ptr(),
        half_len: slot.half.len(),
        cands: cands.as_ptr(),
        n: cands.len(),
        next: AtomicUsize::new(0),
        deadline,
        res: res.as_ptr(),
        done: AtomicBool::new(false),
        stack_hw_bytes: AtomicU32::new(0),
    };

    // Core 1 takes candidates from the same cursor this core does.
    // Measured on the golden's 12: 1 923 ms on one core, 1 367 on two
    // (1.40x, `ft4-bench`'s dual-core probe), which is what brings the
    // whole list inside `TX_TURNAROUND_BUDGET_MS`. §31.1 had measured
    // the same probe at 1.17x and called dual-core closed; the shared
    // decimation is what changed that, by taking most of the PSRAM
    // streaming out of the per-candidate half.
    let created = unsafe {
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
    if !created {
        // Not fatal, and not silent: this is the internal-DRAM
        // constraint biting, and the receiver keeps working at the
        // single-core rate with a shorter candidate list.
        log::warn!("ft4_rx: core-1 worker not created ({WORKER_STACK} B stack) — decoding on one core");
    }

    run_candidates(&shared);

    if created {
        // The frame this `ParShared` lives in must outlive the worker.
        while !shared.done.load(Ordering::Acquire) {
            unsafe { esp_idf_svc::sys::vTaskDelay(1) };
        }
    }

    let started = shared.next.load(Ordering::Acquire).min(cands.len());
    let cut_at_score = cands.get(started).map(|c| c.score);

    // Back into candidate order — descending coarse score, the order a
    // single core produced and the screen expects — and dedup there,
    // because two candidates can land on one signal and which core
    // finished first must not decide which copy survives.
    let mut out: Vec<Ft4Decode> = Vec::new();
    for cell in &res {
        // SAFETY: both cores are finished — the worker through `done`,
        // this one by returning from `run_candidates`.
        let Some(d) = (unsafe { (*cell.get()).take() }) else {
            continue;
        };
        if out.iter().any(|k| k.msg == d.msg) {
            continue;
        }
        out.push(d);
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
    }
}

/// Frequency span the waterfall covers, matching `ui::waterfall`'s
/// own `WF_FREQ_LO_HZ`/`HI` and FT8's row builder — the panel is
/// shared, so the axis has to be.
pub const WF_FREQ_LO_HZ: f32 = 200.0;
/// See [`WF_FREQ_LO_HZ`].
pub const WF_FREQ_HI_HZ: f32 = 2_700.0;

/// Bin spacing of a row from [`Ft4SavgBuilder`]: `12 000 / 2304`.
const ROW_DF_HZ: f32 = 12_000.0 / 2_304.0;

/// Turn one row's power spectrum into [`crate::pipeline::WF_ROW_LEN`]
/// palette indices, 0..15.
///
/// Rows come from [`Ft4SavgBuilder::push_with_rows`], which hands over
/// the spectra it is already averaging — so the *transforms* are free.
/// This mapping is not, and it runs on the capture thread: 152 rows a
/// slot against §33's 21.3 ms per-block budget.
///
/// **Integer log2, not `log10`.** The first version took
/// `10 * log10(p)` per column and measured **623 µs per row, 94 ms a
/// slot, 10 % of the coarse stage** — display-only work costing a
/// tenth of the stage it rides on, which is not a reasonable price for
/// a picture. `f32::log10` is ~620 cycles on this core and there are
/// 240 of them per row.
///
/// So the level comes from the exponent instead, the way FT8's own
/// `decimate_pair_to_wf` has always done it: scale the power by the
/// row's mean, take the bit position of the MSB, and keep one
/// fractional bit for half-octave resolution. One float multiply per
/// column and the rest is integer.
///
/// The palette is 16 coarse steps over [`WF_SPAN_DB`], so quantising
/// the level to half-octaves (~1.5 dB) is below what it can show.
///
/// **Per-column maximum, not mean.** A column is ~2 bins wide
/// (2 500 Hz over 240 columns is 10.4 Hz, against 5.2 Hz bins) and an
/// FT4 tone is narrower than that, so averaging would halve every
/// signal against its neighbouring noise bin while leaving the noise
/// floor alone — visible as a waterfall where the signals are dimmer
/// than the background is bright.
///
/// **The scale is per-row and relative**, against the row's own mean
/// power. An absolute scale would need a calibrated input level, which
/// a receiver taking whatever a radio's USB audio hands it does not
/// have; this shows the band the way an operator reads one, relative
/// to its own noise.
pub fn wf_row(spectrum: &[f32]) -> [u8; crate::pipeline::WF_ROW_LEN] {
    const N: usize = crate::pipeline::WF_ROW_LEN;
    /// `1.0` in the fixed-point ratio below, i.e. `2^SCALE_LOG2`.
    const SCALE_LOG2: u32 = 10;
    /// Half-octaves the palette spans. [`WF_SPAN_DB`] dB of *power* is
    /// `WF_SPAN_DB / 3.01` octaves, doubled.
    const HALF_OCTAVES: u32 = 20;
    /// First column's low bin, and the bin step per column — the whole
    /// frequency mapping, folded into two constants so the loop is an
    /// add and two truncations rather than four divides.
    const BIN0: f32 = WF_FREQ_LO_HZ / ROW_DF_HZ;
    /// See [`BIN0`].
    const BIN_STEP: f32 = (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ) / (N as f32) / ROW_DF_HZ;

    let mut out = [0u8; N];
    if spectrum.is_empty() {
        return out;
    }
    let mean: f32 = spectrum.iter().sum::<f32>() / spectrum.len() as f32;
    // A silent input (all zero) has no scale to speak of; leave the
    // row black rather than dividing by it.
    if !(mean > 0.0) {
        return out;
    }
    // One divide per row, not per column.
    let inv = (1u32 << SCALE_LOG2) as f32 / mean;

    // Column -> bin range, walked rather than computed. The mapping is
    // fixed — it depends on nothing but the constants — yet the first
    // version recomputed it per row with four float divides per
    // column, 960 a row. Dropping `log10` only took 623 us/row to 384;
    // this is the rest of it.
    let mut edge = BIN0;

    for cell in out.iter_mut() {
        let lo = edge as usize;
        edge += BIN_STEP;
        let hi = ((edge as usize) + 1).min(spectrum.len());
        if hi <= lo {
            continue;
        }
        let mut peak = 0.0f32;
        for &p in &spectrum[lo..hi] {
            if p > peak {
                peak = p;
            }
        }
        // Ratio to the row's mean, in units of `2^SCALE_LOG2`.
        let scaled = peak * inv;
        if !(scaled >= 1.0) {
            continue;
        }
        let q = scaled as u32;
        // `31 - leading_zeros` is floor(log2), and the bit below the
        // MSB is the half-octave.
        let e = 31 - q.leading_zeros();
        let frac = if e > 0 { (q >> (e - 1)) & 1 } else { 0 };
        let half_oct = (e * 2 + frac).saturating_sub(SCALE_LOG2 * 2);
        *cell = ((half_oct * 15) / HALF_OCTAVES).min(15) as u8;
    }
    out
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
