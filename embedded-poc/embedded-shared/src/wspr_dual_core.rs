//! Dual-core dispatch for the WSPR candidate loop (issue #260 Phase 2+).
//!
//! Two independent worker lifecycles, not one persistent task like
//! FT8's `dual_core.rs`: WSPR's per-candidate stack need is an order
//! of magnitude bigger than FT8's (100+ KB vs 16 KB) and differs
//! sharply between pass 0/1 (never reach OSD — no confirmed-callsign
//! table exists yet, so `decode_at_baseband`'s `.and_then` gate on
//! `osd_decode_packed` short-circuits — ~93-103 KB peak) and pass 2
//! (does reach it, ~112 KB peak). A single worker sized for the worst
//! case across all three passes and kept alive the whole scan doesn't
//! fit next to `wspr_scan`'s own similarly-sized stack inside this
//! board's ~236 KB largest contiguous free block — that combination
//! is exactly what crashed the first dual-core attempt (see
//! `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`, "Dual-core").
//! So each phase gets its own worker, spawned right before it's
//! needed and torn down right after, and only one such worker is ever
//! alive at a time.
//!
//! ## Pass 0/1 — work-steal
//!
//! Same `AtomicUsize::fetch_add` dispatch as FT8's `dual_core.rs`
//! `Stage3WorkSteal`: per-candidate cost is bimodal (a
//! `minsync1`-rejected or quickly-converged candidate finishes in
//! under a second; the rare one that runs the full Fano ladder and
//! fails takes 10+ seconds), so a static half/half split would strand
//! whichever core drew the slow candidate. 13-14 candidates per pass
//! makes dynamic dispatch worth its (negligible — one shared
//! `AtomicUsize`) bookkeeping cost.
//!
//! ## Pass 2 — static 1-1 split
//!
//! `wspr::decode::PASS2_DEEP_LADDER_TOP_N` (2) already caps pass 2's
//! deep-ladder candidates at exactly the core count, so there's
//! nothing left to dynamically balance: main takes the rank-0
//! survivor, the worker takes rank-1. No queue, no atomic — just two
//! function calls.
//!
//! ## Safety guard
//!
//! Every worker spawn is preceded by a live
//! `heap_caps_get_largest_free_block` check against the stack it's
//! about to request. If the block isn't there, dual-core is skipped
//! for that phase and the candidates run sequentially on `wspr_scan`
//! instead — a slower pass, not a crash. The stack sizes below are
//! sized off real device measurements, but this crate's own stack
//! audit left ~8 KB of pass 2's growth structurally unexplained
//! (compiler stack-slot reuse isn't reliably hand-computable) — this
//! guard is what makes shipping on that residual uncertainty
//! acceptable instead of reckless.
//!
//! Workers **self-delete** (`vTaskDelete(NULL)`) after pushing their
//! result, rather than have the caller hold a `TaskHandle_t` and
//! delete it from outside — FreeRTOS's own recommended idiom for
//! finite-lifetime tasks, and it sidesteps any "is the task still
//! mid-cleanup" race. `vTaskDelete` on the *calling* task only marks
//! it for deletion; the **idle task** performs the actual stack-memory
//! free, so callers wait a few ticks after receiving a worker's result
//! before spawning the next (differently-sized) worker, giving the
//! idle task a chance to run first.

use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::boxed::Box;
use alloc::vec::Vec;

use esp_idf_svc::sys::{
    heap_caps_get_largest_free_block, vTaskDelay, vTaskDelete, xQueueGenericCreate,
    xQueueGenericSend, xQueueReceive, xTaskCreatePinnedToCore, QueueHandle_t,
};

use mfsk_core::wspr::coarse_baseband::BasebandCandidate;
use mfsk_core::wspr::decode::{
    decode_at_baseband, deep_decode_pass2_candidate, WsprCallsignTable, WsprPass2Candidate,
    WsprResult,
};

const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;
const APP_CPU: i32 = 1;

/// Stack for the pass-0/1 work-steal worker. 80 KiB = 52.8 KB
/// (measured pass-0/1 cumulative peak, post-`refine_cascade`
/// ping-pong rewrite — see `SCAN_STACK`'s own doc comment) + ~50 %
/// margin. Generous on purpose: after the ping-pong fix the largest
/// contiguous free block has real room to spare (unlike before it,
/// when every KB was accounted for), so there's no reason to run this
/// close to the measured number the way the pre-fix constants had to.
const PASS01_WORKER_STACK: u32 = 81_920;

/// Stack for the pass-2 worker (reaches `osd_decode_packed`). 88 KiB
/// = 61.45 KB (measured pass-0/1/2 cumulative peak, post-ping-pong —
/// 68 148 B headroom on a 128 KiB stack) + ~43 % margin. Same
/// generous-margin reasoning as [`PASS01_WORKER_STACK`].
const PASS2_WORKER_STACK: u32 = 90_112;

/// Bytes required *beyond* the worker's own stack before a spawn is
/// attempted — covers `wspr_scan`'s own transient allocations (job
/// `Box`, queue handles) made around the same time, so the guard
/// doesn't pass on a block that's technically big enough for the
/// stack alone but leaves nothing over for anything else.
const SAFETY_MARGIN_BYTES: usize = 4096;

/// Ticks to wait after a self-deleted worker's result arrives, before
/// spawning the next (differently-sized) worker — see the module doc
/// comment's "Safety guard" section.
const TEARDOWN_DELAY_TICKS: u32 = 5;

fn largest_free_internal() -> usize {
    unsafe { heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) }
}

/// True if a block of `stack_words` (bytes) plus [`SAFETY_MARGIN_BYTES`]
/// is available right now. Call immediately before every worker spawn
/// — the free-block size is a live, moving quantity, not a
/// compile-time guarantee.
fn have_room_for(stack_bytes: u32) -> bool {
    largest_free_internal() >= stack_bytes as usize + SAFETY_MARGIN_BYTES
}

#[inline]
unsafe fn queue_create(item_size: usize) -> QueueHandle_t {
    let q = unsafe { xQueueGenericCreate(1, item_size as u32, QUEUE_TYPE_BASE) };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

#[inline]
unsafe fn queue_send_ptr<T>(q: QueueHandle_t, ptr: *mut T) {
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&ptr as *const *mut T) as *const core::ffi::c_void,
            PORT_MAX_DELAY,
            QUEUE_SEND_TO_BACK,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueGenericSend failed: {r}");
}

#[inline]
unsafe fn queue_recv_ptr<T>(q: QueueHandle_t) -> *mut T {
    let mut out: *mut T = ptr::null_mut();
    let r = unsafe {
        xQueueReceive(
            q,
            (&mut out as *mut *mut T) as *mut core::ffi::c_void,
            PORT_MAX_DELAY,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueReceive failed: {r}");
    out
}

/// Called once at startup (same point FT8's `dual_core::init()` is
/// called from) — creates the two result queues these workers need.
/// Workers themselves are spawned per-phase, not here.
pub fn init() {
    unsafe {
        PASS01_RESULT_Q.set(queue_create(core::mem::size_of::<
            *mut Vec<(WsprResult, usize)>,
        >()));
        PASS2_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Option<WsprResult>>()));
    }
    log::info!("wspr_dual_core: result queues ready");
}

struct QueueCell(core::cell::UnsafeCell<QueueHandle_t>);
unsafe impl Sync for QueueCell {}
impl QueueCell {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(ptr::null_mut()))
    }
    fn get(&self) -> QueueHandle_t {
        unsafe { *self.0.get() }
    }
    /// SAFETY: only call from `init()` before any other access.
    unsafe fn set(&self, q: QueueHandle_t) {
        unsafe { *self.0.get() = q };
    }
}

static PASS01_RESULT_Q: QueueCell = QueueCell::new();
static PASS2_RESULT_Q: QueueCell = QueueCell::new();

// ---------------------------------------------------------------
// Pass 0 / 1 — work-steal
// ---------------------------------------------------------------

struct Pass01Job {
    idat: *const f32,
    idat_len: usize,
    qdat: *const f32,
    qdat_len: usize,
    sample_rate: u32,
    pad: usize,
    slots_ptr: *mut Option<BasebandCandidate>,
    slots_len: usize,
    next_idx: *const AtomicUsize,
}
unsafe impl Send for Pass01Job {}

/// Pop candidates from the shared atomic-indexed slot array and
/// decode them one at a time. Mirrors `wspr_bench`'s own pass-0/1
/// inner loop (`decode_at_baseband` + the same `dt_sec`/`start_sample`/
/// `snr_db` fixups) so dual-core and sequential produce identical
/// per-candidate results — only the dispatch differs.
///
/// SAFETY: `slots_ptr` must point to `slots_len` valid
/// `Option<BasebandCandidate>` cells, and `next_idx` gives each caller
/// an exclusive index per `fetch_add`, so the same slot is never read
/// by both cores.
unsafe fn drain_pass01_queue(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    slots_ptr: *mut Option<BasebandCandidate>,
    slots_len: usize,
    next_idx: &AtomicUsize,
) -> Vec<(WsprResult, usize)> {
    let mut out = Vec::new();
    loop {
        let i = next_idx.fetch_add(1, Ordering::AcqRel);
        if i >= slots_len {
            break;
        }
        // SAFETY: the atomic fetch_add gives this caller exclusive
        // ownership of slot `i` for the duration of this iteration.
        let cand = unsafe { (*slots_ptr.add(i)).take() };
        let Some(c) = cand else { continue };
        if let Some(mut d) = decode_at_baseband(
            idat,
            qdat,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            c.drift_hz,
        ) {
            let start_refined = d.start_sample;
            d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
            d.start_sample = start_refined.saturating_sub(pad);
            d.snr_db = c.snr_db;
            out.push((d, start_refined));
        }
    }
    out
}

extern "C" fn pass01_worker_main(arg: *mut core::ffi::c_void) {
    let job = unsafe { *Box::from_raw(arg as *mut Pass01Job) };
    let idat = unsafe { core::slice::from_raw_parts(job.idat, job.idat_len) };
    let qdat = unsafe { core::slice::from_raw_parts(job.qdat, job.qdat_len) };
    let next_idx = unsafe { &*job.next_idx };
    #[allow(static_mut_refs)]
    let out = unsafe {
        drain_pass01_queue(
            idat,
            qdat,
            job.sample_rate,
            job.pad,
            job.slots_ptr,
            job.slots_len,
            next_idx,
        )
    };
    let raw = Box::into_raw(Box::new(out));
    unsafe { queue_send_ptr(PASS01_RESULT_Q.get(), raw) };
    unsafe { vTaskDelete(ptr::null_mut()) };
}

/// True if [`pass01_split`] is likely to find room. Check before
/// doing the (otherwise wasted) `Vec<Option<_>>` setup — `pass01_split`
/// re-checks internally right before the actual spawn regardless, so
/// this is an optimization, not the safety-critical check.
pub fn pass01_room_available() -> bool {
    have_room_for(PASS01_WORKER_STACK)
}

/// Split pass 0 or pass 1's candidate list across both cores via
/// work-steal. Falls back to sequential (returns `None`) if the
/// runtime guard finds no room — caller should then drain `cands`
/// itself the way `wspr_bench`'s own sequential loop already does.
pub fn pass01_split(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    cands: Vec<BasebandCandidate>,
) -> Option<Vec<(WsprResult, usize)>> {
    if !have_room_for(PASS01_WORKER_STACK) {
        log::warn!(
            "wspr_dual_core: skipping pass0/1 split, largest free block {} B < {} B needed",
            largest_free_internal(),
            PASS01_WORKER_STACK as usize + SAFETY_MARGIN_BYTES,
        );
        return None;
    }

    let mut slots: Vec<Option<BasebandCandidate>> = cands.into_iter().map(Some).collect();
    let next_idx = AtomicUsize::new(0);
    let slots_ptr = slots.as_mut_ptr();
    let slots_len = slots.len();
    let next_idx_ptr: *const AtomicUsize = &next_idx;

    let job = Box::new(Pass01Job {
        idat: idat.as_ptr(),
        idat_len: idat.len(),
        qdat: qdat.as_ptr(),
        qdat_len: qdat.len(),
        sample_rate,
        pad,
        slots_ptr,
        slots_len,
        next_idx: next_idx_ptr,
    });
    let mut handle: esp_idf_svc::sys::TaskHandle_t = ptr::null_mut();
    let r = unsafe {
        xTaskCreatePinnedToCore(
            Some(pass01_worker_main),
            c"wspr_p01".as_ptr(),
            PASS01_WORKER_STACK,
            Box::into_raw(job) as *mut core::ffi::c_void,
            5,
            &mut handle,
            APP_CPU,
        )
    };
    if r != PD_PASS {
        log::error!("wspr_dual_core: xTaskCreatePinnedToCore(wspr_p01) failed: {r}, falling back to sequential");
        // The job Box has already been leaked into the failed call —
        // nothing safe to reclaim it with, but a failed spawn is rare
        // enough (and precedes an early-boot-only phase) that this
        // is an acceptable one-time leak versus a double-free risk.
        return None;
    }

    // Main drains the same queue concurrently.
    #[allow(static_mut_refs)]
    let mut local = unsafe {
        drain_pass01_queue(
            idat,
            qdat,
            sample_rate,
            pad,
            slots_ptr,
            slots_len,
            &next_idx,
        )
    };

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<(WsprResult, usize)>>(PASS01_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };
    drop(slots);

    local.extend(worker);
    unsafe { vTaskDelay(TEARDOWN_DELAY_TICKS) };
    Some(local)
}

// ---------------------------------------------------------------
// Pass 2 — static 1-1 split
// ---------------------------------------------------------------

struct Pass2Job {
    idat: *const f32,
    idat_len: usize,
    qdat: *const f32,
    qdat_len: usize,
    sample_rate: u32,
    pad: usize,
    sync: f32,
    freq: f32,
    lag: i32,
    drift: f32,
    isqs: mfsk_core::wspr::demod::IsQs,
    snr_db: f32,
    confirmed: *const WsprCallsignTable,
    /// Absolute `esp_timer_get_time()` deadline for this candidate's
    /// ladder — passed as a plain `i64` (not a closure) because a
    /// `&dyn Fn` reference can't safely cross the raw-pointer task
    /// boundary this job travels over; the worker builds its own
    /// closure from this value instead. `None` = no cutoff, run the
    /// full wsprd-faithful ladder.
    deadline_us: Option<i64>,
}
unsafe impl Send for Pass2Job {}

extern "C" fn pass2_worker_main(arg: *mut core::ffi::c_void) {
    let t0 = now_us();
    log::info!(
        "wspr_dual_core: pass2 worker started on core {}",
        current_core()
    );
    let job = unsafe { *Box::from_raw(arg as *mut Pass2Job) };
    let idat = unsafe { core::slice::from_raw_parts(job.idat, job.idat_len) };
    let qdat = unsafe { core::slice::from_raw_parts(job.qdat, job.qdat_len) };
    let confirmed = unsafe { &*job.confirmed };
    // `c` needs a `&BasebandCandidate` to carry `snr_db` through —
    // build a throwaway one-field-used candidate rather than widen
    // `WsprPass2Candidate`'s contract for this one caller.
    let stand_in = BasebandCandidate {
        start_sample: 0,
        freq_hz: job.freq,
        drift_hz: job.drift,
        sync: job.sync,
        snr_db: job.snr_db,
    };
    let r = WsprPass2Candidate {
        sync: job.sync,
        c: &stand_in,
        freq: job.freq,
        lag: job.lag,
        drift: job.drift,
        isqs: job.isqs,
    };
    // `worker_budget_check` is a plain `fn` item, not a capturing
    // closure: a closure over `job.deadline_us` can't cross this
    // task's own creation boundary without `Box<dyn Fn>`, so the
    // deadline instead travels as the plain `i64` already on `job`
    // (set into `WORKER_DEADLINE_US` here, just before use) and the
    // checker reads it back. Safe under this module's single-
    // pass-2-worker-at-a-time invariant — see the module doc comment.
    unsafe { WORKER_DEADLINE_US.set(job.deadline_us.unwrap_or(i64::MAX)) };
    let budget: Option<&(dyn Fn() -> bool + Sync)> = if job.deadline_us.is_some() {
        Some(&worker_budget_check)
    } else {
        None
    };
    let out =
        deep_decode_pass2_candidate(idat, qdat, job.sample_rate, job.pad, confirmed, &r, budget);
    log::info!(
        "wspr_dual_core: pass2 worker done, {} ms{}",
        (now_us() - t0) / 1000,
        if job.deadline_us.is_some() {
            " (budgeted)"
        } else {
            ""
        },
    );
    let raw = Box::into_raw(Box::new(out));
    unsafe { queue_send_ptr(PASS2_RESULT_Q.get(), raw) };
    unsafe { vTaskDelete(ptr::null_mut()) };
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// `i64`-holding cell, same `UnsafeCell` shape as [`QueueCell`] just
/// above (this target has no `AtomicI64`/`AtomicU64` — Xtensa's
/// `max_atomic_width` tops out at 32 bits — so the deadline can't be
/// a plain atomic the way [`AtomicUsize`] is used elsewhere in this
/// file).
struct DeadlineCell(core::cell::UnsafeCell<i64>);
unsafe impl Sync for DeadlineCell {}
impl DeadlineCell {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(i64::MAX))
    }
    fn get(&self) -> i64 {
        unsafe { *self.0.get() }
    }
    /// SAFETY: only ever called from the single writer for this cell
    /// (main for `MAIN_DEADLINE_US`, `pass2_worker_main` for
    /// `WORKER_DEADLINE_US`), strictly before the corresponding
    /// reader can run — main's write happens-before its own
    /// subsequent call; the worker's write happens-before its own
    /// subsequent call on the same task, with no cross-core write to
    /// this cell at all. Never written concurrently with a read.
    unsafe fn set(&self, v: i64) {
        unsafe { *self.0.get() = v };
    }
}

/// Deadline for whichever pass-2 candidate the worker task is
/// currently processing, in [`now_us`] units. Written by
/// [`pass2_worker_main`] right before use; read by
/// [`worker_budget_check`]. Global rather than a captured closure
/// because a closure can't cross the raw-pointer task-creation
/// boundary the worker's job travels over without boxing — safe here
/// under this module's own single-pass-2-worker-at-a-time invariant
/// (see the module doc comment): nothing else touches this while a
/// worker is alive.
static WORKER_DEADLINE_US: DeadlineCell = DeadlineCell::new();

fn worker_budget_check() -> bool {
    now_us() < WORKER_DEADLINE_US.get()
}

/// [`WORKER_DEADLINE_US`]'s counterpart for `pass2_split`'s own
/// (main-side, rank-0) candidate. Two separate cells rather than one
/// shared: main sets its own before entering its own call, the worker
/// sets its own inside `pass2_worker_main` on a different core —
/// sharing one would race.
static MAIN_DEADLINE_US: DeadlineCell = DeadlineCell::new();

fn main_budget_check() -> bool {
    now_us() < MAIN_DEADLINE_US.get()
}

fn current_core() -> i32 {
    unsafe { esp_idf_svc::sys::xTaskGetCoreID(ptr::null_mut()) }
}

/// True if [`pass2_split`] is likely to find room.
pub fn pass2_room_available() -> bool {
    have_room_for(PASS2_WORKER_STACK)
}

/// Run pass 2's (already-ranked, already-truncated-to-`PASS2_DEEP_
/// LADDER_TOP_N`) survivors one per core: `ranked[0]` on `wspr_scan`
/// itself, `ranked[1]` (if it exists) on a freshly-spawned worker.
/// Falls back to running everything on `wspr_scan` sequentially if
/// the runtime guard finds no room, or if there's only 0/1 survivor
/// to begin with (nothing to split).
///
/// `deadline_us`: absolute [`now_us`] deadline applied to *every*
/// survivor's own DT peak-up ladder — `None` (the default everywhere
/// this crate calls in) runs each survivor's full wsprd-faithful
/// sweep unconditionally, matching `pass2_split`'s behaviour before
/// this parameter existed. `Some(t)` bounds each candidate's own
/// worst-case (failing) cost independently — this is not a total
/// pass-2 deadline; two candidates each given the same deadline can
/// each spend up to that much time, one per core, concurrently. See
/// `mfsk_core::wspr::decode::deep_decode_pass2_candidate`'s `budget`
/// parameter for the underlying mechanism and
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Dual-core,
/// take two" for why pass 2's *failing* candidate — not the
/// converging one — is what a deadline here actually bounds (48.1 s
/// vs 11.1 s on the golden file).
pub fn pass2_split(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    ranked: &[WsprPass2Candidate<'_>],
    confirmed: &WsprCallsignTable,
    deadline_us: Option<i64>,
) -> Vec<WsprResult> {
    let mut out = Vec::new();
    if ranked.is_empty() {
        return out;
    }
    // Same global-plus-fn-item pattern as `WORKER_DEADLINE_US` /
    // `worker_budget_check` — main's own call never leaves this
    // function so a captured closure *would* work here, but sharing
    // the pattern with the worker keeps the two sides visibly
    // symmetric instead of one being a closure and one a static.
    if let Some(deadline) = deadline_us {
        unsafe { MAIN_DEADLINE_US.set(deadline) };
    }
    let budget: Option<&(dyn Fn() -> bool + Sync)> =
        deadline_us.map(|_| &main_budget_check as &(dyn Fn() -> bool + Sync));
    if ranked.len() == 1 || !have_room_for(PASS2_WORKER_STACK) {
        if ranked.len() > 1 {
            log::warn!(
                "wspr_dual_core: skipping pass2 split, largest free block {} B < {} B needed",
                largest_free_internal(),
                PASS2_WORKER_STACK as usize + SAFETY_MARGIN_BYTES,
            );
        }
        for r in ranked {
            if let Some(d) =
                deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, r, budget)
            {
                out.push(d);
            }
        }
        return out;
    }

    // Exactly 2 survivors, room available: main takes rank 0, worker
    // takes rank 1.
    let worker_cand = &ranked[1];
    let job = Box::new(Pass2Job {
        idat: idat.as_ptr(),
        idat_len: idat.len(),
        qdat: qdat.as_ptr(),
        qdat_len: qdat.len(),
        sample_rate,
        pad,
        sync: worker_cand.sync,
        freq: worker_cand.freq,
        lag: worker_cand.lag,
        drift: worker_cand.drift,
        isqs: worker_cand.isqs.clone(),
        snr_db: worker_cand.c.snr_db,
        confirmed: confirmed as *const WsprCallsignTable,
        deadline_us,
    });
    let mut handle: esp_idf_svc::sys::TaskHandle_t = ptr::null_mut();
    let r = unsafe {
        xTaskCreatePinnedToCore(
            Some(pass2_worker_main),
            c"wspr_p2".as_ptr(),
            PASS2_WORKER_STACK,
            Box::into_raw(job) as *mut core::ffi::c_void,
            5,
            &mut handle,
            APP_CPU,
        )
    };
    if r != PD_PASS {
        log::error!(
            "wspr_dual_core: xTaskCreatePinnedToCore(wspr_p2) failed: {r}, falling back to sequential"
        );
        for r in ranked {
            if let Some(d) =
                deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, r, budget)
            {
                out.push(d);
            }
        }
        return out;
    }

    // Main processes rank 0 concurrently.
    let t0 = now_us();
    log::info!(
        "wspr_dual_core: pass2 main started on core {}",
        current_core()
    );
    if let Some(d) =
        deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, &ranked[0], budget)
    {
        out.push(d);
    }
    log::info!(
        "wspr_dual_core: pass2 main done, {} ms",
        (now_us() - t0) / 1000
    );

    let worker_ptr = unsafe { queue_recv_ptr::<Option<WsprResult>>(PASS2_RESULT_Q.get()) };
    let worker_result = unsafe { *Box::from_raw(worker_ptr) };
    if let Some(d) = worker_result {
        out.push(d);
    }
    unsafe { vTaskDelay(TEARDOWN_DELAY_TICKS) };
    out
}
