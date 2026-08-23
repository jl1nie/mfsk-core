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
//! ## Where the worker's stack comes from
//!
//! A single `.bss` reservation ([`WORKER_STACK`]), shared by the
//! pass-0/1 and pass-2 workers, via `xTaskCreateStaticPinnedToCore`.
//!
//! It used to be a heap allocation guarded by a live
//! `heap_caps_get_largest_free_block` check: if the contiguous block
//! wasn't there, dual-core was skipped for that phase and the pass ran
//! sequentially — slower, not a crash. That guard was the right call
//! while the stacks were dynamic, but it made having the second core a
//! property of whatever else had touched the heap recently. With a live
//! WiFi association it started losing: measured on a CoreS3 (issue
//! #260), pass 0's worker spawned and **pass 1's did not**, costing
//! that pass 17 583 -> 28 246 ms and the scan 11 s, with nothing in the
//! log to say why. Reserving the stack at link time removes the
//! question — the guard, the fallback and the sizing uncertainty all go
//! with it.
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
    vTaskDelay, xQueueGenericCreate, xQueueGenericSend, xQueueReceive, QueueHandle_t,
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
const APP_CPU: i32 = 1;

/// Statically-reserved worker stack, shared by the pass-0/1 and
/// pass-2 workers.
///
/// **Why `.bss` and not the heap.** FreeRTOS carves a dynamically
/// created task's stack out of internal DRAM in one contiguous piece,
/// and that pool is shared with everything else on the chip — including
/// a live WiFi association, which allocates and frees RX/TX buffers
/// continuously. Measured on a CoreS3 (issue #260): with the radio
/// associated across the decode, pass 0's worker spawned and **pass 1's
/// did not**, costing that pass its dual-core split (17 583 -> 28 246 ms)
/// and the scan 11 s. Nothing had changed but the heap's shape at the
/// moment of the second spawn. A `.bss` reservation is made by the
/// linker, so it cannot be lost to fragmentation, cannot be raced by
/// the WiFi driver, and turns "did we get the second core this time?"
/// from a runtime lottery into a build-time fact. Same reasoning as
/// FT8's [`crate::internal_pool`], one level up: that keeps a hot
/// buffer out of the heap, this keeps a stack out of it.
///
/// **One buffer for both workers.** They never coexist — passes 0 and 1
/// finish before pass 2 begins, and each pass spawns exactly one worker
/// that self-deletes. The existing [`TEARDOWN_DELAY_TICKS`] wait after
/// a worker's result is what makes the reuse safe: with static
/// allocation the idle task still has to finish taking the old task off
/// its termination list before the same `StaticTask_t` is handed back
/// to `xTaskCreateStaticPinnedToCore`.
///
/// 80 KB against a measured 63 360 B peak (~29 % headroom). That peak
/// is higher than the 53 KB / 44 KB the two *per-pass* workers used,
/// and necessarily so: one task serving both job shapes has a frame
/// sized for the union of them. Boxing the payloads and passing the
/// `Box` into `run_pass2_job` recovered most of it (73 616 B → 63 360 B);
/// what remains is the price of not spawning per pass.
const WORKER_STACK_BYTES: usize = 81_920;

// The stack itself lives in `crate::worker_arena` — the same
// reservation FST4's worker and FT8's cs staging use, because only one
// of the three modes runs in a given boot. Still a link-time
// reservation, for the reason this module's own docs give: with WiFi
// up there is no 80 KB contiguous block to allocate.
static mut WORKER_TCB: core::mem::MaybeUninit<esp_idf_svc::sys::StaticTask_t> =
    core::mem::MaybeUninit::uninit();

/// Spawn a worker on `APP_CPU` using the static stack + TCB above.
/// Returns `false` if FreeRTOS refused, which with static allocation
/// means a programming error (a live task on the same buffers), not
/// memory pressure.
///
/// SAFETY: caller must guarantee no previously-spawned worker is still
/// alive — see [`WORKER_STACK`]. Every call site is preceded by the
/// result-queue receive plus [`TEARDOWN_DELAY_TICKS`].
/// Take this mode's worker stack at boot, before WiFi.
///
/// Must run before anything fragments internal DRAM — see
/// [`crate::worker_arena`] for the measurements that make the ordering
/// non-negotiable. Returns `false` if it could not, in which case the
/// worker never spawns and every pass runs single-core.
pub fn reserve_arena() -> bool {
    crate::worker_arena::reserve(crate::worker_arena::Owner::WsprWorker, WORKER_STACK_BYTES)
}

unsafe fn spawn_worker_static() -> bool {
    let Some(stack) =
        crate::worker_arena::claim(crate::worker_arena::Owner::WsprWorker, WORKER_STACK_BYTES)
    else {
        log::error!("wspr_dual_core: worker arena unavailable — staying single-core");
        return false;
    };
    let h = unsafe {
        esp_idf_svc::sys::xTaskCreateStaticPinnedToCore(
            Some(worker_main),
            c"wspr_worker".as_ptr(),
            WORKER_STACK_BYTES as u32,
            core::ptr::null_mut(),
            5,
            stack,
            core::ptr::addr_of_mut!(WORKER_TCB) as *mut esp_idf_svc::sys::StaticTask_t,
            APP_CPU,
        )
    };
    !h.is_null()
}

/// Ticks yielded after a result arrives.
///
/// Was the wait for the idle task to reap a self-deleted worker before
/// the next spawn reused its stack. The worker is persistent now, so
/// nothing is being reaped — this is only a yield so the worker's own
/// post-result logging lands before main's, keeping the trace readable.
const TEARDOWN_DELAY_TICKS: u32 = 5;

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
        JOB_Q.set(queue_create(core::mem::size_of::<*mut Job>()));
        PASS01_RESULT_Q.set(queue_create(core::mem::size_of::<
            *mut Vec<(WsprResult, usize)>,
        >()));
        PASS2_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Option<WsprResult>>()));
        // SAFETY: called once at startup, before any other task exists.
        assert!(spawn_worker_static(), "wspr_dual_core: worker spawn failed");
    }
    log::info!("wspr_dual_core: worker + queues ready");
}

/// Work handed to the persistent worker. One variant per pass shape;
/// the worker blocks on [`JOB_Q`] and never exits.
/// Payloads are boxed, so `Job` is one pointer wide.
///
/// Not a style choice: `Pass2Job` carries an `IsQs` (10 368 B), and an
/// unboxed enum is sized for its largest variant — so receiving a
/// *pass-0/1* job would move 10.4 KB onto the worker's stack for
/// nothing. Against a measured 53 KB pass-0/1 peak in a 64 KB stack
/// that is the difference between fitting and a `Cache error` from the
/// overflow scribbling past the end (issue #260).
enum Job {
    Pass01(Box<Pass01Job>),
    Pass2(Box<Pass2Job>),
}

static JOB_Q: QueueCell = QueueCell::new();

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

/// The one worker task, for the process lifetime.
///
/// Was two tasks spawned per pass and self-deleted. That worked, but it
/// made having a second core a per-pass event with a failure path,
/// a `TEARDOWN_DELAY_TICKS` wait for the idle task to reap the previous
/// one, and — while the stacks were still heap-allocated — a silent
/// dependence on whatever WiFi had done to the heap in the meantime
/// (issue #260). A task that is created once and blocks on a queue has
/// none of those: same shape as FT8's `dual_core::worker_main`.
extern "C" fn worker_main(_arg: *mut core::ffi::c_void) {
    log::info!("wspr_dual_core: worker started on core {}", current_core());
    loop {
        let job_ptr = unsafe { queue_recv_ptr::<Job>(JOB_Q.get()) };
        match unsafe { *Box::from_raw(job_ptr) } {
            Job::Pass01(job) => {
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
                log_worker_stack("p01", WORKER_STACK_BYTES as u32);
            }
            Job::Pass2(job) => {
                let out = run_pass2_job(job);
                let raw = Box::into_raw(Box::new(out));
                unsafe { queue_send_ptr(PASS2_RESULT_Q.get(), raw) };
                log_worker_stack("p2", WORKER_STACK_BYTES as u32);
            }
        }
    }
}

/// Report what a worker task actually used, against what it reserved.
///
/// The main scan task has had this since the stack audit; the workers
/// never did, which left the two largest allocations in the system
/// (80 KB and 88 KB) sized by estimate. That matters now: FreeRTOS task
/// stacks come out of internal DRAM in one contiguous piece, and once
/// WiFi has been initialised the largest such block on a CoreS3 is
/// 156 KB — less than the 170-178 KB this pair plus the scan task need,
/// so the dual-core path silently declines to spawn (issue #260).
/// Right-sizing needs measured peaks, not estimates.
fn log_worker_stack(who: &str, reserved: u32) {
    // `uxTaskGetStackHighWaterMark(NULL)` = smallest free the *calling*
    // task ever had, in bytes on this port.
    let headroom = unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(ptr::null_mut()) };
    log::info!(
        "wspr_dual_core: {who} stack peak {} B of {reserved} B reserved ({} B headroom)",
        reserved.saturating_sub(headroom),
        headroom,
    );
}

/// Always true since the worker stack moved to `.bss` — kept so
/// callers that branch on it keep compiling, and because "is there
/// room?" is exactly the question [`WORKER_STACK`] exists to answer
/// once, at link time, instead of per spawn.
pub fn pass01_room_available() -> bool {
    true
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
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(Box::new(Job::Pass01(job)))) };
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

/// Pass 2's half of the worker loop.
///
/// Takes the `Box`, not the value. `worker_main`'s frame reserves space
/// for every match arm's temporaries at once, so moving a `Pass2Job`
/// (10 368 B of `IsQs`) out at the match site charges that to the
/// *pass-0/1* path as well (issue #260).
fn run_pass2_job(job: Box<Pass2Job>) -> Option<WsprResult> {
    let t0 = now_us();
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
    out
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

/// Always true — see [`pass01_room_available`].
pub fn pass2_room_available() -> bool {
    true
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
    if ranked.len() == 1 {
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
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(Box::new(Job::Pass2(job)))) };
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
