//! Dual-core work-steal for FST4's candidate loop — issue #327.
//!
//! A third sibling to [`dual_core`](crate::dual_core) (FT8) and
//! [`wspr_dual_core`](crate::wspr_dual_core) (WSPR), not a shared
//! abstraction over them. `embedded-poc/CLAUDE.md` states that
//! convention for the existing pair, and it holds again here: FT8's
//! `Job` enum, its four typed result queues and its `CS_SCRATCH_*`
//! buffers are all sized and shaped for FT8's 79-symbol geometry, so
//! adding FST4 variants would drag FST4 types into that module and
//! inflate its 16 KB worker's frame for work it never runs.
//!
//! ## What it borrows from each
//!
//! | | source | why |
//! |---|---|---|
//! | persistent single worker | FT8 | FST4 has no phase-dependent stack-size split, so WSPR's spawn/teardown lifecycle buys nothing |
//! | static `.bss` stack | WSPR | see below |
//! | work-steal via one `AtomicUsize` | both | per-candidate cost is bimodal |
//! | send → do-my-share → blocking recv | both | the whole safety argument |
//!
//! **Static `.bss` stack, not FT8's heap spawn.** FT8 gets away with
//! `xTaskCreatePinnedToCore` because its single spawn happens at boot,
//! before WiFi. WSPR measured the failure mode when that isn't true: a
//! worker spawn was silently lost to heap fragmentation with the radio
//! associated, costing that pass its split (17 583 → 28 246 ms) and the
//! scan 11 s, with nothing in the log. A `.bss` reservation is made by
//! the linker and cannot be lost, which turns "did we get the second
//! core this time?" from a runtime lottery into a build-time fact.
//! FST4's stack need is small enough (see [`WORKER_STACK_BYTES`]) that
//! this costs almost nothing.
//!
//! **Work-steal, not a static split.** Per-candidate cost is strongly
//! bimodal: on the wideband run that motivated this, real signals
//! decoded in 53 ms while individual noise candidates ran the OSD
//! escalation for ~19 s. A static half/half split strands whichever
//! core draws the slow one — the same reasoning `wspr_dual_core`
//! documents for its pass 0/1, and precisely why WSPR's *pass 2* static
//! 1-1 split is the weakest number in its whole table (1.10x).
//!
//! ## Shape of the work
//!
//! The unit is one candidate index. Everything a candidate needs —
//! the coarse-sync list, the DDC baseband, the group delay — is shared
//! read-only for the whole batch, so it lives in [`Ctx`] behind a
//! raw pointer rather than being copied per job. The per-candidate
//! function is a plain `fn` item, not a closure: it has to be callable
//! from the worker task, and a closure's captures cannot cross that
//! boundary. Context reaches it through [`ctx`], the same
//! global-plus-`fn`-item idiom `wspr_dual_core`'s `DeadlineCell` uses
//! for its budget check (and for the same underlying reason — Xtensa's
//! `max_atomic_width` is 32, so several of these cannot be atomics).

use core::cell::UnsafeCell;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::boxed::Box;
use alloc::vec::Vec;

use esp_idf_svc::sys::{
    xQueueGenericCreate, xQueueGenericSend, xQueueReceive, xTaskCreateStaticPinnedToCore,
    xTaskGetCoreID, QueueHandle_t,
};

// Not re-exported by esp-idf-svc; same local declarations
// `wspr_dual_core` makes.
const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;
const APP_CPU: i32 = 1;

/// Measured, not guessed — but sized with margin because the number it
/// has to cover is a *peak*, not an average.
///
/// `fst4_ddc_bench` reserves 96 KiB for its own task and reports
/// ~81.7 KiB still untouched after a full wideband monitor run, i.e.
/// ~14 KiB actually used across `ddc_refine` + `fst4_sync_search` +
/// the LLR/BP/OSD ladder including its ~8 KiB `g_packed` OSD scratch.
/// 48 KiB is ~3.4x that.
///
/// Two orders of magnitude below WSPR's 81 920 B, which it needs
/// because its per-candidate Fano state is genuinely ~63 KiB. FST4
/// shares FT8's `BpPooledFec` machinery (`Ldpc240_101` vs
/// `Ldpc174_91`), so it sits near FT8's 16 KB rather than WSPR's.
///
/// [`log_worker_stack`] reports the real high-water mark every batch so
/// this stays measured rather than reverting to a guess.
pub const WORKER_STACK_BYTES: usize = 48 * 1024;

#[repr(align(16))]
struct WorkerStack([u8; WORKER_STACK_BYTES]);
static mut WORKER_STACK: WorkerStack = WorkerStack([0; WORKER_STACK_BYTES]);
static mut WORKER_TCB: core::mem::MaybeUninit<esp_idf_svc::sys::StaticTask_t> =
    core::mem::MaybeUninit::uninit();

struct QueueCell(UnsafeCell<QueueHandle_t>);
// SAFETY: written once in `init()` before any other access.
unsafe impl Sync for QueueCell {}
impl QueueCell {
    const fn new() -> Self {
        Self(UnsafeCell::new(ptr::null_mut()))
    }
    fn get(&self) -> QueueHandle_t {
        unsafe { *self.0.get() }
    }
    /// SAFETY: only call from `init()` before any other access.
    unsafe fn set(&self, q: QueueHandle_t) {
        unsafe { *self.0.get() = q };
    }
}

static JOB_Q: QueueCell = QueueCell::new();
static RESULT_Q: QueueCell = QueueCell::new();

unsafe fn queue_create(item_size: usize) -> QueueHandle_t {
    let q = unsafe { xQueueGenericCreate(1, item_size as u32, QUEUE_TYPE_BASE) };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

/// SAFETY: caller transfers ownership of `p` to the receiver.
unsafe fn queue_send_ptr<T>(q: QueueHandle_t, p: *mut T) {
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&p as *const *mut T) as *const core::ffi::c_void,
            PORT_MAX_DELAY,
            QUEUE_SEND_TO_BACK,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueGenericSend failed: {r}");
}

/// SAFETY: receiver takes ownership of the returned pointer.
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

/// Everything one candidate needs that does not vary across the batch.
/// Raw pointers rather than slices because this crosses a FreeRTOS task
/// boundary; the send → work → blocking-recv discipline in
/// [`run_split`] is what makes them valid for the whole batch.
pub struct Ctx {
    pub candidates: *const core::ffi::c_void,
    pub n: usize,
    pub coarse_i: *const f32,
    pub coarse_q: *const f32,
    pub coarse_len: usize,
    pub coarse_delay_orig: usize,
    /// Absolute `esp_timer` deadline for the whole batch; a core that
    /// reaches it stops claiming new candidates.
    pub deadline_us: i64,
    /// When the batch started, so per-candidate results can report a
    /// loop-relative timestamp that means the same thing on both cores.
    pub t0_us: i64,
}

/// The per-candidate worker function. A `fn` item, not a closure —
/// see the module doc comment.
pub type CandidateFn<R> = fn(&Ctx, usize) -> Option<R>;

struct Job<R: 'static> {
    ctx: *const Ctx,
    work: CandidateFn<R>,
}
// SAFETY: `ctx` outlives the job — `run_split` blocks on the result
// queue before returning, so the borrow cannot dangle. `work` is a
// plain function pointer.
unsafe impl<R> Send for Job<R> {}

static NEXT_IDX: AtomicUsize = AtomicUsize::new(0);

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn current_core() -> i32 {
    unsafe { xTaskGetCoreID(ptr::null_mut()) }
}

/// Report this task's stack high-water mark against what it reserved —
/// lifted from `wspr_dual_core`, and worth keeping for the same reason:
/// it is what turns [`WORKER_STACK_BYTES`] from a guess into a
/// measurement.
fn log_worker_stack(who: &str, reserved: usize) {
    let free = unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(ptr::null_mut()) } as usize;
    log::info!(
        "fst4_dual_core: {who} stack — {} B free of {} B reserved (peak use {} B)",
        free,
        reserved,
        reserved.saturating_sub(free),
    );
}

/// Claim candidate indices until the list or the deadline runs out.
/// Both cores run this identical loop; `fetch_add` is what grants one
/// of them exclusive ownership of each index.
fn drain<R>(ctx: &Ctx, work: CandidateFn<R>) -> Vec<R> {
    let mut out = Vec::new();
    loop {
        let i = NEXT_IDX.fetch_add(1, Ordering::AcqRel);
        if i >= ctx.n {
            break;
        }
        if now_us() >= ctx.deadline_us {
            break;
        }
        if let Some(r) = work(ctx, i) {
            out.push(r);
        }
    }
    out
}

/// Single-core equivalent of [`run_split`] — same `work`, same
/// deadline, same claim loop, just without the second core. Exists so
/// the dual-core comparison is one build switch over one code path
/// rather than two separate loops that could drift apart.
pub fn drain_single<R>(ctx: &Ctx, work: CandidateFn<R>) -> Vec<R> {
    NEXT_IDX.store(0, Ordering::Release);
    let out = drain(ctx, work);
    log_worker_stack("main(single)", 0);
    out
}

/// The generic worker body, monomorphised per result type `R` by
/// [`spawn_worker_for`].
extern "C" fn worker_main<R: 'static>(_arg: *mut core::ffi::c_void) {
    log::info!("fst4_dual_core: worker started on core {}", current_core());
    loop {
        let job_ptr = unsafe { queue_recv_ptr::<Job<R>>(JOB_Q.get()) };
        // SAFETY: sender boxed this and transferred ownership.
        let job = unsafe { Box::from_raw(job_ptr) };
        // SAFETY: `run_split` blocks until it has received our result,
        // so `ctx` is alive for the whole of this call.
        let ctx = unsafe { &*job.ctx };
        let out = drain(ctx, job.work);
        log_worker_stack("worker", WORKER_STACK_BYTES);
        unsafe { queue_send_ptr(RESULT_Q.get(), Box::into_raw(Box::new(out))) };
    }
}

static INIT_DONE: AtomicUsize = AtomicUsize::new(0);

/// Create the dispatch queues and spawn the persistent worker on
/// APP_CPU. Call once at startup — **before** WiFi or any other
/// large allocator activity, matching `wspr_app`'s ordering constraint
/// (real hardware saw `xTaskCreatePinnedToCore` fail outright when task
/// stacks were reserved after WiFi brought its buffers up; the `.bss`
/// stack here sidesteps that for the stack itself, but the TCB and
/// queues are still allocations).
///
/// `R` fixes the result type this worker will handle; one call site,
/// one `R`. A second `init` with a different `R` would spawn a second
/// worker against the same queues and is rejected.
pub fn init<R: 'static>() {
    if INIT_DONE.swap(1, Ordering::AcqRel) != 0 {
        log::warn!("fst4_dual_core: init called twice — ignoring");
        return;
    }
    unsafe {
        JOB_Q.set(queue_create(core::mem::size_of::<*mut Job<R>>()));
        RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<R>>()));
        let h = xTaskCreateStaticPinnedToCore(
            Some(worker_main::<R>),
            c"fst4_worker".as_ptr(),
            WORKER_STACK_BYTES as u32,
            ptr::null_mut(),
            5,
            ptr::addr_of_mut!(WORKER_STACK) as *mut u8,
            ptr::addr_of_mut!(WORKER_TCB) as *mut esp_idf_svc::sys::StaticTask_t,
            APP_CPU,
        );
        assert!(!h.is_null(), "fst4_dual_core: worker spawn failed");
    }
    log::info!(
        "fst4_dual_core: worker spawned on core {APP_CPU} with {} B static stack",
        WORKER_STACK_BYTES
    );
}

/// Run `work` over `0..ctx.n` across both cores, returning every
/// `Some` result. Order is **not** preserved — two cores complete
/// interleaved, and the caller is expected to sort or key the results
/// itself.
///
/// Strictly send → do-my-own-share → blocking recv, which is the entire
/// safety argument for the raw pointers in [`Ctx`]: this function
/// cannot return while the worker still holds them.
pub fn run_split<R: 'static>(ctx: &Ctx, work: CandidateFn<R>) -> Vec<R> {
    NEXT_IDX.store(0, Ordering::Release);

    let job = Box::new(Job {
        ctx: ctx as *const Ctx,
        work,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut mine = drain(ctx, work);
    log_worker_stack("main", 0);

    let theirs_ptr = unsafe { queue_recv_ptr::<Vec<R>>(RESULT_Q.get()) };
    // SAFETY: the worker boxed this and transferred ownership.
    let theirs = unsafe { Box::from_raw(theirs_ptr) };

    let n_worker = theirs.len();
    mine.extend(*theirs);
    log::info!(
        "fst4_dual_core: batch done — {} results ({} from worker)",
        mine.len(),
        n_worker,
    );
    mine
}
