//! Dual-core worker for Pass 2 / Stage 3 / Stage 2 (Phase E2)
//! candidate-loop parallelism.
//!
//! ## Protocol — host mpsc に対応する単一値転送チャネル
//!
//! 旧実装は `*mut Option<Vec<T>>` (out-pointer) と `xTaskNotify` slot を
//! 別チャネルとして併用していた。slot 跨ぎでデータ書き込みと完了通知
//! が desync し、Phase E2 で slot 2+ が 0 results になる症状が出た。
//!
//! 本実装は **FreeRTOS Queue による値転送1チャネル**に統一する。
//! 入力は `Box<Job>` の生ポインタを `JOB_Q` に send、結果は variant
//! 別の result queue (`*mut Vec<T>`) で送り返す。Queue が data 転送と
//! 完了通知を atomic に提供するため、host の `mpsc::sync_channel` と
//! 構造的に等価になる（depth=1 → `sync_channel(1)` 相当）。
//!
//! ## Memory
//!
//! Worker uses a Q15 basis scratch pair (60 KB each, internal DRAM) so
//! the esp-dsp asm dot product stays at 1 cycle/sample. Phase 0.7: the
//! caller owns these buffers and hands their raw pointers to `init`,
//! letting WiFi-mode boot skip the 120 KB allocation entirely. The
//! pointers must reference 16-byte-aligned `BASIS_SCRATCH_LEN`-element
//! `i16` buffers in internal DRAM (`MALLOC_CAP_INTERNAL`) for the asm
//! kernel to hit its 1 cycle/sample target.
//!
//! ## Safety
//!
//! Job 内の生ポインタ (`audio`, `spec`, `allsum_ptr`) は dispatch 関数
//! の call frame に紐付いた借用を消したもの。dispatch 関数は `xQueueSend`
//! → 自分の half を計算 → `xQueueReceive` をブロック実行するため、
//! worker が触る間 main 側のスライスは必ず生存している。

use core::cell::UnsafeCell;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

use esp_idf_svc::sys::{
    xQueueGenericCreate, xQueueGenericSend, xQueueReceive, xTaskCreatePinnedToCore,
    xTaskGetCoreID, QueueHandle_t,
};

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    coarse_sync, process_candidates_into_with_cs_scratch_tuned, refine_candidates_into,
    RefinedCandidate, Spectrogram, BASIS_SCRATCH_LEN,
};

use crate::internal_pool::{CS_SCRATCH_MAIN, CS_SCRATCH_WORKER};
use crate::pipeline::{self, SpecBundle, Slot};

/// One slot's Phase-C output. Both apps consume it identically:
/// `spec` for `xsnr2_db_simple`, `slot` for `wav_idx` /
/// `inc_total_us`, `results` for the UI + QSO FSM, and the `t_*`
/// timestamps for the per-slot timing log.
pub struct SpeculativeOut {
    pub spec: Box<SpecBundle>,
    pub slot: Box<Slot>,
    pub results: Vec<DecodeResult>,
    pub n_pass1: usize,
    pub n_ready: usize,
    pub n_deferred: usize,
    /// `esp_timer_get_time()` right after `recv_box::<SpecBundle>` returns.
    pub t_post_recv: i64,
    /// after `coarse_sync_split_with_allsum`.
    pub t_coarse_done: i64,
    /// after the speculative pass-2 + stage-3 on `audio_prefix` finish
    /// (or immediately, if `ready` was empty).
    pub t_early_done: i64,
    /// after `recv_box::<Slot>` returns.
    pub t_slot_recv: i64,
    /// after the deferred pass-2 + stage-3 on `slot.audio` finish.
    pub t_done: i64,
}

/// Phase-C audio-tail speculation runner. Receives a SpecBundle and
/// Slot pair from the streaming pipeline, partitions pass1 by
/// whether each candidate's Goertzel window fits inside the
/// SpecBundle audio prefix, runs pass-2 + stage-3 speculatively on
/// the `ready` half before SlotEnd, then completes any deferred
/// candidates on the full `slot.audio` after SlotEnd. The decode
/// pipelines in both `m5stack-s3-app` and `m5stack-core2-app` use
/// this single entry to keep the speculative logic in one place.
///
/// `freq_min` / `freq_max` are the coarse_sync band edges
/// (typically 100..3000 Hz). `sync_min` is the coarse_sync ratio
/// threshold (`1.0` matches both apps' historical setting).
#[allow(clippy::too_many_arguments)]
pub fn run_speculative_slot(
    spec_q: esp_idf_svc::sys::QueueHandle_t,
    slot_q: esp_idf_svc::sys::QueueHandle_t,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    pass1_limit: usize,
    max_cand: usize,
    q_thresh: u32,
    bp_max_iter: u32,
    depth: DecodeDepth,
    basis_re_main: &mut [i16],
    basis_im_main: &mut [i16],
) -> SpeculativeOut {
    use esp_idf_svc::sys::esp_timer_get_time;

    let spec = pipeline::recv_box::<SpecBundle>(spec_q);
    let t_post_recv = unsafe { esp_timer_get_time() };
    let pass1: Vec<SyncCandidate> = coarse_sync_split_with_allsum(
        &spec.spec,
        freq_min,
        freq_max,
        sync_min,
        pass1_limit,
        &spec.allsum_head,
        &spec.allsum_tail,
    );
    let t_coarse_done = unsafe { esp_timer_get_time() };

    let n_pass1 = pass1.len();
    let snap_fill = spec.audio_prefix.len();
    let (ready, deferred): (Vec<SyncCandidate>, Vec<SyncCandidate>) = pass1
        .into_iter()
        .partition(|c| SpecBundle::audio_end_for_dt(c.dt_sec) <= snap_fill);
    let n_ready = ready.len();
    let n_deferred = deferred.len();

    let mut n_early_refined = 0usize;
    let mut results = if !ready.is_empty() {
        let partial_audio: &[i16] = &spec.audio_prefix;
        let p2 = pass2_split(partial_audio, ready, max_cand, basis_re_main, basis_im_main);
        n_early_refined = p2.len();
        stage3_split(
            partial_audio,
            p2,
            depth,
            q_thresh,
            bp_max_iter,
            basis_re_main,
            basis_im_main,
        )
    } else {
        Vec::new()
    };
    let t_early_done = unsafe { esp_timer_get_time() };

    let slot = pipeline::recv_box::<Slot>(slot_q);
    let t_slot_recv = unsafe { esp_timer_get_time() };

    if !deferred.is_empty() {
        // Budget cap (Gemini PR #123 round 8/9): stage-3's wallclock
        // is roughly linear in the number of refined candidates
        // pass-2 hands it. Sum-cap by the *refined* count (not the
        // decoded count) so `n_early_refined + n_late_refined ≤
        // max_cand` holds regardless of whether the early path's
        // candidates went on to decode. Trade-off: when the early
        // half already fills `max_cand` slots (typical qso3-busy
        // case: ready=25-27, pass-2 keeps top 15) the deferred
        // path gets zero budget and any dt > +0.36 s real signals
        // are dropped. Acceptable on FT8 — operational dt clusters
        // tightly under NTP sync; `time_sync`'s slot phase
        // auto-anchor keeps dt within ±0.5 s in steady state.
        let max_cand_late = max_cand.saturating_sub(n_early_refined);
        if max_cand_late > 0 {
            let p2 = pass2_split(
                &slot.audio,
                deferred,
                max_cand_late,
                basis_re_main,
                basis_im_main,
            );
            let late = stage3_split(
                &slot.audio,
                p2,
                depth,
                q_thresh,
                bp_max_iter,
                basis_re_main,
                basis_im_main,
            );
            results.extend(late);
        }
    }
    let t_done = unsafe { esp_timer_get_time() };

    SpeculativeOut {
        spec,
        slot,
        results,
        n_pass1,
        n_ready,
        n_deferred,
        t_post_recv,
        t_coarse_done,
        t_early_done,
        t_slot_recv,
        t_done,
    }
}

const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;

/// Worker-side basis scratch pointers — set once by `init`, then read
/// only by `worker_main`. The `xTaskCreatePinnedToCore` call in `init`
/// is the release barrier that publishes the stores to the worker core.
struct BasisPtrCell(UnsafeCell<*mut i16>);
unsafe impl Sync for BasisPtrCell {}
impl BasisPtrCell {
    const fn new() -> Self {
        Self(UnsafeCell::new(ptr::null_mut()))
    }
    fn get(&self) -> *mut i16 {
        unsafe { *self.0.get() }
    }
    /// SAFETY: only call from `init` before the worker is spawned.
    unsafe fn set(&self, p: *mut i16) {
        unsafe { *self.0.get() = p };
    }
}

static BASIS_RE_C1: BasisPtrCell = BasisPtrCell::new();
static BASIS_IM_C1: BasisPtrCell = BasisPtrCell::new();

enum Job {
    Pass2 {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<SyncCandidate>,
        max_cand: usize,
    },
    /// Work-stealing stage 3: both main and worker pull individual
    /// candidates from a shared `Vec<Option<RefinedCandidate>>` via
    /// `next_idx.fetch_add(1)`. Per-cand BP wall-clock varies a lot
    /// (failed cands run all 4 LLR variants); a static head/tail split
    /// stalls one core waiting for the other. Dynamic dispatch keeps
    /// both cores busy until the queue drains.
    Stage3WorkSteal {
        audio: *const i16,
        audio_len: usize,
        depth: DecodeDepth,
        q_thresh: u32,
        bp_max_iter: u32,
        slots_ptr: *mut Option<RefinedCandidate>,
        slots_len: usize,
        next_idx: *const AtomicUsize,
    },
    CoarseSyncWithAllsum {
        spec: *const Spectrogram,
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        allsum_ptr: *const f32,
        allsum_len: usize,
    },
}

/// SAFETY: Job's raw pointers are produced by dispatch fns that
/// block-wait on the result before returning, so the referenced data
/// outlives the worker's access. Vec ownership transfers via Box.
unsafe impl Send for Job {}

/// Queue handle holder — initialised once in `init()`, then read-only.
struct QueueCell(UnsafeCell<QueueHandle_t>);
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
static PASS2_RESULT_Q: QueueCell = QueueCell::new();
static STAGE3_RESULT_Q: QueueCell = QueueCell::new();
static COARSE_RESULT_Q: QueueCell = QueueCell::new();

#[inline]
unsafe fn queue_create(item_size: usize) -> QueueHandle_t {
    let q = unsafe {
        xQueueGenericCreate(
            1, // depth
            item_size as u32,
            QUEUE_TYPE_BASE,
        )
    };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

/// Send a single boxed pointer through a depth-1 queue.
/// SAFETY: caller transfers ownership of `ptr` to the receiver.
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

/// Receive a single pointer from a depth-1 queue.
/// SAFETY: receiver takes ownership of returned pointer.
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

extern "C" fn worker_main(_arg: *mut core::ffi::c_void) {
    log::info!("dsp_worker: started on core {}", current_core());
    loop {
        let job_ptr = unsafe { queue_recv_ptr::<Job>(JOB_Q.get()) };
        let job = unsafe { *Box::from_raw(job_ptr) };
        match job {
            Job::Pass2 {
                audio,
                audio_len,
                cands,
                max_cand,
            } => {
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                let basis_re = unsafe {
                    core::slice::from_raw_parts_mut(BASIS_RE_C1.get(), BASIS_SCRATCH_LEN)
                };
                let basis_im = unsafe {
                    core::slice::from_raw_parts_mut(BASIS_IM_C1.get(), BASIS_SCRATCH_LEN)
                };
                let result = refine_candidates_into(
                    audio_slice,
                    cands,
                    max_cand,
                    basis_re,
                    basis_im,
                );
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(PASS2_RESULT_Q.get(), raw) };
            }
            Job::Stage3WorkSteal {
                audio,
                audio_len,
                depth,
                q_thresh,
                bp_max_iter,
                slots_ptr,
                slots_len,
                next_idx,
            } => {
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                let basis_re = unsafe {
                    core::slice::from_raw_parts_mut(BASIS_RE_C1.get(), BASIS_SCRATCH_LEN)
                };
                let basis_im = unsafe {
                    core::slice::from_raw_parts_mut(BASIS_IM_C1.get(), BASIS_SCRATCH_LEN)
                };
                #[allow(static_mut_refs)]
                let result = unsafe {
                    drain_stage3_queue(
                        audio_slice,
                        slots_ptr,
                        slots_len,
                        &*next_idx,
                        depth,
                        q_thresh,
                        bp_max_iter,
                        basis_re,
                        basis_im,
                        &mut CS_SCRATCH_WORKER,
                    )
                };
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(STAGE3_RESULT_Q.get(), raw) };
            }
            Job::CoarseSyncWithAllsum {
                spec,
                freq_min,
                freq_max,
                sync_min,
                max_cand,
                allsum_ptr,
                allsum_len,
            } => {
                let spec_ref = unsafe { &*spec };
                let allsum = unsafe { core::slice::from_raw_parts(allsum_ptr, allsum_len) };
                let result = mfsk_core::ft8::decode_block::coarse_sync_with_allsum(
                    spec_ref, freq_min, freq_max, sync_min, max_cand, allsum,
                );
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(COARSE_RESULT_Q.get(), raw) };
            }
        }
    }
}

fn current_core() -> i32 {
    unsafe { xTaskGetCoreID(ptr::null_mut()) }
}

/// Spawn the persistent worker task on APP_CPU and create dispatch
/// queues. Call once at startup, after `link_patches()`,
/// `EspLogger::initialize_default()`, and `esp_dsp_fft::prewarm()`.
///
/// `re_c1` / `im_c1` are legacy BASIS scratch pointers — unused
/// since Phase 1.7.7-Stick (mfsk-core now uses Goertzel for both
/// pass-2 and stage-3 cs builds, zero scratch). Pass null; the
/// stored fields are kept only for ABI shape compatibility with
/// the in-tree closures that still bind them via
/// `BASIS_{RE,IM}_C1.set()`. A follow-up commit drops these
/// fields and the arg.
pub fn init(re_c1: *mut i16, im_c1: *mut i16) {
    unsafe {
        BASIS_RE_C1.set(re_c1);
        BASIS_IM_C1.set(im_c1);
        JOB_Q.set(queue_create(core::mem::size_of::<*mut Job>()));
        PASS2_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<RefinedCandidate>>()));
        STAGE3_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<DecodeResult>>()));
        COARSE_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<SyncCandidate>>()));

        let r = xTaskCreatePinnedToCore(
            Some(worker_main),
            c"dsp_worker".as_ptr(),
            16384,
            ptr::null_mut(),
            5,
            ptr::null_mut(),
            1, // APP_CPU
        );
        assert_eq!(
            r, PD_PASS,
            "xTaskCreatePinnedToCore(dsp_worker) failed: {r}"
        );
    }
    log::info!(
        "dsp_worker: spawned on APP_CPU; main is core {}",
        current_core()
    );
}

/// Run Stage 2 sequentially per frequency half on main. No worker
/// dispatch (Phase E2 race fallback path; rarely hit).
pub fn coarse_sync_split(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let mid = 0.5 * (freq_min + freq_max);
    let mut head = coarse_sync(spec, freq_min, mid, sync_min, max_cand);
    let tail = coarse_sync(spec, mid, freq_max, sync_min, max_cand);
    head.extend(tail);
    head.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    head.truncate(max_cand);
    head
}

/// Phase E2: parallel coarse_sync with pre-built per-half allsums.
/// Worker computes the tail half on APP_CPU; main computes the head
/// half locally in parallel.
pub fn coarse_sync_split_with_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum_head: &[f32],
    allsum_tail: &[f32],
) -> Vec<SyncCandidate> {
    use mfsk_core::ft8::decode_block::coarse_sync_with_allsum;
    let mid = 0.5 * (freq_min + freq_max);

    let job = Box::new(Job::CoarseSyncWithAllsum {
        spec: spec as *const _,
        freq_min: mid,
        freq_max,
        sync_min,
        max_cand,
        allsum_ptr: allsum_tail.as_ptr(),
        allsum_len: allsum_tail.len(),
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut local =
        coarse_sync_with_allsum(spec, freq_min, mid, sync_min, max_cand, allsum_head);

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<SyncCandidate>>(COARSE_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    local.truncate(max_cand);
    local
}

/// Pass 2 split across main + worker. Each half scored with its own
/// BASIS scratch; merged top-`max_cand` returned.
pub fn pass2_split(
    audio: &[i16],
    pass1: Vec<SyncCandidate>,
    max_cand: usize,
    basis_re_main: &mut [i16],
    basis_im_main: &mut [i16],
) -> Vec<RefinedCandidate> {
    let mid = pass1.len() / 2;
    let mut head = pass1;
    let tail = head.split_off(mid);

    let job = Box::new(Job::Pass2 {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        cands: tail,
        max_cand,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut local = refine_candidates_into(audio, head, max_cand, basis_re_main, basis_im_main);

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<RefinedCandidate>>(PASS2_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local.sort_by(|a, b| b.2.cmp(&a.2));
    local.truncate(max_cand);
    local
}

/// Stage 3 across main + worker with **work-stealing** dispatch.
///
/// Both cores share the same `Vec<Option<RefinedCandidate>>` and pull
/// candidates by `AtomicUsize::fetch_add(1)`, so a slow / failed cand
/// on one core doesn't stall the other. Compared to the old
/// fixed head/tail split, this absorbs the per-cand BP wall-clock
/// variance (qso3 ~7 of 15 cands fail and run all 4 LLR variants).
#[allow(clippy::too_many_arguments)]
pub fn stage3_split(
    audio: &[i16],
    pass2: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    basis_re_main: &mut [i16],
    basis_im_main: &mut [i16],
) -> Vec<DecodeResult> {
    let mut slots: Vec<Option<RefinedCandidate>> = pass2.into_iter().map(Some).collect();
    let next_idx = AtomicUsize::new(0);

    let slots_ptr = slots.as_mut_ptr();
    let slots_len = slots.len();
    let next_idx_ptr: *const AtomicUsize = &next_idx;

    // Dispatch worker — it will drain the same queue from APP_CPU.
    let job = Box::new(Job::Stage3WorkSteal {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        depth,
        q_thresh,
        bp_max_iter,
        slots_ptr,
        slots_len,
        next_idx: next_idx_ptr,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    // Main drains in parallel.
    #[allow(static_mut_refs)]
    let mut local = unsafe {
        drain_stage3_queue(
            audio,
            slots_ptr,
            slots_len,
            &next_idx,
            depth,
            q_thresh,
            bp_max_iter,
            basis_re_main,
            basis_im_main,
            &mut CS_SCRATCH_MAIN,
        )
    };

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<DecodeResult>>(STAGE3_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    // `slots` is now drained (all Options taken); drop is fine.
    drop(slots);

    local.extend(worker);
    local
}

/// Pop candidates from a shared atomic-indexed slot array and process
/// them one at a time, accumulating successful decodes.
///
/// SAFETY: `slots_ptr` must point to `slots_len` valid
/// `Option<RefinedCandidate>` cells, and `next_idx` claims an exclusive
/// index per `fetch_add` so the same slot is never read by two callers.
#[allow(clippy::too_many_arguments)]
unsafe fn drain_stage3_queue(
    audio: &[i16],
    slots_ptr: *mut Option<RefinedCandidate>,
    slots_len: usize,
    next_idx: &AtomicUsize,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
    cs_scratch: &mut [[mfsk_core::core::scalar::Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    let mut out: Vec<DecodeResult> = Vec::new();
    loop {
        let i = next_idx.fetch_add(1, Ordering::AcqRel);
        if i >= slots_len {
            break;
        }
        // SAFETY: the atomic fetch_add gives this caller exclusive
        // ownership of slot `i` for the duration of this iteration.
        let cand = unsafe { (*slots_ptr.add(i)).take() };
        let Some(cand) = cand else { continue };
        let mut single = vec![cand];
        let mut results = process_candidates_into_with_cs_scratch_tuned(
            audio,
            core::mem::take(&mut single),
            depth,
            q_thresh,
            bp_max_iter,
            basis_re,
            basis_im,
            cs_scratch,
        );
        out.append(&mut results);
    }
    out
}
