//! Streaming RX pipeline messages and queue helpers.
//!
//! The pipeline is wav_sim → stage1_inc → main, connected by two
//! FreeRTOS Queues. Each stage owns its own buffers and transfers
//! ownership through the queues by sending raw `Box::into_raw` pointers.
//! No shared mutable state, no notification-and-out-pointer split.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ptr;

use esp_idf_svc::sys::{
    xQueueGenericCreate, xQueueGenericSend, xQueueReceive, QueueHandle_t,
};

const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;

/// Audio chunk size pushed by wav_sim each tick (100 ms @ 12 kHz).
pub const CHUNK_LEN: usize = 1_200;

/// Message from wav_sim to stage1_inc.
pub enum ChunkMsg {
    /// New audio samples for the current slot. Variable length so the
    /// final chunk of a WAV can be shorter than `CHUNK_LEN`.
    Samples(Vec<i16>),
    /// End of current slot. stage1_inc finalizes the slot and sends it
    /// downstream, then resets internal state for the next slot.
    SlotEnd {
        wav_idx: usize,
        total_samples: usize,
    },
    /// **Silent reset** — discard in-progress spec/audio without
    /// emitting a Slot or SpecBundle. Sent by `audio::capture_thread`
    /// when the operator presses BtnA (manual slot-sync mark). The
    /// previous in-progress slot may contain mixed pre-/post-BtnA
    /// audio that would produce a misleading DT estimate if decoded,
    /// so we throw it away silently and start fresh from the very
    /// next `Samples` chunk. Phase 1.7.4-Stick (2026-05-17).
    SlotResetMark,
}

/// Spectrogram + per-half allsums, sent by stage1_inc as soon as the
/// last FFT pair (pair 92) finalizes — typically ~200 ms before SlotEnd.
/// Lets main start stage 2 (`coarse_sync_with_allsum`) during the tail
/// of audio capture, so that by the time `Slot` arrives main only has
/// pass 2 + stage 3 left.
///
/// **Phase C audio-tail speculation** (2026-05-21): also carries a
/// raw pointer to the in-progress audio buffer plus a snapshot of
/// the fill position at the time of emit, so main can speculatively
/// run pass-2 + early stage-3 on candidates whose Goertzel window
/// already fits within the captured audio (most do at SpecBundle
/// time). The pointer aliases the `Slot.audio` Vec that arrives next
/// on `slot_q` — `stage1_inc::finalize_slot` uses `mem::replace` so
/// the heap allocation is preserved when ownership transfers to
/// `Slot`.
///
/// **Why a snapshot, not a live atomic**: the pipeline is offset by
/// one slot — by the time main receives SpecBundle K, stage1_inc has
/// already moved on to slot K+1, so any shared "current fill" atomic
/// would reflect K+1's progress (possibly 0 right after
/// `finalize_slot` resets it). The snapshot captures slot K's audio
/// fill at the moment pair 92 completed, which is deterministic
/// (≥ pair 91's audio requirement) and correctly bounds main's
/// speculative reads. Audio that arrives between emit and SlotEnd
/// (typically ~2 more chunks = ~200 ms) is unused by the speculative
/// path; it lands in the next-arriving `Slot` for any deferred
/// candidates.
///
/// **Lifetime contract**: `audio_ptr` is valid from when this
/// `SpecBundle` is delivered through `spec_q` until the matching
/// `Slot` is dropped. After receiving `Slot`, main MUST switch to
/// `slot.audio` and stop dereferencing `audio_ptr`.
pub struct SpecBundle {
    pub spec: mfsk_core::ft8::decode_block::Spectrogram,
    pub allsum_head: Vec<f32>,
    pub allsum_tail: Vec<f32>,
    pub wav_idx: usize,
    /// Raw pointer to the in-progress audio buffer in stage1_inc's
    /// `WorkerCtx::cur::audio`. Aliases the eventual `Slot.audio`.
    pub audio_ptr: *const i16,
    /// Capacity of the audio buffer (= `NMAX` samples). Snapshot
    /// reads must stay within `[0, audio_fill]`.
    pub audio_cap: usize,
    /// Snapshot of `WorkerCtx::cur::audio_fill` captured at
    /// `emit_spec_bundle` time. Always ≥ the audio requirement of
    /// pair 91 (177 600 samples = 14.8 s @ 12 kHz).
    pub audio_fill: usize,
}

// SAFETY: `audio_ptr` references a heap allocation owned first by
// stage1_inc's `WorkerCtx::cur::audio` (until SlotEnd), then by the
// matching `Slot.audio` after `mem::replace`. The queue hand-off
// from stage1_inc to main establishes happens-before for the
// SpecBundle fields and the writes to the audio prefix
// `[0, audio_fill]`.
unsafe impl Send for SpecBundle {}

/// Audio + slot metadata, sent by stage1_inc at SlotEnd. Pairs with the
/// `SpecBundle` for the same `wav_idx` to drive pass 2 / stage 3.
pub struct Slot {
    pub audio: Vec<i16>,
    pub wav_idx: usize,
    pub inc_total_us: i64,
    /// `esp_timer_get_time()` captured at the start of
    /// `finalize_slot` in stage1_inc — i.e. the moment SlotEnd
    /// arrived. Used by the decode-pipeline log to measure how much
    /// of the Phase C speculative window ran before SlotEnd vs after.
    pub slotend_us: i64,
}

/// Streaming-waterfall tick — emitted by stage1_inc once per FFT pair
/// (~80 ms at NSPS/2 step), if a WF queue was supplied to `spawn`.
/// `row` is the pair's spectrogram column already decimated to the
/// host's screen width (135) and palette-indexed (0..15) so the UI
/// thread does no floating-point work in the redraw path.
///
/// Sent on a depth-8 queue so consumers can briefly fall behind
/// without blocking stage1_inc; if the consumer never drains, the
/// queue saturates and the next emit is dropped (no back-pressure on
/// the audio path).
pub const WF_ROW_LEN: usize = 135;
pub struct WfTick {
    pub pair_idx: u8,
    pub row: [u8; WF_ROW_LEN],
}

/// Create a depth-N FreeRTOS queue carrying `*mut ChunkMsg` pointers.
pub fn create_chunk_queue(depth: u32) -> QueueHandle_t {
    create_ptr_queue::<ChunkMsg>(depth)
}

/// Create a depth-N FreeRTOS queue carrying `*mut Slot` pointers.
pub fn create_slot_queue(depth: u32) -> QueueHandle_t {
    create_ptr_queue::<Slot>(depth)
}

/// Create a depth-N FreeRTOS queue carrying `*mut SpecBundle` pointers.
pub fn create_spec_queue(depth: u32) -> QueueHandle_t {
    create_ptr_queue::<SpecBundle>(depth)
}

/// Create a depth-N FreeRTOS queue carrying `*mut WfTick` pointers.
pub fn create_wf_queue(depth: u32) -> QueueHandle_t {
    create_ptr_queue::<WfTick>(depth)
}

fn create_ptr_queue<T>(depth: u32) -> QueueHandle_t {
    let q = unsafe {
        xQueueGenericCreate(
            depth,
            core::mem::size_of::<*mut T>() as u32,
            QUEUE_TYPE_BASE,
        )
    };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

/// Send a heap-allocated message through a queue, transferring
/// ownership to the receiver. Blocks if the queue is full.
pub fn send_box<T>(q: QueueHandle_t, boxed: Box<T>) {
    let raw: *mut T = Box::into_raw(boxed);
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&raw as *const *mut T) as *const core::ffi::c_void,
            PORT_MAX_DELAY,
            QUEUE_SEND_TO_BACK,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueGenericSend failed: {r}");
}

/// Non-blocking send. Returns the boxed message back if the queue is
/// full so the caller can drop it (= produce-and-drop pattern, used
/// by stage1_inc for streaming `WfTick` so a slow UI consumer never
/// stalls the audio path).
pub fn try_send_box<T>(q: QueueHandle_t, boxed: Box<T>) -> Result<(), Box<T>> {
    let raw: *mut T = Box::into_raw(boxed);
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&raw as *const *mut T) as *const core::ffi::c_void,
            0,
            QUEUE_SEND_TO_BACK,
        )
    };
    if r == PD_PASS {
        Ok(())
    } else {
        // SAFETY: xQueueGenericSend with timeout=0 either takes the
        // pointer or rejects without storing it; on rejection we still
        // own `raw` and rebox to drop on the caller's side.
        Err(unsafe { Box::from_raw(raw) })
    }
}

/// Non-blocking receive — returns `Some(box)` if a message was
/// pending, `None` if the queue was empty. Used by capture_thread's
/// BtnA-reset path to drain stale pre-reset chunks before signalling
/// stage1_inc to discard its in-progress spec.
pub fn try_recv_box<T>(q: QueueHandle_t) -> Option<Box<T>> {
    let mut raw: *mut T = ptr::null_mut();
    let r = unsafe {
        xQueueReceive(
            q,
            (&mut raw as *mut *mut T) as *mut core::ffi::c_void,
            0,
        )
    };
    if r == PD_PASS {
        debug_assert!(!raw.is_null());
        Some(unsafe { Box::from_raw(raw) })
    } else {
        None
    }
}

/// Receive a boxed message from a queue, taking ownership. Blocks
/// until a message is available.
pub fn recv_box<T>(q: QueueHandle_t) -> Box<T> {
    let mut raw: *mut T = ptr::null_mut();
    let r = unsafe {
        xQueueReceive(
            q,
            (&mut raw as *mut *mut T) as *mut core::ffi::c_void,
            PORT_MAX_DELAY,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueReceive failed: {r}");
    debug_assert!(!raw.is_null());
    unsafe { Box::from_raw(raw) }
}
