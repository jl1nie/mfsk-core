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
pub struct SpecBundle {
    pub spec: mfsk_core::ft8::decode_block::Spectrogram,
    pub allsum_head: Vec<f32>,
    pub allsum_tail: Vec<f32>,
    pub wav_idx: usize,
}

/// Audio + slot metadata, sent by stage1_inc at SlotEnd. Pairs with the
/// `SpecBundle` for the same `wav_idx` to drive pass 2 / stage 3.
pub struct Slot {
    pub audio: Vec<i16>,
    pub wav_idx: usize,
    pub inc_total_us: i64,
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
