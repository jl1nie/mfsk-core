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
/// **Phase C audio-tail speculation** (2026-05-21): also carries an
/// owned snapshot of the audio captured so far (`audio_prefix`,
/// `audio_fill` samples at the moment `emit_spec_bundle` ran). Main
/// runs speculative pass-2 + early stage-3 on candidates whose
/// Goertzel window fits inside `audio_prefix`; the rest defer to
/// the full `Slot.audio` that arrives next on `slot_q`.
///
/// **Why a copy, not a shared pointer**: prior versions handed main
/// a raw pointer into stage1_inc's still-being-written
/// `WorkerCtx::cur::audio`. Even though stage1_inc only wrote past
/// `audio_fill`, the implicit `&mut` borrow of `cur.audio` it held
/// while doing so technically aliased main's raw-pointer-derived
/// `&[i16]` under stacked borrows (Gemini PR #123 round-4 review).
/// Copying the prefix costs ~2 ms PSRAM-to-PSRAM (~180 KB at
/// ~80 MB/s) per slot — negligible vs the ~800 ms post-SlotEnd
/// budget — and eliminates the aliasing concern entirely.
pub struct SpecBundle {
    pub spec: mfsk_core::ft8::decode_block::Spectrogram,
    pub allsum_head: Vec<f32>,
    pub allsum_tail: Vec<f32>,
    pub wav_idx: usize,
    /// Raw pointer into stage1_inc's currently-frozen audio buffer
    /// (same allocation as the matching `Slot.audio_ptr` that will
    /// arrive on `slot_q` next).
    pub audio_ptr: *const i16,
    /// Snapshot of `cur.audio_fill` at emit time. Always ≥
    /// pair-86's audio requirement (168 000 samples = 14.0 s @
    /// 12 kHz) when emit fires at `SPEC_EMIT_PAIR`.
    pub audio_len: usize,
}

// SAFETY: `audio_ptr` references a long-lived `AudioBuf` allocation
// owned by stage1_inc. Same contract as `Slot::audio` — consumer
// reads are valid for the lifetime of this `SpecBundle` and the
// matching `Slot` that arrives next on `slot_q`.
unsafe impl Send for SpecBundle {}

impl SpecBundle {
    /// Last sample index that
    /// `fill_symbol_spectra_goertzel(audio, freq_hz, dt_sec, ..)`
    /// reads for a candidate at `dt_sec`. Thin re-export of
    /// `mfsk_core::ft8::decode_block::goertzel_window_end_sample`
    /// so the formula and its constants
    /// (`TX_START_OFFSET_S` / `SAMPLE_RATE_HZ` / `NN` / `NSPS`)
    /// live exactly once in mfsk-core and can't drift.
    ///
    /// Used by Phase-C decode-pipeline consumers to decide whether
    /// a candidate's Goertzel window fits inside the SpecBundle
    /// audio snapshot (`ready`) or must defer to the post-SlotEnd
    /// path (`deferred`).
    pub fn audio_end_for_dt(dt_sec: f32) -> usize {
        mfsk_core::ft8::decode_block::goertzel_window_end_sample(dt_sec)
    }

    /// Borrow the snapshot prefix as a slice. SAFETY contract is
    /// the same as [`Slot::audio`].
    pub fn audio_prefix(&self) -> &[i16] {
        unsafe { core::slice::from_raw_parts(self.audio_ptr, self.audio_len) }
    }
}

/// Audio + slot metadata, sent by stage1_inc at SlotEnd. Pairs with the
/// `SpecBundle` for the same `wav_idx` to drive pass 2 / stage 3.
///
/// **Phase C+ double-buffer**: `audio_ptr` references one of
/// stage1_inc's two long-lived `AudioBuf` allocations. It stays
/// valid until stage1_inc next swaps `fill_idx` back to this
/// buffer (= at SlotEnd of the *next* slot, ≥ 15 s away). The
/// consumer (decode_pipeline / main) MUST drop the Slot before
/// that deadline; in steady-state operation post_slotend ≪ 15 s,
/// so the requirement is trivially met.
pub struct Slot {
    /// Raw pointer to the just-completed slot's audio. Same backing
    /// allocation as the matching `SpecBundle.audio_ptr` — both
    /// point into stage1_inc's currently-frozen `AudioBuf`.
    pub audio_ptr: *const i16,
    /// Number of valid samples at `audio_ptr` (usually
    /// `NMAX = 180_000`; smaller on under-fed / BtnA-reset slots).
    pub audio_len: usize,
    pub wav_idx: usize,
    pub inc_total_us: i64,
    /// `esp_timer_get_time()` captured at the start of
    /// `finalize_slot` in stage1_inc — i.e. the moment SlotEnd
    /// arrived. Used by the decode-pipeline log to measure how much
    /// of the Phase C speculative window ran before SlotEnd vs after.
    pub slotend_us: i64,
}

// SAFETY: `audio_ptr` references a long-lived `AudioBuf` allocation
// owned by stage1_inc. Consumer-only access (read-only) is sound
// for the lifetime of the Slot, per `AudioBuf`'s aliasing contract.
unsafe impl Send for Slot {}

impl Slot {
    /// Borrow the slot's audio as a slice. SAFETY: this method
    /// borrows `self`, which keeps the Slot live and therefore
    /// keeps the audio buffer reserved for the consumer.
    pub fn audio(&self) -> &[i16] {
        // SAFETY: `audio_ptr` + `audio_len` came from a valid
        // `AudioBuf` allocation owned by stage1_inc and remain
        // valid for `self`'s lifetime per `AudioBuf`'s rules.
        unsafe { core::slice::from_raw_parts(self.audio_ptr, self.audio_len) }
    }
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
