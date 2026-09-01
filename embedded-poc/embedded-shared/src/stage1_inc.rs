//! Incremental stage-1 spectrogram task.
//!
//! Receives `ChunkMsg` from wav_sim, accumulates audio for one slot,
//! advances FFT pairs as samples arrive, finalizes the slot's spec +
//! per-half allsums on `SlotEnd`, and sends the completed `Slot` to
//! main via SLOT_Q.
//!
//! All slot state lives task-local. No `static` shared with other tasks.
//! Old API (`STATE`, `AUDIO_FILL`, `PAIR_DONE`, `mark_slot_boundary`,
//! `take_spec_and_allsum`, `push_chunk`, `init`) is gone.

use core::ffi::c_void;
use core::ptr;

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use esp_idf_svc::sys::{esp_timer_get_time, xTaskCreatePinnedToCore, QueueHandle_t};

use mfsk_core::engine::fft::{Fft16, FftPlanner16};
use num_complex::Complex;

use crate::pipeline::{recv_box, send_box, ChunkMsg, Slot, SpecBundle};

const PD_PASS: i32 = 1;

// FT8 constants (replicated from mfsk_core::ft8::decode_block).
const NSPS: usize = 1_920;
const NSTEP: usize = NSPS / 2; // 960
const NMAX: usize = 180_000;
const NTONES: usize = 8;
// Pulled from mfsk_core so the spec layout stays in lockstep with the
// downstream pass2/stage3 expectations. If this drifts, coarse_sync
// candidates land in the wrong frequency bins and the entire slot
// silently produces zero results (recall=0). Keep `assert!` below.
const NFFT_SPEC: usize = mfsk_core::ft8::decode_block::NFFT_SPEC;
const _: () = assert!(
    NFFT_SPEC == 3_840,
    "stage1_inc NFFT_SPEC must match mfsk_core (3840)"
);
const FP_SPEC_SHIFT: u32 = 12;
const TONE_SPACING_HZ: f32 = 6.25;
const SAMPLE_RATE_HZ: f32 = 12_000.0;
const N_TIME: usize = NMAX / NSTEP - 3; // 184
const N_PAIRS: usize = N_TIME / 2; // 92
const TARGET_PEAK: i32 = (NFFT_SPEC * 2) as i32;

// Phase C++ (2026-05-21): emit SpecBundle as early as we can
// trade off against block-2 coarse_sync score for dt near ±jz.
//
// `nstep-half` (NSSY=2), jstrt=6, SYNC_LAG_S=1.0 → jz=13. The
// strict-correct `needed_m` bound is 162 + 13 = 175 (block-2 last
// Costas at m=162, plus max lag), which pair 87 completes.
// Emitting at pair 86 (m=0..173) leaves m=174..175 zero in both
// the spec and the per-half allsum, which silently flattens
// block-2's contribution for candidates whose lag lands on ±jz
// (= dt near ±1.0 s). On NTP-synced operation dt is well inside
// ±0.5 s, and the dt median auto-sync (`time_sync::record_decode_dt`)
// re-anchors the slot phase so the lag range can stay tight in
// steady state — taking the pair-86 emit, the 160 ms audio-tail
// overlap gain shows up directly as -160 ms post_slotend.
//
// **Semantics of this constant** (Gemini PR #123 round-14 misread
// guard): the value is compared `next_pair >= SPEC_EMIT_PAIR` in
// `advance_pairs` AFTER each pair's `compute_pair_into` increments
// `next_pair`. So `SPEC_EMIT_PAIR = 87` means "emit when 87 pairs
// have been processed" = "emit just after pair index 86 finishes"
// = "emit with m=0..173 filled" (pair k fills m=2k, 2k+1, so
// pairs 0..86 fill m=0..173). It does NOT mean "emit at pair 87
// producing m=174..175".
//
// Pairs 87..91 (m=174..183) still get computed when `wf_q.is_some()`
// to keep the WF flowing through the slot tail; `wf_q = None`
// (bench) skips them via the `pair_limit` branch in advance_pairs.
const SPEC_EMIT_PAIR: usize = 87; // emit when next_pair == 87 (= pairs 0..86 done, m=0..173 valid)
const _: () = assert!(SPEC_EMIT_PAIR <= N_PAIRS, "SPEC_EMIT_PAIR > N_PAIRS");

// Phase-E2 per-half allsum parameters (matches dual_core
// coarse_sync_split_with_allsum band 100..3000 split at 1550).
const ALLSUM_FREQ_MIN: f32 = 100.0;
const ALLSUM_FREQ_MAX: f32 = 3_000.0;
const ALLSUM_FREQ_MID: f32 = 0.5 * (ALLSUM_FREQ_MIN + ALLSUM_FREQ_MAX);

fn n_freq_for(max_freq_hz: f32) -> usize {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    (((band_top_hz / df).ceil() as usize) + 1).min(NFFT_SPEC / 2)
}

fn band_for(freq_min: f32, freq_max: f32, spec_n_freq: usize) -> (usize, usize) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tone_step_bins = TONE_SPACING_HZ / df;
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(spec_n_freq.saturating_sub(max_tone_off));
    (ia, ib - ia + 1)
}

/// Long-lived NMAX-element i16 audio buffer in PSRAM.
///
/// Phase C+ owns two of these in `WorkerCtx::audio_bufs` and toggles
/// `WorkerCtx::fill_idx` at SlotEnd so the just-completed buffer's
/// pointer can be handed to main via Slot / SpecBundle while
/// stage1_inc starts writing the next slot into the other buffer.
/// This eliminates the per-slot ~340 KB Vec alloc/free + the
/// ~170 KB audio_prefix memcpy that the prior Vec-based design
/// required to keep Rust's aliasing rules satisfied.
///
/// SAFETY: each buffer is logically owned by exactly one of the
/// stage1_inc / consumer pair at any moment, but Rust's borrow
/// checker can't express that across the FreeRTOS queue boundary
/// — the queue carries a raw `*const i16`, not a reference. The
/// invariant the implementation guarantees is:
///   * stage1_inc only WRITES the buffer indexed by `fill_idx`.
///   * The other buffer is read-only from stage1_inc's view, and
///     must have been emitted to main exactly once (via the
///     previous slot's `SpecBundle` + `Slot`) before its index is
///     re-promoted to `fill_idx`.
///   * Main must DROP the previous `Slot` before stage1_inc swaps
///     `fill_idx` back to that buffer. Steady-state pipeline has
///     post-SlotEnd ≪ 15 s so the constraint is easily met; a
///     debug build adds a check in `swap_fill_idx`.
struct AudioBuf {
    /// Box<[i16; NMAX]> reborrowed as a raw pointer so neither
    /// stage1_inc's `&mut WorkerCtx` nor main's `*const i16`
    /// derives an exclusive Rust borrow of the underlying slice.
    /// `Box::into_raw` is paired with `Box::from_raw` in `Drop`.
    ptr: *mut i16,
    /// Generation tag bumped each time `swap_fill_idx` promotes
    /// this buffer back to the fill side. Currently informational
    /// (the queue-FIFO order is the primary guard); a future
    /// `try_promote` could refuse to swap until a corresponding
    /// drop signal lands.
    gen: u32,
}

impl AudioBuf {
    fn new() -> Self {
        // `Box::new([0i16; NMAX])` puts the 360 KB array temporarily
        // on the stack before moving it to the heap and can blow
        // the FreeRTOS task stack on ESP32. Allocate via Vec, which
        // goes straight to the heap (PSRAM under
        // `SPIRAM_MALLOC_ALWAYSINTERNAL=4096`), then convert to a
        // sized `Box<[i16; NMAX]>` *before* taking a raw pointer.
        // The fat→thin cast detour
        // `Box::into_raw(Box<[i16]>) as *mut i16` paired with
        // `Box::from_raw(ptr as *mut [i16; NMAX])` in `Drop` mixes
        // fat- and thin-pointer Box invariants and is technically
        // UB (Gemini PR #123 round-17). Convert via `TryFrom`
        // first so both ends speak `Box<[i16; NMAX]>`.
        let v: Vec<i16> = alloc::vec![0i16; NMAX];
        let boxed_slice: Box<[i16]> = v.into_boxed_slice();
        let boxed_array: Box<[i16; NMAX]> = boxed_slice
            .try_into()
            .expect("AudioBuf: boxed slice length must equal NMAX");
        let ptr_arr: *mut [i16; NMAX] = Box::into_raw(boxed_array);
        // Store as a thin element pointer for ergonomic indexed
        // access in stage1_inc; cast it back to the array type in
        // Drop so `Box::from_raw` sees the same type it gave us.
        Self {
            ptr: ptr_arr as *mut i16,
            gen: 0,
        }
    }
}

impl Drop for AudioBuf {
    fn drop(&mut self) {
        // SAFETY: `ptr` was produced by `Box::into_raw(Box<[i16;
        // NMAX]>)` in `new()` — we reconstruct the same Box type
        // here (cast from `*mut i16` back to `*mut [i16; NMAX]`,
        // valid because that was the original provenance) and let
        // it drop, freeing the heap allocation.
        unsafe {
            let _ = Box::from_raw(self.ptr as *mut [i16; NMAX]);
        }
    }
}

// SAFETY: we treat `AudioBuf` as a pointer to a logically-owned
// allocation. The aliasing contract is documented above and
// enforced at the FreeRTOS-queue ownership-transfer boundary.
unsafe impl Send for AudioBuf {}

/// Task-local state: one in-flight slot + invariant resources.
struct WorkerCtx {
    chunk_q: QueueHandle_t,
    slot_q: QueueHandle_t,
    spec_q: QueueHandle_t,
    /// Optional streaming-WF queue. When `Some`, the worker emits a
    /// `WfTick` per FFT pair (≈ 80 ms cadence) so the UI can show a
    /// flowing waterfall during capture. Non-blocking send: a slow
    /// consumer just drops ticks instead of stalling stage1.
    wf_q: Option<QueueHandle_t>,
    n_freq: usize,
    head_ia: usize,
    head_n_freq: usize,
    tail_ia: usize,
    tail_n_freq: usize,
    /// FFT planner and forward FFT object — created once, reused
    /// across all slots. The plan_forward call returns a Box<dyn Fft16>
    /// so its allocation is one-time.
    _fft_planner: Box<dyn FftPlanner16>,
    fft: Box<dyn Fft16>,
    fft_buf: Vec<Complex<i16>>,
    /// Double-buffered audio. `audio_bufs[fill_idx]` is the buffer
    /// stage1_inc currently writes into; the other was just handed
    /// to main via the previous SlotEnd. Indices are swapped inside
    /// `finalize_slot` so the just-completed buffer's pointer is
    /// what travels in `Slot.audio_ptr`.
    audio_bufs: [AudioBuf; 2],
    fill_idx: usize,
    /// Accumulating slot metadata — counters / spec / allsum / etc.
    /// The audio bytes themselves now live in `audio_bufs[fill_idx]`.
    cur: SlotInProgress,
}

impl WorkerCtx {
    /// Pointer to the buffer stage1_inc currently writes into.
    #[inline]
    fn fill_ptr(&self) -> *mut i16 {
        self.audio_bufs[self.fill_idx].ptr
    }
}

/// State of the slot currently being assembled.
struct SlotInProgress {
    audio_fill: usize,
    /// Spec/allsum buffers — moved out into a `SpecBundle` and sent
    /// downstream as soon as the last pair finalizes (typically ~200 ms
    /// before SlotEnd). After `spec_sent` becomes true these are taken
    /// (`mem::take`'d) and `next_pair == N_PAIRS` so they're not touched
    /// by subsequent advance_pairs calls.
    spec: Vec<u16>,
    allsum_head: Vec<f32>,
    allsum_tail: Vec<f32>,
    spec_sent: bool,
    next_pair: usize,
    shift: u32,
    shift_locked: bool,
    peak_abs: i32,
    inc_total_us: i64,
}

impl SlotInProgress {
    fn new(n_freq: usize, head_n_freq: usize, tail_n_freq: usize) -> Self {
        Self {
            audio_fill: 0,
            spec: vec![0u16; n_freq * N_TIME],
            allsum_head: vec![0f32; head_n_freq * N_TIME],
            allsum_tail: vec![0f32; tail_n_freq * N_TIME],
            spec_sent: false,
            next_pair: 0,
            shift: 0,
            shift_locked: false,
            peak_abs: 1,
            inc_total_us: 0,
        }
    }
}

/// Spawn the stage1_inc worker task. The task receives `ChunkMsg` from
/// `chunk_q`, builds spec / allsum / audio incrementally, and emits
///   - `SpecBundle` on `spec_q` as soon as the last FFT pair finalizes
///     (≈ 200 ms before SlotEnd) so main can start stage 2 during the
///     tail of capture
///   - `Slot` on `slot_q` at SlotEnd so main can run pass 2 / stage 3
pub fn spawn(chunk_q: QueueHandle_t, slot_q: QueueHandle_t, spec_q: QueueHandle_t) {
    spawn_with_wf(chunk_q, slot_q, spec_q, None)
}

/// Variant of [`spawn`] that wires up the optional streaming-WF
/// queue. Emits one [`pipeline::WfTick`] per FFT pair (≈ 80 ms) for
/// the UI thread to render a continuously-flowing waterfall.
pub fn spawn_with_wf(
    chunk_q: QueueHandle_t,
    slot_q: QueueHandle_t,
    spec_q: QueueHandle_t,
    wf_q: Option<QueueHandle_t>,
) {
    let n_freq = n_freq_for(3_000.0);
    let (head_ia, head_n_freq) = band_for(ALLSUM_FREQ_MIN, ALLSUM_FREQ_MID, n_freq);
    let (tail_ia, tail_n_freq) = band_for(ALLSUM_FREQ_MID, ALLSUM_FREQ_MAX, n_freq);

    let mut fft_planner = mfsk_core::engine::fft::default_planner_16();
    let fft = fft_planner.plan_forward(NFFT_SPEC);
    let fft_buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0i16); NFFT_SPEC];

    let ctx = Box::new(WorkerCtx {
        chunk_q,
        slot_q,
        spec_q,
        wf_q,
        n_freq,
        head_ia,
        head_n_freq,
        tail_ia,
        tail_n_freq,
        _fft_planner: fft_planner,
        fft,
        fft_buf,
        audio_bufs: [AudioBuf::new(), AudioBuf::new()],
        fill_idx: 0,
        cur: SlotInProgress::new(n_freq, head_n_freq, tail_n_freq),
    });
    let ctx_ptr = Box::into_raw(ctx) as *mut c_void;

    // Stack: 12 KB. Originally 16 KB but that fails to spawn on S3
    // builds with BLE NimBLE enabled — BT controller + NimBLE host
    // tasks eat ~15 KB internal DRAM and fragment what's left.
    // Worker_main's actual frame is modest (FFT buf is in ctx, not
    // stack); 12 KB has measured ~3 KB headroom on the busiest
    // qso3_busy slot. If a future feature pushes the worker frame
    // deeper, bump this back to 16 KB and revisit the BLE config.
    // Phase 1.7.5-Stick (2026-05-17): prio 6 (above dual_core 5) so
    // stage1_inc can preempt dsp_worker briefly when Samples arrive
    // during BP. Without this, BP for slot N (running on dsp_worker
    // prio 5, same core 1) starves stage1_inc's FFT for slot N+1 →
    // SpecBundle[N+1] late → BP[N+1] cascades. Each Samples chunk
    // costs ~7-14 ms of preemption per 100 ms = ≤14% of core 1's BP
    // time. BP finishes ~50 ms later as a result, well worth it for
    // the steady-state pipeline.
    let r = unsafe {
        xTaskCreatePinnedToCore(
            Some(worker_main),
            c"stage1_inc".as_ptr(),
            12288,
            ctx_ptr,
            6, // above dual_core (5) — preempts BP briefly per chunk
            ptr::null_mut(),
            1, // APP_CPU
        )
    };
    assert_eq!(
        r, PD_PASS,
        "xTaskCreatePinnedToCore(stage1_inc) failed: {r}"
    );
    log::info!(
        "stage1_inc: spawned (APP_CPU prio 6 — preempts dsp_worker); n_time={} n_pairs={} n_freq={}",
        N_TIME,
        N_PAIRS,
        n_freq
    );
}

extern "C" fn worker_main(arg: *mut c_void) {
    let ctx_ptr = arg as *mut WorkerCtx;
    let ctx: &mut WorkerCtx = unsafe { &mut *ctx_ptr };
    log::info!("stage1_inc: worker entered");
    loop {
        let msg = recv_box::<ChunkMsg>(ctx.chunk_q);
        match *msg {
            ChunkMsg::Samples(samples) => {
                ingest_samples(ctx, &samples);
            }
            ChunkMsg::SlotEnd {
                wav_idx,
                total_samples,
            } => {
                finalize_slot(ctx, wav_idx, total_samples);
            }
            ChunkMsg::SlotResetMark => {
                reset_in_progress(ctx);
            }
        }
    }
}

/// Silent reset — operator BtnA mid-slot. Discard the partially-
/// filled in-progress spec/audio without emitting a Slot or
/// SpecBundle (no consumer downstream would benefit from the
/// half-built spec). Phase 1.7.4-Stick (2026-05-17).
fn reset_in_progress(ctx: &mut WorkerCtx) {
    ctx.cur = SlotInProgress::new(ctx.n_freq, ctx.head_n_freq, ctx.tail_n_freq);
    log::info!("stage1_inc: SlotResetMark — discarded in-progress spec");
}

fn ingest_samples(ctx: &mut WorkerCtx, samples: &[i16]) {
    let off = ctx.cur.audio_fill;
    if off + samples.len() > NMAX {
        // More than one slot worth of audio without a SlotEnd — should
        // not happen in normal operation. Drop excess.
        log::warn!(
            "stage1_inc: samples overflow (off={off}, n={}); dropping",
            samples.len()
        );
        return;
    }
    // SAFETY: `fill_ptr()` returns the heap pointer of the buffer
    // currently owned (write-side) by stage1_inc. By the
    // `AudioBuf` invariant documented at its definition, no other
    // task is reading this index range concurrently. The dst range
    // `[off, off+samples.len())` was bounds-checked above.
    let dst = unsafe { ctx.fill_ptr().add(off) };
    unsafe {
        core::ptr::copy_nonoverlapping(samples.as_ptr(), dst, samples.len());
    }
    {
        let cur = &mut ctx.cur;
        for &s in samples {
            let a = (s as i32).unsigned_abs() as i32;
            if a > cur.peak_abs {
                cur.peak_abs = a;
            }
        }
        cur.audio_fill = off + samples.len();
    }
    advance_pairs(ctx);
}

fn advance_pairs(ctx: &mut WorkerCtx) {
    let t0 = unsafe { esp_timer_get_time() };
    let audio_len = ctx.cur.audio_fill;

    if !ctx.cur.shift_locked && audio_len >= 12_000 {
        let mut shift: u32 = 0;
        while ctx.cur.peak_abs << shift < TARGET_PEAK && shift < 8 {
            shift += 1;
        }
        // Auto-gain shift only — rectangular window does not need the
        // +1 Hann coherent-gain compensation (host dropped it at the
        // NFFT=3840 migration; see decode_block.rs:419).
        ctx.cur.shift = shift.min(8);
        ctx.cur.shift_locked = true;
    }
    if !ctx.cur.shift_locked {
        return;
    }

    // Pair-loop limit: when no streaming-waterfall consumer is
    // wired, stop computing once the SpecBundle's needed_m range is
    // covered. With `SPEC_EMIT_PAIR = 87` the loop exits after
    // pair index 86 finishes, skipping pairs 87..91 (= m=174..183,
    // 5 pairs × ~10 ms each); those rows contribute nothing to
    // coarse_sync and would otherwise compute into a buffer that
    // `emit_spec_bundle`'s `mem::replace` immediately discards
    // (Gemini PR #123 round-5 / round-18). Production apps with
    // `wf_q = Some` still need pairs 87..91 to keep the waterfall
    // flowing through the slot tail, so they keep the original
    // N_PAIRS limit.
    let pair_limit = if ctx.wf_q.is_some() {
        N_PAIRS
    } else {
        SPEC_EMIT_PAIR
    };
    loop {
        let j = ctx.cur.next_pair;
        if j >= pair_limit {
            break;
        }
        let j_a = 2 * j;
        let j_b = j_a + 1;
        let need = j_b * NSTEP + NSPS;
        if need > audio_len {
            break;
        }
        compute_pair_into(ctx, j_a, j_b);
        ctx.cur.next_pair = j + 1;
    }

    // Phase C+ (2026-05-21): emit SpecBundle as soon as the last
    // pair that coarse_sync's needed_m bounds depends on completes
    // (= pair `SPEC_EMIT_PAIR - 1`), not at the very last pair. This
    // shifts ~720 ms of audio-tail wallclock from "stage1_inc is
    // still finishing pairs" into "main can run coarse_sync +
    // speculative pass-2/stage-3".
    if ctx.cur.next_pair >= SPEC_EMIT_PAIR && !ctx.cur.spec_sent {
        emit_spec_bundle(ctx);
    }

    let t1 = unsafe { esp_timer_get_time() };
    ctx.cur.inc_total_us += t1 - t0;
}

fn emit_spec_bundle(ctx: &mut WorkerCtx) {
    let n_freq = ctx.n_freq;
    let head_n = ctx.head_n_freq;
    let tail_n = ctx.tail_n_freq;
    // The post-emit "fresh" buffers swapped into `ctx.cur` are only
    // touched if `compute_pair_into` runs for the remaining pairs
    // (87..91). Two conditions must BOTH hold for an alloc to be
    // worthwhile:
    //   * `wf_q.is_some()` — otherwise `pair_limit` in
    //     `advance_pairs` stops the loop at SPEC_EMIT_PAIR.
    //   * `next_pair < N_PAIRS` — otherwise we're emitting from
    //     `finalize_slot`'s late-emit fallback (the slot was
    //     under-fed; all pairs already done) and the fresh buffer
    //     would be discarded by the `mem::replace` later in the
    //     same `finalize_slot` call.
    // Gemini PR #123 round-5 & round-14 reviews.
    let needs_buffers = ctx.wf_q.is_some() && ctx.cur.next_pair < N_PAIRS;
    let (new_spec, new_head, new_tail) = if needs_buffers {
        (
            vec![0u16; n_freq * N_TIME],
            vec![0f32; head_n * N_TIME],
            vec![0f32; tail_n * N_TIME],
        )
    } else {
        (Vec::new(), Vec::new(), Vec::new())
    };
    let spec = core::mem::replace(&mut ctx.cur.spec, new_spec);
    let head = core::mem::replace(&mut ctx.cur.allsum_head, new_head);
    let tail = core::mem::replace(&mut ctx.cur.allsum_tail, new_tail);
    // Phase C+ double-buffer (round-11): hand main a raw pointer
    // to the fill-side audio buffer + the current fill count. No
    // copy. The pointer remains valid until the next time stage1_inc
    // promotes this index back to `fill_idx` (= next SlotEnd at the
    // earliest); main must drop the matching `Slot` before then.
    let audio_ptr: *const i16 = ctx.fill_ptr();
    let audio_len = ctx.cur.audio_fill;
    let bundle = Box::new(SpecBundle::new(
        mfsk_core::ft8::decode_block::Spectrogram::from_parts(n_freq, N_TIME, spec),
        head,
        tail,
        // wav_idx is only known at SlotEnd; main matches SpecBundle to
        // Slot by FIFO order of receipt, so this is informational only.
        usize::MAX,
        audio_ptr,
        audio_len,
    ));
    send_box(ctx.spec_q, bundle);
    ctx.cur.spec_sent = true;
}

/// The per-pair body: pack two audio rows into one complex transform,
/// run it, demux the result into two spectrogram rows.
///
/// Lifted out of [`compute_pair_into`] unchanged so `pair_kernel_f32`
/// can be the same computation in the other scalar and the A/B in
/// [`scalar_ab`] times *this* code rather than a copy of it
/// (issue #349 step 2 — the batch spectrogram had been measured, the
/// streaming one this receiver actually runs had not).
#[allow(clippy::too_many_arguments)]
fn pair_kernel_i16(
    fft: &dyn Fft16,
    buf: &mut [Complex<i16>],
    audio: &[i16],
    ia_a: usize,
    ia_b: usize,
    shift: u32,
    n_freq: usize,
    spec: &mut [u16],
    row_a: usize,
    row_b: usize,
) {
    // Modular wrap (NFFT_SPEC=3840 isn't a power of two so bitmask
    // would alias the high bins). `kn = (NFFT - k) mod NFFT` collapses
    // k=0 to 0 (DC bin is real), as the demux formula expects.

    // Pack audio[ia_a..+NSPS] real, audio[ia_b..+NSPS] imag, both
    // **Rectangular window** — matches host `compute_spectrogram` and
    // WSJT-X `sync8.f90` after the NFFT=3840 migration. Hann was
    // dropped on the host side because at integer tone alignment
    // (tone_step_bins = 2.0 exactly) the rectangular-window sidelobes
    // do not leak onto adjacent FT8 tones; Hann's coherent gain 0.5
    // costs ~3 dB SNR and spreads each tone's mainlobe across 2 bins,
    // negating the integer-bin advantage. See decode_block.rs:107.
    {
        let buf = &mut *buf;
        for k in 0..NFFT_SPEC {
            let re = if k < NSPS && ia_a + k < NMAX {
                let raw = audio[ia_a + k] as i32;
                (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            } else {
                0
            };
            let im = if k < NSPS && ia_b + k < NMAX {
                let raw = audio[ia_b + k] as i32;
                (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            } else {
                0
            };
            buf[k] = Complex::new(re, im);
        }
    }
    fft.process(buf);

    // Demux pair → spec rows.
    // D4: DC bin hoisted (kn=0 special case removed from inner loop);
    // 4× unrolled main loop; per-square u32 cast avoids i32 addition
    // overflow at ±32768 edges (each square ≤ 2^30, sum ≤ 2^31 < u32::MAX).
    {
        let buf = &*buf;
        let spec = &mut *spec;

        // k=0: DC bin — kn=0 so yn=yk; b = imaginary half-band (zero-phase)
        if n_freq > 0 {
            let r0 = buf[0].re as i32;
            let i0 = buf[0].im as i32;
            spec[row_a] = ((r0 * r0) as u32 >> FP_SPEC_SHIFT).min(u16::MAX as u32) as u16;
            spec[row_b] = ((i0 * i0) as u32 >> FP_SPEC_SHIFT).min(u16::MAX as u32) as u16;
        }

        // k=1..n_freq: kn = NFFT_SPEC - k (no branch needed).
        // 4× unrolled to expose independent multiply chains to the FPU
        // pipeline (opt-level=1 does not auto-unroll on Xtensa LX7).
        let mut k = 1usize;
        while k + 4 <= n_freq {
            let yk0 = buf[k];
            let yk1 = buf[k + 1];
            let yk2 = buf[k + 2];
            let yk3 = buf[k + 3];
            let yn0 = buf[NFFT_SPEC - k];
            let yn1 = buf[NFFT_SPEC - k - 1];
            let yn2 = buf[NFFT_SPEC - k - 2];
            let yn3 = buf[NFFT_SPEC - k - 3];

            let (a0r, a0i) = (
                (yk0.re as i32 + yn0.re as i32) >> 1,
                (yk0.im as i32 - yn0.im as i32) >> 1,
            );
            let (b0r, b0i) = (
                (yk0.im as i32 + yn0.im as i32) >> 1,
                (yn0.re as i32 - yk0.re as i32) >> 1,
            );
            let (a1r, a1i) = (
                (yk1.re as i32 + yn1.re as i32) >> 1,
                (yk1.im as i32 - yn1.im as i32) >> 1,
            );
            let (b1r, b1i) = (
                (yk1.im as i32 + yn1.im as i32) >> 1,
                (yn1.re as i32 - yk1.re as i32) >> 1,
            );
            let (a2r, a2i) = (
                (yk2.re as i32 + yn2.re as i32) >> 1,
                (yk2.im as i32 - yn2.im as i32) >> 1,
            );
            let (b2r, b2i) = (
                (yk2.im as i32 + yn2.im as i32) >> 1,
                (yn2.re as i32 - yk2.re as i32) >> 1,
            );
            let (a3r, a3i) = (
                (yk3.re as i32 + yn3.re as i32) >> 1,
                (yk3.im as i32 - yn3.im as i32) >> 1,
            );
            let (b3r, b3i) = (
                (yk3.im as i32 + yn3.im as i32) >> 1,
                (yn3.re as i32 - yk3.re as i32) >> 1,
            );

            spec[row_a + k] = (((a0r * a0r) as u32 + (a0i * a0i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_b + k] = (((b0r * b0r) as u32 + (b0i * b0i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_a + k + 1] = (((a1r * a1r) as u32 + (a1i * a1i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_b + k + 1] = (((b1r * b1r) as u32 + (b1i * b1i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_a + k + 2] = (((a2r * a2r) as u32 + (a2i * a2i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_b + k + 2] = (((b2r * b2r) as u32 + (b2i * b2i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_a + k + 3] = (((a3r * a3r) as u32 + (a3i * a3i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_b + k + 3] = (((b3r * b3r) as u32 + (b3i * b3i) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;

            k += 4;
        }
        while k < n_freq {
            let yn = buf[NFFT_SPEC - k];
            let yk = buf[k];
            let (ar, ai) = (
                (yk.re as i32 + yn.re as i32) >> 1,
                (yk.im as i32 - yn.im as i32) >> 1,
            );
            let (br, bi) = (
                (yk.im as i32 + yn.im as i32) >> 1,
                (yn.re as i32 - yk.re as i32) >> 1,
            );
            spec[row_a + k] = (((ar * ar) as u32 + (ai * ai) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            spec[row_b + k] = (((br * br) as u32 + (bi * bi) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32) as u16;
            k += 1;
        }
    }
}

/// [`pair_kernel_i16`] in `f32` — the same packing, the same
/// two-rows-per-transform demux, the same rows written, with the
/// esp-dsp `f32` transform instead of the `sc16` one and no shift to
/// pick (an `f32` mantissa does not run out of range on a 16-bit
/// input, which is half of what the fixed-point path's `shift` scan
/// exists for).
///
/// Only [`scalar_ab`] calls this. It is not a second production path
/// and nothing downstream reads an `f32` spectrogram — `coarse_sync`,
/// `pass2` and the waterfall all take `&[u16]`.
#[allow(clippy::too_many_arguments)]
fn pair_kernel_f32(
    fft: &dyn mfsk_core::engine::fft::Fft,
    buf: &mut [Complex<f32>],
    audio: &[i16],
    ia_a: usize,
    ia_b: usize,
    n_freq: usize,
    spec: &mut [f32],
    row_a: usize,
    row_b: usize,
) {
    for k in 0..NFFT_SPEC {
        let re = if k < NSPS && ia_a + k < NMAX {
            audio[ia_a + k] as f32
        } else {
            0.0
        };
        let im = if k < NSPS && ia_b + k < NMAX {
            audio[ia_b + k] as f32
        } else {
            0.0
        };
        buf[k] = Complex::new(re, im);
    }
    fft.process(buf);

    if n_freq > 0 {
        spec[row_a] = buf[0].re * buf[0].re;
        spec[row_b] = buf[0].im * buf[0].im;
    }
    for k in 1..n_freq {
        let yk = buf[k];
        let yn = buf[NFFT_SPEC - k];
        let (ar, ai) = ((yk.re + yn.re) * 0.5, (yk.im - yn.im) * 0.5);
        let (br, bi) = ((yk.im + yn.im) * 0.5, (yn.re - yk.re) * 0.5);
        spec[row_a + k] = ar * ar + ai * ai;
        spec[row_b + k] = br * br + bi * bi;
    }
}

/// Time one slot's worth of pairs through both scalars, on the same
/// audio, in the same binary — issue #349 step 2.
///
/// §36 (`docs/notes/FT4_BENCHMARK.md`) measured the **batch**
/// `compute_spectrogram` at 1.11-1.15x for `fixed-point` and said
/// plainly that it said nothing about `stage1_inc`, whose whole
/// design is an incremental `u16` spectrogram fed during capture and
/// where the bandwidth argument might still hold. This measures that.
///
/// Returns `(us_i16, us_f32, bytes_i16, bytes_f32)` over `pairs`
/// pairs — the spec bytes are for a whole slot either way, since that
/// is the buffer a receiver has to hold, not just the part a bench
/// filled.
pub fn scalar_ab(audio: &[i16], pairs: usize) -> (i64, i64, usize, usize) {
    let n_freq = ((3_000.0 / (SAMPLE_RATE_HZ / NFFT_SPEC as f32)) as usize + 1).min(NFFT_SPEC / 2);
    let pairs = pairs.min(N_PAIRS);

    // The shift the worker would have locked for this audio — see
    // `advance_pairs`. Timing does not depend on it (the packing loop
    // shifts either way), but the spectra the cross-check compares do:
    // at shift 0 the `>> FP_SPEC_SHIFT` throws away most of a quiet
    // row and its peak bin becomes noise.
    let peak_abs = audio
        .iter()
        .take(NMAX)
        .map(|&v| (v as i32).abs())
        .max()
        .unwrap_or(1)
        .max(1);
    let mut shift = 0u32;
    while peak_abs << shift < TARGET_PEAK && shift < 8 {
        shift += 1;
    }

    let mut planner16 = crate::esp_dsp_fft::EspDspPlanner16::default();
    let fft16 = planner16.plan_forward(NFFT_SPEC);
    let mut buf16: Vec<Complex<i16>> = vec![Complex::new(0i16, 0i16); NFFT_SPEC];
    let mut spec16: Vec<u16> = vec![0u16; n_freq * N_TIME];

    let mut planner: Box<dyn mfsk_core::engine::fft::FftPlanner> =
        Box::new(crate::esp_dsp_fft::EspDspPlanner::default());
    let fft32 = planner.plan_forward(NFFT_SPEC);
    let mut buf32: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); NFFT_SPEC];
    let mut spec32: Vec<f32> = vec![0f32; n_freq * N_TIME];

    assert_eq!(fft16.len(), NFFT_SPEC, "sc16 planner returned another length");
    assert_eq!(fft32.len(), NFFT_SPEC, "f32 planner returned another length");

    // Three passes each, best kept. §32.1 of `docs/notes/FT4_BENCHMARK.md`
    // is this repo's own example of a device number that was really a
    // heap-alignment accident, and the f32 arm here moved 700 -> 853 ms
    // between two builds that did not touch it.
    let mut us16 = i64::MAX;
    for _ in 0..3 {
        let t0 = unsafe { esp_timer_get_time() };
        for k in 0..pairs {
            let (j_a, j_b) = (2 * k, 2 * k + 1);
            pair_kernel_i16(
                &*fft16,
                &mut buf16,
                audio,
                j_a * NSTEP,
                j_b * NSTEP,
                shift,
                n_freq,
                &mut spec16,
                j_a * n_freq,
                j_b * n_freq,
            );
        }
        us16 = us16.min(unsafe { esp_timer_get_time() } - t0);
    }

    let mut us32 = i64::MAX;
    for _ in 0..3 {
        let t1 = unsafe { esp_timer_get_time() };
        for k in 0..pairs {
            let (j_a, j_b) = (2 * k, 2 * k + 1);
            pair_kernel_f32(
                &*fft32,
                &mut buf32,
                audio,
                j_a * NSTEP,
                j_b * NSTEP,
                n_freq,
                &mut spec32,
                j_a * n_freq,
                j_b * n_freq,
            );
        }
        us32 = us32.min(unsafe { esp_timer_get_time() } - t1);
    }

    // A timing A/B is worthless if one arm is not computing the same
    // thing — a planner that handed back a different transform, or a
    // stub, would look like a win. So check the two spectra agree on
    // where the energy is, row by row.
    //
    // In the signal band, not over the whole row: bin 0 is DC and a
    // check that lands there says nothing. FT8 starts at 200 Hz, which
    // at 12 000 / 3 840 = 3.125 Hz per bin is bin 64.
    let lo = (200.0 / (SAMPLE_RATE_HZ / NFFT_SPEC as f32)) as usize;
    // Correlation, not argmax. The first version of this check
    // compared each row's strongest bin and read 171/184, then 144/184
    // purely from giving the i16 arm the shift the worker really locks
    // — on a band with twenty signals the argmax swaps between two
    // near-equal peaks, and on a noise row it is a coin flip. That is
    // a property of the metric. Pearson r over the whole band answers
    // the question actually being asked: are these two the same
    // spectrum, up to quantisation?
    let mut r_min = f32::MAX;
    let mut r_sum = 0.0f64;
    let mut rows = 0usize;
    let mut flat = 0usize;
    for j in 0..pairs * 2 {
        let row = j * n_freq;
        let a = &spec32[row..row + n_freq];
        let b = &spec16[row..row + n_freq];
        let (mut sa, mut sb) = (0.0f64, 0.0f64);
        for k in lo..n_freq {
            sa += a[k] as f64;
            sb += b[k] as f64;
        }
        let n = (n_freq - lo) as f64;
        let (ma, mb) = (sa / n, sb / n);
        let (mut caa, mut cbb, mut cab) = (0.0f64, 0.0f64, 0.0f64);
        for k in lo..n_freq {
            let (da, db) = (a[k] as f64 - ma, b[k] as f64 - mb);
            caa += da * da;
            cbb += db * db;
            cab += da * db;
        }
        // A row with no variance in either arm has no correlation to
        // report — that is the u16 arm quantising a silent row to all
        // zeros, not a disagreement. Counted, not scored.
        if !(caa > 0.0 && cbb > 0.0) {
            flat += 1;
            continue;
        }
        let r = (cab / (caa.sqrt() * cbb.sqrt())) as f32;
        r_min = r_min.min(r);
        r_sum += r as f64;
        rows += 1;
    }
    let r_mean = r_sum / rows.max(1) as f64;
    log::info!(
        "  stage1_inc A/B: row correlation u16 vs f32 — mean {:.4}, worst {:.4} \
         over {rows} rows ({flat} flat, band from bin {lo}){}",
        r_mean,
        r_min,
        if r_min > 0.9 { "" } else { "  <- DISAGREE" },
    );

    (
        us16,
        us32,
        spec16.len() * core::mem::size_of::<u16>(),
        spec32.len() * core::mem::size_of::<f32>(),
    )
}

fn compute_pair_into(ctx: &mut WorkerCtx, j_a: usize, j_b: usize) {
    // Defensive check (Gemini PR #123 round-10): the `wf_q.is_none()`
    // branch in `emit_spec_bundle` swaps empty Vecs into `ctx.cur`
    // because the `pair_limit` gate in `advance_pairs` is supposed
    // to prevent any further `compute_pair_into` calls for that
    // slot. If that invariant ever breaks (refactor lands a
    // `pair_limit = N_PAIRS` path that forgets to widen
    // emit_spec_bundle's tuple), the indexed writes below would
    // hit empty Vecs and panic anyway — surface it explicitly here.
    debug_assert!(
        !ctx.cur.spec.is_empty(),
        "compute_pair_into invoked after wf_q=None emit replaced ctx.cur.spec with empty Vec"
    );
    let shift = ctx.cur.shift;
    let ia_a = j_a * NSTEP;
    let ia_b = j_b * NSTEP;
    let n_freq = ctx.n_freq;
    let row_a = j_a * n_freq;
    let row_b = j_b * n_freq;
    {
        // SAFETY: `audio_ptr` is the fill-side buffer and this task is
        // its only writer; `AudioBuf` is NMAX samples long, which is
        // the bound every index below is already checked against.
        let audio: &[i16] = unsafe { core::slice::from_raw_parts(ctx.fill_ptr(), NMAX) };
        pair_kernel_i16(
            &*ctx.fft,
            &mut ctx.fft_buf,
            audio,
            ia_a,
            ia_b,
            shift,
            n_freq,
            &mut ctx.cur.spec,
            row_a,
            row_b,
        );
    }
    update_allsum_columns_for_m(ctx, j_a);
    update_allsum_columns_for_m(ctx, j_b);

    // Streaming WF tick — emit one row per pair if a queue is wired.
    // We average the two new spec rows (j_a, j_b) per freq bin then
    // decimate to the host's screen width via boxcar over the WF
    // band [200, 2700] Hz; the result is 0..15 palette indices the
    // UI redraw can blit verbatim.
    if let Some(wf_q) = ctx.wf_q {
        let row = decimate_pair_to_wf(j_a, j_b, ctx.n_freq, &ctx.cur.spec);
        let tick = alloc::boxed::Box::new(crate::pipeline::WfTick {
            pair_idx: j_b as u8,
            row,
        });
        // Drop on full queue — never block stage1.
        let _ = crate::pipeline::try_send_box(wf_q, tick);
    }
}

/// Decimate the two newly-filled spec rows into a single WF row for
/// the host's 135-px screen width over the FT8 audio band
/// (200..2700 Hz). Cost: ~2 × 800 freq bins read + 135-cell average
/// = ~3 µs at the WF cadence (12 ticks/s) — negligible vs FFT.
fn decimate_pair_to_wf(
    j_a: usize,
    j_b: usize,
    n_freq: usize,
    spec: &[u16],
) -> [u8; crate::pipeline::WF_ROW_LEN] {
    const WF_FREQ_LO_HZ: f32 = 200.0;
    const WF_FREQ_HI_HZ: f32 = 2_700.0;
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let row_a = j_a * n_freq;
    let row_b = j_b * n_freq;
    let mut out = [0u8; crate::pipeline::WF_ROW_LEN];
    for col in 0..crate::pipeline::WF_ROW_LEN {
        let f0 = WF_FREQ_LO_HZ
            + (col as f32) * (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ) / crate::pipeline::WF_ROW_LEN as f32;
        let f1 = WF_FREQ_LO_HZ
            + ((col + 1) as f32) * (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ)
                / crate::pipeline::WF_ROW_LEN as f32;
        let bin_lo = (f0 / df).floor() as usize;
        let bin_hi = ((f1 / df).ceil() as usize).min(n_freq);
        if bin_hi <= bin_lo {
            continue;
        }
        let mut sum: u32 = 0;
        for f in bin_lo..bin_hi {
            sum += spec[row_a + f] as u32;
            sum += spec[row_b + f] as u32;
        }
        let cells = (2 * (bin_hi - bin_lo)) as u32;
        let avg = (sum / cells.max(1)).max(1);
        // log2 approx: bit-position of MSB → 0..15. u16 max → 16.
        let log2 = (32u32 - avg.leading_zeros()).min(15) as u8;
        out[col] = log2;
    }
    out
}

fn update_allsum_columns_for_m(ctx: &mut WorkerCtx, m: usize) {
    if m >= N_TIME {
        return;
    }
    update_one_half(
        m,
        ctx.head_ia,
        ctx.head_n_freq,
        ctx.n_freq,
        &ctx.cur.spec,
        &mut ctx.cur.allsum_head,
    );
    update_one_half(
        m,
        ctx.tail_ia,
        ctx.tail_n_freq,
        ctx.n_freq,
        &ctx.cur.spec,
        &mut ctx.cur.allsum_tail,
    );
}

fn update_one_half(
    m: usize,
    ia: usize,
    n_freq: usize,
    spec_n_freq: usize,
    spec: &[u16],
    dst: &mut [f32],
) {
    // 7-tone gather at 2-bin step — matches WSJT-X `sync8.f90:66`
    // (k=0..6; tone 7 is data-only, never a Costas position).
    // NFFT=3840 → tone_step_bins=2.0 exactly → single-bin gather.
    //
    // Sliding window: carriers at fi and fi+2 share 6 of 7 bins.
    // allsum[fi] = allsum[fi-2] + row[ia+fi+12] - row[ia+fi-2].
    // Reduces inner work from 7 adds to 1 add + 1 sub per carrier
    // after the two seed values. Sequential row reads stay in cache.
    if n_freq == 0 {
        return;
    }
    let row_base = m * spec_n_freq;
    let upper = spec_n_freq - 1;
    let row = &spec[row_base..row_base + spec_n_freq];
    // Seed prev[0] (even fi=0) and prev[1] (odd fi=1) with full sums.
    let mut prev = [0.0f32; 2];
    for parity in 0..2usize {
        if parity >= n_freq {
            break;
        }
        let i_carrier = ia + parity;
        let mut s = 0.0f32;
        for k in 0..(NTONES - 1) {
            let bin = (i_carrier + 2 * k).min(upper);
            s += row[bin] as f32;
        }
        dst[parity * N_TIME + m] = s;
        prev[parity] = s;
    }
    for fi in 2..n_freq {
        let i_carrier = ia + fi;
        let drop = (i_carrier - 2).min(upper);
        let add = (i_carrier + 2 * (NTONES - 2)).min(upper);
        let p = fi & 1;
        let s = prev[p] + row[add] as f32 - row[drop] as f32;
        dst[fi * N_TIME + m] = s;
        prev[p] = s;
    }
}

fn finalize_slot(ctx: &mut WorkerCtx, wav_idx: usize, total_samples: usize) {
    let slotend_us = unsafe { esp_timer_get_time() };
    // Drain any remaining pairs that the audio supports.
    advance_pairs(ctx);
    if !ctx.cur.spec_sent {
        // Pair 92 didn't complete during capture (under-fed slot).
        // Send what we have so main doesn't deadlock waiting for spec.
        log::warn!(
            "stage1_inc: slot {wav_idx} pair_done={}/{N_PAIRS}, sending partial SpecBundle",
            ctx.cur.next_pair
        );
        emit_spec_bundle(ctx);
    }
    if ctx.cur.audio_fill != total_samples {
        log::warn!(
            "stage1_inc: slot {wav_idx} audio_fill={} != reported total {total_samples}",
            ctx.cur.audio_fill
        );
    }

    // Snapshot the just-completed buffer's pointer and fill count
    // BEFORE we swap `fill_idx`. main reads from this pointer
    // (= same buffer that was emitted via SpecBundle.audio_ptr).
    let done_audio_ptr: *const i16 = ctx.fill_ptr();
    let done_audio_fill = ctx.cur.audio_fill;

    // Swap fill_idx so stage1_inc's NEXT slot fills the *other*
    // buffer. Main keeps reading the just-snapshotted pointer until
    // it drops the Slot — by which time stage1_inc is on slot K+1,
    // so the only way these conflict is if main blocks for more
    // than 15 s after SlotEnd K (post_slotend ≪ 15 s; debug_assert
    // guards against drift). Bump the buffer's generation tag for
    // future-introspection.
    ctx.fill_idx ^= 1;
    ctx.audio_bufs[ctx.fill_idx].gen = ctx.audio_bufs[ctx.fill_idx].gen.wrapping_add(1);

    let fresh = SlotInProgress::new(ctx.n_freq, ctx.head_n_freq, ctx.tail_n_freq);
    let done = core::mem::replace(&mut ctx.cur, fresh);

    let slot = Box::new(Slot::new(
        done_audio_ptr,
        done_audio_fill,
        wav_idx,
        done.inc_total_us,
        slotend_us,
    ));
    send_box(ctx.slot_q, slot);
}
