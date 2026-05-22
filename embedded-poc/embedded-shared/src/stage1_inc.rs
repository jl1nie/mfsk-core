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

use mfsk_core::core::fft::{Fft16, FftPlanner16};
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

    let mut fft_planner = mfsk_core::core::fft::default_planner_16();
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
        // SAFETY: `audio_ptr` is the fill-side buffer (this is the
        // only writer in this task); reading from indices < NMAX is
        // bounds-safe (`AudioBuf` has fixed NMAX size). Each index
        // is checked against NMAX (= NMAX here is the audio buffer
        // length, matched to `cur.audio_fill`'s bound check).
        let audio_ptr: *const i16 = ctx.fill_ptr();
        let buf = &mut ctx.fft_buf;
        for k in 0..NFFT_SPEC {
            let re = if k < NSPS && ia_a + k < NMAX {
                let raw = unsafe { *audio_ptr.add(ia_a + k) } as i32;
                (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            } else {
                0
            };
            let im = if k < NSPS && ia_b + k < NMAX {
                let raw = unsafe { *audio_ptr.add(ia_b + k) } as i32;
                (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            } else {
                0
            };
            buf[k] = Complex::new(re, im);
        }
    }
    ctx.fft.process(&mut ctx.fft_buf);

    // Demux pair → spec rows.
    let row_a = j_a * n_freq;
    let row_b = j_b * n_freq;
    {
        let buf = &ctx.fft_buf;
        let spec = &mut ctx.cur.spec;
        for k in 0..n_freq {
            let kn = if k == 0 { 0 } else { NFFT_SPEC - k };
            let yk_re = buf[k].re as i32;
            let yk_im = buf[k].im as i32;
            let yn_re = buf[kn].re as i32;
            let yn_im = buf[kn].im as i32;
            let a_re = (yk_re + yn_re) >> 1;
            let a_im = (yk_im - yn_im) >> 1;
            let b_re = (yk_im + yn_im) >> 1;
            let b_im = (yn_re - yk_re) >> 1;
            // mag² saturate at u16::MAX. Wrapping `as u16` (the
            // pre-Phase 1.7.7b behaviour) was a real bug: very strong
            // co-channel bins overflowed u16 after `>> FP_SPEC_SHIFT`
            // and aliased to a tiny residue, making strong stations
            // *invisible* to `coarse_sync`. Caught while tracing the
            // host vs embedded NSTEP divergence — the host fixed-point
            // path already saturates (`spectrogram.rs::compute_spectrogram`
            // mag2 sites). Matching it here keeps the two paths
            // algorithmically identical.
            let mag2_a =
                (((a_re * a_re + a_im * a_im) as u32) >> FP_SPEC_SHIFT).min(u16::MAX as u32);
            let mag2_b =
                (((b_re * b_re + b_im * b_im) as u32) >> FP_SPEC_SHIFT).min(u16::MAX as u32);
            spec[row_a + k] = mag2_a as u16;
            spec[row_b + k] = mag2_b as u16;
        }
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
