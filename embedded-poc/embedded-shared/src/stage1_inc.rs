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

use esp_idf_svc::sys::{
    esp_timer_get_time, xTaskCreatePinnedToCore, QueueHandle_t,
};

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
const _: () = assert!(NFFT_SPEC == 3_840, "stage1_inc NFFT_SPEC must match mfsk_core (3840)");
const FP_SPEC_SHIFT: u32 = 12;
const TONE_SPACING_HZ: f32 = 6.25;
const SAMPLE_RATE_HZ: f32 = 12_000.0;
const N_TIME: usize = NMAX / NSTEP - 3; // 184
const N_PAIRS: usize = N_TIME / 2; // 92
const TARGET_PEAK: i32 = (NFFT_SPEC * 2) as i32;

// Phase C+ (2026-05-21): emit SpecBundle *before* the last few pairs
// finish, so coarse_sync gets more headroom before SlotEnd. coarse_sync
// reads spec at m ∈ {Costas-positions + lag} only; with `nstep-half`
// (NSSY=2), jstrt=6, and SYNC_LAG_S=1.0 (jz≈12-13), the maximum m
// touched is block-2 last Costas + jz = 162 + 13 = 175. Pair 87 fills
// up to m=175 (j_a=174, j_b=175), so emitting at `next_pair == 88`
// gives coarse_sync everything it needs while leaving ~720 ms more
// of audio-tail wallclock for the speculative pass-2 + stage-3 path.
//
// Pairs 88..91 (m=176..183) still get computed after emit, but
// `emit_spec_bundle` swaps in a fresh zero buffer so those writes
// land in a discarded allocation. `xsnr2_db_simple` does sample
// spec at strided m values across the full range; the trailing 8
// zero rows out of 184 (≈ 4.3%) bias the median noise-floor estimate
// slightly but stay well inside the per-station SNR jitter we
// already see across slots. Keep all 92 pairs computed (we discard
// rather than skip) so any future downstream consumer that needs
// the full spec can opt in by hooking a second emit at pair 91.
const SPEC_EMIT_PAIR: usize = 88; // emit after pair 87 done (m=0..175 valid)
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
    /// Accumulating slot — fresh-allocated at start of each slot.
    cur: SlotInProgress,
}

/// State of the slot currently being assembled.
struct SlotInProgress {
    audio: Vec<i16>,
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
            audio: vec![0i16; NMAX],
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
    assert_eq!(r, PD_PASS, "xTaskCreatePinnedToCore(stage1_inc) failed: {r}");
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
            ChunkMsg::SlotEnd { wav_idx, total_samples } => {
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
    let cur = &mut ctx.cur;
    let off = cur.audio_fill;
    if off + samples.len() > NMAX {
        // More than one slot worth of audio without a SlotEnd — should
        // not happen in normal operation. Drop excess.
        log::warn!(
            "stage1_inc: samples overflow (off={off}, n={}); dropping",
            samples.len()
        );
        return;
    }
    cur.audio[off..off + samples.len()].copy_from_slice(samples);
    for &s in samples {
        let a = (s as i32).unsigned_abs() as i32;
        if a > cur.peak_abs {
            cur.peak_abs = a;
        }
    }
    cur.audio_fill = off + samples.len();
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
    // covered (pair `SPEC_EMIT_PAIR - 1`). Pairs 88..91 (m=176..183)
    // contribute nothing to coarse_sync and would otherwise compute
    // 4 × `compute_pair_into` calls into a buffer that
    // `emit_spec_bundle`'s `mem::replace` immediately discards
    // (Gemini PR #123 review). Production apps with `wf_q = Some`
    // still need pairs 88..91 to keep the waterfall flowing through
    // the slot tail, so they keep the original N_PAIRS limit.
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
    let spec = core::mem::replace(&mut ctx.cur.spec, vec![0u16; n_freq * N_TIME]);
    let head = core::mem::replace(&mut ctx.cur.allsum_head, vec![0f32; head_n * N_TIME]);
    let tail = core::mem::replace(&mut ctx.cur.allsum_tail, vec![0f32; tail_n * N_TIME]);
    // Phase C: hand main an owned snapshot of the audio captured
    // so far. Copying (~180 KB PSRAM→PSRAM, ~2 ms) avoids the
    // aliasing UB the prior raw-pointer scheme had under stacked
    // borrows — main's `&[i16]` is now backed by its own
    // allocation, independent of stage1_inc's mutable access to
    // `ctx.cur.audio` for subsequent chunks.
    let audio_prefix = ctx.cur.audio[..ctx.cur.audio_fill].to_vec();
    let bundle = Box::new(SpecBundle {
        spec: mfsk_core::ft8::decode_block::Spectrogram::from_parts(n_freq, N_TIME, spec),
        allsum_head: head,
        allsum_tail: tail,
        // wav_idx is only known at SlotEnd; main matches SpecBundle to
        // Slot by FIFO order of receipt, so this is informational only.
        wav_idx: usize::MAX,
        audio_prefix,
    });
    send_box(ctx.spec_q, bundle);
    ctx.cur.spec_sent = true;
}

fn compute_pair_into(ctx: &mut WorkerCtx, j_a: usize, j_b: usize) {
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
        let cur = &ctx.cur;
        let buf = &mut ctx.fft_buf;
        for k in 0..NFFT_SPEC {
            let re = if k < NSPS && ia_a + k < cur.audio.len() {
                let raw = cur.audio[ia_a + k] as i32;
                (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            } else {
                0
            };
            let im = if k < NSPS && ia_b + k < cur.audio.len() {
                let raw = cur.audio[ia_b + k] as i32;
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
            let mag2_a = (((a_re * a_re + a_im * a_im) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32);
            let mag2_b = (((b_re * b_re + b_im * b_im) as u32) >> FP_SPEC_SHIFT)
                .min(u16::MAX as u32);
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
        let f0 = WF_FREQ_LO_HZ + (col as f32) * (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ)
            / crate::pipeline::WF_ROW_LEN as f32;
        let f1 = WF_FREQ_LO_HZ + ((col + 1) as f32) * (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ)
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
    // (k=0..6; tone 7 is data-only, never a Costas position) and
    // `coarse_sync_inner`'s score-formula divisor `(NTONES - 2) = 6`.
    //
    // NFFT=3840 → tone_step_bins = 2.0 exactly. The earlier 16-bin
    // contiguous sliding window matched the NFFT=4096 era when
    // tone_step ≈ 2.13 + Hann mainlobe leakage made the in-between
    // bins informative; at NFFT=3840 those bins carry pure noise and
    // a 16-contig sum nearly doubles `t0_ref`, halving sync ratio
    // and dropping weak qso3 signals (mid/high band → 0 hits).
    const TONE_STEP_BINS: usize = 2;
    let row_base = m * spec_n_freq;
    let upper = spec_n_freq - 1;
    for fi in 0..n_freq {
        let i_carrier = ia + fi;
        let mut s: f32 = 0.0;
        for k in 0..(NTONES - 1) {
            let bin = (i_carrier + TONE_STEP_BINS * k).min(upper);
            s += spec[row_base + bin] as f32;
        }
        dst[fi * N_TIME + m] = s;
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

    let fresh = SlotInProgress::new(ctx.n_freq, ctx.head_n_freq, ctx.tail_n_freq);
    let done = core::mem::replace(&mut ctx.cur, fresh);

    let slot = Box::new(Slot {
        audio: done.audio,
        wav_idx,
        inc_total_us: done.inc_total_us,
        slotend_us,
    });
    send_box(ctx.slot_q, slot);
}
