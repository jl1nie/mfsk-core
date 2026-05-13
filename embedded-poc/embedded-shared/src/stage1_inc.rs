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

    let r = unsafe {
        xTaskCreatePinnedToCore(
            Some(worker_main),
            c"stage1_inc".as_ptr(),
            16384,
            ctx_ptr,
            3, // below dual_core (5) and wav_sim (4)
            ptr::null_mut(),
            1, // APP_CPU
        )
    };
    assert_eq!(r, PD_PASS, "xTaskCreatePinnedToCore(stage1_inc) failed: {r}");
    log::info!(
        "stage1_inc: spawned (APP_CPU prio 3); n_time={} n_pairs={} n_freq={}",
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
        }
    }
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

    loop {
        let j = ctx.cur.next_pair;
        if j >= N_PAIRS {
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

    // If the last pair just landed and we haven't sent SpecBundle yet,
    // fire it now — main can run stage 2 in parallel with the tail of
    // capture (audio chunks 148–150 still arriving).
    if ctx.cur.next_pair == N_PAIRS && !ctx.cur.spec_sent {
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
    let bundle = Box::new(SpecBundle {
        spec: mfsk_core::ft8::decode_block::Spectrogram::from_parts(n_freq, N_TIME, spec),
        allsum_head: head,
        allsum_tail: tail,
        // wav_idx is only known at SlotEnd; main matches SpecBundle to
        // Slot by FIFO order of receipt, so this is informational only.
        wav_idx: usize::MAX,
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
            let mag2_a = ((a_re * a_re + a_im * a_im) as u32) >> FP_SPEC_SHIFT;
            let mag2_b = ((b_re * b_re + b_im * b_im) as u32) >> FP_SPEC_SHIFT;
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
    });
    send_box(ctx.slot_q, slot);
}
