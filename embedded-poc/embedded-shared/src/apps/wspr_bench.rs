//! WSPR candidate-loop bench for ESP32-S3 (LX7).
//!
//! Implements `docs/notes/WSPR_EMBEDDED_MEASUREMENT_PLAN.md` Phase 1
//! and Phase 2. It answers the one question three rounds of desk
//! analysis on issue #260 could not: what the WSPR candidate loop
//! actually costs on the device, and whether it is bandwidth-bound.
//!
//! ## Why there is no DDC here
//!
//! The bench is fed a **baked 375 Hz baseband** produced on the host by
//! `wspr_bake_golden_baseband` (`mfsk-core/tests/wspr_wsjtx_samples.rs`)
//! from the WSJT-X golden `150426_0918.wav`. `decimate_to_baseband` is
//! the one stage the ESP-DSP FFT backend cannot serve — `NFFT1 =
//! 1_474_560` is far past any plan it has — and it is also the stage
//! #260 proposes replacing with a streaming DDC. Skipping it is what
//! makes the measurement possible today, with no new API and no
//! refactor: every stage below it (`coarse_baseband`,
//! `decode_at_baseband*`, `subtract_signal_baseband`) is already `pub`.
//!
//! ## What it runs
//!
//! `decode_scan_inner`'s full sequence, not the two-pass abbreviation
//! `wspr_diag_candidate_cost_split` uses. Reading the source to write
//! this bench turned up two places where the plan (and the host
//! diagnostic it was drawn from) understate production cost:
//!
//! - `decode_scan` runs **three** inner passes, not two — early passes
//!   0 and 1 (`nblocks = [1]`, drift refine on, `maxdrift = 4`) and
//!   then pass 2. Pass 1 is skipped when pass 0 decoded nothing,
//!   exactly as `wsprd.c:999` does.
//! - Pass 2's ladder is `[1, 2, 3, 0]` — four rungs, not three. The
//!   fourth is wsprd's `ib == 4` bit-by-bit rung.
//!
//! Per-pass totals are logged separately so the plan's
//! "pass 1 + pass 2" subtotal stays derivable from the same run.
//!
//! ## Arms
//!
//! - **A** — `idat`/`qdat` in PSRAM.
//! - **B** — `idat`/`qdat` in internal SRAM. 360 KiB against the S3's
//!   512 KB is tight; if only one buffer fits, the bench runs a split
//!   arm (`idat` internal, `qdat` PSRAM) and says so, which still
//!   brackets the answer.
//! - **C** — oscillator-table cost. `tone_amplitudes` rebuilds eight
//!   `[f32; 257]` tables per symbol — 1.33 MB written per evaluation,
//!   ~4× the 324 KiB of baseband the same evaluation reads, and
//!   invisible to any PSRAM-traffic instrumentation. Arm C times a
//!   table-free copy of the same routine that carries the recurrence
//!   inline, and asserts the result is **bit-identical** (it is the
//!   same recurrence in the same order, merely not stored).
//!
//! Single-core and sequential throughout: the honest per-core number
//! comes before any reasoning about splitting it.

extern crate alloc;

use alloc::vec::Vec;

use mfsk_core::wspr::coarse_baseband::{BasebandCandidate, coarse_baseband};
use mfsk_core::wspr::decode::{
    WsprCallsignTable, WsprResult, decode_at_baseband, decode_at_baseband_nblocks_gated_drift,
};
use mfsk_core::wspr::demod::{IsQs, N_SYMBOLS, NSPS_BASEBAND, TONE_SPACING_HZ, tone_amplitudes};
use mfsk_core::wspr::instrument;
use mfsk_core::wspr::subtract::subtract_signal_baseband;

use crate::esp_dsp_fft;

/// 46080 — `wspr::baseband::NFFT2`, the fixed baseband length.
const NBB: usize = 46_080;
/// `NEGATIVE_DT_PAD_SEC × 12 000`, in audio samples. `coarse_baseband`
/// takes the pad in the audio time base, not the baseband one.
const PAD_AUDIO: usize = 36_000;
const SAMPLE_RATE: u32 = 12_000;
/// Matches `wspr_wsjtx_sample_recall_vs_golden` and
/// `WSPR_BENCHMARK.md`, so the device number is comparable to the
/// published host one. (`SearchParams::default()` is 200.)
const MAX_CANDIDATES: usize = 100;
/// `wsprd.c:1002` — passes 0 and 1 search ±4 Hz of drift.
const EARLY_MAX_DRIFT: i32 = 4;
/// `wsprd.c:1005-1009` — the final pass does not.
const PASS2_MAX_DRIFT: i32 = 0;

const MALLOC_CAP_8BIT: u32 = 1 << 2;
const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Bytes of `BENCH_STACK` never touched so far. FreeRTOS task stacks
/// come off the heap, so an overflow does not fault — it silently
/// scribbles on a neighbouring block's TLSF header and the program
/// dies later, inside an unrelated `free()`. That is exactly how the
/// first run of this bench failed (`StoreProhibited` in `tlsf_free`,
/// called from `coarse_baseband`'s `Vec` drop), so the headroom is
/// logged rather than assumed.
fn stack_headroom() -> u32 {
    // `uxTaskGetStackHighWaterMark` returns words on some ports and
    // bytes on ESP-IDF's; IDF documents it as bytes.
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

/// Abort loudly if the heap is already damaged, rather than at the
/// next unrelated `free()`.
fn assert_heap_ok(where_: &str) {
    let ok = unsafe { esp_idf_svc::sys::heap_caps_check_integrity_all(true) };
    if !ok {
        log::error!("HEAP CORRUPT after {where_}");
    }
}

/// `heap_caps_malloc` an `f32` buffer, or `None` if that capability
/// has no block big enough. Deliberately not a `Vec`: the global
/// allocator picks the memory for us, and picking it is the whole
/// experiment.
fn alloc_caps(n: usize, caps: u32) -> Option<*mut f32> {
    let p = unsafe { esp_idf_svc::sys::heap_caps_malloc(n * 4, caps) } as *mut f32;
    if p.is_null() { None } else { Some(p) }
}

fn free_caps(p: *mut f32) {
    unsafe { esp_idf_svc::sys::heap_caps_free(p as *mut core::ffi::c_void) }
}

/// Decode the baked asset (`idat[46080]` then `qdat[46080]`, f32 LE)
/// into caller-placed buffers.
///
/// Byte-wise rather than a transmute: `include_bytes!` gives a
/// `&[u8]` with 1-byte alignment, and Xtensa faults on an unaligned
/// f32 load. The cost is one pass over 360 KiB, outside every timed
/// region.
fn load_baseband(bin: &[u8], idat: &mut [f32], qdat: &mut [f32]) {
    assert_eq!(bin.len(), NBB * 2 * 4, "baked baseband has the wrong size");
    for (i, dst) in idat.iter_mut().enumerate() {
        let b = &bin[i * 4..i * 4 + 4];
        *dst = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
    }
    let base = NBB * 4;
    for (i, dst) in qdat.iter_mut().enumerate() {
        let b = &bin[base + i * 4..base + i * 4 + 4];
        *dst = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
    }
}

/// Per-pass timing + counter deltas.
struct PassStats {
    label: &'static str,
    coarse_us: i64,
    decode_us: i64,
    subtract_us: i64,
    n_cand: usize,
    n_decoded: usize,
    counts: instrument::Counts,
}

impl PassStats {
    fn total_us(&self) -> i64 {
        self.coarse_us + self.decode_us + self.subtract_us
    }

    fn log(&self) {
        let per_cand = if self.n_cand > 0 {
            self.decode_us / self.n_cand as i64
        } else {
            0
        };
        let c = &self.counts;
        log::info!(
            "  {:<7} coarse {:>7} ms | decode {:>7} ms ({} cand, {:>6} us/cand, {} decoded) \
             | subtract {:>6} ms",
            self.label,
            self.coarse_us / 1000,
            self.decode_us / 1000,
            self.n_cand,
            per_cand,
            self.n_decoded,
            self.subtract_us / 1000,
        );
        let minsync_pct = if c.candidates > 0 {
            c.minsync1_pass * 100 / c.candidates
        } else {
            0
        };
        log::info!(
            "          tone_amplitudes {:>6} ({:>4} /cand, {} MiB read) | minsync1 {}/{} = {}% \
             | fano {}/{} | osd {}/{}",
            c.tone_amplitudes,
            if c.candidates > 0 {
                c.tone_amplitudes / c.candidates
            } else {
                0
            },
            c.baseband_bytes_read() / (1024 * 1024),
            c.minsync1_pass,
            c.candidates,
            minsync_pct,
            c.fano_ok,
            c.fano_attempts,
            c.osd_ok,
            c.osd_attempts,
        );
    }
}

/// One full `decode_scan`-equivalent over a caller-placed baseband.
///
/// Mutates `idat`/`qdat` in place (the subtraction steps do), so the
/// caller reloads the baked asset before each arm — otherwise arm B
/// would be running against arm A's residual and the A/B delta would
/// be measuring different work, not different memory.
#[inline(never)]
fn run_scan(idat: &mut [f32], qdat: &mut [f32]) -> (Vec<PassStats>, Vec<WsprResult>) {
    let mut stats = Vec::new();
    let mut seen: Vec<WsprResult> = Vec::new();
    let mut found: Vec<(WsprResult, usize)> = Vec::new();

    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192;

    // Passes 0 and 1 — `decode_scan_inner`'s `early_pass` loop.
    for early_pass in 0..2 {
        if early_pass == 1 && found.is_empty() {
            log::info!("  pass 1 skipped (pass 0 decoded nothing) — wsprd.c:999");
            break;
        }
        let c0 = instrument::snapshot();

        let t = now_us();
        let mut cands: Vec<BasebandCandidate> = coarse_baseband(
            idat,
            qdat,
            PAD_AUDIO,
            MAX_CANDIDATES,
            EARLY_MAX_DRIFT,
        );
        cands.truncate(MAX_CANDIDATES);
        let coarse_us = now_us() - t;
        log::info!(
            "    [coarse {} done: {} cand, stack headroom {} B]",
            early_pass,
            cands.len(),
            stack_headroom(),
        );
        assert_heap_ok("coarse_baseband");

        let t = now_us();
        let mut raw: Vec<(WsprResult, usize)> = Vec::new();
        for c in &cands {
            if let Some(mut d) = decode_at_baseband(
                idat,
                qdat,
                SAMPLE_RATE,
                c.start_sample,
                c.freq_hz,
                c.drift_hz,
            ) {
                let start_refined = d.start_sample;
                d.dt_sec =
                    (start_refined as i64 - PAD_AUDIO as i64) as f32 / SAMPLE_RATE as f32 - 1.0;
                d.start_sample = start_refined.saturating_sub(PAD_AUDIO);
                d.snr_db = c.snr_db;
                raw.push((d, start_refined));
            }
        }
        let decode_us = now_us() - t;
        log::info!("    [decode {early_pass} done: stack headroom {} B]", stack_headroom());
        assert_heap_ok("pass-1 decode loop");

        let mut this_pass: Vec<(WsprResult, usize)> = Vec::new();
        for (d, start_refined) in raw {
            let dup = seen.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
            if !dup {
                this_pass.push((d.clone(), start_refined));
                seen.push(d);
            }
        }

        let t = now_us();
        for (d, start_refined) in &this_pass {
            let symbols = mfsk_core::wspr::encode_channel_symbols(&d.info_bits);
            let f0_audio = d.freq_hz + 1.5 * TONE_SPACING_HZ;
            let shift_baseband = (*start_refined as i32) / 32;
            subtract_signal_baseband(
                idat,
                qdat,
                f0_audio,
                shift_baseband,
                d.drift_hz,
                &symbols,
            );
        }
        let subtract_us = now_us() - t;

        let n_decoded = this_pass.len();
        found.extend(this_pass);
        stats.push(PassStats {
            label: if early_pass == 0 { "pass 0" } else { "pass 1" },
            coarse_us,
            decode_us,
            subtract_us,
            n_cand: cands.len(),
            n_decoded,
            counts: instrument::snapshot().since(c0),
        });
    }

    // Pass 2 — the final pass, with the callsign table the early
    // passes' Fano decodes populated. It runs unconditionally.
    let mut confirmed = WsprCallsignTable::new();
    for (d, _) in &found {
        confirmed.record(&d.message);
    }
    let c0 = instrument::snapshot();

    let t = now_us();
    let cands2: Vec<BasebandCandidate> =
        coarse_baseband(idat, qdat, PAD_AUDIO, MAX_CANDIDATES, PASS2_MAX_DRIFT);
    let coarse_us = now_us() - t;

    let t = now_us();
    let mut raw2: Vec<WsprResult> = Vec::new();
    for c in &cands2 {
        if let Some(mut d) = decode_at_baseband_nblocks_gated_drift(
            idat,
            qdat,
            SAMPLE_RATE,
            c.start_sample,
            c.freq_hz,
            c.drift_hz,
            &[1, 2, 3, 0],
            Some(&confirmed),
            false,
        ) {
            let start_refined = d.start_sample;
            d.dt_sec = (start_refined as i64 - PAD_AUDIO as i64) as f32 / SAMPLE_RATE as f32 - 1.0;
            d.start_sample = start_refined.saturating_sub(PAD_AUDIO);
            d.snr_db = c.snr_db;
            raw2.push(d);
        }
    }
    let decode_us = now_us() - t;
    log::info!("    [pass 2 decode done: stack headroom {} B]", stack_headroom());
    assert_heap_ok("pass-2 decode loop");

    let mut n_new = 0;
    for d in raw2 {
        let dup = seen.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= TIME_DEDUP_SAMPLES
        });
        if !dup {
            seen.push(d);
            n_new += 1;
        }
    }

    stats.push(PassStats {
        label: "pass 2",
        coarse_us,
        decode_us,
        subtract_us: 0,
        n_cand: cands2.len(),
        n_decoded: n_new,
        counts: instrument::snapshot().since(c0),
    });

    (stats, seen)
}

/// Run one arm end-to-end and log it.
#[inline(never)]
fn arm(name: &str, bin: &[u8], idat: &mut [f32], qdat: &mut [f32]) -> i64 {
    log::info!("--- arm {name} ---");
    load_baseband(bin, idat, qdat);
    instrument::reset();

    let t0 = now_us();
    let (stats, results) = run_scan(idat, qdat);
    let total = now_us() - t0;

    for s in &stats {
        s.log();
    }
    // The plan's decision table is stated per `decode_scan`; the host
    // cost split it was drawn from covers only pass 0 + pass 2, so log
    // both or the two numbers cannot be compared.
    let two_pass: i64 = stats
        .iter()
        .filter(|s| s.label != "pass 1")
        .map(PassStats::total_us)
        .sum();
    log::info!(
        "  arm {name} TOTAL = {} ms  ({} decodes)  [pass0+pass2 subtotal = {} ms]",
        total / 1000,
        results.len(),
        two_pass / 1000,
    );
    for r in &results {
        log::info!(
            "    {} | {:.1} Hz | dt {:.2} s | {} dB",
            r.message,
            r.freq_hz,
            r.dt_sec,
            r.snr_db as i32,
        );
    }
    total
}

/// Table-free `tone_amplitudes`.
///
/// Bit-identical to `mfsk_core::wspr::demod::tone_amplitudes`: the
/// stored `c0[j]`/`s0[j]` are produced by exactly the recurrence the
/// mixing loop then consumes in index order, so carrying them as
/// running scalars performs the same operations on the same values.
/// What changes is 8 × 257 f32 of stores and reloads per symbol —
/// 1.33 MB written per evaluation — collapsing to 8 live f32.
fn tone_amplitudes_inline(
    idat: &[f32],
    qdat: &[f32],
    f0_baseband_hz: f32,
    lag: i32,
    drift_hz: f32,
) -> IsQs {
    use core::f32::consts::PI;
    const BASEBAND_RATE: f32 = 375.0;

    let np = idat.len() as i32;
    let dt = 1.0 / BASEBAND_RATE;
    let df = TONE_SPACING_HZ;
    let twopidt = 2.0 * PI * dt;
    let df15 = df * 1.5;
    let df05 = df * 0.5;

    let mut isqs = IsQs {
        is: [[0.0f32; N_SYMBOLS]; 4],
        qs: [[0.0f32; N_SYMBOLS]; 4],
        cf: [[0.0f32; N_SYMBOLS]; 4],
        sf: [[0.0f32; N_SYMBOLS]; 4],
    };

    for i in 0..N_SYMBOLS {
        let fp = f0_baseband_hz + (drift_hz / 2.0) * ((i as f32 - 81.0) / 81.0);
        let dphi = [
            twopidt * (fp - df15),
            twopidt * (fp - df05),
            twopidt * (fp + df05),
            twopidt * (fp + df15),
        ];
        let mut cd = [0.0f32; 4];
        let mut sd = [0.0f32; 4];
        for t in 0..4 {
            cd[t] = dphi[t].cos();
            sd[t] = dphi[t].sin();
        }

        let mut c = [1.0f32; 4];
        let mut s = [0.0f32; 4];
        let mut iacc = [0.0f32; 4];
        let mut qacc = [0.0f32; 4];

        for j in 0..NSPS_BASEBAND {
            let k = lag + (i as i32) * (NSPS_BASEBAND as i32) + (j as i32);
            if k > 0 && k < np {
                let id = idat[k as usize];
                let qd = qdat[k as usize];
                for t in 0..4 {
                    iacc[t] += id * c[t] + qd * s[t];
                    qacc[t] += -id * s[t] + qd * c[t];
                }
            }
            for t in 0..4 {
                let cn = c[t] * cd[t] - s[t] * sd[t];
                let sn = c[t] * sd[t] + s[t] * cd[t];
                c[t] = cn;
                s[t] = sn;
            }
        }

        for t in 0..4 {
            // Post-loop `c`/`s` are the j = NSPS_BASEBAND values, i.e.
            // the inter-symbol rotation the coherent block step wants.
            isqs.cf[t][i] = c[t];
            isqs.sf[t][i] = s[t];
            isqs.is[t][i] = iacc[t];
            isqs.qs[t][i] = qacc[t];
        }
    }

    isqs
}

/// Arm C — the uncounted oscillator-table term.
#[inline(never)]
fn arm_c(idat: &[f32], qdat: &[f32]) {
    log::info!("--- arm C: oscillator tables ---");
    const REPS: usize = 20;
    // A plausible mid-band alignment; the arm measures per-call cost,
    // which does not depend on the exact hypothesis.
    let (f0, lag, drift) = (0.0f32, 1125i32, 0.0f32);

    // Boxed: `IsQs` is 10 368 B, and two of them live at once for the
    // bit-identity check. On the stack that is a fifth of the bench
    // task's whole allowance.
    let a = alloc::boxed::Box::new(tone_amplitudes(idat, qdat, f0, lag, drift));
    let b = alloc::boxed::Box::new(tone_amplitudes_inline(idat, qdat, f0, lag, drift));
    let identical = a.is == b.is && a.qs == b.qs && a.cf == b.cf && a.sf == b.sf;
    drop(a);
    drop(b);

    let t = now_us();
    for _ in 0..REPS {
        core::hint::black_box(tone_amplitudes(idat, qdat, f0, lag, drift));
    }
    let tabled = (now_us() - t) / REPS as i64;

    let t = now_us();
    for _ in 0..REPS {
        core::hint::black_box(tone_amplitudes_inline(idat, qdat, f0, lag, drift));
    }
    let inlined = (now_us() - t) / REPS as i64;

    log::info!(
        "  tone_amplitudes: tabled {tabled} us, recurrence-inline {inlined} us \
         ({}% of tabled), bit-identical = {identical}",
        if tabled > 0 { inlined * 100 / tabled } else { 0 },
    );
}

fn log_heap(tag: &str) {
    unsafe {
        log::info!(
            "heap {tag}: internal {} KB (largest contig {} KB), PSRAM {} KB",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            ) / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
        );
    }
}

/// Stack for the bench task.
///
/// The candidate loop is stack-hungry in a way the FT8 benches are
/// not: `IsQs` is 10 368 B and the refine cascade holds two live
/// (`best_isqs` plus the current `eval`), while `tone_amplitudes`
/// itself carries 8 × 257 f32 = 8 224 B of oscillator tables in its
/// own frame. The first attempt ran on the 32 KB main task and died
/// in the *first* `log::info!` — LoadProhibited inside
/// `multi_heap_internal_lock`, with SP already 42 KB below the task
/// stack, which is what a blown frame looks like from the outside.
///
/// This is a measured cost of the loop, not bench overhead: any
/// production embedded WSPR path pays the same stack.
const BENCH_STACK: u32 = 96 * 1024;

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` passes the `&'static [u8]` asset as a thin
    // pointer + length pair boxed into a `'static` tuple that outlives
    // the task.
    let bin: &'static [u8] = unsafe { *(arg as *const &'static [u8]) };
    bench_body(bin);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// `bin` is the baked baseband asset (`include_bytes!`).
pub fn run(bin: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let arg: &'static &'static [u8] = alloc::boxed::Box::leak(alloc::boxed::Box::new(bin));
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"wspr_bench".as_ptr(),
            BENCH_STACK,
            arg as *const _ as *mut core::ffi::c_void,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create the bench task ({BENCH_STACK} B stack)");
    }
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn bench_body(bin: &'static [u8]) {
    log::info!("=== WSPR candidate-loop bench (issue #260 Phase 1/2) ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!(
        "baked baseband: {} bytes, {} samples/channel, max_candidates = {}",
        bin.len(),
        NBB,
        MAX_CANDIDATES,
    );
    log_heap("boot");

    // `coarse_baseband`'s FFT is 512-point — well inside the ESP-DSP
    // backend's range. `decimate_to_baseband`'s NFFT1 = 1_474_560 is
    // not, which is exactly why the baseband arrives baked.
    esp_dsp_fft::prewarm(512);

    let ps_i = alloc_caps(NBB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT).expect("PSRAM idat");
    let ps_q = alloc_caps(NBB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT).expect("PSRAM qdat");
    let (ps_i, ps_q) = unsafe {
        (
            core::slice::from_raw_parts_mut(ps_i, NBB),
            core::slice::from_raw_parts_mut(ps_q, NBB),
        )
    };

    let a_total = arm("A (PSRAM)", bin, ps_i, ps_q);

    // Arm C runs against the PSRAM buffers, freshly reloaded — arm A
    // left them holding its own subtraction residual.
    load_baseband(bin, ps_i, ps_q);
    arm_c(ps_i, ps_q);

    // Arm B. Try both buffers internal; fall back to a split arm.
    log_heap("pre-B");
    let int_i = alloc_caps(NBB, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    let int_q = int_i.and_then(|_| alloc_caps(NBB, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    let b_total = match (int_i, int_q) {
        (Some(i), Some(q)) => {
            let (bi, bq) =
                unsafe { (core::slice::from_raw_parts_mut(i, NBB), core::slice::from_raw_parts_mut(q, NBB)) };
            let t = arm("B (both internal SRAM)", bin, bi, bq);
            free_caps(i);
            free_caps(q);
            Some((t, "both internal"))
        }
        (Some(i), None) => {
            log::warn!(
                "only one 180 KiB internal block available — running the split arm \
                 (idat internal, qdat PSRAM). Half the traffic moves, so read the \
                 delta as roughly half of a full arm B."
            );
            let bi = unsafe { core::slice::from_raw_parts_mut(i, NBB) };
            let t = arm("B' (idat internal, qdat PSRAM)", bin, bi, ps_q);
            free_caps(i);
            Some((t, "split (idat only)"))
        }
        _ => {
            log::warn!("no 180 KiB internal block free — arm B not runnable");
            None
        }
    };

    if let Some((b, kind)) = b_total {
        // Positive = PSRAM cost. The plan's reading: < ~15 % compute-
        // bound, > ~40 % bandwidth-bound.
        let delta_pct = if b > 0 { (a_total - b) * 100 / b } else { 0 };
        log::info!(
            "=== Phase 2: A (PSRAM) {} ms vs B ({}) {} ms  ->  A is {}% slower ===",
            a_total / 1000,
            kind,
            b / 1000,
            delta_pct,
        );
    }

    log::info!("=== bench complete ===");
}
