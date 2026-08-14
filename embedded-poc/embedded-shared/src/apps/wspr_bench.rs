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

use mfsk_core::fec::conv::fano::instrument as fano_hist;
use mfsk_core::wspr::coarse_baseband::{coarse_baseband, BasebandCandidate};
use mfsk_core::wspr::decode::{
    decode_at_baseband, rank_pass2_candidates, WsprCallsignTable, WsprResult,
};
use mfsk_core::wspr::demod::{tone_amplitudes, IsQs, NSPS_BASEBAND, N_SYMBOLS, TONE_SPACING_HZ};
use mfsk_core::wspr::instrument;
use mfsk_core::wspr::subtract::subtract_signal_baseband;

use crate::esp_dsp_fft;
use crate::wspr_dual_core;

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
    if p.is_null() {
        None
    } else {
        Some(p)
    }
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
    fano: fano_hist::Snapshot,
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
             | minsync2 rejected {}/{} | fano {}/{} | osd {}/{}",
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
            c.minsync2_rejected,
            c.candidates,
            c.fano_ok,
            c.fano_attempts,
            c.osd_ok,
            c.osd_attempts,
        );
        let f = &self.fano;
        let pct = |n: u32, d: u32| if d > 0 { n / (d / 100).max(1) } else { 0 };
        log::info!(
            "          fano cycles (budget {}): converged {} [mean {}% max {}%] | failed {} [mean {}%]",
            f.budget,
            f.ok_count,
            pct(f.ok_cycles_mean(), f.budget),
            pct(f.ok_cycles_max, f.budget),
            f.fail_count,
            pct(f.fail_cycles_mean(), f.budget),
        );
        log::info!(
            "          decile  ok {:?}\n          decile fail {:?}",
            f.ok_decile,
            f.fail_decile,
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
    // Slot-deadline anchor for `Pass2Budget::DeadlineDriven` — see
    // that variant's own doc comment. Captured here, not inside the
    // pass-2 block, so it reflects the *whole* scan's start (matches
    // a production controller calling `decode_scan` once at slot
    // end), not just pass 2's own local start.
    let scan_start_us = now_us();

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
        let f0 = fano_hist::snapshot();

        let t = now_us();
        let mut cands: Vec<BasebandCandidate> =
            coarse_baseband(idat, qdat, PAD_AUDIO, MAX_CANDIDATES, EARLY_MAX_DRIFT);
        cands.truncate(MAX_CANDIDATES);
        let coarse_us = now_us() - t;
        log::info!(
            "    [coarse {} done: {} cand, stack headroom {} B]",
            early_pass,
            cands.len(),
            stack_headroom(),
        );
        assert_heap_ok("coarse_baseband");

        // `wspr_dual_core::pass01_split` work-steals `cands` across
        // both cores when the runtime guard finds room; falls back to
        // the sequential loop (unchanged from before dual-core
        // existed) otherwise — never a crash, just no speedup for
        // this particular run. See that module's doc comment for why
        // pass 0/1 specifically (not pass 2) uses work-steal.
        let t = now_us();
        let raw: Vec<(WsprResult, usize)> = if let Some(r) =
            wspr_dual_core::pass01_split(idat, qdat, SAMPLE_RATE, PAD_AUDIO, cands.clone())
        {
            log::info!(
                "      p{early_pass}: dual-core work-steal over {} cand",
                cands.len()
            );
            r
        } else {
            let mut raw = Vec::new();
            for (ci, c) in cands.iter().enumerate() {
                let t_c = now_us();
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
                log::info!(
                    "      p{early_pass} cand {}/{}: {} ms",
                    ci + 1,
                    cands.len(),
                    (now_us() - t_c) / 1000,
                );
            }
            raw
        };
        let decode_us = now_us() - t;
        log::info!(
            "    [decode {early_pass} done: stack headroom {} B]",
            stack_headroom()
        );
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
            subtract_signal_baseband(idat, qdat, f0_audio, shift_baseband, d.drift_hz, &symbols);
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
            fano: fano_hist::snapshot().since(f0),
        });
    }

    // Pass 2 — the final pass, with the callsign table the early
    // passes' Fano decodes populated. It runs unconditionally.
    let mut confirmed = WsprCallsignTable::new();
    for (d, _) in &found {
        confirmed.record(&d.message);
    }
    let c0 = instrument::snapshot();
    let f0 = fano_hist::snapshot();

    let t = now_us();
    let cands2: Vec<BasebandCandidate> =
        coarse_baseband(idat, qdat, PAD_AUDIO, MAX_CANDIDATES, PASS2_MAX_DRIFT);
    let coarse_us = now_us() - t;
    log::info!(
        "    [coarse 2 done: {} cand, {} ms, stack headroom {} B]",
        cands2.len(),
        coarse_us / 1000,
        stack_headroom(),
    );

    // The OSD-off replay that measured the Fano/OSD split lived here.
    // Removed once recorded (242.2 s = 26.3 % of pass 2, see
    // `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`): it doubles the
    // most expensive pass for a number that does not change between
    // runs, and that cost was what pushed the opt-level A/B past its
    // capture window before it could print the decode list.
    //
    // Stage 1 (`rank_pass2_candidates`): refine + `minsync2`-filter
    // every coarse candidate (cheap), rank by refined sync, keep the
    // top `PASS2_DEEP_LADDER_TOP_N`. Stage 2
    // (`wspr_dual_core::pass2_split`): the expensive Fano/OSD ladder,
    // one survivor per core when the runtime guard finds room —
    // `PASS2_DEEP_LADDER_TOP_N` (2) is chosen to exactly match the
    // core count, so this is a static 1-1 split, not work-steal (see
    // that module's doc comment for why). Falls back to running both
    // survivors on `wspr_scan` sequentially (still faster than the
    // pre-topN full ladder over every candidate) if there's no room.
    let t = now_us();
    let ranked = rank_pass2_candidates(idat, qdat, &cands2);
    let deadline_us = match PASS2_BUDGET {
        Pass2Budget::Unlimited => None,
        Pass2Budget::DeadlineDriven => Some(scan_start_us + SLOT_US),
        Pass2Budget::RecallPriority(budget) => Some(t + budget),
    };
    let raw2: Vec<WsprResult> = wspr_dual_core::pass2_split(
        idat,
        qdat,
        SAMPLE_RATE,
        PAD_AUDIO,
        &ranked,
        &confirmed,
        deadline_us,
    );
    let decode_us = now_us() - t;
    log::info!(
        "      p2 deep-ladder over {} candidates ({} ranked): {} decoded, {} ms",
        cands2.len(),
        ranked.len(),
        raw2.len(),
        decode_us / 1000,
    );
    log::info!(
        "    [pass 2 decode done: stack headroom {} B]",
        stack_headroom()
    );
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
        fano: fano_hist::snapshot().since(f0),
    });

    (stats, seen)
}

/// Run one arm end-to-end and log it.
#[inline(never)]
fn arm(name: &str, bin: &[u8], idat: &mut [f32], qdat: &mut [f32]) -> i64 {
    log::info!("--- arm {name} ---");
    load_baseband(bin, idat, qdat);
    instrument::reset();
    fano_hist::reset();

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
        if tabled > 0 {
            inlined * 100 / tabled
        } else {
            0
        },
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
/// not: `IsQs` is 10 368 B and `tone_amplitudes` itself carries
/// 8 × 257 f32 = 8 224 B of oscillator tables in its own frame. The
/// first attempt ran on the 32 KB main task and died in the *first*
/// `log::info!` — LoadProhibited inside `multi_heap_internal_lock`,
/// with SP already 42 KB below the task stack, which is what a blown
/// frame looks like from the outside.
///
/// 90 KiB, not the original 128 KiB: `refine_cascade`'s ping-pong
/// rewrite (`tone_amplitudes_into` + `core::mem::swap`, replacing a
/// by-value-returning closure called from five distinct source
/// locations) cut the measured cumulative peak from 112.1 KB to
/// **61.5 KB** (128 KiB stack, 68 148 B headroom on a full pass-0/1/2
/// run through `osd_decode_packed`) — see
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`, "refine_cascade
/// stack audit". 90 KiB keeps ~27 KB of margin over that measured
/// peak while freeing the ~38 KB the old 128 KiB reservation was
/// carrying unused — room `wspr_dual_core`'s workers need to fit
/// inside the ~236 KB largest contiguous free block at all.
///
/// This is a measured cost of the loop, not bench overhead: any
/// production embedded WSPR path pays the same stack.
const SCAN_STACK: u32 = 76 * 1024;

/// WSPR slot period — the wall-clock deadline a production embedded
/// decoder is racing against, matching the protocol's own slot
/// length. Anchor for [`Pass2Budget::DeadlineDriven`].
const SLOT_US: i64 = 120_000_000;

/// How pass 2's failing-candidate ladder (up to 4 x 17 = 68 DT
/// peak-up x nblocks positions, ~48.1 s on the golden file's own
/// non-converging candidate — see "Dual-core, take two" in
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`) is bounded.
/// Threaded to [`wspr_dual_core::pass2_split`]'s `deadline_us`
/// parameter — see that function's own doc comment for the exact
/// semantics (per-candidate, not a total-pass-2 budget: main and the
/// worker each get their own independent deadline check).
#[derive(Clone, Copy)]
#[allow(dead_code)] // both non-default variants are real, selectable configurations
enum Pass2Budget {
    /// No cutoff — wsprd-faithful, full ladder always. What every
    /// measurement in `WSPR_EMBEDDED_MEASUREMENT_RESULTS.md` before
    /// this mechanism existed used.
    Unlimited,
    /// **Deadline priority.** Cuts pass 2's ladder off exactly when
    /// the slot boundary (`scan_start_us + SLOT_US`) arrives, however
    /// much of that budget pass 0/1/coarse already consumed —
    /// guarantees on-time completion as the first priority, the way
    /// a production controller racing the next slot needs. Real-
    /// signal recall risk on pass 2's own candidates therefore VARIES
    /// run-to-run with how much time the earlier stages actually
    /// took, unlike `RecallPriority`'s fixed number; a run where
    /// pass 0/1 ran unusually long leaves pass 2 less room, not more.
    DeadlineDriven,
    /// **Recall priority.** A fixed per-candidate budget (µs) that
    /// ignores the slot deadline — if pass 0/1 already ran long,
    /// TOTAL can still exceed 120 s. 40 000 000 (40 s) = 44 ladder
    /// positions (the measured worst case a converging signal needed
    /// across a 300-trial AWGN sweep concentrated at the -30/-31/
    /// -32 dB sensitivity floor — see `wspr_pass2_ladder_position_
    /// sweep`, `tests/wspr_sweep.rs`) x 0.7071 s/position (S3's own
    /// measured per-position cost, from the golden file's 68-position
    /// full-ladder run) x 1.3 margin, rounded down. Choose this over
    /// `DeadlineDriven` when losing a real weak decode is worse than
    /// running over the slot.
    RecallPriority(i64),
}

const PASS2_BUDGET: Pass2Budget = Pass2Budget::DeadlineDriven;

/// Stack for the bandwidth/oscillator task.
///
/// `tone_amplitudes` alone needs ~25 KB (8 × 257 f32 of tables in its
/// own frame, plus the 10 368 B `IsQs` return slot). 48 KB covers it
/// with margin **and** leaves a 180 KiB contiguous internal-DRAM block
/// free, which is the whole point: at `SCAN_STACK` there is no such
/// block left, so the Phase 2 A/B has to happen before the scan task
/// exists. Measured: with a 96 KB stack the largest free internal
/// block is 136 KB; with 48 KB it is 184 KB.
const BW_STACK: u32 = 48 * 1024;

/// Set by the bandwidth task when it is done, so the scan task (whose
/// stack is too big to coexist with a 180 KiB internal buffer) is not
/// created until the internal DRAM is free again.
static BW_DONE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

extern "C" fn bw_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` leaks a `&'static &'static [u8]` that outlives
    // every task created here.
    let bin: &'static [u8] = unsafe { *(arg as *const &'static [u8]) };
    bandwidth_body(bin);
    BW_DONE.store(true, core::sync::atomic::Ordering::Release);
    // Deleting the task returns its stack to the internal heap, which
    // is what makes room for the scan task.
    unsafe { esp_idf_svc::sys::vTaskDelete(core::ptr::null_mut()) };
}

extern "C" fn scan_task(arg: *mut core::ffi::c_void) {
    // SAFETY: as above.
    let bin: &'static [u8] = unsafe { *(arg as *const &'static [u8]) };
    scan_body(bin);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// `bin` is the baked baseband asset (`include_bytes!`).
/// Device round-trip check for the `EspDspPlanner` fc32 fix (issue
/// #260 follow-up): plans a small FFT, then a much bigger one, then
/// the small size again in the same process — exactly the
/// interleaving that silently produced garbage before
/// `esp_dsp_fft::ensure_fc32_table` started forcing a real
/// `dsps_fft2r_deinit_fc32` + `dsps_fft2r_init_fc32` on every size
/// change instead of trusting the C library's one-shot init gate.
/// Logs PASS/FAIL over serial; does not panic on failure so the rest
/// of the bench still runs and the log is still captured.
fn fft_multisize_selftest() {
    use core::f32::consts::TAU;
    use mfsk_core::engine::fft::default_planner;
    use num_complex::Complex32;

    fn tone(n: usize, bin: usize) -> Vec<Complex32> {
        (0..n)
            .map(|k| Complex32::from_polar(1.0, TAU * bin as f32 * k as f32 / n as f32))
            .collect()
    }

    fn peak_bin(buf: &[Complex32]) -> usize {
        buf.iter()
            .enumerate()
            .max_by(|a, b| a.1.norm_sqr().partial_cmp(&b.1.norm_sqr()).unwrap())
            .unwrap()
            .0
    }

    fn check(label: &str, n: usize, bin: usize) -> bool {
        let mut planner = default_planner();
        let mut buf = tone(n, bin);
        planner.plan_forward(n).process(&mut buf);
        let got = peak_bin(&buf);
        log::info!("fft self-test [{label}]: N={n} bin={bin} -> peak {got}");
        got == bin
    }

    log::info!("=== fft multisize self-test (issue #260 follow-up) ===");
    let mut ok = true;
    // Matches production `coarse_baseband`'s own size.
    ok &= check("a", 512, 7);
    // Well above 512 and well inside `CONFIG_DSP_MAX_FFT_SIZE_8192`
    // (this repo's actual configured ceiling — see
    // `esp_dsp_fft`'s module doc for why the WSPR LPF experiment's
    // 32768 was never really valid here). Any size other than 512
    // exercises the interleaving that used to corrupt silently
    // (reading past a 512-entry twiddle table); 4096 keeps this test
    // inside the ceiling that's actually configured today.
    ok &= check("b", 4096, 100);
    // Back to 512 — a smaller size requested again right after a
    // bigger one, the other half of the same interleaving.
    ok &= check("c", 512, 3);
    if ok {
        log::info!("fft multisize self-test: PASS");
    } else {
        log::error!("fft multisize self-test: FAIL");
    }
}

/// Install the ESP-IDF logger, at most once per boot.
///
/// `EspLogger::initialize_default` ends in `log::set_logger(..).unwrap()`,
/// so calling it twice aborts the process — a boot loop whose only
/// visible cause is a panic inside `esp-idf-svc`'s `log.rs`. A bin that
/// has to do its own setup before [`run`] (bringing WiFi up, for
/// instance, which needs the logger live to report what it did) must go
/// through here rather than calling the ESP-IDF function directly.
///
/// `no_std` crate, hence an atomic rather than `std::sync::Once`.
pub fn init_logger_once() {
    static LOGGER_READY: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);
    if LOGGER_READY
        .compare_exchange(
            false,
            true,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        )
        .is_ok()
    {
        esp_idf_svc::log::EspLogger::initialize_default();
    }
}

pub fn run(bin: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    fft_multisize_selftest();

    // The candidate loop runs compute-bound for minutes at a time
    // without yielding, so IDLE0 starves and the task watchdog fires
    // every 5 s — printing a full symbolised backtrace over the serial
    // link from an ISR, inside the timed regions. Deinit it rather than
    // measure the watchdog's console traffic. Not a bench artefact to
    // wave away: a production embedded WSPR decoder hogs a core the
    // same way and has to make the same call.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");
    wspr_dual_core::init();
    log_heap("boot");

    let arg: &'static &'static [u8] = alloc::boxed::Box::leak(alloc::boxed::Box::new(bin));
    let argp = arg as *const _ as *mut core::ffi::c_void;

    spawn(c"wspr_bw", bw_task, BW_STACK, argp);
    while !BW_DONE.load(core::sync::atomic::Ordering::Acquire) {
        unsafe { esp_idf_svc::sys::vTaskDelay(100) };
    }
    // Give the idle task a moment to reap the deleted task's stack.
    unsafe { esp_idf_svc::sys::vTaskDelay(100) };
    log_heap("post-bandwidth");

    spawn(c"wspr_scan", scan_task, SCAN_STACK, argp);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn spawn(
    name: &core::ffi::CStr,
    entry: extern "C" fn(*mut core::ffi::c_void),
    stack: u32,
    arg: *mut core::ffi::c_void,
) {
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(entry),
            name.as_ptr(),
            stack,
            arg,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create {name:?} ({stack} B stack)");
    }
}

/// Phase 2 + arm C, in a task small enough to leave a 180 KiB
/// contiguous internal-DRAM block free.
///
/// The plan's Phase 2 runs the *whole scan* twice, once per memory
/// tier. That is not available on this hardware: the refine cascade
/// needs ~93 KB of stack (measured — see `SCAN_STACK`), and 93 KB plus
/// two 180 KiB buffers does not fit the S3's internal DRAM, of which
/// ~326 KB is free to an application. So Phase 2 is measured on
/// `tone_amplitudes` directly instead. That is not a weaker
/// substitute: `tone_amplitudes` **is** the loop's traffic — every one
/// of its calls streams 162 x 256 x 2 x 4 B = 324 KiB of baseband, and
/// the whole scan is a few thousand of them.
///
/// Arms, all on identical data at an identical hypothesis:
///   - both buffers in PSRAM,
///   - `idat` internal / `qdat` PSRAM (half the traffic moved),
///   - both internal, when the memory happens to allow it.
fn bandwidth_body(bin: &'static [u8]) {
    log::info!("=== Phase 2: is the candidate loop bandwidth-bound? ===");
    // `coarse_baseband`'s FFT is 512-point — well inside the ESP-DSP
    // backend's range. `decimate_to_baseband`'s NFFT1 = 1_474_560 is
    // not, which is exactly why the baseband arrives baked.
    esp_dsp_fft::prewarm(512);

    let ps_i_raw = alloc_caps(NBB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT).expect("PSRAM idat");
    let ps_q_raw = alloc_caps(NBB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT).expect("PSRAM qdat");
    let (ps_i, ps_q) = unsafe {
        (
            core::slice::from_raw_parts_mut(ps_i_raw, NBB),
            core::slice::from_raw_parts_mut(ps_q_raw, NBB),
        )
    };
    load_baseband(bin, ps_i, ps_q);

    let int_i_raw = alloc_caps(NBB, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    let int_q_raw = int_i_raw.and_then(|_| alloc_caps(NBB, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    log::info!(
        "internal 180 KiB blocks obtained: idat = {}, qdat = {}",
        int_i_raw.is_some(),
        int_q_raw.is_some(),
    );

    let both_psram = time_tone_amplitudes("both PSRAM", ps_i, ps_q);

    if let Some(i) = int_i_raw {
        let int_i = unsafe { core::slice::from_raw_parts_mut(i, NBB) };
        int_i.copy_from_slice(ps_i);
        let split = time_tone_amplitudes("idat internal, qdat PSRAM", int_i, ps_q);

        if let Some(q) = int_q_raw {
            let int_q = unsafe { core::slice::from_raw_parts_mut(q, NBB) };
            int_q.copy_from_slice(ps_q);
            let both_int = time_tone_amplitudes("both internal SRAM", int_i, int_q);
            report_bandwidth(both_psram, both_int, "both internal");
            free_caps(q);
        }
        report_bandwidth(both_psram, split, "idat internal only (half the traffic)");
        free_caps(i);
    } else {
        log::warn!("no 180 KiB internal block free — Phase 2 A/B not runnable");
    }

    arm_c(ps_i, ps_q);

    free_caps(ps_i_raw);
    free_caps(ps_q_raw);
}

/// Mean microseconds per `tone_amplitudes` call over `REPS`.
fn time_tone_amplitudes(label: &str, idat: &[f32], qdat: &[f32]) -> i64 {
    const REPS: usize = 20;
    // A plausible mid-band alignment. The arm measures per-call cost,
    // which does not depend on the hypothesis.
    let (f0, lag, drift) = (0.0f32, 1125i32, 0.0f32);
    // Warm anything cacheable before the timed loop.
    core::hint::black_box(tone_amplitudes(idat, qdat, f0, lag, drift));
    let t = now_us();
    for _ in 0..REPS {
        core::hint::black_box(tone_amplitudes(idat, qdat, f0, lag, drift));
    }
    let per_call = (now_us() - t) / REPS as i64;
    log::info!("  tone_amplitudes [{label}] = {per_call} us/call");
    per_call
}

fn report_bandwidth(psram_us: i64, faster_us: i64, kind: &str) {
    let delta_pct = if faster_us > 0 {
        (psram_us - faster_us) * 100 / faster_us
    } else {
        0
    };
    log::info!(
        "  => PSRAM vs {kind}: {psram_us} us vs {faster_us} us  ->  PSRAM is {delta_pct}% slower \
         (plan's reading: < ~15% compute-bound, > ~40% bandwidth-bound)"
    );
}

/// Which heap does a `Complex32` FFT buffer actually land in, and does
/// asking for 16-byte alignment change the answer?
///
/// Issue #260: aligning `wspr::subtract`'s 8 192-point buffers cut that
/// stage 12.7 %, but the identical change on `coarse_baseband`'s
/// 512-point buffer *cost* 219 ms. The suspected mechanism is
/// placement, not alignment — ESP-IDF's `malloc` applies
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` (4 096 bytes here) to keep
/// small allocations in internal DRAM, while Rust routes any request
/// with alignment above its 8-byte minimum through `posix_memalign`,
/// which need not honour the same policy. A 512-point buffer is
/// exactly 4 096 bytes, i.e. exactly at that threshold; an 8 192-point
/// one is 64 KB and belongs in PSRAM either way.
///
/// The address decides it. On ESP32-S3, internal DRAM is
/// `0x3FC8_0000..0x3FD0_0000` and mapped PSRAM is
/// `0x3C00_0000..0x3E00_0000`.
fn probe_alloc_placement() {
    fn where_is(p: usize) -> &'static str {
        match p {
            0x3FC8_0000..0x3FD0_0000 => "internal DRAM",
            0x3C00_0000..0x3E00_0000 => "PSRAM",
            _ => "other",
        }
    }
    log::info!("=== alloc placement probe (issue #260) ===");

    // Does perturbing the allocation order move `coarse_baseband`'s
    // 735 KB `ps` array, and by how much? `build_spectro` allocates a
    // 4 096-byte FFT buffer and then `ps = vec![0.0f32; n_time*NFFT]`;
    // `refine_alignment_top_k` then reads `ps` ~10^8 times from PSRAM
    // in a 7-float window per inner step. Rows are 2 048 B apart, so
    // every row inherits the base's 32-byte cache-line phase — which
    // makes `ps`'s base address, not the FFT buffer's alignment, the
    // thing that moved when an extra allocation was introduced.
    const PS_BYTES: usize = 359 * 512 * 4;
    for extra in [0usize, 1] {
        let _pad: alloc::vec::Vec<alloc::vec::Vec<u8>> =
            (0..extra).map(|_| alloc::vec![0u8; 4096]).collect();
        let _fftbuf = alloc::vec![num_complex::Complex32::new(0.0, 0.0); 512];
        let ps = alloc::vec![0.0f32; PS_BYTES / 4];
        let p = ps.as_ptr() as usize;
        log::info!("  ps replica, {extra} extra 4K alloc(s): {p:#010x} (mod 32 = {})", p % 32);
    }

    for len in [512usize, 8192] {
        let plain = alloc::vec![num_complex::Complex32::new(0.0, 0.0); len];
        let mut aligned = mfsk_core::engine::fft::AlignedComplexBuf::zeroed(len);
        let (pp, pa) = (
            plain.as_ptr() as usize,
            aligned.as_mut_slice().as_ptr() as usize,
        );
        log::info!(
            "  len {len:>5} ({:>6} B): plain {pp:#010x} [{}] align%16={} | aligned {pa:#010x} [{}] align%16={}",
            len * 8,
            where_is(pp),
            pp % 16,
            where_is(pa),
            pa % 16,
        );
    }
}

fn scan_body(bin: &'static [u8]) {
    log::info!("=== Phase 1: one decode_scan, PSRAM-resident baseband ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!(
        "baked baseband: {} bytes, {} samples/channel, max_candidates = {}",
        bin.len(),
        NBB,
        MAX_CANDIDATES,
    );

    probe_alloc_placement();

    let ps_i = alloc_caps(NBB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT).expect("PSRAM idat");
    let ps_q = alloc_caps(NBB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT).expect("PSRAM qdat");
    let (ps_i, ps_q) = unsafe {
        (
            core::slice::from_raw_parts_mut(ps_i, NBB),
            core::slice::from_raw_parts_mut(ps_q, NBB),
        )
    };

    arm("A (PSRAM)", bin, ps_i, ps_q);

    // Issue #260 open question: does the FFT-based LPF in
    // `wspr::subtract` actually reach the PIE kernel in place, or does
    // every call pay `AlignedStaging`'s copy round-trip? `lpf_apply_fft`
    // works out of plain `Vec<Complex32>` (alignment 4), and a staged
    // 8192-point call copies 64 KB in and 64 KB back out, 13 times per
    // subtract. Whether the allocator hands back a 16-byte-aligned block
    // anyway is only answerable here, on the device.
    let (aligned_calls, aligned_mask, staged_calls, staged_mask) =
        crate::esp_dsp_fft::pie_alignment_report();
    log::info!(
        "  PIE alignment: in-place {aligned_calls} calls (len mask {aligned_mask:#x}) \
         | staged {staged_calls} calls (len mask {staged_mask:#x})",
    );

    log::info!("=== bench complete ===");
}
