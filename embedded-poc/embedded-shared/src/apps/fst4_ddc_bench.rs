//! FST4-60 **DDC-fed** coarse-sync + refine + decode bench, real WAV
//! in, on ESP32-S3 (LX7) — `docs/notes/FST4_DDC_DESIGN.md` stage 5's
//! first half.
//!
//! `apps::fst4_bench` (issue #306/#307) proved the LLR/BP/OSD decoder
//! fits on-device, but explicitly skipped the front end: `coarse_sync`
//! and `downsample_cached` both need non-power-of-two FFT lengths
//! (`nfft1 = 7776`, `fft2_size = 6912` for FST4-60A) the embedded
//! `fft-extern`/ESP-DSP backend can't serve — that module's own doc
//! comment calls this "No `coarse_sync` on-device (can't yet — #307)".
//!
//! The DDC pipeline (`mfsk_core::engine::dsp::ddc` +
//! `mfsk_core::fst4::ddc`, `docs/notes/FST4_DDC_DESIGN.md` stages 1-4)
//! routes around that: its coarse-sync spectrogram is a power-of-two
//! `nfft1` (512 for the sniper `K=256` grid this bench uses) by
//! construction, and neither the DDC's own mixer/`FirStage`/
//! `PolyphaseResampler` cascade nor `fst4_sync_search`'s coherent
//! block correlator touch an FFT at all — only `compute_spectra`'s
//! complex path does, and that one fits the ESP-DSP ceiling. So the
//! **whole** chain — real 12 kHz audio in, decoded message out — has
//! no FFT this backend can't serve, and this bench is the first
//! on-device confirmation of that, not just a host claim.
//!
//! ## What's *not* yet PIE-accelerated
//!
//! `FirStage`/`PolyphaseResampler` are plain portable f32 Rust —
//! correct on Xtensa, not yet routed through `dsps_fird_f32_aes3`
//! (design doc §4.5's own remaining stage-5 item). This bench answers
//! "does it work and what does it cost unaccelerated", the same
//! starting point `fst4_bench`'s own LLR/BP/OSD measurement was
//! before its own six-round optimisation pass.
//!
//! ## Target window
//!
//! Both real signals on the vendored WSJT-X golden (`210115_0058.wav`)
//! sit within one sniper window: N5TM @ 1101 Hz, K9KFR @ 1331 Hz, 230 Hz
//! apart — well inside the sniper cascade's own ±250 Hz shape centred
//! on their midpoint, 1216 Hz. One DDC pass catches both.

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use num_complex::Complex32;

use mfsk_core::engine::dsp::ddc::StreamingComplexDdc;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{REFINE_DS_RATE_HZ, grid_for, sniper_cascade, sniper_refine_recenter};
use mfsk_core::msg::wsjt77::unpack77;

/// Matches `fst4::decode`'s own (private) `SYNC_Q_MIN` — see
/// `fst4_bench`'s identical constant for why it's redeclared here
/// rather than imported.
const SYNC_Q_MIN: u32 = 16;

/// Midpoint of N5TM (1101 Hz) / K9KFR (1331 Hz) — see the module doc
/// comment.
const CENTER_HZ: f32 = 1216.0;
const SEARCH_HALF_WIDTH_HZ: f32 = 250.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 50;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn stack_headroom() -> u32 {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

const MALLOC_CAP_8BIT: u32 = 1 << 2;
const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

fn log_heap(tag: &str) {
    unsafe {
        log::info!(
            "[mem] {tag}: internal {} KB (largest contig {} KB), PSRAM {} KB (largest contig {} KB)",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            ) / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
                / 1024,
        );
    }
}

pub fn init_logger_once() {
    static LOGGER_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
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

/// Recentre `cand`'s own frequency out of the coarse DDC baseband
/// down to the refine (`ds_rate`) baseband, trim the best-effort
/// combined group delay, RMS-normalise (matching `engine::pipeline::
/// refine_candidate_position_impl`'s own convention, so `process_
/// candidate_precomputed`'s LLR scaling sees what it would from
/// `downsample_cached`), then run the real `fst4_sync_search` refine.
/// Same recipe `tests/fst4_ddc_sniper_full_decode.rs::ddc_refine_and_
/// decode` verified against `downsample_cached` on host — see that
/// test for the delay-trim derivation this mirrors.
fn ddc_refine(
    cand: &SyncCandidate,
    coarse_i: &[f32],
    coarse_q: &[f32],
    coarse_delay_orig: usize,
) -> Vec<Complex32> {
    let mut refine = sniper_refine_recenter(cand.freq_hz, CENTER_HZ);
    // Sized from the sniper cascade's own fixed L/M=9/64 ratio (plus a
    // small margin for `flush`'s tail) rather than growing from empty
    // via repeated reallocation — this runs up to ~90 times per bench
    // invocation (Stage 2a + 2c), each on a buffer this module's own
    // doc comment already sizes at up to ~266 KB.
    let cap = coarse_i.len() * 9 / 64 + 16;
    let mut cd0_i = Vec::with_capacity(cap);
    let mut cd0_q = Vec::with_capacity(cap);
    refine.push(coarse_i, coarse_q, &mut cd0_i, &mut cd0_q);
    refine.flush(&mut cd0_i, &mut cd0_q);
    let refine_delay_out = refine.group_delay_output_samples();

    let total_delay_out = (coarse_delay_orig as f32 * REFINE_DS_RATE_HZ / 12_000.0).round()
        as usize
        + refine_delay_out;
    let trim = total_delay_out.min(cd0_i.len());

    let mut cd0: Vec<Complex32> = cd0_i[trim..]
        .iter()
        .zip(cd0_q[trim..].iter())
        .map(|(&i, &q)| Complex32::new(i, q))
        .collect();

    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
    cd0
}

/// `audio_bin` is the raw golden asset (`include_bytes!`,
/// `i16[720000]` little-endian — same layout `fst4_bench`'s own
/// `fst4_60_golden_audio.bin` doc comment describes). Byte-wise, not a
/// transmute: 1-byte `include_bytes!` alignment faults an unaligned
/// `i16` load on Xtensa.
pub fn run(audio_bin: &[u8]) {
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!("fst4_ddc_bench: golden asset {} bytes", audio_bin.len());
    log_heap("boot");

    let audio: Vec<i16> = audio_bin
        .as_chunks::<2>()
        .0
        .iter()
        .map(|b| i16::from_le_bytes(*b))
        .collect();
    log::info!("fst4_ddc_bench: loaded {} samples ({} s @ 12 kHz)", audio.len(), audio.len() / 12_000);

    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // ---- Stage 1: DDC + coarse_sync ----
    let t0 = now_us();
    let coarse_cfg = sniper_cascade(CENTER_HZ);
    let mut coarse = StreamingComplexDdc::new(&coarse_cfg);
    let mut coarse_i = Vec::new();
    let mut coarse_q = Vec::new();
    coarse.push_i16(&audio, &mut coarse_i, &mut coarse_q);
    coarse.flush(&mut coarse_i, &mut coarse_q);
    let coarse_delay_orig = coarse.group_delay_input_samples();
    let t_ddc = now_us() - t0;
    log::info!(
        "fst4_ddc_bench: DDC {} -> {} complex samples in {} ms",
        audio.len(),
        coarse_i.len(),
        t_ddc / 1000,
    );
    log_heap("post-ddc");

    let t1 = now_us();
    let grid = grid_for(600.0);
    let candidates = coarse_sync::<Fst4s60>(
        AudioSource::Complex(&coarse_i, &coarse_q),
        CENTER_HZ - SEARCH_HALF_WIDTH_HZ,
        CENTER_HZ + SEARCH_HALF_WIDTH_HZ,
        SYNC_MIN,
        None,
        MAX_CAND,
        RxGrid::complex(grid.fs_c, CENTER_HZ),
    );
    let t_coarse_sync = now_us() - t1;
    log::info!(
        "fst4_ddc_bench: coarse_sync (via DDC, K={}, nfft1={}) -> {} candidates in {} ms",
        grid.k,
        grid.nfft1,
        candidates.len(),
        t_coarse_sync / 1000,
    );
    for c in &candidates {
        log::info!("    cand {:8.1} Hz  dt {:+.3} s  score {:.2}", c.freq_hz, c.dt_sec, c.score);
    }
    log_heap("post-coarse-sync");

    // ---- Stage 2a: refine every raw candidate, metadata only ----
    //
    // Mirrors `engine::pipeline::dedup_refined_candidates`'s own first
    // pass exactly — same near-duplicate rule (§ below), applied
    // *before* any candidate reaches the expensive nsym=8/OSD stage,
    // not after. That function isn't `pub` (private to `engine::
    // pipeline`, same reason `fst4_bake_golden_refined_candidates`'s
    // own doc comment gives for re-deriving it rather than importing
    // it: "~20 lines and exposing the private `RefinedSurvivor` type
    // alias for one caller wasn't worth the extra public surface") —
    // reproduced here rather than skipped, unlike this bench's first
    // version, which fed every one of coarse_sync's raw (un-deduped)
    // candidates straight into LLR/BP/OSD. On real audio, several
    // near-duplicate coarse-sync cells refine onto the *same* true
    // position for each real signal (visible in this bench's own
    // logged candidate list — several entries within a few Hz/lag of
    // each other), and every one of those duplicates was paying the
    // full ladder independently — the exact cost issue #244 already
    // found and fixed for the production real-audio path.
    //
    // Unlike the host version, this can't just keep every candidate's
    // `cd0` (`Vec<Complex32>`) around to reuse post-dedup — PSRAM here
    // is single-digit MB and `candidates.len()` buffers at ~266 KB
    // each would exceed it. Stage 2a keeps only the cheap refined
    // `(freq_hz, i0, score)` triple per candidate and drops each `cd0`
    // immediately; Stage 2c rebuilds `cd0` for survivors only — a
    // second DDC-recentre pass, cheap next to LLR/BP/OSD, traded for
    // the memory this board doesn't have to spare.
    struct RefineMeta {
        freq_hz: f32,
        i0: i32,
        score: f32,
    }
    let t2a = now_us();
    let refine_meta: Vec<RefineMeta> = candidates
        .iter()
        .enumerate()
        .map(|(i, cand)| {
            let cd0 = ddc_refine(cand, &coarse_i, &coarse_q, coarse_delay_orig);
            let s2 = fst4_sync_search::<Fst4s60>(&cd0, cand);
            log::info!(
                "fst4_ddc_bench: refine-all {}/{} done ({:.1} Hz)",
                i + 1,
                candidates.len(),
                cand.freq_hz,
            );
            RefineMeta {
                freq_hz: s2.freq_hz,
                i0: s2.i0,
                score: s2.score,
            }
        })
        .collect();
    let t_refine_all = now_us() - t2a;
    log::info!(
        "fst4_ddc_bench: refined {} candidates in {} ms",
        candidates.len(),
        t_refine_all / 1000,
    );
    log_heap("post-refine-all");

    // ---- Stage 2b: dedup — same rule as engine::pipeline::
    // dedup_refined_candidates (0.10 * TONE_SPACING_HZ in frequency,
    // +/-2 downsampled samples in i0, highest-score survivor kept).
    use mfsk_core::engine::ModulationParams;
    let freq_tol = 0.10 * <Fst4s60 as ModulationParams>::TONE_SPACING_HZ;
    const I0_TOL: i32 = 2;
    let mut order: Vec<usize> = (0..candidates.len()).collect();
    order.sort_by(|&a, &b| {
        refine_meta[b]
            .score
            .partial_cmp(&refine_meta[a].score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let mut kept_positions: Vec<(f32, i32)> = Vec::new();
    let mut keep = alloc::vec![false; candidates.len()];
    for idx in order {
        let m = &refine_meta[idx];
        let dup = kept_positions
            .iter()
            .any(|&(kf, ki)| (m.freq_hz - kf).abs() < freq_tol && (m.i0 - ki).abs() <= I0_TOL);
        if !dup {
            kept_positions.push((m.freq_hz, m.i0));
            keep[idx] = true;
        }
    }
    let n_survivors = keep.iter().filter(|&&k| k).count();
    log::info!(
        "fst4_ddc_bench: dedup {} -> {} survivors",
        candidates.len(),
        n_survivors,
    );

    // ---- Stage 2c: rebuild cd0 for survivors, LLR/BP/OSD ----
    struct CandidateTiming {
        freq_hz: f32,
        refine_us: i64,
        decode_us: i64,
        decoded: bool,
    }
    let mut timings: Vec<CandidateTiming> = Vec::with_capacity(n_survivors);
    let mut results: Vec<(f32, f32, Option<String>)> = Vec::new();

    let t2c = now_us();
    let mut survivor_rank = 0usize;
    for (idx, cand) in candidates.iter().enumerate() {
        if !keep[idx] {
            continue;
        }
        survivor_rank += 1;
        // Incremental, not just the sorted summary after the loop —
        // this stage has no other progress signal (a slow candidate
        // pays the same ~7-14s/candidate the un-deduped nsym=8/OSD
        // tail cost fst4_bench's own doc comment measured, and a
        // silent multi-minute gap on the UART otherwise looks
        // indistinguishable from a hang).
        log::info!(
            "fst4_ddc_bench: survivor {survivor_rank}/{n_survivors}: {:.1} Hz, dt {:+.3} s ...",
            cand.freq_hz,
            cand.dt_sec,
        );
        let t_r = now_us();
        let cd0 = ddc_refine(cand, &coarse_i, &coarse_q, coarse_delay_orig);
        let refine_us = now_us() - t_r;
        let m = &refine_meta[idx];

        let t_d = now_us();
        let precomputed = (cd0, m.freq_hz, m.i0, m.score);
        let decoded = process_candidate_precomputed::<Fst4s60>(
            cand,
            &[],
            &mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            precomputed,
            true, // skip_snr — see fst4_bench's own identical rationale
            false,
        );
        let decode_us = now_us() - t_d;
        log::info!(
            "fst4_ddc_bench: survivor {survivor_rank}/{n_survivors} done: refine {} ms, decode {} ms, decoded={}",
            refine_us / 1000,
            decode_us / 1000,
            decoded.is_some(),
        );

        timings.push(CandidateTiming {
            freq_hz: cand.freq_hz,
            refine_us,
            decode_us,
            decoded: decoded.is_some(),
        });
        if let Some(res) = decoded {
            let text = res
                .message77()
                .try_into()
                .ok()
                .and_then(|m77: &[u8; 77]| unpack77(m77));
            results.push((res.freq_hz, res.dt_sec, text));
        }
    }
    let total_candidate_us = now_us() - t2c;

    timings.sort_by(|a, b| (b.refine_us + b.decode_us).cmp(&(a.refine_us + a.decode_us)));
    log::info!("fst4_ddc_bench: per-survivor timing, slowest first:");
    for t in &timings {
        log::info!(
            "    {:8.1} Hz  refine {:>6} ms  decode {:>6} ms  decoded={}",
            t.freq_hz,
            t.refine_us / 1000,
            t.decode_us / 1000,
            t.decoded,
        );
    }

    log::info!(
        "fst4_ddc_bench: TOTALS — ddc {} ms, coarse_sync {} ms, refine-all {} ms, dedup {} -> {} survivors, survivor refine+decode {} ms ({} decodes)",
        t_ddc / 1000,
        t_coarse_sync / 1000,
        t_refine_all / 1000,
        candidates.len(),
        n_survivors,
        total_candidate_us / 1000,
        results.len(),
    );
    for (freq_hz, dt_sec, text) in &results {
        log::info!("    {text:?} | {freq_hz:.1} Hz | dt {dt_sec:.2} s");
    }
    log::info!("fst4_ddc_bench: stack headroom = {} B", stack_headroom());
    log_heap("post-decode");
    log::info!("=== fst4_ddc_bench complete ===");
}

/// Stack for the bench task — same starting-guess rationale as
/// `fst4_bench::BENCH_STACK` (this bench's own DDC/refine stages add
/// only heap-allocated `Vec` state, no large stack frames of their
/// own, so the same order of magnitude is a reasonable starting
/// point; resize once a real "stack headroom" log line exists).
pub const BENCH_STACK: u32 = 96 * 1024;

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: mirrors `fst4_bench::bench_task`'s identical arg-passing
    // shape.
    let audio_bin: &'static [u8] = unsafe { *(arg as *const &'static [u8]) };
    run(audio_bin);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

pub fn run_task(audio_bin: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let arg: &'static &'static [u8] = alloc::boxed::Box::leak(alloc::boxed::Box::new(audio_bin));
    let argp = arg as *const _ as *mut core::ffi::c_void;

    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"fst4_ddc_bench".as_ptr(),
            BENCH_STACK,
            argp,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create fst4_ddc_bench task ({BENCH_STACK} B stack)");
    }

    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
