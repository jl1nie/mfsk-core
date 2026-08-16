//! FST4-60 decoder-only bench for ESP32-S3 (LX7) — issue #306/#307.
//!
//! Answers the question issue #306 actually asked: does the *decoder*
//! — LLR/BP/OSD over a real candidate population — fit inside
//! FST4-60's ~7 s post-slot margin on real CoreS3 hardware? Not "is
//! the whole embedded FST4 port done" (it isn't attempted here) and
//! not "is the WSPR result predictive of this" (issue #260's own
//! closing thread argued the opposite: WSPR's dominant cost was
//! Fano-sequential search burning a full node budget per failing
//! candidate, a failure mode FST4's bounded LDPC/BP + OSD-fallback
//! doesn't share).
//!
//! ## Why there is no FFT anywhere in this bench
//!
//! The first version of this bench baked only the wideband
//! `build_fft_cache` output and ran `coarse_sync` + `decode_frame` on
//! device — mirroring `wspr_bench`'s baked-baseband shape. On real
//! hardware it panicked on `coarse_sync`'s very first FFT call:
//! FST4-60A's spectrogram length (`NSPS × 2 = 7776 = 2⁵ × 3⁵`) isn't a
//! power of two, and the embedded `fft-extern`/ESP-DSP backend only
//! serves power-of-two lengths (plus one hand-rolled exception for
//! FT8's own `3840`). `downsample_cached`'s inverse FFT
//! (`fft2_size = 6912 = 2⁸ × 27`) is a second, independently-sized
//! non-power-of-two transform in the same path. Every one of FST4's 5
//! submodes hits this (filed as issue #307, with the full
//! factorization table — FST4-120's carries a bare factor of 41).
//!
//! Rather than wait on #307's FFT kernel to get a first wall-clock
//! number, this bench skips **both** FFT sites by baking each real
//! candidate already refined on a host: `mfsk-core/tests/
//! fst4_wsjtx_samples.rs::fst4_bake_golden_refined_candidates` runs
//! `coarse_sync` + `refine_candidate_position` +
//! `dedup_refined_candidates`'s own near-duplicate rule on the WSJT-X
//! golden, and bakes the survivors' `(cd0, freq_hz, i0, score)`
//! tuples — exactly what `process_candidate_basic_impl`'s
//! `precomputed_refine` parameter accepts. `decode_frame` doesn't
//! expose that parameter, so a new `engine::pipeline::
//! process_candidate_precomputed` (internal-testing-gated, same
//! visibility shape as `process_candidate_basic`) does. The result:
//! **the on-device path here never calls an FFT**, only LLR/BP/OSD —
//! exactly the "decoder" issue #306 asked about, isolated instead of
//! blocked.
//!
//! Two more FFT sites turned up past that first fix, both closed the
//! same day:
//!
//! - `GenericPipelineProtocol::snr_db`'s FST4 override
//!   (`fst4::baseline::fst4_snr_db`) calls `downsample_cached` a
//!   *second*, independent time from `fft_cache` — `fst4_raw_cs` needs
//!   the non-RMS-normalised spectrum, which the already-normalised
//!   `cd0` from `precomputed_refine` can't substitute for. Closed by a
//!   new `skip_snr` parameter on `process_candidate_basic_impl` /
//!   `process_candidate_precomputed`: `true` stores `NAN` in
//!   `DecodeResult::snr_db` instead of calling `P::snr_db` at all —
//!   this bench's SNR values are not real measurements, only its
//!   wall-clock is.
//! - `engine::llr::symbol_spectra` — the actual first step of LLR
//!   extraction, called for *every* candidate regardless of
//!   `precomputed_refine`/`skip_snr` — plans a `ds_spb =
//!   NSPS/NDOWN`-point FFT (36 for FST4-60A; 36-42 across all 5
//!   submodes), also not a power of two. Closed by
//!   `esp_dsp_fft::DirectDft`: a plain O(N²) textbook DFT (not an
//!   approximation — every FFT computes the identical sum by a faster
//!   route, so there is no algorithm-specific correctness risk the
//!   way a new fast-transform derivation would carry), wired into
//!   `EspDspPlanner::plan_forward` for any non-power-of-2 length up to
//!   `DIRECT_DFT_MAX_LEN` (64). Verified against `numpy.fft` on host
//!   at N=36/42 to ~1e-6 (f32-level precision) before flashing.
//!
//! With all three closed, the on-device path genuinely has zero FFT
//! calls left. One more thing turned up before a number came back —
//! `fft_cache` (5.7 MiB) was still being loaded and shipped even
//! though nothing on this path reads it once `precomputed_refine` +
//! `skip_snr = true` are both set; PSRAM math that had looked fine on
//! paper (893 KiB free post-load, per an earlier version of this
//! comment) turned out to leave the candidate loop's own transient
//! BP/OSD allocations no room at all — a 512 KiB request failed.
//! Dropped `fft_cache` from this bench entirely (`run_bench` no
//! longer takes it, `&[]` goes to `process_candidate_precomputed`
//! instead — safe exactly because nothing on this path dereferences
//! it); PSRAM free jumped from 87 KiB to 5.97 MiB post-load.
//!
//! **Measured, 2026-08-16, CoreS3 @ 240 MHz / opt-level 3, single
//! core:** the candidate loop — all 41 baked candidates, LLR/BP/OSD
//! only, zero FFT calls — took **89.691 s** and reached both real
//! decodes (`CQ N5TM EL29`, `CQ K9KFR EN71`), matching the host
//! golden exactly. Host `decode_loop` for the same 41 candidates is
//! 51.9 ms (`MFSK_TRACE_STAGE_FST4=1`, this box) — a **~1728×**
//! device/host ratio, roughly **13× over** FST4-60's ~7 s post-slot
//! margin.
//!
//! That ratio is the real finding here, not just the raw seconds:
//! it's close to WSPR's own *first*, wholly-unoptimized measurement
//! (issue #260: 1214.3 s against a 709.5 ms host baseline, ~1712× —
//! before `minsync2`, `opt-level=3`, the 160→240 MHz clock fix, or
//! any of that investigation's other four-and-a-half-times-total
//! speedup landed). This bench's build already has `opt-level = 3`
//! and the 240 MHz clock fix (both are `m5stack-cores3-app`-wide, not
//! per-bin) — so this *is* already past two of WSPR's early wins, and
//! the ratio still landed in WSPR's pre-optimization territory. The
//! premise this module's own earlier text repeated — that FST4's
//! bounded LDPC/BP/OSD "fails cheaply by construction" and so
//! shouldn't need WSPR's kind of optimization pass — was explicitly
//! flagged elsewhere as *"an untested claim, not a measured one"*
//! (`docs/reference/EMBEDDED.md`). It is now tested, and at this
//! unoptimized state it does not hold: something in FST4's LLR/BP/OSD
//! path costs about as much per real-world candidate as WSPR's
//! Fano-sequential search did before WSPR's own dedicated tuning
//! pass. Candidate suspects, unmeasured: the `LLR_NSYM_MAX = 8`
//! staircase rung (`4⁸ = 65536` tone-combination hypotheses per
//! group — the module doc for `fst4::decode`'s lazy-staircase code
//! calls this "128-256× FT8/FT4's own deepest rung"), and simply that
//! FST4 runs the generic f32 pipeline with no embedded-specific
//! optimisation pass at all, unlike FT8's dedicated fixed-point
//! `decode_block` or WSPR's now-four-rounds-tuned `decode_scan`. Not
//! diagnosed further here — see `docs/reference/EMBEDDED.md` and
//! issue #306 for where this leaves the "does it fit" question.
//!
//! Also measured in passing: peak stack usage was tiny —
//! `BENCH_STACK`'s 96 KiB guess left 95 064 B of headroom untouched
//! (only ~3.2 KiB actually used), a wide margin unlike `wspr_bench`'s
//! own stack history. Worth right-sizing down for a future run, not
//! urgent for a single-shot bench.
//!
//! ## What this does *not* attempt
//!
//! No `coarse_sync` on-device (can't yet — #307), no PSRAM-vs-SRAM
//! bandwidth arms, no dual-core split, no WiFi, no spot reporting, no
//! steady-state multi-slot pipeline. `wspr_bench` grew all of that
//! from real on-device findings over several rounds (issue #260);
//! building the same scaffolding here ahead of a first real
//! measurement would be guessing at what FST4 actually needs.
//!
//! The candidate population processed (41 survivors of 50 raw
//! `coarse_sync` candidates on the golden WAV, post-dedup) is the
//! same one `decode_frame_impl` would actually decode — including
//! every candidate that *fails*, not just the 2 real signals. WSPR's
//! own measurement (issue #260) found that most of a real candidate
//! loop's cost is exactly there; timing only the successes would
//! understate this the same way.

extern crate alloc;

use alloc::vec::Vec;

use num_complex::Complex32;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

/// Matches `fst4::decode`'s own (private) `SYNC_Q_MIN` — WSJT-X's
/// `get_fst4_bitmetrics.f90` pre-ladder nsync gate (issue #197). Not
/// reachable from here (it isn't `pub`, deliberately — see that
/// const's own doc comment), so redeclared; if it ever moves this
/// bench silently stops matching the shipped gate rather than failing
/// to compile, which is the one place this file trades safety for
/// not needing a crate change just to flash a bench.
const SYNC_Q_MIN: u32 = 16;

const MALLOC_CAP_8BIT: u32 = 1 << 2;
const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Bytes of the calling task's stack never touched so far — same
/// technique `wspr_scan::stack_headroom` uses, duplicated rather than
/// shared across a feature boundary this bench doesn't otherwise need.
fn stack_headroom() -> u32 {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

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

/// One baked, already-refined candidate — see the module doc and
/// `fst4_bake_golden_refined_candidates`'s own doc comment for the
/// exact byte layout this parses.
struct RefinedCandidate {
    cand: SyncCandidate,
    cd0: Vec<Complex32>,
    refined_freq_hz: f32,
    refined_i0: i32,
    refined_score: f32,
}

/// `bin` is the baked refined-candidates asset (`include_bytes!`).
/// Byte-wise, not a transmute: `include_bytes!` gives 1-byte alignment
/// and Xtensa faults on an unaligned `f32`/`i32` load.
fn load_refined_candidates(bin: &[u8]) -> Vec<RefinedCandidate> {
    let mut off = 0usize;
    let n = u32::from_le_bytes(bin[off..off + 4].try_into().unwrap()) as usize;
    off += 4;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let freq_hz = f32::from_le_bytes(bin[off..off + 4].try_into().unwrap());
        let dt_sec = f32::from_le_bytes(bin[off + 4..off + 8].try_into().unwrap());
        let score = f32::from_le_bytes(bin[off + 8..off + 12].try_into().unwrap());
        let refined_freq_hz = f32::from_le_bytes(bin[off + 12..off + 16].try_into().unwrap());
        let refined_i0 = i32::from_le_bytes(bin[off + 16..off + 20].try_into().unwrap());
        let refined_score = f32::from_le_bytes(bin[off + 20..off + 24].try_into().unwrap());
        off += 24;
        let mut cd0 = Vec::with_capacity(FST4_60A_DOWNSAMPLE.fft2_size);
        for _ in 0..FST4_60A_DOWNSAMPLE.fft2_size {
            let re = f32::from_le_bytes(bin[off..off + 4].try_into().unwrap());
            let im = f32::from_le_bytes(bin[off + 4..off + 8].try_into().unwrap());
            cd0.push(Complex32::new(re, im));
            off += 8;
        }
        out.push(RefinedCandidate {
            cand: SyncCandidate {
                freq_hz,
                dt_sec,
                score,
            },
            cd0,
            refined_freq_hz,
            refined_i0,
            refined_score,
        });
    }
    assert_eq!(off, bin.len(), "refined-candidates asset byte accounting mismatch");
    out
}

/// Install the ESP-IDF logger, at most once per boot — same guard
/// `wspr_bench::init_logger_once` uses (`EspLogger::initialize_default`
/// aborts on a second call).
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

/// Runs `process_candidate_precomputed::<Fst4s60>` over every baked,
/// already-refined candidate — no FFT anywhere in this call chain
/// (see the module doc for why). `refined_bin` is the `include_bytes!`
/// blob produced by `fst4_bake_golden_refined_candidates`.
///
/// No `fft_cache` here, deliberately — the first real-device run
/// (2026-08-16) loaded it anyway (5.7 MiB) alongside the 2.16 MiB of
/// baked `cd0` buffers, leaving only ~87 KiB PSRAM free, and the
/// candidate loop's own transient allocations (BP/OSD scratch) then
/// failed a 512 KiB request. `fft_cache` was never actually *read* on
/// this path: `precomputed_refine` already skips the one call
/// (`downsample_cached`) that would use it, and `skip_snr = true`
/// skips the other (`fst4_raw_cs`, inside `P::snr_db`) — so it was
/// PSRAM spent on a buffer nothing touches. Dropping it is what turns
/// "peak usage was never actually the risk" (this module's earlier,
/// wrong conclusion — see below) into true.
pub fn run_bench(refined_bin: &[u8]) {
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!(
        "fst4_bench: baked asset refined_candidates={} bytes",
        refined_bin.len(),
    );
    log_heap("boot");

    let t_load = now_us();
    let candidates = load_refined_candidates(refined_bin);
    log::info!(
        "fst4_bench: loaded {} refined candidates in {} ms",
        candidates.len(),
        (now_us() - t_load) / 1000,
    );
    log_heap("post-load");

    // The candidate loop runs compute-bound for however long it takes
    // without yielding — same reasoning wspr_bench gives for
    // deinit-ing the watchdog rather than measuring its console
    // traffic.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // `known: &[]` on every call, matching `decode_frame_impl`'s own
    // per-candidate loop exactly — it does not accumulate this pass's
    // own results into `known` as it goes (that parameter exists for
    // *cross-pass* dedup, e.g. SIC rounds feeding an earlier round's
    // decodes into a later one; `known` is threaded through
    // unchanged from the caller for the whole of a single pass).
    let t0 = now_us();
    let mut results = Vec::new();
    for rc in candidates {
        let precomputed_refine = (rc.cd0, rc.refined_freq_hz, rc.refined_i0, rc.refined_score);
        if let Some(res) = process_candidate_precomputed::<Fst4s60>(
            &rc.cand,
            // Empty, not `fft_cache` — see `run_bench`'s doc comment.
            // Safe: `precomputed_refine` + `skip_snr = true` together
            // mean this call chain never dereferences `fft_cache`.
            &[],
            &FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            precomputed_refine,
            // `skip_snr = true`: FST4's real-SNR formula needs a
            // *second*, independent `downsample_cached` call
            // (`fst4_raw_cs`) that `precomputed_refine` above doesn't
            // cover — the module doc explains why this bench can't
            // serve that FFT either yet (issue #307). `snr_db` on
            // every `DecodeResult` below is `NAN`, not a real
            // measurement; this bench answers the wall-clock question
            // only.
            true,
        ) {
            results.push(res);
        }
    }
    let total_us = now_us() - t0;

    log::info!(
        "fst4_bench: candidate loop (LLR/BP/OSD only, no FFT) TOTAL = {} ms  ({} decodes)",
        total_us / 1000,
        results.len(),
    );
    for r in &results {
        let text = r
            .message77()
            .try_into()
            .ok()
            .and_then(|m77: &[u8; 77]| unpack77(m77));
        log::info!(
            // snr_db is NAN (skip_snr = true, see module doc) — not
            // logged as a number so it can't be mistaken for one.
            "    {:?} | {:.1} Hz | dt {:.2} s | SNR not measured (skip_snr)",
            text,
            r.freq_hz,
            r.dt_sec,
        );
    }
    log::info!("fst4_bench: stack headroom after candidate loop = {} B", stack_headroom());
    log_heap("post-decode");
    log::info!("=== fst4_bench complete ===");
}

/// Stack for the bench task. Unmeasured starting point, not a
/// right-sized figure: `wspr_bench`'s own `SCAN_STACK` (72 KiB) was
/// only reached after finding a 128 KiB reservation and a stack
/// overflow inside `osd_decode_packed` on the way there. FST4 shares
/// the same generic LDPC/BP/OSD machinery FT8 already runs on-device
/// (`Ldpc240_101` vs FT8's `Ldpc174_91`, both `BpPooledFec`), so a
/// comparable order of magnitude is a reasonable starting guess — but
/// it is a guess. The first real run's `stack headroom after
/// candidate loop` log line is what turns this into a measured value;
/// resize once that number exists rather than trusting this one.
pub const BENCH_STACK: u32 = 96 * 1024;

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` leaks a `&'static &'static [u8]` that outlives
    // this task, mirroring wspr_bench's own arg-passing shape for
    // `extern "C"` task entries.
    let refined_bin: &'static [u8] = unsafe { *(arg as *const &'static [u8]) };
    run_bench(refined_bin);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// `refined_bin` is the baked golden asset (`include_bytes!`). Spawns
/// [`bench_task`] on a dedicated stack (see [`BENCH_STACK`]'s doc
/// comment on why that size is a starting guess) pinned to core 0,
/// then idles forever.
pub fn run(refined_bin: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let arg: &'static &'static [u8] = alloc::boxed::Box::leak(alloc::boxed::Box::new(refined_bin));
    let argp = arg as *const _ as *mut core::ffi::c_void;

    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"fst4_bench".as_ptr(),
            BENCH_STACK,
            argp,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create fst4_bench task ({BENCH_STACK} B stack)");
    }

    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
