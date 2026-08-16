//! FST4-60 decoder-only bench for ESP32-S3 (LX7) — issue #306.
//!
//! Answers the question issue #306 actually asked: does the *decoder*
//! — everything past the wideband FFT that produces a candidate list
//! and then LLR/BP/OSD each one — fit inside FST4-60's ~7 s post-slot
//! margin on real CoreS3 hardware? Not "is the whole embedded FST4
//! port done" (it isn't attempted here) and not "is the WSPR result
//! predictive of this" (issue #260's own closing thread argued the
//! opposite: WSPR's dominant cost was Fano-sequential search burning a
//! full node budget per failing candidate, a failure mode FST4's
//! bounded LDPC/BP + OSD-fallback doesn't share).
//!
//! ## Why there is no on-device wideband stage here
//!
//! Same shape as `wspr_bench`'s baked baseband: FST4-60A's
//! `build_fft_cache` runs one `fft1_size = 746_496`-point forward FFT
//! over the whole 60 s slot, and `esp-dsp`'s FFT backend has an 8 192
//! ceiling (`CONFIG_DSP_MAX_FFT_SIZE_8192`) — nowhere close. There is
//! no on-device path from raw audio to that cache today, and building
//! one is out of scope here (issue #306 explicitly doesn't ask for
//! it: "I'm not suggesting a full embedded FST4 port... A
//! decoder-only benchmark would be enough").
//!
//! So the cache is baked on a host by the `#[ignore]`d
//! `fst4_bake_golden_precomputed` in `mfsk-core/tests/fst4_wsjtx_samples.rs`
//! (from the WSJT-X golden `210115_0058.wav`) and fed in via
//! `engine::pipeline::decode_frame`'s `precomputed_fft` parameter —
//! the same seam FT8's own `decode_frame_inner` already uses. The bake
//! test round-trips both baked files back through `decode_frame` on
//! the host and asserts the result matches `DecodeRequest`'s own
//! fresh-computed path exactly, so what ships here is provably what
//! the unbaked path would have produced, not an approximation of it.
//!
//! `coarse_sync` — the candidate search — is a different story:
//! `compute_spectra` is a per-symbol-block spectrogram, not the big
//! FFT, and runs on raw `audio` directly (same as FT8's own coarse
//! sync). It is timed both standalone (below) and again as part of
//! the full `decode_frame` call — the second run repeats the first
//! rather than reusing it, which duplicates compute but keeps each
//! number an honest measurement of its own real path rather than a
//! hand-optimised composite.
//!
//! ## What this does *not* attempt
//!
//! No PSRAM-vs-SRAM bandwidth arms, no dual-core split, no WiFi, no
//! spot reporting, no steady-state multi-slot pipeline. `wspr_bench`
//! grew all of that from real on-device findings over several rounds
//! (issue #260) — building the same scaffolding here ahead of a first
//! real measurement would be guessing at what FST4 actually needs.
//! This is deliberately the single-shot, single-core, single-arm
//! shape wspr_bench started as.
//!
//! ## Measured on hardware (2026-08-16): PSRAM footprint was fine,
//! the FFT backend is the actual blocker
//!
//! The two baked assets are 1.44 MiB (audio, `i16`) + 5.7 MiB
//! (`fft_cache`, `Complex32`) = ~7.16 MiB, decoded into a second
//! in-RAM copy of each (the baked bytes are 1-byte-aligned
//! `include_bytes!` data — Xtensa faults on an unaligned `f32`/`i16`
//! load, so they're parsed byte-wise into fresh `Vec`s rather than
//! transmuted in place). That was the risk flagged before the first
//! real run; it did not materialize — CoreS3 logged 893 KiB PSRAM
//! free / 880 KiB largest-contiguous immediately after both `Vec`s
//! were built, comfortably inside the 8 MiB part.
//!
//! What actually stops this bench: `coarse_sync::<Fst4s60>`'s first
//! FFT call panics —
//! `esp-dsp FFT requires power-of-2 length ≥ 4 (got 7776)`. `7776 =
//! NSPS(3888) × NFFT_PER_SYMBOL_FACTOR(2)` (`engine::sync::SyncDims`)
//! is FST4-60A's coarse-sync spectrogram FFT length, and `2⁵ × 3⁵` is
//! not a power of two — the same shape as FT8's own `1920 × 2 = 3840`,
//! which is why FT8 has a hand-rolled `MixedRadix3840Fft` in
//! `esp_dsp_fft` instead of relying on the ESP-DSP backend's
//! power-of-two-only radix-2 kernel. FST4 has no equivalent yet, and
//! `downsample_cached`'s inverse FFT (`fft2_size = 6912 = 2⁸ × 27`,
//! also non-power-of-two) hasn't even been reached — `coarse_sync`
//! panics first. See `docs/reference/EMBEDDED.md`'s "FST4 on
//! embedded" section for the full writeup, including the two
//! unrelated infrastructure bugs (a stale-4MB `espflash` flash-size
//! default, and the `partitions.csv` growth this bench needed) that
//! had to be fixed before this measurement was even reachable.

extern crate alloc;

use alloc::vec::Vec;

use num_complex::Complex32;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, decode_frame};
use mfsk_core::engine::sync::coarse_sync;
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

/// Search band + candidate cap, matching
/// `fst4_wsjtx_sample_precision_vs_reference_decoder`'s own params so
/// the device number is comparable to the host golden test's.
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 3000.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CANDIDATES: usize = 50;

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

/// `bin` is the baked audio asset (`include_bytes!`) — raw `i16` LE,
/// no header. Byte-wise, not a transmute: `include_bytes!` gives
/// 1-byte alignment and Xtensa faults on an unaligned `i16` load.
fn load_audio(bin: &[u8]) -> Vec<i16> {
    assert_eq!(bin.len() % 2, 0, "baked audio has an odd byte length");
    bin.chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

/// `bin` is the baked FFT-cache asset (`include_bytes!`) — `(f32 re,
/// f32 im)` pairs, LE, no header. Same byte-wise reasoning as
/// [`load_audio`].
fn load_fft_cache(bin: &[u8]) -> Vec<Complex32> {
    assert_eq!(bin.len() % 8, 0, "baked fft_cache has the wrong byte length");
    bin.chunks_exact(8)
        .map(|b| {
            Complex32::new(
                f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
                f32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            )
        })
        .collect()
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

/// One `decode_frame::<Fst4s60>` call over the baked golden, run on
/// the calling task/stack (`main`'s, or whatever the caller already
/// spawned it on). Logs a standalone `coarse_sync` timing, then the
/// full decode, then the decodes themselves and stack/heap headroom.
///
/// `audio_bin`/`fft_cache_bin` are the two `include_bytes!` blobs
/// produced by `fst4_bake_golden_precomputed`.
pub fn run_bench(audio_bin: &[u8], fft_cache_bin: &[u8]) {
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!(
        "fst4_bench: baked assets audio={} bytes, fft_cache={} bytes",
        audio_bin.len(),
        fft_cache_bin.len(),
    );
    log_heap("boot");

    let t_load = now_us();
    let audio = load_audio(audio_bin);
    let fft_cache = load_fft_cache(fft_cache_bin);
    log::info!(
        "fst4_bench: loaded {} audio samples + {} fft_cache bins in {} ms",
        audio.len(),
        fft_cache.len(),
        (now_us() - t_load) / 1000,
    );
    log_heap("post-load");

    // The candidate loop runs compute-bound for however long
    // decode_frame takes without yielding — same reasoning wspr_bench
    // gives for deinit-ing the watchdog rather than measuring its
    // console traffic.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // Standalone coarse_sync timing — the candidate search alone,
    // separate from (and re-run inside) the full decode below. See
    // the module doc for why this duplicates work rather than reusing
    // the candidate list.
    let t_cs = now_us();
    let candidates = coarse_sync::<Fst4s60>(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CANDIDATES);
    let coarse_us = now_us() - t_cs;
    log::info!(
        "fst4_bench: coarse_sync alone = {} ms ({} candidates)",
        coarse_us / 1000,
        candidates.len(),
    );
    log::info!("fst4_bench: stack headroom after coarse_sync = {} B", stack_headroom());

    let t0 = now_us();
    let (results, _cache) = decode_frame::<Fst4s60>(
        &audio,
        &FST4_60A_DOWNSAMPLE,
        FREQ_MIN_HZ,
        FREQ_MAX_HZ,
        SYNC_MIN,
        None,
        DecodeDepth::FULL,
        MAX_CANDIDATES,
        DecodeStrictness::Normal,
        EqMode::Off,
        SYNC_Q_MIN,
        Some(&fft_cache),
        None,
    );
    let total_us = now_us() - t0;

    log::info!(
        "fst4_bench: decode_frame TOTAL = {} ms  ({} decodes)  [coarse_sync-alone was {} ms of that shape]",
        total_us / 1000,
        results.len(),
        coarse_us / 1000,
    );
    for r in &results {
        let text = r
            .message77()
            .try_into()
            .ok()
            .and_then(|m77: &[u8; 77]| unpack77(m77));
        log::info!(
            "    {:?} | {:.1} Hz | dt {:.2} s | {} dB",
            text,
            r.freq_hz,
            r.dt_sec,
            r.snr_db as i32,
        );
    }
    log::info!("fst4_bench: stack headroom after decode_frame = {} B", stack_headroom());
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
/// decode_frame` log line is what turns this into a measured value;
/// resize once that number exists rather than trusting this one.
pub const BENCH_STACK: u32 = 96 * 1024;

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` leaks a `&'static (&'static [u8], &'static [u8])`
    // that outlives this task, mirroring wspr_bench's own arg-passing
    // shape for `extern "C"` task entries.
    let (audio_bin, fft_cache_bin): (&'static [u8], &'static [u8]) =
        unsafe { *(arg as *const (&'static [u8], &'static [u8])) };
    run_bench(audio_bin, fft_cache_bin);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// `audio_bin`/`fft_cache_bin` are the two baked golden assets
/// (`include_bytes!`). Spawns [`bench_task`] on a dedicated stack
/// (see [`BENCH_STACK`]'s doc comment on why that size is a starting
/// guess) pinned to core 0, then idles forever.
pub fn run(audio_bin: &'static [u8], fft_cache_bin: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let arg: &'static (&'static [u8], &'static [u8]) =
        alloc::boxed::Box::leak(alloc::boxed::Box::new((audio_bin, fft_cache_bin)));
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
