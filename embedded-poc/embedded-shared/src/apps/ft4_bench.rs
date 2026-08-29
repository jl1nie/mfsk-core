//! FT4 per-candidate bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! **The question**: FT4 has never been built for, let alone run on,
//! any board in this tree — `docs/reference/EMBEDDED.md`'s per-protocol
//! table said "no embedded path at all yet", and no embedded crate
//! enabled `mfsk-core/ft4`. Does FT4's per-candidate work fit the
//! ~1.96 s a 7.5 s slot leaves after the signal ends?
//!
//! ## The budget
//!
//! FT4's slot is 7.5 s; transmission starts at `TX_START_OFFSET_S =
//! 0.5` and runs `NN2 = 105` symbols × 48 ms = 5.04 s, so the last
//! sample of the frame lands at 5.54 s. [`DECODE_BUDGET_MS`] is what is
//! left. Unlike FST4's and WSPR's monitor loops — built with deliberate
//! slack, where exceeding the slot is a *fault* — this is the same kind
//! of budget FT8's 15 s slot has: genuinely tight, and an overrun is an
//! operating limit rather than a bug. See `m5stack-cores3-app/CLAUDE.md`
//! "Slot-budget logs mean opposite things per mode".
//!
//! ## What runs here, and what is baked
//!
//! Baked on the host by `mfsk-core/tests/ft4_wsjtx_samples.rs::
//! ft4_bake_golden_precomputed`:
//!
//! - **the wideband FFT cache** (`fft1_size = 92_160`). Not a power of
//!   two, and 11× past `CONFIG_DSP_MAX_FFT_SIZE_8192` — ESP-DSP cannot
//!   serve it at all. Fed back through `decode_frame`'s
//!   `precomputed_fft` seam, exactly as FST4 does (issue #306).
//! - **the coarse-candidate list**, so `ft4_coarse_sync`'s own `NFFT1 =
//!   2304` (= 256 × 9) periodogram doesn't need a kernel that doesn't
//!   exist yet either. That stage measured **0.3 ms on host** over the
//!   whole slot, so its absence understates the device total by roughly
//!   that times whatever device/host ratio the rest of this bench
//!   reports — small, but stated rather than left to be assumed.
//!
//! Everything else is the production code path, unmodified:
//!
//! - `downsample_cached`'s inverse transform (`fft2_size = 5120`),
//!   served by `engine::dsp::fft_mixed_5120` (1024 × 5, added for this)
//! - `engine::sync2d::ft4_sync_search` — FT4's coherent absolute-Δt
//!   search, no FFT at all, and by far the dominant per-candidate cost
//!   on host (`ft4_diag_candidate_cost_split`: ~123 ms of ~150 ms over
//!   50 candidates). Measuring it is the point of this bench.
//! - `engine::llr::symbol_spectra`'s per-symbol DFT: `ds_spb =
//!   NSPS/NDOWN = 32`, a power of two, so no new kernel needed.
//!
//! ## Host reference for the same 31 candidates
//!
//! Measured 2026-08-29 by the bake test itself, on the WSJT-X golden
//! `000000_000002.wav`, single pass, this exact search:
//!
//! - **11 distinct decodes**, *identical* under `DecodeDepth::EMBEDDED`
//!   and `DecodeDepth::FULL` — OSD and full LLR effort buy nothing on
//!   this file, so the ship config gives up no recall here.
//! - (The 14/14 golden the host recall test asserts needs
//!   `.sic_rounds(3)`; embedded FT8 ships a single pass, so a
//!   multi-pass figure would not describe what a board would run.)
//! - Whole-slot host wall-clock, rayon across 24 threads: ~100-105 ms
//!   (`docs/notes/BENCHMARKS.md`). Single-threaded is the honest
//!   comparison for an MCU projection — `parallel` is always off on
//!   embedded.
//!
//! ## Three passes, so the total has a split
//!
//! Timing only `process_candidate_basic` gives one number and no way to
//! act on it. This runs three passes over the same candidate list and
//! reports each, the same "inferred by subtraction" framing the host
//! diagnostic `ft4_diag_candidate_cost_split` already uses:
//!
//! | pass | what it calls | what it isolates |
//! |---|---|---|
//! | 1 | `downsample_cached` | the 5120-pt inverse FFT |
//! | 2 | pass 1 + RMS-norm + `ft4_sync_search` | the Δt search |
//! | 3 | `process_candidate_basic` (production) | everything |
//!
//! LLR/BP/OSD is then `pass3 − pass2`. Passes 1 and 2 repeat work pass
//! 3 also does — that is what makes them a *split* rather than a
//! breakdown, and the total to quote is pass 3 alone.

extern crate alloc;

use alloc::vec::Vec;

use num_complex::Complex32;
// `f32::sqrt` in `rms_normalise` resolves without this in every
// feature combination this crate builds under today (esp-idf-svc
// drags std into the graph), same as `fst4_monitor`'s identical
// call site. Kept explicit anyway — see `esp_dsp_fft`'s own copy of
// this import and its reasoning.
#[allow(unused_imports)]
use num_traits::Float;

use mfsk_core::engine::dsp::downsample::downsample_cached;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::engine::sync2d::ft4_sync_search_window;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

/// Mirrors `ft4::decode`'s own (private) `SYNC_Q_MIN`: FT4 has 16 sync
/// symbols (4 × Costas-4) and requires at least half correct. Same
/// trade `fst4_bench` makes with its own copy of that constant — if the
/// crate-side value moves, this bench silently stops matching the
/// shipped gate rather than failing to compile.
///
/// Kept in step with `ft4_wsjtx_samples.rs`'s `bench_assets` module,
/// which is what the baked assets were generated against.
const SYNC_Q_MIN: u32 = 8;

/// Milliseconds between the end of an FT4 frame and the end of its
/// slot: `7.5 s − (0.5 s TX offset + 105 × 48 ms)` = 1.96 s.
///
/// Same role as `fst4_ddc_bench`'s `TX_TURNAROUND_BUDGET_MS`, and the
/// number this bench exists to compare against.
const DECODE_BUDGET_MS: i64 = 1_960;

const MALLOC_CAP_8BIT: u32 = 1 << 2;
const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Bytes of the calling task's stack never touched so far. Same
/// technique — and the same duplicate-rather-than-share reasoning —
/// as `fst4_bench::stack_headroom`.
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

/// `FT4_DOWNSAMPLE.fft2_size` — one candidate's downsampled baseband.
const CD0_LEN: usize = 5_120;
const CD0_BYTES: usize = CD0_LEN * core::mem::size_of::<Complex32>();

/// One 40 KB `cd0` buffer in **internal** DRAM, for the placement
/// comparison this bench exists to make.
///
/// `downsample_cached` returns a plain `Vec`, and with
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` a 40 KB allocation
/// lands in PSRAM. `ft4_sync_search_window` then streams 512
/// `Complex32` per grid cell across ~3159 cells — about 13 MB of reads
/// per candidate, all of it over the PSRAM bus. `internal_pool`'s doc
/// comment records ~5-10x for moving exactly this kind of hot buffer
/// into internal DRAM on FT8's `cs` scratch; whether it holds here is
/// what pass 2i measures.
///
/// 16-byte aligned so `dot_f32`'s esp-dsp backend can take its PIE
/// path where the sub-slice offset also lands even
/// (`dsps_dotprod_f32_aes3` needs 16-byte alignment and `len % 4 == 0`,
/// and falls back to the scalar body otherwise — see
/// `mfsk_core::engine::dsp::dotprod`). Half the `i0` positions will
/// still be odd multiples of 8 bytes; the base alignment is what can
/// be controlled from here.
///
/// Leaked deliberately: it lives for the whole run, and this is a
/// single-shot bench.
fn alloc_internal_cd0() -> Option<&'static mut [Complex32]> {
    const MALLOC_CAP_INTERNAL_8BIT: u32 = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    // SAFETY: `heap_caps_aligned_alloc` returns either null or a
    // 16-byte-aligned block of at least `CD0_BYTES`; `Complex32` is
    // `repr(C)` over two `f32` and needs 4-byte alignment, so the
    // block is validly typed and long enough for `CD0_LEN` of them.
    unsafe {
        let p = esp_idf_svc::sys::heap_caps_aligned_alloc(16, CD0_BYTES, MALLOC_CAP_INTERNAL_8BIT)
            as *mut Complex32;
        if p.is_null() {
            None
        } else {
            Some(core::slice::from_raw_parts_mut(p, CD0_LEN))
        }
    }
}

/// Parse `ft4_golden_fft_cache.bin`: `(f32 re, f32 im) × fft1_size`, LE.
fn load_fft_cache(bin: &[u8]) -> Vec<Complex32> {
    let n = FT4_DOWNSAMPLE.fft1_size;
    assert_eq!(
        bin.len(),
        n * 8,
        "fft_cache asset is {} B, expected {} B — re-run the bake test",
        bin.len(),
        n * 8
    );
    let mut out = Vec::with_capacity(n);
    for b in bin.chunks_exact(8) {
        out.push(Complex32::new(
            f32::from_le_bytes(b[0..4].try_into().unwrap()),
            f32::from_le_bytes(b[4..8].try_into().unwrap()),
        ));
    }
    out
}

/// Parse `ft4_golden_candidates.bin`: `u32 n`, then `n × (f32 freq_hz,
/// f32 dt_sec, f32 score)`, LE.
fn load_candidates(bin: &[u8]) -> Vec<SyncCandidate> {
    let n = u32::from_le_bytes(bin[0..4].try_into().unwrap()) as usize;
    assert_eq!(bin.len(), 4 + n * 12, "candidate asset length mismatch");
    bin[4..]
        .chunks_exact(12)
        .take(n)
        .map(|b| SyncCandidate {
            freq_hz: f32::from_le_bytes(b[0..4].try_into().unwrap()),
            dt_sec: f32::from_le_bytes(b[4..8].try_into().unwrap()),
            score: f32::from_le_bytes(b[8..12].try_into().unwrap()),
        })
        .collect()
}

/// RMS-normalise a downsampled baseband to unit power, matching what
/// `process_candidate_basic_impl` does to its own `downsample_cached`
/// output (WSJT-X `ft4_decode.f90:231-232`). Pass 2 needs it because
/// `ft4_sync_search`'s score is scale-dependent; without it that pass
/// would measure the same arithmetic on differently-scaled data.
fn rms_normalise(cd0: &mut [Complex32]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

fn run_bench(fft_cache_bin: &[u8], cand_bin: &[u8]) {
    log_heap("boot");

    // Take the internal-DRAM search buffer first, while the heap is
    // whole — the same ordering discipline `worker_arena` documents.
    // 40 KB is comfortable here (this bench brings up no WiFi and no
    // USB host, and boot reports ~156 KB largest contiguous), but it
    // would *not* be in the app: with WiFi up the largest free
    // internal block on this board is 31 744 B, so a production FT4
    // mode would have to reserve this through `worker_arena` at boot
    // rather than allocate it here.
    let internal_cd0: Option<&'static mut [Complex32]> = alloc_internal_cd0();
    match &internal_cd0 {
        Some(b) => log::info!(
            "ft4_bench: internal cd0 buffer {} B at {:p} ({}16-byte aligned)",
            CD0_BYTES,
            b.as_ptr(),
            if (b.as_ptr() as usize) % 16 == 0 { "" } else { "NOT " }
        ),
        None => log::warn!(
            "ft4_bench: internal cd0 allocation failed — the placement passes will be skipped"
        ),
    }

    let t_load = now_us();
    let fft_cache = load_fft_cache(fft_cache_bin);
    let candidates = load_candidates(cand_bin);
    log::info!(
        "ft4_bench: loaded {} FFT bins + {} candidates in {} ms",
        fft_cache.len(),
        candidates.len(),
        (now_us() - t_load) / 1000,
    );
    log_heap("post-load");

    // Every pass below runs compute-bound for tens of seconds without
    // yielding, so the task watchdog fires repeatedly and dumps a
    // backtrace each time. Same reasoning — and the same call —
    // `fst4_bench` and `wspr_bench` use: deinit it rather than measure
    // its console traffic. (First run, 2026-08-29, before this line
    // existed: 7 triggers across the three passes.)
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("ft4_bench: task watchdog deinit -> {r}");

    let n = candidates.len() as i64;

    // ── Pass 1: downsample only ──────────────────────────────────────
    let t0 = now_us();
    let mut checksum = 0.0f32;
    for cand in &candidates {
        let cd0 = downsample_cached(&fft_cache, cand.freq_hz, &FT4_DOWNSAMPLE);
        // Consume the result so the loop can't be optimised away.
        checksum += cd0[0].re;
    }
    let downsample_us = now_us() - t0;
    log::info!(
        "ft4_bench: pass1 downsample_cached = {} ms ({} us/cand, checksum {checksum:e})",
        downsample_us / 1000,
        downsample_us / n,
    );

    // ── Passes 2*: the Δt search, one variable at a time ─────────────
    //
    // `search_pass` prepares each candidate's cd0 exactly as
    // `process_candidate_basic_impl` does (downsample + RMS-normalise),
    // optionally copies it into internal DRAM, and times
    // `ft4_sync_search_window` alone. The copy is counted, so the
    // internal-DRAM rows are honest about what they cost as well as
    // what they save.
    let mut search_pass = |label: &str, ib: (i32, i32), internal: Option<&mut [Complex32]>| {
        let mut dst = internal;
        let mut per_cand: Vec<i64> = Vec::with_capacity(candidates.len());
        let mut i0_sum = 0i64;
        let t0 = now_us();
        for cand in &candidates {
            let mut cd0 = downsample_cached(&fft_cache, cand.freq_hz, &FT4_DOWNSAMPLE);
            rms_normalise(&mut cd0);
            let t_s = now_us();
            let r = match dst.as_deref_mut() {
                Some(buf) => {
                    buf.copy_from_slice(&cd0);
                    ft4_sync_search_window::<Ft4>(buf, cand, ib.0, ib.1)
                }
                None => ft4_sync_search_window::<Ft4>(&cd0, cand, ib.0, ib.1),
            };
            per_cand.push(now_us() - t_s);
            i0_sum += r.i0 as i64;
        }
        let total_us = now_us() - t0;
        let search_us: i64 = per_cand.iter().sum();
        per_cand.sort_unstable();
        log::info!(
            "ft4_bench: {label} search {} ms ({} us/cand; min {} p50 {} max {})              | +downsample = {} ms | i0_sum {i0_sum}",
            search_us / 1000,
            search_us / n,
            per_cand[0],
            per_cand[per_cand.len() / 2],
            per_cand[per_cand.len() - 1],
            total_us / 1000,
        );
        search_us
    };

    const FULL_WINDOW: (i32, i32) = (-344, 1012);
    // ±0.5 s — the largest width measured lossless on both the real
    // off-air golden and an ft4sim DT sweep
    // (`docs/notes/FT4_BENCHMARK.md` §18).
    const NARROW_WINDOW: (i32, i32) = (0, 667);

    let psram_full = search_pass("pass2  [PSRAM cd0,    +/-1.0s]", FULL_WINDOW, None);
    let (internal_full, internal_narrow) = match internal_cd0 {
        Some(buf) => {
            let a = search_pass("pass2i [internal cd0, +/-1.0s]", FULL_WINDOW, Some(buf));
            let b = search_pass("pass2n [internal cd0, +/-0.5s]", NARROW_WINDOW, Some(buf));
            (Some(a), Some(b))
        }
        None => (None, None),
    };
    if let (Some(f), Some(nw)) = (internal_full, internal_narrow) {
        log::info!(
            "ft4_bench: placement {}.{:02}x | narrowing {}.{:02}x | combined {}.{:02}x",
            psram_full / f,
            (psram_full * 100 / f) % 100,
            f / nw,
            (f * 100 / nw) % 100,
            psram_full / nw,
            (psram_full * 100 / nw) % 100,
        );
    }
    log_heap("post-search");

    // ── Pass 3: the production per-candidate call ────────────────────
    //
    // `process_candidate_basic` builds its own cd0 internally, so this
    // pass cannot use the internal-DRAM buffer — it is the unmodified
    // production path, kept for the slot total and for the decode
    // check. Project the placement/window wins onto it using the
    // ratios above rather than reading them into this number.
    for (label, depth) in [
        ("EMBEDDED", DecodeDepth::EMBEDDED),
        ("FULL", DecodeDepth::FULL),
    ] {
        let t0 = now_us();
        let mut msgs: Vec<alloc::string::String> = Vec::new();
        for cand in &candidates {
            // `known = &[]` is what `decode_frame_impl`'s own FT4 arm
            // passes — FT4 decodes raw candidates and dedups afterwards.
            // Passing accumulated results here would make the bench
            // cheaper than the code it is measuring.
            if let Some(r) = process_candidate_basic::<Ft4>(
                cand,
                &fft_cache,
                &FT4_DOWNSAMPLE,
                depth,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
                SYNC_Q_MIN,
            ) {
                if let Some(text) = r
                    .message77()
                    .try_into()
                    .ok()
                    .and_then(|m77: &[u8; 77]| unpack77(m77))
                {
                    log::info!("    {text} | {:.1} Hz | dt {:.2} s", r.freq_hz, r.dt_sec);
                    if !msgs.contains(&text) {
                        msgs.push(text);
                    }
                }
            }
        }
        let total_us = now_us() - t0;
        let total_ms = total_us / 1000;
        log::info!(
            "ft4_bench: pass3[{label}] TOTAL = {} ms ({} us/cand) | {} distinct decodes \
             (host reference for this asset: 11) | llr+bp+osd inferred ~{} ms",
            total_ms,
            total_us / n,
            msgs.len(),
            (total_us - psram_full - downsample_us) / 1000,
        );
        log::info!(
            "ft4_bench: pass3[{label}] budget {DECODE_BUDGET_MS} ms -> {}",
            if total_ms <= DECODE_BUDGET_MS {
                "FITS"
            } else {
                "over"
            },
        );
        log_heap(&alloc::format!("post-pass3-{label}"));
    }

    log::info!(
        "ft4_bench: stack headroom {} B of {} B",
        stack_headroom(),
        BENCH_STACK,
    );
    log::info!(
        "ft4_bench: NOTE — ft4_coarse_sync (NFFT1=2304) is baked, not run here; \
         it is 0.3 ms of the host slot, so the totals above are the per-candidate \
         work only, not a whole slot."
    );
}


/// Installs the `EspLogger` exactly once. Must be used instead of
/// `EspLogger::initialize_default` — a second install aborts.
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

/// Starting guess, same as `fst4_bench`'s — which measured only ~3.2
/// KiB actually used out of 96 KiB. The reported headroom line at the
/// end of [`run_bench`] is what right-sizes this.
pub const BENCH_STACK: u32 = 96 * 1024;

struct BenchArgs {
    fft_cache: &'static [u8],
    candidates: &'static [u8],
}

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` leaks the `BenchArgs` so it outlives this task —
    // same arg-passing shape as `fst4_bench`/`wspr_bench`.
    let args: &'static BenchArgs = unsafe { &*(arg as *const BenchArgs) };
    run_bench(args.fft_cache, args.candidates);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// Both arguments are the baked golden assets (`include_bytes!`).
/// Spawns [`bench_task`] on a dedicated stack pinned to core 0, then
/// idles forever.
pub fn run(fft_cache: &'static [u8], candidates: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let args: &'static BenchArgs = alloc::boxed::Box::leak(alloc::boxed::Box::new(BenchArgs {
        fft_cache,
        candidates,
    }));
    let argp = args as *const _ as *mut core::ffi::c_void;

    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"ft4_bench".as_ptr(),
            BENCH_STACK,
            argp,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create ft4_bench task ({BENCH_STACK} B stack)");
    }

    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
