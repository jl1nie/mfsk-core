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
//! `decode_block` or WSPR's now-four-rounds-tuned `decode_scan`.
//!
//! ## Follow-up, same day: per-candidate timing, and an OSD/BP split
//!
//! Added per-candidate wall-clock logging (collected into a `Vec`,
//! dumped *after* the timed loop so UART writes don't contaminate the
//! numbers) and a build-time `MFSK_FST4_BENCH_DEPTH=bp_only` switch
//! (`DecodeDepth::BP_ONLY` vs. the default `::FULL` — differ only in
//! `osd: bool`, since `LlrEffort` doesn't affect FST4 either way; see
//! that enum's own doc comment). Same shape as WSPR's own issue #260
//! controlled experiment (`confirmed = None`, short-circuiting before
//! `osd_decode`).
//!
//! **The 89.4 s is not spread across all 41 candidates — 94% of it is
//! 6 candidates, all failures:**
//!
//! | tier | count | each | total | decoded |
//! |---|---:|---:|---:|---|
//! | nsync-gate fails | 33 | 50-177 ms | ~6 s | no |
//! | **the tail** | **6** | **13.8-14.2 s** | **84.2 s (94%)** | **no** |
//! | real signals | 2 | 54-58 ms | 0.1 s | **yes** |
//!
//! Both real decodes are cheap — the entire cost is 6 candidates the
//! decoder ultimately rejects, matching WSPR's own shape (issue #260:
//! "OSD ran 896 times and succeeded 0 times... the cost is not where
//! any of us was looking").
//!
//! **OSD vs. plain-BP split on the same 6, `BP_ONLY` vs. `FULL`:**
//!
//! | | total | the 6-candidate tail (avg) |
//! |---|---:|---:|
//! | `FULL` (OSD on) | 89.411 s | 14.04 s |
//! | `BP_ONLY` (OSD off) | 51.755 s | 7.76 s |
//! | OSD's share | 37.66 s (42%) | 6.28 s |
//!
//! Both decodes still succeed with OSD off — on this file neither real
//! signal ever needed it. So OSD is a real cost (42% of the total) but
//! **not the majority one**: plain BP/LLR alone is 51.8 s, already
//! ~7.4× over the ~7 s budget by itself.
//!
//! ## Follow-up 2, same day: the `LLR_NSYM_MAX = 8` suspect confirmed
//!
//! `MFSK_FST4_BENCH_DEPTH=llr_probe` (see [`run_llr_stage_probe`])
//! calls `symbol_spectra` + each `compute_llr_*` stage directly, no
//! BP/OSD at all, timing every stage for every one of the 41
//! candidates (not just the tail — this mode skips the real
//! pipeline's early nsync-gate exit on purpose, to compare all four
//! stages against each other on equal footing).
//!
//! | stage | total (41 candidates) | share |
//! |---|---:|---:|
//! | `symbol_spectra` | 1.39 s | 0.5% |
//! | nsym=1 (`compute_llr_fast`) | 0.09 s | <0.1% |
//! | nsym=2 | 0.14 s | <0.1% |
//! | nsym=4 (`LLR_NSYM_MID`) | 1.10 s | 0.4% |
//! | **nsym=8 (`LLR_NSYM_MAX`)** | **301.2 s** | **99.1%** |
//!
//! `nsym=8` alone costs **~7.347 s per candidate**, uniform to within
//! 0.01% across all 41 — a pure function of the computation's size
//! (`4⁸ = 65536` tone-combination hypotheses per symbol group), not
//! data-dependent. That single stage accounts for essentially all of
//! the `BP_ONLY` run's ~7.76 s/candidate tail cost
//! (0.034+0.002+0.003+0.027+7.347 ≈ 7.41 s of it; the remaining
//! ~0.35 s is presumably `decode_soft_pooled`'s own BP iterations,
//! not separately measured here).
//!
//! One structural note this doesn't change: `SYNC_Q_MIN = 16` is
//! already FST4's own equivalent of WSPR's `minsync2` — WSJT-X's own
//! pre-ladder gate (`get_fst4_bitmetrics.f90`), faithfully ported
//! (issue #197), and it's what keeps 33 of 41 candidates from ever
//! reaching this stage in the real pipeline. The 8 that do clear it
//! are the same population a real `jt9` would also have to run this
//! computation on — there is no missing cheap-reject lever the way
//! WSPR's `minsync2` gap was; the cost is intrinsic to the candidates
//! that legitimately warrant deep decoding, not a filtering gap.
//!
//! So the remaining levers are in `compute_llr_partial` itself
//! (algorithmic restructuring, fixed-point, SIMD/PIE — unexamined
//! here), accepting a recall trade-off by skipping `nsym=8` on
//! embedded the way FT8's ship config skips OSD, or parallelism
//! (dual-core). Not pursued further in this session — see
//! `docs/reference/EMBEDDED.md` and issue #306 for where this leaves
//! the "does it fit" question.
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

use mfsk_core::engine::ModulationParams;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::llr::{compute_llr_fast, compute_llr_partial, symbol_spectra};
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

    // Third mode, alongside FULL/BP_ONLY above: does the ~7.76 s the
    // `BP_ONLY` run still pays per hard candidate concentrate in the
    // `LLR_NSYM_MAX = 8` LLR-computation rung specifically, as the
    // module doc's leading suspect predicts? Bypasses
    // `process_candidate_precomputed` (and therefore BP/OSD) entirely
    // — times only `symbol_spectra` + each `compute_llr_*` stage
    // directly, the same building blocks `process_candidate_basic_
    // impl`'s lazy staircase calls internally (`engine::pipeline.rs`
    // lines ~926-967).
    if option_env!("MFSK_FST4_BENCH_DEPTH") == Some("llr_probe") {
        log::info!("fst4_bench: LLR-stage probe (MFSK_FST4_BENCH_DEPTH=llr_probe) — no BP/OSD");
        run_llr_stage_probe(refined_bin);
        return;
    }

    // A/B switch for the issue #306 follow-up: is the per-candidate
    // cost tail (6 of 41 candidates, ~14 s each on the first
    // full-depth run) OSD, or the LLR staircase underneath it?
    // `DecodeDepth::BP_ONLY` differs from `::FULL` only in `osd:
    // false` — `llr_effort` doesn't matter for FST4 either way
    // (`LlrEffort` is FT8-only in practice; `process_candidate_
    // basic_impl` always computes every LLR variant regardless, see
    // that enum's own doc comment), so this isolates OSD's
    // contribution the same way WSPR's issue #260 controlled
    // experiment did (`confirmed = None`, short-circuiting before
    // `osd_decode`) — a build-time switch here rather than a runtime
    // one, same idiom as `wspr_bench`'s `MFSK_WSPR_BENCH_WIFI`.
    let depth = if option_env!("MFSK_FST4_BENCH_DEPTH") == Some("bp_only") {
        log::info!("fst4_bench: DecodeDepth::BP_ONLY (MFSK_FST4_BENCH_DEPTH=bp_only) — OSD off");
        DecodeDepth::BP_ONLY
    } else {
        log::info!("fst4_bench: DecodeDepth::FULL (default) — OSD on");
        DecodeDepth::FULL
    };
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
    // Per-candidate timing, not just the loop total — issue #306
    // follow-up: is the ~90 s roughly uniform across all 41 (something
    // structurally expensive on every candidate, e.g. the
    // `LLR_NSYM_MAX = 8` staircase rung's 65536-hypothesis group
    // running further than it should) or concentrated in the few that
    // reach OSD (WSPR's own shape, issue #260: pass 2 was 76% of that
    // scan for one decode)? Collected into a `Vec` and dumped *after*
    // the timed loop, not logged inline — UART writes inside the timed
    // region would contaminate the very numbers this is trying to
    // measure. `now_us()` itself is two syscalls per candidate,
    // negligible next to what each candidate actually costs here.
    struct CandidateTiming {
        freq_hz: f32,
        us: i64,
        decoded: bool,
    }
    let mut timings: Vec<CandidateTiming> = Vec::with_capacity(41);

    let t0 = now_us();
    let mut results = Vec::new();
    for rc in candidates {
        let freq_hz = rc.cand.freq_hz;
        let precomputed_refine = (rc.cd0, rc.refined_freq_hz, rc.refined_i0, rc.refined_score);
        let t_cand = now_us();
        let decoded = process_candidate_precomputed::<Fst4s60>(
            &rc.cand,
            // Empty, not `fft_cache` — see `run_bench`'s doc comment.
            // Safe: `precomputed_refine` + `skip_snr = true` together
            // mean this call chain never dereferences `fft_cache`.
            &[],
            &FST4_60A_DOWNSAMPLE,
            depth,
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
        );
        timings.push(CandidateTiming {
            freq_hz,
            us: now_us() - t_cand,
            decoded: decoded.is_some(),
        });
        if let Some(res) = decoded {
            results.push(res);
        }
    }
    let total_us = now_us() - t0;

    // Sorted slowest-first — the shape (uniform vs. a long tail) is
    // the point, not any single candidate's identity.
    timings.sort_by(|a, b| b.us.cmp(&a.us));
    log::info!("fst4_bench: per-candidate timing, slowest first:");
    for (rank, t) in timings.iter().enumerate() {
        log::info!(
            "    #{rank:>2} {:8.1} Hz  {:>7} ms  decoded={}",
            t.freq_hz,
            t.us / 1000,
            t.decoded,
        );
    }
    let mean_us: i64 = timings.iter().map(|t| t.us).sum::<i64>() / timings.len() as i64;
    let median_us = timings[timings.len() / 2].us;
    log::info!(
        "fst4_bench: per-candidate mean {} ms, median {} ms, max {} ms, min {} ms",
        mean_us / 1000,
        median_us / 1000,
        timings.first().map_or(0, |t| t.us) / 1000,
        timings.last().map_or(0, |t| t.us) / 1000,
    );

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

/// `MFSK_FST4_BENCH_DEPTH=llr_probe` mode — times `symbol_spectra` and
/// each `compute_llr_*` stage directly, per candidate, with no BP/OSD
/// call at all. Isolates whether the `BP_ONLY` run's ~7.76 s/candidate
/// tail cost is the `LLR_NSYM_MAX = 8` rung specifically (65536
/// tone-combination hypotheses per symbol group) or spread across the
/// staircase — see the module doc's "Follow-up" section for why this
/// is the next thing worth separating out.
///
/// Mirrors `process_candidate_basic_impl`'s own lazy-staircase call
/// order exactly (`engine::pipeline.rs`, same file/line range as the
/// module doc above): `compute_llr_fast` (nsym=1, gives `llra`+`llrd`
/// together, ~5× faster than computing them separately per that
/// function's own doc comment), then `compute_llr_partial` at nsym=2,
/// `LLR_NSYM_MID` (4), and `LLR_NSYM_MAX` (8) in that order — but
/// unconditionally for every candidate here, not lazily stopping at
/// the first variant that would let BP converge, since this probe
/// never calls BP at all and the point is comparing all four stages'
/// costs against each other.
fn run_llr_stage_probe(refined_bin: &[u8]) {
    log_heap("boot");
    let candidates = load_refined_candidates(refined_bin);
    log::info!("fst4_bench: llr_probe over {} candidates", candidates.len());
    log_heap("post-load");

    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    struct StageTiming {
        freq_hz: f32,
        symspec_us: i64,
        fast_us: i64,
        nsym2_us: i64,
        nsym_mid_us: i64,
        nsym_max_us: i64,
    }
    let mut timings: Vec<StageTiming> = Vec::with_capacity(candidates.len());

    let nsym_mid = <Fst4s60 as ModulationParams>::LLR_NSYM_MID
        .expect("FST4-60A sets LLR_NSYM_MID") as usize;
    let nsym_max = <Fst4s60 as ModulationParams>::LLR_NSYM_MAX as usize;

    let t0 = now_us();
    for rc in &candidates {
        let t = now_us();
        let cs = symbol_spectra::<Fst4s60>(&rc.cd0, rc.refined_i0);
        let symspec_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_fast::<Fst4s60, f32>(&cs));
        let fast_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_partial::<Fst4s60, f32, f32>(&cs, 2));
        let nsym2_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_partial::<Fst4s60, f32, f32>(&cs, nsym_mid));
        let nsym_mid_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_partial::<Fst4s60, f32, f32>(&cs, nsym_max));
        let nsym_max_us = now_us() - t;

        timings.push(StageTiming {
            freq_hz: rc.cand.freq_hz,
            symspec_us,
            fast_us,
            nsym2_us,
            nsym_mid_us,
            nsym_max_us,
        });
    }
    let total_us = now_us() - t0;

    timings.sort_by(|a, b| b.nsym_max_us.cmp(&a.nsym_max_us));
    log::info!("fst4_bench: llr_probe per-candidate stage timing, sorted by nsym=8 cost:");
    for t in &timings {
        log::info!(
            "    {:8.1} Hz  symspec {:>5} us  nsym1(fast) {:>6} us  nsym2 {:>6} us  nsym{} {:>6} us  nsym{} {:>7} us",
            t.freq_hz,
            t.symspec_us,
            t.fast_us,
            t.nsym2_us,
            nsym_mid,
            t.nsym_mid_us,
            nsym_max,
            t.nsym_max_us,
        );
    }
    let sum = |f: fn(&StageTiming) -> i64| -> i64 { timings.iter().map(f).sum() };
    let (s_symspec, s_fast, s_2, s_mid, s_max) = (
        sum(|t| t.symspec_us),
        sum(|t| t.fast_us),
        sum(|t| t.nsym2_us),
        sum(|t| t.nsym_mid_us),
        sum(|t| t.nsym_max_us),
    );
    log::info!(
        "fst4_bench: llr_probe totals over {} candidates — symspec {} ms, nsym1 {} ms, nsym2 {} ms, nsym{} {} ms, nsym{} {} ms, stage-sum {} ms, wall {} ms",
        timings.len(),
        s_symspec / 1000,
        s_fast / 1000,
        s_2 / 1000,
        nsym_mid,
        s_mid / 1000,
        nsym_max,
        s_max / 1000,
        (s_symspec + s_fast + s_2 + s_mid + s_max) / 1000,
        total_us / 1000,
    );
    log::info!("fst4_bench: stack headroom after llr_probe = {} B", stack_headroom());
    log_heap("post-probe");
    log::info!("=== fst4_bench (llr_probe) complete ===");
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
