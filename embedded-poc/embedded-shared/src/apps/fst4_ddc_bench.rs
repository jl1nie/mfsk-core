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


use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{process_candidate_precomputed, DecodeDepth, DecodeStrictness};
use mfsk_core::engine::sync::{
    coarse_sync, compute_spectra, AudioSource, SyncCandidate, SyncDims,
};
use mfsk_core::fst4::ddc::grid_for;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::msg::wsjt77::unpack77;

use crate::fst4_dual_core;
use crate::fst4_monitor::{self, Band, CapturedSlot, MonitorConfig, SlotCapture, now_us};

/// Matches `fst4::decode`'s own (private) `SYNC_Q_MIN` — see
/// `fst4_bench`'s identical constant for why it's redeclared here
/// rather than imported.
const SYNC_Q_MIN: u32 = 16;

/// `MFSK_FST4_DDC_BENCH_BAND=wide` switches this bench from the
/// known-target sniper window to the full 100-3000 Hz production search
/// band — same `option_env!` build-time idiom as `fst4_bench`'s
/// `MFSK_FST4_BENCH_DEPTH`.
///
/// The two are genuinely different questions, not a width knob:
///
/// | | sniper (default) | `=wide` |
/// |---|---|---|
/// | cascade | `sniper_cascade` (49-tap /3 + L/M 16/81) | `wideband_cascade` (L/M 64/243, no integer stage) |
/// | `K` / `nfft1` | 256 / 512 | 1024 / 2048 |
/// | `sync_min` | 8.0 (strong-signal demo, see below) | **0.8, the production value** |
/// | what it answers | "does the DDC chain work end-to-end on device" | "does a real wideband search fit the slot budget" |
///
/// `wideband_cascade`/`wideband_refine_recenter` shipped with the DDC
/// stages 1-4 work but had **no non-test caller** until this switch —
/// they were unit-tested against the design doc's own K/Fs_c/nfft1
/// table and a single-tone smoke test, never measured on hardware.
const WIDEBAND: bool = is_wide(option_env!("MFSK_FST4_DDC_BENCH_BAND"));

/// `option_env!(..) == Some("wide")` doesn't compile in a `const`
/// initialiser (`str` equality isn't const-stable), which is why
/// `fst4_bench`'s equivalent switch reads its env var inside a function
/// instead. Here the value has to be a `const` — it selects the other
/// `const`s below — so the comparison is spelled out bytewise.
const fn is_wide(v: Option<&str>) -> bool {
    match v {
        Some(s) => {
            let b = s.as_bytes();
            b.len() == 4 && b[0] == b'w' && b[1] == b'i' && b[2] == b'd' && b[3] == b'e'
        }
        None => false,
    }
}

/// Sniper: midpoint of N5TM (1101 Hz) / K9KFR (1331 Hz) — see the module
/// doc comment. Wideband: centre of the 100-3000 Hz production band, so
/// the DDC's own passband spans it.
const CENTER_HZ: f32 = if WIDEBAND { 1550.0 } else { 1216.0 };
/// Sniper ±250 Hz around the two known signals; wideband ±1450 Hz, i.e.
/// 100-3000 Hz, matching `mfsk-ffi`'s own FST4-60A null-options default.
const SEARCH_HALF_WIDTH_HZ: f32 = if WIDEBAND { 1450.0 } else { 250.0 };
/// Bandwidth handed to `grid_for` — drives `K` and therefore `nfft1`.
/// 600 Hz → K=256/nfft1=512; 2900 Hz → K=1024/nfft1=2048. Both radix-2
/// and both under the configured `CONFIG_DSP_MAX_FFT_SIZE_8192` ceiling.
const GRID_BANDWIDTH_HZ: f32 = if WIDEBAND { 2900.0 } else { 600.0 };
/// Production default is 0.8 (real weak-signal candidates near FST4-60's
/// -28 dB threshold score as low as ~1.4-4.5, per the DDC recall
/// measurement in `fst4::ddc`'s own doc comment) -- and that **is** what
/// the `=wide` build uses, deliberately: a wideband measurement taken at
/// a raised threshold would not be measuring the production search.
///
/// The sniper default raises it to 8.0, sniper-bench-scoped only: a
/// host-side sweep against this exact golden recording (issue #307,
/// 2026-08-21) found the real signals' raw coarse_sync scores (N5TM
/// 38.68, K9KFR 29.79) sit ~5x above the recording's own noise-floor
/// ceiling (5.85, the next-highest of 48 false alarms scattered across
/// the whole ±250 Hz/±2.5 s search grid -- confirmed *not*
/// near-duplicates of each other or of the real signals, so
/// `coarse_sync`'s own ±4 Hz/40 ms NMS legitimately has nothing to
/// collapse there). 8.0 clears that ceiling with margin while staying
/// ~4x under both real signals' scores.
///
/// **Do not carry 8.0 into any production/general-search path**, and
/// note it is not merely "a stricter score gate": `sync_min` is reused
/// as the stage-1 threshold in `engine::sync::coarse_sync`, and
/// `stage1_norm` is normalised to ~1.0 at the noise floor by
/// construction -- so a value of 8.0 effectively disables issue #146's
/// full-slot OR-gate, which is the mechanism that recovers ~3 dB of
/// FST4 sensitivity. Harmless on a strong-signal recording, ruinous on
/// weak ones.
const SYNC_MIN: f32 = if WIDEBAND { 0.8 } else { 8.0 };
const MAX_CAND: usize = 50;

/// `MFSK_FST4_DDC_BENCH_MODE=monitor` replaces the batch
/// refine-all -> dedup -> decode-survivors pipeline with a **streaming,
/// coarse-score-ordered** loop: each candidate is refined and decoded
/// immediately, in the order `coarse_sync` already ranked them, with no
/// dedup barrier ahead of the first decode.
///
/// The batch shape optimises for *completeness* — every survivor gets
/// the full ladder before anything is reported. A monitor optimises for
/// **time-to-first-decode**, and the two want opposite orderings.
/// Measured on the 2026-08-22 wideband run, the batch path's own
/// numbers show why:
///
/// - `coarse_sync` ranked the two real signals **#1 (score 31.43) and
///   #2 (25.46)** against a noise ceiling of 4.33 — a ~6x margin.
/// - `dedup_refined_candidates`' rule ranks by *refined* score, where a
///   near-duplicate cell beat rank 1 by 1107.8 vs 1107.3 (**0.05%**).
///   That marginal difference overrode the 13x coarse-score difference,
///   and the surviving representative sat at position 7 of 38.
/// - Recall was unaffected (both signals decoded either way), but every
///   decode landed *after* 38.9 s of expensive noise candidates.
///
/// Host-verified before building this: refining and decoding the raw
/// coarse ranks in order yields `CQ N5TM EL29` at rank 1 and
/// `CQ K9KFR EN71` at rank 2, so the ordering signal is real and the
/// dedup barrier is what was hiding it.
///
/// **No dedup here, deliberately** — duplicates decode to the same
/// message and are collapsed by message at report time (what
/// `decode_frame_impl`'s own post-decode dedup does anyway), which
/// costs a repeat decode but never delays the first one.
const MONITOR: bool = is_monitor(option_env!("MFSK_FST4_DDC_BENCH_MODE"));

const fn is_monitor(v: Option<&str>) -> bool {
    match v {
        Some(s) => {
            let b = s.as_bytes();
            b.len() == 7
                && b[0] == b'm'
                && b[1] == b'o'
                && b[2] == b'n'
                && b[3] == b'i'
                && b[4] == b't'
                && b[5] == b'o'
                && b[6] == b'r'
        }
        None => false,
    }
}

/// Wall-clock ceiling for the monitor loop, from the moment the
/// candidate loop starts. Generous by design: a receiver that never
/// transmits is not bound by FST4-60's ~7.2 s TX-turnaround guard time
/// (that budget exists to leave room to answer), only by keeping up
/// with the slot cadence — the same reasoning `wspr_app` already ships
/// on, decoding 82.8-90.1 s against a 110 s deadline in a 120 s slot.
/// The summary below reports against *both* bounds so the strict-guard
/// case stays visible.
const MONITOR_DEADLINE_MS: i64 = 45_000;

/// FST4-60's post-slot guard budget (`ROADMAP.md`: FST4-15 ~4.9 s,
/// -30 ~6.6, -60 ~7.2, -120 ~9.7, -300 ~12.3 — the T/R period is *not*
/// the decode budget). Reported as a marker, not enforced.
const TX_TURNAROUND_BUDGET_MS: i64 = 7_200;

/// `MFSK_FST4_DDC_BENCH_CAP_MS=<n>` puts a **per-candidate wall-clock
/// cap** on the decode ladder. `0`/unset keeps the uncapped path.
///
/// The uncapped monitor run (2026-08-22) reached only rank 13 of 50
/// inside its 45 s deadline because two noise candidates ran the full
/// OSD escalation for **~19 s each — 38 s of the 55 s loop**. Real
/// signals in that same run decoded in 53-54 ms, so the ladder's long
/// tail is spent exclusively on candidates that never decode, and
/// cutting it costs a monitor almost nothing while roughly doubling how
/// much of the band it covers per slot.
///
/// This is what `fst4::rung_major::decode_phase_split_timed`'s
/// `budget_ok` parameter has been for since #317 — it has existed with
/// every caller passing `None`. Wiring it is the point of this mode.
/// **Phase A is deliberately never gated** by that function (the
/// cheapest rung always runs), so a capped candidate still gets its
/// `llra` attempt; the cap only truncates the escalation above it.
const MONITOR_CAP_MS: i64 = parse_dec(option_env!("MFSK_FST4_DDC_BENCH_CAP_MS"));

/// `str::parse` isn't const, and this has to be a `const` because
/// [`MONITOR_CAP_MS`] gates a `const` code path — same reason
/// [`is_wide`] spells its comparison out bytewise. Non-numeric input
/// yields 0 (= disabled) rather than failing the build, matching how
/// the other switches here treat an unrecognised value.
const fn parse_dec(v: Option<&str>) -> i64 {
    match v {
        Some(s) => {
            let b = s.as_bytes();
            if b.is_empty() {
                return 0;
            }
            let mut i = 0usize;
            let mut acc = 0i64;
            while i < b.len() {
                let d = b[i];
                if d < b'0' || d > b'9' {
                    return 0;
                }
                acc = acc * 10 + (d - b'0') as i64;
                i += 1;
            }
            acc
        }
        None => 0,
    }
}

/// Run the per-candidate work across both cores (issue #327) — the
/// monitor's decode loop, and the sniper path's `refine_all`.
///
/// **On by default; `MFSK_FST4_DDC_BENCH_DUAL=0` restores single-core**
/// for a before/after. Opt-in while it was new; the same per-candidate
/// function runs either way, it measured 1.67x on the monitor loop, and
/// the receiver app ships it — so the default should be what a receiver
/// runs.
const DUAL_CORE: bool = !is_zero(option_env!("MFSK_FST4_DDC_BENCH_DUAL"));

/// Run the DDC and the spectrogram **incrementally, block by block**,
/// the way a receiver processes audio as it arrives, instead of
/// batching both after the slot ends.
///
/// **On by default; `MFSK_FST4_DDC_BENCH_STREAM=0` restores the batch
/// shape** for a before/after comparison. It was opt-in while it was
/// new and wideband-only; a receiver streams, so the default should be
/// what a receiver does, for both bands.
///
/// This does not make the front end cheaper; it moves it. A receiver
/// has the whole 60 s capture window to do this work, so what it costs
/// against is a *duty cycle*, not the post-slot guard budget. WSPR
/// already ships exactly this shape (`wspr_app`'s `ddc_loop` ->
/// `DDC_READY_IDX` -> `scan_loop`), measured there at 18.5-24.1 s of
/// DDC running beside an 82.8-90.1 s decode inside a 110 s deadline.
///
/// Measured, 2026-08-22, on real CoreS3 — what streaming takes off the
/// post-slot budget:
///
/// | band | batch, all post-slot | streamed: during capture | streamed: post-slot |
/// |---|---:|---:|---:|
/// | wideband | 4818 + 4039 ms | 5753 ms (9% duty) | **1119 ms + decode** |
/// | sniper | 1314 + 394 ms | 1539 ms (2.6% duty) | **250 ms + refine/decode** |
///
/// The sniper's whole post-slot path goes 3365 -> 1887 ms on that
/// move, with the same two decodes at the same frequencies; the
/// wideband's first decode goes 10 060 -> 2301 ms, which is the
/// difference between missing the 7.2 s TX guard and fitting inside it.
///
/// Note the capture-side total is slightly *larger* than the batch
/// DDC alone (1539 vs 1314 ms for the sniper) because it now includes
/// the spectrogram build that used to happen inside `coarse_sync`.
/// That is the point: it is the same work, paid where there is room
/// for it.
const STREAM_FRONTEND: bool = !is_zero(option_env!("MFSK_FST4_DDC_BENCH_STREAM"));

/// Block size the streaming front end is fed in. 12 000 samples = 1 s
/// of 12 kHz audio, a plausible unit for a real capture path and large
/// enough that per-block overhead is irrelevant. The spectrogram is
/// bit-identical at any block size (pinned by
/// `fst4_spectrogram_builder`), so this is a pacing choice, not a
/// numerical one.
const STREAM_BLOCK: usize = 12_000;

/// `Some("0")` — the opt-*out* form, for switches that default on.
const fn is_zero(v: Option<&str>) -> bool {
    match v {
        Some(s) => {
            let b = s.as_bytes();
            b.len() == 1 && b[0] == b'0'
        }
        None => false,
    }
}

const fn is_one(v: Option<&str>) -> bool {
    match v {
        Some(s) => {
            let b = s.as_bytes();
            b.len() == 1 && b[0] == b'1'
        }
        None => false,
    }
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

/// `audio_bin` is the raw golden asset (`include_bytes!`,
/// `i16[720000]` little-endian — same layout `fst4_bench`'s own
/// `fst4_60_golden_audio.bin` doc comment describes). Byte-wise, not a
/// transmute: 1-byte `include_bytes!` alignment faults an unaligned
/// `i16` load on Xtensa.
pub fn run(audio_bin: &[u8]) {
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!("fst4_ddc_bench: golden asset {} bytes", audio_bin.len());
    log::info!(
        "fst4_ddc_bench: band={} center {:.0} Hz +/-{:.0} Hz, sync_min {:.1}, max_cand {}",
        if WIDEBAND {
            "WIDE (100-3000 Hz, production sync_min)"
        } else {
            "sniper"
        },
        CENTER_HZ,
        SEARCH_HALF_WIDTH_HZ,
        SYNC_MIN,
        MAX_CAND,
    );
    log_heap("boot");

    let audio: Vec<i16> = audio_bin
        .as_chunks::<2>()
        .0
        .iter()
        .map(|b| i16::from_le_bytes(*b))
        .collect();
    log::info!(
        "fst4_ddc_bench: loaded {} samples ({} s @ 12 kHz)",
        audio.len(),
        audio.len() / 12_000
    );

    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    if DUAL_CORE {
        // At boot, not per batch: the correlation fill below is the
        // first thing that wants the second core, and a real app has
        // to spawn here anyway — before WiFi, per
        // `fst4_dual_core::init`'s ordering constraint.
        fst4_dual_core::init();
    }

    // ---- Stage 1: DDC + coarse_sync ----
    //
    // Both modes go through `SlotCapture`, the same front end a
    // receiver runs; `STREAM_FRONTEND` decides only whether the
    // spectrogram is built alongside the DDC (as an app does, during
    // capture) or left for `compute_spectra` afterwards, and whether
    // the audio arrives in blocks or one lump. Block size does not
    // change the spectrogram — that is pinned by
    // `fst4_spectrogram_builder` — so this is a pacing choice.
    let cfg = monitor_config();
    let rx_grid = cfg.rx_grid();
    let grid = grid_for(GRID_BANDWIDTH_HZ);

    let t0 = now_us();
    let mut capture = SlotCapture::new(cfg, STREAM_FRONTEND);
    if STREAM_FRONTEND {
        for chunk in audio.chunks(STREAM_BLOCK) {
            capture.push_i16(chunk);
        }
    } else {
        capture.push_i16(&audio);
    }
    let slot = capture.finish();
    let coarse_delay_orig = slot.delay;
    let t_ddc = now_us() - t0;
    log::info!(
        "fst4_ddc_bench: DDC {} -> {} complex samples in {} ms",
        audio.len(),
        slot.coarse_i.len(),
        t_ddc / 1000,
    );
    log_heap("post-ddc");

    // Diagnostic split of `coarse_sync`'s cost into its spectrogram
    // build vs its correlation search — the two have completely
    // different architectural options and the wideband measurement
    // (6869 ms total on the first `=wide` run) doesn't say which
    // dominates.
    //
    // It matters because `compute_spectra`'s per-symbol-window FFT rows
    // are independent and computable **as audio arrives**, exactly like
    // the DDC ahead of it — so whatever share it holds could leave the
    // post-slot budget entirely via the capture-time streaming WSPR
    // already runs in production (`wspr_app`'s `ddc_loop` ->
    // `DDC_READY_IDX` -> `scan_loop`). The correlation search cannot:
    // it needs the completed spectrogram.
    //
    // Costs one redundant `compute_spectra` (this is a bench), and
    // replicates `coarse_sync`'s own `ia`/`ib`/`headroom` derivation
    // (`engine/sync.rs:566-586`) so the timed call crops identically
    // rather than over-reporting on a wider band.
    let mut t_spectra_us = 0i64;
    if !STREAM_FRONTEND {
        let d = SyncDims::of::<Fst4s60>(rx_grid.sample_rate_hz);
        let ntones = <Fst4s60 as mfsk_core::engine::ModulationParams>::NTONES as usize;
        let usable = rx_grid.usable_bins(&d);
        let ia = rx_grid.bin_of(&d, CENTER_HZ - SEARCH_HALF_WIDTH_HZ);
        let headroom = d.nfos * (ntones - 1) + 1;
        let ib = rx_grid
            .bin_of(&d, CENTER_HZ + SEARCH_HALF_WIDTH_HZ)
            .min(usable.saturating_sub(headroom));
        if ib >= ia {
            let t_spec0 = now_us();
            let s = compute_spectra::<Fst4s60>(
                AudioSource::Complex(&slot.coarse_i, &slot.coarse_q),
                ia,
                (ib + headroom).min(usable.saturating_sub(1)),
                rx_grid,
            );
            let t_spec = now_us() - t_spec0;
            log::info!(
                "fst4_ddc_bench: [diag] compute_spectra alone = {} ms (nfft1 {}, bins {}..={}, {} rows) \
                 -- streamable during capture; coarse_sync total minus this is the search",
                t_spec / 1000,
                d.nfft1,
                ia,
                ib,
                d.nhsym,
            );
            core::hint::black_box(&s);
            t_spectra_us = t_spec;
        }
    }
    log_heap("post-diag-spectra");

    let t1 = now_us();
    let candidates = if slot.spectra.is_some() {
        // Streaming: the spectrogram was finished during capture, so
        // only the correlation search is post-slot work — and its fill
        // half runs on both cores.
        let (cands, fill_us, rank_us) = fst4_monitor::coarse_search(&slot);
        log::info!(
            "fst4_ddc_bench: [diag] coarse search split — fill {} ms ({}), rank {} ms",
            fill_us / 1000,
            if DUAL_CORE { "dual-core" } else { "single-core" },
            rank_us / 1000,
        );
        cands
    } else {
        coarse_sync::<Fst4s60>(
            AudioSource::Complex(&slot.coarse_i, &slot.coarse_q),
            CENTER_HZ - SEARCH_HALF_WIDTH_HZ,
            CENTER_HZ + SEARCH_HALF_WIDTH_HZ,
            SYNC_MIN,
            None,
            MAX_CAND,
            rx_grid,
        )
    };
    let t_coarse_sync = now_us() - t1;
    log::info!(
        "fst4_ddc_bench: coarse_sync (via DDC, K={}, nfft1={}) -> {} candidates in {} ms",
        grid.k,
        grid.nfft1,
        candidates.len(),
        t_coarse_sync / 1000,
    );
    for c in &candidates {
        log::info!(
            "    cand {:8.1} Hz  dt {:+.3} s  score {:.2}",
            c.freq_hz,
            c.dt_sec,
            c.score
        );
    }
    log_heap("post-coarse-sync");

    if MONITOR {
        run_monitor_loop(&slot, &candidates, t_ddc, t_spectra_us, t_coarse_sync);
        return;
    }

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
    let t2a = now_us();
    let mut refine_meta = fst4_monitor::refine_all(&slot, &candidates);
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
        // Reuse the baseband Stage 2a already built, when it kept it.
        // Rebuilding it is a second full recentre for a result that is
        // bit-identical to the one just thrown away — `ddc_refine` is a
        // pure function of the same three inputs — so the only reason
        // to do it is memory, and `refine_all` is the thing that knows
        // whether there was any.
        let m = &mut refine_meta[idx];
        let cd0 = match m.cd0.take() {
            Some(cd0) => cd0,
            None => fst4_monitor::ddc_refine(
                &cfg,
                cand,
                &slot.coarse_i,
                &slot.coarse_q,
                coarse_delay_orig,
            ),
        };
        let refine_us = now_us() - t_r;

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

/// What this bench's build-time switches add up to, as the runtime
/// config [`fst4_monitor`] takes. Every field is one `option_env!`
/// constant above, so the bench keeps its "change a value, rebuild,
/// re-measure" shape while the pipeline itself carries no build
/// switches at all.
fn monitor_config() -> MonitorConfig {
    MonitorConfig {
        band: if WIDEBAND { Band::Wide } else { Band::Sniper },
        center_hz: CENTER_HZ,
        half_width_hz: SEARCH_HALF_WIDTH_HZ,
        grid_bandwidth_hz: GRID_BANDWIDTH_HZ,
        sync_min: SYNC_MIN,
        max_cand: MAX_CAND,
        cap_ms: MONITOR_CAP_MS,
        full_depth_ranks: FULL_DEPTH_RANKS,
        deadline_ms: MONITOR_DEADLINE_MS,
        dual_core: DUAL_CORE,
    }
}

/// `MFSK_FST4_DDC_BENCH_MODE=monitor` — see [`MONITOR`] for why this
/// exists and what it measures. Streaming, coarse-score-ordered,
/// deadline-bounded; reports **time-to-first-decode** rather than a
/// total, because that is the number a monitoring receiver is actually
/// judged on.
#[allow(clippy::too_many_arguments)]
/// Whether this candidate, at coarse-score `rank`, gets the **full**
/// decode path or the capped one.
///
/// Measured 2026-08-22, and the reason this is tiered by rank rather
/// than applied uniformly either way:
///
/// - The cap (#332) buys band coverage — 13 -> 39 candidates in a 45 s
///   deadline — but `fst4_monitor_cap_sensitivity` then measured it
///   costing **~2/3 of threshold recall** at -27 dB, because it
///   truncates the OSD rung that carries 43 of 65 decodes there.
/// - Timing diversity (`i0 +/- 1`) is worth a further **10/100** at
///   -27 dB (`fst4_rung_major_recall_gap`), and the production path
///   applies it automatically whenever OSD depth is on
///   (`pipeline.rs:1236`).
/// - Those two cannot be combined under a cap. `Schedule::PhaseSplit`
///   puts extra offsets in **Phase C**, which runs last and is the
///   first thing `budget_ok` cuts — measured directly: adding
///   `&[0, 1, -1]` under a 150 ms cap changed per-candidate decode
///   times by <=1 ms and the loop total by 20 ms out of 31.8 s, i.e.
///   the extra offsets were never reached.
///
/// So a uniform cap gives up both mechanisms that carry weak-signal
/// recall. What makes tiering work instead is that the signal is not
/// uniformly distributed through the list:
/// `fst4_monitor_cap_sensitivity` and `fst4_wideband_max_cand`
/// independently put it at **median coarse-score rank 1 at every SNR**.
/// Spending the full ladder on the first few candidates and capping the
/// tail therefore buys threshold sensitivity exactly where the signal
/// is, and band coverage everywhere else.
/// How many top-ranked candidates get the full, uncapped decode path
/// (which brings OSD *and* production's automatic `i0 +/- 1` retry).
/// `MFSK_FST4_DDC_BENCH_FULL_RANKS=<n>`; `0` restores a uniform cap.
///
/// Default 8, chosen by measuring both ends rather than by intuition.
/// The rank at which the golden signal decodes, out of the 68/100
/// achievable at -27 dB (`fst4_wideband_max_cand`), against the
/// measured loop cost of extending the tier (real CoreS3, wideband,
/// dual-core, streaming front end):
///
/// | tier | loop | vs uniform cap | recall captured |
/// |---:|---:|---:|---:|
/// | 0 (uniform cap) | 31 786 ms | — | ~22 (see #337) |
/// | 4 | 32 240 ms | +1.4% | 55/68 (81%) |
/// | **8** | **33 141 ms** | **+4.3%** | **60/68 (88%)** |
/// | no cap at all | deadline hit at 13 candidates | — | 68/68 |
///
/// 8 buys five more decodes than 4 for 0.9 s, and still leaves the 45 s
/// deadline unreached (all 50 candidates complete) with the first
/// decode unmoved at 5207 ms post-slot. Past 8 the rank distribution
/// flattens — ranks 9-50 hold only five of the 68 — so a larger tier
/// pays the 3x ladder for candidates that are almost all noise.
const FULL_DEPTH_RANKS: usize = {
    let v = parse_dec(option_env!("MFSK_FST4_DDC_BENCH_FULL_RANKS"));
    if v == 0 && option_env!("MFSK_FST4_DDC_BENCH_FULL_RANKS").is_none() {
        8
    } else {
        v as usize
    }
};

/// `MFSK_FST4_DDC_BENCH_MODE=monitor` — see [`MONITOR`] for why this
/// exists and what it measures. Streaming, coarse-score-ordered,
/// deadline-bounded; reports **time-to-first-decode** rather than a
/// total, because that is the number a monitoring receiver is actually
/// judged on.
///
/// `MFSK_FST4_DDC_BENCH_DUAL=1` runs the same candidate list across
/// both cores via [`fst4_dual_core`] (issue #327). Identical work and
/// identical per-candidate function either way — only who runs it
/// differs — so the two are directly comparable on one build pair.
#[allow(clippy::too_many_arguments)]
fn run_monitor_loop(
    slot: &CapturedSlot,
    candidates: &[SyncCandidate],
    t_ddc_us: i64,
    t_spectra_us: i64,
    t_coarse_sync_us: i64,
) {
    log::info!(
        "fst4_ddc_bench: MONITOR mode — {} candidates, coarse-score order, no dedup, \
         deadline {} ms, cap {} ms beyond rank {}, {}",
        candidates.len(),
        MONITOR_DEADLINE_MS,
        MONITOR_CAP_MS,
        FULL_DEPTH_RANKS,
        if DUAL_CORE { "DUAL-CORE" } else { "single-core" },
    );

    // `coarse_sync`'s spectrogram build and the DDC ahead of it are
    // both row/sample-incremental, so a real receiver computes them as
    // audio arrives (WSPR ships exactly this: `wspr_app`'s `ddc_loop`
    // -> `DDC_READY_IDX` -> `scan_loop`). Only the correlation search
    // needs the completed spectrogram. Report both accountings rather
    // than picking one, since which applies is an app-architecture
    // decision this bench doesn't make.
    let t_search_us = t_coarse_sync_us - t_spectra_us;
    let front_end_all_us = t_ddc_us + t_coarse_sync_us;
    let front_end_streamed_us = t_search_us;

    // The worker is already up — `run` spawns it at boot so the
    // correlation fill can use it too.

    let t_loop0 = now_us();
    let hits = fst4_monitor::run_candidate_loop(slot, candidates);
    let t_loop_ms = (now_us() - t_loop0) / 1000;
    let n_tried = hits.len();

    // Collapse by decoded *message*, not by position — a monitor
    // reports stations, and two coarse cells for one signal are the
    // same station. Cheaper than a dedup barrier ahead of the loop and
    // it cannot delay a first decode.
    let mut decoded: Vec<(String, f32, i64)> = Vec::new();
    for h in &hits {
        match &h.msg {
            Some(text) => {
                let dup = decoded.iter().any(|(m, _, _)| m == text);
                log::info!(
                    "fst4_ddc_bench: [monitor] rank {:2}/{} {:8.1} Hz cscore {:5.2} \
                     refine {:4} ms decode {:5} ms  t+{:5} ms  DECODED {:?}{}",
                    h.rank + 1,
                    candidates.len(),
                    h.freq_hz,
                    h.cscore,
                    h.refine_us / 1000,
                    h.decode_us / 1000,
                    h.t_ms,
                    text,
                    if dup { " (dup)" } else { "" },
                );
                if !dup {
                    decoded.push((text.clone(), h.refined_hz, h.t_ms));
                }
            }
            None => {
                log::info!(
                    "fst4_ddc_bench: [monitor] rank {:2}/{} {:8.1} Hz cscore {:5.2} \
                     refine {:4} ms decode {:5} ms  t+{:5} ms",
                    h.rank + 1,
                    candidates.len(),
                    h.freq_hz,
                    h.cscore,
                    h.refine_us / 1000,
                    h.decode_us / 1000,
                    h.t_ms,
                );
            }
        }
    }

    log::info!(
        "fst4_ddc_bench: MONITOR done — {} candidates tried, {} distinct decodes, loop {} ms",
        n_tried,
        decoded.len(),
        t_loop_ms,
    );
    if STREAM_FRONTEND {
        // `t_ddc_us` here is DDC *and* spectrogram: the builder was fed
        // inside the same block loop, so the two are one capture-time
        // cost and cannot be separated after the fact.
        let capture_ms = t_ddc_us / 1000;
        log::info!(
            "fst4_ddc_bench: front end (STREAMING) — ddc+spectrogram {} ms done during capture \
             ({}% duty of a 60 s slot) + search {} ms post-slot = {} ms total",
            capture_ms,
            capture_ms * 100 / 60_000,
            t_search_us / 1000,
            front_end_all_us / 1000,
        );
    } else {
        log::info!(
            "fst4_ddc_bench: front end (BATCH) — ddc {} ms + spectra {} ms \
             (both streamable during capture) + search {} ms = {} ms total",
            t_ddc_us / 1000,
            t_spectra_us / 1000,
            t_search_us / 1000,
            front_end_all_us / 1000,
        );
    }
    for (msg, hz, t_ms) in &decoded {
        let strict = front_end_streamed_us / 1000 + t_ms;
        let naive = front_end_all_us / 1000 + t_ms;
        log::info!(
            "fst4_ddc_bench:   {:?} @ {:.1} Hz — loop t+{} ms | post-slot {} ms (front end streamed) \
             | {} ms (nothing streamed) | TX-guard budget {} ms -> {}",
            msg,
            hz,
            t_ms,
            strict,
            naive,
            TX_TURNAROUND_BUDGET_MS,
            if strict <= TX_TURNAROUND_BUDGET_MS { "FITS" } else { "over" },
        );
    }
    log::info!("fst4_ddc_bench: stack headroom = {} B", stack_headroom());
    log_heap("post-monitor");
    log::info!("=== fst4_ddc_bench (monitor) complete ===");
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
