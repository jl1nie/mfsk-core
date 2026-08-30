//! FT4 whole-slot bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! **The question**: FT4 has never been built for, let alone run on,
//! any board in this tree — `docs/reference/EMBEDDED.md`'s per-protocol
//! table said "no embedded path at all yet", and no embedded crate
//! enabled `mfsk-core/ft4`. Does FT4's slot work fit the ~1.96 s a
//! 7.5 s slot leaves after the signal ends?
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
//! ## Nothing is baked any more except the audio
//!
//! The 2026-08-29 revision of this bench shipped two host-computed
//! assets because two stages could not run here at all, and said so:
//! the wideband FFT cache (`fft1_size = 92_160` — not a power of two
//! and 11× past `CONFIG_DSP_MAX_FFT_SIZE_8192`) and the coarse
//! candidate list (`ft4_coarse_sync`'s own `NFFT1 = 2304`). It measured
//! per-candidate work and was explicitly *not a receiver*.
//!
//! Both holes closed on 2026-08-30, and this revision runs the whole
//! slot from `ft4_golden_audio.bin` — 180 000 bytes of `i16[90_000]`,
//! the same 12 kHz slot a UAC capture would hand it:
//!
//! - `engine::dsp::fft_mixed_2304` (256 × 9, the 9 over `fft_15::fft_3`)
//!   lets **`ft4_coarse_sync` run here**, so the candidate list is
//!   computed rather than shipped. The baked list is kept and compared
//!   against — see [`compare_candidates`] — because that comparison is
//!   the only check that the device's mixed-radix kernel finds what
//!   rustfft found.
//! - `ft4::ddc::candidate_baseband` replaces `downsample_cached`
//!   outright: mixing to the band centre and decimating by an integer
//!   18 needs no transform, so the 92 160-point stage is *gone* rather
//!   than accelerated. `ft4_ddc_equivalence` holds it to the same
//!   decodes on the golden and to 0.0 dB across a 560-file sweep.
//!
//! The FFT cache asset is still `include_bytes!`d, and this bench still
//! runs a `downsample_cached` arm beside the DDC one — as the control
//! that makes the swap's cost a measured delta on this silicon rather
//! than a host projection. A receiver would ship neither: FT4's
//! `snr_db` is a closed form over the coarse candidate score
//! (`pipeline::ft4_snr_db`, `ft4_decode.f90:226,452-457`), so on the
//! DDC arm nothing reads the cache at all, and the arm below passes an
//! **empty slice** to prove it on the board the same way
//! `ft4_ddc_equivalence::ft4_ddc_arm_never_reads_the_wideband_cache`
//! proves it on host.
//!
//! ## Measured, 2026-08-30 (CoreS3, 240 MHz, opt-level 3, 12 candidates)
//!
//! `logs/ft4-bench_wholeslot_2026-08-30.log`, full analysis in
//! `docs/notes/FT4_BENCHMARK.md` §25.
//!
//! | arm | coarse | candidates | slot | vs 1 960 ms |
//! |---|---:|---:|---:|---:|
//! | FFT EMBEDDED | 1 288 | 2 839 | 4 127 | 2.11x |
//! | FFT FULL | 1 288 | 2 831 | 4 119 | 2.10x |
//! | DDC EMBEDDED | 1 288 | 3 856 | 5 144 | 2.62x |
//! | DDC FULL | 1 288 | 3 853 | 5 141 | 2.62x |
//! | **DDC EMBEDDED +/-0.5 s (SHIP)** | 1 288 | 3 287 | **4 576** | **2.33x** |
//!
//! 11 decodes in every arm, matching host. `compare_candidates`: 12/12
//! paired, max dfreq 0.00 Hz, max dscore 0.00 % — the device's 256 x 9
//! factorisation selects exactly the peaks rustfft's planner does.
//!
//! Three things this contradicts, all of them projections made before
//! anything ran:
//!
//! - **`ft4_coarse_sync` is 1 288 ms**, not the "0.3 ms on host, so
//!   negligible" this doc used to claim — 66 % of the budget alone.
//! - **the DDC is 2.3x slower than `downsample_cached`** here (154 006
//!   vs 66 974 us/cand). It is what makes a receiver possible, since
//!   the 92 160-point transform cannot run at all; it is not a saving,
//!   and the budget projection had treated it as one.
//! - **internal-DRAM `cd0` placement is 1.01x** (was 1.12x) and
//!   **OSD is free** on this candidate list (8 ms across all 12) —
//!   its cost scales with BP *failures*, and `sync_min = 1.2` removed
//!   the noise candidates that were failing.
//!
//! Search (960 ms) + LLR/BP (479 ms) fit the budget with room. The
//! overrun is entirely the two stages nothing had measured.
//!
//! ## Host reference for this recording
//!
//! Measured 2026-08-29/30 by the bake test itself, on the WSJT-X golden
//! `000000_000002.wav` (itself an `ft4sim_mult` scene, not an off-air
//! capture — `docs/notes/FT4_BENCHMARK.md` §23.5), single pass, this
//! exact search:
//!
//! - **12 candidates, 11 distinct decodes**, *identical* under
//!   `DecodeDepth::EMBEDDED` and `DecodeDepth::FULL` — OSD and full LLR
//!   effort buy nothing on *this file*. That does not generalise: on
//!   the 560-file sweep corpus at the 50 % crossing, `FULL` reaches 237
//!   and `EMBEDDED` 179 (`tests/ft4_candidate_budget.rs`), so the ship
//!   config does give up recall on weak signals even though it gives up
//!   none here. Which is why both depths are still timed below: the
//!   OSD decision is a cost question, and this is where the cost comes
//!   from.
//! - **Every device number recorded before 2026-08-30
//!   (`docs/notes/FT4_BENCHMARK.md` §17-19) was measured over 31
//!   candidates**, from a list generated with `sync_min = 0.05` —
//!   *below* the noise floor, since `getcandidates4.f90`
//!   baseline-normalises and puts noise at ~1.0. The generator now
//!   passes WSJT-X's own `syncmin = 1.2` (`ft4_decode.f90:195`) and the
//!   same 11 decodes come out of 12 candidates. Per-candidate figures
//!   carry over unchanged across that revision; slot totals do not.
//! - (The 14/14 golden the host recall test asserts needs
//!   `.sic_rounds(3)`; embedded FT8 ships a single pass, so a
//!   multi-pass figure would not describe what a board would run.)
//! - Whole-slot host wall-clock, rayon across 24 threads: ~100-105 ms
//!   (`docs/notes/BENCHMARKS.md`). Single-threaded is the honest
//!   comparison for an MCU projection — `parallel` is always off on
//!   embedded.
//!
//! ## The passes, and why the total has a split
//!
//! Timing one production call gives a number and no way to act on it.
//! This runs the stages separately as well, the same "inferred by
//! subtraction" framing the host diagnostic
//! `ft4_diag_candidate_cost_split` uses:
//!
//! | pass | what it calls | what it isolates |
//! |---|---|---|
//! | 0 | `ft4_coarse_sync` | the 2304-pt periodogram, once per slot |
//! | 1f | `downsample_cached` | the 5120-pt inverse FFT (control) |
//! | 1d | `candidate_baseband` | the DDC front end that replaces it |
//! | 2* | pass 1f + RMS-norm + `ft4_sync_search_window` | the Δt search |
//! | 3 | the production per-candidate call, four ways | everything |
//!
//! LLR/BP/OSD is then `pass3 − pass2 − pass1`. Passes 1 and 2 repeat
//! work pass 3 also does — that is what makes them a *split* rather
//! than a breakdown, and the total to quote is pass 0 + pass 3.

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
use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
use mfsk_core::engine::pipeline::{
    DecodeDepth, DecodeStrictness, process_candidate_basic, process_candidate_precomputed,
};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::engine::sync2d::ft4_sync_search_window;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::ddc::candidate_baseband;
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

/// The coarse search this bench runs, mirroring `ft4_wsjtx_samples.rs`'s
/// `bench_assets` module — which is what the baked control list was
/// generated against, so a divergence in [`compare_candidates`] means
/// the *kernel* differs and not the parameters.
///
/// `SYNC_MIN` is WSJT-X's own (`ft4_decode.f90:195` `syncmin = 1.2`).
/// It was 0.05 until 2026-08-30, which is below the noise floor:
/// `getcandidates4.f90` divides the smoothed spectrum by a fitted
/// baseline, so noise sits at ~1.0 and any lower threshold admits every
/// peak in the band. 31 candidates at 0.05 against 12 at 1.2 on this
/// recording, same 11 decodes; 67.1 against 1.6 on the 560-file sweep
/// corpus, also identical recall (`tests/ft4_candidate_budget.rs`).
/// Every stage below is per-candidate, so the device was paying 2.6×
/// for nothing.
const FREQ_MIN_HZ: f32 = 100.0;
/// See [`FREQ_MIN_HZ`].
const FREQ_MAX_HZ: f32 = 2700.0;
/// See [`FREQ_MIN_HZ`].
const SYNC_MIN: f32 = 1.2;
/// See [`FREQ_MIN_HZ`].
const MAX_CAND: usize = 100;

/// One 7.5 s slot at 12 kHz — the length of `ft4_golden_audio.bin`, and
/// what `ft4_coarse_sync` and `candidate_baseband` are both handed.
const SLOT_SAMPLES: usize = 90_000;

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
/// into internal DRAM on FT8's `cs` scratch; **it does not hold here —
/// measured 1.12× on 2026-08-29**, which is why the projection lives
/// in `feedback_bottleneck_hypothesis_measure_first` and the pass is
/// kept rather than promoted.
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

/// Parse `ft4_golden_audio.bin`: `i16 × 90_000`, LE.
fn load_audio(bin: &[u8]) -> Vec<i16> {
    assert_eq!(
        bin.len(),
        SLOT_SAMPLES * 2,
        "audio asset is {} B, expected {} B — re-run the bake test",
        bin.len(),
        SLOT_SAMPLES * 2
    );
    bin.chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
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

/// Does the device's `fft_mixed_2304` find the candidates rustfft
/// found?
///
/// This is the whole reason the baked list survives now that
/// `ft4_coarse_sync` runs here: it is no longer an *input*, it is the
/// control. `NFFT1 = 2304` is served on host by rustfft's own
/// mixed-radix planner and here by `engine::dsp::fft_mixed_2304`
/// (256 × 9), two different factorisations of the same transform, so
/// bit-equality is not on offer and is not what matters. What matters
/// is that the same peaks clear `SYNC_MIN` in the same places.
///
/// Reported, not asserted — a panic here would lose every timing
/// number in the run, and the interesting outcome is the size and
/// shape of a disagreement rather than its existence. `freq_hz` is
/// quantised to `DF_HZ` by `getcandidates4.f90`'s bin search, so a
/// non-zero Δfreq means a *different bin won*, not rounding.
fn compare_candidates(device: &[SyncCandidate], baked: &[SyncCandidate]) {
    if device.len() != baked.len() {
        log::warn!(
            "ft4_bench: coarse MISMATCH — device found {} candidates, host baked {}",
            device.len(),
            baked.len()
        );
    }
    let mut max_dfreq = 0.0f32;
    let mut max_dscore_pct = 0.0f32;
    let mut paired = 0usize;
    for d in device {
        // Nearest baked candidate in frequency; the lists are both
        // score-ordered, so index-wise pairing would mis-attribute a
        // single reordering as a wholesale disagreement.
        let mut best = f32::INFINITY;
        let mut best_score = 0.0f32;
        for b in baked {
            let df = (d.freq_hz - b.freq_hz).abs();
            if df < best {
                best = df;
                best_score = b.score;
            }
        }
        if best.is_finite() {
            paired += 1;
            max_dfreq = max_dfreq.max(best);
            if best_score.abs() > f32::EPSILON {
                max_dscore_pct =
                    max_dscore_pct.max(100.0 * (d.score - best_score).abs() / best_score.abs());
            }
        }
    }
    log::info!(
        "ft4_bench: coarse vs baked — {} device / {} baked, {paired} paired, \
         max Δfreq {max_dfreq:.2} Hz, max Δscore {max_dscore_pct:.2} %",
        device.len(),
        baked.len(),
    );
    for c in device {
        log::info!(
            "    cand {:7.2} Hz  dt {:+.3} s  score {:.3}",
            c.freq_hz,
            c.dt_sec,
            c.score
        );
    }
}

/// PIE-alignment counters around one stage.
///
/// `pie_alignment_report` is process-global and its length mask cannot
/// tell `fft_mixed_2304`'s 256-point inner rows from `fft_mixed_3840`'s,
/// so attribution has to come from *when* the counters move. That works
/// here because each pass runs exactly one kernel: pass 0 nothing but
/// the coarse stage's 2304s, pass 1f nothing but `downsample_cached`'s
/// 5120s, and pass 1d — a FIR chain — no transform at all, which makes
/// its row count a self-check on this reasoning rather than a
/// measurement.
///
/// **The question this exists to settle** (`FT4_BENCHMARK.md` §25,
/// follow-up 2): `ft4_coarse_sync` costs 1 288 ms, ~8.5 ms for each of
/// 152 transforms, even though the nine inner 256-point passes are
/// already esp-dsp's PIE assembly. One suspect is that none of them
/// take that assembly's in-place path: `MixedRadix2304Fft::process`
/// falls back to a copy-in/copy-out through `AlignedStaging` whenever
/// the caller's buffer is not 16-byte aligned, and the buffer is a
/// plain `vec![Complex::new(0.0, 0.0); NFFT1]`, whose guaranteed
/// alignment is `align_of::<Complex32>() = 4`. Whether esp-idf's
/// allocator happens to return 16 for it is not something host code
/// can answer — hence a counter rather than an argument.
///
/// Returns the fresh reading so the caller can chain it as the next
/// stage's baseline.
fn pie_delta(
    label: &str,
    stage_us: i64,
    before: ((usize, usize, usize, usize), (usize, usize, usize)),
) -> ((usize, usize, usize, usize), (usize, usize, usize)) {
    let now = crate::esp_dsp_fft::pie_alignment_report();
    let t = crate::esp_dsp_fft::pie_timing_report();
    let aligned = now.0.saturating_sub(before.0.0);
    let staged = now.2.saturating_sub(before.0.2);
    let total = aligned + staged;
    log::info!(
        "ft4_bench: {label} PIE inner rows: {aligned} in-place / {staged} staged \
         of {total} ({} % staged) | len mask: in-place {:#x}, staged {:#x}",
        if total > 0 { staged * 100 / total } else { 0 },
        now.1,
        now.3,
    );

    // The layer split. `process` is the whole mixed-radix transform;
    // `kernel` is esp-dsp's assembly inside it; `staging` is the
    // copy-in/copy-out. Combine (twiddles + gather/scatter, scalar Rust
    // in `mfsk_core::engine::dsp::fft_mixed_*`) is what is left, and
    // `outside` is the caller's own work around the transform — for
    // pass 0 that is the Nuttall window and the magnitude accumulation
    // in `symbol_spectra_avg`.
    let process = t.0.saturating_sub(before.1.0) as i64;
    let kernel = t.1.saturating_sub(before.1.1) as i64;
    let staging = t.2.saturating_sub(before.1.2) as i64;
    let combine = process - kernel - staging;
    let outside = stage_us - process;
    let pct = |v: i64| {
        if stage_us > 0 {
            v * 100 / stage_us
        } else {
            0
        }
    };
    log::info!(
        "ft4_bench: {label} split of {} ms: kernel {} ms ({} %) | staging {} ms ({} %) \
         | combine {} ms ({} %) | outside transform {} ms ({} %)",
        stage_us / 1000,
        kernel / 1000,
        pct(kernel),
        staging / 1000,
        pct(staging),
        combine / 1000,
        pct(combine),
        outside / 1000,
        pct(outside),
    );
    (now, t)
}

/// RMS-normalise a downsampled baseband to unit power, matching what
/// `process_candidate_basic_impl` does to its own `downsample_cached`
/// output (WSJT-X `ft4_decode.f90:231-232`). Both the search passes and
/// the DDC arm need it explicitly: `ft4_sync_search`'s score is
/// scale-dependent, and `compute_llr`'s `LLR_SCALE` is calibrated
/// against unit-RMS input. `candidate_baseband` deliberately does not
/// do it itself, so every caller does — same contract the host test
/// `ft4_ddc_equivalence` works under.
fn rms_normalise(cd0: &mut [Complex32]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

/// Production window: WSJT-X `ft4_decode.f90`'s three segments unioned,
/// `[-344, 1012]` downsampled samples = ±1.0 s about `dt = 0`
/// (`i0 = 333` at `ds_rate = 12_000/18`).
const FULL_WINDOW: (i32, i32) = (-344, 1012);
/// ±0.5 s — the largest width measured lossless on both the WSJT-X
/// golden (itself an `ft4sim_mult` scene, not an off-air capture —
/// §23.5) and an ft4sim DT sweep (`docs/notes/FT4_BENCHMARK.md` §18).
/// WSJT-X searches wide because it cannot assume a clock; a
/// UTC-anchored receiver can, so this is the shipping width.
const NARROW_WINDOW: (i32, i32) = (0, 667);

/// One production decode pass over the candidate list, front end
/// selectable.
///
/// The two arms are not symmetric and cannot be made so. The FFT arm is
/// `process_candidate_basic`, which builds its own `cd0` internally and
/// therefore always searches [`FULL_WINDOW`] — it is the unmodified
/// production call, kept as the control. The DDC arm has to build the
/// `cd0` and refine the position itself before handing both to
/// `process_candidate_precomputed`, which is exactly what gives it a
/// window parameter the FFT arm has no way to accept.
///
/// So the honest comparison is DDC-at-[`FULL_WINDOW`] against the FFT
/// arm; DDC-at-[`NARROW_WINDOW`] is the ship configuration and is
/// reported separately rather than compared against anything.
///
/// `fft_cache` is `&[]` on the DDC arm. Not a shortcut — the assertion.
/// If anything downstream read it, this would panic on the first index
/// instead of quietly costing 737 280 bytes of flash on a board that
/// does not need them.
fn decode_pass(
    label: &str,
    audio: &[i16],
    fft_cache: &[Complex32],
    candidates: &[SyncCandidate],
    depth: DecodeDepth,
    ddc: Option<(i32, i32)>,
) -> i64 {
    let t0 = now_us();
    let mut msgs: Vec<alloc::string::String> = Vec::new();
    for cand in candidates {
        // `known = &[]` is what `decode_frame_impl`'s own FT4 arm
        // passes — FT4 decodes raw candidates and dedups afterwards.
        // Passing accumulated results here would make the bench
        // cheaper than the code it is measuring.
        let r = match ddc {
            Some((ib_min, ib_max)) => {
                let mut cd0 = candidate_baseband(audio, cand.freq_hz);
                rms_normalise(&mut cd0);
                let s2 = ft4_sync_search_window::<Ft4>(&cd0, cand, ib_min, ib_max);
                process_candidate_precomputed::<Ft4>(
                    cand,
                    &[],
                    &FT4_DOWNSAMPLE,
                    depth,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    SYNC_Q_MIN,
                    (cd0, s2.freq_hz, s2.i0, s2.score),
                    false,
                    false,
                )
            }
            None => process_candidate_basic::<Ft4>(
                cand,
                fft_cache,
                &FT4_DOWNSAMPLE,
                depth,
                DecodeStrictness::Normal,
                &[],
                EqMode::Off,
                SYNC_Q_MIN,
            ),
        };
        if let Some(r) = r {
            if let Some(text) = r
                .message77()
                .try_into()
                .ok()
                .and_then(|m77: &[u8; 77]| unpack77(m77))
            {
                log::info!(
                    "    {text} | {:.1} Hz | dt {:.2} s | {:.0} dB",
                    r.freq_hz,
                    r.dt_sec,
                    r.snr_db
                );
                if !msgs.contains(&text) {
                    msgs.push(text);
                }
            }
        }
    }
    let total_us = now_us() - t0;
    log::info!(
        "ft4_bench: pass3[{label}] = {} ms ({} us/cand) | {} distinct decodes \
         (host reference for this asset: 11)",
        total_us / 1000,
        total_us / candidates.len().max(1) as i64,
        msgs.len(),
    );
    total_us
}

fn run_bench(audio_bin: &[u8], fft_cache_bin: &[u8], cand_bin: &[u8]) {
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
    let audio = load_audio(audio_bin);
    let fft_cache = load_fft_cache(fft_cache_bin);
    let baked = load_candidates(cand_bin);
    log::info!(
        "ft4_bench: loaded {} audio samples + {} FFT bins + {} baked candidates in {} ms",
        audio.len(),
        fft_cache.len(),
        baked.len(),
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

    // ── Pass 0: the coarse stage, on-device for the first time ───────
    //
    // Baseline the PIE counters here, not at boot: `prewarm` and the
    // asset load run transforms of their own, and they are not what is
    // being attributed.
    let mut pie = (
        crate::esp_dsp_fft::pie_alignment_report(),
        crate::esp_dsp_fft::pie_timing_report(),
    );
    let t0 = now_us();
    let candidates = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    let coarse_us = now_us() - t0;
    log::info!(
        "ft4_bench: pass0 ft4_coarse_sync = {} ms ({} candidates; host: 0.3 ms, {} baked)",
        coarse_us / 1000,
        candidates.len(),
        baked.len(),
    );
    pie = pie_delta("pass0 coarse (2304 = 256 x 9)", coarse_us, pie);
    compare_candidates(&candidates, &baked);
    if candidates.is_empty() {
        log::error!("ft4_bench: coarse stage found nothing — the rest of the run is meaningless");
        return;
    }
    let n = candidates.len() as i64;
    log_heap("post-coarse");

    // ── Pass 1: the two front ends, alone ────────────────────────────
    let t0 = now_us();
    let mut checksum = 0.0f32;
    for cand in &candidates {
        let cd0 = downsample_cached(&fft_cache, cand.freq_hz, &FT4_DOWNSAMPLE);
        // Consume the result so the loop can't be optimised away.
        checksum += cd0[0].re;
    }
    let downsample_us = now_us() - t0;
    log::info!(
        "ft4_bench: pass1f downsample_cached  = {} ms ({} us/cand, checksum {checksum:e})",
        downsample_us / 1000,
        downsample_us / n,
    );
    pie = pie_delta("pass1f downsample (5120 = 1024 x 5)", downsample_us, pie);

    let t0 = now_us();
    let mut checksum = 0.0f32;
    for cand in &candidates {
        let cd0 = candidate_baseband(&audio, cand.freq_hz);
        checksum += cd0[0].re;
    }
    let ddc_us = now_us() - t0;
    log::info!(
        "ft4_bench: pass1d candidate_baseband = {} ms ({} us/cand, checksum {checksum:e}) \
         | front-end delta {} ms over the slot",
        ddc_us / 1000,
        ddc_us / n,
        (ddc_us - downsample_us) / 1000,
    );
    // Expect 0 rows: the DDC is two FIR stages and two mixers, no
    // transform anywhere. A non-zero count here would mean the passes
    // above are not isolating what this report claims they isolate.
    pie = pie_delta("pass1d DDC (expect no transforms)", ddc_us, pie);
    let _ = pie;

    // ── Passes 2*: the Δt search, one variable at a time ─────────────
    //
    // `search_pass` prepares each candidate's cd0 exactly as
    // `process_candidate_basic_impl` does (downsample + RMS-normalise),
    // optionally copies it into internal DRAM, and times
    // `ft4_sync_search_window` alone. The copy is counted, so the
    // internal-DRAM rows are honest about what they cost as well as
    // what they save. Fed by the FFT front end deliberately: these
    // ratios are the ones §18-19 recorded, and changing two things at
    // once would make them incomparable.
    let search_pass = |label: &str, ib: (i32, i32), internal: Option<&mut [Complex32]>| {
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

    // ── Pass 3: the production per-candidate call, four ways ─────────
    let mut ship_us = 0i64;
    for (label, depth, ddc) in [
        ("FFT EMBEDDED", DecodeDepth::EMBEDDED, None),
        ("FFT FULL    ", DecodeDepth::FULL, None),
        ("DDC EMBEDDED", DecodeDepth::EMBEDDED, Some(FULL_WINDOW)),
        ("DDC FULL    ", DecodeDepth::FULL, Some(FULL_WINDOW)),
        ("DDC EMB +-0.5s (SHIP)", DecodeDepth::EMBEDDED, Some(NARROW_WINDOW)),
    ] {
        let us = decode_pass(label, &audio, &fft_cache, &candidates, depth, ddc);
        let slot_ms = (coarse_us + us) / 1000;
        log::info!(
            "ft4_bench: pass3[{label}] slot = coarse {} ms + candidates {} ms = {} ms \
             vs {DECODE_BUDGET_MS} ms budget -> {}",
            coarse_us / 1000,
            us / 1000,
            slot_ms,
            if slot_ms <= DECODE_BUDGET_MS {
                "FITS"
            } else {
                "OVER"
            },
        );
        if ddc == Some(NARROW_WINDOW) {
            ship_us = us;
        }
        log_heap("post-pass3");
    }

    if ship_us > 0 {
        log::info!(
            "ft4_bench: SHIP configuration — DDC front end, EMBEDDED depth, +/-0.5 s window, \
             no baked assets but the audio: {} ms of a {DECODE_BUDGET_MS} ms budget ({}.{:02}x)",
            (coarse_us + ship_us) / 1000,
            (coarse_us + ship_us) / 1000 / DECODE_BUDGET_MS,
            ((coarse_us + ship_us) / 1000 * 100 / DECODE_BUDGET_MS) % 100,
        );
    }

    log::info!(
        "ft4_bench: stack headroom {} B of {} B",
        stack_headroom(),
        BENCH_STACK,
    );
    log::info!(
        "ft4_bench: NOTE — the DDC arms pass an EMPTY fft_cache, so nothing they \
         report depends on the 737 280-byte baked transform; the FFT arms are the \
         control and are the only reason it is still linked in."
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
    audio: &'static [u8],
    fft_cache: &'static [u8],
    candidates: &'static [u8],
}

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` leaks the `BenchArgs` so it outlives this task —
    // same arg-passing shape as `fst4_bench`/`wspr_bench`.
    let args: &'static BenchArgs = unsafe { &*(arg as *const BenchArgs) };
    run_bench(args.audio, args.fft_cache, args.candidates);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// All three arguments are the baked golden assets (`include_bytes!`).
/// Only `audio` is an input — see the module doc: `fft_cache` feeds the
/// control arm and `candidates` is the control for the on-device coarse
/// stage. Spawns [`bench_task`] on a dedicated stack pinned to core 0,
/// then idles forever.
pub fn run(
    audio: &'static [u8],
    fft_cache: &'static [u8],
    candidates: &'static [u8],
) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let args: &'static BenchArgs = alloc::boxed::Box::leak(alloc::boxed::Box::new(BenchArgs {
        audio,
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
