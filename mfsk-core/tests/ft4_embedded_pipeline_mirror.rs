// SPDX-License-Identifier: GPL-3.0-or-later
//! The CoreS3 FT4 per-slot decode, reproduced call for call on the host.
//!
//! `ft4_ddc_equivalence` compares *front ends* — one candidate list, two
//! ways of building `cd0` — over the whole 90 000-sample slot. That is
//! not the receiver. The receiver
//! (`embedded-poc/embedded-shared/src/apps/ft4_rx.rs`) closes its
//! capture window early at `CAPTURE_CLOSE_SAMPLES = 81_300`, builds its
//! periodogram incrementally as audio arrives, decimates by two in the
//! same pass, searches a narrowed `WSJTX_WINDOW`, and shares one set of
//! coarse phasor tables across every candidate. Each of those changes
//! which candidates exist and where they land, so a number measured on
//! the other shape does not transfer.
//!
//! This file follows `decode_slot` instead, with the FreeRTOS plumbing
//! folded away:
//!
//! ```text
//! per 256-sample block   Ft4SavgBuilder::push_with_rows   (137 rows / slot)
//!                        SlotDecimator::push_i16          -> half, 6 kHz
//! at CAPTURE_CLOSE       savg.finish()
//!                        ft4_coarse_sync_from_savg(FREQ_MIN..MAX, SYNC_MIN, MAX_CAND)
//!                        Ft4CoarsePhasors::new::<Ft4>()   (once per slot)
//! per candidate          candidate_baseband_half -> rms_normalise
//!                        ft4_sync_search_window_with::<Ft4>(WSJTX_WINDOW, refs)
//!                        process_candidate_precomputed::<Ft4>(EMBEDDED, SYNC_Q_MIN)
//!                        unpack77
//! ```
//!
//! **What it does not reproduce**, deliberately, and what the board is
//! therefore still the only instrument for:
//!
//! - the deadline and its cut (`TX_TURNAROUND_BUDGET_MS`); every
//!   candidate runs here;
//! - the two-core split, so nothing here sees the IDF heap lock two
//!   cores contend for;
//! - PSRAM, the internal-DRAM threshold and the silent fallback between
//!   them, and the PIE 16-byte alignment that
//!   `docs/notes/FT4_BENCHMARK.md` §32.1 measured at 1 049 -> 1 789 ms;
//! - the slot grid, which on the board decides the phase this file
//!   takes as given.
//!
//! What it *is* the instrument for is everything computational: which
//! messages come out, and — through the counting allocator below — how
//! many allocations and bytes a candidate costs. Those questions do not
//! need a board, and answering them on one costs a build, a capture
//! window and a grid that has to re-anchor.
//!
//! ```sh
//! cargo test -p mfsk-core --release --features full,internal-testing \
//!     --test ft4_embedded_pipeline_mirror -- --nocapture
//! ```
#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use num_complex::Complex;

use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::engine::sync2d::Ft4CoarsePhasors;
use mfsk_core::ft4::Ft4;

#[allow(dead_code)]
mod common;
use common::ft4_rx_mirror as rx;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

// ── the counting allocator ──────────────────────────────────────────
//
// An integration test is its own binary, so this instruments this file
// and nothing else. It is what turns "the candidate loop allocates ~50
// times" from a code reading into a number a change has to move.

// **Thread-local, not global.** `cargo test` runs a file's tests
// concurrently by default, so global counters make each test count the
// others' allocations — which is exactly what happened here: adding the
// breakdown test below doubled the slot test's figures until this was
// fixed. `Cell` in a `const`-initialised `thread_local!` is a plain TLS
// slot, so the allocator does not allocate to record an allocation.
thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
    static ALLOC_BYTES: Cell<usize> = const { Cell::new(0) };
    /// Bytes currently live in allocations of at most
    /// [`INTERNAL_THRESHOLD`] bytes, and the highest that has reached.
    /// On the board those are the ones `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`
    /// steers into internal DRAM, so the peak is the internal-DRAM
    /// appetite of whatever ran in between.
    static SMALL_LIVE: Cell<usize> = const { Cell::new(0) };
    static SMALL_PEAK: Cell<usize> = const { Cell::new(0) };
}

/// `m5stack-cores3-app/sdkconfig.defaults`'s
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`.
const INTERNAL_THRESHOLD: usize = 2_048;

struct Counting;

impl Counting {
    #[inline]
    fn charge(size: usize) {
        // `try_with`: during thread teardown the slot is gone, and a
        // panic from inside the allocator would be unrecoverable.
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        let _ = ALLOC_BYTES.try_with(|c| c.set(c.get() + size));
        if size <= INTERNAL_THRESHOLD {
            let _ = SMALL_LIVE.try_with(|c| {
                let live = c.get() + size;
                c.set(live);
                let _ = SMALL_PEAK.try_with(|p| p.set(p.get().max(live)));
            });
        }
    }
    #[inline]
    fn release(size: usize) {
        if size <= INTERNAL_THRESHOLD {
            let _ = SMALL_LIVE.try_with(|c| c.set(c.get().saturating_sub(size)));
        }
    }
}

// SAFETY: every method forwards to `System` unchanged; the counters are
// thread-local `Cell` updates that cannot affect the pointer returned.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        Self::charge(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        Self::release(layout.size());
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        // A realloc is an allocation as far as the board's heap lock is
        // concerned, so count it as one.
        Self::release(layout.size());
        Self::charge(new_size);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// `(allocations, bytes)` on this thread since it started. Both
/// monotonic; a caller reads the pair around a region and reports the
/// difference.
fn alloc_census() -> (usize, usize) {
    (ALLOCS.with(|c| c.get()), ALLOC_BYTES.with(|c| c.get()))
}

fn slot_audio() -> Option<Vec<i16>> {
    let path = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; rx::SLOT_SAMPLES];
    let copy = raw.len().min(rx::SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

/// What one slot produced, and what it cost to produce.
struct SlotRun {
    messages: Vec<String>,
    cands: usize,
    /// Allocations and bytes inside the candidate loop only — the
    /// figure L1 exists to drive to zero.
    cand_allocs: usize,
    cand_bytes: usize,
    /// The candidate loop's allocations split by stage — `[ddc,
    /// search, tail]`. Which stage a hoist has to target is not
    /// something the byte total answers: the three 40 KB buffers
    /// dominate the bytes and the small ones dominate the count.
    stage_allocs: [usize; 3],
    stage_bytes: [usize; 3],
    /// The same, for the capture side: everything before the window
    /// closes.
    capture_allocs: usize,
    capture_bytes: usize,
}

/// `SlotAccum::push_with_rows` + `decode_slot`, in one function.
fn run_slot(audio: &[i16]) -> SlotRun {
    // ── capture side ────────────────────────────────────────────────
    let c0 = alloc_census();
    let (savg, half) = rx::capture(audio);
    let c1 = alloc_census();

    // ── post-close, candidate-independent ───────────────────────────
    let cands = rx::coarse(&savg);
    let refs = Ft4CoarsePhasors::new::<Ft4>();

    // ── the candidate loop ──────────────────────────────────────────
    let c2 = alloc_census();
    let mut messages: Vec<String> = Vec::new();
    let mut stage_allocs = [0usize; 3];
    let mut stage_bytes = [0usize; 3];
    for cand in &cands {
        let text = decode_candidate(&half, cand, &refs, &mut stage_allocs, &mut stage_bytes);
        if let Some(text) = text
            && !messages.contains(&text)
        {
            messages.push(text);
        }
    }
    let c3 = alloc_census();

    SlotRun {
        messages,
        cands: cands.len(),
        cand_allocs: c3.0 - c2.0,
        cand_bytes: c3.1 - c2.1,
        stage_allocs,
        stage_bytes,
        capture_allocs: c1.0 - c0.0,
        capture_bytes: c1.1 - c0.1,
    }
}

/// `ft4_rx::decode_candidate`, call for call.
fn decode_candidate(
    half: &[f32],
    cand: &SyncCandidate,
    refs: &Ft4CoarsePhasors,
    stage_allocs: &mut [usize; 3],
    stage_bytes: &mut [usize; 3],
) -> Option<String> {
    let mut charge = |k: usize, from: (usize, usize)| {
        let now = alloc_census();
        stage_allocs[k] += now.0 - from.0;
        stage_bytes[k] += now.1 - from.1;
        now
    };
    let t = alloc_census();
    let mut cd0 = mfsk_core::ft4::ddc::candidate_baseband_half(half, cand.freq_hz);
    rx::rms_normalise(&mut cd0);
    let t = charge(0, t);
    let s2 = mfsk_core::engine::sync2d::ft4_sync_search_window_with::<Ft4>(
        &cd0,
        cand,
        rx::WSJTX_WINDOW.0,
        rx::WSJTX_WINDOW.1,
        refs,
    );
    let t = charge(1, t);
    let r = mfsk_core::engine::pipeline::process_candidate_precomputed::<Ft4>(
        cand,
        // FT4's `snr_db` reads the coarse candidate score, not a
        // wide-band cache, so the board passes an empty slice here and
        // so does this.
        &[],
        &mfsk_core::ft4::decode::FT4_DOWNSAMPLE,
        mfsk_core::engine::pipeline::DecodeDepth::EMBEDDED,
        mfsk_core::engine::pipeline::DecodeStrictness::Normal,
        &[],
        mfsk_core::engine::equalize::EqMode::Off,
        rx::SYNC_Q_MIN,
        (cd0, s2.freq_hz, s2.i0, s2.score),
        false,
        false,
    );
    // Charged before the `?`: a candidate that fails still paid for
    // everything the tail allocated on the way there, and those are
    // the majority on a real band.
    charge(2, t);
    let m77: [u8; 77] = r?.message77().try_into().ok()?;
    mfsk_core::msg::wsjt77::unpack77(&m77)
}

fn require_corpus() -> bool {
    std::env::var("MFSK_REQUIRE_CORPUS").is_ok()
}

/// The mirror's own gate: the receiver's arrangement decodes the golden,
/// and the candidate loop's allocation cost is on the record.
///
/// The decode count is an equality, not a floor. A front-end or
/// scheduling change that moves it is exactly what this file exists to
/// catch, and "more" is as much a change as "fewer" — a phantom is a
/// decode too.
#[test]
fn mirror_decodes_the_golden_and_reports_what_a_candidate_allocates() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing"
        );
        common::skip_or_fail("FT4 golden recording");
        return;
    };

    let run = run_slot(&audio);

    let per_cand_allocs = run.cand_allocs as f64 / run.cands.max(1) as f64;
    let per_cand_bytes = run.cand_bytes as f64 / run.cands.max(1) as f64;
    eprintln!(
        "ft4 mirror: {} candidates, {} decodes\n  \
         capture side : {} allocations, {} B\n  \
         candidate loop: {} allocations, {} B  ({:.1}/cand, {:.0} B/cand)\n    \
         ddc    {:.1}/cand, {:.0} B\n    \
         search {:.1}/cand, {:.0} B\n    \
         tail   {:.1}/cand, {:.0} B",
        run.cands,
        run.messages.len(),
        run.capture_allocs,
        run.capture_bytes,
        run.cand_allocs,
        run.cand_bytes,
        per_cand_allocs,
        per_cand_bytes,
        run.stage_allocs[0] as f64 / run.cands.max(1) as f64,
        run.stage_bytes[0] as f64 / run.cands.max(1) as f64,
        run.stage_allocs[1] as f64 / run.cands.max(1) as f64,
        run.stage_bytes[1] as f64 / run.cands.max(1) as f64,
        run.stage_allocs[2] as f64 / run.cands.max(1) as f64,
        run.stage_bytes[2] as f64 / run.cands.max(1) as f64,
    );
    let mut sorted = run.messages.clone();
    sorted.sort();
    for m in &sorted {
        eprintln!("    {m}");
    }

    assert_eq!(
        run.messages.len(),
        11,
        "the receiver's arrangement decodes 11 on this recording; got {:?}",
        sorted
    );
}

/// Where the tail's allocations actually are.
///
/// The per-stage split says the tail is ~80 % of the candidate loop's
/// allocation count while being under half its bytes, which is the
/// shape of many small buffers rather than a few large ones. This
/// breaks it down over one candidate by calling the tail's public
/// pieces directly, so a hoist can be aimed rather than guessed at.
///
/// Diagnostic, not a gate: it asserts only that the pieces it can call
/// account for part of the whole, because the rest of the ladder is
/// inside `process_candidate_precomputed` and has no public seam.
#[test]
fn mirror_tail_allocation_breakdown() {
    use mfsk_core::engine::llr::symbol_spectra;
    use mfsk_core::engine::sync::fine_sync_power_per_block;
    use mfsk_core::engine::sync2d::freq_shift_cd0_into;
    use mfsk_core::engine::{FrameLayout, ModulationParams};

    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };

    // Same capture side as `run_slot`, then one candidate.
    let (savg, half) = rx::capture(&audio);
    let cands = rx::coarse(&savg);
    let refs = Ft4CoarsePhasors::new::<Ft4>();
    let cand = cands.first().expect("the golden yields candidates");

    let mut cd0 = mfsk_core::ft4::ddc::candidate_baseband_half(&half, cand.freq_hz);
    rx::rms_normalise(&mut cd0);
    let s2 = mfsk_core::engine::sync2d::ft4_sync_search_window_with::<Ft4>(
        &cd0,
        cand,
        rx::WSJTX_WINDOW.0,
        rx::WSJTX_WINDOW.1,
        &refs,
    );

    let ds_rate = 12_000.0 / Ft4::NDOWN as f32;
    let mut shift_buf: Vec<Complex<f32>> = Vec::new();

    let a = alloc_census();
    freq_shift_cd0_into(&cd0, s2.freq_hz - cand.freq_hz, ds_rate, &mut shift_buf);
    let b = alloc_census();
    let cs = symbol_spectra::<Ft4>(&shift_buf, s2.i0);
    let c = alloc_census();
    let per_block = fine_sync_power_per_block::<Ft4>(&shift_buf, s2.i0);
    let d = alloc_census();
    let llr = mfsk_core::engine::llr::compute_llr_fast::<Ft4, f32>(&cs);
    let e = alloc_census();

    eprintln!(
        "ft4 tail, one candidate:\n  \
         freq_shift_cd0_into      {} allocations, {} B\n  \
         symbol_spectra           {} allocations, {} B\n  \
         fine_sync_power_per_block {} allocations, {} B\n  \
         compute_llr_fast         {} allocations, {} B",
        b.0 - a.0,
        b.1 - a.1,
        c.0 - b.0,
        c.1 - b.1,
        d.0 - c.0,
        d.1 - c.1,
        e.0 - d.0,
        e.1 - d.1,
    );
    assert_eq!(cs.len(), (Ft4::N_SYMBOLS * Ft4::NTONES) as usize);
    assert!(!per_block.is_empty());
    assert_eq!(llr.llra.len(), 174);
}

/// What would a boxcar front end decode on the real recording?
///
/// The per-candidate DDC is 83 ms of the board's 221 ms candidate and
/// the proposal on the table is to replace its 101 + 263 taps with a
/// mix-and-bin pass costing a sixteenth of the arithmetic. Whether that
/// is worth designing depends on one number — what it decodes — and
/// the WSJT-X golden is the closest thing this crate has to a real
/// band: fourteen signals at the frequencies a real slot put them at.
///
/// Diagnostic, not a gate. It prints both sides and asserts only that
/// the FIR arm is unchanged, because the boxcar arm is the thing being
/// decided rather than a contract.
#[test]
fn what_a_boxcar_front_end_would_decode_on_the_golden() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };

    let fir = rx::run_slot_with(&audio, rx::Variant::SHIPPED);
    let boxcar = rx::run_slot_with(&audio, rx::Variant::BOXCAR);

    let mut only_fir: Vec<&String> = fir.iter().filter(|m| !boxcar.contains(m)).collect();
    let mut only_box: Vec<&String> = boxcar.iter().filter(|m| !fir.contains(m)).collect();
    only_fir.sort();
    only_box.sort();

    eprintln!(
        "\nft4 golden, front ends compared:\n  \
         FIR 101+263 : {} decodes\n  \
         boxcar /9   : {} decodes\n  \
         lost by boxcar : {:?}\n  \
         gained by boxcar: {:?}",
        fir.len(),
        boxcar.len(),
        only_fir,
        only_box,
    );

    assert_eq!(
        fir.len(),
        11,
        "the FIR arm is the control and must not move"
    );
}

/// The stepping rotator is the transcendental one.
///
/// `boxcar_producer` steps a phasor where `boxcar_producer_reference`
/// calls `cos`/`sin` per sample, so that a timing comparison measures
/// the filter rather than the transcendentals. Same discipline as
/// `sync2d`'s own `the_rotator_tracks_the_exact_rotation`: the claim
/// is bounded, not asserted.
#[test]
fn the_stepping_boxcar_matches_the_transcendental_one() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let (_savg, half) = rx::capture(&audio);
    for f0 in [300.0f32, 1_000.0, 2_600.0] {
        let fast = rx::boxcar_producer(&half, f0);
        let slow = rx::boxcar_producer_reference(&half, f0);
        assert_eq!(fast.len(), slow.len());
        let scale = slow
            .iter()
            .map(|c| c.norm())
            .fold(0.0f32, f32::max)
            .max(1e-9);
        let worst = fast
            .iter()
            .zip(&slow)
            .map(|(a, b)| (a - b).norm())
            .fold(0.0f32, f32::max);
        // Measured 2026-09-21 over the golden's 40 609 half-rate
        // samples: 1.0e-3, 3.5e-3, 2.2e-3 at 300 / 1000 / 2600 Hz — it
        // does not fall off monotonically with frequency, so the bound
        // is the worst of the three with headroom rather than anything
        // derived. That is the rotator's *phase* drift, which
        // `Mixer`'s renormalisation bounds in magnitude and not in
        // phase; at ~−50 dB it is two orders below the 0.29 dB of
        // threshold this producer actually costs, which is why the
        // decode counts either side of the switch are identical.
        eprintln!(
            "  f0={f0:>6}: stepping vs transcendental {:.2e}",
            worst / scale
        );
        assert!(
            worst / scale < 8e-3,
            "f0={f0}: stepping and transcendental differ by {:.2e} of full scale",
            worst / scale
        );
    }
}

/// What the cheaper front end actually saves — **on the host**, which
/// is the wrong machine and says so.
///
/// The board's FIR runs its taps through `dsps_dotprod_f32_aes3` at a
/// measured 2.18 cycles per multiply-add (`FT4_BENCHMARK.md` §47); the
/// host runs the portable dot product. So this comparison **flatters
/// the boxcar**, and the ratio here is an upper bound on the one that
/// matters. It is worth having anyway: it is the cheap check that the
/// saving is the order of magnitude the arithmetic claims, before
/// anything is wired into the receiver and flashed.
///
/// `#[ignore]` — timing, so it is noise on a shared runner.
#[test]
#[ignore = "timing — run with --ignored"]
fn what_the_boxcar_front_end_saves_on_the_host() {
    use std::time::Instant;

    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let (savg, half) = rx::capture(&audio);
    let cands = rx::coarse(&savg);
    let f0 = cands[0].freq_hz;

    const REPS: u32 = 300;
    let mut timings = Vec::new();
    for (name, produce) in [
        ("FIR 101+263", rx::fir_producer as rx::Producer),
        ("boxcar /9", rx::boxcar_producer as rx::Producer),
    ] {
        // One untimed call so neither arm pays for a cold cache.
        let mut sink = produce(&half, f0);
        let t0 = Instant::now();
        for _ in 0..REPS {
            sink = produce(&half, f0);
        }
        let per_call = t0.elapsed().as_secs_f64() * 1e6 / REPS as f64;
        core::hint::black_box(&sink);
        timings.push((name, per_call));
    }

    // And the whole slot, which is what the budget is spent in — for
    // each half of the cheaper front end and for both together.
    let mut slots = Vec::new();
    for (name, v) in [
        ("shipped", rx::Variant::SHIPPED),
        ("boxcar", rx::Variant::BOXCAR),
        ("binned search", rx::Variant::BINNED_SEARCH),
        ("both", rx::Variant::BOTH),
    ] {
        let t0 = Instant::now();
        let out = rx::run_slot_with(&audio, v);
        slots.push((name, t0.elapsed().as_secs_f64() * 1e3, out.len()));
    }

    eprintln!("\nft4 front end, host timing ({} candidates):", cands.len());
    for (name, us) in &timings {
        eprintln!("  producer  {name:<14} {us:>9.1} us/candidate");
    }
    eprintln!(
        "  producer  ratio          {:>9.1}x",
        timings[0].1 / timings[1].1
    );
    for (name, ms, n) in &slots {
        eprintln!(
            "  whole slot {name:<13} {ms:>9.1} ms   {n} decodes   {:>5.2}x",
            slots[0].1 / ms
        );
    }
    eprintln!();
}

/// What snapping each candidate's carrier to the periodogram bin costs.
///
/// The precondition for building basebands during capture: a
/// provisional candidate list interpolates each peak slightly
/// differently from the final one, so the two can only share a
/// baseband if both are mixed at the same carrier — the bin centre.
/// The Δt/Δf search still sweeps ±12 Hz from there, which is what
/// should make it cheap. Diagnostic: prints both arms.
#[test]
fn what_snapping_the_carrier_to_the_bin_costs_on_the_golden() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let shipped = rx::run_slot_with(&audio, rx::Variant::SHIPPED);
    let snapped = rx::run_slot_with(&audio, rx::Variant::SNAPPED);
    let lost: Vec<&String> = shipped.iter().filter(|m| !snapped.contains(m)).collect();
    let gained: Vec<&String> = snapped.iter().filter(|m| !shipped.contains(m)).collect();
    eprintln!(
        "\nft4 golden, carrier snapped to the {:.3} Hz bin:\n  \
         shipped {} decodes, snapped {}\n  lost {:?}\n  gained {:?}",
        rx::COARSE_BIN_HZ,
        shipped.len(),
        snapped.len(),
        lost,
        gained
    );
    assert_eq!(shipped.len(), 11, "the control arm must not move");
}

/// How early is the candidate list known?
///
/// Building each candidate's baseband during capture needs the
/// candidates before capture closes. The periodogram accumulates as
/// audio arrives, so a *provisional* list can be read off it at any
/// point — this measures how much of the final list, and in particular
/// how much of the part that decodes, a provisional list taken at each
/// time already holds. Candidates are compared by snapped bin, since
/// that is what a shared baseband would be keyed on.
///
/// Diagnostic: prints the table.
#[test]
fn how_early_the_candidate_list_is_known() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let bin = |f: f32| (f / rx::COARSE_BIN_HZ).round() as i32;

    let (savg, half) = rx::capture(&audio);
    let finals = rx::coarse(&savg);
    let refs = Ft4CoarsePhasors::new::<Ft4>();
    // Which final candidates decode — the ones a provisional list must
    // not miss.
    let decoding: Vec<i32> = finals
        .iter()
        .filter(|c| rx::decode_candidate_with(&half, c, &refs, rx::Variant::SNAPPED).is_some())
        .map(|c| bin(c.freq_hz))
        .collect();
    let final_bins: Vec<i32> = finals.iter().map(|c| bin(c.freq_hz)).collect();

    eprintln!(
        "\nft4 golden: {} final candidates, {} of them decode",
        final_bins.len(),
        decoding.len()
    );
    eprintln!(
        "  {:<14} {:>7} {:>6} {:>12} {:>14} {:>10}",
        "at", "", "prov", "finals held", "decoders held", "spurious"
    );
    // At each Costas block's end for a nominally-timed frame, then the
    // close — protocol-defined points rather than round seconds.
    let points: Vec<(&str, usize)> = vec![
        ("Costas A end", rx::costas_block_end_samples(0)),
        ("Costas B end", rx::costas_block_end_samples(1)),
        ("Costas C end", rx::costas_block_end_samples(2)),
        ("Costas D end", rx::costas_block_end_samples(3)),
        ("capture close", rx::CAPTURE_CLOSE_SAMPLES),
    ];
    for (name, n) in points {
        let n = n.min(rx::CAPTURE_CLOSE_SAMPLES);
        let mut b = mfsk_core::engine::ft4_coarse::Ft4SavgBuilder::new(n);
        let mut fed = 0usize;
        while fed < n {
            let take = rx::BLOCK.min(n - fed);
            b.push_with_rows(&audio[fed..fed + take], &mut |_row| {});
            fed += take;
        }
        let prov: Vec<i32> = rx::coarse(&b.finish())
            .iter()
            .map(|c| bin(c.freq_hz))
            .collect();
        let held = final_bins.iter().filter(|b| prov.contains(b)).count();
        let held_dec = decoding.iter().filter(|b| prov.contains(b)).count();
        let spurious = prov.iter().filter(|b| !final_bins.contains(b)).count();
        eprintln!(
            "  {name:<14} {:>5.3} s {:>6} {:>7}/{:<4} {:>9}/{:<4} {:>10}",
            n as f32 / 12_000.0,
            prov.len(),
            held,
            final_bins.len(),
            held_dec,
            decoding.len(),
            spurious
        );
    }
    eprintln!();
}

/// The pipelined receiver decodes exactly what the snapped one does.
///
/// Building each baseband during capture is only a scheduling change
/// if it is bit-identical: same snapped carrier, same samples, and
/// `FirStage`'s block independence doing the rest. This is the
/// assertion that makes it one, across provisional times from early
/// (where the provisional list misses a candidate and the close has to
/// build it) to late (where it is already the final list).
#[test]
fn pipelined_ddc_decodes_exactly_what_the_snapped_receiver_does() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let mut want = rx::run_slot_with(&audio, rx::Variant::SNAPPED);
    want.sort();
    eprintln!("\nft4 golden, DDC moved into capture time:");
    for (name, n) in [
        ("Costas B end", rx::costas_block_end_samples(1)),
        ("Costas C end", rx::costas_block_end_samples(2)),
        ("Costas D end (shipped)", rx::provisional_samples()),
    ] {
        let (mut got, st) = rx::run_slot_pipelined(&audio, n);
        got.sort();
        eprintln!(
            "  {name:<24} {:.3} s: reused {:>2}, built after close {:>2}, \
             wasted {:>2}, decodes {}",
            n as f32 / 12_000.0,
            st.reused,
            st.fresh,
            st.wasted,
            got.len()
        );
        assert_eq!(got, want, "provisional at {name} changed the decodes");
    }
    eprintln!();
}

/// The coarse sweep run incrementally lands where the one-shot search
/// does, on every golden candidate and however the baseband arrives.
///
/// The one place it can differ is the scale: it scores the raw `cd0`
/// and the one-shot search the normalised one, so a coarse tie inside
/// rounding could go the other way. The fine pass runs on the
/// normalised `cd0` either way, so matching coarse winners means a
/// bit-identical result — which is what this asserts, score included.
#[test]
fn ft4_incremental_sweep_matches_the_one_shot_search() {
    use mfsk_core::engine::sync2d::{Ft4CoarseSweep, Ft4SweepScratch, ft4_sync_search_window_with};
    use mfsk_core::ft4::ddc::{CD0_LEN, candidate_baseband_half};
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let (savg, half) = rx::capture(&audio);
    let refs = Ft4CoarsePhasors::new::<Ft4>();
    let mut scratch = Ft4SweepScratch::new::<Ft4>();
    let cands = rx::coarse(&savg);
    assert!(!cands.is_empty());
    for c in &cands {
        let cand = SyncCandidate {
            freq_hz: rx::snap_to_bin(c.freq_hz),
            ..*c
        };
        let raw = candidate_baseband_half(&half, cand.freq_hz);
        let mut norm = raw.clone();
        rx::rms_normalise(&mut norm);
        let want = ft4_sync_search_window_with::<Ft4>(
            &norm,
            &cand,
            rx::WSJTX_WINDOW.0,
            rx::WSJTX_WINDOW.1,
            &refs,
        );
        // 1: every sample its own call; 29: roughly one UAC block's
        // worth of `cd0`; the whole: the one-shot shape.
        for chunk in [1usize, 29, CD0_LEN] {
            let mut sw = Ft4CoarseSweep::new(rx::WSJTX_WINDOW.0, rx::WSJTX_WINDOW.1, CD0_LEN, 0);
            let mut n = 0;
            // Stop short of the flush, as the board does at the close.
            while n < 4_300 {
                n = (n + chunk).min(4_300);
                sw.advance::<Ft4>(&raw[..n], CD0_LEN, &mut scratch, &refs, usize::MAX);
            }
            sw.complete::<Ft4>(&raw, &mut scratch, &refs);
            let got = sw.fine::<Ft4>(&norm, &cand, &mut scratch, &refs);
            assert_eq!(
                (got.i0, got.freq_hz.to_bits(), got.score.to_bits()),
                (want.i0, want.freq_hz.to_bits(), want.score.to_bits()),
                "{:.1} Hz, chunk {chunk}",
                cand.freq_hz
            );
        }
    }
}

/// The pipelined receiver with the coarse sweep moved into capture too
/// decodes exactly what the snapped receiver does, and says how much of
/// the sweep the capture can have paid for by the close.
#[test]
fn pipelined_sweep_decodes_exactly_what_the_snapped_receiver_does() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let mut want = rx::run_slot_with(&audio, rx::Variant::SNAPPED);
    want.sort();
    let (mut got, st) = rx::run_slot_pipelined_with(&audio, rx::provisional_samples(), true);
    got.sort();
    eprintln!(
        "\nft4 golden, DDC + coarse sweep in capture: reused {}, fresh {}, \
         sweep readable by the close {} of {} block-cells ({:.1} %), decodes {}\n",
        st.reused,
        st.fresh,
        st.sweep_done,
        st.sweep_total,
        100.0 * st.sweep_done as f64 / st.sweep_total.max(1) as f64,
        got.len()
    );
    assert_eq!(got, want);
}

/// How much internal DRAM the pipelined receiver would ask for.
///
/// Building every candidate's baseband during capture keeps twelve
/// `CandidateDdc`s alive at once where today's loop holds at most two
/// (one per core). Each carries FIR histories and phase-shifted tap
/// tables, all under the board's 2 048-byte threshold, so on a CoreS3
/// they would all *prefer* internal DRAM — of which there is ~31 KB
/// largest-block once WiFi is up, and which has already run out once
/// and taken WiFi with it. This measures the peak of those small
/// allocations under each arrangement, on the host, before anything is
/// flashed.
#[test]
fn what_the_pipelined_receiver_asks_of_internal_dram() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the golden is missing"
        );
        return;
    };
    let reset = || {
        SMALL_LIVE.with(|c| c.set(0));
        SMALL_PEAK.with(|c| c.set(0));
    };
    // Small allocations still live from earlier work would count
    // against the arm that runs next, so each arm is measured from a
    // clean live count; `saturating_sub` absorbs frees of blocks
    // allocated before the reset.
    reset();
    let _ = rx::run_slot_with(&audio, rx::Variant::SNAPPED);
    let serial_peak = SMALL_PEAK.with(|c| c.get());
    reset();
    let _ = rx::run_slot_pipelined(&audio, rx::provisional_samples());
    let piped_peak = SMALL_PEAK.with(|c| c.get());
    reset();
    let _ = rx::run_slot_pipelined_with(&audio, rx::provisional_samples(), true);
    let swept_peak = SMALL_PEAK.with(|c| c.get());
    eprintln!(
        "\nft4 small-allocation (<= {INTERNAL_THRESHOLD} B) peak live bytes:\n  \
         candidates one at a time  {serial_peak:>8} B\n  \
         all built during capture  {piped_peak:>8} B\n  \
         + coarse sweep in capture {swept_peak:>8} B (one worker's scratch)\n"
    );
}

/// The constants above are a copy of `ft4_rx.rs`'s, and a copy rots.
///
/// This cannot import them — `embedded-shared` is outside this
/// workspace and builds only under the `+esp` toolchain — so it checks
/// them against the crate-side facts they are derived from instead.
#[test]
fn mirror_constants_match_the_receiver() {
    use mfsk_core::engine::{FrameLayout, ModulationParams};

    // 7.5 s at 12 kHz.
    assert_eq!(rx::SLOT_SAMPLES, 90_000);
    // The window has to cover the frame's own length plus the +1.0 s of
    // DT `WSJTX_WINDOW` allows plus the DDC's group delay, and stop
    // before the slot ends — that is the whole reason it is 81 300 and
    // not 90 000.
    const { assert!(rx::CAPTURE_CLOSE_SAMPLES < rx::SLOT_SAMPLES) };
    let frame_samples = (Ft4::N_SYMBOLS as usize) * (Ft4::NSPS as usize);
    assert!(
        rx::CAPTURE_CLOSE_SAMPLES > frame_samples + 12_000,
        "the capture window must still hold a frame at DT = +1.0 s"
    );
    // `WSJTX_WINDOW` is in `cd0` samples; upstream's own bounds are
    // `ibmin = -344`, `ibmax = 1012` (`lib/ft4_decode.f90:241-242`).
    assert_eq!(rx::WSJTX_WINDOW, (-344, 1012));
    // The search must stay inside the buffer the DDC produces.
    let cd0_len = mfsk_core::ft4::ddc::CD0_LEN as i32;
    let frame_cd0 = (Ft4::N_SYMBOLS * Ft4::NSPS / Ft4::NDOWN) as i32;
    assert!(rx::WSJTX_WINDOW.1 + frame_cd0 <= cd0_len);
}
