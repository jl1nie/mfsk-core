//! The reusable "decode one WSPR slot" core, extracted out of
//! [`super::wspr_bench`].
//!
//! `wspr_bench` is a bench harness — timing arms, memory-placement
//! experiments, a baked-in golden recording — built around one real
//! piece of decode logic: [`run_scan`], `decode_scan_inner`'s
//! three-pass sequence (early passes 0/1 with drift refine, then the
//! deep pass-2 ladder), dual-core-dispatched via
//! [`crate::wspr_dual_core`]. That function was never callable from
//! anywhere except the bench itself — moved here, unchanged, so a
//! production app (`wspr_app`, CoreS3 WSPR receiver UI) can call the
//! same tested decode path over its own caller-owned baseband buffers
//! instead of re-deriving it.
//!
//! `wspr_bench.rs` now imports everything below rather than defining
//! it locally — zero behaviour change, confirmed by rebuilding
//! `wspr-bench` and comparing its own regression coverage.
//!
//! What stayed in `wspr_bench.rs`: the bench-only instrumentation
//! wrapper ([`super::wspr_bench`]'s `arm`), the PSRAM/SRAM
//! memory-placement experiments (`alloc_caps`/`scan_body`), the
//! oscillator-table bandwidth arm, and the baked-golden-asset loading
//! entry point used by the bench's own `main`. A caller that just
//! wants "decode this slot's baseband" needs [`run_scan`] and nothing
//! else here beyond the types it returns.

extern crate alloc;

use alloc::vec::Vec;

use mfsk_core::fec::conv::fano::instrument as fano_hist;
use mfsk_core::wspr::coarse_baseband::{coarse_baseband, BasebandCandidate};
use mfsk_core::wspr::decode::{
    decode_at_baseband, rank_pass2_candidates, WsprCallsignTable, WsprResult,
};
use mfsk_core::wspr::demod::TONE_SPACING_HZ;
use mfsk_core::wspr::instrument;
use mfsk_core::wspr::subtract::subtract_signal_baseband;

use crate::wspr_dual_core;

/// 46080 — `wspr::baseband::NFFT2`, the fixed baseband length. A
/// caller sizing its own `idat`/`qdat` buffers for [`run_scan`] wants
/// exactly this many `f32` samples per channel.
pub const NBB: usize = 46_080;
/// `NEGATIVE_DT_PAD_SEC × 12 000`, in audio samples. `coarse_baseband`
/// takes the pad in the audio time base, not the baseband one.
pub const PAD_AUDIO: usize = 36_000;
pub const SAMPLE_RATE: u32 = 12_000;
/// Matches `wspr_wsjtx_sample_recall_vs_golden` and
/// `WSPR_BENCHMARK.md`, so the device number is comparable to the
/// published host one. (`SearchParams::default()` is 200.)
pub const MAX_CANDIDATES: usize = 100;
/// `wsprd.c:1002` — passes 0 and 1 search ±4 Hz of drift.
pub const EARLY_MAX_DRIFT: i32 = 4;
/// `wsprd.c:1005-1009` — the final pass does not.
pub const PASS2_MAX_DRIFT: i32 = 0;

/// WSPR slot period — the wall-clock deadline a production embedded
/// decoder is racing against, matching the protocol's own slot
/// length. Anchor for [`Pass2Budget::DeadlineDriven`], which targets
/// `SLOT_US - SPOT_RESERVE_US`, not the raw boundary.
pub const SLOT_US: i64 = 120_000_000;
/// Reserve carved out of the slot deadline for whatever the caller
/// does after `run_scan` returns (e.g. a spot-upload POST) —
/// `DeadlineDriven` targets finishing this far before the next slot
/// actually starts, not the boundary itself. Sizing beyond
/// "comfortably more than an upload needs with the link already up"
/// is not yet measured on real hardware — deliberately generous
/// rather than tuned, per the original bench's own measurement
/// (finishes at 94.3 s against a 110 s budget on the golden file).
pub const SPOT_RESERVE_US: i64 = 10_000_000;

/// How pass 2's failing-candidate ladder (up to 4 × 17 = 68 DT
/// peak-up × nblocks positions) is bounded. Threaded to
/// [`wspr_dual_core::pass2_split`]'s `deadline_us` parameter — see
/// that function's own doc comment for the exact semantics
/// (per-candidate, not a total-pass-2 budget: main and the worker
/// each get their own independent deadline check).
#[derive(Clone, Copy)]
#[allow(dead_code)] // both non-default variants are real, selectable configurations
pub enum Pass2Budget {
    /// No cutoff — wsprd-faithful, full ladder always.
    Unlimited,
    /// **Deadline priority.** Cuts pass 2's ladder off when
    /// `scan_start_us + SLOT_US - SPOT_RESERVE_US` arrives — the slot
    /// boundary less the spot-upload reserve — however much of that
    /// budget pass 0/1/coarse already consumed — guarantees on-time
    /// completion as the first priority, the way a production
    /// controller racing the next slot needs. Real-signal recall risk
    /// on pass 2's own candidates therefore VARIES run-to-run with how
    /// much time the earlier stages actually took, unlike
    /// `RecallPriority`'s fixed number.
    DeadlineDriven,
    /// **Recall priority.** A fixed per-candidate budget (µs) that
    /// ignores the slot deadline — if pass 0/1 already ran long, TOTAL
    /// can still exceed 120 s. Choose this over `DeadlineDriven` when
    /// losing a real weak decode is worse than running over the slot.
    RecallPriority(i64),
}

/// The default this crate ships with — see [`Pass2Budget`]'s own doc
/// comment for the tradeoff. A production `wspr_app` caller that
/// wants the recall-priority behaviour instead can bypass this and
/// pass its own deadline to `wspr_dual_core::pass2_split` directly by
/// forking [`run_scan`]'s body rather than overriding a constant —
/// noted here rather than silently assumed away.
pub const PASS2_BUDGET: Pass2Budget = Pass2Budget::DeadlineDriven;

pub fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Bytes of the calling task's stack never touched so far. FreeRTOS
/// task stacks come off the heap, so an overflow does not fault — it
/// silently scribbles on a neighbouring block's TLSF header and the
/// program dies later, inside an unrelated `free()`. Logging this
/// around `run_scan`'s own stages is what first caught that failure
/// mode on real hardware.
pub fn stack_headroom() -> u32 {
    // `uxTaskGetStackHighWaterMark` returns words on some ports and
    // bytes on ESP-IDF's; IDF documents it as bytes.
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

/// Abort loudly if the heap is already damaged, rather than at the
/// next unrelated `free()`.
pub fn assert_heap_ok(where_: &str) {
    let ok = unsafe { esp_idf_svc::sys::heap_caps_check_integrity_all(true) };
    if !ok {
        log::error!("HEAP CORRUPT after {where_}");
    }
}

/// Decode a baked baseband asset (`idat[NBB]` then `qdat[NBB]`, f32
/// LE) into caller-placed buffers.
///
/// Byte-wise rather than a transmute: `include_bytes!` gives a `&[u8]`
/// with 1-byte alignment, and Xtensa faults on an unaligned f32 load.
/// The cost is one pass over the buffer, meant to run outside any
/// timed region.
pub fn load_baseband(bin: &[u8], idat: &mut [f32], qdat: &mut [f32]) {
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
pub struct PassStats {
    pub label: &'static str,
    pub coarse_us: i64,
    pub decode_us: i64,
    pub subtract_us: i64,
    pub n_cand: usize,
    pub n_decoded: usize,
    pub counts: instrument::Counts,
    pub fano: fano_hist::Snapshot,
}

impl PassStats {
    pub fn total_us(&self) -> i64 {
        self.coarse_us + self.decode_us + self.subtract_us
    }

    pub fn log(&self) {
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
        log::info!(
            "          decode split: fano {} ms | bit_metrics {} ms | osd {} ms",
            c.fano_us / 1000,
            c.bit_metrics_us / 1000,
            c.osd_us / 1000,
        );
        log::info!(
            "          coarse split: build_spectro {} ms | refine {} ms",
            c.coarse_spectro_us / 1000,
            c.coarse_refine_us / 1000,
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
/// Mutates `idat`/`qdat` in place (the subtraction steps do), so a
/// caller that wants a clean baseband again afterward must reload it
/// — otherwise a second call would run against the first call's
/// residual, decoding different work rather than the same slot twice.
#[inline(never)]
pub fn run_scan(idat: &mut [f32], qdat: &mut [f32]) -> (Vec<PassStats>, Vec<WsprResult>) {
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

    // Stage 1 (`rank_pass2_candidates`): refine + `minsync2`-filter
    // every coarse candidate (cheap), rank by refined sync, keep the
    // top N. Stage 2 (`wspr_dual_core::pass2_split`): the expensive
    // Fano/OSD ladder, one survivor per core when the runtime guard
    // finds room — a static 1-1 split, not work-steal (see that
    // module's doc comment for why). Falls back to running both
    // survivors sequentially if there's no room.
    let t = now_us();
    let ranked = rank_pass2_candidates(idat, qdat, &cands2);
    let deadline_us = match PASS2_BUDGET {
        Pass2Budget::Unlimited => None,
        Pass2Budget::DeadlineDriven => Some(scan_start_us + SLOT_US - SPOT_RESERVE_US),
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
