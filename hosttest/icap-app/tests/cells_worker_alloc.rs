// SPDX-License-Identifier: GPL-3.0-only
//! The cells flow's per-candidate step allocates no block, no `Vec` of candidates and no BP scratch.
//!
//! A board runs `CellsWorker::decode_candidate` from a work-stealing loop over memory it owns (the staging block in internal DRAM, the BP scratch, the tables):
//! a `Box` of the 79 x 8 block (5 056 B, which lands in PSRAM where BP reads it slowly) and a BP scratch (about 12 KB) per attempt is allocation churn in a heap whose
//! fragmentation has cost a mode before (#260). Counted with the crate's counting allocator over the 116 candidates of a noise-only slot (none decodes), OSD off (the
//! board ships without it; the OSD fallback makes a BP scratch of its own, a known cost not covered here).
//!
//! **What remains**: about one allocation an attempt of at most 696 B, the LLR scratch of the shared `engine::llr` (a `Vec`, freed at once; under the 2 KB of
//! `SPIRAM_MALLOC_ALWAYSINTERNAL`, so it lands in internal DRAM and is the same one the board's stage 3 makes today). Making `engine::llr` allocation-free is a separate item.
#![cfg(feature = "fixed-point")]

use icap_app_hosttest::heap;
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CellTables, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch, coarse_sync_gated_cells,
};

fn noise(n: usize, sd: f32, seed: u64) -> Vec<i16> {
    let mut st = seed;
    (0..n)
        .map(|_| {
            let mut g = 0.0f32;
            for _ in 0..12 {
                st = st
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                g += ((st >> 40) as f32) / (1u64 << 24) as f32;
            }
            ((g - 6.0) * sd).round().clamp(-32768.0, 32767.0) as i16
        })
        .collect()
}

/// **Fails if** an attempt makes a block as large as the staging block (the BP scratch is larger still) (the old `attempt` made a 5 056 B `Box`, a `Vec` and a BP scratch of about 12 KB each
/// time), or more than two allocations an attempt.
#[test]
fn an_attempt_allocates_no_block_and_no_scratch() {
    let slot = noise(15 * 8000, 1500.0, 5);
    let (cands, cells) = coarse_sync_gated_cells(
        &mut SliceSource(&slot),
        Ft8Rate::Hz8000,
        &GatedSearch::new(200.0, 3000.0, 1.3, 200),
        1,
    )
    .expect("8 kHz fixed-point");
    assert!(cands.len() >= 100, "{} candidates", cands.len());
    let tables = CellTables::new();
    let mut cs = Box::new([[Cmplx::<f32>::default(); 8]; 79]);
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(&mut cs, &mut bp, &tables);
    let depth = DecodeDepth::EMBEDDED; // OSD off
    // one warm-up call: lazily initialised statics, the first touch of the scratch
    let _ = worker.decode_candidate(&cells, &cands[0].0, depth, 0.25);
    heap::reset_alloc_max();
    let before = heap::alloc_count();
    let mut decoded = 0;
    for (c, _) in &cands {
        // the retry is made for a candidate whose estimate is 0.25 cell off: up to two attempts a candidate
        if worker.decode_candidate(&cells, c, depth, 0.25).is_some() {
            decoded += 1;
        }
    }
    let made = heap::alloc_count() - before;
    let largest = heap::alloc_max();
    assert_eq!(decoded, 0, "a noise-only slot decodes nothing");
    assert!(
        largest < core::mem::size_of::<[[Cmplx<f32>; 8]; 79]>(),
        "an allocation of {largest} B: as large as the staging block"
    );
    assert!(
        made <= 2 * 2 * cands.len(),
        "{made} allocations over {} candidates",
        cands.len()
    );
}

/// Measurement of the **P0 window configuration** (`CellsOptions::p0_window()`: gate, seeds `a` and `d`, the OSD deferred and capped at 6) on a real busy recording: allocations over
/// the 30 candidates and the OSD pass, and the largest block. The figures are printed (`--nocapture`) and bound loosely: the LLR scratch of `engine::llr` is a small `Vec` an attempt, and
/// the embedded OSD works in the worker's own memory. **Fails if** an allocation reaches the staging block's size, or a candidate costs more than 16 allocations (measured 2026-11-27: **312 allocations over 30
/// candidates, 10.4 a candidate, 119 937 B in all, the largest block 696 B**: every block under the 2 KB of `SPIRAM_MALLOC_ALWAYSINTERNAL`, so internal DRAM, freed at once; a BP scratch or an OSD scratch made per attempt would show as a block of 12 KB or more).
#[test]
fn the_p0_window_round_allocates_small_blocks_only() {
    let wav =
        icap_app_hosttest::wav::load_i16(icap_app_hosttest::profile::asset_path("191111_110130"));
    let slot = icap_app_hosttest::resample::resample(&wav, 12_000, 8_000);
    let (cands, cells) = coarse_sync_gated_cells(
        &mut SliceSource(&slot),
        Ft8Rate::Hz8000,
        &GatedSearch::new(200.0, 3000.0, 1.3, 30),
        1,
    )
    .expect("8 kHz fixed-point");
    let options = mfsk_core::ft8::decode_block::CellsOptions::p0_window();
    let tables = CellTables::new();
    let mut cs = Box::new([[Cmplx::<f32>::default(); 8]; 79]);
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(&mut cs, &mut bp, &tables).with_options(&options);
    let depth = options.depth();
    let mut warm = Vec::new();
    worker.decode_round_osd(&cells, &cands, 30, depth, 0.25, &mut warm, 6); // warm-up: lazy statics
    heap::reset_alloc_max();
    let (before, bytes_before) = (heap::alloc_count(), heap::alloc_bytes());
    let mut known = Vec::new();
    worker.decode_round_osd(&cells, &cands, 30, depth, 0.25, &mut known, 6);
    let (made, bytes, largest) = (
        heap::alloc_count() - before,
        heap::alloc_bytes() - bytes_before,
        heap::alloc_max(),
    );
    println!(
        "P0 window round, 191111_110130: {} decodes, {made} allocations ({:.2} a candidate), {bytes} B in all, largest block {largest} B",
        known.len(),
        made as f32 / cands.len() as f32
    );
    assert!(!known.is_empty());
    assert!(
        largest < core::mem::size_of::<[[Cmplx<f32>; 8]; 79]>(),
        "an allocation of {largest} B"
    );
    assert!(
        made <= 16 * cands.len(),
        "{made} allocations over {} candidates",
        cands.len()
    );
}
