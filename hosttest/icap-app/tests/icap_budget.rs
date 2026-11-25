// SPDX-License-Identifier: GPL-3.0-only
//! The streaming pipeline against the radio's budget: SD reads and heap, for each of the 22 recordings.
//!
//! **One test in this binary, on purpose.** The heap meter is the process's global allocator, so a test running
//! beside this one (the other test binaries are separate processes) would be counted in its peak.
//!
//! Run by package: `cargo test --release -p icap-app-hosttest --test icap_budget` (and again with
//! `--features fixed-point`, the numeric path the heap ceiling is for).

use icap_app_hosttest::{profile, resample::resample, wav};
use mfsk_core::ft8::decode_block::LowMem;

/// What the radio's interface allows the decoder to ask of the SD card, and of the heap, for one slot. The
/// radio reads forward, in calls of a few hundred samples, and can only restart a read by closing the file and
/// reading through it again (`apps/digi-common/digiapp.c`); its data region is 240 KiB. The streaming pipeline
/// (`decode_block_low_mem`) reads the slot only to build the coarse spectrogram (one forward sweep per
/// band, plus one for the fixed-point build's gain) and to build the candidates' 200 Hz basebands (one per
/// group of candidates); the refinement and the decoding run on those. So the sweeps are
/// `gain + bands + ceil(candidates / group)`, nothing is read and thrown away, and the bytes are those sweeps.
///
/// Held to this on every one of the 22 recordings, with the budget's own numbers:
/// - sweeps: exactly the above (restarts = sweeps - 1, no sample skipped);
/// - bytes moved: at most `sweeps x (slot + 2 blocks)`; and, as the 8 kHz slot is two thirds of the 12 kHz one
///   used here, **at most 3.2 MB at 8 kHz** for the ship setting (4 bands, 30 candidates in groups of 4);
/// - the heap: the 12 kHz audio is not in it (it is the source's), and the peak above what was live before is
///   at most 0.23 MB in the fixed-point build (the 240 KiB data region less the working set of the app shell).
///   The f32 build is not held to it: its spectrogram is twice the size (`0.5 MB`, `decode_block`'s `SpecCell`).
#[test]
fn the_streaming_pipeline_stays_inside_the_radios_budget() {
    let cfg = LowMem::new(4, 4);
    let max_cand = 30;
    let gain = usize::from(cfg!(feature = "fixed-point"));
    let (mut worst_bytes_8k, mut worst_heap) = (0u64, 0usize);
    for stem in profile::stems() {
        let slot12 = wav::load_i16(profile::asset_path(&stem));
        let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
        let r = profile::run_slot_stream(&slot, max_cand, cfg);
        let c = r.cost;
        // the number of groups is the number of candidates found, rounded up: between one and max_cand / group
        let sweeps = c.restarts as usize + 1;
        let lo = gain + cfg.coarse_bands + 1;
        let hi = gain + cfg.coarse_bands + max_cand.div_ceil(cfg.group);
        assert!(
            (lo..=hi).contains(&sweeps),
            "{stem}: {sweeps} sweeps, expected {lo}..={hi} ({c:?})"
        );
        assert_eq!(
            c.samples_skipped, 0,
            "{stem}: samples read and thrown away ({c:?})"
        );
        assert!(
            c.samples_read as usize <= sweeps * (slot.len() + 2048),
            "{stem}: {} samples read in {sweeps} sweeps of {} ({c:?})",
            c.samples_read,
            slot.len()
        );
        worst_bytes_8k = worst_bytes_8k.max(c.bytes_moved() * 2 / 3);
        worst_heap = worst_heap.max(r.run.peak_heap);
    }
    println!(
        "worst slot: {} KB moved (8 kHz), heap peak {} B",
        worst_bytes_8k / 1000,
        worst_heap
    );
    assert!(worst_bytes_8k <= 3_200_000, "{worst_bytes_8k} B at 8 kHz");
    if cfg!(feature = "fixed-point") {
        assert!(worst_heap <= 230_000, "heap peak {worst_heap} B");
    }

    // Method C (`band_bb`, `docs/notes/MIDRANGE_EMBEDDED_EVAL.md`): the whole band as one baseband, three passes
    // with the subtraction on it. The audio is read in one sweep, whole, nothing re-read (the SD cost is the
    // slot, 360 KB here, 240 KB at 8 kHz); what it holds instead is the baseband, 180 KB for any input rate. So
    // the heap is the baseband plus the working set of a pass: 0.33 MB in the fixed-point build (the radio's
    // 240 KiB region is not met: the baseband alone is 75 % of it), 0.59 MB in f32 (its band spectrogram is
    // twice the size).
    let mut worst_heap_c = 0usize;
    for stem in profile::stems() {
        let slot12 = wav::load_i16(profile::asset_path(&stem));
        let slot = resample(&resample(&slot12, 12_000, 8_000), 8_000, 12_000);
        let r = profile::run_slot_band_sic(&slot, max_cand, cfg.coarse_bands, cfg.group, 3);
        assert_eq!(r.cost.restarts, 0, "{stem}: the audio was read again");
        assert_eq!(r.cost.samples_skipped, 0, "{stem}: samples thrown away");
        assert_eq!(
            r.cost.samples_read as usize,
            slot.len(),
            "{stem}: not one sweep ({:?})",
            r.cost
        );
        worst_heap_c = worst_heap_c.max(r.run.peak_heap);
    }
    println!("method C: heap peak {worst_heap_c} B");
    let ceiling = if cfg!(feature = "fixed-point") {
        340_000
    } else {
        600_000
    };
    assert!(
        worst_heap_c <= ceiling,
        "method C heap peak {worst_heap_c} B"
    );
    assert!(
        worst_heap_c >= 180_000,
        "the baseband alone is 180 KB: {worst_heap_c} B"
    );
}
