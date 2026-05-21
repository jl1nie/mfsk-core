//! BpAll vs BpAllNoNsym3 vs BpAllOsd recall + wall-clock sweep
//! across the in-repo FT8 corpus.
//!
//! Built to validate the embedded `BpAllNoNsym3` ship config (PR
//! #123, 2026-05-21) — `qso3_busy` alone showed variant-c contributing
//! no decodes, but that's a single strong-signal corpus. Run this
//! sweep on the on-air recordings (qso1, qso2, 191111_*) to confirm
//! variant-c doesn't rescue any decodes those other slots need.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_no_nsym3_sweep \
//!     -- --include-ignored --nocapture
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "fixed-point"))]

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as load_wav_i16;

const WAVS: &[(&str, &str)] = &[
    (
        "qso3_busy (WSJT-X 210703_133430)",
        asset_path!("qso3_busy.wav"),
    ),
    ("qso1 (on-air)", asset_path!("qso1.wav")),
    ("qso2 (on-air)", asset_path!("qso2.wav")),
    ("191111_110130 (on-air)", asset_path!("191111_110130.wav")),
    ("191111_110200 (on-air)", asset_path!("191111_110200.wav")),
];

#[test]
#[ignore = "BpAllNoNsym3 vs BpAll sweep; run with --include-ignored"]
fn no_nsym3_recall_sweep() {
    println!("\n=== BpAll vs BpAllNoNsym3 vs BpAllOsd recall + wall-clock ===\n");
    println!(
        "Truth = BpAllOsd, max_cand=200 (widest host search).\n\
         Each cell = decodes ∩ truth  /  wall-clock ms.\n"
    );

    println!(
        "  {:<38}  {:>4}  {:<10}  {:<10}  {:<10}",
        "WAV", "tru", "BpAll/15", "NoNsym3/15", "BpAllOsd/15"
    );
    println!("  {}", "─".repeat(82));

    let mut sum_tru = 0usize;
    let mut sum_bp = 0usize;
    let mut sum_nn = 0usize;
    let mut sum_osd = 0usize;
    let mut sum_bp_ms = 0u128;
    let mut sum_nn_ms = 0u128;
    let mut sum_osd_ms = 0u128;

    for (label, path) in WAVS {
        let Some(slot) = load_wav_i16(Path::new(path)) else {
            println!("  {label:<38}  (load failed: {path})");
            continue;
        };

        let truth: BTreeSet<String> =
            decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BpAllOsd, 200)
                .iter()
                .filter_map(|x| unpack77(&x.message77))
                .collect();

        let configs: &[(&str, DecodeDepth)] = &[
            ("BpAll", DecodeDepth::BpAll),
            ("NoNsym3", DecodeDepth::BpAllNoNsym3),
            ("BpAllOsd", DecodeDepth::BpAllOsd),
        ];

        let mut cells: Vec<String> = Vec::new();
        let mut hits = Vec::new();
        let mut mss = Vec::new();
        for &(_, depth) in configs {
            let t0 = Instant::now();
            let r: BTreeSet<String> = decode_block(&slot, 100.0, 3000.0, 1.0, depth, 15)
                .iter()
                .filter_map(|x| unpack77(&x.message77))
                .collect();
            let ms = t0.elapsed().as_millis();
            let hit = r.intersection(&truth).count();
            cells.push(format!("{hit:>2}/{ms:<6}"));
            hits.push(hit);
            mss.push(ms);
        }

        println!(
            "  {:<38}  {:>4}  {:<10}  {:<10}  {:<10}",
            label,
            truth.len(),
            cells[0],
            cells[1],
            cells[2]
        );

        sum_tru += truth.len();
        sum_bp += hits[0];
        sum_nn += hits[1];
        sum_osd += hits[2];
        sum_bp_ms += mss[0];
        sum_nn_ms += mss[1];
        sum_osd_ms += mss[2];

        // Per-WAV recall delta callout.
        if hits[0] != hits[1] {
            println!(
                "    ⚠ NoNsym3 recall delta: BpAll={} vs NoNsym3={} (-{})",
                hits[0],
                hits[1],
                hits[0] as i32 - hits[1] as i32
            );
            let bp_set: BTreeSet<String> =
                decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 15)
                    .iter()
                    .filter_map(|x| unpack77(&x.message77))
                    .collect();
            let nn_set: BTreeSet<String> =
                decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BpAllNoNsym3, 15)
                    .iter()
                    .filter_map(|x| unpack77(&x.message77))
                    .collect();
            let lost: Vec<&String> = bp_set.difference(&nn_set).collect();
            let gained: Vec<&String> = nn_set.difference(&bp_set).collect();
            if !lost.is_empty() {
                println!("    NoNsym3 lost  : {:?}", lost);
            }
            if !gained.is_empty() {
                println!("    NoNsym3 gained: {:?}", gained);
            }
        }
    }

    println!("  {}", "─".repeat(82));
    println!(
        "  {:<38}  {:>4}  {:<10}  {:<10}  {:<10}",
        "TOTAL",
        sum_tru,
        format!("{sum_bp:>2}/{sum_bp_ms:<6}"),
        format!("{sum_nn:>2}/{sum_nn_ms:<6}"),
        format!("{sum_osd:>2}/{sum_osd_ms:<6}"),
    );
    println!();
    println!(
        "  NoNsym3 recall: {}/{} ({:.1}%) vs BpAll {}/{} ({:.1}%); time {}ms vs {}ms",
        sum_nn,
        sum_tru,
        100.0 * sum_nn as f32 / sum_tru.max(1) as f32,
        sum_bp,
        sum_tru,
        100.0 * sum_bp as f32 / sum_tru.max(1) as f32,
        sum_nn_ms,
        sum_bp_ms,
    );
}
