//! `DecodeDepth::BP_ONLY` (full `LlrEffort`) vs `DecodeDepth::EMBEDDED`
//! (`LlrEffort::Minimal`) recall + wall-clock sweep across the in-repo
//! FT8 corpus.
//!
//! Built to validate the embedded ship config (PR #123, 2026-05-21;
//! renamed/collapsed in the 0.8.0 `DecodeDepth` redesign, issue #182
//! follow-up) — `qso3_busy` alone showed variants b/c contributing no
//! decodes, but that's a single strong-signal corpus. Run this sweep
//! on the on-air recordings (qso1, qso2, 191111_*) to confirm
//! `LlrEffort::Minimal` doesn't lose any decodes those other slots
//! need. This is the permanent regression guard for the 0.8.0
//! decision to collapse the old 3-tier `BpAll`/`BpAllNoNsym3`/
//! `BpVariantsAd` scheme down to 2 (`LlrEffort::{Minimal, Full}`) —
//! the removed middle tier had zero real callers and this sweep's own
//! history never showed it earning its keep.
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
#[ignore = "BP_ONLY vs EMBEDDED vs FULL sweep; run with --include-ignored"]
fn no_nsym3_recall_sweep() {
    println!("\n=== DecodeDepth::BP_ONLY vs EMBEDDED vs FULL recall + wall-clock ===\n");
    println!(
        "Truth = FULL, max_cand=200 (widest host search).\n\
         Each cell = decodes ∩ truth  /  wall-clock ms.\n"
    );

    println!(
        "  {:<38}  {:>4}  {:<12}  {:<12}  {:<12}",
        "WAV", "tru", "BP_ONLY/15", "EMBEDDED/15", "FULL/15"
    );
    println!("  {}", "─".repeat(88));

    let mut sum_tru = 0usize;
    let mut sum_bp = 0usize;
    let mut sum_ad = 0usize;
    let mut sum_osd = 0usize;
    let mut sum_bp_ms = 0u128;
    let mut sum_ad_ms = 0u128;
    let mut sum_osd_ms = 0u128;

    for (label, path) in WAVS {
        let Some(slot) = load_wav_i16(Path::new(path)) else {
            println!("  {label:<38}  (load failed: {path})");
            continue;
        };

        let truth: BTreeSet<String> =
            decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::FULL, 200)
                .iter()
                .filter_map(|x| unpack77(x.message77()))
                .collect();

        let configs: &[(&str, DecodeDepth)] = &[
            ("BP_ONLY", DecodeDepth::BP_ONLY),
            ("EMBEDDED", DecodeDepth::EMBEDDED),
            ("FULL", DecodeDepth::FULL),
        ];

        let mut cells: Vec<String> = Vec::new();
        let mut hits = Vec::new();
        let mut mss = Vec::new();
        for &(_, depth) in configs {
            let t0 = Instant::now();
            let r: BTreeSet<String> = decode_block(&slot, 100.0, 3000.0, 1.0, depth, 15)
                .iter()
                .filter_map(|x| unpack77(x.message77()))
                .collect();
            let ms = t0.elapsed().as_millis();
            let hit = r.intersection(&truth).count();
            cells.push(format!("{hit:>2}/{ms:<6}"));
            hits.push(hit);
            mss.push(ms);
        }

        println!(
            "  {:<38}  {:>4}  {:<12}  {:<12}  {:<12}",
            label,
            truth.len(),
            cells[0],
            cells[1],
            cells[2],
        );

        sum_tru += truth.len();
        sum_bp += hits[0];
        sum_ad += hits[1];
        sum_osd += hits[2];
        sum_bp_ms += mss[0];
        sum_ad_ms += mss[1];
        sum_osd_ms += mss[2];

        // Per-WAV recall delta callout: BP_ONLY (full LLR effort) vs
        // EMBEDDED (Minimal) — the comparison this sweep exists to
        // guard.
        if hits[0] != hits[1] {
            println!(
                "    ⚠ EMBEDDED recall delta: BP_ONLY={} vs EMBEDDED={} (-{})",
                hits[0],
                hits[1],
                hits[0] as i32 - hits[1] as i32
            );
            let bp_set: BTreeSet<String> =
                decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BP_ONLY, 15)
                    .iter()
                    .filter_map(|x| unpack77(x.message77()))
                    .collect();
            let ad_set: BTreeSet<String> =
                decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::EMBEDDED, 15)
                    .iter()
                    .filter_map(|x| unpack77(x.message77()))
                    .collect();
            let lost: Vec<&String> = bp_set.difference(&ad_set).collect();
            let gained: Vec<&String> = ad_set.difference(&bp_set).collect();
            if !lost.is_empty() {
                println!("    EMBEDDED lost  : {:?}", lost);
            }
            if !gained.is_empty() {
                println!("    EMBEDDED gained: {:?}", gained);
            }
        }
    }

    println!("  {}", "─".repeat(88));
    println!(
        "  {:<38}  {:>4}  {:<12}  {:<12}  {:<12}",
        "TOTAL",
        sum_tru,
        format!("{sum_bp:>2}/{sum_bp_ms:<6}"),
        format!("{sum_ad:>2}/{sum_ad_ms:<6}"),
        format!("{sum_osd:>2}/{sum_osd_ms:<6}"),
    );
    println!();
    println!(
        "  EMBEDDED : {sum_ad}/{sum_tru} ({:.1}%) in {sum_ad_ms}ms  ← LlrEffort::Minimal (a+d only)",
        100.0 * sum_ad as f32 / sum_tru.max(1) as f32
    );
    println!(
        "  BP_ONLY  : {sum_bp}/{sum_tru} ({:.1}%) in {sum_bp_ms}ms  ← LlrEffort::Full, no OSD",
        100.0 * sum_bp as f32 / sum_tru.max(1) as f32
    );
}
