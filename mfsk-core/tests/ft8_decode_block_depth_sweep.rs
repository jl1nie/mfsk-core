//! Sweep DecodeDepth ∈ {Bp, BpAll, BpAllOsd} on real-QSO WAVs to
//! quantify the cost of the deeper LLR/OSD fallback stages on
//! Core2-bound configurations.

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::{DecodeDepth, decode_frame};
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO_WAVS: &[&str] = common::REAL_QSO_WAVS;

use common::load_wav_i16;

#[test]
#[ignore = "sweep ~10s — --include-ignored"]
fn ft8_decode_depth_sweep() {
    println!("\n=== DecodeDepth sweep (fp i16, max_cand=15, q_thresh=12 default) ===");

    let truths: Vec<(String, BTreeSet<String>)> = QSO_WAVS
        .iter()
        .filter_map(|p| {
            let path = Path::new(p);
            if !path.exists() {
                return None;
            }
            let slot = load_wav_i16(path);
            let r = decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200);
            let truth: BTreeSet<String> = r.iter().filter_map(|x| unpack77(&x.message77)).collect();
            Some((
                path.file_name().unwrap().to_string_lossy().to_string(),
                truth,
            ))
        })
        .collect();
    let total_truth: usize = truths.iter().map(|(_, t)| t.len()).sum();

    println!(
        "  {:<12} | {:<22} | qso3 ms | total ms | recall",
        "depth", "qso1/2/3"
    );
    println!("  {}", "─".repeat(75));

    for depth in [DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
        let mut per_truth = Vec::new();
        let mut per_total = Vec::new();
        let mut per_ms = Vec::new();
        let mut total_recall = 0;
        let mut block_only = 0;
        for (label, truth) in &truths {
            let path = Path::new(
                QSO_WAVS
                    .iter()
                    .find(|p| p.contains(label.as_str()))
                    .unwrap(),
            );
            let slot = load_wav_i16(path);
            let t0 = Instant::now();
            let r = decode_block(&slot, 100.0, 3000.0, 1.0, depth, 15);
            per_ms.push(t0.elapsed().as_millis());
            let block: BTreeSet<String> = r.iter().filter_map(|x| unpack77(&x.message77)).collect();
            let hit = block.intersection(truth).count();
            block_only += block.difference(truth).count();
            per_truth.push(hit);
            per_total.push(truth.len());
            total_recall += hit;
        }
        let recall_str = per_truth
            .iter()
            .zip(&per_total)
            .map(|(h, t)| format!("{h}/{t}"))
            .collect::<Vec<_>>()
            .join(" ");
        let total_ms: u128 = per_ms.iter().sum();
        let qso3_ms = per_ms.last().copied().unwrap_or(0);
        println!(
            "  {:<12} | {:<22} | {:>7} | {:>8} | {}/{}+{}",
            format!("{:?}", depth),
            recall_str,
            qso3_ms,
            total_ms,
            total_recall,
            total_truth,
            block_only,
        );
    }
}
