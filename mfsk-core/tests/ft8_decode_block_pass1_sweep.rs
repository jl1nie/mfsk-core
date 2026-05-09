//! PASS1_LIMIT × max_cand sweep on real-QSO WAVs to find the
//! Core2 speed-budget sweet spot under the regularised
//! coarse_sync ratio (`RATIO_EPS=0.5`). Targets `BpAll`
//! (Core2's production depth — OSD off for budget).
//!
//! Run with:
//! ```sh
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_decode_block_pass1_sweep -- --include-ignored --nocapture
//! ```

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
#[ignore = "sweep: ~30 s. --include-ignored to run."]
fn ft8_decode_block_pass1_max_cand_sweep() {
    println!("\n=== PASS1_LIMIT × max_cand sweep (BpAll, fp i16) ===");

    // Frame ground truth per WAV (BpAllOsd, max_cand=200).
    let mut truth_per_wav: Vec<(String, BTreeSet<String>)> = Vec::new();
    let mut total_truth = 0usize;
    for wav_path in QSO_WAVS {
        let path = Path::new(wav_path);
        if !path.exists() {
            continue;
        }
        let slot = load_wav_i16(path);
        let r = decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200);
        let truth: BTreeSet<String> = r.iter().filter_map(|x| unpack77(&x.message77)).collect();
        total_truth += truth.len();
        truth_per_wav.push((
            path.file_name().unwrap().to_string_lossy().to_string(),
            truth,
        ));
    }
    println!(
        "  Frame truth: {total_truth} unique msgs across {} WAVs\n",
        truth_per_wav.len()
    );

    // PASS1 = max_cand simulates "Pass 2 elim" — coarse_sync emits
    // exactly max_cand cands, refine_candidates becomes a no-op
    // truncate (still computes block-0 cs for stage 3 reuse, no
    // wasted re-rank work).
    let pass1_values = [30usize, 50, 75];
    let max_cand_values = [15usize, 20];
    let sync_lag_values = [0.5f32, 0.7, 1.0];

    println!(
        "  {:<5} | {:<8} | {:<8} | {:<28} | qso3 ms | total ms | block-only | recall",
        "lag_s", "PASS1", "max_cand", "qso1/2/3 (truth/frame)"
    );
    println!("  {}", "─".repeat(105));

    for &lag in &sync_lag_values {
        unsafe { std::env::set_var("MFSK_SYNC_LAG_S", lag.to_string()) };
        for &pass1 in &pass1_values {
            // SAFETY: tests run single-threaded by default; env var write is fine.
            unsafe { std::env::set_var("MFSK_PASS1_LIMIT", pass1.to_string()) };
            for &mc in &max_cand_values {
                let mut per_wav_truth: Vec<usize> = Vec::new();
                let mut per_wav_total: Vec<usize> = Vec::new();
                let mut per_wav_ms: Vec<u128> = Vec::new();
                let mut block_only = 0usize;
                let mut total_recall = 0usize;
                for (label, truth) in &truth_per_wav {
                    let path = Path::new(
                        QSO_WAVS
                            .iter()
                            .find(|p| p.contains(label.as_str()))
                            .unwrap(),
                    );
                    let slot = load_wav_i16(path);
                    let t0 = Instant::now();
                    let r = decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, mc);
                    per_wav_ms.push(t0.elapsed().as_millis());
                    let block: BTreeSet<String> =
                        r.iter().filter_map(|x| unpack77(&x.message77)).collect();
                    let hit = block.intersection(truth).count();
                    let only = block.difference(truth).count();
                    per_wav_truth.push(hit);
                    per_wav_total.push(truth.len());
                    block_only += only;
                    total_recall += hit;
                }
                let recall_str = per_wav_truth
                    .iter()
                    .zip(&per_wav_total)
                    .map(|(h, t)| format!("{h}/{t}"))
                    .collect::<Vec<_>>()
                    .join(" ");
                let total_ms: u128 = per_wav_ms.iter().sum();
                let qso3_ms = per_wav_ms.last().copied().unwrap_or(0);
                println!(
                    "  {lag:<5} | {pass1:<8} | {mc:<8} | {recall_str:<28} | {qso3_ms:>7} | {total_ms:>8} | +{block_only}         | {total_recall}/{total_truth}"
                );
            }
        }
        println!();
    }
    unsafe {
        std::env::remove_var("MFSK_PASS1_LIMIT");
        std::env::remove_var("MFSK_SYNC_LAG_S");
    }
}
