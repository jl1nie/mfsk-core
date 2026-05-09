//! FT8 reference WAV: host full-effort vs embedded equivalent
//! wall-clock + recall comparison.
//!
//! The only formally-distributed FT8 reference recording from the
//! WSJT-X project is `samples/FT8/210703_133430.wav` (a busy-band
//! 7-station slot). It's the same audio as `embedded-poc/assets/
//! qso3_busy.wav` (verified via `cmp` 2026-05-04).
//!
//! mfsk-core's `decode_block` under `fixed-point` is bit-identical
//! between host and Xtensa builds (Issue #15 Phase 1 verified
//! 2026-05-03), so the host `decode_block` row is a faithful proxy
//! for the LX6 / LX7 production decoder's recall — only the
//! wall-clock differs (host is ~50× faster than 240 MHz Xtensa).
//! Real on-hardware times come from the rx-wavsim bench logs
//! committed in this branch.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_reference_suite_recall \
//!     -- --include-ignored --nocapture
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "fixed-point"))]

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::{DecodeDepth, decode_frame};
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

/// (label, path, hardware timing if measured for this WAV).
type Entry = (&'static str, &'static str, Option<HwTiming>);

#[derive(Copy, Clone)]
struct HwTiming {
    /// post-SlotEnd ms on M5Stack Core2 LX6 at q_thresh=12 (full).
    core2_post_slotend_ms: u32,
    /// post-SlotEnd ms on M5StickS3 LX7 at q_thresh=12 (full).
    s3_post_slotend_ms: u32,
}

/// Reference WAVs.
///
/// `qso3_busy` is the **WSJT-X formally-distributed FT8 reference
/// sample** (`samples/FT8/210703_133430.wav`). The other two
/// (`qso1`, `qso2`) are private on-air recordings carried in this
/// repo for breadth — they're informational, not "reference" in the
/// formal sense.
const ENTRIES: &[Entry] = &[
    // WSJT-X reference recording (busy band, 7 stations).
    (
        "qso3 busy band  (WSJT-X 210703_133430.wav)",
        asset_path!("qso3_busy.wav"),
        Some(HwTiming {
            core2_post_slotend_ms: 1434,
            s3_post_slotend_ms: 707,
        }),
    ),
    (
        "qso1 mid-band   (informational, on-air capture)",
        asset_path!("qso1.wav"),
        Some(HwTiming {
            core2_post_slotend_ms: 1303,
            s3_post_slotend_ms: 574,
        }),
    ),
    (
        "qso2 mid-band   (informational, on-air capture)",
        asset_path!("qso2.wav"),
        Some(HwTiming {
            core2_post_slotend_ms: 632,
            s3_post_slotend_ms: 370,
        }),
    ),
];

use common::load_wav_i16_opt as load_wav_i16;

#[test]
#[ignore = "reference recall + wall-clock; run with --include-ignored"]
fn ft8_reference_host_vs_embedded() {
    println!("\n=== FT8 reference: host full-decode vs embedded ===\n");
    println!(
        "  Host:    Ryzen / x86_64 desktop, single-thread, rustfft.\n  \
         Embed:   M5Stack Core2 (ESP32 LX6 dual-core 240 MHz, 8 MB QUAD PSRAM)\n  \
                  / M5StickS3   (ESP32-S3 LX7 dual-core 240 MHz + PIE SIMD,\n  \
                                 8 MB Octal PSRAM)\n  \
           Embedded ms = post-SlotEnd wall-clock from rx-wavsim bench\n  \
                         (stage 2 hidden under capture; pass 2 + stage 3\n  \
                         only). Logged in `logs/{{core2_q_sweep,\n  \
                         s3_workstealing}}_2026-05-04.log`."
    );
    println!();

    let bar = "─".repeat(140);
    println!(
        "  {:<48} {:>4} {:>4}  {:<8}  {:<8}  {:<8}  {:<8}  {:<8}  {:<8}",
        "WAV",
        "tru",
        "host",
        "Bp/30/15",
        "Bp/30/30",
        "Bp/50/30",
        "Bp100/30",
        "BpO30/30",
        "BpO100/3"
    );
    println!("  {bar}");

    let mut sum_truth = 0usize;
    let mut sum_host_ms = 0u128;
    let mut sum_bp_hit = 0usize;
    let mut sum_bp_ms = 0u128;
    let mut sum_osd_hit = 0usize;
    let mut sum_osd_ms = 0u128;
    let mut sum_core2 = 0u32;
    let mut sum_s3 = 0u32;

    for (label, path, hw) in ENTRIES {
        let Some(slot) = load_wav_i16(Path::new(path)) else {
            println!("  {label:<48}  (load failed: {path})");
            continue;
        };

        // Host wide-band reference: BpAllOsd, max_cand=200.
        let t0 = Instant::now();
        let truth: BTreeSet<String> =
            decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200)
                .iter()
                .filter_map(|x| unpack77(&x.message77))
                .collect();
        let host_ms = t0.elapsed().as_millis();

        // Sweep: configurations bounded by the FT8 real-time decode
        // budget (≈ 2 s post-SlotEnd before slot N+1 TX must start).
        let mut row: Vec<(String, BTreeSet<String>, u128)> = Vec::new();
        let configs: &[(&str, usize, usize, DecodeDepth)] = &[
            ("Bp/30/15", 30, 15, DecodeDepth::BpAll), // ship
            ("Bp/30/30", 30, 30, DecodeDepth::BpAll),
            ("Bp/50/30", 50, 30, DecodeDepth::BpAll),
            ("Bp/100/30", 100, 30, DecodeDepth::BpAll),
            ("BpO/30/30", 30, 30, DecodeDepth::BpAllOsd),
            ("BpO/100/30", 100, 30, DecodeDepth::BpAllOsd),
        ];
        for &(name, p1, mc, depth) in configs {
            unsafe { std::env::set_var("MFSK_PASS1_LIMIT", p1.to_string()) };
            let t = Instant::now();
            let r: BTreeSet<String> = decode_block(&slot, 100.0, 3000.0, 1.0, depth, mc)
                .iter()
                .filter_map(|x| unpack77(&x.message77))
                .collect();
            let ms = t.elapsed().as_millis();
            row.push((name.to_string(), r, ms));
        }
        unsafe { std::env::remove_var("MFSK_PASS1_LIMIT") };
        let bp = row[0].1.clone();
        let bp_ms = row[0].2;
        let osd = row[3].1.clone();
        let osd_ms = row[3].2;

        let n_truth = truth.len();
        let bp_hit = bp.intersection(&truth).count();
        let osd_hit = osd.intersection(&truth).count();

        let core2_ms = hw.map(|h| h.core2_post_slotend_ms).unwrap_or(0);
        let s3_ms = hw.map(|h| h.s3_post_slotend_ms).unwrap_or(0);

        let cells: Vec<String> = row
            .iter()
            .map(|(_, r, ms)| {
                let hit = r.intersection(&truth).count();
                format!("{hit:>2}/{ms:<5}")
            })
            .collect();
        println!(
            "  {label:<48} {n_truth:>4} {host_ms:>4}  {}",
            cells.join("  "),
        );

        sum_truth += n_truth;
        sum_host_ms += host_ms;
        sum_bp_hit += bp_hit;
        sum_bp_ms += bp_ms;
        sum_osd_hit += osd_hit;
        sum_osd_ms += osd_ms;
        sum_core2 += core2_ms;
        sum_s3 += s3_ms;
        // Track all configs aggregate.
    }

    println!("  {bar}");
    println!(
        "  {:<48}  {sum_truth:>5}  {sum_host_ms:>6}  {sum_bp_hit:>3} / {sum_bp_ms:>5}  {sum_osd_hit:>3} / {sum_osd_ms:>8}  {sum_core2:>9}  {sum_s3:>9}",
        "TOTAL"
    );
    let bp_pct = 100.0 * sum_bp_hit as f64 / sum_truth as f64;
    let osd_pct = 100.0 * sum_osd_hit as f64 / sum_truth as f64;
    println!(
        "\n  BpAll/15      recall: {sum_bp_hit} / {sum_truth} = {bp_pct:.1} %  (host {sum_bp_ms} ms)"
    );
    println!(
        "  BpAllOsd/30   recall: {sum_osd_hit} / {sum_truth} = {osd_pct:.1} %  (host {sum_osd_ms} ms)"
    );
    let lx7_factor = sum_s3 as f64 / sum_bp_ms as f64; // ratio of LX7 wall vs host BpAll wall
    let est_lx7_osd = sum_osd_ms as f64 * lx7_factor;
    println!(
        "  LX7 wall vs host BpAll: ×{:.1} (sum {sum_s3} / {sum_bp_ms} ms)",
        lx7_factor
    );
    println!(
        "  Predicted S3 LX7 BpAllOsd/30: ≈ {est_lx7_osd:.0} ms total, slot avg ≈ {:.0} ms",
        est_lx7_osd / ENTRIES.len() as f64
    );
    println!(
        "  Slot real-time budget: 15 000 ms capture window; current post-SlotEnd ≈ {:.0} ms avg.",
        sum_s3 as f64 / ENTRIES.len() as f64
    );

    println!("\n=== Missed-callsign breakdown (truth − embedded), with SNRs from host run ===");
    for (label, path, _hw) in ENTRIES {
        let Some(slot) = load_wav_i16(Path::new(path)) else {
            continue;
        };
        let host = decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200);
        let embed = decode_block(&slot, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 15);
        let embed_msgs: BTreeSet<String> = embed
            .iter()
            .filter_map(|x| unpack77(&x.message77))
            .collect();
        let missed: Vec<&mfsk_core::ft8::decode::DecodeResult> = host
            .iter()
            .filter(|r| {
                let Some(t) = unpack77(&r.message77) else {
                    return false;
                };
                !embed_msgs.contains(&t)
            })
            .collect();
        println!("\n  {label}");
        if missed.is_empty() {
            println!("    (none missed)");
            continue;
        }
        // Sort by SNR descending so the most embarrassing miss (strongest)
        // is at the top.
        let mut sorted = missed.clone();
        sorted.sort_by(|a, b| {
            b.snr_db
                .partial_cmp(&a.snr_db)
                .unwrap_or(core::cmp::Ordering::Equal)
        });
        for r in sorted {
            let Some(text) = unpack77(&r.message77) else {
                continue;
            };
            println!(
                "    {:>4.0} Hz  SNR={:>5.1} dB  e={:>2}  '{}'",
                r.freq_hz, r.snr_db, r.hard_errors, text
            );
        }
    }
}
