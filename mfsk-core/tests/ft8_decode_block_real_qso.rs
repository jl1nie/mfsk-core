//! Real-QSO FT8 WAV comparison: `decode_frame` (host wide-band path)
//! vs `decode_block` (embedded path) on actual on-air recordings.
//!
//! Synthetic AWGN sweeps (`ft8_decode_block_snr_sweep.rs`) prove
//! sensitivity equivalence in isolation; this harness does the
//! ecologically-valid check — multiple co-channel stations, real
//! channel noise, real fading, and end-to-end wall-clock for both
//! decoders.
//!
//! Run with:
//! ```sh
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8 \
//!     --test ft8_decode_block_real_qso -- --include-ignored --nocapture
//!
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_decode_block_real_qso -- --include-ignored --nocapture
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult, decode_frame};
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

/// On-air recordings — 12 kHz / mono / i16. See
/// [`common::REAL_QSO_WAVS`] for provenance and bit-identity notes.
const QSO_WAVS: &[&str] = common::REAL_QSO_WAVS;

use common::load_wav_i16;

fn quantize_to_i8(slot: &[i16]) -> Vec<i8> {
    slot.iter().map(|&s| (s >> 8) as i8).collect()
}

/// Map decoded message text → (best SNR, freq Hz). Best SNR wins
/// for messages that appear in multiple results.
fn message_snr_map(results: &[DecodeResult]) -> BTreeMap<String, (f32, f32)> {
    let mut out: BTreeMap<String, (f32, f32)> = BTreeMap::new();
    for r in results {
        if let Some(text) = unpack77(&r.message77) {
            out.entry(text)
                .and_modify(|(snr, _f)| {
                    if r.snr_db > *snr {
                        *snr = r.snr_db;
                    }
                })
                .or_insert((r.snr_db, r.freq_hz));
        }
    }
    out
}

#[test]
#[ignore = "slow: ~10 s per WAV. Real on-air recordings, --include-ignored to run."]
fn ft8_decode_block_vs_decode_frame_real_qso() {
    println!("\n=== FT8 real-QSO decoder comparison ===");
    #[cfg(feature = "fixed-point")]
    println!("(fixed-point feature is ON — block(i16/i8) use embedded fixed-point path)");
    #[cfg(not(feature = "fixed-point"))]
    println!("(fixed-point feature is OFF — block(i16/i8) use f32 path)");

    let mut total_frame = 0usize;
    let mut total_block_i16 = 0usize;
    let mut total_block_i8 = 0usize;
    let mut total_t_frame = 0u128;
    let mut total_t_block_i16 = 0u128;
    let mut total_t_block_i8 = 0u128;
    let mut total_unique_to_frame = 0usize;
    let mut total_unique_to_block = 0usize;

    for wav_path in QSO_WAVS {
        let path = Path::new(wav_path);
        if !path.exists() {
            println!("  ⚠  skip: {wav_path} (file not found)");
            continue;
        }
        let slot_i16 = load_wav_i16(path);
        let slot_i8 = quantize_to_i8(&slot_i16);
        let label = path.file_name().unwrap().to_string_lossy();

        // decode_frame (host baseline)
        let t = Instant::now();
        let r_frame = decode_frame(
            &slot_i16,
            /* freq_min */ 100.0,
            /* freq_max */ 3_000.0,
            /* sync_min */ 1.0,
            /* freq_hint */ None,
            DecodeDepth::BpAllOsd,
            /* max_cand */ 30,
        );
        let t_frame = t.elapsed().as_micros();

        // decode_block on i16. max_cand=200 to expose the true
        // ceiling of the embedded path (was 20 — clipped real
        // signals that outranked-by-noise on busy bands).
        let t = Instant::now();
        let r_block_i16 = decode_block(
            &slot_i16,
            100.0,
            3_000.0,
            1.0,
            DecodeDepth::BpAllOsd,
            /* max_cand */ 30,
        );
        let t_block_i16 = t.elapsed().as_micros();

        // decode_block on i8 — matches the embedded data path.
        let t = Instant::now();
        let r_block_i8 = decode_block(&slot_i8, 100.0, 3_000.0, 1.0, DecodeDepth::BpAllOsd, 30);
        let t_block_i8 = t.elapsed().as_micros();

        let snr_frame = message_snr_map(&r_frame);
        let snr_block_i16 = message_snr_map(&r_block_i16);
        let snr_block_i8 = message_snr_map(&r_block_i8);
        let msgs_frame: BTreeSet<String> = snr_frame.keys().cloned().collect();
        let msgs_block_i16: BTreeSet<String> = snr_block_i16.keys().cloned().collect();
        let msgs_block_i8: BTreeSet<String> = snr_block_i8.keys().cloned().collect();

        println!("\n── {label} ─────────────────────────");
        println!(
            "  decode_frame    : {:>3} msgs in {:>7.0} ms",
            msgs_frame.len(),
            t_frame as f64 / 1000.0
        );
        println!(
            "  decode_block i16: {:>3} msgs in {:>7.0} ms",
            msgs_block_i16.len(),
            t_block_i16 as f64 / 1000.0
        );
        println!(
            "  decode_block i8 : {:>3} msgs in {:>7.0} ms",
            msgs_block_i8.len(),
            t_block_i8 as f64 / 1000.0
        );
        println!(
            "  block recall vs frame: {}/{} ({:.0}%)",
            msgs_frame.intersection(&msgs_block_i16).count(),
            msgs_frame.len(),
            100.0 * msgs_frame.intersection(&msgs_block_i16).count() as f64
                / msgs_frame.len().max(1) as f64,
        );

        // Per-message SNR table — every message either decoder caught,
        // sorted by frame's reported SNR (where available) so weak
        // signals sit at the top.
        let all_msgs: BTreeSet<&String> = msgs_frame
            .iter()
            .chain(msgs_block_i16.iter())
            .chain(msgs_block_i8.iter())
            .collect();
        type Row<'a> = (
            &'a String,
            Option<f32>,
            Option<f32>,
            Option<f32>,
            Option<f32>,
        );
        let mut rows: Vec<Row> = all_msgs
            .iter()
            .map(|m| {
                let f_snr = snr_frame.get(*m).map(|(s, _)| *s);
                let f_freq = snr_frame.get(*m).map(|(_, f)| *f);
                let b16 = snr_block_i16.get(*m).map(|(s, _)| *s);
                let b8 = snr_block_i8.get(*m).map(|(s, _)| *s);
                (*m, f_snr, b16, b8, f_freq)
            })
            .collect();
        // Sort: weakest frame SNR first; missed-by-frame to the bottom.
        rows.sort_by(|a, b| match (a.1, b.1) {
            (Some(x), Some(y)) => x.partial_cmp(&y).unwrap(),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => std::cmp::Ordering::Equal,
        });
        println!(
            "  {:<33} | {:>7} | {:>7} | {:>7} | {:>7}",
            "message", "freq", "frame", "blk-i16", "blk-i8"
        );
        println!("  {}", "─".repeat(82));
        for (msg, f_snr, b16, b8, f_freq) in &rows {
            let f_str = f_snr.map_or("    —  ".into(), |x| format!("{:>+5.0} dB", x));
            let b16_str = b16.map_or("    —  ".into(), |x| format!("{:>+5.0} dB", x));
            let b8_str = b8.map_or("    —  ".into(), |x| format!("{:>+5.0} dB", x));
            let freq_str = f_freq.map_or("   —  ".into(), |f| format!("{:>5.0} Hz", f));
            // Trim to 33 cols so the SNR columns line up.
            let mt = if msg.len() > 33 {
                &msg[..33]
            } else {
                msg.as_str()
            };
            println!(
                "  {:<33} | {:>7} | {:>7} | {:>7} | {:>7}",
                mt, freq_str, f_str, b16_str, b8_str
            );
        }

        total_frame += msgs_frame.len();
        total_block_i16 += msgs_block_i16.len();
        total_block_i8 += msgs_block_i8.len();
        total_t_frame += t_frame;
        total_t_block_i16 += t_block_i16;
        total_t_block_i8 += t_block_i8;
        total_unique_to_frame += msgs_frame.difference(&msgs_block_i16).count();
        total_unique_to_block += msgs_block_i16.difference(&msgs_frame).count();
    }

    println!("\n=== Totals ===");
    println!(
        "  decode_frame   : {:>3} unique msgs, {:>7.0} ms total",
        total_frame,
        total_t_frame as f64 / 1000.0
    );
    println!(
        "  decode_block i16: {:>3} unique msgs, {:>7.0} ms total ({:.1}× vs frame)",
        total_block_i16,
        total_t_block_i16 as f64 / 1000.0,
        total_t_block_i16 as f64 / total_t_frame as f64
    );
    println!(
        "  decode_block i8 : {:>3} unique msgs, {:>7.0} ms total ({:.1}× vs frame)",
        total_block_i8,
        total_t_block_i8 as f64 / 1000.0,
        total_t_block_i8 as f64 / total_t_frame as f64
    );
    println!(
        "  recall total    : {:>3}/{} from frame, +{} block-only",
        total_frame - total_unique_to_frame,
        total_frame,
        total_unique_to_block
    );
}
