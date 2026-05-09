//! Diagnostic: dump every candidate `decode_block::coarse_sync`
//! emits for the 3 real-QSO WAVs, cross-reference with the truth
//! freqs `decode_frame` finds. If a truth signal isn't even in the
//! coarse_sync output (top-N), the recall gap originates in stage 1+2,
//! not in BP/OSD.
//!
//! Run with `--features fft-rustfft,ft8,fixed-point` to test the
//! embedded i16 path; without `fixed-point` to test the f32 path.

use std::path::Path;

use mfsk_core::ft8::decode::{DecodeDepth, decode_frame};
use mfsk_core::ft8::decode_block::{coarse_sync, compute_spectrogram};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO_WAVS: &[&str] = common::REAL_QSO_WAVS;

use common::load_wav_i16;

#[test]
#[ignore = "diagnostic — run with --include-ignored --nocapture"]
fn coarse_sync_candidate_diag() {
    println!(
        "\n=== coarse_sync candidate diagnostic ===  (block path, max_cand=200, sync_min=0.5)\n"
    );
    #[cfg(feature = "fixed-point")]
    println!("(fixed-point feature ON — i16 spec / sc16 path)");
    #[cfg(not(feature = "fixed-point"))]
    println!("(fixed-point feature OFF — f32 spec)");

    for wav_path in QSO_WAVS {
        let path = Path::new(wav_path);
        // Diagnostic reads absolute paths that only exist on the
        // developer's box (rs-ft8n bench + WSJT-X samples); CI
        // doesn't check those in. Skip silently rather than
        // panicking — the diagnostic still runs locally for anyone
        // with the recordings.
        if !path.exists() {
            println!("(skip {} — file not present)", path.display());
            continue;
        }
        let label = path.file_name().unwrap().to_string_lossy();
        let slot = load_wav_i16(path);

        // Frame's truth: every signal frame can find.
        let frame = decode_frame(&slot, 100.0, 3000.0, 1.0, None, DecodeDepth::BpAllOsd, 200);
        let truth: Vec<(f32, f32, f32, String)> = frame
            .iter()
            .filter_map(|r| unpack77(&r.message77).map(|t| (r.freq_hz, r.dt_sec, r.snr_db, t)))
            .collect();

        // Block's candidates. Top-200 (already sorted by score
        // descending inside `coarse_sync`).
        let spec = compute_spectrogram(&slot, 3000.0);
        let cands = coarse_sync(&spec, 100.0, 3000.0, 0.5, 200);

        println!("── {} ─────────────────────────────────────", label);
        println!("  truth from decode_frame: {} signals", truth.len());
        println!("  block coarse_sync emitted: {} candidates", cands.len());

        // For each truth, find the closest candidate within 6 Hz / 0.5 s
        // AND its rank in the score-sorted top-200. This tells us
        // whether `max_cand=30` (Core2 budget) would have cut the
        // truth signal out of stage 3.
        println!(
            "\n  {:<32} | {:>7} | {:>5} | {:>4} | rank | closest cand (df, ddt, score)",
            "truth msg", "freq", "SNR", "dt"
        );
        println!("  {}", "─".repeat(102));
        for (tf, tdt, tsnr, tmsg) in &truth {
            let mut best_df = f32::INFINITY;
            let mut best_rank: Option<usize> = None;
            let mut best: Option<&_> = None;
            for (rank, c) in cands.iter().enumerate() {
                let df = (c.freq_hz - tf).abs();
                let ddt = (c.dt_sec - tdt).abs();
                if df < 6.0 && ddt < 0.5 && df < best_df {
                    best_df = df;
                    best_rank = Some(rank);
                    best = Some(c);
                }
            }
            let mt = if tmsg.len() > 32 {
                &tmsg[..32]
            } else {
                tmsg.as_str()
            };
            match best {
                Some(c) => {
                    let rank = best_rank.unwrap();
                    let in_top30 = if rank < 30 { "✓" } else { "✗" };
                    println!(
                        "  {:<32} | {:>5.0}Hz | {:>+4.0}  | {:>+4.2} | {:>3}{} | df={:+.1}Hz ddt={:+.2}s s={:.0}",
                        mt,
                        tf,
                        tsnr,
                        tdt,
                        rank + 1,
                        in_top30,
                        c.freq_hz - tf,
                        c.dt_sec - tdt,
                        c.score,
                    )
                }
                None => println!(
                    "  {:<32} | {:>5.0}Hz | {:>+4.0}  | {:>+4.2} |    — | ✗ MISSING from coarse_sync",
                    mt, tf, tsnr, tdt,
                ),
            }
        }
        println!();
        println!("  ── top-30 by score ──");
        for (i, c) in cands.iter().take(30).enumerate() {
            println!(
                "  rank {:>2}: freq={:>5.0} Hz  dt={:>+5.2} s  score={:.0}",
                i + 1,
                c.freq_hz,
                c.dt_sec,
                c.score
            );
        }
        println!();
    }
}
