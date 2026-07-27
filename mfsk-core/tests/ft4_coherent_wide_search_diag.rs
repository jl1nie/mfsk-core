//! Diagnostic (not a regression gate): does `engine::sync::coarse_sync`'s
//! non-coherent wide-Δt search actually land near the true sync peak on
//! weak FT4 trials, or does a directly-computed coherent wide-Δt scan
//! (bypassing coarse_sync entirely) find a much stronger peak elsewhere?
//!
//! Hypothesis (from a WSJT-X source comparison, 2026-07-18): WSJT-X's
//! FT4 decoder (`ft4_decode.f90` + `sync4d.f90`) does its wide Δt search
//! (~350-450 raw samples per segment, step 4 @ 666.7 Hz ≈ 6 ms)
//! *coherently* — complex Costas correlation, not power spectra. Our
//! `engine::sync::coarse_sync` searches a wider Δt window (±2.5 s) but
//! *non-coherently* (magnitude-squared spectrogram bins) at a coarser
//! 24 ms grid, then `sync2d_refine`'s coherent pass only re-examines
//! ±20 downsampled samples (≈±30 ms) around whatever coarse_sync picked.
//! If the non-coherent stage's peak-location estimate is noisy at low
//! SNR and lands more than ~30 ms from the true peak, the coherent
//! refine never reaches it and the candidate is lost — even though a
//! coherent search would have found it.
//!
//! This test scans a full-slot coherent Costas score directly (mirroring
//! what a `fst4_sync_search`-style full-slot coherent scan would do for
//! FT4) at the known-true frequency (1500 Hz, `ft4sim`'s fixed test
//! frequency) and compares:
//!   - where the true coherent peak actually sits (should be near
//!     dt=0.0, the `ft4sim` DT=0.0 nominal)
//!   - what `coarse_sync` picked as its best candidate near 1500 Hz,
//!     and how far that is from the true coherent peak
//!   - whether that distance exceeds `sync2d_refine`'s ±20-sample
//!     coarse radius (which would mean the coherent refine pass can
//!     never recover the true peak from there)
//!
//! Run:
//! ```sh
//! cargo test --release --test ft4_coherent_wide_search_diag \
//!     --features ft4,fft-rustfft,uvpacket -- --ignored --nocapture
//! ```

#![cfg(feature = "ft4")]

use std::path::{Path, PathBuf};

use mfsk_core::engine::dsp::downsample::downsample;
use mfsk_core::engine::sync::{coarse_sync, fine_sync_power};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const GOLDEN_FREQ_HZ: f32 = 1500.0;
/// FT4's `TX_START_OFFSET_S` — nominal frame start in the 90 000-sample
/// (7.5 s) slot `ft4sim` generates with `DT=0.0`.
const TX_START_OFFSET_S: f32 = 0.5;
const DS_RATE: f32 = 12_000.0 / 18.0; // NDOWN=18

fn sweep_dir() -> PathBuf {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/ft4_sweep")
        .to_path_buf()
}

#[test]
#[ignore = "manual diagnostic — FT4 coherent-vs-noncoherent coarse-sync hypothesis"]
fn coherent_wide_scan_vs_coarse_sync_pick() {
    let dir = sweep_dir();

    // A few trials from cells where recall is currently weak
    // (measured 2026-07-18: awgn -16 dB = 4/20, ccir_moderate -16 dB =
    // 1/20) — pick trials 1..8 so both hits and misses are represented,
    // to see whether the pattern differs between them.
    let cells = [("awgn", "m16"), ("ccir_moderate", "m16")];

    for (chan, tag) in cells {
        println!("\n=== {chan} {tag} ===");
        for t in 1..=8u32 {
            let path = dir.join(format!("ft4_{chan}_{tag}_{t:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                println!("  trial {t:02}: (missing, skip)");
                continue;
            };

            // 1. Direct coherent wide-Δt scan at the known-true frequency.
            //    Downsample once at 1500 Hz, then evaluate `fine_sync_power`
            //    (coherent Costas correlation) at every downsampled-sample
            //    offset across the full plausible Δt range (±2.5 s, matching
            //    coarse_sync's own `jz` radius) — this is what a
            //    `fst4_sync_search`-style full-slot coherent search would do.
            let (cd0, _) = downsample(&audio, GOLDEN_FREQ_HZ, &FT4_DOWNSAMPLE);
            let radius_i0 = (2.5 * DS_RATE) as i32;
            let mut best_i0 = 0i32;
            let mut best_score = -1.0f32;
            for i0 in -radius_i0..=radius_i0 {
                let score = fine_sync_power::<Ft4>(&cd0, i0);
                if score > best_score {
                    best_score = score;
                    best_i0 = i0;
                }
            }
            let true_dt_sec = best_i0 as f32 / DS_RATE - TX_START_OFFSET_S;

            // 2. What coarse_sync actually picked near 1500 Hz.
            let cands = coarse_sync::<Ft4>(&audio, 100.0, 3000.0, 0.0, None, 200);
            let near = cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= 5.0)
                .max_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

            match near {
                Some(c) => {
                    let dt_gap_ms = (c.dt_sec - true_dt_sec).abs() * 1000.0;
                    let coherent_score_there = fine_sync_power::<Ft4>(
                        &cd0,
                        ((c.dt_sec + TX_START_OFFSET_S) * DS_RATE).round() as i32,
                    );
                    let out_of_reach = dt_gap_ms > 30.0; // sync2d_refine coarse radius ≈30ms
                    println!(
                        "  trial {t:02}: true_dt={:+.3}s (coherent peak={:.0})  \
                         coarse_sync_dt={:+.3}s (coherent score there={:.0}, non-coh score={:.3})  \
                         gap={:.0}ms{}",
                        true_dt_sec,
                        best_score,
                        c.dt_sec,
                        coherent_score_there,
                        c.score,
                        dt_gap_ms,
                        if out_of_reach {
                            "  <-- OUT OF sync2d_refine REACH"
                        } else {
                            ""
                        },
                    );
                }
                None => {
                    println!(
                        "  trial {t:02}: true_dt={:+.3}s (coherent peak={:.0})  \
                         coarse_sync found NO candidate near {GOLDEN_FREQ_HZ} Hz",
                        true_dt_sec, best_score,
                    );
                }
            }
        }
    }
}
