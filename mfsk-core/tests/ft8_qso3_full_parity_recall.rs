//! Hard-assertion regression — WSJT-X **full parity** decode of the
//! reference WAV (`samples/FT8/210703_133430.wav` =
//! `embedded-poc/assets/qso3_busy.wav`) under the **host research
//! config** (`DecodeDepth::FULL`, `sync_min=0.8`, `max_cand=60` — the
//! same default already used by `ft8_qso3_jtdx_recall.rs` for the
//! larger JTDX-18 comparison).
//!
//! Distinct from `ft8_qso3_apoff_recall.rs`'s **ship config**
//! (`DecodeDepth::EMBEDDED`, `sync_min=1.3`, `max_cand=15` — the real
//! Core2/S3 production path, which never runs OSD and floors at 7/8
//! missing `K1BZM DK8NE -10`). This test tracks the separate question
//! "how fast can host achieve full WSJT-X-8-entry parity" — the
//! host-side speed/recall target, not what ships on hardware. Do not
//! conflate the two: `DecodeDepth::FULL` is host-only by construction
//! (see its own doc comment) and never runs on an ESP32 target.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_full_parity_recall -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Same 8-entry WSJT-X AP-off golden as `ft8_qso3_apoff_recall.rs` —
/// see that file's own doc comment for provenance
/// (`reference_qso3_busy_wsjtx_decode.md`). Duplicated rather than
/// shared across files to keep each regression test self-contained
/// and independently greppable.
struct GoldenEntry {
    msg: &'static str,
    snr_db: f32,
    df_hz: f32,
}
const WSJTX_GOLDEN: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "CQ F5RXL IN94",
        snr_db: -3.0,
        df_hz: 1197.0,
    },
    GoldenEntry {
        msg: "N1JFU EA6EE R-07",
        snr_db: -13.0,
        df_hz: 641.0,
    },
    GoldenEntry {
        msg: "A92EE F5PSR -14",
        snr_db: -9.0,
        df_hz: 723.0,
    },
    GoldenEntry {
        msg: "W0RSJ EA3BMU RR73",
        snr_db: -15.0,
        df_hz: 400.0,
    },
    GoldenEntry {
        msg: "K1JT HA0DU KN07",
        snr_db: -15.0,
        df_hz: 590.0,
    },
    GoldenEntry {
        msg: "N1PJT HB9CQK -10",
        snr_db: -2.0,
        df_hz: 466.0,
    },
    GoldenEntry {
        msg: "K1BZM DK8NE -10",
        snr_db: -17.0,
        df_hz: 244.0,
    },
    GoldenEntry {
        msg: "KD2UGC F6GCP R-23",
        snr_db: -6.0,
        df_hz: 472.0,
    },
];

// Same tolerance split as `ft8_qso3_apoff_recall.rs`: fixed-point uses
// adjacent-tone SNR (no xsnr2 post-process), which can sit further off
// (N1PJT Δ-13.5 dB on this WAV) than host f32's xsnr2/xbase estimate.
#[cfg(not(feature = "fixed-point"))]
const SNR_TOL_DB: f32 = 12.0;
#[cfg(feature = "fixed-point")]
const SNR_TOL_DB: f32 = 14.0;
const DF_TOL_HZ: f32 = 5.0;

/// Full parity floor: all 8 WSJT-X AP-off golden entries, including
/// `K1BZM DK8NE -10` which `ship-config`'s `DecodeDepth::EMBEDDED`
/// structurally cannot reach (no OSD). Measured 2026-07-26: 8/8, 19
/// total decodes, ~139-140ms (single- or multi-threaded — this
/// candidate count doesn't parallelise much), on an AMD Ryzen 9 9900X.
/// For comparison, real `jt9 -8 -d3`'s own measured total file decode
/// time is ~1.1s (`FT8_BENCHMARK.md`) — full parity here runs ~7-8x
/// faster than jt9 itself.
///
/// Also holds under `fixed-point` (`LlrT = Q11i16` on host, the same
/// numeric path embedded ships): still 8/8, 20 total decodes,
/// ~162ms — `DecodeDepth::FULL`'s OSD dispatch operates on `&[f32]`
/// regardless of the embedded `LlrT` choice (see `osd_strategy.rs`'s
/// own doc comment), so full parity isn't host-f32-exclusive. This
/// isn't the embedded *ship* config though (that's `EMBEDDED`, tested
/// separately in `ft8_qso3_apoff_recall.rs`) — real Xtensa silicon
/// never runs OSD (see `DecodeDepth::osd`'s doc comment) — this just
/// confirms the fixed-point *arithmetic* path itself doesn't leave
/// anything on the table if a future host-side use case wanted it.
const MIN_GOLDEN_HITS: usize = 8;

/// Cap on total output — same rationale as `ft8_qso3_apoff_recall.rs`:
/// not a phantom-zero requirement (JTDX-confirmed extras are fine,
/// see `ft8_qso3_jtdx_recall.rs`), just a catastrophic-regression net.
const MAX_TOTAL_DECODES: usize = 30;

#[test]
fn qso3_full_parity_meets_wsjtx_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // Host research config (same default as ft8_qso3_jtdx_recall.rs):
    // DecodeDepth::FULL (OSD on — host-only by construction, never
    // runs on embedded), sync_min=0.8, max_cand=60.
    let t0 = Instant::now();
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 0.8, DecodeDepth::FULL, 60);
    let elapsed = t0.elapsed();

    let mut by_msg: std::collections::HashMap<String, &mfsk_core::ft8::decode::DecodeResult> =
        std::collections::HashMap::new();
    for r in &decoded {
        if let Some(text) = unpack77(r.message77()) {
            by_msg.insert(text, r);
        }
    }

    println!("\nqso3 host full-parity config (FULL/0.8/60) — content match vs WSJT-X golden:");
    println!(
        "  {:<22} {:>9} {:>9} {:>7} {:>9} {:>9} {:>4}",
        "callsign chunk", "gold DF", "ours DF", "ours dt", "gold SNR", "ours SNR", "e"
    );
    let mut golden_hits = 0usize;
    let mut snr_outliers: Vec<String> = Vec::new();
    let mut df_outliers: Vec<String> = Vec::new();
    for g in WSJTX_GOLDEN {
        match by_msg.get(g.msg) {
            Some(r) => {
                golden_hits += 1;
                let dsnr = r.snr_db - g.snr_db;
                let ddf = r.freq_hz - g.df_hz;
                let snr_ok = dsnr.abs() <= SNR_TOL_DB;
                let df_ok = ddf.abs() <= DF_TOL_HZ;
                let mark_snr = if snr_ok { " " } else { "!" };
                let mark_df = if df_ok { " " } else { "!" };
                println!(
                    "  ✓ {:<20} {:>9.1} {:>8.1}{} {:>+7.3} {:>8.1} {:>8.1}{} {:>3}  '{}'",
                    g.msg.split_whitespace().next().unwrap_or(""),
                    g.df_hz,
                    r.freq_hz,
                    mark_df,
                    r.dt_sec,
                    g.snr_db,
                    r.snr_db,
                    mark_snr,
                    r.hard_errors,
                    g.msg,
                );
                if !snr_ok {
                    snr_outliers.push(format!("{} (Δ={:+.1} dB)", g.msg, dsnr));
                }
                if !df_ok {
                    df_outliers.push(format!("{} (Δ={:+.1} Hz)", g.msg, ddf));
                }
            }
            None => {
                println!(
                    "  ✗ {:<20} {:>9.1} {:>9} {:>9.1} {:>9} {:>3}  '{}' (missing)",
                    g.msg.split_whitespace().next().unwrap_or(""),
                    g.df_hz,
                    "—",
                    g.snr_db,
                    "—",
                    "—",
                    g.msg,
                );
            }
        }
    }

    let golden_msgs: BTreeSet<String> = WSJTX_GOLDEN.iter().map(|g| g.msg.to_string()).collect();
    let phantoms: Vec<&mfsk_core::ft8::decode::DecodeResult> = decoded
        .iter()
        .filter(|r| {
            unpack77(r.message77())
                .map(|t| !golden_msgs.contains(&t))
                .unwrap_or(false)
        })
        .collect();
    if !phantoms.is_empty() {
        println!("\nphantoms ({}):", phantoms.len());
        for r in &phantoms {
            if let Some(text) = unpack77(r.message77()) {
                println!(
                    "  ✗ {:>4.0} Hz  SNR={:>5.1} dB  e={:>2}  '{}'",
                    r.freq_hz, r.snr_db, r.hard_errors, text
                );
            }
        }
    }
    println!(
        "\n  → {}/{} golden hit, {} phantom, {} total, {:.1} ms",
        golden_hits,
        WSJTX_GOLDEN.len(),
        phantoms.len(),
        decoded.len(),
        elapsed.as_secs_f64() * 1000.0,
    );

    assert!(
        golden_hits >= MIN_GOLDEN_HITS,
        "qso3 full-parity recall regression: {} of {} WSJT-X golden, floor {}",
        golden_hits,
        WSJTX_GOLDEN.len(),
        MIN_GOLDEN_HITS,
    );
    assert!(
        decoded.len() <= MAX_TOTAL_DECODES,
        "qso3 phantom regression: {} total decodes, ceiling {}",
        decoded.len(),
        MAX_TOTAL_DECODES,
    );
    assert!(
        snr_outliers.is_empty(),
        "qso3 SNR drift outside ±{:.0} dB on matched callsigns: {:?}",
        SNR_TOL_DB,
        snr_outliers,
    );
    assert!(
        df_outliers.is_empty(),
        "qso3 DF drift outside ±{:.0} Hz on matched callsigns: {:?}",
        DF_TOL_HZ,
        df_outliers,
    );
}
