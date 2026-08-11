//! Hard-assertion regression — **JTDX, RX Sensitivity = High** decode
//! of the qso3_busy.wav reference (`embedded-poc/assets/qso3_busy.wav`).
//!
//! A superset of `ft8_qso3_jtdx_recall.rs`'s 18-entry default-sensitivity
//! golden: same 18 entries (SNR values within ~1 dB — JTDX's own SNR
//! estimate barely moves with this setting, unlike WSJT-X's) plus 2 new
//! ones this more aggressive profile alone recovers: `CQ F5RXL IN94`
//! (present in the WSJT-X-8 golden but documented in the default-JTDX
//! test as a JTDX miss — High sensitivity picks it up) and `K1JT HA5WA
//! 73` (not in any prior reference at any sensitivity setting).
//!
//! This is a **separate reference profile, not a correction** of the
//! default-sensitivity golden in `ft8_qso3_jtdx_recall.rs` or the
//! WSJT-X-8 golden in `ft8_qso3_apoff_recall.rs` — do not merge or
//! replace those with this one. Source: `reference_qso3_busy_jtdx_decode.md`'s
//! 2026-08-10 addendum.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_jtdx_high_sensitivity_recall -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

struct GoldenEntry {
    msg: &'static str,
    snr_db: f32,
    df_hz: f32,
}

/// JTDX **RX Sensitivity = High**, AP-off, decode of the official
/// sample WAV. 20 entries — the default-sensitivity 18
/// (`ft8_qso3_jtdx_recall.rs::JTDX_GOLDEN`) plus `CQ F5RXL IN94` and
/// `K1JT HA5WA 73`.
const JTDX_HIGH_GOLDEN: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "W1FC F5BZB -08",
        snr_db: 0.0,
        df_hz: 2571.0,
    },
    GoldenEntry {
        msg: "N1API F2VX 73",
        snr_db: -18.0,
        df_hz: 1513.0,
    },
    GoldenEntry {
        msg: "A92EE F5PSR -14",
        snr_db: -9.0,
        df_hz: 723.0,
    },
    GoldenEntry {
        msg: "CQ F5RXL IN94",
        snr_db: -7.0,
        df_hz: 1196.0,
    },
    GoldenEntry {
        msg: "WM3PEN EA6VQ -09",
        snr_db: 0.0,
        df_hz: 2157.0,
    },
    GoldenEntry {
        msg: "XE2X HA2NP RR73",
        snr_db: -14.0,
        df_hz: 2854.0,
    },
    GoldenEntry {
        msg: "K1JT EA3AGB -15",
        snr_db: -15.0,
        df_hz: 1648.0,
    },
    GoldenEntry {
        msg: "N1JFU EA6EE R-07",
        snr_db: -14.0,
        df_hz: 641.0,
    },
    GoldenEntry {
        msg: "N1PJT HB9CQK -10",
        snr_db: -12.0,
        df_hz: 465.0,
    },
    GoldenEntry {
        msg: "K1BZM DK8NE -10",
        snr_db: -19.0,
        df_hz: 244.0,
    },
    GoldenEntry {
        msg: "K1BZM EA3GP -09",
        snr_db: -11.0,
        df_hz: 2695.0,
    },
    GoldenEntry {
        msg: "K1JT HA5WA 73",
        snr_db: -18.0,
        df_hz: 2039.0,
    },
    GoldenEntry {
        msg: "K1JT HA0DU KN07",
        snr_db: -14.0,
        df_hz: 590.0,
    },
    GoldenEntry {
        msg: "W1DIG SV9CVY -14",
        snr_db: -9.0,
        df_hz: 2734.0,
    },
    GoldenEntry {
        msg: "W0RSJ EA3BMU RR73",
        snr_db: -15.0,
        df_hz: 399.0,
    },
    GoldenEntry {
        msg: "KD2UGC F6GCP R-23",
        snr_db: -10.0,
        df_hz: 472.0,
    },
    GoldenEntry {
        msg: "K1BZM EA3CJ JN01",
        snr_db: -13.0,
        df_hz: 2522.0,
    },
    GoldenEntry {
        msg: "N1API HA6FQ -23",
        snr_db: -13.0,
        df_hz: 2239.0,
    },
    GoldenEntry {
        msg: "WA2FZW DL5AXX RR73",
        snr_db: -15.0,
        df_hz: 2546.0,
    },
    GoldenEntry {
        msg: "CQ EA2BFM IN83",
        snr_db: -16.0,
        df_hz: 2279.0,
    },
];

/// Current achieved floor: **19/20** (host f32, research config —
/// same `DecodeDepth::FULL`/`sync_min=0.8`/`max_cand=60` as
/// `ft8_qso3_jtdx_recall.rs`). Every default-sensitivity JTDX entry
/// plus `CQ F5RXL IN94` decodes; `K1JT HA5WA 73` @2039 Hz does not —
/// the one entry this High-sensitivity profile alone recovers that
/// this crate doesn't reach yet. Not chased further: single data
/// point, no independent confirmation this is a stable/reproducible
/// JTDX decode rather than a High-sensitivity-specific edge case.
#[cfg(not(feature = "fixed-point"))]
const MIN_HITS: usize = 19;
#[cfg(feature = "fixed-point")]
const MIN_HITS: usize = 8;

/// Same envelope as `ft8_qso3_jtdx_recall.rs` — JTDX's own SNR
/// estimate barely shifts between default and High sensitivity (the
/// entries shared with the default-18 golden differ by ≤1 dB here),
/// so no separate tolerance is needed for this profile. See that
/// file's own `SNR_TOL_DB` doc comment for the post-issue-#253-
/// follow-up `xsnr2` calibration fix this tolerance now reflects.
#[cfg(not(feature = "fixed-point"))]
const SNR_TOL_DB: f32 = 12.0;
#[cfg(feature = "fixed-point")]
const SNR_TOL_DB: f32 = 14.0;
const DF_TOL_HZ: f32 = 5.0;

use common::load_wav_i16;

/// See `ft8_qso3_jtdx_recall.rs`'s identically-named constant — same
/// two entries, same reason (JTDX-only finds; a real local `jt9 -8
/// -d3` build's own internal `xsnr2` for WM3PEN reads 12.4 dB, not
/// JTDX's claimed 0.0 dB, and generates no candidate near W1FC at
/// all). Excluded from the SNR-drift assertion only.
const JTDX_SNR_GOLDEN_UNRELIABLE: &[&str] = &["WM3PEN EA6VQ -09", "W1FC F5BZB -08"];

// Research-ceiling guard, not a ship-config budget check — same
// rationale as ft8_qso3_jtdx_recall.rs. Run explicitly:
//   cargo test --release --features full \
//       --test ft8_qso3_jtdx_high_sensitivity_recall -- --ignored
#[test]
#[ignore = "research-ceiling regression — run manually, not on CI"]
fn qso3_apoff_meets_jtdx_high_sensitivity_recall_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    #[cfg(not(feature = "fixed-point"))]
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 0.8, DecodeDepth::FULL, 60);
    #[cfg(feature = "fixed-point")]
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::BP_ONLY, 15);

    let mut by_msg: std::collections::HashMap<String, &mfsk_core::ft8::decode::DecodeResult> =
        std::collections::HashMap::new();
    for r in &decoded {
        if let Some(text) = unpack77(r.message77()) {
            by_msg.insert(text, r);
        }
    }

    println!("\nqso3 host research config vs JTDX RX-Sensitivity=High 20-entry golden:");
    println!(
        "  {:<22} {:>9} {:>9} {:>7} {:>9} {:>9} {:>4}",
        "callsign chunk", "gold DF", "ours DF", "ours dt", "gold SNR", "ours SNR", "e"
    );
    let mut hits = 0usize;
    let mut snr_outliers: Vec<String> = Vec::new();
    let mut df_outliers: Vec<String> = Vec::new();
    for g in JTDX_HIGH_GOLDEN {
        match by_msg.get(g.msg) {
            Some(r) => {
                hits += 1;
                let dsnr = r.snr_db - g.snr_db;
                let ddf = r.freq_hz - g.df_hz;
                let snr_ok = dsnr.abs() <= SNR_TOL_DB;
                let df_ok = ddf.abs() <= DF_TOL_HZ;
                println!(
                    "  ✓ {:<20} {:>9.1} {:>8.1}{} {:>+7.3} {:>8.1} {:>8.1}{} {:>3}  '{}'",
                    g.msg.split_whitespace().next().unwrap_or(""),
                    g.df_hz,
                    r.freq_hz,
                    if df_ok { " " } else { "!" },
                    r.dt_sec,
                    g.snr_db,
                    r.snr_db,
                    if snr_ok { " " } else { "!" },
                    r.hard_errors,
                    g.msg,
                );
                if !snr_ok && !JTDX_SNR_GOLDEN_UNRELIABLE.contains(&g.msg) {
                    snr_outliers.push(format!("{} (Δ={:+.1} dB)", g.msg, dsnr));
                }
                if !df_ok {
                    df_outliers.push(format!("{} (Δ={:+.1} Hz)", g.msg, ddf));
                }
            }
            None => {
                println!(
                    "  ✗ {:<20} {:>9.1} {:>9} {:>7} {:>9.1} {:>9} {:>3}  '{}' (missing)",
                    g.msg.split_whitespace().next().unwrap_or(""),
                    g.df_hz,
                    "—",
                    "—",
                    g.snr_db,
                    "—",
                    "—",
                    g.msg,
                );
            }
        }
    }
    let golden_msgs: BTreeSet<String> =
        JTDX_HIGH_GOLDEN.iter().map(|g| g.msg.to_string()).collect();
    let extras: Vec<&mfsk_core::ft8::decode::DecodeResult> = decoded
        .iter()
        .filter(|r| {
            unpack77(r.message77())
                .map(|t| !golden_msgs.contains(&t))
                .unwrap_or(false)
        })
        .collect();
    if !extras.is_empty() {
        println!("\nextras (not in JTDX-High golden):");
        for r in &extras {
            if let Some(text) = unpack77(r.message77()) {
                println!(
                    "  • {:>4.0} Hz  SNR={:>5.1} dB  e={:>2}  '{}'",
                    r.freq_hz, r.snr_db, r.hard_errors, text
                );
            }
        }
    }
    println!(
        "\n  → {}/{} JTDX-High hit, {} extras, {} total",
        hits,
        JTDX_HIGH_GOLDEN.len(),
        extras.len(),
        decoded.len()
    );
    assert!(
        hits >= MIN_HITS,
        "qso3 JTDX-High-sensitivity recall regression: {} of {} golden, floor {}",
        hits,
        JTDX_HIGH_GOLDEN.len(),
        MIN_HITS,
    );
    assert!(
        snr_outliers.is_empty(),
        "qso3 SNR drift outside ±{:.0} dB on JTDX-High-matched callsigns: {:?}",
        SNR_TOL_DB,
        snr_outliers,
    );
    assert!(
        df_outliers.is_empty(),
        "qso3 DF drift outside ±{:.0} Hz on JTDX-High-matched callsigns: {:?}",
        DF_TOL_HZ,
        df_outliers,
    );
}
