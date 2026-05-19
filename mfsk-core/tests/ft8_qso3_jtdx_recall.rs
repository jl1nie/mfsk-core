//! Hard-assertion regression — **JTDX** decode of the qso3_busy.wav
//! reference (`embedded-poc/assets/qso3_busy.wav`).
//!
//! JTDX is a more aggressive WSJT-X fork that recovers signals WSJT-X
//! ignores at default settings. The 18-entry golden here is captured
//! from JTDX AP-off and includes everything WSJT-X also finds plus
//! a longer tail of weak / busy-band decodes.
//!
//! This file is the AGGRESSIVE recall regression. The conservative
//! WSJT-X-only check lives in `ft8_qso3_apoff_recall.rs`. **Do not
//! mix the two reference sets** — they capture different decoder
//! ambitions.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_jtdx_recall -- --nocapture
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

/// JTDX **AP-off** decode of the official sample WAV (= more
/// aggressive reference than WSJT-X). 18 entries. F5RXL CQ @1197 is
/// in the WSJT-X golden but NOT in JTDX (a JTDX edge case); all
/// other entries are JTDX-only or shared. Source memory:
/// `reference_qso3_busy_jtdx_decode.md`.
struct GoldenEntry {
    msg: &'static str,
    snr_db: f32,
    df_hz: f32,
}
const JTDX_GOLDEN: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "K1JT EA3AGB -15",
        snr_db: -15.0,
        df_hz: 1648.0,
    },
    GoldenEntry {
        msg: "WM3PEN EA6VQ -9",
        snr_db: 0.0,
        df_hz: 2157.0,
    },
    GoldenEntry {
        msg: "N1PJT HB9CQK -10",
        snr_db: -12.0,
        df_hz: 465.0,
    },
    GoldenEntry {
        msg: "N1JFU EA6EE R-7",
        snr_db: -14.0,
        df_hz: 641.0,
    },
    GoldenEntry {
        msg: "K1BZM DK8NE -10",
        snr_db: -19.0,
        df_hz: 244.0,
    },
    GoldenEntry {
        msg: "W1FC F5BZB -8",
        snr_db: 0.0,
        df_hz: 2571.0,
    },
    GoldenEntry {
        msg: "A92EE F5PSR -14",
        snr_db: -9.0,
        df_hz: 723.0,
    },
    GoldenEntry {
        msg: "XE2X HA2NP RR73",
        snr_db: -14.0,
        df_hz: 2854.0,
    },
    GoldenEntry {
        msg: "N1API F2VX 73",
        snr_db: -18.0,
        df_hz: 1513.0,
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
        msg: "K1JT HA0DU KN07",
        snr_db: -13.0,
        df_hz: 590.0,
    },
    GoldenEntry {
        msg: "N1API HA6FQ -23",
        snr_db: -13.0,
        df_hz: 2239.0,
    },
    GoldenEntry {
        msg: "KD2UGC F6GCP R-23",
        snr_db: -10.0,
        df_hz: 472.0,
    },
    GoldenEntry {
        msg: "CQ EA2BFM IN83",
        snr_db: -15.0,
        df_hz: 2279.0,
    },
    GoldenEntry {
        msg: "K1BZM EA3CJ JN01",
        snr_db: -12.0,
        df_hz: 2522.0,
    },
    GoldenEntry {
        msg: "WA2FZW DL5AXX RR73",
        snr_db: -15.0,
        df_hz: 2546.0,
    },
    GoldenEntry {
        msg: "K1BZM EA3GP -9",
        snr_db: -12.0,
        df_hz: 2696.0,
    },
];

/// Recall floor against JTDX 18.
/// - host f32 (research config: BpAllOsd + max_cand=60 + sync_min=0.8):
///   **13/18 hit** (was 16/18 pre-0.6.3). 0.6.3 added an OSD-pass
///   `hard_errors > 22` ceiling (`OSD_HARDERRORS_MAX` in
///   `decode_block/osd_strategy.rs`) that eliminates 3 CRC-luck
///   phantoms `qso3_busy.wav` previously surfaced from `pass=14`
///   (`N1API F2VX 73` e=30, `N1API HA6FQ -23` e=25, `CQ EA2BFM IN83`
///   e=31). Those 3 were *in* the JTDX golden, so the recall floor
///   drops by exactly 3. The WSJT-X 8-entry golden floor stays at
///   7/8 because none of the dropped phantoms appear there.
/// - host fixed-point (embedded ship config: BpAll + max_cand=15 +
///   sync_min=1.3, **no OSD** because OSD's genmrb + Gauss
///   elimination + thousand-codeword enumeration is not realistic
///   on Xtensa LX7): **8/18 hit**. The 10 missing are -13..-19 dB
///   weak signals where NMS Q3i8 BP can't converge in BP_MAX_ITER=30.
#[cfg(not(feature = "fixed-point"))]
const MIN_JTDX_HITS: usize = 13;
#[cfg(feature = "fixed-point")]
const MIN_JTDX_HITS: usize = 8;

/// Tolerance. host f32 SNR (xsnr2/xbase) sits ~3-7 dB below JTDX,
/// fixed-point uses adjacent-tone SNR with no xsnr2 post-process
/// (cell quantisation breaks log10 baseline) — wider envelope.
#[cfg(not(feature = "fixed-point"))]
const SNR_TOL_DB: f32 = 12.0;
#[cfg(feature = "fixed-point")]
const SNR_TOL_DB: f32 = 14.0;
const DF_TOL_HZ: f32 = 5.0;

use common::load_wav_i16;

// JTDX-aggressive recall is a research-ceiling guard, not a ship-config
// budget check (uses BpAllOsd + sync_min=0.8 + max_cand=60 — ~34 s
// wall-clock per run in release). `ft8_qso3_apoff_recall.rs` already
// covers the production-host budget against the WSJT-X golden floor.
// Run explicitly when investigating aggressive-recall regressions:
//   cargo test --release --features full --test ft8_qso3_jtdx_recall \
//       -- --ignored
#[test]
#[ignore = "research-ceiling regression — run manually, not on CI"]
fn qso3_apoff_meets_jtdx_recall_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    // Host f32 keeps OSD on (host has compute headroom — used for
    // researching the recall ceiling). Host fixed-point mirrors the
    // **embedded ship config**: BpAll only, OSD disabled (OSD's
    // genmrb + Gauss elimination + multi-thousand codeword
    // enumeration is not realistic on Xtensa LX7), max_cand=15,
    // sync_min=1.3 = ndepth=3 default. Real-on-hardware regression.
    #[cfg(not(feature = "fixed-point"))]
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 0.8, DecodeDepth::BpAllOsd, 60);
    #[cfg(feature = "fixed-point")]
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::BpAll, 15);

    let mut by_msg: std::collections::HashMap<String, &mfsk_core::ft8::decode::DecodeResult> =
        std::collections::HashMap::new();
    for r in &decoded {
        if let Some(text) = unpack77(&r.message77) {
            by_msg.insert(text, r);
        }
    }

    println!("\nqso3 ship config (Bp/30/15) vs JTDX 18-entry golden:");
    println!(
        "  {:<22} {:>9} {:>9} {:>7} {:>9} {:>9} {:>4}",
        "callsign chunk", "gold DF", "ours DF", "ours dt", "gold SNR", "ours SNR", "e"
    );
    let mut hits = 0usize;
    let mut snr_outliers: Vec<String> = Vec::new();
    let mut df_outliers: Vec<String> = Vec::new();
    for g in JTDX_GOLDEN {
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
                if !snr_ok {
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
    let golden_msgs: BTreeSet<String> = JTDX_GOLDEN.iter().map(|g| g.msg.to_string()).collect();
    let extras: Vec<&mfsk_core::ft8::decode::DecodeResult> = decoded
        .iter()
        .filter(|r| {
            unpack77(&r.message77)
                .map(|t| !golden_msgs.contains(&t))
                .unwrap_or(false)
        })
        .collect();
    if !extras.is_empty() {
        println!("\nextras (not in JTDX golden — could be WSJT-X-only or other):");
        for r in &extras {
            if let Some(text) = unpack77(&r.message77) {
                println!(
                    "  • {:>4.0} Hz  SNR={:>5.1} dB  e={:>2}  '{}'",
                    r.freq_hz, r.snr_db, r.hard_errors, text
                );
            }
        }
    }
    println!(
        "\n  → {}/{} JTDX hit, {} extras, {} total",
        hits,
        JTDX_GOLDEN.len(),
        extras.len(),
        decoded.len()
    );
    assert!(
        hits >= MIN_JTDX_HITS,
        "qso3 JTDX recall regression: {} of {} JTDX-golden, floor {}",
        hits,
        JTDX_GOLDEN.len(),
        MIN_JTDX_HITS,
    );
    assert!(
        snr_outliers.is_empty(),
        "qso3 SNR drift outside ±{:.0} dB on JTDX-matched callsigns: {:?}",
        SNR_TOL_DB,
        snr_outliers,
    );
    assert!(
        df_outliers.is_empty(),
        "qso3 DF drift outside ±{:.0} Hz on JTDX-matched callsigns: {:?}",
        DF_TOL_HZ,
        df_outliers,
    );
}
