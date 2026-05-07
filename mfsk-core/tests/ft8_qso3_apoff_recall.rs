//! Hard-assertion regression — WSJT-X **AP-off** decode of the
//! reference WAV (`samples/FT8/210703_133430.wav` =
//! `embedded-poc/assets/qso3_busy.wav`).
//!
//! The 8-entry golden in [`WSJTX_GOLDEN`] is what WSJT-X normal mode
//! produces with **a-priori decoding disabled** (the canonical
//! reproducible reference). A companion AP-on regression will be
//! added when AP-list (ft8b.f90 ipass 5..8) is ported — tracked in
//! https://github.com/jl1nie/mfsk-core/issues/31.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_apoff_recall -- --nocapture
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

/// WSJT-X **AP-off** decode of the official sample WAV
/// (`samples/FT8/210703_133430.wav`). 8 entries — this is the ground
/// truth for AP-disabled regression. Source memory:
/// `reference_qso3_busy_wsjtx_decode.md`.
///
/// JTDX produces a different (larger) set; that is checked separately
/// in `ft8_qso3_jtdx_recall.rs`. Do NOT mix the two references in
/// this test.
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
        msg: "N1JFU EA6EE R-7",
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

/// Tolerance for matching a callsign decode to WSJT-X reference. Our
/// SNR (xsnr2/xbase) systematically sits ~7 dB below WSJT-X (host
/// f32). fixed-point uses adjacent-tone SNR (no xsnr2 post-process)
/// which can sit further off (N1PJT Δ-13 dB on this WAV) but stays
/// within 14 dB.
#[cfg(not(feature = "fixed-point"))]
const SNR_TOL_DB: f32 = 12.0;
#[cfg(feature = "fixed-point")]
const SNR_TOL_DB: f32 = 14.0;
const DF_TOL_HZ: f32 = 5.0;

/// Recall floor against the 8-entry WSJT-X AP-off golden.
/// host f32 baseline = **7/8 hits** (only K1BZM DK8NE -17 dB @244
/// missing). host fixed-point (= bit-identical to embedded Q3i8 LLR
/// build) drops 2 borderline-weak decodes (N1JFU @-13, W0RSJ @-15)
/// to LLR quantisation, leaving **5/8** as the embedded floor.
#[cfg(not(feature = "fixed-point"))]
const MIN_GOLDEN_HITS: usize = 7;
#[cfg(feature = "fixed-point")]
const MIN_GOLDEN_HITS: usize = 5;

/// Cap on total output. The test does NOT consider extras outside
/// the WSJT-X golden as failures — they may be JTDX-confirmed real
/// decodes (gated by the separate `ft8_qso3_jtdx_recall.rs` test).
/// 25 leaves significant headroom while still catching catastrophic
/// CRC-noise regressions.
const MAX_TOTAL_DECODES: usize = 25;

fn load_wav_i16(path: &Path) -> Vec<i16> {
    let bytes = std::fs::read(path).expect("WAV present");
    assert_eq!(&bytes[0..4], b"RIFF", "not a RIFF/WAV file");
    let mut i = 12usize;
    let (mut data_off, mut data_len) = (0usize, 0usize);
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"data" {
            data_off = i;
            data_len = len;
        }
        i += len;
        if len % 2 == 1 {
            i += 1;
        }
    }
    assert!(data_off > 0, "no data chunk in WAV");
    bytes[data_off..data_off + data_len.min(bytes.len() - data_off)]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

#[test]
fn qso3_apoff_meets_wsjtx_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // Ship config: BP only, max_cand=15. Matches the Core2 / S3
    // production path (PASS1 default = 30 via DEFAULT_Q_THRESH).
    // sync_min=1.3 = WSJT-X ft8_decode.f90:176 default for ndepth=3
    // ("Deep" mode). User golden has K1BZM DK8NE -17 dB which only
    // surfaces in Deep mode, so user's reference uses ndepth=3.
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::BpAll, 15);

    // Build (msg → result) for matching.
    let mut by_msg: std::collections::HashMap<String, &mfsk_core::ft8::decode::DecodeResult> =
        std::collections::HashMap::new();
    for r in &decoded {
        if let Some(text) = unpack77(&r.message77) {
            by_msg.insert(text, r);
        }
    }

    println!("\nqso3 ship config (Bp/30/15) — content match vs WSJT-X golden:");
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
            unpack77(&r.message77)
                .map(|t| !golden_msgs.contains(&t))
                .unwrap_or(false)
        })
        .collect();
    if !phantoms.is_empty() {
        println!("\nphantoms ({}):", phantoms.len());
        for r in &phantoms {
            if let Some(text) = unpack77(&r.message77) {
                println!(
                    "  ✗ {:>4.0} Hz  SNR={:>5.1} dB  e={:>2}  '{}'",
                    r.freq_hz, r.snr_db, r.hard_errors, text
                );
            }
        }
    }
    println!(
        "\n  → {}/{} golden hit, {} phantom, {} total",
        golden_hits,
        WSJTX_GOLDEN.len(),
        phantoms.len(),
        decoded.len()
    );

    assert!(
        golden_hits >= MIN_GOLDEN_HITS,
        "qso3 recall regression: {} of {} WSJT-X golden, floor {}",
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
