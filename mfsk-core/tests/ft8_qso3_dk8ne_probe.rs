//! One-off probe for issue #116 K1BZM DK8NE @244 Hz classification.
//!
//! Tries `decode_frame_subtract_with_ap` with the *exact* operator
//! context (mycall=K1BZM, hiscall=DK8NE) — if even that doesn't
//! surface DK8NE, it is JTDX false positive #5 (the signal at 244 Hz
//! at SNR -19 dB does not contain enough mutual information with the
//! DK8NE codeword for any of our paths to recover).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_dk8ne_probe -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::decode::{
    ApHint, DecodeDepth, DecodeStrictness, decode_frame_subtract_with_ap, decode_frame_with_ap,
};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

fn try_context(label: &str, audio: &[i16], mycall: &str, hiscall: &str) -> BTreeSet<String> {
    let ap = ApHint::new().with_call1(mycall).with_call2(hiscall);

    // Multi-pass + SIC + AP — the strongest AP-on recovery path we have.
    let decoded = decode_frame_subtract_with_ap(
        audio,
        100.0,
        3000.0,
        1.3,
        None,
        DecodeDepth::BpAllOsd,
        50,
        DecodeStrictness::Normal,
        Some(&ap),
    );
    let msgs: BTreeSet<String> = decoded
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect();
    let hit_dk8ne = msgs.contains("K1BZM DK8NE -10");
    println!(
        "  [{label}] mycall={mycall:<8} hiscall={hiscall:<8} → {} decodes, K1BZM DK8NE -10: {}",
        msgs.len(),
        if hit_dk8ne { "HIT" } else { "miss" }
    );
    msgs
}

#[test]
#[ignore]
fn probe_dk8ne_with_exact_context() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));

    println!("\n=== K1BZM DK8NE @244 Hz / SNR -19 dB — AP context sweep ===");

    // Baseline: no AP context (single-pass, host AP-off equivalent).
    let no_ap = decode_frame_with_ap(
        &audio,
        100.0,
        3000.0,
        1.3,
        None,
        DecodeDepth::BpAllOsd,
        50,
        None,
    );
    let no_ap_msgs: BTreeSet<String> = no_ap
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect();
    println!(
        "  [baseline] AP-off single-pass → {} decodes, K1BZM DK8NE -10: {}",
        no_ap_msgs.len(),
        if no_ap_msgs.contains("K1BZM DK8NE -10") {
            "HIT"
        } else {
            "miss"
        }
    );

    // Try each plausible operator-context configuration.
    try_context("ctx1", &audio, "K1BZM", "DK8NE"); // exact match
    try_context("ctx2", &audio, "DK8NE", "K1BZM"); // swap roles
    try_context("ctx3", &audio, "K1BZM", "K1JT"); // partial (mycall=K1BZM only useful)
    try_context("ctx4", &audio, "K1JT", "DK8NE"); // partial (hiscall=DK8NE only useful)
    try_context("ctx5", &audio, "K1JT", "HA0DU"); // unrelated context (sanity)

    println!("\n  → if all of ctx1..ctx5 + baseline are 'miss', the 244 Hz site does not contain");
    println!("    enough mutual info with the DK8NE codeword for any of our paths to recover.");
    println!(
        "    That classifies K1BZM DK8NE @244 as JTDX false positive #5 (=> true ceiling 13/13)."
    );
}

fn try_context_for(label: &str, audio: &[i16], mycall: &str, hiscall: &str, target: &str) -> bool {
    let ap = ApHint::new().with_call1(mycall).with_call2(hiscall);
    let decoded = decode_frame_subtract_with_ap(
        audio,
        100.0,
        3000.0,
        1.3,
        None,
        DecodeDepth::BpAllOsd,
        50,
        DecodeStrictness::Normal,
        Some(&ap),
    );
    let msgs: BTreeSet<String> = decoded
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect();
    let hit = msgs.contains(target);
    println!(
        "  [{label}] mycall={mycall:<8} hiscall={hiscall:<8} → {} decodes, {}: {}",
        msgs.len(),
        target,
        if hit { "HIT" } else { "miss" }
    );
    hit
}

#[test]
#[ignore]
fn probe_wa2fzw_with_exact_context() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));

    println!("\n=== WA2FZW DL5AXX RR73 @2546 Hz / SNR -15 dB — AP context sweep ===");

    let target = "WA2FZW DL5AXX RR73";
    let _ = try_context_for("ctx1", &audio, "WA2FZW", "DL5AXX", target);
    let _ = try_context_for("ctx2", &audio, "DL5AXX", "WA2FZW", target);
    let _ = try_context_for("ctx3", &audio, "WA2FZW", "K1JT", target);
    let _ = try_context_for("ctx4", &audio, "K1JT", "DL5AXX", target);

    println!(
        "\n  → if all are 'miss', WA2FZW DL5AXX RR73 @2546 has no recoverable codeword content"
    );
    println!(
        "    even with exact operator AP context — confirms JTDX false positive #4 (sync-artifact)."
    );
}
