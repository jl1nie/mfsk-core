//! One-off probe for issue #116 K1BZM DK8NE @244 Hz classification.
//!
//! **Resolved (issue #180/#182 follow-up): DK8NE is real, not a JTDX
//! false positive.** A locally-rebuilt, instrumented jt9 confirms it
//! blind-decodes this exact signal (`iaptype=0`, no AP), and mfsk-core's
//! own SIC residual at these coordinates was verified — down to the
//! per-symbol tone argmax and the LLR reliability ordering OSD actually
//! consumes — to be equivalent to jt9's own residual. The remaining
//! recall gap was `osd_decode_npre1`'s algorithm fidelity vs
//! `osd174_91.f90`'s real ndeep=2 (see #182), not the classification
//! question this file originally asked. The probes below still run and
//! their AP-context sweep is still informative, but the "false positive
//! #5" conclusion they were written to reach is now known incorrect —
//! kept for the historical AP-context data, not as a live hypothesis.
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

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{ApHint, DecodeStrictness};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

fn try_context(label: &str, audio: &[i16], mycall: &str, hiscall: &str) -> BTreeSet<String> {
    let ap = ApHint::new().with_call1(mycall).with_call2(hiscall);

    // Multi-pass + SIC + AP — the strongest AP-on recovery path we have.
    let decoded = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 1.3, 50)
        .strictness(DecodeStrictness::Normal)
        .staged()
        .ap_hint(&ap)
        .decode()
        .results;
    let msgs: BTreeSet<String> = decoded
        .iter()
        .filter_map(|r| unpack77(r.message77()))
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
    let no_ap = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.3, 50)
        .decode()
        .results;
    let no_ap_msgs: BTreeSet<String> = no_ap
        .iter()
        .filter_map(|r| unpack77(r.message77()))
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

    // NOTE (issue #180/#182): this conclusion is now known incorrect —
    // DK8NE is a real signal (jt9 blind-decodes it) and the recall gap
    // was root-caused to `osd_decode_npre1` fidelity, not missing
    // mutual information. Kept for the historical AP-context sweep data.
    println!("\n  → if all of ctx1..ctx5 + baseline are 'miss', the 244 Hz site does not contain");
    println!("    enough mutual info with the DK8NE codeword for any of our paths to recover.");
    println!(
        "    [HISTORICAL, since disproven by #180/#182] would classify K1BZM DK8NE @244 as JTDX false positive #5."
    );
}

fn try_context_for(label: &str, audio: &[i16], mycall: &str, hiscall: &str, target: &str) -> bool {
    let ap = ApHint::new().with_call1(mycall).with_call2(hiscall);
    let decoded = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 1.3, 50)
        .strictness(DecodeStrictness::Normal)
        .staged()
        .ap_hint(&ap)
        .decode()
        .results;
    let msgs: BTreeSet<String> = decoded
        .iter()
        .filter_map(|r| unpack77(r.message77()))
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

/// Before trusting the "no recoverable content" conclusion above: does
/// `coarse_sync` even find a candidate near 2546 Hz at all? If not, the
/// AP contexts in `probe_wa2fzw_with_exact_context` never had a chance
/// to try in the first place (a structural gap, not evidence the claim
/// is false) — if a candidate *is* found there and every AP context
/// still misses, that's real evidence against the codeword content,
/// not just "we never looked."
#[test]
#[ignore]
fn probe_wa2fzw_coarse_sync_candidates() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let spec = mfsk_core::ft8::decode_block::compute_spectrogram(&audio, 3000.0);
    let candidates = mfsk_core::ft8::decode_block::coarse_sync(&spec, 100.0, 3000.0, 0.8, 200);
    let near: Vec<_> = candidates
        .iter()
        .filter(|c| (c.freq_hz - 2546.0).abs() <= 10.0)
        .collect();
    println!("\n=== coarse_sync near 2546 Hz (WA2FZW DL5AXX RR73 claimed site) ===");
    println!(
        "{} candidates total, {} within ±10 Hz of 2546 Hz",
        candidates.len(),
        near.len()
    );
    for c in &near {
        println!(
            "  cand freq={:.2} dt={:.3} score={:.4}",
            c.freq_hz, c.dt_sec, c.score
        );
    }
    if near.is_empty() {
        println!(
            "  → no candidate at all near 2546 Hz: the AP probes above never had a\n    real shot — this is a coarse-sync gap, not evidence against the codeword."
        );
    } else {
        println!(
            "  → candidate(s) exist near 2546 Hz; every AP context in\n    probe_wa2fzw_with_exact_context still missed, which is real evidence\n    the codeword content doesn't match DL5AXX RR73."
        );
    }
}
