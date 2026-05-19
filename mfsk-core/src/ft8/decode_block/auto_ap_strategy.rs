//! Auto-AP iaptype-2 from same-slot decoded callsigns (issue #117).
//!
//! After [`super::process_candidates::decode_block_with_ap`]'s 3-pass
//! driver finishes, this module performs a final coarse-sync pass on
//! the original audio and tries AP iaptype-2 with each callsign
//! extracted from the existing slot's decodes as the AP `mycall`.
//!
//! Rationale: a busy slot frequently contains a strong-signal CQ /
//! caller decode like `K1BZM EA3CJ JN01` plus a weak counterpart like
//! `K1BZM DK8NE -10` at -19 dB SNR. The latter is BP-unrecoverable
//! without AP (LLR sign-agreement ~54 % on `qso3_busy.wav` per
//! issue #40 close) but a 28-bit caller-callsign constraint from
//! iaptype-2 is enough to bring BP into convergence. The callsign is
//! already in our decode set; this module just feeds it back.
//!
//! Departs from WSJT-X parity — `ft8apset.f90` builds `apsym` only
//! from operator-supplied `mycall` and refuses early when mycall isn't
//! configured. Justified by the project's "Rust × embedded DSP
//! differentiation = recall beyond strict WSJT-X parity" positioning:
//! WSJT-X / JTDX goldens that look "AP-off" on busy WAVs typically
//! reflect capture-time operator config not preserved with the WAV.
//!
//! Cost gating:
//!   - host f32 `BpAllOsd` only — embedded ship (`BpAll`) skips
//!     because Xtensa LX7 lacks the wall-clock budget for an
//!     additional per-candidate × per-callsign loop.
//!   - One extra `coarse_sync` + `refine_candidates` (~50-80 ms host).
//!   - Per (candidate, callsign): full `process_one_candidate_inner`
//!     invocation (~80-100 ms). For `qso3_busy.wav` with 60 max_cand
//!     and ~10 distinct callsigns the auto-AP step adds ~5-15 s
//!     wall-clock — significant but bounded, and only runs in the
//!     host research config.

#![cfg(feature = "fft-rustfft")]

extern crate alloc;
use alloc::collections::BTreeSet;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use super::super::decode::{ApHint, DecodeDepth, DecodeResult, DecodeStrictness};
use super::coarse_sync::coarse_sync;
use super::process_candidates::{
    RefinedCandidate, fine_refine_pass1, process_candidates_tuned_with_ap_ref, refine_candidates,
};
use super::spectrogram::compute_spectrogram;
use super::types::{AudioSample, DEFAULT_Q_THRESH};

/// Pass-ID tag for decodes produced by this module. Sits beyond the
/// existing `0..=12` (BP+AP) and `14..=17` (OSD-a/b/c/d) layout so
/// diagnostic tools can attribute auto-AP-rescued decodes
/// unambiguously.
pub(super) const PASS_ID_AUTO_AP: u8 = 18;

/// Auto-AP-specific `hard_errors` ceiling — tighter than the generic
/// BP ceiling (`WSJTX_NHARDERRORS_MAX = 36`) because auto-AP runs
/// many (callsign, candidate) combinations and the false-positive
/// risk scales with the search space. Empirically on `qso3_busy.wav`
/// real auto-AP rescues (K1BZM DK8NE -19 dB e=17) land well below
/// this floor; CRC-luck phantoms at 591 Hz (`CQ JN2ZEM/R AG11` e=27,
/// neighbour-bin leak from a stronger K1JT HA0DU @590 Hz decode) sit
/// above it. Matches `OSD_HARDERRORS_MAX = 22` for the same
/// CRC-luck-phantom-suppression reason.
const AUTO_AP_HARDERRORS_MAX: u32 = 22;

/// Extract the set of distinct caller-callsigns from a decode list.
///
/// For standard FT8 messages the caller is the first token; for `CQ`
/// messages it's the second. Anything that doesn't look like a real
/// callsign (3..=10 alphanumeric with both letter and digit) is
/// dropped via [`looks_like_callsign`].
pub(super) fn extract_caller_callsigns(decodes: &[DecodeResult]) -> Vec<String> {
    let mut set: BTreeSet<String> = BTreeSet::new();
    for d in decodes {
        let Some(text) = crate::msg::wsjt77::unpack77(&d.message77) else {
            continue;
        };
        let mut tokens = text.split_whitespace();
        let first = tokens.next();
        let second = tokens.next();
        let caller = match (first, second) {
            (Some("CQ"), Some(c)) => c,
            (Some(c), _) => c,
            _ => continue,
        };
        if looks_like_callsign(caller) {
            set.insert(caller.to_string());
        }
    }
    set.into_iter().collect()
}

/// Conservative callsign filter — keep only tokens that plausibly are
/// callsigns. Rejects pure-letter / pure-digit / signal-report /
/// `<...>` hashed callsigns. 4-char grid locators (`JN01`) slip
/// through (letter + digit mix); the downstream `ft8apset` /
/// `pack77` rejects non-callsign tokens via round-trip validation,
/// so the worst case is one wasted iaptype-2 attempt per false
/// positive.
fn looks_like_callsign(s: &str) -> bool {
    let len = s.len();
    if !(3..=10).contains(&len) {
        return false;
    }
    if s.starts_with('<') || s.ends_with('>') {
        return false;
    }
    // Reject slashed callsigns (`/P`, `/R`, `JA1ABC/M`). `pack28`
    // doesn't handle them as standard callsigns, so feeding them
    // into iaptype-2 is wasted BP. Real cases of slashed callers
    // are too uncommon to be worth the false-positive overhead.
    if s.contains('/') {
        return false;
    }
    let bytes = s.as_bytes();
    let has_digit = bytes.iter().any(|b| b.is_ascii_digit());
    let has_alpha = bytes.iter().any(|b| b.is_ascii_alphabetic());
    if !(has_alpha && has_digit) {
        return false;
    }
    // Reject standard FT8 report tokens that happen to satisfy the
    // letter+digit rule: "RR73", "RRR" (no digit so already
    // rejected), "73" (length 2 already rejected). RR73 is the only
    // common 4-char alpha+digit token that needs the explicit kill.
    if s == "RR73" {
        return false;
    }
    true
}

/// Top-level entry: rerun the slot's coarse-sync survivors through
/// AP iaptype-2 with each same-slot-decoded callsign as `mycall`.
/// Caller (`decode_block_multipass`) appends the returned decodes
/// to the slot's accumulated result list.
///
/// Returns an empty vec when the gate fires (depth ≠ `BpAllOsd`,
/// no existing decodes, no extractable callsigns).
#[allow(clippy::too_many_arguments)]
pub(super) fn run<S>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    existing: &[DecodeResult],
    depth: DecodeDepth,
    bp_max_iter: u32,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult>
where
    S: AudioSample,
{
    if !matches!(depth, DecodeDepth::BpAllOsd) {
        return Vec::new();
    }
    let callsigns = extract_caller_callsigns(existing);
    if callsigns.is_empty() {
        return Vec::new();
    }

    // Fresh coarse-sync on the original audio. We deliberately don't
    // use the residual `work` from the multipass loop: SIC residuals
    // are cleanest near strong subtracted signals but can have
    // negative-power artifacts elsewhere; the auto-AP target is weak
    // signals that survived sync but failed BP, so the original
    // spectrum is the safer pick.
    let spec = compute_spectrogram(audio, freq_max);
    // PASS1_LIMIT_DEFAULT (=30) keeps only the top-30 coarse-sync
    // candidates by `coarse_sync` rank — that ranking deprioritises
    // weak signals (block-0 q low) like K1BZM DK8NE @-19 dB. For
    // auto-AP we explicitly want the long tail, so use a much higher
    // limit. Cost-bounded by `auto_ap_max_cand` below.
    const AUTO_AP_PASS1_LIMIT: usize = 200;
    let cands = coarse_sync(&spec, freq_min, freq_max, sync_min, AUTO_AP_PASS1_LIMIT);
    drop(spec);
    // Skip coarse-sync candidates that sit at the (freq, dt) of an
    // already-decoded signal — those have already been processed by
    // the standard 3-pass driver and emerged with a decode; re-running
    // them under auto-AP just wastes BP work. Tolerance matches the
    // `subtract_signal_lpf` footprint roughly: ±3 Hz frequency, ±0.5 s
    // dt. Gemini PR #118 medium-priority optimisation.
    let cands: alloc::vec::Vec<_> = cands
        .into_iter()
        .filter(|c| {
            !existing
                .iter()
                .any(|r| (r.freq_hz - c.freq_hz).abs() < 3.0 && (r.dt_sec - c.dt_sec).abs() < 0.5)
        })
        .collect();
    let cands = fine_refine_pass1(audio, cands);
    // Use a generous max_cand for auto-AP — the standard multipass
    // already truncated to `max_cand` (typically 60) by block-0 q,
    // which deprioritises weak signals like K1BZM DK8NE @-19 dB whose
    // block-0 q is small even though the candidate is real. Auto-AP
    // wants those weak candidates because they're precisely where AP
    // rescue pays off. Cap at 4× the regular max_cand to bound cost.
    let auto_ap_max_cand = max_cand.saturating_mul(4).max(200);
    let refined: Vec<RefinedCandidate> = refine_candidates(audio, cands, auto_ap_max_cand);

    // Per-callsign batch processing. Gemini PR #118 high-priority
    // optimisation: passing one (cand, callsign) pair at a time
    // re-allocates `BpScratch` (~12 KB) on every invocation; batching
    // all remaining candidates per callsign reuses the BpScratch
    // inside `process_candidates_tuned_with_ap_ref`'s inner loop and
    // improves cache locality. Candidates that decode are removed
    // from `remaining` so subsequent callsigns don't re-attempt them
    // (semantically equivalent to the original `break` once a
    // (cand, callsign) pair succeeded).
    //
    // Round-2 follow-up: the slice-borrow `_ref` entrypoint lets us
    // pass `&remaining` directly, eliminating the per-callsign
    // `clone_refined` (one ~5 KB `Box` per candidate × per callsign)
    // allocation churn flagged by Gemini.
    let mut remaining: Vec<RefinedCandidate> = refined;
    let mut out: Vec<DecodeResult> = Vec::new();
    for callsign in &callsigns {
        if remaining.is_empty() {
            break;
        }
        let ap = ApHint::new().with_call1(callsign);
        let batch_results = process_candidates_tuned_with_ap_ref(
            audio,
            &remaining,
            depth,
            DEFAULT_Q_THRESH,
            bp_max_iter,
            Some(&ap),
            strictness,
        );
        for mut r in batch_results {
            if existing.iter().any(|x| x.message77 == r.message77)
                || out.iter().any(|x| x.message77 == r.message77)
            {
                continue;
            }
            // Auto-AP-specific tightened hard_errors gate. See
            // [`AUTO_AP_HARDERRORS_MAX`].
            if r.hard_errors > AUTO_AP_HARDERRORS_MAX {
                continue;
            }
            // Remove the candidate that produced this result from
            // future callsign attempts (each candidate decodes to at
            // most one codeword).
            if let Some(pos) = remaining.iter().position(|c| {
                (c.0.freq_hz - r.freq_hz).abs() < 0.1 && (c.0.dt_sec - r.dt_sec).abs() < 0.01
            }) {
                remaining.remove(pos);
            }
            r.pass = PASS_ID_AUTO_AP;
            out.push(r);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn looks_like_callsign_filter_basics() {
        assert!(looks_like_callsign("K1JT"));
        assert!(looks_like_callsign("EA3AGB"));
        assert!(looks_like_callsign("WA2FZW"));
        assert!(!looks_like_callsign("-15"));
        assert!(!looks_like_callsign("RR73"));
        assert!(!looks_like_callsign("CQ"));
        assert!(!looks_like_callsign("<HASH>"));
    }
}
