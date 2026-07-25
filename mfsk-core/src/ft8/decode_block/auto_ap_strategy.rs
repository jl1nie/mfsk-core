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

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use super::super::decode::{ApHint, DecodeDepth, DecodeResult, DecodeStrictness};
use super::super::llr::sync_quality;
use super::coarse_sync::coarse_sync;
use super::fill_symbol_spectra::{SymMask, fill_symbol_spectra};
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
pub(crate) fn extract_caller_callsigns(decodes: &[DecodeResult]) -> Vec<String> {
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
///
/// Widens the candidate search 4x (`max_cand.saturating_mul(4).max(200)`,
/// `coarse_sync` capped at 200 regardless of `max_cand`) to chase weak
/// signals out of the normal ranking — ~4.8s measured on
/// `qso3_busy.wav`. See [`run_bounded`] for a cheaper variant that
/// skips the widening (issue #180 follow-up).
///
/// **Correction (issue #180 follow-up, re-verified against
/// `ft8apset.f90`'s actual source)**: this module's whole approach —
/// self-seeding AP `mycall` from same-slot decoded callsigns — has no
/// counterpart in real WSJT-X at all, for *any* run configuration.
/// `ft8apset.f90`'s first line is `if(len(trim(mycall12)).lt.3) return`
/// — real jt9/WSJT-X only ever applies AP using the *operator's own*
/// configured `mycall`/`hiscall` (from `-c`/`-x` or the GUI), and
/// disables AP entirely (every `apsym` bit left unconstrained) the
/// moment that isn't set. There is no "recently heard" hash table
/// feeding AP automatically from a slot's own decodes. A real
/// `jt9 -8 -d 3` run with no `-c`/`-x` (as used throughout issue #180's
/// own investigation) therefore has AP **fully disabled** — every
/// decode it produces, `K1BZM DK8NE -10` included, is blind
/// (`iaptype=0`). This module is a genuine mfsk-core-original
/// extension beyond WSJT-X's real capability, not a port of one — see
/// the module-level doc comment above, which already said as much
/// ("Departs from WSJT-X parity"). Older comments in this file and in
/// `ft8::decode::decode_frame_subtract_with_auto_ap` describing this
/// as mirroring "WSJT-X's own recently-heard-callsign hash table" were
/// wrong and should be read as referring only to this module's own
/// design, not to any real jt9 behaviour.
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
    const AUTO_AP_PASS1_LIMIT: usize = 200;
    let auto_ap_max_cand = max_cand.saturating_mul(4).max(200);
    run_bounded(
        audio,
        freq_min,
        freq_max,
        sync_min,
        AUTO_AP_PASS1_LIMIT,
        auto_ap_max_cand,
        existing,
        depth,
        bp_max_iter,
        strictness,
    )
}

/// Core of [`run`], with the pass-1 coarse-sync cap and the refine-
/// stage candidate cap taken as explicit parameters instead of `run`'s
/// hardcoded 200 / `max_cand*4` widening.
///
/// Cheaper host variant (issue #180 follow-up,
/// [`crate::ft8::decode::decode_frame_subtract_with_auto_ap`]): calling
/// with `pass1_limit = refine_cap = max_cand` (the *same*, unwidened
/// size the blind multipass already searched) reuses that already-
/// sufficient candidate list instead of `run`'s 4x/200 expansion — for
/// signals whose own coarse-sync candidate already ranks inside the
/// normal `max_cand` list (confirmed directly for `K1BZM DK8NE -10`:
/// rank `#60` of `max_cand=60` on `qso3_busy.wav`), this closes most of
/// the cost this module's own self-seeded-AP search adds, without
/// losing the recall. Not a port of any real jt9 mechanism — see the
/// correction on [`run`] above: WSJT-X's own AP only ever uses the
/// operator-configured `mycall`/`hiscall`, with no automatic seeding
/// from same-slot decodes. Real `jt9 -8 -d 3` (no `-c`/`-x`) decodes
/// `K1BZM DK8NE -10` **blind** in ~1.1s total for the whole file — this
/// module doesn't reproduce however jt9's own blind decode reaches
/// that signal; it reaches the same message through a different,
/// AP-assisted route unique to mfsk-core.
#[allow(clippy::too_many_arguments)]
pub(crate) fn run_bounded<S>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    pass1_limit: usize,
    refine_cap: usize,
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
    let cands = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit);
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
    // `audio` is the original (unsubtracted) slot audio and is never
    // mutated by this function, so one 192k-FFT cache covers both the
    // `refine_candidates` build below and every per-callsign batch call
    // further down — same win as the main multipass driver's cache.
    let audio_i16: Vec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let fft_cache = crate::ft8::downsample::build_fft_cache(&audio_i16);
    let refined: Vec<RefinedCandidate> =
        refine_candidates(audio, cands, refine_cap, Some(&fft_cache));

    // Pre-sync-gate once, *before* the per-callsign loop below —
    // `sync_quality` doesn't depend on `ap_hint` at all, so computing
    // it once here instead of once per (candidate, callsign) pair
    // inside `process_candidates_tuned_with_ap_ref` avoids redundantly
    // re-running the same SyncBlocks12 DFT fill + sync_quality check
    // for every harvested callsign. Measured on `qso3_busy.wav`
    // (issue #180 follow-up): of 66 refined candidates × 15 harvested
    // callsigns (990 pairs), only ~8 candidates ever have real enough
    // sync to be worth an AP retry at all — the other ~958 pairs were
    // paying the sync-gate cost purely to re-derive "no" fifteen times
    // over. This also leaves each surviving candidate's cached `cs`
    // pre-filled through `SyncBlocks12`, so the per-callsign loop's own
    // `fill(..., SyncBlocks12)` becomes a cheap no-op refill from
    // already-computed data rather than fresh DFTs.
    let mut refined = refined;
    refined.retain_mut(|(cand, cs_box, _q_block0)| {
        fill_symbol_spectra(
            cs_box,
            audio,
            cand.freq_hz,
            cand.dt_sec,
            SymMask::SyncBlocks12,
            Some(&fft_cache),
        );
        sync_quality(cs_box) > DEFAULT_Q_THRESH
    });

    // Blind-decode pre-check: candidates that pass the sync gate are
    // "real enough to be worth a full decode attempt" — but most of
    // those, on a real WAV, are just *other* already-decoded signals
    // whose auto-AP coarse_sync coordinates happen to land just
    // outside the ±3 Hz/±0.5 s "already decoded" dedup tolerance
    // above, not new AP-only-recoverable ones. Every sync-gate
    // survivor pays the *same* decode cost on *every* harvested
    // callsign regardless of whether AP was ever going to matter for
    // it. A signal that already decodes *blind* (no AP hint at all)
    // never needed AP in the first place, so one blind batch call here
    // — same cost as a single callsign's worth of work — identifies
    // and drops those before the ×callsigns loop, leaving only
    // genuinely AP-dependent candidates (typically 1-2, e.g. `K1BZM
    // DK8NE -10`) for the expensive part.
    //
    // `DecodeDepth::BpAll` (not the caller's own `depth`, always
    // `BpAllOsd` — see the gate above): OSD is the dominant per-
    // candidate cost (measured: skipping it here cut this whole
    // function's wall-clock roughly in half, 2.2s → 1.3s on
    // `qso3_busy.wav`, with no recall loss), and every AP-rescued
    // signal found so far converges via BP alone once the AP hint's
    // 28-bit constraint is applied — matching this module's own
    // original rationale ("a 28-bit caller-callsign constraint... is
    // enough to bring BP into convergence"). If a future signal turns
    // out to need AP *and* OSD together, this trade-off is the first
    // place to revisit.
    let blind_results = process_candidates_tuned_with_ap_ref(
        audio,
        &refined,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        bp_max_iter,
        None,
        strictness,
        Some(&fft_cache),
    );
    let mut out: Vec<DecodeResult> = Vec::new();
    for mut r in blind_results {
        if let Some(pos) = refined.iter().position(|c| {
            (c.0.freq_hz - r.freq_hz).abs() < 0.1 && (c.0.dt_sec - r.dt_sec).abs() < 0.01
        }) {
            refined.remove(pos);
        }
        // A candidate that decodes blind never needed AP — but if its
        // message is genuinely new (not already in `existing`, e.g. a
        // signal `run_bounded`'s wider coarse_sync happened to surface
        // that the caller's own blind pass missed), keep the find
        // rather than silently dropping it.
        if !existing.iter().any(|x| x.message77 == r.message77) {
            r.pass = PASS_ID_AUTO_AP;
            out.push(r);
        }
    }

    // Per-callsign batch processing, one Rayon task per harvested
    // callsign — this is why the presync/blind-decode filters above
    // matter for more than raw op count: by the time this loop runs,
    // `refined` is down to a handful of genuinely-synced candidates
    // (typically 1-7 on a real WAV), so each callsign's own batch call
    // is cheap and the 15-ish callsigns are fully independent (a
    // different `ApHint::call1` each, same read-only `refined`/`audio`
    // /`fft_cache`) — an easy fit for the 15-way (or however many
    // callsigns) parallelism this crate already uses for the analogous
    // per-candidate loop in `ft8::decode`'s host driver. Each task
    // computes against the *full* `refined` list rather than a
    // shrinking one (the pre-parallelisation version removed a
    // candidate from `remaining` once some callsign decoded it, so
    // later callsigns wouldn't redundantly re-attempt it) — safe to
    // drop now that there are only a handful of candidates left to
    // begin with, and required for parallel workers to stay
    // independent (no shared mutable state to race on).
    //
    // Gemini PR #118 high-priority optimisation (still applies within
    // each task): passing one (cand, callsign) pair at a time
    // re-allocates `BpScratch` (~12 KB) on every invocation; batching
    // all `refined` candidates per callsign reuses the BpScratch inside
    // `process_candidates_tuned_with_ap_ref`'s inner loop.
    #[cfg(feature = "parallel")]
    let all_batches: Vec<Vec<DecodeResult>> = callsigns
        .par_iter()
        .map(|callsign| {
            let ap = ApHint::new().with_call1(callsign);
            // `DecodeDepth::BpAll` — see the blind-precheck step above
            // for why this loop skips OSD too.
            process_candidates_tuned_with_ap_ref(
                audio,
                &refined,
                DecodeDepth::BpAll,
                DEFAULT_Q_THRESH,
                bp_max_iter,
                Some(&ap),
                strictness,
                Some(&fft_cache),
            )
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let all_batches: Vec<Vec<DecodeResult>> = callsigns
        .iter()
        .map(|callsign| {
            let ap = ApHint::new().with_call1(callsign);
            process_candidates_tuned_with_ap_ref(
                audio,
                &refined,
                DecodeDepth::BpAll,
                DEFAULT_Q_THRESH,
                bp_max_iter,
                Some(&ap),
                strictness,
                Some(&fft_cache),
            )
        })
        .collect();

    for batch_results in all_batches {
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
