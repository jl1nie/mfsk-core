//! Regression check for `decode_frame_subtract_staged` (issue #180):
//! WSJT-X-faithful checkpoint SIC (`jt9.f90` `nearly=41/47/50`) vs the
//! existing flat 3-pass `decode_frame_subtract`.
//!
//! # Background
//!
//! Issue #180's ground-truth trace of WSJT-X's *actual* disk-decode path
//! (instrumented `jt9.f90` / `ft8_decode.f90`, not just its exported
//! `.wav` → message behaviour) found that `jt9` never runs a single pass
//! over the whole 15 s slot. It decodes progressively larger buffer
//! *prefixes* at three checkpoints (`nearly=41` → 141_696 samples,
//! `nearly=47` → 162_432 samples subtract-prep only, `nzhsym=50` →
//! 172_800 samples for the final full pass), carrying decoded signals
//! forward across checkpoints and subtracting them from the residual
//! *before* the harder, marginal candidates at the final checkpoint are
//! attempted. `decode_frame_subtract_staged` (`src/ft8/decode.rs`) ports
//! this structure; see its module doc for the full architecture.
//!
//! `CQ DX DL8YHR JO41` (~-17 dB @ 2606 Hz) only decodes in WSJT-X at
//! that final checkpoint, after 13 stronger neighbours have already been
//! removed — something the pre-existing flat pass structurally cannot
//! reproduce, no matter how its sync threshold is tuned, because nothing
//! is ever subtracted from the buffer *before* the residual DL8YHR is
//! found in gets assembled.
//!
//! # Status
//!
//! **Resolved (issue #180, closed).** The remaining gap this doc used to
//! describe (staging the checkpoints alone reached `sync_quality=9`,
//! short of `jt9`'s `nsync=15`) was root-caused to a shape bug in the LPF
//! subtraction kernel itself (`normalized_kernel` in
//! `src/core/dsp/subtract.rs` used `PI / lpf_half` instead of
//! `PI / nfilt`), not a deeper sync/downsample sensitivity gap. Fixing
//! the kernel shape closed it: `decode_frame_subtract_staged` now finds
//! `CQ DX DL8YHR JO41` on this same recording (`has_dl8yhr=true` below is
//! now a hard-asserted regression floor, not a diagnostic print).
//!
//! Verified (2026-07-25) against this real recording:
//! - The staged path finds the same 18/18 golden messages as the flat
//!   `decode_frame_subtract` baseline — no regression from restructuring
//!   the pipeline into checkpoints.
//! - Checkpoint A found 11 signals early (including `W1FC F5BZB`, only
//!   ~35 Hz from DL8YHR's carrier); checkpoint C found the remaining 7
//!   against that cleaned residual.
//! - With the kernel-shape fix, `DL8YHR` decodes too (19/19).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_staged_sic_check -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeStrictness;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// The 18-message set `decode_frame_subtract` (flat 3-pass SIC) already
/// finds on `qso3_busy.wav` — see `ft8_qso3_subtract_fix_check.rs`. The
/// staged path must not lose any of these.
const FLAT_PASS_GOLDEN: &[&str] = &[
    "A92EE F5PSR -14",
    "CQ EA2BFM IN83",
    "CQ F5RXL IN94",
    "K1BZM EA3CJ JN01",
    "K1BZM EA3GP -09",
    "K1JT EA3AGB -15",
    "K1JT HA0DU KN07",
    "K1JT HA5WA 73",
    "KD2UGC F6GCP R-23",
    "N1API F2VX 73",
    "N1API HA6FQ -23",
    "N1JFU EA6EE R-07",
    "N1PJT HB9CQK -10",
    "W0RSJ EA3BMU RR73",
    "W1DIG SV9CVY -14",
    "W1FC F5BZB -08",
    "WM3PEN EA6VQ -09",
    "XE2X HA2NP RR73",
];

use common::load_wav_i16;

#[test]
fn sic_early_matches_sic_rounds_golden_floor() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
        .strictness(DecodeStrictness::Normal)
        .sic_early()
        .decode()
        .results;
    let msgs: BTreeSet<String> = results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();

    println!("staged decode(qso3_busy.wav): {} decodes", msgs.len());
    let mut golden_hits = 0usize;
    for g in FLAT_PASS_GOLDEN {
        let hit = msgs.contains(*g);
        println!("  {} {}", if hit { "✓" } else { "✗" }, g);
        if hit {
            golden_hits += 1;
        }
    }
    let has_dl8yhr = msgs.iter().any(|m| m.contains("DL8YHR"));
    println!("has_dl8yhr={has_dl8yhr}");

    assert_eq!(
        golden_hits,
        FLAT_PASS_GOLDEN.len(),
        "staged SIC regressed vs the flat-pass golden floor: {}/{}",
        golden_hits,
        FLAT_PASS_GOLDEN.len(),
    );
    // Issue #180, closed: the LPF subtraction kernel's shape bug
    // (`normalized_kernel` in `src/core/dsp/subtract.rs`) was the real
    // root cause of DL8YHR's miss, not a deeper sync/downsample
    // sensitivity gap. Now a hard regression floor, not a diagnostic.
    assert!(
        has_dl8yhr,
        "DL8YHR regressed — issue #180's LPF kernel-shape fix should make this a hard hit"
    );
}

/// Issue #191's actual reported bug, end to end: a two-phase pipelined
/// caller (WebFT8's `decode_phase1`/`decode_phase2` shape) that hands a
/// Phase-1 result set + FFT cache into a Phase-2 `.sic_early()` call must
/// get the *same* staged-checkpoint engine `sic_early_matches_sic_rounds_golden_floor`
/// above exercises directly — not a separate, unfixed flat-only path.
///
/// Before #191, `known`/`precomputed_fft` were only accepted by
/// `decode_frame_subtract_with_known_and_ap`, a flat 3-pass loop with
/// none of issue #178/#179/#180's SIC-quality fixes — structurally
/// unable to find `CQ DX DL8YHR JO41` regardless of `known`/cache
/// content (its checkpoint-free structure is exactly what #180's doc
/// comment above explains DL8YHR needs). `SupportsSicEarly`'s
/// `.known()`/`.fft_cache()` handling now routes through the same
/// engine as the plain `.sic_early()` case, so this combination finds it.
#[test]
fn sic_early_with_known_and_cache_finds_dl8yhr() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));

    // Phase 1: a plain single-pass wide-band decode (no SIC) — the
    // shape of WebFT8's `decode_phase1`. Captures both the decoded
    // messages and the FFT cache for reuse.
    let phase1 = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
        .strictness(DecodeStrictness::Normal)
        .decode();
    assert!(
        !phase1.results.is_empty(),
        "phase 1 must decode something for this test to be meaningful"
    );
    assert!(
        !phase1
            .results
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .any(|m| m.contains("DL8YHR")),
        "test setup assumption violated: DL8YHR should NOT be in the plain \
         single-pass phase 1 result set (it should only surface via staged SIC)"
    );

    // Phase 2: staged SIC seeded with phase 1's results + cache — the
    // exact combination issue #191 reported as unreachable.
    let phase2 = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
        .strictness(DecodeStrictness::Normal)
        .sic_early()
        .known(&phase1.results)
        .fft_cache(phase1.fft_cache)
        .decode();

    let phase2_msgs: BTreeSet<String> = phase2
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    println!(
        "phase2 staged+known+cache(qso3_busy.wav): {} new decodes",
        phase2_msgs.len()
    );

    // `known` (phase 1's own results) must not be re-reported.
    for r in &phase1.results {
        assert!(
            !phase2
                .results
                .iter()
                .any(|r2| r2.message77() == r.message77()),
            "phase 2 re-reported a phase-1 `known` message instead of skipping it"
        );
    }

    assert!(
        phase2_msgs.iter().any(|m| m.contains("DL8YHR")),
        "staged SIC with known+cache should find CQ DX DL8YHR JO41 in phase 2 \
         (issue #191 regression: this combination used to route through the \
         unfixed flat-only decode_frame_subtract_with_known_and_ap engine, \
         which cannot reach DL8YHR structurally)"
    );
}

/// Regression guard for issue #253: a reproducible false decode
/// (`7Y8CIH HN1GD OP30` @509 Hz, `hard_errors=31`) found via WebFT8's
/// `decode_phase2` pipeline — `DecodeStrictness::Deep` + `.known()` +
/// `.fft_cache()` + `.sic_early()`, seeded from a fast preliminary
/// phase-1 decode. Root cause: `__staged_sic` used to pre-subtract
/// `known` from the *entire* audio before checkpoint A ran — checkpoint
/// A (the `nzhsym=41` truncated/zero-tailed early pass) in a real `jt9`
/// build is *always* the first decode attempt on fully raw audio for a
/// Rx cycle, so feeding it pre-subtracted audio was an mfsk-core-original
/// composition with no WSJT-X counterpart. Fixed by scoping the
/// `known`-subtraction to checkpoints B/C only (which already rebuild
/// their own buffers fresh from the original audio each time, so this
/// doesn't lose the "known signals don't mask weaker ones" capability —
/// see `sic_early_with_known_and_cache_finds_dl8yhr` above, which still
/// finds `CQ DX DL8YHR JO41` this way), leaving checkpoint A on
/// unmodified audio.
///
/// This test replays the *exact* WebFT8 `decode_phase1`/`decode_phase2`
/// shape (`sync_min=1.5` for phase 1, `1.0` for phase 2, `max_cand=200`,
/// `Deep` strictness) — not the `Normal`/`sync_min=0.8` shape the test
/// above uses — since the false decode was specific to that combination.
#[test]
fn sic_early_deep_with_known_and_cache_does_not_false_decode() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));

    let phase1 = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 200).decode();
    assert!(
        !phase1.results.is_empty(),
        "phase 1 must decode something for this test to be meaningful"
    );

    let phase2 = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.0, 200)
        .strictness(DecodeStrictness::Deep)
        .known(&phase1.results)
        .fft_cache(phase1.fft_cache)
        .sic_early()
        .decode();

    let phase2_msgs: BTreeSet<String> = phase2
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    println!(
        "phase2 Deep+known+cache(qso3_busy.wav): {} new decodes: {:?}",
        phase2_msgs.len(),
        phase2_msgs
    );

    assert!(
        !phase2_msgs.iter().any(|m| m.contains("7Y8CIH")),
        "issue #253 regression: the false decode '7Y8CIH HN1GD OP30' \
         reappeared in phase 2 (got: {phase2_msgs:?})"
    );

    // The real incremental signals this combination should still find
    // beyond phase 1 (issue #253's own investigation confirmed all six
    // remain reachable after the fix, including DL8YHR).
    for expected in [
        "KD2UGC F6GCP R-23",
        "K1BZM EA3CJ JN01",
        "CQ EA2BFM IN83",
        "CQ DX DL8YHR JO41",
        "K1JT HA5WA 73",
        "WA2FZW DL5AXX RR73",
    ] {
        assert!(
            phase2_msgs.contains(expected),
            "issue #253 fix regressed real recall: '{expected}' missing from phase 2 (got: {phase2_msgs:?})"
        );
    }
}
