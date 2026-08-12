//! Δt search-window parity with the reference decoder (issue #282).
//!
//! ## Why this file exists
//!
//! #280 found that FT8's coarse-sync lag window had silently drifted
//! from WSJT-X's and was load-bearing for golden recall. The audit
//! that followed (#282) found the same class of drift in three more
//! protocols — and, critically, that **nothing in the suite could
//! have caught it**: every sweep corpus is generated at Δt = 0
//! (`gen_ft8_sweep_wavs.sh:37` and `gen_fst4_sweep_wavs.sh:34`
//! hardcode `DT=0.0`; `jt9sim.f90:124` hardcodes `k=12000` and takes
//! no Δt argument at all). A too-narrow time window is invisible to a
//! corpus with no time spread, by construction.
//!
//! This is the permanent guard. It synthesises a clean signal
//! in-crate, places it progressively later inside the slot, and
//! asserts the decoder still finds it out to the edge the *reference
//! decoder* reaches.
//!
//! ## Why only the late side
//!
//! Placing a frame *earlier* than nominal eventually pushes its start
//! before sample 0, where it is truncated and effectively re-anchors
//! at 0 — so an "early" miss cannot be distinguished from a window
//! limit. Placing it later is unambiguous as long as the slot has
//! tail room, which these do. The late side is also where the real
//! divergence was: WSJT-X's JT65 and JT9 windows are both deliberately
//! asymmetric toward late starts.
//!
//! ## Provenance of the edges asserted below
//!
//! Each `REFERENCE_*` constant is a *measurement*, not a reading of
//! the Fortran. Real `jt9` was run over a Δt-stepped corpus from the
//! matching reference simulator, and the constant is the last Δt it
//! decoded. Source line numbers are given for orientation only —
//! #282 showed the source alone predicts the direction wrong (FST4's
//! window looked like a gap and was a superset; Q65's was worse than
//! the source implied).
//!
//! Q65's equivalent guard lives in
//! `q65_a15_roundtrip::q65_15a_default_window_covers_wsjtx_plus_one_second`
//! — same idea, kept next to that sub-mode's other roundtrip tests.

#![cfg(all(feature = "jt65", feature = "jt9"))]

const FS: u32 = 12_000;

/// Last Δt real `jt9 -6 -p 60 -d 3` decoded on a `jt65sim -t` sweep
/// at −10 dB (measured 2026-08-13; it also decoded every step from
/// −3.0 up, and failed at +6.0). `sync65.f90:29-30` reads
/// `lag1=-32, lag2=82` in `1024/11025` s units for orientation.
///
/// This crate reached only Δt = 0.0 before #282 — its window was
/// `time_tolerance_symbols: 3` ≈ ±1.11 s.
const REFERENCE_JT65_LATE_SEC: f32 = 5.0;

/// Last Δt real `jt9 -9 -p 60 -d 3` decoded on a shifted `jt9sim`
/// sweep at −15 dB (measured 2026-08-13; failed at +0.75). Note this
/// is far short of the `lag2 = 5.0/tstep` that `jt9_decode.f90:70`
/// suggests — another reason these constants are measured rather than
/// read. This crate already covered it; the assert is a regression
/// guard, not a fix.
const REFERENCE_JT9_LATE_SEC: f32 = 0.6;

/// Place `signal` into a `slot_sec`-long silent slot starting at
/// `start_sec`, and return the slot.
fn place(signal: &[f32], start_sec: f32, slot_sec: f32) -> Vec<f32> {
    let mut slot = vec![0.0f32; (FS as f32 * slot_sec) as usize];
    let start = (start_sec * FS as f32).round() as usize;
    let n = signal.len().min(slot.len().saturating_sub(start));
    slot[start..start + n].copy_from_slice(&signal[..n]);
    slot
}

#[test]
fn jt65_window_reaches_reference_late_edge() {
    use mfsk_core::jt65::search::SearchParams;
    use mfsk_core::jt65::{decode_scan, tx::synthesize_standard};

    let signal = synthesize_standard("K1ABC", "W9XYZ", "EN37", FS, 1500.0, 0.3)
        .expect("JT65 synth must succeed");
    // Nominal JT65 frame start is 1 s into the slot.
    let nominal = 1.0f32;
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1600.0,
        ..Default::default()
    };

    let mut missed: Vec<f32> = Vec::new();
    let mut dt = 0.0f32;
    while dt <= REFERENCE_JT65_LATE_SEC + 1e-3 {
        let slot = place(&signal, nominal + dt, 60.0);
        let decodes = decode_scan(&slot, FS, (nominal * FS as f32) as usize, &params);
        let hit = decodes
            .iter()
            .any(|d| d.message.to_string().contains("W9XYZ"));
        println!("  JT65 Δt={dt:>+5.1}s  {}", if hit { "yes" } else { "NO" });
        if !hit {
            missed.push(dt);
        }
        dt += 1.0;
    }
    assert!(
        missed.is_empty(),
        "JT65 must decode out to Δt = {REFERENCE_JT65_LATE_SEC:+.1} s, the last step real \
         `jt9 -6` decoded on the same kind of input — missed {missed:?}"
    );
}

#[test]
fn jt9_window_reaches_reference_late_edge() {
    use mfsk_core::jt9::search::SearchParams;
    use mfsk_core::jt9::{decode_scan, tx::synthesize_standard};

    let signal = synthesize_standard("K1ABC", "W9XYZ", "EN37", FS, 1500.0, 0.3)
        .expect("JT9 synth must succeed");
    let nominal = 1.0f32;
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1600.0,
        ..Default::default()
    };

    let mut missed: Vec<f32> = Vec::new();
    for &dt in &[0.0f32, 0.25, REFERENCE_JT9_LATE_SEC] {
        let slot = place(&signal, nominal + dt, 60.0);
        let decodes = decode_scan(&slot, FS, (nominal * FS as f32) as usize, &params);
        let hit = decodes
            .iter()
            .any(|d| d.message.to_string().contains("W9XYZ"));
        println!("  JT9  Δt={dt:>+5.2}s  {}", if hit { "yes" } else { "NO" });
        if !hit {
            missed.push(dt);
        }
    }
    assert!(
        missed.is_empty(),
        "JT9 must decode out to Δt = {REFERENCE_JT9_LATE_SEC:+.2} s, the last step real \
         `jt9 -9` decoded on the same kind of input — missed {missed:?}"
    );
}
