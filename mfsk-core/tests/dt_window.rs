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
//! ## Both sides, since #283
//!
//! This file originally asserted the late side only: placing a frame
//! earlier than nominal eventually pushed its start before sample 0,
//! which the decoders could not represent at all, so an "early" miss
//! could not be told apart from a window limit. #283 fixed that (the
//! scan front-pads, so an early frame has a real index and a signed
//! `dt_sec`), and the early edge became assertable — so it is
//! asserted here.
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
//! Q65's *late*-edge guard lives in
//! `q65_a15_roundtrip::q65_15a_default_window_covers_wsjtx_plus_one_second`,
//! kept next to that sub-mode's other roundtrip tests; its early edge
//! is asserted here alongside JT65's.

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

/// Earliest Δt real `jt9 -6 -p 60 -d 3` decoded on the same
/// `jt65sim -t` sweep (measured 2026-08-13; it failed at −4.0).
const REFERENCE_JT65_EARLY_SEC: f32 = -3.0;

/// Earliest Δt real `jt9 -3 -p 15 -b A -d 3` decoded on a `q65sim`
/// sweep at −20 dB (measured 2026-08-12; it failed at −1.25).
/// Matches `q65.f90:127`'s `lag1 = -1.0/dtstep`.
const REFERENCE_Q65_EARLY_SEC: f32 = -1.0;

/// A frame that starts *before* the caller's buffer must still decode
/// — real `jt9` recovers it from the part that landed inside the
/// slot, and before #283 this crate could not represent the alignment
/// at all (`row_min` clamped at 0, so its low edge sat at exactly
/// `−nominal`).
///
/// Asserted through `dt_sec`, since `start_sample` saturates at 0 for
/// exactly these frames and so cannot express the case under test.
#[test]
fn jt65_decodes_frame_starting_before_the_buffer() {
    use mfsk_core::jt65::search::SearchParams;
    use mfsk_core::jt65::{decode_scan, tx::synthesize_standard};

    let signal = synthesize_standard("K1ABC", "W9XYZ", "EN37", FS, 1500.0, 0.3)
        .expect("JT65 synth must succeed");
    let nominal = 1.0f32;
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1600.0,
        ..Default::default()
    };

    // Δt = −3.0 s with a 1.0 s nominal puts the frame start 2.0 s
    // before sample 0: the leading 2 s are simply gone.
    let dt = REFERENCE_JT65_EARLY_SEC;
    let drop = (-(nominal + dt) * FS as f32).round() as usize;
    assert!(drop > 0, "test only meaningful when the frame is truncated");
    let slot = place(&signal[drop..], 0.0, 60.0);

    let decodes = decode_scan(&slot, FS, (nominal * FS as f32) as usize, &params);
    let hit = decodes
        .iter()
        .find(|d| d.message.to_string().contains("W9XYZ"));
    println!(
        "  JT65 Δt={dt:>+5.1}s (first {:.1} s of the frame cut)  {}",
        drop as f32 / FS as f32,
        hit.map(|h| format!("yes, dt_sec={:+.2}", h.dt_sec))
            .unwrap_or_else(|| "NO".into())
    );
    assert!(
        hit.is_some(),
        "JT65 must decode a frame starting {:.1} s before the buffer — real `jt9 -6` does",
        drop as f32 / FS as f32
    );
}

#[test]
fn q65_decodes_frame_starting_before_the_buffer() {
    use mfsk_core::q65::decode_request::DecodeRequest;
    use mfsk_core::q65::search::SearchParams;
    use mfsk_core::q65::{Q65a15, tx::synthesize_standard_for};

    let signal = synthesize_standard_for::<Q65a15>("CQ", "JA1ABC", "PM95", FS, 1500.0, 0.3)
        .expect("Q65-15A synth must succeed");
    // Q65-15's nominal frame start is 0.5 s into the slot.
    let nominal = 0.5f32;
    let dt = REFERENCE_Q65_EARLY_SEC;
    let drop = (-(nominal + dt) * FS as f32).round() as usize;
    assert!(drop > 0, "test only meaningful when the frame is truncated");
    let slot = place(&signal[drop..], 0.0, 15.0);

    let decodes = DecodeRequest::<Q65a15>::new(
        &slot,
        FS,
        (nominal * FS as f32) as usize,
        SearchParams::default(),
    )
    .decode();
    let hit = decodes.iter().find(|d| d.message.contains("JA1ABC"));
    println!(
        "  Q65-15A Δt={dt:>+5.1}s (first {:.1} s of the frame cut)  {}",
        drop as f32 / FS as f32,
        hit.map(|h| format!("yes, dt_sec={:+.2}", h.dt_sec))
            .unwrap_or_else(|| "NO".into())
    );
    assert!(
        hit.is_some(),
        "Q65-15A must decode a frame starting {:.1} s before the buffer — real `jt9 -3` does",
        drop as f32 / FS as f32
    );
}

/// Latest Δt real `jt9 -3 -p 60 -b A -d 3` decoded on a `q65sim`
/// sweep at −25 dB (measured 2026-08-13; it failed at +6.0).
///
/// **The reference Q65 window is asymmetric and sub-mode dependent**:
/// Q65-15A and Q65-30A were measured at −1.0…+1.0, Q65-60A at
/// −1.0…+5.5. Nothing in the suite exercised `SearchParams::default()`
/// before this test, which is how the default went out wrong twice in
/// one issue (#282: too narrow on Q65-15; then its own fix cut
/// Q65-60A's late reach from +3.0 to +1.0).
const REFERENCE_Q65_60_LATE_SEC: f32 = 5.0;

#[test]
#[cfg(feature = "q65")]
fn q65_60a_default_window_reaches_reference_late_edge() {
    use mfsk_core::q65::decode_request::DecodeRequest;
    use mfsk_core::q65::search::SearchParams;
    use mfsk_core::q65::{Q65a60, tx::synthesize_standard_for};

    let signal = synthesize_standard_for::<Q65a60>("CQ", "JA1ABC", "PM95", FS, 1500.0, 0.3)
        .expect("Q65-60A synth must succeed");
    // q65sim places TR>=60 at t = Δt + 1.0 s.
    let nominal = 1.0f32;
    let dt = REFERENCE_Q65_60_LATE_SEC;
    let slot = place(&signal, nominal + dt, 60.0);

    let decodes = DecodeRequest::<Q65a60>::new(
        &slot,
        FS,
        (nominal * FS as f32) as usize,
        SearchParams::default(),
    )
    .decode();
    let hit = decodes.iter().find(|d| d.message.contains("JA1ABC"));
    println!(
        "  Q65-60A Δt={dt:>+5.1}s under SearchParams::default()  {}",
        hit.map(|h| format!("yes, dt_sec={:+.2}", h.dt_sec))
            .unwrap_or_else(|| "NO".into())
    );
    assert!(
        hit.is_some(),
        "Q65-60A must decode at Δt = {dt:+.1} s under the *default* SearchParams — real \
         `jt9 -3 -p 60 -b A` reaches +5.5 s. A symmetric ±1.0 s default fails this."
    );
}
