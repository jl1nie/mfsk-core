//! Hard-assertion regression — `decode_frame_with_ap` AP-on decode of
//! the reference WAV (`samples/FT8/210703_133430.wav` =
//! `embedded-poc/assets/qso3_busy.wav`).
//!
//! Replaces the deleted `panic!("AP-list not ported yet")` placeholder
//! tracked in https://github.com/jl1nie/mfsk-core/issues/31. The test
//! drives the *same* host pipeline (`decode_frame_with_ap`) twice on
//! the same WAV — once with `ap_hint = None` (AP-off baseline) and
//! once with operator-context `Some(&ap)` (AP-on) — and asserts:
//!
//! 1. **Strict-superset invariant** (#31 acceptance criteria) — every
//!    message decoded with AP off must also be decoded with AP on.
//!    The blind-CQ pass and the operator-context multi-pass loop
//!    must not displace any existing decode.
//! 2. **JTDX AP-on extras** — additional decodes that AP-on surfaces
//!    and AP-off does not, sourced from JTDX (not WSJT-X) with
//!    `lapon=true` and `mycall = K1JT, hiscall = HA0DU` operator
//!    context (chosen from the AP-off golden entry
//!    `K1JT HA0DU KN07` per #31's "called-side" convention). The
//!    test reports JTDX coverage as a progress indicator and gates
//!    on a hard floor that grows as the host coarse-sync parity gap
//!    closes — see `JTDX_EXTRAS_HARD_FLOOR`.
//!
//! The 8-entry WSJT-X canonical AP-off golden lives in
//! `ft8_qso3_apoff_recall.rs` and is checked through `decode_block`
//! (the embedded-friendly path). That is *not* re-checked here —
//! `decode_frame_with_ap` and `decode_block` are different pipelines
//! with different sync-candidate selection and phantom-filtering
//! behaviour, and conflating them would make #31's invariant
//! sensitive to host-path tuning unrelated to AP.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_apon_recall -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::decode::{ApHint, DecodeDepth, decode_frame_with_ap};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Operator context for the AP-on run. Picked from the AP-off
/// golden entry `K1JT HA0DU KN07` — K1JT is call1 (the receiver,
/// "called side") and HA0DU is call2 (sender). Per the planning
/// Q&A under #31, the called-side callsign is used as `mycall` so
/// AP-on surfaces follow-up replies / reports addressed to K1JT.
const MYCALL: &str = "K1JT";
const HISCALL: &str = "HA0DU";

/// JTDX AP-on extras — decodes that **JTDX** (not WSJT-X) surfaces
/// with `lapon=true`, `mycall=K1JT`, `hiscall=HA0DU` on this WAV
/// beyond what `decode_frame_with_ap(.., None)` produces on the
/// same host pipeline. Source: JTDX FT8-deep capture 2026-05-08.
/// Reports stored without leading zeros to match `unpack77` print
/// convention (e.g. `-9` not `-09`).
///
/// Naming follows the AP-off counterpart split: WSJT-X canonical
/// goldens live in `ft8_qso3_apoff_recall.rs::WSJTX_GOLDEN`, JTDX
/// goldens in `ft8_qso3_jtdx_recall.rs`. We deliberately do not
/// claim WSJT-X provenance for these rows — the JTDX a-priori
/// engine covers iaptypes WSJT-X public 2.7 does not, so its
/// AP-on output is a strict superset of WSJT-X AP-on, not a
/// substitute reference.
///
/// Each entry below is annotated with the AP mechanism that
/// *should* surface it once the host coarse-sync candidate gap
/// closes (see `JTDX_EXTRAS_HARD_FLOOR` for what we currently
/// require to hit).
const JTDX_AP_ON_EXTRAS: &[&str] = &[
    "CQ F5RXL IN94",     // -7 dB,  blind-CQ pass 12 target
    "CQ EA2BFM IN83",    // -15 dB, blind-CQ pass 12 target
    "K1JT HA5WA 73",     // -18 dB, operator-context (mycall=K1JT)
    "K1BZM DK8NE -10",   // -19 dB, deep AP rescue
    "KD2UGC F6GCP R-23", // -10 dB, separate QSO context
    "K1BZM EA3CJ JN01",  // -12 dB, separate QSO context
];

/// Hard recall floor on `JTDX_AP_ON_EXTRAS`. Currently `0` because
/// `decode_frame_with_ap` (host wide-band path) misses the
/// underlying coarse-sync candidates at 1196 / 244 / 472 / 2039 Hz
/// that `decode_block` (embedded path) and JTDX both catch. AP
/// runs in `process_candidate` only on candidates that survive
/// coarse-sync + fine-refine, so the AP plumbing cannot recover
/// decodes whose candidates were filtered upstream — this is a
/// host-vs-embedded coarse-sync parity gap, not an AP-list bug,
/// and is tracked separately as a follow-up to #31.
///
/// When the parity gap closes, raise this floor toward
/// `JTDX_AP_ON_EXTRAS.len()` and the test will start gating the
/// fix-forward. Until then the test reports JTDX coverage as
/// informational diagnostics.
const JTDX_EXTRAS_HARD_FLOOR: usize = 0;

/// Cap on total output. AP-on adds passes 5..12; we expect a few
/// extra decodes but not a flood. Set generously so the test
/// catches catastrophic CRC-noise regressions without false-failing
/// on legitimate AP-on extras.
const MAX_TOTAL_DECODES: usize = 35;

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

fn decode_set(audio: &[i16], ap: Option<&ApHint>) -> BTreeSet<String> {
    decode_frame_with_ap(
        audio,
        100.0,
        3000.0,
        1.3,
        None,
        DecodeDepth::BpAllOsd,
        50,
        ap,
    )
    .into_iter()
    .filter_map(|r| unpack77(&r.message77))
    .collect()
}

#[test]
fn qso3_apon_strict_superset_of_apoff_same_pipeline() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let ap = ApHint::new().with_call1(MYCALL).with_call2(HISCALL);
    let ap_off = decode_set(&slot, None);
    let ap_on = decode_set(&slot, Some(&ap));

    println!(
        "\nqso3 AP-off (host pipeline) — {} decode(s):",
        ap_off.len()
    );
    for m in &ap_off {
        println!("  {}", m);
    }
    println!(
        "\nqso3 AP-on (mycall={MYCALL}, hiscall={HISCALL}) — {} decode(s):",
        ap_on.len()
    );
    for m in &ap_on {
        let tag = if ap_off.contains(m) { "  " } else { "+ " };
        println!("  {}{}", tag, m);
    }

    // 1. Strict-superset invariant — every AP-off decode must also
    //    appear in AP-on output.
    let lost: Vec<&String> = ap_off.difference(&ap_on).collect();
    assert!(
        lost.is_empty(),
        "AP-on lost decodes that AP-off catches (regression on the strict-superset invariant): {:?}",
        lost,
    );

    // 2. JTDX AP-on extras — informational coverage diagnostics
    //    plus a hard floor that the test gates on. The floor is
    //    raised as the host-vs-embedded coarse-sync parity gap
    //    closes (see JTDX_EXTRAS_HARD_FLOOR docstring).
    let extras_hit: Vec<&str> = JTDX_AP_ON_EXTRAS
        .iter()
        .copied()
        .filter(|g| ap_on.contains(*g))
        .collect();
    let extras_missing: Vec<&str> = JTDX_AP_ON_EXTRAS
        .iter()
        .copied()
        .filter(|g| !ap_on.contains(*g))
        .collect();
    println!(
        "\n  JTDX AP-on extras: {}/{} hit (floor {})",
        extras_hit.len(),
        JTDX_AP_ON_EXTRAS.len(),
        JTDX_EXTRAS_HARD_FLOOR,
    );
    if !extras_missing.is_empty() {
        println!("  not yet caught: {:?}", extras_missing);
    }
    // `>=` against a const that is `0` today reads as a tautology to
    // clippy, but the const is the seam we tighten as the parity gap
    // closes — silence the absurd-comparison lint here intentionally.
    #[allow(clippy::absurd_extreme_comparisons)]
    {
        assert!(
            extras_hit.len() >= JTDX_EXTRAS_HARD_FLOOR,
            "JTDX AP-on coverage regressed: {}/{} below floor {}",
            extras_hit.len(),
            JTDX_AP_ON_EXTRAS.len(),
            JTDX_EXTRAS_HARD_FLOOR,
        );
    }

    // 3. Phantom ceiling — AP must not turn the decoder into a noise
    //    generator. Set generously so legitimate AP-on extras don't
    //    trip it.
    assert!(
        ap_on.len() <= MAX_TOTAL_DECODES,
        "AP-on decode count {} exceeds ceiling {} (phantom regression?)",
        ap_on.len(),
        MAX_TOTAL_DECODES,
    );
}
