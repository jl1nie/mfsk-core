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

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{ApHint, DecodeStrictness};
use mfsk_core::msg::decode_request::DecodeRequest;
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

/// Hard recall floor on `JTDX_AP_ON_EXTRAS`. Issue #40 (host
/// wide-band coarse-sync candidate gap) closed the *negative-dt*
/// half of the divergence: `process_candidate` was casting
/// `i_start = ((refined.dt_sec + 0.5) * 200.0) as usize`, saturating
/// to 0 for any candidate whose actual TX started before the slot's
/// nominal start. CQ F5RXL @ -0.78 s had `i_start = -54` collapse to
/// 0, silently misaligning the symbol grid by ~1.75 symbols and
/// dropping the decode entirely. After the fix (i32 with WSJT-X
/// all-or-nothing boundary check, matching
/// `decode_block::fill_symbol_spectra_via_cd0`), single-pass host
/// AP-off recovers 7/8 of the WSJT-X golden — same as the embedded
/// `decode_block` path — and AP-on surfaces the F5RXL CQ via
/// iaptype-1 blind-CQ (1/6 JTDX extras).
///
/// The remaining 5 JTDX extras (CQ EA2BFM, K1JT HA5WA, K1BZM DK8NE,
/// KD2UGC F6GCP, K1BZM EA3CJ) sit beneath strong neighbours on the
/// raw spectrum and JTDX rescues them via multi-pass SIC + AP — the
/// host equivalent is `decode_frame_subtract_with_ap`, which this
/// test deliberately does *not* exercise (we want the apon test to
/// gate the single-pass `decode_frame_with_ap` invariant). A
/// separate test for the multi-pass form is the obvious follow-up.
const JTDX_EXTRAS_HARD_FLOOR: usize = 1;

/// Cap on total output. AP-on adds passes 5..12; we expect a few
/// extra decodes but not a flood. Set generously so the test
/// catches catastrophic CRC-noise regressions without false-failing
/// on legitimate AP-on extras.
const MAX_TOTAL_DECODES: usize = 35;

use common::load_wav_i16;

fn decode_set(audio: &[i16], ap: Option<&ApHint>) -> BTreeSet<String> {
    let mut req = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 1.3, 50);
    if let Some(ap) = ap {
        req = req.ap_hint(ap);
    }
    req.decode()
        .results
        .into_iter()
        .filter_map(|r| unpack77(r.message77()))
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
    assert!(
        extras_hit.len() >= JTDX_EXTRAS_HARD_FLOOR,
        "JTDX AP-on coverage regressed: {}/{} below floor {}",
        extras_hit.len(),
        JTDX_AP_ON_EXTRAS.len(),
        JTDX_EXTRAS_HARD_FLOOR,
    );

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

/// Diagnostic — same JTDX extras coverage check, but using
/// `decode_frame_subtract_with_ap` (host 3-pass + SIC + AP). The 5
/// remaining JTDX extras at -13 to -19 dB sit beneath strong
/// neighbours; SIC reveals them. Embedded `decode_block_multipass`
/// has the equivalent 3-pass + SIC built in (without AP — that's
/// the deferred A0' work). This test measures what the multipass
/// host pipeline catches; `JTDX_EXTRAS_HARD_FLOOR_MULTIPASS` is the
/// floor for it.
// Multipass floor — bumped 1 → 5 in 0.6.2 after the host
// `decode_frame_subtract_with_ap` driver was rewired to use
// `subtract_signal_lpf` (matching `decode_block`'s WSJT-X-faithful
// channel-aware subtract). Cleaner residual surfaces 4 of the 5
// missing JTDX-extras at coarse-sync stage 1 of pass 1.
// Stepped back to 4 in 0.6.3, restored to 5 in the issue #72
// follow-up (2026-07-18): 0.6.3's OSD hard-error ceiling of 22
// filtered `CQ EA2BFM IN83` (one of the multipass extras,
// `hard_errors = 31`) on the OSD pass, assumed to be a CRC-luck
// phantom. A CCIR-fading sensitivity investigation found that
// assumption wrong — see `decode_block/osd_strategy.rs`'s history
// comment (now also `DecodeStrictness::ft8_nharderrors_max`'s
// docstring) for the full account — and widened the ceiling back to
// WSJT-X's 36 (today's `Normal` default), which restores this extra
// too. The other 4 extras (CQ F5RXL
// IN94, KD2UGC F6GCP, K1BZM EA3CJ, the 4 in 0.6.2) were never
// affected by that gate either way.
//
// K1BZM DK8NE -19 (deepest) still isn't caught here, but **not**
// because of AP-list breadth (the AP hint above is K1JT/HA0DU, a
// different QSO — DK8NE's own real decode is blind, no AP at all;
// confirmed directly against a locally-instrumented jt9 rebuild,
// issue #180 follow-up). The old `q >= 12` OSD gate that used to
// block this candidate outright has since been loosened to `q > 6`
// (WSJT-X-faithful, see `osd_strategy.rs`), and both the SIC
// residual's data-symbol quality and its LLR reliability ordering
// have been verified equivalent to jt9's own residual at these
// coordinates — yet `osd_decode_npre1` (the real ndeep=2 dispatch
// for this candidate's q=11) still fails to produce a codeword where
// WSJT-X's real `osd174_91.f90` ndeep=2 succeeds. That's a genuine
// OSD algorithm fidelity gap, tracked as issue #182, not something a
// wider AP list would fix.
const JTDX_EXTRAS_HARD_FLOOR_MULTIPASS: usize = 5;

#[test]
fn qso3_apon_subtract_jtdx_extras_diag() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let ap = ApHint::new().with_call1(MYCALL).with_call2(HISCALL);

    let decoded = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.3, 50)
        .strictness(DecodeStrictness::Normal)
        .sic_early()
        .ap_hint(&ap)
        .decode()
        .results;
    let messages: BTreeSet<String> = decoded
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();

    println!(
        "\nqso3 AP-on **subtract** (mycall={MYCALL}, hiscall={HISCALL}) — {} decode(s):",
        messages.len()
    );
    for m in &messages {
        println!("  {}", m);
    }

    let extras_hit: Vec<&str> = JTDX_AP_ON_EXTRAS
        .iter()
        .copied()
        .filter(|g| messages.contains(*g))
        .collect();
    let extras_missing: Vec<&str> = JTDX_AP_ON_EXTRAS
        .iter()
        .copied()
        .filter(|g| !messages.contains(*g))
        .collect();
    println!(
        "\n  JTDX AP-on extras (multipass): {}/{} hit (floor {})",
        extras_hit.len(),
        JTDX_AP_ON_EXTRAS.len(),
        JTDX_EXTRAS_HARD_FLOOR_MULTIPASS,
    );
    if !extras_missing.is_empty() {
        println!("  not yet caught: {:?}", extras_missing);
    }
    assert!(
        extras_hit.len() >= JTDX_EXTRAS_HARD_FLOOR_MULTIPASS,
        "JTDX AP-on multipass coverage regressed: {}/{} below floor {}",
        extras_hit.len(),
        JTDX_AP_ON_EXTRAS.len(),
        JTDX_EXTRAS_HARD_FLOOR_MULTIPASS,
    );
}
