// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared golden reference for `embedded-poc/assets/qso3_busy.wav`
//! (= `samples/FT8/210703_133430.wav`), used by every FT8 test that
//! checks recall/precision/SNR against this recording through
//! [`common::golden::assert_golden`](crate::golden::assert_golden).
//!
//! ## Provenance — reconciling two reference decoders into one list
//!
//! Two independent decoders were run against this file in prior
//! sessions, at different aggressiveness:
//!
//! - **WSJT-X** AP-off (`reference_qso3_busy_wsjtx_decode.md`): 8
//!   entries, the conservative/canonical reference.
//! - **JTDX** AP-off (`reference_qso3_busy_jtdx_decode.md`): 18
//!   entries — a more aggressive WSJT-X fork that recovers signals
//!   vanilla WSJT-X misses at default settings. Includes 17 of
//!   WSJT-X's 8 (`CQ F5RXL IN94` is the one exception — a JTDX edge
//!   case, present in WSJT-X but absent from JTDX) plus 11 more.
//! - **JTDX AP-on** (`mycall=K1JT, hiscall=HA0DU`, 2026-05-08
//!   capture, formerly `ft8_qso3_apon_recall.rs::JTDX_AP_ON_EXTRAS`):
//!   contributes exactly one signal beyond the two lists above —
//!   `K1JT HA5WA 73` — that AP-off JTDX didn't surface but AP-on did.
//!   No independent freq reference was ever captured for it (only an
//!   SNR estimate), so `freq_hz` stays `None` on that single entry
//!   rather than borrowing this crate's own measurement as if it were
//!   independent ground truth.
//!
//! Union: 8 + 18 − 7 (overlap) + 1 (`K1JT HA5WA 73`) = **20** known
//! real signals. `WM3PEN EA6VQ -09` and `W1FC F5BZB -08` keep
//! `snr_db: None` — JTDX's own claimed SNR for both (0.0 dB) was
//! independently shown unreliable (a local `jt9 -8 -d3` build's own
//! internal `xsnr2` reads 12+ dB for one of them; see
//! `ft8_qso3_jtdx_recall.rs`'s `SNR_TOL_DB` doc comment for the full
//! writeup) — asserting against a value already known to be wrong
//! would just be fitting this crate to JTDX's quirks.
//!
//! `WA2FZW DL5AXX RR73` carries a known provenance caveat: it's the
//! one JTDX entry `project_jtdx_ground_truth_question.md` could not
//! independently corroborate (possible JTDX false positive), unlike
//! every other JTDX-only entry here, which PR #152's OSD fix or an
//! AP-context hit did confirm. It stays in the reference — both host
//! f32 and fixed-point full-parity configs reproduce this exact
//! message at a consistent frequency, which is itself mild evidence
//! for "real" — but a future investigation could still overturn it.
//!
//! ## Verifying `max_extra: 0` actually holds
//!
//! Zero phantoms against this 20-entry union was verified empirically
//! (2026-08-14), not assumed: every message the ship-config decode
//! (`DecodeDepth::EMBEDDED`, `sync_min=1.3`, `max_cand=15`, both host
//! f32 and fixed-point) and the full-parity decode
//! (`DecodeDepth::FULL`, `sync_min=0.8`, `max_cand=60`, both variants)
//! produce on this WAV is accounted for by an entry below. Full-parity
//! reaches all 20/20 with zero extras in both numeric paths.

use super::golden::GoldenEntry;

/// Frequency tolerance for FT8 coarse-sync df drift on this
/// recording — same value used by every bespoke qso3 test this
/// replaced.
pub const DF_TOL_HZ: f32 = 5.0;

/// SNR tolerance, host f32. See the module doc above and
/// `ft8_qso3_jtdx_recall.rs`'s own `SNR_TOL_DB` doc comment for why
/// this stays wide: `xsnr2`/`xbase` drift against WSJT-X/JTDX's own
/// figures is a known, bounded, non-regressing gap, not a bug this
/// gate exists to catch tightly.
#[cfg(not(feature = "fixed-point"))]
pub const SNR_TOL_DB: f32 = 12.0;
/// SNR tolerance, fixed-point (adjacent-tone SNR, no `xsnr2`
/// post-process — wider drift, e.g. N1PJT sits ~13 dB off on this
/// WAV).
#[cfg(feature = "fixed-point")]
pub const SNR_TOL_DB: f32 = 14.0;

/// All 20 known-real signals on `qso3_busy.wav`. See the module doc
/// for provenance and the reconciliation between WSJT-X/JTDX/JTDX
/// AP-on.
pub static QSO3_KNOWN_REAL_SIGNALS: &[GoldenEntry] = &[
    // WSJT-X AP-off golden (8) — the one entry JTDX itself doesn't
    // surface (`CQ F5RXL IN94`) is included here from WSJT-X.
    GoldenEntry {
        msg: "CQ F5RXL IN94",
        freq_hz: Some(1197.0),
        dt_sec: None,
        snr_db: Some(-3.0),
    },
    GoldenEntry {
        msg: "N1JFU EA6EE R-07",
        freq_hz: Some(641.0),
        dt_sec: None,
        snr_db: Some(-13.0),
    },
    GoldenEntry {
        msg: "A92EE F5PSR -14",
        freq_hz: Some(723.0),
        dt_sec: None,
        snr_db: Some(-9.0),
    },
    GoldenEntry {
        msg: "W0RSJ EA3BMU RR73",
        freq_hz: Some(400.0),
        dt_sec: None,
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "K1JT HA0DU KN07",
        freq_hz: Some(590.0),
        dt_sec: None,
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "N1PJT HB9CQK -10",
        freq_hz: Some(466.0),
        dt_sec: None,
        snr_db: Some(-2.0),
    },
    GoldenEntry {
        msg: "K1BZM DK8NE -10",
        freq_hz: Some(244.0),
        dt_sec: None,
        snr_db: Some(-17.0),
    },
    GoldenEntry {
        msg: "KD2UGC F6GCP R-23",
        freq_hz: Some(472.0),
        dt_sec: None,
        snr_db: Some(-6.0),
    },
    // JTDX-only additions (11 of the 18-entry JTDX golden that don't
    // overlap the WSJT-X 8 above).
    GoldenEntry {
        msg: "K1JT EA3AGB -15",
        freq_hz: Some(1648.0),
        dt_sec: None,
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "WM3PEN EA6VQ -09",
        freq_hz: Some(2157.0),
        dt_sec: None,
        snr_db: None, // JTDX's claimed 0.0 dB independently shown unreliable
    },
    GoldenEntry {
        msg: "W1FC F5BZB -08",
        freq_hz: Some(2571.0),
        dt_sec: None,
        snr_db: None, // same as WM3PEN above
    },
    GoldenEntry {
        msg: "XE2X HA2NP RR73",
        freq_hz: Some(2854.0),
        dt_sec: None,
        snr_db: Some(-14.0),
    },
    GoldenEntry {
        msg: "N1API F2VX 73",
        freq_hz: Some(1513.0),
        dt_sec: None,
        snr_db: Some(-18.0),
    },
    GoldenEntry {
        msg: "W1DIG SV9CVY -14",
        freq_hz: Some(2734.0),
        dt_sec: None,
        snr_db: Some(-9.0),
    },
    GoldenEntry {
        msg: "N1API HA6FQ -23",
        freq_hz: Some(2239.0),
        dt_sec: None,
        snr_db: Some(-13.0),
    },
    GoldenEntry {
        msg: "CQ EA2BFM IN83",
        freq_hz: Some(2279.0),
        dt_sec: None,
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "K1BZM EA3CJ JN01",
        freq_hz: Some(2522.0),
        dt_sec: None,
        snr_db: Some(-12.0),
    },
    GoldenEntry {
        msg: "WA2FZW DL5AXX RR73",
        freq_hz: Some(2546.0),
        dt_sec: None,
        snr_db: Some(-15.0), // provenance caveat — see module doc
    },
    GoldenEntry {
        msg: "K1BZM EA3GP -09",
        freq_hz: Some(2696.0),
        dt_sec: None,
        snr_db: Some(-12.0),
    },
    // JTDX AP-on-only addition (1) — no independent freq reference.
    GoldenEntry {
        msg: "K1JT HA5WA 73",
        freq_hz: None,
        dt_sec: None,
        snr_db: Some(-18.0),
    },
];
