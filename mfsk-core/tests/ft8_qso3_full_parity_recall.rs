//! Hard-assertion regression — **host research config** decode of the
//! reference WAV (`samples/FT8/210703_133430.wav` =
//! `embedded-poc/assets/qso3_busy.wav`) under `DecodeDepth::FULL`
//! (`sync_min=0.8`, `max_cand=60` — the same default already used by
//! `ft8_qso3_jtdx_recall.rs` for the larger JTDX-18 comparison).
//!
//! Distinct from `ft8_qso3_apoff_recall.rs`'s **ship config**
//! (`DecodeDepth::EMBEDDED`, `sync_min=1.3`, `max_cand=15` — the real
//! Core2/S3 production path, which never runs OSD). This test tracks
//! the separate question "how far can host OSD push recall" — the
//! host-side ceiling, not what ships on hardware. `DecodeDepth::FULL`
//! is host-only by construction (see its own doc comment) and never
//! runs on an ESP32 target.
//!
//! Checked through `common::golden::assert_golden` against the
//! 20-entry known-real-signal union in `common::ft8_qso3` — see that
//! module's doc comment for provenance. Full parity means exactly
//! that: this config reaches **all 20/20** with zero extras, in both
//! host f32 and fixed-point (verified 2026-08-14 — `DecodeDepth::FULL`'s
//! OSD dispatch operates on `&[f32]` regardless of the embedded `LlrT`
//! choice, so the numeric path doesn't leave anything on the table
//! here; that isn't the embedded *ship* config though — real Xtensa
//! silicon never runs OSD, see `DecodeDepth::osd`'s doc comment).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_full_parity_recall -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::ft8_qso3::{DF_TOL_HZ, QSO3_KNOWN_REAL_SIGNALS, SNR_TOL_DB};
use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Full parity: all 20 known-real signals, including `K1BZM DK8NE -10`
/// which ship config's `DecodeDepth::EMBEDDED` structurally cannot
/// reach (no OSD). Measured 2026-08-14: 20/20, zero extras, in both
/// host f32 (~140ms) and fixed-point (`LlrT = Q11i16` on host, the
/// same numeric path embedded ships, ~160ms) on an AMD Ryzen 9 9900X.
/// For comparison, real `jt9 -8 -d3`'s own measured total file decode
/// time is ~1.1s (`FT8_BENCHMARK.md`) — full parity here runs ~7-8x
/// faster than jt9 itself.
const MIN_GOLDEN_HITS: usize = 20;

#[test]
fn qso3_full_parity_meets_wsjtx_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // Host research config (same default as ft8_qso3_jtdx_recall.rs):
    // DecodeDepth::FULL (OSD on — host-only by construction, never
    // runs on embedded), sync_min=0.8, max_cand=60.
    let t0 = Instant::now();
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 0.8, DecodeDepth::FULL, 60);
    let elapsed = t0.elapsed();
    println!(
        "qso3 host full-parity config (FULL/0.8/60): {:.1} ms",
        elapsed.as_secs_f64() * 1000.0
    );

    assert_golden(
        &decoded,
        &GoldenSet {
            name: "FT8 qso3_busy.wav (host full-parity config)",
            expected: QSO3_KNOWN_REAL_SIGNALS,
            min_hits: MIN_GOLDEN_HITS,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: DF_TOL_HZ,
            // Every QSO3_KNOWN_REAL_SIGNALS entry has dt_sec: None (no
            // independent dt reference was ever captured for this WAV),
            // so this value is inert — kept at the crate default for
            // when one eventually is.
            dt_sec: Tolerances::default().dt_sec,
            snr_db: SNR_TOL_DB,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}
