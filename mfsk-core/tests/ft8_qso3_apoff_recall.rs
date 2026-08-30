//! Hard-assertion regression — **ship config** decode of the
//! reference WAV (`samples/FT8/210703_133430.wav` =
//! `embedded-poc/assets/qso3_busy.wav`), checked through
//! `common::golden::assert_golden` against the 20-entry known-real-
//! signal union in `common::ft8_qso3` (see that module's doc comment
//! for how the WSJT-X-8 / JTDX-18 / JTDX-AP-on-extras references were
//! reconciled into one list).
//!
//! `max_extra: 0` here is not a loose ceiling like the old
//! `MAX_TOTAL_DECODES` — it was verified empirically (2026-08-14)
//! that ship config emits nothing outside the 20-entry union on this
//! recording, in both host f32 and fixed-point. A companion AP-on
//! regression lives in `ft8_qso3_apon_recall.rs` — see that file's
//! own doc comment for why it's a different invariant (strict-superset
//! across two runs, not a golden-vs-decoder match) that doesn't route
//! through `assert_golden`.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_qso3_apoff_recall -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::ft8_qso3::{DF_TOL_HZ, QSO3_KNOWN_REAL_SIGNALS, SNR_TOL_DB};
use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Ship-config recall floor against the 20-entry known-real-signal
/// union. Ship config (`DecodeDepth::EMBEDDED`) never runs OSD, so the
/// weakest signals (e.g. `K1BZM DK8NE -10` at -17..-19 dB) are
/// structurally out of reach; see `ft8_qso3_full_parity_recall.rs` for
/// the OSD-enabled host config that reaches all 20.
///
/// **The floor keys off `nstep-half`, not `fixed-point`** (corrected
/// 2026-08-30). This said "host f32 14/20, host fixed-point 12/20" and
/// gated on `fixed-point`, which measured the two together: that
/// feature *implies* `nstep-half`, and nothing had ever built one
/// without the other. Separated with `ft8_qso3_decode_set`:
///
/// | build | decodes |
/// |---|---:|
/// | `full` (f32, NSTEP = NSPS/4) | 14 |
/// | `full,nstep-half` (f32, NSTEP = NSPS/2) | **12** |
/// | `full,fixed-point` (implies `nstep-half`) | **12** |
///
/// The last two decode the **same twelve messages**, not merely the
/// same number of them. So the whole 14 → 12 gap is the coarse time
/// grid, and Q11i16 quantisation costs nothing here — which also means
/// a build with `nstep-half` and no `fixed-point` used to fail this
/// test for holding it to a floor it cannot reach.
///
/// The two the halved grid gives up are `K1JT EA3AGB -15` and
/// `W0RSJ EA3BMU RR73`.
#[cfg(not(feature = "nstep-half"))]
const MIN_GOLDEN_HITS: usize = 14;
#[cfg(feature = "nstep-half")]
const MIN_GOLDEN_HITS: usize = 12;

#[test]
fn qso3_apoff_meets_wsjtx_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    // Ship config: BP only, max_cand=15. Matches the Core2 / S3
    // production path (PASS1 default = 30 via DEFAULT_Q_THRESH).
    // sync_min=1.3 = WSJT-X ft8_decode.f90:176 default for ndepth=3
    // ("Deep" mode).
    let decoded: Vec<_> = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);

    assert_golden(
        &decoded,
        &GoldenSet {
            name: "FT8 qso3_busy.wav (ship config)",
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
