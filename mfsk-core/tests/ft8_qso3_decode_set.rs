// SPDX-License-Identifier: GPL-3.0-or-later
//! What the FT8 ship config actually decodes, as a *set*.
//!
//! `ft8_qso3_apoff_recall`'s floor is 14 without `fixed-point` and 12
//! with it, and its doc comment attributes that gap to fixed-point.
//! `fixed-point` implies `nstep-half`, so that measurement never
//! separated two different things: halving the coarse time grid, and
//! quantising the pipeline. This prints the set so the three builds can
//! be diffed rather than compared by count — two builds reaching 12
//! each is not evidence they reach the *same* 12.
//!
//! ```sh
//! for f in full full,nstep-half full,fixed-point; do
//!   cargo test -p mfsk-core --features "$f,internal-testing" --release \
//!     --test ft8_qso3_decode_set -- --ignored --nocapture
//! done
//! ```

#![cfg(all(feature = "ft8", feature = "internal-testing"))]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

#[test]
#[ignore = "diagnostic — prints the decode set for cross-feature diffing"]
fn ft8_qso3_decode_set() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    // Identical call to `ft8_qso3_apoff_recall`'s, so the set this
    // prints is the set that test scores.
    let decoded = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);
    let mut msgs: Vec<String> = decoded
        .iter()
        .filter_map(|d| unpack77(d.message77()))
        .collect();
    msgs.sort();
    msgs.dedup();
    eprintln!(
        "SETDUMP n={} nstep_half={} fixed_point={}",
        msgs.len(),
        cfg!(feature = "nstep-half"),
        cfg!(feature = "fixed-point"),
    );
    for m in &msgs {
        eprintln!("SETDUMP| {m}");
    }
}
