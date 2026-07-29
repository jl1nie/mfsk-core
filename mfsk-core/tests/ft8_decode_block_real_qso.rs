//! Hard-assertion regression: embedded ship config (`decode_block`,
//! `DecodeDepth::BP_ONLY`, `max_cand=15`, `sync_min=1.3` — matches
//! `ft8_qso3_apoff_recall.rs`'s ship config) vs the host wide-band
//! reference (`decode_frame`, `BpAllOsd`, `max_cand=200`) on real
//! on-air recordings.
//!
//! `qso3_busy.wav` already has a WSJT-X-golden-based floor in
//! `ft8_qso3_apoff_recall.rs` / `ft8_qso3_apon_recall.rs`; this test's
//! distinct value is `qso1.wav` / `qso2.wav`, which have no WSJT-X
//! golden reference and are otherwise completely untested. `truth`
//! here is `decode_frame`'s own current output, not an external
//! golden — the assertion is "does the embedded path stay faithful to
//! the host path," not "does either match WSJT-X." qso3_busy is kept
//! in the loop for continuity with the host/embedded comparison this
//! test used to make informally (pre-2026-07-18 this file was
//! print-only diagnostics with no assertions — replaced with hard
//! floors below).
//!
//! Floors measured 2026-07-18: all three WAVs hit 100% of host truth
//! (qso1 4/4, qso2 5/5, qso3_busy 12/12) with qso3_busy carrying 2
//! phantom extras (already tolerated by the apoff/apon ceiling).

use std::collections::BTreeSet;
use std::path::Path;

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

use common::load_wav_i16;

struct Entry {
    label: &'static str,
    path: &'static str,
    min_hit: usize,
    max_extra: usize,
}

const ENTRIES: &[Entry] = &[
    Entry {
        label: "qso1",
        path: asset_path!("qso1.wav"),
        min_hit: 4,
        max_extra: 1,
    },
    Entry {
        label: "qso2",
        path: asset_path!("qso2.wav"),
        min_hit: 5,
        max_extra: 1,
    },
    Entry {
        label: "qso3_busy",
        path: asset_path!("qso3_busy.wav"),
        min_hit: 12,
        max_extra: 4,
    },
];

#[test]
fn decode_block_matches_decode_frame_on_real_qso() {
    let mut failures: Vec<String> = Vec::new();

    for e in ENTRIES {
        let slot = load_wav_i16(Path::new(e.path));

        let truth: BTreeSet<String> = DecodeRequest::<Ft8>::new(&slot, 100.0, 3000.0, 1.0, 200)
            .decode()
            .results
            .iter()
            .filter_map(|x| unpack77(x.message77()))
            .collect();
        let ship: BTreeSet<String> =
            decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::BP_ONLY, 15)
                .iter()
                .filter_map(|x| unpack77(x.message77()))
                .collect();

        let hit = ship.intersection(&truth).count();
        let extra = ship.difference(&truth).count();

        println!(
            "  {:<10} truth={:>2} ship_hit={:>2}/{:<2} (floor {})  extra={:>2} (ceiling {})",
            e.label,
            truth.len(),
            hit,
            truth.len(),
            e.min_hit,
            extra,
            e.max_extra,
        );

        if truth.len() < e.min_hit {
            failures.push(format!(
                "{}: host recall regression — only {} host-truth msgs found, expected at least {}",
                e.label,
                truth.len(),
                e.min_hit
            ));
        } else if hit < e.min_hit {
            failures.push(format!(
                "{}: embedded recall regression — {} of {} host-truth msgs, floor {}",
                e.label,
                hit,
                truth.len(),
                e.min_hit
            ));
        }
        if extra > e.max_extra {
            failures.push(format!(
                "{}: phantom regression — {} extras beyond host truth, ceiling {}",
                e.label, extra, e.max_extra
            ));
        }
    }

    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}
