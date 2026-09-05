//! Hard-assertion regression: embedded ship config (`decode_block`,
//! `DecodeDepth::EMBEDDED`, `max_cand=15`, `sync_min=1.3` — matches
//! `ft8_qso3_apoff_recall.rs`'s ship config) vs a **frozen** host
//! wide-band snapshot (`decode_frame`, `BpAllOsd`, `max_cand=200`) on
//! real on-air recordings.
//!
//! `qso3_busy.wav` already has a WSJT-X-golden-based floor in
//! `ft8_qso3_apoff_recall.rs` / `ft8_qso3_apon_recall.rs`; this test's
//! distinct value is `qso1.wav` / `qso2.wav`, which have no WSJT-X
//! golden reference and are otherwise completely untested. `truth`
//! is not an external golden — the assertion is "does the embedded
//! path stay faithful to the host path," not "does either match
//! WSJT-X."
//!
//! **Like `ft8_qso3_apoff_recall.rs`, this measures the 3-pass
//! subtraction driver, not the one any embedded build ships**
//! (`decode_block_multipass` has two `#[cfg]`-gated bodies, split on
//! `fft-rustfft`, and `mod common` below pulls that flag in
//! transitively whether or not it's requested — see issue #359). For
//! the ship-faithful `not(fft-rustfft)` driver's own recall, see
//! `ft8_embedded_driver_recall.rs` (currently qso3_busy only).
//!
//! ## Why `truth` is a frozen list, not a live `decode_frame` call
//!
//! Before 2026-09-06 `truth` was `decode_frame`'s own current output,
//! recomputed every run, with the floors set once (2026-07-18) as
//! absolute counts against whatever `truth` was that day. `truth` grew
//! as the reference decoder improved — 4→6 (qso1), 12→15 (qso3_busy)
//! — while the floors stayed put, so `full` sat exactly on the floor
//! with zero margin and the actual embedded-vs-host gap was invisible
//! (issue #359). A ratio floor was considered and rejected: the ship
//! config is deliberately lean (`DecodeDepth::EMBEDDED`, no OSD, no
//! SIC, no AP — see `m5stack-cores3-app/src/decode_pipeline.rs`), so a
//! host improvement it structurally cannot follow would fail a ratio
//! floor for a reason it can never act on.
//!
//! So `truth` here is a **frozen snapshot** — the message texts
//! `decode_frame` (`BpAllOsd`, `max_cand=200`, `full` features)
//! actually returned on 2026-09-06, checked to be identical under
//! `fixed-point` too on these three WAVs. A future host improvement
//! adds nothing to this list until someone deliberately refreshes it;
//! the assertion stays about the embedded path, not about how good the
//! host got since.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full,ft8 \
//!     --test ft8_decode_block_real_qso -- --nocapture
//! cargo test --release -p mfsk-core --features full,fixed-point,ft8 \
//!     --test ft8_decode_block_real_qso -- --nocapture
//! ```

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

use common::load_wav_i16;

struct Entry {
    label: &'static str,
    path: &'static str,
    /// Frozen 2026-09-06 snapshot of `decode_frame`'s output
    /// (`BpAllOsd`, `max_cand=200`, `full`) — identical under `full`
    /// and `full,fixed-point` on these three WAVs (checked directly).
    truth: &'static [&'static str],
    #[cfg(not(feature = "nstep-half"))]
    min_hit: usize,
    #[cfg(feature = "nstep-half")]
    min_hit: usize,
    max_extra: usize,
}

const ENTRIES: &[Entry] = &[
    Entry {
        label: "qso1",
        path: asset_path!("qso1.wav"),
        truth: &[
            "7J0DNY/R PZ9BNR BM87",
            "CQ DX R6WA LN32",
            "CQ R7IW LN35",
            "CQ TA6CQ KN70",
            "OH3NIV ZS6S -03",
            "TK4LS YC1MRF 73",
        ],
        #[cfg(not(feature = "nstep-half"))]
        min_hit: 4,
        #[cfg(feature = "nstep-half")]
        min_hit: 3,
        max_extra: 0,
    },
    Entry {
        label: "qso2",
        path: asset_path!("qso2.wav"),
        truth: &[
            "CQ DX R6WA LN32",
            "CQ LZ1JZ KN22",
            "CQ R7IW LN35",
            "CQ TA6CQ KN70",
            "OH3NIV ZS6S RR73",
        ],
        #[cfg(not(feature = "nstep-half"))]
        min_hit: 5,
        #[cfg(feature = "nstep-half")]
        min_hit: 5,
        max_extra: 0,
    },
    Entry {
        label: "qso3_busy",
        path: asset_path!("qso3_busy.wav"),
        truth: &[
            "A92EE F5PSR -14",
            "CQ F5RXL IN94",
            "K1BZM DK8NE -10",
            "K1BZM EA3GP -09",
            "K1JT EA3AGB -15",
            "K1JT HA0DU KN07",
            "N1API F2VX 73",
            "N1API HA6FQ -23",
            "N1JFU EA6EE R-07",
            "N1PJT HB9CQK -10",
            "W0RSJ EA3BMU RR73",
            "W1DIG SV9CVY -14",
            "W1FC F5BZB -08",
            "WM3PEN EA6VQ -09",
            "XE2X HA2NP RR73",
        ],
        // The nstep-half grid drops exactly `K1JT EA3AGB -15` and
        // `W0RSJ EA3BMU RR73` here too — same pair `ft8_qso3_apoff_
        // recall.rs`'s own doc comment names against the 20-entry
        // golden, confirming this is the coarse time grid, not noise.
        #[cfg(not(feature = "nstep-half"))]
        min_hit: 12,
        #[cfg(feature = "nstep-half")]
        min_hit: 10,
        // `K1BZM EA3CJ JN01` and `KD2UGC F6GCP R-23` — same two extras
        // under both grids.
        max_extra: 2,
    },
];

#[test]
fn decode_block_matches_decode_frame_on_real_qso() {
    let mut failures: Vec<String> = Vec::new();

    for e in ENTRIES {
        let slot = load_wav_i16(Path::new(e.path));

        let ship: Vec<String> = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15)
            .iter()
            .filter_map(|x| unpack77(x.message77()))
            .collect();

        let hit = e
            .truth
            .iter()
            .filter(|m| ship.iter().any(|s| s.as_str() == **m))
            .count();
        let extra = ship
            .iter()
            .filter(|s| !e.truth.contains(&s.as_str()))
            .count();

        println!(
            "  {:<10} truth={:>2} ship_hit={:>2}/{:<2} (floor {})  extra={:>2} (ceiling {})",
            e.label,
            e.truth.len(),
            hit,
            e.truth.len(),
            e.min_hit,
            extra,
            e.max_extra,
        );

        if hit < e.min_hit {
            failures.push(format!(
                "{}: embedded recall regression — {} of {} frozen-truth msgs, floor {}",
                e.label,
                hit,
                e.truth.len(),
                e.min_hit
            ));
        }
        if extra > e.max_extra {
            failures.push(format!(
                "{}: phantom regression — {} extras beyond frozen truth, ceiling {}",
                e.label, extra, e.max_extra
            ));
        }
    }

    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}
