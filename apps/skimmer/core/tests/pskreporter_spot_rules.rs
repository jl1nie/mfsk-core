// SPDX-License-Identifier: GPL-3.0-only
//! What is a PSK Reporter spot, against WSJT-X v3.3.0-beta1's own code (tier B).
//!
//! `embedded-poc/assets/golden/pskreporter/` holds what `scripts/pskreporter/spot_oracle.cpp`
//! answers: for single words, `Radio::is_standard_callsign` and `decoded_grid_pattern` (the two
//! JTTY's spot rule is made of); for messages, `DecodedText::deCallAndGrid` over upstream's
//! `tokens_re`, and `pskPost`'s "a locator or a CQ", on messages the real Fortran `stdmsg_` calls
//! standard. Development also ran 143 real decodes off the air through it: all agreed.

use skimmer_core::pskreporter::{de_call_and_grid, is_decoded_grid, is_standard_callsign};

const DIR: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../../embedded-poc/assets/golden/pskreporter/"
);

fn rows(name: &str) -> Vec<Vec<String>> {
    std::fs::read_to_string(format!("{DIR}{name}"))
        .unwrap_or_else(|e| panic!("{name}: {e}"))
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|l| l.split('\t').map(str::to_string).collect())
        .collect()
}

#[test]
fn single_words_are_upstreams_callsigns_and_grids() {
    let rows = rows("tokens_beta1.tsv");
    assert!(rows.len() > 1000);
    let mut bad = Vec::new();
    for r in &rows {
        let (std_call, grid) = (r[1] == "1", r[2] == "1");
        if is_standard_callsign(&r[0]) != std_call || is_decoded_grid(&r[0]) != grid {
            bad.push(format!(
                "{:?}: upstream call={std_call} grid={grid}, ours call={} grid={}",
                r[0],
                is_standard_callsign(&r[0]),
                is_decoded_grid(&r[0])
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "{} differ, e.g. {:?}",
        bad.len(),
        &bad[..bad.len().min(8)]
    );
}

#[test]
fn messages_give_upstreams_sender_grid_and_verdict() {
    let rows = rows("messages_beta1.tsv");
    assert!(rows.len() >= 60);
    let mut bad = Vec::new();
    for r in &rows {
        let (call, grid) = de_call_and_grid(&r[0]);
        let cq = r[0].starts_with("CQ ") || r[0].contains(" CQ ");
        let post = !call.is_empty() && (is_decoded_grid(&grid) || cq);
        let want_post = r[3] == "1";
        if call != r[1]
            || grid != r[2]
            || (want_post && !post)
            || (!want_post && post && !call.is_empty())
        {
            bad.push(format!(
                "{:?}: upstream {:?}/{:?} post={want_post}, ours {call:?}/{grid:?} post={post}",
                r[0], r[1], r[2]
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "{} differ, e.g. {:?}",
        bad.len(),
        &bad[..bad.len().min(6)]
    );
}
