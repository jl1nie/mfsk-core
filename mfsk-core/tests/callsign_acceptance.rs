// SPDX-License-Identifier: GPL-3.0-only
//! The callsign grammar of the plausibility verdict (`msg::wsjt77::is_plausible_call`), against real callsigns.
//!
//! The verdict is a **deliberate divergence from WSJT-X** (`FrameDecodable::MESSAGE_FILTER_DEFAULT`): WSJT-X accepts whatever
//! `unpack77` renders, this crate also asks that the callsigns of a message be grammatical, so that a CRC false positive is not
//! reported and not subtracted. The price of a grammar is the real callsign it does not know: `DM100ZM` and `LZ365BM`
//! (special-event stations; 28 real decodes in `ft8_lib/test/wav/20m_busy`, which `jt9` reports and the verdict refused).
//!
//! This test measures the grammar against a list of real callsigns: **JTDX's `ALLCALL7.TXT`**, 100 302 callsigns spotted on
//! PSK Reporter in FT4/FT8 modes between 2024-06-20 and 2025-06-19 (its header: "being used for filtering out some false
//! FT8/FT4 decodes"). The list is not vendored (746 KB of someone else's data); point `JTDX_ALLCALL7` at it, or the test skips:
//!
//! ```sh
//! JTDX_ALLCALL7=/path/to/ALLCALL7.TXT cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test callsign_acceptance -- --nocapture
//! ```
//!
//! Measured 2026-11-26: the grammar refused **9 of 100 303** (0.009 %); they are typos and garbage (`2EOJKH`, `ZZZZ`, `PSE12M`
//! ...). A grammar that refuses a real station more often than that has to be loosened, and what it costs in phantoms is
//! `wsjt77::tests::phantom_survival_is_bounded`'s to say.

use mfsk_core::msg::wsjt77::is_plausible_call;

/// The entries of a JTDX/WSJT-X callsign list: `//` lines are comments; an entry is a token ended by a comma or a blank.
fn entries(text: &str) -> Vec<String> {
    text.lines()
        .filter(|l| !l.starts_with("//") && !l.trim().is_empty())
        .flat_map(|l| {
            l.split([',', ' ', '\t'])
                .map(str::trim)
                .filter(|t| !t.is_empty())
        })
        .map(str::to_string)
        .collect()
}

#[test]
fn the_grammar_knows_the_callsigns_of_psk_reporter() {
    let Ok(path) = std::env::var("JTDX_ALLCALL7") else {
        eprintln!("skipping: JTDX_ALLCALL7 unset (JTDX's ALLCALL7.TXT)");
        return;
    };
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let calls = entries(&text);
    assert!(
        calls.len() > 100_000,
        "{} entries read from {path}: not the list",
        calls.len()
    );
    let refused: Vec<&String> = calls.iter().filter(|c| !is_plausible_call(c)).collect();
    let share = refused.len() as f64 / calls.len() as f64;
    println!(
        "{} callsigns, {} refused ({:.4} %): {:?}",
        calls.len(),
        refused.len(),
        100.0 * share,
        &refused[..refused.len().min(30)]
    );
    // 0.009 % today; a ceiling with room for a different year's list, not a target
    assert!(
        share <= 0.0002,
        "the grammar refuses {} of {} callsigns spotted on PSK Reporter ({:.4} %)",
        refused.len(),
        calls.len(),
        100.0 * share
    );
}

/// The reader itself: comments, blank lines, trailing commas, several entries on a line.
#[test]
fn the_list_reader_reads_jtdx_and_wsjtx_lines() {
    let t = "// header\n\n2D0OMN,\n2E0AAG,\n1A0C,JN61FV,,,\nK1ABC\n";
    assert_eq!(entries(t), ["2D0OMN", "2E0AAG", "1A0C", "JN61FV", "K1ABC"]);
}
