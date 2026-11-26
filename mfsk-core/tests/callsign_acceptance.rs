// SPDX-License-Identifier: GPL-3.0-only
//! The callsign grammar of the plausibility verdict (`msg::wsjt77::is_plausible_call`), against real callsigns.
//!
//! The verdict is a **deliberate divergence from WSJT-X** (`FrameDecodable::MESSAGE_FILTER_DEFAULT`): WSJT-X accepts whatever
//! `unpack77` renders, this crate also asks that the callsigns of a message be grammatical, so that a CRC false positive is not
//! reported and not subtracted. The price of a grammar is the real callsign it does not know: `DM100ZM` and `LZ365BM`
//! (special-event stations; 28 real decodes in `ft8_lib/test/wav/20m_busy`, which `jt9` reports and the verdict refused).
//!
//! This test measures the grammar against a list of real callsigns: **JTDX's `ALLCALL7.TXT`**, 100 303 callsigns (its header says 100 302) spotted on
//! PSK Reporter in FT4/FT8 modes between 2024-06-20 and 2025-06-19 (its header: "being used for filtering out some false
//! FT8/FT4 decodes"). The list is not vendored (746 KB of someone else's data); point `JTDX_ALLCALL7` at it, or the test skips:
//!
//! ```sh
//! JTDX_ALLCALL7=/path/to/ALLCALL7.TXT cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test callsign_acceptance -- --nocapture
//! ```
//!
//! Measured 2026-11-26: `is_plausible_call` alone refuses **9 of 100 303** (0.009 %): `2EOJKH`, `2EOTFZ`, `2EOUNO` (a letter O for a
//! zero), `ZZZZ`, `PSE12M`, `PA5T0N`, and `8J4Y70A`, `8J9J2BM`, `DL0E35Y`, which read as real special-event calls (a four-character
//! prefix). Only the 225 entries no standard 28-bit call carries are ever sent as `i3=4`; the arm that judges those
//! (`is_plausible_call` **or** the private `is_c58_special_call`) is measured by
//! `wsjt77::tests::the_nonstandard_arm_knows_the_callsigns_of_psk_reporter` and refuses 8 of the 225. A grammar that refuses a real
//! station more often than that has to be loosened, and what it costs in phantoms is
//! `wsjt77::tests::phantom_survival_is_bounded`'s to say.

use mfsk_core::msg::CallsignHashTable;
use mfsk_core::msg::wsjt77::{
    Pack77Options, is_plausible_call, pack77_message, pack77_type4, unpack77_fields,
};

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

/// Pack `text` as WSJT-X's `pack77` does, unpack it with no hash table (a partner call is `<...>`, as on the air before the
/// table has seen it), and ask the verdict. Panics unless it is a type-4 (non-standard call) message: the arm under test.
fn verdict_of(text: &str) -> bool {
    let p = pack77_message(text, &Pack77Options::new()).unwrap_or_else(|e| panic!("{text}: {e}"));
    assert_eq!(p.i3, 4, "{text} is not an i3=4 message");
    unpack77_fields(&p.bits, &CallsignHashTable::new())
        .unwrap_or_else(|| panic!("{text} does not unpack"))
        .is_plausible()
}

/// The real decodes the verdict refused (`ft8_lib/test/wav/20m_busy`, 25 of the 28 reference decodes `jt9` reports for them):
/// the special-event stations `DM100ZM` and `LZ365BM`, in the shapes they were heard in (`CQ`, the partner a hash first, the
/// partner a hash second). Fails if the number part of a `c58` call is taken as one digit again (the prefix of `DM100ZM` is then
/// `DM10`, four characters).
#[test]
fn the_special_event_stations_of_20m_busy_are_plausible() {
    for text in [
        "CQ DM100ZM",
        "<K1ABC> DM100ZM RR73",
        "DM100ZM <K1ABC> 73",
        "CQ LZ365BM",
        "<K1ABC> LZ365BM RR73",
        "LZ365BM <K1ABC> 73",
    ] {
        assert!(verdict_of(text), "the verdict refuses {text}");
    }
}

/// What the loosening must **not** let in: the phantom of the same corpus (`CQ ZNEWVJUSXYW`, 11 letters: a CRC survivor of the
/// band + SIC profile with the verdict off), a call with no digit, one with a prefix of four characters, one with a run of four or
/// five digits. Fails if the digit requirement, the prefix length or the run limit is dropped (a run longer than the limit used to
/// leave its extra digits in the prefix: `AB1234CD` had the prefix `AB1`).
#[test]
fn the_loosening_keeps_the_phantoms_out() {
    for text in [
        "CQ ZNEWVJUSXYW", // the phantom of 20m_busy: no digit
        "CQ ABCDEFGHIJ",  // no digit
        "CQ ABCD123EF",   // a four-character prefix (stage S1b, not taken)
        "CQ AB1234CD",    // a run of four digits: not a callsign's number
        "CQ A12345B",     // five
    ] {
        let bits = match pack77_message(text, &Pack77Options::new()) {
            Ok(p) if p.i3 == 4 => p.bits,
            // a text the packer will not make a type 4 of: build the c58 field directly
            _ => pack77_type4(text.trim_start_matches("CQ "), "", "", true)
                .unwrap_or_else(|| panic!("cannot pack {text}")),
        };
        let fields = unpack77_fields(&bits, &CallsignHashTable::new())
            .unwrap_or_else(|| panic!("{text} does not unpack"));
        assert!(!fields.is_plausible(), "the verdict accepts {text}");
    }
}
