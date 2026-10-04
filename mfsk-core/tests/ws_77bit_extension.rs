// SPDX-License-Identifier: GPL-3.0-only
//! The 77-bit messages WS (the former WSJT-X Improved) composes for QSOs with non-standard, compound
//! and suffixed callsigns, decoded the way WSJT-X v3.2.0-rc1 decodes them (issue #568, #570).
//!
//! `tests/fixtures/ws_77bit_extension.tsv` holds 168 messages with, for each, what rc1's `unpack77`
//! returns for a receiver that has heard nothing and for one that has first unpacked the sender's CQ.
//! A hashed call must resolve exactly as it does there; in particular a call with `/P` or `/R` hashes
//! with its suffix (`packjt77.f90:356-390`, `:1755-1765`).

use mfsk_core::msg::hash_table::CallsignHashTable;
use mfsk_core::msg::wsjt77::{unpack77_learn, unpack77_with_hash};

fn bits(s: &str) -> Vec<u8> {
    s.bytes().map(|b| b - b'0').collect()
}

#[test]
fn ws_messages_decode_as_wsjtx_rc1_does() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/ws_77bit_extension.tsv"
    );
    let text = std::fs::read_to_string(path).expect("fixture file");
    let (mut rows, mut bad) = (0, Vec::new());
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = line.split('\t').collect();
        assert_eq!(f.len(), 13, "malformed fixture row: {line}");
        let (receiver, c77, cq, want_fresh, want_after_cq) =
            (f[2], bits(f[7]), bits(f[8]), f[11], f[12]);
        rows += 1;

        let mut fresh = CallsignHashTable::new();
        fresh.insert(receiver);
        let got_fresh = unpack77_with_hash(&c77, &fresh).unwrap_or_default();

        let mut heard = CallsignHashTable::new();
        heard.insert(receiver);
        let _ = unpack77_learn(&cq, &mut heard);
        let got_heard = unpack77_with_hash(&c77, &heard).unwrap_or_default();

        if got_fresh.trim() != want_fresh.trim() {
            bad.push(format!(
                "{} {} {}: empty table: want {want_fresh:?}, got {got_fresh:?}",
                f[0], f[1], f[4]
            ));
        }
        if got_heard.trim() != want_after_cq.trim() {
            bad.push(format!(
                "{} {} {}: after CQ: want {want_after_cq:?}, got {got_heard:?}",
                f[0], f[1], f[4]
            ));
        }
    }
    assert_eq!(rows, 168);
    assert!(
        bad.is_empty(),
        "{} of {} outcomes differ from rc1:\n{}",
        bad.len(),
        rows * 2,
        bad.join("\n")
    );
}
