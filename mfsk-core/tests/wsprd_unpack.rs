// SPDX-License-Identifier: GPL-3.0-only
//! `msg::wspr::unpack` and the text the WSPR decoder reports, against WSJT-X v3.2.0-rc1's own `wsprd`
//! (tier A: bit-exact parity on a vendored fixture; a missing fixture is a failure).
//!
//! WSJT-X's WSPR receiver is `wsprd` (C), not `wqdecode` (Fortran, which `wspr_pack_message.rs` compares
//! the other direction): the message text is `unpk_`'s `call_loc_pow` (`lib/wsprd/wsprd_utils.c`),
//! printed by `wsprd.c:1535,1553`. `tests/fixtures/wsprd_unpack_rc1.tsv` is made by
//! `scripts/wsprd/gen_cases.sh` from `scripts/wsprd/wsprd_unpack_oracle.c`, which calls rc1's `unpk_` the way
//! `wsprd.c:1496` does, with an empty hash table or one holding the row's seed call.
//!
//! Each row: 50 payload bits, seed call (`-` = empty table), `unpk_`'s return (1 = `wsprd` prints nothing)
//! and the message field.

#![cfg(feature = "wspr")]

use mfsk_core::msg::wspr::unpack;
use mfsk_core::msg::{WsprMessage, wspr::pack_message};
use mfsk_core::wspr::WsprCallsignTable;

struct Row {
    bits: [u8; 50],
    seed: String,
    noprint: bool,
    text: String,
}

fn fixture_text() -> String {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/wsprd_unpack_rc1.tsv"
    );
    std::fs::read_to_string(path).expect("tests/fixtures/wsprd_unpack_rc1.tsv is vendored")
}

fn rows() -> Vec<Row> {
    fixture_text()
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|line| {
            let f: Vec<&str> = line.splitn(4, '\t').collect();
            assert_eq!(f.len(), 4, "malformed fixture row: {line:?}");
            let mut bits = [0u8; 50];
            assert_eq!(f[0].len(), 50, "{line:?}");
            for (b, c) in bits.iter_mut().zip(f[0].bytes()) {
                *b = c - b'0';
            }
            Row {
                bits,
                seed: f[1].to_string(),
                noprint: f[2] == "1",
                text: f[3].to_string(),
            }
        })
        .collect()
}

/// What this crate reports for a payload received with `seed` in the callsign table (`None`: `wsprd`
/// prints nothing): the path of the decoder, `unpack` then `WsprResult::to_decoded`'s rendering.
fn reported(bits: &[u8; 50], seed: &str) -> Option<String> {
    let m = unpack(bits)?;
    let mut table = WsprCallsignTable::new();
    if seed != "-" {
        table.record(&WsprMessage::Type2 {
            callsign: seed.to_string(),
            power_dbm: 0,
        });
    }
    let resolved = match &m {
        WsprMessage::Type3 { callsign_hash, .. } => table.resolve(*callsign_hash),
        _ => None,
    };
    Some(m.render(resolved))
}

#[test]
fn fixture_is_the_one_documented() {
    let text = fixture_text();
    let header: Vec<&str> = text.lines().take_while(|l| l.starts_with('#')).collect();
    assert!(
        header[0].contains("v3.2.0-rc1 (567ad29ce6abf3d4a44f181cdbc7ceba0d73e5f4)"),
        "{}",
        header[0]
    );
    let rows = rows();
    assert!(rows.len() > 20_000, "{}", rows.len());
    assert!(rows.iter().any(|r| r.noprint) && rows.iter().any(|r| !r.noprint));
    assert!(
        rows.iter()
            .any(|r| r.seed != "-" && r.text.starts_with('<'))
    );
    assert!(
        rows.iter()
            .any(|r| r.seed == "-" && r.text.starts_with("<...>"))
    );
}

#[test]
fn unpack_matches_wsprd() {
    let mut failures = Vec::new();
    let mut empty_text = 0;
    for r in rows() {
        let got = reported(&r.bits, &r.seed);
        let want = (!r.noprint).then_some(r.text.clone());
        // The one deliberate divergence (`msg::wspr::unpack`'s doc): `unpk_` returns 0 with an empty
        // `call_loc_pow` for `ntype == 63`, so `wsprd` prints a decode with no text; this crate
        // reports no decode.
        if want.as_deref() == Some("") && got.is_none() {
            empty_text += 1;
            continue;
        }
        if got != want {
            let bits: String = r.bits.iter().map(|&b| char::from(b'0' + b)).collect();
            failures.push(format!(
                "{bits} seed {}: wsprd {want:?}, got {got:?}",
                r.seed
            ));
        }
    }
    assert!(empty_text > 0, "the fixture has no ntype 63 row");
    assert!(
        failures.is_empty(),
        "{} mismatches:\n{}",
        failures.len(),
        failures
            .iter()
            .take(40)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// Every message `pack_message` accepts reads back as `wsprd` prints it, the Type 3 call resolved from the
/// table that heard it.
#[test]
fn encoded_messages_read_back_as_wsprd_prints_them() {
    let p = pack_message("<K1ABC> FN42LX 27").unwrap();
    let m = unpack(&p.bits).unwrap();
    assert_eq!(m.to_string(), "<...> FN42LX 27");
    assert_eq!(m.render(Some("K1ABC")), "<K1ABC> FN42LX 27");
}
