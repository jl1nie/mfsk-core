// SPDX-License-Identifier: GPL-3.0-only
//! `msg::wspr::pack_message` against WSJT-X v3.2.0-rc1's own `wqencode` (tier A: bit-exact parity on a
//! vendored fixture; a missing fixture is a failure).
//!
//! `tests/fixtures/wspr_pack_rc1.tsv` is made by `scripts/wspr/gen_cases.sh` from
//! `scripts/wspr/wspr_pack_oracle.f90`, which calls rc1's `wqencode` and then `wqdecode` (built from the tag
//! by `scripts/wspr/build_oracle.sh`). Each row records what upstream does with one message: packs it
//! (`ok`, with `ntype` and the 50 bits), returns without writing (`reject`), aborts with a Fortran runtime
//! error (`abort`), or reads memory it never wrote (`undefined`: two builds with different fill disagree).

#![cfg(feature = "wspr")]

use mfsk_core::msg::wspr::{WsprMessage, WsprPackError, pack_message, unpack};

struct Row {
    msg: String,
    status: String,
    ntype: i32,
    bits: String,
    decoded: String,
}

fn unescape(s: &str) -> String {
    let mut out = String::new();
    let mut it = s.chars();
    while let Some(c) = it.next() {
        if c == '\\' {
            match it.next() {
                Some('t') => out.push('\t'),
                Some(o) => out.push(o),
                None => out.push('\\'),
            }
        } else {
            out.push(c);
        }
    }
    out
}

fn fixture_text() -> String {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/wspr_pack_rc1.tsv"
    );
    std::fs::read_to_string(path).expect("tests/fixtures/wspr_pack_rc1.tsv is vendored")
}

fn rows() -> Vec<Row> {
    fixture_text()
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|line| {
            let f: Vec<&str> = line.split('\t').collect();
            // The message holds no raw tab (escaped), so five columns.
            assert_eq!(f.len(), 5, "malformed fixture row: {line:?}");
            Row {
                msg: unescape(f[0]),
                status: f[1].to_string(),
                ntype: f[2].parse().unwrap(),
                bits: f[3].to_string(),
                decoded: f[4].to_string(),
            }
        })
        .collect()
}

fn bit_string(bits: &[u8; 50]) -> String {
    bits.iter().map(|&b| char::from(b'0' + b)).collect()
}

fn squash(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
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
    assert!(rows.len() > 2000, "{}", rows.len());
    for status in ["ok", "reject", "abort", "undefined"] {
        assert!(rows.iter().any(|r| r.status == status), "{status}");
    }
}

#[test]
fn pack_message_matches_wqencode() {
    let mut failures = Vec::new();
    let mut compared = 0;
    for r in rows() {
        let got = pack_message(&r.msg);
        match (r.status.as_str(), got) {
            ("ok", Ok(p)) => {
                compared += 1;
                if bit_string(&p.bits) != r.bits || p.ntype != r.ntype {
                    failures.push(format!(
                        "{:?}: want ntype {} {}, got {} {}",
                        r.msg,
                        r.ntype,
                        r.bits,
                        p.ntype,
                        bit_string(&p.bits)
                    ));
                }
            }
            (
                "reject",
                Err(
                    WsprPackError::NotWsprMessage | WsprPackError::BadCall | WsprPackError::BadGrid,
                ),
            ) => {}
            (
                "abort",
                Err(
                    WsprPackError::BadPower
                    | WsprPackError::UnclosedHashedCall
                    | WsprPackError::CallTooLong,
                ),
            ) => {}
            // Upstream packs garbage here; this crate refuses (documented divergence).
            ("undefined", Err(_)) => {}
            (status, got) => failures.push(format!("{:?}: upstream {status}, got {got:?}", r.msg)),
        }
    }
    assert!(compared > 1000, "{compared}");
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

/// The `wsprd` oracle's answer (`tests/fixtures/wsprd_unpack_rc1.tsv`, see `tests/wsprd_unpack.rs`)
/// for a payload with an empty hash table: `Some(text)`, or `None` when `wsprd` prints nothing.
fn wsprd_prints(bits: &str) -> Option<String> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/wsprd_unpack_rc1.tsv"
    );
    let text =
        std::fs::read_to_string(path).expect("tests/fixtures/wsprd_unpack_rc1.tsv is vendored");
    let line = text
        .lines()
        .filter(|l| !l.starts_with('#'))
        .find(|l| l.starts_with(bits) && l[50..].starts_with("\t-\t"))
        .unwrap_or_else(|| panic!("{bits} is not in the wsprd fixture"));
    let f: Vec<&str> = line.splitn(4, '\t').collect();
    (f[2] == "0").then(|| f[3].to_string())
}

/// Every accepted row reads back as `wqdecode` prints it, except where `wsprd` differs from
/// `wqdecode`. WSJT-X's WSPR receiver runs `wsprd` (`lib/wsprd/wsprd_utils.c`), whose `unpk_` is
/// what `msg::wspr::unpack` ports and `tests/wsprd_unpack.rs` compares row by row; `wqdecode`
/// (`lib/wqdecode.f90`) is only the other half of the encoder's roundtrip. Blank runs aside
/// (`wsprd`'s `%2d` power against `wqdecode`'s left-trimmed one) the differences are, each
/// asserted below against the `wsprd` oracle's own answer for the same bits:
///
/// * Swaziland `3DA0..`: `packcall` packs it as `3D0..`; `wqdecode`'s `unpackcall` turns that back
///   into `3DA0..`, `wsprd`'s (`wsprd_utils.c:154-199`) does not, and prints `3D0..`;
/// * a prefix with a character outside `0-9A-Z`, or none: `packpfx` packs a blank for it,
///   `wqdecode` prints a blank for the whole call, `wsprd` prints the part after the last blank
///   (`:259`), `/CALL` when nothing is left;
/// * a call of `CQ` / `QRZ` / `DE`, or a report / `RO` / `RRR` / `73` in the grid slot: `unpackcall`
///   / `unpackgrid` refuse the field (`:330-331`), `wsprd` prints nothing, `unpack` is `None`.
///
/// A type 3 call is `<...>` in both for an empty table (`:394`).
#[test]
fn accepted_rows_unpack_to_what_wqdecode_prints() {
    let mut failures = Vec::new();
    let (mut swaziland, mut blank_prefix, mut unreadable) = (0, 0, 0);
    for r in rows().into_iter().filter(|r| r.status == "ok") {
        let p = pack_message(&r.msg).unwrap();
        let oracle = wsprd_prints(&bit_string(&p.bits));
        let got = unpack(&p.bits).map(|m| {
            if let WsprMessage::Type3 { .. } = m {
                assert!(p.hashed_call.is_some());
            }
            m.to_string()
        });
        // Whatever the difference from `wqdecode`, the text is `wsprd`'s.
        if got != oracle {
            failures.push(format!("{:?}: wsprd {oracle:?}, unpack {got:?}", r.msg));
            continue;
        }
        match got {
            None => unreadable += 1,
            Some(t) if squash(&t) == squash(&r.decoded) => {}
            Some(t) if r.decoded.starts_with("3DA0") && t.starts_with("3D0") => swaziland += 1,
            Some(t) if r.msg.contains('/') && !r.decoded.contains('/') && t.contains('/') => {
                blank_prefix += 1
            }
            Some(t) => failures.push(format!(
                "{:?}: wqdecode {:?}, wsprd {t:?}",
                r.msg, r.decoded
            )),
        }
    }
    assert!(
        failures.is_empty(),
        "{}",
        failures[..failures.len().min(30)].join("\n")
    );
    // The rows of the fixture that are these two documented differences (counted, so a new kind
    // of difference cannot hide in an allowance).
    assert_eq!((swaziland, blank_prefix), (4, 12));
    assert!(unreadable > 0);
}

#[test]
fn power_is_rounded_not_refused() {
    // 38 is sent as 37, 58 as 57, 61 as 60 (wqencode.f90:27-29).
    let a = pack_message("K1ABC FN42 37").unwrap();
    assert_eq!(pack_message("K1ABC FN42 38").unwrap(), a);
    assert_eq!(a.ntype, 37);
    assert_eq!(pack_message("K1ABC FN42 58").unwrap().ntype, 57);
    assert_eq!(pack_message("K1ABC FN42 61").unwrap().ntype, 60);
    assert_eq!(pack_message("K1ABC FN42 -5").unwrap().ntype, 0);
}

#[test]
fn error_variants() {
    use WsprPackError::*;
    let e = |t: &str| pack_message(t).unwrap_err();
    assert_eq!(e("K1ABC fn42 37"), BadGrid);
    assert_eq!(e("K1ABC.. FN42 37"), NotWsprMessage);
    assert_eq!(e("K1ABC FN42 xx"), BadPower);
    assert_eq!(e("K1ABC FN42"), BadPower);
    assert_eq!(e("ONE"), BadCall);
    assert_eq!(e(""), NotWsprMessage);
    assert_eq!(e("<K1ABC FN42LX 37"), UnclosedHashedCall);
    assert_eq!(e("<ABCDEFGHIJKLM> FN42LX 37"), CallTooLong);
    assert_eq!(e("PJ4/ABC 37"), BadCall);
    assert_eq!(e("<K1ABC> ZZ 37"), BadGrid);
    let t3 = pack_message("<K1ABC> FN42LX 27").unwrap();
    assert_eq!(t3.ntype, -28);
    assert_eq!(t3.hashed_call.as_deref(), Some("K1ABC"));
    assert!(!BadPower.to_string().is_empty());
}
