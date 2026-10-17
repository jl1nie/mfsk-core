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

/// Every accepted row reads back as `wqdecode` prints it (blank runs aside: `wsprd`'s `%2d` power
/// against `wqdecode`'s left-trimmed one; a type 3 call is `<...>` there, a hash here). Rows whose
/// bits `msg::wspr::unpack` does not take at all are those `wsprd` cannot decode either: a call of
/// `CQ`/`QRZ`/`DE` and a grid slot holding a report.
#[test]
fn accepted_rows_unpack_to_what_wqdecode_prints() {
    let mut failures = Vec::new();
    let mut unreadable = Vec::new();
    let mut known = 0;
    for r in rows().into_iter().filter(|r| r.status == "ok") {
        let p = pack_message(&r.msg).unwrap();
        match unpack(&p.bits) {
            Some(m) => {
                let mut t = squash(&m.to_string());
                if let WsprMessage::Type3 { .. } = m {
                    let end = t.find('>').unwrap();
                    t = format!("<...>{}", &t[end + 1..]);
                    assert!(p.hashed_call.is_some());
                }
                if t != squash(&r.decoded) {
                    // Known differences of the receive side, which follows `wsprd_utils.c`: it
                    // prints Swaziland `3DA0..` as `3D0..` and a prefix outside 0-9A-Z as a blank.
                    let head = r.msg.split('/').next().unwrap_or("");
                    if r.msg.starts_with("3D")
                        || (r.msg.contains('/')
                            && head.len() <= 3
                            && head.bytes().any(|b| !b.is_ascii_alphanumeric()))
                    {
                        known += 1;
                        continue;
                    }
                    failures.push(format!("{:?}: want {:?}, got {t:?}", r.msg, r.decoded));
                }
            }
            None => unreadable.push(r.msg.clone()),
        }
    }
    assert!(
        failures.is_empty(),
        "{}",
        failures[..failures.len().min(30)].join("\n")
    );
    assert!(known < 20, "{known}");
    assert!(unreadable.len() < 150, "{}", unreadable.len());
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
