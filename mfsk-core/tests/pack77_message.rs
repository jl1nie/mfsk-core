// SPDX-License-Identifier: GPL-3.0-only
//! `msg::wsjt77::pack77_message` against WSJT-X v3.2.0-rc1's own packer (tier A: invariants and
//! bit-exact parity on a vendored fixture; a missing fixture is a failure).
//!
//! `tests/fixtures/pack77_rc1.tsv` is made by `scripts/pack77/gen_cases.sh` from
//! `scripts/pack77/pack77_oracle.f90`, which calls `pack77` of rc1's `lib/77bit/packjt77.f90`
//! (built from the tag by `scripts/pack77/build_oracle.sh`) on rc1's own `lib/77bit/messages.txt` and
//! on cases for every candidate of `pack77_encode_core`. For each message it records what upstream
//! returns: the status, `i3.n3`, the 77 bits, what `unpack77` prints for them, and the calls the packer
//! committed to the hash table. These tests hold this crate to every column.

#[allow(dead_code)]
mod common;

use mfsk_core::msg::wsjt77::{Pack77Error, Pack77Options, pack77_message, unpack77};

struct Row {
    prefer: bool,
    msg: String,
    status: String,
    i3n3: String,
    bits: String,
    decoded: String,
    hashed: Vec<String>,
}

fn rows() -> Vec<Row> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/pack77_rc1.tsv");
    let text = std::fs::read_to_string(path).expect("tests/fixtures/pack77_rc1.tsv is vendored");
    text.lines()
        .filter(|l| !l.starts_with('#'))
        .map(|line| {
            let (prefer, rest) = line.split_once('\t').expect("prefer column");
            // The message itself may hold a tab: split the fixed columns from the right.
            let f: Vec<&str> = rest.rsplitn(6, '\t').collect();
            assert_eq!(f.len(), 6, "malformed fixture row: {line:?}");
            Row {
                prefer: prefer == "1",
                msg: f[5].to_string(),
                status: f[4].to_string(),
                i3n3: f[3].to_string(),
                bits: common::pack77_fixture::beta4_rr73(f[5], f[3], f[2]),
                decoded: f[1].to_string(),
                hashed: f[0]
                    .split(',')
                    .filter(|s| !s.is_empty())
                    .map(str::to_string)
                    .collect(),
            }
        })
        .collect()
}

fn error_of(status: &str) -> Pack77Error {
    match status {
        "not_encoded" => Pack77Error::NotEncoded,
        "free_text_too_long" => Pack77Error::FreeTextTooLong,
        "free_text_invalid" => Pack77Error::FreeTextInvalid,
        "preferred_family_rejected" => Pack77Error::PreferredFamilyRejected,
        "roundtrip_rejected" => Pack77Error::RoundtripRejected,
        other => panic!("unknown status {other:?}"),
    }
}

fn bit_string(bits: &[u8; 77]) -> String {
    bits.iter().map(|&b| char::from(b'0' + b)).collect()
}

fn options(prefer: bool) -> Pack77Options {
    Pack77Options::new().prefer_wspr_50bit(prefer)
}

#[test]
fn fixture_is_the_one_documented() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/pack77_rc1.tsv");
    let text = std::fs::read_to_string(path).expect("fixture");
    let header: Vec<&str> = text.lines().take_while(|l| l.starts_with('#')).collect();
    assert!(
        header[0].contains("v3.2.0-rc1 (567ad29ce6abf3d4a44f181cdbc7ceba0d73e5f4)"),
        "{}",
        header[0]
    );
    let rows = rows();
    // 1 303 cases; the 1 303 prefer=0 rows plus the 860 prefer=1 rows of up to three words.
    assert_eq!(rows.iter().filter(|r| !r.prefer).count(), 1303);
    assert_eq!(rows.len(), 2163);
    // Every type the encoder has is in it: the point of the table is that none is an unexercised branch.
    let mut seen = std::collections::BTreeSet::new();
    for r in rows.iter().filter(|r| r.status == "ok") {
        seen.insert(r.i3n3.clone());
    }
    let want = [
        "0.0", "0.1", "0.3", "0.4", "0.5", "0.6", "1.0", "2.0", "3.0", "4.0", "5.0",
    ];
    for t in want {
        assert!(seen.contains(t), "no ok row of type {t}: {seen:?}");
    }
    let errors: std::collections::BTreeSet<&str> = rows
        .iter()
        .filter(|r| r.status != "ok")
        .map(|r| r.status.as_str())
        .collect();
    for e in [
        "free_text_too_long",
        "free_text_invalid",
        "preferred_family_rejected",
    ] {
        assert!(errors.contains(e), "no {e} row");
    }
}

/// Identical 77 bits and `i3.n3`, or the same failure with upstream's status as the
/// `Pack77Error`, for every row.
#[test]
fn bits_and_status_match_rc1() {
    let mut bad = Vec::new();
    for r in rows() {
        let got = pack77_message(&r.msg, &options(r.prefer));
        match (&got, r.status.as_str()) {
            (Ok(p), "ok") => {
                let tag = format!("{}.{}", p.i3, p.n3);
                if bit_string(&p.bits) != r.bits || tag != r.i3n3 {
                    bad.push(format!(
                        "{:?} (prefer {}): want {} {}, got {} {}",
                        r.msg,
                        r.prefer,
                        r.i3n3,
                        r.bits,
                        tag,
                        bit_string(&p.bits)
                    ));
                }
            }
            (Err(e), s) if s != "ok" => {
                if *e != error_of(s) {
                    bad.push(format!(
                        "{:?} (prefer {}): want {s}, got {e:?}",
                        r.msg, r.prefer
                    ));
                }
            }
            _ => bad.push(format!(
                "{:?} (prefer {}): want {}, got {:?}",
                r.msg,
                r.prefer,
                r.status,
                got.map(|p| format!("{}.{}", p.i3, p.n3))
            )),
        }
    }
    assert!(
        bad.is_empty(),
        "{} rows differ from rc1:\n{}",
        bad.len(),
        bad.join("\n")
    );
}

/// Every accepted message reads back, through `unpack77`, as the text upstream prints.
#[test]
fn accepted_messages_unpack_to_what_rc1_prints() {
    let mut bad = Vec::new();
    let mut accepted = 0;
    for r in rows().iter().filter(|r| r.status == "ok") {
        let p = pack77_message(&r.msg, &options(r.prefer)).expect("accepted by the other test");
        accepted += 1;
        let got = unpack77(&p.bits).unwrap_or_else(|| "*unpack failed*".into());
        let want = r.decoded.as_str();
        if got.trim_end() != want {
            bad.push(format!("{:?}: want {:?}, got {got:?}", r.msg, r.decoded));
        }
    }
    assert!(accepted > 1000, "{accepted}");
    assert!(
        bad.is_empty(),
        "{} rows unpack differently:\n{}",
        bad.len(),
        bad.join("\n")
    );
}

/// `hashed_calls` is what rc1's `pack77` committed to the hash table: the oracle empties the
/// table before each message, packs it and lists `calls22`, so the column is upstream's
/// `pack77_hash_facts` for that message, once per call, oldest first. Covers the `<...>`
/// messages and the types 0.1, 0.6 (50-bit), 4 and 5 that stage a call behind a hash.
#[test]
fn hashed_calls_are_the_hash_facts_of_rc1() {
    let mut bad = Vec::new();
    let mut with_calls = 0;
    let mut by_type = std::collections::BTreeMap::<String, usize>::new();
    for r in rows().iter().filter(|r| r.status == "ok") {
        let p = pack77_message(&r.msg, &options(r.prefer)).expect("accepted");
        if p.hashed_calls != r.hashed {
            bad.push(format!(
                "{:?}: want {:?}, got {:?}",
                r.msg, r.hashed, p.hashed_calls
            ));
        }
        if !r.hashed.is_empty() {
            with_calls += 1;
            *by_type.entry(r.i3n3.clone()).or_default() += 1;
        }
    }
    assert!(
        bad.is_empty(),
        "{} rows differ:\n{}",
        bad.len(),
        bad.join("\n")
    );
    assert!(with_calls > 700, "{with_calls}");
    for t in [
        "0.1", "0.3", "0.4", "0.6", "1.0", "2.0", "3.0", "4.0", "5.0",
    ] {
        assert!(
            by_type.get(t).copied().unwrap_or(0) > 0,
            "no row of type {t} with calls: {by_type:?}"
        );
    }
}

/// The facts the issue spells out, independent of the fixture's column: what is staged for a
/// message that hashes a call, and nothing for free text and telemetry.
#[test]
fn hashed_calls_by_family() {
    let p = |t: &str| pack77_message(t, &Pack77Options::new()).unwrap();
    assert_eq!(p("CQ K1ABC FN42").hashed_calls, ["K1ABC"]);
    assert_eq!(p("<W9XYZ> K1ABC -11").hashed_calls, ["W9XYZ", "K1ABC"]);
    assert_eq!(
        p("<PA9XYZ> <G4ABC/P> 590003 IO91NP").hashed_calls,
        ["PA9XYZ", "G4ABC/P"]
    );
    assert_eq!(p("<W9XYZ> PJ4/K1ABC RR73").hashed_calls, ["W9XYZ"]);
    assert_eq!(p("CQ PJ4/K1ABC").hashed_calls, ["PJ4/K1ABC"]);
    assert_eq!(
        p("K1ABC RR73; W9XYZ <KH1/KH7Z> -08").hashed_calls,
        ["K1ABC", "W9XYZ", "KH1/KH7Z"]
    );
    assert_eq!(p("TNX BOB 73 GL").hashed_calls, Vec::<String>::new());
    assert_eq!(p("123456789ABCDEF012").hashed_calls, Vec::<String>::new());
    let wspr = Pack77Options::new().prefer_wspr_50bit(true);
    assert_eq!(
        pack77_message("<PJ4/K1ABC> FK52UD", &wspr)
            .unwrap()
            .hashed_calls,
        ["PJ4/K1ABC"]
    );
}

#[test]
fn options_and_error_types() {
    assert!(!Pack77Options::default().prefer_wspr_50bit);
    assert!(
        Pack77Options::new()
            .prefer_wspr_50bit(true)
            .prefer_wspr_50bit
    );
    // `<CALL> GRID6` is a WSPR message only on request.
    let text = "<PJ4/K1ABC> FK52UD";
    let plain = pack77_message(text, &Pack77Options::new()).unwrap();
    let wspr = pack77_message(text, &Pack77Options::new().prefer_wspr_50bit(true)).unwrap();
    assert_eq!((plain.i3, plain.n3), (4, 0));
    assert_eq!((wspr.i3, wspr.n3), (0, 6));

    fn is_error<E: core::error::Error + Copy + PartialEq>() {}
    is_error::<Pack77Error>();
    for (e, s) in [
        (Pack77Error::NotEncoded, "not encoded"),
        (Pack77Error::FreeTextTooLong, "free text too long"),
        (Pack77Error::FreeTextInvalid, "invalid free text"),
        (
            Pack77Error::PreferredFamilyRejected,
            "preferred message family rejected",
        ),
        (
            Pack77Error::RoundtripRejected,
            "internal round-trip check failed",
        ),
    ] {
        assert_eq!(e.to_string(), s);
    }
    assert_eq!(
        pack77_message("THIS IS FAR TOO LONG", &Pack77Options::new()),
        Err(Pack77Error::FreeTextTooLong)
    );
    assert_eq!(
        pack77_message("HELLO*WORLD", &Pack77Options::new()),
        Err(Pack77Error::FreeTextInvalid)
    );
}
