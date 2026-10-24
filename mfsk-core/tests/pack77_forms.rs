// SPDX-License-Identifier: GPL-3.0-only
//! The per-form packers of `msg::wsjt77` (`pack77`, `pack77_type1`, `pack77_type4`,
//! `pack77_free_text`, `pack77_wspr`, `pack28`, `pack_grid4`) against WSJT-X v3.2.0-rc1's own
//! packer, except `... RR73`, which is v3.3.0-beta3's acknowledgment (see `common::pack77_fixture::beta4_rr73`) (tier A: bit-exact parity on the vendored fixture `tests/fixtures/pack77_rc1.tsv`,
//! the one `pack77_message.rs` holds `pack77_message` to; a missing fixture is a failure).
//!
//! Each `ok` row is split into the pieces the form takes and the form must give the fixture's
//! bits. For a row rc1 refuses, a form that takes pieces of that message must refuse too
//! (`pack77` / `pack77_type4` / `pack77_free_text` / `pack77_wspr`): rc1 refuses a message
//! only when no candidate, free text included, takes it.
//!
//! A form that takes pieces cannot see the whole-message dispatch, so for an `ok` row that
//! rc1 packs as another type the forms are not asked to agree, with these exceptions (each
//! asserted): `pack77_wspr` refuses everything that is not `0.6`, `pack77_type4` refuses a
//! standard call as its non-standard one, and `pack77` never gives a different type-1/2 word
//! than `pack77_message` for the same text.

#[allow(dead_code)]
mod common;

use mfsk_core::msg::wsjt77::{
    Pack77Options, pack_grid4, pack28, pack77, pack77_free_text, pack77_message, pack77_type1,
    pack77_type4, pack77_wspr,
};
use std::collections::BTreeMap;

struct Row {
    prefer: bool,
    msg: String,
    ok: bool,
    i3n3: String,
    bits: String,
}

fn rows() -> Vec<Row> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/pack77_rc1.tsv");
    let text = std::fs::read_to_string(path).expect("tests/fixtures/pack77_rc1.tsv is vendored");
    text.lines()
        .filter(|l| !l.starts_with('#'))
        .map(|line| {
            let (prefer, rest) = line.split_once('\t').expect("prefer column");
            let f: Vec<&str> = rest.rsplitn(6, '\t').collect();
            assert_eq!(f.len(), 6, "malformed fixture row: {line:?}");
            Row {
                prefer: prefer == "1",
                msg: f[5].to_string(),
                ok: f[4] == "ok",
                i3n3: f[3].to_string(),
                bits: common::pack77_fixture::beta4_rr73(f[5], f[3], f[2]),
            }
        })
        .collect()
}

fn bit_string(bits: &[u8; 77]) -> String {
    bits.iter().map(|&b| char::from(b'0' + b)).collect()
}

fn bits_of(o: Option<[u8; 77]>) -> String {
    o.map_or_else(|| "None".to_string(), |b| bit_string(&b))
}

/// Divergence classes with a count and the first example of each.
#[derive(Default)]
struct Report(BTreeMap<String, (usize, String)>);

impl Report {
    fn add(&mut self, class: &str, example: String) {
        let e = self.0.entry(class.to_string()).or_insert((0, example));
        e.0 += 1;
    }
    fn finish(self, what: &str) {
        for (class, (n, ex)) in &self.0 {
            eprintln!("{what}: {n} x {class}, e.g. {ex}");
        }
        assert!(
            self.0.is_empty(),
            "{what}: {} divergence class(es)",
            self.0.len()
        );
    }
}

/// A directed-CQ modifier (`CQ DX`, `CQ 123`): the first call field of a `1.0` / `2.0` row
/// whose 28 bits are a `CQ nnn` / `CQ xxxx` token.
fn cq_modifier_in_bits(bits: &str) -> bool {
    let n28 = u32::from_str_radix(&bits[..28], 2).unwrap();
    (3..2_063_592).contains(&n28)
}

/// `(call1, call2, report)` of a `1.0` / `2.0` row, as the pieces `pack77` takes.
fn type12_pieces(r: &Row) -> Option<(String, String, String)> {
    let w: Vec<&str> = r.msg.split_whitespace().collect();
    let (c1, rest) = if w.first().is_some_and(|c| c.eq_ignore_ascii_case("CQ"))
        && w.len() >= 3
        && cq_modifier_in_bits(&r.bits)
    {
        (format!("{} {}", w[0], w[1]), &w[2..])
    } else {
        (w.first()?.to_string(), &w[1..])
    };
    let (c2, tail) = rest.split_first()?;
    Some((c1, c2.to_string(), tail.join(" ")))
}

fn class_of(r: &Row) -> &'static str {
    if r.msg.contains('<') {
        "a <hash> call"
    } else if cq_modifier_in_bits(&r.bits) {
        "a directed CQ (CQ DX, CQ 123)"
    } else if r.msg.split_whitespace().any(|t| t.contains('/')) {
        "a /R or /P call"
    } else if r.msg.chars().any(|c| c.is_ascii_lowercase()) {
        "lower case"
    } else {
        "other type 1/2 text"
    }
}

#[test]
fn pack77_and_type1_give_rc1_bits() {
    let mut rep = Report::default();
    for r in rows().iter().filter(|r| !r.prefer) {
        if r.ok && (r.i3n3 == "1.0" || r.i3n3 == "2.0") {
            let Some((c1, c2, report)) = type12_pieces(r) else {
                panic!("cannot split {:?}", r.msg);
            };
            let got = bits_of(pack77(&c1, &c2, &report));
            if got != r.bits {
                rep.add(
                    &format!("pack77 differs on {}", class_of(r)),
                    format!("{:?}: want {}, got {got}", r.msg, r.bits),
                );
            }
            // `pack77_type1` takes a 4-character grid (`RR73` is one).
            if report.len() == 4 && pack_grid4(&report).is_some() {
                let got = bits_of(pack77_type1(&c1, &c2, &report));
                if got != r.bits {
                    rep.add(
                        &format!("pack77_type1 differs on {}", class_of(r)),
                        format!("{:?}: want {}, got {got}", r.msg, r.bits),
                    );
                }
            }
        } else if !r.ok {
            // rc1 refuses the text: no way of cutting it into pieces may make a type 1/2.
            let w: Vec<&str> = r.msg.split_whitespace().collect();
            let mut cuts: Vec<(String, String, String)> = Vec::new();
            if w.len() >= 2 {
                cuts.push((w[0].into(), w[1].into(), w[2..].join(" ")));
            }
            if w.len() >= 3 && w[0].eq_ignore_ascii_case("CQ") {
                cuts.push((format!("{} {}", w[0], w[1]), w[2].into(), w[3..].join(" ")));
            }
            for (a, b, c) in cuts {
                if let Some(bits) = pack77(&a, &b, &c) {
                    rep.add(
                        "pack77 accepts a message rc1 refuses",
                        format!(
                            "{:?} as ({a:?}, {b:?}, {c:?}): {}",
                            r.msg,
                            bit_string(&bits)
                        ),
                    );
                }
            }
        }
    }
    rep.finish("pack77/pack77_type1");
}

/// `pack77` on pieces that rc1 packs as another type is not asked to agree with rc1's bits,
/// but what it does give must be what `pack77_message` gives for the same text, as a
/// type 1/2 message.
#[test]
fn pack77_is_pack77_message_restricted_to_type_1_and_2() {
    let mut rep = Report::default();
    for r in rows().iter().filter(|r| !r.prefer && r.ok) {
        if r.i3n3 == "1.0" || r.i3n3 == "2.0" {
            continue;
        }
        let w: Vec<&str> = r.msg.split_whitespace().collect();
        if w.len() < 2 {
            continue;
        }
        let Some(bits) = pack77(w[0], w[1], &w[2..].join(" ")) else {
            continue;
        };
        let text = w.join(" ");
        let m = pack77_message(&text, &Pack77Options::new());
        if m.map(|p| p.bits) != Ok(bits) {
            rep.add(
                &format!(
                    "pack77 gives a bit pattern pack77_message does not (rc1 type {})",
                    r.i3n3
                ),
                format!("{:?}: pack77 {}", r.msg, bit_string(&bits)),
            );
        }
    }
    rep.finish("pack77 vs pack77_message");
}

#[test]
fn pack28_and_grid4_give_rc1_bits() {
    let mut rep = Report::default();
    for r in rows().iter().filter(|r| !r.prefer && r.ok) {
        if !(r.i3n3 == "1.0" || r.i3n3 == "2.0") {
            continue;
        }
        let Some((c1, c2, report)) = type12_pieces(r) else {
            continue;
        };
        let strip = |c: &str| -> String {
            let c = c.to_ascii_uppercase();
            c.strip_suffix("/P")
                .or_else(|| c.strip_suffix("/R"))
                .map_or(c.clone(), str::to_string)
        };
        for (field, call, lo) in [(1, &c1, 0usize), (2, &c2, 29)] {
            if call.contains('<') {
                continue; // a hashed call is not a standard one; `pack28` refuses it
            }
            let want = u32::from_str_radix(&r.bits[lo..lo + 28], 2).unwrap();
            let got = pack28(&strip(call));
            if got != Some(want) {
                rep.add(
                    &format!("pack28 differs (call {field})"),
                    format!("{:?} in {:?}: want {want}, got {got:?}", call, r.msg),
                );
            }
        }
        if report.len() == 4 {
            let want = u32::from_str_radix(&r.bits[59..74], 2).unwrap();
            let is_grid = (report.as_bytes()[0] as char).is_ascii_alphabetic();
            if is_grid && r.bits.as_bytes()[58] == b'0' && want < 32_400 {
                let got = pack_grid4(&report);
                if got != Some(want) {
                    rep.add(
                        "pack_grid4 differs",
                        format!("{report:?} in {:?}: want {want}, got {got:?}", r.msg),
                    );
                }
            }
        }
    }
    // Grids: every 4-character text rc1's `is_grid4` accepts packs as `a*1800 + b*100 + c*10 + d`.
    for a in b'A'..=b'R' {
        for b in b'A'..=b'R' {
            let g = format!("{}{}42", a as char, b as char);
            let want = u32::from(a - b'A') * 1800 + u32::from(b - b'A') * 100 + 42;
            if pack_grid4(&g) != Some(want) {
                rep.add("pack_grid4 differs on a grid", g);
            }
        }
    }
    for g in ["SA00", "AS00", "AA0A", "AA", "AA000", "A-00"] {
        if pack_grid4(g).is_some() {
            rep.add("pack_grid4 accepts a non-grid", g.to_string());
        }
    }
    rep.finish("pack28/pack_grid4");
}

#[test]
fn pack77_type4_gives_rc1_bits() {
    let mut rep = Report::default();
    let unbracket = |s: &str| s.trim_matches(|c| c == '<' || c == '>').to_string();
    for r in rows().iter().filter(|r| !r.prefer) {
        let w: Vec<String> = r
            .msg
            .split_whitespace()
            .map(|t| t.to_ascii_uppercase())
            .collect();
        if r.ok && r.i3n3 == "4.0" {
            let got;
            let want;
            if w[0] == "CQ" {
                got = bits_of(pack77_type4(&w[1], "", "", true));
                want = r.bits.clone();
            } else {
                let rep_ = w.get(2).cloned().unwrap_or_default();
                if w[0].starts_with('<') {
                    got = bits_of(pack77_type4(&w[1], &unbracket(&w[0]), &rep_, false));
                    want = r.bits.clone();
                } else {
                    // `nonstd <hash>`: the signature carries no order, so the form is the
                    // `<hash> nonstd` one; rc1's bits for that text are the reference.
                    got = bits_of(pack77_type4(&w[0], &unbracket(&w[1]), &rep_, false));
                    let flipped = format!("{} {} {}", w[1], w[0], rep_);
                    want = bit_string(
                        &pack77_message(flipped.trim(), &Pack77Options::new())
                            .expect("flipped form packs")
                            .bits,
                    );
                }
            }
            if got != want {
                rep.add(
                    "pack77_type4 differs",
                    format!("{:?}: want {want}, got {got}", r.msg),
                );
            }
        } else if r.ok && (r.i3n3 == "1.0" || r.i3n3 == "2.0") {
            // rc1 takes the two standard calls as a type 1/2 message, never as type 4.
            let rep_ = w.get(2).cloned().unwrap_or_default();
            if (2..=3).contains(&w.len())
                && !w[1].contains('/')
                && w[0] != "CQ"
                && !w[0].starts_with('<')
                && !w[1].starts_with('<')
                && let Some(b) = pack77_type4(&w[1], &w[0], &rep_, false)
            {
                rep.add(
                    "pack77_type4 accepts a standard call as the non-standard one",
                    format!("{:?}: {}", r.msg, bit_string(&b)),
                );
            }
            if w.len() == 2
                && w[0] == "CQ"
                && let Some(b) = pack77_type4(&w[1], "", "", true)
            {
                rep.add(
                    "pack77_type4 accepts CQ <standard call>",
                    format!("{:?}: {}", r.msg, bit_string(&b)),
                );
            }
        } else if !r.ok && w.len() >= 2 {
            // Only the texts that are literally the type-4 shapes `<hash> nonstd [report]`
            // and `CQ nonstd`: rc1 refusing those means the form must refuse.
            let rep_ = w.get(2).cloned().unwrap_or_default();
            if w.len() <= 3
                && w[0].starts_with('<')
                && w[0].ends_with('>')
                && let Some(b) = pack77_type4(&w[1], &unbracket(&w[0]), &rep_, false)
            {
                rep.add(
                    "pack77_type4 accepts a message rc1 refuses",
                    format!("{:?}: {}", r.msg, bit_string(&b)),
                );
            }
            if w.len() == 2
                && w[0] == "CQ"
                && let Some(b) = pack77_type4(&w[1], "", "", true)
            {
                rep.add(
                    "pack77_type4 accepts a CQ message rc1 refuses",
                    format!("{:?}: {}", r.msg, bit_string(&b)),
                );
            }
        }
    }
    rep.finish("pack77_type4");
}

#[test]
fn pack77_free_text_gives_rc1_bits() {
    let mut rep = Report::default();
    for r in rows().iter().filter(|r| !r.prefer) {
        let got = pack77_free_text(&r.msg);
        if r.ok && r.i3n3 == "0.0" {
            if bits_of(got) != r.bits {
                rep.add(
                    "pack77_free_text differs",
                    format!("{:?}: want {}, got {}", r.msg, r.bits, bits_of(got)),
                );
            }
        } else if !r.ok && got.is_some() {
            rep.add(
                "pack77_free_text accepts a text rc1 refuses",
                format!("{:?}: {}", r.msg, bits_of(got)),
            );
        }
    }
    rep.finish("pack77_free_text");
}

#[test]
fn pack77_wspr_gives_rc1_bits() {
    let mut rep = Report::default();
    for r in rows() {
        let got = pack77_wspr(&r.msg, r.prefer);
        if r.ok && r.i3n3 == "0.6" {
            if bits_of(got) != r.bits {
                rep.add(
                    "pack77_wspr differs",
                    format!(
                        "{:?} (prefer {}): want {}, got {}",
                        r.msg,
                        r.prefer,
                        r.bits,
                        bits_of(got)
                    ),
                );
            }
        } else if let Some(b) = got {
            rep.add(
                "pack77_wspr accepts a message rc1 does not pack as 0.6",
                format!(
                    "{:?} (prefer {}, rc1 {}): {}",
                    r.msg,
                    r.prefer,
                    r.i3n3,
                    bit_string(&b)
                ),
            );
        }
    }
    rep.finish("pack77_wspr");
}
