// SPDX-License-Identifier: GPL-3.0-only
//! `msg::jt72::pack_message` / `unpack` against WSJT-X's own `packmsg` / `unpackmsg` (tier A).
//!
//! `tests/fixtures/jt72_rc1.tsv` is made by `scripts/jt72/gen_cases.sh` from
//! `lib/packjt.f90` of the `v3.2.0-rc1` tag (built by `scripts/jt72/build_oracle.sh`): standard
//! messages, every compound prefix/suffix form, free text of every length, and bare word patterns
//! (`U` rows) for what `packmsg` never makes. A missing fixture is a failure, not a skip.
//! Each row asserts identical words, identical `itype` and identical unpacked text.

use mfsk_core::msg::jt72::{pack_message, unpack};

const FIXTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/jt72_rc1.tsv");

fn words12(s: &str) -> [u8; 12] {
    let v: Vec<u8> = s.split_whitespace().map(|w| w.parse().unwrap()).collect();
    v.try_into().expect("twelve words")
}

#[test]
fn pack_and_unpack_match_rc1_row_by_row() {
    let data = std::fs::read_to_string(FIXTURE)
        .unwrap_or_else(|e| panic!("fixture {FIXTURE} is required: {e}"));
    let (mut p_rows, mut u_rows) = (0, 0);
    let mut bad = Vec::new();
    for line in data.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = line.splitn(5, '\t').collect();
        assert_eq!(f.len(), 5, "{line:?}");
        let (kind, input, itype, words, text) = (f[0], f[1], f[2], f[3], f[4]);
        let want_words = words12(words);
        let want_text = text.replace("\\0", "\0");
        match kind {
            "P" => {
                p_rows += 1;
                let got = pack_message(input).unwrap();
                let got_text = unpack(&got.words).to_string();
                if got.words != want_words
                    || got.itype.to_string() != itype
                    || got_text != want_text
                {
                    bad.push(format!(
                        "P {input:?}: got itype {} {:?} {got_text:?}; rc1 itype {itype} {want_words:?} {want_text:?}",
                        got.itype, got.words
                    ));
                }
            }
            "U" => {
                u_rows += 1;
                let got_text = unpack(&words12(input)).to_string();
                if got_text != want_text {
                    bad.push(format!("U {input}: got {got_text:?}; rc1 {want_text:?}"));
                }
            }
            k => panic!("row kind {k}"),
        }
    }
    assert!(
        p_rows > 3000 && u_rows > 3000,
        "{p_rows} P rows, {u_rows} U rows"
    );
    assert!(
        bad.is_empty(),
        "{} of {} rows differ from rc1; first ones:\n{}",
        bad.len(),
        p_rows + u_rows,
        bad.iter().take(25).cloned().collect::<Vec<_>>().join("\n")
    );
}

/// Non-ASCII text is refused rather than packed byte by byte (documented divergence).
#[test]
fn non_ascii_is_refused() {
    assert!(pack_message("CQ K1ABC FN42").is_ok());
    assert!(pack_message("CQ K1ABC Å").is_err());
}
