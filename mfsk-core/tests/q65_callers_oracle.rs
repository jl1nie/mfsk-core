// SPDX-License-Identifier: GPL-3.0-only
//! Q65's contest caller list against WSJT-X v3.3.0-beta1 itself (tier B).
//!
//! `embedded-poc/assets/golden/q65/callers_script.tsv` is a script of operations — record a
//! decoded message (`R`), expire (`X`), remove (`D`), build the full-AP list (`L`) — and
//! `callers_beta1.txt` is what upstream's `q65_record_caller`, `q65_expire_callers`,
//! `q65_remove_caller` and `q65_set_list2` made of it, through
//! `scripts/jttysim/q65_callers_oracle.f90` linked against a v3.3.0-beta1 `libwsjt_fort`.
//! After every `R` / `X` / `D` the caller list (call, grid, time, frequency) must be the same;
//! after every `L` the list must have the same size and the same first 13 symbols of every
//! codeword. The script covers the cases the beta1 audit (#642) turned on: eviction of the
//! least recently heard with ties, a refreshed caller surviving, expiry before recording,
//! compound calls, an empty second word, a grid past the 37 characters, ` R ` forms, and
//! the DX station as the 51st beside 50 callers (511 codewords).
//!
//! **Not covered, on purpose:** a caller token that is not a callsign (`K1ABC TNEW FN42`).
//! Upstream's `genq65` packs that message as truncated free text and lists codewords for it;
//! `contest_codewords` skips a message that will not pack. A real decoded caller is a
//! callsign, so the script keeps to callsigns.

#![cfg(feature = "q65")]

#[allow(dead_code)]
mod common;

use mfsk_core::q65::{Q65Callers, contest_codewords};

fn replay(script: &str) -> String {
    let mut out = String::new();
    let mut h = Q65Callers::new();
    let dump = |h: &Q65Callers, out: &mut String| {
        out.push_str(&format!("S {}\n", h.callers().len()));
        for c in h.callers() {
            out.push_str(&format!(
                "C {} {} {} {}\n",
                c.call, c.grid, c.last_heard, c.freq_hz
            ));
        }
    };
    for line in script
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let f: Vec<&str> = line.split('\t').collect();
        match f[0] {
            "R" => {
                h.record(
                    f[1].parse::<f32>().unwrap(),
                    f.get(3).copied().unwrap_or(""),
                    f[2].parse().unwrap(),
                );
                dump(&h, &mut out);
            }
            "X" => {
                h.expire(f[1].parse().unwrap());
                dump(&h, &mut out);
            }
            "D" => {
                h.remove(f[1]);
                dump(&h, &mut out);
            }
            "L" => {
                let cw = contest_codewords(f[1], f[2], f.get(3).copied().unwrap_or(""), &h);
                out.push_str(&format!("W {}\n", cw.len()));
                for (i, c) in cw.iter().enumerate() {
                    let s: Vec<String> = c[..13].iter().map(|v| v.to_string()).collect();
                    out.push_str(&format!("K {} {}\n", i + 1, s.join(" ")));
                }
            }
            other => panic!("unknown command {other:?}"),
        }
    }
    out
}

#[test]
fn caller_list_matches_upstream_beta1_operation_by_operation() {
    let script = std::fs::read_to_string(
        common::corpus::golden_path("q65/callers_script.tsv").expect("vendored script"),
    )
    .unwrap();
    let want = std::fs::read_to_string(
        common::corpus::golden_path("q65/callers_beta1.txt").expect("vendored upstream output"),
    )
    .unwrap();
    let want: String = want
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|l| format!("{l}\n"))
        .collect();
    let got = replay(&script);
    if got != want {
        let (g, w): (Vec<&str>, Vec<&str>) = (got.lines().collect(), want.lines().collect());
        let at = g
            .iter()
            .zip(&w)
            .position(|(a, b)| a != b)
            .unwrap_or(g.len().min(w.len()));
        panic!(
            "first difference at output line {at}: ours {:?}, upstream {:?} ({} vs {} lines)",
            g.get(at),
            w.get(at),
            g.len(),
            w.len()
        );
    }
    // the case #642 turned on: 50 callers and the DX station is the 51st
    assert!(want.lines().any(|l| l == "W 511"));
}
