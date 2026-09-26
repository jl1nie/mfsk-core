// SPDX-License-Identifier: GPL-3.0-or-later
//! The JTTY decode ladder against WSJT-X's own (tier A/B): the decoder boundary.
//!
//! `embedded-poc/assets/golden/jtty/ladder_cases.txt` holds 33 generated frames'
//! correlations (`ZS`, full-symbol; `ZH`, half-symbol) with what upstream's
//! `jtty_tbcc_decode` and its list decoder return for them, written by
//! `scripts/jttysim/jtty_ladder_oracle.f90` (see the fixtures' README).
//!
//! Same correlations in, so this checks the trellis and the ladder and nothing
//! in front of them: the same words in the same order with the same CRC flags,
//! start states and metrics (to f64 tolerance), and the same accepted rung.

#![cfg(feature = "jtty")]

#[allow(dead_code)]
mod common;

use mfsk_core::jtty::INFO_BITS;
use mfsk_core::jtty::ladder::Ladder;
use mfsk_core::jtty::trellis::{Correlations, HYPOTHESES, Plan};
use num_complex::Complex32;

struct ListLine {
    rank: usize,
    bits: String,
    crc: bool,
    clean: f64,
    wava: f64,
    start: u32,
}

struct Rung {
    name: String,
    count: usize,
    pool: usize,
    lines: Vec<ListLine>,
}

struct Case {
    id: usize,
    snr: f32,
    zsym: Correlations,
    zhalf: Correlations,
    ok: bool,
    rungs_evaluated: usize,
    block: usize,
    rank: usize,
    half: bool,
    pool: usize,
    metric: f64,
    payload: String,
    rungs: Vec<Rung>,
}

fn kv<'a>(tok: &'a str, key: &str) -> &'a str {
    tok.strip_prefix(key)
        .unwrap_or_else(|| panic!("{tok} lacks {key}"))
}

fn load() -> Option<Vec<Case>> {
    let path = common::corpus::golden_path("jtty/ladder_cases.txt")?;
    let text = std::fs::read_to_string(path).unwrap();
    let mut cases: Vec<Case> = Vec::new();
    let zero = [[Complex32::new(0.0, 0.0); 4]; INFO_BITS];
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = line.split_whitespace().collect();
        match f[0] {
            "CASE" => cases.push(Case {
                id: f[1].parse().unwrap(),
                snr: f[2].parse().unwrap(),
                zsym: zero,
                zhalf: zero,
                ok: false,
                rungs_evaluated: 0,
                block: 0,
                rank: 0,
                half: false,
                pool: 0,
                metric: 0.0,
                payload: String::new(),
                rungs: Vec::new(),
            }),
            "ZS" => {
                let t: usize = f[1].parse::<usize>().unwrap() - 1;
                let v: Vec<f32> = f[2..].iter().map(|x| x.parse().unwrap()).collect();
                cases.last_mut().unwrap().zsym[t] =
                    core::array::from_fn(|k| Complex32::new(v[2 * k], v[2 * k + 1]));
            }
            "ZH" => {
                let t: usize = f[1].parse::<usize>().unwrap() - 1;
                let v: Vec<f32> = f[2..].iter().map(|x| x.parse().unwrap()).collect();
                cases.last_mut().unwrap().zhalf[t] =
                    core::array::from_fn(|k| Complex32::new(v[k], 0.0));
            }
            "DEC" => {
                let c = cases.last_mut().unwrap();
                c.ok = f[1] == "T";
                c.rungs_evaluated = kv(f[2], "rungs=").parse().unwrap();
                c.block = kv(f[3], "block=").parse().unwrap();
                c.rank = kv(f[4], "rank=").parse().unwrap();
                c.half = kv(f[5], "half=") == "T";
                c.pool = kv(f[6], "pool=").parse().unwrap();
                // `metric=` and the value may be split by padding
                // a failed decode prints -huge as `-1.79...+308` (no `E`); ignore it
                if c.ok {
                    c.metric = f[7..]
                        .join("")
                        .trim_start_matches("metric=")
                        .parse()
                        .unwrap();
                }
            }
            "DECPAYLOAD" => cases.last_mut().unwrap().payload = f[1].into(),
            "RUNG" => cases.last_mut().unwrap().rungs.push(Rung {
                name: f[1].into(),
                count: kv(f[2], "count=").parse().unwrap(),
                pool: kv(f[3], "pool=").parse().unwrap(),
                lines: Vec::new(),
            }),
            "H" => cases
                .last_mut()
                .unwrap()
                .rungs
                .last_mut()
                .unwrap()
                .lines
                .push(ListLine {
                    rank: f[1].parse().unwrap(),
                    bits: f[2].into(),
                    crc: f[3] == "T",
                    clean: f[4].parse().unwrap(),
                    wava: f[5].parse().unwrap(),
                    start: f[6].parse().unwrap(),
                }),
            _ => {}
        }
    }
    Some(cases)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn bits_string(b: &[u8]) -> String {
    b.iter().map(|&x| char::from(b'0' + x)).collect()
}

#[test]
fn list_decoder_matches_upstream_word_for_word() {
    let Some(cases) = load() else { return };
    assert_eq!(cases.len(), 33);
    let plans = [Plan::new(1), Plan::new(2), Plan::new(4)];
    let mut lists = 0;
    for c in &cases {
        for r in &c.rungs {
            let (plan, z) = match r.name.as_str() {
                "L1" => (&plans[0], &c.zsym),
                "L2" => (&plans[1], &c.zsym),
                "L4" => (&plans[2], &c.zsym),
                "H1" => (&plans[0], &c.zhalf),
                other => panic!("rung {other}"),
            };
            let got = plan.decode(z, true);
            let ctx = format!("case {} (snr {}) rung {}", c.id, c.snr, r.name);
            assert_eq!(got.pool, r.pool, "{ctx}: pool");
            assert_eq!(got.hypotheses.len(), r.count, "{ctx}: count");
            assert!(got.hypotheses.len() <= HYPOTHESES);
            for (h, want) in got.hypotheses.iter().zip(&r.lines) {
                let ctx = format!("{ctx} rank {}", want.rank);
                assert_eq!(bits_string(&h.bits), want.bits, "{ctx}: word");
                assert_eq!(h.crc_valid, want.crc, "{ctx}: crc flag");
                assert_eq!(h.start_state, want.start, "{ctx}: start state");
                assert!(
                    close(h.clean_metric, want.clean),
                    "{ctx}: clean {} vs {}",
                    h.clean_metric,
                    want.clean
                );
                assert!(
                    close(h.wava_metric, want.wava),
                    "{ctx}: wava {} vs {}",
                    h.wava_metric,
                    want.wava
                );
            }
            lists += 1;
        }
    }
    assert_eq!(lists, 33 * 4);
}

#[test]
fn ladder_accepts_the_same_rung_and_word_as_upstream() {
    let Some(cases) = load() else { return };
    let ladder = Ladder::new();
    let mut seen = [0usize; 5]; // L1, L2, L4, half, fail
    for c in &cases {
        let got = ladder.decode(&c.zsym, &c.zhalf);
        let ctx = format!("case {} (snr {})", c.id, c.snr);
        match (&got, c.ok) {
            (Some(a), true) => {
                assert_eq!(bits_string(&a.payload), c.payload, "{ctx}: payload");
                assert_eq!(a.rung, c.rungs_evaluated, "{ctx}: rung");
                assert_eq!(a.half_symbol, c.half, "{ctx}: half");
                assert_eq!(a.rank, c.rank, "{ctx}: rank");
                assert_eq!(a.pool, c.pool, "{ctx}: pool");
                assert!(
                    close(a.metric, c.metric),
                    "{ctx}: metric {} vs {}",
                    a.metric,
                    c.metric
                );
                if !c.half {
                    assert_eq!(a.coherent, c.block, "{ctx}: block");
                }
                seen[if a.half_symbol {
                    3
                } else {
                    [0, 0, 1, 2][a.rung]
                }] += 1;
            }
            (None, false) => seen[4] += 1,
            _ => panic!(
                "{ctx}: accepted {:?} but upstream ok={}",
                got.is_some(),
                c.ok
            ),
        }
    }
    eprintln!("outcomes L1/L2/L4/half/fail: {seen:?}");
    assert!(
        seen.iter().all(|&n| n > 0),
        "every outcome must occur: {seen:?}"
    );
}
