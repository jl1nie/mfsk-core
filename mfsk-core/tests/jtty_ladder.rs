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

// ── f32 metrics against f64 (the E1b experiment of #499) ─────────────────────────────

struct Lcg(u64);
impl Lcg {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn gauss(&mut self) -> f32 {
        let (a, b) = (self.uniform().max(1e-12), self.uniform());
        ((-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()) as f32
    }
}

/// Correlations of a real frame in the oracle's model (zero drift): each symbol the sum of two
/// half-symbol correlations, the true tone `A/2 + noise` in each, the rest noise.
fn synth(
    info: &[u8; INFO_BITS],
    snr_db: Option<f32>,
    rng: &mut Lcg,
) -> (Correlations, Correlations) {
    use mfsk_core::jtty::tbcc;
    let tones = tbcc::encode(info);
    let amp = snr_db.map_or(0.0, |s| 10f32.powf(s / 20.0));
    let mut zsym = [[Complex32::new(0.0, 0.0); 4]; INFO_BITS];
    let mut zhalf = zsym;
    for k in 0..INFO_BITS {
        for t in 0..4 {
            let s = if snr_db.is_some() && t == usize::from(tones[k]) {
                amp / 2.0
            } else {
                0.0
            };
            let h1 = Complex32::new(s + 0.5 * rng.gauss(), 0.5 * rng.gauss());
            let h2 = Complex32::new(s + 0.5 * rng.gauss(), 0.5 * rng.gauss());
            zsym[k][t] = h1 + h2;
            zhalf[k][t] = Complex32::new((h1.norm_sqr() + h2.norm_sqr()).sqrt(), 0.0);
        }
    }
    (zsym, zhalf)
}

fn random_info(rng: &mut Lcg) -> [u8; INFO_BITS] {
    use mfsk_core::jtty::crc;
    let mut payload = [0u8; 34];
    for b in payload.iter_mut().take(32) {
        *b = u8::from(rng.uniform() < 0.5);
    }
    payload[33] = u8::from(rng.uniform() < 0.5);
    crc::append(&payload)
}

/// The trellis metrics in `f32` against `f64` (the E1b experiment of #499; `f32` is hardware on
/// an Xtensa LX7, `f64` is software). Measured with 300 frames per SNR (and 2 000 of noise):
/// of 12 300 lists one differed in words and order, 0 of 4 100 frames accepted a different
/// word or rung, false accepts were equal, and against upstream's 33 cases every one of the
/// 132 lists is identical, the worst clean metric off by 3.4e-7 relative. This test runs a
/// fifth of that.
#[test]
fn f32_metrics_agree_with_f64_and_with_upstream() {
    let plans = [Plan::new(1), Plan::new(2), Plan::new(4)];
    // 1. the 33 upstream cases
    if let Some(cases) = load() {
        let (mut lists, mut list_diff, mut worst_rel) = (0, 0, 0f64);
        for c in &cases {
            for r in &c.rungs {
                let (plan, z) = match r.name.as_str() {
                    "L1" => (&plans[0], &c.zsym),
                    "L2" => (&plans[1], &c.zsym),
                    "L4" => (&plans[2], &c.zsym),
                    _ => (&plans[0], &c.zhalf),
                };
                let g = plan.decode_f32(z, true);
                lists += 1;
                let same = g.pool == r.pool
                    && g.hypotheses.len() == r.count
                    && g.hypotheses.iter().zip(&r.lines).all(|(h, w)| {
                        bits_string(&h.bits) == w.bits
                            && h.crc_valid == w.crc
                            && h.start_state == w.start
                    });
                if !same {
                    list_diff += 1;
                    eprintln!(
                        "  case {} rung {}: list differs from upstream",
                        c.id, r.name
                    );
                }
                for (h, w) in g.hypotheses.iter().zip(&r.lines) {
                    worst_rel =
                        worst_rel.max((h.clean_metric - w.clean).abs() / w.clean.abs().max(1.0));
                }
            }
        }
        let ladder = Ladder::new().with_f32_metrics();
        let mut accept_diff = 0;
        for c in &cases {
            let got = ladder.decode(&c.zsym, &c.zhalf);
            let ok = match (&got, c.ok) {
                (Some(a), true) => {
                    bits_string(&a.payload) == c.payload && a.rung == c.rungs_evaluated
                }
                (None, false) => true,
                _ => false,
            };
            accept_diff += usize::from(!ok);
        }
        eprintln!(
            "upstream cases: {lists} lists, {list_diff} differ in words/order/crc/start/pool; worst clean-metric relative error {worst_rel:.2e}; ladder accept differs in {accept_diff} of {}",
            cases.len()
        );
        assert_eq!(list_diff, 0, "f32 lists must be upstream's");
        assert_eq!(
            accept_diff, 0,
            "f32 must accept upstream's word at upstream's rung"
        );
        assert!(worst_rel < 1e-5, "clean metric off by {worst_rel:e}");
    }

    // 2. f64 against f32 on many synthetic frames
    let (f64l, f32l) = (Ladder::new(), Ladder::new().with_f32_metrics());
    let mut rng = Lcg(0x477);
    let (mut all_list_diff, mut all_acc_diff, mut all_fa_extra) = (0usize, 0usize, 0i64);
    eprintln!(
        "{:>11} {:>6} {:>10} {:>9} {:>9} {:>10} {:>10}",
        "input", "frames", "list-diff", "acc-diff", "f64 right", "f32 right", "f64/f32 false"
    );
    for (label, snr) in [
        ("+9 dB", Some(9.0)),
        ("+6 dB", Some(6.0)),
        ("+4 dB", Some(4.0)),
        ("+2 dB", Some(2.0)),
        ("0 dB", Some(0.0)),
        ("-2 dB", Some(-2.0)),
        ("-4 dB", Some(-4.0)),
        ("noise", None),
    ] {
        let n = if snr.is_none() { 400 } else { 60 };
        let (mut list_diff, mut acc_diff, mut r64, mut r32, mut fa64, mut fa32) =
            (0, 0, 0, 0, 0, 0);
        for _ in 0..n {
            let info = random_info(&mut rng);
            let (zsym, zhalf) = synth(&info, snr, &mut rng);
            for (i, plan) in plans.iter().enumerate() {
                let (a, b) = (plan.decode(&zsym, true), plan.decode_f32(&zsym, true));
                let same = a.pool == b.pool
                    && a.hypotheses.len() == b.hypotheses.len()
                    && a.hypotheses
                        .iter()
                        .zip(&b.hypotheses)
                        .all(|(x, y)| x.bits == y.bits && x.crc_valid == y.crc_valid);
                list_diff += usize::from(!same);
                let _ = i;
            }
            let (a, b) = (f64l.decode(&zsym, &zhalf), f32l.decode(&zsym, &zhalf));
            let word = |x: &Option<mfsk_core::jtty::ladder::Accepted>| {
                x.as_ref().map(|a| (a.payload, a.rung))
            };
            acc_diff += usize::from(word(&a) != word(&b));
            let truth = |x: &Option<mfsk_core::jtty::ladder::Accepted>| {
                x.as_ref().is_some_and(|a| a.payload[..] == info[..34])
            };
            let false_acc = |x: &Option<mfsk_core::jtty::ladder::Accepted>| {
                x.as_ref().is_some_and(|a| a.payload[..] != info[..34])
            };
            r64 += usize::from(truth(&a));
            r32 += usize::from(truth(&b));
            fa64 += usize::from(false_acc(&a));
            fa32 += usize::from(false_acc(&b));
        }
        eprintln!(
            "{label:>11} {n:>6} {list_diff:>10} {acc_diff:>9} {r64:>9} {r32:>10} {fa64:>5}/{fa32:<4}"
        );
        all_list_diff += list_diff;
        all_acc_diff += acc_diff;
        all_fa_extra += fa32 as i64 - fa64 as i64;
    }
    // a near-tie may reorder one list in thousands; an accepted word or rung must not change
    assert!(all_list_diff <= 2, "{all_list_diff} lists differ");
    assert_eq!(all_acc_diff, 0, "f32 accepted a different word or rung");
    assert!(
        all_fa_extra <= 0,
        "f32 made {all_fa_extra} more false accepts"
    );
}
