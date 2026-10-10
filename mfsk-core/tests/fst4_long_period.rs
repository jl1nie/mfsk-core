//! FST4-900 and FST4-1800 against WSJT-X v3.3.0-beta1's `jt9 -7` (#649, phase 3b).
//!
//! A 900 s recording is 21.6 MB and a 1800 s one 43 MB, so none is vendored: each
//! case is synthesised here (this crate's transmitter and seeded Gaussian
//! noise), `write_vectors` writes the same samples out for
//! `scripts/fst4w/gen_fst4_long_vectors.sh` to run `jt9 -7 -p <T> -d 3 -L 600 -H 1400`
//! over, and only jt9's rows are kept
//! (`embedded-poc/assets/golden/fst4/long_period_beta1.tsv`). Tier B through
//! `assert_golden`, `max_extra: 0`. The FST4 sweep corpus and its upstream task
//! (`fst4/t1`) stay at 15-300 s: a 900/1800 corpus would be tens of GB.
#![cfg(feature = "fst4")]

#[allow(dead_code)]
mod common;

use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::engine::protocol::Protocol;
use mfsk_core::engine::tx::{FskWaveform, message_to_tones, synthesize};
use mfsk_core::fst4::{Fst4s900, Fst4s1800};
use mfsk_core::msg::wsjt77::pack77;

const TSV: &str = asset_path!("golden/fst4/long_period_beta1.tsv");
const SIGMA: f64 = 300.0;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn uniform(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        let (a, b) = (self.uniform(), self.uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * core::f64::consts::PI * b).cos()
    }
}

struct Case {
    name: &'static str,
    period: u32,
    seed: u64,
    /// `(call1, call2, grid, tone-0 Hz, dt s, SNR in 2500 Hz)`
    signals: &'static [(&'static str, &'static str, &'static str, f32, f32, f32)],
}

const CASES: &[Case] = &[
    Case {
        name: "fst4_900",
        period: 900,
        seed: 9001,
        signals: &[
            ("CQ", "JL1NIE", "PM95", 1000.0, 0.2, -30.0),
            ("K1ABC", "W9XYZ", "EN50", 1100.0, -0.3, -32.0),
        ],
    },
    Case {
        name: "fst4_1800",
        period: 1800,
        seed: 1801,
        signals: &[("CQ", "JL1NIE", "PM95", 1000.0, 0.1, -33.0)],
    },
];

fn render_for<P: Protocol + FskWaveform>(c: &Case) -> Vec<i16> {
    let n = c.period as usize * 12_000;
    let mut rng = Rng(c.seed);
    let mut x: Vec<f32> = (0..n).map(|_| (SIGMA * rng.gauss()) as f32).collect();
    for &(a, b, g, f0, dt, snr) in c.signals {
        let msg = pack77(a, b, g).expect("pack77");
        let tones = message_to_tones::<P>(&msg);
        let wave = synthesize::<P>(&tones, 12_000, f0, 1.0);
        let amp = SIGMA * (2.0 * 2500.0 / 6000.0f64).sqrt() * 10f64.powf(snr as f64 / 20.0);
        let start = ((1.0 + dt) * 12_000.0) as usize;
        for (d, w) in x[start..].iter_mut().zip(wave.iter()) {
            *d += (amp * *w as f64) as f32;
        }
    }
    x.iter()
        .map(|v| v.round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

fn render(c: &Case) -> Vec<i16> {
    if c.period == 900 {
        render_for::<Fst4s900>(c)
    } else {
        render_for::<Fst4s1800>(c)
    }
}

/// Writes every case as `<dir>/<name>.wav` (`$FST4_LONG_VECTOR_DIR`).
#[test]
#[ignore = "writes WAV files; run by scripts/fst4w/gen_fst4_long_vectors.sh"]
fn write_vectors() {
    let dir = std::env::var("FST4_LONG_VECTOR_DIR").expect("FST4_LONG_VECTOR_DIR");
    for c in CASES {
        let pcm = render(c);
        let data = (pcm.len() * 2) as u32;
        let mut b = Vec::with_capacity(44 + pcm.len() * 2);
        b.extend(b"RIFF");
        b.extend((36 + data).to_le_bytes());
        b.extend(b"WAVEfmt ");
        b.extend(16u32.to_le_bytes());
        b.extend(1u16.to_le_bytes());
        b.extend(1u16.to_le_bytes());
        b.extend(12_000u32.to_le_bytes());
        b.extend(24_000u32.to_le_bytes());
        b.extend(2u16.to_le_bytes());
        b.extend(16u16.to_le_bytes());
        b.extend(b"data");
        b.extend(data.to_le_bytes());
        for s in &pcm {
            b.extend(s.to_le_bytes());
        }
        std::fs::write(format!("{dir}/{}.wav", c.name), b).unwrap();
    }
}

fn run_case(c: &Case) -> Vec<DecodeView> {
    let pcm = render(c);
    let params = DecodeParams::for_band((600.0, 1400.0));
    let slot = SlotInput::i16(&pcm);
    let rows = if c.period == 900 {
        Decoder::<Fst4s900>::new(params).decode(&slot).rows
    } else {
        Decoder::<Fst4s1800>::new(params).decode(&slot).rows
    };
    rows.iter()
        .map(|r| DecodeView {
            msg: r.decoded.text.clone(),
            freq_hz: r.decoded.freq_hz,
            dt_sec: r.decoded.dt_sec,
            snr_db: Some(r.decoded.snr_db),
        })
        .collect()
}

#[test]
fn fst4_long_periods_match_jt9() {
    let Ok(tsv) = std::fs::read_to_string(TSV) else {
        assert!(
            std::env::var("MFSK_REQUIRE_CORPUS").is_err(),
            "long_period_beta1.tsv missing"
        );
        eprintln!("skipping: long_period_beta1.tsv missing");
        return;
    };
    for c in CASES {
        let mut entries: Vec<GoldenEntry> = Vec::new();
        for line in tsv.lines().filter(|l| !l.starts_with('#')) {
            let f: Vec<&str> = line.split('\t').collect();
            if f[0] != c.name {
                continue;
            }
            let msg: &'static str = Box::leak(f[1].to_string().into_boxed_str());
            let (freq, dt, snr): (f32, f32, f32) = (
                f[2].parse().unwrap(),
                f[3].parse().unwrap(),
                f[4].parse().unwrap(),
            );
            entries.push(GoldenEntry::msg(msg).at(freq, dt).snr(snr));
        }
        assert!(!entries.is_empty(), "{}: no jt9 rows in the TSV", c.name);
        let n = entries.len();
        let set = GoldenSet {
            name: c.name,
            expected: Box::leak(entries.into_boxed_slice()),
            min_hits: n,
            max_extra: 0,
        };
        assert_golden(
            &run_case(c),
            &set,
            Tolerances {
                freq_hz: 1.5,
                dt_sec: 0.2,
                snr_db: 3.0,
            },
            |d| d.clone(),
        );
    }
}
