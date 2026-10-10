//! FST4W receive, against WSJT-X v3.3.0-beta1's `jt9 -W`.
//!
//! Tier B (`tests/common/golden.rs::assert_golden`, `max_extra: 0`): the cases
//! below are synthesised in-process (this crate's transmitter and a seeded
//! Gaussian noise; no recording is vendored), and `jt9 -W -p <T> -d 3 -f 1500
//! -F 100` was run over the same samples, written out by `write_vectors`, by
//! `scripts/fst4w/gen_decode_vectors.sh`. Its rows are
//! `embedded-poc/assets/golden/fst4w/decode_beta1.tsv`. The real FST4W-1800
//! recording from WSJT-X's samples (43 MB) is read from `$WSJTX_SAMPLES_DIR`
//! and skipped without it.
#![cfg(feature = "fst4w")]

#[allow(dead_code)]
mod common;

use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::engine::tx::{synthesize, synthesize_i16};
use mfsk_core::fst4w::encode::message_to_tones;
use mfsk_core::fst4w::{Fst4w120, Fst4w300};

const TSV: &str = asset_path!("golden/fst4w/decode_beta1.tsv");

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

/// `(message, tone-0 Hz, dt s, SNR in 2500 Hz)`
type Sig = (&'static str, f32, f32, f32);

struct Case {
    name: &'static str,
    period: u32,
    seed: u64,
    signals: &'static [Sig],
}

/// Noise sigma in 16-bit counts: loud enough that the signals' quantisation is
/// far below the noise.
const SIGMA: f64 = 300.0;

const CASES: &[Case] = &[
    Case {
        name: "w120_three",
        period: 120,
        seed: 1201,
        signals: &[
            ("K1ABC FN42 37", 1440.0, 0.2, -24.0),
            ("PJ4/K1ABC 37", 1500.0, -0.3, -26.0),
            ("<JA1XYZ> PM95AA", 1560.0, 0.1, -25.0),
        ],
    },
    Case {
        name: "w120_strong_pair",
        period: 120,
        seed: 1202,
        signals: &[
            ("W9XYZ EN50 23", 1470.0, 0.0, -18.0),
            ("VK3NV QF22 13", 1510.0, 0.4, -20.0),
        ],
    },
    Case {
        name: "w300_two",
        period: 300,
        seed: 3001,
        signals: &[
            ("DL1ABC JO62 30", 1455.0, 0.3, -30.0),
            ("K9AN/P 37", 1545.0, -0.2, -31.0),
        ],
    },
];

/// One period of 16-bit audio: the signals, `1 s` after the period's start as
/// FST4W transmits (`TX_START_OFFSET_S`) plus each `dt`, over Gaussian noise.
fn render(c: &Case) -> Vec<i16> {
    let n = c.period as usize * 12_000;
    let mut rng = Rng(c.seed);
    let mut x: Vec<f32> = (0..n).map(|_| (SIGMA * rng.gauss()) as f32).collect();
    for &(msg, f0, dt, snr) in c.signals {
        let tones = if c.period == 120 {
            message_to_tones::<Fst4w120>(msg)
        } else {
            message_to_tones::<Fst4w300>(msg)
        }
        .unwrap_or_else(|| panic!("{msg:?} does not pack"));
        let wave = if c.period == 120 {
            synthesize::<Fst4w120>(&tones, 12_000, f0, 1.0)
        } else {
            synthesize::<Fst4w300>(&tones, 12_000, f0, 1.0)
        };
        // peak amplitude for the SNR: A = sigma * sqrt(2 * 2500/6000) * 10^(snr/20)
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

/// Writes every case as `<dir>/<name>.wav` (`$FST4W_VECTOR_DIR`), for
/// `scripts/fst4w/gen_decode_vectors.sh` to run `jt9` over.
#[test]
#[ignore = "writes WAV files; run by scripts/fst4w/gen_decode_vectors.sh"]
fn write_vectors() {
    let dir = std::env::var("FST4W_VECTOR_DIR").expect("FST4W_VECTOR_DIR");
    for c in CASES {
        let pcm = render(c);
        let mut b = Vec::with_capacity(44 + pcm.len() * 2);
        let data = (pcm.len() * 2) as u32;
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
    let params = DecodeParams::for_band((1400.0, 1600.0))
        .rx_freq(1500.0)
        .tol(100.0);
    let slot = SlotInput::i16(&pcm);
    let rows = if c.period == 120 {
        Decoder::<Fst4w120>::new(params).decode(&slot).rows
    } else {
        Decoder::<Fst4w300>::new(params).decode(&slot).rows
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
fn fst4w_matches_jt9() {
    let Ok(tsv) = std::fs::read_to_string(TSV) else {
        assert!(
            std::env::var("MFSK_REQUIRE_CORPUS").is_err(),
            "decode_beta1.tsv missing"
        );
        eprintln!("skipping: decode_beta1.tsv missing");
        return;
    };
    for c in CASES {
        // jt9's rows for this case
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
        let got = run_case(c);
        assert_golden(
            &got,
            &set,
            Tolerances {
                freq_hz: 1.5,
                dt_sec: 0.15,
                snr_db: 2.5,
            },
            |d| d.clone(),
        );
    }
}

#[test]
fn clean_signal_decodes() {
    let tones = message_to_tones::<Fst4w120>("K1ABC FN42 37").unwrap();
    let audio = synthesize_i16::<Fst4w120>(&tones, 12_000, 1500.0, 1000);
    let mut slot = vec![0i16; 120 * 12_000];
    for (d, s) in slot.iter_mut().skip(12_000).zip(audio.iter()) {
        *d = *s;
    }
    let mut dec = Decoder::<Fst4w120>::with_defaults();
    let _ = &synthesize_i16::<Fst4w120>;
    let out = dec.decode(&SlotInput::i16(&slot));
    for r in &out.rows {
        eprintln!("{:?}", r.decoded);
    }
    assert_eq!(out.rows.len(), 1);
    assert_eq!(out.rows[0].decoded.text, "K1ABC FN42 37");
}

/// The WSJT-X sample `201230_0300.wav` (FST4W-1800, 43 MB, not vendored):
/// `jt9 -W -p 1800 -d 3 -f 1500 -F 200` decodes exactly
/// `0300 -44  0.3 1433 \`  DL0HOT JO60 30`.
#[test]
fn wsjtx_sample_fst4w_1800() {
    use mfsk_core::fst4w::Fst4w1800;
    let Ok(dir) = std::env::var("WSJTX_SAMPLES_DIR") else {
        eprintln!("skipping: WSJTX_SAMPLES_DIR unset");
        return;
    };
    let path = format!("{dir}/FST4+FST4W/201230_0300.wav");
    let Some(pcm) = common::load_wav_i16_opt(&path) else {
        eprintln!("skipping: {path} missing");
        return;
    };
    assert_eq!(pcm.len(), 1800 * 12_000);
    let mut dec = Decoder::<Fst4w1800>::new(
        DecodeParams::for_band((1300.0, 1700.0))
            .rx_freq(1500.0)
            .tol(200.0),
    );
    let t = std::time::Instant::now();
    let out = dec.decode(&SlotInput::i16(&pcm));
    eprintln!("decode took {:?}", t.elapsed());
    for r in &out.rows {
        eprintln!("{:?}", r.decoded);
    }
    assert_eq!(out.rows.len(), 1);
    let d = &out.rows[0].decoded;
    assert_eq!(d.text, "DL0HOT JO60 30");
    assert!((d.freq_hz - 1433.0).abs() < 1.0, "freq {}", d.freq_hz);
    assert!((d.dt_sec - 0.3).abs() < 0.05, "dt {}", d.dt_sec);
    assert!((d.snr_db - -44.0).abs() <= 1.5, "snr {}", d.snr_db);
}

fn params() -> DecodeParams {
    DecodeParams::for_band((1400.0, 1600.0))
        .rx_freq(1500.0)
        .tol(100.0)
}

/// One `K1ABC FN42 37` at 1500 Hz in noise, as `render` makes it.
fn lone_signal(snr: f32, seed: u64) -> Vec<i16> {
    let sig: &'static [Sig] = Box::leak(Box::new([("K1ABC FN42 37", 1500.0, 0.0, snr)]));
    render(&Case {
        name: "lone",
        period: 120,
        seed,
        signals: sig,
    })
}

/// Keff 50 has no CRC, so its word is accepted only when the known-call list
/// holds a non-blank entry that the message contains (`fst4_decode.f90:800-807`,
/// with beta1's `len_trim>0` guard: rc1 matched blank entries too, which
/// accepted any Keff-50 word at all).
///
/// The signal is at -32 dB (2500 Hz) with seed 9008 — one of ten seeds measured
/// at -32 dB, of which Keff 66 decodes six and Keff 50 with this entry eight
/// (the 9008 and 9009 realisations are the two it adds).
#[test]
fn keff50_needs_a_known_call() {
    let pcm = lone_signal(-32.0, 9008);
    let slot = SlotInput::i16(&pcm);
    let decode = |calls: &[&str]| {
        let mut d = Decoder::<Fst4w120>::new(params());
        let list: Vec<String> = calls.iter().map(|s| s.to_string()).collect();
        d.set_wcalls(&list).unwrap();
        d.decode(&slot).rows
    };
    assert!(
        decode(&[]).is_empty(),
        "no list: Keff 66 alone does not reach it"
    );
    assert!(
        decode(&["", "   ", ""]).is_empty(),
        "blank entries must vouch for nothing (beta1)"
    );
    assert!(
        decode(&["JA1XYZ PM95"]).is_empty(),
        "an unrelated entry must not"
    );
    let rows = decode(&["JA1XYZ PM95", "K1ABC FN42"]);
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].decoded.text, "K1ABC FN42 37");
    assert_eq!(rows[0].native.keff, 50);
}

/// A Keff-66 decode of a type-1 message adds its `CALL GRID` once (Deep only);
/// type 2 and type 3 messages add nothing; the list is capped at 100.
#[test]
fn keff66_decodes_teach_the_list() {
    let pcm = lone_signal(-18.0, 9100);
    let slot = SlotInput::i16(&pcm);
    let mut d = Decoder::<Fst4w120>::new(params());
    assert_eq!(d.decode(&slot).rows[0].native.keff, 66);
    assert_eq!(d.wcalls(), ["K1ABC FN42"]);
    d.decode(&slot);
    assert_eq!(
        d.wcalls(),
        ["K1ABC FN42"],
        "a known entry is not added twice"
    );

    let mut normal = Decoder::<Fst4w120>::new(params().depth(mfsk_core::decoder::Depth::Normal));
    assert_eq!(normal.decode(&slot).rows.len(), 1);
    assert!(
        normal.wcalls().is_empty(),
        "only Deep keeps the list (do_k50_decode)"
    );

    let many: Vec<String> = (0..101).map(|i| format!("C{i}")).collect();
    assert!(d.set_wcalls(&many).is_err());
    let full: Vec<String> = (0..100).map(|i| format!("C{i}")).collect();
    d.set_wcalls(&full).unwrap();
    d.decode(&SlotInput::i16(&lone_signal(-18.0, 9101)));
    assert_eq!(d.wcalls().len(), 100);
    assert_eq!(
        d.wcalls()[99],
        "K1ABC FN42",
        "the oldest entry is shifted out"
    );
    assert_eq!(d.wcalls()[0], "C1");
}

/// Two stations whose calls are unresolved read the same, `<...> PM95AA`; they
/// are two rows because their 22-bit hashes differ (`fst4_decode.f90:814-823`).
#[test]
fn unresolved_hashes_keep_two_rows() {
    static SIGS: [Sig; 2] = [
        ("<JA1XYZ> PM95AA", 1440.0, 0.0, -20.0),
        ("<JH1ABC> PM95AA", 1560.0, 0.0, -20.0),
    ];
    let pcm = render(&Case {
        name: "two_hashes",
        period: 120,
        seed: 1203,
        signals: &SIGS,
    });
    let mut d = Decoder::<Fst4w120>::new(params());
    let rows = d.decode(&SlotInput::i16(&pcm)).rows;
    assert_eq!(rows.len(), 2, "{rows:?}");
    assert!(rows.iter().all(|r| r.decoded.text == "<...> PM95AA"));
    let (a, b) = (rows[0].decoded.hash22, rows[1].decoded.hash22);
    assert!(a.is_some() && b.is_some() && a != b, "{a:?} {b:?}");
}
