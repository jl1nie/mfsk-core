//! FT8 busy-band sweep (tier C, #447): recall AND unexpected decodes on files
//! that hold several signals, scattered in time, plus files with noise only.
//!
//! The AWGN/CCIR corpora hold one signal at DT 0 and never an empty band, so
//! they cannot show what crowding, a time offset or an empty band do to a
//! decoder: a candidate-list cap, the lag window of the coarse sync, an
//! acceptance gate that admits CRC-valid garbage. This corpus does
//! (`scripts/gen_ft8_busy_wavs.py`, see its header for how it is built and how its
//! SNR is calibrated):
//!
//! | set | files x signals | measures |
//! |---|---|---|
//! | `dt1` | 100 x 1 | one signal at -16 dB, DT -0.5..+1.5 s: a miss is a time-window problem |
//! | `busy10/20/40` | 40 x 10/20/40 | crowding: 200-2700 Hz, DT -0.5..+1.5 s, SNR -24..-6 dB |
//! | `noise` | 200 x 0 | every decode is unexpected |
//!
//! A truth signal is a **hit** when its message comes out within 5 Hz and 0.5 s
//! of its true frequency and DT. An **extra** is a distinct decoded message that
//! is not any message transmitted in that file, by text (the injected message
//! decoded at the wrong place is a recall miss, not an extra, as in tier B).
//!
//! Run:
//! ```sh
//! MFSK_FT8_BUSY_DIR=<dir> cargo test --release -p mfsk-core \
//!     --features full,internal-testing --test ft8_busy_sweep -- --ignored --nocapture
//! ```
//! `MFSK_FT8_BUSY_CSV=<path>` writes `set,strategy,trial,truth,hits,extra`, one
//! row per file and strategy, which `sweep-regression-check.py` aggregates.
//! `MFSK_FT8_BUSY_MAX_CAND` (default 600, WSJT-X 2.7's `MAXCAND`) is the
//! candidate cap: the default lets the decoder, not the cap, decide; 60 is what
//! the registry hands a caller who asks for the defaults.
#![cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::collections::BTreeMap;
use std::path::PathBuf;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;
use mfsk_core::msg::wsjt77::unpack77;

const FREQ_TOL_HZ: f32 = 5.0;
const DT_TOL_SEC: f32 = 0.5;
/// Sets in the order the corpus generator writes them; also the print order.
const SETS: &[&str] = &["dt1", "busy10", "busy20", "busy40", "noise"];

fn busy_dir() -> PathBuf {
    common::sweep_dir("MFSK_FT8_BUSY_DIR", "ft8_busy_sweep")
}

/// `MFSK_FT8_BUSY_SYNC_MIN` (default 1.3): the coarse-sync score a candidate
/// needs. 1.3 is what WSJT-X's `jt9 -d3` uses (`ft8_decode.f90`; 2.1 for `-d1/-d2`).
/// Measured on this corpus (2900 signals, 420 files, `max_cand` 600), `.sic_early()`
/// at 0.8 / 1.0 / 1.3 / 1.6 / 2.1: recall 82.0 / 81.9 / 82.0 / 81.3 / 78.5 %,
/// unexpected decodes 22 / 14 / 6 / 6 / 6. 0.8 (the FT8 sweep's value) admits noise
/// candidates that OSD then accepts by CRC chance.
fn sync_min() -> f32 {
    std::env::var("MFSK_FT8_BUSY_SYNC_MIN")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(1.3)
}

fn max_cand() -> usize {
    std::env::var("MFSK_FT8_BUSY_MAX_CAND")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(600)
}

struct Truth {
    msg: String,
    f0: f32,
    dt: f32,
}

/// `truth.csv`: `file,msg,f0,dt,snr`; a file with no rows is a noise file.
fn load_truth(dir: &std::path::Path) -> Option<BTreeMap<String, Vec<Truth>>> {
    let text = std::fs::read_to_string(dir.join("truth.csv")).ok()?;
    let mut out: BTreeMap<String, Vec<Truth>> = BTreeMap::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 5 {
            continue;
        }
        out.entry(f[0].trim_end_matches(".wav").to_string())
            .or_default()
            .push(Truth {
                msg: f[1].to_string(),
                f0: f[2].parse().ok()?,
                dt: f[3].parse().ok()?,
            });
    }
    Some(out)
}

/// `(hits, extra)` for one file: a decoded `(message, freq, dt)` list against
/// the transmitted signals. See the module doc for the two rules.
fn score(decoded: &[(String, f32, f32)], truth: &[Truth]) -> (u32, u32) {
    let hits = truth
        .iter()
        .filter(|t| {
            decoded.iter().any(|(m, f, d)| {
                *m == t.msg && (f - t.f0).abs() <= FREQ_TOL_HZ && (d - t.dt).abs() <= DT_TOL_SEC
            })
        })
        .count() as u32;
    let sent: Vec<&str> = truth.iter().map(|t| t.msg.as_str()).collect();
    let mut extras: Vec<&str> = decoded
        .iter()
        .map(|(m, _, _)| m.as_str())
        .filter(|m| !sent.contains(m))
        .collect();
    extras.sort_unstable();
    extras.dedup();
    let extra = extras.len() as u32;
    (hits, extra)
}

#[derive(Clone, Copy)]
enum Strategy {
    Single,
    SicEarly,
    SicRounds(usize),
}

impl Strategy {
    fn name(self) -> &'static str {
        match self {
            Strategy::Single => "single",
            Strategy::SicEarly => "sic_early",
            Strategy::SicRounds(_) => "sic_rounds",
        }
    }
}

/// `MFSK_FT8_BUSY_NO_MESSAGE_FILTER=1` turns the codec's plausibility verdict off (FT8's
/// default keeps it on, `FrameDecodable::MESSAGE_FILTER_DEFAULT`), to see what it removes.
fn no_message_filter() -> bool {
    std::env::var("MFSK_FT8_BUSY_NO_MESSAGE_FILTER").is_ok_and(|v| v == "1")
}

fn decode(audio: &[i16], strategy: Strategy, cap: usize, sync_min: f32) -> Vec<(String, f32, f32)> {
    use mfsk_core::ft8::Ft8;
    use mfsk_core::msg::decode_request::DecodeRequest;
    let req = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, sync_min, cap);
    let results = if no_message_filter() {
        // Every message that unpacks is kept: the codec's plausibility verdict, which
        // `ft8b.f90` does not have, is off (#456).
        let req = req.message_filter(|_| true);
        match strategy {
            Strategy::Single => req.decode().results,
            Strategy::SicEarly => req.sic_early().decode().results,
            Strategy::SicRounds(n) => req.sic_rounds(n).decode().results,
        }
    } else {
        match strategy {
            Strategy::Single => req.decode().results,
            Strategy::SicEarly => req.sic_early().decode().results,
            Strategy::SicRounds(n) => req.sic_rounds(n).decode().results,
        }
    };
    results
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (m, d.freq_hz, d.dt_sec)))
        .collect()
}

#[test]
fn scoring_rules() {
    let truth = vec![
        Truth {
            msg: "CQ A1AA AA11".into(),
            f0: 1000.0,
            dt: 0.0,
        },
        Truth {
            msg: "CQ B2BB BB22".into(),
            f0: 2000.0,
            dt: 1.0,
        },
    ];
    let d = |m: &str, f: f32, t: f32| (m.to_string(), f, t);
    // both found
    assert_eq!(
        score(
            &[
                d("CQ A1AA AA11", 1001.0, 0.1),
                d("CQ B2BB BB22", 2000.0, 1.0)
            ],
            &truth
        ),
        (2, 0)
    );
    // right text at the wrong frequency, and at the wrong time: neither a hit nor an extra
    assert_eq!(
        score(
            &[
                d("CQ A1AA AA11", 1020.0, 0.0),
                d("CQ B2BB BB22", 2000.0, 1.6)
            ],
            &truth
        ),
        (0, 0)
    );
    // a message nobody sent is an extra; the same one twice counts once
    assert_eq!(
        score(
            &[
                d("XX9 Z9ZZ ZZ99", 500.0, 0.0),
                d("XX9 Z9ZZ ZZ99", 900.0, 0.5),
                d("CQ A1AA AA11", 1000.0, 0.0)
            ],
            &truth
        ),
        (1, 1)
    );
    // noise file: no truth, every distinct decode is an extra
    assert_eq!(
        score(&[d("X 1", 0.0, 0.0), d("Y 2", 0.0, 0.0)], &[]),
        (0, 2)
    );
    assert_eq!(score(&[], &[]), (0, 0));
}

#[test]
#[ignore = "manual tier-C sweep — run with --ignored --nocapture"]
fn ft8_busy_sweep() {
    let dir = busy_dir();
    let Some(truth) = load_truth(&dir) else {
        common::skip_or_fail(&format!(
            "{} (generate with scripts/gen_ft8_busy_wavs.py)",
            dir.join("truth.csv").display()
        ));
        return;
    };
    let cap = max_cand();
    let sync_min = sync_min();

    struct Job {
        set: &'static str,
        trial: u32,
        file: String,
    }
    let mut jobs = Vec::new();
    for &set in SETS {
        for trial in 1..=999u32 {
            let file = format!("ft8_busy_{set}_{trial:02}");
            if !dir.join(format!("{file}.wav")).exists() {
                break;
            }
            jobs.push(Job { set, trial, file });
        }
    }
    assert!(!jobs.is_empty(), "no ft8_busy_*.wav in {}", dir.display());

    let mut csv =
        common::sweep_csv_writer("MFSK_FT8_BUSY_CSV", "set,strategy,trial,truth,hits,extra");
    use std::io::Write;

    eprintln!("\n{:-<72}", "");
    eprintln!("  max_cand {cap}  sync_min {sync_min}");
    eprintln!(
        "  {:<8} {:<10} {:>5} {:>6} {:>6} {:>8} {:>6}",
        "set", "strategy", "files", "truth", "hits", "recall", "extra"
    );
    eprintln!("{:-<72}", "");
    let mut strategies = vec![Strategy::Single, Strategy::SicEarly];
    // Diagnostic only (not in the baseline): `MFSK_FT8_BUSY_SIC_ROUNDS=n` adds `.sic_rounds(n)`.
    if let Some(n) = std::env::var("MFSK_FT8_BUSY_SIC_ROUNDS")
        .ok()
        .and_then(|s| s.trim().parse().ok())
    {
        strategies.push(Strategy::SicRounds(n));
    }
    for strategy in strategies {
        let rows: Vec<Option<(u32, u32, u32)>> = common::par_map(&jobs, |j| {
            let audio = load_wav_i16_opt(dir.join(format!("{}.wav", j.file)))?;
            let t = truth.get(&j.file).map(Vec::as_slice).unwrap_or(&[]);
            let (hits, extra) = score(&decode(&audio, strategy, cap, sync_min), t);
            Some((t.len() as u32, hits, extra))
        });
        let mut agg: BTreeMap<&str, (u32, u32, u32, u32)> = BTreeMap::new();
        for (j, r) in jobs.iter().zip(&rows) {
            let Some((n_truth, hits, extra)) = *r else {
                continue;
            };
            let a = agg.entry(j.set).or_default();
            *a = (a.0 + 1, a.1 + n_truth, a.2 + hits, a.3 + extra);
            if let Some(f) = csv.as_mut() {
                writeln!(
                    f,
                    "{},{},{},{n_truth},{hits},{extra}",
                    j.set,
                    strategy.name(),
                    j.trial
                )
                .unwrap();
            }
        }
        for &set in SETS {
            let Some(&(files, n_truth, hits, extra)) = agg.get(set) else {
                continue;
            };
            let recall = if n_truth > 0 {
                format!("{:.1}%", 100.0 * hits as f32 / n_truth as f32)
            } else {
                "-".into()
            };
            eprintln!(
                "  {:<8} {:<10} {:>5} {:>6} {:>6} {:>8} {:>6}",
                set,
                strategy.name(),
                files,
                n_truth,
                hits,
                recall,
                extra
            );
        }
    }
    eprintln!("{:-<72}", "");
}
