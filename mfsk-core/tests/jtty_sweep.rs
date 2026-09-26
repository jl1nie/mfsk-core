// SPDX-License-Identifier: GPL-3.0-or-later
//! JTTY sensitivity sweep (tier C, `#[ignore]`d, never in CI).
//!
//! Reads the corpus `scripts/gen_jtty_sweep_wavs.sh` writes under
//! `embedded-poc/assets/jtty_sweep/` (`jtty_<channel>_<m|p><snr>_<trial>.wav`, the
//! message `CQ K1ABC CQ` at 1500 Hz) and scores every file through the
//! streaming receiver's one-shot form, `Receiver::scan`. A file passes when the
//! injected message comes out; `extra` counts every other decoded frame, which is
//! what `sweep-regression-check.py` compares as the precision column.
//!
//! `MFSK_JTTY_SWEEP_CSV` — per-trial CSV (`channel,snr_db,trial,pass,extra`), the
//! shape `scripts/run-sensitivity-sweeps.sh` hands to the regression checker.
//! Groups are `jtty/<channel>`. Set `MFSK_JTTY_SWEEP_DIR` to read another corpus.

#![cfg(all(feature = "jtty", any(feature = "fft-rustfft", feature = "fft-extern")))]

#[allow(dead_code)]
mod common;

use std::io::Write;
use std::path::PathBuf;

use mfsk_core::jtty::rx::{Params, Receiver};

const MESSAGE: &str = "CQ K1ABC CQ";

struct Trial {
    channel: String,
    snr_db: i32,
    trial: u32,
    path: PathBuf,
}

/// `jtty_<channel>_<m|p><snr>_<trial>.wav`; the channel may itself contain `_`.
fn parse(name: &str) -> Option<(String, i32, u32)> {
    let stem = name.strip_prefix("jtty_")?.strip_suffix(".wav")?;
    let (rest, trial) = stem.rsplit_once('_')?;
    let (channel, snr) = rest.rsplit_once('_')?;
    let mag: i32 = snr.get(1..)?.parse().ok()?;
    let snr_db = match snr.as_bytes()[0] {
        b'm' => -mag,
        b'p' => mag,
        _ => return None,
    };
    Some((channel.to_string(), snr_db, trial.parse().ok()?))
}

#[test]
#[ignore = "tier C: needs scripts/gen_jtty_sweep_wavs.sh's corpus"]
fn jtty_snr_sweep() {
    let dir = match std::env::var("MFSK_JTTY_SWEEP_DIR") {
        Ok(d) => PathBuf::from(d),
        Err(_) => match common::corpus::optional_corpus("jtty_sweep") {
            Some(d) => d,
            None => {
                eprintln!("jtty_sweep corpus absent (scripts/gen_jtty_sweep_wavs.sh) — skipping");
                return;
            }
        },
    };
    let mut trials: Vec<Trial> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| {
            let path = e.ok()?.path();
            let (channel, snr_db, trial) = parse(path.file_name()?.to_str()?)?;
            Some(Trial {
                channel,
                snr_db,
                trial,
                path,
            })
        })
        .collect();
    if trials.is_empty() {
        eprintln!("no jtty_*.wav under {} — skipping", dir.display());
        return;
    }
    trials.sort_by(|a, b| (&a.channel, a.snr_db, a.trial).cmp(&(&b.channel, b.snr_db, b.trial)));

    let rx = Receiver::new();
    let params = Params::default();
    let scored = common::par_map(&trials, |t| {
        let frames = rx.scan(&common::load_wav_i16(&t.path), &params);
        let hit = frames.iter().any(|f| f.atom.render() == MESSAGE);
        let extra = frames.iter().filter(|f| f.atom.render() != MESSAGE).count();
        (hit, extra)
    });

    let mut csv =
        common::sweep_csv_writer("MFSK_JTTY_SWEEP_CSV", "channel,snr_db,trial,pass,extra");
    let mut cells: std::collections::BTreeMap<(&str, i32), (usize, usize, usize)> =
        Default::default();
    for (t, &(hit, extra)) in trials.iter().zip(&scored) {
        if let Some(f) = csv.as_mut() {
            writeln!(
                f,
                "{},{},{},{},{}",
                t.channel,
                t.snr_db,
                t.trial,
                u8::from(hit),
                extra
            )
            .unwrap();
        }
        let c = cells.entry((t.channel.as_str(), t.snr_db)).or_default();
        c.0 += usize::from(hit);
        c.1 += 1;
        c.2 += extra;
    }
    println!("{:<16} {:>6}  recall  unexpected", "channel", "SNR");
    for ((ch, snr), (ok, n, extra)) in &cells {
        println!("{ch:<16} {snr:>6}  {ok:>2}/{n:<3}  {extra}");
    }
}
