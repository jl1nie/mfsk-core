// SPDX-License-Identifier: GPL-3.0-or-later
//! What a JTTY receive window costs, counted and timed (#499). Not an assertion suite: it prints.
//!
//! ```text
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features std,fft-rustfft,jtty-stats --test jtty_profile -- --ignored --nocapture
//! ```
//!
//! Without `parallel`, so that the stage times add up. Workloads: noise alone, upstream's sample
//! recording, and synthetic busy bands of 1, 3 and 6 stations at −12…−2 dB in 2 500 Hz.

#![cfg(feature = "jtty-stats")]

#[allow(dead_code)]
mod common;

use mfsk_core::jtty::pack::{self, ExchangeProfile};
use mfsk_core::jtty::rx::{NCHUNK, Params, Receiver, STEP};
use mfsk_core::jtty::stats::{Counter as C, Snapshot, Stage as S};
use mfsk_core::jtty::tx::{self, Synth};

struct Lcg(u64);
impl Lcg {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn gauss(&mut self) -> f64 {
        let (a, b) = (self.uniform().max(1e-12), self.uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
    }
}

const SIGMA: f64 = 1000.0;

/// White noise, `secs` long.
fn noise(secs: usize, rng: &mut Lcg) -> Vec<f64> {
    (0..secs * 12_000).map(|_| SIGMA * rng.gauss()).collect()
}

/// Add a transmission of `text` at `f0` Hz, starting `start_s` into the audio, `snr_db` in 2 500 Hz.
fn add_station(audio: &mut [f64], text: &str, f0: f32, start_s: f32, snr_db: f64) {
    let tones = pack::tones(text, ExchangeProfile::Unknown)
        .unwrap()
        .unwrap();
    let amp = (2.0 * SIGMA * SIGMA * 2500.0 / 6000.0 * 10f64.powf(snr_db / 10.0)).sqrt();
    let mut synth = Synth::<f64>::new(&tones, f0, amp as f32);
    let mut buf = vec![0f32; synth.total_samples()];
    synth.fill(&mut buf);
    let at = (start_s * 12_000.0) as usize;
    for (i, &x) in buf.iter().enumerate() {
        if let Some(a) = audio.get_mut(at + i) {
            *a += f64::from(x);
        }
    }
}

fn to_i16(a: &[f64]) -> Vec<i16> {
    a.iter()
        .map(|&x| x.round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

/// `n` stations sending messages back to back for `secs`, at random offsets and frequencies.
fn busy(n: usize, secs: usize, seed: u64) -> Vec<i16> {
    let mut rng = Lcg(seed);
    let mut audio = noise(secs, &mut rng);
    let texts = [
        "CQ K1ABC CQ",
        "K1ABC DE JA1XYZ TNX BOB NAME TARO QTH TOKYO",
        "JA1XYZ DE K1ABC UR 599 NAME BOB QTH BOSTON MA K",
        "CQ CQ CQ DE W9XYZ W9XYZ K",
        "73 TU",
    ];
    for s in 0..n {
        // one station in three on channel 0's ±50 Hz, the rest spread over the whole band
        let f0 = if s % 3 == 0 {
            1450.0 + 100.0 * rng.uniform() as f32
        } else {
            400.0 + 2200.0 * rng.uniform() as f32
        };
        let mut t = 8.0 * rng.uniform() as f32;
        while t < secs as f32 - 25.0 {
            let text = texts[(rng.uniform() * texts.len() as f64) as usize % texts.len()];
            let snr = -12.0 + 10.0 * rng.uniform();
            add_station(&mut audio, text, f0, t, snr);
            t += 12.0 + 20.0 * rng.uniform() as f32;
        }
    }
    to_i16(&audio)
}

fn report(name: &str, audio: &[i16], rx: &Receiver) {
    let sequential = std::env::var_os("MFSK_JTTY_SEQUENTIAL").is_some();
    let carry = std::env::var_os("MFSK_JTTY_CARRY").is_some();
    rx.reset_stats();
    let t = std::time::Instant::now();
    let updates = rx.scan_messages(
        audio,
        &Params {
            sequential,
            carry,
            ..Params::default()
        },
    );
    let wall = t.elapsed().as_secs_f64();
    let s: Snapshot = rx.stats();
    let windows = (audio.len() - NCHUNK) / STEP + 1;
    let w = s.count(C::Windows).max(1) as f64;
    let per = |c: C| s.count(c) as f64 / w;
    let done = updates.iter().filter(|u| u.complete).count();
    println!(
        "\n=== {name}: {windows} windows ({:.0} s of audio), {done} messages complete, {:.0} ms per window of {:.0} ms budget ===",
        audio.len() as f64 / 12_000.0,
        1000.0 * wall / w,
        472.0
    );
    println!(
        "per window: analytic {:.2} (incl. retro {:.2}), surface builds {:.2}, retro windows {:.2}, extra rounds {:.2}, subtractions {:.2}",
        per(C::Analytic),
        s.count(C::RetroWindows) as f64 / w,
        per(C::SurfaceBuilds),
        s.count(C::RetroWindows) as f64 / w,
        per(C::ExtraRounds),
        per(C::Subtractions)
    );
    println!(
        "            picks ch0 {:.2} + other {:.2}; peakups {:.2}; shifts {:.2}; gate pass {:.3} fail {:.2}; correlations {:.3}; ladder calls {:.3} accepts {:.3}; sticky retries {:.3}",
        per(C::PicksCh0),
        per(C::PicksOther),
        per(C::Peakups),
        per(C::Shifts),
        per(C::GatePass),
        per(C::GateFail),
        per(C::Correlations),
        per(C::LadderCalls),
        per(C::LadderAccepts),
        per(C::StickyRetries)
    );
    println!(
        "            retry rounds: {:.3} candidates re-decoded per window, {:.3} of them ladder calls (of {:.3} ladder calls in all)",
        per(C::RetryCandidates),
        per(C::RetryLadderCalls),
        per(C::LadderCalls)
    );
    // Who are the gated candidates the ladder rejects? Distance to the nearest accepted one.
    let gated = rx.gated_candidates();
    let good: Vec<_> = gated.iter().filter(|g| g.accepted).collect();
    let bad: Vec<_> = gated.iter().filter(|g| !g.accepted).collect();
    if !bad.is_empty() && !good.is_empty() {
        let near = |b: &&mfsk_core::jtty::stats::GatedCandidate, df: f32, dt: f32| {
            good.iter()
                .any(|g| (g.f1_hz - b.f1_hz).abs() <= df && (g.tsync_s - b.tsync_s).abs() <= dt)
        };
        let n = bad.len() as f64;
        println!(
            "            rejected gated candidates: {}; within 40 Hz and 0.1 s of an accepted frame: {:.0} %, within 120 Hz and 2 s: {:.0} %; median nsync {} / snr {:.1} dB (accepted: {} / {:.1} dB)",
            bad.len(),
            100.0 * bad.iter().filter(|b| near(b, 40.0, 0.1)).count() as f64 / n,
            100.0 * bad.iter().filter(|b| near(b, 120.0, 2.0)).count() as f64 / n,
            {
                let mut v: Vec<_> = bad.iter().map(|b| b.nsync).collect();
                v.sort();
                v[v.len() / 2]
            },
            {
                let mut v: Vec<_> = bad.iter().map(|b| b.snr_db).collect();
                v.sort_by(|a, b| a.total_cmp(b));
                v[v.len() / 2]
            },
            {
                let mut v: Vec<_> = good.iter().map(|b| b.nsync).collect();
                v.sort();
                v[v.len() / 2]
            },
            {
                let mut v: Vec<_> = good.iter().map(|b| b.snr_db).collect();
                v.sort_by(|a, b| a.total_cmp(b));
                v[v.len() / 2]
            },
        );
    } else if !bad.is_empty() {
        let mut v: Vec<_> = bad.iter().map(|b| (b.nsync, b.snr_db)).collect();
        v.sort_by(|a, b| a.0.cmp(&b.0));
        println!(
            "            rejected gated candidates: {} (nothing accepted); median nsync {} snr {:.1} dB",
            bad.len(),
            v[v.len() / 2].0,
            v[v.len() / 2].1
        );
    }
    println!(
        "rungs per window: L1 {:.3}  L2 {:.3}  L4 {:.3}  half {:.3}",
        s.rungs[0] as f64 / w,
        s.rungs[1] as f64 / w,
        s.rungs[2] as f64 / w,
        s.rungs[3] as f64 / w
    );
    let stages = [
        ("analytic", S::Analytic),
        ("surface", S::Surface),
        ("pick", S::Pick),
        ("peakup", S::Peakup),
        ("shift", S::Shift),
        ("gate", S::Gate),
        ("correlate", S::Correlate),
        ("ladder", S::Ladder),
        ("subtract", S::Subtract),
    ];
    let total: f64 = stages.iter().map(|(_, st)| s.seconds(*st)).sum();
    print!("host ms per window by stage:");
    for (n, st) in stages {
        print!("  {n} {:.2}", 1000.0 * s.seconds(st) / w);
    }
    println!(
        "   [sum {:.2}, other {:.2}]",
        1000.0 * total / w,
        1000.0 * (wall - total) / w
    );
}

#[test]
#[ignore = "prints; run with --nocapture (see the module doc)"]
fn profile_the_receive_path() {
    let rx = Receiver::new();
    let mut rng = Lcg(1);
    report("noise only", &to_i16(&noise(60, &mut rng)), &rx);
    if let Some(p) = common::corpus::golden_path("jtty/260807_134110.wav") {
        report("upstream sample recording", &common::load_wav_i16(p), &rx);
    }
    // real audio with no JTTY in it: what a receiver on an occupied band sees as candidates
    let dir = common::corpus::golden_path("").unwrap_or_default();
    fn walk(d: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
        if let Ok(rd) = std::fs::read_dir(d) {
            for e in rd.flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "wav")
                    && !p.to_string_lossy().contains("jtty")
                {
                    out.push(p);
                }
            }
        }
    }
    let mut files = Vec::new();
    walk(&dir, &mut files);
    files.sort();
    let mut real: Vec<Vec<i16>> = Vec::new();
    for f in &files {
        if let Ok(bytes) = std::fs::read(f) {
            if bytes.len() < 44 {
                continue;
            }
            // 12 kHz mono 16-bit only
            let rate = u32::from_le_bytes(bytes[24..28].try_into().unwrap());
            let ch = u16::from_le_bytes(bytes[22..24].try_into().unwrap());
            if rate == 12_000 && ch == 1 {
                real.push(common::load_wav_i16(f));
            }
        }
    }
    let joined: Vec<i16> = real.concat();
    if joined.len() > NCHUNK {
        report(
            &format!(
                "real audio, {} vendored non-JTTY recordings joined",
                real.len()
            ),
            &joined,
            &rx,
        );
    }
    for n in [1, 3, 6] {
        report(
            &format!("busy band, {n} stations"),
            &busy(n, 90, 100 + n as u64),
            &rx,
        );
    }
    let _ = tx::samples_for_frames(1);
}
