// SPDX-License-Identifier: GPL-3.0-or-later
//! The JTTY frame decoder on WSJT-X's own recordings (tier B), phase P2.
//!
//! - the `sjtty` vectors (AWGN at −8 dB SNR in 2500 Hz): every message must come
//!   out, and only it;
//! - upstream's sample recording `260807_134110.wav`: the message must come out
//!   and — as the target, `max_extra: 0` — nothing else. `rjtty` also decodes a
//!   phantom on this file (`4>-P'`, channel 2, 27 symbol errors in 59); see the
//!   fixtures' README.
//!
//! Message assembly is phase P3, so the tests join frames themselves: those on
//! one frequency, one frame period apart.

#![cfg(all(feature = "jtty", any(feature = "fft-rustfft", feature = "fft-extern")))]

#[allow(dead_code)]
mod common;

use std::path::{Path, PathBuf};

use mfsk_core::jtty::rx::{FrameDecode, Params, Receiver, Stream};
use mfsk_core::jtty::source::{Atom, render_message};

fn load(rel: &str) -> Option<Vec<i16>> {
    common::corpus::golden_path(rel).map(common::load_wav_i16)
}

fn scan(audio: &[i16]) -> Vec<FrameDecode> {
    Receiver::new().scan(audio, &Params::default())
}

/// Frames of one transmission: consecutive in time (one frame period apart, with
/// slack for a missed frame) and within 12 Hz of each other. Returns
/// `(text, first frame)` per message, in time order.
fn messages(frames: &[FrameDecode]) -> Vec<(String, FrameDecode)> {
    let mut sorted: Vec<&FrameDecode> = frames.iter().collect();
    sorted.sort_by(|a, b| a.tsync_s.total_cmp(&b.tsync_s));
    let mut groups: Vec<Vec<&FrameDecode>> = Vec::new();
    for f in sorted {
        let fits = |g: &Vec<&FrameDecode>| {
            let last = g.last().unwrap();
            !last.eom
                && (f.f1_hz - last.f1_hz).abs() < 12.0
                && (f.tsync_s - last.tsync_s - 1.888).abs() < 0.1
        };
        match groups.iter_mut().find(|g| fits(g)) {
            Some(g) => g.push(f),
            None => groups.push(vec![f]),
        }
    }
    groups
        .into_iter()
        .map(|g| {
            let atoms: Vec<Atom> = g.iter().map(|f| f.atom.clone()).collect();
            (render_message(&atoms), g[0].clone())
        })
        .collect()
}

/// `rjtty`'s ndebug=1 lines: `(f1, tsync, text-so-far)` per decode.
fn expected_lines(rel: &str) -> Vec<(f32, f32, String)> {
    let path = common::corpus::golden_path(rel).unwrap();
    std::fs::read_to_string(path)
        .unwrap()
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            (
                f[8].parse().unwrap(),
                f[10].parse().unwrap(),
                f[14..].join(" "),
            )
        })
        .collect()
}

#[test]
fn sjtty_vectors_decode_exactly_and_only_them() {
    let Some(manifest) = common::corpus::golden_path("jtty/sim/MANIFEST.tsv") else {
        return;
    };
    let text = std::fs::read_to_string(manifest).unwrap();
    let rx = Receiver::new();
    let mut checked = 0;
    for row in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = row.split('\t').collect();
        if f[8] == "-" {
            continue; // tones only
        }
        let audio = load(&format!("jtty/sim/{}", f[8])).unwrap();
        let frames = rx.scan(&audio, &Params::default());
        let msgs = messages(&frames);
        assert_eq!(
            msgs.len(),
            1,
            "{}: exactly one message, got {:?}",
            f[0],
            msgs.iter().map(|m| &m.0).collect::<Vec<_>>()
        );
        assert_eq!(msgs[0].0, f[11], "{}: text (rjtty says {:?})", f[0], f[11]);
        let (f0, dt): (f32, f32) = (f[3].parse().unwrap(), f[4].parse().unwrap());
        let first = &msgs[0].1;
        assert!(
            (first.f1_hz - f0).abs() < 2.0,
            "{}: f1 {}",
            f[0],
            first.f1_hz
        );
        assert!(
            (first.tsync_s - dt).abs() < 0.01,
            "{}: t {}",
            f[0],
            first.tsync_s
        );
        assert!(
            frames.iter().all(|x| x.channel == 0),
            "{}: only channel 0",
            f[0]
        );
        checked += 1;
    }
    assert_eq!(checked, 7);
}

/// What `rjtty` prints for a frame it only just decodes is exactly what this
/// crate does: same frequency, same time, same text.
#[test]
fn upstream_sample_recording_matches_rjtty_frame_for_frame() {
    let Some(audio) = load("jtty/260807_134110.wav") else {
        return;
    };
    let frames = scan(&audio);
    let want = expected_lines("jtty/260807_134110.expected.txt");

    let ch0: Vec<&FrameDecode> = frames.iter().filter(|f| f.channel == 0).collect();
    // rjtty's first ten lines are the ten frames of the message, cumulatively
    assert_eq!(ch0.len(), 10);
    for (k, (f, (wf, wt, wtext))) in ch0.iter().zip(&want).enumerate() {
        assert!(
            (f.f1_hz - wf).abs() < 0.05,
            "frame {k}: f1 {} vs {wf}",
            f.f1_hz
        );
        assert!(
            (f.tsync_s - wt).abs() < 0.001,
            "frame {k}: t {} vs {wt}",
            f.tsync_s
        );
        let atoms: Vec<Atom> = ch0[..=k].iter().map(|f| f.atom.clone()).collect();
        assert_eq!(&render_message(&atoms), wtext, "frame {k}: cumulative text");
    }
    let msgs = messages(&frames);
    let message = msgs.iter().find(|m| m.1.channel == 0).unwrap();
    assert_eq!(message.0, "RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!");
    assert!(ch0.last().unwrap().eom && ch0[..9].iter().all(|f| !f.eom));
}

/// **Documented debt, not a target.** `rjtty` decodes one more thing on this
/// file — `4>-P'`, channel 2, 9 of 13 sync tones, 27 symbol errors in 59 — and
/// this port, being faithful to the same algorithm, decodes it too. `max_extra`
/// is therefore 1, and it must be *that* frame: any other extra fails.
#[test]
fn upstream_sample_recording_precision() {
    let Some(audio) = load("jtty/260807_134110.wav") else {
        return;
    };
    let frames = scan(&audio);
    let extras: Vec<&FrameDecode> = frames.iter().filter(|f| f.channel != 0).collect();
    assert_eq!(
        extras.len(),
        1,
        "extras: {:?}",
        extras.iter().map(|f| f.atom.render()).collect::<Vec<_>>()
    );
    let x = extras[0];
    assert_eq!((x.channel, x.atom.render().as_str()), (2, "4>-P'"));
    assert!((x.f1_hz - 1695.6).abs() < 0.5 && (x.tsync_s - 24.442).abs() < 0.01);
    assert_eq!((x.nsync, x.nsymerrs), (9, 27));
}

/// Real audio that has no JTTY in it: whatever decodes is a false decode. Upstream
/// `rjtty` gives exactly one on this set (a WSPR recording, `KD7NCT`, 8 of 13 sync
/// tones, 21 symbol errors); so does this port, and nothing else.
#[test]
fn jtty_free_real_audio_gives_only_the_known_false_decode() {
    let assets = common::corpus::golden_dir().join("..");
    let mut files: Vec<PathBuf> = Vec::new();
    let mut stack = vec![common::corpus::golden_dir()];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                if p.file_name().unwrap() != "jtty" {
                    stack.push(p);
                }
            } else if p.extension().is_some_and(|x| x == "wav") {
                files.push(p);
            }
        }
    }
    for e in std::fs::read_dir(&assets).unwrap() {
        let p = e.unwrap().path();
        if p.extension().is_some_and(|x| x == "wav") {
            files.push(p);
        }
    }
    files.sort();
    assert!(
        files.len() >= 20,
        "expected the vendored recordings, found {}",
        files.len()
    );
    let rx = Receiver::new();
    let mut seconds = 0.0;
    let mut found: Vec<(String, String)> = Vec::new();
    for f in &files {
        let audio = common::load_wav_i16(f);
        seconds += audio.len() as f32 / 12_000.0;
        for fr in rx.scan(&audio, &Params::default()) {
            found.push((rel(f, &assets), fr.atom.render()));
        }
    }
    eprintln!(
        "{} recordings, {seconds:.0} s of JTTY-free audio; decodes: {found:?}",
        files.len()
    );
    assert_eq!(
        found,
        [(
            "golden/wspr/150426_0918.wav".to_string(),
            "KD7NCT".to_string()
        )]
    );
}

fn rel(p: &Path, base: &Path) -> String {
    let (p, base) = (p.canonicalize().unwrap(), base.canonicalize().unwrap());
    p.strip_prefix(base)
        .unwrap()
        .to_string_lossy()
        .replace('\\', "/")
}

/// The result must not depend on the thread count.
#[cfg(feature = "parallel")]
#[test]
fn scan_is_identical_for_any_thread_count() {
    let Some(audio) = load("jtty/260807_134110.wav") else {
        return;
    };
    let Some(mix) = load("jtty/sim/mix_four_stations.wav") else {
        return;
    };
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| {
                let rx = Receiver::new();
                let p = Params::default();
                (
                    rx.scan(&audio, &p),
                    rx.scan_messages(&audio, &p),
                    rx.scan_messages(&mix, &p),
                )
            })
    };
    let key = |r: &(
        Vec<FrameDecode>,
        Vec<mfsk_core::jtty::assemble::MessageUpdate>,
        Vec<mfsk_core::jtty::assemble::MessageUpdate>,
    )| {
        (
            r.0.iter()
                .map(|f| {
                    (
                        f.channel,
                        f.f1_hz.to_bits(),
                        f.tsync_s.to_bits(),
                        f.payload.to_vec(),
                        f.rank,
                    )
                })
                .collect::<Vec<_>>(),
            r.1.iter()
                .map(|u| (u.id, u.f1_hz.to_bits(), u.text.clone(), u.complete))
                .collect::<Vec<_>>(),
            r.2.iter()
                .map(|u| (u.id, u.f1_hz.to_bits(), u.text.clone(), u.complete))
                .collect::<Vec<_>>(),
        )
    };
    let one = key(&run(1));
    assert_eq!(one.0.len(), 11);
    assert!(one.2.iter().filter(|u| u.3).count() >= 4);
    for t in [2, 5, 16] {
        assert_eq!(key(&run(t)), one, "{t} threads");
    }
}

/// Tier C-style, skipped when the corpora have not been generated
/// (`scripts/gen_jtty_sweep_wavs.sh`): recall per SNR cell must match what
/// upstream's `rjtty` scored on the same files (`UPSTREAM_RECALL.tsv`), to within
/// one file per cell, with no unexpected decodes.
#[test]
fn sweep_recall_matches_upstream_cell_by_cell() {
    let Some(dir) = common::corpus::optional_corpus("jtty_sweep") else {
        return;
    };
    let tsv = dir.join("UPSTREAM_RECALL.tsv");
    let Ok(text) = std::fs::read_to_string(&tsv) else {
        return;
    };
    let rx = Receiver::new();
    let mut compared = 0;
    for row in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = row.split('\t').collect();
        let (chan, snr, trials, up_ok, up_extra): (&str, i32, usize, usize, usize) = (
            f[0],
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            f[4].parse().unwrap(),
        );
        let tag = if snr < 0 {
            format!("m{:02}", -snr)
        } else {
            format!("p{snr:02}")
        };
        let (mut ok, mut extra) = (0, 0);
        for t in 1..=trials {
            let path = dir.join(format!("jtty_{chan}_{tag}_{t:02}.wav"));
            if !path.exists() {
                return; // partial corpus
            }
            let frames = rx.scan(&common::load_wav_i16(path), &Params::default());
            let good = frames.iter().any(|f| f.atom.render() == "CQ K1ABC CQ");
            ok += usize::from(good);
            extra += frames
                .iter()
                .filter(|f| f.atom.render() != "CQ K1ABC CQ")
                .count();
        }
        assert!(
            ok.abs_diff(up_ok) <= 1,
            "{chan} {snr} dB: {ok}/{trials} vs upstream {up_ok}"
        );
        assert!(
            extra <= up_extra,
            "{chan} {snr} dB: {extra} unexpected decodes vs upstream {up_extra}"
        );
        compared += 1;
    }
    assert!(compared >= 18);
}

/// Tier C-style (`scripts/gen_jtty_noise_wavs.sh`): Gaussian noise alone must
/// decode nothing. Upstream's `rjtty` decodes nothing in the same 30 minutes.
#[test]
fn gaussian_noise_decodes_nothing() {
    let Some(dir) = common::corpus::optional_corpus("jtty_noise") else {
        return;
    };
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "wav"))
        .collect();
    files.sort();
    if files.len() < 10 {
        return;
    }
    let rx = Receiver::new();
    let mut found = Vec::new();
    for f in &files {
        for fr in rx.scan(&common::load_wav_i16(f), &Params::default()) {
            found.push((
                f.file_name().unwrap().to_string_lossy().to_string(),
                fr.atom.render(),
            ));
        }
    }
    assert!(
        found.is_empty(),
        "{} false decodes in {} files: {found:?}",
        found.len(),
        files.len()
    );
}

/// Diagnostic: scan every WAV in `$JTTY_DIAG_DIR` and print one line per frame.
#[test]
#[ignore]
fn diag_dir() {
    let dir = std::env::var("JTTY_DIAG_DIR").expect("JTTY_DIAG_DIR");
    let rx = Receiver::new();
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "wav"))
        .collect();
    files.sort();
    for f in files {
        let audio = common::load_wav_i16(&f);
        for fr in rx.scan(&audio, &Params::default()) {
            eprintln!(
                "FRAME {} ch{} f={:.1} t={:.3} snr={:.1} sync={} errs={} rung={} rank={} pool={} {:?}",
                f.file_name().unwrap().to_string_lossy(),
                fr.channel,
                fr.f1_hz,
                fr.tsync_s,
                fr.snr_db,
                fr.nsync,
                fr.nsymerrs,
                fr.rung,
                fr.rank,
                fr.pool,
                fr.atom.render()
            );
        }
    }
}

// ---- drift study (D6): not a test, a measurement -----------------------------

/// Deterministic Gaussian noise (xorshift + Box–Muller).
struct Noise(u64);
impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        (-2.0 * self.uniform().ln()).sqrt() * (std::f64::consts::TAU * self.uniform()).cos()
    }
}

fn write_wav(path: &Path, samples: &[i16]) {
    let n = samples.len() as u32 * 2;
    let mut b = Vec::with_capacity(44 + n as usize);
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + n).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes()); // PCM
    b.extend_from_slice(&1u16.to_le_bytes()); // mono
    b.extend_from_slice(&12_000u32.to_le_bytes());
    b.extend_from_slice(&24_000u32.to_le_bytes());
    b.extend_from_slice(&2u16.to_le_bytes());
    b.extend_from_slice(&16u16.to_le_bytes());
    b.extend_from_slice(b"data");
    b.extend_from_slice(&n.to_le_bytes());
    samples
        .iter()
        .for_each(|s| b.extend_from_slice(&s.to_le_bytes()));
    std::fs::write(path, b).unwrap();
}

/// D6: how does the decoder cope with a frequency that drifts across the frame,
/// as a satellite's Doppler shift does? Writes WAVs of `CQ K1ABC CQ` (one frame,
/// centred on 1500 Hz, `sjtty`'s 2500 Hz SNR convention, deterministic noise)
/// with a linear drift into `$JTTY_DRIFT_OUT` and prints this crate's recall;
/// `scripts/jtty_drift_study.sh` runs upstream's `rjtty` over the same files.
#[test]
#[ignore]
fn drift_study() {
    use mfsk_core::jtty::source::CallAction;
    use mfsk_core::jtty::tx;
    let out = PathBuf::from(std::env::var("JTTY_DRIFT_OUT").expect("JTTY_DRIFT_OUT"));
    std::fs::create_dir_all(&out).unwrap();
    let rx = Receiver::new();
    let tones = tx::tones(&[Atom::call(CallAction::Cq, "K1ABC")]).unwrap();
    let trials = 20;
    for snr in [-8i32, -12] {
        for drift in [
            0.0f32, 2.0, 5.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 40.0, 80.0,
        ] {
            let mut ok = 0;
            for t in 1..=trials {
                // frequency at the middle of the frame is 1500 Hz
                let f_start = 1500.0 - drift * 0.944;
                let wave = tx::synth_drifting_f32(&tones, f_start, 1.0, drift);
                let sig = (2.0 * 2500.0 / 6000.0f64).sqrt() * 10f64.powf(f64::from(snr) / 20.0);
                let mut noise = Noise(
                    0x9E37_79B9_7F4A_7C15 ^ ((drift as u64) << 32) ^ ((-snr) as u64) << 16 ^ t,
                );
                let (n, offset) = (60_000usize, 3_600usize);
                let samples: Vec<i16> = (0..n)
                    .map(|i| {
                        let s = if i >= offset && i - offset < wave.len() {
                            sig * f64::from(wave[i - offset])
                        } else {
                            0.0
                        };
                        (100.0 * (s + noise.gauss()))
                            .round()
                            .clamp(-32767.0, 32767.0) as i16
                    })
                    .collect();
                write_wav(
                    &out.join(format!(
                        "drift_d{:02}_s{:02}_{t:02}.wav",
                        drift as i32, -snr
                    )),
                    &samples,
                );
                let frames = rx.scan(&samples, &Params::default());
                ok += usize::from(frames.iter().any(|f| f.atom.render() == "CQ K1ABC CQ"));
            }
            eprintln!("DRIFT snr={snr} drift={drift:.0} ours={ok}/{trials}");
        }
    }
}

/// Diagnostic: how long a scan takes on 1 thread and on the default pool.
#[cfg(feature = "parallel")]
#[test]
#[ignore]
fn speed() {
    let Some(audio) = load("jtty/260807_134110.wav") else {
        return;
    };
    let seconds = audio.len() as f32 / 12_000.0;
    let rx = Receiver::new();
    let time = |threads: Option<usize>| {
        let run = || {
            let t = std::time::Instant::now();
            for _ in 0..5 {
                std::hint::black_box(rx.scan(&audio, &Params::default()));
            }
            t.elapsed().as_secs_f32() / 5.0
        };
        match threads {
            Some(n) => rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build()
                .unwrap()
                .install(run),
            None => run(),
        }
    };
    let windows = (audio.len() - mfsk_core::jtty::rx::NCHUNK) / mfsk_core::jtty::rx::STEP + 1;
    for (label, t) in [
        ("1 thread", time(Some(1))),
        ("2 threads", time(Some(2))),
        ("8 threads", time(Some(8))),
        ("default pool", time(None)),
    ] {
        eprintln!(
            "SPEED {label}: {t:.3} s for {seconds:.1} s of audio ({windows} windows): {:.1} ms/window, {:.0}x real time",
            1000.0 * t / windows as f32,
            seconds / t
        );
    }
}

/// Several stations in one recording (`sim/mix_*.wav`, one noise, components at
/// calibrated SNRs — see `scripts/gen_jtty_vectors.sh`): the messages this crate
/// assembles are exactly the ones `rjtty` shows, no more and no fewer. These
/// exercise the subtraction, the three channels, the retro re-sweep and the
/// assembly: `three_channels` and `four_stations` put a station in each channel,
/// `two_close` a weak one 60 Hz from a strong one.
#[test]
fn several_stations_in_one_recording_match_rjtty() {
    let Some(path) = common::corpus::golden_path("jtty/sim/MIXES.tsv") else {
        return;
    };
    let text = std::fs::read_to_string(path).unwrap();
    let rx = Receiver::new();
    let mut checked = 0;
    for row in text.lines().filter(|l| !l.starts_with('#')) {
        let f: Vec<&str> = row.split('\t').collect();
        let audio = load(&format!("jtty/sim/mix_{}.wav", f[0])).unwrap();
        // the last text of every message, complete or not, distinct
        let mut last: std::collections::BTreeMap<u64, String> = Default::default();
        for u in rx.scan_messages(&audio, &Params::default()) {
            last.insert(u.id, u.text);
        }
        let mut ours: Vec<String> = last.into_values().collect();
        let mut theirs: Vec<String> = f[2]
            .split('|')
            .filter(|t| !t.is_empty())
            .map(String::from)
            .collect();
        ours.sort();
        ours.dedup();
        theirs.sort();
        assert_eq!(ours, theirs, "{}: {}", f[0], f[1]);
        checked += 1;
    }
    assert_eq!(checked, 7);
}

/// Diagnostic for `scripts/jtty_multi_study.sh`: the completed messages of every
/// WAV in `$JTTY_DIAG_DIR`, one line each: `MSGS <file> <text>|<text>…`.
#[test]
#[ignore]
fn diag_messages() {
    let dir = std::env::var("JTTY_DIAG_DIR").expect("JTTY_DIAG_DIR");
    let rx = Receiver::new();
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "wav"))
        .collect();
    files.sort();
    for f in files {
        let audio = common::load_wav_i16(&f);
        let done: Vec<String> = rx
            .scan_messages(&audio, &Params::default())
            .into_iter()
            .filter(|u| u.complete)
            .map(|u| u.text)
            .collect();
        eprintln!(
            "MSGS {} {}",
            f.file_name().unwrap().to_string_lossy(),
            done.join("|")
        );
    }
}

/// A scene built here: a strong station and, 20 dB down and 20 Hz away, a weak
/// one that starts a moment later, in a little noise. Nothing in the weak one's
/// sync search can see past the strong one until it is taken off — so with
/// `subtract` off it is lost, and with it on both are decoded.
#[test]
fn a_weak_station_under_a_strong_one_needs_the_subtraction() {
    use mfsk_core::jtty::source::CallAction;
    use mfsk_core::jtty::tx;
    let strong = tx::synth_f32(
        &tx::tones(&[Atom::call(CallAction::Cq, "K1ABC")]).unwrap(),
        1500.0,
        8000.0,
    );
    let weak = tx::synth_f32(
        &tx::tones(&[Atom::call(CallAction::Cq, "W9XYZ")]).unwrap(),
        1520.0,
        800.0,
    );
    let mut noise = Noise(0x1234_5678_9ABC_DEF1);
    let n = 40_000usize;
    let audio: Vec<i16> = (0..n)
        .map(|i| {
            let s = strong.get(i.wrapping_sub(3600)).copied().unwrap_or(0.0);
            let w = weak.get(i.wrapping_sub(4400)).copied().unwrap_or(0.0);
            (s + w + 30.0 * noise.gauss() as f32)
                .round()
                .clamp(-32767.0, 32767.0) as i16
        })
        .collect();
    if let Ok(d) = std::env::var("JTTY_PROBE_OUT") {
        write_wav(&PathBuf::from(d).join("weak_scene.wav"), &audio);
    }
    let rx = Receiver::new();
    let texts = |subtract: bool| -> Vec<String> {
        let p = Params {
            subtract,
            ..Params::default()
        };
        let mut t: Vec<String> = rx
            .scan(&audio, &p)
            .iter()
            .map(|f| f.atom.render())
            .collect();
        t.sort();
        t
    };
    assert_eq!(texts(true), ["CQ K1ABC CQ", "CQ W9XYZ CQ"]);
    assert_eq!(
        texts(false),
        ["CQ K1ABC CQ"],
        "without subtraction only the strong one"
    );
}

/// Fed a piece at a time, in any sizes, a [`Stream`] reports exactly the updates a
/// scan of the whole recording does — same messages, same order, same values.
#[test]
fn streaming_is_scanning_whatever_the_chunk_size() {
    let Some(sample) = load("jtty/260807_134110.wav") else {
        return;
    };
    let Some(four) = load("jtty/sim/mix_four_stations.wav") else {
        return;
    };
    let Some(three) = load("jtty/sim/mix_three_channels.wav") else {
        return;
    };
    let rx = std::sync::Arc::new(Receiver::new());
    for (name, audio) in [
        ("sample", &sample),
        ("four_stations", &four),
        ("three_channels", &three),
    ] {
        let want = rx.scan_messages(audio, &Params::default());
        assert!(!want.is_empty(), "{name}");
        for chunk in [1usize, 333, 4096, 12_000, 28_320, 100_000, audio.len()] {
            let mut got = Vec::new();
            let mut stream = Stream::new(rx.clone(), Params::default());
            let mut most = 0;
            for piece in audio.chunks(chunk) {
                stream.push(piece, &mut |u| got.push(u));
                most = most.max(stream.buffered_samples());
            }
            assert_eq!(got, want, "{name}, chunks of {chunk}");
            assert_eq!(stream.samples_seen(), audio.len());
            // only what a re-sweep can reach is kept
            assert!(
                most <= 28_320 + 3 * 5_664 + chunk,
                "{name}, chunks of {chunk}: {most} buffered"
            );
        }
    }
}

#[test]
fn a_stream_that_ends_reports_what_was_left_unfinished() {
    let Some(sample) = load("jtty/260807_134110.wav") else {
        return;
    };
    let rx = std::sync::Arc::new(Receiver::new());
    let mut stream = Stream::new(rx, Params::default());
    let mut got = Vec::new();
    // 12 s of a 22 s message: several frames in, no end of message
    stream.push(&sample[..12 * 12_000], &mut |u| got.push(u));
    assert!(!got.is_empty() && got.iter().all(|u| !u.complete));
    let before = got.len();
    stream.finish(&mut |u| got.push(u));
    assert_eq!(
        got.len(),
        before + 1,
        "one incomplete report for the one open message"
    );
    let last = got.last().unwrap();
    assert!(
        !last.complete && last.text.starts_with("RAN ALL NIGHT"),
        "{:?}",
        last.text
    );
    // and reset starts over
    stream.reset();
    assert_eq!(stream.samples_seen(), 0);
}

/// A message played out by the streaming synthesiser in `f32`, the way a
/// transmitter feeds a sound device, comes back through the receiver.
#[test]
fn a_message_synthesised_in_f32_pieces_decodes() {
    use mfsk_core::jtty::pack::{self, ExchangeProfile};
    use mfsk_core::jtty::tx::Synth;
    let tones = pack::tones("CQ K1ABC CQ", ExchangeProfile::Unknown)
        .unwrap()
        .unwrap();
    let mut synth = Synth::<f32>::new(&tones, 1500.0, 3000.0);
    let mut audio = vec![0i16; 12_000];
    let mut piece = [0f32; 480];
    while !synth.is_finished() {
        let n = synth.fill(&mut piece);
        audio.extend(piece[..n].iter().map(|&x| x as i16));
    }
    audio.extend(std::iter::repeat_n(0i16, 6 * 12_000));
    let rx = Receiver::new();
    let heard = rx.scan_messages(&audio, &Params::default());
    assert!(
        heard.iter().any(|u| u.complete && u.text == "CQ K1ABC CQ"),
        "{heard:?}"
    );
}

/// The receiver with the trellis metrics in `f32` (`Receiver::with_f32_metrics`, #499 E1b) reads
/// the same recordings as the `f64` one, frame for frame: the sample recording, the simulated
/// vectors and the multi-station mixtures.
#[test]
fn f32_metric_receiver_reads_the_same_frames() {
    let Some(dir) = common::corpus::golden_path("jtty/sim") else {
        return;
    };
    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "wav"))
        .collect();
    files.sort();
    if let Some(s) = common::corpus::golden_path("jtty/260807_134110.wav") {
        files.push(s);
    }
    assert!(files.len() >= 10, "{} files", files.len());
    let (a, b) = (Receiver::new(), Receiver::new().with_f32_metrics());
    for f in &files {
        let audio = common::load_wav_i16(f);
        let key = |v: Vec<FrameDecode>| -> Vec<(String, i64, i64, bool)> {
            v.into_iter()
                .map(|d| {
                    (
                        d.atom.render(),
                        (d.tsync_s * 1000.0).round() as i64,
                        (d.f1_hz * 10.0).round() as i64,
                        d.eom,
                    )
                })
                .collect()
        };
        let (fa, fb) = (
            key(a.scan(&audio, &Params::default())),
            key(b.scan(&audio, &Params::default())),
        );
        assert_eq!(fa, fb, "{}", f.display());
    }
}
