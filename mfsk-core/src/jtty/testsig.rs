// SPDX-License-Identifier: GPL-3.0-only
//! Test recordings made where they are used (#499): JTTY stations in Gaussian noise, with a
//! carrier offset, a linear drift and Rayleigh fading, from a seed. The CoreS3 bench makes the
//! same audio as a host test, so the board's decisions can be compared with the host's file for
//! file without carrying recordings in flash.
//!
//! Levels follow `sjtty`: `snr_db` is the signal's power against the noise's in 2 500 Hz. Fading
//! is a sum of 16 sinusoids at Doppler frequencies `fd·cos α` with random phases (a Clarke
//! model), unit mean power, applied to the analytic signal; `fd = 0` is no fading.

use alloc::vec::Vec;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::pack::{self, ExchangeProfile};
use super::tx::Synth;

/// One transmission.
#[derive(Clone, Debug)]
pub struct Station<'a> {
    /// The message, packed with `ExchangeProfile::Unknown`.
    pub text: &'a str,
    /// Frequency of tone 0, Hz.
    pub f0_hz: f32,
    /// Start, seconds from the recording's start.
    pub start_s: f32,
    /// Signal-to-noise ratio in 2 500 Hz, dB.
    pub snr_db: f32,
    /// Carrier drift, Hz a second (from the transmission's start).
    pub drift_hz_s: f32,
    /// Doppler spread of Rayleigh fading, Hz; 0 for none.
    pub fading_hz: f32,
}

/// A small deterministic generator (the same numbers on every target).
pub struct Rng(u64);

impl Rng {
    /// Seeded.
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0x9E37_79B9_7F4A_7C15)
    }
    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }
    /// Standard normal (Box–Muller).
    pub fn gauss(&mut self) -> f32 {
        let (a, b) = (self.uniform().max(1e-7), self.uniform());
        (-2.0 * a.ln()).sqrt() * (core::f32::consts::TAU * b).cos()
    }
}

/// Standard deviation of the noise, in `i16` units.
pub const NOISE_SIGMA: f32 = 1000.0;
/// Audio sample rate.
const FS: f32 = 12_000.0;

/// `secs` seconds of 12 kHz audio: the stations over Gaussian noise, all from `seed`. `None` if
/// a message cannot be packed.
pub fn render(stations: &[Station<'_>], secs: f32, seed: u64) -> Option<Vec<i16>> {
    let n = (secs * FS) as usize;
    let mut rng = Rng::new(seed);
    let mut audio: Vec<f32> = (0..n).map(|_| NOISE_SIGMA * rng.gauss()).collect();
    let mut buf = alloc::vec![Complex32::new(0.0, 0.0); 4096];
    for st in stations {
        let tones = pack::tones(st.text, ExchangeProfile::Unknown).ok()??;
        // peak amplitude of a real sinusoid of that power against the noise in 2 500 Hz
        let amp = NOISE_SIGMA * (2.0 * 2500.0 / (FS / 2.0) * 10f32.powf(st.snr_db / 10.0)).sqrt();
        let mut synth = Synth::<f32>::new(&tones, st.f0_hz, amp);
        // fading paths: Doppler frequency and phase
        let paths: Vec<(f32, f32)> = (0..16)
            .map(|_| {
                let alpha = core::f32::consts::TAU * rng.uniform();
                (
                    st.fading_hz * alpha.cos(),
                    core::f32::consts::TAU * rng.uniform(),
                )
            })
            .collect();
        let start = (st.start_s * FS) as usize;
        let mut i = 0usize;
        loop {
            let got = synth.fill_complex(&mut buf);
            if got == 0 {
                break;
            }
            for z in &buf[..got] {
                let at = start + i;
                if at >= n {
                    break;
                }
                let t = i as f32 / FS;
                let mut v = *z;
                if st.drift_hz_s != 0.0 {
                    let ph = core::f32::consts::PI * st.drift_hz_s * t * t;
                    v *= Complex32::new(ph.cos(), ph.sin());
                }
                if st.fading_hz > 0.0 {
                    let g = paths
                        .iter()
                        .fold(Complex32::new(0.0, 0.0), |acc, &(fd, phi)| {
                            let a = core::f32::consts::TAU * fd * t + phi;
                            acc + Complex32::new(a.cos(), a.sin())
                        })
                        * 0.25; // 1/sqrt(16): unit mean power
                    v *= g;
                }
                audio[at] += v.re;
                i += 1;
            }
        }
    }
    Some(
        audio
            .iter()
            .map(|&x| x.round().clamp(-32768.0, 32767.0) as i16)
            .collect(),
    )
}

/// One test case: what is on the air, for how long, and from which seed.
#[derive(Clone, Debug)]
pub struct Case {
    /// Pattern name, the same for every trial of it.
    pub pattern: alloc::string::String,
    /// Trial number within the pattern (also the seed's low bits).
    pub trial: u32,
    /// Length of the recording, seconds.
    pub secs: f32,
    /// The transmissions.
    pub stations: Vec<Station<'static>>,
}

impl Case {
    /// The seed the recording is made from.
    pub fn seed(&self) -> u64 {
        let h = self.pattern.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3)
        });
        h ^ u64::from(self.trial)
    }
    /// The recording.
    pub fn audio(&self) -> Option<Vec<i16>> {
        render(&self.stations, self.secs, self.seed())
    }
}

const CQ: &str = "CQ K1ABC CQ";
const CQ2: &str = "CQ W9XYZ CQ";
const LONG: &str = "RAN ALL NIGHT ON BAND NOISE";

fn st(text: &'static str, f0: f32, start: f32, snr: f32) -> Station<'static> {
    Station {
        text,
        f0_hz: f0,
        start_s: start,
        snr_db: snr,
        drift_hz_s: 0.0,
        fading_hz: 0.0,
    }
}

/// The patterns the CoreS3 bench and its host twin run, `trials` of each: an SNR sweep at
/// 1500 Hz, carrier offsets inside channel 0, drift, Rayleigh fading, a multi-frame message, two
/// stations inside channel 0, and noise alone (#499).
pub fn catalogue(trials: u32) -> Vec<Case> {
    use alloc::format;
    let mut cases = Vec::new();
    let mut add = |pattern: alloc::string::String, secs: f32, stations: Vec<Station<'static>>| {
        for trial in 0..trials {
            // the start moves with the trial so the frame lands at different window phases
            let jitter = 0.47 * trial as f32 / trials as f32;
            let stations = stations
                .iter()
                .map(|s| Station {
                    start_s: s.start_s + jitter,
                    ..s.clone()
                })
                .collect();
            cases.push(Case {
                pattern: pattern.clone(),
                trial,
                secs,
                stations,
            });
        }
    };
    for snr in [-18, -17, -16, -15, -14, -12, -10] {
        add(
            format!("awgn {snr} dB"),
            6.0,
            alloc::vec![st(CQ, 1500.0, 1.0, snr as f32)],
        );
    }
    for f0 in [1455.0f32, 1500.37, 1530.0, 1548.0] {
        add(
            format!("offset {f0} Hz, -14 dB"),
            6.0,
            alloc::vec![st(CQ, f0, 1.0, -14.0)],
        );
    }
    for drift in [0.2f32, 0.5, 1.0, 2.0] {
        add(
            format!("drift {drift} Hz/s, -12 dB"),
            6.0,
            alloc::vec![Station {
                drift_hz_s: drift,
                ..st(CQ, 1500.0, 1.0, -12.0)
            }],
        );
    }
    for fd in [0.5f32, 2.0, 10.0] {
        add(
            format!("fading {fd} Hz, -8 dB"),
            6.0,
            alloc::vec![Station {
                fading_hz: fd,
                ..st(CQ, 1500.0, 1.0, -8.0)
            }],
        );
    }
    add(
        "long message, -12 dB".into(),
        16.0,
        alloc::vec![st(LONG, 1500.0, 1.0, -12.0)],
    );
    add(
        "two stations 40 Hz apart, -6/-10 dB".into(),
        8.0,
        alloc::vec![st(CQ, 1470.0, 1.0, -6.0), st(CQ2, 1510.0, 3.5, -10.0)],
    );
    add("noise only".into(), 30.0, Vec::new());
    cases
}

/// Callers of the pileup patterns, in the order a pattern takes them.
pub const CALLERS: [&str; 5] = ["W9XYZ", "JA1ABC", "DL2XY", "K4ABC", "VK3NV"];
const CALLERS_TWICE: [&str; 5] = [
    "W9XYZ W9XYZ",
    "JA1ABC JA1ABC",
    "DL2XY DL2XY",
    "K4ABC K4ABC",
    "VK3NV VK3NV",
];
const LONGS: [&str; 3] = [
    "RAN ALL NIGHT ON BAND NOISE",
    "TNX FER QSO 73 GL",
    "WX HERE SUNNY 25C",
];

/// Heavier patterns for the two-core receiver (#499): a CQ answered by a pileup of `n` callers,
/// each sending its call once or twice, starting 0-1.2 s after the CQ ends at a random offset of
/// up to ±30 Hz from it and -14…-2 dB; two or three stations sending long messages at once
/// inside channel 0; and a band of six stations sending long messages across 300-2700 Hz. The
/// recording starts as the CQ ends (a half-duplex receiver hears nothing of its own CQ).
/// Each trial draws its own offsets from its seed.
pub fn pileups(trials: u32) -> Vec<Case> {
    use alloc::format;
    let mut cases = Vec::new();
    let mut add = |pattern: alloc::string::String,
                   secs: f32,
                   draw: &dyn Fn(&mut Rng) -> Vec<Station<'static>>| {
        for trial in 0..trials {
            let mut case = Case {
                pattern: pattern.clone(),
                trial,
                secs,
                stations: Vec::new(),
            };
            let mut rng = Rng::new(case.seed() ^ 0x5eed);
            case.stations = draw(&mut rng);
            cases.push(case);
        }
    };
    let caller = |text: &'static str, rng: &mut Rng| Station {
        text,
        f0_hz: 1500.0 + 60.0 * (rng.uniform() - 0.5),
        start_s: 0.5 + 1.2 * rng.uniform(),
        snr_db: -14.0 + 12.0 * rng.uniform(),
        drift_hz_s: 0.0,
        fading_hz: 0.0,
    };
    for n in [1usize, 2, 3, 5] {
        add(format!("pileup {n} callers"), 5.0, &|rng: &mut Rng| {
            CALLERS[..n].iter().map(|&c| caller(c, rng)).collect()
        });
        add(
            format!("pileup {n} callers, call twice"),
            7.0,
            &|rng: &mut Rng| CALLERS_TWICE[..n].iter().map(|&c| caller(c, rng)).collect(),
        );
    }
    for n in [2usize, 3] {
        add(
            format!("channel 0, {n} long messages at once"),
            20.0,
            &|rng: &mut Rng| {
                LONGS[..n]
                    .iter()
                    .map(|&text| Station {
                        text,
                        f0_hz: 1500.0 + 60.0 * (rng.uniform() - 0.5),
                        start_s: 0.5 + 1.9 * rng.uniform(),
                        snr_db: -12.0 + 10.0 * rng.uniform(),
                        drift_hz_s: 0.0,
                        fading_hz: 0.0,
                    })
                    .collect()
            },
        );
    }
    add("band, 6 long messages".into(), 20.0, &|rng: &mut Rng| {
        (0..6)
            .map(|i| Station {
                text: LONGS[i % 3],
                f0_hz: 300.0 + 400.0 * i as f32 + 100.0 * rng.uniform(),
                start_s: 0.5 + 1.9 * rng.uniform(),
                snr_db: -12.0 + 10.0 * rng.uniform(),
                drift_hz_s: 0.0,
                fading_hz: 0.0,
            })
            .collect()
    });
    cases
}
