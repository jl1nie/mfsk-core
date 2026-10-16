// SPDX-License-Identifier: GPL-3.0-only
//! A synthetic `rsp_tcp` capture with a known answer (#686, `docs/notes/RTLTCP_EVAL_PLAN.md`).
//!
//! One source (FT8 / FT4 transmissions on several channels, white noise, 8-bit IQ at 1 024 000 S/s) is
//! written three ways, byte-identical except for the loss:
//!
//! - `clean.bin`: every sample;
//! - `delivered.bin`: hole and reset samples removed, as an unmodified `rsp_tcp` delivers them;
//! - `oracle.bin`: the same samples replaced by silence at their true positions (the upper bound).
//!
//! All three are `scripts/rtltcp/record.py` files (greeting first, an arrival time per chunk), so
//! `recorded_iq`, `replay_rtltcp` and the GUI replay read them as they read a real capture.
//! `truth.json` lists every transmission and every loss event.
//!
//! ```text
//! cargo run --release -p skimmer-core --example synth_rtltcp -- --out DIR [--seconds 600] [--seed 1]
//!     [--loss measured|none|trace:FILE.csv] [--noise-lsb 3] [--latency-ms 20] [--jitter-ms 5]
//!     [--callback-ms 50] [--buffered AT_S:LEN_MS,...]
//! ```
//!
//! The default loss is the dbg4 plain run of 2026-10-15 (`IQ_RSP1_PR688_2026-10-15.md`): 3.54 holes a
//! second, mean 10.46 ms, longest 37 ms, and resets of 1.43 s at 107, 335 and 440 s. Only the count, the
//! total and the maximum were recorded, so the log-normal shape of the hole lengths is an assumption;
//! `trace:` replaces it with the real sequence (`scripts/rtltcp/holes_from_clock_log.py`).

use std::fmt::Write as _;
use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use mfsk_core::engine::tx::{FskWaveform, message_to_tones, synthesize};
use mfsk_core::engine::{MessageCodec, MessageFields};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::Wsjt77Message;
use rustfft::FftPlanner;
use rustfft::num_complex::Complex32;

/// The ADC rate of the RSP1 profile; the source is generated at exactly this rate.
const RATE: u32 = 1_024_000;
/// Audio rate the transmissions are made at.
const AUDIO: u32 = 12_000;
/// One generation block: a 15 s UTC slot (every FT8 and FT4 frame fits inside one).
const BLOCK_S: u32 = 15;
/// Unix time of the source's first sample: 2026-10-15 00:00:00 UTC, on the slot grid.
const T0_S: i64 = 1_792_022_400;
/// IQ centre of the RSP1 profile.
const CENTER_HZ: i64 = 7_016_000;
/// Bytes per record chunk (`record.py` reads whatever `recv` returns; the size does not matter).
const CHUNK: usize = 16_384;

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Ft8,
    Ft4,
}

impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Ft8 => "FT8",
            Mode::Ft4 => "FT4",
        }
    }
    /// Slot length, s.
    fn period(self) -> f64 {
        match self {
            Mode::Ft8 => 15.0,
            Mode::Ft4 => 7.5,
        }
    }
    /// Least spacing between two transmissions' base tones, Hz.
    fn spacing(self) -> f64 {
        match self {
            Mode::Ft8 => 100.0,
            Mode::Ft4 => 150.0,
        }
    }
}

struct Channel {
    mode: Mode,
    dial_hz: i64,
    /// Transmissions per slot.
    per_slot: usize,
    /// The SNRs every slot draws from in turn, dB in 2500 Hz.
    snrs: Vec<f64>,
}

#[derive(Clone, Copy)]
enum LossKind {
    Hole,
    Reset,
}

#[derive(Clone, Copy)]
struct Loss {
    kind: LossKind,
    /// First source sample lost.
    start: u64,
    len: u64,
}

struct Tx {
    mode: Mode,
    dial_hz: i64,
    slot_utc_s: f64,
    dt_s: f64,
    audio_hz: f64,
    snr_db: f64,
    text: String,
}

struct Args {
    out: PathBuf,
    seconds: u32,
    seed: u64,
    loss: String,
    noise_lsb: f64,
    latency_ms: f64,
    jitter_ms: f64,
    callback_ms: f64,
    /// Delivery stalls that lose nothing: (source time s, length ms).
    buffered: Vec<(f64, f64)>,
}

fn parse() -> Result<Args, String> {
    let mut a = Args {
        out: PathBuf::new(),
        seconds: 600,
        seed: 1,
        loss: "measured".into(),
        noise_lsb: 3.0,
        latency_ms: 20.0,
        jitter_ms: 5.0,
        callback_ms: 50.0,
        buffered: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        let mut v = || it.next().ok_or(format!("{k} needs a value"));
        let num = |s: String| s.parse::<f64>().map_err(|e| format!("{s}: {e}"));
        match k.as_str() {
            "--out" => a.out = v()?.into(),
            "--seconds" => a.seconds = num(v()?)? as u32,
            "--seed" => a.seed = num(v()?)? as u64,
            "--loss" => a.loss = v()?,
            "--noise-lsb" => a.noise_lsb = num(v()?)?,
            "--latency-ms" => a.latency_ms = num(v()?)?,
            "--jitter-ms" => a.jitter_ms = num(v()?)?,
            "--callback-ms" => a.callback_ms = num(v()?)?,
            "--buffered" => {
                for p in v()?.split(',') {
                    let (t, l) = p.split_once(':').ok_or(format!("{p}: AT_S:LEN_MS"))?;
                    a.buffered.push((num(t.into())?, num(l.into())?));
                }
            }
            _ => return Err(format!("unknown argument {k}")),
        }
    }
    if a.out.as_os_str().is_empty() {
        return Err("--out DIR is required".into());
    }
    if a.seconds == 0 || !a.seconds.is_multiple_of(BLOCK_S) {
        return Err(format!(
            "--seconds must be a positive multiple of {BLOCK_S}"
        ));
    }
    Ok(a)
}

/// SplitMix64: one seeded stream per purpose, so the signals do not change when the loss does.
struct Rng(u64);

impl Rng {
    fn new(seed: u64, stream: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ stream.wrapping_mul(0xD1B5_4A32_D192_ED03))
    }
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn normal_pair(&mut self) -> (f64, f64) {
        let r = (-2.0 * self.uniform().ln()).sqrt();
        let (s, c) = (std::f64::consts::TAU * self.uniform()).sin_cos();
        (r * c, r * s)
    }
    fn shuffle<T>(&mut self, v: &mut [T]) {
        for i in (1..v.len()).rev() {
            v.swap(i, (self.next() % (i as u64 + 1)) as usize);
        }
    }
}

fn channels() -> Vec<Channel> {
    let grid = |lo: i32, hi: i32| (lo..=hi).step_by(2).map(f64::from).collect::<Vec<_>>();
    vec![
        Channel {
            mode: Mode::Ft8,
            dial_hz: 7_074_000,
            per_slot: 13,
            snrs: grid(-24, 0),
        },
        Channel {
            mode: Mode::Ft8,
            dial_hz: 7_041_000,
            per_slot: 13,
            snrs: grid(-24, 0),
        },
        Channel {
            mode: Mode::Ft4,
            dial_hz: 7_047_500,
            per_slot: 11,
            snrs: grid(-20, 0),
        },
    ]
}

/// A standard callsign and locator, unique for `i` within a run.
fn station(i: usize) -> (String, String) {
    const PREFIX: [&str; 8] = ["JA", "JH", "DL", "VK", "W", "K", "G", "F"];
    let l = |k: usize| (b'A' + (k % 26) as u8) as char;
    // Digit, then three letters, then the prefix: a different call for every i below 1.4 million.
    let r = i / 10;
    let call = format!(
        "{}{}{}{}{}",
        PREFIX[(r / 17_576) % PREFIX.len()],
        i % 10,
        l(r / 676),
        l(r / 26),
        l(r)
    );
    let grid = format!(
        "{}{}{}{}",
        (b'A' + (i % 18) as u8) as char,
        (b'A' + ((i / 18) % 18) as u8) as char,
        i % 10,
        (i / 10) % 10
    );
    (call, grid)
}

fn tones(mode: Mode, text: &str) -> Vec<u8> {
    let p: Vec<&str> = text.split(' ').collect();
    let bits: [u8; 77] = Wsjt77Message
        .pack(&MessageFields {
            call1: Some(p[0].into()),
            call2: Some(p[1].into()),
            grid: Some(p[2].into()),
            ..MessageFields::default()
        })
        .unwrap_or_else(|| panic!("cannot pack {text}"))
        .try_into()
        .unwrap();
    match mode {
        Mode::Ft8 => message_to_tones::<Ft8>(&bits),
        Mode::Ft4 => message_to_tones::<Ft4>(&bits),
    }
}

fn audio(mode: Mode, tones: &[u8], f0: f64) -> Vec<f32> {
    fn s<P: FskWaveform>(t: &[u8], f0: f64) -> Vec<f32> {
        synthesize::<P>(t, AUDIO, f0 as f32, 1.0)
    }
    match mode {
        Mode::Ft8 => s::<Ft8>(tones, f0),
        Mode::Ft4 => s::<Ft4>(tones, f0),
    }
}

/// Every transmission of the run: per slot and channel, `per_slot` of them at distinct audio
/// frequencies, the SNR grid shuffled and taken in turn so each SNR gets the same count.
fn plan_transmissions(seconds: u32, seed: u64) -> Vec<Tx> {
    let mut rng = Rng::new(seed, 1);
    let mut out = Vec::new();
    let mut n_station = (seed as usize % 97) * 1_000;
    for ch in channels() {
        let slots = (f64::from(seconds) / ch.mode.period()).round() as usize;
        let mut pool: Vec<f64> = Vec::new();
        for k in 0..slots {
            // Base tones between 300 Hz and 2700 Hz less the signal's width, `spacing` apart, jittered.
            let room = 2_700.0 - 300.0 - 60.0;
            let lanes = (room / ch.mode.spacing()) as usize;
            let mut lane: Vec<usize> = (0..lanes).collect();
            rng.shuffle(&mut lane);
            for &ln in lane.iter().take(ch.per_slot) {
                if pool.is_empty() {
                    pool = ch.snrs.clone();
                    rng.shuffle(&mut pool);
                }
                let snr_db = pool.pop().unwrap();
                let audio_hz = 300.0
                    + ln as f64 * ch.mode.spacing()
                    + (rng.uniform() - 0.5) * (ch.mode.spacing() - 60.0).max(0.0) * 0.5;
                let audio_hz = (audio_hz * 10.0).round() / 10.0;
                // WSJT-X's DT is the start against slot + 0.5 s.
                let dt_s = ((rng.uniform() * 0.6 - 0.2) * 100.0).round() / 100.0;
                let (call, grid) = station(n_station);
                n_station += 1;
                out.push(Tx {
                    mode: ch.mode,
                    dial_hz: ch.dial_hz,
                    slot_utc_s: T0_S as f64 + k as f64 * ch.mode.period(),
                    dt_s,
                    audio_hz,
                    snr_db,
                    text: format!("CQ {call} {grid}"),
                });
            }
        }
    }
    out
}

/// Loss events over the whole source, sorted, not overlapping.
fn plan_loss(spec: &str, seconds: u32, seed: u64) -> Result<Vec<Loss>, String> {
    let total = u64::from(seconds) * u64::from(RATE);
    let ms = |x: f64| (x * 1e-3 * f64::from(RATE)).round() as u64;
    let mut ev = Vec::new();
    match spec {
        "none" => {}
        "measured" => {
            let mut rng = Rng::new(seed, 2);
            let (rate_per_s, mean_ms, max_ms, sigma): (f64, f64, f64, f64) =
                (3.54, 10.46, 37.1, 0.6);
            // Log-normal with this mean: mu = ln(mean) - sigma^2 / 2.
            let mu = mean_ms.ln() - sigma * sigma / 2.0;
            let mut t = 0.0;
            loop {
                t += -rng.uniform().ln() / rate_per_s;
                if t >= f64::from(seconds) {
                    break;
                }
                let len = (mu + sigma * rng.normal_pair().0).exp().min(max_ms);
                ev.push(Loss {
                    kind: LossKind::Hole,
                    start: (t * f64::from(RATE)) as u64,
                    len: ms(len).max(1),
                });
            }
            for at in [107.0, 335.0, 440.0] {
                if at < f64::from(seconds) {
                    ev.push(Loss {
                        kind: LossKind::Reset,
                        start: (at * f64::from(RATE)) as u64,
                        len: ms(1_430.0),
                    });
                }
            }
        }
        s if s.starts_with("trace:") => {
            let text = std::fs::read_to_string(&s[6..]).map_err(|e| format!("{}: {e}", &s[6..]))?;
            for line in text
                .lines()
                .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            {
                let f: Vec<&str> = line.split(',').map(str::trim).collect();
                if f.len() != 3 || f[0] == "kind" {
                    continue;
                }
                let kind = match f[0] {
                    "hole" => LossKind::Hole,
                    "reset" => LossKind::Reset,
                    k => return Err(format!("trace: unknown kind {k}")),
                };
                let n = |x: &str| x.parse::<u64>().map_err(|e| format!("trace: {x}: {e}"));
                ev.push(Loss {
                    kind,
                    start: n(f[1])?,
                    len: n(f[2])?,
                });
            }
        }
        s => return Err(format!("--loss {s}: measured, none or trace:FILE")),
    }
    ev.sort_by_key(|e| e.start);
    // A hole inside a reset (or two overlapping holes) becomes one event; nothing past the end.
    let mut merged: Vec<Loss> = Vec::new();
    for e in ev {
        if e.start >= total {
            break;
        }
        let e = Loss {
            len: e.len.min(total - e.start),
            ..e
        };
        match merged.last_mut() {
            Some(last) if e.start <= last.start + last.len => {
                let end = (last.start + last.len).max(e.start + e.len);
                last.len = end - last.start;
                if matches!(e.kind, LossKind::Reset) {
                    last.kind = LossKind::Reset;
                }
            }
            _ => merged.push(e),
        }
    }
    Ok(merged)
}

/// A `record.py` file being written: the chunk count is patched in at the end.
struct Record {
    w: BufWriter<File>,
    chunks: u32,
    pending: Vec<u8>,
    last_t: f64,
    bytes: u64,
}

impl Record {
    fn create(path: &Path) -> std::io::Result<Self> {
        let mut w = BufWriter::with_capacity(1 << 22, File::create(path)?);
        w.write_all(&(T0_S * 1_000_000_000).to_le_bytes())?;
        w.write_all(&0u32.to_le_bytes())?;
        let mut r = Record {
            w,
            chunks: 0,
            pending: Vec::new(),
            last_t: 0.0,
            bytes: 0,
        };
        // rsp_tcp's greeting without -E: RTL0, tuner 5, 28 gain steps.
        let mut hello = b"RTL0".to_vec();
        hello.extend_from_slice(&5u32.to_be_bytes());
        hello.extend_from_slice(&28u32.to_be_bytes());
        r.chunk(0.0, &hello)?;
        Ok(r)
    }

    fn chunk(&mut self, t: f64, b: &[u8]) -> std::io::Result<()> {
        // `record.py` times are monotonic; so are these.
        let t = t.max(self.last_t);
        self.last_t = t;
        self.w.write_all(&t.to_le_bytes())?;
        self.w.write_all(&(b.len() as u32).to_le_bytes())?;
        self.w.write_all(b)?;
        self.chunks += 1;
        Ok(())
    }

    /// Deliver what is pending at `t` (s since T0).
    fn deliver(&mut self, t: f64) -> std::io::Result<()> {
        let all = std::mem::take(&mut self.pending);
        for c in all.chunks(CHUNK) {
            self.chunk(t, c)?;
        }
        Ok(())
    }

    fn finish(mut self) -> std::io::Result<()> {
        self.w.flush()?;
        let mut f = self.w.into_inner().map_err(|e| e.into_error())?;
        f.seek(SeekFrom::Start(8))?;
        f.write_all(&self.chunks.to_le_bytes())?;
        Ok(())
    }
}

#[derive(Default)]
struct Totals {
    signal_clip: u64,
}

fn main() {
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("synth_rtltcp: {e}");
            std::process::exit(2);
        }
    };
    if let Err(e) = run(&args) {
        eprintln!("synth_rtltcp: {e}");
        std::process::exit(1);
    }
}

fn run(a: &Args) -> Result<(), String> {
    let io = |e: std::io::Error| e.to_string();
    std::fs::create_dir_all(&a.out).map_err(io)?;
    let txs = plan_transmissions(a.seconds, a.seed);
    let loss = plan_loss(&a.loss, a.seconds, a.seed)?;
    let mut buffered: Vec<(u64, f64)> = a
        .buffered
        .iter()
        .map(|&(t, l)| ((t * f64::from(RATE)) as u64, l * 1e-3))
        .collect();
    buffered.sort_by_key(|x| x.0);

    let n = (BLOCK_S * RATE) as usize;
    let m = (BLOCK_S * AUDIO) as usize;
    let mut planner = FftPlanner::<f32>::new();
    let fwd = planner.plan_fft_forward(m);
    let inv = planner.plan_fft_inverse(n);
    let mut noise = Rng::new(a.seed, 3);
    let mut timing = Rng::new(a.seed, 4);
    let sigma = a.noise_lsb;
    // WSJT-X's SNR is signal power over the noise power in 2500 Hz. The noise is complex white with
    // variance 2 sigma^2 over RATE; a tone A e^{jwt} has power A^2.
    let noise_2500 = 2.0 * sigma * sigma * 2_500.0 / f64::from(RATE);

    let names = ["clean", "delivered", "oracle"];
    let mut recs = names
        .iter()
        .map(|s| Record::create(&a.out.join(format!("{s}.bin"))))
        .collect::<Result<Vec<_>, _>>()
        .map_err(io)?;
    let mut totals = Totals::default();
    let mut li = 0usize; // first loss event that may still matter
    let mut bi = 0usize;
    let mut extra_delay = 0.0f64; // a buffered stall that is still being caught up
    let callback = ((a.callback_ms * 1e-3 * f64::from(RATE)) as u64).max(1);
    let mut wide = vec![Complex32::new(0.0, 0.0); n];
    let mut chan = vec![Complex32::new(0.0, 0.0); m];
    let mut iq = vec![0u8; 2 * n];

    for block in 0..(a.seconds / BLOCK_S) as usize {
        let t_block = T0_S as f64 + (block as u32 * BLOCK_S) as f64;
        wide.iter_mut().for_each(|z| *z = Complex32::new(0.0, 0.0));
        // Each channel: real audio at 12 kHz, its analytic spectrum shifted to the dial's offset.
        for ch in channels() {
            let mut sum = vec![0.0f32; m];
            let mut any = false;
            for tx in txs.iter().filter(|t| {
                t.dial_hz == ch.dial_hz && t.slot_utc_s >= t_block && t.slot_utc_s < t_block + 15.0
            }) {
                let wave = audio(tx.mode, &tones(tx.mode, &tx.text), tx.audio_hz);
                let amp = (10f64.powf(tx.snr_db / 10.0) * noise_2500).sqrt() as f32;
                let start = ((tx.slot_utc_s - t_block + 0.5 + tx.dt_s) * f64::from(AUDIO)).round();
                let start = start as usize;
                assert!(
                    start + wave.len() <= m,
                    "{} does not fit its block",
                    tx.text
                );
                for (k, &x) in wave.iter().enumerate() {
                    sum[start + k] += amp * x;
                }
                any = true;
            }
            if !any {
                continue;
            }
            for (c, &x) in chan.iter_mut().zip(&sum) {
                *c = Complex32::new(x, 0.0);
            }
            fwd.process(&mut chan);
            // Bin k of the audio spectrum is k / 15 Hz; so is bin b of the wide one. Positive
            // frequencies only, doubled (the analytic signal), scaled for the unnormalised inverse.
            let shift = (ch.dial_hz - CENTER_HZ) * i64::from(BLOCK_S);
            for (k, &x) in chan.iter().enumerate().take(m / 2).skip(1) {
                let b = (k as i64 + shift).rem_euclid(n as i64) as usize;
                wide[b] += x * (2.0 / m as f32);
            }
        }
        inv.process(&mut wide);
        for (k, z) in wide.iter().enumerate() {
            let (ni, nq) = noise.normal_pair();
            let q = |v: f64, clip: &mut u64| {
                let r = (128.0 + v).round();
                if !(0.0..=255.0).contains(&r) {
                    *clip += 1;
                }
                r.clamp(0.0, 255.0) as u8
            };
            iq[2 * k] = q(f64::from(z.re) + sigma * ni, &mut totals.signal_clip);
            iq[2 * k + 1] = q(f64::from(z.im) + sigma * nq, &mut totals.signal_clip);
        }

        // Deliver the block in callbacks: each callback's surviving samples arrive together.
        let base = block as u64 * n as u64;
        let mut s = 0u64;
        while s < n as u64 {
            let e = (s + callback - (base + s) % callback).min(n as u64);
            let (g0, g1) = (base + s, base + e);
            while li < loss.len() && loss[li].start + loss[li].len <= g0 {
                li += 1;
            }
            // Arrival: capture of the callback's last sample, latency, jitter, and any buffered stall.
            while bi < buffered.len() && buffered[bi].0 < g1 {
                extra_delay = extra_delay.max(buffered[bi].1);
                bi += 1;
            }
            let capture = g1 as f64 / f64::from(RATE);
            let jitter = -timing.uniform().ln() * a.jitter_ms * 1e-3;
            let t = capture + a.latency_ms * 1e-3 + jitter + extra_delay;
            // A stall drains at the line rate: assume the backlog clears 10x faster than real time.
            extra_delay = (extra_delay - (g1 - g0) as f64 / f64::from(RATE) * 9.0).max(0.0);
            let bytes = &iq[2 * s as usize..2 * e as usize];
            recs[0].pending.extend_from_slice(bytes);
            // Lost ranges inside this callback.
            let mut keep = vec![true; (e - s) as usize];
            for l in loss[li..].iter().take_while(|l| l.start < g1) {
                let (a0, a1) = (l.start.max(g0), (l.start + l.len).min(g1));
                keep[(a0 - g0) as usize..(a1 - g0) as usize].fill(false);
            }
            for (j, &k) in keep.iter().enumerate() {
                let pair = &bytes[2 * j..2 * j + 2];
                if k {
                    recs[1].pending.extend_from_slice(pair);
                    recs[2].pending.extend_from_slice(pair);
                } else {
                    recs[2].pending.extend_from_slice(&[128, 128]);
                }
            }
            for r in &mut recs {
                // Nothing to send (a callback wholly inside a reset): nothing arrives.
                if !r.pending.is_empty() {
                    r.bytes += r.pending.len() as u64;
                    r.deliver(t).map_err(io)?;
                }
            }
            s = e;
        }
        eprintln!("block {}/{}", block + 1, a.seconds / BLOCK_S);
    }
    let bytes: Vec<u64> = recs.iter().map(|r| r.bytes).collect();
    for r in recs {
        r.finish().map_err(io)?;
    }
    write_truth(a, &txs, &loss, &bytes, &totals).map_err(io)?;
    Ok(())
}

fn write_truth(
    a: &Args,
    txs: &[Tx],
    loss: &[Loss],
    bytes: &[u64],
    totals: &Totals,
) -> std::io::Result<()> {
    let total = u64::from(a.seconds) * u64::from(RATE);
    let lost: u64 = loss.iter().map(|l| l.len).sum();
    let holes = loss
        .iter()
        .filter(|l| matches!(l.kind, LossKind::Hole))
        .count();
    let mut j = String::new();
    let q = |s: &str| format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\""));
    let _ = writeln!(j, "{{");
    let _ = writeln!(
        j,
        "  \"generator\": \"apps/skimmer/core/examples/synth_rtltcp.rs\","
    );
    let _ = writeln!(
        j,
        "  \"params\": {{\"seconds\": {}, \"seed\": {}, \"loss\": {}, \"noise_lsb\": {}, \"latency_ms\": {}, \"jitter_ms\": {}, \"callback_ms\": {}, \"buffered\": [{}]}},",
        a.seconds,
        a.seed,
        q(&a.loss),
        a.noise_lsb,
        a.latency_ms,
        a.jitter_ms,
        a.callback_ms,
        a.buffered
            .iter()
            .map(|(t, l)| format!("[{t}, {l}]"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let _ = writeln!(
        j,
        "  \"stream\": {{\"rate_hz\": {RATE}, \"center_hz\": {CENTER_HZ}, \"format\": \"cu8\", \"start_utc_ns\": {}, \"snr_reference_hz\": 2500}},",
        T0_S * 1_000_000_000
    );
    let _ = writeln!(
        j,
        "  \"channels\": [{}],",
        channels()
            .iter()
            .map(|c| format!(
                "{{\"mode\": \"{}\", \"dial_hz\": {}}}",
                c.mode.name(),
                c.dial_hz
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let _ = writeln!(
        j,
        "  \"files\": {{\"clean\": {{\"payload_bytes\": {}}}, \"delivered\": {{\"payload_bytes\": {}}}, \"oracle\": {{\"payload_bytes\": {}}}}},",
        bytes[0], bytes[1], bytes[2]
    );
    let _ = writeln!(
        j,
        "  \"stats\": {{\"source_samples\": {total}, \"lost_samples\": {lost}, \"lost_fraction\": {:.6}, \"holes\": {holes}, \"resets\": {}, \"clipped_values\": {}}},",
        lost as f64 / total as f64,
        loss.len() - holes,
        totals.signal_clip
    );
    let _ = writeln!(j, "  \"loss\": [");
    for (i, l) in loss.iter().enumerate() {
        let _ = writeln!(
            j,
            "    {{\"kind\": \"{}\", \"start_sample\": {}, \"len\": {}, \"start_s\": {:.6}}}{}",
            match l.kind {
                LossKind::Hole => "hole",
                LossKind::Reset => "reset",
            },
            l.start,
            l.len,
            l.start as f64 / f64::from(RATE),
            if i + 1 < loss.len() { "," } else { "" }
        );
    }
    let _ = writeln!(j, "  ],");
    let _ = writeln!(j, "  \"transmissions\": [");
    for (i, t) in txs.iter().enumerate() {
        let _ = writeln!(
            j,
            "    {{\"mode\": \"{}\", \"dial_hz\": {}, \"slot_utc_s\": {}, \"dt_s\": {}, \"audio_hz\": {}, \"rf_hz\": {}, \"snr_db\": {}, \"text\": {}}}{}",
            t.mode.name(),
            t.dial_hz,
            t.slot_utc_s,
            t.dt_s,
            t.audio_hz,
            t.dial_hz as f64 + t.audio_hz,
            t.snr_db,
            q(&t.text),
            if i + 1 < txs.len() { "," } else { "" }
        );
    }
    let _ = writeln!(j, "  ]");
    let _ = writeln!(j, "}}");
    std::fs::write(a.out.join("truth.json"), j)
}
