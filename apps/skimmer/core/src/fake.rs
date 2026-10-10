// SPDX-License-Identifier: GPL-3.0-only
//! A stand-in SpyServer: the wire protocol of `spyserver.rs`, server side, and
//! a synthetic band that carries a JTTY station. For trying the skimmer
//! without a radio (`examples/fake_spyserver.rs`) and for testing it end to end
//! over a real socket (`tests/fake_spyserver.rs`).
//!
//! It is as small as the client needs: HELLO answered by DEVICE_INFO and
//! CLIENT_SYNC; the IQ format, centre, decimation and streaming settings kept;
//! CLIENT_SYNC when the decimation changes (as a real server's does); PING
//! answered by PONG; and, while streaming, IQ at the rate the decimation says,
//! paced by the wall clock, so the skimmer's slot grid and clock behave as they
//! do on air.
//!
//! The band is white noise and a scripted JTTY traffic: lines of text, each sent
//! by a station at an audio frequency at a time in a cycle that repeats (see
//! [`Band::default`] for a QSO or two and a CQ caller, at the three places upstream
//! looks: the Rx frequency and the side channels at 1350 and 1650 Hz).

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use mfsk_core::jtty::pack::{ExchangeProfile, pack};

use crate::spyserver::*;

/// One transmission of the script.
#[derive(Clone, Debug)]
pub struct Line {
    /// Seconds into the cycle it starts.
    pub at_s: f64,
    /// Audio frequency of its lowest tone, Hz.
    pub hz: f32,
    /// How loud, against [`Band::amplitude`] (1.0 is the same).
    pub level: f32,
    /// What is said, packed into frames as WSJT-X packs it.
    pub text: String,
}

fn line(at_s: f64, hz: f32, level: f32, text: &str) -> Line {
    Line {
        at_s,
        hz,
        level,
        text: text.into(),
    }
}

/// What the stand-in sends.
#[derive(Clone, Debug)]
pub struct Band {
    /// The dial of the JTTY channel; a signal's audio frequency is above it (USB).
    pub dial_hz: f64,
    /// The script repeats every this many seconds, on multiples of it of the
    /// wall clock.
    pub cycle_s: u64,
    pub script: Vec<Line>,
    /// White noise per component, as a fraction of full scale (1.0).
    pub noise: f32,
    /// A signal of level 1.0 peaks at this fraction of full scale.
    pub amplitude: f32,
}

impl Default for Band {
    /// Two QSOs at once and a caller, over 48 s: K1ABC and JA1ABC on 1500 Hz (the
    /// Rx frequency), W9XYZ and VK3NV on 1350 Hz and DL1ABC calling CQ on 1650 Hz
    /// (the side channels), the weaker the further from the first.
    fn default() -> Self {
        Band {
            dial_hz: 14_090_000.0,
            cycle_s: 48,
            script: vec![
                line(1.0, 1500.0, 1.0, "CQ K1ABC FN42"),
                line(9.0, 1500.0, 0.8, "K1ABC JA1ABC"),
                line(17.0, 1500.0, 1.0, "JA1ABC K1ABC 599"),
                line(26.0, 1500.0, 0.8, "K1ABC JA1ABC 599"),
                line(35.0, 1500.0, 1.0, "JA1ABC TU 73 K1ABC"),
                line(3.0, 1350.0, 0.5, "CQ W9XYZ EN34"),
                line(12.0, 1350.0, 0.4, "W9XYZ VK3NV"),
                line(21.0, 1350.0, 0.5, "VK3NV W9XYZ 599"),
                line(30.0, 1350.0, 0.4, "W9XYZ VK3NV 599 73"),
                line(5.0, 1650.0, 0.25, "CQ DL1ABC JO31"),
                line(25.0, 1650.0, 0.25, "CQ DL1ABC JO31"),
                line(41.0, 1650.0, 0.25, "CQ DL1ABC"),
            ],
            noise: 0.02,
            amplitude: 0.3,
        }
    }
}

/// The rates the stand-in offers: 192 kS/s down to 12 kS/s, a stage at a time.
const MAX_RATE: u32 = 192_000;
const MIN_DECIMATION: u32 = 0;
const MAX_DECIMATION: u32 = 4;
/// The device centre the stand-in reports.
const DEVICE_HZ: u32 = 14_100_000;

/// A running stand-in; dropping it stops it.
pub struct FakeServer {
    pub addr: SocketAddr,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl FakeServer {
    /// Listen on `addr` (port 0: any) and serve clients one at a time.
    pub fn start(addr: &str, band: Band) -> std::io::Result<FakeServer> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        let addr = listener.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        let handle = std::thread::spawn(move || {
            while !flag.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((s, peer)) => {
                        eprintln!("fake spyserver: {peer} connected");
                        let _ = serve(s, &band, &flag);
                        eprintln!("fake spyserver: {peer} left");
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                    Err(_) => break,
                }
            }
        });
        Ok(FakeServer {
            addr,
            stop,
            handle: Some(handle),
        })
    }
}

impl Drop for FakeServer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// One message to the client.
fn send(w: &mut impl Write, kind: u32, seq: &mut u32, body: &[u8]) -> std::io::Result<()> {
    let mut m = Vec::with_capacity(20 + body.len());
    for v in [PROTOCOL_VERSION, kind, 0, *seq, body.len() as u32] {
        m.extend_from_slice(&v.to_le_bytes());
    }
    m.extend_from_slice(body);
    *seq = seq.wrapping_add(1);
    w.write_all(&m)
}

fn words_to_bytes(w: &[u32]) -> Vec<u8> {
    w.iter().flat_map(|v| v.to_le_bytes()).collect()
}

/// What the client has set.
struct State {
    format: u32,
    center_hz: u32,
    decimation: u32,
    streaming: bool,
}

impl State {
    fn rate(&self) -> u32 {
        MAX_RATE >> self.decimation
    }
}

fn client_sync(s: &State) -> Vec<u8> {
    // can_control, gain, device centre, IQ centre, then the ranges.
    words_to_bytes(&[1, 4, DEVICE_HZ, s.center_hz, 0, 0, 0, 0, 0, 0, 0, 0, 0])
}

fn device_info() -> Vec<u8> {
    let mut w = [0u32; 14];
    w[0] = DEVICE_AIRSPY_ONE;
    w[2] = MAX_RATE;
    w[3] = 150_000; // usable band
    w[4] = MAX_DECIMATION;
    w[6] = 8; // max gain index
    w[10] = MIN_DECIMATION;
    words_to_bytes(&w)
}

/// A client: read its commands without blocking the IQ, answer them, and stream.
fn serve(mut sock: TcpStream, band: &Band, stop: &AtomicBool) -> std::io::Result<()> {
    sock.set_nodelay(true)?;
    sock.set_read_timeout(Some(Duration::from_millis(5)))?;
    let mut seq = 0u32;
    let mut st = State {
        format: FORMAT_FLOAT,
        center_hz: DEVICE_HZ - 25_000,
        decimation: MAX_DECIMATION,
        streaming: false,
    };
    let mut inbox: Vec<u8> = Vec::new();
    let mut synth = Synth::new(band);
    let mut next_block = Instant::now();
    // The stream's sample count since streaming began, and the wall-clock
    // second it began at.
    let (mut n, mut t0) = (0u64, 0.0f64);
    while !stop.load(Ordering::Relaxed) {
        // Commands: [u32 command][u32 length][body].
        let mut buf = [0u8; 4096];
        match sock.read(&mut buf) {
            Ok(0) => return Ok(()),
            Ok(k) => inbox.extend_from_slice(&buf[..k]),
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) => {}
            Err(e) => return Err(e),
        }
        while inbox.len() >= 8 {
            let cmd = u32::from_le_bytes(inbox[0..4].try_into().unwrap());
            let len = u32::from_le_bytes(inbox[4..8].try_into().unwrap()) as usize;
            if inbox.len() < 8 + len {
                break;
            }
            let body: Vec<u8> = inbox[8..8 + len].to_vec();
            inbox.drain(..8 + len);
            match cmd {
                CMD_HELLO => {
                    send(&mut sock, MSG_DEVICE_INFO, &mut seq, &device_info())?;
                    send(&mut sock, MSG_CLIENT_SYNC, &mut seq, &client_sync(&st))?;
                }
                CMD_PING => send(&mut sock, MSG_PONG, &mut seq, &[])?,
                CMD_SET_SETTING if body.len() >= 8 => {
                    let w = words(&body);
                    let (setting, value) = (w[0], w[1]);
                    match setting {
                        SET_IQ_FORMAT => st.format = value,
                        SET_IQ_FREQUENCY => st.center_hz = value,
                        SET_IQ_DECIMATION => {
                            st.decimation = value.clamp(MIN_DECIMATION, MAX_DECIMATION);
                            // As a real server: a decimation change is answered
                            // with the state.
                            send(&mut sock, MSG_CLIENT_SYNC, &mut seq, &client_sync(&st))?;
                        }
                        SET_STREAMING_ENABLED => {
                            if value != 0 && !st.streaming {
                                (n, t0) = (0, wall_s());
                                next_block = Instant::now();
                            }
                            st.streaming = value != 0;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
        if !st.streaming {
            continue;
        }
        // One block of IQ, when it is due.
        const BLOCK: usize = 4096;
        let rate = f64::from(st.rate());
        if Instant::now() < next_block {
            continue;
        }
        next_block += Duration::from_secs_f64(BLOCK as f64 / rate);
        let mut iq = Vec::with_capacity(2 * BLOCK);
        for i in 0..BLOCK as u64 {
            let t = t0 + (n + i) as f64 / rate;
            iq.extend(synth.sample(t, f64::from(st.center_hz)));
        }
        n += BLOCK as u64;
        let (kind, body) = match st.format {
            FORMAT_INT16 => (
                MSG_INT16_IQ,
                iq.iter()
                    .flat_map(|&v| ((v * 32_767.0).clamp(-32_768.0, 32_767.0) as i16).to_le_bytes())
                    .collect::<Vec<u8>>(),
            ),
            _ => (
                MSG_FLOAT_IQ,
                iq.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>(),
            ),
        };
        send(&mut sock, kind, &mut seq, &body)?;
    }
    Ok(())
}

/// Seconds since the Unix epoch on the PC's clock.
fn wall_s() -> f64 {
    crate::clock::system_ns() as f64 * 1e-9
}

/// The band: noise and the script, a sample at a time.
struct Synth {
    dial_hz: f64,
    cycle_s: f64,
    /// Each line's start (s into the cycle), 12 kHz audio (unit peak, times its
    /// level) and its end.
    lines: Vec<(f64, Vec<f32>)>,
    amplitude: f32,
    noise: f32,
    rng: u32,
}

impl Synth {
    fn new(band: &Band) -> Self {
        let lines = band
            .script
            .iter()
            .filter_map(|l| {
                let atoms = pack(&l.text, ExchangeProfile::Unknown).ok()?;
                let tones = mfsk_core::jtty::tx::tones(&atoms)?;
                let a = mfsk_core::jtty::tx::synth_f32(&tones, l.hz, 1.0);
                let peak = a.iter().fold(0f32, |m, v| m.max(v.abs())).max(1e-9);
                Some((l.at_s, a.iter().map(|v| v / peak * l.level).collect()))
            })
            .collect();
        Synth {
            dial_hz: band.dial_hz,
            cycle_s: band.cycle_s.max(8) as f64,
            lines,
            amplitude: band.amplitude,
            noise: band.noise,
            rng: 0x2545_f491,
        }
    }

    /// The audio of every station at wall-clock time `t` (s): each line starts
    /// at its time in the cycle.
    fn audio_at(&self, t: f64) -> f32 {
        let into_cycle = t.rem_euclid(self.cycle_s);
        let mut sum = 0.0;
        for (at, a) in &self.lines {
            let x = (into_cycle - at) * 12_000.0;
            if x < 0.0 {
                continue;
            }
            let i = x as usize;
            if i + 1 >= a.len() {
                continue;
            }
            // Linear interpolation between the 12 kHz samples.
            sum += a[i] + (a[i + 1] - a[i]) * (x - i as f64) as f32;
        }
        sum
    }

    fn noise(&mut self) -> f32 {
        // Sum of four uniforms: close enough to Gaussian for a noise floor.
        let mut s = 0.0;
        for _ in 0..4 {
            self.rng = self.rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            s += (self.rng >> 8) as f32 / (1u32 << 24) as f32 - 0.5;
        }
        s * 0.866
    }

    /// One complex baseband sample at wall-clock time `t`, for an IQ centre
    /// `center_hz`: the audio (a real signal) mixed up to the dial, both sidebands,
    /// of which the receiver keeps the one it is placed on.
    fn sample(&mut self, t: f64, center_hz: f64) -> [f32; 2] {
        let a = self.audio_at(t);
        let (re, im) = if a != 0.0 {
            let ph = std::f64::consts::TAU * (self.dial_hz - center_hz) * t;
            (
                (f64::from(a) * ph.cos()) as f32,
                (f64::from(a) * ph.sin()) as f32,
            )
        } else {
            (0.0, 0.0)
        };
        [
            re * self.amplitude + self.noise() * self.noise,
            im * self.amplitude + self.noise() * self.noise,
        ]
    }
}
