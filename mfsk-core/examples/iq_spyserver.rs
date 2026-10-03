// SPDX-License-Identifier: GPL-3.0-or-later
//! Decode FT8 / FT4 / … straight from a SpyServer (Airspy HF+, Airspy,
//! RTL-SDR behind `spyserver`), one `IqReceiver` channel per dial.
//!
//! ```text
//! cargo run --release --features full --example iq_spyserver -- \
//!     --server 192.168.1.26:5555 --ch FT8@7041000 --ch FT8@7074000 --ch FT4@7047500
//! ```
//!
//! **Sharing the radio.** Run it beside the operator's client (SDR#): start
//! SDR# first, since control of the device goes to whoever connects first.
//! This program never tunes the device unless `--tune` is given; it sets only
//! its own IQ (DDC) centre, which SpyServer lets a client without control
//! move anywhere in the device's band. Given control without `--tune`, it
//! leaves and reconnects 10 s later, so the operator's client can take it.
//! When the operator retunes, it plans again: channels outside the new band
//! pause and resume when the band comes back.
//!
//! `--center` and `--rate` are chosen when not given: the IQ centre 25 kHz
//! below the lowest dial (so DC is clear of every channel), moved as little
//! as needed to keep the IQ span inside the device's band, at the lowest
//! rate that holds the most channels by the receiver's own placement rule.
//! `--gain N` sets the device gain index (with `--tune`).
//! `--format float` (the default) or `int16` is the IQ format asked for;
//! float needs no digital-gain guess to stay clear of clipping, at twice the
//! bytes (3.6 MB/s at 456 kS/s). A server that forces uint8 or int24 is
//! read as it comes.
//!
//! The protocol is SpyServer's as SDR++ implements it
//! (`source_modules/spyserver_source/src/spyserver_protocol.h`,
//! `spyserver_client.cpp`): HELLO, then DEVICE_INFO and CLIENT_SYNC from the
//! server, then SET_SETTINGs and IQ messages of `[u32 protocol, u32 type |
//! gain_db << 16, u32 stream, u32 sequence, u32 body size]` + body.
//!
//! **Time.** SpyServer sends no timestamps, so the slot grid comes from this
//! host's clock: each IQ message gives a candidate UTC for sample 0, `now -
//! samples_so_far / fs`. Network and buffering delay only ever make a
//! candidate late, so the anchor is the *minimum* candidate over the last
//! `WINDOW_S` seconds, and the receiver is re-anchored only when that moves
//! by more than `--reanchor-ms`. Between re-anchors the device's own sample
//! clock keeps time; an undisciplined host clock shows as re-anchors and DT
//! offsets. Against an Airspy HF+ on the LAN (2026-10-03), the estimate moved
//! 2 ms in 120 s with a 5 ms delivery delay and no re-anchor.

use std::collections::VecDeque;
use std::io::{BufReader, Read, Write};
use std::net::TcpStream;
use std::process::ExitCode;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mfsk_core::iq::{Channelizer, IqDecode, IqMode, IqReceiver, IqSampleFormat, IqStream};

const PROTOCOL_VERSION: u32 = (2 << 24) | 1700;
const CMD_HELLO: u32 = 0;
const CMD_SET_SETTING: u32 = 2;
const CMD_PING: u32 = 3;
const SET_STREAMING_MODE: u32 = 0;
const SET_STREAMING_ENABLED: u32 = 1;
const SET_GAIN: u32 = 2;
const SET_IQ_FORMAT: u32 = 100;
const SET_IQ_FREQUENCY: u32 = 101;
const SET_IQ_DECIMATION: u32 = 102;
const SET_IQ_DIGITAL_GAIN: u32 = 103;
const STREAM_MODE_IQ_ONLY: u32 = 1;
const FORMAT_INT16: u32 = 2;
const FORMAT_FLOAT: u32 = 4;
const MSG_DEVICE_INFO: u32 = 0;
const MSG_CLIENT_SYNC: u32 = 1;
const MSG_PONG: u32 = 2;
const MSG_UINT8_IQ: u32 = 100;
const MSG_INT16_IQ: u32 = 101;
const MSG_INT24_IQ: u32 = 102;
const MSG_FLOAT_IQ: u32 = 103;
const DEVICE_AIRSPY_ONE: u32 = 1;
/// The protocol's own bound on a message body (`SPYSERVER_MAX_MESSAGE_BODY_SIZE`).
const MAX_BODY: usize = 1 << 20;

/// How long the minimum-delay estimate of the anchor looks back. Long
/// enough that a quiet moment on the network falls inside it, short enough
/// to follow an NTP step within a slot or two.
const WINDOW_S: f64 = 30.0;

fn parse_mode(s: &str) -> Option<IqMode> {
    Some(match s.to_ascii_uppercase().as_str() {
        "FT8" => IqMode::Ft8,
        "FT4" => IqMode::Ft4,
        #[cfg(feature = "fst4")]
        "FST4-60" => IqMode::Fst4S60,
        #[cfg(feature = "wspr")]
        "WSPR" => IqMode::Wspr,
        #[cfg(feature = "jt9")]
        "JT9" => IqMode::Jt9,
        #[cfg(feature = "jt65")]
        "JT65" => IqMode::Jt65,
        #[cfg(feature = "q65")]
        "Q65-60A" => IqMode::Q65A60,
        _ => return None,
    })
}

struct Args {
    server: String,
    channels: Vec<(IqMode, f64, String)>,
    center: Option<f64>,
    rate: Option<u32>,
    gain: Option<u32>,
    tune: bool,
    format: u32,
    channelizer: Channelizer,
    iq_swap: bool,
    reanchor_ns: i64,
}

fn parse_args() -> Option<Args> {
    let mut a = Args {
        server: "127.0.0.1:5555".into(),
        channels: Vec::new(),
        center: None,
        rate: None,
        gain: None,
        tune: false,
        format: FORMAT_FLOAT,
        channelizer: Channelizer::Direct,
        iq_swap: false,
        reanchor_ns: 500_000_000,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--server" => a.server = it.next()?,
            "--center" => a.center = Some(it.next()?.parse().ok()?),
            "--rate" => a.rate = Some(it.next()?.parse().ok()?),
            "--gain" => a.gain = Some(it.next()?.parse().ok()?),
            "--tune" => a.tune = true,
            "--format" => {
                a.format = match it.next()?.as_str() {
                    "float" => FORMAT_FLOAT,
                    "int16" => FORMAT_INT16,
                    _ => return None,
                }
            }
            "--pfb" => a.channelizer = Channelizer::Pfb,
            "--iq-swap" => a.iq_swap = true,
            "--reanchor-ms" => a.reanchor_ns = it.next()?.parse::<i64>().ok()? * 1_000_000,
            "--ch" => {
                let spec = it.next()?;
                let (m, f) = spec.split_once('@')?;
                let Some(mode) = parse_mode(m) else {
                    eprintln!("unknown mode {m:?}");
                    return None;
                };
                a.channels.push((mode, f.parse().ok()?, spec.clone()));
            }
            _ => return None,
        }
    }
    (!a.channels.is_empty()).then_some(a)
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64
}

/// The socket is read on its own thread, so a slot's decode (which runs
/// inside `push`) never stops the read. It did: the first decode takes
/// 400-450 ms (later ones under 20 ms), the server's queue for this client
/// overflowed meanwhile, and on Windows it dropped 1-13 IQ messages 20-31 s
/// into every run (3 of 3). Linux's larger receive buffer absorbed the same
/// stall, so the WSL runs never showed it.
struct Conn {
    w: TcpStream,
    msgs: mpsc::Receiver<std::io::Result<Message>>,
    /// Body bytes read but not yet taken by the decoder.
    queued: Arc<AtomicUsize>,
}

struct Message {
    kind: u32,
    seq: u32,
    body: Vec<u8>,
    /// When the reader thread had the whole message, ns since the epoch:
    /// the time anchor must not include the time it then waited in the queue.
    arrival_ns: i64,
}

impl Conn {
    fn new(sock: TcpStream) -> std::io::Result<Self> {
        let mut r = BufReader::with_capacity(1 << 20, sock.try_clone()?);
        let (tx, msgs) = mpsc::channel();
        let queued = Arc::new(AtomicUsize::new(0));
        let q = queued.clone();
        std::thread::spawn(move || {
            loop {
                let m = read_message(&mut r);
                if let Ok(m) = &m {
                    q.fetch_add(m.body.len(), Ordering::Relaxed);
                }
                let failed = m.is_err();
                if tx.send(m).is_err() || failed {
                    break;
                }
            }
        });
        Ok(Conn {
            w: sock,
            msgs,
            queued,
        })
    }

    fn command(&mut self, cmd: u32, body: &[u8]) -> std::io::Result<()> {
        let mut b = Vec::with_capacity(8 + body.len());
        b.extend_from_slice(&cmd.to_le_bytes());
        b.extend_from_slice(&(body.len() as u32).to_le_bytes());
        b.extend_from_slice(body);
        self.w.write_all(&b)
    }

    fn set(&mut self, setting: u32, value: u32) -> std::io::Result<()> {
        let mut b = [0u8; 8];
        b[..4].copy_from_slice(&setting.to_le_bytes());
        b[4..].copy_from_slice(&value.to_le_bytes());
        self.command(CMD_SET_SETTING, &b)
    }

    fn read(&mut self) -> std::io::Result<Message> {
        let m = self
            .msgs
            .recv()
            .map_err(|_| std::io::Error::other("reader thread ended"))??;
        self.queued.fetch_sub(m.body.len(), Ordering::Relaxed);
        Ok(m)
    }
}

impl Drop for Conn {
    /// Ends the reader thread: its blocking read fails once the socket is shut.
    fn drop(&mut self) {
        let _ = self.w.shutdown(std::net::Shutdown::Both);
    }
}

fn read_message(r: &mut impl Read) -> std::io::Result<Message> {
    let mut h = [0u8; 20];
    r.read_exact(&mut h)?;
    let f = |i: usize| u32::from_le_bytes(h[4 * i..4 * i + 4].try_into().unwrap());
    let size = f(4) as usize;
    if size > MAX_BODY {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("message body of {size} bytes"),
        ));
    }
    let mut body = vec![0u8; size];
    r.read_exact(&mut body)?;
    Ok(Message {
        // The upper 16 bits carry the applied digital gain in dB, a
        // constant scale while the settings stand: not needed, since
        // each slot is normalised before decoding.
        kind: f(1) & 0xffff,
        seq: f(3),
        body,
        arrival_ns: now_ns(),
    })
}

fn words(b: &[u8]) -> Vec<u32> {
    b.as_chunks::<4>()
        .0
        .iter()
        .map(|&w| u32::from_le_bytes(w))
        .collect()
}

/// Minimum-delay estimate of the UTC of sample 0.
struct AnchorEstimate {
    /// (sample index the candidate was taken at, candidate anchor ns)
    window: VecDeque<(u64, i64)>,
}

impl AnchorEstimate {
    fn push(&mut self, samples: u64, fs: u32, arrival_ns: i64) -> i64 {
        let cand = arrival_ns - (samples as f64 * 1e9 / fs as f64) as i64;
        let keep = (WINDOW_S * fs as f64) as u64;
        while self
            .window
            .front()
            .is_some_and(|&(k, _)| k + keep < samples)
        {
            self.window.pop_front();
        }
        // Monotone deque: only candidates that could still be the minimum.
        while self.window.back().is_some_and(|&(_, c)| c >= cand) {
            self.window.pop_back();
        }
        self.window.push_back((samples, cand));
        self.window.front().unwrap().1
    }
}

fn print_decode(d: &IqDecode) {
    let t = d
        .slot_start_utc_ns
        .map(|ns| {
            let s = ns.div_euclid(1_000_000_000).rem_euclid(86_400);
            format!("{:02}{:02}{:02}", s / 3600, s / 60 % 60, s % 60)
        })
        .unwrap_or_else(|| "------".into());
    println!(
        "{t} {:<4} {:>10.0} {:>4.0} {:>5.1}  {}",
        format!("{:?}", d.mode).to_uppercase(),
        d.abs_freq_hz,
        d.decoded.snr_db,
        d.decoded.dt_sec,
        d.decoded.text
    );
}

/// What the server's last CLIENT_SYNC said, as far as this client cares.
#[derive(Clone, Copy, PartialEq)]
struct Sync {
    can_control: bool,
    device_hz: f64,
    iq_hz: f64,
}

impl Sync {
    fn parse(body: &[u8]) -> Self {
        let w = words(body);
        Sync {
            can_control: w[0] != 0,
            device_hz: w[2] as f64,
            iq_hz: w[3] as f64,
        }
    }
}

/// Why streaming stopped without an I/O error.
enum Stop {
    /// The device or this client's IQ centre moved: plan again.
    Moved(Sync),
}

struct Device {
    kind: u32,
    max_rate: u32,
    /// Usable band around the device centre, Hz (`MaximumBandwidth`).
    bandwidth: f64,
    max_gain: u32,
    /// `(decimation, rate)`, lowest rate first.
    rates: Vec<(u32, u32)>,
}

/// The cheapest stream that holds the most channels: for each rate, the
/// IQ centre nearest the wanted one that keeps the whole IQ span inside the
/// device's band, and the channels that fit there. Channels left out are
/// paused until the device moves back.
struct Plan {
    decim: u32,
    rate: u32,
    center: f64,
    channels: Vec<usize>,
}

fn plan(args: &Args, dev: &Device, device_hz: f64, tune: bool) -> Option<Plan> {
    let lowest = args.channels.iter().map(|c| c.1).fold(f64::MAX, f64::min);
    let wanted = args.center.unwrap_or((lowest - 25_000.0).round());
    let mut best: Option<Plan> = None;
    for &(decim, rate) in &dev.rates {
        if args.rate.is_some_and(|r| r != rate) {
            continue;
        }
        // With --tune the device follows the IQ centre; otherwise the IQ
        // span has to sit inside the band someone else tuned.
        let center = if tune {
            wanted
        } else {
            let room = (dev.bandwidth - rate as f64) / 2.0;
            if room < 0.0 {
                continue;
            }
            wanted.clamp(device_hz - room, device_hz + room).round()
        };
        let fits: Vec<usize> = (0..args.channels.len())
            .filter(|&i| {
                let (mode, dial, _) = &args.channels[i];
                IqReceiver::new(IqStream {
                    sample_rate: rate,
                    center_hz: center,
                    format: IqSampleFormat::Cf32,
                    iq_swap: false,
                })
                .add_channel(*dial, *mode)
                .is_ok()
            })
            .collect();
        if best.as_ref().is_none_or(|b| fits.len() > b.channels.len()) {
            let all = fits.len() == args.channels.len();
            best = Some(Plan {
                decim,
                rate,
                center,
                channels: fits,
            });
            if all {
                break;
            }
        }
    }
    best.filter(|p| !p.channels.is_empty())
}

/// Apply a plan with streaming off, and read back what the server made of it.
/// CLIENT_SYNC comes when the decimation changes (not on a frequency alone,
/// measured), so the decimation is stepped away and back after the frequency,
/// and PING / PONG marks the end of the server's replies.
fn apply(c: &mut Conn, dev: &Device, p: &Plan, args: &Args, tune: bool) -> std::io::Result<Sync> {
    c.set(SET_STREAMING_ENABLED, 0)?;
    c.set(SET_IQ_FORMAT, args.format)?;
    c.set(SET_IQ_FREQUENCY, p.center as u32)?;
    if tune && let Some(g) = args.gain {
        c.set(SET_GAIN, g)?;
    }
    let other = if p.decim > dev.rates[0].0 {
        p.decim - 1
    } else {
        p.decim + 1
    };
    c.set(SET_IQ_DECIMATION, other)?;
    c.set(SET_IQ_DECIMATION, p.decim)?;
    // As SDR++ does: 3 dB of digital gain per decimation stage keeps the
    // level in the integer formats (`computeDigitalGain`); the Airspy One
    // also makes up its device gain.
    let digital = (p.decim as f32 * 3.01) as u32
        + if dev.kind == DEVICE_AIRSPY_ONE {
            dev.max_gain.saturating_sub(args.gain.unwrap_or(0))
        } else {
            0
        };
    c.set(SET_IQ_DIGITAL_GAIN, digital)?;
    c.set(SET_STREAMING_MODE, STREAM_MODE_IQ_ONLY)?;
    c.command(CMD_PING, &[])?;
    let mut last = None;
    loop {
        let m = c.read()?;
        match m.kind {
            MSG_CLIENT_SYNC => last = Some(Sync::parse(&m.body)),
            MSG_PONG => break,
            _ => {}
        }
    }
    last.ok_or_else(|| std::io::Error::other("no CLIENT_SYNC in reply to the settings"))
}

fn run(args: &Args) -> std::io::Result<()> {
    let sock = TcpStream::connect(&args.server)?;
    sock.set_nodelay(true)?;
    let mut c = Conn::new(sock)?;
    let mut hello = PROTOCOL_VERSION.to_le_bytes().to_vec();
    hello.extend_from_slice(b"mfsk-core iq_spyserver");
    c.command(CMD_HELLO, &hello)?;

    let (mut info, mut sync) = (None, None);
    while info.is_none() || sync.is_none() {
        let m = c.read()?;
        match m.kind {
            MSG_DEVICE_INFO => info = Some(words(&m.body)),
            MSG_CLIENT_SYNC => sync = Some(Sync::parse(&m.body)),
            _ => {}
        }
    }
    let (info, mut sync) = (info.unwrap(), sync.unwrap());
    let dev = Device {
        kind: info[0],
        max_rate: info[2],
        bandwidth: info[3] as f64,
        max_gain: info[6],
        rates: (info[10]..=info[4])
            .rev()
            .map(|d| (d, info[2] >> d))
            .filter(|&(_, r)| r >= 12_000)
            .collect(),
    };
    eprintln!(
        "{}: device type {}, {} S/s max, band {:.0} Hz, control {}, device centre {:.0}",
        args.server, dev.kind, dev.max_rate, dev.bandwidth, sync.can_control, sync.device_hz
    );
    if sync.can_control && !args.tune {
        // Holding control would lock out the operator's client (SDR#):
        // control goes to whoever connects first, and the protocol has no
        // way to hand it on. Leave, and come back as a guest.
        return Err(std::io::Error::other(
            "got control of the device; leaving it for the operator's client \
             (start SDR# first, or pass --tune to run alone)",
        ));
    }
    let tune = args.tune && sync.can_control;

    loop {
        let Some(p) = plan(args, &dev, sync.device_hz, tune) else {
            eprintln!(
                "no channel fits the band around {:.0} Hz; waiting for the device to move",
                sync.device_hz
            );
            sync = wait_for_move(&mut c, sync)?;
            continue;
        };
        let got = apply(&mut c, &dev, &p, args, tune)?;
        if got.iq_hz != p.center {
            eprintln!(
                "asked for an IQ centre of {:.0} Hz, the server set {:.0}; planning again",
                p.center, got.iq_hz
            );
            sync = got;
            continue;
        }
        sync = got;
        eprintln!(
            "IQ {} S/s (decimation {}) centre {:.0} Hz, span {:.0}..{:.0}; device centre {:.0}",
            p.rate,
            p.decim,
            p.center,
            p.center - p.rate as f64 / 2.0,
            p.center + p.rate as f64 / 2.0,
            sync.device_hz
        );
        for (i, (_, _, spec)) in args.channels.iter().enumerate() {
            let state = if p.channels.contains(&i) {
                ""
            } else {
                " (paused: outside the band)"
            };
            eprintln!("  channel {spec}{state}");
        }
        c.set(SET_STREAMING_ENABLED, 1)?;
        match stream(&mut c, args, &p, sync)? {
            Stop::Moved(s) => {
                eprintln!(
                    "device centre {:.0} -> {:.0} Hz, IQ centre {:.0} -> {:.0}",
                    sync.device_hz, s.device_hz, sync.iq_hz, s.iq_hz
                );
                sync = s;
            }
        }
    }
}

/// Streaming off; block until a CLIENT_SYNC shows a different device centre.
fn wait_for_move(c: &mut Conn, was: Sync) -> std::io::Result<Sync> {
    c.set(SET_STREAMING_ENABLED, 0)?;
    loop {
        let m = c.read()?;
        if m.kind == MSG_CLIENT_SYNC {
            let s = Sync::parse(&m.body);
            if s.device_hz != was.device_hz {
                return Ok(s);
            }
        }
    }
}

/// Decode until the server says the device or this client's IQ centre moved.
fn stream(c: &mut Conn, args: &Args, p: &Plan, sync: Sync) -> std::io::Result<Stop> {
    let stream = IqStream {
        sample_rate: p.rate,
        center_hz: p.center,
        // Every message is converted to f32 and pushed typed, so this field
        // is not read.
        format: IqSampleFormat::Cf32,
        iq_swap: args.iq_swap,
    };
    let mut rx = IqReceiver::with_channelizer(stream, args.channelizer)
        .map_err(|e| std::io::Error::other(format!("{} S/s: {e}", p.rate)))?;
    for &i in &p.channels {
        let (mode, dial, _) = &args.channels[i];
        rx.add_channel(*dial, *mode)
            .map_err(|e| std::io::Error::other(format!("{dial}: {e}")))?;
    }
    let (tx, rx_decodes) = mpsc::channel::<IqDecode>();
    rx.on_decode(move |d| {
        let _ = tx.send(d.clone());
    });
    // Decodes print from their own thread so the socket is never held up;
    // it ends when `rx` (holding the sender) is dropped on return.
    std::thread::spawn(move || {
        for d in rx_decodes {
            print_decode(&d);
        }
    });

    let rate = p.rate;
    let mut est = AnchorEstimate {
        window: VecDeque::new(),
    };
    let mut anchor: Option<i64> = None;
    let mut next_seq: Option<u32> = None;
    let mut f32buf = Vec::<f32>::new();
    let mut last_report = 0u64;
    let (mut gaps, mut reanchors) = (0u64, 0u64);
    // Longest single `push` since the last status line: a slot's decode
    // runs inside it, and the reader queue below has to absorb it.
    let mut worst_push = Duration::ZERO;
    loop {
        let m = c.read()?;
        let arrival = m.arrival_ns;
        let n = match m.kind {
            MSG_UINT8_IQ => m.body.len() / 2,
            MSG_INT16_IQ => m.body.len() / 4,
            MSG_INT24_IQ => m.body.len() / 6,
            MSG_FLOAT_IQ => m.body.len() / 8,
            MSG_CLIENT_SYNC => {
                let s = Sync::parse(&m.body);
                if s.device_hz != sync.device_hz || s.iq_hz != sync.iq_hz {
                    return Ok(Stop::Moved(s));
                }
                continue;
            }
            _ => {
                continue;
            }
        };
        // The sequence counts messages; a hole of d messages is taken to be
        // d messages of this one's length.
        if let Some(want) = next_seq {
            let missed = m.seq.wrapping_sub(want);
            if missed != 0 && missed < u32::MAX / 2 {
                gaps += 1;
                eprintln!(
                    "gap: {missed} message(s) lost at {:.1} s",
                    rx.samples_in() as f64 / rate as f64
                );
                rx.gap(missed as u64 * n as u64);
            }
        }
        next_seq = Some(m.seq.wrapping_add(1));

        // Whatever the server sends becomes f32 at full scale 1.0.
        let b = &m.body;
        f32buf.clear();
        match m.kind {
            MSG_UINT8_IQ => f32buf.extend(b.iter().map(|&v| (v as f32 - 128.0) / 128.0)),
            MSG_INT16_IQ => f32buf.extend(
                b.as_chunks::<2>()
                    .0
                    .iter()
                    .map(|&v| i16::from_le_bytes(v) as f32 / 32_768.0),
            ),
            MSG_INT24_IQ => f32buf.extend(
                b.as_chunks::<3>()
                    .0
                    .iter()
                    .map(|&[l, m, h]| (i32::from_le_bytes([0, l, m, h]) >> 8) as f32 / 8_388_608.0),
            ),
            _ => f32buf.extend(b.as_chunks::<4>().0.iter().map(|&v| f32::from_le_bytes(v))),
        }
        let t = std::time::Instant::now();
        rx.push_cf32(&f32buf);
        worst_push = worst_push.max(t.elapsed());

        // The arrival stamps the *end* of this message.
        let samples = rx.samples_in();
        let best = est.push(samples, rate, arrival);
        let warming_up = samples < 2 * rate as u64;
        match anchor {
            Some(a) if !warming_up && (best - a).abs() <= args.reanchor_ns => {}
            Some(a) if !warming_up => {
                reanchors += 1;
                eprintln!("re-anchor: {:+.3} s", (best - a) as f64 * 1e-9);
                rx.set_time_anchor(best);
                anchor = Some(best);
            }
            // The first two seconds settle the minimum; no slot is complete yet.
            _ => {
                if anchor != Some(best) {
                    rx.set_time_anchor(best);
                    anchor = Some(best);
                }
            }
        }
        if samples - last_report >= 60 * rate as u64 {
            last_report = samples;
            let a = anchor.unwrap();
            eprintln!(
                "status: {:.0} s streamed, last delay {:.0} ms, estimate {:+.0} ms from anchor, \
                 longest push {:.0} ms, queue {:.0} kB, {gaps} gap(s), {reanchors} re-anchor(s)",
                samples as f64 / rate as f64,
                (arrival - a) as f64 * 1e-6 - samples as f64 * 1e3 / rate as f64,
                (best - a) as f64 * 1e-6,
                worst_push.as_secs_f64() * 1e3,
                c.queued.load(Ordering::Relaxed) as f64 / 1e3
            );
            worst_push = Duration::ZERO;
        }
    }
}

fn main() -> ExitCode {
    let Some(args) = parse_args() else {
        eprintln!(
            "usage: iq_spyserver --server HOST:PORT --ch MODE@DIAL_HZ [--ch ...]\n\
             \x20      [--tune] [--center HZ] [--rate S/s] [--gain N] [--format float|int16] [--pfb] [--iq-swap] [--reanchor-ms MS]\n\
             modes: FT8 FT4 (+ FST4-60 WSPR JT9 JT65 Q65-60A with their features)"
        );
        return ExitCode::from(2);
    };
    loop {
        if let Err(e) = run(&args) {
            eprintln!("{}: {e}", args.server);
        }
        std::thread::sleep(Duration::from_secs(10));
    }
}
