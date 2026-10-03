// SPDX-License-Identifier: GPL-3.0-or-later
//! Decode FT8 / FT4 / … straight from a SpyServer (Airspy HF+, Airspy,
//! RTL-SDR behind `spyserver`), one `IqReceiver` channel per dial.
//!
//! ```text
//! cargo run --release --features full --example iq_spyserver -- \
//!     --server 192.168.1.26:5555 --ch FT8@7041000 --ch FT8@7074000 --ch FT4@7047500
//! ```
//!
//! `--center` and `--rate` are chosen when not given: the centre 25 kHz
//! below the lowest dial (so the SDR's DC spike is clear of every channel)
//! and the lowest rate the device offers that holds every channel, by the
//! receiver's own placement rule. `--gain N` sets the device gain index.
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
use std::sync::mpsc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mfsk_core::iq::{Channelizer, IqDecode, IqMode, IqReceiver, IqSampleFormat, IqStream};

const PROTOCOL_VERSION: u32 = (2 << 24) | 1700;
const CMD_HELLO: u32 = 0;
const CMD_SET_SETTING: u32 = 2;
const SET_STREAMING_MODE: u32 = 0;
const SET_STREAMING_ENABLED: u32 = 1;
const SET_GAIN: u32 = 2;
const SET_IQ_FORMAT: u32 = 100;
const SET_IQ_FREQUENCY: u32 = 101;
const SET_IQ_DECIMATION: u32 = 102;
const SET_IQ_DIGITAL_GAIN: u32 = 103;
const STREAM_MODE_IQ_ONLY: u32 = 1;
const FORMAT_INT16: u32 = 2;
const MSG_DEVICE_INFO: u32 = 0;
const MSG_CLIENT_SYNC: u32 = 1;
const MSG_UINT8_IQ: u32 = 100;
const MSG_INT16_IQ: u32 = 101;
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

struct Conn {
    r: BufReader<TcpStream>,
    w: TcpStream,
}

struct Message {
    kind: u32,
    gain_db: u32,
    seq: u32,
    body: Vec<u8>,
}

impl Conn {
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

    fn read(&mut self, body: &mut Vec<u8>) -> std::io::Result<Message> {
        let mut h = [0u8; 20];
        self.r.read_exact(&mut h)?;
        let f = |i: usize| u32::from_le_bytes(h[4 * i..4 * i + 4].try_into().unwrap());
        let size = f(4) as usize;
        if size > MAX_BODY {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("message body of {size} bytes"),
            ));
        }
        body.resize(size, 0);
        self.r.read_exact(body)?;
        Ok(Message {
            kind: f(1) & 0xffff,
            gain_db: f(1) >> 16,
            seq: f(3),
            body: std::mem::take(body),
        })
    }
}

fn words(b: &[u8]) -> Vec<u32> {
    b.as_chunks::<4>()
        .0
        .iter()
        .map(|&w| u32::from_le_bytes(w))
        .collect()
}

/// A receiver with every channel placed, or why it could not be.
fn build(args: &Args, rate: u32, center: f64) -> Result<IqReceiver, String> {
    let stream = IqStream {
        sample_rate: rate,
        center_hz: center,
        format: IqSampleFormat::Cs16,
        iq_swap: args.iq_swap,
    };
    let mut rx = IqReceiver::with_channelizer(stream, args.channelizer)
        .map_err(|e| format!("{rate} S/s: {e}"))?;
    for (mode, dial, spec) in &args.channels {
        rx.add_channel(*dial, *mode)
            .map_err(|e| format!("{spec} at {rate} S/s centre {center:.0}: {e}"))?;
    }
    Ok(rx)
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

fn run(args: &Args) -> std::io::Result<()> {
    let sock = TcpStream::connect(&args.server)?;
    sock.set_nodelay(true)?;
    let mut c = Conn {
        r: BufReader::with_capacity(1 << 20, sock.try_clone()?),
        w: sock,
    };
    let mut hello = PROTOCOL_VERSION.to_le_bytes().to_vec();
    hello.extend_from_slice(b"mfsk-core iq_spyserver");
    c.command(CMD_HELLO, &hello)?;

    let mut scratch = Vec::new();
    let (mut info, mut sync) = (None, None);
    while info.is_none() || sync.is_none() {
        let m = c.read(&mut scratch)?;
        match m.kind {
            MSG_DEVICE_INFO => info = Some(words(&m.body)),
            MSG_CLIENT_SYNC => sync = Some(words(&m.body)),
            _ => {}
        }
        scratch = m.body;
    }
    let (info, sync) = (info.unwrap(), sync.unwrap());
    let (dev_type, max_rate, stages, min_decim) = (info[0], info[2], info[4], info[10]);
    let (can_control, iq_center) = (sync[0] != 0, sync[3] as f64);
    eprintln!(
        "{}: device type {dev_type}, {max_rate} S/s max, decimation {min_decim}..={stages}, \
         control {can_control}, IQ centre {iq_center:.0}",
        args.server
    );

    // A centre we may not set is the one we get.
    let lowest = args.channels.iter().map(|c| c.1).fold(f64::MAX, f64::min);
    let center = if can_control {
        args.center.unwrap_or((lowest - 25_000.0).round())
    } else {
        iq_center
    };
    let rates: Vec<(u32, u32)> = (min_decim..=stages)
        .map(|d| (d, max_rate >> d))
        .filter(|&(_, r)| r >= 12_000)
        .collect();
    let mut chosen = None;
    let mut why = Vec::new();
    // Highest decimation (lowest rate) first: the cheapest that holds them all.
    for &(d, r) in rates.iter().rev() {
        if args.rate.is_some_and(|want| want != r) {
            continue;
        }
        match build(args, r, center) {
            Ok(rx) => {
                chosen = Some((d, r, rx));
                break;
            }
            Err(e) => why.push(e),
        }
    }
    let Some((decim, rate, mut rx)) = chosen else {
        for e in why {
            eprintln!("  {e}");
        }
        return Err(std::io::Error::other("no rate holds every channel"));
    };
    eprintln!(
        "IQ {rate} S/s (decimation {decim}) centre {center:.0} Hz, span {:.0}..{:.0}",
        center - rate as f64 / 2.0,
        center + rate as f64 / 2.0
    );
    for (_, _, spec) in &args.channels {
        eprintln!("  channel {spec}");
    }

    c.set(SET_IQ_FORMAT, FORMAT_INT16)?;
    c.set(SET_IQ_DECIMATION, decim)?;
    if can_control {
        c.set(SET_IQ_FREQUENCY, center as u32)?;
        if let Some(g) = args.gain {
            c.set(SET_GAIN, g)?;
        }
    }
    c.set(SET_STREAMING_MODE, STREAM_MODE_IQ_ONLY)?;
    // As SDR++ does: 3 dB of digital gain per decimation stage keeps the
    // level in 16 bits (`computeDigitalGain`); the Airspy One also makes up
    // its device gain. The applied gain comes back in each message's flags.
    let digital = if dev_type == DEVICE_AIRSPY_ONE {
        let (max_gain, gain) = (info[6], args.gain.unwrap_or(sync[1]));
        max_gain.saturating_sub(gain) + (decim as f32 * 3.01) as u32
    } else {
        (decim as f32 * 3.01) as u32
    };
    c.set(SET_IQ_DIGITAL_GAIN, digital)?;
    c.set(SET_STREAMING_ENABLED, 1)?;

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

    let mut est = AnchorEstimate {
        window: VecDeque::new(),
    };
    let mut anchor: Option<i64> = None;
    let mut cur_center = center;
    let mut next_seq: Option<u32> = None;
    // The server answers each setting with a CLIENT_SYNC, so the first few
    // still carry the old centre, and IQ already in flight was tuned there.
    // Nothing counts until a sync shows the centre we asked for.
    let mut settled = !can_control;
    let mut i16buf = Vec::<i16>::new();
    let mut f32buf = Vec::<f32>::new();
    let mut last_report = 0u64;
    let (mut gaps, mut reanchors) = (0u64, 0u64);
    loop {
        let m = c.read(&mut scratch)?;
        let arrival = now_ns();
        let n = match m.kind {
            MSG_INT16_IQ => m.body.len() / 4,
            MSG_UINT8_IQ => m.body.len() / 2,
            MSG_FLOAT_IQ => m.body.len() / 8,
            MSG_CLIENT_SYNC => {
                let s = words(&m.body);
                let new_center = s[3] as f64;
                if !settled {
                    settled = new_center == cur_center;
                } else if new_center != cur_center {
                    eprintln!("retune by the server: {cur_center:.0} -> {new_center:.0} Hz");
                    match rx.retune(new_center) {
                        Ok(()) => cur_center = new_center,
                        Err(e) => {
                            return Err(std::io::Error::other(format!(
                                "a channel no longer fits at {new_center:.0} Hz: {e}"
                            )));
                        }
                    }
                }
                scratch = m.body;
                continue;
            }
            _ => {
                scratch = m.body;
                continue;
            }
        };
        if !settled {
            scratch = m.body;
            continue;
        }
        // The sequence counts messages; a hole of d messages is taken to be
        // d messages of this one's length.
        if let Some(want) = next_seq {
            let missed = m.seq.wrapping_sub(want);
            if missed != 0 && missed < u32::MAX / 2 {
                gaps += 1;
                eprintln!("gap: {missed} message(s) lost");
                rx.gap(missed as u64 * n as u64);
            }
        }
        next_seq = Some(m.seq.wrapping_add(1));

        match m.kind {
            MSG_INT16_IQ => {
                i16buf.clear();
                i16buf.extend(
                    m.body
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .map(|&b| i16::from_le_bytes(b)),
                );
                rx.push_cs16(&i16buf);
            }
            MSG_UINT8_IQ => {
                i16buf.clear();
                i16buf.extend(m.body.iter().map(|&b| (b as i16 - 128) << 8));
                rx.push_cs16(&i16buf);
            }
            _ => {
                f32buf.clear();
                f32buf.extend(
                    m.body
                        .as_chunks::<4>()
                        .0
                        .iter()
                        .map(|&b| f32::from_le_bytes(b)),
                );
                rx.push_cf32(&f32buf);
            }
        }
        let _ = m.gain_db; // a constant scale; each slot is normalised anyway
        scratch = m.body;

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
                 {gaps} gap(s), {reanchors} re-anchor(s)",
                samples as f64 / rate as f64,
                (arrival - a) as f64 * 1e-6 - samples as f64 * 1e3 / rate as f64,
                (best - a) as f64 * 1e-6
            );
        }
    }
}

fn main() -> ExitCode {
    let Some(args) = parse_args() else {
        eprintln!(
            "usage: iq_spyserver --server HOST:PORT --ch MODE@DIAL_HZ [--ch ...]\n\
             \x20      [--center HZ] [--rate S/s] [--gain N] [--pfb] [--iq-swap] [--reanchor-ms MS]\n\
             modes: FT8 FT4 (+ FST4-60 WSPR JT9 JT65 Q65-60A with their features)"
        );
        return ExitCode::from(2);
    };
    loop {
        if let Err(e) = run(&args) {
            eprintln!("{}: {e}", args.server);
        }
        std::thread::sleep(Duration::from_secs(3));
    }
}
