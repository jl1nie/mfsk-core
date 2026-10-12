// SPDX-License-Identifier: GPL-3.0-only
//! The `rtl_tcp` protocol, client side, as `rtl_tcp` and SDRplay's `rsp_tcp` speak it
//! (`github.com/SDRplay/RSPTCPServer`, `rsp_tcp.c` and `rsp_tcp_api.h`, commit 61b8c91).
//!
//! On connect the server sends `"RTL0"`, the tuner type and the number of gain steps
//! (three big-endian words). `rsp_tcp` started with `-E` then sends a 45-byte capabilities
//! struct that begins `"RSP0"` and says whether the IQ is 8-bit (`sample_format` 1) or
//! 16-bit (2); without `-E` it sends none and the IQ is unsigned 8-bit. After that the
//! server streams interleaved IQ for ever (there is no way to pause it, no timestamps and no
//! decimation) and the client sends 5-byte commands, `u8 command, u32 big-endian
//! parameter`: 0x01 frequency, 0x02 sample rate, 0x03 gain mode (1 manual, 0 automatic),
//! 0x0d gain by index.
//!
//! Measured against an RSP1 on `rsp_tcp` (2026-10-12, #685): a gain command sent just after a
//! frequency command, or on a fresh connection, sometimes did not take (the level stayed at
//! the no-signal floor), so [`Conn::tune`] waits between the two and the gain is written
//! again afterwards.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant, SystemTime};

use mfsk_core::iq::IqSampleFormat;

use crate::now_ns;
use crate::spyserver::{MSG_INT16_IQ, MSG_UINT8_IQ, Message, SLEPT, STALL};

/// What the user asks the sample format to be. The server decides it (with `rsp_tcp`, by
/// `-E -b 16` or not), so a choice other than `Auto` is a check: a server that sends the
/// other format is refused with the reason.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RtlFormat {
    /// Whatever the server sends: 16-bit if it announces it, else 8-bit.
    #[default]
    Auto,
    /// Unsigned 8-bit IQ (plain `rtl_tcp`, or `rsp_tcp` without `-E`).
    U8,
    /// Signed 16-bit IQ (`rsp_tcp -E -b 16`).
    I16,
}

/// `DeviceInfo::kind` of a device reached over `rtl_tcp` (SpyServer's own are 1-3).
pub const DEVICE_RTL_TCP: u32 = 100;

/// The sample rate asked for when none is configured. 1.024 MS/s holds every HF band's FT8 / FT4 /
/// WSPR sub-bands at once, and an `rsp_tcp` on an RSP1 kept up with it (its `queued` stayed 0); at
/// 2.048 MS/s the same server's send queue sat at its ceiling (93-95) throughout, from the first
/// client implementation on (#685, 2026-10-12).
pub const DEFAULT_RATE: u32 = 1_024_000;

/// Time to let a retune settle before the gain is written (and before samples count):
/// without it a gain written at once was not applied (#685).
pub const SETTLE: Duration = Duration::from_millis(600);

const CMD_FREQUENCY: u8 = 0x01;
const CMD_SAMPLE_RATE: u8 = 0x02;
const CMD_GAIN_MODE: u8 = 0x03;
const CMD_GAIN_BY_INDEX: u8 = 0x0d;

/// Size of `rsp_extended_capabilities_t` (packed).
const RSP_CAPS_LEN: usize = 45;
const RSP_MAGIC: &[u8; 4] = b"RSP0";
/// `rsp_tcp_sample_format_t`.
const RSP_FORMAT_U8: u32 = 1;
const RSP_FORMAT_I16: u32 = 2;

/// What the server said about itself on connect.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Hello {
    /// The tuner type (`rtl_tcp`'s `RTLSDR_TUNER_*`; `rsp_tcp` says 5).
    pub tuner: u32,
    /// How many gain steps the server has (`rsp_tcp`: 28).
    pub gains: u32,
    /// The format the IQ comes in.
    pub format: IqSampleFormat,
    /// Whether the server is an `rsp_tcp` in extended mode, and its hardware version.
    pub rsp_hardware: Option<u32>,
}

/// Read the greeting. `want` is the format the user asked for.
pub fn read_hello(r: &mut impl Read, want: RtlFormat) -> std::io::Result<(Hello, Vec<u8>)> {
    let bad = |m: String| std::io::Error::new(std::io::ErrorKind::InvalidData, m);
    let mut h = [0u8; 12];
    r.read_exact(&mut h)?;
    if &h[..4] != b"RTL0" {
        return Err(bad(format!(
            "not an rtl_tcp server (it said {:02x?})",
            &h[..4]
        )));
    }
    let word = |b: &[u8]| u32::from_be_bytes(b.try_into().unwrap());
    let (tuner, gains) = (word(&h[4..8]), word(&h[8..12]));
    // The next four bytes are the capabilities' magic, or the first IQ of a server that
    // sends none. Kept whichever they are.
    let mut first = [0u8; 4];
    r.read_exact(&mut first)?;
    let (format, rsp_hardware, carry) = if &first == RSP_MAGIC {
        let mut rest = [0u8; RSP_CAPS_LEN - 4];
        r.read_exact(&mut rest)?;
        // version, capabilities, reserved, hardware_version, sample_format: big-endian words.
        let hardware = word(&rest[12..16]);
        let format = match word(&rest[16..20]) {
            RSP_FORMAT_I16 => IqSampleFormat::Cs16,
            RSP_FORMAT_U8 => IqSampleFormat::Cu8,
            f => return Err(bad(format!("rsp_tcp announces sample format {f}"))),
        };
        (format, Some(hardware), Vec::new())
    } else {
        (IqSampleFormat::Cu8, None, first.to_vec())
    };
    match (want, format) {
        (RtlFormat::U8, IqSampleFormat::Cs16) => {
            return Err(bad(
                "the server sends 16-bit IQ (rsp_tcp -E -b 16) but 8-bit was chosen: choose Auto or 16-bit"
                    .into(),
            ));
        }
        (RtlFormat::I16, IqSampleFormat::Cu8) => {
            return Err(bad(
                "the server sends 8-bit IQ but 16-bit was chosen: start rsp_tcp with -E -b 16, or choose Auto or 8-bit"
                    .into(),
            ));
        }
        _ => {}
    }
    Ok((
        Hello {
            tuner,
            gains,
            format,
            rsp_hardware,
        },
        carry,
    ))
}

/// Bytes per IQ message, so that one is about 8 ms at 2 Msps.
fn chunk_bytes(format: IqSampleFormat) -> usize {
    match format {
        IqSampleFormat::Cs16 => 65_536,
        _ => 32_768,
    }
}

/// The SpyServer message kind the shared stream loop knows the format by.
fn kind_of(format: IqSampleFormat) -> u32 {
    match format {
        IqSampleFormat::Cs16 => MSG_INT16_IQ,
        _ => MSG_UINT8_IQ,
    }
}

/// A connection read on its own thread (as [`crate::spyserver::Conn`] is, for the same
/// reason: a slot's decode runs inside the stream loop and must not stop the read).
pub struct Conn {
    w: TcpStream,
    msgs: mpsc::Receiver<std::io::Result<Message>>,
    queued: Arc<AtomicUsize>,
    last_rx: Instant,
    stall: Duration,
    tick: (Instant, SystemTime),
    pub hello: Hello,
}

impl Conn {
    pub fn connect(server: &str, want: RtlFormat) -> std::io::Result<Self> {
        let sock = TcpStream::connect(server)?;
        sock.set_nodelay(true)?;
        sock.set_read_timeout(Some(Duration::from_secs(10)))?;
        let mut r = std::io::BufReader::with_capacity(1 << 20, sock.try_clone()?);
        let (hello, carry) = read_hello(&mut r, want)?;
        // After the greeting the stream never pauses: no read timeout, the stall check does it.
        sock.set_read_timeout(None)?;
        let (tx, msgs) = mpsc::channel();
        let queued = Arc::new(AtomicUsize::new(0));
        let q = queued.clone();
        let format = hello.format;
        std::thread::spawn(move || {
            let n = chunk_bytes(format);
            let kind = kind_of(format);
            let mut seq = 0u32;
            let mut carry = carry;
            loop {
                let mut body = vec![0u8; n];
                let have = carry.len();
                body[..have].copy_from_slice(&carry);
                carry.clear();
                let m = r.read_exact(&mut body[have..]).map(|()| {
                    q.fetch_add(body.len(), Ordering::Relaxed);
                    seq = seq.wrapping_add(1);
                    Message {
                        kind,
                        seq,
                        body,
                        arrival_ns: now_ns(),
                    }
                });
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
            last_rx: Instant::now(),
            stall: STALL,
            tick: (Instant::now(), SystemTime::now()),
            hello,
        })
    }

    /// One 5-byte command.
    pub fn command(&mut self, cmd: u8, param: u32) -> std::io::Result<()> {
        let mut b = [0u8; 5];
        b[0] = cmd;
        b[1..].copy_from_slice(&param.to_be_bytes());
        self.w.write_all(&b)
    }

    pub fn set_sample_rate(&mut self, hz: u32) -> std::io::Result<()> {
        self.command(CMD_SAMPLE_RATE, hz)
    }

    /// Set the centre frequency, wait for it to settle ([`SETTLE`]), then write the gain
    /// again: `gain` is a gain step (manual), `None` the server's automatic gain.
    pub fn tune(&mut self, hz: u32, gain: Option<u32>, stop: &AtomicBool) -> std::io::Result<()> {
        self.command(CMD_FREQUENCY, hz)?;
        let until = Instant::now() + SETTLE;
        while Instant::now() < until {
            if stop.load(Ordering::Relaxed) {
                return Err(std::io::ErrorKind::Interrupted.into());
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        self.set_gain(gain)
    }

    /// Manual gain step `Some(i)` (clamped to what the server has), or automatic `None`.
    pub fn set_gain(&mut self, gain: Option<u32>) -> std::io::Result<()> {
        match gain {
            Some(g) => {
                self.command(CMD_GAIN_MODE, 1)?;
                self.command(CMD_GAIN_BY_INDEX, g.min(self.hello.gains.saturating_sub(1)))
            }
            None => self.command(CMD_GAIN_MODE, 0),
        }
    }

    pub fn set_stall(&mut self, stall: Duration) {
        self.stall = stall;
    }

    /// The next message; `Interrupted` once `stop` is set, `TimedOut` after the stall time,
    /// or on waking from sleep (as [`crate::spyserver::Conn::read`]).
    pub fn read(&mut self, stop: &AtomicBool) -> std::io::Result<Message> {
        loop {
            if stop.load(Ordering::Relaxed) {
                return Err(std::io::ErrorKind::Interrupted.into());
            }
            let (now, wall) = (Instant::now(), SystemTime::now());
            let gone = crate::spyserver::slept(
                now.duration_since(self.tick.0),
                wall.duration_since(self.tick.1).ok(),
            );
            self.tick = (now, wall);
            if gone {
                return Err(std::io::Error::new(std::io::ErrorKind::TimedOut, SLEPT));
            }
            match self.msgs.recv_timeout(Duration::from_millis(200)) {
                Ok(m) => {
                    let m = m?;
                    self.queued.fetch_sub(m.body.len(), Ordering::Relaxed);
                    self.last_rx = Instant::now();
                    return Ok(m);
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if self.last_rx.elapsed() >= self.stall {
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::TimedOut,
                            format!("nothing from the server for {} s", self.stall.as_secs_f32()),
                        ));
                    }
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    return Err(std::io::Error::other("reader thread ended"));
                }
            }
        }
    }

    pub fn queued_bytes(&self) -> usize {
        self.queued.load(Ordering::Relaxed)
    }
}

impl Drop for Conn {
    fn drop(&mut self) {
        let _ = self.w.shutdown(std::net::Shutdown::Both);
    }
}

/// The sample rate a server really delivers, measured: `rtl_tcp` has no field for it, and an RSP's
/// was 1.7 % under the one asked for (1.006 MS/s for 1.024, `rsp_tcp` debug statistics, 2026-10-12):
/// 1 s of time in a minute, which the receiver took for a clock step each time (#685). A least-squares
/// line through (arrival time, samples so far) per message; the slope is the rate.
#[derive(Clone, Debug, Default)]
pub struct RateMeter {
    t0: Option<i64>,
    n: f64,
    sx: f64,
    sy: f64,
    sxx: f64,
    sxy: f64,
}

impl RateMeter {
    pub fn new() -> Self {
        Self::default()
    }

    /// A message arrived at `arrival_ns` and brought the stream to `samples` in all.
    pub fn push(&mut self, arrival_ns: i64, samples: u64) {
        let t0 = *self.t0.get_or_insert(arrival_ns);
        let (x, y) = ((arrival_ns - t0) as f64 * 1e-9, samples as f64);
        self.n += 1.0;
        self.sx += x;
        self.sy += y;
        self.sxx += x * x;
        self.sxy += x * y;
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0.0
    }

    /// Samples per second, once the points span something worth a line.
    pub fn rate(&self) -> Option<f64> {
        let d = self.n * self.sxx - self.sx * self.sx;
        // Needs a spread of times, not only many points: 1 s of variance-sum per point squared.
        if self.n < 10.0 || d <= 0.0 {
            return None;
        }
        Some((self.n * self.sxy - self.sx * self.sy) / d)
    }
}

/// Wait until the server has applied what was asked of it. `rtl_tcp` does not answer commands, and
/// `rsp_tcp` does not even read them until its `Init` of the RSP is over (10-15 s on an RSP1, the commands
/// waiting in the socket), while it streams already, at the rate and frequency it was started with: that
/// stream, taken for the asked-for rate, runs ahead of the clock (a row of `re-anchor: -1.0 s` in the
/// first minute, #685). The only sign is the byte rate: windows of 2 s on arrival times, until two in a
/// row are within 20 % of the one the asked-for rate makes. Gives up after `timeout` (and goes on);
/// returns how long it waited.
pub fn await_rate(
    c: &mut Conn,
    stop: &AtomicBool,
    nominal: u32,
    timeout: Duration,
) -> std::io::Result<Duration> {
    let bps = c.hello.format.bytes_per_sample() as f64;
    let want = f64::from(nominal) * bps;
    let start = Instant::now();
    let (mut t_first, mut bytes, mut good) = (None::<i64>, 0usize, 0);
    while start.elapsed() < timeout {
        let m = c.read(stop)?;
        let t0 = *t_first.get_or_insert(m.arrival_ns);
        bytes += m.body.len();
        let elapsed = (m.arrival_ns - t0) as f64 * 1e-9;
        if elapsed >= 2.0 {
            let rate = bytes as f64 / elapsed;
            if (rate / want - 1.0).abs() < 0.2 {
                good += 1;
                if good >= 2 {
                    break;
                }
            } else {
                good = 0;
            }
            (t_first, bytes) = (None, 0);
        }
    }
    Ok(start.elapsed())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn caps(format: u32) -> Vec<u8> {
        let mut c = Vec::new();
        c.extend_from_slice(b"RSP0");
        for w in [1u32, 0x8f, 0, 1, format] {
            c.extend_from_slice(&w.to_be_bytes());
        }
        c.resize(RSP_CAPS_LEN, 0);
        c
    }

    fn rtl0(tuner: u32, gains: u32) -> Vec<u8> {
        let mut h = b"RTL0".to_vec();
        h.extend_from_slice(&tuner.to_be_bytes());
        h.extend_from_slice(&gains.to_be_bytes());
        h
    }

    /// `rsp_tcp -E -b 16`: the capabilities struct follows the greeting and says 16-bit.
    #[test]
    fn extended_mode_announces_16_bit() {
        let mut wire = rtl0(5, 28);
        wire.extend(caps(RSP_FORMAT_I16));
        wire.extend_from_slice(&[9, 9, 9, 9]);
        let mut r = wire.as_slice();
        let (h, carry) = read_hello(&mut r, RtlFormat::Auto).unwrap();
        assert_eq!(
            (h.tuner, h.gains, h.format, h.rsp_hardware),
            (5, 28, IqSampleFormat::Cs16, Some(1))
        );
        assert!(carry.is_empty());
        assert_eq!(r, &[9, 9, 9, 9], "the IQ starts right after the struct");
    }

    /// Plain `rtl_tcp`, or `rsp_tcp` without `-E`: no struct, and the four bytes read to look
    /// for one are the first IQ, carried over.
    #[test]
    fn standard_mode_is_8_bit_and_keeps_the_first_bytes() {
        let mut wire = rtl0(5, 28);
        wire.extend_from_slice(&[127, 128, 126, 129, 120]);
        let (h, carry) = read_hello(&mut wire.as_slice(), RtlFormat::Auto).unwrap();
        assert_eq!((h.format, h.rsp_hardware), (IqSampleFormat::Cu8, None));
        assert_eq!(carry, vec![127, 128, 126, 129]);
    }

    /// An explicit choice that the server does not match is refused, with the way out.
    #[test]
    fn a_wrong_choice_is_refused_with_the_reason() {
        let mut wire16 = rtl0(5, 28);
        wire16.extend(caps(RSP_FORMAT_I16));
        let e = read_hello(&mut wire16.as_slice(), RtlFormat::U8).unwrap_err();
        assert!(e.to_string().contains("16-bit"), "{e}");
        let mut wire8 = rtl0(5, 28);
        wire8.extend_from_slice(&[1, 2, 3, 4]);
        let e = read_hello(&mut wire8.as_slice(), RtlFormat::I16).unwrap_err();
        assert!(e.to_string().contains("-E -b 16"), "{e}");
        // A matching choice passes.
        assert!(read_hello(&mut wire16.as_slice(), RtlFormat::I16).is_ok());
        assert!(read_hello(&mut wire8.as_slice(), RtlFormat::U8).is_ok());
    }

    #[test]
    fn something_else_is_not_an_rtl_tcp_server() {
        let e =
            read_hello(&mut b"SPYS\0\0\0\0\0\0\0\0xxxx".as_slice(), RtlFormat::Auto).unwrap_err();
        assert!(e.to_string().contains("not an rtl_tcp"), "{e}");
    }

    /// A server that sends the greeting, a struct, and IQ: the client yields whole messages of
    /// the announced format, in order, and the commands it writes are 5 bytes each.
    #[test]
    fn messages_and_commands_over_a_socket() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let server = std::thread::spawn(move || {
            let mut s = l.accept().unwrap().0;
            let mut hello = rtl0(5, 28);
            hello.extend(caps(RSP_FORMAT_I16));
            s.write_all(&hello).unwrap();
            let iq: Vec<u8> = (0..2 * chunk_bytes(IqSampleFormat::Cs16))
                .map(|i| (i % 251) as u8)
                .collect();
            s.write_all(&iq).unwrap();
            // Read the five commands the client sends for a tune with a manual gain.
            let mut got = [0u8; 15];
            s.read_exact(&mut got).unwrap();
            got
        });
        let mut c = Conn::connect(&addr, RtlFormat::Auto).unwrap();
        assert_eq!(c.hello.format, IqSampleFormat::Cs16);
        let stop = AtomicBool::new(false);
        let (a, b) = (c.read(&stop).unwrap(), c.read(&stop).unwrap());
        assert_eq!((a.kind, a.seq, a.body.len()), (MSG_INT16_IQ, 1, 65_536));
        assert_eq!((b.seq, b.body[0]), (2, (65_536 % 251) as u8));
        c.tune(7_074_000, Some(40), &stop).unwrap();
        let got = server.join().unwrap();
        // frequency, then (after the settle) gain mode 1 and the gain index, clamped to 27.
        assert_eq!(&got[..5], &[0x01, 0, 0x6b, 0xf0, 0xd0]);
        assert_eq!(&got[5..10], &[0x03, 0, 0, 0, 1]);
        assert_eq!(&got[10..], &[0x0d, 0, 0, 0, 27]);
    }

    /// A line through noisy arrival times finds the rate to a fraction of a per cent, and a server
    /// that delivers 1.7 % short is not taken for its nominal rate.
    #[test]
    fn the_meter_finds_the_real_rate_through_jitter() {
        let real = 1_006_400.0_f64;
        let per = 16_384u64;
        let mut m = RateMeter::new();
        let mut rng = 12345u32;
        for k in 1..=900u64 {
            // Arrival time of the k-th message, +- 15 ms of jitter.
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let jitter = ((rng >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 0.030;
            let t = (k * per) as f64 / real + jitter;
            m.push((t * 1e9) as i64, k * per);
        }
        let r = m.rate().unwrap();
        assert!((r / real - 1.0).abs() < 0.001, "{r}");
        assert!(RateMeter::new().rate().is_none());
    }
}
