// SPDX-License-Identifier: GPL-3.0-or-later
//! SpyServer's wire protocol, client side, as SDR++ implements it
//! (`source_modules/spyserver_source/src/spyserver_protocol.h`,
//! `spyserver_client.cpp`): HELLO, then DEVICE_INFO and CLIENT_SYNC from the
//! server, then SET_SETTINGs and IQ messages of `[u32 protocol, u32 type |
//! gain_db << 16, u32 stream, u32 sequence, u32 body size]` + body.

use std::io::{BufReader, Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use mfsk_core::iq::IqSampleFormat;

use crate::now_ns;

pub const PROTOCOL_VERSION: u32 = (2 << 24) | 1700;
pub const CMD_HELLO: u32 = 0;
pub const CMD_SET_SETTING: u32 = 2;
pub const CMD_PING: u32 = 3;
pub const SET_STREAMING_MODE: u32 = 0;
pub const SET_STREAMING_ENABLED: u32 = 1;
pub const SET_GAIN: u32 = 2;
pub const SET_IQ_FORMAT: u32 = 100;
pub const SET_IQ_FREQUENCY: u32 = 101;
pub const SET_IQ_DECIMATION: u32 = 102;
pub const SET_IQ_DIGITAL_GAIN: u32 = 103;
pub const STREAM_MODE_IQ_ONLY: u32 = 1;
pub const FORMAT_INT16: u32 = 2;
pub const FORMAT_FLOAT: u32 = 4;
pub const MSG_DEVICE_INFO: u32 = 0;
pub const MSG_CLIENT_SYNC: u32 = 1;
pub const MSG_PONG: u32 = 2;
pub const MSG_UINT8_IQ: u32 = 100;
pub const MSG_INT16_IQ: u32 = 101;
pub const MSG_INT24_IQ: u32 = 102;
pub const MSG_FLOAT_IQ: u32 = 103;
pub const DEVICE_AIRSPY_ONE: u32 = 1;
/// The protocol's own bound on a message body (`SPYSERVER_MAX_MESSAGE_BODY_SIZE`).
pub const MAX_BODY: usize = 1 << 20;

pub struct Message {
    pub kind: u32,
    pub seq: u32,
    pub body: Vec<u8>,
    /// When the reader thread had the whole message, ns since the epoch:
    /// the time anchor must not include the time it then waited in the queue.
    pub arrival_ns: i64,
}

/// One message off the wire.
pub fn read_message(r: &mut impl Read) -> std::io::Result<Message> {
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

/// A connection whose socket is read on its own thread, so a slot's decode
/// (which runs inside `push`) never stops the read. It did: the first
/// decode takes 400-650 ms (later ones under 20 ms), the server's queue for
/// this client overflowed meanwhile, and on Windows it dropped 1-13 IQ
/// messages 20-31 s into every run (3 of 3). Linux's larger receive buffer
/// absorbed the same stall, so WSL runs never showed it.
pub struct Conn {
    w: TcpStream,
    msgs: mpsc::Receiver<std::io::Result<Message>>,
    /// Body bytes read but not yet taken by the decoder.
    queued: Arc<AtomicUsize>,
    /// When [`Self::read`] last got a message, for the stall check.
    last_rx: Instant,
    stall: Duration,
}

/// Silence after which [`Conn::read`] takes the connection for dead.
///
/// A peer that goes away without a FIN or RST leaves the socket
/// ESTABLISHED, and this client only writes when it changes a setting, so
/// nothing ever fails: after a Mac slept for 32 s the skimmer sat
/// "connected" with its byte count frozen at 2 180 884 016, never
/// reconnecting. While streaming, IQ arrives many times a second; when
/// streaming is off, [`PING_EVERY`] keeps PONGs coming. 15 s is well clear
/// of both and still reconnects within half a minute. `Instant` does not
/// advance while macOS sleeps, so the time asleep is not counted.
pub const STALL: Duration = Duration::from_secs(15);

/// How often a connection with streaming off is pinged, so that
/// [`STALL`] holds there too.
pub const PING_EVERY: Duration = Duration::from_secs(5);

/// `Conn::read` gave up because the caller asked to stop.
pub fn is_stop(e: &std::io::Error) -> bool {
    e.kind() == std::io::ErrorKind::Interrupted
}

impl Conn {
    pub fn connect(server: &str) -> std::io::Result<Self> {
        let sock = TcpStream::connect(server)?;
        sock.set_nodelay(true)?;
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
            last_rx: Instant::now(),
            stall: STALL,
        })
    }

    /// Replace [`STALL`] for this connection.
    pub fn set_stall(&mut self, stall: Duration) {
        self.stall = stall;
    }

    pub fn command(&mut self, cmd: u32, body: &[u8]) -> std::io::Result<()> {
        let mut b = Vec::with_capacity(8 + body.len());
        b.extend_from_slice(&cmd.to_le_bytes());
        b.extend_from_slice(&(body.len() as u32).to_le_bytes());
        b.extend_from_slice(body);
        self.w.write_all(&b)
    }

    pub fn set(&mut self, setting: u32, value: u32) -> std::io::Result<()> {
        let mut b = [0u8; 8];
        b[..4].copy_from_slice(&setting.to_le_bytes());
        b[4..].copy_from_slice(&value.to_le_bytes());
        self.command(CMD_SET_SETTING, &b)
    }

    /// The next message; `Interrupted` (see [`is_stop`]) once `stop` is set,
    /// `TimedOut` once nothing has arrived for the stall time ([`STALL`]).
    pub fn read(&mut self, stop: &AtomicBool) -> std::io::Result<Message> {
        loop {
            if stop.load(Ordering::Relaxed) {
                return Err(std::io::ErrorKind::Interrupted.into());
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
    /// Ends the reader thread: its blocking read fails once the socket is shut.
    fn drop(&mut self) {
        let _ = self.w.shutdown(std::net::Shutdown::Both);
    }
}

pub fn words(b: &[u8]) -> Vec<u32> {
    b.as_chunks::<4>()
        .0
        .iter()
        .map(|&w| u32::from_le_bytes(w))
        .collect()
}

/// What the server's CLIENT_SYNC says, as far as this client cares.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sync {
    pub can_control: bool,
    /// The device's current gain index (the second word of `CLIENT_SYNC`).
    pub gain: u32,
    pub device_hz: f64,
    pub iq_hz: f64,
}

impl Sync {
    pub fn parse(body: &[u8]) -> Self {
        let w = words(body);
        Sync {
            can_control: w[0] != 0,
            gain: w[1],
            device_hz: w[2] as f64,
            iq_hz: w[3] as f64,
        }
    }
}

/// DEVICE_INFO, as far as this client cares.
#[derive(Clone, Debug)]
pub struct Device {
    pub kind: u32,
    pub max_rate: u32,
    /// Usable band around the device centre, Hz (`MaximumBandwidth`).
    pub bandwidth_hz: f64,
    pub max_gain: u32,
    /// `(decimation, rate)`, lowest rate first, 12 kS/s and up.
    pub rates: Vec<(u32, u32)>,
}

impl Device {
    pub fn parse(body: &[u8]) -> Self {
        let w = words(body);
        Device {
            kind: w[0],
            max_rate: w[2],
            bandwidth_hz: w[3] as f64,
            max_gain: w[6],
            rates: (w[10]..=w[4])
                .rev()
                .map(|d| (d, w[2] >> d))
                .filter(|&(_, r)| r >= 12_000)
                .collect(),
        }
    }
}

/// The library's name for an IQ message's sample layout; `None` for a
/// message that is not IQ. `IqReceiver::push_bytes` converts each of these
/// to f32 at full scale 1.0, so this client never touches samples itself.
pub fn sample_format(kind: u32) -> Option<IqSampleFormat> {
    Some(match kind {
        MSG_UINT8_IQ => IqSampleFormat::Cu8,
        MSG_INT16_IQ => IqSampleFormat::Cs16,
        MSG_INT24_IQ => IqSampleFormat::Cs24,
        MSG_FLOAT_IQ => IqSampleFormat::Cf32,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_header_and_body_round_trip() {
        let mut wire = Vec::new();
        for v in [0x0200_0782u32, MSG_INT16_IQ | (6 << 16), 1, 41, 8] {
            wire.extend_from_slice(&v.to_le_bytes());
        }
        wire.extend_from_slice(&[1, 0, 2, 0, 3, 0, 4, 0]);
        let m = read_message(&mut wire.as_slice()).unwrap();
        // The gain in the upper bits is not part of the type.
        assert_eq!((m.kind, m.seq, m.body.len()), (MSG_INT16_IQ, 41, 8));
    }

    #[test]
    fn oversized_body_is_refused() {
        let mut wire = Vec::new();
        for v in [0u32, MSG_FLOAT_IQ, 1, 0, (MAX_BODY + 1) as u32] {
            wire.extend_from_slice(&v.to_le_bytes());
        }
        assert!(read_message(&mut wire.as_slice()).is_err());
    }

    /// SpyServer's four IQ messages are the library's Cu8 / Cs16 / Cs24 /
    /// Cf32 (uint8 is offset binary, like an RTL-SDR's).
    #[test]
    fn message_kinds_map_to_sample_formats() {
        assert_eq!(sample_format(MSG_UINT8_IQ), Some(IqSampleFormat::Cu8));
        assert_eq!(sample_format(MSG_INT16_IQ), Some(IqSampleFormat::Cs16));
        assert_eq!(sample_format(MSG_INT24_IQ), Some(IqSampleFormat::Cs24));
        assert_eq!(sample_format(MSG_FLOAT_IQ), Some(IqSampleFormat::Cf32));
        assert_eq!(sample_format(MSG_CLIENT_SYNC), None);
    }

    /// The Airspy HF+ that this was written against: 912 kS/s, decimation
    /// 0..=8, 780 kHz of band.
    #[test]
    fn device_rates_lowest_first() {
        let mut b = Vec::new();
        for v in [
            2u32,
            0,
            912_000,
            780_000,
            8,
            0,
            8,
            0,
            1_700_000_000,
            16,
            0,
            0,
        ] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        let d = Device::parse(&b);
        assert_eq!(d.rates.first(), Some(&(6, 14_250)));
        assert_eq!(d.rates.last(), Some(&(0, 912_000)));
        assert_eq!(d.bandwidth_hz, 780_000.0);
    }

    /// A server that accepts and then goes quiet without closing, as one did
    /// across a Mac's sleep: `read` must give up rather than wait forever.
    #[test]
    fn a_silent_peer_times_out() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let server = std::thread::spawn(move || l.accept().unwrap().0);
        let mut c = Conn::connect(&addr).unwrap();
        let _held = server.join().unwrap();
        c.set_stall(Duration::from_millis(400));
        let t = Instant::now();
        let Err(e) = c.read(&AtomicBool::new(false)) else {
            panic!("a silent peer sent a message");
        };
        assert_eq!(e.kind(), std::io::ErrorKind::TimedOut);
        assert!(t.elapsed() >= Duration::from_millis(400));
        assert!(!is_stop(&e), "a stall must reconnect, not stop");
    }

    /// Messages spaced inside the stall time keep the connection alive,
    /// however long it runs in total: the check is on the gap, not the age.
    #[test]
    fn messages_within_the_stall_time_keep_it_alive() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let server = std::thread::spawn(move || {
            let mut s = l.accept().unwrap().0;
            for _ in 0..4 {
                std::thread::sleep(Duration::from_millis(150));
                let mut m = Vec::new();
                for v in [PROTOCOL_VERSION, MSG_PONG, 0, 0, 0] {
                    m.extend_from_slice(&v.to_le_bytes());
                }
                s.write_all(&m).unwrap();
            }
            s
        });
        let mut c = Conn::connect(&addr).unwrap();
        c.set_stall(Duration::from_millis(400));
        let stop = AtomicBool::new(false);
        for _ in 0..4 {
            assert_eq!(c.read(&stop).unwrap().kind, MSG_PONG);
        }
        let _held = server.join().unwrap();
    }
}
