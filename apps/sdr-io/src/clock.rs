// SPDX-License-Identifier: GPL-3.0-only
//! The skimmer's own UTC: the PC's clock plus an offset measured against an
//! NTP server (SNTP, RFC 4330), or the PC's clock as it is.
//!
//! The PC clock is not touched (that would need rights and surprise other
//! programs); [`now_ns`] adds the measured offset. Windows' own time service
//! syncs weekly by default and can be several hundred ms out, which moves
//! every channel's DT; a query is 48 bytes and its error is bounded by half
//! the round trip.

use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Seconds from 1900 (NTP) to 1970 (Unix).
const NTP_UNIX: i64 = 2_208_988_800;
/// Queries per measurement; the one with the shortest round trip is kept,
/// since delay only ever adds error.
const QUERIES: usize = 5;
/// A round trip longer than this says nothing about the offset.
const MAX_RTT_NS: i64 = 1_000_000_000;
/// How long a query waits for its reply: a later one would be over
/// [`MAX_RTT_NS`] and thrown away anyway.
const REPLY_WAIT: Duration = Duration::from_secs(1);
/// Addresses tried before giving up. A pool name resolves to several servers,
/// and one that does not answer is passed over for the next rather than asked
/// again: with five queries at 2 s each, a dead pool member held Connect for
/// 10.6 s before the PC clock was used (2026-10-09, `pool.ntp.org` while
/// `ntp.nict.jp` answered in 38 ms). Three at [`REPLY_WAIT`] bounds that at 3 s.
const MAX_ADDRESSES: usize = 3;
/// Between measurements while they work, and after one failed.
const REFRESH: Duration = Duration::from_secs(600);
const RETRY: Duration = Duration::from_secs(60);

static OFFSET_NS: AtomicI64 = AtomicI64::new(0);
static REPORT: Mutex<String> = Mutex::new(String::new());

/// The PC's clock, ns since the epoch.
pub fn system_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64
}

/// The skimmer's UTC, ns since the epoch: the PC's clock plus the NTP offset
/// when one is being kept.
pub fn now_ns() -> i64 {
    system_ns() + OFFSET_NS.load(Ordering::Relaxed)
}

/// One line for the operator: `NTP +83 ms (round trip 24 ms)`, the error, or
/// empty while the PC clock is used.
pub fn report() -> String {
    REPORT.lock().unwrap().clone()
}

fn set_report(s: String) {
    *REPORT.lock().unwrap() = s;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Server time minus PC time, ns.
    pub offset_ns: i64,
    pub rtt_ns: i64,
}

fn ntp_ns(b: &[u8]) -> i64 {
    let secs = u32::from_be_bytes(b[0..4].try_into().unwrap()) as i64;
    let frac = u32::from_be_bytes(b[4..8].try_into().unwrap()) as i128;
    (secs - NTP_UNIX) * 1_000_000_000 + ((frac * 1_000_000_000) >> 32) as i64
}

/// The offset and round trip from one request/reply, with the four
/// timestamps of RFC 4330: sent, server received, server sent, received.
pub fn offset_of(t1: i64, t2: i64, t3: i64, t4: i64) -> Sample {
    Sample {
        offset_ns: ((t2 - t1) + (t3 - t4)) / 2,
        rtt_ns: (t4 - t1) - (t3 - t2),
    }
}

/// The IPv4 addresses `server` (`host` or `host:port`) resolves to, in the
/// resolver's order.
fn resolve(server: &str) -> Result<Vec<SocketAddr>, String> {
    let addr = if server.contains(':') {
        server.to_string()
    } else {
        format!("{server}:123")
    };
    let all: Vec<SocketAddr> = addr
        .to_socket_addrs()
        .map_err(|e| format!("{server}: {e}"))?
        .filter(|a| a.is_ipv4())
        .collect();
    if all.is_empty() {
        return Err(format!("{server}: no IPv4 address"));
    }
    Ok(all)
}

/// One request/reply with the server at `target`.
fn query(target: SocketAddr) -> Result<Sample, String> {
    let sock = UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
    sock.set_read_timeout(Some(REPLY_WAIT))
        .map_err(|e| e.to_string())?;
    sock.connect(target).map_err(|e| e.to_string())?;
    let mut req = [0u8; 48];
    req[0] = 0x23; // leap 0, version 4, mode 3 (client)
    let t1 = system_ns();
    sock.send(&req).map_err(|e| e.to_string())?;
    let mut rep = [0u8; 48];
    let n = sock.recv(&mut rep).map_err(|e| match e.kind() {
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut => "no reply".to_string(),
        _ => e.to_string(),
    })?;
    let t4 = system_ns();
    if n < 48 || rep[0] & 7 != 4 || rep[1] == 0 {
        return Err("not a time reply".into());
    }
    Ok(offset_of(
        t1,
        ntp_ns(&rep[32..40]),
        ntp_ns(&rep[40..48]),
        t4,
    ))
}

/// The best of a few queries.
pub fn measure(server: &str) -> Result<Sample, String> {
    measure_at(&resolve(server)?)
}

/// The best of [`QUERIES`] queries, starting with the first address and
/// moving to the next when one has not answered at all, up to [`MAX_ADDRESSES`]
/// of them. An address that has answered keeps being asked, and a name that
/// resolves to one address (`ntp.nict.jp`) gets one retry: a UDP packet lost
/// on the way is not a dead server, and giving the address up on the first
/// loss would end the measurement.
fn measure_at(addrs: &[SocketAddr]) -> Result<Sample, String> {
    let mut best: Option<Sample> = None;
    let mut err = String::new();
    let mut at = addrs.iter().take(MAX_ADDRESSES);
    let mut target = at.next();
    let (mut asked, mut tried) = (0, usize::from(target.is_some()));
    // Whether the address being asked has answered, and whether the lone
    // address of the name has been given its one retry.
    let (mut heard, mut retried) = (false, false);
    while let Some(&t) = target
        && asked < QUERIES
    {
        if asked > 0 {
            std::thread::sleep(Duration::from_millis(150));
        }
        match query(t) {
            Ok(s) => {
                asked += 1;
                heard = true;
                if s.rtt_ns > MAX_RTT_NS {
                    err = "round trip over 1 s".into();
                } else if best.is_none_or(|b| s.rtt_ns < b.rtt_ns) {
                    best = Some(s);
                }
            }
            // Never answered: the next one, not this again. One that has
            // answered lost a packet, and is asked again; so is the last
            // address of the name once, since with no other to turn to a
            // lost first packet would end the measurement.
            Err(e) => {
                err = e;
                if heard {
                    asked += 1;
                } else if addrs.len() == 1 && !retried {
                    retried = true;
                    asked += 1;
                } else {
                    target = at.next();
                    tried += usize::from(target.is_some());
                }
            }
        }
    }
    best.ok_or_else(|| match tried {
        1 => err,
        n => format!("{err} from {n} addresses"),
    })
}

fn apply(server: &str, r: Result<Sample, String>) -> bool {
    match r {
        Ok(s) => {
            OFFSET_NS.store(s.offset_ns, Ordering::Relaxed);
            let t = system_ns().div_euclid(1_000_000_000).rem_euclid(86_400);
            set_report(format!(
                "NTP {:+.0} ms (round trip {:.0} ms, measured {:02}:{:02} UTC)",
                s.offset_ns as f64 * 1e-6,
                s.rtt_ns as f64 * 1e-6,
                t / 3600,
                t / 60 % 60
            ));
            true
        }
        Err(e) => {
            set_report(format!("NTP {server} failed: {e}; using the PC clock"));
            false
        }
    }
}

/// Keeps the offset to `server` current while it lives; dropping it returns
/// to the PC clock.
pub struct NtpSync {
    stop: Arc<AtomicBool>,
}

impl NtpSync {
    /// Measures once before returning (a few seconds at worst), so the first
    /// IQ message is stamped with the corrected clock, then refreshes in the
    /// background.
    pub fn start(server: &str) -> NtpSync {
        let server = server.trim().to_string();
        let first = apply(&server, measure(&server));
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        std::thread::spawn(move || {
            let mut ok = first;
            loop {
                let wait = if ok { REFRESH } else { RETRY };
                let until = std::time::Instant::now() + wait;
                while std::time::Instant::now() < until {
                    if flag.load(Ordering::Relaxed) {
                        return;
                    }
                    std::thread::sleep(Duration::from_millis(200));
                }
                let r = measure(&server);
                if flag.load(Ordering::Relaxed) {
                    return;
                }
                // A failed refresh keeps the last offset: it was good a
                // few minutes ago and the PC clock drifts slowly.
                ok = match r {
                    Ok(_) => apply(&server, r),
                    Err(e) => {
                        set_report(format!("NTP {server}: {e}; keeping the last offset"));
                        false
                    }
                };
            }
        });
        NtpSync { stop }
    }
}

impl Drop for NtpSync {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        OFFSET_NS.store(0, Ordering::Relaxed);
        set_report(String::new());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A server 500 ms ahead, 20 ms each way, 1 ms to answer.
    #[test]
    fn offset_and_round_trip_from_the_four_stamps() {
        let s = offset_of(0, 520_000_000, 521_000_000, 41_000_000);
        assert_eq!(s.offset_ns, 500_000_000);
        assert_eq!(s.rtt_ns, 40_000_000);
    }

    /// Needs the network: `cargo test -p skimmer-core live_pool -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn live_pool() {
        let s = measure("pool.ntp.org").unwrap();
        println!(
            "offset {:+.1} ms, round trip {:.1} ms",
            s.offset_ns as f64 * 1e-6,
            s.rtt_ns as f64 * 1e-6
        );
        assert!(s.rtt_ns > 0);
    }

    /// A local socket that never answers.
    fn dead() -> (UdpSocket, SocketAddr) {
        let s = UdpSocket::bind("127.0.0.1:0").unwrap();
        let a = s.local_addr().unwrap();
        (s, a)
    }

    /// A local server that answers every request with the PC's time.
    fn alive() -> SocketAddr {
        lossy(0)
    }

    /// As [`alive`], but it drops the first `lost` requests, as a UDP path
    /// does now and then.
    fn lossy(lost: usize) -> SocketAddr {
        let s = UdpSocket::bind("127.0.0.1:0").unwrap();
        let a = s.local_addr().unwrap();
        std::thread::spawn(move || {
            let mut b = [0u8; 48];
            let mut seen = 0;
            while let Ok((_, from)) = s.recv_from(&mut b) {
                seen += 1;
                if seen <= lost {
                    continue;
                }
                let ns = system_ns() as i128 + NTP_UNIX as i128 * 1_000_000_000;
                let stamp = (
                    (ns / 1_000_000_000) as u32,
                    (((ns % 1_000_000_000) << 32) / 1_000_000_000) as u32,
                );
                let mut r = [0u8; 48];
                r[0] = 0x24; // version 4, mode 4 (server)
                r[1] = 1; // stratum 1
                for off in [32, 40] {
                    r[off..off + 4].copy_from_slice(&stamp.0.to_be_bytes());
                    r[off + 4..off + 8].copy_from_slice(&stamp.1.to_be_bytes());
                }
                let _ = s.send_to(&r, from);
            }
        });
        a
    }

    /// A pool member that does not answer is passed over for the next one
    /// after one wait, not asked again: the time comes from the second.
    #[test]
    fn a_dead_address_is_passed_over_for_the_next() {
        let (_keep, d) = dead();
        let t = std::time::Instant::now();
        let s = measure_at(&[d, alive()]).unwrap();
        let took = t.elapsed();
        assert!(s.rtt_ns < 50_000_000, "{s:?}");
        assert!(took < Duration::from_millis(2_500), "took {took:?}");
    }

    /// Nobody answering costs [`MAX_ADDRESSES`] waits and no more: the fourth
    /// address, alive, is not reached, and the error says how many were tried.
    #[test]
    fn nobody_answering_gives_up_after_three_addresses() {
        let socks: Vec<_> = (0..3).map(|_| dead()).collect();
        let mut addrs: Vec<SocketAddr> = socks.iter().map(|s| s.1).collect();
        addrs.push(alive());
        let t = std::time::Instant::now();
        let e = measure_at(&addrs).unwrap_err();
        let took = t.elapsed();
        assert_eq!(e, "no reply from 3 addresses");
        assert!(
            took >= Duration::from_millis(2_900) && took < Duration::from_secs(8),
            "took {took:?}"
        );
    }

    /// A lost packet is not a dead server: the one address a name resolves to
    /// is asked again, and the measurement comes from the later replies. (Giving
    /// the address up on the first loss left it with nothing to measure.)
    #[test]
    fn a_lost_packet_is_asked_again_not_given_up() {
        let s = measure_at(&[lossy(1)]).unwrap();
        assert!(s.rtt_ns < 50_000_000, "{s:?}");
        // Two lost in a row on the lone address is a dead one: given up after
        // the retry, not asked five times.
        let t = std::time::Instant::now();
        assert!(measure_at(&[lossy(2)]).is_err());
        assert!(
            t.elapsed() < Duration::from_millis(2_900),
            "{:?}",
            t.elapsed()
        );
    }

    #[test]
    fn a_live_server_answers_at_once() {
        let t = std::time::Instant::now();
        assert!(measure_at(&[alive()]).is_ok());
        assert!(
            t.elapsed() < Duration::from_millis(1_000),
            "took {:?}",
            t.elapsed()
        );
    }

    #[test]
    fn ntp_stamps_convert_to_unix_ns() {
        let mut b = [0u8; 8];
        b[0..4].copy_from_slice(&((NTP_UNIX + 10) as u32).to_be_bytes());
        b[4] = 0x80; // half a second
        assert_eq!(ntp_ns(&b), 10_500_000_000);
    }
}
