// SPDX-License-Identifier: GPL-3.0-or-later
//! The skimmer's own UTC: the PC's clock plus an offset measured against an
//! NTP server (SNTP, RFC 4330), or the PC's clock as it is.
//!
//! The PC clock is not touched (that would need rights and surprise other
//! programs); [`now_ns`] adds the measured offset. Windows' own time service
//! syncs weekly by default and can be several hundred ms out, which moves
//! every channel's DT; a query is 48 bytes and its error is bounded by half
//! the round trip.

use std::net::{ToSocketAddrs, UdpSocket};
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

fn query(server: &str) -> Result<Sample, String> {
    let addr = if server.contains(':') {
        server.to_string()
    } else {
        format!("{server}:123")
    };
    let target = addr
        .to_socket_addrs()
        .map_err(|e| format!("{server}: {e}"))?
        .find(|a| a.is_ipv4())
        .ok_or_else(|| format!("{server}: no IPv4 address"))?;
    let sock = UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
    sock.set_read_timeout(Some(Duration::from_secs(2)))
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
        return Err(format!("{server}: not a time reply"));
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
    let mut best: Option<Sample> = None;
    let mut err = String::new();
    for i in 0..QUERIES {
        if i > 0 {
            std::thread::sleep(Duration::from_millis(150));
        }
        match query(server) {
            Ok(s) if s.rtt_ns <= MAX_RTT_NS && best.is_none_or(|b| s.rtt_ns < b.rtt_ns) => {
                best = Some(s)
            }
            Ok(_) => err = "round trip over 1 s".into(),
            Err(e) => err = e,
        }
    }
    best.ok_or(err)
}

fn apply(server: &str, r: Result<Sample, String>) -> bool {
    match r {
        Ok(s) => {
            OFFSET_NS.store(s.offset_ns, Ordering::Relaxed);
            set_report(format!(
                "NTP {:+.0} ms (round trip {:.0} ms)",
                s.offset_ns as f64 * 1e-6,
                s.rtt_ns as f64 * 1e-6
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

    #[test]
    fn ntp_stamps_convert_to_unix_ns() {
        let mut b = [0u8; 8];
        b[0..4].copy_from_slice(&((NTP_UNIX + 10) as u32).to_be_bytes());
        b[4] = 0x80; // half a second
        assert_eq!(ntp_ns(&b), 10_500_000_000);
    }
}
