//! UDP log datagram sink for Phase 0.6.
//!
//! Sends each `log::Log::log` line as a single UDP datagram to a
//! configurable target (broadcast `255.255.255.255:9999` by default).
//! Best-effort: any send error is silently dropped — log path must
//! never block the caller, and packet loss is acceptable for debug
//! traffic.
//!
//! Caller's responsibility:
//! - WiFi STA must be up before constructing (`UdpSocket::bind` on a
//!   modem that's still associating fails). `wifi::connect_sta` blocks
//!   until netif up so this is automatic if you call it post-connect.
//! - Hold the resulting `UdpLogSink` in `LogFanout` so its lifetime
//!   covers all log calls.

use std::net::{SocketAddr, UdpSocket};

pub struct UdpLogSink {
    sock: UdpSocket,
    target: SocketAddr,
}

impl UdpLogSink {
    /// Bind an ephemeral UDP socket and configure broadcast. The
    /// `target` may be unicast or broadcast (`255.255.255.255:port`).
    pub fn new(target: SocketAddr) -> std::io::Result<Self> {
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        sock.set_broadcast(true)?;
        // Don't let log sends block on a stalled stack — if we can't
        // push immediately, drop the line.
        sock.set_nonblocking(true)?;
        Ok(Self { sock, target })
    }

    /// Send one line as a single datagram. Errors silenced.
    pub fn send_line(&self, line: &str) {
        // No newline appended — the listener (`nc -lu`) renders each
        // datagram on its own line already.
        let _ = self.sock.send_to(line.as_bytes(), self.target);
    }
}
