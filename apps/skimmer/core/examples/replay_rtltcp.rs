// SPDX-License-Identifier: GPL-3.0-only
//! Replay record.py IQ on its 15-second phase using the same NTP clock as the GUI.
//! Usage: replay_rtltcp RECORDING.bin [127.0.0.1:5150] [ntp.nict.jp]
use skimmer_core::clock;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

fn serve(
    mut socket: TcpStream,
    greeting: &[u8],
    chunks: &[(f64, Vec<u8>)],
    phase: i64,
) -> std::io::Result<()> {
    socket.set_nodelay(true)?;
    socket.set_write_timeout(Some(Duration::from_secs(5)))?;
    let mut commands = socket.try_clone()?;
    std::thread::spawn(move || {
        let mut bytes = [0u8; 4096];
        while commands.read(&mut bytes).is_ok_and(|n| n > 0) {}
    });
    let now = clock::now_ns();
    let mut wait = (phase - now).rem_euclid(15_000_000_000);
    if wait < 10_000_000 {
        wait += 15_000_000_000;
    }
    let target = now + wait;
    let start = Instant::now() + Duration::from_nanos(wait as u64);
    eprintln!(
        "{}",
        serde_json::json!({"event":"replay_start","source_utc_ns":now,
        "target_utc_ns":target,"ntp":clock::report(),"source_offset_ns":now-clock::system_ns()})
    );
    // Both greeting and first-IQ reads have a ten-second timeout in stock
    // clients. Delay the greeting at most eight seconds, then first IQ at
    // most seven more, without delivering samples ahead of their schedule.
    if let Some(delay) = start
        .checked_duration_since(Instant::now())
        .and_then(|d| d.checked_sub(Duration::from_secs(7)))
    {
        std::thread::sleep(delay);
    }
    socket.write_all(greeting)?;
    let mut max_late = Duration::ZERO;
    for (elapsed, bytes) in chunks {
        let deadline = start + Duration::from_secs_f64(*elapsed);
        if let Some(delay) = deadline.checked_duration_since(Instant::now()) {
            std::thread::sleep(delay);
        }
        max_late = max_late.max(Instant::now().saturating_duration_since(deadline));
        socket.write_all(bytes)?;
    }
    eprintln!(
        "{}",
        serde_json::json!({"event":"replay_end","max_late_ms":max_late.as_secs_f64()*1000.0})
    );
    socket.shutdown(std::net::Shutdown::Both)?;
    Ok(())
}

fn main() -> std::io::Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .expect("usage: replay_rtltcp RECORDING.bin [ADDRESS] [NTP]");
    let address = args.next().unwrap_or_else(|| "127.0.0.1:5150".into());
    let ntp = args.next().unwrap_or_else(|| "ntp.nict.jp".into());
    let mut file = std::io::BufReader::new(std::fs::File::open(path)?);
    let mut header = [0; 12];
    file.read_exact(&mut header)?;
    let original_ns = i64::from_le_bytes(header[..8].try_into().unwrap());
    let count = u32::from_le_bytes(header[8..].try_into().unwrap());
    let mut greeting = Vec::new();
    let mut chunks = Vec::new();
    for _ in 0..count {
        file.read_exact(&mut header)?;
        let elapsed = f64::from_le_bytes(header[..8].try_into().unwrap());
        let size = u32::from_le_bytes(header[8..].try_into().unwrap()) as usize;
        let mut bytes = vec![0; size];
        file.read_exact(&mut bytes)?;
        if greeting.len() < 12 {
            let n = (12 - greeting.len()).min(bytes.len());
            greeting.extend(bytes.drain(..n));
        }
        if !bytes.is_empty() {
            chunks.push((elapsed, bytes));
        }
    }
    assert_eq!(&greeting[..4], b"RTL0");
    assert!(
        !chunks.is_empty() && !chunks[0].1.starts_with(b"RSP0"),
        "this replay accepts plain 8-bit captures"
    );
    let mut clock_sync = clock::NtpSync::start(&ntp);
    if !clock::report().starts_with("NTP +") && !clock::report().starts_with("NTP -") {
        return Err(std::io::Error::other(clock::report()));
    }
    let listener = TcpListener::bind(&address)?;
    eprintln!(
        "{}",
        serde_json::json!({"event":"listening","address":address,
        "ntp_server":ntp,"ntp":clock::report(),"recording_start_ns":original_ns,
        "phase_basis":"recorded PC wall clock; original NTP offset is unknown"})
    );
    for connection in listener.incoming() {
        // Refresh alongside the receiver's initial measurement instead of
        // comparing a new GUI clock with a sender correction several minutes old.
        drop(clock_sync);
        clock_sync = clock::NtpSync::start(&ntp);
        if !clock::report().starts_with("NTP +") && !clock::report().starts_with("NTP -") {
            return Err(std::io::Error::other(clock::report()));
        }
        if let Err(e) = connection.and_then(|socket| {
            serve(
                socket,
                &greeting,
                &chunks,
                original_ns.rem_euclid(15_000_000_000),
            )
        }) {
            eprintln!("client ended: {e}");
        }
    }
    Ok(())
}
