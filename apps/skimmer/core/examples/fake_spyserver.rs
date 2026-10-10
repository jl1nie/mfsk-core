// SPDX-License-Identifier: GPL-3.0-only
//! A stand-in SpyServer with a JTTY station on the band, to try the skimmer
//! without a radio (see `skimmer_core::fake`):
//!
//! ```text
//! cargo run -p skimmer-core --release --example fake_spyserver
//! cargo run -p skimmer --release -- --server 127.0.0.1:5555 --ch JTTY@14090000
//! ```
//!
//! or point the GUI's server at `127.0.0.1:5555` with a JTTY channel at 14 090 kHz.
//! The band carries a 48 s script on repeat (`skimmer_core::fake::Band::default`):
//! a QSO on 1500 Hz (K1ABC and JA1ABC, a call, an answer, reports and a 73), another on
//! 1350 Hz (W9XYZ and VK3NV, weaker) and DL1ABC calling CQ on 1650 Hz (weaker still), the
//! two last on the side channels upstream looks at; all within a few dB of JTTY's decoding
//! limit (SNR reported about -9 to -15 dB), in additive white Gaussian noise, and fading.
//! Options: `--listen ADDR` (default `127.0.0.1:5555`), `--dial HZ` (the JTTY
//! channel's dial, default 14090000), `--noise F` (per component, a fraction of full
//! scale, default 0.02), `--amplitude F` (a level-1 signal's peak, default 0.0017: 7 dB above the 50 % point),
//! `--jitter SECONDS` (each message starts up to this much later than its time in the
//! script, differently every cycle; default 1.5, 0 for the script's own times) and
//! `--qsb F` (fading depth: a signal swings between full and `1 - F` of its level about
//! every ten seconds, each station on its own phase; default 0.3, 0 for steady).

use skimmer_core::fake::{Band, FakeServer};

fn main() {
    let mut listen = "127.0.0.1:5555".to_string();
    let mut band = Band::default();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut next = || args.next().unwrap_or_else(|| usage());
        match a.as_str() {
            "--listen" => listen = next(),
            "--dial" => band.dial_hz = next().parse().unwrap_or_else(|_| usage()),
            "--noise" => band.noise = next().parse().unwrap_or_else(|_| usage()),
            "--amplitude" => band.amplitude = next().parse().unwrap_or_else(|_| usage()),
            "--jitter" => band.jitter_s = next().parse().unwrap_or_else(|_| usage()),
            "--qsb" => band.qsb = next().parse().unwrap_or_else(|_| usage()),
            _ => usage(),
        }
    }
    let server = FakeServer::start(&listen, band.clone()).unwrap_or_else(|e| {
        eprintln!("{listen}: {e}");
        std::process::exit(1);
    });
    eprintln!(
        "fake spyserver on {}: JTTY at dial {} Hz, a {} s script on repeat:",
        server.addr, band.dial_hz, band.cycle_s
    );
    let mut script = band.script.clone();
    script.sort_by(|a, b| a.at_s.total_cmp(&b.at_s));
    for l in &script {
        eprintln!(
            "  {:5.1} s  {:6.0} Hz  level {:.2}  {}",
            l.at_s, l.hz, l.level, l.text
        );
    }
    eprintln!("Ctrl-C to stop");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(3600));
    }
}

fn usage() -> ! {
    eprintln!(
        "usage: fake_spyserver [--listen ADDR] [--dial HZ] [--noise F] [--amplitude F] [--jitter S] [--qsb F]"
    );
    std::process::exit(2);
}
