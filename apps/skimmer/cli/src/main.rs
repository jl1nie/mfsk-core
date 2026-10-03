// SPDX-License-Identifier: GPL-3.0-or-later
//! `skimmer`: decode from a SpyServer, one line per message.
//!
//! ```text
//! cargo run --release -p skimmer -- \
//!     --server 192.168.1.26:5555 --ch FT8@7041000 --ch FT8@7074000 --ch FT4@7047500
//! ```
//!
//! Start SDR# (or any controlling client) first; see `skimmer-core` for why.
//! `--log FILE` appends every decode to FILE as WSJT-X `ALL.TXT`-style lines.

use std::fs::OpenOptions;
use std::io::Write;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use skimmer_core::Station;
use skimmer_core::modes::{MODES, mode_name, parse_channel};
use skimmer_core::{Channelizer, Config, Event, WireFormat, all_txt_line};

fn usage() -> ExitCode {
    let modes: Vec<&str> = MODES.iter().map(|m| m.0).collect();
    eprintln!(
        "usage: skimmer --server HOST:PORT --ch MODE@DIAL_HZ[:band=LO-HI][:dx=CALL][:depth=fast|normal|deep] [--ch ...] [--mycall CALL --mygrid GRID] [--log FILE]\n\
         \x20      [--tune] [--yield] [--center HZ] [--rate S/s] [--gain N] [--format float|int16]\n\
         \x20      [--pfb | --direct] [--iq-swap] [--reanchor-ms MS]\n\
         channelizer: filter bank from {} active channels, else direct, unless forced\n\
         modes: {}",
        skimmer_core::AUTO_PFB_CHANNELS,
        modes.join(" ")
    );
    ExitCode::from(2)
}

fn parse_args() -> Option<(Config, Option<String>)> {
    let mut cfg = Config::new("127.0.0.1:5555", Vec::new());
    let mut log = None;
    let (mut mycall, mut mygrid) = (String::new(), String::new());
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--server" => cfg.server = it.next()?,
            "--log" => log = Some(it.next()?),
            "--tune" => cfg.tune = true,
            "--yield" => cfg.yield_control = true,
            "--center" => cfg.center_hz = Some(it.next()?.parse().ok()?),
            "--rate" => cfg.rate = Some(it.next()?.parse().ok()?),
            "--gain" => cfg.gain = Some(it.next()?.parse().ok()?),
            "--format" => {
                cfg.format = match it.next()?.as_str() {
                    "float" => WireFormat::Float,
                    "int16" => WireFormat::Int16,
                    _ => return None,
                }
            }
            "--pfb" => cfg.channelizer = Some(Channelizer::Pfb),
            "--direct" => cfg.channelizer = Some(Channelizer::Direct),
            "--iq-swap" => cfg.iq_swap = true,
            "--reanchor-ms" => {
                cfg.reanchor = Duration::from_millis(it.next()?.parse().ok()?);
            }
            "--mycall" => mycall = it.next()?.to_ascii_uppercase(),
            "--mygrid" => mygrid = it.next()?.to_ascii_uppercase(),
            "--ch" => match parse_channel(&it.next()?) {
                Ok(ch) => cfg.channels.push(ch),
                Err(e) => {
                    eprintln!("{e}");
                    return None;
                }
            },
            _ => return None,
        }
    }
    cfg.live.set_station(Station {
        call: mycall,
        grid: mygrid,
    });
    (!cfg.channels.is_empty()).then_some((cfg, log))
}

fn hhmmss(ns: Option<i64>) -> String {
    ns.map(|ns| {
        let s = ns.div_euclid(1_000_000_000).rem_euclid(86_400);
        format!("{:02}{:02}{:02}", s / 3600, s / 60 % 60, s % 60)
    })
    .unwrap_or_else(|| "------".into())
}

fn main() -> ExitCode {
    let Some((cfg, log)) = parse_args() else {
        return usage();
    };
    let mut log = match log {
        None => None,
        Some(path) => match OpenOptions::new().create(true).append(true).open(&path) {
            Ok(f) => Some(f),
            Err(e) => {
                eprintln!("{path}: {e}");
                return ExitCode::FAILURE;
            }
        },
    };
    // Nothing sets it: the CLI runs until killed. The GUI owns its flag.
    let stop = AtomicBool::new(false);
    skimmer_core::run(&cfg, &stop, |ev| match ev {
        Event::Decode(d) => {
            println!(
                "{} {:<8} {:>10.0} {:>4.0} {:>5.1}  {}",
                hhmmss(d.slot_utc_ns),
                mode_name(d.mode),
                d.freq_hz,
                d.snr_db,
                d.dt_s,
                d.text
            );
            if let Some(f) = log.as_mut()
                && let Err(e) = writeln!(f, "{}", all_txt_line(&d))
            {
                eprintln!("log: {e}");
            }
        }
        Event::Connecting { server } => eprintln!("connecting to {server}"),
        Event::Connected {
            device,
            control,
            device_hz,
        } => eprintln!(
            "connected: device type {}, {} S/s max, band {:.0} Hz, control {control}, \
             device centre {device_hz:.0} Hz",
            device.kind, device.max_rate, device.bandwidth_hz
        ),
        Event::Radio(_) | Event::Waterfall(_) => {}
        Event::Yielded => eprintln!(
            "got control of the device; leaving it for the operator's client \
             (--yield is set; drop it to hold control and tune the radio)"
        ),
        Event::NoChannelFits { device_hz } => eprintln!(
            "no channel fits the band around {device_hz:.0} Hz; waiting for the device to move"
        ),
        Event::Streaming {
            rate,
            decimation,
            center_hz,
            device_hz,
            active,
            channelizer,
        } => {
            eprintln!(
                "IQ {rate} S/s (decimation {decimation}) centre {center_hz:.0} Hz, \
                 span {:.0}..{:.0}; device centre {device_hz:.0}; {channelizer:?} channelizer",
                center_hz - rate as f64 / 2.0,
                center_hz + rate as f64 / 2.0
            );
            for (c, on) in cfg.channels.iter().zip(active) {
                let state = if on {
                    ""
                } else {
                    " (paused: outside the band)"
                };
                eprintln!("  channel {}@{:.0}{state}", mode_name(c.mode), c.dial_hz);
            }
        }
        Event::Moved { device_hz, iq_hz } => {
            eprintln!("moved: device centre {device_hz:.0} Hz, IQ centre {iq_hz:.0} Hz")
        }
        Event::Gap { messages, at_s } => {
            eprintln!("gap: {messages} message(s) lost at {at_s:.1} s")
        }
        Event::Reanchor { by_s } => eprintln!("re-anchor: {by_s:+.3} s"),
        Event::Status(s) => eprintln!(
            "status: {:.0} s streamed, delay {:.0} ms, drift {:+.0} ms, longest push {:.0} ms, \
             queue {:.0} kB, decode {:.0} ms, {} slot(s) queued, {} dropped, \
             {} gap(s), {} re-anchor(s)",
            s.streamed_s,
            s.delay_ms,
            s.drift_ms,
            s.longest_push_ms,
            s.queued_bytes as f64 / 1e3,
            s.longest_decode_ms,
            s.queued_slots,
            s.dropped_slots,
            s.gaps,
            s.reanchors
        ),
        Event::Disconnected { error } => eprintln!("{}: {error}", cfg.server),
    });
    ExitCode::SUCCESS
}
