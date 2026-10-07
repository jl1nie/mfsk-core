// SPDX-License-Identifier: GPL-3.0-only
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
use skimmer_core::{Channelizer, Config, Event, Step, WireFormat, all_txt_line};

fn usage() -> ExitCode {
    let modes: Vec<&str> = MODES.iter().map(|m| m.0).collect();
    eprintln!(
        "usage: skimmer --server HOST:PORT --ch MODE@DIAL_HZ[:band=LO-HI][:dx=CALL][:depth=fast|normal|deep] [--ch ...] [--mycall CALL --mygrid GRID] [--log FILE]\n\
         \x20      (several servers: repeat --server [NAME=]HOST:PORT with its own options and --ch; a rotation: --step MINUTES before the --ch heard in that step)\n\
         \x20      [--tune] [--yield] [--ntp HOST] [--net-delay MS] [--center HZ] [--rate S/s] [--gain N] [--format float|int16]\n\
         \x20      [--pfb | --direct] [--iq-swap] [--reanchor-ms MS] [--slot-budget SHARE|off] [--lanes N] [--detail]\n\
         channelizer: filter bank from {} active channels, else direct, unless forced\n\
         modes: {}",
        skimmer_core::AUTO_PFB_CHANNELS,
        modes.join(" ")
    );
    ExitCode::from(2)
}

/// The servers named on the command line, and the log file.
///
/// `--server [NAME=]HOST:PORT` starts a server; the options and `--ch` that
/// follow belong to it. `--step MINUTES` starts a step of a rotation among that
/// server's channels: the `--ch` after it are heard in that step.
fn parse_args() -> Option<(Vec<Config>, Option<String>, bool)> {
    let mut cfgs: Vec<Config> = Vec::new();
    let mut cfg = Config::new("127.0.0.1:5555", Vec::new());
    let mut started = false;
    let mut log = None;
    let mut detail = false;
    let (mut mycall, mut mygrid) = (String::new(), String::new());
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--server" => {
                if started {
                    cfgs.push(cfg);
                    cfg = Config::new("127.0.0.1:5555", Vec::new());
                }
                started = true;
                let v = it.next()?;
                match v.split_once('=') {
                    Some((name, addr)) => {
                        cfg.name = name.to_string();
                        cfg.server = addr.to_string();
                    }
                    None => cfg.server = v,
                }
            }
            "--step" => {
                started = true;
                cfg.steps.push(Step {
                    channels: Vec::new(),
                    minutes: it.next()?.parse().ok()?,
                    hours: None,
                    follow: None,
                });
            }
            "--log" => log = Some(it.next()?),
            "--tune" => cfg.tune = true,
            "--yield" => cfg.yield_control = true,
            "--ntp" => cfg.ntp = Some(it.next()?),
            "--net-delay" => cfg.live.set_network_delay_ms(it.next()?.parse().ok()?),
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
            // A slot may decode this share of its period, then stops and reports
            // what it has; off (the default) decodes every slot to the end.
            "--slot-budget" => {
                cfg.slot_budget = match it.next()?.as_str() {
                    "off" => None,
                    v => Some(v.parse::<f32>().ok().filter(|x| *x > 0.0)?),
                }
            }
            // Decoder threads per channel (4 by default): the next slot is
            // decoded on another thread while the last is still being decoded.
            "--lanes" => cfg.decode_lanes = it.next()?.parse::<usize>().ok().filter(|n| *n > 0)?,
            "--detail" => detail = true,
            "--mycall" => mycall = it.next()?.to_ascii_uppercase(),
            "--mygrid" => mygrid = it.next()?.to_ascii_uppercase(),
            "--ch" => match parse_channel(&it.next()?) {
                Ok(ch) => {
                    started = true;
                    cfg.channels.push(ch);
                    let at = cfg.channels.len() - 1;
                    if let Some(step) = cfg.steps.last_mut() {
                        step.channels.push(at);
                    }
                }
                Err(e) => {
                    eprintln!("{e}");
                    return None;
                }
            },
            _ => return None,
        }
    }
    cfgs.push(cfg);
    for (i, c) in cfgs.iter_mut().enumerate() {
        if c.name.is_empty() {
            c.name = if i == 0 && c.server == "127.0.0.1:5555" {
                String::new()
            } else {
                c.server.clone()
            };
        }
        c.live.set_station(Station::new(&mycall, &mygrid));
        // Channels given before the first --step are heard in the first one.
        if !c.steps.is_empty() {
            let listed: std::collections::HashSet<usize> = c
                .steps
                .iter()
                .flat_map(|s| s.channels.iter().copied())
                .collect();
            let loose: Vec<usize> = (0..c.channels.len())
                .filter(|i| !listed.contains(i))
                .collect();
            c.steps[0].channels.extend(loose);
            // A step nobody is in is no step.
            c.steps.retain(|s| !s.channels.is_empty());
            if c.steps.len() < 2 {
                c.steps.clear();
            }
        }
    }
    cfgs.iter()
        .all(|c| !c.channels.is_empty())
        .then_some((cfgs, log, detail))
}

fn hhmmss(ns: Option<i64>) -> String {
    ns.map(|ns| {
        let s = ns.div_euclid(1_000_000_000).rem_euclid(86_400);
        format!("{:02}{:02}{:02}", s / 3600, s / 60 % 60, s % 60)
    })
    .unwrap_or_else(|| "------".into())
}

/// What the decoder knew of a row, for `--detail`: sync score, the errors the
/// FEC corrected, and the message key (the bits, hex) that identifies it.
fn detail_text(d: &skimmer_core::Decode) -> String {
    let k = &d.detail;
    let key: String = k
        .key
        .chunks(4)
        .map(|c| {
            format!(
                "{:x}",
                c.iter().fold(0u8, |a, &b| a << 1 | (b & 1)) << (4 - c.len())
            )
        })
        .collect();
    let key = if key.is_empty() { "-".into() } else { key };
    format!(
        "  [sync {:.1} err {}{} key {key}]",
        k.sync_score,
        k.hard_errors,
        if k.copied_last_tx {
            " copied-last-tx"
        } else {
            ""
        },
    )
}

fn main() -> ExitCode {
    let Some((cfgs, log, detail)) = parse_args() else {
        return usage();
    };
    let many = cfgs.len() > 1;
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
    skimmer_core::run_all(&cfgs, &stop, |server, ev| {
        let cfg = &cfgs[server];
        // Which server, when there is more than one.
        let tag = if many {
            format!("[{}] ", cfg.name)
        } else {
            String::new()
        };
        match ev {
            Event::Decode(d) => {
                println!(
                    "{tag}{} {:<8} {:>10.0} {:>4.0} {:>5.1}  {}{}{}",
                    hhmmss(d.slot_utc_ns),
                    mode_name(d.mode),
                    d.freq_hz,
                    d.snr_db,
                    d.dt_s,
                    d.text,
                    // The same row again, its `<...>` resolved.
                    if d.update { "  (resolved)" } else { "" },
                    if detail {
                        detail_text(&d)
                    } else {
                        String::new()
                    }
                );
                // ALL.TXT has each message once, as it was first heard: the
                // resolved form of a row already written is not a new line.
                if !d.update
                    && let Some(f) = log.as_mut()
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
            Event::Clock(text) => eprintln!("{text}"),
            Event::Step {
                index,
                of,
                ends_utc_s,
                held,
            } => {
                let until = ends_utc_s % 86_400 / 3600 * 100 + ends_utc_s % 3600 / 60;
                match index {
                    Some(i) if held => eprintln!("rotation held on step {}/{}", i + 1, of),
                    Some(i) => eprintln!("rotation step {}/{} until {until:04} UTC", i + 1, of),
                    None => eprintln!("no band is in at this hour; waiting until {until:04} UTC"),
                }
            }
            Event::Status(s) => eprintln!(
                "status: {:.0} s streamed, delay {:.0} ms, drift {:+.0} ms, longest push {:.0} ms, \
             queue {:.0} kB, decode {:.0} ms, {} slot(s) queued, {} dropped, \
             {} over budget, {} gap(s), {} re-anchor(s)",
                s.streamed_s,
                s.delay_ms,
                s.drift_ms,
                s.longest_push_ms,
                s.queued_bytes as f64 / 1e3,
                s.longest_decode_ms,
                s.queued_slots,
                s.dropped_slots,
                s.budget_cut_slots,
                s.gaps,
                s.reanchors
            ),
            Event::Off => eprintln!("{tag}{}: off", cfg.server),
            Event::Disconnected { error } => eprintln!("{tag}{}: {error}", cfg.server),
        }
    });
    ExitCode::SUCCESS
}
