// SPDX-License-Identifier: GPL-3.0-only
//! Causal acquisition -> API callback -> FIFO transport fixture for the actual GUI.
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::engine::{MessageCodec, MessageFields};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::Wsjt77Message;
use serde_json::json;
use std::fs::OpenOptions;
use std::io::{BufWriter, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
const RATE: u32 = 1_024_000;
const CALLS: [&str; 6] = ["K1ABC", "W9XYZ", "JA1ABC", "DL1ABC", "VK3ABC", "G1ABC"];
#[derive(Clone)]
struct Settings {
    address: String,
    impairment: String,
    ppm: f64,
    snr: f64,
    fixed_delay: f64,
    hole: f64,
    period: f64,
    api_delay: f64,
    seed: u32,
    vary: bool,
    variable_loss: bool,
    truth: Option<String>,
}
fn main() -> std::io::Result<()> {
    let mut cfg = Settings {
        address: "127.0.0.1:1234".into(),
        impairment: "filled".into(),
        ppm: 0.0,
        snr: -10.0,
        fixed_delay: 0.0,
        hole: 0.060,
        period: 1.85,
        api_delay: 0.0,
        seed: 99,
        vary: false,
        variable_loss: false,
        truth: None,
    };
    let (mut loss_ms, mut api_ms) = (None, None);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--vary-messages" {
            cfg.vary = true;
            continue;
        }
        if arg == "--vary-loss" {
            cfg.variable_loss = true;
            continue;
        }
        let value = args.next().expect("option requires a value");
        match arg.as_str() {
            "--listen" => cfg.address = value,
            "--impairment" => cfg.impairment = value,
            "--ppm" => cfg.ppm = value.parse().unwrap(),
            "--snr" => cfg.snr = value.parse().unwrap(),
            "--fixed-delay-ms" => cfg.fixed_delay = value.parse::<f64>().unwrap() / 1000.0,
            "--loss-ms" => loss_ms = Some(value.parse::<f64>().unwrap()),
            "--loss-period-ms" => cfg.period = value.parse::<f64>().unwrap() / 1000.0,
            "--api-delay-ms" => api_ms = Some(value.parse::<f64>().unwrap()),
            "--seed" => cfg.seed = value.parse().unwrap(),
            "--truth" => cfg.truth = Some(value),
            _ => panic!(
                "options: --listen ADDR --impairment clean|filled|dropped|stall --ppm N --snr DB --fixed-delay-ms N --loss-ms N --loss-period-ms N --api-delay-ms N --seed N --truth PATH --vary-messages"
            ),
        }
    }
    assert!(["clean", "filled", "dropped", "stall"].contains(&cfg.impairment.as_str()));
    if cfg.impairment == "clean" || cfg.impairment == "stall" {
        cfg.hole = 0.0;
    }
    if cfg.impairment == "stall" {
        cfg.period = 30.0;
        cfg.api_delay = 0.750;
    }
    if let Some(v) = loss_ms {
        cfg.hole = v / 1000.0;
    }
    if let Some(v) = api_ms {
        cfg.api_delay = v / 1000.0;
    }
    assert!(cfg.ppm.is_finite() && cfg.ppm.abs() <= 50_000.0);
    assert!((-30.0..=0.0).contains(&cfg.snr));
    assert!((0.0..=30.0).contains(&cfg.fixed_delay));
    assert!(cfg.period.is_finite() && cfg.period > 0.0);
    assert!(cfg.hole >= 0.0 && cfg.hole < cfg.period);
    assert!(cfg.api_delay >= 0.0 && cfg.api_delay < cfg.period);
    assert!(cfg.hole == 0.0 || ["filled", "dropped"].contains(&cfg.impairment.as_str()));
    let audio: Arc<Vec<Vec<i16>>> = Arc::new(
        CALLS
            .iter()
            .map(|call| {
                let bits: [u8; 77] = Wsjt77Message
                    .pack(&MessageFields {
                        call1: Some("CQ".into()),
                        call2: Some((*call).into()),
                        grid: Some("FN42".into()),
                        ..MessageFields::default()
                    })
                    .unwrap()
                    .try_into()
                    .unwrap();
                let frame =
                    synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, 1500.0, 25_000);
                let mut slot = vec![0; 180_000];
                slot[6000..6000 + frame.len()].copy_from_slice(&frame);
                slot
            })
            .collect(),
    );
    let listener = TcpListener::bind(&cfg.address)?;
    eprintln!(
        "FT8 rtl_tcp on {}: {} target {}dB, actual {:+}ppm, fixed {}ms, API {}ms, hole {}ms/{}ms",
        cfg.address,
        cfg.impairment,
        cfg.snr,
        cfg.ppm,
        cfg.fixed_delay * 1000.0,
        cfg.api_delay * 1000.0,
        cfg.hole * 1000.0,
        cfg.period * 1000.0
    );
    for stream in listener.incoming() {
        if let Err(e) = stream.and_then(|s| serve(s, audio.clone(), cfg.clone())) {
            eprintln!("client disconnected: {e}");
        }
    }
    Ok(())
}
struct Captured {
    bytes: Vec<u8>,
    first: u64,
    end: u64,
    erased: usize,
}
fn wall_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64
}
fn serve(mut sock: TcpStream, audio: Arc<Vec<Vec<i16>>>, cfg: Settings) -> std::io::Result<()> {
    sock.set_nodelay(true)?;
    sock.set_write_timeout(Some(Duration::from_secs(5)))?;
    sock.write_all(b"RTL0")?;
    sock.write_all(&5u32.to_be_bytes())?;
    sock.write_all(&28u32.to_be_bytes())?;
    let mut caps = [0u8; 45];
    caps[..4].copy_from_slice(b"RSP0");
    caps[4..8].copy_from_slice(&1u32.to_be_bytes());
    caps[16..20].copy_from_slice(&1u32.to_be_bytes());
    caps[20..24].copy_from_slice(&2u32.to_be_bytes());
    sock.write_all(&caps)?;
    let center = Arc::new(AtomicU32::new(14_049_000));
    let running = Arc::new(AtomicBool::new(true));
    let (c, flag) = (center.clone(), running.clone());
    let mut commands = sock.try_clone()?;
    std::thread::spawn(move || {
        let mut cmd = [0; 5];
        while commands.read_exact(&mut cmd).is_ok() {
            let value = u32::from_be_bytes(cmd[1..].try_into().unwrap());
            eprintln!("command {:02x}: {value}", cmd[0]);
            if cmd[0] == 1 {
                c.store(value, Ordering::Relaxed);
            }
            if cmd[0] == 2 && value != RATE {
                eprintln!("fixture requires {RATE} S/s");
                break;
            }
        }
        flag.store(false, Ordering::Relaxed);
    });
    let t0 = wall_ns();
    let start = Instant::now();
    let actual = f64::from(RATE) * (1.0 + cfg.ppm * 1e-6);
    let (tx, rx) = mpsc::channel::<Captured>();
    let (s, c, flag) = (cfg.clone(), center, running.clone());
    // Acquisition runs independently of callback/transport stalls. A block becomes
    // available only after its LAST source sample has been acquired (no lookahead).
    std::thread::spawn(move || {
        let rms = (audio[0].iter().map(|&x| f64::from(x).powi(2)).sum::<f64>() / (12.64 * 12000.0))
            .sqrt();
        let sigma = 1000.0;
        let gain = sigma * (10000.0 * 10.0f64.powf(s.snr / 10.0) / actual).sqrt() / rms;
        let mut rng = s.seed;
        let mut n = 0u64;
        while flag.load(Ordering::Relaxed) {
            let end = n + 4096;
            if let Some(wait) =
                Duration::from_secs_f64(end as f64 / actual).checked_sub(start.elapsed())
            {
                std::thread::sleep(wait);
            }
            let mut bytes = Vec::with_capacity(4096 * 4);
            let mut erased = 0;
            let center = f64::from(c.load(Ordering::Relaxed));
            for k in n..end {
                let elapsed = k as f64 / actual;
                let utc = t0 as f64 * 1e-9 + elapsed;
                let phase = utc.rem_euclid(15.0);
                let hole_length = s.hole
                    * if s.variable_loss {
                        [0.3, 1.0, 1.7][(elapsed / 10.0) as usize % 3]
                    } else {
                        1.0
                    };
                let hole = s.hole > 0.0
                    && elapsed >= 1.05
                    && (elapsed - 1.05).rem_euclid(s.period) < hole_length;
                let mut uniform = || {
                    rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (f64::from(rng >> 8) + 1.0) / f64::from(1u32 << 24)
                };
                let (u, v) = (uniform(), uniform());
                let radius = sigma * (-2.0 * u.ln()).sqrt();
                let noise_phase = std::f64::consts::TAU * v;
                if hole {
                    erased += 1;
                    if s.impairment == "dropped" {
                        continue;
                    }
                }
                let index = if s.vary {
                    (utc / 15.0).floor() as usize % CALLS.len()
                } else {
                    0
                };
                let a = &audio[index];
                let x = phase * 12000.0;
                let i = x as usize;
                let value = (f64::from(a[i])
                    + (f64::from(a[(i + 1) % a.len()]) - f64::from(a[i])) * x.fract())
                    * gain;
                let p = std::f64::consts::TAU * (14_074_000.0 - center) * elapsed;
                let samples = if hole {
                    [0.0, 0.0]
                } else {
                    [
                        value * p.cos() + radius * noise_phase.cos(),
                        value * p.sin() + radius * noise_phase.sin(),
                    ]
                };
                for value in samples {
                    bytes.extend_from_slice(&(value.round() as i16).to_le_bytes());
                }
            }
            if tx
                .send(Captured {
                    bytes,
                    first: n,
                    end,
                    erased,
                })
                .is_err()
            {
                break;
            }
            n = end;
        }
    });
    let mut truth = cfg
        .truth
        .as_ref()
        .map(|p| {
            OpenOptions::new()
                .create(true)
                .append(true)
                .open(p)
                .map(BufWriter::new)
        })
        .transpose()?;
    if let Some(log) = truth.as_mut() {
        writeln!(
            log,
            "{}",
            json!({"type":"source","t0_ns":t0,"requested_rate_hz":RATE,"actual_rate_hz":actual,"target_snr":cfg.snr,"calls":CALLS,"vary_messages":cfg.vary,"fixed_delay_ms":cfg.fixed_delay*1000.0,"loss_ms":cfg.hole*1000.0,"loss_period_ms":cfg.period*1000.0,"api_delay_ms":cfg.api_delay*1000.0,"filled":cfg.impairment=="filled","seed":cfg.seed,"variable_loss":cfg.variable_loss})
        )?;
        log.flush()?;
    }
    let mut count = 0;
    let result = (|| {
        for block in rx {
            if !running.load(Ordering::Relaxed) {
                break;
            }
            let capture_end = block.end as f64 / actual;
            let phase = (capture_end - 1.05).rem_euclid(cfg.period);
            let api_delay = if cfg.variable_loss {
                cfg.hole * [0.3, 1.0, 1.7][(capture_end / 10.0) as usize % 3] + 0.025
            } else {
                cfg.api_delay
            };
            let wait = if capture_end >= 1.05 && phase < api_delay {
                api_delay - phase
            } else {
                0.0
            };
            // FIFO: callback bursts cannot reorder acquisition blocks.
            let release = Duration::from_secs_f64(capture_end + cfg.fixed_delay + wait);
            if let Some(wait) = release.checked_sub(start.elapsed()) {
                std::thread::sleep(wait);
            }
            if !block.bytes.is_empty() {
                sock.write_all(&block.bytes)?;
            }
            let delivered = wall_ns();
            let capture_end_ns = t0 + (capture_end * 1e9).round() as i64;
            assert!(delivered >= capture_end_ns, "fixture sent future samples");
            if let Some(log) = truth.as_mut() {
                writeln!(
                    log,
                    "{}",
                    json!({"type":"block","first_sample":block.first,"end_sample_exclusive":block.end,"capture_end_utc_ns":capture_end_ns,"delivery_utc_ns":delivered,"payload_samples":block.bytes.len()/4,"erased_samples":block.erased})
                )?;
                count += 1;
                if count % 100 == 0 {
                    log.flush()?;
                }
            }
        }
        Ok(())
    })();
    running.store(false, Ordering::Relaxed);
    if let Some(log) = truth.as_mut() {
        log.flush()?;
    }
    result
}
