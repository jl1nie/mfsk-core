// SPDX-License-Identifier: GPL-3.0-only
//! Diagnostic, opt-in experiment through the actual skimmer stream loop.
//! Acquisition UTC, callback delivery UTC and consumed sample count are independent.
use super::*;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::engine::{MessageCodec, MessageFields};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::Wsjt77Message;
use serde_json::{Value, json};
use std::sync::Arc;

const RATE: u32 = 192_000;
const T0: i64 = 1_700_000_010_000_000_000;
const DIAL: f64 = 14_074_000.0;
const CENTER: f64 = DIAL - 25_000.0;
const BLOCK: usize = 16_384; // rtl_tcp's actual 65536-byte signed-16 read size
const CALLS: [&str; 6] = ["K1ABC", "W9XYZ", "JA1ABC", "DL1ABC", "VK3ABC", "G1ABC"];

#[derive(Clone, Copy, Debug)]
enum Correction {
    Nominal,
    Envelope,
    ClockOnly,
    Measured,
    Payload,
    TrueRate,
}

#[derive(Clone, Debug)]
struct Case {
    name: String,
    ppm: f64,
    delay: f64,
    compensate: bool,
    hole: f64,
    period: f64,
    fill: bool,
    api_delay: f64,
    first_hole: f64,
    variable_loss: bool,
    stochastic_loss: Option<u8>,
    correction: Correction,
}

fn suite() -> Vec<Case> {
    let base = Case {
        name: "clean".into(),
        ppm: 0.0,
        delay: 0.0,
        compensate: false,
        hole: 0.0,
        period: 1.85,
        fill: false,
        api_delay: 0.0,
        first_hole: 1.05,
        variable_loss: false,
        stochastic_loss: None,
        correction: Correction::Nominal,
    };
    let mut cases = vec![
        base.clone(),
        Case {
            name: "clean_Payload".into(),
            correction: Correction::Payload,
            ..base.clone()
        },
    ];
    for correction in [
        Correction::Nominal,
        Correction::ClockOnly,
        Correction::Measured,
    ] {
        cases.push(Case {
            name: format!("actual_minus_17000_{correction:?}"),
            ppm: -17_000.0,
            correction,
            ..base.clone()
        });
    }
    for delay in [0.25, 0.75, 1.5, 3.0, 15.0] {
        for compensate in [false, true] {
            cases.push(Case {
                name: format!("fixed_{delay}s_compensated_{compensate}"),
                delay,
                compensate,
                ..base.clone()
            });
        }
    }
    for correction in [
        Correction::Nominal,
        Correction::ClockOnly,
        Correction::Measured,
    ] {
        cases.push(Case {
            name: format!("loss65_{correction:?}"),
            hole: 0.065,
            api_delay: 0.090,
            correction,
            ..base.clone()
        });
    }
    for hole in [0.020, 0.065, 0.120, 0.240] {
        cases.push(Case {
            name: format!("filled_{}ms_3p5pct", (hole * 1000.0) as u32),
            hole,
            period: hole / 0.035,
            fill: true,
            api_delay: hole + 0.025,
            ..base.clone()
        });
    }
    cases.push(Case {
        name: "filled65_measured".into(),
        hole: 0.065,
        api_delay: 0.090,
        fill: true,
        correction: Correction::Measured,
        ..base.clone()
    });
    for correction in [
        Correction::Nominal,
        Correction::Measured,
        Correction::TrueRate,
    ] {
        cases.push(Case {
            name: format!("combined_dropped_{correction:?}"),
            ppm: -17_000.0,
            delay: 0.750,
            hole: 0.065,
            api_delay: 0.090,
            correction,
            compensate: true,
            ..base.clone()
        });
    }
    for (correction, compensate) in [
        (Correction::Nominal, false),
        (Correction::TrueRate, false),
        (Correction::TrueRate, true),
        (Correction::Measured, true),
    ] {
        cases.push(Case {
            name: format!("combined_filled_{correction:?}_delaycomp_{compensate}"),
            ppm: -17_000.0,
            delay: 0.750,
            hole: 0.065,
            api_delay: 0.090,
            fill: true,
            correction,
            compensate,
            ..base.clone()
        });
    }
    cases.push(Case {
        name: "combined_reduced_loss10_true_delaycomp".into(),
        ppm: -17_000.0,
        delay: 0.750,
        hole: 0.010,
        api_delay: 0.035,
        correction: Correction::TrueRate,
        compensate: true,
        ..base.clone()
    });
    for correction in [
        Correction::Nominal,
        Correction::Measured,
        Correction::Payload,
    ] {
        cases.push(Case {
            name: format!("buffered750_{correction:?}"),
            api_delay: 0.750,
            period: 30.0,
            correction,
            ..base.clone()
        });
    }
    for hole in [0.001, 0.005, 0.010, 0.020, 0.040] {
        cases.push(Case {
            name: format!("reduced_drop_{}ms", (hole * 1000.0) as u32),
            hole,
            api_delay: hole + 0.025,
            ..base.clone()
        });
    }
    for hole in [0.400, 0.640, 1.000] {
        cases.push(Case {
            name: format!("long_filled_{}ms_3p5pct", (hole * 1000.0) as u32),
            hole,
            period: hole / 0.035,
            fill: true,
            api_delay: hole + 0.025,
            ..base.clone()
        });
    }
    for fraction in [0.07, 0.14, 0.28] {
        cases.push(Case {
            name: format!("filled65_loss_{}pct", (fraction * 100.0) as u32),
            hole: 0.065,
            period: 0.065 / fraction,
            fill: true,
            api_delay: 0.090,
            ..base.clone()
        });
    }
    for first in [0.25, 0.75] {
        for correction in [Correction::Nominal, Correction::Measured] {
            cases.push(Case {
                name: format!("phase_{first}s_filled65_{correction:?}"),
                first_hole: first,
                hole: 0.065,
                api_delay: 0.090,
                fill: true,
                correction,
                ..base.clone()
            });
        }
    }
    for correction in [
        Correction::Nominal,
        Correction::ClockOnly,
        Correction::Measured,
    ] {
        cases.push(Case {
            name: format!("varying_drop_{correction:?}"),
            hole: 0.065,
            api_delay: 0.090,
            variable_loss: true,
            correction,
            ..base.clone()
        });
    }
    for correction in [Correction::Nominal, Correction::Measured] {
        cases.push(Case {
            name: format!("varying_filled_{correction:?}"),
            hole: 0.065,
            api_delay: 0.090,
            variable_loss: true,
            fill: true,
            correction,
            ..base.clone()
        });
    }
    for (fill, correction) in [
        (false, Correction::Measured),
        (true, Correction::Measured),
        (true, Correction::TrueRate),
    ] {
        cases.push(Case {
            name: format!("varying_combined_fill{fill}_{correction:?}"),
            ppm: -17_000.0,
            delay: 0.750,
            compensate: true,
            hole: 0.065,
            api_delay: 0.090,
            variable_loss: true,
            fill,
            correction,
            ..base.clone()
        });
    }
    cases
}

fn source(ppm: f64, snr: f64, seed: u32) -> Arc<Vec<[i16; 2]>> {
    let rate = f64::from(RATE) * (1.0 + ppm * 1e-6);
    let audio: Vec<Vec<i16>> = CALLS
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
        .collect();
    let active_rms =
        (audio[0].iter().map(|&x| f64::from(x).powi(2)).sum::<f64>() / (12.64 * 12000.0)).sqrt();
    let sigma = 1000.0;
    let gain = sigma * (10000.0 * 10.0f64.powf(snr / 10.0) / rate).sqrt() / active_rms;
    let mut rng = seed;
    Arc::new(
        (0..(150.0 * rate) as usize)
            .map(|n| {
                let t = n as f64 / rate;
                let slot = &audio[(t / 15.0) as usize % CALLS.len()];
                let x = t.rem_euclid(15.0) * 12000.0;
                let i = x as usize;
                let a = (f64::from(slot[i])
                    + (f64::from(slot[(i + 1) % slot.len()]) - f64::from(slot[i])) * x.fract())
                    * gain;
                let mut uniform = || {
                    rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (f64::from(rng >> 8) + 1.0) / f64::from(1u32 << 24)
                };
                let (u, v) = (uniform(), uniform());
                let r = sigma * (-2.0 * u.ln()).sqrt();
                let p = std::f64::consts::TAU * (DIAL - CENTER) * t;
                let q = std::f64::consts::TAU * v;
                [
                    (a * p.cos() + r * q.cos()).round() as i16,
                    (a * p.sin() + r * q.sin()).round() as i16,
                ]
            })
            .collect(),
    )
}

struct Recorded {
    iq: Arc<Vec<[i16; 2]>>,
    case: Case,
    index: usize,
    seq: u32,
    last_arrival: i64,
    last_capture: i64,
    emitted: u64,
    lost: u64,
    events: Arc<Vec<(f64, f64)>>,
    event_cursor: usize,
}

impl Recorded {
    fn rate(&self) -> f64 {
        f64::from(RATE) * (1.0 + self.case.ppm * 1e-6)
    }
    fn capture_ns(&self) -> i64 {
        T0 + (self.index as f64 / self.rate() * 1e9).round() as i64
    }
    fn hole_length(&self, t: f64) -> f64 {
        self.case.hole
            * if self.case.variable_loss {
                [0.3, 1.0, 1.7][(t / 10.0) as usize % 3]
            } else {
                1.0
            }
    }
}

impl Link for Recorded {
    fn read(&mut self, _: &AtomicBool) -> std::io::Result<Message> {
        let mut body = Vec::with_capacity(BLOCK * 4);
        while body.len() < BLOCK * 4 && self.index < self.iq.len() {
            let t = self.index as f64 / self.rate();
            while self.event_cursor < self.events.len() && self.events[self.event_cursor].1 <= t {
                self.event_cursor += 1;
            }
            let hole = if self.case.stochastic_loss.is_some() {
                self.events
                    .get(self.event_cursor)
                    .is_some_and(|&(start, end)| start <= t && t < end)
            } else {
                self.case.hole > 0.0
                    && t >= self.case.first_hole
                    && (t - self.case.first_hole).rem_euclid(self.case.period) < self.hole_length(t)
            };
            let pair = self.iq[self.index];
            self.index += 1;
            if hole {
                self.lost += 1;
                if !self.case.fill {
                    continue;
                }
            }
            for sample in if hole { [0, 0] } else { pair } {
                body.extend_from_slice(&sample.to_le_bytes());
            }
        }
        if body.len() != BLOCK * 4 {
            return Err(std::io::ErrorKind::Interrupted.into());
        }
        let t = self.index as f64 / self.rate();
        let phase = (t - self.case.first_hole).rem_euclid(self.case.period);
        let api_delay = if self.case.variable_loss {
            self.hole_length(t) + 0.025
        } else {
            self.case.api_delay
        };
        let callback_wait = if self.case.stochastic_loss.is_some() {
            self.events
                .iter()
                .filter(|&&(start, end)| start <= t && t < end + 0.025)
                .map(|&(_, end)| end + 0.025 - t)
                .fold(0.0, f64::max)
        } else if t >= self.case.first_hole && phase < api_delay {
            api_delay - phase
        } else {
            0.0
        };
        let arrival = self.capture_ns() + ((self.case.delay + callback_wait) * 1e9).round() as i64;
        // FIFO delivery: a later acquisition can never overtake an older queued block.
        self.last_arrival = arrival.max(self.last_arrival);
        self.last_capture = self.capture_ns();
        assert!(self.last_arrival >= self.capture_ns());
        self.emitted += BLOCK as u64;
        self.seq += 1;
        Ok(Message {
            kind: MSG_INT16_IQ,
            seq: self.seq,
            body,
            arrival_ns: self.last_arrival,
        })
    }
    fn queued_bytes(&self) -> usize {
        0
    }
    fn write_gain(&mut self, _: u32, _: &Device, _: &Plan) -> std::io::Result<()> {
        Ok(())
    }
}

fn run(case: &Case, iq: Arc<Vec<[i16; 2]>>, snr: f64, seed: u32) -> Value {
    let events = Arc::new(stochastic_events(case.stochastic_loss, seed));
    let mut link = Recorded {
        iq,
        case: case.clone(),
        index: 0,
        seq: 0,
        last_arrival: 0,
        last_capture: T0,
        emitted: 0,
        lost: 0,
        events: events.clone(),
        event_cursor: 0,
    };
    let mut meter = rtltcp::RateMeter::new();
    // The startup measurement really consumes the first 30 seconds of this stream.
    let mut preview = Recorded {
        iq: link.iq.clone(),
        case: case.clone(),
        index: 0,
        seq: 0,
        last_arrival: 0,
        last_capture: T0,
        emitted: 0,
        lost: 0,
        events: events.clone(),
        event_cursor: 0,
    };
    while preview.index as f64 / preview.rate() < 30.0 {
        let m = preview.read(&AtomicBool::new(false)).unwrap();
        meter.push(m.arrival_ns, preview.emitted);
    }
    let measured = meter.rate().unwrap();
    let mut cfg = Config::new("virtual-clock", vec![ChannelSpec::new(Mode::Ft8, DIAL)]);
    cfg.protocol = Protocol::RtlTcp;
    cfg.rtl_calibrate = Duration::ZERO;
    cfg.rtl_minimum_arrival = false;
    cfg.rtl_infer_gaps = false;
    cfg.early_decode = false;
    cfg.waterfall = false;
    cfg.live.set_network_delay_ms(if case.compensate {
        case.delay * 1000.0
    } else {
        0.0
    });
    let mut p = Plan {
        rate: RATE,
        decimation: 0,
        center_hz: CENTER,
        active: vec![0],
    };
    let mut setup = RateSetup {
        ppm: 0.0,
        track: false,
        acquisition_hz: None,
    };
    match case.correction {
        Correction::Envelope => cfg.rtl_minimum_arrival = true,
        Correction::Measured | Correction::Payload => {
            cfg.rtl_calibrate = Duration::from_secs(30);
            cfg.rtl_minimum_arrival = true;
            cfg.rtl_infer_gaps = matches!(case.correction, Correction::Measured);
            if matches!(case.correction, Correction::Payload) {
                setup.acquisition_hz = Some(f64::from(RATE));
            }
            setup.track = true;
            // Same acceptance gate as rtltcp::measure_rate, including rare
            // long outages that make a 30-second fit implausibly low.
            if (measured / f64::from(RATE) - 1.0).abs() < 0.15 {
                let (n, ppm) = mfsk_core::iq::plannable_rate(measured).unwrap();
                p.rate = n;
                setup.ppm = ppm;
            }
            link = preview;
        }
        Correction::TrueRate => setup.ppm = case.ppm,
        _ => {}
    }
    let dev = Device {
        kind: rtltcp::DEVICE_RTL_TCP,
        max_rate: RATE,
        bandwidth_hz: f64::from(RATE),
        max_gain: 0,
        rates: vec![(0, RATE)],
    };
    let sync = Sync {
        can_control: false,
        gain: 0,
        device_hz: CENTER,
        iq_hz: CENTER,
    };
    let mut bank = WfBank::new();
    let mut live = Some(receiver(&cfg, &p, IqSampleFormat::Cs16, &mut bank, setup).unwrap());
    if matches!(case.correction, Correction::ClockOnly) {
        let l = live.take().unwrap();
        live = Some(Live {
            rx: l.rx.with_clock(
                mfsk_core::slotgrid::SampleClock::new(p.rate).with_max_slew_ppm(50_000),
            ),
            ..l
        });
    }
    let mut rows = Vec::new();
    let mut gaps = 0;
    let mut reanchors = Vec::new();
    let mut dropped = 0;
    let mut sink = |event| match event {
        Event::Decode(d) => rows.push(d),
        Event::Gap { .. } => gaps += 1,
        Event::Reanchor { by_s } => reanchors.push(by_s),
        Event::Status(s) => dropped += s.dropped_slots,
        _ => {}
    };
    assert!(matches!(
        stream_inner(
            &mut link,
            &AtomicBool::new(false),
            &cfg,
            &dev,
            &p,
            sync,
            None,
            &mut live,
            &mut bank,
            setup,
            &mut sink
        )
        .unwrap(),
        StreamEnd::Stopped
    ));
    settle(&mut live, &mut sink);
    assert_eq!(
        live.as_ref().unwrap().busy.load(Ordering::Relaxed),
        0,
        "decoder workers did not finish"
    );
    let rx = &live.as_ref().unwrap().rx;
    let clock_error = (rx.utc_of(rx.samples_in()).unwrap() - link.last_capture) as f64 * 1e-9;
    let mut decoded = std::collections::BTreeSet::new();
    let mut correct_utc = std::collections::BTreeSet::new();
    let mut kept = Vec::new();
    for d in &rows {
        for slot in 3..9 {
            let expected = format!("CQ {} FN42", CALLS[slot % CALLS.len()]);
            let true_utc = T0 + slot as i64 * 15_000_000_000;
            if d.text == expected
                && d.slot_utc_ns
                    .is_some_and(|t| (t - true_utc).abs() <= 30_000_000_000)
            {
                decoded.insert(slot);
                let utc_error = (d.slot_utc_ns.unwrap() - true_utc) as f64 * 1e-9;
                if utc_error == 0.0 {
                    correct_utc.insert(slot);
                }
                kept.push(json!({"source_slot":slot,"slot_utc_error_s":utc_error,"dt_s":d.dt_s,"freq_error_hz":d.freq_hz-(DIAL+1500.0),"snr_db":d.snr_db,"text":d.text}));
            }
        }
    }
    let result = json!({"case":case.name,"seed":seed,"target_snr":snr,"actual_ppm":case.ppm,
        "requested_rate_hz":RATE,"actual_rate_hz":link.rate(),"source_t0_ns":T0,"dial_offset_hz":DIAL-CENTER,
        "fixed_delay_ms":case.delay*1000.0,"delay_compensated":case.compensate,"hole_ms":case.hole*1000.0,"hole_period_ms":case.period*1000.0,
        "filled":case.fill,"api_delay_ms":case.api_delay*1000.0,"first_hole_s":case.first_hole,
        "variable_loss":case.variable_loss,"correction":format!("{:?}",case.correction),
        "stochastic_profile":case.stochastic_loss,"loss_events_s":events.as_ref(),
        "evaluation_loss_fraction":case.stochastic_loss.map(|_|events.iter().map(|&(a,b)|(b.min(135.0)-a.max(45.0)).max(0.0)).sum::<f64>()/90.0),
        "measured_rate_hz":measured,"measured_ppm":(measured/f64::from(RATE)-1.0)*1e6,"initial_rx_rate_hz":p.rate,
        "final_offset_ppm":rx.rate_offset_ppm(),"final_effective_rate_hz":f64::from(p.rate)*(1.0+rx.rate_offset_ppm()*1e-6),
        "opportunities":6,"decoded":decoded.len(),"correct_slot_utc":correct_utc.len(),"clock_error_end_s":clock_error,
        "gaps":gaps,"reanchors_s":reanchors,"worker_dropped_slots":dropped,"rows":kept});
    eprintln!(
        "{} seed {seed} {snr}dB: {}/6 decoded, {} correct UTC; clock {clock_error:+.3}s, measured {:.0}ppm",
        case.name,
        decoded.len(),
        correct_utc.len(),
        (measured / f64::from(RATE) - 1.0) * 1e6
    );
    result
}

#[test]
#[ignore = "controlled transport study; use --release --ignored --nocapture"]
fn acquisition_delivery_and_loss_study() {
    let path = std::env::var("MFSK_IQ_SIM_OUTPUT")
        .expect("set MFSK_IQ_SIM_OUTPUT to a writable JSON path");
    let seeds = std::env::var("MFSK_IQ_SIM_SEEDS")
        .ok()
        .and_then(|v| v.parse::<u32>().ok())
        .unwrap_or(3);
    let snrs: Vec<f64> = std::env::var("MFSK_IQ_SIM_SNRS")
        .unwrap_or_else(|_| "-10,-18".into())
        .split(',')
        .map(|s| s.parse().unwrap())
        .collect();
    let filter = std::env::var("MFSK_IQ_SIM_CASES").ok();
    let cases: Vec<_> = suite()
        .into_iter()
        .filter(|c| {
            filter
                .as_ref()
                .is_none_or(|f| f.split(',').any(|s| s == c.name))
        })
        .collect();
    assert!(
        !cases.is_empty(),
        "no experiment cases matched MFSK_IQ_SIM_CASES"
    );
    assert!(seeds > 0, "MFSK_IQ_SIM_SEEDS must be positive");
    let mut results = Vec::new();
    for snr in snrs {
        for seed in 1..=seeds {
            for ppm in [0.0, -17_000.0] {
                let group: Vec<_> = cases.iter().filter(|c| c.ppm == ppm).collect();
                if group.is_empty() {
                    continue;
                }
                let iq = source(ppm, snr, 99 + seed * 1009);
                for c in group {
                    results.push(run(c, iq.clone(), snr, seed));
                    std::fs::write(&path, serde_json::to_string_pretty(&results).unwrap()).unwrap();
                }
            }
        }
    }
    assert!(
        results.iter().all(|r| r["worker_dropped_slots"] == 0),
        "host decoding overload invalidates this experiment"
    );
    for r in results.iter().filter(|r| {
        matches!(
            r["case"].as_str(),
            Some("clean_Payload" | "buffered750_Payload")
        )
    }) {
        assert_eq!(r["decoded"], 6, "complete payload must remain decodable");
        assert_eq!(r["correct_slot_utc"], 6);
        assert_eq!(r["gaps"], 0, "buffering must not invent sample loss");
        assert!(r["reanchors_s"].as_array().unwrap().is_empty());
    }
    for r in results.iter().filter(|r| r["case"] == "clean") {
        assert_eq!(
            r["decoded"], 6,
            "baseline signal/noise must decode before drawing transport conclusions"
        );
        assert_eq!(r["correct_slot_utc"], 6);
        let target = r["target_snr"].as_f64().unwrap();
        assert!(
            r["rows"]
                .as_array()
                .unwrap()
                .iter()
                .all(|d| { (d["snr_db"].as_f64().unwrap() - target).abs() <= 3.0 }),
            "baseline IQ did not have the requested weak-signal SNR"
        );
    }
}

// Alternating renewal process: exponentially distributed healthy intervals,
// independent random outage lengths. Quiet mean sets expected loss duty to 3.5%.
// The schedule is independent of IQ noise and identical for every matched repair.
fn stochastic_events(profile: Option<u8>, seed: u32) -> Vec<(f64, f64)> {
    let Some(kind) = profile else {
        return Vec::new();
    };
    let mut state = seed
        .wrapping_mul(6361)
        .wrapping_add(0xabc123 + u32::from(kind) * 123457);
    let mut uniform = || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (f64::from(state >> 8) + 0.5) / f64::from(1u32 << 24)
    };
    let mean_duration = if kind == 2 { 2.5 } else { 0.065 };
    let mean_quiet = mean_duration * (1.0 - 0.035) / 0.035;
    let mut now = 0.0;
    let mut events = Vec::new();
    loop {
        now += -uniform().ln() * mean_quiet;
        if now >= 150.0 {
            break;
        }
        let duration = match kind {
            0 => 0.010 + 0.110 * uniform(),
            1 => {
                let tail = uniform() < 0.1;
                if tail {
                    0.200 + 0.450 * uniform()
                } else {
                    0.010 + 0.030 * uniform()
                }
            }
            2 => 1.0 + 3.0 * uniform(),
            _ => unreachable!(),
        };
        events.push((now, now + duration));
        now += duration;
    }
    events
}

#[test]
#[ignore = "stochastic paired transport experiment; --release --ignored --nocapture"]
fn stochastic_acquisition_study() {
    let path = std::env::var("MFSK_IQ_SIM_OUTPUT").expect("set MFSK_IQ_SIM_OUTPUT");
    let seeds = std::env::var("MFSK_IQ_SIM_SEEDS")
        .ok()
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(20);
    assert!(seeds > 0);
    let base = suite().remove(0);
    let mut cases = vec![
        base.clone(),
        Case {
            name: "clean_Payload".into(),
            correction: Correction::Payload,
            ..base.clone()
        },
    ];
    for (kind, name) in [(0, "bounded"), (1, "tail650ms"), (2, "tail4s")] {
        for (fill, correction) in [
            (false, Correction::Nominal),
            (false, Correction::Measured),
            (true, Correction::Nominal),
            (true, Correction::Envelope),
        ] {
            cases.push(Case {
                name: format!("stochastic_{name}_fill{fill}_{correction:?}"),
                stochastic_loss: Some(kind),
                hole: 0.065,
                fill,
                correction,
                ..base.clone()
            });
        }
    }
    let start_seed = std::env::var("MFSK_IQ_SIM_SEED_START")
        .ok()
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(1);
    assert!(start_seed > 0 && start_seed <= seeds);
    if let Ok(filter) = std::env::var("MFSK_IQ_SIM_CASES") {
        cases.retain(|c| filter.split(',').any(|n| n == c.name));
        assert!(!cases.is_empty());
    }
    let mut results = Vec::new();
    for seed in start_seed..=seeds {
        let iq = source(0.0, -18.0, 99 + seed * 1009);
        for c in &cases {
            results.push(run(c, iq.clone(), -18.0, seed));
            std::fs::write(&path, serde_json::to_string_pretty(&results).unwrap()).unwrap();
        }
    }
    assert!(results.iter().all(|r| r["worker_dropped_slots"] == 0));
    // Noise-induced baseline failures are valid data here, not transport errors.
    for r in results.iter().filter(|r| r["case"] == "clean") {
        assert_eq!(r["decoded"], r["correct_slot_utc"]);
        assert!(
            r["rows"]
                .as_array()
                .unwrap()
                .iter()
                .all(|d| (d["snr_db"].as_f64().unwrap() + 18.0).abs() <= 3.0)
        );
    }
}
