// SPDX-License-Identifier: GPL-3.0-only
//! Opt-in, faster-than-real-time diagnosis of record.py captures.
use super::*;
use std::io::Read;
use std::sync::Arc;

struct Recording {
    file: std::io::BufReader<std::fs::File>,
    remaining: u32,
    start_ns: i64,
    pending: std::collections::VecDeque<u8>,
    seq: u32,
    last_arrival: i64,
    skipped_bytes: usize,
    started: bool,
    busy: Arc<std::sync::atomic::AtomicUsize>,
    skip_s: f64,
    end_s: f64,
    /// Harness-only candidates D / E (RTLTCP_EVAL_PLAN.md section 5): zero samples put into the stream.
    /// `insert`: zeros per delivered sample, spread evenly (D). `fill[i]`: zeros before record `i` (E).
    insert: f64,
    insert_acc: f64,
    fill: Vec<usize>,
    index: usize,
}

const ZERO_SAMPLE: [u8; 2] = [128, 128];

/// (elapsed, payload bytes) of every record of a recording, without reading the payloads.
fn scan(path: &str) -> Vec<(f64, usize)> {
    let mut file = std::io::BufReader::new(std::fs::File::open(path).unwrap());
    let mut header = [0u8; 12];
    file.read_exact(&mut header).unwrap();
    let n = u32::from_le_bytes(header[8..].try_into().unwrap());
    let mut out = Vec::new();
    for _ in 0..n {
        if file.read_exact(&mut header).is_err() {
            break;
        }
        let size = u32::from_le_bytes(header[8..].try_into().unwrap());
        out.push((
            f64::from_le_bytes(header[..8].try_into().unwrap()),
            size as usize,
        ));
        std::io::Seek::seek_relative(&mut file, i64::from(size)).unwrap();
    }
    out
}

/// Candidate E: where a pause in the arrivals was samples lost rather than delayed, decided with
/// `lookahead` seconds of what arrived after it. A pause is a gap between records longer than
/// `threshold` s; it was delayed (nothing to fill) when the records in the next `lookahead` s carry
/// more than 1.1 times what `rate` would deliver in that time (the catch-up burst); else lost, and
/// `(gap - usual interval) * nominal` zero samples go in before the record that ended it.
fn pause_fill(
    records: &[(f64, usize)],
    threshold: f64,
    lookahead: f64,
    rate: f64,
    nominal: f64,
) -> Vec<usize> {
    let mut gaps: Vec<f64> = records.windows(2).map(|w| w[1].0 - w[0].0).collect();
    gaps.sort_by(f64::total_cmp);
    let usual = gaps.get(gaps.len() / 2).copied().unwrap_or(0.05);
    let mut fill = vec![0; records.len()];
    for i in 1..records.len() {
        let gap = records[i].0 - records[i - 1].0;
        if gap <= threshold {
            continue;
        }
        let carried: usize = records[i..]
            .iter()
            .take_while(|r| r.0 <= records[i].0 + lookahead)
            .map(|r| r.1 / 2)
            .sum();
        if (carried as f64) <= 1.1 * rate * lookahead {
            fill[i] = ((gap - usual) * nominal).round() as usize;
        }
    }
    fill
}

impl Link for Recording {
    fn read(&mut self, _: &AtomicBool) -> std::io::Result<Message> {
        while self.busy.load(Ordering::Relaxed) > 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
        loop {
            if self.pending.len() >= 32_768 {
                self.seq += 1;
                return Ok(Message {
                    kind: MSG_UINT8_IQ,
                    seq: self.seq,
                    body: self.pending.drain(..32_768).collect(),
                    arrival_ns: self.last_arrival,
                });
            }
            if self.remaining == 0 {
                break;
            }
            let mut header = [0u8; 12];
            self.file.read_exact(&mut header)?;
            let elapsed = f64::from_le_bytes(header[..8].try_into().unwrap());
            let size = u32::from_le_bytes(header[8..].try_into().unwrap()) as usize;
            let mut body = vec![0; size];
            self.file.read_exact(&mut body)?;
            self.remaining -= 1;
            if elapsed < self.skip_s {
                self.skipped_bytes += size;
                continue;
            }
            if elapsed > self.end_s {
                break;
            }
            if !self.started {
                if !self.skipped_bytes.is_multiple_of(2) {
                    body.remove(0);
                }
                self.started = true;
            }
            self.last_arrival = self.start_ns + (elapsed * 1e9).round() as i64;
            let record = self.index;
            self.index += 1;
            for _ in 0..self.fill.get(record).copied().unwrap_or(0) {
                self.pending.extend(ZERO_SAMPLE);
            }
            if self.insert > 0.0 {
                for block in body.chunks(2048) {
                    self.pending.extend(block);
                    self.insert_acc += self.insert * (block.len() / 2) as f64;
                    while self.insert_acc >= 1.0 {
                        self.pending.extend(ZERO_SAMPLE);
                        self.insert_acc -= 1.0;
                    }
                }
            } else {
                self.pending.extend(body);
            }
        }
        Err(std::io::ErrorKind::Interrupted.into())
    }
    fn queued_bytes(&self) -> usize {
        0
    }
    fn write_gain(&mut self, _: u32, _: &Device, _: &Plan) -> std::io::Result<()> {
        Ok(())
    }
}

#[test]
#[ignore = "requires MFSK_RTL_RECORDING; diagnostic, not a regression assertion"]
fn decode_recorded_iq() {
    let number =
        |key: &str, default: f64| std::env::var(key).map_or(default, |v| v.parse::<f64>().unwrap());
    let path = std::env::var("MFSK_RTL_RECORDING").expect("set MFSK_RTL_RECORDING");
    let open = |busy: Arc<std::sync::atomic::AtomicUsize>, skip_s: f64, end_s: f64| {
        let mut file = std::io::BufReader::new(std::fs::File::open(&path).unwrap());
        let mut header = [0u8; 12];
        file.read_exact(&mut header).unwrap();
        Recording {
            file,
            remaining: u32::from_le_bytes(header[8..].try_into().unwrap()),
            start_ns: i64::from_le_bytes(header[..8].try_into().unwrap()),
            pending: Default::default(),
            seq: 0,
            last_arrival: 0,
            skipped_bytes: 0,
            started: false,
            busy,
            skip_s,
            end_s,
            insert: 0.0,
            insert_acc: 0.0,
            fill: Vec::new(),
            index: 0,
        }
    };
    // MFSK_RTL_MEASURE=1: the rate the client's own start-up calibration (rtltcp::measure_rate: skip,
    // then a 30 s window, a line through arrival time against samples) would take from this recording,
    // in place of MFSK_RTL_EFFECTIVE_RATE. MFSK_RTL_MEASURE_AT starts the window that many seconds into
    // the recording (default 0), MFSK_RTL_MEASURE_WINDOW sets its length (30).
    // (pauses are taken out of the time axis by RateMeter itself, candidate F).
    let zero_insert = number("MFSK_RTL_ZERO_INSERT", 0.0) != 0.0;
    let measured = (number("MFSK_RTL_MEASURE", 0.0) != 0.0).then(|| {
        let (at, skip, window) = (
            number("MFSK_RTL_MEASURE_AT", 0.0),
            5.0,
            number("MFSK_RTL_MEASURE_WINDOW", 30.0),
        );
        let mut pre = open(Default::default(), 0.0, at + skip + window);
        let origin = pre.start_ns;
        let (mut meter, mut samples) = (rtltcp::RateMeter::new(), 0u64);
        while let Ok(m) = pre.read(&AtomicBool::new(false)) {
            if ((m.arrival_ns - origin) as f64) < (at + skip) * 1e9 {
                continue;
            }
            samples += (m.body.len() / 2) as u64;
            meter.push(m.arrival_ns, samples);
        }
        meter
            .rate()
            .expect("measure_rate: no rate from the recording")
    });
    let effective = match measured {
        Some(r) if !zero_insert => r,
        _ => number("MFSK_RTL_EFFECTIVE_RATE", 1_024_000.0),
    };
    let dial = number("MFSK_RTL_DIAL", 7_074_000.0);
    let center = number("MFSK_RTL_CENTER", 7_016_000.0);
    let (rate, ppm) = mfsk_core::iq::plannable_rate(effective).unwrap();
    let receive_dial = dial;
    // MFSK_RTL_CHANNELS="ft8:7074000,ft8:7041000,ft4:7047500" decodes several channels of one stream (the
    // synth_rtltcp corpus); unset, one FT8 channel at MFSK_RTL_DIAL.
    let channels: Vec<ChannelSpec> = match std::env::var("MFSK_RTL_CHANNELS") {
        Ok(list) => list
            .split(',')
            .map(|item| {
                let (mode, hz) = item
                    .split_once(':')
                    .expect("MFSK_RTL_CHANNELS: mode:dial_hz,...");
                let mode = match mode {
                    "ft8" => Mode::Ft8,
                    "ft4" => Mode::Ft4,
                    other => panic!("MFSK_RTL_CHANNELS: unsupported mode {other}"),
                };
                ChannelSpec::new(mode, hz.parse::<f64>().unwrap())
            })
            .collect(),
        Err(_) => vec![ChannelSpec::new(Mode::Ft8, receive_dial)],
    };
    let n_channels = channels.len();
    let mut cfg = Config::new("recorded", channels);
    cfg.protocol = Protocol::RtlTcp;
    cfg.rtl_calibrate = Duration::ZERO;
    cfg.rtl_minimum_arrival = number("MFSK_RTL_MIN_ARRIVAL", 1.0) != 0.0;
    cfg.rtl_infer_gaps = number("MFSK_RTL_INFER_GAPS", 0.0) != 0.0;
    cfg.early_decode = false;
    cfg.waterfall = false;
    let p = Plan {
        rate,
        decimation: 0,
        center_hz: center,
        active: (0..n_channels).collect(),
    };
    let setup = RateSetup {
        ppm,
        track: number("MFSK_RTL_TRACK", 0.0) != 0.0,
        acquisition_hz: (number("MFSK_RTL_PIN_ACQUISITION", 0.0) != 0.0).then_some(1_024_000.0),
    };
    let dev = Device {
        kind: rtltcp::DEVICE_RTL_TCP,
        max_rate: 1_024_000,
        bandwidth_hz: 1_024_000.0,
        max_gain: 0,
        rates: vec![(0, 1_024_000)],
    };
    let sync = Sync {
        can_control: false,
        gain: 0,
        device_hz: center,
        iq_hz: center,
    };
    let mut bank = WfBank::new();
    let mut live = Some(receiver(&cfg, &p, IqSampleFormat::Cu8, &mut bank, setup).unwrap());
    let mut link = open(
        live.as_ref().unwrap().busy.clone(),
        number("MFSK_RTL_SKIP", 40.0),
        number("MFSK_RTL_END", 240.0),
    );
    // D: the measured (or MFSK_RTL_INSERT_RATE) deficit made up by zeros spread evenly; the receiver
    // keeps the nominal rate. E (MFSK_RTL_PAUSE_FILL=1): and the pauses judged lost are zero-filled.
    if zero_insert {
        let r = measured.unwrap_or_else(|| number("MFSK_RTL_INSERT_RATE", 1_024_000.0));
        link.insert = (1_024_000.0 / r - 1.0).max(0.0);
        if number("MFSK_RTL_PAUSE_FILL", 0.0) != 0.0 {
            link.fill = pause_fill(
                &scan(&path),
                0.5,
                number("MFSK_RTL_LOOKAHEAD", 1.5),
                r,
                1_024_000.0,
            );
        }
    }
    let mut rows = Vec::new();
    let mut reanchors = Vec::new();
    let mut gaps = 0;
    let mut dropped = 0;
    let mut sink = |e| match e {
        Event::Decode(d) => rows.push(
            serde_json::json!({"mode":format!("{:?}", d.mode).to_uppercase(),
            "dial":d.dial_hz,"text":d.text,"utc":d.slot_utc_ns,
            "frequency":d.freq_hz,"dt":d.dt_s,"snr":d.snr_db}),
        ),
        Event::Reanchor { by_s } => reanchors.push(by_s),
        Event::Gap { .. } => gaps += 1,
        Event::Status(s) => dropped += s.dropped_slots,
        _ => {}
    };
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
        &mut sink,
    )
    .unwrap();
    settle(&mut live, &mut sink);
    assert_eq!(dropped, 0, "offline decoder queue lost slots");
    let result = serde_json::json!({"rate":effective,"measured":measured,"insert":link.insert,"filled":link.fill.iter().sum::<usize>(),"receive_dial":receive_dial,
        "acquisition_hz":setup.acquisition_hz,
        "skip_s":link.skip_s,"end_s":link.end_s,"track":setup.track,"gaps":gaps,"reanchors":reanchors,"rows":rows});
    let output = std::env::var("MFSK_RTL_OUTPUT").expect("set MFSK_RTL_OUTPUT");
    std::fs::write(output, serde_json::to_string_pretty(&result).unwrap()).unwrap();
    eprintln!("recorded IQ: {} rows", rows.len());
}
