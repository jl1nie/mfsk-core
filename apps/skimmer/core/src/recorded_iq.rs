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
            self.pending.extend(body);
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
    let mut file = std::io::BufReader::new(
        std::fs::File::open(std::env::var("MFSK_RTL_RECORDING").expect("set MFSK_RTL_RECORDING"))
            .unwrap(),
    );
    let mut header = [0u8; 12];
    file.read_exact(&mut header).unwrap();
    let start_ns = i64::from_le_bytes(header[..8].try_into().unwrap());
    let remaining = u32::from_le_bytes(header[8..].try_into().unwrap());
    let effective = number("MFSK_RTL_EFFECTIVE_RATE", 1_024_000.0);
    let dial = number("MFSK_RTL_DIAL", 7_074_000.0);
    let center = number("MFSK_RTL_CENTER", 7_016_000.0);
    let (rate, ppm) = mfsk_core::iq::plannable_rate(effective).unwrap();
    let receive_dial = dial;
    let mut cfg = Config::new("recorded", vec![ChannelSpec::new(Mode::Ft8, receive_dial)]);
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
        active: vec![0],
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
    let mut link = Recording {
        file,
        remaining,
        start_ns,
        pending: Default::default(),
        seq: 0,
        last_arrival: 0,
        skipped_bytes: 0,
        started: false,
        busy: live.as_ref().unwrap().busy.clone(),
        skip_s: number("MFSK_RTL_SKIP", 40.0),
        end_s: number("MFSK_RTL_END", 240.0),
    };
    let mut rows = Vec::new();
    let mut reanchors = Vec::new();
    let mut gaps = 0;
    let mut dropped = 0;
    let mut sink = |e| match e {
        Event::Decode(d) => rows.push(serde_json::json!({"text":d.text,"utc":d.slot_utc_ns,
            "frequency":d.freq_hz,"dt":d.dt_s,"snr":d.snr_db})),
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
    let result = serde_json::json!({"rate":effective,"receive_dial":receive_dial,
        "acquisition_hz":setup.acquisition_hz,
        "skip_s":link.skip_s,"end_s":link.end_s,"track":setup.track,"gaps":gaps,"reanchors":reanchors,"rows":rows});
    let output = std::env::var("MFSK_RTL_OUTPUT").expect("set MFSK_RTL_OUTPUT");
    std::fs::write(output, serde_json::to_string_pretty(&result).unwrap()).unwrap();
    eprintln!("recorded IQ: {} rows", rows.len());
}
