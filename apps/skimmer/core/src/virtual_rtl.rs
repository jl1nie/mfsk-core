// SPDX-License-Identifier: GPL-3.0-only
//! The decoding half of the `rtl_tcp` tests (#685, #686), on a virtual clock.
//!
//! `tests/fake_rtltcp.rs` keeps what needs a socket: the greeting, the format detection, the commands
//! the client writes. What needs time lives here: the stand-in server's IQ (`fake::RtlSignal`, the same
//! samples `FakeRtlServer` sends) is cut into messages like `rtltcp::Conn` cuts them, each stamped with
//! the arrival time the virtual clock gives it (the server delivers `scale` times the rate asked for; a
//! pause is a stretch of the clock with no messages while the band's time goes on), and handed to the
//! stream loop (`stream`, as `recorded_iq` hands it a capture). Nothing here reads the wall clock or
//! sleeps, so what a test asserts does not depend on the speed of the machine or on its clock.
use super::*;
use crate::fake::{Band, FAKE_RTL_RATE, Line, RtlSignal};
use crate::jtty::{JttyMessage, UpdateKind};
use crate::rtltcp::{RateGate, RateMeter};

/// Samples per message: `rtltcp::Conn` cuts the stream into 32 768 bytes (8-bit) or 65 536 bytes
/// (16-bit), which is the same count of samples.
const MESSAGE_SAMPLES: usize = 16_384;
/// A UTC second that is a whole number of 16 s cycles (and of 15 s ones), so a test says where in the
/// cycle the stream starts by the offset it adds.
const T0_S: i64 = 1_800_000_000;
const MESSAGE: &str = "CQ K1ABC CQ FN42";

fn band(amplitude: f32) -> Band {
    Band {
        cycle_s: 16,
        script: vec![Line {
            at_s: 1.0,
            hz: 1500.0,
            level: 1.0,
            text: MESSAGE.into(),
        }],
        jitter_s: 0.0,
        qsb: 0.0,
        amplitude,
        ..Band::default()
    }
}

/// A server that streams the stand-in band on a virtual clock, as a [`Link`].
struct VirtualRtl {
    signal: RtlSignal,
    kind: u32,
    /// Where the stream starts, UTC ns.
    t0_ns: i64,
    /// Seconds on the virtual clock since the start, pauses included.
    clock_s: f64,
    seq: u32,
    /// `(at_s, for_s)`: after `at_s` seconds of clock, one pause.
    pause: Option<(f64, f64)>,
    /// The stream ends (as a stop) when the clock reaches this.
    end_s: f64,
}

impl VirtualRtl {
    fn new(band: &Band, bits16: bool, scale: f64, start_into_cycle_s: f64, end_s: f64) -> Self {
        let t0_ns = T0_S * 1_000_000_000 + (start_into_cycle_s * 1e9).round() as i64;
        let mut signal = RtlSignal::new(band, bits16, scale, t0_ns as f64 * 1e-9);
        // What the client asks for first (`session_rtl`): the rate, then the centre.
        signal.set_rate(FAKE_RTL_RATE);
        signal.set_center(band.dial_hz - 25_000.0);
        Self {
            signal,
            kind: if bits16 { MSG_INT16_IQ } else { MSG_UINT8_IQ },
            t0_ns,
            clock_s: 0.0,
            seq: 0,
            pause: None,
            end_s,
        }
    }

    fn with_pause(mut self, at_s: f64, for_s: f64) -> Self {
        self.pause = Some((at_s, for_s));
        self
    }

    fn now_ns(&self) -> i64 {
        self.t0_ns + (self.clock_s * 1e9).round() as i64
    }

    fn bytes_per_sample(&self) -> usize {
        if self.kind == MSG_INT16_IQ { 4 } else { 2 }
    }
}

impl Link for VirtualRtl {
    fn read(&mut self, _: &AtomicBool) -> std::io::Result<Message> {
        if let Some((at_s, for_s)) = self.pause
            && self.clock_s >= at_s
        {
            self.pause = None;
            self.clock_s += for_s;
            self.signal.pause(for_s);
        }
        if self.clock_s >= self.end_s {
            return Err(std::io::ErrorKind::Interrupted.into());
        }
        let body = self.signal.block(MESSAGE_SAMPLES);
        // The server sends a block when its last sample exists.
        self.clock_s += MESSAGE_SAMPLES as f64 / self.signal.delivered_rate();
        self.seq += 1;
        Ok(Message {
            kind: self.kind,
            seq: self.seq,
            body,
            arrival_ns: self.now_ns(),
        })
    }
    fn queued_bytes(&self) -> usize {
        0
    }
    fn write_gain(&mut self, _: u32, _: &Device, _: &Plan) -> std::io::Result<()> {
        Ok(())
    }
}

#[derive(Default)]
struct Run {
    messages: Vec<JttyMessage>,
    /// `(rate, centre Hz)` of the stream that began.
    streaming: Vec<(u32, f64)>,
    reanchors: usize,
    gaps: usize,
}

/// `session_rtl` for a link that is not a socket: the first stream the client takes (`rtltcp::await_rate`,
/// then `rtltcp::measure_rate`, both counted on the arrival times instead of the wall clock), the plan
/// and the rate setup it makes of them, then the stream loop to the end of the link. Config as the
/// socket tests have it: the stand-in is exact and has no start-up phase, unless a test says otherwise.
fn run(link: &mut VirtualRtl, tweak: impl FnOnce(&mut Config)) -> Run {
    let mut cfg = Config::new(
        "virtual",
        vec![ChannelSpec::new(ChannelMode::Jtty, 14_090_000.0)],
    );
    cfg.protocol = Protocol::RtlTcp;
    cfg.rate = Some(FAKE_RTL_RATE);
    cfg.rtl_calibrate = Duration::ZERO;
    cfg.rtl_commands_floor = Duration::ZERO;
    tweak(&mut cfg);
    let rate = FAKE_RTL_RATE;
    let dev = Device {
        kind: rtltcp::DEVICE_RTL_TCP,
        max_rate: rate,
        bandwidth_hz: f64::from(rate),
        max_gain: 27,
        rates: vec![(0, rate)],
    };
    let mut setup = RateSetup::for_rtl(&cfg);
    if !cfg.rtl_calibrate.is_zero() && cfg.rtl_nominal_acquisition {
        setup.acquisition_hz = Some(f64::from(rate));
    }
    let mut p = plan::plan_of(
        &cfg.channels,
        &[true],
        cfg.center_hz,
        Some(rate),
        &dev,
        0.0,
        true,
    )
    .expect("a plan for the one channel");
    link.signal.set_center(p.center_hz);
    let stop = AtomicBool::new(false);

    // await_rate: windows of 2 s on arrival times until two in a row are at the rate asked for.
    let mut gate = RateGate::new(
        f64::from(rate) * link.bytes_per_sample() as f64,
        cfg.rtl_commands_floor,
    );
    while let Ok(m) = link.read(&stop) {
        if gate.push(m.arrival_ns, m.body.len()) {
            break;
        }
    }
    // measure_rate: skip, then a window, a line through arrival time against samples.
    if !cfg.rtl_calibrate.is_zero() {
        let skip = (cfg.rtl_calibrate / 4).min(Duration::from_secs(5));
        let start = link.now_ns();
        let (mut meter, mut samples) = (RateMeter::new(), 0u64);
        while ((link.now_ns() - start) as f64 * 1e-9) < (skip + cfg.rtl_calibrate).as_secs_f64() {
            let m = link.read(&stop).expect("the link outlasts the measurement");
            if ((m.arrival_ns - start) as f64 * 1e-9) < skip.as_secs_f64() {
                continue;
            }
            samples += MESSAGE_SAMPLES as u64;
            meter.push(m.arrival_ns, samples);
        }
        let measured = meter
            .rate()
            .filter(|r| (r / f64::from(rate) - 1.0).abs() < 0.15);
        if let Some((nominal, ppm)) = measured.and_then(mfsk_core::iq::plannable_rate) {
            p.rate = nominal;
            setup.ppm = ppm;
        }
    }

    let mut out = Run::default();
    out.streaming.push((p.rate, p.center_hz));
    let sync = Sync {
        can_control: true,
        gain: 0,
        device_hz: p.center_hz,
        iq_hz: p.center_hz,
    };
    let mut sink = |ev: Event| match ev {
        Event::Jtty(m) => out.messages.push(m),
        Event::Reanchor { .. } => out.reanchors += 1,
        Event::Gap { .. } => out.gaps += 1,
        _ => {}
    };
    let mut bank = WfBank::new();
    let end = stream(
        link, &stop, &cfg, &dev, &p, sync, None, &mut bank, &mut setup, &mut sink,
    )
    .expect("the stream ends as a stop");
    assert!(matches!(end, StreamEnd::Stopped));
    out
}

fn done<'a>(all: &'a [JttyMessage], text: &str) -> Option<&'a JttyMessage> {
    all.iter()
        .find(|m| m.kind == UpdateKind::Complete && m.text == text)
}

/// Seconds into its 16 s cycle that a message began, on the stream's UTC.
fn into_cycle(m: &JttyMessage) -> f64 {
    (m.start_utc_ns.unwrap().rem_euclid(16_000_000_000)) as f64 * 1e-9
}

/// `rsp_tcp -E -b 16` (16-bit IQ): the stream is decoded to the message it carries, at the frequency it
/// was sent on, with the rate and centre the plan makes of the request, and the clock is not stepped.
/// (The commands the client writes, and the format detection, are the socket tests'.)
#[test]
fn sixteen_bit_stream_is_decoded() {
    let mut link = VirtualRtl::new(&band(0.01), true, 1.0, 0.0, 48.0);
    let r = run(&mut link, |_| {});
    let cq = done(&r.messages, MESSAGE).unwrap_or_else(|| panic!("{:?}", r.messages));
    assert!((cq.freq_hz - 14_091_500.0).abs() < 4.0, "{}", cq.freq_hz);
    // Calibration zero keeps exactly the requested nominal rate; the IQ centre is 25 kHz under the dial.
    assert_eq!(r.streaming[0], (FAKE_RTL_RATE, 14_065_000.0));
    assert_eq!(r.reanchors, 0);
    assert_eq!(r.gaps, 0);
}

/// Plain `rtl_tcp` (8-bit IQ, 1/127 steps): the same band, a signal well above the steps, is decoded.
#[test]
fn eight_bit_stream_is_decoded() {
    let mut link = VirtualRtl::new(&band(0.05), false, 1.0, 0.0, 48.0);
    let r = run(&mut link, |_| {});
    let cq = done(&r.messages, MESSAGE).unwrap_or_else(|| panic!("{:?}", r.messages));
    assert!((cq.freq_hz - 14_091_500.0).abs() < 4.0, "{}", cq.freq_hz);
}

/// An RSP delivered 1.7 % fewer samples than the rate it was asked for (#685), which the receiver took
/// for a clock step every minute. The rate is measured at the start and used: the stream decodes, at the
/// rate really delivered, and the clock is not stepped.
#[test]
fn a_server_that_delivers_short_is_measured_and_followed() {
    const SCALE: f64 = 0.983;
    let mut link = VirtualRtl::new(&band(0.01), true, SCALE, 0.0, 60.0);
    let r = run(&mut link, |c| {
        c.rtl_calibrate = Duration::from_secs(6);
        // This fixture changes the acquisition clock, rather than deleting IQ.
        c.rtl_nominal_acquisition = false;
    });
    assert!(done(&r.messages, MESSAGE).is_some(), "{:?}", r.messages);
    // The nearest rate the receiver takes (within 500 ppm of the measurement), with the rest as an offset.
    let rate = f64::from(r.streaming[0].0);
    let real = f64::from(FAKE_RTL_RATE) * SCALE;
    assert!((rate / real - 1.0).abs() < 0.002, "{rate} against {real}");
    assert_eq!(r.reanchors, 0);
}

/// A server that pauses (an `rsp_tcp` restarting its RSP after a failure): the samples of the pause never
/// arrive and the band's time goes on. With the pause-loss heuristic enabled for this known source, the
/// receiver learns samples were lost (`IqReceiver::gap`), and later messages are at the time they were
/// sent: without that, the minimum-delay window held the old anchor for 30 s and put the stream the
/// pause too early (#686). A message is sent 1.0 s into every 16 s cycle of the stream's UTC.
#[test]
fn a_pause_of_the_server_does_not_displace_what_comes_after() {
    const PAUSE_S: f64 = 1.5;
    // Where in its cycle the stream starts decides which frames fall on which side of the pause.
    for start in [0.0, 5.3, 11.7] {
        let mut link =
            VirtualRtl::new(&band(0.01), true, 1.0, start, 80.0).with_pause(14.0, PAUSE_S);
        let started = link.t0_ns;
        let r = run(&mut link, |c| {
            c.rtl_calibrate = Duration::from_secs(6);
            c.rtl_infer_gaps = true;
        });
        assert!(r.gaps >= 1, "start {start}: the pause was not found");
        assert_eq!(r.reanchors, 0, "start {start}: a clock step");
        // Whole messages that began well after the pause was found and the cycle after it.
        let after: Vec<_> = r
            .messages
            .iter()
            .filter(|x| {
                x.kind == UpdateKind::Complete
                    && x.start_utc_ns.is_some_and(|t| t > started + 28_000_000_000)
            })
            .collect();
        assert!(after.len() >= 2, "start {start}: {:?}", r.messages);
        for m in after {
            // Seconds into its cycle: 1.0, not 1.0 - the pause (1.5 s). The stream's UTC is exact here, so
            // what is left is the JTTY receiver's own start estimate: measured 1.0006 s, once 0.9959 s,
            // over these starts; 0.05 s is ten times that and a thirtieth of the displacement caught.
            let into = into_cycle(m);
            assert!(
                (into - 1.0).abs() < 0.05,
                "start {start}: a message at {into:.3} s into its cycle"
            );
        }
    }
}
