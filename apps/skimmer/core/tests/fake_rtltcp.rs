//! #685: the skimmer over the `rtl_tcp` protocol, against the stand-in `rtl_tcp` / `rsp_tcp` server
//! (`skimmer_core::fake::FakeRtlServer`) over a real socket: the greeting, 8-bit and 16-bit
//! detection, the commands the client writes and their order, and a format that does not match
//! refused with its reason.
//!
//! These assert protocol facts only. The stand-in paces its IQ by the wall clock, and a test that
//! asserted what the receiver made of it (a message's time, a measured rate, a clock step) would
//! depend on the speed and the clock of the machine. Those tests (16-bit and 8-bit streams decoded,
//! a server that delivers short, a server that pauses) run the same IQ on a virtual clock, in
//! `src/virtual_rtl.rs`. Every wait here ends as soon as its fact is in, and is bounded.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mfsk_core::iq::IqSampleFormat;
use skimmer_core::fake::{Band, FAKE_RTL_RATE, FakeRtlServer};
use skimmer_core::rtltcp::{Conn, RtlFormat};
use skimmer_core::spyserver::{MSG_INT16_IQ, MSG_UINT8_IQ};
use skimmer_core::{ChannelMode, ChannelSpec, Config, Event, Protocol};

/// How long a fact may take to arrive before the test says it did not: a bound, not a wait.
const BOUND: Duration = Duration::from_secs(20);

#[derive(Default)]
struct Run {
    errors: Vec<String>,
    /// How many streams began.
    streaming: usize,
    messages: usize,
}

/// Run the skimmer on `server` until `want` holds of what it has said, or `BOUND` passes.
fn run(
    server: &FakeRtlServer,
    tweak: impl FnOnce(&mut Config),
    want: impl Fn(&Run) -> bool,
) -> Run {
    let mut cfg = Config::new(
        server.addr.to_string(),
        vec![ChannelSpec::new(ChannelMode::Jtty, 14_090_000.0)],
    );
    cfg.protocol = Protocol::RtlTcp;
    cfg.rate = Some(FAKE_RTL_RATE);
    cfg.retry = Duration::from_millis(300);
    cfg.rtl_calibrate = Duration::ZERO;
    cfg.rtl_commands_floor = Duration::ZERO;
    tweak(&mut cfg);
    let stop = Arc::new(AtomicBool::new(false));
    let got: Arc<Mutex<Run>> = Arc::default();
    let (flag, sink) = (stop.clone(), got.clone());
    let t = std::thread::spawn(move || {
        skimmer_core::run(&cfg, &flag, |ev| match ev {
            Event::Jtty(_) => sink.lock().unwrap().messages += 1,
            Event::Disconnected { error } => sink.lock().unwrap().errors.push(error),
            Event::Streaming { .. } => sink.lock().unwrap().streaming += 1,
            _ => {}
        });
    });
    let until = Instant::now() + BOUND;
    while Instant::now() < until && !want(&got.lock().unwrap()) {
        std::thread::sleep(Duration::from_millis(10));
    }
    stop.store(true, Ordering::Relaxed);
    t.join().unwrap();
    Arc::try_unwrap(got).ok().unwrap().into_inner().unwrap()
}

/// Wait (bounded) until the server has logged `n` commands, and return them.
fn commands(server: &FakeRtlServer, n: usize) -> Vec<(u8, u32)> {
    let until = Instant::now() + BOUND;
    while Instant::now() < until && server.commands.lock().unwrap().len() < n {
        std::thread::sleep(Duration::from_millis(10));
    }
    server.commands.lock().unwrap().clone()
}

/// Verifies: `rsp_tcp -E -b 16` says so in its greeting (`RTL0` then the `RSP0` capabilities
/// struct), the client reads it as 16-bit IQ with the hardware version, and the IQ follows the
/// struct, in messages of the 16-bit kind and size.
#[test]
fn rsp_tcp_extended_mode_is_detected_as_16_bit() {
    let server = FakeRtlServer::start("127.0.0.1:0", Band::default(), true).unwrap();
    let mut c = Conn::connect(&server.addr.to_string(), RtlFormat::Auto).unwrap();
    assert_eq!(
        (
            c.hello.tuner,
            c.hello.gains,
            c.hello.format,
            c.hello.rsp_hardware
        ),
        (5, 28, IqSampleFormat::Cs16, Some(1))
    );
    let m = c.read(&AtomicBool::new(false)).unwrap();
    assert_eq!((m.kind, m.body.len()), (MSG_INT16_IQ, 65_536));
}

/// Verifies: plain `rtl_tcp` sends no struct, so the client reads 8-bit IQ with no hardware
/// version, and its first message is of the 8-bit kind and size (the four bytes read to look for a
/// struct are not lost, or the first message would be short).
#[test]
fn plain_rtl_tcp_is_detected_as_8_bit() {
    let server = FakeRtlServer::start("127.0.0.1:0", Band::default(), false).unwrap();
    let mut c = Conn::connect(&server.addr.to_string(), RtlFormat::Auto).unwrap();
    assert_eq!(
        (
            c.hello.tuner,
            c.hello.gains,
            c.hello.format,
            c.hello.rsp_hardware
        ),
        (5, 28, IqSampleFormat::Cu8, None)
    );
    let m = c.read(&AtomicBool::new(false)).unwrap();
    assert_eq!((m.kind, m.body.len()), (MSG_UINT8_IQ, 32_768));
}

/// Verifies: what the skimmer writes to a 16-bit server is the sample rate, then the frequency
/// (the IQ centre is 25 kHz under the dial, the plan's default), then automatic gain (no `--gain`),
/// in that order, and it reports no error. (What it decodes from the stream is `virtual_rtl`'s.)
#[test]
fn the_commands_written_to_a_16_bit_server_are_rate_frequency_gain_mode() {
    let server = FakeRtlServer::start("127.0.0.1:0", Band::default(), true).unwrap();
    let r = run(&server, |_| {}, |_| logged(&server, 3));
    let cmds = server.commands.lock().unwrap().clone();
    assert_eq!(
        &cmds[..3],
        &[(0x02, FAKE_RTL_RATE), (0x01, 14_065_000), (0x03, 0)]
    );
    assert!(r.errors.is_empty(), "{:?}", r.errors);
}

/// True once the server has logged `n` commands: the `run` condition that ends it at once.
fn logged(server: &FakeRtlServer, n: usize) -> bool {
    server.commands.lock().unwrap().len() >= n
}

/// Verifies: a configured gain on an 8-bit server is written as manual mode plus the step, after
/// the rate and the frequency.
#[test]
fn a_gain_step_is_written_as_manual_mode_and_the_step() {
    let server = FakeRtlServer::start("127.0.0.1:0", Band::default(), false).unwrap();
    run(&server, |c| c.gain = Some(7), |_| logged(&server, 4));
    let cmds = commands(&server, 4);
    assert_eq!(
        &cmds[..4],
        &[
            (0x02, FAKE_RTL_RATE),
            (0x01, 14_065_000),
            (0x03, 1),
            (0x0d, 7)
        ]
    );
}

/// Verifies: a format chosen by the user that the server does not send is refused, says why (how
/// to start the server), and nothing is decoded from the mismatched stream.
#[test]
fn a_format_that_does_not_match_the_server_is_refused_with_the_reason() {
    let server = FakeRtlServer::start("127.0.0.1:0", Band::default(), false).unwrap();
    let r = run(
        &server,
        |c| c.rtl_format = RtlFormat::I16,
        |r| !r.errors.is_empty(),
    );
    assert!(
        r.errors[0].contains("-E -b 16"),
        "the error should say how to start the server: {:?}",
        r.errors
    );
    assert_eq!((r.streaming, r.messages), (0, 0));
}
