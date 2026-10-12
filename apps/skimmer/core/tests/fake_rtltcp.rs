//! #685: the skimmer over the `rtl_tcp` protocol, end to end over a real socket against the
//! stand-in `rtl_tcp` / `rsp_tcp` server (`skimmer_core::fake::FakeRtlServer`): the greeting,
//! 8-bit and 16-bit IQ, the commands written, a format that does not match refused with its
//! reason, and a JTTY message out of the stream.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use skimmer_core::fake::{Band, FAKE_RTL_RATE, FakeRtlServer, Line};
use skimmer_core::jtty::{JttyMessage, UpdateKind};
use skimmer_core::rtltcp::RtlFormat;
use skimmer_core::{ChannelMode, ChannelSpec, Config, Event, Protocol};

fn band(amplitude: f32) -> Band {
    Band {
        cycle_s: 16,
        script: vec![Line {
            at_s: 1.0,
            hz: 1500.0,
            level: 1.0,
            text: "CQ K1ABC CQ FN42".into(),
        }],
        jitter_s: 0.0,
        qsb: 0.0,
        amplitude,
        ..Band::default()
    }
}

#[derive(Default)]
struct Run {
    messages: Vec<JttyMessage>,
    errors: Vec<String>,
    /// `(rate, centre Hz)` of each stream that began.
    streaming: Vec<(u32, f64)>,
}

/// Run the skimmer on `server` until `want` holds of the messages, or 45 s pass.
fn run(
    server: &FakeRtlServer,
    tweak: impl FnOnce(&mut Config),
    want: impl Fn(&[JttyMessage], &[String]) -> bool,
) -> Run {
    let mut cfg = Config::new(
        server.addr.to_string(),
        vec![ChannelSpec::new(ChannelMode::Jtty, 14_090_000.0)],
    );
    cfg.protocol = Protocol::RtlTcp;
    cfg.rate = Some(FAKE_RTL_RATE);
    cfg.retry = Duration::from_millis(300);
    tweak(&mut cfg);
    let stop = Arc::new(AtomicBool::new(false));
    let got: Arc<Mutex<Run>> = Arc::default();
    let (flag, sink) = (stop.clone(), got.clone());
    let t = std::thread::spawn(move || {
        skimmer_core::run(&cfg, &flag, |ev| match ev {
            Event::Jtty(m) => sink.lock().unwrap().messages.push(m),
            Event::Disconnected { error } => sink.lock().unwrap().errors.push(error),
            Event::Streaming {
                rate, center_hz, ..
            } => sink.lock().unwrap().streaming.push((rate, center_hz)),
            _ => {}
        });
    });
    let until = Instant::now() + Duration::from_secs(45);
    while Instant::now() < until {
        std::thread::sleep(Duration::from_millis(200));
        let g = got.lock().unwrap();
        if want(&g.messages, &g.errors) {
            break;
        }
    }
    stop.store(true, Ordering::Relaxed);
    t.join().unwrap();
    Arc::try_unwrap(got).ok().unwrap().into_inner().unwrap()
}

fn done<'a>(all: &'a [JttyMessage], text: &str) -> Option<&'a JttyMessage> {
    all.iter()
        .find(|m| m.kind == UpdateKind::Complete && m.text == text)
}

fn has_cq(m: &[JttyMessage], _: &[String]) -> bool {
    done(m, "CQ K1ABC CQ FN42").is_some()
}

/// `rsp_tcp -E -b 16`: the capabilities struct says 16-bit, the skimmer decodes the stream, and
/// what it wrote to the server is the sample rate, then the frequency, then the gain mode.
#[test]
fn sixteen_bit_stream_is_decoded_and_the_commands_are_written() {
    let server = FakeRtlServer::start("127.0.0.1:0", band(0.01), true).unwrap();
    let r = run(&server, |_| {}, has_cq);
    let cq = done(&r.messages, "CQ K1ABC CQ FN42")
        .unwrap_or_else(|| panic!("{:?} / {:?}", r.messages, r.errors));
    assert!((cq.freq_hz - 14_091_500.0).abs() < 4.0, "{}", cq.freq_hz);
    let cmds = server.commands.lock().unwrap().clone();
    // Sample rate first; the IQ centre is 25 kHz under the dial (the plan's default); then
    // automatic gain (no `--gain`), written after the frequency.
    assert_eq!(cmds.first(), Some(&(0x02, FAKE_RTL_RATE)), "{cmds:?}");
    assert_eq!(cmds.get(1), Some(&(0x01, 14_065_000)), "{cmds:?}");
    assert_eq!(cmds.get(2), Some(&(0x03, 0)), "{cmds:?}");
    assert_eq!(r.streaming.first(), Some(&(FAKE_RTL_RATE, 14_065_000.0)));
}

/// Plain `rtl_tcp`: no struct, 8-bit IQ (the four bytes read to look for one are not lost), and
/// a configured gain is written as manual mode plus the step.
#[test]
fn eight_bit_stream_is_decoded_and_a_gain_step_is_manual() {
    // 8-bit IQ is 1/127 steps: a signal well above them, as the noise is ~1 step.
    let server = FakeRtlServer::start("127.0.0.1:0", band(0.05), false).unwrap();
    let r = run(&server, |c| c.gain = Some(7), has_cq);
    assert!(
        done(&r.messages, "CQ K1ABC CQ FN42").is_some(),
        "{:?} / {:?}",
        r.messages,
        r.errors
    );
    let cmds = server.commands.lock().unwrap().clone();
    assert!(
        cmds.contains(&(0x03, 1)) && cmds.contains(&(0x0d, 7)),
        "{cmds:?}"
    );
}

/// A format chosen by the user that the server does not send is refused, and says why.
#[test]
fn a_format_that_does_not_match_the_server_is_refused_with_the_reason() {
    let server = FakeRtlServer::start("127.0.0.1:0", band(0.05), false).unwrap();
    let r = run(
        &server,
        |c| c.rtl_format = RtlFormat::I16,
        |_, errors| !errors.is_empty(),
    );
    assert!(
        r.errors[0].contains("-E -b 16"),
        "the error should say how to start the server: {:?}",
        r.errors
    );
    assert!(r.messages.is_empty());
}
