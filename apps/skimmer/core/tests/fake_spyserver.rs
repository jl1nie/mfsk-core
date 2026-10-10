//! #650: the skimmer, end to end over a real socket against the stand-in
//! SpyServer (`skimmer_core::fake`): handshake, settings, IQ at the rate the
//! plan chose, the clock, a JTTY channel, the message and its callsigns out.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use skimmer_core::fake::{Band, FakeServer, Line};
use skimmer_core::jtty::{JttyMessage, UpdateKind};
use skimmer_core::{ChannelMode, ChannelSpec, Config, Event};

fn line(at_s: f64, hz: f32, level: f32, text: &str) -> Line {
    Line {
        at_s,
        hz,
        level,
        text: text.into(),
    }
}

/// Run the skimmer against `band` until `want` holds of the finished messages,
/// or 40 s pass; returns every JTTY message reported.
fn run_against(band: Band, want: impl Fn(&[JttyMessage]) -> bool) -> Vec<JttyMessage> {
    let server = FakeServer::start("127.0.0.1:0", band).expect("listen");
    let cfg = Config::new(
        server.addr.to_string(),
        vec![ChannelSpec::new(ChannelMode::Jtty, 14_090_000.0)],
    );
    let stop = Arc::new(AtomicBool::new(false));
    let got: Arc<Mutex<Vec<JttyMessage>>> = Arc::default();
    let (flag, sink) = (stop.clone(), got.clone());
    let t = std::thread::spawn(move || {
        skimmer_core::run(&cfg, &flag, |ev| {
            if let Event::Jtty(m) = ev {
                sink.lock().unwrap().push(m);
            }
        });
    });
    let until = Instant::now() + Duration::from_secs(40);
    while Instant::now() < until {
        std::thread::sleep(Duration::from_millis(200));
        if want(&got.lock().unwrap()) {
            break;
        }
    }
    stop.store(true, Ordering::Relaxed);
    t.join().unwrap();
    drop(server);
    // The thread has ended and dropped its sink: this is the only owner.
    Arc::try_unwrap(got).unwrap().into_inner().unwrap()
}

fn done<'a>(all: &'a [JttyMessage], text: &str) -> Option<&'a JttyMessage> {
    all.iter()
        .find(|m| m.kind == UpdateKind::Complete && m.text == text)
}

/// A call and its answer, on the Rx frequency and on a side channel: each comes
/// out whole, at its own frequency, with its callsigns, and the one with a
/// sender is a station.
#[test]
fn a_qso_and_a_side_channel_come_out_of_the_stand_in_server() {
    let band = Band {
        cycle_s: 16,
        script: vec![
            line(1.0, 1500.0, 1.0, "CQ K1ABC FN42"),
            line(7.0, 1500.0, 0.8, "K1ABC JA1ABC"),
            line(3.0, 1350.0, 0.5, "CQ W9XYZ EN34"),
        ],
        // Steady, on the script's own times: the test asks for whole messages.
        jitter_s: 0.0,
        qsb: 0.0,
        ..Band::default()
    };
    let all = run_against(band, |m| {
        done(m, "CQ K1ABC FN42").is_some()
            && done(m, "K1ABC JA1ABC").is_some()
            && done(m, "CQ W9XYZ EN34").is_some()
    });
    let cq = done(&all, "CQ K1ABC FN42").unwrap_or_else(|| panic!("{all:?}"));
    assert!((cq.freq_hz - 14_091_500.0).abs() < 4.0, "{}", cq.freq_hz);
    assert_eq!(cq.sender(), Some(("K1ABC".into(), Some("FN42".into()))));
    // Typed text packs into 5-character frames, not call atoms: `calls` is for the
    // structured ones, and the sender is read from the text by WSJT-X's rule.
    let ans = done(&all, "K1ABC JA1ABC").unwrap_or_else(|| panic!("{all:?}"));
    assert_eq!(ans.sender().map(|s| s.0), Some("JA1ABC".to_string()));
    let side = done(&all, "CQ W9XYZ EN34").unwrap_or_else(|| panic!("{all:?}"));
    assert!(
        (side.freq_hz - 14_091_350.0).abs() < 6.0,
        "{}",
        side.freq_hz
    );
    // A message grows before it is over: its first report is not complete.
    assert!(
        all.iter()
            .any(|m| m.key == cq.key && m.kind == UpdateKind::Growing),
        "{all:?}"
    );
    // The clock was known by then: the start time is the wall clock's.
    let now = skimmer_core::now_ns();
    let t = cq.start_utc_ns.expect("a UTC start");
    assert!(
        (now - t).abs() < 60_000_000_000,
        "{} s ago",
        (now - t) / 1_000_000_000
    );
}
