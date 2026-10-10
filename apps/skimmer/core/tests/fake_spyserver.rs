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
    run_with(band, false, want).0
}

/// [`run_against`], with the waterfall on or off, and how many waterfall rows
/// the JTTY channel (channel 0) produced.
fn run_with(
    band: Band,
    waterfall: bool,
    want: impl Fn(&[JttyMessage]) -> bool,
) -> (Vec<JttyMessage>, usize) {
    let server = FakeServer::start("127.0.0.1:0", band).expect("listen");
    let mut cfg = Config::new(
        server.addr.to_string(),
        vec![ChannelSpec::new(ChannelMode::Jtty, 14_090_000.0)],
    );
    cfg.waterfall = waterfall;
    let rows = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let counted = rows.clone();
    let stop = Arc::new(AtomicBool::new(false));
    let got: Arc<Mutex<Vec<JttyMessage>>> = Arc::default();
    let (flag, sink) = (stop.clone(), got.clone());
    let t = std::thread::spawn(move || {
        skimmer_core::run(&cfg, &flag, |ev| match ev {
            Event::Jtty(m) => sink.lock().unwrap().push(m),
            Event::Waterfall(w) if w.channel == 0 => {
                counted.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
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
    let all = Arc::try_unwrap(got).unwrap().into_inner().unwrap();
    (all, rows.load(Ordering::Relaxed))
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
            line(1.0, 1500.0, 1.0, "CQ K1ABC CQ FN42"),
            line(7.0, 1500.0, 0.8, "K1ABC JA1ABC"),
            line(3.0, 1350.0, 0.5, "CQ W9XYZ CQ EN34"),
        ],
        // Steady, on the script's own times: the test asks for whole messages.
        jitter_s: 0.0,
        qsb: 0.0,
        ..Band::default()
    };
    let all = run_against(band, |m| {
        done(m, "CQ K1ABC CQ FN42").is_some()
            && done(m, "K1ABC JA1ABC").is_some()
            && done(m, "CQ W9XYZ CQ EN34").is_some()
    });
    let cq = done(&all, "CQ K1ABC CQ FN42").unwrap_or_else(|| panic!("{all:?}"));
    assert!((cq.freq_hz - 14_091_500.0).abs() < 4.0, "{}", cq.freq_hz);
    // A CQ is one call atom (and a grid): its call is in `calls`, and the sender is
    // the second word by WSJT-X's spotting rule (which finds no grid after `CQ`).
    assert_eq!(cq.calls, ["K1ABC"]);
    assert_eq!(cq.sender(), Some(("K1ABC".into(), None)));
    let ans = done(&all, "K1ABC JA1ABC").unwrap_or_else(|| panic!("{all:?}"));
    assert_eq!(ans.calls, ["K1ABC", "JA1ABC"]);
    assert_eq!(ans.sender().map(|s| s.0), Some("JA1ABC".to_string()));
    let side = done(&all, "CQ W9XYZ CQ EN34").unwrap_or_else(|| panic!("{all:?}"));
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

/// A JTTY channel has a waterfall too (#650): the audio the receiver thread
/// reads is drawn as well, and neither takes it from the other.
#[test]
fn a_jtty_channel_has_a_waterfall_and_still_decodes() {
    let band = Band {
        cycle_s: 12,
        script: vec![line(1.0, 1500.0, 1.0, "CQ K1ABC CQ")],
        jitter_s: 0.0,
        qsb: 0.0,
        ..Band::default()
    };
    let (all, rows) = run_with(band, true, |m| done(m, "CQ K1ABC CQ").is_some());
    assert!(done(&all, "CQ K1ABC CQ").is_some(), "{all:?}");
    // About six rows a second reach the window; a few seconds of them is plenty.
    assert!(rows >= 10, "{rows} waterfall rows");
}
