// SPDX-License-Identifier: GPL-3.0-only
//! The PSK Reporter sender over a real UDP socket: a listener on the loopback stands in for the
//! collector. What the protocol page asks of a client is checked on the datagrams that arrive:
//! one source port for the whole session, the templates in the first three datagrams and no
//! more until the hour is up, `informationSource` 1, a sequence that counts spots, a callsign
//! once per band, and no datagram before the interval.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use skimmer_core::pskreporter::{Endpoint, PskConfig, PskReporter, Spot};

fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_be_bytes([b[o], b[o + 1]])
}
fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_be_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// `(set id, body)` of each set of a message.
fn sets(p: &[u8]) -> Vec<(u16, &[u8])> {
    let (mut out, mut o) = (Vec::new(), 16);
    while o + 4 <= p.len() {
        let len = u16_at(p, o + 2) as usize;
        if len < 4 || o + len > p.len() {
            break;
        }
        out.push((u16_at(p, o), &p[o + 4..o + len]));
        o += len;
    }
    out
}

/// The spots of a sender set: callsign, frequency, snr, mode, locator, source, time.
fn spots_of(body: &[u8]) -> Vec<(String, u64, i8, String, String, u8, u32)> {
    let mut v = Vec::new();
    let mut o = 0;
    let s = |o: &mut usize| {
        let n = body[*o] as usize;
        let t = String::from_utf8(body[*o + 1..*o + 1 + n].to_vec()).unwrap();
        *o += 1 + n;
        t
    };
    // the tail of a set is up to three bytes of padding: a record is at least 11 bytes
    while o + 11 <= body.len() {
        let call = s(&mut o);
        let freq = (u64::from(body[o]) << 32) | u64::from(u32_at(body, o + 1));
        let snr = body[o + 5] as i8;
        o += 6;
        let mode = s(&mut o);
        let loc = s(&mut o);
        let src = body[o];
        let time = u32_at(body, o + 1);
        o += 5;
        v.push((call, freq, snr, mode, loc, src, time));
    }
    v
}

fn spot(call: &str, f: u64) -> Spot {
    Spot {
        callsign: call.into(),
        locator: "PM95".into(),
        snr: -12,
        frequency: f,
        mode: "FT8".into(),
        time_unix: 1_778_068_800,
    }
}

#[test]
fn the_protocol_pages_rules_on_the_datagrams_that_arrive() {
    let listener = UdpSocket::bind("127.0.0.1:0").unwrap();
    listener
        .set_read_timeout(Some(Duration::from_millis(1500)))
        .unwrap();
    let mut cfg = PskConfig::new("k1abc", "FN20");
    cfg.endpoint = Endpoint::Custom(listener.local_addr().unwrap().to_string());
    cfg.interval = Duration::from_millis(250);
    cfg.antenna = "3-el yagi".into();
    let started = Instant::now();
    let rep = PskReporter::start(cfg).unwrap();

    // two batches of spots with a duplicate (same call, same band) inside each, and a spot
    // for the same call on another band
    for s in [
        spot("JA1XYZ", 14_074_100),
        spot("JA1XYZ", 14_074_900),
        spot("W1AW", 14_074_200),
        spot("JA1XYZ", 7_074_100),
    ] {
        rep.spot(s);
    }

    let mut buf = [0u8; 2048];
    let mut datagrams: Vec<(Vec<u8>, std::net::SocketAddr)> = Vec::new();
    // a first, then three more flushes (each carrying the receiver record at least)
    while datagrams.len() < 4 {
        let (n, from) = listener.recv_from(&mut buf).expect("a datagram");
        datagrams.push((buf[..n].to_vec(), from));
        if datagrams.len() == 1 {
            rep.spot(spot("VK3NV", 14_074_300));
            rep.spot(spot("VK3NV", 14_074_400)); // a duplicate
        }
        // With nothing to send, the templates still go out in the first three datagrams; after
        // them an empty interval sends nothing, so a fourth datagram needs a spot.
        if datagrams.len() == 3 {
            rep.spot(spot("DL1ABC", 14_074_500));
        }
    }
    assert!(
        started.elapsed() >= Duration::from_millis(250),
        "no datagram before the interval"
    );

    // one source port for the whole session
    let port = datagrams[0].1.port();
    assert!(datagrams.iter().all(|(_, from)| from.port() == port));

    // the first three carry the templates (set ids 2 and 3), the fourth does not
    for (i, (p, _)) in datagrams.iter().enumerate() {
        let ids: Vec<u16> = sets(p).iter().map(|s| s.0).collect();
        assert!(
            ids.contains(&0x50e2),
            "datagram {i}: the receiver record is in every one"
        );
        assert_eq!(
            ids.contains(&2) && ids.contains(&3),
            i < 3,
            "datagram {i}: templates only in the first three"
        );
        assert_eq!(u16_at(p, 0), 10);
        assert_eq!(u16_at(p, 2) as usize, p.len());
    }
    // the same observation id throughout, and a sequence that counts spots
    let obs = u32_at(&datagrams[0].0, 12);
    assert!(datagrams.iter().all(|(p, _)| u32_at(p, 12) == obs));
    let mut seen = Vec::new();
    let mut expected_seq = 0u32;
    for (p, _) in &datagrams {
        assert_eq!(u32_at(p, 8), expected_seq);
        for (id, body) in sets(p) {
            if id == 0x50e3 {
                let spots = spots_of(body);
                expected_seq += spots.len() as u32;
                seen.extend(spots);
            }
        }
    }
    assert!(seen.iter().all(|s| s.5 == 1), "informationSource is 1");
    let calls: Vec<(&str, u64)> = seen.iter().map(|s| (s.0.as_str(), s.1)).collect();
    assert_eq!(
        calls,
        [
            ("JA1XYZ", 14_074_100),
            ("W1AW", 14_074_200),
            ("JA1XYZ", 7_074_100),
            ("VK3NV", 14_074_300),
            ("DL1ABC", 14_074_500)
        ],
        "each callsign once per band, in the order heard"
    );
    assert_eq!(seen[0].2, -12);
    assert_eq!(seen[0].3, "FT8");
    assert_eq!(seen[0].4, "PM95");
    assert_eq!(seen[0].6, 1_778_068_800);

    // the receiver record: callsign, locator, software, antenna, rig
    let rx = sets(&datagrams[0].0)
        .into_iter()
        .find(|s| s.0 == 0x50e2)
        .unwrap()
        .1
        .to_vec();
    let mut o = 0;
    let mut fields = Vec::new();
    for _ in 0..5 {
        let n = rx[o] as usize;
        fields.push(String::from_utf8(rx[o + 1..o + 1 + n].to_vec()).unwrap());
        o += 1 + n;
    }
    assert_eq!(fields[0], "K1ABC");
    assert_eq!(fields[1], "FN20");
    assert!(fields[2].starts_with("mfsk-skimmer "));
    assert_eq!(fields[3], "3-el yagi");

    let st = rep.stats();
    assert_eq!(st.offered, 7);
    assert_eq!(st.duplicates, 2);
    assert_eq!(st.spots_sent, 5);
    assert!(st.datagrams_sent >= 4);
    assert!(st.last_error.is_none(), "{:?}", st.last_error);
    rep.stop();
}

#[test]
fn a_reporter_that_may_not_send_does_not_start() {
    assert!(PskReporter::start(PskConfig::new("", "FN20")).is_err());
    let mut c = PskConfig::new("K1ABC", "FN20");
    c.endpoint = Endpoint::Production;
    c.interval = Duration::from_secs(1);
    assert!(PskReporter::start(c).is_err());
}
