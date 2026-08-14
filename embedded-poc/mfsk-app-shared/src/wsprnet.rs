//! wsprnet.org spot reporting — request construction, and sinks that
//! range from "build nothing" to an actual upload.
//!
//! Port of WSJT-X's `Network/wsprnet.cpp`, which is the only normative
//! description of this interface there is: wsprnet.org publishes no
//! specification, so every field name, format and unit below is taken
//! from that source rather than inferred.
//!
//! ## What WSJT-X does
//!
//! One HTTP `POST` per spot to `http://wsprnet.org/post/`, body
//! `application/x-www-form-urlencoded`, spots dequeued and sent one at
//! a time (`WSPRNet::work`). The query carries the decode
//! (`urlEncodeSpotFromWSPR` / the `ALL_WSPR.TXT` parse) plus the
//! receiving station (`urlEncodeSpot`):
//!
//! | field | meaning | source |
//! |---|---|---|
//! | `function` | `wspr` for a spot, `wsprstat` for a heartbeat | literal |
//! | `date` / `time` | `yyMMdd` / `HHmm`, slot start, UTC | decode |
//! | `sig` | SNR, dB | decode |
//! | `dt` | time offset, s | decode |
//! | `drift` | drift, Hz | decode |
//! | `tqrg` | **transmitter** frequency, MHz, 6 dp | dial + (audio − 1500)/1e6 |
//! | `tcall` / `tgrid` / `dbm` | transmitted message fields | decode |
//! | `rcall` / `rgrid` / `rqrg` | **receiving** station and dial MHz | config |
//! | `mode` | `2` = WSPR-2, `15` = WSPR-15, else FST4W TR minutes | config |
//! | `version` | reporting software version | config |
//!
//! ## Reporting is opt-in
//!
//! **Reporting is off unless asked for.** [`SpotSink`] defaults to
//! [`SpotSink::Disabled`], which builds nothing. [`SpotSink::Dummy`]
//! formats and logs, which is what exercises the encoder against real
//! decoder output without asserting anything to anyone.
//! [`SpotSink::Http`] does upload — the URL is explicit and has no
//! default, and `wsprnet.org` appears in this file only in
//! documentation. A spot is a public claim that a named station was
//! heard at a time and a frequency; which endpoint hears that claim,
//! and whether one is made at all, belongs at the call site.

use core::fmt::Write as _;

/// The receiving station, and how it describes itself. Constant across
/// a session.
#[derive(Clone, Debug)]
pub struct Reporter {
    /// Receiver callsign (`rcall`).
    pub call: String,
    /// Receiver Maidenhead locator (`rgrid`).
    pub grid: String,
    /// Dial frequency in MHz (`rqrg`) — the same number `tqrg` is
    /// derived from.
    pub dial_mhz: f64,
    /// Reporting software version (`version`).
    pub version: String,
    /// `mode` — see [`Mode`].
    pub mode: Mode,
}

/// wsprnet's `mode` field. WSJT-X's `WSPRNet::encode_mode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// WSPR-2, the 120 s slot. Encodes as `2`.
    Wspr2,
    /// WSPR-15. Encodes as `15`.
    Wspr15,
    /// FST4W, encoded as its TR period in minutes — except that a
    /// 2-minute FST4W reports as `3`, because `2` is taken by WSPR-2.
    /// Straight from `encode_mode`; the oddity is theirs, not ours.
    Fst4w { tr_minutes: u32 },
}

impl Mode {
    pub fn code(self) -> String {
        match self {
            Mode::Wspr2 => "2".into(),
            Mode::Wspr15 => "15".into(),
            Mode::Fst4w { tr_minutes } => {
                let tr = if tr_minutes == 2 { 3 } else { tr_minutes };
                tr.to_string()
            }
        }
    }
}

/// One decoded transmission, in the units wsprnet wants.
#[derive(Clone, Debug)]
pub struct Spot {
    /// Slot start, UTC, as `yyMMdd` — wsprnet takes the two fields
    /// pre-formatted, so callers convert once from whatever clock they
    /// have rather than this module assuming one.
    pub date: String,
    /// Slot start, UTC, as `HHmm`.
    pub time: String,
    /// Transmitter callsign, as decoded. May carry `/P`-style
    /// compound forms, or `<...>` for a hashed callsign — both are
    /// percent-encoded on the way out.
    pub call: String,
    /// Transmitter locator. Empty for a type-2 message, which carries
    /// no grid; WSJT-X sends the empty field rather than omitting it.
    pub grid: String,
    /// Transmitted power, dBm.
    pub dbm: i32,
    /// Reported SNR, dB.
    pub snr_db: i32,
    /// Time offset, seconds.
    pub dt_sec: f32,
    /// Drift, Hz.
    pub drift_hz: i32,
    /// Decoded **audio** frequency in Hz. Converted to `tqrg` against
    /// the reporter's dial — see [`tx_freq_mhz`].
    pub audio_hz: f32,
}

/// `tqrg`: transmitter frequency in MHz.
///
/// `Network/wsprnet.cpp:162` — `rfreq + (tqrg - 1500) / 1e6`, i.e. the
/// dial frequency plus the audio offset from the 1500 Hz centre,
/// converted to MHz. Reported to 6 decimal places (1 Hz), which is the
/// precision the field has.
pub fn tx_freq_mhz(dial_mhz: f64, audio_hz: f32) -> f64 {
    dial_mhz + (audio_hz as f64 - 1500.0) / 1e6
}

/// Percent-encode one query value for
/// `application/x-www-form-urlencoded`.
///
/// Deliberately conservative: everything outside the unreserved set of
/// RFC 3986 is escaped. Callsigns are the reason this cannot be
/// skipped — `/` in a compound call and `<`/`>` around a hashed one
/// both have to survive intact, and a bare `/` in a form body would be
/// read as a path separator by some receivers.
fn encode_value(v: &str, out: &mut String) {
    for b in v.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char)
            }
            b' ' => out.push('+'),
            _ => {
                let _ = write!(out, "%{b:02X}");
            }
        }
    }
}

fn push_field(body: &mut String, key: &str, value: &str) {
    if !body.is_empty() {
        body.push('&');
    }
    body.push_str(key);
    body.push('=');
    encode_value(value, body);
}

/// Build the `application/x-www-form-urlencoded` body for one spot.
///
/// Field order follows WSJT-X: the decode's own fields as
/// `urlEncodeSpotFromWSPR` adds them, then the reporter's as
/// `urlEncodeSpot` appends. Order is not semantically required by a
/// form body, but matching it keeps a captured request diffable
/// against a real WSJT-X one.
pub fn encode_spot(spot: &Spot, reporter: &Reporter) -> String {
    let mut body = String::new();
    push_field(&mut body, "function", "wspr");
    push_field(&mut body, "date", &spot.date);
    push_field(&mut body, "time", &spot.time);
    push_field(&mut body, "sig", &spot.snr_db.to_string());
    push_field(&mut body, "dt", &format!("{:.1}", spot.dt_sec));
    push_field(&mut body, "drift", &spot.drift_hz.to_string());
    push_field(
        &mut body,
        "tqrg",
        &format!("{:.6}", tx_freq_mhz(reporter.dial_mhz, spot.audio_hz)),
    );
    push_field(&mut body, "tcall", &spot.call);
    push_field(&mut body, "tgrid", &spot.grid);
    push_field(&mut body, "dbm", &spot.dbm.to_string());
    push_field(&mut body, "version", &reporter.version);
    push_field(&mut body, "rcall", &reporter.call);
    push_field(&mut body, "rgrid", &reporter.grid);
    push_field(&mut body, "rqrg", &format!("{:.6}", reporter.dial_mhz));
    push_field(&mut body, "mode", &reporter.mode.code());
    body
}

/// The `function=wsprstat` heartbeat WSJT-X sends when a slot produced
/// no decodes (`urlEncodeNoSpot`), so the station still shows as
/// listening.
///
/// `tpct` is the configured transmit duty cycle in percent; `tqrg` and
/// `dbm` describe what this station transmits, not what it heard.
pub fn encode_no_spot(reporter: &Reporter, tx_pct: u32, tx_mhz: f64, tx_dbm: i32) -> String {
    let mut body = String::new();
    push_field(&mut body, "function", "wsprstat");
    push_field(&mut body, "rcall", &reporter.call);
    push_field(&mut body, "rgrid", &reporter.grid);
    push_field(&mut body, "rqrg", &format!("{:.6}", reporter.dial_mhz));
    push_field(&mut body, "tpct", &tx_pct.to_string());
    push_field(&mut body, "tqrg", &format!("{tx_mhz:.6}"));
    push_field(&mut body, "dbm", &tx_dbm.to_string());
    push_field(&mut body, "version", &reporter.version);
    push_field(&mut body, "mode", &reporter.mode.code());
    body
}

/// Where a built request goes — and whether it goes anywhere.
///
/// Reporting is **off unless asked for**. A spot is a public claim that
/// a named station was heard at a time and frequency, so the decision
/// to make one belongs at the call site, not in a default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum SpotSink {
    /// Build nothing, send nothing. The default.
    #[default]
    Disabled,
    /// Build the requests and log them. Exercises the encoder against
    /// real decoder output without asserting anything to anyone.
    Dummy,
    /// Build and POST. The URL is explicit and has no default — see
    /// this module's header for why `wsprnet.org` is not written down
    /// as one.
    Http { url: String },
}

impl SpotSink {
    /// Parse a configuration string: `off`/empty, `dummy`, or a URL.
    ///
    /// Anything that is not a recognised keyword is taken as an
    /// endpoint, so pointing a build at a local receiver is one env
    /// var and needs no code change.
    pub fn from_config(v: Option<&str>) -> Self {
        match v.map(str::trim) {
            None | Some("") | Some("off") | Some("0") | Some("disabled") => SpotSink::Disabled,
            Some("dummy") | Some("log") => SpotSink::Dummy,
            Some(url) => SpotSink::Http { url: url.into() },
        }
    }

    pub fn is_enabled(&self) -> bool {
        !matches!(self, SpotSink::Disabled)
    }
}

/// POST one form body. Returns the HTTP status.
///
/// Only compiled for the ESP-IDF target: the encoder above is pure and
/// is unit-tested on the host (`hosttest/mfsk-app-shared`), which it
/// could not be if this pulled `esp-idf-svc` unconditionally.
#[cfg(target_os = "espidf")]
fn post_form(url: &str, body: &str) -> anyhow::Result<u16> {
    use esp_idf_svc::http::Method;
    use esp_idf_svc::http::client::{Configuration, EspHttpConnection};

    let mut conn = EspHttpConnection::new(&Configuration {
        // wsprnet's own endpoint is plain HTTP; a TLS endpoint would
        // additionally need a root-cert bundle wired here.
        crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
        ..Default::default()
    })?;
    let len = body.len().to_string();
    conn.initiate_request(
        Method::Post,
        url,
        &[
            ("Content-Type", "application/x-www-form-urlencoded"),
            ("Content-Length", &len),
        ],
    )?;
    conn.write(body.as_bytes())?;
    conn.initiate_response()?;
    Ok(conn.status())
}

/// Hand a slot's decodes to the configured sink.
///
/// Returns the bodies it built — empty when reporting is disabled, so a
/// caller can tell "nothing to report" from "not reporting" by asking
/// the sink rather than by counting.
pub fn report_slot(spots: &[Spot], reporter: &Reporter, sink: &SpotSink) -> Vec<String> {
    if !sink.is_enabled() {
        return Vec::new();
    }
    let bodies: Vec<String> = spots.iter().map(|s| encode_spot(s, reporter)).collect();
    match sink {
        SpotSink::Disabled => unreachable!("returned above"),
        SpotSink::Dummy => {
            if bodies.is_empty() {
                log::info!("wsprnet[dummy]: no decodes this slot (would send wsprstat)");
            }
            for (i, b) in bodies.iter().enumerate() {
                log::info!("wsprnet[dummy] {}/{}: POST /post/ {b}", i + 1, bodies.len());
            }
        }
        SpotSink::Http { url } => {
            // One request per spot, sequentially, matching
            // `WSPRNet::work`'s single-dequeue loop. A failure is
            // reported and skipped rather than retried: the next slot
            // is 120 s away and a stale spot is worth less than the
            // time spent resending it.
            for (i, b) in bodies.iter().enumerate() {
                #[cfg(target_os = "espidf")]
                match post_form(url, b) {
                    Ok(status) if (200..300).contains(&status) => {
                        log::info!("wsprnet {}/{}: {status}", i + 1, bodies.len())
                    }
                    Ok(status) => {
                        log::warn!("wsprnet {}/{}: HTTP {status}", i + 1, bodies.len())
                    }
                    Err(e) => log::warn!("wsprnet {}/{}: {e:#}", i + 1, bodies.len()),
                }
                #[cfg(not(target_os = "espidf"))]
                {
                    let _ = (i, b, url);
                    log::warn!("wsprnet: upload is ESP-IDF-only; nothing sent");
                }
            }
        }
    }
    bodies
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reporter() -> Reporter {
        Reporter {
            call: "JL1NIE".into(),
            grid: "PM95".into(),
            dial_mhz: 14.095_600,
            version: "mfsk-core".into(),
            mode: Mode::Wspr2,
        }
    }

    /// `Network/wsprnet.cpp:162`'s formula, at the centre and either
    /// side of it.
    #[test]
    fn tqrg_is_dial_plus_audio_offset_from_1500() {
        assert!((tx_freq_mhz(14.0956, 1500.0) - 14.0956).abs() < 1e-12);
        assert!((tx_freq_mhz(14.0956, 1600.0) - 14.095_700).abs() < 1e-9);
        assert!((tx_freq_mhz(14.0956, 1400.0) - 14.095_500).abs() < 1e-9);
    }

    #[test]
    fn mode_codes_match_wsjtx_encode_mode() {
        assert_eq!(Mode::Wspr2.code(), "2");
        assert_eq!(Mode::Wspr15.code(), "15");
        // The one that is not simply the period: 2-minute FST4W has to
        // dodge WSPR-2's own code.
        assert_eq!(Mode::Fst4w { tr_minutes: 2 }.code(), "3");
        assert_eq!(Mode::Fst4w { tr_minutes: 5 }.code(), "5");
        assert_eq!(Mode::Fst4w { tr_minutes: 15 }.code(), "15");
    }

    #[test]
    fn spot_body_carries_every_wsjtx_field() {
        let spot = Spot {
            date: "150426".into(),
            time: "0918".into(),
            call: "G8VDQ".into(),
            grid: "IO91".into(),
            dbm: 37,
            snr_db: -23,
            dt_sec: 2.23,
            drift_hz: 0,
            audio_hz: 1462.9,
        };
        let body = encode_spot(&spot, &reporter());
        for expect in [
            "function=wspr",
            "date=150426",
            "time=0918",
            "sig=-23",
            "dt=2.2",
            "drift=0",
            "tcall=G8VDQ",
            "tgrid=IO91",
            "dbm=37",
            "version=mfsk-core",
            "rcall=JL1NIE",
            "rgrid=PM95",
            "rqrg=14.095600",
            "mode=2",
        ] {
            assert!(body.contains(expect), "missing {expect} in {body}");
        }
        // 1462.9 Hz is 37.1 Hz below centre.
        assert!(body.contains("tqrg=14.095563"), "{body}");
    }

    /// Compound and hashed callsigns are the reason the encoder exists;
    /// an unescaped `/` or `<` would change how a receiver parses the
    /// body.
    #[test]
    fn callsigns_needing_escapes_survive() {
        let mut spot = Spot {
            date: "150426".into(),
            time: "0918".into(),
            call: "VK3NV/P".into(),
            grid: "".into(),
            dbm: 23,
            snr_db: -28,
            dt_sec: -0.4,
            drift_hz: -1,
            audio_hz: 1500.0,
        };
        let body = encode_spot(&spot, &reporter());
        assert!(body.contains("tcall=VK3NV%2FP"), "{body}");
        // A type-2 message has no grid; the field is sent empty, not
        // omitted, matching WSJT-X.
        assert!(body.contains("tgrid=&"), "{body}");
        assert!(body.contains("dt=-0.4"), "{body}");
        assert!(body.contains("drift=-1"), "{body}");

        spot.call = "<PJ4/K1ABC>".into();
        let body = encode_spot(&spot, &reporter());
        assert!(body.contains("tcall=%3CPJ4%2FK1ABC%3E"), "{body}");
    }

    #[test]
    fn no_spot_heartbeat_matches_wsjtx_fields() {
        let body = encode_no_spot(&reporter(), 20, 14.097_1, 23);
        for expect in [
            "function=wsprstat",
            "rcall=JL1NIE",
            "rgrid=PM95",
            "rqrg=14.095600",
            "tpct=20",
            "tqrg=14.097100",
            "dbm=23",
            "mode=2",
        ] {
            assert!(body.contains(expect), "missing {expect} in {body}");
        }
    }

    #[test]
    fn disabled_is_the_default_and_builds_nothing() {
        assert_eq!(SpotSink::default(), SpotSink::Disabled);
        assert!(!SpotSink::default().is_enabled());
        let spots = vec![Spot {
            date: "150426".into(),
            time: "0918".into(),
            call: "G8VDQ".into(),
            grid: "IO91".into(),
            dbm: 37,
            snr_db: -23,
            dt_sec: 0.0,
            drift_hz: 0,
            audio_hz: 1500.0,
        }];
        assert!(report_slot(&spots, &reporter(), &SpotSink::Disabled).is_empty());
    }

    #[test]
    fn config_string_selects_the_sink() {
        for off in [None, Some(""), Some("off"), Some("0"), Some("disabled")] {
            assert_eq!(SpotSink::from_config(off), SpotSink::Disabled, "{off:?}");
        }
        assert_eq!(SpotSink::from_config(Some("dummy")), SpotSink::Dummy);
        assert_eq!(SpotSink::from_config(Some("log")), SpotSink::Dummy);
        // Anything else is an endpoint, so aiming a build at a local
        // receiver needs no code change.
        assert_eq!(
            SpotSink::from_config(Some("http://127.0.0.1:5000/post/")),
            SpotSink::Http {
                url: "http://127.0.0.1:5000/post/".into()
            }
        );
        assert!(SpotSink::from_config(Some("http://127.0.0.1:5000/post/")).is_enabled());
    }

    /// The dummy sink builds the same bodies it would send, and sends
    /// nothing.
    #[test]
    fn dummy_sink_builds_one_body_per_spot() {
        let spots: Vec<Spot> = ["G8VDQ", "W3BI"]
            .iter()
            .map(|c| Spot {
                date: "150426".into(),
                time: "0918".into(),
                call: (*c).into(),
                grid: "IO91".into(),
                dbm: 37,
                snr_db: -23,
                dt_sec: 0.0,
                drift_hz: 0,
                audio_hz: 1500.0,
            })
            .collect();
        let bodies = report_slot(&spots, &reporter(), &SpotSink::Dummy);
        assert_eq!(bodies.len(), 2);
        assert!(bodies[0].contains("tcall=G8VDQ"));
        assert!(bodies[1].contains("tcall=W3BI"));
    }
}
