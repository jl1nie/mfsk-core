// SPDX-License-Identifier: GPL-3.0-only
//! PSK Reporter's IPFIX packets: the receiver's record, the spots, and the templates that
//! describe them.
//!
//! A port of WSJT-X v3.3.0-beta1's `Network/PSKReporterIPFIX.cpp`, byte for byte (the golden
//! `embedded-poc/assets/golden/pskreporter/` is what that file makes of a script of cases,
//! through `scripts/pskreporter/ipfix_oracle.cpp`). The protocol is described at
//! <https://pskreporter.info/pskdev.html>: IPFIX (RFC 5101), simplified, with PSK Reporter's own
//! elements under enterprise number 30351. All integers are big-endian, a string is a length
//! byte and its UTF-8, and every set and message is padded with zeros to a multiple of four.
//!
//! This is the builder only. Sending (one UDP socket, the five-minute batches, the per-callsign
//! dedupe, template resends) and the rules about which decodes become spots are a layer above.

/// The largest IPFIX message that fits one UDP datagram on a 1000-byte IPv6 path:
/// `1000 - 40 (IPv6) - 8 (UDP)`.
pub const MAX_UDP_IPFIX_PAYLOAD_BYTES: usize = 1000 - 40 - 8;
/// The largest message a TCP connection carries (the length field is 16 bits).
pub const MAX_TCP_IPFIX_PAYLOAD_BYTES: usize = 0xffff;

const CALLSIGN_LIMIT: usize = 32;
const LOCATOR_LIMIT: usize = 16;
const MODE_LIMIT: usize = 16;
const PROGRAM_INFO_LIMIT: usize = 80;
const ANTENNA_LIMIT: usize = 128;
const RIG_INFORMATION_LIMIT: usize = 128;

/// PSK Reporter's private enterprise number.
const PEN: u32 = 30351;
/// Template for a sender / spot record.
const SENDER_TEMPLATE_ID: u16 = 0x50e3;
/// Options template for the receiver record, with `receiverCallsign` as its scope.
const RECEIVER_TEMPLATE_ID: u16 = 0x50e2;

/// The receiving station, in every datagram.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Receiver {
    pub callsign: String,
    pub locator: String,
    /// The decoding software, name and version (`decodingSoftware`).
    pub program_info: String,
    pub antenna: String,
    pub rig_information: String,
}

/// One station heard.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Spot {
    /// The sending station, as it was heard.
    pub callsign: String,
    pub locator: String,
    /// dB, written as a signed byte (it wraps beyond -128..127, as upstream's cast does).
    pub snr: i32,
    /// Hz, five bytes.
    pub frequency: u64,
    /// ADIF mode, e.g. `FT8`.
    pub mode: String,
    /// UTC seconds since the epoch (written as 32 bits).
    pub time_unix: i64,
}

/// One datagram (or TCP message) and how many spots it carries.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    pub payload: Vec<u8>,
    pub spot_count: usize,
}

fn pad_bytes(len: usize) -> usize {
    (4 - len % 4) % 4
}

/// `boundedUtf8`: at most `max_bytes` bytes, never cut inside a character.
fn bounded(value: &str, max_bytes: usize) -> &[u8] {
    let b = value.as_bytes();
    if b.len() <= max_bytes {
        return b;
    }
    let mut n = max_bytes;
    while n > 0 && (b[n] & 0xc0) == 0x80 {
        n -= 1;
    }
    &b[..n]
}

fn put_string(out: &mut Vec<u8>, value: &str, max_bytes: usize) {
    let b = bounded(value, max_bytes);
    out.push(b.len() as u8);
    out.extend_from_slice(b);
}

fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_be_bytes());
}

fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_be_bytes());
}

/// `setLength`: pad to a multiple of four, then write the padded length at `offset`.
fn set_length(out: &mut Vec<u8>, offset: usize) {
    out.resize(out.len() + pad_bytes(out.len()), 0);
    let n = out.len() as u16;
    out[offset..offset + 2].copy_from_slice(&n.to_be_bytes());
}

/// The two templates: the sender record (callsign, frequency, SNR, mode, locator, information
/// source, time) and the receiver options template (one scope field, `receiverCallsign`).
fn descriptor_sets() -> Vec<u8> {
    let mut sets = Vec::new();
    {
        let mut d = Vec::new();
        put_u16(&mut d, 2); // template set
        put_u16(&mut d, 0); // length, below
        put_u16(&mut d, SENDER_TEMPLATE_ID);
        put_u16(&mut d, 7); // field count
        for (id, len) in [
            (1u16, 0xffffu16), // senderCallsign
            (5, 5),            // frequency, a 5-byte unsigned integer
            (6, 1),            // sNR
            (10, 0xffff),      // mode
            (3, 0xffff),       // senderLocator
            (11, 1),           // informationSource
        ] {
            put_u16(&mut d, 0x8000 + id);
            put_u16(&mut d, len);
            put_u32(&mut d, PEN);
        }
        put_u16(&mut d, 150); // dateTimeSeconds (an IANA element: no enterprise number)
        put_u16(&mut d, 4);
        set_length(&mut d, 2);
        sets.extend_from_slice(&d);
    }
    {
        let mut d = Vec::new();
        put_u16(&mut d, 3); // options template set
        put_u16(&mut d, 0);
        put_u16(&mut d, RECEIVER_TEMPLATE_ID);
        put_u16(&mut d, 5); // field count
        put_u16(&mut d, 1); // scope field count: must stay 1
        for id in [
            2u16, // receiverCallsign (the scope field)
            4,    // receiverLocator
            8,    // decodingSoftware
            9,    // antennaInformation
            13,   // rigInformation
        ] {
            put_u16(&mut d, 0x8000 + id);
            put_u16(&mut d, 0xffff);
            put_u32(&mut d, PEN);
        }
        set_length(&mut d, 2);
        sets.extend_from_slice(&d);
    }
    sets
}

fn receiver_set(r: &Receiver) -> Vec<u8> {
    let mut d = Vec::new();
    put_u16(&mut d, RECEIVER_TEMPLATE_ID);
    put_u16(&mut d, 0);
    put_string(&mut d, &r.callsign, CALLSIGN_LIMIT);
    put_string(&mut d, &r.locator, LOCATOR_LIMIT);
    put_string(&mut d, &r.program_info, PROGRAM_INFO_LIMIT);
    put_string(&mut d, &r.antenna, ANTENNA_LIMIT);
    put_string(&mut d, &r.rig_information, RIG_INFORMATION_LIMIT);
    set_length(&mut d, 2);
    d
}

/// A sender set holding `record_bytes` of records: 4 bytes of header, padded.
fn sender_set_length(record_bytes: usize) -> usize {
    let len = 4 + record_bytes;
    len + pad_bytes(len)
}

/// A message holding `set_bytes` of sets: 16 bytes of header, padded.
fn message_length(set_bytes: usize) -> usize {
    let len = 16 + set_bytes;
    len + pad_bytes(len)
}

/// One spot, in the order of the sender template.
fn spot_record(s: &Spot) -> Vec<u8> {
    let mut d = Vec::new();
    put_string(&mut d, &s.callsign, CALLSIGN_LIMIT);
    d.push(((s.frequency >> 32) & 0xff) as u8);
    put_u32(&mut d, (s.frequency & 0xffff_ffff) as u32);
    d.push(s.snr as i8 as u8);
    put_string(&mut d, &s.mode, MODE_LIMIT);
    put_string(&mut d, &s.locator, LOCATOR_LIMIT);
    d.push(1); // informationSource: automatically extracted
    put_u32(&mut d, s.time_unix as u32);
    d
}

fn sender_set(records: &[Vec<u8>]) -> Vec<u8> {
    let mut d = Vec::new();
    put_u16(&mut d, SENDER_TEMPLATE_ID);
    put_u16(&mut d, 0);
    for r in records {
        d.extend_from_slice(r);
    }
    set_length(&mut d, 2);
    d
}

fn message(sets: &[u8], sequence_number: u32, observation_id: u32, export_time: u32) -> Vec<u8> {
    let mut d = Vec::new();
    put_u16(&mut d, 10); // IPFIX version
    put_u16(&mut d, 0);
    put_u32(&mut d, export_time);
    put_u32(&mut d, sequence_number);
    put_u32(&mut d, observation_id);
    d.extend_from_slice(sets);
    set_length(&mut d, 2);
    d
}

/// The datagrams that carry `spots` from `receiver`, none longer than `max_payload_bytes`
/// (`buildPackets`).
///
/// - Every datagram has the receiver record; the templates are in the first only, and only when
///   `include_descriptors` (PSK Reporter wants them in the first three datagrams after a start
///   and about hourly).
/// - `sequence_number` counts spots, not datagrams: it advances by each datagram's spot count.
///   `observation_id` is random and constant for a session; `export_time` is Unix seconds.
/// - No spots gives one datagram with the receiver record alone (which also tells PSK Reporter
///   where the station is).
pub fn build_packets(
    receiver: &Receiver,
    spots: &[Spot],
    include_descriptors: bool,
    mut sequence_number: u32,
    observation_id: u32,
    export_time: u32,
    max_payload_bytes: usize,
) -> Vec<Packet> {
    let mut first_base_sets = Vec::new();
    if include_descriptors {
        first_base_sets.extend_from_slice(&descriptor_sets());
    }
    let receiver_set = receiver_set(receiver);
    first_base_sets.extend_from_slice(&receiver_set);

    let base_payload = message(
        &first_base_sets,
        sequence_number,
        observation_id,
        export_time,
    );
    debug_assert!(base_payload.len() <= max_payload_bytes);
    if spots.is_empty() {
        return vec![Packet {
            payload: base_payload,
            spot_count: 0,
        }];
    }

    let mut packets = Vec::new();
    let mut records: Vec<Vec<u8>> = Vec::new();
    let mut record_bytes = 0usize;
    let mut base_sets = first_base_sets;
    for spot in spots {
        let record = spot_record(spot);
        let candidate_payload_size =
            message_length(base_sets.len() + sender_set_length(record_bytes + record.len()));
        if !records.is_empty() && candidate_payload_size > max_payload_bytes {
            let mut sets = base_sets.clone();
            sets.extend_from_slice(&sender_set(&records));
            packets.push(Packet {
                payload: message(&sets, sequence_number, observation_id, export_time),
                spot_count: records.len(),
            });
            sequence_number = sequence_number.wrapping_add(records.len() as u32);
            records.clear();
            record_bytes = 0;
            // Follow-on datagrams keep the receiver record and drop the templates.
            base_sets = receiver_set.clone();
        }
        record_bytes += record.len();
        records.push(record);
    }
    if !records.is_empty() {
        let mut sets = base_sets;
        sets.extend_from_slice(&sender_set(&records));
        packets.push(Packet {
            payload: message(&sets, sequence_number, observation_id, export_time),
            spot_count: records.len(),
        });
    }
    packets
}

// ──────────────────────────────────────────────────────────────────────────
// What becomes a spot, and the sender
// ──────────────────────────────────────────────────────────────────────────

use std::collections::{HashMap, VecDeque};
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::Decode;
use crate::jtty::JttyMessage;

/// The shortest interval between two timed sends the protocol page allows
/// (<https://pskreporter.info/pskdev.html>: "no more than one every five minutes (unless the
/// packet becomes full)"). WSJT-X sends about every two minutes; this does not.
pub const MIN_SEND_INTERVAL: Duration = Duration::from_secs(300);
/// A callsign on a band is reported at most once per this. The page: "no more than once per
/// five minute period", "ideally ... once per hour if it has not changed". Five minutes is what
/// WSJT-X does; a longer value is kinder to the database.
pub const DEFAULT_REPEAT: Duration = Duration::from_secs(300);
/// Pending spots beyond this drop the oldest (WSJT-X's `MAX_PENDING_SPOTS`).
pub const MAX_PENDING_SPOTS: usize = 2048;
/// Templates go in the first three datagrams after a start, then about hourly.
const DESCRIPTOR_SENDS: u32 = 3;
const DESCRIPTOR_PERIOD: Duration = Duration::from_secs(3600);

/// Where the datagrams go.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Endpoint {
    /// `pskreporter.info:14739`, the page's test listener: it analyses what it receives
    /// (`/cgi-bin/psk-analysis.pl`) and records nothing. The default.
    Test,
    /// `report.pskreporter.info:4739`. The `report.` host name is required. **Not for use until
    /// PSK Reporter's author has been asked about skimmers** (#655).
    Production,
    /// `host:port`, for a test listener of one's own.
    Custom(String),
}

impl Endpoint {
    pub fn address(&self) -> String {
        match self {
            Endpoint::Test => "pskreporter.info:14739".into(),
            Endpoint::Production => "report.pskreporter.info:4739".into(),
            Endpoint::Custom(a) => a.clone(),
        }
    }
}

/// The receiving station and how often to send.
#[derive(Clone, Debug)]
pub struct PskConfig {
    /// Under which callsign to report (not necessarily an amateur call: the page's example is
    /// `W/SWL/BOSTON`).
    pub callsign: String,
    /// **The receiver's** locator: where the antenna is, which for a remote SpyServer is not
    /// where the operator sits.
    pub locator: String,
    pub antenna: String,
    pub rig_information: String,
    /// `decodingSoftware`, name and version.
    pub software: String,
    pub endpoint: Endpoint,
    /// Between timed sends, before the random addition. Never below [`MIN_SEND_INTERVAL`] for
    /// [`Endpoint::Production`]; the test endpoint and a custom one take any value.
    pub interval: Duration,
    /// A callsign on a band is not reported again within this.
    pub repeat: Duration,
}

impl PskConfig {
    pub fn new(callsign: &str, locator: &str) -> Self {
        PskConfig {
            callsign: callsign.trim().to_ascii_uppercase(),
            locator: locator.trim().to_string(),
            antenna: String::new(),
            rig_information: String::new(),
            software: format!("mfsk-skimmer {}", env!("CARGO_PKG_VERSION")),
            endpoint: Endpoint::Test,
            interval: MIN_SEND_INTERVAL,
            repeat: DEFAULT_REPEAT,
        }
    }

    /// Why this configuration may not send, if so.
    pub fn problem(&self) -> Option<&'static str> {
        if self.callsign.is_empty() {
            return Some("a callsign to report under is needed");
        }
        if self.locator.len() < 4 {
            return Some("the receiver's locator (at least four characters) is needed");
        }
        if self.endpoint == Endpoint::Production && self.interval < MIN_SEND_INTERVAL {
            return Some("the production endpoint wants at most one datagram every five minutes");
        }
        None
    }
}

/// PSK Reporter's name for a mode: the registry name without its period and sub-mode.
/// `None` for modes that are not reported here (WSPR goes to WSPRnet).
fn adif_mode(name: &str) -> Option<&'static str> {
    Some(match name.split('-').next().unwrap_or(name) {
        "FT8" => "FT8",
        "FT4" => "FT4",
        "FST4" => "FST4",
        "JT9" => "JT9",
        "JT65" => "JT65",
        "Q65" => "Q65",
        "JTTY" => "JTTY",
        _ => return None,
    })
}

/// The part of a callsign that identifies the station: `JA1ABC/P` and `PJ4/K1ABC` give the
/// longer piece (`JA1ABC`, `K1ABC`).
fn base_call(c: &str) -> &str {
    c.split('/').max_by_key(|p| p.len()).unwrap_or(c)
}

/// WSJT-X's rule (`MainWindow::pskPost`): a decoded text that holds our own base call and our
/// locator's first four characters is our own transmission heard again (two instances, say).
fn is_ours(text: &str, my_call: &str, my_grid: &str) -> bool {
    let call = base_call(my_call);
    !call.is_empty() && my_grid.len() >= 4 && text.contains(call) && text.contains(&my_grid[..4])
}

/// `DecodedText`'s `tokens_re` (`Decoder/decodedtext.cpp`), without its `(?!RR73)`: the regex
/// crate has no look-ahead, so a `word4` of `RR73` is dropped after the match, which is what the
/// look-ahead's failure to match amounts to.
fn tokens_re() -> &'static regex::Regex {
    static RE: std::sync::OnceLock<regex::Regex> = std::sync::OnceLock::new();
    RE.get_or_init(|| {
        regex::Regex::new(
            r"^(?:(?P<dual>[A-Z0-9/]+)\sRR73;\s)?(?:(?P<word1>(?:CQ|DE|QRZ)(?:\s?DX|\s(?:[A-Z]{1,4}|\d{3}))|[A-Z0-9/]+|\.{3})\s)(?:(?P<word2>[A-Z0-9/]+)(?:\s(?P<word3>[-+A-Z0-9]+)(?:\s(?P<word4>OOO|[A-R]{2}[0-9]{2}|5[0-9]{5})(?:\s(?P<word5>[A-R]{2}[0-9]{2}[A-X]{2}))?)?)?)?",
        )
        .expect("tokens_re")
    })
}

/// `DecodedText::deCallAndGrid`: the second word is the sender, the third a locator, or the
/// fourth after an `R`. (The text first loses its `<` `>`, and anything up to a `"; "`.)
pub fn de_call_and_grid(text: &str) -> (String, String) {
    let msg: String = text.chars().filter(|&c| c != '<' && c != '>').collect();
    let msg = msg.trim();
    let msg = msg.split_once("; ").map_or(msg, |(_, rest)| rest);
    let Some(m) = tokens_re().captures(msg) else {
        return (String::new(), String::new());
    };
    let get = |n: &str| m.name(n).map_or("", |x| x.as_str());
    let call = get("word2").to_string();
    let mut grid = get("word3");
    if grid == "R" {
        grid = get("word4");
        if grid == "RR73" {
            grid = "";
        }
    }
    (call, grid.to_string())
}

/// `Radio::is_standard_callsign` (`Radio.cpp`): one or two letters, or a letter and a digit, or a
/// digit and a letter; then a digit and up to three letters; then an optional `/R` or `/P`.
pub fn is_standard_callsign(call: &str) -> bool {
    static RE: std::sync::OnceLock<regex::Regex> = std::sync::OnceLock::new();
    call.is_ascii()
        && RE
            .get_or_init(|| {
                regex::Regex::new(
                    r"(?i)^\s*([A-Z]{0,2}|[A-Z][0-9]|[0-9][A-Z])([0-9][A-Z]{0,3})(/R|/P)?\s*$",
                )
                .expect("is_standard_callsign")
            })
            .is_match(call)
}

/// `Radio::decoded_grid_pattern()`: four or six characters, not `RR73`.
pub fn is_decoded_grid(g: &str) -> bool {
    let b = g.as_bytes();
    let letters = |x: &[u8], hi: u8| {
        x.iter()
            .all(|c| (b'A'..=hi).contains(&c.to_ascii_uppercase()))
    };
    let digits = |x: &[u8]| x.iter().all(u8::is_ascii_digit);
    (b.len() == 4 || b.len() == 6)
        && letters(&b[..2], b'R')
        && digits(&b[2..4])
        && (b.len() == 4 || letters(&b[4..], b'X'))
        && !g.eq_ignore_ascii_case("RR73")
}

/// `stdMsg` (`stdmsg`, `lib/stdmsg.f90`): the message packs as a structured type, `i3 > 0` or
/// `n3 > 0`; free text (`0.0`) does not. Read off the decoded bits, 77 of them in the key. The
/// 72-bit JT modes have no such bits to read here; their text has to name a sender.
fn is_standard(d: &Decode) -> bool {
    let k = &d.detail.key;
    if k.len() >= 77 {
        let bit = |i: usize| u32::from(k[i] & 1);
        let n3 = bit(71) << 2 | bit(72) << 1 | bit(73);
        let i3 = bit(74) << 2 | bit(75) << 1 | bit(76);
        return i3 > 0 || n3 > 0;
    }
    true
}

/// The spot a decoded row makes, by WSJT-X's rules (`MainWindow::pskPost` and the checks before
/// it): a standard (non-free-text) message, a real decode with a clock (a spot needs a time),
/// not low-confidence (a text ending in `?`), not our own call or our own transmission heard
/// again, the sender and locator by `deCallAndGrid`, and a locator or a CQ.
pub fn spot_from_decode(d: &Decode, my_call: &str, my_grid: &str) -> Option<Spot> {
    let mode = adif_mode(crate::modes::mode_name(d.mode))?;
    if !is_standard(d) || d.text.trim_end().ends_with('?') || is_ours(&d.text, my_call, my_grid) {
        return None;
    }
    let (call, grid) = de_call_and_grid(&d.text);
    if call.is_empty() || base_call(&call).eq_ignore_ascii_case(base_call(my_call)) {
        return None;
    }
    let cq = d.text.starts_with("CQ ") || d.text.contains(" CQ ");
    if !is_decoded_grid(&grid) && !cq {
        return None;
    }
    Some(Spot {
        callsign: call,
        locator: if is_decoded_grid(&grid) {
            grid
        } else {
            String::new()
        },
        snr: d.snr_db.round() as i32,
        frequency: d.freq_hz.round().max(0.0) as u64,
        mode: mode.to_string(),
        time_unix: d.slot_utc_ns?.div_euclid(1_000_000_000),
    })
}

/// The spot a JTTY message makes: WSJT-X's rule, [`JttyMessage::sender`], on a message that is
/// over and complete; time is the start of its first frame.
pub fn spot_from_jtty(m: &JttyMessage, my_call: &str) -> Option<Spot> {
    let (call, grid) = m.sender()?;
    if base_call(&call).eq_ignore_ascii_case(base_call(my_call)) {
        return None;
    }
    Some(Spot {
        callsign: call,
        locator: grid.unwrap_or_default(),
        snr: m.snr_db.round() as i32,
        frequency: m.freq_hz.round().max(0.0) as u64,
        mode: "JTTY".to_string(),
        time_unix: m.start_utc_ns?.div_euclid(1_000_000_000),
    })
}

/// A band, for "once per band": the amateur bands below 148 MHz by their edges, anything else by
/// its MHz.
fn band_of(frequency_hz: u64) -> u32 {
    const EDGES: &[(u64, u64)] = &[
        (1_800_000, 2_000_000),
        (3_500_000, 4_000_000),
        (5_250_000, 5_450_000),
        (7_000_000, 7_300_000),
        (10_100_000, 10_150_000),
        (14_000_000, 14_350_000),
        (18_068_000, 18_168_000),
        (21_000_000, 21_450_000),
        (24_890_000, 24_990_000),
        (28_000_000, 29_700_000),
        (50_000_000, 54_000_000),
        (70_000_000, 70_500_000),
        (144_000_000, 148_000_000),
    ];
    EDGES
        .iter()
        .position(|&(lo, hi)| (lo..hi).contains(&frequency_hz))
        .map(|i| i as u32)
        .unwrap_or(1000 + (frequency_hz / 1_000_000) as u32)
}

/// Each callsign on a band once per `repeat`.
struct Dedupe {
    seen: HashMap<(String, u32), Instant>,
    repeat: Duration,
}

impl Dedupe {
    fn new(repeat: Duration) -> Self {
        Dedupe {
            seen: HashMap::new(),
            repeat,
        }
    }

    fn admit(&mut self, spot: &Spot, now: Instant) -> bool {
        let key = (spot.callsign.to_ascii_uppercase(), band_of(spot.frequency));
        match self.seen.get(&key) {
            Some(&t) if now.saturating_duration_since(t) < self.repeat => false,
            _ => {
                self.seen.insert(key, now);
                true
            }
        }
    }

    fn prune(&mut self, now: Instant) {
        let keep = self.repeat * 2;
        self.seen
            .retain(|_, &mut t| now.saturating_duration_since(t) <= keep);
    }
}

/// xorshift64: a session's random identifier and the timers' random addition. Not for anything
/// that needs to be unpredictable.
struct Rng(u64);

impl Rng {
    fn new() -> Self {
        let t = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(1, |d| d.as_nanos() as u64);
        let mut r = Rng(t ^ (u64::from(std::process::id()) << 32) ^ 0x9e37_79b9_7f4a_7c15);
        for _ in 0..4 {
            r.next();
        }
        r
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0.max(1);
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    /// Up to `d`, not including it.
    fn up_to(&mut self, d: Duration) -> Duration {
        let ms = d.as_millis().max(1) as u64;
        Duration::from_millis(self.next() % ms)
    }
}

/// What the sender has done, for a status line.
#[derive(Clone, Debug, Default)]
pub struct PskStats {
    /// Spots handed to the reporter.
    pub offered: u64,
    /// Not queued: the same callsign on the same band was already reported inside the repeat.
    pub duplicates: u64,
    /// Dropped from a full queue.
    pub overflowed: u64,
    /// Dropped for having begun before the radio was last retuned: the slot may hold the old
    /// band's samples ([`PskReporter::retuned`]).
    pub stale: u64,
    pub spots_sent: u64,
    pub datagrams_sent: u64,
    /// Unix seconds of the last datagram sent.
    pub last_send_unix: Option<i64>,
    pub last_error: Option<String>,
    pub pending: usize,
}

enum Msg {
    Spot(Spot),
    /// Nothing that began before this UTC second is a spot.
    NotBefore(i64),
    Stop,
}

/// A running reporter: spots go in with [`PskReporter::spot`]; a thread of its own batches, dedupes
/// and sends them on one UDP socket.
pub struct PskReporter {
    tx: Sender<Msg>,
    stats: Arc<Mutex<PskStats>>,
    handle: Option<JoinHandle<()>>,
}

impl PskReporter {
    /// Start sending to `cfg.endpoint`. The first timed send is `cfg.interval` plus a random
    /// addition after the start: the timers are not tied to the clock, as the page asks.
    pub fn start(cfg: PskConfig) -> Result<PskReporter, String> {
        if let Some(p) = cfg.problem() {
            return Err(p.to_string());
        }
        let stats = Arc::new(Mutex::new(PskStats::default()));
        let (tx, rx) = mpsc::channel();
        let st = Arc::clone(&stats);
        let handle = std::thread::Builder::new()
            .name("psk-reporter".into())
            .spawn(move || run(cfg, rx, st))
            .map_err(|e| e.to_string())?;
        Ok(PskReporter {
            tx,
            stats,
            handle: Some(handle),
        })
    }

    /// Offer a spot; it is sent in the next batch unless the callsign was reported on this band
    /// inside the repeat.
    pub fn spot(&self, spot: Spot) {
        let _ = self.tx.send(Msg::Spot(spot));
    }

    /// The radio was (re)tuned at `utc_unix`: a slot or message that began before then may hold
    /// samples of the old band, and is not reported as a spot of the new one. WSJT-X waits
    /// four fifths of a period after a band change for the same reason (`okToPost`); this is
    /// exact, by the slot's own start.
    pub fn retuned(&self, utc_unix: i64) {
        let _ = self.tx.send(Msg::NotBefore(utc_unix));
    }

    pub fn stats(&self) -> PskStats {
        self.stats.lock().map(|s| s.clone()).unwrap_or_default()
    }

    /// Stop the thread. Spots still pending are not sent: the protocol's five minutes are the
    /// limit even at the end.
    pub fn stop(mut self) {
        let _ = self.tx.send(Msg::Stop);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for PskReporter {
    fn drop(&mut self) {
        let _ = self.tx.send(Msg::Stop);
    }
}

fn resolve(addr: &str) -> Result<SocketAddr, String> {
    let all: Vec<SocketAddr> = addr
        .to_socket_addrs()
        .map_err(|e| format!("{addr}: {e}"))?
        .collect();
    all.iter()
        .find(|a| a.is_ipv4())
        .or(all.first())
        .copied()
        .ok_or_else(|| format!("{addr}: no address"))
}

/// One socket, one source port for the whole session (the page asks for it), connected to the
/// collector.
fn open(addr: &str) -> Result<UdpSocket, String> {
    let to = resolve(addr)?;
    let bind = if to.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" };
    let s = UdpSocket::bind(bind).map_err(|e| e.to_string())?;
    s.connect(to).map_err(|e| e.to_string())?;
    Ok(s)
}

fn now_unix() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64)
}

fn run(cfg: PskConfig, rx: mpsc::Receiver<Msg>, stats: Arc<Mutex<PskStats>>) {
    let mut rng = Rng::new();
    let receiver = Receiver {
        callsign: cfg.callsign.clone(),
        locator: cfg.locator.clone(),
        program_info: cfg.software.clone(),
        antenna: cfg.antenna.clone(),
        rig_information: cfg.rig_information.clone(),
    };
    let observation_id = rng.next() as u32;
    let address = cfg.endpoint.address();
    let mut socket: Option<UdpSocket> = None;
    let mut dedupe = Dedupe::new(cfg.repeat);
    let mut queue: VecDeque<Spot> = VecDeque::new();
    let mut sequence: u32 = 0;
    let mut not_before = i64::MIN;
    let mut descriptors_left = DESCRIPTOR_SENDS;
    let started = Instant::now();
    let jitter = |rng: &mut Rng| rng.up_to(cfg.interval / 5 + Duration::from_millis(1));
    let mut next_flush = started + cfg.interval + jitter(&mut rng);
    let mut next_descriptors = started + DESCRIPTOR_PERIOD;

    loop {
        let wait = next_flush.saturating_duration_since(Instant::now());
        match rx.recv_timeout(wait) {
            Ok(Msg::NotBefore(t)) => not_before = not_before.max(t),
            Ok(Msg::Spot(s)) => {
                let now = Instant::now();
                let mut st = stats.lock().unwrap();
                st.offered += 1;
                if s.time_unix < not_before {
                    st.stale += 1;
                    continue;
                }
                if !dedupe.admit(&s, now) {
                    st.duplicates += 1;
                    continue;
                }
                queue.push_back(s);
                while queue.len() > MAX_PENDING_SPOTS {
                    queue.pop_front();
                    st.overflowed += 1;
                }
                st.pending = queue.len();
            }
            Ok(Msg::Stop) | Err(RecvTimeoutError::Disconnected) => break,
            Err(RecvTimeoutError::Timeout) => {
                let now = Instant::now();
                if now >= next_descriptors {
                    descriptors_left = DESCRIPTOR_SENDS;
                    next_descriptors = now + DESCRIPTOR_PERIOD;
                }
                dedupe.prune(now);
                next_flush = now + cfg.interval + jitter(&mut rng);
                if queue.is_empty() && descriptors_left == 0 {
                    continue;
                }
                if socket.is_none() {
                    match open(&address) {
                        Ok(s) => socket = Some(s),
                        Err(e) => {
                            stats.lock().unwrap().last_error = Some(e);
                            continue;
                        }
                    }
                }
                let spots: Vec<Spot> = queue.drain(..).collect();
                let packets = build_packets(
                    &receiver,
                    &spots,
                    descriptors_left > 0,
                    sequence,
                    observation_id,
                    now_unix() as u32,
                    MAX_UDP_IPFIX_PAYLOAD_BYTES,
                );
                descriptors_left = descriptors_left.saturating_sub(1);
                let sock = socket.as_ref().unwrap();
                let mut st = stats.lock().unwrap();
                for p in &packets {
                    match sock.send(&p.payload) {
                        Ok(_) => {
                            sequence = sequence.wrapping_add(p.spot_count as u32);
                            st.spots_sent += p.spot_count as u64;
                            st.datagrams_sent += 1;
                            st.last_send_unix = Some(now_unix());
                            st.last_error = None;
                        }
                        Err(e) => {
                            // Start from a fresh lookup next time.
                            st.last_error = Some(e.to_string());
                            socket = None;
                            break;
                        }
                    }
                }
                st.pending = 0;
            }
        }
    }
}

#[cfg(test)]
mod spot_rule_tests {
    use super::*;
    use crate::jtty::UpdateKind;
    use crate::{Decode, DecodeDetail};
    use mfsk_core::Mode;

    fn decode(mode: Mode, text: &str) -> Decode {
        Decode {
            channel: 0,
            mode,
            slot_utc_ns: Some(1_778_068_800_000_000_000),
            dial_hz: 14_074_000.0,
            freq_hz: 14_075_465.4,
            snr_db: -6.4,
            dt_s: 0.2,
            text: text.to_string(),
            detail: DecodeDetail::default(),
            update: false,
        }
    }

    #[test]
    fn a_standard_message_makes_a_spot_of_its_sender() {
        let s =
            spot_from_decode(&decode(Mode::Ft8, "RA0ANO UA0LQE PN53"), "K1ABC", "FN20").unwrap();
        assert_eq!(s.callsign, "UA0LQE");
        assert_eq!(s.locator, "PN53");
        assert_eq!(s.mode, "FT8");
        assert_eq!(s.frequency, 14_075_465);
        assert_eq!(s.snr, -6);
        assert_eq!(s.time_unix, 1_778_068_800);
    }

    #[test]
    fn a_cq_without_a_grid_is_a_spot_without_a_locator() {
        let s = spot_from_decode(&decode(Mode::Ft8, "CQ DX BG1SUD"), "K1ABC", "FN20").unwrap();
        assert_eq!((s.callsign.as_str(), s.locator.as_str()), ("BG1SUD", ""));
    }

    /// `stdMsg` is `i3 > 0 || n3 > 0` of the 77 bits: free text (0.0) is not a message about a
    /// station, a structured type is.
    #[test]
    fn the_message_bits_tell_a_standard_message_from_free_text() {
        use mfsk_core::msg::wsjt77::{pack77, pack77_free_text};
        let with_key = |bits: [u8; 77], text: &str| {
            let mut d = decode(Mode::Ft8, text);
            d.detail.key = bits.to_vec();
            d
        };
        let std = with_key(
            pack77("RA0ANO", "UA0LQE", "PN53").unwrap(),
            "RA0ANO UA0LQE PN53",
        );
        assert!(spot_from_decode(&std, "K1ABC", "FN20").is_some());
        // free text that reads like a message is still free text
        let free = with_key(
            pack77_free_text("RA0ANO UA0LQE").unwrap(),
            "RA0ANO UA0LQE PN53",
        );
        assert!(spot_from_decode(&free, "K1ABC", "FN20").is_none());
    }

    #[test]
    fn an_r_before_the_locator_is_still_the_locator() {
        let s =
            spot_from_decode(&decode(Mode::Ft8, "K1ABC W9XYZ R EN37"), "N0CALL", "AA00").unwrap();
        assert_eq!((s.callsign.as_str(), s.locator.as_str()), ("W9XYZ", "EN37"));
        // `RR73` has a grid's shape and is not one
        assert!(
            spot_from_decode(&decode(Mode::Ft8, "K1ABC W9XYZ RR73"), "N0CALL", "AA00").is_none()
        );
    }

    #[test]
    fn what_is_not_a_spot() {
        let skip = |mode, text: &str| spot_from_decode(&decode(mode, text), "K1ABC", "FN20");
        // a report, not a grid, and not a CQ: WSJT-X wants one or the other
        assert!(skip(Mode::Ft8, "BG5VPO BG9PQZ -12").is_none());
        // an unresolved hashed call names no sender
        assert!(skip(Mode::Ft8, "CQ <...> NO33").is_none());
        // low confidence
        assert!(skip(Mode::Ft8, "BG5VPO BG9PQZ OM89?").is_none());
        // free text
        assert!(skip(Mode::Ft8, "TNX FER QSO").is_none());
        // WSPR is WSPRnet's
        assert!(skip(Mode::Wspr, "BG9PQZ OM89 37").is_none());
        // our own call (any form) as the sender
        assert!(skip(Mode::Ft8, "CQ K1ABC FN20").is_none());
        assert!(skip(Mode::Ft8, "CQ K1ABC/P FN20").is_none());
        // our own call and our own grid in the text: our transmission heard again
        assert!(skip(Mode::Ft8, "K1ABC JA1XYZ FN20").is_none());
        // but a station answering us is a station heard
        assert!(skip(Mode::Ft8, "K1ABC JA1XYZ PM95").is_some());
        // no clock, no time, no spot
        let mut d = decode(Mode::Ft8, "RA0ANO UA0LQE PN53");
        d.slot_utc_ns = None;
        assert!(spot_from_decode(&d, "K1ABC", "FN20").is_none());
    }

    #[test]
    fn modes_are_psk_reporters_names() {
        for (m, name) in [
            (Mode::Ft4, "FT4"),
            (Mode::Fst4S60, "FST4"),
            (Mode::Jt65, "JT65"),
            (Mode::Jt9, "JT9"),
            (Mode::Q65A60, "Q65"),
        ] {
            let s = spot_from_decode(&decode(m, "CQ UA0LQE PN53"), "K1ABC", "FN20").unwrap();
            assert_eq!(s.mode, name);
        }
    }

    fn jtty(text: &str, kind: UpdateKind) -> JttyMessage {
        JttyMessage {
            channel: 0,
            key: 1,
            start_utc_ns: Some(1_778_068_790_000_000_000),
            end_utc_ns: Some(1_778_068_800_000_000_000),
            dial_hz: 14_083_000.0,
            freq_hz: 14_084_500.2,
            snr_db: -4.6,
            text: text.to_string(),
            calls: Vec::new(),
            kind,
        }
    }

    #[test]
    fn a_complete_jtty_message_is_a_spot_at_its_start() {
        let s = spot_from_jtty(&jtty("CQ JA1XYZ PM95", UpdateKind::Complete), "K1ABC").unwrap();
        assert_eq!(
            (s.callsign.as_str(), s.locator.as_str(), s.mode.as_str()),
            ("JA1XYZ", "PM95", "JTTY")
        );
        assert_eq!(s.time_unix, 1_778_068_790);
        assert_eq!(s.frequency, 14_084_500);
        // a message that grew, expired or was cut off is not heard whole
        for k in [
            UpdateKind::Growing,
            UpdateKind::Expired,
            UpdateKind::ReceptionEnded,
        ] {
            assert!(
                spot_from_jtty(&jtty("CQ JA1XYZ PM95", k), "K1ABC").is_none(),
                "{k:?}"
            );
        }
        assert!(spot_from_jtty(&jtty("CQ K1ABC FN20", UpdateKind::Complete), "K1ABC").is_none());
    }

    #[test]
    fn bands_and_the_once_per_band_rule() {
        assert_eq!(band_of(14_074_000), band_of(14_080_000));
        assert_ne!(band_of(14_074_000), band_of(7_074_000));
        assert_ne!(
            band_of(14_074_000),
            band_of(14_400_000),
            "outside the band falls to MHz"
        );
        let spot = |call: &str, f: u64| Spot {
            callsign: call.into(),
            locator: "FN21".into(),
            snr: -10,
            frequency: f,
            mode: "FT8".into(),
            time_unix: 0,
        };
        let mut d = Dedupe::new(Duration::from_secs(300));
        let t0 = Instant::now();
        assert!(d.admit(&spot("JA1XYZ", 14_074_000), t0));
        assert!(
            !d.admit(&spot("ja1xyz", 14_080_000), t0 + Duration::from_secs(10)),
            "same band, case-insensitive"
        );
        assert!(
            d.admit(&spot("JA1XYZ", 7_074_000), t0 + Duration::from_secs(10)),
            "another band"
        );
        assert!(d.admit(&spot("W1AW", 14_074_000), t0 + Duration::from_secs(10)));
        assert!(
            d.admit(&spot("JA1XYZ", 14_074_000), t0 + Duration::from_secs(301)),
            "after the repeat"
        );
        d.prune(t0 + Duration::from_secs(2000));
        assert!(d.seen.is_empty());
    }

    #[test]
    fn configuration_rules() {
        let mut c = PskConfig::new("k1abc", "FN20");
        assert_eq!(c.callsign, "K1ABC");
        assert_eq!(
            c.endpoint.address(),
            "pskreporter.info:14739",
            "the test listener is the default"
        );
        assert_eq!(
            Endpoint::Production.address(),
            "report.pskreporter.info:4739"
        );
        assert!(c.problem().is_none());
        c.locator = "FN".into();
        assert!(c.problem().is_some());
        c.locator = "FN20".into();
        c.endpoint = Endpoint::Production;
        c.interval = Duration::from_secs(120);
        assert!(
            c.problem().is_some(),
            "production never sends faster than every five minutes"
        );
        c.endpoint = Endpoint::Custom("127.0.0.1:9".into());
        assert!(c.problem().is_none(), "a custom listener may");
        assert!(PskConfig::new("", "FN20").problem().is_some());
    }
}
