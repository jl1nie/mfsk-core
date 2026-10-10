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
