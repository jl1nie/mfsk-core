// SPDX-License-Identifier: GPL-3.0-only
//! The PSK Reporter IPFIX builder against WSJT-X v3.3.0-beta1's own (tier B).
//!
//! `embedded-poc/assets/golden/pskreporter/ipfix_script.tsv` is a script of receivers, spots and
//! build parameters; `ipfix_beta1.txt` is what `PSKReporterIPFIX::buildPackets` made of it,
//! through `scripts/pskreporter/ipfix_oracle.cpp`. Every datagram must be the same bytes: UDP
//! and TCP, with and without the templates, split across datagrams, multi-byte text cut at its
//! limits, a five-byte frequency, a wrapping SNR and a 32-bit time. The structural tests under
//! it are upstream's own (`tests/unit/network/test_pskreporter_ipfix.cpp`).

use skimmer_core::pskreporter::{
    MAX_TCP_IPFIX_PAYLOAD_BYTES, MAX_UDP_IPFIX_PAYLOAD_BYTES, Receiver, Spot, build_packets,
};

const DIR: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../../embedded-poc/assets/golden/pskreporter/"
);

/// `{rep:N:TEXT}` is TEXT repeated N times; anything else is itself.
fn expand(s: &str) -> String {
    if let Some(rest) = s.strip_prefix("{rep:").and_then(|r| r.strip_suffix('}')) {
        let (n, text) = rest.split_once(':').expect("{rep:N:TEXT}");
        return text.repeat(n.parse().unwrap());
    }
    s.to_string()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn replay(script: &str) -> String {
    let mut out = String::new();
    let mut rx = Receiver::default();
    let mut spots: Vec<Spot> = Vec::new();
    let (mut tcp, mut desc, mut seq, mut obs, mut exp) = (false, false, 0u32, 0u32, 0u32);
    for line in script
        .lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
    {
        let f: Vec<&str> = line.split('\t').collect();
        match f[0] {
            "P" => {
                tcp = f[1] == "tcp";
                desc = f[2] == "1";
                seq = f[3].parse().unwrap();
                obs = f[4].parse().unwrap();
                exp = f[5].parse().unwrap();
            }
            "R" => {
                rx = Receiver {
                    callsign: expand(f[1]),
                    locator: expand(f[2]),
                    program_info: expand(f[3]),
                    antenna: expand(f[4]),
                    rig_information: expand(f[5]),
                }
            }
            "S" => spots.push(Spot {
                callsign: expand(f[1]),
                locator: expand(f[2]),
                snr: f[3].parse().unwrap(),
                frequency: f[4].parse().unwrap(),
                mode: expand(f[5]),
                time_unix: f[6].parse().unwrap(),
            }),
            "GO" => {
                let max = if tcp {
                    MAX_TCP_IPFIX_PAYLOAD_BYTES
                } else {
                    MAX_UDP_IPFIX_PAYLOAD_BYTES
                };
                let packets = build_packets(&rx, &spots, desc, seq, obs, exp, max);
                for p in &packets {
                    out.push_str(&format!("K {} {}\n", p.spot_count, hex(&p.payload)));
                }
                out.push_str(&format!("E {} {}\n", packets.len(), u8::from(tcp)));
                spots.clear();
            }
            other => panic!("unknown command {other:?}"),
        }
    }
    out
}

#[test]
fn packets_are_upstreams_bytes() {
    let script =
        std::fs::read_to_string(format!("{DIR}ipfix_script.tsv")).expect("vendored script");
    let want: String = std::fs::read_to_string(format!("{DIR}ipfix_beta1.txt"))
        .expect("vendored upstream output")
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|l| format!("{l}\n"))
        .collect();
    let got = replay(&script);
    if got != want {
        let (g, w): (Vec<&str>, Vec<&str>) = (got.lines().collect(), want.lines().collect());
        let at = g
            .iter()
            .zip(&w)
            .position(|(a, b)| a != b)
            .unwrap_or(g.len().min(w.len()));
        panic!(
            "first difference at output line {at} of {} vs {}:\n ours     {:.200}\n upstream {:.200}",
            g.len(),
            w.len(),
            g.get(at).unwrap_or(&""),
            w.get(at).unwrap_or(&"")
        );
    }
    assert!(
        want.lines().filter(|l| l.starts_with("K ")).count() >= 50,
        "the script covers splits"
    );
    assert!(
        want.lines().any(|l| l.starts_with("K 30 ")),
        "full UDP datagrams carry 30 spots"
    );
}

fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_be_bytes([b[o], b[o + 1]])
}
fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_be_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn set_ids(p: &[u8]) -> Vec<u16> {
    let (mut ids, mut o) = (Vec::new(), 16);
    while o + 4 <= p.len() {
        ids.push(u16_at(p, o));
        let len = u16_at(p, o + 2) as usize;
        if len < 4 {
            break;
        }
        o += len;
    }
    ids
}
fn rx() -> Receiver {
    Receiver {
        callsign: "K1ABC".into(),
        locator: "FN20".into(),
        program_info: "mfsk-skimmer test".into(),
        antenna: "N/A".into(),
        rig_information: "N/A".into(),
    }
}
fn spot(i: u64) -> Spot {
    Spot {
        callsign: format!("K{i}ABC"),
        locator: "FN21".into(),
        snr: -10,
        frequency: 14_074_000 + i,
        mode: "FT8".into(),
        time_unix: 1_778_068_800,
    }
}

#[test]
fn a_header_says_its_own_length_the_sequence_and_the_ids() {
    let spots: Vec<Spot> = (0..5).map(spot).collect();
    let p = &build_packets(&rx(), &spots, true, 77, 0xdead_beef, 1_700_000_000, 1000)[0];
    assert_eq!(u16_at(&p.payload, 0), 10, "IPFIX version");
    assert_eq!(u16_at(&p.payload, 2) as usize, p.payload.len());
    assert_eq!(p.payload.len() % 4, 0);
    assert_eq!(u32_at(&p.payload, 4), 1_700_000_000);
    assert_eq!(u32_at(&p.payload, 8), 77);
    assert_eq!(u32_at(&p.payload, 12), 0xdead_beef);
}

#[test]
fn many_spots_split_into_bounded_datagrams_and_the_sequence_counts_spots() {
    let spots: Vec<Spot> = (0..200).map(spot).collect();
    let packets = build_packets(
        &rx(),
        &spots,
        true,
        100,
        23,
        29,
        MAX_UDP_IPFIX_PAYLOAD_BYTES,
    );
    assert!(packets.len() > 1);
    let mut expected = 100u32;
    for p in &packets {
        assert!(p.payload.len() <= MAX_UDP_IPFIX_PAYLOAD_BYTES);
        assert_eq!(
            u32_at(&p.payload, 8),
            expected,
            "sequence = spots before it"
        );
        expected += p.spot_count as u32;
        let ids = set_ids(&p.payload);
        assert!(
            ids.contains(&0x50e2) && ids.contains(&0x50e3),
            "receiver and sender sets"
        );
    }
    assert_eq!(expected, 100 + spots.len() as u32);
    // the templates (set ids 2 and 3) are in the first datagram only
    assert!(set_ids(&packets[0].payload).contains(&2) && set_ids(&packets[0].payload).contains(&3));
    for p in &packets[1..] {
        assert!(!set_ids(&p.payload).iter().any(|&i| i == 2 || i == 3));
    }
    // TCP takes the lot in one
    let tcp = build_packets(&rx(), &spots, true, 1, 2, 3, MAX_TCP_IPFIX_PAYLOAD_BYTES);
    assert_eq!(tcp.len(), 1);
    assert_eq!(tcp[0].spot_count, 200);
}

#[test]
fn a_receiver_only_datagram_does_not_move_the_sequence() {
    let p = build_packets(&rx(), &[], true, 44, 23, 29, 1000);
    assert_eq!(p.len(), 1);
    assert_eq!(p[0].spot_count, 0);
    assert_eq!(u32_at(&p[0].payload, 8), 44);
    // the options template has exactly one scope field
    let b = &p[0].payload;
    let mut o = 16;
    assert_eq!(u16_at(b, o), 2);
    o += u16_at(b, o + 2) as usize;
    assert_eq!(u16_at(b, o), 3);
    assert_eq!(u16_at(b, o + 8), 1);
}
