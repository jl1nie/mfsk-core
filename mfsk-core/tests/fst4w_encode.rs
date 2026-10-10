//! FST4W transmit encoding against WSJT-X's own `genfst4` with `iwspr=1`
//! (`v3.3.0-beta1`, #649): for each message in
//! `embedded-poc/assets/golden/fst4w/tones_beta1.tsv`, this crate must reject it
//! exactly when upstream says `*** bad message ***`, and otherwise give the same
//! 74 bits (50 payload + CRC-24) and the same 160 tones.
//!
//! The file is made by `scripts/fst4w/gen_tones_cases.sh` from curated and
//! seeded-random messages (all three WSPR message types, the edges of each
//! field, rejections); regenerating it with the oracle reproduces it byte for
//! byte. `msgsent` is compared where it does not depend on upstream's hash
//! table: for a `<CALL>` message upstream, having just transmitted it, renders
//! the call, where a fresh receiver renders `<...>`.
#![cfg(feature = "fst4w")]

#[allow(dead_code)]
mod common;

use mfsk_core::fst4w::encode::{message_to_tones, payload_to_info};
use mfsk_core::fst4w::{Fst4w120, Fst4w300, Fst4wMessage};
use mfsk_core::msg::wsjt77::unpack77;

const TSV: &str = asset_path!("golden/fst4w/tones_beta1.tsv");

#[test]
fn fst4w_encode_matches_genfst4() {
    let Ok(text) = std::fs::read_to_string(TSV) else {
        assert!(
            std::env::var("MFSK_REQUIRE_CORPUS").is_err(),
            "tones_beta1.tsv missing"
        );
        eprintln!("skipping: tones_beta1.tsv missing");
        return;
    };
    let (mut accepted, mut rejected, mut mangled) = (0, 0, 0);
    let mut by_kind = [0usize; 3]; // type 1, type 2 (has '/'), type 3 ('<')
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        // The message itself may hold a tab: split from the right.
        let f: Vec<&str> = line.rsplitn(6, '\t').collect();
        let (tones, bits, _i3n3, iwspr, msgsent, msg) = (f[0], f[1], f[2], f[3], f[4], f[5]);
        assert_eq!(iwspr, "1");
        // Upstream bug, deliberately not reproduced: `genfst4.f90:40-43` strips a
        // leading blank with `message=message(i+1:)`, which drops `i+1`
        // characters at pass `i`. One leading blank is fine; two or more eat the
        // start of the message (`  PJ4/K1ABC 37` is sent as `J4/K1ABC 37`).
        // `pack_text` strips blanks properly.
        if msg.starts_with("  ") {
            mangled += 1;
            continue;
        }
        let got = Fst4wMessage::pack_text(msg);
        if msgsent == "*** bad message ***" {
            rejected += 1;
            assert!(got.is_none(), "{msg:?}: upstream rejects, this accepts");
            assert!(message_to_tones::<Fst4w120>(msg).is_none());
            continue;
        }
        accepted += 1;
        let payload =
            got.unwrap_or_else(|| panic!("{msg:?}: upstream accepts ({msgsent}), this rejects"));
        let info = payload_to_info(&payload);
        let want_bits: Vec<u8> = bits.bytes().map(|b| b - b'0').collect();
        assert_eq!(info.to_vec(), want_bits, "{msg:?}: 74 bits");
        let want_tones: Vec<u8> = tones.bytes().map(|b| b - b'0').collect();
        assert_eq!(want_tones.len(), 160);
        assert_eq!(
            message_to_tones::<Fst4w120>(msg).unwrap(),
            want_tones,
            "{msg:?}: tones"
        );
        assert_eq!(
            message_to_tones::<Fst4w300>(msg).unwrap(),
            want_tones,
            "{msg:?}: tones are period-independent"
        );
        // What a receiver renders for these 50 bits.
        let shown = unpack77(&Fst4wMessage::payload_to_77(&payload).unwrap()).unwrap();
        if msgsent.contains('<') {
            by_kind[2] += 1;
            let up = msgsent.split_whitespace().last().unwrap();
            assert!(shown.ends_with(up), "{msg:?}: {shown:?} vs {msgsent:?}");
        } else {
            by_kind[usize::from(msg.contains('/'))] += 1;
            assert_eq!(shown, msgsent, "{msg:?}: msgsent");
        }
    }
    eprintln!(
        "accepted {accepted} (type1 {}, type2 {}, type3 {}), rejected {rejected}, skipped {mangled} with 2+ leading blanks",
        by_kind[0], by_kind[1], by_kind[2]
    );
    assert!(by_kind.iter().all(|&n| n >= 10) && rejected >= 200);
}
