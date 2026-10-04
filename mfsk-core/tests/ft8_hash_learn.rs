// SPDX-License-Identifier: GPL-3.0-only
//! `<...>` resolves once the table has heard the callsign.
//!
//! The gap this pins: `unpack77_with_hash` existed and resolved
//! hashes, but nothing in the crate or its consumers ever *filled* a
//! table outside a unit test, so a receiver rendered `<...>` forever.
//! A CoreS3 on 7041 kHz measured 69 of 380 decodes carrying one
//! (2026-09-20).

use mfsk_core::msg::CallsignHashTable;
use mfsk_core::msg::wsjt77::{pack77, unpack77, unpack77_learn, unpack77_with_hash};

/// **The mechanism, end to end.** A 28-bit callsign field can instead
/// carry a 22-bit hash (`n28 >= NTOKENS`, `unpack28_h`), which is how
/// `<...> JH1BCS PM95` reaches the air — 13 copies of exactly that
/// line in the CoreS3's 7041 kHz log. It resolves only if the table
/// has previously *heard* the station in a message that spelled it
/// out.
#[test]
fn a_hashed_callsign_field_resolves_once_the_call_has_been_heard() {
    // Same layout `unpack77_with_hash`'s `i3 = 1 | 2` arm reads.
    const NTOKENS: u32 = 2_063_592;
    fn payload(n28a: u32, n28b: u32, igrid: u32, i3: u32) -> Vec<u8> {
        let mut m = vec![0u8; 77];
        let put = |m: &mut Vec<u8>, off: usize, n: u32, w: usize| {
            for i in 0..w {
                m[off + i] = ((n >> (w - 1 - i)) & 1) as u8;
            }
        };
        put(&mut m, 0, n28a, 28);
        put(&mut m, 29, n28b, 28);
        put(&mut m, 59, igrid, 15);
        put(&mut m, 74, i3, 3);
        m
    }

    let hashed = NTOKENS + mfsk_core::msg::hash_table::ihashcall("JA1ABC", 22);
    let plain = mfsk_core::msg::wsjt77::pack28("JE1NGI").expect("pack28 JE1NGI");
    let msg = payload(hashed, plain, 12_345, 1);

    // Cold: the field has no name behind it.
    let cold = CallsignHashTable::new();
    let cold_text = unpack77_with_hash(&msg, &cold).expect("unpack cold");
    assert!(
        cold_text.starts_with("<...>"),
        "expected an unresolved placeholder, got {cold_text}"
    );

    // Warm: hear `JA1ABC` spelled out in an earlier message, through
    // the learning path, and the same payload names it.
    let mut warm = CallsignHashTable::new();
    let intro = pack77("JA1ABC", "JE1NGI", "PM95").expect("pack intro");
    let _ = unpack77_learn(&intro, &mut warm);
    let warm_text = unpack77_with_hash(&msg, &warm).expect("unpack warm");
    // WSJT-X keeps the angle brackets on a *resolved* hash — they say
    // "this was sent as a hash", which is information the operator
    // wants. `<...>` means unresolved; `<JA1ABC>` means resolved.
    assert!(
        warm_text.starts_with("<JA1ABC>"),
        "heard the call and still could not resolve it: {warm_text}"
    );
}

/// The standard calls in an ordinary exchange are registered too, so a
/// later hashed reference to either of them resolves.
#[test]
fn standard_calls_are_registered_from_an_ordinary_exchange() {
    let m = pack77("JA1ABC", "JE1NGI", "PM95").expect("pack std std grid");
    let mut ht = CallsignHashTable::new();
    let _ = unpack77_learn(&m, &mut ht);

    // `insert` fills all three widths; the 22-bit one is the table's
    // own record of what it has heard.
    assert!(
        ht.len22() >= 2,
        "expected both calls registered, got {}",
        ht.len22()
    );
}

/// **Nothing that is not a callsign may enter the table.** A token
/// scan over the rendered text would register `RRR`, `DX` and the
/// grid; those collide with real hashes and resolve a later `<...>`
/// to the wrong station, which is worse than leaving it unresolved.
#[test]
fn reports_grids_and_cq_are_not_registered() {
    let mut ht = CallsignHashTable::new();
    for (a, b, c) in [
        ("CQ", "JA1ABC", "PM95"),
        ("JA1ABC", "JE1NGI", "RRR"),
        ("JA1ABC", "JE1NGI", "RR73"),
        ("JA1ABC", "JE1NGI", "73"),
    ] {
        if let Some(m) = pack77(a, b, c) {
            let _ = unpack77_learn(&m, &mut ht);
        }
    }
    for junk in ["RRR", "RR73", "73", "PM95", "DX", "TU"] {
        let n22 = mfsk_core::msg::hash_table::ihashcall(junk, 22);
        assert!(
            ht.lookup22(n22).is_none_or(|c| c != junk),
            "{junk} was registered as a callsign"
        );
    }
}

/// The learning path must not change what a message unpacks to.
#[test]
fn learning_does_not_change_the_rendered_text() {
    let m = pack77("JA1ABC", "JE1NGI", "PM95").expect("pack");
    let mut ht = CallsignHashTable::new();
    assert_eq!(unpack77_learn(&m, &mut ht), unpack77(&m));
}
