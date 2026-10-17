// SPDX-License-Identifier: GPL-3.0-only
//! `unpack77` and `pack77_wspr` against WSJT-X v3.2.0-rc1 on hand-built bit patterns (tier A).
//!
//! Each vector below was run through rc1's own `unpack77` (`lib/77bit/packjt77.f90` from the
//! `v3.2.0-rc1` tag, built as `scripts/pack77/build_oracle.sh` builds it, with a driver that reads
//! raw 77-bit strings instead of messages). The expected text is what that run printed; `None` is
//! `ok = .false.`. `packjt77.f90` is unchanged between v3.3.0-beta1 and beta2.
//!
//! Telemetry text and the unresolved-hash placeholders of types 0.1, 4 and 5 are also covered row by
//! row by `pack77_message.rs` against `tests/fixtures/pack77_rc1.tsv`.

use mfsk_core::msg::wsjt77::{Pack77Options, pack77_message, pack77_wspr, unpack77};

fn bits(s: &str) -> Vec<u8> {
    assert_eq!(s.len(), 77, "{s}");
    s.bytes().map(|b| b - b'0').collect()
}

/// `(bits, rc1 text, what it pins)`. Type-1 vectors: `n28a 0 n28b 0 0 igrid15 001`, grid `AA03`,
/// second call `K1ABC` (or first, for the second call under test).
const VECTORS: &[(&str, Option<&str>, &str)] = &[
    // 1. `unpack28_core` fails a standard call `callok` refuses (`packjt77.f90:1078`, `:2438-2485`).
    (
        "01010010011000101010101111010000010011011110111100011010100000000000000011001",
        None,
        "call1 AB1 (callok)",
    ),
    (
        "00001001101111011110001101010011101100111101101111110011000000000000000011001",
        None,
        "call2 FN4 (callok)",
    ),
    (
        "00001001101111011110001101010000011000010100100111011100000000000000000011001",
        Some("K1ABC W9XYZ AA03"),
        "two calls callok accepts",
    ),
    // 1b. a blank inside a directional CQ word fails the same way (`:1088-1094`).
    (
        "00000000000001010001000001000000010011011110111100011010100000000000000011001",
        None,
        "CQ with a blank inside its four letters",
    ),
    (
        "00000000000001010110110100010000010011011110111100011010100000000000000011001",
        Some("CQ ABC K1ABC AA03"),
        "CQ ABC",
    ),
    // 2. 28-bit fields past the tokens and in the 22-bit hash range, no table: `<...>`
    //    (`hash22`, `:283-297`: `c13='<...>'`; a negative hash matches no entry).
    (
        "00000000100100100111110000000000010011011110111100011010100000000000000011001",
        Some("<...> K1ABC AA03"),
        "n28 600000, between the CQ tokens and NTOKENS",
    ),
    (
        "00000001111101111100111001110000010011011110111100011010100000000000000011001",
        Some("<...> K1ABC AA03"),
        "n28 NTOKENS-1",
    ),
    (
        "00000001111101111100111011010000010011011110111100011010100000000000000011001",
        Some("<...> K1ABC AA03"),
        "n28 NTOKENS+5, a real 22-bit hash",
    ),
    // 3. telemetry: `format(3z6.6)` (`:1640`), 18 digits with the zeros kept.
    (
        "00000000000000000000000000000000000000000000000000000000000000000000000101000",
        Some("000000000000000000"),
        "telemetry, all zero",
    ),
    (
        "00000000000000000000000000000000000000000000000000000000000000000000001101000",
        Some("000000000000000001"),
        "telemetry, one",
    ),
];

#[test]
fn unpack77_matches_rc1_on_callok_hash_placeholders_and_telemetry() {
    let mut bad = Vec::new();
    for (b, want, what) in VECTORS {
        let got = unpack77(&bits(b));
        if got.as_deref().map(str::trim_end) != *want {
            bad.push(format!("{what}: want {want:?}, got {got:?}"));
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// No decode path prints `<?>` any more: it is `<...>`, the one placeholder rc1 has.
#[test]
fn no_28_bit_field_renders_the_old_placeholder() {
    for n28 in (532_444u32..2_063_592).step_by(997).chain([2_063_591]) {
        let mut b = [0u8; 77];
        for (i, bit) in b.iter_mut().enumerate().take(28) {
            *bit = ((n28 >> (27 - i)) & 1) as u8;
        }
        b[74 + 2] = 1; // i3 = 1
        let s = unpack77(&b);
        assert!(
            s.as_deref().is_some_and(|s| !s.contains("<?>")),
            "{n28}: {s:?}"
        );
    }
}

/// `pack77_06_candidate` (`packjt77.f90:2259-2303`): the 0.6 candidate goes through
/// `lower_c28_slot` and the unpack gate, so a call `callok` refuses is no WSPR message; rc1's
/// whole-message `pack77` falls back to free text (`AB1 FN42 37`: status ok, 0.0, run on the oracle).
#[test]
fn pack77_wspr_refuses_what_callok_refuses() {
    assert!(pack77_wspr("K1ABC FN42 37", false).is_some());
    assert!(pack77_wspr("AB1 FN42 37", false).is_none());
    assert!(pack77_wspr("FN4 FN42 37", false).is_none());
    // a type-2 form keeps the base call: `AB1/K1ABC` is a prefix of K1ABC (oracle: 0.6, ok).
    assert!(pack77_wspr("AB1/K1ABC 37", false).is_some());
    let p = pack77_message("AB1 FN42 37", &Pack77Options::default()).expect("free text");
    assert_eq!((p.i3, p.n3), (0, 0));
}
