// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65 a-priori candidate-codeword generator.
//!
//! Builds the same 206-codeword "full AP list" that
//! `lib/qra/q65/q65_set_list.f90` emits in WSJT-X — every standard
//! exchange a known callsign pair could legally produce, encoded
//! through the Q65 FEC and ready to feed
//! [`crate::fec::qra::Q65Codec::decode_with_codeword_list`].
//!
//! The list comprises:
//!
//! - `MYCALL HISCALL` (bare exchange)
//! - `MYCALL HISCALL RRR` / `RR73` / `73` (acknowledgements)
//! - `CQ HISCALL HISGRID4` and `MYCALL HISCALL HISGRID4` (when a
//!   grid is supplied)
//! - 200-entry SNR ladder — alternating `±SS` and `R±SS` for every
//!   SNR in `-50..=49`
//!
//! Templates that the WSJT77 packer rejects (e.g. an SNR outside
//! the legal range, an invalid grid) are silently skipped, so the
//! returned list is always a self-consistent subset.
//!
//! ## Scope
//!
//! Non-standard callsigns (the `<MYCALL>` bracket logic in the
//! Fortran) and the contest-mode caller history from
//! `q65_set_list2.f90` are intentionally out of scope here — they
//! are handled by separate generators (not yet ported).

use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::q65::pack77_to_symbols;
use crate::msg::wsjt77::pack77;

/// Hard cap matching `MAX_NCW` in the Fortran reference. Used both
/// to size the returned `Vec` and to bound the generated list when
/// every template happens to pack successfully.
pub const MAX_AP_CODEWORDS: usize = 206;

/// Build the full AP candidate set for a known standard-callsign
/// pair. `his_grid` may be empty to skip the two grid-bearing
/// templates (i = 5 and i = 6).
///
/// Each entry is a 63-symbol GF(64) channel codeword — the same
/// shape [`crate::fec::qra::Q65Codec::decode_with_codeword_list`]
/// expects.
///
/// Returns an empty vector if `pack77` rejects every candidate (for
/// example because either callsign is unrepresentable).
pub fn standard_qso_codewords(my_call: &str, his_call: &str, his_grid: &str) -> Vec<[i32; 63]> {
    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut out: Vec<[i32; 63]> = Vec::with_capacity(MAX_AP_CODEWORDS);

    let mut push = |codec: &mut Q65Codec, c1: &str, c2: &str, report: &str| {
        if let Some(bits) = pack77(c1, c2, report) {
            let info = pack77_to_symbols(&bits);
            let mut cw = [0_i32; 63];
            codec.encode(&info, &mut cw);
            out.push(cw);
        }
    };

    // i = 1..=4: bare and acknowledgement variants.
    push(&mut codec, my_call, his_call, "");
    push(&mut codec, my_call, his_call, "RRR");
    push(&mut codec, my_call, his_call, "RR73");
    push(&mut codec, my_call, his_call, "73");

    // i = 5..=6: grid-bearing variants. The Fortran emits these
    // only when `his_std`; we approximate by skipping when the grid
    // is empty (pack77 rejects everything else for us).
    if !his_grid.trim().is_empty() {
        push(&mut codec, "CQ", his_call, his_grid);
        push(&mut codec, my_call, his_call, his_grid);
    }

    // i = 7..=206: SNR ladder. WSJT-X iterates isnr = -50..=49 with
    // alternating "+nn" and "R+nn" reports. The packer rejects
    // values outside `-50..=49` so we stay strictly inside that range.
    for isnr in -50..=49_i32 {
        // Width 3, sign-aware, zero-padded — matches the Fortran
        // `i3.2` format with the leading-space-to-'+' fixup applied.
        let bare = format!("{isnr:+03}");
        let r = format!("R{isnr:+03}");
        push(&mut codec, my_call, his_call, &bare);
        push(&mut codec, my_call, his_call, &r);
    }

    debug_assert!(
        out.len() <= MAX_AP_CODEWORDS,
        "AP list overflowed MAX_AP_CODEWORDS"
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{DecodeContext, MessageCodec};
    use crate::msg::Q65Message;
    use crate::msg::q65::unpack_symbols_to_bits77;

    fn unpack_codeword(cw: &[i32; 63]) -> String {
        // The first 13 channel symbols of a systematic Q65 codeword
        // ARE the user info — there's no permutation in the
        // qra15_65_64_irr_e23 systematic encoding. unpack to get a
        // human-readable message.
        let mut info = [0_i32; 13];
        info.copy_from_slice(&cw[..13]);
        let bits77 = unpack_symbols_to_bits77(&info);
        Q65Message
            .unpack(&bits77, &DecodeContext::default())
            .unwrap_or_else(|| "<unparseable>".into())
    }

    #[test]
    fn standard_list_contains_expected_templates() {
        let cws = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        let messages: Vec<String> = cws.iter().map(unpack_codeword).collect();
        for expected in [
            "K1ABC JA1ABC",
            "K1ABC JA1ABC RRR",
            "K1ABC JA1ABC RR73",
            "K1ABC JA1ABC 73",
            "CQ JA1ABC PM95",
            "K1ABC JA1ABC PM95",
            "K1ABC JA1ABC -10",
            "K1ABC JA1ABC R-10",
            "K1ABC JA1ABC +05",
            "K1ABC JA1ABC R+25",
        ] {
            assert!(
                messages.iter().any(|m| m == expected),
                "missing expected template {expected:?}; first 12 = {:?}",
                &messages[..12.min(messages.len())]
            );
        }
    }

    #[test]
    fn standard_list_size_matches_reference() {
        // Standard-call pair with grid → 4 + 2 grid + 200 SNR = 206.
        let with_grid = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        assert_eq!(
            with_grid.len(),
            MAX_AP_CODEWORDS,
            "with-grid list size must hit MAX_NCW = 206"
        );
        // No-grid call pair → 4 + 200 = 204.
        let no_grid = standard_qso_codewords("K1ABC", "JA1ABC", "");
        assert_eq!(
            no_grid.len(),
            MAX_AP_CODEWORDS - 2,
            "no-grid list size must be MAX_NCW - 2"
        );
    }

    #[test]
    fn invalid_my_callsign_only_emits_callsign_independent_templates() {
        // The "CQ HISCALL HISGRID4" template is independent of
        // `my_call`, so a garbage `my_call` should still leave that
        // one alone — but every template that depends on `my_call`
        // (the bare/RRR/RR73/73, the SNR ladder, the
        // `MYCALL HISCALL grid` form) must be rejected by the WSJT77
        // packer and silently dropped.
        let cws = standard_qso_codewords("!!!", "JA1ABC", "PM95");
        assert_eq!(
            cws.len(),
            1,
            "garbage my_call should leave only the CQ template, got {} codewords",
            cws.len()
        );
        assert_eq!(unpack_codeword(&cws[0]), "CQ JA1ABC PM95");
    }

    #[test]
    fn distinct_callsign_pairs_yield_distinct_lists() {
        // Pin a basic uniqueness invariant: the codeword set for
        // (K1ABC, JA1ABC) must NOT equal the set for (K1ABC, JA9XYZ)
        // — protects against accidentally hashing call signs out of
        // the encoder.
        let a = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        let b = standard_qso_codewords("K1ABC", "JA9XYZ", "PM95");
        assert_eq!(a.len(), b.len());
        assert_ne!(a, b, "different call pairs produced identical AP lists");
    }
}
