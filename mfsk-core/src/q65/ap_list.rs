// SPDX-License-Identifier: GPL-3.0-only
//! Q65 a-priori candidate-codeword generator.
//!
//! Builds the same 206-codeword (411 for a `/R` or `/P` DX call) "full AP list" that
//! `lib/qra/q65/q65_set_list.f90` (WSJT-X v3.3.0-beta4) emits — every standard
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
//! - for a standard `HISCALL` with a `/R` or `/P` suffix (`q65_stdcall`,
//!   `6b6124f9d`, `b2204822b`): `CQ HISCALL` (207), and when `MYCALL` is
//!   standard too `<MYCALL> HISCALL` + nothing / `RRR` / `RR73` / `73`
//!   (208-211) and `MYCALL <HISCALL>` + the same 200-entry ladder (212-411,
//!   no ±30 dB cut-off, unlike FT8's a8)
//!
//! Templates that the WSJT77 packer rejects (e.g. an SNR outside
//! the legal range, an invalid grid) are silently skipped, so the
//! returned list is always a self-consistent subset.
//!
//! ## Scope
//!
//! Non-standard callsigns (the `<MYCALL>` bracket logic for entries 1-206 in
//! the Fortran) are out of scope here; the `/R` and `/P` entries 207-411 are in. The contest-mode list,
//! `q65_set_list2.f90`, is [`super::contest::contest_codewords`].

use alloc::format;
use alloc::vec::Vec;

use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::q65::pack77_q65;
use crate::msg::q65::pack77_to_symbols;
use crate::msg::q65::q65_rr73_fixup;
use crate::msg::wsjt77::{Pack77Options, has_rp_suffix, is_standard_callsign, pack77_message};

/// Hard cap matching `MAX_NCW` in the Fortran reference (206 up to v3.3.0-beta3, 411 since
/// beta4). Used both to size the returned `Vec` and to bound the generated list when
/// every template happens to pack successfully.
pub const MAX_AP_CODEWORDS: usize = 411;

/// The 206 entries every pair gets (`ncw` of `q65_set_list`, before the `/R` `/P` extension).
const BASE_AP_CODEWORDS: usize = 206;

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
    let mut out: Vec<[i32; 63]> = Vec::with_capacity(BASE_AP_CODEWORDS);

    let mut push = |codec: &mut Q65Codec, c1: &str, c2: &str, report: &str| {
        if let Some(bits) = pack77_q65(c1, c2, report) {
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

    // i = 207..=411 (v3.3.0-beta4): a standard call with a /R or /P suffix keeps the candidates
    // the former nonstandard-call path produced: a CQ without a grid (207) and, when MyCall is
    // standard, <MyCall> HisCall with nothing, RRR, RR73 or 73 (208-211) and
    // MyCall <HisCall> with a report or R-report (212-411, as 7-206).
    if is_standard_callsign(his_call) && has_rp_suffix(his_call) {
        let (my, his) = (my_call.trim(), his_call.trim());
        let mut push_text = |codec: &mut Q65Codec, text: &str| {
            // `genq65`: pack the text, rewrite a grid-square RR73 (a hashed call is packed as
            // by `pack77`; nothing is committed to a hash table here).
            // A message that does not pack is `genq65`'s `*** bad message ***`; `q65_set_list`
            // still stores the slot, as codewords of -1 (as `contest_codewords` does).
            if let Ok(p) = pack77_message(text.trim_end(), &Pack77Options::new()) {
                let mut bits = p.bits;
                q65_rr73_fixup(&mut bits);
                let info = pack77_to_symbols(&bits);
                let mut cw = [0_i32; 63];
                codec.encode(&info, &mut cw);
                out.push(cw);
            } else {
                out.push([-1_i32; 63]);
            }
        };
        push_text(&mut codec, &format!("CQ {his}"));
        if is_standard_callsign(my) {
            for tail in ["", "RRR", "RR73", "73"] {
                push_text(&mut codec, &format!("<{my}> {his} {tail}"));
            }
            for k in 7..=BASE_AP_CODEWORDS {
                // `isnr = -50 + (i-205-7)/2`, odd `i-205` plain, even `R`-prefixed.
                let isnr = -50 + (k as i32 - 7) / 2;
                let r = if k % 2 == 1 { "" } else { "R" };
                push_text(&mut codec, &format!("{my} <{his}> {r}{isnr:+03}"));
            }
        }
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
    use crate::engine::{DecodeContext, MessageCodec};
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
            BASE_AP_CODEWORDS,
            "with-grid list size must be 206"
        );
        // No-grid call pair → 4 + 200 = 204.
        let no_grid = standard_qso_codewords("K1ABC", "JA1ABC", "");
        assert_eq!(
            no_grid.len(),
            BASE_AP_CODEWORDS - 2,
            "no-grid list size must be 204"
        );
    }

    /// The /R and /P entries 207-411 (`q65_set_list`, v3.3.0-beta4).
    #[test]
    fn suffix_dx_call_gets_the_extra_candidates() {
        let cws = standard_qso_codewords("JA1ABC", "K1ABC/R", "FN42");
        assert_eq!(cws.len(), MAX_AP_CODEWORDS);
        let t: Vec<String> = cws.iter().map(unpack_codeword).collect();
        // A hashed call reads back as `<...>` without a hash table.
        assert_eq!(t[0], "JA1ABC K1ABC/R");
        assert_eq!(t[206], "CQ K1ABC/R");
        assert_eq!(t[207], "<...> K1ABC/R");
        assert_eq!(t[208], "<...> K1ABC/R RRR");
        assert_eq!(t[209], "<...> K1ABC/R RR73");
        assert_eq!(t[210], "<...> K1ABC/R 73");
        // j = 7 (i = 212): -50, no +-30 dB cut-off here.
        assert_eq!(t[211], "JA1ABC <...> -50");
        assert_eq!(t[212], "JA1ABC <...> R-50");
        assert_eq!(t[410], "JA1ABC <...> R+49");

        // MyCall with a suffix too: `JA1ABC/P <K1ABC/P> -10` is no Type 1 message (a hashed
        // call cannot sit next to a suffixed one) and not free text either, so `genq65` has no
        // codeword for the 200 ladder entries 212-411: they keep their slots as -1, and ncw
        // stays 411 as upstream (4 + 200 without a grid, then the 207-411 block).
        let p = standard_qso_codewords("JA1ABC/P", "K1ABC/P", "");
        assert_eq!(p.len(), 4 + 200 + 205);
        assert!(p[204 + 5..].iter().all(|cw| cw == &[-1_i32; 63]));
        // MyCall not standard: only the CQ is added (the entries 1-206 that need a hashed
        // MyCall are out of scope here).
        let q = standard_qso_codewords("PJ4/K1ABC", "K1ABC/P", "FN42");
        let qt: Vec<String> = q.iter().map(unpack_codeword).collect();
        assert_eq!(qt.last().map(String::as_str), Some("CQ K1ABC/P"));
        assert!(!qt.iter().any(|m| m.starts_with("<...>")));
        // A plain DX call stays at 206, and the extra candidates are the same bits as the
        // text packed directly.
        assert_eq!(standard_qso_codewords("JA1ABC", "K1ABC", "FN42").len(), 206);
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut bits = pack77_message("<JA1ABC> K1ABC/R RR73", &Pack77Options::new())
            .unwrap()
            .bits;
        q65_rr73_fixup(&mut bits);
        let mut cw = [0_i32; 63];
        codec.encode(&pack77_to_symbols(&bits), &mut cw);
        assert_eq!(cws[209], cw);
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
