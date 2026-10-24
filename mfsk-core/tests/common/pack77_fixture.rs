// SPDX-License-Identifier: GPL-3.0-only
//! Shared by the tests that read `tests/fixtures/pack77_rc1.tsv`.

/// The fixture is rc1's packer; v3.3.0-beta3 (58c7a522f) packs a non-CQ type 1/2 `... RR73` as the
/// acknowledgment `MAXGRID4 + 3` (32403 = `111111010010011`) instead of the grid RR73 (32373 =
/// `111111001110101`). Everything else in the fixture is unchanged in beta4, so the expected
/// bits of those rows are moved here rather than editing the generated file. Only a message
/// whose tail is the single word `RR73` (not `R RR73`, ir = 1) and whose first word is not
/// `CQ` is remapped.
pub fn beta4_rr73(msg: &str, i3n3: &str, bits: &str) -> String {
    let w: Vec<&str> = msg.split_whitespace().collect();
    let tail_is_rr73_alone = w.last() == Some(&"RR73") && w.len() >= 3 && w[w.len() - 2] != "R";
    if (i3n3 == "1.0" || i3n3 == "2.0")
        && tail_is_rr73_alone
        && w[0] != "CQ"
        && &bits[58..59] == "0"
        && &bits[59..74] == "111111001110101"
    {
        format!("{}111111010010011{}", &bits[..59], &bits[74..])
    } else {
        bits.to_string()
    }
}
