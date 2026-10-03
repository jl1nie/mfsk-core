// SPDX-License-Identifier: GPL-3.0-or-later
//! The sending station and its locator, read from a decoded message's text.
//!
//! A heuristic over the standard message shapes, which is all a skimmer's
//! statistics need: `CQ [mod] CALL [GRID]`, `TO FROM [GRID|report]`, and
//! WSPR's `CALL GRID dBm`. Hashed calls (`<...>`) and free text give no sender.

/// `(sender, locator)`.
pub fn sender(mode: &str, text: &str) -> (Option<String>, Option<String>) {
    // A resolved hashed call prints as <K1ABC>; <...> stays unresolved.
    let t: Vec<&str> = text
        .split_whitespace()
        .map(|x| x.trim_matches(['<', '>']))
        .collect();
    if mode == "WSPR" {
        return match t.as_slice() {
            [call, grid, _] if is_call(call) && is_grid(grid) => {
                (Some((*call).into()), Some((*grid).into()))
            }
            [call, _, _] if is_call(call) => (Some((*call).into()), None),
            _ => (None, None),
        };
    }
    let from = if t.first() == Some(&"CQ") {
        // CQ DX / NA / EU / POTA / TEST / 000..999 before the call.
        t.iter()
            .skip(1)
            .take(2)
            .position(|x| is_call(x))
            .map(|i| i + 1)
    } else if t.len() >= 2 && is_call(t[1]) {
        Some(1)
    } else {
        None
    };
    let Some(i) = from else {
        return (None, None);
    };
    let grid = t
        .get(i + 1)
        .filter(|g| is_grid(g))
        .map(|g| (*g).to_string());
    (Some(t[i].to_string()), grid)
}

/// A letter-and-digit callsign, possibly with `/`: at least 3 characters, one
/// digit, one letter. Hashed `<...>` calls are not.
pub fn is_call(s: &str) -> bool {
    s.len() >= 3
        && s.len() <= 13
        && s.bytes().all(|c| c.is_ascii_alphanumeric() || c == b'/')
        && s.bytes().any(|c| c.is_ascii_digit())
        && s.bytes().any(|c| c.is_ascii_alphabetic())
        && !is_grid(s)
        && s != "RR73"
}

pub fn is_grid(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 4
        && b[0].is_ascii_uppercase()
        && b[1].is_ascii_uppercase()
        && (b'A'..=b'R').contains(&b[0])
        && (b'A'..=b'R').contains(&b[1])
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit()
        && s != "RR73"
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(m: &str, t: &str) -> (Option<String>, Option<String>) {
        sender(m, t)
    }
    fn some(c: &str, g: Option<&str>) -> (Option<String>, Option<String>) {
        (Some(c.into()), g.map(Into::into))
    }

    #[test]
    fn standard_messages() {
        assert_eq!(s("FT8", "CQ JA1ABC PM95"), some("JA1ABC", Some("PM95")));
        assert_eq!(s("FT8", "CQ DX K1ABC FN42"), some("K1ABC", Some("FN42")));
        assert_eq!(s("FT8", "CQ POTA W9XYZ EN34"), some("W9XYZ", Some("EN34")));
        assert_eq!(s("FT8", "JA1ABC K1XYZ -10"), some("K1XYZ", None));
        assert_eq!(s("FT8", "JA1ABC K1XYZ FN31"), some("K1XYZ", Some("FN31")));
        assert_eq!(s("FT8", "JA1ABC K1XYZ RR73"), some("K1XYZ", None));
        assert_eq!(s("FT8", "JA1ABC K1XYZ R-05"), some("K1XYZ", None));
        assert_eq!(s("FT8", "CQ 000 DL1ABC JO62"), some("DL1ABC", Some("JO62")));
    }

    #[test]
    fn hashed_and_free_text() {
        assert_eq!(s("FT8", "<...> K1ABC FN42"), some("K1ABC", Some("FN42")));
        assert_eq!(s("FT8", "K1ABC <...> -10"), (None, None));
        assert_eq!(s("FT8", "K1ABC <JA1ABC> -10"), some("JA1ABC", None));
        assert_eq!(s("FT8", "TNX 73 GL"), (None, None));
    }

    #[test]
    fn wspr() {
        assert_eq!(s("WSPR", "K1ABC FN42 37"), some("K1ABC", Some("FN42")));
        assert_eq!(s("WSPR", "<K1ABC> FN42XX 37"), some("K1ABC", None));
        assert_eq!(s("WSPR", "<...> FN42XX 37"), (None, None));
    }
}
