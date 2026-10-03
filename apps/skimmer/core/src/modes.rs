// SPDX-License-Identifier: GPL-3.0-or-later
//! The names a user types for each `IqMode`, one table both ways.

use mfsk_core::iq::IqMode;

pub const MODES: &[(&str, IqMode)] = &[
    ("FT8", IqMode::Ft8),
    ("FT4", IqMode::Ft4),
    ("FST4-15", IqMode::Fst4S15),
    ("FST4-30", IqMode::Fst4S30),
    ("FST4-60", IqMode::Fst4S60),
    ("FST4-120", IqMode::Fst4S120),
    ("FST4-300", IqMode::Fst4S300),
    ("WSPR", IqMode::Wspr),
    ("JT9", IqMode::Jt9),
    ("JT65", IqMode::Jt65),
    ("Q65-15A", IqMode::Q65A15),
    ("Q65-30A", IqMode::Q65A30),
    ("Q65-60A", IqMode::Q65A60),
    ("Q65-60B", IqMode::Q65B60),
    ("Q65-60C", IqMode::Q65C60),
    ("Q65-60D", IqMode::Q65D60),
    ("Q65-60E", IqMode::Q65E60),
    ("Q65-120D", IqMode::Q65D120),
    ("Q65-120E", IqMode::Q65E120),
    ("Q65-300A", IqMode::Q65A300),
];

/// Case-insensitive.
pub fn parse_mode(s: &str) -> Option<IqMode> {
    MODES
        .iter()
        .find(|(n, _)| n.eq_ignore_ascii_case(s))
        .map(|&(_, m)| m)
}

pub fn mode_name(m: IqMode) -> &'static str {
    MODES
        .iter()
        .find(|&&(_, x)| x == m)
        .map(|&(n, _)| n)
        .unwrap_or("?")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_round_trip() {
        for &(n, m) in MODES {
            assert_eq!(parse_mode(n), Some(m));
            assert_eq!(parse_mode(&n.to_ascii_lowercase()), Some(m));
            assert_eq!(mode_name(m), n);
        }
        assert_eq!(parse_mode("FT9"), None);
    }
}
