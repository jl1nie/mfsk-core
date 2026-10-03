// SPDX-License-Identifier: GPL-3.0-or-later
//! The names a user types for each `Mode`, one table both ways, and each
//! mode's slot length from the library's registry.

use mfsk_core::Mode;

/// (name a user types, mode, the registry's name for it).
pub const MODES: &[(&str, Mode, &str)] = &[
    ("FT8", Mode::Ft8, "FT8"),
    ("FT4", Mode::Ft4, "FT4"),
    ("FST4-15", Mode::Fst4S15, "FST4-15"),
    ("FST4-30", Mode::Fst4S30, "FST4-30"),
    ("FST4-60", Mode::Fst4S60, "FST4-60A"),
    ("FST4-120", Mode::Fst4S120, "FST4-120"),
    ("FST4-300", Mode::Fst4S300, "FST4-300"),
    ("WSPR", Mode::Wspr, "WSPR"),
    ("JT9", Mode::Jt9, "JT9"),
    ("JT65", Mode::Jt65, "JT65"),
    ("Q65-15A", Mode::Q65A15, "Q65-15A"),
    ("Q65-30A", Mode::Q65A30, "Q65-30A"),
    ("Q65-60A", Mode::Q65A60, "Q65-60A"),
    ("Q65-60B", Mode::Q65B60, "Q65-60B"),
    ("Q65-60C", Mode::Q65C60, "Q65-60C"),
    ("Q65-60D", Mode::Q65D60, "Q65-60D"),
    ("Q65-60E", Mode::Q65E60, "Q65-60E"),
    ("Q65-120D", Mode::Q65D120, "Q65-120D"),
    ("Q65-120E", Mode::Q65E120, "Q65-120E"),
    ("Q65-300A", Mode::Q65A300, "Q65-300A"),
];

/// Case-insensitive.
pub fn parse_mode(s: &str) -> Option<Mode> {
    MODES
        .iter()
        .find(|(n, ..)| n.eq_ignore_ascii_case(s))
        .map(|&(_, m, _)| m)
}

pub fn mode_name(m: Mode) -> &'static str {
    MODES
        .iter()
        .find(|&&(_, x, _)| x == m)
        .map(|&(n, ..)| n)
        .unwrap_or("?")
}

/// The mode's slot (T/R period) in seconds, as the registry has it; slots
/// start on multiples of it from 00:00 UTC.
pub fn slot_seconds(m: Mode) -> f32 {
    MODES
        .iter()
        .find(|&&(_, x, _)| x == m)
        .and_then(|&(.., r)| mfsk_core::registry::by_name(r))
        .map(|meta| meta.t_slot_s)
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_round_trip() {
        for &(n, m, _) in MODES {
            assert_eq!(parse_mode(n), Some(m));
            assert_eq!(parse_mode(&n.to_ascii_lowercase()), Some(m));
            assert_eq!(mode_name(m), n);
        }
        assert_eq!(parse_mode("FT9"), None);
    }

    /// Every row names a registry entry, so every mode has a slot length.
    #[test]
    fn every_mode_has_a_registry_slot() {
        for &(n, m, _) in MODES {
            assert!(slot_seconds(m) > 0.0, "{n}");
        }
        assert_eq!(slot_seconds(Mode::Ft8), 15.0);
        assert_eq!(slot_seconds(Mode::Ft4), 7.5);
        assert_eq!(slot_seconds(Mode::Wspr), 120.0);
    }
}
