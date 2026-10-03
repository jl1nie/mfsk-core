// SPDX-License-Identifier: GPL-3.0-or-later
//! The names a user types for each `IqMode`, one table both ways, and each
//! mode's slot length from the library's registry.

use mfsk_core::iq::IqMode;

/// (name a user types, mode, the registry's name for it).
pub const MODES: &[(&str, IqMode, &str)] = &[
    ("FT8", IqMode::Ft8, "FT8"),
    ("FT4", IqMode::Ft4, "FT4"),
    ("FST4-15", IqMode::Fst4S15, "FST4-15"),
    ("FST4-30", IqMode::Fst4S30, "FST4-30"),
    ("FST4-60", IqMode::Fst4S60, "FST4-60A"),
    ("FST4-120", IqMode::Fst4S120, "FST4-120"),
    ("FST4-300", IqMode::Fst4S300, "FST4-300"),
    ("WSPR", IqMode::Wspr, "WSPR"),
    ("JT9", IqMode::Jt9, "JT9"),
    ("JT65", IqMode::Jt65, "JT65"),
    ("Q65-15A", IqMode::Q65A15, "Q65-15A"),
    ("Q65-30A", IqMode::Q65A30, "Q65-30A"),
    ("Q65-60A", IqMode::Q65A60, "Q65-60A"),
    ("Q65-60B", IqMode::Q65B60, "Q65-60B"),
    ("Q65-60C", IqMode::Q65C60, "Q65-60C"),
    ("Q65-60D", IqMode::Q65D60, "Q65-60D"),
    ("Q65-60E", IqMode::Q65E60, "Q65-60E"),
    ("Q65-120D", IqMode::Q65D120, "Q65-120D"),
    ("Q65-120E", IqMode::Q65E120, "Q65-120E"),
    ("Q65-300A", IqMode::Q65A300, "Q65-300A"),
];

/// Case-insensitive.
pub fn parse_mode(s: &str) -> Option<IqMode> {
    MODES
        .iter()
        .find(|(n, ..)| n.eq_ignore_ascii_case(s))
        .map(|&(_, m, _)| m)
}

pub fn mode_name(m: IqMode) -> &'static str {
    MODES
        .iter()
        .find(|&&(_, x, _)| x == m)
        .map(|&(n, ..)| n)
        .unwrap_or("?")
}

/// The mode's slot (T/R period) in seconds, as the registry has it; slots
/// start on multiples of it from 00:00 UTC.
pub fn slot_seconds(m: IqMode) -> f32 {
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
        assert_eq!(slot_seconds(IqMode::Ft8), 15.0);
        assert_eq!(slot_seconds(IqMode::Ft4), 7.5);
        assert_eq!(slot_seconds(IqMode::Wspr), 120.0);
    }
}
