// SPDX-License-Identifier: GPL-3.0-or-later
//! Whether this boot brings WiFi up, from the two things that bear on it — the
//! CONFIG page's `WIFI` row and its `TIME` row (#381).
//!
//! Pure, and in its own module, because the two types that carry those
//! settings (`wifi_pref::WifiPref`, `grid_src::GridSource`) sit beside NVS code
//! and only build for the ESP-IDF; the rule itself is a truth table and is
//! compiled and tested on the host (`hosttest/mfsk-app-shared`).
//!
//! **`TIME: AIR DT` turns WiFi off, whatever `WIFI` says.** AIR DT means "no
//! infrastructure here, take the phase off the band", and an association
//! campaign on a station with nothing to associate to costs the decoder ~40 %
//! of its throughput (`fst4_sync_search` 711 → 1 395 ms per candidate, measured
//! 2026-08-22) on exactly the first slots an air-placed grid needs. The
//! `WIFI` row is not lost: it is the choice under `TIME: NTP`, and stays as
//! stored for the day the grid source changes.
//!
//! This reverses an earlier decision. `wifi_pref` was made its own setting
//! *because* tying WiFi to the grid source was rejected (a hilltop with a
//! phone hotspot wants AIR DT **and** a log). The maintainer chose the tie on
//! 2026-09-30 anyway; the price is in [`WifiDecision::OffAirDt`].

/// What the boot does about WiFi, and why.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WifiDecision {
    /// Associate.
    On,
    /// The `WIFI: OFF` row.
    OffConfig,
    /// `TIME: AIR DT`. In `BootMode::Uac` the USB host driver has taken
    /// USB-Serial-JTAG, so with WiFi down there is **no console at all**: the
    /// UDP log and the HTTP config page go with it, and the panel is what is
    /// left.
    OffAirDt,
}

impl WifiDecision {
    pub fn enabled(self) -> bool {
        self == WifiDecision::On
    }

    /// For a log line: why there is no radio.
    pub fn reason(self) -> &'static str {
        match self {
            WifiDecision::On => "on",
            WifiDecision::OffConfig => "WIFI: OFF (CONFIG page)",
            WifiDecision::OffAirDt => {
                "TIME: AIR DT (no WiFi while the phase comes from the air; the WIFI row applies under TIME: NTP)"
            }
        }
    }
}

/// `pref_on`: the `WIFI` row is ON. `air_dt`: the `TIME` row is AIR DT. The
/// grid source wins, so the reason a log line gives is the one that is true
/// even if `WIFI` is also OFF.
pub fn decide(pref_on: bool, air_dt: bool) -> WifiDecision {
    if air_dt {
        WifiDecision::OffAirDt
    } else if !pref_on {
        WifiDecision::OffConfig
    } else {
        WifiDecision::On
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn air_dt_turns_wifi_off_whatever_the_wifi_row_says() {
        assert_eq!(decide(true, true), WifiDecision::OffAirDt);
        assert_eq!(decide(false, true), WifiDecision::OffAirDt);
        assert!(!decide(true, true).enabled());
    }

    #[test]
    fn under_ntp_the_wifi_row_decides() {
        assert_eq!(decide(true, false), WifiDecision::On);
        assert_eq!(decide(false, false), WifiDecision::OffConfig);
        assert!(decide(true, false).enabled());
        assert!(!decide(false, false).enabled());
    }

    #[test]
    fn only_on_is_enabled_and_every_off_says_why() {
        for (p, a) in [(true, true), (true, false), (false, true), (false, false)] {
            let d = decide(p, a);
            assert_eq!(d.enabled(), d == WifiDecision::On);
            assert!(!d.reason().is_empty());
        }
        assert!(decide(true, true).reason().contains("AIR DT"));
        assert!(decide(false, false).reason().contains("WIFI: OFF"));
    }
}
