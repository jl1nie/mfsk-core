//! WSPR band plan — the standard per-band dial (USB) frequencies.
//!
//! Nothing in `mfsk-core::wspr` models a band plan at all (its only
//! frequency constant, `wspr::baseband::CENTER_HZ`, is the fixed
//! 1500 Hz audio IF the decoder itself runs at — unrelated to RF).
//! This table is operational/regulatory metadata, not signal
//! processing, so it lives here in the controller layer rather than
//! growing `mfsk-core`'s protocol-glue surface.
//!
//! **Verification status**: these are the widely-published WSPR
//! segment dial frequencies (the same ones listed on wsprnet.org and
//! used consistently across the WSPR community). An attempt to
//! cross-check them against the in-tree WSJT-X source
//! (`/home/minoru/src/WSJT-X`) found no static, greppable default-
//! frequency table there — WSJT-X's own `models/FrequencyList.cpp`
//! has no literal matches for any of these values, so its defaults
//! most likely live in a runtime-populated resource this search
//! couldn't reach, not a compiled-in constant. Treat these as
//! standard-band-plan values pending an authoritative WSJT-X
//! cross-check, not as independently re-derived from that source the
//! way most of this crate's protocol constants are.

/// One WSPR band: a display label and its dial (USB carrier)
/// frequency in MHz. The actual WSPR signal occupies dial_mhz + a
/// 1400-1600 Hz audio passband (`wspr::baseband::CENTER_HZ` sits at
/// the middle of that range).
pub struct WsprBand {
    pub label: &'static str,
    pub dial_mhz: f64,
}

/// Standard WSPR band segments, ordered low to high. Index into this
/// slice is what `settings::Settings::band_idx` stores — keep the
/// order stable across releases, since a stored index effectively
/// pins a band across a firmware update.
pub const WSPR_BANDS: &[WsprBand] = &[
    WsprBand {
        label: "160m",
        dial_mhz: 1.836_600,
    },
    WsprBand {
        label: "80m",
        dial_mhz: 3.568_600,
    },
    WsprBand {
        label: "60m",
        dial_mhz: 5.364_700,
    },
    WsprBand {
        label: "40m",
        dial_mhz: 7.038_600,
    },
    WsprBand {
        label: "30m",
        dial_mhz: 10.138_700,
    },
    WsprBand {
        label: "20m",
        dial_mhz: 14.095_600,
    },
    WsprBand {
        label: "17m",
        dial_mhz: 18.104_600,
    },
    WsprBand {
        label: "15m",
        dial_mhz: 21.094_600,
    },
    WsprBand {
        label: "12m",
        dial_mhz: 24.924_600,
    },
    WsprBand {
        label: "10m",
        dial_mhz: 28.124_600,
    },
    WsprBand {
        label: "6m",
        dial_mhz: 50.293_000,
    },
    WsprBand {
        label: "2m",
        dial_mhz: 144.489_000,
    },
];

/// Default band index into [`WSPR_BANDS`] — 20m, the band the
/// existing golden WAV/baseband fixture
/// (`assets/wspr_golden_baseband.bin`, `150426_0918.wav`) was
/// recorded on. Also matches `ui::state::UiState::current_band_idx`'s
/// own default choice of 20m for FT8 (a different band-index table
/// entirely — not reused, just precedent for picking 20m as the
/// sensible default).
pub const DEFAULT_BAND_IDX: u8 = 5;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_band_idx_is_in_range_and_is_20m() {
        let band = &WSPR_BANDS[DEFAULT_BAND_IDX as usize];
        assert_eq!(band.label, "20m");
    }

    #[test]
    fn bands_are_ordered_low_to_high() {
        for pair in WSPR_BANDS.windows(2) {
            assert!(
                pair[0].dial_mhz < pair[1].dial_mhz,
                "{} should be below {}",
                pair[0].label,
                pair[1].label
            );
        }
    }
}
