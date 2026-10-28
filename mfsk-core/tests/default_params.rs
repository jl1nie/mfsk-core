//! `decoder::default_params(mode)` is the one source of a mode's defaults
//! (#698: the registry's `DecodeDefaults` went, having been read by nothing
//! but this function's fallback band).
//!
//! Two checks, against two different failures. `the_band_is_each_modes_own`
//! ties the scan modes' band to their own search default, so a hand copy
//! cannot drift from it. `default_params_did_not_move` pins every mode's
//! values as they were on `main` before the registry defaults were removed
//! (df956b45), so the move itself is shown to have changed nothing.
#![cfg(all(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    feature = "fst4w",
    feature = "wspr",
    feature = "jt9",
    feature = "jt65",
    feature = "q65",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::Mode;
use mfsk_core::decoder::{ApMode, DecodeParams, Decoder, Depth, default_params};
use mfsk_core::ft8::Ft8;

/// Fails if a scan mode's default band is copied by hand and drifts from the
/// search default its decoder was written against.
#[test]
fn the_band_is_each_modes_own() {
    let band = |p: mfsk_core::engine::search::SearchParams| (p.freq_min_hz, p.freq_max_hz);
    for &m in Mode::ALL {
        let want = match m.name() {
            "WSPR" => band(mfsk_core::wspr::search::default_search_params()),
            "JT9" => band(mfsk_core::jt9::search::default_search_params()),
            "JT65" => band(mfsk_core::jt65::search::default_search_params()),
            n if n.starts_with("Q65-") => band(mfsk_core::q65::search::default_search_params()),
            _ => continue,
        };
        assert_eq!(default_params(m).band_hz, want, "{}", m.name());
    }
}

/// Fails if any mode's default band, depth or AP mode moved. The values are
/// what `default_params` returned on df956b45, with the provenance
/// `default_params`' doc gives.
#[test]
fn default_params_did_not_move() {
    let expected: &[(&str, (f32, f32), ApMode)] = &[
        ("FT8", (200.0, 4000.0), ApMode::Off),
        ("FT4", (200.0, 4000.0), ApMode::Full),
        ("FST4-15", (600.0, 1400.0), ApMode::Full),
        ("FST4-30", (600.0, 1400.0), ApMode::Full),
        ("FST4-60A", (600.0, 1400.0), ApMode::Full),
        ("FST4-120", (600.0, 1400.0), ApMode::Full),
        ("FST4-300", (600.0, 1400.0), ApMode::Full),
        ("FST4-900", (600.0, 1400.0), ApMode::Full),
        ("FST4-1800", (600.0, 1400.0), ApMode::Full),
        ("FST4W-120", (1400.0, 1600.0), ApMode::Full),
        ("FST4W-300", (1400.0, 1600.0), ApMode::Full),
        ("FST4W-900", (1400.0, 1600.0), ApMode::Full),
        ("FST4W-1800", (1400.0, 1600.0), ApMode::Full),
        ("WSPR", (1400.0, 1600.0), ApMode::Full),
        ("JT9", (200.0, 4000.0), ApMode::Full),
        ("JT65", (1000.0, 2000.0), ApMode::Off),
        ("Q65-15A", (200.0, 3000.0), ApMode::Full),
        ("Q65-30A", (200.0, 3000.0), ApMode::Full),
        ("Q65-60A", (200.0, 3000.0), ApMode::Full),
        ("Q65-60B", (200.0, 3000.0), ApMode::Full),
        ("Q65-60C", (200.0, 3000.0), ApMode::Full),
        ("Q65-60D", (200.0, 3000.0), ApMode::Full),
        ("Q65-60E", (200.0, 3000.0), ApMode::Full),
        ("Q65-120D", (200.0, 3000.0), ApMode::Full),
        ("Q65-120E", (200.0, 3000.0), ApMode::Full),
        ("Q65-300A", (200.0, 3000.0), ApMode::Full),
    ];
    assert_eq!(expected.len(), Mode::ALL.len(), "every mode is pinned");
    for &(name, band, ap) in expected {
        let m = Mode::from_name(name).unwrap_or_else(|| panic!("{name} not in this build"));
        let p = default_params(m);
        assert_eq!(p.band_hz, band, "{name}: band");
        assert_eq!(p.depth, Depth::Deep, "{name}: depth");
        assert_eq!(p.ap, ap, "{name}: ap");
    }
}

/// The documented way to change only the band, `default_params(mode).band(lo, hi)`,
/// keeps every other default of the mode; `for_band` is the bare block and does
/// not (#711). Fails if `band` starts touching another field, or if `for_band`
/// becomes a mode's defaults and the docs' distinction goes stale.
#[test]
fn band_keeps_the_modes_other_defaults_and_for_band_does_not() {
    for &m in Mode::ALL {
        let mut want = default_params(m);
        want.band_hz = (300.0, 2_700.0);
        assert_eq!(
            default_params(m).band(300.0, 2_700.0),
            want,
            "{}: band() changed more than the band",
            m.name()
        );
    }
    for m in [Mode::Ft8, Mode::Jt65] {
        let bare = DecodeParams::for_band(default_params(m).band_hz);
        assert_eq!(bare.ap, ApMode::Full, "{}", m.name());
        assert_ne!(
            bare,
            default_params(m),
            "{}: for_band is not the mode's defaults",
            m.name()
        );
    }
    // A decoder's own defaults are `default_params`, edited in place the same way.
    let mut d = Decoder::<Ft8>::with_defaults();
    d.params_mut().band_hz = (300.0, 2_700.0);
    assert_eq!(*d.params(), default_params(Mode::Ft8).band(300.0, 2_700.0));
}
