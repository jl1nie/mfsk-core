// SPDX-License-Identifier: GPL-3.0-only
//! The names a user types for each `Mode`, one table both ways, and each
//! mode's slot length from the library's registry.

use mfsk_core::Mode;
use mfsk_core::decoder::{ApMode, Contest, QsoProgress};

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

/// What a user types for a JTTY channel (#650); JTTY is not a registry mode.
pub const JTTY_NAME: &str = "JTTY";

/// A channel's mode by its name, case-insensitive: a slotted mode or JTTY.
pub fn parse_channel_mode(s: &str) -> Option<crate::ChannelMode> {
    if s.eq_ignore_ascii_case(JTTY_NAME) {
        Some(crate::ChannelMode::Jtty)
    } else {
        parse_mode(s).map(crate::ChannelMode::Slot)
    }
}

pub fn channel_mode_name(m: crate::ChannelMode) -> &'static str {
    match m {
        crate::ChannelMode::Slot(m) => mode_name(m),
        crate::ChannelMode::Jtty => JTTY_NAME,
    }
}

/// A channel's slot in seconds; 0 for JTTY, which has none.
pub fn channel_slot_seconds(m: crate::ChannelMode) -> f32 {
    m.slot().map_or(0.0, slot_seconds)
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

/// Where a frame sits in its slot and on the band, from the registry:
/// (seconds from the slot start to the first symbol at dt = 0, frame length
/// in seconds, Hz from the reported frequency, the lowest tone, to the top of
/// the highest).
pub fn frame_geometry(m: Mode) -> (f32, f32, f32) {
    MODES
        .iter()
        .find(|&&(_, x, _)| x == m)
        .and_then(|&(.., r)| mfsk_core::registry::by_name(r))
        .map(|meta| {
            (
                meta.tx_start_offset_s,
                meta.n_symbols as f32 * meta.symbol_dt,
                meta.ntones as f32 * meta.tone_spacing_hz,
            )
        })
        .unwrap_or((0.5, 0.0, 50.0))
}

/// `fast`, `normal` or `deep` (any case); WSJT-X's `ndepth` 1, 2, 3.
pub fn parse_depth(s: &str) -> Option<mfsk_core::decoder::Depth> {
    use mfsk_core::decoder::Depth;
    match s.to_ascii_lowercase().as_str() {
        "fast" | "1" => Some(Depth::Fast),
        "normal" | "2" => Some(Depth::Normal),
        "deep" | "3" => Some(Depth::Deep),
        _ => None,
    }
}

fn num(k: &str, v: &str) -> Result<f32, String> {
    v.parse().map_err(|_| format!("bad {k} {v:?}"))
}

fn flag(k: &str, v: &str) -> Result<bool, String> {
    match v {
        "1" | "on" | "true" => Ok(true),
        "0" | "off" | "false" => Ok(false),
        _ => Err(format!("bad {k} {v:?}, expected 0|1")),
    }
}

/// `nQSOProgress`, 0 to 5, or its name.
pub fn parse_progress(s: &str) -> Option<QsoProgress> {
    Some(match s.to_ascii_lowercase().as_str() {
        "0" | "calling" => QsoProgress::Calling,
        "1" | "replying" => QsoProgress::Replying,
        "2" | "report" => QsoProgress::Report,
        "3" | "rogerreport" => QsoProgress::RogerReport,
        "4" | "rogers" => QsoProgress::Rogers,
        "5" | "signoff" => QsoProgress::Signoff,
        _ => return None,
    })
}

/// `ncontest`'s activities by name.
pub fn parse_contest(s: &str) -> Option<Contest> {
    Some(match s.to_ascii_lowercase().as_str() {
        "none" | "" => Contest::None,
        "grid" | "gridexchange" => Contest::GridExchange,
        "euvhf" => Contest::EuVhf,
        "fieldday" => Contest::FieldDay,
        "rtty" | "rttyroundup" => Contest::RttyRoundup,
        "fox" => Contest::Fox,
        "hound" => Contest::Hound,
        _ => return None,
    })
}

/// A channel written `MODE@DIAL_HZ` followed by any of `:band=LO-HI` (audio
/// Hz), `:dx=CALL`, `:depth=fast|normal|deep`, `:rx=HZ`, `:tol=HZ`, `:tx=HZ`,
/// `:ap=off|cq|full`, `:hiscall=`, `:hisgrid=`, `:progress=0..5`, `:contest=NAME`,
/// `:avg=1`, `:deepsearch=1` and `:eme=1` (WSJT-X's parameter block).
pub fn parse_channel(spec: &str) -> Result<crate::ChannelSpec, String> {
    let mut parts = spec.split(':');
    let head = parts.next().unwrap_or("");
    let (m, f) = head
        .split_once('@')
        .ok_or_else(|| format!("{spec:?}: expected MODE@DIAL_HZ"))?;
    let mode = parse_channel_mode(m).ok_or_else(|| format!("unknown mode {m:?}"))?;
    let dial: f64 = f.parse().map_err(|_| format!("bad dial frequency {f:?}"))?;
    let mut ch = crate::ChannelSpec::new(mode, dial);
    for opt in parts {
        let (k, v) = opt
            .split_once('=')
            .ok_or_else(|| format!("{opt:?}: expected key=value"))?;
        match k {
            "band" => {
                let (lo, hi) = v
                    .split_once('-')
                    .and_then(|(a, b)| Some((a.parse().ok()?, b.parse().ok()?)))
                    .filter(|(a, b): &(f32, f32)| a < b)
                    .ok_or_else(|| format!("bad band {v:?}, expected LO-HI"))?;
                ch.options.band_hz = Some((lo, hi));
            }
            "dx" => ch.options.dx_call = Some(v.to_ascii_uppercase()),
            "depth" => {
                ch.options.depth = Some(parse_depth(v).ok_or_else(|| format!("bad depth {v:?}"))?);
            }
            "rx" => ch.options.rx_freq_hz = Some(num(k, v)?),
            "tol" => ch.options.tol_hz = Some(num(k, v)?),
            "tx" => ch.options.tx_freq_hz = Some(num(k, v)?),
            "ap" => {
                ch.options.ap = Some(match v {
                    "off" => ApMode::Off,
                    "cq" => ApMode::CqOnly,
                    "full" => ApMode::Full,
                    _ => return Err(format!("bad ap {v:?}, expected off|cq|full")),
                });
            }
            "hiscall" => ch.options.qso.his_call = v.to_ascii_uppercase(),
            "hisgrid" => ch.options.qso.his_grid = v.to_ascii_uppercase(),
            "progress" => {
                ch.options.qso.progress =
                    parse_progress(v).ok_or_else(|| format!("bad progress {v:?}, expected 0-5"))?;
            }
            "contest" => {
                ch.options.contest =
                    parse_contest(v).ok_or_else(|| format!("bad contest {v:?}"))?;
            }
            "avg" => ch.options.averaging = flag(k, v)?,
            "deepsearch" => ch.options.deep_search = flag(k, v)?,
            "eme" => ch.options.eme_delay = flag(k, v)?,
            _ => return Err(format!("unknown option {k:?}")),
        }
    }
    Ok(ch)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// JTTY is a channel mode of its own (#650): not a registry mode, no slot.
    #[test]
    fn jtty_is_a_channel_with_no_slot() {
        let c = parse_channel("jtty@14090000:rx=1700:tol=80").unwrap();
        assert_eq!(c.mode, crate::ChannelMode::Jtty);
        assert_eq!(c.options.rx_freq_hz, Some(1700.0));
        assert_eq!(c.options.tol_hz, Some(80.0));
        assert_eq!(channel_mode_name(c.mode), "JTTY");
        assert_eq!(channel_slot_seconds(c.mode), 0.0);
        assert!(parse_mode("JTTY").is_none(), "not a slotted mode");
    }

    #[test]
    fn channel_specs_parse_their_options() {
        let c = parse_channel("ft8@7074000:band=300-3000:dx=ja1abc:depth=fast").unwrap();
        assert_eq!(c.mode, crate::ChannelMode::Slot(Mode::Ft8));
        assert_eq!(c.dial_hz, 7_074_000.0);
        assert_eq!(c.options.band_hz, Some((300.0, 3000.0)));
        assert_eq!(c.options.dx_call.as_deref(), Some("JA1ABC"));
        assert_eq!(c.options.depth, Some(mfsk_core::decoder::Depth::Fast));
        assert_eq!(
            parse_channel("FT8@7074000").unwrap(),
            crate::ChannelSpec::new(Mode::Ft8, 7_074_000.0)
        );
        let d = parse_channel(
            "FT8@7074000:rx=1500:tol=20:ap=cq:hiscall=ja1abc:progress=2:contest=fieldday:avg=1",
        )
        .unwrap();
        assert_eq!(d.options.rx_freq_hz, Some(1500.0));
        assert_eq!(d.options.ap, Some(ApMode::CqOnly));
        assert_eq!(d.options.qso.his_call, "JA1ABC");
        assert_eq!(d.options.qso.progress, QsoProgress::Report);
        assert_eq!(d.options.contest, Contest::FieldDay);
        assert!(d.options.averaging);
        for bad in [
            "FT8@1:ap=maybe",
            "FT8@1:progress=9",
            "FT8@1:avg=2",
            "FT8",
            "FT9@1",
            "FT8@x",
            "FT8@1:band=5-3",
            "FT8@1:depth=max",
            "FT8@1:foo=1",
            "FT8@1:dx",
        ] {
            assert!(parse_channel(bad).is_err(), "{bad}");
        }
    }

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
