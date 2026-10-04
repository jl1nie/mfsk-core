// SPDX-License-Identifier: GPL-3.0-only
//! Where the Sun is, and when it rises and sets at a place: enough for "day" and
//! "night" bands of a rotation that follow the calendar. The low-precision almanac
//! formulas (a minute of arc for the position, a few minutes for the times), the
//! same as the GUI's (`gui/src/lib/analysis.ts`), which draws the grey line and shows
//! the times.

/// Where the Sun is overhead at `t_ms` (UTC ms): `(longitude, latitude)`, degrees.
pub fn subsolar(t_ms: f64) -> (f64, f64) {
    let rad = std::f64::consts::PI / 180.0;
    let d = t_ms / 86_400_000.0 + 2_440_587.5 - 2_451_545.0; // days since J2000
    let g = (357.529 + 0.98560028 * d) * rad;
    let q = 280.459 + 0.98564736 * d;
    let l = (q + 1.915 * g.sin() + 0.02 * (2.0 * g).sin()) * rad;
    let e = (23.439 - 0.00000036 * d) * rad;
    let dec = (e.sin() * l.sin()).asin() / rad;
    let ra = (e.cos() * l.sin()).atan2(l.cos()) / rad;
    let gmst = (18.697374558 + 24.06570982441908 * d) * 15.0;
    let mut lon = (ra - gmst) % 360.0;
    if lon > 180.0 {
        lon -= 360.0;
    }
    if lon < -180.0 {
        lon += 360.0;
    }
    (lon, dec)
}

/// One solar day at a place.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SolarDay {
    /// The Sun rises and sets: the UTC seconds of each (sunrise before sunset;
    /// either can fall on the neighbouring UTC date).
    Normal {
        rise_s: i64,
        set_s: i64,
    },
    /// It does not set (polar day) / does not rise (polar night).
    PolarDay,
    PolarNight,
}

/// The solar day whose noon falls on the UTC date starting at `day_start_s`, at
/// `(lon, lat)` degrees; the Sun's upper limb on the horizon, with refraction (-0.833).
pub fn solar_day(lon: f64, lat: f64, day_start_s: i64) -> SolarDay {
    let rad = std::f64::consts::PI / 180.0;
    // Declination and the equation of time at noon UTC of the date; the subsolar
    // longitude is minus the equation of time.
    let (sun_lon, dec) = subsolar((day_start_s * 1000 + 12 * 3_600_000) as f64);
    let eot_min = -sun_lon * 4.0;
    let arg = ((-0.833 * rad).sin() - (lat * rad).sin() * (dec * rad).sin())
        / ((lat * rad).cos() * (dec * rad).cos());
    if arg > 1.0 {
        return SolarDay::PolarNight;
    }
    if arg < -1.0 {
        return SolarDay::PolarDay;
    }
    let h0 = arg.acos() / rad; // degrees of hour angle
    let noon_s = day_start_s + ((720.0 - 4.0 * lon - eot_min) * 60.0).round() as i64;
    let half = (4.0 * h0 * 60.0).round() as i64;
    SolarDay::Normal {
        rise_s: noon_s - half,
        set_s: noon_s + half,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DAY: i64 = 1_790_985_600; // 2026-10-03 00:00 UTC

    /// The same values as the GUI's `sunTimes` for PM95 (139 E, 35.5 N) on 2026-10-04:
    /// sunrise 20:41 (on the 3rd, UTC) and sunset 08:24 UTC.
    #[test]
    fn tokyo_on_the_4th_of_october() {
        let SolarDay::Normal { rise_s, set_s } = solar_day(139.0, 35.5, DAY + 86_400) else {
            panic!("the Sun rises and sets")
        };
        let hm = |s: i64| (s.rem_euclid(86_400) / 3600, s.rem_euclid(86_400) / 60 % 60);
        assert_eq!(hm(rise_s), (20, 41));
        assert_eq!(hm(set_s), (8, 24));
        assert!(
            rise_s < DAY + 86_400,
            "the sunrise is the evening before in UTC"
        );
    }

    #[test]
    fn the_poles() {
        // Tromso (JP99): midsummer day and midwinter night, as the GUI says.
        let june = 1_782_000_000 / 86_400 * 86_400; // 2026-06-21
        let dec = 1_797_800_000 / 86_400 * 86_400; // 2026-12-21
        assert_eq!(solar_day(18.0, 69.5, june), SolarDay::PolarDay);
        assert_eq!(solar_day(18.0, 69.5, dec), SolarDay::PolarNight);
    }

    #[test]
    fn the_subsolar_point_as_the_gui_has_it() {
        let (lon, lat) = subsolar(1_790_985_600_000.0 + 86_400_000.0 + 12.0 * 3_600_000.0); // 2026-10-04 12:00 UTC
        assert!(
            (lon - -2.83).abs() < 0.02 && (lat - -4.46).abs() < 0.02,
            "{lon} {lat}"
        );
    }
}
