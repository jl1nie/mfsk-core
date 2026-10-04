// SPDX-License-Identifier: GPL-3.0-only
//! Maidenhead locators to a point, and the bearing and distance between two.

const R_KM: f64 = 6371.0;

/// The centre of a 4- or 6-character locator, `(lat, lon)` in degrees.
pub fn grid_center(g: &str) -> Option<(f64, f64)> {
    let b = g.trim().as_bytes();
    if !(b.len() == 4 || b.len() == 6) {
        return None;
    }
    let up = |c: u8| c.to_ascii_uppercase();
    let (f0, f1) = (up(b[0]), up(b[1]));
    if !(b'A'..=b'R').contains(&f0) || !(b'A'..=b'R').contains(&f1) {
        return None;
    }
    if !b[2].is_ascii_digit() || !b[3].is_ascii_digit() {
        return None;
    }
    let mut lon = -180.0 + 20.0 * f64::from(f0 - b'A') + 2.0 * f64::from(b[2] - b'0');
    let mut lat = -90.0 + 10.0 * f64::from(f1 - b'A') + f64::from(b[3] - b'0');
    if b.len() == 6 {
        let (s0, s1) = (up(b[4]), up(b[5]));
        if !(b'A'..=b'X').contains(&s0) || !(b'A'..=b'X').contains(&s1) {
            return None;
        }
        lon += f64::from(s0 - b'A') * 5.0 / 60.0 + 2.5 / 60.0;
        lat += f64::from(s1 - b'A') * 2.5 / 60.0 + 1.25 / 60.0;
    } else {
        lon += 1.0;
        lat += 0.5;
    }
    Some((lat, lon))
}

/// Initial great-circle bearing (degrees from north, 0..360) and distance
/// (km) from `a` to `b`, both `(lat, lon)`.
pub fn bearing_distance(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (p1, p2) = (a.0.to_radians(), b.0.to_radians());
    let dl = (b.1 - a.1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    let bearing = y.atan2(x).to_degrees().rem_euclid(360.0);
    let h = ((p2 - p1) / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    (bearing, 2.0 * R_KM * h.sqrt().asin())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn locator_centres() {
        let (lat, lon) = grid_center("PM95").unwrap();
        assert!((lat - 35.5).abs() < 1e-9 && (lon - 139.0).abs() < 1e-9);
        let (lat, lon) = grid_center("PM95tl").unwrap();
        assert!((lat - 35.4792).abs() < 1e-3 && (lon - 139.625).abs() < 1e-3);
        assert!(
            grid_center("RR73").is_some(),
            "RR73 is a legal locator; the parser excludes it"
        );
        assert!(grid_center("ZZ99").is_none());
        assert!(grid_center("PM9").is_none());
    }

    /// Tokyo to Seattle is about 7 700 km, heading a little north of due east
    /// on the great circle (about 41 degrees); Tokyo to Sydney ~7 800 km at ~165.
    #[test]
    fn great_circle() {
        let tokyo = grid_center("PM95").unwrap();
        let (b, d) = bearing_distance(tokyo, grid_center("CN87").unwrap());
        assert!((7_400.0..7_900.0).contains(&d), "{d}");
        assert!((35.0..50.0).contains(&b), "{b}");
        let (b, d) = bearing_distance(tokyo, grid_center("QF56").unwrap());
        assert!((7_500.0..8_100.0).contains(&d), "{d}");
        assert!((160.0..175.0).contains(&b), "{b}");
    }
}
