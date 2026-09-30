//! Dial-frequency presets for the CONFIG > FREQ page, one table per
//! receiver, and the rig frequency the operator last applied.
//!
//! Only the running receiver's table is shown: an FT8 board has no use
//! for an FT4 dial, and one mode's list already needs paging on a
//! four-row widget ([`page`]).
//!
//! The FT8 table carries the JA channels beside the IARU ones — the list
//! the operator confirmed on 2026-09-23 (the IC-705 this board runs
//! against sat on 7.041 for every live session). They are labelled
//! `JA` so an operator abroad can tell them from the IARU channel on
//! the same band. The IARU FT8 channels and the whole FT4 table are
//! the ones WSJT-X itself lists (`widgets/mainwindow.cpp:5528`, the
//! `Freq` vector its combined-message check compares against). There was a
//! WSPR table until the CoreS3 WSPR receiver was removed (2026-09-30). FST4
//! has no table yet: WSJT-X's defaults for it are LF/MF, which the
//! IC-705 does not transmit on, and no HF channel has been agreed.

/// One row of a table: what the page draws, and the dial it sets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FreqPreset {
    pub label: &'static str,
    pub hz: u32,
}

const fn p(label: &'static str, hz: u32) -> FreqPreset {
    FreqPreset { label, hz }
}

pub const FT8: &[FreqPreset] = &[
    p("160m JA  1.908", 1_908_000),
    p("80m JA   3.531", 3_531_000),
    p("80m      3.573", 3_573_000),
    p("40m JA   7.041", 7_041_000),
    p("40m      7.074", 7_074_000),
    p("30m     10.136", 10_136_000),
    p("20m     14.074", 14_074_000),
    p("17m     18.100", 18_100_000),
    p("15m     21.074", 21_074_000),
    p("12m     24.915", 24_915_000),
    p("10m     28.074", 28_074_000),
    p("6m      50.313", 50_313_000),
    p("2m JA  144.460", 144_460_000),
];

pub const FT4: &[FreqPreset] = &[
    p("80m      3.575", 3_575_000),
    p("40m      7.0475", 7_047_500),
    p("30m     10.140", 10_140_000),
    p("20m     14.080", 14_080_000),
    p("17m     18.104", 18_104_000),
    p("15m     21.140", 21_140_000),
    p("12m     24.919", 24_919_000),
    p("10m     28.180", 28_180_000),
    p("6m      50.318", 50_318_000),
];

pub const FST4: &[FreqPreset] = &[];

/// Which table a receiver shows, by the name the picker gives it
/// (`mode_picker::mode_name`). Empty for anything without one.
pub fn for_mode(name: &str) -> &'static [FreqPreset] {
    match name {
        "FT8" => FT8,
        "FT4" => FT4,
        "FST4" => FST4,
        _ => &[],
    }
}

/// One screenful of a table on a widget with `rows` bands.
///
/// A table that fits is shown whole. One that does not gives the last
/// band to a NEXT row and shows `rows - 1` presets per page; NEXT past
/// the last page wraps to the first, so there is no BACK to aim for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Page {
    /// Index of the first preset on this page.
    pub start: usize,
    /// Presets on this page.
    pub count: usize,
    /// Whether the row after them is NEXT.
    pub has_next: bool,
    /// 1-based, for the header.
    pub number: usize,
    pub of: usize,
}

pub fn page(len: usize, rows: usize, index: usize) -> Page {
    if len <= rows {
        return Page {
            start: 0,
            count: len,
            has_next: false,
            number: 1,
            of: 1,
        };
    }
    let per = rows - 1;
    let of = len.div_ceil(per);
    let index = index % of;
    let start = index * per;
    Page {
        start,
        count: per.min(len - start),
        has_next: true,
        number: index + 1,
        of,
    }
}

#[cfg(target_os = "espidf")]
const NVS_KEY: &str = "rig_hz";

/// The dial the operator last applied, re-sent to the rig at boot.
#[cfg(target_os = "espidf")]
pub fn read(nvs: &esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>) -> Option<u32> {
    nvs.get_u32(NVS_KEY).ok().flatten()
}

#[cfg(target_os = "espidf")]
pub fn write(
    nvs: &esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>,
    hz: u32,
) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_u32(NVS_KEY, hz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_table_that_fits_has_no_next() {
        assert_eq!(
            page(4, 4, 0),
            Page {
                start: 0,
                count: 4,
                has_next: false,
                number: 1,
                of: 1
            }
        );
    }

    #[test]
    fn ft8_pages_three_at_a_time_and_wraps() {
        let n = FT8.len();
        let pages = n.div_ceil(3);
        let mut seen = 0;
        for i in 0..pages {
            let pg = page(n, 4, i);
            assert!(pg.has_next);
            assert_eq!(pg.start, seen);
            seen += pg.count;
        }
        assert_eq!(seen, n);
        assert_eq!(page(n, 4, pages), page(n, 4, 0));
    }

    #[test]
    fn labels_fit_the_widget_and_frequencies_ascend() {
        // 208 px of FONT_6X10 less the 10 px indent and the `*` column.
        for t in [FT8, FT4, FST4] {
            for f in t {
                assert!(f.label.len() <= 30, "{} too long", f.label);
            }
            for w in t.windows(2) {
                assert!(w[0].hz < w[1].hz, "{} !< {}", w[0].label, w[1].label);
            }
        }
    }
}
