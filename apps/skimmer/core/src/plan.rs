// SPDX-License-Identifier: GPL-3.0-or-later
//! Which IQ stream to ask the server for.

use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};

use crate::ChannelSpec;
use crate::spyserver::Device;

/// A stream and the channels it holds; the others are paused.
#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub decimation: u32,
    pub rate: u32,
    pub center_hz: f64,
    /// Indices into the configured channels.
    pub active: Vec<usize>,
}

/// The cheapest stream that holds the most channels. For each rate, lowest
/// first: the IQ centre nearest the wanted one (default 25 kHz below the
/// lowest dial, so DC is clear of every channel) that keeps the whole IQ
/// span inside the device's band, and the channels that fit there by the
/// receiver's own placement rule. With `tune` the device follows the IQ
/// centre, so there is no band to stay inside. `None` when nothing fits.
pub fn plan(
    channels: &[ChannelSpec],
    center_hz: Option<f64>,
    rate: Option<u32>,
    dev: &Device,
    device_hz: f64,
    tune: bool,
) -> Option<Plan> {
    let lowest = channels.iter().map(|c| c.dial_hz).fold(f64::MAX, f64::min);
    let wanted = center_hz.unwrap_or((lowest - 25_000.0).round());
    let mut best: Option<Plan> = None;
    for &(decimation, r) in &dev.rates {
        if rate.is_some_and(|want| want != r) {
            continue;
        }
        let center = if tune {
            wanted
        } else {
            let room = (dev.bandwidth_hz - r as f64) / 2.0;
            if room < 0.0 {
                continue;
            }
            wanted.clamp(device_hz - room, device_hz + room).round()
        };
        let active: Vec<usize> = (0..channels.len())
            .filter(|&i| fits(&channels[i], r, center))
            .collect();
        if best.as_ref().is_none_or(|b| active.len() > b.active.len()) {
            let all = active.len() == channels.len();
            best = Some(Plan {
                decimation,
                rate: r,
                center_hz: center,
                active,
            });
            if all {
                break;
            }
        }
    }
    best.filter(|p| !p.active.is_empty())
}

fn fits(c: &ChannelSpec, rate: u32, center_hz: f64) -> bool {
    IqReceiver::new(IqStream::new(rate, center_hz, IqSampleFormat::Cf32))
        .add_channel(c.dial_hz, c.mode)
        .is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use mfsk_core::Mode;

    /// The Airspy HF+ these were measured on.
    fn hf_plus() -> Device {
        Device {
            kind: 2,
            max_rate: 912_000,
            bandwidth_hz: 780_000.0,
            max_gain: 8,
            rates: (0..=6).rev().map(|d| (d, 912_000 >> d)).collect(),
        }
    }

    fn ft8(dial_hz: f64) -> ChannelSpec {
        ChannelSpec {
            mode: Mode::Ft8,
            dial_hz,
        }
    }

    /// What the live run chose beside SDR# at 7.1 MHz (2026-10-03):
    /// 114 kS/s centred 7016 kHz.
    #[test]
    fn one_channel_takes_the_lowest_rate_that_holds_it() {
        let p = plan(
            &[ft8(7_041_000.0)],
            None,
            None,
            &hf_plus(),
            7_100_000.0,
            false,
        )
        .unwrap();
        assert_eq!(
            (p.rate, p.center_hz, p.active.clone()),
            (114_000, 7_016_000.0, vec![0])
        );
    }

    /// A channel off the band is paused, not refused; the rest stream.
    #[test]
    fn a_channel_outside_the_band_is_paused() {
        let chs = [ft8(7_041_000.0), ft8(14_074_000.0)];
        let p = plan(&chs, None, None, &hf_plus(), 7_100_000.0, false).unwrap();
        assert_eq!(p.active, vec![0]);
    }

    /// The device tuned to 20 m by SDR#: the wanted centre (25 kHz under
    /// 7041 kHz) is clamped into the band, as the live run did (14044 kHz).
    #[test]
    fn the_centre_is_clamped_into_the_band() {
        let chs = [ft8(7_041_000.0), ft8(14_074_000.0)];
        let p = plan(&chs, None, None, &hf_plus(), 14_320_000.0, false).unwrap();
        assert_eq!(
            (p.rate, p.center_hz, p.active.clone()),
            (228_000, 14_044_000.0, vec![1])
        );
    }

    #[test]
    fn nothing_fits_is_none() {
        assert_eq!(
            plan(
                &[ft8(7_041_000.0)],
                None,
                None,
                &hf_plus(),
                21_000_000.0,
                false
            ),
            None
        );
    }

    /// With control and --tune the device moves to the IQ centre.
    #[test]
    fn tune_ignores_the_current_band() {
        let p = plan(
            &[ft8(7_041_000.0)],
            None,
            None,
            &hf_plus(),
            100_000_000.0,
            true,
        )
        .unwrap();
        assert_eq!(p.center_hz, 7_016_000.0);
    }
}
