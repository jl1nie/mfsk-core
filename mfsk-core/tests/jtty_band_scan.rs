// SPDX-License-Identifier: GPL-3.0-only
//! What a band scan beside channel 0 finds and costs (#499): single stations across the audio
//! band at several SNRs and stretches of noise, made by `jtty::testsig`, through
//! `Params::embedded()` with each `SideChannels` choice. Prints; not an assertion suite.
//!
//! ```text
//! cargo test -p mfsk-core --release --features full,jtty-stats --test jtty_band_scan -- --ignored --nocapture
//! ```

#![cfg(all(
    feature = "jtty-stats",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::jtty::rx::{Params, Receiver, SideChannels};
use mfsk_core::jtty::stats::Counter as C;
use mfsk_core::jtty::testsig::{Station, render};

#[test]
#[ignore = "prints"]
fn band_scan_recall_and_cost() {
    let embedded = Params::default().embedded();
    let configs: [(&str, Params); 5] = [
        ("channel 0 only", embedded),
        (
            "upstream side channels 1350/1650 +-150",
            Params {
                ch0_only: false,
                side_channels: SideChannels::Upstream,
                ..embedded
            },
        ),
        (
            "band 200-2800, 200 Hz channels",
            Params {
                ch0_only: false,
                side_channels: SideChannels::Band {
                    lo_hz: 200.0,
                    hi_hz: 2800.0,
                    width_hz: 200.0,
                    picks: 2,
                },
                ..embedded
            },
        ),
        (
            "band 200-2800, budget 2",
            Params {
                ch0_only: false,
                ladder_budget: Some(2),
                side_channels: SideChannels::Band {
                    lo_hz: 200.0,
                    hi_hz: 2800.0,
                    width_hz: 200.0,
                    picks: 2,
                },
                ..embedded
            },
        ),
        (
            "band 800-2200, 200 Hz channels",
            Params {
                ch0_only: false,
                side_channels: SideChannels::Band {
                    lo_hz: 800.0,
                    hi_hz: 2200.0,
                    width_hz: 200.0,
                    picks: 2,
                },
                ..embedded
            },
        ),
    ];
    // stations: every 100 Hz from 300 to 2700 (+ a few Hz so they sit off the grid)
    let mut cases = Vec::new();
    for (i, f) in (300..=2700).step_by(100).enumerate() {
        for snr in [-14.0f32, -10.0, -6.0, -2.0] {
            for trial in 0..3u64 {
                cases.push((
                    f as f32 + 7.3 * trial as f32,
                    snr,
                    (i as u64) << 8 | trial << 4 | (snr.abs() as u64),
                ));
            }
        }
    }
    let rx = Receiver::new().with_f32_metrics();
    for (name, params) in configs {
        rx.reset_stats();
        // region: 0 = channel 0 (1450..1550), 1 = 1200..1800, 2 = elsewhere
        let mut found = [[0u32; 4]; 3];
        let mut total = [[0u32; 4]; 3];
        let mut unexpected = 0;
        for &(f, snr, seed) in &cases {
            let st = Station {
                text: "CQ K1ABC CQ",
                f0_hz: f,
                start_s: 1.0,
                snr_db: snr,
                drift_hz_s: 0.0,
                fading_hz: 0.0,
            };
            let audio = render(&[st], 6.0, seed).unwrap();
            let got = rx.scan(&audio, &params);
            let region = if (1450.0..=1550.0).contains(&f) {
                0
            } else if (1200.0..=1800.0).contains(&f) {
                1
            } else {
                2
            };
            let s = [-14.0, -10.0, -6.0, -2.0]
                .iter()
                .position(|&x| x == snr)
                .unwrap();
            total[region][s] += 1;
            found[region][s] += u32::from(got.iter().any(|d| d.atom.render() == "CQ K1ABC CQ"));
            unexpected += got
                .iter()
                .filter(|d| d.atom.render() != "CQ K1ABC CQ")
                .count();
        }
        let sig = rx.stats();
        let per = |c: C| sig.count(c) as f64 / sig.count(C::Windows).max(1) as f64;
        let (lc, gp, pk) = (
            per(C::LadderCalls),
            per(C::GatePass),
            per(C::PicksOther) + per(C::PicksCh0),
        );
        // noise alone
        rx.reset_stats();
        let mut noise_unexpected = 0;
        for seed in 0..10u64 {
            let audio = render(&[], 30.0, 0xA0 + seed).unwrap();
            noise_unexpected += rx.scan(&audio, &params).len();
        }
        let n = rx.stats();
        let nper = |c: C| n.count(c) as f64 / n.count(C::Windows).max(1) as f64;
        println!("\n== {name}");
        for (r, label) in ["channel 0", "1200-1800 Hz", "elsewhere"]
            .iter()
            .enumerate()
        {
            let cells: Vec<String> = (0..4)
                .map(|s| format!("{}/{}", found[r][s], total[r][s]))
                .collect();
            println!("  {label:<13} -14/-10/-6/-2 dB: {}", cells.join("  "));
        }
        println!(
            "  unexpected: {unexpected} with a station, {noise_unexpected} in 300 s of noise; per window with a station: picks {pk:.1} gate passes {gp:.2} ladder {lc:.3}; noise: gate passes {:.3} ladder {:.3}",
            nper(C::GatePass),
            nper(C::LadderCalls)
        );
    }
}
