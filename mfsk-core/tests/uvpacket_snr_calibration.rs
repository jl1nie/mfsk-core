// SPDX-License-Identifier: GPL-3.0-or-later
//! Calibrate / verify the per-mode SNR_2500Hz reporting.
//!
//! For each mode:
//! 1. Encode a test frame at known Eb/N0_info.
//! 2. Compute the truth SNR_2500Hz from Eb/N0:
//!    truth_snr = Eb/N0 + 10·log10(R_info / 2500)
//! 3. Decode and read back `frame.snr_db`.
//! 4. Report the residual (estimate − truth) per Eb/N0 step.
//!
//! Run with `cargo test --release --features uvpacket --test
//! uvpacket_snr_calibration -- --ignored --nocapture`. The CSV
//! output shows whether `SNR_CALIBRATION_DB[mode]` needs adjustment.

#![cfg(feature = "uvpacket")]

use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, rx, tx};

// `common` is shared scaffolding included by 43 test binaries, each
// using a different subset — dead-code analysis of it is per-binary
// noise. Every other consumer already carries this attribute; these
// six only compiled without it because `common`'s own `#[cfg(test)]`
// blocks kept the helpers "used" until they moved to
// `tests/common_selftest.rs`.
#[allow(dead_code)]
mod common;
use common::channel::{AwgnChannel, signal_power};
use common::uvpacket_channel::awgn_sigma_for_eb_n0_info;

const N_BLOCKS: u8 = 4;
const N_TRIALS: usize = 20;
const PAYLOAD: usize = 16;
const ALL_MODES: [Mode; 4] = [
    Mode::UltraRobust,
    Mode::Robust,
    Mode::Standard,
    Mode::Express,
];

fn r_info_bps(mode: Mode) -> f32 {
    // 2 bit/sym × baud × FEC rate. Robust/Std/Express @ 1200 baud,
    // UltraRobust @ 600 baud. All four use the puncture rates from
    // the FEC table (UR/Robust 0.42, Std 0.50, Express 0.75).
    let baud = match mode {
        Mode::UltraRobust => 600.0,
        _ => 1200.0,
    };
    let rate = match mode {
        Mode::UltraRobust | Mode::Robust => 0.42,
        Mode::Standard => 0.50,
        Mode::Express => 0.75,
    };
    2.0 * baud * rate
}

fn truth_snr_2500(mode: Mode, eb_n0_db: f32) -> f32 {
    eb_n0_db + 10.0 * (r_info_bps(mode) / 2500.0).log10()
}

#[test]
#[ignore = "calibration: prints residuals, doesn't assert"]
fn snr_calibration_residuals() {
    eprintln!("mode,eb_n0_db,truth_snr_2500,reported_snr_2500,residual,n_decoded");
    for mode in ALL_MODES {
        // Sweep above-threshold range where the high-SNR approximation
        // holds and the LDPC decoder reliably succeeds.
        let eb_n0_grid: &[f32] = match mode {
            Mode::UltraRobust => &[6.0, 8.0, 10.0, 14.0, 18.0],
            Mode::Robust => &[8.0, 10.0, 14.0, 18.0],
            Mode::Standard => &[8.0, 10.0, 14.0, 18.0],
            Mode::Express => &[10.0, 14.0, 18.0, 22.0],
        };
        for &eb_n0 in eb_n0_grid {
            let truth = truth_snr_2500(mode, eb_n0);
            let mut decoded = 0;
            let mut sum_snr = 0.0_f32;
            for trial in 0..N_TRIALS {
                let header = FrameHeader {
                    mode,
                    block_count: N_BLOCKS,
                    app_type: 0,
                    sequence: 0,
                };
                let payload: Vec<u8> = (0..PAYLOAD).map(|i| ((i + trial) ^ 0xA5) as u8).collect();
                let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
                let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&audio));
                AwgnChannel::new(sigma, 0xCAFE_F000 + trial as u64).apply(&mut audio);
                if let Ok(f) =
                    rx::decode_known_layout(&audio, 0, AUDIO_CENTRE_HZ, mode, &Default::default())
                    && f.payload[..payload.len()] == payload[..]
                {
                    sum_snr += f.snr_db;
                    decoded += 1;
                }
            }
            if decoded > 0 {
                let avg = sum_snr / decoded as f32;
                let residual = avg - truth;
                eprintln!("{mode:?},{eb_n0:+.1},{truth:+.2},{avg:+.2},{residual:+.2},{decoded}");
            } else {
                eprintln!("{mode:?},{eb_n0:+.1},{truth:+.2},NA,NA,0");
            }
        }
    }
}
