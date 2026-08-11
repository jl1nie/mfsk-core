// SPDX-License-Identifier: GPL-3.0-or-later
//! Comprehensive PER characterisation across all four [`Mode`]s
//! and the channel models that ship in the in-tree air-channel
//! sim. Each sweep is `#[ignore]` (slow); run individually with
//! `cargo test --release --test uvpacket_per_modes_sweep <name>
//! -- --ignored --nocapture`.
//!
//! Channels covered:
//! - **AWGN** — clean / noise-only baseline
//! - **Rayleigh flat fading** — magnitude-only, single-tap
//!   (existing `RayleighFlatChannel`)
//! - **SSB realistic** — clarifier offset + LO walk + soft BPF
//! - **FM realistic** — discriminator drift + de-emphasis + light
//!   multipath
//! - **Multipath** — multi-tap acoustic-style (3 + 8 + 15 ms)

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
use common::air_channel::{FmChannel, PhaseFadingModel, SsbChannel};
use common::channel::{AwgnChannel, RayleighFlatChannel, signal_power};
use common::uvpacket_channel::awgn_sigma_for_eb_n0_info;

const N_BLOCKS: u8 = 4;
const N_TRIALS: usize = 30;
const PAYLOAD: usize = 16;
const ALL_MODES: [Mode; 4] = [
    Mode::UltraRobust,
    Mode::Robust,
    Mode::Standard,
    Mode::Express,
];

fn header_for(mode: Mode) -> FrameHeader {
    FrameHeader {
        mode,
        block_count: N_BLOCKS,
        app_type: 0,
        sequence: 0,
    }
}

fn payload(trial: usize) -> Vec<u8> {
    (0..PAYLOAD).map(|i| ((i + trial) ^ 0xA5) as u8).collect()
}

fn try_decode(audio: &[f32], expected: &[u8], mode: Mode, afc: &rx::AfcOpts) -> bool {
    matches!(
        rx::decode_known_layout_with_afc(audio, 0, AUDIO_CENTRE_HZ, mode, &Default::default(), afc),
        Ok(f) if f.payload[..expected.len()] == expected[..]
    )
}

/// Run `n_trials` decodes and return the count that passed.
fn run_per<F>(mode: Mode, mut per_trial_audio: F) -> usize
where
    F: FnMut(&FrameHeader, &[u8], usize) -> Vec<f32>,
{
    let header = header_for(mode);
    let afc = rx::AfcOpts::default();
    (0..N_TRIALS)
        .filter(|&t| {
            let pl = payload(t);
            let audio = per_trial_audio(&header, &pl, t);
            try_decode(&audio, &pl, mode, &afc)
        })
        .count()
}

/// AWGN sweep — threshold reference for the differential demod path.
#[test]
#[ignore = "slow: AWGN PER sweep across all modes"]
fn modes_awgn_sweep() {
    let eb_n0_grid = [-2.0_f32, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0];
    eprintln!("channel,mode,eb_n0_db,decoded,total");
    for mode in ALL_MODES {
        for &eb_n0 in &eb_n0_grid {
            let decoded = run_per(mode, |h, pl, t| {
                let mut audio = tx::encode(h, pl, AUDIO_CENTRE_HZ).unwrap();
                let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&audio));
                AwgnChannel::new(sigma, 0xCAFE0000 + t as u64).apply(&mut audio);
                audio
            });
            eprintln!("AWGN,{mode:?},{eb_n0:+.1},{decoded},{N_TRIALS}");
        }
    }
}

/// Rayleigh flat fading (magnitude-only, no phase). Sweeps Doppler ×
/// Eb/N0. The new differential demod is invariant to the phase the
/// existing `RayleighFlatChannel` doesn't simulate, so this measures
/// pure amplitude-fade tolerance.
#[test]
#[ignore = "slow: Rayleigh PER sweep across all modes"]
fn modes_rayleigh_sweep() {
    let dopplers = [1.0_f32, 5.0, 10.0];
    let eb_n0_grid = [4.0_f32, 8.0, 12.0, 16.0, 20.0, 25.0];
    eprintln!("channel,mode,doppler_hz,eb_n0_db,decoded,total");
    for mode in ALL_MODES {
        for &fd in &dopplers {
            for &eb_n0 in &eb_n0_grid {
                let decoded = run_per(mode, |h, pl, t| {
                    let mut audio = tx::encode(h, pl, AUDIO_CENTRE_HZ).unwrap();
                    let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&audio));
                    RayleighFlatChannel::new(fd, sigma, 0x1234_0000 + t as u64).apply(&mut audio);
                    audio
                });
                eprintln!("Rayleigh,{mode:?},{fd:.0},{eb_n0:+.1},{decoded},{N_TRIALS}");
            }
        }
    }
}

/// SSB realistic — clarifier offset + LO phase walk + multipath
/// modest. The headline use case for the redesign.
#[test]
#[ignore = "slow: SSB realistic PER sweep across all modes"]
fn modes_ssb_realistic_sweep() {
    let eb_n0_grid = [4.0_f32, 6.0, 8.0, 10.0, 12.0, 15.0];
    eprintln!("channel,mode,eb_n0_db,decoded,total");
    for mode in ALL_MODES {
        for &eb_n0 in &eb_n0_grid {
            let probe = tx::encode(&header_for(mode), &[0u8; PAYLOAD], AUDIO_CENTRE_HZ).unwrap();
            let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&probe));
            let decoded = run_per(mode, |h, pl, t| {
                let mut audio = tx::encode(h, pl, AUDIO_CENTRE_HZ).unwrap();
                let chan = SsbChannel {
                    bpf_hz: (300.0, 2700.0),
                    bpf_transition_hz: 100.0,
                    clarifier_offset_hz: 100.0,
                    awgn_sigma: sigma,
                    phase_fading: PhaseFadingModel {
                        lo_phase_walk_rad_per_sqrt_s: 2.0,
                        phase_jitter_rms_rad: 0.1,
                        phase_jitter_corr_ms: 2.0,
                        doppler_hz: 0.0,
                        rician_k_db: f32::INFINITY,
                        multipath_taps: vec![(5.0, -10.0)],
                    },
                    seed: 0xBABE_0000 + t as u64,
                };
                chan.apply(&mut audio);
                audio
            });
            eprintln!("SSB-realistic,{mode:?},{eb_n0:+.1},{decoded},{N_TRIALS}");
        }
    }
}

/// FM realistic — de-emphasis + small discriminator drift + light
/// multipath + Rician fading typical of VHF NFM repeater paths.
#[test]
#[ignore = "slow: FM realistic PER sweep across all modes"]
fn modes_fm_realistic_sweep() {
    let eb_n0_grid = [6.0_f32, 8.0, 10.0, 12.0, 15.0, 20.0];
    eprintln!("channel,mode,eb_n0_db,decoded,total");
    for mode in ALL_MODES {
        for &eb_n0 in &eb_n0_grid {
            let probe = tx::encode(&header_for(mode), &[0u8; PAYLOAD], AUDIO_CENTRE_HZ).unwrap();
            let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&probe));
            let decoded = run_per(mode, |h, pl, t| {
                let mut audio = tx::encode(h, pl, AUDIO_CENTRE_HZ).unwrap();
                let chan = FmChannel {
                    deemphasis_us: 75.0,
                    discriminator_dc_drift_hz: 50.0,
                    awgn_sigma: sigma,
                    phase_fading: PhaseFadingModel {
                        lo_phase_walk_rad_per_sqrt_s: 1.0,
                        phase_jitter_rms_rad: 0.15,
                        phase_jitter_corr_ms: 1.0,
                        doppler_hz: 0.0,
                        rician_k_db: 10.0,
                        multipath_taps: vec![(5.0, -10.0)],
                    },
                    seed: 0xDEAD_0000 + t as u64,
                };
                chan.apply(&mut audio);
                audio
            });
            eprintln!("FM-realistic,{mode:?},{eb_n0:+.1},{decoded},{N_TRIALS}");
        }
    }
}

/// Multi-tap multipath (3 + 8 + 15 ms) — pure ISI test, no other
/// impairments beyond AWGN. Isolates the equaliser's contribution.
#[test]
#[ignore = "slow: multipath PER sweep across all modes"]
fn modes_multipath_sweep() {
    let eb_n0_grid = [6.0_f32, 8.0, 10.0, 12.0, 15.0, 20.0];
    eprintln!("channel,mode,eb_n0_db,decoded,total");
    for mode in ALL_MODES {
        for &eb_n0 in &eb_n0_grid {
            let probe = tx::encode(&header_for(mode), &[0u8; PAYLOAD], AUDIO_CENTRE_HZ).unwrap();
            let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&probe));
            let decoded = run_per(mode, |h, pl, t| {
                let mut audio = tx::encode(h, pl, AUDIO_CENTRE_HZ).unwrap();
                let chan = SsbChannel {
                    bpf_hz: (200.0, 2900.0),
                    bpf_transition_hz: 50.0,
                    clarifier_offset_hz: 0.0,
                    awgn_sigma: sigma,
                    phase_fading: PhaseFadingModel {
                        lo_phase_walk_rad_per_sqrt_s: 0.0,
                        phase_jitter_rms_rad: 0.0,
                        phase_jitter_corr_ms: 1.0,
                        doppler_hz: 0.0,
                        rician_k_db: f32::INFINITY,
                        multipath_taps: vec![(3.0, -6.0), (8.0, -12.0), (15.0, -15.0)],
                    },
                    seed: 0xACE0_0000 + t as u64,
                };
                chan.apply(&mut audio);
                audio
            });
            eprintln!("Multipath,{mode:?},{eb_n0:+.1},{decoded},{N_TRIALS}");
        }
    }
}
