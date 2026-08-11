// SPDX-License-Identifier: GPL-3.0-or-later
//! Phase A: confirm the FM through-air channel sim reproduces the
//! field failure of the current coherent-QPSK uvpacket through an
//! NFM relay path (de-emphasis + discriminator DC drift + acoustic-
//! style multipath).
//!
//! Same structure as `uvpacket_ssb_channel.rs`: a smoke gate proves
//! the channel is wired correctly when impairments are off, and a
//! "field failure reproduction" gate proves the harsh-but-plausible
//! parameter set breaks the current decoder.
//!
//! Background: `~/.claude/plans/dynamic-cooking-mountain.md`
//! §1.3, §4 Phase A.

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
use common::air_channel::{FmChannel, PhaseFadingModel};
use common::channel::signal_power;
use common::uvpacket_channel::awgn_sigma_for_eb_n0_info;

const N_BLOCKS: u8 = 4;

fn fm_per(mode: Mode, payload_size: usize, chan: FmChannel, n_trials: usize) -> usize {
    let header = FrameHeader {
        mode,
        block_count: N_BLOCKS,
        app_type: 0,
        sequence: 0,
    };
    let mut decoded = 0;
    for trial in 0..n_trials {
        let payload: Vec<u8> = (0..payload_size)
            .map(|i| ((i + trial) ^ 0x5A) as u8)
            .collect();
        let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let mut chan_t = chan.clone();
        chan_t.seed = chan.seed.wrapping_add(trial as u64);
        chan_t.apply(&mut audio);
        if let Ok(frame) = rx::decode_known_layout_with_afc(
            &audio,
            0,
            AUDIO_CENTRE_HZ,
            mode,
            &Default::default(),
            &Default::default(),
        ) && frame.payload[..payload_size] == payload[..]
        {
            decoded += 1;
        }
    }
    decoded
}

/// Pin-down: with FM impairments mostly off (no de-emphasis, zero
/// drift, off-defaulted PhaseFadingModel) and +20 dB Eb/N0, Robust
/// must decode all frames. Catches sim wiring bugs.
#[test]
fn fm_smoke_quiet_channel_decodes_robust() {
    let n = 10;
    let payload_size = 16;
    let mut audio_for_power = tx::encode(
        &FrameHeader {
            mode: Mode::Robust,
            block_count: N_BLOCKS,
            app_type: 0,
            sequence: 0,
        },
        &vec![0u8; payload_size],
        AUDIO_CENTRE_HZ,
    )
    .unwrap();
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 20.0, signal_power(&audio_for_power));
    audio_for_power.clear();
    let chan = FmChannel {
        deemphasis_us: 0.0,
        discriminator_dc_drift_hz: 0.0,
        awgn_sigma: sigma,
        phase_fading: PhaseFadingModel::off(),
        seed: 0xC0DE_F00D,
    };
    let decoded = fm_per(Mode::Robust, payload_size, chan, n);
    eprintln!("fm_smoke_quiet decoded {decoded}/{n}");
    assert!(
        decoded >= n * 8 / 10,
        "quiet FM sim should let Robust through cleanly: {decoded}/{n}",
    );
}

/// **Headline reproduction gate**: under FM repeater impairments
/// stacked past current QPSK+AFC+DDPT margin (de-emphasis 75 µs +
/// 250 Hz discriminator drift beyond AFC + heavy phase walk +
/// multi-tap multipath + Eb/N0 just above LDPC threshold), Robust
/// QPSK breaks. Mirrors the SSB gate's escalation pattern
/// documented in `~/.claude/plans/dynamic-cooking-mountain.md` §1.4.
#[test]
fn fm_field_failure_reproduction_robust() {
    let n = 20;
    let payload_size = 16;
    let mut audio_for_power = tx::encode(
        &FrameHeader {
            mode: Mode::Robust,
            block_count: N_BLOCKS,
            app_type: 0,
            sequence: 0,
        },
        &vec![0u8; payload_size],
        AUDIO_CENTRE_HZ,
    )
    .unwrap();
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 3.0, signal_power(&audio_for_power));
    audio_for_power.clear();
    let chan = FmChannel {
        deemphasis_us: 75.0,
        discriminator_dc_drift_hz: 250.0,
        awgn_sigma: sigma,
        phase_fading: PhaseFadingModel {
            lo_phase_walk_rad_per_sqrt_s: 4.0,
            phase_jitter_rms_rad: 0.4,
            phase_jitter_corr_ms: 1.0,
            doppler_hz: 0.0,
            rician_k_db: 8.0,
            multipath_taps: vec![(3.0, -6.0), (8.0, -12.0), (15.0, -15.0)],
        },
        seed: 0xDEAD_C0DE,
    };
    let decoded = fm_per(Mode::Robust, payload_size, chan, n);
    eprintln!("fm_field_failure decoded {decoded}/{n}");
    assert!(
        decoded <= n / 2,
        "stacked FM impairments should break Robust at >= 50% PER, but decoded {decoded}/{n}",
    );
}

/// Diagnostic sweep — PER vs (deemphasis, drift) at fixed +10 dB.
#[test]
#[ignore = "slow: FM channel sweep"]
fn fm_sweep_deemphasis_x_drift() {
    let n_trials = 20;
    let payload_size = 16;
    let mode = Mode::Robust;
    let mut audio_for_power = tx::encode(
        &FrameHeader {
            mode,
            block_count: N_BLOCKS,
            app_type: 0,
            sequence: 0,
        },
        &vec![0u8; payload_size],
        AUDIO_CENTRE_HZ,
    )
    .unwrap();
    let sigma = awgn_sigma_for_eb_n0_info(mode, 10.0, signal_power(&audio_for_power));
    audio_for_power.clear();

    let deemphasis = [0.0_f32, 25.0, 50.0, 75.0, 100.0];
    let drifts = [0.0_f32, 50.0, 100.0, 200.0];

    eprintln!("deemphasis_us,drift_hz,decoded,total");
    for &de in &deemphasis {
        for &dr in &drifts {
            let chan = FmChannel {
                deemphasis_us: de,
                discriminator_dc_drift_hz: dr,
                awgn_sigma: sigma,
                phase_fading: PhaseFadingModel::fm_typical(),
                seed: 0xFADE_BABE_C0DE_BEEF,
            };
            let decoded = fm_per(mode, payload_size, chan, n_trials);
            eprintln!("{de:.0},{dr:.0},{decoded},{n_trials}");
        }
    }
}
