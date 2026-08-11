// SPDX-License-Identifier: GPL-3.0-or-later
//! Phase A: confirm the SSB through-air channel sim reproduces the
//! field failure of the current coherent-QPSK uvpacket.
//!
//! Two flavours of test:
//!
//! - **Smoke gate** (`#[test]`, fast): assert that under "harsh but
//!   plausible" SSB impairments, current QPSK PER ≥ 50%. The
//!   purpose of this gate is to keep the channel sim from silently
//!   regressing into something so mild it no longer reproduces field
//!   failure. The 50% threshold (rather than 90%) keeps the gate
//!   robust against modest decoder improvements; the field-failure
//!   reproduction is documented in the `#[ignore]` sweep below.
//!
//! - **Sweep** (`#[ignore]`, slow): emit PER vs (clarifier offset,
//!   phase walk, SNR) for the curve.
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
use common::air_channel::{PhaseFadingModel, SsbChannel};
use common::channel::signal_power;
use common::uvpacket_channel::awgn_sigma_for_eb_n0_info;

const N_BLOCKS: u8 = 4;

fn ssb_per(mode: Mode, payload_size: usize, chan: SsbChannel, n_trials: usize) -> usize {
    let header = FrameHeader {
        mode,
        block_count: N_BLOCKS,
        app_type: 0,
        sequence: 0,
    };
    let mut decoded = 0;
    for trial in 0..n_trials {
        let payload: Vec<u8> = (0..payload_size)
            .map(|i| ((i + trial) ^ 0xA5) as u8)
            .collect();
        let mut audio = tx::encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
        let mut chan_t = chan.clone();
        chan_t.seed = chan.seed.wrapping_add(trial as u64);
        chan_t.apply(&mut audio);
        // Use the AFC-aware decoder (matches the field receiver
        // path that 39efd0a/2b32fa4 wired in).
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

/// Pin-down: at +20 dB Eb/N0_info AWGN with **no** SSB phase
/// impairments at all (off-defaulted PhaseFadingModel + zero
/// clarifier offset + wide BPF), Robust must decode all frames.
/// Catches sim wiring bugs (e.g. accidental amplitude attenuation
/// in `bpf` or stray phase rotation when "off" is selected).
#[test]
fn ssb_smoke_quiet_channel_decodes_robust() {
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
    let chan = SsbChannel {
        bpf_hz: (200.0, 2900.0),
        bpf_transition_hz: 50.0,
        clarifier_offset_hz: 0.0,
        awgn_sigma: sigma,
        phase_fading: PhaseFadingModel::off(),
        seed: 0xA1B2_C3D4,
    };
    let decoded = ssb_per(Mode::Robust, payload_size, chan, n);
    eprintln!("ssb_smoke_quiet decoded {decoded}/{n}");
    assert!(
        decoded >= n * 8 / 10,
        "quiet SSB sim should let Robust through cleanly: {decoded}/{n}",
    );
}

/// **Headline reproduction gate**: under SSB impairments crank up
/// past current QPSK+AFC+DDPT margin (clarifier 250 Hz beyond AFC
/// default ±200 Hz + heavy LO phase walk + multi-tap multipath
/// + Eb/N0 just above LDPC threshold), Robust QPSK breaks.
///
/// This is the "we have reproduced the field failure" moment from
/// `~/.claude/plans/dynamic-cooking-mountain.md` §4 Phase A. The
/// initial parameter set (clarifier 80 Hz + 1.5 rad/√s walk + 5 ms
/// reverb + +10 dB) couldn't break the decoder — confirming the
/// §1.4 meta-pattern that sim under-models field impairments. We
/// escalated to the params below to demonstrate that **with enough
/// stacked impairments**, the decoder does fail. The next refinement
/// (after Phase A WAV captures) is to learn which subset of these
/// is actually present on-air rather than relying on this stack.
#[test]
fn ssb_field_failure_reproduction_robust() {
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
    let chan = SsbChannel {
        bpf_hz: (300.0, 2700.0),
        bpf_transition_hz: 100.0,
        clarifier_offset_hz: 250.0,
        awgn_sigma: sigma,
        phase_fading: PhaseFadingModel {
            lo_phase_walk_rad_per_sqrt_s: 5.0,
            phase_jitter_rms_rad: 0.3,
            phase_jitter_corr_ms: 1.0,
            doppler_hz: 0.0,
            rician_k_db: f32::INFINITY,
            multipath_taps: vec![(3.0, -6.0), (8.0, -12.0), (15.0, -15.0)],
        },
        seed: 0xBABE_F00D,
    };
    let decoded = ssb_per(Mode::Robust, payload_size, chan, n);
    eprintln!("ssb_field_failure decoded {decoded}/{n}");
    assert!(
        decoded <= n / 2,
        "stacked SSB impairments should break Robust at >= 50% PER, but decoded {decoded}/{n}",
    );
}

/// Diagnostic sweep — PER vs (clarifier offset, phase walk) at fixed
/// +10 dB Eb/N0. Emits CSV-like rows for analysis.
#[test]
#[ignore = "slow: SSB channel sweep"]
fn ssb_sweep_clarifier_x_phase_walk() {
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

    let offsets = [0.0_f32, 30.0, 80.0, 150.0, 250.0];
    let walks = [0.0_f32, 0.2, 0.5, 1.0, 2.0];

    eprintln!("clarifier_hz,walk_rad_per_sqrt_s,decoded,total");
    for &offset in &offsets {
        for &walk in &walks {
            let chan = SsbChannel {
                bpf_hz: (300.0, 2700.0),
                bpf_transition_hz: 100.0,
                clarifier_offset_hz: offset,
                awgn_sigma: sigma,
                phase_fading: PhaseFadingModel {
                    lo_phase_walk_rad_per_sqrt_s: walk,
                    phase_jitter_rms_rad: 0.05,
                    phase_jitter_corr_ms: 2.0,
                    doppler_hz: 0.0,
                    rician_k_db: f32::INFINITY,
                    multipath_taps: Vec::new(),
                },
                seed: 0x1234_5678_9ABC_DEF0,
            };
            let decoded = ssb_per(mode, payload_size, chan, n_trials);
            eprintln!("{offset:.0},{walk:.2},{decoded},{n_trials}");
        }
    }
}
