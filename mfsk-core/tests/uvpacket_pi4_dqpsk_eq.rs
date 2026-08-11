// SPDX-License-Identifier: GPL-3.0-or-later
//! Phase B-C: equalised π/4-DQPSK with the long 127-chip preamble +
//! adaptive LMS equaliser. Targets the harsh-channel scenarios that
//! Phase B-A (π/4-DQPSK + AFC alone) cannot handle: multi-tap
//! multipath that creates intra-symbol ISI on the matched-filter
//! output.
//!
//! Comparison axis (per `~/.claude/plans/dynamic-cooking-mountain.md`
//! §3.4 "AX.25 replacement / Reiwa-era AX.25"):
//!
//! | Sim                     | Coh QPSK | π/4-DQPSK + AFC | Eq π/4-DQPSK |
//! |-------------------------|---------:|----------------:|-------------:|
//! | AWGN clean (high SNR)   |  10/10   |  10/10          |   ?/10       |
//! | SSB mid-stress (+10 dB) |  13/30 † |  29/30          |   ?/30       |
//! | SSB harsh (+3 dB)       |   0/30   |   0/30          |   ?/30       |
//! | FM harsh (+3 dB)        |   0/30   |   0/30          |   ?/30       |
//!
//! † coherent baseline measured at +6 dB Eb/N0_info.

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
use common::channel::{AwgnChannel, signal_power};
use common::uvpacket_channel::awgn_sigma_for_eb_n0_info;

const N_BLOCKS: u8 = 4;

fn eq_per_awgn(
    mode: Mode,
    payload_size: usize,
    eb_n0_db: f32,
    n_trials: usize,
    base_seed: u64,
) -> usize {
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
        let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0_db, signal_power(&audio));
        let mut chan = AwgnChannel::new(sigma, base_seed.wrapping_add(trial as u64));
        chan.apply(&mut audio);
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

fn eq_per_ssb(
    mode: Mode,
    payload_size: usize,
    chan_template: SsbChannel,
    n_trials: usize,
) -> usize {
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
        let mut chan = chan_template.clone();
        chan.seed = chan_template.seed.wrapping_add(trial as u64);
        chan.apply(&mut audio);
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

fn eq_per_fm(mode: Mode, payload_size: usize, chan_template: FmChannel, n_trials: usize) -> usize {
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
        let mut chan = chan_template.clone();
        chan.seed = chan_template.seed.wrapping_add(trial as u64);
        chan.apply(&mut audio);
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

#[test]
fn eq_pi4_dqpsk_awgn_smoke_gate() {
    let n = 10;
    let decoded = eq_per_awgn(Mode::Robust, 16, 8.0, n, 0xCAFE_BABE);
    eprintln!("Eq π/4-DQPSK AWGN +8 dB Robust decoded {decoded}/{n}");
    assert!(
        decoded >= n * 8 / 10,
        "Eq π/4-DQPSK AWGN smoke gate broke at +8 dB: {decoded}/{n}",
    );
}

#[test]
fn eq_pi4_dqpsk_clean_channel_decodes_all() {
    let n = 10;
    let decoded = eq_per_awgn(Mode::Robust, 16, 20.0, n, 0xC0DE);
    eprintln!("Eq π/4-DQPSK AWGN +20 dB Robust decoded {decoded}/{n}");
    assert_eq!(
        decoded, n,
        "Eq π/4-DQPSK should decode every frame at +20 dB: {decoded}/{n}",
    );
}

/// **Headline**: equalised π/4-DQPSK on a multipath-dominated SSB
/// channel. Other impairments held to "moderate but realistic"
/// levels (clarifier 100 Hz inside AFC range, modest LO walk) so
/// the test isolates the equaliser's contribution against the
/// multi-tap multipath (3 ms / -6 dB principal + 8 ms / -12 dB +
/// 15 ms / -15 dB). The 9-tap LMS equaliser covers the principal
/// 3.6-symbol-delay multipath; the longer taps contribute less
/// energy and are expected to leave residual ISI.
#[test]
fn eq_pi4_dqpsk_recovers_on_ssb_multipath() {
    let n = 30;
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
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 10.0, signal_power(&audio_for_power));
    audio_for_power.clear();
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
            multipath_taps: vec![(3.0, -6.0), (8.0, -12.0), (15.0, -15.0)],
        },
        seed: 0xBABE_F00D,
    };
    let decoded = eq_per_ssb(Mode::Robust, payload_size, chan, n);
    eprintln!("Eq π/4-DQPSK SSB multipath (+10 dB) decoded {decoded}/{n}");
    assert!(
        decoded > 0,
        "Eq π/4-DQPSK should at least partially recover on SSB + multipath: {decoded}/{n}",
    );
}

/// FM-channel multipath recovery — same isolation principle, FM
/// repeater path (de-emphasis + modest discriminator drift +
/// multi-tap MP). The headline use case for "Reiwa-era AX.25".
#[test]
fn eq_pi4_dqpsk_recovers_on_fm_multipath() {
    let n = 30;
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
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 10.0, signal_power(&audio_for_power));
    audio_for_power.clear();
    let chan = FmChannel {
        deemphasis_us: 75.0,
        discriminator_dc_drift_hz: 100.0,
        awgn_sigma: sigma,
        phase_fading: PhaseFadingModel {
            lo_phase_walk_rad_per_sqrt_s: 1.0,
            phase_jitter_rms_rad: 0.15,
            phase_jitter_corr_ms: 1.0,
            doppler_hz: 0.0,
            rician_k_db: 10.0,
            multipath_taps: vec![(3.0, -6.0), (8.0, -12.0), (15.0, -15.0)],
        },
        seed: 0xDEAD_C0DE,
    };
    let decoded = eq_per_fm(Mode::Robust, payload_size, chan, n);
    eprintln!("Eq π/4-DQPSK FM multipath (+10 dB) decoded {decoded}/{n}");
    assert!(
        decoded > 0,
        "Eq π/4-DQPSK should at least partially recover on FM + multipath: {decoded}/{n}",
    );
}

/// Helper for the true-harsh tests: same as `eq_per_ssb` but with
/// a caller-supplied AfcOpts (default ±200 Hz isn't enough when the
/// clarifier offset is ~250 Hz).
fn eq_per_ssb_with_afc(
    mode: Mode,
    payload_size: usize,
    chan_template: SsbChannel,
    n_trials: usize,
    afc: rx::AfcOpts,
) -> usize {
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
        let mut chan = chan_template.clone();
        chan.seed = chan_template.seed.wrapping_add(trial as u64);
        chan.apply(&mut audio);
        if let Ok(frame) = rx::decode_known_layout_with_afc(
            &audio,
            0,
            AUDIO_CENTRE_HZ,
            mode,
            &Default::default(),
            &afc,
        ) && frame.payload[..payload_size] == payload[..]
        {
            decoded += 1;
        }
    }
    decoded
}

fn eq_per_fm_with_afc(
    mode: Mode,
    payload_size: usize,
    chan_template: FmChannel,
    n_trials: usize,
    afc: rx::AfcOpts,
) -> usize {
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
        let mut chan = chan_template.clone();
        chan.seed = chan_template.seed.wrapping_add(trial as u64);
        chan.apply(&mut audio);
        if let Ok(frame) = rx::decode_known_layout_with_afc(
            &audio,
            0,
            AUDIO_CENTRE_HZ,
            mode,
            &Default::default(),
            &afc,
        ) && frame.payload[..payload_size] == payload[..]
        {
            decoded += 1;
        }
    }
    decoded
}

/// **True-harsh SSB**: the original "everything goes wrong"
/// parameter set (clarifier 250 Hz beyond default AFC range,
/// LO walk 5 rad/√s, multi-tap multipath) — this time with the
/// AFC range explicitly widened to ±400 Hz to cover the clarifier
/// offset, and the SNR raised from the original +3 dB (well below
/// LDPC threshold for differential) to +10 dB to give the diff
/// demod its expected ~5 dB threshold loss back.
#[test]
fn eq_pi4_dqpsk_recovers_on_ssb_true_harsh() {
    let n = 30;
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
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 10.0, signal_power(&audio_for_power));
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
    let wide_afc = rx::AfcOpts { search_hz: 400.0 };
    let decoded = eq_per_ssb_with_afc(Mode::Robust, payload_size, chan, n, wide_afc);
    eprintln!("Eq π/4-DQPSK SSB true-harsh (+10 dB, AFC ±400) decoded {decoded}/{n}");
    assert!(
        decoded > 0,
        "Eq π/4-DQPSK with wide AFC should recover ≥1 frame on true-harsh SSB: {decoded}/{n}",
    );
}

/// **True-harsh FM**: same impairment escalation through the FM
/// repeater path. The headline "Reiwa-era AX.25" stress test.
#[test]
fn eq_pi4_dqpsk_recovers_on_fm_true_harsh() {
    let n = 30;
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
    // FM true-harsh threshold sits at ~+15 dB Eb/N0_info in the
    // characterisation sweep (de-emphasis + Rician fading + multi-
    // tap MP combine to make this a much harder channel than SSB
    // true-harsh). Use +15 dB to give the gate margin.
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 15.0, signal_power(&audio_for_power));
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
    let wide_afc = rx::AfcOpts { search_hz: 400.0 };
    let decoded = eq_per_fm_with_afc(Mode::Robust, payload_size, chan, n, wide_afc);
    eprintln!("Eq π/4-DQPSK FM true-harsh (+15 dB, AFC ±400) decoded {decoded}/{n}");
    assert!(
        decoded > 0,
        "Eq π/4-DQPSK with wide AFC should recover ≥1 frame on true-harsh FM: {decoded}/{n}",
    );
}

/// Diagnostic SNR sweep on the true-harsh SSB / FM channels with
/// wide AFC. Shows where Eq π/4-DQPSK threshold lands when faced
/// with the full impairment stack.
#[test]
#[ignore = "slow: Eq π/4-DQPSK true-harsh SNR sweep"]
fn eq_pi4_dqpsk_true_harsh_snr_sweep() {
    let n = 30;
    let payload_size = 16;
    let mode = Mode::Robust;
    let probe = tx::encode(
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
    let wide_afc = rx::AfcOpts { search_hz: 400.0 };

    eprintln!("channel,eb_n0_db,decoded,total");
    for eb_n0 in [6.0_f32, 8.0, 10.0, 12.0, 15.0, 20.0] {
        let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&probe));
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
        let decoded = eq_per_ssb_with_afc(mode, payload_size, chan, n, wide_afc);
        eprintln!("SSB-true-harsh,{eb_n0:+.0},{decoded},{n}");
    }
    for eb_n0 in [6.0_f32, 8.0, 10.0, 12.0, 15.0, 20.0] {
        let sigma = awgn_sigma_for_eb_n0_info(mode, eb_n0, signal_power(&probe));
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
        let decoded = eq_per_fm_with_afc(mode, payload_size, chan, n, wide_afc);
        eprintln!("FM-true-harsh,{eb_n0:+.0},{decoded},{n}");
    }
}

#[test]
#[ignore = "slow: Eq π/4-DQPSK AWGN PER sweep"]
fn eq_pi4_dqpsk_awgn_per_sweep() {
    let n = 30;
    eprintln!("mode,eb_n0_db,decoded,total");
    for mode in [
        Mode::Robust,
        Mode::Standard,
        Mode::UltraRobust,
        Mode::Express,
    ] {
        for eb_n0 in [-2.0_f32, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0] {
            let decoded = eq_per_awgn(mode, 16, eb_n0, n, 0x1234_5678);
            eprintln!("{mode:?},{eb_n0:+.1},{decoded},{n}");
        }
    }
}
