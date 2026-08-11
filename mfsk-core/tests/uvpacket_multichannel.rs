// SPDX-License-Identifier: GPL-3.0-or-later
//! Multi-channel SSB receive + slot-survey tests for the
//! 0.3.3 slotted-ALOHA design.

#![cfg(feature = "uvpacket")]

use mfsk_core::engine::FecOpts;
use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::rx::{MultiChannelOpts, decode_multichannel, measure_slot_energies};
use mfsk_core::uvpacket::{Mode, tx};

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

fn default_fec_opts() -> FecOpts<'static> {
    FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ap_mask: None,
        verify_info: None,
        ..FecOpts::default()
    }
}

fn header_for(seq: u8) -> FrameHeader {
    FrameHeader {
        mode: Mode::Robust,
        block_count: 4,
        app_type: 0,
        sequence: seq,
    }
}

fn add_at(dst: &mut Vec<f32>, src: &[f32], offset: usize) {
    let need = offset + src.len();
    if dst.len() < need {
        dst.resize(need, 0.0);
    }
    for (i, &s) in src.iter().enumerate() {
        dst[offset + i] += s;
    }
}

/// Two simultaneous frames at different audio centres in the
/// same audio buffer — both must decode, with the detected
/// centre matching each frame's true centre.
#[test]
fn decode_multichannel_finds_two_simultaneous_frames() {
    let header_a = header_for(1);
    let header_b = header_for(2);
    let payload_a: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();
    let payload_b: Vec<u8> = (0..20).map(|i| (i ^ 0xC3) as u8).collect();

    let burst_a = tx::encode(&header_a, &payload_a, 800.0).unwrap();
    let burst_b = tx::encode(&header_b, &payload_b, 2000.0).unwrap();

    let mut audio: Vec<f32> = Vec::new();
    add_at(&mut audio, &burst_a, 0);
    add_at(&mut audio, &burst_b, 100); // small time offset

    let frames = decode_multichannel(&audio, &MultiChannelOpts::default(), &default_fec_opts());
    assert_eq!(frames.len(), 2, "got {} frames", frames.len());

    let frame_a = frames
        .iter()
        .find(|(f, _)| (*f - 800.0).abs() < 50.0)
        .expect("no frame near 800 Hz");
    let frame_b = frames
        .iter()
        .find(|(f, _)| (*f - 2000.0).abs() < 50.0)
        .expect("no frame near 2000 Hz");
    assert_eq!(frame_a.1.payload[..payload_a.len()], payload_a[..]);
    assert_eq!(frame_b.1.payload[..payload_b.len()], payload_b[..]);
    assert_eq!(frame_a.1.sequence, 1);
    assert_eq!(frame_b.1.sequence, 2);
}

/// Single frame anywhere in the band: regression that
/// multi-channel rx still finds the lone signal.
#[test]
fn decode_multichannel_finds_single_frame() {
    let header = header_for(7);
    let payload: Vec<u8> = (0..16).map(|i| (i ^ 0xAA) as u8).collect();
    let burst = tx::encode(&header, &payload, 1500.0).unwrap();

    let frames = decode_multichannel(&burst, &MultiChannelOpts::default(), &default_fec_opts());
    assert_eq!(frames.len(), 1);
    let (centre, frame) = &frames[0];
    assert!((*centre - 1500.0).abs() < 50.0, "centre = {centre}");
    assert_eq!(frame.payload[..payload.len()], payload[..]);
    assert_eq!(frame.sequence, 7);
}

/// Empty / silent audio → no frames.
#[test]
fn decode_multichannel_empty_returns_empty() {
    let audio = vec![0.0_f32; 12_000];
    let frames = decode_multichannel(&audio, &MultiChannelOpts::default(), &default_fec_opts());
    assert!(frames.is_empty());
}

/// Two simultaneous frames with AWGN at moderate Eb/N0_info:
/// both must decode.
#[test]
fn decode_multichannel_awgn_smoke() {
    let header_a = header_for(11);
    let header_b = header_for(12);
    let payload: Vec<u8> = (0..16).map(|i| (i ^ 0x5A) as u8).collect();

    let burst_a = tx::encode(&header_a, &payload, 800.0).unwrap();
    let burst_b = tx::encode(&header_b, &payload, 2000.0).unwrap();

    let mut audio: Vec<f32> = Vec::new();
    add_at(&mut audio, &burst_a, 0);
    add_at(&mut audio, &burst_b, 200);

    // Use one σ, calibrated to the combined audio power. Each
    // burst sees ~3 dB less effective SNR; with the post-redesign
    // differential-demod threshold near +6 dB Eb/N0, both bursts
    // need ≥ +9 dB combined to clear threshold. +12 dB gives
    // comfortable margin for both decodes.
    let sp = signal_power(&audio);
    let sigma = awgn_sigma_for_eb_n0_info(Mode::Robust, 12.0, sp);
    AwgnChannel::new(sigma, 0xCAFE_BABE).apply(&mut audio);

    let frames = decode_multichannel(&audio, &MultiChannelOpts::default(), &default_fec_opts());
    assert!(
        frames.len() >= 2,
        "expected ≥ 2 frames at +12 dB, got {}",
        frames.len()
    );
}

// ───────── Slot survey ──────────────────────────────────────────────

/// A single uvpacket signal at a known audio centre lights up
/// the matched filter at the corresponding slot more than the
/// other one.
#[test]
fn measure_slot_energies_busy_vs_free() {
    let header = header_for(0);
    let payload: Vec<u8> = vec![0xAA; 20];
    let burst = tx::encode(&header, &payload, 800.0).unwrap();

    let slots = measure_slot_energies(&burst, &MultiChannelOpts::default(), 1200.0);
    assert_eq!(slots.len(), 2);

    // Default band: 300–2700 Hz, slot_spacing = 1200 Hz →
    // centres at 900 and 2100 Hz.
    let (busy, free) = if (slots[0].audio_centre_hz - 900.0).abs() < 200.0 {
        (&slots[0], &slots[1])
    } else {
        (&slots[1], &slots[0])
    };
    assert!(
        busy.mean_mf_magnitude > free.mean_mf_magnitude * 5.0,
        "busy {} should be ≫ free {}",
        busy.mean_mf_magnitude,
        free.mean_mf_magnitude
    );
}

/// Silent audio: both slots report comparably low energies.
#[test]
fn measure_slot_energies_silent_audio_uniform() {
    let audio = vec![0.0_f32; 12_000];
    let slots = measure_slot_energies(&audio, &MultiChannelOpts::default(), 1200.0);
    assert_eq!(slots.len(), 2);
    let max_e = slots
        .iter()
        .map(|s| s.mean_mf_magnitude)
        .fold(0.0_f32, f32::max);
    let min_e = slots
        .iter()
        .map(|s| s.mean_mf_magnitude)
        .fold(f32::INFINITY, f32::min);
    assert!(max_e <= min_e * 2.0 + 1e-9, "{max_e} vs {min_e}");
}
