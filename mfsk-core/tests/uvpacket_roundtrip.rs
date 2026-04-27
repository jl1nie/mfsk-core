// SPDX-License-Identifier: GPL-3.0-or-later
//! Headless uvpacket round-trip and channel-impairment tests.
//!
//! These tests exercise the full TX → channel → RX path on synthetic
//! audio for every uvpacket sub-mode. They cover three channels:
//!
//! 1. **Clean baseband** — pad-to-buffer, no impairments. Sanity check
//!    that the encoder, [`crate::core::tx::codeword_to_itone`]
//!    interleaver, two-Costas-block sync layout, RX deinterleaver and
//!    [`crate::msg::PacketBytesMessage`] CRC-7 all line up byte-for-byte.
//!
//! 2. **AWGN at moderate SNR** — additive Gaussian noise, no fading.
//!    Confirms LDPC corrects scattered errors.
//!
//! 3. **Time-selective Rayleigh fading + AWGN** — the design driver:
//!    a slowly time-varying complex Gaussian channel gain (~5 Hz
//!    Doppler) modulates the signal envelope, producing deep
//!    short-lived nulls. Tests the bit-interleaver + dual-Costas
//!    sync combination by exposing the receiver to bursts of
//!    corrupted channel bits.

#![cfg(feature = "uvpacket")]

use mfsk_core::uvpacket::{
    DecodedPacket, UvPacket150, UvPacket300, UvPacket600, UvPacket1200,
    decode_frame, synthesize_packet,
};
use mfsk_core::{ModulationParams, Protocol};

const AUDIO_SAMPLES: usize = 12_000; // 1 s @ 12 kHz
const TX_FREQ_HZ: f32 = 1_500.0; // mid-band, fits every sub-mode
const TX_AMPLITUDE: f32 = 0.5;

// ────────────────────────────────────────────────────────────────────
// Helpers — channel impairments

/// 32-bit linear-congruential PRNG (deterministic, seedable).
/// Test reproducibility matters more than statistical purity.
struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xDEAD_BEEF_CAFE_F00D)
    }
    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        (self.0 >> 32) as u32
    }
    /// Two independent N(0,1) samples via Box-Muller.
    fn next_gauss_pair(&mut self) -> (f32, f32) {
        // Avoid log(0): subtract from 1.
        let u1 = (self.next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0);
        let u2 = (self.next_u32() as f32) / (u32::MAX as f32);
        let r = (-2.0_f32 * u1.ln()).sqrt();
        let theta = 2.0 * std::f32::consts::PI * u2;
        (r * theta.cos(), r * theta.sin())
    }
    fn next_gauss(&mut self) -> f32 {
        self.next_gauss_pair().0
    }
}

/// Add zero-mean Gaussian noise of the given standard deviation.
fn add_awgn(samples: &mut [f32], sigma: f32, seed: u64) {
    let mut rng = Lcg::new(seed);
    for s in samples {
        *s += sigma * rng.next_gauss();
    }
}

/// Apply a flat-fading Rayleigh channel gain in place.
///
/// Generates two independent low-pass-filtered Gaussian processes
/// (in-phase and quadrature) at `doppler_hz` cutoff, forms the
/// complex envelope, and modulates the real signal by its magnitude.
/// The Rayleigh envelope statistics match a
/// time-selective-but-frequency-flat U/VHF channel.
///
/// `doppler_hz` of 5 Hz is a typical mobile rate at U/VHF; bumping
/// it to 20+ Hz produces faster fades. `mean_gain` controls the
/// long-run average (1.0 keeps the signal at the same average power).
fn apply_rayleigh_fade(samples: &mut [f32], doppler_hz: f32, mean_gain: f32, seed: u64) {
    let sample_rate = 12_000.0_f32;
    let mut rng = Lcg::new(seed.wrapping_add(0x5EED_C0DE));

    // First-order IIR low-pass for each Gaussian process. Cutoff =
    // doppler_hz; alpha = exp(-2π · fc / fs).
    let alpha = (-2.0 * std::f32::consts::PI * doppler_hz / sample_rate).exp();
    let one_minus = 1.0 - alpha;
    // Variance scaling so the filtered output has unit variance per
    // axis. For a one-pole IIR with white-Gaussian input of variance
    // σ² and pole α, output variance is σ² · (1 - α) / (1 + α).
    // Solve for σ given output variance = 1: σ² = (1 + α) / (1 - α).
    let drive = ((1.0 + alpha) / (1.0 - alpha)).sqrt();

    let mut h_i = 0.0f32;
    let mut h_q = 0.0f32;
    for s in samples {
        let (g_i, g_q) = rng.next_gauss_pair();
        h_i = alpha * h_i + one_minus * drive * g_i;
        h_q = alpha * h_q + one_minus * drive * g_q;
        // Rayleigh-distributed envelope = sqrt(h_i² + h_q²) / sqrt(2)
        // (the factor of √2 normalises so E|h|² = 1 when both axes
        // have unit variance).
        let env = (h_i * h_i + h_q * h_q).sqrt() * std::f32::consts::FRAC_1_SQRT_2;
        *s *= env * mean_gain;
    }
}

// ────────────────────────────────────────────────────────────────────
// Test scaffolding

fn synth<P: Protocol>(payload: &[u8]) -> Vec<f32> {
    let mut audio =
        synthesize_packet::<P>(payload, TX_FREQ_HZ, TX_AMPLITUDE).expect("payload fits in frame");
    audio.resize(AUDIO_SAMPLES, 0.0);
    audio
}

fn to_i16(samples: &[f32]) -> Vec<i16> {
    samples
        .iter()
        .map(|&s| (s * 20_000.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

fn run_decode<P: Protocol>(audio_i16: &[i16]) -> Vec<DecodedPacket> {
    decode_frame::<P>(
        audio_i16,
        /* freq_min */ 100.0,
        /* freq_max */ 5_500.0,
        // Lower sync_min for fading channels — partial sync is the
        // expected case; LDPC + interleaver carries the data.
        /* sync_min */ 0.4,
        /* max_cand */ 50,
    )
}

fn assert_decoded(decoded: &[DecodedPacket], expected: &[u8], label: &str) {
    assert!(
        !decoded.is_empty(),
        "{label}: no frames decoded (expected payload {expected:?})"
    );
    let hit = decoded.iter().find(|d| d.payload == expected);
    assert!(
        hit.is_some(),
        "{label}: payload {expected:?} not in decoded set {:?}",
        decoded.iter().map(|d| &d.payload).collect::<Vec<_>>()
    );
}

// ────────────────────────────────────────────────────────────────────
// (1) Clean baseband — every sub-mode round-trips

macro_rules! clean_roundtrip_test {
    ($name:ident, $sub:ident, $label:literal) => {
        #[test]
        fn $name() {
            // 1-byte minimum, max 10-byte payload, mid-range pattern.
            for payload in [b"x".as_slice(), b"hello", b"\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09"]
            {
                let audio_f32 = synth::<$sub>(payload);
                let audio_i16 = to_i16(&audio_f32);
                let decoded = run_decode::<$sub>(&audio_i16);
                assert_decoded(&decoded, payload, $label);
            }
        }
    };
}

clean_roundtrip_test!(uv150_clean_roundtrip, UvPacket150, "UvPacket150 clean");
clean_roundtrip_test!(uv300_clean_roundtrip, UvPacket300, "UvPacket300 clean");
clean_roundtrip_test!(uv600_clean_roundtrip, UvPacket600, "UvPacket600 clean");
clean_roundtrip_test!(uv1200_clean_roundtrip, UvPacket1200, "UvPacket1200 clean");

// ────────────────────────────────────────────────────────────────────
// (2) AWGN at moderate SNR

/// Compute the noise-σ that yields the requested per-sample SNR (dB)
/// against a sinusoid of amplitude 1.0.
///
/// Signal power for amplitude A is A²/2. Noise power for σ is σ².
/// SNR (dB) = 10·log₁₀(A² / (2σ²))  →  σ = A / sqrt(2 · 10^(SNR/10)).
fn awgn_sigma_for_snr_db(snr_db: f32, signal_amplitude: f32) -> f32 {
    let snr_lin = 10.0_f32.powf(snr_db / 10.0);
    signal_amplitude / (2.0 * snr_lin).sqrt()
}

#[test]
fn uv300_decodes_at_moderate_awgn() {
    // UvPacket300 at +5 dB SNR — comfortable above LDPC threshold
    // (~−10 dB at 25 Hz noise BW for FT4-class codes; uvpacket has
    // similar shape).
    let payload = b"hello uv";
    let mut audio = synth::<UvPacket300>(payload);
    let sigma = awgn_sigma_for_snr_db(/* snr_db */ 5.0, TX_AMPLITUDE);
    add_awgn(&mut audio, sigma, /* seed */ 0xA5A5_A5A5);
    let decoded = run_decode::<UvPacket300>(&to_i16(&audio));
    assert_decoded(&decoded, payload, "UvPacket300 + AWGN(+5 dB)");
}

#[test]
fn uv1200_decodes_at_moderate_awgn() {
    // UvPacket1200 has the widest BW so noise integrated over its
    // band is largest; bump SNR to +10 dB so we're cleanly above the
    // operating point.
    let payload = b"fm-data";
    let mut audio = synth::<UvPacket1200>(payload);
    let sigma = awgn_sigma_for_snr_db(/* snr_db */ 10.0, TX_AMPLITUDE);
    add_awgn(&mut audio, sigma, /* seed */ 0xFEED_FACE);
    let decoded = run_decode::<UvPacket1200>(&to_i16(&audio));
    assert_decoded(&decoded, payload, "UvPacket1200 + AWGN(+10 dB)");
}

// ────────────────────────────────────────────────────────────────────
// (3) Time-selective Rayleigh fading + AWGN
//
// This is the headline test: it confirms the bit-interleaver and
// two-Costas-block sync layout actually buy what the docs claim.

#[test]
fn uv300_decodes_through_rayleigh_fade() {
    // 5 Hz Doppler ≈ 200 ms coherence time; over the 317 ms
    // UvPacket300 frame that's ~1.5 fades, plenty to corrupt
    // consecutive symbols if the interleaver weren't in place.
    let payload = b"fade me";
    let mut audio = synth::<UvPacket300>(payload);
    apply_rayleigh_fade(
        &mut audio,
        /* doppler_hz */ 5.0,
        /* mean_gain */ 1.4, // compensate for sqrt(2) loss + envelope variance
        /* seed */ 0xDEAD_BEEF,
    );
    add_awgn(
        &mut audio,
        awgn_sigma_for_snr_db(/* snr_db */ 12.0, TX_AMPLITUDE),
        /* seed */ 0xC0FF_EE00,
    );
    let decoded = run_decode::<UvPacket300>(&to_i16(&audio));
    assert_decoded(&decoded, payload, "UvPacket300 + Rayleigh(5 Hz) + AWGN(+12 dB)");
}

#[test]
fn uv150_decodes_through_slow_rayleigh_fade() {
    // UvPacket150 has the longest 633 ms frame and the deepest
    // burst-tolerance need: at 2 Hz Doppler the coherence time is
    // ~500 ms so a single deep null can swallow ~1/4 of the frame.
    // The interleaver scatters those losses across the 174-bit
    // codeword and LDPC takes it from there.
    let payload = b"slow fade";
    let mut audio = synth::<UvPacket150>(payload);
    apply_rayleigh_fade(
        &mut audio,
        /* doppler_hz */ 2.0,
        /* mean_gain */ 1.4,
        /* seed */ 0x1234_5678,
    );
    add_awgn(
        &mut audio,
        awgn_sigma_for_snr_db(/* snr_db */ 10.0, TX_AMPLITUDE),
        /* seed */ 0x9999_AAAA,
    );
    let decoded = run_decode::<UvPacket150>(&to_i16(&audio));
    assert_decoded(&decoded, payload, "UvPacket150 + Rayleigh(2 Hz) + AWGN(+10 dB)");
}

// ────────────────────────────────────────────────────────────────────
// (4) Burst-null sanity check
//
// Hard-zero a contiguous segment of audio big enough to wipe out the
// head Costas block, then verify the mid-frame Costas + interleaver
// still recovers the frame. Targets the "what if a deep fade hits the
// frame head?" question directly.

#[test]
fn uv300_decodes_with_head_null() {
    let payload = b"survive";
    let mut audio = synth::<UvPacket300>(payload);
    // Zero out the first 50 ms — comfortably eats the entire head
    // Costas block (4 symbols × 40 NSPS = 160 samples = 13 ms) plus
    // ~11 leading data symbols (≈ 22 channel bits). After
    // deinterleaving, those 22 bits scatter across the 174-bit
    // codeword at stride 7, well within Ldpc174_91's correction
    // envelope. The mid-frame Costas at symbol 47 (≈ 157 ms in the
    // 12-kHz stream) is intact and gives coarse_sync the lock it
    // needs.
    let null_samples = (0.050 * 12_000.0) as usize;
    for s in audio.iter_mut().take(null_samples) {
        *s = 0.0;
    }
    let decoded = run_decode::<UvPacket300>(&to_i16(&audio));
    assert_decoded(&decoded, payload, "UvPacket300 + 50 ms head null");
}

// ────────────────────────────────────────────────────────────────────
// Cross-sub-mode property: every payload length 1..=10 round-trips clean

#[test]
fn uv600_every_payload_length_roundtrips() {
    for n in 1..=10 {
        let payload: Vec<u8> = (0..n as u8).collect();
        let audio = synth::<UvPacket600>(&payload);
        let decoded = run_decode::<UvPacket600>(&to_i16(&audio));
        assert_decoded(
            &decoded,
            &payload,
            &format!("UvPacket600 clean payload len={n}"),
        );
    }
}

// Sanity: rate-ladder constants match what the docs claim.
#[test]
fn rate_ladder_constants_match_docs() {
    // All sub-modes derive from the macro so this is mostly belt-
    // and-braces. NSPS × baud must = 12 000 (the 12 kHz pipeline
    // clock).
    fn check<P: ModulationParams>(name: &str, expected_baud: u32) {
        let actual_baud = 12_000 / P::NSPS;
        assert_eq!(actual_baud, expected_baud, "{name} baud mismatch");
        assert_eq!(
            P::TONE_SPACING_HZ,
            expected_baud as f32,
            "{name}: tone spacing must equal baud (orthogonal-FSK)"
        );
    }
    check::<UvPacket150>("UvPacket150", 150);
    check::<UvPacket300>("UvPacket300", 300);
    check::<UvPacket600>("UvPacket600", 600);
    check::<UvPacket1200>("UvPacket1200", 1200);
}
