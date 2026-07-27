// SPDX-License-Identifier: GPL-3.0-or-later
//! Direct-LDPC characterisation: bypass the QPSK modem entirely
//! and feed Gaussian-noise LLRs straight to Ldpc240_101 + puncture.
//!
//! Purpose: distinguish "modem implementation loss" from "FEC
//! threshold loss". If the direct path also shows Robust ≈ Express
//! at the threshold, the issue lives in the LDPC / puncture / OSD
//! layer; if the direct path shows the expected rate ordering
//! (Robust beats Express by ~2 dB), the issue lives in the QPSK
//! demod LLR construction.

#![cfg(feature = "uvpacket")]

use mfsk_core::engine::{FecCodec, FecOpts};
use mfsk_core::fec::Ldpc240_101;
use mfsk_core::uvpacket::Mode;
use mfsk_core::uvpacket::puncture::{de_puncture_llr, puncture};

const N: usize = 240;
const K: usize = 101;

/// Box-Muller AWGN.
struct Awgn {
    state: u64,
}

impl Awgn {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
        }
    }
    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }
    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }
}

/// Encode K random info bits, puncture per mode, add per-channel-bit
/// Gaussian noise scaled for `Eb/N0_info`, decode, return whether the
/// info round-tripped.
fn one_trial(mode: Mode, eb_n0_db: f32, seed: u64) -> bool {
    // 1. Random K=101 info bits.
    let mut rng = Awgn::new(seed);
    let info: Vec<u8> = (0..K)
        .map(|_| if rng.uniform() < 0.5 { 0 } else { 1 })
        .collect();

    // 2. Encode → 240 channel bits, then puncture.
    let fec = Ldpc240_101;
    let mut codeword = vec![0u8; N];
    fec.encode(&info, &mut codeword);
    let punctured = puncture(&codeword, mode);

    // 3. BPSK-channel-equivalent LLR construction.
    //
    // Convention: bit `b` is transmitted as `s = 1 - 2b ∈ {+1, -1}`.
    // After AWGN with σ²_n on each axis: r = s + n, n ~ N(0, σ²_n).
    // True LLR(b) = log P(b=1|r)/P(b=0|r) = -2·r/σ²_n.
    //
    // For a target Eb/N0_info γ (linear) at code rate r_code:
    //   Eb_ch = Eb_info · r_code   (energy per channel bit)
    //   With BPSK Es=1: Eb_ch = 1, so N0 = 1/(γ · r_code) → σ²_n = 1/(2·γ·r_code).
    let r_code = K as f32 / mode.ch_bits_per_block() as f32;
    let target_lin = 10f32.powf(eb_n0_db / 10.0);
    let sigma_sq = 1.0 / (2.0 * target_lin * r_code);
    let sigma = sigma_sq.sqrt();

    let mut channel_llrs: Vec<f32> = Vec::with_capacity(punctured.len());
    for &b in &punctured {
        let s = if b == 0 { 1.0_f32 } else { -1.0_f32 };
        let r = s + sigma * rng.gaussian();
        // True max-log LLR with sign convention LLR>0 ⇔ b=1:
        let llr = -2.0 * r / sigma_sq;
        channel_llrs.push(llr);
    }

    // 4. De-puncture and decode.
    let full_llrs = de_puncture_llr(&channel_llrs, mode);
    let opts = FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ap_mask: None,
        verify_info: None,
        ..FecOpts::default()
    };
    let Some(result) = fec.decode_soft(&full_llrs, &opts) else {
        return false;
    };
    result.info == info
}

/// Measure success-count at a given (mode, Eb/N0) over `n_trials`.
fn per(mode: Mode, eb_n0_db: f32, n_trials: usize, base_seed: u64) -> usize {
    (0..n_trials)
        .filter(|t| one_trial(mode, eb_n0_db, base_seed.wrapping_add(*t as u64)))
        .count()
}

/// Fast / Express clean-channel sanity. Catches infrastructure
/// breakage (puncture / de_puncture / encode roundtrip) without
/// running the full sweep.
#[test]
fn direct_ldpc_clean_channel_decodes_every_mode() {
    let n = 10;
    for mode in [
        Mode::Robust,
        Mode::Standard,
        Mode::UltraRobust,
        Mode::Express,
    ] {
        let decoded = per(mode, 30.0, n, 0xCAFE_BABE);
        assert_eq!(
            decoded, n,
            "{mode:?}: clean direct-LDPC failed {decoded}/{n}"
        );
    }
}

/// Headline diagnostic: sweep Eb/N0 across all four modes with the
/// modem cut out of the loop. With the Ldpc240_101 mother code
/// designed for native rate 0.42 (FST4), Robust should hit threshold
/// 2–3 dB *before* Express.
#[test]
#[ignore = "slow: LDPC-only Eb/N0 sweep"]
fn direct_ldpc_threshold_finder_per_mode() {
    let n_trials = 30;
    let eb_n0_grid: [f32; 11] = [-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0];

    eprintln!("mode,eb_n0_db,decoded,total");
    for mode in [
        Mode::Robust,
        Mode::Standard,
        Mode::UltraRobust,
        Mode::Express,
    ] {
        for &eb_n0 in &eb_n0_grid {
            let decoded = per(mode, eb_n0, n_trials, 0xC0FFEE);
            eprintln!("{mode:?},{eb_n0:+.1},{decoded},{n_trials}");
        }
    }
}
