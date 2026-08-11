// SPDX-License-Identifier: GPL-3.0-or-later
//! Eb/N0 helpers for the **uvpacket** tests.
//!
//! These moved out of `common/channel.rs` because they are not
//! protocol-agnostic in any sense: `SYMBOL_RATE_HZ = 1200` and
//! `BITS_PER_SYMBOL = 2` are uvpacket's own π/4-DQPSK PHY, `K_INFO`
//! is its block size, and `info_rate` takes a `uvpacket::Mode`.
//!
//! Keeping them in the shared module made `common/channel.rs` import
//! `mfsk_core::uvpacket::Mode` at file scope, which in turn made
//! **every one of the 43 test binaries that write `mod common;`**
//! require the `uvpacket` feature to compile — including the FT8,
//! FT4 and FST4 tests, which have nothing to do with it. uvpacket is
//! an experimental example built on this crate, not a feature of it,
//! so the shared test scaffolding must not depend on it.

use mfsk_core::uvpacket::Mode;

use super::channel::SAMPLE_RATE_HZ;

/// - Average signal power      `P = signal_power`         (caller-supplied)
/// - Energy per channel bit    `Eb_ch = P / (R_s · b/sym)`
/// - Energy per info bit       `Eb_info = Eb_ch / r_info` where
///   `r_info = K / ch_bits_per_block`
/// - Target ratio              `linear = 10^(eb_n0_db / 10)`
/// - One-sided AWGN PSD        `N0 = Eb_info / linear`
/// - Per-sample variance       `σ² = N0 · fs / 2`
pub fn awgn_sigma_for_eb_n0_info(mode: Mode, eb_n0_db: f32, signal_power: f32) -> f32 {
    let e_b_ch = signal_power / (SYMBOL_RATE_HZ * BITS_PER_SYMBOL);
    let e_b_info = e_b_ch / info_rate(mode);
    let target_linear = 10f32.powf(eb_n0_db / 10.0);
    let n0 = e_b_info / target_linear;
    let sigma_sq = n0 * SAMPLE_RATE_HZ / 2.0;
    sigma_sq.sqrt()
}
/// Per-mode info-rate (information bits per channel bit).
fn info_rate(mode: Mode) -> f32 {
    K_INFO / mode.ch_bits_per_block() as f32
}
/// `Ldpc240_101` info-bit count — used to compute the per-mode
/// info rate (`K / ch_bits_per_block`).
const K_INFO: f32 = 101.0;
const SYMBOL_RATE_HZ: f32 = 1_200.0;
const BITS_PER_SYMBOL: f32 = 2.0;
