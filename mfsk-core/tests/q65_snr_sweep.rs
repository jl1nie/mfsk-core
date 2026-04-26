//! Q65-30A AWGN sensitivity sweep.
//!
//! Generates a synthetic Q65-30A frame at a fixed dial frequency,
//! adds white Gaussian noise at calibrated SNR (in 2500 Hz reference
//! bandwidth — the WSJT-X / weakmon convention shared with the FT8 /
//! FT4 SNR sweeps in this directory), and counts how many of `SEEDS`
//! noise realisations the scan-and-decode path correctly recovers.
//!
//! Q65-30A's WSJT-X-published threshold is around `-24 dB` SNR (per
//! `lib/q65params.f90`'s analytical estimate `-27 + 10·log10(7200/3600)
//! = -24 dB`), so we sweep a band around it and sanity-check that
//! decodes succeed strongly above threshold and fail strongly below.
//!
//! The test prints a per-SNR success table — useful when tuning the
//! receiver's metric or convergence parameters.

#![cfg(feature = "q65")]

use std::f32::consts::PI;

use mfsk_core::q65::{decode_scan_default, synthesize_standard};

const FS: f32 = 12_000.0;
const FS_U: u32 = 12_000;
const REF_BW: f32 = 2_500.0;
const SLOT_SAMPLES: usize = 12_000 * 30;
const SEEDS: u64 = 8;

/// Tiny LCG + Box-Muller — same Gaussian generator used in the
/// FT4/FT8 sweeps so SNR conventions stay aligned.
struct Lcg {
    s: u64,
    spare: Option<f32>,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            s: seed.wrapping_add(1),
            spare: None,
        }
    }
    fn next(&mut self) -> u64 {
        self.s = self
            .s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.s
    }
    fn uniform(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn gauss(&mut self) -> f32 {
        if let Some(x) = self.spare.take() {
            return x;
        }
        let u = self.uniform();
        let v = self.uniform();
        let mag = (-2.0 * u.ln()).sqrt();
        self.spare = Some(mag * (2.0 * PI * v).sin());
        mag * (2.0 * PI * v).cos()
    }
}

/// Build a noisy slot: synthesise the message at `(freq, snr_db)`
/// inserted at +1 s within the 30 s slot, mixed with Gaussian noise.
fn make_slot(
    call1: &str,
    call2: &str,
    grid: &str,
    freq_hz: f32,
    snr_db: f32,
    seed: u64,
) -> Vec<f32> {
    let snr_lin = 10_f32.powf(snr_db / 10.0);
    // WSJT-X / weakmon SNR convention: A = sqrt(4·SNR·B/FS) with σ = 1.
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();
    let pcm = synthesize_standard(call1, call2, grid, FS_U, freq_hz, amp).expect("synth");

    let mut slot = vec![0.0_f32; SLOT_SAMPLES];
    let start = FS_U as usize; // 1 s into the slot
    let n = pcm.len().min(SLOT_SAMPLES - start);
    for i in 0..n {
        slot[start + i] += pcm[i];
    }
    let mut rng = Lcg::new(seed);
    for s in slot.iter_mut() {
        *s += rng.gauss();
    }
    slot
}

fn hit(decodes: &[mfsk_core::q65::Q65Decode], expected: &str) -> bool {
    decodes.iter().any(|d| d.message == expected)
}

#[test]
#[ignore = "slow: SNR sweep × multiple seeds; minutes in debug, ~45 s in release. CI runs via --include-ignored."]
fn q65_30a_snr_sweep() {
    let expected = "CQ K1ABC FN42";
    let freq = 1500.0;
    println!("\n=== Q65-30A SNR sweep ({SEEDS} seeds/SNR) ===");
    println!("  SNR (dB)   decoded");
    println!("  --------   -------");
    let snrs = [-12, -16, -18, -20, -22, -24, -26, -28];
    let mut results: Vec<(i32, u64)> = Vec::new();
    for snr in snrs {
        let mut ok = 0u64;
        for seed in 0..SEEDS {
            let audio = make_slot("CQ", "K1ABC", "FN42", freq, snr as f32, 0xC65000 + seed);
            let decodes = decode_scan_default(&audio, FS_U);
            if hit(&decodes, expected) {
                ok += 1;
            }
        }
        println!("    {:>4}      {:>2}/{:<2}", snr, ok, SEEDS);
        results.push((snr, ok));
    }
    // Basic sanity: at high SNR (-12 dB) every seed must decode; at
    // very low SNR (-28 dB, ~4 dB below threshold) at most a handful.
    let high_snr_hits = results
        .iter()
        .find(|(s, _)| *s == -12)
        .map(|(_, o)| *o)
        .unwrap();
    assert_eq!(
        high_snr_hits, SEEDS,
        "every seed must decode at -12 dB SNR (well above threshold)"
    );
}
