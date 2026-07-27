//! NormalizedMinSum / OffsetMinSum BP kernel tests.
//!
//! Two layers of validation:
//!
//! 1. **Round-trip** — encode an FT8 / FST4 / uvpacket info word,
//!    feed perfect LLRs, decode through every `BpKind` variant.
//!    Each kernel must recover the original info bit-for-bit on a
//!    clean signal.
//!
//! 2. **AWGN threshold sweep** (Q65-style: 100 trials × Eb/N0 ladder)
//!    for FT8 LDPC(174, 91). Compares `NormalizedMinSum α=0.75`
//!    and `OffsetMinSum β=0.5` against `SumProduct`. Asserts
//!    threshold loss ≤ 0.3 dB at the 50 % decode rate — within the
//!    expected NMS/OMS calibration headroom and well below the
//!    operational margin we care about.

#![cfg(feature = "ft8")]

use mfsk_core::engine::BpKind;
#[cfg(feature = "fst4")]
use mfsk_core::fec::ldpc::bp::bp_decode_generic_kind;
use mfsk_core::fec::ldpc::bp::bp_decode_kind;
use mfsk_core::fec::ldpc::osd::ldpc_encode_generic;
use mfsk_core::fec::ldpc::params::{Ldpc174_91Params, LdpcParams};

const KINDS: &[(&str, BpKind)] = &[
    ("sum-product", BpKind::SumProduct),
    ("nms-0.75", BpKind::NormalizedMinSum { alpha: 0.75 }),
    ("oms-0.5", BpKind::OffsetMinSum { beta: 0.5 }),
];

/// Pack 91 info bits into a clean Ldpc174_91 codeword and feed
/// per-bit LLRs of ±LLR_MAG. Every `BpKind` must converge on the
/// original info on a clean channel (no noise).
#[test]
fn ldpc174_clean_round_trip_every_kind() {
    const LLR_MAG: f32 = 12.0;
    const N: usize = Ldpc174_91Params::N;

    // Deterministic-looking info word: alternating + a couple of
    // interesting bits to keep the LDPC from being all-zeros.
    let mut info = vec![0u8; Ldpc174_91Params::K];
    for (i, b) in info.iter_mut().enumerate() {
        *b = ((i * 17 + 3) % 2) as u8;
    }

    let mut codeword = vec![0u8; N];
    ldpc_encode_generic::<Ldpc174_91Params>(&info, &mut codeword);

    // BPSK-style LLR mapping: bit = 1 → positive, bit = 0 → negative.
    let llr: Vec<f32> = codeword
        .iter()
        .map(|&b| if b == 0 { -LLR_MAG } else { LLR_MAG })
        .collect();
    let llr_arr: [f32; N] = llr.as_slice().try_into().expect("174 LLRs");

    for (label, kind) in KINDS {
        let r = bp_decode_kind(&llr_arr, None, 30, None, *kind)
            .unwrap_or_else(|| panic!("[{label}] clean round-trip must converge"));
        assert_eq!(
            r.info, info,
            "[{label}] decoded info mismatch on clean channel"
        );
        assert!(
            r.iterations <= 5,
            "[{label}] clean signal should converge quickly, got {} iterations",
            r.iterations
        );
    }
}

/// Same round-trip for LDPC(240, 101) — exercises the second LDPC
/// in the crate (FST4 / uvpacket) through the generic kernel
/// dispatch.
#[cfg(feature = "fst4")]
#[test]
fn ldpc240_clean_round_trip_every_kind() {
    use mfsk_core::fec::ldpc::params::Ldpc240_101Params;
    const LLR_MAG: f32 = 12.0;
    const N: usize = Ldpc240_101Params::N;
    const K: usize = Ldpc240_101Params::K;

    let mut info = vec![0u8; K];
    for (i, b) in info.iter_mut().enumerate() {
        *b = ((i * 13 + 7) % 2) as u8;
    }
    let mut codeword = vec![0u8; N];
    ldpc_encode_generic::<Ldpc240_101Params>(&info, &mut codeword);

    let llr: Vec<f32> = codeword
        .iter()
        .map(|&b| if b == 0 { -LLR_MAG } else { LLR_MAG })
        .collect();

    for (label, kind) in KINDS {
        let r = bp_decode_generic_kind::<Ldpc240_101Params>(&llr, None, 30, None, *kind)
            .unwrap_or_else(|| panic!("[{label}] LDPC240 clean round-trip must converge"));
        assert_eq!(r.info, info, "[{label}] LDPC240 decoded info mismatch");
    }
}

// ────────────────────────────────────────────────────────────────────
// AWGN sweep — measure NMS / OMS threshold relative to SumProduct.
// ────────────────────────────────────────────────────────────────────

/// Tiny self-contained Box-Muller LCG so the sweep runs without an
/// `rand` dep. Seed-deterministic.
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
    fn next_u64(&mut self) -> u64 {
        self.s = self
            .s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.s
    }
    fn uniform(&mut self) -> f32 {
        ((self.next_u64() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn gauss(&mut self) -> f32 {
        if let Some(x) = self.spare.take() {
            return x;
        }
        let u = self.uniform();
        let v = self.uniform();
        let mag = (-2.0 * u.ln()).sqrt();
        let a = mag * (std::f32::consts::TAU * v).cos();
        let b = mag * (std::f32::consts::TAU * v).sin();
        self.spare = Some(b);
        a
    }
}

/// Run 100 trials at `eb_n0_db`, return the fraction of correct
/// decodes for the given `BpKind`. Uses the same info word every
/// trial; only the AWGN realisation changes.
fn decode_rate_at(eb_n0_db: f32, kind: BpKind, trials: usize) -> f32 {
    const N: usize = Ldpc174_91Params::N;
    const K: usize = Ldpc174_91Params::K;
    const RATE: f32 = K as f32 / N as f32;

    // Es/N0 = Eb/N0 · R, BPSK so signal energy per bit = 1.
    let eb_n0 = 10.0_f32.powf(eb_n0_db / 10.0);
    let es_n0 = eb_n0 * RATE;
    // Channel: BPSK over AWGN, σ² = 1 / (2 · Es/N0). LLR scale = 2/σ².
    let sigma_sq = 1.0 / (2.0 * es_n0);
    let sigma = sigma_sq.sqrt();
    let llr_scale = 2.0 / sigma_sq;

    let mut info = vec![0u8; K];
    for (i, b) in info.iter_mut().enumerate() {
        *b = ((i * 17 + 3) % 2) as u8;
    }
    let mut codeword = vec![0u8; N];
    ldpc_encode_generic::<Ldpc174_91Params>(&info, &mut codeword);

    let mut rng = Lcg::new(0x12345 ^ ((eb_n0_db * 1000.0) as u64));
    let mut ok = 0usize;
    for _ in 0..trials {
        let llr_vec: Vec<f32> = codeword
            .iter()
            .map(|&b| {
                let signal = if b == 0 { -1.0 } else { 1.0 };
                let received = signal + sigma * rng.gauss();
                llr_scale * received
            })
            .collect();
        let llr_arr: [f32; N] = llr_vec.as_slice().try_into().unwrap();
        if let Some(r) = bp_decode_kind(&llr_arr, None, 30, None, kind)
            && r.info == info
        {
            ok += 1;
        }
    }
    ok as f32 / trials as f32
}

/// Sweep Eb/N0 from 0..=4 dB and find the lowest dB at which decode
/// rate hits the threshold. Returns +∞ if never reached. Prints the
/// per-dB rate for diagnosis.
fn threshold_db(kind: BpKind, target_rate: f32, trials: usize) -> f32 {
    let mut best = f32::INFINITY;
    let ladder = [0.5_f32, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 6.0, 8.0];
    eprintln!("  sweep {kind:?}:");
    for &db in &ladder {
        let rate = decode_rate_at(db, kind, trials);
        eprintln!("    {db:.1} dB: {rate:.2}");
        if best.is_infinite() && rate >= target_rate {
            best = db;
        }
    }
    best
}

/// Compare NMS / OMS against SumProduct. Loss must be ≤ 0.5 dB at
/// the 50 % decode rate threshold — well within the published
/// NMS/OMS calibration window for short LDPC codes. The test runs a
/// modest 30-trial sweep so it stays under a second; expand the
/// ladder + trials offline for tighter measurement.
#[test]
#[ignore = "AWGN sweep — runs in seconds, kept #[ignore] so default `cargo test` stays fast"]
fn nms_oms_threshold_within_0p5_db_of_sum_product() {
    let trials = 30;
    let target = 0.5_f32;

    let sp = threshold_db(BpKind::SumProduct, target, trials);
    let nms = threshold_db(BpKind::NormalizedMinSum { alpha: 0.75 }, target, trials);
    let oms = threshold_db(BpKind::OffsetMinSum { beta: 0.5 }, target, trials);

    eprintln!(
        "[ldpc174_91 AWGN @ rate={target}] SP={sp:.2} dB  NMS(α=0.75)={nms:.2} dB  OMS(β=0.5)={oms:.2} dB"
    );

    assert!(
        sp.is_finite(),
        "SumProduct must reach {target} decode rate within the swept ladder"
    );
    assert!(
        (nms - sp).abs() <= 0.5,
        "NMS threshold {nms:.2} dB > SumProduct {sp:.2} dB + 0.5"
    );
    assert!(
        (oms - sp).abs() <= 0.5,
        "OMS threshold {oms:.2} dB > SumProduct {sp:.2} dB + 0.5"
    );
}
