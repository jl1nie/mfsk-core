//! Q65-30A AP-decoding SNR sweep.
//!
//! Companion to `q65_snr_sweep.rs`: same channel and seed setup, but
//! also runs the decoder with an AP hint (`call1 = "CQ"`,
//! `call2 = "K1ABC"`) and prints the side-by-side success counts.
//! AP biasing is the dominant mechanism that lifts Q65 above the
//! plain-AWGN threshold; the WSJT-X documentation (Joe Taylor's
//! Q65 manual) cites a 2–4 dB improvement when call signs are
//! known a-priori.
//!
//! The test asserts a numerical floor — at the band where plain
//! AWGN decoding starts to fail, AP must still produce decodes —
//! to guard against regressions in the AP plumbing (mask packing,
//! `_q65_mask` masking step, decode-with-ap call site).

#![cfg(feature = "q65")]

use std::f32::consts::PI;

use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{Q65Decode, decode_scan_default, decode_scan_with_ap, synthesize_standard};

const FS: f32 = 12_000.0;
const FS_U: u32 = 12_000;
const REF_BW: f32 = 2_500.0;
const SLOT_SAMPLES: usize = 12_000 * 30;
const SEEDS: u64 = 8;

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

fn make_slot(
    call1: &str,
    call2: &str,
    grid: &str,
    freq_hz: f32,
    snr_db: f32,
    seed: u64,
) -> Vec<f32> {
    let snr_lin = 10_f32.powf(snr_db / 10.0);
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();
    let pcm = synthesize_standard(call1, call2, grid, FS_U, freq_hz, amp).expect("synth");

    let mut slot = vec![0.0_f32; SLOT_SAMPLES];
    let start = FS_U as usize;
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

fn hit(decodes: &[Q65Decode], expected: &str) -> bool {
    decodes.iter().any(|d| d.message == expected)
}

#[test]
#[ignore = "slow: SNR × seed sweep; minutes in debug, tens of seconds in release. CI runs via --include-ignored."]
fn q65_30a_ap_sweep_call1_call2() {
    let expected = "CQ K1ABC FN42";
    let freq = 1500.0;
    // The AP "client" knows it's a CQ from K1ABC but doesn't know
    // the grid — typical "responding to a CQ" scenario.
    let hint = ApHint::new().with_call1("CQ").with_call2("K1ABC");
    // Wider time tolerance than the default since we're inserting
    // at +1 s into the slot; matches q65_snr_sweep.rs.
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 5,
        score_threshold: 0.1,
        max_candidates: 8,
    };

    println!("\n=== Q65-30A AP sweep ({SEEDS} seeds/SNR) ===");
    println!("  SNR (dB)   plain     AP(CQ K1ABC)");
    println!("  --------   -----     ------------");

    let snrs = [-18, -20, -22, -24, -26, -28];
    let mut plain_results: Vec<(i32, u64)> = Vec::new();
    let mut ap_results: Vec<(i32, u64)> = Vec::new();

    let nominal_mid = (FS_U as usize) * 2;
    for snr in snrs {
        let mut ok_plain = 0u64;
        let mut ok_ap = 0u64;
        for seed in 0..SEEDS {
            let audio = make_slot("CQ", "K1ABC", "FN42", freq, snr as f32, 0xA9_0000 + seed);
            if hit(&decode_scan_default(&audio, FS_U), expected) {
                ok_plain += 1;
            }
            if hit(
                &decode_scan_with_ap(&audio, FS_U, nominal_mid, &params, &hint),
                expected,
            ) {
                ok_ap += 1;
            }
        }
        println!(
            "    {:>4}      {:>2}/{:<2}     {:>2}/{:<2}",
            snr, ok_plain, SEEDS, ok_ap, SEEDS
        );
        plain_results.push((snr, ok_plain));
        ap_results.push((snr, ok_ap));
    }

    // Sanity asserts. AP must improve or match plain at every SNR
    // (modulo seed noise — allow 1 fewer hit for AP to absorb
    // single-seed flips).
    for ((s, plain), (_, ap)) in plain_results.iter().zip(ap_results.iter()) {
        assert!(
            *ap as i64 + 1 >= *plain as i64,
            "AP regression at {s} dB: plain {plain}, AP {ap}"
        );
    }
    // At the threshold band (-24 dB) AP should give a clear lift —
    // require AP to produce at least one more decode than plain
    // somewhere in the {-22, -24, -26} window.
    let plain_total: u64 = plain_results
        .iter()
        .filter(|(s, _)| (-26..=-22).contains(s))
        .map(|(_, ok)| *ok)
        .sum();
    let ap_total: u64 = ap_results
        .iter()
        .filter(|(s, _)| (-26..=-22).contains(s))
        .map(|(_, ok)| *ok)
        .sum();
    assert!(
        ap_total > plain_total,
        "AP must lift threshold-band decode count above plain \
         (plain {plain_total} vs AP {ap_total} across -22..-26 dB)"
    );
}
