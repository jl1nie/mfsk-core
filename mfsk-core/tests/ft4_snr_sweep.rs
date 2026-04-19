//! FT4 threshold characterisation — synthetic SNR sweep.
//!
//! Isolated from the ft8-bench super-run so iteration on tuning is cheap.
//! Deliberately small seed counts (10/row instead of 20) to keep CI time
//! bounded; the bench-grade version in ft8-bench uses 20.

use std::f32::consts::PI;

use mfsk_core::core::equalize::EqMode;
use mfsk_core::core::{MessageCodec, MessageFields};
use mfsk_core::ft4::decode::{ApHint, DecodeResult, decode_frame, decode_sniper_ap};
use mfsk_core::ft4::encode;

const FS: f32 = 12_000.0;
const REF_BW: f32 = 2_500.0;
const SLOT: usize = 90_000;
const SEEDS: u64 = 10;

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

fn pack_cq(call: &str, grid: &str) -> [u8; 77] {
    let codec = mfsk_core::msg::Wsjt77Message;
    let bits = codec
        .pack(&MessageFields {
            call1: Some("CQ".into()),
            call2: Some(call.into()),
            grid: Some(grid.into()),
            ..Default::default()
        })
        .unwrap();
    let mut out = [0u8; 77];
    out.copy_from_slice(&bits);
    out
}

fn make_slot(msg77: &[u8; 77], freq_hz: f32, snr_db: f32, seed: u64) -> Vec<i16> {
    let mut mix = vec![0.0f32; SLOT];
    let snr_lin = 10f32.powf(snr_db / 10.0);
    // WSJT-X / weakmon SNR convention: A = sqrt(4·SNR·B/FS) with σ_noise = 1.
    // Earlier sweeps used `sqrt(2·…)` which produced signals 3 dB weaker than
    // the requested SNR — reinterpret old labels as +3 dB of the true SNR.
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();
    let itone = encode::message_to_tones(msg77);
    let pcm = encode::tones_to_f32(&itone, freq_hz, amp);
    let start = (0.5 * FS) as usize;
    let n = pcm.len().min(SLOT - start);
    for i in 0..n {
        mix[start + i] += pcm[i];
    }
    let mut rng = Lcg::new(seed);
    for s in mix.iter_mut() {
        *s += rng.gauss();
    }
    let peak = mix.iter().map(|x| x.abs()).fold(0.0f32, f32::max).max(1e-6);
    let scale = 29_000.0 / peak;
    mix.iter()
        .map(|&s| (s * scale).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

fn hit(results: &[DecodeResult], truth: &[u8; 77]) -> bool {
    results.iter().any(|r| &r.message77 == truth)
}

#[test]
fn ft4_snr_sweep_basic_vs_ap() {
    let msg = pack_cq("JA1ABC", "PM95");
    let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");

    println!("\n=== FT4 SNR sweep ({SEEDS} seeds/SNR) ===");
    println!("  SNR    basic    AP(EQ=Adaptive)");

    for snr in [-4, -6, -8, -10, -12, -14, -16, -18] {
        let mut ok_b = 0;
        let mut ok_a = 0;
        for seed in 0..SEEDS {
            let audio = make_slot(&msg, 1000.0, snr as f32, 0xF70000 + seed);
            if hit(&decode_frame(&audio, 800.0, 1200.0, 1.2, 50), &msg) {
                ok_b += 1;
            }
            if hit(
                &decode_sniper_ap(&audio, 1000.0, 30, EqMode::Adaptive, Some(&ap)),
                &msg,
            ) {
                ok_a += 1;
            }
        }
        println!(
            "  {:>3} dB   {:>3}/{}     {:>3}/{}",
            snr, ok_b, SEEDS, ok_a, SEEDS
        );
    }
}
