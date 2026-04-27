//! Per-slot decode-time budget measurement — the figure relevant for WASM
//! where the browser gives us only single-threaded compute within each
//! 7.5 s FT4 slot.
//!
//! Measures `decode_sniper_ap` wall-clock at representative SNR points.
//! The bench harness (ft8-bench) uses rayon parallel processing which
//! hides single-thread cost; this test isolates it.

use std::time::Instant;

use mfsk_core::core::equalize::EqMode;
use mfsk_core::core::{MessageCodec, MessageFields};
use mfsk_core::ft4::decode::{ApHint, decode_sniper_ap};
use mfsk_core::ft4::encode;

const FS: f32 = 12_000.0;
const REF_BW: f32 = 2_500.0;
const SLOT: usize = 90_000;

struct Lcg {
    s: u64,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            s: seed.wrapping_add(1),
        }
    }
    fn next(&mut self) -> u64 {
        self.s = self
            .s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.s
    }
    fn u(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn g(&mut self) -> f32 {
        let u = self.u();
        let v = self.u();
        (-2.0 * u.ln()).sqrt() * (2.0 * std::f32::consts::PI * v).cos()
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

fn make_slot(msg: &[u8; 77], snr_db: f32, seed: u64) -> Vec<i16> {
    let snr_lin = 10f32.powf(snr_db / 10.0);
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();
    let itone = encode::message_to_tones(msg);
    let pcm = encode::tones_to_f32(&itone, 1000.0, amp);
    let mut mix = vec![0.0f32; SLOT];
    let start = (0.5 * FS) as usize;
    for i in 0..pcm.len().min(SLOT - start) {
        mix[start + i] += pcm[i];
    }
    let mut rng = Lcg::new(seed);
    for s in mix.iter_mut() {
        *s += rng.g();
    }
    let peak = mix.iter().map(|x| x.abs()).fold(0.0f32, f32::max).max(1e-6);
    let scale = 29_000.0 / peak;
    mix.iter()
        .map(|&s| (s * scale).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

#[test]
#[ignore]
fn ft4_sniper_ap_wallclock() {
    let msg = pack_cq("JA1ABC", "PM95");
    let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");

    eprintln!("\nFT4 decode_sniper_ap single-call wall-clock (3 seeds/SNR):");
    eprintln!("  FT4 slot length = 7.5 s (budget for WASM single-thread)");
    eprintln!("  (EQ=Adaptive, max_cand=30, DecodeDepth=BpAllOsd)");
    eprintln!("   SNR    avg      min      max    status");

    for snr in [-4, -10, -14, -16, -18] {
        let mut times = Vec::new();
        let mut decoded = 0;
        for seed in 0..3u64 {
            let audio = make_slot(&msg, snr as f32, 0xBEEF + seed);
            let t0 = Instant::now();
            let results = decode_sniper_ap(&audio, 1000.0, 30, EqMode::Adaptive, Some(&ap));
            let dt = t0.elapsed();
            times.push(dt);
            if results.iter().any(|r| r.message77() == msg) {
                decoded += 1;
            }
        }
        let avg_ms = times.iter().map(|d| d.as_secs_f32() * 1000.0).sum::<f32>() / 3.0;
        let min_ms = times
            .iter()
            .map(|d| d.as_secs_f32() * 1000.0)
            .fold(f32::INFINITY, f32::min);
        let max_ms = times
            .iter()
            .map(|d| d.as_secs_f32() * 1000.0)
            .fold(0.0f32, f32::max);
        eprintln!(
            "  {:>4} dB  {:>6.0} ms  {:>6.0} ms  {:>6.0} ms   decoded {}/3",
            snr, avg_ms, min_ms, max_ms, decoded
        );
    }
}
