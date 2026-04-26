//! Diagnostic: what fails at -16 dB? For a single seed, report
//! per-candidate: sync_quality, refine drift, BP result.
//!
//! Marked `#[ignore]` so `cargo test` stays fast — run explicitly with
//! `cargo test -p ft4-core --release --test diag_low_snr -- --ignored --nocapture`.

use std::f32::consts::PI;

use mfsk_core::core::dsp::downsample::{build_fft_cache, downsample_cached};
use mfsk_core::core::llr::{compute_llr, symbol_spectra, sync_quality};
use mfsk_core::core::sync::{coarse_sync, refine_candidate};
use mfsk_core::core::{
    FecCodec, FecOpts, FrameLayout, MessageCodec, MessageFields, ModulationParams, Protocol,
};
use mfsk_core::ft4::{Ft4, encode};

const FS: f32 = 12_000.0;
const REF_BW: f32 = 2_500.0;
const SLOT: usize = 90_000;

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
    fn u(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn g(&mut self) -> f32 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let u = self.u();
        let v = self.u();
        let m = (-2.0 * u.ln()).sqrt();
        self.spare = Some(m * (2.0 * PI * v).sin());
        m * (2.0 * PI * v).cos()
    }
}

fn make_slot(msg: &[u8; 77], snr_db: f32, seed: u64) -> Vec<i16> {
    let snr_lin = 10f32.powf(snr_db / 10.0);
    let amp = (2.0 * snr_lin * REF_BW / FS).sqrt();
    let itone = encode::message_to_tones(msg);
    let pcm = encode::tones_to_f32(&itone, 1000.0, amp);
    let mut mix = vec![0.0f32; SLOT];
    let start = (0.5 * FS) as usize;
    let n = pcm.len().min(SLOT - start);
    for i in 0..n {
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
fn why_does_neg16db_fail() {
    let codec = mfsk_core::msg::Wsjt77Message;
    let msg = {
        let bits = codec
            .pack(&MessageFields {
                call1: Some("CQ".into()),
                call2: Some("JA1ABC".into()),
                grid: Some("PM95".into()),
                ..Default::default()
            })
            .unwrap();
        let mut out = [0u8; 77];
        out.copy_from_slice(&bits);
        out
    };
    let ap = mfsk_core::msg::ApHint::new()
        .with_call1("CQ")
        .with_call2("JA1ABC");

    let audio = make_slot(&msg, -16.0, 0xCAFE);
    let cands = coarse_sync::<Ft4>(&audio, 800.0, 1200.0, 0.3, None, 200);
    eprintln!("\n-16 dB signal: {} coarse candidates", cands.len());

    // Find the truth-proximate candidate
    let truth = cands
        .iter()
        .enumerate()
        .find(|(_, c)| (c.freq_hz - 1000.0).abs() < 15.0 && c.dt_sec.abs() < 0.1);
    match truth {
        Some((i, c)) => eprintln!(
            "  truth-like at rank {i}: freq={:.1} dt={:+.3} score={:.3}",
            c.freq_hz, c.dt_sec, c.score
        ),
        None => {
            eprintln!("  NO truth-proximate candidate in top 200");
            return;
        }
    }
    let (_, truth_cand) = truth.unwrap();

    let ds_rate = 12_000.0 / <Ft4 as ModulationParams>::NDOWN as f32;
    let fft_cache = build_fft_cache(&audio, &mfsk_core::ft4::decode::FT4_DOWNSAMPLE);
    let cd0 = downsample_cached(
        &fft_cache,
        truth_cand.freq_hz,
        &mfsk_core::ft4::decode::FT4_DOWNSAMPLE,
    );

    for &steps in &[32i32, 48, 64] {
        let refined = refine_candidate::<Ft4>(&cd0, truth_cand, steps);
        let i0 =
            ((refined.dt_sec + <Ft4 as FrameLayout>::TX_START_OFFSET_S) * ds_rate).round() as usize;
        let cs = symbol_spectra::<Ft4>(&cd0, i0);
        let nsync = sync_quality::<Ft4>(&cs);
        eprintln!(
            "  refine steps={:3}  dt_sec={:+.4}  i0={:4}  sync_q={}/16",
            steps, refined.dt_sec, i0, nsync
        );

        // Try BP with AP on this timing
        let llr_set = compute_llr::<Ft4>(&cs);
        let (mask, values) = ap.build_bits(<Ft4 as Protocol>::Fec::N);
        let fec = <Ft4 as Protocol>::Fec::default();
        let opts = FecOpts {
            bp_max_iter: 30,
            osd_depth: 3,
            ap_mask: Some((&mask, &values)),
            verify_info: Some(<<Ft4 as Protocol>::Msg as MessageCodec>::verify_info),
        };
        for (name, llr) in [
            ("a", &llr_set.llra),
            ("b", &llr_set.llrb),
            ("c", &llr_set.llrc),
            ("d", &llr_set.llrd),
        ] {
            let res = fec.decode_soft(llr, &opts);
            match res {
                Some(r) if r.info[..77] == msg[..] => {
                    eprintln!("    llr{name}: OK (hard_errors={})", r.hard_errors);
                }
                Some(r) => eprintln!(
                    "    llr{name}: WRONG message (hard_errors={})",
                    r.hard_errors
                ),
                None => eprintln!("    llr{name}: FAIL"),
            }
        }
    }
}
