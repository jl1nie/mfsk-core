// SPDX-License-Identifier: GPL-3.0-only
//! The cells flow's QSO-context AP rung **in the board's build** (`fft-extern`, no `fft-rustfft`: the faithful ladder's blind-CQ step is not there), the sibling of
//! `mfsk-core/tests/cells_ap.rs` (which builds with `fft-rustfft` and so runs the blind-CQ pass before the rung; section 24 of `docs/notes/PI3_JT9_GAP.md`).
//!
//! **Fails if** a weak reply to the operator (`MyCall DxCall RR73`, -20 dB, AWGN, 12 seeds) is not decoded with the context far more often than without it; the context puts a message
//! with the locked calls out of 100 noise-only slots (more than one); or the reply is decoded with the partner put 200 Hz or 1.5 s away from it. (12 seeds and 100 noise slots: the test is the board build's smoke, `cells_ap.rs` has the sizes.)
//!
//! Run by package: `cargo test --release -p icap-app-hosttest --features fixed-point` (a workspace-wide run unifies `fft-rustfft` into mfsk-core and the test says so).
#![cfg(feature = "fixed-point")]

use icap_app_hosttest::{fft_shim, resample::resample};
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode_block::{
    ApContext, ApStage, CellTables, CellsOptions, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch,
    coarse_sync_gated_cells,
};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

const MYCALL: &str = "K1JT";
const PARTNER: &str = "HA0DU";
const F0: f32 = 1_500.0;

struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f32 {
        let (u, v) = (self.unit(), self.unit());
        ((-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()) as f32
    }
}

fn slot(clean: Option<&[i16]>, snr_db: f32, seed: u64) -> Vec<i16> {
    let amp = 6_000.0f32;
    let mut audio = vec![0f32; 180_000];
    if let Some(c) = clean {
        for (i, &v) in c.iter().enumerate() {
            audio[6_000 + i] = f32::from(v);
        }
    }
    let sigma = ((amp * amp / 2.0) / (10f32.powf(snr_db / 10.0) * 2500.0 / 6000.0)).sqrt();
    let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    for v in audio.iter_mut() {
        *v += sigma * rng.gauss();
    }
    audio
        .iter()
        .map(|v| v.round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

/// The window path (`p0_window`'s staircase and the capped OSD), then the partner's candidates, as the board would run them.
fn window(audio12: &[i16], ctx: Option<ApContext>) -> Vec<String> {
    let a = resample(audio12, 12_000, 8_000);
    let opts = CellsOptions::p0_window();
    let (cands, cells) = coarse_sync_gated_cells(
        &mut SliceSource(&a),
        Ft8Rate::Hz8000,
        &GatedSearch::new(200.0, 3000.0, 1.3, 30),
        1,
    )
    .expect("8 kHz");
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> = vec![[Cmplx::<f32>::default(); 8]; 79]
        .try_into()
        .expect("79 x 8");
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
        .with_options(&opts)
        .with_ap_context(ctx);
    let mut known = Vec::new();
    w.decode_round_osd(
        &cells,
        &cands,
        opts.max_cand,
        opts.depth(),
        0.25,
        &mut known,
        6,
    );
    w.decode_partner(&cells, opts.depth(), 0.25, &mut known);
    known
        .iter()
        .filter_map(|f| unpack77(f.res.message77()))
        .collect()
}

fn reply() -> Vec<i16> {
    let bits = pack77(MYCALL, PARTNER, "RR73").expect("pack");
    synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, F0, 6_000)
}

fn hits(clean: &[i16], ctx: Option<ApContext>) -> u32 {
    let msg = format!("{MYCALL} {PARTNER} RR73");
    (1..=12u64)
        .map(|seed| {
            u32::from(
                window(&slot(Some(clean), -20.0, seed), ctx)
                    .iter()
                    .any(|m| *m == msg),
            )
        })
        .sum()
}

#[test]
fn the_ap_rung_works_in_the_boards_build() {
    let clean = reply();
    let ctx = ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 1.0, 0.02);
    let (with, without) = (hits(&clean, ctx), hits(&clean, None));
    assert!(
        fft_shim::factory_calls() > 0,
        "built with fft-rustfft: this ran the host driver (run `-p icap-app-hosttest` alone)"
    );
    println!("board build, RR73 at -20 dB, 12 seeds: with the context {with}, without {without}");
    assert!(with >= 9, "with the context {with} of 12");
    assert!(without <= 2, "without {without} of 12");
    // the partner put far from where the reply is: the rung does not run on it
    for far in [
        ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 200.0, 0.02),
        ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 1.0, 1.5),
    ] {
        let n = hits(&clean, far);
        assert!(n <= 2, "{far:?}: {n} of 12");
    }
}

#[test]
fn the_ap_rung_puts_no_locked_message_out_of_noise_in_the_boards_build() {
    let mut n = 0;
    for stage in [ApStage::Rogers, ApStage::Replying] {
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0, 0.0);
        for seed in 1..=50u64 {
            for text in window(&slot(None, -20.0, 90_000 + seed), ctx) {
                if text.contains(MYCALL) || text.contains(PARTNER) {
                    n += 1;
                    eprintln!("noise {stage:?} {seed}: {text}");
                }
            }
        }
    }
    assert!(
        n <= 1,
        "{n} messages with the locked calls out of 100 noise slots"
    );
}
