// SPDX-License-Identifier: GPL-3.0-only
//! The cells flow's AP rung (`CellsWorker::with_ap_context`, `decode_partner`; `docs/notes/PI3_JT9_GAP.md` sections 22-24): the operator's QSO locks the bits of the message the partner is
//! expected to send, on the candidates near the partner's frequency **and** DT.
//!
//! **Fails if** (1) a context that cannot apply (a stage with no pass, a partner far away in frequency or in DT) changes a decode of the recordings in any bit of the message, frequency,
//! DT or SNR (the rung or the partner's search leaks into the AP-less flow); (2) a weak reply to the operator (`MyCall DxCall RR73` at -20 dB in AWGN, 20 seeds) is not decoded with the
//! context far more often than without it (the lock's polarity, its bit positions, the masked BP or OSD, or the partner's candidates are wrong); (3) the context decodes a message with the
//! locked calls out of noise-only slots more than the bound (only a **gross** failure trips this one: 300 slots see 0-1 of the 3-4 in 6 000 that the DT window removes, so it does not
//! pin the window; the window is pinned by (4)); (4) the reply is decoded when the context puts the partner 200 Hz or 1.5 s away (the window of the rung is gone); (5) a context whose
//! calls are in none of the recordings, put at every place a decode sits in them (where the rung runs on real signals), decodes a message with those calls (the rung accepts a message
//! it should not: `max_extra` 0); (6) an AP decode leaves the blind OSD attempt of its candidate in the queue, or the partner's candidates leave theirs in it (they would take places of
//! the next slot's `osd_pass`).
#![cfg(all(feature = "fixed-point", feature = "internal-testing"))]
#[allow(dead_code)]
mod common;

use mfsk_core::Ft8;
use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::decode_block::{
    ApContext, ApStage, CellTables, CellsOptions, CellsWorker, Found, Ft8BpScratch, Ft8Rate,
    GatedSearch, coarse_sync_gated_cells,
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

/// An AWGN slot with the reply in it (`None`: noise only) at `snr_db` in 2 500 Hz, the reply starting 0.5 s in.
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

/// The cells flow's window (`p0_window`'s staircase and the capped OSD) over `audio` (12 kHz) with the context (`None`: none), then the partner's own candidates.
fn window(audio: &[i16], ctx: Option<ApContext>) -> Vec<Found> {
    let a = resample_to_8k(audio, 12_000);
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
}

fn keys(f: &[Found]) -> Vec<(String, u32, u32, u32)> {
    f.iter()
        .map(|x| {
            (
                unpack77(x.res.message77()).unwrap_or_default(),
                x.res.freq_hz.to_bits(),
                x.res.dt_sec.to_bits(),
                x.res.snr_db.to_bits(),
            )
        })
        .collect()
}

fn reply(field: &str) -> Vec<i16> {
    let bits = pack77(MYCALL, PARTNER, field).expect("pack");
    synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, F0, 6_000)
}

fn hits(clean: &[i16], snr: f32, seeds: u64, ctx: Option<ApContext>, msg: &str) -> u32 {
    (1..=seeds)
        .map(|seed| {
            let r = window(&slot(Some(clean), snr, seed), ctx);
            u32::from(keys(&r).iter().any(|k| k.0 == msg))
        })
        .sum()
}

fn assets(name: &str) -> Vec<i16> {
    common::load_wav_i16(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/").to_string() + name + ".wav",
    )
}

#[test]
fn a_context_that_cannot_apply_changes_no_decode() {
    for name in ["qso3_busy", "websdr_test4", "191111_110130"] {
        let a = assets(name);
        let base = keys(&window(&a, None));
        assert!(base.len() >= 5, "{name}: {} decodes", base.len());
        // no pass at this stage; the partner 800 Hz away; the partner 3 s off in DT (the window is 0.4 s): the rung and the partner's candidates do not run on anything
        for ctx in [
            ApContext::new(MYCALL, PARTNER, ApStage::Calling, 1500.0, 0.0),
            ApContext::new(MYCALL, PARTNER, ApStage::Rogers, 100.0, 0.0),
            ApContext::new(MYCALL, PARTNER, ApStage::Rogers, 1500.0, 3.0),
        ] {
            let with = keys(&window(&a, ctx));
            // the decodes the window path made are all still there, unchanged; a context that finds nothing adds nothing
            assert_eq!(with, base, "{name} {ctx:?}");
        }
    }
}

#[test]
fn a_weak_reply_is_decoded_with_the_context_and_not_without() {
    let clean = reply("RR73");
    let msg = format!("{MYCALL} {PARTNER} RR73");
    let ctx = ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 1.0, 0.02);
    let (with, without) = (
        hits(&clean, -20.0, 20, ctx, &msg),
        hits(&clean, -20.0, 20, None, &msg),
    );
    // measured on 100 seeds: 94 with, 2 without
    assert!(with >= 15, "with the context {with} of 20");
    assert!(without <= 3, "without {without} of 20");
}

#[test]
fn the_rung_runs_only_near_the_partner() {
    let clean = reply("RR73");
    let msg = format!("{MYCALL} {PARTNER} RR73");
    for (label, ctx) in [
        (
            "200 Hz away",
            ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 200.0, 0.02),
        ),
        (
            "1.5 s away",
            ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 1.0, 1.5),
        ),
    ] {
        let n = hits(&clean, -20.0, 20, ctx, &msg);
        assert!(n <= 3, "{label}: {n} of 20");
    }
}

#[test]
fn the_context_adds_no_message_with_the_locked_calls_out_of_noise() {
    // 300 noise-only slots for each stage. Measured over 6 000 slots each (after the DT window): see section 24; a false accept is a message with the locked calls, which noise has no
    // business to produce. The bound is the measured count (0 in this size) plus one, so that the ceiling, the lock check and the window are what keep it there.
    let mut n = 0;
    for stage in [ApStage::Rogers, ApStage::Replying] {
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0, 0.0);
        for seed in 1..=300u64 {
            for f in window(&slot(None, -20.0, 90_000 + seed), ctx) {
                let text = unpack77(f.res.message77()).unwrap_or_default();
                if text.contains(MYCALL) || text.contains(PARTNER) {
                    n += 1;
                    eprintln!(
                        "noise {stage:?} {seed}: {text} ({} errors)",
                        f.res.hard_errors
                    );
                }
            }
        }
    }
    assert!(
        n <= 1,
        "{n} messages with the locked calls out of 600 noise slots"
    );
}

/// The decodes of the AP-less window at `(f, dt)` are where the rung runs on real signals: a context whose calls are in none of the recordings must decode nothing with them there.
#[test]
fn a_context_with_calls_nobody_has_decodes_nothing_on_real_recordings() {
    let (mine, his) = ("JA1ZZZ", "VK3YYY");
    let mut places = 0;
    for name in ["qso3_busy", "websdr_test4", "websdr_test7", "191111_110630"] {
        let a = assets(name);
        let base = window(&a, None);
        for found in &base {
            for stage in [ApStage::Rogers, ApStage::Replying] {
                let ctx = ApContext::new(mine, his, stage, found.res.freq_hz, found.res.dt_sec);
                places += 1;
                for f in window(&a, ctx) {
                    let text = unpack77(f.res.message77()).unwrap_or_default();
                    assert!(
                        !text.contains(mine) && !text.contains(his),
                        "{name} {stage:?} at {} Hz: {text}",
                        found.res.freq_hz
                    );
                }
            }
        }
    }
    assert!(places >= 60, "{places} places: the guard would be thin");
}

/// An AP decode is a decode: the blind OSD attempt its candidate queued must not stay in the queue (`OsdMode::Defer`), and the partner's own candidates queue none: one place fewer than
/// without the context (the reply's candidate), never more.
#[test]
fn an_ap_decode_and_the_partners_candidates_leave_nothing_in_the_osd_queue() {
    let clean = reply("RR73");
    let queue = |audio: &[i16], ctx: Option<ApContext>| -> usize {
        let a = resample_to_8k(audio, 12_000);
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
        // the round alone (the OSD deferred, not run), then the partner's candidates: what is left queued
        w.decode_round(
            &cells,
            &cands,
            opts.max_cand,
            opts.depth(),
            0.25,
            &mut known,
        );
        w.decode_partner(&cells, opts.depth(), 0.25, &mut known);
        w.take_osd_attempts().len()
    };
    let ctx = ApContext::new(MYCALL, PARTNER, ApStage::Rogers, F0 + 1.0, 0.02);
    let (mut fewer, mut more) = (0, 0);
    for seed in 1..=10u64 {
        let audio = slot(Some(&clean), -20.0, seed);
        let (without, with) = (queue(&audio, None), queue(&audio, ctx));
        fewer += usize::from(with < without);
        more += usize::from(with > without);
    }
    assert_eq!(
        more, 0,
        "the context queued an attempt that there was none for"
    );
    assert!(
        fewer >= 7,
        "the reply's candidate stayed queued in {} of 10",
        10 - fewer
    );
    // a reply too weak for the AP: the partner's candidates fail, and must still queue nothing of their own (they would run against the next slot's cells)
    for seed in 1..=10u64 {
        let audio = slot(Some(&clean), -27.0, seed);
        let (without, with) = (queue(&audio, None), queue(&audio, ctx));
        assert!(
            with <= without,
            "seed {seed}: {with} queued with the context, {without} without"
        );
    }
}
