// SPDX-License-Identifier: GPL-3.0-only
//! The QSO-context AP at the reply window, measured (`docs/notes/PI3_JT9_GAP.md` section 22): a weak reply to the operator in an AWGN band, decoded by
//!
//! - `cells`: the cells flow's `p0_late` as the board runs it (no AP for a reply: the blind-CQ rung only takes `CQ`),
//! - `blind`: the faithful `Decoder<Ft8>` at `Depth::Deep`, AP off,
//! - `ap`: the same with the QSO context and the Rx frequency (the ceiling: the faithful pipeline's own search),
//! - `c1` / `c9`: `decode_candidates` of **only the partner's frequency** (one candidate, or a 3 x 3 grid of 3.125 Hz and 40 ms round it) with the QSO-context AP: the window hybrid.
//!
//! ```text
//! cargo run --release -p mfsk-core --features full,fixed-point,internal-testing --example ap_qso -- [trials] [phantom-slots]
//! ```
//! stdout: per scenario and SNR the hits of `trials` and the mean ms of each decoder; then the phantoms: noise-only slots with the context set.

use std::time::Instant;

use mfsk_core::Ft8;
use mfsk_core::decoder::{
    ApMode, DecodeParams, Decoder, Depth, Ft8Candidate, Ft8Extras, Ft8Strategy, QsoProgress,
    SlotInput,
};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::msg::wsjt77::pack77;

const MYCALL: &str = "K1JT";
const PARTNER: &str = "HA0DU";
const F0: f32 = 1_500.0;
const START: usize = 6_000;

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

fn slot(clean: Option<&[i16]>, amp: f32, snr_db: f32, seed: u64) -> Vec<i16> {
    let mut audio = vec![0f32; 180_000];
    if let Some(c) = clean {
        for (i, &v) in c.iter().enumerate() {
            if START + i < audio.len() {
                audio[START + i] = f32::from(v);
            }
        }
    }
    // SNR in 2500 Hz: (A^2 / 2) / (sigma^2 * 2500 / 6000)
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

fn ctx_params(progress: QsoProgress, ap: ApMode) -> DecodeParams {
    DecodeParams::for_band((200.0, 3000.0))
        .depth(Depth::Deep)
        .ap(ap)
        .station(MYCALL, "FN20")
        // `AP_QSO_NOGRID=1`: no grid for the partner, so the a8 list decoder (it needs both calls, a grid and the Rx frequency) does not run and the AP passes are alone
        .qso(
            PARTNER,
            if std::env::var_os("AP_QSO_NOGRID").is_some() {
                ""
            } else {
                "KN07"
            },
            progress,
        )
        .rx_freq(F0)
}

fn extras(strategy: Option<Ft8Strategy>) -> Ft8Extras {
    let mut e = Ft8Extras::default();
    e.tuning.sync_min = Some(1.3);
    e.tuning.max_cand = Some(50);
    e.tuning.strategy = strategy;
    e
}

#[cfg(feature = "fixed-point")]
fn cells(pcm: &[i16]) -> Vec<String> {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::ft8::decode_block::{
        CellsOptions, Ft8Rate, GatedSearch, decode_block_cells_with,
    };
    use mfsk_core::msg::wsjt77::unpack77;
    let a = resample_to_8k(pcm, 12_000);
    decode_block_cells_with(
        &a,
        Ft8Rate::Hz8000,
        &GatedSearch::new(200.0, 3000.0, 1.3, 30),
        &CellsOptions::p0_late(),
    )
    .expect("8 kHz")
    .iter()
    .filter_map(|d| unpack77(d.message77()))
    .map(|m| m.split_whitespace().collect::<Vec<_>>().join(" "))
    .collect()
}
#[cfg(not(feature = "fixed-point"))]
fn cells(_: &[i16]) -> Vec<String> {
    Vec::new()
}

fn texts(r: mfsk_core::decoder::SlotResult<mfsk_core::engine::frame::DecodeResult>) -> Vec<String> {
    r.rows.into_iter().map(|r| r.decoded.text).collect()
}

/// A flat Rayleigh fade on the reply (about 1 Hz of Doppler spread, unit mean power) in AWGN: the SNR is the average one.
fn faded_slot(clean: &[i16], amp: f32, snr_db: f32, seed: u64) -> Vec<i16> {
    let mut rng = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
    // a complex gain at 100 Hz, one-pole low-passed (1 Hz), then |g|, normalised to unit mean power
    let a = (-2.0 * std::f32::consts::PI * 1.0 / 100.0).exp();
    let (mut gr, mut gi) = (0f32, 0f32);
    let mut g = Vec::with_capacity(1_900);
    for _ in 0..400 {
        gr = a * gr + (1.0 - a) * rng.gauss();
        gi = a * gi + (1.0 - a) * rng.gauss();
    }
    for _ in 0..1_900 {
        gr = a * gr + (1.0 - a) * rng.gauss();
        gi = a * gi + (1.0 - a) * rng.gauss();
        g.push((gr * gr + gi * gi).sqrt());
    }
    let mean_sq = g.iter().map(|x| x * x).sum::<f32>() / g.len() as f32;
    let norm = 1.0 / mean_sq.sqrt();
    let faded: Vec<i16> = clean
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let t = (START + i) as f32 / 120.0; // 100 Hz at 12 kHz
            let k = (t as usize).min(g.len() - 2);
            let f = t - k as f32;
            let e = (g[k] * (1.0 - f) + g[k + 1] * f) * norm;
            (f32::from(v) * e).round().clamp(-32768.0, 32767.0) as i16
        })
        .collect();
    slot(Some(&faded), amp, snr_db, seed)
}

fn load_wav(path: &str) -> Vec<i16> {
    let w = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    w[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .take(180_000)
        .map(|b| i16::from_le_bytes(*b))
        .collect()
}

/// The reply added into a real recording at `f0`, at `snr_db` on the decoder's own scale: `snr0` is what the faithful decode reports for the reply at amplitude `a0`, and the
/// level is linear in the amplitude.
fn into_recording(
    rec: &[i16],
    clean: &[i16],
    a0: f32,
    snr0: f32,
    snr_db: f32,
    start: usize,
) -> Vec<i16> {
    let gain = 10f32.powf((snr_db - snr0) / 20.0);
    let mut out: Vec<i16> = rec.to_vec();
    let _ = a0;
    for (i, &v) in clean.iter().enumerate() {
        if start + i < out.len() {
            let x = f32::from(out[start + i]) + gain * f32::from(v);
            out[start + i] = x.round().clamp(-32768.0, 32767.0) as i16;
        }
    }
    out
}

/// A frequency of the recording with nothing decoded within 40 Hz, in 800-2400 Hz.
fn free_frequency(rec: &[i16]) -> f32 {
    let rows = Decoder::<Ft8>::new(
        DecodeParams::for_band((200.0, 3000.0))
            .depth(Depth::Deep)
            .ap(ApMode::Off),
    )
    .with_extras(extras(None))
    .decode(&SlotInput::i16(rec))
    .rows;
    let busy: Vec<f32> = rows.iter().map(|r| r.decoded.freq_hz).collect();
    let mut best = (0f32, 1_500.0f32);
    let mut f = 800.0f32;
    while f <= 2_400.0 {
        let d = busy.iter().map(|b| (b - f).abs()).fold(f32::MAX, f32::min);
        if d > best.0 {
            best = (d, f);
        }
        f += 12.5;
    }
    best.1
}

/// Real recordings as the background: the reply goes at a free frequency, at two start times, so a cell is the recordings x 2 trials.
fn busy(paths: Vec<String>) {
    let scenarios: [(&str, &str, QsoProgress); 3] = [
        ("Rogers   RR73", "RR73", QsoProgress::Rogers),
        ("Report   -15 ", "-15", QsoProgress::Replying),
        ("Calling  grid", "KN07", QsoProgress::Calling),
    ];
    let recs: Vec<Vec<i16>> = paths.iter().map(|p| load_wav(p)).collect();
    let fs: Vec<f32> = recs.iter().map(|r| free_frequency(r)).collect();
    println!(
        "busy bands: {} recordings, free frequencies {:?}",
        recs.len(),
        fs
    );
    println!(
        "hits of {} trials (mean ms) for cells | blind | ap | c1 | c9",
        recs.len() * 2
    );
    for (label, field, progress) in scenarios {
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let bits = pack77(MYCALL, PARTNER, field).expect("pack");
        let clean_at = |f0: f32, amp: i16| {
            synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, f0, amp)
        };
        for snr in [-16.0f32, -18.0, -20.0, -22.0, -24.0] {
            let mut hits = [0u32; 6];
            let mut ms = [0f64; 6];
            let mut trials = 0u32;
            for (rec, &f0) in recs.iter().zip(&fs) {
                // the decoder's own reading of the reply at a level it can decode and that is not saturated (-14..+2 dB): the ruler
                let mut ruler = None;
                for a in [100i16, 200, 400, 800, 1_600, 3_200] {
                    let probe =
                        into_recording(rec, &clean_at(f0, a), f32::from(a), 0.0, 0.0, START);
                    let snr = Decoder::<Ft8>::new(
                        DecodeParams::for_band((200.0, 3000.0))
                            .depth(Depth::Deep)
                            .ap(ApMode::Off),
                    )
                    .with_extras(extras(None))
                    .decode(&SlotInput::i16(&probe))
                    .rows
                    .iter()
                    .find(|r| r.decoded.text == msg)
                    .map(|r| r.decoded.snr_db);
                    if let Some(x) = snr.filter(|x| (-14.0..=2.0).contains(x)) {
                        ruler = Some((a, x));
                        break;
                    }
                }
                let Some((a0, snr0)) = ruler else {
                    eprintln!("no ruler for {f0} Hz");
                    continue;
                };
                for start in [START, START + 2_400] {
                    let pcm =
                        into_recording(rec, &clean_at(f0, a0), f32::from(a0), snr0, snr, start);
                    trials += 1;
                    let params = |ap| {
                        DecodeParams::for_band((200.0, 3000.0))
                            .depth(Depth::Deep)
                            .ap(ap)
                            .station(MYCALL, "FN20")
                            .qso(PARTNER, "", progress)
                            .rx_freq(f0)
                    };
                    let dt = (start as f32 / 12_000.0) - 0.5;
                    let mut run = |i: usize, f: &mut dyn FnMut() -> Vec<String>| {
                        let t = Instant::now();
                        let r = f();
                        ms[i] += t.elapsed().as_secs_f64() * 1e3;
                        hits[i] += u32::from(r.iter().any(|m| m == &msg));
                    };
                    run(0, &mut || cells(&pcm));
                    run(1, &mut || {
                        texts(
                            Decoder::<Ft8>::new(params(ApMode::Off))
                                .with_extras(extras(None))
                                .decode(&SlotInput::i16(&pcm)),
                        )
                    });
                    run(2, &mut || {
                        texts(
                            Decoder::<Ft8>::new(params(ApMode::Full))
                                .with_extras(extras(None))
                                .decode(&SlotInput::i16(&pcm)),
                        )
                    });
                    let c1 = [Ft8Candidate::new(f0 + 1.0, dt + 0.02)];
                    let g9: Vec<Ft8Candidate> = [-3.125f32, 0.0, 3.125]
                        .iter()
                        .flat_map(|df| {
                            [-0.04f32, 0.0, 0.04]
                                .iter()
                                .map(move |d| Ft8Candidate::new(f0 + df, dt + d))
                        })
                        .collect();
                    let cand = |l: &[Ft8Candidate]| {
                        texts(
                            Decoder::<Ft8>::new(params(ApMode::Full))
                                .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                                .decode_candidates(&SlotInput::i16(&pcm), l)
                                .expect("single pass"),
                        )
                    };
                    run(3, &mut || cand(&c1));
                    run(4, &mut || cand(&g9));
                    run(5, &mut || {
                        texts(
                            Decoder::<Ft8>::new(params(ApMode::Off))
                                .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                                .decode_candidates(&SlotInput::i16(&pcm), &c1)
                                .expect("single pass"),
                        )
                    });
                }
            }
            let n = trials.max(1) as f64;
            println!(
                "{label} {snr:6.1} dB ({trials}): {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1})",
                hits[0],
                ms[0] / n,
                hits[1],
                ms[1] / n,
                hits[2],
                ms[2] / n,
                hits[3],
                ms[3] / n,
                hits[4],
                ms[4] / n,
                hits[5],
                ms[5] / n
            );
        }
    }
}

/// The false accepts of the context's locks: noise-only slots, and real recordings with calls nobody in them has (any decode with those calls is a phantom).
fn phantoms(noise_slots: u64, paths: Vec<String>) {
    let amp = 6_000.0f32;
    let grid9: Vec<Ft8Candidate> = [-3.125f32, 0.0, 3.125]
        .iter()
        .flat_map(|df| {
            [-0.04f32, 0.0, 0.04]
                .iter()
                .map(move |d| Ft8Candidate::new(F0 + df, *d))
        })
        .collect();
    let mut found = Vec::new();
    for progress in [
        QsoProgress::Rogers,
        QsoProgress::Replying,
        QsoProgress::Calling,
    ] {
        let mut n = 0u32;
        for seed in 1..=noise_slots {
            let pcm = slot(None, amp, -20.0, 50_000 + seed);
            let r = Decoder::<Ft8>::new(ctx_params(progress, ApMode::Full))
                .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                .decode_candidates(&SlotInput::i16(&pcm), &grid9)
                .expect("single pass");
            for row in r.rows {
                n += 1;
                found.push(format!(
                    "noise {progress:?}: {} (hard errors {:?}, pass {:?})",
                    row.decoded.text, row.detail.hard_errors, row.detail.pass
                ));
            }
        }
        println!("noise-only, {progress:?}: {noise_slots} slots, {n} decodes");
    }
    // real recordings: other people's calls locked, five Rx frequencies a recording
    let (mine, his) = ("JA1ZZZ", "VK3YYY");
    let mut n_rec = 0u32;
    let mut n_ph = 0u32;
    for path in &paths {
        let rec = load_wav(path);
        for progress in [
            QsoProgress::Rogers,
            QsoProgress::Replying,
            QsoProgress::Calling,
        ] {
            for rx in [500.0f32, 900.0, 1_300.0, 1_700.0, 2_100.0] {
                let params = DecodeParams::for_band((200.0, 3000.0))
                    .depth(Depth::Deep)
                    .ap(ApMode::Full)
                    .station(mine, "PM95")
                    .qso(his, "", progress)
                    .rx_freq(rx);
                let list: Vec<Ft8Candidate> = [-3.125f32, 0.0, 3.125]
                    .iter()
                    .flat_map(|df| {
                        [-0.04f32, 0.0, 0.04, 0.5, 1.0]
                            .iter()
                            .map(move |d| Ft8Candidate::new(rx + df, *d))
                    })
                    .collect();
                let r = Decoder::<Ft8>::new(params)
                    .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                    .decode_candidates(&SlotInput::i16(&rec), &list)
                    .expect("single pass");
                n_rec += 1;
                for row in r.rows {
                    if row.decoded.text.contains(mine) || row.decoded.text.contains(his) {
                        n_ph += 1;
                        found.push(format!("{path} {progress:?} rx {rx}: {}", row.decoded.text));
                    }
                }
            }
        }
    }
    println!(
        "real recordings, other people's calls: {n_rec} decodes of a recording x progress x Rx frequency (15 candidates each), {n_ph} with the locked calls"
    );
    for f in found {
        println!("PHANTOM {f}");
    }
}

/// The hard errors of the true decodes at the partner's frequency, by reply and SNR: where a limit on them can sit between these and the noise's.
fn errors(trials: u64) {
    for (label, field, progress) in [
        ("Rogers   RR73", "RR73", QsoProgress::Rogers),
        ("Report   -15 ", "-15", QsoProgress::Replying),
        ("Calling  grid", "KN07", QsoProgress::Calling),
    ] {
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let bits = pack77(MYCALL, PARTNER, field).expect("pack");
        let clean = synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, F0, 6_000);
        for snr in [-18.0f32, -20.0, -22.0, -24.0] {
            let mut errs: Vec<u32> = Vec::new();
            for seed in 1..=trials {
                let pcm = slot(Some(&clean), 6_000.0, snr, 7_000 + seed);
                let r = Decoder::<Ft8>::new(ctx_params(progress, ApMode::Full))
                    .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                    .decode_candidates(&SlotInput::i16(&pcm), &[Ft8Candidate::new(F0 + 1.0, 0.02)])
                    .expect("single pass");
                for row in r.rows.iter().filter(|r| r.decoded.text == msg) {
                    errs.push(row.detail.hard_errors.unwrap_or(999));
                }
            }
            errs.sort_unstable();
            println!(
                "{label} {snr:6.1} dB: {} of {trials} decoded, hard errors {:?}",
                errs.len(),
                errs
            );
        }
    }
}

/// The cells flow's window (`p0_window`'s staircase and the capped OSD) with and without the QSO context: the AP rung of the cells flow and the partner's own candidates
/// (`CellsWorker::with_ap_context`, `decode_partner`). `None`: no context.
#[cfg(feature = "fixed-point")]
fn cells_ap(
    pcm: &[i16],
    ctx: Option<mfsk_core::ft8::decode_block::ApContext>,
) -> Vec<(String, u32)> {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::scalar::Cmplx;
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::ft8::decode_block::{
        CellTables, CellsOptions, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch,
        coarse_sync_gated_cells,
    };
    use mfsk_core::msg::wsjt77::unpack77;
    let a = resample_to_8k(pcm, 12_000);
    let opts = CellsOptions::p0_window();
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let (cands, cells) =
        coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1).expect("8 kHz");
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
        .filter_map(|f| {
            unpack77(f.res.message77()).map(|m| {
                (
                    m.split_whitespace().collect::<Vec<_>>().join(" "),
                    f.res.hard_errors,
                )
            })
        })
        .collect()
}

/// `cellsap`: AWGN band, the reply at the partner's frequency: the cells window without the context, with it, and the false accepts (noise-only slots with it).
#[cfg(feature = "fixed-point")]
fn cellsap(trials: u64, phantom_slots: u64) {
    use mfsk_core::ft8::decode_block::{ApContext, ApStage};
    let amp = 6_000.0f32;
    println!(
        "cells window: hits of {trials} (mean ms) without the context | with it (+ the partner's candidates) | hard errors of its decodes"
    );
    for (label, field, stage) in [
        ("Rogers   RR73", "RR73", ApStage::Rogers),
        ("Report   -15 ", "-15", ApStage::Replying),
        ("Calling  grid", "KN07", ApStage::Calling),
    ] {
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let bits = pack77(MYCALL, PARTNER, field).expect("pack");
        let clean = synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, F0, amp as i16);
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0 + 1.0, 0.02);
        for snr in [-18.0f32, -20.0, -22.0, -24.0] {
            let (mut h0, mut h1) = (0u32, 0u32);
            let (mut m0, mut m1) = (0f64, 0f64);
            let mut errs: Vec<u32> = Vec::new();
            for seed in 1..=trials {
                let pcm = slot(Some(&clean), amp, snr, seed);
                let t = Instant::now();
                let r0 = cells_ap(&pcm, None);
                m0 += t.elapsed().as_secs_f64() * 1e3;
                let t = Instant::now();
                let r1 = cells_ap(&pcm, ctx);
                m1 += t.elapsed().as_secs_f64() * 1e3;
                h0 += u32::from(r0.iter().any(|m| m.0 == msg));
                h1 += u32::from(r1.iter().any(|m| m.0 == msg));
                for m in r1.iter().filter(|m| m.0 == msg) {
                    errs.push(m.1);
                }
                for m in r1.iter().filter(|m| m.0 != msg) {
                    println!(
                        "PHANTOM cells ctx {label} {snr}: {} (hard errors {})",
                        m.0, m.1
                    );
                }
            }
            let n = trials as f64;
            errs.sort_unstable();
            println!(
                "{label} {snr:6.1} dB: {h0:2} ({:5.1}) | {h1:2} ({:5.1}) | hard errors {errs:?}",
                m0 / n,
                m1 / n
            );
        }
    }
    let mut count = 0u32;
    for stage in [ApStage::Rogers, ApStage::Replying] {
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0, 0.0);
        let mut n = 0u32;
        for seed in 1..=phantom_slots {
            let pcm = slot(None, amp, -20.0, 90_000 + seed);
            for m in cells_ap(&pcm, ctx) {
                n += 1;
                println!(
                    "PHANTOM cells noise {stage:?}: {} (hard errors {})",
                    m.0, m.1
                );
            }
        }
        println!("cells, noise-only, {stage:?}: {phantom_slots} slots, {n} decodes");
        count += n;
    }
    let _ = count;
}
#[cfg(not(feature = "fixed-point"))]
fn cellsap(_: u64, _: u64) {}

/// `a8cells`: a noncoherent list decision on the cells (research, PI3_JT9_GAP.md 26): the hypotheses of the stage (the replies a8 would try), scored by the share of each
/// symbol's energy on the expected tone, summed over the 79 symbols, the best over places around the partner (+-3 bins, +-5 rows). The threshold is the largest score of
/// `cal` noise slots (it is measured, the cells are not a8's iid exponentials); then the false accepts in `fa` other noise slots, and the hits against the AP passes on the same slots.
#[cfg(feature = "fixed-point")]
fn a8cells(trials: u64, cal: u64, fa: u64) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::ft8::decode_block::{
        ApContext, ApStage, Ft8Rate, GatedSearch, HalfDbCells, coarse_sync_gated_cells,
    };
    let amp = 6_000.0f32;
    let pow_of = |h: u8| 10f32.powf(f32::from(h) / 20.0);
    let tones_of = |field: &str| -> [u8; 79] {
        let bits = pack77(MYCALL, PARTNER, field).expect("pack");
        let t = message_to_tones::<Ft8>(&bits);
        let mut a = [0u8; 79];
        a.copy_from_slice(&t[..79]);
        a
    };
    let rogers: Vec<(String, [u8; 79])> = ["RRR", "RR73", "73"]
        .iter()
        .map(|f| (f.to_string(), tones_of(f)))
        .collect();
    let mut reports: Vec<(String, [u8; 79])> = Vec::new();
    for r in -30..=32i32 {
        for pre in ["", "R"] {
            let f = format!("{pre}{r:+03}");
            reports.push((f.clone(), tones_of(&f)));
        }
    }
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let cells_of = |pcm: &[i16]| -> HalfDbCells {
        let a = resample_to_8k(pcm, 12_000);
        coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1)
            .expect("8 kHz")
            .1
    };
    // `A8_FAST=<top>,<dk>,<dj>`: the places (+-dk bins, +-dj rows) are ranked on the 21 Costas symbols (they do not depend on the hypothesis) and only the `top` best are
    // scored against the hypotheses, from a table of the energy shares made once for the place. Without it: every place, every hypothesis (the first run).
    let fast: Option<(usize, i64, i32)> = std::env::var("A8_FAST").ok().map(|v| {
        let x: Vec<i64> = v.split(',').map(|n| n.parse().unwrap()).collect();
        (x[0] as usize, x[1], x[2] as i32)
    });
    let ptab: Vec<f32> = (0..256u32).map(|h| pow_of(h as u8)).collect();
    let costas_syms: Vec<usize> = (0..7).chain(36..43).chain(72..79).collect();
    let best_ns = std::cell::Cell::new((0u128, 0u64));
    let last_veto = std::cell::Cell::new(0u32);
    let best_at = |cells: &HalfDbCells,
                   list: &[(String, [u8; 79])],
                   fc: f32,
                   dtc: f32|
     -> [(f32, usize); 2] {
        let t_start = Instant::now();
        let bin_hz = 3.125f32;
        let tstep = 0.08f32;
        let k0 = (fc / bin_hz).round() as i64;
        let j00 = ((dtc + 0.5) / tstep).round() as i32;
        let (top, kr, jr) = fast.unwrap_or((usize::MAX, 3, 5));
        let spectra =
            |k: i64, j0: i32, syms: &mut dyn Iterator<Item = usize>| -> Vec<(usize, [f32; 8])> {
                syms.map(|sym| {
                    let mut row = [0f32; 8];
                    for (t, v) in row.iter_mut().enumerate() {
                        let (r, b) = (j0 + 2 * sym as i32, k + 2 * t as i64);
                        if r >= 0 && b >= 0 {
                            *v = cells
                                .cell(r as usize, b as usize)
                                .map_or(0.0, |h| ptab[h as usize]);
                        }
                    }
                    (sym, row)
                })
                .collect()
            };
        // rank the places
        let mut places: Vec<(f32, i64, i32)> = Vec::new();
        for dk in -kr..=kr {
            for dj in -jr..=jr {
                let (k, j0) = (k0 + dk, j00 + dj);
                let sc = if fast.is_some() {
                    spectra(k, j0, &mut costas_syms.iter().copied())
                        .iter()
                        .map(|(sym, row)| {
                            row[list[0].1[*sym] as usize] / (row.iter().sum::<f32>() + 1e-30)
                        })
                        .sum::<f32>()
                } else {
                    0.0
                };
                places.push((sc, k, j0));
            }
        }
        if fast.is_some() {
            places.sort_by(|a, b| b.0.total_cmp(&a.0));
            places.truncate(top);
        }
        let mut out = [(f32::MIN, 0usize); 2];
        for &(_, k, j0) in &places {
            let p = spectra(k, j0, &mut (0..79));
            // the share and the log-likelihood-ratio term of every (symbol, tone), once for the place
            let mut sh = [[0f32; 8]; 79];
            let mut ll = [[0f32; 8]; 79];
            for (sym, row) in &p {
                let tot: f32 = row.iter().sum::<f32>() + 1e-30;
                for t in 0..8 {
                    sh[*sym][t] = row[t] / tot;
                    ll[*sym][t] = (1.0 + 7.0 * row[t] / (tot - row[t] + 1e-30)).ln();
                }
            }
            for (hi, (_, tones)) in list.iter().enumerate() {
                let (mut s1, mut s2) = (0f32, 0f32);
                for sym in 0..79 {
                    s1 += sh[sym][tones[sym] as usize];
                    s2 += ll[sym][tones[sym] as usize];
                }
                if s1 > out[0].0 {
                    out[0] = (s1, hi);
                }
                if s2 > out[1].0 {
                    out[1] = (s2, hi);
                    // the symbols that say another tone, plainly: a data symbol whose strongest tone has >= 0.7 of the energy and is not the hypothesis's
                    let mut veto = 0u32;
                    for sym in (7..36).chain(43..72) {
                        let (mut mx, mut at) = (0f32, 0usize);
                        for t in 0..8 {
                            if sh[sym][t] > mx {
                                (mx, at) = (sh[sym][t], t);
                            }
                        }
                        if mx >= 0.7 && at != tones[sym] as usize {
                            veto += 1;
                        }
                    }
                    last_veto.set(veto);
                }
            }
        }
        let (a, n) = best_ns.get();
        best_ns.set((a + t_start.elapsed().as_nanos(), n + 1));
        out
    };
    let best =
        |cells: &HalfDbCells, list: &[(String, [u8; 79])]| best_at(cells, list, F0 + 1.0, 0.02);
    if let Ok(v) = std::env::var("A8_INQSO") {
        // the in-QSO case: every signal the cells window decodes on the recordings, taken as a partner (`C1 C2 X`: me = C1, the partner = C2), the list built from the pair and the
        // decision made at the signal's own place. The third field X decides what the list is for it: not in the list (the partner sent something else) = every acceptance is false;
        // in the list = the right hypothesis must win.
        use mfsk_core::engine::dsp::resample::resample_to_8k as rs8;
        use mfsk_core::engine::scalar::Cmplx;
        use mfsk_core::ft8::decode_block::{CellTables, CellsOptions, CellsWorker, Ft8BpScratch};
        use mfsk_core::msg::wsjt77::unpack77;
        let mut sigs: Vec<(String, String, String, f32, f32, usize)> = Vec::new(); // (c1, c2, x, f, dt, recording)
        let mut cells_all: Vec<HalfDbCells> = Vec::new();
        for (ri, path) in v.split(',').enumerate() {
            let pcm = load_wav(path);
            let a = rs8(&pcm, 12_000);
            let opts = CellsOptions::p0_window();
            let (cands, cells) =
                coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1)
                    .expect("8 kHz");
            let tables = CellTables::new();
            let mut cs: Box<[[Cmplx<f32>; 8]; 79]> = vec![[Cmplx::<f32>::default(); 8]; 79]
                .try_into()
                .expect("79 x 8");
            let mut bp = Box::new(Ft8BpScratch::new());
            let mut w = CellsWorker::new(&mut cs, &mut bp, &tables).with_options(&opts);
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
            for f in &known {
                if let Some(t) = unpack77(f.res.message77()) {
                    let w: Vec<&str> = t.split_whitespace().collect();
                    if w.len() == 3
                        && pack77(w[0], w[1], w[2]).is_some()
                        && !w[0].starts_with("CQ")
                        && !w[0].starts_with('<')
                        && !w[1].starts_with('<')
                    {
                        sigs.push((
                            w[0].into(),
                            w[1].into(),
                            w[2].into(),
                            f.res.freq_hz,
                            f.res.dt_sec,
                            ri,
                        ));
                    }
                }
            }
            cells_all.push(cells);
        }
        println!(
            "{} signals decoded on {} recordings that a list can be built for",
            sigs.len(),
            cells_all.len()
        );
        let is_rogers = |x: &str| matches!(x, "RRR" | "RR73" | "73");
        let is_report = |x: &str| {
            let y = x.strip_prefix('R').unwrap_or(x);
            y.len() >= 2
                && (y.starts_with('+') || y.starts_with('-'))
                && y[1..].chars().all(|c| c.is_ascii_digit())
        };
        let kinds: [(&str, f32, &dyn Fn(&str) -> bool); 2] = [
            ("Rogers", 74.66 + 1.0, &is_rogers),
            ("Report", 78.17 + 1.0, &is_report),
        ];
        for (name, thr, listed) in kinds {
            let (mut n_un, mut acc, mut acc_v3, mut acc_v1) = (0u32, 0u32, 0u32, 0u32);
            let (mut n_li, mut right, mut right_v3, mut wrong) = (0u32, 0u32, 0u32, 0u32);
            for (c1, c2, x, f, dt, ri) in &sigs {
                let mut list: Vec<(String, [u8; 79])> = Vec::new();
                let mk = |field: &str| -> Option<(String, [u8; 79])> {
                    let bits = pack77(c1, c2, field)?;
                    let t = message_to_tones::<Ft8>(&bits);
                    let mut a = [0u8; 79];
                    a.copy_from_slice(&t[..79]);
                    Some((field.to_string(), a))
                };
                if name == "Rogers" {
                    list.extend(["RRR", "RR73", "73"].iter().filter_map(|f| mk(f)));
                } else {
                    for r in -30..=32i32 {
                        for pre in ["", "R"] {
                            list.extend(mk(&format!("{pre}{r:+03}")));
                        }
                    }
                }
                if list.is_empty() {
                    continue;
                }
                let b = best_at(&cells_all[*ri], &list, *f, *dt)[1];
                let veto = last_veto.get();
                if listed(x) {
                    n_li += 1;
                    if b.0 > thr {
                        if list[b.1].0 == *x {
                            right += 1;
                            right_v3 += u32::from(veto < 3);
                        } else {
                            wrong += 1;
                            println!(
                                "  WRONG hypothesis {name}: {c1} {c2} {x} -> {} score {:.1} veto {veto}",
                                list[b.1].0, b.0
                            );
                        }
                    }
                } else {
                    n_un += 1;
                    if b.0 > thr {
                        acc += 1;
                        acc_v3 += u32::from(veto < 3);
                        acc_v1 += u32::from(veto < 2);
                        if veto < 3 {
                            println!(
                                "  FALSE accept {name}: {c1} {c2} {x} -> {} score {:.1} veto {veto}",
                                list[b.1].0, b.0
                            );
                        }
                    }
                }
            }
            println!(
                "{name} list: signals whose message is NOT in the list {n_un}: over the AWGN threshold {acc}, and veto<3 {acc_v3}, veto<2 {acc_v1}; signals whose message IS in the list {n_li}: right hypothesis over the threshold {right} (veto<3: {right_v3}), wrong hypothesis {wrong}"
            );
        }
        return;
    }
    if let Ok(v) = std::env::var("A8_REC") {
        // recorded slots: the partner's place slid over the band (every 12.5 Hz, three DTs) on each recording; the hypotheses carry calls that are not on the air, so any score
        // over the AWGN threshold is a false accept of a real band
        let mut qs: Vec<(String, Vec<f32>)> =
            vec![("Rogers".into(), vec![]), ("Report".into(), vec![])];
        let mut nrec = 0;
        let mut vetoes: [Vec<u32>; 2] = [vec![], vec![]];
        for path in v.split(',') {
            let pcm = load_wav(path);
            let cells = cells_of(&pcm);
            nrec += 1;
            for (li, list) in [&rogers, &reports].into_iter().enumerate() {
                let mut f = 300.0f32;
                while f < 2_800.0 {
                    for dtc in [0.02f32, 0.9, 1.8] {
                        let sc = best_at(&cells, list, f, dtc)[1].0;
                        qs[li].1.push(sc);
                        let thr = if li == 0 { 74.66 + 1.0 } else { 78.17 + 1.0 };
                        if sc > thr {
                            if last_veto.get() <= 3 {
                                println!(
                                    "  low veto: {path} {} f {f} dt {dtc} score {sc:.1} veto {}",
                                    ["Rogers", "Report"][li],
                                    last_veto.get()
                                );
                            }
                            vetoes[li].push(last_veto.get());
                        }
                    }
                    f += 12.5;
                }
            }
        }
        for (li, vs) in vetoes.iter().enumerate() {
            let mut h = [0u32; 8];
            for &x in vs {
                h[(x as usize / 2).min(7)] += 1;
            }
            println!(
                "{}: vetoes of the places over the threshold, by 2s (0-1, 2-3, ..., 14+): {h:?}",
                ["Rogers", "Report"][li]
            );
        }
        for (name, v) in &mut qs {
            v.sort_by(|a, b| a.total_cmp(b));
            let n = v.len();
            let thr = if name == "Rogers" {
                74.66 + 1.0
            } else {
                78.17 + 1.0
            };
            println!(
                "{name}: {nrec} recordings, {n} places: median {:.2}, 99th {:.2}, 99.9th {:.2}, max {:.2}; over the AWGN threshold ({thr:.2}): {}",
                v[n / 2],
                v[n * 99 / 100],
                v[n * 999 / 1000],
                v[n - 1],
                v.iter().filter(|x| **x > thr).count()
            );
        }
        return;
    }
    println!(
        "noncoherent list decision on the cells ({} Rogers hypotheses, {} report hypotheses)",
        rogers.len(),
        reports.len()
    );
    for (label, field, stage, list) in [
        ("Rogers RR73", "RR73", ApStage::Rogers, &rogers),
        ("Report -15 ", "-15", ApStage::Replying, &reports),
    ] {
        if let Ok(v) = std::env::var("A8_DUMP") {
            // raw scores for an offline threshold study: `A8_DUMP=<seed0>` (noise seeds seed0+1..=seed0+cal, detection seeds seed0+1..=seed0+trials)
            let seed0: u64 = v.parse().unwrap();
            let truth = list
                .iter()
                .position(|h| h.0 == field)
                .expect("truth in list");
            let clean = synthesize_i16::<Ft8>(
                &message_to_tones::<Ft8>(&pack77(MYCALL, PARTNER, field).expect("pack")),
                12_000,
                F0,
                amp as i16,
            );
            for seed in 1..=cal {
                let b = best(
                    &cells_of(&slot(None, amp, -20.0, 5_000_000 + seed0 + seed)),
                    list,
                );
                println!("N {label} {:.3} {:.3} v{}", b[0].0, b[1].0, last_veto.get());
            }
            for snr in [-22.0f32, -23.0, -24.0, -25.0, -26.0, -27.0] {
                for seed in 1..=trials {
                    let pcm = slot(Some(&clean), amp, snr, seed0 + seed);
                    let b = best(&cells_of(&pcm), list);
                    println!(
                        "D {label} {snr} {:.3} {:.3} {} {} v{}",
                        b[0].0,
                        b[1].0,
                        u8::from(b[0].1 == truth),
                        u8::from(b[1].1 == truth),
                        last_veto.get()
                    );
                }
            }
            continue;
        }
        let mut thr = [f32::MIN; 2];
        for seed in 1..=cal {
            let b = best(&cells_of(&slot(None, amp, -20.0, 500_000 + seed)), list);
            for i in 0..2 {
                thr[i] = thr[i].max(b[i].0);
            }
        }
        let mut fas = [0u32; 2];
        for seed in 1..=fa {
            let b = best(&cells_of(&slot(None, amp, -20.0, 900_000 + seed)), list);
            for i in 0..2 {
                fas[i] += u32::from(b[i].0 > thr[i]);
            }
        }
        println!(
            "{label}: threshold (max of {cal} noise slots) S1 {:.2} S2 {:.2}; false accepts in {fa} other noise slots: S1 {} S2 {}",
            thr[0], thr[1], fas[0], fas[1]
        );
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let truth = list
            .iter()
            .position(|h| h.0 == field)
            .expect("truth in list");
        let clean = synthesize_i16::<Ft8>(
            &message_to_tones::<Ft8>(&pack77(MYCALL, PARTNER, field).expect("pack")),
            12_000,
            F0,
            amp as i16,
        );
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0 + 1.0, 0.02);
        for snr in [-18.0f32, -20.0, -22.0, -24.0, -26.0] {
            let (mut hit_s, mut hit_ap, mut hit_blind) = ([0u32; 2], 0u32, 0u32);
            for seed in 1..=trials {
                let pcm = slot(Some(&clean), amp, snr, seed);
                let b = best(&cells_of(&pcm), list);
                for i in 0..2 {
                    hit_s[i] += u32::from(b[i].0 > thr[i] && b[i].1 == truth);
                }
                hit_ap += u32::from(cells_ap(&pcm, ctx).iter().any(|m| m.0 == msg));
                hit_blind += u32::from(cells_ap(&pcm, None).iter().any(|m| m.0 == msg));
            }
            println!(
                "{label} {snr:6.1} dB: of {trials}: blind {hit_blind:3} | AP passes {hit_ap:3} | list S1 {:3} S2 {:3}",
                hit_s[0], hit_s[1]
            );
        }
    }
    let (a, n) = best_ns.get();
    println!(
        "the list decision alone: {:.2} ms a call on average over {n} calls (host)",
        a as f64 / n as f64 / 1e6
    );
}
#[cfg(not(feature = "fixed-point"))]
fn a8cells(_: u64, _: u64, _: u64) {}

fn main() {
    let mut args = std::env::args().skip(1).peekable();
    match args.peek().map(String::as_str) {
        Some("busy") => {
            args.next();
            busy(args.collect());
        }
        Some("fading") => {
            args.next();
            awgn(&mut args, true);
        }
        Some("a8cells") => {
            args.next();
            let t = args.next().map_or(20, |x| x.parse().unwrap());
            let c = args.next().map_or(300, |x| x.parse().unwrap());
            a8cells(t, c, args.next().map_or(300, |x| x.parse().unwrap()));
        }
        Some("cellsap") => {
            args.next();
            let t = args.next().map_or(10, |x| x.parse().unwrap());
            cellsap(t, args.next().map_or(100, |x| x.parse().unwrap()));
        }
        Some("errors") => {
            args.next();
            errors(args.next().map_or(200, |x| x.parse().unwrap()));
        }
        Some("phantoms") => {
            args.next();
            let n: u64 = args.next().expect("noise slots").parse().unwrap();
            phantoms(n, args.collect());
        }
        _ => awgn(&mut args, false),
    }
}

fn awgn(args: &mut impl Iterator<Item = String>, fading: bool) {
    let trials: u64 = args.next().map_or(10, |x| x.parse().unwrap());
    let phantom_slots: u64 = args.next().map_or(100, |x| x.parse().unwrap());
    let amp = 6_000.0f32;
    // (label, the reply's text, the call, the QSO progress that expects it)
    let scenarios: [(&str, &str, &str, QsoProgress); 3] = [
        ("Rogers   RR73", "RR73", "", QsoProgress::Rogers),
        ("Report   -15 ", "-15", "", QsoProgress::Replying),
        ("Calling  grid", "KN07", "", QsoProgress::Calling),
    ];
    let grid9: Vec<Ft8Candidate> = [-3.125f32, 0.0, 3.125]
        .iter()
        .flat_map(|df| {
            [-0.04f32, 0.0, 0.04]
                .iter()
                .map(move |ddt| Ft8Candidate::new(F0 + df, 0.0 + ddt))
        })
        .collect();
    // the partner's place as the QSO knows it: a little off (1 Hz, 20 ms)
    let c1 = [Ft8Candidate::new(F0 + 1.0, 0.02)];
    println!(
        "trials {trials} per cell; SNR in 2500 Hz; hits of {trials} (mean ms) for cells | blind | ap | c1 | c9"
    );
    for (label, field, _, progress) in scenarios {
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let bits = pack77(MYCALL, PARTNER, field).expect("pack");
        let clean = synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, F0, amp as i16);
        for snr in [-16.0f32, -18.0, -20.0, -22.0, -24.0, -26.0] {
            let mut hits = [0u32; 5];
            let mut ms = [0f64; 5];
            for seed in 1..=trials {
                let pcm = if fading {
                    faded_slot(&clean, amp, snr, seed)
                } else {
                    slot(Some(&clean), amp, snr, seed)
                };
                let t = Instant::now();
                let r0 = cells(&pcm);
                ms[0] += t.elapsed().as_secs_f64() * 1e3;
                let t = Instant::now();
                let r1 = texts(
                    Decoder::<Ft8>::new(ctx_params(progress, ApMode::Off))
                        .with_extras(extras(None))
                        .decode(&SlotInput::i16(&pcm)),
                );
                ms[1] += t.elapsed().as_secs_f64() * 1e3;
                let t = Instant::now();
                let r2 = texts(
                    Decoder::<Ft8>::new(ctx_params(progress, ApMode::Full))
                        .with_extras(extras(None))
                        .decode(&SlotInput::i16(&pcm)),
                );
                ms[2] += t.elapsed().as_secs_f64() * 1e3;
                let cand = |list: &[Ft8Candidate]| {
                    texts(
                        Decoder::<Ft8>::new(ctx_params(progress, ApMode::Full))
                            .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                            .decode_candidates(&SlotInput::i16(&pcm), list)
                            .expect("single pass"),
                    )
                };
                let t = Instant::now();
                let r3 = cand(&c1);
                ms[3] += t.elapsed().as_secs_f64() * 1e3;
                let t = Instant::now();
                let r4 = cand(&grid9);
                ms[4] += t.elapsed().as_secs_f64() * 1e3;
                for (i, r) in [r0, r1, r2, r3, r4].iter().enumerate() {
                    hits[i] += u32::from(r.iter().any(|m| m == &msg));
                    // a decode that is not the reply is a phantom (the band holds nothing else)
                    for m in r.iter().filter(|m| *m != &msg) {
                        println!("PHANTOM decoder {i} {label} {snr}: {m}");
                    }
                }
            }
            let n = trials as f64;
            println!(
                "{label} {snr:6.1} dB: {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1}) | {:2} ({:5.1})",
                hits[0],
                ms[0] / n,
                hits[1],
                ms[1] / n,
                hits[2],
                ms[2] / n,
                hits[3],
                ms[3] / n,
                hits[4],
                ms[4] / n
            );
        }
    }
    // phantoms: noise only, the context set, the partner's frequency given
    let mut count = [0u32; 3];
    for progress in [
        QsoProgress::Rogers,
        QsoProgress::Replying,
        QsoProgress::Calling,
    ] {
        let k = match progress {
            QsoProgress::Rogers => 0,
            QsoProgress::Replying => 1,
            _ => 2,
        };
        for seed in 1..=phantom_slots {
            let pcm = slot(None, amp, -20.0, 1_000 + seed);
            let r = Decoder::<Ft8>::new(ctx_params(progress, ApMode::Full))
                .with_extras(extras(Some(Ft8Strategy::SinglePass)))
                .decode_candidates(&SlotInput::i16(&pcm), &grid9)
                .expect("single pass");
            for row in r.rows {
                println!("PHANTOM noise {progress:?}: {}", row.decoded.text);
                count[k] += 1;
            }
        }
    }
    println!(
        "noise-only slots with the context set ({phantom_slots} each, c9): phantoms Rogers {} Report {} Calling {}",
        count[0], count[1], count[2]
    );
}
