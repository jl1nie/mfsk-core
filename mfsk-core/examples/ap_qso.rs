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

fn main() {
    let mut args = std::env::args().skip(1);
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
                let pcm = slot(Some(&clean), amp, snr, seed);
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
