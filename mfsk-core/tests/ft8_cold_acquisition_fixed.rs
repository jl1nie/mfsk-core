//! Cold acquisition on the numeric path the board actually runs (#358).
//!
//! `ft8_cold_acquisition.rs` is gated `not(feature = "fixed-point")`,
//! so every measurement in it — including the one that concluded the
//! shipped estimator is fine — was taken in f32. The receiver is a
//! `fixed-point` build, and #280 already recorded that the two differ
//! for exactly the phenomenon at issue here: under `u16` quantisation
//! the far-lag ghosts do not climb into the top-5 the way they do in
//! f32, which is why one of that issue's tests carries a
//! `cfg(not(fixed-point))` of its own.
//!
//! On hardware, acquisition answered `-5.75 s` at `R 0.93` for a grid
//! 1.2 s out. In f32 the same estimator picks a phase that decodes
//! 15-16 messages at every offset tried. One of those is not measuring
//! the shipped receiver, and this file is the one that is.
//!
//! ```sh
//! cargo test -p mfsk-core --features full,fixed-point --release \
//!     --test ft8_cold_acquisition_fixed -- --ignored --nocapture
//! ```
#![cfg(feature = "fixed-point")]

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phase};
use mfsk_core::msg::decode_request::DecodeRequest;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;
use std::path::Path;

const SR: usize = 12_000;
const SLOT: usize = 180_000;
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
const MAX_CAND: usize = 200;

fn roll(audio: &[i16], n: i64) -> Vec<i16> {
    let len = audio.len() as i64;
    (0..audio.len())
        .map(|k| audio[((k as i64 + n).rem_euclid(len)) as usize])
        .collect()
}

fn acquire_tiled_k(slot: &[i16], top_k: usize) -> Option<(f32, f32)> {
    let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
    while long.len() < REQUIRED_SAMPLES {
        long.extend_from_slice(slot);
    }
    acquire_slot_phase(&long, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, top_k)
}

#[allow(dead_code)]
fn acquire_tiled(slot: &[i16]) -> Option<(f32, f32)> {
    let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
    while long.len() < REQUIRED_SAMPLES {
        long.extend_from_slice(slot);
    }
    acquire_slot_phase(&long, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 5)
}

/// Decodes obtainable once the grid sits at phase `p`.
fn decodes_at(slot: &[i16], p: f32) -> usize {
    let shifted = roll(slot, (p * SR as f32).round() as i64);
    DecodeRequest::<Ft8>::new(&shifted, FREQ_MIN, FREQ_MAX, SYNC_MIN, 200)
        .decode()
        .results
        .len()
}

/// Does the phase acquisition returns actually decode, on the numeric
/// path the receiver runs?
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn phase_quality_by_decodes_fixed_point() {
    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    assert_eq!(audio.len(), SLOT);

    println!("\n roll | acquired dt /    R | dec | best");
    println!("{:-<44}", "");
    let (mut tot, mut best_tot, mut zero) = (0usize, 0usize, 0usize);
    for i in 0..20 {
        let off_s = i as f32 * 0.75;
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        let est = acquire_tiled(&rolled);
        let d = est.map(|(p, _)| decodes_at(&rolled, p)).unwrap_or(0);
        let b = decodes_at(&rolled, -off_s);
        match est {
            Some((p, r)) => println!("{off_s:5.2} | {p:+11.2} / {r:.2} | {d:3} | {b:3}"),
            None => println!("{off_s:5.2} |        none        | {d:3} | {b:3}"),
        }
        tot += d;
        best_tot += b;
        if d == 0 {
            zero += 1;
        }
    }
    println!(
        "\nacquired: {tot} decodes total, {zero}/20 rolls decoded nothing\n\
         best:     {best_tot} decodes total"
    );
}

/// Does narrowing the estimate to the *strongest* candidates help?
///
/// The ghosts that poison the top-5 are weaker than the signal they
/// come from — on `qso3_busy` the real peak scored 229.5 against
/// ghosts at 112/78/64/57 — so a smaller `top_k` should stop averaging
/// them in. This costs nothing: `top_k` is already a parameter, and no
/// BP runs either way.
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn top_k_sweep_fixed_point() {
    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    println!("\ntop_k | decodes | rolls with none | R(ok) min..max | R(bad) min..max");
    println!("{:-<74}", "");
    for &k in &[1usize, 2, 3, 5, 8, 12, 20] {
        let (mut tot, mut zero) = (0usize, 0usize);
        let (mut okmin, mut okmax) = (2.0f32, -1.0f32);
        let (mut bmin, mut bmax) = (2.0f32, -1.0f32);
        for i in 0..40 {
            let off_s = i as f32 * 0.375;
            let rolled = roll(&audio, (off_s * SR as f32) as i64);
            match acquire_tiled_k(&rolled, k) {
                Some((p, r)) => {
                    let d = decodes_at(&rolled, p);
                    tot += d;
                    if d == 0 {
                        zero += 1;
                        bmin = bmin.min(r);
                        bmax = bmax.max(r);
                    } else {
                        okmin = okmin.min(r);
                        okmax = okmax.max(r);
                    }
                }
                None => zero += 1,
            }
        }
        println!(
            "{k:5} | {tot:7} | {zero:15} | {okmin:.2}..{okmax:.2}      | {bmin:.2}..{bmax:.2}"
        );
    }
    println!("\n(40 offsets at 0.375 s; best achievable is ~600 decodes, 0 rolls with none)");
}

/// With one candidate per tile the mean-resultant `R` is degenerate
/// (a lone angle always agrees with itself), so this asks whether the
/// candidate's own coarse **score** works as the confidence instead.
///
/// It measures a different thing: `R` asks whether candidates agree,
/// the score asks whether this one is a signal — which is the question
/// the gate actually needs answered, and the one every agreement-based
/// statistic tried so far has failed (#358).
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn top1_score_as_confidence() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Strongest candidate across the tiles, with its score.
    let top1 = |slot: &[i16]| -> Option<(f32, f32)> {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        let mut best: Option<SyncCandidate> = None;
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let spec = compute_spectrogram(&long[start..start + SLOT], FREQ_MAX);
            for c in coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5) {
                if best.as_ref().is_none_or(|b| c.score > b.score) {
                    best = Some(SyncCandidate {
                        freq_hz: c.freq_hz,
                        dt_sec: c.dt_sec + w,
                        score: c.score,
                    });
                }
            }
        }
        best.map(|c| (c.dt_sec, c.score))
    };

    println!("\n roll |     dt |  score | decodes");
    println!("{:-<40}", "");
    let (mut ok, mut bad): (Vec<f32>, Vec<f32>) = (Vec::new(), Vec::new());
    for i in 0..40 {
        let off_s = i as f32 * 0.375;
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        if let Some((dt, sc)) = top1(&rolled) {
            let d = decodes_at(&rolled, dt);
            println!("{off_s:5.2} | {dt:+6.2} | {sc:6.1} | {d:3}");
            if d == 0 { bad.push(sc) } else { ok.push(sc) }
        }
    }
    let stat = |v: &mut Vec<f32>| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        (
            v.first().copied().unwrap_or(0.0),
            v.last().copied().unwrap_or(0.0),
        )
    };
    let (okmin, okmax) = stat(&mut ok);
    let (bmin, bmax) = stat(&mut bad);
    println!(
        "\ndecoding rolls   (n={}): score {okmin:.1}..{okmax:.1}\n\
         non-decoding     (n={}): score {bmin:.1}..{bmax:.1}",
        ok.len(),
        bad.len()
    );
}

/// Is an absolute score gate safe on a weak band? (#358)
///
/// The worry was that a threshold tuned on this fixture would close
/// permanently where signals are weaker. But a band whose best signal
/// cannot be decoded has no grid to acquire, so the gate closing there
/// is correct rather than harmful — *if* score and decodability fall
/// together. This adds noise to the real recording (rather than
/// attenuating it, which leaves SNR untouched, or synthesising a clean
/// one, which is not a trustworthy instrument) and watches both.
#[test]
#[ignore = "diagnostic, decodes at every noise level — slow"]
fn score_tracks_decodability_as_snr_falls() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

    let clean = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Deterministic white noise, amplitude `amp` in i16 counts.
    let noisy = |amp: i32| -> Vec<i16> {
        let mut x: u32 = 0x2545_F491;
        clean
            .iter()
            .map(|&s| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                // Two draws summed: closer to Gaussian than one.
                let n = ((x >> 16) as i32 & 0xff) + ((x >> 8) as i32 & 0xff) - 255;
                (s as i32 + n * amp / 255).clamp(-32768, 32767) as i16
            })
            .collect()
    };

    // Top-1 score, and the same relative to the median candidate in
    // its own tile — a scale-free version that should not care how
    // loud the band is, only how far the winner stands above it.
    let top1_score = |slot: &[i16]| -> (f32, f32) {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        let mut best: Option<SyncCandidate> = None;
        let mut best_ratio = 0.0f32;
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let spec = compute_spectrogram(&long[start..start + SLOT], FREQ_MAX);
            let cs = coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5);
            if cs.is_empty() {
                continue;
            }
            let mut sc: Vec<f32> = cs.iter().map(|c| c.score).collect();
            sc.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = sc[sc.len() / 2].max(1e-6);
            for c in cs {
                if best.as_ref().is_none_or(|b| c.score > b.score) {
                    best_ratio = c.score / med;
                    best = Some(c);
                }
            }
        }
        (best.map(|c| c.score).unwrap_or(0.0), best_ratio)
    };

    println!("\n noise amp | top-1 score | top1/median | decodes at the true grid");
    println!("{:-<66}", "");
    for &amp in &[0i32, 200, 400, 800, 1600, 3200, 6400] {
        let a = noisy(amp);
        let (sc, ratio) = top1_score(&a);
        println!(
            "{amp:10} | {sc:11.1} | {ratio:11.2} | {:3}",
            decodes_at(&a, 0.0)
        );
    }
    println!(
        "\nIf the two columns fall together, a band too weak to gate through\n\
         is also a band with nothing to acquire, and an absolute gate is safe."
    );
}

/// Sync quality as the acquisition gate — phantom-tolerant, no BP (#358).
///
/// The discriminator acquisition needs is "is a real FT8 frame here",
/// not "does a valid message come out". A phantom decodes to nonsense
/// but its *sync* is genuine, so its DT is the grid's DT — which is
/// the only thing being acquired. That removes the reason to reach for
/// a decode: `refine_candidates_into` demodulates Costas block 0 and
/// scores it (56 DFT per candidate, ~13 ms on Core2), and never runs
/// belief propagation or an LDPC check.
///
/// Measured per roll: the acquired phase, and how many of that phase's
/// own coarse candidates clear a block-0 sync quality of 4 out of 7.
#[test]
#[ignore = "diagnostic, refines at every phase — slow"]
fn sync_quality_as_acquisition_gate() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{
        coarse_sync_with_lag, compute_spectrogram, refine_candidates_into,
    };

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Candidates whose dt sits at the acquired phase, refined; how many
    // look like an FT8 frame.
    let evidence = |slot: &[i16], phase: f32| -> (usize, u32) {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        // Refine each tile's candidates against *that tile's* window —
        // the audio the frame would actually be demodulated from.
        let mut ngood = 0usize;
        let mut qmax = 0u32;
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let win = &long[start..start + SLOT];
            let spec = compute_spectrogram(win, FREQ_MAX);
            let mut at_phase: Vec<SyncCandidate> = Vec::new();
            for c in coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5) {
                let mut d = (c.dt_sec + w) - phase;
                while d > 7.5 {
                    d -= 15.0;
                }
                while d <= -7.5 {
                    d += 15.0;
                }
                if d.abs() <= 0.25 {
                    at_phase.push(c);
                }
            }
            if at_phase.is_empty() {
                continue;
            }
            let n = at_phase.len().min(20);
            for r in refine_candidates_into(win, at_phase, n) {
                if r.2 >= 4 {
                    ngood += 1;
                }
                qmax = qmax.max(r.2);
            }
        }
        (ngood, qmax)
    };

    println!("\n roll |   phase | good q | max q | decodes");
    println!("{:-<50}", "");
    let (mut ok, mut bad): (Vec<usize>, Vec<usize>) = (Vec::new(), Vec::new());
    for i in 0..40 {
        let off_s = i as f32 * 0.375;
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        if let Some((p, _)) = acquire_tiled_k(&rolled, 2) {
            let d = decodes_at(&rolled, p);
            let (ngood, qmax) = evidence(&rolled, p);
            println!("{off_s:5.2} | {p:+7.2} | {ngood:6} | {qmax:5} | {d:3}");
            if d == 0 {
                bad.push(ngood)
            } else {
                ok.push(ngood)
            }
        }
    }
    let rng = |v: &mut Vec<usize>| {
        v.sort();
        (
            v.first().copied().unwrap_or(0),
            v.last().copied().unwrap_or(0),
        )
    };
    let (okmin, okmax) = rng(&mut ok);
    let (bmin, bmax) = rng(&mut bad);
    println!(
        "\ndecoding     (n={}): good-q candidates {okmin}..{okmax}\n\
         non-decoding (n={}): good-q candidates {bmin}..{bmax}",
        ok.len(),
        bad.len()
    );
}

/// Use sync quality to *select* the candidates, not to grade a phase
/// afterwards (#358).
///
/// A ghost is a coincidence in the coarse correlation; it does not
/// demodulate as a Costas array. `refine_candidates_into` demodulates
/// block 0 and scores it, without BP or an LDPC check — and phantoms
/// are welcome here, since a phantom's sync is real and its DT is the
/// grid's. So: refine each tile's strongest candidates, keep the ones
/// that look like frames, and take the medoid of *those* DTs.
#[test]
#[ignore = "diagnostic, refines at every phase — slow"]
fn refine_selected_acquisition() {
    use mfsk_core::engine::sync::{SyncCandidate, circular_dt_medoid};
    use mfsk_core::ft8::decode_block::{
        coarse_sync_with_lag, compute_spectrogram, refine_candidates_into,
    };

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    let acquire_refined = |slot: &[i16], q_min: u32, per_tile: usize| -> Option<(f32, f32)> {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        let mut kept: Vec<SyncCandidate> = Vec::new();
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let win = &long[start..start + SLOT];
            let spec = compute_spectrogram(win, FREQ_MAX);
            let cs = coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5);
            if cs.is_empty() {
                continue;
            }
            for r in refine_candidates_into(win, cs, per_tile) {
                if r.2 >= q_min {
                    kept.push(SyncCandidate {
                        freq_hz: r.0.freq_hz,
                        dt_sec: r.0.dt_sec + w,
                        // Weight by how much it looks like a frame.
                        score: r.2 as f32,
                    });
                }
            }
        }
        circular_dt_medoid(&kept, kept.len().max(1), 15.0)
    };

    println!("\n q_min | per_tile | decodes | rolls with none");
    println!("{:-<48}", "");
    for &(q_min, per_tile) in &[(4u32, 10usize), (5, 10), (6, 10), (6, 20), (7, 20)] {
        let (mut tot, mut zero) = (0usize, 0usize);
        for i in 0..40 {
            let off_s = i as f32 * 0.375;
            let rolled = roll(&audio, (off_s * SR as f32) as i64);
            match acquire_refined(&rolled, q_min, per_tile) {
                Some((p, _)) => {
                    let d = decodes_at(&rolled, p);
                    tot += d;
                    if d == 0 {
                        zero += 1;
                    }
                }
                None => zero += 1,
            }
        }
        println!("{q_min:6} | {per_tile:8} | {tot:7} | {zero:15}");
    }
    println!("\n(coarse-only medoid at top_k=2 is 458 decodes, 7 with none; best is ~600)");
}

/// Are the candidates that spoil the median phantoms, or sidelobes?
/// (#358)
///
/// The distinction decides whether including them should help. For
/// acquisition only the *macro* grid DT matters, and a phantom carries
/// the right one — its message is nonsense but its frame is real. A
/// correlation sidelobe of a strong signal does not: it is the same
/// frame seen at a displaced lag, so its DT is wrong by construction.
///
/// Test: compare every coarse candidate's DT against the DTs of the
/// messages the decoder actually recovers. A candidate that lines up
/// with one is a frame (decodable or phantom, either is fine). One
/// that does not is an artefact.
#[test]
#[ignore = "diagnostic, prints a table"]
fn are_the_top_candidates_frames_or_sidelobes() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::Ft8;
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};
    use mfsk_core::msg::decode_request::DecodeRequest;

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Where the real frames are, from the decoder.
    let res = DecodeRequest::<Ft8>::new(&audio, FREQ_MIN, FREQ_MAX, SYNC_MIN, 200).decode();
    let frames: Vec<(f32, f32)> = res.results.iter().map(|r| (r.freq_hz, r.dt_sec)).collect();
    println!("\n{} decoded frames", frames.len());

    let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
    while long.len() < REQUIRED_SAMPLES {
        long.extend_from_slice(&audio);
    }
    let spec = compute_spectrogram(&long[..SLOT], FREQ_MAX);
    let mut cs: Vec<SyncCandidate> =
        coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5);
    cs.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());

    println!("\nrank |    freq |     dt |  score | nearest decoded frame");
    println!("{:-<74}", "");
    for (i, c) in cs.iter().take(12).enumerate() {
        // Closest decoded frame in frequency, and how far its dt is.
        let near = frames
            .iter()
            .min_by(|a, b| {
                (a.0 - c.freq_hz)
                    .abs()
                    .partial_cmp(&(b.0 - c.freq_hz).abs())
                    .unwrap()
            })
            .copied();
        match near {
            Some((f, dt)) => {
                let same_dt = (dt - c.dt_sec).abs() <= 0.2;
                println!(
                    "{:4} | {:7.1} | {:+6.2} | {:6.1} | {:7.1} Hz {:+.2} s  (df {:+5.1} Hz, ddt {:+.2} s){}",
                    i + 1,
                    c.freq_hz,
                    c.dt_sec,
                    c.score,
                    f,
                    dt,
                    f - c.freq_hz,
                    dt - c.dt_sec,
                    if same_dt { "  <- frame" } else { "" }
                );
            }
            None => println!(
                "{:4} | {:7.1} | {:+6.2} | {:6.1} | none",
                i + 1,
                c.freq_hz,
                c.dt_sec,
                c.score
            ),
        }
    }

    let matched = cs
        .iter()
        .take(12)
        .filter(|c| {
            frames
                .iter()
                .any(|(f, dt)| (f - c.freq_hz).abs() <= 10.0 && (dt - c.dt_sec).abs() <= 0.2)
        })
        .count();
    println!(
        "\n{matched} of the top 12 coincide with a decoded frame in both \
         frequency and dt.\nThe rest carry a dt no real frame has — so they are not \
         phantoms, and\nincluding them cannot help the median."
    );
}

/// Choose the phase that puts the most stations *inside the search
/// window*, rather than the one that estimates the middle (#358).
///
/// Acquisition exists to make the receiver decode, and what decides
/// that is coverage: a station is decodable if its dt falls within the
/// coarse window of wherever the grid ends up. That is a
/// maximum-coverage question, not a central-tendency one, and the two
/// differ exactly when the population is skewed — `qso3_busy` has
/// fourteen stations near +0.26 s and F5RXL at -0.77 s, so the medoid
/// sits at +0.26 and its ±1.0 s window ends at -0.74, missing F5RXL by
/// 30 ms. A centre near +0.1 would hold all fifteen.
///
/// Sweeps the kernel half-width, since the right one is the search
/// window the decoder will actually use.
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn coverage_maximising_acquisition() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    let acquire_coverage =
        |slot: &[i16], half: f32, weighted: bool, per_tile: usize| -> Option<f32> {
            let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
            while long.len() < REQUIRED_SAMPLES {
                long.extend_from_slice(slot);
            }
            let mut all: Vec<SyncCandidate> = Vec::new();
            for &w in &[0.0_f32, 5.0, 10.0] {
                let start = (w * 12_000.0) as usize;
                let spec = compute_spectrogram(&long[start..start + SLOT], FREQ_MAX);
                let mut cs =
                    coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5);
                cs.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
                for c in cs.into_iter().take(per_tile) {
                    all.push(SyncCandidate {
                        freq_hz: c.freq_hz,
                        dt_sec: c.dt_sec + w,
                        score: c.score,
                    });
                }
            }
            if all.is_empty() {
                return None;
            }
            let wrap = |mut d: f32| {
                while d > 7.5 {
                    d -= 15.0;
                }
                while d <= -7.5 {
                    d += 15.0;
                }
                d
            };
            // Centre the window on each candidate, take the one that covers
            // the most. Ties break toward the centroid of what it covers.
            let mut best: Option<(f32, f32)> = None;
            for a in &all {
                let covered: Vec<&SyncCandidate> = all
                    .iter()
                    .filter(|b| wrap(b.dt_sec - a.dt_sec).abs() <= half)
                    .collect();
                let mass: f32 = if weighted {
                    covered.iter().map(|b| b.score).sum()
                } else {
                    covered.len() as f32
                };
                if best.as_ref().is_none_or(|&(_, m)| mass > m) {
                    let off: f32 = covered
                        .iter()
                        .map(|b| wrap(b.dt_sec - a.dt_sec))
                        .sum::<f32>()
                        / covered.len().max(1) as f32;
                    best = Some((wrap(a.dt_sec + off), mass));
                }
            }
            best.map(|(dt, _)| dt)
        };

    println!("\n half | weighted | per_tile | decodes | rolls with none");
    println!("{:-<60}", "");
    for &(half, weighted, per_tile) in &[
        (1.0f32, false, 2usize),
        (1.0, true, 2),
        (1.0, false, 3),
        (1.0, true, 3),
        (1.0, true, 5),
        (0.75, true, 3),
        (1.25, true, 3),
    ] {
        let (mut tot, mut zero) = (0usize, 0usize);
        for i in 0..40 {
            let off_s = i as f32 * 0.375;
            let rolled = roll(&audio, (off_s * SR as f32) as i64);
            match acquire_coverage(&rolled, half, weighted, per_tile) {
                Some(p) => {
                    let d = decodes_at(&rolled, p);
                    tot += d;
                    if d == 0 {
                        zero += 1;
                    }
                }
                None => zero += 1,
            }
        }
        println!(
            "{half:5.2} | {:8} | {per_tile:8} | {tot:7} | {zero:15}",
            weighted
        );
    }
    println!("\n(medoid at top_k=2 is 458 decodes, 7 with none; best is ~600)");
}

/// Two stages: medoid to find the grid, coverage to centre it (#358).
///
/// The medoid answers the question it is good at — *which* of the 15 s
/// positions the grid sits at — and nothing measured so far beats it
/// there. What it is not built for is the last second: it returns the
/// middle of the station population, and the middle is not where the
/// window covers the most stations when that population is skewed.
/// `qso3_busy` has fourteen stations near +0.26 s and F5RXL at
/// -0.77 s, a spread of 1.07 s against a ±1.0 s window — so the centre
/// matters and the medoid does not choose it.
///
/// So localise with the medoid, then slide the window within a second
/// of it to cover the most. Same shape as the decoder's own
/// coarse-then-fine.
///
/// Scored with `MFSK_SYNC_LAG_S=1.0` — the embedded window. At the
/// 2.5 s default a phase two seconds out still decodes, so the metric
/// cannot see a centring change at all.
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn medoid_then_coverage() {
    use mfsk_core::engine::sync::{SyncCandidate, circular_dt_medoid};
    use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    let wrap = |mut d: f32| {
        while d > 7.5 {
            d -= 15.0;
        }
        while d <= -7.5 {
            d += 15.0;
        }
        d
    };

    // All tiles' candidates, folded, plus the medoid over the top-2.
    let gather = |slot: &[i16]| -> (Vec<SyncCandidate>, Option<f32>) {
        let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
        while long.len() < REQUIRED_SAMPLES {
            long.extend_from_slice(slot);
        }
        let mut all: Vec<SyncCandidate> = Vec::new();
        for &w in &[0.0_f32, 5.0, 10.0] {
            let start = (w * 12_000.0) as usize;
            let spec = compute_spectrogram(&long[start..start + SLOT], FREQ_MAX);
            for c in coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 2.5) {
                all.push(SyncCandidate {
                    freq_hz: c.freq_hz,
                    dt_sec: c.dt_sec + w,
                    score: c.score,
                });
            }
        }
        let medoid = circular_dt_medoid(&all, 2, 15.0).map(|(dt, _)| dt);
        (all, medoid)
    };

    // Slide a ±`half` window within `reach` of the medoid; keep the
    // offset covering the most candidate mass.
    let centre = |all: &[SyncCandidate], m: f32, half: f32, reach: f32| -> f32 {
        let local: Vec<&SyncCandidate> = all
            .iter()
            .filter(|c| wrap(c.dt_sec - m).abs() <= reach)
            .collect();
        if local.is_empty() {
            return m;
        }
        let mut best = (m, -1.0f32);
        // Candidate centres: every local candidate, and the medoid.
        for cand in local.iter().map(|c| c.dt_sec).chain(core::iter::once(m)) {
            let mass: f32 = local
                .iter()
                .filter(|c| wrap(c.dt_sec - cand).abs() <= half)
                .map(|c| c.score)
                .sum();
            if mass > best.1 {
                best = (wrap(cand), mass);
            }
        }
        best.0
    };

    println!("\n half | reach | decodes | rolls with none");
    println!("{:-<46}", "");
    // Baseline first: the medoid alone.
    {
        let (mut tot, mut zero) = (0usize, 0usize);
        for i in 0..40 {
            let off_s = i as f32 * 0.375;
            let rolled = roll(&audio, (off_s * SR as f32) as i64);
            let (_, m) = gather(&rolled);
            match m {
                Some(p) => {
                    let d = decodes_at(&rolled, p);
                    tot += d;
                    if d == 0 {
                        zero += 1;
                    }
                }
                None => zero += 1,
            }
        }
        println!("  --  |   --  | {tot:7} | {zero:15}   (medoid alone)");
    }
    for &(half, reach) in &[(1.0f32, 1.0f32), (1.0, 1.5), (0.8, 1.0), (1.0, 0.6)] {
        let (mut tot, mut zero) = (0usize, 0usize);
        for i in 0..40 {
            let off_s = i as f32 * 0.375;
            let rolled = roll(&audio, (off_s * SR as f32) as i64);
            let (all, m) = gather(&rolled);
            match m {
                Some(m) => {
                    let p = centre(&all, m, half, reach);
                    let d = decodes_at(&rolled, p);
                    tot += d;
                    if d == 0 {
                        zero += 1;
                    }
                }
                None => zero += 1,
            }
        }
        println!("{half:5.1} | {reach:5.1} | {tot:7} | {zero:15}");
    }
}

/// What is actually happening at the offsets that decode nothing?
/// (#358)
///
/// Seven statistics have been tried as discriminators without once
/// looking at the failures themselves. First question, and it decides
/// whether there is anything subtle here at all: is the acquired phase
/// simply outside the window the decoder searches, or is it inside and
/// something further down rejects?
///
/// `residual` is how far the acquired phase is from one that undoes
/// the roll. The embedded coarse search covers ±1.0 s, so a residual
/// past that puts every station out of reach and there is nothing to
/// reject; a residual inside it means the loss happens later.
#[test]
#[ignore = "diagnostic, decodes at every phase — slow"]
fn where_do_the_failures_lose_it() {
    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    let wrap = |mut d: f32| {
        while d > 7.5 {
            d -= 15.0;
        }
        while d <= -7.5 {
            d += 15.0;
        }
        d
    };

    println!("\n roll | acquired | residual | decodes | inside ±1.0 s?");
    println!("{:-<58}", "");
    let (mut in_win_dead, mut out_win_dead, mut in_win_live) = (0, 0, 0);
    for i in 0..40 {
        let off_s = i as f32 * 0.375;
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        let Some((p, _)) = acquire_tiled_k(&rolled, 2) else {
            continue;
        };
        // Undoing the roll is a correct phase; the fixture's own offset
        // is common to both sides and cancels.
        let residual = wrap(p + off_s);
        let d = decodes_at(&rolled, p);
        let inside = residual.abs() <= 1.0;
        println!(
            "{off_s:5.2} | {p:+8.2} | {residual:+8.2} | {d:7} | {}",
            if inside { "yes" } else { "no" }
        );
        match (inside, d) {
            (true, 0) => in_win_dead += 1,
            (false, 0) => out_win_dead += 1,
            (true, _) => in_win_live += 1,
            (false, _) => {}
        }
    }
    println!(
        "\ndecoded nothing, residual outside ±1.0 s: {out_win_dead}\n\
         decoded nothing, residual *inside*  ±1.0 s: {in_win_dead}\n\
         decoded, residual inside:                   {in_win_live}\n\n\
         If the failures are all in the first row the loss is acquisition\n\
         landing out of reach, and nothing downstream is involved."
    );
}
