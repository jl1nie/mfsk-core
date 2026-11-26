// SPDX-License-Identifier: GPL-3.0-only
//! The low-memory FT8 decoders on 8 kHz audio read as it is (`Ft8Rate::Hz8000`), with no resample to 12 kHz in
//! front. Tier A (invariant), on synthetic slots: what the rate changes is the frame, the FFT and the step in
//! samples (NSPS 1280, FFT 2560) and the first stage of the 200 Hz cascade (8 kHz -> 4 kHz); the bins (3.125 Hz),
//! the rows (0.04 s) and everything from the 200 Hz baseband on are the 12 kHz path's.
//!
//! What would break it: a frame, FFT or step length off by the rate's ratio (the candidates move or vanish), the
//! stage-1 decimation, cutoff or group delay (a signal's dt or tones are off: the decode is lost, not just a few dB
//! down), the mixer at the wrong rate (every frequency is off by 1.5x).
//!
//! The `fixed-point` build has the 2560-point 16-bit plan (`Plan2560Sc16`): its 8 kHz tests are `an_8k_fixed_point_s3_flow_decodes_its_signals`
//! below and `icap_profile.rs`'s floors.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::engine::source::SliceSource;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeDepth;
#[cfg(not(feature = "fixed-point"))]
use mfsk_core::ft8::decode_block::coarse_sync_gated;
use mfsk_core::ft8::decode_block::{Ft8Rate, GatedSearch};
#[cfg(not(feature = "fixed-point"))]
use mfsk_core::ft8::decode_block::{LowMem, decode_block_low_mem};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

/// Deterministic Gaussian-ish noise (sum of 12 uniforms), so the test is the same on every machine.
struct Noise(u64);
impl Noise {
    fn next(&mut self) -> f32 {
        let mut a = 0.0f32;
        for _ in 0..12 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            a += ((self.0 >> 40) as f32) / (1u64 << 24) as f32;
        }
        a - 6.0
    }
}

type Sig = (&'static str, &'static str, &'static str, f32, f32);

/// A slot of `signals` (message, tone-0 frequency Hz, dt s) at amplitude `amp`, in noise of standard deviation
/// `sigma12` per sample **at 12 kHz** (the same spectral density at any rate: the deviation scales with the root of
/// the rate), `rate`'s 15 s.
fn slot_at(rate: Ft8Rate, signals: &[Sig], amp: i16, sigma12: f32, seed: u64) -> Vec<i16> {
    let hz = rate.hz();
    let mut slot = vec![0f32; 15 * hz as usize];
    for &(a, b, c, freq, dt) in signals {
        let m77 = pack77(a, b, c).expect("pack");
        let tones = message_to_tones::<Ft8>(&m77);
        let pcm = synthesize_i16::<Ft8>(&tones, hz, freq, amp);
        let at = ((0.5 + dt) * hz as f32).round() as i64;
        for (i, &v) in pcm.iter().enumerate() {
            let idx = at + i as i64;
            if idx >= 0 && (idx as usize) < slot.len() {
                slot[idx as usize] += v as f32;
            }
        }
    }
    let sigma = sigma12 * (hz as f32 / 12_000.0).sqrt();
    let mut n = Noise(seed);
    slot.iter()
        .map(|&v| (v + sigma * n.next()).round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

#[cfg(not(feature = "fixed-point"))]
fn decode(rate: Ft8Rate, slot: &[i16], keep: usize) -> Vec<(String, f32, f32)> {
    decode_block_low_mem(
        &mut SliceSource(slot),
        rate,
        &GatedSearch::new(100.0, 3000.0, 1.3, keep),
        DecodeDepth::EMBEDDED,
        LowMem::new(3, 4),
    )
    .iter()
    .filter_map(|d| unpack77(d.message77()).map(|m| (m, d.freq_hz, d.dt_sec)))
    .collect()
}

const CASES: [Sig; 6] = [
    ("CQ", "K1ABC", "FN42", 217.3, -0.45),
    ("K1ABC", "W9XYZ", "-12", 612.8, 0.0),
    ("CQ", "JA1ABC", "PM95", 1234.567, 0.31),
    ("W9XYZ", "K1ABC", "RR73", 1999.99, 1.1),
    ("CQ", "DL1XYZ", "JO62", 2870.4, -0.2),
    ("OH3NIV", "ZS6S", "-03", 2987.2, 0.74),
];

/// The 12 kHz test's cases (`ft8_embedded_driver_recall`), at 8 kHz: signals whose frequency and start are off every
/// grid, at the band's low and high ends, in noise of the same density (about -9 dB in 2500 Hz). Each is decoded at
/// its frequency and dt. Fails if a length in samples, the stage-1 cascade or the mixer rate is wrong.
#[cfg(not(feature = "fixed-point"))]
#[test]
fn an_8k_slot_decodes_its_signals_at_their_frequency_and_dt() {
    for (i, c) in CASES.iter().enumerate() {
        let slot = slot_at(
            Ft8Rate::Hz8000,
            std::slice::from_ref(c),
            3000,
            10_000.0,
            0x5EED,
        );
        let out = decode(Ft8Rate::Hz8000, &slot, 15);
        let want = format!("{} {} {}", c.0, c.1, c.2);
        let (_, f, dt) = out.iter().find(|(m, _, _)| *m == want).unwrap_or_else(|| {
            panic!(
                "case {i}: {want} at {} Hz, dt {}: not decoded ({out:?})",
                c.3, c.4
            )
        });
        assert!((f - c.3).abs() < 3.0, "case {i}: {} Hz decoded as {f}", c.3);
        assert!(
            (dt - c.4).abs() < 0.05,
            "case {i}: dt {} decoded as {dt}",
            c.4
        );
    }
}

/// Three signals 40-45 Hz apart decoded together: the cascade's stage 1 at 8 kHz must not let a neighbour in.
#[cfg(not(feature = "fixed-point"))]
#[test]
fn an_8k_slot_separates_neighbours() {
    let sigs: [Sig; 3] = [
        ("CQ", "K1ABC", "FN42", 1000.0, 0.1),
        ("K1ABC", "W9XYZ", "-12", 1043.0, 0.35),
        ("W9XYZ", "K1ABC", "RR73", 1088.0, -0.1),
    ];
    let slot = slot_at(Ft8Rate::Hz8000, &sigs, 3000, 6_000.0, 0xBEEF);
    let out = decode(Ft8Rate::Hz8000, &slot, 15);
    for s in &sigs {
        let want = format!("{} {} {}", s.0, s.1, s.2);
        assert!(
            out.iter().any(|(m, _, _)| *m == want),
            "{want} not decoded: {out:?}"
        );
    }
}

/// Precision: noise alone at 8 kHz decodes nothing (the same noise density, several seeds).
#[cfg(not(feature = "fixed-point"))]
#[test]
fn noise_alone_at_8k_decodes_nothing() {
    for seed in [1u64, 2, 3, 4, 5] {
        let slot = slot_at(Ft8Rate::Hz8000, &[], 0, 10_000.0, seed);
        let out = decode(Ft8Rate::Hz8000, &slot, 30);
        assert!(out.is_empty(), "seed {seed}: {out:?}");
    }
}

/// The rate is what reads the audio: the same 8 kHz slot read as 12 kHz (or a 12 kHz slot as 8 kHz) holds no
/// decodable signal, so an argument that was ignored would not pass `an_8k_slot_decodes_its_signals_...` and
/// this together.
#[cfg(not(feature = "fixed-point"))]
#[test]
fn the_rate_is_what_reads_the_audio() {
    let c = &CASES[2];
    let at8 = slot_at(
        Ft8Rate::Hz8000,
        std::slice::from_ref(c),
        3000,
        10_000.0,
        0x5EED,
    );
    let at12 = slot_at(
        Ft8Rate::Hz12000,
        std::slice::from_ref(c),
        3000,
        10_000.0,
        0x5EED,
    );
    let want = format!("{} {} {}", c.0, c.1, c.2);
    assert!(
        decode(Ft8Rate::Hz8000, &at8, 15)
            .iter()
            .any(|(m, _, _)| *m == want)
    );
    assert!(
        decode(Ft8Rate::Hz12000, &at12, 15)
            .iter()
            .any(|(m, _, _)| *m == want)
    );
    assert!(
        !decode(Ft8Rate::Hz12000, &at8, 15)
            .iter()
            .any(|(m, _, _)| *m == want),
        "8 kHz audio read as 12 kHz"
    );
    assert!(
        !decode(Ft8Rate::Hz8000, &at12, 15)
            .iter()
            .any(|(m, _, _)| *m == want),
        "12 kHz audio read as 8 kHz"
    );
}

/// The coarse search sees the same slot at either rate: the bins and rows are 3.125 Hz and 0.04 s at both, so a
/// signal's candidate is at the same frequency (within a bin) and dt (within a row), and the hard-sync count (of 21)
/// is full for each. Fails if a frame, FFT or step length is not the rate's.
#[cfg(not(feature = "fixed-point"))]
#[test]
fn an_8k_frame_gives_the_12k_candidates() {
    let sigs: [Sig; 3] = [CASES[0], CASES[2], CASES[4]];
    let search = GatedSearch::new(100.0, 3000.0, 1.3, 30);
    let list = |rate| {
        let slot = slot_at(rate, &sigs, 8000, 1.0, 7);
        coarse_sync_gated(&mut SliceSource(&slot), rate, &search, 3)
    };
    let (a, b) = (list(Ft8Rate::Hz12000), list(Ft8Rate::Hz8000));
    for s in &sigs {
        let near = |l: &[(mfsk_core::engine::sync::SyncCandidate, u8)]| {
            l.iter()
                .filter(|(c, _)| (c.freq_hz - s.3).abs() < 3.2 && (c.dt_sec - s.4).abs() < 0.045)
                .map(|(c, n)| (c.freq_hz, c.dt_sec, *n))
                .next()
        };
        let (ca, cb) = (near(&a), near(&b));
        let (ca, cb) = (
            ca.unwrap_or_else(|| panic!("{} Hz: no 12 kHz candidate in {a:?}", s.3)),
            cb.unwrap_or_else(|| panic!("{} Hz: no 8 kHz candidate in {b:?}", s.3)),
        );
        assert_eq!(ca.2, 21, "12 kHz hard sync of {} Hz", s.3);
        assert_eq!(cb.2, 21, "8 kHz hard sync of {} Hz", s.3);
        assert!(
            (ca.0 - cb.0).abs() <= 3.2 && (ca.1 - cb.1).abs() <= 0.045,
            "{} Hz: {ca:?} against {cb:?}",
            s.3
        );
    }
}

/// The S3's own flow (`S3_GATE_OSD_ABLATION.md`'s V9: the gate, the first candidates by hard sync, `fine_sync_at`, block-0
/// refine, stage 3 with the f32 Goertzel, all in the fixed-point numeric path) on 8 kHz audio read as it is: the signals of the
/// 12 kHz test, decoded at their frequency and dt. Fails if any of the rate-taking pieces (the 2560-point plan and the
/// spectrogram's gain, the fine-sync bins, the Goertzel symbol length and scale, the window end) is not the rate's.
#[cfg(feature = "fixed-point")]
#[test]
fn an_8k_fixed_point_s3_flow_decodes_its_signals() {
    use mfsk_core::engine::scalar::Cmplx;
    use mfsk_core::ft8::decode_block::{
        DEFAULT_Q_THRESH, SymMask, coarse_sync_gated, fill_symbol_spectra_goertzel_at,
        fine_sync_at, process_candidates_into_with_cs_scratch_tuned_with_fill,
        refine_candidates_into_at,
    };
    let rate = Ft8Rate::Hz8000;
    let depth = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    for (i, c) in CASES.iter().enumerate() {
        let slot = slot_at(rate, std::slice::from_ref(c), 3000, 6_000.0, 0x5EED);
        let cands: Vec<_> = coarse_sync_gated(
            &mut SliceSource(&slot),
            rate,
            &GatedSearch::new(100.0, 3000.0, 1.3, 15),
            1,
        )
        .into_iter()
        .map(|x| x.0)
        .collect();
        let cands = fine_sync_at(&slot, rate, &cands);
        let refined = refine_candidates_into_at(&slot, rate, cands, 15);
        let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
        let out = process_candidates_into_with_cs_scratch_tuned_with_fill(
            &slot,
            refined,
            depth,
            DEFAULT_Q_THRESH,
            30,
            &mut cs,
            |cs, cand, mask: SymMask| {
                fill_symbol_spectra_goertzel_at(cs, &slot, rate, cand.freq_hz, cand.dt_sec, mask)
            },
        );
        let want = format!("{} {} {}", c.0, c.1, c.2);
        let hit = out
            .iter()
            .find(|d| unpack77(d.message77()).is_some_and(|m| m == want))
            .unwrap_or_else(|| panic!("case {i}: {want} at {} Hz, dt {}: not decoded", c.3, c.4));
        assert!(
            (hit.freq_hz - c.3).abs() < 3.0,
            "case {i}: {} Hz decoded as {}",
            c.3,
            hit.freq_hz
        );
        assert!(
            (hit.dt_sec - c.4).abs() < 0.05,
            "case {i}: dt {} decoded as {}",
            c.4,
            hit.dt_sec
        );
    }
}

/// The block-0 refine at 8 kHz ranks a real candidate above a decoy by its Costas quality `q` (0..=7: 6-7 for a real signal,
/// about 1 for noise): the true candidate first with `q >= 6`, the decoy low. A refine that read 8 kHz audio as 12 kHz would
/// give the true one a noise `q` (the stage-3 `q` is recomputed, so only this ranking, under a `max_cand` cut, tells).
#[cfg(feature = "fixed-point")]
#[test]
fn the_8k_block_0_refine_ranks_the_real_candidate_first() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::refine_candidates_into_at;
    let rate = Ft8Rate::Hz8000;
    let c = &CASES[2];
    let slot = slot_at(rate, std::slice::from_ref(c), 3000, 3_000.0, 0x5EED);
    let cands = vec![
        SyncCandidate {
            freq_hz: 1900.0,
            dt_sec: 0.2,
            score: 9.0,
        },
        SyncCandidate {
            freq_hz: c.3,
            dt_sec: c.4,
            score: 1.0,
        },
    ];
    let refined = refine_candidates_into_at(&slot, rate, cands, 2);
    assert_eq!(refined.len(), 2);
    let (best, q_best) = (&refined[0].0, refined[0].2);
    assert!(
        (best.freq_hz - c.3).abs() < 0.1,
        "the real candidate is not first: {best:?}"
    );
    assert!(q_best >= 6, "q of the real candidate {q_best}");
    assert!(refined[1].2 <= 3, "q of the decoy {}", refined[1].2);
}

/// `fine_sync_at` at 8 kHz finds an off-grid signal to within one search step (0.5 Hz, 5 ms), as `fine_sync_12k` does at 12 kHz
/// (its unit test, the same signal and noise density); and at 12 kHz it **is** `fine_sync_12k`. Fails if the bin
/// (`rate / 200` samples), the mixer's phase step or the symbol length is the 12 kHz one.
#[test]
fn fine_sync_at_finds_an_off_grid_signal_at_either_rate() {
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode_block::{fine_sync_12k, fine_sync_at};
    const F_TRUE: f32 = 1501.2;
    const DT_TRUE: f32 = 0.137;
    let msg: Vec<u8> = (0..77).map(|i| ((i * 7 + 3) % 5 == 0) as u8).collect();
    let itone = message_to_tones::<Ft8>(msg.as_slice().try_into().unwrap());
    let coarse = SyncCandidate {
        freq_hz: 1_500.0,
        dt_sec: 0.10,
        score: 1.0,
    };
    for rate in [Ft8Rate::Hz12000, Ft8Rate::Hz8000] {
        let hz = rate.hz();
        let sig = synthesize_i16::<Ft8>(&itone, hz, F_TRUE, 2_000);
        let mut audio = vec![0i16; 15 * hz as usize];
        let start = ((0.5 + DT_TRUE) * hz as f32).round() as usize;
        // the noise density of the 12 kHz unit test (+-4000 uniform): scaled to the rate
        let amp = 4_000.0 * (hz as f32 / 12_000.0).sqrt();
        let mut state: u32 = 0x1234_5678;
        for (i, a) in audio.iter_mut().enumerate() {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            let noise = ((state % 8_001) as f32 / 8_000.0 * 2.0 - 1.0) * amp;
            let sg = if i >= start && i - start < sig.len() {
                sig[i - start] as f32
            } else {
                0.0
            };
            *a = (sg + noise).clamp(-32_768.0, 32_767.0) as i16;
        }
        let r = fine_sync_at(&audio, rate, std::slice::from_ref(&coarse)).remove(0);
        assert!(
            (r.freq_hz - F_TRUE).abs() <= 0.5,
            "{rate:?}: freq {} against {F_TRUE}",
            r.freq_hz
        );
        assert!(
            (r.dt_sec - DT_TRUE).abs() <= 0.005,
            "{rate:?}: dt {} against {DT_TRUE}",
            r.dt_sec
        );
        if rate == Ft8Rate::Hz12000 {
            let old = fine_sync_12k(&audio, std::slice::from_ref(&coarse)).remove(0);
            assert_eq!(
                (old.freq_hz.to_bits(), old.dt_sec.to_bits()),
                (r.freq_hz.to_bits(), r.dt_sec.to_bits())
            );
        }
    }
}

/// The f32 Goertzel's cells of one signal are the same at either rate (the sums are scaled by `12000 / rate`): the
/// transmitted tone's magnitude in every symbol within 3 % of the 12 kHz one's, the other seven tones small; and the window
/// end is the symbol length of the rate (`i0 + 79 x rate x 0.16`). Fails if the symbol is not `rate x 0.16` samples, the
/// scale is missing (2/3 of it) or the tone frequencies use the wrong rate.
#[test]
fn the_goertzel_cells_are_the_same_at_either_rate() {
    use mfsk_core::engine::scalar::Cmplx;
    use mfsk_core::ft8::decode_block::{
        SymMask, fill_symbol_spectra_goertzel_at, goertzel_window_end_sample_at,
    };
    let c = &CASES[2];
    let m77 = pack77(c.0, c.1, c.2).unwrap();
    let tones = message_to_tones::<Ft8>(&m77);
    let cells = |rate: Ft8Rate| {
        let slot = slot_at(rate, std::slice::from_ref(c), 3000, 0.0, 1);
        let mut cs = [[Cmplx::<f32>::default(); 8]; 79];
        for mask in [SymMask::SyncOnly, SymMask::DataOnly] {
            fill_symbol_spectra_goertzel_at(&mut cs, &slot, rate, c.3, c.4, mask);
        }
        cs
    };
    let (a, b) = (cells(Ft8Rate::Hz12000), cells(Ft8Rate::Hz8000));
    let mag = |x: &Cmplx<f32>| (x.re * x.re + x.im * x.im).sqrt();
    for (sym, &tone) in tones.iter().enumerate() {
        let t = tone as usize;
        let (ma, mb) = (mag(&a[sym][t]), mag(&b[sym][t]));
        assert!(
            (mb / ma - 1.0).abs() < 0.03,
            "symbol {sym}: 8 kHz {mb} against 12 kHz {ma}"
        );
        for k in (0..8).filter(|&k| k != t) {
            assert!(
                mag(&b[sym][k]) < 0.15 * mb,
                "symbol {sym}: tone {k} leaks at 8 kHz"
            );
        }
    }
    let end = |r: Ft8Rate| goertzel_window_end_sample_at(c.4, r);
    assert_eq!(
        end(Ft8Rate::Hz12000),
        ((0.5 + c.4) * 12_000.0).round() as usize + 79 * 1920
    );
    assert_eq!(
        end(Ft8Rate::Hz8000),
        ((0.5 + c.4) * 8_000.0).round() as usize + 79 * 1280
    );
}

/// `from_hz` is the inverse of `hz` for the rates there are, and nothing else is one.
#[test]
fn a_rate_is_found_from_its_hz_and_nothing_else_is_one() {
    for r in [Ft8Rate::Hz12000, Ft8Rate::Hz8000] {
        assert_eq!(Ft8Rate::from_hz(r.hz()), Some(r));
    }
    for hz in [0u32, 6_000, 8_001, 11_025, 12_001, 16_000, 48_000] {
        assert_eq!(Ft8Rate::from_hz(hz), None, "{hz}");
    }
}
