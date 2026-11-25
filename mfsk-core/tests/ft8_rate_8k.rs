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
//! The `fixed-point` build has no 8 kHz spectrogram (no 2560-point 16-bit plan): it panics, and says so.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::engine::source::SliceSource;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeDepth;
#[cfg(not(feature = "fixed-point"))]
use mfsk_core::ft8::decode_block::coarse_sync_gated;
use mfsk_core::ft8::decode_block::{Ft8Rate, GatedSearch, LowMem, decode_block_low_mem};
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

/// The 8 kHz spectrogram does not exist in the fixed-point build: it says so and stops.
#[cfg(feature = "fixed-point")]
#[test]
#[should_panic(expected = "12 kHz only")]
fn the_fixed_point_build_has_no_8k_spectrogram() {
    let slot = vec![0i16; 120_000];
    let _ = decode(Ft8Rate::Hz8000, &slot, 15);
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
