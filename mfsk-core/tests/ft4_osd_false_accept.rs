//! FT4's OSD must not hand back wrong codewords from noise at a rate the
//! reference does not (#456).
//!
//! `ft4_decode.f90` decodes every pass, blind or a-priori, through
//! `decode174_91(llr, Keff=91, maxosd=2, ndeep=2, apmask)`: BP, then OSD on the BP
//! sum after 1 and after 2 iterations, the CRC checked once, on the winner. This
//! crate's FT4 goes through `Ldpc174_91::decode_soft`, whose OSD stage ran
//! `osd_decode_deep` on the raw LLR: an order-`depth` search over every bit, the
//! CRC on every candidate.
//!
//! Measured on iid Gaussian LLRs (sd 2.83), the count being results that pass
//! the CRC (BP or OSD), and, in brackets, those that also clear the pipeline's
//! `hard_errors < osd_max_errors(depth)` gate (`DecodeStrictness::Normal`):
//!
//! | | draws | CRC-valid | of those, under the hard-error gate |
//! |---|---|---|---|
//! | upstream `decode174_91` (maxosd 2, norder 2), gfortran | 260 000 | 15 (5.8e-5) | (no such gate) |
//! | this crate, `osd_depth` 2, before | 30 000 | 6787 (22.6 %) | 19 |
//! | this crate, `osd_depth` 3, before | 1 200 | 1200 (every one) | 1 |
//! | this crate, `osd_depth` 4, before | 1 200 | 1200 (every one) | 4 |
//! | this crate, `osd_depth` 2, after | 300 000 | 29 (9.7e-5) | 25 |
//!
//! After, depth 3 and 4 are the same search as depth 2 (`ft4_decode.f90` has a
//! fixed `ndeep = 2`): 2 in 30 000 each. Through the pipeline, on 3 000 slots of
//! white noise (`ft4_noise_slots_measure`): `(sync_min 1.2, 50)` 0 phantoms before
//! and after, `(0.05, 100)` 9 before and 2 after.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::engine::pipeline::DecodeStrictness;
use mfsk_core::engine::protocol::{FecCodec, FecOpts};
use mfsk_core::fec::Ldpc174_91;
use mfsk_core::fec::ldpc::check_crc14;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn unif(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f32 {
        let (u1, u2) = (self.unif(), self.unif());
        ((-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()) as f32
    }
}

/// `(results that pass the CRC, of those the ones under the pipeline's hard-error gate)`.
fn crc_valid_decodes(osd_depth: u32, draws_per_thread: usize) -> (u64, u64) {
    const THREADS: usize = 12;
    let ceiling = DecodeStrictness::Normal.osd_max_errors(osd_depth as u8);
    let hs: Vec<(u64, u64)> = std::thread::scope(|sc| {
        let hs: Vec<_> = (0..THREADS)
            .map(|t| {
                sc.spawn(move || {
                    let mut rng = Rng(0x9e37_79b9_7f4a_7c15u64 ^ (t as u64 + 2001));
                    let opts = FecOpts {
                        osd_depth,
                        verify_info: Some(check_crc14),
                        ..FecOpts::default()
                    };
                    let (mut all, mut gated) = (0u64, 0u64);
                    for _ in 0..draws_per_thread {
                        let mut llr = [0f32; 174];
                        for v in llr.iter_mut() {
                            *v = 2.83 * rng.gauss();
                        }
                        if let Some(r) = Ldpc174_91.decode_soft(&llr, &opts) {
                            all += 1;
                            if r.hard_errors < ceiling {
                                gated += 1;
                            }
                        }
                    }
                    (all, gated)
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    hs.into_iter().fold((0, 0), |a, b| (a.0 + b.0, a.1 + b.1))
}

/// The gate: on noise, `osd_depth` 2 (what FT4's pipeline requests below its
/// `osd_depth3_min`) must pass the CRC about as rarely as `decode174_91` does.
/// 240 000 draws: about 14 expected at 5.8e-5, 54 000 at the old 22.6 %.
#[test]
fn ft4_osd_on_noise_returns_wrong_codewords_at_the_reference_rate() {
    let (all, _) = crc_valid_decodes(2, 20_000);
    assert!(
        all <= 60,
        "{all} CRC-valid FT4 OSD results in 240 000 noise draws; decode174_91 gives 5.8e-5 (about 14)"
    );
}

/// Prints the table's crate cells. `--ignored --nocapture`.
#[test]
#[ignore = "measurement, prints; run with --release --ignored --nocapture"]
fn ft4_measure() {
    for (depth, per_thread) in [(2u32, 25_000usize), (3, 2_500), (4, 2_500)] {
        let (all, gated) = crc_valid_decodes(depth, per_thread);
        println!(
            "FT4_MEASURE osd_depth {depth}: {all} CRC-valid in {} draws ({gated} under the hard-error gate)",
            per_thread * 12
        );
    }
}

/// Whole-pipeline phantoms: FT4 slots of white noise and nothing else, so every
/// decode is a false one. Measures the two request shapes the tests use
/// (`(sync_min 1.2, 50)` is WSJT-X's `syncmin`; `(0.05, 100)` the loose one).
/// `--ignored --nocapture`.
#[test]
#[ignore = "measurement, prints; run with --release --ignored --nocapture"]
fn ft4_noise_slots_measure() {
    use mfsk_core::ft4::Ft4;
    use mfsk_core::msg::decode_request::DecodeRequest;
    const SLOTS: usize = 3_000;
    const SAMPLES: usize = 90_000; // 7.5 s at 12 kHz
    for (label, sync_min, max_cand) in [("(1.2, 50)", 1.2f32, 50usize), ("(0.05, 100)", 0.05, 100)]
    {
        let per_thread = SLOTS / 12;
        let total: usize = std::thread::scope(|sc| {
            let hs: Vec<_> = (0..12)
                .map(|t| {
                    sc.spawn(move || {
                        let mut rng = Rng(0xD1B5_4A32_D192_ED03u64 ^ (t as u64 + 31));
                        let mut n = 0usize;
                        for _ in 0..per_thread {
                            let audio: Vec<i16> = (0..SAMPLES)
                                .map(|_| (1000.0 * rng.gauss()).clamp(-32000.0, 32000.0) as i16)
                                .collect();
                            n += DecodeRequest::<Ft4>::new(
                                &audio, 100.0, 2700.0, sync_min, max_cand,
                            )
                            .decode()
                            .results
                            .len();
                        }
                        n
                    })
                })
                .collect();
            hs.into_iter().map(|h| h.join().unwrap()).sum()
        });
        println!(
            "FT4_NOISE {label}: {total} decodes in {} noise slots",
            per_thread * 12
        );
    }
}
