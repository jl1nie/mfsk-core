//! FT8's a-priori OSD must not hand back wrong codewords from noise at a rate
//! the reference does not (#456).
//!
//! `ft8b.f90` runs its AP passes through the same `decode174_91` call as the
//! blind ones, with an `apmask`: BP holds the locked bits, OSD runs on the BP
//! sum after 1 and after 2 iterations (never the raw LLR), a test pattern that
//! flips a locked bit is skipped, and the CRC is checked on the winner. This
//! crate's AP rung ran `osd_decode_deep(&llr_ap, 2, Some(check_crc14))`
//! instead: the raw LLR, an order-2 search over every bit, the CRC on every
//! candidate.
//!
//! Measured on iid Gaussian LLRs (sd 2.83) with the lock `ft8b.f90` sets up
//! (`apmag = maxval(|llr|)*1.1`, locked bits `+-apmag`), the count being decodes
//! that pass the CRC (BP or OSD, before any message check):
//!
//! | lock | upstream `decode174_91`, gfortran | this crate, before | this crate, after |
//! |---|---|---|---|
//! | CQ: bits 1-29, 75-77 | 111 in 1 200 000 (9.3e-5) | 6676 in 30 000 (22 %) | 34 in 300 000 (1.1e-4) |
//! | bits 1-58, 75-77 | 109 in 1 200 000 (9.1e-5) | 6730 in 30 000 (22 %) | 32 in 300 000 (1.1e-4) |
//!
//! "Before" passed the CRC in a fifth of the noise candidates because an
//! order-2 search over every bit, with the locked ones at the top of the
//! reliability order, has thousands of candidates that flip them, and each was
//! CRC-checked; `validate` and `ap_max_errors` in `process_one_candidate_inner`
//! were what kept those out of the decode list. The gate below holds "after"
//! near the reference; the measurement (`--ignored`) prints all four cells.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::fec::ldpc::bp::{BpScratch, bp_decode, bp_llr_zsum_ap_with_scratch};
use mfsk_core::fec::ldpc::check_crc14;
use mfsk_core::fec::ldpc::osd::{osd_decode_deep, osd_decode_npre1_masked};
use mfsk_core::fec::ldpc::params::Ldpc174_91Params;

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

/// `(mask, locked llr)` for a lock on bits `0..n_msg` (pattern from `rng`) and 74..77.
fn lock(rng: &mut Rng, noise: &[f32; 174], n_msg: usize, cq: bool) -> ([bool; 174], [f32; 174]) {
    let apmag = noise.iter().fold(0f32, |m, v| m.max(v.abs())) * 1.1;
    let mut mask = [false; 174];
    let mut llr = *noise;
    for i in 0..n_msg {
        mask[i] = true;
        llr[i] = if cq {
            // mcq: only bit 27 (1-based) set
            if i == 26 { apmag } else { -apmag }
        } else if rng.next() & 1 == 1 {
            apmag
        } else {
            -apmag
        };
    }
    for (i, v) in [(74, -apmag), (75, -apmag), (76, apmag)] {
        mask[i] = true;
        llr[i] = v;
    }
    (mask, llr)
}

#[derive(Clone, Copy)]
enum Rung {
    /// What the AP rung ran before: BP with the mask, then `osd_decode_deep` on the raw LLR.
    Before,
    /// `decode174_91`'s way: BP with the mask, then OSD on the BP sum after 1 and 2 iterations,
    /// masked `npre1`, CRC on the winner.
    After,
}

fn crc_valid_decodes(rung: Rung, draws_per_thread: usize, n_msg: usize, cq: bool) -> u64 {
    const THREADS: usize = 12;
    std::thread::scope(|sc| {
        let hs: Vec<_> = (0..THREADS)
            .map(|t| {
                sc.spawn(move || {
                    let mut rng = Rng(0x2545_F491_4F6C_DD1Du64 ^ ((t as u64 + 7) * 0x9e37_79b9));
                    let mut scratch = BpScratch::<Ldpc174_91Params, f32>::new();
                    let mut hits = 0u64;
                    for _ in 0..draws_per_thread {
                        let mut noise = [0f32; 174];
                        for v in noise.iter_mut() {
                            *v = 2.83 * rng.gauss();
                        }
                        let (mask, llr) = lock(&mut rng, &noise, n_msg, cq);
                        if bp_decode(&llr, Some(&mask), 30, Some(check_crc14)).is_some() {
                            hits += 1;
                            continue;
                        }
                        let found = match rung {
                            Rung::Before => osd_decode_deep(&llr, 2, Some(check_crc14)).is_some(),
                            Rung::After => [1u32, 2].into_iter().any(|n_iter| {
                                let z = bp_llr_zsum_ap_with_scratch::<Ldpc174_91Params>(
                                    &mut scratch,
                                    &llr,
                                    Some(&mask),
                                    n_iter,
                                );
                                let mut zz = [0f32; 174];
                                zz.copy_from_slice(z);
                                osd_decode_npre1_masked(&zz, Some(&mask)).is_some()
                            }),
                        };
                        if found {
                            hits += 1;
                        }
                    }
                    hits
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).sum()
    })
}

/// The gate: with the CQ lock, on noise, the rung must pass the CRC about as
/// rarely as `decode174_91` does. 240 000 draws: about 22 expected at 1e-4,
/// 53 000 at the old 22 %.
#[test]
fn ap_rung_on_noise_returns_wrong_codewords_at_the_reference_rate() {
    let hits = crc_valid_decodes(Rung::After, 20_000, 29, true);
    assert!(
        hits <= 70,
        "{hits} CRC-valid AP decodes in 240 000 noise draws; decode174_91 gives 9.3e-5 (about 22)"
    );
}

/// Prints the table's crate columns. `--ignored --nocapture`; sized so the
/// expected hits at the reference rate are a few dozen, not a thousand.
#[test]
#[ignore = "measurement, prints; run with --release --ignored --nocapture"]
fn ap_measure() {
    for (name, n_msg, cq) in [("CQ 1-29,75-77", 29, true), ("1-58,75-77", 58, false)] {
        let per_thread = 25_000; // 300 000 draws
        let after = crc_valid_decodes(Rung::After, per_thread, n_msg, cq);
        let before = crc_valid_decodes(Rung::Before, per_thread / 10, n_msg, cq);
        println!(
            "AP_MEASURE {name}: after {after} in {} ; before {before} in {}",
            per_thread * 12,
            per_thread / 10 * 12
        );
    }
}

/// Whole-pipeline phantoms for FT8: slots of white noise and nothing else, so every
/// decode is a false one, under the three a-priori conditions the sweeps use: no
/// hint (only the blind-CQ pass), `CQ,JL1NIE` (a heavy lock) and `K1ABC,W9XYZ`. The
/// request is the sweep's, `(sync_min 0.8, 50)`. `--ignored --nocapture`.
#[test]
#[ignore = "measurement, prints; run with --release --ignored --nocapture"]
fn ft8_ap_noise_slots_measure() {
    use mfsk_core::ft8::Ft8;
    use mfsk_core::msg::ap::ApHint;
    use mfsk_core::msg::decode_request::DecodeRequest;
    use mfsk_core::msg::wsjt77::unpack77;
    const SLOTS: usize = 3_000;
    const SAMPLES: usize = 180_000; // 15 s at 12 kHz
    for (label, hint) in [
        ("no hint", None),
        (
            "CQ,JL1NIE",
            Some(ApHint::new().with_call1("CQ").with_call2("JL1NIE")),
        ),
        (
            "K1ABC,W9XYZ",
            Some(ApHint::new().with_call1("K1ABC").with_call2("W9XYZ")),
        ),
    ] {
        let per_thread = SLOTS / 12;
        let hint = &hint;
        let msgs: Vec<String> = std::thread::scope(|sc| {
            let hs: Vec<_> = (0..12)
                .map(|t| {
                    sc.spawn(move || {
                        let mut rng = Rng(0xC0FF_EE00_1234_5678u64 ^ (t as u64 + 77));
                        let mut out = Vec::new();
                        for _ in 0..per_thread {
                            let audio: Vec<i16> = (0..SAMPLES)
                                .map(|_| (1000.0 * rng.gauss()).clamp(-32000.0, 32000.0) as i16)
                                .collect();
                            let mut req = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 50);
                            if let Some(h) = hint.as_ref() {
                                req = req.ap_hint(h);
                            }
                            for d in req.decode().results {
                                out.push(format!(
                                    "{} (pass {}, {} hard errors)",
                                    unpack77(d.message77()).unwrap_or_default(),
                                    d.pass,
                                    d.hard_errors
                                ));
                            }
                        }
                        out
                    })
                })
                .collect();
            hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
        });
        println!(
            "FT8_NOISE {label}: {} decodes in {} noise slots {:?}",
            msgs.len(),
            per_thread * 12,
            &msgs[..msgs.len().min(4)]
        );
    }
}
