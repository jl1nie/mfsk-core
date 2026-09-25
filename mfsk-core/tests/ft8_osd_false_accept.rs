//! FT8's OSD must not hand back wrong codewords from noise at a rate the
//! reference does not (#452).
//!
//! `osd174_91.f90` picks the closest of all its candidate codewords and checks
//! the CRC once, on that winner. This crate used to check every candidate and
//! keep the closest CRC-valid one: more sensitive on weak signals and about
//! forty times likelier to return a wrong codeword from noise.
//!
//! Measured on iid Gaussian LLRs (sd 2.83, the scale `ft8b.f90` normalises to),
//! decoded as the ladder does — `bp_llr_zsum` after 1 then 2 iterations into
//! `osd_decode_npre1`, first success wins:
//!
//! | | draws | CRC-valid results |
//! |---|---|---|
//! | upstream `decode174_91` (maxosd 2, norder 2), gfortran | 260 000 | 15 (5.8e-5) |
//! | this crate, CRC on every candidate | 200 000 | 452 (2.3e-3) |
//! | this crate, CRC on the winner | 2 000 000 | 180 (9.0e-5) |
//!
//! The bound below sits between the last two, well clear of both.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::fec::ldpc::bp::{BpScratch, bp_llr_zsum_with_scratch};
use mfsk_core::fec::ldpc::osd::osd_decode_npre1;
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

#[test]
fn osd_on_noise_returns_wrong_codewords_at_the_reference_rate() {
    const THREADS: usize = 12;
    const PER_THREAD: usize = 17_000; // 204 000 draws
    let crc_valid: u64 = std::thread::scope(|sc| {
        let hs: Vec<_> = (0..THREADS)
            .map(|t| {
                sc.spawn(move || {
                    let mut rng = Rng(0x9e37_79b9_7f4a_7c15u64 ^ (t as u64 + 1001));
                    let mut scratch = BpScratch::<Ldpc174_91Params, f32>::new();
                    let mut hits = 0u64;
                    for _ in 0..PER_THREAD {
                        let mut llr = [0f32; 174];
                        for v in llr.iter_mut() {
                            *v = 2.83 * rng.gauss();
                        }
                        for n_iter in [1u32, 2] {
                            let z = bp_llr_zsum_with_scratch::<Ldpc174_91Params>(
                                &mut scratch,
                                &llr,
                                n_iter,
                            );
                            let mut zz = [0f32; 174];
                            zz.copy_from_slice(z);
                            if osd_decode_npre1(&zz).is_some() {
                                hits += 1;
                                break;
                            }
                        }
                    }
                    hits
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).sum()
    });
    // 204 000 draws: about 18 expected at 9e-5, about 470 at the old 2.3e-3.
    assert!(
        crc_valid <= 70,
        "{crc_valid} wrong codewords in 204 000 noise draws (reference rate ≈ 5.8e-5, i.e. ≈ 12); \
         the OSD is checking the CRC on every candidate again?"
    );
}
