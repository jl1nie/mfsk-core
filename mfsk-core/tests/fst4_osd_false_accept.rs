//! FST4's OSD on noise: how often does `Ldpc240_101::decode_soft` return a CRC-valid codeword from
//! iid Gaussian LLRs (#456)? A measurement, not a gate: it prints the count and asserts nothing.
//! `#[ignore]` because depth 3 takes about 25 s per 6 000 draws.
//!
//! ```sh
//! N=6000 DEPTH=3 cargo test --release -p mfsk-core --features full,internal-testing \
//!     --test fst4_osd_false_accept -- --ignored --nocapture
//! ```
//!
//! Measured for #456 (sd 2.83, `verify_info` = `check_crc24`): depth 3, CRC checked on every
//! candidate: 35 in 24 000; CRC on the winner (as `osd240_101.f90`): 0 in 6 000; depth 2: 0 in
//! 3 600 both ways. `scripts/osd-false-rate/build.sh <wsjtx>/lib 240 ...` runs the upstream
//! routine on the same kind of input.
#![cfg(all(feature = "fst4", feature = "internal-testing"))]
use mfsk_core::engine::protocol::{FecCodec, FecOpts};
use mfsk_core::fec::ldpc240_101::{Ldpc240_101, check_crc24};

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
#[ignore = "measurement: prints the false-accept count, asserts nothing"]
fn fst4_osd_false_accept_rate() {
    let n: usize = std::env::var("N")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(6_000);
    let depth: u32 = std::env::var("DEPTH")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(3);
    let threads = 12usize;
    let per = n / threads;
    let hits: u64 = std::thread::scope(|sc| {
        let hs: Vec<_> = (0..threads)
            .map(|t| {
                sc.spawn(move || {
                    let mut rng = Rng(0x9e37_79b9_7f4a_7c15u64 ^ (t as u64 + 77));
                    let opts = FecOpts {
                        osd_depth: depth,
                        verify_info: Some(check_crc24),
                        ..FecOpts::default()
                    };
                    let mut hits = 0u64;
                    for _ in 0..per {
                        let mut llr = [0f32; 240];
                        for v in llr.iter_mut() {
                            *v = 2.83 * rng.gauss();
                        }
                        if Ldpc240_101.decode_soft(&llr, &opts).is_some() {
                            hits += 1;
                        }
                    }
                    hits
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).sum()
    });
    println!(
        "FALSERATE240 n={} depth={depth} crc_pass={hits}",
        per * threads
    );
}
