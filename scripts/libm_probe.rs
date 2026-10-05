// SPDX-License-Identifier: GPL-3.0-only
//! Does this machine's libm give the same f32 results as another's? (#579)
//!
//! `decode_snapshot` pins f32 bit patterns, and the decoder's SNR goes through
//! `f32::log10` and `f32::powf`, which call the platform libm (glibc, Apple's,
//! musl's...), not code in this repository. This probe feeds those functions a fixed
//! input list, built with IEEE basic operations only (so the inputs are identical
//! everywhere), and prints one hash per function. Different hashes on two machines
//! mean the libm differs; the `dump` / `cmp` modes then show how many values differ
//! and by how many ULP.
//!
//!     rustc --edition 2021 -O -o libm_probe scripts/libm_probe.rs
//!     ./libm_probe                  # one line per function, plus the machine
//!     ./libm_probe dump DIR         # every result as f32 bits, DIR/<fn>.bin (4 MB each)
//!     ./libm_probe cmp DIR_A DIR_B  # differing values and the largest ULP gap, per function
//!
//! `sqrt` is a control: IEEE requires it exact, so it must match on every machine.
//! Build with the same `rustc` flags on both machines (plain `-O`): no target-cpu.

use std::fs;
use std::path::Path;

const N: usize = 1_000_000;

/// xorshift64*, integer only.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// A float in [lo, hi): 24 random bits are exact in f32, the rest is IEEE basic ops.
    fn uniform(&mut self, lo: f32, hi: f32) -> f32 {
        let u = (self.next() >> 40) as f32 / 16_777_216.0;
        lo + u * (hi - lo)
    }
    /// A positive float with a random mantissa and an exponent in [e_lo, e_hi].
    fn log_uniform(&mut self, e_lo: i32, e_hi: i32) -> f32 {
        let r = self.next();
        let e = e_lo + ((r >> 41) % ((e_hi - e_lo + 1) as u64)) as i32;
        f32::from_bits((((127 + e) as u32) << 23) | (r as u32 & 0x007f_ffff))
    }
}

type Case = (&'static str, fn(&mut Rng) -> f32, fn(f32) -> f32);

fn cases() -> Vec<Case> {
    vec![
        // The SNR path: 10*log10(ratio) - cal, and 10^(0.1*(sbase-40)).
        ("log10", |r| r.log_uniform(-20, 20), |x| x.log10()),
        ("powf10", |r| r.uniform(-8.0, 8.0), |x| 10f32.powf(0.1 * x)),
        // The rest of what the decoders call.
        ("ln", |r| r.log_uniform(-20, 20), |x| x.ln()),
        ("exp", |r| r.uniform(-20.0, 20.0), |x| x.exp()),
        ("sin", |r| r.uniform(-1000.0, 1000.0), |x| x.sin()),
        ("cos", |r| r.uniform(-1000.0, 1000.0), |x| x.cos()),
        ("atan2", |r| r.uniform(-8.0, 8.0), |x| x.atan2(1.0 + 0.5 * x)),
        // Control: exact by IEEE 754.
        ("sqrt", |r| r.log_uniform(-20, 20), |x| x.sqrt()),
    ]
}

fn results(c: &Case) -> Vec<u32> {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    (0..N).map(|_| (c.2)((c.1)(&mut rng)).to_bits()).collect()
}

fn fnv(bits: &[u32]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bits {
        for byte in b.to_le_bytes() {
            h = (h ^ byte as u64).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    h
}

/// Distance in representable f32 values (same sign, as all these are near each other).
fn ulp_gap(a: u32, b: u32) -> u32 {
    let key = |x: u32| if x >> 31 == 1 { !x } else { x | 0x8000_0000 };
    key(a).abs_diff(key(b))
}

fn read(dir: &Path, name: &str) -> Vec<u32> {
    let bytes = fs::read(dir.join(format!("{name}.bin"))).unwrap_or_else(|e| panic!("{}: {e}", dir.display()));
    bytes.chunks_exact(4).map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("dump") => {
            let dir = Path::new(args.get(1).expect("dump DIR"));
            fs::create_dir_all(dir).unwrap();
            for c in cases() {
                let bytes: Vec<u8> = results(&c).iter().flat_map(|b| b.to_le_bytes()).collect();
                fs::write(dir.join(format!("{}.bin", c.0)), bytes).unwrap();
            }
        }
        Some("cmp") => {
            let (a, b) = (Path::new(args.get(1).expect("cmp A B")), Path::new(args.get(2).expect("cmp A B")));
            for c in cases() {
                let (x, y) = (read(a, c.0), read(b, c.0));
                let diff: Vec<u32> = x.iter().zip(&y).filter(|(p, q)| p != q).map(|(p, q)| ulp_gap(*p, *q)).collect();
                println!(
                    "{:7} {:7} of {} differ, largest gap {} ULP",
                    c.0,
                    diff.len(),
                    x.len(),
                    diff.iter().max().copied().unwrap_or(0)
                );
            }
        }
        _ => {
            println!("machine {} {}", std::env::consts::ARCH, std::env::consts::OS);
            for c in cases() {
                let r = results(&c);
                println!("{:7} {:016x}  first {:08x} {:08x} {:08x}", c.0, fnv(&r), r[0], r[1], r[2]);
            }
        }
    }
}
