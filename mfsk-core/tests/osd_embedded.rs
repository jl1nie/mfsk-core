//! `fec::ldpc::osd_emb::osd174_91_into` (the embedded OSD) against `osd_npre::osd174_91`, which is the port pinned to WSJT-X's own
//! `osd174_91` by `tests/osd_upstream_fixture.rs`: on every input the same winner codeword, the same `hard_errors`, and `None`
//! exactly when `osd174_91` is `None`.
//!
//! Inputs: (1) every `ndeep = 2` LLR of the upstream fixture, with its a-priori mask (LLRs that reached OSD in real decodes, and
//! upstream's answers, which the embedded one must give too); (2) 20 000 synthetic words, a CRC-valid codeword through a noise ladder
//! from clean to hopeless, with LLR magnitudes drawn from a few levels (so equal magnitudes, which the reliability sort can break
//! either way, occur all the time), with and without a mask of random locked bits; (3) pure noise (almost always `None`).
//! **Fails if** any input differs, or if the set exercises only one outcome. Each of these was put into `osd_emb.rs` once and seen to fail
//! it: a distance that stops early (`>= dmin - 1.0`), a stable sort (ties break differently), a one-bit-short `npre1` window (`NT = 39`),
//! `ntheta = 11`.

use mfsk_core::fec::ldpc::bp::append_crc14;
use mfsk_core::fec::ldpc::osd_emb::{Osd174Scratch, osd174_91_into};
use mfsk_core::fec::ldpc::osd_npre::{NDEEP2_174_91, osd174_91};
use mfsk_core::fec::ldpc::{LDPC_N, ldpc_encode};

const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/osd_upstream");

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
    /// A roughly normal draw (sum of four uniforms).
    fn normal(&mut self) -> f32 {
        (self.unit() + self.unit() + self.unit() + self.unit() - 2.0) * 1.732
    }
}

/// Runs both on one input; returns whether it decoded.
fn same(
    sc: &mut Osd174Scratch,
    llr: &[f32; LDPC_N],
    mask: Option<&[bool; LDPC_N]>,
    what: &str,
) -> bool {
    let want = osd174_91(llr, NDEEP2_174_91, mask);
    let got = osd174_91_into(sc, llr, mask);
    match (want, got) {
        (None, None) => false,
        (Some(w), Some(g)) => {
            assert_eq!(
                &w.codeword[..],
                &sc.codeword()[..],
                "{what}: another codeword"
            );
            assert_eq!(&w.info[..], &sc.codeword()[..91], "{what}: info");
            assert_eq!(w.message77, g.message77, "{what}: message");
            assert_eq!(w.hard_errors, g.hard_errors, "{what}: hard errors");
            true
        }
        (Some(_), None) => panic!("{what}: osd174_91 decodes, the embedded one does not"),
        (None, Some(_)) => panic!("{what}: the embedded one decodes, osd174_91 does not"),
    }
}

#[test]
fn the_upstream_fixture_inputs_give_upstream_answers() {
    let d = std::fs::read(format!("{DIR}/osd174_91_in.bin")).unwrap();
    let o = std::fs::read(format!("{DIR}/osd174_91_out.bin")).unwrap();
    let n = LDPC_N;
    let count = i32::from_le_bytes(d[0..4].try_into().unwrap()) as usize;
    let (ri, ro) = (4 + n + 4 * n, 8 + n);
    let mut sc = Osd174Scratch::new();
    let (mut seen, mut decoded) = (0, 0);
    for i in 0..count {
        let r = &d[4 + i * ri..4 + (i + 1) * ri];
        let q = &o[i * ro..(i + 1) * ro];
        if i32::from_le_bytes(r[0..4].try_into().unwrap()) != 2 {
            continue;
        }
        let mut mask = [false; LDPC_N];
        for (m, &b) in mask.iter_mut().zip(&r[4..4 + n]) {
            *m = b != 0;
        }
        let mut llr = [0f32; LDPC_N];
        for (x, c) in llr.iter_mut().zip(r[4 + n..].as_chunks::<4>().0) {
            *x = f32::from_le_bytes(*c);
        }
        let mask = mask.iter().any(|&b| b).then_some(&mask);
        let nhardmin = i32::from_le_bytes(q[0..4].try_into().unwrap());
        let got = osd174_91_into(&mut sc, &llr, mask);
        seen += 1;
        if nhardmin >= 0 {
            decoded += 1;
            let g = got.unwrap_or_else(|| panic!("fixture #{i}: upstream decodes, this does not"));
            assert_eq!(
                &q[8..],
                &sc.codeword()[..],
                "fixture #{i}: another codeword"
            );
            assert_eq!(g.hard_errors as i32, nhardmin, "fixture #{i}: hard errors");
        } else {
            assert!(
                got.is_none(),
                "fixture #{i}: upstream's winner fails its CRC"
            );
        }
        same(&mut sc, &llr, mask, &format!("fixture #{i}"));
    }
    assert!(
        seen > 100 && decoded > 0 && decoded < seen,
        "{decoded}/{seen}"
    );
}

#[test]
fn synthetic_words_and_noise_give_the_same_answer() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let mut sc = Osd174Scratch::new();
    let (mut decoded, mut total) = (0u32, 0u32);
    for t in 0..20_000u32 {
        let mut msg = [0u8; 77];
        for b in msg.iter_mut() {
            *b = (rng.next() & 1) as u8;
        }
        let cw = ldpc_encode(&append_crc14(&msg));
        // Noise from clean (sigma 0.3) to hopeless (sigma 2.2) across the set.
        let sigma = 0.3 + 1.9 * (t % 20) as f32 / 19.0;
        let levels = t % 3; // 0: continuous, 1: four magnitude levels, 2: two
        let mut llr = [0f32; LDPC_N];
        for i in 0..LDPC_N {
            let s = if cw[i] == 1 { 1.0 } else { -1.0 };
            let x = s + sigma * rng.normal();
            llr[i] = match levels {
                0 => x * 4.0,
                1 => (x * 2.0).round().clamp(-3.0, 3.0),
                _ => {
                    if x >= 0.0 {
                        2.0
                    } else {
                        -2.0
                    }
                }
            };
        }
        // Every third word carries a mask of up to 60 locked bits (the lock does not need to be right).
        let mut mask = [false; LDPC_N];
        let locked = if t % 3 == 0 {
            (rng.next() % 61) as usize
        } else {
            0
        };
        for _ in 0..locked {
            mask[(rng.next() % LDPC_N as u64) as usize] = true;
        }
        let m = mask.iter().any(|&b| b).then_some(&mask);
        total += 1;
        decoded += same(&mut sc, &llr, m, &format!("synthetic #{t}")) as u32;
    }
    for t in 0..2_000u32 {
        let mut llr = [0f32; LDPC_N];
        for x in llr.iter_mut() {
            *x = rng.normal() * 3.0;
        }
        total += 1;
        decoded += same(&mut sc, &llr, None, &format!("noise #{t}")) as u32;
    }
    assert!(
        decoded > 3_000 && decoded < total - 3_000,
        "both outcomes are exercised: {decoded} of {total} decoded"
    );
}
