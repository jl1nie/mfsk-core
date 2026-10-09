//! `fec::ldpc::osd_npre` against WSJT-X's own `osd174_91` / `osd240_101`
//! (#417): every recorded input must give upstream's answer — the same
//! codeword when upstream's winner passes its CRC, no result when it fails, and
//! `hard_errors` equal to upstream's `nhardmin`.
//!
//! The inputs under `tests/fixtures/osd_upstream/` are LLRs that reached OSD
//! while decoding real audio (five FT8 recordings and the FT4 golden at
//! `Depth::Deep`; the FST4-60 golden and fifteen FST4-60 sweep recordings near
//! threshold), every one upstream decodes plus a sample of the rest, and
//! synthetic codewords with and without an a-priori mask, at `ndeep` 2 and 3.
//! The outputs are upstream's, from `scripts/osd-upstream/run.sh` (gfortran,
//! release flags, v3.2.0-rc1); re-running it on the inputs reproduces them
//! byte for byte. The full set behind the subset (16 230 + 2 630 inputs)
//! agreed as well.

use mfsk_core::fec::ldpc::bp::check_crc14;
use mfsk_core::fec::ldpc::osd::PartialCrc;
use mfsk_core::fec::ldpc::osd_npre::{NDEEP2_174_91, NDEEP3_174_91, NpreDepth, osd_npre};
use mfsk_core::fec::ldpc::{Ldpc174_91Params, Ldpc240_101Params};
use mfsk_core::fec::ldpc240_101::append_crc24;

const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/osd_upstream");

/// `fst4_decode.f90:478`: `Keff = 91`.
const FST4_PC: PartialCrc = PartialCrc {
    keff: 91,
    with_crc: append_crc24,
};

fn crc24_ok(info: &[u8]) -> bool {
    let mut m = [0u8; 77];
    m.copy_from_slice(&info[..77]);
    append_crc24(&m)[..] == info[..101]
}

struct Case {
    ndeep: i32,
    mask: Vec<bool>,
    llr: Vec<f32>,
    nhardmin: i32,
    cw: Vec<u8>,
}

fn load(name: &str, n: usize) -> Vec<Case> {
    let d = std::fs::read(format!("{DIR}/{name}_in.bin")).unwrap();
    let o = std::fs::read(format!("{DIR}/{name}_out.bin")).unwrap();
    let count = i32::from_le_bytes(d[0..4].try_into().unwrap()) as usize;
    let (ri, ro) = (4 + n + 4 * n, 8 + n);
    assert_eq!(d.len(), 4 + count * ri);
    assert_eq!(o.len(), count * ro);
    (0..count)
        .map(|i| {
            let r = &d[4 + i * ri..4 + (i + 1) * ri];
            let q = &o[i * ro..(i + 1) * ro];
            Case {
                ndeep: i32::from_le_bytes(r[0..4].try_into().unwrap()),
                mask: r[4..4 + n].iter().map(|&b| b != 0).collect(),
                llr: r[4 + n..]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|c| f32::from_le_bytes(*c))
                    .collect(),
                nhardmin: i32::from_le_bytes(q[0..4].try_into().unwrap()),
                cw: q[8..].to_vec(),
            }
        })
        .collect()
}

fn check(name: &str, n: usize, run: impl Fn(&Case, Option<&[bool]>) -> Option<(Vec<u8>, u32)>) {
    let cases = load(name, n);
    let mut decoded = 0;
    for (i, c) in cases.iter().enumerate() {
        let mask = c.mask.iter().any(|&b| b).then_some(&c.mask[..]);
        let got = run(c, mask);
        if c.nhardmin >= 0 {
            decoded += 1;
            let (cw, he) = got.unwrap_or_else(|| {
                panic!(
                    "{name} #{i} (ndeep {}): upstream decodes, this does not",
                    c.ndeep
                )
            });
            assert_eq!(
                cw, c.cw,
                "{name} #{i} (ndeep {}): another codeword",
                c.ndeep
            );
            assert_eq!(he as i32, c.nhardmin, "{name} #{i}: hard errors");
        } else {
            assert!(
                got.is_none(),
                "{name} #{i} (ndeep {}): upstream's winner fails its CRC, this decodes",
                c.ndeep
            );
        }
    }
    // The fixture must exercise both outcomes.
    assert!(
        decoded > 0 && decoded < cases.len(),
        "{name}: {decoded}/{}",
        cases.len()
    );
}

#[test]
fn osd174_91_matches_upstream() {
    check("osd174_91", 174, |c, mask| {
        let depth = if c.ndeep == 3 {
            NDEEP3_174_91
        } else {
            NDEEP2_174_91
        };
        osd_npre::<Ldpc174_91Params>(&c.llr, depth, None, mask, Some(check_crc14))
            .map(|r| (r.codeword, r.hard_errors))
    });
}

#[test]
fn osd240_101_matches_upstream() {
    check("osd240_101", 240, |c, mask| {
        // `osd240_101.f90`: `ntheta = 12` at both depths, `ntau = 14` at 3.
        let depth = NpreDepth {
            ntheta: 12,
            npre2: c.ndeep == 3,
            ntau: 14,
        };
        osd_npre::<Ldpc240_101Params>(&c.llr, depth, Some(FST4_PC), mask, Some(crc24_ok))
            .map(|r| (r.codeword, r.hard_errors))
    });
}
