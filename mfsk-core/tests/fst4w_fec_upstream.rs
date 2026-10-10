//! `fec::ldpc240_74::decode240_74` against WSJT-X's own `decode240_74_owned`
//! (FST4W, `v3.3.0-beta1`; #649): every recorded input must give upstream's
//! answer — the same codeword, `ntype` and `nharderror`, `dmin` to 1e-4
//! relative, and no result where upstream returns none.
//!
//! The inputs under `embedded-poc/assets/golden/fst4w/fec74_in.bin` are
//! noisy codewords of random payloads (BPSK LLRs at several noise levels),
//! pure-noise words, and a-priori-masked words, at the settings
//! `fst4_decode.f90` uses (`Keff` 66 with `maxosd` 2 / `norder` 3, `Keff` 50 with
//! `maxosd` 1 / `norder` 4) and the other `maxosd` / `norder` the routine
//! accepts. The outputs are upstream's, from `scripts/osd-upstream/run.sh 74`
//! (gfortran, release flags); re-running it on the inputs reproduces them byte
//! for byte. `regenerate_inputs` (ignored) rewrites the inputs from a fixed seed.

#[allow(dead_code)]
mod common;

use mfsk_core::fec::ldpc240_74::{Osd74Work, decode240_74, encode};

const IN: &str = asset_path!("golden/fst4w/fec74_in.bin");
const OUT: &str = asset_path!("golden/fst4w/fec74_out.bin");
const N: usize = 240;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn uniform(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        let (a, b) = (self.uniform(), self.uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * core::f64::consts::PI * b).cos()
    }
}

struct Rec {
    keff: i32,
    maxosd: i32,
    norder: i32,
    mask: Vec<i8>,
    llr: Vec<f32>,
}

fn make_records() -> Vec<Rec> {
    let mut rng = Rng(0x0F57_4A11_0649_u64);
    let mut v = Vec::new();
    // (keff, maxosd, norder)
    let settings = [
        (66, 2, 3),
        (50, 1, 4),
        (66, 0, 3),
        (66, 3, 2),
        (66, 1, 1),
        (66, -1, 3),
        (66, 2, 4),
        (50, 2, 3),
        (74, 2, 3),
    ];
    let sigmas = [0.55f64, 0.65, 0.75, 0.82, 0.9, 0.98, 1.06, 1.2];
    for (si, &(keff, maxosd, norder)) in settings.iter().enumerate() {
        for (gi, &sigma) in sigmas.iter().enumerate() {
            let reps = if si < 2 { 30 } else { 4 };
            for rep in 0..reps {
                let mut p = [0u8; 50];
                for b in p.iter_mut() {
                    *b = (rng.next() >> 40) as u8 & 1;
                }
                let info = mfsk_core::fec::ldpc240_74::append_crc24_50(&p);
                let cw = encode(&info);
                // Every ninth record is a-priori masked: first 20 bits locked.
                let masked = (gi * 7 + rep) % 9 == 8;
                let mut mask = vec![0i8; N];
                let llr: Vec<f32> = cw
                    .iter()
                    .enumerate()
                    .map(|(i, &b)| {
                        let x = if b == 1 { 1.0 } else { -1.0 };
                        let y = x + sigma * rng.gauss();
                        let mut l = (2.0 * y / (sigma * sigma)) as f32;
                        if masked && i < 20 {
                            mask[i] = 1;
                            l = if b == 1 { 12.0 } else { -12.0 };
                        }
                        l
                    })
                    .collect();
                v.push(Rec {
                    keff,
                    maxosd,
                    norder,
                    mask,
                    llr,
                });
            }
        }
        // Pure noise: no codeword anywhere near.
        for _ in 0..6 {
            let llr = (0..N).map(|_| (3.0 * rng.gauss()) as f32).collect();
            v.push(Rec {
                keff,
                maxosd,
                norder,
                mask: vec![0; N],
                llr,
            });
        }
    }
    v
}

fn write_inputs(recs: &[Rec]) -> Vec<u8> {
    let mut b = (recs.len() as i32).to_le_bytes().to_vec();
    for r in recs {
        b.extend(r.keff.to_le_bytes());
        b.extend(r.maxosd.to_le_bytes());
        b.extend(r.norder.to_le_bytes());
        b.extend(r.mask.iter().map(|&x| x as u8));
        for l in &r.llr {
            b.extend(l.to_le_bytes());
        }
    }
    b
}

fn read_inputs(d: &[u8]) -> Vec<Rec> {
    let count = i32::from_le_bytes(d[0..4].try_into().unwrap()) as usize;
    let rl = 12 + N + 4 * N;
    assert_eq!(d.len(), 4 + count * rl);
    (0..count)
        .map(|i| {
            let r = &d[4 + i * rl..4 + (i + 1) * rl];
            let g = |o: usize| i32::from_le_bytes(r[o..o + 4].try_into().unwrap());
            Rec {
                keff: g(0),
                maxosd: g(4),
                norder: g(8),
                mask: r[12..12 + N].iter().map(|&x| x as i8).collect(),
                llr: r[12 + N..]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|c| f32::from_le_bytes(*c))
                    .collect(),
            }
        })
        .collect()
}

/// Rewrites the inputs. Then run `scripts/osd-upstream/run.sh` for the outputs.
#[test]
#[ignore = "rewrites the vendored inputs"]
fn regenerate_inputs() {
    std::fs::write(IN, write_inputs(&make_records())).unwrap();
}

#[test]
fn decode240_74_matches_upstream() {
    let (Ok(din), Ok(dout)) = (std::fs::read(IN), std::fs::read(OUT)) else {
        assert!(
            std::env::var("MFSK_REQUIRE_CORPUS").is_err(),
            "fec74 fixtures missing"
        );
        eprintln!("skipping: fec74 fixtures missing");
        return;
    };
    let recs = read_inputs(&din);
    let ro = 8 + 4 + N + 74;
    assert_eq!(dout.len(), recs.len() * ro);
    let mut work = Osd74Work::new();
    let mut by_ntype = [0usize; 5];
    for (i, r) in recs.iter().enumerate() {
        let q = &dout[i * ro..(i + 1) * ro];
        let ntype = i32::from_le_bytes(q[0..4].try_into().unwrap());
        let nhe = i32::from_le_bytes(q[4..8].try_into().unwrap());
        let dmin = f32::from_le_bytes(q[8..12].try_into().unwrap());
        let cw = &q[12..12 + N];
        let mask: Vec<bool> = r.mask.iter().map(|&x| x != 0).collect();
        let ap = mask.iter().any(|&b| b).then_some(&mask[..]);
        let got = decode240_74(&mut work, &r.llr, r.keff as usize, r.maxosd, r.norder, ap);
        let what = format!(
            "#{i} (Keff {} maxosd {} norder {})",
            r.keff, r.maxosd, r.norder
        );
        if ntype > 0 {
            by_ntype[ntype as usize] += 1;
            let g = got.unwrap_or_else(|| {
                panic!("{what}: upstream decodes (ntype {ntype}), this does not")
            });
            assert_eq!(g.cw[..], *cw, "{what}: another codeword");
            assert_eq!(g.ntype as i32, ntype, "{what}: ntype");
            assert_eq!(g.nharderror as i32, nhe, "{what}: nharderror");
            let tol = 1e-4 * dmin.abs().max(1e-3);
            assert!(
                (g.dmin - dmin).abs() <= tol,
                "{what}: dmin {} vs {dmin}",
                g.dmin
            );
        } else {
            by_ntype[0] += 1;
            assert!(got.is_none(), "{what}: upstream fails, this decodes");
        }
    }
    // The fixture must exercise BP, both OSD snapshots, and failure.
    eprintln!("ntype histogram (0=fail,1=BP,2..4=OSD i): {by_ntype:?}");
    assert!(by_ntype[0] > 0 && by_ntype[1] > 0 && by_ntype[2] > 0 && by_ntype[3] > 0);
}
