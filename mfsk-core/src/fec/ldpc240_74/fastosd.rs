//! Ordered-statistics decoder for the (240,74) code, ported line for line from
//! WSJT-X `lib/fst4/fastosd240_74.f90` at `v3.3.0-beta1` (`fastosd240_74_owned`,
//! `mrbencode74`, `partial_syndrome`, `nextpat74`) and `lib/indexx.f90`.
//!
//! This is **not** [`crate::fec::ldpc::osd_npre`]: that is `osd240_101`'s
//! algorithm (one basis, `npre1`/`npre2` hashing). This one runs two bases (the
//! second swaps `NSWAP` = 20 columns at the MRB boundary), enumerates error
//! patterns of weight 1..=`nord` with `nextpat74`, bounds them by `rho·dmin`,
//! and prefilters each with a 32-bit partial syndrome. It follows the #417
//! standard — the same arithmetic, in the same order, so that the Fortran
//! oracle (`scripts/osd-upstream/run.sh`, `driver240_74.f90`) matches exactly.
//! The reliability sort is `indexx` itself, so equal magnitudes order as they
//! do upstream.
//!
//! The generator for `k` free bits (`Keff`): row `i ≤ 50` is message bit `i`
//! followed by its CRC with bits `51..=k` zeroed; rows `51..=k` are unit
//! vectors. With `k = 66` the first 16 CRC bits are searched and the last 8
//! cascaded into the code; with `k = 50` all 24 are cascaded and every
//! candidate satisfies the CRC (`fastosd240_74.f90:1-17`, "No CRC in this
//! mode"). The generator is cached per `k` in [`Osd74Work`], as upstream's
//! `work%generator(k)`.
//!
//! What differs from the Fortran, deliberately: nothing in the arithmetic. The
//! per-pattern bookkeeping is restructured (#670): the partial syndrome is the
//! xor of packed `u32` columns of the pattern's set bits, and the pattern is a
//! position list; both give the same values, in the same order.
//! `apmask` is not a parameter because upstream only permutes it
//! (`apmaskr`) and never reads the result.

use alloc::vec;
use alloc::vec::Vec;

use super::{LDPC_K, LDPC_N, crc24, encode};

const N: usize = LDPC_N;
/// Columns swapped at the MRB boundary for the second basis
/// (`fastosd240_74.f90:80`).
const NSWAP: usize = 20;
/// Partial-syndrome length (`fastosd240_74.f90:191`).
const NP: usize = 32;

/// Per-`k` generator cache: upstream's `fst4_osd_workspace_type%generator`.
#[derive(Default)]
pub struct Osd74Work {
    /// `generator[k]` is `k` rows of `N` bits, row-major.
    generator: Vec<Option<Vec<u8>>>,
}

impl Osd74Work {
    pub fn new() -> Self {
        Self {
            generator: vec![None; LDPC_K + 1],
        }
    }

    fn generator(&mut self, k: usize) -> &[u8] {
        if self.generator.len() <= LDPC_K {
            self.generator.resize(LDPC_K + 1, None);
        }
        self.generator[k].get_or_insert_with(|| {
            // fastosd240_74.f90:48-72
            let mut gen_ = vec![0u8; k * N];
            for i in 0..k {
                let mut message74 = [0u8; LDPC_K];
                message74[i] = 1;
                if i < 50 {
                    let ncrc24 = crc24(&message74);
                    for b in 0..24 {
                        message74[50 + b] = ((ncrc24 >> (23 - b)) & 1) as u8;
                    }
                    for x in message74.iter_mut().take(k).skip(50) {
                        *x = 0;
                    }
                }
                gen_[i * N..(i + 1) * N].copy_from_slice(&encode(&message74));
            }
            gen_
        })
    }
}

/// `lib/indexx.f90`: indices that sort `arr` ascending (Numerical Recipes'
/// quicksort with an insertion-sort tail, not stable). Returned 0-based.
pub(crate) fn indexx(arr: &[f32]) -> Vec<usize> {
    const M: usize = 7;
    const NSTACK: usize = 50;
    let n = arr.len();
    // 1-based, as the Fortran.
    let mut indx: Vec<usize> = (0..=n).collect();
    let a_ = |indx: &Vec<usize>, i: usize| arr[indx[i] - 1];
    let mut istack = [0usize; NSTACK + 1];
    let mut jstack = 0usize;
    let mut l = 1usize;
    let mut ir = n;
    loop {
        if (ir as i64) - (l as i64) < M as i64 {
            for j in l + 1..=ir {
                let indxt = indx[j];
                let a = arr[indxt - 1];
                let mut i = j - 1;
                loop {
                    if i == 0 {
                        break;
                    }
                    if a_(&indx, i) <= a {
                        break;
                    }
                    indx[i + 1] = indx[i];
                    i -= 1;
                }
                indx[i + 1] = indxt;
            }
            if jstack == 0 {
                break;
            }
            ir = istack[jstack];
            l = istack[jstack - 1];
            jstack -= 2;
        } else {
            let k = (l + ir) / 2;
            indx.swap(k, l + 1);
            if a_(&indx, l + 1) > a_(&indx, ir) {
                indx.swap(l + 1, ir);
            }
            if a_(&indx, l) > a_(&indx, ir) {
                indx.swap(l, ir);
            }
            if a_(&indx, l + 1) > a_(&indx, l) {
                indx.swap(l + 1, l);
            }
            let mut i = l + 1;
            let mut j = ir;
            let indxt = indx[l];
            let a = arr[indxt - 1];
            loop {
                loop {
                    i += 1;
                    if a_(&indx, i) >= a {
                        break;
                    }
                }
                loop {
                    j -= 1;
                    if a_(&indx, j) <= a {
                        break;
                    }
                }
                if j < i {
                    break;
                }
                indx.swap(i, j);
            }
            indx[l] = indx[j];
            indx[j] = indxt;
            jstack += 2;
            assert!(jstack <= NSTACK, "NSTACK too small in indexx");
            if (ir as i64) - (i as i64) + 1 >= (j as i64) - (l as i64) {
                istack[jstack] = ir;
                istack[jstack - 1] = i;
                ir = j - 1;
            } else {
                istack[jstack] = j - 1;
                istack[jstack - 1] = l;
                l = i;
            }
        }
    }
    indx[1..].iter().map(|&x| x - 1).collect()
}

/// Result of [`fastosd240_74`].
pub struct FastOsd74 {
    /// First 74 bits of the winning codeword, original bit order.
    pub message74: [u8; LDPC_K],
    pub cw: [u8; N],
    /// Hamming distance of the winner to the hard decisions of the input;
    /// **negative** when neither basis' winner passed the CRC
    /// (`fastosd240_74.f90:265`).
    pub nhardmin: i32,
    pub dmin: f32,
}

/// `fastosd240_74_owned(work, llr, k, apmask, ndeep, message74, cw, nhardmin, dmin)`.
pub fn fastosd240_74(work: &mut Osd74Work, llr: &[f32], k: usize, ndeep: i32) -> FastOsd74 {
    assert_eq!(llr.len(), N);
    assert!((50..=LDPC_K).contains(&k));
    let gen_ = work.generator(k).to_vec();

    let mut out = FastOsd74 {
        message74: [0; LDPC_K],
        cw: [0; N],
        nhardmin: 0,
        dmin: 0.0,
    };
    let mut ndeep = ndeep;

    for ibasis in 1..=2 {
        // Hard decisions on the received word.
        let mut hdec = [0u8; N];
        for i in 0..N {
            hdec[i] = u8::from(llr[i] >= 0.0);
        }
        let absrx_full: Vec<f32> = llr.iter().map(|x| x.abs()).collect();
        let indx = indexx(&absrx_full);

        // Columns of the generator in order of decreasing reliability.
        let mut gm = vec![0u8; k * N]; // genmrb(row, col), row-major
        let mut indices = [0usize; N];
        for i in 0..N {
            let src = indx[N - 1 - i];
            for r in 0..k {
                gm[r * N + i] = gen_[r * N + src];
            }
            indices[i] = src;
        }

        if ibasis == 2 {
            for i in (k - NSWAP)..k {
                for r in 0..k {
                    gm.swap(r * N + i, r * N + i + NSWAP);
                }
                indices.swap(i, i + NSWAP);
            }
        }

        // Gaussian elimination: most reliable bits in positions 1:k.
        let mut icol = 0usize;
        let mut indices2 = [0usize; N];
        let mut nskipped = 0usize;
        for id in 0..k {
            let mut iflag = false;
            while !iflag {
                if gm[id * N + icol] != 1 {
                    for j in id + 1..k {
                        if gm[j * N + icol] == 1 {
                            for c in 0..N {
                                gm.swap(id * N + c, j * N + c);
                            }
                            iflag = true;
                        }
                    }
                    if !iflag {
                        // skip this column
                        nskipped += 1;
                        indices2[k + nskipped - 1] = icol;
                        icol += 1;
                    }
                } else {
                    iflag = true;
                }
            }
            indices2[id] = icol;
            for j in 0..k {
                if id != j && gm[j * N + icol] == 1 {
                    for c in 0..N {
                        gm[j * N + c] ^= gm[id * N + c];
                    }
                }
            }
            icol += 1;
        }
        for (i, x) in indices2.iter_mut().enumerate().skip(k + nskipped) {
            *x = i;
        }
        let mut row = [0u8; N];
        for r in 0..k {
            row.copy_from_slice(&gm[r * N..(r + 1) * N]);
            for c in 0..N {
                gm[r * N + c] = row[indices2[c]];
            }
        }
        let old = indices;
        for i in 0..N {
            indices[i] = old[indices2[i]];
        }

        // Hard decisions, reliabilities in the new order.
        let mut hd = [0u8; N];
        let mut absrx = [0f32; N];
        for i in 0..N {
            hd[i] = hdec[indices[i]];
            absrx[i] = absrx_full[indices[i]];
        }
        let hdec = hd;
        let m0: Vec<u8> = hdec[..k].to_vec();

        let mrbencode = |me: &[u8], cw: &mut [u8; N]| {
            *cw = [0; N];
            for i in 0..k {
                if me[i] == 1 {
                    for c in 0..N {
                        cw[c] ^= gm[i * N + c];
                    }
                }
            }
        };

        let mut c0 = [0u8; N];
        mrbencode(&m0, &mut c0);
        let mut nxor = [0u8; N];
        for i in 0..N {
            nxor[i] = c0[i] ^ hdec[i];
        }
        let mut nhardmin: i32 = nxor.iter().map(|&x| x as i32).sum();
        let mut dmin = 0f32;
        for i in 0..N {
            dmin += nxor[i] as f32 * absrx[i];
        }

        let mut cw = c0;

        'search: {
            if ndeep == 0 {
                break 'search;
            }
            if ndeep > 4 {
                ndeep = 4;
            }
            let (nord, xlambda, nsyndmax) = match ndeep {
                1 => (1usize, 0.0f32, NP as u32),
                2 => (2, 0.0, NP as u32),
                3 => (3, 4.0, 11),
                _ => (4, 3.4, 12),
            };
            let mut s1 = 0f32;
            for x in &absrx[..k] {
                s1 += x;
            }
            let mut s2 = 0f32;
            for x in &absrx[k..N] {
                s2 += x;
            }
            let rho = s1 / (s1 + xlambda * s2);
            let mut rhodmin = rho * dmin;
            let mut me = vec![0u8; k];
            // The partial syndrome of `me = m0 ^ mi` is linear in `me`, so it is
            // the one of `m0` xor the columns of the (at most `nord`) set bits
            // of `mi`: packed as `u32`, 32 columns of rows K..K+NP-1 at a time.
            // Same value as `partial_syndrome`, without its O(k*NP) loop
            // per pattern (#670).
            let col32: Vec<u32> = (0..k)
                .map(|i| {
                    let mut w = 0u32;
                    for p in 0..NP {
                        w |= (gm[i * N + (k - 1) + p] as u32) << p;
                    }
                    w
                })
                .collect();
            let mut hd32 = 0u32;
            for p in 0..NP {
                hd32 |= (hdec[(k - 1) + p] as u32) << p;
            }
            let mut sp0 = 0u32;
            for i in 0..k {
                if m0[i] == 1 {
                    sp0 ^= col32[i];
                }
            }
            // The test pattern `mi` is held as the ascending positions of its
            // `iorder` ones, which is all `nextpat74` ever manipulates.
            let mut pos = [0usize; 4];
            for iorder in 1..=nord {
                for (j, p) in pos[..iorder].iter_mut().enumerate() {
                    *p = k - iorder + j;
                }
                let mut iflag = (k - iorder + 1) as i32;
                while iflag >= 0 {
                    // Set bits of `mi`, ascending: the Fortran's
                    // `sum(mi(i)*absrx(i))` adds exact zeros for the rest.
                    let mut d1 = 0f32;
                    let mut sp = sp0;
                    for &p in &pos[..iorder] {
                        d1 += absrx[p];
                        sp ^= col32[p];
                    }
                    if d1 > rhodmin {
                        break;
                    }
                    let nwhsp = (sp ^ hd32).count_ones();
                    if nwhsp <= nsyndmax {
                        me.copy_from_slice(&m0);
                        for &p in &pos[..iorder] {
                            me[p] ^= 1;
                        }
                        let mut ce = [0u8; N];
                        mrbencode(&me, &mut ce);
                        let mut dd = 0f32;
                        let mut nh = 0i32;
                        for i in 0..N {
                            let x = ce[i] ^ hdec[i];
                            nxor[i] = x;
                            nh += x as i32;
                            dd += x as f32 * absrx[i];
                        }
                        if dd < dmin {
                            dmin = dd;
                            rhodmin = rho * dmin;
                            cw = ce;
                            nhardmin = nh;
                        }
                    }
                    iflag = nextpat74(&mut pos, k, iorder);
                }
            }
        }

        // Re-order the codeword to [message bits][parity bits] format.
        let mut cw_out = [0u8; N];
        for i in 0..N {
            cw_out[indices[i]] = cw[i];
        }
        let mut message74 = [0u8; LDPC_K];
        message74.copy_from_slice(&cw_out[..LDPC_K]);
        out.cw = cw_out;
        out.message74 = message74;
        out.dmin = dmin;
        out.nhardmin = nhardmin;
        if crc24(&message74) == 0 {
            break;
        }
        out.nhardmin = -nhardmin;
    }
    out
}

/// `nextpat74`: the next test error pattern of weight `iorder`, with the
/// pattern held as the ascending positions `pos[..iorder]` of its ones instead
/// of as a 0/1 vector (#670). The Fortran moves the rightmost 1 that has a 0
/// on its left one place left and repacks the 1s that followed it at the end of
/// the vector; both are the same operation on the positions. Returns `iflag`
/// — the 1-based position of the lowest-index 1, or -1 when the last pattern
/// has been generated. `fec/ldpc240_74` is checked against the Fortran's own
/// output (`tests/fst4w_fec_upstream.rs`), which pins the order.
fn nextpat74(pos: &mut [usize; 4], k: usize, iorder: usize) -> i32 {
    let mut j = iorder;
    while j > 0 {
        j -= 1;
        // `mi(i-1) == 0 .and. mi(i) == 1`
        if pos[j] > 0 && (j == 0 || pos[j - 1] + 1 < pos[j]) {
            pos[j] -= 1;
            let nz = iorder - (j + 1);
            for (t, p) in pos[j + 1..iorder].iter_mut().enumerate() {
                *p = k - nz + t;
            }
            return (pos[0] + 1) as i32;
        }
    }
    -1
}
