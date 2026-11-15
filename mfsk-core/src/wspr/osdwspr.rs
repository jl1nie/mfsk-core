//! Ordered-statistics decoding exactly as `wsprd -o n` runs it: a line-for-line port of
//! `lib/wsprd/osdwspr.f90` (with `lib/indexx.f90`, whose tie order decides which columns
//! make the most-reliable basis), `ndeep` 0 to 5.
//!
//! [`super::osd`] is this crate's own OSD (an order-2 sweep with no pruning); it is what
//! the [`Depth`](crate::decoder::Depth) mapping uses and is left as it was. This one exists
//! so that a caller who asks for `wsprd`'s `-o n` gets `wsprd`'s result, down to the
//! hard-error count and the codeword when two candidates tie.
//!
//! Arithmetic is single precision in the Fortran's order (`sum(...)` over `real` is a
//! sequential reduction at `-O3` without `-ffast-math`), and the `ndeep` ladder is the
//! source's:
//!
//! | `ndeep` | `nord` | `npre1` | `npre2` | `ntheta` |
//! |---|---|---|---|---|
//! | 0 | order 0 only | | | |
//! | 1 | 1 | 0 | 0 | 16 |
//! | 2 | 1 | 1 | 0 | 22 |
//! | 3 | 1 | 1 | 1 | 22 |
//! | 4 | 2 | 1 | 1 | 22 |
//! | 5 | 3 | 1 | 1 | 22 |
//!
//! `apmask` is all zero in `wsprd` (`wsprd.c:802`, `calloc`), so the `apmaskr` tests are
//! not ported.

use alloc::vec;
use alloc::vec::Vec;

pub(crate) const N: usize = 162;
pub(crate) const K: usize = 50;
/// `L` of the Fortran: the constraint length, two generator bits per shift.
const L: usize = 32;
/// `nt`: how many parity columns the pruning test looks at.
const NT: usize = 66;
/// `ntau`: how many parity columns the `npre2` pattern boxes use.
const NTAU: usize = 16;

/// The first two rows of the generator matrix, `gg` of `osdwspr.f90`.
const GG: [u8; 2 * L] = [
    1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1,
];

/// What `osdwspr_` returns through its arguments.
#[derive(Clone, Debug)]
pub(crate) struct OsdOut {
    /// The codeword, one bit per byte, in the order the soft symbols came in.
    pub cw: [u8; N],
    /// `nhardmin`.
    pub nhardmin: i32,
    /// `dmin`.
    #[allow(dead_code)]
    pub dmin: f32,
}

/// `gmat(i, :)`: row 1 is `gg` padded with zeros, each next row the previous shifted
/// right by two places, circularly (`cshift(gmat(i-1,:), -2)`).
fn generator() -> [[u8; N]; K] {
    let mut gmat = [[0u8; N]; K];
    gmat[0][..2 * L].copy_from_slice(&GG);
    for i in 1..K {
        for j in 0..N {
            gmat[i][j] = gmat[i - 1][(j + N - 2) % N];
        }
    }
    gmat
}

/// `indexx.f90`: the ascending order of `arr` as 1-based indices, `Numerical Recipes`'
/// quicksort with its insertion sort below `M = 7`. Ported statement for statement,
/// including the insertion loop that runs down to element 1 rather than `l`, because the
/// permutation of equal reliabilities (the clamped soft symbols are many) comes from it.
fn indexx(arr: &[f32; N]) -> [usize; N + 1] {
    const M: usize = 7;
    const NSTACK: usize = 50;
    // 1-based views.
    let a_of = |idx: &[usize; N + 1], i: usize| arr[idx[i] - 1];
    let mut indx = [0usize; N + 1];
    for (j, v) in indx.iter_mut().enumerate().skip(1) {
        *v = j;
    }
    let mut istack = [0usize; NSTACK + 1];
    let mut jstack = 0usize;
    let mut l = 1usize;
    let mut ir = N;
    loop {
        if (ir as isize) - (l as isize) < M as isize {
            for j in (l + 1)..=ir {
                let indxt = indx[j];
                let a = arr[indxt - 1];
                let mut i = j - 1;
                loop {
                    if i == 0 {
                        break;
                    }
                    if a_of(&indx, i) <= a {
                        break;
                    }
                    indx[i + 1] = indx[i];
                    i -= 1;
                }
                indx[i + 1] = indxt;
            }
            if jstack == 0 {
                return indx;
            }
            ir = istack[jstack];
            l = istack[jstack - 1];
            jstack -= 2;
        } else {
            let k = (l + ir) / 2;
            indx.swap(k, l + 1);
            if a_of(&indx, l + 1) > a_of(&indx, ir) {
                indx.swap(l + 1, ir);
            }
            if a_of(&indx, l) > a_of(&indx, ir) {
                indx.swap(l, ir);
            }
            if a_of(&indx, l + 1) > a_of(&indx, l) {
                indx.swap(l + 1, l);
            }
            let mut i = l + 1;
            let mut j = ir;
            let indxt = indx[l];
            let a = arr[indxt - 1];
            loop {
                loop {
                    i += 1;
                    if a_of(&indx, i) >= a {
                        break;
                    }
                }
                loop {
                    j -= 1;
                    if a_of(&indx, j) <= a {
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
            if (ir as isize) + 1 - (i as isize) >= (j as isize) - (l as isize) {
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
}

/// `mrbencode`: the XOR of the columns of `g2` (as rows here, `g2[i]` being column `i` of
/// the Fortran's `g2(N,K)`) selected by `me`.
fn mrbencode(me: &[u8; K], g2: &[[u8; N]; K]) -> [u8; N] {
    let mut cw = [0u8; N];
    for i in 0..K {
        if me[i] == 1 {
            for c in 0..N {
                cw[c] ^= g2[i][c];
            }
        }
    }
    cw
}

/// `nextpat`: the next error pattern of weight `iorder` in `mi`, and `iflag`, the 0-based
/// position of its lowest-index 1, or `None` once the last has been generated
/// (the Fortran's negative `iflag`).
fn nextpat(mi: &mut [u8; K], iorder: usize) -> Option<usize> {
    let mut ind: Option<usize> = None;
    for i in 0..K - 1 {
        if mi[i] == 0 && mi[i + 1] == 1 {
            ind = Some(i);
        }
    }
    let ind = ind?;
    let mut ms = [0u8; K];
    ms[..ind].copy_from_slice(&mi[..ind]);
    ms[ind] = 1;
    ms[ind + 1] = 0;
    // Fortran `ind + 1 .lt. k` with 1-based `ind`: 0-based `ind + 2 < K`.
    if ind + 2 < K {
        let sum: usize = ms.iter().map(|&b| usize::from(b)).sum();
        let nz = iorder - sum;
        for v in &mut ms[K - nz..] {
            *v = 1;
        }
    }
    *mi = ms;
    mi.iter().position(|&b| b == 1)
}

/// The `boxit` / `fetchit` pattern store (`/boxes/`), `ntau` bits of the parity of a pair
/// of basis bits as the key.
struct Boxes {
    /// `indexes(:, 1:2)`, 1-based slot `npindex`, the two basis bits (1-based).
    pairs: Vec<(usize, usize)>,
    /// `fp(ipat)`, -1 when empty.
    fp: Vec<i32>,
    /// `np(slot)`.
    np: Vec<i32>,
    lastpat: i64,
    inext: i32,
}

impl Boxes {
    fn new() -> Self {
        Self {
            pairs: vec![(0, 0); 4001],
            fp: vec![-1; 1 << NTAU],
            np: vec![-1; 4001],
            lastpat: -1,
            inext: 0,
        }
    }

    fn ipat(e2: &[u8]) -> usize {
        let mut ipat = 0usize;
        for (i, &b) in e2.iter().enumerate().take(NTAU) {
            if b == 1 {
                ipat += 1 << (NTAU - 1 - i);
            }
        }
        ipat
    }

    /// `boxit`: add slot `npindex` (pair `i1`, `i2`, 1-based) under the pattern `e2`.
    fn boxit(&mut self, e2: &[u8], npindex: usize, i1: usize, i2: usize) {
        self.pairs[npindex] = (i1, i2);
        let ipat = Self::ipat(e2);
        let ip = self.fp[ipat];
        if ip == -1 {
            self.fp[ipat] = npindex as i32;
        } else {
            let mut ip = ip as usize;
            while self.np[ip] != -1 {
                ip = self.np[ip] as usize;
            }
            self.np[ip] = npindex as i32;
        }
    }

    /// `fetchit`: the next pair stored under `e2`, `(-1, -1)`-style `None` when there is
    /// none. Keeps the Fortran's `save`d `lastpat` / `inext`.
    fn fetchit(&mut self, e2: &[u8]) -> Option<(usize, usize)> {
        let ipat = Self::ipat(e2);
        let index = self.fp[ipat];
        let out;
        if self.lastpat != ipat as i64 && index > 0 {
            out = Some(self.pairs[index as usize]);
            self.inext = self.np[index as usize];
        } else if self.lastpat == ipat as i64 && self.inext > 0 {
            out = Some(self.pairs[self.inext as usize]);
            self.inext = self.np[self.inext as usize];
        } else {
            out = None;
            self.inext = -1;
        }
        self.lastpat = ipat as i64;
        out
    }
}

/// `osdwspr(ss, apmask, ndeep, cw, nhardmin, dmin)`. `ss` are the soft symbols as
/// `wsprd.c:1424` hands them over, `symbols[i] - 128`, in deinterleaved order.
pub(crate) fn osdwspr(ss: &[f32; N], ndeep: i32) -> OsdOut {
    let gmat = generator();

    let mut rx = [0f32; N];
    for i in 0..N {
        rx[i] = ss[i] / 127.0;
    }
    let mut hdec = [0u8; N];
    for i in 0..N {
        if rx[i] >= 0.0 {
            hdec[i] = 1;
        }
    }
    let mut absrx = [0f32; N];
    for i in 0..N {
        absrx[i] = rx[i].abs();
    }
    let indx = indexx(&absrx);

    // Columns of the generator in order of decreasing reliability. `genmrb[row][col]`.
    let mut genmrb = [[0u8; N]; K];
    let mut indices = [0usize; N]; // 0-based source column of each ordered column
    for i in 0..N {
        let src = indx[N - i] - 1; // indx(N+1-i), 1-based i
        indices[i] = src;
        for row in 0..K {
            genmrb[row][i] = gmat[row][src];
        }
    }

    // Gaussian elimination so the first K ordered columns are the identity. The Fortran
    // looks for a pivot in columns `id .. K+20` and carries on without one.
    for id in 0..K {
        for icol in id..(K + 20).min(N) {
            if genmrb[id][icol] == 1 {
                if icol != id {
                    for row in 0..K {
                        genmrb[row].swap(id, icol);
                    }
                    indices.swap(id, icol);
                }
                for ii in 0..K {
                    if ii != id && genmrb[ii][id] == 1 {
                        let pivot = genmrb[id];
                        for c in 0..N {
                            genmrb[ii][c] ^= pivot[c];
                        }
                    }
                }
                break;
            }
        }
    }

    // `g2 = transpose(genmrb)`; kept as `g2[i]` = column `i` of the Fortran's g2(N, K)... that
    // is, row `i` of `genmrb`.
    let g2 = genmrb;

    // Reorder the received word.
    let mut hdec_o = [0u8; N];
    let mut absrx_o = [0f32; N];
    for i in 0..N {
        hdec_o[i] = hdec[indices[i]];
        absrx_o[i] = absrx[indices[i]];
    }
    let hdec = hdec_o;
    let absrx = absrx_o;

    let mut m0 = [0u8; K];
    m0.copy_from_slice(&hdec[..K]);
    let c0 = mrbencode(&m0, &g2);

    let mut nxor = [0u8; N];
    for i in 0..N {
        nxor[i] = c0[i] ^ hdec[i];
    }
    let mut nhardmin: i32 = nxor.iter().map(|&b| i32::from(b)).sum();
    let mut dmin = 0f32;
    for i in 0..N {
        dmin += f32::from(nxor[i]) * absrx[i];
    }
    let mut cw = c0;

    let mut ndeep = ndeep;
    if ndeep > 0 {
        if ndeep > 5 {
            ndeep = 5;
        }
        let (nord, npre1, npre2, ntheta): (usize, bool, bool, usize) = match ndeep {
            1 => (1, false, false, 16),
            2 => (1, true, false, 22),
            3 => (1, true, true, 22),
            4 => (2, true, true, 22),
            _ => (3, true, true, 22),
        };

        for iorder in 1..=nord {
            let mut misub = [0u8; K];
            for v in &mut misub[K - iorder..] {
                *v = 1;
            }
            let mut iflag = Some(K - iorder); // 0-based position of the lowest 1
            while let Some(flag) = iflag {
                let iend = if iorder == nord && !npre1 { flag } else { 0 };
                let mut d1 = 0f32;
                let mut e2sub = [0u8; N - K];
                // `do n1 = iflag, iend, -1`
                let mut n1 = flag as i64;
                while n1 >= iend as i64 {
                    let n1u = n1 as usize;
                    let mut mi = misub;
                    mi[n1u] = 1;
                    let mut me = [0u8; K];
                    for i in 0..K {
                        me[i] = m0[i] ^ mi[i];
                    }
                    let mut e2 = [0u8; N - K];
                    let nd1kpt: usize;
                    if n1u == flag {
                        let c = mrbencode(&me, &g2);
                        for i in 0..N - K {
                            e2sub[i] = c[K + i] ^ hdec[K + i];
                        }
                        e2 = e2sub;
                        nd1kpt = e2sub[..NT].iter().map(|&b| usize::from(b)).sum::<usize>() + 1;
                        d1 = 0.0;
                        for i in 0..K {
                            d1 += f32::from(me[i] ^ hdec[i]) * absrx[i];
                        }
                    } else {
                        for i in 0..N - K {
                            e2[i] = e2sub[i] ^ g2[n1u][K + i];
                        }
                        nd1kpt = e2[..NT].iter().map(|&b| usize::from(b)).sum::<usize>() + 2;
                    }
                    if nd1kpt <= ntheta {
                        let ce = mrbencode(&me, &g2);
                        let mut nx = [0u8; N];
                        for i in 0..N {
                            nx[i] = ce[i] ^ hdec[i];
                        }
                        let mut s = 0f32;
                        let dd = if n1u == flag {
                            for i in 0..N - K {
                                s += f32::from(e2sub[i]) * absrx[K + i];
                            }
                            d1 + s
                        } else {
                            for i in 0..N - K {
                                s += f32::from(e2[i]) * absrx[K + i];
                            }
                            d1 + f32::from(ce[n1u] ^ hdec[n1u]) * absrx[n1u] + s
                        };
                        if dd < dmin {
                            dmin = dd;
                            cw = ce;
                            nhardmin = nx.iter().map(|&b| i32::from(b)).sum();
                        }
                    }
                    n1 -= 1;
                }
                iflag = nextpat(&mut misub, iorder);
            }
        }

        if npre2 {
            let mut boxes = Boxes::new();
            let mut ntotal = 0usize;
            // `do i1 = K, 1, -1; do i2 = i1-1, 1, -1` (1-based).
            for i1 in (1..=K).rev() {
                for i2 in (1..i1).rev() {
                    ntotal += 1;
                    let mut mi = [0u8; NTAU];
                    for t in 0..NTAU {
                        mi[t] = g2[i1 - 1][K + t] ^ g2[i2 - 1][K + t];
                    }
                    boxes.boxit(&mi, ntotal, i1, i2);
                }
            }

            boxes.lastpat = -1;
            let mut misub = [0u8; K];
            for v in &mut misub[K - nord..] {
                *v = 1;
            }
            let mut iflag = Some(K - nord);
            while iflag.is_some() {
                let mut me = [0u8; K];
                for i in 0..K {
                    me[i] = m0[i] ^ misub[i];
                }
                let ce = mrbencode(&me, &g2);
                let mut e2sub = [0u8; N - K];
                for i in 0..N - K {
                    e2sub[i] = ce[K + i] ^ hdec[K + i];
                }
                for i2 in 0..=NTAU {
                    let mut r2pat = e2sub;
                    if i2 > 0 {
                        r2pat[i2 - 1] ^= 1;
                    }
                    // `778 continue`, `fetchit`, `goto 778`; the Fortran's `cycle` for a
                    // pattern of too low a weight leaves this `i2`.
                    'fetch: loop {
                        let Some((in1, in2)) = boxes.fetchit(&r2pat[..NTAU]) else {
                            break 'fetch;
                        };
                        let mut mi = misub;
                        mi[in1 - 1] = 1;
                        mi[in2 - 1] = 1;
                        let w: usize = mi.iter().map(|&b| usize::from(b)).sum();
                        if w < nord + usize::from(npre1) + usize::from(npre2) {
                            break 'fetch;
                        }
                        let mut me = [0u8; K];
                        for i in 0..K {
                            me[i] = m0[i] ^ mi[i];
                        }
                        let ce = mrbencode(&me, &g2);
                        let mut nx = [0u8; N];
                        for i in 0..N {
                            nx[i] = ce[i] ^ hdec[i];
                        }
                        let mut dd = 0f32;
                        for i in 0..N {
                            dd += f32::from(nx[i]) * absrx[i];
                        }
                        if dd < dmin {
                            dmin = dd;
                            cw = ce;
                            nhardmin = nx.iter().map(|&b| i32::from(b)).sum();
                        }
                    }
                }
                iflag = nextpat(&mut misub, nord);
            }
        }
    }

    // `cw(indices) = cw`: back to the order the symbols came in.
    let mut out = [0u8; N];
    for i in 0..N {
        out[indices[i]] = cw[i];
    }
    OsdOut {
        cw: out,
        nhardmin,
        dmin,
    }
}
