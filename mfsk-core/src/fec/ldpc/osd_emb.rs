//! `osd174_91` at `ndeep = 2` for a 32-bit target: [`osd_npre`](super::osd_npre)'s answer, bit for bit, with no heap and no big stack.
//!
//! Ported from the same `lib/ft8/osd174_91.f90` as [`osd_npre::osd174_91`](super::osd_npre::osd174_91), and **equal to it on every input**: the same winner
//! codeword, the same `hard_errors`, `None` exactly when it is `None` (tests: `tests/osd_embedded.rs`). What differs is how it is computed, for the CoreS3
//! (LX7, 32-bit, an f32 FPU, no `u64` shift and no popcount instruction):
//!
//! - **`u32` words** (six to a codeword) for the elimination and the search, so a row operation is a `u32` XOR and a variable shift is one instruction. `osd_npre`'s
//!   `u64` rows are two registers and a funnel of shifts each on this core. Only the 40-bit `npre1` window is a `u64`, built once a row.
//! - **The generator's columns come from a table built at compile time** (`COLS`, three words a column), not from a `[u128; 256]` assembled on the stack each
//!   call (`u128` is emulated on this target; `osd_npre` pays it once a call, 4 KB of stack and ~11 000 `u128` operations).
//! - **The working memory is the caller's** ([`Osd174Scratch`], about 4.5 KB, allocated once): no stack frame of its own beyond a few hundred bytes, no `Vec`.
//! - **A distance stops early.** `weighted`'s terms are non-negative and added in a fixed order, so once `base + partial >= dmin` the final `dd = base + sum` cannot be
//!   `< dmin` (f32 addition is monotone), and the candidate is dropped without summing the rest. The winner is unchanged; this is the cost cut that needs the proof.
//!
//! What is *not* changed, so that the answer is the same: the reliability sort is the same `sort_unstable_by` over a `[usize]` (a different element type may break
//! ties differently), the pivot search and the column swap are upstream's, the order of the candidates and the strict `dd < dmin` are `osd_npre`'s.
//!
//! Only `ndeep = 2` (`nord = 1`, `npre1 = 1`, no `npre2`): what `ft8b.f90` calls for FT8 and `ft4_decode.f90` for FT4. FST4's `osd240_101` and the hash-table pass of
//! `ndeep = 3` stay in `osd_npre`.

#[cfg(any(feature = "fft-rustfft", feature = "osd"))]
use super::bp::BpScratch;
use super::bp::check_crc14;
use super::params::{Ldpc174_91Params, LdpcParams};

const N: usize = Ldpc174_91Params::N; // 174
const K: usize = Ldpc174_91Params::K; // 91
/// `u32` words per packed codeword (174 bits).
const W: usize = N.div_ceil(32);
/// `osd174_91.f90:88`: `do icol=id,k+20` — "The 20 is ad hoc - beware".
const PIVOT_WINDOW_SLACK: usize = 20;
/// `nt`: the parity bits the `npre1` gate counts.
const NT: usize = 40;
/// `ntheta` of `ndeep = 2` for FT8 and FT4.
const NTHETA: u32 = 10;

type Row = [u32; W];

/// Column `j` of the generator as the rows it has set: bit `r` of the 91-bit value is row `r`. An information column `j < 91` is the unit vector `e_j`; a parity
/// column is `GEN_PARITY_PACKED[j - 91]` (`osd_npre` assembles the same `orig[j]` in a `[u128; 256]` on every call).
const COLS: [[u32; 3]; N] = cols();

const fn cols() -> [[u32; 3]; N] {
    let mut o = [[0u32; 3]; N];
    let mut j = 0;
    while j < K {
        o[j][j / 32] |= 1 << (j % 32);
        j += 1;
    }
    let info_mask = (1u128 << K) - 1;
    while j < N {
        let sel = Ldpc174_91Params::GEN_PARITY_PACKED[j - K] & info_mask;
        let mut r = 0;
        while r < K {
            if (sel >> r) & 1 != 0 {
                o[j][r / 32] |= 1 << (r % 32);
            }
            r += 1;
        }
        j += 1;
    }
    o
}

const fn range_mask(lo: usize, hi: usize) -> Row {
    let mut m = [0u32; W];
    let mut c = lo;
    while c < hi {
        m[c / 32] |= 1 << (c % 32);
        c += 1;
    }
    m
}

/// Every bit of a codeword, and the parity bits `k..n`.
const ALL: Row = range_mask(0, N);
const PARITY: Row = range_mask(K, N);

#[inline(always)]
fn bit(r: &Row, c: usize) -> bool {
    (r[c / 32] >> (c % 32)) & 1 != 0
}

#[inline(always)]
fn xor_into(a: &mut Row, b: &Row) {
    for w in 0..W {
        a[w] ^= b[w];
    }
}

/// `base + sum(absrx[c])` over the set bits of `x & mask`, summed in ascending position order as `osd_npre::weighted` does, or `None` as soon as
/// `base + partial >= dmin`: the terms are `>= 0` and f32 addition is monotone, so the final `base + sum` is then `>= dmin` too and `dd < dmin` is false.
/// `Some(sum)` is the whole sum; the caller adds `base` itself, as `osd_npre` does, so the value compared is the same f32.
#[inline(always)]
fn weighted_below(x: &Row, mask: &Row, absrx: &[f32; N], base: f32, dmin: f32) -> Option<f32> {
    let mut s = 0.0f32;
    for w in 0..W {
        let mut v = x[w] & mask[w];
        while v != 0 {
            let b = v.trailing_zeros() as usize;
            s += absrx[w * 32 + b];
            v &= v - 1;
        }
        if base + s >= dmin {
            return None;
        }
    }
    Some(s)
}

/// The same sum with no limit (`dmin` is not known yet).
#[inline(always)]
fn weighted(x: &Row, mask: &Row, absrx: &[f32; N]) -> f32 {
    let mut s = 0.0f32;
    for w in 0..W {
        let mut v = x[w] & mask[w];
        while v != 0 {
            let b = v.trailing_zeros() as usize;
            s += absrx[w * 32 + b];
            v &= v - 1;
        }
    }
    s
}

/// The `nt` parity bits of a row, as one word: bits `k..k + nt`.
#[inline(always)]
fn window(r: &Row) -> u64 {
    let (w, b) = (K / 32, K % 32);
    let mut v = (r[w] as u64) >> b;
    v |= (r[w + 1] as u64) << (32 - b);
    v |= (r[w + 2] as u64) << (64 - b);
    v & ((1u64 << NT) - 1)
}

/// Working memory of [`osd174_91_into`]: about 4.5 KB. Hold one per decoder and pass it to every call; nothing in it carries over.
pub struct Osd174Scratch {
    indices: [usize; N],
    g: [Row; K],
    absrx: [f32; N],
    gw: [u64; K],
    codeword: [u8; N],
}

impl Osd174Scratch {
    pub const fn new() -> Self {
        Self {
            indices: [0; N],
            g: [[0; W]; K],
            absrx: [0.0; N],
            gw: [0; K],
            codeword: [0; N],
        }
    }

    /// The winner's codeword in the original bit order, valid after a call that returned `Some`.
    pub fn codeword(&self) -> &[u8; N] {
        &self.codeword
    }
}

impl Default for Osd174Scratch {
    fn default() -> Self {
        Self::new()
    }
}

/// Everything the FT8 OSD fallback needs besides the LLRs: the OSD's own memory and the f32 BP scratch that makes its `zsave` inputs. One per
/// [`BpScratch`], made on the first OSD run of a candidate and kept, so the fallback allocates nothing after that.
#[cfg(any(feature = "fft-rustfft", feature = "osd"))]
pub struct OsdWork {
    pub(crate) osd: Osd174Scratch,
    pub(crate) zsum: BpScratch<Ldpc174_91Params, f32>,
    /// The score of the attempt that was queued for the OSD instead of run (`OsdMode::Defer`), taken by the worker after the attempt.
    #[cfg(feature = "fixed-point")]
    pub(crate) deferred: Option<f32>,
}

#[cfg(any(feature = "fft-rustfft", feature = "osd"))]
impl OsdWork {
    /// Everything the fallback will ever allocate, now: the buffers the BP scratch grows on its first use (`ensure_tanhtoc`, `ensure_zsum`) are grown here, so that where
    /// they live is decided by the caller of [`crate::ft8::decode_block::CellsWorker::reserve_osd_work`] and not by the first candidate that needs the OSD.
    pub(crate) fn new() -> Self {
        let mut zsum = BpScratch::new();
        zsum.ensure_tanhtoc();
        zsum.ensure_zsum();
        Self {
            osd: Osd174Scratch::new(),
            zsum,
            #[cfg(feature = "fixed-point")]
            deferred: None,
        }
    }
}

/// What a successful [`osd174_91_into`] found; the codeword is in [`Osd174Scratch::codeword`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OsdHit {
    /// The first 77 information bits.
    pub message77: [u8; 77],
    /// Bits of the winner that differ from the hard decision of `llr`.
    pub hard_errors: u32,
}

/// `osd174_91` at `ndeep = 2` (see the module doc): [`NDEEP2_174_91`](super::osd_npre::NDEEP2_174_91) with the CRC-14 on the winner. `ap_mask[i]` locks bit `i`.
pub fn osd174_91_into(
    s: &mut Osd174Scratch,
    llr: &[f32; N],
    ap_mask: Option<&[bool; N]>,
) -> Option<OsdHit> {
    let Osd174Scratch {
        indices,
        g,
        absrx,
        gw,
        codeword,
    } = s;

    // ── Reliability order (`indexx`, reversed) ──────────────────────────
    for (i, x) in indices.iter_mut().enumerate() {
        *x = i;
    }
    indices.sort_unstable_by(|&a, &b| {
        llr[b]
            .abs()
            .partial_cmp(&llr[a].abs())
            .unwrap_or(core::cmp::Ordering::Equal)
    });

    // ── Generator, held by row, columns in reliability order ────────────
    for r in g.iter_mut() {
        *r = [0; W];
    }
    for (c, &j) in indices.iter().enumerate() {
        for (q, &word) in COLS[j].iter().enumerate() {
            let mut v = word;
            while v != 0 {
                let r = q * 32 + v.trailing_zeros() as usize;
                g[r][c / 32] |= 1 << (c % 32);
                v &= v - 1;
            }
        }
    }

    // ── Gaussian elimination, upstream's column-swap search ─────────────
    let upper = (K + PIVOT_WINDOW_SLACK).min(N);
    for id in 0..K {
        let Some(icol) = (id..upper).find(|&c| bit(&g[id], c)) else {
            continue;
        };
        if icol != id {
            for r in g.iter_mut() {
                let d = ((r[id / 32] >> (id % 32)) ^ (r[icol / 32] >> (icol % 32))) & 1;
                r[id / 32] ^= d << (id % 32);
                r[icol / 32] ^= d << (icol % 32);
            }
            indices.swap(id, icol);
        }
        let pivot = g[id];
        // All six words, written out: a loop of `w0..W` (two to six trips, 8 281 times a call) cost 17 % of the call on the CoreS3 (`ESP32S3_TWO_CORE_BEHAVIOUR.md`); the words below `w0` of
        // `pivot` are zero when every row so far found its pivot, so xoring them is the same.
        for (ii, r) in g.iter_mut().enumerate() {
            let m = (((r[id / 32] >> (id % 32)) & 1) & (ii != id) as u32).wrapping_neg();
            r[0] ^= pivot[0] & m;
            r[1] ^= pivot[1] & m;
            r[2] ^= pivot[2] & m;
            r[3] ^= pivot[3] & m;
            r[4] ^= pivot[4] & m;
            r[5] ^= pivot[5] & m;
        }
    }

    // ── Received word in the same order ─────────────────────────────────
    let mut hdec: Row = [0; W];
    let mut apm = [false; K];
    for c in 0..N {
        let x = llr[indices[c]];
        if x >= 0.0 {
            hdec[c / 32] |= 1 << (c % 32);
        }
        absrx[c] = x.abs();
    }
    if let Some(m) = ap_mask {
        for c in 0..K {
            apm[c] = m[indices[c]];
        }
    }
    for (i, gi) in g.iter().enumerate() {
        gw[i] = window(gi);
    }

    // Order 0: `m0 = hdec(1:k)`, `c0 = m0 · G`.
    let mut c0: Row = [0; W];
    for (i, gi) in g.iter().enumerate() {
        if bit(&hdec, i) {
            xor_into(&mut c0, gi);
        }
    }
    let mismatch = |c: &Row| {
        let mut x = *c;
        xor_into(&mut x, &hdec);
        x
    };
    let mut dmin = weighted(&mismatch(&c0), &ALL, absrx);
    let mut cw = c0;

    // ── nord = 1, npre1 = 1 (`osd174_91.f90:177-228`) ───────────────────
    for iflag in (0..K).rev() {
        if apm[iflag] {
            continue;
        }
        let mut ce1 = c0;
        xor_into(&mut ce1, &g[iflag]);
        let e2sub = mismatch(&ce1);
        let d1 = absrx[iflag];
        let e2sub_w = window(&e2sub);
        if e2sub_w.count_ones() < NTHETA
            && let Some(sum) = weighted_below(&e2sub, &PARITY, absrx, d1, dmin)
        {
            let dd = d1 + sum;
            if dd < dmin {
                dmin = dd;
                cw = ce1;
            }
        }
        for n1 in (0..iflag).rev() {
            if apm[n1] {
                continue;
            }
            if (e2sub_w ^ gw[n1]).count_ones() + 2 > NTHETA {
                continue;
            }
            let mut e2 = e2sub;
            xor_into(&mut e2, &g[n1]);
            let mut ce = ce1;
            xor_into(&mut ce, &g[n1]);
            let x = if bit(&ce, n1) != bit(&hdec, n1) {
                absrx[n1]
            } else {
                0.0
            };
            let base = d1 + x;
            if let Some(sum) = weighted_below(&e2, &PARITY, absrx, base, dmin) {
                let dd = base + sum;
                if dd < dmin {
                    dmin = dd;
                    cw = ce;
                }
            }
        }
    }

    // ── Back to bit order; the CRC on the winner ────────────────────────
    for c in 0..N {
        codeword[indices[c]] = bit(&cw, c) as u8;
    }
    if !check_crc14(&codeword[..K]) {
        return None;
    }
    let mut hard_errors = 0u32;
    for i in 0..N {
        hard_errors += ((codeword[i] == 1) != (llr[i] >= 0.0)) as u32;
    }
    let mut message77 = [0u8; 77];
    message77.copy_from_slice(&codeword[..77]);
    Some(OsdHit {
        message77,
        hard_errors,
    })
}
