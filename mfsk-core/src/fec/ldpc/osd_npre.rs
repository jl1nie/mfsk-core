//! The ordered-statistics decoder WSJT-X runs on its LDPC codes, ported line
//! for line from `lib/ft8/osd174_91.f90` and `lib/fst4/osd240_101.f90`
//! (v3.2.0-rc1) — the two files are the same algorithm on a different
//! generator — for the `nord = 1` depths every mode here calls: `ndeep = 2`
//! (`npre1`) and `ndeep = 3` (`npre1` + `npre2`).
//!
//! **Faithful, including the parts that look accidental** (issue #417):
//! - The basis is chosen as upstream chooses it: row `id` takes the first
//!   column in `id..k+20` (reliability order) whose bit is set and swaps it
//!   into position `id` (`osd174_91.f90:86-107`). That is not the textbook
//!   "first `k` independent columns", and a row with no pivot in the window
//!   is left un-eliminated, as upstream leaves it ("beware").
//! - Every distance is upstream's own expression, summed in upstream's order,
//!   so `dd < dmin` ties break as they do there.
//! - `npre2`'s hash lookup keeps `fetchit91`'s state across calls, and a pair
//!   rejected by the weight or a-priori test ends that key's chain (`cycle`
//!   leaves the `goto 778` loop), as upstream does.
//!
//! Checked against the Fortran itself (built by `scripts/osd-upstream/run.sh`
//! with a release build's flags): the same result on every input tried —
//! 16 230 for `osd174_91` (the LLRs that reach OSD while decoding five FT8
//! recordings and the FT4 golden at `Depth::Deep`, plus synthetic codewords
//! and noise) and 2 630 for `osd240_101` (the FST4-60 golden and fifteen
//! FST4-60 sweep recordings near threshold, plus synthetic ones), with and
//! without an a-priori mask, at `ndeep` 2 and 3. A subset of those inputs and
//! upstream's answers is pinned in `tests/osd_upstream_fixture.rs`.
//!
//! The arithmetic is packed: a codeword is up to four `u64`s, the generator
//! comes from a table built at compile time ([`LdpcParams::GEN_PARITY_PACKED`]),
//! the elimination is branch-free, the `npre1` gate is one `count_ones` on a
//! word holding the `nt` parity bits, and a distance walks only the bits that
//! disagree. Per call, on the LLRs FT8 hands OSD (`ndeep = 2`, Ryzen 9 9900X,
//! one thread): upstream 198 µs, this 24 µs; the two ports it replaced, 57 µs
//! (FT8/FT4) and about 200 µs (FST4). `qso3_busy` at `Depth::Deep` decodes in
//! 504 ms instead of 570, the FST4-60 golden in 597 ms instead of 951.
//! The elimination and the search are `u64`-wide, so a 32-bit target pays
//! `u128` emulation only once a call, assembling the generator's columns.
//!
//! What differs from the Fortran, deliberately:
//! - The reliability sort is Rust's, not `indexx`; the two can order equal
//!   magnitudes differently. Equal magnitudes come mostly from a-priori bits,
//!   all clamped to the same `apmag`; every recorded input with a mask agreed.
//! - Upstream computes `nhardmin` and returns a CRC-failing winner with it
//!   negated; here a winner `verify` rejects is `None`, which is how every
//!   caller (`decode174_91.f90`, `decode240_101.f90`) reads a negative value.

use alloc::vec;
use alloc::vec::Vec;

use super::bp::check_crc14;
use super::osd::{OsdResult, PartialCrc};
use super::params::{Ldpc174_91Params, LdpcParams};

/// Upper bound on `P::N` (240 for FST4, 174 for FT8/FT4).
const MAX_N: usize = 256;
/// `u64` words per packed codeword.
const W: usize = MAX_N / 64;
/// Upper bound on the number of free information bits: `Keff` ≤ `P::K` ≤ 101.
/// (The generator's columns are `u128`s, so it could not exceed 128.)
const MAX_K: usize = 101;
/// `osd174_91.f90:88` `do icol=id,k+20` — "The 20 is ad hoc - beware".
const PIVOT_WINDOW_SLACK: usize = 20;
/// `nt`: how many leading parity bits the `npre1` gate counts. 40 for every
/// `ndeep` below 6 in both files; it must fit one `u64`.
const NT: usize = 40;

type Row = [u64; W];

#[inline]
fn bit(r: &Row, c: usize) -> bool {
    (r[c / 64] >> (c % 64)) & 1 != 0
}

#[inline]
fn xor_into(a: &mut Row, b: &Row) {
    for w in 0..W {
        a[w] ^= b[w];
    }
}

/// A mask of bit positions `lo..hi`.
fn range_mask(lo: usize, hi: usize) -> Row {
    let mut m = [0u64; W];
    for c in lo..hi {
        m[c / 64] |= 1 << (c % 64);
    }
    m
}

/// `sum(x * absrx)` over the set bits of `x` restricted to `mask`, added
/// in ascending position order — the order a Fortran `sum` of the same
/// elementwise product adds them in, zeros contributing nothing exactly.
#[inline]
fn weighted(x: &Row, mask: &Row, absrx: &[f32]) -> f32 {
    let mut s = 0.0f32;
    for w in 0..W {
        let mut v = x[w] & mask[w];
        while v != 0 {
            let b = v.trailing_zeros() as usize;
            s += absrx[w * 64 + b];
            v &= v - 1;
        }
    }
    s
}

/// The parameters `osd174_91.f90` / `osd240_101.f90` set per `ndeep`, for
/// the `nord = 1` depths.
#[derive(Clone, Copy, Debug)]
pub struct NpreDepth {
    /// `ntheta`: the `npre1` gate. 10 for FT8/FT4 at `ndeep = 2`, 12 otherwise.
    pub ntheta: u32,
    /// `npre2 = 1` (`ndeep = 3`) runs the hash-table pass.
    pub npre2: bool,
    /// `ntau`: the hash key's width in parity bits (14 at `ndeep = 3`).
    pub ntau: usize,
}

/// `osd174_91` / `osd240_101` at `nord = 1`.
///
/// - `partial_crc`: `None` searches all `P::K` information bits (`k = 91` for
///   FT8 and FT4, `decode174_91.f90`'s `Keff`); `Some` searches the first
///   `keff` and cascades the rest of the CRC with the code, as
///   `osd240_101.f90` does for FST4 (`Keff = 91`).
/// - `ap_mask[i]` (original bit order, length `P::N`): `true` locks bit `i`;
///   a test pattern that would flip it is skipped (`apmaskr`).
/// - `verify` is run once, on the winner (upstream's closing CRC check).
pub fn osd_npre<P: LdpcParams>(
    llr: &[f32],
    depth: NpreDepth,
    partial_crc: Option<PartialCrc>,
    ap_mask: Option<&[bool]>,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<OsdResult> {
    let n = P::N;
    let kinfo = P::K;
    let k = partial_crc.map_or(kinfo, |pc| pc.keff);
    debug_assert_eq!(llr.len(), n);
    debug_assert!(n <= MAX_N && k <= MAX_K && k <= kinfo);
    debug_assert!(ap_mask.is_none_or(|m| m.len() == n));
    debug_assert!(!depth.npre2 || depth.ntau <= 16);

    // ── Reliability order (`indexx`, reversed) ──────────────────────────
    let mut indices = [0usize; MAX_N];
    let indices = &mut indices[..n];
    for (i, x) in indices.iter_mut().enumerate() {
        *x = i;
    }
    indices.sort_unstable_by(|&a, &b| {
        llr[b]
            .abs()
            .partial_cmp(&llr[a].abs())
            .unwrap_or(core::cmp::Ordering::Equal)
    });

    // ── Generator, held by column ──────────────────────────────────────
    // Bit `r` of a column is row `r`. Row `r` encodes the unit information
    // word `e_r`; with a partial CRC, a message row also carries its CRC's
    // cascaded tail (`osd240_101.f90:47-60`). An information column `q` is
    // then the set of rows with `u_r(q) = 1`, and parity is linear, so a parity
    // column is the XOR of the information columns its generator row selects.
    // Built in a scope of its own, so its 4 KB of stack can be reused below.
    let mut g = [[0u64; W]; MAX_K];
    let g = &mut g[..k];
    {
        let mut orig = [0u128; MAX_N];
        let orig = &mut orig[..n];
        for (q, c) in orig[..k].iter_mut().enumerate() {
            *c = 1 << q;
        }
        if let Some(pc) = partial_crc {
            for r in 0..77.min(k) {
                let mut m = [0u8; 77];
                m[r] = 1;
                let with = (pc.with_crc)(&m);
                for q in k..kinfo {
                    if with[q] != 0 {
                        orig[q] |= 1 << r;
                    }
                }
            }
        }
        let info_mask = (1u128 << k) - 1;
        for j in kinfo..n {
            let sel = P::GEN_PARITY_PACKED[j - kinfo];
            let mut c = sel & info_mask;
            for q in k..kinfo {
                if sel >> q & 1 != 0 {
                    c ^= orig[q];
                }
            }
            orig[j] = c;
        }
        // Rows, packed by position in reliability order: `g[r]` has bit `c` set
        // when column `indices[c]` has bit `r`.
        for (c, &j) in indices.iter().enumerate() {
            let mut v = orig[j];
            while v != 0 {
                let r = v.trailing_zeros() as usize;
                g[r][c / 64] |= 1 << (c % 64);
                v &= v - 1;
            }
        }
    }

    // ── Gaussian elimination, upstream's column-swap search ────────────
    // Row `id` takes the first column in `id..k+20` with its bit set, swapped
    // into position `id`; every other row with a 1 there gets row `id` added.
    // Row-wise in `u64` words, branch-free: on a host, 24 µs a call against
    // 33 for a column-wise `u128` form of the same elimination, and native
    // on 32-bit targets, where `u128` is not.
    let upper = (k + PIVOT_WINDOW_SLACK).min(n);
    let nw = n.div_ceil(64);
    // While every row so far found its pivot, columns `0..id` are unit
    // vectors and row `id` is zero there, so adding it can skip those words.
    let mut all_pivoted = true;
    for id in 0..k {
        let Some(icol) = (id..upper).find(|&c| bit(&g[id], c)) else {
            all_pivoted = false;
            continue; // no pivot in the window: left as is, as upstream does
        };
        if icol != id {
            for r in g.iter_mut() {
                let d = ((r[id / 64] >> (id % 64)) ^ (r[icol / 64] >> (icol % 64))) & 1;
                r[id / 64] ^= d << (id % 64);
                r[icol / 64] ^= d << (icol % 64);
            }
            indices.swap(id, icol);
        }
        let pivot = g[id];
        let w0 = if all_pivoted { id / 64 } else { 0 };
        // Branch-free: whether a row has bit `id` is a coin toss, and a
        // branch on it cost more than the XOR it skips.
        for (ii, r) in g.iter_mut().enumerate() {
            let m = (((r[id / 64] >> (id % 64)) & 1) & (ii != id) as u64).wrapping_neg();
            for w in w0..nw {
                r[w] ^= pivot[w] & m;
            }
        }
    }

    // ── Received word in the same order ────────────────────────────────
    let mut hdec: Row = [0; W];
    let mut absrx = [0.0f32; MAX_N];
    let absrx = &mut absrx[..n];
    let mut apm = [false; MAX_K];
    for c in 0..n {
        let x = llr[indices[c]];
        if x >= 0.0 {
            hdec[c / 64] |= 1 << (c % 64);
        }
        absrx[c] = x.abs();
    }
    if let Some(m) = ap_mask {
        for c in 0..k {
            apm[c] = m[indices[c]];
        }
    }
    let all = range_mask(0, n);
    let parity = range_mask(k, n);
    let nt = NT.min(n - k);
    // The gate's `nt` parity bits of a row, as one word.
    let win = |r: &Row| {
        let (w, b) = (k / 64, k % 64);
        let lo = r[w] >> b;
        let hi = if b == 0 || w + 1 >= W {
            0
        } else {
            r[w + 1] << (64 - b)
        };
        (lo | hi) & ((1u64 << nt) - 1)
    };
    let mut gw = [0u64; MAX_K];
    for (i, gi) in g.iter().enumerate() {
        gw[i] = win(gi);
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
    let mut dmin = weighted(&mismatch(&c0), &all, absrx);
    let mut cw = c0;

    // ── nord = 1, npre1 = 1 (`osd174_91.f90:177-228`) ──────────────────
    for iflag in (0..k).rev() {
        // Every pattern below contains `iflag`.
        if apm[iflag] {
            continue;
        }
        let mut ce1 = c0;
        xor_into(&mut ce1, &g[iflag]);
        let e2sub = mismatch(&ce1);
        // `d1 = sum(ieor(me(1:k),hdec(1:k))*absrx(1:k))`: `m0 = hdec(1:k)`, so
        // only `iflag` differs.
        let d1 = absrx[iflag];
        // `nd1kpt = sum(e2sub(1:nt)) + 1; if (nd1kpt .le. ntheta)`
        let e2sub_w = win(&e2sub);
        if e2sub_w.count_ones() < depth.ntheta {
            let dd = d1 + weighted(&e2sub, &parity, absrx);
            if dd < dmin {
                dmin = dd;
                cw = ce1;
            }
        }
        for n1 in (0..iflag).rev() {
            if apm[n1] {
                continue;
            }
            if (e2sub_w ^ gw[n1]).count_ones() + 2 > depth.ntheta {
                continue;
            }
            let mut e2 = e2sub;
            xor_into(&mut e2, &g[n1]);
            let mut ce = ce1;
            xor_into(&mut ce, &g[n1]);
            // `dd = d1 + ieor(ce(n1),hdec(n1))*absrx(n1) + sum(e2*absrx(k+1:N))`
            let x = if bit(&ce, n1) != bit(&hdec, n1) {
                absrx[n1]
            } else {
                0.0
            };
            let dd = d1 + x + weighted(&e2, &parity, absrx);
            if dd < dmin {
                dmin = dd;
                cw = ce;
            }
        }
    }

    // ── npre2 = 1 (`osd174_91.f90:230-282`, `boxit91` / `fetchit91`) ───
    if depth.npre2 {
        let ntau = depth.ntau;
        let key_of = |r: &Row| {
            let mut p = 0usize;
            for t in 0..ntau {
                if bit(r, k + t) {
                    p |= 1 << (ntau - 1 - t);
                }
            }
            p
        };
        let mut row_key = [0usize; MAX_K];
        for (i, gi) in g.iter().enumerate() {
            row_key[i] = key_of(gi);
        }
        // `boxit91`: entries in insertion order (`i1 = k..1`, `i2 = i1-1..1`),
        // chained per key, appended at the tail.
        let npairs = k * (k - 1) / 2;
        let mut pairs: Vec<(u8, u8)> = Vec::with_capacity(npairs);
        let mut next: Vec<u32> = Vec::with_capacity(npairs);
        let mut head = vec![u32::MAX; 1 << ntau];
        let mut tail = vec![u32::MAX; 1 << ntau];
        for i1 in (0..k).rev() {
            for i2 in (0..i1).rev() {
                let key = row_key[i1] ^ row_key[i2];
                let idx = pairs.len() as u32;
                pairs.push((i1 as u8, i2 as u8));
                next.push(u32::MAX);
                if head[key] == u32::MAX {
                    head[key] = idx;
                } else {
                    next[tail[key] as usize] = idx;
                }
                tail[key] = idx;
            }
        }

        // `fetchit91`'s saved state: the last key looked up and where its
        // chain continues.
        let mut lastpat = usize::MAX;
        let mut inext = u32::MAX;
        let mut fetch = |ipat: usize| -> Option<(usize, usize)> {
            let index = head[ipat];
            let got = if lastpat != ipat && index != u32::MAX {
                inext = next[index as usize];
                Some(pairs[index as usize])
            } else if lastpat == ipat && inext != u32::MAX {
                let at = inext as usize;
                inext = next[at];
                Some(pairs[at])
            } else {
                inext = u32::MAX;
                None
            };
            lastpat = ipat;
            got.map(|(a, b)| (a as usize, b as usize))
        };

        for iflag in (0..k).rev() {
            let mut ce1 = c0;
            xor_into(&mut ce1, &g[iflag]);
            let e2sub = mismatch(&ce1);
            let base = key_of(&e2sub);
            for i2 in 0..=ntau {
                let ipat = if i2 == 0 {
                    base
                } else {
                    base ^ (1 << (ntau - i2))
                };
                while let Some((in1, in2)) = fetch(ipat) {
                    // `if(sum(mi).lt.nord+npre1+npre2 .or. any(apmaskr.and.mi)) cycle`
                    // — `cycle` leaves this key's chain.
                    if in1 == iflag || in2 == iflag || apm[iflag] || apm[in1] || apm[in2] {
                        break;
                    }
                    let mut ce = ce1;
                    xor_into(&mut ce, &g[in1]);
                    xor_into(&mut ce, &g[in2]);
                    let dd = weighted(&mismatch(&ce), &all, absrx);
                    if dd < dmin {
                        dmin = dd;
                        cw = ce;
                    }
                }
            }
        }
    }

    // ── Back to bit order; the CRC on the winner ───────────────────────
    let mut codeword = vec![0u8; n];
    for c in 0..n {
        codeword[indices[c]] = bit(&cw, c) as u8;
    }
    let info = codeword[..kinfo].to_vec();
    if let Some(f) = verify
        && !f(&info)
    {
        return None;
    }
    let hard_errors = (0..n)
        .filter(|&i| (codeword[i] == 1) != (llr[i] >= 0.0))
        .count() as u32;
    let mut message77 = [0u8; 77];
    message77.copy_from_slice(&info[..77]);
    Some(OsdResult {
        message77,
        info,
        codeword,
        hard_errors,
    })
}

/// `osd174_91.f90`'s `ndeep = 2`: `nord = 1`, `npre1 = 1`, `ntheta = 10`. What
/// FT8 (`ft8b.f90`, `norder = 2`) and FT4 (`ft4_decode.f90`) call.
pub const NDEEP2_174_91: NpreDepth = NpreDepth {
    ntheta: 10,
    npre2: false,
    ntau: 0,
};

/// `osd174_91.f90`'s `ndeep = 3`: adds `npre2` with `ntau = 14`, and loosens
/// `ntheta` to 12.
pub const NDEEP3_174_91: NpreDepth = NpreDepth {
    ntheta: 12,
    npre2: true,
    ntau: 14,
};

/// `osd174_91` on the (174,91) code with `k = 91`, as `decode174_91.f90`
/// calls it for FT8 and FT4: the CRC-14 checked on the winner.
pub fn osd174_91(
    llr: &[f32; Ldpc174_91Params::N],
    depth: NpreDepth,
    ap_mask: Option<&[bool; Ldpc174_91Params::N]>,
) -> Option<OsdResult> {
    osd_npre::<Ldpc174_91Params>(llr, depth, None, ap_mask.map(|m| &m[..]), Some(check_crc14))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::ldpc::osd::ldpc_encode;
    use crate::fec::ldpc::{LDPC_K, LDPC_N};

    /// Smoke: `ndeep = 2` on an all-zero LLR returns `None` or a codeword
    /// without panicking.
    #[test]
    fn npre1_zero_llr_no_panic() {
        let llr = [0.0f32; LDPC_N];
        let _ = osd174_91(&llr, NDEEP2_174_91, None);
    }

    /// Smoke: `ndeep = 3` on an all-zero LLR — also builds the `npre2`
    /// hash table.
    #[test]
    fn npre1_npre2_zero_llr_no_panic() {
        let llr = [0.0f32; LDPC_N];
        let _ = osd174_91(&llr, NDEEP3_174_91, None);
    }

    /// Clean-LLR round-trip via the ndeep=3 entry. Order-0 (= encode
    /// of MRB hard decisions) is exactly the input codeword, so this
    /// also serves as a sanity check that the npre1 → npre2 pass
    /// sequence doesn't accidentally overwrite the order-0 best with
    /// a worse-distance candidate.
    #[test]
    fn npre1_npre2_decodes_clean_codeword() {
        use crate::fec::ldpc::bp::crc14;
        let mut info_with_crc = [0u8; LDPC_K];
        for i in 0..77 {
            info_with_crc[i] = ((i * 13 + 7) & 1) as u8;
        }
        let mut bytes = [0u8; 12];
        for (i, &bit) in info_with_crc[..77].iter().enumerate() {
            bytes[i / 8] |= (bit & 1) << (7 - (i % 8));
        }
        let crc = crc14(&bytes);
        for j in 0..14 {
            info_with_crc[77 + j] = ((crc >> (13 - j)) & 1) as u8;
        }
        let cw = ldpc_encode(&info_with_crc);
        let mut llr = [0.0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if cw[i] == 1 { 4.0 } else { -4.0 };
        }
        let osd = osd174_91(&llr, NDEEP3_174_91, None).expect("clean LLR must decode");
        assert_eq!(&osd.info[..91], &info_with_crc[..]);
        assert_eq!(osd.hard_errors, 0);
    }

    /// Round-trip: a well-conditioned LLR derived from a known
    /// CRC-valid info word should decode back to the same info at
    /// `ndeep = 2`. The order-0 candidate (encode of hard
    /// decisions) is exactly the input codeword on this clean LLR, so
    /// the algorithm only needs to validate CRC and return — exercises
    /// the permutation and elimination.
    #[test]
    fn npre1_decodes_clean_codeword() {
        use crate::fec::ldpc::bp::crc14;
        // Build a CRC-valid 91-bit info: a fixed 77-bit payload + its
        // CRC-14 in bits 77..91. `check_crc14` packs the 77-bit field
        // into 12 bytes (big-endian, MSB-first) — replicate the same
        // packing here to compute the expected CRC.
        let mut info_with_crc = [0u8; LDPC_K];
        for i in 0..77 {
            info_with_crc[i] = ((i * 13 + 7) & 1) as u8;
        }
        let mut bytes = [0u8; 12];
        for (i, &bit) in info_with_crc[..77].iter().enumerate() {
            bytes[i / 8] |= (bit & 1) << (7 - (i % 8));
        }
        let crc = crc14(&bytes);
        for j in 0..14 {
            info_with_crc[77 + j] = ((crc >> (13 - j)) & 1) as u8;
        }
        // Sanity: `check_crc14` accepts our hand-rolled info.
        assert!(
            check_crc14(&info_with_crc),
            "test CRC packing must round-trip"
        );
        let cw = ldpc_encode(&info_with_crc);
        // Make a strong-confidence LLR: positive for cw=1 bits, negative
        // for cw=0 — magnitude well above any noise floor.
        let mut llr = [0.0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if cw[i] == 1 { 4.0 } else { -4.0 };
        }
        let osd = osd174_91(&llr, NDEEP2_174_91, None).expect("clean LLR must decode");
        assert_eq!(
            &osd.info[..91],
            &info_with_crc[..],
            "decoded info bits must match the encoded input"
        );
        assert_eq!(osd.hard_errors, 0, "clean LLR has zero hard errors");
    }
}
