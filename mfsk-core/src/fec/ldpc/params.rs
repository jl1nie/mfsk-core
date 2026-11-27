//! Compile-time parameters for the WSJT-family LDPC codes.
//!
//! Both [`crate::fec::Ldpc174_91`] (FT8 / FT4) and
//! [`crate::fec::Ldpc240_101`] (FST4) share an identical
//! belief-propagation + ordered-statistics decoding algorithm — only
//! the parity-check matrix shape and CRC width differ. This module
//! abstracts those shape-only differences behind a sealed
//! [`LdpcParams`] trait so both codes target a single generic
//! implementation living in [`super::bp`] and [`super::osd`].
//!
//! ## Closed-set property
//!
//! The trait is **sealed** by design: in practice the WSJT LDPC
//! generator matrices are hand-crafted for very short blocklengths,
//! and adding a third matrix is a research project rather than a
//! refactor. Sealing the trait keeps the closed-set property
//! visible at compile time — external crates that try to add a new
//! params type fail to link, signalling the impossibility correctly.
//!
//! Adding a fourth WSJT-family LDPC code (e.g. a new FST4 sub-mode
//! with a different generator matrix) means: define a new
//! `XxxParams` ZST, implement `sealed::Sealed` and [`LdpcParams`]
//! for it inside this crate, supply the table data, and add a
//! `pub type Xxx = LdpcCodec<XxxParams>` alias in [`super`].

use super::tables;

/// Sealed implementation of [`LdpcParams`].
///
/// External crates cannot satisfy this bound, so they cannot
/// implement [`LdpcParams`] either. This is the compile-time
/// expression of "the WSJT LDPC generator matrices are a closed
/// set".
mod sealed {
    pub trait Sealed {}
}

/// Compile-time parameters describing one specific WSJT-family LDPC
/// code. Implementors are zero-sized types ([`Ldpc174_91Params`],
/// [`Ldpc240_101Params`]) that ferry constants and table accessors
/// to the generic BP / OSD / encode bodies.
///
/// All accessors are written so the optimiser inlines them down to
/// a direct array index — one extra layer of indirection that
/// disappears at codegen.
pub trait LdpcParams: sealed::Sealed + Copy + Default + 'static {
    /// Codeword length in bits.
    const N: usize;
    /// Information length in bits (systematic prefix).
    const K: usize;
    /// Number of parity checks (= `N - K`; every code here is full rank).
    const M: usize = Self::N - Self::K;
    /// Maximum row weight across the parity-check matrix. Column
    /// weight (`MN`) is uniform at 3 for both codes; row weight
    /// varies (Ldpc174_91 has 6 or 7 entries per row, Ldpc240_101
    /// has 5 or 6).
    const MAX_ROW: usize;

    /// Per-bit check-node indices: `mn(bit)` gives the three checks
    /// the `bit`-th codeword bit participates in. Indices `0 .. M`.
    fn mn(bit: usize) -> [u8; 3];

    /// Per-check bit-node index: `nm(check, slot)` gives the
    /// `slot`-th bit participating in the `check`-th parity
    /// equation. Reading past `nrw(check)` yields a sentinel value
    /// (zero or 255 depending on the underlying table) that the BP
    /// loop does not consult — it truncates at `nrw(check)`.
    fn nm(check: usize, slot: usize) -> u8;

    /// Number of valid bit-node entries in row `check` (i.e. the
    /// row weight). Always ≤ [`Self::MAX_ROW`].
    fn nrw(check: usize) -> u8;

    /// Generator parity sub-matrix entry at `(row, col)`.
    /// `row ∈ 0..M`, `col ∈ 0..K`. Used by the systematic encoder
    /// and OSD's permuted-generator construction.
    fn gen_parity(row: usize, col: usize) -> u8;

    /// The generator's parity rows packed: bit `q` of entry `row` is
    /// `gen_parity(row, q)`. Built at compile time from the same table;
    /// what the ordered-statistics decoder (`osd_npre`) assembles its
    /// generator from, a column at a time.
    const GEN_PARITY_PACKED: &'static [u128];

    /// The code's edges as flat tables, made at compile time from `mn` / `nm` / `nrw`: what the NMS decoder reads on every pass in place of searching them
    /// ([`NmsEdges`]). No heap, no state in the scratch.
    #[doc(hidden)]
    const NMS_EDGES: &'static NmsEdges;
}

/// The edges of a code as flat tables (see [`LdpcParams::NMS_EDGES`]), every index range-checked when they are made (a compile-time assertion) so the decoder can read
/// with them unchecked. `nm_flat` / `c2v` are `M * MAX_ROW` long (check `j`'s `i`-th edge at `j * MAX_ROW + i`), `nrw` `M`, `v2c_e` / `v2c_chk` `N * 3` (bit `b`'s `k`-th
/// check at `b * 3 + k`).
#[doc(hidden)]
pub struct NmsEdges {
    /// The bit of check `j`'s `i`-th edge (`j * MAX_ROW + i`); padding is 0.
    pub(crate) nm_flat: &'static [u8],
    /// The row weight of each check.
    pub(crate) nrw: &'static [u8],
    /// The `tov` index (`bit * 3 + k`) of the entry that belongs to check `j`'s `i`-th edge: where the bit's own list holds `j`.
    pub(crate) c2v: &'static [u16],
    /// For the variable edge `bit * 3 + k`: its place on the check side, `check * MAX_ROW + slot`.
    pub(crate) v2c_e: &'static [u16],
    /// For the variable edge `bit * 3 + k`: its check.
    pub(crate) v2c_chk: &'static [u8],
}

/// The arrays [`NmsEdges`] points into.
pub(crate) struct EdgeArrays<const M: usize, const ER: usize, const V: usize> {
    nm_flat: [u8; ER],
    nrw: [u8; M],
    c2v: [u16; ER],
    v2c_e: [u16; V],
    v2c_chk: [u8; V],
}

/// Build the edge tables of a code at compile time: `mn` is `N` bits x 3 checks, `nm` `M` checks x `R` slots, `nrw` the row weights; `ER = M * R` and `V = N * 3` are
/// the table lengths. Every `assert!` is a **compile-time** check of what the decoder's unchecked reads rely on: a row weight at most `R`, each check listed once in a
/// bit's list, each bit found in its check's row, every index in range of the array it indexes, indices that fit `u8` / `u16`.
pub(crate) const fn edge_arrays<
    const N: usize,
    const M: usize,
    const R: usize,
    const ER: usize,
    const V: usize,
>(
    mn: &[[u8; 3]; N],
    nm: &[[u8; R]; M],
    nrw: &[u8; M],
) -> EdgeArrays<M, ER, V> {
    assert!(ER == M * R && V == N * 3);
    assert!(N <= 256 && M <= 256 && ER < 65536 && V < 65536);
    let mut a = EdgeArrays {
        nm_flat: [0; ER],
        nrw: [0; M],
        c2v: [0; ER],
        v2c_e: [0; V],
        v2c_chk: [0; V],
    };
    let mut j = 0;
    while j < M {
        assert!(nrw[j] as usize <= R);
        a.nrw[j] = nrw[j];
        let mut i = 0;
        while i < nrw[j] as usize {
            let bit = nm[j][i] as usize;
            assert!(bit < N);
            a.nm_flat[j * R + i] = bit as u8;
            // the bit's own list holds this check exactly once
            let mut found = 0;
            let mut kk = 0;
            while kk < 3 {
                if mn[bit][kk] as usize == j {
                    found += 1;
                    a.c2v[j * R + i] = (bit * 3 + kk) as u16;
                }
                kk += 1;
            }
            assert!(found == 1);
            i += 1;
        }
        j += 1;
    }
    let mut b = 0;
    while b < N {
        let mut k = 0;
        while k < 3 {
            let chk = mn[b][k] as usize;
            assert!(chk < M);
            // the first slot of the check's row that holds this bit (the search the decoder used to run on every pass)
            let mut slot = 0;
            while slot < nrw[chk] as usize && nm[chk][slot] as usize != b {
                slot += 1;
            }
            assert!(slot < nrw[chk] as usize);
            a.v2c_e[b * 3 + k] = (chk * R + slot) as u16;
            a.v2c_chk[b * 3 + k] = chk as u8;
            k += 1;
        }
        b += 1;
    }
    a
}

const EDGES_174_91: EdgeArrays<83, 581, 522> =
    edge_arrays::<174, 83, 7, 581, 522>(&tables::MN, &tables::NM, &tables::NRW);
const EDGES_240_101: EdgeArrays<139, 834, 720> = edge_arrays::<240, 139, 6, 834, 720>(
    &crate::fec::ldpc240_101::tables::MN,
    &crate::fec::ldpc240_101::tables::NM,
    &crate::fec::ldpc240_101::tables::NRW,
);
const EDGES_240_74: EdgeArrays<166, 830, 720> = edge_arrays::<240, 166, 5, 830, 720>(
    &crate::fec::ldpc240_74::tables::MN,
    &crate::fec::ldpc240_74::tables::NM,
    &crate::fec::ldpc240_74::tables::NRW,
);
const EDGES_128_90: EdgeArrays<38, 418, 384> = edge_arrays::<128, 38, 11, 418, 384>(
    &crate::fec::ldpc_128_90::tables::MN,
    &crate::fec::ldpc_128_90::tables::NM,
    &crate::fec::ldpc_128_90::tables::NRW,
);

macro_rules! nms_edges {
    ($name:ident, $arrays:ident) => {
        const $name: &'static NmsEdges = &NmsEdges {
            nm_flat: &$arrays.nm_flat,
            nrw: &$arrays.nrw,
            c2v: &$arrays.c2v,
            v2c_e: &$arrays.v2c_e,
            v2c_chk: &$arrays.v2c_chk,
        };
    };
}

/// Packs a `M × K` 0/1 generator-parity table into one `u128` per row
/// (`K ≤ 128`).
const fn pack_gen_parity<const K: usize, const M: usize>(t: &[[u8; K]; M]) -> [u128; M] {
    assert!(K <= 128);
    let mut out = [0u128; M];
    let mut r = 0;
    while r < M {
        let mut q = 0;
        while q < K {
            if t[r][q] != 0 {
                out[r] |= 1 << q;
            }
            q += 1;
        }
        r += 1;
    }
    out
}

const GEN_PARITY_174_91: [u128; 83] = pack_gen_parity(&super::osd::GEN_PARITY);
const GEN_PARITY_240_101: [u128; 139] =
    pack_gen_parity(&crate::fec::ldpc240_101::tables::GEN_PARITY);
const GEN_PARITY_240_74: [u128; 166] = pack_gen_parity(&crate::fec::ldpc240_74::tables::GEN_PARITY);
const GEN_PARITY_128_90: [u128; 38] = pack_gen_parity(&crate::fec::ldpc_128_90::tables::GEN_PARITY);

// ────────────────────────────────────────────────────────────────────
// LDPC(174, 91) — FT8 / FT4 / FT2
// ────────────────────────────────────────────────────────────────────

/// Parameters for the WSJT LDPC(174, 91) code (FT8 / FT4 / FT2).
/// Tables come from [`super::tables`] and the `GEN_PARITY` constant
/// in [`super::osd`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc174_91Params;

impl sealed::Sealed for Ldpc174_91Params {}

impl LdpcParams for Ldpc174_91Params {
    const N: usize = 174;
    const K: usize = 91;
    const MAX_ROW: usize = 7;
    const GEN_PARITY_PACKED: &'static [u128] = &GEN_PARITY_174_91;
    nms_edges!(NMS_EDGES, EDGES_174_91);

    #[inline]
    fn mn(bit: usize) -> [u8; 3] {
        tables::MN[bit]
    }

    #[inline]
    fn nm(check: usize, slot: usize) -> u8 {
        tables::NM[check][slot]
    }

    #[inline]
    fn nrw(check: usize) -> u8 {
        tables::NRW[check]
    }

    #[inline]
    fn gen_parity(row: usize, col: usize) -> u8 {
        super::osd::GEN_PARITY[row][col]
    }
}

// ────────────────────────────────────────────────────────────────────
// LDPC(240, 101) — FST4 / FST4W
// ────────────────────────────────────────────────────────────────────

/// Parameters for the WSJT LDPC(240, 101) code (FST4 / FST4W).
/// Tables come from [`crate::fec::ldpc240_101::tables`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc240_101Params;

impl sealed::Sealed for Ldpc240_101Params {}

impl LdpcParams for Ldpc240_101Params {
    const N: usize = 240;
    const K: usize = 101;
    const MAX_ROW: usize = 6;
    const GEN_PARITY_PACKED: &'static [u128] = &GEN_PARITY_240_101;
    nms_edges!(NMS_EDGES, EDGES_240_101);

    #[inline]
    fn mn(bit: usize) -> [u8; 3] {
        crate::fec::ldpc240_101::tables::MN[bit]
    }

    #[inline]
    fn nm(check: usize, slot: usize) -> u8 {
        crate::fec::ldpc240_101::tables::NM[check][slot]
    }

    #[inline]
    fn nrw(check: usize) -> u8 {
        crate::fec::ldpc240_101::tables::NRW[check]
    }

    #[inline]
    fn gen_parity(row: usize, col: usize) -> u8 {
        crate::fec::ldpc240_101::tables::GEN_PARITY[row][col]
    }
}

// ────────────────────────────────────────────────────────────────────
// LDPC(240, 74) — FST4W
// ────────────────────────────────────────────────────────────────────

/// Parameters for the WSJT LDPC(240, 74) code (FST4W: 50 message bits +
/// CRC-24). Tables come from [`crate::fec::ldpc240_74::tables`], generated
/// from `lib/fst4/ldpc_240_74_{parity,generator}.f90` at `v3.3.0-beta1`.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc240_74Params;

impl sealed::Sealed for Ldpc240_74Params {}

impl LdpcParams for Ldpc240_74Params {
    const N: usize = 240;
    const K: usize = 74;
    const MAX_ROW: usize = 5;
    const GEN_PARITY_PACKED: &'static [u128] = &GEN_PARITY_240_74;
    nms_edges!(NMS_EDGES, EDGES_240_74);

    #[inline]
    fn mn(bit: usize) -> [u8; 3] {
        crate::fec::ldpc240_74::tables::MN[bit]
    }

    #[inline]
    fn nm(check: usize, slot: usize) -> u8 {
        crate::fec::ldpc240_74::tables::NM[check][slot]
    }

    #[inline]
    fn nrw(check: usize) -> u8 {
        crate::fec::ldpc240_74::tables::NRW[check]
    }

    #[inline]
    fn gen_parity(row: usize, col: usize) -> u8 {
        crate::fec::ldpc240_74::tables::GEN_PARITY[row][col]
    }
}

// ────────────────────────────────────────────────────────────────────
// LDPC(128, 90) — MSK144
// ────────────────────────────────────────────────────────────────────

/// Parameters for the WSJT LDPC(128, 90) code (MSK144). Tables come
/// from [`crate::fec::ldpc_128_90::tables`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc128_90Params;

impl sealed::Sealed for Ldpc128_90Params {}

impl LdpcParams for Ldpc128_90Params {
    const N: usize = 128;
    const K: usize = 90;
    const MAX_ROW: usize = 11;
    const GEN_PARITY_PACKED: &'static [u128] = &GEN_PARITY_128_90;
    nms_edges!(NMS_EDGES, EDGES_128_90);

    #[inline]
    fn mn(bit: usize) -> [u8; 3] {
        crate::fec::ldpc_128_90::tables::MN[bit]
    }

    #[inline]
    fn nm(check: usize, slot: usize) -> u8 {
        crate::fec::ldpc_128_90::tables::NM[check][slot]
    }

    #[inline]
    fn nrw(check: usize) -> u8 {
        crate::fec::ldpc_128_90::tables::NRW[check]
    }

    #[inline]
    fn gen_parity(row: usize, col: usize) -> u8 {
        crate::fec::ldpc_128_90::tables::GEN_PARITY[row][col]
    }
}
