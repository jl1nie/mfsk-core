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
//! `XxxParams` ZST, implement [`sealed::Sealed`] and [`LdpcParams`]
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
    /// Number of parity checks (= `N - K`).
    const M: usize;
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
}

// ────────────────────────────────────────────────────────────────────
// LDPC(174, 91) — FT8 / FT4 / FT2
// ────────────────────────────────────────────────────────────────────

/// Parameters for the WSJT LDPC(174, 91) code (FT8 / FT4 / FT2).
/// Tables come from [`super::tables`] and [`super::osd::GEN_PARITY`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc174_91Params;

impl sealed::Sealed for Ldpc174_91Params {}

impl LdpcParams for Ldpc174_91Params {
    const N: usize = 174;
    const K: usize = 91;
    const M: usize = 83;
    const MAX_ROW: usize = 7;

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
    const M: usize = 139;
    const MAX_ROW: usize = 6;

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
