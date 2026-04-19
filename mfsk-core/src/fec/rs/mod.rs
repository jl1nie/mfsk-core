//! Reed-Solomon RS(63, 12) over GF(2^6) — the codec used by JT65.
//!
//! Ported from Phil Karn's classic Reed-Solomon library (as
//! wrapped in WSJT-X `wrapkarn.c` + `init_rs.c` / `encode_rs.c` /
//! `decode_rs.c`). Parameters match the `init_rs_int(6, 0x43, 3, 1,
//! 51, 0)` call used for JT65:
//!
//! - symbol size: 6 bits (field: GF(2^6), 63 nonzero elements)
//! - field generator polynomial: `x^6 + x + 1` (0x43)
//! - first consecutive root of generator polynomial: `α^3`
//! - primitive element for root stride: 1
//! - number of parity symbols (generator poly roots): 51
//! - code: (63, 12), corrects up to `⌊(63 − 12) / 2⌋ = 25` symbol errors
//!
//! ## Symbol ordering
//!
//! This module provides two entry-point pairs:
//!
//! - [`Rs63_12::encode_native`] / [`Rs63_12::decode_native`] — the
//!   canonical Karn layout: codeword = `[data12 || parity51]` with
//!   data / parity each in their original order. General-purpose.
//! - [`Rs63_12::encode_jt65`] / [`Rs63_12::decode_jt65`] — the
//!   JT65-specific byte ordering used by WSJT-X (`wrapkarn.c`):
//!   `sent[0..51]` is the parity block, reversed; `sent[51..63]`
//!   is the data block, reversed. Use these from `jt65-core`.
//!
//! This type implements [`super::FecCodec`] only minimally: the
//! `encode` path packs a bit-level 72-bit info into 378 codeword bits
//! (63 × 6 bits), and `decode_soft` always returns `None` because
//! Reed-Solomon needs hard symbols rather than bit LLRs. The real
//! RS entry points are [`Rs63_12::encode_symbols`] /
//! [`Rs63_12::decode_symbols`] (and the JT65-specific reversed-layout
//! variants). The FecCodec impl exists so `Rs63_12` can be named as
//! a `Protocol::Fec` associated type; callers that want to actually
//! decode JT65 should go through `jt65-core`'s decode helpers rather
//! than the generic `decode_frame` pipeline.

/// Sentinel used to mark "log of zero" in the index_of table. Matches
/// Karn's `A0 = NN` convention: any log value equal to `A0` represents
/// the field zero.
const A0: u8 = Rs63_12::NN as u8;

/// Encoder / decoder for Reed-Solomon (63, 12) over GF(2^6).
#[derive(Clone, Debug)]
pub struct Rs63_12 {
    /// `alpha_to[i] = α^i` for `0 ≤ i < 63`, 0 otherwise.
    alpha_to: [u8; 64],
    /// `index_of[x] = log_α(x)` for x ≠ 0, [`A0`] for x = 0.
    index_of: [u8; 64],
    /// Generator polynomial in index form.
    genpoly: [u8; Rs63_12::NROOTS + 1],
}

impl Default for Rs63_12 {
    fn default() -> Self {
        Self::new()
    }
}

impl Rs63_12 {
    /// Code length in symbols (2^6 − 1 = 63).
    pub const NN: usize = 63;
    /// Symbol size in bits.
    pub const MM: u32 = 6;
    /// First consecutive root of generator polynomial: `α^FCR = α^3`.
    pub const FCR: u32 = 3;
    /// Primitive element for generator roots (α^(FCR·PRIM + i·PRIM)).
    pub const PRIM: u32 = 1;
    /// Number of parity symbols.
    pub const NROOTS: usize = 51;
    /// Info-symbol count (NN − NROOTS).
    pub const K_SYMBOLS: usize = Self::NN - Self::NROOTS;
    /// Full codeword length in symbols.
    pub const N_SYMBOLS: usize = Self::NN;
    /// Field generator polynomial (x^6 + x + 1).
    const GFPOLY: u32 = 0x43;

    /// Build the codec: pre-compute alpha_to, index_of, and genpoly.
    pub fn new() -> Self {
        let mut alpha_to = [0u8; 64];
        let mut index_of = [0u8; 64];

        // Galois field tables.
        index_of[0] = A0; // log(0) = −∞
        alpha_to[A0 as usize] = 0; // α^{−∞} = 0
        let mut sr: u32 = 1;
        for i in 0..Self::NN {
            index_of[sr as usize] = i as u8;
            alpha_to[i] = sr as u8;
            sr <<= 1;
            if sr & (1 << Self::MM) != 0 {
                sr ^= Self::GFPOLY;
            }
            sr &= Self::NN as u32;
        }
        debug_assert_eq!(sr, 1, "gfpoly must be primitive");

        // Generator polynomial g(x) = Π (x − α^(FCR + i)·PRIM) for i=0..NROOTS.
        let mut genpoly = [0u8; Self::NROOTS + 1];
        genpoly[0] = 1;
        let mut root = Self::FCR * Self::PRIM;
        for i in 0..Self::NROOTS {
            genpoly[i + 1] = 1;
            // Multiply by (x + α^root). In the poly loop below, j descends.
            for j in (1..=i).rev() {
                if genpoly[j] != 0 {
                    let idx = Self::modnn(index_of[genpoly[j] as usize] as u32 + root);
                    genpoly[j] = genpoly[j - 1] ^ alpha_to[idx as usize];
                } else {
                    genpoly[j] = genpoly[j - 1];
                }
            }
            // genpoly[0] is always nonzero at this stage.
            let idx = Self::modnn(index_of[genpoly[0] as usize] as u32 + root);
            genpoly[0] = alpha_to[idx as usize];
            root += Self::PRIM;
        }
        // Convert genpoly to index form for faster encoding.
        for g in genpoly.iter_mut() {
            *g = index_of[*g as usize];
        }

        Self {
            alpha_to,
            index_of,
            genpoly,
        }
    }

    /// `x mod NN` via a subtract-in-a-loop that terminates in at most
    /// one iteration for inputs `< 2·NN`.
    #[inline]
    fn modnn(mut x: u32) -> u32 {
        while x >= Self::NN as u32 {
            x -= Self::NN as u32;
            x = (x >> Self::MM) + (x & Self::NN as u32);
        }
        x
    }

    /// Systematic encode: 12 info symbols → 51 parity symbols. Layout
    /// in the native Karn order is `[info[0..12] || parity[0..51]]`.
    pub fn encode_native(&self, info: &[u8; Self::K_SYMBOLS]) -> [u8; Self::N_SYMBOLS] {
        let mut bb = [0u8; Self::NROOTS];
        for i in 0..Self::K_SYMBOLS {
            let feedback = self.index_of[(info[i] ^ bb[0]) as usize];
            if feedback != A0 {
                for j in 1..Self::NROOTS {
                    bb[j] ^= self.alpha_to
                        [Self::modnn(feedback as u32 + self.genpoly[Self::NROOTS - j] as u32) as usize];
                }
            }
            // Shift bb left by one.
            for j in 0..Self::NROOTS - 1 {
                bb[j] = bb[j + 1];
            }
            bb[Self::NROOTS - 1] = if feedback != A0 {
                self.alpha_to[Self::modnn(feedback as u32 + self.genpoly[0] as u32) as usize]
            } else {
                0
            };
        }
        let mut out = [0u8; Self::N_SYMBOLS];
        out[..Self::K_SYMBOLS].copy_from_slice(info);
        out[Self::K_SYMBOLS..].copy_from_slice(&bb);
        out
    }

    /// Decode a received codeword in the native Karn layout with no
    /// erasures. Returns `Some((corrected, err_count))` on success,
    /// `None` when uncorrectable.
    pub fn decode_native(
        &self,
        data: &[u8; Self::N_SYMBOLS],
    ) -> Option<([u8; Self::K_SYMBOLS], u32)> {
        self.decode_native_erasures(data, &[])
    }

    /// Like [`decode_native`] but also accepts a list of **erasure
    /// positions** (symbol indices 0..=62 in the native codeword
    /// layout that the caller has flagged as unreliable). Each
    /// erasure lets RS correct one more symbol than the
    /// ⌊(NROOTS)/2⌋ = 25 hard-error bound: the combined limit is
    /// `2·errors + erasures ≤ NROOTS = 51`. Passing erasures is
    /// particularly helpful at low SNR where the demodulator has
    /// per-symbol confidence information.
    ///
    /// Ported from Phil Karn's `decode_rs.c` with the `no_eras > 0`
    /// branch active. Duplicate or out-of-range entries in
    /// `eras_pos` will produce `None` from the Chien search.
    pub fn decode_native_erasures(
        &self,
        data: &[u8; Self::N_SYMBOLS],
        eras_pos: &[u32],
    ) -> Option<([u8; Self::K_SYMBOLS], u32)> {
        let no_eras = eras_pos.len();
        if no_eras > Self::NROOTS {
            return None; // more erasures than parity — uncorrectable a priori
        }
        let mut recd = *data;

        // 1. Syndromes — evaluate recd(x) at α^(FCR + i·PRIM) for i=0..NROOTS.
        let mut s = [0u8; Self::NROOTS];
        for i in 0..Self::NROOTS {
            s[i] = recd[0];
        }
        for j in 1..Self::NN {
            for i in 0..Self::NROOTS {
                if s[i] == 0 {
                    s[i] = recd[j];
                } else {
                    let sidx = self.index_of[s[i] as usize] as u32
                        + (Self::FCR + i as u32) * Self::PRIM;
                    s[i] = recd[j] ^ self.alpha_to[Self::modnn(sidx) as usize];
                }
            }
        }

        // Convert syndromes to index form + detect non-zero syndrome.
        let mut syn_error: u8 = 0;
        for i in 0..Self::NROOTS {
            syn_error |= s[i];
            s[i] = self.index_of[s[i] as usize];
        }
        if syn_error == 0 {
            let mut info = [0u8; Self::K_SYMBOLS];
            info.copy_from_slice(&recd[..Self::K_SYMBOLS]);
            return Some((info, 0));
        }

        // 2. Berlekamp-Massey. When erasures are supplied, initialise
        //    λ(x) to the erasure locator polynomial
        //        λ(x) = Π (1 + β_j·x), β_j = α^(PRIM·(NN−1−pos_j))
        //    and start BM at r = el = no_eras.
        let mut lambda = [0u8; Self::NROOTS + 1];
        lambda[0] = 1;
        let mut b = [0u8; Self::NROOTS + 1];
        let mut t = [0u8; Self::NROOTS + 1];

        if no_eras > 0 {
            for &pos in eras_pos {
                if pos as usize >= Self::NN {
                    return None;
                }
            }
            let e0 = Self::modnn(Self::PRIM * (Self::NN as u32 - 1 - eras_pos[0]));
            lambda[1] = self.alpha_to[e0 as usize];
            for i in 1..no_eras {
                let u = Self::modnn(Self::PRIM * (Self::NN as u32 - 1 - eras_pos[i]));
                for j in (1..=i + 1).rev() {
                    let tmp = self.index_of[lambda[j - 1] as usize];
                    if tmp != A0 {
                        lambda[j] ^=
                            self.alpha_to[Self::modnn(u + tmp as u32) as usize];
                    }
                }
            }
        }

        for i in 0..Self::NROOTS + 1 {
            b[i] = self.index_of[lambda[i] as usize];
        }

        let mut el: i32 = no_eras as i32;
        for r in (no_eras + 1)..=Self::NROOTS {
            // Discrepancy at step r (in poly form).
            let mut discr_r: u8 = 0;
            for i in 0..r {
                if lambda[i] != 0 && s[r - i - 1] != A0 {
                    let idx = self.index_of[lambda[i] as usize] as u32 + s[r - i - 1] as u32;
                    discr_r ^= self.alpha_to[Self::modnn(idx) as usize];
                }
            }
            let discr_idx = self.index_of[discr_r as usize];
            if discr_idx == A0 {
                // B(x) ← x·B(x)
                for j in (1..=Self::NROOTS).rev() {
                    b[j] = b[j - 1];
                }
                b[0] = A0;
            } else {
                // T(x) ← λ(x) − discr_r·x·B(x)
                t[0] = lambda[0];
                for i in 0..Self::NROOTS {
                    if b[i] != A0 {
                        t[i + 1] = lambda[i + 1]
                            ^ self.alpha_to[Self::modnn(discr_idx as u32 + b[i] as u32) as usize];
                    } else {
                        t[i + 1] = lambda[i + 1];
                    }
                }
                // With erasures the BM invariant becomes
                // 2·el ≤ r + no_eras − 1.
                if 2 * el <= r as i32 + no_eras as i32 - 1 {
                    el = r as i32 + no_eras as i32 - el;
                    for i in 0..=Self::NROOTS {
                        b[i] = if lambda[i] == 0 {
                            A0
                        } else {
                            Self::modnn(
                                self.index_of[lambda[i] as usize] as u32 + Self::NN as u32
                                    - discr_idx as u32,
                            ) as u8
                        };
                    }
                } else {
                    for j in (1..=Self::NROOTS).rev() {
                        b[j] = b[j - 1];
                    }
                    b[0] = A0;
                }
                lambda.copy_from_slice(&t);
            }
        }

        // deg(λ) and conversion to index form.
        let mut deg_lambda = 0usize;
        let mut lambda_idx = [0u8; Self::NROOTS + 1];
        for i in 0..Self::NROOTS + 1 {
            lambda_idx[i] = self.index_of[lambda[i] as usize];
            if lambda_idx[i] != A0 {
                deg_lambda = i;
            }
        }

        // 3. Chien search for roots of λ(x).
        let iprim: u32 = Self::find_iprim();
        let mut reg = [0u8; Self::NROOTS + 1];
        reg[1..].copy_from_slice(&lambda_idx[1..]);
        let mut root = [0u32; Self::NROOTS];
        let mut loc = [0u32; Self::NROOTS];
        let mut count = 0usize;
        let mut k_idx: u32 = Self::modnn(iprim + Self::NN as u32 - 1);
        for i in 1..=Self::NN as u32 {
            let mut q: u8 = 1;
            for j in (1..=deg_lambda).rev() {
                if reg[j] != A0 {
                    reg[j] = Self::modnn(reg[j] as u32 + j as u32) as u8;
                    q ^= self.alpha_to[reg[j] as usize];
                }
            }
            if q != 0 {
                k_idx = Self::modnn(k_idx + iprim);
                continue;
            }
            root[count] = i;
            loc[count] = k_idx;
            count += 1;
            if count == deg_lambda {
                break;
            }
            k_idx = Self::modnn(k_idx + iprim);
        }
        if deg_lambda != count {
            return None; // uncorrectable
        }

        // 4. Compute ω(x) = s(x)·λ(x) mod x^NROOTS.
        let deg_omega = deg_lambda.saturating_sub(1);
        let mut omega = [0u8; Self::NROOTS + 1];
        for i in 0..=deg_omega {
            let mut tmp: u8 = 0;
            for j in 0..=i {
                if s[i - j] != A0 && lambda_idx[j] != A0 {
                    tmp ^= self.alpha_to
                        [Self::modnn(s[i - j] as u32 + lambda_idx[j] as u32) as usize];
                }
            }
            omega[i] = self.index_of[tmp as usize];
        }

        // 5. Forney's formula for error values, applied in place.
        for j in (0..count).rev() {
            // num1 = Σ ω[i] · root[j]^i
            let mut num1: u8 = 0;
            for i in (0..=deg_omega).rev() {
                if omega[i] != A0 {
                    num1 ^= self.alpha_to
                        [Self::modnn(omega[i] as u32 + (i as u32) * root[j]) as usize];
                }
            }
            let num2 = self.alpha_to[Self::modnn(root[j] * (Self::FCR - 1) + Self::NN as u32) as usize];

            // den = λ_prime(X^{-1}) — formal derivative, odd-indexed terms.
            let mut den: u8 = 0;
            let end = deg_lambda.min(Self::NROOTS - 1) & !1;
            let mut i = end as i32;
            while i >= 0 {
                if lambda_idx[i as usize + 1] != A0 {
                    den ^= self.alpha_to
                        [Self::modnn(lambda_idx[i as usize + 1] as u32 + (i as u32) * root[j]) as usize];
                }
                i -= 2;
            }
            if den == 0 {
                return None;
            }
            if num1 != 0 {
                let err = self.alpha_to[Self::modnn(
                    self.index_of[num1 as usize] as u32
                        + self.index_of[num2 as usize] as u32
                        + Self::NN as u32
                        - self.index_of[den as usize] as u32,
                ) as usize];
                let pos = loc[j] as usize;
                if pos < Self::NN {
                    recd[pos] ^= err;
                }
            }
        }

        // Extract the (now corrected) info symbols from the systematic
        // prefix and return the error count.
        let mut info = [0u8; Self::K_SYMBOLS];
        info.copy_from_slice(&recd[..Self::K_SYMBOLS]);
        Some((info, count as u32))
    }

    /// Primitive-root helper. With PRIM = 1, the prim-th root of 1 is
    /// just 1 itself, so iprim = 1. Computed generically for clarity.
    fn find_iprim() -> u32 {
        let mut iprim: u32 = 1;
        while iprim % Self::PRIM != 0 {
            iprim += Self::NN as u32;
        }
        iprim / Self::PRIM
    }

    // ─────────────────────────────────────────────────────────────────
    // WSJT-X (JT65) wrappers
    // ─────────────────────────────────────────────────────────────────

    /// Encode for JT65 with the byte ordering WSJT-X expects
    /// (`wrapkarn.c::rs_encode_`): info is reversed before encoding,
    /// and the output places parity (reversed) at `[0..51]` and data
    /// (reversed) at `[51..63]`.
    pub fn encode_jt65(&self, info: &[u8; Self::K_SYMBOLS]) -> [u8; Self::N_SYMBOLS] {
        let mut dat1 = [0u8; Self::K_SYMBOLS];
        for i in 0..Self::K_SYMBOLS {
            dat1[i] = info[Self::K_SYMBOLS - 1 - i];
        }
        let cw = self.encode_native(&dat1);
        // cw = [dat1 || parity]; transform to WSJT-X layout.
        let mut sent = [0u8; Self::N_SYMBOLS];
        for i in 0..Self::NROOTS {
            sent[Self::NROOTS - 1 - i] = cw[Self::K_SYMBOLS + i];
        }
        for i in 0..Self::K_SYMBOLS {
            sent[Self::NROOTS + i] = dat1[Self::K_SYMBOLS - 1 - i];
        }
        sent
    }

    /// Decode JT65 symbols with the WSJT-X layout. Returns
    /// `Some((info, err_count))` or `None` if uncorrectable.
    pub fn decode_jt65(
        &self,
        recd0: &[u8; Self::N_SYMBOLS],
    ) -> Option<([u8; Self::K_SYMBOLS], u32)> {
        self.decode_jt65_erasures(recd0, &[])
    }

    /// JT65-layout decode with a caller-supplied list of **erasure
    /// positions in the WSJT-X `sent[]` layout** (0..=50 = parity
    /// reversed; 51..=62 = data reversed). The positions are
    /// translated to the native Karn layout (identity mapping
    /// `native = NN − 1 − wsjt` for both halves) before entering
    /// [`decode_native_erasures`].
    pub fn decode_jt65_erasures(
        &self,
        recd0: &[u8; Self::N_SYMBOLS],
        eras_pos_wsjt: &[u32],
    ) -> Option<([u8; Self::K_SYMBOLS], u32)> {
        let mut recd = [0u8; Self::N_SYMBOLS];
        for i in 0..Self::K_SYMBOLS {
            recd[i] = recd0[Self::NN - 1 - i];
        }
        for i in 0..Self::NROOTS {
            recd[Self::K_SYMBOLS + i] = recd0[Self::NROOTS - 1 - i];
        }
        // The WSJT-X ↔ native index relation is `native = NN − 1 − wsjt`
        // on both halves of the codeword (verified against the loops
        // above). Translate the caller's erasure positions.
        let eras_native: Vec<u32> = eras_pos_wsjt
            .iter()
            .filter(|&&p| (p as usize) < Self::NN)
            .map(|&p| (Self::NN as u32 - 1) - p)
            .collect();
        let (info_native, nerr) = self.decode_native_erasures(&recd, &eras_native)?;
        let mut info = [0u8; Self::K_SYMBOLS];
        for i in 0..Self::K_SYMBOLS {
            info[i] = info_native[Self::K_SYMBOLS - 1 - i];
        }
        Some((info, nerr))
    }
}

// ─────────────────────────────────────────────────────────────────────────
// FecCodec boundary stub
//
// Rs63_12 is used as `<Jt65 as Protocol>::Fec`. The `Protocol` trait
// requires `type Fec: FecCodec`, which is a bit-LLR-oriented interface
// that does not map naturally onto hard-decision symbol-level RS. We
// provide the minimum viable impl: `encode` packs 12 × 6 = 72 info
// bits into 63 × 6 = 378 codeword bits using the JT65 layout, and
// `decode_soft` always returns `None` — hard-symbol RS decoding lives
// in `jt65-core` and uses `decode_jt65` / `decode_native` directly.
// ─────────────────────────────────────────────────────────────────────────

use crate::core::{FecOpts, FecResult};

impl super::FecCodec for Rs63_12 {
    const N: usize = Rs63_12::N_SYMBOLS * 6; // 63 × 6 = 378 bits
    const K: usize = Rs63_12::K_SYMBOLS * 6; // 12 × 6 = 72 bits

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), Self::K);
        assert_eq!(codeword.len(), Self::N);
        // Pack 72 bits (MSB-first within each 6-bit symbol) into 12 symbols.
        let mut info_syms = [0u8; Rs63_12::K_SYMBOLS];
        for (i, slot) in info_syms.iter_mut().enumerate() {
            let mut w = 0u8;
            for b in 0..6 {
                w = (w << 1) | (info[6 * i + b] & 1);
            }
            *slot = w;
        }
        let sent = self.encode_jt65(&info_syms);
        // Expand 63 × 6-bit symbols back into 378 bits (MSB-first).
        for (i, &sym) in sent.iter().enumerate() {
            for b in 0..6 {
                codeword[6 * i + b] = (sym >> (5 - b)) & 1;
            }
        }
    }

    /// Symbol-hard RS decoding cannot consume bit LLRs, so this path
    /// returns `None`. Callers that want JT65 decoding should use the
    /// symbol-level methods on [`Rs63_12`] from `jt65-core`.
    fn decode_soft(&self, _llr: &[f32], _opts: &FecOpts) -> Option<FecResult> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_are_primitive() {
        let rs = Rs63_12::new();
        // Every nonzero element 1..=62 should have a log, and α^log(x) = x.
        for x in 1u8..=62 {
            let lg = rs.index_of[x as usize];
            assert_ne!(lg, A0, "log of {x} is A0");
            assert_eq!(rs.alpha_to[lg as usize], x, "alpha_to[index_of[{x}]] != {x}");
        }
    }

    #[test]
    fn encode_clean_codeword_has_zero_syndrome() {
        // Round-tripping a clean codeword through decode_native must
        // yield err_count == 0 and recover the original info.
        let rs = Rs63_12::new();
        let info: [u8; 12] = [0, 1, 2, 3, 4, 5, 62, 61, 7, 8, 33, 44];
        let cw = rs.encode_native(&info);
        let (decoded, nerr) = rs.decode_native(&cw).expect("clean decode");
        assert_eq!(decoded, info);
        assert_eq!(nerr, 0);
    }

    #[test]
    fn corrects_single_error() {
        let rs = Rs63_12::new();
        let info: [u8; 12] = [12, 34, 56, 7, 8, 9, 10, 11, 42, 21, 0, 63 & 0x3f];
        let mut cw = rs.encode_native(&info);
        cw[17] ^= 0x2a; // inject error
        let (decoded, nerr) = rs.decode_native(&cw).expect("1 error correctable");
        assert_eq!(decoded, info);
        assert_eq!(nerr, 1);
    }

    #[test]
    fn corrects_max_errors() {
        // (63 − 12) / 2 = 25 errors correctable.
        let rs = Rs63_12::new();
        let info: [u8; 12] = [5, 17, 29, 41, 53, 62, 1, 13, 25, 37, 49, 61];
        let mut cw = rs.encode_native(&info);
        // Flip 25 scattered symbols, each by a different nonzero value.
        let positions = [0, 3, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 56, 59, 62, 1, 4, 7];
        // Pick a nonzero XOR value (1..=31) so every flip really is an
        // error — a 0x40 XOR masked down to 0 is a no-op and would
        // reduce the effective error count.
        for (i, &p) in positions.iter().enumerate() {
            let delta = ((i as u8 * 7 + 1) & 0x1f) | 1;
            cw[p] ^= delta;
            debug_assert!(delta != 0);
        }
        let (decoded, nerr) = rs.decode_native(&cw).expect("25 errors correctable");
        assert_eq!(decoded, info);
        assert_eq!(nerr, 25);
    }

    #[test]
    fn rejects_26_errors() {
        let rs = Rs63_12::new();
        let info: [u8; 12] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
        let mut cw = rs.encode_native(&info);
        // 26 symbol errors exceed the correctable bound; decoder should
        // return None or miscorrect — either way, it should NOT silently
        // claim the wrong info with err_count == 26.
        for p in 0..26 {
            cw[p] ^= 0x15;
        }
        match rs.decode_native(&cw) {
            None => {} // expected — uncorrectable
            Some((decoded, _)) => {
                // Decoder may return "a valid codeword" ≠ original.
                assert_ne!(decoded, info, "must not decode to original beyond bound");
            }
        }
    }

    #[test]
    fn erasures_only_all_51_parity() {
        // 51 erasures on the parity block + 0 errors in data → should
        // decode. Saturates the `2·errors + eras ≤ NROOTS = 51` bound.
        let rs = Rs63_12::new();
        let info: [u8; 12] = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 42, 21];
        let mut cw = rs.encode_native(&info);
        // Zero out parity (positions 12..63 in native layout) and
        // mark them erased.
        let mut eras = Vec::new();
        for p in 12..63u32 {
            cw[p as usize] = 0;
            eras.push(p);
        }
        let (decoded, nerr) = rs
            .decode_native_erasures(&cw, &eras)
            .expect("51-erasure decode");
        assert_eq!(decoded, info);
        assert_eq!(nerr, 51);
    }

    #[test]
    fn erasures_let_us_correct_beyond_25_errors() {
        // Inject 30 symbol errors BUT tell the decoder where 20 of them
        // are (erasures). That leaves 10 unknown error positions — well
        // inside the new bound (`2·10 + 20 = 40 ≤ 51`).
        let rs = Rs63_12::new();
        let info: [u8; 12] = [1, 13, 25, 37, 49, 61, 5, 17, 29, 41, 53, 62];
        let mut cw = rs.encode_native(&info);
        // Flip 30 distinct positions. (i*2) mod 63 walks all residues
        // once because gcd(2, 63) = 1, but dedupe defensively.
        let positions: Vec<usize> = {
            let mut s = Vec::with_capacity(30);
            let mut used = [false; 63];
            for i in 0..63 {
                let p = (i * 2) % 63;
                if !used[p] {
                    used[p] = true;
                    s.push(p);
                    if s.len() == 30 {
                        break;
                    }
                }
            }
            s
        };
        for (i, &p) in positions.iter().enumerate() {
            let delta = ((i as u8 * 7 + 1) & 0x1f) | 1;
            cw[p] ^= delta;
        }
        // Reveal the first 20 as erasures.
        let eras: Vec<u32> = positions.iter().take(20).map(|&p| p as u32).collect();
        let (decoded, _nerr) = rs
            .decode_native_erasures(&cw, &eras)
            .expect("20 erasures + 10 errors must decode");
        assert_eq!(decoded, info);
    }

    #[test]
    fn jt65_wrapper_roundtrip() {
        // Verify the reversed-layout wrappers are mutual inverses on a
        // clean codeword.
        let rs = Rs63_12::new();
        let info: [u8; 12] = [0, 1, 2, 3, 4, 5, 62, 61, 7, 8, 33, 44];
        let sent = rs.encode_jt65(&info);
        let (decoded, nerr) = rs.decode_jt65(&sent).expect("jt65 clean roundtrip");
        assert_eq!(decoded, info);
        assert_eq!(nerr, 0);
    }
}
