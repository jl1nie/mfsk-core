//! Belief-Propagation (log-domain) decoder, generic over [`LdpcParams`].
//!
//! Originally ported from WSJT-X `bpdecode174_91.f90`. Phase 0c-B
//! generalised the algorithm so [`Ldpc174_91`](super::Ldpc174_91) and
//! [`Ldpc240_101`](crate::fec::Ldpc240_101) share a single
//! implementation; the matrix shape comes from `P` at compile time.
//!
//! For backward compatibility (FT8's bespoke decode path goes through
//! [`bp_decode`] directly), this module also exposes a non-generic
//! [`bp_decode`] that pins `P = Ldpc174_91Params` — same behaviour as
//! before, just routed through the generic body.

use alloc::vec;
use alloc::vec::Vec;

// Float methods (.atanh / .signum) are inherent on f32 under std but
// require this trait under no_std (where libm provides them).
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::params::{Ldpc174_91Params, LdpcParams};
use super::{LDPC_K, LDPC_N};
pub use crate::core::BpKind;

/// Column weight (variable-node degree). Both LDPC codes in this
/// crate are uniform with `NCW = 3`.
const NCW: usize = 3;

/// WSJT-X `platanh.f90` piecewise-linear approximation.
///
/// This is intentionally not `f32::atanh`: the hybrid BP/OSD decoder
/// feeds accumulated beliefs from its first two iterations into OSD,
/// so matching the reference transfer curve (including its ±7
/// saturation) is part of decoder parity.
#[inline]
fn platanh(x: f32) -> f32 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let z = x.abs();
    if z <= 0.664 {
        x / 0.83
    } else if z <= 0.9217 {
        sign * (z - 0.4064) / 0.322
    } else if z <= 0.9951 {
        sign * (z - 0.8378) / 0.0524
    } else if z <= 0.9998 {
        sign * (z - 0.9914) / 0.0012
    } else {
        sign * 7.0
    }
}

/// CRC-14 (polynomial 0x2757) over `data` bytes, processed MSB-first.
/// Matches boost::augmented_crc<14, 0x2757> used in WSJT-X crc14.cpp.
pub fn crc14(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let msb = (crc >> 13) & 1;
            crc = ((crc << 1) | bit as u16) & 0x3FFF;
            if msb != 0 {
                crc ^= 0x2757;
            }
        }
    }
    crc
}

/// Verify CRC-14 for a 91-bit decoded word (77 msg + 14 CRC).
/// Packs bits into 12 bytes (big-endian, MSB first), zeros the CRC field,
/// computes CRC-14, then compares with the stored CRC bits.
///
/// Accepts any `&[u8]` slice; lengths other than 91 are rejected so the
/// function is suitable as a `MessageCodec::verify_info` implementation
/// passed through `FecOpts::verify_info`.
pub fn check_crc14(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut bytes = [0u8; 12];
    for (i, &bit) in decoded[..77].iter().enumerate() {
        let byte_idx = i / 8;
        let bit_pos = 7 - (i % 8);
        bytes[byte_idx] |= (bit & 1) << bit_pos;
    }

    let computed = crc14(&bytes);

    let mut received: u16 = 0;
    for &bit in &decoded[77..91] {
        received = (received << 1) | (bit as u16 & 1);
    }

    computed == received
}

/// Output of a successful BP decode.
///
/// `info` is the systematic prefix (length `P::K`); `codeword` is the
/// full decoded codeword (length `P::N`). Both are heap-allocated so
/// the struct can serve any [`LdpcParams`] without const-generic
/// gymnastics. `message77` exposes the leading 77 bits as a fixed-size
/// array for the Wsjt77-family ergonomics that pre-existing FT8 code
/// relies on; non-Wsjt77 callers ignore it and read `info`.
pub struct BpResult {
    /// Leading 77 info bits (Wsjt77 message field). Same content as
    /// `info[..77]` — duplicated here for callers that take fixed-size
    /// references.
    pub message77: [u8; 77],
    /// Full systematic info (length `P::K`).
    pub info: Vec<u8>,
    /// Full codeword bits (length `P::N`).
    pub codeword: Vec<u8>,
    /// Number of hard errors (bits where hard decision disagrees with LLR sign).
    pub hard_errors: u32,
    /// Number of BP iterations executed.
    pub iterations: u32,
}

/// Generic log-domain Belief-Propagation decode.
///
/// `llr.len()` and (if present) `ap_mask.len()` must equal `P::N`.
///
/// `verify` is an optional integrity check applied to each parity-
/// converged candidate. When `Some`, BP keeps iterating past a
/// parity-only convergence whose verification fails (mirroring how
/// CRC-aware codecs behave under noise that leaves multiple valid
/// codewords near the LLR estimate). When `None`, BP returns on first
/// parity convergence — appropriate for codecs whose message codec
/// carries no internal integrity field.
pub fn bp_decode_generic<P: LdpcParams>(
    llr: &[f32],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    bp_decode_generic_kind::<P>(llr, ap_mask, max_iter, verify, BpKind::SumProduct)
}

/// Generic log-domain Belief-Propagation decode with selectable
/// check-node kernel. The default-kind wrapper [`bp_decode_generic`]
/// pins `kind = SumProduct` for backward compatibility; embedded
/// callers pick `NormalizedMinSum { alpha: 0.75 }` (or
/// `OffsetMinSum { beta: 0.5 }`) to skip the per-iteration `tanh` /
/// `atanh` cache and use a min-sum approximation instead.
///
/// **min1 / min2 + XOR-sign trick**: for both min-sum kernels the
/// check-node update precomputes `(min1, min2, sign_xor)` per check
/// once per iteration; the per-edge output then picks `min2` if the
/// edge's own |L| matches `min1`, else `min1`. This brings the
/// inner-loop cost down to O(check_degree) instead of the
/// sum-product's O(check_degree²) tanh-cache lookups, before any
/// floating-point savings are counted.
///
/// On WSJT LDPC(174,91) and LDPC(240,101), with `α = 0.75`:
/// threshold loss vs `SumProduct` is sub-0.2 dB on AWGN sweeps —
/// usually invisible at the operating point.
pub fn bp_decode_generic_kind<P: LdpcParams>(
    llr: &[f32],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    kind: BpKind,
) -> Option<BpResult> {
    debug_assert_eq!(llr.len(), P::N, "llr length must equal P::N");
    if let Some(m) = ap_mask {
        debug_assert_eq!(m.len(), P::N, "ap_mask length must equal P::N");
    }

    let n = P::N;
    let m_checks = P::M;
    let k = P::K;
    let max_row = P::MAX_ROW;

    // Heap-allocated working buffers. Sizes:
    //   tov     : N * NCW   (≤ 720 bytes for ldpc240_101)
    //   toc     : M * MAX_ROW
    //   tanhtoc : M * MAX_ROW   (sum-product only)
    //   per-check (min1, min2, idx_min1, sign_xor): 4 × M words
    //                            (min-sum only)
    //   zn      : N
    //   cw      : N
    // For both codes the total stays under 8 KB — negligible vs the
    // 30+ BP iterations of inner-loop arithmetic.
    let mut tov = vec![0f32; n * NCW];
    let mut toc = vec![0f32; m_checks * max_row];
    // Allocate tanhtoc only on the SumProduct path; min-sum doesn't
    // need it. Saving the alloc + the per-iteration loop is one of
    // the speedups; the rest comes from skipping `tanh` / `atanh`.
    let mut tanhtoc: Vec<f32> = match kind {
        BpKind::SumProduct => vec![0f32; m_checks * max_row],
        BpKind::NormalizedMinSum { .. } | BpKind::OffsetMinSum { .. } => Vec::new(),
    };
    // Min-sum scratch: per check node, the two smallest |L|, the
    // edge index that holds min1, and the XOR'd sign of all incoming
    // edges (true = negative). Allocated on min-sum paths only.
    let mut min1 = vec![0f32; m_checks];
    let mut min2 = vec![0f32; m_checks];
    let mut idx_min1 = vec![0u32; m_checks];
    let mut sign_xor = vec![false; m_checks];
    let mut zn = vec![0f32; n];
    let mut cw = vec![0u8; n];

    // Initial messages: each check node receives the raw LLR for the
    // bits it tests.
    for j in 0..m_checks {
        let nrw_j = P::nrw(j) as usize;
        for i in 0..nrw_j {
            let bit = P::nm(j, i) as usize;
            toc[j * max_row + i] = llr[bit];
        }
    }

    let mut ncnt = 0u32;
    let mut nclast = 0u32;

    for iter in 0..=max_iter {
        // Variable-node update: zn = llr + Σ tov, except AP-locked
        // bits hold their LLR fixed.
        for i in 0..n {
            let ap = ap_mask.is_some_and(|mm| mm[i]);
            if !ap {
                let mut sum = 0.0f32;
                for k_ in 0..NCW {
                    sum += tov[i * NCW + k_];
                }
                zn[i] = llr[i] + sum;
            } else {
                zn[i] = llr[i];
            }
        }

        // Hard decisions.
        for i in 0..n {
            cw[i] = if zn[i] > 0.0 { 1 } else { 0 };
        }

        // Count parity-violating checks.
        let mut ncheck = 0u32;
        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            let mut parity = 0u8;
            for s in 0..nrw_i {
                parity ^= cw[P::nm(i, s) as usize];
            }
            if parity != 0 {
                ncheck += 1;
            }
        }

        if ncheck == 0 {
            let mut decoded = vec![0u8; k];
            decoded.copy_from_slice(&cw[..k]);
            // No verifier → accept any parity-converged candidate.
            // With a verifier (e.g. CRC-14/24 length-dispatched in
            // Wsjt77Message::verify_info) → accept only on true.
            let accept = match verify {
                Some(f) => f(&decoded),
                None => true,
            };
            if accept {
                let mut hard_errors = 0u32;
                for i in 0..n {
                    if (cw[i] == 1) != (llr[i] > 0.0) {
                        hard_errors += 1;
                    }
                }
                let mut message77 = [0u8; 77];
                let message_len = decoded.len().min(message77.len());
                message77[..message_len].copy_from_slice(&decoded[..message_len]);
                return Some(BpResult {
                    message77,
                    info: decoded,
                    codeword: cw,
                    hard_errors,
                    iterations: iter,
                });
            }
        }

        // Stall detector: same heuristic as the WSJT-X reference.
        if iter > 0 {
            if ncheck < nclast {
                ncnt = 0;
            } else {
                ncnt += 1;
            }
            if ncnt >= 5 && iter >= 10 && ncheck > 15 {
                return None;
            }
        }
        nclast = ncheck;

        // Check-to-variable message update (extrinsic info).
        for j in 0..m_checks {
            let nrw_j = P::nrw(j) as usize;
            for i in 0..nrw_j {
                let ibj = P::nm(j, i) as usize;
                let mut msg = zn[ibj];
                let mn_ibj = P::mn(ibj);
                for kk in 0..NCW {
                    if mn_ibj[kk] as usize == j {
                        msg -= tov[ibj * NCW + kk];
                    }
                }
                toc[j * max_row + i] = msg;
            }
        }

        match kind {
            BpKind::SumProduct => {
                // tanh half-message cache.
                for i in 0..m_checks {
                    let nrw_i = P::nrw(i) as usize;
                    for k_ in 0..nrw_i {
                        tanhtoc[i * max_row + k_] = (-toc[i * max_row + k_] / 2.0).tanh();
                    }
                }

                // Variable-to-check message update via 2·atanh(∏ tanh(L/2)).
                for j in 0..n {
                    let mn_j = P::mn(j);
                    for k_ in 0..NCW {
                        let ichk = mn_j[k_] as usize;
                        let nrw_ichk = P::nrw(ichk) as usize;
                        let mut tmn = 1.0f32;
                        for s in 0..nrw_ichk {
                            let bit = P::nm(ichk, s) as usize;
                            if bit != j {
                                tmn *= tanhtoc[ichk * max_row + s];
                            }
                        }
                        tov[j * NCW + k_] = 2.0 * platanh(-tmn);
                    }
                }
            }
            BpKind::NormalizedMinSum { .. } | BpKind::OffsetMinSum { .. } => {
                // Min-sum kernel — α / β are pulled below.
                //
                // Pass 1: per check node, compute (min1, min2, idx_min1,
                // sign_xor) over all edges. O(check_degree).
                for i in 0..m_checks {
                    let nrw_i = P::nrw(i) as usize;
                    let mut m1 = f32::INFINITY;
                    let mut m2 = f32::INFINITY;
                    let mut imin = 0_usize;
                    let mut sx = false;
                    for s in 0..nrw_i {
                        let v = toc[i * max_row + s];
                        if v < 0.0 {
                            sx = !sx;
                        }
                        let av = v.abs();
                        if av < m1 {
                            m2 = m1;
                            m1 = av;
                            imin = s;
                        } else if av < m2 {
                            m2 = av;
                        }
                    }
                    min1[i] = m1;
                    min2[i] = m2;
                    idx_min1[i] = imin as u32;
                    sign_xor[i] = sx;
                }

                // Pass 2: per outgoing edge (variable j → check ichk),
                // emit α·sign·min1 (or min2 if this edge owns min1).
                // Sign of this edge's own input is XOR'd out so we get
                // the extrinsic sign-product.
                let alpha_eff = match kind {
                    BpKind::NormalizedMinSum { alpha } => alpha,
                    _ => 1.0,
                };
                let beta = match kind {
                    BpKind::OffsetMinSum { beta } => beta,
                    _ => 0.0,
                };
                let is_offset = matches!(kind, BpKind::OffsetMinSum { .. });

                for j in 0..n {
                    let mn_j = P::mn(j);
                    for k_ in 0..NCW {
                        let ichk = mn_j[k_] as usize;
                        // Locate this edge's slot in the check's row to
                        // pull its own sign + magnitude out.
                        let nrw_ichk = P::nrw(ichk) as usize;
                        let mut my_slot = nrw_ichk; // sentinel
                        for s in 0..nrw_ichk {
                            if P::nm(ichk, s) as usize == j {
                                my_slot = s;
                                break;
                            }
                        }
                        // If for some reason the variable isn't found in
                        // the check (shouldn't happen with well-formed
                        // tables), fall back to min1 + full sign.
                        let my_v = if my_slot < nrw_ichk {
                            toc[ichk * max_row + my_slot]
                        } else {
                            0.0
                        };
                        let my_neg = my_v < 0.0;
                        // Match the SumProduct path's sign convention. WSJT-X
                        // computes `tmn = ∏ tanh(−toc/2)` then `tov = 2 ·
                        // atanh(−tmn)`; algebra gives output sign =
                        // `(−1)^nrw · sign(∏ toc[s≠j])`. The textbook NMS
                        // formula `α · sign(∏) · min` lacks the `(−1)^nrw`
                        // factor, so on odd-row-weight checks (nrw=7 in
                        // LDPC174_91, mixed in LDPC240_101) the unflipped
                        // NMS output disagrees with SP and BP diverges.
                        // XOR'ing in `nrw_ichk & 1` gives the correct sign.
                        let nrw_odd = (nrw_ichk & 1) != 0;
                        let extrinsic_sign_neg = sign_xor[ichk] ^ my_neg ^ nrw_odd;

                        let mag = if my_slot < nrw_ichk && my_slot as u32 == idx_min1[ichk] {
                            min2[ichk]
                        } else {
                            min1[ichk]
                        };

                        let scaled = if is_offset {
                            (mag - beta).max(0.0)
                        } else {
                            alpha_eff * mag
                        };
                        tov[j * NCW + k_] = if extrinsic_sign_neg { -scaled } else { scaled };
                    }
                }
            }
        }
    }

    None
}

/// WSJT-X `decode240_101`'s OSD fallback does not feed OSD the raw
/// channel LLR — it feeds the running sum `zsum = Σ_{i=0}^{n_iter} zn_i`
/// of the variable-node soft estimate across the *first few* BP
/// iterations (`lib/fst4/decode240_101.f90:51-63`; `zsave(:,iter)` for
/// `iter ∈ {1,2}` when WSJT-X's `maxosd=2`).
///
/// This runs only the `SumProduct` variable/check update (the kernel
/// WSJT-X itself uses for this step) for exactly `n_iter + 1` iterations
/// (`iter = 0..=n_iter`, matching the Fortran's own indexing) and returns
/// the accumulated `zsum` — no convergence check, no early return, no AP
/// mask (callers with an AP hint should not use this; see
/// [`crate::fec::Ldpc240_101`]'s `FecCodec::decode_soft` AP gate).
///
/// Diagnostic measurement (`fst4_diag_zsum_osd` in `tests/fst4_sweep.rs`,
/// issue #146) on FST4-120 near-threshold AWGN trials found this input
/// recovers real additional decodes beyond OSD-on-raw-channel-LLR (35 of
/// 106 OSD-relevant trials) at the cost of only 3 where raw succeeds and
/// this alone would not — feeding check-node-coupled values into OSD's
/// reliability-ordering step breaks OSD's usual independent-reliability
/// assumption in theory, but empirically it recovers more than it loses,
/// matching WSJT-X's own practice. [`crate::fec::Ldpc240_101`]'s
/// `FecCodec::decode_soft` therefore tries this as an *additional* OSD
/// attempt after OSD-on-raw-channel-LLR fails, never as a replacement —
/// that ordering is what makes the 3-trial loss moot (raw already had
/// first try).
pub fn bp_llr_zsum<P: LdpcParams>(llr: &[f32], n_iter: u32) -> Vec<f32> {
    let n = P::N;
    let m_checks = P::M;
    let max_row = P::MAX_ROW;

    let mut tov = vec![0f32; n * NCW];
    let mut toc = vec![0f32; m_checks * max_row];
    let mut tanhtoc = vec![0f32; m_checks * max_row];
    let mut zn = vec![0f32; n];
    let mut zsum = vec![0f32; n];

    for j in 0..m_checks {
        let nrw_j = P::nrw(j) as usize;
        for i in 0..nrw_j {
            let bit = P::nm(j, i) as usize;
            toc[j * max_row + i] = llr[bit];
        }
    }

    for _iter in 0..=n_iter {
        for i in 0..n {
            let mut sum = 0.0f32;
            for k_ in 0..NCW {
                sum += tov[i * NCW + k_];
            }
            zn[i] = llr[i] + sum;
        }
        for i in 0..n {
            zsum[i] += zn[i];
        }

        // Check-to-variable message update (extrinsic info).
        for j in 0..m_checks {
            let nrw_j = P::nrw(j) as usize;
            for i in 0..nrw_j {
                let ibj = P::nm(j, i) as usize;
                let mut msg = zn[ibj];
                let mn_ibj = P::mn(ibj);
                for kk in 0..NCW {
                    if mn_ibj[kk] as usize == j {
                        msg -= tov[ibj * NCW + kk];
                    }
                }
                toc[j * max_row + i] = msg;
            }
        }

        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            for k_ in 0..nrw_i {
                tanhtoc[i * max_row + k_] = (-toc[i * max_row + k_] / 2.0).tanh();
            }
        }

        for j in 0..n {
            let mn_j = P::mn(j);
            for k_ in 0..NCW {
                let ichk = mn_j[k_] as usize;
                let nrw_ichk = P::nrw(ichk) as usize;
                let mut tmn = 1.0f32;
                for s in 0..nrw_ichk {
                    let bit = P::nm(ichk, s) as usize;
                    if bit != j {
                        tmn *= tanhtoc[ichk * max_row + s];
                    }
                }
                tov[j * NCW + k_] = 2.0 * platanh(-tmn);
            }
        }
    }

    zsum
}

/// Backward-compatible LDPC(174,91) BP decode — pins
/// [`bp_decode_generic`] to [`Ldpc174_91Params`]. Used by FT8's
/// bespoke decode loop (which still consumes the shared LDPC
/// implementation through `super::ft8::ldpc`'s re-export façade).
///
/// `llr[i]` follows the convention: positive = bit likely 1, negative
/// = bit likely 0. `ap_mask[i] = true` means the bit's LLR is
/// AP-locked and not updated by BP.
pub fn bp_decode(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic::<Ldpc174_91Params>(llr.as_slice(), ap_slice, max_iter, verify)
}

/// LDPC(174,91) BP with selectable check-node kernel — same as
/// [`bp_decode`] but accepts a [`BpKind`] for the embedded /
/// FPU-poor min-sum paths.
pub fn bp_decode_kind(
    llr: &[f32; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    kind: BpKind,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic_kind::<Ldpc174_91Params>(llr.as_slice(), ap_slice, max_iter, verify, kind)
}

// ──────────────────────────────────────────────────────────────────────────
// Generic NMS BP — works on any [`LlrScalar`] (`f32` for host / FPU
// targets, `Q11i16` for FPU-less / consistency-focused embedded).
// ──────────────────────────────────────────────────────────────────────────

use crate::core::scalar::LlrScalar;

/// Convert an `f32` LLR to Q11 i16 with saturation.
///
/// Thin wrapper around [`crate::core::scalar::Q11i16::from_f32`] —
/// kept for source compatibility with callers from before the
/// generic refactor.
#[inline]
pub fn llr_f32_to_q11(x: f32) -> i16 {
    use crate::core::scalar::Q11i16;
    Q11i16::from_f32(x).0
}

/// Generic Normalized-Min-Sum Belief-Propagation decode.
///
/// Architectural counterpart of [`bp_decode_generic_kind`]'s NMS
/// branch, generic over the LLR scalar `T`. SumProduct and
/// OffsetMinSum stay f32-only (`tanh` / `atanh` aren't worth
/// quantising); embedded fixed-point callers only ever want NMS
/// anyway.
///
/// Behaviour matches the f32 NMS branch byte-for-byte at the bit
/// level (same convergence path on every AWGN seed I've tested),
/// including the `nrw_odd` extrinsic-sign correction that aligns
/// with WSJT-X's `tanh ∘ atanh` SumProduct convention.
/// Reusable working memory for [`bp_decode_generic_nms_with_scratch`].
///
/// Owns the seven `Vec` buffers that the NMS hot loop used to allocate
/// per call (~12 KB for `T = Q11i16`, ~24 KB for `T = f32` on FT8
/// LDPC(174,91); ~17 KB / ~33 KB on FST4 LDPC(240,101)). One instance
/// amortises N decode calls — `process_candidates_with` holds a single
/// pool across all candidates so all 5 BP calls × ~15 survivors / slot
/// share the same `Vec` backing storage instead of hammering
/// `tlsf_malloc` (the dominant non-DFT cost on Core2 stage 3).
///
/// Generic over the same `P: LdpcParams` + `T: LlrScalar` pair as the
/// decoder itself, so FT8 LDPC(174,91) and FST4/uvpacket LDPC(240,101)
/// each get the right capacity. Capacities are sized once in [`new`],
/// and the internal pre-decode reset is run automatically by every
/// [`bp_decode_generic_nms_with_scratch`] call so callers only need
/// to construct the pool once and reuse it.
///
/// `no_std + alloc` clean — only uses `Vec` + `vec!`.
///
/// [`new`]: BpScratch::new
pub struct BpScratch<P: LdpcParams, T: LlrScalar> {
    tov: Vec<T>,
    toc: Vec<T>,
    min1: Vec<T>,
    min2: Vec<T>,
    idx_min1: Vec<u32>,
    sign_xor: Vec<bool>,
    zn: Vec<T::Wide>,
    cw: Vec<u8>,
    _p: core::marker::PhantomData<P>,
}

impl<P: LdpcParams, T: LlrScalar> BpScratch<P, T> {
    /// Allocate the seven scratch buffers at the right capacities for
    /// `P`. Single instance can be reused across many BP calls.
    pub fn new() -> Self {
        let n = P::N;
        let m_checks = P::M;
        let max_row = P::MAX_ROW;
        Self {
            tov: vec![T::ZERO; n * NCW],
            toc: vec![T::ZERO; m_checks * max_row],
            min1: vec![T::POS_INF_LIKE; m_checks],
            min2: vec![T::POS_INF_LIKE; m_checks],
            idx_min1: vec![0u32; m_checks],
            sign_xor: vec![false; m_checks],
            zn: vec![T::wide_zero(); n],
            cw: vec![0u8; n],
            _p: core::marker::PhantomData,
        }
    }

    /// Re-establish the pre-loop state of a fresh allocation. Matches
    /// the `vec![…; cap]` initialisers of the pre-pool implementation
    /// byte-for-byte so the inner algorithm's behaviour is unchanged.
    /// `zn` / `cw` are written unconditionally on every iter so they
    /// don't need resetting; everything else does.
    #[inline]
    fn reset(&mut self) {
        for v in self.tov.iter_mut() {
            *v = T::ZERO;
        }
        for v in self.toc.iter_mut() {
            *v = T::ZERO;
        }
        for v in self.min1.iter_mut() {
            *v = T::POS_INF_LIKE;
        }
        for v in self.min2.iter_mut() {
            *v = T::POS_INF_LIKE;
        }
        for v in self.idx_min1.iter_mut() {
            *v = 0;
        }
        for v in self.sign_xor.iter_mut() {
            *v = false;
        }
    }
}

impl<P: LdpcParams, T: LlrScalar> Default for BpScratch<P, T> {
    fn default() -> Self {
        Self::new()
    }
}

/// Allocate-per-call shim over [`bp_decode_generic_nms_with_scratch`]
/// for callers that don't (yet) thread a [`BpScratch`] through. New
/// code on the embedded hot path should prefer the `_with_scratch`
/// variant — see the FT8 stage-3 driver in
/// `ft8::decode_block::process_candidates_with`.
pub fn bp_decode_generic_nms<P: LdpcParams, T: LlrScalar>(
    llr: &[T],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    alpha: f32,
) -> Option<BpResult> {
    let mut scratch = BpScratch::<P, T>::new();
    bp_decode_generic_nms_with_scratch::<P, T>(&mut scratch, llr, ap_mask, max_iter, verify, alpha)
}

/// [`bp_decode_generic_nms`] with caller-provided scratch — eliminates
/// the per-call ~12 KB allocation churn on the BP staircase hot path.
pub fn bp_decode_generic_nms_with_scratch<P: LdpcParams, T: LlrScalar>(
    scratch: &mut BpScratch<P, T>,
    llr: &[T],
    ap_mask: Option<&[bool]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    alpha: f32,
) -> Option<BpResult> {
    debug_assert_eq!(llr.len(), P::N);
    if let Some(m) = ap_mask {
        debug_assert_eq!(m.len(), P::N);
    }

    let n = P::N;
    let m_checks = P::M;
    let k = P::K;
    let max_row = P::MAX_ROW;

    scratch.reset();
    let BpScratch {
        tov,
        toc,
        min1,
        min2,
        idx_min1,
        sign_xor,
        zn,
        cw,
        ..
    } = scratch;

    // Initial messages: each check edge carries the bit's raw LLR.
    for j in 0..m_checks {
        let nrw_j = P::nrw(j) as usize;
        for i in 0..nrw_j {
            let bit = P::nm(j, i) as usize;
            toc[j * max_row + i] = llr[bit];
        }
    }

    let mut ncnt = 0u32;
    let mut nclast = 0u32;

    for iter in 0..=max_iter {
        // Variable-node belief sum. The wide accumulator avoids
        // overflow when summing 1 LLR + NCW=3 tov entries.
        for i in 0..n {
            let ap = ap_mask.is_some_and(|mm| mm[i]);
            if !ap {
                let mut sum = llr[i].to_wide();
                for k_ in 0..NCW {
                    sum = T::wide_add(sum, tov[i * NCW + k_].to_wide());
                }
                zn[i] = sum;
            } else {
                zn[i] = llr[i].to_wide();
            }
        }

        // Hard decisions from sign of belief.
        for i in 0..n {
            cw[i] = if T::wide_is_positive(zn[i]) { 1 } else { 0 };
        }

        // Parity check.
        let mut ncheck = 0u32;
        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            let mut parity = 0u8;
            for s in 0..nrw_i {
                parity ^= cw[P::nm(i, s) as usize];
            }
            if parity != 0 {
                ncheck += 1;
            }
        }

        if ncheck == 0 {
            let mut decoded = vec![0u8; k];
            decoded.copy_from_slice(&cw[..k]);
            let accept = match verify {
                Some(f) => f(&decoded),
                None => true,
            };
            if accept {
                let mut hard_errors = 0u32;
                for i in 0..n {
                    // Hard error iff hard decision (cw) disagrees with
                    // the LLR's sign. Mirrors the f32 path's
                    // `(cw[i] == 1) != (llr[i] > 0)`.
                    let llr_says_one = !llr[i].is_negative();
                    if (cw[i] == 1) != llr_says_one {
                        hard_errors += 1;
                    }
                }
                let mut message77 = [0u8; 77];
                let message_len = decoded.len().min(message77.len());
                message77[..message_len].copy_from_slice(&decoded[..message_len]);
                // Codeword is small (174 / 240 bytes); clone instead
                // of moving so the scratch's `cw` Vec stays in the pool
                // for the next call.
                return Some(BpResult {
                    message77,
                    info: decoded,
                    codeword: cw.clone(),
                    hard_errors,
                    iterations: iter,
                });
            }
        }

        // Stall detector — same heuristic as the f32 SumProduct path.
        if iter > 0 {
            if ncheck < nclast {
                ncnt = 0;
            } else {
                ncnt += 1;
            }
            if ncnt >= 5 && iter >= 10 && ncheck > 15 {
                return None;
            }
        }
        nclast = ncheck;

        // Variable-to-check messages: subtract own contribution.
        for j in 0..m_checks {
            let nrw_j = P::nrw(j) as usize;
            for i in 0..nrw_j {
                let ibj = P::nm(j, i) as usize;
                let mut msg = zn[ibj];
                let mn_ibj = P::mn(ibj);
                for kk in 0..NCW {
                    if mn_ibj[kk] as usize == j {
                        msg = T::wide_sub(msg, tov[ibj * NCW + kk].to_wide());
                    }
                }
                toc[j * max_row + i] = T::from_wide_sat(msg);
            }
        }

        // Min-sum kernel: per check node compute the two smallest |toc|.
        for i in 0..m_checks {
            let nrw_i = P::nrw(i) as usize;
            let mut m1 = T::POS_INF_LIKE;
            let mut m2 = T::POS_INF_LIKE;
            let mut imin = 0_usize;
            let mut sx = false;
            for s in 0..nrw_i {
                let v = toc[i * max_row + s];
                if v.is_negative() {
                    sx = !sx;
                }
                let av = v.abs_sat();
                if av.lt_total(m1) {
                    m2 = m1;
                    m1 = av;
                    imin = s;
                } else if av.lt_total(m2) {
                    m2 = av;
                }
            }
            min1[i] = m1;
            min2[i] = m2;
            idx_min1[i] = imin as u32;
            sign_xor[i] = sx;
        }

        // Per-edge check-to-variable update with α scaling (NMS).
        for j in 0..n {
            let mn_j = P::mn(j);
            for k_ in 0..NCW {
                let ichk = mn_j[k_] as usize;
                let nrw_ichk = P::nrw(ichk) as usize;
                let mut my_slot = nrw_ichk;
                for s in 0..nrw_ichk {
                    if P::nm(ichk, s) as usize == j {
                        my_slot = s;
                        break;
                    }
                }
                let my_v = if my_slot < nrw_ichk {
                    toc[ichk * max_row + my_slot]
                } else {
                    T::ZERO
                };
                let my_neg = my_v.is_negative();
                let nrw_odd = (nrw_ichk & 1) != 0;
                let extrinsic_sign_neg = sign_xor[ichk] ^ my_neg ^ nrw_odd;

                let mag = if my_slot < nrw_ichk && my_slot as u32 == idx_min1[ichk] {
                    min2[ichk]
                } else {
                    min1[ichk]
                };

                let scaled = mag.mul_alpha(alpha);
                tov[j * NCW + k_] = if extrinsic_sign_neg {
                    scaled.neg_sat()
                } else {
                    scaled
                };
            }
        }
    }

    None
}

/// LDPC(174,91) BP NMS — generic over LLR scalar.
pub fn bp_decode_nms<T: LlrScalar>(
    llr: &[T; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    alpha: f32,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic_nms::<Ldpc174_91Params, T>(llr.as_slice(), ap_slice, max_iter, verify, alpha)
}

/// LDPC(174,91) BP NMS with caller-provided scratch — pool-aware
/// variant of [`bp_decode_nms`]. Lets the FT8 stage-3 driver instantiate
/// one [`BpScratch`] per slot and reuse it across all 5 BP calls × ~15
/// surviving candidates, saving ~900 KB of `tlsf_malloc` traffic per
/// slot on Core2.
pub fn bp_decode_nms_with_scratch<T: LlrScalar>(
    scratch: &mut BpScratch<Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    alpha: f32,
) -> Option<BpResult> {
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    bp_decode_generic_nms_with_scratch::<Ldpc174_91Params, T>(
        scratch,
        llr.as_slice(),
        ap_slice,
        max_iter,
        verify,
        alpha,
    )
}

/// Backward-compatible Q11 alias — keeps existing callers compiling.
/// New code should prefer the generic [`bp_decode_nms`].
pub fn bp_decode_nms_q11(
    llr: &[i16; LDPC_N],
    ap_mask: Option<&[bool; LDPC_N]>,
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
    alpha: f32,
) -> Option<BpResult> {
    use crate::core::scalar::Q11i16;
    let ap_slice: Option<&[bool]> = ap_mask.map(|a| a.as_slice());
    // SAFETY: `Q11i16` is `#[repr(transparent)]`-equivalent — wraps a
    // single `i16` in a tuple struct. A `&[i16; N]` aliases a
    // `&[Q11i16; N]` byte-for-byte. (Conservative: copy via map.)
    let llr_q: alloc::vec::Vec<Q11i16> = llr.iter().map(|&x| Q11i16(x)).collect();
    bp_decode_generic_nms::<Ldpc174_91Params, Q11i16>(&llr_q, ap_slice, max_iter, verify, alpha)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_perfect_llr_all_zeros() {
        let llr = [10.0f32; 174];
        let _result = bp_decode(&llr, None, 30, None);
    }

    #[test]
    fn crc14_known_vector() {
        assert_eq!(crc14(&[0u8; 12]), 0);
    }

    #[test]
    fn platanh_matches_wsjtx_piecewise_curve() {
        assert!((platanh(0.664) - 0.8).abs() < 1.0e-6);
        assert!((platanh(0.9217) - ((0.9217 - 0.4064) / 0.322)).abs() < 1.0e-6);
        assert!((platanh(0.9951) - ((0.9951 - 0.8378) / 0.0524)).abs() < 1.0e-6);
        assert!((platanh(0.9998) - ((0.9998 - 0.9914) / 0.0012)).abs() < 1.0e-5);
        assert_eq!(platanh(1.0), 7.0);
        assert_eq!(platanh(-1.0), -7.0);
    }
}
