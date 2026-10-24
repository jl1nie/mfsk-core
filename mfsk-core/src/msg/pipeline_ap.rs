//! A-priori hypotheses for the WSJT 77-bit family: which configurations
//! to try, and what bits each one locks.
//!
//! **This module used to own a decoder as well**, and that was the
//! problem. `decode_sniper_ap` ran its own per-candidate ladder —
//! refine, spectra, a plain-BP staircase, then AP passes with OSD at
//! depth 2 — parallel to the one `engine::pipeline` runs and shallower
//! than it, with no depth-3/4 escalation and no Top-K rescue. Because
//! AP lived there, reaching AP meant leaving the real ladder: routing a
//! wide-band FT4 decode through it returned 4 decodes where the
//! wide-band engine returns 11 on the WSJT-X golden.
//!
//! AP is now a rung at the end of the wide-band ladder
//! (`engine::pipeline::process_candidate_basic_impl`), which is where
//! WSJT-X has it too — `ft4_decode.f90`'s `npasses = 3 + nappasses(…)`
//! continues the same pass loop rather than starting a second decoder.
//! What remains here is the part that was always this module's own: the
//! hypothesis set, and the mapping from an `ApHint` to locked bits.
//!
//! Typical threshold improvement is 2–4 dB when both call1 and call2 are
//! known (CQ + DX scenario) and can exceed that when a specific response
//! token (RRR / RR73 / 73) is also locked. The blind CQ hypothesis,
//! which needs nothing, is worth about 1.1 dB on FT4's AWGN sweep
//! (`docs/historical/FT4_BENCHMARK_LOG.md` §48).

use alloc::vec::Vec;

use crate::engine::{FecCodec, Protocol};

use super::ap::{ApHint, WsjtApCompatible};

/// Build one AP configuration: derive the mask/values bit vectors from a
/// hint for this protocol's codeword length. Convenience for callers that
/// want to try several hint shapes (full lock, partial lock, …).
///
/// Bound on [`WsjtApCompatible`] keeps callers honest: the hint encodes
/// callsign / grid / report at fixed Wsjt77 bit positions and is meaningless
/// for protocols whose info layout differs (e.g. byte-oriented codecs).
// FT4 and FST4 build their AP bits here; FT8 only uses `ap_passes`.
#[cfg_attr(not(any(feature = "ft4", feature = "fst4")), allow(dead_code))]
pub(crate) fn ap_bits_for<P: Protocol>(hint: &ApHint) -> (Vec<u8>, Vec<u8>)
where
    P::Msg: WsjtApCompatible,
{
    let (mask, mut values) = hint.build_bits(P::Fec::N);
    // **The hint describes the message; the decoder works on the
    // codeword.** FT4 and FST4 XOR the 77-bit message with their own
    // RVEC before CRC and FEC encode (`ModulationParams::INFO_SCRAMBLE_RVEC`),
    // so the info bits inside the codeword are the *scrambled* message.
    // A value locked in message space is therefore the wrong value
    // wherever the RVEC bit is 1 — which is about half of them, i.e. AP
    // was locking roughly half its bits to the opposite of the truth.
    //
    // Measured on an FT4 AWGN sweep at 12 trials per point, hinting a
    // CQ the decoder is trying to find: 2/12 → 10/12 at −18 dB and
    // 0/12 → 5/12 at −19 dB once the values are scrambled. Before this,
    // AP-hinted decoding on FT4 was not merely weak, it was actively
    // worse than plain decoding.
    //
    // The mask is untouched: an XOR moves no positions. FT8 has no
    // RVEC and is unaffected.
    if let Some(rvec) = <P as crate::engine::ModulationParams>::INFO_SCRAMBLE_RVEC {
        let n = rvec.len().min(values.len());
        for (b, &r) in values[..n].iter_mut().zip(rvec.iter()) {
            *b = (*b ^ r) & 1;
        }
    }
    (mask, values)
}

/// Enumerate the multi-pass AP configurations WSJT-X cycles through in
/// sniper mode — the `u8` is a pass-id tag for diagnostics.
///
/// - 9/10/11: full 77-bit lock with `RRR` / `RR73` / `73` (QSO in progress).
/// - 7:       CQ + DX call (expected "CQ DXCALL GRID").
/// - 8:       my-call + DX call (directed message).
/// - 6:       DX call only (partial lock, fallback).
///
/// **FT8 analog for pass 7**: `ft8::decode_block::process_candidates`'s
/// own blind-CQ `Pass 12` (gated by `BLIND_CQ_MIN_NSYNC`) is FT8's
/// bespoke equivalent, independently implemented and tuned — review
/// both when adjusting either (issue #285, split from #192). Unlike
/// the OSD-escalation gate (`osd_escalation_gates`, which FT8's own
/// `Q_NDEEP3_THRESHOLD` was once paired with), this pass has no single
/// paired numeric threshold to ratchet-test against — `ap_passes` doesn't gate pass 7
/// on an nsync value of its own, so there's nothing here for
/// `BLIND_CQ_MIN_NSYNC` to be asserted equal to. Prose cross-reference
/// only; a future retune of either still needs a human to remember
/// this comment.
pub(crate) fn ap_passes(base: &ApHint) -> Vec<(ApHint, u8)> {
    let mut passes = Vec::new();
    if base.call1.is_some() && base.call2.is_some() {
        for (rpt, pid) in [("RRR", 9u8), ("RR73", 10), ("73", 11)] {
            passes.push((base.clone().with_report(rpt), pid));
        }
    }
    if base.call2.is_some() && base.call1.is_none() {
        passes.push((base.clone().with_call1("CQ"), 7));
    }
    if base.call1.is_some() && base.call2.is_some() {
        passes.push((base.clone(), 8));
    }
    passes.push((base.clone(), 6));
    passes.retain(|(_, pid)| base.allow.allows(*pid));
    passes
}

/// The RR73 AP lock is what `pack77` transmits, in every protocol that uses a 77-bit
/// `ApHint` (WSJT-X v3.3.0-beta4: `PACK77_TYPE1_RR73_TAIL` in ft8b.f90 / q65_ap.f90, and the
/// iaptype-6 `apbits` of fst4_decode.f90 / ft4_decode.f90, a packed `MyCall HisCall RR73`).
/// A lock on the grid 32373 would fail this.
#[cfg(test)]
mod rr73_lock_tests {
    use super::*;
    use crate::msg::wsjt77::pack77;

    fn assert_locks_what_pack77_sends<P: Protocol>()
    where
        P::Msg: WsjtApCompatible,
    {
        let hint = ApHint::new()
            .with_call1("K1ABC")
            .with_call2("W9XYZ")
            .with_report("RR73");
        let sent = pack77("K1ABC", "W9XYZ", "RR73").unwrap();
        let (mask, mut values) = ap_bits_for::<P>(&hint);
        // Back to message space: ap_bits_for XORs the protocol's RVEC into the values.
        if let Some(rvec) = <P as crate::engine::ModulationParams>::INFO_SCRAMBLE_RVEC {
            for (b, &r) in values.iter_mut().zip(rvec.iter()).take(77) {
                *b ^= r & 1;
            }
        }
        for i in 58..77 {
            assert_eq!(mask[i], 1, "bit {i} locked");
            assert_eq!(values[i], sent[i], "bit {i} of the RR73 lock vs pack77");
        }
    }

    #[test]
    #[cfg(feature = "ft8")]
    fn ft8_locks_what_pack77_sends() {
        assert_locks_what_pack77_sends::<crate::ft8::Ft8>();
        // FT8's own AP rung calls `build_bits` directly.
        let hint = ApHint::new()
            .with_call1("K1ABC")
            .with_call2("W9XYZ")
            .with_report("RR73");
        let (mask, v) = hint.build_bits(174);
        let sent = pack77("K1ABC", "W9XYZ", "RR73").unwrap();
        assert!((58..77).all(|i| mask[i] == 1 && v[i] == sent[i]));
    }

    #[test]
    #[cfg(feature = "ft4")]
    fn ft4_locks_what_pack77_sends() {
        assert_locks_what_pack77_sends::<crate::ft4::Ft4>();
    }

    #[test]
    #[cfg(feature = "fst4")]
    fn fst4_locks_what_pack77_sends() {
        assert_locks_what_pack77_sends::<crate::fst4::Fst4s60>();
    }
}
