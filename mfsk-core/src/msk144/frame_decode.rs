//! MSK144 per-frame decode: sync-quality gate, LLR scaling, LDPC
//! decode, and message-type plausibility filter.
//!
//! Ported from WSJT-X `msk144decodeframe.f90:69-111`. The carrier-
//! phase estimate + OQPSK matched filter
//! (`msk144decodeframe.f90:51-67`) already live in
//! [`crate::engine::dsp::msk::matched_filter_softbits`] (Phase 2); this
//! module picks up from its 144 soft symbols.

use alloc::string::String;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::dsp::msk::bipolar_sync_word;
use crate::engine::{DecodeContext, FecCodec, FecOpts};
use crate::fec::Ldpc128_90;
use crate::fec::ldpc_128_90::check_crc13;
use crate::msg::CallsignHashTable;
use crate::msg::wsjt77;

/// Result of a successful frame decode.
#[derive(Clone, Debug)]
pub struct FrameDecodeResult {
    /// Human-readable decoded message (from `unpack77`/`unpack77_with_hash`).
    pub message: String,
    /// The 77-bit decoded message payload (pre-`unpack77`), in case a
    /// caller wants the raw bits (e.g. for a hash-table `save` pass).
    pub info77: [u8; 77],
    /// Count of codeword bits whose hard decision disagrees with the
    /// LLR sign — WSJT-X's soft confidence gate (`nharderror`).
    pub hard_errors: u32,
}

/// Count sync-word hard-decision mismatches at the 8-bit sync word
/// starting at soft-symbol offset `offset` (0 or 56 — the frame's two
/// embedded copies). Matches `msk144decodeframe.f90:71-78`: hard-slice
/// each soft bit, compare its bipolar sign against the (bipolar) sync
/// word, and count disagreements.
fn count_bad_sync(softbits: &[f32; 144], sync_bipolar: &[i8; 8], offset: usize) -> i32 {
    let mut sum = 0i32;
    for k in 0..8 {
        let hard_bipolar = if softbits[offset + k] >= 0.0 { 1 } else { -1 };
        sum += hard_bipolar * sync_bipolar[k] as i32;
    }
    (8 - sum) / 2
}

/// Extract the `n3`/`i3` message-type discriminator bits from a
/// decoded 77-bit payload: two 3-bit fields at bits 71..74 and
/// 74..77 (0-based, MSB-first). Matches
/// `msk144decodeframe.f90:101-102`: `read(c77(72:77),'(2b3)') n3,i3`.
fn extract_n3_i3(info77: &[u8; 77]) -> (u8, u8) {
    let n3 = (info77[71] << 2) | (info77[72] << 1) | info77[73];
    let i3 = (info77[74] << 2) | (info77[75] << 1) | info77[76];
    (n3, i3)
}

/// Reject certain implausible `(n3, i3)` combinations before even
/// attempting `unpack77` — a cheap plausibility filter WSJT-X applies
/// on top of CRC + the BP hard-error gate. Matches
/// `msk144decodeframe.f90:103`.
fn n3_i3_plausible(n3: u8, i3: u8) -> bool {
    !((i3 == 0 && (n3 == 1 || n3 == 3 || n3 == 4 || n3 > 5)) || i3 == 3 || i3 > 5)
}

/// Decode one MSK144 frame's worth of matched-filter soft symbols
/// into a message. `softbits` is the output of
/// [`crate::engine::dsp::msk::matched_filter_softbits`] for an
/// already-aligned (correct symbol timing, derotated) frame.
///
/// Ported from `msk144decodeframe.f90:69-111`: sync hard-error gate
/// (reject before running the decoder at all if `nbadsync > 4`),
/// zero-mean/unit-variance softbit normalization, LLR scaling by the
/// fixed `sigma = 0.60` calibration constant, LDPC(128,90) decode with
/// CRC-13 as the parity-convergence verifier, the `hard_errors < 18`
/// soft-confidence gate on top of CRC, the `n3`/`i3` plausibility
/// filter, and finally `unpack77` (hash-table-aware if `ctx` carries
/// one, matching the same dispatch `crate::msg::Wsjt77Message::unpack`
/// uses for FT8/FT4/FST4).
pub fn decode_frame(softbits: &[f32; 144], ctx: &DecodeContext) -> Option<FrameDecodeResult> {
    let sync_bipolar = bipolar_sync_word();
    let nbadsync =
        count_bad_sync(softbits, &sync_bipolar, 0) + count_bad_sync(softbits, &sync_bipolar, 56);
    if nbadsync > 4 {
        return None;
    }

    // Normalize: zero-mean, unit-variance (population statistics —
    // `msk144decodeframe.f90:85-88`).
    let sav: f32 = softbits.iter().sum::<f32>() / 144.0;
    let s2av: f32 = softbits.iter().map(|&s| s * s).sum::<f32>() / 144.0;
    let ssig = (s2av - sav * sav).sqrt();
    let normalized: [f32; 144] = {
        let mut n = [0.0f32; 144];
        for i in 0..144 {
            n[i] = softbits[i] / ssig;
        }
        n
    };

    // sigma=0.60: a hand-tuned WSJT-X calibration constant
    // (`msk144decodeframe.f90:90`), carried over verbatim rather than
    // re-derived.
    const SIGMA: f32 = 0.60;
    let mut llr = [0.0f32; 128];
    llr[0..48].copy_from_slice(&normalized[8..56]);
    llr[48..128].copy_from_slice(&normalized[64..144]);
    for v in llr.iter_mut() {
        *v = 2.0 * *v / (SIGMA * SIGMA);
    }

    let opts = FecOpts {
        bp_max_iter: 10, // msk144decodeframe.f90:95 `max_iterations=10`
        verify_info: Some(check_crc13),
        ..Default::default()
    };
    let result = Ldpc128_90.decode_soft(&llr, &opts)?;
    if result.hard_errors >= 18 {
        return None;
    }

    let mut info77 = [0u8; 77];
    info77.copy_from_slice(&result.info[..77]);

    let (n3, i3) = extract_n3_i3(&info77);
    if !n3_i3_plausible(n3, i3) {
        return None;
    }

    let message = if let Some(any) = ctx.callsign_hash_table.as_ref()
        && let Some(ht) = any.downcast_ref::<CallsignHashTable>()
    {
        wsjt77::unpack77_with_hash(&info77, ht)
    } else {
        wsjt77::unpack77(&info77)
    }?;

    Some(FrameDecodeResult {
        message,
        info77,
        hard_errors: result.hard_errors,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::dsp::msk::{build_bitseq, matched_filter_softbits, synth_frame};

    fn build_info_with_crc(pattern: impl Fn(usize) -> u8) -> [u8; 90] {
        let mut info = [0u8; 90];
        for i in 0..77 {
            info[i] = pattern(i);
        }
        let mut bytes = [0u8; 12];
        for (i, &b) in info[..77].iter().enumerate() {
            let byte_idx = i / 8;
            let bit_pos = 7 - (i % 8);
            bytes[byte_idx] |= (b & 1) << bit_pos;
        }
        let crc = crate::fec::ldpc_128_90::crc13(&bytes);
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }
        info
    }

    /// End-to-end with a real standard-message payload (not just an
    /// arbitrary bit pattern), through the full Phase 2 + Phase 4
    /// chain: pack77 -> encode -> synth -> matched filter ->
    /// decode_frame -> recovered message string.
    #[test]
    fn decodes_standard_message_end_to_end() {
        let msg77 = wsjt77::pack77("K1ABC", "W9XYZ", "EN37").expect("valid standard message");
        let mut info = [0u8; 90];
        info[..77].copy_from_slice(&msg77);
        let mut bytes = [0u8; 12];
        for (i, &b) in info[..77].iter().enumerate() {
            let byte_idx = i / 8;
            let bit_pos = 7 - (i % 8);
            bytes[byte_idx] |= (b & 1) << bit_pos;
        }
        let crc = crate::fec::ldpc_128_90::crc13(&bytes);
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }

        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);
        let frame = synth_frame(&bitseq);
        let softbits = matched_filter_softbits(&frame);

        let ctx = DecodeContext::default();
        let result = decode_frame(&softbits, &ctx).expect("clean frame decodes");
        assert!(result.message.contains("K1ABC"));
        assert!(result.message.contains("W9XYZ"));
        assert!(result.message.contains("EN37"));
        assert!(result.hard_errors < 18);
    }

    /// A frame with a corrupted sync word (well past the `nbadsync>4`
    /// tolerance) must be rejected before the decoder even runs.
    #[test]
    fn rejects_frame_with_bad_sync() {
        let info = build_info_with_crc(|i| ((i * 7 + 3) & 1) as u8);
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);
        let frame = synth_frame(&bitseq);
        let mut softbits = matched_filter_softbits(&frame);

        // Flip all 8 bits of the first sync word's sign.
        for s in softbits.iter_mut().take(8) {
            *s = -*s;
        }

        let ctx = DecodeContext::default();
        assert!(decode_frame(&softbits, &ctx).is_none());
    }

    /// `n3`/`i3` plausibility filter: direct truth-table check against
    /// the documented boolean expression, independent of the full
    /// decode chain.
    #[test]
    fn n3_i3_filter_matches_truth_table() {
        for n3 in 0..8u8 {
            for i3 in 0..8u8 {
                let expected =
                    !((i3 == 0 && (n3 == 1 || n3 == 3 || n3 == 4 || n3 > 5)) || i3 == 3 || i3 > 5);
                assert_eq!(n3_i3_plausible(n3, i3), expected, "n3={n3} i3={i3}");
            }
        }
    }
}
