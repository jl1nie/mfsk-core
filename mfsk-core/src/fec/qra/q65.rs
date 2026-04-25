//! Q65 application wrapper around the generic QRA codec.
//!
//! Ports the CRC + puncturing layer from `lib/qra/q65/q65.c` (Nico
//! Palermo IV3NWV / Joe Taylor K1JT). Q65 sits on top of the
//! [`super::QraCode`] framework with three additions:
//!
//! 1. A **CRC-12** computed over the user message and prepended (as
//!    two GF(64) symbols) to the K=15 information vector before QRA
//!    encoding.
//! 2. **Puncturing** of those two CRC symbols from the 65-symbol
//!    codeword, so only **63 channel symbols** are transmitted per
//!    Q65 frame.
//! 3. A CRC verification step on the receiver after MAP decode —
//!    used as the false-decode rejection mechanism.
//!
//! The wrapper exposes [`Q65Codec`] holding the underlying code +
//! reusable scratch buffers; instantiate it once with the
//! [`qra15_65_64_irr_e23`](crate::fec::qra15_65_64::QRA15_65_64_IRR_E23)
//! code and call [`Q65Codec::encode`] / [`Q65Codec::decode`] on each
//! message.
//!
//! The other Q65 features in the C source (a-priori / list decoding,
//! fast-fading channel metrics, Es/No estimation) are not yet ported.
//! [`Q65Codec::decode`] handles plain AWGN with the
//! [`QraCode::mfsk_bessel_metric`] front-end, which is the core path
//! used by every Q65 decode regardless of subsequent AP refinement.

use super::{DecoderScratch, ExtrinsicResult, QraCode, QraCodeType, pdmath};

/// CRC-6 generator polynomial: x^6 + x + 1 — matches the C reference.
const CRC6_GEN_POL: i32 = 0x30;

/// CRC-12 generator polynomial: x^12 + x^11 + x^3 + x^2 + x + 1.
const CRC12_GEN_POL: i32 = 0xF01;

/// Why a Q65 decode call did not yield a clean message.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Q65DecodeError {
    /// The QRA belief-propagation loop did not converge within
    /// `max_iters`. The caller may choose to retry with a larger
    /// budget or with AP information.
    NotConverged,
    /// BP converged but the MAP-decoded CRC did not match the
    /// recomputed CRC over the same payload — almost always the sign
    /// of a wrong decode that the CRC rescued the application from.
    CrcMismatch,
}

/// Stateful Q65 codec. Construct once with [`Q65Codec::new`] and reuse
/// across messages; the internal scratch buffers (BP messages,
/// intrinsic / extrinsic rows, working symbol arrays) are sized for
/// the underlying QRA code and recycled on every call.
pub struct Q65Codec {
    code: &'static QraCode,
    scratch: DecoderScratch,
    /// Working K-symbol info buffer (user message + CRC).
    px: Vec<i32>,
    /// Working N-symbol codeword buffer.
    py: Vec<i32>,
    /// Depunctured intrinsic distributions, length M * N.
    ix: Vec<f32>,
    /// Extrinsic distributions, length M * N.
    ex: Vec<f32>,
}

impl Q65Codec {
    /// Wrap the given QRA code as a Q65 codec. Currently only
    /// [`QraCodeType::CrcPunctured2`] is supported (the type used by
    /// `qra15_65_64_irr_e23`); other variants will panic in
    /// [`Self::encode`] / [`Self::decode`] with a clear message.
    pub fn new(code: &'static QraCode) -> Self {
        Self {
            scratch: DecoderScratch::for_code(code),
            px: vec![0; code.K],
            py: vec![0; code.N],
            ix: vec![0.0; code.M * code.N],
            ex: vec![0.0; code.M * code.N],
            code,
        }
    }

    /// Number of user-visible information symbols carried per frame
    /// (`K` minus the CRC symbol count). 13 for Q65.
    pub fn msg_len(&self) -> usize {
        match self.code.code_type {
            QraCodeType::Normal => self.code.K,
            QraCodeType::Crc | QraCodeType::CrcPunctured => self.code.K - 1,
            QraCodeType::CrcPunctured2 => self.code.K - 2,
        }
    }

    /// Number of channel symbols actually transmitted per frame
    /// (`N` minus the punctured symbol count). 63 for Q65.
    pub fn channel_len(&self) -> usize {
        match self.code.code_type {
            QraCodeType::Normal | QraCodeType::Crc => self.code.N,
            QraCodeType::CrcPunctured => self.code.N - 1,
            QraCodeType::CrcPunctured2 => self.code.N - 2,
        }
    }

    /// Encode `msg` (length [`Self::msg_len`]) into `out` (length
    /// [`Self::channel_len`]). Each symbol value must be in
    /// `0 .. self.code.M`.
    ///
    /// For Q65 the flow is: `13 user → +CRC12 → 15 info → QRA encode
    /// → 65 codeword → puncture 2 CRC → 63 transmitted`.
    pub fn encode(&mut self, msg: &[i32], out: &mut [i32]) {
        let n_k_user = self.msg_len();
        let n_chan = self.channel_len();
        assert_eq!(msg.len(), n_k_user, "encode: msg.len() != msg_len()");
        assert_eq!(out.len(), n_chan, "encode: out.len() != channel_len()");

        // Stage 1: copy the user message into px and append the CRC.
        self.px[..n_k_user].copy_from_slice(msg);
        match self.code.code_type {
            QraCodeType::Normal => {}
            QraCodeType::Crc | QraCodeType::CrcPunctured => {
                self.px[n_k_user] = crc6(&self.px[..n_k_user]);
            }
            QraCodeType::CrcPunctured2 => {
                let crc = crc12(&self.px[..n_k_user]);
                self.px[n_k_user] = crc[0];
                self.px[n_k_user + 1] = crc[1];
            }
        }

        // Stage 2: QRA-encode the K-symbol info into the N-symbol
        // codeword.
        self.code.encode(&self.px, &mut self.py);

        // Stage 3: puncture as required and copy to the caller's
        // output buffer. The C code uses `nK = _q65_get_message_length`
        // (= user-visible 13 for Q65) here, NOT the underlying QRA
        // code's K (= 15). The CRC symbols sit at codeword positions
        // [n_k_user, n_k_user + crc_count) and are skipped.
        match self.code.code_type {
            QraCodeType::Normal | QraCodeType::Crc => {
                out.copy_from_slice(&self.py);
            }
            QraCodeType::CrcPunctured => {
                out[..n_k_user].copy_from_slice(&self.py[..n_k_user]);
                out[n_k_user..].copy_from_slice(&self.py[n_k_user + 1..]);
            }
            QraCodeType::CrcPunctured2 => {
                out[..n_k_user].copy_from_slice(&self.py[..n_k_user]);
                out[n_k_user..].copy_from_slice(&self.py[n_k_user + 2..]);
            }
        }
    }

    /// Soft-decision Q65 decode.
    ///
    /// `intrinsics` holds per-channel-symbol probability distributions
    /// over GF(M) — `intrinsics.len()` must equal `M *
    /// channel_len()`. The wrapper depunctures by inserting uniform
    /// distributions at the CRC positions, then runs QRA BP and MAP
    /// decoding, and finally verifies the recovered CRC matches.
    ///
    /// On success returns the number of BP iterations consumed and
    /// fills `msg` (length [`Self::msg_len`]). On failure returns
    /// either [`Q65DecodeError::NotConverged`] or
    /// [`Q65DecodeError::CrcMismatch`].
    pub fn decode(
        &mut self,
        intrinsics: &[f32],
        msg: &mut [i32],
        max_iters: u32,
    ) -> Result<u32, Q65DecodeError> {
        self.decode_inner(intrinsics, msg, max_iters, None)
    }

    /// Q65 decode with **a-priori** information.
    ///
    /// `ap_mask` (length [`Self::msg_len`]) carries the 6-bit "which
    /// bits are known" mask per user info symbol; `ap_symbols` holds
    /// the corresponding known GF(64) values (the bits where
    /// `ap_mask[k]` is 0 are ignored). Call
    /// [`crate::msg::q65::ap_hint_to_q65_mask`] to convert from a
    /// human-readable [`crate::msg::ApHint`].
    ///
    /// AP masking is applied to the depunctured intrinsics
    /// **before** BP — the same `_q65_mask` step the C reference
    /// uses. CRC verification on success is unchanged.
    pub fn decode_with_ap(
        &mut self,
        intrinsics: &[f32],
        msg: &mut [i32],
        max_iters: u32,
        ap_mask: &[i32],
        ap_symbols: &[i32],
    ) -> Result<u32, Q65DecodeError> {
        assert_eq!(
            ap_mask.len(),
            self.msg_len(),
            "decode_with_ap: ap_mask length"
        );
        assert_eq!(
            ap_symbols.len(),
            self.msg_len(),
            "decode_with_ap: ap_symbols length"
        );
        self.decode_inner(intrinsics, msg, max_iters, Some((ap_mask, ap_symbols)))
    }

    /// Shared implementation for [`Self::decode`] and
    /// [`Self::decode_with_ap`].
    fn decode_inner(
        &mut self,
        intrinsics: &[f32],
        msg: &mut [i32],
        max_iters: u32,
        ap: Option<(&[i32], &[i32])>,
    ) -> Result<u32, Q65DecodeError> {
        let n_k_user = self.msg_len();
        let n_chan = self.channel_len();
        let big_m = self.code.M;
        assert_eq!(msg.len(), n_k_user, "decode: msg.len() != msg_len()");
        assert_eq!(
            intrinsics.len(),
            big_m * n_chan,
            "decode: intrinsics length"
        );

        // Stage 1: depuncture the intrinsics into the full N-row
        // layout the QRA decoder expects. The CRC rows that were
        // never transmitted get the uniform distribution (no a-priori
        // information). All offsets are in the **user-visible**
        // numbering (matching C's `nK = _q65_get_message_length`),
        // so the CRC sits at rows [n_k_user, n_k_user + crc_count).
        let uniform = pdmath::uniform(big_m);
        match self.code.code_type {
            QraCodeType::Normal | QraCodeType::Crc => {
                self.ix.copy_from_slice(intrinsics);
            }
            QraCodeType::CrcPunctured => {
                let info_bytes = big_m * n_k_user;
                self.ix[..info_bytes].copy_from_slice(&intrinsics[..info_bytes]);
                self.ix[info_bytes..info_bytes + big_m].copy_from_slice(uniform);
                self.ix[info_bytes + big_m..].copy_from_slice(&intrinsics[info_bytes..]);
            }
            QraCodeType::CrcPunctured2 => {
                let info_bytes = big_m * n_k_user;
                self.ix[..info_bytes].copy_from_slice(&intrinsics[..info_bytes]);
                self.ix[info_bytes..info_bytes + big_m].copy_from_slice(uniform);
                self.ix[info_bytes + big_m..info_bytes + 2 * big_m].copy_from_slice(uniform);
                self.ix[info_bytes + 2 * big_m..].copy_from_slice(&intrinsics[info_bytes..]);
            }
        }

        // Stage 1.5: apply AP mask if any. For each of the n_k_user
        // info symbols with non-zero mask, zero out probabilities for
        // GF(M) values that disagree with the known bits, then
        // re-normalise. Mirrors `_q65_mask` in q65.c.
        if let Some((ap_mask, ap_symbols)) = ap {
            for k in 0..n_k_user {
                let smask = ap_mask[k];
                if smask == 0 {
                    continue;
                }
                let xk = ap_symbols[k];
                let row = &mut self.ix[big_m * k..big_m * (k + 1)];
                for kk in 0..big_m {
                    if (kk as i32 ^ xk) & smask != 0 {
                        row[kk] = 0.0;
                    }
                }
                pdmath::norm(row);
            }
        }

        // Stage 2: BP. Convergence here doesn't yet prove the decode
        // is right — that's the CRC's job in stage 3.
        let rc = self
            .code
            .extrinsic(&mut self.ex, &self.ix, max_iters, &mut self.scratch);
        let iterations = match rc {
            ExtrinsicResult::Converged { iterations } => iterations,
            ExtrinsicResult::NotConverged | ExtrinsicResult::BadCodeTables => {
                return Err(Q65DecodeError::NotConverged);
            }
        };

        // Stage 3: MAP decode → recover all K info symbols (including
        // the CRC), then verify the CRC matches.
        self.code.map_decode(&mut self.ex, &self.ix, &mut self.px);

        match self.code.code_type {
            QraCodeType::Normal => {}
            QraCodeType::Crc | QraCodeType::CrcPunctured => {
                let crc = crc6(&self.px[..n_k_user]);
                if crc != self.px[n_k_user] {
                    return Err(Q65DecodeError::CrcMismatch);
                }
            }
            QraCodeType::CrcPunctured2 => {
                let crc = crc12(&self.px[..n_k_user]);
                if crc[0] != self.px[n_k_user] || crc[1] != self.px[n_k_user + 1] {
                    return Err(Q65DecodeError::CrcMismatch);
                }
            }
        }

        msg.copy_from_slice(&self.px[..n_k_user]);
        Ok(iterations)
    }
}

/// CRC-6 over a sequence of 6-bit GF(64) symbols.
///
/// Generator polynomial g(x) = x^6 + x + 1, processed LSB-first. Bit-
/// for-bit equivalent to `_q65_crc6` from the C reference.
pub fn crc6(x: &[i32]) -> i32 {
    let mut sr: i32 = 0;
    for &symbol in x {
        let mut t = symbol;
        for _ in 0..6 {
            if (t ^ sr) & 0x01 != 0 {
                sr = (sr >> 1) ^ CRC6_GEN_POL;
            } else {
                sr >>= 1;
            }
            t >>= 1;
        }
    }
    sr
}

/// CRC-12 over a sequence of 6-bit GF(64) symbols, returned as two
/// 6-bit symbols (`[low6, high6]`) ready to append to the K-symbol
/// information vector.
///
/// Generator polynomial g(x) = x^12 + x^11 + x^3 + x^2 + x + 1.
/// Matches `_q65_crc12` from the C reference.
pub fn crc12(x: &[i32]) -> [i32; 2] {
    let mut sr: i32 = 0;
    for &symbol in x {
        let mut t = symbol;
        for _ in 0..6 {
            if (t ^ sr) & 0x01 != 0 {
                sr = (sr >> 1) ^ CRC12_GEN_POL;
            } else {
                sr >>= 1;
            }
            t >>= 1;
        }
    }
    [sr & 0x3F, sr >> 6]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;

    fn perfect_intrinsics(channel: &[i32], m: usize) -> Vec<f32> {
        let mut ix = vec![0.0_f32; m * channel.len()];
        for (k, &sym) in channel.iter().enumerate() {
            ix[m * k + sym as usize] = 1.0;
        }
        ix
    }

    #[test]
    fn shape_for_q65() {
        let codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        assert_eq!(codec.msg_len(), 13, "Q65 carries 13 user info symbols");
        assert_eq!(codec.channel_len(), 63, "Q65 transmits 63 channel symbols");
    }

    #[test]
    fn crc12_returns_two_six_bit_symbols() {
        // Both halves of the CRC must be valid GF(64) symbols.
        let crc = crc12(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]);
        assert!(crc[0] < 64, "low half = {} not a 6-bit symbol", crc[0]);
        assert!(crc[1] < 64, "high half = {} not a 6-bit symbol", crc[1]);
    }

    #[test]
    fn crc12_zero_input_yields_zero() {
        // A CRC over an all-zero message must itself be zero (the
        // shift register never gets disturbed).
        let crc = crc12(&[0_i32; 13]);
        assert_eq!(crc, [0, 0]);
    }

    #[test]
    fn crc6_zero_input_yields_zero() {
        assert_eq!(crc6(&[0_i32; 14]), 0);
    }

    #[test]
    fn encode_then_decode_clean_recovers_message() {
        // Clean-channel roundtrip: encode 13 user symbols, build a
        // perfect intrinsic from the resulting 63-symbol channel
        // sequence, and decode. The recovered message must match
        // exactly and the CRC must verify.
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);

        let msg: Vec<i32> = (0..13).map(|i| (i * 11 + 7) % 64).collect();
        let mut channel = vec![0_i32; 63];
        codec.encode(&msg, &mut channel);

        let intrinsics = perfect_intrinsics(&channel, 64);
        let mut decoded = vec![0_i32; 13];
        let iters = codec
            .decode(&intrinsics, &mut decoded, 50)
            .expect("clean decode must succeed");

        assert_eq!(decoded, msg);
        assert!(iters < 50, "took too many iterations: {iters}");
    }

    #[test]
    fn encode_then_decode_all_zero_message() {
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let msg = vec![0_i32; 13];
        let mut channel = vec![0_i32; 63];
        codec.encode(&msg, &mut channel);

        // For an all-zero message + zero CRC, the entire codeword is
        // also zero — sanity-check that.
        assert!(channel.iter().all(|&s| s == 0));

        let intrinsics = perfect_intrinsics(&channel, 64);
        let mut decoded = vec![0_i32; 13];
        codec
            .decode(&intrinsics, &mut decoded, 50)
            .expect("clean zero decode must succeed");
        assert_eq!(decoded, msg);
    }

    #[test]
    fn decode_with_ap_zero_mask_matches_plain_decode() {
        // An all-zero AP mask should be a no-op: decode_with_ap and
        // decode must produce the same result on the same input.
        let mut codec_a = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut codec_b = Q65Codec::new(&QRA15_65_64_IRR_E23);

        let msg: Vec<i32> = (0..13).map(|i| (i * 7 + 11) % 64).collect();
        let mut channel = vec![0_i32; 63];
        codec_a.encode(&msg, &mut channel);

        let intrinsics = perfect_intrinsics(&channel, 64);
        let mut decoded_plain = vec![0_i32; 13];
        let mut decoded_ap = vec![0_i32; 13];
        let zero_mask = [0_i32; 13];
        let zero_syms = [0_i32; 13];

        codec_a.decode(&intrinsics, &mut decoded_plain, 50).unwrap();
        codec_b
            .decode_with_ap(&intrinsics, &mut decoded_ap, 50, &zero_mask, &zero_syms)
            .unwrap();
        assert_eq!(decoded_plain, decoded_ap);
        assert_eq!(decoded_plain, msg);
    }

    #[test]
    fn decode_with_ap_full_mask_locks_to_known_message() {
        // A fully-known AP mask (every bit locked) should decode
        // immediately even with a uniform intrinsic — BP has nothing
        // left to figure out. This pins the AP mask wiring without
        // relying on the noise channel.
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let msg: Vec<i32> = (0..13).map(|i| (i * 13 + 5) % 64).collect();

        // Use the encoded channel to construct the intrinsic so the
        // parity rows are consistent with the message.
        let mut channel = vec![0_i32; 63];
        codec.encode(&msg, &mut channel);
        let intrinsics = perfect_intrinsics(&channel, 64);

        let full_mask = [0x3F_i32; 13];
        let mut decoded = vec![0_i32; 13];
        let iters = codec
            .decode_with_ap(&intrinsics, &mut decoded, 50, &full_mask, &msg)
            .expect("full-mask decode must succeed");
        assert_eq!(decoded, msg);
        assert!(iters < 5, "fully-locked decode should converge fast");
    }

    #[test]
    fn decode_with_wrong_ap_never_yields_wrong_message() {
        // Lying to the AP layer must NEVER cause the codec to return
        // the lie as a "valid" decode. Acceptable outcomes are:
        //   - Err(_)               : decoder rejected outright
        //   - Ok with the truth    : the parity constraints pulled
        //     BP back to the real codeword despite the wrong hint
        //     (pd_norm graciously falls back to uniform when AP
        //     zeroes every value the intrinsic favoured)
        // A wrong-message Ok would be a silent corruption — we test
        // explicitly that this never happens.
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let msg: Vec<i32> = (0..13).map(|i| (i * 11 + 1) % 64).collect();
        let mut channel = vec![0_i32; 63];
        codec.encode(&msg, &mut channel);
        let intrinsics = perfect_intrinsics(&channel, 64);

        let mask = [0x3F_i32; 13];
        let mut wrong_syms = msg.clone();
        wrong_syms[0] = (msg[0] + 1) % 64;

        let mut decoded = vec![0_i32; 13];
        let result = codec.decode_with_ap(&intrinsics, &mut decoded, 50, &mask, &wrong_syms);
        match result {
            Err(_) => {}
            Ok(_) => assert_eq!(
                decoded, msg,
                "wrong AP must not produce a wrong-message Ok decode"
            ),
        }
    }

    #[test]
    fn decode_with_wrong_crc_returns_an_error() {
        // Construct a 15-symbol info vector with deliberately wrong
        // CRC bytes at positions [13, 14], encode it with the raw
        // QRA encoder, puncture the CRC out of the channel, and feed
        // perfect intrinsics back to the wrapper. Either:
        //   - BP converges to the original 15-symbol info, the CRC
        //     check then fails (CrcMismatch), or
        //   - BP fails to converge because the punctured rows on a
        //     non-self-consistent codeword keep the EXIT chart short
        //     of the (1, 1) corner (NotConverged).
        // Both outcomes correctly reject the bad frame; either is
        // acceptable as "CRC layer did its job".
        let code = &QRA15_65_64_IRR_E23;
        let mut codec = Q65Codec::new(code);

        let mut info = vec![0_i32; code.K];
        for i in 0..code.K {
            info[i] = (i as i32 * 5 + 1) % 64;
        }
        let mut full_codeword = vec![0_i32; code.N];
        code.encode(&info, &mut full_codeword);

        let n_k_user = codec.msg_len();
        let mut channel = vec![0_i32; codec.channel_len()];
        channel[..n_k_user].copy_from_slice(&full_codeword[..n_k_user]);
        channel[n_k_user..].copy_from_slice(&full_codeword[n_k_user + 2..]);

        let intrinsics = perfect_intrinsics(&channel, 64);
        let mut decoded = vec![0_i32; n_k_user];
        let err = codec
            .decode(&intrinsics, &mut decoded, 50)
            .expect_err("invalid frame must be rejected");
        assert!(
            matches!(
                err,
                Q65DecodeError::CrcMismatch | Q65DecodeError::NotConverged
            ),
            "got unexpected error variant: {err:?}"
        );
    }
}
