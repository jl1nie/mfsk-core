// SPDX-License-Identifier: GPL-3.0-or-later
//! Frame-chain layer for >10-byte payloads.
//!
//! [`crate::msg::PacketBytesMessage`] caps a single uvpacket frame at
//! 10 bytes, which is fine for a status beacon but too small for the
//! realistic applications this protocol targets (APRS frames,
//! ECDSA-signed QSL exchanges of ~250 bytes).
//!
//! This module wraps a longer byte stream into a sequence of single-
//! frame payloads with a small header so a receiver can reassemble
//! the original bytes from frames that may arrive out of order or
//! interleaved with other transmitters' chains.
//!
//! ## Frame header (2 bytes per frame)
//!
//! ```text
//! byte 0 : chain_id (8 bits) — random per-chain tag
//! byte 1 : seq (6 bits, MSB) | end-of-chain (1 bit) | reserved (1 bit, must be zero)
//! ```
//!
//! - `chain_id` lets receivers tell concurrent transmitters' chains
//!   apart in a multi-signal channel. It SHOULD be drawn at random
//!   by the encoder; collisions are tolerated but reduce the
//!   reassembly success rate.
//! - `seq` is a 0-indexed frame counter; max chain length 64.
//! - `end-of-chain = 1` signals "this is the last frame of the chain".
//!   The decoder can finalise as soon as both `end` is seen and every
//!   sequence index from 0 to that frame's seq has been received.
//!
//! ## Per-frame payload
//!
//! After the 2-byte header, each frame carries up to **8 bytes of
//! application payload** (10-byte [`crate::msg::PacketBytesMessage`]
//! payload minus the 2-byte chain header). Maximum chain capacity:
//! 64 frames × 8 bytes = **512 bytes**.
//!
//! ## Concurrent chains
//!
//! [`ChainDecoder`] keeps a bounded ring of in-flight chains keyed
//! on `(audio_freq_quantised, chain_id)` so that two transmitters
//! concurrently emitting chains at different audio frequencies don't
//! cross-pollute each other's reassembly state. The audio frequency
//! is quantised to 10 Hz to absorb sync jitter.

/// Bytes of application payload per chained frame (10-byte frame
/// payload minus the 2-byte chain header).
pub const CHAIN_PAYLOAD_BYTES: usize = 8;

/// Maximum frames per chain (6-bit sequence field).
pub const CHAIN_MAX_FRAMES: usize = 64;

/// Maximum bytes per chain.
pub const CHAIN_MAX_BYTES: usize = CHAIN_PAYLOAD_BYTES * CHAIN_MAX_FRAMES;

/// One chained frame's wire bytes — 10 bytes of [`crate::msg::PacketBytesMessage`]
/// payload (2-byte header + up to 8 bytes of app data).
pub type ChainFrame = Vec<u8>;

/// Why [`ChainEncoder::encode`] could not produce a chain.
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum ChainEncodeError {
    /// Input was empty. Chains must carry at least one byte.
    Empty,
    /// Input exceeds [`CHAIN_MAX_BYTES`] (= 512 B with the default
    /// 64-frame × 8-byte layout). Caller must split into multiple
    /// chains.
    TooLarge { len: usize, max: usize },
}

/// Splits a byte stream into chained uvpacket frame payloads.
///
/// `chain_id` is opaque to the encoder — a real application picks a
/// random 8-bit value or derives it from a hash of the message + a
/// monotonic counter to avoid collisions across concurrent senders.
pub struct ChainEncoder {
    chain_id: u8,
}

impl ChainEncoder {
    /// Construct an encoder that emits frames stamped with the given
    /// `chain_id`.
    pub fn new(chain_id: u8) -> Self {
        Self { chain_id }
    }

    /// Encode `bytes` into a series of chained frame payloads.
    ///
    /// Each returned `Vec<u8>` is exactly 10 bytes — the format
    /// [`crate::msg::PacketBytesMessage::pack`] expects when the
    /// caller marks the message field with the 10-byte payload.
    /// (Frames carrying fewer than 8 application bytes — i.e. the
    /// last frame of a partial-fill chain — are zero-padded to 10
    /// bytes before being handed to the message codec.)
    pub fn encode(&self, bytes: &[u8]) -> Result<Vec<ChainFrame>, ChainEncodeError> {
        if bytes.is_empty() {
            return Err(ChainEncodeError::Empty);
        }
        if bytes.len() > CHAIN_MAX_BYTES {
            return Err(ChainEncodeError::TooLarge {
                len: bytes.len(),
                max: CHAIN_MAX_BYTES,
            });
        }
        let n_frames = bytes.len().div_ceil(CHAIN_PAYLOAD_BYTES);
        let mut out = Vec::with_capacity(n_frames);
        for i in 0..n_frames {
            let start = i * CHAIN_PAYLOAD_BYTES;
            let end = (start + CHAIN_PAYLOAD_BYTES).min(bytes.len());
            let last = i == n_frames - 1;
            let mut frame = vec![0u8; 10];
            frame[0] = self.chain_id;
            // seq in upper 6 bits, end-of-chain in bit 1 (LSB-1), reserved bit 0.
            let seq6 = (i as u8) & 0x3f;
            let end_bit = if last { 1u8 } else { 0u8 };
            frame[1] = (seq6 << 2) | (end_bit << 1);
            frame[2..2 + (end - start)].copy_from_slice(&bytes[start..end]);
            // remainder of the 10-byte frame is left as zero pad
            out.push(frame);
        }
        Ok(out)
    }
}

/// In-flight chain reassembly state.
struct PartialChain {
    received: [Option<Vec<u8>>; CHAIN_MAX_FRAMES],
    end_seen_at: Option<usize>,
}

impl PartialChain {
    fn new() -> Self {
        // `Option<Vec<u8>>` is not `Copy` so we can't use the array literal
        // shorthand; build it elementwise.
        let received: [Option<Vec<u8>>; CHAIN_MAX_FRAMES] =
            std::array::from_fn(|_| None);
        Self {
            received,
            end_seen_at: None,
        }
    }

    fn try_finalise(&self) -> Option<Vec<u8>> {
        let end_seq = self.end_seen_at?;
        let mut out = Vec::new();
        for i in 0..=end_seq {
            out.extend_from_slice(self.received[i].as_ref()?);
        }
        Some(out)
    }
}

/// Reassembles complete byte streams from a stream of decoded uvpacket
/// frames that may arrive out of order, partially overlap with other
/// transmitters' chains, or have gaps.
///
/// The decoder keeps a small bounded ring of in-flight chains keyed
/// on `(audio_freq_quantised, chain_id)` so concurrent transmitters
/// at different audio centres do not cross-pollute each other's
/// reassembly state.
pub struct ChainDecoder {
    /// Up to `max_chains` partial chains in flight at any time.
    /// Oldest entries are evicted when capacity is reached.
    chains: Vec<((u32, u8), PartialChain)>,
    max_chains: usize,
}

impl Default for ChainDecoder {
    fn default() -> Self {
        Self::new(16)
    }
}

impl ChainDecoder {
    /// Construct a decoder that holds at most `max_chains` partial
    /// chains in flight. Older entries are evicted FIFO when the
    /// limit is hit.
    pub fn new(max_chains: usize) -> Self {
        assert!(max_chains > 0, "max_chains must be > 0");
        Self {
            chains: Vec::with_capacity(max_chains),
            max_chains,
        }
    }

    /// Feed one decoded uvpacket frame's payload bytes (the 10-byte
    /// output of [`crate::msg::PacketBytesMessage::unpack`]) plus the
    /// audio-centre frequency at which it was decoded. Returns
    /// `Some(complete_bytes)` if this frame completes a chain;
    /// `None` otherwise.
    pub fn submit(&mut self, frame: &[u8], audio_freq_hz: f32) -> Option<Vec<u8>> {
        if frame.len() < 2 {
            return None;
        }
        let chain_id = frame[0];
        let seq = (frame[1] >> 2) & 0x3f;
        let end = ((frame[1] >> 1) & 1) == 1;
        let reserved = frame[1] & 1;
        if reserved != 0 {
            return None; // header malformed
        }
        // Quantise audio frequency to 10 Hz to absorb sync jitter so two
        // frames from the same transmitter at slightly different reported
        // audio centres still collide on the same key.
        let freq_q = (audio_freq_hz / 10.0).round() as i64 as u32;
        let key = (freq_q, chain_id);

        // Find existing chain or start a new one (with FIFO eviction).
        let pos = match self.chains.iter().position(|(k, _)| *k == key) {
            Some(p) => p,
            None => {
                if self.chains.len() == self.max_chains {
                    self.chains.remove(0);
                }
                self.chains.push((key, PartialChain::new()));
                self.chains.len() - 1
            }
        };

        let payload_end = frame.len().min(10);
        let payload = &frame[2..payload_end];
        let payload_len = payload.len().min(CHAIN_PAYLOAD_BYTES);
        let chain = &mut self.chains[pos].1;
        if chain.received[seq as usize].is_none() {
            chain.received[seq as usize] = Some(payload[..payload_len].to_vec());
        }
        if end && chain.end_seen_at.is_none() {
            chain.end_seen_at = Some(seq as usize);
        }
        if let Some(complete) = chain.try_finalise() {
            // Remove the chain from the in-flight ring once it's done.
            let actual_len = if let Some(end_seq) = chain.end_seen_at {
                // Trim trailing zero pad on the LAST frame: when the
                // final chunk is < 8 bytes, the encoder pads with
                // zeros; the decoder can't tell zero-pad from real
                // zero bytes without external length info, so we
                // return the chain bytes as-is. The application is
                // responsible for any external length framing (e.g.
                // CBOR / serde-bincode self-delimiting payloads).
                let _ = end_seq;
                complete.len()
            } else {
                complete.len()
            };
            let truncated = complete[..actual_len].to_vec();
            self.chains.remove(pos);
            return Some(truncated);
        }
        None
    }

    /// Number of chains currently in flight (mostly useful for tests).
    pub fn in_flight(&self) -> usize {
        self.chains.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_rejected() {
        let enc = ChainEncoder::new(0x42);
        match enc.encode(&[]) {
            Err(ChainEncodeError::Empty) => {}
            other => panic!("expected Empty error, got {other:?}"),
        }
    }

    #[test]
    fn oversize_input_rejected() {
        let enc = ChainEncoder::new(0x42);
        let big = vec![0u8; CHAIN_MAX_BYTES + 1];
        match enc.encode(&big) {
            Err(ChainEncodeError::TooLarge { len, max }) => {
                assert_eq!(len, CHAIN_MAX_BYTES + 1);
                assert_eq!(max, CHAIN_MAX_BYTES);
            }
            other => panic!("expected TooLarge error, got {other:?}"),
        }
    }

    #[test]
    fn single_frame_roundtrip() {
        let enc = ChainEncoder::new(0xAB);
        let mut dec = ChainDecoder::default();
        let frames = enc.encode(b"hi").expect("encode");
        assert_eq!(frames.len(), 1, "2 bytes fits in one frame");
        let out = dec.submit(&frames[0], 1500.0);
        // The "hi" payload is 2 bytes, but a single-frame chain
        // returns the full 8-byte payload region (zero-padded). The
        // application is responsible for trimming via external
        // length info.
        let out = out.expect("first frame finalises a 1-frame chain");
        assert_eq!(&out[..2], b"hi");
    }

    #[test]
    fn multi_frame_roundtrip_in_order() {
        let enc = ChainEncoder::new(0xCD);
        let mut dec = ChainDecoder::default();
        let bytes: Vec<u8> = (0..20).collect(); // 20 bytes → 3 frames
        let frames = enc.encode(&bytes).expect("encode");
        assert_eq!(frames.len(), 3);
        let mut last = None;
        for f in &frames {
            last = dec.submit(f, 1500.0);
        }
        let out = last.expect("third frame finalises chain");
        assert_eq!(&out[..bytes.len()], bytes.as_slice());
    }

    #[test]
    fn multi_frame_roundtrip_out_of_order() {
        let enc = ChainEncoder::new(0xCD);
        let mut dec = ChainDecoder::default();
        let bytes: Vec<u8> = (0..24).collect(); // 24 bytes → 3 frames
        let mut frames = enc.encode(&bytes).expect("encode");
        // Reverse and submit
        frames.reverse();
        let mut completed = None;
        for f in &frames {
            if let Some(out) = dec.submit(f, 1500.0) {
                completed = Some(out);
            }
        }
        let out = completed.expect("chain completes regardless of order");
        assert_eq!(&out[..bytes.len()], bytes.as_slice());
    }

    #[test]
    fn concurrent_chains_at_different_freqs_dont_collide() {
        let enc_a = ChainEncoder::new(0xAA);
        let enc_b = ChainEncoder::new(0xAA); // same chain_id deliberately
        let mut dec = ChainDecoder::default();
        let frames_a = enc_a.encode(b"chain-A-data").expect("a");
        let frames_b = enc_b.encode(b"chain-B-data").expect("b");
        // Submit interleaved at distinct audio freqs.
        let mut a_done = None;
        let mut b_done = None;
        let max = frames_a.len().max(frames_b.len());
        for i in 0..max {
            if i < frames_a.len()
                && let Some(o) = dec.submit(&frames_a[i], 1200.0)
            {
                a_done = Some(o);
            }
            if i < frames_b.len()
                && let Some(o) = dec.submit(&frames_b[i], 1800.0)
            {
                b_done = Some(o);
            }
        }
        let a = a_done.expect("chain A completes");
        let b = b_done.expect("chain B completes");
        assert!(
            a.starts_with(b"chain-A-data"),
            "chain A reassembled: {a:?}"
        );
        assert!(
            b.starts_with(b"chain-B-data"),
            "chain B reassembled: {b:?}"
        );
    }

    #[test]
    fn frame_count_for_max_chain() {
        let enc = ChainEncoder::new(0);
        let bytes = vec![0u8; CHAIN_MAX_BYTES];
        let frames = enc.encode(&bytes).expect("encode max");
        assert_eq!(frames.len(), CHAIN_MAX_FRAMES);
    }
}
