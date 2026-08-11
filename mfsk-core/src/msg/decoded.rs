//! `Decoded` — a unified, owned, human-readable decode row for host UIs.
//!
//! Every protocol's native decode result (`engine::pipeline::DecodeResult`
//! for FT8/FT4/FST4, [`WsprResult`](crate::wspr::WsprResult),
//! [`Q65Result`](crate::q65::Q65Result), [`Jt65Result`](crate::jt65::Jt65Result),
//! [`Jt9Result`](crate::jt9::Jt9Result)) is structurally distinct — different
//! field names, and four different message-text paths (FT8-family unpack via
//! [`unpack77`](crate::msg::wsjt77::unpack77), `Display` for WSPR/JT65/JT9, an
//! already-resolved `String` for Q65). A host UI, a spotting uploader, or an
//! IPC/websocket bridge that shows
//! decodes from more than one mode ends up re-writing the same "pull out the
//! text + frequency + dt + SNR" glue per result type.
//!
//! [`Decoded`] is that common row, resolved once at the conversion boundary:
//! the exact columns a decode list binds to for *every* mode, as an owned
//! (`Clone`, `Send`) value that crosses a thread/channel boundary cleanly —
//! e.g. dropped into a `tokio::sync::mpsc::Sender` inside a streaming
//! `.on_result` callback (see `docs/reference/STREAMING.md`).
//!
//! It is **additive, not a replacement**: the native result types stay, carry
//! their mode-specific diagnostics (`sync_score`, `hard_errors`, `iterations`,
//! …), and remain `Clone`/`Send` for callers that need that detail across a
//! channel. `Decoded` deliberately holds only the cross-mode intersection —
//! those extras don't generalise into clean shared columns, so hoisting them
//! here would just reintroduce per-mode branching.
//!
//! ## Conversions
//!
//! Each protocol's result grows a `to_decoded(..)` method. The signatures
//! differ because the modes genuinely differ — this is honest, not an
//! oversight:
//!
//! - **FT8/FT4/FST4** (`DecodeResult::to_decoded`) is **fallible**
//!   (`Option<Decoded>`): the 77-bit payload is unpacked here, and a payload
//!   that survived the CRC but fails to unpack (rare) yields `None` rather
//!   than a placeholder row. It also takes the caller's
//!   [`ProtocolId`](crate::engine::protocol::ProtocolId) (the result type is
//!   shared across the three) and an optional
//!   [`CallsignHashTable`](crate::msg::hash_table::CallsignHashTable) to
//!   resolve `<...>` hashed callsigns.
//! - **WSPR** ([`WsprResult`](crate::wspr::WsprResult)`::to_decoded`) is
//!   infallible — text via `Display`, `dt_sec` already present on the result.
//! - **Q65 / JT65 / JT9** take `(sample_rate, nominal_start_sample)` to derive
//!   `dt_sec` from the result's `start_sample` (those modes report a sample
//!   index, not a pre-computed dt). `nominal_start_sample` is the same anchor
//!   the caller passed to `decode_scan` / the decode builder.
//!
//! ## `serde`
//!
//! Under `--features serde`, [`Decoded`] and
//! [`ProtocolId`](crate::engine::protocol::ProtocolId) derive
//! `Serialize`/`Deserialize` (`no_std`-clean via serde's `alloc` feature), so
//! a UI can emit decode rows straight to JSON for spotting, a websocket, or
//! IPC. Off by default; purely additive.

use alloc::string::String;
// `ToString` is only used by the modes whose message text comes via `Display`
// (WSPR / JT65 / JT9); gate the import so an FT8/FT4/FST4-only build (whose
// text path is `unpack77`, returning `String` directly) doesn't warn on it.
#[cfg(all(
    any(feature = "wspr", feature = "jt65", feature = "jt9"),
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
use alloc::string::ToString;

use crate::engine::protocol::ProtocolId;

/// One decoded message, resolved into the columns a host UI actually shows,
/// as an owned value that moves across threads/channels cleanly.
///
/// See the [module docs](self) for the rationale (why this is the cross-mode
/// intersection and not a superset) and the per-protocol `to_decoded`
/// conversions that produce it.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Decoded {
    /// Human-readable message text, already resolved (unpacked / formatted).
    pub text: String,
    /// Tone-0 (base) frequency in the audio passband, Hz.
    pub freq_hz: f32,
    /// Signal-start offset in seconds, signed, relative to the mode's nominal
    /// slot anchor (positive = late). `0.0` where the source mode has no
    /// meaningful dt to report.
    pub dt_sec: f32,
    /// SNR estimate in dB. WSJT-X 2500 Hz reference-bandwidth convention for
    /// every mode except JT9 (see [`Jt9Result::snr_db`](crate::jt9::Jt9Result),
    /// whose estimate is comparable across JT9 decodes but not in absolute
    /// terms to other modes).
    pub snr_db: f32,
    /// Which protocol produced this decode (UI mode column). Q65 sub-modes and
    /// uvpacket sub-modes collapse onto their family tag here.
    pub protocol: ProtocolId,
}

// ── FT8 / FT4 / FST4 ─────────────────────────────────────────────────────────
//
// These three share `engine::pipeline::DecodeResult`. The impl lives in `msg`
// (not `engine`) because it calls `unpack77` — `engine` never depends on `msg`
// (the crate-wide dependency direction), so authoring the conversion here keeps
// that arrow pointing the right way while still giving `result.to_decoded(..)`
// method syntax.
#[cfg(all(
    any(feature = "ft8", feature = "ft4", feature = "fst4"),
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
impl crate::engine::pipeline::DecodeResult {
    /// Resolve into a [`Decoded`] UI row, unpacking the 77-bit payload.
    ///
    /// `protocol` tags the row (the result type is shared across FT8/FT4/FST4,
    /// so the caller says which). `hash`, when supplied, resolves `<...>`
    /// hashed callsigns via [`unpack77_with_hash`](crate::msg::wsjt77::unpack77_with_hash);
    /// pass `None` to unpack without a hash table.
    ///
    /// Returns `None` if the payload fails to unpack — a decode that passed
    /// the CRC but can't be interpreted is not surfaced as a row (see the
    /// module docs).
    pub fn to_decoded(
        &self,
        protocol: ProtocolId,
        hash: Option<&crate::msg::hash_table::CallsignHashTable>,
    ) -> Option<Decoded> {
        let text = match hash {
            Some(ht) => crate::msg::wsjt77::unpack77_with_hash(self.message77(), ht)?,
            None => crate::msg::wsjt77::unpack77(self.message77())?,
        };
        Some(Decoded {
            text,
            freq_hz: self.freq_hz,
            dt_sec: self.dt_sec,
            snr_db: self.snr_db,
            protocol,
        })
    }
}

// ── WSPR ─────────────────────────────────────────────────────────────────────
#[cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]
impl crate::wspr::WsprResult {
    /// Resolve into a [`Decoded`] UI row. Infallible: text via the message's
    /// `Display`, `dt_sec` copied through from the result.
    pub fn to_decoded(&self) -> Decoded {
        Decoded {
            text: self.message.to_string(),
            freq_hz: self.freq_hz,
            dt_sec: self.dt_sec,
            snr_db: self.snr_db,
            protocol: ProtocolId::Wspr,
        }
    }
}

// ── Q65 ──────────────────────────────────────────────────────────────────────
#[cfg(all(feature = "q65", any(feature = "fft-rustfft", feature = "fft-extern")))]
impl crate::q65::Q65Result {
    /// Resolve into a [`Decoded`] UI row.
    ///
    /// `dt_sec` is derived from the result's `start_sample`:
    /// `(start_sample − nominal_start_sample) / sample_rate`, where
    /// `nominal_start_sample` is the anchor the caller passed to the decode
    /// builder.
    pub fn to_decoded(&self, sample_rate: u32, nominal_start_sample: usize) -> Decoded {
        Decoded {
            text: self.message.clone(),
            freq_hz: self.freq_hz,
            dt_sec: dt_from_samples(self.start_sample, nominal_start_sample, sample_rate),
            snr_db: self.snr_db,
            protocol: ProtocolId::Q65,
        }
    }
}

// ── JT65 ─────────────────────────────────────────────────────────────────────
#[cfg(all(feature = "jt65", any(feature = "fft-rustfft", feature = "fft-extern")))]
impl crate::jt65::Jt65Result {
    /// Resolve into a [`Decoded`] UI row. Text via the message's `Display`;
    /// `dt_sec` derived from `start_sample` the same way Q65's conversion does.
    pub fn to_decoded(&self, sample_rate: u32, nominal_start_sample: usize) -> Decoded {
        Decoded {
            text: self.message.to_string(),
            freq_hz: self.freq_hz,
            dt_sec: dt_from_samples(self.start_sample, nominal_start_sample, sample_rate),
            snr_db: self.snr_db,
            protocol: ProtocolId::Jt65,
        }
    }
}

// ── JT9 ──────────────────────────────────────────────────────────────────────
#[cfg(all(feature = "jt9", any(feature = "fft-rustfft", feature = "fft-extern")))]
impl crate::jt9::Jt9Result {
    /// Resolve into a [`Decoded`] UI row. Text via the message's `Display`;
    /// `dt_sec` derived from `start_sample` the same way Q65's conversion does.
    ///
    /// Note `snr_db` here is JT9's own per-symbol estimate, comparable across
    /// JT9 decodes but not in absolute terms to other modes' `snr_db`.
    pub fn to_decoded(&self, sample_rate: u32, nominal_start_sample: usize) -> Decoded {
        Decoded {
            text: self.message.to_string(),
            freq_hz: self.freq_hz,
            dt_sec: dt_from_samples(self.start_sample, nominal_start_sample, sample_rate),
            snr_db: self.snr_db,
            protocol: ProtocolId::Jt9,
        }
    }
}

/// `(start − nominal) / sample_rate`, the signed dt convention every mode's
/// `Decoded` uses. Factored out so the three sample-index modes share one
/// definition.
#[cfg(any(feature = "q65", feature = "jt65", feature = "jt9"))]
#[inline]
fn dt_from_samples(start_sample: usize, nominal_start_sample: usize, sample_rate: u32) -> f32 {
    (start_sample as f32 - nominal_start_sample as f32) / sample_rate as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "serde")]
    #[test]
    fn decoded_and_protocol_id_are_serde() {
        // Proving the derives are wired needs no serialization format — a
        // trait-bound assertion is enough and keeps the test dependency-free.
        fn assert_serde<T: serde::Serialize + serde::de::DeserializeOwned>() {}
        assert_serde::<Decoded>();
        assert_serde::<ProtocolId>();
    }

    #[cfg(all(
        any(feature = "ft8", feature = "ft4", feature = "fst4"),
        any(feature = "fft-rustfft", feature = "fft-extern")
    ))]
    #[test]
    fn ft8_family_to_decoded_carries_unpacked_text_and_fields() {
        use crate::engine::pipeline::DecodeResult;
        use crate::msg::wsjt77::{pack77_type1, unpack77};

        let bits = pack77_type1("K1ABC", "W9XYZ", "FN42").expect("packs");
        // `info` is the K-bit payload; the conversion only reads the leading
        // 77 message bits (`message77()`), so a 77-length payload is enough.
        let info = bits.to_vec().into_boxed_slice();
        let expected = unpack77(&bits).expect("unpacks");

        let r = DecodeResult {
            info,
            freq_hz: 1234.5,
            dt_sec: 0.2,
            hard_errors: 0,
            sync_score: 3.0,
            pass: 1,
            sync_cv: 0.1,
            snr_db: -5.0,
        };

        let d = r
            .to_decoded(ProtocolId::Ft8, None)
            .expect("valid payload unpacks");
        assert_eq!(d.text, expected);
        assert_eq!(d.freq_hz, 1234.5);
        assert_eq!(d.dt_sec, 0.2);
        assert_eq!(d.snr_db, -5.0);
        assert_eq!(d.protocol, ProtocolId::Ft8);
    }

    #[cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]
    #[test]
    fn wspr_to_decoded_is_infallible_and_passes_dt_through() {
        use crate::msg::wspr::WsprMessage;
        use crate::wspr::WsprResult;

        let msg = WsprMessage::Type1 {
            callsign: "K1ABC".into(),
            grid: "FN42".into(),
            power_dbm: 37,
        };
        let expected = msg.to_string();

        let r = WsprResult {
            message: msg,
            freq_hz: 1400.0,
            start_sample: 0,
            dt_sec: 1.0,
            drift_hz: 0.0,
            info_bits: [0u8; 50],
            snr_db: -20.0,
        };

        let d = r.to_decoded();
        assert_eq!(d.text, expected);
        assert_eq!(d.freq_hz, 1400.0);
        assert_eq!(d.dt_sec, 1.0);
        assert_eq!(d.protocol, ProtocolId::Wspr);
    }

    #[cfg(all(feature = "q65", any(feature = "fft-rustfft", feature = "fft-extern")))]
    #[test]
    fn q65_to_decoded_derives_dt_from_sample_rate() {
        use crate::q65::Q65Result;

        let r = Q65Result {
            message: "K1ABC W9XYZ FN42".into(),
            freq_hz: 1000.0,
            start_sample: 12_000,
            iterations: 3,
            snr_db: -24.0,
        };

        // (12000 − 6000) / 12000 = 0.5 s late.
        let d = r.to_decoded(12_000, 6_000);
        assert_eq!(d.text, "K1ABC W9XYZ FN42");
        assert_eq!(d.protocol, ProtocolId::Q65);
        assert!((d.dt_sec - 0.5).abs() < 1e-6);
    }
}
