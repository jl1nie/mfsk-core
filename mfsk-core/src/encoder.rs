// SPDX-License-Identifier: GPL-3.0-only
//! One transmit API over [`Mode`], as [`AnyDecoder`](crate::decoder::AnyDecoder)
//! is for receive: [`encode`] a message text to channel symbols, then
//! [`Encoded::synthesize`] them to audio.
//!
//! Nothing here is new signal processing. Each mode goes through the code it
//! has always had (`engine::tx::message_to_tones`, `q65::encode_channel_symbols_flagged`,
//! `fst4w::encode::payload_to_tones`, `wspr::encode_channel_symbols`,
//! `jt9` / `jt65::encode_channel_symbols`); this module picks the route by
//! [`Mode`] and packs the text with
//! `msg::wsjt77::pack77_message`, the port of WSJT-X's `pack77`. The waveform of a message is the one the per-mode
//! functions give (`tests/encoder.rs` compares them).
//!
//! The module needs no FFT backend (transmit does no spectral analysis), so
//! it is available in every build; a [`Mode`] exists only for the protocol
//! features compiled in, and so does its arm here.
//!
//! MSK144 and JTTY are not [`Mode`]s and keep their own transmit modules
//! (`msk144::tx`, `jtty::pack` + `jtty::tx`).
//!
//! # Message forms
//!
//! | modes | text |
//! |---|---|
//! | FT8, FT4, FST4, Q65 | any message `pack77` takes (standard, non-standard, free text, ...) |
//! | FST4W | a WSPR-family message: `CALL GRID4 DBM`, `PFX/CALL DBM`, `<CALL> GRID6 DBM` |
//! | WSPR | `CALL GRID4 DBM` (type 1), `PFX/CALL DBM` or `CALL/SFX DBM` (type 2), `<CALL> GRID6 DBM` (type 3) |
//! | JT9, JT65 | any message `packmsg` takes: standard, `CQ nnn`, add-on prefix/suffix, free text (13 characters) |
//!
//! # WSPR compound calls
//!
//! A WSPR message carries a plain call and a 4-character grid (type 1). A
//! station with a compound call (`PJ4/K1ABC`, `K1ABC/7`) or a 6-character
//! grid cannot send both in one message: it alternates a type 2 message
//! (`PJ4/K1ABC 37`, the compound call and the power) and a type 3 message
//! (`<PJ4/K1ABC> FN42LX 37`, the call as a 15-bit hash, the 6-character grid
//! and the power). Which of the two goes in which transmission is the
//! application's schedule, as in WSJT-X's GUI; [`encode`] sends the one it is
//! given. The text is upper-cased and its blank runs are collapsed before it
//! is packed; the power is rounded to a value WSPR sends (38 dBm goes out as
//! 37), as `wqencode` does.
//!
//! # Example
//!
//! ```
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::Mode;
//! use mfsk_core::encoder::{TxAudio, encode};
//!
//! let tx = encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap();
//! assert_eq!(tx.text(), "CQ JA1ABC PM95");
//! assert_eq!(tx.tones().len(), 79);
//! let audio = tx.synthesize(&TxAudio::new(12_000, 1_500.0));
//! assert_eq!(audio.len(), tx.synth_len(12_000));
//! # }
//! ```

use alloc::string::String;
#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
use alloc::string::ToString;
use alloc::vec::Vec;

use crate::msg::wsjt77::Pack77Error;
#[cfg(any(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    feature = "fst4w",
    feature = "q65"
))]
use crate::msg::wsjt77::{Pack77Options, pack77_message};
use crate::registry::Mode;

/// What [`encode`] produced: the channel symbols, and what a receiver will print.
///
/// The fields are read through accessors: the tones are the ones [`encode`]
/// made for [`mode`](Self::mode), so [`synthesize`](Self::synthesize) always
/// has one in-range tone per symbol of that mode and does not fail (#700).
/// Tones from elsewhere go to `engine::tx::synthesize`, which panics on a
/// count or a tone the mode does not have.
///
/// ```compile_fail,E0616
/// # use mfsk_core::{Mode, encoder::encode};
/// let mut tx = encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap();
/// tx.tones.clear(); // E0616: the tones stay the mode's
/// ```
#[derive(Clone, Debug)]
pub struct Encoded {
    mode: Mode,
    tones: Vec<u8>,
    text: String,
    hashed_calls: Vec<String>,
}
/// Per-call options of [`encode_with`].
#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct EncodeOptions {
    /// Q65 only: set the "copied last Tx" bit of WSJT-X 3.2's Q65 Pileup
    /// (`genq65.f90`'s `iflag`; `q65::encode_channel_symbols_flagged`).
    /// Ignored by the other modes.
    pub q65_copied_last_tx: bool,
}

impl EncodeOptions {
    /// The defaults: no options set.
    pub const fn new() -> Self {
        Self {
            q65_copied_last_tx: false,
        }
    }

    /// Set [`q65_copied_last_tx`](Self::q65_copied_last_tx).
    pub const fn q65_copied_last_tx(mut self, on: bool) -> Self {
        self.q65_copied_last_tx = on;
        self
    }
}

/// How [`Encoded::synthesize`] renders the tones.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct TxAudio {
    pub sample_rate: u32,
    /// The carrier: the frequency of tone 0 (the audio frequency WSJT-X shows).
    pub f0_hz: f32,
    pub amplitude: f32,
}

impl TxAudio {
    /// `sample_rate` and `f0_hz`, at unit amplitude.
    pub const fn new(sample_rate: u32, f0_hz: f32) -> Self {
        Self {
            sample_rate,
            f0_hz,
            amplitude: 1.0,
        }
    }

    /// Set [`amplitude`](Self::amplitude).
    pub const fn amplitude(mut self, a: f32) -> Self {
        self.amplitude = a;
        self
    }
}

/// Why [`encode`] did not produce a message.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum EncodeError {
    /// The text is not a message this mode can send. For the 77-bit modes
    /// `reason` is why `pack77_message` refused it. (WSPR texts have their
    /// own variant, [`EncodeError::Wspr`]; this one is used for WSPR only when
    /// the packed bits do not unpack, `RoundtripRejected`: a `CQ` call or a
    /// report in the grid slot, which `wqencode` packs and no receiver reads.)
    Message { mode: Mode, reason: Pack77Error },
    /// A message form this mode has but this crate does not encode.
    Unsupported { mode: Mode, what: &'static str },
    /// WSPR: `wqencode` does not take the text (`reason` says how).
    Wspr {
        mode: Mode,
        reason: crate::msg::wspr::WsprPackError,
    },
    /// JT9 / JT65: `pack_message` refused the text (`packmsg` itself falls back to
    /// free text, so this is only a character it cannot take).
    Jt72 {
        mode: Mode,
        reason: crate::msg::Jt72PackError,
    },
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Message { mode, reason } => {
                write!(f, "{}: cannot send this message: {}", mode.name(), reason)
            }
            Self::Unsupported { mode, what } => {
                write!(f, "{}: {} are not encoded", mode.name(), what)
            }
            Self::Wspr { mode, reason } => {
                write!(f, "{}: cannot send this message: {}", mode.name(), reason)
            }
            Self::Jt72 { mode, reason } => {
                write!(f, "{}: cannot send this message: {}", mode.name(), reason)
            }
        }
    }
}

impl core::error::Error for EncodeError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Message { reason, .. } => Some(reason),
            Self::Wspr { reason, .. } => Some(reason),
            Self::Unsupported { .. } => None,
            Self::Jt72 { reason, .. } => Some(reason),
        }
    }
}

/// Encode `text` for `mode` with the default [`EncodeOptions`].
pub fn encode(mode: Mode, text: &str) -> Result<Encoded, EncodeError> {
    encode_with(mode, text, &EncodeOptions::new())
}

/// Encode `text` for `mode`.
///
/// Stateless: nothing records the hashes the message sends. Hand
/// [`Encoded::hashed_calls`] to the decoder.
///
/// WSPR: `CALL GRID4 DBM` (type 1), `PFX/CALL DBM` / `CALL/SFX DBM` (type 2)
/// and `<CALL> GRID6 DBM` (type 3), packed by `msg::wspr::pack_message` (the
/// port of `wqencode`); a text it refuses is [`EncodeError::Wspr`].
///
/// JT9 and JT65: `packmsg` takes any text, as WSJT-X does: a callsign form
/// (standard, add-on prefix or suffix) when it parses as one, else free
/// text, cut to 13 characters of its alphabet. Only a non-ASCII character is
/// [`EncodeError::Jt72`].
pub fn encode_with(
    mode: Mode,
    text: &str,
    options: &EncodeOptions,
) -> Result<Encoded, EncodeError> {
    let (tones, text, hashed_calls) = route(mode, text, options)?;
    Ok(Encoded {
        mode,
        tones,
        text,
        hashed_calls,
    })
}

impl Encoded {
    /// The mode the tones are for.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// The channel symbols, one tone index per symbol.
    pub fn tones(&self) -> &[u8] {
        &self.tones
    }

    /// The message as `unpack` renders the transmitted bits (WSJT-X shows
    /// this in its Tx box): it can differ from the input in case, spacing, or
    /// where the mode cannot carry the input as typed. A call sent as a hash
    /// reads as `<CALL>` (the sender knows it; a receiver that has not heard
    /// it prints `<...>`).
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Calls the message sends as hashes; give them to the decoder
    /// (`learn_callsign`) so their `<...>` resolves when the other station
    /// answers (WSJT-X's `record_tx_hashes`). For WSPR the call of a type 3
    /// message; empty for the other WSPR types, JT9 and JT65.
    pub fn hashed_calls(&self) -> &[String] {
        &self.hashed_calls
    }

    /// The audio of one frame: [`synth_len`](Self::synth_len) samples at
    /// `audio.sample_rate`, from sample 0 (the caller places it in the period
    /// at the mode's start offset).
    ///
    /// Does not panic. Every sample rate renders: the samples per symbol are
    /// `sample_rate · symbol time`, rounded, so a rate too low for the mode
    /// gives degenerate or empty audio (0 Hz gives none) rather than an error;
    /// choosing a rate the mode's tones fit under Nyquist is the caller's part
    /// (measured over every mode family at 0 Hz to 192 kHz, #700).
    pub fn synthesize(&self, audio: &TxAudio) -> Vec<f32> {
        synth(
            self.mode,
            &self.tones,
            audio.sample_rate,
            audio.f0_hz,
            audio.amplitude,
        )
    }

    /// 16-bit form of [`synthesize`](Self::synthesize): the waveform at unit
    /// amplitude scaled so its peak is `amplitude_i16`
    /// (`engine::tx::synthesize_i16`); `audio.amplitude` is not used. Does not
    /// panic, as [`synthesize`](Self::synthesize).
    pub fn synthesize_i16(&self, audio: &TxAudio, amplitude_i16: i16) -> Vec<i16> {
        synth_i16(
            self.mode,
            &self.tones,
            audio.sample_rate,
            audio.f0_hz,
            amplitude_i16,
        )
    }

    /// The length of the audio, in samples, at `sample_rate`.
    pub fn synth_len(&self, sample_rate: u32) -> usize {
        synth_len(self.mode, sample_rate)
    }
}

// ── routes ──────────────────────────────────────────────────────────────

type Routed = (Vec<u8>, String, Vec<String>);

#[cfg(any(feature = "ft8", feature = "ft4", feature = "fst4"))]
fn route_s77<P: crate::engine::Protocol>(mode: Mode, text: &str) -> Result<Routed, EncodeError> {
    let p = pack77_message(text, &Pack77Options::new())
        .map_err(|reason| EncodeError::Message { mode, reason })?;
    let tones = crate::engine::tx::message_to_tones::<P>(&p.bits);
    Ok((
        tones,
        text77(mode, &p.bits, &p.hashed_calls)?,
        p.hashed_calls,
    ))
}

/// What `unpack77` renders for `bits`, with the sender's own calls known: a
/// hashed `<W9XYZ>` reads as such, as in WSJT-X's Tx box (a receiver that has
/// not heard the call prints `<...>`).
#[cfg(any(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    feature = "q65",
    feature = "fst4w"
))]
fn text77(mode: Mode, bits: &[u8; 77], hashed_calls: &[String]) -> Result<String, EncodeError> {
    let mut table = crate::msg::hash_table::CallsignHashTable::new();
    for c in hashed_calls {
        table.insert(c);
    }
    crate::msg::wsjt77::unpack77_with_hash(bits, &table).ok_or(EncodeError::Message {
        mode,
        reason: Pack77Error::RoundtripRejected,
    })
}

#[cfg(feature = "q65")]
fn route_q65(mode: Mode, text: &str, options: &EncodeOptions) -> Result<Routed, EncodeError> {
    let p = pack77_message(text, &Pack77Options::new())
        .map_err(|reason| EncodeError::Message { mode, reason })?;
    let tones = crate::q65::encode_channel_symbols_flagged(&p.bits, options.q65_copied_last_tx);
    Ok((
        tones.to_vec(),
        text77(mode, &p.bits, &p.hashed_calls)?,
        p.hashed_calls,
    ))
}

#[cfg(feature = "fst4w")]
fn route_fst4w<P: crate::engine::Protocol>(mode: Mode, text: &str) -> Result<Routed, EncodeError> {
    use crate::fec::ldpc240_74::PAYLOAD_BITS;
    let p = pack77_message(text, &Pack77Options::new().prefer_wspr_50bit(true))
        .map_err(|reason| EncodeError::Message { mode, reason })?;
    // `prefer_wspr_50bit` only orders the candidates; a text that is not a
    // WSPR-family message still packs, as another type (free text "K1ABC
    // FN42 38" is `i3.n3 = 0.0`). Its first 50 bits are then not a message:
    // refuse it, as `Fst4wMessage::pack_text` (`pack77_wspr`) does. Measured
    // by `tests/encoder.rs::fst4w_agrees_with_pack_text`.
    if (p.i3, p.n3) != (0, 6) {
        return Err(EncodeError::Message {
            mode,
            reason: Pack77Error::PreferredFamilyRejected,
        });
    }
    let mut payload = [0u8; PAYLOAD_BITS];
    payload.copy_from_slice(&p.bits[..PAYLOAD_BITS]);
    let tones = crate::fst4w::encode::payload_to_tones::<P>(&payload);
    // What a receiver unpacks: the payload with `PAYLOAD_SUFFIX` appended.
    let bits = crate::fst4w::Fst4wMessage::payload_to_77(&payload).expect("50 bits");
    Ok((tones, text77(mode, &bits, &p.hashed_calls)?, p.hashed_calls))
}

#[cfg(feature = "wspr")]
fn route_wspr(mode: Mode, text: &str) -> Result<Routed, EncodeError> {
    use crate::msg::wspr::{WsprMessage, pack_message, unpack};
    // WSJT-X's GUI hands `wqencode` an upper-case, single-blank message.
    let norm = text
        .split_whitespace()
        .map(str::to_ascii_uppercase)
        .collect::<Vec<_>>()
        .join(" ");
    let p = pack_message(&norm).map_err(|reason| EncodeError::Wspr { mode, reason })?;
    let tones = crate::wspr::encode_channel_symbols(&p.bits).to_vec();
    let unreadable = EncodeError::Message {
        mode,
        reason: Pack77Error::RoundtripRejected,
    };
    let m = unpack(&p.bits).ok_or(unreadable)?;
    let hashed_calls: Vec<String> = p
        .hashed_call
        .into_iter()
        .filter(|c| !c.is_empty())
        .collect();
    // A type 3 call is a hash on the air; the sender knows it (a receiver
    // that has not heard it prints `<...>`), so show it as typed, as for the
    // hashed calls of the 77-bit modes.
    let text = match (&m, hashed_calls.first()) {
        (
            WsprMessage::Type3 {
                grid6, power_dbm, ..
            },
            Some(call),
        ) => alloc::format!("<{call}> {grid6} {power_dbm:>2}"),
        _ => m.to_string(),
    };
    Ok((tones, text, hashed_calls))
}

/// `CALL1 CALL2 GRID/REPORT` packed as the 12 six-bit words of JT9 / JT65.
#[cfg(any(feature = "jt9", feature = "jt65"))]
fn jt_words(mode: Mode, text: &str) -> Result<([u8; 12], String), EncodeError> {
    // `packmsg` (packjt.f90:401-535) takes any text: a callsign form when the text
    // parses as one, free text otherwise.
    let packed = crate::msg::jt72::pack_message(text)
        .map_err(|reason| EncodeError::Jt72 { mode, reason })?;
    let text = crate::msg::jt72::unpack(&packed.words).to_string();
    Ok((packed.words, text))
}

#[cfg(feature = "jt9")]
fn route_jt9(mode: Mode, text: &str) -> Result<Routed, EncodeError> {
    let (words, text) = jt_words(mode, text)?;
    // 12 x 6-bit words -> 72 bits, MSB first (as `jt9::synthesize_standard`).
    let mut info = [0u8; 72];
    for (i, bit) in info.iter_mut().enumerate() {
        *bit = (words[i / 6] >> (5 - (i % 6))) & 1;
    }
    Ok((
        crate::jt9::encode_channel_symbols(&info).to_vec(),
        text,
        Vec::new(),
    ))
}

#[cfg(feature = "jt65")]
fn route_jt65(mode: Mode, text: &str) -> Result<Routed, EncodeError> {
    let (words, text) = jt_words(mode, text)?;
    Ok((
        crate::jt65::encode_channel_symbols(&words).to_vec(),
        text,
        Vec::new(),
    ))
}

#[allow(unused_macros)] // unused when no protocol feature is on
macro_rules! route_one {
    (s77, $ty:ty, $mode:expr, $text:expr, $opts:expr) => {
        route_s77::<$ty>($mode, $text)
    };
    (q65, $ty:ty, $mode:expr, $text:expr, $opts:expr) => {
        route_q65($mode, $text, $opts)
    };
    (fst4w, $ty:ty, $mode:expr, $text:expr, $opts:expr) => {
        route_fst4w::<$ty>($mode, $text)
    };
    (wspr, $ty:ty, $mode:expr, $text:expr, $opts:expr) => {
        route_wspr($mode, $text)
    };
    (jt9, $ty:ty, $mode:expr, $text:expr, $opts:expr) => {
        route_jt9($mode, $text)
    };
    (jt65, $ty:ty, $mode:expr, $text:expr, $opts:expr) => {
        route_jt65($mode, $text)
    };
}

/// One line per [`Mode`], the same list as `AnyDecoder`'s: the feature, the
/// variant, its protocol type and how its text is routed. A new mode is one
/// line here and one there.
macro_rules! encoder_modes {
    ($( $feat:literal $var:ident $ty:ty => $class:ident ),* $(,)?) => {
        fn route(mode: Mode, text: &str, options: &EncodeOptions) -> Result<Routed, EncodeError> {
            let _ = (text, options);
            match mode {
                $( #[cfg(feature = $feat)] Mode::$var => route_one!($class, $ty, mode, text, options), )*
            }
        }

        fn synth(mode: Mode, tones: &[u8], fs: u32, f0_hz: f32, amplitude: f32) -> Vec<f32> {
            let _ = (tones, fs, f0_hz, amplitude);
            match mode {
                $( #[cfg(feature = $feat)] Mode::$var =>
                    crate::engine::tx::synthesize::<$ty>(tones, fs, f0_hz, amplitude), )*
            }
        }

        fn synth_i16(mode: Mode, tones: &[u8], fs: u32, f0_hz: f32, amplitude_i16: i16) -> Vec<i16> {
            let _ = (tones, fs, f0_hz, amplitude_i16);
            match mode {
                $( #[cfg(feature = $feat)] Mode::$var =>
                    crate::engine::tx::synthesize_i16::<$ty>(tones, fs, f0_hz, amplitude_i16), )*
            }
        }

        fn synth_len(mode: Mode, fs: u32) -> usize {
            let _ = fs;
            match mode {
                $( #[cfg(feature = $feat)] Mode::$var => crate::engine::tx::synth_len::<$ty>(fs), )*
            }
        }
    };
}

encoder_modes! {
    "ft8" Ft8 crate::Ft8 => s77,
    "ft4" Ft4 crate::Ft4 => s77,
    "fst4" Fst4S15 crate::fst4::Fst4s15 => s77,
    "fst4" Fst4S30 crate::fst4::Fst4s30 => s77,
    "fst4" Fst4S60 crate::fst4::Fst4s60 => s77,
    "fst4" Fst4S120 crate::fst4::Fst4s120 => s77,
    "fst4" Fst4S300 crate::fst4::Fst4s300 => s77,
    "fst4" Fst4S900 crate::fst4::Fst4s900 => s77,
    "fst4" Fst4S1800 crate::fst4::Fst4s1800 => s77,
    "fst4w" Fst4W120 crate::fst4w::Fst4w120 => fst4w,
    "fst4w" Fst4W300 crate::fst4w::Fst4w300 => fst4w,
    "fst4w" Fst4W900 crate::fst4w::Fst4w900 => fst4w,
    "fst4w" Fst4W1800 crate::fst4w::Fst4w1800 => fst4w,
    "wspr" Wspr crate::Wspr => wspr,
    "jt9" Jt9 crate::Jt9 => jt9,
    "jt65" Jt65 crate::Jt65 => jt65,
    "q65" Q65A15 crate::q65::Q65a15 => q65,
    "q65" Q65A30 crate::q65::Q65a30 => q65,
    "q65" Q65A60 crate::q65::Q65a60 => q65,
    "q65" Q65B60 crate::q65::Q65b60 => q65,
    "q65" Q65C60 crate::q65::Q65c60 => q65,
    "q65" Q65D60 crate::q65::Q65d60 => q65,
    "q65" Q65E60 crate::q65::Q65e60 => q65,
    "q65" Q65D120 crate::q65::Q65d120 => q65,
    "q65" Q65E120 crate::q65::Q65e120 => q65,
    "q65" Q65A300 crate::q65::Q65a300 => q65,
}
