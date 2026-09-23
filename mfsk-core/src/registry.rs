// SPDX-License-Identifier: GPL-3.0-or-later
//! Compile-time registry of every protocol mfsk-core builds with.
//!
//! [`PROTOCOLS`] is a `&'static [ProtocolMeta]` populated by an
//! internal macro from each protocol's
//! [`crate::ModulationParams`] / [`crate::FrameLayout`] /
//! [`crate::Protocol`] associated constants. It exists so that
//! consumers (UI layers, FFI bridges, autodetect probes) can
//! enumerate the supported protocols without hardcoding a list of
//! their own.
//!
//! Entries are gated on Cargo features — disabling `q65` removes the
//! ten Q65 entries from the registry, etc. The order is stable but
//! not load-bearing; consume [`PROTOCOLS`] as a set or filter via
//! [`by_id`] / [`by_name`] / [`for_protocol_id`].
//!
//! ## Adding a new protocol
//!
//! After implementing the [`crate::Protocol`] super-trait for your
//! ZST, add one line to the [`PROTOCOLS`] slice using the
//! `protocol_meta!` macro:
//!
//! ```text
//! protocol_meta!("Pretty-Name", MyProtocolZst, MY_PROFILE),
//! ```
//!
//! `tests/protocol_invariants.rs` cross-checks every registry entry
//! against its ZST's trait constants — drift between the macro
//! invocation and the actual trait values trips that test.
//!
//! ## Q65 sub-modes
//!
//! All ten wired Q65 sub-modes (Q65-15A, -30A, -60A‥E, -120D‥E, -300A)
//! appear as distinct registry entries because their `NSPS` /
//! `TONE_SPACING_HZ` / `T_SLOT_S` differ; they share `ProtocolId::Q65`
//! because the FFI protocol tag is family-level. [`by_id`] returns
//! *all* entries sharing a given id, so a Q65 lookup yields ten
//! metadata records.
//!
//! ## FST4 sub-modes
//!
//! Same story as Q65: all five wired FST4 T/R-period sub-modes
//! (FST4-15, -30, -60A, -120, -300) share `ProtocolId::Fst4`.
//! FST4-60A is listed first (ahead of FST4-15, even though 15 < 60)
//! so [`for_protocol_id`]`(ProtocolId::Fst4)` keeps returning the
//! dominant terrestrial sub-mode as the default — this is one of the
//! rare spots where entry order *is* load-bearing.

// These imports look unused when *every* protocol feature is off
// (the `protocol_meta!` invocations that consume them all gate on a
// feature). Suppress the lint so `--no-default-features` builds stay
// clean under `-D warnings`.
#[allow(unused_imports)]
use crate::{FecCodec, FrameLayout, MessageCodec, ModulationParams, Protocol};

use crate::ProtocolId;

/// What a protocol's decode API can actually be asked to do.
///
/// The registry has always described *geometry* — tones, symbol rate,
/// FEC widths. It has never described **capability**, and a consumer
/// that cannot see the difference between "this build has FST4-120" and
/// "FST4 has no SIC at all" ends up hardcoding the matrix. The C ABI
/// needs to publish it, and hardcoding it there would guarantee it
/// drifts, so it is declared here beside the geometry and cross-checked
/// against the real trait impls by `tests/registry_caps.rs` — in both
/// directions: a bit claimed without the trait fails, and a trait
/// implemented without the bit fails too.
///
/// A plain `u32` of bits rather than a `bitflags` dependency: this has
/// to cross a C boundary unchanged, and `no_std` builds carry it too.
pub mod caps {
    /// Drives the `DecodeRequest`/`SniperRequest` builder pair (the
    /// `FrameDecodable` trait). Protocols without this bit decode
    /// through their own entry point — Q65 takes search parameters and
    /// reports a start sample rather than a `dt`; WSPR/JT9/JT65 have no
    /// builder at all. They are not lesser, they are shaped differently.
    pub const DECODE_HANDLE: u32 = 1 << 0;
    /// Narrow-band single-target search (`SniperRequest`, gated on the
    /// `SupportsSniper` trait): a ±250 Hz window around a known carrier.
    ///
    /// Not a software convenience for chasing a spotted station. It is
    /// the receive-side half of narrowing the radio's *analogue*
    /// roofing filter — available on the few transceivers that offer a
    /// true analogue roofing filter at these widths — so the audio
    /// reaching the decoder is already band-limited. That is also why
    /// [`EQ_MODE`] sits next to it: the filter's skirt tilts the
    /// passband, and local equalisation is what flattens it again.
    ///
    /// **FT8 only, and that is the design rather than a gap.** The
    /// wide-band path is the main path for every mode in this crate; if
    /// it is not WSJT-X-faithful without a sniper, that is a bug in the
    /// wide-band path, not a reason to reach for this. FT4 is a contest
    /// protocol whose whole premise is working a full band, and FST4
    /// narrows through its own DDC channelizer instead.
    pub const SNIPER: u32 = 1 << 1;
    /// A-priori hint on a *targeted* search — one where the carrier is
    /// already known, so there is a single hypothesis to lock bits for.
    ///
    /// Two different builders carry this: `SniperRequest::ap_hint` on
    /// FT8, and `q65::Q65DecodeRequest::ap_hint` on Q65, whose decode is
    /// inherently targeted (a nominal frequency plus a tolerance) and so
    /// has no wide-band counterpart to offer. Contrast [`AP_WIDEBAND`],
    /// which is the same hint applied to a whole-band search.
    pub const AP_NARROW: u32 = 1 << 2;
    /// A-priori hint on the *wide-band* search (`SupportsWideBandAp`).
    ///
    /// This used to be FT8-only, and the reason was an artefact twice
    /// over. AP lived in a parallel engine (`msg::pipeline_ap`) whose
    /// candidate loop broke out on `if has_ap` — the *presence of a
    /// hint*, not the search width, is what made it single-target — and
    /// whose per-candidate ladder was shallower than the wide-band one
    /// (OSD at depth 2, no depth-3/4 escalation, no Top-K rescue).
    /// Driving a wide-band decode through it returned 4 of the 11
    /// decodes the plain path finds on the FT4 golden, losing the hinted
    /// station itself and returning the same set for a present and an
    /// absent hint: the cost was the ladder, not AP.
    ///
    /// Wide-band AP is therefore a *rung on the shared ladder* —
    /// `process_candidate_basic` takes an AP option — and reaches FT8,
    /// FT4 and every FST4 sub-mode. The parallel engine is gone.
    pub const AP_WIDEBAND: u32 = 1 << 3;
    /// Flat successive-interference cancellation (`SupportsSicRounds`).
    pub const SIC_ROUNDS: u32 = 1 << 4;
    /// Checkpoint-emulation early decode (`SupportsSicEarly`). FT8 only;
    /// WSJT-X has no equivalent checkpoint architecture elsewhere.
    pub const SIC_EARLY: u32 = 1 << 5;
    /// `.osd(bool)` is honoured. Note this is about the *switch*: FT4
    /// and FST4 run OSD by default through the shared pipeline, so the
    /// capability being absent would mean "cannot turn it off", not
    /// "does not have it".
    pub const OSD: u32 = 1 << 6;
    /// `.eq_mode()` reaches the decoder.
    ///
    /// A property of the *input audio*, not of the search: local
    /// equalisation flattens a passband that an analogue filter has
    /// tilted. It therefore matters on both builders — the narrow-band
    /// one because a roofing filter is the reason that path exists, and
    /// the wide-band one because filtered audio can be handed to it
    /// too. FT8's `eq_mode_recovers_bpf_edge_signal` pins exactly that
    /// case: a signal at the band-pass edge that decodes with `Local`
    /// and not with `Off`, through the wide-band SIC engine.
    ///
    /// On flat-spectrum input it can only cost: on the `ft4sim`-
    /// generated FT4 golden, which has no receiver filter at all,
    /// `Local` loses two decodes of fourteen.
    pub const EQ_MODE: u32 = 1 << 7;
    /// `.strictness()` changes an acceptance threshold that the
    /// protocol's non-AP path actually reads. FST4 does not have this:
    /// its OSD hard-error ceiling is bypassed to match WSJT-X's own
    /// `fst4_decode.f90`, which has no such gate.
    pub const STRICTNESS: u32 = 1 << 8;
    /// `.budget()` — the caller-supplied wall-clock predicate.
    pub const BUDGET: u32 = 1 << 9;
    /// `.known()` is honoured as a post-filter: already-decoded
    /// messages are not re-reported, but the work of re-decoding them
    /// is still done.
    pub const KNOWN_FILTER: u32 = 1 << 10;
    /// `.known()` reaches the engine: known signals are subtracted from
    /// the audio, so they stop masking weaker ones. Strictly stronger
    /// than [`KNOWN_FILTER`], and implies it.
    pub const KNOWN_SUBTRACT: u32 = 1 << 11;
    /// `.fft_cache()` round-trips, so a second decode of the *same*
    /// slot skips rebuilding the forward transform.
    pub const FFT_CACHE: u32 = 1 << 12;
    /// `.on_result()` streaming delivery.
    pub const ON_RESULT: u32 = 1 << 13;
    /// A synthesiser exists: message bits in, audio out.
    pub const ENCODE: u32 = 1 << 14;
}

/// How to read a protocol's `sync_min`, because the three scales are
/// not the same number.
///
/// This exists because the single worst trap in the current C ABI is
/// three per-protocol `sync_min` defaults sitting in one function with
/// nothing saying they are incomparable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SyncScale {
    /// Absolute Costas correlation score. Noise has no fixed value, so
    /// a threshold here is empirical. FT8, FST4.
    CostasAbsolute = 0,
    /// The smoothed spectrum is divided by a fitted baseline before
    /// scoring, so **noise sits at ~1.0 by construction** and any
    /// threshold below that admits every peak in the band. FT4 — and
    /// this is why WSJT-X's own `syncmin = 1.2` (`ft4_decode.f90:195`)
    /// is a floor rather than a knob.
    BaselineNormalised = 1,
}

/// The search parameters a caller gets if it does not supply its own.
///
/// There is no default anywhere in the library today — `sync_min`,
/// `max_cand` and the band are positional arguments of
/// `DecodeRequest::new`, so every caller invents them and the C ABI
/// invented three different sets. These are this crate's own
/// host-configuration values, with the provenance recorded per entry.
#[derive(Clone, Copy, Debug)]
pub struct DecodeDefaults {
    pub freq_min_hz: f32,
    pub freq_max_hz: f32,
    pub sync_min: f32,
    pub max_cand: u32,
}

/// Capability and default-search description for one registry entry.
#[derive(Clone, Copy, Debug)]
pub struct DecodeProfile {
    /// Bitwise OR of [`caps`] constants.
    pub caps: u32,
    pub defaults: DecodeDefaults,
    pub sync_scale: SyncScale,
    /// Upper bound the sniper path silently applies to `max_cand`, or
    /// `None` where there is none. FT4 clamps to 15 and says so
    /// nowhere a caller can see (`ft4::decode`).
    pub sniper_max_cand_cap: Option<u32>,
}

/// Compile-time metadata describing one wired protocol.
///
/// Every field is sourced from the trait surface — see the
/// `protocol_meta!` macro in this module's source for the explicit
/// mapping. Field order matches a typical "what does this protocol
/// look like" display: identity → modulation → frame → FEC →
/// payload.
#[derive(Clone, Copy, Debug)]
pub struct ProtocolMeta {
    /// Family-level protocol id used at the FFI boundary. Multiple
    /// `ProtocolMeta` entries may share an id (e.g. all ten Q65
    /// sub-modes are `ProtocolId::Q65`).
    pub id: ProtocolId,
    /// Human-readable name (e.g. `"FT8"`, `"Q65-60D"`). Stable —
    /// safe for logs, UI strings, and as a [`by_name`] key.
    pub name: &'static str,
    /// Number of FSK tones (`ModulationParams::NTONES`).
    pub ntones: u32,
    /// Information bits per modulated symbol.
    pub bits_per_symbol: u32,
    /// Samples per symbol at 12 kHz.
    pub nsps: u32,
    /// Symbol duration in seconds.
    pub symbol_dt: f32,
    /// Tone-to-tone spacing in Hz.
    pub tone_spacing_hz: f32,
    /// Gaussian bandwidth-time product (0 = plain FSK).
    pub gfsk_bt: f32,
    /// FSK modulation index (h).
    pub gfsk_hmod: f32,
    /// Data symbols per frame.
    pub n_data: u32,
    /// Sync symbols per frame (interleaved-sync protocols report 0).
    pub n_sync: u32,
    /// Total channel symbols per frame (`= n_data + n_sync`).
    pub n_symbols: u32,
    /// Nominal slot length in seconds (15 / 7.5 / 30 / 60 / 120).
    pub t_slot_s: f32,
    /// FEC info-bit budget — `FecCodec::K`.
    pub fec_k: usize,
    /// FEC codeword length in bits — `FecCodec::N`.
    pub fec_n: usize,
    /// Message-codec payload width — `MessageCodec::PAYLOAD_BITS`.
    pub payload_bits: u32,
    /// Seconds from the start of the slot buffer to the first frame
    /// symbol — the `dt = 0` reference (`FrameLayout::TX_START_OFFSET_S`).
    /// 0.5 for FT8, FT4, FST4-15 and the Q65 15/30 s periods; 1.0 for
    /// the other FST4 sub-modes and for Q65 from 60 s up. Q65 follows
    /// `nsps`, as upstream does (`q65.f90:130-131`) — this enumeration
    /// omitted Q65 entirely until #399, which is how it went unnoticed
    /// that every Q65 sub-mode was publishing 1.0.
    /// A host that synthesises a slot has to know this and could not
    /// ask for it.
    pub tx_start_offset_s: f32,
    /// Slot length in samples at the 12 kHz working rate — `t_slot_s`
    /// made exact, so a caller sizes a buffer without repeating the
    /// multiply. FT4 90 000, FT8 180 000, FST4-300 3 600 000.
    pub slot_samples_12k: u32,
    /// Length of the forward FFT the decoder takes over the whole slot
    /// (`Protocol::DECODE_FFT1_SIZE`), or 0 for a mode with its own
    /// front end. FT4 92 160, FT8 192 000, **FST4-300 4 194 304** — the
    /// number that makes "one call shape for every mode" wrong as a
    /// memory story, and which a host could not previously ask for.
    pub decode_fft1_size: u32,
    /// What the decode API can be asked to do, and what it defaults to.
    pub profile: DecodeProfile,
}

/// Build a [`ProtocolMeta`] from a `Protocol`-impl ZST `$ty` plus a
/// stable display name. Used internally to populate [`PROTOCOLS`].
///
/// All fields are read out of the trait constants, so any
/// per-protocol divergence between the macro invocation and the
/// type's actual constants is impossible by construction.
#[allow(unused_macros)] // dead under --no-default-features when every
// protocol-feature gate evaluates to false.
macro_rules! protocol_meta {
    ($name:literal, $ty:ty, $profile:expr) => {
        ProtocolMeta {
            id: <$ty as Protocol>::ID,
            name: $name,
            ntones: <$ty as ModulationParams>::NTONES,
            bits_per_symbol: <$ty as ModulationParams>::BITS_PER_SYMBOL,
            nsps: <$ty as ModulationParams>::NSPS,
            symbol_dt: <$ty as ModulationParams>::SYMBOL_DT,
            tone_spacing_hz: <$ty as ModulationParams>::TONE_SPACING_HZ,
            gfsk_bt: <$ty as ModulationParams>::GFSK_BT,
            gfsk_hmod: <$ty as ModulationParams>::GFSK_HMOD,
            n_data: <$ty as FrameLayout>::N_DATA,
            n_sync: <$ty as FrameLayout>::N_SYNC,
            n_symbols: <$ty as FrameLayout>::N_SYMBOLS,
            t_slot_s: <$ty as FrameLayout>::T_SLOT_S,
            fec_k: <<$ty as Protocol>::Fec as FecCodec>::K,
            fec_n: <<$ty as Protocol>::Fec as FecCodec>::N,
            payload_bits: <<$ty as Protocol>::Msg as MessageCodec>::PAYLOAD_BITS,
            tx_start_offset_s: <$ty as FrameLayout>::TX_START_OFFSET_S,
            slot_samples_12k: (<$ty as FrameLayout>::T_SLOT_S * 12_000.0) as u32,
            decode_fft1_size: <$ty as Protocol>::DECODE_FFT1_SIZE,
            profile: $profile,
        }
    };
}

/// FT8. Everything the builder offers.
///
/// Defaults are this crate's own host research configuration — the one
/// `tests/ft8_qso3_full_parity_recall.rs` and `tests/ft8_sweep.rs` drive,
/// which reaches the full 20-entry golden set. Deliberately *not* the
/// C ABI's historical `sync_min = 2.0`, a pre-0.8.0 value no test in the
/// tree uses, nor the embedded ship config (1.0 / 15), which is tuned
/// for an ESP32's power budget rather than a desktop's recall.
///
/// Real `jt9` varies its own threshold by depth (1.6 for `-d1`/`-d2`,
/// 1.3 for `-d3` — `ft8_decode.f90:176-177`), so there is no single
/// upstream number to copy here the way there is for FT4.
#[allow(dead_code)] // unused when the matching protocol feature is off
const FT8_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::DECODE_HANDLE
        | caps::SNIPER
        | caps::AP_NARROW
        | caps::AP_WIDEBAND
        | caps::SIC_ROUNDS
        | caps::SIC_EARLY
        | caps::OSD
        | caps::EQ_MODE
        | caps::STRICTNESS
        | caps::BUDGET
        | caps::KNOWN_FILTER
        | caps::KNOWN_SUBTRACT
        | caps::FFT_CACHE
        | caps::ON_RESULT
        | caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 100.0,
        freq_max_hz: 3000.0,
        sync_min: 0.8,
        max_cand: 60,
    },
    sync_scale: SyncScale::CostasAbsolute,
    sniper_max_cand_cap: None,
};

/// FT4. No checkpoint SIC (no upstream equivalent to port), no
/// wide-band AP (the shared AP engine early-exits after the first hit).
///
/// `sync_min = 1.2` is **WSJT-X's own** (`ft4_decode.f90:195`), and on
/// FT4's baseline-normalised scale it is a floor rather than a
/// preference: noise sits at 1.0 by construction, so anything lower
/// admits every peak in the band. Measured on 560 sweep files, 0.05 vs
/// 1.2 costs 2.6×-42× the candidates for identical recall
/// (`tests/ft4_candidate_budget.rs`). `max_cand = 100` mirrors
/// `getcandidates4.f90`'s `MAXCAND`.
#[allow(dead_code)]
const FT4_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::DECODE_HANDLE
        | caps::AP_WIDEBAND
        | caps::SIC_ROUNDS
        | caps::OSD
        | caps::EQ_MODE
        | caps::STRICTNESS
        | caps::BUDGET
        | caps::KNOWN_FILTER
        | caps::FFT_CACHE
        | caps::ON_RESULT
        | caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 300.0,
        freq_max_hz: 2700.0,
        sync_min: 1.2,
        max_cand: 100,
    },
    sync_scale: SyncScale::BaselineNormalised,
    sniper_max_cand_cap: Some(15),
};

/// Every FST4 sub-mode. No SIC of either kind — WSJT-X's own
/// `fst4_decode.f90` has no subtract path at all, because FST4 targets
/// point-to-point links rather than crowded shared bands. No
/// `STRICTNESS` either: the OSD hard-error ceiling is deliberately
/// bypassed for FST4 to match upstream, whose only acceptance test is
/// the CRC-24 (`engine::pipeline`).
///
/// `0.8 / 50` are the values this crate's own FST4 tests and the
/// embedded wideband monitor both call the production configuration.
#[allow(dead_code)]
const FST4_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::DECODE_HANDLE
        | caps::AP_WIDEBAND
        | caps::OSD
        | caps::EQ_MODE
        | caps::BUDGET
        | caps::KNOWN_FILTER
        | caps::FFT_CACHE
        | caps::ON_RESULT
        | caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 100.0,
        freq_max_hz: 3000.0,
        sync_min: 0.8,
        max_cand: 50,
    },
    sync_scale: SyncScale::CostasAbsolute,
    sniper_max_cand_cap: None,
};

/// Q65. Not `FrameDecodable`: its own builder family takes search
/// parameters with a time tolerance and a nominal start-sample anchor,
/// and reports `start_sample` rather than a `dt`. It has streaming and
/// a callsign hash table, and neither `known` nor a budget.
///
/// The defaults mirror what `mfsk-ffi`'s Q65 family already hardcodes.
#[allow(dead_code)]
const Q65_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::AP_NARROW | caps::ON_RESULT | caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 200.0,
        freq_max_hz: 3000.0,
        sync_min: 0.05,
        max_cand: 32,
    },
    sync_scale: SyncScale::CostasAbsolute,
    sniper_max_cand_cap: None,
};

/// WSPR. A whole-slot scan with its own subtract pass; no builder, no
/// candidate budget a caller can set.
#[allow(dead_code)]
const WSPR_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::ON_RESULT | caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 1400.0,
        freq_max_hz: 1600.0,
        sync_min: 0.0,
        max_cand: 0,
    },
    sync_scale: SyncScale::CostasAbsolute,
    sniper_max_cand_cap: None,
};

/// JT9 and JT65. Fixed-carrier, fixed-alignment `decode_at` — there is
/// no search to configure, which is why the defaults are the nominal
/// carrier rather than a band.
#[allow(dead_code)]
const JT_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::ON_RESULT | caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 0.0,
        freq_max_hz: 0.0,
        sync_min: 0.0,
        max_cand: 0,
    },
    sync_scale: SyncScale::CostasAbsolute,
    sniper_max_cand_cap: None,
};

/// uvpacket sub-modes — TX/RX exist, but none of the WSJT-family
/// search machinery applies.
#[allow(dead_code)]
const UV_PROFILE: DecodeProfile = DecodeProfile {
    caps: caps::ENCODE,
    defaults: DecodeDefaults {
        freq_min_hz: 0.0,
        freq_max_hz: 0.0,
        sync_min: 0.0,
        max_cand: 0,
    },
    sync_scale: SyncScale::CostasAbsolute,
    sniper_max_cand_cap: None,
};

/// Compile-time list of every `Protocol` impl wired into the
/// current build. Indexable, iterable, and safe to `static`-borrow.
///
/// ```
/// # use mfsk_core::PROTOCOLS;
/// // What does this build support?
/// for p in PROTOCOLS {
///     println!("{}: {} tones, {} s slot", p.name, p.ntones, p.t_slot_s);
/// }
/// ```
pub static PROTOCOLS: &[ProtocolMeta] = &[
    #[cfg(feature = "ft8")]
    protocol_meta!("FT8", crate::Ft8, FT8_PROFILE),
    #[cfg(feature = "ft4")]
    protocol_meta!("FT4", crate::Ft4, FT4_PROFILE),
    // FST4-60A stays first among the FST4 entries — it's the dominant
    // terrestrial sub-mode and `by_id`/`for_protocol_id` return the
    // *first* matching entry, so this preserves the pre-existing
    // "FST4-60A is the default FST4" behaviour for callers that don't
    // care about sub-mode.
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-60A", crate::Fst4s60, FST4_PROFILE),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-15", crate::fst4::Fst4s15, FST4_PROFILE),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-30", crate::fst4::Fst4s30, FST4_PROFILE),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-120", crate::fst4::Fst4s120, FST4_PROFILE),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-300", crate::fst4::Fst4s300, FST4_PROFILE),
    #[cfg(feature = "wspr")]
    protocol_meta!("WSPR", crate::Wspr, WSPR_PROFILE),
    #[cfg(feature = "jt9")]
    protocol_meta!("JT9", crate::Jt9, JT_PROFILE),
    #[cfg(feature = "jt65")]
    protocol_meta!("JT65", crate::Jt65, JT_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-15A", crate::q65::Q65a15, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-30A", crate::q65::Q65a30, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60A", crate::q65::Q65a60, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60B", crate::q65::Q65b60, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60C", crate::q65::Q65c60, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60D", crate::q65::Q65d60, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60E", crate::q65::Q65e60, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-120D", crate::q65::Q65d120, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-120E", crate::q65::Q65e120, Q65_PROFILE),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-300A", crate::q65::Q65a300, Q65_PROFILE),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvRobust", crate::UvRobust, UV_PROFILE),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvStandard", crate::UvStandard, UV_PROFILE),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvUltraRobust", crate::UvUltraRobust, UV_PROFILE),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvExpress", crate::UvExpress, UV_PROFILE),
];

/// Iterator over every registry entry sharing `id`. For most
/// protocols this yields exactly one entry; Q65 yields ten (one per
/// sub-mode).
pub fn by_id(id: ProtocolId) -> impl Iterator<Item = &'static ProtocolMeta> {
    PROTOCOLS.iter().filter(move |p| p.id == id)
}

/// Look up a single protocol by its display name (case-sensitive).
/// Returns `None` if no entry matches — useful for parsing CLI flags
/// or config files.
pub fn by_name(name: &str) -> Option<&'static ProtocolMeta> {
    PROTOCOLS.iter().find(|p| p.name == name)
}

/// Convenience for the common "single-mode-family" lookup: returns
/// the *first* registry entry with the given `id`, or `None` when
/// the build was compiled without that protocol's feature. For Q65
/// this yields the Q65-30A terrestrial entry; use [`by_id`] when
/// you need every sub-mode.
pub fn for_protocol_id(id: ProtocolId) -> Option<&'static ProtocolMeta> {
    by_id(id).next()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_is_non_empty_in_default_build() {
        // `cargo test` with default features must wire at least one
        // protocol; otherwise the registry is meaningless.
        assert!(!PROTOCOLS.is_empty());
    }

    #[test]
    fn names_are_unique() {
        let mut names: Vec<&str> = PROTOCOLS.iter().map(|p| p.name).collect();
        names.sort_unstable();
        let dedup_len = {
            let mut v = names.clone();
            v.dedup();
            v.len()
        };
        assert_eq!(
            dedup_len,
            names.len(),
            "duplicate protocol names in registry: {names:?}"
        );
    }

    #[test]
    fn by_name_round_trips() {
        for p in PROTOCOLS {
            let q = by_name(p.name).expect("by_name should find every registered name");
            assert!(
                std::ptr::eq(p, q),
                "by_name returned a different entry for {}",
                p.name
            );
        }
    }

    #[test]
    fn by_name_returns_none_for_unknown() {
        assert!(by_name("NotAProtocol-9000").is_none());
    }

    #[test]
    fn by_id_yields_at_least_one_entry_for_each_distinct_id() {
        let mut ids: Vec<ProtocolId> = PROTOCOLS.iter().map(|p| p.id).collect();
        ids.sort_unstable_by_key(|id| *id as u8);
        ids.dedup();
        for id in ids {
            assert!(
                by_id(id).next().is_some(),
                "by_id({id:?}) found no entries despite the id appearing in the registry"
            );
        }
    }

    #[cfg(feature = "q65")]
    #[test]
    fn q65_id_yields_all_ten_submodes() {
        let q65_entries: Vec<&ProtocolMeta> = by_id(ProtocolId::Q65).collect();
        assert_eq!(
            q65_entries.len(),
            10,
            "expected ten Q65 sub-modes in the registry, got {}: {:?}",
            q65_entries.len(),
            q65_entries.iter().map(|p| p.name).collect::<Vec<_>>()
        );
        // Names are the canonical sub-mode labels.
        let names: Vec<&str> = q65_entries.iter().map(|p| p.name).collect();
        for expected in &[
            "Q65-15A", "Q65-30A", "Q65-60A", "Q65-60B", "Q65-60C", "Q65-60D", "Q65-60E",
            "Q65-120D", "Q65-120E", "Q65-300A",
        ] {
            assert!(
                names.contains(expected),
                "Q65 registry missing sub-mode {expected}; have {names:?}"
            );
        }
    }

    #[cfg(feature = "fst4")]
    #[test]
    fn fst4_id_yields_all_five_submodes() {
        let fst4_entries: Vec<&ProtocolMeta> = by_id(ProtocolId::Fst4).collect();
        assert_eq!(
            fst4_entries.len(),
            5,
            "expected five FST4 sub-modes in the registry, got {}: {:?}",
            fst4_entries.len(),
            fst4_entries.iter().map(|p| p.name).collect::<Vec<_>>()
        );
        let names: Vec<&str> = fst4_entries.iter().map(|p| p.name).collect();
        for expected in &["FST4-15", "FST4-30", "FST4-60A", "FST4-120", "FST4-300"] {
            assert!(
                names.contains(expected),
                "FST4 registry missing sub-mode {expected}; have {names:?}"
            );
        }
    }

    #[cfg(feature = "fst4")]
    #[test]
    fn for_protocol_id_defaults_to_fst4_60a() {
        // FST4-60A must stay the default entry (order-dependent — see
        // the "FST4 sub-modes" module doc section) since it's the
        // dominant terrestrial sub-mode and pre-existing callers rely
        // on `for_protocol_id(Fst4)` resolving to it.
        let meta = for_protocol_id(ProtocolId::Fst4).expect("fst4 feature is on");
        assert_eq!(meta.name, "FST4-60A");
    }
}
