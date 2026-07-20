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
//! protocol_meta!("Pretty-Name", MyProtocolZst),
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
    ($name:literal, $ty:ty) => {
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
        }
    };
}

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
    protocol_meta!("FT8", crate::Ft8),
    #[cfg(feature = "ft4")]
    protocol_meta!("FT4", crate::Ft4),
    // FST4-60A stays first among the FST4 entries — it's the dominant
    // terrestrial sub-mode and `by_id`/`for_protocol_id` return the
    // *first* matching entry, so this preserves the pre-existing
    // "FST4-60A is the default FST4" behaviour for callers that don't
    // care about sub-mode.
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-60A", crate::Fst4s60),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-15", crate::fst4::Fst4s15),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-30", crate::fst4::Fst4s30),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-120", crate::fst4::Fst4s120),
    #[cfg(feature = "fst4")]
    protocol_meta!("FST4-300", crate::fst4::Fst4s300),
    #[cfg(feature = "wspr")]
    protocol_meta!("WSPR", crate::Wspr),
    #[cfg(feature = "jt9")]
    protocol_meta!("JT9", crate::Jt9),
    #[cfg(feature = "jt65")]
    protocol_meta!("JT65", crate::Jt65),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-15A", crate::q65::Q65a15),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-30A", crate::q65::Q65a30),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60A", crate::q65::Q65a60),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60B", crate::q65::Q65b60),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60C", crate::q65::Q65c60),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60D", crate::q65::Q65d60),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-60E", crate::q65::Q65e60),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-120D", crate::q65::Q65d120),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-120E", crate::q65::Q65e120),
    #[cfg(feature = "q65")]
    protocol_meta!("Q65-300A", crate::q65::Q65a300),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvRobust", crate::UvRobust),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvStandard", crate::UvStandard),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvUltraRobust", crate::UvUltraRobust),
    #[cfg(feature = "uvpacket")]
    protocol_meta!("UvExpress", crate::UvExpress),
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
