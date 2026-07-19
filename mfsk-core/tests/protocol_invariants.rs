//! Cross-protocol trait-surface invariants.
//!
//! Every protocol that satisfies the [`mfsk_core::Protocol`]
//! super-trait MUST satisfy a small set of consistency invariants
//! between its [`ModulationParams`], [`FrameLayout`], [`FecCodec`]
//! and [`MessageCodec`] associated values. These invariants don't
//! depend on the protocol's *content* — only on the trait surface.
//!
//! Pinning them here:
//!
//! 1. **Proves the trait abstractions are real, not decorative**:
//!    every wired protocol is checked through the *same* generic
//!    function, demonstrating the surface carries enough
//!    information to express end-to-end consistency.
//! 2. **Surfaces silent breakage when adding a new protocol**: a
//!    contributor who implements `Protocol` for a new ZST gets one
//!    line to add here, and any structural inconsistency
//!    (mismatched payload widths, malformed Gray map, etc.) trips
//!    the test rather than waiting for runtime decode bugs.
//! 3. **Documents the trait contract by example**: the assertions
//!    in this file are the source-of-truth for what "a valid Q65
//!    sub-mode looks like at the trait level".
//!
//! ## Feature coverage
//!
//! Each per-protocol test is gated on its protocol feature, so the
//! default `cargo test` (features `ft8`, `ft4`) only exercises FT8
//! and FT4 invariants. Run `cargo test --features full --test
//! protocol_invariants` to validate every wired ZST (the seven WSJT
//! protocols + four uvpacket sub-modes), or pass an individual
//! protocol feature to scope the run. The registry-size and
//! ProtocolId-uniqueness checks adapt to whatever feature
//! combination is active so they're always meaningful.

use mfsk_core::{
    FecCodec, FrameLayout, MessageCodec, ModulationParams, PROTOCOLS, Protocol, ProtocolId,
    ProtocolMeta, SyncMode, by_name,
};

#[cfg(feature = "fst4")]
use mfsk_core::Fst4s60;
#[cfg(feature = "ft4")]
use mfsk_core::Ft4;
#[cfg(feature = "ft8")]
use mfsk_core::Ft8;
#[cfg(feature = "jt9")]
use mfsk_core::Jt9;
#[cfg(feature = "jt65")]
use mfsk_core::Jt65;
#[cfg(feature = "wspr")]
use mfsk_core::Wspr;
#[cfg(feature = "fst4")]
use mfsk_core::fst4::{Fst4s15, Fst4s30, Fst4s120, Fst4s300};

#[cfg(feature = "q65")]
use mfsk_core::q65::{
    Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60, Q65e120,
};

#[cfg(feature = "uvpacket")]
use mfsk_core::{UvExpress, UvRobust, UvStandard, UvUltraRobust};

/// Invariants that depend only on `<P as ModulationParams>`.
fn assert_modulation_invariants<P: ModulationParams>(name: &str) {
    let ntones = P::NTONES;
    let bps = P::BITS_PER_SYMBOL;
    let data_alphabet = 2u32.pow(bps);

    assert!(
        data_alphabet <= ntones,
        "{name}: 2^BITS_PER_SYMBOL ({data_alphabet}) > NTONES ({ntones}). \
         The data alphabet must fit inside the tone set."
    );

    // SYMBOL_DT in seconds and NSPS at 12 kHz must agree to within
    // half a sample (rounding tolerance).
    let nsps_from_dt = (P::SYMBOL_DT * 12_000.0).round() as u32;
    assert_eq!(
        nsps_from_dt,
        P::NSPS,
        "{name}: SYMBOL_DT × 12 000 = {nsps_from_dt} != NSPS = {}",
        P::NSPS
    );

    // Positive-finite physical/DSP constants.
    assert!(
        P::TONE_SPACING_HZ > 0.0,
        "{name}: TONE_SPACING_HZ must be > 0"
    );
    assert!(
        P::TONE_SPACING_HZ.is_finite(),
        "{name}: TONE_SPACING_HZ must be finite"
    );
    assert!(P::NDOWN > 0, "{name}: NDOWN must be > 0");
    assert!(
        P::NSTEP_PER_SYMBOL > 0,
        "{name}: NSTEP_PER_SYMBOL must be > 0"
    );
    assert!(
        P::NFFT_PER_SYMBOL_FACTOR > 0,
        "{name}: NFFT_PER_SYMBOL_FACTOR must be > 0"
    );

    // GFSK_BT == 0 means "plain FSK" (Q65, JT65, WSPR, JT9). FT8/FT4/FST4
    // use a positive BT product. Both regimes are valid; only negative is not.
    assert!(P::GFSK_BT >= 0.0, "{name}: GFSK_BT must be ≥ 0");
    assert!(P::GFSK_HMOD > 0.0, "{name}: GFSK_HMOD must be > 0");

    // GRAY_MAP must cover at least the data alphabet and at most the full
    // tone set. Some protocols (JT9) only map their 2^BPS data tones;
    // others (JT65, Q65) extend to NTONES with identity for the sync slots.
    let glen = P::GRAY_MAP.len();
    assert!(
        glen >= data_alphabet as usize,
        "{name}: GRAY_MAP length ({glen}) < data alphabet ({data_alphabet})"
    );
    assert!(
        glen <= ntones as usize,
        "{name}: GRAY_MAP length ({glen}) > NTONES ({ntones})"
    );

    // Map entries must be valid tone indices and pairwise unique
    // (so the de-Gray inverse is a function).
    let mut seen = vec![false; ntones as usize];
    for &t in P::GRAY_MAP {
        let t = t as u32;
        assert!(
            t < ntones,
            "{name}: GRAY_MAP has out-of-range entry {t} ≥ NTONES {ntones}"
        );
        assert!(
            !seen[t as usize],
            "{name}: GRAY_MAP has duplicate entry {t}"
        );
        seen[t as usize] = true;
    }
}

/// Invariants that depend only on `<P as FrameLayout>`.
fn assert_frame_layout_invariants<P: FrameLayout + ModulationParams>(name: &str) {
    assert_eq!(
        P::N_SYMBOLS,
        P::N_DATA + P::N_SYNC,
        "{name}: N_SYMBOLS = {} != N_DATA + N_SYNC = {} + {}",
        P::N_SYMBOLS,
        P::N_DATA,
        P::N_SYNC,
    );
    assert!(P::T_SLOT_S > 0.0, "{name}: T_SLOT_S must be > 0");
    assert!(P::T_SLOT_S.is_finite(), "{name}: T_SLOT_S must be finite");
    assert!(
        P::TX_START_OFFSET_S >= 0.0,
        "{name}: TX_START_OFFSET_S must be ≥ 0"
    );

    // Sync layout must be self-consistent.
    match P::SYNC_MODE {
        SyncMode::Block(blocks) => {
            let total_sync: u32 = blocks.iter().map(|b| b.pattern.len() as u32).sum();
            assert_eq!(
                total_sync,
                P::N_SYNC,
                "{name}: Σ SyncBlock.pattern.len() = {total_sync} != N_SYNC = {}",
                P::N_SYNC,
            );
            for (i, b) in blocks.iter().enumerate() {
                assert!(
                    b.start_symbol + b.pattern.len() as u32 <= P::N_SYMBOLS,
                    "{name}: SyncBlock #{i} (start={}, len={}) extends past N_SYMBOLS = {}",
                    b.start_symbol,
                    b.pattern.len(),
                    P::N_SYMBOLS,
                );
                // Each pattern entry must be a valid tone index.
                for &tone in b.pattern {
                    assert!(
                        (tone as u32) < P::NTONES,
                        "{name}: SyncBlock #{i} has out-of-range tone {tone} ≥ NTONES {}",
                        P::NTONES,
                    );
                }
            }
        }
        SyncMode::Interleaved {
            sync_bit_pos,
            vector,
        } => {
            assert_eq!(
                vector.len() as u32,
                P::N_SYMBOLS,
                "{name}: interleaved sync vector length ({}) != N_SYMBOLS ({})",
                vector.len(),
                P::N_SYMBOLS,
            );
            assert!(
                (sync_bit_pos as u32) < P::BITS_PER_SYMBOL,
                "{name}: sync_bit_pos {sync_bit_pos} ≥ BITS_PER_SYMBOL {}",
                P::BITS_PER_SYMBOL,
            );
            for (i, &b) in vector.iter().enumerate() {
                assert!(
                    b <= 1,
                    "{name}: interleaved sync vector[{i}] = {b} (must be 0 or 1)"
                );
            }
        }
    }
}

/// Invariants that link the FEC + message + modulation associated types.
fn assert_codec_consistency<P: Protocol>(name: &str) {
    let fec_n = <P::Fec as FecCodec>::N;
    let fec_k = <P::Fec as FecCodec>::K;
    let payload = <P::Msg as MessageCodec>::PAYLOAD_BITS as usize;

    assert!(
        payload > 0,
        "{name}: MessageCodec::PAYLOAD_BITS must be > 0"
    );
    assert!(fec_k > 0, "{name}: FecCodec::K must be > 0");
    assert!(
        fec_n > fec_k,
        "{name}: FecCodec::N must exceed K (have parity)"
    );

    assert!(
        fec_k >= payload,
        "{name}: FecCodec::K ({fec_k}) < PAYLOAD_BITS ({payload}). \
         The FEC info-bit budget must hold at least the message payload."
    );

    // The FEC codeword must fit inside the data symbols of one frame.
    // Equality is the dense case (FT8/FT4/FST4/JT65/Q65); WSPR uses
    // every symbol's LSB for sync so its data bits per symbol is
    // BITS_PER_SYMBOL − 1 — codeword still has to fit.
    let chan_bits = (P::N_DATA * P::BITS_PER_SYMBOL) as usize;
    assert!(
        fec_n <= chan_bits,
        "{name}: FecCodec::N ({fec_n}) > N_DATA × BITS_PER_SYMBOL ({chan_bits}). \
         The FEC codeword must fit in the channel symbols."
    );

    // ID is read at runtime to make sure the const is wired.
    let id: ProtocolId = P::ID;
    let _ = id;
}

/// Single entry point that runs *every* trait-level invariant for one ZST.
/// Used by every per-protocol test below — adding a new wired protocol is
/// a one-line change.
fn assert_protocol_invariants<P: Protocol>(name: &str) {
    assert_modulation_invariants::<P>(name);
    assert_frame_layout_invariants::<P>(name);
    assert_codec_consistency::<P>(name);
}

// ─── Per-protocol invocations of the generic checker ───────────────────

#[cfg(feature = "ft8")]
#[test]
fn ft8_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Ft8>("Ft8");
}

#[cfg(feature = "ft4")]
#[test]
fn ft4_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Ft4>("Ft4");
}

#[cfg(feature = "fst4")]
#[test]
fn fst4s60_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Fst4s60>("Fst4s60");
}

#[cfg(feature = "fst4")]
#[test]
fn fst4_other_submodes_satisfy_protocol_invariants() {
    // The other four T/R-period sub-modes share frame layout, FEC,
    // message format, and GFSK shaping with Fst4s60, so they must
    // satisfy the same invariants — running them through the same
    // generic asserter pins that sub-mode-only changes (NSPS, NDOWN,
    // TX_START_OFFSET_S for FST4-15) don't accidentally break the
    // contract.
    assert_protocol_invariants::<Fst4s15>("Fst4s15");
    assert_protocol_invariants::<Fst4s30>("Fst4s30");
    assert_protocol_invariants::<Fst4s120>("Fst4s120");
    assert_protocol_invariants::<Fst4s300>("Fst4s300");
}

#[cfg(feature = "wspr")]
#[test]
fn wspr_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Wspr>("Wspr");
}

#[cfg(feature = "jt9")]
#[test]
fn jt9_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Jt9>("Jt9");
}

#[cfg(feature = "jt65")]
#[test]
fn jt65_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Jt65>("Jt65");
}

#[cfg(feature = "q65")]
#[test]
fn q65a15_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Q65a15>("Q65a15");
}

#[cfg(feature = "q65")]
#[test]
fn q65a30_satisfies_protocol_invariants() {
    assert_protocol_invariants::<Q65a30>("Q65a30");
}

#[cfg(feature = "q65")]
#[test]
fn q65_eme_submodes_satisfy_protocol_invariants() {
    // All five EME sub-modes share the FEC, message format and frame
    // layout with Q65a30, so they must satisfy the same invariants —
    // running them through the same generic asserter pins that
    // sub-mode-only changes (NSPS, TONE_SPACING_HZ) don't accidentally
    // break the contract.
    assert_protocol_invariants::<Q65a60>("Q65a60");
    assert_protocol_invariants::<Q65b60>("Q65b60");
    assert_protocol_invariants::<Q65c60>("Q65c60");
    assert_protocol_invariants::<Q65d60>("Q65d60");
    assert_protocol_invariants::<Q65e60>("Q65e60");
}

#[cfg(feature = "q65")]
#[test]
fn q65_long_period_scatter_submodes_satisfy_protocol_invariants() {
    // Q65-120D/120E/300A share the same FEC, message format and
    // frame layout too -- only NSPS (T/R period) and TONE_SPACING_HZ
    // differ, same as the EME sub-modes above.
    assert_protocol_invariants::<Q65d120>("Q65d120");
    assert_protocol_invariants::<Q65e120>("Q65e120");
    assert_protocol_invariants::<Q65a300>("Q65a300");
}

#[cfg(feature = "uvpacket")]
#[test]
fn uvpacket_submodes_satisfy_protocol_invariants() {
    // All four uvpacket sub-modes share Ldpc240_101 + UvPacketRawMessage
    // + the same Costas-prefixed-LDPC-block frame layout. Only the
    // (inherent) `MODE` puncture-posture constant differs.
    assert_protocol_invariants::<UvRobust>("UvRobust");
    assert_protocol_invariants::<UvStandard>("UvStandard");
    assert_protocol_invariants::<UvUltraRobust>("UvUltraRobust");
    assert_protocol_invariants::<UvExpress>("UvExpress");
}

// ─── Registry / trait-surface cross-check ─────────────────────────────

/// Generic helper: assert that a [`ProtocolMeta`] entry's fields
/// equal the trait constants of `P`. Reads the trait constants
/// through a *different* code path than the `protocol_meta!` macro
/// in `mfsk_core::registry`, so macro-internal field typos
/// (e.g. accidentally writing `ntones: BITS_PER_SYMBOL`) trip this
/// assertion.
fn assert_meta_matches_trait<P: Protocol>(meta: &ProtocolMeta) {
    assert_eq!(meta.id, P::ID, "registry id mismatch for {}", meta.name);
    assert_eq!(
        meta.ntones,
        P::NTONES,
        "registry ntones mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.bits_per_symbol,
        P::BITS_PER_SYMBOL,
        "registry bits_per_symbol mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.nsps,
        P::NSPS,
        "registry nsps mismatch for {}",
        meta.name
    );
    assert!(
        (meta.symbol_dt - P::SYMBOL_DT).abs() < 1e-7,
        "registry symbol_dt mismatch for {}: {} vs {}",
        meta.name,
        meta.symbol_dt,
        P::SYMBOL_DT
    );
    assert!(
        (meta.tone_spacing_hz - P::TONE_SPACING_HZ).abs() < 1e-3,
        "registry tone_spacing_hz mismatch for {}: {} vs {}",
        meta.name,
        meta.tone_spacing_hz,
        P::TONE_SPACING_HZ
    );
    assert_eq!(
        meta.gfsk_bt,
        P::GFSK_BT,
        "registry gfsk_bt mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.gfsk_hmod,
        P::GFSK_HMOD,
        "registry gfsk_hmod mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.n_data,
        P::N_DATA,
        "registry n_data mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.n_sync,
        P::N_SYNC,
        "registry n_sync mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.n_symbols,
        P::N_SYMBOLS,
        "registry n_symbols mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.t_slot_s,
        P::T_SLOT_S,
        "registry t_slot_s mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.fec_k,
        <P::Fec as FecCodec>::K,
        "registry fec_k mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.fec_n,
        <P::Fec as FecCodec>::N,
        "registry fec_n mismatch for {}",
        meta.name
    );
    assert_eq!(
        meta.payload_bits,
        <P::Msg as MessageCodec>::PAYLOAD_BITS,
        "registry payload_bits mismatch for {}",
        meta.name
    );
}

#[test]
fn registry_entries_match_zst_trait_constants() {
    // Every wired protocol's registry entry is cross-checked against
    // its ZST through `assert_meta_matches_trait`. The lookup
    // happens by name (the user-facing handle), so a name typo or
    // a wrong-type macro arg in `registry.rs` is caught here.
    //
    // After the per-name checks we assert that the *count* of
    // verified entries equals `PROTOCOLS.len()`. Adding a registry
    // entry but forgetting the matching `check!` line trips that
    // count assertion — without it, missing entries would be a
    // silent gap.
    let mut checked = 0usize;
    macro_rules! check {
        ($name:literal, $ty:ty) => {{
            let meta = by_name($name).unwrap_or_else(|| panic!("registry missing entry {}", $name));
            assert_meta_matches_trait::<$ty>(meta);
            checked += 1;
        }};
    }

    #[cfg(feature = "ft8")]
    check!("FT8", Ft8);
    #[cfg(feature = "ft4")]
    check!("FT4", Ft4);
    #[cfg(feature = "fst4")]
    {
        check!("FST4-60A", Fst4s60);
        check!("FST4-15", Fst4s15);
        check!("FST4-30", Fst4s30);
        check!("FST4-120", Fst4s120);
        check!("FST4-300", Fst4s300);
    }
    #[cfg(feature = "wspr")]
    check!("WSPR", Wspr);
    #[cfg(feature = "jt9")]
    check!("JT9", Jt9);
    #[cfg(feature = "jt65")]
    check!("JT65", Jt65);
    #[cfg(feature = "q65")]
    {
        check!("Q65-15A", Q65a15);
        check!("Q65-30A", Q65a30);
        check!("Q65-60A", Q65a60);
        check!("Q65-60B", Q65b60);
        check!("Q65-60C", Q65c60);
        check!("Q65-60D", Q65d60);
        check!("Q65-60E", Q65e60);
        check!("Q65-120D", Q65d120);
        check!("Q65-120E", Q65e120);
        check!("Q65-300A", Q65a300);
    }
    #[cfg(feature = "uvpacket")]
    {
        check!("UvRobust", UvRobust);
        check!("UvStandard", UvStandard);
        check!("UvUltraRobust", UvUltraRobust);
        check!("UvExpress", UvExpress);
    }

    assert_eq!(
        checked,
        PROTOCOLS.len(),
        "registry has {} entries but only {checked} were verified by name; \
         a `check!` line is missing for some PROTOCOLS entry",
        PROTOCOLS.len(),
    );
}

#[test]
fn registry_size_matches_wired_protocols() {
    // The registry must contain exactly one entry per wired ZST.
    // Anyone adding a new ZST who forgets to register it (or
    // registers it twice) trips this test — matching the count
    // documented at the top of `mfsk_core::registry`.
    let mut expected = 0usize;
    #[cfg(feature = "ft8")]
    {
        expected += 1;
    }
    #[cfg(feature = "ft4")]
    {
        expected += 1;
    }
    #[cfg(feature = "fst4")]
    {
        expected += 5;
    }
    #[cfg(feature = "wspr")]
    {
        expected += 1;
    }
    #[cfg(feature = "jt9")]
    {
        expected += 1;
    }
    #[cfg(feature = "jt65")]
    {
        expected += 1;
    }
    #[cfg(feature = "q65")]
    {
        expected += 10;
    }
    #[cfg(feature = "uvpacket")]
    {
        expected += 4;
    }
    assert_eq!(
        PROTOCOLS.len(),
        expected,
        "registry size {} != expected count of wired ZSTs ({expected})",
        PROTOCOLS.len()
    );
}

// ─── ProtocolId coverage check ─────────────────────────────────────────

#[test]
#[allow(clippy::vec_init_then_push)]
fn every_wired_protocol_has_a_unique_protocol_id() {
    // Collect IDs from every wired ZST; dedupe and confirm each one
    // is a known ProtocolId variant. This protects against the
    // common "I forgot to set ID" bug — the const default would
    // be... well, you can't have a default for a `const ID:
    // ProtocolId` so a missing value is a compile error, but a
    // protocol implementer might choose the wrong variant by
    // accident. Listing them here makes the chosen IDs auditable.
    // Each push is gated on its protocol feature, so build the list
    // imperatively rather than as a single literal.
    #[allow(unused_mut)]
    let mut ids: Vec<(&'static str, ProtocolId)> = Vec::new();

    #[cfg(feature = "ft8")]
    ids.push(("Ft8", <Ft8 as Protocol>::ID));
    #[cfg(feature = "ft4")]
    ids.push(("Ft4", <Ft4 as Protocol>::ID));
    #[cfg(feature = "fst4")]
    {
        // Every FST4 sub-mode shares the same ProtocolId::Fst4 —
        // that's by design (the sub-mode is below the FFI
        // granularity), same rationale as Q65.
        ids.push(("Fst4s60", <Fst4s60 as Protocol>::ID));
        ids.push(("Fst4s15", <Fst4s15 as Protocol>::ID));
        ids.push(("Fst4s30", <Fst4s30 as Protocol>::ID));
        ids.push(("Fst4s120", <Fst4s120 as Protocol>::ID));
        ids.push(("Fst4s300", <Fst4s300 as Protocol>::ID));
    }
    #[cfg(feature = "wspr")]
    ids.push(("Wspr", <Wspr as Protocol>::ID));
    #[cfg(feature = "jt9")]
    ids.push(("Jt9", <Jt9 as Protocol>::ID));
    #[cfg(feature = "jt65")]
    ids.push(("Jt65", <Jt65 as Protocol>::ID));
    #[cfg(feature = "q65")]
    {
        // Every Q65 sub-mode shares the same ProtocolId::Q65 — that's
        // by design (the sub-mode is below the FFI granularity).
        ids.push(("Q65a15", <Q65a15 as Protocol>::ID));
        ids.push(("Q65a30", <Q65a30 as Protocol>::ID));
        ids.push(("Q65a60", <Q65a60 as Protocol>::ID));
        ids.push(("Q65b60", <Q65b60 as Protocol>::ID));
        ids.push(("Q65c60", <Q65c60 as Protocol>::ID));
        ids.push(("Q65d60", <Q65d60 as Protocol>::ID));
        ids.push(("Q65e60", <Q65e60 as Protocol>::ID));
        ids.push(("Q65d120", <Q65d120 as Protocol>::ID));
        ids.push(("Q65e120", <Q65e120 as Protocol>::ID));
        ids.push(("Q65a300", <Q65a300 as Protocol>::ID));
    }
    #[cfg(feature = "uvpacket")]
    {
        // All four uvpacket sub-modes share ProtocolId::UvPacket —
        // family-level FFI tag, same rationale as Q65.
        ids.push(("UvRobust", <UvRobust as Protocol>::ID));
        ids.push(("UvStandard", <UvStandard as Protocol>::ID));
        ids.push(("UvUltraRobust", <UvUltraRobust as Protocol>::ID));
        ids.push(("UvExpress", <UvExpress as Protocol>::ID));
    }

    // Distinct protocol families must have distinct IDs.
    let unique: Vec<ProtocolId> = {
        let mut v: Vec<ProtocolId> = ids.iter().map(|(_, id)| *id).collect();
        v.sort_by_key(|id| *id as u8);
        v.dedup();
        v
    };

    // Expected count of distinct ProtocolId families under the active
    // feature combination. Q65 contributes one id (6 ZSTs share it);
    // uvpacket contributes one id (4 ZSTs share it). Every other
    // wired protocol is one family = one id. Computed imperatively
    // mirroring `registry_size_matches_wired_protocols` so the check
    // adapts to any --features combo, not just `--features full`.
    #[allow(unused_mut)]
    let mut expected_distinct = 0usize;
    #[cfg(feature = "ft8")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "ft4")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "fst4")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "wspr")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "jt9")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "jt65")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "q65")]
    {
        expected_distinct += 1;
    }
    #[cfg(feature = "uvpacket")]
    {
        expected_distinct += 1;
    }
    assert_eq!(
        unique.len(),
        expected_distinct,
        "distinct ProtocolId count {} != expected {expected_distinct} \
         under the active feature set; got {:?} from {ids:?}",
        unique.len(),
        unique,
    );

    // Each ID is a known variant (this is a static guarantee from the enum,
    // but match-exhaustively to surface "added a variant but forgot to update
    // this test" right here in CI).
    for (name, id) in &ids {
        match id {
            ProtocolId::Ft8
            | ProtocolId::Ft4
            | ProtocolId::Ft2
            | ProtocolId::Fst4
            | ProtocolId::Jt65
            | ProtocolId::Jt9
            | ProtocolId::Wspr
            | ProtocolId::Q65
            | ProtocolId::UvPacket => {}
        }
        let _ = name;
    }
}
