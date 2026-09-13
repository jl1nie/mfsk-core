//! C ABI for the rs-ft8n decoder suite.
//!
//! # Overview
//!
//! Exposes FT8 / FT4 / FST4 / WSPR / JT9 / JT65 / Q65 decoders and
//! synthesisers behind a small opaque-handle C API that C++ and
//! Kotlin consumers (Android JNI via a thin shim) can link against.
//! cbindgen generates `include/mfsk.h` on every build; see
//! `examples/cpp_smoke` for a round-trip demo that exercises every
//! protocol through the ABI.
//!
//! Q65 has six sub-modes (Q65-30A for terrestrial, Q65-60A‥60E for
//! the EME band lineup) and four decoder strategies (AWGN Bessel,
//! AP-hint BP, fast-fading metric, AP-list template matching).
//! The simple `mfsk_decoder_new(MFSK_PROTOCOL_Q65A30)` path covers
//! the most common terrestrial Q65 case; the dedicated
//! `mfsk_q65_*` function family exposes every sub-mode and every
//! strategy.
//!
//! Status codes, the decode-depth/strictness/equalisation enums and
//! the mode/row/params types are shared with `mfsk-ffi-ft8` via
//! `mfsk-ffi-abi` (issue #205) — this crate used to define its own
//! `MfskStatus` with colliding numeric codes and a
//! heap-`CString`-per-message result shape.
//!
//! # Memory ownership
//!
//! **Decode results never cross the boundary as an allocation.**
//! [`mfsk_session_decode_i16`] and friends write into an array the
//! caller owns and report how many rows they needed, so there is
//! nothing to free and no way to leak one by unwinding past a free —
//! the thing that makes Kotlin and Swift wrappers fiddly.
//!
//! - [`mfsk_session_open`] / [`mfsk_session_close`]: the decode
//!   session, which owns the callsign hash table, the previous slot's
//!   rows and its own error slot.
//! - Transmit is the same shape: `mfsk_pack77*` → [`mfsk_message_to_tones`]
//!   → [`mfsk_tones_to_i16`]/[`mfsk_tones_to_f32`], each stage writing
//!   into a buffer you sized with [`mfsk_symbol_count`] and
//!   [`mfsk_synth_output_len`]. The `mfsk_encode_*` convenience calls
//!   write PCM into your buffer too.
//! - [`MfskDecode::text`] is a fixed inline buffer, so a row is plain
//!   data you can copy and keep.
//!
//! # Enum-typed arguments cross as `uint32_t`
//!
//! A `#[repr(C)]` fieldless enum is an `int` to C, so a caller can pass
//! a value from a config file, a newer header, or a plain mistake —
//! and reading an out-of-range discriminant *as a Rust enum* is
//! undefined behaviour, not a wrong answer. Every entry point therefore
//! takes the mode (and the Q65 sub-mode and fading model) as `uint32_t`
//! and validates it. C callers still write `MFSK_MODE_FT8`: an unscoped
//! enum constant converts implicitly in both C and C++.
//!
//! `read_params` applies the same rule to the enum and `bool` fields
//! inside [`MfskDecodeParams`].
//!
//! # Thread safety
//!
//! **One [`MfskDecodeSession`] per thread.** This is stricter than the
//! pre-v2 handle's contract, deliberately: that handle carried only a
//! protocol tag, so sharing one across threads happened to work, and
//! this module's own documentation said that any change adding cached
//! state must "tighten this documented contract back to strict
//! one-per-thread". A session caches a callsign hash table it mutates
//! on every decode, so that is now the contract.
//!
//! Concurrent decodes on *separate* sessions are supported and
//! exercised by the C++ driver in `examples/cpp_smoke` on every build.
//! They allocate their own FFT planners and scratch buffers.
//!
//! Errors live on the session ([`mfsk_session_last_error`]) rather than
//! in `thread_local!` storage, because a Kotlin coroutine or a Swift
//! `async` caller legitimately hops threads between checking a status
//! and reading the message, and would otherwise find NULL.
//! [`mfsk_last_error`] remains for the handle-less calls.

use std::ffi::{CStr, CString, c_char, c_int};
use std::os::raw::c_void;
use std::ptr;
use std::slice;

use mfsk_core::ft8::decode as ft8;

pub use mfsk_ffi_abi::{
    MfskDecode, MfskDecodeDefaults, MfskDecodeDepth, MfskDecodeOptions, MfskDecodeParams,
    MfskDecodeSession, MfskEqMode, MfskMode, MfskModeInfo, MfskStatus, MfskStrictness,
    MfskSyncScale,
};
/// Inline capacity of each `MfskDecodeParams` a-priori field.
///
/// A literal here, not a re-export of `mfsk_ffi_abi`'s, for the same
/// reason as the capability bits below: cbindgen writes the *name* into
/// the struct it generates (`char ap_call1[MFSK_AP_FIELD_LEN]`) but
/// cannot emit a `#define` for a constant that lives in a dependency,
/// so the header referenced an undeclared identifier and would not
/// compile. Caught by `tests/header_compile.sh`, which is exactly the
/// check that did not exist before this branch.
pub const MFSK_AP_FIELD_LEN: usize = 16;

/// Capacity of `MfskDecode::text`, including the NUL. See
/// [`MFSK_AP_FIELD_LEN`] for why it is a literal.
pub const MFSK_DECODE_TEXT_LEN: usize = 64;

/// `MfskDecode::flags` bit 0: the text needed the session's callsign
/// hash table to resolve a `<...>` reference.
///
/// A literal here for the same cbindgen reason as the capability bits —
/// left in the dependency it reaches C as an undeclared identifier, so
/// a consumer reading `flags` cannot name the bit. Found by the Kotlin
/// JNI shim failing to compile, which is the first thing in this repo
/// to read that field from C.
pub const MFSK_DECODE_FLAG_HASH_RESOLVED: u8 = 1 << 0;

// ──────────────────────────────────────────────────────────────────────────
// Capability bits
//
// These mirror `mfsk_core::registry::caps` and are written here as
// literals rather than re-exported from it, because cbindgen emits a
// root-level literal `pub const` from *this* crate as a `#define` and
// cannot evaluate one that references another crate's path (verified:
// `pub const X: u64 = mfsk_ffi_abi::caps::AP_WIDEBAND;` produces
// nothing). Without them in the header a C caller gets `caps` as a bare
// `uint64_t` and re-derives every bit position by hand — the exact
// failure this surface exists to end.
//
// `tests/mode_introspection.rs::abi_caps_match_the_registry` is what
// keeps the copy honest; it compares each one against the registry
// constant it mirrors, and mfsk-core's own `registry_caps.rs` ties
// those to the trait impls in both directions.
// ──────────────────────────────────────────────────────────────────────────

/// Drives the `DecodeRequest` builder, i.e. `mfsk_decode_i16` and
/// friends apply. Modes without this bit decode through their own
/// entry point (Q65 takes a nominal start sample and a tolerance;
/// WSPR/JT9/JT65 have no builder at all). They are not lesser, they
/// are shaped differently — this is the bit that says which is
/// which.
pub const MFSK_CAP_DECODE_HANDLE: u64 = 1 << 0;
/// Narrow-band single-target search. **FT8 only, by design**: it is
/// the receive-side half of narrowing a transceiver's *analogue*
/// roofing filter, not a general "hunt one known station" feature.
pub const MFSK_CAP_SNIPER: u64 = 1 << 1;
/// A-priori hint on a targeted search (FT8's sniper, Q65's decode).
pub const MFSK_CAP_AP_NARROW: u64 = 1 << 2;
/// A-priori hint on the wide-band search. FT8, FT4 and every FST4
/// sub-mode.
pub const MFSK_CAP_AP_WIDEBAND: u64 = 1 << 3;
/// Flat successive-interference cancellation.
pub const MFSK_CAP_SIC_ROUNDS: u64 = 1 << 4;
/// Checkpoint-emulation early decode. FT8 only.
pub const MFSK_CAP_SIC_EARLY: u64 = 1 << 5;
/// The OSD *switch* is honoured. Absent means "cannot be turned
/// off", not "does not have it" — FT4 and FST4 run OSD by default.
pub const MFSK_CAP_OSD: u64 = 1 << 6;
/// Equalisation mode reaches the decoder.
pub const MFSK_CAP_EQ_MODE: u64 = 1 << 7;
/// The strictness profile is honoured rather than accepted and
/// dropped.
pub const MFSK_CAP_STRICTNESS: u64 = 1 << 8;
/// A caller-supplied budget predicate is polled.
pub const MFSK_CAP_BUDGET: u64 = 1 << 9;
/// Known signals can be excluded from the reported results.
pub const MFSK_CAP_KNOWN_FILTER: u64 = 1 << 10;
/// Known signals are subtracted from the audio, not merely filtered
/// out of the output. Strictly stronger than [`MFSK_CAP_KNOWN_FILTER`].
pub const MFSK_CAP_KNOWN_SUBTRACT: u64 = 1 << 11;
/// A slot FFT can be handed back for a second pass over the same
/// audio.
pub const MFSK_CAP_FFT_CACHE: u64 = 1 << 12;
/// Results can be delivered through a callback as they are found.
pub const MFSK_CAP_ON_RESULT: u64 = 1 << 13;
/// The mode can synthesise as well as decode.
pub const MFSK_CAP_ENCODE: u64 = 1 << 14;

// ──────────────────────────────────────────────────────────────────────────
// Public C types
// ──────────────────────────────────────────────────────────────────────────

/// Q65 sub-mode selector for the dedicated `mfsk_q65_*` function
/// family. All sub-modes share the same FEC, sync layout and
/// message format — only the T/R period and tone spacing change.
///
/// Picked from the type-level `Q65a30 / Q65a60 / Q65b60 / Q65c60 /
/// Q65d60 / Q65e60` ZSTs in `mfsk_core::q65`.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MfskQ65SubMode {
    /// Q65-30A — 30 s slot, ×1 spacing. Terrestrial weak-signal
    /// HF/VHF and ionoscatter; the most common Q65 sub-mode.
    A30 = 0,
    /// Q65-60A — 60 s slot, ×1 spacing. 6 m EME.
    A60 = 1,
    /// Q65-60B — 60 s slot, ×2 spacing. 70 cm / 23 cm EME.
    B60 = 2,
    /// Q65-60C — 60 s slot, ×4 spacing. ~3 GHz microwave EME.
    C60 = 3,
    /// Q65-60D — 60 s slot, ×8 spacing. 5.7 / 10 GHz EME (libration
    /// spread requires the fast-fading metric).
    D60 = 4,
    /// Q65-60E — 60 s slot, ×16 spacing. 24 GHz+ / extreme spread.
    E60 = 5,
    /// Q65-15A — 15 s slot, ×1 spacing. Fastest wired Q65 sub-mode;
    /// stable terrestrial HF/VHF paths that prefer a shorter T/R
    /// period over Q65-30A's extra sensitivity margin. Appended
    /// after `E60` (rather than inserted before `A30`) to keep the
    /// existing discriminant values stable for this `#[repr(C)]` ABI.
    A15 = 6,
    /// Q65-120D — 120 s slot, ×8 spacing. 10 GHz rainscatter/
    /// troposcatter.
    D120 = 7,
    /// Q65-120E — 120 s slot, ×16 spacing. 6 m ionoscatter with
    /// wider Doppler than Q65-30A/60A comfortably tolerate.
    E120 = 8,
    /// Q65-300A — 300 s slot, ×1 spacing. The deepest wired Q65
    /// sub-mode (~-34 dB AWGN threshold); optical (laser) scatter.
    A300 = 9,
}

/// Channel-spread fading model used by `mfsk_q65_decode_fading`.
/// Matches the Gaussian / Lorentzian calibration tables shipped
/// with WSJT-X.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MfskQ65FadingModel {
    /// Gaussian-spread channel — fits libration-limited EME and
    /// most AWGN-with-jitter scenarios.
    Gaussian = 0,
    /// Lorentzian-spread channel — heavier tails; fits some
    /// ionoscatter / meteor-burst signatures.
    Lorentzian = 1,
}

// ──────────────────────────────────────────────────────────────────────────
// Error handling (thread-local last message)
// ──────────────────────────────────────────────────────────────────────────

std::thread_local! {
    static LAST_ERROR: std::cell::RefCell<Option<CString>> = const { std::cell::RefCell::new(None) };
}

fn set_error(msg: impl Into<String>) {
    let s = msg.into();
    LAST_ERROR.with(|e| {
        *e.borrow_mut() = CString::new(s).ok();
    });
}

/// Returns a pointer to the thread-local last-error string, or NULL if
/// no error has been recorded on this thread. The pointer is valid until
/// the next fallible call on this thread.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_last_error() -> *const c_char {
    LAST_ERROR.with(|e| {
        e.borrow()
            .as_ref()
            .map(|s| s.as_ptr())
            .unwrap_or(ptr::null())
    })
}

// ──────────────────────────────────────────────────────────────────────────
// Handle lifecycle
// ──────────────────────────────────────────────────────────────────────────

// ──────────────────────────────────────────────────────────────────────────
// Decode options (opaque handle, issue #205)
// ──────────────────────────────────────────────────────────────────────────

fn map_osd(d: MfskDecodeDepth) -> bool {
    matches!(d, MfskDecodeDepth::BpAllOsd)
}

fn map_strictness(s: MfskStrictness) -> mfsk_core::engine::pipeline::DecodeStrictness {
    use mfsk_core::engine::pipeline::DecodeStrictness as S;
    match s {
        MfskStrictness::Strict => S::Strict,
        MfskStrictness::Normal => S::Normal,
        MfskStrictness::Deep => S::Deep,
    }
}

fn map_eq_mode(e: MfskEqMode) -> mfsk_core::engine::equalize::EqMode {
    use mfsk_core::engine::equalize::EqMode as E;
    match e {
        MfskEqMode::Off => E::Off,
        MfskEqMode::Local => E::Local,
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Decode entry points
// ──────────────────────────────────────────────────────────────────────────

// ──────────────────────────────────────────────────────────────────────────
// Streaming decode (issue #246 follow-up: `.on_result()` was never
// exposed across this FFI — a real gap, not a deliberate omission)
// ──────────────────────────────────────────────────────────────────────────

/// Wraps a C `user_data` pointer to make it `Sync`, which
/// `DecodeRequest::on_result`'s callback bound (`Fn(&DecodeResult) +
/// Sync`) requires since the parallel strategy calls it from multiple
/// rayon worker threads. Sound because this crate never dereferences
/// the pointer itself — it passes straight through to the caller's C
/// callback, whose thread-safety is the caller's own responsibility
/// (documented on [`MfskResultCallback`]).
struct SyncUserData(*mut c_void);
unsafe impl Sync for SyncUserData {}
unsafe impl Send for SyncUserData {}

impl SyncUserData {
    /// Accessor rather than a direct `.0` field read at the call site:
    /// edition-2021 disjoint closure captures would otherwise capture
    /// the bare `*mut c_void` field itself (not `Sync`) instead of the
    /// whole `SyncUserData` wrapper, silently defeating the `unsafe
    /// impl Sync` above at the closure-creation site below.
    fn ptr(&self) -> *mut c_void {
        self.0
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Q65 callsign hash table (opaque handle, issue #250)
// ──────────────────────────────────────────────────────────────────────────

/// Opaque callsign hash-table handle. Resolves `<...>` Type-4
/// hashed-callsign placeholders (WSJT-X's compact encoding for a
/// non-standard call paired with a standard one) in Q65 decode output.
///
/// Construct with [`mfsk_callsign_hash_table_new`], populate with
/// [`mfsk_callsign_hash_table_insert`] as real callsigns become known
/// (e.g. from earlier decodes, a station log, or any other source the
/// caller trusts), then pass into any `mfsk_q65_decode_*` function's
/// `hash_table` parameter. NULL there (the pre-#250 behaviour) leaves
/// hashed callsigns unresolved as literal `<...>` text — nothing else
/// about decode success or timing changes; this only affects how the
/// final message *text* renders. Free with
/// [`mfsk_callsign_hash_table_free`].
///
/// Mirrors `mfsk_core::q65::decode_request::DecodeRequest::hash_table`
/// (`Arc<CallsignHashTable>`) — deliberately not folded into
/// [`MfskDecodeOptions`], which Q65's own function family never uses.
/// Emitted as an incomplete type (`struct X;`) rather than a struct with a
/// zero-length array member: `uint8_t _priv[0]` is a GCC/Clang extension
/// that ISO C rejects (`-Werror=pedantic`), and MSVC accepts only under a
/// warning. A pointer to an incomplete type is exactly as opaque, is
/// standard in both C and C++, and is what every consumer already treats
/// this as. Binary-compatible: the handle only ever crosses as a pointer.
pub struct MfskCallsignHashTable {
    _marker: core::marker::PhantomData<*mut ()>,
}

fn hash_table_inner(
    ht: *const MfskCallsignHashTable,
) -> Option<&'static mfsk_core::msg::hash_table::CallsignHashTable> {
    unsafe { (ht as *const mfsk_core::msg::hash_table::CallsignHashTable).as_ref() }
}

fn hash_table_inner_mut(
    ht: *mut MfskCallsignHashTable,
) -> Option<&'static mut mfsk_core::msg::hash_table::CallsignHashTable> {
    unsafe { (ht as *mut mfsk_core::msg::hash_table::CallsignHashTable).as_mut() }
}

/// Construct an empty callsign hash table. Free with
/// [`mfsk_callsign_hash_table_free`].
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_callsign_hash_table_new() -> *mut MfskCallsignHashTable {
    let inner = Box::new(mfsk_core::msg::hash_table::CallsignHashTable::new());
    Box::into_raw(inner) as *mut MfskCallsignHashTable
}

/// Free a handle from [`mfsk_callsign_hash_table_new`]. NULL is a no-op.
///
/// # Safety
/// `ht` must be a pointer previously returned by
/// [`mfsk_callsign_hash_table_new`], or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_callsign_hash_table_free(ht: *mut MfskCallsignHashTable) {
    if !ht.is_null() {
        drop(unsafe { Box::from_raw(ht as *mut mfsk_core::msg::hash_table::CallsignHashTable) });
    }
}

/// Register a known callsign so a later `<...>` hashed placeholder
/// that matches it resolves to the real call. Mirrors
/// `CallsignHashTable::insert` exactly, including its documented skip
/// rules (empty strings, `<...>` itself, strings under 2 characters,
/// and `CQ`-prefixed calls are all silently ignored — not an error).
///
/// # Safety
/// `ht` must be a live handle from [`mfsk_callsign_hash_table_new`].
/// `call` must be a NUL-terminated UTF-8 string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_callsign_hash_table_insert(
    ht: *mut MfskCallsignHashTable,
    call: *const c_char,
) -> MfskStatus {
    let Ok(s) = cstr_to_str(call) else {
        return MfskStatus::InvalidArg;
    };
    let Some(inner) = hash_table_inner_mut(ht) else {
        set_error("mfsk_callsign_hash_table_insert: null handle");
        return MfskStatus::InvalidArg;
    };
    inner.insert(s);
    MfskStatus::Ok
}

// ──────────────────────────────────────────────────────────────────────────
// Q65 helpers (sub-mode dispatch + decoded-message push)
// ──────────────────────────────────────────────────────────────────────────

/// Wide search params used by every `mfsk_q65_*` decode entry point —
/// matches the Rust-side defaults that work across both terrestrial
/// Q65-30A and EME 60A‥E recordings.
fn q65_default_search(submode: MfskQ65SubMode) -> mfsk_core::q65::SearchParams {
    // Was a flat `time_tolerance_symbols: 50` before issue #282 moved
    // the field to seconds. Symbols are sub-mode-dependent, so the
    // conversion is per sub-mode (50 × that sub-mode's symbol length)
    // to keep every FFI entry point's effective window byte-identical
    // to what it searched before. This is a deliberately *wide*
    // scan — much wider than `SearchParams::default()`'s WSJT-X-parity
    // ±1.0 s — because these entry points take no alignment hint and
    // are documented to work on unaligned EME recordings.
    let symbol_dt = match submode {
        MfskQ65SubMode::A15 => 0.15,
        MfskQ65SubMode::A30 => 0.30,
        MfskQ65SubMode::D120 | MfskQ65SubMode::E120 => 16_000.0 / 12_000.0,
        MfskQ65SubMode::A300 => 41_472.0 / 12_000.0,
        _ => 0.60,
    };
    mfsk_core::q65::SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 50.0,
        time_tolerance_late_sec: 50.0 * symbol_dt,
        score_threshold: 0.05,
        max_candidates: 32,
    }
}

/// Slot midpoint sample index for a sub-mode (used as the nominal
/// search anchor for sub-modes other than Q65-30A).
fn q65_nominal_mid(submode: MfskQ65SubMode) -> usize {
    let slot_s = match submode {
        MfskQ65SubMode::A15 => 15,
        MfskQ65SubMode::A30 => 30,
        MfskQ65SubMode::D120 | MfskQ65SubMode::E120 => 120,
        MfskQ65SubMode::A300 => 300,
        _ => 60,
    };
    12_000 * slot_s / 2
}

/// Plain-AWGN sub-mode-aware scan. Dispatches at runtime to the right
/// `DecodeRequest::<Q65*>` instantiation in `mfsk_core::q65`.
///
/// `hash_table`, when `Some`, resolves `<...>` Type-4 hashed-callsign
/// placeholders in the output message text (issue #250) — a clone of
/// the caller's table is wrapped in a fresh `Arc` per call, matching
/// `DecodeRequest::hash_table`'s own `Arc<CallsignHashTable>` shape.
fn q65_scan_for(
    submode: MfskQ65SubMode,
    audio: &[f32],
    hash_table: Option<&mfsk_core::msg::hash_table::CallsignHashTable>,
) -> Vec<mfsk_core::q65::Q65Result> {
    use mfsk_core::q65::{
        DecodeRequest, Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60,
        Q65e120,
    };
    use std::sync::Arc;
    let params = q65_default_search(submode);
    let mid = q65_nominal_mid(submode);
    macro_rules! scan {
        ($p:ty) => {{
            let mut req = DecodeRequest::<$p>::new(audio, 12_000, mid, params);
            if let Some(ht) = hash_table {
                req = req.hash_table(Arc::new(ht.clone()));
            }
            req.decode()
        }};
    }
    match submode {
        MfskQ65SubMode::A15 => scan!(Q65a15),
        MfskQ65SubMode::A30 => scan!(Q65a30),
        MfskQ65SubMode::A60 => scan!(Q65a60),
        MfskQ65SubMode::B60 => scan!(Q65b60),
        MfskQ65SubMode::C60 => scan!(Q65c60),
        MfskQ65SubMode::D60 => scan!(Q65d60),
        MfskQ65SubMode::E60 => scan!(Q65e60),
        MfskQ65SubMode::D120 => scan!(Q65d120),
        MfskQ65SubMode::E120 => scan!(Q65e120),
        MfskQ65SubMode::A300 => scan!(Q65a300),
    }
}

/// See [`q65_scan_for`]'s `hash_table` doc — same convention here.
fn q65_scan_with_ap_for(
    submode: MfskQ65SubMode,
    audio: &[f32],
    hint: &mfsk_core::msg::ApHint,
    hash_table: Option<&mfsk_core::msg::hash_table::CallsignHashTable>,
) -> Vec<mfsk_core::q65::Q65Result> {
    use mfsk_core::q65::{
        DecodeRequest, Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60,
        Q65e120,
    };
    use std::sync::Arc;
    let params = q65_default_search(submode);
    let mid = q65_nominal_mid(submode);
    macro_rules! scan {
        ($p:ty) => {{
            let mut req = DecodeRequest::<$p>::new(audio, 12_000, mid, params).ap_hint(hint);
            if let Some(ht) = hash_table {
                req = req.hash_table(Arc::new(ht.clone()));
            }
            req.decode()
        }};
    }
    match submode {
        MfskQ65SubMode::A15 => scan!(Q65a15),
        MfskQ65SubMode::A30 => scan!(Q65a30),
        MfskQ65SubMode::A60 => scan!(Q65a60),
        MfskQ65SubMode::B60 => scan!(Q65b60),
        MfskQ65SubMode::C60 => scan!(Q65c60),
        MfskQ65SubMode::D60 => scan!(Q65d60),
        MfskQ65SubMode::E60 => scan!(Q65e60),
        MfskQ65SubMode::D120 => scan!(Q65d120),
        MfskQ65SubMode::E120 => scan!(Q65e120),
        MfskQ65SubMode::A300 => scan!(Q65a300),
    }
}

/// See [`q65_scan_for`]'s `hash_table` doc — same convention here.
fn q65_scan_fading_for(
    submode: MfskQ65SubMode,
    audio: &[f32],
    b90_ts: f32,
    model: mfsk_core::fec::qra::FadingModel,
    hash_table: Option<&mfsk_core::msg::hash_table::CallsignHashTable>,
) -> Vec<mfsk_core::q65::Q65Result> {
    use mfsk_core::q65::{
        DecodeRequest, Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60,
        Q65e120,
    };
    use std::sync::Arc;
    let params = q65_default_search(submode);
    let mid = q65_nominal_mid(submode);
    macro_rules! scan {
        ($p:ty) => {{
            let mut req =
                DecodeRequest::<$p>::new(audio, 12_000, mid, params).fading(model, b90_ts);
            if let Some(ht) = hash_table {
                req = req.hash_table(Arc::new(ht.clone()));
            }
            req.decode()
        }};
    }
    match submode {
        MfskQ65SubMode::A15 => scan!(Q65a15),
        MfskQ65SubMode::A30 => scan!(Q65a30),
        MfskQ65SubMode::A60 => scan!(Q65a60),
        MfskQ65SubMode::B60 => scan!(Q65b60),
        MfskQ65SubMode::C60 => scan!(Q65c60),
        MfskQ65SubMode::D60 => scan!(Q65d60),
        MfskQ65SubMode::E60 => scan!(Q65e60),
        MfskQ65SubMode::D120 => scan!(Q65d120),
        MfskQ65SubMode::E120 => scan!(Q65e120),
        MfskQ65SubMode::A300 => scan!(Q65a300),
    }
}

/// See [`q65_scan_for`]'s `hash_table` doc — same convention here.
fn q65_scan_with_ap_list_for(
    submode: MfskQ65SubMode,
    audio: &[f32],
    candidates: &[[i32; 63]],
    hash_table: Option<&mfsk_core::msg::hash_table::CallsignHashTable>,
) -> Vec<mfsk_core::q65::Q65Result> {
    use mfsk_core::q65::{
        DecodeRequest, Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60,
        Q65e120,
    };
    use std::sync::Arc;
    let params = q65_default_search(submode);
    let mid = q65_nominal_mid(submode);
    macro_rules! scan {
        ($p:ty) => {{
            let mut req = DecodeRequest::<$p>::new(audio, 12_000, mid, params).ap_list(candidates);
            if let Some(ht) = hash_table {
                req = req.hash_table(Arc::new(ht.clone()));
            }
            req.decode()
        }};
    }
    match submode {
        MfskQ65SubMode::A15 => scan!(Q65a15),
        MfskQ65SubMode::A30 => scan!(Q65a30),
        MfskQ65SubMode::A60 => scan!(Q65a60),
        MfskQ65SubMode::B60 => scan!(Q65b60),
        MfskQ65SubMode::C60 => scan!(Q65c60),
        MfskQ65SubMode::D60 => scan!(Q65d60),
        MfskQ65SubMode::E60 => scan!(Q65e60),
        MfskQ65SubMode::D120 => scan!(Q65d120),
        MfskQ65SubMode::E120 => scan!(Q65e120),
        MfskQ65SubMode::A300 => scan!(Q65a300),
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Encode entry points
// ──────────────────────────────────────────────────────────────────────────

fn cstr_to_str<'a>(p: *const c_char) -> Result<&'a str, MfskStatus> {
    if p.is_null() {
        set_error("null C string");
        return Err(MfskStatus::InvalidArg);
    }
    unsafe {
        CStr::from_ptr(p).to_str().map_err(|e| {
            set_error(format!("invalid UTF-8 in C string: {e}"));
            MfskStatus::InvalidArg
        })
    }
}

/// Build an [`mfsk_core::msg::ApHint`] from up to 4 optional
/// NUL-terminated C strings (call1/call2/grid/report) — each may be
/// NULL (skip) or a valid UTF-8 string (empty also skips, matching
/// `ApHint`'s own builder semantics). Shared by
/// [`mfsk_q65_decode_with_ap`] and
/// [`mfsk_decode_options_set_ap_hint`], which independently
/// duplicated this exact pattern before it was factored out here
/// (issue #162 follow-up).
///
/// # Safety
/// Each non-null pointer must point to a valid NUL-terminated C string.
unsafe fn build_ap_hint_from_cstrs(
    call1: *const c_char,
    call2: *const c_char,
    grid: *const c_char,
    report: *const c_char,
) -> Result<mfsk_core::msg::ApHint, MfskStatus> {
    let mut hint = mfsk_core::msg::ApHint::new();
    let mut maybe_attach = |p: *const c_char,
                            f: fn(mfsk_core::msg::ApHint, &str) -> mfsk_core::msg::ApHint|
     -> Result<(), MfskStatus> {
        if p.is_null() {
            return Ok(());
        }
        let s = cstr_to_str(p)?;
        if !s.is_empty() {
            // Builder consumes by value, so we replace via temporary.
            hint = f(std::mem::take(&mut hint), s);
        }
        Ok(())
    };
    maybe_attach(call1, |h, s| h.with_call1(s))?;
    maybe_attach(call2, |h, s| h.with_call2(s))?;
    maybe_attach(grid, |h, s| h.with_grid(s))?;
    maybe_attach(report, |h, s| h.with_report(s))?;
    Ok(hint)
}

/// Synthesise a standard FT8 message ("CALL1 CALL2 REPORT") at `freq_hz`
/// carrier. Writes 12 kHz f32 PCM into `out`.
///
/// # Safety
///
/// `call1`/`call2`/`report` must be NUL-terminated UTF-8 strings.
/// `out` must be `cap` writable `f32`; `*out_len` receives the
/// sample count (or the count needed, if `cap` was too small).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_ft8(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(rep) = cstr_to_str(report) else {
        return MfskStatus::InvalidArg;
    };
    let Some(msg77) = mfsk_core::msg::wsjt77::pack77(c1, c2, rep) else {
        set_error("FT8 pack77 failed");
        return MfskStatus::InvalidArg;
    };
    let tones = mfsk_core::ft8::wave_gen::message_to_tones(&msg77);
    let pcm = mfsk_core::ft8::wave_gen::tones_to_f32(&tones, freq_hz, 1.0);
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

/// Synthesise a standard FT4 message at `freq_hz`. 12 kHz f32 PCM.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_ft4(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(rep) = cstr_to_str(report) else {
        return MfskStatus::InvalidArg;
    };
    let Some(msg77) = mfsk_core::msg::wsjt77::pack77(c1, c2, rep) else {
        set_error("FT4 pack77 failed");
        return MfskStatus::InvalidArg;
    };
    let tones = mfsk_core::ft4::encode::message_to_tones(&msg77);
    let pcm = mfsk_core::ft4::encode::tones_to_f32(&tones, freq_hz, 1.0);
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

/// Synthesise a standard FST4-60A message at `freq_hz`. 12 kHz f32 PCM.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_fst4s60(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(rep) = cstr_to_str(report) else {
        return MfskStatus::InvalidArg;
    };
    let Some(msg77) = mfsk_core::msg::wsjt77::pack77(c1, c2, rep) else {
        set_error("FST4 pack77 failed");
        return MfskStatus::InvalidArg;
    };
    let tones = mfsk_core::fst4::encode::message_to_tones(&msg77);
    let pcm = mfsk_core::fst4::encode::tones_to_f32(&tones, freq_hz, 1.0);
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

/// Synthesise a Type-1 WSPR message (`call grid power_dbm`).
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_wspr(
    call: *const c_char,
    grid: *const c_char,
    power_dbm: i32,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(g) = cstr_to_str(grid) else {
        return MfskStatus::InvalidArg;
    };
    let Some(pcm) = mfsk_core::wspr::synthesize_type1(c1, g, power_dbm, 12_000, freq_hz, 0.3)
    else {
        set_error("WSPR synth failed (bad call/grid/power)");
        return MfskStatus::InvalidArg;
    };
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

/// Synthesise a standard JT9 message at `freq_hz`.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_jt9(
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(gr) = cstr_to_str(grid_or_report) else {
        return MfskStatus::InvalidArg;
    };
    let Some(pcm) = mfsk_core::jt9::synthesize_standard(c1, c2, gr, 12_000, freq_hz, 0.3) else {
        set_error("JT9 synth failed (bad pack)");
        return MfskStatus::InvalidArg;
    };
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

/// Synthesise a standard JT65 message at `freq_hz`.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_jt65(
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(gr) = cstr_to_str(grid_or_report) else {
        return MfskStatus::InvalidArg;
    };
    let Some(pcm) = mfsk_core::jt65::synthesize_standard(c1, c2, gr, 12_000, freq_hz, 0.3) else {
        set_error("JT65 synth failed (bad pack)");
        return MfskStatus::InvalidArg;
    };
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

/// Synthesise a standard Q65 message at `freq_hz` for the requested
/// sub-mode. 12 kHz f32 PCM. The 30 s vs 60 s slot duration and tone
/// spacing follow the Q65 spec; the FEC and message format are
/// shared across every sub-mode.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_q65(
    submode: u32,
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(submode) = q65_submode_of(submode) else {
        set_error("mfsk_encode_q65: not a Q65 sub-mode");
        return MfskStatus::InvalidArg;
    };
    use mfsk_core::q65::{
        Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60, Q65e120,
        synthesize_standard_for,
    };

    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(gr) = cstr_to_str(grid_or_report) else {
        return MfskStatus::InvalidArg;
    };
    let pcm_opt = match submode {
        MfskQ65SubMode::A15 => synthesize_standard_for::<Q65a15>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::A30 => synthesize_standard_for::<Q65a30>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::A60 => synthesize_standard_for::<Q65a60>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::B60 => synthesize_standard_for::<Q65b60>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::C60 => synthesize_standard_for::<Q65c60>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::D60 => synthesize_standard_for::<Q65d60>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::E60 => synthesize_standard_for::<Q65e60>(c1, c2, gr, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::D120 => {
            synthesize_standard_for::<Q65d120>(c1, c2, gr, 12_000, freq_hz, 0.3)
        }
        MfskQ65SubMode::E120 => {
            synthesize_standard_for::<Q65e120>(c1, c2, gr, 12_000, freq_hz, 0.3)
        }
        MfskQ65SubMode::A300 => {
            synthesize_standard_for::<Q65a300>(c1, c2, gr, 12_000, freq_hz, 0.3)
        }
    };
    let Some(pcm) = pcm_opt else {
        set_error("Q65 synth failed (bad pack)");
        return MfskStatus::InvalidArg;
    };
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
}

// ──────────────────────────────────────────────────────────────────────────
// Q65 decode entry points (4 strategies × 6 sub-modes)
// ──────────────────────────────────────────────────────────────────────────

/// Helper used by the four `mfsk_q65_decode_*` functions to validate
/// their input pointers and lift the audio buffer to a 12 kHz f32
/// slice. Returns `Err(status)` if anything is wrong with the input.
unsafe fn q65_prepare_audio(
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    fn_name: &'static str,
) -> Result<Vec<f32>, MfskStatus> {
    if samples.is_null() {
        set_error(format!("{fn_name}: null buffer pointer"));
        return Err(MfskStatus::InvalidArg);
    }
    let slice_f32 = unsafe { slice::from_raw_parts(samples, n_samples) };
    let audio: Vec<f32> = if sample_rate == 12_000 {
        slice_f32.to_vec()
    } else {
        mfsk_core::engine::dsp::resample::resample_f32_to_12k_f32(slice_f32, sample_rate)
    };
    Ok(audio)
}

/// Plain AWGN Q65 scan-and-decode for any sub-mode. The default
/// strategy — every other `mfsk_q65_decode_*` function trades
/// computational cost or extra inputs for a few dB of threshold gain
/// against this baseline.
///
/// `hash_table` may be NULL (hashed `<...>` callsigns stay
/// unresolved, the pre-#250 behaviour) or a handle from
/// [`mfsk_callsign_hash_table_new`] — see that type's doc comment.
///
/// # Safety
///
/// `samples` must point to `n_samples` valid `f32` values.
/// `hash_table`, if non-NULL, must be a live handle from
/// [`mfsk_callsign_hash_table_new`].
/// `out` must point to at least `cap` writable [`MfskDecode`] rows;
/// `*out_len` receives the number of decodes found.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_decode(
    submode: u32,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    hash_table: *const MfskCallsignHashTable,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let audio =
        match unsafe { q65_prepare_audio(samples, n_samples, sample_rate, "mfsk_q65_decode") } {
            Ok(a) => a,
            Err(s) => return s,
        };
    let Some(submode) = q65_submode_of(submode) else {
        set_error("mfsk_q65_decode: not a Q65 sub-mode");
        return MfskStatus::InvalidArg;
    };
    let rows = q65_rows(
        submode,
        &q65_scan_for(submode, &audio, hash_table_inner(hash_table)),
    );
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

/// AP-hint Q65 scan-and-decode. Up to four optional hints
/// (`call1`, `call2`, `grid`, `report`) — each may be NULL when
/// unknown. Lifts the effective decode threshold by ~2 dB when the
/// supplied hints are correct.
///
/// `hash_table`: see [`mfsk_q65_decode`]'s doc.
///
/// # Safety
///
/// As [`mfsk_q65_decode`]. The four hint strings, when non-NULL,
/// must be NUL-terminated UTF-8.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_decode_with_ap(
    submode: u32,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    ap_call1: *const c_char,
    ap_call2: *const c_char,
    ap_grid: *const c_char,
    ap_report: *const c_char,
    hash_table: *const MfskCallsignHashTable,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let audio = match unsafe {
        q65_prepare_audio(samples, n_samples, sample_rate, "mfsk_q65_decode_with_ap")
    } {
        Ok(a) => a,
        Err(s) => return s,
    };
    let Some(submode) = q65_submode_of(submode) else {
        set_error("mfsk_q65_decode_with_ap: not a Q65 sub-mode");
        return MfskStatus::InvalidArg;
    };
    let out = unsafe { &mut *out };

    let hint = match unsafe { build_ap_hint_from_cstrs(ap_call1, ap_call2, ap_grid, ap_report) } {
        Ok(h) => h,
        Err(st) => return st,
    };

    let ht = hash_table_inner(hash_table);
    let decodes = if hint.has_info() {
        q65_scan_with_ap_for(submode, &audio, &hint, ht)
    } else {
        // Empty hint → fall through to the plain path so callers
        // don't need to special-case it.
        q65_scan_for(submode, &audio, ht)
    };
    let rows = q65_rows(submode, &decodes);
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

/// Fast-fading Q65 scan-and-decode. Recovers the 5–8 dB the AWGN
/// Bessel front end loses on Doppler-spread channels — required for
/// microwave EME at 5.7 GHz / 10 GHz / 24 GHz. `b90_ts` is the
/// spread bandwidth × symbol period (typical: 0.05 = near-AWGN,
/// 1.0 = moderate, 5.0+ = severe). `model` chooses the calibration
/// shape. `hash_table`: see [`mfsk_q65_decode`]'s doc.
///
/// # Safety
///
/// As [`mfsk_q65_decode`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_decode_fading(
    submode: u32,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    b90_ts: f32,
    fading_model: u32,
    hash_table: *const MfskCallsignHashTable,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let audio = match unsafe {
        q65_prepare_audio(samples, n_samples, sample_rate, "mfsk_q65_decode_fading")
    } {
        Ok(a) => a,
        Err(s) => return s,
    };
    let Some(submode) = q65_submode_of(submode) else {
        set_error("mfsk_q65_decode_fading: not a Q65 sub-mode");
        return MfskStatus::InvalidArg;
    };
    let Some(fading_model) = q65_fading_of(fading_model) else {
        set_error("mfsk_q65_decode_fading: not a fading model");
        return MfskStatus::InvalidArg;
    };
    let model = match fading_model {
        MfskQ65FadingModel::Gaussian => mfsk_core::fec::qra::FadingModel::Gaussian,
        MfskQ65FadingModel::Lorentzian => mfsk_core::fec::qra::FadingModel::Lorentzian,
    };
    let decodes = q65_scan_fading_for(submode, &audio, b90_ts, model, hash_table_inner(hash_table));
    let rows = q65_rows(submode, &decodes);
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

/// AP-list (template-matching) Q65 scan-and-decode. Builds the
/// standard 206-codeword candidate set internally from
/// `(my_call, his_call, his_grid)` and picks the matching exchange,
/// if any. `his_grid` may be NULL or empty to skip the two
/// grid-bearing templates. Yields ~3 dB threshold gain over plain
/// BP when the truth is in the candidate set. `hash_table`: see
/// [`mfsk_q65_decode`]'s doc.
///
/// # Safety
///
/// As [`mfsk_q65_decode`]. `my_call` and `his_call` must be
/// NUL-terminated UTF-8 strings; `his_grid` may be NULL or
/// NUL-terminated UTF-8.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_decode_with_ap_list(
    submode: u32,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    my_call: *const c_char,
    his_call: *const c_char,
    his_grid: *const c_char,
    hash_table: *const MfskCallsignHashTable,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let audio = match unsafe {
        q65_prepare_audio(
            samples,
            n_samples,
            sample_rate,
            "mfsk_q65_decode_with_ap_list",
        )
    } {
        Ok(a) => a,
        Err(s) => return s,
    };
    let Some(submode) = q65_submode_of(submode) else {
        set_error("mfsk_q65_decode_with_ap_list: not a Q65 sub-mode");
        return MfskStatus::InvalidArg;
    };
    let Ok(mc) = cstr_to_str(my_call) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(hc) = cstr_to_str(his_call) else {
        return MfskStatus::InvalidArg;
    };
    let hg = if his_grid.is_null() {
        ""
    } else {
        match cstr_to_str(his_grid) {
            Ok(s) => s,
            Err(st) => return st,
        }
    };

    let candidates = mfsk_core::q65::standard_qso_codewords(mc, hc, hg);
    if candidates.is_empty() {
        set_error("mfsk_q65_decode_with_ap_list: candidate set empty (bad calls?)");
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return MfskStatus::DecodeFailed;
    }

    let decodes =
        q65_scan_with_ap_list_for(submode, &audio, &candidates, hash_table_inner(hash_table));
    let rows = q65_rows(submode, &decodes);
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

// ──────────────────────────────────────────────────────────────────────────
// Mode addressing and introspection (FFI v2 slice 1)
//
// `mfsk_version()` was the entire introspection surface, while
// `registry::PROTOCOLS` — which knows every wired mode with full
// geometry — was not exposed at all. So every consumer that needed to
// know what a mode supports hardcoded a matrix, and the differences
// between FT8, FT4 and the FST4 sub-modes were invisible from C rather
// than merely inconvenient.
//
// The bridge is the registry's own stable display name, not an index:
// `MfskMode` discriminants are ABI and fixed forever, while registry
// membership is feature-gated and shifts between builds.
// ──────────────────────────────────────────────────────────────────────────

/// Every `MfskMode`, with the name it is known by and whether that name
/// is a `registry::PROTOCOLS` key.
///
/// MSK144 is the one entry with no registry key: it is deliberately
/// outside the `Protocol` trait — not FSK, and its decoder bypasses the
/// shared pipeline by design — so it is addressed here and described
/// from the constants below.
///
/// Declaration order is the order `mfsk_mode_at` reports, which is
/// deliberately the enum's order rather than the registry's: it is the
/// one a caller can reason about from the header alone.
///
/// Names carry an explicit NUL so `mfsk_mode_name` can hand out a
/// `const char*` with no allocation and no lifetime question.
const MODE_TABLE: &[(MfskMode, &str, bool)] = &[
    (MfskMode::Ft8, "FT8\0", true),
    (MfskMode::Ft4, "FT4\0", true),
    (MfskMode::Fst4s15, "FST4-15\0", true),
    (MfskMode::Fst4s30, "FST4-30\0", true),
    (MfskMode::Fst4s60, "FST4-60A\0", true),
    (MfskMode::Fst4s120, "FST4-120\0", true),
    (MfskMode::Fst4s300, "FST4-300\0", true),
    (MfskMode::Wspr, "WSPR\0", true),
    (MfskMode::Jt9, "JT9\0", true),
    (MfskMode::Jt65, "JT65\0", true),
    (MfskMode::Q65a15, "Q65-15A\0", true),
    (MfskMode::Q65a30, "Q65-30A\0", true),
    (MfskMode::Q65a60, "Q65-60A\0", true),
    (MfskMode::Q65b60, "Q65-60B\0", true),
    (MfskMode::Q65c60, "Q65-60C\0", true),
    (MfskMode::Q65d60, "Q65-60D\0", true),
    (MfskMode::Q65e60, "Q65-60E\0", true),
    (MfskMode::Q65d120, "Q65-120D\0", true),
    (MfskMode::Q65e120, "Q65-120E\0", true),
    (MfskMode::Q65a300, "Q65-300A\0", true),
    (MfskMode::Msk144, "MSK144\0", false),
    (MfskMode::UvRobust, "UvRobust\0", true),
    (MfskMode::UvStandard, "UvStandard\0", true),
    (MfskMode::UvUltraRobust, "UvUltraRobust\0", true),
    (MfskMode::UvExpress, "UvExpress\0", true),
];

/// MSK144's geometry, which no registry entry carries. Reported rather
/// than omitted so a C caller enumerating modes sees a complete list;
/// the capability word is what says the decode handle does not drive it.
struct Msk144Geometry;
impl Msk144Geometry {
    const NTONES: u32 = 2; // MSK is binary
    const BITS_PER_SYMBOL: u32 = 1;
    const NSPS: u32 = 6; // at 12 kHz
    const SYMBOL_DT: f32 = 0.0005;
    const TONE_SPACING_HZ: f32 = 1_000.0; // 1/(2*T) for T = 0.5 ms
    const N_DATA: u32 = 128;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 144;
    const T_FRAME_S: f32 = 0.072;
    const FEC_K: u32 = 90; // LDPC(128,90)
    const FEC_N: u32 = 128;
    const PAYLOAD_BITS: u32 = 77;
}

/// Turn a caller-supplied mode value into an `MfskMode`, or `None`.
///
/// **Every `extern "C"` entry point takes the mode as `uint32_t`, not
/// as `MfskMode`, and this is why.** A `#[repr(C)]` fieldless enum is
/// an `int` to C: a caller can pass a value from a config file, a
/// newer header, or a plain mistake, and reading an out-of-range
/// discriminant as a Rust enum is undefined behaviour — the compiler is
/// entitled to assume the value is one of the listed variants and
/// optimise the match accordingly.
///
/// That is not theoretical. `mfsk_mode_name((MfskMode)9999)` from the
/// C++ driver segfaulted, which is how this was found; the same class
/// of defect as a `memset` options struct producing an invalid
/// `MfskDecodeDepth`, and the reason `read_params` validates its enum
/// fields as integers too.
///
/// C callers still write `MFSK_MODE_FT8` — an unscoped enum constant
/// converts to `uint32_t` implicitly in both C and C++ — so the
/// ergonomics are unchanged and the soundness question is gone.
fn mode_of(raw: u32) -> Option<MfskMode> {
    MODE_TABLE
        .iter()
        .map(|(m, _, _)| *m)
        .find(|m| *m as u32 == raw)
}

fn mode_index(mode: MfskMode) -> Option<usize> {
    MODE_TABLE.iter().position(|(m, _, _)| *m == mode)
}

/// The display name with its trailing NUL stripped — what a Rust-side
/// comparison wants.
fn mode_name_str(i: usize) -> &'static str {
    MODE_TABLE[i].1.trim_end_matches('\0')
}

/// Registry entry for `mode`, or `None` if this build was compiled
/// without that protocol's feature (or the mode has no entry at all).
fn mode_meta(mode: MfskMode) -> Option<&'static mfsk_core::ProtocolMeta> {
    let i = mode_index(mode)?;
    if !MODE_TABLE[i].2 {
        return None;
    }
    let name = mode_name_str(i);
    mfsk_core::PROTOCOLS.iter().find(|p| p.name == name)
}

/// Number of modes **this build** actually supports, which is not the
/// number of `MfskMode` discriminants: protocols are feature-gated.
/// Pair with `mfsk_mode_at` to enumerate.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_mode_count() -> u32 {
    MODE_TABLE
        .iter()
        .filter(|(m, _, _)| mode_is_present(*m))
        .count() as u32
}

fn mode_is_present(mode: MfskMode) -> bool {
    if mode == MfskMode::Msk144 {
        return cfg!(feature = "protocols");
    }
    mode_meta(mode).is_some()
}

/// The `index`-th mode this build supports, `0 <= index < mfsk_mode_count()`.
///
/// Writes the mode to `out` and returns `MFSK_STATUS_OK`; returns
/// `MFSK_STATUS_INVALID_ARGUMENT` for a null `out` or an index past the
/// end. A status rather than a returned enum because C has no way to
/// spell "no such mode" inside an enum whose every value is legal.
///
/// # Safety
/// `out` must be null or point to a writable `MfskMode`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_mode_at(index: u32, out: *mut MfskMode) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_mode_at: out is NULL");
        return MfskStatus::InvalidArg;
    }
    match MODE_TABLE
        .iter()
        .filter(|(m, _, _)| mode_is_present(*m))
        .nth(index as usize)
    {
        Some((m, _, _)) => {
            unsafe { *out = *m };
            MfskStatus::Ok
        }
        None => {
            set_error("mfsk_mode_at: index past the end of this build's mode list");
            MfskStatus::InvalidArg
        }
    }
}

/// Stable display name for `mode` (`"FT8"`, `"FST4-120"`), or NULL if
/// `mode` is not a value this library knows.
///
/// The returned pointer is a static NUL-terminated string with the
/// lifetime of the library; do not free it. It is also the key
/// [`mfsk_mode_from_name`] accepts, so the two round-trip.
///
/// Answers for a mode this build lacks — the name is a property of the
/// mode, not of the build.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_mode_name(mode: u32) -> *const c_char {
    match mode_of(mode).and_then(mode_index) {
        // The literal carries its own NUL, so this is a valid C string.
        Some(i) => MODE_TABLE[i].1.as_ptr() as *const c_char,
        None => ptr::null(),
    }
}

/// Look `name` up as a mode. Case-sensitive, matching the registry's own
/// display strings exactly.
///
/// Writes the mode to `out` and returns `MFSK_STATUS_OK`;
/// `MFSK_STATUS_INVALID_ARGUMENT` for a null argument or a name that is
/// not a mode, `MFSK_STATUS_UNKNOWN_PROTOCOL` for a real mode this build
/// was compiled without — the distinction a caller needs in order to
/// tell a typo from a missing feature.
///
/// # Safety
/// `name` must be a valid NUL-terminated C string; `out` must point to a
/// writable `MfskMode`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_mode_from_name(
    name: *const c_char,
    out: *mut MfskMode,
) -> MfskStatus {
    if name.is_null() || out.is_null() {
        set_error("mfsk_mode_from_name: NULL argument");
        return MfskStatus::InvalidArg;
    }
    let Ok(name) = (unsafe { CStr::from_ptr(name) }).to_str() else {
        set_error("mfsk_mode_from_name: name is not valid UTF-8");
        return MfskStatus::InvalidArg;
    };
    let found = MODE_TABLE
        .iter()
        .enumerate()
        .find(|(i, _)| mode_name_str(*i) == name);
    match found {
        Some((_, (m, _, _))) if mode_is_present(*m) => {
            unsafe { *out = *m };
            MfskStatus::Ok
        }
        Some(_) => {
            set_error("mfsk_mode_from_name: this build was compiled without that mode");
            MfskStatus::UnknownProtocol
        }
        None => {
            set_error("mfsk_mode_from_name: not a mode name");
            MfskStatus::InvalidArg
        }
    }
}

/// Geometry and capability for `mode`.
///
/// **Set `out->size = sizeof(MfskModeInfo)` before calling**, or zero
/// the struct and the library fills it in. Only the prefix the caller
/// declared is written, so a newer library stays usable from an older
/// header.
///
/// Returns `MFSK_STATUS_UNKNOWN_PROTOCOL` if this build lacks `mode`.
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_mode_info(mode: u32, out: *mut MfskModeInfo) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_mode_info: out is NULL");
        return MfskStatus::InvalidArg;
    }
    let Some(mode) = mode_of(mode) else {
        set_error("mfsk_mode_info: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    let Some(i) = mode_index(mode) else {
        set_error("mfsk_mode_info: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    if !mode_is_present(mode) {
        set_error("mfsk_mode_info: this build was compiled without that mode");
        return MfskStatus::UnknownProtocol;
    }

    let mut info = MfskModeInfo {
        size: core::mem::size_of::<MfskModeInfo>() as u32,
        mode,
        name: [0; 16],
        ntones: 0,
        bits_per_symbol: 0,
        nsps: 0,
        symbol_dt: 0.0,
        tone_spacing_hz: 0.0,
        gfsk_bt: 0.0,
        gfsk_hmod: 0.0,
        n_data: 0,
        n_sync: 0,
        n_symbols: 0,
        t_slot_s: 0.0,
        slot_samples_12k: 0,
        tx_start_offset_s: 0.0,
        fec_k: 0,
        fec_n: 0,
        payload_bits: 0,
        decode_fft1_size: 0,
        caps: 0,
    };
    copy_name(&mut info.name, mode_name_str(i));

    match mode_meta(mode) {
        Some(m) => {
            info.ntones = m.ntones;
            info.bits_per_symbol = m.bits_per_symbol;
            info.nsps = m.nsps;
            info.symbol_dt = m.symbol_dt;
            info.tone_spacing_hz = m.tone_spacing_hz;
            info.gfsk_bt = m.gfsk_bt;
            info.gfsk_hmod = m.gfsk_hmod;
            info.n_data = m.n_data;
            info.n_sync = m.n_sync;
            info.n_symbols = m.n_symbols;
            info.t_slot_s = m.t_slot_s;
            info.slot_samples_12k = m.slot_samples_12k;
            info.tx_start_offset_s = m.tx_start_offset_s;
            info.fec_k = m.fec_k as u32;
            info.fec_n = m.fec_n as u32;
            info.payload_bits = m.payload_bits;
            info.decode_fft1_size = m.decode_fft1_size;
            info.caps = u64::from(m.profile.caps);
        }
        None => {
            // MSK144: no registry entry by design.
            info.ntones = Msk144Geometry::NTONES;
            info.bits_per_symbol = Msk144Geometry::BITS_PER_SYMBOL;
            info.nsps = Msk144Geometry::NSPS;
            info.symbol_dt = Msk144Geometry::SYMBOL_DT;
            info.tone_spacing_hz = Msk144Geometry::TONE_SPACING_HZ;
            info.n_data = Msk144Geometry::N_DATA;
            info.n_sync = Msk144Geometry::N_SYNC;
            info.n_symbols = Msk144Geometry::N_SYMBOLS;
            info.t_slot_s = Msk144Geometry::T_FRAME_S;
            info.slot_samples_12k = (Msk144Geometry::T_FRAME_S * 12_000.0) as u32;
            info.fec_k = Msk144Geometry::FEC_K;
            info.fec_n = Msk144Geometry::FEC_N;
            info.payload_bits = Msk144Geometry::PAYLOAD_BITS;
            info.caps = MFSK_CAP_ENCODE;
        }
    }

    unsafe { write_size_versioned(out, &info) };
    MfskStatus::Ok
}

/// Capability bitmask for `mode` — the same word `mfsk_mode_info` puts
/// in `caps`, for callers that want only that. Returns 0 for a mode this
/// build lacks, which is also a legal "supports nothing" answer; use
/// `mfsk_mode_info` when the difference matters.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_mode_caps(mode: u32) -> u64 {
    let Some(mode) = mode_of(mode) else {
        return 0;
    };
    match mode_meta(mode) {
        Some(m) => u64::from(m.profile.caps),
        None if mode == MfskMode::Msk144 && mode_is_present(mode) => MFSK_CAP_ENCODE,
        None => 0,
    }
}

/// Default search parameters for `mode`.
///
/// This is what removes the ABI's worst trap: three different
/// per-protocol NULL-option defaults lived inside one function, one of
/// them a `sync_min` of 2.0 that no test in the tree uses. Defaults are
/// data now, published per mode, and `sync_scale` says which of them are
/// even comparable.
///
/// Size-versioned on the same contract as `mfsk_mode_info`. Returns
/// `MFSK_STATUS_UNSUPPORTED` for a mode with no wide-band search to
/// describe.
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_mode_defaults(mode: u32, out: *mut MfskDecodeDefaults) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_mode_defaults: out is NULL");
        return MfskStatus::InvalidArg;
    }
    let Some(mode) = mode_of(mode) else {
        set_error("mfsk_mode_defaults: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    let Some(m) = mode_meta(mode) else {
        set_error("mfsk_mode_defaults: no such mode in this build");
        return MfskStatus::UnknownProtocol;
    };
    let d = m.profile.defaults;
    if !(d.freq_max_hz > d.freq_min_hz && d.max_cand > 0) {
        set_error("mfsk_mode_defaults: this mode publishes no searchable band");
        return MfskStatus::Unsupported;
    }
    let defaults = MfskDecodeDefaults {
        size: core::mem::size_of::<MfskDecodeDefaults>() as u32,
        freq_min_hz: d.freq_min_hz,
        freq_max_hz: d.freq_max_hz,
        sync_min: d.sync_min,
        max_cand: d.max_cand,
        sync_scale: match m.profile.sync_scale {
            mfsk_core::registry::SyncScale::CostasAbsolute => MfskSyncScale::CostasAbsolute,
            mfsk_core::registry::SyncScale::BaselineNormalised => MfskSyncScale::BaselineNormalised,
        },
    };
    unsafe { write_size_versioned(out, &defaults) };
    MfskStatus::Ok
}

/// ABI revision, distinct from [`mfsk_version`].
///
/// `mfsk_version` tracks the crate's release number and moves for
/// reasons that have nothing to do with the boundary. This moves only
/// when the C surface changes shape, so it is the one to check before
/// deciding a header and a library agree.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_abi_version() -> u32 {
    2
}

/// Copy a `size`-versioned struct into caller memory, writing only the
/// prefix the caller declared.
///
/// The caller sets `out->size` to its own `sizeof`. A zero (or
/// oversized) value means "I have the same header you do", so the whole
/// struct is written. A smaller value means an older header, and only
/// that many bytes are copied — with `size` itself rewritten to what was
/// actually written, so the caller can tell.
///
/// # Safety
/// `out` must point to at least `min(out->size, sizeof(T))` writable
/// bytes, and `T` must be `#[repr(C)]` with `size: u32` first.
unsafe fn write_size_versioned<T: Copy>(out: *mut T, value: &T) {
    let full = core::mem::size_of::<T>();
    // `size` is the first field of every struct this is used with.
    let declared = unsafe { core::ptr::read_unaligned(out as *const u32) } as usize;
    let n = if declared == 0 || declared > full {
        full
    } else {
        declared
    };
    unsafe {
        core::ptr::copy_nonoverlapping(value as *const T as *const u8, out as *mut u8, n);
        core::ptr::write_unaligned(out as *mut u32, n as u32);
    }
}

fn copy_name(dst: &mut [c_char; 16], src: &str) {
    let b = src.as_bytes();
    let n = b.len().min(dst.len() - 1);
    for (d, &s) in dst.iter_mut().zip(&b[..n]) {
        *d = s as c_char;
    }
    dst[n] = 0;
}

// ──────────────────────────────────────────────────────────────────────────
// The decoder handle, parameters and decode calls (FFI v2 slice 3)
// ──────────────────────────────────────────────────────────────────────────

/// What a handle owns, and why each piece is on the handle rather than
/// rebuilt per call.
struct V2Decoder {
    mode: MfskMode,
    params: MfskDecodeParams,
    /// **The functional fix this handle exists for.** Every decode call
    /// used to build a fresh empty `CallsignHashTable`, so a hashed
    /// `<...>` callsign could never resolve over this ABI — for any
    /// protocol, in any call, since the ABI was written. A table has to
    /// outlive the slot that populated it to be worth anything.
    hashes: mfsk_core::msg::CallsignHashTable,
    /// The previous call's native rows, kept so `copy_info` can hand
    /// back FEC information bits without the row carrying a pointer.
    last: Vec<(Vec<u8>, MfskDecode)>,
    /// Streaming delivery, set by `mfsk_session_set_on_decode`.
    on_decode: MfskDecodeCallback,
    on_decode_user: SyncUserData,
    /// Per-handle error slot. The process-global `thread_local!` is
    /// wrong for a coroutine or `async` caller, which legitimately hops
    /// threads between checking a status and reading the message and
    /// then finds NULL.
    error: Option<CString>,
}

impl V2Decoder {
    fn fail(&mut self, msg: impl Into<String>) -> MfskStatus {
        let m = msg.into();
        set_error(m.clone());
        self.error = CString::new(m).ok();
        MfskStatus::InvalidArg
    }
}

fn v2(dec: *mut MfskDecodeSession) -> Option<&'static mut V2Decoder> {
    unsafe { (dec as *mut V2Decoder).as_mut() }
}

fn v2_ref(dec: *const MfskDecodeSession) -> Option<&'static V2Decoder> {
    unsafe { (dec as *const V2Decoder).as_ref() }
}

fn cstr_field(f: &[c_char]) -> &str {
    let bytes: &[u8] = unsafe { slice::from_raw_parts(f.as_ptr() as *const u8, f.len()) };
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    core::str::from_utf8(&bytes[..end]).unwrap_or("")
}

fn write_field(dst: &mut [c_char], s: &str) {
    let b = s.as_bytes();
    let n = b.len().min(dst.len() - 1);
    for (d, &c) in dst.iter_mut().zip(&b[..n]) {
        *d = c as c_char;
    }
    dst[n] = 0;
}

/// Fill `out` with `mode`'s published defaults.
///
/// Always call this before touching a `MfskDecodeParams`. Zeroing it by
/// hand is not equivalent: a zero `max_cand` or a zero band decodes
/// nothing, and `freq_hint_hz` has to be NaN rather than 0 to mean
/// "unset" — 0 Hz is a frequency.
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decode_params_init(
    mode: u32,
    out: *mut MfskDecodeParams,
) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_decode_params_init: out is NULL");
        return MfskStatus::InvalidArg;
    }
    let Some(mode) = mode_of(mode) else {
        set_error("mfsk_decode_params_init: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    let Some(meta) = mode_meta(mode) else {
        set_error("mfsk_decode_params_init: no such mode in this build");
        return MfskStatus::UnknownProtocol;
    };
    let d = meta.profile.defaults;
    let mut p = MfskDecodeParams {
        size: core::mem::size_of::<MfskDecodeParams>() as u32,
        freq_min_hz: d.freq_min_hz,
        freq_max_hz: d.freq_max_hz,
        sync_min: d.sync_min,
        max_cand: d.max_cand,
        depth: MfskDecodeDepth::BpAllOsd,
        strictness: MfskStrictness::Normal,
        eq_mode: MfskEqMode::Off,
        freq_hint_hz: f32::NAN,
        sic_rounds: 0,
        sic_early: false,
        has_ap_hint: false,
        ap_call1: [0; MFSK_AP_FIELD_LEN],
        ap_call2: [0; MFSK_AP_FIELD_LEN],
        ap_grid: [0; MFSK_AP_FIELD_LEN],
        search_hz: 0.0,
    };
    // A mode that cannot turn OSD off should not be told to.
    if meta.profile.caps & mfsk_core::registry::caps::OSD == 0 {
        p.depth = MfskDecodeDepth::BpAll;
    }
    unsafe { write_size_versioned(out, &p) };
    MfskStatus::Ok
}

/// Reject a parameter set that asks a mode for something it does not
/// have, instead of dropping the field silently.
///
/// The old ABI accepted eleven options and quietly ignored six of them
/// depending on protocol — `ap_hint` reached FT8 only, `sic_rounds`
/// FT8+FT4, `strictness` was a no-op on FST4 — and `depth` was worse
/// than ignored: `BP_ALL` was silently *upgraded* to the full ladder,
/// so a caller asking for the cheap path paid for the expensive one.
/// On FST4-300 that ladder sits behind a 4 194 304-point transform.
fn validate_params(mode: MfskMode, p: &MfskDecodeParams) -> Result<(), String> {
    let caps = mfsk_mode_caps(mode as u32);
    let name = mode_index(mode).map(mode_name_str).unwrap_or("?");

    // First, because everything below is about a wide-band search this
    // mode may not have at all. A mode without the bit publishes no
    // usable default band either, so checking anything else first
    // produces a confusing message about `max_cand` for what is really
    // "wrong entry point".
    if caps & MFSK_CAP_DECODE_HANDLE == 0 {
        return Err(format!(
            "{name} has no decode-handle entry point — it is not lesser, it is \
             shaped differently (Q65 takes a nominal start sample and a time \
             tolerance; WSPR/JT9/JT65 have no builder). Check \
             MFSK_CAP_DECODE_HANDLE and use that mode's own family instead"
        ));
    }

    // Spelled out rather than `!(max > min)`: a NaN edge is not
    // "inverted", it is a band that can never match anything, and it
    // arrives from a caller that memset the struct to a float pattern
    // instead of calling init.
    if !p.freq_min_hz.is_finite() || !p.freq_max_hz.is_finite() || p.freq_max_hz <= p.freq_min_hz {
        return Err(format!(
            "{name}: search band [{}, {}] is not a usable range — did you call \
             mfsk_decode_params_init?",
            p.freq_min_hz, p.freq_max_hz
        ));
    }
    if p.max_cand == 0 {
        return Err(format!(
            "{name}: max_cand is 0, which decodes nothing — did you call \
             mfsk_decode_params_init?"
        ));
    }
    if p.sic_rounds > 0 && caps & MFSK_CAP_SIC_ROUNDS == 0 {
        return Err(format!(
            "{name} has no successive-interference cancellation"
        ));
    }
    if p.sic_early && caps & MFSK_CAP_SIC_EARLY == 0 {
        return Err(format!(
            "{name} has no checkpoint-emulation early decode (FT8 only)"
        ));
    }
    if p.has_ap_hint && caps & (MFSK_CAP_AP_WIDEBAND | MFSK_CAP_AP_NARROW) == 0 {
        return Err(format!("{name} does not take an a-priori hint"));
    }
    if p.search_hz != 0.0 && caps & MFSK_CAP_SNIPER != 0 && !p.freq_hint_hz.is_finite() {
        return Err(format!(
            "{name}: a narrow-band search needs freq_hint_hz as the carrier to \
             aim at; search_hz alone says how wide, not where"
        ));
    }
    if p.search_hz != 0.0 && caps & MFSK_CAP_SNIPER == 0 {
        return Err(format!(
            "{name} has no narrow-band search — search_hz is meaningful only \
             with MFSK_CAP_SNIPER, which is FT8's alone"
        ));
    }
    if p.eq_mode != MfskEqMode::Off && caps & MFSK_CAP_EQ_MODE == 0 {
        return Err(format!("{name} does not honour eq_mode"));
    }
    Ok(())
}

/// Create a decoder for `mode`, validated against `params`.
///
/// `params` may be NULL for the mode's defaults. On failure this
/// returns NULL and writes the reason to `out_status` (which may itself
/// be NULL if you only care that it failed); `mfsk_last_error()` carries
/// the detail.
///
/// **A parameter the mode does not support is an error here**, not a
/// field silently dropped at decode time.
///
/// # Safety
/// `params` must be null or point to a valid `MfskDecodeParams`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_open(
    mode: u32,
    params: *const MfskDecodeParams,
    out_status: *mut MfskStatus,
) -> *mut MfskDecodeSession {
    let report = |st: MfskStatus| {
        if !out_status.is_null() {
            unsafe { *out_status = st };
        }
    };
    let Some(mode) = mode_of(mode) else {
        set_error("mfsk_session_open: not a mode this library knows");
        report(MfskStatus::InvalidArg);
        return ptr::null_mut();
    };
    if mode_meta(mode).is_none() {
        set_error("mfsk_session_open: no such mode in this build");
        report(MfskStatus::UnknownProtocol);
        return ptr::null_mut();
    }
    let mut p = MfskDecodeParams {
        size: core::mem::size_of::<MfskDecodeParams>() as u32,
        freq_min_hz: 0.0,
        freq_max_hz: 0.0,
        sync_min: 0.0,
        max_cand: 0,
        depth: MfskDecodeDepth::BpAllOsd,
        strictness: MfskStrictness::Normal,
        eq_mode: MfskEqMode::Off,
        freq_hint_hz: f32::NAN,
        sic_rounds: 0,
        sic_early: false,
        has_ap_hint: false,
        ap_call1: [0; MFSK_AP_FIELD_LEN],
        ap_call2: [0; MFSK_AP_FIELD_LEN],
        ap_grid: [0; MFSK_AP_FIELD_LEN],
        search_hz: 0.0,
    };
    if unsafe { mfsk_decode_params_init(mode as u32, &mut p) } != MfskStatus::Ok {
        report(MfskStatus::UnknownProtocol);
        return ptr::null_mut();
    }
    if !params.is_null()
        && let Err(e) = unsafe { read_params(params, &mut p) }
    {
        set_error(format!("mfsk_session_open: {e}"));
        report(MfskStatus::InvalidArg);
        return ptr::null_mut();
    }
    if let Err(e) = validate_params(mode, &p) {
        set_error(format!("mfsk_session_open: {e}"));
        report(MfskStatus::Unsupported);
        return ptr::null_mut();
    }
    report(MfskStatus::Ok);
    Box::into_raw(Box::new(V2Decoder {
        mode,
        params: p,
        hashes: mfsk_core::msg::CallsignHashTable::new(),
        last: Vec::new(),
        on_decode: None,
        on_decode_user: SyncUserData(ptr::null_mut()),
        error: None,
    })) as *mut MfskDecodeSession
}

/// Release a handle from [`mfsk_session_open`]. Null is a no-op.
///
/// # Safety
/// `dec` must be a handle from `mfsk_session_open`, released once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_close(dec: *mut MfskDecodeSession) {
    if !dec.is_null() {
        drop(unsafe { Box::from_raw(dec as *mut V2Decoder) });
    }
}

/// The last error recorded **on this handle**, or NULL.
///
/// Prefer this over `mfsk_last_error()` whenever you have a handle. The
/// global one is a `thread_local!`, which a Kotlin coroutine or a Swift
/// `async` caller reads as NULL after hopping threads between the status
/// check and the message. The pointer is valid until the next call on
/// this handle.
///
/// # Safety
/// `dec` must be a live handle or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_last_error(dec: *const MfskDecodeSession) -> *const c_char {
    match v2_ref(dec) {
        Some(d) => d.error.as_ref().map(|s| s.as_ptr()).unwrap_or(ptr::null()),
        None => ptr::null(),
    }
}

/// Read a caller-supplied [`MfskDecodeParams`], taking only the prefix
/// they declared and **validating every enum and bool before the bytes
/// are ever read as Rust types**.
///
/// This is the whole reason it is not a `copy_nonoverlapping` into a
/// `MfskDecodeParams`. A `#[repr(C)]` fieldless enum is an `int` to C,
/// so a caller can `memset` the struct, pass a value from a config
/// file, or simply be a version ahead — and reading an out-of-range
/// discriminant as a Rust enum is undefined behaviour, not a wrong
/// answer. Same for `bool`, which must be exactly 0 or 1 in Rust and is
/// whatever the caller wrote in C.
///
/// The boundary is where that has to stop, and it is the same class of
/// defect as the `&'static mut`-from-a-raw-pointer accessor this struct
/// replaces — caught here by `rustc` refusing to zero-initialise the
/// type in a test.
///
/// # Safety
/// `src` must point to at least `src->size` readable bytes.
unsafe fn read_params(
    src: *const MfskDecodeParams,
    dst: &mut MfskDecodeParams,
) -> Result<(), String> {
    use core::mem::{MaybeUninit, offset_of, size_of};

    let full = size_of::<MfskDecodeParams>();
    let declared = unsafe { core::ptr::read_unaligned(src as *const u32) } as usize;
    let n = if declared == 0 || declared > full {
        full
    } else {
        declared
    };

    // Seed with the defaults already in `dst`, so a short struct leaves
    // the tail at the mode default rather than at zero.
    let mut img: MaybeUninit<MfskDecodeParams> = MaybeUninit::uninit();
    unsafe {
        core::ptr::copy_nonoverlapping(
            dst as *const MfskDecodeParams as *const u8,
            img.as_mut_ptr() as *mut u8,
            full,
        );
        core::ptr::copy_nonoverlapping(src as *const u8, img.as_mut_ptr() as *mut u8, n);
    }
    let base = img.as_ptr() as *const u8;

    let enum_at =
        |off: usize| -> i32 { unsafe { core::ptr::read_unaligned(base.add(off) as *const i32) } };
    let bool_at = |off: usize| -> u8 { unsafe { core::ptr::read_unaligned(base.add(off)) } };

    let depth = enum_at(offset_of!(MfskDecodeParams, depth));
    if !(0..=2).contains(&depth) {
        return Err(format!("depth: {depth} is not an MfskDecodeDepth"));
    }
    let strictness = enum_at(offset_of!(MfskDecodeParams, strictness));
    if !(0..=2).contains(&strictness) {
        return Err(format!("strictness: {strictness} is not an MfskStrictness"));
    }
    let eq = enum_at(offset_of!(MfskDecodeParams, eq_mode));
    if !(0..=1).contains(&eq) {
        return Err(format!("eq_mode: {eq} is not an MfskEqMode"));
    }
    for (name, off) in [
        ("sic_early", offset_of!(MfskDecodeParams, sic_early)),
        ("has_ap_hint", offset_of!(MfskDecodeParams, has_ap_hint)),
    ] {
        let b = bool_at(off);
        if b > 1 {
            return Err(format!("{name}: {b} is not a bool"));
        }
    }

    // Every discriminant checked: the image is now a valid value.
    let mut v = unsafe { img.assume_init() };
    v.size = full as u32;
    if v.depth == MfskDecodeDepth::ModeDefault {
        // "Whatever the mode publishes" — which is what `dst` still
        // carries from `mfsk_decode_params_init`.
        v.depth = dst.depth;
    }
    *dst = v;
    Ok(())
}

/// Build the mfsk-core AP hint a params struct describes, if any.
fn ap_hint_of(p: &MfskDecodeParams) -> Option<mfsk_core::msg::ApHint> {
    if !p.has_ap_hint {
        return None;
    }
    let mut h = mfsk_core::msg::ApHint::new();
    let (c1, c2, g) = (
        cstr_field(&p.ap_call1),
        cstr_field(&p.ap_call2),
        cstr_field(&p.ap_grid),
    );
    if !c1.is_empty() {
        h = h.with_call1(c1);
    }
    if !c2.is_empty() {
        h = h.with_call2(c2);
    }
    if !g.is_empty() {
        h = h.with_grid(g);
    }
    h.has_info().then_some(h)
}

/// One decoded row, built from a `DecodeResult` plus the text its
/// protocol's codec produced.
fn row(mode: MfskMode, r: &ft8::DecodeResult, text: &str, resolved: bool) -> MfskDecode {
    let mut d = MfskDecode {
        size: core::mem::size_of::<MfskDecode>() as u32,
        mode,
        text: [0; MFSK_DECODE_TEXT_LEN],
        freq_hz: r.freq_hz,
        dt_sec: r.dt_sec,
        snr_db: r.snr_db,
        sync_score: r.sync_score,
        sync_cv: r.sync_cv,
        hard_errors: r.hard_errors,
        info_bits: r.info.len() as u16,
        pass: r.pass,
        flags: 0,
    };
    write_field(&mut d.text, text);
    if resolved {
        d.flags |= MFSK_DECODE_FLAG_HASH_RESOLVED;
    }
    d
}

/// Decode one slot of `audio` (12 kHz, i16) for whichever mode the
/// handle carries, storing the rows on the handle.
fn run_decode(d: &mut V2Decoder, audio: &[i16], p: &MfskDecodeParams) -> Result<(), String> {
    use mfsk_core::msg::decode_request::DecodeRequest;

    d.last.clear();

    /// Collect one protocol's rows onto the handle.
    ///
    /// Split from the search itself because the narrow-band arm exists
    /// for FT8 alone: `SupportsSniper` is implemented only there, so a
    /// macro that named `::sniper` for every protocol would not
    /// type-check even with the branch statically false. That is the
    /// trait doing its job — the sniper is the receive half of an
    /// analogue roofing filter, not something every mode should have.
    macro_rules! collect {
        ($proto:ty, $results:expr) => {{
            let results = $results;
            for r in &results {
                // The handle's table is what lets a `<...>` reference
                // resolve at all, and it is fed from each decode so a
                // later slot can read what an earlier one named.
                let plain = mfsk_core::msg::wsjt77::unpack77(r.message77()).unwrap_or_default();
                let text = mfsk_core::msg::wsjt77::unpack77_with_hash(r.message77(), &d.hashes)
                    .unwrap_or_default();
                let resolved = text != plain;
                // `insert` already skips `<...>`, CQ-prefixed and
                // too-short tokens, so feeding it the message's words
                // is safe and is what it was shaped for.
                for word in text.split_whitespace() {
                    d.hashes.insert(word);
                }
                let mode = d.mode;
                let built = row(mode, r, &text, resolved);
                if let Some(cb) = d.on_decode {
                    // Valid for the duration of the call only, which is
                    // what the callback's contract promises.
                    unsafe { cb(&built, d.on_decode_user.ptr()) };
                }
                d.last.push((r.info.to_vec(), built));
            }
            Ok(())
        }};
    }

    /// The wide-band search — the main path for every mode here.
    macro_rules! wide {
        ($proto:ty) => {{
            let hint = ap_hint_of(p);
            let mut req = <DecodeRequest<$proto>>::new(
                audio,
                p.freq_min_hz,
                p.freq_max_hz,
                p.sync_min,
                p.max_cand as usize,
            )
            .osd(map_osd(p.depth))
            .strictness(map_strictness(p.strictness))
            .eq_mode(map_eq_mode(p.eq_mode));
            if p.freq_hint_hz.is_finite() {
                req = req.freq_hint(p.freq_hint_hz);
            }
            if let Some(h) = hint.as_ref() {
                req = req.ap_hint(h);
            }
            collect!($proto, req.decode().results)
        }};
    }

    /// Same, for a protocol that also implements `SupportsSicRounds`.
    ///
    /// SIC cannot live in `wide!` because the strategy extensions are
    /// trait-gated per protocol — `.sic_rounds()` needs
    /// `SupportsSicRounds` (FT8, FT4) and `.sic_early()` needs
    /// `SupportsSicEarly` (FT8 alone), mirroring an upstream absence.
    /// A single macro naming them would fail to compile for FST4 even
    /// with the branch statically dead, which is the gating working as
    /// intended: `validate_params` has already rejected the request at
    /// `mfsk_session_open` if the mode cannot do it.
    macro_rules! wide_sic {
        ($proto:ty, $early:expr) => {{
            let hint = ap_hint_of(p);
            let mut req = <DecodeRequest<$proto>>::new(
                audio,
                p.freq_min_hz,
                p.freq_max_hz,
                p.sync_min,
                p.max_cand as usize,
            )
            .osd(map_osd(p.depth))
            .strictness(map_strictness(p.strictness))
            .eq_mode(map_eq_mode(p.eq_mode));
            if p.freq_hint_hz.is_finite() {
                req = req.freq_hint(p.freq_hint_hz);
            }
            if let Some(h) = hint.as_ref() {
                req = req.ap_hint(h);
            }
            if p.sic_rounds > 0 {
                req = req.sic_rounds(p.sic_rounds as usize);
            }
            let _ = $early;
            collect!($proto, req.decode().results)
        }};
    }

    match d.mode {
        // FT8 is the one mode with a narrow-band arm, chosen by
        // `search_hz`. See `MFSK_CAP_SNIPER`.
        MfskMode::Ft8 if p.search_hz > 0.0 => {
            let hint = ap_hint_of(p);
            let mut req = <DecodeRequest<mfsk_core::ft8::Ft8>>::sniper(
                audio,
                p.freq_hint_hz,
                p.max_cand as usize,
            )
            .osd(map_osd(p.depth))
            .strictness(map_strictness(p.strictness))
            .eq_mode(map_eq_mode(p.eq_mode))
            .search_hz(p.search_hz);
            if let Some(h) = hint.as_ref() {
                req = req.ap_hint(h);
            }
            collect!(mfsk_core::ft8::Ft8, req.decode().results)
        }
        // FT8 additionally has `.sic_early()`; FT4 has `.sic_rounds()`
        // only. `validate_params` rejects anything else up front.
        MfskMode::Ft8 if p.sic_early => {
            let hint = ap_hint_of(p);
            let mut req = <DecodeRequest<mfsk_core::ft8::Ft8>>::new(
                audio,
                p.freq_min_hz,
                p.freq_max_hz,
                p.sync_min,
                p.max_cand as usize,
            )
            .osd(map_osd(p.depth))
            .strictness(map_strictness(p.strictness))
            .eq_mode(map_eq_mode(p.eq_mode))
            .sic_early();
            if p.freq_hint_hz.is_finite() {
                req = req.freq_hint(p.freq_hint_hz);
            }
            if let Some(h) = hint.as_ref() {
                req = req.ap_hint(h);
            }
            collect!(mfsk_core::ft8::Ft8, req.decode().results)
        }
        MfskMode::Ft8 => wide_sic!(mfsk_core::ft8::Ft8, false),
        MfskMode::Ft4 => wide_sic!(mfsk_core::ft4::Ft4, false),
        MfskMode::Fst4s15 => wide!(mfsk_core::fst4::Fst4s15),
        MfskMode::Fst4s30 => wide!(mfsk_core::fst4::Fst4s30),
        MfskMode::Fst4s60 => wide!(mfsk_core::fst4::Fst4s60),
        MfskMode::Fst4s120 => wide!(mfsk_core::fst4::Fst4s120),
        MfskMode::Fst4s300 => wide!(mfsk_core::fst4::Fst4s300),
        other => Err(format!(
            "{} has no decode-handle entry point — check MFSK_CAP_DECODE_HANDLE \
             and use that mode's own family instead",
            mode_index(other).map(mode_name_str).unwrap_or("?")
        )),
    }
}

/// Copy the handle's rows into the caller's array.
///
/// Short buffer is `MFSK_STATUS_INVALID_ARG` with `*out_len` set to what
/// would be needed, so a caller can size and retry without decoding
/// twice.
unsafe fn emit(
    d: &V2Decoder,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let n = d.last.len();
    if !out_len.is_null() {
        unsafe { *out_len = n };
    }
    if n > out_cap {
        set_error("decode: output buffer too small; *out_len is the count needed");
        return MfskStatus::InvalidArg;
    }
    for (i, (_, r)) in d.last.iter().enumerate() {
        unsafe { write_size_versioned(out.add(i), r) };
    }
    MfskStatus::Ok
}

/// Called once per decode, as it is found, if the session has one set.
///
/// The row pointer is valid **only for the duration of the call** —
/// copy anything you need. See [`mfsk_session_set_on_decode`] for the
/// threading contract, which depends on whether this build has `rayon`.
pub type MfskDecodeCallback =
    Option<unsafe extern "C" fn(row: *const MfskDecode, user_data: *mut c_void)>;

/// Deliver decodes through `callback` as they are found, in addition to
/// writing them to the output array at the end of the call.
///
/// Pass a null `callback` to stop. The callback applies to every
/// subsequent decode on this session.
///
/// **Threading.** With `rayon` (the `desktop` feature) the callback
/// fires from a worker thread, possibly several concurrently, in
/// completion order — `docs/reference/STREAMING.md` §3b. Without it
/// (`mobile`) there is one thread and candidate order, which is a
/// *stronger* contract and is the honest answer to "what does dropping
/// rayon cost". Either way the array written at the end of the call is
/// the authoritative set.
///
/// # Safety
/// `callback`, if non-null, must be safely callable from any thread,
/// any number of times including zero, for as long as it is set;
/// `user_data` must stay valid for that time if the callback
/// dereferences it.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_set_on_decode(
    dec: *mut MfskDecodeSession,
    callback: MfskDecodeCallback,
    user_data: *mut c_void,
) -> MfskStatus {
    let Some(d) = v2(dec) else {
        set_error("mfsk_session_set_on_decode: null session handle");
        return MfskStatus::InvalidArg;
    };
    d.on_decode = callback;
    d.on_decode_user = SyncUserData(user_data);
    MfskStatus::Ok
}

/// Decode one slot of 16-bit PCM.
///
/// `params` may be NULL to use the handle's own, set at
/// [`mfsk_session_open`]. Passing one here overrides for this call only
/// and is validated the same way.
///
/// Rows go into `out[0..out_cap]`; `*out_len` always receives the number
/// of decodes found, so a short buffer returns
/// `MFSK_STATUS_INVALID_ARG` with the required count rather than a
/// truncated answer you cannot detect.
///
/// # Safety
/// `samples` must be `n_samples` readable `int16_t`; `out` must be
/// `out_cap` writable `MfskDecode`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_decode_i16(
    dec: *mut MfskDecodeSession,
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    params: *const MfskDecodeParams,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(d) = v2(dec) else {
        set_error("mfsk_session_decode_i16: null decoder handle");
        return MfskStatus::InvalidArg;
    };
    if samples.is_null() || (out.is_null() && out_cap != 0) {
        return d.fail("mfsk_session_decode_i16: null buffer pointer");
    }
    let mut p = d.params;
    if !params.is_null() {
        if let Err(e) = unsafe { read_params(params, &mut p) } {
            return d.fail(format!("mfsk_session_decode_i16: {e}"));
        }
        if let Err(e) = validate_params(d.mode, &p) {
            return d.fail(format!("mfsk_session_decode_i16: {e}"));
        }
    }
    let pcm = unsafe { slice::from_raw_parts(samples, n_samples) };
    let audio: Vec<i16> = if sample_rate == 12_000 {
        pcm.to_vec()
    } else {
        mfsk_core::engine::dsp::resample::resample_to_12k(pcm, sample_rate)
    };
    if let Err(e) = in_pool_mut(|| run_decode(d, &audio, &p)) {
        return d.fail(e);
    }
    unsafe { emit(d, out, out_cap, out_len) }
}

/// Decode one slot of 32-bit float PCM, nominally `-1.0..=1.0`.
///
/// **Not a lossier wrapper.** The pre-v2 `mfsk_decode_f32` quantised
/// f32 to i16 before decoding, which made the float entry point
/// strictly worse than the integer one; this resamples in float and
/// converts once at the end, where the decoders take i16 anyway.
///
/// # Safety
/// As [`mfsk_session_decode_i16`], with `samples` as `float`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_decode_f32(
    dec: *mut MfskDecodeSession,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    params: *const MfskDecodeParams,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(d) = v2(dec) else {
        set_error("mfsk_session_decode_f32: null decoder handle");
        return MfskStatus::InvalidArg;
    };
    if samples.is_null() || (out.is_null() && out_cap != 0) {
        return d.fail("mfsk_session_decode_f32: null buffer pointer");
    }
    let mut p = d.params;
    if !params.is_null() {
        if let Err(e) = unsafe { read_params(params, &mut p) } {
            return d.fail(format!("mfsk_session_decode_f32: {e}"));
        }
        if let Err(e) = validate_params(d.mode, &p) {
            return d.fail(format!("mfsk_session_decode_f32: {e}"));
        }
    }
    let pcm = unsafe { slice::from_raw_parts(samples, n_samples) };
    // Unconditionally, including at 12 kHz. `resample_f32_to_12k`
    // interpolates in f64 and peak-normalises to 0.8 full-scale before
    // quantising; the pre-v2 path took that route only when a resample
    // was needed and did a bare `(s * 32767.0) as i16` at 12 kHz. So a
    // quiet float buffer — a USB radio adapter at a low Windows volume,
    // which is the common case this normalisation exists for — lost
    // dynamic range precisely when no resampling was required.
    let audio = mfsk_core::engine::dsp::resample::resample_f32_to_12k(pcm, sample_rate);
    if let Err(e) = in_pool_mut(|| run_decode(d, &audio, &p)) {
        return d.fail(e);
    }
    unsafe { emit(d, out, out_cap, out_len) }
}

/// FEC information bits for the `index`-th row of the last decode.
///
/// The raw bits are deliberately not a row field: they are 91 or 101
/// bytes and only a caller doing subtraction or persistence wants them.
/// `MfskDecode::info_bits` says how many there are.
///
/// # Safety
/// `out` must be `cap` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_copy_info(
    dec: *const MfskDecodeSession,
    index: usize,
    out: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(d) = v2_ref(dec) else {
        set_error("mfsk_session_copy_info: null decoder handle");
        return MfskStatus::InvalidArg;
    };
    let Some((info, _)) = d.last.get(index) else {
        set_error("mfsk_session_copy_info: index past the last decode's row count");
        return MfskStatus::InvalidArg;
    };
    if !out_len.is_null() {
        unsafe { *out_len = info.len() };
    }
    if info.len() > cap || out.is_null() {
        set_error("mfsk_session_copy_info: buffer too small; *out_len is the size needed");
        return MfskStatus::InvalidArg;
    }
    unsafe { ptr::copy_nonoverlapping(info.as_ptr(), out, info.len()) };
    MfskStatus::Ok
}

/// Teach the handle a callsign, so a later slot's `<...>` reference to
/// it resolves.
///
/// Decoded messages populate the table automatically; this is for
/// callsigns known from outside the decoder — a band map, a previous
/// session, an operator's log.
///
/// # Safety
/// `call` must be a valid NUL-terminated C string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_add_callsign(
    dec: *mut MfskDecodeSession,
    call: *const c_char,
) -> MfskStatus {
    let Some(d) = v2(dec) else {
        set_error("mfsk_session_add_callsign: null decoder handle");
        return MfskStatus::InvalidArg;
    };
    if call.is_null() {
        return d.fail("mfsk_session_add_callsign: call is NULL");
    }
    let Ok(s) = (unsafe { CStr::from_ptr(call) }).to_str() else {
        return d.fail("mfsk_session_add_callsign: call is not valid UTF-8");
    };
    d.hashes.insert(s);
    MfskStatus::Ok
}

// ──────────────────────────────────────────────────────────────────────────
// Entry points for modes that are not driven by the decode session
//
// Q65 takes a nominal start sample and a time tolerance and reports
// `start_sample` rather than `dt`; WSPR/JT9/JT65 have no builder at all,
// and JT9/JT65 are fixed-carrier point decodes rather than searches.
// Forcing them through one `session_decode(params)` would recreate the
// "eleven options, six silently ignored" failure this redesign exists
// to end, so they keep their own shapes — and `MFSK_CAP_DECODE_HANDLE`
// is the bit that tells a caller which is which.
//
// What they now share is the row type, so a host has one result struct
// for every mode rather than one per family.
// ──────────────────────────────────────────────────────────────────────────

/// A decode from a mode with no FEC information block to report.
fn simple_row(mode: MfskMode, freq_hz: f32, dt_sec: f32, snr_db: f32, text: &str) -> MfskDecode {
    let mut d = MfskDecode {
        size: core::mem::size_of::<MfskDecode>() as u32,
        mode,
        text: [0; MFSK_DECODE_TEXT_LEN],
        freq_hz,
        dt_sec,
        snr_db,
        sync_score: 0.0,
        sync_cv: 0.0,
        hard_errors: 0,
        info_bits: 0,
        pass: 0,
        flags: 0,
    };
    write_field(&mut d.text, text);
    d
}

/// Copy rows into the caller's array, reporting the count needed.
///
/// # Safety
/// `out` must be `cap` writable `MfskDecode`, or null when `cap` is 0.
unsafe fn emit_rows(
    rows: &[MfskDecode],
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    if !out_len.is_null() {
        unsafe { *out_len = rows.len() };
    }
    if rows.len() > cap || (out.is_null() && !rows.is_empty()) {
        set_error("decode: output buffer too small; *out_len is the count needed");
        return MfskStatus::InvalidArg;
    }
    for (i, r) in rows.iter().enumerate() {
        unsafe { write_size_versioned(out.add(i), r) };
    }
    MfskStatus::Ok
}

/// Resample caller PCM to the 12 kHz f32 the non-session decoders want.
///
/// # Safety
/// `samples` must be `n` readable `int16_t`.
unsafe fn pcm_to_12k_f32(samples: *const i16, n: usize, rate: u32) -> Option<Vec<f32>> {
    if samples.is_null() {
        return None;
    }
    let pcm = unsafe { slice::from_raw_parts(samples, n) };
    Some(if rate == 12_000 {
        pcm.iter().map(|&s| s as f32 / 32768.0).collect()
    } else {
        mfsk_core::engine::dsp::resample::resample_i16_to_12k_f32(pcm, rate)
    })
}

/// Scan a 120 s WSPR slot.
///
/// # Safety
/// `samples` must be `n_samples` readable `int16_t`; `out` must be `cap`
/// writable `MfskDecode`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_wspr_decode(
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(audio) = (unsafe { pcm_to_12k_f32(samples, n_samples, sample_rate) }) else {
        set_error("mfsk_wspr_decode: samples is NULL");
        return MfskStatus::InvalidArg;
    };
    let rows: Vec<MfskDecode> = mfsk_core::wspr::decode::decode_scan_default(&audio, 12_000)
        .iter()
        .map(|d| {
            simple_row(
                MfskMode::Wspr,
                d.freq_hz,
                d.start_sample as f32 / 12_000.0,
                d.snr_db,
                &d.message.to_string(),
            )
        })
        .collect();
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

/// Decode a JT9 or JT65 frame at a known carrier.
///
/// These are **point decodes, not searches** — WSJT-X's own JT9/JT65
/// front end finds the carrier, and this crate's entry points take it.
/// The pre-v2 ABI reached them only through the generic decode call and
/// hardcoded 1500 Hz (JT9) and 1270 Hz (JT65) with no way to say
/// otherwise, which is why the frequency is an argument here. `snr_db`
/// is reported as 0: neither decoder estimates one.
///
/// # Safety
/// As [`mfsk_wspr_decode`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jt9_decode_at(
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    freq_hz: f32,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(audio) = (unsafe { pcm_to_12k_f32(samples, n_samples, sample_rate) }) else {
        set_error("mfsk_jt9_decode_at: samples is NULL");
        return MfskStatus::InvalidArg;
    };
    let rows: Vec<MfskDecode> = mfsk_core::jt9::decode_at(&audio, 12_000, 0, freq_hz)
        .map(|m| vec![simple_row(MfskMode::Jt9, freq_hz, 0.0, 0.0, &m.to_string())])
        .unwrap_or_default();
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

/// See [`mfsk_jt9_decode_at`].
///
/// # Safety
/// As [`mfsk_wspr_decode`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jt65_decode_at(
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    freq_hz: f32,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(audio) = (unsafe { pcm_to_12k_f32(samples, n_samples, sample_rate) }) else {
        set_error("mfsk_jt65_decode_at: samples is NULL");
        return MfskStatus::InvalidArg;
    };
    let rows: Vec<MfskDecode> = mfsk_core::jt65::decode_at(&audio, 12_000, 0, freq_hz)
        .map(|m| {
            vec![simple_row(
                MfskMode::Jt65,
                freq_hz,
                0.0,
                0.0,
                &m.to_string(),
            )]
        })
        .unwrap_or_default();
    unsafe { emit_rows(&rows, out, cap, out_len) }
}

/// As [`mode_of`], for the Q65 sub-mode tag. Same reason: a C caller
/// can put any integer in an `enum` parameter, and matching an
/// out-of-range one as a Rust enum is undefined behaviour.
fn q65_submode_of(raw: u32) -> Option<MfskQ65SubMode> {
    use MfskQ65SubMode::*;
    [A15, A30, A60, B60, C60, D60, E60, D120, E120, A300]
        .into_iter()
        .find(|m| *m as u32 == raw)
}

/// As [`mode_of`], for the fading model.
fn q65_fading_of(raw: u32) -> Option<MfskQ65FadingModel> {
    [MfskQ65FadingModel::Gaussian, MfskQ65FadingModel::Lorentzian]
        .into_iter()
        .find(|m| *m as u32 == raw)
}

/// Which `MfskMode` a Q65 sub-mode tag addresses, so a Q65 row carries
/// the same mode identity as every other row.
fn q65_mode(sub: MfskQ65SubMode) -> MfskMode {
    match sub {
        MfskQ65SubMode::A15 => MfskMode::Q65a15,
        MfskQ65SubMode::A30 => MfskMode::Q65a30,
        MfskQ65SubMode::A60 => MfskMode::Q65a60,
        MfskQ65SubMode::B60 => MfskMode::Q65b60,
        MfskQ65SubMode::C60 => MfskMode::Q65c60,
        MfskQ65SubMode::D60 => MfskMode::Q65d60,
        MfskQ65SubMode::E60 => MfskMode::Q65e60,
        MfskQ65SubMode::D120 => MfskMode::Q65d120,
        MfskQ65SubMode::E120 => MfskMode::Q65e120,
        MfskQ65SubMode::A300 => MfskMode::Q65a300,
    }
}

fn q65_rows(sub: MfskQ65SubMode, ds: &[mfsk_core::q65::Q65Result]) -> Vec<MfskDecode> {
    ds.iter()
        .map(|d| {
            simple_row(
                q65_mode(sub),
                d.freq_hz,
                d.start_sample as f32 / 12_000.0,
                d.snr_db,
                &d.message,
            )
        })
        .collect()
}

// ──────────────────────────────────────────────────────────────────────────
// Transmit: the three-stage zero-allocation pipeline (FFI v2 slice 4)
//
// `mfsk-ffi-ft8` already had the better of this repo's two TX designs —
// pack → tones → PCM, each stage writing into a buffer the caller owns.
// The `mfsk-ffi` side had seven heap-allocating `mfsk_encode_*`
// functions that accepted only the three-string convenience path, so a
// caller with a message that is not "call1 call2 report" had no way in,
// and FST4 reached 60A alone.
//
// Both are replaced by one pipeline over `MfskMode`. Stage 2 and 3 exist
// for the modes whose message codec is WSJT-77 — FT8, FT4 and all five
// FST4 sub-modes — because those are the protocols that expose a tone
// sequence at all; `mfsk_symbol_count` returns 0 for the rest, which is
// how a caller asks.
// ──────────────────────────────────────────────────────────────────────────

/// The GFSK shaping a mode synthesises with, for the FST4 family.
fn fst4_gfsk(mode: MfskMode) -> Option<&'static mfsk_core::engine::dsp::gfsk::GfskCfg> {
    use mfsk_core::fst4::encode as f;
    Some(match mode {
        MfskMode::Fst4s15 => &f::FST4_15_GFSK,
        MfskMode::Fst4s30 => &f::FST4_30_GFSK,
        MfskMode::Fst4s60 => &f::FST4_60A_GFSK,
        MfskMode::Fst4s120 => &f::FST4_120_GFSK,
        MfskMode::Fst4s300 => &f::FST4_300_GFSK,
        _ => return None,
    })
}

/// Channel symbols per frame, or 0 for a mode with no exposed tone
/// stage.
///
/// A non-zero answer is what says [`mfsk_message_to_tones`] and
/// [`mfsk_tones_to_i16`] apply. WSPR, JT9, JT65 and Q65 synthesise from
/// their own message codecs in one step and report 0 here.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_symbol_count(mode: u32) -> usize {
    let Some(mode) = mode_of(mode) else {
        return 0;
    };
    if !mode_has_tone_stage(mode) {
        return 0;
    }
    mode_meta(mode).map(|m| m.n_symbols as usize).unwrap_or(0)
}

fn mode_has_tone_stage(mode: MfskMode) -> bool {
    matches!(
        mode,
        MfskMode::Ft8
            | MfskMode::Ft4
            | MfskMode::Fst4s15
            | MfskMode::Fst4s30
            | MfskMode::Fst4s60
            | MfskMode::Fst4s120
            | MfskMode::Fst4s300
    )
}

/// Samples a full frame synthesises to at 12 kHz — the buffer size
/// [`mfsk_tones_to_i16`] needs. 0 if the mode has no tone stage.
///
/// **Ask rather than assume.** The five FST4 sub-modes differ by a
/// factor of 30 here (720 → 21 504 samples per symbol), so a constant
/// baked for 60A is silently wrong for the other four — which is
/// exactly the trap the old `tones_to_f32` wrapper carried.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_synth_output_len(mode: u32) -> usize {
    let Some(mode) = mode_of(mode) else {
        return 0;
    };
    match mode {
        MfskMode::Ft8 => mfsk_core::ft8::wave_gen::TONES_OUTPUT_LEN,
        MfskMode::Ft4 => mfsk_core::ft4::encode::TONES_OUTPUT_LEN,
        _ => match fst4_gfsk(mode) {
            Some(cfg) => mfsk_core::fst4::encode::synth_sample_count(cfg),
            None => 0,
        },
    }
}

/// Copy a packed 77-bit message into caller memory.
///
/// # Safety
/// `out` must be 77 writable bytes.
unsafe fn put_message77(msg: &[u8; 77], out: *mut u8) {
    unsafe { ptr::copy_nonoverlapping(msg.as_ptr(), out, 77) };
}

unsafe fn arg_str(p: *const c_char, what: &str) -> Result<&'static str, MfskStatus> {
    if p.is_null() {
        set_error(format!("{what}: null string argument"));
        return Err(MfskStatus::InvalidArg);
    }
    match unsafe { CStr::from_ptr(p) }.to_str() {
        Ok(s) => Ok(s),
        Err(_) => {
            set_error(format!("{what}: not valid UTF-8"));
            Err(MfskStatus::InvalidArg)
        }
    }
}

// Written out rather than generated by a macro, and that is not a
// style preference: cbindgen parses this crate syntactically and
// **cannot expand `macro_rules!`**, so a macro-generated
// `extern "C"` function exists in the library and never reaches
// `mfsk.h`. A C consumer cannot call what the header does not declare.
// Caught by the C++ driver failing to compile.

/// Pack a standard exchange: `call1 call2 report` (WSJT type 1/2).
///
/// Writes 77 bytes, one bit per byte, to `out_message77` — the form
/// every stage-2 call takes.
///
/// # Safety
/// Strings must be NUL-terminated; `out_message77` must be 77 writable
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_pack77(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    out_message77: *mut u8,
) -> MfskStatus {
    if out_message77.is_null() {
        set_error("mfsk_pack77: out_message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let a = match unsafe { arg_str(call1, "mfsk_pack77") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let b = match unsafe { arg_str(call2, "mfsk_pack77") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let c = match unsafe { arg_str(report, "mfsk_pack77") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let Some(msg) = mfsk_core::msg::wsjt77::pack77(a, b, c) else {
        set_error("mfsk_pack77: the message does not fit this format");
        return MfskStatus::InvalidArg;
    };
    unsafe { put_message77(&msg, out_message77) };
    MfskStatus::Ok
}

/// Pack a type-1 message: `call1 call2 grid`.
///
/// # Safety
/// As [`mfsk_pack77`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_pack77_type1(
    call1: *const c_char,
    call2: *const c_char,
    grid: *const c_char,
    out_message77: *mut u8,
) -> MfskStatus {
    if out_message77.is_null() {
        set_error("mfsk_pack77_type1: out_message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let a = match unsafe { arg_str(call1, "mfsk_pack77_type1") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let b = match unsafe { arg_str(call2, "mfsk_pack77_type1") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let g = match unsafe { arg_str(grid, "mfsk_pack77_type1") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let Some(msg) = mfsk_core::msg::wsjt77::pack77_type1(a, b, g) else {
        set_error("mfsk_pack77_type1: the message does not fit this format");
        return MfskStatus::InvalidArg;
    };
    unsafe { put_message77(&msg, out_message77) };
    MfskStatus::Ok
}

/// Pack up to 13 characters of free text.
///
/// # Safety
/// As [`mfsk_pack77`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_pack77_free_text(
    text: *const c_char,
    out_message77: *mut u8,
) -> MfskStatus {
    if out_message77.is_null() {
        set_error("mfsk_pack77_free_text: out_message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let t = match unsafe { arg_str(text, "mfsk_pack77_free_text") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    let Some(msg) = mfsk_core::msg::wsjt77::pack77_free_text(t) else {
        set_error("mfsk_pack77_free_text: the message does not fit this format");
        return MfskStatus::InvalidArg;
    };
    unsafe { put_message77(&msg, out_message77) };
    MfskStatus::Ok
}

/// Pack a type-4 message: one non-standard callsign in full, plus a
/// **hashed** reference to the standard one.
///
/// The hashed half decodes as `<...>` unless the receiving session has
/// seen that callsign — see [`mfsk_session_add_callsign`].
///
/// # Safety
/// Strings must be NUL-terminated; `out_message77` must be 77 writable
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_pack77_type4(
    nonstd_call: *const c_char,
    std_call: *const c_char,
    report: *const c_char,
    is_cq: bool,
    out_message77: *mut u8,
) -> MfskStatus {
    if out_message77.is_null() {
        set_error("mfsk_pack77_type4: out_message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let nonstd = match unsafe { arg_str(nonstd_call, "mfsk_pack77_type4") } {
        Ok(s) => s,
        Err(e) => return e,
    };
    // `std_call` and `report` may legitimately be empty for a CQ.
    let std_s = if std_call.is_null() {
        ""
    } else {
        match unsafe { arg_str(std_call, "mfsk_pack77_type4") } {
            Ok(s) => s,
            Err(e) => return e,
        }
    };
    let rep = if report.is_null() {
        ""
    } else {
        match unsafe { arg_str(report, "mfsk_pack77_type4") } {
            Ok(s) => s,
            Err(e) => return e,
        }
    };
    let Some(msg) = mfsk_core::msg::wsjt77::pack77_type4(nonstd, std_s, rep, is_cq) else {
        set_error("mfsk_pack77_type4: the message does not fit this format");
        return MfskStatus::InvalidArg;
    };
    unsafe { put_message77(&msg, out_message77) };
    MfskStatus::Ok
}

/// Render a packed 77-bit message as text.
///
/// Pass a session to resolve hashed `<...>` callsigns from its table;
/// `session` may be NULL, in which case they stay unresolved. Writes at
/// most `cap` bytes including the NUL, and reports the size needed if
/// that is not enough.
///
/// # Safety
/// `message77` must be 77 readable bytes; `out` must be `cap` writable
/// bytes; `session`, if non-null, must be a live session.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_unpack77(
    session: *const MfskDecodeSession,
    message77: *const u8,
    out: *mut c_char,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    if message77.is_null() {
        set_error("mfsk_unpack77: message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let bits = unsafe { slice::from_raw_parts(message77, 77) };
    let text = match v2_ref(session) {
        Some(d) => mfsk_core::msg::wsjt77::unpack77_with_hash(bits, &d.hashes),
        None => mfsk_core::msg::wsjt77::unpack77(bits),
    };
    let Some(text) = text else {
        set_error("mfsk_unpack77: not a decodable 77-bit message");
        return MfskStatus::DecodeFailed;
    };
    let needed = text.len() + 1;
    if !out_len.is_null() {
        unsafe { *out_len = needed };
    }
    if out.is_null() || cap < needed {
        set_error("mfsk_unpack77: buffer too small; *out_len is the size needed");
        return MfskStatus::InvalidArg;
    }
    unsafe {
        ptr::copy_nonoverlapping(text.as_ptr() as *const c_char, out, text.len());
        *out.add(text.len()) = 0;
    }
    MfskStatus::Ok
}

/// Stage 2: a packed message becomes this mode's channel symbols.
///
/// `mfsk_symbol_count(mode)` is the required capacity; 0 means the mode
/// has no tone stage.
///
/// # Safety
/// `message77` must be 77 readable bytes; `out_itone` must be `cap`
/// writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_message_to_tones(
    mode: u32,
    message77: *const u8,
    out_itone: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(m) = mode_of(mode) else {
        set_error("mfsk_message_to_tones: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    if !mode_has_tone_stage(m) {
        set_error("mfsk_message_to_tones: this mode has no tone stage — see mfsk_symbol_count");
        return MfskStatus::Unsupported;
    }
    if message77.is_null() {
        set_error("mfsk_message_to_tones: message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let bits = unsafe { slice::from_raw_parts(message77, 77) };
    let mut msg = [0u8; 77];
    msg.copy_from_slice(bits);

    let tones: Vec<u8> = match m {
        MfskMode::Ft8 => mfsk_core::ft8::wave_gen::message_to_tones(&msg).to_vec(),
        MfskMode::Ft4 => mfsk_core::ft4::encode::message_to_tones(&msg),
        _ => mfsk_core::fst4::encode::message_to_tones(&msg),
    };
    if !out_len.is_null() {
        unsafe { *out_len = tones.len() };
    }
    if out_itone.is_null() || cap < tones.len() {
        set_error("mfsk_message_to_tones: buffer too small; *out_len is the size needed");
        return MfskStatus::InvalidArg;
    }
    unsafe { ptr::copy_nonoverlapping(tones.as_ptr(), out_itone, tones.len()) };
    MfskStatus::Ok
}

/// Shared body for the two stage-3 calls.
///
/// Validates the mode, the tone count and the buffer, then hands back
/// the destination slice. Written as a helper with the two public
/// functions spelled out, because cbindgen cannot expand
/// `macro_rules!` — a macro-generated `extern "C"` function never
/// reaches the header, and a C consumer cannot call what is not
/// declared.
fn synth_check(mode: u32, n_tones: usize, cap: usize, what: &str) -> Result<usize, MfskStatus> {
    let Some(m) = mode_of(mode) else {
        set_error(format!("{what}: not a mode this library knows"));
        return Err(MfskStatus::InvalidArg);
    };
    if !mode_has_tone_stage(m) {
        set_error(format!(
            "{what}: this mode has no tone stage — see mfsk_symbol_count"
        ));
        return Err(MfskStatus::Unsupported);
    }
    let want = mfsk_symbol_count(mode);
    if n_tones != want {
        set_error(format!(
            "{what}: this mode has {want} channel symbols, got {n_tones}"
        ));
        return Err(MfskStatus::InvalidArg);
    }
    let need = mfsk_synth_output_len(mode);
    if cap < need {
        set_error(format!(
            "{what}: buffer too small; *out_len is the size needed"
        ));
        return Err(MfskStatus::InvalidArg);
    }
    Ok(need)
}

/// Stage 3: channel symbols become 16-bit PCM at 12 kHz.
///
/// `mfsk_synth_output_len(mode)` is the required capacity. The
/// synthesis writes straight into your buffer — nothing is allocated
/// and nothing has to be freed.
///
/// # Safety
/// `itone` must be `n_tones` readable bytes; `out` must be `cap`
/// writable `int16_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_tones_to_i16(
    mode: u32,
    itone: *const u8,
    n_tones: usize,
    freq_hz: f32,
    amplitude: i16,
    out: *mut i16,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    if !out_len.is_null() {
        unsafe { *out_len = mfsk_synth_output_len(mode) };
    }
    if itone.is_null() || out.is_null() {
        set_error("mfsk_tones_to_i16: null pointer");
        return MfskStatus::InvalidArg;
    }
    let need = match synth_check(mode, n_tones, cap, "mfsk_tones_to_i16") {
        Ok(n) => n,
        Err(e) => return e,
    };
    let tones = unsafe { slice::from_raw_parts(itone, n_tones) };
    let dst = unsafe { slice::from_raw_parts_mut(out, need) };
    match mode_of(mode).expect("checked") {
        MfskMode::Ft8 => {
            let mut fixed = [0u8; 79]; // FT8 N_SYMBOLS, pinned by synth_check
            fixed.copy_from_slice(tones);
            mfsk_core::ft8::wave_gen::tones_to_i16_into(dst, &fixed, freq_hz, amplitude);
        }
        MfskMode::Ft4 => mfsk_core::ft4::encode::tones_to_i16_into(dst, tones, freq_hz, amplitude),
        m => mfsk_core::fst4::encode::tones_to_i16_into(
            dst,
            tones,
            freq_hz,
            amplitude,
            fst4_gfsk(m).expect("checked"),
        ),
    }
    MfskStatus::Ok
}

/// Stage 3: channel symbols become 32-bit float PCM at 12 kHz.
///
/// # Safety
/// As [`mfsk_tones_to_i16`], with `out` as `float`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_tones_to_f32(
    mode: u32,
    itone: *const u8,
    n_tones: usize,
    freq_hz: f32,
    amplitude: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    if !out_len.is_null() {
        unsafe { *out_len = mfsk_synth_output_len(mode) };
    }
    if itone.is_null() || out.is_null() {
        set_error("mfsk_tones_to_f32: null pointer");
        return MfskStatus::InvalidArg;
    }
    let need = match synth_check(mode, n_tones, cap, "mfsk_tones_to_f32") {
        Ok(n) => n,
        Err(e) => return e,
    };
    let tones = unsafe { slice::from_raw_parts(itone, n_tones) };
    let dst = unsafe { slice::from_raw_parts_mut(out, need) };
    match mode_of(mode).expect("checked") {
        MfskMode::Ft8 => {
            let mut fixed = [0u8; 79];
            fixed.copy_from_slice(tones);
            mfsk_core::ft8::wave_gen::tones_to_f32_into(dst, &fixed, freq_hz, amplitude);
        }
        MfskMode::Ft4 => mfsk_core::ft4::encode::tones_to_f32_into(dst, tones, freq_hz, amplitude),
        m => mfsk_core::fst4::encode::tones_to_f32_into(
            dst,
            tones,
            freq_hz,
            amplitude,
            fst4_gfsk(m).expect("checked"),
        ),
    }
    MfskStatus::Ok
}

// ──────────────────────────────────────────────────────────────────────────
// Streaming ingestion and the slot grid (FFI v2 slice 5)
//
// `mfsk-ffi-ft8` had the only streaming front end in this repo, sized
// for FT8 and taking i16 alone. This generalises it over `MfskMode` —
// the ring is sized from `slot_samples_12k`, so FST4-300's 3.6 M-sample
// slot works the same way FT4's 90 000-sample one does.
//
// **Time enters as a parameter and is never read.** No `Instant`, no
// `SystemTime`, no clock of any kind: the host says what UTC second the
// next sample belongs to, and the grid does arithmetic. That is what
// keeps this usable from wasm, from `no_std`, and from an iOS app that
// was backgrounded for four minutes — and it is the same choice
// `BudgetCheck` makes for the decode deadline.
// ──────────────────────────────────────────────────────────────────────────

/// Opaque streaming-capture handle.
pub struct MfskStream {
    _marker: core::marker::PhantomData<*mut ()>,
}

struct StreamInner {
    mode: MfskMode,
    /// Samples in one slot at 12 kHz — the ring's capacity and the
    /// amount `take_slot` hands over.
    slot_samples: usize,
    /// `None` when the source is already 12 kHz.
    resampler: Option<mfsk_core::engine::dsp::resample::LinearResamplerI16To12k>,
    ring: Vec<i16>,
    /// Total 12 kHz samples ever pushed. The grid is arithmetic on
    /// this, so it survives the ring wrapping.
    pushed: u64,
    /// Samples consumed by `take_slot`, so `pushed - taken` is what is
    /// available.
    taken: u64,
    /// UTC second that sample index `epoch_at` fell on, if the host has
    /// said. `None` means free-running from the first sample, which is
    /// exactly right for replaying a recording.
    epoch_utc: Option<f64>,
    epoch_at: u64,
}

impl StreamInner {
    /// UTC of the sample at absolute index `i`, if the epoch is known.
    fn utc_of(&self, i: u64) -> f64 {
        match self.epoch_utc {
            Some(t0) => t0 + (i as f64 - self.epoch_at as f64) / 12_000.0,
            None => i as f64 / 12_000.0,
        }
    }
}

fn stream_inner(s: *mut MfskStream) -> Option<&'static mut StreamInner> {
    unsafe { (s as *mut StreamInner).as_mut() }
}

fn stream_ref(s: *const MfskStream) -> Option<&'static StreamInner> {
    unsafe { (s as *const StreamInner).as_ref() }
}

/// Open a capture stream for `mode`, accepting audio at `sample_rate`.
///
/// The ring holds exactly one slot. Pushing more than that before
/// taking one overwrites the oldest audio, which is the right failure
/// for a live receiver: the newest slot is the one worth decoding.
///
/// Returns NULL and writes the reason to `out_status` on failure.
///
/// # Safety
/// `out_status` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_open(
    mode: u32,
    sample_rate: u32,
    out_status: *mut MfskStatus,
) -> *mut MfskStream {
    let report = |st: MfskStatus| {
        if !out_status.is_null() {
            unsafe { *out_status = st };
        }
    };
    let Some(m) = mode_of(mode) else {
        set_error("mfsk_stream_open: not a mode this library knows");
        report(MfskStatus::InvalidArg);
        return ptr::null_mut();
    };
    let Some(meta) = mode_meta(m) else {
        set_error("mfsk_stream_open: no such mode in this build");
        report(MfskStatus::UnknownProtocol);
        return ptr::null_mut();
    };
    if meta.profile.caps & mfsk_core::registry::caps::DECODE_HANDLE == 0 {
        set_error(
            "mfsk_stream_open: this mode has no decode handle to feed — check \
             MFSK_CAP_DECODE_HANDLE",
        );
        report(MfskStatus::Unsupported);
        return ptr::null_mut();
    }
    if sample_rate == 0 {
        set_error("mfsk_stream_open: sample_rate is 0");
        report(MfskStatus::InvalidArg);
        return ptr::null_mut();
    }
    let slot_samples = meta.slot_samples_12k as usize;
    report(MfskStatus::Ok);
    Box::into_raw(Box::new(StreamInner {
        mode: m,
        slot_samples,
        resampler: (sample_rate != 12_000)
            .then(|| mfsk_core::engine::dsp::resample::LinearResamplerI16To12k::new(sample_rate)),
        ring: Vec::with_capacity(slot_samples),
        pushed: 0,
        taken: 0,
        epoch_utc: None,
        epoch_at: 0,
    })) as *mut MfskStream
}

/// Release a stream. Null is a no-op.
///
/// # Safety
/// `s` must be a handle from [`mfsk_stream_open`], released once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_close(s: *mut MfskStream) {
    if !s.is_null() {
        drop(unsafe { Box::from_raw(s as *mut StreamInner) });
    }
}

fn ring_push(st: &mut StreamInner, src: &[i16]) {
    st.ring.extend_from_slice(src);
    st.pushed += src.len() as u64;
    // Keep at most one slot: a live receiver wants the newest audio,
    // and holding more would only delay the decode.
    if st.ring.len() > st.slot_samples {
        let drop_n = st.ring.len() - st.slot_samples;
        st.ring.drain(..drop_n);
        st.taken += drop_n as u64;
    }
}

/// Push 16-bit PCM at the rate the stream was opened with.
///
/// # Safety
/// `samples` must be `n` readable `int16_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_push_i16(
    s: *mut MfskStream,
    samples: *const i16,
    n: usize,
) -> MfskStatus {
    let Some(st) = stream_inner(s) else {
        set_error("mfsk_stream_push_i16: null stream");
        return MfskStatus::InvalidArg;
    };
    if n == 0 {
        return MfskStatus::Ok;
    }
    if samples.is_null() {
        set_error("mfsk_stream_push_i16: null samples");
        return MfskStatus::InvalidArg;
    }
    let src = unsafe { slice::from_raw_parts(samples, n) };
    match st.resampler.as_mut() {
        None => ring_push(st, src),
        Some(_) => {
            // Resample in bounded chunks so a long push does not
            // allocate a second copy of the whole buffer.
            let mut scratch = [0i16; 512];
            let mut pos = 0;
            while pos < src.len() {
                let r = st.resampler.as_mut().expect("checked");
                let (consumed, produced) = r.process(&src[pos..], &mut scratch);
                if consumed == 0 && produced == 0 {
                    break;
                }
                let chunk: Vec<i16> = scratch[..produced].to_vec();
                ring_push(st, &chunk);
                pos += consumed;
            }
        }
    }
    MfskStatus::Ok
}

/// Push 32-bit float PCM, nominally `-1.0..=1.0`.
///
/// # Safety
/// `samples` must be `n` readable `float`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_push_f32(
    s: *mut MfskStream,
    samples: *const f32,
    n: usize,
) -> MfskStatus {
    if samples.is_null() && n != 0 {
        set_error("mfsk_stream_push_f32: null samples");
        return MfskStatus::InvalidArg;
    }
    if n == 0 {
        return MfskStatus::Ok;
    }
    let src = unsafe { slice::from_raw_parts(samples, n) };
    let as_i16: Vec<i16> = src
        .iter()
        .map(|&x| (x * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    unsafe { mfsk_stream_push_i16(s, as_i16.as_ptr(), as_i16.len()) }
}

/// How many 12 kHz samples are buffered.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_stream_buffered(s: *const MfskStream) -> usize {
    stream_ref(s).map(|st| st.ring.len()).unwrap_or(0)
}

/// Tell the stream what UTC second the **next** sample pushed belongs
/// to, so slot boundaries land where the protocol says.
///
/// Without this the grid free-runs from the first sample, which is
/// exactly right for replaying a recording and wrong for a live
/// receiver. Call it whenever your clock is resynchronised; the grid
/// re-anchors from that point rather than shifting what is already
/// buffered.
///
/// # Safety
/// `s` must be a live stream or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_set_epoch(s: *mut MfskStream, utc_seconds: f64) {
    if let Some(st) = stream_inner(s) {
        st.epoch_utc = Some(utc_seconds);
        st.epoch_at = st.pushed;
    }
}

/// Whether a whole slot is buffered and ready to take.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_stream_slot_ready(s: *const MfskStream) -> bool {
    stream_ref(s)
        .map(|st| st.ring.len() >= st.slot_samples)
        .unwrap_or(false)
}

/// Take the buffered slot, copying it into `out` and reporting the UTC
/// second its first sample fell on.
///
/// Returns the number of samples written, or 0 if no slot is ready or
/// `cap` is too small — ask [`mfsk_stream_slot_ready`] first and size
/// from `MfskModeInfo::slot_samples_12k`.
///
/// # Safety
/// `out` must be `cap` writable `int16_t`; `out_slot_start_utc` may be
/// null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_take_slot_i16(
    s: *mut MfskStream,
    out: *mut i16,
    cap: usize,
    out_slot_start_utc: *mut f64,
) -> usize {
    let Some(st) = stream_inner(s) else {
        return 0;
    };
    if st.ring.len() < st.slot_samples || out.is_null() || cap < st.slot_samples {
        return 0;
    }
    let n = st.slot_samples;
    if !out_slot_start_utc.is_null() {
        let first = st.taken;
        unsafe { *out_slot_start_utc = st.utc_of(first) };
    }
    unsafe { ptr::copy_nonoverlapping(st.ring.as_ptr(), out, n) };
    st.ring.drain(..n);
    st.taken += n as u64;
    n
}

/// Drop everything buffered, keeping the epoch.
///
/// # Safety
/// `s` must be a live stream or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_clear(s: *mut MfskStream) {
    if let Some(st) = stream_inner(s) {
        st.taken += st.ring.len() as u64;
        st.ring.clear();
    }
}

/// Decode the stream's buffered slot directly, without copying it out
/// and back in.
///
/// Fused on purpose: FST4-300's slot is 3 600 000 samples, and a
/// take-then-decode round trip moves 7 MB for nothing.
///
/// Returns `MFSK_STATUS_UNSUPPORTED` with `*out_len = 0` when no slot
/// is ready yet, so a caller can poll this instead of
/// [`mfsk_stream_slot_ready`] if it prefers.
///
/// # Safety
/// As [`mfsk_session_decode_i16`], plus `stream` must be a live stream
/// opened for the same mode as `dec`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_session_decode_stream(
    dec: *mut MfskDecodeSession,
    stream: *mut MfskStream,
    params: *const MfskDecodeParams,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
    out_slot_start_utc: *mut f64,
) -> MfskStatus {
    let Some(d) = v2(dec) else {
        set_error("mfsk_session_decode_stream: null session handle");
        return MfskStatus::InvalidArg;
    };
    let Some(st) = stream_inner(stream) else {
        return d.fail("mfsk_session_decode_stream: null stream handle");
    };
    if st.mode != d.mode {
        return d.fail(
            "mfsk_session_decode_stream: the stream and the session are for different modes",
        );
    }
    if !out_len.is_null() {
        unsafe { *out_len = 0 };
    }
    if st.ring.len() < st.slot_samples {
        set_error("mfsk_session_decode_stream: no whole slot buffered yet");
        return MfskStatus::Unsupported;
    }
    let mut p = d.params;
    if !params.is_null() {
        if let Err(e) = unsafe { read_params(params, &mut p) } {
            return d.fail(format!("mfsk_session_decode_stream: {e}"));
        }
        if let Err(e) = validate_params(d.mode, &p) {
            return d.fail(format!("mfsk_session_decode_stream: {e}"));
        }
    }
    if !out_slot_start_utc.is_null() {
        unsafe { *out_slot_start_utc = st.utc_of(st.taken) };
    }
    let slot: Vec<i16> = st.ring[..st.slot_samples].to_vec();
    st.ring.drain(..st.slot_samples);
    st.taken += st.slot_samples as u64;

    if let Err(e) = in_pool_mut(|| run_decode(d, &slot, &p)) {
        return d.fail(e);
    }
    unsafe { emit(d, out, out_cap, out_len) }
}

// ──────────────────────────────────────────────────────────────────────────
// Runtime configuration (FFI v2 slice 6)
//
// The single most important mobile fix in this redesign, and there was
// no hook for it at any layer before.
//
// Even with `parallel` on, the decode used rayon's **global** pool:
// `num_cpus` threads with 2 MiB stacks each, spawned lazily on the
// first decode and never joined. On Android those threads are not
// attached to ART, so a callback from one cannot touch a JNIEnv; on
// iOS they sit outside GCD's quality-of-service classes, competing with
// the audio render thread; and on both they keep running after the app
// is backgrounded. A host had no way to say otherwise.
//
// `mfsk_runtime_configure` builds a private pool and every decode runs
// inside `pool.install(...)`, so thread count, stack size and the
// start/stop hooks are the caller's to set. The two hooks map straight
// onto rayon's `start_handler`/`exit_handler`, which is what makes
// `AttachCurrentThread`/`DetachCurrentThread` possible from JNI — and
// therefore what makes a callback from a worker thread legal there.
// ──────────────────────────────────────────────────────────────────────────

/// Called on each worker thread as it starts and as it exits.
///
/// On Android these are where a JNI consumer calls
/// `AttachCurrentThread` and `DetachCurrentThread`. `index` is rayon's
/// own worker index, stable for the life of the pool.
pub type MfskThreadHook = Option<unsafe extern "C" fn(index: u32, user_data: *mut c_void)>;

/// How the decode should use threads.
///
/// Size-versioned like every other growable struct here: set
/// `size = sizeof(MfskRuntimeConfig)`, or zero it and the library fills
/// `size` in — a zeroed struct means "rayon's defaults, no hooks",
/// which is the pre-v2 behaviour.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct MfskRuntimeConfig {
    /// `sizeof(MfskRuntimeConfig)` as the caller understands it.
    pub size: u32,
    /// Worker threads. 0 for rayon's default (`num_cpus`); **1 forces
    /// serial decoding**, which is also what a build without the
    /// `parallel` feature does.
    pub num_threads: u32,
    /// Stack bytes per worker. 0 for rayon's default, which is 2 MiB —
    /// `num_cpus * 2 MiB` of address space reserved on a phone before
    /// the first sample is decoded.
    pub thread_stack_bytes: u32,
    /// Called as each worker starts. See [`MfskThreadHook`].
    pub on_thread_start: MfskThreadHook,
    /// Called as each worker exits.
    pub on_thread_stop: MfskThreadHook,
    /// Passed to both hooks, untouched.
    pub thread_user: *mut c_void,
}

#[cfg(feature = "parallel")]
struct HookUser(*mut c_void);
#[cfg(feature = "parallel")]
unsafe impl Send for HookUser {}
#[cfg(feature = "parallel")]
unsafe impl Sync for HookUser {}

#[cfg(feature = "parallel")]
static POOL: std::sync::OnceLock<rayon::ThreadPool> = std::sync::OnceLock::new();

/// Run the decode on the configured pool, or directly if none was
/// configured — which keeps the pre-v2 behaviour exactly: rayon's
/// global pool, the right default for a desktop host and the wrong one
/// for a phone.
///
/// No `unsafe` and no `Send` wrapper, despite the closure borrowing the
/// session mutably: every field of `V2Decoder` is already `Send` — the
/// only one not inherently so is the C `user_data` pointer, and
/// `SyncUserData` carries that claim with its own reasoning. So
/// `&mut V2Decoder` is `Send` and the closure crosses on its own terms.
///
/// `install` moves the closure to one pool thread and blocks this one
/// until it returns, so the session is never touched from two threads
/// at once. What runs in parallel is the `par_iter` inside `mfsk_core`,
/// which is the whole reason the pool has to be installed here rather
/// than left to rayon's global one.
#[cfg(feature = "parallel")]
fn in_pool_mut<R: Send>(f: impl FnOnce() -> R + Send) -> R {
    match POOL.get() {
        Some(p) => p.install(f),
        None => f(),
    }
}

/// Without `parallel` there is one thread and nothing to install on —
/// a *stronger* contract than the pool provides, not a missing one.
#[cfg(not(feature = "parallel"))]
fn in_pool_mut<R>(f: impl FnOnce() -> R) -> R {
    f()
}

/// Configure the thread pool every subsequent decode runs on.
///
/// **Call once, before the first decode.** The pool is built on the
/// first call and kept for the life of the process; a second call
/// returns `MFSK_STATUS_UNSUPPORTED` rather than silently ignoring you,
/// because rayon cannot rebuild a pool threads may be parked in.
///
/// Pass NULL to mean "rayon's defaults", which is also what happens if
/// this is never called.
///
/// Returns `MFSK_STATUS_UNSUPPORTED` on a build without the `parallel`
/// feature — there is one thread there and nothing to configure, which
/// is a *stronger* contract rather than a missing one.
///
/// # Safety
/// `config` must be null or point to at least `config->size` readable
/// bytes. The two hooks, if set, must be safely callable from a thread
/// this library spawns, and `thread_user` must outlive the pool — which
/// is the life of the process.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_runtime_configure(config: *const MfskRuntimeConfig) -> MfskStatus {
    #[cfg(not(feature = "parallel"))]
    {
        let _ = config;
        set_error(
            "mfsk_runtime_configure: this build has no thread pool (built without \
             `parallel`), so decoding is already single-threaded",
        );
        MfskStatus::Unsupported
    }
    #[cfg(feature = "parallel")]
    {
        if POOL.get().is_some() {
            set_error(
                "mfsk_runtime_configure: already configured — rayon cannot rebuild a \
                 pool its threads may be parked in, so call this once before the \
                 first decode",
            );
            return MfskStatus::Unsupported;
        }

        let mut cfg = MfskRuntimeConfig {
            size: core::mem::size_of::<MfskRuntimeConfig>() as u32,
            num_threads: 0,
            thread_stack_bytes: 0,
            on_thread_start: None,
            on_thread_stop: None,
            thread_user: ptr::null_mut(),
        };
        if !config.is_null() {
            let full = core::mem::size_of::<MfskRuntimeConfig>();
            let declared = unsafe { core::ptr::read_unaligned(config as *const u32) } as usize;
            let n = if declared == 0 || declared > full {
                full
            } else {
                declared
            };
            unsafe {
                core::ptr::copy_nonoverlapping(
                    config as *const u8,
                    &mut cfg as *mut MfskRuntimeConfig as *mut u8,
                    n,
                );
            }
            cfg.size = full as u32;
        }

        let mut builder = rayon::ThreadPoolBuilder::new();
        if cfg.num_threads > 0 {
            builder = builder.num_threads(cfg.num_threads as usize);
        }
        if cfg.thread_stack_bytes > 0 {
            builder = builder.stack_size(cfg.thread_stack_bytes as usize);
        }
        let user = HookUser(cfg.thread_user);
        if let Some(start) = cfg.on_thread_start {
            let u = HookUser(user.0);
            builder = builder.start_handler(move |i| {
                // Bind the wrapper, not the field: edition-2021
                // closures capture disjoint fields, which would move
                // the bare `*mut c_void` and lose the `Send + Sync`
                // the wrapper exists to assert.
                let u = &u;
                unsafe { start(i as u32, u.0) }
            });
        }
        if let Some(stop) = cfg.on_thread_stop {
            let u = HookUser(user.0);
            builder = builder.exit_handler(move |i| {
                let u = &u;
                unsafe { stop(i as u32, u.0) }
            });
        }

        match builder.build() {
            Ok(pool) => {
                // `set` cannot fail: the `get` above ran on this thread
                // and nothing else writes this, but a lost race would
                // mean two pools, so report it rather than assume.
                if POOL.set(pool).is_err() {
                    set_error("mfsk_runtime_configure: raced with another call");
                    return MfskStatus::Unsupported;
                }
                MfskStatus::Ok
            }
            Err(e) => {
                set_error(format!("mfsk_runtime_configure: {e}"));
                MfskStatus::Internal
            }
        }
    }
}

/// How many worker threads decoding will use.
///
/// 1 means serial — either because this build has no `parallel` feature
/// or because [`mfsk_runtime_configure`] was told to. Useful for a host
/// deciding how much other work to run alongside.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_runtime_thread_count() -> u32 {
    #[cfg(feature = "parallel")]
    {
        match POOL.get() {
            Some(p) => p.current_num_threads() as u32,
            None => rayon::current_num_threads() as u32,
        }
    }
    #[cfg(not(feature = "parallel"))]
    {
        1
    }
}

/// Write synthesised f32 PCM into caller memory.
///
/// The seven `mfsk_encode_*` functions used to hand back a heap
/// `MfskSamples` the caller had to free. That is the same "a Rust
/// global-allocator pointer crosses the boundary" category the decode
/// rows shed, and it bites the same way: a wrapper that throws between
/// the call and the free leaks.
///
/// # Safety
/// `out` must be `cap` writable `f32`, or null when `cap` is 0.
unsafe fn emit_pcm(
    pcm: &[f32],
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
    what: &str,
) -> MfskStatus {
    if !out_len.is_null() {
        unsafe { *out_len = pcm.len() };
    }
    if out.is_null() || cap < pcm.len() {
        set_error(format!(
            "{what}: buffer too small; *out_len is the sample count needed"
        ));
        return MfskStatus::InvalidArg;
    }
    unsafe { ptr::copy_nonoverlapping(pcm.as_ptr(), out, pcm.len()) };
    MfskStatus::Ok
}

/// Library version, major.minor.patch packed into a 32-bit integer (8
/// bits per field). Useful for the consumer to sanity-check ABI
/// compatibility.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_version() -> u32 {
    let v: &str = env!("CARGO_PKG_VERSION");
    let mut parts = v.split('.').map(|s| s.parse::<u32>().unwrap_or(0));
    let major = parts.next().unwrap_or(0);
    let minor = parts.next().unwrap_or(0);
    let patch = parts.next().unwrap_or(0);
    (major << 16) | (minor << 8) | patch
}

// Keep cbindgen-visible types discoverable.
const _: fn() -> (c_int, *mut c_void) = || (0, ptr::null_mut());
