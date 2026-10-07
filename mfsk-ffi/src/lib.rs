//! C ABI for `mfsk-core`, the WSJT-family decoder suite.
//!
//! # Overview
//!
//! Exposes FT8 / FT4 / FST4 / WSPR / JT9 / JT65 / Q65 / MSK144 / JTTY
//! (and the experimental uvpacket profiles) decoders and synthesisers
//! behind a small opaque-handle C API that C++, Kotlin (Android JNI via a
//! thin shim) and Swift consumers can link against — 26 `MfskMode`s in
//! all. cbindgen generates `include/mfsk.h` on every build; see
//! `examples/cpp_smoke` for a round-trip demo that exercises every
//! protocol through the ABI.
//!
//! # The decoder handle
//!
//! One persistent decoder per mode, as WSJT-X runs one `jt9` per mode:
//! [`mfsk_decoder_open`] takes the upstream parameter block
//! (`MfskParams`, `jt9com`'s `nfa/nfb/nfqso/ndepth/mycall/hiscall/…`) and
//! the library's own options (`MfskExtras`), both size-versioned and
//! initialised by `mfsk_params_init` / `mfsk_extras_init`. What upstream
//! keeps between periods — the callsign hash table, FT8's a7 rows, Q65's
//! averages — lives in the handle. [`mfsk_decoder_decode_i16`] and
//! [`mfsk_decoder_decode_f32`] decode one period and stream each row to
//! the [`mfsk_decoder_set_on_decode`] callback as it is found; an option
//! the mode does not have is `MFSK_UNSUPPORTED`, never silently ignored.
//! Q65 has ten sub-modes; the Q65 history/callers handles carry the
//! caller's `q65_hist` / `q65_hist2` state.
//!
//! JTTY is not slotted, so it has no decoder: a receiver handle of its
//! own, [`mfsk_jtty_open`] … `mfsk_jtty_poll`, takes audio in chunks and
//! hands out message updates, and `mfsk_jtty_encode_tones` and the two
//! `mfsk_jtty_tones_to_*` functions transmit.
//!
//! Status codes, the decode-depth/strictness/equalisation enums and
//! the mode/row/params types live in `mfsk-ffi-abi` (issue #205).
//!
//! # Memory ownership
//!
//! **Decode results never cross the boundary as an allocation.** The
//! decode calls write into an array the caller owns and report how many
//! rows they needed, so there is nothing to free and no way to leak one
//! by unwinding past a free — the thing that makes Kotlin and Swift
//! wrappers fiddly.
//!
//! - [`mfsk_decoder_open`] / [`mfsk_decoder_close`]: the decoder, which
//!   owns the callsign hash table, the previous period's rows and its
//!   own error slot.
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
//! # Thread safety
//!
//! **One decoder per thread.** It caches state it mutates on every
//! decode (the callsign table, a7, averages).
//!
//! Concurrent decodes on *separate* decoders are supported and
//! exercised by the C++ driver in `examples/cpp_smoke` on every build.
//! They allocate their own FFT planners and scratch buffers.
//!
//! Errors live on the decoder ([`mfsk_decoder_last_error`]) rather than
//! in `thread_local!` storage, because a Kotlin coroutine or a Swift
//! `async` caller legitimately hops threads between checking a status
//! and reading the message, and would otherwise find NULL.
//! [`mfsk_last_error`] remains for the handle-less calls.

use mfsk_core::engine::tx::message_to_tones;
use std::ffi::{CStr, CString, c_char, c_int};
use std::os::raw::c_void;
use std::ptr;
use std::slice;

mod decoder;

pub use decoder::*;
pub use mfsk_ffi_abi::{
    MfskDecode, MfskDecoder, MfskExtras, MfskIqDecode, MfskIqReceiver, MfskJttyParams,
    MfskJttyReceiver, MfskJttyUpdate, MfskMode, MfskModeInfo, MfskParams, MfskQ65Caller,
    MfskQ65Callers, MfskQ65Dx, MfskQ65History, MfskStatus,
};
/// `mfsk_iq_open`'s `format`: `f32` I, `f32` Q, little-endian. A literal here
/// for the cbindgen reason [`MFSK_AP_FIELD_LEN`] gives.
pub const MFSK_IQ_FORMAT_CF32: u32 = 0;
/// `i16` I, `i16` Q, full scale 32768.
pub const MFSK_IQ_FORMAT_CS16: u32 = 1;
/// `i8` I, `i8` Q, full scale 128 (HackRF).
pub const MFSK_IQ_FORMAT_CS8: u32 = 2;
/// `u8` I, `u8` Q, 128 = zero, full scale 128 (RTL-SDR).
pub const MFSK_IQ_FORMAT_CU8: u32 = 3;
/// 24-bit signed I, Q, full scale 8 388 608.
pub const MFSK_IQ_FORMAT_CS24: u32 = 4;

/// `mfsk_iq_open_with`'s `channelizer`: one filter chain per channel from the
/// input rate. The default, and the cheapest for up to about four channels.
pub const MFSK_IQ_CHANNELIZER_DIRECT: u32 = 0;
/// A polyphase filter bank shared by every channel: a fixed cost of about
/// three direct channels, then about a quarter of one per channel.
pub const MFSK_IQ_CHANNELIZER_PFB: u32 = 1;

/// Inline capacity of each a-priori callsign/grid field of `MfskExtras`.
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
/// Bytes of `MfskDecode::key`. A literal here for the same cbindgen reason as the
/// constants below.
pub const MFSK_DECODE_KEY_LEN: usize = 10;

/// `MfskDecode::flags` bit 0: the text needed the decoder's callsign
/// hash table to resolve a `<...>` reference.
///
/// A literal here for the same cbindgen reason as the capability bits —
/// left in the dependency it reaches C as an undeclared identifier, so
/// a consumer reading `flags` cannot name the bit. Found by the Kotlin
/// JNI shim failing to compile, which is the first thing in this repo
/// to read that field from C.
pub const MFSK_DECODE_FLAG_HASH_RESOLVED: u8 = 1 << 0;
/// `MfskDecode::flags` bit 1: the sender set WSJT-X 3.2's Q65 Pileup
/// "copied last Tx" flag. Q65 rows only. A literal here for the same
/// reason as the constant above.
pub const MFSK_DECODE_FLAG_COPIED_LAST_TX: u8 = 1 << 1;
/// `MfskDecode::flags` bit 2: `sync_score` is a value the mode reported, not
/// the `0.0` of a mode that reports none (WSPR, JT9, JT65, Q65, FT8's a7/a8). A
/// literal here for the same reason as the constants above.
pub const MFSK_DECODE_FLAG_HAS_SYNC_SCORE: u8 = 1 << 2;
/// `MfskDecode::flags` bit 3: `sync_cv` is a value the mode reported.
pub const MFSK_DECODE_FLAG_HAS_SYNC_CV: u8 = 1 << 3;
/// `MfskDecode::flags` bit 4: `hard_errors` is a count the mode reported (a
/// clean decode is `0` with the flag set; WSPR, JT9, JT65 and Q65 never set it).
pub const MFSK_DECODE_FLAG_HAS_HARD_ERRORS: u8 = 1 << 4;

/// `MfskDecode::stage`: not from a `mfsk_decoder_decode_prefix_*` sequence.
/// A literal here for the same cbindgen reason as the constants above.
pub const MFSK_STAGE_NONE: u8 = 0;
/// `MfskDecode::stage`: found by a prefix call made before the period ended
/// (FT8's checkpoint A, ~11.8 s in), in time to answer in the next period.
pub const MFSK_STAGE_EARLY: u8 = 1;
/// `MfskDecode::stage`: found by the prefix call whose audio was the whole
/// period.
pub const MFSK_STAGE_FINAL: u8 = 2;

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

/// The mode is the 77-bit-message slot family (FT8, FT4, FST4): the
/// QSO-context AP of the parameter block, a7 and the sniper window apply.
/// Modes without this bit (WSPR, JT9, JT65, Q65) decode through the same
/// `mfsk_decoder_*` handle, shaped differently — their `MfskExtras` fields
/// differ and an option they lack is `MFSK_UNSUPPORTED`.
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
/// The mode is received by a stateful, continuously fed receiver handle
/// with its own entry points rather than by a slot decode — JTTY, whose
/// frames have no slot. Audio goes in with `mfsk_jtty_push_*` and message
/// updates come out of `mfsk_jtty_poll`.
pub const MFSK_CAP_STREAM_RECEIVER: u64 = 1 << 15;
/// WSJT-X's impulse-noise blanker (`nb_percent`, `nb_sweep_step`).
/// Every FST4 sub-mode and no other mode.
pub const MFSK_CAP_NOISE_BLANKER: u64 = 1 << 16;
/// The operator's transmit frequency (`tx_freq_hz`, WSJT-X's `nftx`)
/// steers the a-priori search. FT8 only.
pub const MFSK_CAP_TX_FREQ: u64 = 1 << 17;

// ──────────────────────────────────────────────────────────────────────────
// Public C types
// ──────────────────────────────────────────────────────────────────────────

/// Q65 sub-mode selector (the `submode` argument of `mfsk_encode_q65*`).
/// All sub-modes share the same FEC, sync layout and
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

/// Channel-spread fading model used through `MfskExtras::fading_model`.
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
#[derive(Clone, Copy)]
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
    let tones = message_to_tones::<mfsk_core::ft8::Ft8>(&msg77);
    let pcm =
        mfsk_core::engine::tx::synthesize::<mfsk_core::ft8::Ft8>(&tones, 12_000, freq_hz, 1.0);
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
    let tones = message_to_tones::<mfsk_core::ft4::Ft4>(&msg77);
    let pcm =
        mfsk_core::engine::tx::synthesize::<mfsk_core::ft4::Ft4>(&tones, 12_000, freq_hz, 1.0);
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
    let tones = message_to_tones::<mfsk_core::fst4::Fst4s60>(&msg77);
    let pcm =
        mfsk_core::engine::tx::synthesize::<mfsk_core::fst4::Fst4s60>(&tones, 12_000, freq_hz, 1.0);
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
    unsafe {
        encode_q65_inner(
            "mfsk_encode_q65",
            submode,
            call1,
            call2,
            grid_or_report,
            false,
            freq_hz,
            out,
            cap,
            out_len,
        )
    }
}

/// [`mfsk_encode_q65`] with WSJT-X 3.2's **Q65 Pileup** "copied last Tx"
/// flag: a non-zero `copied_last_tx` sets the spare 78th payload bit
/// (`genq65.f90`'s `iflag`), which a Pileup receiver reports as
/// `MFSK_DECODE_FLAG_COPIED_LAST_TX`. `copied_last_tx` is an integer, not a
/// `bool`, so any value a C caller writes is a defined one; 0 is exactly
/// [`mfsk_encode_q65`].
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_q65_flagged(
    submode: u32,
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    copied_last_tx: u32,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    unsafe {
        encode_q65_inner(
            "mfsk_encode_q65_flagged",
            submode,
            call1,
            call2,
            grid_or_report,
            copied_last_tx != 0,
            freq_hz,
            out,
            cap,
            out_len,
        )
    }
}

/// The body both Q65 encode entry points share.
///
/// # Safety
/// As [`mfsk_encode_q65`].
#[allow(clippy::too_many_arguments)]
unsafe fn encode_q65_inner(
    who: &str,
    submode: u32,
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    copied_last_tx: bool,
    freq_hz: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(submode) = q65_submode_of(submode) else {
        set_error(format!("{who}: not a Q65 sub-mode"));
        return MfskStatus::InvalidArg;
    };
    use mfsk_core::q65::{
        Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60, Q65e120,
        tx::synthesize_standard_flagged_for as synth,
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
    let f = copied_last_tx;
    let pcm_opt = match submode {
        MfskQ65SubMode::A15 => synth::<Q65a15>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::A30 => synth::<Q65a30>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::A60 => synth::<Q65a60>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::B60 => synth::<Q65b60>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::C60 => synth::<Q65c60>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::D60 => synth::<Q65d60>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::E60 => synth::<Q65e60>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::D120 => synth::<Q65d120>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::E120 => synth::<Q65e120>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
        MfskQ65SubMode::A300 => synth::<Q65a300>(c1, c2, gr, f, 12_000, freq_hz, 0.3),
    };
    let Some(pcm) = pcm_opt else {
        set_error("Q65 synth failed (bad pack)");
        return MfskStatus::InvalidArg;
    };
    unsafe { emit_pcm(&pcm, out, cap, out_len, "encode") }
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
    (MfskMode::Jtty, "JTTY\0", false),
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

/// JTTY's frame geometry, which no registry entry carries (it is outside
/// `Protocol`, like MSK144). `t_slot_s` is the **frame period** — the mode has
/// no slot — and `n_symbols` counts one frame: 13 sync + 46 data.
struct JttyGeometry;
impl JttyGeometry {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 384; // at 12 kHz: 31.25 baud
    const SYMBOL_DT: f32 = 0.032;
    const TONE_SPACING_HZ: f32 = 31.25;
    const GFSK_BT: f32 = 2.0;
    const GFSK_HMOD: f32 = 1.0;
    const N_DATA: u32 = 46;
    const N_SYNC: u32 = 13;
    const N_SYMBOLS: u32 = 59;
    const T_FRAME_S: f32 = 1.888;
    /// 34 payload bits + CRC-12 through the rate-1/2 tail-biting code: 92 coded bits.
    const FEC_K: u32 = 46;
    const FEC_N: u32 = 92;
    const PAYLOAD_BITS: u32 = 34;
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
    if mode == MfskMode::Jtty {
        return cfg!(feature = "jtty");
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
        None if mode == MfskMode::Jtty => {
            // JTTY: no registry entry, no slot; one frame is described.
            info.ntones = JttyGeometry::NTONES;
            info.bits_per_symbol = JttyGeometry::BITS_PER_SYMBOL;
            info.nsps = JttyGeometry::NSPS;
            info.symbol_dt = JttyGeometry::SYMBOL_DT;
            info.tone_spacing_hz = JttyGeometry::TONE_SPACING_HZ;
            info.gfsk_bt = JttyGeometry::GFSK_BT;
            info.gfsk_hmod = JttyGeometry::GFSK_HMOD;
            info.n_data = JttyGeometry::N_DATA;
            info.n_sync = JttyGeometry::N_SYNC;
            info.n_symbols = JttyGeometry::N_SYMBOLS;
            info.t_slot_s = JttyGeometry::T_FRAME_S;
            // `NSPS * N_SYMBOLS`, exact, for the reason MSK144's row gives.
            info.slot_samples_12k = JttyGeometry::NSPS * JttyGeometry::N_SYMBOLS;
            info.fec_k = JttyGeometry::FEC_K;
            info.fec_n = JttyGeometry::FEC_N;
            info.payload_bits = JttyGeometry::PAYLOAD_BITS;
            info.caps = MFSK_CAP_STREAM_RECEIVER | MFSK_CAP_ENCODE;
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
            // `NSPS * N_SYMBOLS`, not `T_FRAME_S * 12_000`: the latter
            // is 863 rather than 864, because 0.072 is not exact in
            // binary32 and `as u32` truncates. Every other mode copies a
            // precomputed integer from the registry, so MSK144 was the
            // only row that could be off by a sample — and it was. Found
            // by the Swift binding's geometry test, which checks
            // `slot_samples_12k == t_slot_s * 12 kHz` for every mode.
            info.slot_samples_12k = Msk144Geometry::NSPS * Msk144Geometry::N_SYMBOLS;
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
        None if mode == MfskMode::Jtty && mode_is_present(mode) => {
            MFSK_CAP_STREAM_RECEIVER | MFSK_CAP_ENCODE
        }
        None => 0,
    }
}

/// ABI revision, distinct from [`mfsk_version`].
///
/// `mfsk_version` tracks the crate's release number and moves for
/// reasons that have nothing to do with the boundary. This moves only
/// when the C surface changes shape, so it is the one to check before
/// deciding a header and a library agree.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_abi_version() -> u32 {
    3
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

/// As [`mode_of`], for the Q65 sub-mode tag. Same reason: a C caller can put
/// any integer in an `enum` parameter, and matching an out-of-range one as a
/// Rust enum is undefined behaviour.
fn q65_submode_of(raw: u32) -> Option<MfskQ65SubMode> {
    use MfskQ65SubMode::*;
    [A15, A30, A60, B60, C60, D60, E60, D120, E120, A300]
        .into_iter()
        .find(|m| *m as u32 == raw)
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

fn copy_name(dst: &mut [c_char; 16], src: &str) {
    let b = src.as_bytes();
    let n = b.len().min(dst.len() - 1);
    for (d, &s) in dst.iter_mut().zip(&b[..n]) {
        *d = s as c_char;
    }
    dst[n] = 0;
}

// ──────────────────────────────────────────────────────────────────────────
// Q65 state the caller keeps: `q65_hist` and `q65_hist2`
//
// Not decoder state (a decoder owns what its own periods produced); these
// are the operator's lists, built from what any source heard, which the
// contest list of `mfsk_decoder_set_q65_callers` and the DX-call lookup
// read.
// ──────────────────────────────────────────────────────────────────────────

fn q65_history<'a>(h: *mut MfskQ65History) -> Option<&'a mut mfsk_core::q65::Q65History> {
    unsafe { (h as *mut mfsk_core::q65::Q65History).as_mut() }
}

/// A new, empty history. Free with [`mfsk_q65_history_free`]. **Not
/// thread-safe**: one per thread, or guard it yourself.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_q65_history_new() -> *mut MfskQ65History {
    Box::into_raw(Box::new(mfsk_core::q65::Q65History::new())) as *mut MfskQ65History
}

/// Free a history. NULL is a no-op.
///
/// # Safety
/// `h` must be from [`mfsk_q65_history_new`], freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_history_free(h: *mut MfskQ65History) {
    if !h.is_null() {
        drop(unsafe { Box::from_raw(h as *mut mfsk_core::q65::Q65History) });
    }
}

/// Remember one decode at `freq_hz` (tone 0). The 100 most recent are kept.
///
/// # Safety
/// `h` must be live; `message` a NUL-terminated UTF-8 string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_history_push(
    h: *mut MfskQ65History,
    freq_hz: f32,
    message: *const c_char,
) -> MfskStatus {
    let Some(h) = q65_history(h) else {
        set_error("mfsk_q65_history_push: null handle");
        return MfskStatus::NullPointer;
    };
    let msg = match cstr_to_str(message) {
        Ok(m) => m,
        Err(st) => return st,
    };
    h.push(freq_hz, msg);
    MfskStatus::Ok
}

/// Remember every row of a decode, as `q65_decode.f90` calls `q65_hist` after
/// each one. `rows` is an array of `n` [`MfskDecode`] as this library wrote
/// them (their own stride), e.g. straight from `mfsk_decoder_decode_f32`.
///
/// # Safety
/// `h` must be live; `rows` must point to `n` valid rows.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_history_record(
    h: *mut MfskQ65History,
    rows: *const MfskDecode,
    n: usize,
) -> MfskStatus {
    let Some(h) = q65_history(h) else {
        set_error("mfsk_q65_history_record: null handle");
        return MfskStatus::NullPointer;
    };
    if rows.is_null() && n != 0 {
        set_error("mfsk_q65_history_record: rows is NULL");
        return MfskStatus::NullPointer;
    }
    for r in (0..n).map(|i| unsafe { &*rows.add(i) }) {
        h.push(r.freq_hz, cstr_field(&r.text));
    }
    MfskStatus::Ok
}

/// How many decodes the history holds (at most 100).
///
/// # Safety
/// `h` must be live or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_history_len(h: *const MfskQ65History) -> usize {
    unsafe { (h as *const mfsk_core::q65::Q65History).as_ref() }.map_or(0, |h| h.len())
}

/// The DX station from the most recent decode within 10 Hz of `rx_freq_hz`
/// whose first word is 3 to 12 characters — WSJT-X's "Decode Again" with no
/// DX call entered, so a `CQ ...` decode is passed over for an older one.
/// Returns `MFSK_STATUS_OK` and fills `out`, or `MFSK_STATUS_DECODE_FAILED`
/// when nothing qualifies (`out` is left untouched).
///
/// # Safety
/// `h` must be live; `out` must point to `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_history_lookup(
    h: *const MfskQ65History,
    rx_freq_hz: f32,
    out: *mut MfskQ65Dx,
) -> MfskStatus {
    let Some(h) = (unsafe { (h as *const mfsk_core::q65::Q65History).as_ref() }) else {
        set_error("mfsk_q65_history_lookup: null handle");
        return MfskStatus::NullPointer;
    };
    if out.is_null() {
        set_error("mfsk_q65_history_lookup: out is NULL");
        return MfskStatus::NullPointer;
    }
    let Some(dx) = h.lookup(rx_freq_hz) else {
        return MfskStatus::DecodeFailed;
    };
    let mut v = MfskQ65Dx {
        size: core::mem::size_of::<MfskQ65Dx>() as u32,
        has_grid: u32::from(dx.grid.is_some()),
        call: [0; 16],
        grid: [0; 8],
    };
    write_field(&mut v.call, &dx.call);
    if let Some(g) = &dx.grid {
        write_field(&mut v.grid, g);
    }
    unsafe { write_size_versioned(out, &v) };
    MfskStatus::Ok
}

// ── Q65Callers: the contest caller list (`q65_hist2`) ────────────────────

fn q65_callers<'a>(h: *mut MfskQ65Callers) -> Option<&'a mut mfsk_core::q65::Q65Callers> {
    unsafe { (h as *mut mfsk_core::q65::Q65Callers).as_mut() }
}

/// A new, empty caller list. Free with [`mfsk_q65_callers_free`]. **Not
/// thread-safe.**
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_q65_callers_new() -> *mut MfskQ65Callers {
    Box::into_raw(Box::new(mfsk_core::q65::Q65Callers::new())) as *mut MfskQ65Callers
}

/// Free a caller list. NULL is a no-op.
///
/// # Safety
/// `h` must be from [`mfsk_q65_callers_new`], freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_callers_free(h: *mut MfskQ65Callers) {
    if !h.is_null() {
        drop(unsafe { Box::from_raw(h as *mut mfsk_core::q65::Q65Callers) });
    }
}

/// Remember a decode at `freq_hz` heard at `now` (Unix seconds — the library
/// reads no clock): a compound call is ignored, ` R ` is taken out, the second
/// word is the caller and the next four characters its grid. A known caller
/// is refreshed; a new one is added only if it sent a grid, the oldest making
/// room once 50 are held.
///
/// # Safety
/// `h` must be live; `message` a NUL-terminated UTF-8 string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_callers_record(
    h: *mut MfskQ65Callers,
    freq_hz: f32,
    message: *const c_char,
    now: u64,
) -> MfskStatus {
    let Some(h) = q65_callers(h) else {
        set_error("mfsk_q65_callers_record: null handle");
        return MfskStatus::NullPointer;
    };
    let msg = match cstr_to_str(message) {
        Ok(m) => m,
        Err(st) => return st,
    };
    h.record(freq_hz, msg, now);
    MfskStatus::Ok
}

/// Drop callers not heard for more than 24 hours. Call before each decode.
///
/// # Safety
/// `h` must be live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_callers_expire(h: *mut MfskQ65Callers, now: u64) -> MfskStatus {
    let Some(h) = q65_callers(h) else {
        set_error("mfsk_q65_callers_expire: null handle");
        return MfskStatus::NullPointer;
    };
    h.expire(now);
    MfskStatus::Ok
}

/// Forget one caller (worked, say) — `rm_q3list`. A call that is not listed is
/// not an error.
///
/// # Safety
/// `h` must be live; `call` a NUL-terminated UTF-8 string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_callers_remove(
    h: *mut MfskQ65Callers,
    call: *const c_char,
) -> MfskStatus {
    let Some(h) = q65_callers(h) else {
        set_error("mfsk_q65_callers_remove: null handle");
        return MfskStatus::NullPointer;
    };
    let c = match cstr_to_str(call) {
        Ok(c) => c,
        Err(st) => return st,
    };
    h.remove(c);
    MfskStatus::Ok
}

/// How many callers are listed (at most 50).
///
/// # Safety
/// `h` must be live or NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_callers_len(h: *const MfskQ65Callers) -> usize {
    unsafe { (h as *const mfsk_core::q65::Q65Callers).as_ref() }.map_or(0, |h| h.callers().len())
}

/// The `index`th caller, oldest first. `MFSK_STATUS_INVALID_ARG` past the end.
///
/// # Safety
/// `h` must be live; `out` must point to `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_q65_callers_get(
    h: *const MfskQ65Callers,
    index: usize,
    out: *mut MfskQ65Caller,
) -> MfskStatus {
    let Some(h) = (unsafe { (h as *const mfsk_core::q65::Q65Callers).as_ref() }) else {
        set_error("mfsk_q65_callers_get: null handle");
        return MfskStatus::NullPointer;
    };
    if out.is_null() {
        set_error("mfsk_q65_callers_get: out is NULL");
        return MfskStatus::NullPointer;
    }
    let Some(c) = h.callers().get(index) else {
        set_error("mfsk_q65_callers_get: index out of range");
        return MfskStatus::InvalidArg;
    };
    let mut v = MfskQ65Caller {
        size: core::mem::size_of::<MfskQ65Caller>() as u32,
        freq_hz: c.freq_hz,
        last_heard: c.last_heard,
        call: [0; 8],
        grid: [0; 8],
    };
    write_field(&mut v.call, &c.call);
    write_field(&mut v.grid, &c.grid);
    unsafe { write_size_versioned(out, &v) };
    MfskStatus::Ok
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

/// Evaluate `$body` with `$P` bound to the transmit type `$mode` names —
/// the FFI's one bridge from a runtime `MfskMode` to the compile-time
/// `P` that `engine::tx::synthesize` is generic over. Modes with a tone
/// stage (FT8, FT4, every FST4 sub-mode) only; anything else is `$none`.
macro_rules! with_tone_mode {
    ($mode:expr, $P:ident => $body:expr, else $none:expr) => {{
        use mfsk_core::fst4::{Fst4s15, Fst4s30, Fst4s60, Fst4s120, Fst4s300};
        match $mode {
            MfskMode::Ft8 => {
                type $P = mfsk_core::ft8::Ft8;
                $body
            }
            MfskMode::Ft4 => {
                type $P = mfsk_core::ft4::Ft4;
                $body
            }
            MfskMode::Fst4s15 => {
                type $P = Fst4s15;
                $body
            }
            MfskMode::Fst4s30 => {
                type $P = Fst4s30;
                $body
            }
            MfskMode::Fst4s60 => {
                type $P = Fst4s60;
                $body
            }
            MfskMode::Fst4s120 => {
                type $P = Fst4s120;
                $body
            }
            MfskMode::Fst4s300 => {
                type $P = Fst4s300;
                $body
            }
            _ => $none,
        }
    }};
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
    with_tone_mode!(mode, P => mfsk_core::engine::tx::synth_len::<P>(12_000), else 0)
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
/// The hashed half decodes as `<...>` unless the receiving decoder has
/// seen that callsign — see [`mfsk_decoder_add_callsign`].
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

/// Render a packed 77-bit message as text, `<...>` callsigns left
/// unresolved. Writes at most `cap` bytes including the NUL, and reports the
/// size needed if that is not enough. To resolve hashed callsigns from a
/// decoder's table use [`mfsk_decoder_unpack77`].
///
/// # Safety
/// `message77` must be 77 readable bytes; `out` must be `cap` writable
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_unpack77(
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
    unsafe {
        put_text(
            mfsk_core::msg::wsjt77::unpack77(bits),
            "mfsk_unpack77",
            out,
            cap,
            out_len,
        )
    }
}

/// Write `text` as a NUL-terminated string into `out`.
///
/// # Safety
/// `out` must be `cap` writable bytes; `out_len` may be null.
pub(crate) unsafe fn put_text(
    text: Option<String>,
    who: &str,
    out: *mut c_char,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(text) = text else {
        set_error(format!("{who}: not a decodable 77-bit message"));
        return MfskStatus::DecodeFailed;
    };
    let needed = text.len() + 1;
    if !out_len.is_null() {
        unsafe { *out_len = needed };
    }
    if out.is_null() || cap < needed {
        set_error(format!(
            "{who}: buffer too small; *out_len is the size needed"
        ));
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
        MfskMode::Ft8 => message_to_tones::<mfsk_core::ft8::Ft8>(&msg),
        MfskMode::Ft4 => message_to_tones::<mfsk_core::ft4::Ft4>(&msg),
        // Every FST4 sub-mode shares one 160-symbol layout.
        _ => message_to_tones::<mfsk_core::fst4::Fst4s60>(&msg),
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
    with_tone_mode!(
        mode_of(mode).expect("checked"),
        P => mfsk_core::engine::tx::synthesize_i16_into::<P>(dst, tones, 12_000, freq_hz, amplitude),
        else unreachable!("synth_check admits tone-stage modes only")
    );
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
    with_tone_mode!(
        mode_of(mode).expect("checked"),
        P => mfsk_core::engine::tx::synthesize_into::<P>(dst, tones, 12_000, freq_hz, amplitude),
        else unreachable!("synth_check admits tone-stage modes only")
    );
    MfskStatus::Ok
}

// ──────────────────────────────────────────────────────────────────────────
// Streaming ingestion and the slot grid (FFI v2 slice 5)
//
// Audio in, slots out, cut on the mode's UTC grid by
// `mfsk_core::slotgrid::SlotCutter` (the same cutter the IQ receiver's
// channels use), so FST4-300's 3.6 M-sample slot works the way FT4's
// 90 000-sample one does.
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

/// A slot cut on the grid and waiting to be taken or decoded.
struct ReadySlot {
    period: i64,
    utc_ns: Option<i64>,
    audio: Vec<i16>,
    /// The whole slot, not a prefix of it.
    whole: bool,
}

struct StreamInner {
    mode: MfskMode,
    /// `None` when the source is already 12 kHz.
    resampler: Option<mfsk_core::engine::dsp::resample::LinearResamplerI16To12k>,
    /// Cuts the 12 kHz samples into the mode's slots, each on its own UTC
    /// boundary, following the clock as it slews.
    cutter: mfsk_core::slotgrid::SlotCutter<i16>,
    clock: mfsk_core::slotgrid::SampleClock,
    /// The newest completed slot. A newer one replaces it: a live receiver
    /// wants the latest, and `dropped` counts what it replaced.
    ready: Option<ReadySlot>,
    dropped: u64,
    /// The period the stream delivered a prefix of and not yet its whole.
    open: Option<i64>,
    /// The last period delivered, prefix or whole.
    last: Option<i64>,
    /// The period whose prefixes a decoder took through
    /// `mfsk_decoder_decode_stream`, so its whole slot ends that sequence.
    pub(crate) decoding: Option<i64>,
}

fn stream_inner(s: *mut MfskStream) -> Option<&'static mut StreamInner> {
    unsafe { (s as *mut StreamInner).as_mut() }
}

fn stream_ref(s: *const MfskStream) -> Option<&'static StreamInner> {
    unsafe { (s as *const StreamInner).as_ref() }
}

/// Open a capture stream for `mode`, accepting audio at `sample_rate`.
///
/// Slots are cut on the mode's UTC grid from the sample count, the way the
/// IQ receiver cuts them: with no clock set (`mfsk_stream_set_time`) the grid
/// free-runs from the first sample, right for replaying a recording; with
/// one, a slot starts on its own boundary and a drifting clock moves the
/// boundary by milliseconds, losing no slot. At most one completed slot
/// waits; a newer one replaces it.
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
    if iq_mode_of(m).is_none() {
        set_error("mfsk_stream_open: this mode is not cut into slots");
        report(MfskStatus::Unsupported);
        return ptr::null_mut();
    }
    if sample_rate == 0 {
        set_error("mfsk_stream_open: sample_rate is 0");
        report(MfskStatus::InvalidArg);
        return ptr::null_mut();
    }
    let period_ns = (meta.t_slot_s * 10.0).round() as i64 * 100_000_000;
    report(MfskStatus::Ok);
    Box::into_raw(Box::new(StreamInner {
        mode: m,
        resampler: (sample_rate != 12_000)
            .then(|| mfsk_core::engine::dsp::resample::LinearResamplerI16To12k::new(sample_rate)),
        cutter: mfsk_core::slotgrid::SlotCutter::new(
            mfsk_core::slotgrid::SlotGrid::new(period_ns, 12_000),
            0,
        ),
        clock: mfsk_core::slotgrid::SampleClock::new(12_000),
        ready: None,
        dropped: 0,
        open: None,
        last: None,
        decoding: None,
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

fn stream_feed(st: &mut StreamInner, src: &[i16]) {
    let anchor = st.clock.anchor_ns();
    // Prefixes and wholes in the order they were cut.
    let mut done: Vec<(i64, Vec<i16>, bool)> = Vec::new();
    {
        let done = core::cell::RefCell::new(&mut done);
        st.cutter.feed_parts(
            anchor,
            src,
            |j, _start, buf| done.borrow_mut().push((j, buf.to_vec(), false)),
            |j, _start, buf| done.borrow_mut().push((j, buf, true)),
        );
    }
    let period_ns = (st.cutter_period_ns()) as i128;
    let prefixes = st.cutter.has_points();
    for (j, audio, whole) in done {
        // A period already partly delivered, opened again (a clock stepped
        // back): its audio is another recording, and a decoder continuing
        // that period's prefix sequence would splice the two
        // (`IQ_PREFIX_DESIGN.md` §5). Only a stream with points refuses it.
        if prefixes && st.open != Some(j) && st.last.is_some_and(|l| j <= l) {
            continue;
        }
        st.open = (!whole).then_some(j);
        st.last = Some(j);
        let replaced = st.ready.replace(ReadySlot {
            period: j,
            utc_ns: anchor.map(|_| (j as i128 * period_ns) as i64),
            audio,
            whole,
        });
        // A later delivery of the same period supersedes its prefix: only a
        // slot of another period is lost.
        if replaced.is_some_and(|r| r.period != j) {
            st.dropped += 1;
        }
    }
}

impl StreamInner {
    fn cutter_period_ns(&self) -> i64 {
        mode_meta(self.mode)
            .map(|m| (m.t_slot_s * 10.0).round() as i64 * 100_000_000)
            .unwrap_or(1)
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
        None => stream_feed(st, src),
        Some(_) => {
            // Resample in bounded chunks so a long push does not allocate a
            // second copy of the whole buffer.
            let mut scratch = [0i16; 512];
            let mut pos = 0;
            while pos < src.len() {
                let r = st.resampler.as_mut().expect("checked");
                let (consumed, produced) = r.process(&src[pos..], &mut scratch);
                if consumed == 0 && produced == 0 {
                    break;
                }
                let chunk: Vec<i16> = scratch[..produced].to_vec();
                stream_feed(st, &chunk);
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

/// How many 12 kHz samples the stream has taken in: its clock, to pass as
/// `at_sample` to [`mfsk_stream_set_time`].
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_stream_position(s: *const MfskStream) -> u64 {
    stream_ref(s).map(|st| st.cutter.position()).unwrap_or(0)
}

/// What [`mfsk_stream_set_time`] did, written to `*out_change`.
pub const MFSK_CLOCK_FIRST: i32 = 0;
/// The clock moved towards the reading by at most its slew limit; open slots
/// are unaffected.
pub const MFSK_CLOCK_SLEWED: i32 = 1;
/// The reading was more than a second away: the clock re-anchored and the
/// slot that straddled the jump is dropped.
pub const MFSK_CLOCK_STEPPED: i32 = 2;

/// The stream's sample `at_sample` (12 kHz, as [`mfsk_stream_position`]
/// counts) was at UTC `utc_ns` (ns since the Unix epoch). Call it as often as
/// you have a reading: the stream follows the readings at up to 400 ppm, so
/// noisy readings and a drifting clock move slot boundaries by milliseconds
/// and lose nothing. `*out_change` receives `MFSK_CLOCK_*`.
///
/// # Safety
/// `out_change` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_set_time(
    s: *mut MfskStream,
    utc_ns: i64,
    at_sample: u64,
    out_change: *mut i32,
) -> MfskStatus {
    let Some(st) = stream_inner(s) else {
        set_error("mfsk_stream_set_time: null stream");
        return MfskStatus::NullPointer;
    };
    use mfsk_core::slotgrid::ClockChange;
    let change = st.clock.observe(utc_ns, at_sample);
    let code = match change {
        ClockChange::First => MFSK_CLOCK_FIRST,
        ClockChange::Slewed { .. } => MFSK_CLOCK_SLEWED,
        _ => MFSK_CLOCK_STEPPED,
    };
    if !matches!(change, ClockChange::Slewed { .. }) {
        // The grid jumped: audio spanning the jump is not a slot.
        st.cutter.forget_slots();
        st.open = None;
    }
    if !out_change.is_null() {
        unsafe { *out_change = code };
    }
    MfskStatus::Ok
}

/// Whether a slot is waiting: a completed one, or with prefix points
/// ([`mfsk_stream_set_prefix_points`]) the slot so far.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_stream_slot_ready(s: *const MfskStream) -> bool {
    stream_ref(s).map(|st| st.ready.is_some()).unwrap_or(false)
}

/// Whether the waiting slot is whole rather than a prefix of it; `false`
/// when none is waiting. A stream without prefix points only has whole
/// slots.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_stream_slot_is_whole(s: *const MfskStream) -> bool {
    stream_ref(s)
        .and_then(|st| st.ready.as_ref())
        .is_some_and(|r| r.whole)
}

/// Early decode on a stream (#601), off by default: from the next slot that
/// opens, the stream also makes the slot so far ready at each of these
/// 12 kHz sample counts, then the whole slot. Pass the decoder's
/// `mfsk_decoder_prefix_points`, and decode every slot the stream makes ready
/// with `mfsk_decoder_decode_stream` (or, after
/// `mfsk_stream_take_slot_i16`, with `mfsk_decoder_decode_prefix_i16`, the
/// whole slot included): checkpoint A's rows then arrive at ~11.8 s with
/// `stage == MFSK_STAGE_EARLY`. A newer delivery of the same period replaces
/// an untaken prefix without counting in [`mfsk_stream_dropped`]. A period
/// the stream already delivered part of is not delivered again (a clock
/// stepped back). `n == 0` turns it off. Off by default because a caller of
/// `mfsk_stream_take_slot_i16` would otherwise get short slots it did not
/// ask for.
///
/// # Safety
/// `points` must be `n` readable `size_t` (or null when `n` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_set_prefix_points(
    s: *mut MfskStream,
    points: *const usize,
    n: usize,
) -> MfskStatus {
    let Some(st) = stream_inner(s) else {
        set_error("mfsk_stream_set_prefix_points: null stream");
        return MfskStatus::NullPointer;
    };
    if points.is_null() && n != 0 {
        set_error("mfsk_stream_set_prefix_points: points is NULL");
        return MfskStatus::NullPointer;
    }
    let pts = if n == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(points, n) }
    };
    st.cutter.set_points(pts);
    MfskStatus::Ok
}

/// Completed slots a newer one replaced before they were taken.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_stream_dropped(s: *const MfskStream) -> u64 {
    stream_ref(s).map(|st| st.dropped).unwrap_or(0)
}

/// Take the waiting slot, copying it into `out`, with its index on the grid
/// and, when a clock is set, its UTC start.
///
/// Returns the number of samples written, or 0 if no slot is ready or `cap`
/// is too small — size from `MfskModeInfo::slot_samples_12k`. With prefix
/// points the slot may be a prefix: fewer samples, and
/// [`mfsk_stream_slot_is_whole`] false before the take.
///
/// # Safety
/// `out` must be `cap` writable `int16_t`; `out_period` and `out_utc_ns` may
/// be null (`*out_utc_ns` is 0 with no clock).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_take_slot_i16(
    s: *mut MfskStream,
    out: *mut i16,
    cap: usize,
    out_period: *mut i64,
    out_utc_ns: *mut i64,
) -> usize {
    let Some(st) = stream_inner(s) else {
        return 0;
    };
    let Some(slot) = st.ready.as_ref() else {
        return 0;
    };
    if out.is_null() || cap < slot.audio.len() {
        return 0;
    }
    let slot = st.ready.take().expect("checked");
    if !out_period.is_null() {
        unsafe { *out_period = slot.period };
    }
    if !out_utc_ns.is_null() {
        unsafe { *out_utc_ns = slot.utc_ns.unwrap_or(0) };
    }
    unsafe { ptr::copy_nonoverlapping(slot.audio.as_ptr(), out, slot.audio.len()) };
    slot.audio.len()
}

/// Drop the waiting slot and the one being cut, keeping the clock.
///
/// # Safety
/// `s` must be a live stream or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_stream_clear(s: *mut MfskStream) {
    if let Some(st) = stream_inner(s) {
        st.ready = None;
        st.cutter.forget_slots();
        st.open = None;
    }
}

// ──────────────────────────────────────────────────────────────────────────
// JTTY receiver (#477, P4b)
//
// JTTY has no slot, so none of the slot families above fit: audio arrives
// continuously, the receiver carries state (the search window, the messages
// under assembly, the audio a retro re-sweep still needs), and the output is
// message *updates*, not a list per call. So it gets a handle of its own.
//
// Decoding runs inside `mfsk_jtty_push_*`, on the calling thread (and on the
// pool `mfsk_runtime_configure` installed, or rayon's). The updates go into a
// queue in the handle, which `mfsk_jtty_poll` drains: the callback the Rust API
// offers (`jtty::rx::Stream::push`) cannot cross a C boundary without a
// user-data contract nobody wants for a chat window, and upstream's own
// `jtty_get_updates` is a poll for the same reason. The queue coalesces per
// message — a message that grew twice between polls is reported once, with its
// latest text — which is also upstream's rule and what bounds the queue.
//
// The handle is not thread-safe: one thread at a time, like `MfskStream`.
// ──────────────────────────────────────────────────────────────────────────

/// Most distinct messages the queue holds between polls. A caller that never
/// polls loses the oldest, rather than growing the queue for as long as the
/// receiver runs.
#[cfg(feature = "jtty")]
const JTTY_QUEUE_MAX: usize = 1024;

#[cfg(feature = "jtty")]
struct JttyInner {
    stream: mfsk_core::jtty::rx::Stream,
    sample_rate: u32,
    resampler: Option<mfsk_core::engine::dsp::resample::LinearResamplerI16To12k>,
    /// Pending updates, one per message id, oldest first.
    queue: std::collections::VecDeque<mfsk_core::jtty::assemble::MessageUpdate>,
}

#[cfg(feature = "jtty")]
impl JttyInner {
    fn queue_update(
        queue: &mut std::collections::VecDeque<mfsk_core::jtty::assemble::MessageUpdate>,
        u: mfsk_core::jtty::assemble::MessageUpdate,
    ) {
        if let Some(slot) = queue.iter_mut().find(|q| q.id == u.id) {
            *slot = u;
            return;
        }
        if queue.len() >= JTTY_QUEUE_MAX {
            queue.pop_front();
        }
        queue.push_back(u);
    }

    fn push_12k(&mut self, samples: &[i16]) {
        let Self { stream, queue, .. } = self;
        in_pool_mut(|| stream.push(samples, &mut |u| Self::queue_update(queue, u)));
    }
}

#[cfg(feature = "jtty")]
fn jtty_inner<'a>(rx: *mut MfskJttyReceiver) -> Option<&'a mut JttyInner> {
    unsafe { (rx as *mut JttyInner).as_mut() }
}

/// The defaults `rjtty` uses: 1500 Hz ± 50 Hz, `smin` 4.6 dB, band 200–2800 Hz,
/// subtraction on. Sets `size`.
///
/// # Safety
/// `out` must be null or point to a writable `MfskJttyParams`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_params_init(out: *mut MfskJttyParams) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_jtty_params_init: out is NULL");
        return MfskStatus::NullPointer;
    }
    unsafe { out.write(jtty_default_params()) };
    MfskStatus::Ok
}

fn jtty_default_params() -> MfskJttyParams {
    MfskJttyParams {
        size: core::mem::size_of::<MfskJttyParams>() as u32,
        subtract: 1,
        f0_hz: 1500.0,
        ftol_hz: 50.0,
        smin_db: 4.6,
        nfa_hz: 200.0,
        nfb_hz: 2800.0,
    }
}

/// Read a caller's (possibly older, shorter) `MfskJttyParams` over the defaults,
/// and check it. A NULL pointer is the defaults.
///
/// # Safety
/// `src` must be null or point to at least `src->size` readable bytes.
unsafe fn read_jtty_params(src: *const MfskJttyParams) -> Result<MfskJttyParams, String> {
    let mut p = jtty_default_params();
    if !src.is_null() {
        let full = core::mem::size_of::<MfskJttyParams>();
        let declared = unsafe { core::ptr::read_unaligned(src as *const u32) } as usize;
        let n = if declared == 0 || declared > full {
            full
        } else {
            declared
        };
        unsafe {
            core::ptr::copy_nonoverlapping(
                src as *const u8,
                &mut p as *mut MfskJttyParams as *mut u8,
                n,
            );
        }
        p.size = full as u32;
    }
    let ok = |x: f32| x.is_finite();
    if !(ok(p.f0_hz) && ok(p.ftol_hz) && ok(p.smin_db) && ok(p.nfa_hz) && ok(p.nfb_hz)) {
        return Err("MfskJttyParams: a frequency or threshold is not finite".into());
    }
    if p.ftol_hz < 0.0 || p.nfb_hz <= p.nfa_hz {
        return Err("MfskJttyParams: ftol_hz must be >= 0 and nfb_hz above nfa_hz".into());
    }
    Ok(p)
}

#[cfg(feature = "jtty")]
fn jtty_core_params(p: &MfskJttyParams) -> mfsk_core::jtty::rx::Params {
    mfsk_core::jtty::rx::Params {
        f0_hz: p.f0_hz,
        ftol_hz: p.ftol_hz,
        smin_db: p.smin_db,
        nfa_hz: p.nfa_hz,
        nfb_hz: p.nfb_hz,
        subtract: p.subtract != 0,
        sequential: false,
        carry: false,
        ch0_only: false,
        decimate_sync: false,
        raw_first: false,
        fir_analytic: false,
        ladder_budget: None,
        coarse_sync_grid: false,
        ladder_rungs: mfsk_core::jtty::ladder::Rungs::ALL,
        skip_decoded_hz: 0.0,
        side_channels: mfsk_core::jtty::rx::SideChannels::Upstream,
        retro_sweep: true,
        subtract_side_channels: true,
        side_ladder_budget: None,
    }
}

/// Open a JTTY receiver taking audio at `sample_rate` (any rate; anything but
/// 12 000 Hz is resampled, linearly). `params` may be NULL for the defaults.
///
/// Returns NULL and writes the reason to `out_status` on failure:
/// `MFSK_STATUS_UNKNOWN_PROTOCOL` for a build without the `jtty` feature,
/// `MFSK_STATUS_INVALID_ARG` for a zero rate or bad parameters.
///
/// # Safety
/// `params` must be null or point to at least `params->size` readable bytes;
/// `out_status` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_open(
    sample_rate: u32,
    params: *const MfskJttyParams,
    out_status: *mut MfskStatus,
) -> *mut MfskJttyReceiver {
    let report = |st: MfskStatus| {
        if !out_status.is_null() {
            unsafe { *out_status = st };
        }
    };
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (sample_rate, params);
        set_error("mfsk_jtty_open: this build was compiled without the jtty feature");
        report(MfskStatus::UnknownProtocol);
        ptr::null_mut()
    }
    #[cfg(feature = "jtty")]
    {
        if sample_rate == 0 {
            set_error("mfsk_jtty_open: sample_rate is 0");
            report(MfskStatus::InvalidArg);
            return ptr::null_mut();
        }
        let p = match unsafe { read_jtty_params(params) } {
            Ok(p) => p,
            Err(e) => {
                set_error(format!("mfsk_jtty_open: {e}"));
                report(MfskStatus::InvalidArg);
                return ptr::null_mut();
            }
        };
        let rx = std::sync::Arc::new(mfsk_core::jtty::rx::Receiver::new());
        report(MfskStatus::Ok);
        Box::into_raw(Box::new(JttyInner {
            stream: mfsk_core::jtty::rx::Stream::new(rx, jtty_core_params(&p)),
            sample_rate,
            resampler: (sample_rate != 12_000).then(|| {
                mfsk_core::engine::dsp::resample::LinearResamplerI16To12k::new(sample_rate)
            }),
            queue: Default::default(),
        })) as *mut MfskJttyReceiver
    }
}

/// Release a receiver. Null is a no-op.
///
/// # Safety
/// `rx` must be a handle from [`mfsk_jtty_open`], released once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_close(rx: *mut MfskJttyReceiver) {
    #[cfg(feature = "jtty")]
    if !rx.is_null() {
        drop(unsafe { Box::from_raw(rx as *mut JttyInner) });
    }
    #[cfg(not(feature = "jtty"))]
    let _ = rx;
}

/// Change the receive settings; they apply from the next window.
///
/// # Safety
/// `rx` must be a live handle; `params` must point to at least `params->size`
/// readable bytes (NULL means the defaults).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_set_params(
    rx: *mut MfskJttyReceiver,
    params: *const MfskJttyParams,
) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (rx, params);
        set_error("mfsk_jtty_set_params: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        let Some(r) = jtty_inner(rx) else {
            set_error("mfsk_jtty_set_params: null receiver");
            return MfskStatus::NullPointer;
        };
        match unsafe { read_jtty_params(params) } {
            Ok(p) => {
                r.stream.set_params(jtty_core_params(&p));
                MfskStatus::Ok
            }
            Err(e) => {
                set_error(format!("mfsk_jtty_set_params: {e}"));
                MfskStatus::InvalidArg
            }
        }
    }
}

/// Feed 16-bit mono PCM at the rate the receiver was opened with, any number of
/// samples per call (including none). Every window this completes is decoded
/// before the call returns; what it found waits in the queue for
/// [`mfsk_jtty_poll`]. A call can take a few tens of milliseconds per 0.47 s of
/// audio it completes.
///
/// # Safety
/// `samples` must be `n` readable `int16_t` (or null when `n` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_push_i16(
    rx: *mut MfskJttyReceiver,
    samples: *const i16,
    n: usize,
) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (rx, samples, n);
        set_error("mfsk_jtty_push_i16: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        let Some(r) = jtty_inner(rx) else {
            set_error("mfsk_jtty_push_i16: null receiver");
            return MfskStatus::NullPointer;
        };
        if n == 0 {
            return MfskStatus::Ok;
        }
        if samples.is_null() {
            set_error("mfsk_jtty_push_i16: null samples");
            return MfskStatus::NullPointer;
        }
        let src = unsafe { slice::from_raw_parts(samples, n) };
        match r.resampler.as_mut() {
            None => r.push_12k(src),
            Some(_) => {
                // Resample in bounded chunks, as `mfsk_stream_push_i16` does.
                let mut scratch = [0i16; 4096];
                let mut pos = 0;
                while pos < src.len() {
                    let rs = r.resampler.as_mut().expect("checked");
                    let (consumed, produced) = rs.process(&src[pos..], &mut scratch);
                    if consumed == 0 && produced == 0 {
                        break;
                    }
                    r.push_12k(&scratch[..produced]);
                    pos += consumed;
                }
            }
        }
        MfskStatus::Ok
    }
}

/// [`mfsk_jtty_push_i16`] for 32-bit float PCM, nominally `-1.0..=1.0`.
///
/// # Safety
/// `samples` must be `n` readable `float` (or null when `n` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_push_f32(
    rx: *mut MfskJttyReceiver,
    samples: *const f32,
    n: usize,
) -> MfskStatus {
    if n == 0 {
        return unsafe { mfsk_jtty_push_i16(rx, ptr::null(), 0) };
    }
    if samples.is_null() {
        set_error("mfsk_jtty_push_f32: null samples");
        return MfskStatus::NullPointer;
    }
    let src = unsafe { slice::from_raw_parts(samples, n) };
    let as_i16: Vec<i16> = src
        .iter()
        .map(|&x| (x * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    unsafe { mfsk_jtty_push_i16(rx, as_i16.as_ptr(), as_i16.len()) }
}

/// The audio has ended: queue a last (incomplete) update for every message
/// still waiting for a continuation. Call it when a recording is exhausted;
/// a live receiver never needs it.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_finish(rx: *mut MfskJttyReceiver) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = rx;
        set_error("mfsk_jtty_finish: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        let Some(r) = jtty_inner(rx) else {
            set_error("mfsk_jtty_finish: null receiver");
            return MfskStatus::NullPointer;
        };
        let JttyInner { stream, queue, .. } = r;
        stream.finish(&mut |u| JttyInner::queue_update(queue, u));
        MfskStatus::Ok
    }
}

/// Forget everything — audio, messages, the queue, the resampler's state — and
/// start again at sample 0 with the same settings.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_reset(rx: *mut MfskJttyReceiver) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = rx;
        set_error("mfsk_jtty_reset: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        let Some(r) = jtty_inner(rx) else {
            set_error("mfsk_jtty_reset: null receiver");
            return MfskStatus::NullPointer;
        };
        r.stream.reset();
        r.queue.clear();
        if r.resampler.is_some() {
            r.resampler =
                Some(mfsk_core::engine::dsp::resample::LinearResamplerI16To12k::new(r.sample_rate));
        }
        MfskStatus::Ok
    }
}

/// How many updates are waiting.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_jtty_pending(rx: *mut MfskJttyReceiver) -> usize {
    #[cfg(feature = "jtty")]
    {
        jtty_inner(rx).map_or(0, |r| r.queue.len())
    }
    #[cfg(not(feature = "jtty"))]
    {
        let _ = rx;
        0
    }
}

/// Take the oldest waiting update into `out` (size-versioned: set
/// `out->size = sizeof(MfskJttyUpdate)`, or 0 for the whole struct).
///
/// Returns 1 when an update was written, 0 when none is waiting, and a negative
/// `MfskStatus` on error (a null handle or `out`, or a build without the
/// feature). Call it until it returns 0 after every push:
///
/// ```c
/// MfskJttyUpdate u = {0};
/// while (mfsk_jtty_poll(rx, &u) == 1) show(u.id, u.text, u.complete);
/// ```
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_poll(
    rx: *mut MfskJttyReceiver,
    out: *mut MfskJttyUpdate,
) -> i32 {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (rx, out);
        set_error("mfsk_jtty_poll: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol as i32
    }
    #[cfg(feature = "jtty")]
    {
        let Some(r) = jtty_inner(rx) else {
            set_error("mfsk_jtty_poll: null receiver");
            return MfskStatus::NullPointer as i32;
        };
        if out.is_null() {
            set_error("mfsk_jtty_poll: out is NULL");
            return MfskStatus::NullPointer as i32;
        }
        let Some(u) = r.queue.pop_front() else {
            return 0;
        };
        let mut v = MfskJttyUpdate {
            size: core::mem::size_of::<MfskJttyUpdate>() as u32,
            complete: u32::from(u.complete),
            id: u.id,
            f1_hz: u.f1_hz,
            start_s: u.start_s,
            text: [0; 128],
        };
        // Truncate on a character boundary, leaving room for the NUL.
        let mut end = u.text.len().min(v.text.len() - 1);
        while !u.text.is_char_boundary(end) {
            end -= 1;
        }
        for (d, &b) in v.text.iter_mut().zip(&u.text.as_bytes()[..end]) {
            *d = b as c_char;
        }
        unsafe { write_size_versioned(out, &v) };
        1
    }
}

/// Text → channel tones: pack `text` (NUL-terminated UTF-8, at most 80 characters)
/// into the fewest JTTY frames under `profile` (0 unknown, 1 Field Day, 2 RTTY
/// Roundup) and write the tones, 0‥3, 59 per frame.
///
/// `*out_len` is always set to the number of tones needed (0 for an empty
/// message, which is `MFSK_STATUS_OK` with nothing to send). Pass `tones = NULL`
/// with `cap = 0` to ask for the size first; a buffer too small is
/// `MFSK_STATUS_INVALID_ARG`, as everywhere in the transmit family. A message
/// that cannot be packed (over 80 characters, over 16 frames, an RTTY serial that
/// does not fit) is `MFSK_STATUS_INVALID_ARG` with the reason in `mfsk_last_error`.
///
/// This is upstream's `pack_jtty` and `genjtty`; the F-key templates and N1MM tags
/// around them in WSJT-X are not part of this library.
///
/// # Safety
/// `text` must be a NUL-terminated string; `tones` must be `cap` writable bytes
/// (or null with `cap` 0); `out_len` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_encode_tones(
    text: *const c_char,
    profile: u32,
    tones: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (text, profile, tones, cap, out_len);
        set_error("mfsk_jtty_encode_tones: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        use mfsk_core::jtty::pack::{self, ExchangeProfile};
        if text.is_null() {
            set_error("mfsk_jtty_encode_tones: text is NULL");
            return MfskStatus::NullPointer;
        }
        let profile = match profile {
            0 => ExchangeProfile::Unknown,
            1 => ExchangeProfile::FieldDay,
            2 => ExchangeProfile::RttyRoundup,
            _ => {
                set_error("mfsk_jtty_encode_tones: profile must be 0, 1 or 2");
                return MfskStatus::InvalidArg;
            }
        };
        let Ok(text) = unsafe { CStr::from_ptr(text) }.to_str() else {
            set_error("mfsk_jtty_encode_tones: text is not valid UTF-8");
            return MfskStatus::InvalidArg;
        };
        let atoms = match pack::pack(text, profile) {
            Ok(a) => a,
            Err(e) => {
                set_error(format!("mfsk_jtty_encode_tones: {e}"));
                return MfskStatus::InvalidArg;
            }
        };
        let t = mfsk_core::jtty::tx::tones(&atoms).unwrap_or_default();
        if !out_len.is_null() {
            unsafe { *out_len = t.len() };
        }
        if t.is_empty() {
            return MfskStatus::Ok;
        }
        if tones.is_null() && cap == 0 {
            return MfskStatus::Ok; // a size query
        }
        if tones.is_null() || cap < t.len() {
            set_error("mfsk_jtty_encode_tones: buffer too small; *out_len is the size needed");
            return MfskStatus::InvalidArg;
        }
        unsafe { ptr::copy_nonoverlapping(t.as_ptr(), tones, t.len()) };
        MfskStatus::Ok
    }
}

/// Samples at 12 kHz that `n_tones` JTTY tones synthesise to (whole frames of
/// 59 tones), or 0 if `n_tones` is not a positive multiple of 59.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_jtty_synth_len(n_tones: usize) -> usize {
    #[cfg(feature = "jtty")]
    {
        if n_tones == 0 || !n_tones.is_multiple_of(59) {
            return 0;
        }
        mfsk_core::jtty::tx::samples_for_frames(n_tones / 59)
    }
    #[cfg(not(feature = "jtty"))]
    {
        let _ = n_tones;
        0
    }
}

/// Shared by the two JTTY synthesis entry points: validate, synthesise, and
/// report the length. Returns the samples (empty on a size query).
#[cfg(feature = "jtty")]
#[allow(clippy::too_many_arguments)]
fn jtty_synth(
    what: &str,
    tones: *const u8,
    n_tones: usize,
    freq_hz: f32,
    amplitude: f32,
    out_len: *mut usize,
    cap: usize,
    have_out: bool,
) -> Result<Option<Vec<f32>>, MfskStatus> {
    let need = mfsk_jtty_synth_len(n_tones);
    if !out_len.is_null() {
        unsafe { *out_len = need };
    }
    if tones.is_null() {
        set_error(format!("{what}: tones is NULL"));
        return Err(MfskStatus::NullPointer);
    }
    if need == 0 {
        set_error(format!(
            "{what}: n_tones must be a positive multiple of 59 (one frame is 59 tones)"
        ));
        return Err(MfskStatus::InvalidArg);
    }
    let t = unsafe { slice::from_raw_parts(tones, n_tones) };
    if t.iter().any(|&x| x > 3) {
        set_error(format!("{what}: a tone is above 3"));
        return Err(MfskStatus::InvalidArg);
    }
    if !have_out && cap == 0 {
        return Ok(None); // a size query
    }
    if !have_out || cap < need {
        set_error(format!(
            "{what}: buffer too small; *out_len is the size needed"
        ));
        return Err(MfskStatus::InvalidArg);
    }
    Ok(Some(mfsk_core::jtty::tx::synth_f32(t, freq_hz, amplitude)))
}

/// JTTY tones → 16-bit PCM at 12 kHz, `freq_hz` the frequency of tone 0 (the others
/// are 31.25 Hz apart), `amplitude` the peak in counts (8000 is a sound default).
/// `mfsk_jtty_synth_len(n_tones)` is the capacity needed; `*out_len` reports it, and
/// `out = NULL` with `cap = 0` is a size query. `n_tones` must be a positive multiple
/// of 59.
///
/// # Safety
/// `tones` must be `n_tones` readable bytes; `out` must be `cap` writable `int16_t`
/// (or null with `cap` 0); `out_len` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_tones_to_i16(
    tones: *const u8,
    n_tones: usize,
    freq_hz: f32,
    amplitude: f32,
    out: *mut i16,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (tones, n_tones, freq_hz, amplitude, out, cap, out_len);
        set_error("mfsk_jtty_tones_to_i16: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        match jtty_synth(
            "mfsk_jtty_tones_to_i16",
            tones,
            n_tones,
            freq_hz,
            amplitude,
            out_len,
            cap,
            !out.is_null(),
        ) {
            Err(e) => e,
            Ok(None) => MfskStatus::Ok,
            Ok(Some(pcm)) => {
                let dst = unsafe { slice::from_raw_parts_mut(out, pcm.len()) };
                for (d, &x) in dst.iter_mut().zip(&pcm) {
                    *d = x.round().clamp(-32_768.0, 32_767.0) as i16;
                }
                MfskStatus::Ok
            }
        }
    }
}

/// [`mfsk_jtty_tones_to_i16`] as 32-bit float PCM; `amplitude` is the peak in
/// full-scale units (0.25 is a sound default).
///
/// # Safety
/// As [`mfsk_jtty_tones_to_i16`], with `out` as `float`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_jtty_tones_to_f32(
    tones: *const u8,
    n_tones: usize,
    freq_hz: f32,
    amplitude: f32,
    out: *mut f32,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    #[cfg(not(feature = "jtty"))]
    {
        let _ = (tones, n_tones, freq_hz, amplitude, out, cap, out_len);
        set_error("mfsk_jtty_tones_to_f32: this build was compiled without the jtty feature");
        MfskStatus::UnknownProtocol
    }
    #[cfg(feature = "jtty")]
    {
        match jtty_synth(
            "mfsk_jtty_tones_to_f32",
            tones,
            n_tones,
            freq_hz,
            amplitude,
            out_len,
            cap,
            !out.is_null(),
        ) {
            Err(e) => e,
            Ok(None) => MfskStatus::Ok,
            Ok(Some(pcm)) => {
                unsafe { ptr::copy_nonoverlapping(pcm.as_ptr(), out, pcm.len()) };
                MfskStatus::Ok
            }
        }
    }
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
/// which is rayon's own default.
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
/// configured — rayon's
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

// ──────────────────────────────────────────────────────────────────────────
// Wideband IQ receiver (#534, phase 3)
//
// `mfsk_core::iq::IqReceiver` behind a handle of its own, the way JTTY has
// one: the input is a stream of IQ, not audio, the receiver carries state
// (per-channel filters, open slots, the sample clock), and a slot that
// completes is decoded inside the `push` that finishes it. The rows go into
// a queue in the handle which `mfsk_iq_poll` drains, for the reason
// `mfsk_jtty_poll` gives: a callback cannot cross the C boundary without a
// user-data contract, and Kotlin / Swift / C# wrap a poll more easily.
//
// The handle is not thread-safe: one thread at a time. Since decoding runs
// inside `push`, a caller that cannot block pushes from a worker thread.
// ──────────────────────────────────────────────────────────────────────────

/// Most decodes the queue holds between polls; a caller that never polls
/// loses the oldest.
const IQ_QUEUE_MAX: usize = 4096;

/// One decode with where it came from, as `mfsk_iq_poll` hands it out.
struct IqRow {
    channel: mfsk_core::iq::ChannelId,
    mode: mfsk_core::Mode,
    decoded: mfsk_core::msg::Decoded,
    detail: mfsk_core::decoder::RowDetail,
    abs_freq_hz: f64,
    period: i64,
    slot_start_sample: u64,
    slot_start_utc_ns: Option<i64>,
}

/// The receiver cuts slots; this handle decodes them, one decoder per
/// channel, so a channel keeps its own options and callsign table. The
/// decoders are boxed so their addresses are stable: they are handed out as
/// `MfskDecoder*` by [`mfsk_iq_channel_decoder`].
struct IqInner {
    rx: mfsk_core::iq::IqReceiver,
    decoders: std::collections::HashMap<usize, Box<decoder::FfiDecoder>>,
    queue: std::collections::VecDeque<IqRow>,
    /// Channels [`mfsk_iq_set_early`] turned early decode off for.
    early_off: std::collections::HashSet<usize>,
    /// The prefix points last handed to the receiver, by channel.
    points: std::collections::HashMap<usize, &'static [usize]>,
    /// By channel, the period whose prefixes its decoder has decoded and
    /// whose whole slot has not come yet.
    open: std::collections::HashMap<usize, i64>,
}

fn iq_inner<'a>(rx: *mut MfskIqReceiver) -> Option<&'a mut IqInner> {
    unsafe { (rx as *mut IqInner).as_mut() }
}

/// The `Mode` a `MfskMode` addresses, or `None` for a mode the IQ receiver
/// does not carry (MSK144, JTTY, uvpacket) or a build without it.
fn iq_mode_of(m: MfskMode) -> Option<mfsk_core::Mode> {
    use mfsk_core::Mode as I;
    Some(match m {
        MfskMode::Ft8 => I::Ft8,
        MfskMode::Ft4 => I::Ft4,
        MfskMode::Fst4s15 => I::Fst4S15,
        MfskMode::Fst4s30 => I::Fst4S30,
        MfskMode::Fst4s60 => I::Fst4S60,
        MfskMode::Fst4s120 => I::Fst4S120,
        MfskMode::Fst4s300 => I::Fst4S300,
        MfskMode::Wspr => I::Wspr,
        MfskMode::Jt9 => I::Jt9,
        MfskMode::Jt65 => I::Jt65,
        MfskMode::Q65a15 => I::Q65A15,
        MfskMode::Q65a30 => I::Q65A30,
        MfskMode::Q65a60 => I::Q65A60,
        MfskMode::Q65b60 => I::Q65B60,
        MfskMode::Q65c60 => I::Q65C60,
        MfskMode::Q65d60 => I::Q65D60,
        MfskMode::Q65e60 => I::Q65E60,
        MfskMode::Q65d120 => I::Q65D120,
        MfskMode::Q65e120 => I::Q65E120,
        MfskMode::Q65a300 => I::Q65A300,
        _ => return None,
    })
}

/// The `MfskMode` an `Mode` came from: the inverse of [`iq_mode_of`], by
/// search, so the two cannot disagree.
fn mfsk_mode_of_iq(m: mfsk_core::Mode) -> MfskMode {
    (0u32..)
        .map_while(mode_of_index)
        .find(|x| iq_mode_of(*x) == Some(m))
        .expect("every Mode is some MfskMode's")
}

/// `MfskMode` by discriminant, for as long as there is one.
fn mode_of_index(i: u32) -> Option<MfskMode> {
    mode_of(i)
}

fn iq_error_status(e: mfsk_core::iq::IqError) -> MfskStatus {
    let _ = e;
    MfskStatus::InvalidArg
}

/// Open a receiver for an IQ stream: `sample_rate` complex samples per second
/// (any integer of 12 000 or more whose ratio to 12 kHz is a small fraction),
/// `center_hz` the RF frequency of DC, `format` one of `MFSK_IQ_FORMAT_*`,
/// `iq_swap` non-zero when I and Q are exchanged (sound-card IQ often is).
///
/// Returns NULL and writes the reason to `out_status` on failure:
/// `MFSK_STATUS_INVALID_ARG` for an unknown format, a rate below 12 kHz or a
/// non-finite centre.
///
/// # Safety
/// `out_status` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_open(
    sample_rate: u32,
    center_hz: f64,
    format: u32,
    iq_swap: u32,
    out_status: *mut MfskStatus,
) -> *mut MfskIqReceiver {
    unsafe {
        mfsk_iq_open_with(
            sample_rate,
            center_hz,
            format,
            iq_swap,
            MFSK_IQ_CHANNELIZER_DIRECT,
            out_status,
        )
    }
}

/// [`mfsk_iq_open`] with the channelizer chosen: `MFSK_IQ_CHANNELIZER_DIRECT`
/// (what `mfsk_iq_open` gives) or `MFSK_IQ_CHANNELIZER_PFB`. Both give the
/// decoders the same audio at the same 120 dB selectivity; the bank costs
/// more for one channel and less from about four (768 kS/s: 2.7 % of a core
/// for one, 10 % for 32, against 0.9 % and 30 % direct). `INVALID_ARG` for an
/// unknown value, or `PFB` at a rate no bank fits (under 40 kS/s).
///
/// # Safety
/// `out_status` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_open_with(
    sample_rate: u32,
    center_hz: f64,
    format: u32,
    iq_swap: u32,
    channelizer: u32,
    out_status: *mut MfskStatus,
) -> *mut MfskIqReceiver {
    use mfsk_core::iq::IqSampleFormat as F;
    let report = |st: MfskStatus| {
        if !out_status.is_null() {
            unsafe { *out_status = st };
        }
    };
    let format = match format {
        MFSK_IQ_FORMAT_CF32 => F::Cf32,
        MFSK_IQ_FORMAT_CS16 => F::Cs16,
        MFSK_IQ_FORMAT_CS8 => F::Cs8,
        MFSK_IQ_FORMAT_CU8 => F::Cu8,
        MFSK_IQ_FORMAT_CS24 => F::Cs24,
        _ => {
            set_error("mfsk_iq_open: not an MFSK_IQ_FORMAT_* value");
            report(MfskStatus::InvalidArg);
            return ptr::null_mut();
        }
    };
    if sample_rate < 12_000 || !center_hz.is_finite() {
        set_error("mfsk_iq_open: sample_rate must be at least 12000 and center_hz finite");
        report(MfskStatus::InvalidArg);
        return ptr::null_mut();
    }
    let kind = match channelizer {
        MFSK_IQ_CHANNELIZER_DIRECT => mfsk_core::iq::Channelizer::Direct,
        MFSK_IQ_CHANNELIZER_PFB => mfsk_core::iq::Channelizer::Pfb,
        _ => {
            set_error("mfsk_iq_open_with: not an MFSK_IQ_CHANNELIZER_* value");
            report(MfskStatus::InvalidArg);
            return ptr::null_mut();
        }
    };
    let stream = mfsk_core::iq::IqStream::new(sample_rate, center_hz, format).iq_swap(iq_swap != 0);
    let rx = match mfsk_core::iq::IqReceiver::with_channelizer(stream, kind) {
        Ok(rx) => rx,
        Err(e) => {
            set_error(format!("mfsk_iq_open_with: {e}"));
            report(MfskStatus::InvalidArg);
            return ptr::null_mut();
        }
    };
    report(MfskStatus::Ok);
    Box::into_raw(Box::new(IqInner {
        rx,
        decoders: Default::default(),
        early_off: Default::default(),
        points: Default::default(),
        open: Default::default(),
        queue: Default::default(),
    })) as *mut MfskIqReceiver
}

/// Release a receiver. Null is a no-op.
///
/// # Safety
/// `rx` must be a handle from [`mfsk_iq_open`], released once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_close(rx: *mut MfskIqReceiver) {
    if !rx.is_null() {
        drop(unsafe { Box::from_raw(rx as *mut IqInner) });
    }
}

/// Add a channel whose dial (audio 0 Hz) is `dial_hz`, carrying `mode` (a
/// `MfskMode`: FT8, FT4, the five FST4 periods, WSPR, JT9, JT65 or a Q65
/// sub-mode), decoded with its own decoder: `params` and `extras` are as
/// [`mfsk_decoder_open`]'s (NULL for the mode's defaults). On success
/// `*out_channel` is the handle rows carry.
///
/// `MFSK_STATUS_INVALID_ARG` when the channel cannot be placed (DC inside its
/// 0-6 kHz audio window, or the window outside the IQ band) or the mode is not
/// one the receiver carries; `MFSK_STATUS_UNSUPPORTED` for an option the mode
/// lacks; `MFSK_STATUS_UNKNOWN_PROTOCOL` for a mode this build was compiled
/// without.
///
/// # Safety
/// `params` and `extras` must each be null or valid; `out_channel` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_add_channel(
    rx: *mut MfskIqReceiver,
    dial_hz: f64,
    mode: u32,
    params: *const MfskParams,
    extras: *const MfskExtras,
    out_channel: *mut u32,
) -> MfskStatus {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_add_channel: null receiver");
        return MfskStatus::NullPointer;
    };
    let Some(m) = mode_of(mode) else {
        set_error("mfsk_iq_add_channel: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    // A mode the receiver never carries (MSK144, JTTY, uvpacket) is the
    // caller's mistake whatever the build; only then is a carried mode
    // that this build lacks `UnknownProtocol`.
    let Some(iq_mode) = iq_mode_of(m) else {
        set_error("mfsk_iq_add_channel: the IQ receiver does not carry this mode");
        return MfskStatus::InvalidArg;
    };
    if mode_meta(m).is_none() {
        set_error("mfsk_iq_add_channel: no such mode in this build");
        return MfskStatus::UnknownProtocol;
    }
    if !dial_hz.is_finite() {
        set_error("mfsk_iq_add_channel: dial_hz is not finite");
        return MfskStatus::InvalidArg;
    }
    let decoder = match unsafe { decoder::open_decoder(mode, params, extras) } {
        Ok(d) => d,
        Err((st, msg)) => {
            set_error(format!("mfsk_iq_add_channel: {msg}"));
            return st;
        }
    };
    match r.rx.add_channel(dial_hz, iq_mode) {
        Ok(id) => {
            r.decoders.insert(id.0, Box::new(decoder));
            if !out_channel.is_null() {
                unsafe { *out_channel = id.0 as u32 };
            }
            MfskStatus::Ok
        }
        Err(e) => {
            set_error(format!("mfsk_iq_add_channel: {e}"));
            iq_error_status(e)
        }
    }
}

/// The channel's decoder, as an `MfskDecoder*` for the calls that configure
/// one: `mfsk_decoder_set_params`, `_set_extras`, `_add_callsign`,
/// `_unpack77`, `_last_error`, `_set_q65_callers`, `_clear`. Borrowed: **do
/// not close it**; it lives until the channel is removed or the receiver is
/// closed. The receiver decodes with it, so do not call its `decode_*`
/// yourself. NULL if there is no such channel.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_channel_decoder(
    rx: *mut MfskIqReceiver,
    channel: u32,
) -> *mut MfskDecoder {
    match iq_inner(rx).and_then(|r| r.decoders.get_mut(&(channel as usize))) {
        Some(d) => &mut **d as *mut decoder::FfiDecoder as *mut MfskDecoder,
        None => ptr::null_mut(),
    }
}

/// `mfsk_iq_channel_state`: being received.
pub const MFSK_IQ_CHANNEL_ACTIVE: i32 = 0;
/// Paused: its audio window no longer fits the IQ band after a retune. It
/// keeps its dial and its decoder, and resumes when a later retune brings it
/// back inside the band.
pub const MFSK_IQ_CHANNEL_PAUSED: i32 = 1;

/// Whether a channel is being received: `MFSK_IQ_CHANNEL_ACTIVE`,
/// `MFSK_IQ_CHANNEL_PAUSED`, or -1 if there is no such channel.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_channel_state(rx: *mut MfskIqReceiver, channel: u32) -> i32 {
    use mfsk_core::iq::ChannelState;
    match iq_inner(rx).and_then(|r| {
        r.rx.channel_state(mfsk_core::iq::ChannelId(channel as usize))
    }) {
        Some(ChannelState::Active) => MFSK_IQ_CHANNEL_ACTIVE,
        Some(_) => MFSK_IQ_CHANNEL_PAUSED,
        None => -1,
    }
}

/// Decode a channel early, or not (#601). On (the default for every
/// channel): when the channel decoder has checkpoints — FT8 at Normal or
/// Deep depth, whose `SicEarly` strategy acts at ~11.8 s — the receiver hands
/// it the slot so far at each one, so its rows reach the decoder's
/// `mfsk_decoder_set_on_decode` callback and [`mfsk_iq_poll`] before the slot
/// is whole, with `stage == MFSK_STAGE_EARLY`, as WSJT-X shows them; the
/// whole slot then adds the rest, and a row already queued early is not
/// queued again. Every other mode and depth decodes the whole slot either
/// way. Off: whole slots only, as before. Takes effect from the next slot
/// that opens. `MFSK_STATUS_INVALID_ARG` if there is no such channel.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_set_early(
    rx: *mut MfskIqReceiver,
    channel: u32,
    on: bool,
) -> MfskStatus {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_set_early: null receiver");
        return MfskStatus::NullPointer;
    };
    let id = channel as usize;
    if !r.decoders.contains_key(&id) {
        set_error("mfsk_iq_set_early: no such channel");
        return MfskStatus::InvalidArg;
    }
    if on {
        r.early_off.remove(&id);
    } else {
        r.early_off.insert(id);
    }
    MfskStatus::Ok
}

/// Remove a channel. `MFSK_STATUS_INVALID_ARG` if there is no such channel.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_remove_channel(
    rx: *mut MfskIqReceiver,
    channel: u32,
) -> MfskStatus {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_remove_channel: null receiver");
        return MfskStatus::NullPointer;
    };
    if r.rx
        .remove_channel(mfsk_core::iq::ChannelId(channel as usize))
    {
        let id = channel as usize;
        r.decoders.remove(&id);
        r.early_off.remove(&id);
        r.points.remove(&id);
        r.open.remove(&id);
        MfskStatus::Ok
    } else {
        set_error("mfsk_iq_remove_channel: no such channel");
        MfskStatus::InvalidArg
    }
}

/// The stream's complex sample `at_sample` (as `mfsk_iq_samples_in` counts)
/// was at UTC `utc_ns` (ns since the Unix epoch). Call it as often as you
/// have a reading: the receiver follows the readings at up to 400 ppm, so a
/// drifting crystal or host clock moves slot boundaries by milliseconds and
/// loses no slot; only a jump of more than a second drops the slots that
/// straddle it. Without any reading the grid free-runs from sample 0, right
/// for replaying a recording. `*out_change` receives `MFSK_CLOCK_*`.
///
/// # Safety
/// `rx` must be a live handle; `out_change` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_set_time(
    rx: *mut MfskIqReceiver,
    utc_ns: i64,
    at_sample: u64,
    out_change: *mut i32,
) -> MfskStatus {
    use mfsk_core::slotgrid::ClockChange;
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_set_time: null receiver");
        return MfskStatus::NullPointer;
    };
    let code = match r.rx.set_time(utc_ns, at_sample) {
        ClockChange::First => MFSK_CLOCK_FIRST,
        ClockChange::Slewed { .. } => MFSK_CLOCK_SLEWED,
        _ => MFSK_CLOCK_STEPPED,
    };
    if !out_change.is_null() {
        unsafe { *out_change = code };
    }
    MfskStatus::Ok
}

/// The tuner moved to `center_hz`: every channel that still fits is re-placed
/// against it, one that no longer fits is paused (it keeps its dial and its
/// decoder, and resumes when a later retune brings it back inside the band),
/// and the open slots are dropped; the sample clock continues.
/// `*out_paused` and `*out_resumed` receive how many channels changed state;
/// ask [`mfsk_iq_channel_state`] which.
///
/// # Safety
/// `rx` must be a live handle; the out pointers may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_retune(
    rx: *mut MfskIqReceiver,
    center_hz: f64,
    out_paused: *mut u32,
    out_resumed: *mut u32,
) -> MfskStatus {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_retune: null receiver");
        return MfskStatus::NullPointer;
    };
    if !center_hz.is_finite() {
        set_error("mfsk_iq_retune: center_hz is not finite");
        return MfskStatus::InvalidArg;
    }
    let report = r.rx.retune(center_hz);
    if !out_paused.is_null() {
        unsafe { *out_paused = report.paused.len() as u32 };
    }
    if !out_resumed.is_null() {
        unsafe { *out_resumed = report.resumed.len() as u32 };
    }
    MfskStatus::Ok
}

/// `lost` samples never arrived: the clock advances past them and the open
/// slots are dropped.
///
/// # Safety
/// `rx` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_gap(rx: *mut MfskIqReceiver, lost: u64) -> MfskStatus {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_gap: null receiver");
        return MfskStatus::NullPointer;
    };
    r.rx.gap(lost);
    MfskStatus::Ok
}

/// Push `n_bytes` of IQ in the format the receiver was opened with,
/// little-endian, I then Q; a sample split across calls is carried over. Every
/// slot this completes is decoded before the call returns, and so is every
/// early checkpoint it reaches on a channel that decodes early
/// ([`mfsk_iq_set_early`]); what they found goes to the channel decoder's
/// callback as it is found and waits for [`mfsk_iq_poll`].
///
/// # Safety
/// `data` must be `n_bytes` readable bytes (or null when `n_bytes` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_push(
    rx: *mut MfskIqReceiver,
    data: *const c_void,
    n_bytes: usize,
) -> MfskStatus {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_push: null receiver");
        return MfskStatus::NullPointer;
    };
    if n_bytes == 0 {
        return MfskStatus::Ok;
    }
    if data.is_null() {
        set_error("mfsk_iq_push: data is NULL");
        return MfskStatus::NullPointer;
    }
    let bytes = unsafe { slice::from_raw_parts(data as *const u8, n_bytes) };
    // Each channel's points from its decoder as it is now: the caller sets
    // the depth or strategy on the borrowed decoder, which has no hook back
    // here, so this is where a change is seen (from the next slot that opens).
    for (&id, d) in &r.decoders {
        let want: &'static [usize] = if r.early_off.contains(&id) {
            &[]
        } else {
            d.prefix_points()
        };
        if r.points.get(&id) != Some(&want) {
            r.rx.set_prefix_points(mfsk_core::iq::ChannelId(id), want);
            r.points.insert(id, want);
        }
    }
    in_pool_mut(|| {
        let mut slots = Vec::new();
        r.rx.push_bytes(bytes, &mut slots);
        for slot in &slots {
            let id = slot.channel.0;
            let Some(d) = r.decoders.get_mut(&id) else {
                continue;
            };
            // A prefix, or the whole slot of a period whose prefixes this
            // decoder took: one `decode_prefix` sequence. A whole slot with
            // none before it is a plain decode.
            let whole = slot.is_whole();
            let prefix = !whole || r.open.get(&id) == Some(&slot.period);
            d.decode_slot(&slot.audio, slot.period, prefix);
            if whole {
                r.open.remove(&id);
            } else {
                r.open.insert(id, slot.period);
            }
            for (decoded, detail) in d.rows() {
                // The final call returns the period's whole set; its early
                // rows were queued by the prefix call that found them.
                if prefix && whole && detail.stage == Some(mfsk_core::decoder::Stage::Early) {
                    continue;
                }
                if r.queue.len() >= IQ_QUEUE_MAX {
                    r.queue.pop_front();
                }
                r.queue.push_back(IqRow {
                    channel: slot.channel,
                    mode: slot.mode,
                    abs_freq_hz: slot.abs_freq_hz(decoded.freq_hz),
                    period: slot.period,
                    slot_start_sample: slot.start_sample,
                    slot_start_utc_ns: slot.utc_ns,
                    decoded: decoded.clone(),
                    detail: detail.clone(),
                });
            }
        }
    });
    MfskStatus::Ok
}

/// Complex samples consumed so far, gaps included: the stream's clock.
///
/// # Safety
/// `rx` must be a live handle or null (0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_samples_in(rx: *mut MfskIqReceiver) -> u64 {
    iq_inner(rx).map(|r| r.rx.samples_in()).unwrap_or(0)
}

/// How many decodes wait for [`mfsk_iq_poll`].
///
/// # Safety
/// `rx` must be a live handle or null (0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_pending(rx: *mut MfskIqReceiver) -> usize {
    iq_inner(rx).map(|r| r.queue.len()).unwrap_or(0)
}

/// Take the oldest waiting decode into `*out` (`out->size` is
/// `sizeof(MfskIqDecode)`, or 0 for the whole struct).
///
/// Returns 1 when a decode was written, 0 when none is waiting, and a negative
/// `MfskStatus` on error (a null handle or `out`). Call it until it returns 0
/// after every push:
///
/// ```c
/// MfskIqDecode d = {0};
/// while (mfsk_iq_poll(rx, &d) == 1) show(d.channel, d.text, d.abs_freq_hz);
/// ```
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_iq_poll(rx: *mut MfskIqReceiver, out: *mut MfskIqDecode) -> i32 {
    let Some(r) = iq_inner(rx) else {
        set_error("mfsk_iq_poll: null receiver");
        return MfskStatus::NullPointer as i32;
    };
    if out.is_null() {
        set_error("mfsk_iq_poll: out is NULL");
        return MfskStatus::NullPointer as i32;
    }
    let Some(d) = r.queue.pop_front() else {
        return 0;
    };
    let mut v = MfskIqDecode {
        size: core::mem::size_of::<MfskIqDecode>() as u32,
        channel: d.channel.0 as u32,
        mode: mfsk_mode_of_iq(d.mode),
        has_utc: u32::from(d.slot_start_utc_ns.is_some()),
        abs_freq_hz: d.abs_freq_hz,
        period: d.period,
        slot_start_sample: d.slot_start_sample,
        slot_start_utc_ns: d.slot_start_utc_ns.unwrap_or(0),
        freq_hz: d.decoded.freq_hz,
        dt_sec: d.decoded.dt_sec,
        snr_db: d.decoded.snr_db,
        text: [0; mfsk_ffi_abi::MFSK_DECODE_TEXT_LEN],
        sync_score: 0.0,
        sync_cv: 0.0,
        hard_errors: 0,
        delivery: -1,
        pass: 0,
        flags: 0,
        key_bits: 0,
        key: [0; mfsk_ffi_abi::MFSK_DECODE_KEY_LEN],
        stage: 0,
    };
    // The detail exactly as the channel decoder's own row gives it.
    let row = decoder::row_of(v.mode, &d.decoded, &d.detail);
    v.sync_score = row.sync_score;
    v.sync_cv = row.sync_cv;
    v.hard_errors = row.hard_errors;
    v.delivery = row.delivery;
    v.pass = row.pass;
    v.flags = row.flags;
    v.key_bits = row.key_bits;
    v.key = row.key;
    v.stage = row.stage;
    let mut end = d.decoded.text.len().min(v.text.len() - 1);
    while !d.decoded.text.is_char_boundary(end) {
        end -= 1;
    }
    for (dst, &b) in v.text.iter_mut().zip(&d.decoded.text.as_bytes()[..end]) {
        *dst = b as c_char;
    }
    unsafe { write_size_versioned(out, &v) };
    1
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
