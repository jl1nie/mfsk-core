//! Embedded-friendly C ABI for the FT8 block decoder slice of
//! `mfsk-core`.
//!
//! This crate is `no_std + alloc` under the `embedded-fixed-point`
//! feature so the resulting `libmfsk_ft8.a` can drop into a non-Rust
//! ESP-IDF / RP2040 / Cortex-M C project without dragging Rust's
//! `std` runtime. The linking C project supplies:
//!
//! 1. A `#[panic_handler]` and `#[global_allocator]` (typically a
//!    ~30-line Rust shim — see `embedded-poc/idf-component/shim/`).
//! 2. The `mfsk_core_make_default_fft_planner` extern Rust symbol
//!    (FFT backend factory — esp-dsp on Xtensa, CMSIS-DSP on
//!    Cortex-M).
//!
//! Under the default `host` feature the crate builds with `std` and
//! rustfft for desktop testing; the resulting `cdylib` /
//! `staticlib` works exactly like a slimmed-down `mfsk-ffi`.
//!
//! # API shape
//!
//! [`mfsk_ft8_decode_i16`] runs the embedded fixed-point decode path
//! and populates a [`MfskFt8ResultList`] the caller frees with
//! [`mfsk_ft8_result_list_free`]. Prior to 0.8.0 this function also
//! took caller-provided BASIS scratch buffers (`basis_re`/`basis_im`)
//! — removed (issue #162) once the Goertzel fill path (Phase
//! 1.7.7-Stick) became the sole production path on every embedded
//! target, making the scratch dead weight. **Breaking change**: C
//! callers built against pre-0.8.0 headers must drop the two
//! `int16_t*` scratch arguments from their `mfsk_ft8_decode_i16`
//! call site.

#![cfg_attr(not(feature = "host"), no_std)]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ffi::{c_char, c_int};

#[cfg(feature = "host")]
use mfsk_core::ft8::decode_block::decode_block;

#[cfg(feature = "embedded-fixed-point")]
use mfsk_core::ft8::decode_block::decode_block_into;

use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::wave_gen::{
    TONES_OUTPUT_LEN, message_to_tones, tones_to_f32_into, tones_to_i16_into,
};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// ── Status codes ────────────────────────────────────────────────────────────

/// Outcome of a decode call.
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MfskFt8Status {
    /// Decode completed (zero or more results in `out`).
    Ok = 0,
    /// One of the input pointers was null.
    NullPointer = -1,
    /// `n_samples` is too short for a 14-second slot at 12 kHz.
    AudioTooShort = -2,
    /// `depth` is outside `0..=2`.
    BadDepth = -3,
    /// Caller-provided output buffer too small (see
    /// [`mfsk_ft8_tones_to_i16`] / [`mfsk_ft8_tones_to_f32`]).
    ScratchTooSmall = -4,
}

// ── Decode-depth selector ───────────────────────────────────────────────────

/// Mirrors `mfsk_core::ft8::decode::DecodeDepth`. The deeper levels
/// trade decode time for recall on busy bands.
///
/// Discriminant `0` is intentionally unassigned — the legacy
/// `Bp` (single-metric) rung was retired in 0.7.0 (issue #74).
/// Existing C callers passing `1` / `2` remain valid; passing `0`
/// is now rejected by `map_depth` and surfaces as a caller bug.
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MfskFt8Depth {
    /// Four full LLR variants + BP run per candidate.
    BpAll = 1,
    /// Above + OSD-1 / OSD-3 fallback (gated on sync-quality ≥ 12).
    BpAllOsd = 2,
}

#[inline]
fn map_depth(d: MfskFt8Depth) -> DecodeDepth {
    match d {
        MfskFt8Depth::BpAll => DecodeDepth::BP_ONLY,
        MfskFt8Depth::BpAllOsd => DecodeDepth::FULL,
    }
}

// ── Result records ──────────────────────────────────────────────────────────

/// One decoded FT8 message.
///
/// `text` is a C string of at most 39 visible UTF-8 characters plus
/// NUL terminator (FT8's 77-bit message decompresses to ≤ ~36 chars
/// in practice — the buffer is sized at 40 for safety).
#[repr(C)]
pub struct MfskFt8Result {
    /// NUL-terminated UTF-8 unpacked message. ASCII in practice.
    pub text: [c_char; 40],
    /// Carrier frequency of the decoded slot, Hz.
    pub freq_hz: f32,
    /// Time offset relative to the slot start, seconds.
    pub dt_sec: f32,
    /// SNR estimate (dB, WSJT-X 2500 Hz reference). On the embedded
    /// path this reads ~4–12 dB low on strong signals — see
    /// `docs/reference/EMBEDDED.md` "Known limitations".
    pub snr_db: f32,
    /// Number of bits the LDPC decoder corrected before CRC pass.
    pub hard_errors: u32,
    /// Stage that produced the decode (0 = fast Bp, 1 = full Bp, …).
    pub pass: u8,
    /// Padding to keep the struct C-friendly across compilers.
    pub _pad: [u8; 3],
}

/// Result list — owned by the FFI side. Free with
/// [`mfsk_ft8_result_list_free`].
#[repr(C)]
pub struct MfskFt8ResultList {
    /// Pointer to the first result, or null if `len == 0`.
    pub items: *mut MfskFt8Result,
    /// Number of valid entries.
    pub len: usize,
    /// Capacity (private — only the free function reads this).
    pub _capacity: usize,
}

#[inline]
fn empty_result_list() -> MfskFt8ResultList {
    MfskFt8ResultList {
        items: core::ptr::null_mut(),
        len: 0,
        _capacity: 0,
    }
}

#[inline]
fn finalise_results(results: Vec<DecodeResult>, out: *mut MfskFt8ResultList) {
    let mut converted: Vec<MfskFt8Result> = Vec::with_capacity(results.len());
    for r in results {
        let mut rec = MfskFt8Result {
            text: [0; 40],
            freq_hz: r.freq_hz,
            dt_sec: r.dt_sec,
            snr_db: r.snr_db,
            hard_errors: r.hard_errors,
            pass: r.pass,
            _pad: [0; 3],
        };
        if let Some(text) = unpack77(&r.message77) {
            let bytes = text.as_bytes();
            let n = bytes.len().min(rec.text.len() - 1);
            // Reinterpret c_char vs u8 portably (c_char is i8 on
            // most targets, u8 on a few — narrowing copy below
            // handles both).
            for (dst, &src) in rec.text.iter_mut().zip(bytes.iter()).take(n) {
                *dst = src as c_char;
            }
        }
        converted.push(rec);
    }
    let mut boxed = converted.into_boxed_slice();
    let items = boxed.as_mut_ptr();
    let len = boxed.len();
    core::mem::forget(boxed);
    unsafe {
        (*out).items = items;
        (*out).len = len;
        (*out)._capacity = len;
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

/// **Primary FT8 decode** for embedded fixed-point targets. **Use
/// this in production embedded code.**
///
/// `freq_min_hz` / `freq_max_hz` bound the carrier search range
/// (typical: 200, 3000). `sync_min` is the stage-2 candidate
/// threshold (typical 1.0). `max_cand` caps the survivors after
/// Pass 2 (typical 30 for embedded busy-band). `depth` picks the
/// decoder staircase (`MFSK_FT8_DEPTH_BP_ALL_OSD` = 2 is the most
/// thorough).
///
/// Only available under the `embedded-fixed-point` feature (the
/// path lives in mfsk-core's `cfg(fixed-point)` block). Host code
/// should use [`mfsk_ft8_decode_i16_alloc`].
///
/// **Breaking change (0.8.0, issue #162)**: this function used to
/// also take caller-provided BASIS scratch (`basis_re`/`basis_im`
/// `int16_t*` arrays, sized via a now-removed
/// `mfsk_ft8_basis_scratch_len()`). The BASIS fill path was retired
/// in favour of the scratch-free Goertzel fill (Phase 1.7.7-Stick),
/// making the scratch arguments dead weight — callers built against
/// pre-0.8.0 headers must drop both arguments from their call site.
///
/// # Safety
/// `audio` must point to `n_samples` valid `i16` values; at least
/// 168 000 (14 s × 12 kHz). `out` must point to a writable
/// [`MfskFt8ResultList`]. Single-threaded — one decode at a time per
/// process.
#[unsafe(no_mangle)]
#[cfg(feature = "embedded-fixed-point")]
pub unsafe extern "C" fn mfsk_ft8_decode_i16(
    audio: *const i16,
    n_samples: usize,
    freq_min_hz: f32,
    freq_max_hz: f32,
    sync_min: f32,
    max_cand: c_int,
    depth: MfskFt8Depth,
    out: *mut MfskFt8ResultList,
) -> MfskFt8Status {
    if audio.is_null() || out.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if n_samples < 168_000 {
        unsafe { *out = empty_result_list() };
        return MfskFt8Status::AudioTooShort;
    }
    let slot = unsafe { core::slice::from_raw_parts(audio, n_samples) };
    unsafe { *out = empty_result_list() };
    let results = decode_block_into(
        slot,
        freq_min_hz,
        freq_max_hz,
        sync_min,
        map_depth(depth),
        max_cand.max(0) as usize,
    );
    finalise_results(results, out);
    MfskFt8Status::Ok
}

/// **Host-only heap-alloc convenience** — same shape as
/// [`mfsk_ft8_decode_i16`] but allocates the BASIS scratch
/// internally on every call (~60 KB × 2 = ~120 KB heap traffic per
/// decode).
///
/// Only available on host (`feature = "host"`). Deliberately
/// excluded from embedded builds: surprise per-call heap-allocation
/// of a 60 KB scratch is a bad default for an MCU, and on Core2 with
/// PSRAM the scratch lands in slow PSRAM and tanks the dot-product
/// inner kernel. Embedded callers must use [`mfsk_ft8_decode_i16`]
/// with caller-managed scratch in internal RAM.
///
/// # Safety
/// Same as [`mfsk_ft8_decode_i16`] minus the scratch arguments.
#[unsafe(no_mangle)]
#[cfg(feature = "host")]
pub unsafe extern "C" fn mfsk_ft8_decode_i16_alloc(
    audio: *const i16,
    n_samples: usize,
    freq_min_hz: f32,
    freq_max_hz: f32,
    sync_min: f32,
    max_cand: c_int,
    depth: MfskFt8Depth,
    out: *mut MfskFt8ResultList,
) -> MfskFt8Status {
    if audio.is_null() || out.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if n_samples < 168_000 {
        unsafe { *out = empty_result_list() };
        return MfskFt8Status::AudioTooShort;
    }
    let slot = unsafe { core::slice::from_raw_parts(audio, n_samples) };
    unsafe { *out = empty_result_list() };
    let results = decode_block(
        slot,
        freq_min_hz,
        freq_max_hz,
        sync_min,
        map_depth(depth),
        max_cand.max(0) as usize,
    );
    finalise_results(results, out);
    MfskFt8Status::Ok
}

/// Free a result list previously populated by
/// `mfsk_ft8_decode_i16` / `mfsk_ft8_decode_i16_into`. Safe to call
/// with a zero-length list. After this returns, the list's `items`
/// pointer is invalid; the struct itself remains valid for reuse.
///
/// # Safety
/// `list` must be either null (no-op) or point to a `MfskFt8ResultList`
/// previously populated by this crate.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_result_list_free(list: *mut MfskFt8ResultList) {
    if list.is_null() {
        return;
    }
    let l = unsafe { &mut *list };
    if !l.items.is_null() && l._capacity > 0 {
        let slice = unsafe { core::slice::from_raw_parts_mut(l.items, l._capacity) };
        let _ = unsafe { Box::from_raw(slice) };
    }
    l.items = core::ptr::null_mut();
    l.len = 0;
    l._capacity = 0;
}

// Suppress dead-code warning on the host build for the embedded-only
// helper imports.
#[cfg(all(feature = "host", not(feature = "embedded-fixed-point")))]
#[allow(dead_code)]
fn _silence_unused() {
    let _ = empty_result_list;
}

// ── TX (encode + synthesise) ────────────────────────────────────────────────
//
// Mirror of `mfsk_core::ft8::wave_gen` shape — caller-provided
// output buffers throughout (no surprise heap-alloc on embedded).
// The standard FT8 transmit chain is:
//
//     mfsk_ft8_pack77   (3 strings → 77-bit message)
//     mfsk_ft8_message_to_tones (77-bit → 79-tone Gray-mapped sequence)
//     mfsk_ft8_tones_to_i16    (79-tone → 12 kHz PCM, 151 680 samples)
//
// Each step is independently callable — e.g. you can synth from a
// `[u8; 79]` pre-cooked tone array without ever calling `pack77`,
// or stop after `message_to_tones` and feed itone to your own GFSK.

/// Required output length (in samples) for the synth functions.
/// Equals `NN × 1920 = 151 680` (12.64 s of 12 kHz mono).
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_ft8_synth_output_len() -> usize {
    TONES_OUTPUT_LEN
}

/// Pack a standard 77-bit FT8 message from three tokens — the
/// typical CQ shape `mfsk_ft8_pack77("CQ", "JA1ABC", "PM86")`. For
/// reply / report messages: `mfsk_ft8_pack77("JA1ABC", "W1AW",
/// "-12")`. Strings must be NUL-terminated UTF-8 (ASCII in
/// practice).
///
/// Writes 77 bytes (each 0 or 1) to `out_message77`. Returns
/// `MfskFt8Status::Ok` on success, `BadDepth` is reused as a
/// generic "bad input" indicator if any string fails to pack
/// (callsign too long, bad characters, etc).
///
/// # Safety
/// `call1`, `call2`, `report` must point to valid NUL-terminated
/// C strings. `out_message77` must point to a writable 77-byte
/// buffer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_pack77(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    out_message77: *mut u8,
) -> MfskFt8Status {
    if call1.is_null() || call2.is_null() || report.is_null() || out_message77.is_null() {
        return MfskFt8Status::NullPointer;
    }
    // SAFETY: caller upholds NUL-terminated strings.
    let s1 = match unsafe { core::ffi::CStr::from_ptr(call1) }.to_str() {
        Ok(s) => s,
        Err(_) => return MfskFt8Status::BadDepth,
    };
    let s2 = match unsafe { core::ffi::CStr::from_ptr(call2) }.to_str() {
        Ok(s) => s,
        Err(_) => return MfskFt8Status::BadDepth,
    };
    let s3 = match unsafe { core::ffi::CStr::from_ptr(report) }.to_str() {
        Ok(s) => s,
        Err(_) => return MfskFt8Status::BadDepth,
    };
    let Some(msg) = pack77(s1, s2, s3) else {
        return MfskFt8Status::BadDepth;
    };
    // SAFETY: caller provided 77-byte writable buffer.
    let dst = unsafe { core::slice::from_raw_parts_mut(out_message77, 77) };
    dst.copy_from_slice(&msg);
    MfskFt8Status::Ok
}

/// Convert a 77-bit FT8 message into the 79-tone Gray-mapped
/// sequence. Wraps `mfsk_core::ft8::wave_gen::message_to_tones`.
/// LDPC encode + CRC-14 + Costas insertion all happen inside.
///
/// `message77` must point to 77 valid bytes (each 0 or 1).
/// `out_itone` receives 79 bytes (each 0..7).
///
/// # Safety
/// Both pointers must be non-null and writable for their lengths.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_message_to_tones(
    message77: *const u8,
    out_itone: *mut u8,
) -> MfskFt8Status {
    if message77.is_null() || out_itone.is_null() {
        return MfskFt8Status::NullPointer;
    }
    let msg_slice = unsafe { core::slice::from_raw_parts(message77, 77) };
    let mut msg77 = [0u8; 77];
    msg77.copy_from_slice(msg_slice);
    let itone = message_to_tones(&msg77);
    let dst = unsafe { core::slice::from_raw_parts_mut(out_itone, 79) };
    dst.copy_from_slice(&itone);
    MfskFt8Status::Ok
}

/// Synthesise FT8 i16 PCM (12 kHz mono) from a 79-tone sequence
/// into a caller-provided buffer. `out` must be at least
/// [`mfsk_ft8_synth_output_len`] = 151 680 samples; longer is fine
/// (only the prefix is written). `f0_hz` is the carrier frequency
/// (typical 1500 Hz). `amplitude_i16` peaks at the given value
/// (typical 16 384 ≈ i16_max / 2 for ~50 % full scale headroom).
///
/// Output covers 12.64 s of audio (79 symbols × 1920 samples /
/// 12 kHz). Caller is responsible for slot-aligning this within an
/// FT8 14-second window if transmitting (typical: prepend 0.5 s of
/// silence + append 0.86 s of trailing silence to match the
/// receive-side `TX_START_OFFSET_S`).
///
/// # Safety
/// `itone` must point to 79 valid bytes (each 0..7). `out` must
/// point to a writable buffer of at least `out_len` i16 samples.
/// Returns `ScratchTooSmall` if `out_len < TONES_OUTPUT_LEN`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_tones_to_i16(
    itone: *const u8,
    f0_hz: f32,
    amplitude_i16: i16,
    out: *mut i16,
    out_len: usize,
) -> MfskFt8Status {
    if itone.is_null() || out.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if out_len < TONES_OUTPUT_LEN {
        return MfskFt8Status::ScratchTooSmall;
    }
    let itone_slice = unsafe { core::slice::from_raw_parts(itone, 79) };
    let mut itone_arr = [0u8; 79];
    itone_arr.copy_from_slice(itone_slice);
    let out_slice = unsafe { core::slice::from_raw_parts_mut(out, TONES_OUTPUT_LEN) };
    tones_to_i16_into(out_slice, &itone_arr, f0_hz, amplitude_i16);
    MfskFt8Status::Ok
}

/// f32 variant of [`mfsk_ft8_tones_to_i16`]. `amplitude` is unitless
/// (typical 0.5 for ~−6 dBFS). Same buffer-length rules.
///
/// # Safety
/// Same as [`mfsk_ft8_tones_to_i16`] — `itone` must point to 79
/// valid bytes, `out` must point to a writable f32 buffer of at
/// least `out_len` samples; returns `ScratchTooSmall` if `out_len`
/// is below `mfsk_ft8_synth_output_len()`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_tones_to_f32(
    itone: *const u8,
    f0_hz: f32,
    amplitude: f32,
    out: *mut f32,
    out_len: usize,
) -> MfskFt8Status {
    if itone.is_null() || out.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if out_len < TONES_OUTPUT_LEN {
        return MfskFt8Status::ScratchTooSmall;
    }
    let itone_slice = unsafe { core::slice::from_raw_parts(itone, 79) };
    let mut itone_arr = [0u8; 79];
    itone_arr.copy_from_slice(itone_slice);
    let out_slice = unsafe { core::slice::from_raw_parts_mut(out, TONES_OUTPUT_LEN) };
    tones_to_f32_into(out_slice, &itone_arr, f0_hz, amplitude);
    MfskFt8Status::Ok
}

// ── Streaming wrapper: I2S / USB-Audio chunk capture → 12 kHz ring ─────────
//
// Real-time receivers feed audio in small DMA chunks at whatever
// rate their codec runs (typically 16/24/48 kHz from I2S, 48 kHz
// from USB Audio Class). The decoder, on the other hand, operates
// on a 15-second 12 kHz slot. Bridging those two sides used to be
// every consumer's homework — `MfskFt8Stream` packages the standard
// pieces:
//
//  1. A streaming linear resampler from `src_rate_hz` to 12 kHz that
//     carries interpolation state across calls (no chunk-boundary
//     glitches).
//  2. A fixed-cap ring buffer at 12 kHz (oldest samples overwritten
//     when full — a "rolling N-second window" model that matches
//     the slot-based decoder's needs).
//
// Decoding itself is *not* bundled. The capture and decode tasks
// typically run on separate cores / RTOS tasks; the caller takes a
// snapshot via `_peek_latest` into their own scratch and hands that
// to `mfsk_ft8_decode_i16`. After decoding the slot they `_drain`
// the consumed prefix to make room for new audio.
//
// The wrapper is available under both `host` and `embedded-fixed-point`
// — the streaming primitives are pure-arithmetic, no FFT / no DSP
// backend, so they don't need a target-specific kernel.

use mfsk_core::engine::dsp::resample::LinearResamplerI16To12k;

/// Opaque handle returned by [`mfsk_ft8_stream_new`].
///
/// Owns one resampler + one 12 kHz ring buffer. Single-threaded —
/// callers that capture and decode on different tasks should put the
/// stream on the capture side and copy out via `_peek_latest` for the
/// decoder.
pub struct MfskFt8Stream {
    /// `None` when `src_rate == 12_000` (resampler bypassed).
    resampler: Option<LinearResamplerI16To12k>,
    /// 12 kHz sample ring. Length is the configured capacity.
    ring: Box<[i16]>,
    /// Index of the oldest sample currently in the ring.
    /// `(head + len) % capacity` is the next write slot.
    head: usize,
    /// Number of valid samples in the ring (≤ `capacity`).
    len: usize,
}

/// Append `src` to the ring at 12 kHz; overwrite oldest if full.
/// Free function (rather than a method on `MfskFt8Stream`) so the
/// caller can hold a `&mut` borrow on `resampler` simultaneously.
fn ring_push(ring: &mut [i16], head: &mut usize, len: &mut usize, src: &[i16]) {
    let cap = ring.len();
    for &s in src {
        let write_idx = (*head + *len) % cap;
        ring[write_idx] = s;
        if *len == cap {
            *head = (*head + 1) % cap;
        } else {
            *len += 1;
        }
    }
}

/// Allocate a new streaming wrapper.
///
/// `src_rate_hz`: the rate at which the caller will push samples
/// (typically 16000 / 24000 / 48000). Resampled internally to 12 kHz.
/// Must be > 0; pass `12000` to bypass the resampler.
///
/// `capacity_samples`: 12 kHz ring-buffer capacity. Pass `180_000`
/// for the standard 15 s FT8 slot. Smaller values save memory at the
/// cost of less history; larger values let the decoder lag without
/// overwriting. Must be ≥ 168_000 (the `decode_block` minimum) for
/// the typical "snapshot the latest 15 s and decode" pattern.
///
/// Returns `NULL` on `src_rate_hz == 0`, `capacity_samples == 0`, or
/// allocation failure.
///
/// # Safety
/// Free with [`mfsk_ft8_stream_free`].
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_ft8_stream_new(
    src_rate_hz: u32,
    capacity_samples: usize,
) -> *mut MfskFt8Stream {
    if src_rate_hz == 0 || capacity_samples == 0 {
        return core::ptr::null_mut();
    }
    let resampler = if src_rate_hz == 12_000 {
        None
    } else {
        Some(LinearResamplerI16To12k::new(src_rate_hz))
    };
    let ring: Vec<i16> = alloc::vec![0i16; capacity_samples];
    let stream = Box::new(MfskFt8Stream {
        resampler,
        ring: ring.into_boxed_slice(),
        head: 0,
        len: 0,
    });
    Box::into_raw(stream)
}

/// Free a stream allocated by [`mfsk_ft8_stream_new`]. Pointer must
/// not be used afterwards. `NULL` is safe (no-op).
///
/// # Safety
/// `stream` must be a pointer previously returned by
/// [`mfsk_ft8_stream_new`] and not yet freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_stream_free(stream: *mut MfskFt8Stream) {
    if !stream.is_null() {
        drop(unsafe { Box::from_raw(stream) });
    }
}

/// Push `n` source-rate i16 samples into the stream. Resamples to
/// 12 kHz internally and appends to the ring (oldest samples
/// overwritten if the ring is full).
///
/// Returns [`MfskFt8Status::Ok`] on success, [`MfskFt8Status::NullPointer`]
/// if `stream` or `samples` is null (or `n == 0` with non-null
/// samples is allowed and is a no-op).
///
/// # Safety
/// `samples` must point to `n` valid `i16` values; `stream` must be a
/// live handle from [`mfsk_ft8_stream_new`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_stream_push_i16(
    stream: *mut MfskFt8Stream,
    samples: *const i16,
    n: usize,
) -> MfskFt8Status {
    if stream.is_null() {
        return MfskFt8Status::NullPointer;
    }
    if n == 0 {
        return MfskFt8Status::Ok;
    }
    if samples.is_null() {
        return MfskFt8Status::NullPointer;
    }
    let stream = unsafe { &mut *stream };
    let src = unsafe { core::slice::from_raw_parts(samples, n) };

    // Destructure so the resampler and ring borrows don't overlap.
    let MfskFt8Stream {
        resampler,
        ring,
        head,
        len,
    } = stream;

    match resampler.as_mut() {
        // 12 kHz pass-through: append directly.
        None => ring_push(ring, head, len, src),
        // Resample chunk-wise into a small stack scratch so we never
        // need a heap alloc proportional to `n`.
        Some(r) => {
            let mut scratch = [0i16; 256];
            let mut src_pos = 0;
            while src_pos < src.len() {
                let (consumed, produced) = r.process(&src[src_pos..], &mut scratch);
                if consumed == 0 && produced == 0 {
                    break;
                }
                ring_push(ring, head, len, &scratch[..produced]);
                src_pos += consumed;
            }
        }
    }
    MfskFt8Status::Ok
}

/// Number of 12 kHz samples currently buffered.
///
/// # Safety
/// `stream` must be a live handle from [`mfsk_ft8_stream_new`].
/// Returns 0 if `stream` is null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_stream_buffered_samples(stream: *const MfskFt8Stream) -> usize {
    if stream.is_null() {
        return 0;
    }
    unsafe { (*stream).len }
}

/// Copy the most recent `cap` 12 kHz samples (in chronological order)
/// into `out`. Returns the number actually written —
/// `min(cap, buffered_samples)`. Does not modify the ring.
///
/// Pass `cap = 180000` and `out`-buffer of the same size to grab a
/// standard 15 s FT8 slot for `mfsk_ft8_decode_i16`.
///
/// # Safety
/// `stream` must be a live handle; `out` must point to `cap` writable
/// `i16`s. Returns 0 if either is null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_stream_peek_latest(
    stream: *const MfskFt8Stream,
    out: *mut i16,
    cap: usize,
) -> usize {
    if stream.is_null() || out.is_null() || cap == 0 {
        return 0;
    }
    let s = unsafe { &*stream };
    let n = cap.min(s.len);
    if n == 0 {
        return 0;
    }
    // The most recent `n` samples end at logical index `s.len - 1`,
    // i.e. ring index `(s.head + s.len - 1) % capacity`. Their start
    // is at logical index `s.len - n` → ring index
    // `(s.head + s.len - n) % capacity`.
    let cap_ring = s.ring.len();
    let start = (s.head + s.len - n) % cap_ring;
    let dst = unsafe { core::slice::from_raw_parts_mut(out, n) };
    if start + n <= cap_ring {
        dst.copy_from_slice(&s.ring[start..start + n]);
    } else {
        let first = cap_ring - start;
        dst[..first].copy_from_slice(&s.ring[start..]);
        dst[first..].copy_from_slice(&s.ring[..n - first]);
    }
    n
}

/// Drop the oldest `n` 12 kHz samples (advance the ring tail).
/// Use after a successful decode to free room for the next slot's
/// fresh audio. Clamped to `buffered_samples`.
///
/// # Safety
/// `stream` must be a live handle. `NULL` is a no-op.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_stream_drain(stream: *mut MfskFt8Stream, n: usize) {
    if stream.is_null() {
        return;
    }
    let s = unsafe { &mut *stream };
    let drop_n = n.min(s.len);
    let cap = s.ring.len();
    s.head = (s.head + drop_n) % cap;
    s.len -= drop_n;
}

/// Discard everything in the ring buffer and reset the resampler
/// state. Use on tuning changes / band switches.
///
/// # Safety
/// `stream` must be a live handle. `NULL` is a no-op.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_ft8_stream_clear(stream: *mut MfskFt8Stream) {
    if stream.is_null() {
        return;
    }
    let s = unsafe { &mut *stream };
    s.head = 0;
    s.len = 0;
    if let Some(r) = s.resampler.as_ref() {
        // Recreate to reset phase / primed.
        s.resampler = Some(LinearResamplerI16To12k::new(r.src_rate()));
    }
}

// ── Embedded runtime (no_std + staticlib needs these) ──────────────────────
//
// `cdylib`/`staticlib` Rust artifacts under `no_std` require a
// `#[panic_handler]` and a `#[global_allocator]`. The linking C
// project is expected to provide standard `malloc`/`free`/`abort`
// (esp-idf, newlib, picolibc all do). These hooks turn those into
// the Rust runtime symbols Cargo expects.
//
// **Override:** if the same final image already supplies a Rust
// panic handler / global allocator (e.g. a separate Rust shim crate
// that depends on mfsk-ffi-ft8), build mfsk-ffi-ft8 without the
// `embedded-runtime` feature to disable these defaults.
#[cfg(all(not(feature = "host"), feature = "embedded-runtime"))]
mod embedded_runtime {
    use core::alloc::{GlobalAlloc, Layout};
    use core::panic::PanicInfo;

    struct LibcAllocator;
    unsafe impl GlobalAlloc for LibcAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            unsafe extern "C" {
                fn malloc(size: usize) -> *mut u8;
            }
            unsafe { malloc(layout.size()) }
        }
        unsafe fn dealloc(&self, ptr: *mut u8, _: Layout) {
            unsafe extern "C" {
                fn free(ptr: *mut u8);
            }
            unsafe { free(ptr) }
        }
    }

    #[global_allocator]
    static ALLOC: LibcAllocator = LibcAllocator;

    #[panic_handler]
    fn panic(_: &PanicInfo) -> ! {
        unsafe extern "C" {
            fn abort() -> !;
        }
        unsafe { abort() }
    }
}
