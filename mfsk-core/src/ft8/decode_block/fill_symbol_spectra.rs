//! Per-symbol direct DFT — the symbol-spectrum fill stage.
//!
//! Bypasses the wide-band FFT cache by computing the 79 × 8 complex
//! tone spectra for one candidate via direct DFT at the exact tone
//! frequencies (host f32 path) or via the i16 basis-product
//! `cd0_to_symbol_spectra` kernel (embedded fixed-point path). The
//! `SymMask` enum drives which subset of symbols gets filled —
//! `SyncOnly` for Pass-1 candidate gating, `DataOnly` for Pass-2
//! LLR, `All` for the legacy single-call path.
//!
//! ε.4 of the `docs/CLEANUP_2026_05.md` `decode_block` split. The
//! parent (`decode_block.rs`) re-exports `SymMask`,
//! `symbol_spectra_direct`, the `fill_symbol_spectra*` family, and
//! the embedded-only `BASIS_SCRATCH_LEN` / `symbol_spectra_direct_into`
//! so external callers (`super::decode::*` host path, `mfsk-ffi-ft8`
//! embedded entry, `embedded-shared::stage1_inc`) keep the same
//! `mfsk_core::ft8::decode_block::*` paths as before.

use alloc::boxed::Box;
use alloc::vec;
#[cfg(feature = "fixed-point")]
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::params::{COSTAS, COSTAS_POS, NN, NSPS, NTONES};
use super::types::{AudioSample, SAMPLE_RATE_HZ, TONE_SPACING_HZ, TX_START_OFFSET_S};
use crate::core::scalar::Cmplx;

// Thread-local scratch for the `S -> i16` conversion in
// `fill_symbol_spectra_via_cd0`'s `fft_cache=None` branch. The
// `Vec<i16>` capacity is grown once per thread and reused across
// every per-candidate call — replaces the previous per-call
// `collect::<Vec<i16>>()` that allocated ~360 KB × 30 cand × 3 pass
// = ~32 MB of allocator traffic per slot (Gemini PR #80 review).
// Only the `fft-rustfft` path uses this (embedded `fft-extern` path
// doesn't build `fill_symbol_spectra_via_cd0` at all), so the
// `thread_local!` requirement on `std` is satisfied.
#[cfg(feature = "fft-rustfft")]
std::thread_local! {
    static AUDIO_I16_SCRATCH: core::cell::RefCell<alloc::vec::Vec<i16>> =
        const { core::cell::RefCell::new(alloc::vec::Vec::new()) };
}

// ── Per-symbol direct DFT (no FFT cache) ────────────────────────────────────

/// Compute the 79 × 8 complex tone spectra for one candidate by
/// direct DFT at the exact tone frequencies. Bypasses the wide-band
/// FFT cache entirely.
///
/// **Phase-rotator recursion.** Naïve per-sample `cos/sin` would be
/// ~25 M libm calls per `decode_block` invocation (8 candidates × 5
/// dt offsets × 79 symbols × 8 tones × 1920 samples) — minutes on
/// LX6. We replace it with one cos/sin pair per (symbol, tone) and
/// a single complex multiply per sample.
///
/// **PSRAM-aware access pattern.** The audio buffer (360 KB) lives
/// in PSRAM on Core2 (40 MHz quad, ~5× slower than internal RAM).
/// A naïve "for tone × for sample" loop would re-read each audio
/// sample 8 times across PSRAM. Instead we copy each 1920-sample
/// symbol into a stack-local f32 buffer once, then run all 8 tone
/// integrations over that internal-RAM copy. Reduces audio reads
/// from PSRAM by 8× — the dominant cost on LX6.
///
/// Numerical error: each rotation is a unit-magnitude multiply with
/// f32 round-off ≈ 6e-8; over 1920 samples the cumulative magnitude
/// drift stays below 0.012 % — negligible for LLR computation.
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn symbol_spectra_direct<S: AudioSample>(
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    sym_mask: SymMask,
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra(&mut out, audio, freq_hz, dt_sec, sym_mask, None);
    out
}

/// Which subset of the 79 symbols to compute. Used for the
/// Costas-first early-reject in `process_candidates`: the first
/// pass fills only Costas tone positions (21 symbols, 27 % of
/// full DFT cost) for the `sync_quality` gate; only on a hit do
/// we go back and fill the data-symbol positions.
///
/// **Pub for benchmarking only.**
#[doc(hidden)]
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum SymMask {
    /// Costas symbols only — all three blocks (positions 0-6, 36-42,
    /// 72-78). 21 symbols. Used for full-precision sync_quality
    /// gating in stage 3.
    SyncOnly,
    /// Costas block 0 only (positions 0-6). 7 symbols — 1/3 the cost
    /// of `SyncOnly`. Used for Pass 2 sync_quality re-rank where the
    /// finer ranking precision of all three blocks is unnecessary.
    SyncBlock0,
    /// Everything except Costas block 0 — fills positions 7-78
    /// (data symbols + Costas blocks 1, 2). 72 symbols. Used in
    /// stage 3 to "top up" a `SyncBlock0`-filled spectrum.
    NotBlock0,
    /// Data symbols only (positions 7-35, 43-71). Skips the 21 sync
    /// positions — used to "top up" a `SyncOnly`-filled spectrum.
    DataOnly,
    /// Costas blocks 1 and 2 only (positions 36-42, 72-78). 14
    /// symbols — 2/3 the cost of `SyncOnly`. Used in stage 3 to top
    /// up a `SyncBlock0`-filled cs (Pass 2 output) into a full
    /// `SyncOnly`-equivalent without redoing block 0.
    SyncBlocks12,
}

#[inline]
fn sym_in_mask(sym: usize, mask: SymMask) -> bool {
    let (in_block_a, in_block_b, in_block_c) = (
        sym < COSTAS.len(),                                         // 0..7
        sym >= COSTAS_POS[1] && sym < COSTAS_POS[1] + COSTAS.len(), // 36..43
        sym >= COSTAS_POS[2] && sym < COSTAS_POS[2] + COSTAS.len(), // 72..79
    );
    let is_sync = in_block_a || in_block_b || in_block_c;
    match mask {
        SymMask::SyncOnly => is_sync,
        SymMask::SyncBlock0 => in_block_a,
        SymMask::NotBlock0 => !in_block_a,
        SymMask::DataOnly => !is_sync,
        SymMask::SyncBlocks12 => in_block_b || in_block_c,
    }
}

/// **Pub for benchmarking only — do not depend on it.**
///
/// f32 wrapper. **WSJT-X-faithful** when `fft-rustfft` is enabled:
/// routes through the `ft8_downsample` chain (192k FFT → tapered LPF
/// → 200 sps cd0) + per-symbol 32-pt FFT, matching
/// `lib/ft8/ft8b.f90:154-161` exactly. Out-of-band signals (broadband
/// birdies, sidelobes) are suppressed by the downsample's
/// edge-tapered filter, instead of leaking into per-tone DFT
/// sidelobes as they would in a rectangular-window per-tone DFT.
/// Used by both host f32 and host fixed-point builds.
#[doc(hidden)]
#[cfg(feature = "fft-rustfft")]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    fill_symbol_spectra_via_cd0(out, audio, freq_hz, dt_sec, mask, fft_cache);
}

/// Embedded fallback (no `fft-rustfft` available — Xtensa cannot run
/// the 192k cd0 FFT). Reverts to the rectangular-window per-tone DFT
/// for non-fixed-point builds; the fixed-point variant has its own
/// basis-precompute path in `fill_symbol_spectra_into`.
///
/// The `fft_cache` parameter is accepted for API parity with the
/// `fft-rustfft` variant but ignored — there is no 192k FFT to skip
/// on this path.
#[doc(hidden)]
#[cfg(all(not(feature = "fft-rustfft"), not(feature = "fixed-point")))]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    let _ = fft_cache;
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// WSJT-X cd0-based per-symbol FFT. Mirrors `ft8b.f90:154-161`:
/// ```fortran
/// call ft8_downsample(dd, newdat, f1, cd0)
/// do k=1,NN
///   i1 = ibest + (k-1)*32
///   csymb = cd0(i1:i1+31)
///   call four2a(csymb, 32, 1, -1, 1)   ! 32-pt FFT
///   cs(0:7,k) = csymb(1:8) / 1e3
/// enddo
/// ```
/// Per-call cost: 79 × 32-pt FFT + (one 192k forward FFT only if the
/// caller did not supply `fft_cache`) + one 3.2k inverse FFT.
/// Passing `Some(cache)` from the multipass driver (the cache built
/// once per slot in `decode_frame_inner`) skips the 192k forward FFT
/// per candidate — ~5 ms saved × 30+ candidates × 3 passes.
#[cfg(feature = "fft-rustfft")]
fn fill_symbol_spectra_via_cd0<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    use rustfft::FftPlanner;
    extern crate alloc;

    // S → i16 conversion (no-op when S=i16 already). Hoisted into a
    // thread-local Vec so we don't re-allocate ~360 KB per candidate
    // (the cand loop calls this ~30 cand × 3 pass = 90 times per slot
    // in the multipass driver). Skipped entirely when `fft_cache` is
    // supplied (the slot-cache path doesn't touch the audio bytes —
    // only the precomputed forward FFT). Gemini PR #80 review.
    let cd0 = match fft_cache {
        Some(cache) => crate::core::dsp::downsample::downsample_cached(
            cache,
            freq_hz,
            &crate::ft8::downsample::FT8_CFG,
        ),
        None => AUDIO_I16_SCRATCH.with_borrow_mut(|buf| {
            buf.clear();
            buf.reserve(audio.len());
            buf.extend(audio.iter().map(|s| s.to_i16()));
            crate::ft8::downsample::downsample(buf.as_slice(), freq_hz, None).0
        }),
    };

    // ibest in cd0 sample units (200 sps). dt_sec is offset from
    // TX_START_OFFSET_S = 0.5 s; cd0[0] corresponds to slot t=0,
    // so the first symbol starts at sample (0.5 + dt) × 200.
    let ibest = ((TX_START_OFFSET_S + dt_sec) * 200.0).round() as i32;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(32);
    let mut buf = [Complex::new(0.0_f32, 0.0); 32];

    // WSJT-X scales `cs = csymb / 1e3` (ft8b.f90:159). The /1e3 is
    // absorbed by `normalize_bmet` in the LLR pipeline, but we keep
    // it for traceable parity with WSJT-X numerics.
    const CS_SCALE: f32 = 1.0 / 1000.0;

    let np2 = cd0.len() as i32;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let i1 = ibest + (sym as i32) * 32;
        // WSJT-X ft8b.f90:155-157 — *all-or-nothing* boundary check:
        //   csymb = cmplx(0.0, 0.0)
        //   if( i1.ge.0 .and. i1+31 .le. NP2-1 ) csymb = cd0(i1:i1+31)
        // i.e. when ANY of the 32 samples falls outside cd0, the whole
        // window is set to zero. Per-element fill (= use partial cd0
        // data) was an incorrect simplification that pulled extra
        // signal energy into edge symbols and shifted the LLR sign
        // pattern away from WSJT-X's.
        if i1 >= 0 && i1 + 31 < np2 {
            for j in 0..32 {
                buf[j] = cd0[(i1 + j as i32) as usize];
            }
        } else {
            for j in 0..32 {
                buf[j] = Complex::new(0.0, 0.0);
            }
        }
        fft.process(&mut buf);
        // csymb(1:8) (Fortran) = bins 0..7 (0-based) = tones 0..7.
        for tone in 0..NTONES {
            out[sym][tone] = Cmplx {
                re: buf[tone].re * CS_SCALE,
                im: buf[tone].im * CS_SCALE,
            };
        }
    }
}

/// Generic per-symbol DFT — writes `Cmplx<Sc>` for any spec scalar
/// `Sc: SpecScalar`. For `Sc = f32` (`NEEDS_AUTOGAIN = false`) the
/// inner loop writes f32 components directly — byte-identical to the
/// pre-Phase-2.6 implementation. For fixed-point `Sc` (`NEEDS_AUTOGAIN
/// = true`) the function runs a 2-pass scan-and-scale: compute all
/// 79 × 8 Complex<f32> entries into a stack tmp buffer (~5 KB), find
/// the peak |re|/|im| across the active mask, then write
/// `Sc::from_f32_scaled(value, scale)` with `scale = i16::MAX × 0.95
/// / peak` so the i16 range is fully utilised without saturation.
#[doc(hidden)]
#[cfg(not(feature = "fixed-point"))]
pub fn fill_symbol_spectra_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;

    let mut rotators = [Complex::new(0.0f32, 0.0); NTONES];
    for tone in 0..NTONES {
        let tone_freq = freq_hz + tone as f32 * TONE_SPACING_HZ;
        let dphi = -two_pi_over_fs * tone_freq;
        rotators[tone] = Complex::new(dphi.cos(), dphi.sin());
    }

    let mut sym_buf = [0.0f32; NSPS];

    if !Sc::NEEDS_AUTOGAIN {
        // Sc = f32: inline write via `Sc::from_f32` (no-op for f32).
        // Const dispatch — LLVM eliminates the `else` branch when
        // monomorphised for `Sc = f32`.
        for sym in 0..NN {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            let sym_start = i0 + (sym as i64) * (NSPS as i64);
            for k in 0..NSPS {
                let idx = sym_start + k as i64;
                sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                    audio[idx as usize].to_f32()
                } else {
                    0.0
                };
            }
            for tone in 0..NTONES {
                let rotator = rotators[tone];
                let mut osc = Complex::new(1.0f32, 0.0);
                let mut acc = Complex::new(0.0f32, 0.0);
                for &s in sym_buf.iter() {
                    acc.re += s * osc.re;
                    acc.im += s * osc.im;
                    osc *= rotator;
                }
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32(acc.re),
                    im: Sc::from_f32(acc.im),
                };
            }
        }
        return;
    }

    // Fixed-point path: 2-pass with auto-gain.
    let mut tmp = [[Complex::new(0.0f32, 0.0); 8]; 79];
    let mut peak: f32 = 0.0;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        for k in 0..NSPS {
            let idx = sym_start + k as i64;
            sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                audio[idx as usize].to_f32()
            } else {
                0.0
            };
        }
        for tone in 0..NTONES {
            let rotator = rotators[tone];
            let mut osc = Complex::new(1.0f32, 0.0);
            let mut acc = Complex::new(0.0f32, 0.0);
            for &s in sym_buf.iter() {
                acc.re += s * osc.re;
                acc.im += s * osc.im;
                osc *= rotator;
            }
            tmp[sym][tone] = acc;
            peak = peak.max(acc.re.abs()).max(acc.im.abs());
        }
    }
    let scale = if peak > 1e-9 {
        (i16::MAX as f32 * 0.95) / peak
    } else {
        0.0
    };
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        for tone in 0..NTONES {
            let c = tmp[sym][tone];
            out[sym][tone] = Cmplx {
                re: Sc::from_f32_scaled(c.re, scale),
                im: Sc::from_f32_scaled(c.im, scale),
            };
        }
    }
}

/// Required scratch length for [`fill_symbol_spectra_into`] — one
/// flat array per axis (cos / sin), `NTONES × NSPS = 15 360` i16.
/// Caller must provide two slices of at least this length.
#[cfg(feature = "fixed-point")]
pub const BASIS_SCRATCH_LEN: usize = NTONES * NSPS;

/// Fixed-point per-symbol DFT — basis-precompute + dot-product
/// kernel. Drop-in heap-allocating wrapper around
/// [`fill_symbol_spectra_into`]: allocates 60 KB × 2 of basis scratch
/// from the default heap on every call. Convenient for host use; on
/// embedded targets the scratch typically lands in PSRAM (slow reads
/// in the dot-product inner loop), so callers that care about Core2
/// throughput should pre-allocate scratch in **internal RAM**
/// (`static [i16; BASIS_SCRATCH_LEN]` in `.bss`, or
/// `heap_caps_malloc(MALLOC_CAP_INTERNAL)`) and call
/// [`fill_symbol_spectra_into`] directly.
// Fixed-point host with `fft-rustfft` uses the cd0-based
// `fill_symbol_spectra` defined above. Pure embedded fixed-point
// (no fft-rustfft) goes through `fill_symbol_spectra_into` instead
// — this short heap-allocating wrapper is only kept for the
// `(fixed-point, !fft-rustfft)` build, which is the only one that
// would have called this entry.
#[doc(hidden)]
#[cfg(all(feature = "fixed-point", not(feature = "fft-rustfft")))]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    let _ = fft_cache;
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// Generic fixed-point fill — writes `Cmplx<Sc>`. f32 wrapper above.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let mut basis_re: Vec<i16> = alloc::vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im: Vec<i16> = alloc::vec![0i16; BASIS_SCRATCH_LEN];
    fill_symbol_spectra_into_generic::<Sc, S>(
        out,
        audio,
        freq_hz,
        dt_sec,
        mask,
        &mut basis_re,
        &mut basis_im,
    );
}

/// Fixed-point per-symbol DFT with caller-provided basis scratch.
///
/// Two phases per call:
/// 1. **Basis precompute** (in `basis_re` / `basis_im`) — 8 tones ×
///    {cos, sin} = 16 vectors of NSPS=1920 i16 samples, generated by
///    a Q15 rotator (one cos+sin pair per tone, then 1920 complex
///    multiplies to fill the vector).
/// 2. **Per-symbol dot products** — for each symbol in `mask`,
///    16 calls to [`crate::core::dotprod::dot_q15_i32`] against the
///    basis. Default is a Rust loop; embedded targets can override
///    via `mfsk_core_dot_q15_i32` to bridge to chip-native asm
///    (e.g. esp-dsp `dsps_dotprod_s16_ae32` on Xtensa LX6).
///
/// **Why caller-provided scratch?** On Core2 the basis is the inner
/// loop's hot data — esp-dsp's asm dot product runs at 1 cycle/sample
/// only when the basis lives in fast internal RAM. Default heap on
/// ESP32 with `CONFIG_SPIRAM_USE_MALLOC` puts a 60 KB allocation in
/// PSRAM (~5–10 cycles/sample read latency), which kills the asm
/// kernel's advantage. Pre-allocating scratch in `.bss` (static
/// arrays land in internal DRAM) lets the dot product reach its
/// theoretical speed.
///
/// Both `basis_re` and `basis_im` must be at least
/// [`BASIS_SCRATCH_LEN`] long — debug-asserted; longer is fine
/// (only the prefix is used).
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_into<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) {
    fill_symbol_spectra_into_generic::<f32, S>(
        out, audio, freq_hz, dt_sec, mask, basis_re, basis_im,
    );
}

/// Generic version of [`fill_symbol_spectra_into`] — writes
/// `Cmplx<Sc>` for any spec scalar. f32 wrapper above.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_into_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) {
    use crate::core::dotprod::dot_q15_i32;
    debug_assert!(basis_re.len() >= BASIS_SCRATCH_LEN);
    debug_assert!(basis_im.len() >= BASIS_SCRATCH_LEN);
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;

    // ── Phase 1: precompute Q15 basis vectors (cos, sin × 8 tones).
    for tone in 0..NTONES {
        let tone_freq = freq_hz + tone as f32 * TONE_SPACING_HZ;
        let dphi = -two_pi_over_fs * tone_freq;
        let rot_re = (dphi.cos() * 32767.0).round() as i32;
        let rot_im = (dphi.sin() * 32767.0).round() as i32;
        let mut osc_re: i32 = 32767;
        let mut osc_im: i32 = 0;
        let base = tone * NSPS;
        for k in 0..NSPS {
            basis_re[base + k] = osc_re as i16;
            basis_im[base + k] = osc_im as i16;
            let new_re = ((osc_re * rot_re) - (osc_im * rot_im)) >> 15;
            let new_im = ((osc_re * rot_im) + (osc_im * rot_re)) >> 15;
            osc_re = new_re;
            osc_im = new_im;
        }
    }

    // Stack buffer: one symbol of audio as i16. 1920 × 2 = 3.8 KB.
    let mut sym_buf = [0i16; NSPS];

    // ── Phase 2: per-symbol dot products (audio × basis).
    //
    // For `Sc = f32` (no autogain) we write each cell straight into
    // `out` via `Sc::from_f32` (no-op cast). For fixed-point types
    // we collect i32 accumulators into a stack tmp buffer
    // (~40 KB i32×8×79×2), find the peak, and write
    // `Sc::from_f32_scaled(acc as f32, scale)` so the i16 output
    // range is fully utilised. The scratch is a stack array — fits
    // the 32 KB Core2 main task stack with the existing 16 KB used
    // by basis scratch references and the spec.
    let mut tmp_re = [[0i32; 8]; 79];
    let mut tmp_im = [[0i32; 8]; 79];
    let mut peak: i32 = 0;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        for k in 0..NSPS {
            let idx = sym_start + k as i64;
            sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                audio[idx as usize].to_i16()
            } else {
                0
            };
        }
        for tone in 0..NTONES {
            let base = tone * NSPS;
            let basis_re_tone = &basis_re[base..base + NSPS];
            let basis_im_tone = &basis_im[base..base + NSPS];
            let acc_re = dot_q15_i32(&sym_buf, basis_re_tone);
            let acc_im = dot_q15_i32(&sym_buf, basis_im_tone);
            if !Sc::NEEDS_AUTOGAIN {
                // f32 fast path — direct write, identical to the
                // pre-2.6 cast.
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32(acc_re as f32),
                    im: Sc::from_f32(acc_im as f32),
                };
            } else {
                tmp_re[sym][tone] = acc_re;
                tmp_im[sym][tone] = acc_im;
                peak = peak.max(acc_re.unsigned_abs() as i32);
                peak = peak.max(acc_im.unsigned_abs() as i32);
            }
        }
    }

    // 2-pass auto-gain: scale i32 accumulators into i16 range,
    // saturating safe, peak ≈ 95 % of i16::MAX. Skipped on the f32
    // fast path (`out` is already populated above).
    if Sc::NEEDS_AUTOGAIN {
        let scale = if peak > 0 {
            (i16::MAX as f32 * 0.95) / peak as f32
        } else {
            0.0
        };
        for sym in 0..NN {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            for tone in 0..NTONES {
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32_scaled(tmp_re[sym][tone] as f32, scale),
                    im: Sc::from_f32_scaled(tmp_im[sym][tone] as f32, scale),
                };
            }
        }
    }
}

/// Heap-allocating sibling of [`symbol_spectra_direct`] that uses a
/// caller-provided basis scratch (passed through to
/// [`fill_symbol_spectra_into`]). Only the fixed-point variant is
/// exposed — host f32 path doesn't need a scratch (`fill_symbol_spectra`
/// f32 has no basis precompute step).
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn symbol_spectra_direct_into<S: AudioSample>(
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    sym_mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra_into(
        &mut out, audio, freq_hz, dt_sec, sym_mask, basis_re, basis_im,
    );
    out
}
