//! Per-symbol direct DFT — the symbol-spectrum fill stage.
//!
//! Bypasses the wide-band FFT cache by computing the 79 × 8 complex
//! tone spectra for one candidate via direct DFT at the exact tone
//! frequencies (host f32 path / generic rotator-based fill) or via
//! the Goertzel recursion (embedded path, zero scratch — see
//! [`fill_symbol_spectra_goertzel`]). The `SymMask` enum drives which
//! subset of symbols gets filled — `SyncOnly` for Pass-1 candidate
//! gating, `DataOnly` for Pass-2 LLR, `All` for the legacy
//! single-call path.
//!
//! ε.4 of the `docs/CLEANUP_2026_05.md` `decode_block` split. The
//! parent (`decode_block.rs`) re-exports `SymMask`,
//! `symbol_spectra_direct`, and the `fill_symbol_spectra*` family so
//! external callers (`super::decode::*` host path,
//! `embedded-shared::stage1_inc`) keep the same
//! `mfsk_core::ft8::decode_block::*` paths as before. The legacy
//! BASIS (Q15 sin/cos dot-product) fill path was removed in 0.8.0
//! (issue #162) — `fill_symbol_spectra_goertzel` (Phase
//! 1.7.7-Stick) has been the sole production path on every embedded
//! target since 0.6.4.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::params::{COSTAS, COSTAS_POS, NN, NSPS, NTONES};
use super::types::{AudioSample, Ft8Rate, SAMPLE_RATE_HZ, TONE_SPACING_HZ, TX_START_OFFSET_S};
use crate::engine::scalar::Cmplx;

/// Last sample index that
/// [`fill_symbol_spectra_goertzel`] reads for a candidate at
/// `dt_sec`. Same `i0 + NN * NSPS` formula as the per-symbol loop
/// inside the kernel, exposed here so embedded callers (Phase C
/// audio-tail speculation in `embedded-shared`) can decide whether
/// a candidate's Goertzel window fits inside the currently-captured
/// audio prefix without re-implementing the constants.
pub fn goertzel_window_end_sample(dt_sec: f32) -> usize {
    goertzel_window_end_sample_at(dt_sec, Ft8Rate::Hz12000)
}

/// [`goertzel_window_end_sample`] for audio at `rate` ([`Ft8Rate`]): the samples are the rate's, the symbol is `rate x 0.16`.
pub fn goertzel_window_end_sample_at(dt_sec: f32, rate: Ft8Rate) -> usize {
    let i0 = ((TX_START_OFFSET_S + dt_sec) * rate.hz() as f32).round() as i64;
    // a Hann window over two symbols reaches half a symbol past the last one
    let tail = if rate.symbol_hann_2t() {
        rate.nsps() as i64 / 2
    } else {
        0
    };
    (i0 + (NN as i64) * (rate.nsps() as i64) + tail).max(0) as usize
}

// Thread-local scratch for the `S -> i16` conversion in
// `fill_symbol_spectra_via_cd0`'s `fft_cache=None` branch. The
// `Vec<i16>` capacity is grown once per thread and reused across
// every per-candidate call — replaces the previous per-call
// `collect::<Vec<i16>>()` that allocated ~360 KB × 30 cand × 3 pass
// = ~32 MB of allocator traffic per slot (Gemini PR #80 / #100
// review). Gated on `fft-rustfft` only — matches the usage site's
// gate; `fft-rustfft` implies `std` via the Cargo.toml feature
// closure (`fft-rustfft = ["std", "dep:rustfft"]`).
#[cfg(feature = "fft-rustfft")]
std::thread_local! {
    static AUDIO_I16_SCRATCH: core::cell::RefCell<alloc::vec::Vec<i16>> =
        const { core::cell::RefCell::new(alloc::vec::Vec::new()) };
}

// 32-pt rustfft plan cached at first use. The size is constant
// (per-symbol DFT in `fill_symbol_spectra_via_cd0`), so building a
// fresh `FftPlanner` + `plan_fft_forward(32)` per candidate just
// re-allocates the same twiddle tables (~30 cand × 3 pass = 90 plans
// per slot). Gemini PR #80 review. `OnceLock` is `Sync` so the same
// `Arc<dyn Fft>` is shared across all decoder threads; rustfft
// `Fft::process` is thread-safe (no internal mutable state — it
// works on the caller's scratch buffer).
#[cfg(feature = "fft-rustfft")]
use std::sync::OnceLock;
#[cfg(feature = "fft-rustfft")]
static SYMBOL_FFT_32: OnceLock<alloc::sync::Arc<dyn rustfft::Fft<f32>>> = OnceLock::new();

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
    fft_cache: Option<&[Complex<f32>]>,
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra(&mut out, audio, freq_hz, dt_sec, sym_mask, fft_cache);
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
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
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
pub(super) fn sym_in_mask(sym: usize, mask: SymMask) -> bool {
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
/// `lib/ft8/ft8b.f90:155-162` exactly. Out-of-band signals (broadband
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
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::SymbolSpectra);
    fill_symbol_spectra_via_cd0(out, audio, freq_hz, dt_sec, mask, fft_cache);
}

/// Embedded fallback (no `fft-rustfft` available — Xtensa cannot run
/// the 192k cd0 FFT). Reverts to the rectangular-window per-tone DFT
/// via [`fill_symbol_spectra_generic`] — the same generic fill used
/// on host for non-`fft-rustfft` builds, fixed-point or not.
///
/// The `fft_cache` parameter is accepted for API parity with the
/// `fft-rustfft` variant but ignored — there is no 192k FFT to skip
/// on this path.
#[doc(hidden)]
#[cfg(not(feature = "fft-rustfft"))]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    fft_cache: Option<&[Complex<f32>]>,
) {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::SymbolSpectra);
    let _ = fft_cache;
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// WSJT-X cd0-based per-symbol FFT. Mirrors `ft8b.f90:155-162`:
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
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::SymbolSpectra);
    use rustfft::FftPlanner;
    extern crate alloc;

    // S → i16 conversion (no-op when S=i16 already). Hoisted into a
    // thread-local Vec so we don't re-allocate ~360 KB per candidate
    // (the cand loop calls this ~30 cand × 3 pass = 90 times per slot
    // in the multipass driver). Skipped entirely when `fft_cache` is
    // supplied (the slot-cache path doesn't touch the audio bytes —
    // only the precomputed forward FFT). Gemini PR #80 review.
    let cd0 = match fft_cache {
        Some(cache) => crate::engine::dsp::downsample::downsample_cached(
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

    // FFT planner + 32-pt plan are constant (size never changes
    // here); building them inside the candidate loop allocates fresh
    // twiddle tables per call. Cache once at first use — Gemini PR
    // #80 review. `Arc<dyn Fft>` is what `plan_fft_forward` returns;
    // each call here just clones the Arc (~10 ns refcount bump).
    let fft = SYMBOL_FFT_32
        .get_or_init(|| {
            let mut planner = FftPlanner::<f32>::new();
            planner.plan_fft_forward(32)
        })
        .clone();
    let mut buf = [Complex::new(0.0_f32, 0.0); 32];

    // WSJT-X scales `cs = csymb / 1e3` (ft8b.f90:159). A true division,
    // not `* (1/1000)`: gfortran -O2 and -O3 -funroll-loops (the release
    // flags) divide componentwise, and the two differ in the last bit for
    // 81% of 2M random samples (scratch oracle, 2026-10-09).
    const CS_DIV: f32 = 1000.0;

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
                re: buf[tone].re / CS_DIV,
                im: buf[tone].im / CS_DIV,
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
pub fn fill_symbol_spectra_generic<Sc: crate::engine::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::SymbolSpectra);
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

/// Per-symbol DFT via **generalised Goertzel** — the embedded
/// production fill (Phase 1.7.7-Stick) with **zero caller-provided
/// scratch**. Replaced the legacy BASIS (Q15 sin/cos dot-product)
/// fill path, which was removed in 0.8.0 (issue #162) once this had
/// been the sole production path on every embedded target for
/// several releases.
///
/// Same **magnitude** as `Σ x[n] exp(-jωn)` for each tone at
/// `ω = 2π·f/Fs` across NSPS=1920 samples (see "Phase convention"
/// below for the complex-value caveat), produced via a 2-tap IIR
/// recursion instead of a sin/cos dot-product. Needs no basis
/// scratch at all — the whole point of Phase 1.7.7-Stick.
///
/// Algorithm (per `(sym, tone)`):
/// ```text
/// ω      = 2π · tone_freq / Fs
/// coeff  = 2·cos(ω)
/// s[-2] = s[-1] = 0
/// for n in 0..NSPS:
///     s[n] = x[i0 + sym·NSPS + n] + coeff·s[n-1] - s[n-2]
/// X(ω)   = s[N-1] - exp(-jω)·s[N-2]
///        = (s_prev - cos(ω)·s_prev2) + j·sin(ω)·s_prev2
/// ```
/// Per-(sym, tone) state = 3 f32 values; per-call all-tones state =
/// `[f32; NTONES] × 2 = 64 B` on the stack.
///
/// **Loop order**: sample-outer / tone-inner. The 8 Goertzel
/// recursions for one symbol are independent (no cross-tone deps);
/// nesting tones inside the sample loop lets LLVM unroll the
/// `NTONES = 8` constant-bound inner and issue all 8 independent
/// dependent chains into the FPU pipeline at once. On Xtensa LX6/LX7
/// where `fmul` has a 3-4 cycle latency but 1 cycle throughput, this
/// hides the per-chain latency for a ~2× speedup over the
/// per-tone-outer layout. The sample fetch is also done once per
/// `n` instead of `NTONES × NSPS` times — 8× less audio-buffer
/// traffic, friendlier to the LX7 L1.
///
/// **Phase convention.** The generalised-Goertzel extraction does
/// not return the DFT bin itself but `X_goertzel = exp(jω(N-1)) ·
/// X_dft`. Measured against [`fill_symbol_spectra_generic`] (the
/// rotator-based direct DFT) on a tone-plus-noise buffer, at
/// `f0 = 1000 Hz` and `f0 = 1003 Hz`:
///
/// - `|X_goertzel / X_dft| = 1.0000` — max deviation `7.3e-5` across
///   every filled cell, i.e. the magnitudes are the same number.
/// - `arg(X_goertzel / X_dft)` is constant across symbols and tilts
///   by exactly `-ω` per tone step: `-0.1875°` per tone, `1.3°`
///   across all eight. `NSPS × TONE_SPACING_HZ / Fs = 1` exactly
///   (the MFSK orthogonality condition), so the `exp(jωN)` half of
///   the factor is common to every tone and only `exp(-jω_tone)`
///   survives as a tone-dependent term.
///
/// Consequence, by consumer: [`sync_quality`](crate::ft8::llr::sync_quality),
/// the `xsnr2` / [`snr_ratio`](crate::engine::llr::snr_ratio)
/// estimators, and the `nsym = 1` LLRs (llra / llrd) read magnitudes
/// only, so the rotation is exactly irrelevant to them — and
/// `DecodeDepth::EMBEDDED` (`LlrEffort::Minimal`) stops there, which
/// is why this is the ship configuration's whole story. The
/// `nsym ≥ 2` LLRs (llrb / llrc) do **not**: `engine::llr::
/// build_group_amplitudes` sums complex cells across a symbol group
/// and takes `|a + b|`, so the `1.3°` tilt across the tone axis is
/// carried into that coherent sum. It is far below the phase error
/// the channel itself contributes, and no golden test has ever moved
/// on it, but it is not zero — earlier notes (0.6.4's CHANGELOG
/// entry) that called the rotation irrelevant because "downstream
/// consumes `|cs|²`" were describing the `Minimal` path only.
///
/// **Numerics**: f32 internal arithmetic. Host validation on
/// qso3_busy.wav (pre-0.8.0, against the now-removed BASIS Q15 path)
/// showed recall 7/7 with per-station SNR +0.0..+0.6 dB (f32's extra
/// precision over Q15 helped the weakest decodes).
pub fn fill_symbol_spectra_goertzel<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    fill_symbol_spectra_goertzel_at(out, audio, Ft8Rate::Hz12000, freq_hz, dt_sec, mask);
}

/// [`fill_symbol_spectra_goertzel`] on audio at `rate` ([`Ft8Rate`]): a symbol is `rate x 0.16` samples (1280 at 8 kHz, 2/3 of
/// the work of 1920), the tone frequencies and the symbol grid are the same. The sums are scaled by `12000 / rate` (3/2 at
/// 8 kHz), so that a cell is the 12 kHz path's for the same signal and what reads it (the LLRs, `sync_quality`, the SNR) does
/// not know the rate. **A deliberate divergence from WSJT-X** (12 kHz only), as [`Ft8Rate`].
pub fn fill_symbol_spectra_goertzel_at<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    rate: Ft8Rate,
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let _stage = crate::ft8::stage_timing::scope(crate::ft8::stage_timing::Stage::SymbolSpectra);
    let fs = rate.hz() as f32;
    let nsps = rate.nsps();
    let two_pi_over_fs = core::f32::consts::TAU / fs;
    let i0 = ((TX_START_OFFSET_S + dt_sec) * fs).round() as i64;
    // 1.0 at 12 kHz: not applied, so that path is the one it was
    let to_12k = SAMPLE_RATE_HZ / fs;

    // Per-tone constants — 8 tones × 3 f32 = 96 B on the stack.
    let mut cos_w = [0.0_f32; NTONES];
    let mut sin_w = [0.0_f32; NTONES];
    let mut coeff = [0.0_f32; NTONES];
    for tone in 0..NTONES {
        let tone_freq = freq_hz + (tone as f32) * TONE_SPACING_HZ;
        let omega = two_pi_over_fs * tone_freq;
        cos_w[tone] = omega.cos();
        sin_w[tone] = omega.sin();
        coeff[tone] = 2.0 * cos_w[tone];
    }

    if rate.symbol_hann_2t() {
        fill_hann_2t(
            out,
            audio,
            nsps,
            i0,
            mask,
            &cos_w,
            &sin_w,
            &coeff,
            two_pi_over_fs,
            freq_hz,
            to_12k,
        );
        return;
    }

    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (nsps as i64);
        // 8 independent recursion states.
        let mut s_prev = [0.0_f32; NTONES];
        let mut s_prev2 = [0.0_f32; NTONES];

        // Hot path: entire symbol window lies within audio bounds.
        // The bounds check (idx >= 0 && idx < audio.len()) can then be
        // hoisted out of the 1920-iteration inner loop, leaving a clean
        // sequential load that LLVM's unroll + Xtensa FPU pipeline can
        // run at ~1 cycle/tone/sample. Cold path (first/last ~1 symbol
        // per candidate) keeps the original per-sample check.
        if sym_start >= 0 && (sym_start as usize) <= audio.len().saturating_sub(nsps) {
            let base = sym_start as usize;
            for n in 0..nsps {
                let sample = audio[base + n].to_i16() as f32;
                for tone in 0..NTONES {
                    let s = sample + coeff[tone] * s_prev[tone] - s_prev2[tone];
                    s_prev2[tone] = s_prev[tone];
                    s_prev[tone] = s;
                }
            }
        } else {
            let audio_len = audio.len() as i64;
            for n in 0..nsps {
                let idx = sym_start + n as i64;
                let sample = if idx >= 0 && idx < audio_len {
                    audio[idx as usize].to_i16() as f32
                } else {
                    0.0
                };
                for tone in 0..NTONES {
                    let s = sample + coeff[tone] * s_prev[tone] - s_prev2[tone];
                    s_prev2[tone] = s_prev[tone];
                    s_prev[tone] = s;
                }
            }
        }

        for tone in 0..NTONES {
            let re = s_prev[tone] - cos_w[tone] * s_prev2[tone];
            let im = sin_w[tone] * s_prev2[tone];
            out[sym][tone] = if rate == Ft8Rate::Hz12000 {
                Cmplx { re, im }
            } else {
                Cmplx {
                    re: re * to_12k,
                    im: im * to_12k,
                }
            };
        }
    }
}

/// `w[k] = (1 - cos(theta k)) / 2` for `k = 0, 1, ...` by rotating `(cos, sin)` by `theta` each step (`theta = 2 pi / len` makes
/// it the periodic Hann, `sin^2(pi k / len)`, over `len` samples).
///
/// The rotation and not the Chebyshev recurrence `c[k+1] = 2 cos(theta) c[k] - c[k-1]`: with `theta` this small
/// (`2 pi / 2560`) the recurrence's error grows with `k^2` in `f32` (2e-4 by `k` = 114 and worse after), the rotation's with `k`.
struct HannRecurrence {
    cos_t: f32,
    sin_t: f32,
    c: f32,
    s: f32,
}

impl HannRecurrence {
    fn new(theta: f32) -> Self {
        Self {
            cos_t: theta.cos(),
            sin_t: theta.sin(),
            c: 1.0,
            s: 0.0,
        }
    }

    fn next(&mut self) -> f32 {
        let w = 0.5 * (1.0 - self.c);
        let (c, s) = (
            self.c * self.cos_t - self.s * self.sin_t,
            self.s * self.cos_t + self.c * self.sin_t,
        );
        self.c = c;
        self.s = s;
        w
    }
}

/// The stage-3 symbol DFT with a **Hann window over two symbols** centred on the symbol (`Ft8Rate::symbol_hann_2t`): the
/// Goertzel recursion over `2 x nsps` samples from `nsps / 2` before the symbol's first, each weighted by
/// `w[k] = (1 - cos(2 pi k / (2 nsps))) / 2`.
///
/// The weight is made by a rotation of `(cos, sin)` (`HannRecurrence`: one `cos` and one `sin` per call, then multiplies and
/// adds) into a table of the window's first half, once per call (`nsps + 1` floats: 5 KB at 8 kHz) and read mirrored for the 79
/// symbols; computing it per sample instead costs 25 % more instructions. (One allocation per call: about 45 to 90 of 5 KB a slot
/// at 15 to 30 candidates; the table depends on the rate only and could be built once.) Only IEEE `f32` arithmetic after the `cos` and `sin`,
/// so the host and a board agree to the last bit given the same two. Its error over 2 560 steps is below 5e-4 (a test). The sum of
/// the window is `nsps`, so a steady tone's cell is the rectangular one's. A sample outside the audio reads as 0, as in the
/// rectangular kernel.
#[allow(clippy::too_many_arguments)]
fn fill_hann_2t<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    nsps: usize,
    i0: i64,
    mask: SymMask,
    cos_w: &[f32; NTONES],
    sin_w: &[f32; NTONES],
    coeff: &[f32; NTONES],
    two_pi_over_fs: f32,
    freq_hz: f32,
    to_12k: f32,
) {
    let len = 2 * nsps;
    // The window starts `nsps / 2` before the symbol and the recursion's output refers to the window's last sample, so the cell
    // is the rectangular kernel's cell times `e^{j w nsps/2}` (`w` the tone's angular frequency): with the tone step of 6.25 Hz
    // and `nsps / 2` samples of 0.08 s that is half a cycle a tone, **a sign flip on every odd tone** next to the common
    // `f0` term. The coherent metrics (`llrb`, `llrc`, `llre`, which OSD always computes) add cells of different symbols and
    // tones, so the cell is rotated back by `e^{-j w nsps/2}`, the rectangular kernel's reference. The angle is reduced to a
    // fraction of a cycle first (`w nsps/2` is about 500 rad: too large for `f32` directly).
    let mut rot_c = [0.0_f32; NTONES];
    let mut rot_s = [0.0_f32; NTONES];
    for tone in 0..NTONES {
        let omega = two_pi_over_fs * (freq_hz + (tone as f32) * TONE_SPACING_HZ);
        let cycles = omega * (nsps / 2) as f32 / core::f32::consts::TAU;
        let frac = cycles - cycles.floor();
        let theta = core::f32::consts::TAU * frac;
        rot_c[tone] = theta.cos();
        rot_s[tone] = theta.sin();
    }
    // the window once per call, its first half (`w[k] == w[len - k]`, `nsps + 1` floats: 5 KB at 8 kHz), made by `HannRecurrence`
    let mut weight = HannRecurrence::new(core::f32::consts::TAU / len as f32);
    let half: Vec<f32> = (0..=nsps).map(|_| weight.next()).collect();
    let audio_len = audio.len() as i64;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let start = i0 + (sym as i64) * (nsps as i64) - (nsps / 2) as i64;
        let mut s_prev = [0.0_f32; NTONES];
        let mut s_prev2 = [0.0_f32; NTONES];
        let inside = start >= 0 && start + len as i64 <= audio_len;
        for n in 0..len {
            let w = half[n.min(len - n)];
            // the window lies inside the audio (the hot path, no bounds test), or a sample outside it reads as 0
            let raw = if inside {
                audio[start as usize + n].to_i16() as f32
            } else {
                let idx = start + n as i64;
                if idx >= 0 && idx < audio_len {
                    audio[idx as usize].to_i16() as f32
                } else {
                    0.0
                }
            };
            let sample = raw * w;
            for tone in 0..NTONES {
                let s = sample + coeff[tone] * s_prev[tone] - s_prev2[tone];
                s_prev2[tone] = s_prev[tone];
                s_prev[tone] = s;
            }
        }
        for tone in 0..NTONES {
            let re = s_prev[tone] - cos_w[tone] * s_prev2[tone];
            let im = sin_w[tone] * s_prev2[tone];
            // x e^{-j theta}
            out[sym][tone] = Cmplx {
                re: (re * rot_c[tone] + im * rot_s[tone]) * to_12k,
                im: (im * rot_c[tone] - re * rot_s[tone]) * to_12k,
            };
        }
    }
}

#[cfg(test)]
mod tests_hann {
    use super::*;

    /// The recurrence is `sin^2` over the window: 2 x 1 280 and 2 x 1 920 samples, within 5e-4 of the double-precision value
    /// at every sample, and the window sums to half its length (the property the cell scale rests on). Fails if the recurrence
    /// starts at the wrong phase, uses the wrong `theta`, or drifts.
    #[test]
    fn the_hann_recurrence_is_sin_squared() {
        for len in [2560usize, 3840] {
            let theta = core::f32::consts::TAU / len as f32;
            let mut r = HannRecurrence::new(theta);
            let mut sum = 0.0f64;
            for k in 0..len {
                let w = r.next() as f64;
                let want = (core::f64::consts::PI * k as f64 / len as f64)
                    .sin()
                    .powi(2);
                assert!((w - want).abs() < 5e-4, "{len}: k {k}: {w} against {want}");
                sum += w;
            }
            assert!(
                (sum / (len as f64 / 2.0) - 1.0).abs() < 1e-3,
                "{len}: the window sums to {sum}"
            );
        }
    }

    /// The Hann cell has the **phase** of the rectangular kernel's cell, not only its magnitude: for a steady tone at each of the
    /// eight tone frequencies (a different run each), the 8 kHz cell of the tone equals `1.5 x sum_m x[s0 + m] e^{jw (nsps - 1 - m)}`
    /// (the rectangular Goertzel's value over the symbol, computed here in double precision, times the `12000 / rate` scale)
    /// within 3 %, as a complex number. The coherent metrics (`llrb`, `llrc`, `llre`) add cells of different symbols and tones;
    /// before the rotation by `e^{-j w nsps/2}` every odd tone's cell had the opposite sign. Fails without that rotation.
    #[cfg(feature = "fixed-point")]
    #[test]
    fn the_hann_cell_has_the_rectangular_cells_phase() {
        let rate = Ft8Rate::Hz8000;
        let (nsps, fs) = (1280usize, 8000.0f64);
        let (freq_hz, dt) = (1003.3f32, 0.2f32);
        let i0 = ((0.5f32 + dt) * 8000.0).round() as i64;
        for tone in 0..8usize {
            let f = freq_hz as f64 + tone as f64 * 6.25;
            let slot: Vec<i16> = (0..15 * 8000)
                .map(|n| (3000.0 * (core::f64::consts::TAU * f * n as f64 / fs + 0.7).cos()) as i16)
                .collect();
            let mut cs = [[Cmplx::<f32>::default(); 8]; 79];
            for mask in [SymMask::SyncOnly, SymMask::DataOnly] {
                fill_symbol_spectra_goertzel_at(&mut cs, &slot, rate, freq_hz, dt, mask);
            }
            for sym in [10usize, 11, 40] {
                let s0 = (i0 + (sym * nsps) as i64) as usize;
                let w = core::f64::consts::TAU * f / fs;
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for m in 0..nsps {
                    let a = w * (nsps - 1 - m) as f64;
                    re += slot[s0 + m] as f64 * a.cos();
                    im += slot[s0 + m] as f64 * a.sin();
                }
                let (re, im) = (1.5 * re, 1.5 * im);
                let c = cs[sym][tone];
                let err = ((c.re as f64 - re).powi(2) + (c.im as f64 - im).powi(2)).sqrt();
                let mag = (re * re + im * im).sqrt();
                assert!(
                    err < 0.03 * mag,
                    "tone {tone} symbol {sym}: cell ({}, {}) against the rectangular ({re}, {im})",
                    c.re,
                    c.im
                );
            }
        }
    }
}
