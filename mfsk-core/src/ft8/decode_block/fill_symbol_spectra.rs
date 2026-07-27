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
//! external callers (`super::decode::*` host path, `mfsk-ffi-ft8`
//! embedded entry, `embedded-shared::stage1_inc`) keep the same
//! `mfsk_core::ft8::decode_block::*` paths as before. The legacy
//! BASIS (Q15 sin/cos dot-product) fill path was removed in 0.8.0
//! (issue #162) — `fill_symbol_spectra_goertzel` (Phase
//! 1.7.7-Stick) has been the sole production path on every embedded
//! target since 0.6.4.

use alloc::boxed::Box;
use alloc::vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::params::{COSTAS, COSTAS_POS, NN, NSPS, NTONES};
use super::types::{AudioSample, SAMPLE_RATE_HZ, TONE_SPACING_HZ, TX_START_OFFSET_S};
use crate::engine::scalar::Cmplx;

/// Last sample index that
/// [`fill_symbol_spectra_goertzel`] reads for a candidate at
/// `dt_sec`. Same `i0 + NN * NSPS` formula as the per-symbol loop
/// inside the kernel, exposed here so embedded callers (Phase C
/// audio-tail speculation in `embedded-shared`) can decide whether
/// a candidate's Goertzel window fits inside the currently-captured
/// audio prefix without re-implementing the constants.
pub fn goertzel_window_end_sample(dt_sec: f32) -> usize {
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    (i0 + (NN as i64) * (NSPS as i64)).max(0) as usize
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
pub fn fill_symbol_spectra_generic<Sc: crate::engine::scalar::SpecScalar, S: AudioSample>(
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

/// Per-symbol DFT via **generalised Goertzel** — the embedded
/// production fill (Phase 1.7.7-Stick) with **zero caller-provided
/// scratch**. Replaced the legacy BASIS (Q15 sin/cos dot-product)
/// fill path, which was removed in 0.8.0 (issue #162) once this had
/// been the sole production path on every embedded target for
/// several releases.
///
/// Same output (`Σ x[n] exp(-jωn)` for each tone at `ω = 2π·f/Fs`
/// across NSPS=1920 samples), produced via a 2-tap IIR recursion
/// instead of a sin/cos dot-product. Needs no basis scratch at all —
/// the whole point of Phase 1.7.7-Stick.
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
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;

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

    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        // 8 independent recursion states.
        let mut s_prev = [0.0_f32; NTONES];
        let mut s_prev2 = [0.0_f32; NTONES];

        // Hot path: entire symbol window lies within audio bounds.
        // The bounds check (idx >= 0 && idx < audio.len()) can then be
        // hoisted out of the 1920-iteration inner loop, leaving a clean
        // sequential load that LLVM's unroll + Xtensa FPU pipeline can
        // run at ~1 cycle/tone/sample. Cold path (first/last ~1 symbol
        // per candidate) keeps the original per-sample check.
        if sym_start >= 0 && (sym_start as usize) <= audio.len().saturating_sub(NSPS) {
            let base = sym_start as usize;
            for n in 0..NSPS {
                let sample = audio[base + n].to_i16() as f32;
                for tone in 0..NTONES {
                    let s = sample + coeff[tone] * s_prev[tone] - s_prev2[tone];
                    s_prev2[tone] = s_prev[tone];
                    s_prev[tone] = s;
                }
            }
        } else {
            let audio_len = audio.len() as i64;
            for n in 0..NSPS {
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
            out[sym][tone] = Cmplx { re, im };
        }
    }
}
