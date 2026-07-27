//! Linear-interpolation resampler: arbitrary input rate → 12 000 Hz.
//!
//! Used at the decode entry point so the rest of the pipeline can
//! assume a fixed 12 000 Hz sample rate.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

const TARGET_RATE: f64 = 12_000.0;

/// Resample `samples` from `src_rate` Hz to 12 000 Hz using linear interpolation.
///
/// Returns the resampled buffer.  If `src_rate` is already 12 000, the
/// input is returned as-is (zero-copy via `Cow` semantics at the call site).
pub fn resample_to_12k(samples: &[i16], src_rate: u32) -> Vec<i16> {
    let ratio = TARGET_RATE / src_rate as f64;
    let out_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);

    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;

        if idx + 1 < samples.len() {
            let a = samples[idx] as f64;
            let b = samples[idx + 1] as f64;
            let v = a + (b - a) * frac;
            out.push(v.round() as i16);
        } else if idx < samples.len() {
            out.push(samples[idx]);
        }
    }

    out
}

/// f32 → 12 000 Hz i16 in a single pass (linear interpolation + scaling).
///
/// Used by the WASM live-capture path so the JS side can hand a Float32Array
/// straight from the AudioWorklet without an intermediate i16 conversion loop.
///
/// **Normalization:** before resampling the input is peak-normalised to
/// `TARGET_PEAK` (0.8 full-scale).  This ensures the full i16 dynamic range
/// is used regardless of the hardware input level — a common problem with USB
/// radio audio adapters whose Windows volume setting may be very low.
/// Signal-to-noise ratio is preserved because signal and noise are scaled
/// equally.  Buffers whose peak is below `SILENCE_FLOOR` are treated as
/// silence and left at 0.
///
/// If `src_rate == 12000`, this still allocates and converts (no zero-copy)
/// because the output is i16 and the input is f32.
pub fn resample_f32_to_12k(samples: &[f32], src_rate: u32) -> Vec<i16> {
    const TARGET_PEAK: f64 = 0.8;
    const SILENCE_FLOOR: f64 = 1e-6;

    // Find peak amplitude
    let peak = samples.iter().fold(0.0f64, |m, &s| m.max((s as f64).abs()));
    let scale = if peak > SILENCE_FLOOR {
        TARGET_PEAK / peak
    } else {
        1.0
    };

    let ratio = TARGET_RATE / src_rate as f64;
    let out_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);

    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;

        let v = if idx + 1 < samples.len() {
            let a = samples[idx] as f64;
            let b = samples[idx + 1] as f64;
            a + (b - a) * frac
        } else if idx < samples.len() {
            samples[idx] as f64
        } else {
            continue;
        };

        let scaled = (v * scale * 32767.0).clamp(-32768.0, 32767.0);
        out.push(scaled.round() as i16);
    }

    out
}

/// f32 → 12 000 Hz f32, linear interpolation, **no normalisation**.
///
/// Preserves absolute amplitude — use this from decoders whose LLR
/// scaling depends on the raw signal/noise ratio (WSPR's noncoherent
/// 4-FSK LLR, for instance). If `src_rate == 12000`, the input is
/// copied verbatim; otherwise standard linear resampling applies.
pub fn resample_f32_to_12k_f32(samples: &[f32], src_rate: u32) -> Vec<f32> {
    if src_rate == 12_000 {
        return samples.to_vec();
    }
    let ratio = TARGET_RATE / src_rate as f64;
    let out_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;
        let v = if idx + 1 < samples.len() {
            let a = samples[idx] as f64;
            let b = samples[idx + 1] as f64;
            a + (b - a) * frac
        } else if idx < samples.len() {
            samples[idx] as f64
        } else {
            continue;
        };
        out.push(v as f32);
    }
    out
}

/// i16 → 12 000 Hz f32. Thin wrapper: resample as i16, convert to f32
/// in [-1, 1]. Used at WSPR WAV entry points where the incoming PCM
/// is `Int16Array` but the decoder wants `f32`.
pub fn resample_i16_to_12k_f32(samples: &[i16], src_rate: u32) -> Vec<f32> {
    if src_rate == 12_000 {
        return samples.iter().map(|&s| s as f32 / 32768.0).collect();
    }
    resample_to_12k(samples, src_rate)
        .into_iter()
        .map(|s| s as f32 / 32768.0)
        .collect()
}

/// Stateful chunk-based linear resampler `src_rate → 12 000 Hz`.
///
/// The batch [`resample_to_12k`] family allocates a fresh `Vec` per
/// call and assumes the entire input is in hand. Streaming receivers
/// (I2S DMA on ESP32 / RP2350 / Cortex-M, or sound-card capture on
/// host) instead push small chunks as they arrive and need the
/// resampler to carry interpolation state across calls so the chunk
/// boundary doesn't introduce a discontinuity.
///
/// `LinearResamplerI16To12k` is that streaming variant: same
/// linear-interpolation math as the batch path, plus a fixed-point
/// `phase_q32` (Q32 fractional source position) and a `last_in`
/// carry-over sample. Output is written into a caller-provided
/// buffer — no per-call heap allocation. Pure scalar i64 arithmetic;
/// runs on FPU-less MCUs.
///
/// **Pairs with** `MfskFt8Stream` in `mfsk-ffi-ft8`, which holds one
/// of these plus a 12 kHz ring buffer for the FT8 decode entry.
pub struct LinearResamplerI16To12k {
    src_rate: u32,
    /// Q32 source-sample step per output sample.
    /// `step_q32 = (src_rate << 32) / 12_000`.
    step_q32: u64,
    /// Q32 fractional source position relative to `last_in`.
    /// Invariant: maintained `< 2^32` whenever output is being produced
    /// (loop drains integer parts by consuming source samples first).
    phase_q32: u64,
    /// Last input sample absorbed; used as the left endpoint of the
    /// next interpolation pair.
    last_in: i16,
    /// `false` until the very first sample has been absorbed into
    /// `last_in`. The first `process()` call consumes one src sample
    /// to prime; from then on the resampler can produce one output
    /// per `step_q32` worth of phase.
    primed: bool,
}

impl LinearResamplerI16To12k {
    /// Construct a resampler for `src_rate_hz` → 12 000 Hz.
    /// Panics if `src_rate_hz == 0`.
    pub fn new(src_rate_hz: u32) -> Self {
        assert!(src_rate_hz > 0, "src_rate_hz must be > 0");
        let step_q32 = ((src_rate_hz as u64) << 32) / 12_000;
        Self {
            src_rate: src_rate_hz,
            step_q32,
            phase_q32: 0,
            last_in: 0,
            primed: false,
        }
    }

    /// Source rate this resampler was constructed with.
    pub fn src_rate(&self) -> u32 {
        self.src_rate
    }

    /// Consume up to `src.len()` input samples and emit up to
    /// `dst.len()` output samples at 12 kHz.
    ///
    /// Returns `(consumed, produced)`: the number of source samples
    /// drained from `src` and the number of output samples written
    /// into `dst[..produced]`. Either limit can be the binding one;
    /// the caller drives the loop by feeding more `src` chunks until
    /// `produced` reaches the desired count.
    ///
    /// Linear interpolation over the (`last_in`, next-src) pair —
    /// rounded half-up via `(diff * frac + (1 << 31)) >> 32`.
    pub fn process(&mut self, src: &[i16], dst: &mut [i16]) -> (usize, usize) {
        let mut src = src;
        let mut consumed = 0usize;
        let mut produced = 0usize;

        // Prime the carry sample on the very first call.
        if !self.primed {
            if src.is_empty() {
                return (0, 0);
            }
            self.last_in = src[0];
            src = &src[1..];
            consumed += 1;
            self.primed = true;
            // phase_q32 starts at 0 → first emitted output equals
            // `last_in` (matches the batch resampler, whose first
            // output is `samples[0]`).
        }

        while produced < dst.len() {
            // Drain whole-source-sample integer parts of phase by
            // shifting the (last_in, src[0]) pair forward.
            while self.phase_q32 >= 1u64 << 32 {
                if src.is_empty() {
                    return (consumed, produced);
                }
                self.last_in = src[0];
                src = &src[1..];
                consumed += 1;
                self.phase_q32 -= 1u64 << 32;
            }

            // Emit one output. If phase is exactly 0 the answer is
            // `last_in` itself (no interpolation needed, no src lookup
            // required — important for the tail of a finite stream).
            let out = if self.phase_q32 == 0 {
                self.last_in
            } else {
                // Need src[0] as the right endpoint.
                if src.is_empty() {
                    return (consumed, produced);
                }
                let a = self.last_in as i64;
                let b = src[0] as i64;
                let frac = self.phase_q32 as i64; // < 2^32
                // (b - a) ∈ [-65535, 65535]; * frac ∈ [-2^48, 2^48]; fits i64.
                let interp = a + (((b - a) * frac + (1 << 31)) >> 32);
                interp as i16
            };
            dst[produced] = out;
            produced += 1;
            self.phase_q32 += self.step_q32;
        }

        (consumed, produced)
    }

    /// Maximum number of output samples that *could* be produced
    /// from `src_len` source samples (worst case, ignoring fractional
    /// phase state). Used by the streaming wrapper to size scratch
    /// buffers.
    pub fn max_output_for(&self, src_len: usize) -> usize {
        // out ≈ src_len * 12000 / src_rate, plus 1 for boundary
        // rounding.
        ((src_len as u64) * 12_000 / self.src_rate as u64) as usize + 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passthrough_at_12k() {
        let input: Vec<i16> = (0..100).collect();
        let out = resample_to_12k(&input, 12000);
        assert_eq!(out.len(), 100);
        assert_eq!(out, input);
    }

    #[test]
    fn downsample_from_48k() {
        // 48000 → 12000 = factor 4
        let input: Vec<i16> = (0..4800).map(|i| (i % 100) as i16).collect();
        let out = resample_to_12k(&input, 48000);
        assert_eq!(out.len(), 1200);
    }

    #[test]
    fn downsample_from_44100() {
        // 44100 → 12000: non-integer ratio
        let input: Vec<i16> = vec![0i16; 44100];
        let out = resample_to_12k(&input, 44100);
        // Should be close to 12000 samples for 1 second
        assert!((out.len() as i32 - 12000).abs() <= 1);
    }

    // ── Streaming resampler tests ────────────────────────────────────

    #[test]
    fn streaming_passthrough_at_12k() {
        let input: Vec<i16> = (0..100).collect();
        let mut r = LinearResamplerI16To12k::new(12_000);
        let mut out = vec![0i16; 100];
        let (cons, prod) = r.process(&input, &mut out);
        assert_eq!(prod, 100);
        // Pass-through should emit the input verbatim (modulo boundary
        // tail). At 12k → 12k, step_q32 = 2^32 so phase is always 0
        // when emitting and every output equals last_in = src[i].
        assert_eq!(&out[..prod], &input[..prod]);
        // Consumed 1 prime + (prod - 1) post-prime drains = prod source samples.
        assert_eq!(cons, prod);
    }

    #[test]
    fn streaming_downsample_48k_to_12k() {
        // 48k → 12k = factor 4. step_q32 = 4 * 2^32.
        let input: Vec<i16> = (0..400).map(|i| (i * 100) as i16).collect();
        let mut r = LinearResamplerI16To12k::new(48_000);
        let mut out = vec![0i16; 100];
        let (cons, prod) = r.process(&input, &mut out);
        assert_eq!(prod, 100);
        // Output[0] = src[0], Output[1] = src[4], Output[2] = src[8], …
        // (phase exactly 0 at every emission since the ratio is integer).
        assert_eq!(out[0], 0);
        assert_eq!(out[1], 400);
        assert_eq!(out[2], 800);
        assert_eq!(cons, 397); // 1 prime + 99 post-prime × 4 = 397
    }

    #[test]
    fn streaming_chunked_matches_single_call() {
        // Splitting the input into chunks must not change the output —
        // this is the whole point of carrying state.
        let input: Vec<i16> = (0..4410).map(|i| (i % 200) as i16).collect();

        let mut r1 = LinearResamplerI16To12k::new(44_100);
        let mut single = vec![0i16; 1500];
        let (_c1, p1) = r1.process(&input, &mut single);

        let mut r2 = LinearResamplerI16To12k::new(44_100);
        let mut chunked = vec![0i16; 1500];
        let mut produced = 0;
        let mut src_pos = 0;
        while src_pos < input.len() && produced < chunked.len() {
            let chunk_end = (src_pos + 137).min(input.len()); // odd chunk size
            let (c, p) = r2.process(&input[src_pos..chunk_end], &mut chunked[produced..]);
            src_pos += c;
            produced += p;
            if c == 0 && p == 0 {
                break; // would be infinite loop otherwise
            }
        }

        assert_eq!(produced, p1);
        assert_eq!(&chunked[..produced], &single[..p1]);
    }

    #[test]
    fn streaming_upsample_6k_to_12k() {
        // 6k → 12k = factor 0.5. step_q32 = 2^31.
        // Outputs alternate: src[i], midpoint(src[i], src[i+1]), src[i+1], midpoint, …
        let input: Vec<i16> = vec![0, 100, 200, 300, 400, 500];
        let mut r = LinearResamplerI16To12k::new(6_000);
        let mut out = vec![0i16; 11];
        let (_cons, prod) = r.process(&input, &mut out);
        // First output = src[0] = 0.
        assert_eq!(out[0], 0);
        // Second output = midpoint(0, 100) = 50.
        assert_eq!(out[1], 50);
        // Third output = src[1] = 100.
        assert_eq!(out[2], 100);
        // Fourth output = midpoint(100, 200) = 150.
        assert_eq!(out[3], 150);
        assert!(prod >= 10);
    }

    // Integration tests that depend on ft8-core's decode pipeline live in
    // `ft8-core/tests/resample_ft8.rs` (moved there alongside this module's
    // migration to mfsk-core).
}
