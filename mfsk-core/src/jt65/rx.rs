//! JT65 receiver: audio → 63 hard-decision RS symbols → message.
//!
//! JT65 demodulation is hard-decision (unlike FT8/FT4/FST4/WSPR's
//! bit-LLR path): for each of the 63 data positions we run a
//! symbol-length FFT and take the argmax across the 64 data-tone
//! bins. The resulting symbols are de-Gray'd, de-interleaved, and
//! fed straight to [`crate::fec::Rs63_12::decode_jt65`].
//!
//! Geometry: NSPS = 4460 samples at 12 kHz gives bin width ≈
//! 2.6906 Hz = one JT65A tone spacing.

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::ModulationParams;
use crate::engine::dsp::symbol_fft::SymbolFft;
use num_complex::Complex;

use super::Jt65;
use super::gray::inv_gray6;
use super::interleave::deinterleave;
use super::sync_pattern::JT65_NPRC;

/// Everything one pass of the JT65 demodulator produces.
///
/// One function, [`demodulate_aligned`], returns all of it. Until #390
/// four public functions ran the same pass and each handed back a
/// different subset (symbols; + `conf`; + `snr_db`; everything).
#[derive(Clone, Debug)]
pub struct Jt65Demod {
    /// The 63 hard-decision symbols in **RS codeword order**
    /// (Gray-decoded and de-interleaved), ready for
    /// [`crate::fec::Rs63_12::decode_jt65`].
    pub symbols: [u8; 63],
    /// Per-position confidence `(best_power − second_power) /
    /// best_power`, in `[0, 1]`, codeword order. 1 means the winning
    /// tone dominates; 0 means the top two tones are tied.
    pub conf: [f32; 63],
    /// Each position's runner-up symbol (Gray-decoded, codeword
    /// order) — its identity, not just its power. Needed by
    /// [`crate::jt65::chase`]'s soft-distance ranking, which mirrors
    /// WSJT-X `ftrsdap`'s `nsoft` check of whether a correction landed
    /// on the second-best guess.
    pub second_symbols: [u8; 63],
    /// WSJT-X `demod64a.f90`'s real `mrprob`/`p1` reliability metric,
    /// codeword order: `best_pwr / total_pwr`, where `total_pwr` sums
    /// *all 64* tone powers at that position, not just the top two.
    ///
    /// **Not the same quantity as `conf`**, which is a top-2-only
    /// margin. `rel` also captures how much energy leaked into the
    /// other 62 tones (the position's own local noise floor), which
    /// `conf` discards. This matters: `chase::decode_at_with_chase`
    /// first used `conf` (incorrectly) where WSJT-X's `ftrsdap` uses
    /// `rxprob`/`mrprob`, i.e. this `rel`, for both the
    /// erasure-priority ordering and the `nsoft` soft-distance
    /// weighting. See `chase`'s module doc for the fix and what it
    /// changed. `conf` is still needed separately: WSJT-X's
    /// `rxprob2/rxprob` ratio *is* just `second_pwr/best_pwr`
    /// regardless of the `psum` normalisation, so `1 - conf` remains
    /// the right quantity for that one piece.
    pub rel: [f32; 63],
    /// `raw_pwr[j][tone]` is the un-thresholded FFT-bin power for data
    /// tone `tone` (0..64) at the `j`-th data symbol position (0..63)
    /// **in raw temporal order**. That is the order WSJT-X's
    /// `s3(64,63)` array is in, *before* [`deinterleave`] permutes the
    /// positions. [`crate::jt65::chase`]'s `getpp` port needs it to
    /// re-consult the original spectrum when scoring candidate
    /// codewords (WSJT-X `extract.f90` keeps a raw copy `s3a=s3` for
    /// exactly this). 63×64 `f32` is about 16 KB.
    pub raw_pwr: [[f32; 64]; 63],
    /// Decode-side SNR estimate (dB). Signal is the power at each
    /// symbol's winning tone; noise is the mean power of the other 63
    /// candidate tones in the same symbol slot. That is the same
    /// "opposite-bin" logic as
    /// [`crate::engine::llr::compute_snr_db_generic`] for FT8/FT4/FST4,
    /// generalised from a single opposite tone to a 63-tone average,
    /// since JT65's data alphabet has no natural comb midpoint.
    ///
    /// It is converted to WSJT-X's 2500 Hz reference bandwidth by
    /// `10·log10(2500/df)`, the same bandwidth-normalisation shape as
    /// FT8's `-27 dB` and wsprd's `-26.3 dB` constants
    /// (`engine/llr.rs`, `wspr/coarse_baseband.rs`). It is **not
    /// independently calibrated** against a real JT65 signal corpus
    /// (`jt65sim` isn't buildable in this environment; see
    /// `tests/jt65_sweep.rs`), so treat it as accurate to roughly ±1-2
    /// dB against WSJT-X's own JT65 SNR readout until validated. It is
    /// clamped to `[-30, -1]` the way WSJT-X's display is.
    pub snr_db: f32,
}

/// Demodulate one JT65 frame from aligned audio. Returns `None` if the
/// 126-symbol frame or the 66-tone band does not fit.
pub fn demodulate_aligned(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<Jt65Demod> {
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32; // ≡ TONE_SPACING_HZ
    let base_bin = (base_freq_hz / df).round() as usize;
    if start_sample + 126 * nsps > audio.len() || base_bin + 66 >= nsps / 2 {
        return None;
    }

    let mut fft = SymbolFft::new(nsps);
    // Walk 126 symbol windows. Data positions (NPRC[i] == 0) each get
    // argmax of 64 data-tone magnitudes (+ runner-up for confidence).
    // `second_tone` tracks the runner-up's *identity* (not just its
    // power) — needed by `chase::decode_at_with_chase`'s soft-distance
    // candidate ranking (mirrors WSJT-X `ftrsdap`'s `nsoft`, which
    // checks whether a correction landed on the 2nd-best guess).
    let mut symbols = [0u8; 63];
    let mut conf = [0f32; 63];
    let mut rel = [0f32; 63];
    let mut second_tone_sym = [0u8; 63];
    // Raw un-thresholded power per (temporal position, tone) — kept in
    // WSJT-X's `s3a` order (temporal, i.e. *not* deinterleaved) since
    // that's the order `chase::getpp` re-projects a candidate codeword
    // into before looking values up. See `Jt65Demod::raw_pwr`.
    let mut raw_pwr = [[0f32; 64]; 63];
    let mut xsig_sum = 0.0f32;
    let mut xnoi_sum = 0.0f32;
    let mut k = 0usize;

    // Sub-bin frequency correction. `base_bin` is `base_freq_hz`
    // rounded to the nearest FFT bin — for a caller passing a
    // fractional (refined) frequency, `residual_hz` is the leftover
    // offset (≤ half a bin, ≈1.35 Hz worst case at this NSPS/rate).
    // Left uncorrected, that residual costs real detection sensitivity
    // — a rectangular-window FFT's worst-case (exact half-bin) power
    // loss is a well-known ≈3.9 dB "scalloping loss", confirmed by
    // measurement on this crate's own AWGN corpus (see
    // `docs/notes/BENCHMARKS.md`'s JT65 section). WSJT-X avoids this
    // by correcting the residual on the *time-domain* signal before
    // any FFT (`twkfreq65.f90`, driven by `afc65b`'s continuous
    // frequency fit) — this mirrors that, via a running-phase NCO
    // applied while converting each real sample to complex, so the
    // correction is phase-continuous across all 126 symbol windows
    // (they tile the buffer with no gaps, so a per-sample running
    // phase computed once here stays exact throughout — no need to
    // reset or re-derive it per window). Same running-accumulator
    // pattern as `engine::dsp::subtract`'s NCO loops.
    let residual_hz = base_freq_hz - base_bin as f32 * (sample_rate as f32 / nsps as f32);
    let dphi = -core::f32::consts::TAU * residual_hz / sample_rate as f32;
    let mut phase = 0.0f32;

    for sym_idx in 0..126 {
        let sym_start = start_sample + sym_idx * nsps;
        let buf = fft.with_input(|buf| {
            for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
                *slot = Complex::new(s, 0.0) * Complex::new(phase.cos(), phase.sin());
                phase += dphi;
                if phase > core::f32::consts::PI {
                    phase -= core::f32::consts::TAU;
                } else if phase < -core::f32::consts::PI {
                    phase += core::f32::consts::TAU;
                }
            }
        });
        if JT65_NPRC[sym_idx] == 1 {
            continue;
        }
        let mut best_tone = 0u8;
        let mut best_pwr = f32::NEG_INFINITY;
        let mut second_tone = 0u8;
        let mut second_pwr = f32::NEG_INFINITY;
        let mut total_pwr = 0.0f32;
        for tone in 0u8..64 {
            let bin = base_bin + 2 + tone as usize;
            let p = buf[bin].norm_sqr();
            raw_pwr[k][tone as usize] = p;
            total_pwr += p;
            if p > best_pwr {
                second_pwr = best_pwr;
                second_tone = best_tone;
                best_pwr = p;
                best_tone = tone;
            } else if p > second_pwr {
                second_pwr = p;
                second_tone = tone;
            }
        }
        symbols[k] = inv_gray6(best_tone);
        second_tone_sym[k] = inv_gray6(second_tone);
        conf[k] = if best_pwr > 0.0 {
            ((best_pwr - second_pwr.max(0.0)) / best_pwr).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // WSJT-X `demod64a.f90`: `if(psum.eq.0.0) psum=1.e-6` — avoid
        // div-by-zero on a fully silent position; degenerate case, no
        // real reliability signal either way.
        rel[k] = if total_pwr > 0.0 {
            (best_pwr / total_pwr).clamp(0.0, 1.0)
        } else {
            0.0
        };
        xsig_sum += best_pwr;
        xnoi_sum += (total_pwr - best_pwr) / 63.0;
        k += 1;
    }
    debug_assert_eq!(k, 63);
    deinterleave(&mut symbols);
    deinterleave(&mut second_tone_sym);
    // Apply the same permutation to confidence/reliability so
    // positions line up. Re-run the same 7×9 transpose `deinterleave`
    // uses (it's `[u8;63]`-only, so `f32` arrays can't call it
    // directly) so both stay aligned with the permuted symbols.
    let mut conf_perm = [0f32; 63];
    let mut rel_perm = [0f32; 63];
    for i in 0..7 {
        for j in 0..9 {
            conf_perm[j * 7 + i] = conf[i * 9 + j];
            rel_perm[j * 7 + i] = rel[i * 9 + j];
        }
    }

    // WSJT-X's real JT65 display clamp, `jt65_decode.f90:254-255`:
    //
    //     nsnr=nint(s2db)
    //     if(nsnr.lt.-30) nsnr=-30
    //     if(nsnr.gt.-1) nsnr=-1
    //
    // JT65 is the one protocol here whose displayed SNR *saturates by
    // design*: real `jt9` reports `-1` for anything stronger, verified
    // directly (a `jt65sim` signal injected at +10 dB and at +5 dB
    // both come back `-1`; 0 dB comes back `-3`). Reproducing the
    // clamp matters for matching what a JT65 operator actually sees —
    // and the floor matters too: the previous ad-hoc `-24` floor bound
    // before WSJT-X's own `-30` did, truncating the weakest decodes.
    //
    // Replaces an ad-hoc `[-24, +49]` pair. The ceiling also doubles
    // as the answer when `xnoi_sum` is (near) exactly zero: a
    // perfectly clean synthetic signal sampled with an integer number
    // of cycles per FFT window can leave *zero* measurable leakage in
    // the non-winning tones — that means "no measurable noise" (best
    // case), not the worst case the floor implies. See the identical
    // fix + explanation in `q65::rx::snr_db_from_sig_noi`, which keeps
    // its own `49.0` because Q65 has no such display clamp.
    const SNR_FLOOR_DB: f32 = -30.0;
    const SNR_CEIL_DB: f32 = -1.0;
    let snr_db = if xnoi_sum < f32::EPSILON {
        if xsig_sum < f32::EPSILON {
            SNR_FLOOR_DB
        } else {
            SNR_CEIL_DB
        }
    } else {
        let ratio = xsig_sum / xnoi_sum - 1.0;
        if ratio <= 0.001 {
            SNR_FLOOR_DB
        } else {
            let bw_offset_db =
                10.0 * (2500.0 / <Jt65 as ModulationParams>::TONE_SPACING_HZ).log10();
            (10.0 * ratio.log10() - bw_offset_db).clamp(SNR_FLOOR_DB, SNR_CEIL_DB)
        }
    };

    Some(Jt65Demod {
        symbols,
        conf: conf_perm,
        second_symbols: second_tone_sym,
        rel: rel_perm,
        raw_pwr,
        snr_db,
    })
}

#[cfg(test)]
mod tests {
    use super::super::tx::synthesize_standard;
    use super::*;
    use crate::engine::{DecodeContext, MessageCodec};
    use crate::fec::Rs63_12;
    use crate::msg::{Jt72Codec, Jt72Message};

    #[test]
    fn synth_decode_roundtrip_cq_k1abc_fn42() {
        let freq = 1270.0;
        let audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("pack+synth");
        let received = demodulate_aligned(&audio, 12_000, 0, freq)
            .expect("demod")
            .symbols;
        let rs = Rs63_12::new();
        let (info, nerr) = rs.decode_jt65(&received).expect("clean decode");
        assert_eq!(nerr, 0, "clean synth should have zero errors");

        // Pack 12 × 6-bit words into 72 MSB-first bits, then unpack
        // via Jt72 codec.
        let mut payload = [0u8; 72];
        for (i, bit) in payload.iter_mut().enumerate() {
            let word = info[i / 6];
            let shift = 5 - (i % 6);
            *bit = (word >> shift) & 1;
        }
        let msg = Jt72Codec::default()
            .unpack(&payload, &DecodeContext::default())
            .expect("unpack");
        match msg {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => {
                assert_eq!(call1, "CQ");
                assert_eq!(call2, "K1ABC");
                assert_eq!(grid_or_report, "FN42");
            }
            other => panic!("expected Standard, got {:?}", other),
        }
    }
}
