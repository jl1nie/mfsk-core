//! Per-symbol complex demodulation on the 375 Hz baseband.
//!
//! Faithful port of WSJT-X `wsprd.c::noncoherent_sequence_detection`
//! (line 342). Runs four explicit complex oscillators per symbol — one
//! per WSPR tone at `fp ± 0.5·df` and `fp ± 1.5·df` — accumulates the
//! 256 baseband samples that make up one symbol, and stores the
//! resulting complex amplitude per (tone, symbol) into an `IsQs` table.
//!
//! That table feeds two paths:
//!  - **nblock = 1** (this slice): per-symbol bit metric
//!    `bm[i] = |p_data1| − |p_data0|`, matching the magnitude formula
//!    from `wsprd.c:451-468` with no coherent block sum.
//!  - **nblock = 2 / 3 / 6 / 9** (next slice): coherent block sum of
//!    complex amplitudes across consecutive symbols *with phase
//!    tracking* between symbols (`cm, sm` rotation), then take the
//!    magnitude of the coherent sum. ~3 dB SNR gain per doubling of
//!    `nblock` over non-coherent detection — this is what wsprd uses
//!    in pass 3 and what gets us the W5BIT / NM7J / W3BI weak decodes.

use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::WSPR_SYNC_VECTOR;
use super::baseband::BASEBAND_RATE;

/// WSPR tone spacing in Hz. Identical to `df = 375.0 / 256.0` from
/// `wsprd.c:355` (line 355: `dt = 1.0/375.0`, line 357: `df = 375/256`).
pub const TONE_SPACING_HZ: f32 = BASEBAND_RATE / 256.0; // = 1.4648437...

/// Samples per symbol on the baseband. Matches `nspersym = 256` from
/// `wsprd.c:355` and `noncoherent_sequence_detection`'s inner DFT
/// length.
pub const NSPS_BASEBAND: usize = 256;

/// Total channel symbols in a WSPR frame.
pub const N_SYMBOLS: usize = 162;

/// Per-(tone, symbol) complex amplitude table, sized `[4][162]`.
/// Output of [`tone_amplitudes`]. Layout: `is[tone][symbol]`,
/// `qs[tone][symbol]` — same as wsprd's `is[4][162]` and `qs[4][162]`.
#[derive(Clone, Debug)]
pub struct IsQs {
    /// `is[tone][symbol]` — in-phase coherent sum over one symbol's
    /// 256 baseband samples mixed against the per-tone oscillator.
    pub is: [[f32; N_SYMBOLS]; 4],
    /// `qs[tone][symbol]` — quadrature counterpart.
    pub qs: [[f32; N_SYMBOLS]; 4],
    /// Per-(tone, symbol) inter-symbol phase advance (cos / sin).
    /// Used by coherent block detection (next slice) to correctly
    /// rotate the complex amplitude when summing across symbols.
    /// Matches wsprd's `cf` / `sf` arrays.
    pub cf: [[f32; N_SYMBOLS]; 4],
    pub sf: [[f32; N_SYMBOLS]; 4],
}

/// Build per-tone complex amplitudes for all 162 WSPR symbols on a
/// 375 Hz complex baseband. `f0_baseband_hz` is the tone-CENTER
/// frequency relative to the 1500 Hz dial (so `f0 = 0` means tone
/// center at 1500 Hz, which is **between** tones 1 and 2 of a WSPR
/// signal whose tone 0 sits at 1500 − 1.5·df Hz). `lag` is the
/// baseband sample index where symbol 0 starts; `drift_hz` is the
/// linear-drift coefficient (full Hz of drift over the whole frame).
///
/// Mirrors `wsprd.c:354-436`. The four explicit oscillators sit at:
///   tone 0: fp − 1.5·df
///   tone 1: fp − 0.5·df
///   tone 2: fp + 0.5·df
///   tone 3: fp + 1.5·df
/// where `fp = f0 + drift/2 · (i − 81)/81` per symbol `i`. This
/// drift-compensated layout is what lets coherent block detection
/// accumulate energy across multi-second windows on real (drifting)
/// WSPR signals.
pub fn tone_amplitudes(
    idat: &[f32],
    qdat: &[f32],
    f0_baseband_hz: f32,
    lag: i32,
    drift_hz: f32,
) -> IsQs {
    debug_assert_eq!(idat.len(), qdat.len());
    let np = idat.len() as i32;
    let dt = 1.0 / BASEBAND_RATE;
    let df = TONE_SPACING_HZ;
    let twopidt = 2.0 * PI * dt;
    let df15 = df * 1.5;
    let df05 = df * 0.5;

    let mut isqs = IsQs {
        is: [[0.0f32; N_SYMBOLS]; 4],
        qs: [[0.0f32; N_SYMBOLS]; 4],
        cf: [[0.0f32; N_SYMBOLS]; 4],
        sf: [[0.0f32; N_SYMBOLS]; 4],
    };

    // Per-tone oscillator lookup (reused across symbols when the
    // drift-corrected freq fp is unchanged from the previous symbol —
    // wsprd does this optimisation; we just rebuild every symbol for
    // clarity, the cost is trivial at NSPS=256).
    let mut c0 = [0.0f32; NSPS_BASEBAND + 1];
    let mut s0 = [0.0f32; NSPS_BASEBAND + 1];
    let mut c1 = [0.0f32; NSPS_BASEBAND + 1];
    let mut s1 = [0.0f32; NSPS_BASEBAND + 1];
    let mut c2 = [0.0f32; NSPS_BASEBAND + 1];
    let mut s2 = [0.0f32; NSPS_BASEBAND + 1];
    let mut c3 = [0.0f32; NSPS_BASEBAND + 1];
    let mut s3 = [0.0f32; NSPS_BASEBAND + 1];

    for i in 0..N_SYMBOLS {
        let fp = f0_baseband_hz + (drift_hz / 2.0) * ((i as f32 - 81.0) / 81.0);

        let dphi0 = twopidt * (fp - df15);
        let dphi1 = twopidt * (fp - df05);
        let dphi2 = twopidt * (fp + df05);
        let dphi3 = twopidt * (fp + df15);
        let (cdphi0, sdphi0) = (dphi0.cos(), dphi0.sin());
        let (cdphi1, sdphi1) = (dphi1.cos(), dphi1.sin());
        let (cdphi2, sdphi2) = (dphi2.cos(), dphi2.sin());
        let (cdphi3, sdphi3) = (dphi3.cos(), dphi3.sin());

        c0[0] = 1.0;
        s0[0] = 0.0;
        c1[0] = 1.0;
        s1[0] = 0.0;
        c2[0] = 1.0;
        s2[0] = 0.0;
        c3[0] = 1.0;
        s3[0] = 0.0;
        for j in 1..=NSPS_BASEBAND {
            c0[j] = c0[j - 1] * cdphi0 - s0[j - 1] * sdphi0;
            s0[j] = c0[j - 1] * sdphi0 + s0[j - 1] * cdphi0;
            c1[j] = c1[j - 1] * cdphi1 - s1[j - 1] * sdphi1;
            s1[j] = c1[j - 1] * sdphi1 + s1[j - 1] * cdphi1;
            c2[j] = c2[j - 1] * cdphi2 - s2[j - 1] * sdphi2;
            s2[j] = c2[j - 1] * sdphi2 + s2[j - 1] * cdphi2;
            c3[j] = c3[j - 1] * cdphi3 - s3[j - 1] * sdphi3;
            s3[j] = c3[j - 1] * sdphi3 + s3[j - 1] * cdphi3;
        }

        // wsprd `wsprd.c:413-416`: store the 256-sample-on tone phase
        // (= the inter-symbol rotation) for the coherent block step.
        isqs.cf[0][i] = c0[NSPS_BASEBAND];
        isqs.sf[0][i] = s0[NSPS_BASEBAND];
        isqs.cf[1][i] = c1[NSPS_BASEBAND];
        isqs.sf[1][i] = s1[NSPS_BASEBAND];
        isqs.cf[2][i] = c2[NSPS_BASEBAND];
        isqs.sf[2][i] = s2[NSPS_BASEBAND];
        isqs.cf[3][i] = c3[NSPS_BASEBAND];
        isqs.sf[3][i] = s3[NSPS_BASEBAND];

        // Mix one symbol's 256 baseband samples against each of the
        // 4 tone oscillators. wsprd `wsprd.c:418-435` literal port.
        let mut i0_acc = 0.0f32;
        let mut q0_acc = 0.0f32;
        let mut i1_acc = 0.0f32;
        let mut q1_acc = 0.0f32;
        let mut i2_acc = 0.0f32;
        let mut q2_acc = 0.0f32;
        let mut i3_acc = 0.0f32;
        let mut q3_acc = 0.0f32;
        for j in 0..NSPS_BASEBAND {
            let k = lag + (i as i32) * (NSPS_BASEBAND as i32) + (j as i32);
            if k > 0 && k < np {
                let id = idat[k as usize];
                let qd = qdat[k as usize];
                i0_acc += id * c0[j] + qd * s0[j];
                q0_acc += -id * s0[j] + qd * c0[j];
                i1_acc += id * c1[j] + qd * s1[j];
                q1_acc += -id * s1[j] + qd * c1[j];
                i2_acc += id * c2[j] + qd * s2[j];
                q2_acc += -id * s2[j] + qd * c2[j];
                i3_acc += id * c3[j] + qd * s3[j];
                q3_acc += -id * s3[j] + qd * c3[j];
            }
        }
        isqs.is[0][i] = i0_acc;
        isqs.qs[0][i] = q0_acc;
        isqs.is[1][i] = i1_acc;
        isqs.qs[1][i] = q1_acc;
        isqs.is[2][i] = i2_acc;
        isqs.qs[2][i] = q2_acc;
        isqs.is[3][i] = i3_acc;
        isqs.qs[3][i] = q3_acc;
    }

    isqs
}

/// Per-symbol non-coherent bit metric (`nblock = 1` path of
/// `noncoherent_sequence_detection`). For each symbol `i`:
/// ```text
///   sync = WSPR_SYNC_VECTOR[i]     (∈ {0, 1})
///   data = 0 → tone (sync), data = 1 → tone (sync + 2)
///   bm[i] = |is_data1 + j·qs_data1| − |is_data0 + j·qs_data0|
/// ```
///
/// Then z-score normalise the 162-element bm vector (matches wsprd's
/// `normalizebmet` plus `symfac/fac` scale). Output is bm scaled by
/// `LLR_SCALE` so the same Fano metric calibration that works for
/// FT4 / FT8 in `core::fec::ConvFano` works here.
pub fn nblock1_bit_metrics(isqs: &IsQs) -> [f32; N_SYMBOLS] {
    nblock1_bit_metrics_opt(isqs, false)
}

/// [`nblock1_bit_metrics`] with wsprd's `bitmetric` (`bitbybit`)
/// normalisation — the fourth rung of its blocksize ladder.
///
/// `wsprd.c:465-468`:
///
/// ```c
/// fsymb[i+ib] = xm1 - xm0;
/// if (bitbybit == 1) {
///     fsymb[i+ib] = fsymb[i+ib] / (xm1 > xm0 ? xm1 : xm0);
/// }
/// ```
///
/// Dividing each symbol's metric by the larger of its two tone powers
/// turns an *absolute* difference into a *relative* one, so a symbol's
/// confidence stops scaling with its own instantaneous amplitude. The
/// z-score normalisation applied to the whole 162-symbol vector
/// afterwards cannot do this — it only removes overall scale, not
/// symbol-to-symbol amplitude variation. That is why the rung exists
/// and why it should matter most on a fading channel.
///
/// `wsprd` reaches it via `ib == 4` → `blocksize = 1, bitmetric = 1`
/// (`wsprd.c:1318-1319`), and only on passes ≥ 1 — `nblocksize` is 1
/// on pass 0 and 4 afterwards (`wsprd.c:1001,1006`). This crate had
/// the `blocksize` axis of that ladder but not this one.
pub fn nblock1_bit_metrics_opt(isqs: &IsQs, bit_by_bit: bool) -> [f32; N_SYMBOLS] {
    let mut bm = [0.0f32; N_SYMBOLS];
    for i in 0..N_SYMBOLS {
        let sync = WSPR_SYNC_VECTOR[i] as usize;
        let t0 = sync; // data = 0
        let t1 = sync + 2; // data = 1
        let p0 = (isqs.is[t0][i].powi(2) + isqs.qs[t0][i].powi(2)).sqrt();
        let p1 = (isqs.is[t1][i].powi(2) + isqs.qs[t1][i].powi(2)).sqrt();
        // LLR convention: positive → bit 0 more likely (matches our
        // existing `mags_to_llrs`, `core::fec::ConvFano::build_branch_metrics`).
        bm[i] = p0 - p1;
        if bit_by_bit {
            let m = p0.max(p1);
            if m > 0.0 {
                bm[i] /= m;
            }
        }
    }
    // z-score normalise
    let n = N_SYMBOLS as f32;
    let mean = bm.iter().sum::<f32>() / n;
    let mean_sq = bm.iter().map(|x| x * x).sum::<f32>() / n;
    let var = mean_sq - mean * mean;
    let sig = if var > 0.0 {
        var.sqrt()
    } else {
        mean_sq.sqrt()
    };
    if sig > 0.0 {
        for x in bm.iter_mut() {
            *x /= sig;
        }
    }
    // Scale to Fano-friendly LLR magnitudes. wsprd uses symfac=50 then
    // quantises to ±127; our Fano takes float scaled by `LLR_SCALE`.
    const LLR_SCALE: f32 = 2.83;
    for x in bm.iter_mut() {
        *x *= LLR_SCALE;
    }
    bm
}

/// Coherent-block-sum bit metrics for `nblock ∈ {1, 2, 3}`. Faithful
/// port of `wsprd.c:438-470`: for each block of `nblock` consecutive
/// symbols, enumerate all `2^nblock` hypotheses for the data bits in
/// the block; for each hypothesis `j`, coherently accumulate the
/// per-tone IQ amplitudes across the block with running phase
/// rotation (`cm, sm` updated by each symbol's `cf[tone], sf[tone]`);
/// take the magnitude of the coherent sum; then for each bit position
/// `bm = max_p(bit=1) − max_p(bit=0)`.
///
/// SNR gain is roughly +3 dB / doubling of `nblock`, gated by the
/// channel coherence time. wsprd's pass 2 tries nblock=1, 2, 3.
/// `nblock` higher than 3 (= 6, 9) needs `162 / nblock` to divide
/// evenly, but in practice 6 / 9 add little above 3 on real WSPR
/// channels; we stop at 3 to keep the search tree small.
/// Sync-score correlation on a precomputed `IsQs` table. Sums the
/// magnitudes of tones consistent with `WSPR_SYNC_VECTOR` minus the
/// off-tones (matching wsprd's `sync_and_demodulate` output `sync1`).
/// Used by the mode-0 lag refine and mode-1 freq refine in
/// `decode_at_baseband_nblocks` to pick the best alignment WITHOUT
/// running Fano per cell — Fano is then run once at the final
/// refined alignment.
pub fn sync_score_isqs(isqs: &IsQs) -> f32 {
    let mut ss = 0.0f32;
    let mut pow = 0.0f32;
    for i in 0..N_SYMBOLS {
        // Magnitudes for the 4 tones at symbol i.
        let m: [f32; 4] =
            core::array::from_fn(|t| (isqs.is[t][i].powi(2) + isqs.qs[t][i].powi(2)).sqrt());
        let pr3 = WSPR_SYNC_VECTOR[i] as f32;
        // wsprd `wsprd.c:280` style: ss += (2·pr3 - 1) · ((p1+p3) - (p0+p2))
        ss += (2.0 * pr3 - 1.0) * ((m[1] + m[3]) - (m[0] + m[2]));
        pow += m[0] + m[1] + m[2] + m[3];
    }
    if pow > 0.0 { ss / pow } else { 0.0 }
}

pub fn nblock_bit_metrics(isqs: &IsQs, nblock: usize) -> [f32; N_SYMBOLS] {
    debug_assert!(matches!(nblock, 1..=3));
    if nblock == 1 {
        return nblock1_bit_metrics(isqs);
    }
    let nseq = 1usize << nblock;
    let mut bm = [0.0f32; N_SYMBOLS];
    let mut p = vec![0.0f32; nseq];

    let mut i = 0usize;
    while i + nblock <= N_SYMBOLS {
        for j in 0..nseq {
            let mut xi = 0.0f32;
            let mut xq = 0.0f32;
            let mut cm = 1.0f32;
            let mut sm = 0.0f32;
            for ib in 0..nblock {
                // bit `ib` (MSB-first) of `j` is the data bit for symbol i+ib.
                let b = (j >> (nblock - 1 - ib)) & 1;
                let itone = WSPR_SYNC_VECTOR[i + ib] as usize + 2 * b;
                let is_t = isqs.is[itone][i + ib];
                let qs_t = isqs.qs[itone][i + ib];
                xi += is_t * cm + qs_t * sm;
                xq += qs_t * cm - is_t * sm;
                let cf_t = isqs.cf[itone][i + ib];
                let sf_t = isqs.sf[itone][i + ib];
                let cmp = cf_t * cm - sf_t * sm;
                let smp = sf_t * cm + cf_t * sm;
                cm = cmp;
                sm = smp;
            }
            p[j] = (xi * xi + xq * xq).sqrt();
        }
        for ib in 0..nblock {
            let imask = 1usize << (nblock - 1 - ib);
            let mut xm1 = 0.0f32;
            let mut xm0 = 0.0f32;
            for j in 0..nseq {
                if (j & imask) != 0 {
                    if p[j] > xm1 {
                        xm1 = p[j];
                    }
                } else if p[j] > xm0 {
                    xm0 = p[j];
                }
            }
            // LLR convention: positive → bit 0 more likely.
            bm[i + ib] = xm0 - xm1;
        }
        i += nblock;
    }
    // Tail symbols (when 162 isn't divisible by nblock): fall back to
    // nblock=1 metrics for the remaining symbols. nblock=2 leaves 0
    // tail (162/2=81), nblock=3 leaves 0 tail (162/3=54). So this is a
    // no-op for the supported nblock values, but kept for safety.
    while i < N_SYMBOLS {
        let sync = WSPR_SYNC_VECTOR[i] as usize;
        let p0 = (isqs.is[sync][i].powi(2) + isqs.qs[sync][i].powi(2)).sqrt();
        let p1 = (isqs.is[sync + 2][i].powi(2) + isqs.qs[sync + 2][i].powi(2)).sqrt();
        bm[i] = p0 - p1;
        i += 1;
    }
    // z-score normalise + LLR_SCALE, identical to nblock=1 path.
    let n = N_SYMBOLS as f32;
    let mean = bm.iter().sum::<f32>() / n;
    let mean_sq = bm.iter().map(|x| x * x).sum::<f32>() / n;
    let var = mean_sq - mean * mean;
    let sig = if var > 0.0 {
        var.sqrt()
    } else {
        mean_sq.sqrt()
    };
    if sig > 0.0 {
        for x in bm.iter_mut() {
            *x /= sig;
        }
    }
    const LLR_SCALE: f32 = 2.83;
    for x in bm.iter_mut() {
        *x *= LLR_SCALE;
    }
    bm
}

/// Bit metrics from an already-decimated 375 Hz baseband at the
/// supplied (`f0_audio_hz`, `lag_audio`, `drift_hz`). Use this when
/// running many candidates against the same audio — call
/// [`super::baseband::decimate_to_baseband`] once, then this for each
/// candidate.
pub fn bit_metrics_from_baseband(
    idat: &[f32],
    qdat: &[f32],
    f0_audio_hz: f32,
    lag_audio: i32,
    drift_hz: f32,
) -> [f32; N_SYMBOLS] {
    bit_metrics_from_baseband_nblock(idat, qdat, f0_audio_hz, lag_audio, drift_hz, 1)
}

/// Coherent-block-sum variant of [`bit_metrics_from_baseband`].
/// `nblock ∈ {1, 2, 3}`. nblock = 1 matches the original noncoherent
/// per-symbol path.
pub fn bit_metrics_from_baseband_nblock(
    idat: &[f32],
    qdat: &[f32],
    f0_audio_hz: f32,
    lag_audio: i32,
    drift_hz: f32,
    nblock: usize,
) -> [f32; N_SYMBOLS] {
    let f0_baseband_hz = f0_audio_hz - super::baseband::CENTER_HZ;
    // Audio rate / baseband rate ratio is 12000 / 375 = 32; lag scales accordingly.
    let lag_baseband = lag_audio / 32;
    let isqs = tone_amplitudes(idat, qdat, f0_baseband_hz, lag_baseband, drift_hz);
    nblock_bit_metrics(&isqs, nblock)
}

/// Convenience: full 162-bit-metric pipeline at nblock=1 from
/// 12 kHz audio. Decimate → tone amplitudes at the requested
/// (`f0_audio_hz`, `lag_audio`) → bit metrics.
///
/// **Performance note**: this decimates inside the call. For the
/// hot path (multi-candidate `decode_scan`), use
/// [`bit_metrics_from_baseband`] with a cached
/// `(idat, qdat)` instead.
pub fn bit_metrics_from_audio(
    audio: &[f32],
    f0_audio_hz: f32,
    lag_audio: i32,
    drift_hz: f32,
) -> [f32; N_SYMBOLS] {
    let (idat, qdat) = super::baseband::decimate_to_baseband(audio);
    bit_metrics_from_baseband(&idat, &qdat, f0_audio_hz, lag_audio, drift_hz)
}

#[allow(dead_code)]
fn _silence_unused_imports() {
    let _ = vec![0u8];
    let _: Vec<u8> = Vec::new();
}

#[cfg(test)]
mod bitmetric_tests {
    use super::*;

    /// Build an `IsQs` where every symbol carries the same *relative*
    /// tone contrast but wildly different absolute amplitude.
    fn contrast_isqs() -> IsQs {
        let mut z = IsQs {
            is: [[0.0; N_SYMBOLS]; 4],
            qs: [[0.0; N_SYMBOLS]; 4],
            cf: [[0.0; N_SYMBOLS]; 4],
            sf: [[0.0; N_SYMBOLS]; 4],
        };
        for i in 0..N_SYMBOLS {
            // Amplitude swings 1x .. 100x across the frame — a stand-in
            // for the deep fading `bitbybit` exists to handle.
            let amp = 1.0 + (i as f32) * 0.6;
            let sync = WSPR_SYNC_VECTOR[i] as usize;
            // data = 0 twice as strong as data = 1, at every symbol.
            z.is[sync][i] = 2.0 * amp;
            z.is[sync + 2][i] = amp;
        }
        z
    }

    /// wsprd's `bitbybit` divides each symbol's metric by the larger of
    /// its two tone powers (`wsprd.c:465-468`), turning an absolute
    /// difference into a relative one. With constant tone contrast and
    /// varying amplitude, that must flatten the metric vector — which
    /// the plain metric cannot do, because it tracks amplitude.
    ///
    /// Both rungs run through this module's own z-score pass
    /// afterwards, so compare *spread*, not absolute values: the
    /// z-score removes overall scale but cannot remove symbol-to-symbol
    /// amplitude variation, which is precisely why the rung exists.
    #[test]
    fn bitbybit_normalises_away_per_symbol_amplitude() {
        let z = contrast_isqs();
        let plain = nblock1_bit_metrics_opt(&z, false);
        let bybit = nblock1_bit_metrics_opt(&z, true);

        let spread = |v: &[f32; N_SYMBOLS]| {
            let min = v.iter().cloned().fold(f32::INFINITY, f32::min);
            let max = v.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            max - min
        };

        // Plain: metric tracks the 1x..~97x amplitude ramp.
        assert!(
            spread(&plain) > 1.0,
            "plain metric should track amplitude, spread was {}",
            spread(&plain)
        );
        // bitbybit: p0 = 2·p1 at every symbol, so every relative metric
        // is (2a − a) / 2a = 0.5 regardless of `a` — a flat vector.
        assert!(
            spread(&bybit) < 1e-4,
            "bitbybit should flatten the metric, spread was {}",
            spread(&bybit)
        );
    }

    /// `nblock1_bit_metrics` must stay exactly the un-normalised rung.
    #[test]
    fn plain_wrapper_matches_opt_false() {
        let z = contrast_isqs();
        assert_eq!(nblock1_bit_metrics(&z), nblock1_bit_metrics_opt(&z, false));
    }
}
