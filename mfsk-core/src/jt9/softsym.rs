//! WSJT-X-faithful JT9 demodulator pipeline.
//!
//! Direct port of `lib/softsym.f90` and the subroutines it calls
//! (`downsam9`, `peakdt9`, `afc9`, `twkfreq`, `symspec2`,
//! `interleave9`). Replaces the `baseband.rs` + `demod_bb.rs`
//! box-car path. The decisive step is **`downsam9`**: a single
//! NFFT1=653184-point FFT of the entire 60-s slot, from which we
//! select NFFT2=1512 bins centred at the candidate carrier and
//! IFFT back to a 27.78-Hz complex baseband. That brick-wall band
//! selection rejects the wide-band noise that the box-car path
//! drags into our LLRs.

use num_complex::Complex;
use rustfft::FftPlanner;

use super::interleave::deinterleave_llrs;
use super::sync_pattern::JT9_ISYNC;

/// Big FFT length — covers ~54.43 s of 12 kHz audio. Chosen so that
/// `NFFT1 / NFFT2 = 432 = 8 × 54` is an integer decimation factor and
/// `NFFT2 / NSPSD = 1512 / 16` ≈ 94.5 symbols straddles the 85-symbol
/// JT9 frame plus comfortable pre/post buffer.
pub const NFFT1: usize = 653_184;
pub const NFFT2: usize = 1512;
/// Samples per symbol at the 27.78 Hz downsampled rate.
pub const NSPSD: usize = 16;
/// 85 symbols × 16 samples — the per-candidate signal slice.
pub const NZ3: usize = 1360;
/// Decimation factor: NFFT1 / NFFT2 = 432.
pub const NDOWN: usize = NFFT1 / NFFT2;
/// Sample rate of the downsampled signal: 12000 / 432 = 27.778 Hz.
pub const FSAMPLE_DOWN: f32 = 12_000.0 / NDOWN as f32;
/// JT9 tone spacing in Hz at 12 kHz.
pub const TONE_SPACING: f32 = 12_000.0 / 6912.0;

const SCALE: f32 = 10.0;
/// LLR clamp matching WSJT-X (it uses int8 = ±127).
const LLR_CLAMP: f32 = 127.0;

/// Pre-computed audio FFT, reused across many candidate frequencies.
///
/// The big FFT is the dominant cost; once `c1` is built it can be
/// re-used for every coarse-search candidate without re-FFT.
pub struct AudioFft {
    /// Half-spectrum (NFFT1/2 + 1 complex bins) of the input audio.
    pub c1: Vec<Complex<f32>>,
    /// 1 Hz-resolution power envelope (5000 entries: 0..5000 Hz).
    pub envelope: Vec<f32>,
}

impl AudioFft {
    /// Build the big FFT once for the whole slot.
    pub fn build(audio: &[f32]) -> Self {
        // Pad/truncate to NFFT1; scale to int16-equivalent so noise
        // estimates land in WSJT-X's calibrated regime (downsam9
        // ingests int16 samples directly).
        //
        // `audio` is real-valued, and only the first `NFFT1/2 + 1` bins
        // of its spectrum are ever used below (the old code ran a full
        // `NFFT1`-point complex-to-complex FFT via `rustfft` and threw
        // away the upper (redundant, Hermitian-symmetric) half) —
        // `realfft` computes exactly those same `NFFT1/2 + 1` complex
        // values directly, in roughly half the work. Measured as ~57%
        // of `decode_scan`'s wall time before this fix
        // (`jt9::tests::phase_breakdown_diag`).
        let mut real_planner = realfft::RealFftPlanner::<f32>::new();
        let r2c = real_planner.plan_fft_forward(NFFT1);
        let mut indata = r2c.make_input_vec();
        let n = audio.len().min(NFFT1);
        for i in 0..n {
            indata[i] = audio[i] * 32_768.0;
        }
        let mut buf = r2c.make_output_vec();
        r2c.process(&mut indata, &mut buf)
            .expect("fixed NFFT1-length input/output, shape always matches");

        // 1 Hz-resolution power envelope across 0..5 kHz.
        let df1 = 12_000.0 / NFFT1 as f32;
        let nadd = (1.0 / df1).round() as usize;
        let env_len = 5000usize;
        let mut envelope = vec![0.0f32; env_len];
        for i in 0..env_len {
            let j_start = ((i as f32) / df1).round() as usize;
            // `j = j_start + n_off` is monotonic in `n_off`, so the
            // `j < buf.len()` check only ever matters once, at the top
            // of the range — clamp `n_off`'s bound once instead of
            // re-checking every one of the `nadd` inner iterations.
            let n_off_max = nadd.min(buf.len().saturating_sub(j_start));
            for n_off in 0..n_off_max {
                envelope[i] += buf[j_start + n_off].norm_sqr();
            }
        }

        Self { c1: buf, envelope }
    }

    /// `downsam9`: extract a 27.78 Hz complex baseband centred at `fpk` Hz.
    /// Returns NFFT2 = 1512 complex samples.
    pub fn downsam9(&self, fpk: f32) -> Vec<Complex<f32>> {
        let df1 = 12_000.0 / NFFT1 as f32;
        let i0 = (fpk / df1) as i64;
        let nh2 = (NFFT2 / 2) as i64;

        // 40th-percentile noise floor in a ±100 Hz window around fpk.
        let nf = fpk.round() as i64;
        let ia = (nf - 100).max(1) as usize;
        let ib = (nf + 100).min(self.envelope.len() as i64 - 1) as usize;
        let mut env_slice: Vec<f32> = self.envelope[ia..=ib].to_vec();
        env_slice.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let pcl = env_slice.len() * 40 / 100;
        let avenoise = env_slice[pcl.min(env_slice.len() - 1)].max(1e-6);
        let fac = (1.0 / avenoise).sqrt();

        // Place selected bins into c2 with FFT-shift convention so that
        // the IFFT yields a baseband centred at fpk. `i in 0..=nh2` maps
        // to `j = i0+i` and `i in (nh2, NFFT2)` maps to `j = i0+i-NFFT2`
        // — each piece is a separately-monotonic (contiguous) run, so
        // clamp each piece's valid `i` range to `0 <= j < c1_len` once
        // instead of re-checking the bound on every one of the NFFT2
        // iterations. `i`s outside both ranges leave `c2[i]` at its
        // zero-init value, matching the original per-element skip.
        let mut c2 = vec![Complex::new(0.0, 0.0); NFFT2];
        let c1_len = self.c1.len() as i64;
        let nfft2 = NFFT2 as i64;

        // Piece 1: i in [0, nh2], j = i0 + i.
        let p1_lo = (-i0).clamp(0, nh2 + 1);
        let p1_hi = (c1_len - i0).clamp(0, nh2 + 1);
        for i in p1_lo..p1_hi {
            let j = (i0 + i) as usize;
            c2[i as usize] = self.c1[j] * fac;
        }

        // Piece 2: i in (nh2, NFFT2), j = i0 + i - NFFT2.
        let p2_lo = (nfft2 - i0).clamp(nh2 + 1, nfft2);
        let p2_hi = (c1_len - i0 + nfft2).clamp(nh2 + 1, nfft2);
        for i in p2_lo..p2_hi {
            let j = (i0 + i - nfft2) as usize;
            c2[i as usize] = self.c1[j] * fac;
        }

        // IFFT to time domain.
        let mut planner = FftPlanner::<f32>::new();
        let ifft = planner.plan_fft_inverse(NFFT2);
        let mut scratch = vec![Complex::new(0.0, 0.0); ifft.get_inplace_scratch_len()];
        ifft.process_with_scratch(&mut c2, &mut scratch);
        c2
    }
}

/// `peakdt9`: integrated symbol-power score across 85 sliding-window
/// positions, ratio of sync-symbol power to data-symbol power.
///
/// Returns `(lagpk, score, c3)` — the best lag, the
/// (sync_avg/data_avg − 1) score, and the 1360-sample slice
/// extracted starting from that lag (zero-padded outside the input).
pub fn peakdt9(c2: &[Complex<f32>]) -> (i64, f32, Vec<Complex<f32>>) {
    assert_eq!(c2.len(), NFFT2);

    // Sliding-window coherent sum over `NSPSD` samples → integrated
    // power at each lag. WSJT-X scales by 1e-3 to keep magnitudes
    // away from f32 overflow when downsam9 amplified them.
    //
    // Running sum instead of recomputing the up-to-`NSPSD`-wide window
    // from scratch at every `i` (O(NFFT2·NSPSD) → O(NFFT2)): the
    // window's `lo(i) = max(0, i-(NSPSD-1))` bound means each step
    // either grows the window by one sample (the first `NSPSD-1`
    // steps, `lo` pinned at 0) or slides it by one (`lo`/`hi` both
    // advance together thereafter) — either way, exactly one sample
    // enters (`c2[i]`) and at most one leaves (`c2[i-NSPSD]`, once
    // `i >= NSPSD`) per step.
    let mut p = vec![0.0f32; NFFT2 + 5 * NSPSD];
    let i0 = 5 * NSPSD;
    let mut z = Complex::new(0.0f32, 0.0);
    for i in 0..NFFT2 {
        z += c2[i];
        if i >= NSPSD {
            z -= c2[i - NSPSD];
        }
        p[i0 + i] = (z * 1e-3).norm_sqr();
    }

    // Lag bounds match WSJT-X getlags for nsps8=864 (NSPSD=16):
    //   lag0=123, lag1=39, lag2=291. Search lag1..=lag2.
    let lag0: i64 = 123;
    let lag1: i64 = 39;
    let lag2: i64 = 291;
    let mut smax = f32::NEG_INFINITY;
    let mut lagpk = lag0;
    for lag in lag1..=lag2 {
        let mut sum0 = 0.0f32;
        let mut sum1 = 0.0f32;
        // `idx = sym*NSPSD + lag` is monotonic increasing in `sym` and
        // never negative (`lag >= lag1 = 39 > 0`), so the only edge
        // that can trigger is the upper one, only for `sym` near 85 at
        // large `lag` — clamp `sym`'s range once instead of checking
        // `idx` every one of the 85 inner iterations.
        let sym_hi = ((p.len() as i64 - lag + NSPSD as i64 - 1) / NSPSD as i64).clamp(0, 85);
        for sym in 0..sym_hi as usize {
            let idx = ((sym * NSPSD) as i64 + lag) as usize;
            let v = p[idx];
            if JT9_ISYNC[sym] == 1 {
                sum1 += v;
            } else {
                sum0 += v;
            }
        }
        if sum0 <= 0.0 {
            continue;
        }
        let ss = (sum1 / 16.0) / (sum0 / 69.0) - 1.0;
        if ss > smax {
            smax = ss;
            lagpk = lag;
        }
    }

    // Extract NZ3 samples starting at lagpk (with the WSJT-X offsetting
    // convention: c3(i) = c2(i + lagpk - i0 - NSPSD + 1)). `j` is
    // monotonic in `i` (fixed offset per call), so clamp the valid `i`
    // range once instead of checking `j`'s bounds every iteration —
    // `i` outside the range leaves `c3[i]` at its zero-init value,
    // matching the original skip.
    let offset = lagpk - i0 as i64 - NSPSD as i64 + 1;
    let i_lo = (-offset).clamp(0, NZ3 as i64) as usize;
    let i_hi = (NFFT2 as i64 - offset).clamp(0, NZ3 as i64) as usize;
    let mut c3 = vec![Complex::new(0.0, 0.0); NZ3];
    for i in i_lo..i_hi {
        let j = (i as i64 + offset) as usize;
        c3[i] = c2[j];
    }
    (lagpk, smax, c3)
}

/// Compute the WSJT-X `ss2[0..8][0..84]` table — coherent-sum power
/// per (tone, symbol) over 16-sample windows. Used both for LLRs in
/// [`symspec2`] and for the [`chkss2`] sync-quality check.
fn compute_ss2(c5: &[Complex<f32>]) -> [[f32; 85]; 9] {
    assert_eq!(c5.len(), NZ3);
    let mut ss2 = [[0.0f32; 85]; 9];
    let mut work: Vec<Complex<f32>> = c5.to_vec();
    let dphi = -2.0 * std::f32::consts::PI * TONE_SPACING / FSAMPLE_DOWN;
    let step = Complex::new(dphi.cos(), dphi.sin());

    for i in 0..9usize {
        if i >= 1 {
            let mut w = Complex::new(1.0f32, 0.0);
            for s in work.iter_mut() {
                *s = w * *s;
                w *= step;
            }
        }
        for j in 0..85usize {
            let lo = j * NSPSD;
            let mut z = Complex::new(0.0f32, 0.0);
            for k in 0..NSPSD {
                z += work[lo + k];
            }
            ss2[i][j] = z.norm_sqr();
        }
    }
    ss2
}

/// `chkss2`: average normalised tone-0 power at the 16 sync
/// positions. Mirrors `lib/chkss2.f90`. Higher = stronger sync
/// alignment; WSJT-X gates with `schk ≥ 1.5` for non-narrow decode.
pub fn chkss2(ss2: &[[f32; 85]; 9]) -> f32 {
    let mut total = 0.0f32;
    for col in ss2.iter() {
        for &v in col {
            total += v;
        }
    }
    let ave = (total / (9.0 * 85.0)).max(1e-9);
    let mut s1 = 0.0f32;
    for j in 0..85 {
        if JT9_ISYNC[j] == 1 {
            s1 += ss2[0][j] / ave - 1.0;
        }
    }
    s1 / 16.0
}

/// `symspec2`: tone-shift c5 by 1.736 Hz × i for i=0..8, coherent-sum
/// `NSPSD` samples per symbol, then compute max-log-MAP LLRs from
/// the resulting `ss3[0..7][0..69]` table. Returns 207 LLRs in
/// channel-symbol (interleaved) order.
fn symspec2_from_ss2(ss2: &[[f32; 85]; 9]) -> ([f32; 207], f32) {
    // Build ss3[0..7][0..69] = power for data-tone i+1, data-symbol m+1.
    let mut ss3 = [[0.0f32; 69]; 8];
    for i in 1..9usize {
        let mut m = 0usize;
        for j in 0..85usize {
            if JT9_ISYNC[j] == 0 {
                ss3[i - 1][m] = ss2[i][j];
                m += 1;
            }
        }
    }

    // Baseline: average of the seven non-max ss3 entries per symbol
    // (WSJT-X `ave`), plus `xsig_sum`, the running total of per-symbol
    // strongest-tone power that becomes WSJT-X's `sig` below.
    let mut ss_total = 0.0f32;
    let mut xsig_sum = 0.0f32;
    for j in 0..69 {
        let mut smax = 0.0f32;
        let mut col_sum = 0.0f32;
        for i in 0..8 {
            let v = ss3[i][j];
            if v > smax {
                smax = v;
            }
            col_sum += v;
        }
        ss_total += col_sum - smax;
        xsig_sum += smax;
    }
    let ave = (ss_total / (69.0 * 7.0)).max(1e-9);
    for col in ss3.iter_mut() {
        for v in col.iter_mut() {
            *v /= ave;
        }
    }

    // Real WSJT-X displayed SNR (`symspec2.f90:52-54`, the tail of the
    // very subroutine this function ports):
    //
    //     sig   = sig/69.              !Signal
    //     t     = max(1.0, sig - 1.0)
    //     snrdb = db(t) - 61.3
    //
    // `sig` is the mean over the 69 data symbols of that symbol's
    // strongest tone power, taken in the **raw** `ss2`/`ss3` scale —
    // note WSJT-X accumulates it *before* the `ss3 = ss3/ave`
    // normalisation two lines above, so the `- 1.0` and the `- 61.3`
    // together carry the absolute scale of the whole `downsam9` →
    // `peakdt9` → coherent-sum chain, not a noise-relative ratio.
    // That makes this the one protocol whose SNR port is only valid
    // because the upstream chain is itself scale-faithful; verified by
    // instrumenting a real local `jt9` build's own `symspec2.f90` with
    // a `write(0,...)` probe on `WSJT-X/samples/JT9/130418_1742.wav`
    // and confirming its `sig` lands in the same magnitude range this
    // port produces (real: 4.2e3-9.6e5 across the file's five decodes;
    // ours: same range), so no rescaling term is needed or wanted.
    //
    // Supersedes a raw signal/noise power ratio that was explicitly
    // documented as *not* WSJT-X-referenced ("use it to compare JT9
    // decodes against each other, not against FT8/JT65/WSPR/Q65's
    // dB2500-referenced values"). That caveat rested on the real
    // formula being unavailable — it is `symspec2.f90:54`, three
    // lines past where this port previously stopped (issue #255).
    //
    // No clamping: `jt9_decode.f90:148` is a bare `nsnr=nint(snrdb)`,
    // with none of the `[-30,-1]` clamping JT65's own path applies
    // (`jt65_decode.f90:254-255`). `max(1.0, ...)` inside makes the
    // floor -61.3 dB, which is far enough below any real decode to
    // never bind.
    let sig = xsig_sum / 69.0;
    let snr_db = 10.0 * (sig - 1.0).max(1.0).log10() - 61.3;

    // Max-log-MAP LLRs. WSJT-X convention: positive ⇒ bit=1 likely.
    // We adopt the OPPOSITE sign (positive ⇒ bit=0) to stay
    // consistent with the rest of mfsk-core's FEC pipeline.
    let mut out_207 = [0.0f32; 207];
    let mut k = 0usize;
    for j in 0..69usize {
        for m in (0..3i32).rev() {
            let (r1, r0) = match m {
                2 => (
                    [ss3[4][j], ss3[5][j], ss3[6][j], ss3[7][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                    [ss3[0][j], ss3[1][j], ss3[2][j], ss3[3][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                ),
                1 => (
                    [ss3[2][j], ss3[3][j], ss3[4][j], ss3[5][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                    [ss3[0][j], ss3[1][j], ss3[6][j], ss3[7][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                ),
                _ => (
                    [ss3[1][j], ss3[2][j], ss3[4][j], ss3[7][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                    [ss3[0][j], ss3[3][j], ss3[5][j], ss3[6][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                ),
            };
            // Flip sign so positive ⇒ bit=0, matching mfsk-core's
            // FEC sign convention.
            let llr = SCALE * (r0 - r1);
            out_207[k] = llr.clamp(-LLR_CLAMP, LLR_CLAMP);
            k += 1;
        }
    }

    (out_207, snr_db)
}

/// Full pipeline: feed `c5` into `symspec2`, deinterleave the
/// resulting 207 LLRs in place, drop the padding bit, and return
/// the 206 LLRs ready for `ConvFano232::decode_soft`. Also returns
/// the [`chkss2`] sync-quality score so callers can apply the
/// WSJT-X two-stage gate before invoking the Fano decoder, and a
/// decode-side SNR estimate (dB) — see the doc comment inside
/// [`symspec2_from_ss2`] for the formula and its calibration caveat.
pub fn llrs_from_c5(c5: &[Complex<f32>]) -> (f32, [f32; 206], f32) {
    let ss2 = compute_ss2(c5);
    let schk = chkss2(&ss2);
    let (s207, snr_db) = symspec2_from_ss2(&ss2);
    let mut s206 = [0f32; 206];
    s206.copy_from_slice(&s207[..206]);
    deinterleave_llrs(&mut s206);
    (schk, s206, snr_db)
}

/// `twkfreq` (polynomial WSJT-X form): apply
/// `dphi(k) = (a0 + x*a1 + (1.5 x² − 0.5)*a2) * 2π/fs` where
/// `x = 2*(k − (N+1)/2) / N` runs over [−1, +1] across the buffer.
/// `a0` is the constant frequency offset in Hz, `a1` is linear drift
/// (Hz across half the buffer, matching WSJT-X's
/// "Hz/(0.5·TxT)" units), and `a2` is the parabolic chirp term used
/// by `afc9` for sync-power optimisation.
///
/// Sign matches WSJT-X `lib/twkfreq.f90`: positive `a0` shifts the
/// signal **up** in frequency.
pub fn twkfreq_poly(buf: &mut [Complex<f32>], a: [f32; 3]) {
    let n = buf.len() as f32;
    let x0 = 0.5 * (n + 1.0);
    let s = 2.0 / n;
    let two_pi_over_fs = 2.0 * std::f32::consts::PI / FSAMPLE_DOWN;
    let mut w = Complex::new(1.0f32, 0.0);
    for (i, slot) in buf.iter_mut().enumerate() {
        // Fortran is 1-indexed (i = 1..N) — match exactly.
        let xi = s * ((i as f32 + 1.0) - x0);
        let p2 = 1.5 * xi * xi - 0.5;
        let dphi = (a[0] + xi * a[1] + p2 * a[2]) * two_pi_over_fs;
        let wstep = Complex::new(dphi.cos(), dphi.sin());
        w *= wstep;
        *slot = w * *slot;
    }
}

/// `shft`: integer-sample circular shift of `c3a` by `n` samples,
/// zero-filling the wraparound region. Mirrors
/// `lib/afc9.f90::shft`.
///
/// For `n > 0`, samples shift toward lower indices (Fortran `cshift`
/// convention) and the last `n` samples are zeroed. For `n < 0`,
/// samples shift toward higher indices and the first `|n|` samples
/// are zeroed.
fn shft(c3a: &[Complex<f32>], n: i32) -> Vec<Complex<f32>> {
    let len = c3a.len();
    let mut c3 = vec![Complex::new(0.0, 0.0); len];
    if n == 0 {
        c3.copy_from_slice(c3a);
        return c3;
    }
    let abs_n = n.unsigned_abs() as usize;
    if abs_n >= len {
        return c3; // entirely zeroed
    }
    if n > 0 {
        // c3[i] = c3a[i + n]; last n entries zero.
        c3[..len - abs_n].copy_from_slice(&c3a[abs_n..]);
    } else {
        // c3[i] = c3a[i - |n|]; first |n| entries zero.
        c3[abs_n..].copy_from_slice(&c3a[..len - abs_n]);
    }
    c3
}

/// `fchisq`: WSJT-X-faithful chi-square objective for `afc9`.
/// Applies the polynomial `twkfreq` mix to `c3` and returns the
/// negated sync power (`−sum1/10000`) — smaller is better. Mirrors
/// `lib/fchisq.f90`.
fn fchisq(c3: &[Complex<f32>], a: [f32; 3]) -> f32 {
    let mut work: Vec<Complex<f32>> = c3.to_vec();
    twkfreq_poly(&mut work, a);
    let mut sum1 = 0.0f32;
    for j in 0..85usize {
        let lo = j * NSPSD;
        let mut z = Complex::new(0.0f32, 0.0);
        for k in 0..NSPSD {
            z += work[lo + k];
        }
        if JT9_ISYNC[j] == 1 {
            sum1 += z.norm_sqr();
        }
    }
    -sum1 / 10_000.0
}

/// AFC result from [`afc9`].
#[derive(Copy, Clone, Debug)]
#[allow(dead_code)]
pub struct Afc9Result {
    /// Constant frequency offset, in Hz. Sign follows WSJT-X
    /// convention: subtract from the assumed `fpk` to get the
    /// corrected carrier (`freq = fpk − a0`).
    pub a0: f32,
    /// Linear drift parameter. WSJT-X reports `drift = −2·a1` Hz
    /// per (TxT/2).
    pub a1: f32,
    /// Final integer-sample time shift that was applied to `c3a`.
    pub time_shift: i32,
    /// Final `−chisqr` value (= `sum1/10000` of the optimised
    /// alignment). Higher = stronger sync.
    pub syncpk: f32,
}

/// `afc9`: 3-parameter chi-square AFC over (frequency, drift,
/// integer-sample time shift) using WSJT-X's parabolic line search.
/// Mutates `c3a` in place to apply the discovered integer time shift,
/// matching `lib/afc9.f90` (where `c3a=c3` at exit).
///
/// The caller should subsequently apply [`twkfreq_poly`] with
/// `[a0, a1, 0]` (i.e. clearing the chirp parameter) to mix down the
/// signal before symbol detection — same flow as
/// `lib/softsym.f90:36-44`.
pub fn afc9(c3a: &mut Vec<Complex<f32>>) -> Afc9Result {
    let mut a = [0.0f32; 3];
    let mut deltaa = [TONE_SPACING, TONE_SPACING, 1.0f32];
    let nterms = 3usize;

    // `a3_applied` tracks the integer shift currently baked into
    // `c3` — we re-run shft only when `nint(a[2])` changes.
    let mut c3 = c3a.clone();
    let mut a3_applied = 0.0f32;

    let mut chisqr = 0.0f32;
    let mut chisqr0 = 1.0e6f32;

    for _iter in 0..4 {
        for j in 0..nterms {
            // Re-shift c3 from c3a if a[2] changed.
            if (a[2] - a3_applied).abs() > f32::EPSILON {
                a3_applied = a[2];
                c3 = shft(c3a, a3_applied.round() as i32);
            }
            let mut chisq1 = fchisq(&c3, a);
            let mut fn_count = 0.0f32;
            let mut delta = deltaa[j];
            // Loop label 10: step until chisq2 != chisq1.
            let mut chisq2;
            loop {
                a[j] += delta;
                if j == 2 && (a[2] - a3_applied).abs() > f32::EPSILON {
                    a3_applied = a[2];
                    c3 = shft(c3a, a3_applied.round() as i32);
                }
                chisq2 = fchisq(&c3, a);
                if chisq2 != chisq1 {
                    break;
                }
            }
            // If we stepped uphill, reverse direction.
            if chisq2 > chisq1 {
                delta = -delta;
                a[j] += delta;
                core::mem::swap(&mut chisq1, &mut chisq2);
            }
            // Loop label 20: continue stepping while chisq3 < chisq2.
            let mut chisq3;
            loop {
                fn_count += 1.0;
                a[j] += delta;
                if j == 2 && (a[2] - a3_applied).abs() > f32::EPSILON {
                    a3_applied = a[2];
                    c3 = shft(c3a, a3_applied.round() as i32);
                }
                chisq3 = fchisq(&c3, a);
                if chisq3 < chisq2 {
                    chisq1 = chisq2;
                    chisq2 = chisq3;
                    continue;
                }
                break;
            }
            // Parabolic minimum from the last three samples — matches
            // `lib/afc9.f90` exactly.
            let frac = 1.0 / (1.0 + (chisq1 - chisq2) / (chisq3 - chisq2)) + 0.5;
            let new_delta = delta * frac;
            a[j] -= new_delta;
            if j < 2 {
                deltaa[j] *= fn_count / 3.0;
            }
        }
        if (a[2] - a3_applied).abs() > f32::EPSILON {
            a3_applied = a[2];
            c3 = shft(c3a, a3_applied.round() as i32);
        }
        chisqr = fchisq(&c3, a);
        if chisqr0.abs() > 1e-12 && chisqr / chisqr0 > 0.99 {
            break;
        }
        chisqr0 = chisqr;
    }

    // Mirror `c3a=c3` at function exit: the integer shift is baked
    // back into the caller's buffer.
    *c3a = c3;

    Afc9Result {
        a0: a[0],
        a1: a[1],
        time_shift: a3_applied.round() as i32,
        syncpk: -chisqr,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::{DecodeContext, FecCodec, FecOpts, MessageCodec};
    use crate::fec::ConvFano232;
    use crate::jt9::tx::synthesize_standard;
    use crate::msg::{Jt72Codec, Jt72Message};

    fn load_wav(path: &std::path::Path) -> Option<Vec<f32>> {
        let bytes = std::fs::read(path).ok()?;
        let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let data = &bytes[44..44 + data_len];
        Some(
            data.chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                .collect(),
        )
    }

    /// Timing probe isolating `AudioFft::build` (once per `decode_scan`
    /// call) from `downsam9` (once per coarse-search candidate — ~50
    /// simulated here). Added while investigating an apparent JT9
    /// `decode_scan` slowdown after the branch-hoisting fix to both
    /// functions (issue: their own indexing bound checks, same shape
    /// already fixed in `engine::dsp::subtract::apply_at_offset`) —
    /// isolated per-function timing here showed **no** regression in
    /// either function (statistically indistinguishable before/after,
    /// multiple runs). A properly *interleaved* re-measurement of the
    /// full `decode_scan` end-to-end (the original signal) then also
    /// came back flat. The initial ~5% "regression" reading was a
    /// same-machine A/B methodology artifact: measuring all of one
    /// side's runs, then all of the other's, let ordinary run-to-run
    /// noise on this box look like a systematic difference — kept as a
    /// standing probe/lesson, not because these two functions turned
    /// out to need fixing.
    #[test]
    #[ignore = "manual timing probe — AudioFft::build vs downsam9, see doc comment"]
    fn probe_isolate_build_vs_downsam9() {
        use std::time::Instant;
        let path = std::path::Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        let audio = load_wav(path).expect("golden WAV must load");

        fn median(mut v: Vec<f64>) -> f64 {
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[v.len() / 2]
        }

        let _ = AudioFft::build(&audio);
        let mut build_times = Vec::with_capacity(10);
        for _ in 0..10 {
            let t0 = Instant::now();
            let f = AudioFft::build(&audio);
            build_times.push(t0.elapsed().as_secs_f64() * 1000.0);
            std::hint::black_box(&f);
        }
        eprintln!(
            "AudioFft::build: median={:.3}ms min={:.3}ms max={:.3}ms",
            median(build_times.clone()),
            build_times.iter().cloned().fold(f64::INFINITY, f64::min),
            build_times
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max),
        );

        let big = AudioFft::build(&audio);
        let freqs: Vec<f32> = (0..50).map(|i| 1050.0 + i as f32 * 10.0).collect();
        let _ = big.downsam9(freqs[0]);
        let mut ds_times = Vec::with_capacity(10);
        for _ in 0..10 {
            let t0 = Instant::now();
            for &f in &freqs {
                let c2 = big.downsam9(f);
                std::hint::black_box(&c2);
            }
            ds_times.push(t0.elapsed().as_secs_f64() * 1000.0);
        }
        eprintln!(
            "downsam9 x{}: median={:.3}ms min={:.3}ms max={:.3}ms ({:.4}ms/call)",
            freqs.len(),
            median(ds_times.clone()),
            ds_times.iter().cloned().fold(f64::INFINITY, f64::min),
            ds_times.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            median(ds_times) / freqs.len() as f64,
        );
    }

    #[test]
    fn softsym_golden_grid_roundtrips() {
        let cases = &[
            ("CQ", "GM7GAX", "IO75"),
            ("TF3G", "N7MQ", "CN84"),
            ("K1JT", "KF4RWA", "73"),
            ("CQ", "M0WAY", "IO82"),
            ("K1JT", "N5KDV", "EM41"),
        ];
        for &(c1, c2, grid) in cases {
            let audio = synthesize_standard(c1, c2, grid, 12_000, 1346.0, 0.5).expect("synth");
            let mut padded = vec![0f32; 720_000];
            let n = audio.len().min(padded.len());
            padded[..n].copy_from_slice(&audio[..n]);
            let big = AudioFft::build(&padded);
            let c2_buf = big.downsam9(1346.0);
            let (_lag, sc, c3) = peakdt9(&c2_buf);
            assert!(
                sc > 0.5,
                "sync score for {} {} {} too low: {}",
                c1,
                c2,
                grid,
                sc
            );
            let (_schk, llrs, _snr_db) = llrs_from_c5(&c3);
            let res = ConvFano232
                .decode_soft(&llrs, &FecOpts::default())
                .unwrap_or_else(|| panic!("Fano failed for {} {} {}", c1, c2, grid));
            let mut payload = [0u8; 72];
            payload.copy_from_slice(&res.info);
            let msg = Jt72Codec::default()
                .unpack(&payload, &DecodeContext::default())
                .unwrap_or_else(|| panic!("unpack failed for {} {} {}", c1, c2, grid));
            match msg {
                Jt72Message::Standard {
                    call1,
                    call2,
                    grid_or_report,
                } => {
                    assert_eq!(call1, c1);
                    assert_eq!(call2, c2);
                    assert_eq!(
                        grid_or_report, grid,
                        "grid mismatch for {} {} {}",
                        c1, c2, grid
                    );
                }
                other => panic!(
                    "expected Standard for {} {} {}, got {:?}",
                    c1, c2, grid, other
                ),
            }
        }
    }

    /// Encode a message with a CHOSEN Gray-code direction so we can
    /// hand WSJT-X two WAVs and see which one (if any) it decodes.
    /// Returns the 85 channel tones.
    fn encode_with_gray_dir(c1: &str, c2: &str, grid: &str, invert_gray: bool) -> [u8; 85] {
        use crate::engine::FecCodec;
        use crate::fec::ConvFano232;
        use crate::jt9::interleave::interleave;
        use crate::jt9::sync_pattern::JT9_ISYNC;
        use crate::msg::jt72::pack_standard;

        // Same forward-gray as production:
        let fwd_gray3 = |n: u8| -> u8 { (n ^ (n >> 1)) & 0x7 };
        // Inverse:
        let inv_gray3 = |g: u8| -> u8 {
            let mut n = g & 0x7;
            n ^= n >> 1;
            n ^= n >> 2;
            n & 0x7
        };

        let words = pack_standard(c1, c2, grid).expect("pack");
        let mut info = [0u8; 72];
        for (i, b) in info.iter_mut().enumerate() {
            *b = (words[i / 6] >> (5 - (i % 6))) & 1;
        }
        let mut cw206 = vec![0u8; 206];
        ConvFano232.encode(&info, &mut cw206);
        let mut bits206 = [0u8; 206];
        bits206.copy_from_slice(&cw206);
        interleave(&mut bits206);
        // Pad to 207 with zero (matches gen9.f90: i1ScrambledBits(207)=0)
        let mut bits207 = [0u8; 207];
        bits207[..206].copy_from_slice(&bits206);

        let mut tones = [0u8; 85];
        let mut j = 0usize;
        for (i, slot) in tones.iter_mut().enumerate() {
            if JT9_ISYNC[i] == 1 {
                *slot = 0;
            } else {
                let b0 = bits207[3 * j];
                let b1 = bits207[3 * j + 1];
                let b2 = bits207[3 * j + 2];
                let raw = (b0 << 2) | (b1 << 1) | b2;
                let gc = if invert_gray {
                    inv_gray3(raw)
                } else {
                    fwd_gray3(raw)
                };
                *slot = gc + 1;
                j += 1;
            }
        }
        tones
    }

    fn synth_audio(tones: &[u8; 85], freq: f32, amp: f32) -> Vec<f32> {
        const NSPS: usize = 6912;
        const SR: f32 = 12_000.0;
        let spacing: f32 = SR / NSPS as f32;
        let mut out = Vec::with_capacity(NSPS * 85);
        let mut phase = 0.0f32;
        for &sym in tones {
            let f = freq + sym as f32 * spacing;
            let dphi = 2.0 * std::f32::consts::PI * f / SR;
            for _ in 0..NSPS {
                out.push(amp * phase.cos());
                phase += dphi;
                if phase > 2.0 * std::f32::consts::PI {
                    phase -= 2.0 * std::f32::consts::PI;
                }
            }
        }
        out
    }

    fn write_wav(audio: &[f32], path: &str) {
        let mut bytes: Vec<u8> = Vec::with_capacity(44 + audio.len() * 2);
        let data_len = (audio.len() * 2) as u32;
        bytes.extend_from_slice(b"RIFF");
        bytes.extend_from_slice(&(36u32 + data_len).to_le_bytes());
        bytes.extend_from_slice(b"WAVE");
        bytes.extend_from_slice(b"fmt ");
        bytes.extend_from_slice(&16u32.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&12_000u32.to_le_bytes());
        bytes.extend_from_slice(&24_000u32.to_le_bytes());
        bytes.extend_from_slice(&2u16.to_le_bytes());
        bytes.extend_from_slice(&16u16.to_le_bytes());
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&data_len.to_le_bytes());
        for &s in audio {
            let v = (s * 32_767.0).round().clamp(-32_768.0, 32_767.0) as i16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        std::fs::write(path, &bytes).expect("write WAV");
    }

    /// Emit two WAVs differing only in Gray-code direction at encode.
    /// Whichever decodes in WSJT-X is the correct direction.
    #[test]
    #[ignore]
    fn dump_em41_gray_ab() {
        for (invert, label, fname) in [
            (false, "FORWARD gray (current)", "/tmp/260506_1300.wav"),
            (true, "INVERSE gray (test)", "/tmp/260506_1400.wav"),
        ] {
            let tones = encode_with_gray_dir("K1JT", "N5KDV", "EM41", invert);
            let signal = synth_audio(&tones, 1500.0, 0.3);
            let mut audio = vec![0f32; 12_000 * 60];
            let off = 1_200usize;
            let n = signal.len().min(audio.len() - off);
            audio[off..off + n].copy_from_slice(&signal[..n]);
            write_wav(&audio, fname);
            eprintln!("Wrote {} → {}", label, fname);
            eprintln!("  first 20 tones: {:?}", &tones[..20]);
        }
        eprintln!("\nDecode BOTH in WSJT-X (mode JT9). Whichever produces");
        eprintln!("'K1JT N5KDV EM41' is the WSJT-X-compatible direction.");
    }

    /// Diagnostic: write our `synthesize_standard` output for
    /// "K1JT N5KDV EM41" to a 12 kHz mono PCM-16 WAV. Hand to WSJT-X
    /// jt9 decoder to verify whether our encoder is bit-compatible
    /// with the reference.
    ///
    /// Run with: `cargo test --release --features jt9,fft-rustfft \
    ///   --lib jt9::softsym::tests::dump_synth_em41 -- --ignored --nocapture`
    /// Then point WSJT-X File→Open at `/tmp/mfsk_jt9_em41_synth.wav`.
    #[test]
    #[ignore]
    fn dump_synth_em41() {
        // 60-second slot at 12 kHz: WSJT-X expects 720_000 frames.
        let mut audio = vec![0f32; 12_000 * 60];
        // dt = +1.0 s — matches WSJT-X jt9sim's "k=12000" signal start.
        // The 130418_1742.wav golden signals also live at dt ≈ +0.86..+1.04
        // (WSJT-X's coarse search expects signals near +1 s, not +0.1 s).
        let signal =
            synthesize_standard("K1JT", "N5KDV", "EM41", 12_000, 1500.0, 0.3).expect("synth");
        let off = 12_000usize; // 1.0 s × 12 kHz
        let n = signal.len().min(audio.len() - off);
        audio[off..off + n].copy_from_slice(&signal[..n]);

        // WSJT-X parses the slot timestamp from `YYMMDD_HHMM.wav`.
        // Use a real date so File→Open accepts it and File→Open Next
        // doesn't reject the slot.
        let path = "/tmp/260506_1200.wav";
        let mut bytes: Vec<u8> = Vec::with_capacity(44 + audio.len() * 2);
        let data_len = (audio.len() * 2) as u32;
        bytes.extend_from_slice(b"RIFF");
        bytes.extend_from_slice(&(36u32 + data_len).to_le_bytes());
        bytes.extend_from_slice(b"WAVE");
        bytes.extend_from_slice(b"fmt ");
        bytes.extend_from_slice(&16u32.to_le_bytes()); // fmt chunk size
        bytes.extend_from_slice(&1u16.to_le_bytes()); // PCM
        bytes.extend_from_slice(&1u16.to_le_bytes()); // mono
        bytes.extend_from_slice(&12_000u32.to_le_bytes()); // sample rate
        bytes.extend_from_slice(&24_000u32.to_le_bytes()); // byte rate = 12k * 2
        bytes.extend_from_slice(&2u16.to_le_bytes()); // block align
        bytes.extend_from_slice(&16u16.to_le_bytes()); // bits per sample
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&data_len.to_le_bytes());
        for &s in &audio {
            let v = (s * 32_767.0).round().clamp(-32_768.0, 32_767.0) as i16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        std::fs::write(path, &bytes).expect("write WAV");
        eprintln!("Wrote noiseless 'K1JT N5KDV EM41' synth → {}", path);
        eprintln!("  carrier = 1500 Hz, dt = +1.0 s, 60 s slot, mono PCM-16 12 kHz");
        eprintln!("  amplitude = 0.3 (~ -10 dBFS peak)");
        eprintln!("  Decode with WSJT-X (mode=JT9) and report what it says.");
    }

    #[test]
    fn synth_softsym_roundtrip() {
        let freq = 1500.0f32;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        // Pad audio to >= NFFT1 / 8 samples to give downsam9 something to work with.
        let mut padded = vec![0f32; 720_000];
        let n = audio.len().min(padded.len());
        padded[..n].copy_from_slice(&audio[..n]);
        let big_fft = AudioFft::build(&padded);
        let c2 = big_fft.downsam9(freq);
        let (_lag, sc, c3) = peakdt9(&c2);
        eprintln!("synth roundtrip peakdt9 score = {}", sc);
        assert!(
            sc > 0.5,
            "sync score for clean synth should be high; got {}",
            sc
        );

        let (_schk, llrs, _snr_db) = llrs_from_c5(&c3);
        let res = ConvFano232
            .decode_soft(&llrs, &FecOpts::default())
            .expect("Fano must converge on clean synth via WSJT-X-faithful pipeline");
        let mut payload = [0u8; 72];
        payload.copy_from_slice(&res.info);
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
