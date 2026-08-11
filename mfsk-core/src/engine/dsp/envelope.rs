//! Transmit-envelope shaping shared by the continuous-phase FSK
//! transmit paths (WSPR, JT65, JT9, Q65).
//!
//! ## Why these four and not FT8/FT4/FST4
//!
//! WSJT-X generates its transmit audio two different ways, selected by
//! the sign of the `toneSpacing` argument `mainwindow.cpp` hands to
//! `Modulator::start`:
//!
//! - **Negative** — "Transmit a pre-computed, filtered waveform"
//!   (`mainwindow.cpp`'s own comment). FT8, FT4 and FST4 take this
//!   path; `gen_ft8wave.f90` / `gen_ft4wave.f90` / `gen_fst4wave.f90`
//!   apply both GFSK symbol shaping *and* a raised-cosine envelope
//!   ramp at each end before the samples ever reach the modulator.
//!   This crate matches that already, via
//!   [`super::gfsk::GfskCfg::ramp_samples`].
//! - **Positive** — plain CPFSK, generated sample-by-sample inside
//!   `Modulator::modulate` (`m_phi += m_dphi; sample = m_amp·sin(m_phi)`).
//!   WSPR, JT65, JT9 and Q65 take this path, and this crate's
//!   `wspr::tx` / `jt65::tx` / `jt9::tx` / `q65::tx` match it — no
//!   symbol shaping, deliberately (see issue #259: adding GFSK here
//!   would *deviate* from WSJT-X, which shapes none of these).
//!
//! What this crate did *not* match is the envelope. WSJT-X's modulator
//! fades the CPFSK path out at the end of every transmission
//! (`Modulator.cpp`: `i0 = (m_symbolsLength - 0.017)·4·m_nsps`, then
//! `if (m_ic > i0) m_amp = 0.98 · m_amp`), while this crate wrote
//! `amplitude · cos(phase)` from the first sample to the last. A step
//! discontinuity in the envelope is a broadband click, independent of
//! any symbol-transition shaping.
//!
//! ## Why both ends, when WSJT-X's modulator only fades out
//!
//! `Modulator::start` assigns `m_amp = numeric_limits<qint16>::max()`
//! on every transmission, so the CPFSK path genuinely begins at full
//! amplitude — there is no fade-in anywhere in it.
//!
//! That is not a deliberate choice for slow modes, which was the
//! obvious hypothesis and is refuted by WSJT-X's own longest-period
//! mode: `gen_fst4wave.f90` ramps **up** as well as down
//! (`wave(1:nsps/4) *= (1 - cos(…))/2`), unconditionally
//! (`data lshape/.true./`), and FST4's periods run 15 s to 1800 s.
//! Its ramp is `nsps/4` where FT8's is `nsps/8` — *longer* for the
//! slower mode, the opposite of avoiding it. The CPFSK path simply
//! never received the treatment the pre-computed path got when
//! FT8/FT4/FST4 were migrated to it; `mainwindow.cpp` still carries a
//! commented-out `// toneSpacing=-4.0;` beside JT65's positive value,
//! marking an abandoned migration.
//!
//! So: fading out matches WSJT-X's intent for these modes, and fading
//! in matches what WSJT-X does everywhere it generates a buffer rather
//! than streaming one. This crate's transmit functions produce a
//! buffer, which is the `gen_*wave` situation, not the modulator's.

/// Raised-cosine ramp length for the CPFSK transmit paths, in
/// milliseconds.
///
/// Bracketed by WSJT-X's own choices rather than picked freely: its
/// modulator's exponential fade-out (`0.98` per sample at 48 kHz)
/// reaches −60 dB in ≈7.1 ms, `gen_ft8wave`'s `nsps/8` is 20 ms, and
/// `gen_fst4wave`'s `nsps/4` is 81 ms at FST4-60. 10 ms sits inside
/// that range and is under 2 % of a symbol for every protocol using
/// this helper (WSPR 683 ms, JT65 372 ms, JT9 580 ms, Q65 ≥ 128 ms).
pub const RAMP_MS: f32 = 10.0;

/// Ramp length in samples for `sample_rate`, capped at `nsps/8` so a
/// hypothetical short-symbol caller can never taper a meaningful
/// fraction of its first and last symbol.
#[inline]
pub fn ramp_samples(sample_rate: u32, nsps: usize) -> usize {
    let by_time = (sample_rate as f32 * RAMP_MS / 1000.0) as usize;
    by_time.min(nsps / 8)
}

/// Apply a raised-cosine ramp-up over the first `nramp` samples of
/// `out` and a ramp-down over the last `nramp`.
///
/// Same envelope shape as `gen_ft8wave.f90:69-73` and
/// `gen_fst4wave.f90:75-80` — `(1 − cos(2πi/2N))/2` rising,
/// `(1 + cos(2πi/2N))/2` falling — so every transmit path in this
/// crate tapers identically regardless of which synthesiser produced
/// the samples.
///
/// No-ops when `nramp` is 0. `nramp` is clamped to `out.len()/2` so
/// the two ends cannot overlap on a very short buffer.
pub fn apply_ramp(out: &mut [f32], nramp: usize) {
    let n = out.len();
    let nramp = nramp.min(n / 2);
    if nramp == 0 {
        return;
    }
    let twopi = core::f32::consts::TAU;
    for i in 0..nramp {
        #[cfg(not(feature = "std"))]
        use num_traits::Float as _;
        let env = (1.0 - (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
        out[i] *= env;
    }
    let k1 = n - nramp;
    for i in 0..nramp {
        #[cfg(not(feature = "std"))]
        use num_traits::Float as _;
        let env = (1.0 + (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
        out[k1 + i] *= env;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_starts_and_ends_at_zero_and_leaves_middle_untouched() {
        let mut buf = [1.0f32; 100];
        apply_ramp(&mut buf, 10);
        assert!(
            buf[0].abs() < 1e-6,
            "first sample must be ~0, got {}",
            buf[0]
        );
        assert!(
            buf[99].abs() < 0.05,
            "last sample must be near 0, got {}",
            buf[99]
        );
        // Middle is untouched.
        for (i, &v) in buf.iter().enumerate().take(90).skip(10) {
            assert_eq!(v, 1.0, "sample {i} inside the flat region was modified");
        }
    }

    #[test]
    fn ramp_is_monotonic_on_both_ends() {
        let mut buf = [1.0f32; 200];
        apply_ramp(&mut buf, 40);
        for i in 1..40 {
            assert!(buf[i] >= buf[i - 1], "ramp-up not monotonic at {i}");
        }
        for i in 161..200 {
            assert!(buf[i] <= buf[i - 1], "ramp-down not monotonic at {i}");
        }
    }

    #[test]
    fn zero_ramp_is_identity() {
        let mut buf = [0.5f32; 20];
        apply_ramp(&mut buf, 0);
        assert!(buf.iter().all(|&v| v == 0.5));
    }

    /// The two ends must not overlap and corrupt each other even when
    /// the caller asks for a ramp longer than the buffer.
    #[test]
    fn oversized_ramp_is_clamped() {
        let mut buf = [1.0f32; 10];
        apply_ramp(&mut buf, 999);
        assert!(buf.iter().all(|v| v.is_finite()));
        assert!(buf[0].abs() < 1e-6);
    }

    #[test]
    fn ramp_samples_capped_by_symbol_length() {
        // 10 ms at 12 kHz = 120 samples, well under WSPR's 8192/8.
        assert_eq!(ramp_samples(12_000, 8192), 120);
        // Short symbol: nsps/8 wins.
        assert_eq!(ramp_samples(12_000, 400), 50);
    }
}
