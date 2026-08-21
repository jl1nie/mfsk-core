//! `SpectrogramBuilder` must be bit-identical to `compute_spectra`.
//!
//! The builder exists so a receiver can complete spectrogram rows as
//! audio arrives instead of waiting for the whole slot (issue #307 —
//! on FST4-60 wideband that is 2842 ms moved off the post-slot decode
//! budget). That is only sound if the streaming result is *exactly*
//! what the batch path would have produced, so this pins equality of
//! the raw f32 bins rather than a tolerance: same FFT, same window,
//! same fftshift, same zero-fill past end-of-stream.
//!
//! Block boundaries must also be irrelevant — a receiver's block size
//! is set by its audio transport, not by the DSP — so the same input is
//! fed at several chunk sizes, including ones that deliberately fall
//! mid-row and mid-overlap.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use mfsk_core::engine::sync::{AudioSource, RxGrid, SpectrogramBuilder, compute_spectra};
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::grid_for;

/// Deterministic complex baseband with structure at several rates, so a
/// row-indexing or overlap-retention bug shows up as a real difference
/// rather than being hidden by a flat signal.
fn synth(n: usize) -> (Vec<f32>, Vec<f32>) {
    let mut xi = Vec::with_capacity(n);
    let mut xq = Vec::with_capacity(n);
    for k in 0..n {
        let t = k as f32;
        let re = (t * 0.017).sin() * 0.6 + (t * 0.31).cos() * 0.25 + (t * 0.0031).sin() * 0.4;
        let im = (t * 0.019).cos() * 0.55 + (t * 0.29).sin() * 0.2 + (t * 0.0027).cos() * 0.35;
        xi.push(re);
        xq.push(im);
    }
    (xi, xq)
}

#[test]
fn spectrogram_builder_matches_compute_spectra() {
    let grid_desc = grid_for(2900.0);
    let grid = RxGrid::complex(grid_desc.fs_c, 1550.0);

    // Roughly a 60 s FST4-60 slot at the wideband DDC's own output rate,
    // which is the case this was built for.
    let n = (grid_desc.fs_c * 60.0) as usize;
    let (xi, xq) = synth(n);

    let bin_lo = 84usize;
    let bin_hi = 1964usize;

    let want = compute_spectra::<Fst4s60>(AudioSource::Complex(&xi, &xq), bin_lo, bin_hi, grid);

    // Chunk sizes chosen to land badly on purpose: 1 (pathological),
    // a prime, one just under and one just over `nstep`, one that is a
    // whole number of rows, and the whole slot in one push.
    for chunk in [1usize, 7, 511, 512, 513, 1024, 4096, n] {
        let mut b = SpectrogramBuilder::new::<Fst4s60>(bin_lo, bin_hi, grid);
        let mut off = 0usize;
        while off < n {
            let end = (off + chunk).min(n);
            b.push(&xi[off..end], &xq[off..end]);
            off = end;
        }
        let got = b.finish();

        assert_eq!(got.n_freq, want.n_freq, "chunk {chunk}: n_freq");
        assert_eq!(got.n_time, want.n_time, "chunk {chunk}: n_time");
        assert_eq!(
            got.freq_offset(),
            want.freq_offset(),
            "chunk {chunk}: freq_offset"
        );

        // Bit-identical, not approximate — see the module doc comment.
        let a = want.avg_power_per_bin();
        let c = got.avg_power_per_bin();
        assert_eq!(a.len(), c.len(), "chunk {chunk}: avg_power len");
        for (i, (x, y)) in a.iter().zip(c.iter()).enumerate() {
            assert_eq!(
                x.to_bits(),
                y.to_bits(),
                "chunk {chunk}: avg_power_per_bin[{i}] {x} != {y}"
            );
        }
    }
}

/// A stream that stops short of the full slot must zero-fill the
/// remaining rows exactly as `compute_spectra` does for a short input —
/// the case a real receiver hits when capture is cut off.
#[test]
fn spectrogram_builder_zero_fills_a_short_stream() {
    let grid_desc = grid_for(2900.0);
    let grid = RxGrid::complex(grid_desc.fs_c, 1550.0);
    let n = (grid_desc.fs_c * 12.0) as usize; // deliberately far short
    let (xi, xq) = synth(n);
    let (bin_lo, bin_hi) = (100usize, 400usize);

    let want = compute_spectra::<Fst4s60>(AudioSource::Complex(&xi, &xq), bin_lo, bin_hi, grid);

    let mut b = SpectrogramBuilder::new::<Fst4s60>(bin_lo, bin_hi, grid);
    b.push(&xi, &xq);
    let got = b.finish();

    let a = want.avg_power_per_bin();
    let c = got.avg_power_per_bin();
    for (i, (x, y)) in a.iter().zip(c.iter()).enumerate() {
        assert_eq!(x.to_bits(), y.to_bits(), "avg_power_per_bin[{i}]");
    }
}
