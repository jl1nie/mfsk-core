//! WSJT-X-faithful 3-stage fine refinement (idt → ifr → idt).
//!
//! Direct port of `lib/ft8/ft8b.f90:104-150` of WSJT-X. Operates on the
//! complex baseband at 200 Hz (`cd0`, length [`CD0_LEN`]); the caller is
//! responsible for the initial mix-and-decimate via
//! [`crate::ft8::downsample::downsample`].
//!
//! Three stages:
//!
//! 1. ±10 idt sample search at the candidate's initial frequency, using
//!    `fine_sync_power_signed` (= sync8d-equivalent Costas correlation
//!    power summed over the 3 sync blocks).
//! 2. ±5 ifr × 0.5 Hz frequency sweep at the Stage-1 best `ibest`.
//!    Applies a 32-sample phasor tweak `exp(j·2π·delf·k·dt2)` to the
//!    *reference* Costas waveform (mirrors WSJT-X's real `ctwk`
//!    construction, `ft8b.f90:133-140`, multiplied against the cached
//!    Costas reference inside `sync8d.f90`) — `cd0` itself is never
//!    shifted. Tied score is broken by smaller |delf| (favour the
//!    original frequency).
//! 3. ±4 idt re-search, tweaked by the winning Stage-2 `delfbest`.
//!
//! The output `dt_sec` follows the host convention
//! `(ibest - 0.5) / 200`. The caller decides whether to also accept
//! sub-sample dt refinement (parabolic) on the final score; this module
//! returns integer dt for closer match to WSJT-X.

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Precomputed reference phasor table for [`fine_sync_power_signed`]'s
/// inner correlation loop: `ref_table[tone][j] = exp(i * 2π * tone/32 * j)`
/// for `tone in 0..8`, `j in 0..32`. These 8×32 = 256 values are the
/// *entire* space `fine_sync_power_signed` ever needs — `tone` is always
/// one of the 8 fixed Costas tones (`ft8::params::COSTAS`) and `j` always
/// runs 0..32 (`DS_SPB`), never anything candidate- or trial-dependent.
/// Building this once per [`fine_refine_3stage`] call instead of
/// recomputing `cos`/`sin` fresh on every one of its 41 calls (Stage
/// A/B/C combined) cuts ~27,552 transcendental-function evaluations per
/// candidate down to 256 — same values, not an approximation (issue
/// #182 follow-up).
type CostasRefTable = [[Complex<f32>; 32]; 8];

fn build_costas_ref_table() -> CostasRefTable {
    const DS_SPB: usize = 32;
    let mut table = [[Complex::new(0.0_f32, 0.0); DS_SPB]; 8];
    for (tone, row) in table.iter_mut().enumerate() {
        let dphi = core::f32::consts::TAU * (tone as f32) / (DS_SPB as f32);
        let mut phi = 0.0_f32;
        for slot in row.iter_mut() {
            *slot = Complex::new(phi.cos(), phi.sin());
            phi += dphi;
            if phi > core::f32::consts::PI {
                phi -= core::f32::consts::TAU;
            }
        }
    }
    table
}

/// Process-lifetime-cached accessor for [`build_costas_ref_table`]
/// (issue #182 follow-up) — mirrors WSJT-X `sync8d.f90`'s own
/// `data first/.true.` + `save first,twopi,csync`: the reference table
/// is computed **once for the whole process**, not once per candidate.
/// Host (`std`) builds get the full match via `OnceLock`; `no_std`
/// builds fall back to a fresh build per call (Phase 1's already-good
/// once-per-candidate — no `OnceLock` without `std`, and embedded isn't
/// this speed work's target).
#[cfg(feature = "std")]
fn costas_ref_table() -> &'static CostasRefTable {
    static TABLE: std::sync::OnceLock<CostasRefTable> = std::sync::OnceLock::new();
    TABLE.get_or_init(build_costas_ref_table)
}
#[cfg(not(feature = "std"))]
fn costas_ref_table() -> CostasRefTable {
    build_costas_ref_table()
}

/// Signed-`i` analogue of WSJT-X `sync8d`. Mirrors WSJT-X
/// `sync8d.f90:43-45`: each Costas block contributes 0 when its
/// start sample falls outside the cd0 window. This is the right
/// behaviour for a candidate whose dt < -0.5 s — block 0 sits in
/// truncated audio but blocks 1 / 2 are still valid.
///
/// `tweak`, when present, mirrors `sync8d.f90`'s `itwk=1` path
/// (`csync2=ctwk*csync2`) — applied to the 32-sample *reference*
/// waveform, not to `cd0`. This is how WSJT-X's real Stage-B/C
/// frequency sweep works: `ft8b.f90:133-146` builds a 32-element
/// `ctwk` tweak per trial and never touches the 3200-sample `cd0`
/// data buffer at all. Our earlier port instead shifted the entire
/// `cd0` per trial (~100x more arithmetic) — see [`build_tweak`]'s
/// doc comment for the full story (issue #182 follow-up).
fn fine_sync_power_signed(
    cd0: &[Complex<f32>],
    i: i32,
    ref_table: &CostasRefTable,
    tweak: Option<&[Complex<f32>; 32]>,
) -> f32 {
    use crate::ft8::params::COSTAS;
    const DS_SPB: i32 = 32;
    let icos7: [u8; 7] = [3, 1, 4, 0, 6, 5, 2];
    debug_assert_eq!(icos7.len(), COSTAS.len());
    let np2 = cd0.len() as i32;
    let mut total = 0.0_f32;
    // 3 sync blocks at symbol offsets 0, 36, 72 — see sync8.f90:34-37.
    for block_off in [0_i32, 36, 72].iter().copied() {
        let mut block_power = 0.0_f32;
        // 7 Costas tones per block.
        for (k, &tone) in icos7.iter().enumerate() {
            let start = i + block_off * DS_SPB + (k as i32) * DS_SPB;
            if start < 0 || start + DS_SPB > np2 {
                continue;
            }
            let ref_row = &ref_table[tone as usize];
            let mut z = Complex::new(0.0_f32, 0.0);
            for j in 0..DS_SPB {
                let s = cd0[start as usize + j as usize];
                let r = ref_row[j as usize];
                // `csync2 = ctwk*csync2` (Fortran) computed before the
                // `conjg` below — multiply first, conjugate the
                // product, matching sync8d.f90's actual operation
                // order (mathematically equivalent to conjugating
                // each factor separately, kept literal for traceability).
                let r = match tweak {
                    Some(t) => r * t[j as usize],
                    None => r,
                };
                z += s * r.conj();
            }
            block_power += z.norm_sqr();
        }
        total += block_power;
    }
    total
}

/// Length of `cd0` produced by the FT8 downsampler (200 Hz × 16 s).
pub const CD0_LEN: usize = 3200;

/// Downsampled sample rate (Hz).
pub const DS_RATE: f32 = 200.0;

/// Result of [`fine_refine_3stage`].
#[derive(Debug, Clone, Copy)]
pub struct FineRefine {
    /// Refined dt offset relative to the slot's nominal TX start (seconds).
    /// Equivalent to WSJT-X's `xdt = (ibest-1) * dt2` minus the 0.5 s
    /// `TX_START_OFFSET_S`, exposed as `dt_sec` for symmetry with the
    /// rest of mfsk-core.
    pub dt_sec: f32,
    /// Refined frequency offset relative to the candidate's initial
    /// `freq_hz`. The caller adds this to its initial frequency.
    pub delf_hz: f32,
    /// Final Stage-3 sync power (= max over the ±4 idt re-search).
    /// Useful for downstream gating (e.g. `nsync_quality > 6`).
    pub score: f32,
}

/// Build the 32-element frequency-tweak reference waveform
/// `exp(j·2π·delf·k·dt2)` for `k in 0..32`. Direct port of
/// `ft8b.f90:133-140`'s `ctwk` construction — **not negated**
/// (`dphi = +2π·delf·dt2`), matching the real Fortran exactly. The
/// original per-candidate `shift_freq` (issue #182 Phase 2, since
/// removed) shifted the entire ~3200-sample `cd0` buffer per trial —
/// this instead builds a tiny 32-sample tweak applied to
/// [`fine_sync_power_signed`]'s reference waveform, matching what
/// WSJT-X's real `sync8d.f90` (`csync2=ctwk*csync2`) actually does:
/// ~100x less arithmetic per trial (32 elements vs 3200), and exact
/// rather than an NCO approximation, since 32 elements is cheap enough
/// to just compute directly with `cos`/`sin`.
fn build_tweak(delf_hz: f32) -> [Complex<f32>; 32] {
    let dt2 = 1.0 / DS_RATE;
    let dphi = core::f32::consts::TAU * delf_hz * dt2;
    let mut tweak = [Complex::new(0.0_f32, 0.0); 32];
    let mut phi = 0.0_f32;
    for slot in tweak.iter_mut() {
        *slot = Complex::new(phi.cos(), phi.sin());
        phi += dphi;
        if phi > core::f32::consts::PI {
            phi -= core::f32::consts::TAU;
        }
    }
    tweak
}

/// 3-stage fine refinement. Mirrors `ft8b.f90:104-150`.
///
/// `initial_dt_sec` is the candidate's initial dt (relative to the
/// nominal TX start, i.e. with the 0.5 s offset already removed).
/// Returns the refined `(dt_sec, delf_hz, score)`. Negative dt is
/// supported via `fine_sync_power_signed`, which mirrors WSJT-X
/// `sync8d.f90:43-45` (zero contribution from any sync block whose
/// samples fall outside the cd0 window).
///
/// Stage B/C sweep frequency by tweaking `fine_sync_power_signed`'s
/// *reference* waveform (via `build_tweak`), never `cd0` itself —
/// see `build_tweak`'s doc comment for why this matters (issue #182
/// follow-up: matches real WSJT-X, ~100x less arithmetic than the
/// data-shifting approach this replaced).
pub fn fine_refine_3stage(cd0: &[Complex<f32>], initial_dt_sec: f32) -> FineRefine {
    debug_assert_eq!(cd0.len(), CD0_LEN);

    // Process-lifetime cached on std builds, built fresh per call on
    // no_std — see [`costas_ref_table`]'s doc comment.
    #[cfg(feature = "std")]
    let ref_table = costas_ref_table();
    #[cfg(not(feature = "std"))]
    let ref_table = &costas_ref_table();

    // ── Stage A: ±10 idt at the initial frequency ─────────────────────
    let i0 = ((initial_dt_sec + 0.5) * DS_RATE).round() as i32;
    let mut ibest_a = i0;
    let mut smax_a = f32::MIN;
    for delta in -10..=10_i32 {
        let i = i0 + delta;
        let s = fine_sync_power_signed(cd0, i, ref_table, None);
        if s > smax_a {
            smax_a = s;
            ibest_a = i;
        }
    }

    // ── Stage B: ±5 ifr × 0.5 Hz freq sweep at ibest_a ────────────────
    let mut delfbest = 0.0_f32;
    let mut smax_b = fine_sync_power_signed(cd0, ibest_a, ref_table, None);
    for ifr in -5..=5_i32 {
        if ifr == 0 {
            continue;
        }
        let delf = ifr as f32 * 0.5;
        let tweak = build_tweak(delf);
        let s = fine_sync_power_signed(cd0, ibest_a, ref_table, Some(&tweak));
        // Strict `>` keeps the smaller-|delf| value when scores tie
        // (favour the original frequency).
        if s > smax_b {
            smax_b = s;
            delfbest = delf;
        }
    }

    // ── Stage C: ±4 idt re-search, tweaked by delfbest ────────────────
    let tweak_c = (delfbest.abs() > f32::EPSILON).then(|| build_tweak(delfbest));
    let mut ibest_c = ibest_a;
    let mut smax_c = f32::MIN;
    for delta in -4..=4_i32 {
        let i = ibest_a + delta;
        let s = fine_sync_power_signed(cd0, i, ref_table, tweak_c.as_ref());
        if s > smax_c {
            smax_c = s;
            ibest_c = i;
        }
    }

    let dt_sec = (ibest_c as f32) / DS_RATE - 0.5;
    FineRefine {
        dt_sec,
        delf_hz: delfbest,
        score: smax_c,
    }
}

#[cfg(all(test, feature = "fft-rustfft"))]
mod tests {
    use super::*;
    use crate::ft8::downsample::downsample;
    use crate::ft8::params::COSTAS;
    use crate::ft8::wave_gen::tones_to_f32;
    use alloc::vec;
    use alloc::vec::Vec;

    /// Frozen per-sample-`cos`/`sin` reference copy of
    /// `fine_sync_power_signed` as it existed before the
    /// [`CostasRefTable`] lookup-table rewrite (issue #182 perf
    /// follow-up). Do not "fix" this to match future changes to the
    /// real function — it exists solely so
    /// `ref_table_matches_per_sample_reference` can prove the table
    /// rewrite produces bit-identical output.
    fn fine_sync_power_signed_reference(cd0: &[Complex<f32>], i: i32) -> f32 {
        const DS_SPB: i32 = 32;
        let icos7: [u8; 7] = [3, 1, 4, 0, 6, 5, 2];
        let np2 = cd0.len() as i32;
        let mut total = 0.0_f32;
        for block_off in [0_i32, 36, 72].iter().copied() {
            let mut block_power = 0.0_f32;
            for (k, &tone) in icos7.iter().enumerate() {
                let start = i + block_off * DS_SPB + (k as i32) * DS_SPB;
                if start < 0 || start + DS_SPB > np2 {
                    continue;
                }
                let dphi = core::f32::consts::TAU * (tone as f32) / (DS_SPB as f32);
                let mut z = Complex::new(0.0_f32, 0.0);
                let mut phi = 0.0_f32;
                for j in 0..DS_SPB {
                    let s = cd0[start as usize + j as usize];
                    let r = Complex::new(phi.cos(), phi.sin());
                    z += s * r.conj();
                    phi += dphi;
                    if phi > core::f32::consts::PI {
                        phi -= core::f32::consts::TAU;
                    }
                }
                block_power += z.norm_sqr();
            }
            total += block_power;
        }
        total
    }

    /// Differential test (issue #182 perf follow-up): the
    /// [`CostasRefTable`]-based `fine_sync_power_signed` must return
    /// bit-identical scores to the frozen per-sample-cos/sin reference
    /// above, across several real-shaped inputs (not just one), since
    /// the two are supposed to compute the exact same values — not an
    /// approximation.
    #[test]
    fn ref_table_matches_per_sample_reference() {
        let ref_table = build_costas_ref_table();

        let signal_slot = {
            let tones = costas_only_tones();
            let pcm_f32 = tones_to_f32(&tones, 1500.0, 0.5);
            let mut slot = vec![0i16; 15 * 12_000];
            let start = (0.5_f32 * 12_000.0).round() as usize;
            for (i, &s) in pcm_f32.iter().enumerate() {
                if start + i < slot.len() {
                    slot[start + i] = (s * 16_000.0) as i16;
                }
            }
            slot
        };
        let (cd0_signal, _) = downsample(&signal_slot, 1500.0, None);

        let noise_slot: Vec<i16> = (0..15 * 12_000)
            .map(|n| {
                let x = (n as i64 * 2_654_435_761) as u32;
                ((x >> 16) as i16).wrapping_sub(i16::MAX / 2)
            })
            .collect();
        let (cd0_noise, _) = downsample(&noise_slot, 1500.0, None);

        for (label, cd0) in [("signal", &cd0_signal), ("noise", &cd0_noise)] {
            for i in [-10_i32, -3, 0, 5, 12, 1590, 1605] {
                let expected = fine_sync_power_signed_reference(cd0, i);
                let actual = fine_sync_power_signed(cd0, i, &ref_table, None);
                assert_eq!(
                    expected, actual,
                    "{label} i={i}: table-lookup diverged from per-sample reference"
                );
            }
        }
    }

    /// Frozen composition of the pre-this-change data-shift approach
    /// (issue #182 follow-up, superseding Phase 2's NCO): shifts `cd0`
    /// by `delf_hz` via exact per-sample `cos`/`sin` (the original
    /// `shift_freq` body, before it was deleted in favour of
    /// [`build_tweak`]'s reference-tweak approach), then scores it with
    /// the untweaked per-sample reference. This is the last
    /// known-correct implementation (validated end-to-end by the 5
    /// `fine_refine_3stage` tests below) — the baseline the new
    /// reference-tweak approach must match. Do not "fix" this to track
    /// future changes to the real functions.
    fn fine_sync_power_signed_via_data_shift_reference(
        cd0: &[Complex<f32>],
        i: i32,
        delf_hz: f32,
    ) -> f32 {
        let dt2 = 1.0 / DS_RATE;
        let mut shifted = vec![Complex::new(0.0_f32, 0.0); cd0.len()];
        for (k, (c, o)) in cd0.iter().zip(shifted.iter_mut()).enumerate() {
            let phi = -core::f32::consts::TAU * delf_hz * (k as f32) * dt2;
            let rot = Complex::new(phi.cos(), phi.sin());
            *o = *c * rot;
        }
        fine_sync_power_signed_reference(&shifted, i)
    }

    /// Differential test (issue #182 follow-up): the reference-tweak
    /// `fine_sync_power_signed(..., Some(&build_tweak(delf)))` must
    /// match the frozen data-shift reference above within a tight
    /// tolerance. Unlike Phase 2's NCO (a genuine speed/accuracy trade,
    /// needing a `1e-4` tolerance), this is two *exactly equivalent*
    /// formulations of the same sum (tweak-then-conjugate-product vs
    /// shift-then-correlate) — the only expected divergence is
    /// floating-point reassociation, so the tolerance is tighter and
    /// chosen empirically, not assumed, same discipline as Phase 1/2.
    #[test]
    fn tweak_matches_data_shift_reference_within_tolerance() {
        const TWEAK_MAX_ERROR: f32 = 1e-5;
        let ref_table = build_costas_ref_table();

        let tones = costas_only_tones();
        let pcm_f32 = tones_to_f32(&tones, 1500.0, 0.5);
        let mut slot = vec![0i16; 15 * 12_000];
        let start = (0.5_f32 * 12_000.0).round() as usize;
        for (i, &s) in pcm_f32.iter().enumerate() {
            if start + i < slot.len() {
                slot[start + i] = (s * 16_000.0) as i16;
            }
        }
        let (cd0, _) = downsample(&slot, 1500.0, None);

        for ifr in -5_i32..=5 {
            if ifr == 0 {
                continue;
            }
            let delf = ifr as f32 * 0.5;
            let tweak = build_tweak(delf);
            for i in [-10_i32, -3, 0, 5, 12] {
                let expected = fine_sync_power_signed_via_data_shift_reference(&cd0, i, delf);
                let actual = fine_sync_power_signed(&cd0, i, &ref_table, Some(&tweak));
                let denom = expected.max(1.0);
                let rel_err = (actual - expected).abs() / denom;
                assert!(
                    rel_err < TWEAK_MAX_ERROR,
                    "delf={delf} i={i}: tweak score {actual} vs data-shift reference \
                     {expected}, relative error {rel_err} exceeds tolerance {TWEAK_MAX_ERROR}"
                );
            }
        }
    }

    /// Build a 79-symbol FT8 tone sequence with the 3 Costas blocks
    /// in their canonical positions. Data symbols (positions 7..36 and
    /// 43..72) are zeros — recall this synthesised audio is for
    /// **sync** verification only, the data tones don't matter.
    fn costas_only_tones() -> [u8; 79] {
        let mut t = [0u8; 79];
        for (i, &c) in COSTAS.iter().enumerate() {
            t[i] = c as u8; // block 0 at symbols 0..7
            t[36 + i] = c as u8; // block 1 at symbols 36..43
            t[72 + i] = c as u8; // block 2 at symbols 72..79
        }
        t
    }

    /// Synthesise an FT8 audio slot at the given frequency, with the
    /// signal starting at the given `dt_sec` offset within the slot.
    /// Output is 15 s × 12 kHz = 180_000 samples i16.
    fn synth_slot(freq_hz: f32, dt_sec: f32) -> Vec<i16> {
        let tones = costas_only_tones();
        let pcm_f32 = tones_to_f32(&tones, freq_hz, 0.5);
        let mut slot = vec![0i16; 15 * 12_000];
        // TX start offset = 0.5 s + dt_sec, in samples at 12 kHz.
        let start = ((0.5 + dt_sec) * 12_000.0).round() as isize;
        for (i, &s) in pcm_f32.iter().enumerate() {
            let dst = start + i as isize;
            if (0..slot.len() as isize).contains(&dst) {
                slot[dst as usize] = (s * 16_000.0) as i16;
            }
        }
        slot
    }

    #[test]
    fn freq_snap_zero_offset() {
        // True signal at 1500 Hz, dt=0. Refine should return
        // delf ≈ 0, dt ≈ 0.
        let slot = synth_slot(1500.0, 0.0);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        assert!(
            r.delf_hz.abs() <= 0.5,
            "expected |delf| ≤ 0.5, got {}",
            r.delf_hz,
        );
        assert!(
            r.dt_sec.abs() <= 0.02,
            "expected |dt| ≤ 20 ms, got {}",
            r.dt_sec,
        );
        assert!(r.score > 0.0, "score should be positive on signal");
    }

    #[test]
    fn freq_snap_positive_offset() {
        // True signal at 1500.7 Hz; downsample-mix at 1500 Hz so cd0
        // baseband sees the signal at +0.7 Hz. Refine should pick
        // delfbest ∈ {+0.5, +1.0}.
        let slot = synth_slot(1500.7, 0.0);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        let close_to_grid = (r.delf_hz - 0.5).abs() < 0.01 || (r.delf_hz - 1.0).abs() < 0.01;
        assert!(
            close_to_grid,
            "expected delf snap to +0.5 or +1.0, got {}",
            r.delf_hz,
        );
    }

    #[test]
    fn freq_snap_negative_offset() {
        let slot = synth_slot(1500.0 - 1.3, 0.0);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        let close_to_grid = (r.delf_hz - (-1.5)).abs() < 0.01 || (r.delf_hz - (-1.0)).abs() < 0.01;
        assert!(
            close_to_grid,
            "expected delf snap to -1.5 or -1.0, got {}",
            r.delf_hz,
        );
    }

    #[test]
    fn dt_snap_positive() {
        // Signal at 1500 Hz, dt = +0.04 s (= 8 cd0 samples). Refine
        // should converge to ibest ≈ 8, i.e. dt_sec ≈ +0.04.
        let slot = synth_slot(1500.0, 0.04);
        let (cd0, _) = downsample(&slot, 1500.0, None);
        let r = fine_refine_3stage(&cd0, 0.0);
        assert!(
            (r.dt_sec - 0.04).abs() <= 0.015,
            "expected dt ≈ 0.04, got {}",
            r.dt_sec,
        );
    }

    #[test]
    fn no_signal_low_score() {
        // Pure-noise input → score should be much smaller than the
        // signal-bearing test cases (~10× lower is the empirical
        // envelope). We don't pin an absolute floor.
        let slot_signal = synth_slot(1500.0, 0.0);
        let (cd0_signal, _) = downsample(&slot_signal, 1500.0, None);
        let s_signal = fine_refine_3stage(&cd0_signal, 0.0).score;

        let slot_noise = vec![0i16; 15 * 12_000];
        let (cd0_noise, _) = downsample(&slot_noise, 1500.0, None);
        let s_noise = fine_refine_3stage(&cd0_noise, 0.0).score;

        assert!(
            s_signal > 5.0 * s_noise,
            "signal score {} should dominate noise score {}",
            s_signal,
            s_noise,
        );
    }
}
