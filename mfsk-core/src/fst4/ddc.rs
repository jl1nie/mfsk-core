// SPDX-License-Identifier: GPL-3.0-or-later
//! FST4-60 coarse-sync DDC: `K`-selection and cascade decomposition.
//!
//! `docs/notes/FST4_DDC_DESIGN.md` §4.3, §7 stage 3. Scope matches the
//! design doc's own (§3): FST4-60 only, and only the two search shapes
//! `coarse_sync` actually uses today — a narrow sniper window (±250 Hz
//! around a frequency hint) and the wideband scan (100-3000 Hz).
//!
//! ## `K`-selection
//!
//! §2's invariants force `Fs_c = K/SYMBOL_DT` once `nfft1 = 2K` is
//! chosen (both `df = Fs_c/nfft1` and `nfft1` fixed simultaneously
//! pins `Fs_c`) — so picking `K` *is* picking the analysis rate, not a
//! free parameter alongside it. [`choose_k`] picks the smallest power
//! of two whose `Fs_c` covers a target bandwidth: `K ≥ bandwidth_hz ×
//! SYMBOL_DT` (this falls out of `Fs_c = K/SYMBOL_DT ≥ bandwidth_hz`).
//! Reproduces the design doc's own worked table exactly: `600 Hz →
//! K=256` (sniper), `2900 Hz → K=1024` (wideband) — see this module's
//! own `choose_k_matches_design_doc_table` test.
//!
//! ## Cascade split
//!
//! `Fs_c/12_000 = K/NSPS` always reduces to `(K/16)/243` for FST4-60
//! (`NSPS = 3888 = 2^4·3^5`; a power-of-two `K ≥ 16` always has
//! `gcd(K, 3888) = 16`) — the `243 = 3^5` denominator is exactly the
//! irreducible part §1 says a DDC exists to route around, and it's
//! fixed regardless of `K`. The two configs below split that ratio
//! differently, both taken from the design doc's own worked table
//! rather than re-derived here: sniper pre-decimates by an integer 3
//! (`FirStage`, loose filter, cheap) before the final rational
//! `16/81` stage; wideband skips the integer stage entirely (`1/1`)
//! and does the whole `64/243` in one rational stage. Tap counts are
//! this module's own Crochiere-Rabiner-style derivation (§4.3's own
//! `~84`/`~184` taps/phase are explicitly estimates, not a spec to
//! reproduce exactly) — see the `SNIPER_*`/`WIDEBAND_*` constants
//! below for the transition-width choices, and
//! `tests::cascade_configs_hit_their_target_fs_c` for the
//! self-consistency check that the two integer/rational splits
//! actually multiply out to `Fs_c`.

use crate::engine::dsp::ddc::DdcCascadeConfig;

/// `SYMBOL_DT` for FST4-60, `NSPS/12_000 = 3888/12000`. A plain `f32`
/// literal rather than reaching through
/// `<Fst4s60 as ModulationParams>::SYMBOL_DT` — this module fixes the
/// sub-mode in scope by construction (see the module doc comment), so
/// there's no generic `P` to derive it from; keeping this as the one
/// named constant (used by every derivation below and pinned against
/// the trait value in `tests::symbol_dt_matches_fst4s60`) is cheaper
/// than adding a `P: ModulationParams` bound this module doesn't
/// otherwise need.
const SYMBOL_DT_S: f32 = 0.324;

/// Smallest power-of-two `K` whose `Fs_c = K/SYMBOL_DT_S` covers
/// `bandwidth_hz` — see the module doc comment's `K`-selection
/// derivation.
pub fn choose_k(bandwidth_hz: f32) -> u32 {
    let raw = (bandwidth_hz * SYMBOL_DT_S).ceil() as u32;
    raw.max(1).next_power_of_two()
}

/// The complex analysis grid `choose_k` implies: `K`, `Fs_c` (Hz),
/// `nfft1 = 2K`.
#[derive(Copy, Clone, Debug)]
pub struct Fst4DdcGrid {
    pub k: u32,
    pub fs_c: f32,
    pub nfft1: usize,
}

pub fn grid_for(bandwidth_hz: f32) -> Fst4DdcGrid {
    let k = choose_k(bandwidth_hz);
    Fst4DdcGrid {
        k,
        fs_c: k as f32 / SYMBOL_DT_S,
        nfft1: 2 * k as usize,
    }
}

/// Sniper stage-1 (integer, decimate-by-3) tap count. `fc_norm`
/// cutoff 300 Hz (comfortably above the ±250 Hz search window, which
/// is all this loose pre-filter needs to protect — the final
/// resampler carries the real passband/stopband requirement, same
/// "stage 1 only needs to keep stage 2 honest" logic
/// `wspr::ddc`'s own cascade doc comment uses), 1000 Hz transition
/// (`4/ntaps` rule, `design_lowpass`'s own convention) — a
/// deliberately loose, cheap filter: `ntaps = ⌈4×12000/1000⌉` rounded
/// up to odd.
const SNIPER_STAGE1_NTAPS: usize = 49;
const SNIPER_STAGE1_DECIM: usize = 3;
const SNIPER_STAGE1_FC_NORM: f32 = 300.0 / 12_000.0;

/// Sniper final resampler: `L=16, M=81` (see the module doc comment's
/// cascade-split derivation), 150 Hz transition around the output
/// Nyquist (`Fs_c/2 ≈ 395 Hz`) — flat through the ±250 Hz search
/// window with room to spare before the ~395 Hz edge.
/// `ntaps = ⌈4×L×F1/150⌉` rounded up to odd, `F1 = 12000/3 = 4000 Hz`
/// (stage 1's output rate).
const SNIPER_RESAMPLER_L: u32 = 16;
const SNIPER_RESAMPLER_M: u32 = 81;
const SNIPER_RESAMPLER_NTAPS: usize = 1707;

/// Wideband final (only) resampler: `L=64, M=243`, 300 Hz transition
/// around the output Nyquist (`Fs_c/2 ≈ 1580 Hz`) — flat through the
/// ±1450 Hz half-width the 2900 Hz search band needs.
/// `ntaps = ⌈4×L×12000/300⌉` rounded up to odd (no integer stage, so
/// `F_in = 12000` directly).
const WIDEBAND_RESAMPLER_L: u32 = 64;
const WIDEBAND_RESAMPLER_M: u32 = 243;
const WIDEBAND_RESAMPLER_NTAPS: usize = 10_241;

/// History margin every stage in both configs uses — both cascades'
/// stages are small (see the tap-count constants above), so a
/// generous fixed margin costs little and keeps compaction well
/// amortised (same tradeoff `wspr::ddc`'s own cascade documents).
const HIST_MARGIN: usize = 512;

/// FST4-60 sniper DDC: ±250 Hz search window centred on `center_hz`,
/// `K=256`. Matches `SniperRequest`'s own search shape
/// (`docs/notes/FST4_DDC_DESIGN.md` §4.3's "スナイパー ±250 Hz" row).
pub fn sniper_cascade(center_hz: f32) -> DdcCascadeConfig {
    DdcCascadeConfig {
        center_hz,
        input_rate_hz: 12_000.0,
        int_stages: alloc::vec![(
            SNIPER_STAGE1_NTAPS,
            SNIPER_STAGE1_DECIM,
            SNIPER_STAGE1_FC_NORM
        )],
        resampler: (
            SNIPER_RESAMPLER_L,
            SNIPER_RESAMPLER_M,
            SNIPER_RESAMPLER_NTAPS,
        ),
        hist_margin: HIST_MARGIN,
    }
}

/// FST4-60 wideband DDC: `center_hz` should be the mid-point of the
/// 100-3000 Hz scan (matching `K=1024`'s ≈3160 Hz `Fs_c` against the
/// 2900 Hz band — `docs/notes/FST4_DDC_DESIGN.md` §4.3's "広帯域
/// 100-3000 Hz" row).
pub fn wideband_cascade(center_hz: f32) -> DdcCascadeConfig {
    DdcCascadeConfig {
        center_hz,
        input_rate_hz: 12_000.0,
        int_stages: alloc::vec::Vec::new(),
        resampler: (
            WIDEBAND_RESAMPLER_L,
            WIDEBAND_RESAMPLER_M,
            WIDEBAND_RESAMPLER_NTAPS,
        ),
        hist_margin: HIST_MARGIN,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symbol_dt_matches_fst4s60() {
        use crate::engine::protocol::ModulationParams;
        assert!(
            (SYMBOL_DT_S - <super::super::Fst4s60 as ModulationParams>::SYMBOL_DT).abs() < 1e-6
        );
    }

    /// Reproduces `docs/notes/FST4_DDC_DESIGN.md` §4.3's worked table:
    /// `600 Hz -> K=256` (sniper), `2900 Hz -> K=1024` (wideband).
    /// These are exact invariants (§2), not estimates — unlike the
    /// tap-count constants, which the doc itself marks with `~`.
    #[test]
    fn choose_k_matches_design_doc_table() {
        assert_eq!(choose_k(600.0), 256, "sniper");
        assert_eq!(choose_k(2900.0), 1024, "wideband");
    }

    #[test]
    fn grid_for_matches_design_doc_table() {
        let sniper = grid_for(600.0);
        assert_eq!(sniper.k, 256);
        assert_eq!(sniper.nfft1, 512);
        assert!((sniper.fs_c - 790.123).abs() < 0.01, "{}", sniper.fs_c);

        let wideband = grid_for(2900.0);
        assert_eq!(wideband.k, 1024);
        assert_eq!(wideband.nfft1, 2048);
        assert!((wideband.fs_c - 3160.494).abs() < 0.01, "{}", wideband.fs_c);
    }

    /// Self-consistency: each cascade's integer-stage decimation times
    /// its final resampler's `L/M` must land on exactly the `Fs_c`
    /// `grid_for` predicts for that config's bandwidth — the check the
    /// module doc comment's cascade-split section promises.
    #[test]
    fn cascade_configs_hit_their_target_fs_c() {
        let sniper = sniper_cascade(1500.0);
        let sniper_grid = grid_for(600.0);
        let sniper_decim: usize = sniper.int_stages.iter().map(|&(_, d, _)| d).product();
        let sniper_fs_c = sniper.input_rate_hz / sniper_decim as f32 * SNIPER_RESAMPLER_L as f32
            / SNIPER_RESAMPLER_M as f32;
        assert!(
            (sniper_fs_c - sniper_grid.fs_c).abs() < 0.01,
            "sniper cascade Fs_c {sniper_fs_c} != grid Fs_c {}",
            sniper_grid.fs_c
        );

        let wideband = wideband_cascade(1500.0);
        assert!(wideband.int_stages.is_empty());
        let wideband_grid = grid_for(2900.0);
        let wideband_fs_c =
            wideband.input_rate_hz * WIDEBAND_RESAMPLER_L as f32 / WIDEBAND_RESAMPLER_M as f32;
        assert!(
            (wideband_fs_c - wideband_grid.fs_c).abs() < 0.01,
            "wideband cascade Fs_c {wideband_fs_c} != grid Fs_c {}",
            wideband_grid.fs_c
        );
    }

    /// End-to-end equivalence (design doc §6 item 3): the DDC-fed
    /// complex `coarse_sync` path finds the same candidate — same
    /// frequency, comparable score — the real 12 kHz path finds, on a
    /// clean synthetic FST4-60 signal. Not the WSJT-X golden WAV the
    /// design doc's own item 3 names (`210115_0058.wav`); a
    /// self-consistency check on this crate's own encoder, same
    /// caveat `fst4::decode::tests::synth_roundtrip_for`'s doc comment
    /// already carries for the same reason (encode/decode share
    /// `NSPS`/`NDOWN` — this can't catch a wrong-vs-WSJT-X parameter).
    /// Sufficient here: this test's job is confirming the *DDC
    /// pipeline* reproduces what the real-input path already computes
    /// correctly, not re-verifying `coarse_sync` itself against
    /// WSJT-X, which the existing FST4-60 golden tests already do on
    /// the real path.
    #[test]
    fn ddc_coarse_sync_matches_real_path_on_synth_signal() {
        use crate::engine::dsp::ddc::ddc_block;
        use crate::engine::sync::{AudioSource, RxGrid, coarse_sync};
        use crate::fst4::Fst4s60;
        use crate::fst4::encode::{message_to_tones, tones_to_i16};
        use crate::msg::wsjt77::pack77;

        let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let tones = message_to_tones(&msg77);
        let target_freq = 1500.0f32;
        let audio = tones_to_i16(&tones, target_freq, 10_000);

        let mut slot = alloc::vec![0i16; 60 * 12_000];
        let offset = 12_000;
        let copy_len = audio.len().min(slot.len() - offset);
        slot[offset..offset + copy_len].copy_from_slice(&audio[..copy_len]);

        let real_cands = coarse_sync::<Fst4s60>(
            AudioSource::Real(&slot),
            target_freq - 250.0,
            target_freq + 250.0,
            0.0,
            None,
            10,
            RxGrid::real(12_000.0),
        );
        let real_best = real_cands
            .iter()
            .max_by(|a, b| a.score.partial_cmp(&b.score).unwrap())
            .expect("real path found no candidate at all");

        let cfg = sniper_cascade(target_freq);
        let (audio_i, audio_q) = ddc_block(&slot, &cfg);
        let grid = grid_for(600.0);
        let ddc_cands = coarse_sync::<Fst4s60>(
            AudioSource::Complex(&audio_i, &audio_q),
            target_freq - 250.0,
            target_freq + 250.0,
            0.0,
            None,
            10,
            RxGrid::complex(grid.fs_c, target_freq),
        );
        let ddc_best = ddc_cands
            .iter()
            .max_by(|a, b| a.score.partial_cmp(&b.score).unwrap())
            .expect("DDC path found no candidate at all");

        assert!(
            (ddc_best.freq_hz - real_best.freq_hz).abs() < 2.0,
            "real best freq={} Hz, DDC best freq={} Hz",
            real_best.freq_hz,
            ddc_best.freq_hz
        );
        // Score comparison, not dt_sec: the DDC cascade's own group
        // delay isn't trimmed from its output (`StreamingComplexDdc::
        // flush`'s own doc comment — a conservative, not exact,
        // estimate), so `dt_sec` carries an unremoved, unknown-but-
        // bounded offset. `coarse_sync`'s ±2.5 s lag search already
        // absorbs that (it exists to tolerate unknown timing), so
        // candidate *detection* doesn't depend on it — only reporting
        // a precise `dt_sec` does, which is refine's job (stage 4),
        // not coarse search's.
        //
        // Not a tight ratio: measured on this synth signal, the DDC
        // path's score lands at roughly 1/3 of the real path's
        // (~115 vs. ~355) — a real processing-margin cost from
        // routing through a mixer + 2-stage filter cascade instead of
        // one direct 7776-point FFT, not a bug (this is still a
        // factor of ~30+ above the ~1-5 noise-floor scores
        // `coarse_sync`'s own normalisation produces — see e.g.
        // `stage1_pass`'s `sync_min` gate). Whether that margin cost
        // is acceptable for real (non-synthetic, noisy) signals is a
        // sensitivity-sweep question (design doc §6 item 4), and
        // whether the `SNIPER_*`/`WIDEBAND_*` tap counts above are
        // the right cost/margin tradeoff is open for that sweep to
        // answer — this assertion only guards against the DDC path
        // going *dark* (a real bug), not against it costing SNR
        // margin, which some cost is expected to.
        let noise_floor_headroom = 20.0;
        assert!(
            ddc_best.score > noise_floor_headroom,
            "DDC best score={} looks noise-floor-scale, not a detected signal",
            ddc_best.score
        );
        assert!(
            ddc_best.score > real_best.score * 0.2,
            "real best score={}, DDC best score={} — margin cost far past what §4.3's \
             estimated tap counts should plausibly produce",
            real_best.score,
            ddc_best.score
        );
    }

    /// `wideband_cascade` isn't exercised by the sniper-focused
    /// end-to-end test above (its `~10K`-tap resampler is the more
    /// expensive of the two configs) — this pins that its own
    /// construction and single-tone response are sane: a centre tone
    /// lands with non-trivial magnitude and the output length roughly
    /// matches the `K/NSPS` decimation ratio `grid_for` predicts.
    /// Cheaper than another full `coarse_sync` round-trip and still
    /// catches an `L`/`M` swap or ntaps-parity mistake specific to
    /// this config.
    #[test]
    fn wideband_cascade_single_tone_lands_near_dc() {
        use crate::engine::dsp::ddc::ddc_block;

        let center = 1550.0f32;
        let cfg = wideband_cascade(center);
        let n = 240_000; // 20 s @ 12 kHz — enough to clear the ~10K-tap resampler's settling
        let w = 2.0 * core::f64::consts::PI * center as f64 / 12_000.0;
        let audio: Vec<i16> = (0..n)
            .map(|k| (8000.0 * (w * k as f64).cos()) as i16)
            .collect();

        let (i, q) = ddc_block(&audio, &cfg);
        let grid = grid_for(2900.0);
        let expected_len = (n as f32 * grid.fs_c / 12_000.0) as usize;
        assert!(
            i.len().abs_diff(expected_len) < expected_len / 4,
            "got {} outputs, expected ~{expected_len}",
            i.len()
        );

        let settle = i.len() / 3;
        let span = settle..(i.len() - settle);
        let peak = span
            .map(|k| (i[k] * i[k] + q[k] * q[k]).sqrt())
            .fold(0.0f32, f32::max);
        assert!(peak > 500.0, "centre tone peak magnitude too small: {peak}");
    }
}
