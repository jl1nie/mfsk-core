// SPDX-License-Identifier: GPL-3.0-or-later
//! Puncturing patterns for the four uvpacket modes.
//!
//! All four modes share `Ldpc240_101` as their FEC mother code:
//! 101 systematic info bits + 139 parity bits = 240 channel bits at
//! native rate 0.421. Higher-rate modes drop a uniform-spread subset
//! of the **parity** bits before transmission. The decoder recovers
//! by inserting an erasure LLR (value 0.0) at each punctured position
//! before BP / OSD.
//!
//! Per-mode keep counts (info kept always; parity kept after
//! puncture):
//!
//! | Mode        | parity kept | total ch bits | FEC rate          |
//! |-------------|------------:|--------------:|-------------------|
//! | UltraRobust | 139         | 240           | 101/240 = 0.421 † |
//! | Robust      | 139         | 240           | 101/240 = 0.421   |
//! | Standard    | 101         | 202           | 101/202 = 0.500   |
//! | Express     |  33         | 134           | 101/134 = 0.754   |
//!
//! † UltraRobust shares Robust's no-puncture pattern; the rate
//! difference at the modulation layer comes entirely from
//! UltraRobust's half symbol rate (600 baud → 504 net bps vs
//! Robust's 1008 net bps).
//!
//! `Express` applies aggressive puncturing (76 % of the mother
//! code's 139 parity bits) — the WSJT-X authors did not design
//! `Ldpc240_101` for puncturing, so decoder convergence at this
//! rate is not given. The empirical AWGN sweep in
//! `tests::modes_awgn_sweep_uniform_vs_kSR` and
//! `tests::experimental_rate_3_4` characterises the per-mode
//! behaviour and motivates the kSR-greedy puncture-set selection.
//!
//! Empirical findings (200-trial AWGN sweep, OSD-2 fallback):
//!
//! - `Express` decodes 99 % at Eb/N0 = +3 dB with OSD-2; BP-only
//!   needs ~+5 dB. Without kSR-greedy the BP floor is ~3 dB worse.
//! - `Fast` shows BP improvement of ~0.5–1 dB from kSR-greedy.
//! - `Standard` is mostly indifferent to selector choice (puncture
//!   density too low to matter).
//! - `Robust` is unpunctured — both selectors emit the same set.
//!
//! Two puncture-set construction methods are implemented:
//!
//! 1. **Uniform spread** — Bresenham-style integer recurrence over
//!    the 139 parity slots: keep position `i` iff
//!    `floor((i + 1) · keep / 139) > floor(i · keep / 139)`.
//!    Maximally uniform spread without any tables. Reference
//!    baseline; ignores H-matrix structure entirely.
//!
//! 2. **kSR-greedy** — at each step, add the parity position
//!    whose addition leaves the puncture set most recoverable,
//!    measured by the per-bit `k`-step recoverability classifier
//!    described below. This is the Ha-McLaughlin-style construction
//!    used in the LDPC-puncturing literature.
//!
//! The default returned by [`keep_indices`] is **kSR-greedy**.
//! `keep_indices_uniform` and `keep_indices_kSR_greedy` expose
//! both for direct comparison; the convergence sweep test in
//! `tests::compare_uniform_vs_kSR_greedy` measures the difference.
//!
//! Selected sets are computed once per process (via `OnceLock`)
//! since the greedy search is not free (~1 s wall-clock for the
//! Fast mode at 88 punctures, in release mode).
//!
//! ## k-step recoverability (k-SR)
//!
//! After Ha & McLaughlin (2002, 2004): a punctured variable node
//! `v` is **k-SR** iff there exists a check-node neighbour `c` of
//! `v` such that every other variable node in `c` is either
//! unpunctured or `j`-SR with `j < k`. Equivalently, BP recovers
//! `v`'s value within `k` iterations under the assumption that
//! all unpunctured neighbours are perfectly known.
//!
//! Lower `k` is better for the punctured bit. Maximising the
//! count of low-`k` punctured bits across the whole puncture set
//! is the objective of the greedy construction.

/// `Ldpc240_101` codeword length.
const N: usize = 240;
/// `Ldpc240_101` info-bit count.
const K_INFO: usize = 101;
/// Parity-bit count = N − K_INFO.
const N_PARITY: usize = N - K_INFO;

/// Per-mode rate + symbol-rate posture.
///
/// All modes share the same `Ldpc240_101` mother code; they differ
/// in (a) the puncture pattern applied to the parity bits and
/// (b) the symbol rate at which the channel bits are transmitted.
/// `UltraRobust` is the only mode that uses a half-baud (600 baud)
/// symbol rate — the others run at the canonical 1200 baud.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    /// **Marathon** mode — unpunctured `Ldpc240_101` (native rate
    /// ≈ 0.42) at **600 baud** (half the canonical symbol rate).
    /// 504 net bps, ~−1.75 dB SNR_3kHz threshold; the lowest-
    /// threshold mode in the lineup. The half-baud chip duration
    /// also halves per-symbol phase walk and the relative size of
    /// any multipath delay, giving a substantial real-channel
    /// margin on top of the 3 dB symbol-energy gain over Robust.
    UltraRobust,
    /// Unpunctured `Ldpc240_101` at the canonical 1200 baud. Native
    /// rate ≈ 0.42, 1008 net bps. Standard weak-signal posture.
    Robust,
    /// Punctured to rate 1/2 at 1200 baud. 1200 net bps. AFSK 1200
    /// throughput parity plus FEC for typical NFM channels.
    Standard,
    /// Punctured to rate 3/4 at 1200 baud. 1800 net bps. Headline-
    /// fast mode for strong signals; 76 % parity puncturing makes
    /// OSD-2 essentially mandatory at the BP threshold.
    Express,
}

impl Mode {
    /// 2-bit encoding for the frame-header `mode` field.
    pub const fn header_code(self) -> u8 {
        match self {
            Mode::UltraRobust => 0,
            Mode::Robust => 1,
            Mode::Standard => 2,
            Mode::Express => 3,
        }
    }

    /// Inverse of [`Self::header_code`]. Returns `None` for any
    /// value `> 3`.
    pub const fn from_header_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Mode::UltraRobust),
            1 => Some(Mode::Robust),
            2 => Some(Mode::Standard),
            3 => Some(Mode::Express),
            _ => None,
        }
    }

    /// Channel bits transmitted per LDPC block at this mode.
    pub const fn ch_bits_per_block(self) -> usize {
        match self {
            // UltraRobust shares Robust's puncture pattern (none) —
            // the rate difference vs Robust is purely from the
            // halved symbol rate, not from the FEC layer.
            Mode::UltraRobust => 240,
            Mode::Robust => 240,
            Mode::Standard => 202,
            Mode::Express => 134,
        }
    }

    /// Samples per symbol at the modem's 12 kHz sample rate. All
    /// modes share the same audio bandwidth; UltraRobust just runs
    /// at half the symbol rate (600 baud → 20 samples/sym), which
    /// doubles the matched-filter integration window and yields
    /// 3 dB more symbol energy per info bit.
    pub const fn nsps(self) -> usize {
        match self {
            Mode::UltraRobust => 20,
            _ => 10,
        }
    }

    /// Parity bits kept (transmitted) at this mode.
    const fn parity_kept(self) -> usize {
        self.ch_bits_per_block() - K_INFO
    }
}

/// Build the kSR-greedy keep-index list for a given mode. Returned
/// vector has length `mode.ch_bits_per_block()`; positions are
/// codeword indices (in `0..240`) given in ascending order. Info
/// bits `0..101` always appear first.
///
/// Result is cached per mode in a `OnceLock`; first call per mode
/// pays the greedy-search cost (~1 s for Fast mode in release
/// mode), subsequent calls are O(1).
pub fn keep_indices(mode: Mode) -> Vec<usize> {
    use std::sync::OnceLock;
    static ROBUST: OnceLock<Vec<usize>> = OnceLock::new();
    static STANDARD: OnceLock<Vec<usize>> = OnceLock::new();
    static EXPRESS: OnceLock<Vec<usize>> = OnceLock::new();

    let cell = match mode {
        // UltraRobust shares Robust's no-puncture pattern.
        Mode::UltraRobust | Mode::Robust => &ROBUST,
        Mode::Standard => &STANDARD,
        Mode::Express => &EXPRESS,
    };
    cell.get_or_init(|| keep_indices_kSR_greedy(mode)).clone()
}

/// Uniform-spread keep-index list — the reference baseline.
///
/// Parity position `p` (where `p ∈ 0..139`) is kept iff
/// `((p + 1) · keep) / 139 > (p · keep) / 139` (integer division),
/// where `keep = mode.parity_kept()`. This is a Bresenham-style
/// maximally-uniform selection that ignores the H-matrix structure.
pub fn keep_indices_uniform(mode: Mode) -> Vec<usize> {
    let mut keep = Vec::with_capacity(mode.ch_bits_per_block());
    keep.extend(0..K_INFO);
    let n_keep = mode.parity_kept();
    if n_keep == N_PARITY {
        keep.extend(K_INFO..N);
    } else if n_keep > 0 {
        for p in 0..N_PARITY {
            let cur = ((p + 1) * n_keep) / N_PARITY;
            let prev = (p * n_keep) / N_PARITY;
            if cur > prev {
                keep.push(K_INFO + p);
            }
        }
    }
    debug_assert_eq!(keep.len(), mode.ch_bits_per_block());
    keep
}

/// kSR-greedy keep-index list for the standard rate-mode count.
#[allow(non_snake_case)]
pub fn keep_indices_kSR_greedy(mode: Mode) -> Vec<usize> {
    keep_indices_kSR_greedy_with_count(N_PARITY - mode.parity_kept())
}

/// kSR-greedy keep-index list for an arbitrary number of parity
/// punctures (`0..=N_PARITY`). Info bits `0..101` are always kept.
///
/// Constructs the puncture set one position at a time, picking the
/// candidate whose tentative addition produces the lowest
/// "puncture pain" — minimise the count of unrecoverable bits
/// first, then minimise the sum of k-SR levels (so 1-SR is preferred
/// over 2-SR, etc.).
///
/// Used directly for experimental sweeps at puncture counts that
/// aren't part of the production [`Mode`] set (e.g. rate-3/4 study).
#[allow(non_snake_case)]
pub fn keep_indices_kSR_greedy_with_count(target_punctures: usize) -> Vec<usize> {
    use crate::fec::ldpc::Ldpc240_101Params as P;

    assert!(
        target_punctures <= N_PARITY,
        "target_punctures must not exceed parity count {N_PARITY}",
    );

    if target_punctures == 0 {
        return (0..N).collect();
    }

    let mut punctured = vec![false; N];

    for _ in 0..target_punctures {
        let mut best_p = K_INFO;
        let mut best_score = (i64::MIN, i64::MIN);
        for p in K_INFO..N {
            if punctured[p] {
                continue;
            }
            punctured[p] = true;
            let lvls = classify_kSR::<P>(&punctured, 12);
            let s = score_levels(&lvls);
            if s > best_score {
                best_score = s;
                best_p = p;
            }
            punctured[p] = false;
        }
        punctured[best_p] = true;
    }

    (0..N).filter(|&i| !punctured[i]).collect()
}

/// Classify every variable bit by its k-step recoverability after
/// puncturing. `level[v] == 0` means `v` is unpunctured (the
/// decoder always knows its LLR); `level[v] == k > 0` means `v` is
/// `k`-SR (recoverable in `k` BP iterations); `level[v] == u32::MAX`
/// means unrecoverable within `max_k` steps.
#[allow(non_snake_case)]
pub(crate) fn classify_kSR<P: crate::fec::ldpc::LdpcParams>(
    punctured: &[bool],
    max_k: u32,
) -> Vec<u32> {
    let mut level = vec![u32::MAX; P::N];
    for v in 0..P::N {
        if !punctured[v] {
            level[v] = 0;
        }
    }

    for k in 1..=max_k {
        for v in 0..P::N {
            if !punctured[v] || level[v] != u32::MAX {
                continue;
            }
            // Check each check-node neighbour of v.
            let checks = P::mn(v);
            for &c_idx in &checks {
                let c = c_idx as usize;
                let row_w = P::nrw(c) as usize;
                let mut all_lower = true;
                for slot in 0..row_w {
                    let v2 = P::nm(c, slot) as usize;
                    if v2 == v {
                        continue;
                    }
                    if level[v2] >= k {
                        all_lower = false;
                        break;
                    }
                }
                if all_lower {
                    level[v] = k;
                    break;
                }
            }
        }
    }

    level
}

/// Score function for the greedy: (−unrecoverable count, −Σ levels).
/// Higher (= less negative) is better.
fn score_levels(levels: &[u32]) -> (i64, i64) {
    let mut unrecoverable = 0i64;
    let mut sum_levels = 0i64;
    for &l in levels {
        if l == u32::MAX {
            unrecoverable += 1;
        } else {
            sum_levels += l as i64;
        }
    }
    (-unrecoverable, -sum_levels)
}

/// Apply puncturing to a full `Ldpc240_101` codeword: drop the
/// punctured-parity positions and return only the channel bits
/// to transmit, in their original codeword order (the puncture
/// step does not reorder bits, only drops them).
///
/// Panics if `codeword.len() != 240`.
pub fn puncture(codeword: &[u8], mode: Mode) -> Vec<u8> {
    assert_eq!(codeword.len(), N, "codeword must be 240 bits");
    let keep = keep_indices(mode);
    keep.iter().map(|&i| codeword[i]).collect()
}

/// Inverse of [`puncture`]: given LLRs of the transmitted channel
/// bits (length = `mode.ch_bits_per_block()`), expand back to a
/// length-240 LLR vector by inserting an erasure LLR (value 0.0)
/// at every punctured position. The result is fed to
/// `Ldpc240_101::decode_soft`.
///
/// Panics if `channel_llrs.len() != mode.ch_bits_per_block()`.
pub fn de_puncture_llr(channel_llrs: &[f32], mode: Mode) -> Vec<f32> {
    assert_eq!(
        channel_llrs.len(),
        mode.ch_bits_per_block(),
        "channel LLR vector must equal mode's ch_bits_per_block",
    );
    let keep = keep_indices(mode);
    let mut out = vec![0.0f32; N];
    for (k, &j) in keep.iter().enumerate() {
        out[j] = channel_llrs[k];
    }
    out
}

// ────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_MODES: [Mode; 4] = [
        Mode::Robust,
        Mode::Standard,
        Mode::UltraRobust,
        Mode::Express,
    ];

    #[test]
    fn ch_bits_match_keep_indices_len() {
        for mode in ALL_MODES {
            assert_eq!(keep_indices(mode).len(), mode.ch_bits_per_block());
        }
    }

    #[test]
    fn info_bits_always_kept_first() {
        for mode in ALL_MODES {
            let keep = keep_indices(mode);
            for (i, &k) in keep.iter().take(K_INFO).enumerate() {
                assert_eq!(k, i, "{mode:?}: info position {i} not kept first");
            }
        }
    }

    #[test]
    fn keep_indices_sorted_no_dup() {
        for mode in ALL_MODES {
            let keep = keep_indices(mode);
            for w in keep.windows(2) {
                assert!(
                    w[0] < w[1],
                    "{mode:?}: indices not strictly ascending: {} >= {}",
                    w[0],
                    w[1]
                );
            }
            assert!(keep.iter().all(|&i| i < N));
        }
    }

    #[test]
    fn puncture_de_puncture_roundtrip() {
        // Build a codeword whose bit value equals (i & 1) so each
        // surviving channel bit can be checked individually.
        let codeword: Vec<u8> = (0..N).map(|i| (i & 1) as u8).collect();
        for mode in ALL_MODES {
            let punctured = puncture(&codeword, mode);
            assert_eq!(punctured.len(), mode.ch_bits_per_block());

            // Convert hard bits to LLRs (large magnitude, sign by
            // bit value) so de_puncture_llr's interface can be
            // tested. +∞ for 0, −∞ for 1 follows the WSJT
            // convention; here use ±10.0 to keep numbers finite.
            let llrs: Vec<f32> = punctured
                .iter()
                .map(|&b| if b == 0 { 10.0 } else { -10.0 })
                .collect();
            let expanded = de_puncture_llr(&llrs, mode);

            // Punctured positions get LLR 0.0; kept positions match
            // the bit-encoded LLR.
            let keep = keep_indices(mode);
            let kept_set: std::collections::HashSet<_> = keep.iter().copied().collect();
            for i in 0..N {
                if kept_set.contains(&i) {
                    let expected = if codeword[i] == 0 { 10.0 } else { -10.0 };
                    assert!(
                        (expanded[i] - expected).abs() < 1e-6,
                        "{mode:?}: position {i} llr {} ≠ {expected}",
                        expanded[i]
                    );
                } else {
                    assert_eq!(expanded[i], 0.0, "{mode:?}: punctured position {i} not 0");
                }
            }
        }
    }

    #[test]
    fn robust_keeps_everything() {
        let keep = keep_indices(Mode::Robust);
        assert_eq!(keep, (0..N).collect::<Vec<_>>());
    }

    #[test]
    fn rates_match_published_design() {
        // Sanity check against the per-mode rates documented in the
        // module-level table.
        let cases = [
            // UltraRobust shares Robust's no-puncture pattern; the
            // rate difference is in the symbol rate, not the FEC.
            (Mode::UltraRobust, 240, 0.421),
            (Mode::Robust, 240, 0.421),
            (Mode::Standard, 202, 0.500),
            (Mode::Express, 134, 0.754),
        ];
        for (mode, expected_ch, expected_rate) in cases {
            assert_eq!(mode.ch_bits_per_block(), expected_ch);
            let rate = K_INFO as f32 / expected_ch as f32;
            assert!(
                (rate - expected_rate).abs() < 0.005,
                "{mode:?}: rate {rate:.3} ≠ {expected_rate}",
            );
        }
    }

    /// Empirical decoder-convergence sweep: for each mode, encode
    /// a random codeword, puncture per mode, expand back via
    /// `de_puncture_llr` with high-magnitude clean LLRs, and confirm
    /// the BP / OSD chain recovers the info bits.
    ///
    /// "High SNR" means the channel introduced no errors at all —
    /// the only "noise" the decoder sees is the erasure (0.0 LLR)
    /// at every punctured position. If the decoder cannot recover
    /// in this trivial channel, the puncture set has destroyed the
    /// code structure to the point where BP / OSD do not converge,
    /// and that mode cannot be shipped without a redesigned puncture
    /// pattern (or a different mother code).
    ///
    /// Failure here gates the mode from shipping; a flaky pass
    /// (< 90 %) suggests the puncture set is borderline and would
    /// need either a different selection rule or more decoder
    /// effort (deeper OSD).
    /// Generate an AWGN noise sample via Box-Muller. Deterministic
    /// for a given seed pair via xorshift-style PRNG.
    fn boxmuller(state: &mut u64) -> f32 {
        // 32-bit LCG → uniform (0, 1) — avoid the 0 boundary by
        // adding 1 before normalising.
        let mut u = || {
            *state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((((*state) >> 32) & 0xFFFF_FFFF) as f32 + 1.0) / 4_294_967_297.0
        };
        let u1: f32 = u();
        let u2: f32 = u();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }

    /// Simulate AWGN at given Eb/N0 (dB) over a punctured codeword,
    /// decode, return whether the info bits were recovered. The
    /// channel rate (n_keep_per_codeword / K_INFO) is folded into
    /// the noise calculation so that Eb/N0 comparisons across rates
    /// are honest: a rate-1/2 mode and a rate-3/4 mode at the same
    /// Eb/N0 see the same per-info-bit energy budget but different
    /// per-channel-bit noise.
    fn awgn_sweep(keep: &[usize], eb_n0_db: f32, n_trials: usize) -> (usize, usize) {
        use crate::engine::{FecCodec, FecOpts};
        use crate::fec::Ldpc240_101;

        let fec = Ldpc240_101;
        let mut bp_ok = 0usize;
        let mut osd_ok = 0usize;

        // BPSK: signal = ±1, noise N(0, σ²). Eb = 1 (per channel bit).
        // Per-info-bit Eb/N0: code rate r = K_INFO / keep.len().
        let rate = K_INFO as f32 / keep.len() as f32;
        let eb_n0_linear = 10f32.powf(eb_n0_db / 10.0);
        // N0 = Eb / (Eb/N0) ; per-channel-bit noise variance = N0 / 2 / r
        // (so per-info-bit Eb/N0 stays as advertised across rates).
        let sigma_sq = 1.0 / (2.0 * rate * eb_n0_linear);
        let sigma = sigma_sq.sqrt();

        for trial in 0..n_trials {
            let mut info_state = (trial as u64).wrapping_mul(0x9E37_79B1_5BF0_3F39);
            info_state = info_state.wrapping_add(0x1234_5678_DEAD_BEEF);
            let mut info = vec![0u8; K_INFO];
            for b in info.iter_mut() {
                info_state = info_state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                *b = ((info_state >> 33) & 1) as u8;
            }

            let mut codeword = vec![0u8; N];
            fec.encode(&info, &mut codeword);

            let mut noise_state = (trial as u64)
                .wrapping_mul(0xBF58_476D_1CE4_E5B9)
                .wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut llrs_full = vec![0.0f32; N];
            for &j in keep {
                let bit = codeword[j];
                let signal = if bit == 1 { 1.0_f32 } else { -1.0 };
                let noise = boxmuller(&mut noise_state) * sigma;
                let received = signal + noise;
                // BPSK LLR for AWGN: 2 · received / σ². Sign matches
                // bp_decode_generic's convention (LLR > 0 → bit 1).
                llrs_full[j] = 2.0 * received / sigma_sq;
            }
            // Punctured positions stay at 0.0 (erasure LLR).

            let bp_opts = FecOpts {
                bp_max_iter: 50,
                osd_depth: 0,
                ap_mask: None,
                verify_info: None,
                ..FecOpts::default()
            };
            if let Some(r) = fec.decode_soft(&llrs_full, &bp_opts)
                && r.info == info
            {
                bp_ok += 1;
                osd_ok += 1;
                continue;
            }

            let osd_opts = FecOpts {
                bp_max_iter: 50,
                osd_depth: 2,
                ap_mask: None,
                verify_info: None,
                ..FecOpts::default()
            };
            if let Some(r) = fec.decode_soft(&llrs_full, &osd_opts)
                && r.info == info
            {
                osd_ok += 1;
            }
        }

        (bp_ok, osd_ok)
    }

    /// At a clean channel (Eb/N0 = 12 dB) both selectors must
    /// converge for every shipping mode. Basic correctness gate.
    #[test]
    fn modes_decode_at_high_snr() {
        let n = 30;
        for mode in ALL_MODES {
            let unif = keep_indices_uniform(mode);
            let greedy = keep_indices_kSR_greedy(mode);
            let (u_bp, u_osd) = awgn_sweep(&unif, 12.0, n);
            let (g_bp, g_osd) = awgn_sweep(&greedy, 12.0, n);
            eprintln!(
                "{mode:?} @12dB uniform: BP {u_bp}/{n}, OSD {u_osd}/{n}; \
                 kSR-greedy: BP {g_bp}/{n}, OSD {g_osd}/{n}"
            );
            assert!(u_osd >= n * 9 / 10);
            assert!(g_osd >= n * 9 / 10);
        }
    }

    /// AWGN sweep showing whether kSR-greedy beats uniform-spread
    /// at moderate Eb/N0 (the operating point where the puncture
    /// set's quality actually shows up). Diagnostic-only — output
    /// is the data we use to decide between selectors.
    #[test]
    #[ignore = "slow: AWGN PER sweep across modes × selectors × Eb/N0; run with --ignored"]
    #[allow(non_snake_case)]
    fn modes_awgn_sweep_uniform_vs_kSR() {
        let n = 200;
        for mode in ALL_MODES {
            let unif = keep_indices_uniform(mode);
            let greedy = keep_indices_kSR_greedy(mode);
            for eb_n0_db in [-1.0, 0.0, 1.0, 2.0, 3.0, 5.0] {
                let (u_bp, u_osd) = awgn_sweep(&unif, eb_n0_db, n);
                let (g_bp, g_osd) = awgn_sweep(&greedy, eb_n0_db, n);
                eprintln!(
                    "{mode:?} Eb/N0={eb_n0_db:+.0}dB  uniform: BP {u_bp:3}/{n} OSD {u_osd:3}/{n}  \
                     kSR-greedy: BP {g_bp:3}/{n} OSD {g_osd:3}/{n}"
                );
            }
        }
    }

    /// Hypothetical rate-3/4 study: 106 parity punctures. Diagnostic
    /// only — output decides whether a fourth mode is realistic.
    #[test]
    #[ignore = "slow: experimental rate-3/4 puncturing; run with --ignored"]
    fn experimental_rate_3_4() {
        let n = 200;
        let target_punctures = 106;

        let n_keep_parity = N_PARITY - target_punctures;
        let mut unif_keep: Vec<usize> = (0..K_INFO).collect();
        for p in 0..N_PARITY {
            let cur = ((p + 1) * n_keep_parity) / N_PARITY;
            let prev = (p * n_keep_parity) / N_PARITY;
            if cur > prev {
                unif_keep.push(K_INFO + p);
            }
        }
        let greedy_keep = keep_indices_kSR_greedy_with_count(target_punctures);

        for eb_n0_db in [3.0, 5.0, 8.0, 12.0] {
            let (u_bp, u_osd) = awgn_sweep(&unif_keep, eb_n0_db, n);
            let (g_bp, g_osd) = awgn_sweep(&greedy_keep, eb_n0_db, n);
            eprintln!(
                "rate 3/4 ({target_punctures} punctures) Eb/N0={eb_n0_db:+.0}dB  \
                 uniform: BP {u_bp:3}/{n} OSD {u_osd:3}/{n}  \
                 kSR-greedy: BP {g_bp:3}/{n} OSD {g_osd:3}/{n}"
            );
        }
    }

    #[test]
    fn header_code_roundtrip() {
        for mode in ALL_MODES {
            let code = mode.header_code();
            assert!(code < 4);
            assert_eq!(Mode::from_header_code(code), Some(mode));
        }
        // Codes 4..=255 are invalid (the field is only 2 bits).
        for code in 4u8..=255 {
            assert_eq!(
                Mode::from_header_code(code),
                None,
                "invalid code {code} decoded successfully",
            );
        }
    }
}
