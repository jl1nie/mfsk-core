//! Stochastic Chase decoder — port of WSJT-X's `ftrsdap`
//! (`lib/ftrsd/ftrsdap.c`), issue #169.
//!
//! [`super::decode_at_with_erasures`] tries exactly one deterministic,
//! increasing-erasure-count ladder over a single confidence ordering.
//! Real WSJT-X (`jt9 -6`, no `kvasd`) instead runs a *randomized*
//! multi-trial search: many trials, each marking a different random
//! subset of low-confidence positions as erasures (weighted by how
//! close each position's top-2 candidate tones are), keeping the
//! best-ranked successful candidate across all trials, then applying
//! an acceptance-margin gate before returning anything. This is a
//! materially stronger search of the same RS(63,12) erasure-correction
//! space — see `docs/notes/BENCHMARKS.md`'s JT65 section for the full
//! root-cause writeup this module closes.
//!
//! This ports the *algorithmic shape*, not WSJT-X's literal hand-tuned
//! magic numbers (`ftrsdap.c`'s `perr[8][8]` erasure-threshold table,
//! `nd0`/`r0` acceptance thresholds) — those came from a dedicated
//! empirical word-error-rate study (`WSJT-X/lib/ftrsd/ftrsd_paper/`)
//! that isn't reproducible from first principles, and this crate's
//! demodulator confidence distribution isn't numerically identical to
//! WSJT-X's raw power ratios. [`ChaseParams`]'s defaults are starting
//! points, retuned empirically against `tests/jt65_sweep.rs`.
//!
//! Quality-ranking design note: WSJT-X's `pp` metric re-consults the
//! *original* per-position×64-tone spectrum (via a callback into
//! Fortran-side state) to rank candidates. This module deliberately
//! does not thread that spectrum through — it's a real architectural
//! change (the demod loop would need to survive past its current
//! per-symbol scope) for a benefit `nerr` (already returned free by
//! [`crate::fec::Rs63_12::decode_jt65_erasures`]) plus a soft-distance
//! proxy using the retained runner-up tone identity
//! ([`super::rx::demodulate_aligned_with_runnerup`]) already gets
//! mostly for free — WSJT-X's own early-exit is itself count-based, so
//! `pp` is only doing real work as a *tiebreaker* among trials that
//! already passed a count check, which is exactly the role given to
//! `cost` here.

use crate::engine::{DecodeContext, MessageCodec};
use crate::fec::Rs63_12;
use crate::msg::{Jt72Codec, Jt72Message};

use super::rx;

/// Tunable parameters for [`decode_at_with_chase`]. Plain public
/// fields + `Default`, matching [`super::search::SearchParams`]'s
/// shape — JT65 has no builder-pattern precedent
/// (`msg::decode_request`'s doc comment explicitly scopes JT65 out of
/// that redesign, along with Q65/WSPR/JT9/uvpacket, since each keeps
/// its own bespoke decode entry points).
#[derive(Clone, Debug)]
pub struct ChaseParams {
    /// Max randomized erasure-pattern trials to run if the fast
    /// zero-erasure path fails. WSJT-X's own range is roughly
    /// 1000-100000 depending on its "aggressive decoding" UI setting;
    /// this default is a conservative starting point (see module doc).
    pub max_trials: usize,
    /// LCG seed. Deterministic and reproducible by design — same
    /// audio in ⇒ same result out, every time, on every platform.
    /// Matches WSJT-X's own choice (`ftrsdap.c` reseeds to 1 every
    /// call) rather than being a corner-cutting shortcut.
    pub seed: u32,
    /// Hard cap on erasures marked per trial. Must not exceed
    /// [`Rs63_12::NROOTS`] (51) — the RS(63,12) code's erasure-only
    /// correction bound.
    pub max_erasures: usize,
    /// Early-exit trigger: stop once the leading candidate has been
    /// independently reproduced by at least this many distinct trials
    /// with `nerr` at or below `early_exit_nerr`.
    pub early_exit_hits: u32,
    pub early_exit_nerr: u32,
    /// Acceptance-margin gate (see [`decode_at_with_chase`]): when two
    /// or more distinct candidate messages are found, the leading one
    /// is accepted only if it clears the runner-up by at least this
    /// hit-count ratio...
    pub min_hit_margin_ratio: f32,
    /// ...or by at least this `nerr` margin (whichever margin is
    /// easier to satisfy accepts the decode; failing both returns
    /// `None` rather than guessing between two plausible codewords).
    pub min_nerr_margin: u32,
}

impl Default for ChaseParams {
    fn default() -> Self {
        Self {
            max_trials: 2000,
            seed: 1,
            max_erasures: Rs63_12::NROOTS,
            early_exit_hits: 3,
            early_exit_nerr: 10,
            min_hit_margin_ratio: 2.0,
            min_nerr_margin: 4,
        }
    }
}

/// Cost weight for a corrected position that lands on the received
/// hard symbol's runner-up guess — cheap, mirrors WSJT-X `ftrsdap`'s
/// `nsoft` check (`workdat[i] != rxdat2[i]`). Module-private, same
/// "starting point, tune against the sweep" status as `ChaseParams`'s
/// numeric defaults.
const W_RUNNERUP: f32 = 0.5;
/// Cost weight for a corrected position that agrees with neither the
/// received hard symbol nor its runner-up — expensive.
const W_OTHER: f32 = 1.0;

/// Erasure-threshold ceiling: even a maximally-ambiguous position
/// (`conf == 0`) is marked an erasure at most this often per trial.
/// WSJT-X's own `perr` table tops out below 1.0 too (its largest
/// entries are in the 80s out of 100).
const CHASE_PERR_MAX: f32 = 0.85;
/// Shape exponent on `(1 - conf)`. `1.0` = linear. A more aggressive
/// concave curve (`< 1.0`, marking moderate-confidence positions as
/// erasures more readily) was tried during tuning against
/// `tests/jt65_sweep.rs` and measured *worse* mid-range recall (more
/// erasures raises the odds of exceeding the RS(63,12) correction
/// bound before the true low-confidence positions are even reached) —
/// kept linear.
const CHASE_PERR_GAMMA: f32 = 1.0;

/// Erasure probability in `[0, 1]` for a position with confidence
/// `conf` (`demodulate_aligned_with_confidence`'s convention: 0 = top
/// two tones tied, 1 = winner dominates). Lower confidence ⇒ higher
/// erasure probability. `conf` already captures what WSJT-X's
/// `perr[8][8]` ratio-lookup table captures (`conf ≈ 1 −
/// second/best`), so no separate table is needed — see module doc.
fn erasure_probability(conf: f32) -> f32 {
    (CHASE_PERR_MAX * (1.0 - conf).clamp(0.0, 1.0).powf(CHASE_PERR_GAMMA)).clamp(0.0, 1.0)
}

/// Ordering of the 63 symbol positions from least → most confident.
/// Shared by [`super::decode_at_with_erasures`] and
/// [`decode_at_with_chase`] so both erasure strategies agree on what
/// "least reliable" means.
pub(super) fn confidence_order(conf: &[f32; 63]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..63).collect();
    order.sort_by(|&a, &b| {
        conf[a]
            .partial_cmp(&conf[b])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    order
}

/// A POSIX-style linear congruential generator — the exact recurrence
/// WSJT-X's `ftrsdap.c` uses (`nseed = nseed*1103515245 + 12345`), and
/// already present test-only in `fec/ldpc/bp.rs`. Promoted to
/// production use here. Deterministic and seedable by design (see
/// [`ChaseParams::seed`]'s doc comment).
struct Lcg(u32);

impl Lcg {
    fn new(seed: u32) -> Self {
        Self(seed)
    }

    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        self.0
    }

    /// Draw in `0..100`. POSIX-style LCGs have weak low-order bits, so
    /// extract from the high half (same precaution `ftrsdap.c` takes:
    /// `ir = (unsigned)(nseed/65536) % 32768`).
    fn next_pct(&mut self) -> u32 {
        (self.next_u32() >> 16) % 100
    }
}

/// One RS-decode success found during the trial loop.
struct Candidate {
    message: Jt72Message,
    nerr: u32,
    cost: f32,
    hit_count: u32,
}

/// Ranking order for candidates: most-reproduced first, then fewest
/// RS corrections, then lowest soft-distance cost. Shared by the
/// trial loop's early-exit check and the final acceptance sort so
/// both agree on what "leading candidate" means.
fn candidate_rank(a: &Candidate, b: &Candidate) -> core::cmp::Ordering {
    b.hit_count
        .cmp(&a.hit_count)
        .then(a.nerr.cmp(&b.nerr))
        .then(
            a.cost
                .partial_cmp(&b.cost)
                .unwrap_or(core::cmp::Ordering::Equal),
        )
}

/// Decode a JT65 signal at a known alignment via randomized multi-trial
/// erasure search — see module doc for the full algorithm and its
/// relationship to WSJT-X's `ftrsdap`.
///
/// Tries the fast zero-erasure path first (matches
/// [`super::decode_at`]/[`super::decode_at_with_erasures`]'s `n_eras =
/// 0` case; free, no RNG spent). If that fails, runs up to
/// `params.max_trials` randomized erasure trials, ranks any successful
/// candidates by `(hit_count, nerr, cost)`, and applies an
/// acceptance-margin gate before returning — an ambiguous result
/// (two candidates too close to call) returns `None` rather than
/// guessing, since random erasure trials can occasionally satisfy the
/// RS syndrome for a *wrong* codeword (see `chase_never_false_decodes_*`
/// in this module's tests).
pub fn decode_at_with_chase(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    params: &ChaseParams,
) -> Option<Jt72Message> {
    decode_at_with_chase_and_snr(audio, sample_rate, start_sample, base_freq_hz, params)
        .map(|(msg, _snr)| msg)
}

/// Like [`decode_at_with_chase`] but also returns the decode-side SNR
/// estimate, for [`super::decode_scan_chase`]'s [`super::Jt65Result`]
/// wiring — mirrors `super::decode_at_with_snr`'s private-sibling
/// shape.
pub(super) fn decode_at_with_chase_and_snr(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    params: &ChaseParams,
) -> Option<(Jt72Message, f32)> {
    let (symbols, conf, second_sym, snr_db) =
        rx::demodulate_aligned_with_runnerup(audio, sample_rate, start_sample, base_freq_hz)?;

    let rs = Rs63_12::new();
    let codec = Jt72Codec::default();
    let ctx = DecodeContext::default();

    let unpack = |info: &[u8; 12]| -> Option<Jt72Message> {
        let mut payload = [0u8; 72];
        for (i, bit) in payload.iter_mut().enumerate() {
            let word = info[i / 6];
            let shift = 5 - (i % 6);
            *bit = (word >> shift) & 1;
        }
        codec.unpack(&payload, &ctx)
    };

    // Fast path: plain zero-erasure hard decision, no RNG spent.
    if let Some((info, _nerr)) = rs.decode_jt65_erasures(&symbols, &[])
        && let Some(msg) = unpack(&info)
    {
        return Some((msg, snr_db));
    }

    let order = confidence_order(&conf);
    let max_erasures = params.max_erasures.min(Rs63_12::NROOTS);
    let mut lcg = Lcg::new(params.seed);
    let mut candidates: alloc::vec::Vec<Candidate> = alloc::vec::Vec::new();

    'trials: for _trial in 0..params.max_trials {
        let mut eras: alloc::vec::Vec<u32> = alloc::vec::Vec::with_capacity(max_erasures);
        // Walk worst→best every trial (not stopping early on the walk
        // itself) so the draw sequence stays reproducible trial-to-
        // trial; only the *marking* is capped at `max_erasures`.
        for &pos in &order {
            let draw = lcg.next_pct();
            let threshold = (100.0 * erasure_probability(conf[pos])) as u32;
            if draw < threshold && eras.len() < max_erasures {
                eras.push(pos as u32);
            }
        }

        let Some((info, nerr)) = rs.decode_jt65_erasures(&symbols, &eras) else {
            continue;
        };
        let Some(message) = unpack(&info) else {
            continue;
        };

        let cand_sent = rs.encode_jt65(&info);
        let mut cost = 0.0f32;
        for i in 0..63 {
            if cand_sent[i] == symbols[i] {
                // Matches received hard decision — free.
            } else if cand_sent[i] == second_sym[i] {
                cost += W_RUNNERUP * conf[i];
            } else {
                cost += W_OTHER * conf[i];
            }
        }

        if let Some(existing) = candidates.iter_mut().find(|c| c.message == message) {
            existing.hit_count += 1;
            // Keep this trial's (nerr, cost) only if it's a strictly
            // better read of the same message than what we already had.
            let better = nerr < existing.nerr || (nerr == existing.nerr && cost < existing.cost);
            if better {
                existing.nerr = nerr;
                existing.cost = cost;
            }
        } else {
            candidates.push(Candidate {
                message,
                nerr,
                cost,
                hit_count: 1,
            });
        }

        let best = candidates
            .iter()
            .min_by(|a, b| candidate_rank(a, b))
            .expect("just pushed at least one candidate");
        if best.hit_count >= params.early_exit_hits && best.nerr <= params.early_exit_nerr {
            break 'trials;
        }
    }

    candidates.sort_by(candidate_rank);

    match candidates.len() {
        0 => None,
        1 => Some((candidates.into_iter().next().unwrap().message, snr_db)),
        _ => {
            let top = &candidates[0];
            let runner_up = &candidates[1];
            let margin_ok = (top.hit_count as f32)
                >= (runner_up.hit_count as f32) * params.min_hit_margin_ratio
                || runner_up.nerr.saturating_sub(top.nerr) >= params.min_nerr_margin;
            if margin_ok {
                Some((candidates.into_iter().next().unwrap().message, snr_db))
            } else {
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jt65::tx::synthesize_standard;

    #[test]
    fn chase_decodes_clean_synth_via_fast_path() {
        let freq = 1270.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let msg = decode_at_with_chase(&audio, 12_000, 0, freq, &ChaseParams::default())
            .expect("chase decoder must decode clean synth via the fast zero-erasure path");
        assert!(matches!(
            msg,
            Jt72Message::Standard { ref call1, ref call2, ref grid_or_report }
                if call1 == "CQ" && call2 == "K1ABC" && grid_or_report == "FN42"
        ));
    }

    /// A tiny xorshift32 + Box-Muller Gaussian generator, local to this
    /// test module — matches the existing "own local struct, not a
    /// shared helper" convention already used independently in
    /// `msk144/decode.rs` and `ft8/decode.rs`'s own test modules.
    struct NoiseGen(u32);
    impl NoiseGen {
        fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            x
        }
        fn next_f32(&mut self) -> f32 {
            (self.next_u32() as f32) / (u32::MAX as f32)
        }
        fn gaussian(&mut self) -> f32 {
            let u1 = self.next_f32().max(1e-9);
            let u2 = self.next_f32();
            (-2.0 * u1.ln()).sqrt() * (2.0 * core::f32::consts::PI * u2).cos()
        }
        fn fill_noise(&mut self, buf: &mut [f32], amplitude: f32) {
            for s in buf.iter_mut() {
                *s = amplitude * self.gaussian();
            }
        }
    }

    /// New false-decode risk surface specific to this module: unlike
    /// [`super::super::decode_at_with_erasures`]'s single deterministic
    /// erasure ordering (which can only ever explore one nested
    /// prefix sequence), a randomized multi-trial search can in
    /// principle satisfy the RS syndrome for a *wrong* codeword on
    /// some unlucky draw. Deliberately bounded to `decode_at_with_chase`
    /// at a fixed (start_sample, freq_hz) rather than the full
    /// `decode_scan_chase` — a coarse-search-driven scan over noise can
    /// turn up many spurious candidates, each burning up to
    /// `max_trials` RS-decode attempts, which would make the default
    /// (non-`--ignored`) test suite slow.
    #[test]
    fn chase_never_false_decodes_on_pure_noise() {
        const NSAMPLES: usize = 126 * 4460; // one 46.8 s JT65 frame at 12 kHz
        for seed in 1..=20u32 {
            let mut rng = NoiseGen(seed.wrapping_mul(2_654_435_761) | 1);
            let mut audio = vec![0.0f32; NSAMPLES];
            rng.fill_noise(&mut audio, 0.3);
            let msg = decode_at_with_chase(&audio, 12_000, 0, 1270.0, &ChaseParams::default());
            assert!(
                msg.is_none(),
                "chase decoder must not decode pure noise (seed={seed}), got {msg:?}"
            );
        }
    }

    /// Same guardrail, against a genuine signal buried far below the
    /// sweep's known-impossible floor (< -25 dB, see
    /// `tests/jt65_sweep.rs`'s recall table) rather than pure noise —
    /// covers the case where a real (but hopelessly weak) carrier's
    /// structure could in principle bias trials toward a plausible-but-
    /// wrong codeword differently than unstructured noise would.
    #[test]
    fn chase_never_false_decodes_below_floor_snr() {
        let freq = 1270.0;
        let clean = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        for seed in 1..=20u32 {
            let mut rng = NoiseGen(seed.wrapping_mul(2_654_435_761) | 1);
            let mut audio = clean.clone();
            // Scale the signal down ~40 dB (×0.01) and add full-scale
            // noise on top — well below the -25 dB floor where the
            // AWGN sweep already shows 0% recall for every decoder.
            for s in audio.iter_mut() {
                *s *= 0.01;
            }
            let mut noise = vec![0.0f32; audio.len()];
            rng.fill_noise(&mut noise, 0.3);
            for (s, n) in audio.iter_mut().zip(noise.iter()) {
                *s += n;
            }
            let msg = decode_at_with_chase(&audio, 12_000, 0, freq, &ChaseParams::default());
            assert!(
                msg.is_none(),
                "chase decoder must not decode a signal this deep below the floor (seed={seed}), got {msg:?}"
            );
        }
    }
}
