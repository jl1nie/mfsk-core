//! Stochastic Chase decoder — faithful port of WSJT-X's `ftrsdap`
//! (`lib/ftrsd/ftrsdap.c`, called from `lib/extract.f90`), issue #169.
//!
//! [`super::decode_at_with_erasures`] tries exactly one deterministic,
//! increasing-erasure-count ladder over a single confidence ordering.
//! Real WSJT-X (`jt9 -6`, no `kvasd`) instead runs a *randomized*
//! multi-trial search: many trials, each marking a different random
//! subset of low-confidence positions as erasures (weighted by a
//! per-position erasure-probability table keyed on both the
//! most-vs-second-most-reliable-tone confidence ratio and the
//! position's overall reliability rank), scoring every successful
//! trial's codeword by re-projecting it back onto the original raw
//! spectrum, and keeping the best- and second-best-scoring candidates
//! to decide (via a margin gate) whether to accept anything at all.
//! This module ports that algorithm literally — magic numbers
//! included — rather than approximating its shape:
//!
//! - **Erasure-probability table**: `PERR`, WSJT-X's own hand-tuned
//!   `perr[8][8]` lookup (`ftrsdap.c:43-51`), keyed by a confidence-
//!   ratio bucket and a reliability-rank bucket, scaled ×1.3.
//! - **Candidate quality metric**: `getpp`, a literal port of
//!   `extract.f90`'s `getpp` subroutine — re-encodes a successful
//!   trial's codeword, walks it back through the same
//!   interleave+Gray-encode the transmitter uses, and averages the
//!   *original* (pre-decision) FFT-bin power at the resulting 63
//!   (position, tone) pairs. Needs the raw un-thresholded spectrum,
//!   which [`super::rx::demodulate_aligned_with_runnerup`] now retains
//!   for exactly this purpose (`raw_pwr`, ~16 KB — JT65 already
//!   requires `std`/`fft-rustfft`, so this isn't an embedded concern).
//! - **`nhard`/`nsoft`/`ntotal`**: `ftrsdap.c`'s literal soft-distance
//!   formula, using the retained runner-up-tone identity
//!   ([`super::rx::demodulate_aligned_with_runnerup`]) exactly as
//!   WSJT-X's `rxdat2`/`mr2sym` does.
//! - **Acceptance gate**: `ntotal ≤ nd0 && pp2/pp1 ≤ r0`
//!   (`extract.f90:169-176`), literal `nd0=81`/`r0=0.87` defaults
//!   (WSJT-X's "normal" aggressiveness; `83`/`0.90` is its
//!   "aggressive" pair, both exposed via [`ChaseParams`]).
//! - **Early exit**: `nhard ≤ 41 && ntotal ≤ 71` (`ftrsdap.c:211`).
//! - **RNG**: the exact POSIX-style LCG recurrence and `ir` extraction
//!   `ftrsdap.c` uses, reseeded once per decode attempt.
//! - **Trial count**: `1000`, matching WSJT-X's own `jt9 -6` (the
//!   exact build this crate's JT65 sensitivity is benchmarked against
//!   in `docs/notes/BENCHMARKS.md`) via `decoder.f90`'s
//!   `nranera=6 → ntrials=10**(6/2)=1000` formula.
//!
//! Not ported: WSJT-X's AP-hint passes (`extract.f90`'s `ipass` loop,
//! reusing `ftrsdap` with pinned symbols) and the `hint65` correlation
//! fallback — out of scope for this pass, tracked as possible future
//! work if requested. The measured effect of this port on JT65
//! sensitivity is reported honestly in `docs/notes/BENCHMARKS.md`,
//! whatever it turns out to be — this doc comment describes the
//! *algorithm* being ported, not a promised outcome.
//!
//! ## Correction: `rel` vs. `conf` (same-day follow-up)
//!
//! The reliability metric this module actually needs is WSJT-X
//! `demod64a.f90`'s `mrprob` — `best_pwr / total_pwr` across **all 64**
//! tones at a position, an SNR-like peakiness measure that reflects
//! how much energy leaked into the noise floor everywhere, not just
//! into the runner-up tone. An initial version of this port instead
//! reused [`super::rx`]'s pre-existing `conf` (`(best−second)/best`, a
//! top-2-only margin) for the erasure-priority ordering and the
//! `nsoft` weighting — a real mismatch, not just a naming difference:
//! two positions with the same top-2 margin can have very different
//! `mrprob` if one has a quiet noise floor and the other doesn't, and
//! `conf` can't tell them apart. Fixed by adding `rel` (WSJT-X's real
//! `mrprob`) to [`super::rx::demodulate_aligned_with_runnerup`]'s
//! return and using it everywhere WSJT-X uses `rxprob`/`mrprob` —
//! `conf` is *still* correct and still used for the `PERR` table's
//! `ii` ratio bucket, since `rxprob2/rxprob` algebraically reduces to
//! `second_pwr/best_pwr` regardless of the `total_pwr` normalization,
//! which is exactly what `1 - conf` already gives. Measured effect:
//! a further ~0.9 dB of 50%-crossing improvement — see
//! `docs/notes/BENCHMARKS.md`'s JT65 section for the corrected numbers.

use crate::engine::{DecodeContext, MessageCodec};
use crate::fec::Rs63_12;
use crate::msg::{Jt72Codec, Jt72Message};

use super::gray::gray6;
use super::interleave::interleave;
use super::rx;

/// Tunable parameters for [`decode_at_with_chase`]. Plain public
/// fields + `Default`, matching [`super::search::SearchParams`]'s
/// shape — JT65 has no builder-pattern precedent
/// (`msg::decode_request`'s doc comment explicitly scopes JT65 out of
/// that redesign, along with Q65/WSPR/JT9/uvpacket, since each keeps
/// its own bespoke decode entry points). Defaults are WSJT-X's own
/// literal constants (see module doc) — this is a faithful port, not
/// an approximation left to be tuned.
#[derive(Clone, Debug)]
pub struct ChaseParams {
    /// Max randomized erasure-pattern trials to run if the fast
    /// zero-erasure path fails. Default `1000` matches WSJT-X's own
    /// `jt9 -6` (`nranera=6`).
    pub max_trials: usize,
    /// LCG seed. Deterministic and reproducible by design — same
    /// audio in ⇒ same result out, every time, on every platform.
    /// Matches WSJT-X's own choice (`ftrsdap.c` reseeds to 1 every
    /// call) rather than being a corner-cutting shortcut.
    pub seed: u32,
    /// Hard cap on erasures marked per trial. Must not exceed
    /// [`Rs63_12::NROOTS`] (51) — the RS(63,12) code's erasure-only
    /// correction bound; matches `ftrsdap.c`'s `numera < 51` cap.
    pub max_erasures: usize,
    /// Early-exit trigger (`ftrsdap.c:211`, hardcoded there — exposed
    /// here for experimentation, defaulted to the literal values):
    /// stop once the leading candidate's `nhard` and `ntotal` are both
    /// at or below these.
    pub early_exit_nhard: u32,
    pub early_exit_ntotal: f32,
    /// Final acceptance gate (`extract.f90:169-176`): accept only if
    /// the leading candidate's `ntotal` is at or below `nd0` **and**
    /// its soft-distance margin over the runner-up (`pp2/pp1`) is at
    /// or below `r0`. Defaults are WSJT-X's "normal" aggressiveness
    /// pair; its "aggressive" pair is `(83, 0.90)`.
    pub nd0: u32,
    pub r0: f32,
}

impl Default for ChaseParams {
    fn default() -> Self {
        Self {
            max_trials: 1000,
            seed: 1,
            max_erasures: Rs63_12::NROOTS,
            early_exit_nhard: 41,
            early_exit_ntotal: 71.0,
            nd0: 81,
            r0: 0.87,
        }
    }
}

/// WSJT-X `ftrsdap.c:43-51`'s hand-tuned erasure-probability table
/// (percent, pre-×1.3 scale), literal. Row `ii` = confidence-ratio
/// bucket (`0` = winning tone dominates its runner-up, `7` = the top
/// two are nearly tied); column `jj` = reliability-rank bucket within
/// the 63 positions (`0` = one of the most-reliable positions overall,
/// `7` = one of the least-reliable). Described in the source only as
/// "power-percentage symbol metrics — composite gnnf/hf"; empirically
/// fitted, not first-principles-derived (see `WSJT-X/lib/ftrsd/ftrsd_paper/`
/// for the underlying WER study).
#[rustfmt::skip]
const PERR: [[f32; 8]; 8] = [
    [ 4.0,  9.0, 11.0, 13.0, 14.0, 14.0, 15.0, 15.0],
    [ 2.0, 20.0, 20.0, 30.0, 40.0, 50.0, 50.0, 50.0],
    [ 7.0, 24.0, 27.0, 40.0, 50.0, 50.0, 50.0, 50.0],
    [13.0, 25.0, 35.0, 46.0, 52.0, 70.0, 50.0, 50.0],
    [17.0, 30.0, 42.0, 54.0, 55.0, 64.0, 71.0, 70.0],
    [25.0, 39.0, 48.0, 57.0, 64.0, 66.0, 77.0, 77.0],
    [32.0, 45.0, 54.0, 63.0, 66.0, 75.0, 78.0, 83.0],
    [51.0, 58.0, 57.0, 66.0, 72.0, 77.0, 82.0, 86.0],
];

/// Per-trial erasure threshold (percent, `0..≈112` — some `PERR`
/// cells ×1.3 exceed 100, meaning "always erase", same as WSJT-X)
/// for each position in `order` (least→most confident, `order[0]` =
/// worst). Computed once per decode attempt, outside the trial loop —
/// matches `ftrsdap.c:128-147`'s setup loop, which runs once before
/// `for k=1..=ntrials`.
///
/// `ratio` (second-best / best tone confidence at a position) is
/// `1 - conf[pos]` given this crate's `conf = (best-second)/best`
/// convention — algebraically the same ratio `ftrsdap.c` computes as
/// `rxprob2[j]/(rxprob[j]+0.01)`, so no separate raw-probability pair
/// is needed here.
fn build_thresh0(order: &[usize], conf: &[f32; 63]) -> Vec<f32> {
    order
        .iter()
        .enumerate()
        .map(|(i, &pos)| {
            let ratio = (1.0 - conf[pos]).clamp(0.0, 1.0);
            let ii = ((7.999 * ratio) as usize).min(7);
            let jj = ((62 - i) / 8).min(7);
            1.3 * PERR[ii][jj]
        })
        .collect()
}

/// Ordering of the 63 symbol positions from least → most confident.
/// Shared by [`super::decode_at_with_erasures`] and
/// [`decode_at_with_chase`] so both erasure strategies agree on what
/// "least reliable" means; also the same rank WSJT-X's own descending
/// `probs`/`indexes` sort (`ftrsdap.c:75-92`) produces, just read from
/// the opposite end (their `indexes[62-i]` for `i=0..62` is this
/// crate's `order[i]`).
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
/// and `ir` extraction WSJT-X's `ftrsdap.c` uses:
/// ```text
/// nseed = nseed*1103515245 + 12345
/// ir = (unsigned)(nseed/65536) % 32768
/// ir = (100*ir)/32768
/// ```
/// The same core recurrence was already present test-only in
/// `fec/ldpc/bp.rs`; promoted to production use here. Deterministic
/// and seedable by design (see [`ChaseParams::seed`]'s doc comment) —
/// WSJT-X itself reseeds to `1` on every `ftrsdap` call, making its
/// own decode fully reproducible per input; this port preserves that.
struct Lcg(u32);

impl Lcg {
    fn new(seed: u32) -> Self {
        Self(seed)
    }

    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        self.0
    }

    /// Draw in `0..100`, matching `ftrsdap.c`'s exact `ir` formula
    /// (not a simpler `% 100` shortcut — the intermediate `% 32768`
    /// step and truncating `*100/32768` division both affect which
    /// values are reachable, so are ported literally for bit-for-bit
    /// draw-sequence fidelity to the original, seed-for-seed).
    fn next_pct(&mut self) -> u32 {
        let ir = (self.next_u32() / 65536) % 32768;
        (100 * ir) / 32768
    }
}

/// Literal port of `extract.f90`'s `getpp` subroutine: re-derives the
/// channel-order tone sequence a candidate codeword would have
/// transmitted, then averages the *original* raw FFT-bin power at
/// those (position, tone) pairs. This is WSJT-X's actual candidate
/// quality metric (`pp`) — distinct from `nhard`/`ntotal`, which only
/// gate the early exit and final acceptance, not the pp1/pp2 ranking
/// itself.
///
/// `cand_sent` is in the same WSJT `sent[]` layout as `symbols`
/// (RS-codeword order, i.e. what [`super::super::decode_at_with_erasures`]
/// and [`Rs63_12::decode_jt65_erasures`]/`encode_jt65` operate on).
/// `raw_pwr` is [`super::rx::demodulate_aligned_with_runnerup`]'s
/// retained pre-decision spectrum, indexed `[temporal position][tone]`
/// — see that function's doc for why this specific order matches
/// WSJT-X's `s3a`.
///
/// Derivation note (not in any doc comment on the WSJT-X side, worked
/// out from the Fortran source): `getpp`'s own `a = reverse(workdat)`
/// step exactly cancels `ftrsdap.c`'s earlier `workdat[i] =
/// mrsym[62-i]` reversal, so — restated in this crate's un-reversed
/// `sent[]` convention — the net transform is simply "interleave, then
/// Gray-encode", identical in shape to `tx::encode_channel_symbols`'s
/// own TX pipeline.
fn getpp(cand_sent: &[u8; 63], raw_pwr: &[[f32; 64]; 63]) -> f32 {
    let mut a = *cand_sent;
    interleave(&mut a);
    for x in a.iter_mut() {
        *x = gray6(*x);
    }
    let mut psum = 0.0f32;
    for (j, &tone) in a.iter().enumerate() {
        psum += raw_pwr[j][tone as usize];
    }
    psum / 63.0
}

fn unpack_jt72(info: &[u8; 12]) -> Option<Jt72Message> {
    let mut payload = [0u8; 72];
    for (i, bit) in payload.iter_mut().enumerate() {
        let word = info[i / 6];
        let shift = 5 - (i % 6);
        *bit = (word >> shift) & 1;
    }
    Jt72Codec::default().unpack(&payload, &DecodeContext::default())
}

/// The best (highest-`pp`) candidate seen so far in the trial loop.
struct Best {
    info: [u8; 12],
    nhard: u32,
    ntotal: f32,
}

/// Decode a JT65 signal at a known alignment via randomized multi-trial
/// erasure search — see module doc for the full algorithm, a literal
/// port of WSJT-X's `ftrsdap`.
///
/// Tries the fast zero-erasure path first (matches
/// [`super::decode_at`]/[`super::decode_at_with_erasures`]'s `n_eras =
/// 0` case and `ftrsdap.c`'s own unconditional first attempt; free, no
/// RNG spent). If that fails, runs up to `params.max_trials`
/// randomized erasure trials, tracks the best- and second-best-scoring
/// successful candidates by `getpp`, and applies WSJT-X's own
/// `ntotal ≤ nd0 && pp2/pp1 ≤ r0` acceptance gate before returning —
/// an ambiguous result (best and runner-up too close in quality)
/// returns `None` rather than guessing, since random erasure trials
/// can occasionally satisfy the RS syndrome for a *wrong* codeword
/// (see `chase_never_false_decodes_*` in this module's tests).
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
    let (symbols, conf, second_sym, rel, raw_pwr, snr_db) =
        rx::demodulate_aligned_with_runnerup(audio, sample_rate, start_sample, base_freq_hz)?;

    let rs = Rs63_12::new();

    // Fast path: plain zero-erasure hard decision, no RNG spent.
    // `ftrsdap.c:94-115` tries this unconditionally before any
    // randomized trial and returns immediately on success.
    if let Some((info, _nerr)) = rs.decode_jt65_erasures(&symbols, &[])
        && let Some(msg) = unpack_jt72(&info)
    {
        return Some((msg, snr_db));
    }

    // `ftrsdap.c:149`: `if(nsum<=0) return;` — no usable reliability
    // signal at all (e.g. a fully silent slot). `nsum` sums `rel`
    // (WSJT-X's real `rxprob`/`mrprob`), **not** `conf` — see `rel`'s
    // doc comment on `DemodWithRunnerup` for why these differ and why
    // `rel` is the correct quantity here.
    let nsum: f32 = rel.iter().sum();
    if nsum <= 0.0 {
        return None;
    }

    let order = confidence_order(&rel);
    let thresh0 = build_thresh0(&order, &conf);
    let max_erasures = params.max_erasures.min(Rs63_12::NROOTS);
    let mut lcg = Lcg::new(params.seed);

    // pp1/pp2 = best/second-best candidate quality seen across all
    // trials (`ftrsdap.c:151-214`) — not a per-message tally. Two
    // *different* trials can (and often do) land on the same message
    // via different erasure sets; WSJT-X doesn't count or care, it
    // only ever compares raw `pp` scores.
    let mut pp1 = 0.0f32;
    let mut pp2 = 0.0f32;
    let mut best: Option<Best> = None;

    for _trial in 1..=params.max_trials {
        let mut eras: Vec<u32> = Vec::with_capacity(max_erasures);
        // Walk worst→best every trial (not stopping early on the walk
        // itself) so the draw sequence stays reproducible trial-to-
        // trial; only the *marking* is capped at `max_erasures`,
        // matching `ftrsdap.c:173`'s `numera < 51` guard.
        for (i, &pos) in order.iter().enumerate() {
            let draw = lcg.next_pct();
            if (draw as f32) < thresh0[i] && eras.len() < max_erasures {
                eras.push(pos as u32);
            }
        }

        let Some((info, _nerr)) = rs.decode_jt65_erasures(&symbols, &eras) else {
            continue;
        };
        let cand_sent = rs.encode_jt65(&info);

        // Literal `nhard`/`nsoft`/`ntotal` (`ftrsdap.c:184-196`): a
        // position that differs from the received hard decision but
        // *matches* the runner-up guess contributes to `nhard` only,
        // not `nsoft` — a cheap correction. One that matches neither
        // costs its full `rel` weight (WSJT-X's `rxprob[i]`, not the
        // top-2 margin `conf` — same distinction as `order` above).
        let mut nhard = 0u32;
        let mut nsoft_raw = 0.0f32;
        for i in 0..63 {
            if cand_sent[i] != symbols[i] {
                nhard += 1;
                if cand_sent[i] != second_sym[i] {
                    nsoft_raw += rel[i];
                }
            }
        }
        let nsoft = 63.0 * nsoft_raw / nsum;
        let ntotal = nhard as f32 + nsoft;

        let pp = getpp(&cand_sent, &raw_pwr);
        if pp > pp1 {
            pp2 = pp1;
            pp1 = pp;
            best = Some(Best {
                info,
                nhard,
                ntotal,
            });
        } else if pp > pp2 && pp != pp1 {
            pp2 = pp;
        }

        if let Some(b) = &best
            && b.nhard <= params.early_exit_nhard
            && b.ntotal <= params.early_exit_ntotal
        {
            break;
        }
    }

    let best = best?;
    // `extract.f90:169-176`'s acceptance gate. `rtt=0.0` (trivially
    // `<= r0`) when only one distinct `pp` value was ever seen —
    // matches WSJT-X's own behaviour, where `pp2` never leaves its
    // `0.0` initial value in that case.
    let rtt = if pp1 > 0.0 { pp2 / pp1 } else { 0.0 };
    if (best.ntotal > params.nd0 as f32) || (rtt > params.r0) {
        return None;
    }
    let msg = unpack_jt72(&best.info)?;
    Some((msg, snr_db))
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
    /// some unlucky draw — this is exactly the risk WSJT-X's own
    /// `ntotal`/`pp2/pp1` acceptance gate exists to catch; this test
    /// verifies the ported gate actually catches it too. Deliberately
    /// bounded to `decode_at_with_chase` at a fixed (start_sample,
    /// freq_hz) rather than the full `decode_scan_chase` — a
    /// coarse-search-driven scan over noise can turn up many spurious
    /// candidates, each burning up to `max_trials` RS-decode attempts,
    /// which would make the default (non-`--ignored`) test suite slow.
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
