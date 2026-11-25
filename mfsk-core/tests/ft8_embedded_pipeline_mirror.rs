//! The CoreS3 FT8 per-slot decode, reproduced call for call on the host.
//!
//! `ft8_embedded_grid_phase.rs` measures phase against `decode_block`,
//! which is the board's single-pass driver but not the board's
//! *pipeline*: it searches +-2.5 s where the board searches +-1.0 s,
//! it searches from 200 Hz where the board's `DecodeConfig` starts at
//! 100, and it has its own pass-1 limit and refine where the board
//! splits candidates across two cores and an audio prefix. Scored on
//! that instrument, the best phase it could find decoded 7 inside the
//! board's window — while the board itself has run at 8.
//!
//! So this file follows `embedded-shared/src/dual_core.rs`'s
//! `run_speculative_slot` instead, with the FreeRTOS plumbing (queues,
//! work-stealing, `esp_timer`) folded away. Everything it calls is the
//! same public `mfsk-core` entry point the board calls, in the same
//! order, with the same constants:
//!
//! ```text
//! spectrogram, rows m >= 174 zero       (stage1_inc emits at SPEC_EMIT_PAIR = 87)
//! coarse_sync_with_lag x2, split at mid  (coarse_sync_split_with_allsum, SYNC_LAG_S = 1.0)
//!   merge, sort by score, truncate 30    (PASS1_LIMIT)
//! partition by goertzel_window_end_sample(dt) <= prefix
//! early: refine_candidates_into x2 on the prefix, merge, sort, truncate 15
//!        process_candidates_into_with_cs_scratch_tuned, one candidate at a time
//! late:  only with budget left (max_cand - early refined), on the full slot
//! ```
//!
//! What it does not reproduce: the stage-3 deadline (the board's `cut`),
//! and the exact audio fill at emit time, taken here as the 168 000
//! samples `SpecBundle`'s own doc names as its floor
//! (`MFSK_MIRROR_PREFIX` overrides it; the board's real fill is 170 400,
//! and setting it changes no decode count measured so far).
//!
//! **Sweep the phase at 0.01 s or finer when the count matters.** A
//! weak station's decode is a 5 ms-scale comb in dt, so a 0.02 s grid
//! aliases it: over −0.40..+0.40 without fine sync, a 0.02 s sweep
//! peaked at 7 and a 0.005 s sweep found 8 at φ = +0.190 — the board's
//! own eight, message for message, at the phase its log recorded as
//! +0.201 (`m5stack-cores3-app/logs/hw_maxcand15_2026-09-05.log`).
//! The header of this file used to say the board "has run at 8" where
//! this mirror could not; it can, at a phase a coarse sweep steps
//! over.
//!
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_embedded_pipeline_mirror -- --ignored --nocapture
//! ```
#![cfg(not(feature = "fft-rustfft"))]

use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    DEFAULT_Q_THRESH, GatedSearch, PartialBlock2, RefinedCandidate, SpecCell, SpecFill,
    coarse_sync_gated_spec, coarse_sync_with_lag, coarse_sync_with_lag_and_partial,
    compute_spectrogram, goertzel_window_end_sample, process_candidates_into_with_cs_scratch_tuned,
    refine_candidates_into,
};
use mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER;
use mfsk_core::msg::wsjt77::unpack77;

#[path = "common/ft8_qso3.rs"]
#[allow(dead_code)]
mod ft8_qso3;
#[path = "common/golden.rs"]
#[allow(dead_code)]
mod golden;
#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// One FT8 slot at 12 kHz.
const SLOT: usize = 180_000;

// `m5stack-cores3-app/src/decode_pipeline.rs`'s `DecodeConfig`.
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
/// The board's own compile-time knobs, `MFSK_FT8_PASS1_LIMIT` /
/// `MFSK_FT8_MAX_CAND`, read at run time here so a cap sweep needs no
/// rebuild. Defaults are what ships.
fn pass1_limit() -> usize {
    std::env::var("MFSK_FT8_PASS1_LIMIT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(30)
}
fn max_cand() -> usize {
    std::env::var("MFSK_FT8_MAX_CAND")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(15)
}

/// `embedded-shared/src/dual_core.rs`'s `EMBEDDED_SYNC_LAG_S`, and
/// `stage1_inc`'s `SPEC_EMIT_PAIR`. Both ship as constants; here they
/// are the two axes of the emit-point question, so they are read
/// through accessors that a sweep can override.
///
/// The board ties them together: `SPEC_EMIT_PAIR = 87` is chosen from
/// `needed_m = 162 + jz`, where `jz` is `SYNC_LAG_S` in time rows
/// (`NSTEP` = 960 samples = 80 ms, so a 1.0 s lag is 13 rows).
/// Emitting earlier without narrowing the lag leaves the rows the
/// large-lag half of block 2 would have used zero, which is a
/// measurement, not a guess — that is what this axis is for.
const SHIP_EMIT_PAIR: usize = 87;
/// What `decode_pipeline` passes to `dual_core::set_sync_lag_s`.
///
/// **0.88 since 2026-09-20**, swept here — see
/// `mirror_partial_block2_policies`. It was 1.0, which `jz = round(lag
/// / 0.08)` turned into 13 rows, i.e. ±1.04 s; the sweep's `±1.04` arm
/// still reproduces that count exactly. 0.88 is 11 rows, the widest
/// lag whose block 2 still lands inside the rows pair 87 fills.
const SHIP_SYNC_LAG_S: f32 = 0.88;
/// `SpecBundle`'s audio prefix at the shipping emit point: "always >=
/// 168 000 samples".
const SHIP_PREFIX_SAMPLES: usize = 168_000;
/// One time row, in samples (`NSTEP` = `NSPS`/2 under `nstep-half`).
const NSTEP_SAMPLES: usize = 960;

thread_local! {
    static EMIT_PAIR_OVERRIDE: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
    static SYNC_LAG_OVERRIDE: std::cell::Cell<Option<f32>> = const { std::cell::Cell::new(None) };
    static ARM: std::cell::Cell<Arm> = const { std::cell::Cell::new(Arm::SHIP) };
}

/// Which of the S3 plan's pieces this run adds to the shipped pipeline, set in process by
/// `mirror_gate_and_osd` (the env vars `MFSK_MIRROR_GATE=1`, `MFSK_MIRROR_OSD=1` set the same for the other tests).
///
/// - `gate`: pass 1 is `coarse_sync_gated_spec` on the board's own spectrogram (hard-sync order, the first
///   `max_cand` of each half go to pass 2 without the re-rank by block-0 quality), not `coarse_sync_with_lag` by score.
/// - `fine`: the board's `fine_sync_12k` on the prefix between the coarse search and the partition (the same as
///   `MFSK_MIRROR_FINE_REFINE=lib`), and the board's dedupe by message.
/// - `osd`: stage 3 at `DecodeDepth { osd: true, ..EMBEDDED }`. Needs the `osd` feature.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
struct Arm {
    gate: bool,
    fine: bool,
    osd: bool,
}

impl Arm {
    const SHIP: Arm = Arm {
        gate: false,
        fine: false,
        osd: false,
    };
}

fn arm() -> Arm {
    let a = ARM.with(|a| a.get());
    Arm {
        gate: a.gate || std::env::var("MFSK_MIRROR_GATE").is_ok(),
        fine: a.fine || std::env::var("MFSK_MIRROR_FINE_REFINE").as_deref() == Ok("lib"),
        osd: a.osd || std::env::var("MFSK_MIRROR_OSD").is_ok(),
    }
}

/// Which pair stage1_inc emits the SpecBundle at. `MFSK_MIRROR_EMIT_PAIR`,
/// or a sweep's override.
fn emit_pair() -> usize {
    EMIT_PAIR_OVERRIDE
        .with(|c| c.get())
        .or_else(|| {
            std::env::var("MFSK_MIRROR_EMIT_PAIR")
                .ok()
                .and_then(|v| v.trim().parse().ok())
        })
        .unwrap_or(SHIP_EMIT_PAIR)
}

fn sync_lag_s() -> f32 {
    SYNC_LAG_OVERRIDE
        .with(|c| c.get())
        .or_else(|| {
            std::env::var("MFSK_MIRROR_SYNC_LAG")
                .ok()
                .and_then(|v| v.trim().parse().ok())
        })
        .unwrap_or(SHIP_SYNC_LAG_S)
}

/// Time rows the emitted spectrogram has: pair *k* fills rows 2k and
/// 2k+1, and the emit fires once pairs `0..emit_pair` are done.
fn spec_valid_rows() -> usize {
    2 * emit_pair()
}

/// The audio prefix that goes with that emit point — the shipping
/// prefix moved by the same number of rows.
///
/// `MFSK_MIRROR_PREFIX=<samples>` overrides it outright. The board's
/// own fill at emit is **not** the 168 000 this file has always used:
/// pair 86 fills rows 172-173, row 173 reads to sample
/// `173 * 960 + 3840` = 169 920, and stage1_inc is fed in 1 200-sample
/// chunks, so the fill when the emit fires is 170 400. 2 400 samples —
/// 200 ms of audio — that the board's early path has and this mirror
/// did not.
fn prefix_samples() -> usize {
    if let Some(n) = std::env::var("MFSK_MIRROR_PREFIX")
        .ok()
        .and_then(|v| v.trim().parse::<usize>().ok())
    {
        return n.min(SLOT);
    }
    let rows = spec_valid_rows() as i64 - 2 * SHIP_EMIT_PAIR as i64;
    (SHIP_PREFIX_SAMPLES as i64 + rows * NSTEP_SAMPLES as i64).clamp(0, SLOT as i64) as usize
}

/// How much earlier than the shipping emit point this is, in ms —
/// exactly the wall-clock the board's decoder would gain, since it
/// blocks on the SpecBundle and nothing else moves.
fn emit_gain_ms() -> i64 {
    (SHIP_PREFIX_SAMPLES as i64 - prefix_samples() as i64) / 12
}

struct SlotOut {
    n_pass1: usize,
    n_ready: usize,
    n_deferred: usize,
    results: Vec<DecodeResult>,
}

/// `MFSK_MIRROR_FINE_REFINE=1` inserts the host's per-candidate fine
/// refine between the coarse search and the prefix partition — the step
/// `decode_block_multipass`'s host body runs (`fine_refine_pass1`) and
/// its embedded body documents as a deliberate no-op ("deferred for
/// compute reasons", 0.6.3). The board has no way to build this yet: the
/// reference here downsamples through a 192 000-point FFT the embedded
/// planner does not carry. It is used only to measure what the board
/// would gain *if* it refined, before anyone builds the embedded
/// equivalent.
///
/// Refined from the audio the board's early path would have at that
/// moment — the prefix — since coarse sync runs on the `SpecBundle`
/// before slot end.
fn fine_refine_enabled() -> bool {
    std::env::var("MFSK_MIRROR_FINE_REFINE").is_ok()
}

/// `MFSK_MIRROR_FINE_REFINE=full` refines from the whole slot instead of
/// the prefix. Not what the board's early path could do; here to tell a
/// refine that is wrong from a refine that was starved of audio.
fn fine_refine_audio(slot: &[i16]) -> &[i16] {
    match std::env::var("MFSK_MIRROR_FINE_REFINE").as_deref() {
        Ok("full") => slot,
        _ => &slot[..prefix_samples()],
    }
}

fn fine_refine(prefix: &[i16], cands: Vec<SyncCandidate>) -> Vec<SyncCandidate> {
    use mfsk_core::engine::dsp::downsample::downsample_cached;
    use mfsk_core::ft8::downsample::{FT8_CFG, build_fft_cache};
    use mfsk_core::ft8::refine_fine::fine_refine_3stage;
    if cands.is_empty() {
        return cands;
    }
    let cache = build_fft_cache(prefix);
    cands
        .into_iter()
        .map(|c| {
            let cd0 = downsample_cached(&cache, c.freq_hz, &FT8_CFG);
            let r = fine_refine_3stage(&cd0, c.dt_sec);
            // `MFSK_MIRROR_FINE_REFINE=dt` / `=freq` apply one half of
            // the correction. The board's `refine_candidates_into` builds
            // symbol spectra by Goertzel on the raw audio, where the
            // host refine was written to feed `refine_candidates`'
            // downsampled path; a correction one path absorbs the other
            // may not.
            let mode = std::env::var("MFSK_MIRROR_FINE_REFINE").unwrap_or_default();
            SyncCandidate {
                freq_hz: if mode == "dt" {
                    c.freq_hz
                } else {
                    c.freq_hz + r.delf_hz
                },
                dt_sec: if mode == "freq" { c.dt_sec } else { r.dt_sec },
                score: c.score,
            }
        })
        .collect()
}

/// `MFSK_MIRROR_FINE_REFINE=stagea12k`: WSJT-X `ft8b.f90`'s Stage A
/// (±10 steps at 200 Hz = ±50 ms in 5 ms, at the coarse frequency),
/// computed the way the board could compute it — on the 12 kHz audio
/// directly, no baseband and no FFT.
///
/// `refine_fine::fine_sync_power_signed` sums, over the 21 Costas symbols
/// (blocks at symbol offsets 0 / 36 / 72, tones `[3,1,4,0,6,5,2]`),
/// the power of a 32-sample correlation of `cd0` against that symbol's
/// known tone. On the 12 kHz audio the same quantity is a 1920-sample
/// single-frequency correlation at `f0 + tone · 6.25 Hz`; a symbol whose
/// window leaves the audio contributes nothing, as it does in the
/// baseband version. Only the DT is taken — the frequency stays at the
/// coarse value, which the board's Goertzel refine handles and a
/// frequency correction was measured to break.
fn stage_a_12k(audio: &[i16], cands: Vec<SyncCandidate>) -> Vec<SyncCandidate> {
    stage_12k(audio, cands, false)
}

/// `sync8d` on the 12 kHz audio: summed power of the 21 Costas symbols'
/// single-tone correlations, the frame starting at cd0 index `i`
/// (200 Hz, 60 audio samples a step) with the carrier at `f0`. A symbol
/// whose window leaves the audio contributes nothing, as in `sync8d`.
fn sync8d_12k(audio: &[i16], f0: f32, i: i32) -> f32 {
    use num_complex::Complex;
    const NSPS: usize = 1_920;
    const FS: f32 = 12_000.0;
    const ICOS7: [usize; 7] = [3, 1, 4, 0, 6, 5, 2];
    let start0 = i as i64 * 60;
    let mut p = 0.0f32;
    for block in [0usize, 36, 72] {
        for (k, &tone) in ICOS7.iter().enumerate() {
            let st = start0 + ((block + k) * NSPS) as i64;
            if st < 0 || st as usize + NSPS > audio.len() {
                continue;
            }
            // Oscillator phase restarts at the window: |z|^2 ignores it.
            let w = core::f32::consts::TAU * (f0 + tone as f32 * 6.25) / FS;
            let mut z = Complex::new(0.0f32, 0.0);
            for (n, x) in audio[st as usize..st as usize + NSPS].iter().enumerate() {
                let ph = w * n as f32;
                z += Complex::new(ph.cos(), -ph.sin()) * (*x as f32);
            }
            p += z.norm_sqr();
        }
    }
    p
}

/// `ft8b.f90`'s refine on the 12 kHz audio. Stage A: DT over +-10 cd0
/// steps at the coarse frequency. With `abc`, also Stage B — frequency
/// over +-2.5 Hz in 0.5 Hz at Stage A's DT — and Stage C, DT over +-4
/// steps at the new frequency (`ft8b.f90:136-174`). Ties keep the first
/// maximum in every stage, as `sync.gt.smax` and `maxloc` do.
fn stage_12k(audio: &[i16], cands: Vec<SyncCandidate>, abc: bool) -> Vec<SyncCandidate> {
    cands
        .into_iter()
        .map(|c| {
            // cd0 index convention: i = round((dt + 0.5) * 200).
            let i0 = ((c.dt_sec + 0.5) * 200.0).round() as i32;
            let argmax = |it: &mut dyn Iterator<Item = (f32, f32)>| {
                let mut best = (0.0f32, f32::MIN);
                for (x, p) in it {
                    if p > best.1 {
                        best = (x, p);
                    }
                }
                best.0
            };
            let mut ibest = argmax(
                &mut (-10..=10).map(|d| ((i0 + d) as f32, sync8d_12k(audio, c.freq_hz, i0 + d))),
            ) as i32;
            let mut f1 = c.freq_hz;
            if abc {
                let delf = argmax(&mut (-5..=5).map(|k| {
                    let df = k as f32 * 0.5;
                    (df, sync8d_12k(audio, c.freq_hz + df, ibest))
                }));
                f1 = c.freq_hz + delf;
                ibest = argmax(
                    &mut (-4..=4).map(|d| ((ibest + d) as f32, sync8d_12k(audio, f1, ibest + d))),
                ) as i32;
            }
            let refined = SyncCandidate {
                freq_hz: f1,
                dt_sec: ibest as f32 / 200.0 - 0.5,
                ..c
            };
            COARSE_DT.with(|m| {
                m.borrow_mut()
                    .push((refined.freq_hz, refined.dt_sec, c.freq_hz, c.dt_sec))
            });
            refined
        })
        .collect()
}

/// `stage_12k(.., abc = true)` computed the way a board can afford it.
///
/// Each of the 21 Costas symbols is mixed down at its known tone **once**,
/// into 60-sample bins (cd0's 200 Hz grid, 32 bins a symbol), over the
/// span Stage A and Stage C can reach (+-14 cd0 steps). Every `sync8d`
/// evaluation after that is a sum of 32 bins: Stage A and C move the
/// start bin, Stage B multiplies the bins by `ft8b.f90`'s own 32-point
/// `ctwk` tweak. Cost is one mixing pass of (32 + 28 + 31) x 60 samples
/// per symbol instead of 1 920 per evaluation per symbol, 41 evaluations.
///
/// The approximation against `stage_12k`: the tweak phase is constant
/// across a 60-sample bin, off by at most 2 pi x 2.5 Hz x 30 / 12 000 =
/// 0.04 rad at the bin edges.
fn stage_abc_12k_binned(audio: &[i16], cands: Vec<SyncCandidate>) -> Vec<SyncCandidate> {
    use num_complex::Complex;
    const FS: f32 = 12_000.0;
    const ICOS7: [usize; 7] = [3, 1, 4, 0, 6, 5, 2];
    const REACH: i32 = 14; // Stage A +-10, then Stage C +-4
    const NB: usize = 32 + 2 * REACH as usize; // bins per symbol span
    cands
        .into_iter()
        .map(|c| {
            let i0 = ((c.dt_sec + 0.5) * 200.0).round() as i32;
            // bins[s][j]: symbol s, cd0 index i0 - REACH + (symbol offset) + j.
            let mut bins = vec![[Complex::new(0.0f32, 0.0); NB]; 21];
            for (s, (block, k)) in [0usize, 36, 72]
                .iter()
                .flat_map(|b| (0..7).map(move |k| (*b, k)))
                .enumerate()
            {
                let w = core::f32::consts::TAU * (c.freq_hz + ICOS7[k] as f32 * 6.25) / FS;
                let first = i0 - REACH + ((block + k) * 32) as i32;
                for j in 0..NB {
                    let bin = first + j as i32;
                    let st = bin as i64 * 60;
                    if st < 0 || st as usize + 60 > audio.len() {
                        continue; // left zero: contributes nothing
                    }
                    // Phase referenced to the span start, so f32 holds it.
                    let n_ref = (first as i64 * 60).max(0);
                    let mut z = Complex::new(0.0f32, 0.0);
                    for n in 0..60i64 {
                        let ph = w * (st + n - n_ref) as f32;
                        z += Complex::new(ph.cos(), -ph.sin()) * (audio[(st + n) as usize] as f32);
                    }
                    bins[s][j] = z;
                }
            }
            // sync8d at cd0 start i0 + d, carrier tweak delf.
            let sync = |d: i32, delf: f32| -> f32 {
                let dphi = core::f32::consts::TAU * delf / 200.0;
                let tw: Vec<Complex<f32>> = (0..32)
                    .map(|m| Complex::from_polar(1.0, -dphi * m as f32))
                    .collect();
                let mut p = 0.0f32;
                for sym in bins.iter() {
                    let j0 = (d + REACH) as usize;
                    let mut z = Complex::new(0.0f32, 0.0);
                    for m in 0..32 {
                        z += sym[j0 + m] * tw[m];
                    }
                    p += z.norm_sqr();
                }
                p
            };
            let argmax = |it: &mut dyn Iterator<Item = (f32, f32)>| {
                let mut best = (0.0f32, f32::MIN);
                for (x, p) in it {
                    if p > best.1 {
                        best = (x, p);
                    }
                }
                best.0
            };
            let da = argmax(&mut (-10..=10).map(|d| (d as f32, sync(d, 0.0)))) as i32;
            let delf = argmax(&mut (-5..=5).map(|k| (k as f32 * 0.5, sync(da, k as f32 * 0.5))));
            let dc = argmax(&mut (-4..=4).map(|d| ((da + d) as f32, sync(da + d, delf)))) as i32;
            let refined = SyncCandidate {
                freq_hz: c.freq_hz + delf,
                dt_sec: (i0 + dc) as f32 / 200.0 - 0.5,
                ..c
            };
            COARSE_DT.with(|m| {
                m.borrow_mut()
                    .push((refined.freq_hz, refined.dt_sec, c.freq_hz, c.dt_sec))
            });
            refined
        })
        .collect()
}

/// `MFSK_MIRROR_FINE_REFINE=stagea12k` or `stageabc12k`, applied to
/// pass 1 the way `run_slot` and the loss breakdown both need it.
/// `false` when neither is selected.
fn apply_12k_refine(prefix: &[i16], pass1: &mut Vec<SyncCandidate>) -> bool {
    let mode = if arm().fine {
        Ok("lib".to_string())
    } else {
        std::env::var("MFSK_MIRROR_FINE_REFINE")
    };
    match mode.as_deref() {
        Ok("stagea12k") => *pass1 = stage_12k(prefix, core::mem::take(pass1), false),
        Ok("stageabc12k") => *pass1 = stage_12k(prefix, core::mem::take(pass1), true),
        Ok("stageabc12kbin") => *pass1 = stage_abc_12k_binned(prefix, core::mem::take(pass1)),
        // What the board runs: the library kernel, then the same coarse
        // table the fallback reads.
        Ok("lib") => {
            let refined = mfsk_core::ft8::decode_block::fine_sync_12k(prefix, pass1);
            COARSE_DT.with(|m| {
                m.borrow_mut().extend(
                    refined
                        .iter()
                        .zip(pass1.iter())
                        .map(|(r, c)| (r.freq_hz, r.dt_sec, c.freq_hz, c.dt_sec)),
                )
            });
            *pass1 = refined;
        }
        _ => return false,
    }
    true
}

thread_local! {
    /// `(refined freq, refined dt, coarse freq, coarse dt)` for every
    /// candidate `stage_12k` saw in the current slot, so a failure at the
    /// refined position can be retried where the coarse search put it.
    static COARSE_DT: core::cell::RefCell<Vec<(f32, f32, f32, f32)>> =
        const { core::cell::RefCell::new(Vec::new()) };
}

/// `MFSK_MIRROR_COARSE_FALLBACK=1`: a candidate whose refined DT fails
/// to decode gets exactly one more attempt, at its coarse DT. Stage A
/// alone erased `N1PJT HB9CQK` on `qso3_busy` and `CQ LZ1JZ KN22` on
/// `qso2` at every phase — stations the shipped pipeline decodes at
/// 30-45 % of phases — so a refine that is right on average can still be
/// wrong for a particular station. With the fallback, no station the
/// shipped pipeline would decode at a given phase is lost to the refine,
/// by construction; the price is one attempt per failure.
fn coarse_fallback_enabled() -> bool {
    std::env::var("MFSK_MIRROR_COARSE_FALLBACK").is_ok()
}

/// What this run does with a lag whose Costas block 2 is not covered
/// by the rows `spec_valid_rows()` leaves real.
///
/// `MFSK_MIRROR_PARTIAL=score|gate`; `score` is the shipping
/// board's behaviour and the default, so every other test in this file
/// keeps measuring what ships.
fn partial_policy() -> PartialBlock2 {
    match std::env::var("MFSK_MIRROR_PARTIAL").as_deref() {
        Ok("gate") => PartialBlock2::Gate {
            valid_rows: spec_valid_rows(),
        },
        _ => PartialBlock2::Score,
    }
}

/// The `|dt|` past which block 2 is not fully covered, in seconds —
/// `stage1_inc::max_lag_s` re-derived here so the mirror does not
/// depend on the embedded crate.
fn partial_ceiling_s() -> f32 {
    // Block 2's last Costas symbol sits at row 162; a lag of `j` rows
    // moves it to `162 + j`, and rows `0 ..= spec_valid_rows() - 1`
    // are real, so it is covered while `162 + j <= valid - 1`.
    let j = spec_valid_rows() as i32 - 163;
    if j <= 0 {
        0.0
    } else {
        j as f32 * (NSTEP_SAMPLES as f32 / 12_000.0)
    }
}

/// `dual_core::coarse_sync_split_with_allsum`, sequential.
fn coarse_split(spec: &mfsk_core::ft8::decode_block::Spectrogram) -> Vec<SyncCandidate> {
    coarse_split_with(spec, partial_policy())
}

/// [`coarse_split`] with the policy named outright, for the comparison
/// that has to run all three in one process.
fn coarse_split_with(
    spec: &mfsk_core::ft8::decode_block::Spectrogram,
    partial: PartialBlock2,
) -> Vec<SyncCandidate> {
    if partial == PartialBlock2::Score {
        return coarse_split_score(spec);
    }
    let mid = 0.5 * (FREQ_MIN + FREQ_MAX);
    let mut all = coarse_sync_with_lag_and_partial(
        spec,
        FREQ_MIN,
        mid,
        SYNC_MIN,
        pass1_limit(),
        sync_lag_s(),
        partial,
    );
    all.extend(coarse_sync_with_lag_and_partial(
        spec,
        mid,
        FREQ_MAX,
        SYNC_MIN,
        pass1_limit(),
        sync_lag_s(),
        partial,
    ));
    all.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    all.truncate(pass1_limit());
    all
}

/// The gate on the board's own spectrogram (`coarse_sync_gated_spec`): one call over the whole band, the board's lag window,
/// the spectrogram's fill point given, block 2 scored on the rows there are (or gated with `MFSK_MIRROR_PARTIAL=gate`).
fn coarse_gated(
    spec: &mfsk_core::ft8::decode_block::Spectrogram,
    partial: PartialBlock2,
) -> Vec<SyncCandidate> {
    coarse_sync_gated_spec(
        spec,
        &GatedSearch::new(FREQ_MIN, FREQ_MAX, SYNC_MIN, pass1_limit())
            .lag_s(Some(sync_lag_s()))
            .fill(Some(
                SpecFill::new(spec_valid_rows())
                    .gate_block2(matches!(partial, PartialBlock2::Gate { .. })),
            )),
    )
    .into_iter()
    .map(|c| c.0)
    .collect()
}

fn coarse_split_score(spec: &mfsk_core::ft8::decode_block::Spectrogram) -> Vec<SyncCandidate> {
    let mid = 0.5 * (FREQ_MIN + FREQ_MAX);
    let mut all = coarse_sync_with_lag(spec, FREQ_MIN, mid, SYNC_MIN, pass1_limit(), sync_lag_s());
    all.extend(coarse_sync_with_lag(
        spec,
        mid,
        FREQ_MAX,
        SYNC_MIN,
        pass1_limit(),
        sync_lag_s(),
    ));
    all.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    all.truncate(pass1_limit());
    all
}

/// `dual_core::pass2_split`, sequential: halve, refine each half to
/// `max_cand`, merge, keep the top `max_cand` by pass-2 score.
fn pass2_split(audio: &[i16], pass1: Vec<SyncCandidate>, max_cand: usize) -> Vec<RefinedCandidate> {
    let mid = pass1.len() / 2;
    let mut head = pass1;
    let tail = head.split_off(mid);
    let mut local = refine_candidates_into(audio, head, max_cand);
    local.extend(refine_candidates_into(audio, tail, max_cand));
    local.sort_by(|a, b| b.2.cmp(&a.2));
    local.truncate(max_cand);
    local
}

/// `MFSK_MIRROR_DT_JITTER=5,10` (milliseconds): when a candidate fails
/// to decode, retry it at DT +-5 ms, then +-10 ms, keeping the first
/// success. Near the BP threshold a weak station's decodability in DT is
/// a comb with 5 ms holes (`mirror_dt_window`), so one exactly-right DT
/// can still land in a hole; the retry is per station and costs nothing
/// for candidates that decode first time.
fn dt_jitter_ms() -> Vec<f32> {
    std::env::var("MFSK_MIRROR_DT_JITTER")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_default()
}

/// One row per coarse retry actually run: (position in retry order,
/// failures that slot, full sync quality at the refined position, at the
/// coarse position, recovered a decode).
static FB_STATS: std::sync::Mutex<Vec<(usize, usize, u32, u32, bool)>> =
    std::sync::Mutex::new(Vec::new());

static RETRIES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// One row per candidate that failed its first decode with jitter on:
/// (coarse score, pass-2 score, retry recovered it). What a gate on
/// "worth retrying" could key on, and what it would have kept.
static FAILED: std::sync::Mutex<Vec<(f32, u32, bool)>> = std::sync::Mutex::new(Vec::new());

/// `MFSK_MIRROR_RETRY_TOP=N`: after every candidate has had its normal
/// single attempt, retry only the `N` failures with the highest coarse
/// score (0 or unset = all of them). A rank rather than a score
/// threshold, because coarse scores are baseline-normalised per band:
/// the recoverable stations on a quiet band sit at scores a busy band
/// discards (a fixed `score >= 3.0` keeps 97 % of `qso3_busy`'s
/// recoveries and 0 % of `qso1`'s).
fn retry_top() -> usize {
    std::env::var("MFSK_MIRROR_RETRY_TOP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0)
}

thread_local! {
    /// Stage-3 failures awaiting the slot's single coarse retry round.
    static PENDING_FB: core::cell::RefCell<Vec<(SyncCandidate, u32)>> =
        const { core::cell::RefCell::new(Vec::new()) };
}

/// `dual_core`'s coarse retry round: every failure the preceding stage-3
/// call queued, retried at its coarse position on `audio`, after that
/// path's first attempts.
fn coarse_retry_round(audio: &[i16]) -> Vec<DecodeResult> {
    let mut failed: Vec<(SyncCandidate, u32)> =
        PENDING_FB.with(|p| core::mem::take(&mut *p.borrow_mut()));
    let mut cs = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
    let decode = |c: RefinedCandidate, cs: &mut [[Cmplx<f32>; 8]; 79]| {
        process_candidates_into_with_cs_scratch_tuned(
            audio,
            vec![c],
            DecodeDepth::EMBEDDED,
            DEFAULT_Q_THRESH,
            DEFAULT_BP_MAX_ITER,
            cs,
        )
    };
    let mut out = Vec::new();
    // `MFSK_MIRROR_FB_TOP=N`: retry only the first N failures in the
    // order `MFSK_MIRROR_FB_ORDER` picks (`score`: coarse score,
    // `p2`: pass-2 block-0 q, `rank`: stage-3 order = pass-2 rank).
    // Unset = all, in stage-3 order, which is what ships.
    let fb_top: usize = std::env::var("MFSK_MIRROR_FB_TOP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(usize::MAX);
    match std::env::var("MFSK_MIRROR_FB_ORDER").as_deref() {
        Ok("score") => failed.sort_by(|a, b| {
            b.0.score
                .partial_cmp(&a.0.score)
                .unwrap_or(core::cmp::Ordering::Equal)
        }),
        Ok("p2") => failed.sort_by(|a, b| b.1.cmp(&a.1)),
        _ => {}
    }
    let n_failed = failed.len();
    let mut still: Vec<(SyncCandidate, u32)> = Vec::new();
    for (order, (base, p2_score)) in failed.into_iter().enumerate() {
        if order >= fb_top {
            still.push((base, p2_score));
            continue;
        }
        let coarse = COARSE_DT.with(|m| {
            m.borrow()
                .iter()
                .find(|(f, d, _, _)| *f == base.freq_hz && *d == base.dt_sec)
                .map(|e| (e.2, e.3))
        });
        let Some((coarse_f, coarse_dt)) =
            coarse.filter(|(f, d)| *f != base.freq_hz || *d != base.dt_sec)
        else {
            still.push((base, p2_score));
            continue;
        };
        RETRIES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let p2 = refine_candidates_into(
            audio,
            vec![SyncCandidate {
                freq_hz: coarse_f,
                dt_sec: coarse_dt,
                ..base.clone()
            }],
            1,
        );
        let got = match p2.into_iter().next() {
            Some(c) => decode(c, &mut cs),
            None => Vec::new(),
        };
        let full_q = |f: f32, dt: f32| {
            use mfsk_core::ft8::decode_block::{SymMask, fill_symbol_spectra_goertzel};
            let mut cs = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
            fill_symbol_spectra_goertzel(&mut cs, audio, f, dt, SymMask::SyncOnly);
            mfsk_core::ft8::llr::sync_quality(&cs)
        };
        FB_STATS.lock().unwrap().push((
            order,
            n_failed,
            full_q(base.freq_hz, base.dt_sec),
            full_q(coarse_f, coarse_dt),
            !got.is_empty(),
        ));
        if got.is_empty() {
            still.push((base, p2_score));
        } else {
            out.extend(got);
        }
    }
    let _ = still;
    out
}

/// `dual_core::stage3_split` without the deadline: every candidate on
/// its own, as `drain_stage3_queue` does, then the optional DT-jitter
/// retry pass over the failures.
fn stage3(audio: &[i16], cands: Vec<RefinedCandidate>) -> Vec<DecodeResult> {
    let mut cs = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
    let decode = |c: RefinedCandidate, cs: &mut [[Cmplx<f32>; 8]; 79]| {
        process_candidates_into_with_cs_scratch_tuned(
            audio,
            vec![c],
            if arm().osd {
                DecodeDepth {
                    osd: true,
                    ..DecodeDepth::EMBEDDED
                }
            } else {
                DecodeDepth::EMBEDDED
            },
            DEFAULT_Q_THRESH,
            DEFAULT_BP_MAX_ITER,
            cs,
        )
    };

    let jitter = dt_jitter_ms();
    let mut out = Vec::new();
    let mut failed: Vec<(SyncCandidate, u32)> = Vec::new();
    for c in cands {
        let (base, p2_score) = (c.0.clone(), c.2);
        let got = decode(c, &mut cs);
        if got.is_empty() {
            failed.push((base, p2_score));
        } else {
            out.extend(got);
        }
    }
    if coarse_fallback_enabled() {
        // Queued for `coarse_retry_round`, which the caller runs after
        // this path's first attempts. On the board the early path's
        // retries run in the tail window and yield to the slot's arrival,
        // so they can never take a deferred candidate's first attempt's
        // time; without a clock that order and this one decode the same.
        PENDING_FB.with(|p| p.borrow_mut().extend(failed));
        return out;
    }
    if jitter.is_empty() {
        return out;
    }

    failed.sort_by(|a, b| {
        b.0.score
            .partial_cmp(&a.0.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let top = retry_top();
    let n_retry = if top == 0 {
        failed.len()
    } else {
        top.min(failed.len())
    };
    for (i, (base, p2_score)) in failed.into_iter().enumerate() {
        let mut recovered = false;
        if i < n_retry {
            'retry: for ms in &jitter {
                for sign in [1.0f32, -1.0] {
                    let shifted = SyncCandidate {
                        dt_sec: base.dt_sec + sign * ms / 1000.0,
                        ..base.clone()
                    };
                    RETRIES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    let p2 = refine_candidates_into(audio, vec![shifted], 1);
                    let Some(c) = p2.into_iter().next() else {
                        continue;
                    };
                    let got = decode(c, &mut cs);
                    if !got.is_empty() {
                        out.extend(got);
                        recovered = true;
                        break 'retry;
                    }
                }
            }
        }
        FAILED
            .lock()
            .unwrap()
            .push((base.score, p2_score, recovered));
    }
    out
}

/// `dual_core::run_speculative_slot`, one slot.
fn run_slot(slot: &[i16]) -> SlotOut {
    run_slot_with(slot, partial_policy())
}

fn run_slot_with(slot: &[i16], partial: PartialBlock2) -> SlotOut {
    assert_eq!(slot.len(), SLOT);
    COARSE_DT.with(|m| m.borrow_mut().clear());
    PENDING_FB.with(|p| p.borrow_mut().clear());
    let mut spec = compute_spectrogram(slot, FREQ_MAX);
    for t in spec_valid_rows()..spec.n_time {
        let row = t * spec.n_freq;
        for cell in &mut spec.data[row..row + spec.n_freq] {
            *cell = SpecCell::default();
        }
    }

    let mut pass1 = if arm().gate {
        coarse_gated(&spec, partial)
    } else {
        coarse_split_with(&spec, partial)
    };
    if apply_12k_refine(&slot[..prefix_samples()], &mut pass1) {
    } else if fine_refine_enabled() {
        pass1 = fine_refine(fine_refine_audio(slot), pass1);
    }
    let n_pass1 = pass1.len();
    // **How the stage-3 budget is split between the two halves.**
    //
    // Shipping (`half`): the early half takes up to `max_cand` and the
    // late half gets what is left, which is nothing whenever the early
    // half fills it — `dual_core`'s own comment says so, and
    // `mirror_emit_earlier` puts a number on it: emitting 320 ms
    // earlier costs 0.83 decodes this way and 0.14 when the budget is
    // shared.
    //
    // `MFSK_MIRROR_CAND_ALLOC=value`: take the top `max_cand` of pass 1
    // by coarse score — the ranking both halves already share, and the
    // only one available before the slot arrives — and give each half
    // as many slots as it holds of that set. Same total, so the same
    // stage-3 cost; only *which* candidates get it changes.
    let by_value = std::env::var("MFSK_MIRROR_CAND_ALLOC").as_deref() == Ok("value");
    let is_ready = |c: &SyncCandidate| goertzel_window_end_sample(c.dt_sec) <= prefix_samples();
    let (early_budget, late_budget) = if by_value {
        let mut early = 0usize;
        for c in pass1.iter().take(max_cand()) {
            if is_ready(c) {
                early += 1;
            }
        }
        (early, max_cand() - early)
    } else {
        (max_cand(), 0)
    };
    let (ready, deferred): (Vec<_>, Vec<_>) = pass1.into_iter().partition(&is_ready);
    let (n_ready, n_deferred) = (ready.len(), deferred.len());

    let prefix = &slot[..prefix_samples()];
    let mut n_early_refined = 0;
    let (mut ready, mut deferred) = (ready, deferred);
    // the gate's order is the ranking: take the first of it, not the best by block-0 quality
    if arm().gate {
        ready.truncate(early_budget);
    }
    let mut results = if ready.is_empty() || early_budget == 0 {
        Vec::new()
    } else {
        let p2 = pass2_split(prefix, ready, early_budget);
        n_early_refined = p2.len();
        stage3(prefix, p2)
    };

    results.extend(coarse_retry_round(prefix));

    let max_cand_late = if by_value {
        late_budget
    } else {
        max_cand().saturating_sub(n_early_refined)
    };
    if arm().gate {
        deferred.truncate(max_cand_late);
    }
    if max_cand_late > 0 && !deferred.is_empty() {
        let p2 = pass2_split(slot, deferred, max_cand_late);
        results.extend(stage3(slot, p2));
    }
    results.extend(coarse_retry_round(slot));
    // The board dedups by message once fine sync can pull two coarse
    // bins onto one carrier; mirror it in the mode that models the board.
    if arm().fine || arm().gate || std::env::var("MFSK_MIRROR_FINE_REFINE").as_deref() == Ok("lib")
    {
        let mut seen: Vec<Vec<u8>> = Vec::new();
        results.retain(|r| {
            let m = r.message77().to_vec();
            if seen.contains(&m) {
                false
            } else {
                seen.push(m);
                true
            }
        });
    }

    SlotOut {
        n_pass1,
        n_ready,
        n_deferred,
        results,
    }
}

fn at_phase(slot: &[i16], phi_s: f32) -> SlotOut {
    at_phase_with(slot, phi_s, partial_policy())
}

fn at_phase_with(slot: &[i16], phi_s: f32, partial: PartialBlock2) -> SlotOut {
    let k = ((phi_s * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
    let mut rotated = slot.to_vec();
    rotated.rotate_left(k);
    run_slot_with(&rotated, partial)
}

/// `MFSK_MIRROR_WAV=<file under embedded-poc/assets/>` swaps the
/// recording, so a change tuned on `qso3_busy` can be checked on the
/// other on-air slots before it is believed.
fn load_slot() -> Vec<i16> {
    let path = match std::env::var("MFSK_MIRROR_WAV") {
        Ok(f) => format!("{}/../embedded-poc/assets/{f}", env!("CARGO_MANIFEST_DIR")),
        Err(_) => QSO3_PATH.to_string(),
    };
    let mut slot = load_wav_i16(std::path::Path::new(&path));
    // The SIM harness loops a whole number of slots; so does this.
    slot.truncate(SLOT);
    slot
}

/// The acceptance check: at the recording's own phase the board's
/// `wav_sim` log reads `p1=30 ready=27 defer=3 dec=7` on every slot of
/// every capture that ran it (2026-08-28 through 2026-09-16).
#[test]
#[ignore = "diagnostic — compares the mirror with the board at the recording's own phase"]
fn mirror_at_the_recordings_own_phase() {
    let slot = load_slot();
    let out = at_phase(&slot, 0.0);
    let mut msgs: Vec<String> = out
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect();
    msgs.sort();
    println!(
        "mirror  p1={} ready={} defer={} dec={}",
        out.n_pass1,
        out.n_ready,
        out.n_deferred,
        out.results.len()
    );
    println!("board   p1=30 ready=27 defer=3 dec=7   (src=wav, every log)");
    for m in msgs {
        println!("  {m}");
    }
}

/// Phase response on the board's own pipeline. `MFSK_GRID_PHASE=
/// start,end,step` (seconds) overrides the default comb.
#[test]
#[ignore = "diagnostic — the board pipeline's decode count against grid phase"]
fn mirror_phase_response() {
    let slot = load_slot();
    let (lo, hi, dphi) = match std::env::var("MFSK_GRID_PHASE") {
        Ok(v) => {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            assert_eq!(f.len(), 3, "MFSK_GRID_PHASE wants start,end,step");
            (f[0], f[1], f[2])
        }
        Err(_) => (-1.5, 1.5, 0.05),
    };

    // `MFSK_MIRROR_MSGS=1` appends each phase's sorted message set, so a
    // decode set seen on the board can be located on this axis by its
    // contents rather than by its count — two phases with the same
    // count can decode different stations.
    let with_msgs = std::env::var("MFSK_MIRROR_MSGS").is_ok();
    println!("  phi(s)  p1  ready defer  dec   min dt   med dt   max dt");
    let mut hist = [0usize; 16];
    let mut best = (0usize, 0.0f32);
    let mut step = 0i32;
    while lo + step as f32 * dphi <= hi + 1e-6 {
        let phi = lo + step as f32 * dphi;
        let out = at_phase(&slot, phi);
        let n = out.results.len();
        hist[n.min(15)] += 1;
        if n > best.0 {
            best = (n, phi);
        }
        let mut dts: Vec<f32> = out.results.iter().map(|r| r.dt_sec).collect();
        dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let msgs = if with_msgs {
            let mut m: Vec<String> = out
                .results
                .iter()
                .filter_map(|r| unpack77(r.message77()))
                .collect();
            m.sort();
            format!("   | {}", m.join(" | "))
        } else {
            String::new()
        };
        if dts.is_empty() {
            println!(
                "  {phi:+6.2}  {:2}   {:2}   {:2}    0        -        -        -",
                out.n_pass1, out.n_ready, out.n_deferred
            );
        } else {
            println!(
                "  {phi:+6.3}  {:2}   {:2}   {:2}   {n:2}   {:+6.3}   {:+6.3}   {:+6.3}{msgs}",
                out.n_pass1,
                out.n_ready,
                out.n_deferred,
                dts[0],
                dts[dts.len() / 2],
                dts[dts.len() - 1]
            );
        }
        step += 1;
    }
    let hist_s: Vec<String> = hist
        .iter()
        .enumerate()
        .filter(|(_, c)| **c > 0)
        .map(|(d, c)| format!("{d}:{c}"))
        .collect();
    println!("  dec histogram {}", hist_s.join(" "));
    println!("  best dec={} at phi={:+.2} s", best.0, best.1);
    if let Ok(v) = std::env::var("MFSK_MIRROR_FB_DUMP") {
        use std::io::Write;
        let mut f = std::fs::File::create(v).unwrap();
        for (o, n, p2, sc, ok) in FB_STATS.lock().unwrap().iter() {
            writeln!(f, "{o} {n} {p2} {sc} {}", *ok as u8).unwrap();
        }
    }
    if let Ok(v) = std::env::var("MFSK_MIRROR_FAILED_DUMP") {
        use std::io::Write;
        let mut f = std::fs::File::create(v).unwrap();
        for (cs, p2, ok) in FAILED.lock().unwrap().iter() {
            writeln!(f, "{cs} {p2} {}", *ok as u8).unwrap();
        }
    }
    println!(
        "  dt-jitter retries: {} over {} slots",
        RETRIES.load(std::sync::atomic::Ordering::Relaxed),
        step
    );
}

/// Where each station is lost, stage by stage, across the plateau.
///
/// At its best 5 ms phases this pipeline decodes 8 of `qso3_busy`; at
/// most phases 6 or 7. Chasing the lucky phase is not a fix — the grid
/// is set once and the traffic after it is different — so the question
/// is which *stage* drops the 7th and 8th station at ordinary phases.
/// A stage that drops them for a reason unrelated to this recording's
/// exact timing is a real receiver fix; one that only drops them at
/// unlucky sub-step alignments is not.
///
/// Stations are identified by carrier frequency (a rotation moves DT,
/// not frequency), taken from the board's own decode lines: its 8-decode
/// set on 2026-09-05 plus `XE2X HA2NP`, which this pipeline decodes at
/// the recording's own phase and the board's 8 did not.
#[test]
#[ignore = "diagnostic — per-station loss stage across the plateau"]
fn mirror_where_stations_are_lost() {
    // `tests/common/ft8_qso3.rs`'s 20 known-real signals, less the one
    // with no frequency reference (`K1JT HA5WA 73`).
    const TARGETS: [(&str, f32); 19] = [
        ("CQ F5RXL IN94", 1197.0),
        ("N1JFU EA6EE R-07", 641.0),
        ("A92EE F5PSR -14", 723.0),
        ("W0RSJ EA3BMU RR73", 400.0),
        ("K1JT HA0DU KN07", 590.0),
        ("N1PJT HB9CQK -10", 466.0),
        ("K1BZM DK8NE -10", 244.0),
        ("KD2UGC F6GCP R-23", 472.0),
        ("K1JT EA3AGB -15", 1648.0),
        ("WM3PEN EA6VQ -09", 2157.0),
        ("W1FC F5BZB -08", 2571.0),
        ("XE2X HA2NP RR73", 2854.0),
        ("N1API F2VX 73", 1513.0),
        ("W1DIG SV9CVY -14", 2734.0),
        ("N1API HA6FQ -23", 2239.0),
        ("CQ EA2BFM IN83", 2279.0),
        ("K1BZM EA3CJ JN01", 2522.0),
        ("WA2FZW DL5AXX RR73", 2546.0),
        ("K1BZM EA3GP -09", 2696.0),
    ];
    /// Half a tone: a candidate this close to the carrier is that
    /// station's.
    const DF: f32 = 3.2;
    const STAGES: [&str; 5] = [
        "not in pass1 top",
        "deferred, dropped",
        "cut at refine top",
        "refined, no decode",
        "decoded",
    ];

    let slot = load_slot();
    let (lo, hi, dphi) = match std::env::var("MFSK_GRID_PHASE") {
        Ok(v) => {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            assert_eq!(f.len(), 3, "MFSK_GRID_PHASE wants start,end,step");
            (f[0], f[1], f[2])
        }
        Err(_) => (-0.6, 0.4, 0.005),
    };
    let near = |f: f32, t: f32| (f - t).abs() <= DF;

    let mut counts = vec![[0usize; 5]; TARGETS.len()];
    let mut n_phases = 0usize;
    let mut step = 0i32;
    while lo + step as f32 * dphi <= hi + 1e-6 {
        let phi = lo + step as f32 * dphi;
        step += 1;
        n_phases += 1;

        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let mut rot = slot.clone();
        rot.rotate_left(k);

        // The same stages as `run_slot`, keeping the intermediate sets.
        let mut spec = compute_spectrogram(&rot, FREQ_MAX);
        for t in spec_valid_rows()..spec.n_time {
            let row = t * spec.n_freq;
            for cell in &mut spec.data[row..row + spec.n_freq] {
                *cell = SpecCell::default();
            }
        }
        COARSE_DT.with(|m| m.borrow_mut().clear());
        let mut pass1 = coarse_split(&spec);
        if apply_12k_refine(&rot[..prefix_samples()], &mut pass1) {
        } else if fine_refine_enabled() {
            pass1 = fine_refine(fine_refine_audio(&rot), pass1);
        }
        let (ready, deferred): (Vec<_>, Vec<_>) = pass1
            .clone()
            .into_iter()
            .partition(|c| goertzel_window_end_sample(c.dt_sec) <= prefix_samples());
        let prefix = &rot[..prefix_samples()];
        let p2_early = if ready.is_empty() {
            Vec::new()
        } else {
            pass2_split(prefix, ready.clone(), max_cand())
        };
        let late_budget = max_cand().saturating_sub(p2_early.len());
        let p2_late = if late_budget > 0 && !deferred.is_empty() {
            pass2_split(&rot, deferred.clone(), late_budget)
        } else {
            Vec::new()
        };
        let mut results = stage3(prefix, p2_early.clone());
        results.extend(coarse_retry_round(prefix));
        results.extend(stage3(&rot, p2_late.clone()));
        results.extend(coarse_retry_round(&rot));

        let decoded: Vec<String> = results
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .collect();
        for (i, &(msg, f)) in TARGETS.iter().enumerate() {
            let stage = if decoded.iter().any(|m| m == msg) {
                4
            } else if p2_early
                .iter()
                .chain(p2_late.iter())
                .any(|c| near(c.0.freq_hz, f))
            {
                3
            } else if ready.iter().any(|c| near(c.freq_hz, f))
                || (late_budget > 0 && deferred.iter().any(|c| near(c.freq_hz, f)))
            {
                2
            } else if deferred.iter().any(|c| near(c.freq_hz, f)) {
                1
            } else {
                0
            };
            counts[i][stage] += 1;
        }
    }

    println!(
        "  {n_phases} phases, {lo:+.3}..{hi:+.3} s at {:.0} ms, pass1={} max_cand={}",
        dphi * 1000.0,
        pass1_limit(),
        max_cand()
    );
    print!("  {:<18}", "station");
    for s in STAGES {
        print!("  {s:>18}");
    }
    println!();
    for (i, &(name, f)) in TARGETS.iter().enumerate() {
        if counts[i][0] == n_phases {
            continue; // never reached pass 1 at any phase here
        }
        print!("  {name:<18}");
        for c in counts[i] {
            print!("  {:>17.0}%", 100.0 * c as f32 / n_phases as f32);
        }
        println!("   ({f:.0} Hz)");
    }
}

/// What the fine refine does to each pass-1 candidate near a station
/// that it stops decoding, at the recording's own phase.
///
/// With the refine on, `N1PJT HB9CQK`, `W1DIG SV9CVY` and `XE2X HA2NP`
/// decode at no phase in the plateau, whatever the caps — raising
/// `MAX_CAND` to 25 changes nothing, and refining from the whole slot
/// instead of the prefix changes nothing. So the refine is moving those
/// candidates somewhere they cannot decode. This prints where.
#[test]
#[ignore = "diagnostic — coarse vs refined position for candidates the refine loses"]
fn mirror_refine_moves() {
    use mfsk_core::engine::dsp::downsample::downsample_cached;
    use mfsk_core::ft8::downsample::{FT8_CFG, build_fft_cache};
    use mfsk_core::ft8::refine_fine::fine_refine_3stage;

    const WATCH: [(&str, f32); 5] = [
        ("N1PJT HB9CQK", 466.0),
        ("W1DIG SV9CVY", 2734.0),
        ("XE2X HA2NP", 2853.0),
        ("K1JT EA3AGB", 0.0), // frequency unknown from the board log; matched by message
        ("W0RSJ EA3BMU", 400.0),
    ];

    let slot = load_slot();
    let mut spec = compute_spectrogram(&slot, FREQ_MAX);
    for t in spec_valid_rows()..spec.n_time {
        let row = t * spec.n_freq;
        for cell in &mut spec.data[row..row + spec.n_freq] {
            *cell = SpecCell::default();
        }
    }
    let pass1 = coarse_split(&spec);
    let cache = build_fft_cache(&slot);

    println!(
        "  rank  coarse f / dt / score       refined f / dt        coarse-decode  refined-decode"
    );
    let decode_one = |c: SyncCandidate| -> Option<String> {
        let p2 = refine_candidates_into(&slot, vec![c], 1);
        stage3(&slot, p2)
            .first()
            .and_then(|r| unpack77(r.message77()))
    };
    for (rank, c) in pass1.iter().enumerate() {
        let watched = WATCH
            .iter()
            .any(|(_, f)| *f > 0.0 && (c.freq_hz - f).abs() <= 6.5);
        let cd0 = downsample_cached(&cache, c.freq_hz, &FT8_CFG);
        let r = fine_refine_3stage(&cd0, c.dt_sec);
        let refined = SyncCandidate {
            freq_hz: c.freq_hz + r.delf_hz,
            dt_sec: r.dt_sec,
            score: c.score,
        };
        let before = decode_one(c.clone());
        let after = decode_one(refined.clone());
        let interesting = watched
            || before
                .as_deref()
                .is_some_and(|m| WATCH.iter().any(|(w, _)| m.starts_with(w)))
            || after
                .as_deref()
                .is_some_and(|m| WATCH.iter().any(|(w, _)| m.starts_with(w)));
        if interesting {
            println!(
                "  {rank:>3}   {:7.1} {:+6.3} {:6.2}      {:7.1} {:+6.3}      {:<16} {:<16}",
                c.freq_hz,
                c.dt_sec,
                c.score,
                refined.freq_hz,
                refined.dt_sec,
                before.unwrap_or_else(|| "-".into()),
                after.unwrap_or_else(|| "-".into()),
            );
        }
    }
}

/// Where each candidate can decode in DT, and where the refine puts it.
///
/// The DT half of the fine refine lifts `qso3_busy`'s plateau from 6.35
/// to 8.27 decodes, and also erases some stations outright at every
/// phase — `N1PJT HB9CQK` and `XE2X HA2NP` there, `CQ LZ1JZ KN22` on
/// `qso2`. At `phi=0` a +14 ms refine move was enough to lose XE2X.
///
/// For every pass-1 candidate this holds frequency at the coarse value,
/// sweeps DT across +-120 ms in 5 ms steps through the board's own
/// refine-and-decode, and prints the window it decodes in beside the
/// coarse and refined DT. Two readings are possible and they call for
/// different fixes: the lost stations alone land outside their window
/// (a per-station estimate problem), or every refined DT sits a similar
/// distance from its window's centre (a convention mismatch between
/// `fine_refine_3stage`'s 200 Hz baseband and `refine_candidates_into`'s
/// Goertzel on the raw audio).
///
/// `MFSK_MIRROR_PHI` picks the phase (default 0).
#[test]
#[ignore = "diagnostic — per-candidate decodable DT window vs coarse and refined DT"]
fn mirror_dt_window() {
    use mfsk_core::engine::dsp::downsample::downsample_cached;
    use mfsk_core::ft8::downsample::{FT8_CFG, build_fft_cache};
    use mfsk_core::ft8::refine_fine::fine_refine_3stage;

    const SPAN_S: f32 = 0.120;
    const STEP_S: f32 = 0.005;

    let phi: f32 = std::env::var("MFSK_MIRROR_PHI")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    let base = load_slot();
    let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
    let mut slot = base.clone();
    slot.rotate_left(k);

    let mut spec = compute_spectrogram(&slot, FREQ_MAX);
    for t in spec_valid_rows()..spec.n_time {
        let row = t * spec.n_freq;
        for cell in &mut spec.data[row..row + spec.n_freq] {
            *cell = SpecCell::default();
        }
    }
    let pass1 = coarse_split(&spec);
    let cache = build_fft_cache(&slot);

    let decode_at = |freq: f32, dt: f32| -> Option<String> {
        let c = SyncCandidate {
            freq_hz: freq,
            dt_sec: dt,
            score: 1.0,
        };
        let p2 = refine_candidates_into(&slot, vec![c], 1);
        stage3(&slot, p2)
            .first()
            .and_then(|r| unpack77(r.message77()))
    };

    let n = (2.0 * SPAN_S / STEP_S).round() as i32;
    println!(
        "  phi={phi:+.3}  window bar spans coarse dt -{:.0}..+{:.0} ms, '#' = decodes, 'C' coarse, 'R' refined",
        SPAN_S * 1000.0,
        SPAN_S * 1000.0
    );
    let mut offsets: Vec<(String, f32)> = Vec::new();
    for c in pass1.iter() {
        let r = fine_refine_3stage(&downsample_cached(&cache, c.freq_hz, &FT8_CFG), c.dt_sec);
        let mut bar = String::new();
        let mut hits: Vec<f32> = Vec::new();
        let mut msg: Option<String> = None;
        for i in 0..=n {
            let dt = c.dt_sec - SPAN_S + i as f32 * STEP_S;
            match decode_at(c.freq_hz, dt) {
                Some(m) => {
                    bar.push('#');
                    hits.push(dt);
                    msg.get_or_insert(m);
                }
                None => bar.push('.'),
            }
        }
        let Some(msg) = msg else { continue };
        // Mark coarse (centre of the bar) and refined positions.
        let mut chars: Vec<char> = bar.chars().collect();
        let idx = |dt: f32| ((dt - (c.dt_sec - SPAN_S)) / STEP_S).round() as i32;
        let ci = idx(c.dt_sec);
        let ri = idx(r.dt_sec);
        if (0..=n).contains(&ci) {
            chars[ci as usize] = 'C';
        }
        if (0..=n).contains(&ri) {
            chars[ri as usize] = if ri == ci { 'B' } else { 'R' };
        }
        let lo = hits.first().copied().unwrap_or(f32::NAN);
        let hi = hits.last().copied().unwrap_or(f32::NAN);
        let centre = 0.5 * (lo + hi);
        offsets.push((msg.clone(), r.dt_sec - centre));
        println!(
            "  {:7.1} Hz  {}  win {:+.3}..{:+.3}  coarse {:+.3}  refined {:+.3} (centre{:+.0} ms)  {msg}",
            c.freq_hz,
            chars.into_iter().collect::<String>(),
            lo,
            hi,
            c.dt_sec,
            r.dt_sec,
            (r.dt_sec - centre) * 1000.0,
        );
    }
    if !offsets.is_empty() {
        let mean = offsets.iter().map(|o| o.1).sum::<f32>() / offsets.len() as f32;
        println!(
            "  refined dt minus window centre, mean over {} candidates: {:+.1} ms",
            offsets.len(),
            mean * 1000.0
        );
    }
}

/// Acquisition from anywhere in the period, scored on what the steady
/// pipeline decodes where it lands.
///
/// For each starting misalignment across the whole 15 s period, take the
/// 25 s capture the board takes, run the acquisition, and decode one slot
/// at the landed phase through `run_slot` — which carries whatever steady
/// pipeline the environment configures, so the same test scores the
/// shipped pipeline and an improved one.
///
/// Two acquisitions:
///
/// - **current** — `acquire_slot_phases` (tiled coarse search, circular
///   clusters), best-of-N trial decode, trial-median correction. What
///   `m5stack-cores3-app` runs today.
/// - **expand** — the same estimate, then trial decodes at +-`K` steps of
///   `STEP` around it, counting only decodes at |dt| <= 1.0 (what the
///   per-slot search can reach). The grid goes to the midpoint of the run
///   of steps that reach at least half the best count: the *edges of the
///   decodable range*, not its peak count, since the edges are where
///   frames and DTs fit the window — common to every station — and the
///   peak is which marginal station squeezed in.
///
/// The capture's own bounds are honoured the way the app honours them: a
/// trial offset that would read past the 25 s is skipped, never read from
/// the loop buffer beyond it.
///
/// `MFSK_ACQ_EXPAND=step_s,k` (default 0.2,5).
#[test]
#[ignore = "diagnostic — acquisition methods scored on the steady pipeline's decodes"]
fn mirror_acquisition_methods() {
    use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phases};
    use mfsk_core::ft8::decode_block::decode_block_tuned;

    const ACQ_MAX_CAND: usize = 200;
    const ACQ_MAX_TRIALS: usize = 5;
    let (step_s, k_max) = match std::env::var("MFSK_ACQ_EXPAND") {
        Ok(v) => {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            assert_eq!(f.len(), 2, "MFSK_ACQ_EXPAND wants step_s,k");
            (f[0], f[1] as i32)
        }
        Err(_) => (0.2f32, 5i32),
    };

    let slot = load_slot();
    let mut loopbuf = Vec::with_capacity(SLOT * 3);
    for _ in 0..3 {
        loopbuf.extend_from_slice(&slot);
    }
    let wrap = |mut dt: f32| {
        while dt > 7.5 {
            dt -= 15.0;
        }
        while dt <= -7.5 {
            dt += 15.0;
        }
        dt
    };
    // The app's trial decoder, on the capture only.
    let trial = |audio: &[i16], offset_s: f32| -> Option<Vec<DecodeResult>> {
        let off = ((offset_s * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        if audio.len() < off + SLOT {
            return None;
        }
        Some(decode_block_tuned(
            &audio[off..off + SLOT],
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            DecodeDepth::EMBEDDED,
            MAX_CAND_TRIAL,
            DEFAULT_BP_MAX_ITER,
        ))
    };
    let unique = |rs: &[DecodeResult]| -> usize {
        let mut m: Vec<String> = rs.iter().filter_map(|r| unpack77(r.message77())).collect();
        m.sort();
        m.dedup();
        m.len()
    };
    let in1s = |rs: &[DecodeResult]| rs.iter().filter(|r| r.dt_sec.abs() <= 1.0).count();

    println!(
        "  start   current: land  dec     expand: land  dec   ({} step {:.2} s, k={k_max})",
        if fine_refine_enabled() {
            "refine"
        } else {
            "no refine"
        },
        step_s
    );
    let (mut s_cur, mut s_exp, mut n) = (0usize, 0usize, 0usize);
    let (mut e8_cur, mut e8_exp) = (0usize, 0usize);
    for step in 0..30 {
        let start = step as f32 * 0.5;
        let cap_k = (start * 12_000.0) as usize;
        let audio = &loopbuf[cap_k..cap_k + REQUIRED_SAMPLES];
        let phases = acquire_slot_phases(
            audio,
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            ACQ_MAX_CAND,
            ACQ_MAX_TRIALS,
        );

        // current: best-of-N trial, median correction.
        let mut best: Option<(usize, f32)> = None;
        for &(centre, _) in phases.iter() {
            let Some(got) = trial(audio, centre) else {
                continue;
            };
            if got.is_empty() {
                continue;
            }
            let mut dts: Vec<f32> = got.iter().map(|r| r.dt_sec).collect();
            dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            let dt = wrap(centre + dts[dts.len() / 2]);
            if best.is_none_or(|(bn, _)| got.len() > bn) {
                best = Some((got.len(), dt));
            }
        }
        let Some((_, dt_cur)) = best else {
            println!("  {start:5.1}   (nothing decoded)");
            continue;
        };

        // expand: scan around the estimate on the capture, land mid-range.
        let scan: Vec<(f32, usize)> = (-k_max..=k_max)
            .filter_map(|k| {
                let dt = dt_cur + k as f32 * step_s;
                trial(audio, dt).map(|rs| (dt, in1s(&rs)))
            })
            .collect();
        let peak = scan.iter().map(|s| s.1).max().unwrap_or(0);
        let good: Vec<f32> = scan
            .iter()
            .filter(|s| peak > 0 && 2 * s.1 >= peak)
            .map(|s| s.0)
            .collect();
        let dt_exp = if good.is_empty() {
            dt_cur
        } else {
            0.5 * (good[0] + good[good.len() - 1])
        };

        let land = |dt: f32| (start + wrap(dt)).rem_euclid(15.0);
        let (l_cur, l_exp) = (land(dt_cur), land(dt_exp));
        let d_cur = unique(&at_phase(&slot, l_cur).results);
        let d_exp = unique(&at_phase(&slot, l_exp).results);
        s_cur += d_cur;
        s_exp += d_exp;
        e8_cur += (d_cur >= 8) as usize;
        e8_exp += (d_exp >= 8) as usize;
        n += 1;
        let signed = |l: f32| if l > 7.5 { l - 15.0 } else { l };
        println!(
            "  {start:5.1}          {:+6.2}  {d_cur:3}            {:+6.2}  {d_exp:3}",
            signed(l_cur),
            signed(l_exp)
        );
    }
    println!(
        "  mean over {n} starts: current {:.2} (>=8: {:.0}%)   expand {:.2} (>=8: {:.0}%)",
        s_cur as f32 / n as f32,
        100.0 * e8_cur as f32 / n as f32,
        s_exp as f32 / n as f32,
        100.0 * e8_exp as f32 / n as f32
    );
}

/// The acquisition trial's candidate cap, `decode_pipeline.rs`'s
/// `MAX_CAND`. Kept separate from the steady `max_cand()` knob so a cap
/// sweep on the steady path does not silently change the trial too.
const MAX_CAND_TRIAL: usize = 15;

/// Sanity check for `stage_a_12k` before trusting a sweep built on it:
/// at the recording's own phase, where does it put each pass-1
/// candidate, next to the coarse DT? `mirror_dt_window` measured each
/// station's decodable DT window at this phase; a correct Stage A lands
/// inside those windows for the strong stations.
#[test]
#[ignore = "diagnostic — stage_a_12k DT per candidate at phi = 0"]
fn mirror_stage_a_12k_positions() {
    let slot = load_slot();
    let mut spec = compute_spectrogram(&slot, FREQ_MAX);
    for t in spec_valid_rows()..spec.n_time {
        let row = t * spec.n_freq;
        for cell in &mut spec.data[row..row + spec.n_freq] {
            *cell = SpecCell::default();
        }
    }
    let pass1 = coarse_split(&spec);
    let refined = stage_a_12k(&slot[..prefix_samples()], pass1.clone());
    for (c, r) in pass1.iter().zip(refined.iter()) {
        println!(
            "  {:7.1} Hz  coarse {:+.3}  stageA12k {:+.3}  ({:+.0} ms)",
            c.freq_hz,
            c.dt_sec,
            r.dt_sec,
            (r.dt_sec - c.dt_sec) * 1000.0
        );
    }
}

/// Can the board's decoder reach the three stations that reach stage 3
/// and never decode?
///
/// On the landing band (+0.10..+0.26 s) with Stage A and the coarse
/// fallback, `K1JT EA3AGB`, `N1API F2VX` and `K1BZM EA3GP` pass pass 1,
/// the prefix split and the refine cap, then fail stage 3 at 85-100 % of
/// phases. Two causes call for different fixes: the candidate sits at
/// the wrong (freq, dt) and a better estimate would decode it, or no
/// position decodes it on this decoder and the loss is in the LLR / BP
/// depth. This grids each one's candidate across +-3 Hz x +-30 ms
/// through the board's own refine and stage 3 and prints, per cell,
/// whether full sync quality clears the gate and which depth decodes.
///
/// `MFSK_MIRROR_PHIS=a,b,...` picks phases (default the landing band).
#[test]
#[ignore = "diagnostic — freq x dt decodability of stations lost in stage 3"]
fn mirror_lost_station_grid() {
    use mfsk_core::ft8::decode_block::{SymMask, fill_symbol_spectra_goertzel};
    use mfsk_core::ft8::llr::sync_quality;

    const STATIONS: [(&str, f32); 5] = [
        ("K1JT EA3AGB -15", 1648.0),
        ("N1API F2VX 73", 1513.0),
        ("K1BZM EA3GP -09", 2696.0),
        // Controls: one that decodes at most phases, one partial.
        ("WM3PEN EA6VQ -09", 2157.0),
        ("W1DIG SV9CVY -14", 2734.0),
    ];
    // `MFSK_GRID_DF=lo,hi,step` (Hz) / `MFSK_GRID_DDT_MS=lo,hi,step`
    // widen or narrow the grid; `MFSK_GRID_ONLY=<freq>` keeps one station.
    let range = |var: &str, dflt: (f32, f32, f32)| -> Vec<f32> {
        let (lo, hi, st) = std::env::var(var)
            .ok()
            .map(|v| {
                let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                assert_eq!(f.len(), 3, "{var} wants lo,hi,step");
                (f[0], f[1], f[2])
            })
            .unwrap_or(dflt);
        let n = ((hi - lo) / st).round() as i32;
        (0..=n).map(|i| lo + i as f32 * st).collect()
    };
    let df_axis = range("MFSK_GRID_DF", (-3.0, 3.0, 0.5));
    let ddt_axis = range("MFSK_GRID_DDT_MS", (-30.0, 30.0, 5.0));
    let only: Option<f32> = std::env::var("MFSK_GRID_ONLY")
        .ok()
        .and_then(|v| v.parse().ok());
    let phis: Vec<f32> = std::env::var("MFSK_MIRROR_PHIS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![0.10, 0.14, 0.18, 0.22, 0.26]);

    let base = load_slot();
    for &phi in &phis {
        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let mut slot = base.clone();
        slot.rotate_left(k);
        let mut spec = compute_spectrogram(&slot, FREQ_MAX);
        for t in spec_valid_rows()..spec.n_time {
            let row = t * spec.n_freq;
            for cell in &mut spec.data[row..row + spec.n_freq] {
                *cell = SpecCell::default();
            }
        }
        let pass1 = coarse_split(&spec);
        let staged = stage_a_12k(&slot[..prefix_samples()], pass1.clone());
        println!("\n=== phi={phi:+.3}");
        for &(msg, f_ref) in STATIONS.iter() {
            if only.is_some_and(|o| o != f_ref) {
                continue;
            }
            let Some((i, c)) = pass1
                .iter()
                .enumerate()
                .filter(|(_, c)| (c.freq_hz - f_ref).abs() <= 3.2)
                .min_by(|a, b| {
                    (a.1.freq_hz - f_ref)
                        .abs()
                        .partial_cmp(&(b.1.freq_hz - f_ref).abs())
                        .unwrap()
                })
            else {
                println!("  {msg}: not in pass1");
                continue;
            };
            let sa = &staged[i];
            let audio: &[i16] = &slot;
            // One cell: full sync quality, then the decoders.
            let cell = |f: f32, dt: f32| -> (u32, [bool; 3]) {
                let mut cs = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
                fill_symbol_spectra_goertzel(&mut cs, audio, f, dt, SymMask::SyncOnly);
                let q = sync_quality(&cs);
                let cand = SyncCandidate {
                    freq_hz: f,
                    dt_sec: dt,
                    score: 1.0,
                };
                let mut ok = [false; 3];
                for (j, depth) in [
                    DecodeDepth::EMBEDDED,
                    DecodeDepth::BP_ONLY,
                    DecodeDepth::FULL,
                ]
                .into_iter()
                .enumerate()
                {
                    let p2 = refine_candidates_into(audio, vec![cand.clone()], 1);
                    let mut scratch = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
                    let got = process_candidates_into_with_cs_scratch_tuned(
                        audio,
                        p2,
                        depth,
                        DEFAULT_Q_THRESH,
                        DEFAULT_BP_MAX_ITER,
                        &mut scratch,
                    );
                    ok[j] = got
                        .iter()
                        .any(|r| unpack77(r.message77()).as_deref() == Some(msg));
                }
                (q, ok)
            };
            println!(
                "  {msg}  coarse {:.2} Hz {:+.3} s  stageA {:+.3} s  rank {i}",
                c.freq_hz, c.dt_sec, sa.dt_sec
            );
            println!(
                "    rows df Hz, cols ddt {:+.0}..{:+.0} ms from stageA; E=embedded B=+nsym2/3 LLR (O: OSD, compiled out here) digit=q/3 gate-pass .=gate-fail",
                ddt_axis[0],
                ddt_axis[ddt_axis.len() - 1]
            );
            let (mut n_e, mut n_b, mut n_o, mut q_max) = (0, 0, 0, 0u32);
            for &df in df_axis.iter() {
                let mut row = String::new();
                for &dd in ddt_axis.iter() {
                    let (q, ok) = cell(c.freq_hz + df, sa.dt_sec + dd / 1000.0);
                    q_max = q_max.max(q);
                    let ch = if ok[0] {
                        n_e += 1;
                        'E'
                    } else if ok[1] {
                        n_b += 1;
                        'B'
                    } else if ok[2] {
                        n_o += 1;
                        'O'
                    } else if q > DEFAULT_Q_THRESH {
                        char::from_digit(q / 3, 10).unwrap()
                    } else {
                        '.'
                    };
                    row.push(ch);
                    row.push(if df.abs() < 1e-3 && dd.abs() < 1e-3 {
                        '<'
                    } else {
                        ' '
                    });
                }
                println!("    {df:+4.1}  {row}");
            }
            println!("    cells: E={n_e} B={n_b} O={n_o}  max q={q_max}");
        }
    }
}

/// The library kernel against the binned prototype it was lifted from,
/// candidate by candidate at the recording's own phase. The two compute
/// the same bins with a different phase reference (per span here, per
/// bin in the library), so a near-tie can resolve differently; this
/// prints how often, rather than asserting it never does.
#[test]
#[ignore = "diagnostic — fine_sync_12k vs the mirror's binned prototype"]
fn mirror_fine_sync_lib_matches_prototype() {
    let slot = load_slot();
    let mut spec = compute_spectrogram(&slot, FREQ_MAX);
    for t in spec_valid_rows()..spec.n_time {
        let row = t * spec.n_freq;
        for cell in &mut spec.data[row..row + spec.n_freq] {
            *cell = SpecCell::default();
        }
    }
    let pass1 = coarse_split(&spec);
    let proto = stage_abc_12k_binned(&slot[..prefix_samples()], pass1.clone());
    let lib = mfsk_core::ft8::decode_block::fine_sync_12k(&slot[..prefix_samples()], &pass1);
    let mut differ = 0;
    for ((c, p), l) in pass1.iter().zip(proto.iter()).zip(lib.iter()) {
        let same = p.freq_hz == l.freq_hz && (p.dt_sec - l.dt_sec).abs() < 1e-4;
        differ += (!same) as usize;
        println!(
            "  {:7.2} {:+.3} -> proto {:7.2} {:+.3}  lib {:7.2} {:+.3}{}",
            c.freq_hz,
            c.dt_sec,
            p.freq_hz,
            p.dt_sec,
            l.freq_hz,
            l.dt_sec,
            if same { "" } else { "   <-- differs" }
        );
    }
    println!("  {differ} of {} candidates differ", pass1.len());
}

/// Where stage 3's time goes, shipped against fine sync, at one phase.
///
/// On hardware (2026-09-17, `sim_finesync_2026-09-17.log`) fine sync
/// took the early path from 1.10-1.27 s to 1.86-1.98 s with `cut=5`,
/// of which `fine=230 ms` and two retries account for well under half.
/// This splits the rest per candidate: pass-2 rank, full sync quality
/// (the `q > 6` gate stage 3 applies before any BP), the decode, and
/// host wall time for the stage-3 call (min of `REPS`, so a relative
/// figure; the board is ~2 cores of a much slower FPU).
///
/// The early path only, since at the board's locked phase `defer=0`.
/// `MFSK_MIRROR_PHIS=a,b` picks phases (default +0.20: the recording's
/// stations sit at median DT ~+0.2 s, the board locked at +0.005 s).
#[test]
#[ignore = "diagnostic — per-candidate stage-3 cost, shipped vs fine sync"]
fn mirror_stage3_cost() {
    use mfsk_core::ft8::decode_block::{SymMask, fill_symbol_spectra_goertzel, fine_sync_12k};
    use mfsk_core::ft8::llr::sync_quality;
    use std::time::Instant;
    const REPS: usize = 5;

    let phis: Vec<f32> = std::env::var("MFSK_MIRROR_PHIS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![0.20]);
    let base = load_slot();
    for &phi in &phis {
        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let mut slot = base.clone();
        slot.rotate_left(k);
        let prefix = &slot[..prefix_samples()];
        let mut spec = compute_spectrogram(&slot, FREQ_MAX);
        for t in spec_valid_rows()..spec.n_time {
            let row = t * spec.n_freq;
            for cell in &mut spec.data[row..row + spec.n_freq] {
                *cell = SpecCell::default();
            }
        }
        let coarse = coarse_split(&spec);

        for mode in ["shipped", "fine"] {
            let t0 = Instant::now();
            let pass1 = if mode == "fine" {
                fine_sync_12k(prefix, &coarse)
            } else {
                coarse.clone()
            };
            let t_fine = t0.elapsed();
            let (ready, deferred): (Vec<_>, Vec<_>) = pass1
                .iter()
                .cloned()
                .partition(|c| goertzel_window_end_sample(c.dt_sec) <= prefix_samples());
            let t1 = Instant::now();
            let p2 = pass2_split(prefix, ready.clone(), max_cand());
            let t_p2 = t1.elapsed();

            let timed = |c: &SyncCandidate| -> (u32, Option<String>, f64) {
                let mut cs = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
                fill_symbol_spectra_goertzel(
                    &mut cs,
                    prefix,
                    c.freq_hz,
                    c.dt_sec,
                    SymMask::SyncOnly,
                );
                let q = sync_quality(&cs);
                let mut best = f64::MAX;
                let mut msg = None;
                for _ in 0..REPS {
                    let r2 = refine_candidates_into(prefix, vec![c.clone()], 1);
                    let mut scratch = Box::new([[Cmplx::<f32>::new(0.0, 0.0); 8]; 79]);
                    let t = Instant::now();
                    let got = process_candidates_into_with_cs_scratch_tuned(
                        prefix,
                        r2,
                        DecodeDepth::EMBEDDED,
                        DEFAULT_Q_THRESH,
                        DEFAULT_BP_MAX_ITER,
                        &mut scratch,
                    );
                    best = best.min(t.elapsed().as_secs_f64() * 1e3);
                    msg = got.first().and_then(|r| unpack77(r.message77()));
                }
                (q, msg, best)
            };

            println!(
                "\n=== phi={phi:+.3} {mode}: p1={} ready={} defer={} p2={}  host fine={:.1} ms pass2={:.1} ms",
                pass1.len(),
                ready.len(),
                deferred.len(),
                p2.len(),
                t_fine.as_secs_f64() * 1e3,
                t_p2.as_secs_f64() * 1e3
            );
            println!("  rank     freq     dt   p2q  q  ms     message");
            let (mut n_gate, mut n_dec, mut ms_dec, mut ms_gate_fail, mut ms_bp_fail) =
                (0, 0, 0.0f64, 0.0f64, 0.0f64);
            let mut failed: Vec<SyncCandidate> = Vec::new();
            for (rank, (c, _, p2q)) in p2.iter().enumerate() {
                let (q, msg, ms) = timed(c);
                if q > DEFAULT_Q_THRESH {
                    n_gate += 1;
                }
                match &msg {
                    Some(_) => {
                        n_dec += 1;
                        ms_dec += ms;
                    }
                    None if q > DEFAULT_Q_THRESH => {
                        ms_bp_fail += ms;
                        failed.push(c.clone());
                    }
                    None => {
                        ms_gate_fail += ms;
                        failed.push(c.clone());
                    }
                }
                println!(
                    "  {rank:>4}  {:7.2} {:+.3}  {p2q:>3} {q:>2} {ms:6.2}  {}",
                    c.freq_hz,
                    c.dt_sec,
                    msg.as_deref().unwrap_or("-")
                );
            }
            // The retry round, where fine sync moved the candidate.
            let mut ms_fb = 0.0f64;
            let mut n_fb = 0;
            let mut n_fb_dec = 0;
            if mode == "fine" {
                for f in &failed {
                    let Some(i) = pass1.iter().position(|r| {
                        r.freq_hz.to_bits() == f.freq_hz.to_bits()
                            && r.dt_sec.to_bits() == f.dt_sec.to_bits()
                    }) else {
                        continue;
                    };
                    let c = &coarse[i];
                    if c.freq_hz.to_bits() == f.freq_hz.to_bits()
                        && c.dt_sec.to_bits() == f.dt_sec.to_bits()
                    {
                        continue;
                    }
                    let (q, msg, ms) = timed(c);
                    n_fb += 1;
                    ms_fb += ms;
                    n_fb_dec += msg.is_some() as usize;
                    println!(
                        "  retry {:7.2} {:+.3}      {q:>2} {ms:6.2}  {}",
                        c.freq_hz,
                        c.dt_sec,
                        msg.as_deref().unwrap_or("-")
                    );
                }
            }
            println!(
                "  gate-pass {n_gate}/{}  decoded {n_dec}  | ms: decoded {ms_dec:.1}  gate-fail {ms_gate_fail:.1}  bp-fail {ms_bp_fail:.1}  retries({n_fb}, {n_fb_dec} dec) {ms_fb:.1}  total {:.1}",
                p2.len(),
                ms_dec + ms_gate_fail + ms_bp_fail + ms_fb
            );
        }
    }
}

/// Where does the acquisition trial's DT statistic actually land, and
/// does the choice of statistic matter? (`decode_pipeline.rs`, the
/// acquisition block's `med`.)
///
/// The board applies `centre + median(trial DTs) + capture offset`. The
/// median is over whichever stations that one trial decoded, and a real
/// band's stations do not share a clock: on `qso3_busy` they spread
/// 1.07 s end to end (#358). So the statistic moves with the mix, and
/// two hardware acquisitions of the same recording landed at +0.385 and
/// -0.015 where the recording's own best phase is around +0.20.
///
/// This prints, per capture start: the winning trial's DT sample (n,
/// min, median, max) and, for each candidate statistic, the phase it
/// lands on and what the *steady* per-slot pipeline then decodes there
/// — the same scoring `mirror_acquisition_methods` uses, because the
/// grid is only as good as the slots that follow it.
///
/// `pooled` is the one estimator that is not a different average of the
/// same sample: every trial that decoded describes the same physical
/// grid, so their `centre + dt` values can be pooled into one larger
/// sample before the median is taken.
#[test]
#[ignore = "diagnostic — acquisition DT statistic: landing bias and what it costs"]
fn mirror_acquisition_dt_statistic() {
    use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phases};
    use mfsk_core::ft8::decode_block::decode_block_tuned;

    const ACQ_MAX_CAND: usize = 200;
    const ACQ_MAX_TRIALS: usize = 5;

    let slot = load_slot();
    let mut loopbuf = Vec::with_capacity(SLOT * 3);
    for _ in 0..3 {
        loopbuf.extend_from_slice(&slot);
    }
    let wrap = |mut dt: f32| {
        while dt > 7.5 {
            dt -= 15.0;
        }
        while dt <= -7.5 {
            dt += 15.0;
        }
        dt
    };
    let trial = |audio: &[i16], offset_s: f32| -> Option<Vec<DecodeResult>> {
        let off = ((offset_s * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        if audio.len() < off + SLOT {
            return None;
        }
        Some(decode_block_tuned(
            &audio[off..off + SLOT],
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            DecodeDepth::EMBEDDED,
            MAX_CAND_TRIAL,
            DEFAULT_BP_MAX_ITER,
        ))
    };
    let unique = |rs: &[DecodeResult]| -> usize {
        let mut m: Vec<String> = rs.iter().filter_map(|r| unpack77(r.message77())).collect();
        m.sort();
        m.dedup();
        m.len()
    };
    let sorted = |rs: &[DecodeResult]| -> Vec<f32> {
        let mut v: Vec<f32> = rs.iter().map(|r| r.dt_sec).collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        v
    };
    // The statistics, all over one trial's DT sample except `pooled`,
    // which is fed the whole acquisition's.
    const NAMES: [&str; 5] = ["median", "midrange", "trimmed", "snrtop", "pooled"];
    let stat = |name: &str, rs: &[DecodeResult]| -> f32 {
        let v = sorted(rs);
        match name {
            "median" => v[v.len() / 2],
            // Centre of the band's spread rather than of its population:
            // immune to a mix that happens to be all-early or all-late,
            // exposed instead to one outlier station (F5RXL, -0.77 s).
            "midrange" => 0.5 * (v[0] + v[v.len() - 1]),
            // The compromise: drop the extremes, average the rest.
            "trimmed" => {
                let inner = if v.len() >= 3 {
                    &v[1..v.len() - 1]
                } else {
                    &v[..]
                };
                inner.iter().sum::<f32>() / inner.len() as f32
            }
            // The loudest half only — their DT estimates are the ones
            // coarse sync places best.
            "snrtop" => {
                let mut by_snr: Vec<&DecodeResult> = rs.iter().collect();
                by_snr.sort_by(|a, b| {
                    b.snr_db
                        .partial_cmp(&a.snr_db)
                        .unwrap_or(core::cmp::Ordering::Equal)
                });
                let keep = by_snr.len().div_ceil(2);
                let mut d: Vec<f32> = by_snr[..keep].iter().map(|r| r.dt_sec).collect();
                d.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                d[d.len() / 2]
            }
            _ => unreachable!(),
        }
    };

    // `at_phase` is the expensive half; the statistics often agree.
    //
    // Each landing is scored twice: the unique decodes the steady path
    // gets there, and that path's own median decode DT — the number the
    // board logs as `median DT` and the one hardware runs were read as
    // "the grid landed 0.2 s out". Whether it can be read that way is
    // the question: it is a median over the *steady* window's station
    // mix, not over the trial's.
    let mut cache: std::collections::HashMap<i32, (usize, f32)> = std::collections::HashMap::new();
    let mut steady = |slot: &[i16], land: f32| -> (usize, f32) {
        let key = (land * 1000.0).round() as i32;
        *cache.entry(key).or_insert_with(|| {
            let out = at_phase(slot, land);
            let med = if out.results.is_empty() {
                f32::NAN
            } else {
                sorted(&out.results)[out.results.len() / 2]
            };
            (unique(&out.results), med)
        })
    };

    println!(
        "  start  trial sample (n  min  med  max) | {}",
        NAMES
            .iter()
            .map(|n| format!("{n:>8}: land dec  medDT"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    let mut sum = [0usize; NAMES.len()];
    let mut ge8 = [0usize; NAMES.len()];
    let mut lands: [Vec<f32>; NAMES.len()] = Default::default();
    let mut steady_med: [Vec<f32>; NAMES.len()] = Default::default();
    let mut n_starts = 0usize;
    for step in 0..30 {
        let start = step as f32 * 0.5;
        let cap_k = (start * 12_000.0) as usize;
        let audio = &loopbuf[cap_k..cap_k + REQUIRED_SAMPLES];
        let phases = acquire_slot_phases(
            audio,
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            ACQ_MAX_CAND,
            ACQ_MAX_TRIALS,
        );

        // The board's selection rule: every cluster is tried, the one
        // that decoded most wins, ties to the higher-scoring cluster.
        let mut best: Option<(usize, f32, Vec<DecodeResult>)> = None;
        let mut pool: Vec<f32> = Vec::new();
        for &(centre, _) in phases.iter() {
            let Some(got) = trial(audio, centre) else {
                continue;
            };
            if got.is_empty() {
                continue;
            }
            for r in got.iter() {
                pool.push(wrap(centre + r.dt_sec));
            }
            if best.as_ref().is_none_or(|(bn, _, _)| got.len() > *bn) {
                best = Some((got.len(), centre, got));
            }
        }
        let Some((_, centre, got)) = best else {
            println!("  {start:5.1}   (nothing decoded)");
            continue;
        };
        n_starts += 1;

        let v = sorted(&got);
        let mut line = format!(
            "  {start:5.1}         {:2} {:+.3} {:+.3} {:+.3} |",
            v.len(),
            v[0],
            v[v.len() / 2],
            v[v.len() - 1]
        );
        for (i, name) in NAMES.iter().enumerate() {
            let dt = if *name == "pooled" {
                // Pooled values are already grid-relative; the median is
                // taken after unwrapping onto the winner's branch so two
                // trials 15 s apart do not average to nothing.
                let mut p: Vec<f32> = pool.iter().map(|&x| wrap(x - centre)).collect();
                p.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                centre + p[p.len() / 2]
            } else {
                centre + stat(name, &got)
            };
            let land = (start + wrap(dt)).rem_euclid(15.0);
            let (dec, med) = steady(&slot, land);
            sum[i] += dec;
            ge8[i] += (dec >= 8) as usize;
            lands[i].push(if land > 7.5 { land - 15.0 } else { land });
            steady_med[i].push(med);
            line.push_str(&format!(
                " {:+6.2} {dec:3} {med:+.2} ",
                if land > 7.5 { land - 15.0 } else { land }
            ));
        }
        println!("{line}");
    }

    println!("\n  over {n_starts} starts:");
    for (i, name) in NAMES.iter().enumerate() {
        let n = lands[i].len().max(1) as f32;
        let mean = lands[i].iter().sum::<f32>() / n;
        let sd = (lands[i].iter().map(|l| (l - mean).powi(2)).sum::<f32>() / n).sqrt();
        let med_ok: Vec<f32> = steady_med[i]
            .iter()
            .cloned()
            .filter(|m| m.is_finite())
            .collect();
        let med_mean = med_ok.iter().sum::<f32>() / med_ok.len().max(1) as f32;
        println!(
            "    {name:>8}: dec {:.2}  >=8 {:3.0}%   landing mean {mean:+.3} sd {sd:.3} \
             (min {:+.3} max {:+.3})   steady median DT mean {med_mean:+.3}",
            sum[i] as f32 / n,
            100.0 * ge8[i] as f32 / n,
            lands[i].iter().cloned().fold(f32::INFINITY, f32::min),
            lands[i].iter().cloned().fold(f32::NEG_INFINITY, f32::max),
        );
    }
}

/// What a cheaper cold acquisition would cost in grid quality (#357).
///
/// Acquisition runs every cluster the tiles propose — five full-slot
/// decodes on top of three tiled searches — and on hardware that is
/// **15.2 s of compute on the decode task**, one whole slot period
/// (`logs/sim_acqstart_noclock_offset3000_2026-09-18.log`, 77.3 s to
/// 92.5 s, `CPU 1: IDLE1` throughout). The next slot's SpecBundle
/// therefore waits in the queue and is picked up 174 ms before that
/// slot ends, leaving no room for the unguarded `coarse` + `fine`
/// that follow: the slot finishes 763-1344 ms past key-up with
/// `dec=0`.
///
/// Best-of-N exists for a reason — stopping at the first trial that
/// decoded *anything* set the grid from one decode and was measured
/// costing whole minutes of dark band (see `decode_pipeline.rs`). But
/// "anything" is not the only stopping rule available: `grid_state`
/// already names three decodes as the bar a grid has to clear
/// (`LOCK_MIN_DECODES`), so a trial that clears it is, by the
/// receiver's own standard, a grid.
///
/// This scores the stopping rules against each other on the steady
/// pipeline's decodes, and reports the trials each one runs — the
/// number the board pays in seconds.
#[test]
#[ignore = "diagnostic — acquisition stopping rule: grid quality against trials run"]
fn mirror_acquisition_early_stop() {
    use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phases};
    use mfsk_core::ft8::decode_block::decode_block_tuned;

    const ACQ_MAX_CAND: usize = 200;
    const ACQ_MAX_TRIALS: usize = 5;
    /// Stop as soon as a trial decodes this many, `usize::MAX` for
    /// "try them all". `1` is what the board did before best-of-N.
    const RULES: [(&str, usize); 4] = [("all-5", usize::MAX), (">=5", 5), (">=3", 3), (">=1", 1)];

    let slot = load_slot();
    let mut loopbuf = Vec::with_capacity(SLOT * 3);
    for _ in 0..3 {
        loopbuf.extend_from_slice(&slot);
    }
    let wrap = |mut dt: f32| {
        while dt > 7.5 {
            dt -= 15.0;
        }
        while dt <= -7.5 {
            dt += 15.0;
        }
        dt
    };
    let unique = |rs: &[DecodeResult]| -> usize {
        let mut m: Vec<String> = rs.iter().filter_map(|r| unpack77(r.message77())).collect();
        m.sort();
        m.dedup();
        m.len()
    };
    let mut cache: std::collections::HashMap<i32, usize> = std::collections::HashMap::new();

    println!(
        "  start  trial decodes      | {}",
        RULES
            .iter()
            .map(|(n, _)| format!("{n:>5}: trials land  dec"))
            .collect::<Vec<_>>()
            .join("  ")
    );
    let mut sum = [0usize; RULES.len()];
    let mut trials_run = [0usize; RULES.len()];
    let mut n_starts = 0usize;
    for step in 0..30 {
        let start = step as f32 * 0.5;
        let cap_k = (start * 12_000.0) as usize;
        let audio = &loopbuf[cap_k..cap_k + REQUIRED_SAMPLES];
        let phases = acquire_slot_phases(
            audio,
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            ACQ_MAX_CAND,
            ACQ_MAX_TRIALS,
        );

        // Every trial once; the rules then read this same list, since a
        // stopping rule only ever truncates it.
        let mut got: Vec<(usize, f32)> = Vec::new(); // (decodes, dt applied)
        for &(centre, _) in phases.iter() {
            let off = ((centre * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
            if audio.len() < off + SLOT {
                got.push((0, 0.0));
                continue;
            }
            let rs = decode_block_tuned(
                &audio[off..off + SLOT],
                FREQ_MIN,
                FREQ_MAX,
                SYNC_MIN,
                DecodeDepth::EMBEDDED,
                MAX_CAND_TRIAL,
                DEFAULT_BP_MAX_ITER,
            );
            if rs.is_empty() {
                got.push((0, 0.0));
                continue;
            }
            let mut dts: Vec<f32> = rs.iter().map(|r| r.dt_sec).collect();
            dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            got.push((rs.len(), wrap(centre + dts[dts.len() / 2])));
        }
        if got.iter().all(|&(n, _)| n == 0) {
            println!("  {start:5.1}   (nothing decoded)");
            continue;
        }
        n_starts += 1;

        let mut line = format!(
            "  {start:5.1}  {:16} |",
            got.iter()
                .map(|&(n, _)| n.to_string())
                .collect::<Vec<_>>()
                .join(" ")
        );
        for (i, &(_, stop_at)) in RULES.iter().enumerate() {
            // Best-of-N over the trials the rule actually ran.
            let mut best: Option<(usize, f32)> = None;
            let mut ran = 0usize;
            for &(n, dt) in got.iter() {
                ran += 1;
                if n > 0 && best.is_none_or(|(bn, _)| n > bn) {
                    best = Some((n, dt));
                }
                if n >= stop_at {
                    break;
                }
            }
            let (_, dt) = best.expect("a start with no decodes was skipped above");
            let land = (start + wrap(dt)).rem_euclid(15.0);
            let key = (land * 1000.0).round() as i32;
            let dec = *cache
                .entry(key)
                .or_insert_with(|| unique(&at_phase(&slot, land).results));
            sum[i] += dec;
            trials_run[i] += ran;
            line.push_str(&format!(
                " {ran:6} {:+6.2} {dec:3} ",
                if land > 7.5 { land - 15.0 } else { land }
            ));
        }
        println!("{line}");
    }

    println!("\n  over {n_starts} starts:");
    for (i, (name, _)) in RULES.iter().enumerate() {
        println!(
            "    {name:>5}: dec {:.2}   trials {:.2}/acquisition",
            sum[i] as f32 / n_starts as f32,
            trials_run[i] as f32 / n_starts as f32,
        );
    }
}

/// **What emitting the SpecBundle earlier costs — the cost side only.**
///
/// The board's decoder blocks on the SpecBundle, so every row earlier
/// that `stage1_inc` emits is 80 ms more wall clock before key-up:
/// with the audio clock accurate the whole budget is ~1.1 s
/// (0.93 s of tail + 0.5 s to key-up − the 320 ms guard), and six
/// rows would be half of it again. What it buys is arithmetic and
/// needs no test. What it *costs* is not, and this measures that:
///
/// - the coarse search runs on a spectrogram with the tail rows zero,
///   so block 2's large-lag half is flattened (the board's own
///   `SPEC_EMIT_PAIR` comment: `needed_m = 162 + jz`, 13 rows at
///   `SYNC_LAG_S` = 1.0), and
/// - a shorter prefix moves candidates from ready to deferred, whose
///   first attempt cannot start until the slot ends.
///
/// **This mirror does not model the deadline** (see the header) — so
/// read the decode counts here as "what the pipeline can still find",
/// never as what the board will decode. Pairing a cost measured here
/// with the gain computed above is the whole point; the 2026-09-17
/// fine-sync decision came from this file's decode counts alone and
/// was wrong on the board for exactly that reason.
///
/// `MFSK_MIRROR_EMIT_PAIRS=87,85,83` picks the emit points,
/// `MFSK_MIRROR_SYNC_LAGS=1.0,0.6` the lag windows to cross them with,
/// `MFSK_GRID_PHASE=start,end,step` the phases each is averaged over.
#[test]
#[ignore = "diagnostic — the cost of emitting the SpecBundle earlier"]
fn mirror_emit_earlier() {
    let slot = load_slot();
    let phases: Vec<f32> = {
        let (lo, hi, dphi) = match std::env::var("MFSK_GRID_PHASE") {
            Ok(v) => {
                let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                assert_eq!(f.len(), 3, "MFSK_GRID_PHASE wants start,end,step");
                (f[0], f[1], f[2])
            }
            // The plateau the board locks within, at a step fine
            // enough not to alias the 5 ms comb (see the header).
            Err(_) => (-0.4, 0.4, 0.01),
        };
        let mut v = Vec::new();
        let mut step = 0i32;
        while lo + step as f32 * dphi <= hi + 1e-6 {
            v.push(lo + step as f32 * dphi);
            step += 1;
        }
        v
    };
    let list = |var: &str, default: &str| -> Vec<String> {
        std::env::var(var)
            .unwrap_or_else(|_| default.to_string())
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect()
    };
    let pairs: Vec<usize> = list("MFSK_MIRROR_EMIT_PAIRS", "87,86,85,84,83,82")
        .iter()
        .filter_map(|s| s.parse().ok())
        .collect();
    let lags: Vec<f32> = list("MFSK_MIRROR_SYNC_LAGS", "1.0")
        .iter()
        .filter_map(|s| s.parse().ok())
        .collect();

    // One run of a configuration over every phase: the decode count per
    // phase, and the message set per phase so a loss can be named.
    let run_cfg = |pair: usize, lag: f32| -> (Vec<usize>, Vec<Vec<String>>, f32, f32) {
        EMIT_PAIR_OVERRIDE.with(|c| c.set(Some(pair)));
        SYNC_LAG_OVERRIDE.with(|c| c.set(Some(lag)));
        let mut counts = Vec::new();
        let mut sets = Vec::new();
        let (mut ready, mut defer) = (0f32, 0f32);
        for &phi in &phases {
            let out = at_phase(&slot, phi);
            counts.push(out.results.len());
            ready += out.n_ready as f32;
            defer += out.n_deferred as f32;
            let mut msgs: Vec<String> = out
                .results
                .iter()
                .filter_map(|r| unpack77(r.message77()))
                .map(|m| m.trim().to_string())
                .collect();
            msgs.sort();
            msgs.dedup();
            sets.push(msgs);
        }
        EMIT_PAIR_OVERRIDE.with(|c| c.set(None));
        SYNC_LAG_OVERRIDE.with(|c| c.set(None));
        let n = phases.len() as f32;
        (counts, sets, ready / n, defer / n)
    };

    let mean = |v: &[usize]| v.iter().sum::<usize>() as f32 / v.len() as f32;
    let (base_counts, base_sets, _, _) = run_cfg(SHIP_EMIT_PAIR, SHIP_SYNC_LAG_S);
    println!(
        "recording: {}  phases: {} ({:+.2}..{:+.2})",
        std::env::var("MFSK_MIRROR_WAV").unwrap_or_else(|_| "qso3_busy.wav".into()),
        phases.len(),
        phases[0],
        phases[phases.len() - 1],
    );
    println!(
        "  baseline (pair {SHIP_EMIT_PAIR}, lag {SHIP_SYNC_LAG_S:.2}): mean dec {:.2}",
        mean(&base_counts)
    );
    println!("  pair  gain(ms)  lag   mean dec   ready  defer   lost  gained");

    let mut losses: Vec<(String, usize)> = Vec::new();
    for &lag in &lags {
        for &pair in &pairs {
            EMIT_PAIR_OVERRIDE.with(|c| c.set(Some(pair)));
            let gain = emit_gain_ms();
            EMIT_PAIR_OVERRIDE.with(|c| c.set(None));
            let (counts, sets, ready, defer) = run_cfg(pair, lag);
            let (mut lost, mut gained) = (0usize, 0usize);
            for (base, now) in base_sets.iter().zip(sets.iter()) {
                for m in base {
                    if !now.contains(m) {
                        lost += 1;
                        match losses.iter_mut().find(|(k, _)| k == m) {
                            Some((_, n)) => *n += 1,
                            None => losses.push((m.clone(), 1)),
                        }
                    }
                }
                gained += now.iter().filter(|m| !base.contains(m)).count();
            }
            println!(
                "  {pair:>4}  {gain:>8}  {lag:.2}   {:>8.2}   {ready:>5.1}  {defer:>5.1}   {lost:>4}  {gained:>6}",
                mean(&counts),
            );
        }
    }
    if !losses.is_empty() {
        losses.sort_by(|a, b| b.1.cmp(&a.1));
        println!("  stations lost at least once (phase-count):");
        for (m, n) in losses.iter().take(12) {
            println!("    {n:>3}x  {m}");
        }
    }
}

/// **The phases a 25 s capture cannot cut a slot at, and what
/// clamping the trial to the nearest reachable offset recovers.**
///
/// `acquire_slot_phases` returns centres in `(−7.5, +7.5]`; the board
/// turns one into a capture offset with `rem_euclid(SLOT)`, so a
/// negative centre becomes an offset in `(7.5, 15]` s. The capture is
/// [`REQUIRED_SAMPLES`] = 25 s, so a whole slot can only be cut at an
/// offset up to 10 s: **centres below −5 s have no slot behind them**
/// and the trial loop skips them (`c47a78fb` made the skip visible;
/// before that it was silent). Capturing two slots would fix it and
/// costs ~1.44 MB transiently — the board died of `rust_oom` trying,
/// see `uac::ACQUIRE_CAPTURE_SAMPLES`.
///
/// The fix has to come from the trial's own window, and the trial
/// already has the room: `decode_block_tuned` searches ±2.5 s about
/// wherever the slot is cut, and the phase applied is not the cut
/// position but that position corrected by the median DT of what the
/// trial decoded. The farthest any centre sits from the reachable
/// band `[0, 10]` s is 2.5 s (the band's complement is 5 s wide and
/// wraps at both ends), which is exactly the search half-width. So
/// clamping the offset circularly into the band — and measuring the
/// applied phase from the offset actually used rather than from the
/// centre — makes every centre reachable, with the decode's own
/// window absorbing the difference.
///
/// This scores both rules the way `mirror_acquisition_early_stop`
/// does: run the trials, apply the best, and decode a steady slot at
/// the grid that results.
#[test]
#[ignore = "diagnostic — unreachable acquisition trials, skipped vs clamped"]
fn mirror_acquisition_unreachable_phases() {
    use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phases};
    use mfsk_core::ft8::decode_block::decode_block_tuned;

    const ACQ_MAX_CAND: usize = 200;
    const ACQ_MAX_TRIALS: usize = 5;
    /// `grid_state::LOCK_MIN_DECODES` — the board stops trialling here.
    const STOP_AT: usize = 3;
    /// The last offset a whole slot fits behind.
    const MAX_OFF: usize = REQUIRED_SAMPLES - SLOT;

    let slot = load_slot();
    let mut loopbuf = Vec::with_capacity(SLOT * 3);
    for _ in 0..3 {
        loopbuf.extend_from_slice(&slot);
    }
    let wrap = |mut dt: f32| {
        while dt > 7.5 {
            dt -= 15.0;
        }
        while dt <= -7.5 {
            dt += 15.0;
        }
        dt
    };
    let unique = |rs: &[DecodeResult]| -> usize {
        let mut m: Vec<String> = rs.iter().filter_map(|r| unpack77(r.message77())).collect();
        m.sort();
        m.dedup();
        m.len()
    };
    // The nearest offset in `[0, MAX_OFF]` going round the 15 s
    // period, and how far it had to move.
    let reachable = |off: usize| -> (usize, i64) {
        if off <= MAX_OFF {
            (off, 0)
        } else if off - MAX_OFF < SLOT - off {
            (MAX_OFF, (off - MAX_OFF) as i64)
        } else {
            (0, -((SLOT - off) as i64))
        }
    };
    let mut cache: std::collections::HashMap<i32, usize> = std::collections::HashMap::new();

    println!(
        "  start | centres (* = no slot behind it)            | skip: land  dec | clamp: land  dec"
    );
    println!("  {:-<100}", "");
    let (mut sum_skip, mut sum_clamp, mut n_starts) = (0usize, 0usize, 0usize);
    let (mut n_unreach, mut n_cent) = (0usize, 0usize);
    let mut differed = 0usize;
    for step in 0..30 {
        let start = step as f32 * 0.5;
        let cap_k = (start * 12_000.0) as usize;
        let audio = &loopbuf[cap_k..cap_k + REQUIRED_SAMPLES];
        let phases = acquire_slot_phases(
            audio,
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            ACQ_MAX_CAND,
            ACQ_MAX_TRIALS,
        );

        // Every trial once, under both rules. `skip` is what the board
        // does today: an unreachable centre contributes nothing.
        // `clamp` cuts at the nearest reachable offset instead and
        // measures the applied phase from there.
        let mut marks = String::new();
        let mut got_skip: Vec<(usize, f32)> = Vec::new();
        let mut got_clamp: Vec<(usize, f32)> = Vec::new();
        for &(centre, _) in phases.iter() {
            let off = ((centre * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
            let (used, moved) = reachable(off);
            n_cent += 1;
            marks.push_str(&format!(
                "{centre:+5.2}{} ",
                if moved == 0 { ' ' } else { '*' }
            ));
            if moved != 0 {
                n_unreach += 1;
                got_skip.push((0, 0.0));
            }
            let rs = decode_block_tuned(
                &audio[used..used + SLOT],
                FREQ_MIN,
                FREQ_MAX,
                SYNC_MIN,
                DecodeDepth::EMBEDDED,
                MAX_CAND_TRIAL,
                DEFAULT_BP_MAX_ITER,
            );
            if rs.is_empty() {
                if moved == 0 {
                    got_skip.push((0, 0.0));
                }
                got_clamp.push((0, 0.0));
                continue;
            }
            let mut dts: Vec<f32> = rs.iter().map(|r| r.dt_sec).collect();
            dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            // Measured from the offset actually cut at, not from the
            // centre — the two are the same only when nothing moved.
            let dt = wrap(used as f32 / 12_000.0 + dts[dts.len() / 2]);
            if moved == 0 {
                got_skip.push((rs.len(), dt));
            }
            got_clamp.push((rs.len(), dt));
        }

        // Best-of-N with the board's early stop, over what each rule ran.
        let best_of = |got: &[(usize, f32)]| -> Option<(usize, f32)> {
            let mut best: Option<(usize, f32)> = None;
            for &(n, dt) in got.iter() {
                if n > 0 && best.is_none_or(|(bn, _)| n > bn) {
                    best = Some((n, dt));
                }
                if n >= STOP_AT {
                    break;
                }
            }
            best
        };
        let mut dec_at = |dt: f32| -> (f32, usize) {
            let land = (start + dt).rem_euclid(15.0);
            let key = (land * 1000.0).round() as i32;
            let dec = *cache
                .entry(key)
                .or_insert_with(|| unique(&at_phase(&slot, land).results));
            (if land > 7.5 { land - 15.0 } else { land }, dec)
        };
        let skip = best_of(&got_skip).map(|(_, dt)| dec_at(dt));
        let clamp = best_of(&got_clamp).map(|(_, dt)| dec_at(dt));
        if skip.is_none() && clamp.is_none() {
            println!("  {start:5.1} | {marks:42} | (nothing decoded either way)");
            continue;
        }
        n_starts += 1;
        sum_skip += skip.map_or(0, |(_, d)| d);
        sum_clamp += clamp.map_or(0, |(_, d)| d);
        if skip.map(|(_, d)| d) != clamp.map(|(_, d)| d) {
            differed += 1;
        }
        let show = |o: Option<(f32, usize)>| match o {
            Some((land, dec)) => format!("{land:+6.2} {dec:3}"),
            None => "  none   -".to_string(),
        };
        println!(
            "  {start:5.1} | {marks:42} | {} | {}",
            show(skip),
            show(clamp)
        );
    }

    println!(
        "\n  {n_unreach} of {n_cent} centres had no slot behind them\n  \
         over {n_starts} starts that decoded at all:\n    \
         skip  (today): dec {:.2}\n    clamp        : dec {:.2}   ({differed} starts differed)",
        sum_skip as f32 / n_starts as f32,
        sum_clamp as f32 / n_starts as f32,
    );
}

/// **How many of the refined candidates are the same station twice?**
///
/// The slot-budget note's largest number is that 44 % of the 15
/// stage-3 slots produce nothing, every slot, on the air. Depth does
/// not recover it (pass-1 ranks 16-25 convert at ~0.1 %), so the
/// question is what the 15 are spent on. Coarse sync works a 3.125 Hz
/// grid and a strong carrier lights more than one cell, so some of
/// those slots may be second and third looks at a station already
/// being refined — `dedup_by_message` runs *after* stage 3 and cannot
/// give the time back.
#[test]
#[ignore = "diagnostic — duplicate carriers inside the refined set"]
fn how_much_of_the_refine_budget_is_the_same_carrier_twice() {
    let slot = load_slot();
    let mut phases: Vec<f32> = Vec::new();
    let mut p = -0.40f32;
    while p <= 0.40 + 1e-6 {
        phases.push(p);
        p += 0.02;
    }
    // A duplicate for this purpose: close enough in frequency that one
    // carrier could produce both, and close enough in time that they
    // are the same transmission rather than two stations sharing a
    // slot. FT8 tone spacing is 6.25 Hz and the coarse grid is 3.125.
    const DF_HZ: f32 = 6.5;
    const DDT_S: f32 = 0.20;

    let (mut tot, mut dup, mut worst) = (0usize, 0usize, 0usize);
    for &phi in &phases {
        // `coarse_split` is what the board's pass 1 is; the refined
        // set is its top `max_cand`, which is what pass 2 keeps.
        let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
        let mut rotated = slot.clone();
        rotated.rotate_left(k);
        let spec = compute_spectrogram(&rotated, FREQ_MAX);
        let pass1 = coarse_split(&spec);
        let mut kept: Vec<(f32, f32)> = Vec::new();
        let mut d = 0usize;
        for c in pass1.iter().take(max_cand()) {
            if kept
                .iter()
                .any(|(f, t)| (f - c.freq_hz).abs() <= DF_HZ && (t - c.dt_sec).abs() <= DDT_S)
            {
                d += 1;
            } else {
                kept.push((c.freq_hz, c.dt_sec));
            }
            tot += 1;
        }
        dup += d;
        worst = worst.max(d);
    }
    println!(
        "\n  {} phases, {} refined candidates: {dup} are a carrier already in the set          ({:.1} %), worst slot {worst} of {}",
        phases.len(),
        tot,
        100.0 * dup as f32 / tot as f32,
        max_cand()
    );
    println!(
        "  At ~72 ms a candidate on the board, that is {:.0} ms a slot spent on a second\n           look at a station already being refined.",
        72.0 * dup as f32 / phases.len() as f32
    );
}

/// The three treatments of an incomplete Costas block 2, on the same
/// slot at the same phases.
///
/// Measured on a radio (CoreS3, 40 m, 2026-09-20) a partial-coverage
/// candidate outranked real stations on most slots and was rank 0 or 1
/// on several, so the shipping `Score` spends refined slots on a
/// population that cannot decode. `Gate` removes it and takes the real
/// signals below `-0.88 s` with it; `Shrink` ranks it fairly instead.
/// This is the host measurement that decides between them.
///
/// ```sh
/// cargo test -p mfsk-core --release --no-default-features \
///     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
///     --test ft8_embedded_pipeline_mirror \
///     -- --ignored --nocapture mirror_partial_block2_policies
/// ```
#[test]
#[ignore = "diagnostic — incomplete-block-2 policy comparison"]
fn mirror_partial_block2_policies() {
    let slot = load_slot();
    let (lo, hi, dphi) = match std::env::var("MFSK_GRID_PHASE") {
        Ok(v) => {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            assert_eq!(f.len(), 3, "MFSK_GRID_PHASE wants start,end,step");
            (f[0], f[1], f[2])
        }
        Err(_) => (-0.40, 0.40, 0.01),
    };
    let ceiling = partial_ceiling_s();
    println!(
        "  emit_pair={} valid_rows={} ceiling=±{ceiling:.2}s search=±{:.2}s",
        emit_pair(),
        spec_valid_rows(),
        sync_lag_s(),
    );

    // **`Gate` and an asymmetric window are the same experiment.**
    // The gate admits `lag <= 11` — `(173 - 150 - lag) / 2 + 1 >= 7` —
    // which is `dt <= +0.88 s` exactly, and it never touches negative
    // lag. So the `gate` arm below *is* `-1.00 .. +0.88`, computed the
    // expensive way (score the lag, then discard it) rather than by
    // not visiting it. The arm that is genuinely different is the
    // symmetric narrowing, which also gives up `-1.04 .. -0.88`.
    let ship = SHIP_SYNC_LAG_S;
    let gate = PartialBlock2::Gate {
        valid_rows: spec_valid_rows(),
    };
    // The window is quantised to whole rows (`jz = round(lag / 0.08)`),
    // so these four are every distinct search width between 0.80 and
    // the shipped 1.00 — asking for 0.84 gets 0.88's grid.
    let arms: [(&str, PartialBlock2, f32); 6] = [
        ("score +-0.80", PartialBlock2::Score, 0.80),
        ("score +-0.88", PartialBlock2::Score, 0.88),
        ("score +-0.96", PartialBlock2::Score, 0.96),
        ("score +-1.04", PartialBlock2::Score, 1.04),
        ("score ship  ", PartialBlock2::Score, ship),
        ("gate  ship  ", gate, ship),
    ];

    // Per arm: total decodes over the sweep, the distinct messages it
    // ever found, and how many pass-1 candidates sat past the ceiling.
    println!(
        "  arm           lags  phases  dec_total  dec_best  distinct_msgs  partial_p1  \
         partial_top15"
    );
    for (name, partial, lag) in arms {
        SYNC_LAG_OVERRIDE.with(|c| c.set(Some(lag)));
        // Lag steps actually visited, as a stand-in for coarse cost:
        // the loop is `-jz..=jz` and `jz` is the window in rows.
        let n_lags = 2 * (lag / (NSTEP_SAMPLES as f32 / 12_000.0)).round() as i32 + 1;
        let (mut total, mut best, mut n_partial, mut n_partial_top) =
            (0usize, 0usize, 0usize, 0usize);
        let mut msgs: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
        let mut phases = 0usize;
        let mut step = 0i32;
        while lo + step as f32 * dphi <= hi + 1e-6 {
            let phi = lo + step as f32 * dphi;
            let out = at_phase_with(&slot, phi, partial);
            total += out.results.len();
            best = best.max(out.results.len());
            for r in &out.results {
                if let Some(m) = unpack77(r.message77()) {
                    msgs.insert(m);
                }
            }
            // Pass 1 again, purely to count the population. Cheap
            // against the decode it sits beside, and it keeps the
            // count honest: it is the same ranking `run_slot_with`
            // just used.
            let k = ((phi * 12_000.0).round() as i64).rem_euclid(SLOT as i64) as usize;
            let mut rotated = slot.to_vec();
            rotated.rotate_left(k);
            let mut spec = compute_spectrogram(&rotated, FREQ_MAX);
            for t in spec_valid_rows()..spec.n_time {
                let row = t * spec.n_freq;
                for cell in &mut spec.data[row..row + spec.n_freq] {
                    *cell = SpecCell::default();
                }
            }
            let p1 = coarse_split_with(&spec, partial);
            for (rank, c) in p1.iter().enumerate() {
                // **Positive lag only.** Negative lag moves block 2
                // earlier, where it is always fully covered; what a
                // negative lag truncates is block 0, and block 0 is
                // absent from `sync_tail` anyway, so the `max()` of
                // the two is untouched. Counting `|dt|` overstates
                // this population — it was counted that way on the
                // board on 2026-09-20 and the gate arm then appeared
                // to leave candidates behind that it had in fact
                // removed.
                if c.dt_sec > ceiling {
                    n_partial += 1;
                    if rank < max_cand() {
                        n_partial_top += 1;
                    }
                }
            }
            phases += 1;
            step += 1;
        }
        println!(
            "  {name}  {n_lags:4}  {phases:6}  {total:9}  {best:8}  {:13}  {n_partial:10}  \
             {n_partial_top:13}",
            msgs.len(),
        );
    }
    SYNC_LAG_OVERRIDE.with(|c| c.set(None));
}

/// `jt9` beta6's decodes of ft8_lib's 22 recordings (the tier-B reference): `qso1` and `qso2` are
/// `191111_110130` and `191111_110200` of them.
const JT9_REF: &str = include_str!("fixtures/ft8_lib_jt9_beta6.tsv");

fn norm_msg(m: &str) -> String {
    m.split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_uppercase()
}

fn jt9_messages(stem: &str) -> Vec<String> {
    JT9_REF
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .filter_map(|l| {
            let f: Vec<&str> = l.split('\t').collect();
            (f[0] == stem).then(|| norm_msg(f[4]))
        })
        .collect()
}

/// **The S3 plan's stage 0 on the board's own pipeline** (`docs/notes/S3_GATE_OSD_ABLATION.md`): the gate, the board's
/// `fine_sync_12k` and OSD, added to `run_speculative_slot`'s flow one at a time, over the recording's phases. The
/// ablation in `hosttest/icap-app` runs one pass over the whole slot; this runs the board's: the spectrogram with its last
/// rows zero, `coarse_sync` split at 100-3000 Hz with the board's lag window and `sync_min` 1.0, the early half on the
/// 168 000-sample prefix and the late half on the slot, 15 candidates between them, the dedupe by message.
///
/// Per arm and recording: the distinct decodes of the known signals per phase (mean / min / max), the distinct known
/// signals over all phases, and **every decode that is not one** (the unexplained: a phantom, a signal the reference
/// lacks, or a copy of a strong signal at another frequency). Known = `QSO3_KNOWN_REAL_SIGNALS` (WSJT-X + JTDX) for
/// `qso3_busy`, `jt9`'s lines for `qso1` / `qso2`.
///
/// Needs the `osd` feature (without it OSD is a silent no-op and the OSD arms would repeat the arms before them):
/// ```sh
/// cargo test -p mfsk-core --release --no-default-features \
///     --features alloc,ft8,fft-extern,fixed-point,osd,internal-testing \
///     --test ft8_embedded_pipeline_mirror mirror_gate_and_osd -- --ignored --nocapture
/// ```
/// `MFSK_MIRROR_STEP=0.01` refines the phase grid (default 0.02 over -0.40..+0.40).
#[test]
#[ignore = "diagnostic - a phase sweep of the gate and OSD arms against the shipped pipeline"]
fn mirror_gate_and_osd() {
    assert!(
        cfg!(feature = "osd"),
        "run with the `osd` feature, or the OSD arms silently equal the arms without it"
    );
    let step: f32 = std::env::var("MFSK_MIRROR_STEP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.02);
    let n = (0.80 / step).round() as i32;
    let phases: Vec<f32> = (0..=n).map(|i| -0.40 + i as f32 * step).collect();
    let arms: [(&str, Arm); 7] = [
        ("ship", Arm::SHIP),
        (
            "ship+fine",
            Arm {
                fine: true,
                ..Arm::SHIP
            },
        ),
        (
            "ship+fine+osd",
            Arm {
                fine: true,
                osd: true,
                ..Arm::SHIP
            },
        ),
        (
            "gate",
            Arm {
                gate: true,
                ..Arm::SHIP
            },
        ),
        (
            "gate+fine",
            Arm {
                gate: true,
                fine: true,
                osd: false,
            },
        ),
        (
            "gate+osd",
            Arm {
                gate: true,
                osd: true,
                fine: false,
            },
        ),
        (
            "gate+fine+osd",
            Arm {
                gate: true,
                fine: true,
                osd: true,
            },
        ),
    ];
    let qso3_known: Vec<String> = ft8_qso3::QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .map(|g| norm_msg(g.msg))
        .collect();
    let recordings: [(&str, Vec<String>); 3] = [
        ("qso3_busy.wav", qso3_known),
        ("qso1.wav", jt9_messages("191111_110130")),
        ("qso2.wav", jt9_messages("191111_110200")),
    ];
    for (wav, known) in recordings {
        let path = format!(
            "{}/../embedded-poc/assets/{wav}",
            env!("CARGO_MANIFEST_DIR")
        );
        let mut slot = load_wav_i16(std::path::Path::new(&path));
        slot.truncate(SLOT);
        println!(
            "\n== {wav}: {} known, {} phases, max_cand {}",
            known.len(),
            phases.len(),
            max_cand()
        );
        println!(
            "{:<15} {:>6} {:>4} {:>4} {:>9}  unexplained decodes (distinct, phases they appear in)",
            "arm", "mean", "min", "max", "distinct"
        );
        for (name, a) in arms {
            ARM.with(|c| c.set(a));
            let t = std::time::Instant::now();
            let mut per_phase = Vec::new();
            let mut distinct = std::collections::BTreeSet::new();
            let mut extra: std::collections::BTreeMap<String, usize> = Default::default();
            for &phi in &phases {
                let out = at_phase(&slot, phi);
                let mut seen = std::collections::BTreeSet::new();
                for r in &out.results {
                    if let Some(m) = unpack77(r.message77()) {
                        seen.insert(norm_msg(&m));
                    }
                }
                let hits = seen.iter().filter(|m| known.contains(m)).count();
                for m in seen {
                    if known.contains(&m) {
                        distinct.insert(m);
                    } else {
                        *extra.entry(m).or_default() += 1;
                    }
                }
                per_phase.push(hits);
            }
            let mean = per_phase.iter().sum::<usize>() as f32 / per_phase.len() as f32;
            let extras: Vec<String> = extra.iter().map(|(m, n)| format!("{m} x{n}")).collect();
            println!(
                "{name:<15} {mean:>6.2} {:>4} {:>4} {:>9}  {}   [{:.0} s]",
                per_phase.iter().min().unwrap(),
                per_phase.iter().max().unwrap(),
                distinct.len(),
                if extras.is_empty() {
                    "-".to_string()
                } else {
                    extras.join("; ")
                },
                t.elapsed().as_secs_f32()
            );
        }
        ARM.with(|c| c.set(Arm::SHIP));
    }
}
