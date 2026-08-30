// SPDX-License-Identifier: GPL-3.0-or-later
//! FT4 receive pipeline — board-agnostic half.
//!
//! The fourth receiver in this binary, and the first whose coarse stage
//! runs **during** capture rather than after the slot. That is the
//! whole reason it can exist: FT4's slot is 7.5 s and its frame ends at
//! 5.54 s, leaving 1 960 ms to decode in, and the periodogram alone was
//! 758 ms of that (`docs/notes/FT4_BENCHMARK.md` §25-32). Accumulated a
//! block at a time through [`Ft4SavgBuilder`] it costs 6 ms after the
//! slot instead, and the worst single block is 25 % of its own
//! real-time budget (§33) — so it is fed straight from the audio
//! callback with no queue of its own.
//!
//! ## Where the boundary is
//!
//! Everything here is data flow: feed [`SlotAccum`] audio blocks from
//! wherever they come from and it hands back a [`CapturedSlot`] when
//! one is complete; [`decode_slot`] turns that into messages. No
//! threading, no global state, no peripheral — the handoff between an
//! audio callback and a decode task belongs to the board crate, which
//! has `std` and knows which tasks exist. Same shared/board split the
//! rest of this tree uses: data flow crosses it, callbacks do not.
//!
//! Two callers: `m5stack-cores3-app`'s FT4 boot mode (a radio through
//! `uac.rs`'s `AudioSink`) and its `ft4-demo` bin (the vendored WSJT-X
//! golden replayed at 12 kHz — the FT4 equivalent of the FT8
//! controller's `wav_sim`, which FT4 had no answer to until now). Both
//! feed the 256-sample blocks a UAC read produces, so the demo
//! exercises the shipping cadence rather than a friendlier one.
//!
//! ## What it does not do
//!
//! No TX, no QSO state machine, no band control. This is a monitor.
//!
//! [`Ft4SavgBuilder`]: mfsk_core::engine::ft4_coarse::Ft4SavgBuilder

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::{ft4_coarse_sync_from_savg, Ft4SavgBuilder};
use mfsk_core::engine::pipeline::{process_candidate_precomputed, DecodeDepth, DecodeStrictness};
use mfsk_core::engine::sync2d::ft4_sync_search_window;
use mfsk_core::ft4::ddc::candidate_baseband;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::ft4::Ft4;
use mfsk_core::msg::wsjt77::unpack77;
use num_complex::Complex;

/// One FT4 slot at 12 kHz: 7.5 s.
pub const SLOT_SAMPLES: usize = 90_000;

/// `ft4::decode`'s own private `SYNC_Q_MIN` — 16 sync symbols
/// (4 × Costas-4), at least half correct. Mirrored the same way
/// `ft4_bench` mirrors it, and with the same caveat: if the crate-side
/// value moves this silently stops matching the shipped gate.
const SYNC_Q_MIN: u32 = 8;

/// The coarse search, mirroring `ft4_wsjtx_samples.rs`'s `bench_assets`
/// so a number from this receiver is comparable to a number from
/// `ft4-bench`. `SYNC_MIN` is WSJT-X's own `syncmin = 1.2`
/// (`ft4_decode.f90:195`).
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 2700.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 100;

/// Δt search window, ±0.5 s about `dt = 0` in downsampled samples.
///
/// Narrower than WSJT-X's `[-344, 1012]` (±1.0 s), which it searches
/// because it cannot assume a clock. Measured lossless on both the
/// golden and an `ft4sim` DT sweep, and worth 1.5x on the stage
/// (§18-19): what it gives up is reach, not sensitivity.
const NARROW_WINDOW: (i32, i32) = (0, 667);

/// A finished slot: its audio, and the periodogram accumulated while
/// that audio was arriving.
pub struct CapturedSlot {
    pub audio: Vec<i16>,
    /// Already averaged — [`Ft4SavgBuilder::finish`] has run.
    pub savg: Vec<f32>,
    /// `esp_timer` microseconds at the moment the slot closed, so the
    /// decoder can report post-slot latency the way the benches do.
    pub closed_us: i64,
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Accumulates one slot of audio and its periodogram together.
///
/// Both halves have to advance in step: the DDC needs the slot's raw
/// samples (it re-filters them per candidate) and the coarse stage
/// needs the rows, so the builder cannot simply be run over the buffer
/// afterwards — that is the 758 ms this receiver exists to avoid.
pub struct SlotAccum {
    audio: Vec<i16>,
    savg: Ft4SavgBuilder,
}

impl Default for SlotAccum {
    fn default() -> Self {
        Self::new()
    }
}

impl SlotAccum {
    pub fn new() -> Self {
        Self {
            audio: Vec::with_capacity(SLOT_SAMPLES),
            savg: Ft4SavgBuilder::new(SLOT_SAMPLES),
        }
    }

    /// Feed the next block. Returns the finished slot on the block that
    /// completes it, and resets for the next.
    ///
    /// A block straddling the boundary is split, so the caller's block
    /// size never shifts the slot grid — which matters because a UAC
    /// read is not a divisor of 90 000.
    pub fn push(&mut self, samples: &[i16]) -> Option<CapturedSlot> {
        self.push_with_rows(samples, &mut |_| {})
    }

    /// [`push`](Self::push), forwarding each completed spectrum row to
    /// `on_row` — see [`Ft4SavgBuilder::push_with_rows`]. This is how a
    /// waterfall gets 152 rows a slot without a transform of its own;
    /// [`wf_row`] turns one into palette indices.
    pub fn push_with_rows(
        &mut self,
        samples: &[i16],
        on_row: &mut dyn FnMut(&[f32]),
    ) -> Option<CapturedSlot> {
        let mut rest = samples;
        let mut done = None;
        while !rest.is_empty() {
            let room = SLOT_SAMPLES - self.audio.len();
            let take = room.min(rest.len());
            self.audio.extend_from_slice(&rest[..take]);
            self.savg.push_with_rows(&rest[..take], on_row);
            rest = &rest[take..];
            if self.audio.len() == SLOT_SAMPLES {
                let prev = core::mem::replace(self, Self::new());
                done = Some(CapturedSlot {
                    audio: prev.audio,
                    savg: prev.savg.finish(),
                    closed_us: now_us(),
                });
            }
        }
        done
    }
}

/// One decoded transmission.
pub struct Ft4Decode {
    pub msg: String,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
    /// BP hard-error count, as `DecodedRow::hard_errors` wants it —
    /// the shared FT8 screen marks a row borderline at 24 or more.
    pub hard_errors: u32,
}

/// Milliseconds between the end of an FT4 frame and the end of its
/// slot: `7.5 s − (0.5 s TX offset + 105 × 48 ms)`.
///
/// The budget a **transceiver** has, and the one `ft4-bench` reports
/// against. A receiver that never transmits has until the next slot
/// closes instead — see [`decode_slot`]'s `budget_ms`.
pub const TX_TURNAROUND_BUDGET_MS: i64 = 1_960;

/// A whole FT4 slot, so a receive-only monitor can spend one.
///
/// Nothing is lost by overrunning [`TX_TURNAROUND_BUDGET_MS`] unless
/// the radio has to key up: slot `N + 1`'s audio is accumulating on
/// the capture side while slot `N` decodes, and only running past
/// *this* costs a slot.
pub const RX_ONLY_BUDGET_MS: i64 = 7_500;

/// What one slot's decode produced, and what it cost.
pub struct SlotOutcome {
    pub decodes: Vec<Ft4Decode>,
    /// Candidates the coarse stage produced.
    pub cands: usize,
    /// How many of them the loop actually started before the deadline.
    /// `tried < cands` is the cut.
    pub tried: usize,
    /// Microseconds from slot close to the loop returning — including
    /// the overshoot of the candidate that was already running when
    /// the deadline passed.
    pub elapsed_us: i64,
    /// Coarse score of the first candidate that was skipped, if any.
    /// The number that says what the cut actually gave up: these are
    /// baseline-normalised, so 1.2 is WSJT-X's own threshold and a cut
    /// landing near it dropped almost nothing.
    pub cut_at_score: Option<f32>,
}

/// Decode one captured slot, stopping when `budget_ms` after the slot
/// closed have passed.
///
/// The deadline is measured from [`CapturedSlot::closed_us`], not from
/// entry: a decoder that started late because it was still finishing
/// the previous slot has correspondingly less time, which is the
/// honest accounting and the one that keeps a receiver current.
///
/// **The cut takes the weakest candidates.** `ft4_coarse_sync` returns
/// them in descending coarse score, so truncation drops the tail —
/// which is both the cheapest thing to lose and, on the golden slot,
/// the candidates that decode last. Same shape as `fst4_monitor`'s
/// `run_candidate_loop`, which checks the deadline before starting each
/// candidate rather than trying to predict whether the next one fits;
/// predicting needs a per-candidate estimate that is itself wrong
/// whenever the band changes, and stopping one candidate early is worse
/// than overshooting by part of one.
///
/// This is the production per-candidate path `ft4-bench`'s SHIP arm
/// measures: DDC baseband, RMS normalise, narrowed Δt search, then
/// `process_candidate_precomputed`. The wideband `fft_cache` argument
/// is an **empty slice** — FT4's `snr_db` is a closed form over the
/// coarse candidate score (`pipeline::ft4_snr_db`) and never reads a
/// spectrum, which is what lets a receiver exist without the
/// 92 160-point transform no embedded backend can serve.
///
/// `depth` is [`DecodeDepth::EMBEDDED`]. On the 560-file sweep corpus
/// `FULL` reaches 237 decodes against `EMBEDDED`'s 179, so this gives
/// up about a quarter of the recall on weak signals — the OSD ladder is
/// what costs, and at 12 candidates it is ~900 ms of a 1 960 ms budget.
/// Revisit when the budget has room, not before.
pub fn decode_slot(slot: &CapturedSlot, budget_ms: i64) -> SlotOutcome {
    let deadline = slot.closed_us + budget_ms * 1_000;
    let cands = ft4_coarse_sync_from_savg(
        &slot.savg,
        FREQ_MIN_HZ,
        FREQ_MAX_HZ,
        SYNC_MIN,
        None,
        MAX_CAND,
    );
    let mut out: Vec<Ft4Decode> = Vec::new();
    let mut tried = 0usize;
    let mut cut_at_score = None;
    for cand in &cands {
        if now_us() >= deadline {
            cut_at_score = Some(cand.score);
            break;
        }
        tried += 1;
        let mut cd0 = candidate_baseband(&slot.audio, cand.freq_hz);
        rms_normalise(&mut cd0);
        let s2 = ft4_sync_search_window::<Ft4>(&cd0, cand, NARROW_WINDOW.0, NARROW_WINDOW.1);
        let Some(r) = process_candidate_precomputed::<Ft4>(
            cand,
            &[],
            &FT4_DOWNSAMPLE,
            DecodeDepth::EMBEDDED,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            (cd0, s2.freq_hz, s2.i0, s2.score),
            false,
            false,
        ) else {
            continue;
        };
        let Some(text) = r
            .message77()
            .try_into()
            .ok()
            .and_then(|m77: &[u8; 77]| unpack77(m77))
        else {
            continue;
        };
        // FT4 decodes raw candidates and dedups afterwards, the way
        // `decode_frame_impl`'s own FT4 arm does — two candidates can
        // land on one signal.
        if out.iter().any(|d| d.msg == text) {
            continue;
        }
        out.push(Ft4Decode {
            msg: text,
            freq_hz: r.freq_hz,
            dt_sec: r.dt_sec,
            snr_db: r.snr_db,
            hard_errors: r.hard_errors,
        });
    }
    SlotOutcome {
        decodes: out,
        cands: cands.len(),
        tried,
        elapsed_us: now_us() - slot.closed_us,
        cut_at_score,
    }
}

/// Frequency span the waterfall covers, matching `ui::waterfall`'s
/// own `WF_FREQ_LO_HZ`/`HI` and FT8's row builder — the panel is
/// shared, so the axis has to be.
pub const WF_FREQ_LO_HZ: f32 = 200.0;
/// See [`WF_FREQ_LO_HZ`].
pub const WF_FREQ_HI_HZ: f32 = 2_700.0;

/// Bin spacing of a row from [`Ft4SavgBuilder`]: `12 000 / 2304`.
const ROW_DF_HZ: f32 = 12_000.0 / 2_304.0;

/// Turn one row's power spectrum into [`crate::pipeline::WF_ROW_LEN`]
/// palette indices, 0..15.
///
/// Rows come from [`Ft4SavgBuilder::push_with_rows`], which hands over
/// the spectra it is already averaging — so the *transforms* are free.
/// This mapping is not, and it runs on the capture thread: 152 rows a
/// slot against §33's 21.3 ms per-block budget.
///
/// **Integer log2, not `log10`.** The first version took
/// `10 * log10(p)` per column and measured **623 µs per row, 94 ms a
/// slot, 10 % of the coarse stage** — display-only work costing a
/// tenth of the stage it rides on, which is not a reasonable price for
/// a picture. `f32::log10` is ~620 cycles on this core and there are
/// 240 of them per row.
///
/// So the level comes from the exponent instead, the way FT8's own
/// `decimate_pair_to_wf` has always done it: scale the power by the
/// row's mean, take the bit position of the MSB, and keep one
/// fractional bit for half-octave resolution. One float multiply per
/// column and the rest is integer.
///
/// The palette is 16 coarse steps over [`WF_SPAN_DB`], so quantising
/// the level to half-octaves (~1.5 dB) is below what it can show.
///
/// **Per-column maximum, not mean.** A column is ~2 bins wide
/// (2 500 Hz over 240 columns is 10.4 Hz, against 5.2 Hz bins) and an
/// FT4 tone is narrower than that, so averaging would halve every
/// signal against its neighbouring noise bin while leaving the noise
/// floor alone — visible as a waterfall where the signals are dimmer
/// than the background is bright.
///
/// **The scale is per-row and relative**, against the row's own mean
/// power. An absolute scale would need a calibrated input level, which
/// a receiver taking whatever a radio's USB audio hands it does not
/// have; this shows the band the way an operator reads one, relative
/// to its own noise.
pub fn wf_row(spectrum: &[f32]) -> [u8; crate::pipeline::WF_ROW_LEN] {
    const N: usize = crate::pipeline::WF_ROW_LEN;
    /// `1.0` in the fixed-point ratio below, i.e. `2^SCALE_LOG2`.
    const SCALE_LOG2: u32 = 10;
    /// Half-octaves the palette spans. [`WF_SPAN_DB`] dB of *power* is
    /// `WF_SPAN_DB / 3.01` octaves, doubled.
    const HALF_OCTAVES: u32 = 20;
    /// First column's low bin, and the bin step per column — the whole
    /// frequency mapping, folded into two constants so the loop is an
    /// add and two truncations rather than four divides.
    const BIN0: f32 = WF_FREQ_LO_HZ / ROW_DF_HZ;
    /// See [`BIN0`].
    const BIN_STEP: f32 = (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ) / (N as f32) / ROW_DF_HZ;

    let mut out = [0u8; N];
    if spectrum.is_empty() {
        return out;
    }
    let mean: f32 = spectrum.iter().sum::<f32>() / spectrum.len() as f32;
    // A silent input (all zero) has no scale to speak of; leave the
    // row black rather than dividing by it.
    if !(mean > 0.0) {
        return out;
    }
    // One divide per row, not per column.
    let inv = (1u32 << SCALE_LOG2) as f32 / mean;

    // Column -> bin range, walked rather than computed. The mapping is
    // fixed — it depends on nothing but the constants — yet the first
    // version recomputed it per row with four float divides per
    // column, 960 a row. Dropping `log10` only took 623 us/row to 384;
    // this is the rest of it.
    let mut edge = BIN0;

    for cell in out.iter_mut() {
        let lo = edge as usize;
        edge += BIN_STEP;
        let hi = ((edge as usize) + 1).min(spectrum.len());
        if hi <= lo {
            continue;
        }
        let mut peak = 0.0f32;
        for &p in &spectrum[lo..hi] {
            if p > peak {
                peak = p;
            }
        }
        // Ratio to the row's mean, in units of `2^SCALE_LOG2`.
        let scaled = peak * inv;
        if !(scaled >= 1.0) {
            continue;
        }
        let q = scaled as u32;
        // `31 - leading_zeros` is floor(log2), and the bit below the
        // MSB is the half-octave.
        let e = 31 - q.leading_zeros();
        let frac = if e > 0 { (q >> (e - 1)) & 1 } else { 0 };
        let half_oct = (e * 2 + frac).saturating_sub(SCALE_LOG2 * 2);
        *cell = ((half_oct * 15) / HALF_OCTAVES).min(15) as u8;
    }
    out
}

/// Unit-power normalisation, matching what
/// `process_candidate_basic_impl` applies to its own
/// `downsample_cached` output (WSJT-X `ft4_decode.f90:231-232`).
/// `candidate_baseband` deliberately does not do it, so every caller
/// must — `compute_llr`'s `LLR_SCALE` is calibrated against unit-RMS
/// input.
fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}
