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
use mfsk_core::engine::ft4_coarse::{Ft4SavgBuilder, ft4_coarse_sync_from_savg};
use mfsk_core::engine::pipeline::{
    DecodeDepth, DecodeStrictness, process_candidate_precomputed,
};
use mfsk_core::engine::sync2d::ft4_sync_search_window;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::ddc::candidate_baseband;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
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
        let mut rest = samples;
        let mut done = None;
        while !rest.is_empty() {
            let room = SLOT_SAMPLES - self.audio.len();
            let take = room.min(rest.len());
            self.audio.extend_from_slice(&rest[..take]);
            self.savg.push(&rest[..take]);
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
}

/// Decode one captured slot.
///
/// This is the production per-candidate path `ft4-bench`'s SHIP arm
/// measures: DDC baseband, RMS normalise, narrowed Δt search, then
/// `process_candidate_precomputed`. The wideband `fft_cache` argument
/// is an **empty slice** — FT4's `snr_db` is a closed form over the
/// coarse candidate score (`pipeline::ft4_snr_db`) and never reads a
/// spectrum, which is what lets this receiver exist without the
/// 92 160-point transform no embedded backend can serve.
///
/// `depth` is [`DecodeDepth::EMBEDDED`] here. On the 560-file sweep
/// corpus `FULL` reaches 237 decodes against `EMBEDDED`'s 179, so this
/// gives up about a quarter of the recall on weak signals — the OSD
/// ladder is what costs, and at 12 candidates it is ~900 ms of a
/// 1 960 ms budget. Revisit when the budget has room, not before.
pub fn decode_slot(slot: &CapturedSlot) -> (Vec<Ft4Decode>, usize, i64) {
    let t0 = now_us();
    let cands = ft4_coarse_sync_from_savg(
        &slot.savg,
        FREQ_MIN_HZ,
        FREQ_MAX_HZ,
        SYNC_MIN,
        None,
        MAX_CAND,
    );
    let mut out: Vec<Ft4Decode> = Vec::new();
    for cand in &cands {
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
        });
    }
    (out, cands.len(), now_us() - t0)
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
