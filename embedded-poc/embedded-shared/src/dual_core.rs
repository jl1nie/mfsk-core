//! Dual-core worker for Pass 2 / Stage 3 / Stage 2 (Phase E2)
//! candidate-loop parallelism.
//!
//! ## Protocol — host mpsc に対応する単一値転送チャネル
//!
//! 旧実装は `*mut Option<Vec<T>>` (out-pointer) と `xTaskNotify` slot を
//! 別チャネルとして併用していた。slot 跨ぎでデータ書き込みと完了通知
//! が desync し、Phase E2 で slot 2+ が 0 results になる症状が出た。
//!
//! 本実装は **FreeRTOS Queue による値転送1チャネル**に統一する。
//! 入力は `Box<Job>` の生ポインタを `JOB_Q` に send、結果は variant
//! 別の result queue (`*mut Vec<T>`) で送り返す。Queue が data 転送と
//! 完了通知を atomic に提供するため、host の `mpsc::sync_channel` と
//! 構造的に等価になる（depth=1 → `sync_channel(1)` 相当）。
//!
//! ## Memory
//!
//! Pass-2 and stage-3 cs builds use the Goertzel fill (Phase
//! 1.7.7-Stick, zero scratch) — the legacy BASIS Q15 scratch pair
//! this module used to thread through `init` / `pass2_split` /
//! `stage3_split` was removed in 0.8.0 (issue #162).
//!
//! ## Safety
//!
//! Job 内の生ポインタ (`audio`, `spec`, `allsum_ptr`) は dispatch 関数
//! の call frame に紐付いた借用を消したもの。dispatch 関数は `xQueueSend`
//! → 自分の half を計算 → `xQueueReceive` をブロック実行するため、
//! worker が触る間 main 側のスライスは必ず生存している。

use core::cell::UnsafeCell;
use core::ptr;
use core::sync::atomic::{AtomicI32, AtomicUsize, Ordering};

use esp_idf_svc::sys::{
    uxQueueMessagesWaiting, xQueueGenericCreate, xQueueGenericSend, xQueuePeek, xQueueReceive,
    xTaskCreatePinnedToCore, xTaskGetCoreID, QueueHandle_t,
};

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use mfsk_core::engine::sync::{bootstrap_dt_median, SyncCandidate};
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    coarse_sync_gated_spec, coarse_sync_with_lag, fine_sync_12k,
    process_candidates_into_with_cs_scratch_tuned, refine_candidates_into, RefinedCandidate,
    SpecFill, Spectrogram,
};

use crate::internal_pool::{cs_scratch_main, cs_scratch_worker};
use crate::pipeline::{self, Slot, SpecBundle};

/// ±lag window every coarse-sync call on this path pins, instead of
/// inheriting `mfsk_core`'s FT8 default (WSJT-X's own ±2.5 s).
///
/// Not a preference — the streaming pipeline is built around it.
/// `stage1_inc::SPEC_EMIT_PAIR` emits the SpecBundle once `m=0..173`
/// is filled, which is derived from `SYNC_LAG_S=1.0 → jz=13` giving
/// `needed_m = 162 + 13 = 175` (see that constant's own comment).
/// At ±2.5 s, `jz` is 31 and `needed_m` runs past `N_TIME=184`
/// entirely, so block-2 would silently flatten for far-lag
/// candidates and the whole 160 ms audio-tail overlap gain would be
/// built on a bound that no longer holds.
///
/// With a disciplined clock both boards run dt well inside ±0.5 s,
/// so the tight window is right here on its own merits too — and it
/// keeps [`SpeculativeOut::bootstrap_dt_med`] valid, which a ±2.5 s
/// list would not be (issue #280).
///
/// This doc used to add "and `time_sync::record_decode_dt`
/// re-anchors slot phase". It does not, since 2026-09-05: the
/// per-slot DT feedback was removed (lock-and-hold, see
/// `m5stack-cores3-app/src/decode_pipeline.rs`), and what places the
/// phase now is NTP/RTC or a one-shot cold acquisition, held
/// thereafter. Without a clock the window's margin is therefore
/// whatever acquisition left — measured 0.16 s off on hardware
/// 2026-09-16 and held there, `dec=6` against 8 at a better phase.
const EMBEDDED_SYNC_LAG_S: f32 = 1.0;

/// **Refuse to score a lag whose Costas block 2 is incomplete.**
/// `MFSK_FT8_BLOCK2_GATE=1` at build time; off by default.
///
/// The SpecBundle is emitted before the slot ends and declares the
/// full `n_time` regardless, because that is what sets the allsum's
/// stride — so its tail rows are present and zero. A lag past
/// `stage1_inc::max_lag_s` therefore scores block 2 over fewer Costas
/// symbols. That does not inflate the score (numerator and
/// denominator are sums over the same symbols; measured bit-identical
/// in `mfsk-core/tests/ft8_coarse_partial_blocks.rs`) — it raises the
/// **variance**, and a noisier score competes on equal terms with a
/// quiet one. On `qso3_busy` at the shipped ±1.0 s that puts two
/// such candidates in the pass-1 list, one at rank 8, inside the
/// refined top-`max_cand`: a stage-3 slot (~72 ms) spent on something
/// that cannot decode, from a budget where only ~11 of 15 finish
/// before the reply boundary.
///
/// **A deliberate divergence from WSJT-X**, which scores the
/// truncated block and takes the result (`sync8.f90` guards the read
/// and skips). Upstream never needs it: upstream's spectrogram ends
/// where its slot ends. This pipeline's does not, and the embedded
/// FT8 path is already a documented divergence (single-pass BP,
/// `LlrEffort::Minimal`, no OSD, SIC or AP) for the same reason —
/// a bounded slot on a battery is a different trade from a desktop.
///
/// Off by default until an on-air A-B says what it costs in
/// *decodes*; candidate counts are not the quantity that decides it.
/// **No-op at the shipped search window since 2026-09-20, retained on
/// purpose.** `decode_pipeline` now searches the coverage ceiling
/// itself, so no lag it visits has an incomplete block 2 and there is
/// nothing here to refuse. Kept as the other half of an experiment
/// that is still open — emit earlier and gate, rather than emit at 87
/// and search narrower — which deleting it would close. See
/// `mfsk_core::ft8::decode_block::PartialBlock2::Gate`.
pub const FT8_BLOCK2_GATE: bool = option_env!("MFSK_FT8_BLOCK2_GATE").is_some();

/// The lag the next coarse search will use, in milliseconds.
///
/// [`EMBEDDED_SYNC_LAG_S`] is what a *working* grid needs: ±1.0 s is
/// as far as the emitted SpecBundle can score anyway, since block 2's
/// last Costas sits at row 162 and the emit carries rows 0..173
/// (`162 + 1.0/0.08 = 175`).
///
/// But a grid that is not yet placed can be further out than that, and
/// then the per-slot search cannot see the band at all: measured on a
/// radio (2026-09-19) the grid sat 1.65 s out, every slot decoded 0,
/// and the receiver spent two minutes and two cold acquisitions
/// getting back — while the stations were in the audio the whole time.
///
/// So while the grid is unproven the caller widens this, and the full
/// slot's 184 rows put the ceiling at `(184 - 162) * 0.08` = 1.76 s.
/// Past that there is no spectrogram to score against and the answer
/// is acquisition, which searches the whole period by construction.
static SYNC_LAG_MS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new((EMBEDDED_SYNC_LAG_S * 1_000.0) as u32);

/// Widen or narrow the coarse search. Takes effect on the next slot.
pub fn set_sync_lag_s(lag_s: f32) {
    SYNC_LAG_MS.store(
        (lag_s.clamp(0.1, 1.75) * 1_000.0) as u32,
        core::sync::atomic::Ordering::Release,
    );
}

pub(crate) fn sync_lag_s() -> f32 {
    SYNC_LAG_MS.load(core::sync::atomic::Ordering::Acquire) as f32 / 1_000.0
}

/// One slot's Phase-C output. Both apps consume it identically:
/// `spec` for `xsnr2_db_simple`, `slot` for `wav_idx` /
/// `inc_total_us`, `results` for the UI + QSO FSM, and the `t_*`
/// timestamps for the per-slot timing log.
pub struct SpeculativeOut {
    pub spec: Box<SpecBundle>,
    pub slot: Box<Slot>,
    pub results: Vec<DecodeResult>,
    pub n_pass1: usize,
    pub n_ready: usize,
    pub n_deferred: usize,
    /// Candidates the early (prefix) half actually handed to stage 3.
    /// With `results.len()` and the `early` timing it gives the cost
    /// per candidate, which is what shows whether an allocation change
    /// moved work or merely moved it around: a BP that converges exits
    /// on the iteration it converges (`fec::ldpc::bp`), a hopeless one
    /// leaves via the stall detector at ~10-15, and a marginal one
    /// burns all `bp_max_iter`. Selecting better candidates is
    /// therefore also selecting cheaper ones.
    pub n_early_refined: usize,
    /// Of `results`, how many were decoded **before the slot
    /// boundary** — in time to be answered in the next period. See
    /// [`FT8_KEY_UP_AFTER_SLOT_END_US`]: the reply is committed at the
    /// boundary, while stage 3 is allowed to claim for another 0.5 s,
    /// so `results.len() - n_in_time` is decoded for the screen and
    /// for the period after next, not for this exchange.
    pub n_in_time: usize,
    /// Pass 2's fixed head/tail split, in microseconds for the whole
    /// slot: `(main's own half, main blocked on the worker, the
    /// worker's own half)`. See [`PASS2_MAIN_US`].
    ///
    /// `wait` is the idle the main core paid; `main - worker` is the
    /// imbalance, and its sign says which core drew the cheaper half.
    /// A large `wait` with a large `worker` means the split is simply
    /// uneven; a large `wait` with a small `worker` means the worker
    /// was late to start, which is a scheduling question rather than a
    /// partitioning one.
    pub pass2_us: (i64, i64, i64),
    /// Pass-1 candidates whose `|dt_sec|` is past
    /// [`crate::stage1_inc::SPEC_EMIT_MAX_LAG_S`], i.e. scored on two
    /// Costas blocks instead of three because the emitted spectrogram
    /// does not reach block 2 at that lag.
    ///
    /// **Observation only — nothing here acts on it.** The gate that
    /// would (`FT8_BLOCK2_GATE`) stays off; this counts how often it
    /// would have anything to do. The last knob wired on the strength
    /// of a comment's reasoning rather than a measurement turned out
    /// to be a no-op after it had been threaded through three boards
    /// (`valid_rows`, 2026-09-20), so the count comes first.
    pub n_gate2_p1: usize,
    /// The best (lowest) pass-1 rank held by such a candidate, or
    /// `u16::MAX` if there is none.
    ///
    /// This is the number that decides whether the gate is worth
    /// anything: pass 1 is score-ordered and stage 3 takes from the
    /// top, so a two-block candidate at rank 25 of 30 costs nothing
    /// and one at rank 8 costs a stage-3 slot (~72 ms on this board).
    /// `max_lag_s`'s own doc reports rank 8 on `qso3_busy`; whether a
    /// live band does the same has never been measured.
    pub gate2_best_rank: u16,
    /// Stage-3 candidates the [`DecodeConfig::budget_ms`] deadline
    /// stopped from being tried (speculative + deferred, first attempts
    /// and coarse-position retries alike). `0` when the budget is
    /// disabled or the slot finished inside it.
    pub n_cut: usize,
    /// Candidates whose fine-synced position failed stage 3 and were
    /// tried again at their coarse position ([`DecodeConfig::fine_sync`]).
    /// `0` when fine sync is off.
    pub n_fallback: usize,
    /// DT median over the top-5 highest-score pass1 candidates.
    /// `None` if pass1 was empty. Empirically lines up with the
    /// confirmed-decode median to within ±70 ms on reference
    /// fixtures, gated by the `ft8_coarse_sync_bootstrap`
    /// integration test.
    ///
    /// **No controller reads it any more** — both apps destructure it
    /// as `bootstrap_dt_med: _`. It was the auto-sync path's
    /// cold-start fallback for slots with zero confirmed decodes;
    /// lock-and-hold (2026-09-05) dropped that ±0.2 s/slot nudge
    /// because its own doc admitted it is "essentially always
    /// `Some`, just a small near-random value when the true signal is
    /// outside ±1 s" — a random walk rather than an acquisition, and
    /// acquisition is `ft8::acquire`'s job. Kept as a measurement the
    /// bench and the test still pin, not as a live input.
    pub bootstrap_dt_med: Option<f32>,
    /// `esp_timer_get_time()` right after `recv_box::<SpecBundle>` returns.
    pub t_post_recv: i64,
    /// after `coarse_sync_split_with_allsum`.
    pub t_coarse_done: i64,
    /// after [`fine_sync_split`]; equal to `t_coarse_done` when
    /// [`DecodeConfig::fine_sync`] is off.
    pub t_fine_done: i64,
    /// after the speculative pass-2 + stage-3 on `audio_prefix` finish
    /// (or immediately, if `ready` was empty).
    pub t_early_done: i64,
    /// after `recv_box::<Slot>` returns.
    pub t_slot_recv: i64,
    /// after the deferred pass-2 + stage-3 on `slot.audio` finish.
    pub t_done: i64,
    /// This slot was dropped without decoding: its SpecBundle reached
    /// the decode task too late to finish before key-up
    /// ([`DecodeConfig::slot_floor_ms`]). `results` is empty and every
    /// candidate count is `0`; the audio was received and dropped, so
    /// the pipeline stays drained.
    pub skipped: bool,
    /// The three strongest pass-1 candidates as coarse sync ranked
    /// them: `(score, dt_sec, freq_hz)` each, `n_pass1` says how many
    /// are real.
    ///
    /// A slot that decodes nothing says nothing about *why* on its
    /// own: `p1=30` only means the candidate list hit its limit, which
    /// it does on noise as readily as on signal. The score does say —
    /// `sync_min` is 1.0 because that is about where the noise floor
    /// sits, so a top candidate at 1.x is an empty band and one at 3-5
    /// is a station the decoder failed to read.
    pub top3: [(f32, f32, f32); 3],
    /// What [`DecodeConfig::slot_end_hint`] answered when this slot
    /// arrived, or `None` if the caller supplies no hint. Reported so
    /// a caller can log it against `slot.slotend_us` — the two date
    /// the same boundary from the audio sink's clock and from
    /// stage1_inc's, and [`Self::skipped`] is decided on the first of
    /// them before the second exists.
    pub slot_end_hint_us: Option<i64>,
}

/// Per-slot decoder configuration shared between Phase-C speculative
/// callers. Grouped into a single struct so
/// [`run_speculative_slot`] doesn't need to accept ~10 positional
/// `f32` / `usize` / `u32` args (Gemini PR #123 round-16 review).
#[derive(Debug, Clone, Copy)]
pub struct DecodeConfig {
    /// coarse_sync lower band edge in Hz (apps use `100.0`).
    pub freq_min: f32,
    /// coarse_sync upper band edge in Hz (apps use `3_000.0`).
    pub freq_max: f32,
    /// coarse_sync ratio threshold (apps use `1.0`).
    pub sync_min: f32,
    /// Maximum pass-1 candidates fed into pass-2 / stage-3.
    pub pass1_limit: usize,
    /// Maximum refined candidates per half passed into stage-3.
    pub max_cand: usize,
    /// `process_candidates` sync-quality early-reject threshold.
    pub q_thresh: u32,
    /// BP iteration cap per LLR variant.
    pub bp_max_iter: u32,
    /// LLR-variant staircase depth (embedded ship uses
    /// [`DecodeDepth::EMBEDDED`]).
    pub depth: DecodeDepth,
    /// Wall-clock budget for stage 3 (BP/OSD), in milliseconds from the
    /// moment this slot's SpecBundle arrived (`t_post_recv`). `0`
    /// disables it — the historical behaviour, where a slow slot runs
    /// every committed candidate to completion however long that takes
    /// and the overrun steals the next slot's headroom (#357).
    ///
    /// When set, [`run_speculative_slot`] passes the derived absolute
    /// deadline to both the speculative and the deferred
    /// [`stage3_split`], whose work-stealing loop stops claiming
    /// candidates once it is reached. `coarse_sync` and `pass2` (refine)
    /// are *not* bounded — stage 3 is where the wall-clock variance is
    /// (`project_phasewise_hotspot_survey_246`).
    pub budget_ms: i64,
    /// Refine every pass-1 candidate's frequency and DT with WSJT-X's
    /// `ft8b.f90` Stages A/B/C before the prefix partition
    /// ([`mfsk_core::ft8::decode_block::fine_sync_12k`]), and give a
    /// candidate that then fails stage 3 one more attempt at its coarse
    /// position.
    ///
    /// The retry is not optional decoration. Fine sync alone erased
    /// stations the coarse position decodes — `N1PJT HB9CQK` on
    /// `qso3_busy` and `CQ LZ1JZ KN22` on `qso2`, at every phase — and
    /// the retry is what makes it a strict addition: measured on the
    /// host mirror of this function (`mfsk-core/tests/
    /// ft8_embedded_pipeline_mirror.rs`, 2026-09-17), acquisition plus
    /// decode on `qso2` falls from 4.96 to 4.65 without it, and
    /// `qso3_busy` from 8.29 to 8.00.
    pub fine_sync: bool,
    /// Pick pass 1 by the hard-sync gate instead of by coarse score
    /// ([`mfsk_core::ft8::decode_block::coarse_sync_gated_spec`]): the
    /// candidates are the ones whose 21 Costas symbols mostly have the
    /// expected tone as the strongest of the 8 bins, read from the same
    /// spectrogram `stage1_inc` built, and **that order is the ranking**
    /// (the first `max_cand` of the ready half and of the deferred half
    /// go to pass 2, not the best by block-0 quality, which undoes it).
    ///
    /// **Off, and nothing measured on a board yet.** On the host's mirror
    /// of this function (`mfsk-core/tests/ft8_embedded_pipeline_mirror.rs`,
    /// `mirror_gate_and_osd`, 81 phases of `qso3_busy`) the gate alone adds
    /// 0.4 decodes of 20 (6.56 to 6.98); with OSD (`depth`) it adds 2.25
    /// (8.81), and with `fine_sync` as well 4.8 (11.35), with no unexplained
    /// decode. Its own cost on the board is the unmeasured part: it scores
    /// the whole band on one core from the spectrogram (the allsum
    /// `stage1_inc` built is not used), where `coarse_sync_split_with_allsum`
    /// splits the score over both cores. `FT8_BLOCK2_GATE` applies as for
    /// the score search. `docs/notes/S3_GATE_OSD_ABLATION.md`.
    pub hard_sync_gate: bool,
    /// Stop claiming stage-3 candidates this many milliseconds before
    /// this station's key-up, [`FT8_KEY_UP_AFTER_SLOT_END_US`] past the
    /// slot boundary. `0` leaves only [`budget_ms`](Self::budget_ms).
    ///
    /// `budget_ms` is anchored to the SpecBundle's arrival and was
    /// derived assuming that arrival is at least 1 336 ms before slot
    /// end. It is not: measured on hardware 2026-09-18 it runs anywhere
    /// from 1 027 to 1 936 ms depending on where the grid sits, and
    /// below 1 336 ms the budget's own deadline falls after key-up.
    /// That alone put slots 472 ms past key-up. This bound is anchored
    /// to the slot boundary itself instead, and applies from the moment
    /// the slot is known — peeked from `slot_q` while the early path is
    /// still running, exact once it has been received.
    ///
    /// It has to be a margin rather than key-up itself because a
    /// deadline only stops *claiming*: a candidate already in BP runs to
    /// the end. Across the same captures (three runs, 60 slots), work
    /// finished up to 313 ms past the deadline that stopped it.
    pub key_up_guard_ms: i64,
    /// Least time a slot may have before key-up and still be worth
    /// starting, in milliseconds. `0` disables the check.
    ///
    /// [`Self::key_up_guard_ms`] bounds stage 3 only — `coarse_sync`
    /// and fine sync run to completion whatever the clock says, and on
    /// this board they are ~370 ms together. A slot whose bundle
    /// arrives with less than that left therefore cannot help
    /// overrunning key-up, and it was measured doing exactly that:
    /// after a cold acquisition (15.2 s of compute on this same task)
    /// the next bundle was picked up 174 ms before its slot ended and
    /// the slot finished 763-1344 ms past key-up with `dec=0`
    /// (`m5stack-cores3-app/logs/sim_*_2026-09-18.log`, every run).
    ///
    /// Below this floor the slot is dropped instead: its `Slot` is
    /// received and freed, nothing is decoded, and the cores go to the
    /// audio pipeline, which after an acquisition has a backlog to
    /// clear. It costs no decodes — the slots this fires on decoded
    /// zero — and it is what makes the key-up bound true for the whole
    /// slot rather than for stage 3 alone.
    ///
    /// Minimum slack before key-up, in milliseconds, for
    /// [`Self::fine_sync`] to run at all.
    ///
    /// Fine sync costs ~292 ms on this board against a budget of about
    /// 1.1 s, and it is worth that only when stage 3 still has room
    /// afterwards. Rather than moving it past key-up — which meant
    /// carrying work into the next slot, and the next slot is where the
    /// audio pipeline needs the cores — it simply does not run when the
    /// slot arrived without the time for it.
    ///
    /// 0 runs it always, which is what the boards without a key-up
    /// bound want.
    pub fine_sync_min_slack_ms: i64,
    /// Share the stage-3 candidate budget across both halves by coarse
    /// rank, instead of letting the early half take all of it.
    ///
    /// Off, the early half refines up to [`Self::max_cand`] and the
    /// late half gets `max_cand - n_early_refined`, which is nothing
    /// whenever the early half fills it — so a candidate whose audio
    /// window does not fit the prefix is never refined at all, however
    /// strong it is. On, the top `max_cand` of pass 1 by coarse score
    /// decides the split: each half gets as many stage-3 slots as it
    /// holds of that set. **The total is unchanged**, so this is a
    /// reallocation, not more work.
    ///
    /// Measured on the host mirror (`mirror_emit_earlier`, 81 phases,
    /// fine sync on, `max_cand` 15): at the shipping emit point
    /// `qso3_busy` is unchanged at 8.04 while `qso1` goes 1.83 → 2.60
    /// and `qso2` 1.96 → 3.07, and it is what makes an earlier emit
    /// nearly free (`qso3` at 320 ms earlier: 7.21 → 7.79).
    ///
    /// What the mirror cannot say, and the board must: the slots it
    /// moves to the late half are spent *after* SlotEnd, where only
    /// `FT8_KEY_UP_AFTER_SLOT_END_US` minus the guard remains. Round
    /// 19 tried a different route to the same end — an `i < max_cand`
    /// guard that pushed high-dt candidates into `deferred` — and it
    /// grew post-SlotEnd wallclock by 190-400 ms for no recall, which
    /// is why this is a flag and not a rewrite.
    pub share_cand_budget: bool,
    /// Needs [`Self::slot_end_hint`]; without one the floor cannot be
    /// applied and is ignored.
    pub slot_floor_ms: i64,
    /// Where the slot now being decoded ends, in `esp_timer_get_time()`
    /// microseconds, or `None` if the caller cannot say yet.
    ///
    /// Called once per slot, right after the SpecBundle arrives, and
    /// only to apply [`Self::slot_floor_ms`].
    ///
    /// **It has to come from the audio thread's clock.** The obvious
    /// source — the bundle's own emit timestamp plus the audio it was
    /// emitted with — is wrong in exactly the case the floor exists
    /// for: `SpecBundle::audio_len` counts audio stage1_inc has
    /// *consumed*, and a decode task hogging a core (cold acquisition)
    /// starves stage1_inc too, so it emits late with the same sample
    /// count and the estimate slides late with it. Measured
    /// 2026-09-18: the estimate landed ~0.8 s past the real boundary
    /// and the floor never fired, on the very slot that then finished
    /// 738 ms past key-up. A hint anchored to
    /// `mfsk_app_shared::time_sync`'s capture-slot publish — stamped
    /// by the audio sink, which is never behind — does not have that
    /// failure mode.
    pub slot_end_hint: Option<fn() -> Option<i64>>,
}

/// This station's **transmit audio start** relative to the FT8 slot
/// boundary: 0.5 s.
///
/// **The name says key-up; WSJT-X keys up earlier than this, and the
/// difference is the whole point.** Ported from WSJT-X, which fixes
/// every number in the period (`widgets/mainwindow.cpp`,
/// `Modulator/Modulator.cpp`, `helper_functions.cpp`). For FT8, with
/// the period boundary at t = 0:
///
/// ```text
///  0.000 s  the boundary. `m_bTxTime = (t2p >= tx1) && (t2p < tx2)`
///           with `tx1 = 0` (mainwindow.cpp:4552, 4596), so the
///           transmit window opens *here*. The message is taken from
///           the auto-sequencer (`txMsg = ui->txN->text()`,
///           mainwindow.cpp:4657-4663) and PTT is asserted
///           (`transceiver_ptt(true)`, mainwindow.cpp:4711) in the
///           same pass. **A decode that lands after this moment
///           cannot change what is sent this period.**
///  +txDelay the rig confirms PTT; `ptt1Timer` then waits
///           `Configuration::txDelay()`, default **0.200 s**
///           (Configuration.cpp:1602), or a hard **20 ms** for FT4
///           (mainwindow.cpp:8252-8253). It fires `startTx2()`
///           (mainwindow.cpp:861-862), which starts the modulator.
///  0.500 s  audio begins. `Modulator::start` pads silence so the
///           waveform lands exactly here — `delay_ms = 500` for FT8,
///           300 for FT4, 1000 otherwise (Modulator.cpp:71-74), and
///           `m_silentFrames = (delay_ms - mstr) * frameRate / 1000`.
///           **A late start is truncated, not shifted**:
///           `m_ic = (mstr - delay_ms) * frameRate / 1000`
///           (Modulator.cpp:94-97) skips into the waveform to stay on
///           the grid. This matches the decoder's own reference,
///           `xdt = xdt - 0.5` (lib/ft8_decode.f90:210).
/// 13.140 s  audio ends: 79 x 1920 / 12000 = 12.64 s.
/// 13.640 s  `m_bTxTime` closes. `tx_duration("FT8") = 1.0 + 12.64`
///           (helper_functions.cpp:7) — a 1 s guard past audio end,
///           not extra transmission.
/// ```
///
/// So the 0.5 s this constant names is **the transceiver's, not the
/// decoder's**: WSJT-X spends it on PTT assert, rig turnaround and
/// modulator padding, having already committed the message at the
/// boundary. Two consequences for this pipeline:
///
/// 1. **A decode that finishes inside this 0.5 s is too late to
///    answer**, even though the deadline lets it run. What it is in
///    time for is the *next* period's choice, and for the screen.
///    A receiver that must reply in the following period wants its
///    stage 3 bounded at the boundary, not 0.5 s past it.
/// 2. Bounding at the boundary is nonetheless **not** what this
///    constant should become, because stage 3's late path needs the
///    full slot and the full slot does not exist until the boundary.
///    The 0.5 s is what makes a late path possible at all; it is
///    borrowed from the transmitter, and a build that transmits has
///    to pay it back — see `FT8_KEY_UP_GUARD_MS` on the CoreS3.
///
/// WSJT-X never stops a decode for a transmission: `jt9` is a separate
/// process and `MainWindow::decode()` only declines to *start* one
/// while the previous is still running. The bound here exists because
/// this board decodes on the same cores that will drive the
/// transmitter, which upstream does not have to consider.
pub const FT8_KEY_UP_AFTER_SLOT_END_US: i64 = 500_000;

/// When stage 3 stops claiming candidates. Both cores check it once per
/// candidate.
#[derive(Clone, Copy)]
pub struct Stage3Stop {
    /// Absolute `now_us()` deadline. `i64::MAX` when disabled.
    pub deadline_us: i64,
    /// The pipeline's `slot_q`, or null. While the slot sits in it
    /// unreceived, its `slotend_us` is peeked for the key-up bound.
    pub slot_q: QueueHandle_t,
    /// Also stop as soon as `slot_q` holds the slot — the early path's
    /// retries, which must hand the cores back to the deferred path.
    pub yield_on_slot: bool,
    /// [`DecodeConfig::key_up_guard_ms`] in µs; `0` disables the peek.
    pub key_up_guard_us: i64,
}

impl Stage3Stop {
    /// No deadline, no slot: run every candidate.
    pub const fn never() -> Self {
        Self {
            deadline_us: i64::MAX,
            slot_q: ptr::null_mut(),
            yield_on_slot: false,
            key_up_guard_us: 0,
        }
    }

    /// `true` once no further candidate may be claimed.
    fn reached(&self) -> bool {
        let now = now_us();
        if now >= self.deadline_us {
            return true;
        }
        if self.slot_q.is_null() {
            return false;
        }
        if self.key_up_guard_us > 0 {
            let mut raw: *mut Slot = ptr::null_mut();
            // SAFETY: non-blocking peek of a depth-1 queue of `*mut Slot`;
            // only this pipeline's decode task ever receives from it, and
            // it is blocked here, so the pointee cannot be freed under us.
            let got = unsafe {
                xQueuePeek(
                    self.slot_q,
                    (&mut raw as *mut *mut Slot) as *mut core::ffi::c_void,
                    0,
                )
            };
            if got == PD_PASS
                && !raw.is_null()
                && now >= key_up_deadline(unsafe { (*raw).slotend_us }, self.key_up_guard_us)
            {
                return true;
            }
        }
        self.yield_on_slot && unsafe { uxQueueMessagesWaiting(self.slot_q) } > 0
    }
}

/// The claim deadline [`DecodeConfig::key_up_guard_ms`] implies for a
/// slot that ended at `slotend_us`.
fn key_up_deadline(slotend_us: i64, guard_us: i64) -> i64 {
    slotend_us + FT8_KEY_UP_AFTER_SLOT_END_US - guard_us
}

/// `esp_timer_get_time()`, monotonic microseconds. The stage-3 deadline
/// is compared against this.
#[inline]
fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Phase-C audio-tail speculation runner. Receives a SpecBundle and
/// Slot pair from the streaming pipeline, partitions pass1 by
/// whether each candidate's Goertzel window fits inside the
/// SpecBundle audio prefix, runs pass-2 + stage-3 speculatively on
/// the `ready` half before SlotEnd, then completes any deferred
/// candidates on the full `slot.audio` after SlotEnd. The decode
/// pipelines in both `m5stack-s3-app` and `m5stack-core2-app` use
/// this single entry to keep the speculative logic in one place.
pub fn run_speculative_slot(
    spec_q: esp_idf_svc::sys::QueueHandle_t,
    slot_q: esp_idf_svc::sys::QueueHandle_t,
    cfg: &DecodeConfig,
) -> SpeculativeOut {
    use esp_idf_svc::sys::esp_timer_get_time;

    let spec = pipeline::recv_box::<SpecBundle>(spec_q);
    let t_post_recv = unsafe { esp_timer_get_time() };
    // Is there time to finish this slot before key-up? `slot_end_hint`
    // says where the slot ends; `slot_floor_ms` says how much of what
    // is left the un-deadlined work ahead (coarse + fine sync) needs.
    let slot_end_hint = cfg.slot_end_hint.and_then(|f| f());
    if let (true, Some(slotend)) = (cfg.slot_floor_ms > 0, slot_end_hint) {
        let key_up_at = key_up_deadline(slotend, cfg.key_up_guard_ms.max(0) * 1_000);
        if t_post_recv + cfg.slot_floor_ms * 1_000 > key_up_at {
            // Take the slot so stage1_inc's buffer is released on the
            // usual schedule, then hand the answer back empty.
            let slot = pipeline::recv_box::<Slot>(slot_q);
            let t_slot_recv = unsafe { esp_timer_get_time() };
            return SpeculativeOut {
                spec,
                slot,
                results: Vec::new(),
                n_pass1: 0,
                n_ready: 0,
                n_deferred: 0,
                n_early_refined: 0,
                n_in_time: 0,
                n_cut: 0,
                n_fallback: 0,
                bootstrap_dt_med: None,
                t_post_recv,
                t_coarse_done: t_post_recv,
                t_fine_done: t_post_recv,
                t_early_done: t_post_recv,
                t_slot_recv,
                t_done: unsafe { esp_timer_get_time() },
                pass2_us: (0, 0, 0),
                n_gate2_p1: 0,
                gate2_best_rank: u16::MAX,
                skipped: true,
                top3: [(0.0, 0.0, 0.0); 3],
                slot_end_hint_us: slot_end_hint,
            };
        }
    }
    pass2_timing_reset();
    let pass1: Vec<SyncCandidate> = if cfg.hard_sync_gate {
        coarse_sync_gated_spec(
            &spec.spec,
            cfg.freq_min,
            cfg.freq_max,
            cfg.sync_min,
            cfg.pass1_limit,
            Some(sync_lag_s()),
            Some(SpecFill {
                valid_rows: spec.valid_rows,
                gate_block2: FT8_BLOCK2_GATE,
            }),
        )
        .into_iter()
        .map(|c| c.0)
        .collect()
    } else {
        coarse_sync_split_with_allsum(
            &spec.spec,
            cfg.freq_min,
            cfg.freq_max,
            cfg.sync_min,
            cfg.pass1_limit,
            &spec.allsum_head,
            &spec.allsum_tail,
            FT8_BLOCK2_GATE.then_some(spec.valid_rows),
        )
    };
    let t_coarse_done = unsafe { esp_timer_get_time() };

    // Two-block candidates, counted and not acted on. See
    // `SpeculativeOut::n_gate2_p1`.
    let mut n_gate2_p1 = 0usize;
    let mut gate2_best_rank = u16::MAX;
    for (i, c) in pass1.iter().enumerate() {
        // **Positive lag only.** Block 2 runs late in the slot, so only
        // a positive lag pushes it past the filled rows; a negative
        // lag moves it earlier, where it is always covered. What a
        // negative lag truncates is block 0, which `sync_tail` does
        // not include, so the `max()` of the two scores is untouched.
        //
        // Counted as `|dt|` on 2026-09-20 and the number was roughly
        // double what it should have been — which then made the gated
        // arm of the host comparison look as though it had left
        // candidates behind that it had in fact removed.
        if c.dt_sec > crate::stage1_inc::SPEC_EMIT_MAX_LAG_S {
            n_gate2_p1 += 1;
            if gate2_best_rank == u16::MAX {
                gate2_best_rank = i as u16;
            }
        }
    }

    let n_pass1 = pass1.len();
    let mut top3 = [(0.0f32, 0.0f32, 0.0f32); 3];
    for (slot, c) in top3.iter_mut().zip(pass1.iter()) {
        *slot = (c.score, c.dt_sec, c.freq_hz);
    }
    // Taken from the coarse positions: it is documented against them,
    // and the fixture that pins it measures them.
    let bootstrap_dt_med = bootstrap_dt_median(&pass1, 5);
    // Fine sync runs on the audio prefix the speculative path already
    // has. Symbols past it contribute nothing, which costs the late
    // candidates some Stage A power but not their place: they still
    // partition into `deferred` on the refined DT below.
    // **Fine sync only with room for it.** Its ~292 ms is worth
    // spending when stage 3 still has time afterwards, and not when the
    // slot arrived late. The alternative tried before this was to run
    // it after key-up — which carried work into the next slot, and the
    // next slot is exactly where the audio pipeline needs the cores.
    // **Slack is time until the next slot needs the cores**, not time
    // until key-up.
    //
    // WSJT-X lets a decode run to completion because a PC has the CPU
    // to spare; this board does not. Stage 3 dispatches halves to
    // `dsp_worker` on APP_CPU, which is where `stage1_inc` lives, so a
    // decode that runs long competes with the next slot's spectrogram.
    //
    // The wall is the next SpecBundle. This slot's arrived ~1 s before
    // its slot ended and the next one comes 14 s after that end — one
    // slot period, 15 s, between bundles — so slack is measured from
    // the boundary the audio sink publishes, not from key-up.
    // Measured on a radio, 2026-09-19, with slack read against the
    // *next bundle* instead — i.e. with no real bound at all. Stage 3
    // ran 1.0-2.0 s past slot end; through that window the UAC reader
    // could not keep up and ~1 s of audio was dropped (`rx tick` at
    // 2 816-7 936 sa/s against 12 032 steady, byte totals contiguous);
    // the sink's boundary therefore slipped ~1 s behind UTC every
    // slot, re-anchored, and cut the slot short, so `stage1_inc` sent
    // 58-86 of 92 pairs and nothing decoded at all. The next bundle is
    // not the wall — the next slot's *audio* is, and it starts at the
    // boundary.
    // **A hint more than half a slot out is not a hint.** It is the
    // same argument the half-slot discriminator in
    // `decode_pipeline::slot_end_hint` makes, applied to the answer
    // rather than the input: a deliberately abnormal slot (cold
    // acquisition stretches one to 25 s) published a boundary that put
    // key-up 9.2 s in the past, and a deadline already expired claims
    // no candidates at all — `hint_err=-10775ms`, `cut=15`, `dec=0` on
    // a slot whose audio was fine (2026-09-19). A hint that far out is
    // wrong about which slot it names, so the budget cap stands alone
    // until it is sane again. A second or two in the past is a
    // different thing — that is a backlogged pipeline, it is true, and
    // it is meant to bite.
    const HINT_SANE_US: i64 = 7_500_000;
    let guard_us_cfg = cfg.key_up_guard_ms.max(0) * 1_000;
    // The boundary itself, kept separate from the key-up deadline
    // derived from it. They are 0.5 s apart and answer different
    // questions: the deadline is when stage 3 stops claiming, the
    // boundary is when a reply has to have been chosen. Same sanity
    // filter, applied through the same derived value so the two cannot
    // disagree about which slots have a usable hint.
    let boundary_us = cfg.slot_end_hint.and_then(|f| f()).filter(|slotend| {
        (key_up_deadline(*slotend, guard_us_cfg) - t_post_recv).abs() <= HINT_SANE_US
    });
    let key_up_us = boundary_us.map(|slotend| key_up_deadline(slotend, guard_us_cfg));
    let slack_ms = key_up_us
        .map(|d| (d - t_post_recv) / 1_000)
        .unwrap_or(i64::MAX);
    let fine_ok = cfg.fine_sync && slack_ms >= cfg.fine_sync_min_slack_ms;
    if cfg.fine_sync && !fine_ok {
        log::info!("stage1: fine sync skipped — {slack_ms} ms of slack");
    }
    // `coarse` is empty unless fine sync moved the candidates, since
    // it exists only to map a failure back to where it started.
    let (pass1, coarse) = if fine_ok {
        let refined = fine_sync_split(spec.audio_prefix(), &pass1);
        (refined, pass1)
    } else {
        (pass1, Vec::new())
    };
    let t_fine_done = unsafe { esp_timer_get_time() };
    // Absolute stage-3 deadline. From `t_post_recv`, not from slot end —
    // slot end is not known until `recv_box::<Slot>` below, and the
    // early speculative path is where a slow slot overruns. When the
    // pipeline is backlogged `t_post_recv` is itself already past slot
    // end (the SpecBundle send blocked on a full `spec_q`), so the
    // effective budget shrinks exactly when the pipeline is behind —
    // which is what breaks the compounding-backlog loop (#357).
    let mut deadline_us: i64 = if cfg.budget_ms > 0 {
        t_post_recv + cfg.budget_ms * 1_000
    } else {
        i64::MAX
    };
    // **And never past this station's key-up.** `budget_ms` is the
    // runaway cap; the operating bound is the boundary the audio sink
    // published, which is known here — before the Slot arrives, which
    // is where a slow slot overruns and where the `slot_q` peek in
    // `Stage3Stop` cannot see it yet. `hint_err` measures the two
    // against each other every slot: ±27 ms on a radio, 2026-09-19.
    if let Some(k) = key_up_us {
        deadline_us = deadline_us.min(k);
    }
    let key_up_guard_us = cfg.key_up_guard_ms.max(0) * 1_000;
    // Before the slot is received its end is only known by peeking
    // `slot_q`; the stop condition does that per candidate.
    let early_stop = |yield_on_slot: bool| Stage3Stop {
        deadline_us,
        slot_q,
        yield_on_slot,
        key_up_guard_us,
    };
    let mut n_cut = 0usize;
    let mut n_fallback = 0usize;
    let snap_fill = spec.audio_len();
    // Partition by audio-window fit only.
    //
    // An earlier revision (round 19) added an `i < cfg.max_cand`
    // guard so high-rank pass1 cands with dt > +0.36 s would land
    // in `deferred` instead of being outcompeted by lower-rank
    // cands in the early path. On qso3-busy the guard moved
    // 12-19 candidates per slot into the late path; the resulting
    // post-SlotEnd wallclock grew (190..400 ms across slots) and
    // we couldn't observe the theoretical recall benefit that
    // motivated the change. Reverted — pass-2's own
    // sync_quality_block0 heap top-N inside both halves filters
    // the strongest cands per side, and the budget cap on
    // `max_cand_late` keeps the union ≤ max_cand. High-score /
    // high-dt cands that don't fit the prefix still naturally
    // land in `deferred` and compete for a slot there.
    // The partition consumes pass 1; the retry needs the refined
    // positions to match failures back to `coarse` by index.
    let refined: Vec<SyncCandidate> = if cfg.fine_sync {
        pass1.clone()
    } else {
        Vec::new()
    };
    let is_ready = |c: &SyncCandidate| SpecBundle::audio_end_for_dt(c.dt_sec) <= snap_fill;
    // See `DecodeConfig::share_cand_budget`. `pass1` is in coarse-score
    // order (`coarse_sync_split_with_allsum` sorts before truncating),
    // so its first `max_cand` entries are the set to divide.
    let (early_budget, late_budget) = if cfg.share_cand_budget {
        let early = pass1
            .iter()
            .take(cfg.max_cand)
            .filter(|c| is_ready(c))
            .count();
        (early, cfg.max_cand - early)
    } else {
        (cfg.max_cand, 0)
    };
    let (ready, deferred): (Vec<SyncCandidate>, Vec<SyncCandidate>) =
        pass1.into_iter().partition(&is_ready);
    let n_ready = ready.len();
    let n_deferred = deferred.len();
    // the gate's order is the ranking: its first candidates go to pass 2, not the best of them by block-0
    // quality (`refine_candidates_into` re-ranks what it is given). The counts above stay what coarse found.
    let (mut ready, mut deferred) = (ready, deferred);
    if cfg.hard_sync_gate {
        ready.truncate(early_budget);
    }

    let mut n_early_refined = 0usize;
    // Decodes completed before the slot boundary — the ones in time to
    // be answered in the next period. Exact for the early first-attempt
    // batch below, which is the only one that straddles the boundary in
    // steady state; every later batch starts after it, and is counted
    // in time only when it also *finishes* before it.
    let mut n_in_time = 0usize;
    // Early-path retries that the slot's arrival stopped; they join the
    // late path's retries on the full slot below.
    let mut retry_leftover: Vec<RefinedCandidate> = Vec::new();
    let mut results = if !ready.is_empty() && early_budget > 0 {
        let partial_audio: &[i16] = spec.audio_prefix();
        let p2 = pass2_split(partial_audio, ready, early_budget);
        n_early_refined = p2.len();
        // **Run the batch in two halves split at the slot boundary, so
        // the count of decodes that were in time to be answered comes
        // out exactly.**
        //
        // The work is identical either way: `stage3_split` claims one
        // candidate at a time and hands back what it did not claim, so
        // stopping at the boundary and resuming on `unclaimed` covers
        // the same candidates in the same order for the same cost. What
        // it adds is the split itself, and on a radio that split is the
        // number that matters — measured 2026-09-19, 29 % of this batch
        // runs past the boundary, which is past the moment WSJT-X
        // commits the reply (`FT8_KEY_UP_AFTER_SLOT_END_US`). `dec`
        // alone cannot see it.
        //
        // Skipped when the boundary is already behind us or past the
        // deadline anyway; then the batch runs as one and everything it
        // produces is late by definition.
        let first = match boundary_us.filter(|b| *b > now_us() && *b < deadline_us) {
            Some(b) => {
                let in_time = stage3_split(
                    partial_audio,
                    p2,
                    cfg.depth,
                    cfg.q_thresh,
                    cfg.bp_max_iter,
                    Stage3Stop {
                        deadline_us: b,
                        ..early_stop(false)
                    },
                );
                n_in_time = in_time.results.len();
                let mut rest = stage3_split(
                    partial_audio,
                    in_time.unclaimed,
                    cfg.depth,
                    cfg.q_thresh,
                    cfg.bp_max_iter,
                    early_stop(false),
                );
                let mut merged = in_time.results;
                merged.append(&mut rest.results);
                let mut failed = in_time.failed;
                failed.append(&mut rest.failed);
                Stage3Split {
                    results: merged,
                    failed,
                    unclaimed: rest.unclaimed,
                }
            }
            None => stage3_split(
                partial_audio,
                p2,
                cfg.depth,
                cfg.q_thresh,
                cfg.bp_max_iter,
                early_stop(false),
            ),
        };
        n_cut += first.unclaimed.len();
        let mut r = first.results;
        // **Retries run only on time no first attempt can use.** Before
        // the slot arrives the deferred candidates cannot start — their
        // audio does not exist yet — so the tail window is free, and the
        // retries stop claiming the moment `slot_q` holds the slot.
        // Retries are the lowest-value work here (on the host mirror,
        // 0.26-0.46 decodes a slot recovered from 7-13 retries), and
        // they used to run straight through to the deadline, which on a
        // slot with deferred candidates would have skipped those
        // candidates' first attempts altogether.
        let retry = coarse_retry_candidates(&first.failed, &refined, &coarse);
        if !retry.is_empty() && !early_stop(true).reached() {
            let n = retry.len();
            let p2 = pass2_split(partial_audio, retry, n);
            let fb = stage3_split(
                partial_audio,
                p2,
                cfg.depth,
                cfg.q_thresh,
                cfg.bp_max_iter,
                early_stop(true),
            );
            n_fallback += n - fb.unclaimed.len();
            r.extend(fb.results);
            retry_leftover = fb.unclaimed;
        } else {
            n_cut += retry.len();
        }
        r
    } else {
        Vec::new()
    };
    let t_early_done = unsafe { esp_timer_get_time() };

    let slot = pipeline::recv_box::<Slot>(slot_q);
    let t_slot_recv = unsafe { esp_timer_get_time() };
    let slot_audio: &[i16] = slot.audio();
    if key_up_guard_us > 0 {
        deadline_us = deadline_us.min(key_up_deadline(slot.slotend_us, key_up_guard_us));
    }
    let late_stop = Stage3Stop {
        deadline_us,
        slot_q: ptr::null_mut(),
        yield_on_slot: false,
        key_up_guard_us: 0,
    };

    let mut late_retry: Vec<SyncCandidate> = Vec::new();
    if !deferred.is_empty() {
        // Budget cap (Gemini PR #123 round 8/9): stage-3's wallclock
        // is roughly linear in the number of refined candidates
        // pass-2 hands it. Sum-cap by the *refined* count (not the
        // decoded count) so `n_early_refined + n_late_refined ≤
        // max_cand` holds regardless of whether the early path's
        // candidates went on to decode. Trade-off: when the early
        // half already fills `max_cand` slots (typical qso3-busy
        // case: ready=25-27, pass-2 keeps top 15) the deferred
        // path gets zero budget and any dt > +0.36 s real signals
        // are dropped. Acceptable on FT8 — operational dt clusters
        // tightly under NTP sync; `time_sync`'s slot phase
        // auto-anchor keeps dt within ±0.5 s in steady state.
        let max_cand_late = if cfg.share_cand_budget {
            late_budget
        } else {
            cfg.max_cand.saturating_sub(n_early_refined)
        };
        if cfg.hard_sync_gate {
            deferred.truncate(max_cand_late);
        }
        if max_cand_late > 0 && now_us() < deadline_us {
            let p2 = pass2_split(slot_audio, deferred, max_cand_late);
            let late = stage3_split(
                slot_audio,
                p2,
                cfg.depth,
                cfg.q_thresh,
                cfg.bp_max_iter,
                late_stop,
            );
            n_cut += late.unclaimed.len();
            // In time only if the whole batch landed before the
            // boundary, which needs the early path to have left room.
            if boundary_us.is_some_and(|b| now_us() <= b) {
                n_in_time += late.results.len();
            }
            results.extend(late.results);
            late_retry = coarse_retry_candidates(&late.failed, &refined, &coarse);
        } else if max_cand_late > 0 {
            // Out of budget before the deferred path even started — the
            // speculative half used all of it. Every candidate that
            // would have been refined here is a cut.
            n_cut += n_deferred.min(max_cand_late);
        }
    }

    // Every first attempt has now had its turn. What is left of the
    // early retries, then the late path's, share the rest of the budget
    // on the whole slot. A leftover early retry keeps its pass-2
    // spectrum: block 0 is the same samples in the prefix and the slot,
    // and stage 3 refills everything else from the audio it is given.
    let n_late_retry = late_retry.len();
    let mut tail_retries = retry_leftover;
    if n_late_retry > 0 && now_us() < deadline_us {
        tail_retries.extend(pass2_split(slot_audio, late_retry, n_late_retry));
    } else {
        n_cut += n_late_retry;
    }
    if !tail_retries.is_empty() {
        let n = tail_retries.len();
        let fb = stage3_split(
            slot_audio,
            tail_retries,
            cfg.depth,
            cfg.q_thresh,
            cfg.bp_max_iter,
            late_stop,
        );
        n_fallback += n - fb.unclaimed.len();
        n_cut += fb.unclaimed.len();
        if boundary_us.is_some_and(|b| now_us() <= b) {
            n_in_time += fb.results.len();
        }
        results.extend(fb.results);
    }
    // One row per message. Duplicates were possible before — two
    // coarse cells on one strong carrier both decoding — and fine sync
    // makes them common, since it pulls adjacent 3.125 Hz cells onto the
    // same carrier: on the host mirror, 31 duplicate rows over 201 phases
    // of `qso2` against 7 without it. The UI already dedups by text; the
    // count in the slot log, the grid-lock policy that reads it, and the
    // QSO state machine did not. The first row is kept, which is the
    // higher pass-2 rank within whichever path produced it first.
    dedup_by_message(&mut results);
    // Dedup can drop a row the in-time count already claimed, so cap
    // rather than let `n_in_time > results.len()` reach a log line.
    // Which of a duplicate pair survives is the first produced, so the
    // cap loses at most the difference and never the wrong side.
    n_in_time = n_in_time.min(results.len());
    let t_done = unsafe { esp_timer_get_time() };

    SpeculativeOut {
        spec,
        slot,
        results,
        n_pass1,
        n_ready,
        n_deferred,
        n_early_refined,
        pass2_us: pass2_timing(),
        n_gate2_p1,
        gate2_best_rank,
        n_in_time,
        n_cut,
        n_fallback,
        bootstrap_dt_med,
        t_post_recv,
        t_coarse_done,
        t_fine_done,
        t_early_done,
        t_slot_recv,
        t_done,
        skipped: false,
        top3,
        slot_end_hint_us: slot_end_hint,
    }
}


/// Keep the first [`DecodeResult`] per 77-bit message.
fn dedup_by_message(results: &mut Vec<DecodeResult>) {
    let mut i = 0;
    while i < results.len() {
        if results[..i]
            .iter()
            .any(|r| r.message77() == results[i].message77())
        {
            results.remove(i);
        } else {
            i += 1;
        }
    }
}

/// The coarse positions to retry for the candidates whose fine-synced
/// position failed stage 3 ([`DecodeConfig::fine_sync`]).
///
/// `refined[i]` is `coarse[i]` after fine sync, so a failure is matched
/// back by its exact refined position. Two coarse cells that fine sync
/// moved onto the same position share one retry — the first cell's —
/// which is what the host mirror this was measured on does too. A
/// candidate fine sync did not move has nothing different to retry.
fn coarse_retry_candidates(
    failed: &[SyncCandidate],
    refined: &[SyncCandidate],
    coarse: &[SyncCandidate],
) -> Vec<SyncCandidate> {
    let same = |a: &SyncCandidate, b: &SyncCandidate| {
        a.freq_hz.to_bits() == b.freq_hz.to_bits() && a.dt_sec.to_bits() == b.dt_sec.to_bits()
    };
    let mut retry: Vec<SyncCandidate> = Vec::new();
    if coarse.is_empty() {
        return retry;
    }
    for f in failed {
        let Some(i) = refined.iter().position(|r| same(r, f)) else {
            continue;
        };
        if !same(&coarse[i], f) {
            retry.push(coarse[i].clone());
        }
    }
    retry
}

/// [`fine_sync_12k`] over pass 1, halves on main and worker. Output is
/// in input order.
pub fn fine_sync_split(audio: &[i16], pass1: &[SyncCandidate]) -> Vec<SyncCandidate> {
    let mid = pass1.len() / 2;
    let tail: Vec<SyncCandidate> = pass1[mid..].to_vec();
    let job = Box::new(Job::FineSync {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        cands: tail,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut local = fine_sync_12k(audio, &pass1[..mid]);

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<SyncCandidate>>(FINE_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };
    local.extend(worker);
    local
}

const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;

enum Job {
    Pass2 {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<SyncCandidate>,
        max_cand: usize,
    },
    /// Work-stealing stage 3: both main and worker pull individual
    /// candidates from a shared `Vec<Option<RefinedCandidate>>` via
    /// `next_idx.fetch_add(1)`. Per-cand BP wall-clock varies a lot
    /// (failed cands run all 4 LLR variants); a static head/tail split
    /// stalls one core waiting for the other. Dynamic dispatch keeps
    /// both cores busy until the queue drains.
    Stage3WorkSteal {
        audio: *const i16,
        audio_len: usize,
        depth: DecodeDepth,
        q_thresh: u32,
        bp_max_iter: u32,
        slots_ptr: *mut Option<RefinedCandidate>,
        slots_len: usize,
        next_idx: *const AtomicUsize,
        /// When the loop stops claiming candidates.
        stop: Stage3Stop,
    },
    FineSync {
        audio: *const i16,
        audio_len: usize,
        cands: Vec<SyncCandidate>,
    },
    CoarseSyncWithAllsum {
        spec: *const Spectrogram,
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
        allsum_ptr: *const f32,
        allsum_len: usize,
        /// `None` unless [`FT8_BLOCK2_GATE`]; see
        /// `pipeline::SpecBundle::valid_rows`. The worker half needs it
        /// as much as the main half — both score block 2 against the
        /// same partly-filled spectrogram.
        valid_rows: Option<usize>,
    },
}

/// SAFETY: Job's raw pointers are produced by dispatch fns that
/// block-wait on the result before returning, so the referenced data
/// outlives the worker's access. Vec ownership transfers via Box.
unsafe impl Send for Job {}

/// Queue handle holder — initialised once in `init()`, then read-only.
struct QueueCell(UnsafeCell<QueueHandle_t>);
unsafe impl Sync for QueueCell {}
impl QueueCell {
    const fn new() -> Self {
        Self(UnsafeCell::new(ptr::null_mut()))
    }
    fn get(&self) -> QueueHandle_t {
        unsafe { *self.0.get() }
    }
    /// SAFETY: only call from `init()` before any other access.
    unsafe fn set(&self, q: QueueHandle_t) {
        unsafe { *self.0.get() = q };
    }
}

static JOB_Q: QueueCell = QueueCell::new();
static PASS2_RESULT_Q: QueueCell = QueueCell::new();
static STAGE3_RESULT_Q: QueueCell = QueueCell::new();
static COARSE_RESULT_Q: QueueCell = QueueCell::new();
static FINE_RESULT_Q: QueueCell = QueueCell::new();

#[inline]
unsafe fn queue_create(item_size: usize) -> QueueHandle_t {
    let q = unsafe {
        xQueueGenericCreate(
            1, // depth
            item_size as u32,
            QUEUE_TYPE_BASE,
        )
    };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

/// Send a single boxed pointer through a depth-1 queue.
/// SAFETY: caller transfers ownership of `ptr` to the receiver.
#[inline]
unsafe fn queue_send_ptr<T>(q: QueueHandle_t, ptr: *mut T) {
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&ptr as *const *mut T) as *const core::ffi::c_void,
            PORT_MAX_DELAY,
            QUEUE_SEND_TO_BACK,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueGenericSend failed: {r}");
}

/// Receive a single pointer from a depth-1 queue.
/// SAFETY: receiver takes ownership of returned pointer.
#[inline]
unsafe fn queue_recv_ptr<T>(q: QueueHandle_t) -> *mut T {
    let mut out: *mut T = ptr::null_mut();
    let r = unsafe {
        xQueueReceive(
            q,
            (&mut out as *mut *mut T) as *mut core::ffi::c_void,
            PORT_MAX_DELAY,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueReceive failed: {r}");
    out
}

extern "C" fn worker_main(_arg: *mut core::ffi::c_void) {
    log::info!("dsp_worker: started on core {}", current_core());
    loop {
        let job_ptr = unsafe { queue_recv_ptr::<Job>(JOB_Q.get()) };
        let job = unsafe { *Box::from_raw(job_ptr) };
        match job {
            Job::Pass2 {
                audio,
                audio_len,
                cands,
                max_cand,
            } => {
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                let t0 = now_us();
                let result = refine_candidates_into(audio_slice, cands, max_cand);
                PASS2_WORKER_US.fetch_add((now_us() - t0) as i32, Ordering::Relaxed);
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(PASS2_RESULT_Q.get(), raw) };
            }
            Job::Stage3WorkSteal {
                audio,
                audio_len,
                depth,
                q_thresh,
                bp_max_iter,
                slots_ptr,
                slots_len,
                next_idx,
                stop,
            } => {
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                #[allow(static_mut_refs)]
                let result = unsafe {
                    drain_stage3_queue(
                        audio_slice,
                        slots_ptr,
                        slots_len,
                        &*next_idx,
                        depth,
                        q_thresh,
                        bp_max_iter,
                        stop,
                        cs_scratch_worker(),
                    )
                };
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(STAGE3_RESULT_Q.get(), raw) };
            }
            Job::FineSync {
                audio,
                audio_len,
                cands,
            } => {
                let audio_slice = unsafe { core::slice::from_raw_parts(audio, audio_len) };
                let result = fine_sync_12k(audio_slice, &cands);
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(FINE_RESULT_Q.get(), raw) };
            }
            Job::CoarseSyncWithAllsum {
                spec,
                freq_min,
                freq_max,
                sync_min,
                max_cand,
                allsum_ptr,
                allsum_len,
                valid_rows,
            } => {
                let spec_ref = unsafe { &*spec };
                let allsum = unsafe { core::slice::from_raw_parts(allsum_ptr, allsum_len) };
                let result = match valid_rows {
                    Some(v) => mfsk_core::ft8::decode_block::coarse_sync_with_allsum_lag_and_rows(
                        spec_ref,
                        freq_min,
                        freq_max,
                        sync_min,
                        max_cand,
                        allsum,
                        sync_lag_s(),
                        v,
                    ),
                    None => mfsk_core::ft8::decode_block::coarse_sync_with_allsum_and_lag(
                        spec_ref,
                        freq_min,
                        freq_max,
                        sync_min,
                        max_cand,
                        allsum,
                        sync_lag_s(),
                    ),
                };
                let raw = Box::into_raw(Box::new(result));
                unsafe { queue_send_ptr(COARSE_RESULT_Q.get(), raw) };
            }
        }
    }
}

fn current_core() -> i32 {
    unsafe { xTaskGetCoreID(ptr::null_mut()) }
}

/// Spawn the persistent worker task on APP_CPU and create dispatch
/// queues. Call once at startup, after `link_patches()`,
/// `EspLogger::initialize_default()`, and `esp_dsp_fft::prewarm()`.
pub fn init() {
    unsafe {
        JOB_Q.set(queue_create(core::mem::size_of::<*mut Job>()));
        PASS2_RESULT_Q.set(queue_create(
            core::mem::size_of::<*mut Vec<RefinedCandidate>>(),
        ));
        STAGE3_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Stage3Out>()));
        COARSE_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<SyncCandidate>>()));
        FINE_RESULT_Q.set(queue_create(core::mem::size_of::<*mut Vec<SyncCandidate>>()));

        let r = xTaskCreatePinnedToCore(
            Some(worker_main),
            c"dsp_worker".as_ptr(),
            16384,
            ptr::null_mut(),
            5,
            ptr::null_mut(),
            1, // APP_CPU
        );
        assert_eq!(
            r, PD_PASS,
            "xTaskCreatePinnedToCore(dsp_worker) failed: {r}"
        );
    }
    log::info!(
        "dsp_worker: spawned on APP_CPU; main is core {}",
        current_core()
    );
}

/// Run Stage 2 sequentially per frequency half on main. No worker
/// dispatch (Phase E2 race fallback path; rarely hit).
pub fn coarse_sync_split(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let mid = 0.5 * (freq_min + freq_max);
    let mut head =
        coarse_sync_with_lag(spec, freq_min, mid, sync_min, max_cand, sync_lag_s());
    let tail = coarse_sync_with_lag(spec, mid, freq_max, sync_min, max_cand, sync_lag_s());
    head.extend(tail);
    head.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    head.truncate(max_cand);
    head
}

/// Phase E2: parallel coarse_sync with pre-built per-half allsums.
/// Worker computes the tail half on APP_CPU; main computes the head
/// half locally in parallel.
pub fn coarse_sync_split_with_allsum(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    allsum_head: &[f32],
    allsum_tail: &[f32],
    valid_rows: Option<usize>,
) -> Vec<SyncCandidate> {
    use mfsk_core::ft8::decode_block::{
        coarse_sync_with_allsum_and_lag, coarse_sync_with_allsum_lag_and_rows,
    };
    let mid = 0.5 * (freq_min + freq_max);

    let job = Box::new(Job::CoarseSyncWithAllsum {
        spec: spec as *const _,
        freq_min: mid,
        freq_max,
        sync_min,
        max_cand,
        allsum_ptr: allsum_tail.as_ptr(),
        allsum_len: allsum_tail.len(),
        valid_rows,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let mut local = match valid_rows {
        Some(v) => coarse_sync_with_allsum_lag_and_rows(
            spec,
            freq_min,
            mid,
            sync_min,
            max_cand,
            allsum_head,
            sync_lag_s(),
            v,
        ),
        None => coarse_sync_with_allsum_and_lag(
            spec,
            freq_min,
            mid,
            sync_min,
            max_cand,
            allsum_head,
            sync_lag_s(),
        ),
    };

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<SyncCandidate>>(COARSE_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    local.truncate(max_cand);
    local
}

/// Pass 2 split across main + worker; merged top-`max_cand` returned.
/// Per-slot totals for [`pass2_split`], in microseconds: how long the
/// main core spent on its own half, how long it then sat blocked on
/// the worker, and how long the worker's half actually took.
///
/// **`pass2_split` is the one stage that is still a fixed head/tail
/// split.** `stage3_split` moved to work stealing because per-candidate
/// BP wall-clock varies a lot; pass 2 did not, so it hands the worker
/// `pass1[mid..]` and blocks. Whichever core draws the cheaper half
/// idles for the difference, and until these counters existed nobody
/// had measured it in either direction — `main_wait` sees only the
/// case where the worker is slower.
// `AtomicI32`, not `I64`: Xtensa has no 64-bit atomics, and a stage
// that ran for 35 minutes would have bigger problems than a wrapped
// counter.
static PASS2_MAIN_US: AtomicI32 = AtomicI32::new(0);
static PASS2_WAIT_US: AtomicI32 = AtomicI32::new(0);
static PASS2_WORKER_US: AtomicI32 = AtomicI32::new(0);

/// Zero the [`PASS2_MAIN_US`] family; `run_speculative_slot` calls this
/// once a slot so the totals cover that slot's two pass-2 batches.
fn pass2_timing_reset() {
    PASS2_MAIN_US.store(0, Ordering::Relaxed);
    PASS2_WAIT_US.store(0, Ordering::Relaxed);
    PASS2_WORKER_US.store(0, Ordering::Relaxed);
}

/// `(main, wait, worker)` microseconds since the last reset.
fn pass2_timing() -> (i64, i64, i64) {
    (
        PASS2_MAIN_US.load(Ordering::Relaxed) as i64,
        PASS2_WAIT_US.load(Ordering::Relaxed) as i64,
        PASS2_WORKER_US.load(Ordering::Relaxed) as i64,
    )
}

pub fn pass2_split(
    audio: &[i16],
    pass1: Vec<SyncCandidate>,
    max_cand: usize,
) -> Vec<RefinedCandidate> {
    let mid = pass1.len() / 2;
    let mut head = pass1;
    let tail = head.split_off(mid);

    let job = Box::new(Job::Pass2 {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        cands: tail,
        max_cand,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    let t0 = now_us();
    let mut local = refine_candidates_into(audio, head, max_cand);
    let t_local = now_us();

    let worker_ptr = unsafe { queue_recv_ptr::<Vec<RefinedCandidate>>(PASS2_RESULT_Q.get()) };
    let t_join = now_us();
    PASS2_MAIN_US.fetch_add((t_local - t0) as i32, Ordering::Relaxed);
    PASS2_WAIT_US.fetch_add((t_join - t_local) as i32, Ordering::Relaxed);
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    local.extend(worker);
    local.sort_by(|a, b| b.2.cmp(&a.2));
    local.truncate(max_cand);
    local
}

/// Stage 3 across main + worker with **work-stealing** dispatch.
///
/// Both cores share the same `Vec<Option<RefinedCandidate>>` and pull
/// candidates by `AtomicUsize::fetch_add(1)`, so a slow / failed cand
/// on one core doesn't stall the other. Compared to the old
/// fixed head/tail split, this absorbs the per-cand BP wall-clock
/// variance (qso3 ~7 of 15 cands fail and run all 4 LLR variants).
///
/// Both cores stop claiming once `stop` says so ([`Stage3Stop`]). A
/// candidate already being decoded finishes either way.
pub fn stage3_split(
    audio: &[i16],
    pass2: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    stop: Stage3Stop,
) -> Stage3Split {
    let mut slots: Vec<Option<RefinedCandidate>> = pass2.into_iter().map(Some).collect();
    let next_idx = AtomicUsize::new(0);

    let slots_ptr = slots.as_mut_ptr();
    let slots_len = slots.len();
    let next_idx_ptr: *const AtomicUsize = &next_idx;

    // Dispatch worker — it will drain the same queue from APP_CPU.
    let job = Box::new(Job::Stage3WorkSteal {
        audio: audio.as_ptr(),
        audio_len: audio.len(),
        depth,
        q_thresh,
        bp_max_iter,
        slots_ptr,
        slots_len,
        next_idx: next_idx_ptr,
        stop,
    });
    unsafe { queue_send_ptr(JOB_Q.get(), Box::into_raw(job)) };

    // Main drains in parallel.
    #[allow(static_mut_refs)]
    let mut local = unsafe {
        drain_stage3_queue(
            audio,
            slots_ptr,
            slots_len,
            &next_idx,
            depth,
            q_thresh,
            bp_max_iter,
            stop,
            cs_scratch_main(),
        )
    };

    let worker_ptr = unsafe { queue_recv_ptr::<Stage3Out>(STAGE3_RESULT_Q.get()) };
    let worker = unsafe { *Box::from_raw(worker_ptr) };

    // Both cores have joined; `next_idx` is final. Each claim past
    // `slots_len` is a core's last `fetch_add` before it saw the end,
    // so clamp. Claims are a contiguous prefix, so whatever was never
    // claimed is the tail — still `Some`, handed back to the caller.
    let claimed = next_idx.load(Ordering::Acquire).min(slots_len);
    let unclaimed: Vec<RefinedCandidate> = slots.drain(claimed..).flatten().collect();
    drop(slots);

    local.results.extend(worker.results);
    local.failed.extend(worker.failed);
    Stage3Split {
        results: local.results,
        failed: local.failed,
        unclaimed,
    }
}

/// What [`stage3_split`] did with its candidates.
pub struct Stage3Split {
    pub results: Vec<DecodeResult>,
    /// Tried and decoded nothing, by position — for
    /// [`DecodeConfig::fine_sync`]'s coarse retry.
    pub failed: Vec<SyncCandidate>,
    /// Never claimed: the deadline or `yield_to` stopped both cores
    /// first. On the first-attempt paths these are the deadline's cuts.
    pub unclaimed: Vec<RefinedCandidate>,
}

/// One core's share of a stage-3 drain.
struct Stage3Out {
    results: Vec<DecodeResult>,
    /// Tried and decoded nothing.
    failed: Vec<SyncCandidate>,
}

/// Pop candidates from a shared atomic-indexed slot array and process
/// them one at a time, accumulating successful decodes.
///
/// SAFETY: `slots_ptr` must point to `slots_len` valid
/// `Option<RefinedCandidate>` cells, and `next_idx` claims an exclusive
/// index per `fetch_add` so the same slot is never read by two callers.
#[allow(clippy::too_many_arguments)]
unsafe fn drain_stage3_queue(
    audio: &[i16],
    slots_ptr: *mut Option<RefinedCandidate>,
    slots_len: usize,
    next_idx: &AtomicUsize,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    stop: Stage3Stop,
    cs_scratch: &mut [[mfsk_core::engine::scalar::Cmplx<f32>; 8]; 79],
) -> Stage3Out {
    let mut out = Stage3Out {
        results: Vec::new(),
        failed: Vec::new(),
    };
    loop {
        // Checked before the claim so `next_idx` reflects exactly what
        // was processed — `stage3_split` derives the cut count from it.
        // The check is once per candidate (BP/OSD is 10-100 ms each), so
        // a timer read and a non-blocking queue peek here are free.
        if stop.reached() {
            break;
        }
        let i = next_idx.fetch_add(1, Ordering::AcqRel);
        if i >= slots_len {
            break;
        }
        // SAFETY: the atomic fetch_add gives this caller exclusive
        // ownership of slot `i` for the duration of this iteration.
        let cand = unsafe { (*slots_ptr.add(i)).take() };
        let Some(cand) = cand else { continue };
        let pos = cand.0.clone();
        let mut single = vec![cand];
        let mut results = process_candidates_into_with_cs_scratch_tuned(
            audio,
            core::mem::take(&mut single),
            depth,
            q_thresh,
            bp_max_iter,
            cs_scratch,
        );
        if results.is_empty() {
            out.failed.push(pos);
        } else {
            out.results.append(&mut results);
        }
    }
    out
}
