//! WAV-fed decode pipeline for CoreS3 (Phase 0-Core).
//!
//! Structurally identical to `m5stack-core2-app/src/decode_pipeline.rs`.
//! All heavy lifting is in `embedded_shared` and `mfsk_app_shared` which
//! are board-agnostic. Board-specific touch points are `crate::log_free_internal`
//! and the `QSO_WAVS` slice (same qso3_busy.wav reference as Core2).
//! Phase 1-Core replaces the wav_sim source with a UAC host capture.

extern crate alloc;

use core::fmt::Write as _;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{DEFAULT_Q_THRESH, NFFT_SPEC};

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};
use esp_idf_svc::sys::QueueHandle_t;

use mfsk_app_shared::qso::{self, QsoManager, QsoState};
use mfsk_app_shared::ui::state::{SlotDecode, UI};

/// Operator identity, from `cfg.toml`'s `[station]` section.
///
/// These were literals until 2026-08-23 — this repository shipped one
/// operator's callsign compiled into the source, so anyone else's build
/// identified as them. `m5stack-s3-app` has taken them from `cfg.toml`
/// since Phase 1.7; this crate was never brought across.
///
/// Empty is a valid value and leaves the QSO FSM idle, which is the
/// right behaviour for a receiver with no operator configured.
pub(crate) const MY_CALL: &str = env!("MY_CALL");
pub(crate) const MY_GRID: &str = env!("MY_GRID");

/// The baked FT8 slot. `pub` so the `MFSK_CORES3_SIM` harness can feed
/// it through `Ft8ChunkSink` (the real UAC sink) instead of the direct
/// `wav_sim` path.
pub static QSO_WAVS: &[&[u8]] = &[include_bytes!("../../assets/qso3_busy.wav")];

/// Pass-1 candidate cap and refined-candidate cap.
///
/// The embedded FT8 decode is deliberately lean — single-pass BP,
/// `LlrEffort::Minimal`, **no OSD, no SIC, no AP** (`stage3_split` →
/// `process_candidates_with_ap` runs one pass and never touches the raw
/// audio a subtract would need). It decodes 7 on qso3_busy where host
/// JTDX gets ~18; that gap is the cost of the leanness, and it is the
/// right trade for battery-budgeted field operation where a bounded,
/// phantom-free slot matters more than the last few dB (the phantom
/// bugs this suite has shipped were all in the subtraction paths this
/// config does not use). Not a bug to chase.
///
/// Compile-time knobs, kept for #357 investigation only —
/// `MFSK_FT8_MAX_CAND` / `MFSK_FT8_PASS1_LIMIT`. Defaults are what
/// ships.
const PASS1_LIMIT: usize = match option_env!("MFSK_FT8_PASS1_LIMIT") {
    Some(s) => parse_u32(s) as usize,
    None => 30,
};
const MAX_CAND: usize = match option_env!("MFSK_FT8_MAX_CAND") {
    Some(s) => parse_u32(s) as usize,
    None => 15,
};

/// Wall-clock cap on stage 3, **measured from `t_post_recv`** — the
/// moment the decode task receives the SpecBundle, 14.0 s into the
/// slot and ~0.93 s before it ends.
///
/// **A runaway guard, not the operating bound.** That bound is the
/// slot boundary the audio sink publishes, applied in
/// `dual_core::run_speculative_slot` (`slot_end_hint`); this only
/// catches a decode that has gone wrong rather than long, so it is
/// deliberately far out — consecutive bundles are one slot apart,
/// 15 s, and 13 s from receipt still leaves 2 s.
///
/// It was briefly the *only* bound, and a radio said what that costs
/// within three slots (2026-09-19): stage 3 ran 1.0-2.0 s past slot
/// end, the UAC reader lost ~1 s of audio in that window, the sink's
/// boundary slipped ~1 s behind UTC, and every slot decoded 0. The
/// 1836 ms it replaced shared this origin and took its value from the
/// other end — bundle receipt to key-up measured 1.836 s — which is
/// the same bound this one now defers to, computed per slot instead of
/// once.
const FT8_BUDGET_MS: i64 = match option_env!("MFSK_FT8_BUDGET_MS") {
    Some(s) => parse_u32(s) as i64,
    None => 13_000,
};

/// `dual_core::DecodeConfig::slot_end_hint` — when the slot now being
/// decoded ends, on `esp_timer_get_time()`'s clock.
///
/// Read from the capture-slot boundary the **audio sink** publishes,
/// which is the one clock here that cannot fall behind: it advances
/// with arriving audio whatever the decode task is doing. The split
/// between "this slot" and "the one before it" sits at half a slot,
/// deliberately far from both readings — an earlier version split at
/// the emit point, which is where the decoder reads it, so a few ms of
/// jitter flipped the answer by a whole slot.
fn slot_end_hint() -> Option<i64> {
    /// One FT8 slot in µs.
    const SLOT_US: i64 = 15_000_000;
    let now = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let (_, into, len) = mfsk_app_shared::time_sync::current_capture_info_with_len(now)?;
    Some(mfsk_app_shared::time_sync::decoded_slot_end_us(
        now,
        into,
        len.unwrap_or(SLOT_US),
    ))
}

/// Drops this task's priority for as long as it is held.
///
/// **Cold acquisition is background work and must stop holding a
/// real-time priority for it.** It runs 10-15 s on the decode task —
/// three tiled searches inside `acquire_slot_phases` plus up to
/// `ACQUIRE_MAX_TRIALS` full-slot decodes — and that task is priority 6
/// on core 0, where the display loop is the main task at priority 1. So
/// the panel stopped repainting for the whole acquisition, including
/// the `SYNC: capturing n/25 s` line that exists to say what is
/// happening. Reported from the bench as "everything appears to stop".
///
/// Yielding more often was the other candidate and is not enough: the
/// existing `vTaskDelay(1)` points are six in fifteen seconds, and the
/// tiled searches between them are in `mfsk_core` with no yield at all,
/// so the best it can do is a repaint every 2.5 s. Dropping to the
/// display's own priority instead lets it run *when it needs to*, and
/// costs the acquisition only the time the display actually uses —
/// that loop sleeps between frames rather than spinning.
///
/// The audio path is untouched — it runs above this and the panel
/// (`uac::AUDIO_TASK_PRIORITY`) and keeps its core.
struct LowPriorityWhile(u32);

impl LowPriorityWhile {
    /// Priority to drop to. The display's own, so the two share the
    /// core by round-robin rather than one starving the other.
    const ACQUIRE_PRIORITY: u32 = 1;

    fn new() -> Self {
        // SAFETY: both take a null handle, documented as "the calling
        // task", and `INCLUDE_uxTaskPriorityGet` / `_vTaskPrioritySet`
        // are enabled in this build.
        let was = unsafe { esp_idf_svc::sys::uxTaskPriorityGet(core::ptr::null_mut()) };
        unsafe {
            esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), Self::ACQUIRE_PRIORITY)
        };
        Self(was)
    }
}

impl Drop for LowPriorityWhile {
    fn drop(&mut self) {
        // Restored on every exit path, including the ones that `continue`
        // out of the slot: a decode task left at priority 1 would miss
        // its own deadline on every slot after.
        // SAFETY: as above.
        unsafe { esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), self.0) };
    }
}

/// Coarse score above which a slot that decoded nothing is reporting a
/// *signal* rather than the noise floor.
///
/// `sync_min` is 1.0 because that is about where the floor sits, and a
/// real station runs tens to low hundreds — the slots measured on a
/// radio 2026-09-19 that decoded nothing while the grid was a second
/// out read 14 to 220. Five is well clear of the floor and well under
/// anything a station makes.
///
/// It decides two things: what the panel says about an empty slot, and
/// whether that slot counts toward a cold acquisition at all
/// (`grid_state::observe_slot`).
const COARSE_SIGNAL_SCORE: f32 = 5.0;

/// Per-candidate fine sync (WSJT-X `ft8b.f90` Stages A/B/C) plus a
/// coarse-position retry — `dual_core::DecodeConfig::fine_sync`.
///
/// **Off, and the flag now says so.** It read `true` while
/// `FT8_FINE_SYNC_MIN_SLACK_MS` refused it on every slot a healthy
/// receiver produces, so the configuration claimed a stage that was
/// unreachable — and with `core2-app` and `m5stack-s3-app` both
/// passing `fine_sync: false`, `fine_sync_12k` had no caller on any
/// board at all.
///
/// Kept rather than deleted, because both halves of the trade are
/// measured and the answer turns on a budget that is not fixed
/// forever: it wins recall (4.21 → 4.66 mean decodes over 61 phases on
/// the shipped numeric path) and costs ~292 ms off the front of the
/// early path, which this slot cannot spare (`cut` 0 → 3-15 and three
/// slots past key-up, on a radio, 2026-09-19). Change `MAX_CAND`,
/// speed up stage 3, or move the emit point and it is worth measuring
/// again — `MFSK_FT8_FINE_SYNC=1` turns it back on, and the host
/// mirror's `MFSK_MIRROR_FINE_REFINE` measures the recall half without
/// a board.
const FT8_FINE_SYNC: bool = match option_env!("MFSK_FT8_FINE_SYNC") {
    Some(s) => parse_u32(s) != 0,
    None => false,
};

/// `dual_core::DecodeConfig::hard_sync_gate` — pass 1 picked by the hard-sync gate from the spectrogram, its order the ranking.
///
/// **Off, and not measured on a board.** `MFSK_FT8_GATE=1` turns it on. On the host's mirror of this pipeline
/// (`mirror_gate_and_osd`) it adds 0.4 decodes of 20 on `qso3_busy` alone and 2.25 together with OSD
/// (`MFSK_FT8_OSD`), with no unexplained decode; its cost here is the unknown (it scores on one core).
/// `docs/notes/S3_GATE_OSD_ABLATION.md`, and `ft8-gate-bench` is the bench that measures it.
const FT8_HARD_SYNC_GATE: bool = match option_env!("MFSK_FT8_GATE") {
    Some(s) => parse_u32(s) != 0,
    None => false,
};

/// OSD in stage 3 (`DecodeDepth { osd: true, ..EMBEDDED }`). **Off.** `MFSK_FT8_OSD=1` turns it on and **needs
/// `--features ft8-osd`** (which builds `mfsk-core/osd`); without the feature `DecodeDepth::osd` is a silent no-op, so a
/// build that asks for it without it does not compile. OSD is a time and battery decision this board's own notes keep
/// off (`CORES3_FT8_SLOT_BUDGET.md`): FT4 on an S3 measured +2 345 ms over 31 candidates for it, FT8's own cost on a
/// board is unmeasured (`ft8-gate-bench`). Its arrays are on the stack (about 9 KB against 16 KB).
const FT8_OSD: bool = match option_env!("MFSK_FT8_OSD") {
    Some(s) => parse_u32(s) != 0,
    None => false,
};
#[cfg(not(feature = "ft8-osd"))]
const _: () = assert!(
    !FT8_OSD,
    "MFSK_FT8_OSD=1 needs --features ft8-osd (mfsk-core/osd); without it OSD silently does nothing"
);

/// The stage-3 depth: the board's `EMBEDDED`, with OSD when [`FT8_OSD`].
const FT8_DEPTH: DecodeDepth = DecodeDepth {
    osd: FT8_OSD,
    ..DecodeDepth::EMBEDDED
};

/// `dual_core::DecodeConfig::fine_sync_min_slack_ms` — how much of the
/// period must still be ahead for fine sync to earn its ~292 ms.
///
/// 2 s, against a steady-state slack of ~1.43 s — so fine sync does
/// **not** run on healthy real audio, and that is a measurement rather
/// than an oversight.
///
/// It was lowered to 1.2 s for one flash, on an argument that was
/// right in both halves and wrong in the join. The recall half holds:
/// on the shipped numeric path
/// (`ft8_embedded_pipeline_mirror::mirror_phase_response`,
/// `--features fixed-point`, 61 phases with and without
/// `MFSK_MIRROR_FINE_REFINE`) fine sync moves mean decodes 4.21 →
/// 4.66 and slots at 6-7 decodes 24 → 32. The time half was the
/// mistake: ~292 ms out of ~1 430 ms looked free because #357's
/// deadline sweep found `dec` flat down to 800 ms — but that sweep ran
/// `MFSK_CORES3_FORCE_MODE=decode`, the wav_sim path, which has **no
/// early/late split**. In the real pipeline those 292 ms come off the
/// front of the *early* path and buy nothing back; they simply reduce
/// how many candidates are tried before the slot arrives.
///
/// On a radio the difference was immediate and unambiguous
/// (2026-09-19): `cut` went from 0 on every healthy slot to 3, 15, 10,
/// 11, 1, 3, and three slots finished past key-up. `dec` moved too,
/// but it swings 4-11 on this band from slot to slot and was not the
/// evidence.
///
/// WSJT-X runs its decode to completion because a PC can afford to;
/// this board cannot, and this is where that difference is spent.
const FT8_FINE_SYNC_MIN_SLACK_MS: i64 = match option_env!("MFSK_FT8_FINE_SYNC_MIN_SLACK_MS") {
    Some(s) => parse_u32(s) as i64,
    None => 2_000,
};

/// `dual_core::DecodeConfig::key_up_guard_ms` — **0: claim right up to
/// the deadline.**
///
/// This was 320 ms of clearance on top of the 500 ms key-up offset, so
/// the decoder would be out of the way before this station transmits.
/// That reason was wrong: WSJT-X never stops a decode for a
/// transmission (`MainWindow::decode()` only declines to *start* one
/// while the previous is busy), and a transmission needs a lead rather
/// than a clearance — `txDelay` is 0.2 s between PTT and audio, 20 ms
/// for FT4 (`mainwindow.cpp:8252`, `Configuration.cpp:1602`), and a
/// solid-state radio switches in milliseconds.
///
/// **What that argument got wrong, read against the upstream
/// schedule** (`dual_core::FT8_KEY_UP_AFTER_SLOT_END_US` has it in
/// full): `txDelay` is not a lead *added* to the 0.5 s, it is spent
/// *inside* it. WSJT-X asserts PTT at the period boundary and the
/// modulator pads silence so audio still lands at 0.5 s — so the
/// transceiver is not what the 0.5 s is short of. What a transmitting
/// build actually loses is earlier: the message is taken from the
/// auto-sequencer **at the boundary** (`mainwindow.cpp:4657-4711`),
/// so a decode finishing inside this 0.5 s is already too late to be
/// answered this period.
///
/// The guard therefore stays 0 — clearance was never the issue — but
/// **0 is not "the budget is free"**. Deciding what to send is a
/// separate, earlier deadline at the boundary itself, and this
/// constant cannot express it: it moves when stage 3 stops claiming,
/// not when the QSO state machine is polled. On the air 2026-09-19,
/// `post_slotend` ran median 332 ms / p90 479 ms against the 500 ms
/// here (118 slots, `logs/udp_ts_2026-09-19.log`) — inside this
/// deadline throughout, and past the reply deadline on most slots.
/// Nothing is wrong today because this board does not transmit; the
/// number to watch when it does is `post_slotend` against **0**, not
/// against 500.
///
/// The deadline itself stayed, because the thing it protects is not TX
/// at all: it is the next slot's **audio**. `FT8_KEY_UP_AFTER_SLOT_END_US`
/// gives stage 3's late path the ~0.5 s past the boundary it needs to
/// use the full slot, and no more — past that the reader starts losing
/// samples and the grid slips (`FT8_BUDGET_MS`'s note has the
/// measurement). The constant is named after key-up; what it buys here
/// is the audio pipeline.
const FT8_KEY_UP_GUARD_MS: i64 = 0;

/// `dual_core::DecodeConfig::slot_floor_ms` — **0, i.e. off.**
///
/// It dropped a slot outright when the bundle arrived too late to
/// finish before key-up. Unnecessary now that the deadline is computed
/// from the published boundary every slot: a bundle that arrives past
/// it simply claims nothing and returns, which frees the cores in the
/// same breath without a skip that has to be right about the future.
const FT8_SLOT_FLOOR_MS: i64 = 0;

/// The ±lag coarse sync searches, in seconds, **before** the emit
/// point's coverage ceiling is applied to it.
///
/// Swept on the host mirror over every distinct width the row grid
/// allows (fixed-point, 51 capture phases per recording,
/// `mirror_partial_block2_policies`, 2026-09-20) — decodes:
///
/// ```text
///   window   lags   qso3_busy   qso1   qso2
///   ±0.80     21        331       88     87
///   ±0.88     23        337      118    115
///   ±0.96     25        335      116    112
///   ±1.04     27        335      112    112
/// ```
///
/// A maximum on all three with no distinct station lost, and 23 lag
/// steps instead of 27 — 15 % off coarse sync, which on this board is
/// 100-180 ms a slot. It replaced `1.0`, which `jz = round(lag /
/// 0.08)` had been turning into 13 rows, i.e. ±1.04 s.
///
/// Confirmed on a radio the same day (40 m, IC-705): `dec` 8.90 ± 1.70
/// over the first slots against 8.54 ± 1.93 over 63 baseline slots —
/// a difference this sample cannot resolve, but no regression, and
/// `defer` went from 1.27 a slot to 0 because the candidates that
/// needed the whole slot were the ones at the extremes of the old
/// window.
const FT8_SEARCH_LAG_S: f32 = 0.88;

/// `dual_core::DecodeConfig::share_cand_budget` — off, and **inert
/// while [`FT8_SEARCH_LAG_S`] sits at the coverage ceiling**.
///
/// It divides the stage-3 budget between the early (prefix) half and
/// the late half instead of letting the early half take all of it.
/// With no deferred candidates there is no late half to give anything
/// to, and both settings compute the same two numbers:
///
/// ```text
///                     share_cand ON            share_cand OFF
///   early_budget      ready in top max_cand    max_cand
///   late_budget       max_cand - early         max_cand - n_early_refined
/// ```
///
/// When `defer == 0` every one of the top `max_cand` is ready, so the
/// first row is `max_cand` either way and the second is 0 either way.
/// The two arms run identical code.
///
/// That is the state the board is in: narrowing the search window to
/// 0.88 s took `defer` from 1.27 a slot to 0 (see
/// [`FT8_SEARCH_LAG_S`]), and a run on 2026-09-20 had **48 of 48 slots
/// at `defer = 0`**. The A/B numbers from that run — 10.00 against
/// 10.29 over 24 slots each — are two arms hearing different stations,
/// not an effect.
///
/// **This used to say "off pending a board measurement".** It was
/// measured, twice, while the window was still ±1.04 s and deferred
/// candidates existed: `-0.64` and `+0.24` decodes a slot over n = 14
/// each. Opposite signs, and smaller than that sample can resolve.
///
/// So: do not read the host mirror's gain on `qso1`/`qso2` as
/// something the board is leaving on the table. It is a gain in a
/// configuration the board no longer runs. Re-measuring this means
/// first arranging for candidates to be deferred at all —
/// `MFSK_FT8_LAG_AB` or a wider window — and the A/B harness
/// (`MFSK_FT8_SHARE_CAND_AB`) is there for when that happens.
const FT8_SHARE_CAND: bool = match option_env!("MFSK_FT8_SHARE_CAND") {
    Some(s) => parse_u32(s) != 0,
    None => false,
};

/// Alternate `share_cand_budget` in blocks of this many slots, so one
/// image measures both arms against the same band.
///
/// **Blocks, not alternate slots, and the block must be even.** Which
/// stations are audible changes with the TX period, so odd/even
/// alternation would compare arm A against one set of stations and arm
/// B against another and call the difference an effect. An even block
/// holds equally many slots of each period, so neither arm inherits a
/// period.
///
/// **Smaller blocks do not cost resolution — they buy balance.** What
/// resolves a difference is slots per arm, not block length: at the
/// `dec` spread measured here (sd 1.94) one arm needs ~45 slots to
/// resolve 1.2 decodes, ~20 for 1.9, ~12 for 2.4. Block length only
/// decides how a drift across the session (band opening, battery sag)
/// distributes over the two arms, and short blocks distribute it
/// evenly. So on a run bounded by the battery rather than by patience,
/// shorten the block and read the A/B as indicative.
///
/// `0` (the default) means no experiment: `FT8_SHARE_CAND` holds for
/// every slot, which is what a shipped build does. Set it only for a
/// measurement run — `MFSK_FT8_SHARE_CAND_AB=20` — and the boot line
/// and every `SLOT[...]` line say which arm produced them, because two
/// builds that differ only in an `option_env!` are otherwise
/// indistinguishable in a log.
const FT8_SHARE_CAND_AB: u32 = match option_env!("MFSK_FT8_SHARE_CAND_AB") {
    Some(s) => parse_u32(s),
    None => 0,
};

/// Alternate the coarse search window in blocks of this many slots,
/// so one image measures the narrowed window against the old one on
/// the same band.
///
/// `0` (the default) means no experiment. Set it for a measurement
/// run — `MFSK_FT8_LAG_AB=2` — and each `SLOT` line says which arm
/// produced it: `W` for the old ±1.04 s, `N` for [`FT8_SEARCH_LAG_S`].
///
/// **Must be even, and 2 is the right value.** Block length does not
/// set the resolution — slots per arm does — so the only thing it
/// buys is how evenly a drift in the band spreads across the two
/// arms, and short is better. The floor is 2 rather than 1 because
/// the TX period alternates every slot and each period carries a
/// different set of stations: a 1-slot alternation gives one arm
/// every odd slot and the other every even one, and the station mix
/// becomes the arm. An even block holds one of each.
///
/// **Why this exists at all.** The narrowed window was first checked
/// before-and-after in time (63 baseline slots, then 30), which gave
/// `dec` +1.43 at 3.49σ — six times what the host sweep predicted,
/// on a 40 m evening where the band itself was opening. A sequential
/// comparison cannot separate the two. Blocks put the drift on both
/// arms.
const FT8_LAG_AB: u32 = match option_env!("MFSK_FT8_LAG_AB") {
    Some(s) => parse_u32(s),
    None => 0,
};

/// The window this crate shipped before the sweep, kept only as the
/// control arm of [`FT8_LAG_AB`]. `jz = round(1.0 / 0.08)` makes it
/// 13 rows, so it is really ±1.04 s.
const FT8_SEARCH_LAG_WIDE_S: f32 = 1.0;

// **Two block experiments on one counter would be confounded.** They
// alternate on the same `SLOT_SEEN` with the same phase, so arm A of
// one is always arm A of the other and neither result means anything.
const _: () = assert!(
    FT8_LAG_AB.is_multiple_of(2),
    "MFSK_FT8_LAG_AB must be even — an odd block gives each arm one TX period"
);
const _: () = assert!(
    FT8_LAG_AB == 0 || FT8_SHARE_CAND_AB == 0,
    "MFSK_FT8_LAG_AB and MFSK_FT8_SHARE_CAND_AB alternate on the same slot counter — run one"
);

/// Slots seen since boot — the A/B block index comes off this rather
/// than off `wav_idx`, which is the sink's own numbering and need not
/// start at zero or stay dense across a re-anchor.
static SLOT_SEEN: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// `const`-context unsigned parse — `str::parse` is not `const`. Digits
/// only; anything else is a build-time panic, which is what you want
/// for a typo in a sweep env var that would otherwise silently fall
/// back to the default.
pub(crate) const fn parse_u32(s: &str) -> u32 {
    let b = s.as_bytes();
    let mut i = 0;
    let mut v: u32 = 0;
    while i < b.len() {
        assert!(b[i] >= b'0' && b[i] <= b'9', "MFSK_FT8_* knob: digits only");
        v = v * 10 + (b[i] - b'0') as u32;
        i += 1;
    }
    v
}

/// `BootMode::Decode` entry — runs the decode pipeline with the baked
/// `QSO_WAVS` playlist as the audio source. Thin wrapper around
/// [`run_with_source`].
pub fn run() -> ! {
    run_with_source("wav", |q| {
        wav_sim::spawn_with_tap(QSO_WAVS, q, crate::waterfall_feed::push)
    })
}

/// Source-agnostic entry. Allocates the pipeline queues, spawns
/// `stage1_inc`, calls `source_spawn` (which must push
/// `ChunkMsg::Samples` + `ChunkMsg::SlotEnd` into the chunk queue),
/// then runs the decode loop. Never returns.
///
/// `Decode` mode passes `|q| wav_sim::spawn(QSO_WAVS, q)`.
/// `Uac` mode passes `|q| uac::set_chunk_q(q)` — the UAC reader thread
/// starts pushing once it sees the queue handle land in its static slot.
pub fn run_with_source<F: FnOnce(QueueHandle_t)>(source: &'static str, source_spawn: F) -> ! {
    crate::log_free_internal("pre-decode-loop (post-Goertzel: no BASIS alloc)");

    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(NFFT_SPEC);
    dual_core::init();

    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    // No waterfall queue: the panel's rows come from the audio itself,
    // the same feed every mode uses (`waterfall_feed`). This used to
    // wire stage 1's per-pair spectra to a `wf_drain` task — one more
    // 4 KB internal-DRAM stack, for rows only FT8 could produce.
    stage1_inc::spawn_with_wf(chunk_q, slot_q, spec_q, None);
    source_spawn(chunk_q);

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, cores3-app phase 0)");
    // **The arm belongs in the boot line**, same reason `stage1_inc`
    // puts the emit point in its own: a measurement build and a
    // shipped build differ by one `option_env!` and a log that does
    // not say which is which cannot be re-read later.
    if FT8_LAG_AB != 0 {
        log::warn!(
            "decode pipeline: SEARCH LAG A/B in blocks of {FT8_LAG_AB} slots — arm W = \
             {FT8_SEARCH_LAG_WIDE_S:.2} s (old), arm N = {FT8_SEARCH_LAG_S:.2} s"
        );
    }
    log::info!(
        "decode pipeline: gate {} osd {} fine_sync {} (hard-sync gate / OSD / fine sync; see ft8-gate-bench)",
        FT8_HARD_SYNC_GATE,
        FT8_OSD,
        FT8_FINE_SYNC
    );
    log::info!(
        "decode pipeline: share_cand {}",
        if FT8_SHARE_CAND_AB == 0 {
            format!("fixed {FT8_SHARE_CAND} (no A/B)")
        } else {
            format!(
                "A/B in blocks of {FT8_SHARE_CAND_AB} slots — arm A = off, arm B = on; \
                 `arm=` on each SLOT line"
            )
        }
    );

    // **What the transmit side costs, measured before it exists.**
    //
    // The reply deadline is the slot boundary, not `slot end + 0.5 s`
    // (`dual_core::FT8_KEY_UP_AFTER_SLOT_END_US`), and the chain
    // between "the decoder is done" and "audio is on the air" — pick
    // the message, `pack77`, PTT, the rig's settle, synthesise
    // 151 680 samples — is in no budget anywhere. On this board
    // `post_slotend` runs median 332 ms and p90 479 ms against a 500 ms
    // bound, so the size of that chain decides whether the decoder has
    // to give time back once TX lands.
    //
    // Pure compute: no PTT, no audio interface, nothing reaches the
    // radio. `MFSK_CORES3_TX_SYNTH_BENCH=1` at build time.
    if option_env!("MFSK_CORES3_TX_SYNTH_BENCH").is_some() {
        use mfsk_core::engine::tx::message_to_tones;
        use mfsk_core::ft8::Ft8;
        // 79 symbols x 1920 samples at 12 kHz — the 12.64 s frame.
        const TX_SAMPLES_12K: usize = 79 * 1920;
        let t_pack0 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        let packed = mfsk_core::msg::wsjt77::pack77("CQ", MY_CALL, MY_GRID);
        let t_pack1 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        match packed {
            None => log::warn!("tx-synth bench: pack77 failed for CQ {MY_CALL} {MY_GRID}"),
            Some(msg77) => {
                // Five runs: the first carries the 303 KB allocation
                // the TX scheduler would also pay, the rest do not, and
                // the difference is the part a preallocated buffer
                // could remove.
                let mut first = 0i64;
                let mut best = i64::MAX;
                let mut buf: alloc::vec::Vec<i16> = alloc::vec::Vec::new();
                for run in 0..5 {
                    let t0 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                    if run == 0 {
                        buf = alloc::vec![0i16; TX_SAMPLES_12K];
                    }
                    let tones = message_to_tones::<Ft8>(&msg77);
                    mfsk_core::engine::tx::synthesize_i16_into::<mfsk_core::ft8::Ft8>(&mut buf, &tones, 12_000, 1_500.0, 20_000);
                    let dt = unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t0;
                    if run == 0 {
                        first = dt;
                    } else {
                        best = best.min(dt);
                    }
                }
                // **Where the time goes, rather than a guess at it.**
                // The synthesiser is generic `mfsk-core` code — the
                // decode side got esp-dsp backends through
                // `embedded-shared` and this path never did — so the
                // figure above is an unoptimised starting point and
                // the split decides what to aim at:
                //
                //  * `message_to_tones` is LDPC encode + Costas, once
                //    per message and nothing to do with sample rate.
                //  * `synthesize_into` is the whole cost except the
                //    i16 round trip: `synth_i16_into` allocates a
                //    second 607 KB f32 buffer and converts, on top of
                //    the 622 KB `dphi` the f32 path allocates itself.
                //    Both live in PSRAM.
                //  * what is left of `synthesize_i16_into` after that is
                //    exactly that round trip.
                let t_a = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                let tones = message_to_tones::<Ft8>(&msg77);
                let t_b = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                let mut f32buf = alloc::vec![0f32; TX_SAMPLES_12K];
                let t_c = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                mfsk_core::engine::tx::synthesize_into::<mfsk_core::ft8::Ft8>(&mut f32buf, &tones, 12_000, 1_500.0, 1.0);
                let t_d = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                log::warn!(
                    "tx-synth split: message_to_tones {} us | alloc 607 KB f32 {} ms | \
                     tones_to_f32_into {} ms | i16 round trip {} ms",
                    t_b - t_a,
                    (t_c - t_b) / 1_000,
                    (t_d - t_c) / 1_000,
                    (best - (t_d - t_c)) / 1_000
                );
                log::warn!(
                    "tx-synth bench: pack77 {} us | synth first (with 303 KB alloc) {} ms | \
                     synth best (buffer reused) {} ms | {} samples",
                    t_pack1 - t_pack0,
                    first / 1_000,
                    best / 1_000,
                    TX_SAMPLES_12K
                );
                // **The streaming synthesiser, timed the way a
                // transmitter would drive it**: 20 ms chunks into a
                // DMA-sized buffer, which is what `tx::play` sends.
                // No 622 KB `dphi`, no 607 KB f32 temp, no 303 KB
                // output — the whole waveform never exists.
                const CHUNK: usize = 240; // 20 ms at 12 kHz
                let mut chunk = [0i16; CHUNK];
                let mut stream_us = i64::MAX;
                for _ in 0..3 {
                    let t0 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                    let mut st = mfsk_core::engine::dsp::gfsk::GfskStream::new(
                        &tones,
                        1_500.0,
                        &mfsk_core::ft8::wave_gen::FT8_GFSK,
                    );
                    let mut n = 0usize;
                    while st.remaining() > 0 {
                        n += st.fill_i16(&mut chunk, 20_000);
                    }
                    let dt = unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t0;
                    stream_us = stream_us.min(dt);
                    debug_assert_eq!(n, TX_SAMPLES_12K);
                }
                log::warn!(
                    "tx-synth stream: {} ms total in {} us chunks of 20 ms \
                     ({} us per chunk, {:.1} % duty against 12.64 s of playback)",
                    stream_us / 1_000,
                    stream_us / (TX_SAMPLES_12K / CHUNK) as i64,
                    stream_us / (TX_SAMPLES_12K / CHUNK) as i64,
                    100.0 * stream_us as f32 / 12_640_000.0
                );
                log::warn!(
                    "tx-synth bench: WSJT-X puts TX audio at +0.5 s and PTT at the boundary; \
                     the rig settle is ~100 ms (IC-705). Decoder must be done by \
                     0.5 s - 0.1 s - synth."
                );
            }
        }
    }

    let mut qso = QsoManager::new(MY_CALL, MY_GRID);
    // **The callsign hash table, and the fact that it has to live
    // here.** A 28-bit callsign field can carry a 22-bit hash instead
    // of a call, and that only resolves against stations heard
    // earlier — so the table has to outlive the slot, which means it
    // belongs to this loop rather than to a decode. Without one,
    // `unpack77` renders `<...>` forever: 69 of 380 decodes on 7041
    // kHz carried one (2026-09-20), 13 of them the same station.
    //
    // **It is 7 kB in PSRAM now, and it did not used to be.** This
    // comment read "PSRAM by way of the global allocator, and bounded
    // by construction", and both halves were wrong in the way that
    // matters. The entry counts really were bounded — by the hash key
    // spaces, collisions overwriting — but a count bound is not a
    // memory bound, and every entry was a `String` plus map nodes,
    // ~130 B of *small* allocations.
    // `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` routes small to
    // internal DRAM, so the table grew in the one pool this board has
    // ~40 kB of: it fell 10.7 kB -> 3.4 kB over 22 minutes of live
    // reception on 2026-09-20 until `esp-aes` could not allocate and
    // WiFi — the only console a host-mode board has — went silent,
    // while the decoder carried on. It presented as a network fault.
    //
    // Two changes upstream of here fixed it: callsigns are stored
    // inline (`msg::hash_table::Call13`, which is what WSJT-X's
    // `character*13` always was), and this crate builds `mfsk-core`
    // with `hash-table-small`, which folds the three tables into one
    // 256-entry table. One 7.2 kB allocation, deliberately over the
    // 4 096 threshold so it lands in PSRAM, and it never grows again.
    //
    // **What is still unmeasured is the cost of that.** One shared
    // LRU evicts by recency, so all three hash widths forget a
    // station together where direct-indexed slots kept theirs until a
    // collision. The `ht=` and `unres=` fields on the `t2:` line
    // below are the instrument: `ht=` saturating at 256 while
    // `unres=` climbs is eviction costing resolutions, and is the
    // signal to raise `N_ENTRIES`.
    let mut calls = mfsk_core::msg::CallsignHashTable::new();
    // Cumulative over the session, not per slot — the question is
    // whether the unresolved *rate* drifts up as the LRU fills, which
    // a per-slot count at these decode rates cannot show. Compare
    // against the pre-rewrite baseline: 69 of 380 decodes on 7041 kHz
    // carried a `<...>` (2026-09-20), 13 of them the same station.
    let mut unresolved_total: u32 = 0;
    let mut rendered_total: u32 = 0;
    let initial = qso.call_cq(None);
    push_tx_line(&qso, Some(&initial));

    let mut slot_seq: u32 = 0;
    // Lock state and the under-par run that sends the receiver back
    // for a new grid (#356). The policy — and its host tests — live in
    // `mfsk_app_shared::grid_state`.
    // **An unplaced grid captures from the first slot, not the fourth.**
    //
    // With `TIME: AIR DT` and no persisted fix, the only thing that can
    // place the phase is a 25 s capture — so the under-par run starts
    // pre-charged and the ring starts filling now, rather than after
    // three slots have demonstrated what the configuration already
    // said. Reported from the bench: "起動したのに3スロット経ってから
    // 25s キャプチャが始まる。何を待っているのかわからない".
    // `AIR DT` is always a cold start. A phase carried in from a stored
    // fix or from the RTC is a phase nobody has checked, and a receiver
    // that starts from one spends minutes finding out it was wrong —
    // which is what the 2026-09-19 session watched happen twice. The
    // capture costs 25 s and answers the question.
    // **Unplaced means the clock could not place it**, not merely that
    // the operator chose the air.
    //
    // This read `== AirDt` alone, from when `AIR DT` skipped the sink's
    // RTC anchor and the grid genuinely started nowhere. It takes the
    // anchor now, so a board with a plausible clock starts on a phase
    // worth one slot's trial — measured on a radio 2026-09-19, that
    // phase decoded 3 stations on the second slot where the pre-charged
    // path spent 2 min 9 s and two captures. Without a clock there is
    // still nothing to try, and the capture starts from the first slot
    // exactly as before.
    // Whether this placement has already spent its one trim. Cleared
    // wherever the grid is placed afresh.
    let mut trim_used = false;
    let unplaced = crate::grid_source() == mfsk_app_shared::grid_src::GridSource::AirDt
        && mfsk_app_shared::time_sync::utc_now_ms().is_none();
    let mut grid = if unplaced {
        log::warn!("air-sync: AIR DT — cold start, capturing from this slot");
        crate::uac::arm_acquisition();
        mfsk_app_shared::grid_state::GridState::new_unplaced()
    } else {
        mfsk_app_shared::grid_state::GridState::new()
    };
    /// Coarse-candidate cap for *each* of `acquire_slot_phase`'s three
    /// tiled windows. Deliberately **not** `MAX_CAND` (15, the decode
    /// candidate cap) — `MFSK_CORES3_SIM` caught reusing it here: with
    /// only 15 pass1 candidates per ±2.5 s window, `qso3_busy`'s real
    /// signals get crowded out of the top-15 as often as not, and the
    /// circular estimate came back at `R 0.39` on a 4 s offset the
    /// phase was otherwise recovered correctly for (`dt -3.6 s`). 200 matches `tests/ft8_cold_acquisition.rs`,
    /// where the measurement that chose the tiled search over the wide
    /// one used it. One-time cost — this only runs during acquisition,
    /// never in the per-slot decode loop.
    const ACQUIRE_MAX_CAND: usize = 200;
    /// Candidate phases the acquisition tries before giving up and
    /// waiting for the next capture.
    ///
    /// Five. `ft8_cold_acquisition_fixed::cluster_then_decode_acquisition`,
    /// 40 offsets across the period on the receiver's own numeric path,
    /// finds a usable phase in the first cluster 23 times, within three
    /// 37 times, and within five every time — nothing needed a sixth.
    /// Each trial is one `decode_block_tuned` over the captured slot,
    /// so the cost is bounded by five of those, and only when
    /// acquisition runs, which is when the receiver is decoding nothing
    /// anyway.
    const ACQUIRE_MAX_TRIALS: usize = 5;
    /// One slot at 12 kHz — the window the acceptance trial decodes.
    const SLOT_TRIAL_SAMPLES: usize = 180_000;
    // **Why the correction is the median and not the earliest.**
    //
    // A frame that starts before the capture window is clipped and one
    // that ends early is not, so aligning on the *earliest* decode
    // looks right and measures right — until the measurement uses the
    // wrong search width. On the block driver's own +-2.5 s coarse
    // search it wins by 0.8 decodes a slot and reaches this
    // recording's best phase (`mfsk-core/tests/ft8_embedded_grid_
    // phase.rs`, 2026-09-16). But the per-slot path is not that
    // driver: `stage1_inc` runs `SYNC_LAG_S = 1.0`, and the earliest
    // rule pushes the station spread to +0.45..+1.75 s, where its late
    // half is outside the window the steady loop can see. Counting
    // only decodes at |dt| <= 1.0, the same sweep puts that phase at
    // **1** decode against the median rule's landing at **7**.
    //
    // Which is what this file already said, one screen down: the trial
    // searches +-2.5 s, the per-slot path does not, and the median
    // correction is what decouples them. Left here as well because the
    // earliest rule is the obvious-looking change, it was made, and
    // only a sweep that modelled the narrow window caught it.
    //
    // **And not some other average of the same sample.** The median is
    // taken over whichever stations one trial decoded, and a band's
    // stations do not share a clock (`qso3_busy` spreads 1.07 s end to
    // end), so the statistic moves with the mix. Four alternatives were
    // scored on the steady pipeline's own decodes over 24 capture
    // phases per recording, 2026-09-18
    // (`ft8_embedded_pipeline_mirror::mirror_acquisition_dt_statistic`):
    //
    // ```text
    //             qso3            qso1    qso2     landing sd (qso3/1/2)
    //   median    8.00 (88% >=8)  3.88    4.74     0.204 / 0.014 / 0.027
    //   midrange  8.38 (88%)      3.17    4.74     0.167 / 0.017 / 0.043
    //   trimmed   7.29 (38%)      3.88    4.39     0.188 / 0.013 / 0.035
    //   snrtop    8.04 (79%)      3.88    4.74     0.205 / 0.021 / 0.033
    //   pooled    7.96 (92%)      3.83    4.70     0.203 / 0.025 / 0.022
    // ```
    //
    // Nothing beats the median on more than one recording — midrange's
    // +0.38 on `qso3` is -0.71 on `qso1`, i.e. it lands 0.4 s earlier
    // and that happens to suit this one slot. The scatter the table
    // reports is not the mix either: on `qso3` all but one capture
    // phase land inside +0.10..+0.26 s, and the outlier is the one
    // trial that decoded a single station. Between `qso1` and `qso2`,
    // adjacent slots of one real session, the landing moves 29 ms.
    //
    // It costs little in any case: the phase response is flat at 7-9
    // decodes from -0.40 to +0.40 s, so the +-0.2 s this lands within
    // is worth about one station, and a landing bad enough to matter
    // decodes under `LOCK_MIN_DECODES` and re-acquires by itself.
    loop {
        // **Search wide until the grid is proven.**
        //
        // The shipping ±1.0 s is all a placed grid needs and all the
        // emitted SpecBundle can score. A grid that is *not* placed can
        // be further out — 1.65 s on a radio, 2026-09-19 — and then
        // every slot decodes nothing, so nothing can correct it either
        // and the only way back is a 25 s acquisition. Widening while
        // unlocked lets the ordinary slot find the band and the DT trim
        // pull the grid to centre, with acquisition left for errors
        // past what a spectrogram can hold.
        // **Reverted to ±1.0 s, and the reason is the emit point.**
        //
        // Widening to 1.75 s looked free because the *slot* has 184
        // rows. The SpecBundle does not: it carries rows 0..173, so a
        // candidate past `(174 - 162) * 0.08` = 0.96 s is scored
        // against rows that are **zero**, and a correlation against
        // zeros does not come out small — it comes out whatever the
        // normalisation makes of it. Measured on a radio directly after
        // the change: top candidates at −1.64, +1.72, +1.08 s with
        // scores of 20-30, `ready` down to 14 with 16 deferred, and
        // nothing decoded. The junk fills the pass-1 limit and the real
        // stations never reach stage 3.
        //
        // An error past ±1.0 s is acquisition's job — it searches the
        // whole period on a full slot, which is the only place a wide
        // search is sound.
        // **0.88, and the number is the coverage ceiling, not a round
        // one.** `stage1_inc` emits at pair 87, which fills rows
        // 0..173; block 2's last Costas symbol sits at row 162, so a
        // lag of 11 rows still lands it inside real data and a lag of
        // 12 does not. Eleven rows is 0.88 s.
        //
        // Swept on the host mirror over every distinct width the row
        // grid allows (fixed-point, 51 capture phases per recording,
        // `mirror_partial_block2_policies`, 2026-09-20). Decodes:
        //
        // ```text
        //   window   lags   qso3_busy   qso1   qso2
        //   ±0.80     21        331       88     87
        //   ±0.88     23        337      118    115
        //   ±0.96     25        335      116    112
        //   ±1.04     27        335      112    112
        // ```
        //
        // A maximum on all three, with no distinct station lost
        // anywhere, and 23 lag steps instead of 27 — 15 % off coarse
        // sync, which on this board is 100-180 ms a slot.
        //
        // **This used to read `1.0`, and that was never 1.0**: `jz =
        // round(lag / 0.08)` makes it 13 rows, i.e. ±1.04 s. The
        // sweep's `±1.04` row reproduces the shipped count exactly,
        // which is how that was noticed.
        //
        // Narrowing also retires `FT8_BLOCK2_GATE`: inside ±0.88 no
        // lag truncates block 2, so there is nothing for the gate to
        // refuse. The gate was the cleanup for lags this window should
        // not have visited — and it only ever removed the positive
        // half, which is why it *lost* decodes on `qso1`/`qso2` (91
        // and 92 against 118 and 115): the junk it left on the
        // negative side was promoted into the refine budget it freed.
        //
        // **And it is not a free constant — it is a dependent one.**
        // The ceiling moves with the emit point, so a build that
        // emits earlier silently re-opens everything above: `87` is
        // the only pair for which `0.88` happens to be right. Clamped
        // rather than documented, because the version of this that
        // was only documented is the one being fixed.
        let slot_n = SLOT_SEEN.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        let (share_cand, arm) = if FT8_SHARE_CAND_AB == 0 {
            (FT8_SHARE_CAND, '-')
        } else if (slot_n / FT8_SHARE_CAND_AB).is_multiple_of(2) {
            (false, 'A')
        } else {
            (true, 'B')
        };
        // The window arm, when that is the experiment running. Decided
        // from the same counter, which is why the const assert above
        // refuses to have both.
        //
        // **Arithmetic, not a select between two f32 constants.**
        // `if wide { 1.0 } else { 0.88 }` makes LLVM materialise a
        // `[2 x float]` constant pool and the Xtensa backend cannot
        // select it: `rustc-LLVM ERROR: Cannot select:
        // XtensaISD::PCREL_WRAPPER TargetConstantPool`. Same family as
        // the regression `release.opt-level = 1` exists for, and the
        // same workaround `engine::pipeline` already carries.
        let wide_arm = FT8_LAG_AB != 0 && (slot_n / FT8_LAG_AB.max(1)).is_multiple_of(2);
        let arm = if FT8_LAG_AB == 0 {
            arm
        } else if wide_arm {
            'W'
        } else {
            'N'
        };
        // The control arm is deliberately *above* the ceiling — that
        // is the condition being measured — so it bypasses the clamp.
        // `FT8_LAG_AB` is a constant, so only one side of this
        // survives in a given build.
        let want = FT8_SEARCH_LAG_S
            + (FT8_SEARCH_LAG_WIDE_S - FT8_SEARCH_LAG_S) * (wide_arm as u32 as f32);
        let lag_s = if FT8_LAG_AB == 0 {
            want.min(embedded_shared::stage1_inc::SPEC_EMIT_MAX_LAG_S)
        } else {
            want
        };
        if lag_s < want {
            log::warn!(
                "FT8 search lag clamped {want:.2} -> {lag_s:.2} s by the emit \
                 point's coverage ceiling — re-sweep the window for this emit pair"
            );
        }
        dual_core::set_sync_lag_s(lag_s);

        // Arm for this slot. Read once and used for both the config and
        // the log line, so the label can never disagree with what ran.

        let cfg = dual_core::DecodeConfig {
            freq_min: 100.0,
            freq_max: 3_000.0,
            sync_min: 1.0,
            pass1_limit: PASS1_LIMIT,
            max_cand: MAX_CAND,
            q_thresh: DEFAULT_Q_THRESH,
            bp_max_iter: mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
            depth: FT8_DEPTH,
            budget_ms: FT8_BUDGET_MS,
            fine_sync: FT8_FINE_SYNC,
            hard_sync_gate: FT8_HARD_SYNC_GATE,
            key_up_guard_ms: FT8_KEY_UP_GUARD_MS,
            fine_sync_min_slack_ms: FT8_FINE_SYNC_MIN_SLACK_MS,
            share_cand_budget: share_cand,
            slot_floor_ms: FT8_SLOT_FLOOR_MS,
            slot_end_hint: Some(slot_end_hint),
        };
        let out = dual_core::run_speculative_slot(spec_q, slot_q, &cfg);
        let dual_core::SpeculativeOut {
            spec,
            slot,
            results,
            n_pass1,
            n_cut,
            n_fallback,
            n_ready,
            n_deferred,
            n_early_refined,
            n_in_time,
            pass2_us,
            n_gate2_p1,
            gate2_best_rank,
            // Not read any more: the ±0.2 s/slot nudge it fed was a
            // random walk, not an acquisition (see the lock-and-hold
            // comment below). Acquisition is cold acquisition's job.
            bootstrap_dt_med: _,
            t_post_recv,
            t_coarse_done,
            t_fine_done,
            t_early_done,
            t_slot_recv,
            t_done,
            skipped,
            top3,
            slot_end_hint_us,
        } = out;
        let wav_idx = slot.wav_idx;

        // Nothing was decoded and nothing should be read from this slot:
        // its bundle reached this task with less time than the
        // un-deadlined coarse + fine sync need (`FT8_SLOT_FLOOR_MS`).
        // The audio has been received and dropped; say so and wait for
        // the next bundle, which frees the cores for the audio pipeline
        // — after a cold acquisition it has a backlog to clear.
        if skipped {
            // Two readings of the same margin: the one the floor
            // decided on (the audio sink's clock, the only one
            // available before the slot ends) and the one stage1_inc's
            // own boundary gives once the Slot arrives. They agreeing
            // is what makes a skip right; the first run of this floor
            // skipped every other slot and only the second number
            // showed it (2026-09-18).
            let margin_at = |slotend: i64| {
                (slotend + dual_core::FT8_KEY_UP_AFTER_SLOT_END_US
                    - FT8_KEY_UP_GUARD_MS * 1_000
                    - t_post_recv)
                    / 1_000
            };
            log::warn!(
                "SLOT[{wav_idx}] src={source} skipped — bundle arrived with {} ms \
                 to key-up, under the {FT8_SLOT_FLOOR_MS} ms floor \
                 (slot says {} ms, q_wait={} us)",
                slot_end_hint_us.map(margin_at).unwrap_or(0),
                margin_at(slot.slotend_us),
                t_post_recv - spec.emit_us
            );
            slot_seq = slot_seq.wrapping_add(1);
            continue;
        }

        let slotend = slot.slotend_us;
        let tail_window = (slotend - t_post_recv).max(0);
        let coarse_us = t_coarse_done - t_post_recv;
        let tail_use = (slotend.min(t_early_done) - t_coarse_done).max(0);
        let post_slotend = (t_done - slotend).max(0);
        // **Warn only past key-up.** The line used to fire whenever
        // `slot_wait` was near zero — no idle before the next slot. With
        // per-candidate fine sync that is every slot: its coarse retries
        // are the lowest-value work and are meant to fill whatever the
        // budget leaves, and on hardware (2026-09-17,
        // `logs/sim_finesync_2026-09-17.log`) they did, finishing
        // 260-505 ms past slot end on each of seven slots that all
        // decoded 8. A warning that fires every slot teaches its reader
        // to skip it — two partial slots doing that is why `full_slot`
        // below exists.
        //
        // What the operator actually loses is the reply: the budget runs
        // to this station's own key-up, `TX_START_OFFSET_S` = 0.5 s after
        // the slot boundary (`FT8_BUDGET_MS`'s derivation). A decode that
        // lands later cannot be answered this period. So that is the
        // line — the deadline stops *claiming* candidates there, and a
        // candidate already in BP can still carry the slot past it.
        //
        // Busy periods crossing it are still an operating limit on FT8,
        // not a fault: stations alternate periods, and the denser one is
        // the one that runs out (measured 2026-08-23 on 40 m, seven
        // decodes on one period against one or two on the other).
        // WSPR's and FST4's monitor loops are the other case — built
        // with deliberate slack, so an overrun there is a fault. Do not
        // carry this framing across.
        //
        // Only full slots are judged: the first slots after the grid
        // anchors to UTC are partial by construction, and comparing one
        // against a whole slot's budget produces a warning about nothing.
        const FULL_SLOT_SAMPLES: usize = 180_000;
        let full_slot = slot.audio().len() >= FULL_SLOT_SAMPLES;
        const KEY_UP_AFTER_SLOT_END_US: i64 = dual_core::FT8_KEY_UP_AFTER_SLOT_END_US;
        if full_slot && post_slotend > KEY_UP_AFTER_SLOT_END_US {
            log::warn!(
                "SLOT[{wav_idx}] src={source} PAST KEY-UP — finished {post_slotend} us after slot \
                 end, {} ms past key-up ({n_cut} candidates cut, {n_deferred} deferred)",
                (post_slotend - KEY_UP_AFTER_SLOT_END_US) / 1_000
            );
        }
        // The key-up floor reads the boundary from the audio sink
        // before the slot ends; this is that reading against
        // stage1_inc's own, the only check on it that costs nothing.
        // A slot's worth of error here is the bug of 2026-09-18.
        let hint_err = match slot_end_hint_us {
            Some(h) => format!("{:+}ms", (h - slotend) / 1_000),
            None => "na".to_string(),
        };
        // **Two lines, because the UDP sink clips at
        // `log_sink::LINE_MAX` (160).** In UAC mode that sink is the
        // only channel off the board, and this line had grown to ~250
        // characters: on a radio it arrived as
        // `... coarse=98307us fine=114~`, with every timing after
        // `fine` gone — which is most of what the line is for. Asking
        // for a longer buffer would cost internal DRAM in 44 staged
        // copies; two lines cost nothing.
        log::info!(
            "SLOT[{wav_idx}] src={source} arm={arm} grid={} p1={n_pass1} ready={n_ready} \
             defer={n_deferred} ref={n_early_refined} cut={n_cut} fb={n_fallback} dec={} \
             intime={n_in_time} gate2={n_gate2_p1}@{}",
            mfsk_app_shared::grid_src::grid_label(
                crate::grid_source(),
                mfsk_app_shared::time_sync::grid_lock(),
            ),
            results.len(),
            // Rank of the best two-block candidate, or `-` for none.
            // The count alone cannot say whether gating would save
            // anything: 12 of them below rank 25 cost nothing.
            if gate2_best_rank == u16::MAX {
                String::from("-")
            } else {
                format!("{gate2_best_rank}")
            },
        );
        // **In ms, and still two lines.** Even split, the µs form
        // clipped at `slot_wait` on a radio — six seven-digit numbers
        // do not fit in 160 characters with their labels. Nothing here
        // is decided at µs resolution: the smallest quantity that
        // matters is `hint_err`, already in ms.
        log::info!(
            "SLOT[{wav_idx}] t: cap={FT8_BUDGET_MS}ms \
             tail_win={}ms q_wait={}us hint_err={hint_err} coarse={}ms fine={}ms early={}ms",
            tail_window / 1_000,
            // How long this bundle sat in `spec_q` — a busy decode
            // task. Small while `tail_win` is also small means the
            // other case: stage1_inc emitted late, starved.
            t_post_recv - spec.emit_us,
            coarse_us / 1_000,
            // Inside `early` below, which keeps its historical meaning
            // (from coarse done) so logs from before fine sync compare.
            (t_fine_done - t_coarse_done) / 1_000,
            (t_early_done - t_coarse_done) / 1_000,
        );
        log::info!(
            "SLOT[{wav_idx}] t2: ht={}/{} unres={}/{} tail_use={}ms post_slotend={}ms \
             slot_wait={}ms late={}ms audio={}sa p2={}/{}/{}us",
            // `ht=<live>/<cap>` and `unres=<with-placeholder>/<total>`,
            // beside the `int=` the `uac: rx tick` line already
            // reports. Together they answer the one question the hash
            // table rewrite left open.
            //
            // `int=` no longer has anything to do with this table —
            // it is one fixed 7.2 kB PSRAM allocation now. What is
            // open is whether folding three tables into one
            // recency-evicting LRU costs resolutions. `ht=` pinned at
            // the cap with `unres=` climbing says yes; `ht=` below
            // the cap says the table has not even filled and any
            // `<...>` is a station genuinely never heard.
            //
            // `len22` and not the number of callsigns *inserted*.
            // Before the rewrite the 10-/12-bit tables were keyed on
            // the hash, so a collision overwrote and their `len()`
            // counted occupied slots rather than stations — 39 and 41
            // against 42 real callsigns, measured on air 2026-09-20.
            // `len22` was the one number that meant what it said, and
            // in the unified table it is the only one there is.
            //
            // The counters lag the line by one slot: this logs before
            // the decodes below are rendered. They are cumulative, so
            // it does not matter, but it is why slot 0 reads 0/0.
            calls.len22(),
            calls.capacity22(),
            unresolved_total,
            rendered_total,
            tail_use / 1_000,
            post_slotend / 1_000,
            (t_slot_recv - t_early_done) / 1_000,
            (t_done - t_slot_recv) / 1_000,
            // The slot the decoder was actually handed. A re-anchor
            // shortens it, and a short slot is why a healthy-looking
            // `p1` decodes nothing.
            slot.audio().len(),
            // Pass 2's fixed head/tail split: main's own half, main
            // blocked on the worker, the worker's own half. See
            // `dual_core::PASS2_MAIN_US` — `stage3_split` work-steals
            // and this one does not, so whichever core draws the
            // cheaper half idles for the difference.
            pass2_us.0,
            pass2_us.1,
            pass2_us.2,
        );
        // **What the coarse search saw, when nothing decoded.**
        //
        // `p1=30` is the candidate limit, not evidence of signal: the
        // list fills from noise just as readily. `sync_min` is 1.0
        // because that is about where the noise floor sits, so the top
        // score separates "an empty band or an audio problem" from "a
        // station this decoder could not read".
        // **The status strip, decided in one place further down.**
        //
        // It shares the TX line's row, so only one thing can be on it
        // and the precedence has to be explicit: an acquisition in
        // flight, then a grid the operator asked the air for and the
        // air has not confirmed, then what a slot that decoded nothing
        // actually heard. Empty hands the row back to the TX line.
        let mut strip: heapless::String<32> = heapless::String::new();

        // **Was there anything to decode?** Coarse sync's own top score,
        // which the panel shows and the acquisition trigger counts.
        let had_signal = !results.is_empty()
            || top3
                .first()
                .is_some_and(|&(sc, _, _)| sc >= COARSE_SIGNAL_SCORE);
        if results.is_empty() && !skipped {
            let n = n_pass1.min(3);
            let mut line: heapless::String<96> = heapless::String::new();
            for (s, dt, f) in top3.iter().take(n) {
                let _ = write!(&mut line, " [{s:.2} @ {dt:+.2}s {f:.0}Hz]");
            }
            log::info!("SLOT[{wav_idx}] nothing decoded — top {n} coarse:{line}");
            // **And put it on the panel.**
            //
            // "Is the grid off, or is the band empty?" is the question
            // an operator asks at exactly this moment, and the answer
            // was computed here and then only logged — on a board whose
            // console the USB host driver has taken. A strong candidate
            // at a large |dt| is a signal the grid is missing; a top
            // score at the noise floor (`sync_min` is 1.0, which is
            // about where the floor sits) is an empty band or a dead
            // input, and those want opposite actions.
            //
            // The strip is the TX line's, shown only while this is
            // non-empty, so it costs no screen space — the panel is
            // already narrow enough that a second row was declined.
            if let Some(&(sc, dt, _)) = top3.first() {
                if sc >= COARSE_SIGNAL_SCORE {
                    let _ = write!(&mut strip, "SIG {sc:.0} @ {dt:+.2}s — grid off?");
                } else {
                    let _ = write!(&mut strip, "SIG {sc:.1} — band quiet");
                }
            }
        }

        for r in results.iter() {
            mfsk_app_shared::time_sync::record_decode_dt(r.dt_sec);
        }
        mfsk_app_shared::time_sync::finalize_slot();
        let n_dec = results.len();
        // This slot's median, or `None` if it produced no decodes — in
        // which case `finalize_slot` kept the previous estimate and
        // `slot_dt_offset` would report *that*, which is not this slot.
        let slot_median = if n_dec > 0 {
            mfsk_app_shared::time_sync::slot_dt_offset()
        } else {
            None
        };
        if let Some(off) = mfsk_app_shared::time_sync::slot_dt_offset() {
            log::info!(
                "  median DT = {:+.3} s ({} slots)",
                off,
                mfsk_app_shared::time_sync::slots_finalised()
            );
        }
        // Cross-slot phase filter (#356b) — tracks under NTP too, so the
        // panel's estimate is meaningful whatever the grid follows.
        // Fed from the *pooled* multi-slot median, not the raw
        // single-slot one — see `pooled_dt_median`'s doc comment for
        // why a single slot's median is too small a sample on a real,
        // multi-station band to smooth into a servo signal by itself.
        if source == "uac" {
            if slot_median.is_some() {
                if let Some(pooled) = mfsk_app_shared::time_sync::pooled_dt_median() {
                    mfsk_app_shared::time_sync::observe_slot_phase(pooled, 15.0);
                }
            }
        }

        // Self-align the slot grid from the air (#356), for the live
        // source before NTP has disciplined the clock. `Ft8ChunkSink`
        // hands the phase to the UTC drift check the moment
        // `clock_is_disciplined()` turns true; on a hilltop with no
        // network it never does, and the grid would otherwise free-run
        // at a phase uniform over 15 s against a mode that tolerates
        // ±2.5 s. Coarse sync's own DT — the confirmed-decode median
        // once decodes exist, the top-5 candidate median
        // (`bootstrap_dt_med`) before then — is a time reference present
        // wherever the receiver is, posted through
        // `set_bootstrap_slot_shift_12k`.
        //
        // Gated on `!clock_is_disciplined()` so that once NTP is up this
        // whole block — the median read, the shift maths, the log — does
        // not run at all, rather than computing a correction
        // `Ft8ChunkSink` would only drain. The `wav` source defines its
        // own boundaries and is never touched.
        if source == "uac" && !mfsk_app_shared::time_sync::clock_is_disciplined() {
            // **Lock and hold.** The grid is established once — by cold
            // acquisition below, or by NTP/RTC — and then left alone.
            // Nothing steers it per-slot any more, and that is the point.
            //
            // The board's own oscillator is ~3 ppm: 45 µs of drift per
            // 15 s slot, 11 ms per hour, 0.26 s per *day*, against a
            // search window measured in seconds. There is no physical
            // process fast enough to need a per-slot servo.
            //
            // What the old servo was actually tracking was noise. A
            // decode's DT is *that station's* clock error, not ours —
            // WSJT-X reports it and never feeds it back into its own
            // capture window. Which stations decode changes slot to
            // slot, so the median moves with the station mix; and since
            // fading correlates over tens of seconds, the same biased
            // subset can persist for several slots running, which no
            // amount of pooling or EMA smoothing can separate from a
            // real error. Measured on hardware via `MFSK_CORES3_SIM`:
            // the raw per-slot median jumped 0.205 s between adjacent
            // slots with *zero* shift applied in between; feeding it
            // back oscillated the grid to ±0.6 s; pooling the raw
            // per-decode DTs across slots and EMA-filtering the result
            // still wandered to -0.74 s over six consecutive slots,
            // with `dec` falling 8 → 4 while it did.
            //
            // So: no `set_bootstrap_slot_shift_12k` from decode DTs at
            // all, in any branch. `bootstrap_dt_med`'s ±0.2 s/slot
            // nudge goes too — its own doc admits it is "essentially
            // always `Some`, just a small near-random value when the
            // true signal is outside ±1 s", which is a random walk, not
            // an acquisition. Acquisition is cold acquisition's job
            // (25 s capture, ±2.5 s tiled search, circular statistics,
            // R ≥ 0.55 gate), and the same trigger doubles as the
            // recovery path if the grid ever is genuinely lost.
            //
            // `observe_slot_phase` above still runs: the filtered phase
            // is a panel readout, which is what #356b built it for.
            // Lock / re-acquire policy lives in
            // `mfsk_app_shared::grid_state`, which is host-tested —
            // both bugs it guards against (one decode counting as a
            // lock, and an under-par run that never reaches
            // acquisition) are reachable from a short sequence of
            // decode counts, and finding them by reflashing the board
            // cost several cycles apiece.
            //
            // **Only full slots vote.** A partial slot — the first after
            // the grid anchors, whose audio starts wherever the anchor
            // fell — places the signals somewhere a full slot on the
            // same grid does not, so what it decodes says nothing about
            // the grid. On hardware (`logs/sim_finesync_2026-09-17.log`,
            // `sim_finesync_retryorder_2026-09-18.log`) slot 0 held
            // 153 600 and 134 400 samples, decoded 8 and 5, and locked;
            // every full slot on that grid then decoded 0, and the
            // six-slot relock wait plus the acquisition kept the band
            // dark for about 2.5 minutes. The run before (2026-09-16)
            // had a slot 0 of 87 600 samples that decoded nothing, did
            // not lock, and reached steady decoding four slots sooner.
            // The partial slot's decodes are still shown; it just
            // neither locks nor counts as under par.
            // **The acquiring flag is read before the observation,
            // because the observation sets it.**
            //
            // `observe_slot` flips `acquiring` inside the same call
            // that returns `Acquire`, so a trim gated on
            // `!grid.is_acquiring()` after it is gated on a decision
            // that has already been taken — which is how the first cut
            // of this reordering (2026-09-19) moved `arm_acquisition`
            // below the trim and changed nothing at all: `trimmed`
            // could never be true on the one path it was written for.
            let was_acquiring = grid.is_acquiring();
            // **The cheap correction is decided first.**
            //
            // `arm_acquisition` used to run in the `Acquire` arm below
            // and this block was gated on `!grid.is_acquiring()`, so
            // the trim could only fire on a grid that was already good
            // enough not to want it — and never on the one case it
            // exists for. Measured on a radio 2026-09-19: a slot
            // decoded 3 stations at a median DT of −0.997 s, one
            // millisecond inside the coarse search's own edge, and the
            // receiver armed a 25 s capture instead of moving 1.0 s
            // from the three decodes already in hand. Two captures
            // followed, both decoding nothing, and the slots between
            // them were spent.
            // **Trim the grid from the air, once, when the pool agrees.**
            //
            // This is what `TIME: AIR DT` is for and what it was not
            // doing: with no NTP the phase comes from the RTC, whose
            // battery life on this board is unknown and whose read is
            // whole seconds, and nothing corrected the remainder. The
            // DT median was measured, logged, and thrown away.
            //
            // Not the per-slot servo that was removed on 2026-09-05 —
            // that fed every slot's median, including slots with one or
            // two decodes, and oscillated the grid to ±0.6 s with `dec`
            // falling 8 → 4. This waits for `DT_TRIM_MIN_OBS`
            // observations pooled across slots, moves once, and clears
            // the pool so the next move needs fresh evidence. A station's
            // own clock error averages out over that many; a grid error
            // does not.
            //
            // Saturated at `DT_TRIM_MAX_S`: more than that is not a trim
            // and belongs to cold acquisition, which searches the whole
            // period.
            const DT_TRIM_MIN_OBS: usize = 8;
            const DT_TRIM_MIN_S: f32 = 0.15;
            /// The same floor for a grid running *early*, where the
            /// correction costs a slot outright.
            ///
            /// 0.85 s, just inside the ±1.0 s search. An early grid is
            /// absorbed by that search rather than lost to it — the
            /// slots measured at +0.65 s on a radio 2026-09-19 decoded
            /// four and eight stations — so the error has to be about
            /// to leave the window before a whole slot is worth
            /// spending on it. At 0.6 s the same log shows the cost:
            /// +0.645 and +0.606 each bought a 0.6 s stub slot that
            /// decoded nothing, and the stubs came fast enough to keep
            /// the loop in them.
            const DT_TRIM_MIN_EARLY_S: f32 = 0.85;
            const DT_TRIM_MAX_S: f32 = 1.0;
            // **Pooled evidence only.**
            //
            // A single slot's median was admitted for a while — first
            // for any error over `LOCK_MIN_DECODES` decodes, which is
            // the 2026-09-05 per-slot servo under another name and
            // walked the grid −0.844, −1.004, −0.764, −0.202, +0.671 on
            // five consecutive slots; then only past 0.5 s, on the
            // argument that no station's own clock explains that much.
            //
            // Both were reaching for a correction the receiver needed
            // *often*, and it needed one often because the RTC write
            // was releasing a second late and every `AIR DT` boot
            // started a second out (fixed in
            // `rtc::write_from_system_clock`, 2026-09-19 — the same
            // band then read `dt +0.04` with no trim at all). With the
            // clock right this is back to what it was designed as: a
            // correction that waits for a sample across transmitters,
            // because that is the only kind that tells a grid error
            // from one loose station.

            let pooled_sample = if mfsk_app_shared::time_sync::dt_pool_len() >= DT_TRIM_MIN_OBS {
                mfsk_app_shared::time_sync::pooled_dt_median()
            } else {
                None
            };
            let mut trimmed = false;
            // **One trim per placement, and then never again.**
            //
            // A cooldown was tried first — wait two slots, measure,
            // correct again — and it is the wrong shape for this
            // problem. With the audio rate fixed (−5.8 ms a slot,
            // measured over 30 minutes) a grid that is placed *stays*
            // placed: there is nothing to track, so a controller that
            // keeps correcting is only ever reacting to the noise in
            // its own measurement. That is the 2026-09-05 finding
            // restated, and the cooldown version walked the grid the
            // same way, just more slowly.
            //
            // So the trim is a one-shot belonging to a placement, not a
            // rate. `trim_used` is cleared where the grid is placed —
            // the sink's anchor and an applied acquisition — and set
            // here. A placement that ends up wrong is not this
            // mechanism's problem: it is `REACQUIRE_TRIGGER_SLOTS`'s,
            // which re-places and re-arms this.
            //
            // **Full slots only.** A short slot — a correction's own
            // stub, or the first after a re-anchor — has a DT measured
            // against audio that is not a slot.
            if !full_slot || trim_used {
                // nothing to measure, or nothing left to spend
            } else if !mfsk_app_shared::time_sync::clock_is_disciplined() && !was_acquiring {
                if let Some(m) = pooled_sample {
                    // **Asymmetric, because the two directions do not
                    // cost the same.** A grid running late is corrected
                    // by shortening one slot, which still reaches
                    // `SPEC_EMIT_PAIR` and still decodes. A grid
                    // running early cannot be corrected by lengthening
                    // — `stage1_inc::NMAX` is 180 000 and is the
                    // spectrogram's geometry — so the only exact move
                    // is a stub slot of `15 s − error`, and that slot
                    // decodes nothing (`p1=0`, measured). Inside the
                    // ±1.0 s search an uncorrected early grid still
                    // works, just off-centre, so it has to be worth a
                    // whole slot before it is worth correcting.
                    let floor = if m > 0.0 {
                        DT_TRIM_MIN_EARLY_S
                    } else {
                        DT_TRIM_MIN_S
                    };
                    if m.abs() >= floor {
                        let applied = m.clamp(-DT_TRIM_MAX_S, DT_TRIM_MAX_S);
                        // Same channel and the same sign as cold
                        // acquisition's one-shot: DT > 0 means the slot
                        // opened early, so lengthen the next one.
                        mfsk_app_shared::time_sync::set_acquisition_shift_12k(
                            (applied * 12_000.0).round() as i32,
                        );
                        log::warn!(
                            "  air-sync: trimming the grid {applied:+.3} s from the pooled \
                             decodes (median {m:+.3} s) — one shot"
                        );
                        trimmed = true;
                        trim_used = true;
                        mfsk_app_shared::time_sync::reset_dt_pool();
                        mfsk_app_shared::time_sync::reset_slot_phase();
                    }
                }
            }

            /// How late a decode has to finish before the slot stops
            /// being evidence about the grid.
            ///
            /// **A cold acquisition wrecks the slots behind it.** It
            /// holds the decode task for 25 s of capture plus 10-15 s
            /// of compute, and a bundle that queues up meanwhile is
            /// decoded after its own slot has gone: no tail window, the
            /// early path with nothing to work on, candidates cut
            /// wholesale. Measured on hardware — `tail_win=0`,
            /// `cut=15`, `dec=0` on a full 180 000-sample slot with 30
            /// coarse candidates, `post_slotend` 2 747 ms (2026-09-20,
            /// SIM) and 27 798 / 13 982 ms (2026-09-19, radio).
            ///
            /// Counted as evidence, such a slot says "signal present,
            /// nothing decoded" — exactly what the acquisition trigger
            /// looks for, so **the acquisition's own wreckage asks for
            /// another acquisition**, and a failed capture leaves the
            /// run standing so the retry is immediate. That is the loop
            /// that kept `AIR DT` acquiring.
            ///
            /// The test is the pair, not the queue wait. `q_wait` was
            /// tried first at half a slot and missed the case above by
            /// a factor of two: a *successful* acquisition holds the
            /// pipeline ~3 s, a failed one ~40, and both ruin the slot
            /// equally. `tail_win == 0` says the bundle arrived at or
            /// after its own slot end, which never happens on a healthy
            /// slot (measured 889-927 ms, every slot of a 102-slot
            /// run); `post_slotend` past this separates it from a slot
            /// that merely ran long (median 336 ms, p90 454 ms in the
            /// same run).
            const WRECKED_POST_SLOTEND_US: i64 = 1_000_000;
            let backlogged = tail_window == 0 && post_slotend > WRECKED_POST_SLOTEND_US;
            if backlogged {
                log::warn!(
                    "  air-sync: slot decoded {} ms after its own end with no tail window — \
                     not counted toward the grid",
                    post_slotend / 1_000
                );
            }
            let action = if full_slot && !backlogged {
                // The slot's own DT median is the phase error, and the
                // lock decision needs it: a count alone cannot tell a
                // centred grid from one 0.7 s out that still catches
                // the loud half of the band.
                grid.observe_slot(n_dec, slot_median, had_signal)
            } else {
                if !full_slot {
                    // The backlogged case has already said so above,
                    // with the number that explains it.
                    log::info!(
                        "  air-sync: partial slot ({} samples, {n_dec} decoded) — not counted \
                         toward the grid",
                        slot.audio().len()
                    );
                }
                mfsk_app_shared::grid_state::GridAction::Hold
            };
            match action {
                mfsk_app_shared::grid_state::GridAction::Lock { n_dec } => {
                    log::info!(
                        "  air-sync: grid locked (N={n_dec} ≥ {}) — holding",
                        mfsk_app_shared::grid_state::LOCK_MIN_DECODES
                    );
                    mfsk_app_shared::time_sync::note_grid_lock(
                        mfsk_app_shared::time_sync::GridLock::Air,
                    );
                }
                mfsk_app_shared::grid_state::GridAction::Acquire { slots, relock } if trimmed => {
                    // `observe_slot` already set `acquiring`; hand it
                    // back, or nothing fills the ring, the acquisition
                    // never completes, and the trim stays gated off for
                    // the rest of the session. `true` because a phase
                    // *was* applied — the trim's — so the under-par run
                    // restarts on the same terms a capture would give
                    // it.
                    grid.acquisition_done(true);
                    // **The trim just moved the grid**, so the slots
                    // that led here were measured at the old phase and
                    // say nothing about the new one. Acquisition is for
                    // a grid that decodes *nothing* — the one case a
                    // trim cannot reach, because it has no DT to use.
                    let _ = (slots, relock);
                    log::info!(
                        "  air-sync: acquisition deferred — the trim moved the grid this slot"
                    );
                }
                mfsk_app_shared::grid_state::GridAction::Acquire { slots, relock } => {
                    if relock {
                        // The panel stops claiming a grid the decoder
                        // can no longer demonstrate.
                        mfsk_app_shared::time_sync::note_grid_lock(
                            mfsk_app_shared::time_sync::GridLock::FreeRun,
                        );
                    }
                    crate::uac::arm_acquisition();
                    log::warn!(
                        "  cold-acquisition: {} {slots} slots — capturing 25 s of FT8",
                        if relock {
                            "no decode since the lock for"
                        } else {
                            "grid under par for"
                        }
                    );
                }
                mfsk_app_shared::grid_state::GridAction::Hold => {}
            }
            if grid.is_locked() {
                mfsk_app_shared::time_sync::note_grid_lock(
                    mfsk_app_shared::time_sync::GridLock::Air,
                );
            }

            // Keep the channel drained at 0 so nothing stale can reach
            // `Ft8ChunkSink` — the only writer left is cold acquisition,
            // through its own uncapped one-shot channel.
            mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(0);

            if grid.is_acquiring() {
                // **Say so on the panel.** Nothing decodes for the 25 s
                // capture and the arithmetic after it, and the only
                // grid indicator on screen is one character saying what
                // the grid *is* — so a receiver working on a lock and a
                // receiver that has stopped looked identical.
                if let Some((have, want)) =
                    crate::uac::acquisition_fill(crate::uac::ACQUIRE_CAPTURE_SAMPLES)
                {
                    let mut line: heapless::String<32> = heapless::String::new();
                    let _ = write!(
                        &mut line,
                        "SYNC: capturing {}/{} s",
                        have / 12_000,
                        want / 12_000
                    );
                    if let Ok(mut ui) = UI.lock() {
                        ui.set_acq_line(line.as_str());
                    }
                }
                if let Some(audio) =
                    crate::uac::take_acquisition_audio(crate::uac::ACQUIRE_CAPTURE_SAMPLES)
                {
                    // Everything from here to the end of this block is
                    // the 10-15 s of compute; hand the core back to the
                    // panel for the duration.
                    let _low = LowPriorityWhile::new();
                    // **Try the clusters; the decoder decides.**
                    //
                    // Acquisition used to reduce the candidates to one
                    // phase and gate it on `r`. Neither part worked:
                    // the reduction returned a phase decoding nothing
                    // on 16 of 40 offsets, and `r` read up to 1.00 on
                    // exactly those — on hardware three acquisitions in
                    // a row were accepted at `r >= 0.98` while taking
                    // the receiver from one decode to none.
                    //
                    // What no statistic over the candidates can say is
                    // which cluster is the grid rather than a loud
                    // outlier station or a correlation artefact. What
                    // can is a decode, and the 25 s capture needed for
                    // one is already in hand. A phantom counts — its
                    // message is nonsense, its sync is real, and only
                    // its dt matters here.
                    //
                    // The trial searches the crate default ±2.5 s, not
                    // the narrower window the per-slot path runs, and
                    // it may: the phase applied is not the cluster
                    // centre but that centre corrected by the median DT
                    // of what the trial decoded. The correction is what
                    // decouples the two windows — measured, a 2.5 s
                    // trial with it accepts nothing outside ±1.0 s,
                    // where without it four of forty were.
                    // **Let the idle task run between the pieces.**
                    //
                    // Everything below is one uninterrupted stretch of
                    // compute on this task: three tiled searches inside
                    // `acquire_slot_phases` (543-635 ms each, measured
                    // on hardware 2026-09-16) and then up to
                    // `ACQUIRE_MAX_TRIALS` full-slot decodes at ~1.1 s.
                    // 10-15 s in total, during which `IDLE1` never gets
                    // scheduled and the task watchdog fires every 5 s —
                    // seven times in one 180 s capture the first time
                    // this path ran on a board at all. Nothing is
                    // wedged (`CONFIG_ESP_TASK_WDT_PANIC` is off, and
                    // the acquisition completes and locks), but the
                    // log then carries a backtrace per event, and that
                    // reads as a fault to whoever finds it next.
                    //
                    // One tick, four or five times per acquisition, is
                    // enough for `IDLE1` to feed its own watchdog. The
                    // slot-budget path never sees it: acquisition runs
                    // between slots, not inside one.
                    unsafe { esp_idf_svc::sys::vTaskDelay(1) };
                    let phases = mfsk_core::ft8::acquire::acquire_slot_phases(
                        &audio,
                        100.0,
                        3_000.0,
                        1.0,
                        ACQUIRE_MAX_CAND,
                        ACQUIRE_MAX_TRIALS,
                    );
                    let mut applied: Option<(f32, usize, usize)> = None;
                    // **Where in its slot the capture began.** The ring
                    // starts filling whenever `arm_acquisition` ran —
                    // after the slot that triggered it had decoded, so
                    // part-way into the next — and every phase below is
                    // measured from the capture's first sample. The grid
                    // shift is relative to a slot boundary, so the
                    // capture's own offset into its slot belongs in it.
                    //
                    // It was missing. On hardware (2026-09-18, clockless,
                    // feed 3.000 s late) the first acquisition armed
                    // ~0.98 s into a slot and applied +1.90 s, leaving the
                    // grid 1.10 s short — outside the per-slot ±1.0 s
                    // search, so a second acquisition was needed and the
                    // band stayed dark ~3 minutes. The next run recorded
                    // the offset directly: 0.395 s, with the grid landing
                    // ~0.2 s short. The remainder is the trial median's
                    // own station-mix bias, which this does not remove —
                    // measured since at ~0.2 s worst case and ~0.03 s
                    // between adjacent real slots, and no cheaper to
                    // remove by averaging differently (see the note
                    // beside `ACQUIRE_MAX_TRIALS`).
                    let start_s = crate::uac::acquisition_start_in_slot()
                        .map_or(0.0, |n| n as f32 / 12_000.0);
                    for (trial, &(centre, _)) in phases.iter().enumerate() {
                        unsafe { esp_idf_svc::sys::vTaskDelay(1) };
                        {
                            let mut line: heapless::String<32> = heapless::String::new();
                            let _ =
                                write!(&mut line, "SYNC: trying {}/{}", trial + 1, phases.len());
                            if let Ok(mut ui) = UI.lock() {
                                ui.set_acq_line(line.as_str());
                            }
                        }
                        // **Cut at the nearest offset a slot fits
                        // behind, not at the centre itself.**
                        //
                        // `acquire_slot_phases` returns centres in
                        // (−7.5, +7.5]; `rem_euclid` turns a negative
                        // one into an offset in (7.5, 15] s, and a
                        // whole slot only fits behind an offset up to
                        // `max_off` = 10 s of a 25 s capture. So every
                        // centre in (−5, 0) — **a third of the phase
                        // space** — had no slot behind it, and its
                        // trial was skipped (silently until
                        // `c47a78fb`). On the host mirror
                        // (`mirror_acquisition_unreachable_phases`)
                        // that is 50 of 150 centres on each of the
                        // three recordings, and on `qso3_busy` six of
                        // thirty capture starts acquired *nothing at
                        // all*: every centre that would have decoded
                        // was one of the skipped ones.
                        //
                        // Capturing two slots would remove the limit
                        // and needs ~1.44 MB transiently — the board
                        // died of `rust_oom` trying, see
                        // `uac::ACQUIRE_CAPTURE_SAMPLES`. The trial's
                        // own window has the room instead.
                        // `decode_block_tuned` searches ±2.5 s about
                        // wherever the slot is cut; the reachable band
                        // `[0, max_off]`'s complement is 5 s wide and
                        // wraps at both ends, so no centre is further
                        // than 2.5 s from it and the clamp always
                        // lands inside the search.
                        //
                        // What makes it correct is the line below
                        // measuring the applied phase from the offset
                        // actually cut at rather than from the centre:
                        // the median DT is relative to the cut. The
                        // two agree only while nothing moves.
                        //
                        // Measured, skip → clamp, decodes at the grid
                        // that results: qso3 4.70 → 6.07, qso1
                        // 3.32 → 3.86, qso2 3.48 → 4.38.
                        //
                        // **And on the board, against the board
                        // without it** (`MFSK_CORES3_SIM`, feed 12.5 s
                        // out = a true phase of −2.5 s, the middle of
                        // the band with nothing behind it;
                        // `logs/sim_acq_{clamp,control_noclamp}_offset12500_2026-09-20.log`).
                        // Without: the two heaviest clusters
                        // (−0.377, −3.018) both skipped, trials 3 and
                        // 4 decoded 0, trial 5 at −5.367 decoded *one*
                        // and set the grid 0.96 s out — 2 decodes a
                        // slot for six slots, one of them 0 at
                        // `cut=15`, then a second 25 s acquisition.
                        // With: trial 2 cuts at 10.10 s instead of
                        // 11.98 s, decodes 6, grid lands at −2.55 s,
                        // steady 6 a slot from the next slot on,
                        // `cut=0`, one acquisition, no trim. Seven
                        // slots earlier.
                        let max_off = audio.len().saturating_sub(SLOT_TRIAL_SAMPLES);
                        let want = ((centre * 12_000.0).round() as i64)
                            .rem_euclid(SLOT_TRIAL_SAMPLES as i64)
                            as usize;
                        let off = if want <= max_off {
                            want
                        } else if want - max_off < SLOT_TRIAL_SAMPLES - want {
                            max_off
                        } else {
                            0
                        };
                        if off != want {
                            log::info!(
                                "    acq trial {}/{}: centre={centre:+.3} has no slot behind \
                                 it — cutting at {:+.3} s",
                                trial + 1,
                                phases.len(),
                                off as f32 / 12_000.0,
                            );
                        }
                        if audio.len() < off + SLOT_TRIAL_SAMPLES {
                            // Only reachable if the capture is shorter
                            // than one slot, which `acquire_slot_phases`
                            // refuses before returning any centre at
                            // all. Kept so that a short buffer cannot
                            // panic the decode task.
                            log::warn!(
                                "    acq trial {}/{}: SKIPPED — needs {} samples, capture has {}",
                                trial + 1,
                                phases.len(),
                                off + SLOT_TRIAL_SAMPLES,
                                audio.len(),
                            );
                            continue;
                        }
                        let got = mfsk_core::ft8::decode_block::decode_block_tuned(
                            &audio[off..off + SLOT_TRIAL_SAMPLES],
                            100.0,
                            3_000.0,
                            1.0,
                            DecodeDepth::EMBEDDED,
                            MAX_CAND,
                            mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
                        );
                        if got.is_empty() {
                            log::info!(
                                "    acq trial {}/{}: centre={centre:+.3} decoded=0",
                                trial + 1,
                                phases.len(),
                            );
                            continue;
                        }
                        // The trial's own decodes place the grid far
                        // better than the cluster centre does. The
                        // median of them, not the earliest — see the
                        // note beside the acquisition constants for the
                        // measurement that says so.
                        let mut dts: heapless::Vec<f32, 32> = heapless::Vec::new();
                        for r in got.iter() {
                            let _ = dts.push(r.dt_sec);
                        }
                        dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                        let med = dts[dts.len() / 2];
                        // From the cut, not from the centre — see the
                        // clamp above. `off` is in [0, 15) s where
                        // `centre` was in (−7.5, +7.5]; the wrap below
                        // normalises either.
                        let mut dt = off as f32 / 12_000.0 + med + start_s;
                        while dt > 7.5 {
                            dt -= 15.0;
                        }
                        while dt <= -7.5 {
                            dt += 15.0;
                        }
                        // **Best of the trials, not the first that
                        // decodes anything.**
                        //
                        // This used to `break` here. One decode was
                        // therefore enough to set the grid for the
                        // session — the same "one decode is not a
                        // lock" that `grid_state` refuses by name
                        // (`LOCK_MIN_DECODES = 3`, and its doc: a grid
                        // a full second out still decodes the odd
                        // station). Measured 2026-09-16 across four
                        // acquisitions on the board, the accepted
                        // trial decoded 7, 2, 1 and 6; the 1 came from
                        // trial 3 of 5, so two better phases were
                        // computed, ranked, and never tried. The grid
                        // it set then decoded 0-1 per slot until the
                        // next acquisition three minutes later.
                        //
                        // Ranking by decode count is the same evidence
                        // the acceptance rule already uses, applied to
                        // all of the candidates instead of to the
                        // first. The extra cost is bounded and small
                        // against what acquisition already spends: at
                        // most `ACQUIRE_MAX_TRIALS - 1` further
                        // full-slot decodes (~1.1 s each on this
                        // board) on top of a 25 s capture.
                        //
                        // Ties keep the earlier trial, which is the
                        // higher-scoring cluster from
                        // `acquire_slot_phases`.
                        if applied.is_none_or(|(_, _, best_n)| got.len() > best_n) {
                            applied = Some((dt, trial + 1, got.len()));
                        }
                        // **Stop once a trial clears the lock bar.**
                        //
                        // Best-of-N is about not settling for a trial
                        // that decoded one station; it is not about
                        // trying every cluster for its own sake, and
                        // trying them all is expensive: acquisition
                        // measured 15.2 s of compute on this task
                        // (2026-09-18), one whole slot period, which is
                        // what pushes the next slot past key-up.
                        //
                        // `LOCK_MIN_DECODES` is the receiver's own
                        // statement of what counts as a grid, so a
                        // trial that reaches it is not a lucky single
                        // decode and the remaining clusters have
                        // nothing to prove. Measured on the host mirror
                        // (`mirror_acquisition_early_stop`, 24 capture
                        // phases per recording): identical decodes on
                        // all three recordings — qso3 8.00, qso1 3.88,
                        // qso2 4.74, the same as trying all five — for
                        // 2.3 trials per acquisition instead of 5.0.
                        // Stopping at one decode instead, which is what
                        // this loop did before best-of-N, is the rule
                        // that cost minutes of dark band on hardware.
                        // Per-trial, because the summary below reports
                        // only the winner: the log that found this bug
                        // said "trial 3/5, 1 decoded" and could not say
                        // what 4 and 5 would have given.
                        log::info!(
                            "    acq trial {}/{}: centre={centre:+.3} decoded={} dt={dt:+.3}",
                            trial + 1,
                            phases.len(),
                            got.len(),
                        );
                        // Stop once a trial clears the lock bar — the
                        // measurement is in the note beside
                        // `ACQUIRE_MAX_TRIALS`. After the log line: the
                        // trial that ends the search is the one the log
                        // can least afford to be missing, and putting
                        // the break first silently dropped it
                        // (`logs/sim_slotfloor_*_2026-09-18.log` says
                        // "trial 3/5, 6 decoded" with no trial-3 line).
                        if got.len() >= mfsk_app_shared::grid_state::LOCK_MIN_DECODES {
                            break;
                        }
                    }
                    // An applied phase restarts the under-par run — the
                    // grid just moved, so the slots that led here say
                    // nothing about the new one. A run where nothing
                    // decoded leaves it standing, so the retry is
                    // immediate.
                    grid.acquisition_done(applied.is_some());
                    // Back to the QSO line; the link bar's `a` now says
                    // what the grid is.
                    if let Ok(mut ui) = UI.lock() {
                        ui.set_acq_line("");
                    }
                    match applied {
                        Some((dt, trial, n)) => {
                            let shift = (dt * 12_000.0).round() as i32;
                            mfsk_app_shared::time_sync::set_acquisition_shift_12k(shift);
                            // A fresh placement gets a fresh trim: the
                            // capture puts the grid within a cluster's
                            // width, and one pooled correction after it
                            // is what centres it. The sink's boot
                            // anchor is the other placement, and
                            // `trim_used` starts false for it.
                            trim_used = false;
                            mfsk_app_shared::time_sync::note_grid_lock(
                                mfsk_app_shared::time_sync::GridLock::Air,
                            );
                            // Not `observe_slot_phase(dt, ...)` — `dt` is
                            // the *pre*-correction residual, and the
                            // one-shot `shift` above is about to erase
                            // it from the grid. Feeding it to the EMA
                            // seeded the cross-slot filter with a value
                            // that no longer applied once the shift
                            // landed, so the filtered estimate spent
                            // several slots decaying off a stale number
                            // instead of tracking the (near-zero)
                            // post-correction residual — caught via
                            // MFSK_CORES3_SIM (filtered=+1.524s against
                            // a same-slot raw median of +0.601s, right
                            // after a cold-acquisition lock).  Clear the
                            // filter instead so it starts fresh from the
                            // next slot's post-correction observation.
                            mfsk_app_shared::time_sync::reset_slot_phase();
                            // Same reasoning — the pooled multi-slot
                            // ring is entirely pre-correction evidence
                            // at this point, so clear it too rather than
                            // let it drag a stale median into the first
                            // several post-correction slots.
                            mfsk_app_shared::time_sync::reset_dt_pool();
                            // Persist it so the reboot into FT4 mode
                            // ("QSY to FT4", #356) keeps the lock.
                            if let Some(now_ms) = mfsk_app_shared::time_sync::utc_now_ms() {
                                crate::uac::persist_grid_fix(mfsk_app_shared::grid_fix::GridFix {
                                    offset_us: (dt * 1_000_000.0).round() as i32,
                                    period_s: 15.0,
                                    epoch_at_fix: (now_ms / 1000) as i64,
                                    // The trial decoded `n` messages
                                    // at this phase, which is a far
                                    // better statement of confidence
                                    // than the `r` this used to carry.
                                    // Saturates at 1.0 by
                                    // `LOCK_MIN_DECODES`, so a phase
                                    // good enough to lock persists as
                                    // fully trusted.
                                    confidence: (n as f32
                                        / mfsk_app_shared::grid_state::LOCK_MIN_DECODES as f32)
                                        .min(1.0),
                                });
                            }
                            log::warn!(
                                "  cold-acquisition: grid phase {dt:+.2} s → shift {shift:+} samples \
                                 (trial {trial}/{}, {n} decoded)",
                                phases.len()
                            );
                            log::warn!(
                                "  cold-acquisition: capture began {start_s:.3} s into its slot \
                                 (counted in the phase)"
                            );
                        }
                        None => log::warn!(
                            "  cold-acquisition: none of {} candidate phases decoded — will retry",
                            phases.len()
                        ),
                    }
                }
            }
        }

        let mut had_response_this_slot = false;
        // The slot's rows, published together below through the list's
        // one entry point (`UiState::publish_slot`) — which rows count
        // as heard this slot is the panel's call, by time.
        let mut published: Vec<(String, f32, f32, f32, u32)> = Vec::new();
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                // Resolve against what earlier slots taught us, then
                // learn this message's own callsigns for the next.
                if let Some(text) =
                    mfsk_core::msg::wsjt77::unpack77_learn(r.message77(), &mut calls)
                {
                    rendered_total += 1;
                    // `<...>` is what `unpack77` emits for a hash the
                    // table could not resolve, so counting the
                    // rendered text is counting exactly the failures
                    // — no need to reach into the fields.
                    if text.contains("<...>") {
                        unresolved_total += 1;
                    }
                    const FP_SPEC_SHIFT: u32 = 12;
                    let cell_scale = (1u32 << FP_SPEC_SHIFT) as f32;
                    let calibrated_snr =
                        mfsk_core::ft8::decode_block::xsnr2_db_simple(&spec.spec, r, cell_scale);
                    let snr_i8 = calibrated_snr.round().clamp(-128.0, 127.0) as i8;
                    published.push((
                        text.clone(),
                        r.freq_hz,
                        calibrated_snr,
                        r.dt_sec,
                        r.hard_errors,
                    ));
                    log::info!(
                        // WSJT-X's order — dB, DT, Freq, message — so
                        // a log read beside its Band Activity window
                        // lines up. **Per-station DT is the point**:
                        // the slot median below cannot separate a grid
                        // that is off from a band whose clocks are
                        // loose, and those want opposite responses.
                        "{:+5.1}dB (raw={:+5.1}) {:+5.2}s {:4.0}Hz {}",
                        calibrated_snr,
                        r.snr_db,
                        r.dt_sec,
                        r.freq_hz,
                        text
                    );
                    qso.set_rx_snr(snr_i8);
                    let parity_lock_ok = mfsk_app_shared::parity::framing_settled_for_parity_lock();
                    if qso
                        .process_message(&text, wav_idx as u32, parity_lock_ok)
                        .is_some()
                    {
                        had_response_this_slot = true;
                    }
                }
            }
            // Every slot, decoded or not — an empty slot costs nothing
            // and keeps the list's own slot count honest.
            let rows: Vec<SlotDecode> = published
                .iter()
                .map(|(text, freq_hz, snr_db, dt_sec, hard)| SlotDecode {
                    freq_hz: *freq_hz,
                    snr_db: *snr_db,
                    dt_sec: *dt_sec,
                    text,
                    hard_errors: *hard,
                })
                .collect();
            crate::storage::publish_slot(
                &mut ui,
                mfsk_app_shared::boot_mode::BootMode::Uac,
                crate::storage::decoded_slot_unix(15_000),
                &rows,
            );
        }

        // **This period's transmission is decided here** — before
        // key-up, from what decoded before key-up. That is the bound's
        // whole purpose.
        if qso.state == QsoState::Idle {
            qso.call_cq(None);
        }
        // **One writer for the status strip.**
        //
        // An acquisition in flight has already put its own progress
        // there and cleared it on the way out, so this only speaks when
        // one is not running. The `AIR DT` line is the case that had no
        // display at all: the operator chose the air, the grid is
        // running on the clock's one-shot anchor, and nothing has
        // confirmed it yet — which since the RTC write was fixed is the
        // *normal* state and looks exactly like an ordinary receive.
        // Reported from the bench twice.
        if !grid.is_acquiring() {
            if source == "uac"
                && crate::grid_source() == mfsk_app_shared::grid_src::GridSource::AirDt
                && !grid.is_locked()
            {
                let mut l: heapless::String<32> = heapless::String::new();
                let _ = l.push_str("AIR DT: clock phase, unproven");
                if let Ok(mut ui) = UI.lock() {
                    ui.set_acq_line(l.as_str());
                }
            } else if let Ok(mut ui) = UI.lock() {
                ui.set_acq_line(strip.as_str());
            }
        }

        let intent = qso.next_tx();
        push_tx_line(&qso, intent.as_ref());

        if !had_response_this_slot && qso.state != QsoState::Idle {
            let _ = qso.on_period_end();
        }

        // **Last, so this slot's decodes carry this slot's number.**
        // It used to land mid-body, above the row push, which tagged
        // every row with the *next* slot's sequence — so a station
        // decoded in slot N was marked current through slot N+1. With
        // the watermark now doing real work, that off-by-one is the
        // difference between one green slot and two.
        slot_seq = slot_seq.wrapping_add(1);
    }
}

fn push_tx_line(qso: &QsoManager, intent: Option<&qso::TxIntent>) {
    let line = qso::format_tx_line(qso, intent);
    log::info!("[QSO] {}", line.as_str());
    if let Ok(mut ui) = UI.lock() {
        ui.set_tx_line(line.as_str());
    }
}

