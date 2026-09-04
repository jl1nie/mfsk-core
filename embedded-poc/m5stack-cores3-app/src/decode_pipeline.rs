//! WAV-fed decode pipeline for CoreS3 (Phase 0-Core).
//!
//! Structurally identical to `m5stack-core2-app/src/decode_pipeline.rs`.
//! All heavy lifting is in `embedded_shared` and `mfsk_app_shared` which
//! are board-agnostic. Board-specific touch points are `crate::log_free_internal`
//! and the `QSO_WAVS` slice (same qso3_busy.wav reference as Core2).
//! Phase 1-Core replaces the wav_sim source with a UAC host capture.

extern crate alloc;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{DEFAULT_Q_THRESH, NFFT_SPEC};
use mfsk_core::msg::wsjt77::unpack77;

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};
use esp_idf_svc::sys::QueueHandle_t;

use mfsk_app_shared::qso::{self, QsoManager, QsoState};
use mfsk_app_shared::ui::state::{DecodedRow, UI};

/// Operator identity, from `cfg.toml`'s `[station]` section.
///
/// These were literals until 2026-08-23 — this repository shipped one
/// operator's callsign compiled into the source, so anyone else's build
/// identified as them. `m5stack-s3-app` has taken them from `cfg.toml`
/// since Phase 1.7; this crate was never brought across.
///
/// Empty is a valid value and leaves the QSO FSM idle, which is the
/// right behaviour for a receiver with no operator configured.
const MY_CALL: &str = env!("MY_CALL");
const MY_GRID: &str = env!("MY_GRID");

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

/// Wall-clock budget for stage 3, milliseconds from the SpecBundle
/// arriving (#357). Bounds the worst-case slot so a dense period cannot
/// overrun and steal the next slot's headroom — the failure the live
/// radio showed (transmit-heavy period ~0.7 s past slot end, 8–11
/// candidates deferred and dropped, 0–2 decoded against the other
/// period's 4–8).
///
/// **2000 ms, from the qso3_busy sweep** (`logs/ft8_357_bud*`,
/// 2026-09-04). Stage 3 there measures ~985 ms; the recall-vs-budget
/// curve is flat at `dec=7` down to 1000 ms, still 7 at 800 ms (the
/// deadline sheds only the doomed tail — candidates run in descending
/// coarse score and the all-LLR-variant BP failures are last), then
/// 5 at 600 ms and 2–3 at 400 ms. 2000 ms is ~2× the measured work,
/// margin for a denser real band and the ~180 ms slow-period coarse,
/// while still bounding a pathological slot. `0` disables it;
/// `MFSK_FT8_BUDGET_MS=` overrides. Pending confirmation on a live
/// radio.
const FT8_BUDGET_MS: i64 = match option_env!("MFSK_FT8_BUDGET_MS") {
    Some(s) => parse_u32(s) as i64,
    None => 2_000,
};

/// `const`-context unsigned parse — `str::parse` is not `const`. Digits
/// only; anything else is a build-time panic, which is what you want
/// for a typo in a sweep env var that would otherwise silently fall
/// back to the default.
const fn parse_u32(s: &str) -> u32 {
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
    run_with_source("wav", |q| wav_sim::spawn(QSO_WAVS, q))
}

/// Source-agnostic entry. Allocates the pipeline queues, spawns
/// `stage1_inc` + `wf_drain`, calls `source_spawn` (which must push
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
    let wf_q = pipeline::create_wf_queue(8);
    stage1_inc::spawn_with_wf(chunk_q, slot_q, spec_q, Some(wf_q));
    source_spawn(chunk_q);

    let wf_q_addr = wf_q as usize;
    crate::board::spawn_named(c"wf_drain", 4 * 1024, move || {
        wf_drain(wf_q_addr as esp_idf_svc::sys::QueueHandle_t)
    })
    .expect("spawn wf drainer");

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, cores3-app phase 0)");

    let mut qso = QsoManager::new(MY_CALL, MY_GRID);
    let initial = qso.call_cq(None);
    push_tx_line(&qso, Some(&initial));

    let mut slot_seq: u32 = 0;
    // Air-sync lock strength (#356): the decode count of the slot the
    // current slot-shift estimate came from. A later slot only overrides
    // it with strictly more decodes, so one noisy decode cannot pull a
    // good lock off. Only used for the live (`uac`) source.
    let mut best_n: usize = 0;
    // Cold-acquisition state (#356b). `lost_slots` counts consecutive
    // slots with no clock, no decode and no coarse DT — i.e. the grid is
    // off past what the ±1 s search can even see. After
    // `ACQUIRE_TRIGGER_SLOTS` of that, arm `uac`'s capture ring and run
    // `ft8::acquire::acquire_slot_phase` on the 25 s it collects.
    let mut lost_slots: u32 = 0;
    let mut acquiring = false;
    /// Consecutive decode-less slots before a cold acquisition, from a
    /// standing start (nothing has ever locked).
    const ACQUIRE_TRIGGER_SLOTS: u32 = 3;
    /// The same count once a lock has produced decodes. Higher, because
    /// after a lock a run of empty slots is usually a quiet band rather
    /// than a lost grid, and re-acquiring costs 25 s of capture plus
    /// whatever decodes the wrong-until-then grid would have made. Six
    /// slots is 90 s of genuinely nothing heard.
    const REACQUIRE_TRIGGER_SLOTS: u32 = 6;
    /// Minimum mean-resultant confidence to act on an acquisition.
    const ACQUIRE_R_MIN: f32 = 0.55;
    /// Coarse-candidate cap for *each* of `acquire_slot_phase`'s three
    /// tiled windows. Deliberately **not** `MAX_CAND` (15, the decode
    /// candidate cap) — `MFSK_CORES3_SIM` caught reusing it here: with
    /// only 15 pass1 candidates per ±2.5 s window, `qso3_busy`'s real
    /// signals get crowded out of the top-15 as often as not, and the
    /// circular estimate came back at `R 0.39` (below `ACQUIRE_R_MIN`)
    /// on a 4 s offset the phase was otherwise recovered correctly for
    /// (`dt -3.6 s`). 200 matches `tests/ft8_cold_acquisition.rs`,
    /// where the measurement that chose the tiled search over the wide
    /// one used it. One-time cost — this only runs during acquisition,
    /// never in the per-slot decode loop.
    const ACQUIRE_MAX_CAND: usize = 200;
    loop {
        let cfg = dual_core::DecodeConfig {
            freq_min: 100.0,
            freq_max: 3_000.0,
            sync_min: 1.0,
            pass1_limit: PASS1_LIMIT,
            max_cand: MAX_CAND,
            q_thresh: DEFAULT_Q_THRESH,
            bp_max_iter: mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
            depth: DecodeDepth::EMBEDDED,
            budget_ms: FT8_BUDGET_MS,
        };
        let out = dual_core::run_speculative_slot(spec_q, slot_q, &cfg);
        let dual_core::SpeculativeOut {
            spec,
            slot,
            results,
            n_pass1,
            n_cut,
            n_ready,
            n_deferred,
            // Not read any more: the ±0.2 s/slot nudge it fed was a
            // random walk, not an acquisition (see the lock-and-hold
            // comment below). Acquisition is cold acquisition's job.
            bootstrap_dt_med: _,
            t_post_recv,
            t_coarse_done,
            t_early_done,
            t_slot_recv,
            t_done,
        } = out;
        let wav_idx = slot.wav_idx;

        let slotend = slot.slotend_us;
        let tail_window = (slotend - t_post_recv).max(0);
        let coarse_us = t_coarse_done - t_post_recv;
        let tail_use = (slotend.min(t_early_done) - t_coarse_done).max(0);
        let post_slotend = (t_done - slotend).max(0);
        // `slot_wait` near zero means the pipeline never got to wait for
        // the next slot — it was still working when the slot ended.
        //
        // On FT8 that is an operating limit rather than a fault: 15 s
        // is genuinely tight, `coarse` scales with how much signal is
        // on the band, and stations transmit in alternating periods, so
        // the busier of the two runs out of time and defers candidates.
        // Measured 2026-08-23 on 40 m: seven decodes on one period,
        // one or two on the other, with `coarse` at 101 ms against
        // 180 ms. Worth surfacing precisely because it is expected —
        // the alternative is reading it out of one field in seven.
        //
        // WSPR and FST4 are the other case: their monitor loops are
        // built with deliberate slack, so an over-budget slot there
        // means a fault. Do not carry this framing across.
        let slot_wait_us = t_slot_recv - t_early_done;
        // Only judge a full slot.
        //
        // The first slots after the grid anchors to UTC are partial by
        // construction — the anchor shortens the current one so the
        // next boundary lands on the grid — and comparing a truncated
        // slot against a whole slot's budget produces a warning about
        // nothing. Two of them fired within seconds of adding this,
        // which is exactly how an indicator teaches its reader to skip
        // it.
        const FULL_SLOT_SAMPLES: usize = 180_000;
        let full_slot = slot.audio().len() >= FULL_SLOT_SAMPLES;
        if full_slot && slot_wait_us < 10_000 {
            log::warn!(
                "SLOT[{wav_idx}] src={source} OVER BUDGET — no idle before the next slot \
                 ({post_slotend} us past slot end, {n_deferred} candidates deferred)"
            );
        }
        log::info!(
            "SLOT[{wav_idx}] src={source} grid={} p1={n_pass1} ready={n_ready} defer={n_deferred} \
             cut={n_cut} dec={} budget={FT8_BUDGET_MS}ms \
             tail_win={}us coarse={}us early={}us tail_use={}us post_slotend={}us \
             slot_wait={}us late={}us",
            mfsk_app_shared::time_sync::grid_lock().label(),
            results.len(),
            tail_window,
            coarse_us,
            t_early_done - t_coarse_done,
            tail_use,
            post_slotend,
            t_slot_recv - t_early_done,
            t_done - t_slot_recv,
        );
        slot_seq = slot_seq.wrapping_add(1);

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
            if slot_median.is_some() {
                // First confirmed decode: the grid demonstrably follows
                // the band. Record that and stop — `Ft8ChunkSink` will
                // raise this to `Ntp` if NTP ever lands.
                if best_n == 0 {
                    log::info!("  air-sync: grid locked on first confirmed decode (N={n_dec}) — holding");
                }
                mfsk_app_shared::time_sync::note_grid_lock(
                    mfsk_app_shared::time_sync::GridLock::Air,
                );
                best_n = best_n.max(n_dec);
            }
            // Keep the channel drained at 0 so nothing stale can reach
            // `Ft8ChunkSink` — the only writer left is cold acquisition,
            // through its own uncapped one-shot channel.
            mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(0);

            // Cold acquisition (#356b): the *only* thing that moves the
            // grid, and — under lock-and-hold — also the only way back
            // from a bad one. `bootstrap_dt_med.is_none()` was tried as
            // the trigger and does not work: `pass1` almost never comes
            // back empty on a real spectrum (`sync_min = 1.0` alone
            // clears noise peaks up to `PASS1_LIMIT`), so it is
            // essentially always `Some`, just a small near-random value
            // when the true signal is outside ±1 s — `MFSK_CORES3_SIM`
            // showed a deliberate 4 s offset never tripping it. A run
            // of slots decoding nothing is the real signal.
            //
            // The trigger deliberately does *not* look at `best_n`. It
            // did once, as `n_dec == 0 && best_n == 0`, and that made a
            // lock permanent: `best_n` only ever rises, so after the
            // first decode the condition could never be true again and
            // the receiver could never re-acquire — a mis-locked or
            // later-drifted grid would have stayed wrong until a
            // reboot. `ACQUIRE_R_MIN` is not a guarantee of a *correct*
            // phase, only a confident one (a 0.61 acceptance landed
            // 2.5 s out on 2026-09-05), so the way back matters as much
            // as the way in.
            //
            // Held longer than the cold-start count, because after a
            // lock the same run of empty slots is more often a quiet
            // band than a lost grid, and a needless re-acquire throws
            // away a grid that was working.
            let relock = best_n > 0;
            let trigger_slots = if relock {
                REACQUIRE_TRIGGER_SLOTS
            } else {
                ACQUIRE_TRIGGER_SLOTS
            };
            lost_slots = if n_dec == 0 { lost_slots + 1 } else { 0 };

            if lost_slots == trigger_slots && !acquiring {
                if relock {
                    // Drop the lock so the state matches reality while
                    // the capture runs, and so the panel stops claiming
                    // a grid the decoder can no longer demonstrate.
                    best_n = 0;
                    mfsk_app_shared::time_sync::note_grid_lock(
                        mfsk_app_shared::time_sync::GridLock::FreeRun,
                    );
                }
                acquiring = true;
                crate::uac::arm_acquisition();
                log::warn!(
                    "  cold-acquisition: {} {lost_slots} slots — capturing 25 s of FT8",
                    if relock {
                        "no decode since the lock for"
                    } else {
                        "grid lost"
                    }
                );
            }
            if acquiring {
                if let Some(audio) = crate::uac::take_acquisition_audio(
                    mfsk_core::ft8::acquire::REQUIRED_SAMPLES,
                ) {
                    acquiring = false;
                    lost_slots = 0;
                    match mfsk_core::ft8::acquire::acquire_slot_phase(
                        &audio,
                        100.0,
                        3_000.0,
                        1.0,
                        ACQUIRE_MAX_CAND,
                        5,
                    ) {
                        Some((dt, r)) if r >= ACQUIRE_R_MIN => {
                            let shift = (dt * 12_000.0).round() as i32;
                            mfsk_app_shared::time_sync::set_acquisition_shift_12k(shift);
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
                                    confidence: r,
                                });
                            }
                            log::warn!(
                                "  cold-acquisition: grid phase {dt:+.2} s (R {r:.2}) → shift {shift:+} samples"
                            );
                        }
                        Some((dt, r)) => log::warn!(
                            "  cold-acquisition: inconclusive (dt {dt:+.2} s, R {r:.2} < {ACQUIRE_R_MIN}) — will retry"
                        ),
                        None => {
                            log::warn!("  cold-acquisition: no candidates — will retry")
                        }
                    }
                }
            }
        }

        let mut had_response_this_slot = false;
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                if let Some(text) = unpack77(r.message77()) {
                    let mut msg: heapless::String<22> = heapless::String::new();
                    let take = text.len().min(msg.capacity());
                    let _ = msg.push_str(&text[..take]);
                    const FP_SPEC_SHIFT: u32 = 12;
                    let cell_scale = (1u32 << FP_SPEC_SHIFT) as f32;
                    let calibrated_snr =
                        mfsk_core::ft8::decode_block::xsnr2_db_simple(&spec.spec, r, cell_scale);
                    let snr_i8 = calibrated_snr.round().clamp(-128.0, 127.0) as i8;
                    let row = DecodedRow {
                        df_hz: r.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                        snr_db: snr_i8,
                        hard_errors: r.hard_errors.min(255) as u8,
                        msg,
                        slot_seq,
                        first_seq: slot_seq,
                    };
                    ui.push_decode(row);
                    log::info!(
                        "{:4.0}Hz {:+5.1}dB (raw={:+5.1}) {}",
                        r.freq_hz,
                        calibrated_snr,
                        r.snr_db,
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
        }

        if !had_response_this_slot && qso.state != QsoState::Idle {
            let _ = qso.on_period_end();
        }
        if qso.state == QsoState::Idle {
            qso.call_cq(None);
        }
        let intent = qso.next_tx();
        push_tx_line(&qso, intent.as_ref());
    }
}

fn push_tx_line(qso: &QsoManager, intent: Option<&qso::TxIntent>) {
    let line = qso::format_tx_line(qso, intent);
    log::info!("[QSO] {}", line.as_str());
    if let Ok(mut ui) = UI.lock() {
        ui.set_tx_line(line.as_str());
    }
}

fn wf_drain(wf_q: esp_idf_svc::sys::QueueHandle_t) -> ! {
    let n: u32 = 0;
    loop {
        let tick = pipeline::recv_box::<pipeline::WfTick>(wf_q);
        if let Ok(mut ui) = UI.lock() {
            ui.push_waterfall(tick.row);
        }
        let _ = n;
    }
}
