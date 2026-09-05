# Changelog

## 0.10.1 — ハムフェア2026 booth material, the 12 kHz literal classification (#323), release tooling that runs on macOS, FT4 embedded fits its slot

**Why a patch bump.** Additive public API (five new functions, no
signature changed and nothing removed), embedded-only performance work,
documentation and tooling. **No host decoder behaviour change**: every
numeric change below is behind `dotprod-extern`, and the paths that are
shared with host are pinned bit-identical by test. No measured
sensitivity movement. This section accumulates until the next tag — see
`CLAUDE.md`'s "Release cadence".

(An earlier revision of this paragraph said "no public API change".
That was true when it was written and stopped being true with the
streaming coarse stage below.)

### Added

- **The FST4 and WSPR screens stopped being two copies of one screen
  (issue #353).** `slot_list` (FST4) and `wspr_list`/`wspr_state`
  (WSPR) held the same container twice — latest-slot `Vec`, history
  `Deque`, `last_slot_hhmm`, the pre-truncation count that makes a
  truncated slot read as `16/23` rather than as a quiet band, and the
  `AtomicU32` the render loop polls — and the same three drawing
  helpers, with identical layout constants written out twice. Now
  `ui::spot_state::SpotState<Row, SLOT_CAP, HIST_CAP>` and
  `ui::spot_render` (`FormatRow`, `text_style`, `fill`, `render_rows`,
  the panel geometry). What stays per screen is what differs: region
  offsets, the header line, and the columns of a row.

  413 + 289 + 179 lines become 332 + 233 + 130, plus 282 shared — and
  the container is now `heapless` and one atomic with no draw stack, so
  `hosttest/mfsk-app-shared` compiles it and its truncation,
  history-rolling and dirty-seq rules have tests **that run**. They had
  none before, in either copy.

  Verified on the board: FT4 unaffected (11-12 candidates, 10-11
  decodes, 1 276-1 444 ms), FST4 and WSPR screens checked by eye
  through the mode picker.

  Not done here, deliberately: FT8/FT4's `run_log_panel` is a
  waterfall-and-decode-ring panel, not a spot list, and stays where it
  is (#353 has the reasoning).

- **The Δt search's carrier phasors are computed once per slot, and
  the 1-2 candidates §46 gave up are back.** `ft4_sync_search_window`
  spent 30 % of itself in `FlatRef::fill` — 18 calls per candidate,
  each with a `cos`/`sin` per sample — for references that depend only
  on `df`, `ds_spb` and `ds_rate`, so twelve candidates built the same
  nine references twelve times. New `Ft4CoarsePhasors` (~9 KB) holds
  the phasor tables; `ft4_sync_search_window_cached` takes them and is
  **bit-identical** to the uncached path, pinned by test on `i0`,
  `freq_hz` and `score` bit patterns. On the board: 10-11 candidates
  and 9-10 decodes become **11-12 and 10-11**.

  **Caching the references themselves (~144 KB) instead was a 2.6×
  regression**, and the reason is worth more than the fix: the
  *uncached* path in the same binary slowed from 87 to 223 ms, because
  144 KB displaced the per-call `FlatRef`s (~1 KB, and so
  internal-DRAM-resident under `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`)
  into PSRAM. The dots never left the PIE path. Before caching
  anything on this board, ask what it evicts.
  `docs/notes/FT4_BENCHMARK.md` §47.

- **FT4 searches WSJT-X's own ±1.0 s Δt window again, and the mixer
  stopped pushing one sample at a time.** The embedded receiver had
  narrowed the search to ±0.5 s (measured lossless on the golden, worth
  1.5-1.9× on the stage). The golden could not have said otherwise —
  its DTs span −0.44…+0.30 s, inside the narrow window by construction
  — and what the narrowing gives up is every station whose clock is off
  by more than half a second, which on a real band outnumbers the
  marginal-SNR stations the saved time buys. The errors add, too: this
  receiver's own slot alignment is still open (#313).

  Restoring it moves `CAPTURE_CLOSE_SAMPLES` to 6.775 s and the budget
  to 1 225 ms, and costs one to two of the weakest candidates on the
  14-signal golden (11 decodes → 9-10). The doubled search stage itself
  barely shows — it is a smaller share of a candidate than it was
  before the shared decimation and the second core, so §19's pricing no
  longer holds.

  Separately, `CandidateDdc`'s mixer wrote its output with `Vec::push`
  per input sample — 45 000 capacity-and-length tests per candidate for
  5 000 outputs. Writing through pre-sized slices instead is
  bit-identical and measured **63.1 → 57.4 ms per candidate in situ**,
  1 290-1 401 → 1 259-1 360 ms end to end. `docs/notes/FT4_BENCHMARK.md`
  §46, issue #352.

- **FT4's decode budget is now derived from key-up, and the receiver
  fits inside it with two cores and honestly-sized stacks.** Three
  changes that only make sense together, all measured on a CoreS3:

  **The budget was anchored to the wrong end of the slot.** The
  deadline had been 1 960 ms from the *slot boundary*; for a station
  that has to transmit, the deadline is key-up at 0.5 s into the next
  slot. That left a QSO-capable build **500 ms**, not 1 960. The
  capture window now closes at `ft4_rx::CAPTURE_CLOSE_SAMPLES`
  (6.25 s — everything `ft4_sync_search_window` can reach, plus the DDC
  chain's group delay) and the budget is `8.0 − 6.25 = 1 750 ms`. The
  1.25 s that used to be waited out is audio no candidate reads;
  `SlotAccum` discards it so the slot grid stays a 7.5 s grid. Measured
  lossless on the WSJT-X golden at every close point from 6.041 s up,
  with the periodogram averaged over the shorter span
  (`tests/ft4_early_close.rs`, new).

  **Dual-core is worth 1.40×, not the 1.17× that closed it.** §31.1's
  conclusion was correct on its evidence; the shared decimation changed
  the evidence by taking most of the per-candidate PSRAM streaming out
  of the loop. `decode_slot` now runs a worker pinned to core 1, with
  both cores taking candidates from a shared cursor rather than
  splitting the list (candidates are not equal cost, and a cursor makes
  the deadline per-core with no coordination). A stage pipeline was
  measured beside it and **lost** — 1.32× against a 1.70× ceiling —
  which disproves the premise it was built on, that the global FFT
  guard was the binding constraint.

  **The stacks were the memory problem.** With WiFi associated the slot
  went 1 387-1 585 → 1 789-1 969 ms, and stopping the radio after NTP
  changed nothing (`esp_wifi_stop` frees no buffers), because the cost
  was internal DRAM, not CPU: the largest free block fell to 31 744 B
  and the decoder's allocations went to PSRAM. `board::log_task_stacks`
  now reports every task's headroom rather than only the tight ones,
  and the decode tasks turned out to use **2 584 B and 4 536 B of their
  32 KB**. At 8 KB each the slot runs **1 290-1 401 ms with WiFi up**,
  faster than it ever ran without it, and all 12 candidates fit inside
  the 1 750 ms budget: **12 of 12 tried, 11 decodes**, against 11 of 12
  and 10 at the start of the day. `docs/notes/FT4_BENCHMARK.md`
  §43-§45.

- **One network bring-up for every receiver on the CoreS3
  (`crate::net`).** `wspr_app` and `fst4_app` each carried their own
  copy of associate → UDP log → NTP → HTTP config, and the copies had
  drifted in ways that were real: unbounded retry versus bounded
  campaigns, and `WIFI_PS_MIN_MODEM` on one but not the other. Those
  are now `Policy` and `power_save` parameters, each app keeping the
  behaviour it had. **FT4 gains the network it never had** — it is the
  reason this was consolidated rather than copied a third time, since
  a QSO needs absolute time and NTP is where it comes from. 324 lines
  net removed from the two apps.

- **`embedded-poc/scripts/capture.sh`** wraps `flash-monitor.sh` with
  the five things that went wrong repeatedly in one session: waiting
  for another capture to release the serial port, re-attaching a board
  that `usbipd` reports as attached while `lsusb` cannot see it, never
  overwriting an existing log, failing loudly when a capture produced
  no marker line, and naming the physical step (a short RST press, or
  a 2 s hold for an image that installs the USB host driver) instead of
  retrying into the same wall.

- **`ft4::ddc` gained a shared front end, and FT4 stopped cutting
  candidates on the board.** The per-candidate chain filtered all
  90 000 samples of the slot at 12 kHz — 61 ms of a candidate's ~96 —
  where the FFT path it replaced computed its wideband transform once
  and let every candidate read it. `NDOWN = 18` factors as `2 · 9`, so
  the first factor moves in front of the candidate loop: new
  `SlotDecimator` / `decimate_slot` (165 taps, ÷2, real input through
  `FirStage::push_block_real`) run once per slot, and new
  `CandidateDdc::new_half_rate` / `candidate_baseband_half` run the
  same chain in Hz at 6 kHz with 101 taps and ÷9.

  `SlotAccum` drives the decimator from the capture path alongside
  `Ft4SavgBuilder`, so the ÷2 is off the post-slot budget entirely —
  safe because the stage is **bit-identical whatever block sizes it is
  fed** (`slot_decimator_is_block_independent`, streamed through
  251/1/1 024/37/4 096-sample blocks).

  On a CoreS3 (`ft4-demo`, replayed golden, 1 960 ms transceiver
  budget): **2 067-2 118 ms → 1 998-2 000**, and the slot deadline
  stops firing — **12 of 12 candidates and 11 decodes, against 11 of
  12 and 10**. The decode it buys is `W7BOB KJ7G RR73` at −17 dB, the
  one §34 recorded the cut giving up. §37 argued that on this receiver
  milliseconds are decodes rather than headroom; this is that argument
  measured.

  Sensitivity re-established rather than assumed, since a third stage
  makes nothing bit-identical: equivalent noise bandwidth against the
  reference band is **+0.021 dB with the shared stage and +0.021 dB
  without**; `ft4_ddc_equivalence`'s new third arm decodes the same
  eleven golden messages at both depths with identical `i0`; and
  `shared_rejects_content_that_folds_into_the_band` pins the one new
  failure mode — content above the 3 kHz Nyquist folding into a
  candidate's band — at more than 50 dB down, which is why the corner
  is 2 800 Hz and the stage 165 taps. **Still owed**: the 560-file
  paired sweep, whose arm is written but whose corpus was not on the
  machine this was built on. `docs/notes/FT4_BENCHMARK.md` §42.

- **The streaming spectrogram measured too: `fixed-point` is
  0.63× f32 there (issue #349 step 2).** §36 timed the *batch*
  `compute_spectrogram` and said plainly that it settled nothing about
  `stage1_inc`, the incremental u16 builder a receiver actually runs
  during capture — where halving the bytes written per pair could
  still pay. It does not. `stage1_inc::compute_pair_into`'s body is
  now `pair_kernel_i16`, with `pair_kernel_f32` the same computation
  in the other scalar and `stage1_inc::scalar_ab` timing both over one
  slot on the same audio: **1 342 ms u16 against 852 ms f32**, 345 KB
  against 690 KB.

  Per row that is 7.29 ms u16 / 4.63 ms f32, against the batch
  builder's 7.86 / 8.49 — the inversion is the pair trick, which packs
  two rows into one transform. The f32 arm collects the whole of that
  saving and the u16 arm almost none, because the `sc16` 3840-point
  mixed-radix transform costs 14.6 ms against `fc32`'s 9.3 on this
  core. `stage1_inc` is **not** switched to f32: the cell type is not
  local to it (`coarse_sync`, `pass2` and the waterfall all take
  `&[u16]`) and the slot buffer would double. What changes is the
  reason to keep it — the 345 KB, not speed.
  `docs/notes/FT4_BENCHMARK.md` §41.

- **`fixed-point` no longer implies the Q11i16 LLR/BP hot loop
  (issue #349).** The scalar is its own feature, `fixed-point-llr`,
  **off by default** — a `fixed-point` build keeps the u16 spectrogram
  (702 → 351 KB, which is what the feature actually defends) and runs
  LLR/BP in `f32`. The coupling had never been measured; when it was,
  on the LX7 this exists for, the integer BP came out **0.85× f32**
  (22 813 vs 19 456 µs, same LLRs and `max_iter`) because the core has
  an f32 FPU and the saturating i16 helpers cost more than the narrower
  loads save.

  End-to-end on a CoreS3 (`ft8-bench`, `qso3_busy.wav`), stage 3
  (refine + LLR + BP + OSD) drops **3.2-3.9 %** across three re-rank
  widths and the whole slot **0.7 %** — 4.355 → 4.325 s at the ship
  width. That is well short of #349's own "18 % of a stage that is
  34 % of a slot": the 18 % is a kernel that in the pipeline exits
  early on a CRC hit rather than running to `max_iter`, inside a stage
  that is more than BP. Decodes unchanged — 7 on the board in every
  arm, and the same 12 golden messages on host across both features
  (`ft8_qso3_decode_set`, which now reports the LLR type separately
  from the spectrogram's).

  `m5stack-core2-app` pins `fixed-point-llr` on: it is an LX6, the
  measurement is an LX7, and its logs were all taken with the integer
  loop. Consumers on a target without an FPU should do the same.
  `docs/notes/FT4_BENCHMARK.md` §40.

- **FT4's embedded decode fits its slot at realistic band occupancy.**
  A day of hardware measurement on a CoreS3 took the FT4 ship
  configuration from **2.33× over its 1 960 ms post-slot budget to
  1.24×**, and with the coarse stage streamed during capture the whole
  of the 5-10 signal occupancy range §23 established as realistic is
  now inside it (0.55× and 0.96×; FT8 density is 1.28×). Decodes are
  unchanged throughout — 11 on the WSJT-X golden in every arm of every
  run, and the coarse stage's candidate list matches the host's to
  0.00 Hz and 0.00 %.

  What moved it, in order of size, each measured rather than projected
  (`docs/notes/FT4_BENCHMARK.md` §25-32):

  - the 2304-point transform's scratch into internal DRAM — the coarse
    stage's 54 KB working set does not fit the S3's ~32 KB data cache,
    so **coarse 1 290 → 758 ms**;
  - `dot_f32` reaching esp-dsp's PIE path at all. It needs both operands
    16-byte aligned and `len % 4 == 0`, silently taking a 2.3×-slower
    scalar body otherwise, and *nothing* in FT4's hot loops satisfied
    it: the DDC's 199- and 263-tap FIRs failed on length, and the
    coherent sync scorer failed on alignment for 78 % of its 284 688
    dots per slot. **DDC 1 848 → 1 166 ms, Δt search 960 → 700 ms**;
  - a real block-mode body for `FirStage::push_block`, which had
    documented itself as one since `wspr::ddc` needed it while its body
    stayed `push_one` in a loop — and which `ft4::ddc` had never called;
  - streaming the coarse periodogram, below, which takes another
    **754 ms out of the post-slot budget entirely**.

- **`engine::ft4_coarse::Ft4SavgBuilder` and `ft4_coarse_sync_from_savg`**
  — the coarse stage split so a receiver can accumulate its periodogram
  *during* capture and pay only the peak search after the slot, the
  shape `wspr_app` already ships (`ddc_loop` → `DDC_READY_IDX` →
  `scan_loop`). On hardware that is 761 ms → 6 ms of post-slot cost.
  Output is bit-identical to the all-at-once path at any block size,
  pinned across nine chunk sizes. The builder requires the slot length
  up front, which is not a convenience: `getcandidates4.f90` averages
  exactly `(nz − NFFT1)/NSTEP` rows, not every row that fits, and a
  greedy builder would average one row more and change the result. No
  embedded binary wires this to a capture path yet.

- **`engine::dsp::fft_mixed_2304::fft_2304_with_scratch` and
  `fft_mixed_3840::fft_3840_with_scratch`** — the same transforms with a
  caller-owned scratch buffer, so an embedded backend can place and
  align it deliberately instead of paying a `vec![…; N]` per call. The
  existing entry points are unchanged and now delegate.

- **`mfsk_app_shared::time_sync::samples_to_next_slot_12k_ms`** — the
  UTC slot-phase source in milliseconds. `samples_to_next_slot_12k`
  takes whole seconds and so cannot express FT4's 7.5 s slot; it now
  delegates. The modular arithmetic is factored into a private helper
  that the `mfsk-app-shared` host harness tests directly, since
  `utc_now_ms` reads `SystemTime::now()` with no seam. Groundwork for
  the FT4 receiver's slot-grid alignment (#354).

- **The CoreS3 FT8 receiver cold-acquires its slot grid from the air,
  and the lock survives the QSY to FT4 (#356b).** When there is no NTP
  and the grid is off far enough that coarse sync sees nothing — no
  decode and no `bootstrap_dt_med` for three slots running —
  `decode_pipeline` arms a 25 s capture ring in `uac.rs`, runs
  `ft8::acquire::acquire_slot_phase` on it, and if the result is
  confident (mean resultant ≥ 0.55) posts a one-shot uncapped slot
  shift through `time_sync::set_acquisition_shift_12k`. `Ft8ChunkSink`
  applies it by lengthening a single slot by up to a whole period, sets
  `GridLock::Air`, and hands back to the ±1 s tracking.

  The recovered phase is persisted through the new
  `mfsk_app_shared::grid_fix` — an NVS record in the `"mfsk"` namespace,
  deliberately not the RTC (whose seconds-only write granularity would
  discard the sub-second phase, and whose "only NTP writes it back"
  invariant, #354, is worth keeping absolute). `apps/ft4.rs` reads it
  at boot, `GridFix::correction_for` re-wraps the 15 s FT8 phase to
  FT4's 7.5 s grid and refuses a fix older than 2 h or below the
  confidence bar, and the first anchor folds it in — so "lock on FT8,
  QSY to FT4" works across the reboot the mode switch is.

  Host-reasoned; the acquisition primitive and the `grid_fix` re-wrap
  are unit-tested but the on-device path has not been run against a
  radio.

- **`mfsk_core::ft8::acquire::acquire_slot_phase`** — cold slot-phase
  acquisition from ~25 s of off-air FT8 (#356b). Three ±2.5 s
  `coarse_sync_with_lag` searches at 0 / 5 / 10 s, each window's offset
  folded into the candidate `dt_sec`, then `circular_dt_estimate` over
  the union; returns `(dt_sec ∈ (−7.5, 7.5], r)`. `tests/ft8_cold_
  acquisition.rs` is the measurement that chose it over a single wide
  search: on `qso3_busy.wav` swept across the full period tiled recovers
  every offset to within ~115 ms (an FT8 symbol is 160 ms), the wide
  search is off by up to 720 ms with a misleadingly high `R` — the
  far-lag ghosts of issue #280, spread across a ±6.28 s window (all a
  single 15 s spectrogram can reach), poison the estimate.

- **`mfsk_core::engine::sync::circular_dt_estimate`** — a score-weighted
  circular DT estimate for a grid whose phase error can span a whole
  slot, with a mean-resultant-length confidence. `bootstrap_dt_median`
  sorts `dt_sec` linearly and is right only inside one ±2.5 s window;
  once a cold-acquisition search covers a full period, `dt` is an angle
  and `+7.4 s`/`−7.4 s` must not average to zero. First piece of #356's
  air-phase lock (the tracking half — `bootstrap_dt_median` — already
  shipped).

- **`mfsk_app_shared::time_sync::GridLock`** (`FreeRun` / `Rtc` / `Air`
  / `Ntp`) — what the slot grid's *phase* is anchored to, a distinct
  question from `ClockSource` (what set the system *clock*): with no
  network the grid can be `Air`-locked while the clock is only `Rtc`.
  `note_grid_lock` / `grid_lock` / `grid_is_locked` (the last is `Air`
  or `Ntp` — `Rtc` alone can be seconds out, past the ±1 s coarse
  search). The CoreS3 app now sets it at every anchor point (`uac.rs`,
  `apps/ft4.rs`, `apps/fst4.rs`), shows it as one char on the link bar
  (`T` / `a` / `r` / `-`), and puts `grid=` on every per-slot log line
  — the issue-#356 rule that an operator never guesses what the grid
  follows.

- **`time_sync::observe_slot_phase` / `filtered_slot_phase`** — a
  circular EMA (α 0.4, ~4 slots to a step then holds) across per-slot
  DT medians, wrapping at ±½ period. The "tracking across slots" half
  of #356 that `MEDIAN_DT_US` never had; the CoreS3 FT8 and FT4 paths
  feed it their slot medians.

- **`dual_core::run_speculative_slot` takes an optional stage-3
  wall-clock deadline (#357).** It had none: a slow slot ran every
  committed BP/OSD candidate to completion however long that took, and
  the overrun stole the next slot's headroom (on a busy 40 m band the
  transmit-heavy period overran ~0.7 s, deferred 8–11 candidates it
  dropped, and decoded 0–2). `DecodeConfig::budget_ms` (0 = off,
  unchanged behaviour) derives an absolute deadline from the
  SpecBundle-arrival time; `stage3_split`'s work-stealing loop stops
  claiming candidates once it passes, and `SpeculativeOut::n_cut`
  reports how many were left. `coarse_sync` and `pass2` are not
  bounded — stage 3 is where the variance is. The CoreS3 FT8 decode
  now defaults to **2000 ms**, from the qso3_busy sweep: stage 3
  measures ~985 ms there and the recall-vs-budget curve is flat at
  `dec=7` down to 800 ms — the deadline sheds only the doomed tail
  (candidates run in descending coarse score, the all-LLR-variant BP
  failures last) — so 2000 ms is ~2× margin at zero measured recall
  cost. `MFSK_FT8_BUDGET_MS=` overrides; the S3 / Core2 / wav-sim
  callers pass 0. The embedded FT8 decode is single-pass with no OSD,
  no SIC and no AP already, so this is the only per-slot cost bound
  it was missing.

- **The CoreS3 FT8 receiver self-aligns its slot grid from the air
  when there is no clock (#356).** `Ft8ChunkSink` anchors the grid to
  UTC once NTP has run; on a hilltop with no network it never does, and
  the grid free-ran at a phase uniform over 15 s. `decode_pipeline` now
  keeps `run_speculative_slot`'s `bootstrap_dt_med` (the top-5
  coarse-candidate DT median, valid before any decode) instead of
  discarding it, tracks a confirmed-decode lock afterwards with a
  Tier-1 drift reset, and — while `!clock_is_disciplined()` — posts the
  correction through `set_bootstrap_slot_shift_12k`. `Ft8ChunkSink`
  gained a `slot_target`; it still takes a rough one-time anchor from a
  plausible RTC, but hands the phase to its UTC drift check only once
  NTP has disciplined the clock, and rides the air-sync shift until
  then. Once NTP is up the whole air-sync path — the median read, the
  shift maths, the log — is skipped rather than computing a correction
  that would only be drained, so it adds nothing to the steady-state
  slot. The `wav` source is never touched. Behaviour change for a
  board that never gets NTP: its FT8 grid now rides the air rather than
  a possibly-wrong free-running RTC. Still not covered: a grid more
  than ~1 s out at the first slot, since coarse sync searches only
  ±1 s. Host-reasoned; not yet run against a radio.

- **The CoreS3 FT4 boot mode anchors its slot grid to UTC (#354).**
  Until now the grid was counted from whenever the USB audio stream
  started, so which 7.5 s window it captured was an accident — hidden
  entirely by the baked replay, which is a whole slot the decoder finds
  its own `dt` in. `embedded_shared::apps::ft4_rx::SlotAccum` (which
  owns the boundary, unlike the FT8 path where `Ft8ChunkSink` does)
  gains `anchor_or_reanchor` / `shift_next_window` / `is_aligned`;
  `apps/ft4.rs` drives them — the board half owns the clock
  (`time_sync`), the shared half only moves the grid when told. The
  first block of live audio anchors the next window to the next UTC
  boundary; a phase error past 100 ms re-anchors (the NTP step on an
  RTC-seeded clock can be seconds, past what the ±1.0 s DT search could
  pull back); the median DT of each slot's decodes then trims the
  residual and settles to zero. Not covered: a cold start with no clock
  and no decodes — FT4's coarse stage returns `dt = 0`, so there is no
  `bootstrap_dt_median` to pull the grid up from an arbitrary phase.
  That is #356. Host-reasoned; not yet confirmed against a radio.
  `docs/reference/EMBEDDED.md` "FT4 on embedded".

- **`.githooks/pre-push`**, and the FT4 embedded measurement harness:
  PIE-path counters in the esp-dsp dot-product and FFT backends, a
  layer split (kernel / staging / combine) for the mixed-radix
  transforms, and `ft4-bench` passes for the coarse stage, both DDC
  front ends, and a dual-core feasibility probe.

### Changed

- **The CoreS3 FT8 receiver locks its slot grid once and holds it,
  instead of steering it every slot (#356).** The grid is set by cold
  acquisition, or by NTP/RTC, and then left alone. The per-slot
  air-sync servo that used to trim it from the decode-DT median is
  gone, and so is `bootstrap_dt_median`'s ±0.2 s/slot nudge.

  A decode's DT is *that station's* clock error — WSJT-X reports it and
  never feeds it back into its own capture window. Which stations
  decode changes from slot to slot, so the median moves with the
  station mix, and because fading correlates over tens of seconds the
  same biased subset persists for several slots at a time, which is
  indistinguishable from a real error on the same timescale. Meanwhile
  there is nothing to track: the board's oscillator is ~3 ppm, i.e.
  45 µs per slot and 0.26 s per *day*, against a search window measured
  in seconds.

  Measured on hardware through `MFSK_CORES3_SIM`, where each attempt at
  a better servo was worse than not having one: the raw per-slot median
  oscillated the grid to ±0.6 s; pooling raw per-decode DTs across
  slots and EMA-filtering them still wandered to −0.74 s over six
  consecutive slots with `dec` falling 8 → 4; holding sat at −0.12 to
  −0.20 s all run, a spread of 0.08 s, with zero shifts applied. Most
  of the swing the servo was correcting had been its own motion. The
  leftover residual is cold acquisition's own accuracy, and the search
  window absorbs it — that is what the tolerance is for.

  Holding also repaired acquisition, which the nudging had been
  corrupting by moving the grid *during* the 25 s capture: `R` went
  0.63–0.84 before and 0.99 after, and the first locked slot decoded 7,
  the `qso3_busy` reference, where every earlier attempt started at 8
  and decayed.

  Re-acquisition no longer keys off `best_n`. It was gated on
  `n_dec == 0 && best_n == 0`, and `best_n` only rises, so a lock was
  permanent — a mis-locked grid stayed wrong until a reboot.

- **`ft8::acquire::acquire_slot_phase` reduces its candidates with a
  circular *medoid*, over two of them rather than five (#358).**
  `circular_dt_estimate` is a score-weighted circular mean, and a real
  band supplies the outlier that defeats one: on `qso3_busy` fifteen
  stations cluster at a median DT of +0.260 s while F5RXL sits at
  −0.770 s, loud enough to take the answer. `circular_dt_medoid` picks
  the candidate whose total circular distance to the others is least,
  so a lone far-off station is outvoted however strong it is; `r` keeps
  its definition, about the chosen centre.

  Measured on the numeric path the receiver ships (40 offsets across
  the period, scored by whether the acquired phase decodes), **and with
  the search window the receiver uses** — `MFSK_SYNC_LAG_S=1.0`, not
  the 2.5 s crate default, which forgives a phase two seconds out and
  so cannot see this at all: **307 → 345 decodes**, against ~600 for a
  perfect grid. More candidates is monotonically worse — 358 at
  `top_k=1`, 240 at 12 — because the leaders behind the first are
  artefacts rather than more stations. `1` scores best and is not
  taken: with one candidate `r` is identically 1.00, which would leave
  `ACQUIRE_R_MIN` accepting everything while looking like a gate.

  **What this does not fix**: the number of offsets acquiring a phase
  that decodes *nothing* is 16 of 40 at `top_k=2` against 17 at 5. The
  decode count improves by 12 %; the outright-failure rate does not
  move. (An earlier revision of this entry quoted 392 → 458 decodes and
  14 → 7 failures. Those were scored at the 2.5 s default and overstate
  what the board sees on both counts.)

  On f32 it is a wash (303 → 297 decodes, 0 → 0 failures).

- **One decode is not a lock (#356).** The first cut of lock-and-hold
  set `best_n` on the first confirmed decode, and the re-acquisition
  trigger was `n_dec == 0`; since `best_n` only rises, the trigger could
  then never fire again. A grid a full second out still decodes the odd
  station — measured: `F5RXL` and only `F5RXL`, 13 slots running, at a
  1.2 s offset where the same audio aligned gives 8 — so the receiver
  sat at 1-of-8 indefinitely with nothing able to correct it. The grid
  now locks only once a slot clears `LOCK_MIN_DECODES` (3), and below
  that the slot counts toward acquisition instead. The same 1.2 s
  offset now recovers 1 → 5.

  The policy moved to `mfsk_app_shared::grid_state`, host-tested: it is
  a pure function of the per-slot decode count, and both bugs it guards
  against are reachable from a handful of counts.

- **`mfsk-core/ft4` now has a complete embedded path.** `ft4-bench`
  runs a whole FT4 slot on a CoreS3 from nothing but 12 kHz audio: the
  coarse stage computes its own candidates through
  `engine::dsp::fft_mixed_2304`, and `ft4::ddc` replaces the
  92 160-point wideband transform that no embedded backend can serve —
  the DDC arm passes an **empty** `fft_cache` and decodes the same 11
  messages, since FT4's `snr_db` is a closed form over the coarse
  candidate score and never reads the spectrum.

- **`engine::dsp::fir_decimate::FirStage`** keeps its history in a
  16-byte-aligned store and carries one zero-padded tap table per
  window phase; **`engine::sync2d`'s `FlatRef`** does the same with two
  phases, and `AlignedCd0` aligns the caller's `cd0` once per search.
  All of it is behind `dotprod-extern`; a build with no backend keeps
  the exact arithmetic it had.

### Changed

- **The CoreS3 app's FST4 and WSPR receiver modes are behind default-off
  cargo features.** Each is a whole receiver — 1 382 and 1 815 lines
  with its own tasks, screens, slot grids and worker stacks — and pulls
  its decoder in behind it, while one image can only boot into one
  mode. `--features fst4` / `--features wspr` bring them back, and the
  four bins that need one carry `required-features` so a plain `cargo
  build --bins` skips rather than fails. Default ELF 2 770 968 B against
  3 412 112 with both, i.e. **626 KB**. `boot_mode` keeps its `Wspr` and
  `Fst4` variants either way, since an NVS setting outlives a reflash:
  `main` names the missing mode and continues as an FT8 controller
  rather than appearing to ignore it.

### Fixed

- **Every #358 measurement was taken in f32; the receiver is a
  `fixed-point` build.** `ft8_cold_acquisition.rs` is gated
  `not(feature = "fixed-point")`, so the sweep could not have seen the
  difference — and #280 had already recorded that the two numeric paths
  diverge for exactly this ghost mechanism. Same fixture, same offsets,
  scored by whether the acquired phase decodes: **f32 303 decodes and
  0/20 total failures, `fixed-point` 194 and 7/20**. `r` marks none of
  them (0.73–0.97 against successes at 0.79–0.98), which is the
  hardware behaviour exactly — the board answered −5.75 s at R 0.93 for
  a grid 1.2 s out. New `tests/ft8_cold_acquisition_fixed.rs` runs the
  measurement where the receiver lives.

- **A stray `eprintln!` in `compute_spectrogram`'s fixed-point path**,
  unconditional under `std`, once per spectrogram. Compiled out on
  `no_std` so hardware never saw it, but it spammed every host
  `fixed-point` run — the mode `CLAUDE.md` asks for when touching the
  decode pipeline, so the debris discouraged the practice it belongs to.

- **Two `coarse_sync` unit tests failed for the whole `full,fixed-point`
  matrix**, on `main`, by hardcoding the quarter-symbol grid's 372 time
  rows. `fixed-point` implies `nstep-half`, where a slot is 184 rows and
  `NSSY` is 2 — so the arithmetic they cover had *no* coverage at all on
  the grid every embedded build ships. Expectations now derive from
  `NSSY`/`COSTAS_POS` and hold on both. (A third failure in that matrix
  is a real recall gap and is #359.)

- **`tests/ft8_cold_acquisition.rs` swept whole seconds, and whole
  seconds are where cold acquisition happens to work (#358).** Stepping
  0.25 s instead of 1 s shows 51 of 60 offsets out by 0.2–1.6 s, with
  `r` at 0.79–0.98 while they are wrong — so `ACQUIRE_R_MIN`, the gate
  callers rely on, accepts them. On hardware at the worst case
  (`MFSK_SIM_OFFSET_MS=7500`, half a period out, the largest error a
  free-running grid can have) acquisition returned −6.02 s at R 0.62,
  then −6.84 s at R 0.72, both applied, never decoding; at 4000 ms the
  same code locks at R 0.99 and decodes 7/slot. The errors repeat
  exactly every 5 s, the tile spacing. The assertion is now a ratchet on
  the measured worst case rather than the 0.16 s target, so #358's fix
  can be seen to move it. **Whole-second phases are a measure-zero slice
  of the RTC-less case this path exists for**, which is why a sweep that
  only sampled them reported a clean bill of health for years.

- **The CoreS3 sim feed looped on the raw file length, sliding the audio
  under a stationary grid.** `qso3_busy.wav` is 180 101 samples — 101
  past one 15 s slot — so every loop moved the content 8.4 ms and
  spliced a discontinuity into whichever frame straddled the seam. With
  the grid held perfectly still the measured median DT still crept
  +8.2 ms/slot against 8.42 predicted, and `dec` wobbled 4–7 as the
  window crossed the splice: a harness artifact that reads as receiver
  drift, and was nearly diagnosed as one. `ready`/`defer` had been
  27/3, 26/4, 29/1, 28/2 on consecutive slots of supposedly identical
  audio; truncating the loop to whole slots makes it 25/5 on every one.

- **The mixed-radix FFT wrappers leaked their scratch, once per plan.**
  `MixedRadix2304Fft`/`MixedRadix3840Fft` each allocated an
  internal-DRAM workspace in `new` and never freed it, so every
  `plan_forward` cost another 18 or 30 KB. Two planners in one binary
  was enough to shift the heap and cost `engine::sync2d`'s dot products
  their 16-byte alignment — 100 % of them on esp-dsp's PIE path became
  0 %, and FT4's Δt search went 1 049 → 1 789 ms, from a change that
  touched neither. There is now one process-global block, sized for the
  longest user and shared under the `Fc32Guard` every mixed-radix
  transform already holds for its whole body, reserved from `main`
  before WiFi starts.

- **`scripts/pre-push-check.sh` never ran as a hook.** It installed
  itself into `.git/hooks/pre-push`, while README.md and
  CONTRIBUTING.md tell you to set `core.hooksPath .githooks` — which
  makes git ignore `$GIT_DIR/hooks` entirely. Following the documented
  setup therefore disabled the 15-combo feature matrix silently, and
  README/CONTRIBUTING never mentioned the installer at all, so
  contributors never had that gate. The hook now lives in `.githooks/`
  where the same one-line config finds it, and `install-hooks.sh` is a
  wrapper for that line.

- **Two warnings in `embedded-poc/embedded-shared`** that nothing was
  gating — that tree is outside the host workspace, so neither the
  pre-commit hook nor CI ever compiles it.

- **Four more places calling the FT4 golden a real off-air recording.**
  `000000_000002.wav` is `ft4sim_mult` output; the previous pass caught
  three files and missed these.

### Added

- **FT4 runs on hardware for the first time, and does not yet fit.**
  `docs/reference/EMBEDDED.md` said FT4 had "no embedded path at all
  yet"; no embedded crate enabled `mfsk-core/ft4`, and neither
  `scripts/pre-push-check.sh` nor CI carried an `alloc ft4 fft-extern`
  rung, so the `ft4 = []` feature's long-standing claim that FT4 is
  backend-agnostic had never been tested. It was very nearly true: the
  first build failed on one missing `use alloc::vec::Vec;` in
  `ft4/subtract.rs`, the same latent gap issue #306 found twice in
  FST4. Both feature matrices now carry the rung.

  Getting a candidate decoded on an ESP32-S3 needed one new FFT:
  `engine::dsp::fft_mixed_5120` (Cooley-Tukey 1024 × 5, reusing the
  existing `fft_15::fft_5` kernel and shaped exactly like
  `fft_mixed_3840`'s 256 × 15) serves `downsample_cached`'s
  per-candidate `fft2_size = 5120` inverse transform, which is not a
  power of two and so had no radix-2 kernel. The once-per-slot
  `fft1_size = 92_160` transform is baked on the host and fed through
  `decode_frame`'s existing `precomputed_fft` seam, as FST4 does.

  New `ft4-bench` bin on the CoreS3 measures the per-candidate work
  against the 1.96 s an FT4 slot leaves after the frame ends.
  **Measured 2026-08-29, 31 candidates from the WSJT-X golden: 17 339
  ms, 8.8× over budget, with 11 decodes matching the host exactly at
  both `DecodeDepth::EMBEDDED` and `FULL`.** `ft4_sync_search` is 76 %
  of it (13 225 ms) at a 0.2 % spread across candidates — a fixed
  ~19 900-cell grid, not anything candidate-dependent. Memory was never
  the constraint: 7.47 MB PSRAM free throughout. Full account in
  `docs/reference/EMBEDDED.md` "FT4 on embedded" and
  `docs/notes/FT4_BENCHMARK.md` §17.

- **What narrowing FT4's Δt search window costs: nothing, up to
  ±0.5 s.** `ft4_sync_search`'s cost is set by its Δt window alone, and
  the production window turns out to be ±1.0 s rather than the full
  slot it reads as (`i0` counts downsampled samples; `dt = 0` sits at
  `i0 = 333`). WSJT-X searches that wide because it cannot assume a
  clock; a UTC-anchored receiver can. Two new host diagnostics measure
  the trade: `ft4_diag_sync_window_recall`
  (`tests/ft4_wsjtx_samples.rs`) on the real off-air golden, and
  `ft4_diag_dt_window_reach` (`tests/ft4_sweep.rs`) on a DT-swept
  `ft4sim` corpus.

  **±0.5 s keeps all 11 golden decodes** — whose true DTs span
  −0.44 … +0.30 s — at a measured **1.91×** on the search, and the DT
  sweep shows a hard cliff exactly at the window edge: 100 % inside,
  0 % outside, with recall *identical* column-to-column near threshold
  wherever the DT is inside. Narrowing costs reach, not sensitivity.
  Applied to the device numbers that is 8.8× over budget → 5.7× over:
  necessary, not sufficient.

  The DT sweep deliberately does not use the tier-C corpus:
  `gen_ft4_sweep_wavs.sh` fixes `DT=0.0`, so every signal in it sits
  dead centre of every window under test and it would have reported no
  loss at any width — a property of the fixture, not the decoder. See
  `docs/notes/FT4_BENCHMARK.md` §18.

- **`mfsk_core::ft4::ddc` — FT4's per-candidate baseband without the
  92 160-point FFT.** `downsample_cached` is the stage that keeps FT4
  off a board: its `fft1_size = 92_160` forward transform is neither a
  power of two nor within an order of magnitude of ESP-DSP's 8192 limit,
  so the `ft4-bench` numbers above depend on a 737 KB FFT cache baked on
  a host. The new module builds the same `cd0` by mixing and filtering
  instead — two `FirStage`s and two mixers, no transform at all.

  FT4 is the easy case, and this is why it was worth doing before
  finishing `fst4::ddc`'s harder one: `NDOWN = 18` divides 12 kHz
  exactly, `666.667 Hz` is already `SyncDims::ds_rate`, and
  `ds_spb = 32` is a power of two, so no rational resampler and no
  `RxGrid` are needed and nothing downstream of `cd0` changes.

  The non-obvious constraint is the passband. `downsample_cached` keeps
  `[f0 − 31.25, f0 + 93.75] Hz` and zeroes the rest — asymmetric about
  `f0` — and `process_candidate_basic_impl` RMS-normalises `cd0` over
  its whole length against a `LLR_SCALE` calibrated on that band, so a
  wider filter rescales every LLR feeding BP (the full ±333 Hz baseband
  would have been ~2.3× high). The chain therefore centres the *band*,
  filters symmetrically with real taps, and rotates `f0` back to DC.
  Measured equivalent noise bandwidth against the reference:
  **+0.021 dB**.

  Verified twice, on host. On the WSJT-X golden from the same 31
  candidates: **11 distinct decodes on both front ends, identical
  sets**, at both `DecodeDepth::EMBEDDED` and `FULL`, with the refined
  sync position unmoved and one candidate of eleven landing one 1 Hz
  `ft4_sync_search` grid step away. On a new tier-C paired sweep over
  560 files spanning four channels' 50% crossings: **FFT 237 decodes,
  DDC 238**, five disagreements split three/two — the swap costs
  0.0 dB. Tests: `tests/ft4_ddc_equivalence.rs`.

  Not yet measured on hardware, not bound to esp-dsp's FIR
  (`dsps_fird_f32_aes3`), and — like `fst4::ddc` — not wired into
  `decode_frame`: a building block callers reach for, not a feature
  flag that swaps the host's front end. `ft4_coarse_sync`'s
  `NFFT1 = 2304` is still baked; a `fft_mixed_2304` (256 × 9) would
  close that without any DDC. See `docs/notes/FT4_BENCHMARK.md` §20.

- **`engine::dsp::fft_mixed_2304` — the last non-power-of-two length in
  FT4's path.** `ft4_coarse_sync` runs a Nuttall-windowed
  `NFFT1 = 4·NSPS = 2304`-point transform over every one-symbol step of
  the slot (~152 per slot), and 2304 = 2⁸·3² has no radix-2 kernel, so
  the CoreS3 bench shipped a candidate *list* baked on a host. Third
  member of the `fft_mixed_3840` / `fft_mixed_5120` family and built the
  same way — 256 × 9 Cooley-Tukey, the power-of-two factor served by the
  external backend, the 9-point factor a further 3 × 3 over the existing
  `fft_15::fft_3`. `EspDspPlanner` wires both directions.

  Verified against rustfft (the 9-point kernel alone, then the full
  transform forward and inverse on impulse and pseudorandom input) plus
  a single-tone test across seven bins spanning both index dimensions —
  the impulse test cannot see a transposition error and a tone can.
  Nothing has run it on hardware; the bench still reads the baked list.

- **FT4's candidate budget, measured for the first time — and the search
  parameter was never WSJT-X's.** Every stage after `ft4_coarse_sync`
  is per-candidate, so the candidate count multiplies every other lever;
  nothing had measured it. `getcandidates4.f90` baseline-normalises the
  smoothed spectrum, which puts noise at ~1.0, so the bench's
  `sync_min = 0.05` (and the sweep harness's 0.8) admitted **every peak
  in the band**. Upstream passes `syncmin = 1.2`
  (`ft4_decode.f90:195`).

  Measured across 560 sweep files straddling four channels' 50%
  crossings, plus the WSJT-X golden: 0.05 → 1.2 cuts the candidate count
  from 67.1 to 1.6 (sweep) and 31 to 12 (golden) **with identical recall
  on both**. The knee is at 1.4 (−2 of 237) — the faithful value is
  where the cliff isn't. `max_cand` is bounded by the golden's deepest
  decoding candidate, rank 11 of 31; the sweep corpus cannot answer that
  question at all, since one signal per file puts every decode at rank 0.

  `bench_assets::SYNC_MIN` is now 1.2 and
  `embedded-poc/assets/ft4_golden_candidates.bin` was re-baked: 31
  candidates → 12, same 11 decodes at both `DecodeDepth::EMBEDDED` and
  `FULL`. **Device numbers in `docs/notes/FT4_BENCHMARK.md` §17-19 were
  all measured over 31 candidates**; per-candidate figures carry over,
  slot totals do not. New test: `tests/ft4_candidate_budget.rs`, one
  ratchet on the ranking and one tier-C measurement of both curves.

  The same pass corrected a claim in the other direction: the bench's
  "`EMBEDDED` and `FULL` decode identically" holds on the golden and
  **not** on weak data — 237 vs 179 of 560 at the crossing, so skipping
  OSD costs about a quarter of the recall an embedded receiver would
  otherwise have. See `docs/notes/FT4_BENCHMARK.md` §21.

### Added

- **A band-occupancy corpus for FT4, and the design point it settles.**
  Every FT4 measurement in this crate had been made on either one
  recording or a corpus with exactly one signal per slot at `DT = 0.0`.
  `scripts/build_ft4sim.sh` now also links WSJT-X's own multi-signal
  simulator (`lib/ft4/ft4sim_mult.f90`) and `scripts/gen_ft4_mult_wavs.sh`
  builds a corpus from it: 5 occupancies × 50 slots × (5/10/14/20/30)
  signals, each at its own SNR and frequency and a random DT in ±0.5 s,
  with a `manifest.tsv` of ground truth — 3 950 signals in 45 MB. New
  test: `tests/ft4_crowded_band.rs`.

  There is no design point to target, because **nobody has measured FT4
  band occupancy** — not this repo and not upstream (see the next
  entry). The corpus brackets it instead: 5-10 signals is the plausible
  ordinary case for FT4, 14 is FT8's 40 m density rendered as FT4 and so
  a pessimistic bound, and 20-30 is an upper bound with no evidence that
  FT4 reaches it.

  - **Candidate count grows sub-linearly with occupancy**: 5.3 / 9.2 /
    12.3 / 14.8 / 17.3 for 5 / 10 / 14 / 20 / 30 signals, because
    `sync_min = 1.2` is a threshold on a baseline-normalised spectrum
    rather than a cap. The golden's 12 candidates — the number the
    embedded budget projection is built on — is exactly what a
    14-signal band produces, so the projection holds where it matters
    and degrades to 1.1× over at contest density.
  - **`max_cand = 12` must not be applied as a setting.** Decodes reach
    rank 15 at the design point and 22 under stress.
  - **What occupancy costs is interference, not sensitivity.** Recall
    single-pass against `sic_rounds(2)`: 87→96 % (5 signals), 74→88 %
    (10), 68→81 % (14), 56→72 % (20), 42→59 % (30). At the occupancies
    FT4 plausibly sees, a board with no subtract path — which
    `dual_core` is — reports 4.4 of 5 or 7.4 of 10 stations where a full
    decoder reports 4.8 and 8.8: **about one station a slot for a whole
    extra decode pass**, which is a defensible trade rather than a
    blocker. It only becomes a blocker at a density FT4 has not been
    shown to reach. A signal at +5 dB, 22 dB above FT4's threshold, is
    still missed a quarter of the time at 14 signals.
  - **Precision, measured beyond one file for the first time**: 6
    phantoms of 5 118 decodes (0.12 %), about one every 50 slots at the
    design point.

  What the corpus cannot see is written down with it: no fading, an SNR
  floor of −17 dB, DT confined to ±0.5 s, and the fact that it shares a
  generator with the golden. See `docs/notes/FT4_BENCHMARK.md` §23.

- **`fixed-point` does not apply to FT4 — the board runs the same f32
  arithmetic the host does.** The CoreS3 build enables
  `mfsk-core/fixed-point`, which reads as "host f32 numbers do not
  transfer to the device". They do, for every protocol on the generic
  `engine::pipeline`. The feature is gated into six files — five in
  `ft8/` and one site in `engine/fft.rs` (`default_planner_16`, which
  only `decode_block` consumes) — and none of them is on FT4's path.
  Confirmed by measurement as well as by reading: the tier-C FT4 sweep
  run with the feature on reproduces the f32 crossings to the digit
  (−16.89 / −17.46 / −15.71 / −16.00 dB, +0.00 dB on all four
  channels).

  `run-sensitivity-sweeps.sh` now takes `MFSK_SWEEP_FEATURES` so that
  kind of run is a one-liner, with a comment recording what the
  override does and does not cover.

  Consequence: every recall figure measured on host for FT4 is the
  board's figure too, and what remains between them is the FFT kernel
  (esp-dsp vs rustfft, already shown to give identical decodes on the
  golden), `dotprod-extern`, and everything that is not arithmetic —
  slot timing, capture, memory. `docs/notes/FT4_BENCHMARK.md` §24 also
  tabulates the FT4 evidence base and names the two rows no simulator
  closes: real receiver artefacts, and true FT4 band occupancy.

- **The FT4 "real off-air golden" is a simulated scene, it holds 19
  signals rather than 14, and upstream had no other.**
  `WSJT-X/samples/FT4/000000_000002.wav` is `ft4sim_mult` output from
  upstream's `lib/ft4/messages.txt` `File 2` block. Four in-tree
  checks: the filename is the simulator's own `000000_%06d.wav` pattern
  while every other protocol's sample carries a real UTC timestamp
  (`FT8/210703_133430.wav`, `JT9/130418_1742.wav`, …); the 19 rows
  reproduce the file; those rows are dated `190106` and say `Rx FT8`;
  and FT4 was introduced in WSJT-X 2.1 on 15 July 2019, six months
  later. The scene is a real 40 m **FT8** snapshot rendered as FT4 —
  the only thing upstream could do for a brand-new mode, and the reason
  the file is not evidence about FT4 band occupancy.

  The five signals `FT4_FULL_REFERENCE` does not list all sit above
  2700 Hz, outside the band `jt9 -H 2700` and these tests search, so
  "14/14 parity with jt9" is parity over a shared band rather than
  100 % of the file. **Neither this crate nor upstream has an off-air
  FT4 recording** — comments in three test files said otherwise and now
  say this. Its per-signal ground truth (SNR, DT, frequency) is exact,
  which is an independent check on `pipeline::ft4_snr_db` nobody had
  noticed was available.

### Changed

- **FT4 no longer climbs the blind `llrd` rung — a fidelity fix that is
  also 20-22 % of the LLR/BP/OSD ladder.** WSJT-X's FT4 decoder has no
  fourth *blind* LLR variant: `ft4_decode.f90:341-342` builds `llrd`
  only for `ipass > 3`, as `llrd = llrc` with the first 29 bits
  overwritten by an a-priori pattern — it is the **AP** variant. A
  fourth blind `llrd = scalefac*bmetd` is FT8's shape
  (`ft8c.f90:192`), which the generic ladder inherited and applied to
  FT4 as well. This crate's AP path (`msg::pipeline_ap`) uses
  `llr_set.llrd` for exactly WSJT-X's purpose and is untouched.

  Measured before removing it (`tests/ft4_llr_ladder_ablation.rs`, new):
  over 560 sweep files straddling four channels' 50% crossings the rung
  contributes **zero** decodes in both regimes — 235 with OSD and 179
  without, with or without it — and zero on the real WSJT-X golden.
  Verified after: golden 11/14 single-pass and 14/14 with SIC unchanged
  with zero phantoms, and `run-sensitivity-sweeps.sh ft4` **+0.00 dB on
  all four channels** (160 trials each).

  The same ablation says what *does* pay, which is the reason it was run
  rather than assumed from FST4's result: `llrc` (nsym=4) is worth 46
  decodes and OSD 56, so neither is a candidate for the same treatment.
  FST4's `nsym=8` conclusion does not port — its rung enumerates
  4⁸ = 65 536 tone hypotheses per group against FT4's 4⁴ = 256. See
  `docs/notes/FT4_BENCHMARK.md` §22.

- **`DecodeDepth::osd`'s doc comment was wrong about embedded.** It said
  OSD is "host-only", "compiled out of non-`fft-rustfft` builds
  entirely", and "a permanent architectural boundary". Neither
  `fec::ldpc::osd` nor the pipeline's OSD block carries an FFT-backend
  `cfg`, and both the FST4 (#306) and FT4 embedded benches have run
  `DecodeDepth::FULL` on an ESP32-S3 and reported its cost (`ft4-bench`:
  10 987 ms against `EMBEDDED`'s 8 642 ms over 31 candidates). OSD on
  embedded is a budget decision, not an impossibility — and it is worth
  56 decodes of 560 at the crossing.

- **`ft4_sync_search` rebuilt its frequency-shift phasor once per grid
  cell, and did not need to.** The shift was applied to `cd0` inside
  the innermost sample loop as a rotating phasor restarted at every
  `(df, i0)` cell — but `twid[n] = step^n` is indexed by the offset
  *within the Costas block*, not by `i0`, so it was identical across
  all ~340 `i0` positions each `df` sweeps. `fst4_sync_search` already
  folded it into the reference (`FlatRef`), which also leaves a plain
  inner product that `dot_f32` — and so `dotprod-extern`'s
  `dsps_dotprod_f32_aes3` on LX7 — can serve; FT4 now uses the same
  machinery. The identity was already recorded in this function's own
  comment ("the same dot product as twiddling each sample of `cd0` in
  place"); nothing new was derived.

  Not bit-identical (the products reassociate, and `FlatRef::fill`
  evaluates the phasor per sample instead of accumulating a recurrence,
  so it carries *less* rounding error). Verified before landing: golden
  14/14 total / 6/6 / 0 phantoms unchanged, every stage counter
  identical on all three SIC passes, and
  `run-sensitivity-sweeps.sh ft4` **+0.00 dB on all four channels**
  (160 trials each).

  **Host**: `ft4_sync_search` over 31 candidates 76.6 → 27.9 ms
  (2.75×); golden 3-pass `decode_loop` 36.5 → 26.6 ms (1.37×).
  **CoreS3**: search 13 225 → 4 447 ms (2.97×), and the production
  slot total 17 339 → 8 642 ms (2.01×) with the same 11 decodes.

  Stacking the ±0.5 s window from the entry above and a 40 KB
  internal-DRAM `cd0` buffer takes the search to 2 492 ms — **5.31×
  cumulative**, and the slot from 8.8× over budget to ~3.4× over. The
  `cd0` placement was expected to be worth 5–10× on `internal_pool`'s
  precedent and **measured 1.12×**: the byte count was right, the
  inference that it was the bottleneck was not. The search is no longer
  dominant (downsample 34 % / search 37 % / LLR+BP 29 %), which points
  the next work at `ft4::ddc` rather than at more of this. See
  `docs/notes/FT4_BENCHMARK.md` §19.

- **ハムフェア2026 booth material** (`docs/presentations/hamfair2026/`)
  — a ten-page A4-landscape flipchart and a two-page A4 leaflet for
  demonstrating the M5Stack CoreS3 receiver at the show. Most visitors are operators rather than
  developers, so the front half is "what is happening in front of you"
  and the back half is for the smaller number of people who might
  actually build on the crate.

  The wording lives in `content.md` rather than in the HTML, with
  `build.py` running the round trip: `stamp` gives each slot a `data-t`
  attribute, `extract` pulls HTML → Markdown, `build` pushes Markdown →
  HTML → PDF. `build` also checks for overflow every time, because
  `.page` is `overflow:hidden` and `.body` is a flex container — text
  that does not fit is dropped silently, and as a clipped card or table
  rather than a visibly short page. Round-trip losslessness is verified
  by pixel comparison: an `extract` → `build` cycle with no edits
  reproduces all twelve pages exactly.

### Changed

- **`ft8::decode_block`'s module doc described a decoder that no
  longer exists.** It claimed an 8192-pt spectrogram FFT with a
  `≈ 1.465 Hz` bin width, Costas tone positions "computed at
  fractional bins and rounded to the nearest integer", and a
  resulting `≤ 0.7 Hz` alignment jitter. All of that predates the
  move to `NFFT_SPEC = 3840 = 2 × NSPS` (WSJT-X `sync8.f90`'s own
  `NFFT1`), where `tone_step_bins` is **2.0 exactly** and there is no
  fractional-bin rounding or Hann window left to compensate. Rewritten
  to say what the module actually does, and to state why the embedded
  path runs two different frequency analyses rather than reusing one:
  the 3840-pt spectrogram is the *search* grid (`coarse_sync` wants
  every bin, so an FFT is the right shape), while
  `fill_symbol_spectra_goertzel` is the *extraction* step at each
  candidate's own continuous `(freq_hz, dt_sec)`, where only 8 of 1920
  bins are wanted. The spectrogram cannot stand in for the second:
  its time grid is quantised to `NSTEP`, its frequency grid to
  `3.125 Hz`, and under `fixed-point` it stores `u16` magnitude-squared
  with the phase already gone.

- **The generalised-Goertzel phase rotation is documented as measured,
  not as "irrelevant".** 0.6.4's CHANGELOG entry justified the
  rotation with "FT8 downstream consumes `|cs|²`", which is true of
  the shipping embedded configuration and not of the decoder as a
  whole — `engine::llr::build_group_amplitudes` sums complex cells
  across a symbol group and only then takes a magnitude, so the
  `nsym ≥ 2` LLRs (llrb / llrc) do see phase.
  `fill_symbol_spectra_goertzel` now carries a "Phase convention"
  section with the measurement: the factor is `exp(jω(N-1))`,
  magnitudes agree to `7.3e-5`, and since `NSPS × 6.25 / 12000 = 1`
  exactly, the `exp(jωN)` half is common to every tone, leaving a
  `-0.1875°`-per-tone-step tilt (`1.3°` across the tone axis) as the
  only tone-dependent term. Below the channel's own phase error and
  never observed to move a golden test, but not zero — and
  `DecodeDepth::EMBEDDED` (`LlrEffort::Minimal`) stops at `nsym = 1`,
  so it does not reach that path at all. The 0.6.4 entry in
  `docs/historical/CHANGELOG-0.6-0.7.md` gains a correction note
  rather than a rewrite.

- **`CLAUDE.md` describes the codebase, not just the workflow.** The
  agent notes had accumulated procedure — how to run the tests, how to
  cut a release — without ever saying what the repository *is*: the
  crate map, `mfsk-core`'s own module layout, which feature flags bite
  and why, and the fact that there is no `src/core/`. A cold start had
  to rediscover all of it.

- **README no longer names the published version.** The Status section
  read "Latest published tag: `v0.9.0`" — stale through both `v0.9.1`
  and `v0.10.0`, on the page most readers see first. The crates.io
  badge at the top of the same file has always shown the right number,
  so the hardcoded one is gone rather than corrected; a fact with two
  sources only stays true by luck. The release-mechanism bullet's
  `v0.6.x` example is now `vX.Y.Z` for the same reason.

- **`jt9::softsym::TONE_SPACING` reads the trait constant.** It and
  `<Jt9 as ModulationParams>::TONE_SPACING_HZ` were both written out as
  `12_000.0 / 6912.0` — identical `f32` to the bit, so nothing was
  wrong, but they agreed by coincidence rather than by construction,
  and a change to JT9's modulation geometry would have moved one and
  left the other behind. Same value, same public name, no call site
  touched.

- **`scripts/release-status.sh` tells a documentation pass from a
  decoder change.** The tier-C section listed every protocol whose
  directory had commits since the sweep baseline, which meant a
  comment-only commit read exactly like a decoder change — and a
  tier-C sweep is tens of minutes to hours per protocol. #323's own
  classification pass demonstrated it: one commit added a doc comment
  to `ft8/params.rs`, `jt9/softsym.rs`, `wspr/baseband.rs` and
  `msk144/spd.rs`, and four protocols appeared as needing re-sweeping
  against a diff that adds and removes no executable line.

  Each commit's diff under the protocol's own directory is now checked
  for a non-comment changed line. Prose-only commits are printed with
  a `[comments only]` marker rather than hidden — "nothing to sweep"
  should be visibly derived, not silently assumed — and a protocol
  whose commits are all prose-only drops out of the
  `run-sensitivity-sweeps.sh` line. The same filter applies to the
  shared-code (`engine`/`dsp`) warning.

  Rust line comments only: a `/* … */` body reads as code, and a merge
  commit is treated as code because plain `git show` renders it as an
  empty diff. Both err toward re-sweeping something that did not need
  it, never toward skipping something that did.

- **The tier-C sweep baseline records where its numbers came from.**
  `docs/notes/sweep-baseline.json` held 54 crossing SNRs and nothing
  about their provenance, so `release-status.sh` had to date the whole
  file by its last commit. One date for nine protocols is wrong in
  both directions: refreshing two of them
  (`run-sensitivity-sweeps.sh ft4 fst4`) marked all nine as current,
  while a documentation commit touching the file aged nothing and
  still reset the comparison. Both mistakes end the same way — a sweep
  that was needed gets skipped before a tag.

  The file now carries a `_meta` key holding, per protocol, the date,
  commit, machine and trial count its numbers were measured at.
  `sweep-regression-check.py --update-baseline` stamps only the
  protocols in that run, so a partial re-sweep cannot backdate the
  groups it never measured, and `release-status.sh` compares each
  protocol's own source against its own baseline date. With `_meta`
  (or python3) absent, both fall back to the previous file-date
  behaviour.

  The check also prints the baseline's provenance above its diff — a
  `+0.00 dB` line otherwise says the number matches without saying
  what it matches — and takes `--machine` to override the recorded
  box. The machine is recorded because the sweeps are rayon-scheduled
  under `parallel`, and a differing core count has been mistaken for a
  regression before; `BENCHMARKS.md` records it for the same reason.

  No measured value changed: all 54 crossings are identical. The
  seeded `_meta` entries name c757d32 / 2026-08-20, the commit and
  date that last wrote them, and carry no trial count because those
  CSVs were not kept; every later entry is written by the run itself.

- **The ハムフェア PDFs are generated, not committed.** They are build
  output — `content.md` and the HTML are the sources — and keeping them
  in the tree meant a text fix silently left a stale printable artifact
  behind. That is exactly what happened to the version line: the HTML
  said v0.10.0 while the committed PDFs still said v0.9.1, and the PDF
  is the half that gets printed. Now in `.gitignore`; run
  `python3 build.py build` before printing.

  `build` subsets the fonts afterwards when `pdftocairo` (poppler) is
  present. Chromium's PDF backend does not subset CJK everywhere: on
  Linux it emits the glyphs as paths and the flipchart comes to 1.5 MB,
  on macOS it embeds seven whole 2.19 MB faces, one per weight, for
  11.6 MB. Subsetting gives 2.67 MB and 1.09 MB — which is what makes
  the files small enough for convenience-store print services to accept
  at all.

  Worth knowing that this was tried and reverted once. While `.hl` was
  a highlighter band (`linear-gradient(transparent 62%, #fde3c3 62%)`),
  Chromium wrote it as a soft-masked shading and poppler/cairo re-emitted
  it as a dark grey bar across the text. `.hl` is a plain underline now
  and the documents contain no gradient at all, so the construct that
  broke is gone; re-verified page by page, with differences peaking at
  0.15 % of pixels and every one inspected being a hairline rule moving
  by a pixel. `build.py` carries the warning to re-check by *looking* if
  a gradient or blend mode is ever added back — the comparison that
  missed the breakage the first time measured 0.37 % and called it
  antialiasing.

  `README.md` gains the prerequisites this all implies — including that
  the static Noto Sans JP is the one to install, since Homebrew's
  `font-noto-sans-jp` is a single variable-weight file that Chromium
  handles badly.

### Fixed

- **`TRIALS` never reached the sweep corpus generators.** All seven
  `scripts/gen_*_sweep_wavs.sh` assigned `TRIALS=20` (q65: 15)
  unconditionally, so the `TRIALS=100 scripts/gen_fst4_sweep_wavs.sh`
  that `FST4_BENCHMARK.md` §11.3 prescribes for #311's n=100 re-run
  silently generated a 20-trial corpus. Nothing announces it: the
  script prints the count it is using per cell, and the sweeps skip
  absent trial indices without complaint, so the only symptom is a
  measurement quietly answering a smaller question than the one asked
  — and #311's whole open item is that its n=20 answer needs n=100.
  `"${TRIALS:-20}"` now, defaults unchanged.

  The generators' headers gained the corollary, which is the part that
  bites even with the override working: a cell is skipped only when
  every trial is already present, so raising the count regenerates an
  existing cell whole — the simulator is invoked once for all `TRIALS`
  — and overwrites the files already there with fresh realisations.
  In place, that both perturbs the tier-C corpus and re-shuffles the
  trial indices (`FST4_BENCHMARK.md`'s trap 1). Generate into a
  separate out-dir; the second positional argument is there for it.

- **The ハムフェア build script found no browser outside its author's
  machine.** `docs/presentations/hamfair2026/build.py` shells out to
  headless Chromium to print the PDFs, and looked for it in a fixed
  list headed by `/opt/pw-browsers/chromium-1194/chrome-linux/chrome` —
  one machine's Playwright cache root *and* one build number — followed
  by `shutil.which` on the usual names. On macOS that finds nothing at
  all, because Chrome lives inside an `.app` bundle, which is a
  directory and never on `PATH`.

  Resolved from the environment now, most specific first: `$CHROME` /
  `$CHROMIUM`, then Playwright's own cache globbed rather than pinned
  (so the build number does not matter and `PLAYWRIGHT_BROWSERS_PATH`
  is followed), then `PATH`, then known application-bundle locations on
  macOS and Windows.

  `build.py` also warns now when the intended body font is missing.
  Rendering is only reproducible if the font is: the CSS stack falls
  through to Hiragino / Yu Gothic / Meiryo, which is right for reading
  the HTML anywhere and wrong for regenerating an artifact — on a Mac
  without Noto, two boxes overflowed that do not overflow with it, and
  the PDF came out 12 MB against the committed 1.5 MB. Detected by
  measuring rendered text width against each generic family;
  `document.fonts.check()` is no use here, as it answers "available"
  for any non-webfont family, including one that does not exist.

- **`scripts/release-status.sh`'s cadence section died on macOS.**
  `date -d` is GNU-only; this repository's development machine moved to
  macOS, where BSD `date` rejects the flag and the whole `== cadence ==`
  block failed with `illegal option -- d` followed by an arithmetic
  error on the empty operand — so the one check that says how long it
  has been since the last tag printed nothing at all. `CLAUDE.md` calls
  this script "step 0, before anything else" precisely because release
  state kept being reconstructed by hand, so a section that silently
  stops working is the failure mode the script exists to remove. Fixed
  by not parsing a date string at all: `git log -1 --format=%ct` yields
  the same commit's timestamp already in seconds, from the same source
  as the `%cs` on the line above. `find -newermt` further down was
  checked and is fine — BSD `find` supports it.

### Documentation

- **The remaining bare `12_000.0` literals are classified** (issue
  #323, closing out #321's deferral). #321 consolidated three misnamed
  constants into `engine::protocol::SAMPLE_RATE_HZ` and left the rest
  as literals, calling a mechanical replacement churn — that decision
  stands. What it left undone was the distinction #307/#309 actually
  need: a definitional rate is not an analysis-grid rate.

  All 120 sites are now classified (47 test, 13 prose, 60 production),
  and the production sites are modulation spec, synthesiser output
  rate, raw-PCM ingest grid, or analysis grid. **Only the last would
  have to move for a receiver analysing at another rate, and it is
  seven sites, not a hundred**: `ft8::params::DF`,
  `engine::ft4_coarse::DF_HZ`, `jt9::softsym::FSAMPLE_DOWN`,
  `jt9::softsym`'s two `df1`, `wspr::baseband`'s `df`, and
  `msk144::spd`'s `fs`/`df`. FST4's own are already parametrised
  (`SyncDims::of(sample_rate_hz)`, `DdcCascadeConfig::input_rate_hz`)
  because #307 needed them to be.

  None of the seven is wrong today, and that is checkable rather than
  assumed: `RxGrid` — the only way a non-12 kHz analysis rate enters
  the crate — is constructed in exactly three places (`fst4::ddc`, one
  `engine::sync` test, the embedded FST4 monitor), so no
  FT8/FT4/JT9/WSPR/MSK144 path can reach a rate other than the
  canonical ingest one. They are annotated rather than parametrised:
  two are `pub const`, so parametrising would cost a breaking change
  for a rate nothing varies, and #321's own note — "classifying the
  sites is work that refactor should do with the measurement in hand" —
  is borne out by what #307 produced, where the right shape turned out
  to be a split rather than a rate threaded through everything.

  One finding ran opposite to the issue's expectation. `wspr::ddc` —
  the one place outside FST4 where a non-12 kHz analysis rate is
  already real production behaviour — is correctly *named* but not
  parametrised: `mixer_table()`'s eight entries exist because
  `1500 / AUDIO_RATE_HZ` is exactly `1/8`, so another input rate
  changes the mixer's period and leaves the table silently wrong
  rather than failing to compile. Nothing needs it to vary, so this is
  recorded at the site rather than fixed.

  The classification itself lives on `SAMPLE_RATE_HZ`'s own doc
  comment, with a one-line marker at each analysis-grid site so grep
  finds them too.

## 0.10.0 — search windows denominated in seconds (#282, breaking), FT8 coarse-sync lag window matches WSJT-X (#278/#280), early-frame decode (#283), WSPR host/embedded parity + streaming front end + the CoreS3 receiver decoding off a radio (#163/#260), FST4 npre1/npre2 OSD + i0±1 timing retry + rung-major scheduling (#198/#306/#308), code-sharing audit + cleanup (#290-298), FT8/generic OSD-gate ratchet (#285)

**Why a minor bump.** This crate's convention is that new protocols and
capabilities are patch-level (MSK144 shipped as `0.7.4`); minor bumps mark
structural API change. Three public search-parameter fields change type and
name here — `jt65::search::SearchParams::time_tolerance_symbols: u32` →
`time_tolerance_sec: f32`, the same rename on `jt9`, and
`q65::search::SearchParams::time_tolerance_symbols` →
`time_tolerance_early_sec` + `time_tolerance_late_sec` — because symbols are
not a transferable unit across sub-modes whose symbol lengths differ by 20×.
Callers that set these must update; callers that used the defaults get
corrected (and, for Q65 and JT65, substantially wider) windows automatically.
Same handling as `0.8.0`, which collected its breaking changes into a minor
rather than distributing them across patches.

### Added

- **The docs say what the hardware does.** README still read "live
  IC-705 hardware verification hasn't happened yet (issue #163)" and
  led with the StickS3 as the working receiver; ROADMAP's Phase B-Core
  and the root notes carried the same claim and listed the touch UI as
  pending when it is how the mode is chosen. `LIBRARY.md` needed
  nothing — no public API changed, and its examples are doctests the
  merge gate runs.
- **CoreS3 operator manual**, in English and Japanese
  (`docs/reference/MANUAL_M5STACK_CORES3.md` / `.ja.md`), plus the
  crate's first `CLAUDE.md`. The project's main hardware target had no
  user-facing documentation: four receivers in one image, mode
  switching from the touch panel, the host-versus-peripheral power rule
  that decides at boot whether the board can talk to a radio or be
  flashed, and why a receiver with no clock decodes nothing.
- **`scripts/release-status.sh`** — the release state computed from the
  repository rather than recalled: version against tags, CHANGELOG
  agreement and freshness, cadence position, and which protocols' own
  source has changed since `sweep-baseline.json` was refreshed, with
  commit subjects so clippy drift and a decoder change are
  distinguishable at a glance.
- **`[boot-summary]`** on the CoreS3 — the boot-critical state re-emitted
  once a log sink exists. In USB host mode there is no serial console,
  and WiFi associates seconds after the power and USB decisions are
  made, so every line about them lands in the log fanout's staging ring
  and is overwritten before anything can read it.
- **A link bar in every CoreS3 mode** carrying USB role and state, open
  device count, the three VBUS enables, VBUS and battery volts, whether
  the clock is set, and WiFi RSSI.
- **Per-slot logging that names its audio source and states its time
  budget**, in all three CoreS3 receivers. The FT8 controller labelled
  every slot `WAV[n]` whatever the source. The budget means opposite
  things per mode: FT8's 15 s slot genuinely runs out on a busy band,
  while the WSPR and FST4 monitor loops are built with deliberate slack
  and exceeding it is a fault.

- **FST4 reports SNR without a whole-slot FFT** — `fst4::baseline`'s
  `fst4_ddc_snr_db`, reached automatically when `SnrCtx::fft_cache` is
  empty. WSJT-X's own formula reads both its noise baseline and its
  signal power out of the big forward FFT of the raw slot; a DDC
  receiver never computes that FFT, so on that path `snr_db` was `NaN`.

  The new estimator takes its noise reference from the part of the
  refined baseband the signal does not occupy — 111.111 Hz wide against
  ~12 Hz of FST4-60 tones — so every term comes from one buffer and the
  pipeline's RMS normalisation cancels. Against a corpus whose SNR is
  known by construction it holds a **1.0 dB bias spread from -20 to
  -28 dB**, and on the WSJT-X FST4 sample it lands closer to `jt9`'s own
  probed values than the whole-slot formula does (N5TM -5.7 vs jt9
  -6.9, K9KFR 15.7 vs 16.1). `fst4::rung_major`'s scheduler fills the
  field in too, where it also used to write `NaN`.

  `SnrCtx` gains `cd0`/`ds_rate_hz` for this, and
  `engine::llr::snr_ratio` is split out of `compute_snr_db` — the
  scale-free half is shared across protocols while the dB tail is not.

- **`coarse_sync`'s correlation matrix is now a public seam**
  (issue #327) — `sync2d_shape` / `fill_sync2d_row` /
  `coarse_sync_from_sync2d`, alongside the existing
  `coarse_sync_from_spectra`, which is now just those three composed.

  Every row of that matrix is a pure function of the finished
  spectrogram, and the host has parallelised the fill through rayon
  since it was written. An embedded caller could not: the loop was
  buried inside one call. Exposing the seam lets the CoreS3 FST4
  monitor spread the fill across both cores — measured 1131 → 714 ms on
  the wideband 100-3000 Hz FST4-60 search — with bit-identical output,
  since the split decides only which core writes which row.

  This is the same shape as `0.10.0`'s earlier `SpectrogramBuilder`
  split (#307/#336), which moved the spectrogram *build* out of the
  post-slot budget: both exist so a receiver can pay a stage somewhere
  other than "all at once, on one core, after the slot ends".

- **A standalone WSPR receiver application for the CoreS3**
  (`embedded-poc/m5stack-cores3-app/src/bin/wspr_app.rs`). Phase E (#260)
  answered the decoder-level question — `wspr::ddc` exists, the steady-state
  pipeline decodes in 82.8–90.1 s against a 110 s deadline, golden 9/9 holds
  every slot. This is the application built on top of it: a portrait
  240×320 spot list, band selection, WiFi with unbounded background retry,
  an HTTP configuration server, NVS-persisted settings, NTP time sync, and
  UDP log fanout — the last because a device that has to sit on an antenna
  for hours can't stay tethered to a serial monitor.

  Real USB audio is wired in. `uac.rs`'s reader-thread → consumer coupling
  was generalized into an **`AudioSink` trait**, so the FT8 controller
  (`Ft8ChunkSink`, wrapping the previous behaviour verbatim — `set_chunk_q`
  keeps its old name and signature, so `main.rs`/`decode_pipeline.rs` needed
  no changes) and the WSPR receiver (`WsprDdcSink`, feeding a per-slot
  `StreamingDdcCascade`) share one USB host / class driver / hot-plug /
  resample implementation instead of two.

  **Not hardware-verified against a real UAC source** — that is issue #163,
  which this change makes load-bearing for two applications rather than one.
  Until a device enumerates, `UAC_AUDIO_ACTIVE` never flips and the app
  keeps running its synthetic generator, so it stays fully functional and
  host-independently testable with nothing plugged in. Remaining open items
  (wall-clock slot alignment, `SpotSink::Http` never run against a real
  wsprnet endpoint) are collected in #313.

- **`wspr::ddc::StreamingDdcCascade`** and the `wspr-ddc-cascade` feature —
  a two-stage decimator, ~4.5× less compute than the single-stage
  `StreamingDdc` it sits beside. DDC compute occupancy measured 24.8–39.2 %
  of the 120 s slot budget on real hardware; pointer-address logging
  root-caused it to `DdcBufs`' taps and I/Q history (7 172 B / 9 220 B /
  9 220 B) all exceeding `SPIRAM_MALLOC_ALWAYSINTERNAL=4096` and landing in
  PSRAM rather than internal DRAM — a gap `StreamingDdc::new_in`'s own doc
  comment had warned about and nothing in the tree worked around.

  The fix was a from-first-principles Crochiere-Rabiner derivation rather
  than a reaction to the benchmark number: splitting `DECIM = 32` into 8×4
  gives stage 1 `N = 43` (loose transition, only needs to keep stage 2
  honest) and stage 2 `N = 223` (carrying the real 27 Hz [174, 201] Hz
  requirement, same as the single-stage design). Every resulting buffer is
  small enough to auto-place in internal DRAM, so no manual placement API
  was needed. Six new host tests — coherence > 0.99 against the whole-slot
  FFT reference, out-of-band rejection, and direct agreement with the
  single-stage implementation — alongside the original five. The two DDC
  features are mutually exclusive (`compile_error!` in `decode_scan_inner`),
  each with its own real-signal golden test against the WSJT-X recording.

- **`engine::dsp::fir_decimate`** — the generic filter-and-decimate
  primitive (`design_lowpass` + `FirStage`) extracted from `wspr::ddc`,
  which carries no WSPR-specific logic. The mixer table, `REFERENCE_GAIN`
  and the cascade tap-count constants stay in `wspr/ddc.rs`; both DDC
  implementations now share the filter core.

- **`fst4::rung_major`** — rung-major candidate scheduling for FST4
  (#306 item 3, VK3NV), as a separate FST4-specific entry point.
  `engine::pipeline::process_candidate_basic_impl` stays exactly as-is,
  serving FT4/FT8/FST4/MSK144 depth-first.

  Depth-first pays every rung for a candidate that never decodes before
  moving to the next, so one hard candidate early in the list can delay
  every easy decode after it arbitrarily far. Measured on the FST4-60
  golden: both real signals decode at ~0.15 s / ~0.30 s *only because they
  fall early in candidate order*; a worst-case ordering pushes them to
  ~30 s of a 30.15 s total. Rung-major sweeps each rung across every
  still-undecided candidate before descending, which bounds worst-case
  time-to-first-decode to one rung's cost (~7.4 s projected for the first)
  regardless of ordering. Total work is unchanged — only the order.

  Deliberately a 5-stage, not 6-stage, ladder: a targeted ablation found
  `llrd` contributes **exactly zero** additional recall beyond
  `llra`/`llrb`/`llre`/`llrc` + OSD — byte-identical hit counts with and
  without it in all four configurations tested, on both the AWGN and
  CCIR-moderate corpora. A stage that never wins shouldn't be scheduled.

  `skip_llrc` and `offsets` are caller choices rather than baked policy.
  `offsets` in particular: #308's `i0±1` timing retry is an unconditional
  win on host, but on real hardware porting it here unconditionally triples
  `full` (40.102 s → 121.281 s) and more than doubles `no8_osd`
  (13.643 s → 34.200 s) for a few recall points, so the production default
  stays `&[0]` and a deployment that can tolerate spanning slots can opt in.
  Whether to fold `offsets` into cost-ordered scheduling was #310's
  original proposal; it was **measured and declined** — see below.

- **`fst4::rung_major::Schedule`** — the escalation schedule as an
  explicit choice, with `decode_phase_split_timed` alongside the existing
  `decode_rung_major_timed` (issue #310). Both are `internal-testing`-gated
  and unreachable from `DecodeRequest`, so no shipped decode path changes.

  `Schedule::RungMajor` is what this module shipped with: every rung swept
  across every candidate. That buys the ordering-independent
  time-to-first-decode bound — but **the bound is bought entirely by the
  first rung**. Continuing breadth-first past it defers OSD, where most
  decodes come from, until every candidate has had every cheaper stage.
  Measured over 1 263 post-first-rung candidates: **350 vs 380 decodes in
  386 at a realistic ~50 % budget, 35 vs 166 at 10 %.**

  `Schedule::PhaseSplit` keeps the bound and drops the cost. **Phase A** —
  `llra` at `offsets[0]` across every candidate, breadth-first, never
  budget-gated (making the invariant interruptible would give it away).
  **Phase B** — the rest of that offset's ladder, depth-first, candidates
  ordered by the `nsync` Phase A already computed for the gate. **Phase C**
  — the remaining offsets, same order. `budget_ok` is polled before every
  Phase B/C stage.

  Ordering and schedule are one decision, not two: under breadth-first a
  10 % budget buys ~98 % of the first-stage sweep, so every candidate
  ordering visits nearly the same candidates and sorting is worthless. It
  only pays once Phase B goes depth-first.

  Folding `offsets` into a cost-ordered queue — #310's original shape — was
  declined on measurement: it would triple the first-rung sweep, offset
  setup (`symbol_spectra` + bit-metrics rebuild) is not free, and the
  payoff cannot be aimed, since ranking offsets by sync quality matched
  exhaustive retry in only 1 of 8 runs. `docs/notes/FST4_BENCHMARK.md`
  §13/§14 carries the reasoning and the numbers.

  A non-`#[ignore]`d test asserts the two schedules return byte-identical
  decodes without a budget gate, across three offset configurations × both
  `skip_llrc` settings — the change is meant to move *when* decodes
  appear, never *whether*.

### Changed

- **`SAMPLE_RATE_HZ` is now one crate-level constant** (issue #321),
  re-exported at the crate root. It was defined three times under two
  names — `uvpacket::tx` and `ft8::decode_block::types` privately, and
  `engine::dsp::msk::FS_HZ` publicly, which advertised the crate's
  sample rate under a name suggesting it belonged to MSK144. `FS_HZ`
  remains as a `#[deprecated]` alias since it is public API, with a test
  pinning it to the constant it now aliases so the two cannot drift.

  Scope is deliberately just that. The ~100 remaining bare `12_000.0`
  literals are left alone: each reads unambiguously in its own context,
  and the distinction a future refactor would actually need to draw —
  definitional rate (`SYMBOL_DT`, `TONE_SPACING_HZ`, invariant) versus
  analysis-grid rate (`SyncDims`' `df`/`tstep`, variable) — is
  conceptual and is not addressed by naming the constant. See #307/#309.

### Fixed

- **The CoreS3 receiver decodes off a radio** (issue #163). Verified
  2026-08-23 against an IC-705 on 40 m: six to eight FT8 stations per
  slot, +8 to −24 dB, over USB Audio. Three faults stood between the
  working USB transport and a decode, and each looked like something
  else:
  - **The USB host never installed.** Gating a diagnostic panel behind
    a build flag narrowed the condition of the block that panel
    happened to start, and that block ran as far as `start_host()`.
    VBUS came up, all three enable bits read back correctly, the radio
    was attached, and nothing was ever asked to enumerate.
  - **The slot grid never anchored.** The FT8 controller had never
    started NTP, so the system clock stayed unset and
    `time_sync::samples_to_next_slot_12k` returned `None`. Unanchored,
    the 15 s grid free-runs at a phase uniform over 15 s against a mode
    that tolerates ±2.5 s. The board now reads the CoreS3's
    battery-backed BM8563 before WiFi exists and writes it back after
    NTP syncs, so the clock survives a power cycle and works out of
    range.
  - **`StatusInfo::utc_sod` was never assigned**, so the panel read
    `--:--:--` whatever the clock was doing — the one indicator that
    would have said why thirty candidates a slot resolved to nothing.
- **The CoreS3's WSPR receiver never saw the radio.** Its USB host
  enumerated the IC-705's hub and stopped — `num_devices=1` where FT8
  reaches 3, the panel reading `NODEV d1 V111` with VBUS bits identical
  to a working FT8 boot. Three separate omissions, each a thing one of
  the other two receivers already did:
  - `start_host` was called the moment VBUS was enabled, before the
    boost had ramped. "The boost is up" is now
    `enable_usb_host_vbus`'s postcondition rather than something three
    call sites agree on by hand.
  - The ninety-nine lines around `start_host` were not shared, so
    WSPR and FST4 never installed `esp_log_bridge` — and `ENUM` and
    `EXT_HUB` are C-side tags, so in those modes a board that would
    not enumerate had no way to say why. One
    `uac::start_host_when_ready()` now. Gone with it:
    `USB_HOST_DELAY_MS`, six seconds on every host boot, whose stated
    purpose was a re-flash window that cannot exist since the firmware
    started deciding host-versus-peripheral from VBUS.
  - With the trace finally reaching the log: `EXT_HUB: Interrupt EP
    allocation failure: ESP_ERR_NO_MEM`. Endpoint buffers must be
    DMA-capable internal memory, and WSPR's display task was holding
    32 KiB of it in a stack that FST4 has kept in PSRAM since it was
    written.
- **A receiver with nothing to hear said otherwise.** WSPR decoded a
  baked 2008 recording at startup and fabricated a `DDC_TEST_CALL`
  burst every slot when no radio was attached; FST4 replayed a baked
  slot the same way. Nine real callsigns, and then `K1ABC` every two
  minutes, on a spot list where nothing distinguished them from
  received stations. Both are opt-in now (`MFSK_WSPR_SYNTH=1`,
  `MFSK_FST4_REPLAY=1`), with the fixtures behind Cargo features so
  the bytes are not linked in: 4 423 632 -> 2 604 128 B of image, off
  every flash. The FT8 `decode` mode keeps its WAV — that is a mode
  the operator picks by name, not something that happens when a radio
  is missing.
- **The link bar named its USB states three different ways.** `chg`,
  `no dev`, `NOHOST`, `STREAM`, `ERROR ` read as unrelated fields
  rather than one state machine, and `no dev` did not say whether it
  was a fault or a wait. Six characters, upper case, one name each.
  WSPR's `src=` had the same problem in miniature: three things can
  produce a slot — a baked recording, a fabricated burst, a radio —
  and it was computed from a bool, so the golden slot printed
  `src=uac`.
- **The CoreS3's WSPR receiver decoded nothing at all.** Its scan task
  asks for a 72 KiB stack and the dual-core worker arena for 80 KiB,
  both out of one 128 KiB contiguous internal block — 152 does not fit
  in 128, so `xTaskCreatePinnedToCore` failed and the DDC filled
  buffers nobody consumed. Both numbers were measured, but against a
  decode path that kept three 10 368 B `IsQs` alive at once:
  `refine_cascade` returned its champion by value on top of the two it
  ping-pongs, and the jitter ladder built one per position through the
  by-value `tone_amplitudes`. With those gone the measured peak fell
  63 192 B → **42 536 B**, and both reservations are 48 KiB, confirmed
  on hardware at 41-43 KB. Pure refactor: the merge gate passes,
  `wspr_golden_recall_and_precision` is unchanged, and the AWGN
  sensitivity sweep moves +0.00 dB. `wspr-bench` also never called
  `reserve_arena` and panicked on worker spawn — same `worker_arena`
  change, second casualty.
- **WSPR called every DDC-fed slot synthetic, including real ones.**
  The flag was hardcoded `true`, correct while a DDC-fed slot could
  only be the fabricated test burst — the receiver had never been run
  against a radio. It gates wsprnet reporting, so real receptions were
  being withheld from the spot database and labelled `src=synthetic`.
  It comes from `UAC_AUDIO_ACTIVE` now.
- **The CoreS3 no longer ships one operator's callsign compiled in.**
  `decode_pipeline.rs` carried `MY_CALL`/`MY_GRID` as literals; they
  come from `cfg.toml`'s `[station]` section now, as they have in
  `m5stack-s3-app` since Phase 1.7. Empty leaves the QSO FSM idle,
  which is correct for a receive-only station.
- **WSPR and FST4 stay peripherals on external power.** The FT8
  controller has checked VBUS before taking USB host mode since #163;
  the other two receivers had not, so a board plugged into a PC boosted
  VBUS back into it and stopped enumerating — taking the port a flasher
  would use with it.
- **The CoreS3 stopped powering hardware it does not use.** Copying
  M5GFX's display init raised the AW9523B's speaker-amplifier enable
  and wrote AXP2101 `0x90 = 0xBF`, turning on the AW88298, the ES7210,
  the camera rail and both BLDOs — on a board sourcing 5 V to a radio
  from its own cell. Espressif's BSP names every one of those pins.
  Register `0x90` is written whole now: it survives a reset, so
  read-modify-write could only ever accumulate what earlier boots
  turned on.
- **`coarse_sync`'s de-duplication is no longer quadratic** (issue #327).
  It compared every candidate against every earlier one; candidates
  within 4 Hz are a contiguous run, because the list is
  frequency-sorted by construction, so it now walks a sliding window
  instead.

  Output is unchanged — bit-identical, not merely equivalent-in-effect,
  and that distinction is load-bearing here: the loop zeroes scores as
  it goes, so which pairs are compared in which order decides the
  result. Skipped pairs are exactly those that failed the 4 Hz test and
  mutated nothing, and `dedup_suppress_matches_all_pairs` pins the new
  loop against a brute-force reference over lists built to stress ties,
  boundary spacing and suppression chains.

  On a narrow search this was invisible; on a wide one it was most of
  the search. Real CoreS3 hardware, FST4-60 wideband (1881 bins, ~15k
  candidates before dedup): the ranking half of `coarse_sync` fell
  **2989 → 292 ms**, and the whole search 4123 → 1009 ms with the
  dual-core fill above. It also explains a gap #307 had left open and
  provisionally attributed to the spectrogram working set — the wideband
  search cost 2.147 ms/bin against the sniper's 0.671 ms/bin for
  identical per-bin work. Per-bin *fill* cost is in fact the same for
  both (0.60 ms/bin); the entire 3.2× was this superlinear stage.

- **`coarse_sync`'s de-duplication decision is now final** (issue #312,
  found by VK3NV). The dedup pass marks the losing near-duplicate (within
  4 Hz / 40 ms) with `score = 0.0`, but the `retain` immediately after is
  an OR — `score >= sync_min || stage1_pass(fi)` — so on FST4, where
  `stage1_norm` is populated, a candidate the dedup had just rejected was
  re-admitted through the second arm and went on to occupy a slot after
  `max_cand` truncation.

  Measured before the fix: **3 of the 50 slots** at the production
  `max_cand = 50` on the FST4-60 golden's K9KFR target, all three inside
  `rank_candidates`' reserved near-`freq_hint` group, i.e. displacing
  real candidates rather than falling off the bottom of the score sort.

  **No sensitivity cost.** Across 80 near-threshold sweep trials not one
  re-admitted duplicate ever decoded, and on the wideband production path
  (`DecodeRequest`, 100-3000 Hz, no hint) recall over the partial-recall
  band is byte-identical before and after — 82/120, unchanged in every
  cell.

  Deliberately *not* the narrower-looking `score >= sync_min &&
  stage1_pass(fi)`: that would also drop candidates which clear the score
  gate but fail stage 1, which is exactly what #146's OR-gate exists to
  keep. The defect was only that a rejected duplicate came back, so the
  suppression is now tracked separately from the score and only that is
  fixed.


- **The `wspr_app` crash loop — PSRAM-backed thread stacks.** With internal
  DRAM driven down to ~2 KB free by `wifi_driver_init`, the
  `uac_app`/`usb_events`/`uac_reader` `std::thread` spawns all pulled their
  stacks from that same tight pool via the ESP-IDF pthread compat layer. On
  this `esp-idf-svc` std target a later sub-4 KiB allocation failure
  surfaces as a genuine Rust panic, which poisons whichever `Mutex` it held;
  the next task to touch that lock panics too. The visible symptom was a
  double-panic crash loop whose backtrace landed on whichever task lost the
  race to be second — i.e. never on the actual cause. `spawn_psram_thread()`
  mirrors the existing `spawn_network_task` fix via `std::thread`'s
  `ThreadSpawnConfiguration` hook, and all three `uac.rs` spawns route
  through it.

- **The CoreS3 LCD stayed dark end-to-end — four independent root causes**,
  none catchable by host build, clippy or test; found over ~15
  flash-and-observe iterations against the physical panel. (1) AXP2101
  DLDO1, the backlight power rail, was never enabled — `pmic.rs` read the
  chip ID as a presence check and relied on power-on defaults for every
  rail; register readback confirmed bit `0x80` of `0x90` clear beforehand,
  so the backlight had zero power regardless of what the LCD controller's
  SPI logic did underneath. Cross-checked against M5Stack's own M5GFX
  source. (2) `board.rs`'s AW9523B `LCD_RST`/`TP_RST` bits were swapped
  versus M5GFX's verified mapping (`P1_1` is `LCD_RST`) — harmless today
  since both are driven together, fixed for when Phase 6-Core touch needs
  `TP_RST` alone. (3) `DrawTarget::clear()` gave partial or wrapped
  coverage on this mipidsi/SPI setup every time it was tried. (4) The
  display task was starved by the scan task sharing its core.

- **Both FT8 controllers drove their ILI9342C panels as ILI9341** — the
  same model bug in each, fixed in both.

- **WiFi bring-up on the CoreS3**: STA connect now retries on association
  refusal with a full rescan-and-reconfigure rather than giving up, power
  save is disabled, and the `httpd` task stack moved to PSRAM. The
  DHCP-silent symptom that prompted the investigation was confirmed
  router-side, not a defect in this crate — recorded so it isn't
  re-investigated here.

### Changed

- **WSPR's Fano cycle budget is now `wsprd`'s own 10 000 cycles/bit on
  host, with embedded opting into 5 000 via `wspr-fano-cap-fast`**
  (issue #260). The cap introduced during embedded work was a single
  constant shared by both, chosen for the CoreS3 slot deadline it buys;
  host has no deadline reason to pay for it. 10 000 is
  `lib/wsprd/wsprd.c:799`'s `maxcycles`, so the host default is neither
  slower nor less faithful than the reference decoder, and it is what
  this path effectively ran at before the cap was wired up.

  Raising it further is not free, which is why it is a feature and not
  a number to tune. Swept over the 500-trial AWGN corpus — every file
  holds one transmitted message, so any other decode is a false one by
  construction:

  | cycles/bit | -32 dB | -31 dB | -30 dB | phantoms/500 |
  |---:|---:|---:|---:|---:|
  | 5 000 | 19 % | 67 % | 96 % | 0 |
  | **10 000** | 22 % | 70 % | 96 % | 0 |
  | 20 000 | 24 % | 73 % | 96 % | 0 |
  | 50 000 | 27 % | 76 % | 98 % | **2** |
  | 100 000 | 31 % | 75 % | 98 % | **7** |

  On recall alone this reads "higher is better"; the phantom column
  says the usable range ends between 20 000 and 50 000. Given long
  enough, Fano finds codewords that satisfy the CRC but are not the
  transmitted message, and in a multi-pass SIC decoder those enter the
  carried callsign table and are subtracted from the residual.

  `wspr_awgn_snr_sweep` now reports phantoms alongside recall, so this
  cannot be measured one-eyed again.

### Added

- **`mfsk_app_shared::wsprnet`** — wsprnet.org spot reporting, ported
  from WSJT-X's `Network/wsprnet.cpp` (wsprnet.org publishes no
  specification, so that is the only normative source for the field
  names, formats and units). Covers the `function=wspr` spot and the
  `function=wsprstat` heartbeat.

  Reporting is **off by default**: `SpotSink` is `Disabled` unless
  configured, with `Dummy` (build and log) and `Http { url }` (build
  and POST, explicit URL, no default) as the other states. A spot is a
  public claim that a named station was heard at a time and frequency,
  so making one is a call-site decision.

  Two details that are easy to get wrong and are therefore tested:
  `tqrg = dial + (audio − 1500) / 1e6` — the field is the
  *transmitter's* frequency, derived from the audio tone — and `mode`,
  where a 2-minute FST4W reports as `3` because `2` already means
  WSPR-2.

- **`hosttest/mfsk-app-shared`** — runs the target-independent parts of
  `embedded-poc/mfsk-app-shared` on the host. That crate pulls
  `esp-idf-svc` and is excluded from the workspace, so unit tests
  written inside it would execute nowhere — the same shape as the
  golden tests that silently skipped in CI before `MFSK_REQUIRE_CORPUS`.
  Modules are pulled in by `#[path]`, so there is one source file and
  it is the one that ships.

### Fixed

- **FST4's OSD now uses WSJT-X's real `npre1`/`npre2`-pruned search**
  (issue #198/#306), not the unpruned k1/k2/k3 combinatorial search
  FST4 and MSK144 previously shared with FT4/FT8 by default — WSJT-X's
  own `osd240_101.f90`/`osd128_90.f90` use the same
  `npre1`/`npre2`/`ntheta`/`ntau`-pruned architecture FT8's
  `osd_decode_npre1(_npre2)` already ported (issue #63), just tuned
  differently; FST4/MSK144 had been running a different, unpruned
  algorithm instead. `osd_decode_npre_generic<P: LdpcParams>` ports it
  generically (FST4 wired; MSK144 not yet). Real-hardware measurement
  (CoreS3): FST4-60's candidate loop dropped 1.25-1.5× on top of every
  prior optimisation in this line, with recall unchanged on the AWGN
  corpus checked and a small (recovered by the fix below) recall cost
  under CCIR-moderate fading.
- **FST4 now retries each candidate at `i0±1` timing offsets**
  (issue #308), matching `fst4_decode.f90`'s own `ijitter=0,+1,-1`
  retry at normal/deep decode depth — every FST4 candidate was
  previously decoded at exactly one refined timing position. Found
  while investigating the npre-port's CCIR-moderate recall cost above:
  this recovers it fully at the SNR checked (m26: 15/100 → 20/100,
  exactly matching the pre-port unpruned search's own 20/100) via a
  *more* WSJT-X-faithful mechanism rather than reverting the OSD port.
- **JT9's default search band is 200-4000 Hz**, `jt9`'s own CLI
  defaults (`--lowest` 200, `--highest` 4007). It was 1400-1600 Hz —
  narrower than any real JT9 sub-band, and narrow enough that it
  **could not decode this crate's own JT9 golden recording**;
  `tests/jt9_wsjtx_samples.rs` had to override it, with a comment
  saying the default "excludes every" golden decode.

  Measured on `130418_1742.wav`: 1400-1600 Hz found 2 decodes, every
  wider band found 5, and wall clock was flat at ~60 ms from 200 Hz
  wide to 3800 Hz wide. The coarse search's cost is the whole-buffer
  spectrogram build, not the per-bin scan, so the narrow band was
  buying nothing.

- **Q65 and JT65 decode a frame that started before the audio buffer**
  (issue #283). Both coarse searches clamped `row_min` at 0 and never
  scored such an alignment at all, so their early edge sat at exactly
  `−nominal_start` — the clamp's signature, not a window limit.
  Measured against real `jt9` on the reference simulators' Δt sweeps:

  | | ours before | ours after | `jt9` |
  |---|---|---|---|
  | Q65-15A | −0.50 s | **−1.00 s** | −1.00 s |
  | JT65 | −1.00 s | **−4.00 s** | −3.00 s |

  Q65 now matches the reference exactly; JT65 exceeds it. WSJT-X gets
  there by scoring the hypothesis and skipping out-of-buffer terms
  inline (`xcor.f90:49-50`, `sync9.f90:40`); this crate front-pads the
  buffer instead, so the leading silence *is* the erasure and every
  `extract_*_energies` keeps its unsigned arithmetic and full-frame
  bounds check untouched. The numerics come out the same. Padding is
  allocated only when the search window actually reaches past the
  buffer start, so the common path still borrows the caller's slice.

  Costs nothing measurable: the tier-A+B suite is 91-94 s across
  before/after runs.

- **The "roughly 80% of code is shared across protocols" claim
  (`README.md`, `lib.rs` crate doc) was never measured** — introduced
  as scene-setting prose in commit `929114c` and never revisited, not
  even when Q65 (one of the least-shared protocols) was added and the
  protocol count changed from six to seven with the percentage left
  untouched. Measured (`wc -l`, excluding the experimental uvpacket
  example): about a third of `src/` (18.7k / 58.9k lines) is generic
  over `P: Protocol`; only FT4 and FST4 actually route through the
  shared `engine::pipeline` decode driver. Both doc sites now cite the
  measured figure and point at `docs/reference/LIBRARY.md` §0.5's
  per-protocol table, which was already accurate and is now the
  source of truth for future edits. Also fixed a flatly incorrect
  claim in `wspr/mod.rs`'s module doc (dating to the initial commit,
  never revised) that WSPR's FEC and message codecs are "shared with
  the other modes" — they are not; WSPR has its own `ConvFano` and
  `Wspr50Message`, as the same module doc's own preceding paragraph
  already said.

### Added

- **`Q65Result::dt_sec` / `Jt65Result::dt_sec`** — frame start in
  seconds from the buffer origin, i.e. the *signed* form of
  `start_sample`. `start_sample` saturates at 0 for a frame that began
  before the buffer, so it cannot express the case #283 added; `dt_sec`
  can, and is what to compare against a reference decoder's DT column
  (subtract your own nominal start). Additive — these structs are
  outputs, constructed only inside the crate.

- **JT65's Δt search window reaches WSJT-X's late edge** (issue #282,
  **breaking**: `jt65::search::SearchParams::time_tolerance_symbols:
  u32` → `time_tolerance_sec: f32`). It was `3` symbols ≈ ±1.11 s.
  Measured against real `jt9 -6 -p 60 -d 3` over a `jt65sim -t` Δt
  sweep at −10 dB: `jt9` decoded every step out to **Δt = +5.0 s**;
  this crate stopped at **Δt = 0.0**. Now 7.62 s, from
  `sync65.f90:29-30`'s `lag1=-32, lag2=82` (× `1024/11025` s), and the
  same sweep decodes out to +6.0 s.

  A JT65 frame is 46.8 s inside a 60 s slot, so a late start has far
  more room than an early one and WSJT-X's window is deliberately
  asymmetric (−2.97 / +7.62 s). Searched symmetrically at the wider
  half here: `row_min` clamps at row 0 for any realistic nominal
  start, so the extra negative span costs nothing reachable.

  **Free**: `jt65_sweep` is 7.40 s before and 7.44 s after; the whole
  tier-A+B suite is unchanged at ~93 s.

- **JT9's window is unchanged in span but now stated in seconds**
  (**breaking**, same rename). 1.728 s is numerically identical to the
  previous `3` symbols. `jt9_decode.f90:69-70` reads
  `lag1=-2.5/tstep, lag2=+5.0/tstep`, which the #282 audit took at
  face value and flagged as a 4.3× gap — but real `jt9 -9` on a
  shifted `jt9sim` sweep decodes only out to **Δt ≈ +0.6 s**. The
  apparent late reach is not usable reach, and this crate already
  covered it (out to +1.5 s). No behaviour change; the audit's JT9
  entry was wrong.

### Added

- **`tests/dt_window.rs` — the Δt regression harness** the #282 audit
  said was the prerequisite for everything else. Synthesises a signal
  in-crate, places it progressively later in the slot, and asserts the
  decoder still finds it out to the edge *real `jt9`* reaches. No
  corpus, runs in CI, ~0.2 s.

  This closes the hole the audit's most important finding named: every
  sweep corpus is generated at Δt = 0 (`gen_ft8_sweep_wavs.sh:37` and
  `gen_fst4_sweep_wavs.sh:34` hardcode `DT=0.0`; `jt9sim.f90:124`
  hardcodes `k=12000` and takes no Δt argument), so a too-narrow time
  window was invisible to the entire suite by construction. JT9
  measured "at or above parity" in #20/#23-#26 while carrying a window
  the audit believed was 4.3× too narrow, and nothing could have
  flagged either the belief or its refutation.

  Asserts the late side only: placing a frame early enough eventually
  pushes its start before sample 0, where truncation and a window
  limit are indistinguishable. The late side is also where both
  reference windows are deliberately asymmetric.

- **Q65's Δt search window matches the reference, which is
  asymmetric** (issue #282, **breaking**:
  `q65::search::SearchParams::time_tolerance_symbols: u32` →
  `time_tolerance_early_sec` + `time_tolerance_late_sec: f32`,
  defaulting to 1.0 / 5.5).

  The window was `time_tolerance_symbols: 5`, and symbols are not a
  transferable unit: Q65 symbols run 0.15 s (Q65-15) to 3.456 s
  (Q65-300), so one constant silently meant ±0.75 s on the shortest
  sub-mode and ±17.3 s on the longest.

  Measured by running real `jt9 -3 -d 3` over `q65sim` Δt sweeps:

  | sub-mode | reference window |
  |---|---|
  | Q65-15A (`nsps=1800`) | −1.0 … +1.0 s |
  | Q65-30A (`nsps=3600`) | −1.0 … +1.0 s |
  | Q65-60A (`nsps=7200`) | −1.0 … **+5.5 s** |

  `q65.f90:127-129` extends `lag2` to `5.5/dtstep` when
  `nsps >= 3600 .and. emedelay > 0`. The measurement says that
  extension is live for TR≥60 and not for TR=30 — which the
  `nsps >= 3600` half alone does not explain, since Q65-30A *is*
  `nsps=3600` — so these constants follow the measurement rather than
  the source, per `tests/dt_window.rs`'s own doctrine. +5.5 s is
  applied uniformly rather than gated on NSPS: on the short sub-modes
  the extra span is geometrically self-limiting (a Q65-15 frame placed
  +5.5 s late does not fit in a 15 s slot, so those rows are rejected
  by the frame-fits guard for the cost of a scan).

  Result, on the same sweeps — Q65-60A now matches the reference
  exactly, and Q65-15A slightly exceeds it:

  | | Q65-15A | Q65-60A |
  |---|---|---|
  | `jt9` | −1.0 … +1.0 | −1.0 … +5.5 |
  | before | −0.75 … +0.75 | −3.0 … +3.0 |
  | after | −1.0 … +1.5 | −1.0 … +5.5 |

  **This default has now been wrong twice in one issue.** The first
  fix replaced the symbol unit with a symmetric `time_tolerance_sec:
  1.0`, which fixed Q65-15 and cut Q65-60A's late reach from +3.0 to
  +1.0 against a reference that reaches +5.5 — a regression on the
  sub-modes EME uses, where multi-second Δt is normal. Both mistakes
  survived a green suite because **every in-tree Q65 test passes
  explicit tolerances and none exercised the default**;
  `dt_window.rs::q65_60a_default_window_reaches_reference_late_edge`
  and its Q65-120D sibling now do, and both are verified to fail at
  the symmetric value.

  The long sub-modes were checked too, since a uniform +5.5 s late is
  a *narrowing* for them (the old symbol unit gave Q65-120 ±6.7 s and
  Q65-300 ±17.3 s). Measured harmless — the reference itself only
  reaches ~+2.0 s there, and this crate covers +5.0 s:

  | | Q65-120D | Q65-300A |
  |---|---|---|
  | `jt9` | −1.5 … +2.0 | −1.5 … +2.0 |
  | after | −1.5 … +5.0 | −1.5 … +5.0 |

  Q65-60B/C/D/E need no separate measurement: the window is
  denominated in seconds and tone spacing does not enter it, so they
  are identical to Q65-60A by construction (all `NSPS = 7200`).

  A note on method, since these edges are quoted as if they were
  crisp: **they move with SNR.** Q65-120D measured −1.0…0.0 at −25 dB
  and −1.5…+2.0 at −12 dB, because near the edge only part of the
  frame is inside the slot and sync degrades gradually rather than
  cutting off. A *positive* result at low SNR is therefore solid
  evidence the window reaches that far; a failure is ambiguous between
  the window and plain sensitivity.

  Guarded by `q65_a15_roundtrip::q65_15a_default_window_covers_wsjtx_plus_one_second`,
  which decodes at Δt = +1.0 s under the *default* `SearchParams` —
  verified to fail at the old ±0.75 s window.

  **FST4 was audited in the same pass and needs no change.** WSJT-X's
  `fst4_sync_search` covers Δt −1.0…+2.0 s; this crate covers
  −2.0…+3.0 s and decodes correctly across all of it, matching `jt9`
  5/5 per cell down to −24 dB. Ours is a strict superset, so clamping
  it to the reference would remove working range, not add fidelity.

- **FT8's coarse-sync lag window is WSJT-X's own ±2.5 s again**
  (`SYNC_LAG_S_DEFAULT`, issue #280). It had been ±1.0 s as an embedded
  compute trade, on the assumption that the narrower window cost only
  time. It didn't: at ±2.5 s `qso3_busy.wav` lost `K1BZM DK8NE -10`
  and `K1JT HA5WA 73`, so the narrow window was quietly load-bearing
  for golden recall. All seven FT8 golden/recall suites now produce
  identical results at either width (8/8 WSJT-X golden, 18/18 JTDX,
  20/20 JTDX-High, exact `sic_rounds` golden-set match).

  Root-caused by probing the real `jt9` binary, not by reasoning about
  our port. Temporary `write(0,…)` probes in `sync8.f90` /
  `ft8_decode.f90`, run over the same recording, settle three things
  the issue had been guessing at:

  - `K1BZM DK8NE` reaches WSJT-X **only through the secondary
    (full-`±JZ`) channel** — `red2 = 2.92-3.85` at `jpeak2 = 14`. Its
    fixed-`±mlag=10` primary channel scores `0.95`, far under
    `syncmin = 1.3`, at `jpeak = -7`. The fixed-window primary channel
    restored in #281 is faithful to `sync8.f90`, but it is structurally
    incapable of producing this decode — on either side.
  - It is **not a single-pass catch**: WSJT-X decodes it at
    `nzhsym=50, ipass=2, ndeep=3`, after 18 earlier decodes have been
    subtracted — the same multi-pass SIC shape this crate's
    `decode_block_multipass` runs.
  - WSJT-X applies **no pass-1 truncation at all** (`MAXCAND=600`,
    `ft8_decode.f90:217-228` calls `ft8b` on every `sync8` candidate in
    coarse-score order). It decodes `DK8NE` from rank 35 of 337.

  Ours produced the same candidate at both window widths
  (243.75 Hz, `dt +0.466`); what dropped it was `PASS1_LIMIT_DEFAULT`.
  Instrumenting our own passes: at ±1.0 s the candidate's rank climbs
  57 → 34 → 20 over the three SIC passes and enters the top-30 in time;
  at ±2.5 s it climbs 93 → 61 → 38 and never does.

- **`coarse_sync` is no longer asked for fewer candidates than the
  caller's own `max_cand`** (`pass1_limit_for`). Pass 1 capped at 30
  regardless, so a host caller passing `max_cand=60` silently got 30 —
  and once `PASS1_LIMIT > max_cand`, `refine_candidates`' `sync_quality`
  re-rank becomes the binding truncation instead of an inert
  pass-through. That re-rank is lossy and has no WSJT-X analog:
  measured on `qso3_busy.wav`, `PASS1_LIMIT=150` with `max_cand=60`
  drops three golden entries (8/8 → 6/8) that the same run keeps at
  `PASS1_LIMIT = max_cand = 150`. With both raised together, recall is
  flat at 8/8 out to 400 candidates.

  Decode-identical at the old ±1.0 s window (8/8 golden + 12 phantom +
  20 total; ship config 7/8 + 7 phantom + 14 total). Embedded callers
  pass `max_cand ≤ 15`, so their pass-1 cap is unchanged at 30. Host
  full-parity config costs 113 → 162 ms on a Ryzen 9 9900X — still
  ~7× faster than `jt9 -8 -d3`'s own ~1.1 s on the same file.

### Added

- **`coarse_sync_with_lag` / `coarse_sync_with_allsum_and_lag`** — the
  ±lag search window as an explicit argument rather than a crate-wide
  default. The window is not purely a speed knob: it changes which
  candidates exist and how they rank, so a caller whose downstream use
  carries its own timing assumptions should state them.
  `allsum` is indexed by `(fi, m)` only, so one table stays valid at
  any width.

### Changed

- **`bootstrap_dt_median`'s contract now names its lag window.** The
  cold-start slot-alignment helper takes a top-K DT median, which is
  only meaningful over a window comparable to the timing error being
  estimated. Over a ±2.5 s list its top-5 median on `qso3_busy.wav`
  lands 1.9 s from truth, because a strong signal's far-lag ghost can
  outscore its own true-lag peak and crowd the top ranks. Real `jt9`
  shows the identical ghosts at the identical frequencies (2534.38 Hz
  scoring higher at `xdt=+2.38` than at its true `+0.14`) — WSJT-X just
  never takes a top-K statistic over that list. Both in-tree consumers
  (`ft8_coarse_sync_bootstrap`, `embedded-shared::dual_core`) now pin
  ±1.0 s explicitly, and a new test asserts the wide-window divergence
  so the boundary can't be silently "simplified" away.

  `embedded-shared` pins it for a second, independent reason: the
  streaming pipeline's `stage1_inc::SPEC_EMIT_PAIR` is derived from
  `SYNC_LAG_S=1.0 → jz=13` (`needed_m = 162 + 13 = 175`). At ±2.5 s
  `jz` is 31 and `needed_m` runs past `N_TIME=184` entirely.

- `ft8_coarse_sync_bootstrap`'s host-f32 budget 100 → 130 ms, matching
  the `fixed-point` path. The estimator did not get worse — its K=5
  answer on `191111_110130.wav` is unchanged at +0.820 s. The
  *reference* moved: the wider decode window finds a 6th decode there,
  shifting the confirmed-decode median +0.910 → +0.940 s. The old
  budget had 10 ms of headroom on that fixture.

- #281 (merged after 0.9.1, no entry at the time): `coarse_sync_inner`
  now runs WSJT-X's two independent noise-floor channels — a primary
  anchored to a fixed `±MLAG=10` window regardless of `jz`, and the
  full-`±jz` secondary contributing a second candidate per bin only
  when its peak lag disagrees (`sync8.f90`'s
  `if (jpeak2(n)==jpeak(n)) cycle`). #279 (from @nicksbar) restored the
  trailing-block lag bounds that made a widened window safe in the
  first place.

### Fixed

- **Q65's cross-candidate dedup window now scales to tone spacing**
  (issue #287, no entry at the time). It was a fixed ±4 Hz regardless
  of sub-mode — tighter than one tone spacing on the wide sub-modes
  (Q65-...D/E), so two lobes of a single Doppler-spread signal could
  each independently survive coarse search's own local-max suppression
  (±1×`TONE_SPACING_HZ`, matching WSJT-X's `q65_ccf_22` admission rule)
  while still being more than one tone spacing apart, and dedup — being
  narrower — didn't catch it. Measured on Q65-120D 10 GHz rainscatter
  (`210117_0920.wav`, `TONE_SPACING_HZ=6.0`): two candidates 7.4 Hz
  apart both decoded the identical message. Now `(2 ×
  TONE_SPACING_HZ).max(4.0)`, applied at all three dedup call sites via
  a shared `dedup_freq_tol_hz<P>()` helper; message-text equality
  (CRC-protected) remains the real safety net against merging distinct
  signals, so widening the window carries negligible risk.

- **WSPR's `pack_call` was missing three slot-alphabet checks**
  `packjt.f90:97-116` performs (slot 1 must not be a space, slot 2 must
  be a digit, slots 3-5 must not be digits) — found incidentally while
  extracting the shared `msg::callsign28` core below. Only ever
  affected malformed input; every real callsign this crate's golden
  corpus exercises was already well-formed, so this changes no golden
  test's output, only what previously-invalid input now correctly
  rejects instead of silently mis-encoding.

### Changed

- **Code-sharing audit** (issues raised by a "does faithful WSJT-X
  porting cause excess per-protocol divergence?" review): the
  README/crate-doc's unsourced "~80% shared" claim (fixed above) turned
  out to have the right instinct but the wrong cause — most of the
  crate's protocol-bound code is *genuinely* protocol-bound (QRA, Fano,
  Reed-Solomon and LDPC are different algorithms, not one thing written
  four times, and WSJT-X's own Fortran repeats itself per-protocol the
  same way), but a handful of shared mechanisms had quietly stalled at
  2-3 adopting protocols with nothing flagging the rest. Consolidated,
  each verified byte-identical (or behaviour-preserving modulo an
  explicit per-protocol parameter) against golden tests before and
  after:

  - `engine::sync::refine_freq_hz_log_power` — a scalloping-loss fix
    independently reinvented three times (WSPR, then JT65 issue #169,
    then JT9), now one function JT65 and JT9 both call.
  - `engine::pipeline::scan_dedup_match` / `scan_dedup_match_cross` —
    9 candidate-dedup call sites across JT9/JT65/WSPR/Q65, each
    protocol's own frequency/time tolerance preserved as an explicit
    argument rather than a silently-different inline constant.
  - `engine::spectrogram::Spectrogram` — a 3x-duplicated coarse-search
    kernel (JT9/JT65/Q65) found by diffing actual file contents rather
    than a category-level "this looks shareable" audit, which had
    missed it entirely. WSPR keeps its own (a real difference: a
    fixed-point FFT backend plus baseline-fit normalisation).
  - `engine::interleave::{interleave_bitrev, deinterleave_bitrev}` — a
    5x-duplicated bit-reversal permutation (WSPR ×3, including a
    self-documented "duplicate of a duplicate", plus JT9 ×1), tracing
    to one upstream routine (`packjt.f90`'s `packcall`/`unpackcall`
    interleave, also called from `wsprcode/wspr_old_subs.f90`). JT65's
    own interleave is a different algorithm (matrix transpose) and is
    correctly untouched.
  - `msg::callsign28::{pack_call28, unpack_call28}` — WSPR's own third
    independent copy of the same base-37/36/10/27³ callsign encoding
    JT9 and JT65 already shared via `msg::jt72`.

  A permanent `sharing_ratchet_selftest` (in `tests/common_selftest.rs`)
  now fails if a protocol that adopted one of these loses the evidence,
  without blocking non-adoption elsewhere — the failure mode this audit
  exists to catch is a mechanism quietly built and then only ever used
  by 2-3 of the 8 protocols.

  Migrating JT9/JT65/WSPR/Q65 onto the generic `engine::pipeline` (the
  step that would have consolidated the most) turned out to be
  architecturally blocked, not merely unstarted: the pipeline requires
  `P::Fec: BpPooledFec`, a belief-propagation scratch-reuse shape only
  the two LDPC codecs implement — Fano-sequential, Reed-Solomon and
  GF(64) QRA codes have no equivalent operation. Closed without a
  numeric sharing target by explicit choice: the audit's own honest
  ceiling under either measure (protocol-generic vs. shared-directory
  line count) is capped by that same wall.

  Also removed `jt9::demod_bb` (606 lines), the box-car demod path
  `softsym.rs` superseded in 0.5.9 — self-documented as due for removal
  once issue #19 closed, which it had, months earlier.

- **FT8's OSD-escalation threshold is now ratchet-tested against the
  generic FT4/FST4 gate** (issue #285, split from #192, closed). FT8's
  own `Q_NDEEP3_THRESHOLD = 18` (`ft8::decode_block::osd_strategy`) and
  `engine::pipeline::osd_escalation_gates<P>` (FT4/FST4's equivalent —
  its default-branch value, which `osd_escalation_gates::<Ft8>()`
  turns out to already reach cleanly, since the function only requires
  `P: Protocol`) are two independently-tuned implementations of the
  same decision that happened to agree only because neither had been
  retuned recently. A new in-crate test asserts they match, so a
  future silent divergence fails CI instead of relying on a doc
  comment being reread — verified the ratchet actually catches drift
  before shipping it. The blind-CQ pair (`BLIND_CQ_MIN_NSYNC` /
  `msg::pipeline_ap::ap_passes`'s pass 7) has no equivalent paired
  numeric threshold to ratchet the same way; documented why instead.
  Deliberately did not attempt reconciling the two OSD-gate *shapes*
  (FT8's single threshold vs. the generic `(low, high)` pair) — same
  WSJT-X-fidelity regression risk the issue itself flagged, the
  reason #192's narrower version of this proposal was rejected.

## 0.9.1 — WSPR parity with `wsprd` (#275) + phantom elimination, TX envelope ramps (#259), FT4 sniper aim (#257), SNR formula close-out (#255), test taxonomy rework, FST4/Q65 sub-mode coverage

### Fixed

- **WSPR reached full parity with `wsprd`** (issue #275, closed). The
  WSJT-X golden `150426_0918.wav` goes **7/9 → 9/9** — every spot the
  reference decoder reports on that file — with 0 phantoms, and the
  AWGN sensitivity curve now matches `wsprd` cell for cell (same
  corpus, reference decoder run over the identical WAVs):

  | SNR | wsprd | before | after |
  |---|---|---|---|
  | −28 dB | 20/20 | 20/20 | 20/20 |
  | −29 dB | 20/20 | 11/20 | 20/20 |
  | −30 dB | 20/20 | 4/20 | 20/20 |
  | −31 dB | 16/20 | 0/20 | 16/20 |
  | −32 dB | 4/20 | 0/20 | 5/20 |

  Six faithful ports, no tuning. In descending order of effect:

  - **The Fano branch metric was linear in the LLR.** `wsprd` uses a
    *measured* log-likelihood table (`metric_tables.c` row 2, selected
    at `wsprd.c:912-913`) indexed by the soft symbol after normalising
    it to `[0, 255]` (`wsprd.c:472-481`). The table is strongly
    asymmetric and saturating (`T[0] = +1.0`, `T[255] = −13.25`); a
    linear metric cannot express that, so one confident-but-wrong
    symbol dominates the path metric and Fano — a sequential search —
    settles on a *wrong codeword* rather than failing. WSPR has no CRC,
    so nothing downstream catches it. Diagnosed by dumping `wsprd`'s own
    `symbols[162]` and feeding it to our decoder: before, it returned
    `W65/G9MNK 33`; after, `G8VDQ IO91 37`, `wsprd`'s own answer.
    Normalising inside the metric build also makes it scale-invariant,
    retiring the constraint recorded in `METRIC_BIAS`'s old comment
    ("would need an LLR-normalisation pass before Fano — deferred").

  - **The final pass was conditional on the earlier passes decoding
    something.** `wsprd.c:999` skips *pass 1* when pass 0 comes up
    empty, never pass 2 — a slot where the early passes found nothing is
    exactly the one that needs the final pass's coherent-block ladder
    and zero-drift estimate. −31 dB goes 0/20 → 16/20 on this alone.

  - **No DT peak-up loop.** `wsprd.c:1321-1327` retries the whole
    demod-and-decode step at `shift1 ± 8k`, `k = 0..8`, as the *inner*
    loop, so every rung gets all seventeen positions. Not redundant with
    the refine cascade: the cascade maximises *sync*, and the alignment
    Fano converges from is not always the one with the best sync —
    `wsprd` wins `G8VDQ` at `shift1 + 16` having scored position 0
    higher. Worth ~1 dB.

  - **Subtraction discarded the decoded drift.** `subtract_signal_baseband`
    has always taken `drift_hz`; both call sites passed `0.0`, where
    `wsprd` passes `drift1` (`wsprd.c:1446`). A stationary replica walks
    away from a drifting signal across the 110.6 s frame, so most of it
    survives. Every station on the golden decodes at −0.5 or −1.5 Hz
    drift, so *every* subtraction left residue, and the leftovers sat on
    the weakest signal in the band. This is the one that recovers
    `G8VDQ IO91 37`.

  - **Two passes instead of three**, and the per-pass configuration was
    wrong in both directions (`wsprd.c:998-1010`): passes 0/1 use
    `maxdrift = 4` and discarded the coarse drift estimate; the final
    pass uses `maxdrift = 0` ("no drift for smaller frequency estimator
    variance") and we were handing it 4. Fixing the latter is what
    recovers `W3BI FN20 30` at −25 dB, which stops being an OSD-only
    decode.

  - **Three of five refine stages missing** (`wsprd.c:1221-1272`): drift
    refine (`drift ± 0.5`), fine lag (±32, step 16) and fine freq
    (±0.1 Hz, step 0.05), plus coarse freq running at ±1.0 Hz/0.5 Hz
    instead of ±0.5 Hz/0.25 Hz. Also added `wsprd`'s `minrms`
    plausibility gate (`wsprd.c:1338-1345`) and the fourth demod rung
    `bitmetric` (`wsprd.c:465-468`), completing the blocksize ladder.

  Cost: the golden slot goes ~0.15 s → ~0.76 s, against a 120 s slot.

  The sync scorer was *not* at fault, contrary to an intermediate
  diagnosis recorded on the issue: `sync_score_isqs` matches
  `wsprd.c:280-320` in formula, magnitude-not-power convention and
  normalisation.

- **FST4-15/30/120/300 were never decoded in CI**, and two mechanisms
  hid it. `tests/fst4_sim_roundtrip.rs` reported "5 passed" in 0.00 s
  with no corpus — its WAVs are generated and untracked, and its skip
  path predated `MFSK_REQUIRE_CORPUS` — while the in-crate synth
  roundtrips covering every sub-mode were gated behind an opt-in
  `RUN_FST4_ROUNDTRIP=1` on the grounds of being slow (measured: 2.4 s
  for all five). The synth roundtrips are now un-gated and the
  corpus-dependent sim roundtrips are tier C, `#[ignore]`d and wired
  into `scripts/run-sensitivity-sweeps.sh`. Q65 was audited for the
  same defect across its ten sub-modes and does not have it.


- **WSPR emitted one phantom decode for every real one** (50 % false-
  decode rate). On the WSJT-X golden `150426_0918.wav` — 9 real
  signals — `decode_scan` returned 16 decodes: 8 real and 8 invented,
  with callsigns like `UZC/7D0DKY`, `IWR/4B2BBE`, `05S/C30EQG`. Real
  `wsprd` reports 9 real and 0 phantom on the same audio. Reported
  from live operation, where the symptom was a decode list dominated
  by nonsense.

  All 8 came from the **OSD** path; the Fano path produced none
  (verified by sweeping `ConvFano::METRIC_BIAS` 1.0→6.0, which changed
  nothing, then by disabling OSD, which removed every phantom). OSD
  synthesises a valid codeword for *any* input, and WSPR has no CRC,
  so the only thing standing between it and a well-formed callsign was
  an `nhardmin ≤ 44` threshold. **That threshold cannot work**: the
  one genuine OSD decode (W3BI, -25 dB) lands at `nhardmin = 39` and
  the phantoms at 40/40/40/41/41/41/42/42 — a separation of one hard
  error.

  Now gated the way `wsprd.c:1396` gates it — structurally rather than
  by threshold. New `wspr::decode::WsprCallsignTable` records
  Fano-confirmed callsigns, and an OSD result is accepted only for a
  callsign already in it, so OSD can re-find a known station but never
  invent one. Result on the golden: **7 real / 0 phantom**.

  W3BI is OSD-only and unreachable by Fano in that file, so a single
  isolated slot loses it. Real `wsprd` keeps it because its `hashtab`
  persists across slots and invocations; new
  `wspr::decode::decode_scan_with_table` gives callers the same
  ability — feed one `WsprCallsignTable` back each slot and a station
  confirmed once stays recoverable (verified: seeding W3BI restores
  8 real / 0 phantom, while seeding an absent callsign changes
  nothing). Guards: `wspr_wsjtx_sample_has_no_phantom_decodes` and
  `wspr_carried_table_recovers_osd_only_decode`.

- **No transmit-envelope ramp on WSPR, JT65, JT9 and Q65** (issue
  #259, reported from a WebFT8 WSPR-beacon evaluation). Every one of
  these `synthesize_audio` paths wrote `amplitude · cos(phase)` from
  the first sample to the last, so a transmission began and ended on a
  step discontinuity in the envelope — a broadband click at both
  edges, independent of symbol-transition shaping. Measured on WSPR,
  windowing the burst *start* embedded in silence (16384-pt Hann,
  dBc relative to the in-band peak): +100 Hz `-46.2 → -55.2`,
  +250 Hz `-53.9 → -101.7`, +500 Hz `-59.5 → -100.3`, +1000 Hz
  `-64.7 → -118.4`. Close-in (+50 Hz) barely moves, correctly — that
  region is set by the CPFSK symbol structure, not the transient.
  New `engine::dsp::envelope` applies the same raised-cosine shape
  `gen_ft8wave.f90` / `gen_fst4wave.f90` use, so all transmit paths in
  the crate now taper identically. Ramp is 10 ms (bracketed by WSJT-X's
  own choices — its modulator's exponential fade-out reaches -60 dB in
  ≈7.1 ms, `gen_ft8wave`'s `nsps/8` is 20 ms), capped at `nsps/8`.
  No decode regression: full suite clean, and WSPR/JT65/JT9/Q65
  roundtrips unaffected. Guard: `tests/tx_envelope_ramp.rs`.

  **GFSK symbol shaping was deliberately *not* added**, though #259
  requested it and measured a real gain (`-53.8 → -85.9` dBc at
  +25 Hz for a T/8 pulse). WSJT-X does not shape these four either:
  `mainwindow.cpp` passes a *positive* `toneSpacing` for WSPR/JT65/
  JT9/Q65, selecting `Modulator::modulate`'s plain-CPFSK branch rather
  than the `toneSpacing < 0` "pre-computed, filtered waveform" branch
  FT8/FT4/FST4 use — so the measured `-53.8` dBc is the reference
  implementation's own figure, and shaping here would emit a different
  waveform than WSJT-X. FT8/FT4/FST4 were audited and already match
  WSJT-X on both shaping and ramp.

### Verified

- **FST4 SNR confirmed across all five sub-modes** (issue #255 §4
  follow-up). `fst4::baseline::fst4_snr_db` shipped FST4-60-verified
  only — the one sub-mode with a real off-air recording available
  locally — with 15/30/120/300 left as "share the same formula but
  aren't individually confirmed". Now closed against the `fst4sim`
  corpus, with real `jt9 -7` confirming injected SNR is a valid
  reference (within ~1 dB on every sub-mode). Mean error, AWGN /
  CCIR-moderate: FST4-15 -0.45/-0.35, FST4-30 +0.43/-0.01, FST4-60
  -0.01/-0.44, FST4-120 -0.19/+0.07, FST4-300 **-1.26/-1.92**. Four of
  five inside ±0.5 dB including under fading; FST4-300 carries a real
  SNR-independent ~1.3 dB offset, *not* a wrong parameter (`nsps`,
  `ndown`, `snr_calfac` all verified identical to
  `fst4_decode.f90:182-214,597-613` for every sub-mode) but most
  likely the `xsig · NDOWN` scale correction, which was derived on
  FST4-60 — the sub-mode that now reads -0.01 dB. Recorded as a
  measured residual rather than fitted away. No code change; new guard
  `tests/fst4_sweep.rs::fst4_reported_snr_tracks_injected_all_submodes`.

- **JT65's reported-SNR clamp was ad-hoc `[-24, +49]`, not WSJT-X's
  real `[-30, -1]`** (issue #255). `jt65_decode.f90:254-255` pins the
  displayed value to `[-30, -1]`, and both ends of the old pair were
  wrong in opposite ways: the `-24` floor bound before WSJT-X's own
  `-30` did, truncating the weakest decodes, while JT65 is the one
  protocol here whose display **saturates by design** — verified
  directly against a real local `jt9 -6 -b A` build, a `jt65sim`
  signal injected at +10 dB and one at +5 dB both come back `-1`.
  The underlying estimator was **audited and deliberately left
  alone**: issue #255 listed JT65 as running on the generic
  `engine::llr::compute_snr_db` adjacent-tone heuristic, but
  `jt65::rx::demodulate_aligned_with_confidence_and_snr` has always
  had its own, carrying a real `10·log10(2500/TONE_SPACING_HZ)`
  bandwidth offset — and over a 283-decode `jt65sim` corpus it lands
  within **±0.7 dB of real `jt9` across -22…-10 dB**, where JT65
  operates. Porting `sync2 = 3.7e-4·ccfbest/sq0` (`decode65a.f90:55`,
  an empirical constant on a coherent-AFC cross-correlation this crate
  computes differently) to replace an already-accurate number was not
  worth it. New guard
  `tests/jt65_sweep.rs::jt65_reported_snr_tracks_injected`; details in
  `docs/notes/SNR_FORMULAS.md`.

- **JT9's reported SNR was ~32 dB high** (issue #255). `jt9::softsym::
  symspec2_from_ss2` ports WSJT-X `symspec2.f90`, but stopped three
  lines short of that subroutine's own displayed-SNR formula
  (`symspec2.f90:52-54` — `sig = sig/69`, `t = max(1, sig-1)`,
  `snrdb = db(t) - 61.3`, reached via `jt9_decode.f90:148`'s bare
  `nsnr = nint(snrdb)`). A generic signal/noise power ratio stood in
  for it, reading **+31.8 dB high on average** against real `jt9` on
  `WSJT-X/samples/JT9/130418_1742.wav` — e.g. `+16.9` dB reported for
  `TF3G N7MQ CN84`, which real `jt9` calls `-18`. Now +0.33…+2.86 dB
  (mean +1.3), residual largest on the strongest signal. The formula
  reads `sig` in the raw `ss2` scale, *before* the `ss3 /= ave`
  normalisation, so the `-1.0` and `-61.3` together carry the absolute
  scale of the whole `downsam9` → `peakdt9` → coherent-sum chain; it
  works unmodified only because this crate's port of that chain is
  itself scale-faithful, confirmed by instrumenting a real local
  `jt9`'s own `symspec2.f90` with a `write(0,...)` probe and matching
  its `sig` magnitude range. `Jt9Result::snr_db`'s doc comment
  previously said deriving the real value "needs either WSJT-X's own
  JT9 SNR formula or an empirical `jt9sim` corpus (unavailable in this
  environment)" — both halves were stale; the formula is in the file
  already being ported. Regression test
  `tests/jt9_wsjtx_samples.rs::jt9_wsjtx_sample_snr_matches_real_jt9`.

- **FT4 sniper mode was blind to signals 15-99 Hz off the aim point**
  (issue #257, reported from a WebFT8 downstream synthetic harness).
  `SniperRequest::<Ft4>` asks `coarse_sync` for a ±250 Hz search
  window, but in practice decoded only within ~±14 Hz of
  `target_freq` or beyond ~±100 Hz — returning nothing at all in
  between, at any SNR, on buffers wide-band decoded 8/8. Root cause
  was `coarse_sync`'s `freq_hint` *ranking*, not its scoring: the
  hint applied strict lexicographic precedence ("within 10 Hz of the
  aim point" first, score only as a tie-break). Sniper runs at
  `sync_min = 0.8` with `max_cand` clamped to 15, and `coarse_sync`'s
  per-bin NMS emits up to 8 lag peaks per frequency bin — so on FT4
  (`df` = 5.21 Hz, three bins inside ±10 Hz) the aim point alone
  produced more than 15 candidates, nearly all noise-floor lags
  scoring ~1.0. They took every slot, and the real signal 16-99 Hz
  away — scoring 12-17, by far the strongest candidate in the band —
  was truncated off the list before any decoder ever saw it. The
  annulus's edges follow from that: below ~14 Hz the promoted
  aim-point candidates are within `refine_candidate_position`'s own
  ±12 Hz frequency pull-in, and beyond ~100 Hz (just past FT4's
  83.3 Hz occupied bandwidth) the aim-adjacent bins stop catching the
  signal's energy, fall under `sync_min`, and stop crowding the list
  on their own. New `engine::sync::rank_candidates` reserves the aim
  point at most *half* the candidate budget and fills the rest by
  score, keeping the hint's actual intent (a weak signal at the aim
  point should not be ranked out by stronger QRM elsewhere in the
  window) without the starvation; `engine::ft4_coarse::ft4_coarse_sync`
  now shares it so the two coarse-sync paths cannot disagree.
  Regression test `tests/ft4_sniper_aim_offset.rs` covers the former
  annulus (16/20/40/60 Hz) and the offsets that already worked
  (0/12/100 Hz); it fails on the parent commit. Note the issue's own
  `max_cand` sweep (15/30/50/100/200) was inert — `Ft4::__sniper`
  clamps `req.max_cand.min(15)` — which is why raising it looked like
  it made no difference.

- **`--features fst4` (without `ft4`) failed to build** — `ft4_snr_db`
  and `SnrCtx::cand_score`, added by issue #255's FT4 work, are read
  only by FT4's `snr_db` override, so that CI feature-matrix cell hit
  `-D dead-code`. Silenced under `cfg(not(feature = "ft4"))` rather
  than `cfg`'d away, so the intra-doc links to `ft4_snr_db` from
  `GenericPipelineProtocol::snr_db` keep resolving under every
  feature set.

- **FT4/FST4 AP-hint decode path (`msg::pipeline_ap::process_candidate_ap`)
  used a coarser, non-frequency-correcting refine than the wide-band
  path, and skipped the RMS-normalisation `compute_llr`'s scale
  calibration expects** (issue #255 follow-up, prompted by a WebFT8
  downstream deployment report on `dd934b8`/`e1200b6`). This path
  previously called the generic, time-only `refine_candidate` (no
  frequency correction) on a raw, non-normalised `cd0`; wide-band's
  own `process_candidate_basic_impl` has used FT4/FST4's real 2-D
  (frequency + time) coherent refine (`refine_candidate_position`,
  `engine::sync2d::ft4_sync_search`/`fst4_sync_search`) plus RMS
  normalisation for a while. Now shared via the same function.
  A companion fix (swapping this path's *coarse* candidate search
  from generic `coarse_sync` to `ft4_coarse_sync`, so `ft4_snr_db`
  would see the score formula it's actually written against) was
  tried and reverted after a real regression: `ft4_coarse_sync`'s own
  coarse-frequency estimate is only accurate to its own ~78 Hz
  smoothing width, fine for wide-band search but not for a narrow,
  single-target sniper search — a controlled test showed it locking a
  clean 1000 Hz synthetic signal's only nearby candidates 60+ Hz away,
  outside the ±12 Hz fine-refine capture range, losing the decode
  entirely. Left as a known, documented gap (`process_candidate_ap`'s
  own doc comment): the AP/sniper path's reported SNR still uses
  `coarse_sync`'s Costas-correlation score rather than
  `getcandidates4.f90`'s own value, which the wide-band path already
  gets right. Zero regression: full `cargo test --release --features
  full,internal-testing --all-targets` clean (74 binaries, 0 failed).

- **FT8 `.sic_early()` false decode, actually eliminated** (issue #253
  follow-up) — the `Deep`-threshold retune below turned out not to fix
  the reported anecdote (`hard_errors=31` clears even `Normal`'s 36);
  further investigation traced it to the wrong layer entirely.
  Root cause: `__staged_sic`'s checkpoint-A/B/C engine used to
  pre-subtract the caller's `.known()` list from the *entire* audio
  buffer once, before checkpoint A (the `nzhsym=41` truncated/
  zero-tailed early pass) ever ran. A real `jt9` build's own disk-read
  "Early" pass is *always* the first decode attempt on fully raw,
  unmodified audio for a Rx cycle (`jt9.f90` builds its zero-tailed
  buffer straight from the freshly-read WAV) — there is no WSJT-X code
  path where checkpoint A ever sees audio some earlier, external pass
  has already subtracted from. Feeding it pre-subtracted audio is an
  mfsk-core-original composition with no WSJT-X counterpart to validate
  against, and it produced a real CRC-14 false-accept
  (`7Y8CIH HN1GD OP30` @509 Hz on `qso3_busy.wav`, reproduced via
  WebFT8's `decode_phase2` pipeline: `Deep` + `.known()` +
  `.fft_cache()` + `.sic_early()`). Confirmed via a controlled A/B
  (identical `sync_min`/`max_cand`/strictness, only variable was
  whether `known` was pre-subtracted before checkpoint A): with
  pre-subtraction, 7 results including the false one; without, 22
  results — a strict superset of the pre-subtraction run's *real*
  signals, none false. Cross-checked against a real local `jt9 -8 -d3`
  build on the same file: 9 clean decodes, nothing near 509 Hz.

  Fixed by scoping the `known` pre-subtraction *away* from checkpoint A
  specifically, keeping it for checkpoints B/C only (which already
  rebuild their own buffers fresh from the original audio each call,
  never reusing checkpoint A's discarded residual — routing *that*
  fresh copy through a `known`-subtracted buffer instead of raw audio
  restores the "known signals don't mask weaker ones underneath"
  capability without ever exposing checkpoint A to anything
  WSJT-X-unvalidated). New regression test
  `sic_early_deep_with_known_and_cache_does_not_false_decode`
  (`tests/ft8_qso3_staged_sic_check.rs`) replays the exact WebFT8
  pipeline shape and asserts both that `7Y8CIH` never reappears *and*
  all 6 real incremental signals (including `CQ DX DL8YHR JO41`,
  issue #191's own regression signal) still decode. Zero regression:
  full `cargo test --lib --features full` (392 passed), every
  `qso3_busy.wav` recall suite unchanged from pre-fix numbers,
  `scripts/pre-push-check.sh`'s full feature matrix clean.

- **FT8 `DecodeStrictness::Deep` false-decode ceiling calibration**
  (issue #253) — a separate, independently-justified fix alongside the
  one above. New `ft8_strictness_probe` (`tests/ft8_sweep.rs`, mirrors
  issue #72's FT4 methodology) swept `DecodeStrictness`'s effect on
  both golden recall and false-accept count across 320 `ft8sim`
  AWGN/CCIR trials at Strict/Normal/Deep, across both the single-pass
  and `.sic_early()` strategies — found `Deep`'s golden-recall gain
  over `Normal` saturates entirely by `ft8_nharderrors_max=37`
  (bit-for-bit identical 37→40) while false-accepts keep climbing past
  that point with zero further benefit. Retuned `Deep`: `40 → 37`.
  Doesn't by itself explain the anecdote above (see that entry), but
  stands on its own sweep evidence regardless.

- **FT8 host `xsnr2` SNR systematically under-reported vs WSJT-X**
  (issue #253 follow-up) — `qso3_busy.wav`'s host-f32 SNR used to sit
  ~3-7 dB below WSJT-X/JTDX (a long-standing, previously-undiagnosed
  gap; `SNR_TOL_DB` in the `qso3` recall tests was widened to 12 dB
  years ago to tolerate it rather than fix it). Root-caused to two
  independent, *compounding* non-faithful simplifications, both found
  by comparing against a real local `jt9 -8 -d3` build's own internal
  values (via a temporary `SNRAUDIT_PROBE` instrumented directly into
  `ft8b.f90`/`ft8_decode.f90`, not just its final display — essential,
  since real `jt9` doesn't even decode 2 of the JTDX-golden's most
  divergent entries, so their JTDX-reported SNR alone couldn't be
  trusted as ground truth either):

  1. `xsig` (signal power) was read from `compute_spectrogram`'s
     rectangular-window coarse-sync spectrum. WSJT-X's real `xsig`
     comes from an entirely different pipeline — the `cd0`/per-symbol
     32-point-FFT chain (`ft8_downsample` + `ft8b.f90:154-161`) that
     also produces the soft-symbol LLRs. mfsk-core already has a
     faithful port of that exact pipeline (`fill_symbol_spectra`'s
     `fill_symbol_spectra_via_cd0`) — it just wasn't being reused for
     `xsig`, only for LLR (where an absolute-scale bug is invisible:
     LLR is a same-candidate relative comparison, `.sic_rounds()`
     never had anything to catch this).
  2. `xbase` (noise-floor baseline) was fit from that same rectangular
     spectrum too. WSJT-X's real `sbase` comes from
     `get_spectrum_baseline.f90` — a *dedicated* Nuttall-windowed, 50%-
     overlap spectrum, deliberately not `sync8.f90`'s own rectangular
     one, for much lower far-sidelobe leakage from other, frequency-
     distant signals on a busy band. A first attempt at porting this
     (`compute_baseline_spectrum`) introduced its own new ~19.6 dB
     bug: `get_spectrum_baseline.f90`'s `savg=savg+s(1:NH1,j)` frame
     loop is a raw *sum* over `NF≈93` frames despite its own "Average
     spectrum" comment — no `/NF` anywhere in the real subroutine. The
     port added the missing-looking division, which is wrong
     (`10·log10(93) ≈ 19.7 dB`, matching the measured miscalibration
     almost exactly once found).

  Both must be fixed *together* — verified experimentally that fixing
  only #2 first made things worse (xsig and xbase's absolute gains
  don't match by construction across different windows, so an
  inconsistent pairing adds a spurious offset rather than cancelling
  one out; this is also why the historical baseline-only ~3-7 dB gap
  looked "smaller" than either individual bug — the old rectangular-
  spectrum reuse put both `xsig` and `xbase` on the *same* miscalibrated
  scale, which mostly cancelled in the ratio). Post-fix, deltas against
  real `jt9`'s own probed internal `xsnr2` land within ~3 dB on a clean
  isolated synthetic signal (down from the pre-fix ~18-24 dB this
  investigation's own intermediate, reverted attempts produced when
  only one side was corrected). `qso3_busy.wav` recall entries mostly
  land within ±3.5 dB of their WSJT-X/JTDX golden (down from the
  systematic 3-7 dB low bias); `ft8_qso3_jtdx_recall.rs` /
  `..._high_sensitivity_recall.rs`'s `SNR_TOL_DB` stays at 12 dB
  (JTDX's own SNR figure doesn't reliably track vanilla WSJT-X's on
  every entry — tightening it would fit mfsk-core to JTDX's quirks,
  not WSJT-X's), with two known-unreliable JTDX-only entries
  (`WM3PEN`/`W1FC`, absent from real `jt9`'s own decode list on this
  file entirely) excluded from the SNR-drift assertion specifically
  via a documented `JTDX_SNR_GOLDEN_UNRELIABLE` list. Zero recall
  regression across every `qso3_busy.wav` suite, `ft8_sic_early`
  regression guards (issue #253's own false-decode fix), and streaming
  contract tests; full `cargo test --lib --features full` (392 passed)
  and `scripts/pre-push-check.sh`'s full feature matrix clean.

- **FT8 host `xsnr2` now shared by every entry point, not just
  `decode_block`** (issue #253 follow-up, same day) — found via a
  production consumer: WebFT8 (`decode.rs`'s `DecodeRequest`/
  `SniperRequest` engines: `.sic_rounds()`, `.sic_early()`, plain
  single-pass `.decode()`, `.sniper()`) reported a visibly different
  SNR than `decode_block()` for the same signal on the same commit,
  because those four engines never called the `xsnr2` fix above at
  all — they still used `engine::llr::compute_snr_db`'s adjacent-tone
  metric, an entirely different WSJT-X quantity (`ft8b.f90`'s `xsnr`,
  vs. `xsnr2` — WSJT-X itself overwrites `xsnr` with `xsnr2` for every
  non-AP-retry decode, `ft8b.f90:459`, so `xsnr2` is the one real
  WSJT-X actually displays in the normal case). Extracted the
  `xsnr2` computation into two shared, reusable functions
  (`compute_xsig_wsjtx`, `apply_wsjtx_xsnr2`) and wired them into all
  four `decode.rs` engines the same way `decode_block_multipass`
  already used it, so every FT8 entry point now reports the same SNR
  for the same signal (verified: `decode_block()` and all four
  `DecodeRequest`/`SniperRequest` strategies now agree within ~1 dB of
  each other on `qso3_busy.wav`, where before this fix the plain
  single-pass `.decode()` read ~15 dB lower than `decode_block()` for
  the same strong signal). Zero regression: full `cargo test --lib
  --features full` (392 passed), every `qso3_busy.wav` suite,
  `ft8_sic_early` regression guards, streaming-contract tests, and
  `scripts/pre-push-check.sh`'s full feature matrix (including
  `fft-extern`/no-`std`/`fixed-point` combos, which don't compile the
  new code at all — `compute_snr_db` stays the only SNR source there,
  unchanged) all clean.

- **FT4 SNR systematically under-reported by ~6.9 dB** (issue #255,
  first protocol tackled from the audit the FT8 `xsnr2` work above
  prompted: does every `GenericPipelineProtocol` implementor's shared
  `engine::llr::compute_snr_db` adjacent-tone heuristic actually match
  its own protocol's real WSJT-X formula, or is FT8 not the only one
  standing in for something it isn't?). Real WSJT-X's FT4 SNR
  (`ft4_decode.f90:226,452-457`) turns out not to be an adjacent-tone
  ratio at all: `snr = candidate(2,icand) - 1.0` → `xsnr =
  10·log10(snr) - 14.8` (`-21.0` floor) — computed directly from the
  coarse-sync *candidate's own score*, no separate baseline pass. A
  lucky find: `engine::ft4_coarse::ft4_coarse_sync` (a faithful port of
  WSJT-X's `getcandidates4.f90`, already shipped) already computes
  exactly that score as `SyncCandidate::score` — it just wasn't being
  read for SNR, only for candidate ranking. (Careful: `DecodeResult::
  sync_score` is a *different* WSJT-X quantity — `ft4_sync_search`'s
  own later coherent Δt-search score, `sync4d.f90`-equivalent — using
  it here would be wrong.) New `engine::pipeline::ft4_snr_db(cand_score)`
  ports the real formula and replaces `compute_snr_db` at all three
  `P::ID == Ft4` call sites in `process_candidate_basic_impl`; FST4 (the
  same generic pipeline's other implementor) is untouched, still on
  `compute_snr_db` — its own real formula is a different, submode-
  dependent one (tracked separately, issue #255). Verified two ways:
  a clean synthetic signal against a real local `jt9 -5` build's own
  probed values (`-1.77` dB vs. `-0.655` dB / displayed `-1`, down from
  `-7.52` dB), and all 6 entries of the real WSJT-X-recorded
  `WSJT-X/samples/FT4/000000_000002.wav` golden (previously "SNR
  calibration unconfirmed, reference only" — now confirmed, every
  entry within ±1.5 dB). Zero regression: full `cargo test --lib
  --features full` (392 passed), every FT4 recall/streaming/subtract
  test, FST4's own test suite (untouched code path, confirmed
  unaffected), `scripts/pre-push-check.sh`'s full feature matrix clean.

- **FT4 AP-hint decode path was still reporting the wrong SNR**
  (issue #255 follow-up to the entry above) — the FT4 SNR fix landed
  as three `if P::ID == ProtocolId::Ft4 {...} else {...}` branches
  inside `engine::pipeline`'s basic decode path, but missed a 4th call
  site in `msg::pipeline_ap::finalise_result` (the AP-assisted /
  sniper path), which kept calling the generic adjacent-tone
  `compute_snr_db` directly. Exactly the branch-duplication risk that
  fix's own commit message flagged as the motivation for a non-ad-hoc
  mechanism, playing out for real. Replaced with a
  `GenericPipelineProtocol::snr_db(SnrCtx)` trait method (default =
  today's `compute_snr_db`, cited WSJT-X-formula overrides per
  protocol) — all 4 call sites (3 in `engine/pipeline.rs` + the
  `pipeline_ap.rs` one) now dispatch through the same
  `P::snr_db(SnrCtx { cs, itone, cand_score, baseline_lin })` call, so
  there is no longer a second copy of the branch to miss. `Ft4`'s
  override wraps the existing `ft4_snr_db`; FST4 keeps the trait's
  default for now (its own real formula, submode-dependent, is a
  separate piece of issue #255 not yet ported). New pinning unit test
  `ft4::decode::tests::snr_db_dispatches_to_ft4_formula` guards the
  override wiring directly (a full-decode A/B comparison of the two
  paths turned out to be confounded by search-bandwidth-dependent
  candidate scoring, not a reliable regression signal). Also
  generalised `engine::baseline::fit_baseline` into
  `fit_baseline_with`/`BaselineParams` (zero behavior change for
  existing FT8/FT4 callers, confirmed via the existing
  `engine::baseline::tests`) — infrastructure for FST4's own baseline
  port, still to come. Zero regression: full `cargo test --lib
  --features full` (393 passed), FT4/FST4 golden-sample and
  SIC/streaming integration tests, `cargo clippy --features
  full,internal-testing --all-targets -- -D warnings` clean.

- **FST4 real SNR formula, ported and shipped (issue #255 §4).** New
  `fst4::baseline` module ports `get_candidates_fst4.f90`'s
  noise-baseline extraction (local-window simplification of its
  whole-band fit, see the module's own doc comment) and
  `fst4_decode.f90:592-621`'s `xsig`/`arg`/`xsnr` formula, with a
  `snr_calfac` literal added per sub-mode to `fst4_submode!`; wired
  into `fst4/decode.rs`'s `GenericPipelineProtocol::snr_db` override,
  replacing the generic adjacent-tone default FST4 had been reporting
  through since issue #255 was opened. Verified against a real local
  `jt9 -7 -d3` build's own probed values on both of
  `WSJT-X/samples/FST4+FST4W/210115_0058.wav`'s real decodes — lands
  within 1-2 dB (N5TM: ours -8.61 dB vs. jt9 -6.90 dB; K9KFR: ours
  16.82 dB vs. jt9 16.14 dB), guarded by a new
  `fst4_60_wsjtx_sample_snr_matches_jt9_ground_truth` regression test
  (±3 dB gate).

  Getting there took two corrections past the naive formula port, both
  found by comparing against real `jt9` intermediate values rather
  than guessing: (1) this crate's shared decode pipeline
  RMS-normalises the downsampled baseband before computing per-symbol
  spectra (needed elsewhere for `compute_llr`'s scale calibration,
  issue #18) — WSJT-X's own FST4 path never does this, so `xsig`
  needed a freshly-recomputed, un-normalised `cs` rather than reusing
  the one built for LLR/BP; (2) even with that fixed, `xsig` was still
  short by a consistent ~100-115× in power — traced to a downsample
  scale-convention mismatch (`downsample_cached`'s `1/sqrt(fft1·
  fft2)` vs. WSJT-X's own `1/fft2`) and closed with an
  analytically-derived (not fitted) `NDOWN`-in-power correction. An
  earlier pass through this same investigation mis-read (2)'s
  candidate-dependent-*looking* residual as unfixable — an artifact of
  comparing final `dB`-space numbers instead of the underlying linear
  `s4`/`xsig` quantities directly; a deeper probe comparing individual
  `s4(tone,symbol)` values caught the actual constant factor. Full
  derivation in `fst4::baseline`'s module doc comment.

- **Q65 real displayed SNR, ported and shipped (issue #255 §5).** New
  `q65::snr` module ports `q65_snr` (`q65.f90:744-793`) — the value
  WSJT-X actually displays, confirmed distinct from (and always
  overwriting) the `esnodb`-based value computed inside `q65_dec_q3`/
  `q65_dec_q012` that `fec::qra::fast_fading::esnodb_fast_fading`
  already faithfully ports and which stays intentionally unused.
  Unlike FT8/FT4/FST4's formulas (signal power at the one decoded tone
  vs. a baseline), `q65_snr` builds a composite tone-aligned spectrum
  across all 85 symbols (22 sync + 63 data) and reads a guard-band
  baseline + integrated excess power off that. Wired into the four
  single-slot decode paths that have direct `audio` access
  (`decode_at_inner`, `decode_at_fading_for`,
  `decode_at_with_ap_list_for`, `decode_at_grid_for`); the three
  multi-period-averaging (`iavg=1,2`) call sites only receive
  already-averaged energies, not a raw audio buffer this formula's own
  per-symbol FFT extraction needs, and keep reporting the existing
  adjacent-tone heuristic (documented at each call site, follow-up if
  ever revisited). Verified against a real local `jt9 -3 -d3` build's
  own displayed SNR on four real off-air recordings, one per sub-mode
  (`WSJT-X/samples/Q65/{60D_EME_10GHz,120D_Rainscatter_10_GHz,
  120E_Ionoscatter_6m,300A_Optical_Scatter}`) — landed within ~1 dB on
  the *first* implementation attempt, no scale-factor archaeology
  needed this time: unlike FST4, Q65's own energy extraction FFTs the
  raw audio directly with no shared downsample/RMS-normalisation
  pipeline in between to introduce a mismatch. New regression test
  `q65_snr_matches_jt9_ground_truth` (±3 dB gate) guards all four.

- **Q65 real SNR extended to the multi-period-averaging (`iavg=1,2`)
  decode path** (issue #255 §5 follow-up, issue #256). Read WSJT-X's
  own `q65_dec0`/`q65_symspec` (`q65.f90:30,265-304`) to check whether
  the multi-period path's real SNR reuses one representative slot or
  an average: it's a genuine average — `s1 = s1a(:,:,iseq)` when
  `iavg>=1`, accumulated via `s1a = u·s1 + (1-u)·s1a` with
  `u = 1/min(navg,4)`, the *same* EMA weight formula
  `q65::search::Spectrogram`-based multi-period coarse search already
  implements. New `q65::snr::q65_composite_spectrum_averaged`/
  `q65_snr_db_averaged` apply that identical EMA directly to the
  composite spectrum (mathematically equivalent to averaging the
  underlying per-symbol energies first, since the composite-spectrum
  sum is linear) rather than switching to `Spectrogram`'s own
  coarser grid — WSJT-X's `s1` also applies `smo121` frequency
  smoothing and a time-domain interpolation trick neither this
  crate's `Spectrogram` nor its existing single-slot extraction
  replicate, and the single-slot verification data (4 real signals
  spanning `mode_q65` 1/8/8/16, i.e. WSJT-X's own `smo121` pass count
  1/32/32/128) showed no correlation between that gap and the
  reported-dB accuracy, so porting it wasn't judged worth the risk of
  a bigger rewrite. Wired into all 3 multi-period call sites
  (`decode_averaged_ap_list_for`, `decode_fading_with_energies`,
  `decode_averaged_plain_for`). Not checked against real `jt9` output
  for this path specifically — `jt9`'s CLI batch mode has no flag
  driving the GUI-only Rx-cycle accumulation `iavg` depends on
  (confirmed empirically); verified structurally against `q65.f90`
  and for recall (existing multi-period golden tests still decode,
  reported SNR values sane: -19 to -28 dB range for signals that only
  decode via averaging at all).

### Changed

- **`wspr::SearchParams::default().max_candidates` 16 → 200**, matching
  `wsprd`'s own cap (`wsprd.c:1088`). Measured effect on both corpora:
  none — `smspec` is smoothed before peak detection, so the number of
  local maxima is under either cap on the material we have. Changed
  anyway because 16 would bind on a genuinely crowded band.

- **`fec::ConvFano::METRIC_BIAS` 1.0 → 0.45 and `DEFAULT_DELTA` 17 →
  60**, i.e. `wsprd`'s own `bias` and `delta` (`wsprd.c:822-823`). The
  previous values were fitted around the linear metric's
  input-dependent scale; the normalisation added above makes the real
  ones usable. `ConvFano` backs WSPR only — JT9 uses `ConvFano232`,
  which already carried an equivalent LUT.

### Added

- **`wspr::WsprResult::drift_hz`** — the linear drift the successful
  demodulation ran at, `wsprd`'s `drift1`. Load-bearing rather than
  informational: it is what the subtraction needs to reconstruct a
  replica that drifts the same way the signal did.

  Note for anyone *constructing* `WsprResult` with a struct literal:
  this is a new required field. Code that only reads decode results —
  which is what the type is for — is unaffected.

- **Precision guards on every FST4 and Q65 sub-mode.** Both protocols
  now execute a decode per sub-mode in CI (above), but the assertions
  were recall-only: "the expected message is somewhere in the results".
  A clean single-signal synth slot is the cheapest place to check the
  other half — one signal in, so anything else out is a phantom, with no
  interferer to blame. Added to FST4's `synth_roundtrip_for` (15/30/120/
  300) and the FST4-60 test that predates it, and to the scan-path tests
  in `q65_roundtrip`, `q65_a15_roundtrip` and `q65_eme_submodes`. FT4's
  clean-synth paths gained the same. All pass with zero phantoms, so
  these pin current behaviour rather than fix a defect — which is the
  point: FST4-60's guard had been the only one of fifteen sub-modes
  across the two protocols.

  JT65/JT9/MSK144 needed none: their in-crate roundtrips go through
  `demodulate_aligned`, a single-target demod returning one codeword
  asserted for exact content, so there is no set of extra decodes for a
  phantom to hide in.

- **`tests/common/corpus.rs::missing()`** — skips quietly by default and
  **panics** under `MFSK_REQUIRE_CORPUS=1`, so a corpus-gated test
  cannot report a green run for assertions it never reached.

- **`tests/ft8_qso3_jtdx_high_sensitivity_recall.rs`** — a JTDX "RX
  Sensitivity = High" 20-entry golden for `qso3_busy.wav` (superset of
  the existing default-sensitivity 18: adds `CQ F5RXL IN94` and `K1JT
  HA5WA 73`). Current floor 19/20 host research-config single-pass
  (only `K1JT HA5WA 73` missing); confirmed 20/20 reachable via
  `.sic_early()`/`.sic_rounds(3)` during the investigation above.
  Source: `reference_qso3_busy_jtdx_decode.md`'s 2026-08-10 addendum —
  a separate reference profile, not a correction of the existing
  default-sensitivity WSJT-X-8/JTDX-18 goldens.

## 0.9.0 — streaming ergonomics for host UIs

Theme for this cycle: make the streaming decode surface easier to
build desktop/host UIs on. Redundant-processing cleanup from recent
PRs rides along as internal hygiene (it shrinks the surface these
additive APIs sit on); it is not the headline and has no behavioural
effect on any decode path.

### Added

- **Streaming decode delivery: `.on_result(cb)` / `*_streaming` siblings**
  (issue #204; PRs #237/#239/#240) — a synchronous callback fired once
  per accepted decode result, delivered as candidates resolve instead
  of only after the whole slot finishes. Purely additive: every
  existing batch `decode()`/`decode_scan(...)` call keeps returning its
  full `Vec`/`DecodeOutcome` unchanged, callers who don't opt in see
  zero difference. Deliberately a plain `Fn(&Result) + Sync` callback,
  not async/a channel/Tokio — `engine`/`protocol` stay executor-free so
  the `no_std` embedded targets that are first-class consumers of this
  same decode path keep working; a host wanting cross-thread delivery
  (e.g. into a GUI) wraps the callback itself. Design rationale and the
  two delivery contracts (sequential exact-match vs. parallel
  completion-order-with-possible-transient-duplicate) are written up in
  the new `docs/reference/STREAMING.md`/`.ja.md`, including a worked
  Tokio `spawn_blocking` + `mpsc` example.

  - **FT8**: `DecodeRequest`/`SniperRequest::on_result`, plus a new
    `decode_block_streaming` sibling to `decode_block` — originally
    embedded-only (`#[cfg(not(feature = "fft-rustfft"))]`), given a
    host `fft-rustfft` sibling with the same signature and contract
    later in this same cycle (issue #243's follow-up, see Fixed
    below). Threaded through all four
    decode strategies (`.decode()`/sniper/`.sic_rounds()`/
    `.sic_early()`). On the two `rayon`-parallel strategies the callback
    fires inside the per-candidate closure, before the later
    cross-candidate dedup pass — documented as a possible transient
    duplicate the returned `Vec` later excludes. On the three
    sequential/SIC strategies the fire point already is final
    acceptance, so delivery is an exact match with zero divergence.
    Candidates are visited in Costas-sync-score-descending order on
    every strategy (`coarse_sync` sorts before returning), so results
    also *tend* to favor strong signals first — a correlation, not a
    guarantee (sync score isn't a direct predictor of post-demod
    BP/OSD cost, and the sequential strategies still suffer
    head-of-line blocking behind a high-scored-but-OSD-needing
    candidate).
  - **Q65**: `.on_result()` on `DecodeRequest`, `SniperRequest`, and
    `MultiPeriodRequest` (`src/q65/decode_request.rs`) — the first two
    fire per accepted candidate (exact-match, sequential, no early
    exit); `MultiPeriodRequest` fires once per *slot* that yields an
    accepted decode, the natural streaming unit for its multi-period
    EME/ionoscatter averaging. `SniperRequest` fires 0-or-1 times
    (single-candidate decode, no loop to stream across) — kept for
    builder-API consistency, documented as such.
  - **WSPR/JT9/JT65**: none of the three have a builder API (plain `pub
    fn` families), so — matching FT8's own `decode_block`/
    `decode_block_streaming` precedent — each gains a non-breaking
    `decode_scan_streaming` sibling instead of a new parameter on the
    existing function. WSPR additionally gets
    `decode_scan_subtract_streaming` (fires at the outer SIC-pass
    accept point only, not inside each pass's internal `decode_scan`
    call). JT65/JT9's loops are sequential with no parallelism, so
    their streamed delivery is an exact match against the batch `Vec`,
    same order, with no divergence mechanism.
  - **`mfsk-ffi`: `mfsk_decode_i16_streaming`** — the streaming surface
    above was never exposed across the C ABI at all (found while
    auditing the FFI layer, issue #246 follow-up). FT8 only for now
    (other protocols return `MfskStatus::UnknownProtocol`); additive
    to `mfsk_decode_i16` in every sense `DecodeRequest::on_result`
    already establishes (`out` still gets the full batch list,
    `callback` is optional). First function in this crate to run
    caller-supplied code mid-decode, so it's also the first to need
    `catch_unwind` (a Rust panic must not unwind across an `extern
    "C"` boundary) and a `Sync`-asserted `user_data` wrapper (the
    parallel/rayon strategy invokes the callback from worker threads).
    Verified via `tests/streaming_ffi.rs` and a new `cpp_smoke`
    section exercising the real generated header from compiled C++.
  - **`mfsk-ffi`: `MfskDecodeOptions` builder parity** (issue #162
    follow-up) — `MfskDecodeOptions` hadn't grown a single setter
    since its creation (issue #205), despite its own doc comment
    anticipating exactly that; `mfsk_core`'s `DecodeRequest` builder
    grew `.strictness()`/`.eq_mode()`/`.freq_hint()`/`.ap_hint()`
    (FT8)/`.sic_rounds()`/`.sic_early()` over many sessions with zero
    FFI follow-through. New setters, each mutating an existing
    `MfskDecodeOptions` handle in place (one Rust builder-chain call ↔
    one C setter call before the handle is passed to a decode
    function): `mfsk_decode_options_set_strictness`/`_eq_mode`/
    `_freq_hint` (FT8/FT4/FST4-60A), `_sic_rounds`/`_sic_early`
    (FT8, `_sic_rounds` also FT4), `_ap_hint` (FT8 only, wide-band —
    reuses `mfsk_q65_decode_with_ap`'s existing 4-string convention via
    a new shared `build_ap_hint_from_cstrs` helper). Wired into every
    relevant `mfsk_decode_i16`/`_f32` branch and
    `mfsk_decode_i16_streaming`. New two new enums, `MfskStrictness`/
    `MfskEqMode` (`mfsk-ffi-abi`, alongside the existing
    `MfskDecodeDepth`). Verified with real recall-gain proofs on
    `qso3_busy.wav` for `sic_rounds`/`sic_early` (more stations than
    default) and `ap_hint` (reuses `mfsk-core`'s own already-proven
    AP-on gain, strict-superset + a known JTDX extra) — not just
    round-trip wiring checks. `.known()` (#247) and `SniperRequest`
    exposure (#249) are deliberately still deferred (each is a real
    design question of its own, not just a missing setter);
    `.fft_cache()` reuse (#248) was closed outright (no consumer
    materialized).
  - **`mfsk-ffi`: Q65 callsign hash-table exposure** (issue #250,
    the builder-parity pass's one deferred item that did ship). New
    opaque `MfskCallsignHashTable` handle
    (`mfsk_callsign_hash_table_new`/`_insert`/`_free`, same
    handle-pair shape as `MfskDecodeOptions`) mirroring
    `q65::DecodeRequest::hash_table`'s `Arc<CallsignHashTable>` —
    resolves `<...>` Type-4 hashed-callsign placeholders in decoded
    message text. Threaded through all four `mfsk_q65_decode_*`
    functions as a new optional trailing parameter (before `out`);
    NULL keeps the pre-#250 unresolved-placeholder behaviour, a
    breaking signature change for existing callers of those four
    functions (recompile + pass NULL to keep current behaviour).
    Deliberately not folded into `MfskDecodeOptions` — Q65's function
    family doesn't use that type at all. Verified with the same
    differential-test shape as `q65::rx`'s own Rust-side hash-table
    test: a Type-4 message built directly via `mfsk_core`'s Rust API
    (`mfsk_encode_q65` only packs standard messages) decodes to a
    literal `<...>` with `hash_table = NULL` and to the resolved
    `<JA1ABC>` once the same callsign is registered via
    `mfsk_callsign_hash_table_insert`.
  - **Bug, found and fixed same day (2026-08-08): `on_result` silently
    never fired for `Ft4`/FST4.** `on_result` is a field on the shared
    `DecodeRequest`/`SniperRequest<P>` structs (issue #191's generic
    builder, so it type-checked and built for any `P: FrameDecodable` —
    `Ft8` and every `Ft4`/FST4 sub-mode), but the doc comment on
    `DecodeRequest::on_result` promised delivery unconditionally, with
    no protocol scoping (unlike e.g. `ap_hint`, which does document
    itself as "FT8 only"). Only FT8's `FrameDecodable` impl actually
    read the field; `Ft4`/FST4's `__single_pass`/`__sniper`/
    `__flat_sic` never passed `req.on_result` down into
    `engine::pipeline::decode_frame`/`decode_frame_subtract`/
    `msg::pipeline_ap::decode_sniper_ap` — calling `.on_result(cb)` on
    an `Ft4`/FST4 `DecodeRequest` compiled cleanly and silently never
    invoked `cb`. No compile error (the field genuinely is read, by
    FT8's impl, so `dead_code` never fires for a struct-level field
    across every generic instantiation), and no test caught it: the
    entire test suite's `.on_result(` call sites were `ft8_streaming_
    decode.rs` and `q65_wsjtx_samples.rs` only — nothing exercised the
    `Ft4`/FST4 combination that silently broke the contract.

    Fixed by threading `on_result: Option<&(dyn Fn(&DecodeResult) +
    Sync)>` through all three call sites (`decode_frame_impl`'s two
    `filter_map` closures — same "before dedup, possible transient
    duplicate" contract as FT8's own parallel single-pass strategy;
    `decode_frame_subtract`'s per-pass `all_results.extend(deduped)`
    point — sequential exact-match, same as FT8's `.sic_rounds()`;
    `decode_sniper_ap`'s `results.push(r)` point — sequential,
    0-or-1+ depending on the AP early-exit) and wiring `req.on_result`
    through from `Ft4`'s and every FST4 sub-mode's `FrameDecodable`/
    `SupportsSicRounds` impls. New regression coverage:
    `tests/ft4_streaming_decode.rs` (all three FT4 strategies —
    single-pass, `.sic_rounds()`, sniper) and
    `tests/fst4_streaming_decode.rs` (FST4 has no SIC path, issue
    #193, so only single-pass applies); the sniper test is also the
    first in the suite to exercise `SniperRequest::on_result` for
    *any* protocol, FT8 included. Byte-identical recall verified via
    the existing golden-WAV suites (FT4 6/6, FST4-60A 1/1) plus the
    full `scripts/pre-push-check.sh` matrix.

  - **Same-day audit turned up a second, lower-severity instance of the
    same root cause: `DecodeRequest::fft_cache` was also a silent
    `Ft4`/FST4 no-op.** Unlike `on_result` this one didn't break
    correctness — the fix's own original 0.8.0-era CHANGELOG entry
    (`### Added`, `#191`, above) already called it out in passing
    ("`fft_cache` is reused where the underlying engine's buffer shape
    permits (FT8 single-pass/flat pass 0); elsewhere it's a documented
    no-op degrade… since `core::pipeline`'s generic engine has no
    cache injection point") — but that caveat was never propagated
    into `DecodeRequest::fft_cache`'s own doc comment (the surface a
    docs.rs/IDE-hover reader actually sees), unlike `ap_hint`'s
    equivalent "`Ft8` only" note on the same struct. Gave
    `engine::pipeline::decode_frame`/`decode_frame_subtract` the cache-
    injection point they'd never had (`decode_frame_subtract` only
    trusts it for pass 0 — later passes' residual has been mutated by
    subtraction, so their cache must always be rebuilt regardless) and
    wired `req.fft_cache` through the same `Ft4`/FST4 call sites.
    `SniperRequest` has no `fft_cache` field at all, so
    `decode_sniper_ap` is unaffected. New coverage:
    `tests/ft4_fft_cache_decode.rs` / `fst4_fft_cache_decode.rs` — a
    same-audio round-trip can't distinguish "reused" from "silently
    rebuilt" (both give the same answer when the audio hasn't
    changed), so these use a differential test instead: hand the
    decoder a real-but-*wrong* `FftCache` built from silence and
    assert the golden message stops decoding, proving the supplied
    cache is actually consumed rather than quietly discarded.

- **`msg::decoded::Decoded`** — a unified, owned, human-readable decode
  row for host UIs, plus a `to_decoded(..)` conversion on every
  protocol's native result type (`engine::pipeline::DecodeResult` for
  FT8/FT4/FST4, `WsprResult`, `Q65Result`, `Jt65Result`, `Jt9Result`).
  Each native result is structurally distinct with a different
  message-text path (FT8-family `unpack77`, `Display` for WSPR/JT65/JT9,
  an already-resolved `String` for Q65); `Decoded` resolves that once at
  the conversion boundary into the columns a decode list binds to for
  *every* mode — `text` / `freq_hz` / `dt_sec` / `snr_db` / `protocol`
  (`ProtocolId`). Owned (`Clone` + `Send`), so it drops straight into a
  channel from inside a streaming `.on_result` callback without the
  hand-rolled per-mode extraction the Tokio example in
  `docs/reference/STREAMING.md` previously spelled out.

  Additive, not a replacement: the native result types stay and keep
  their mode-specific diagnostics (`sync_score`, `hard_errors`,
  `iterations`, …). `Decoded` deliberately carries only the cross-mode
  intersection — those extras don't generalise into clean shared
  columns. Conversion signatures differ where the modes genuinely do:
  FT8/FT4/FST4 is fallible (`Option<Decoded>` — an unpack failure yields
  no row) and takes the caller's `ProtocolId` + an optional
  `CallsignHashTable`; WSPR is infallible; Q65/JT65/JT9 take
  `(sample_rate, nominal_start_sample)` to derive `dt_sec` from their
  sample-index `start_sample`. Design note: `docs/notes/DECODED_ROW.md`.

- **`serde` feature** (off by default) — derives `Serialize`/
  `Deserialize` on `Decoded` and `ProtocolId`, `no_std`-clean via
  serde's `alloc` feature, so a UI can emit decode rows as JSON for
  spotting / websocket / IPC. Folded into `full`. Purely additive.

- **JT65 stochastic Chase decoder** (`jt65::chase`, issue #169) — a
  faithful port of WSJT-X's `ftrsdap` stochastic Chase decoder,
  literal hand-tuned magic numbers included (not just the algorithmic
  shape — an initial same-day pass shipped a simplified approximation,
  then was rewritten for literal fidelity on request):
  `decode_at_with_chase`/`decode_scan_chase`/`_streaming`/`_default`,
  fully additive siblings of the existing `decode_at_with_erasures`/
  `decode_scan*` family (zero changes to any existing function or
  signature). Ported: WSJT-X's `perr[8][8]` erasure-probability table
  (`ftrsdap.c`, ×1.3 scale, keyed by confidence-ratio and reliability-
  rank buckets); the real `getpp` candidate-quality metric (re-encodes
  each successful trial's codeword, walks it through the same
  interleave+Gray-encode the transmitter uses, and averages the
  *original* raw FFT-bin power at the resulting positions — needed
  retaining the full 63×64 pre-decision spectrum through the demod
  step, `rx::demodulate_aligned_with_runnerup`, ~16 KB, not an
  embedded/no_std concern since JT65 already requires `std`); the
  literal `nhard`/`nsoft`/`ntotal` soft-distance formula using a
  newly-retained runner-up-tone identity; the literal acceptance gate
  (`ntotal ≤ nd0(81) && pp2/pp1 ≤ r0(0.87)`, tracking best/second-best
  candidate quality directly across all trials rather than a per-
  message tally — WSJT-X doesn't dedup by message at all); the literal
  `nhard ≤ 41 && ntotal ≤ 71` early exit; the exact LCG/`ir`-extraction
  RNG (a POSIX-style recurrence already present test-only in
  `fec/ldpc/bp.rs`, promoted to production use here); and WSJT-X's own
  `jt9 -6` trial count (`1000`, via `decoder.f90`'s
  `nranera=6 → ntrials=10**(6/2)=1000` formula — the initial pass had
  guessed `2000` with no real derivation). Not ported: AP-hint passes
  and the `hint65` correlation fallback (out of scope, no request for
  them). Covered by two always-run (not `#[ignore]`d) false-decode-rate
  tests, each asserting exactly zero false decodes (not a tolerance)
  across 20 seeds of pure noise and 20 seeds of a signal synthesized
  well below the measured sensitivity floor — re-verified against the
  ported acceptance gate.

  Measured on the existing `tests/jt65_sweep.rs` AWGN corpus (300
  files, 20 trials/SNR) at the faithful port's literal 1000-trial
  default, before the reliability-metric fix below: 50% recall
  crossing moved from −14 dB to ≈ −18.3 dB.

  **Follow-up fix, same day, user-prompted**: a challenge over the
  still-sizeable residual gap — reasonable to suspect something was
  being overlooked — turned up a real bug. The "faithful" pass above
  had reused this
  crate's pre-existing `conf` metric (`(best−second)/best`, a top-2
  -tone-only margin) everywhere WSJT-X's real `demod64a.f90` uses a
  materially different quantity: `mrprob = best_pwr/total_pwr` summed
  over *all 64* tones at a position, not just the top two — an
  SNR-like peakiness measure that reflects the local noise floor
  across the whole spectrum, which `conf` can't see (two positions
  with an identical top-2 margin can have very different `mrprob`
  depending on how much power sits in the other 62 tones). This fed
  the wrong quantity into both the erasure-priority ordering and the
  `nsoft` soft-distance weighting. Fixed by adding a `rel` field
  (`best_pwr/total_pwr`, cheap — `total_pwr` was already computed for
  the SNR estimate) to `rx::demodulate_aligned_with_runnerup` and using
  it everywhere WSJT-X uses `rxprob`/`mrprob`; `conf` was still correct
  and unchanged for the `PERR` table's ratio bucket, since
  `rxprob2/rxprob` algebraically reduces to `second_pwr/best_pwr`
  regardless of the `psum` normalization.

  50% recall crossing after this fix: −14 dB → ≈ −19.1 dB (~5.1 dB of
  the ~7-8 dB gap), ~2-3 dB still remaining at the deepest cells.

  **Second follow-up, same day, further user pushback**: still not
  satisfied with the residual gap, the user asked for a phase-by-phase
  comparison against WSJT-X's *real* pipeline, not just `ftrsdap.c` in
  isolation. That comparison (`decode65a.f90` → `decode65b.f90` →
  `extract.f90`) found the actual dominant cause, unrelated to
  `ftrsdap`: WSJT-X applies a continuous (non-quantized) frequency+
  drift correction to the *time-domain* signal (`afc65b.f90`'s
  chi-square fit + `twkfreq65.f90`'s phase-continuous correction)
  before any FFT. This crate's JT65 demod had no equivalent at all —
  every candidate frequency got rounded to the nearest FFT bin and
  stopped there. A rectangular window pays a well-known worst-case
  ≈3.9 dB "scalloping loss" for a tone sitting exactly half a bin off
  center — and this crate's own AWGN sweep golden frequency, 1500 Hz,
  sits at *exactly* bin 557.5 (NSPS=4460 @ 12 kHz ⇒ 2.6906 Hz/bin),
  the worst possible case, on every single trial in the corpus.
  Confirmed directly: a throwaway A/B probe (identical synth signal +
  identical noise, differing only in whether the frequency landed on a
  bin center) measured 100% vs. 43-50% recall at identical SNRs — the
  true scale of "what was being missed," bigger than anything found
  inside `ftrsdap` itself.

  Fixed with two additive pieces that apply to *every* JT65 decode
  path (not just chase, since they all share `search`/`rx`):
  `search::refine_freq_hz` (3-point log-power parabolic "Jacobsen"
  interpolation of the sync-tone power around the coarse bin, reusing
  the already-built spectrogram, no extra FFTs) gives candidate
  frequencies genuine sub-bin precision; a running-phase NCO in
  `rx::demodulate_aligned_with_confidence_inner` (same accumulator
  pattern as `engine::dsp::subtract`'s NCO loops) cancels the residual
  sub-bin offset on the audio before each symbol's FFT, phase-
  continuous across the whole 126-symbol frame — mirroring WSJT-X's
  `twkfreq65` (without the drift term; this crate's search doesn't
  estimate drift, and the AWGN corpus has none). Exactly a no-op for
  any caller already passing a bin-aligned frequency.

  **Final measured result**: plain `decode_at_with_erasures`'s 50%
  crossing moved −14 dB → ≈ −21.8 dB — with **zero code changes to
  that function itself**, purely by sharing the fixed `search`/`rx`.
  `decode_at_with_chase`'s moved to ≈ −23.8 dB. Both now at or beyond
  the previously-cited WSJT-X `jt9 -6` reference floor (~100% to
  −22 dB) — treated with real caution, not declared as "beats
  WSJT-X": that reference figure's own provenance wasn't independently
  re-verified against a real `jt9` binary this session, and the
  corpus's −25 dB floor is no longer deep enough to fully characterize
  either decoder (chase already shows partial recall there). The
  honest, load-bearing conclusion: **the originally-diagnosed ~7-8 dB
  gap is gone** on this crate's AWGN corpus — issue #169 can reasonably
  be considered closed as filed. Full before/after tables and the
  complete phase-by-phase writeup: `docs/notes/BENCHMARKS.md`'s JT65
  section.

- **JT9: configurable decode depth (`Jt9Depth`)** — found while
  profiling why JT9's candidate loop dominates real-recording decode
  time (`jt9::tests::candidate_loop_stage_diag`): `ConvFano232::
  decode_soft` is ~92% of it, almost entirely non-converging
  candidates burning the full `max_cycles_per_bit` budget before
  giving up (a real signal converges in microseconds regardless of the
  budget). WSJT-X's own `jt9_decode.f90` doesn't use one flat budget
  either — it escalates `limit=5000` (its own automatic-scan default,
  `-d1`) → `10000` (`-d2`) → `30000` (`-d3`) → `100000` ("Decode
  Again"). `Jt9Depth::{Fast,Normal,Deep,Max}` exposes the same four
  tiers; new `decode_scan_with_depth`/`decode_scan_streaming_with_depth`/
  `decode_at_baseband_with_fft_depth` siblings (existing `decode_scan`/
  `decode_scan_streaming` unchanged, `Jt9Depth::default()` = `Normal`
  = 10 000, this crate's pre-existing value, so default behavior is
  identical to before).

  Checked whether the default should move to `Fast` (WSJT-X's own
  `-d1`) by running a real `jt9 -9` build (defaults to `-d1`) against
  the same 20-file-per-SNR AWGN corpus `tests/jt9_sweep.rs` uses:
  real `jt9 -d1` scores 70%/10% (14/20, 2/20) at −26/−27 dB; this
  crate at `Fast` (5 000, matching WSJT-X's cycle budget exactly) gets
  60%/5%, at `Normal` (10 000) gets 65%/10% — so `Fast` would trail
  real `jt9` by *more* at both points, not close a gap to it, and
  `Normal` already falls ~5 points short of real `jt9 -d1` at −26 dB
  even at double its cycle budget (a small, separate, not-yet-
  investigated sensitivity gap). Kept `Normal` as the default.
  `docs/notes/BENCHMARKS.md`'s JT9 section has the full numbers and
  corrects a stale "80% at −26 dB" figure for real `jt9` to a freshly
  re-verified 70%.

- **JT9: fixed the small sensitivity gap above (task #24), and it
  wasn't cycle-budget-related at all.** Root-caused via two
  `jt9::decode::gate_diag` probes on the specific AWGN files this
  crate missed but real `jt9 -d1` decoded: `jt9::search::coarse_search`'s
  frequency grid is one bin wide (~1.736 Hz, exactly the tone spacing)
  with no sub-bin refinement, so candidates routinely landed 0.3-1 Hz
  off the true frequency — inside Fano's sharp non-convergence zone
  regardless of `Jt9Depth`. The same "coarse bin center isn't close
  enough, and nothing downstream recovers it" shape as JT65's own
  scalloping-loss fix (issue #169) — fixed with the identical
  technique, ported directly: `search::refine_freq_hz`, 3-point
  log-power parabolic interpolation of the sync-tone power across the
  already-built spectrogram (no extra FFTs, no new algorithm).

  Result: this crate now **exceeds** real `jt9 -d1` at both points
  that used to trail it — −26 dB 65%→**85%** (13/20→17/20), −27 dB
  10%→**25%** (2/20→5/20). Golden WAV recall unchanged (7/7). With the
  bug gone, `Jt9Depth::Fast` and `Normal` score *identically* on the
  full sweep — the extra cycle budget `Normal` was kept for above
  turned out to have been silently compensating for this bug, not
  buying real sensitivity — so `Jt9Depth::default()` moved to `Fast`
  (same result, half the non-converging-candidate cost). Golden-WAV
  `decode_scan` wall time: ~302 ms → **~159 ms**, roughly halved again
  on top of the `realfft` win earlier this file. Full writeup:
  `docs/notes/BENCHMARKS.md`'s JT9 section.

- **Re-verified FT8/FT9's sibling sensitivity claims against real
  binaries, not just published figures — found FT4 and FST4's
  documented "gaps" don't reproduce live, and found a genuine,
  root-caused gap in JT65.** Prompted by "does JT9's fix mean
  everything is now at parity?" — checked.

  - **FT4/FST4**: the previously-documented AWGN gaps (FT4 ~0.6 dB,
    FST4 0.10-0.60 dB across sub-modes) were measured against WSJT-X's
    *published* sensitivity figures, never against a live binary on
    the same corpus. Ran real `jt9 -5`/`jt9 -7` directly against
    `tests/ft4_sweep.rs`/`tests/fst4_sweep.rs`'s own AWGN corpora:
    FT4 this crate ≈−16.89 dB vs. real `jt9 -5` ≈−16.75 dB (crate
    slightly ahead, not 0.6 dB behind); FST4-60 **exact match** at
    both tested SNR points (75%/75%, 30%/30%); FST4-120 ≈−30.71 dB vs.
    ≈−30.67 dB (real, live) — a ~0.04 dB difference, noise-level at
    20 trials/point. Both FT4's and FST4's AP paths require `mycall`
    ≥3 chars (confirmed in `lib/ft4_decode.f90`/`lib/fst4_decode.f90`),
    so the bare-CLI real-binary runs are genuinely AP-free, an
    apples-to-apples comparison. Status upgraded to "at/above parity"
    for both in `docs/notes/BENCHMARKS.md`'s Summary table.
  - **JT65**: same check, opposite result — real `jt9 -6` scores
    50% (10/20) at −25 dB vs. this crate's chase decoder's 15%
    (3/20) on the identical `jt65_sweep` corpus, a real gap issue
    #169's closure didn't catch (that comparison was never checked
    against a live binary either, until now). **Initially misdiagnosed
    as a free CQ-AP-hypothesis asymmetry** (`lib/extract.f90`
    unconditionally *populates* a CQ AP entry) — corrected on closer
    reading: that entry is only ever *consulted* when `npass>1`, which
    itself requires non-empty `mycall` (`lib/extract.f90:142-150`) —
    JT65's AP path gates on `mycall` length exactly like FT4/FST4's,
    so the bare-CLI comparison above is genuinely AP-free on both
    sides. Also checked and ruled out: WSJT-X's real trial count at
    the CLI default depth is 100 (`jt65_decode.f90:106-119`), not the
    1000 this crate's `ChaseParams` assumed — yet real `jt9` still
    wins with 10× fewer trials, so trial count isn't it either. A
    direct probe (tried every `coarse_search` candidate, both
    `decode_at_with_erasures` and `decode_at_with_chase`, on the 7
    files real `jt9` decodes but this crate misses) found the correct
    candidate (exact frequency/timing) present in every file's
    candidate list — ruling out a repeat of JT9's coarse-frequency-grid
    issue — yet decode fails at every candidate regardless: the real
    gap is downstream, in demod confidence quality or `chase.rs`'s own
    `ftrsdap` calibration, not yet isolated. Needs the same
    phase-by-phase methodology that closed issue #169 originally;
    tracked as task #26, not attempted further this session. Full
    numbers and source citations: `docs/notes/BENCHMARKS.md`'s JT65
    section.

### Changed

- **wasm32 builds now enable rustfft's `wasm_simd` feature** — unlike
  `avx`/`sse`/`neon` on their native targets (auto-detected by rustfft
  at compile time, no flag needed), `wasm_simd` is a separate opt-in
  that the plain `rustfft` dependency did not previously enable, despite
  a stale comment in `Cargo.toml` claiming otherwise. Measured via a
  fresh A/B in `bench/wasm/` under `wasm32-unknown-unknown` + Node:
  ~15-17% faster on FT8's default decode path, ~23-30% faster on
  `.sic_early()` (the FFT-heavier multipass strategy — a `node --prof`
  breakdown showed ~66% of its wall-clock in FFT/subtract). Recall
  unchanged. No caller action needed — reaches every wasm32 consumer of
  mfsk-core via Cargo feature unification. Follow-up to issue #246.
- **`engine::sync::coarse_sync` no longer heap-allocates inside its
  hottest loop.** `fill_sync2d_row!`'s per-(freq-bin, lag)-cell
  accumulators (`t_blocks`/`t0_blocks`, `sync.rs`) were a fresh
  `vec![0.0f32; num_blocks]` pair allocated on every one of the
  `n_freq × (2·d.jz+1)` cells a candidate search visits — thousands of
  tiny allocations per call, shared by every protocol routing through
  `coarse_sync` (FT8/FT4/FST4). Replaced with fixed-size stack arrays
  (`[f32; 8]`, `num_blocks` ≤ 5 across every wired protocol) reused
  across the lag loop via `.fill(0.0)`. Behavior-preserving (same
  scores, byte-identical recall on FT8 `qso3_busy`/JTDX/full-parity/
  AP-on, FT4, FST4-60A; AWGN/CCIR 50%-crossings for FT8/FT4/FST4-30
  reproduced to the documented BENCHMARKS.md figures). No measurable
  wall-clock change on this box (same-session git-worktree A/B,
  `tests/coarse_sync_alloc_timing_probe.rs`, added alongside this fix)
  — consistent with `eb859cf`'s own finding that this class of loop's
  cost is dominated by arithmetic, not allocation churn; landed as a
  real elimination of allocator-call optimization barriers and
  needless churn regardless, not for a benchmark delta.
- **`engine::dsp::subtract::apply_at_offset`'s two per-sample loops**
  (the host `subtract_tones_lpf_fft` production path, shared by FT8's
  and FT4's SIC subtract) no longer branch on `j >= 0 && j <
  audio.len()` every iteration — the valid `i` range is computed once
  (`i_lo..i_hi`) and both loops run branch-free over it, since the
  out-of-range case has nothing to do in either loop (no fallback
  value, no `audio[j]` to touch). Byte-identical recall verified on
  the same FT8/FT4 golden suites.
- **JT65's and JT9's `Spectrogram`/`AudioFft::build` no longer sort
  the entire FFT-magnitude array just to read a bottom-95%
  trimmed-mean noise floor.** Found while auditing where a
  candidate-loop `rayon` parallelism pass (added, measured as zero
  benefit across 5 protocols, and reverted — see the entry below) would
  actually have paid off: profiling showed this `sort_unstable_by`
  step was itself *larger* than the FFT loop it was meant to be a
  minor adjunct to (JT65: 13-14.5ms sort vs. 7-7.7ms FFT). Only the
  *set* of bottom-95% values is needed, not their order, so
  `select_nth_unstable_by` (O(n) average partition) replaces the full
  O(n log n) sort — the same fix Q65's `Spectrogram::build_for` and
  FT8's `xsnr2_db_simple` noise median already had. `msk144::spd`'s
  noise-floor quantile had the identical pattern (a single order
  statistic, not even a range) and got the same fix. Measured:
  `jt65::search::Spectrogram::build` 22ms→10.4ms on the AWGN sweep
  corpus (release). No output change (order-independent sum / single
  index), all tests pass unmodified.
- **`jt9::softsym::AudioFft::build`'s big per-slot FFT now uses a
  real-input transform (`realfft`, new optional dependency gated
  behind the already-host-only `jt9` feature) instead of packing real
  audio into a full complex buffer.** Found via a phase-breakdown
  diagnostic (`jt9::tests::phase_breakdown_diag`) written to chase
  down a stale "unexplained ~61% of `decode_scan` time" note from the
  parallelism audit below — that figure predated the
  `select_nth_unstable_by` fix above and never accounted for this
  *second*, separate 653,184-point FFT (distinct from
  `jt9::search::AudioFft`). Real breakdown: `coarse_search` ~36%, this
  FFT ~57%, the actual per-candidate loop only ~7% — explaining why
  candidate-loop parallelism never helped JT9 either. The old code ran
  a full `rustfft` complex-to-complex transform and threw away the
  upper (Hermitian-redundant) half; `realfft` computes exactly the
  same `NFFT1/2+1` bins directly, in ~half the work — same numerics
  (it wraps `rustfft` itself), not a new algorithm. Measured:
  `AudioFft::build` ~9.4-10.0ms→~5.1ms, `decode_scan` total
  ~17-18ms→~12.8-13ms (release, real `jt9_sweep` AWGN WAVs).
  Byte-identical recall (`jt9_wsjtx_sample_recall_vs_golden`, both
  softsym golden-grid roundtrip tests) and unchanged AWGN sweep
  crossing point confirm correctness.
- **Investigated candidate-loop `rayon` parallelism across all 5
  protocols still missing it (JT65/Q65/JT9/uvpacket/MSK144) —
  measured zero benefit everywhere, including a deeper attempt at
  Q65's per-candidate `(Δf,Δt,b90)` grid search, and reverted all of
  it.** The first pass added `par_iter()` to each protocol's top-level
  candidate-decode loop on the (untested) assumption that independent
  candidates parallelize well; real timing showed no speedup on any of
  the five (MSK144's genuine `ScanState` cross-block dependency was
  also correctly identified and handled, for no eventual benefit). A
  second, better-informed pass profiled Q65's `decode_at_grid_for`
  specifically (its `Spectrogram::build_for` was already optimized,
  unlike JT65/JT9 above) and parallelized its `ibw`/`b90` sub-sweep —
  correct (`find_map_first` preserves the exact sequential
  first-success semantics; every real off-air recording still
  decoded identically) but, measured against real WSJT-X sample WAVs,
  also showed no speedup: `GridDepth::Fast` only ever has ≤7 candidate
  cells, called repeatedly inside a sequential outer loop, so
  `rayon`'s per-region dispatch overhead ate the gain — the same
  too-small-parallel-region failure mode as the first pass, one level
  deeper. Nothing from either pass shipped; the two real wins that did
  ship are the `select_nth_unstable_by` and `realfft` entries above,
  found *while* profiling for a parallelism target rather than by
  adding threads.
- Investigated, on host x86_64 (`objdump`, address-bounded to each
  symbol to avoid mis-attributing neighbouring/inlined code — an
  earlier unbounded capture falsely suggested AVX/FMA usage that
  turned out to be `rustfft`'s own kernels bleeding into the dump):
  `engine::sync::score_costas_block` already auto-vectorizes (packed
  SSE `mulps`/`addps`/`subps`/`shufps` complex multiply-conjugate, 2
  elements/iteration) with no code change needed.
  `engine::llr::fill_bmet_for_nsym`'s max-reduction — vectorized for
  `wasm32 +simd128` in PR #213 — does **not** carry the same win to
  host as currently built: its reduction runs scalar (`maxss`/`addss`/
  `subss`), not packed, under this crate's default `target-cpu=generic`
  (SSE2-baseline, no explicit vectorization hint). Left unchanged: the
  wasm measurement found this function is only ~2.9% of total decode
  time even *with* vectorization enabled there (CHANGELOG.md's #208
  entry), so a host fix's expected payoff is similarly small relative
  to the code complexity of forcing it — noted here so it isn't
  re-investigated from scratch, not acted on.

### Fixed

- **FT8 host `decode_block_multipass`'s `xsnr2` SNR-gate baseline was
  frozen at pass 1 instead of recomputed every pass (issue #243).**
  Found investigating why the host multipass path can't safely expose
  a streaming callback (`docs/reference/STREAMING.md` already
  documents the symptom). This crate captured the noise-floor
  baseline (`sbase`/`spec`) only once, at `ipass == 0`, then reused
  that single pre-subtraction snapshot for the final `xsnr2` gate
  applied to *every* candidate from *all 3* passes, in one batch,
  after the whole multipass loop finished. WSJT-X's own
  `ft8_decode.f90` calls `sync8` (which produces the baseline)
  *inside* the pass loop, once per pass, against that pass's
  then-current (already-subtracted-by-prior-passes) audio — pass-2/3
  candidates get judged against a cleaner baseline than pass-1's, and
  each candidate's decode→gate→return happens atomically inside one
  `ft8b` call, never revisited by a later pass.

  Fixed to match: `sbase`/`spec` are now recomputed at the top of
  *every* pass, and the `xsnr2` gate is applied *per pass* (right
  after that pass's own candidate loop, using that pass's own fresh
  baseline) instead of once, globally, after all 3 passes. This also
  means a candidate is fully finalised — accepted or dropped — before
  the *next* pass's subtraction and decoding begins, narrowing (though
  not on its own fully closing) the streaming-unsafe window
  `STREAMING.md` documents.

  Verified via git-worktree A/B on the `qso3_busy.wav` JTDX 18-entry
  golden: **recall byte-identical** (18/18 JTDX hit, 1 extra, both
  before and after), but several pass-2/3 candidates' reported
  `snr_db` changed to be visibly more accurate against the JTDX gold
  reference — most notably `WA2FZW DL5AXX RR73` (previously flagged
  as this suite's one lingering possible-false-positive suspect,
  `-6.4 dB → -16.0 dB`, now close to JTDX's own `-15.0 dB`). Full
  `qso3_apoff`/`qso3_apon`/`qso3_full_parity` golden suites and the
  AWGN/CCIR sweep (50% crossing unchanged at ≈−21.6 dB) all confirm no
  recall regression. `--features fixed-point` (embedded, which doesn't
  run this gate at all) unaffected.

- **FT8 host `decode_block_multipass`'s `xsnr2` gate is now atomic
  per candidate, not per pass — the streaming-unsafe window from the
  entry above is fully closed, not just narrowed.** Follow-up to
  issue #243: applying the gate per-pass (previous entry) still left
  a batch boundary *within* a pass — a candidate decoded early in a
  pass could still, in principle, be reported and then dropped by
  that same pass's own end-of-pass gate before the pass finished. The
  gate now runs inline, immediately after each candidate's signal is
  subtracted from the working buffer and before it is accepted, so a
  result is fully final — accepted or dropped — the instant it is
  processed, matching WSJT-X's own `ft8b.f90` one-candidate-per-call
  decode→gate→return atomicity exactly rather than approximating it
  at pass granularity.

  This removes the only reason `decode_block_streaming` wasn't
  available under `fft-rustfft`: `ft8::decode_block::decode_block_streaming`
  now has a host `#[cfg(feature = "fft-rustfft")]` implementation
  alongside the existing embedded `#[cfg(not(feature = "fft-rustfft"))]`
  one, same signature (`&mut dyn FnMut(&DecodeResult)`, both variants
  are always sequential — no rayon inside `decode_block_multipass`), same
  exact-match delivery contract. `docs/reference/STREAMING.md` updated;
  new `tests/ft8_decode_block_streaming_host.rs` verifies exact
  callback/batch equality (message order and `snr_db`) on
  `qso3_busy.wav`. `qso3_apoff`/`qso3_apon`/`qso3_full_parity`/JTDX
  golden suites and the full lib test suite confirm byte-identical
  output to the per-pass fix above — this is a pure restructuring of
  *when* a result becomes final, not a change to any computed value.

- **`DecodeRequest::sic_early().on_result(cb)` combined with
  `.known(...)` could deliver a callback for a result that then never
  appeared in the returned `Vec` — a second, distinct instance of the
  revoke-less-retract hazard issue #243 closed on the `decode_block`
  engine, this time in `SupportsSicEarly::__staged_sic`
  (`ft8/decode.rs`), a completely different code path from
  `decode_block_multipass`.** Found from a real user report against
  the #243 fix above, reproduced with the exact two messages
  reported (`K1BZM DK8NE -10`, `XE2X HA2NP RR73` — both real,
  marginal-SNR decodes at `hard_errors` 20/16 on `qso3_busy.wav`).

  `__staged_sic` subtracted `.known(...)` from the audio up front
  (correct), but never threaded `known` into any of the three
  checkpoints' own message77 dedup — instead relying on a *post-hoc*
  `results.retain(|r| !known.iter().any(...))` after
  `decode_frame_subtract_staged_with_ap_inner` had already fired
  `on_result` for every checkpoint's raw candidates. When subtraction
  of a `known` signal left enough residual for a checkpoint to
  independently re-decode that same message — routine for
  marginal/high-hard-error signals — the callback fired and the
  retain then silently dropped the result before it reached the
  returned `Vec`, violating `.on_result`'s own documented "exact
  match, zero divergence" contract for this sequential strategy.

  Fixed by threading `known` (renamed `outer_known` inside the inner
  function) into every checkpoint's own `sic_inner_passes` call —
  combined with each checkpoint's own already-decoded set at
  checkpoint C — so the message77 dedup that already ran atomically
  *before* the subtract/callback point (same shape the #243 fix
  established) now also covers caller-supplied `known`, not just
  same-call duplicates. The post-hoc `retain` is removed entirely
  rather than kept as a backstop — keeping it would have re-admitted
  exactly the hazard being closed.

  New tests `ft8_streaming_sic_early_matches_batch_exactly` and
  `ft8_streaming_sic_early_with_known_matches_batch_exactly`
  (`tests/ft8_streaming_decode.rs`) — the latter directly reproduces
  the reported bug (fails on the pre-fix code, passes after).
  `qso3_apoff`/`qso3_apon`/`qso3_full_parity` golden suites, the
  `.sic_early()`-specific probes (`ft8_qso3_dl8yhr_probe`,
  `ft8_qso3_dk8ne_probe`, `ft8_qso3_staged_sic_check`,
  `ft8_qso3_subtract_fix_check`, `ft8_wsjtx_depth_ladder`,
  `ft8_qso3_sync_cv_iteration_correlation`) and the full lib suite all
  confirm no recall change on the (much more common) `known = &[]`
  path — this only changes behaviour when `.known(...)` is combined
  with `.sic_early()`, and even then only removes phantom callback
  deliveries, never removes anything from the returned `Vec`.

- **The same revoke-less-retract pattern also existed on FT4's
  `.sic_rounds()`/default single-pass strategies and FST4's default
  single-pass strategy — found by grepping for the pattern rather than
  assuming the FT8 fix above was exhaustive, per the retrospective
  above.** `Ft4`/FST4's `dedup_known` post-filtered the *returned*
  `Vec` against `.known(...)` after `req.on_result` had already been
  threaded straight into the shared `engine::pipeline::decode_frame`/
  `decode_frame_subtract` and fired there — the generic pipeline has
  no `known` parameter of its own, so nothing gated the callback
  before it fired. Fixed with a new `pipeline::known_filtered_on_result`
  helper: wraps the caller's callback so a `known`-duplicate never
  reaches it, closing the gap without threading `known` through the
  protocol-agnostic engine itself. New regression test
  `ft4_streaming_sic_rounds_with_known_matches_batch_exactly`
  (`tests/ft4_streaming_decode.rs`); FST4 has no `.sic_rounds()` (no
  `SubtractCfg` exists for it) so its only affected strategy is the
  default single-pass one, already covered by the existing superset
  contract but tightened to exact anyway for consistency. Q65/WSPR/
  JT65/JT9 checked and confirmed unaffected — none of them has a
  `known`/cross-phase-dedup concept at all. No recall change on the
  `known = &[]` path (all existing FT4/FST4 recall + streaming tests
  pass unchanged).

- **FST4 could independently re-decode the same real signal up to 9x
  via distinct sync candidates that converge on the same true `(freq,
  dt)` only after per-candidate refinement — each redundant candidate
  paid the full LLR/BP/OSD staircase before a post-decode,
  message-based dedup threw all but one away.** Found auditing
  `on_result`'s §3b "possible transient duplicate" contract (issue
  #244) — the duplicates are legitimate per that contract (nothing
  incorrect ends up in the returned `Vec`), but the *compute cost* of
  redundantly decoding the same signal several times over is real and,
  compared to WSJT-X, avoidable.

  WSJT-X's own `fst4_decode.f90:310-353` never hits this: after a
  cheap per-candidate sync-refine (`fst4_sync_search`, no BP/OSD), an
  explicit "remove duplicate candidates" pass collapses any two
  candidates whose *refined* `(freq, isbest)` land within `0.10*baud`
  Hz / ±2 samples — only survivors ever reach the expensive decode
  loop. `engine::pipeline::decode_frame_impl` (FST4's shared engine)
  had no equivalent stage: coarse candidates went straight from
  `coarse_sync`'s own (coarser, pre-refine) NMS into full decode.
  Tightening that pre-refine NMS tolerance to match WSJT-X's ratio was
  tried first and measured *worse* (5→7 redundant firings on a test
  signal) — a tighter coarse filter lets more raw candidates survive
  into the expensive path, which `fst4_sync_search`'s own wide
  coherent search then independently pulls onto the same true position
  anyway. Ported WSJT-X's actual two-stage mechanism instead: a new
  `refine_candidate_position` (downsample + RMS-normalise +
  `fst4_sync_search` only) run for every coarse candidate, followed by
  `dedup_refined_candidates` — pre-decode NMS on the refined position,
  WSJT-X's exact tolerance, greedy by refined score. Scoped to the
  non-FT4 branch only; FT4's own `ft4_coarse_sync` measured zero
  redundant firings already (both a real WSJT-X sample and a clean
  synthetic signal) and wasn't touched.

  Measured on the real WSJT-X `FST4+FST4W/210115_0058.wav` golden
  sample: `on_result` firing count for the file's 2 real signals went
  from **9 → 2** — the redundancy is fully eliminated on this file, not
  just reduced (a separate clean-synthetic test went 5→2; the residual
  pair there sits outside even WSJT-X's own tolerance window, so
  WSJT-X's algorithm wouldn't catch it either — not a gap in the port).
  New regression test `fst4_60_wsjtx_sample_on_result_fires_once_per_decode`
  asserts this exactly (not just "fewer than 9") so a future regression
  is caught immediately. Every existing FST4 recall/golden test
  (`fst4_wsjtx_samples`, `fst4_sim_roundtrip` all 5 sub-modes,
  `fst4_streaming_decode`, `fst4_fft_cache_decode`) unchanged; full lib
  suite and `pre-push-check.sh`'s full feature matrix clean. Full AWGN
  sweep (`fst4_snr_sweep`, all 5 sub-modes) 50% crossing points land
  within 0.15 dB of the documented baseline table (FST4-300 essentially
  exact, -34.78 dB measured vs -34.78 dB documented) — normal
  sampling noise at n=20 trials/point, not a shift.

  **Follow-up, same day**: a user question ("BENCHMARKS.md should be
  updated — do we have the data?") prompted an actual before/after
  wall-clock measurement, which the `on_result` firing-count metric
  above never was. First result: the fix above was a **21% wall-clock
  regression** (369ms → 447ms, single-threaded, git-worktree A/B, 3
  runs each) on the same golden file — `refine_candidate_position`'s
  result was discarded after the dedup decision, so every surviving
  candidate paid the downsample+`fst4_sync_search` refine cost *twice*
  (once in the new dedup pass, once again inside
  `process_candidate_basic_impl`, unchanged). Fixed by threading the
  already-computed `cd0`/refined position through
  `process_candidate_basic_impl`'s new `precomputed_refine` parameter
  instead of discarding it — survivors now pay that cost exactly once,
  same as before this whole change. Re-measured with proper sample
  sizes (8-10 runs each, not 3): before ≈378.95ms, after ≈379.05ms —
  statistically indistinguishable, i.e. **wall-clock neutral on this
  file**, not a regression and not a measured speedup either. The
  earlier "worse" 3-sample measurements this correction itself relied
  on turned out to be as noisy as the original 3-sample "regression"
  reading — small sample counts on a ~370-450ms task aren't reliable
  here; 8+ samples were needed to separate signal from noise in both
  directions. No BENCHMARKS.md update: the honest conclusion is
  "eliminates wasted redundant compute (verified via the 9→2
  firing-count metric), wall-clock-neutral on the one file measured,
  no user-visible speedup claim to make." Golden-WAV output
  (message/freq/dt for both signals) verified byte-identical before
  and after this correction — pure performance refactor, no decode
  logic changed.

- **Q65 and MSK144 can now resolve `<...>` hashed-callsign
  placeholders — a gap this session first mis-scoped as "5 protocols
  wide" (Q65/WSPR/JT65/JT9/MSK144) before actually checking each
  protocol's own message format.** The hashed-callsign mechanism
  (WSJT-X's 77-bit message Type 4: one non-standard callsign + one
  12-bit-hashed standard callsign) is specific to
  `msg::wsjt77::Wsjt77Message`'s packing, not a cross-protocol concept
  — so the real scope, once checked, was narrower:
  - **MSK144** unpacks via the exact same `Wsjt77Message`/
    `unpack77_with_hash` dispatch FT8/FT4/FST4 use
    (`frame_decode::decode_frame`'s own doc comment already claimed
    this parity), but `decode_slot` — the crate's only public MSK144
    driver — had no parameter to supply a table, so every session
    permanently showed `<...>` for any hashed callsign heard. Fixed
    with an additive sibling, `decode_slot_with_hash_table` (same
    "`_streaming`-sibling-not-a-breaking-parameter" precedent used
    throughout this crate) — `decode_slot` now just calls it with
    `None`.
  - **Q65** packs its 77-bit payload through the same `Wsjt77Message`
    format internally, and `msg::q65::Q65Message::unpack` already
    honored `ctx.callsign_hash_table` when given one (`msg/q65.rs:150-153`)
    — but none of `q65::rx`'s nine decode functions, nor
    `q65::decode_request`'s three builders (`DecodeRequest`/
    `SniperRequest`/`MultiPeriodRequest`), had any way to build that
    `ctx` with a real table; every call site hardcoded
    `DecodeContext::default()`. Fixed by threading `ctx: &DecodeContext`
    through the full call chain (`decode_at_inner`/`decode_at_grid_for`/
    `decode_scan_inner`/the three `decode_averaged_*`/`decode_multi_period_for`
    helpers) and adding `.hash_table(Arc<CallsignHashTable>)` to all
    three builders — built once per `decode()` call from the caller's
    `Arc` (a refcount bump, not a table clone), not per grid cell,
    since `decode_at_grid_for`'s `(Δf,Δt,b90)` sweep can reach the
    unpack call site dozens of times in one decode attempt.
  - **WSPR** was *not* a gap: its own Type-3 hashed-callsign scheme
    (a distinct 15-bit `nhash`, not `Wsjt77Message`'s 12-bit hash) is
    deliberately exposed raw on the decoded message
    (`WsprMessage::Type3 { callsign_hash: u32, .. }`, `msg/wspr.rs:50-54`,
    "exposed raw so callers with a compatible WSPR hash table can
    resolve it") — already a complete design, caller-side resolution
    by intent, not an oversight.
  - **JT65/JT9** were *not* a gap either: `msg::jt72::Jt72Codec` (the
    72-bit `packjt.f90`-derived format both share) has no hashed-
    callsign concept in its wire format at all — an older WSJT
    message layout that predates the Type-4 compression scheme FT8
    introduced, not a crate limitation.

  New coverage: a differential test per fixed protocol (a same-audio
  round-trip can't distinguish "resolved" from "nothing to resolve"
  when the golden message has no hashed callsign, so both pin a
  message built via `pack77_type4`/`unpack77_with_hash`'s own recipe
  and assert the placeholder only resolves when a table is supplied) —
  `msk144::decode::tests::decode_slot_with_hash_table_resolves_hashed_callsign`
  and `q65::rx::tests::sniper_hash_table_resolves_hashed_callsign`.
  Byte-identical recall on every existing golden-WAV suite (MSK144 2/2,
  all 7 Q65 WSJT-X-golden-WAV tests), full `scripts/pre-push-check.sh`
  matrix clean.

### Docs

- **`docs/reference/STREAMING.md`/`.ja.md` §3 gained a "revoke-less
  retract" audit section** documenting the failure mode behind the two
  `on_result` fixes above (a callback fires for a candidate that a
  *separate*, later post-processing step then silently excludes from
  the returned `Vec`, with no revise/retract event) and a line-cited
  table confirming every other `on_result`/`cb` call site in the crate
  (WSPR ×2, Q65 ×5, JT65, JT9, plus FT8/FT4/FST4's own non-`known`
  paths) commits the callback-fired value/set to the returned
  collection with no filtering step in between — checked directly, not
  inferred from the fixes. Written up so a future `_streaming` sibling
  added to a protocol that also gains a `.known(...)`-style
  cross-phase-dedup parameter has a concrete pattern to check against,
  instead of every consumer needing their own reproduction experiment
  to confirm it.

### Changed

- **The remaining WSPR/JT9/MSK144 hot-loop findings from the same
  vectorization audit that produced `engine::sync::coarse_sync`'s and
  `engine::dsp::subtract::apply_at_offset`'s fixes above.** All six are
  the same two anti-patterns applied to different files — per-element
  index-only bounds branches (hoistable, same fix shape as
  `apply_at_offset`) and heap allocation inside a hot loop (same fix
  shape as `fill_sync2d_row!`) — not new vectorization work of their
  own. Byte-identical recall and AWGN sweep crossings on every affected
  protocol's golden/sweep suite (WSPR 8/8 golden + full 13-point AWGN
  sweep, MSK144 3/3 golden + full short/long-ping sweep, JT9 7/7 golden
  + full AWGN sweep) — all reproduced their documented BENCHMARKS.md
  percentages exactly.
  - **`wspr::subtract::subtract_signal_baseband`**: its two per-sample
    loops (`camp` build, final subtract) branched on `k > 0 && k < np`
    every iteration — the identical anti-pattern
    `apply_at_offset` had before today's earlier fix, just never
    applied here. Clamped the valid `i` range once per loop instead;
    the second loop's `n > 0.0` guard (a *value*, not a pure index
    boundary — `partial[]`'s startup-transient correction) stays a
    per-iteration check, not hoisted.
  - **`msk144::sync::msk144_freq_search`**'s per-CFO-trial loop
    allocated four fresh `Vec`s (`mixed`/`c`/`ct2`/`xcc`) every trial.
    Added `tweak1_into` (writes into a caller buffer instead of
    allocating) and hoisted all four buffers outside the trial loop,
    reused via `.fill`/`.copy_from_slice`; `best_frame`/`best_xcc` now
    copy from the scratch buffers only on an actual improvement
    (rarer than every trial) instead of moving a freshly-allocated
    buffer out of the loop unconditionally.
  - **`msk144::sync::rotate_to_shift`** used a per-element `%
    NSPM` index for what's structurally two contiguous copies (the
    same file's own `ct2` doubled-buffer build, a few lines above,
    already uses the correct two-`copy_from_slice` idiom for an
    equivalent wraparound — this function just didn't reuse it).
  - **`msk144::spd::detect_burst_candidates`**'s per-offset scan
    allocated two fresh buffers (`ctmp`/`tonespec`) every offset;
    `NFFT == NSPM` makes both fixed-size for the whole scan, so
    they're now allocated once and reused.
  - **`jt9::softsym::AudioFft::build`**'s envelope loop and
    **`AudioFft::downsam9`**'s FFT-shift bin remap both had an
    index-only `j < buf.len()` / `j >= 0 && j < c1_len` branch inside
    their hot inner loop, monotonic in the loop variable — same
    "boundary only, never mid-range" shape as `apply_at_offset`'s
    original bug. `downsam9`'s remap in particular is two
    separately-monotonic contiguous pieces (an FFT-shift split, `i in
    0..=nh2` vs. `i in (nh2, NFFT2)`) rather than one, so it's clamped
    as two ranges. Both now compute the valid sub-range once and loop
    branch-free over it.

  Not touched, per the same audit's own explicit "don't" list (see the
  `### Changed` entry above this one and the audit's own findings):
  `wspr::coarse_baseband`'s `refine_alignment_top_k` (gather-bound,
  matches `wsprd.c`'s own scattered access, no restructuring
  available without diverging from the reference algorithm), JT65's
  `score_candidate`/`decode_at_with_erasures`/demodulate argmax
  (gather-bound sync-position table / inherently sequential retry
  ladder / stateful reduction — none restructurable), and
  `msk144::sync::tweak1`'s NCO recurrence (loop-carried, single
  channel, no independent lane to interleave against — would need a
  materially bigger redesign, e.g. batching multiple CFO trials'
  rotors as parallel lanes, tracked as a future idea rather than done
  here).

- **Actually measured the above, not just asserted it — findings were
  mixed, and one initial reading was itself a measurement artifact.**
  Same-session git-worktree A/B, `msk144::decode::decode_slot` on both
  WSJT-X golden WAVs: **~13-15% faster** (843→730 ms, 804→680 ms) —
  `msk144_freq_search`'s CFO loop turned out to be a genuinely hot
  path (once per candidate × navmask × dither combination), so
  removing its four-alloc-per-trial cost was a real win, not just
  hygiene. `wspr::decode::decode_scan_subtract` on its golden WAV:
  flat (369.5 ms vs. the already-recorded 369.7 ms, within noise) —
  the LPF convolution step this doesn't touch already dominates.
  `jt9::decode_scan` on its golden WAV **initially measured ~5%
  slower** (311→325 ms) — investigated by isolating `AudioFft::build`
  and `downsam9` with a dedicated timing probe
  (`jt9::softsym::tests::probe_isolate_build_vs_downsam9`, kept as a
  standing diagnostic) and found **no regression in either function**,
  then re-ran the original end-to-end comparison *properly
  interleaved* (alternating worktrees every run instead of measuring
  one side's whole block, then the other's) and it also came back
  flat. The initial reading was this box's own run-to-run noise
  (already documented elsewhere in this file as ~15%) lining up
  against block-grouped measurement order, not a real effect from the
  code change — a live example of why every dated re-measurement
  entry in this file uses interleaved/isolated methodology rather
  than sequential before/after blocks.

- **`jt9::softsym::peakdt9`** — found while writing the isolation probe
  above, not part of the original audit, and a different fix shape
  from the rest of this cluster: its sliding-window coherent-sum loop
  recomputed the whole up-to-`NSPSD`-wide window from scratch at every
  one of `NFFT2` positions (O(NFFT2·NSPSD)); replaced with a running
  sum that adds the entering sample and subtracts the one that fell
  out of the window each step (O(NFFT2)), verified equivalent via the
  window's own `lo(i) = max(0, i-(NSPSD-1))` growth/slide structure.
  Also hoisted two more instances of the same index-only-branch
  pattern already fixed elsewhere in this file: the sync-score search
  loop's `idx >= p.len()` bound (monotonic in `sym`, only the upper
  edge ever triggers, and only for large `lag`) and the `c3` extraction
  loop's `j` bound (monotonic in `i`, a single fixed per-call offset).
  Properly interleaved measurement (git-worktree A/B, alternating every
  run — the methodology the entry above this one exists to justify)
  showed a modest but consistent ~1-2% `decode_scan` speedup across 4
  rounds, smaller than hoped for a supposedly-16x-fewer-ops change,
  but real and in the right direction (unlike the `AudioFft::build`/
  `downsam9` hoists, which measured as genuinely flat). Byte-identical
  recall (JT9 7/7 golden) and AWGN sweep (exact percentage match at
  every SNR point) confirm the running-sum rewrite is equivalent, not
  just faster.

## 0.8.1 — decode-side `snr_db` for WSPR/JT65/JT9/Q65 (#226)

### Added

- **`snr_db: f32` on `WsprResult`, `Jt65Result`, `Jt9Result`, and
  `Q65Result`** (issue #226, breaking — new required struct field on
  four public types). FT8/FT4/FST4 already share `engine::pipeline::
  DecodeResult`, which carries `snr_db`; MSK144 has its own
  (`SlotDecode::snr_db`, WSJT-X `mskrtd.f90` formula). The other four
  WSJT-family decoders had **no** decode-side signal-quality field at
  all — `mfsk-ffi`'s `push_simple` (used for WSPR, and the fixed-
  alignment JT9/JT65 FFI entry points) was hardcoding `snr_db: 0.0`
  for every one of them, which is why the gap wasn't visible from the
  FFI surface. Filed after WebFT8 tried to wire up Q65 QSO-exchange
  reports ("+NN"/"-NN") and found there was no SNR to report from —
  auditing the sibling protocols for the same gap (prompted by "is a
  missing field like this really just one mode's bug?") turned up all
  three others too.

  - **WSPR**: `wspr::coarse_baseband::BasebandCandidate` already
    computed a wsprd-calibrated candidate SNR (`10·log10(smspec) −
    26.3`, `wsprd.c:1093`) during coarse search — it was being
    discarded before reaching `WsprResult`. `decode_scan`/
    `decode_scan_subtract` now thread it through; direct
    `decode_at`/`decode_at_baseband`/`decode_at_baseband_nblocks`
    calls (no coarse candidate to derive it from) report `0.0`.
  - **JT65** and **Q65**: new decode-side estimate — signal = power at
    each data symbol's decoded tone, noise = mean power of the other
    63 tones in the same per-symbol FFT bin (same "opposite-tone"
    shape as `engine::llr::compute_snr_db_generic`, FT8/FT4/FST4),
    converted to WSJT-X's 2500 Hz reference bandwidth via `10·log10
    (2500/tone_spacing_hz)` — cross-checked against FT8's literal
    `-27 dB` @ 6.25 Hz and wsprd's literal `-26.3 dB` @ ~5.1 Hz, both
    within ~1 dB of that formula, so expect similar-order accuracy
    pending real calibration. Q65's four wide-energies decode paths
    (fast-fading metric, grid search) get a layout-aware variant of
    the same estimator; Q65's info-symbols are re-encoded back to the
    63-symbol channel codeword via `Q65Codec::encode` to know which
    tone was "decoded" at each slot (AP-list paths use the winning
    candidate codeword directly, no re-encode needed).
  - **JT9**: same signal/noise decomposition, but **not** WSJT-X
    2500 Hz-referenced — `jt9::softsym`'s `downsam9`→`peakdt9`→
    `symspec2` pipeline runs the per-symbol power through AGC scaling,
    an unnormalised IFFT, and a coherent sample sum before the tone
    comparison, so the tone-spacing bandwidth offset that works for
    JT65/Q65 doesn't apply; an initial attempt that borrowed it
    produced an implausible reading (clean noiseless synth ≈ −4 dB).
    `Jt9Result::snr_db` is documented as relative-only: useful to
    compare JT9 decodes against each other, not against the other
    protocols' dB2500 values.
  - Caught in testing: the shared floor/ratio formula returned the
    **-24 dB floor** for a perfectly clean noiseless synth signal
    across all three new estimators, because an exactly-orthogonal
    synthetic tone can leave literally zero measured power in the
    non-signal bins — "no measurable noise" (the *best* case), which
    the original floor-on-zero-noise branch (mirroring `engine::llr::
    compute_snr_db_generic`'s existing behavior) reported as the
    *worst* case instead. Fixed by returning a `+49 dB` ceiling
    (matching WSJT-X's own display-clamp convention) when noise is
    zero and signal isn't, in `q65::rx::snr_db_from_sig_noi`,
    `jt65::rx`, and `jt9::softsym::symspec2_from_ss2`.
  - `mfsk-ffi`'s `push_simple` now takes `snr_db` explicitly (WSPR and
    Q65 pass the real value; the JT9/JT65 fixed-alignment entry points
    still pass `0.0` — that path uses the bare `decode_at`, which has
    no SNR estimate available, not `decode_scan`).

## 0.8.0 — JT65 decode-chain bug fix (#24) + JT9 AWGN SNR sweep + Q65-15A/120D/120E/300A + fine-timing sensitivity fix + CQ-AP-hint parity note (#171) + BASIS removal (#162, breaking FFI change) + FT8 `DecodeDepth` redesign + auto-AP removal (issue #182 follow-up, breaking) + CCIR moderate/poor sweep gap closed (#190) + `DecodeRequest`/`SniperRequest` consolidation (#191, breaking) + `core::pipeline` dead-code cleanup (#192, breaking) + pre-#191 raw decode API demotion (#203, breaking) + `core` → `engine` module rename (#206, breaking) + FT8/FT4/FST4 `DecodeResult` unification (#194, breaking) + sealed `FecCodec` (#198) + Q65 `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` builder migration (#204, breaking) + unified `mfsk-ffi`/`mfsk-ffi-ft8` C-ABI conventions via new `mfsk-ffi-abi` shared crate (#205, breaking) + WSPR/JT9/JT65/Q65 decode-result naming convention (#206, breaking) + `downsample_cached` FFT-plan caching fix (#211) + wasm `+simd128` LLR vectorization (#208) + embedded-poc `+esp` compile fix (#215) + `DecodeRequest`/`SniperRequest::depth` → `.osd(bool)` (breaking) + FT8 `WsjtxDepth`/`wsjtx_depth` jt9-comparison preset + `DecodeRequest::flat()`/`.staged()` → `.sic_rounds(n)`/`.sic_early()` (#218, breaking) + FT8 `DecodeStrictness` wiring for the non-AP decode path (#221)

### Added

- **`DecodeRequest<P: FrameDecodable>` / `SniperRequest<P>`** (issue
  #191) — a single, ZST/trait-driven builder replacing the FT8
  `decode_frame*`/`decode_frame_subtract*`/`decode_sniper*` family (15
  public functions), FT4's `decode_frame`/`_with_options`/`_with_cache`/
  `_with_cache_and_options`/`decode_frame_subtract`/`_with_options`/
  `decode_sniper_ap`/`_with_options` (8 functions), and FST4's
  `decode_frame_for`/`_with_options_for`/`_with_cache_for`/
  `_with_cache_and_options_for` plus the FST4-60A convenience wrappers (8
  functions) — 31 functions total collapsed into two generic types plus
  a handful of builder methods. Lives in `msg::decode_request`,
  re-exported from each protocol's `decode` module.

  Suffix-exploded functions encoded three orthogonal axes (`ap_hint`,
  `precomputed_fft`/`known`, SIC strategy) as combinatorial function
  names, the same disease #188 fixed for `DecodeDepth` alone. The
  reported symptom: `decode_frame_subtract_with_known_and_ap` — the
  *only* function accepting `known`/`precomputed_fft`, and the one an
  external pipelined consumer (WebFT8's `decode_phase1`/`decode_phase2`)
  actually called — ran its own unfixed flat-3-pass engine, never
  receiving the staged-checkpoint SIC (#180) or sequential-subtract
  (#178/#179) fixes the "regular" subtract path got. Structurally
  guaranteed to recur with more suffixes.

  New design, capability-gated via marker traits so invalid
  protocol/feature combinations are compile errors, not silent no-ops or
  runtime panics:
  - `FrameDecodable: Protocol` — `type DecodeResult` + hidden dispatch;
    implemented for `Ft8`, `Ft4`, every FST4 sub-mode. Deliberately not
    implemented for Q65/WSPR/JT65/JT9/uvpacket, which keep their
    existing bespoke entry points.
  - `SupportsFlatSic` (`Ft8`, `Ft4`) gates `.flat()`; `SupportsStagedSic`
    (`Ft8` only) gates `.staged()`; `SupportsWideBandAp` (`Ft8` only)
    gates `DecodeRequest::ap_hint` (FT4/FST4's AP engine has an
    early-exit-after-first-hit optimization only valid for
    `SniperRequest`'s narrow-band single-target search — enabling
    wide-band AP for them would be new, unvalidated capability, kept
    out of scope). `SniperRequest::ap_hint` is gated by the existing
    `P::Msg: WsjtApCompatible` sealed trait instead, and is available
    for all three protocols.
  - `.known()`/`.fft_cache()` are universal (any `FrameDecodable`
    protocol): `known` is always honoured as a dedup filter, and as an
    upfront `subtract_signal_lpf_refine_dt` subtraction for FT8's
    `.flat()`/`.staged()` (the actual #191 fix — see below).
    `fft_cache` is reused where the underlying engine's buffer shape
    permits (FT8 single-pass/flat pass 0); elsewhere it's a documented
    no-op degrade (always-correct recompute, not a silent behavior
    change) since `core::pipeline`'s generic engine has no cache
    injection point.

  The actual bug fix, not just the API reshape: FT8's staged-checkpoint
  engine (`SupportsStagedSic::__staged_sic`) now subtracts `known` from
  the full audio buffer *before* checkpoint A runs (using
  `subtract_signal_lpf_refine_dt` — plain `subtract_signal_lpf`
  measurably lost `CQ DX DL8YHR JO41`, ~35 Hz from a `known` W1FC
  signal, in end-to-end testing; the dt-refined ±90-sample
  best-alignment search checkpoint B/C already used for their own
  carried-forward decodes turned out to matter for `known` too), so all
  three checkpoints see a residual with caller-supplied signals already
  removed. New regression test
  `ft8_qso3_staged_sic_check::staged_with_known_and_cache_finds_dl8yhr`
  reproduces the exact two-phase pipelined-caller shape end to end.

  Type unification required as a prerequisite for a genuinely generic
  builder (previously duplicated, non-interchangeable definitions):
  `ApHint` (canonical: `msg::ap::ApHint`; FT8's own copy was already
  byte-for-byte identical, reusing the same `pack28`/`pack_grid4`),
  `FftCache` (canonical: `core::pipeline::FftCache`, same underlying
  `Vec<Complex<f32>>`), `DecodeStrictness` (canonical:
  `core::pipeline::DecodeStrictness`; `ap_max_errors` — previously
  duplicated in `msg::pipeline_ap` too — moved onto the type, no numeric
  change), `DecodeDepth` (canonical: the #188-redesigned
  `{llr_effort, osd}` struct, replacing `core::pipeline`'s stale
  `BpAll`/`BpAllOsd` 2-variant enum FT4/FST4 were still on).

  Deleted outright (confirmed zero callers anywhere in the crate,
  including internal): `decode_frame_subtract_with_known`(`_and_ap`)
  (the buggy engine above), `decode_frame_with_cache` (FT8/FT4/FST4, all
  three), `decode_sniper_sic`. No deprecation shims — matches #188's
  precedent of a hard breaking rename with all callers migrated in the
  same change.

  Engine unification (porting FT8's `fine_refine_3stage`/nsync-gate/
  sync_cv into the generic `core::pipeline` engine FT4/FST4 share, so
  FT8 could stop having its own bespoke decode engine) was investigated
  and found to be organic drift rather than a necessary architectural
  boundary (git archaeology: both engines already existed side-by-side
  at the initial fork from `jl1nie/webft8`; FT8 pulled ahead via two
  unported investment commits) — but explicitly **not** bundled into
  this change; the trait boundary here doesn't block it (`decode()`'s
  internal engine dispatch is a private implementation detail), so it's
  tracked separately as issue #192. FST4 SIC support (issue #193, no
  `SubtractCfg` exists yet — new numerical work, not a refactor) and
  full `DecodeResult` unification (issue #194, FT8's `message77` strips
  CRC bits FT4/FST4's `info` retains — a real semantic difference, not
  just a naming one) are likewise deferred as separate issues.

- **`ft8::decode::WsjtxDepth` / `DecodeRequest::<Ft8>::wsjtx_depth`**
  — three named tiers (`D1`/`D2`/`D3`) mirroring real WSJT-X's `jt9 -d
  1/2/3` CLI flag, for apples-to-apples benchmarking against a real
  `jt9` build. `DecodeDepth` alone (LLR effort + OSD) has no SIC/AP
  dimension — jt9's real `ndepth` axis (`ft8_decode.f90:168-192`,
  `ft8b.f90:403-412`) simultaneously varies SIC pass count, `syncmin`,
  subtract dt-refine, and OSD strength — so this bundles `.osd(...)` +
  `.flat()`/`.staged()` + `.ap_hint()` into one preset per tier. Local
  `jt9 -8 -d1/-d2/-d3` measurements on `qso3_busy.wav` (this session):
  D1 14 decodes/370ms vs. mfsk-core 14/237ms; D2 19/1040ms vs.
  22/1078ms; D3 22/2110ms vs. 22/2991ms — see the type's own doc
  comment for the exact tier→builder-method mapping and known
  limitations (pass-count and OSD-strength don't exactly match jt9's;
  mfsk-core has no equivalent of jt9's lighter `maxosd=0` OSD branch at
  all, a separate follow-up). New durable regression test
  `tests/ft8_wsjtx_depth_ladder.rs` replaces the ad-hoc probes used to
  derive this.

- **`ft8::decode::decode_frame_subtract_staged` / `_with_ap`** (issue
  #180) — WSJT-X-faithful checkpoint SIC for FT8, ported from
  instrumented `jt9.f90`/`ft8_decode.f90` ground truth rather than the
  issue's own initial paraphrase. `jt9`'s disk-file FT8 decode is not a
  single pass over the 15 s slot: it decodes progressively larger audio
  *prefixes* at three checkpoints (0..141_696 / 0..162_432 / 0..172_800
  samples), carrying decoded signals forward and subtracting them from
  the residual *before* the final, hardest checkpoint's candidates are
  attempted — something the pre-existing flat 3-pass
  `decode_frame_subtract` structurally cannot reproduce regardless of
  threshold tuning, since nothing is ever subtracted before the
  residual a marginal candidate is found in gets assembled. Verified
  no regression vs the flat pass on all 3 real reference WAVs
  (`tests/ft8_qso3_staged_sic_check.rs`, `191111_110130.wav`,
  `191111_110200.wav`) — identical decode sets on each. The specific
  case that motivated #180 (`CQ DX DL8YHR JO41`, ~-17 dB on
  `qso3_busy.wav`) still does not decode even with the correct
  checkpoint architecture in place (checkpoint A finds 11 signals
  early, including its closest neighbour `W1FC` only ~35 Hz away,
  correctly subtracted before checkpoint C runs).

  **Root-caused, not just reproduced.** Re-ran real `jt9 -8 -d3`
  (rebuilt from the local `WSJT-X` checkout, `build_jt9/jt9`, on the
  byte-identical `samples/FT8/210703_133430.wav`) to get fresh ground
  truth instead of relying on the issue's prior transcript: at
  DL8YHR's true coordinates jt9's own per-Costas-block breakdown is
  `is1=2 is2=7 is3=6` → `nsync=15`. A new diagnostic
  (`ft8::decode::tests::issue_180_dl8yhr_staged_checkpoint_c_probe`,
  using `decode_frame_subtract_staged_with_ap_debug_residual` to
  reproduce the same per-block count against mfsk-core's *actual*
  production checkpoint-C residual) found:
  - **Raw, unsubtracted audio**: `is1=0 is2=3 is3=3` → `nsync=6`.
  - **Staged checkpoint-C residual** (all 11 early signals cleanly
    subtracted): `is1=1 is2=4 is3=4` → `nsync=9`.
  - A ±15 ms (±3-sample) `dt` sweep around the ground truth only ever
    moved the total between 8 and 10 — the off-by-one dt-convention
    hypothesis (#180 "Bug 1") explains at most ~1 of the 6-point
    shortfall, not the bulk of it.

  Two conclusions from the aggregate counts: (1) subtraction/SIC
  quality **is** doing real work (+3 points, raw→residual) — the
  staged architecture isn't wasted effort; (2) the *remaining* 6-point
  gap barely moves under a ±15 ms timing sweep (nsync stayed in
  8..10), ruling out the off-by-one dt-convention hypothesis (#180
  "Bug 1") as the dominant cause.

  **Symbol-level dump pinpoints the actual mechanism.** Re-instrumented
  `ft8b.f90` to also emit its raw 8-tone `s8` magnitudes for Costas
  blocks 2 and 3 (only block 1 was previously dumped), rebuilt `jt9`,
  and diffed symbol-by-symbol against mfsk-core's own tone magnitudes
  at the identical `(f1, dt)`, on both raw and staged-residual audio.
  Finding: at symbol positions **t=5 and t=6 of every 7-symbol Costas
  block**, mfsk-core computes a large spurious energy spike at **tone
  0** (DL8YHR's own base carrier frequency, 2606.25 Hz) that swamps
  the true, weak Costas tone — e.g. block 2 t=5: mfsk-core residual
  tone0=9.7 vs true tone6=0.2 (raw audio: tone0=42.0 vs tone6=2.7).
  Critically, **jt9's own real-signal data shows this same tone-0
  artifact only in block 1** (where it explains jt9's own weak
  `is1=2`) — its block-2/3 t=5,6 values have the *true* Costas tone
  clearly dominant (e.g. block 2 t=5: tone6=2641 vs tone0=1140; t=6:
  tone5=3570 vs tone0=1584). mfsk-core reproduces the artifact
  correctly where it's real (block 1) but *also* manufactures it where
  the real signal has none (blocks 2 and 3) — a structural,
  position-periodic pattern (always t=5,6 of a block, at three
  absolute times spread across the ~13 s message), not diffuse
  numeric imprecision.

  **Raw baseband (`cd0`) comparison localises the mechanism further —
  and revises the diagnosis.** Re-instrumented `ft8b.f90` a second
  time to dump its raw complex `cd0` (200 sps downsampled baseband)
  samples over the exact 64-sample window spanning block-1's t=5/t=6
  (Fortran index 268..331, i.e. `ibest+128 .. ibest+191`), rebuilt
  `jt9`, and diffed those against mfsk-core's own `cd0` at the
  identical physical samples (0-indexed `n` == Fortran's 1-indexed
  `n+1` — confirmed aligned, not an indexing bug: values track the
  same sign and shape throughout, ruling out a real off-by-one at the
  `cd0` level). The magnitudes are not just noisier — mfsk-core's
  `|cd0|` runs **2-4× (6-12 dB) larger than jt9's own post-subtraction
  `|cd0|` at the same samples**, most pronounced exactly over Fortran
  idx 280-327 (e.g. idx 285: jt9 `|cd0|`=205 vs mfsk-core 632; idx
  291: jt9 121 vs mfsk-core 597). That's not a tone-detection/FFT
  computation bug — it means **mfsk-core's residual genuinely has
  more uncancelled signal energy left in it than jt9's residual does**,
  at the same point in time and frequency, even though both have
  subtracted an overlapping set of interferers by this point.

  **Isolated (a) [subtraction quality] from (b) [a missing
  interferer], and ruled out (b).** Re-instrumented `ft8_decode.f90` a
  third time to dump `f1_save`/`xdt_save`/`allmessages(1:ndec_early)`
  — jt9's *actual* list of the 13 signals subtracted before the
  nzhsym=50 pass — rather than inferring it from stdout decode-print
  order. jt9's 13 match mfsk-core's 18-message final set on 12 calls,
  but include a 13th, `WA2FZW DL5AXX RR73` @ 2545.88 Hz (only 60 Hz
  from DL8YHR's own 2606.25 Hz carrier) that **mfsk-core never decodes
  anywhere** in this investigation (not in either checkpoint's results,
  not in the flat-pass baseline) — a real, plausible candidate for
  hypothesis (b): an unsubtracted nearby signal leaking into DL8YHR's
  band. Tested directly: manually built a `DecodeResult` from jt9's own
  reported `WA2FZW` coordinates and subtracted it from the staged
  residual with the same `subtract_signal_lpf` call production uses.
  Result: **no change** — `is1=1 is2=4 is3=4, nsync=9`, identical to
  before the subtract. Hypothesis (b) is ruled out for this specific
  signal (its own contribution isn't what's showing up as excess `cd0`
  energy near DL8YHR — either the reconstructed/refined waveform
  doesn't fit what's actually there, or the energy comes from
  elsewhere entirely). That left hypothesis (a) — `subtract_tones_lpf`
  vs `subtractft8.f90`'s actual per-signal cancellation depth on the
  *already-subtracted* candidates — as the standing lead.

  **Confirmed definitively: hypothesis (a), 100% subtraction quality,
  zero mfsk-core tone-detection bug.** The decisive test: swap the
  residual, keep mfsk-core's algorithm untouched. Re-instrumented
  `ft8_decode.f90` a fourth time to dump jt9's actual `dd` array —
  raw i16, taken right after jt9's real SIC (its 13 subtracted
  signals) and right before its own `nzhsym=50` `sync8`/`ft8b` search
  — to `/tmp/jt9_post_sic_dd.raw` (180,000 samples, sanity-checked:
  RMS 115, ±577 range, consistent with a heavily-subtracted residual).
  Loaded that exact buffer into mfsk-core and ran its *unmodified*
  `symbol_spectra_direct`/`sync_quality` at DL8YHR's coordinates:
  **`is1=2 is2=7 is3=6` → `nsync=15`** — bit-for-bit the same
  per-block breakdown jt9 itself reports on this buffer. Then ran the
  full BP/OSD chain (`compute_llr` → `bp_decode`/`osd_decode_*`) on
  the same residual: **decodes `"CQ DX DL8YHR JO41"` outright.**
  mfsk-core's downsample → per-symbol tone extraction → LLR → BP/OSD
  pipeline has no bug at all with respect to this signal — given the
  same clean residual WSJT-X itself decodes from, it decodes it too,
  byte-for-byte matching sync counts. The *entire* gap between
  mfsk-core's own decode (`nsync=9`, no decode) and jt9's (`nsync=15`,
  decodes) traces to one place: **mfsk-core's own SIC subtraction
  leaves more residual interference behind than `subtractft8.f90`
  does**, on the exact same set of already-decoded interfering
  signals. The next concrete, scoped step for #180 is a direct
  `subtract_tones_lpf` vs `subtractft8.f90` per-signal suppression-dB
  comparison (GFSK waveform reconstruction fidelity, LPF width/shape,
  QSB/fading-envelope tracking) — not sync-quality computation, not
  pipeline architecture, not a missing candidate. All diagnostics
  (per-block breakdown, `cd0` dump, WA2FZW confirmation, residual-swap
  confirmation) kept in
  `ft8::decode::tests::issue_180_dl8yhr_staged_checkpoint_c_probe`.
  As of the follow-up entry below, `decode_frame_subtract_with_ap`
  (and therefore `decode_frame_subtract`) delegates to this staged
  checkpoint SIC by default; the pre-#180 flat 3-pass behaviour is
  still available via `decode_frame_subtract_flat_with_ap` for callers
  that specifically want it.
  **Cost**: checkpoint A runs its own full 3-sub-pass search on top of
  checkpoint C's, so this is strictly more decode work than the flat
  pass. Measured (release, `--features full`, 10 reps, host, real
  reference WAVs): 1.31–1.87× the flat pass's wall-clock
  (`191111_110130.wav` 231ms→303ms, `191111_110200.wav`
  249ms→460ms, `qso3_busy.wav` 388ms→611ms).

- **`core::dsp::subtract::subtract_tones_lpf_refine_dt` / FT8's
  `subtract_signal_lpf_refine_dt`** (issue #180 follow-up) — closes
  part of the "next concrete step" left open above: a direct
  `subtract_tones_lpf` vs `subtractft8.f90` comparison found the two
  algorithms structurally identical (GFSK waveform synthesis, cos²
  LPF kernel, end-correction — all line-for-line matches), *except*
  for one missing feature: `subtractft8.f90` takes an `lrefinedt`
  argument, and `ft8_decode.f90`'s two early-decode-and-subtract
  checkpoints (lines 132, 162 — exactly the calls
  `decode_frame_subtract_staged` above is modelling) pass
  `lrefinedt=.true.`. That path (`subtractft8.f90`'s internal `sqf`/
  `peakup`) re-searches `dt` by ±90 samples (±7.5 ms @ 12 kHz) around
  the candidate's own value, scoring each trial by the residual power
  left in the signal's own tone band after subtraction, and commits
  the subtraction at the parabolic-fit minimum — rather than trusting
  the early candidate's own (still-coarse) `dt` outright. mfsk-core's
  staged-checkpoint SIC had no equivalent: every early subtract used
  the plain, non-refining `subtract_signal_lpf`. Ported `sqf`/`peakup`
  faithfully (`core::dsp::subtract::fft_lpf::subtract_tones_lpf_refine_dt_fft`)
  and wired it into `decode_frame_subtract_staged_with_ap_inner`'s
  checkpoint-B and checkpoint-C deferred subtracts (the flat 3-pass
  `sic_inner_passes`'s single confirmed-decode subtract is unchanged —
  matches `ft8b.f90:474`'s `lrefinedt=.false.`).
  **Effect on the DL8YHR case**: checkpoint-C's residual `sync_quality`
  at WSJT-X's own ground-truth coordinates improved from `nsync=9` to
  `nsync=10` (jt9 itself gets `nsync=15` on this signal) — a real,
  measured reduction in leftover interference from the same 13
  already-subtracted signals, though not yet enough on its own to
  flip this specific -17 dB decode (`any_hit` still `false` in
  `issue_180_dl8yhr_staged_checkpoint_c_probe`). No regression on any
  of the 18 golden `qso3_busy.wav` decodes or any other reference WAV
  (`ft8_qso3_staged_sic_check.rs`, `ft8_qso3_subtract_fix_check.rs`,
  `ft8_qso3_apoff_recall.rs`, `ft8_qso3_apon_recall.rs`, full
  workspace `cargo test --release --features full`: 100% pass). New
  regression test:
  `core::dsp::subtract::refine_dt_tests::refine_dt_beats_plain_subtract_when_dt_is_off`.
  Remaining gap (candidates per the prior comment: LPF width/shape,
  QSB/fading-envelope tracking, or f32 vs Fortran-single-precision
  accumulation differences over the 151_680-sample reference) is still
  open — logged here rather than pursued further in this pass.

- **`core::dsp::subtract::fft_lpf::normalized_kernel` LPF window shape
  bug — root cause of the DL8YHR gap, issue #180 closed.** Auditing
  the "LPF width/shape" item left open above (per the entry directly
  above this one) against `subtractft8.f90`/`subtractft4.f90` found the
  actual bug: `window(j) = cos(pi*j/NFILT)**2` in WSJT-X (`NFILT =
  2*lpf_half`) had been ported as `cos((j - lpf_half) * pi / lpf_half)²`
  — dividing by `lpf_half` instead of `NFILT` (`2*lpf_half`), doubling
  the cosine's argument range from `[-pi/2, pi/2]` to `[-pi, pi]`.
  `cos²` is monotonic (a proper single taper, 1 at the centre → 0 at
  the edges) only over the first range; over the doubled range it dips
  to 0 at the quarter points and rises back to **1 — full weight — at
  the true edges** (`lpf_half` samples / 166 ms away from the centre
  sample, for FT8's `lpf_half=2000`). Verified numerically: at
  offset=2000, shipped kernel=1.000 vs correct=0.000; at offset=1000,
  shipped=0.000 vs correct=0.500. So every `subtract_tones_lpf` /
  `subtract_signal_lpf` call since this became the canonical FT8/FT4
  SIC entry point (v0.6.2) had been running a badly-misshapen "lowpass"
  that gave 166-ms-stale channel samples as much weight as the current
  one — actively corrupting the complex-amplitude/QSB estimate instead
  of smoothing it. Root cause of the residual DL8YHR gap the two
  entries above this one were chasing (`refine_signal_freq`'s dt-search
  only got `nsync` from 9 to 10 on top of this broken window).
  One-line-formula fix, shared by both the FFT-cached path
  (`fft_lpf::normalized_kernel`, feeds both `cached_window_fft` and
  `end_correction`) and the `no_std` direct-convolution fallback
  (`subtract_tones_lpf_direct`) — fixing both fixes FT4 subtract too
  (`subtractft4.f90` uses the identical window formula, confirmed by
  inspection; FT4's own recall floor is unaffected — see regression
  results below). **Result: `CQ DX DL8YHR JO41` now decodes**
  (`decode_frame_subtract_staged`'s `qso3_busy.wav` golden-set test,
  `staged_sic_matches_flat_pass_golden_floor`: still 18/18 golden hits,
  0 phantoms in that set, unique-decode count 19→20, `has_dl8yhr` flips
  `false`→`true`) — issue #180 is closed. Full workspace `cargo test
  --release --features full` (all 70 test binaries): 100% pass, no regression on
  any golden decode set (FT8 or FT4).

- **`decode_frame_subtract_with_ap` now delegates to the staged
  checkpoint SIC by default** (issue #180 follow-up) — the flat 3-pass
  behaviour is a strict recall subset (verified: identical golden-set
  hits on every reference WAV, plus `CQ DX DL8YHR JO41` on
  `qso3_busy.wav`, which the flat loop structurally cannot reach) at
  1.3-1.9× the flat pass's wall-clock, still comfortably inside FT8's
  15 s slot budget on host. The old flat behaviour remains available
  as its own entry point, `decode_frame_subtract_flat_with_ap`, for
  callers that specifically want the cheaper non-staged path.
  **Fixed a stack-overflow regression this introduced**: the staged
  path's own two fallback branches (buffer shorter than checkpoint A;
  checkpoint A finds nothing) used to fall back to
  `decode_frame_subtract_with_ap` — which, after this change, calls
  right back into the staged path, recursing indefinitely. Both now
  fall back to `decode_frame_subtract_flat_with_ap` instead. Caught by
  the full-workspace regression run
  (`decode_frame_subtract_with_ap_silence_shape`, a short-buffer test)
  before release.

- **`decode_frame_subtract_with_auto_ap`** (`fft-rustfft` only) —
  `decode_frame_subtract_with_ap` plus a final rescue pass: harvest
  caller callsigns from the blind decodes and retry `coarse_sync`
  candidates that didn't decode blind, using each harvested callsign
  as an AP `mycall` hint. Recovers signals too weak for blind BP/OSD
  but not too weak to sync — e.g. `K1BZM DK8NE -10` (-19 dB) on
  `qso3_busy.wav`. **This is a genuine mfsk-core-original extension,
  not a port of any real jt9/WSJT-X mechanism** — checked directly
  against `ft8apset.f90`'s actual source: real WSJT-X only applies AP
  using the *operator-configured* `mycall`/`hiscall` and disables AP
  entirely the moment `mycall` isn't set (`if(len(trim(mycall12)).lt.3)
  return`, the subroutine's first line). A real `jt9 -8 -d 3` run with
  no `-c`/`-x` has AP fully disabled; its own `K1BZM DK8NE -10` decode
  is blind (`iaptype=0`) — a real, separate, still-open blind-decode
  sensitivity gap this function does not explain or close, it just
  reaches the same message through a different, AP-assisted route.
  Cost-optimized from a naive per-(candidate, callsign) sweep (~5.7s
  on `qso3_busy.wav`) via three measured steps: (1) a presync
  `sync_quality` gate before the callsign loop (66→8 candidates), (2)
  a blind-decode precheck sharing the same refined-candidate cache,
  (3) `DecodeDepth::BpAll` instead of `BpAllOsd` for the AP retries
  (OSD is redundant once AP narrows the search) — cut to ~1.25-1.3s
  single-threaded, ~0.9s on 24 cores via `rayon` (the per-callsign
  loop is embarrassingly parallel, no shared mutable state). The
  parallel speedup is real, independent task-level parallelism — not a
  substitute for closing the blind-decode gap above, and single-
  threaded this function alone still costs more wall-clock than real
  `jt9 -8 -d 3`'s entire file decode (~1.1s), since it does provably
  more search than jt9 needs for this signal (nothing — its own blind
  pass already finds it).

- **OSD fallback gate loosened from `q >= 12` to `q > 6`**
  (`decode_block/osd_strategy.rs`, issue #180 follow-up) — the old
  gate was a mfsk-core-specific deviation with no `ft8b.f90`
  counterpart, which only bails (`nsync <= 6`) before attempting any
  decode at all. Root-caused directly: `K1BZM DK8NE -10` sits at
  `q=11` on mfsk-core's own SIC residual — verified an *exact*
  per-block match to real jt9's own residual at the same coordinates
  (`is1=1 is2=7 is3=3`, confirmed via a locally-instrumented jt9
  rebuild) — yet never reached the OSD fallback at all under the old
  gate. Full regression suite showed zero change from loosening it —
  nothing was relying on the tighter gate to stay green.
  **Not sufficient on its own**: even past this gate, the real OSD
  dispatch for `q=11` (`osd_decode_npre1`, ndeep=2) still fails to
  decode this candidate on any LLR variant, where WSJT-X's real
  `osd174_91.f90` ndeep=2 succeeds (`hard_errors=18`). Verified this
  isn't a SIC/LLR-input problem: mfsk-core's own residual matches
  jt9's own residual not just on sync quality but on all 58
  data-symbol tone decisions (0/58 argmax disagreements) and on the
  actual LLR reliability ordering OSD consumes (top-91-most-reliable
  overlap 90-91/91 across all 4 LLR variants, zero hard-decision
  disagreements within the shared basis). Filed as a genuine OSD
  algorithm fidelity gap — issue #182, open.

- **FT8 OSD now seeds with BP-refined LLR, closing issue #182.**
  Root-caused the `osd_decode_npre1` fidelity gap above: WSJT-X's real
  `decode174_91.f90` driver never feeds `osd174_91` the raw channel LLR
  when `maxosd>0` — and FT8's blind `ndepth=3` dispatch always sets
  `maxosd=2` (`ft8b.f90:434-441`). It feeds `zsave(:,i)`, the running
  sum of the BP variable-node soft estimate `zn` across the first `i`
  BP iterations, trying `i=1` then `i=2` (`decode174_91.f90:52-64,
  137-148`). mfsk-core's OSD had only ever used the raw channel LLR
  (the 4 `a/b/c/d` variants) — confirmed exhaustively: `K1BZM DK8NE
  -10` never decodes via any channel-LLR variant at any OSD depth up
  to and including brute-force order-2 exhaustive search (no
  `ntheta` gate at all), so the true codeword simply isn't reachable
  from a channel-LLR-selected basis. The mechanism needed
  (`bp_llr_zsum`) already existed and is wired for FST4-120
  (`Ldpc240_101`, issue #146) but was never ported to FT8's
  `osd_strategy.rs` dispatch. Wired it in as a second-stage fallback
  (after the existing 4 channel-LLR attempts fail): feeding
  `bp_llr_zsum(llrd, 2)` into the *same*, unmodified `osd_decode_npre1`
  decodes DK8NE outright at `hard_errors=17` — better than jt9's own
  real result (`hard_errors=18`) on this exact candidate. (A parallel
  hypothesis — that WSJT-X's `osd174_91.f90:86-107` bounded-window
  `k+20` pivot search selects a different MRB basis than mfsk-core's
  unbounded one — was tested directly and disproven: the bases do
  differ, but `osd_decode_npre1` fails on *either* basis. Kept as a
  documented, `#[cfg(test)]`-only diagnostic so it isn't
  re-investigated from scratch.)
  **Effect**: `K1BZM DK8NE -10` now decodes through the real
  production dispatch (`decode_frame_subtract_with_ap`'s AP-on
  multipass path, `qso3_apon_subtract_jtdx_extras_diag`: 5/6 → 6/6
  JTDX extras). **Cost**: ~30-40% wall-clock increase on candidates
  that reach the OSD fallback at all (most candidates decode earlier
  in the BP/OSD staircase and never reach this code path) —
  `ft8_qso3_staged_sic_check.rs` ~1.0s → ~1.4s. **One new
  near-ceiling phantom observed** (`qso3_busy.wav`, freq 2570.25 Hz,
  `hard_errors=30`, malformed callsign token — ~1 Hz from the real,
  strong `W1FC F5BZB` at 2571.38 Hz, almost certainly spectral
  leakage): same class of tradeoff already accepted in this file's
  `OSD_HARDERRORS_MAX` 22→36 history, where tightening the ceiling to
  suppress phantoms was found to silently discard genuine weak
  decodes living in the same hard-error range. Full workspace `cargo
  test --release --features full`: 100% pass, no regressions.

- **FT8 OSD dispatch: dropped the direct-channel-LLR loop, since it
  was never what real WSJT-X does either** (issue #182 follow-up).
  Checking `ft8b.f90`'s actual `do ipass=1,4` / `decode174_91.f90`'s
  `maxosd` branches confirmed the raw-channel-LLR OSD path
  (`maxosd=0`) is a *different* WSJT-X depth setting FT8's blind
  `ndepth=3` dispatch never takes (that always sets `maxosd=2`, which
  only ever calls `osd174_91` on `zsave(:,i)`). So the pre-#182
  `osd_strategy.rs::try_fallback` loop that tried the 4 llr variants
  directly against `osd_decode_npre1`/`_npre2` — predating this
  session, from issue #63 — was itself not WSJT-X-faithful for this
  depth; the `bp_llr_zsum` fallback added above was layered *on top*
  of it rather than replacing it. Removed the direct-channel loop
  entirely and restructured the `zsave(:,1)`/`zsave(:,2)` loop to nest
  in WSJT-X's actual order (outer = llr variant / `ipass`, inner =
  `zsave` index — `ft8b.f90:294-298`, `decode174_91.f90:137-148`),
  rather than the previous variant-innermost ordering. Full workspace
  regression: 100% pass, zero decodes lost — every candidate that used
  to succeed via the direct-channel path also succeeds via
  `zsave`-seeded OSD. **Net effect vs the immediately-preceding
  zsum-as-addition state**: 12 OSD dispatches worst-case → 8 (33%
  fewer), single-threaded `qso3_busy.wav` blind decode
  1.37s → 1.27-1.30s, still 22/22 decodes.

- **`decode_frame_subtract_with_auto_ap` marked explicitly opt-in-only**
  (issue #182 follow-up) — with the `bp_llr_zsum` OSD fix above, this
  function's own motivating case (`K1BZM DK8NE -10`) now decodes via
  blind `decode_frame_subtract_with_ap` alone: measured on
  `qso3_busy.wav`, blind-only and blind-plus-auto-AP both return the
  same 22 decodes, so the auto-AP pass currently finds *zero*
  additional signals there while still paying its own full search cost
  (~0.3-0.5s single-threaded on top of blind). It was never wired into
  any default decode path (always a separate, explicitly-called
  function), so this is a documentation-only change making that
  explicit rather than a behavioural one — but the doc comment now
  says plainly not to reach for it by default until a concrete
  zsum-unreachable case justifies its cost again.

- **Reverted the now-pointless auto-AP `rayon` parallelism and
  `AudioSample: Sync` bound** (issue #182 follow-up) —
  `decode_block::auto_ap_strategy`'s per-callsign `par_iter` loop was
  parallelising a search that, per the entry above, now finds zero
  additional decodes on `qso3_busy.wav`: real multi-core speedup with
  no offsetting value, plus a second (parallel/sequential) code path
  to maintain for a function not wired into any default path. Reverted
  to one sequential path. With that gone, `AudioSample`'s `+ Sync`
  supertrait (added specifically to let `&[S]` cross that `rayon` task
  boundary) has no remaining caller that needs it — every other
  `rayon` usage in the codebase (`core/sync.rs`, `ft8/decode.rs`,
  `core/pipeline.rs`) operates on concrete `i16` or non-`AudioSample`-
  generic types where `Sync` was already automatic. Reverted to `Copy`
  only. Full workspace regression: 100% pass.

- **Bit-packed `osd_setup_ldpc174_91`'s Gaussian elimination — 6.8×
  faster setup, ~4× faster overall OSD dispatch** (issue #182
  perf follow-up). Profiling `try_fallback` found it was ~22-25% of
  total decode wall-clock (~281ms of ~1.27s on `qso3_busy.wav`, ~200
  candidates × up to 8 zsave-seeded dispatches each, ~1% success rate).
  Splitting a single `osd_decode_npre1` call's cost further (2000-rep
  synthetic-LLR microbenchmark) found **86% of it was
  `osd_setup_ldpc174_91`** (sort + Gaussian elimination to build the
  MRB systematic generator) — 113.5µs/call — versus only ~18µs/call
  for the actual `npre1` combinatorial search (the OSD algorithm
  itself). Root cause: `OsdSetup.g` stored the GF(2) generator matrix
  as **one byte per bit** (`Vec<u8>`, 91×174 bytes), so the
  elimination's row-XOR step did up to 174 individual byte XORs per
  row instead of ~3 packed 64-bit-word XORs — paid fresh on every one
  of the 8 zsave-seeded dispatches per candidate, since each has a
  different LLR reliability ordering and can't share a cached basis.
  Note this isn't a WSJT-X-fidelity gap: `osd174_91.f90`'s own
  `genmrb` is `integer*1` (byte-per-bit) too — bit-packing is a
  legitimate algorithmic improvement *beyond* what WSJT-X itself does,
  not a port of anything.

  Rewrote only `osd_setup_ldpc174_91`'s internals to build and
  eliminate on a packed `[u64; 3]`-per-row representation, unpacking
  to the existing `Vec<u8>` `g` format at the end — `OsdSetup`'s shape
  and every downstream consumer (`osd_npre1_pass`, `osd_npre2_pass`,
  `build_npre2_table`, `try_candidate_ldpc174_91`) are byte-for-byte
  unchanged, confining all risk to one function. Verified with a new
  differential test (`packed_elimination_matches_byte_reference`)
  asserting the packed rewrite produces identical `perm`/`g`/
  `hdec_perm`/`absrx_perm`/`c_perm` to a frozen byte-per-bit reference
  copy of the pre-rewrite code, across 5 seeded synthetic LLR vectors
  plus all-zero and exact-tie edge cases — not just "does the
  regression suite still pass."

  **Result**: `osd_setup_ldpc174_91` 113.5µs → 16.7µs/call (6.8×);
  total OSD dispatch (setup + `npre1_pass`) 131.6µs → 34.1µs/call
  (3.9×). End-to-end single-threaded blind decode of `qso3_busy.wav`:
  **1.27-1.30s → 1.09-1.12s** — landing right at real `jt9 -8 -d3`'s
  own measured ~1.1s total file decode time. Still 22/22 decodes, zero
  regressions (full workspace `cargo test --release --features full`,
  `qso3_apon_subtract_jtdx_extras_diag` still 6/6 JTDX extras).

- **`fine_refine_3stage` (`ft8/refine_fine.rs`) sped up ~45% by porting
  WSJT-X's real Stage-B/C algorithm** (issue #182 follow-up). Pulling
  real `jt9 -8 -d3`'s own `timer.out` breakdown found its BP+OSD stage
  (`dec174_9`) was 65% of jt9's *own* total runtime — while our own
  BP+OSD staircase (after the fixes above) was already ~2.5x faster
  than jt9's, our `process_candidate` "prelude" (dominated by
  `fine_refine_3stage`) still ran at ~2.2x jt9's own per-candidate rate
  for the equivalent step. Root-caused by reading `lib/ft8/sync8d.f90`
  and `lib/ft8/ft8b.f90:104-154` directly: our port's Stage-B/C
  frequency sweep shifted the *entire* ~3200-sample `cd0` baseband
  buffer per trial (11 trials/candidate, ~35,200 elements of work),
  while WSJT-X's real algorithm tweaks only the **32-sample Costas
  reference waveform** per trial (`ft8b.f90:133-140`'s `ctwk`,
  multiplied into `sync8d.f90`'s cached `csync` before conjugating) and
  never touches the data buffer at all — ~100x less arithmetic by
  construction, not a missing optimization. `sync8d.f90`'s `save
  first,twopi,csync` also confirmed WSJT-X caches its Costas reference
  table once for the *entire process*, not once per candidate.

  Two incremental fixes earlier in this same investigation (a
  once-per-candidate Costas-table lookup, then an NCO-rotation
  approximation for the data-shift with a tolerance-validated
  differential test) had already cut wall-clock from ~966ms to ~753ms
  single-threaded on `qso3_busy.wav` — real wins, but optimizing the
  *wrong* algorithm faster rather than replacing it. Porting WSJT-X's
  actual reference-tweak approach obsoletes both: the Costas table is
  now cached for the process lifetime (`std::sync::OnceLock`, `no_std`
  builds keep the once-per-candidate fallback), and the frequency sweep
  tweaks a 32-element reference (`build_tweak`, mirroring `ctwk` exactly
  — including the *non*-negated sign convention, since the tweak lands
  on the conjugated reference rather than the data) instead of shifting
  `cd0`. The old NCO-approximated `shift_freq` (and its `tmp` buffer
  allocation) is deleted entirely — nothing left to call it.

  Verified via a new differential test comparing the reference-tweak
  formulation against a frozen copy of the exact data-shift approach it
  replaces: measured max relative error `5.3e-6`, tighter than the
  `1e-5` bound (this is floating-point reassociation of two
  *exactly-equivalent* formulations, not an approximation — unlike the
  NCO fix it replaces). All 5 pre-existing `fine_refine_3stage` tests
  (including the sign-sensitive `freq_snap_positive_offset`/
  `freq_snap_negative_offset`) pass unchanged. Also hoisted the same
  "don't recompute a value that hasn't changed" fix into
  `core::sync::fine_sync_power_per_block` (used for `sync_cv`,
  shared across protocols) — FT8's 3 identical Costas sync blocks no
  longer rebuild the same reference waveform 3x per call; content-equal
  (not pointer-identity) caching keeps it correct for any `Protocol`.

  **Result**: single-threaded blind decode of `qso3_busy.wav`:
  **~753ms → ~682-719ms (avg ~700ms)** — ~39% faster than real
  `jt9 -8 -d3`'s own ~1.1s total file decode time. Full workspace
  regression (100% pass), `qso3_apon_subtract_jtdx_extras_diag` (still
  6/6 JTDX extras), and the no_std/fixed-point feature matrix across
  every protocol touched by the shared `core::sync.rs` change: all
  clean.

  Two smaller, related fixes landed in the same investigation: (1)
  `process_candidate`'s prelude reordered so `sync_quality`'s
  `nsync<=6` gate runs right after the cheap sync-symbol-only extraction
  and before the expensive 58-data-symbol FFT extraction — `sync_quality`
  never reads the data symbols, so the ~82% of candidates that fail this
  gate on `qso3_busy.wav` no longer pay for work whose result is never
  used. (2) `try_fallback`'s OSD dispatch now reuses the LLR variants
  `process_one_candidate_inner`'s own BP staircase already computed
  (`llra`/`llrd`/`llrb`/`llrc`) instead of a fresh `compute_llr`
  recompute — that recompute previously only got skipped when an AP
  hint was present (Gemini PR #81's original intent, avoiding a
  *second* recompute between OSD and the AP loop), so blind decode (the
  common case) always paid for nsym=3's full cost twice per
  OSD-reaching candidate. Also cached `bp_step_select`'s
  `MFSK_BP_KIND` env-var lookup (`std::sync::OnceLock`) instead of a
  syscall-and-allocate `std::env::var` on every one of the up-to-4
  BP calls per candidate — a debug-only A/B switch that never needs to
  change mid-process.

- **`core::dsp::subtract::refine_freq` sped up ~2.2x/call, fixing an FT4
  decode-speed regression** (issue #182 follow-up). While re-verifying
  every protocol's decode-speed benchmark row after the
  `fine_refine_3stage` fix above (`core::sync.rs`'s cache hoist touches
  every protocol), FT4's `decode_frame_subtract` golden-WAV wall-clock
  turned out to have silently regressed from 48.8ms to ~526-576ms —
  caused by an earlier, unrelated, already-merged commit (issues
  #178-#180, migrating FT4 onto FT8's WSJT-X-faithful channel-aware LPF
  subtract for recall-quality reasons) that was never re-benchmarked
  afterward. Root-caused (not just documented) via temporary `Instant`
  timers around the SIC subtract loop's two calls per accepted
  candidate: `subtract_tones_lpf` was fine (already FFT-cached from
  issue #180, <1ms/call); `refine_freq` cost ~35ms/call — essentially
  the entire regression (×14 real decodes ≈ 490ms).

  `refine_freq` grid-searches ±5Hz (FT4) / ±2.5Hz (FT8) at 0.1Hz
  resolution to compensate for coarse-sync's bin-quantized carrier
  estimate before subtracting. Every one of its ~50-100 evaluations
  called `generate_iq` → `synth_complex_f32_into` fresh, fully
  rebuilding the GFSK-shaped modulation (erf-based Gaussian pulse
  table, `O(nsym·pulse_len)` per-symbol convolution, full `O(nwave)`
  phase-integration + `sin`/`cos` loop) even though only the carrier
  frequency differs between evaluations of one call — the same
  "recompute something invariant across a search loop" bug pattern as
  the `fine_refine_3stage` fix above, this time in the protocol-agnostic
  subtract path shared by FT4 and FT8.

  `generate_iq`'s carrier term is added uniformly to every sample before
  phase integration, so `phi(k; f0) = phi_mod(k) + k·(2π·f0·dt)` — any
  carrier is a pure linear phase ramp on top of the carrier-free
  (tone-modulation-only) phase. Fixed by building the carrier-free
  phasor once per `refine_freq` call and deriving each grid point via a
  cheap per-sample NCO rotation + angle-addition instead of a full
  resynthesis (`ls_amp_mag_tweaked`), with periodic rotor
  renormalization to bound f32 drift over the ~59k (FT4) / ~334k (FT8)
  sample buffers. Verified against the frozen full-resynthesis path
  (`ls_amp_mag`, now `#[cfg(test)]`-only) with a differential test using
  a combined absolute/relative tolerance — the LS amplitude has a comb
  of deep correlation nulls a few tenths of a Hz apart where relative
  error alone isn't meaningful — plus an argmax-preservation test
  confirming the search still finds the true off-grid carrier.

  **Result**: `refine_freq` ~35ms/call → ~15.7ms/call
  (`RAYON_NUM_THREADS=1`, isolating per-call cost from thread count).
  `decode_frame_subtract` real production wall-clock: multi-threaded
  ~575.8ms → ~280ms, single-threaded ~893.6ms → ~602ms. Not a full
  return to 48.8ms — the LPF subtract + freq-refine step is a
  deliberate, permanent recall-quality cost (10/10 vs 0/10 on a
  Rayleigh-faded-interferer scenario, `docs/notes/FT4_BENCHMARK.md`
  section 13) that will always cost more than the pre-#178
  constant-amplitude path; this fix removes the *redundant* part of
  that cost, not the cost itself. Candidate count unchanged (31/31, no
  redundancy) and recall unaffected (`ft4_wsjtx_sample_recall_vs_golden`
  still 6/6). Also re-confirmed FT8's own SIC/JTDX golden suite
  byte-identical (`qso3_apoff` 7/8+7 phantom, `qso3_jtdx` 18/18,
  `qso3_apon` 6/6 JTDX extras) since `refine_freq` sits on FT8's subtract
  path too.

- **`refine_freq`'s search radius was 5x too wide, closing out the FT4
  decode-speed regression** (issue #182 follow-up). The NCO fix above
  cut `refine_freq` from ~35ms/call to ~15.7-16.2ms/call, but FT4's
  `decode_frame_subtract` golden-WAV wall-clock was still ~280ms — 5.7x
  the pre-#178 baseline of 48.8ms. Isolated `refine_freq` /
  `subtract_tones_lpf` with a standalone microbenchmark (14 calls each,
  matching the golden WAV's real accepted-decode count): `refine_freq`
  alone was 227ms of the 280ms total (81%), `subtract_tones_lpf` only
  13ms (already FFT-cached). The NCO fix reduced *per-evaluation* cost;
  the *evaluation count* (101/call, ±5Hz radius at 0.1Hz step) was
  untouched.

  Cross-checked the call site's "+/-5 Hz refine radius…matches WSJT-X"
  comment against `lib/ft4/subtractft4.f90` directly: WSJT-X's
  `subtractft4` has **no frequency-refine step at all** — it subtracts
  directly at the decoded `f0`, no grid search. The comment's "matches
  WSJT-X" claim was wrong; `refine_freq` compensates for mfsk-core's own
  coarse-frequency resolution, not anything WSJT-X does in its subtract
  path. The ±5Hz figure traced to `refine_freq`'s generic doc comment
  (written for `core::sync::coarse_sync`'s ~2.93Hz FFT-bin uncertainty)
  and was never re-derived after FT4 moved onto `ft4_coarse_sync` +
  `core::sync2d::ft4_sync_search` (issue #72). Reading `ft4_sync_search`'s
  df search line by line: both its coarse (`idf` step 3) and fine (`si`
  step 1) passes only ever produce integer-Hz `df` values, so the
  `freq_hz` `refine_freq` receives is always within ±0.5Hz of the true
  continuous optimum by construction — a much tighter bound than the
  ±2.5Hz the borrowed comment assumed.

  Fix: `ft4::decode::decode_frame_subtract_with_options`'s
  `refine_freq_radius_hz` `5.0 → 1.0` (0.1Hz step unchanged — the
  response's mainlobe is well under 1Hz wide for a ~7.5s tone train, so
  widening the step risks skipping it; only the radius was oversized).
  Cuts the grid 101 → 21 evaluations/call. **Result**: golden-WAV
  `decode_frame_subtract` wall-clock **280ms → 110ms** (~2.5x further,
  2.3x off the 48.8ms floor vs. 5.7x before this fix), recall
  byte-identical (6/6 golden, 14/14 total decodes). The Rayleigh-fading
  busy-band regression guard the LPF-subtract migration (#177-179) was
  built to fix (`ft4_busy_band_fading_probe.rs::busy_band_fading_baseline`)
  stayed 10/10. See `docs/notes/FT4_BENCHMARK.md` section 16.

- **`ft8::decode::DecodeDepth` redesigned from a flat, ad-hoc 4-variant
  enum into an orthogonal 2-field struct (breaking); the automatic
  auto-AP rescue it used to gate was found unconditionally costing
  ~1.2s for zero recall benefit and removed entirely** (issue #182
  follow-up). Investigating why FT8's WSJT-X AP-off golden floor stays
  at 7/8 (`K1BZM DK8NE -10` missing) surfaced that the ship-config
  benchmark call uses `DecodeDepth::BpVariantsAd`, which skips OSD
  entirely by design — not a fidelity bug, DK8NE genuinely decodes
  under `BpAllOsd` (confirmed directly: `decode_block(..., BpAllOsd,
  ..)` finds it at `freq=244.2 dt=0.510`). But the old
  `DecodeDepth` enum (`BpAll`/`BpAllOsd`/`BpAllNoNsym3`/`BpVariantsAd`)
  conflated two independent concerns — which LLR variants to try, and
  whether to escalate to OSD — into 4 named combinations that couldn't
  express "cheap variants + OSD" or any combination the original
  author hadn't happened to name. `BpAllNoNsym3` (a middle tier between
  the cheap and full variant sets) had zero real callers outside one
  dedicated sweep test.

  **New shape**:
  ```rust
  pub enum LlrEffort { Minimal, Full }
  pub struct DecodeDepth { pub llr_effort: LlrEffort, pub osd: bool }
  impl DecodeDepth {
      pub const EMBEDDED: Self; // Minimal, osd: false — was BpVariantsAd
      pub const BP_ONLY: Self;  // Full, osd: false — was BpAll
      pub const FULL: Self;     // Full, osd: true — was BpAllOsd
  }
  ```
  `BpAllNoNsym3` has no replacement — collapsed into the 2-tier scheme
  after both its own sweep test's history and a fresh host measurement
  (`qso3_busy.wav`) showed the LLR variants it dropped (`b`/`c`,
  WSJT-X `ft8b.f90`'s own `llrb`/`llrc` naming) contribute +2.5ms/+5.5ms
  wall-clock for **zero** extra decodes over the cheap `a`+`d` pair —
  not enough signal to justify a third named tier.
  `osd: true` is host-only by construction, not just convention: the
  OSD dispatch code (`decode_block::osd_strategy`, gated the same way
  `auto_ap_strategy` already was) is now `#[cfg(feature =
  "fft-rustfft")]`-excluded from non-host builds entirely, so it's
  impossible for an embedded build to accidentally link in OSD's
  Gaussian-elimination/combinatorial-search machinery, and `osd: true`
  is a silent no-op there rather than a footgun. OSD has never shipped
  on an ESP32 target and there is no plan to add it — this was always
  a permanent architectural boundary, now enforced structurally.

  **Auto-AP removed entirely, not just re-gated.** While migrating
  callers, `depth.osd`'s gate on `auto_ap_strategy`'s harvest-callsigns-
  and-retry rescue (issue #117) turned out to be doing double duty: it
  wasn't just gating the deliberately opt-in
  `decode_frame_subtract_with_auto_ap` research function, but was also
  the *only* thing preventing `auto_ap_strategy::run` (a separate,
  unbounded, 4x/200-candidate-widening variant) from firing
  **unconditionally** inside `decode_block_multipass` — the driver
  shared by *every* `decode_block*` entry point, `ap_hint` or not —
  whenever `depth.osd` was true. A plain `decode_block(audio, ...,
  DecodeDepth::FULL, ...)` call with no AP involvement at all was
  silently paying this cost. Measured directly on `qso3_busy.wav`
  (`RAYON_NUM_THREADS=1`): with it, `decode_block(FULL)` took
  ~1320-1450ms for 19 decodes at `max_cand` 60/200; without it, ~145-
  151ms for the *same* 19 decodes — 9x wall-clock for zero recall
  difference at any realistic `max_cand` (only `max_cand=15` lost 2
  decodes, a budget artifact of the unbounded variant's own internal
  200-candidate floor, not real AP value). This matches what this
  file's own CHANGELOG already found for the explicit
  `decode_frame_subtract_with_auto_ap` path — zero additional decodes
  once the OSD `bp_llr_zsum` fix (below) closed this mechanism's
  original motivating case (`K1BZM DK8NE -10`) through a different
  route. Also not a WSJT-X port: `ft8apset.f90`'s AP only ever uses the
  *operator-configured* `mycall`/`hiscall`, never same-slot decoded
  callsigns. With zero measured value, zero FFI/embedded/production
  consumers, and a real correctness surprise (`ap_hint`-independent
  cost), `auto_ap_strategy` (module, `run`/`run_bounded`,
  `decode_frame_subtract_with_auto_ap`) was deleted outright rather
  than re-gated — an app wanting this policy can rebuild it from the
  still-present, genuinely WSJT-X-faithful primitives (`ApHint`,
  `decode_block_with_ap`).

  Migration: `DecodeDepth::BpAll` → `DecodeDepth::BP_ONLY`,
  `::BpAllOsd` → `::FULL`, `::BpVariantsAd` → `::EMBEDDED`,
  `::BpAllNoNsym3` → no replacement (use `::BP_ONLY` or `::FULL`).
  `mfsk-ffi-ft8`'s C-facing `MfskFt8Depth` enum is unchanged (only its
  internal `map_depth()` target type changed shape) — no ABI break.
  `core::pipeline::DecodeDepth` (FT4/FST4's own, separate, already-
  clean 2-variant enum of the same name in a different module) is
  untouched — deliberately out of scope, since neither protocol runs
  on embedded today and it doesn't have the conflation problem this
  redesign targets.

  Verified: full workspace `cargo test --release --features full`
  (100% pass) and `-D clippy::perf -D warnings` clean; golden recall
  byte-identical everywhere it was checked (`DecodeDepth::EMBEDDED`
  still 7/8 WSJT-X AP-off golden / 14 total on `qso3_busy.wav`; JTDX
  AP-on extras still 6/6; staged-SIC `CQ DX DL8YHR JO41` still decodes;
  `LlrEffort::Minimal` vs `Full` still 32/40 vs 32/40 recall parity
  across the full in-repo corpus, `ft8_no_nsym3_sweep.rs`, adapted to
  the 2-tier scheme rather than deleted); all 3 embedded app crates
  (`m5stack-s3-app`, `m5stack-core2-app`, `m5stack-cores3-app`) plus the
  compute-bench crate build clean for their Xtensa targets; `mfsk-ffi`
  + C++ smoke driver green.

  **`fixed-point` (Q11i16, the numeric path embedded ships) separately
  verified — not part of the `full` feature set, so not covered by the
  checks above.** All FT8 tests pass under `--features
  fft-rustfft,ft8,uvpacket,parallel,fixed-point`, including both new
  tests (`ft8_qso3_full_parity_recall.rs` needed the same `SNR_TOL_DB`
  12→14 dB fixed-point widening `ft8_qso3_apoff_recall.rs` already
  uses — golden recall itself is 8/8 under fixed-point too, only the
  SNR-drift assertion needed the existing tolerance pattern). One
  unrelated pre-existing failure found and confirmed *not* caused by
  this change (reproduced identically on `main` before this PR, via a
  throwaway git-worktree check): `ft8_coarse_sync_bootstrap.rs`'s
  `bootstrap_dt_median_top5_matches_confirmed` under `fixed-point` —
  filed as [#189](https://github.com/jl1nie/mfsk-core/issues/189),
  left unfixed as out of scope here.

  **Also updated this pass** (issue #182 follow-up, same day):
  `docs/notes/BENCHMARKS.md` and `docs/notes/FT8_BENCHMARK.md`'s FT8
  AWGN/CCIR sweep tables re-measured (all 4 channels moved 0.6-1.0 dB
  more sensitive vs the last-tracked figures — confirmed by scope
  audit + a direct re-run that `decode_frame_inner`'s separate call
  graph never touched `auto_ap_strategy`, so this is accumulated prior
  work never rolled into the table, not an effect of this PR); a new
  permanent regression, `tests/ft8_qso3_full_parity_recall.rs`, tracks
  the **host full-parity** config (`DecodeDepth::FULL`, `sync_min=0.8`,
  `max_cand=60`) hitting the full WSJT-X 8-entry golden in ~139-148 ms
  (~7-8× faster than real `jt9 -8 -d3`'s own ~1.1 s); `README.md` /
  `docs/reference/LIBRARY.md` / `docs/reference/EMBEDDED.md` (+ `.ja.md`
  mirrors) updated for the renamed `DecodeDepth` API and to stop
  conflating ship-config's permanent 7/8 floor with the achievable 8/8
  host figure in top-level summary tables (they're different code
  paths by construction now, not a temporary gap — see `DecodeDepth`'s
  own doc comment).

### Changed

- **`DecodeRequest::flat()`/`.staged()` → `.sic_rounds(n)`/`.sic_early()`,
  `SupportsFlatSic`/`SupportsStagedSic` → `SupportsSicRounds`/
  `SupportsSicEarly`** (issue #218, breaking). The flat-SIC engine's
  round count was hardcoded to 3 (`for ipass in 0..3` in
  `sic_inner_passes_with_cache`, mirroring WSJT-X's `do ipass=1,npass`
  with `npass` fixed) with no caller-tunable upper bound — the exact gap
  `WsjtxDepth`'s own doc comment flagged: jt9 `-d1` runs SIC with
  `npass=2` (vs. 3 for `-d2`/`-d3`), but neither `.flat()` nor
  `.staged()` exposed a 2-vs-3-round knob, so `WsjtxDepth::D1` silently
  ran the full 3 rounds instead of matching jt9 `-d1` exactly. Round
  count is now a required argument on the strategy-selecting method
  itself (`.sic_rounds(n)`, clamped 1..=3 — WSJT-X's own `npass`/`nsp`
  never exceeds 3) rather than an independently-settable field, so
  `.sic_rounds(_).sic_early()` — where the round count would be
  silently ignored by the early-decode strategy — is structurally
  unwritable rather than a compiling no-op. `WsjtxDepth::D1` now uses
  `.sic_rounds(2)`, closing the gap; `D2`/`D3` use `.sic_early()`
  (checkpoint structure is fixed at 3, not exposed as a knob).

  Renamed rather than kept as an additive alias: `flat`/`staged`
  described mfsk-core's own internal buffer-structure axis (single
  full buffer vs. checkpoint-replayed growing prefixes), not
  self-descriptive at a bare call site and not WSJT-X's own vocabulary.
  `sic_early` borrows WSJT-X's actual term for the checkpoint mechanism
  (`ndec_early`/`MAX_EARLY` in `ft8_decode.f90`, checkpointed at
  `nzhsym` = 41/47/50 out of 79 symbols); `sic_rounds` avoids `pass`,
  which is overloaded in WSJT-X's own source — FT8's `ft8_decode.f90`
  `ipass`/`npass` *is* the subtraction loop, but FT4's `ft4_decode.f90`
  reserves `ipass`/`npasses` for an unrelated AP-hint-variant loop
  inside a single decode attempt, using `isp`/`nsp` for the actual
  subtraction loop instead. `SupportsFlatSic`/`SupportsStagedSic` were
  renamed alongside the methods they gate so a trait-bound compile
  error names something a reader can connect back to the method they
  called.

  FT4's flat-SIC engine (`engine::pipeline::decode_frame_subtract`)
  gained the same `.sic_rounds(n)` knob — its progressive-`sync_min`-
  relaxation pass array (`&[1.0, 0.75, 0.5]`, mfsk-core's own
  pre-migration design, not WSJT-X's) is now sliced to `n` rounds
  rather than always iterated in full. FT4's SIC still isn't
  WSJT-X-faithful in its own right (`ft4_decode.f90` uses a **fixed**
  `syncmin=1.18` across rounds, the same fixed-threshold design FT8
  already migrated to — see `flat_sic_inner`'s doc comment) — tracked
  as a separate follow-up, out of scope here.

  `.sic_rounds(3)` verified byte-identical to the pre-rename `.flat()`
  default on `qso3_busy.wav` (20/20 matching decodes, checked against a
  clean pre-#218 `main` worktree); new `tests/ft8_sic_rounds_recall.rs`
  locks in both that golden and a `sic_rounds(1) ⊆ sic_rounds(2) ⊆
  sic_rounds(3)` monotonicity invariant (measured 13/19/20 decodes).
  `&[1.0, 0.75, 0.5][..max_rounds]` is genuinely new slicing logic on
  FT4's side, not just a rename, and the FT8 golden above doesn't
  exercise it — a self-contained (no external sample-tree dependency)
  six-station synthetic scene in new `tests/ft4_sic_rounds_recall.rs`
  covers the same monotonicity invariant there (measured 4/6/6
  decodes; asserts `sic_rounds(1) < sic_rounds(3)` so the scenario
  can't pass vacuously).

- **`DecodeRequest`/`SniperRequest::depth(DecodeDepth)` → `.osd(bool)`
  (breaking).** `LlrEffort` was a dead lever on host — its own doc
  comment already says the 2-/3-symbol LLR estimates it toggles
  "empirically add zero extra decodes"; a full-repo grep of every
  `.depth(...)` builder call site (crate tests, `mfsk-ffi`,
  `bench/wasm`) found zero cases ever passing `LlrEffort::Minimal`
  through either builder — it exists solely for `decode_block_into`'s
  ESP32 power budget (`DecodeDepth::EMBEDDED`). Both builders now
  hardcode `LlrEffort::Full` internally and expose only the `osd`
  toggle that callers actually used (`DecodeDepth::FULL`/`BP_ONLY` →
  `.osd(true)`/`.osd(false)`, default unchanged at `true`).
  `DecodeDepth` itself is untouched and still required positionally by
  `decode_block`/`decode_block_into` (the embedded/host-shared
  function API) and by `mfsk-ffi-ft8`'s C ABI (`MfskDecodeDepth`,
  unaffected — it's a separate 2-variant `#[repr(C)]` mirror that never
  crossed the builder boundary). `mfsk-ffi`'s internal `map_depth` helper
  is renamed `map_osd` (returns `bool`) to match; its C ABI surface is
  unaffected.
- **Q65-60B/30A `decode_multi_period_for` sped up ~4×**
  (`q65/rx.rs`, `tests/q65_wsjtx_samples.rs`) — two stacked fixes: (1)
  the fast-fading `b90 × model` sweep (6 combinations) was redundantly
  re-extracting FFT-based energies from scratch for each combination
  instead of extracting once and reusing; (2) `max_candidates=32` hit
  its cap on every slot, paying the full 8-stage decode ladder for
  candidates far below the real signal's own score (confirmed #0-ranked
  in every slot of both golden recordings via score-distribution
  profiling) — cut in two verified steps (32→16, then — after
  re-profiling showed the fading-BP stage still ~79% of wall-clock even
  after fix (1) — 16→8, matching `SearchParams::default()`). Golden-WAV
  wall-clock: Q65-60B 1.89s → 0.49s, Q65-30A 2.55s → 0.64s,
  bit-identical recall at every step.
- **Q65-60A/`decode_scan_for` family rewritten as a faithful `(Δf, Δt,
  b90)` grid search** (`q65/rx.rs::decode_at_grid_for`) — WSJT-X's
  `q65_loops.f90` has no separate "plain BP" path at all, always
  sweeping the fast-fading metric over a submode-specific b90 range in
  an `ndepth`-gated grid; our previous port diverged into a
  narrow-window AWGN-only Bessel metric with a time-only retry. Landing
  the faithful port surfaced three further bugs found via a real-`jt9`
  cross-check: a missing full/unpruned `ibw` sweep at the origin cell
  (WSJT-X's primary `q65_dec_q012` decode stage, which the pruned
  `q65_loops` fallback alone doesn't cover — worst for wide-`ibwa`
  C/D/E sub-modes); coarse-sync time resolution 4× coarser than
  WSJT-X's own `NSTEP=8` (`q65/search.rs::Spectrogram`, fixed alongside
  a `q65_ccf_22`-style restructure — per-frequency time-collapse +
  local-max NMS + noise-adaptive percentile admission — needed to avoid
  regressing a real off-air multi-signal recording); and a wrong fading
  model — `q65_dec1`/`q65_dec2` hardcode Lorentzian, not Gaussian, but
  the port used Gaussian, invisible for narrow-`b90` A/B sub-modes and
  costing wide-`b90` C/D/E sub-modes ~2.5-3 dB. With all three fixed,
  all ten sub-modes sit within ~1 dB of real `jt9`'s own crossing. Real
  off-air golden-test recall also *improved* (6 m EME sample: 3 → 4
  messages recovered). A follow-up `max_candidates` calibration
  (`tests/q65_wsjtx_samples.rs`, score-distribution profiling — same
  methodology as the Q65-60B/30A cut above) found all 4 real signals in
  the 6 m EME golden recording ranked within the top 8 of 530 coarse
  candidates; cut the test's `max_candidates` 32→16 (2× headroom,
  deliberately not pushed to the edge given a thin ~0.002 score margin
  at the weakest signal's rank), dropping golden-WAV time 1.49s→0.69s
  (~2.2×) with bit-identical recall. The same calibration then extended
  to the four fading-metric golden tests (`decode_scan_fading_for`,
  which don't go through `decode_at_grid_for` but share the coarse-sync
  overhaul's `coarse_search_for` and had gotten slower from it): real
  signals ranked within the top 3 of 94-3094 candidates in all four
  recordings (Q65-300A's margin over the next candidate is thin,
  ~0.0004, so its cut kept more headroom than the others). Cutting
  each test's `max_candidates` (100/30/30/200 -> 8/8/8/20) dropped
  Q65-60D 0.39s->0.08s, Q65-120D 0.15s->0.12s, Q65-120E 0.32s->0.26s,
  Q65-300A 1.05s->0.34s, bit-identical recall throughout. See
  `docs/notes/Q65_BENCHMARK.md`.
- **FST4-60A OSD depth-escalation gate hand-calibrated for its own
  `N_SYNC=40`** (`core/pipeline.rs`) — the shared `osd_depth3_min=18`
  gate was calibrated against FT8's `N_SYNC=21`; FST4's larger sync
  sequence made 18 a far looser bar, escalating roughly half of all
  candidates into the expensive OSD depth-3/4 tier regardless of signal
  quality. Golden-WAV decode time dropped **2.60 s → 0.27 s (~8.4×)**,
  recall verified unchanged (4-channel AWGN/CCIR sweep matching the
  documented pre-fix baseline almost exactly, FST4-120/300 spot-checked).
  A first attempt (reusing FT4's exact `N_SYNC`-scaled formula) measured
  as a real ~0.5 dB AWGN sensitivity regression and was corrected before
  shipping — see `docs/notes/FST4_BENCHMARK.md` section 8.
- **FT4 coarse-candidate stage replaced with a faithful
  `getcandidates4.f90` port** (`core::ft4_coarse::ft4_coarse_sync`) —
  the generic Costas-lag coarse_sync FT4 previously shared with
  FT8/FST4 was a structurally different algorithm from what WSJT-X
  actually does for FT4 (a 2-D freq×lag correlation search vs. WSJT-X's
  frequency-only periodogram), producing ~4.5× redundant candidates
  per real signal on busy recordings. Golden-WAV decode time dropped
  **1.20 s → 0.049 s (~25×)** with byte-identical 6/6 recall; AWGN 50%
  sensitivity crossing moved −17.2 dB → −16.9 dB (structural algorithm
  swap reshaped rather than uniformly improved the SNR curve — 3 of 4
  channels held or improved). See `docs/notes/FT4_BENCHMARK.md` section
  13.
- **Q65-60B/30A `decode_multi_period_for` sped up further, ~2.3×/~1.3×**
  (`q65/search.rs::Spectrogram::build_for`) — follow-up to a separate
  FFT-cache wiring investigation on FT8's `decode_block` (same "does
  this pattern exist elsewhere" question, applied to Q65). That
  specific pattern wasn't present here: `decode_at_grid_for`'s
  `GridDepth::Fast` has exactly one `(Δf,Δt)` grid cell per candidate
  (confirmed via direct call-count instrumentation — 16 calls for
  Q65-60A's 16 candidates, not the ~333 an earlier estimate assumed),
  so no redundant FFT-planner construction existed to cache. Phase-by-
  phase profiling of `decode_multi_period_for` found a different bug
  instead: `Spectrogram::build_for`'s noise-floor estimate (run once per
  candidate-search call) computed a trimmed mean of the bottom 95% of
  FFT-magnitude bins via a full `sort_unstable_by` (O(n log n)) over the
  entire magnitude array (hundreds of thousands to millions of cells for
  a 60-120 s slot) when only the unordered *set* of bottom-95% values
  was needed. Same class of fix already applied to FT8's
  `xsnr2_db_simple` noise-floor median — swapped to
  `select_nth_unstable_by` (O(n) average partition), Q65 just hadn't had
  it applied. Measured as 64% of wall-clock on Q65-60B (short slot, low
  candidate count) and 12% on Q65-30A (longer multi-slot audio where the
  BP/fading-metric stage dominates instead). Golden-WAV decode time:
  Q65-60B **0.28 s → 0.12 s**, Q65-30A **0.72 s → 0.56 s**, byte-
  identical recall on both (VK7MO/VK7PD on 60B, K1JT/K9AN AP-list on
  30A).

### Removed

- **Legacy BASIS (Q15 sin/cos dot-product) per-symbol DFT fill path**
  (#162) — dead weight since the 0.6.4 Goertzel migration (Phase
  1.7.7-Stick) made Goertzel the sole production fill on every
  embedded target, but the old path's functions and scratch
  parameters were never actually deleted. Removed
  `fill_symbol_spectra_into`/`fill_symbol_spectra_into_generic`,
  `BASIS_SCRATCH_LEN`, `symbol_spectra_direct_into`, the
  `mfsk_core::core::dotprod` module (`dot_q15_i32` + the
  `mfsk_core_dot_q15_i32` extern symbol contract), and its esp-dsp
  ASM bridge in `embedded-shared::esp_dsp_fft`
  (`mfsk_core_dot_q15_i32`, `dsps_dotprod_s16_ae32`,
  `ESP_DSP_DOT_SHIFT`/`set_dot_shift`). `fill_symbol_spectra_generic`
  (the plain rotator-based DFT) is now unconditional — it no longer
  needs a separate `fixed-point`-gated BASIS-backed overload.
- **Breaking FFI change**: `mfsk_ft8_decode_i16`'s C ABI
  (`mfsk-ffi-ft8`) dropped its `basis_re`/`basis_im` `int16_t*`
  scratch parameters — they were dead weight once Goertzel took
  over, and the crate's `mfsk_ft8_basis_scratch_len()` sizing
  function is gone with them. **C callers must drop both arguments
  from their call site** when upgrading past 0.7.x. This forces the
  minor version bump for this release (0.7.4 → 0.8.0) ahead of the
  otherwise-patch-level content below.
- `basis_re`/`basis_im` parameters dropped from every Rust function
  in the call chain down to that FFI boundary:
  `decode_block_into[_tuned]`, `refine_candidates_into`,
  `process_candidates_into[_tuned]`,
  `process_candidates_into_with_cs_scratch[_tuned]` (all
  `mfsk-core`), and `dual_core::init`/`pass2_split`/`stage3_split`/
  `run_speculative_slot` (`embedded-shared`) — all embedded app
  crates (`m5stack-s3-app`, `m5stack-core2-app`, `m5stack-cores3-app`)
  updated to match; none of them were actually using the scratch for
  anything by this point (Goertzel needs none), so this is a pure
  signature simplification with no behavioral change on any target.
- `mfsk-core/tests/ft8_goertzel_vs_basis.rs` deleted — its entire
  purpose was A/B-validating BASIS against Goertzel during the
  Phase 1.7.7-Stick migration, which is moot once BASIS no longer
  exists to compare against.
- **`core::pipeline`'s unreachable generic-refine fallback + dead
  `refine_candidate_double`** (issue #192 investigation) — auditing
  whether FT8's decode engine should unify with `core::pipeline` (as
  #192 originally proposed) found the reverse problem first: the
  generic engine already carried speculative generality nothing used.
  An exhaustive call-graph audit (every call site in `src`, `tests`,
  and doc examples) confirmed two dead branches:
  - `process_candidate_basic`'s bare `refine_candidate::<P>` fallback
    (time-only refine, no frequency correction) — every real caller is
    `Ft4` or an FST4 sub-mode, both of which already take the
    frequency-aware `ft4_sync_search`/`fst4_sync_search` branch; FT8
    never calls into this pipeline at all (it has its own bespoke
    engine). Replaced the runtime branch with a new sealed-by-convention
    marker trait, `core::pipeline::GenericPipelineProtocol: Protocol`,
    implemented only for `Ft4` and the five FST4 sub-modes — calling
    `process_candidate_basic`/`decode_frame`/`decode_frame_subtract`
    with any other protocol is now a compile error instead of silently
    falling back to an unvalidated code path.
  - `core::sync::refine_candidate_double` and its `FineSyncDetail`
    result type — a generalized "double sync" (independent first-block/
    last-block refine + drift estimate) with zero callers anywhere in
    the crate; FT8's own copy of this idea was already removed in #49.
    Deleted both outright, along with the now-pointless `FineSyncDetail`
    re-export from `ft8::sync`.
  - The now-dead `refine_steps` parameter threaded through
    `process_candidate_basic`/`decode_frame`/`decode_frame_subtract`
    (only consumed by the removed fallback) was also dropped from all
    three signatures — `Ft4`/FST4 callers no longer pass it.
  - **Not done**: full FT8/`core::pipeline` engine unification, as #192
    originally scoped it. That would mean porting FT8-only machinery
    (blind-CQ AP pass, non-AP OSD fallback, lazy LLR-effort staircase,
    two-phase sync/data fill, frequency-aware 3-stage refine) into a
    layer with no second consumer — the opposite direction from this
    cleanup. Issue #192 closed with this narrower fix instead; FST4 SIC
    and `DecodeResult` semantic unification remain tracked separately
    in #193/#194.

    **Correction, next day (2026-07-27): #192 reopened.** A re-audit
    against the then-current code found 3 of the above close
    rationale's 4 "FT8-only, no second consumer" claims didn't hold
    up — the blind-CQ AP pass and non-AP OSD fallback both already had
    independent, parallel implementations serving FT4/FST4
    (`msg::pipeline_ap`'s pass 7, `core::pipeline::osd_escalation_gates`
    respectively) at the time this entry was written, and the lazy
    LLR-effort staircase was ported into `core::pipeline` the very
    next day (`4801722`, folded into this same 0.8.0 cycle) — only the
    frequency-aware 3-stage refine (`fine_refine_3stage`) is still
    genuinely FT8-only. Closing #192 made this duplication invisible
    instead of resolving it; it remains open as of this writing, not
    resolved by the narrower fix described above. As of 2026-08-08,
    still unstarted (`fine_refine_3stage` has had zero commits since
    the reopen) — see the issue thread for the full re-audit.
- **Breaking**: demoted the pre-#191 raw engine functions `DecodeRequest`/
  `SniperRequest` wrap to `pub(crate)` (issue #203, part of the pre-0.8.0
  public-API review tracked in #206) — `core::pipeline::{decode_frame,
  decode_frame_subtract, process_candidate_basic, osd_escalation_gates}`,
  the `GenericPipelineProtocol` trait, and `msg::pipeline_ap::{
  decode_sniper_ap, ap_bits_for, ap_passes, process_candidate_ap}` were
  still reachable at their pre-#191 legacy-shaped call sites even though
  #191 removed the protocol-module wrappers around them. `DecodeRequest`/
  `SniperRequest` are the only supported entry points now. Also
  `#[doc(hidden)]`'d `uvpacket::rx::{diag_sync_at, diag_estimate_freq_offset}`
  and `ft8::decode_block::process_candidates_into_with_cs_scratch{,_tuned,
  _tuned_with_fill}` (kept `pub`, not `pub(crate)`, since `embedded-poc`'s
  `embedded-shared::dual_core` depends on the `_tuned` variant as an
  external path dependency outside the workspace). A new non-`full`
  `internal-testing` feature keeps the demoted `core::pipeline` items
  `pub` for `mfsk-core/tests/{fst4_sweep,ft4_sweep,fst4_wsjtx_samples}.rs`,
  which call them directly as diagnostics — CI enables it alongside
  `full` for `cargo test`/`cargo clippy --all-targets` only; `cargo doc`/
  `cargo publish` stay on the `pub(crate)` shape downstream consumers
  actually see.
- **Breaking**: `core::pipeline::FftCache` (re-exported at
  `ft8::decode::FftCache`) is now an opaque newtype instead of `pub type
  FftCache = Vec<Complex<f32>>` (issue #206, pre-0.8.0 public-API
  review) — the old alias leaked `num_complex::Complex`, a dependency's
  type, into the public API. No public constructor and no way to
  inspect the contents; obtain one from a `decode_frame`-family return
  value / `DecodeOutcome::fft_cache` and pass it straight back into
  `DecodeRequest::fft_cache`. Added `FftCache::len`/`is_empty` for the
  one query downstream code plausibly needs.
- #203's `pub(crate)` demotion (above) surfaced `-D warnings`
  `dead_code` failures across the `feature-matrix` CI job's
  single-protocol builds (`fst4`, `jt9`, `jt65`, `q65`, `uvpacket`, the
  `alloc ft8 fft-extern[,fixed-point]` embedded presets) — several
  `core::pipeline`/`msg::pipeline_ap` items are only reachable via
  `ft4`/`fst4`'s `decode` modules, so any combination excluding both
  now correctly sees them as unreachable (previously masked because
  `pub` items are exempt from the lint regardless of in-crate callers).
  `#[allow(dead_code)]` added at each such item.
- **Breaking**: renamed the `mfsk_core::core` module to `mfsk_core::engine`
  (issue #206, pre-0.8.0 public-API review) — `pub mod core` shadowed
  Rust's own `core` crate at every scope where both names were
  simultaneously visible (the crate root, and anywhere writing a bare
  `core::` path meaning std rather than this module), which is exactly
  the kind of surprise a public API shouldn't hand downstream
  consumers. Mechanical rename: `core::pipeline`, `core::protocol`,
  `core::sync`/`sync2d`, `core::dsp::*`, `core::llr`, `core::equalize`,
  `core::scalar`, `core::fft`, `core::tx`, `core::ft4_coarse`,
  `core::baseline`, and the flattened re-exports (`DecodeContext`,
  `FecCodec`, `FecOpts`, `FecResult`, `FrameLayout`, `MessageCodec`,
  `MessageFields`, `ModulationParams`, `Protocol`, `ProtocolId`,
  `SyncBlock`, `SyncMode`) all move to `engine::*` with no behavior
  change. `embedded-poc` (path-dependency, outside the workspace)
  updated to match — not compile-verified here (no `+esp` toolchain in
  this environment); run `cargo check` there before flashing.
- **Breaking**: unified `ft8::decode::DecodeResult` with
  `engine::pipeline::DecodeResult` (issue #194, pre-0.8.0 public-API
  review) — FT8's own struct was byte-for-byte identical to the
  generic pipeline type (used by FT4/FST4) except for
  `message77: [u8; 77]` (CRC bits stripped) vs. the generic type's
  `info: Box<[u8]>` (full `K` FEC info bits, CRC retained) +
  `message77()` accessor slicing the leading 77. FT8's own BP/OSD
  engine already produced the full `info` at its one production
  construction site (`fec::ldpc::bp::BpResult::info`) — it was just
  being discarded in favor of the 77-bit-only field. Rather than just
  matching the shape, FT8 now literally re-exports
  `engine::pipeline::DecodeResult`, so a protocol-generic caller over
  `DecodeRequest<P>` can read every protocol's results the same way.
  `mfsk_core::msg::wsjt77::{unpack77, unpack77_with_hash}` and
  `mfsk_core::ft8::wave_gen::message_to_tones` relaxed from `&[u8; 77]`
  to `&[u8]` to match — a strict widening (any existing `&[u8; 77]`
  caller still compiles via unsized coercion) that also lets
  `result.message77()`'s `&[u8]` return flow in directly, without the
  copy-into-a-scratch-`[u8; 77]`-array dance FT4/FST4 callers
  previously needed. `WsprDecode`/`Jt9Decode`/`Jt65Decode`/`Q65Decode`
  are deliberately untouched — none of those protocols adopted
  `DecodeRequest`/`FrameDecodable` (#191), so unifying their naming
  with this family is deferred to #204's Q65 builder design pass
  rather than done piecemeal here.
- **Sealed `FecCodec`** (issue #198, pre-0.8.0 public-API review) — a
  private `sealed::Sealed` supertrait bound, implemented for all seven
  in-crate implementors (`Ldpc174_91`, `Ldpc240_101`, `Ldpc128_90`,
  `ConvFano`, `ConvFano232`, `Rs63_12`, `Q65Fec`), blocks downstream
  crates from implementing `FecCodec` themselves. `decode_soft` is
  still f32-hardcoded — genericizing it (`decode_soft<T: LlrScalar>`,
  which would unlock fixed-point BP for FT4/FST4/MSK144 through the
  generic pipeline, currently FT8-only via its own bespoke engine) is
  deliberately deferred: real numerical work with its own verification
  cost, decided (2026-07-27) as a stretch goal rather than a 0.8.0
  requirement. Sealing now means that redesign can land later as a
  signature change on existing implementors without breaking any
  downstream implementor, since none can exist.
- **Breaking**: migrated Q65's decode API to a `DecodeRequest`/
  `SniperRequest`/`MultiPeriodRequest` builder (issue #204, pre-0.8.0
  public-API review) — the largest single item in the review. Q65 still
  carried the full pre-#191 `_with_*`/`_for` suffix explosion (15 public
  entry points in `q65::rx`: `decode_at`, `decode_at_for`,
  `decode_at_fading_for`, `decode_at_with_ap`, `decode_at_with_ap_for`,
  `decode_at_with_ap_list_for`, `decode_multi_period`,
  `decode_multi_period_for`, `decode_scan`, `decode_scan_default`,
  `decode_scan_for`, `decode_scan_fading_for`, `decode_scan_with_ap`,
  `decode_scan_with_ap_for`, `decode_scan_with_ap_list_for`) — the exact
  combinatorial disease #191 collapsed into `DecodeRequest<P>` for
  FT8/FT4/FST4, shipping 0.8.0 with two contradictory decode-API
  philosophies otherwise.

  Q65 gets its **own** builder types (`q65::{DecodeRequest, SniperRequest,
  MultiPeriodRequest}`) rather than reusing
  `msg::decode_request::{DecodeRequest, SniperRequest}`: every `q65::rx`
  function (and the FFI layer wrapping it) operates on `&[f32]` audio,
  but the WSJT77-family builders hardcode `audio: &'a [i16]` — a real
  architectural difference (Q65's own decode chain works in float
  throughout, unlike the integer WSJT77-family path), not an oversight.
  `decode_multi_period*`'s `&[&[f32]]` (one buffer per T/R slot) is a
  further distinct shape from either wide-scan or single-target decode,
  hence the third builder type.

  - `DecodeRequest::<P>::new(audio, sample_rate, nominal_start_sample,
    params)` replaces `decode_scan*`; `.sniper(...)` (or
    `SniperRequest::<P>::new`) replaces `decode_at*`.
  - `.ap_hint(&ApHint)`, `.ap_list(&[[i32; 63]])`, and
    `.fading(model, b90_ts)` are plain inherent methods (not
    capability-gated marker traits like FT8/FT4/FST4's
    `SupportsWideBandAp`): every Q65 sub-mode supports every capability
    uniformly, so there is no invalid combination to guard against at
    the type level. A new `Q65SubMode: Protocol` marker (implemented for
    the 10 wired sub-mode ZSTs) stops the builders from compiling
    against a non-Q65 protocol and silently producing garbage.
  - `MultiPeriodRequest::<P>::new(audio_slots, sample_rate,
    nominal_start_sample, params)` + `.ap_list(...)` replaces
    `decode_multi_period*`.
  - The Q65-30A convenience wrappers (`decode_at`, `decode_at_with_ap`,
    `decode_scan`, `decode_scan_with_ap`, `decode_scan_default`,
    `decode_multi_period`) are gone entirely — `DecodeRequest::<Q65a30>`
    is not meaningfully more to type.

  Migration (issue #207 — call-site reshapes lose `rustc`'s "did you
  mean" hint that plain renames get, so worked examples save the
  source-reading detour):

  ```rust
  // before: wide-band scan
  decode_scan_for::<Q65a30>(&audio, sr, start, &params)
  // after
  DecodeRequest::<Q65a30>::new(&audio, sr, start, params).decode()

  // before: single-target sniper
  decode_at_for::<Q65a30>(&audio, sr, start, freq_hz)
  // after
  SniperRequest::<Q65a30>::new(&audio, sr, start, freq_hz).decode()

  // before: wide-band scan + fast-fading metric + AP hint
  decode_scan_fading_for::<Q65a60>(&audio, sr, start, &params, b90_ts, model, Some(&ap_hint))
  // after
  DecodeRequest::<Q65a60>::new(&audio, sr, start, params)
      .fading(model, b90_ts)
      .ap_hint(&ap_hint)
      .decode()
  ```

  All 15 functions demoted to `pub(crate)` (the underlying engine, used
  internally by the new builders). Updated every call site: 9
  `mfsk-core/tests/q65_*.rs` integration tests (~85 call sites) and
  `mfsk-ffi`'s Q65 FFI dispatch (~50 call sites across the 10-sub-mode ×
  4-capability match tables) — the C ABI itself is unchanged, only the
  internal Rust wiring moved to the new builders.

  Verified clean across all 14 feature-matrix combinations, clippy
  `--workspace --all-targets`, full test suite + doctests (including
  every Q65 WSJT-X real-off-air-sample recall gate: 10 GHz EME, 6 m
  ionoscatter/EME, 1296 MHz troposcatter, 120D rainscatter, optical
  scatter), `mfsk-ffi`/`mfsk-ffi-ft8` test suites, and `cargo doc`.
- **Breaking**: unified `mfsk-ffi` and `mfsk-ffi-ft8`'s C ABI
  conventions (issue #205, pre-0.8.0 public-API review) — the two FFI
  crates had independently evolved incompatible shapes for the same
  domain: clashing status vocabularies (`MfskStatus` vs
  `MfskFt8Status`, both using `-1..-4` for different meanings),
  duplicate result structs with different text-ownership models
  (`mfsk-ffi`'s `MfskMessage` held a heap `CString*` per message;
  `mfsk-ffi-ft8`'s `MfskFt8Result` always used a fixed inline buffer),
  and decode entry points that hardcoded every tuning knob
  positionally with no room to grow.

  New zero-dependency `no_std` crate, `mfsk-ffi-abi`, is now the
  single source of truth for the shared shape: `MfskStatus` (`Ok`,
  `NullPointer`, `InvalidArg`, `UnknownProtocol`, `DecodeFailed`,
  `Internal`), `MfskDecodeDepth` (`BpAll`/`BpAllOsd`, mirroring
  `engine::pipeline::DecodeDepth`), `MfskResult`/`MfskResultList`
  (fixed 40-byte inline `text` buffer, not a heap pointer — the whole
  list is one allocation freed in one call), and an opaque
  `MfskDecodeOptions` handle. Not published, not a C ABI on its own —
  each consuming crate `pub use`-re-exports these types so its own
  cbindgen-generated header carries identical definitions; not
  parsed together in the same C translation unit (only one of
  `mfsk.h`/`mfsk_ft8.h` is linked per target).

  - `mfsk-ffi-ft8`: dropped its local `MfskFt8Status`/`MfskFt8Depth`/
    `MfskFt8Result`/`MfskFt8ResultList` in favor of the shared types.
    `mfsk_ft8_decode_i16` (host feature) and the pre-existing
    `mfsk_ft8_decode_i16_alloc` are unified into one symbol name,
    `mfsk_ft8_decode_i16`, taking an `MfskDecodeOptions*` (NULL = this
    crate's built-in default) instead of five positional tuning
    arguments — **C callers must rename `mfsk_ft8_decode_i16_alloc`
    call sites and switch to an options handle.** Host error strings
    moved from `thread_local!` (unavailable in `no_std`) to a
    documented single-threaded `static mut` buffer written via raw
    pointer arithmetic (`&raw mut`/`copy_nonoverlapping`, satisfying
    the `dangerous_implicit_autorefs` lint).
  - `mfsk-ffi`: dropped its local `MfskStatus`/`MfskMessage`/
    `MfskMessageList` in favor of the shared types (renamed
    `MfskMessageList`→`MfskResultList`, `MfskMessage`→`MfskResult`,
    `mfsk_message_list_free`→`mfsk_result_list_free`). `MfskResult::text`
    is now a fixed inline buffer instead of a heap `CString*` — no
    more per-message `CString::from_raw` in the free path, matching
    `mfsk-ffi-ft8`'s existing model. **Breaking**: `mfsk_decode_f32`/
    `mfsk_decode_i16` gained a new `options: *const MfskDecodeOptions`
    parameter (construct with `mfsk_decode_options_new`, release with
    `mfsk_decode_options_free`) — **C callers must add a NULL (or
    real) argument at every call site.** NULL preserves each
    protocol's pre-0.8.0 hardcoded defaults exactly (FT8: 200-3000 Hz/
    sync_min 2.0/max_cand 50/`BpAllOsd`; FT4: sync_min 1.2; FST4-60A:
    100-3000 Hz/sync_min 0.8/max_cand 30); a non-null handle overrides
    freq range/sync_min/max_cand uniformly, and depth for FT8. Q65's
    generic-handle path (`decode_q65_default`) also honours the
    freq-range/max_cand override; `decode_wspr`/`decode_jt9_aligned`/
    `decode_jt65_aligned` and the dedicated `mfsk_q65_*` function
    family are untouched (no tunable search knobs to wire, or a
    separate ABI surface out of scope for this issue).

  Caught during verification, fixed as part of this same change:
  both crates' `cbindgen.toml` had `parse_deps = false`, which made
  cbindgen silently emit function signatures referencing the shared
  types **without ever `typedef`-ing them** in the generated header
  (a `pub use`-re-exported type isn't visible to cbindgen unless it
  parses the defining crate) — `MfskResult` in particular collapsed
  to a field-less opaque forward declaration because its `text` field
  used a `MFSK_TEXT_CAP + 1` compound array-length expression cbindgen
  can't evaluate across a crate boundary (a bare literal works fine,
  hence `mfsk-ffi-abi::MFSK_TEXT_BUF_LEN` — a derived constant used
  Rust-side, with a `const _` assertion keeping it in sync — while the
  struct field itself stays a literal `40`). Fixed by turning on
  `parse_deps = true` with `include = ["mfsk-ffi-abi"]` (not a bare
  `parse_deps = true`, which would additionally — and needlessly —
  parse all of `mfsk-core` through the `mfsk-ffi`/`mfsk-ffi-ft8`
  dependency graph). Verified both regenerated headers now carry
  full, byte-identical (differing only in indentation style per
  crate's `cbindgen.toml`) definitions for every shared type.

  Updated every call site: `mfsk-ffi/tests/{q65_ffi,wsjt_ffi}.rs`,
  `mfsk-ffi-ft8/tests/streaming.rs`, `mfsk-ffi/examples/cpp_smoke`
  (built + run, including the `RUN_FST4_ROUNDTRIP=1` gated path),
  `mfsk-ffi-ft8/tests/c_smoke/{smoke.c,tx_rx_round_trip.c}` (built +
  run against a real WAV and a synthesised round-trip),
  `mfsk-ffi-ft8/examples/streaming_recipe.c` (compile-checked; it's a
  documentation artefact with no `main`), `mfsk-ffi/examples/kotlin_jni/
  native/mfsk_jni.c` + its `README.md`, and `embedded-poc/embedded-shared/
  src/apps/compute_bench.rs`'s FFI smoke path (path-dependency outside
  the workspace, not compile-verified here — no `+esp` toolchain in
  this environment; run `cargo check` there before flashing).
  `docs/reference/{LIBRARY,EMBEDDED}.md` and their `.ja.md`
  counterparts updated to match, including a pre-existing stale
  `NULL, NULL, // Goertzel` leftover in `EMBEDDED.md`'s streaming
  recipe (from the #162 BASIS-scratch removal, unrelated to this
  issue but caught in the same doc pass).

  Verified clean across the `mfsk-core` feature matrix (unaffected by
  this issue, reconfirmed), clippy `--workspace --all-targets`, full
  test suite (`mfsk-ffi`/`mfsk-ffi-ft8`/`mfsk-ffi-abi`), `cargo doc`,
  and the C/C++ smoke drivers above.
- **Breaking**: unified the four legacy protocols' decode-result type
  names onto one naming convention (issue #206, pre-0.8.0 public-API
  review, decided 2026-07-27) — `WsprDecode`→`WsprResult`
  (`wspr::decode`), `Jt9Decode`→`Jt9Result` (`jt9`),
  `Jt65Decode`→`Jt65Result` (`jt65`), `Q65Decode`→`Q65Result`
  (`q65::rx`), matching `engine::pipeline::DecodeResult`'s `*Result`
  suffix instead of the WSJT77-family's leftover `*Decode`.
  Naming-convention-only: `decode_at`/`decode_scan`/`decode_slot`
  entry-point verbs are unchanged, and all four types keep their
  existing protocol-specific shapes — WSPR/JT9/JT65 carry an
  already-unpacked human message (`WsprMessage`/`Jt72Message`) plus
  mode-specific timing metadata via their own `decode_at`/`decode_scan`
  engines, which (unlike FT8/FT4/FST4) never route through
  `engine::pipeline`; `Q65Result` carries an unpacked `String` message
  rather than raw FEC info bits. A full structural merge onto
  `engine::pipeline::DecodeResult` would require first porting
  WSPR/JT9/JT65 onto the generic pipeline engine — the same scale of
  work as issue #192's FT8 proposal, times three protocols — and was
  explicitly scoped out of the 0.8.0 window as too large/risky for the
  time remaining before the release cut; tracked as future work rather
  than bundled here.

### Fixed

- **JT65 total decode failure, root-caused and fixed** (#24). Every
  `jt65sim`-generated (or otherwise WSJT-X-compatible) JT65 signal
  failed to decode regardless of SNR, even when `search::coarse_search`
  landed candidates within ~1 Hz / a few symbols of ground truth.
  Root cause: `jt65::interleave::interleave`/`deinterleave` (the 7×9
  transpose WSJT-X's `interleave63.f90` implements) had their
  permutations swapped relative to WSJT-X's TX/RX convention
  (`gen65.f90`/`jt65sim.f90` call `interleave63(sent, idir=1)` at TX;
  `extract.f90` calls `interleave63(mrsym, idir=-1)` at RX). The swap
  was internally self-consistent — TX and RX remained exact mutual
  inverses of each other — so every self-roundtrip test passed while
  real/independent-reference signals decoded to garbage RS codewords.
  Structurally identical to the JT9 encoder bug (#19): a decode path
  only ever cross-checked against this crate's own encoder, never an
  independent reference.
- New `tests/jt65_sweep.rs` (`#[ignore]`d, mirrors `ft8_sweep.rs`) +
  `scripts/gen_jt65_sweep_wavs.sh` build a `jt65sim`-generated AWGN
  SNR-sweep corpus and characterise the post-fix recall curve: 100%
  recall down to 0 dB, 50% around -14 dB, near-zero below -19 dB (all
  in `jt65sim`'s 2500 Hz reference bandwidth convention).
- Found and documented, but did **not** port in this fix: WSJT-X's
  own hard-decision-only path (`jt9 -6`, no `kvasd`) holds ~100%
  recall down to -22 dB on the same corpus — a further ~7-8 dB
  sensitivity gap. Root cause traced to `lib/ftrsd/ftrsdap.c`, a
  stochastic Chase decoder (randomized multi-trial soft-symbol
  erasure-pattern search around Berlekamp-Massey RS) that WSJT-X runs
  even without `kvasd` — a materially different (and more involved)
  algorithm than this crate's single-pass confidence-ordered erasure
  decode (`decode_at_with_erasures`). Tracked as a follow-up (#169);
  see the sweep test's doc comment for the full provenance.
- **Q65 AWGN sensitivity gap, root-caused and substantially
  narrowed** (#171). The initial sweep found a real, reproducible
  ~2-3 dB gap vs. WSJT-X's own plain-BP decode (`jt9 -3 -p 30 -b A`)
  for Q65-30A specifically (50% recall crossing ~-24 dB vs. WSJT-X's
  ~-26 to -27 dB), confirmed directly on a single failing file (not a
  batch-script artifact). The entire FEC/BP stack was verified
  byte-for-byte correct against `WSJT-X/lib/qra/q65/{qracodes.c,
  npfwht.c,pdmath.c}` first (all 10 code tables, the WHT, and every
  `pdmath` primitive — diffed programmatically, zero discrepancies),
  which pointed the search upstream: `coarse_search_for`'s reported
  best `(start_sample, freq_hz)` is measurably imprecise at low SNR
  — off by up to ~1/5 of a symbol period — and neither
  `decode_scan_for` nor `decode_at_for` ever refined that alignment
  before attempting decode. WSJT-X's `q65_loops` never trusts its own
  coarse alignment either — it always runs a local fine-timing
  (`idt`) retry loop before the real decode attempt. Added
  `decode_at_with_fine_timing_for` (tries the reported alignment
  first, then a symmetric ±3-step retry grid in units of `nsps/16`)
  to `decode_scan_inner`, the shared implementation behind every
  scan-level Q65 entry point. Full 990-file sweep re-run: all six
  wired sub-modes improved by roughly 1-2 dB at their threshold
  region (e.g. Q65-60A 20%→100% at -27 dB; Q65-30A 40%→93% at -24 dB,
  0%→33% at -25 dB).
- **Remaining residual gap traced to a comparison-methodology
  difference, not a further implementation bug.** Even after the
  fine-timing fix, a gap persisted at the deep end (e.g. Q65-30A ~0%
  at -26 dB vs. WSJT-X's ~40%). Traced this to WSJT-X's Q65 decode
  chain always having access to the free "CQ ??? ???" AP hypothesis
  (`aptype=1` in `extract.f90`'s AP table — needs no user-supplied
  callsign), so *every* real `jt9` decode attempt implicitly gets
  some AP-list benefit on CQ-calling signals, whether or not the user
  configured `-c`/`-x`. `decode_scan_for` (this crate's genuinely
  blind baseline) has no equivalent for-free hypothesis by design —
  the four decoder strategies stay deliberately separate
  (`docs/reference/LIBRARY.md` §3). Re-measured with
  `decode_scan_with_ap_for` + a `"CQ"` hint (now also reported by
  `tests/q65_sim_sweep.rs` as a second column) and it closes the gap
  almost exactly across all six sub-modes: Q65-30A -26 dB 0%→40%
  (matches WSJT-X's reported 40%), -25 dB 33%→93% (WSJT-X: 87%),
  -24 dB 93%→100% (WSJT-X: 100%); Q65-60A -29 dB 7%→53%, -28 dB
  47%→93%; Q65-60B/C/D/E show the same ~40-50 point jump at their
  respective thresholds. **Usage note, not a code change**:
  applications wanting WSJT-X-equivalent behavior for CQ traffic
  should call the AP-hinted path with at least a `"CQ"` hint rather
  than the plain one — matching what WSJT-X's own decoder always
  does internally. Issue #171 left open with this as the closing
  analysis (no further action expected; re-open if a real remaining
  gap is found with matched AP context on both sides).
- **`DecodeRequest<Ft8>::strictness()` was a documented no-op for
  FT8's non-AP decode path** (#221, found while benchmarking WebFT8's
  decode presets). `.strictness(Deep)` produced identical recall and
  wall-clock to `.strictness(Strict)` on every scenario tested — FT8's
  BP staircase and OSD fallback used hardcoded `36`
  (`OSD_HARDERRORS_MAX`/`WSJTX_NHARDERRORS_MAX`, WSJT-X's own
  `ft8b.f90:422` ceiling) unconditionally, dead since #188 removed the
  code that used to consume a strictness-tiered version;
  `DecodeStrictness::ap_max_errors` (the AP loop's own gate) was the
  only method FT8 ever actually called. New
  `DecodeStrictness::ft8_nharderrors_max()` wires all four BP-variant
  acceptance checks and the OSD dispatch to the per-request
  strictness: `Normal = 36` (unchanged default, zero regression —
  full FT8 golden/JTDX/full-parity/depth-ladder regression suite
  green), `Strict = 22` (real prior art reused from the issue #72
  investigation, not a fresh guess), `Deep = 40` (mfsk-core-original,
  exceeds WSJT-X's own ceiling, not yet swept against a fading
  corpus). FST4 remains unaffected by any `DecodeStrictness` method by
  design (#146).

### Added

- **Q65-15A**: a new sub-mode ZST (`Q65a15`, 15 s T/R period, ×1 tone
  spacing = 6.667 Hz), the fastest wired Q65 mode — one line via the
  existing `q65_submode!` macro, following the established
  15/30/60-s-period axis `q65params.f90` already defines. Wired
  through the FFI (`MfskQ65SubMode::A15`, appended after `E60` rather
  than inserted before `A30` to keep the `#[repr(C)]` enum's existing
  discriminant values stable) and the `PROTOCOLS` registry
  (`"Q65-15A"`). No real off-air recording exists for this period in
  WSJT-X's sample tree (same situation as Q65-60C/60E already
  documented), so no golden-WAV test is possible — covered instead by
  a dedicated `tests/q65_a15_roundtrip.rs` (synth + aligned/offset
  scan recovery) and folded into the `q65sim`-based AWGN sweep below
  (50% crossing ≈ -21 dB, matching `q65params.f90`'s analytical
  formula for the 15 s period).
- **Q65-120D, Q65-120E, Q65-300A**: three longer-period sub-modes
  (`Q65d120`/`Q65e120`/`Q65a300`), chosen because WSJT-X's own user
  guide (`doc/user_guide/en/protocols.adoc`) and sample tree document
  real, specific use cases for exactly these three: Q65-120D (10 GHz
  rainscatter/troposcatter, backed by 14 files in WSJT-X's own
  `UnitTests.txt` regression corpus), Q65-120E (6 m ionoscatter),
  and Q65-300A (optical/laser scatter — the deepest wired Q65
  sub-mode, ~-34 dB AWGN threshold, matching the published table
  value almost exactly). Unlike Q65-15A, these three **do** have
  golden-WAV tests: `tests/q65_wsjtx_samples.rs` gained
  `rainscatter_10ghz_120d_decodes_with_fading_metric`,
  `ionoscatter_6m_120e_decodes_with_fading_metric`, and
  `optical_scatter_300a_decodes_with_fading_metric` — golden messages
  ("VK3WE VK7MO QE37", "KB7IJ N0AN 73", "VK7MO VK7PD QE38")
  independently confirmed via `jt9 -3 -p {120,300} -b {D,E,A}` first.
  All three need the fast-fading metric to decode (plain BP fails,
  same shape as the existing Q65-60D EME test) — for Q65-300A this
  holds even though it's stable-path scatter rather than classic
  Doppler-spread EME, suggesting the fading metric's robustness helps
  generally at threshold-adjacent SNR, not only under true multipath.
  Now **10 wired Q65 sub-modes total**; docs (`LIBRARY.md`/`.ja.md`),
  `tests/protocol_invariants.rs`, FFI (`MfskQ65SubMode::{D120,E120,A300}`,
  discriminants 7-9), and the `q65sim` AWGN sweep all updated to match
  (the 120/300 s configs use 5 trials instead of 15 — their WAVs are
  proportionally larger and the #171 fine-timing retry multiplies
  decode cost further).
- **Direct WSJT-X cross-check for Q65-120D/120E/300A**: ran `jt9 -3
  -p {120,300} -b {D,E,A}` (no `-c`/`-x`) over the identical 165-file
  sweep corpus per sub-mode used above. Result: **no regression, and
  two sub-modes exceed WSJT-X's own plain decode**. Q65-300A's curve
  is statistically identical to `jt9`'s at every tested SNR point
  (both cross 50% at ≈-35 dB). Q65-120D and Q65-120E's `decode_scan_for`
  50% crossings (≈-30.7 dB, ≈-31.0 dB) are **2.5-3.4 dB better** than
  `jt9 -3`'s own plain-decode crossings (≈-28.2 dB, ≈-27.6 dB) —
  consistent across 4+ SNR points each, not sampling noise. Likely
  explanation (not fully confirmed): the #171 fine-timing retry tries
  a fixed ±3-step grid per coarse candidate regardless of T/R period,
  which may end up relatively more thorough than WSJT-X's own
  `q65_loops.f90` `idt`/`idf` retry granularity at these slower-baud,
  longer-period sub-modes specifically.
- New `scripts/gen_q65_sweep_wavs.sh` + `tests/q65_sim_sweep.rs`
  (`#[ignore]`d): a `q65sim`-generated AWGN SNR sweep covering every
  sub-mode ZST this crate actually wires (`Q65a15`, `Q65a30`,
  `Q65a60`, `Q65b60`, `Q65c60`, `Q65d60`, `Q65e60`, `Q65d120`,
  `Q65e120`, `Q65a300` — WSJT-X's Q65 also has other (period,
  sub-mode) combinations this crate doesn't implement, so the sweep
  intentionally covers only what's shipped). `q65sim` has a real
  CMakeLists.txt target (unlike `jt9sim`), so no new build script was
  needed — build via `cmake --build ~/wsjtx-build --target q65sim`.
  Below period=30 s, `q65sim` uses a completely different filename
  format (`000000_MMSS.wav`, not the sequential index used at ≥30 s)
  — `gen_q65_sweep_wavs.sh` special-cases this for Q65-15A.
- **New `tests/q65_wsjtx_samples.rs::tropo_1296_60b_decodes_via_averaging`**:
  Q65-60B was the only wired sub-mode with a real off-air recording
  already vendored (`WSJT-X/samples/Q65/60B_1296_Troposcatter/`) but
  no corresponding test. Single-slot decode fails on this dataset (as
  expected, same as the existing 10 GHz EME test's plain path); the
  multi-period EMA-on-spectrogram path (`decode_multi_period_for`,
  same mechanism the existing ionoscatter test uses) recovers the
  golden message "VK7MO VK7PD QE38" cleanly.
- Q65-60C and Q65-60E have no real off-air recording anywhere in
  WSJT-X's sample tree, so no golden-WAV test is possible for those
  two sub-modes (same situation JT65 was already in before this
  session — no real recording exists to validate against).
- New `scripts/build_jt9sim.sh` + `scripts/gen_jt9_sweep_wavs.sh` +
  `tests/jt9_sweep.rs` (`#[ignore]`d, mirrors `tests/jt65_sweep.rs`):
  a `jt9sim`-generated AWGN SNR sweep for JT9. Unlike `ft8sim`/
  `ft4sim`/`fst4sim`/`jt65sim`, `jt9sim` has no CMakeLists.txt target
  in WSJT-X at all — `build_jt9sim.sh` assembles its actual dependency
  closure (`gen9` → `packjt`/`entail`/`encode232`/`interleave9`/
  `graycode`, plus `jt9fano`/`fano232` for jt9sim's own internal
  self-verify step) as a standalone binary from source.
- Result: `decode_scan_default` holds **100% recall down to -24 dB**,
  crossing 50% around -26 dB — closely tracking WSJT-X's own `jt9 -9`
  on the identical 300-file corpus (100% to -25 dB, 80% at -26 dB; the
  per-cell differences are within 20-trial sampling noise at the
  steep part of the curve, not a systematic gap). Confirms JT9 has
  **no JT65-style hidden sensitivity gap** — the three remaining
  misses in the real-recording golden test
  (`tests/jt9_wsjtx_samples.rs`) are congestion/wrong-codeword-lock
  issues specific to that busy recording, not a general AWGN
  weakness.
- Also re-confirms the #19 encoder fix (`pack_grid4_plain`/
  `unpack_grid`) against a second, independently-built reference
  encoder: a fresh `jt9sim` signal ("CQ JL1NIE PM95" @ 1400 Hz)
  decodes cleanly, matching WSJT-X's own `jt9 -9` output exactly.

### Fixed

- **Q65 AWGN sensitivity gap, root-caused and substantially
  narrowed** (#171). The initial sweep found a real, reproducible
  ~2-3 dB gap vs. WSJT-X's own plain-BP decode (`jt9 -3 -p 30 -b A`)
  for Q65-30A specifically (50% recall crossing ~-24 dB vs. WSJT-X's
  ~-26 to -27 dB), confirmed directly on a single failing file (not a
  batch-script artifact). The entire FEC/BP stack was verified
  byte-for-byte correct against `WSJT-X/lib/qra/q65/{qracodes.c,
  npfwht.c,pdmath.c}` first (all 10 code tables, the WHT, and every
  `pdmath` primitive — diffed programmatically, zero discrepancies),
  which pointed the search upstream: `coarse_search_for`'s reported
  best `(start_sample, freq_hz)` is measurably imprecise at low SNR
  — off by up to ~1/5 of a symbol period — and neither
  `decode_scan_for` nor `decode_at_for` ever refined that alignment
  before attempting decode. WSJT-X's `q65_loops` never trusts its own
  coarse alignment either — it always runs a local fine-timing
  (`idt`) retry loop before the real decode attempt. Added
  `decode_at_with_fine_timing_for` (tries the reported alignment
  first, then a symmetric ±3-step retry grid in units of `nsps/16`)
  to `decode_scan_inner`, the shared implementation behind every
  scan-level Q65 entry point. Full 990-file sweep re-run: all six
  wired sub-modes improved by roughly 1-2 dB at their threshold
  region (e.g. Q65-60A 20%→100% at -27 dB; Q65-30A 40%→93% at -24 dB,
  0%→33% at -25 dB).
- **Remaining residual gap traced to a comparison-methodology
  difference, not a further implementation bug.** Even after the
  fine-timing fix, a gap persisted at the deep end (e.g. Q65-30A ~0%
  at -26 dB vs. WSJT-X's ~40%). Traced this to WSJT-X's Q65 decode
  chain always having access to the free "CQ ??? ???" AP hypothesis
  (`aptype=1` in `extract.f90`'s AP table — needs no user-supplied
  callsign), so *every* real `jt9` decode attempt implicitly gets
  some AP-list benefit on CQ-calling signals, whether or not the user
  configured `-c`/`-x`. `decode_scan_for` (this crate's genuinely
  blind baseline) has no equivalent for-free hypothesis by design —
  the four decoder strategies stay deliberately separate
  (`docs/reference/LIBRARY.md` §3). Re-measured with
  `decode_scan_with_ap_for` + a `"CQ"` hint (now also reported by
  `tests/q65_sim_sweep.rs` as a second column) and it closes the gap
  almost exactly across all six sub-modes: Q65-30A -26 dB 0%→40%
  (matches WSJT-X's reported 40%), -25 dB 33%→93% (WSJT-X: 87%),
  -24 dB 93%→100% (WSJT-X: 100%); Q65-60A -29 dB 7%→53%, -28 dB
  47%→93%; Q65-60B/C/D/E show the same ~40-50 point jump at their
  respective thresholds. **Usage note, not a code change**:
  applications wanting WSJT-X-equivalent behavior for CQ traffic
  should call the AP-hinted path with at least a `"CQ"` hint rather
  than the plain one — matching what WSJT-X's own decoder always
  does internally. Issue #171 left open with this as the closing
  analysis (no further action expected; re-open if a real remaining
  gap is found with matched AP context on both sides).
- **WSPR AWGN SNR sweep test infra** (`tests/wspr_sweep.rs`,
  `scripts/gen_wspr_sweep_wavs.sh`), closing the one gap left where
  WSJT-X ships a simulator but this crate had no simulator-driven
  objective benchmark — every other supported protocol
  (FT8/FT4/JT9/JT65/Q65/MSK144/FST4) already had a `*sim`-based AWGN
  sweep; WSPR's only prior validation was a single real-world WAV's
  fixed-SNR golden recall (`wspr_wsjtx_samples.rs`). WSJT-X's
  CMakeLists.txt only wires the C `wsprsim` (`lib/wsprd/wsprsim.c`),
  which writes `.c2` complex-baseband files for `wsprd`, not WAV audio
  this crate's decode path consumes — the WAV-capable simulator is a
  second, orphaned Fortran program (`lib/wsprd/wsprsimf.f90`, no
  CMake target, same situation as `jt9sim`) that this crate now
  builds standalone via `scripts/build_wsprsim.sh` (picking out its
  actual dependency closure, plus a local no-op `watterson` stub to
  avoid pulling in FFTW for a code path — the `.c2` branch — this
  build never exercises). 13-point AWGN sweep, 20 trials/point:
  100% recall from 0 dB down to -27 dB, 95% at -28/-29 dB, 40% at
  -30 dB, 0% at -31 dB and below — consistent with WSJT-X's published
  WSPR sensitivity floor.

### Tests

- **`mfsk-ffi` Rust-level ABI test coverage for the non-Q65
  protocols** (`mfsk-ffi/tests/wsjt_ffi.rs`). Previously only Q65 had
  a `cargo test`-driven FFI test (`q65_ffi.rs`); FT8/FT4/WSPR/JT9/
  JT65/FST4-60A were exercised solely by the C++ smoke driver
  (`examples/cpp_smoke`). Added encode/decode round-trips for all
  six (mirroring the C++ driver's known-good test vectors), an
  `mfsk_decode_i16` case (the f32 path was the only one covered
  before), and NULL/bad-input negative-path tests
  (`mfsk_decode_f32` with a null decoder/samples pointer,
  `mfsk_encode_ft8` with an unpackable callsign, freeing null
  pointers). Also fixed a CI gap found while adding this: the `ffi`
  job built `mfsk-ffi` and ran the C++ driver but never ran
  `cargo test -p mfsk-ffi` at all, so `q65_ffi.rs` itself was not
  actually executing in CI; `.github/workflows/ci.yml` now runs it
  (FST4's slow slot decode stays behind the same
  `RUN_FST4_ROUNDTRIP` gate the C++ driver already used).
- **`mfsk-ffi-ft8` wired into the same `ffi` CI job** — previously
  had zero CI coverage of any kind (not even a build step), despite
  shipping a `tests/streaming.rs` suite (7 tests, ring-buffer /
  resample / chunk-boundary coverage for the `mfsk_ft8_stream_*`
  API) that only ever ran locally. Added `cargo build`/`cargo test -p
  mfsk-ffi-ft8` steps using the crate's default `host` feature only
  — the `embedded-fixed-point` feature builds `no_std` and has no
  `cargo test` harness to run on a CI runner, so the
  xtensa-esp32/esp32s3-espidf cross-compiles stay build-only-verified
  in `release.yml` as before (they can't be executed without real
  hardware anyway).

### Changed

- **`mfsk-ffi`'s version unstuck from a stale `0.1.0`** — it had
  never been bumped since the crate's initial commit despite
  gaining Q65 sub-modes and other features release after release.
  Introduced `[workspace.package].version` in the root `Cargo.toml`
  as the single source of truth; `mfsk-core`, `mfsk-ffi`, and
  `mfsk-ffi-ft8` now all declare `version.workspace = true` instead
  of a hand-copied literal, so a release version bump is one edit
  instead of three manually-synced ones (the exact class of bug that
  let `mfsk-ffi` drift). `release.yml`'s tag-vs-Cargo.toml gate
  (`verify-tag` job) updated to read the workspace version from the
  root `Cargo.toml` instead of `mfsk-core/Cargo.toml` directly, since
  the latter no longer carries a literal version string.
- **`mfsk-ffi` release-build policy decided: desktop/mobile stays a
  single all-decoder library, and it's now distributed the same way
  as `mfsk-ffi-ft8`.** Confirmed design split — `mfsk-ffi-ft8` stays
  the embedded-only FT8 slice, `mfsk-ffi` stays the desktop/mobile
  superset covering all seven WSJT modes (Kotlin/Android's binary
  footprint isn't a concern here, unlike the embedded no_std targets;
  Android release binaries deliberately deferred — see below) — and
  closed the distribution gap this implied for the desktop target:
  `release.yml` gained `build-ffi-desktop-host` (linux-x86_64:
  `libmfsk.{so,a}` + `mfsk.h`), gating `publish` alongside the
  existing `mfsk-ffi-ft8` build jobs so a broken build blocks the
  release instead of leaving it half-published. `mfsk-ffi/README.md`,
  root `README.md`, and `docs/reference/LIBRARY.md` §8-9 updated to
  describe the new prebuilt-binary distribution instead of "clone and
  build". Android (arm64 via NDK cross-build) was drafted in the same
  pass — a working `build-ffi-desktop-android` job was written and
  locally verified for target/toolchain-name correctness — but
  deliberately left out of this release-build pass; Kotlin/Android
  consumers keep building `mfsk-ffi` locally with cargo-ndk per
  `mfsk-ffi/examples/kotlin_jni/README.md` for now.
- **`examples/kotlin_jni/` scaffold renamed to match the
  `wsjt-ffi` → `mfsk-ffi` crate rename it had missed** — found while
  drafting the Android release build above. `Wsjt.kt` /
  `wsjt_jni.c` / package `io.github.rsft8n` still referenced the
  pre-rename crate name (`cargo build -p wsjt-ffi`, `libwsjt.so`,
  `WsjtMessageList`, …), none of which exist anymore; the scaffold's
  own documented build commands would not have run. Renamed to
  `Mfsk.kt` / `mfsk_jni.c` / package `io.github.mfskcore` throughout,
  matching what `docs/reference/LIBRARY.md` §9 already (aspirationally)
  described. Also expanded `Mfsk.Protocol` from `FT8`/`FT4` only to
  all seven `MfskProtocol` values, matching `mfsk-ffi`'s actual
  all-decoder scope.

### Fixed

- **FT8 `decode_block` multi-pass decode ~10.5× faster on busy-band
  recordings, WSJT-X-faithful subtract algorithm.** A decode-speed
  investigation (prompted by `qso3_busy.wav` benchmarking 4.73 s —
  suspiciously slow for a 15 s slot on host hardware) profiled
  `decode_block`'s three-pass successive-interference-cancellation
  loop phase by phase. Coarse-sync, fine-refine, and per-candidate
  BP/OSD were all sub-5 ms each; the entire cost (~310 ms per
  accepted decode, ~13 calls on this recording) was
  `core::dsp::subtract::subtract_tones_lpf` — a naive O(candidates ×
  NFRAME × lpf_half) direct time-domain convolution for the
  channel-tracking LPF (≈608 M MACs/call at FT8's `lpf_half=2000`).
  Checked WSJT-X's own `lib/ft8/subtractft8.f90` /
  `lib/ft4/subtractft4.f90`: both use a *cached* filter-response FFT
  (built once, `first`/`save` in the Fortran) plus one forward +
  inverse FFT of the full slot per call — O(N log N), not O(N×M).
  Ported that algorithm exactly (including FT8's `endcorrection`
  edge-response boost, which `subtractft4.f90` omits and the port
  now matches per-protocol via a new `endcorrection: bool` parameter
  on `subtract_tones_lpf`) behind the existing `fft-rustfft` (host)
  feature; the direct-convolution version is kept as the `no_std`
  fallback, unchanged. No embedded target is affected either way —
  `decode_block`'s embedded (`not(fft-rustfft)`) variant of
  `decode_block_multipass` has no subtract loop at all (single-pass,
  no SIC), confirmed by grepping `embedded-shared`'s actual decode
  pipeline before assuming otherwise.
  `ft8_qso3_apoff_recall.rs`'s `qso3_apoff_meets_wsjtx_golden_floor`:
  **4.73 s → 0.45 s**, byte-identical recall (7/8 golden, 7 phantom,
  14 total). Also caught and fixed a second, independent staleness
  bug found during the same investigation: this test's comment
  claimed to match "the Core2 / S3 production path" while passing
  `DecodeDepth::BpAll`; the actual embedded ship config is
  `DecodeDepth::BpVariantsAd` (introduced later, skips the two most
  expensive BP LLR variants for failed candidates) — updated the
  test to match, verified identical recall either way. FT4's golden
  test is on a different (non-LPF, constant-amplitude) subtract path
  in `core::pipeline` and was unaffected by this change.
- **FT8 `decode_block` a further ~3.6× faster on busy-band recordings
  — two FFT-cache wiring gaps closed.** Follow-up profiling of
  `decode_block_multipass` (post the 10.5× subtract fix above) found
  its cost had moved, not disappeared: (1) `refine_candidates` and
  the per-candidate `process_candidates_tuned_with_ap[_ref]` calls
  were all passing `fft_cache: None`, so each of up to ~45
  candidates/slot re-ran the 192 k-point forward FFT from scratch
  even though the sibling `fine_refine_pass1` stage in the same pass
  already built and reused exactly this cache — a wiring gap, not a
  missing algorithm. Threaded the existing
  `build_fft_cache`/`downsample_cached` helpers through
  `refine_candidates`, `process_candidates_tuned_with_ap[_ref]`, and
  `auto_ap_strategy::run`, rebuilding the cache lazily only when the
  WSJT-X-mandated sequential subtract actually mutates the working
  buffer (not eagerly every pass) — alone dropped
  `qso3_apoff_meets_wsjtx_golden_floor` 0.43 s → ~0.12–0.22 s. (2)
  `subtract_tones_lpf_fft`'s filter-response FFT was already cached
  (`cached_window_fft`, from the fix above), but its forward/inverse
  `FftPlanner` *plans* for `nfft = 180 000` were rebuilt on every
  call — measured at ~2.8 ms/call for plan construction vs ~0.7 ms
  for the transform itself, i.e. plan-rebuild cost ~4× the FFT it
  was gating. Cached the plans in an `nfft`-keyed `OnceLock`, the
  same pattern `fill_symbol_spectra.rs`'s `SYMBOL_FFT_32` already
  used for the per-symbol 32-pt FFT. Combined: **0.43 s → 0.12 s
  (~3.6×)**, byte-identical recall (7/8 golden, 7 phantom, 14 total
  on `qso3_apoff`; 5/6 JTDX AP-on extras unchanged on `qso3_apon`).
  Both fixes are cache wiring, not new algorithms, so risk is low;
  `fft-rustfft` is `std`-gated but thread-free, so both caches also
  apply under `wasm32-unknown-unknown` (build-verified). No embedded
  impact either way, since `decode_block_multipass`'s
  `not(fft-rustfft)` variant has no subtract loop (single-pass, no
  SIC) and never calls these paths.
- **FT8 CCIR moderate/poor sweep sensitivity gap closed, root-caused
  as a comparison-methodology gap, not a numerical deficiency**
  (#190). The ~0.6-0.7 dB gap vs real `jt9 -8 -d3` on the
  `ft8sim`-driven CCIR moderate/poor 50%-crossing sweep traced (via a
  locally-instrumented `jt9` build, `ISSUE190_PROBE` prints following
  the `DL8YHR_PROBE` precedent from #180) to `jt9`'s CLI hardcoding
  `lft8apon=.true.` (`lib/jt9.f90:302`, independent of the GUI's own
  default-off `FT8AP` setting) — every `jt9 -8 -d3` invocation used
  throughout this project's WSJT-X-parity work has implicitly
  included a free "CQ ??? ???" AP hypothesis pass (WSJT-X iaptype-1,
  needs no operator-supplied mycall/hiscall), and the sweep corpus's
  message (`CQ JL1NIE PM95`) is exactly the class that pass targets.
  Traced nsync (14/16/18) matching jt9 almost exactly on the 3
  `ccir_moderate_m19_{01,05,14}.wav` trials jt9 wins and mfsk-core
  lost — ruling out the fine-sync/coherent-combining lead
  `FT8_BENCHMARK.md` section 10 had flagged — and found jt9's own
  blind `ipass=1..4` also failed identically on all 3, succeeding
  only via `ipass=5` (`iaptype=1`). mfsk-core's
  `process_one_candidate_inner` already had the matching "Pass 12:
  blind-CQ" logic, but it was gated behind `ap_hint.is_some()`, so
  plain `decode_frame` (no AP hint) never reached it. Not a revival of
  the auto-AP mechanism removed earlier in this release — that was
  iaptype-2 self-seeding from same-slot callsigns, an mfsk-core
  original with no WSJT-X counterpart; this is iaptype-1, a faithful
  port. Fix: pass 12 now runs whenever the blind BP/OSD staircase
  fails **and** `sync_quality (nsync) ≥ BLIND_CQ_MIN_NSYNC (12)`,
  independent of `ap_hint`. The nsync floor is a cost gate added after
  measuring an un-gated first version: on `qso3_busy.wav`'s multipass
  staged-SIC path it sent 188 candidates through this pass (140 at
  nsync 7-9, none producing a decode there — real recoveries needed
  nsync 14-18), pushing wall-clock 0.7 s → 1.5 s, slower than real
  jt9's own ~1.15 s on the same file. Gating at nsync≥12 cuts that to
  34 candidates and ~0.85 s (faster than jt9 again), zero recall
  change either way. Re-measured 50%-crossings (same 780-file corpus,
  gated version): AWGN -21.4→-21.6 dB, CCIR good -20.8→-21.1 dB, CCIR
  moderate -18.9→-20.0 dB, CCIR poor -19.0→-19.7 dB — moderate now
  ahead of real `jt9 -8 -d3`, poor at parity (was behind on both). No
  recall regression on any FT8 golden test (WSJT-X AP-off 7/8, JTDX
  18/18, full-parity 8/8, staged-SIC 18/18, AP-on JTDX-extras 6/6);
  `qso3_busy.wav` single-pass `DecodeDepth::FULL` wall-clock ~139-141
  ms → ~165-175 ms (~6-7× faster than real `jt9 -8 -d3`'s ~1.1-1.2 s).
  See `FT8_BENCHMARK.md` section 11 for the full trace and cost
  investigation.
- **`engine::dsp::downsample::downsample_cached` rebuilt its inverse-FFT
  plan (twiddle table) from scratch on every call instead of reusing one
  across a session** (#211), found while profiling the wasm `+simd128`
  benchmark harness (#208, Stage C — `node --prof` with real function
  symbols). Called once per FT8 candidate (~90×/slot), it constructed a
  fresh `default_planner()` and called `plan_inverse(cfg.fft2_size)`
  every time, even though `fft2_size` never varies within a decode
  session — discarding `rustfft::FftPlanner`'s own per-size cache each
  call and rebuilding the twiddle table via scalar `sin`/`cos`/`rem_pio2`.
  Same anti-pattern already fixed twice nearby (`SYMBOL_FFT_32` in
  `fill_symbol_spectra.rs`, `subtract_tones_lpf_fft`'s filter-response
  plan above) — `default_planner()`'s own doc comment already says to
  "reuse the same instance across all decodes in a session so rustfft's
  twiddle cache hits," this call site just didn't. Fixed with a
  `std`-gated `thread_local!` planner, reused across calls on the same
  thread; `no_std`/`fft-extern` (embedded) callers are unaffected — the
  only hot-loop caller (`fill_symbol_spectra_via_cd0`) is itself
  `fft-rustfft`-gated. Measured impact: ~12.5% of total wasm decode
  wall-clock (`node --prof` flat profile, `qso3_busy.wav`), bigger than
  any dense-kernel SIMD target found in the same profiling pass; not
  wasm-specific, the same redundant work happens on every target. Golden
  recall unchanged: FT8 full-parity 8/8, AP-off 7/8 (7 phantom, 14
  total), JTDX 18/18 (1 extra) — byte-identical to pre-fix.
- **`engine::llr::fill_bmet_for_nsym`'s max-reduction loop rewritten to
  actually vectorize under `wasm32 +simd128`** (#208 Stage D). The
  per-element `if (i >> bit_sel) & 1 == 1 { max_one[..] = v } else {
  max_zero[..] = v }` blocked LLVM's vectorizer two ways at once:
  branchy per-lane control flow, and an `i`-outer/`bit_sel`-inner loop
  nest that would need outer-loop vectorization LLVM doesn't attempt
  here. Making the branch branchless (feed `f32::NEG_INFINITY` to
  whichever accumulator the bit doesn't select, then an unconditional
  `max()`) alone changed nothing — confirmed empirically via
  `wasm-objdump`, the compiled region's `v128` count was unchanged.
  Swapping the loop order (`bit_sel` outer, `i` inner — a flat
  reduction over `s2` with a loop-invariant `bit_sel`) combined with
  the branchless rewrite is what actually unblocked it: 40 → 86 `v128`
  ops in the function's compiled region. This was the a-priori top
  Part-2 target from #208's dense-kernel survey, but profiling (Stage
  C) found its real ceiling is small (~2.9% of total decode time,
  `qso3_busy.wav`, wasm), well below what a `node --prof` flat profile
  found in an unrelated FFT-plan-caching bug (#211, fixed separately)
  — wall-clock impact here is within run-to-run measurement noise even
  though the vectorization itself is real and verified.
  Byte-identical recall on all three FT8 golden regressions
  (full-parity 8/8, AP-off 7/8/7-phantom/14-total, JTDX 18/18/1-extra)
  and the full `--features full` test suite.
- **`embedded-poc` didn't compile against the real `+esp` Xtensa
  toolchain** (#215) — #194's `DecodeResult.message77` field→method
  change wasn't propagated there, since `embedded-poc` is
  workspace-excluded and never built by host CI. 5 call sites still
  used `&r.message77` as a field: `embedded-shared/src/apps/{compute_bench,rx_wavsim}.rs`
  and each app's `decode_pipeline.rs`
  (`m5stack-s3-app`/`m5stack-core2-app`/`m5stack-cores3-app`). Found by
  installing `espup` fresh and running `cargo check` across all four
  `embedded-poc` crates — closes the "embedded-poc +esp check still
  owed" item from #206. All four now compile clean.

Older releases are archived out of this file to keep it skimmable:
0.6.0 – 0.7.4 in
[`docs/historical/CHANGELOG-0.6-0.7.md`](docs/historical/CHANGELOG-0.6-0.7.md),
0.1.0 – 0.5.12 in
[`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md).
