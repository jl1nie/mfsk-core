# JTTY on the CoreS3 app — design (draft 2, 2026-09-27)

Receive first, as a fifth boot mode of `m5stack-cores3-app` beside FT8, FT4, FST4 and WSPR (#499). Transmit is a later
phase (§8). What the decoder costs and why it is configured as it is: `JTTY_EMBEDDED_BUDGET.md` §11–§14. Draft 1 was
reviewed against the code the same day; the corrections are folded in and listed in §11.

## 1. What JTTY asks of the app, compared with the slotted modes

| | FT8 / FT4 / FST4 / WSPR | JTTY |
|---|---|---|
| time base | UTC slot grid (NTP / RTC, `SlotAccum`, grid lock) | **the sample count** — frames continue a message by their spacing in samples |
| decode cadence | once a slot, against a deadline | every quarter frame (472 ms), continuously |
| what a decode is | a finished message | a `MessageUpdate { id, f1_hz, start_s, text, complete }` that **grows frame by frame** |
| cost profile | a burst per slot | steady: Front ~310–380 ms of every 472 ms on core 1, Back ~200–320 ms mean on core 0 |
| falling behind | the slot is late | drop whole windows at the queue (`Front::push_or_drop`), never stall the audio |

Three consequences drive the design: the sample count must stay continuous (a lost run of samples shifts every later frame
and breaks messages), the UI has to show a message that changes after it first appears, and nothing is ever idle — there
is no quiet point for flash writes or for the panel.

## 2. Boot mode and build

- `BootMode::Jtty` in `mfsk-app-shared/src/boot_mode.rs`, in every per-variant match (`as_str` "jtty", `label`,
  `from_cfg_str`, `read`, `flipped` → Decode where the board lacks it, `slot_period_ms` — see §5, it must not be 0).
  Fifth entry in `ui::mode_picker::MODES` (5 × 48 + 40 = 280 px fits). `m5stack-s3-app` matches `BootMode` too and is
  outside CI: add its arm and `cargo check` it by hand.
- App feature `jtty-rx = ["mfsk-core/jtty"]`. The bench feature `jtty` pulls `jtty-stats` (a mutex in the hot path);
  never build both, since features unify.
- `main.rs`: the feature-gated `boot::run::<apps::jtty::JttyRx>` arm; `display.rs`'s `host_mode` list gets `Jtty`.
- `apps/jtty.rs`, modelled on `apps/ft4.rs`. `prepare` also prewarms the esp-dsp tables the receiver uses (256, 2048
  points) before WiFi, as the bench's `prewarm(8192)` does; lazily they would install after WiFi has taken the heap.

## 3. Memory: the decision this design turns on

**What must be internal for speed** (measured on the board):

| buffer | size | internal vs PSRAM |
|---|---|---|
| trellis survivors (`with_f32_metrics`) | 2 × 32 KB | a rung 190 ms vs 410 ms (§11 of the budget) |
| side-surface FFT buffer, 16-byte aligned | 32 KB | 4096 points 2.3 ms vs 10 ms (§13) |
| side-surface power row | 16 KB | part of 485 → 148 ms (§13) |
| `Back` stack (the ladder's correlations live there) | 32 KB (bench, internal) | PSRAM stack not measured |
| `Front` stack | 16 KB (bench, internal) | not measured |

That is ~160 KB with the band scan, ~112 KB without it (no FFT buffer or power row), each piece needing up to a 32 KB
contiguous block.

**What the app has**: ~173 KB free internal before WiFi (largest block 155 KB); WiFi takes ~105 KB, after which the
largest block is 31.7 KB; the USB host and the panel take more, and FT8 runs with a 15.8 KB floor. So:

| configuration | internal JTTY needs | fits with WiFi? |
|---|---|---|
| as the bench (scan, all hot buffers internal, internal stacks) | ~160 KB | **no** |
| channel 0 only, survivors internal, internal stacks | ~112 KB | **no** |
| survivors internal, stacks in PSRAM | ~64 KB (+48 with the scan) | only just, without the scan — unmeasured |
| survivors in PSRAM | small | yes, but each ladder call ~2.2× slower (worst window ~740 ms → ~1.4 s, estimated from 190 vs 410 ms a rung; E0 measures it) |

WiFi is not optional in practice: with the USB host installed, USB-Serial-JTAG is gone and the UDP log is the only
console; it also serves the files and settings. So the real choice is between **survivors in PSRAM** (slower ladder, more
windows dropped on a busy band) and **a smaller trellis working set** (library work: the survivors packed tighter, or the
list-Viterbi restructured). E0 measures the first; the second is designed only if the first is not good enough.

**PSRAM** is not free either: one `Prepared` is the window's analytic signal (113 KB) + channel-0 surface (66 KB) + side
surface (~426 KB) ≈ 605 KB, so a queue of 6 plus the two in hand is ~4.8 MB of the 8 MB, in 426 KB blocks churned every
472 ms. E0 records PSRAM free and largest block at a full queue; a depth of 5 (it matched the host in §14) or a quantised
side surface are the fallbacks. At the app's 2048-byte `ALWAYSINTERNAL` all of these land in PSRAM — and the channel-0
surface sits 152 bytes above 64 KiB, so the bench's 64 KiB build threshold must never be left on while decoding.

Build the receiver **once**, in `prepare`: a second build moved ~32 KB to PSRAM and slowed busy bands by half (§14).

### E0, first half: measured in the bench (2026-09-27/28)

The CoreS3, no WiFi, audio at its real rate, five trials each of six stations across the band and of noise only. The
buffers were placed by build order: `Receiver::new` allocates the side surface's FFT buffer and power row, and
`with_f32_metrics` allocates the survivors. Internal use is per receiver.

| configuration | internal | a ladder call (band) | front / back mean, ms | back worst | delay worst | windows dropped | band found |
|---|---|---|---|---|---|---|---|
| A scan, all internal (builder order) | 154 KB | 337 ms | 407 / 371 | 1057 | 3.13 s | 0 | 20/30 |
| B scan, survivors in PSRAM | 121 KB | 556 ms | 502 / 572 | 1547 | 6.44 s | 40 | 11/30 |
| C scan, survivors internal, the rest in PSRAM | 78 KB | 235 ms | 481 / 352 | 855 | 2.24 s | 0 | 20/30 |
| D scan, all in PSRAM | 14 KB | 705 ms | 778 / 778 | 2015 | 12.1 s | 39 | 16/30 |
| E channel 0 only, survivors internal | 78 KB | 276 ms | 147 / 133 | 457 | 0.63 s | 0 | 3/30 |
| F channel 0 only, all in PSRAM | 14 KB | 614 ms | 176 / 220 | 930 | 1.77 s | 0 | 3/30 |
| G = C with both stacks in PSRAM | 78 KB | 244 ms | 499 / 370 | 913 | 2.58 s | 0 | 20/30 |
| A built again | 122 KB | 559 ms | 501 / 566 | 1547 | 6.42 s | 39 | 9/30 |

- **The survivors decide the ladder.** A call costs 235 ms with both survivors internal and 556–705 ms with them in PSRAM.
  Built after the rest, as `new().with_f32_metrics()` does, one of the two found no free 32 KB internal block even on a
  fresh heap (A is 33 KB above B, and slower than C). A second receiver got neither ("A built again" repeats B). That
  was §14's rebuild slowdown. `Receiver::new_f32_metrics` now allocates them first.
- **The scan's buffers decide the front end.** In PSRAM they cost ~150 ms a window (C's front 481 ms against a 472 ms
  window). With everything in PSRAM (D) the front end does not keep up at all.
- **Stacks in PSRAM cost little**: C → G is +4 % on a ladder call and +18 ms on the back end.
- **What fits beside WiFi:** channel 0 only. E (78 KB internal) is the lightest, and whether it fits is E0's second
  half. F (14 KB) certainly fits, and keeps up. The scan needs ~126 KB internal (survivors plus the scan's buffers), so
  it needs library work first: smaller survivors, or a side surface that is fast in PSRAM.

## 4. Audio path, tasks, and the sample clock

```
uac_reader (core 0, prio 8)             jtty_front (core 1, prio 4)          jtty_back (core 0, prio 5)
  JttySink::push_samples ──► ring ──►     drain, fill gaps with zeros,         recv (generation, Prepared)
    never blocks; on overflow a            Front::push_or_drop ──► queue ──►   Back::process
    gap marker (position, count)           room = queue < N && ring backlog      MessageUpdate → UI (§5)
                                             under ~1 window
```

**The sample clock.** Ways samples get lost, and what each does:

- *Ring overflow* (the consumer fell behind): the sink writes a **gap marker** (position, count) into the ring instead of
  a bare counter, so the zeros go exactly where the samples were lost.
- *Reader timeouts* (the reader continues on 100 ms read timeouts for up to 3 s before the stall watchdog re-opens), *dropped
  isochronous frames* (uncounted), and *flash writes stalling both cores*: none of these is visible in the stream. The
  front end reconciles samples delivered against `esp_timer` (12 000 a second) and inserts zeros for a deficit above
  ~20 ms; a deficit above ~1 s means a reset.
- *Stream re-open or reset*: the queue is drained and discarded, `Back::finish` reports the open messages as incomplete,
  and new `Front`/`Back` are made around the same `Arc<Receiver>` (plain struct inits). Every `Prepared` carries a
  **stream generation**, so a stale window can never reach a new `Back` (whose window-order assert would panic, or whose
  timeline it would corrupt), and UI rows are keyed by (generation, message id) because `Assembler` restarts ids at 1.

**Cores and priorities.**

- Front on core 1 at 4, under WiFi (23) and lwIP (18): ~2/3 of a window with the scan before WiFi traffic. It drops a
  window (saving the surfaces, ~250 ms; the FIR work is still paid) when the queue is full **or** its own ring backlog
  passes about one window — the second is how the front end catches up when it is the one behind.
- Back on core 0 at 5, below the UAC reader and class driver (8). **The panel goes above Back**, not below: FT4 keeps the
  panel at 1 only because of its reply deadline, and JTTY has none; with Back busy continuously a priority-1 panel would
  starve, and with it the waterfall drain. FT8's `PANEL_PRIORITY = 7` is the precedent (8–13 % of core 0, measured).
- Watchdog: `DEINIT_WDT = true`, the default for non-FT8 receivers (the front end never lets IDLE1 run).
- Stacks: Back stays on a 32 KB internal stack until a PSRAM stack is measured (the correlations are on it); every stack is
  sized from `board::log_task_stacks`, not copied.
- Log volume: per-window lines are rate-limited — the UDP log goes out through lwIP on core 1, beside the front end.

## 5. UI under the one-screen rule

No new screen, no new renderer. What the shared code does today, and what JTTY needs from it:

- **Rows**: `push_decode` deduplicates by exact text and moves a refreshed row to the top (LRU). JTTY adds
  `UiState::publish_update(key, …)` that replaces the row with the same (generation, id) and then behaves as every other
  refresh: it moves to the top. Hosttested in `hosttest/mfsk-app-shared`.
- **Text**: the row shows the **head** of the message (`msg` is `String<22>`; the row has room for 25). The calls are at
  the head, and `selected_decode_msg` hands the row text to `qso::call_station` — a tail would lose them.
- **SNR / DT**: the renderer prints a non-finite SNR as `+0` and DT as `+0.0`, which would be fabricated values. Either
  `MessageUpdate` carries the last frame's S/N (a small library + FFI change, preferred) or the shared renderer learns a
  blank field — decided in E2.
- **Slot period**: `slot_period_ms` cannot be 0 — `decoded_slot_unix(0)` divides by zero, no row is ever green, and the
  ALL.TXT flush treats every moment as quiet. JTTY needs the three uses split: the green-row age (a few seconds), the
  waterfall rules (none — the waterfall already handles a period of 0), and the flush policy (below).
- **ALL.TXT**: one line per message, on `complete` (or on an incomplete one being pruned, marked), stamped from a UTC
  anchor taken at stream start plus the message's `start_s` — not from a slot.
- **Flash writes**: they stall both cores and there is no quiet point; flush when no message is active and the queue is
  empty, bounded by a maximum age.
- **Status strip**: queue depth, windows dropped, gaps filled, through `set_acq_line`.

## 6. Settings and WiFi

WiFi as FT4 (`http` on), for the console and files; no NTP requirement for decoding, but the UTC anchor for ALL.TXT
wants the clock. The operator's `f0_hz` / `ftol_hz`: a fixed default (1500 Hz ± 50 Hz, the scan 200–2800 Hz) in E1, a
setting later.

## 7. Verification

- **Host mode on the radio for E0.** `MFSK_CORES3_SIM` keeps the board a USB peripheral, so the USB host's internal DRAM
  and its interrupt load on core 0 are absent: memory numbers from the SIM feed do not decide anything.
- **SIM feed for correctness** (E1): the golden `jtty/260807_134110.wav` and the `testsig` patterns through the real sink,
  with `MFSK_SIM_NO_CLOCK` and a loop that the slot re-alignment does not cut (today it re-aligns every pass when the loop
  equals the slot length, skipping samples). Pass: when nothing was dropped, decodes equal the host's on the same looped
  stream.
- Logged per run: front / back per window, delay, queue depth, dropped windows, gaps filled, internal and PSRAM free /
  largest block (at a full queue), task stack high-water.

## 8. Transmit (later phase)

`jtty_tx` (pure, half-duplex, driven by the 12 kHz audio clock), the Phase-T0 OUT probe (mono 48 kHz only), and a CI-V
`ptt` frame nobody sends. During a transmit segment the sequencer's zeros **replace** the incoming audio (not the
overflow path, which inserts). If the IC-705 stops its IN stream while transmitting, the 3 s stall watchdog would reset
the receiver every transmission — to check first in T0.

## 9. Phases

- **E0** memory and speed: in the bench first (no WiFi needed to time it), survivors in PSRAM vs internal, with and without
  the band scan, Back's stack internal vs PSRAM; then in host mode on the radio with WiFi: internal and PSRAM floors at a
  full queue. Decides the configuration of §3, or opens the trellis-memory library work.
- **E1** mode skeleton, sink with gap markers, clock reconciliation, generations, the two tasks, panel priority, publish
  on `complete` with the slot-period split and the UTC anchor; SIM verification.
- **E2** live-updating rows (`publish_update`), SNR in `MessageUpdate`, flush policy; hosttests.
- **E3** on the air with the IC-705.
- **E4** transmit (separate design).

## 10. Open questions

1. Is a ladder with its survivors in PSRAM good enough (E0), or does the trellis working set have to shrink?
2. Band scan by default, given it costs ~48 KB internal and ~426 KB of PSRAM per queued window?
3. SNR: into `MessageUpdate`, or a blank field in the shared renderer?

## 11. Review of draft 1 (2026-09-27), what changed

- The internal-DRAM estimate left out the trellis survivors (64 KB, 190 vs 410 ms a rung) and the task stacks; with them
  the bench configuration does not fit beside WiFi at all (§3).
- "WiFi off with a serial console" does not exist in host mode (§3).
- The app's baseline is ~173 KB free before WiFi, not the bench's 245 KB (§3).
- `Prepared` is ~605 KB of PSRAM each; a queue of 6 is ~4.8 MB (§3).
- A reset needs stream generations and a drained queue; ids restart at 1 (§4).
- Silent sample loss (reader timeouts, dropped isochronous frames, flash stalls) needs clock reconciliation; overflow needs
  a positioned gap marker, not a counter (§4).
- The panel must be above Back, not below (§4).
- `slot_period_ms = 0` panics in `decoded_slot_unix`; NaN SNR renders as `+0`; showing the tail would break
  `qso::call_station`; refreshed rows move to the top (§5).
- The SIM feed cannot measure memory (the board stays a USB peripheral) and cuts a looped stream at slot boundaries (§7).
- Also: prewarm FFT tables before WiFi, `m5stack-s3-app`'s `BootMode` match, feature unification with `jtty-stats`, UDP log
  volume on core 1, the IN stream during TX.
