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
| `Back` stack (the ladder's correlations live there) | 32 KB | **not measured** — the §7b "no difference" compared internal against internal (§12) |
| `Front` stack | 16 KB | **not measured** — §7b's +25% compared a pinned prio-4 task against an unpinned default-priority one, both stacks internal (§12) |

That is ~160 KB with the band scan, ~112 KB without it (no FFT buffer or power row), each piece needing up to a 32 KB
contiguous block.

**What the app has**: ~173 KB free internal before WiFi (largest block 155 KB); WiFi takes ~105 KB, after which the
largest block is 31.7 KB; the USB host and the panel take more, and FT8 runs with a 15.8 KB floor. So:

| configuration | internal JTTY needs | fits with WiFi? |
|---|---|---|
| as the bench (scan, all hot buffers internal, internal stacks) | ~160 KB | **no** |
| channel 0 only, survivors internal, internal stacks | ~112 KB | **no** |
| survivors internal, `Back`'s stack in PSRAM | ~80 KB (+48 with the scan) | **no** — free (no speed cost), but still short by itself |
| survivors internal, both stacks in PSRAM | ~64 KB (+48 with the scan) | only just, without the scan — and at a real cost: `Front`'s stack in PSRAM alone costs it +25% a window and nearly doubles decode latency (§7b) |
| survivors in PSRAM | small | yes, but each ladder call ~2.2× slower (worst window ~740 ms → ~1.4 s, estimated from 190 vs 410 ms a rung; E0 measures it) |

**Superseded 2026-09-28 (user decision): JTTY mode runs without WiFi** — §6 has the consequences, §12 the measurement
that forced it. The argument below is kept as it was made.

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

### E0, survivors first: measured on the board (2026-10-01)

`jtty-bench` built from `main` (`f1ae0a5a`, after #513), the same CoreS3, no WiFi, the same five trials of six stations
and of noise only. Log (local; `logs/` is not committed): `jtty-bench_main-f1ae0a5a_2026-10-01.log`. "Internal" is the internal
heap the receiver took (free before the build minus free after).

| configuration | internal | a ladder call (band) | front / back mean, ms | back worst | delay worst | windows dropped | band found |
|---|---|---|---|---|---|---|---|
| A' scan, all internal, `new_f32_metrics` (survivors first) | 169 KB | 206 ms | 314 / 259 | 699 | 1.40 s | 0 | 22/30 |
| C scan, survivors internal, the rest in PSRAM | 62 KB | 234 ms | 480 / 352 | 866 | 2.21 s | 0 | 22/30 |
| A' built again | 171 KB | 206 ms | 314 / 258 | 696 | 1.40 s | 0 | 22/30 |

- **Allocating the survivors first does what #513 said.** Against A (builder order, 2026-09-27) a ladder call is 337 →
  206 ms, the back end's worst window 1057 → 699 ms, the worst delay 3.13 → 1.40 s, and 20 → 22 of 30 band messages.
- **The rebuild slowdown is gone.** "A built again" fell to 559 ms, 39 dropped windows and 9/30; "A' built again"
  repeats A' to the millisecond.
- The two-core split's correctness gate passed on the board in the same run (`SELFTEST: PASS`, 2 cases: the upstream
  recording, and six long messages on a busy band).
- 47 task-watchdog warnings (IDLE0 starved while the bench synthesises its audio), as in earlier runs; no reset.
- Still open: whether channel 0 (E) fits beside WiFi is moot since JTTY runs without WiFi (§6); A' needs 169 KB
  internal, so the app must build the receiver before anything else takes the large blocks.

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
- Stacks: both internal. Back's is 20 KB, reserved from `embedded_shared::worker_arena` in `prepare` while the heap is
  whole and run as a static FreeRTOS task (a pthread's stack is allocated at spawn, when a 32 KB internal block could no
  longer be found, §12); its peak is 12 996 B (1.56x margin, `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK` behind it).
  Front's is 8 KB, peak 2 616 B. Both measured with `board::log_task_stacks` on the golden and band6 scenes (§13).
- Log volume: per-window lines are rate-limited — the UDP log goes out through lwIP on core 1, beside the front end.

## 5. UI under the one-screen rule

No new screen, no new renderer. What the shared code does today, and what JTTY needs from it:

- **Rows**: `push_decode` deduplicates by exact text and moves a refreshed row to the top (LRU). JTTY adds
  `UiState::publish_update(key, …)` that replaces the row with the same (generation, id) and then behaves as every other
  refresh: it moves to the top. Hosttested in `hosttest/mfsk-app-shared`.
- **Text**: the row shows the **head** of the message (`msg` is `String<22>`; the row has room for 25). The calls are at
  the head, and `selected_decode_msg` hands the row text to `qso::call_station` — a tail would lose them.
- **SNR / DT**: the renderer prints a non-finite SNR as `+0` and DT as `+0.0`, which would be fabricated values. **Decided
  (#646, 2026-10-10): `MessageUpdate` carries the S/N** — `MessageUpdate::snr_db`, the *first* frame's SNR in 2 500 Hz floored
  at -17 dB, which is what WSJT-X v3.3.0-beta1 reports (`start_snrdb`; the draft said "last frame's", upstream settled on the
  first). The app passes it to the row and to `ALL.TXT`; a message still has no DT (no slot), so that column stays blank.
  On the board (`jtty-demo`, busy-band scene of six stations, 4 of 6 complete, 2026-10-10, log
  `logs/jtty_demo_snr646b_2026-10-10.log`): 331 Hz -10 dB (true -9.6), 1523 Hz -5 (-5.3), 1969 Hz -4 (-2.8), 2361 Hz -4 (-4.7).
- **Slot period**: `slot_period_ms` cannot be 0 — `decoded_slot_unix(0)` divides by zero, no row is ever green, and the
  ALL.TXT flush treats every moment as quiet. JTTY needs the three uses split: the green-row age (a few seconds), the
  waterfall rules (none — the waterfall already handles a period of 0), and the flush policy (below).
- **ALL.TXT**: one line per message, on `complete` (or on an incomplete one being pruned, marked), stamped from a UTC
  anchor taken at stream start plus the message's `start_s` — not from a slot.
- **Flash writes**: they stall both cores and there is no quiet point; flush when no message is active and the queue is
  empty, bounded by a maximum age.
- **Status strip**: queue depth, windows dropped, gaps filled, through `set_acq_line`.

## 6. Settings and WiFi

**No WiFi in JTTY mode** (user decision, 2026-09-28): `JttyRx::net_config` returns `None`, whatever the CONFIG page
says; the other modes keep their WiFi as before. Beside the WiFi driver internal DRAM ran to 1–7 KB free and the
receiver dropped windows (§12). What that costs on the radio, where the board is the USB host:

- **No console but the LCD.** USB-Serial-JTAG is gone in host mode and the UDP log needs WiFi, so nothing streams off
  the board; the status strip (queue, drops, gaps, resets) is the operator's view. Measurement runs happen on the SIM
  feed, where the board is a USB peripheral and the serial console works.
- **No NTP.** The `all.txt` anchor comes from the BM8563 RTC, which `pmic::init` reads into the system clock — at the
  panel's start, *after* `start`, so the first audio can arrive while the clock is still `Unset`. The sink therefore
  re-takes the anchor whenever `time_sync::clock_epoch()` moves (measured: `Unset` → `Rtc`, the anchor moved by 640 and
  933 ms on two boots, and `all.txt` lines were written from it). The RTC is only as good as its last setting: another
  mode's NTP sync writes it back (`rtc::write_from_system_clock`), and it holds the minute for weeks (`MANUAL_M5STACK_CORES3.md` §7). A board whose RTC was never set writes no `all.txt` lines — a
  line stamped 1970 is worse.
- **No HTTP config page**, so no file download or settings over the network in this mode; files stay on LittleFS for
  the next boot in a mode with WiFi.

The operator's `f0_hz` / `ftol_hz`: a fixed default (1500 Hz ± 50 Hz, the scan 200–2800 Hz) in E1, a
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
- **E2** live-updating rows (`publish_update`), SNR in `MessageUpdate` (done, #646), flush policy; hosttests.
- **E3** on the air with the IC-705.
- **E4** transmit (separate design).

## 10. Open questions

1. Is a ladder with its survivors in PSRAM good enough (E0), or does the trellis working set have to shrink?
2. Band scan by default, given it costs ~48 KB internal and ~426 KB of PSRAM per queued window?
3. ~~SNR: into `MessageUpdate`, or a blank field in the shared renderer?~~ Into `MessageUpdate` (#646).

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
- `slot_period_ms = 0` panics in `decoded_slot_unix`; NaN SNR renders as `+0` (JTTY now passes `MessageUpdate::snr_db`, #646); showing the tail would break
  `qso::call_station`; refreshed rows move to the top (§5).
- The SIM feed cannot measure memory (the board stays a USB peripheral) and cuts a looped stream at slot boundaries (§7).
- Also: prewarm FFT tables before WiFi, `m5stack-s3-app`'s `BootMode` match, feature unification with `jtty-stats`, UDP log
  volume on core 1, the IN stream during TX.

## 12. E1 on the board (2026-09-28), what changed

Measured with the SIM feed (§7: the board is a USB peripheral, so the USB host's DRAM and interrupts are absent — these
are floors, not the host-mode figures E0 still owes). Logs: `m5stack-cores3-app/logs/jtty_e1_sim_*_2026-09-28.log`.

- **`jtty-bench` §7b (#516) did not measure a PSRAM stack.** `esp_pthread_set_cfg` rejects `stack_alloc_caps` without
  `MALLOC_CAP_8BIT` (`ESP_ERR_INVALID_ARG`, esp-idf v5.5.3 `pthread.c:159`); the bench set `EnumSet::only(Spiram)` and
  discarded `set()`'s result, so each "PSRAM" case ran with the previous (default) config: unpinned, default priority,
  internal stack. Its "Back: no difference" compared internal with internal; its "Front: +25 %" compared pinned prio 4
  with unpinned default priority. The §3 stack rows are struck; the app spawns with `Spiram | Cap8bit` and checks every
  `set()`.
- **An internal stack for Back cannot be allocated.** Building the receiver takes 122 KB internal (214 KB free before,
  largest block 136 KB; 92 KB left); a 32 KB internal Back stack then fails with ENOMEM even with WiFi off
  (`…_nowifi_backinternal`). Back's stack is in PSRAM by necessity, not by choice.
- **Stack high-water** (`board::log_task_stacks`, golden and band6): Front uses ≤ 2.6 KB of its 16 KB internal stack
  (13 808 bytes free at worst), Back ≤ 13 KB of its 32 KB (19 680 free, band6). Front's stack is the one internal
  allocation here that is plainly oversized.
- **Beside WiFi, internal DRAM is exhausted and the receiver runs over budget.** Golden loop, WiFi on: internal 1–7 KB
  free (minimum 0), front 479–558 ms a window, back 385–515 ms mean, delay 2.9–3.8 s mean, 23 windows dropped in ~2 min,
  and only the pass with no drop decodes the whole text. `band6` (`testsig::pileups(1)`), WiFi on: internal 4–6 KB,
  front 612–842 ms, back 698–1017 ms mean (1.9 s worst), 67 dropped, 4 messages complete with gaps where the host
  decodes 12 on the same 3-pass stream. WiFi off (`…_nowifi`): internal 47 KB free, front 415–476 ms, back 353–471 ms,
  7 dropped; start times match the host's (3.14 / 33.38 / 63.63 s) — still slower than the bench on the same recording
  (front 320, back 223 ms). E0's "then in host mode on the radio with WiFi" was never run before E1; this is what it would
  have found, and it reopens §3's decision (open question 1) before E3.
- E1's own choices where the draft left a number open: the clock reconciliation tracks a baseline that follows the
  deficit at up to 1000 ppm (the IC-705 against the ESP crystal is well inside; a ±325 ppm source over an hour is
  hosttested), and fills only a jump above it (> 20 ms), resets above 1 s. The green-row age is 10 s
  (`BootMode::fresh_row_ms`); the waterfall has no slot rules (`slot_rules_ms` = 0). ALL.TXT is written when no message
  is open and the queue is empty (`storage::quiet_rx`) for lines ≥ 30 s old, and in any case at 10 min. The JTTY SIM
  feed is continuous (`uac::spawn_sim_feed_continuous`) — no boundary alignment and no per-pass re-alignment.

## 13. E1b: WiFi off, Back's stack internal, and why the receiver is still slower than the bench (2026-09-28)

Golden and band6 on the SIM feed, WiFi off (`logs/jtty_e1b_*_2026-09-28.log`).

- **Memory.** The receiver build takes 125 124 B internal and 71 692 B PSRAM (218 999 B internal free before, largest
  131 072; 93 875 B after, largest 31 744). Back's 20 KB reservation leaves 73 391 B; running, internal sits at
  34–35 KB free (min 19–20, largest 13). Before this change (Back's stack in PSRAM, Front 16 KB) it was 47 KB free.
- **Both trellis survivor buffers are in PSRAM.** A heap walk around the build lists every new block: two 32 768 B
  blocks landed side by side in PSRAM (`0x3c3bd9b8`, `0x3c3c59d0`); two more 32 768 B blocks, two of 17 408 and two
  of 4 352 B are internal. The walk gives placement, not order; that the PSRAM pair is the survivors follows from the
  construction order (`TrellisScratch::new` allocates its two 32 KB buffers last) and from the region arithmetic below. Before the build the internal free blocks are 138 912, 32 024, 32 020
  and 8 156 B: only one region can hold a 32 KB block (the two ~32 KB regions are 744 B short; one is probably the
  32 KB internal pool the IDF logs reserving at boot), four such blocks need 131 088 B of it, and `Receiver::new()` fills that region with its
  smaller pieces first because the survivors come last (`with_f32_metrics` → `TrellisScratch::new`). The same build in
  `jtty-bench` had 245 KB free with a 156 KB region and took 154 KB internal. §3 puts the survivors in PSRAM at 410 ms
  a rung against 190.
- **Front's gap is the other core, not core 1.** Per-task CPU over 30 s spans: core 1 is `jtty_front` 84–88 % and
  IDLE1 12–16 %, nothing else; core 0 is `jtty_back` 63–75 %, `main` (panel, prio 7) 8.2–8.6 %, `uac_sim` 1.1–1.3 %.
  Timing each window's push by what Back was doing meanwhile: **255 ms with Back idle, 430–462 ms with Back decoding**
  (golden, four spans, 16–22 and 31–92 windows each). The bench's 320 ms is between the two because its Back was
  idle more. The cores share the data cache and the PSRAM bus, and Back's survivors are now PSRAM traffic.
- **Back's gap**: the survivors in PSRAM (above), plus the panel's ~8.5 % of core 0 above it. Back's stack in PSRAM was
  not it: moved internal, golden back mean went 353–471 → 327–436 ms.
- **Results.** Golden: front 391–433 ms, back 327–436 ms mean (1.2–1.3 s worst), delay 1.0–1.9 s mean, 10 windows
  dropped in 150 s, every message at the host's start times, intact only in the passes with no drop. Band6: back
  623–795 ms mean, core 0 at 84–90 %, 118 windows dropped in ~180 s, 23 completes of which 6 intact (the host decodes
  4 intact a 20 s pass).

Putting the survivors in internal DRAM needs them allocated before the smaller buffers — a change in `mfsk-core`
(`Receiver`'s construction order, or a constructor that takes the f32 scratch first). Left for the user to decide.


## 14. E1c: the survivors internal (`Receiver::new_with_f32_metrics`), and internal DRAM runs out (2026-09-28)

`Receiver::new_with_f32_metrics()` builds the same receiver largest allocation first (`mfsk-core`, bit-identical
decodes, `tests/jtty_rx.rs`). SIM feed, WiFi off (`logs/jtty_e1c_*_2026-09-28.log`).

- **Placement** (heap walk): the three 32 768 B blocks allocated first — both survivor arrays and the transform buffer —
  and the 20 480 B sync wave are internal; a 16 896 B block and a 6 144 B block went to PSRAM. The build takes
  160 968 B internal and 23 048 B PSRAM (218 999 B free before; 58 031 B after, largest 31 744). After Back's 20 KB
  reservation: 37 547 B (largest 15 872).
- **Speed now matches the bench.** Golden: front 330–340 ms a window, back 203–247 ms mean (612–637 worst), delay
  530–597 ms mean, queue at most 1, **0 windows dropped**, every message intact. Band6: front 366–394 ms, back
  318–420 ms mean (862–907 worst), delay 0.8–1.5 s mean, queue at most 5, **0 dropped**, every message intact, and the
  first three passes equal to the host's `sim_streams_looped_on_the_host` message for message (start, frequency,
  text). Per-core CPU: golden `jtty_back` 39–48 %, IDLE0 43–52 %; band6 `jtty_back` 64–76 %.
- **Front under Back mostly went away.** Front a window with Back idle / busy: golden 278–279 / 318–329 ms, band6
  279–280 / 344–375 ms (before: 255 / 430–462 on golden). The remaining ~15–30 % is the shared cache and PSRAM bus
  (the surfaces and `Prepared` windows are still PSRAM).
- **Internal DRAM does not fit.** Steady state 3–6 KB free, **minimum 0**, largest block 0–4 KB; the storage task
  could not get its 5 KB internal stack (`storage: could not spawn the task (Not enough space)`), so **no `all.txt`
  this boot**. Stacks: Back 12 996 B peak of 20 480, Front 2 616 of 8 192, `uac_sim` (SIM only) 1 920 of 8 192,
  `main` 9 816 B free. On the radio there is no `uac_sim`, but the USB host and the UAC driver need internal DRAM of
  their own — E3 needs more room than this, not less.

## 15. E1d: survivors packed, the storage stack reserved (2026-09-28)

User decision on §14: pack the survivor entries (`mfsk-core`) and reserve the storage task's stack at boot; Back's and
Front's stacks and the power row / sync wave stay where they are. SIM feed, WiFi off (`logs/jtty_e1d_*_2026-09-28.log`).

- **Packing.** `Surv<f32>` is 12 bytes (`repr(C, packed(4))`); the survivor arrays are 24 576 B each, both internal
  (heap walk). Decodes are bit-identical (a fingerprint of every list for 360 frames, pinned from the unpacked
  layout; the 33 upstream ladder cases; the golden/band6 stream and scan comparison), and nothing got slower: golden
  back 184–228 ms mean against 203–247 unpacked, front 304–318 against 330–340.
- **Storage.** `storage::reserve_stack()` (JTTY only; every other mode keeps its lazy pthread) takes the 5 120 B stack
  in `prepare`, and the task runs on it as a static FreeRTOS task. `all.txt` lines are written again
  (`storage: [quiet] all.txt +192 B`). Peak 3 008 B of 5 120.
- **Memory.** The build takes 144 584 B internal and 23 048 B PSRAM (218 655 B free before, largest 131 072; 74 071 B
  after, largest 31 744). After Back's 20 KB: 53 587 B; after storage's 5 KB: 48 463 B (largest 31 744). Running:
  9–16 KB free, largest 5–7 KB, **minimum 0–1 KB**. The dips come from each window's small allocations (at the
  board's 2 KB `ALWAYSINTERNAL` rule they try internal first and fall back to PSRAM); no allocation failed in either
  run.
- **Golden**: front 304–318 ms, back 184–228 ms mean (582–605 worst), delay 473–537 ms mean, queue at most 1,
  0 dropped, every message intact. Front with Back idle / busy: 253–254 / 332–335 ms.
- **Band6**: front 342–362 ms, back 293–388 ms mean (807–848 worst), delay 0.7–1.0 s mean, queue at most 4, 0 dropped,
  39 completes with no gaps, the first three passes equal to the host's message for message. Front with Back idle /
  busy: 255–262 / 337–348 ms. Core 0: `jtty_back` 58–72 %.
- **Stacks**: Back 13 020 B peak of 20 480, Front 2 624 of 8 192, storage 3 008 of 5 120.
- **Still open for E3**: on the radio the USB host and UAC driver need internal DRAM that `uac_sim` (8 KB here) does
  not; with 9–16 KB free and a 0–1 KB minimum, that is the next number to measure.
