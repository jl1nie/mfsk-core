# The CoreS3 FT8 flow on the half-dB cells: implementation plan

Written 2026-11-26, from the host results of `FT8_WINDOW_AND_COARSE_DESIGN.md` (sections 11-23b), the board's own record
(`CORES3_FT8_SLOT_BUDGET.md`, `S3_GATE_OSD_ABLATION.md`, `embedded-poc/CLAUDE.md`, `worker_arena.rs`, `esp_dsp_fft.rs`, `dual_core.rs`) and one read of
the firmware's memory and dual-core structure. **Nothing in this plan has been run on an Xtensa core.** Every host number is a host instruction count or
a host recall; every board number is from the existing record and is named as such. Where a fact is not established it is listed in section 7, not assumed.

## 1. What is to be built, and why

The shipped CoreS3 flow (the "old S3"): the u16 power spectrogram built while the slot is captured (`stage1_inc`, 12 kHz, 3 840-point 16-bit FFT), the hard-sync
gate on it (`coarse_sync_gated_spec`), `fine_sync_at`, a block-0 refine, and stage 3 with a per-candidate Goertzel DFT (`fill_symbol_spectra_goertzel`), BP and
no OSD / SIC / AP. The replacement reads each candidate's 79 x 8 symbol spectra from a **half-dB byte cell store** made while the slot is captured, drops the
per-candidate DFT and the fine sync, optionally adds `ft8dec`'s subtraction passes, and takes its audio at **8 kHz** (the radio's USB audio is 48 kHz stereo,
so 8 kHz is a 6:1 conversion in the reader in place of 4:1).

The library side exists and is pushed (`decode_block_cells`, `decode_block_cells_sic`, `HalfDbCells`, `coarse_sync_gated_cells`; host only). The firmware side
does not. This plan is the firmware side, and the library changes the firmware needs first.

## 2. Established facts (host)

60 real slots (38 `ft8_lib/test/wav/20m_busy` + the 22 recordings), 8 kHz, fixed-point, S3 flow with OSD, host instructions a slot over 12 busy slots
(`FT8_WINDOW_AND_COARSE_DESIGN.md` 21-23b; the first two columns are the old S3's own flow on the same host build):

| | old S3 (Goertzel + fine sync), 15 / 30 cand. | cells flow, 1 pass | cells + `ft8dec` subtraction, 2 passes |
|---|---|---|---|
| `jt9` hits of 1 338 | 761 / 1 009 | 754 / 1 001 | 922 / **1 192** |
| found of the union of `jt9`'s and `ft8dec`'s lists (1 409) | 769 / 1 036 | 765 / 1 035 | 941 / 1 247 |
| M instr a slot | 295 / 549 | **180 / 396** | 488 / 881 |
| peak heap on the host meter | 422 KB | 258 KB | 295-299 KB |

* `ft8dec` itself: 990 / 1 189 / 1 204 `jt9` hits at 1 / 2 / 3 passes for 335 / 850 / 1 278 M instructions. The cells flow with 2-3 passes reaches the same recall
  (1 192 / 1 206 at 30 candidates) for 881 / 1 443 M. The number of passes is the caller's.
* The cells flow with one pass at 15 candidates is **40 % fewer instructions at the same union recall** as the old S3 (180 against 295 M; 765 against 769).
  At 24-30 candidates one pass already passes `ft8dec`'s single pass (979 / 1 001 against 990 `jt9` hits). Fifteen candidates cannot reach it with any
  number of passes (940 at three): the first pass is candidate-limited and a later pass only takes what a subtraction uncovers.
* What these runs include: the staircase with **OSD on** (the S3 flow's V9). The board ships without OSD (`S3_GATE_OSD_ABLATION.md`: "OSD is decided on the
  board"). The cells flow's figures without OSD are **not measured** (OSD is 102 of the 558 M of the old S3 at 30 candidates, about 9 M a try, 90 % of tries fail).
* 8 kHz against 12 kHz is already measured and was not what limited recall (`S3_GATE_OSD_ABLATION.md`, "The S3's flow at 8 kHz, native": 22 recordings, 30 candidates,
  267 at 12 kHz, 267 via an 8 kHz round trip, 268 native).
* **Not established on the host either**: noise-only slots (tier C: the phantom rate of the cells flow and of the subtraction passes; the Linux box), `snr_db`
  of these flows (about 10 dB below `jt9`'s, uncalibrated, the cells flow 1.3 dB below the old S3's), and two new decodes of the 2-pass flow that read like noise
  (`LU5HA UA9TK R-13` / `R-20`: a subtraction ghost is the likely reading).

## 3. Established facts (board)

**Time** (`CORES3_FT8_SLOT_BUDGET.md`, 278 on-air slots on 7041 kHz, five builds, 2026-09-20): pre-boundary compute 810 ms (emit point at row pair 87), post-boundary
compute 500 ms of which about 270 used, pass-1 candidates 30 (saturated every slot), stage-3 candidates `max_cand` 15 (used in full), about 72 ms a candidate
(BP + refine), 8.42 decodes a slot of which 7.91 in time (94 %), hit rate of stage-3 candidates 56 %, **no OSD, no SIC, no AP**. The deadline almost never
binds (`cut > 0` on 2 of 278 slots); the receiver was found **selection-limited, not time-limited** (ranks 16-25 of pass 1 convert at about 0.1 % on the host
fixtures). A decode after the slot boundary cannot be answered in that period; it still reaches the screen and the next period. The reply deadline is the
boundary: transmit audio starts 0.5 s after it. Stage timings in the firmware's own record (`dual_core.rs` docs, exploration of 2026-11-26): coarse about 103 ms,
fine sync about 292 ms, stage 3 about 1.06 s on both cores.

**Memory**:
* Internal DRAM is the scarce resource. At boot, before WiFi and the USB host start: 247 471 B free, largest block 155 648 B. With WiFi associated the
  largest free internal block is **31 744 B** and about **15 kB** are free at a slot boundary (`worker_arena.rs` module docs; `sdkconfig.defaults` of the CoreS3 app: floor
  15 851 B).
* **`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` is 2048 on the CoreS3** (`embedded-poc/m5stack-cores3-app/sdkconfig.defaults:206`, the source of truth; lowered from 4096, the
  floor of free internal DRAM went from 1 699 B to 15 851 B; never raise it: 16 384 drained internal DRAM into tlsf corruption). A heap allocation of 2 KB or less
  lands in internal DRAM, a larger one may go to PSRAM; a `static` goes to `.bss` (internal) whatever its size.
* Large buffers are taken **at boot before WiFi** (`worker_arena::reserve`, `Owner::Ft8Scratch` 10 112 B for the two 5 056 B `cs` staging buffers;
  `esp_dsp_fft::reserve_mixed_scratch` 30 720 B of internal DRAM for the mixed-radix 3 840 transform, "worth 2.44x of the transform"). A late allocation can fail
  once WiFi is up (issue #260).
* In PSRAM (8 MB): the u16 spectrogram 359 536 B, the `allsum` head and tail 342 240 B each, two 360 000 B audio buffers, the Pass-2 `cs` boxes (5 kB each).
* **The FFT backend**: `EspDspPlanner16` gives the 3 840 plan only (`esp_dsp_fft.rs`: "Plans for non-power-of-2 sizes panic"); a 2 560-point sc16 plan
  (`fft_mixed_2560_sc16`, `Plan2560Sc16`, 256 x 10, exists on the host: `S3_GATE_OSD_ABLATION.md`) has no esp-dsp wrapper. The twiddle table of esp-dsp is one global
  resized per length, guarded by `Fc32Guard`; the mixed scratch is shared by every mixed-radix user and sized for the longest.
* **Dual core** (`dual_core.rs`): core 0 runs the decode thread (32 KB stack), the head half of the coarse search and the main half of stage 3; core 1 runs
  `stage1_inc` (priority 6), the `dsp_worker` (priority 5, 16 KB) and WiFi. Work crosses on depth-1 FreeRTOS queues carrying `Box` pointers. Pass 2 and fine sync are
  static midpoint splits; stage 3 is **work stealing, group size 1** (per-candidate BP time varies too much for a static split).
* **Input**: the IC-705's USB audio is a single alt-setting, **48 kHz / stereo / 16-bit** (`uac.rs:12,45-47,306-310`); the reader takes the left channel and resamples
  4:1 to 12 kHz (`LinearResamplerI16To12k`, WSJT-X's own `fil4` 49-tap low-pass first: `engine/dsp/resample.rs`). (The ICAP radio-resident app gets 8 kHz mono from the
  radio itself; that is a different target and is not this plan.)
* Documentation inconsistencies found while reading, not resolved here: `embedded-poc/CLAUDE.md` gave 4096 for the CoreS3's `ALWAYSINTERNAL` (fixed to 2048 with the
  config as source), and `EMBEDDED.md` gives a different priority for `stage1_inc` (3) than the code (6).

## 4. Design

**Principle**: the firmware keeps its shape (capture on core 1 builds the spectrum while audio arrives; the decode thread on core 0; stage 3 by work stealing). Three
things change inside it: what `stage1_inc` produces (byte half-dB cells at 8 kHz), what the gate scores (the `ft8_lib` statistic on those cells) and what stage 3
reads (the cells, not a DFT of the audio). Subtraction passes are an optional later stage on the same stores.

### 4.1 Front end: 48 kHz to 8 kHz in the UAC reader
A 6:1 decimation with a low-pass designed like `fil4` (pass to about 3.5 kHz, stop from 4 kHz: the 8 kHz Nyquist), in `engine::dsp::resample` beside the 12 kHz target
(whose `TARGET_RATE` is a constant: the module takes an arbitrary input rate to 12 kHz today). FT8 only: FT4, WSPR, JTTY and the other modes stay on 12 kHz, so the reader
selects the ratio by the mode of the boot. Host gate: the recall of the 22 recordings through the new 48 kHz -> 8 kHz path (a 12 kHz recording up-converted to 48 kHz is the
model) is within the floors the 8 kHz flow has through the 12 -> 8 kHz model (`icap-app/src/resample.rs`), and the SNR crossing on noise is not worse than 0.2 dB.

### 4.2 FFT: a 2 560-point sc16 plan on esp-dsp
A wrapper beside the 3 840 one (`MixedRadix3840Sc16Fft`): the same 256-point esp-dsp inner kernel, ten rows instead of fifteen, `Aligned16Rows`, the shared `Fc32Guard`,
the shared internal scratch (20 480 B for 2 560; **the 30 720 B reservation is sized for 3 840 and stays so unless no mode uses a 3 840-point transform in the build**: the
saving of about 10 KB of internal DRAM is a conditional, not a given). `EspDspPlanner16::plan_forward(2560)` must stop panicking.

### 4.3 The producer: byte cells in `stage1_inc`
`stage1_inc` already builds the spectrogram row by row as audio arrives, on core 1. For FT8 it makes, per pair of rows (the two-for-one real FFT of the library's
demux, `spectrogram_cells`), a Hann-window-over-two-symbols frame (`hann_q15`: **the half table, 2.5 KB, `static` so that it is internal**), the 2 560-point FFT, and the
byte `half_db` of the 32-bit power (`clz` + a 16-entry table: no division, no `log`): **172 224 B of cells** (`HalfDbCells`: 184 rows x 936 bins at 200-3 000 Hz) in
PSRAM, written row by row. **Library change needed** (section 5, step 0): `HalfDbCells` today is built by the demux in one call; the firmware needs a store it can
write a row at a time, with a count of rows filled, and a search that respects that count. The gain: the host demux takes the slot's peak over the whole slot; the board's
`stage1_inc` locks it from the first second (the two agree on real recordings, `spectrogram.rs` comments); keep the board's rule and record the difference in
the cells it makes.

### 4.4 The gate and the early emit
The old gate (`coarse_sync_gated_spec`) knows a spectrogram still being filled (`SpecFill`: rows past the fill are skipped in the hard-sync count, "an unmeasured row
reads as tone 0, a false hit wherever the Costas array has a 0"). The cells search of the library (`coarse_lib::search`, `band_best`) takes a **complete** store. The
board emits pass 1 at row pair 87 of 92, before the slot's tail has arrived (`CORES3_FT8_SLOT_BUDGET.md`), so the cells search needs the same awareness: scores and
hard-sync counts over the rows measured, no row beyond the fill read. This is **new library work with its own precision guard** (step 0).
The search is band-count invariant by construction (the library's test: 1, 2, 4, 7 bands give the same list, bit for bit; the merge is a total order), which gives the
two-core split without a new rule: **core 0 scores the lower half of the carriers, the worker the upper half, the merge is the library's `merge` + `suppress`** (the
old coarse search splits the same way, head and tail). Scores: n_carriers x 60 B = 56 KB, PSRAM.

### 4.5 Stage 3: one candidate at a time
`decode_round` decodes a candidate at its grid place from the cells, then, if that fails and the estimate is a quarter cell off, once more at the interpolated
place. It is a **per-candidate step** and maps onto the firmware's work-stealing loop (group size 1): both cores pull candidates; each worker owns a `cs` staging
buffer (**the existing internal pair, `Owner::Ft8Scratch`, 5 056 B each**, reused: the cells fill writes the 79 x 8 block straight into it) and the 194-entry amplitude
and power tables (`static`, about 1.6 KB). BP scratch as today. **The library change** (step 0): `attempt` today allocates a `cs` `Box` and a `Vec` per attempt and
per candidate; on the board that is allocation churn in the heap whose fragmentation has already cost a mode (#260), and a 5 kB `Box` lands in PSRAM, paying PSRAM
latency in BP that `internal_pool.rs` was written to avoid. The flow takes its workspace from the caller.

### 4.6 Subtraction passes (optional, after 4.5 works)
`decode_block_cells_sic` and its `ResidualSource` read the audio forward and subtract each decoded signal as it goes, with a ring of 8 192 `f32` (**32 KB**) and a state of
about 64 B a signal (about 25 signals). Placement: the ring is read and written every sample of every signal's 79 symbols (about 120 000 samples x 25 signals a pass):
a PSRAM ring is a bandwidth question that **is not measured**; the alternatives are a ring of `i16` (16 KB) and an internal chunk buffer (640 samples = 2.5 KB) with only the
ring's far end in PSRAM. The pass runs **after the slot boundary** (it needs the whole slot and the pass takes seconds at host-instruction counts that have not been
converted to ESP32 time): what it adds are decodes for the screen and the next period, **not** earlier replies, which is where the budget document's deferred-candidate
analysis already puts late decodes. The second pass rebuilds the cells of the residual: it needs the audio buffer (PSRAM, 240 KB at 8 kHz) and the cells store reused, not
doubled. `passes` is the caller's parameter (the library API and `icap-profile --passes N`); the firmware's default is a decision for the owner (section 6), pending the
measured time.

### 4.7 Memory budget (to be confirmed by step 1; sizes are the library's)
| buffer | bytes | placement | note |
|---|---:|---|---|
| half-dB cell store | 172 224 | PSRAM | written by core 1's producer, read by scoring and stage 3 |
| scores of the scoring band | 56 000 | PSRAM | freed after the search |
| `cs` staging x 2 | 10 112 | internal (existing reservation) | reused as is |
| amplitude and power tables | about 1 560 | internal (`static`) | read in the hot loop |
| Hann half table | 2 560 | internal (`static`) | read per sample in the pack |
| 2 560-point scratch | 20 480 | internal (the existing block, resized only if no 3 840 user remains) | |
| SIC ring + 25 signal states | 32 768 + about 1 600 | PSRAM (ring), internal (states) | ring placement open (4.6) |
| 8 kHz audio buffers | 240 000 each | PSRAM | 360 000 today |
| u16 spectrogram, `allsum` head and tail | removed for FT8 | | about 1 MB of PSRAM freed |
Internal DRAM added: about 4-5 KB of `static` tables, against about 15 kB free at a slot boundary today; everything else new is in PSRAM or reuses a reservation. Taken
at boot before WiFi (`worker_arena`-style), not lazily.

## 5. Steps, gates and who does what

Each step ends with a measurement recorded in this directory and a commit; a step that fails its gate is reported, not worked around. Board steps need the owner to
flash and capture (`embedded-poc/CLAUDE.md`: logs only through `scripts/flash-monitor.sh`, never a second `espflash monitor`; a host check is not enough for a runtime-path
change).

0. **Library, host** (no board). **Status 2026-11-26: (a), (b), (c) done (commits `7d14c115`, `d0d39689`, `9f41fe0a`, design note section 25); (d) not done, see section 9.** (a) The workspace API: no allocation per call, the `cs` block, the tables and the cells store passed in; a per-candidate step
   (`decode_round` as a step). (b) `HalfDbCells` written a row at a time with a fill count, and the search over a partially filled store. Public items and what fails
   them: a store filled row by row equals the demux's cells (**fails if** a row lands in the wrong place or the fill count is off by one); the search of a complete store
   equals `coarse_sync_gated_cells`'s list bit for bit, and the search of a store filled to row F returns only candidates whose score and hard-sync count read rows < F
   (**fails if** a row past the fill is read: a signal placed after row F must not be found, a pre-F one must); the per-candidate step over a workspace decodes the
   same messages as `decode_block_cells` (**fails if** state leaks between candidates). (c) The 48 -> 8 kHz decimator and its host gate (4.1). (d) The host mirror
   (`ft8_embedded_pipeline_mirror`, the board pipeline on the host) run at the emit point with the cells flow, **OSD off**, over the budget document's phases
   (81 phases a point): the decodes in time against the old S3's on the same phases. Cost: record the instruction count of every stage as in section 21 of the design note.
1. **Board bench** (owner flashes). A 2 560-point esp-dsp wrapper, the byte demux and `half_db`, the Hann pack on core 1: time a row pair against the 80 ms cadence of the
   rows, the internal DRAM and PSRAM used, the free internal DRAM at the slot boundary against the 15.8 kB floor. Gate: the producer keeps up with the audio on core 1
   beside WiFi and the UAC reader (the old `stage1_inc` does today), and the floor of free internal DRAM is **not lower** than the old build's.
2. **Board, one core, wav replay** (`wav_sim`). The cells gate and stage 3 in place of the old gate and stage 3 for FT8; the same recordings as the old S3 (the 22 and
   `qso3_busy`). Gates: the decodes are at least the old S3's on the same replay (the host floors, 226 / 286 of `jt9`'s 402 at 15 / 30 candidates one pass, are the reference);
   the time of stage 3 a candidate and in all, recorded **next to the old S3's on the same board and the same replay** (this step produces the host-instruction to ESP32-time
   ratio for integer / table code, which no record has); free internal DRAM at the boundary not lower; no tlsf warning in 200 slots.
3. **Board, dual core.** The scoring split by carriers, the work-stealing loop over the per-candidate step. Gates: the same decodes as step 2 (the split changes nothing: the
   library's invariance), the time falls with the second core as the old stage 3's does, the interaction with WiFi on core 1 is recorded.
4. **Subtraction passes.** `ResidualSource` on the board, the ring placed by measurement (PSRAM against an internal chunk), `passes` from the app's settings. Gates: the
   time of a pass after the boundary, the free internal DRAM during it, the decodes added on the replays and (owner's call) on air; a pass must not delay the next slot's capture.
5. **Live.** Ten minutes of live IC-705 audio with WiFi on (the record of `UAC_BRINGUP_CORES3.md`), then on-air arms against the old S3 with the sample-size rule of
   `CORES3_FT8_SLOT_BUDGET.md` section 5 (a 45-slot arm resolves only about one decode a slot at 3 sigma: state the predicted effect before the arm). Tier C noise-only
   slots on the Linux box **before** the passes are released: the phantom rate of the cells flow and of the subtraction is the one precision number this plan does not have.

## 6. Decisions for the owner
* The firmware's default `passes` (1 reproduces the old S3's behaviour with no subtraction; 2 is the `ft8dec`-level recall at about 2.2 times the cells flow's instructions).
* OSD on the board: the cells flow's recall and cost without it are to be measured in step 0(d); the old S3 ships without.
* FT8 on 8 kHz only, other modes on 12 kHz (section 4.1), or a build option for either.
* Whether the old S3 path stays in the firmware beside the new one (a build feature) until step 5 has run, or is replaced at step 2.
* The release order: tier C results gate the release of the library items already pushed (5.1a / 5.1b / 5.2 / the cells flow / the passes).

## 7. Open questions (not established)
1. **Time**: the host instruction counts do not convert to ESP32 milliseconds for this code (the old stage 3 is f32 Goertzel on the FPU; the new one is integer and table
   code); step 2 measures the ratio. Until then no time figure for the new flow is claimed.
2. **PSRAM bandwidth** of the subtraction ring and of the scoring's row-wise reads of the 172 KB store against the S3's 32 KB data cache.
3. **The early emit**: how much recall the cells gate keeps on a partially filled store at row pair 87 (step 0(d) on the host; the old gate's own loss is in the budget document).
4. **Gain lock**: the board locks the gain from the first second; with 16-bit cells and a half-dB byte conversion a clipped or locked-low slot changes the cells (the old u16 cell
   shared the same gain), to be checked on the replay of a loud recording.
5. **OSD** on the board and its cost on core 0 beside the work-stealing loop.
6. The `fil4`-style design for 8 kHz (4.1) is a host task with a gate; its rolloff against the 200-3 000 Hz band edge and the noise bandwidth it sets is not designed here.
7. `snr_db` of the cells flows (not validated against WSJT-X's SNR; about 10 dB below `jt9`'s in the host runs, the old S3's flow as well) and the on-screen SNR the app shows.

## 8. Not in this plan
The ICAP radio-resident application (its audio is 8 kHz from the radio and its memory is the radio's), other modes, the embedded `fixed-point` decoders of other protocols,
the release itself (`docs/notes/RELEASE_PROCEDURE.md`), and the tier C sweep (the Linux box).

## 9. Status and handoff to the machine with the CoreS3 attached (2026-11-26)

**Done on the host (pushed):** the library pieces steps 1-3 need. Nothing here has run on a board.

| firmware need | library item (all `fixed-point`, `Hz8000`, no_std + alloc) |
|---|---|
| a candidate decoded over the firmware's own memory | `CellsWorker::new(staging, bp_scratch, tables)` / `decode_candidate` (`cells_flow.rs`); the 5 056 B internal pair is the staging block |
| the producer writes cells as audio arrives | `HalfDbCells::new`, `row_mut(row)`, `mark_filled(rows)` (`coarse_lib.rs`) |
| two-core scoring, one half of the carriers each | `search_cells(store, search, carriers)` per core, `merge_cells_parts(parts, search, rate)` on one |
| emit before the slot's tail | `candidate_ready(store, cand)`: decode the ready ones first, the rest after the fill completes |
| 48 kHz -> 8 kHz in the UAC reader | `LinearResamplerI16To8k::new(48_000)` / `process` (about 1.1 M MAC/s, ~135 taps); the 12 kHz resampler is unchanged |

**Next work, in this order**
1. Read `embedded-poc/CLAUDE.md` ("USB host VBUS on CoreS3", "Stacks, heaps, and the space between them"), this plan's sections 3-4, and `CORES3_FT8_SLOT_BUDGET.md`. Logs only through `embedded-poc/scripts/flash-monitor.sh`.
2. **Step 1, board bench** (needs the board, no library change): the 2 560-point esp-dsp wrapper beside the 3 840 one (`embedded-shared/src/esp_dsp_fft.rs`; `plan_forward(2560)` must stop panicking), the byte demux + `half_db` and the Hann pack on core 1,
   writing `HalfDbCells` with `row_mut` / `mark_filled`. Measure a row pair against the 80 ms cadence, the internal DRAM / PSRAM used, and the free internal DRAM at the slot boundary against the 15.8 kB floor (`worker_arena`-style reservation at boot, before WiFi).
   Use the library's `Plan2560Sc16` demux output as the reference: the board's cells must equal it row for row on the same audio (compare, do not assume).
3. Step 2 (wav replay, one core) then step 3 (dual core), as section 5. Step 2 yields the host-instruction -> ESP32-time ratio no record has; until then no time claim.
4. **Step 0d can run on either machine** (host measurements, no library item): `icap-profile --no-osd` for the cells flow and the passes next to the OSD-on ledger of design note 23b (decides section 6's OSD question); and `run_slot_s3_cells_early` (a store filled to row 2 x 87 and 2 x 85, `candidate_ready` decodes first, then the rest) against the full-slot flow and the old S3 on the same 60 slots (open question 3).
5. The `fil4`-style 8 kHz filter is built (section 7 item 6 is answered for the host: pass 3 200 / stop 4 000 Hz). Check on the board that the reader's 6:1 path does not cost the 48 kHz stereo -> mono step more time than the 12 kHz path did.

**Not done, and not to be assumed:** OSD-off numbers, the early-emit numbers, any ESP32 time, PSRAM bandwidth, tier C noise-only slots (the Linux box), `snr_db` calibration, an allocation-free `engine::llr`.
Release gate before the library items go out: tier C (section 6).

## 10. Board and ft8dec results (2026-11-26; logs `embedded-poc/m5stack-cores3-app/logs/ft8-cells-bench_step{1,1b,2b}_2026-11-26.log`, not committed)

**Step 1 (CoreS3, no WiFi / USB, one core, `ft8-cells-bench`).** `plan_forward(2560)` works. One 2 560-point FFT: esp-dsp 5.44 ms, software `Plan2560Sc16` 11.13 ms. The two are **not equal**: |esp - soft| up to 83 LSB, mean 3.1, at every signal level (mean 4.3-4.7 for |soft| 8-512 and above); the shipped 3 840 plan against `Plan3840Sc16` is as far apart (123 / 3.7). Through the producer 22 115 of 172 224 cells differ (max 27 half-dB units). Cause not measured. Row pair (Hann pack, FFT, demux, `half_db`, `row_mut`): mean 6.98 ms, max 7.80 ms of 160 ms. The 172 224 B stores sit in PSRAM; internal DRAM free 319 387 -> 318 771 B (no WiFi, so **not** the slot-boundary floor).

**Step 2 (wav replay, one core, `decode_block_cells`, OSD off).** qso3_busy / qso1 / qso2: 9 / 5 / 5 decodes at 15 and at 30 candidates, the same sets as the host and each inside ft8dec's list (test `cells_flow_host_reference`). Time 2.25 / 2.28 / 2.36 s at 15 candidates, 3.06 / 2.94 / 3.02 s at 30. `resample_to_8k` took 6.9 s here: that is the one-shot 12 -> 8 kHz helper, not the reader's 48 -> 8 kHz path (step 5 of section 9 is still open). No time claim against the old S3 (no streaming, no second core).

**Against ft8dec (icap-sdk 1ca9f04, host driver, same 8 kHz audio, 3 recordings), `scripts/compare-ft8dec-cells.sh`.** Decodes (sum of 3): ft8dec 30 candidates / OSD off: 19 / 21 / 22 at 1 / 2 / 3 passes, cells 19 (15 or 30 candidates) / 21 / 21; ft8dec's defaults (300 candidates, OSD, 3 passes) 26. M instructions (x86-64, WSL2, valgrind, sum of 3), **with the host build's AP pass on** (see the correction below): ft8dec 601 / 1 293 / 1 585 (defaults 2 101), cells 992 (15 cand.) / 1 531 (30) / 2 434 (sic 2) / 2 776 (sic 3). **With the AP pass off (the board's build): cells 720 (15 cand.) / 924 (30) / 1 691 (sic 2) / 1 992 (sic 3), i.e. 1.54x ft8dec at one pass (30 candidates), 1.31x at two passes, 1.26x at three.**
**This contradicts section 2's instruction counts (Apple Silicon, `/usr/bin/time -l`) in direction** (section 2: the cells flow takes fewer instructions than ft8dec at one pass). Instruction counts depend on the ISA, so the two are not comparable as they stand; whether the ratio flips on arm64 is open until `scripts/compare-ft8dec-cells.sh` is run on that machine (it measures with the host build's AP pass on; see below).

**Correction (2026-11-27): the host cells flow runs an AP pass the board does not have.** `PassCtx::FIRST` has `ap: true`, and with `fft-rustfft` (the host) every candidate that does not decode goes on to the blind-CQ AP pass (WSJT-X `iaptype` 1, #721: four LLR variants, a BP scratch of its own); the board's build (no `fft-rustfft`) has no such code. So the host figures above (instructions, BP calls, allocations) include work the ESP32 does not do: with AP off the cells flow's 30-candidate instructions drop from 1 531 M to 924 M (3 files) and the 38 `20m_busy` slots lose 6 hits and 8 extras (679 -> 673, 26 -> 18: what the AP pass adds there). **The earlier reading "BP is the gap" (cells 179 BP calls, 263 M, against ft8dec's 18 calls, 9 M, qso1 at 30 candidates) is withdrawn**: most of those calls were the AP pass's. The board measurements (below) are of the board's build and stand. What explains the board's 2.0-2.4x at one pass against the host's 1.54x (AP off) is not measured. Whether the host's `decode_block_cells` should run the AP pass at all is open: its docs say "no OSD / SIC / AP" for the board's driver, and `process_candidate_with_scratch`'s doc said it allocates nothing for a candidate that does not decode, which is false (three small LLR buffers per attempt, 696 / 64 / 32 bytes, and with `fft-rustfft` the AP pass's BP scratch); both docs are corrected.

**SIC on the CoreS3 as the library is, and `ft8dec` on the same board (2026-11-26; logs `ft8-cells-bench_s0_sic_*` and `ft8-cells-bench_ft8dec_*`, not committed).** `decode_block_cells_sic` (30 candidates, OSD off, one core, warm, from PSRAM): 2 passes 5.98 / 4.80 / 4.96 s, 3 passes 8.44 / 4.81 / 4.96 s on qso3_busy / qso1 / qso2 (the plain flow 3.06 / 2.94 / 3.02 s), the decoded sets the host's (11 / 5 / 5); about 10 % above the host instruction ratio (not explained). The 48 KB bench thread used about 21 KB of stack, the same as the plain flow. `ft8dec` (icap-sdk 1ca9f04, built as an ESP-IDF component for the Xtensa core, its work area, 234 360 B, in PSRAM: an internal 240 KB block was not available): 30 candidates / OSD off / 1, 2, 3 passes 1.56 / 3.88 / 6.01 s on qso3_busy (10 / 12 / 13 decodes), 1.43 / 2.87 / 2.83 s on qso1 (4 / 4 / 4), 1.27 / 2.91 / 2.87 s on qso2 (5 / 5 / 5); its defaults (300 candidates, OSD, 3 passes) 9.41 / 3.90 / 3.88 s (16 / 5 / 5). The decode counts equal the x86 host run of the same C. On the board the cells flow takes 1.96-2.37x `ft8dec`'s time at one pass and 1.54-1.70x at two (the x86 instruction ratios are 2.4-2.7x and 1.8-1.9x). Not measured: how much of `ft8dec`'s time is PSRAM against its SRAM design.

**One signal `ft8dec` decodes and the cells flow does not (qso3_busy, `N1PJT HB9CQK -10`, 465.4 Hz, DT 0.26).** It is a candidate (rank 13 of 30 at 30 candidates: 465.6 Hz, DT 0.30, hard sync 13 of 21), so the search is not what loses it. BP fails there at the grid place with either LLR effort; OSD rescues it. The place estimate points half a row (its cap) to the previous row, and reading **that** row as it is decodes with the cheapest LLR and no OSD; the retry's 50/50 blend of the two rows does not decode at any of 144 weights. `ft8dec` decodes it with OSD off, with `interp` off and with each of its three `llr` variants, so none of those explains it. A prototype that snaps the retry to the neighbour cell when a weight reaches 0.5 (then the old blend if that fails) was measured and **discarded**: 22 recordings, `jt9` references, 30 candidates: recall and extras identical in four configurations; the 38 slots of `ft8_lib`'s `test/wav/20m_busy` against their `.txt` (936 decodes): +1 hit without OSD (679 -> 680, extras 26), +2 with OSD, 0 with two SIC passes (a true decode lost, `CQ DL4SBF JN48`, with SIC + OSD), extras 26 -> 26 / 34 -> 35 / 47 -> 46 / 64 -> 67 (two of the added ones, `CQ SV2BRA KN10` and `<...> 9A3KG JN83`, are decoded by a local `jt9` too, `G3WAG SP4TXI 73` is not). With the host's AP pass on that is not worth a divergence from `ft8dec`'s and WSJT-X's flow (+0 to +2, not monotone under SIC). **Re-measured with the AP pass off (the board's build), 2026-11-27: +2 hits at one pass and +2 at two SIC passes on the 38 slots (673 -> 675, 804 -> 806; extras 18 and 35 unchanged; every added decode is in the `.txt`; nothing lost) and no change on the 22 recordings: monotone, still 0.3 %.**

**Corpus.** `kgoba/ft8_lib` at 9fec6ca3 (`test/wav/20m_busy`, 38 slots with a `.txt` each, 936 decodes) is not vendored; clone it for any evaluation of the 60 slots. A local `jt9` (not the beta6 of the design note: version unchecked) is at `~/src/WSJT-X/build_jt9/jt9`.

**Mac run, BP breakdown and the iteration cap (2026-11-27): design note section 26.** On the Mac the cells flow is 3.3x `ft8dec`'s instructions at 30 candidates / OSD off (4.4x its wall time): the same direction as x86. (Both are of the host build, which runs an AP pass the board does not: see the correction in section 10; the Mac's BP counts are the staircase's and hold for the board.) BP is 60 % of the time (17 % of the instructions); the gap is the number of failed candidates sent to BP (68 against 16) rather than the cost of a call. A BP cap of 25 costs no recall on 60 slots and saves 2-3 % (not applied). For the board: predict time from BP iterations, and consider a 12-slot `20m_busy` replay (section 26, "On the board").

**SIC on the CoreS3, library side (S1, 2026-11-27).** Step 1 (`CellsWorker::decode_round`, one worker for every pass) and step 2 (`search_near_into`: the residual's cells are written into the first pass's `HalfDbCells`) are in, both bit-identical to what they replace (22 recordings: 286/402 at one pass, 332/402 at two, 335/402 at three; the three recordings; `search_near_into_a_used_store_equals_search_near`). Step 3 (2026-11-27) is in too: `sic_passes` runs the passes over a `SicStorage` (ring 32 KB, chunk buffers, symbol table, states) made once and a `CellsWorker`, from the first pass's candidates and cells; `decode_block_cells_sic` is its allocating front end; four slots through one storage equal fresh memory to the bit. **S1 is done.** The storage's placement is the allocator's (the 32 KB ring is above the board's 2 KB threshold, so PSRAM unless routed): S2 measures it and, if the ring has to be in internal DRAM, `SicScratch` gets a constructor over the firmware's own slices. Not allocation-free: the LLR step makes three small buffers per attempt, and `band_best`'s scores (about 56 KB) are made per search. With `--features fixed-point` the whole-crate test run has 24 failures (the 12 kHz resampler's golden hashes, `decode_snapshot`, `ft8_real_recordings_vs_jt9`, ...), the same 24 before this change: that is not the merge gate's configuration, and they are not looked into here.

**SIC on the CoreS3, S2 (2026-11-27; `ft8-sic-bench`, one core, 30 candidates, OSD off, wav replay from PSRAM, no WiFi / USB host; log `ft8-sic-bench_s2_2026-11-27.log`, not committed).** Three arms, each run twice (the second read), the search of the cells and then the passes: `alloc` (`decode_block_cells_sic`), `storage` (`sic_passes` over a `SicStorage` made once, the `cs` staging the firmware's internal-DRAM one, a worker made once) and `ring-int` (the whole `SicScratch`, 37 888 B, one internal-DRAM block through `SicScratch::new`). qso3_busy / qso1 / qso2 at 2 passes: `alloc` 6.01 / 4.83 / 4.99 s, `storage` 5.99 / 4.77 / 4.97 s, `ring-int` 5.94 / 4.76 / 4.92 s; qso3_busy at 3 passes 8.48 / 8.48 / 8.34 s. **The decoded set of every arm is `alloc`'s** (11 / 5 / 5). The ring in internal DRAM against PSRAM is 0.8-1.7 % faster: **the ring's placement is not what makes the board 10 % dearer than the host instruction ratio** (S0), and a PSRAM ring is fine. Of a run, the search of the cells (the demux's FFTs and `band_best`) is about 1.8 s; the passes are 4.19 s (qso3_busy, 2 passes), 2.98 s (qso1), 3.18 s (qso2). `SicStorage` takes 2 568 B of internal DRAM and 38 476 B of PSRAM. Heap minima since boot (cumulative over the arms: floors, not per-arm peaks): internal 203 143 B from 258 151 B (this includes the 37.9 KB block and the plain flow's BP), PSRAM 6 761 540 B from 8 369 768 B; the 48 KB thread had 32.5 KB of stack free. **Not measured and not claimable:** SIC's own transient internal DRAM (the dip is shared with the plain flow's BP), anything under WiFi and the USB host (the 15.8 kB floor), the running firmware (it still runs the old S3 flow: the cells flow's producer, wav replay and dual core, steps 1-3 of section 5, are not in the app), and the search's split between the demux and `band_best`.


## 11. Instructions after the producer: the old S3 against the cells flow, on the host (2026-11-27)

`scripts/s3-stage-instr.sh` (callgrind, fixed-point, **OSD off, no AP pass**: the board's build; inclusive cost of named functions, M instructions a slot; `icap-profile --no-osd` is new: the S3 flows hardcoded OSD on). Old S3 = `--s3` at 12 kHz (`sync8` search on the spectrogram, fine sync, refine, Goertzel stage 3); cells = `--s3 --native --cells`. qso3_busy / qso1 / qso2.

"Producer" is what the board does while the slot is captured (old: the spectrogram, 152 M; cells: the 2 560-point FFTs of the demux, 88 M). **After the producer** is the rest:

| M instr after the producer | qso3_busy | qso1 | qso2 |
|---|---|---|---|
| old S3, 15 cand | 122 | 122 | 123 |
| cells, 15 cand | 136 | 142 | 151 |
| old S3, 30 cand | 224 | 221 | 202 |
| cells, 30 cand | 224 | 198 | 215 |
| cells + 2 SIC passes, 30 cand | 540 | 396 | 445 |

Decodes (OSD off): old S3 10 / 4 / 5 at 15 and 30 candidates, cells 9 / 4 / 5, 2 passes 11 / 4 / 5, 3 passes 12 / 4 / 5.

* **The cells flow is cheaper in total (224 against 274 M at 15 candidates) only because less of it is after the producer's share: after the producer it is not cheaper.** What moved to the producer is smaller (88 M against 152 M) and what stays is larger: `band_best` 64 M against the old scoring's 41 M (old: search 193 − spectrogram 152), stage 3 the same size at 30 candidates (cells 144 M, BP 139 M; old 143 M, Goertzel 82 M, BP 66 M).
* **The board's time follows the instructions at about 10 ns per M** (cells at 30 candidates: 224 M, measured 3.06 s − 0.94 s for the software FFT the wav replay runs and the app would not = 2.1 s; 2 SIC passes: 540 M, measured 5.99 − 0.94 = 5.05 s; old S3 at 15 candidates: 122 M, `CORES3_FT8_SLOT_BUDGET.md`'s coarse 103 ms + 15 x 72 ms = 1.18 s). Predicted after the producer: old 15 cand 1.1-1.3 s; cells 15 cand 1.2-1.5 s; cells 30 cand 2.0-2.4 s; 2 SIC passes 4.8-5.8 s. Against the old S3's own window (810 ms before the boundary + 500 ms after = 1.31 s): the cells flow at 15 candidates is at the edge, at 30 over by 0.7-1.1 s, SIC cannot be inside it at any setting.
* **Not established.** (1) The host harness' search uses the default lag window; the board's old coarse uses 27 lags (103 ms measured, `CORES3_FT8_SLOT_BUDGET.md`) against about 0.4 s predicted from the host's 41 M: the instruction-to-time ratio is not transferable to that stage until the harness uses the board's window, and the cells search has a fixed 60 positions (`NJ`) of its own. (2) `band_best` alone is not timed on the board (the S2 figure of 1.79 s is FFT + `band_best` + the rest, soft FFT). (3) `--s3` 12 kHz on the host against `coarse_sync_gated_spec` on a filling spectrogram (the board's path): not compared. (4) Recall of the old flow on the 22 recordings against `jt9` at this build: only the three recordings above.

## 12. The cells flow stage by stage on the CoreS3 (2026-11-27; `ft8-stage-bench`; logs `embedded-poc/m5stack-cores3-app/logs/ft8-stage-bench_2026-11-27.log` and `..._band_2026-11-27.log`, not committed; predictions are in the bench's module doc, committed before the run: `925ce5ef`, `a2a5d8ea`)

One core, wav replay from PSRAM, no WiFi / USB host, 30 candidates, OSD off, `qso3_busy` / `qso1` / `qso2`, the soft `Plan2560Sc16` in the demux (the app would use esp-dsp).

| | predicted | measured |
|---|---|---|
| `search` (demux + `band_best`) | 1.79 s (S2) | 1.838 / 1.796 / 1.836 s |
| `band` (`search_cells` over every carrier + merge) | 0.56-0.68 s | **0.906 / 0.941 / 0.902 s** (33-40 % over: the stop rule) |
| demux (`search` - `band`) | 1.1-1.2 s | 0.932 / 0.855 / 0.934 s |
| stage 3, floor 0 (the whole staircase) | 1.27 / 1.11 / 1.23 s | 1.477 / 1.311 / 1.431 s |
| stage 3, floor 16 (`CELLS_D_MIN_SCORE`, #765) | 0.95 / 0.67 / 0.74 s | **0.927 / 0.708 / 0.742 s** |
| BP passes, floor 0 / 16 (host numbers) | 2 145 / 1 360, 1 781 / 899, 2 021 / 1 012 | 1 987 / 1 240, 1 838 / 951, 1 947 / 995 (the board's LLRs differ slightly) |
| one BP pass | 0.57-0.70 ms | **0.69-0.75 ms** (floor 16: 0.690 / 0.687 / 0.706) |
| decodes | 9 / 4 / 5 | 9 / 4 / 5 at both floors |

* **The floor of #765 saves 0.55 / 0.60 / 0.69 s of stage 3 on the board (-37 / -46 / -48 %)**, more than the 0.32-0.39 s predicted (the floor-0 time was 16-18 % above its prediction: not explained).
* **One BP pass was about 0.7 ms**: 76 000 host instructions (same-run figure; 65 000 was a division across two runs) at about 9 ns each. The BP pass of the cells flow was 1.57x `ft8dec`'s in instructions (76 000 against 48 500, design note section 27 correction). After the flat edge tables (section 12a) it is 37 600 host instructions and 0.26-0.44 ms on the board.
* **`band_best` is the cost after the producer's share, and it does not fit the budget**: 0.90-0.94 s one core against the 500 ms of post-boundary compute (the old S3 uses about 270 ms; its coarse search is 103 ms). 14 ns per host instruction, against 9-10 for the other stages.
* **Why: CPU-bound, not memory-bound** (hypotheses written first, `band narrow` lines). `search_cells` over four ranges of 64 carriers (the cells of a range fit the data cache) costs **16.7 us a position**, over 256 carriers 16.7-17.3 us, over the whole band 16.8 us: the same within 3 % (memory-bound would have been at most 55 %). Host: `band_best` 64 M instructions, 14.0 M conditional branches (22 % of the instructions), 0.49 M mispredicts, about 1 200 instructions and 260 branches a position; the board needs about 4 000 cycles a position. The release profile is already `opt-level = 3`. The loop (`coarse_lib::score`) has per symbol three row tests, four neighbour tests and four bounds-checked loads, which `ft8dec`'s C does not pay (its `find_candidates`, 62 M on the host, is inside a 1.3-1.6 s whole decode on the board).
* **Not examined**: the instruction count of `score` on Xtensa (no disassembly looked at), so "bounds checks and branches" is a reading of the source, not a measurement.

**Ways to bring the search under the budget (estimates; (A) was built the same day, section 12a):** (A) a restructured `score`: per carrier, precompute for every row and Costas symbol the exact neighbour sum and count (`w[r][k]`, `n[r][k]`, with the same row-edge rules, so the integer result and the truncating division are the same), then each of the 60 lags is 21 lookups: about 1.75x fewer operations (an estimate from operation counts: 0.9 s -> about 0.5 s); (B) the two-core split already in the library (`search_cells` per half of the carriers, `merge_cells_parts`): about 2x; (C) **score incrementally in the producer**: when row `r + 2` arrives, the contributions of row `r` are added to the 21 lag accumulators of each carrier (about 240 operations a carrier a row, about 0.5 ms a row pair against an 80 ms cadence), so that at the emit point only the selection and the hard sync of the survivors are left; bit-identical to `search_cells` over the rows filled so far if the two last rows are added with the edge rule at the finalisation (a test over every fill point would pin it). (C) removes the search from the post-boundary path altogether and is the plan's section 4.4; (A) and (B) shorten it.

## 12a. After the changes of the day (2026-11-27; logs `ft8-stage-bench_regroup_`, `_bpflat_`, `_bpconst_2026-11-27.log` in `embedded-poc/m5stack-cores3-app/logs/`, git-ignored; the predictions are in the bench's module doc)

One core, 30 candidates, OSD off, the `d` floor 16, `qso3_busy` / `qso1` / `qso2`; the decodes (9 / 4 / 5) and the BP passes (1 240 / 951 / 995) are the same in every run.

| | section 12 | regrouped scoring (`f17f1a82`) | + flat-table BP, heap tables (`86308daa`) | + compile-time tables (this) |
|---|---|---|---|---|
| `band` | 0.906 / 0.941 / 0.902 s | **0.460 / 0.456 / 0.455 s** | same | same |
| stage 3, floor 16 | 0.927 / 0.708 / 0.742 s | same | 0.590 / 0.460 / 0.480 s | **0.623 / 0.484 / 0.464 s** |
| one BP pass (host instructions) | 76 000 | 76 000 | 39 900 | 37 600 |
| min free internal DRAM (qso2) | 251 647 B | 251 647 B | 247 059 B | 251 647 B |

The scoring: `band_best` 63.8 -> 42.4 M host instructions, 14.0 -> 5.6 M conditional branches, bit for bit the same scores. The tables of the BP in the scratch cost 4.6 KB of internal DRAM on this board (every table under the 2 KB threshold goes to internal), which the compile-time version does not.

**The trade of the compile-time tables** (the logs `_bpflat_` and `_bpconst_`): against the first version's heap tables the const ones are about **6 % slower per BP pass on the board** (floor 0: 0.382 / 0.383 / 0.388 -> 0.408 / 0.409 / 0.414 ms; stage 3 at floor 0 0.832 / 0.754 / 0.799 -> 0.884 / 0.802 / 0.850 s; at floor 16 +5 % / +5 % / -3 %), for **4.6 KB of internal DRAM kept free** and no state in the scratch. The cause is a hypothesis, not measured: the tables are flash rodata read through the cache, where the heap tables sat in internal DRAM. Host instructions a pass: 76 000 -> 39 900 (heap tables) -> 37 600 (const), `scripts/bp-instr-per-pass.sh`.


## 13. Does the cells flow fit the window? The timeline from the measured numbers (2026-11-27; an estimate, one core, nothing else running)

**A correction first.** Section 12 compared `band` (0.90 s as measured there; 0.46 s after the regrouped scoring, section 12a) with the 500 ms of post-boundary compute and said the search had to be shortened to fit. That put the search **after** the boundary; the design (section 4.4, `CORES3_FT8_SLOT_BUDGET.md`) runs it at the **emit point, pair 87 of 92, which leaves 810 ms before the boundary**, and decodes there every candidate whose rows have arrived (`candidate_ready`); only the late ones wait for the slot's end. The estimate below follows that design.

Inputs (board, `ft8-stage-bench` after the `d` floor, the regrouped scoring and the compile-time-table BP; log `..._bpconst_2026-11-27.log`; the same estimate from the heap-table run `..._bpflat_` is 0.03-0.04 s earlier): `band` 0.456-0.460 s over the 174 rows of the emit point (one core); stage 3 a candidate as logged (`#i ... us`); which candidates are ready at 174 rows: `j0 + 157 < 174` with `j0 = round((dt + 0.5) / 0.08)` from the candidates' DT (host list of the same slots); the first 15 or 30 in the search's order (hard sync).

| | candidates ready at the emit point | 15 candidates: ready / late (stage 3) | 30 candidates: ready / late |
|---|---|---|---|
| `qso3_busy` | 27 of 30 | 0.297 s / 0.000 s | 0.559 s / 0.062 s |
| `qso1` | 18 of 30 | 0.117 s / 0.134 s | 0.265 s / 0.218 s |
| `qso2` | 19 of 30 | 0.141 s / 0.103 s | 0.263 s / 0.200 s |

Timeline (one core, seconds relative to the boundary; the emit point is at -0.81): the search ends at -0.81 + 0.46 = **-0.35**; the ready candidates are decoded from there: with 15 candidates all of them end **before the boundary** (0.28 s of 0.35 s at most); the late ones follow after it, so the last decode is at **+0.00 / +0.13 / +0.10 s**. With 30 candidates `qso3_busy` runs 0.21 s past the boundary on its ready ones and its last decode is at **+0.27 s** (`qso1` +0.22, `qso2` +0.20). **All inside the 500 ms after the boundary** (the old S3 uses about 270 ms), and the shipped 15 candidates are the cheap ones (ranks 0-14 decode in few passes: 15 candidates cost 0.12-0.30 s, 30 cost 0.38-0.62 s).

So **one core is enough for the window; the two-core split is not needed to fit it**: it would move the end of the last decode earlier (the search 0.23 s, stage 3 halved: everything before the boundary), a margin, not a requirement.

**What this does not establish:** (1) the firmware's other load: the UAC reader and `USB UAC Host` (priority 6), WiFi, the display and the producer on core 1 (about 7 ms of each 160 ms): the bench ran alone on one core; the old S3 had the same neighbours and used 270 ms of its 500; (2) the search at the emit point over 174 rows was timed over 184 (a little cheaper in the app); (3) the producer, the 48 kHz to 8 kHz reader and the early emit are not in the app (section 5 steps 1-3): the cells flow's own timeline is this estimate, not a measurement of the app; (4) the soft FFT in the bench's demux (0.93 s) is not on this path (the producer does it, with esp-dsp 5.44 ms an FFT); (5) subtraction passes (SIC) are after the boundary and take seconds: not in this window.

**Decisions for the owner that this fixes the numbers of:** the candidate count (15 as shipped, or 30: the first needs 0.12-0.30 s of stage 3, the second 0.38-0.62 s; ranks 16-25 convert at about 0.1 % on the old flow, `CORES3_FT8_SLOT_BUDGET.md` section 4, not re-measured on this one); the emit point (87); whether the app gets the cells flow (section 5 steps 1-3 are the work).

## 14. Is there room for SIC? The board after the day's speed-ups (2026-11-27; `ft8-sic-bench`, log `ft8-sic-bench_after_2026-11-27.log`, git-ignored; predictions committed first, `f0267c3b`)

One core, 30 candidates, OSD off, the `d` floor 16, wav replay from PSRAM, no WiFi / USB host, `storage` arm (`sic_passes` over a `SicStorage`), `qso3_busy` / `qso1` / `qso2`; the decoded set of every arm is `alloc`'s (11 / 5 / 5).

| | S2 (morning) | predicted | measured |
|---|---|---|---|
| 2 passes, total | 5.99 / 4.77 / 4.97 s | 4.4 (4.0-4.8) / 3.9 (3.6-4.2) / 4.05 (3.7-4.4) s | **4.24 / 3.43 / 3.49 s** |
| 3 passes, total (`qso3_busy`; the others stop at the second pass) | 8.44 s | 6.8 (6.3-7.3) s | **6.53 s** |
| 2 passes: the first search / the passes | 1.79 / 4.2, 3.0, 3.2 s | | 1.32 / 2.91, 2.11, 2.17 s |
| 2 passes, `2core` arm | | | 3.96 / 3.28 / 3.27 s (4-8 % under `storage`) |

(`qso1` and `qso2` came in 5-6 % under their predicted ranges; everything inside 25 %.)

**Room, by the clock.** In the app the first search (1.32 s: the demux and `band`) is the producer's and the emit point's; what SIC adds is the passes, **2.1-2.9 s for two passes** (5.2 s for three on the busiest slot). The primary decode ends at +0.1 to +0.25 s after the boundary (section 13), so two passes end at about +3 s, three at about +5.5 s; the next period's emit point is at about +14 s: **roughly 9-11 s of slack**. SIC is after the boundary by design (plan 4.6): its decodes are for the screen and the next period, not for the reply.

**Conditions, apart from time:** the slot's 8 kHz audio (240 KB of PSRAM) must be kept for the subtraction to read again (the app keeps the cells, not the audio); `SicStorage` is 2.6 KB of internal DRAM and 38.5 KB of PSRAM, the ring may stay in PSRAM (internal is only 1-3 % faster); SIC runs on one core (the second is the next slot's producer; the `2core` arm saves 4-8 %).

**Not measured:** the firmware's load (WiFi, USB host, display, the UAC reader: a decode thread preempted for 3-5 s is slower by what it loses); the phantom rate of the passes on noise-only slots (tier C: the two new decodes of the two-pass flow on the 22 recordings that read like subtraction ghosts, section 2); esp-dsp in the residual's demux (about half of the 0.9 s of FFT in each pass; the bench uses the soft FFT).
