# Profiles: which implementation, OSD and SIC options for which class of hardware

**What this is.** A design note on the *variants* to build: for four classes of hardware, which decoder flow, which OSD option and which SIC option each gets, and what of that exists today.
It is **not** a performance claim. Only the ESP32-S3 (CoreS3) has been measured (`ESP32S3_TWO_CORE_BEHAVIOUR.md`, `FT8_WINDOW_AND_COARSE_DESIGN.md` sections 28-30, `S3_CELLS_IMPLEMENTATION_PLAN.md`);
there is no Cortex-A9, Raspberry Pi 3B or Pi 4+ here, so no figure below for those classes is a measurement, and none should be quoted as one. Where a cell says "not built" or "to measure", that is the work this note
exists to plan. Normative rules stay in `CLAUDE.md`; WSJT-X remains the reference (principle 1): every divergence below is an *option*, off by default, marked in code and in Issue #766.

## 1. The profiles

The class names are the owner's. The descriptions are the public, approximate characteristics of each class (to be replaced by measurements of the actual boards); what matters for the design is the *kind* of constraint.

| | P0 small ESP32 | P1 Cortex-A9 | P2 Raspberry Pi 3B class | P3 Raspberry Pi 4 and later |
|---|---|---|---|---|
| core | Xtensa LX7 (S3) / LX6 (classic ESP32), 240 MHz, two cores, single-precision FPU, no OS threads beyond FreeRTOS | ARMv7, about 1 GHz, 1-2 cores, VFP / NEON, Linux | Cortex-A53, 4 cores, about 1.2 GHz, AArch64 NEON, Linux | Cortex-A72 / A76, 4 cores, 1.5-2.4 GHz, AArch64 NEON, Linux |
| memory | 512 KB SRAM (about 280 KB usable on the classic ESP32), 8 MB PSRAM: **the binding constraint** (internal-DRAM floor of the running firmware 15.8 KB; a PSRAM miss is 1.1 us and two cores do not get two) | hundreds of MB; caches 32 KB + L2 | 1 GB | 1-8 GB |
| deadline | the radio's reply: the decode must end by the boundary + 0.5 s to answer in the next slot | typically a receiver / logger: no reply deadline, or an auto-QSO host with one | receiver / skimmer; an operator console with a deadline is possible | skimmer, multi-band, heavy modes; operator console |
| today | **built and measured** (cells flow, OSD options, two cores) | nothing built for it; the host library builds for it in principle (unverified) | the same | the same; `apps/skimmer` runs on desktop-class hosts |

P0 is the CoreS3 (the main embedded target). The classic ESP32 (LX6, Core2) is P0 with less memory and no measurement of the cells flow: **P0-lite** below.

## 2. The axes along which an implementation varies

Every one of these is a knob that already exists, or would be one. A profile is a choice of value on each.

| axis | values (existing -> to build) |
|---|---|
| flow | **cells flow** (8 kHz, fixed-point, half-dB byte cells from the producer: `decode_block_cells[_sic]`, `CellsWorker`) / **faithful pipeline** (12 kHz, f32, `Decoder<Ft8>` through `engine::pipeline`) / the old S3 flow (12 kHz, `sync8`, Goertzel: **to be replaced wholesale by the cells flow**, section 9) |
| numeric path | `fixed-point` (`nstep-half`, LLR in f32 or `fixed-point-llr`) / f32 with `fft-rustfft` |
| FFT backend | esp-dsp (`fft-extern`) / rustfft (scalar on armv7, NEON on aarch64: **to verify**) |
| candidates | 15 / 30 (cells) / the WSJT-X count (faithful) |
| pre-BP gate | none / `CELLS_GATE` (OSD off) / `CELLS_GATE_OSD` (OSD on); fitted on generated sets, real slots held out |
| variant `d` floor | `CELLS_D_MIN_SCORE` (16) / none |
| OSD | off / **faithful** (`osd174_91`, ten runs a candidate) / **embedded** (`osd174_91_into`, bit-equal, caller's memory) x seeds (`OsdSeeds::Faithful`, `AD`) x timing (`OsdMode::Inline`, `Defer` + `osd_pass(k)`) |
| OSD parallelism | one worker / two workers (`take_osd_attempts`, `rank_osd_attempts`, `run_osd_attempt`) / **a thread pool: not built** |
| SIC | none / `sic_passes` N passes / `sic_passes_osd` (the OSD's decodes subtracted for the next pass) / the faithful pipeline's own SIC strategy (non-default, `CLAUDE.md` section 4) |
| AP | off / blind CQ + QSO context (host f32 only: **not in the fixed-point board build**) |
| threading | one core / two (round and OSD pass dealt out) / rayon |
| modes | the ten modes of `CLAUDE.md` section 1; MSK144 and JTTY sit outside the `Protocol` trait |

## 3. The assignment

`built` = in the tree and tested on the host; `S3` = measured on the CoreS3; `to build` = does not exist; `to measure` = exists, no figure for this class.

| axis | P0 small ESP32 (S3) | P0-lite (classic ESP32) | P1 Cortex-A9 | P2 Pi 3B class | P3 Pi 4+ |
|---|---|---|---|---|---|
| flow | cells flow (built, S3) | cells flow, fewer candidates (built; to measure on LX6) | faithful pipeline as the default; cells flow as the low-latency option (built on the host; **cells flow needs `fixed-point` and has never been the Linux target: decision D1**) | as P1 | faithful pipeline; cells flow not needed |
| numeric / FFT | `fixed-point`, esp-dsp | same | f32, rustfft scalar (armv7: to verify) | f32, rustfft NEON | same |
| candidates | 30 (15 when the budget is short) | 15 | WSJT-X's | WSJT-X's | WSJT-X's, deeper depth available |
| pre-BP gate | `CELLS_GATE` / `_OSD` (built, S3) | `CELLS_GATE` | not used (the faithful pipeline has no gate) | not used | not used |
| OSD flavour | embedded, seeds `AD` (built, S3) | embedded, seeds `AD`, K small or off | faithful `osd174_91` (built) | faithful | faithful |
| OSD timing | deferred, capped K (**K = 6 on one core is the default**; two cores are optional, for latency margin and not for recall; built, S3) | deferred, K = 2-4 or off | inline (the WSJT-X order; built) | inline | inline |
| OSD parallelism | one / two workers (built, S3) | one | one worker or a pool (**to build**) | a pool (**to build**) | a pool |
| SIC | none in the window; after it `sic_passes_osd` (built, S3) if PSRAM holds a second cell store and the previous audio (344 KB + 240 KB, plan section 14: **not decided**) | none | the faithful SIC strategy, or `sic_passes_osd` over the cells flow (built on the host) | same, more passes | same |
| AP | off (not in the board build) | off | on (built) | on | on, with QSO context |
| threads | 1 or 2 (round and OSD pass dealt out; built, S3) | 1 | 1-2 | 4 (rayon: to build for the cells flow, built for the faithful pipeline) | all |
| modes | FT8, FT4, JTTY (WSPR bench only) | FT8 | FT8, FT4, WSPR, JT65 | + FST4, Q65 | all, including MSK144 in real time and the long-period modes |
| role | operator terminal with a reply deadline | simple monitor / demo | receiver, logger, auto-QSO host | receiver, small skimmer | multi-band skimmer (`apps/skimmer`), operator console |

Reading the matrix:

- **P0 is a different decoder, not a smaller one.** It buys its fit by changes that are WSJT-X divergences (the cells flow's search and stage 3, the `d` floor, the gate, `OsdSeeds::AD`, the capped OSD pass). Each is an option measured against the real slots (decodes **and** phantoms), not a default.
- **P1-P3 should not need the divergences.** Their budget lets the faithful pipeline run as WSJT-X does; the OSD options there are the faithful ones and inline. The embedded OSD (`osd174_91_into`) is still the better *implementation* for them (bit-equal, no heap), and may replace the host port where a host measurement agrees (an open item in section 5).
- **SIC and OSD are independent options on every profile**, with one measured interaction: the OSD's decodes are worth subtracting (design note section 29: of the 71 decodes the OSD adds on the 60 real slots, K = 6 a round in the SIC keeps 50, one pass after the SIC 29). On P0 that makes the SIC a post-window path whose results feed the auto-QSO's waiting queue, not a reply-deadline path.

## 4. The OSD and SIC option sets, per profile

What each option is for, and where it belongs. Decodes below are the host, 60 real slots, 30 candidates (truth = jt9), as in design note sections 28-29 and the ft8dec comparison of 2026-11-27; they are the *same* on every profile, because the options are bit-defined: a profile changes how long they take, not what they decode.

| option | what it does | decodes (real 60) | for |
|---|---|---|---|
| OSD off | BP only | 974 (1 pass) / 1 121 (SIC 2 passes) | P0-lite; any profile with no time left |
| OSD inline, faithful seeds | `ft8b.f90`'s ten runs a candidate, at once | 1 024 / 1 192 | P1-P3 default |
| OSD inline, seeds `AD` | `a1 a2 d1 d2` (the other six never returned on 4 780 slots) | identical to the above on all 143 400 candidates | the cheaper equivalent; candidate for P1-P3 as well once reviewed |
| OSD deferred, cap K | BP first, then the K best-scored attempts of the slot (`OsdRank`) | K = 6: 1 007; K = 10: 1 018 (1 pass) | **P0**: the window decides K |
| OSD deferred, K split over two workers | the same, the attempts dealt out | the same decodes | P0 on two cores, P2-P3 with a pool |
| SIC off | one pass | | P0 inside the window; P0-lite |
| SIC N passes, OSD off | `sic_passes` | 1 121 (2) / 1 130 (3) | P1-P3 as a cheap SIC |
| SIC N passes, OSD per round capped K | `sic_passes_osd` | K = 6: 1 171 (2) / 1 183 (3) | P0 after the window |
| SIC N passes, OSD per round at once | | 1 192 (2) / 1 206 (3) | P1-P3 |

(ft8dec, 30 candidates, OSD, 2 / 3 passes: 1 036 / 1 053; its default, 300 candidates, 3 passes: 1 204.)

## 5. What to build, and in what order

None of this is begun unless listed under "exists".

1. **Profile presets as data, not code paths.** *(Exists for the faithful pipeline, 2026-11-28: `mfsk_core::decoder::Profile` `P1` = `Depth::Normal`, `P2` = `Depth::Deep`, AP on, 2 / 4 lanes, a slot budget of 0.8 / 0.9 of the period; `--profile` of `skimmer` and `mfsk-serve`; tier B `tests/profile_conformance.rs`. **Extrapolated from the host's time (section 8), not measured on an A9 or a Pi**; the cells flow's presets are `CellsOptions::p0_window()` / `p0_late()`, the hybrid of `PI3_JT9_GAP.md` sections 15-16 has none yet.)* A `Profile` is a named set of values on the axes of section 2 (flow, candidates, gate, OSD flavour / timing / K, SIC passes, AP, threads, modes). Where it lives is **decision D2**: in `mfsk-core` as documented constants next to the flows (so that `mfsk-ffi` and the bindings can name a profile), or in each application. Whatever the choice, a preset must be expressible with the options that exist; **a preset that needs a new code path is a sign the option is missing**.
2. **A conformance test per profile** (tier B, vendored corpus): the preset's decodes meet a recall floor **and** a phantom ceiling against jt9 on the real slots, through `assert_golden`. The P0 preset already has its numbers (above); the P1-P3 presets would pin the faithful pipeline's.
3. **OSD on a thread pool** (P2, P3; P1 with two threads): the attempts of `take_osd_attempts` / `rank_osd_attempts` / `run_osd_attempt` already separate *which* attempts run from *where*; a rayon `par_iter` over the ranked list is the whole change. Test: the decodes equal the one-worker pass (the two-worker test of `tests/cells_worker_reuse.rs`, with threads).
4. **The cells flow on Linux (D1).** It runs on the host today (`icap-cands`, `icap-sic`, `ft8-*-bench` are the embedded twins) but is `fixed-point`-only and its search is the board's. Whether P1 / P2 want it as the low-latency option (a reply-deadline host on a Cortex-A) or the faithful pipeline alone is enough is the question; the numbers that decide it do not exist yet.
5. **An armv7 build** (`armv7-unknown-linux-gnueabihf`) of the host library and of `icap-app`, run under QEMU for equality (tier A) before any real board: **to verify**, including which FFT path `rustfft` takes there.
6. **AP and OSD in the fixed-point build**: the faithful ladder's AP (step 4) is still host f32 only, but the cells flow has its own QSO-context AP rung (`ApContext`, 2026-11-28, `PI3_JT9_GAP.md` section 24: masked NMS BP and OSD on the candidates near the partner; not on the board yet, no blind-CQ pass). P0 without a context decodes as before.
7. **Replacing the host's `osd174_91` by `osd174_91_into`** in the faithful pipeline: bit-equal on 22 000 inputs; the host's instructions are 7 % *higher* with `u32` rows (x86-64), so it is a P0 gain and a possible P1 / P2 loss: measure on the hardware before deciding (an `u64` build for 64-bit targets is the alternative).

## 6. When hardware is available: what to measure first

The discipline of `AGENT_WORKFLOW.md` holds: host first, predictions written before a board run, the stop rule (25 % off prediction). Per profile, in this order:

1. **One core, one recording**: the stage times of the flow (search, band, round, OSD attempt, one SIC pass) for the three vendored recordings. The embedded benches (`ft8-stage-bench`, `ft8-osd-bench`, `ft8-sic-bench`) have Linux twins to write (`icap-profile`, `icap-sic` exist for the host).
2. **Two cores against one, the same inputs**: on the ESP32-S3 two cores cost each about 12 % of the work plus 2-3 cycles an iteration, PSRAM / flash misses are served in turn, and the 32 KB data cache is shared; **none of this is assumed to carry over** to a Cortex-A's cache hierarchy. Measure the one-core baseline; never infer it (the old S3 note's "single-core work" was twice the wall time).
3. **The option sets of section 4** on the 60 real slots: the decodes are the host's (bit-defined), so the run is for time and for a check that a different FFT / FPU did not change a decode.
4. **Memory**: resident set of the flow, the cell store / spectrogram, the SIC's second store.

## 7. Open decisions (owner)

- **D1** whether the cells flow is a Linux profile at all.
- **D2** where profile presets live (library constants or application configuration). **Decided 2026-11-27: the library.** `CellsOptions::p0_window()` / `p0_late()` stay in `mfsk-core` (so `mfsk-ffi`, the bindings and the host floors name the same values); an application overrides a field through the builders.
- **D3** whether `OsdSeeds::AD` and the capped OSD may become the default on any profile (Issue #766: all are options, off, until the owner agrees and a reviewer has seen the public API).
- **D4** whether P0 keeps the post-window SIC (needs the second cell store and the held audio, plan section 14) or stays inside the window.
- **D5** the classic ESP32 (P0-lite): worth a measurement of the cells flow on a Core2, or left as a documented variant.

## 8. Extrapolation from P0 to P1 (Cortex-A9) and P2 (Pi 3B class)

**An estimate, not a measurement.** There is no A9 or Pi here. The method: the CoreS3 measurements give a cost in nanoseconds per host (x86-64) instruction, the host gives the instruction count of each flow, and a class's cost per instruction is
reasoned from its core type. Error bars are wide (plus or minus 50 % is honest); a first measurement on the real board replaces every figure here.

**Measured on P0 (CoreS3, one core).** BP pass 37 600 host instructions in 0.41 ms (10.9 ns an instruction); the band search 42.4 M in 0.46 s (10.8 ns); the embedded OSD call 0.76 M in 5.13 ms (6.7 ns: the hand-tuned
loops). So the S3 costs **about 10-11 ns per host instruction** (about 2.5 cycles at 240 MHz: single-issue, the FPU and PSRAM latencies).

**Reasoned for the other classes (one core).** A Cortex-A9 (about 1 GHz, 2-wide, out of order, 512 KB L2) and an A53 (1.2 GHz, 2-wide in order) run this kind of code at an IPC of about 0.7-1.2; ARM needs 20-40 % more instructions than x86 for the
same work (no memory operands): **about 1.0-1.6 ns per host instruction** per core, i.e. **7-10 times the S3 per core**. The cells (172 KB) fit the L2, so the PSRAM-bound phases of the S3 (the band search) should gain more than the
compute-bound ones. The S3's two-core costs (section 3 of `ESP32S3_TWO_CORE_BEHAVIOUR.md`) are **not** assumed to carry over to a Cortex-A's cache hierarchy.

**Host instruction counts (x86-64, `qso3_busy`, 30 candidates; the faithful rows measured on 2026-11-27 with callgrind, the cells rows from design notes sections 26-29).**

| flow | M instructions | decodes on `qso3_busy` | A9, one core (1.0-1.6 ns) | Pi 3B, one core (1.0-1.5 ns) | Pi 3B, four cores (60-80 % efficiency) |
|---|---|---|---|---|---|
| cells, OSD off, 1 pass | 312 | 9 | 0.31-0.5 s | 0.31-0.47 s | |
| cells, OSD seeds `AD` at once, 1 pass | about 430 | 13 | 0.43-0.69 s | 0.43-0.65 s | |
| cells, SIC 2 passes, OSD `AD` at once | about 700-850 | 14-16 | 0.7-1.4 s | 0.7-1.3 s | about 0.3-0.7 s |
| cells, SIC 3 passes, OSD `AD` at once | about 1 000-1 200 | 15-17 | 1.0-1.9 s | 1.0-1.8 s | about 0.4-0.9 s |
| faithful `Decoder<Ft8>`, Normal | **7 280** | 21 | **7.3-11.7 s** | 7.3-11 s | **2.3-4.9 s** |
| faithful `Decoder<Ft8>`, Deep | **13 050** | 21 | 13-21 s | 13-20 s | 4.1-8.7 s |

The cells rows beyond the first two are the least certain (the SIC's passes were last counted before the BP speed-ups: 628 / 930 M for 2 / 3 passes with the OSD off; the OSD adds about 120-250 M a round).

**What it says for the assignment.**
- **P1 (A9)**: the cells flow with the SIC and the OSD `AD` *at once* (no deferral, no cap, no gate) fits a reply deadline of boundary + 0.5 s on **one core** for the 1-pass flow (finishing about -0.1 s) and plausibly for a 2-pass SIC (about +0.1-0.6 s); the
  cap K and the deferral that P0 needs are unnecessary there. The faithful pipeline (Normal) takes 7-12 s on one core: inside the 15 s slot, outside any reply deadline: a logger / receiver option, with two threads about 4-7 s.
  Deep (13-21 s on one core) does not fit a slot on one core.
- **P2 (Pi 3B)**: the same flows; with four threads the faithful pipeline (Normal) is 2-5 s and Deep 4-9 s: **host-equivalent decoding (the faithful pipeline, equal to jt9 on the golden tests) fits a slot on this class**; the cells flow with a 3-pass SIC
  and the OSD is about 1-2 s on one core and finishes around the reply deadline with threads.
- **How much the cells flow gives up against the faithful pipeline**: on the 60 real slots the cells flow with a 3-pass SIC and the OSD finds 1 206 of the 1 338 references (about 90 %); the faithful pipeline is the reference (on `qso3_busy` 21 decodes against 16-17).
  That is the price of the cells flow's divergences (the search, 30 candidates, no AP); P1 / P2 can avoid it by running the faithful pipeline whenever the reply deadline allows.

## 9. Replacing the old S3 flow wholesale

Decision of the owner (2026-11-27): the old S3 flow (12 kHz spectrogram, `sync8`, fine sync, Goertzel stage 3, `dual_core.rs`, `decode_pipeline.rs`) is replaced by the cells flow, not kept beside it, because the cells flow is already faster (312 against 423 M
host instructions at 30 candidates, OSD off) and decodes more (974 against 822 of the references with the same knobs; the old flow was the ft8dec-class result). The work, in order (the first two are the plan of `S3_CELLS_IMPLEMENTATION_PLAN.md` section 5, steps 1-3, not yet done):

1. **The app's decode path onto the cells flow**: the producer (the demux into the half-dB cells as the slot is captured), the emit point (pair 87, 0.81 s before the boundary), the search, stage 3 over two workers, the deferred OSD pass (K = 6 on one core, up to 14 on two), the result to the auto-QSO FSM;
   under WiFi and the USB host (the second worker's staging and scratch about 22 KB of internal DRAM against a 15.8 KB floor: **to find or to forgo**).
2. **Parity before deletion**: the app's decodes against the old path's on the 22 vendored recordings and the wav-sim, at least equal on every one (tier B floors exist for the cells flow: `cells_flow_*` tests, `qso3_compare`).
3. **An inventory of what else uses the old flow** (the library's `decode_block`, `decode_block_streaming`, `decode_block_low_mem`, `coarse_sync_gated_spec`, `fine_sync_12k`, the FFI, `gate-bench`, `ft8-bench`, the Core2 app, `m5stack-s3-app`): a read-only pass to decide what is deleted, what stays for another consumer, and what needs a new owner.
4. **Deletion**, as its own change after parity: the modules, the `Cargo` features and the benches that only the old flow used, with `CHANGELOG` and the docs that name them.

New open decisions: **D6** the inventory of step 3 (what the old flow's pieces are still used by); **D7** whether the classic ESP32 / Core2 app moves to the cells flow in the same change or keeps the old flow until D5 is answered.

## 10. P0 configuration, decided 2026-11-27

**The window path** (decodes available before the reply deadline, boundary + 0.5 s) and **the late path** (decodes after it, fed to the auto-QSO). All of it is an option over the WSJT-X reference, off by default in the library, marked and tracked in Issue #766.

| | window path | late path |
|---|---|---|
| flow | cells flow, 8 kHz, `fixed-point`, producer during capture, emit at pair 87 (0.81 s before the boundary); **replaces the old S3 flow wholesale** (section 9) | the same cells, a second store for the SIC |
| candidates | 30 | the first pass's, then the SIC's near-signal candidates |
| variant `d` floor | 16 (`CELLS_D_MIN_SCORE`, Issue #765) | the same |
| pre-BP gate | `CELLS_GATE_OSD` | the same |
| OSD | `osd174_91_into`, seeds `AD`, deferred, **K = 6**, ranked by `OsdRank`, one core | OSD in every SIC round, K = 6 (`sic_passes_osd`) |
| SIC | none | 2 passes (3 passes: +1 decode on `qso3_busy`, 2 s more); the OSD's decodes subtracted for the next pass |
| cores | one (the second is an option for latency margin: round x1.5, OSD pass x1.5, band x1.1 on the S3) | core 1, a task below `uac_reader` and the USB host in priority |
| AP | none (not in the board build) | none |
| ends | about boundary + 0.31-0.42 s on one core (qso3_busy / qso1 / qso2 computed from bench stages, not measured in the app) | about boundary + 4-5 s (`ft8-sic-bench` 4.94 s with the OSD on `qso3_busy`) |

**Why K = 6 and not more.** On the 60 real slots the OSD adds 50 decodes to the 974 of the BP alone (1 024 with every attempt run); K = 6 takes 33 of them (1 007), K = 8 38, K = 10 44, K = 14 47 (1 021): **K = 14 is +14 decodes (1.4 %) over K = 6** and +1 on `qso3_busy` on the board. What a second core buys is latency (the K = 6 pass ends about 0.11 s after the boundary instead of 0.42 s), not recall. The recall is in the SIC: 3 passes with the OSD at K = 6 find 1 183 against 1 007 (+17 %). The pre-BP gate costs no decode with the capped OSD (host, 4 780 slots: 7 347 / 1 007 with and without it at K = 6, 7 393 / 1 021 at K = 14) and takes 16 % off the BP passes.

**Against ft8dec** (60 real slots, jt9's 1 338 references, host; no SIC in the first two blocks): ft8dec 30 candidates, OSD off 822, OSD on 836; 300 candidates, OSD off 958, OSD on 989. **Cells, 30 candidates: OSD off 974, K = 6 1 007, K = 14 1 021.** With the SIC: ft8dec 30 candidates 1 036 / 1 053 (2 / 3 passes), its default (300 candidates, 3 passes) 1 204; cells with the OSD at K = 6 in every round 1 171 / 1 183, with the OSD at once 1 192 / 1 206.
Phantoms (decodes not in the reference): ft8dec 15-19 (1 pass, 30 candidates), cells 24-34 (K = 6: 34): the OSD adds about 10 and not all are false, as jt9 does not decode everything real.

**The late path into the QSO machine.** `activator.rs`'s `Activator::decide(period, heard)` is called once per own transmit period at the reply deadline with *every message addressed to this station decoded since the last call, late ones included* (its own doc: "a caller repeats, so a decode that missed its deadline is still worth answering from"); the partner's message is picked by exchange stage, not by arrival. A late decode therefore enters the queue like any other and is answered **one own period later** (about 30 s with every other slot). Nothing about the machine changes for it.
**But the app does not use `Activator` yet**: `decode_pipeline.rs` calls `QsoManager::process_message` (`qso.rs`, the Phase 4 dry-run machine, TX unwired) for each result inline, with no queue, no `Heard`, no `decide` at the deadline. So the first work item is wiring (below).

**Work, in order** (the first two are step 1 of section 9):
1. **Wire `Activator` into the decode path**: a queue of `Heard` between the decode task(s) and the decision point; `decide` at the reply deadline once per own period; replace `QsoManager` there.
2. **The cells flow in the app** (producer, emit, search, stage 3, the deferred OSD pass), then the parity check and the deletion of the old S3 flow.
3. **The late path**: a task on core 1 below `uac_reader` that runs `sic_passes_osd` over the slot's cells and audio (a second cell store 172 KB and the previous slot's audio 240 KB held, `SicStorage` in PSRAM: the PSRAM left after replacing the 351 KB spectrogram by the 172 KB store is ample) and enqueues each decode not already queued (the 77 bits), dropping any older than `PENDING_TTL` (2 own periods); the queue safe for two producers.
4. **Measure in the app**, not in the benches: the end-to-end latency of the window path under WiFi and the USB host, the internal DRAM left (the K = 6 flow's extra over the old one, the SIC's scratch), and what the SIC on core 1 does to the producer on core 0 (the esp-dsp FFT scratch is behind a spin lock: **untested contention**), the board time of the pre-BP gate together with the capped OSD (the host shows no loss; the board has not run the combination).

**Decisions closed by this section**: D4 (the late SIC stays in P0), the K of the window path (6), the core count (one by default). **Open**: D1-D3, D5-D7 of section 7-9, and ~~D8~~ **decided 2026-11-27**: the queue is a **`heapless` MPMC queue in `mfsk-app-shared`** (host-testable); producers (the window task on core 0, the late SIC on core 1) push without blocking; **the QSO task owns `decide`'s clock**: at the reply deadline it drains the queue, drops duplicates by the 77 bits and anything older than `PENDING_TTL` (2 own periods), and passes the slice to `Activator::decide`. A full queue drops the oldest (a late decode is the least valuable).

