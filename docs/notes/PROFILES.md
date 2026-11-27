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
| flow | **cells flow** (8 kHz, fixed-point, half-dB byte cells from the producer: `decode_block_cells[_sic]`, `CellsWorker`) / **faithful pipeline** (12 kHz, f32, `Decoder<Ft8>` through `engine::pipeline`) / the old S3 flow (12 kHz, `sync8`, Goertzel; retained, not extended) |
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
| OSD timing | deferred, capped K (K = 6 on one core, 14 on two; built, S3) | deferred, K = 2-4 or off | inline (the WSJT-X order; built) | inline | inline |
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

1. **Profile presets as data, not code paths.** A `Profile` is a named set of values on the axes of section 2 (flow, candidates, gate, OSD flavour / timing / K, SIC passes, AP, threads, modes). Where it lives is **decision D2**: in `mfsk-core` as documented constants next to the flows (so that `mfsk-ffi` and the bindings can name a profile), or in each application. Whatever the choice, a preset must be expressible with the options that exist; **a preset that needs a new code path is a sign the option is missing**.
2. **A conformance test per profile** (tier B, vendored corpus): the preset's decodes meet a recall floor **and** a phantom ceiling against jt9 on the real slots, through `assert_golden`. The P0 preset already has its numbers (above); the P1-P3 presets would pin the faithful pipeline's.
3. **OSD on a thread pool** (P2, P3; P1 with two threads): the attempts of `take_osd_attempts` / `rank_osd_attempts` / `run_osd_attempt` already separate *which* attempts run from *where*; a rayon `par_iter` over the ranked list is the whole change. Test: the decodes equal the one-worker pass (the two-worker test of `tests/cells_worker_reuse.rs`, with threads).
4. **The cells flow on Linux (D1).** It runs on the host today (`icap-cands`, `icap-sic`, `ft8-*-bench` are the embedded twins) but is `fixed-point`-only and its search is the board's. Whether P1 / P2 want it as the low-latency option (a reply-deadline host on a Cortex-A) or the faithful pipeline alone is enough is the question; the numbers that decide it do not exist yet.
5. **An armv7 build** (`armv7-unknown-linux-gnueabihf`) of the host library and of `icap-app`, run under QEMU for equality (tier A) before any real board: **to verify**, including which FFT path `rustfft` takes there.
6. **AP and OSD in the fixed-point build** are not wired together (AP is host f32 only); P0 does without, and nothing here changes that.
7. **Replacing the host's `osd174_91` by `osd174_91_into`** in the faithful pipeline: bit-equal on 22 000 inputs; the host's instructions are 7 % *higher* with `u32` rows (x86-64), so it is a P0 gain and a possible P1 / P2 loss: measure on the hardware before deciding (an `u64` build for 64-bit targets is the alternative).

## 6. When hardware is available: what to measure first

The discipline of `AGENT_WORKFLOW.md` holds: host first, predictions written before a board run, the stop rule (25 % off prediction). Per profile, in this order:

1. **One core, one recording**: the stage times of the flow (search, band, round, OSD attempt, one SIC pass) for the three vendored recordings. The embedded benches (`ft8-stage-bench`, `ft8-osd-bench`, `ft8-sic-bench`) have Linux twins to write (`icap-profile`, `icap-sic` exist for the host).
2. **Two cores against one, the same inputs**: on the ESP32-S3 two cores cost each about 12 % of the work plus 2-3 cycles an iteration, PSRAM / flash misses are served in turn, and the 32 KB data cache is shared; **none of this is assumed to carry over** to a Cortex-A's cache hierarchy. Measure the one-core baseline; never infer it (the old S3 note's "single-core work" was twice the wall time).
3. **The option sets of section 4** on the 60 real slots: the decodes are the host's (bit-defined), so the run is for time and for a check that a different FFT / FPU did not change a decode.
4. **Memory**: resident set of the flow, the cell store / spectrogram, the SIC's second store.

## 7. Open decisions (owner)

- **D1** whether the cells flow is a Linux profile at all.
- **D2** where profile presets live (library constants or application configuration).
- **D3** whether `OsdSeeds::AD` and the capped OSD may become the default on any profile (Issue #766: all are options, off, until the owner agrees and a reviewer has seen the public API).
- **D4** whether P0 keeps the post-window SIC (needs the second cell store and the held audio, plan section 14) or stays inside the window.
- **D5** the classic ESP32 (P0-lite): worth a measurement of the cells flow on a Core2, or left as a documented variant.
