# S3 / CoreS3: what the hard-sync gate and OSD give without the FIR baseband (stage 0, host)

2026-11-25. Stage 0 of the S3 plan (`MIDRANGE_EMBEDDED_EVAL.md` is the earlier experiment). The question: the low-memory
profile (`decode_block_low_mem`) reaches `qso3_busy` 14 of 20 and 279 of `jt9`'s 402, the ESP32 driver 7 and 131. How much
of that needs the FIR baseband (`cd0_fir`), which the S3 tried in 0.6.2 and gave up (`ROADMAP.md`, "Embedded fine_refine
attempt postmortem": PSRAM bandwidth) and which costs more instructions than Goertzel? **Everything here is host data**
(Apple M-series, fixed-point build, 8 kHz input unless said); nothing was run on an Xtensa core.

## Variants (all with `LlrEffort::Minimal`, coarse search in one band, `sync_min` 1.3)

| | candidates | fine refine | symbol spectra | OSD |
|---|---|---|---|---|
| V0 | coarse score order (`decode_block`, the ESP32 driver) | none (the embedded `fine_refine_pass1` is a no-op) | Goertzel | no |
| V3 | score order | none | Goertzel | yes |
| V8 | score order | `fine_sync_12k` (the board's `fine_sync_split`) | Goertzel | no |
| V1 / V2 | hard-sync gate, then the pass-2 re-rank | none | Goertzel | no / yes |
| V6 / V7 | gate, then the pass-2 re-rank | `fine_sync_12k` | Goertzel | no / yes |
| V10 / **V9** | **the first `max_cand` by hard sync, no re-rank** | `fine_sync_12k` | Goertzel | no / **yes** |
| V5 / V4 | gate | FIR baseband (`fine_refine_3stage`) | FIR baseband | no / yes (the ICAP profile) |

## Results (`hosttest/icap-app/tests/s3_gate_osd.rs`, `icap-profile --variant vN`)

| | 22 recordings (of 402), 15 / 30 cand. | `qso3_busy` (of 20), 15 / 30 | instructions, `qso3_busy`, 15 cand. |
|---|---:|---:|---:|
| V0 ESP32 driver | 131 / 144 | 7 / 7 | 308 M |
| V3 score order + OSD | 135 / 148 | 7 / 7 | |
| V8 score order + `fine_sync_12k` | 136 / 145 | 7 / 7 | 248 M |
| V1 gate | 173 / 233 | 7 / 7 | |
| V2 gate + OSD | 184 / 249 | 8 / 9 | |
| V6 gate + `fine_sync_12k` | 174 / 248 | 9 / 10 | |
| V7 gate + `fine_sync_12k` + OSD (with re-rank) | 185 / 267 | 11 / 14 | |
| V10 gate order + `fine_sync_12k` | 211 / 248 | 9-10 / 10 | 259 M |
| **V9** gate order + `fine_sync_12k` + OSD | **221 / 267** | **12-13 / 14** | **283 M** |
| V5 FIR baseband, no OSD | 216 / 257 | 10 / 10 | 596 M |
| V4 FIR baseband + OSD | 227 / 279 | 13 / 14 | 614 M |

(`qso3_busy` 13 at 8 kHz input, 12 at 12 kHz for V9 and V4 at 15 candidates; the 22-recording figures are the same at both rates.)

**What each piece is worth** (22 recordings / `qso3_busy`):
- The gate alone: +42 / +89 (15 / 30 candidates) on the recordings, **nothing on `qso3_busy`** (7 stays 7).
- `fine_sync_12k` after the gate: `qso3_busy` 7 to 9-10; recordings +1 / +15. Without the gate it adds nothing (V8 = V0 + 5).
- OSD: small on its own (V3 = V0 + 4) and +10 / +19 on top of gate and fine sync, `qso3_busy` +3 / +4. **It only pays on a good candidate list.**
- Re-ranking the gate's list by block-0 quality (the pass-2 step of the boards) **costs** recall at 15 candidates: V7 185 against V9 221.
- **V9 has 94 % / 91 % of V4's recall gain over V0 on the recordings and all of it on `qso3_busy`**, with no FIR baseband.

**Cost** (instructions per slot, `qso3_busy`, 12 kHz input so the harness's resampler is not counted; host, not LX7): V9 283 M is
below V0's 308 M. The FIR variants are twice that (596-614 M). So the gate, `fine_sync_12k` and OSD do not raise the
instruction count of the ESP32 driver, and the FIR path doubles it; this agrees with the operation count (the FIR cascade
is about 3.0 M multiply-adds a candidate, Goertzel about 1.2 M iterations).

## The gate on the boards' own spectrogram (`coarse_sync_gated_spec`)

The boards build the `u16` spectrogram while the slot is captured (`stage1_inc`) and search it about 0.9 s before the slot ends
(`SPEC_EMIT_PAIR` 87: rows 0..173 of 184 are measured, the rest zero). `coarse_sync_gated_spec` reads the gate from such a
spectrogram, no audio. `hosttest/icap-app/tests/s3_gate_osd.rs`, 22 recordings (of 402) and `qso3_busy` (of 20), 15 / 30 candidates:

| | 22 recordings | `qso3_busy` | |
|---|---:|---:|---|
| V9 gate from the audio | 221 / 267 | 13 / 14 | |
| **V12** gate from a whole-slot spectrogram | 221 / 267 | 13 / 14 | **the same decodes as V9**, in every file |
| **V11** gate from the boards' partial spectrogram (last 10 rows zero), block 2 scored on the rows there are | **216 / 269** | 13 / 14 | what the boards do (`valid_rows` not gated) |
| V13 the same with block 2 **gated** (`PartialBlock2::Gate`, the boards' `FT8_BLOCK2_GATE`) | 116 / 168 | 11 / 14 | refuses every lag whose block 2 is not covered |

So the gate survives the unmeasured tail: -5 and +2 decodes against the whole slot. Gating block 2 is what hurts (the boards keep it off), and the
test pins that. The hard-sync count skips rows past the fill point (an unmeasured row reads as tone 0, a false hit wherever the Costas array has a 0;
`hard_sync_skips_the_rows_a_streaming_spectrogram_has_not_measured_yet`). The harness here searches the whole slot's lag window (±1.0 s, the
boards' too: `EMBEDDED_SYNC_LAG_S`) on a spectrogram with the tail zeroed; the boards' own search runs on `stage1_inc`'s buffers and was not run.

## On the board's own pipeline (the mirror, `mirror_gate_and_osd`)

The ablation above is one pass over a whole slot. `ft8_embedded_pipeline_mirror.rs` is `run_speculative_slot`'s flow call for
call on the host (the spectrogram with its last rows zero, `coarse_sync` 100-3000 Hz split in two with the board's lag window and `sync_min` 1.0,
the early half on the 168 000-sample prefix and the late half on the slot, 15 candidates between them, the dedupe by message); no deadline, no clock.
`mirror_gate_and_osd` adds the pieces to it one at a time and sweeps the recording's phase (-0.40..+0.40 s, 0.01 s: 81 phases; a weak station's
decode is a 5 ms-scale comb in dt, so a coarser grid aliases it). Decodes of the **known** signals per phase, mean (min-max); fixed-point, `max_cand` 15:

| arm | `qso3_busy` (20 known) | distinct known over the phases | `qso1` (6) | `qso2` (5) |
|---|---:|---:|---:|---:|
| ship (the board's pipeline) | 6.56 (5-8) | 9 | 1.98 | 2.11 |
| + `fine_sync_12k` | 8.20 (7-10) | 10 | 1.88 | 2.00 |
| + `fine_sync_12k` + OSD | 10.35 (8-12) | 13 | 1.94 | 2.62 |
| gate | 6.98 (5-8) | 10 | 2.05 | 2.11 |
| gate + `fine_sync_12k` | 9.26 (8-11) | 11 | 1.99 | 2.31 |
| gate + OSD | 8.81 (7-12) | 13 | 2.21 | 2.83 |
| **gate + `fine_sync_12k` + OSD** | **11.35 (10-13)** | **13** | 2.05 | 2.73 |

- The ship arm's 6.56 is the board's own figure (it runs at 7-8 on `qso3_busy`), so the mirror is on the board's scale.
- On the busy slot the three pieces add up: `fine_sync_12k` +1.6, OSD +2.2 on top, the gate +1.0 on top of both; **+4.8 decodes of 20 in all**, and the worst phase of the full arm (10) is above the best phase of the ship arm (8).
- On `qso1` and `qso2` there is little to gain here (the board decodes 2 of 5-6 at any phase and the full arm 2.1 / 2.7): their signals are mostly not in reach of the 15 candidates, as `CORES3_FT8_SLOT_BUDGET.md` found.
- **No unexplained decode in any arm, recording or phase** (a decode that is not one of the known signals: a phantom, a signal the reference lacks, or a copy). 7 arms x 3 recordings x 81 phases.
- What the mirror cannot say: the time. The deadline (`cut`), the stage-3 time with OSD and the 292 ms `fine_sync_split` are the board's; a candidate that OSD makes slower can be one that the deadline then drops.

## The board bench (`ft8-gate-bench`, stage 1 of the plan; written, **not yet run**)

`embedded-poc/embedded-shared/src/apps/gate_bench.rs`, run by `m5stack-cores3-app`'s `ft8-gate-bench` bin (feature `ft8-gate-bench`, which turns on
`mfsk-core/osd` through `embedded-shared/gate-bench`). Six arms (`ship`, `ship+fine`, `gate`, `gate+fine`, `ship+fine+osd`, `gate+fine+osd`) on `qso3_busy`,
`qso1`, `qso2`, each twice (cold, warm; read the warm one), one core, all through the same stage 3 as `dual_core::stage3_split` (the `cs` scratch in
internal DRAM). It prints one `GATEBENCH` line per run: the time of the spectrogram, coarse search, `fine_sync_12k`, pass 2 and stage 3, the number of
candidates into stage 3 and of decodes, and the minimum free internal DRAM and PSRAM and the task's stack high-water mark. The OSD arms run last, so
the stack mark after the arms before them is the stack without OSD. It does not model the streaming spectrogram, the second core, the deadline or the prefix: read
the candidate stages (fine sync, pass 2, stage 3), not the totals.

Built and linked on this Mac (`cargo install ldproxy` was needed; `target/xtensa-esp32s3-espidf/release/ft8-gate-bench`, 2.16 MB against the 9 MiB factory partition), **not flashed**. To run it:

```sh
cd embedded-poc/m5stack-cores3-app
cargo build --release --features ft8-gate-bench --bin ft8-gate-bench
# the reference, same board and session: the shipped driver's stage times (no OSD)
cargo build --release --features ft8-bench --bin ft8-bench
# flash and capture through the script only (no second `espflash monitor`); change a source line before re-flashing the same ELF
../scripts/flash-monitor.sh target/xtensa-esp32s3-espidf/release/ft8-gate-bench logs/cores3_gate_bench_$(date +%F).log 900
```

What to read first: `us_fine` of `ship+fine` (the board's `fine_sync_split` is documented at ~292 ms), `us_stage3` of `gate+fine+osd` against `gate+fine` (OSD's cost, and `n_stage3 - decodes` is the
count of candidates it may have been tried on), `stack_free_min` after `gate+fine` and after `gate+fine+osd` (OSD's stack against the 16 KB main task), and
`min_free_internal` (the plan's floor is 12 KB).

## In the board's pipeline, switched off (`dual_core.rs`, `decode_pipeline.rs`; **not run on a board**)

`DecodeConfig::hard_sync_gate` (default `false`, every constructor sets it) replaces `coarse_sync_split_with_allsum` with `coarse_sync_gated_spec` on
the SpecBundle's spectrogram (`SpecFill` from its `valid_rows`, the board's lag window, `FT8_BLOCK2_GATE` as before) and, with it on, takes the first
`early_budget` of the ready half and the first `max_cand_late` of the deferred half to pass 2 so that the gate's order is not undone by
`refine_candidates_into`'s re-rank (the mirror's `gate` arm does the same). With it off the code path is the same calls with the same arguments as before.

The CoreS3 app has two compile-time knobs, both off, in the style of `MFSK_FT8_FINE_SYNC`: `MFSK_FT8_GATE=1` and `MFSK_FT8_OSD=1`. OSD needs
`--features ft8-osd` (which builds `mfsk-core/osd`); **asking for it without the feature does not compile** (a `const` assertion), because without it
`DecodeDepth::osd` silently does nothing and a log that says "osd" would lie. The start-up log says which are on (`decode pipeline: gate .. osd .. fine_sync ..`).
To try it: `MFSK_FT8_GATE=1 MFSK_FT8_OSD=1 cargo build --release --features ft8-osd`.

**Fine sync is off on the board** (`FT8_FINE_SYNC`: its ~292 ms did not fit the early path, `cut` rose and slots ran past key-up on a radio, 2026-09-19; and
`FT8_FINE_SYNC_MIN_SLACK_MS` 2 s against a steady slack of 1.43 s keeps it unreachable). So the configuration that is practical today is the mirror's
**gate + OSD without fine sync**: 8.81 decodes of 20 on `qso3_busy` against the shipped 6.56 (+2.25), the same 13 distinct known signals as the full arm. Whether the gate and OSD fit
the board's time is exactly what `ft8-gate-bench` and then a radio say.

What is not covered by any test: `dual_core.rs` runs only on the board (the mirror is a re-implementation of it, not a test of it).

## OSD's stack, from the linked ELF (device-free; **the plan's "about 9 KB" was too small**)

`ft8-gate-bench` links (this Mac now has `ldproxy`), so the frames of the call chain can be read from the ELF
(`xtensa-esp32s3-elf-objdump -d`, each function's `entry a1, N`, the call graph through `call8` and the `l32r` + `callx8` pairs the
linker uses for far calls; Rust symbols only, the IDF's C leaves and indirect calls through trait objects are not followed, so this is a floor):

| from `process_candidates_into_with_cs_scratch_tuned` (stage 3, one candidate) | frame | cumulative |
|---|---:|---:|
| the function itself (the BP and LLR scratch of one candidate, inlined) | 9 632 B | |
| `compute_llr_metric` and below (no OSD) | ~1.5 KB | **~11.2 KB without OSD** |
| `osd_strategy::try_fallback` | 5 168 B | |
| `osd_npre` (`g`, `orig`, `indices` on the stack) | 10 304 B | |
| the sort inside it | ~0.3 KB | **~25.4 KB with OSD** |

OSD adds **~14.3 KB**, not 9. `dsp_worker` (16 KB) and the CoreS3 main task (16 KB) run stage 3, so **OSD overflows both as they are**; internal DRAM has 7-15 KB
to give at run time (`embedded-poc/CLAUDE.md`, "Stacks, heaps, and the space between them": an overflow rewrites TLSF headers and shows up later as heap
corruption, `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK=y` turns it into a panic at the instruction). Without OSD stage 3 fits with ~5 KB to spare, which is how
the board runs today. Consequences made in the code:

- `MFSK_FT8_OSD=1` now needs `--features ft8-osd` **and** `MFSK_FT8_OSD_STACKS_RAISED=1` (a `const` assertion says why): the switch cannot be turned on
  by accident into a stack overflow. Raising the stacks is **not done**: it is a placement decision (PSRAM stacks for `dsp_worker`, which slows every frame in
  them, or OSD's arrays off the stack, a library change) that wants the board's numbers first.
- The bench runs on a thread with a 48 KB stack (the OSD arms would overflow the 16 KB main task) and logs the high-water mark, which is the measured figure
  to set against this estimate.
- The plan's risk 4 stands, larger: OSD costs stack as well as time, and the stack is the scarcer resource.

## Caveats (read before relying on this)

- **Host only.** LX7 times, PSRAM effects and the dual-core split are not in any of it. In particular `fine_sync_12k`
  is documented at **~292 ms on the board** (`dual_core.rs:366`, for the candidates it is given; split over two cores), and V9 depends on it:
  without it (V1) `qso3_busy` is 7. It is the piece to measure first.
- The harness has **no per-message dedupe** between candidates (the boards' `run_speculative_slot` has `dedup_by_message`). V9's
  extras are two copies of a strong signal at another frequency (`PA3EPP SP8NFO` 150-250 Hz from `jt9`'s 906 Hz line); they
  are attested in `profile::ATTESTED_EXTRAS` with that reason.
- The boards run `sync_min` 1.0; the harness uses 1.3. The partial spectrogram is covered above (V11).
- V9 takes the first `max_cand` by hard sync *before* fine sync; the boards fine-sync all 30 of pass 1 when there is slack. Not the same flow.
- OSD's cost on LX7 is unmeasured for FT8 (FT4 on an S3: +2 345 ms over 31 candidates; `FT4_BENCHMARK_LOG.md` §22). Here OSD is +10 / +19 decodes
  on the recordings and +3 / +4 on the busy slot.

## What this changes in the plan

- **The S3 needs no FIR baseband for this recall.** Stage 1's arms become: B1 the ESP32 driver (V0), V8 (the board with time to spare), V10 (gate,
  no OSD), **V9 (gate + OSD)**; V4 stays as a reference only.
- The new library item, the gate on the boards' own spectrogram (`coarse_sync_gated_spec`), is in. The pass-2 re-rank is bypassed for the gate's list in the harness (V9 takes the first `max_cand` by hard sync before `fine_sync_12k`); on the boards that is `dual_core.rs`'s step 2 to change.
- OSD is decided on the board: G1 (V10) against G2 (V9) costs, with the number of candidates that reach OSD.
