# ESP32-S3 (LX7): what two cores and the memories cost — measured on the CoreS3

**Read this before designing anything for the ESP32-S3 boards that assumes two cores, a working set in PSRAM, or a table in flash.** Every figure is from a board run of this repo
(CoreS3, ESP-IDF 5.5.3, 2026-11-27) with the bench named next to it; the logs are in `embedded-poc/m5stack-cores3-app/logs/` (not committed). What is measured, what is only inferred
and what is unknown are kept apart below. The numbers belong to **this configuration** (section 1); a different `sdkconfig` can move them.

## 1. The configuration measured

| | |
|---|---|
| CPU | 240 MHz (`esp_rom_get_cpu_ticks_per_us` and the cycle counter agree, one core alone or both busy: **no clock throttling**) |
| data cache | 32 KB, 8 ways, 32 B lines, **one cache for both cores** (`CONFIG_ESP32S3_DATA_CACHE_32KB`) |
| instruction cache | 16 KB, 8 ways, 32 B lines |
| flash | DIO, 80 MHz; PSRAM: Quad, 80 MHz; `SPIRAM_MALLOC_ALWAYSINTERNAL` 2048 (an allocation above 2 KB goes to PSRAM) |
| internal SRAM | 32 KB banks (Espressif forum: two masters on different banks run at full speed, on one bank are arbitrated round-robin: not checked against the current TRM) |

## 2. The memory benchmark (`ft8-mem-bench`, `embedded-shared/src/apps/mem_bench.rs`; module doc has the method and the predictions)

The same loop on core 0 alone and on both cores at once (common start; the shared-table arms start the second core half a cycle on), the second run of each kept. A memory arm chases a
cyclic permutation (`i = table[i]`: the working set is the whole table, each step waits for its load). **Cycles a step**, core 0, alone → with core 1 running the same loop:

| loop | alone | both cores | slowdown | what it says |
|---|---|---|---|---|
| `mul` (dependent multiply-add), code in flash | 8.0 | 10.0 | x1.25 | |
| `mul-iram` (code in IRAM) | 8.0 | 10.0 | x1.25 | **same as flash code: not the instruction cache** |
| `alu` (eight f32 chains), flash / IRAM code | 13.0 | 14.8 / 15.0 | x1.14 / x1.15 | |
| chase over 8 KB in internal DRAM: tables in **different banks**, in **one bank**, **one shared table** | 8.0 | 10.0 | x1.25 | **bank placement made no difference** |
| chase over 8 KB of flash `.rodata` (fits the data cache) | 8.0 | 10.0 | x1.25 | |
| chase over 32 KB of PSRAM, **a table per core** (64 KB together) | 8.1 | **293** | **x36** | the shared 32 KB data cache thrashes |
| chase over 32 KB of PSRAM, one table shared | 8.1 | 10.1 | x1.25 | fits together: no thrash |
| chase over 512 KB of PSRAM (misses) | 262 (1.09 us) | 537 (2.24 us) | **x2.05** | **PSRAM misses of the two cores are served one after the other** |
| chase over 128 KB of flash `.rodata` (misses) | 385 (1.60 us) | 768 (3.20 us) | **x2.00** | **flash misses likewise** |

**Measured facts.**
1. **A loop that touches no shared memory still runs slower on each core while the other core runs a loop**: about **12 % more cycles for the work itself, plus about 2-3 cycles per loop
   iteration**. The same for code in IRAM and code in flash, for any internal-DRAM placement, and the clock does not change (the cycle counter counts the extra cycles). Measured with a
   non-foldable step (a multiply-add and an xorshift, 5 cycles alone) unrolled N times (`mx-uN`; cycles a step alone -> with both cores): N = 1 10.0 -> 14.0 (x1.40), 2 7.5 -> 10.0 (x1.33),
   4 6.25 -> 7.5 (x1.20), 8 5.63 -> 6.52 (x1.16), 32 5.16 -> 5.76 (x1.12). So **a tight loop of one or two short steps pays x1.3-1.4, a long unrolled body x1.12**; the x1.12 does not go away.
   The tight loops of the first runs (`mul`: 8 -> 10 cycles an iteration, x1.25; `alu`'s inner loop 13 -> 14.9) are the small-N end of this. The cause is **not found**; inference only: a path both
   cores use for fetching or issuing (the fetch of a loop's head costs more than a straight run).
2. **A cache miss to PSRAM (1.09 us) or to flash (1.60 us) is serialized across the two cores**: two cores missing at once take twice as long each. A second core adds no miss throughput.
3. **The data cache is shared and 32 KB**: two cores whose PSRAM working sets together exceed it thrash it (x36 for two 32 KB tables).
4. A hit costs what internal DRAM costs (8 cycles a step here); a miss is 260-385 cycles (the 1.1-1.6 us above), 30-50 times a hit.

## 3. What it meant for the cells flow (`ft8-two-core-bench`, design note `FT8_WINDOW_AND_COARSE_DESIGN.md` section 30)

One core against two, 30 candidates, same inputs:

| | one core | two cores | speed-up |
|---|---|---|---|
| BP round (30 candidates; internal DRAM staging, PSRAM cells read) | 0.585 / 0.447 / 0.469 s | 0.383 / 0.297 / 0.308 s | x1.50-1.53 |
| OSD pass, K = 14 attempts | 0.529 / 0.586 / 0.561 s | 0.362 / 0.390 / 0.384 s | x1.46-1.50 |
| band search (`band_best`, reads the PSRAM cells) | 0.458 s | 0.406-0.418 s | **x1.10-1.13** |

An OSD attempt is 38 ms on one core and **about 55 ms on each of two**; every attempt of a pass is slower, the shares of the two cores are equal. Moving the OSD's 45 KB of scratch from PSRAM
to internal DRAM changed the two-core pass by 5-6 % only: the cost is not that memory. **Read with section 2:** the x1.25-1.4 of the round and the pass is the per-iteration cost of fact 1
(plus the cells it reads); the band search is PSRAM-bound (facts 2 and 3) and two cores gave it x1.1.

## 4. Rules for a design on these boards

- **Unroll a variable-trip inner loop that runs thousands of times a call.** Tried on the embedded OSD (`ft8-two-core-bench`, K = 14 pass; `ft8-osd-bench`): the elimination's row update was a loop of `w0..W` words (2-6 trips,
  8 281 times a call); written out over all six words it took **17 % off the pass on one core and on two (537 / 362 -> 447 / 303 ms), 23 % off one OSD call (7.26 -> 5.60 ms)**, bit for bit the same answers. The
  two-core / one-core ratio did not move (0.68): this is the loop overhead of an inner loop whose trip count is not a constant, not the cost of the second core. The other loop tried, the `npre1` gate
  (4 095 tests a call, a software popcount in each), tested four pairs a round: 1 %, not kept. **Predicted before the run: 5 % at most; wrong for the elimination.** Variable-trip inner loops
  with a few instructions in the body are the first place to look in a hot path; the `trailing_zeros` scatter that builds the generator (about 8 000 data-dependent trips a call) was replaced by 18 32 x 32 bit transposes (constant trip counts): **5.60 -> 5.13 ms a call (-8 %)**, bit for bit the same answers (predicted 5.15-5.35 ms). Together with the elimination, one OSD call went 7.26 -> 5.13 ms.
- **Do not assume 2x from two cores.** Budget **x1.5 for compute- and internal-memory-bound work** (BP, OSD) and **about x1.1 for work that streams a PSRAM working set** (the band search), and
  measure the one-core figure on the board: the old S3 dual-core note's "244 ms of single-core work" was the 122 ms wall times two, an assumption (`PHASE_D_PIE_SIMD.md`), so the
  cost of two cores was not visible in it.
- **Keep the PSRAM working sets of both cores together under the 32 KB data cache, or give the cores disjoint, sequentially-read data.** Two random-access working sets in PSRAM that
  sum to more than the cache can cost 30 times (here x36), not 2.
- **A miss is 1.1 us (PSRAM) to 1.6 us (flash), and two cores do not get two of them at once.** A table in flash `.rodata` (the BP edge tables, the OSD's generator columns, the LDPC tables)
  is fine while it stays in the data cache and slow when it does not, and the cache is shared with everything else: a table that fits alone can be evicted by the other core's data.
- Internal DRAM bank placement did not matter in this test (8 KB tables, one chain of loads per core); it was not tested under a streaming load.
- Putting hot code in IRAM did not remove the two-core slowdown of a tight loop.

## 5. Not known

- **Why a loop with no shared data is slower when the other core runs** (12 % of the work and 2-3 cycles an iteration). Not the clock, not the instruction cache, not the memory banks (as
  measured); it depends on the iterations and on the work, not on either alone. Untested: the instruction bus, other tasks and interrupts on the second core, ESP-IDF's per-core FreeRTOS tick, the
  two cores' load/store units contending for the same internal-SRAM port, whether the 12 % depends on the instruction mix (loads and stores, floating point).
- The cache sizes and the PSRAM mode are the ones in section 1; the 64 KB data cache option, the 32 KB instruction cache, octal PSRAM and the "SPIRAM_FETCH_INSTRUCTIONS" / "SPIRAM_RODATA"
  options were not measured.
- Whether the 2-cycle cost applies to every core pair state (core 1 idle in the FreeRTOS idle task was the "alone" case; a core halted in `waiti` against one running a different, light task).
- The figures are without WiFi, USB host and the app's other tasks; those add their own load on the second core.

## 6. How to reproduce

`cd embedded-poc/m5stack-cores3-app && source ~/export-esp.sh && cargo build --release --features ft8-mem-bench --bin ft8-mem-bench && ../scripts/capture.sh target/xtensa-esp32s3-espidf/release/ft8-mem-bench logs/<name>.log 150 "mem-bench: done"`
(`ft8-two-core-bench` likewise). The first version of `ft8-mem-bench` was void for the memory arms (a walk that cycled through a few hundred entries and ran from the cache) and was
rewritten with cyclic permutations; the history is in the commits `2d33cd3b`..., and in the module doc.
