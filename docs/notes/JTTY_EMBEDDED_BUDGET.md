# JTTY receive on the CoreS3: where a window's time goes, and a budget (#499)

Everything here is about **receive**; transmit (`jtty::tx::Synth`, `mfsk-app-shared::jtty_tx`) fits with room to
spare. Sources: host counters and timers from `jtty-stats` (`tests/jtty_profile.rs`, single thread), and
on-device timings from `jtty-bench` on a CoreS3 (ESP32-S3, 240 MHz, quad PSRAM at 80 MHz): E0 and E1b
(`JTTY_UPSTREAM.md`) plus the DSP section added with this note. "Measured" means one of those; "estimated"
says so.

**A correction to E0.** E0 priced the sync surface as *three* bands. It is one: `sync_surface` builds a single
surface over the union of the three channels' bins (`lo..=hi`, here 1200–1800 Hz, 411 bins), one 8192-point FFT
per column, once per window. The measured 1.26 s per surface stands; the "3 × 1.26 s" and the "35–43 ms per
band, three bands" did not, and the decimated design below is priced for one shared surface.

## 1. The decode path, stage by stage

The receiver decodes a **window** of 1.25 frames (28 320 samples at 12 kHz, 14 160 complex at 6 kHz) every quarter
frame (5 664 samples, 0.472 s). Each frame start time falls in the first quarter of exactly one window, so each
window has to be searched; there is no skipping one.

| # | stage | work | counted as |
|---|---|---|---|
| 1 | analytic signal | 32 768-point FFT, clear negative bins, 16 384-point inverse (`analytic_6k`) | once per window, and once per retro window |
| 2 | sync surface | 237 columns: multiply the 2 496-sample sync wave into the window, zero-pad to 8 192, FFT, power over 411 bins, 1-2-3-2-1 smoothing | once per window, again after any subtraction, and once per retro window |
| 3 | pick | `pick_max` scans 237 × band bins per pick (5 peaks on channel 0, 2 on each of channels 1 and 2), `suppress` around each | 9 picks per window, more after a subtraction |
| 4 | peak-up | channel 0 only: 11 frequency steps (±2.5 Hz), each mixes 2.6 k samples and slides the sync correlation over ±4 ms in 4-sample hops, then a phase-ramp fit | once per channel-0 candidate |
| 5 | mix to DC | `shift_frequency` over the **whole** window, 14 160 complex samples, plus a 113 KB allocation | once per candidate, gated or not |
| 6 | sync gate | 13 sync symbols × 4 tones × 192 samples of complex MAC, count hard-decision hits, S/N | once per candidate |
| 7 | payload correlation | 46 symbols × 4 tones × (full + two half symbols) × 192 samples | once per candidate that passes the gate |
| 8 | ladder | list-WAVA over 512 states, four rungs (L=1, 2, 4, then half-symbol L=1) until one accepts; ≈ 377 k–786 k path extensions per rung | once per gated candidate; a candidate that decodes nothing runs all four |
| 9 | subtract | re-encode, synthesise the reference (11 328 complex samples), estimate the gain with a `cos²` filter, subtract; **`f64` throughout** | once per decoded frame |
| 10 | retro re-sweep | for every subtracted signal, stages 1, 2, 3–8 again on each of the 3 windows before | 3 windows per subtraction |
| 11 | assemble | duplicates, continuation, text | negligible |

## 2. What a window does, counted (host, `jtty-stats`)

Per analysed window, means over the recording (`tests/jtty_profile.rs`; busy bands are synthetic, 90 s, stations
of −12…−2 dB in 2 500 Hz, back-to-back messages):

| | noise only | sample recording | 1 station | 3 stations | 6 stations |
|---|---|---|---|---|---|
| analytic signals | 1.00 | 1.55 | 1.19 | 1.23 | 1.69 |
| **surface builds** | 1.00 | 1.73 | 1.26 | 1.30 | 2.21 |
| retro windows | 0 | 0.55 | 0.19 | 0.23 | 0.69 |
| subtractions | 0 | 0.73 | 0.26 | 0.30 | 1.14 |
| candidates mixed to DC | 9.0 | 15.5 | 11.3 | 11.7 | 18.7 |
| peak-ups | 5.0 | 9.2 | 6.5 | 6.8 | 11.4 |
| gate passes | 0.065 | 0.90 | 0.26 | 0.54 | 1.30 |
| ladder accepts | 0 | 0.25 | 0.07 | 0.11 | 0.47 |
| rungs run L1 / L2 / L4 / half | .065 each | .90 / .67 / .65 / .65 | .26 / .19 / .19 / .19 | .54 / .43 / .43 / .43 | 1.30 / .84 / .83 / .83 |
| host ms per window | 4 | 10 | 5 | 7 | 13 |

What that says:
- **Nine candidates a window is fixed**, whatever the band: 5 + 2 + 2 picks, each mixed, peaked up (channel 0) and
  gated. On noise 6.5 % of windows still pass the gate once, and a candidate that passes runs all four rungs.
- **The retro re-sweep and the rebuilds multiply the front end**: a decoded frame costs three more windows' analytic,
  surface and candidates, plus a surface rebuild in its own window. The sample recording does 1.7 surfaces a window,
  six stations 2.2.
- **Most ladder calls fail**: 0.9 calls a window on the recording, 0.25 accepts. About three in four candidates that
  pass the gate are not decodable and run all four rungs.

## 3. Unit costs on the CoreS3

| unit | ms | source |
|---|---|---|
| FFT 8 192, buffer in internal DRAM / PSRAM | 4.8 / 67 | E0, measured |
| FFT 256 / 512 / 1 024 / 2 048 / 4 096 | 0.107 / 0.229 / 0.494 / 1.06 / 2.27 | E0, measured (PSRAM the same to 4 096) |
| FFT 32 768 and 16 384 (stage 1) | — | **not supported**: `CONFIG_DSP_MAX_FFT_SIZE` is 8 192 |
| one surface column, 8 192-point, internal / PSRAM | 5.3 / 73 | E0, measured (×237 = 1 257 / 17 330 per surface) |
| ladder rung 1, `f64` / `f32` metrics, PSRAM | 845 / 635 | E1b, measured |
| same, trellis in internal DRAM | 611 / 414 | E1b, measured |
| full ladder, rung-1 success / all four fail (`f32`) | 651 / 2 100 | E1b, measured |
| `shift_frequency`, 14 160 samples, PSRAM | 12.0 | this note, measured |
| `peakup` | 100 | this note, measured |
| `correlate_payload` | 6.9 | this note, measured |
| `subtract_frame` | **1 832** | this note, measured |
| sync gate (9 984 complex MAC) | ~2 | estimated from `correlate_payload` |
| picks (about 0.8 M compares) | ~30 | estimated |

Two of those are startling. `subtract_frame` is 1.8 s: it builds `Complex64` prefix sums and calls `from_polar` in
`f64` for every one of 11 328 samples, three times. `peakup` is 100 ms, times nine a window.

## 4. The current algorithm, on this board, per window

| | noise only | busy (sample recording) |
|---|---|---|
| surface builds × 1 257 ms | 1 257 | 2 175 |
| peak-ups × 100 | 500 | 918 |
| mixes × 12 | 108 | 186 |
| gates × 2 | 18 | 31 |
| picks | 30 | 50 |
| ladder (rungs × measured cost) | 137 | ~1 570 |
| subtractions × 1 832 | 0 | 1 337 |
| payload correlation | 0 | 6 |
| **total** | **≈ 2.1 s** | **≈ 6.3 s** |
| against 0.472 s | 4.4× | **13×** |

(and the analytic stage cannot run at all). Where a busy window's time goes: surface 35 %, ladder 25 %, subtract
21 %, peak-up 15 %, the rest 4 %. Two cores would give at most 2× of that. Nothing in the algorithm as it stands
fits; the question is what it takes to make it fit and what that costs in behaviour.

## 5. A budget for a receiver that fits

Per window, per stage, what each change would make it, and what has to be shown before it is believed. "Host"
means it can be built and checked on the host (equality with the current decoder, recall on `jtty_sweep`,
the mixtures); "device" means it needs the board.

| stage | change | est. ms per window (busy) | to show |
|---|---|---|---|
| analytic | streaming Hilbert FIR + decimate to 6 kHz, one continuous stream cached in a ring, so retro windows reuse it | ~30 | host: same frames; device: cost |
| surface | build once per window over the union band decimated to 750 Hz (1 024-point FFT, 0.494 ms) = 126 ms per build; column step 4 ms instead of 2 ms halves it to ~63 | 63–160 | host: recall equal to the reference surface |
| surface rebuilds and retro | no rebuild after subtraction, retro re-sweep off or limited to strong frames | −(0.7 to 1.2 builds) | host: what the weak-under-strong test and the mixtures lose |
| peak-up | phase-slope estimate from the gate's 13 phasors instead of 11 mixes and a sliding search | ≤ 10 | host: recall, timing error |
| mix to DC | correlate against tone references rotated to the candidate's frequency; no whole-window mix, no allocation | ~15 | host: bit-level differences only in the gate's near-ties |
| subtract | `f32`, phasors by recurrence, filter at 2 ms resolution | ~15 | host: mixtures and the hard set |
| ladder | a cheap reject before it (three in four calls are hopeless), `f32`, workspace reserved in internal DRAM, split keys, a specialised L=1 loop | ≤ 250 | host: bit-exact words; device: cost |
| picks | scan the decimated surface | ~10 | host |
| **total** | | **≈ 400–500 ms** single core | |

**That is 1.0× the budget on one core, before any dual-core split, on estimates.** It is a design target, not a
result: only the stages marked host can be checked without the board, the analytic and surface figures are
extrapolated from FFT timings, and the ladder figure is a hope (E1b got 2×, this asks for 6×).

## 6. Using the second core

The S3 has two. The repo already has the pattern (`dual_core.rs`, `fst4_dual_core.rs`, `wspr_dual_core.rs`); FT4's
dual-core attempt gave 1.17×, so the split has to follow the data flow, not the code.

The receiver already separates the work that depends on message state from the work that does not: `Receiver::prepare`
(analytic signal and sync surface of a window) is state-independent, and the host batches it on rayon's pool. That is
the natural pipeline:

- **Core B: `prepare(window k+1)`**, the analytic signal and the surface. About 30 + 126 (or 63) ms.
- **Core A: `step(window k)`**, picks, gates, the ladder, subtraction, assembly, about 250–350 ms.

The stages are balanced within about 2× and the hand-off is one window's analytic signal and surface (113 KB and about
150 KB). Latency is one window (0.472 s) more, which is invisible in a chat receiver. Variations, in order of how
much they buy:

1. **Split the ladder's rungs across the cores** when a candidate reaches the ladder: the host already runs the four
   rungs concurrently (`rayon::join`); on the device rung 1 on A and rung 2 (then 4, half) on B halves the time a
   hopeless candidate takes, from roughly `t₁+t₂+t₄+t₅` to `max` — but only when B is idle, and B is the front end's.
2. **Candidates in parallel** (the host's `par_iter`): nine candidates a window, all but about one cheap; worth it only if
   the peak-up and mix are still the expensive part, which the budget above removes.
3. **Throughput, not latency**: the load is bursty (noise windows are 4 ms on the host, busy ones 13), so a queue of two or
   three windows between B and A absorbs the difference; a full queue drops the oldest window, which loses the frames that
   start in its quarter (a sticky retry still catches continuations of an active message).
4. Keep the ladder's 100 KB of tables and the FFT scratch in **internal DRAM** by reserving both at boot in JTTY mode from
   the arena (the placement result of E0/E1b: 2× on the trellis, 14× on the FFT).

What the second core cannot fix is a serial dependency: a frame decoded in window k is subtracted before window k+1's
surface is *used* only if the retro re-sweep is kept, and that couples the cores. Dropping the retro re-sweep (a
single-signal receiver, `Params::subtract = false`) removes the coupling; keeping it means A sends B "rebuild window k-1
without this signal" and waits.

## 7. Open, and what is next

Measured: FFT sizes and placements, the surface column, the ladder in three placements, the DSP units above.
Estimated only: the analytic stage, the sync gate, picks, everything in the right-hand column of section 5.
Not measured at all: L2, L4 and half-symbol rungs separately, the decimated surface on the device, the Hilbert FIR,
a whole `Receiver` on the device.

Next, in this order: (1) host experiments that change no output — rotated references, cached analytic signal,
`subtract_frame` in `f32`, trellis loop restructuring — each checked bit- or frame-exact against the current receiver;
(2) host experiments that trade behaviour for cost — the decimated surface, the phase-slope peak-up, the ladder
pre-reject, retro off — each measured on `jtty_sweep`, the mixtures and the hard set; (3) then, and only then, the device.

## 8. What the host showed next (same day)

Three things came out of the counters, one of them a surprise.

**Who the ladder's rejects are.** A hypothesis first: the candidates that pass the gate and fail the ladder are
the *sidelobes of a signal that has just decoded*, gated against the signal because a pass is decoded at once.
`Params::sequential` (candidates one after another, each against what the earlier ones left, upstream's order)
tests it: ladder calls fall by only 10 % (0.90 → 0.82 a window on the recording, 1.30 → 1.19 with six stations).
Wrong. The counters say what they are: of the rejected gated candidates, none lies within 40 Hz and 0.1 s of an
accepted frame and 81–97 % lie within 120 Hz and 2 s of one, median 7–8 sync tones right at 5.2 dB where the accepted
ones have 12–13 at 8–12 dB. They are **the tails of frames in the windows after the one that decoded them**: a frame
is 1.888 s, a window starts every quarter of that, so a transmission is in up to four windows and in three of them only
its tail is there, which the sync search finds partial matches in. Subtracting the frame in its own window does
nothing for the next window, whose analytic signal is computed afresh. (On 19 vendored non-JTTY recordings, 1 350 s,
the ladder is called 0.067 times a window, all rejects: the noise floor of the mechanism, one call per 15 windows.)

**`Params::carry`** subtracts a decoded frame from the later windows it overlaps (a persistent residual, as a
receiver with a continuous analytic stream would keep). Ladder calls a window: recording 0.90 → 0.45, three stations
0.54 → 0.35, six stations 1.30 → 0.74; ladder time on the host 4.6 → 1.5 ms on the recording. On a device with a
continuous ring the subtraction is done once per frame, not once per later window.

And it **finds more**. Against WSJT-X's own simulator through `scripts/jtty_multi_study.sh`, weak stations recovered:

| | rjtty | this crate, default | `sequential` | `carry` | `carry` + `sequential` |
|---|---|---|---|---|---|
| easy, 100 pairs | 69 | 69 | 69 | **95** | **95** (0 messages only rjtty has) |
| hard, 200 pairs | 103 | 101 (2 only rjtty) | **103**, identical | 108 | **110** |

`sequential` closes the whole schedule gap of P3 (hard set: identical to `rjtty`, message for message; the 2 it lacked
were D4's price). `carry` is not in upstream: in the windows after a strong station's frame decodes, its tail still
masks the weak one, and upstream never takes it off. It is off by default; the single-station `jtty_sweep` (360
files) is byte-identical with either option, since one frame has no later window to matter in.

**`subtract_frame` in `f32`.** The reference now comes from `tx::Synth` (`Synth::at`, `fill_complex`) and the `cos²`
filter is three sliding sums over `f32` phasor tables with `Σw = N/2` in closed form: no `f64`, no per-sample
`sin`/`cos` beyond the reference's. It agrees with the `f64` form to 1e-3 of the frame per sample (tested against
the kept `subtract_frame_f64`), passes the `cos²`-convolution test at 1e-4, and the studies above are unchanged by
it (69/101 and 95/110). The 1 832 ms of section 3 was that function's 50 000 software `f64` `sin`/`cos`; the `f32`
form is a few million cycles, **not yet measured on the board**.

Revised picture for section 5: `carry` removes 45 % of the ladder calls (the largest single lever on the busiest
band), `sequential` costs parallelism the device does not have, and the subtraction line drops from 21 % of a busy
window to something small pending the measurement.


## 9. The embedded-v1 policy, measured on the host (2026-09-27)

The policy of section 8's end — channel 0 only, no subtraction, a **mask** of decoded frames in place of `carry`, a
**decimated** channel-0 sync surface — had two measurable claims. Both were tried as throw-away code (env switches, not
merged; the patch is not kept because the numbers are the point). Corpora as before: `scripts/jtty_multi_study.sh` easy
(100 pairs) and hard (200 pairs), the 360-file `jtty_sweep`, `jtty_profile`.

**Mask** (zero the sync surface within ±W Hz of a decoded frame's `f1`, for start times from `tsync − 0.1 s` to
`tsync + 1.888 s − 0.1 s`; `Params::mask`-shaped, applied before picking). Weak station recovered / ladder calls a window
(recording, 3 stations, 6 stations); default: 69 / 101, 0.90 / 0.54 / 1.30:

| | easy | hard | recording | 3 st. | 6 st. |
|---|---|---|---|---|---|
| mask ±5 Hz | 68 | 100 | 0.88 | 0.46 | 0.95 |
| mask ±20 Hz | 68 | 93 | 0.60 | 0.38 | 0.61 |
| mask −15/+100 Hz | 64 (A: 91) | 78 | | | |
| mask ±120 Hz | 50 (A: 80) | 53 | | | |
| no subtraction | 42 | **0** | 0.55 | 0.38 | 0.54 |
| no subtraction + mask ±5 Hz | 29 (±120) | 0 | 0.55 | 0.32 | 0.47 |

- The rejects are 80–97 % *within 120 Hz and 2 s* of a decoded frame but only 0–11 % within 40 Hz and 0.1 s. A mask
  wide enough to remove them also removes the **other station**: two stations 25–120 Hz apart are the ordinary case, and
  at ±120 Hz the strong one is lost too (easy A 100 → 80) because the weak one decoded first masked it. The mask trades
  the weak station's recall for ladder calls at about 1 : 1 above ±5 Hz. **It is not a substitute for `carry`**, which
  removes 45 % of the calls and *gains* 26 weak stations.
- Without subtraction the ladder calls drop by half (fewer residual phantoms) and the weak station under a strong one is
  gone: hard set 101 → 0. A v1 without subtraction is a single-station-per-slot-region receiver by construction.
- Noise alone is untouched by any of this (0.065 ladder calls a window); the mask/subtraction levers only matter on a busy
  band.

**Decimated channel-0 sync surface.** The product of window and sync wave is 2 496 samples (13 symbols); it is summed
`D` at a time after mixing the band centre to DC, and transformed with an `8192 / D`-point FFT, so the bins stay on the
same 0.732 Hz grid. Decisions: `D` = 8, 16, 32 give the **same pass/fail on all 360 sweep files as the full 8192-point
surface** (0 differing), and 77 against 78 weak stations on the hard set (200 pairs). Host surface time 1.66 → 0.47 ms a
window (single thread). On the LX7 the point is not the 3.5× fewer flops but that a 512-point transform and the 2 496
product samples fit the data cache where the 8192-point one did not (section 3: 14× in PSRAM); the estimate is
237 × (2 496 multiply-adds + one 512-point FFT) ≈ 90 ms, **not measured on the board**.

**Channel 0 alone costs sensitivity, which was not in the estimate.** `jtty_sweep`, weak-signal passes (of 360):
default 160; channel 0 only 141 (−19, all between −17 and −14 dB); the same with the decimated surface 141. It is *not*
the surface width (141 with the wide surface) and *not* the candidate count (3, 9, 14 picks: 141). It is the peak-up:
skipping it on channel 0 gives 152 with channel 0 only, and 154 (against 160) with all channels — channels 1 and 2 take
their candidates raw, so on a weak signal they are an unrefined second attempt at the same peak. Peak-up (100 ms × 5 a
window on the board) buys 6 passes with the second attempt present and costs 11 without.

**Raw first, peak-up second — measured.** Channel 0 tries every pick unrefined; only when that attempt fails is the
pick refined and tried again (the decimated surface, channel 0 only in all rows). `jtty_sweep`, weak-signal passes of
360 (0 unexpected decodes in every row):

| channel 0 | passes |
|---|---|
| peak-up only (as v1 stood) | 141 |
| raw only (no peak-up) | 152 |
| default, all three channels | 160 |
| **raw, then peak-up** | **164** |
| raw, then peak-up only if the raw gate saw ≥ 6 of 13 sync tones | **164** |
| … ≥ 7 tones | 161 |
| all three channels, raw then peak-up | 166 |

The hard multi-station set, weak station: 77 (peak-up only) → **92**, and 11 more than `rjtty` finds; easy set and
unexpected messages unchanged (1 in every variant, the same one). So the fallback restores the 19 passes channel 0
alone lost and gains 4 over the default, and it is not the peak-up that has to be paid on every pick: a sync count of
6 in the raw gate (noise expects ~3.3 of 13) is a sufficient reason to refine, and threshold 6 loses nothing against
refining every failure. Cost per window, noise alone (default channel 0 with peak-up → raw with fallback at 6):
peak-ups 5.0 → **1.8**, gate calls 5.0 → 6.8, ladder calls 0.073 → 0.098 (+34 %: raw picks that pass the gate now
reach the ladder as well); six stations 10.3 → 4.2 peak-ups, ladder 0.94 → 1.34. Each gate call here is preceded by a
whole-window `shift_frequency`; the raw gate needs only the 13 sync symbols' worth of samples, so with the rotated
reference of section 8's list it is 2 496 multiply-adds a tone instead of a 14 160-sample mix — the gate count is
cheap, the extra ladder calls are the price (~0.025 a window in noise, ≈ 1 s each on the CoreS3).

**Implemented** as `Params::ch0_only`, `decimate_sync`, `raw_first` and `Params::embedded()` (default off; the
first two are the flags of the study, the third refines only above 6 of 13 sync tones). `jtty_sweep` with
`MFSK_JTTY_SWEEP_EMBEDDED=1` gives 164/360, the hard set 92/200, as measured; the regression test
`embedded_search_options_decide_like_the_full_surface_and_do_not_lose_the_weak_station` keeps the properties.

Next: the rotated-reference candidate path (no whole-window `shift_frequency`), then the board — `jtty-bench` with
`Params::embedded()` for the surface, the gate and the peak-up.

## 10. On the board: rotated references and `Params::embedded()` (2026-09-27)

`jtty-bench` part 5 (`Receiver::bench_window` / `bench_surface`, `#[doc(hidden)]`): everything a window costs after its
analytic signal, from synthetic noise (unit variance per component) and the same with one frame at 1500 Hz, about 12 dB
in the symbol bandwidth. `f32` trellis metrics, ladder in PSRAM. Logs: `m5stack-cores3-app/logs/jtty_bench_rotated_*` and
`jtty_bench_rotated2_*`.

| | before (section 3/4) | now |
|---|---|---|
| candidate gate: whole-window mix + gate | 12 + ~2 ms, 113 KB allocated | rotated references + 13-symbol gate **1.6 ms**, nothing allocated |
| `peakup` | 100 ms | **42 ms** (shifts the ~3 000 samples it reads into a 24 KB buffer) |
| `subtract_frame` (not used by v1) | 1 832 ms | 200 ms (`f32`) |
| sync surface, default (8 192-point, PSRAM) | 17.3 s | 17.7 s |
| sync surface, `embedded()`, whole sync wave as reference | est. 90 ms | 547 ms |
| sync surface, `embedded()`, four 192-sample tables + a phasor per symbol | | **172 ms** |
| noise window, default search | | 21.4 s |
| noise window, `embedded()` | | **321 ms** (68 % of 472) |
| window with a frame that decodes, `embedded()`, no subtraction | | **991 ms** (321 + 670: the ladder's rung-1 success) |
| the same with subtraction | | 1 566 ms |
| the same, default search | | 43.7 s |

- The first `embedded()` surface was 6× the estimate: 2.1 ms of each 2.3 ms column was the 2 496 multiply-adds, whose
  reference (20 KB) and window (20 KB) together exceed the 32 KB data cache. Splitting the reference into its four tone
  tables (6 KB) gave 3.2×. That is consistent with the cache explanation and was not isolated further (the tables were
  not varied one at a time). The 172 ms is still twice the estimate; the transform itself is 0.23 ms × 237 = 54 ms.
- Decisions did not change: `jtty_sweep` 164/360 for `embedded()`, none differing from the version before the
  restructuring, multi-station studies as before.
- A window with nothing to decode now fits (321 ms). A window in which a candidate decodes does not: 991 ms, of which
  the ladder is two thirds. A frame decodes once in its four windows; the other three have its tail, which the sync search
  finds partial matches in (section 8) and the ladder rejects at up to 2.1 s a call. `carry` is what removed those on the
  host and it needs the subtraction v1 does without; `Params::mask` (section 9) does not substitute for it. So the next
  item is the ladder's cost (rung 1 alone 414 ms with the trellis in internal DRAM), not the front end.
