# FT8 at 8 kHz: the analysis window, the coarse statistic and the position choice (design, from the ft8dec gap analysis)

Mode: design. 2026-11-26. **Everything here is host data** (Apple M-series, `fixed-point` build, 8 kHz input, the S3 flow
`run_slot_s3` of `hosttest/icap-app`, `icap-profile --rate 8000 --native --low-mem 1 --s3`), measured on one corpus with
experiment builds that are **not in the tree**. Nothing was run on an Xtensa core, at 12 kHz, in the f32 build, or on
noise-only slots (tier C). Read section 7 before treating any number as a result for a release.

## 1. The question

The S3 flow (`S3_GATE_OSD_ABLATION.md`) decodes fewer of the real recordings than `ft8dec` (icap-sdk `apps/ft8/ft8dec.c`,
MIT, float, the radio's own decoder). Where does it lose, and what is the cheapest change that closes the gap?

Corpus (all real): `ft8_lib/test/wav/20m_busy` (38 slots) and the 22 recordings with `jt9` v3.3 beta6 references
(`tests/fixtures/ft8_lib_jt9_beta6.tsv`): **1 338 reference decodes**. `ft8dec`, one pass (no subtraction), 8 kHz, decodes
**1 026** messages on it, 990 (96 %) of them also in the references; that set is the yardstick below ("ft8dec's 1 026").
`extras` = decodes that are not in the references. `max_cand` 60 unless said.

## 2. Where the S3 flow loses (shipped state, 60 candidates)

Hits 896 of 1 338 (67.0 %), extras 19, **879 of ft8dec's 1 026 (147 missing)**. The 147, by the stage that loses them
(measured at `ft8dec`'s own frequency / time of each message):

| stage | lost |
|---|---:|
| no coarse entry at the true place, at any score | 38 |
| coarse score below `sync_min` 1.3 | 37 |
| cut by the 300-by-score pool (`HARD_SYNC_POOL`) | 22 |
| beyond the first 60 by hard sync | 2 |
| reaches stage 3, fails at our fine-synced position | 50 |

Not the causes (each measured): fixed-point quantisation and the 80 ms rows (the f32 build, 40 ms rows, raises the hard-sync
count at the true place by 1-2 of 21); a strong neighbour within 7 Hz (18 % of the lost against 7 % of the decoded); the
distance of the true frequency from the 3.125 Hz bin grid (7-11 % lost in every 0.4 Hz class); the LLR effort, OSD and the BP
iteration count (`Full`, `Full + jitter`, BP 100 iterations rescue 0 of 34); the 4 Hz / 40 ms dedupe (0 of 38); the lag
window (`coarse_sync_gated` ignores `lag_s` anyway).

## 3. The causes found, in the order they were found

### 3.1 The analysis window (the largest)

`ft8dec` measures every tone with a **Hann window over two symbols**. The coarse spectrogram and the Goertzel symbol DFT here
use a **rectangular window over one symbol**. Hard-sync count (of 21 Costas symbols) at `ft8dec`'s own position, from the
audio, 8 kHz:

| window | the 97 lost before stage 3 | 100 decoded signals |
|---|---:|---:|
| rectangular, one symbol (ours) | 10 | 18 |
| Hann, one symbol | 17 | 19 |
| Hann, two symbols (ft8dec) | 17 | 19 |
| rectangular, two symbols | 8 | 17 |

(`ft8dec`'s own count for the 97: 17.) The loss concentrates in a subset of signals (Hann gains 4 or more on 181 of 1 026, 79
of them among the 97), independent of the frequency offset and of strong neighbours; **why that subset is window-sensitive is
not understood** (leakage of distant strong signals is a hypothesis, untested). Data tones wrong at `ft8dec`'s position (of
58): the 56 signals our back end could not decode there, rectangular 36, Hann 1T 20, Hann 2T 18; for decoded signals 2 / 1 / 0.

The back end needs the same window: Hann in the coarse spectrogram alone, or in the symbol DFT alone, buys little
(measured, ft8dec's 1 026: rect + rect 879; Hann 2T coarse only 905; Hann 2T back end only 915; rect coarse + Hann 1T back end
884; both Hann 1T 936; both Hann 2T 981).

### 3.2 The coarse statistic

`sync8` (the port; its frame is one symbol rectangular, zero-padded to two) scores a carrier by `sum(Costas-tone power) / mean(other six tones)` over the 21 symbols: a **sum of
powers**, dominated by the few symbols with the most energy. `ft8_lib`'s search scores `mean(dB contrasts)` of the Costas
tone against the adjacent tones and the adjacent symbols on an **8-bit dB** waterfall, over **every** (time, frequency) cell,
and keeps the best N. Rank of the 23 signals the Hann-2T coarse still loses, among all positions of the slot (own waterfall,
Hann 2T, 80 ms rows, 3.125 Hz bins, non-maximum suppression +-2 bins / +-2 rows):

| statistic | the 23: top 60 | 981 decoded: top 60 |
|---|---:|---:|
| `ft8_lib` (frequency + time contrasts, dB) | 22 | 99.8 % |
| frequency contrasts only / time contrasts only | 20 / 19 | 99.6 % / 99.7 % |
| dB ratio against the mean of the other seven | 19 | 99.7 % |
| linear ratio, each symbol clipped at 10 | 22 | 99.8 % |
| **linear ratio, summed (`sync8`)** | **0** | **82.6 %** |

The dB domain, or any per-symbol limit, is what matters; a drop-in clip inside `sync8`'s own loop did **not** carry over
(Hann 2T: ft8dec's 1 026: 981 -> 980 at clip 3-5, 970 at 10; +9..+18 / -17..-50 against the current, the 40 %-percentile
normalisation and `sync_min` 1.3 not retuned). Rebuilt on `ft8_lib`'s statistic it does (section 5).

### 3.3 The fine sync moves some signals away

`fine_sync_at` is a faithful `ft8b.f90` Stage A/B/C (rectangular, coherent). For 16 signals whose coarse place is within
2.7 Hz and 60 ms of the truth it lands further away (e.g. coarse 1687.50 Hz, truth 1685.94, fine 1690.00) and decodes 1 of 16,
while the coarse place decodes 10 of 16. Over all 1 026 (decoding each candidate alone): coarse place 984, fine place 993,
**either 1 007**. The cause is not known (drift / fading / the rectangular coherent objective are untested).
`ft8dec` retries a failed candidate once with interpolated tone powers (`interp`, 23 of its 1 026 decodes come from it).

### 3.4 Smaller

`HARD_SYNC_POOL` 300 -> 600: +10 decodes at 60 candidates (896 -> 906 hits), none at 15; 1 000 and 3 000 add nothing.
A message with `/R` (`ET3RFG/R IN3ADG -23`, 2 of the remaining) is dropped by `ft8b.f90` itself without a contest
(`wsjtx_quirky`): `jt9` does not report it either; not a gap to WSJT-X.

## 4. The measured ladder (60 candidates; hits of 1 338, extras, ft8dec's 1 026)

| configuration | hits | extras | ft8dec's 1 026 | missing |
|---|---:|---:|---:|---:|
| shipped (rectangular + rectangular, `sync8`, fine sync) | 896 (67.0 %) | 19 | 879 | 147 |
| Hann 1T coarse + Hann 1T back end | 932 | 20 | 936 | 90 |
| Hann 1T coarse + Hann 2T back end | 976 | 24 | 964 | 62 |
| **Hann 2T coarse + Hann 2T back end** | 995 (74.4 %) | 25 | 981 | 45 |
| ... + run every candidate again at its coarse place | 1 002 | 34 | 993 | 33 |
| ... + choose coarse / fine place by the Costas energy ratio (21 symbols), one decode | 999 | 26 | 988 | 38 |
| **`ft8_lib` coarse (section 5) + fine sync** | 1 009 (75.4 %) | 35 | 999 | 27 |
| ... + choose grid / fine place by the Costas energy ratio (21 symbols) | **1 013 (75.7 %)** | 40 | **1 007** | **19** |

At 15 candidates the gain is small and the `ft8_lib` order is *worse* (shipped 710 / 702; Hann 2T `sync8` 738 / 736;
`ft8_lib` coarse 733 / 722): it has no hard-sync re-rank of the top of the list.

Host time per slot (this Mac, the experiment builds; **not an embedded cost**): shipped 24 ms, Hann 2T 33 ms, `ft8_lib` coarse
36 ms, run-twice 62 ms. The position-choice builds were unoptimised experiments (100-275 ms) and have no honest time here.

## 5. The design

Three changes, separable, each measured on top of the previous. All three are **deliberate divergences from WSJT-X** (which
this path mirrors elsewhere) and must carry that in a comment where they land; they apply to the **8 kHz S3 flow**
(`Ft8Rate::Hz8000`, fixed-point), not to `Decoder<Ft8>` (12 kHz, f32), until measured there.

### 5.1 Hann 2T windows in both places

* **Coarse spectrogram**: the frame is the two-symbol `nfft` samples (2560 at 8 kHz) centred on the row's symbol
  (start `nsps / 2` earlier), weighted by `sin^2` (the frame is `nsps` zero-padded to `nfft` today). No amplitude
  compensation (x2 changed nothing). The FFT size, the two-frames-per-FFT trick and the 80 ms step are unchanged; what
  changes is the samples read per frame (2x) and a window table.
* **Symbol DFT (stage 3)**: the Goertzel runs over `2 * nsps` samples centred on the symbol with the same window: **2x the
  stage-3 DFT work**. Validated for `LlrEffort::Minimal` (`|cs|` only); the coherent multi-symbol LLRs (`nsym` 2 / 3, `Full`)
  assume independent one-symbol DFTs and are **not validated** with overlapping 2T windows.
* Gain at 60 candidates: ft8dec's 1 026 879 -> 981, hits 896 -> 995, extras 19 -> 25.

### 5.2 The candidate search on `ft8_lib`'s statistic

* An 8-bit dB waterfall (0.5 dB units, 3.125 Hz bins, 80 ms rows; about 190 rows x 936 bins = 178 KB, against the u16
  spectrogram) and the integer differential score (adds and subtracts, no division, no per-carrier percentile).
  **The cells must be made from the FFT's 32-bit power, not from the u16 spectrogram**: the `>> FP_SPEC_SHIFT` (12) of the
  shipped cell throws away the noise floor, and the u16 range cannot hold the floor and the strong signals together.
  Measured (Hann 2T, 60 candidates, ft8dec's 1 026): dB of the shipped u16 cell 933 (shift 12), 962 (11), 982 (10), 984 (9),
  964 (8), 903 (6); dB cells from the 32-bit power (leading-zero count + a 16-entry table, 0.5 dB) **1 002**; the float
  waterfall of the first experiment 999. So the demux of the two-for-one FFT (`spectrogram.rs`) writes the dB cell
  (`20 log10 p` in 0.5 dB, 0..255) instead of `p >> 12`, and nothing else reads the linear cell on this path.
* Every (time, frequency) cell of DT -2.26..+2.46 s (`ft8dec`'s own range) and 200..2950 Hz, score >= 8, **best `max_cand`
  after a non-maximum suppression of +-2 bins and +-2 rows** (`ft8dec` has none, it decodes up to 300 and skips duplicates after
  a success). A range wider than `ft8dec`'s finds real signals at DT +2.6 s that WSJT-X's own window does not: keep the
  range, or move it on purpose.
* A hard-sync floor on top (7-11 of 21) changes nothing.
* Gain over 5.1: ft8dec's 1 026 981 -> 999, hits 995 -> 1 009, extras 25 -> 35; the 23 coarse-stage losses 0 -> 16.

### 5.3 The position choice

Per candidate, evaluate the Costas-tone energy over the eight tones at the grid place and at the fine-synced place (all 21
Costas symbols, Hann 2T) and decode **once** at the higher. Retrying the decode at the other place instead gives more (993
against 988 on the `sync8` coarse) but doubles the decode cost; the choice costs the 21 symbols x 8 tones x 2 places of DFT
per candidate (Goertzel, 2T) instead.

* Gain over 5.2: 999 -> 1 007 (missing 27 -> 19), hits 1 009 -> 1 013, extras 35 -> 40, one decode lost.
* The ratio over all eight tones is not needed: **the expected-tone energy alone** (the numerator; one tone per symbol, so
  1/8 of the DFT work) gives 1 006 against 1 007 (hits 1 011 against 1 013). Block 0 only (7 symbols) is cheaper and worse
  (numerator 999 = no gain; ratio 1 004).
* Evaluating the choice only where the two places differ: by 0.02 s or more in time (54 % of the candidates) 1 005; by 1 Hz
  or more in frequency (43 %) 1 003; both 0.5 Hz / 0.02 s (43 %) 1 001. Each saves its share of the cost for 2-6 decodes.
* A parabolic refinement of the place (the interpolation `ft8dec` does) added 0-2 decodes over the choice and is not part of
  the design.

## 6. What remains (19 of ft8dec's 1 026 with the full design)

* 11: decodable at `ft8dec`'s place, not at the chosen one (SNR median about -15; the choice has only two places);
* 7 at the back end: 2 are the `/R` messages above, 2 are slot-edge signals (`R1CBP SP9LKP RR73` at DT -1.11 s,
  `CQ UY5AX KO70` at +1.60 s) that a rectangular window decodes and Hann does not (cause not tested), 3 are at SNR -17..-24;
* 1 not in the first 60.
Of the 19, 10 are in the `jt9` references: **against WSJT-X the gap is about 10 of 1 338, not 19 of 1 026.**

## 6a. Computational cost, and the trade-offs it allows

Instructions retired per slot (`/usr/bin/time -l`, 12 busy slots of `20m_busy`, difference of 6 and 1 repetitions so start-up and
file reading drop out; fixed-point build, 8 kHz, Apple M-series). **A host instruction count, not an Xtensa cycle count**:
ratios carry over, absolute values do not (the CoreS3 spends about 72 ms a candidate on BP + refine and has 810 ms before the
slot boundary, `CORES3_FT8_SLOT_BUDGET.md`). The experiment builds are unoptimised where marked.

### The cost model of the shipped flow

`instructions per slot = fixed + candidates x per_candidate`, fitted over 1, 15 and 60 candidates:

| configuration | fixed | per candidate |
|---|---:|---:|
| A shipped (rectangular windows, `sync8`) | 80 M | 7.5 M |
| B Hann 2T in the coarse spectrogram and the symbol DFT, `sync8` | 99 M | 12.2 M |
| C B with `ft8_lib` on dB cells (experiment build, unoptimised scoring) | 266 M | 12.4 M |

* **Per candidate**: 7.5 M = about 4.7 M of symbol DFT (the 2T window doubles exactly that: +4.7 M, measured as the 2T back end
  alone, 816 against 534 M at 60 candidates) + about 2.8 M of refine, LLR, BP and OSD. Hann 2T costs **+62 % per candidate**.
* **Fixed**: the Hann 2T frames add 19 M (twice the samples read and a window multiply per frame; the FFT, its two-for-one
  trick and the 80 ms step are unchanged, so the FFT cost is the same).
* **The `ft8_lib` coarse** measures +167 M fixed here, but that is the experiment's own: bounds-checked indexing in the scoring
  (910 bins x 60 rows x 21 symbols x up to 4 contrasts = 4.6 M inner terms at ~25 instructions each), a division per position,
  Vec allocations. The work itself is integer adds: **a careful implementation (row pointers, the term count known in closed
  form, no division, a histogram threshold) is estimated at 20-40 M, unmeasured**; the cell conversion (leading-zero count and a
  table, 190 x 936 cells) is about 1-2 M. The sync8 scoring it replaces (896 carriers x 63 lags x 21 symbols x 2 reads, plus the
  per-carrier sums and two divisions per lag) is of the same order, so the net could be close to zero; that needs the
  optimised implementation to be measured.
* **The position choice** (8 tones x 21 symbols x 2 places, per candidate; measured as C minus the same without it): +324 M at
  60 candidates (+77 M at 15), i.e. 5.4 M per candidate, **as much as the whole 2T back end**. The expected-tone-only form is
  one tone in eight: about 0.7 M per candidate (about +40 M at 60), estimated from the work, unmeasured; evaluated for 54 % of
  the candidates only, about 0.4 M.
* **Running every candidate a second time** (the retry) doubles the per-candidate cost: about +730 M at 60 candidates (60 x
  12.2 M; the host time per slot went 33 -> 62 ms) for +12 decodes (+7 hits), against the choice's +7..9 for about +40 M.

### Recall against cost (60 slots; hits of 1 338, ft8dec's 1 026, extras; M instructions per slot from the 12 slots)

| candidates | A shipped | B Hann 2T, `sync8` | C Hann 2T, `ft8_lib` on dB cells |
|---:|---|---|---|
| 10 | 519 / 512 / 1 extra, 143 M | 520 / 515 / 5, 216 M | 516 / 514 / 6, 386 M |
| 15 | 710 / 702 / 4, 173 M | 738 / 736 / 12, 277 M | 733 / 723 / 9, 456 M |
| 20 | 821 / 808 / 10, 226 M | 886 / 878 / 19, 355 M | 876 / 866 / 18, 534 M |
| 30 | 893 / 877 / 18, 344 M | **992 / 980 / 24, 541 M** | 1 000 / 988 / 31, 696 M |
| 45 | 896 / 879 / 19, 464 M | 995 / 981 / 25, 729 M | 1 010 / 1 001 / 35, 884 M |
| 60 | 896 / 879 / 19, 535 M | 995 / 981 / 25, 830 M | 1 010 / 1 002 / 36, 1 011 M |

(C's instruction figures include the unoptimised scoring above, about 170 M more than a careful one.)

What the table says:

* **A saturates at 30 candidates** (893 hits; 896 at 45 and 60): more candidates buy nothing; its limit is the window.
  **B at 30 candidates (992 hits, 541 M) beats A at 60 (896 hits, 535 M) for the same instructions**: the 2T windows move the
  ceiling by 96 hits at no extra cost *when the candidate count is cut to 30*.
* **B saturates at 30-45**; C needs 45 for its last 10 hits and is worse than B below 30 (733 against 738 at 15; 876 against 886
  at 20): the `ft8_lib` order has no hard-sync re-rank of the top of the list.
* At 15 candidates (the CoreS3's `max_cand`, saturated every slot) the 2T windows give +28 hits for +103 M (+60 %); at 30
  they give +99 hits for +198 M (+58 %).
* Stage-3 candidates **are** the budget: per candidate 12.2 M (2T) against 7.5 M; the fixed part (coarse) is 99 M against 80 M.
  So the trade is mostly "how many candidates the slot can afford", and **candidates x per_candidate** is the lever, not the coarse.

### The options as a trade

| option | what it buys (60 candidates, against the row above it) | instructions per slot | memory | risk |
|---|---|---|---|---|
| **5.1a Hann 2T in the coarse spectrogram only** (**implemented**, section 9) | at 60 candidates +26 decodes (879 -> 905), +26 hits (896 -> 922), extras 19 -> 15; at 30 +28 hits; **at 15 -10 hits (710 -> 700)** | +0.4 M at 60 candidates, +25 M at 30, +17 M at 15 (production build, 12 slots): the 2T frame read and a window multiply, the FFT unchanged | the window table | none beyond tier C; **pays from about 20 candidates, loses below** |
| 5.1b Hann 2T in the symbol DFT only | +36 decodes (879 -> 915), +38 hits (896 -> 934), extras 19 -> 17 | +281 M (+53 %) | the window table | not validated for coherent LLRs |
| **5.1 both** | +102 decodes (879 -> 981), +99 hits | **+295 M (+55 %)**; at 30 candidates the same ceiling as A at 60 | window tables | extras 19 -> 25: tier C |
| 5.2 `ft8_lib` coarse on dB cells | +21 decodes (981 -> 1 002), +15 hits | measured +181 M (unoptimised); estimated -10..+40 M | the 8-bit waterfall (178 KB) | needs the dB cell from the 32-bit power; worse than B below 30 candidates |
| 5.3 choice, 8 tones, both places | +8 (999 -> 1 007) | +324 M | none | as much as the back end: not worth it as built |
| 5.3 choice, expected tone only | +7 (1 006) | about +40 M (estimated) | none | evaluate where the places differ: 54 % of candidates for 1 005 |
| retry the decode at the other place | +12 (981 -> 993) | about +730 M (stage 3 twice: 60 x 12.2 M; the host time went 33 -> 62 ms) | none | not recommended |

Order by gain per instruction: **5.1a (the coarse window: +26 for about nothing, at 20 candidates or more; at the CoreS3's 15 it
loses 10)**, then **5.1b at 30 candidates** (the
back end window: +71 more for +53 % per candidate, paid for by halving the candidates); then **5.3 in its cheap form**; then
**5.2** if its implementation is as cheap as the operation count says. Where the budget is fixed (15 candidates, as on the CoreS3 today), 5.1
costs +60 % per slot for +28 hits; the alternative that reaches the same ceiling cheaper is **30 candidates with 5.1
instead of 60 without**.

## 7. What was and was not checked, and what must be before this lands

Extras (35 -> 40 with all three): each new one was checked against `ft8dec` (3 passes), against the other slots' references,
and by the wrong-tone count of the decoded message over a (time, frequency) scan at the decoded frequency (noise: at least
40 of 58 wrong; real: 0-25); a hashed partner call was resolved by trying the 168 callsigns of the references
(`<IZ7NLM> ZY50Y 73`: 25 wrong, next 37). **All checkable extras are real signals `jt9` missed.** That is not a false-decode
rate: **no noise-only slot was run** (tier C is on the Linux box, `ft8_busy_sweep` 2 400 slots), and the position choice and
the 2T window each add CRC chances.

Not done, in order of what blocks landing:

1. **Tier C noise-only slots** for each stage (`CONTRIBUTING.md`, a new strategy ships with its precision guard); the
   `FrameDecodable::MESSAGE_FILTER_DEFAULT` verdict stays on.
2. **The boards** (the host instruction counts of 6a are not cycles; the optimised `ft8_lib` scoring and the single-tone choice
   were estimated from the work, not built): the 2T Goertzel doubles stage-3 DFT work and the 2T frames read twice the samples; `stage1_inc`
   (two-frames-per-FFT on the radio's stream) and `esp_dsp_fft.rs` must produce Hann 2T frames; the 8-bit waterfall replaces
   the u16 spectrogram (memory and time unmeasured). `embedded-poc/**` is not built by host CI.
3. **12 kHz, f32 and `Decoder<Ft8>`**: unmeasured; the host path is the WSJT-X-faithful one (principle 1).
4. **The `jt9` fixtures (tier B)**: a new strategy needs a golden with `max_extra` and a recall floor for it; the 22 recordings
   and `20m_busy` are the only corpus used, and the 60 slots were also the corpus the design was tuned on (no held-out set).
5. Why the window matters for the signals it matters for, why fine sync moves some signals away, and the two slot-edge
   failures: causes not found.
6. The method is host code that was removed again (window patches, `ft8_lib` coarse, the position choice, the scans); the analysis tools
   (`diag_*.rs`) are not in the tree. Rebuild from the descriptions above; no experiment binary is in the tree.

## 8. Order if it goes ahead

0. Decide the candidate count with 6a in hand (30 candidates with 5.1 against 60 without is the same instruction count and +96 hits).
1. 5.1 (window), at 8 kHz only, `Minimal` depth: tier A (unit: window table, symmetry, the same cell for a pure tone),
   tier B (the two corpora above, `assert_golden`, extras budget as measured), tier C noise slots; then the boards.
2. 5.2 (coarse), behind the hard-sync-gated candidate list the flow already has, with its own precision guard.
3. 5.3 (position choice) last, because it depends on 5.1's window and is the smallest gain per cost.
Each stage: CHANGELOG entry (what, why, the measured effect above, this note), `reviewer` (a divergence from WSJT-X, a new
decode strategy, fixed-point), and the `Ported from...` pointers kept: the coarse statistic leaves `sync8.f90`, the fine sync
stays `ft8b.f90`.

## 9. 5.1a as built (2026-11-26)

In the tree (uncommitted): `Ft8Rate::coarse_hann_2t` (`types.rs`, `fixed-point` only, `Hz8000` only) and the frame in
`compute_spectrogram_bins_src` (`spectrogram.rs`): `nfft` samples weighted by a Q15 `sin^2` table, starting `nsps / 2`
earlier; `Hz12000` and the f32 build are unchanged (the 12 kHz fixed-point and f32 floors pass unchanged).

Tests: `tests_fixed` in `spectrogram.rs` (the 8 kHz cell of a steady tone equals the 12 kHz one's, within 3 %, as before; a strong tone
leaves the cells 25 / 50 / 100 Hz away at 0 where the rectangular frame leaves 260 / 57 / 13; a one-symbol burst is strongest in the row it
starts at; the noise cell is 0.75 of 12 kHz's; the Q15 table is the periodic Hann, symmetric, summing to `nfft / 2`) and, in `icap_profile.rs`,
`the_s3_flow_at_native_8khz_meets_its_floor` (22 recordings, floors 216 / 272 of 402, no unattested extra) and
`the_low_memory_profile_at_native_8khz_meets_its_floor`, now in both builds (fixed-point floors 221 / 284; one decode `jt9` lacks and nothing
attests, pinned by identity: now attested, see below). Mutations checked: the 8 kHz frame rectangular again fails the leak and noise tests and the S3 floor at 30
candidates (268 < 272); no `nsps / 2` lead fails the burst test; a half-sine instead of `sin^2` fails the noise test, the 12 kHz / 8 kHz cell
test and the table test (not the leak and burst tests).

**The public low-memory path moves too.** `decode_block_low_mem` / `coarse_sync_gated` at `Hz8000` in the fixed-point build reach the same
spectrogram (found by the review, which ran the existing native-8 kHz body in a fixed-point build: it had no fixed-point test, its doc said the
build panics). With the window: 221 / 284 of 402 at 15 / 30 candidates, extras 3 / 3; without: 221 / 280, extras 1 / 2. The one added extra is
`websdr_test6`'s `SM2EKA UT7IS -06` at 1615.6 Hz where `jt9` has it at 1716 Hz: the message's tones read 0 of 58 wrong at both, so the audio holds
the signal and a copy of it 100 Hz below (the review first read it as a baseband alias; it is not). It is attested (`profile::ATTESTED_EXTRAS`); the
S3 flow gives the same decode without the window at 30 candidates.

Measured on the production build against the same tree with the window off (60 slots, S3 flow, 8 kHz, host):

| candidates | window off: hits / ft8dec's / extras, M instr | window on |
|---:|---|---|
| 10 | 519 / 512 / 1, 159 M | 506 / 500 / 1, 173 M |
| 15 | 710 / 702 / 4, 198 M | 700 / 695 / 4, 215 M |
| 20 | 821 / 808 / 10, 259 M | 831 / 822 / 9, 278 M |
| 30 | 893 / 877 / 18, 392 M | 921 / 905 / 15, 417 M |
| 60 | 896 / 879 / 19, 607 M | 922 / 905 / 15, 608 M |

The reading is the one of section 6a with one correction: **the window alone loses below 20 candidates**. At 15 the first 15 by
hard sync are reordered before the rest improves (the 2T back end, 5.1b, then takes 15 candidates to 738 hits); what the
window moves is the ceiling (896 -> 922), not the low end. A board that keeps `max_cand` 15 should take 5.1a together with 5.1b or
not at all; one that can afford 20-30 candidates takes the gain.

Not done for 5.1a: noise-only slots (tier C), the boards (`stage1_inc`, `esp_dsp_fft.rs` build the rectangular frame), the f32 build at 8 kHz and
12 kHz (unmeasured, left rectangular). **SNR**: a steady tone keeps its cell but a one-symbol tone has 0.67 of it (-1.7 dB) and noise 0.75
(-1.25 dB), so a signal is about 0.5 dB weaker against the noise and `xsnr2_db_simple` (not rate-aware) would read about 0.5 dB low at 8 kHz;
unmeasured, not compensated. **WSJT-X**: `sync8.f90:35-37` is one symbol, rectangular, zero-padded to `NFFT1` (this note said two symbols in
its first version: wrong).

## 10. 5.1b as built (2026-11-26)

In the tree: `Ft8Rate::symbol_hann_2t` (`types.rs`, `Hz8000` and `fixed-point`) and `fill_hann_2t` in `fill_symbol_spectra.rs`: the stage-3
Goertzel over `2 x nsps` samples centred on the symbol, weighted by a half-window table made by rotating `(cos, sin)` once per call (the
Chebyshev recurrence first tried drifts with `k^2` in `f32` at this `theta` and failed its own test: 2e-4 by sample 114), the output
rotated back by `e^{-j w nsps/2}`, and `goertzel_window_end_sample_at` half a symbol later. `Hz12000` and the f32 build are unchanged.

**The phase.** The window starts `nsps / 2` before the symbol and the recursion's output refers to the window's last sample, so the first
version's cell was the rectangular cell times `e^{j w nsps/2}`: with a 6.25 Hz tone step and 0.08 s that is half a cycle a tone, **a sign flip
on every odd tone** (and a common `f0` term). The magnitudes were right and every magnitude-only reader (`sync_quality`, llra, llrd, the
block-0 q) was unaffected, but OSD computes the coherent `llrb`/`llrc`/`llre` whatever the LLR effort, so the measured flow read cells of
different tones with the wrong sign (found by the review, which measured 224 / 287 with the sign fixed against 223 / 286 without, on the
22 recordings). The first version's docs said "validated for `LlrEffort::Minimal`"; that was wrong for the flow measured. The cell now has the
rectangular kernel's phase (`the_hann_cell_has_the_rectangular_cells_phase`, checked against a double-precision DFT for all eight tones).
The coherent metrics with **overlapping** windows are validated by the corpora's recall only.

Measured on the final code (60 slots, S3 flow, 8 kHz, host; hits of 1 338, extras, ft8dec's 1 026, M instructions per slot from 12 slots):

| candidates | neither window (shipped) | 5.1a (coarse) | **5.1a + 5.1b** |
|---:|---|---|---|
| 10 | 519 / 1 / 512, 159 M | 506 / 1 / 500, 173 M | 523 / 4 / 516, 203 M |
| 15 | 710 / 4 / 702, 198 M | 700 / 4 / 695, 215 M | **737 / 12 / 734, 255 M** |
| 20 | 821 / 10 / 808, 259 M | 831 / 9 / 822, 278 M | 888 / 19 / 880, 327 M |
| 30 | 893 / 18 / 877, 392 M | 921 / 15 / 905, 417 M | **993 / 25 / 980, 500 M** |
| 45 | 896 / 19 / 879, 527 M | 922 / 15 / 905, 544 M | 997 / 25 / 982, 674 M |
| 60 | 896 / 19 / 879, 608 M | 922 / 15 / 905, 608 M | 997 / 25 / 982, 760 M |

* **The 15-candidate loss of 5.1a is closed**: 737 against the shipped 710 (+27) for +29 % instructions (255 against 198 M).
* **The cost is lower than the model of section 6a** (+60 % per candidate at the symbol DFT alone): the whole flow is +25..+29 %, the window
  table is once per call and the rotation 25 % cheaper than per sample, and fewer candidates reach the full DFT.
* **30 candidates with both windows (993 hits, 500 M) against the shipped 60 (896 hits, 608 M): +97 hits for 18 % fewer instructions.**
* The shipped flow saturates at 30 candidates (893 -> 896); the windowed one at 30-45 (993 -> 997).
* Extras at 60 candidates 19 -> 25 (15 added, 9 gone): 10 of the 15 are listed by `ft8dec`, 10 read under 40 of 58 data tones wrong at the
  decoded frequency (noise-only: 40 or more), 4 are hashed or special-event calls the count cannot judge (one, `<...> JH1RGG`,
  unconfirmed by anything). **No noise-only slot was run** (tier C).

Tests: `the_8k_hann_symbol_dft_reads_every_sent_tone` (all 79 symbols of four signals read as the strongest tone: fails without the
`nsps / 2` lead), `the_goertzel_cells_are_the_same_at_either_rate` (fixed-point: 0.75..1.05 of the 12 kHz cell, the window end half a symbol
later), `the_hann_cell_has_the_rectangular_cells_phase` (fails without the rotation, tone 0 included), `the_hann_recurrence_is_sin_squared`, and the
22-recording floors (`icap_profile.rs`: S3 flow 224 / 287 of 402, native low-memory 221 / 284). Mutations checked: the symbol window off fails the
window-end test and the S3 floor; no lead fails four tests of `ft8_rate_8k.rs`; a recurrence from the wrong phase fails its own test and two of
`ft8_rate_8k.rs`; no phase rotation fails the phase test. A one-sample shift of the mirror passes every test (the claim of the tone-reading test
is "grossly asymmetric").

Not done: noise-only slots, the boards (`esp_dsp` Goertzel / the board's symbol DFT), 12 kHz and f32, the SNR at 8 kHz (computed about 0.5 dB
low from the cells, not measured, not compensated), one allocation of the window table per call (45 to 90 of 5 KB a slot at 15 to 30
candidates; it depends on the rate only and could be built once).

## 11. 5.2 as built: the candidate search on `ft8_lib`'s statistic (2026-11-26)

In the tree: `coarse_lib.rs` (new; `Ft8Rate::coarse_ft8lib`, `Hz8000` and `fixed-point`), the arm in `coarse_sync_gated`, and the demux of
`spectrogram.rs` made **generic over the cell conversion** (`LinearShift`: the shipped `p >> FP_SPEC_SHIFT`, bit for bit, pinned by a checksum
test of the u16 cells at 12 kHz and 8 kHz, whole range and band; `HalfDb`: `half_db(p)`, `20 log10 p` of the 32-bit power in 0.5 dB units from the
leading-zero count and a 16-entry table in quarter units, rounded once, within 0.82 units of the exact value, at most 193).

**What it does.** Per band of carriers (the dB cells of the band are built, scored and dropped, as `banded_peaks` does): every position (carrier bin
x row `j0` = -22..37, DT -2.26..+2.46 s, `ft8dec`'s rows) is scored by the mean over the Costas symbols of the tone's dB minus the adjacent
tones' and the adjacent symbols' (integer, one division a position); the band keeps its best `12 x pool` positions by a total order (score, carrier,
row) with a histogram cut, each with its hard-sync count; the union is cut to the best `12 x pool` of that order, suppressed (+-2 bins,
+-2 rows), cut to `pool` (300), ordered by hard sync (stable) and to `keep`. The result does not depend on the number of bands, to the bit.

**Deviations from the plan.** (1) The cells stay u16 (the half-dB value is at most 193) and live one band at a time, so the memory is the banded
u16 spectrogram's, not the whole-slot 178 KB of u8 the plan counted: a u8 store of the whole slot would have been more memory, not less, than the
banded flow already has. (2) The pool (300) and the order (hard sync over the pool) were chosen by measurement, below; the plan listed three.
(3) **The instruction gate was replaced** (agreed with the user): the plan asked for at most +50 M at 60 candidates and +40 M at 15; it failed at 60
(+252 M; +40 M at 15 candidates, +54 M at 30, so it also failed the 15-candidate limit by nothing: exactly 40 M). The search itself is dearer by 45 M
(`coarse_sync_gated` alone, measured on the stage: 106.7 M with `sync8`, 151.7 M with `ft8_lib`, the same at 15 / 30 / 60 candidates), and the rest is that its
candidates are more often real and so reach the full stage-3 DFT and OSD (about 15 against 11 M a candidate, a fit). The earlier reading of the fit's intercepts
(105 against 112 M, "the search is cheaper") was wrong: the score-order variant, running the same search, fitted 142 M. The gate that means something
is instructions **at the same recall**, below, with the caveat that the `sync8` cost at 919 hits is a linear interpolation between two points.

Measured (60 slots, S3 flow, 8 kHz, host, both windows on; hits of 1 338 / extras / ft8dec's 1 026 / M instructions per slot from 12 slots):

| candidates | `sync8` order (5.1a + 5.1b) | `ft8_lib`, score order | `ft8_lib`, pool 300 -> hard sync (**built**) |
|---:|---|---|---|
| 10 | 523 / 4 / 516, 203 M | 514 / 6 / 512, 242 M | 534 / 5 / 529, 236 M |
| 15 | 737 / 12 / 734, 255 M | 733 / 9 / 722, 305 M | **761 / 11 / 754, 295 M** |
| 20 | 888 / 19 / 880, 327 M | 877 / 16 / 865, 372 M | 919 / 23 / 909, 373 M |
| 30 | 993 / 25 / 980, 499 M | 1 001 / 31 / 987, 522 M | **1 009 / 36 / 1 001, 553 M** |
| 60 | 997 / 25 / 982, 760 M | 1 011 / 36 / 1 001, 823 M | 1 011 / 37 / 1 001, 1 012 M |

The pool of 100 and of 600 gave the same hits within one (15 candidates 761 both; 30 candidates 1 010 with 100 and 1 009 with 600; 60 candidates 1 011), 919 / 1 019 M at 60.
The `ft8_lib` score order alone is below the `sync8` order at 15 and 20 candidates (733 against 737; 877 against 888): it has no hard-sync
re-rank of the top of the list, which the gated search always had. Fixed part and per candidate (fit over 1 / 15 / 30 / 60 candidates):
`sync8` 112 M + 11.1 M, `ft8_lib` hard-sync order 105 M + 15.0 M, score order 142 M + 11.6 M.

**Gates, as checked.** ft8dec's 1 026 at 60 candidates 1 001 (>= 1 000: passed); hits at 15 candidates 761 (>= 737) and at 30 candidates 1 009
(>= 993): passed. **Instructions at the same recall**: 919 hits cost 373 M at 20 candidates, the `sync8` flow reaches that between its
888 hits (327 M) and 993 (499 M), about 378 M: equal; 1 009 hits at 30 candidates cost 553 M, which the `sync8` flow does not reach at all
(997 at 60 candidates, 760 M): the ceiling moved. `Hz12000` / f32 / banded invariance tests green; the u16 cells bit-for-bit.

**Extras** (60 candidates, against the `sync8` flow): 25 -> 37 (15 added, 3 gone); at 30 candidates 25 -> 36 (14 added). Of the 15: 12 are
listed by `ft8dec` (3 passes), and all 15 read under 40 of 58 data tones wrong at the decoded place (noise-only: 40 or more); none is
hashed, none unconfirmed. Two of the 22 recordings' decodes `jt9` lacks are new in the profile and attested: `191111_110700`'s `CQ DX IK0YVV JN62`
(`ft8dec` lists it, 2 of 58 tones wrong at 2828.6 Hz: a copy 100 Hz above the signal) and `websdr_test3`'s `EY8MM YB1BZV OI33` (23 of 58 wrong;
the same station is in `websdr_test1`; not listed by `ft8dec` or `jt9`). **No noise-only slot was run.**

**22 recordings** (`icap_profile.rs`): S3 flow 230 / 289 of 402 at 15 / 30 candidates (224 / 287 with the `sync8` search); the native
low-memory profile at 8 kHz 229 / 287 (221 / 284), with no extra at 15 candidates (the `SM2EKA` copy is not decoded by it now).

**Tests** (`coarse_lib.rs`): the clean signals of a slot (one with a strong neighbour 25 Hz away) are among the first candidates at their place, the
strong one's hard sync is the whole 21, no two candidates are neighbours; the list is the same for 1, 2, 4 and 7 bands (both orders); the
selection equals a plain sort and suppression written out again in the test; every candidate is inside the rows and the band, the signal at DT +2.6 s
is not one. `spectrogram.rs`: the u16 cells' checksum, `half_db` against `20 log10` over the whole range and against the linear cell. Mutations
checked: suppression off fails the neighbour test and the plain-sort test; the DT range widened fails the range test; the cut too small fails the
plain-sort test; the adjacent-symbol term reading the wrong row fails all four; the dB table off by an octave step fails the `half_db` test.

Not done: noise-only slots, the boards (`stage1_inc` and the streamed spectrogram would have to produce the half-dB cells: the library side first), 12 kHz and
f32, the position choice (5.3), and the idea of dropping the per-candidate DFT (5.4, section 12).

## 12. Against `ft8dec`, and the per-candidate DFT (5.4, not built)

**Recall and cost against `ft8dec`** (same 60 slots, `jt9`'s 1 338; host instructions, `ft8dec` as the icap-sdk's C built `-O2`, its driver's wav read and
double-precision 12 -> 8 kHz resample (1 546 M a slot) taken out; the two are different code on one CPU: a ratio, not ESP32 time):

| | hits | extras | M instr / slot |
|---|---:|---:|---:|
| `ft8dec`, 1 pass | 990 (74.0 %) | 36 | 335 |
| `ft8dec`, 2 passes | 1 189 (88.9 %) | 68 | 850 |
| `ft8dec`, 3 passes | 1 204 (90.0 %) | 71 | 1 278 |
| S3 flow, both windows + `ft8_lib` search, 30 candidates | 1 009 (75.4 %) | 36 | 553 |
| the same, 15 candidates | 761 (56.9 %) | 11 | 295 |

At 30 candidates the flow reaches `ft8dec`'s one-pass recall (1 009 against 990 hits) for 1.65 times its instructions, and has no subtraction pass (`ft8dec`'s second pass
adds 15 points). `ft8dec`'s peak working memory was 234 KB (1 and 3 passes), the flow's 425 KB at 30 candidates (before this change).

**Why `ft8dec` is cheaper.** It decodes about 200 candidates a slot from the 8-bit dB waterfall alone: an LLR is a lookup of eight cells, about 1.7 M
instructions a candidate with BP and OSD. The flow recomputes 79 symbols x 8 tones from the audio for each candidate that passes the Costas gate
(about 10 M instructions a candidate, 63 % of it the DFT).

**How the Goertzel got here** (`git log -S fill_symbol_spectra_goertzel`; `ROADMAP.md` 0.6.4, "fine_refine attempt postmortem"; `524a9ad1`):
0.6.2 tried two ways to give the board a per-candidate refinement: one DFT per (symbol, tone) per candidate (about 100 ms projected, over 5 s measured, the
watchdog fired) and a 200 Hz complex baseband by FIR (PSRAM bandwidth, about 1 s for 15 candidates; recall stayed 7 of 18). The shipped per-symbol DFT was a **Q15
sin/cos table and esp-dsp's dot product** (60 KB x re/im x 2 cores = 120 KB of internal DRAM). 0.6.4 (`18f798c7`, 2026-05-18) replaced it by the Goertzel recursion
**to free that DRAM** (M5StickS3's I2S DMA descriptor could not be allocated beside it), at equal speed (S3 stage 3 about 1.4 s) and +0.06..+0.63 dB. It was
a memory fix; the reasons given for having a per-candidate extraction at all (`524a9ad1`: the spectrogram is a search grid, 80 ms and 3.125 Hz, with a u16 power and
no phase) are the three things `ft8dec` shows it can live without (its waterfall is the same grid; a dB cell keeps the floor; it uses no phase).

**The phase is not needed (measured).** The same S3 flow with the Goertzel cell's phase thrown away (the cell replaced by its magnitude) and/or OSD without `llrb` / `llrc`
(the coherent metrics), 60 slots, hits of 1 338 at 15 / 30 / 60 candidates: phase kept 737 / 993 / 997; magnitude only 736 / 990 / 994; OSD without `llrb`, `llrc`
736 / 991 / 995; both 736 / 990 / 994 (ft8dec's 1 026: 982 -> 981 at 60). Extras unchanged. The coherent metrics are worth at most 3 hits of 1 338; leaving them out of OSD
saves 6 to 7 % of the instructions for 2 hits. (Measured before 5.2, on the `sync8` search.)

**What 5.4 would be**: after 5.2 the dB waterfall exists; an LLR from its cells at the candidate's grid place (the max-log of the eight tone cells, `ft8dec`'s `llr == 0`), into the
existing BP and OSD, with `ft8dec`'s retry at a place interpolated from the Costas tones, would replace the per-candidate DFT (and `fine_sync_at`, and the
block-0 refine) for the candidates it decodes. **Unmeasured**: the recall of an LLR from half-dB cells on the grid place against the Goertzel's at the fine place
(the Goertzel at `ft8dec`'s own places decodes 1 018 of 1 026; the grid place without fine sync gave 1 000 against 999 with it), and the instruction count. It also needs the
cells kept for the whole slot (the 5.2 search drops each band after scoring): 180 KB of u8, against the 425 KB the flow peaks at.

Notes added after the review: the score is clamped at 127 (a tie of very strong signals is broken by carrier and row, not by the peak; none is that strong in the corpora);
the histogram cut and the per-band / merge truncation never bind in `search` at `RAW_FACTOR` 12 on a real slot (150-616 positions above the minimum a slot against 3 600), so
their invariance is tested one stage lower (`the_merged_bands_are_the_best_positions_of_the_whole_range`, with mutations for the threshold, the merge and the
per-band truncation; the tie-break order is an equivalent mutant: the stable sort and the arrival order give the same list). 8 kHz banding on real audio is untested
(the S3 flow runs one band). `websdr_test3`'s `EY8MM YB1BZV OI33` is **not** attested: `band_baseband.rs` already counts it as a chance match, and it is pinned as debt (`S3_DEBT`).

## 13. 5.3 the position choice: measured, not landed (2026-11-26)

`choose_position_at(audio, rate, grid, fine)`: per candidate the grid place or `fine_sync_at`'s, whichever has the higher energy of the 21 Costas symbols' expected
tones (one Goertzel tone a symbol, the stage-3 Hann window of two symbols), only where the two differ by 0.02 s or more in time; between `fine_sync_at` and the block-0
refine in the S3 flow. 60 slots, 8 kHz, fixed-point, S3 flow, after 5.2 (`da758a11`); hits of `jt9`'s 1 338, extras, those of `ft8dec`'s 1 026 (1 pass) found, instructions
per slot (12 slots, host):

| candidates | without | with the choice | cost |
|---:|---|---|---:|
| 15 | 761 / 11 / 754, 295 M | 761 / 12 / 755, 312 M | +17 M |
| 30 | 1 009 / 36 / 1 001, 552 M | 1 011 / 39 / 1 006, 589 M | +36 M |
| 60 | 1 011 / 37 / 1 001, 1 012 M | 1 013 / 40 / 1 006, 1 110 M | +98 M |

Comparing at every candidate (no 0.02 s limit) was no better, on the 38 `20m_busy` slots alone at 60 candidates: 717 hits / 40 extras, against 717 / 38 with the limit.

**Gate (plan): `ft8dec`'s 1 026 at 60 >= the 5.2 state + 5 (met, exactly: 1 001 -> 1 006); no regression at 15 / 30 (met); instructions at most +60 M at 60 candidates
(failed: +98 M); extras explained (not checked: +3).** Not landed. The plan's experiment gave 999 -> 1 006 (+7) on the `sync8` search's candidates; after 5.2 the
`ft8_lib` search already places the candidates on the grid the choice would have recovered, and what is left is 5 signals of 1 026 for 5 to 10 % more instructions
and 3 more unverified extras. For comparison the candidates themselves buy 246 hits for 257 M between 15 and 30, and none between 30 and 60. The code is not in the tree
(a patch is kept outside it); an energy comparison of this kind could be revisited if a cheaper place test appears (the kernel is 42 transforms of 2 560 samples a differing
candidate: about 0.4 M).

## 14. 5.4 experiment: the LLRs from the half-dB cells, no per-candidate DFT (2026-11-26, host, not built)

**What was run** (temporary code, not in the tree; a patch is kept outside it): the S3 flow's stage 3 (BP, OSD, the same `process_candidates_into_with_cs_scratch_tuned_with_fill`) with the
79 x 8 symbol spectra read from the half-dB cells of 5.2 at the candidate's **grid place** (`rows j0 + 2 s`, bins `k + 2 t`, amplitude `10^(h/40)`, no phase), no `fine_sync_at`, the candidates
ranked by `sync_quality_block0` of those cells. **Control**: the same flow with the Goertzel at the same grid place and no fine sync, to separate what the cells change from what leaving
out the fine sync changes. 60 slots, 8 kHz, fixed-point, after 5.2: hits of `jt9`'s 1 338 / extras / `ft8dec`'s 1 026 found.

| candidates | current (fine sync, Goertzel) | control: Goertzel at the grid place | cells at the grid place |
|---:|---|---|---|
| 15 | 761 / 11 / 754 | 754 / 12 / 756 | 753 / 12 / 755 |
| 30 | 1 009 / 36 / 1 001 | 989 / 39 / 1 001 | 987 / 41 / 1 000 |
| 60 | 1 011 / 37 / 1 001 | 991 / 41 / 1 001 | 989 / 42 / 1 000 |

**Reading.** The cells cost nothing against the Goertzel at the same place (753 against 754, 987 against 989, 989 against 991 `jt9` hits; `ft8dec`'s found 1 000 against 1 001): the DFT's per-candidate
extraction is not needed for the recall, as the phase experiment (section 12) already suggested. What the grid place costs is the fine sync: **-20 of `jt9`'s 1 338** at 30 and 60 candidates (the weak
signals `jt9` finds whose place the grid has off), none of `ft8dec`'s (1 001 -> 1 001 / 1 000), which is the yardstick this thread has used and is a different thing from `jt9`'s list. 5.3's place choice
(section 13) recovered 2 of those 20.

**Cost** (instructions per slot, 12 slots, host): **corrected in section 15**. The figures first written here (385 / 531 / 930 M as run, "+5 % / -18 % / -18 %" shared) were wrong: the harness ran the
gated search twice (the flow's own, then the experiment's) and built the cells a third time. Without the duplicate search the experiment is 248 / 393 / 790 M, and with the cells shared with the search
(one demux less, 76.7 M measured) **171 / 316 / 713 M against the current 298 / 558 / 1 022 M: -43 % / -43 % / -30 %** (an estimate: the sharing is not built).

**Memory (not measured here)**: the whole slot's cells are 173 KB as u8 (the cell is at most 193) or 345 KB as u16 on the host, against the banded flow's one band; the board's heap budget is the open question.

**Not done**: the search at the same time as the cells (no second demux), `ft8dec`'s retry at a place interpolated from the Costas tones (what would recover the 20 `jt9`-only signals without the fine sync's
cost), the cost of a fine sync on the cells alone, tier C, a board. Whether 5.4 is worth building turns on the `jt9`-only 20 (1.5 % of the reference list, the weaker signals) against -43 % instructions at 30
candidates (section 15) and the memory.

## 15. Where the instructions go (S3 flow, 8 kHz, fixed-point, host; 2026-11-26)

**Method.** Instructions retired (`/usr/bin/time -l`, `--reps 6` minus `--reps 1`, 12 `20m_busy` slots), by **difference between runs of the same flow cut short or with a stage left out**
(temporary switches, not in the tree; a patch is kept outside it): stop after the search, after `fine_sync_at`, after the block-0 refine; OSD off (`DecodeDepth::osd = false`); and extra
72-symbol Goertzel fills (`SymMask::NotBlock0`) per candidate, 1 against 3, for the cost of one fill. Wall-time shares from `MFSK_TRACE_STAGE_FT8` were taken too and **disagree** with the instruction shares
(symbol spectra 50 % of the time at 15 and 30 candidates, the search 37 % / 19 %): the Goertzel recursion is latency-bound (low IPC), so time overstates it. Instructions are used here; neither is ESP32 cycles.

| M instr / slot | 15 candidates | 30 | 60 |
|---|---:|---:|---:|
| search (`coarse_sync_gated`, includes the demux, 76.7 M) | 137 | 137 | 137 |
| `fine_sync_at` | 11 | 22 | 43 |
| block-0 refine (7 symbols, 0.78 M a candidate) | 12 | 23 | 46 |
| stage 3 in all (full - refine) | 138 | 376 | 796 |
| of which OSD (full - OSD off) | 13 | 102 | 291 |
| of which the symbol fill (one 72-symbol fill 7.8 M a candidate, 2 per candidate fill the whole spectrum) | about 117 | about 234 | about 410-470 |
| of which BP, LLRs, the rest (by difference) | about 8 | about 40 | about 40-90 |
| **total** | **298** | **558** | **1 022** |

OSD ran about 1 / 11 / 29 times a slot (trace calls) at about 10 M a run. Per candidate, the symbol fill is the largest part at 15 and 30 candidates (39 % / 42 %), OSD at 60 (28 %, fill 40-46 %); the search is
46 % of the total at 15 candidates and 13 % at 60.
BP, LLRs and the rest together are the smallest part (the difference, with the error of the number of fills, which is taken from the trace's call counts); BP's own instructions were not isolated (its wall-time share is 4-15 %).

**What this says about 5.4.** The per-candidate DFT is the cost 5.4 removes (7.8 M a candidate plus 0.8 M for block 0 and 0.7 M of fine sync: about 9 M of the 12-17 M a candidate). The cells experiment (section 14), with the duplicate search taken out
of its harness: 248 / 393 / 790 M, and 171 / 316 / 713 M with the cells shared (one demux less): **-43 % / -43 % / -30 %** against the current flow, at ft8dec-yardstick recall unchanged and `jt9`'s hits -20 (section 14). What stays is
the search (137 M, 46-80 % of the 5.4 flow at 15 candidates) and OSD (10 M a run, which does not depend on where the LLRs come from). OSD at about 10 M a run, run for candidates that mostly do not decode, is the next
cost after the fill: its success rate per run was **not measured** here.

**Not done**: instruction counts of the stages on the ESP32 (the mix differs: the FPU Goertzel against integer demux), BP separately by difference (the BP-off switch changes which candidates reach OSD), OSD successes
per run, the stage costs at 12 kHz.

## 16. OSD: how often it is tried and how often it decodes (S3 flow, 8 kHz, fixed-point; 60 slots; 2026-11-26)

**Method.** Counters (temporary, not in the tree; a patch is kept outside it) in `process_one_candidate_inner` and `osd_strategy::try_fallback`: candidates that reach the staircase, those BP decodes
(`bp_ok`), those that reach OSD (`q > 6`, BP failed: **one try is one `try_fallback`**, up to 8-9 `osd174_91` runs inside it: four LLR variants x two BP snapshots, and `llre`), those OSD returns a codeword for
(`some`), and the decodes finally emitted by pass id (OSD 13-22, BP 0-4) against `jt9`'s list (hit) or not (extra). All 60 slots, counts summed.

| candidates | reach the staircase | BP decodes | OSD tries | OSD returns | rate | `osd174_91` runs a try | emitted by OSD (hits / extras) | emitted by BP (hits / extras) |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 15 | 885 | 761 | 124 | 22 | 17.7 % | 8.8 | 18 / 4 | 743 / 7 |
| 30 | 1 678 | 1 005 | 673 | 66 | 9.8 % | 9.3 | 52 / 12 | 957 / 24 |
| 60 | 2 674 | 1 006 | 1 668 | 69 | 4.1 % | 9.7 | 54 / 13 | 957 / 24 |

(The OSD returns that are not emitted, 2 and 2 at 30 and 60 candidates, were refused by the plausibility policy or were duplicates.)

**Reading.**
* **Beyond 30 candidates the list is nearly empty of decodable signals**: 31 to 60 add 996 candidates, 1 BP decode and 3 OSD returns (2 hits, 1 extra) for 995 more OSD tries, that is +464 M instructions a slot (section 15)
  for +2 of `jt9`'s hits. 15 to 30 add 549 tries and 44 returns (34 hits, 8 extras) for +260 M.
* At 30 candidates 673 OSD tries a 60 slots (11 a slot, about 9 M each by section 15's totals) return a codeword 9.8 % of the time, **and 90 % of the tries fail** (607 of 673; the instructions are counted per run, not per outcome, so "90 % of the OSD instructions" assumes a failing try costs what a returning one does): OSD's 102 M a slot over 60 slots
  is about 93 M per returned codeword and 118 M per `jt9` hit.
* **By `q`** (sync quality of the candidate as the staircase reads it, 30 candidates, tries / returns): `q` 7: 75 / 0; 8: 81 / 0; 9: 65 / 2; 10: 57 / 1; 11: 52 / 4; 12: 58 / 3; 13: 56 / 6; 14: 77 / 13; 15: 55 / 13; 16: 33 / 7;
  17: 25 / 10; 18..21: 39 / 7. A gate at `q >= 9` would have cost no decode on this corpus and saved 156 of 673 tries (23 % of the OSD instructions); at `q >= 11`, 278 tries (41 %) for 3 returns.
  **That gate is WSJT-X's** (`nsync > 6`: `ft8b.f90`; the comment in `osd_strategy.rs` records a real signal, `K1BZM DK8NE -10`, at `q` 11 that a stricter gate had refused); raising it would be a divergence to be
  justified on more than 60 slots with no noise-only run, and the extras it would remove are not separated from the hits in these counts.

**Not done**: successes by `q` split into hits and extras; the cost of a successful try against a failing one (the runs per try are counted, the instructions are not); a stop after fewer than 8 runs for a candidate whose first runs
come nowhere near a codeword (`osd174_91`'s `hard_errors` of the best candidate would say); the cost of the other flows' OSD; noise-only slots.

## 17. OSD early stop: where the 8-10 runs of a try succeed (30 candidates; 60 slots; 2026-11-26)

**Method.** Temporary logging (not in the tree; a patch is kept outside it) of every `osd174_91` run inside `try_fallback`: the winner's hard errors `he` (taken before the CRC check, so a failing run has one too), whether
the CRC passed, and the run's index in the fixed order **variant a, b, c, d, e (`llra`..`llrd`, `llre`) x BP snapshot 1, 2 = runs 0..9**. 673 tries, 6 286 runs, 66 returns (52 `jt9` hits, 12 extras, 2 not emitted); a return is a CRC pass with
`he <= 36` (`DecodeStrictness::Normal`).

**Where the successes are** (run index: hits / extras): 0: 27 / 6; 1: 5 / 2 (+1 not emitted); 2: 0 / 1; 3: 1 / 0 (+1); **6: 18 / 3**; 8: 1 / 0; **runs 4, 5, 7, 9: none**. Run 6 is variant `d`, snapshot 1. 41 of the 66 returns are in the first two runs (variant `a`).
Runs reaching each index: 673 at 0, about 630 at 2 to 6, 608 at 7 to 9.

**Skipping runs that never succeeded** (on this corpus): runs 4 and 5 (variant `c`): 20 % of the runs, no return lost; 4, 5, 7: 30 %; 4, 5, 7, 9: **39 %** (2 473 of 6 286 runs), no return lost. Also skipping 2 and 3 (variant `b`): 59 %, loses 3 returns
(1 hit, 1 extra, 1 not emitted). If a run costs the same (not measured: a run is the BP snapshot, the OSD, and a share of the LLR), 39 % of the OSD's 102 M a slot at 30 candidates (section 15) is about **40 M of 558 M (7 %)**.
This is a corpus fact, not a reason: variants `b` and `c` are `ft8b.f90`'s `ipass` 2 and 3, and WSJT-X runs the whole staircase. Dropping them is a divergence to be marked and measured on more than these slots (tier C, the
noise-only slots, the other corpora) before it is considered; AP passes and other strictness levels would use them differently.

**A stop on `he`** (stop a try after `k` runs if the best `he` so far exceeds a threshold): the failing tries' `he` overlaps the returns' (the best `he` of the first four runs: returns 17-33, median 23; failing tries mostly 22-30), so
the rule saves little for what it loses: after 2 runs, threshold 30: 16 % of the runs, 1 hit and 1 extra lost; after 4 runs, threshold 30: 6 % of the runs, 1 extra lost; after 4 runs, threshold 28: 11 %, 1 hit and 2 extras lost; threshold 26 after 2 runs:
38 %, 5 hits and 2 extras lost. `he` is not a good early signal: the CRC is what separates a codeword from a nearest-neighbour.

**Not done**: instructions per run (the saving above is by count of runs); the same on other corpora and noise; the order of the variants (a, then d) as a rule; a stop on the weighted distance `dmin / sum|llr|` (logged, not analysed).

## 18. 5.4 on the cells, with fine sync replaced by `ft8dec`'s interpolation (host experiment; 2026-11-26)

**Variants** (temporary code, not in the tree; a patch is kept outside it; the cells are built once and shared by the search and the symbols, so the instructions below are **measured**, not estimated):
V0 the grid place; **V1** `ft8dec`'s retry: for a candidate that failed at the grid place (BP and OSD), the shift is estimated by the parabola on the 3 x 3 energy of the 21 Costas tones (rows +-1, cells +-1),
the four cells are mixed linearly in power and BP and OSD run again; V2 one attempt at the interpolated place; V3 the retry of V1 with BP only. No `fine_sync_at`, no Goertzel. 60 slots, 8 kHz, fixed-point.

| | 15 candidates: hits / extras / `ft8dec` found, M instr | 30 candidates | 60 candidates (M instr) |
|---|---|---|---:|
| current flow (fine sync, Goertzel) | 761 / 11 / 754, **298** | 1 009 / 36 / 1 001, **558** | **1 022** |
| V0 grid | 753 / 12 / 755, 171 | 987 / 41 / 1 000, 316 | 714 |
| **V1 retry, BP + OSD** | 754 / 15 / **758**, 187 (-37 %) | 1 002 / 46 / **1 016**, 451 (-19 %) | 1 130 (+11 %) |
| V2 interpolate, one attempt | 750 / 15 / 755, 177 (-40 %) | 990 / 38 / 1 008, 310 (-44 %) | 597 (-42 %) |
| V3 retry, BP only | 753 / 12 / 755, 176 | 991 / 41 / 1 004, 355 (-36 %) | 833 (-18 %) |

**Gates of the plan (per variant, 15 / 30 candidates)**: `ft8dec` found >= current - 2 (754 / 1 001): **V1, V2, V3 pass** (V1 clearly: +4 / +15); `jt9` hits >= current - 5 (761 / 1 009, that is 756 / 1 004): **none passes**
(V1 754 / 1 002: short by 2 and 2; V2 750 / 990; V3 753 / 991); extras <= current + 3 (11 / 36): **V1 15 / 46 fails, V2 15 / 38 fails at 15 and passes at 30, V3 12 / 41 passes at 15 and fails at 30**; instructions
<= the current 298 / 558: **all pass** at 15 and 30 (V1 187 / 451); at 60 candidates only V1 is above the current flow.

**Extras of V1 at 30 candidates**: 46, of which 34 are in `ft8dec`'s 3-pass list; 11 are not among the current flow's 36, 7 of those in `ft8dec`'s list; the 4 not listed by it are `CQ OR18OSB` (test_01), `CQ RW6PA LN23` (test_03), `<...> ZY50Y 73`
(test_38), `KC8MUE V51MA RRR` (websdr_test9): callsigns that read as real, **not checked** by a wrong-tone count or a second recording (no noise-only slot run). V1 loses 19 of the current flow's `jt9` hits and gains 12 others (net -7).

**Reading.** The cells path removes 37-44 % of the instructions at 15 and 30 candidates for the variants that keep one decode attempt (V2) or one cheap retry (V1 at 15), and **V1 finds more of `ft8dec`'s decodes than the
current flow does** (1 016 against 1 001 at 30 candidates) while finding 7 fewer of `jt9`'s: `jt9`'s list and `ft8dec`'s differ in the weak signals they hear, and the cells with `ft8dec`'s retry sit on `ft8dec`'s side. Against the strict
`jt9`-based gate V1 misses by two hits and by extras, narrowly; V1's cost at 30 candidates (451 M) is the retry's second BP and OSD for the candidates that fail (+135 M over V0); V3 shows BP alone recovers little of it.
The OSD stays the dearest part (section 16: 90 % of its tries fail), so a retry that spends it only where it pays is the next thing to look at.

**Not done**: the wrong-tone scan and a second look at V1's 4 unlisted extras; a retry limited by `q` or by the first attempt's best `hard_errors` (the OSD's runs are logged in section 17: the first attempt's `he` is not a good signal);
V4 (the choice between the grid and the interpolated place by Costas energy); the ESP32 instruction counts (the cells path replaces the f32 Goertzel by table lookups, integer cell reads and a `sqrt` of the mixed power: the host ratio need not hold);
the memory of the whole-slot cells (173 KB as u8); noise-only slots (tier C). Nothing here is built: it is the evidence for a decision.

## 19. 5.4: restricting V1's retry, and a fairer yardstick (host experiment; 2026-11-26)

**What was tried** (temporary code, not in the tree; a patch is kept outside it): V1's retry (section 18) only for candidates whose estimated shift `max(|ta|, |fb|)` is at least a threshold; only for those with
hard-sync count `hs` of at least 10 / 12 / 14 / 16 of 21; or with OSD only above an `hs`. 60 slots, 8 kHz, fixed-point; hits of `jt9`'s 1 338 / extras / `ft8dec` 1-pass found of 1 026, then instructions (12 slots).

| 30 candidates | hits / extras / `ft8dec` found | M instr | 15 candidates | hits / extras / `ft8dec` found | M instr |
|---|---|---:|---|---|---:|
| current flow | 1 009 / 36 / 1 001 | 558 | current flow | 761 / 11 / 754 | 298 |
| V1 full | 1 002 / 46 / 1 016 | 451 | V1 full | 754 / 15 / 758 | 187 |
| V1, shift >= 0.15 | 1 002 / 46 / 1 016 | (as full) | | | |
| **V1, shift >= 0.25** | 1 001 / 45 / 1 015 | **400** | V1, shift >= 0.25 and `hs` >= 10 | 754 / 12 / 756 | 180 |
| V1, shift >= 0.25 and `hs` >= 10 | 1 000 / 43 / 1 013 | 387 | | | |
| V1, `hs` >= 10 / 12 / 14 / 16 | 1 001 / 1 000 / 995 / 989; extras 44 / 44 / 43 / 42 | 432 / 406 / - / - | | | |
| V1, OSD in the retry only `hs` >= 12 | 1 001 / 44 / 1 014 | 419 | | | |
| V1, shift >= 0.35 | 996 / 45 / 1 011 | - | | | |

A shift under 0.15 never gains anything (the same list as the full retry); under 0.25 almost nothing (1 hit of 1 338, 1 of `ft8dec`'s); the retry's cost falls with the threshold because the interpolation, the second LLRs and the second BP and OSD
are skipped for the candidates it would not change.

**A fairer yardstick.** `jt9` and `ft8dec` hear different weak signals, so a gate on `jt9`'s list alone punishes a flow that sits on `ft8dec`'s side. The union of `jt9`'s list and `ft8dec`'s 3-pass list is **1 409** decodes on these slots
(1 338 of them `jt9`'s). Found of the union, and decodes in neither list (30 candidates / 15 candidates):

| | 30: found, in neither list | 15: found, in neither list |
|---|---|---|
| current flow | 1 036, 9 | 769, 3 |
| V0 (grid, no retry) | 1 018, 10 | 762, 3 |
| V1 full | **1 036**, 12 | 765, 4 |
| V1, shift >= 0.25 | 1 035, 11 | 765, 3 |
| V1, shift >= 0.25 and `hs` >= 10 | 1 032, 11 | 763, 3 |
| V2 (one attempt) | 1 020, 8 | 761, 4 |

**Reading.** On the union, V1 with the shift threshold of 0.25 finds **as many as the current flow at 30 candidates (1 035 against 1 036) for 28 % fewer instructions (400 against 558 M)**, and at 15 candidates 4 fewer
(765 against 769, 0.5 %) for 40 % fewer (180 against 298 M). Decodes in neither list: +2 at 30 candidates (11 against 9), +0 at 15; those are not verified (no wrong-tone scan, no noise-only run); they are the price to look at before this is believed.
On `jt9`'s list alone the gate of the plan (current - 5) is still missed (1 001 against 1 009 at 30; 754 against 761 at 15). **The plan's gate was set on `jt9` only; whether the union is the right yardstick is the owner's call**,
not this note's. The `hs` and OSD-above-`hs` restrictions save a little more instructions (387-419 M) for 1-4 decodes; the shift threshold is the one with the best exchange.

**Not done**: the wrong-tone scan of the 11 and 3 decodes in neither list (the 4 of section 18 among them); the shift threshold on other corpora; the ESP32 counts; the memory; noise-only slots (tier C).

## 20. The decodes in neither list: wrong-tone scan (V1, shift >= 0.25; 15 and 30 candidates; 2026-11-26)

**What.** The decodes of V1 with the shift threshold of 0.25 (section 19) that are in neither `jt9`'s list nor `ft8dec`'s 3-pass list: **11** in all (the 3 of the 15-candidate run are among the 11 of the 30-candidate run; section 19 counted them
per run). For each, the temporary scan of the earlier sections: the message re-packed to its 79 tones, the symbol error of the strongest-of-eight tone decision (Hann window of two symbols, 8 kHz) against those true tones, at the decoded frequency
+-0.5 Hz and every DT from -2.2 to +2.6 s in 40 ms steps, the best taken. **Control**: the same scan for 38 trials with a *wrong* standard message at the same slots and frequencies: best 45, median 53, maximum 60 wrong of 79
(the frequencies are occupied by real signals, which share some tones by chance, so the control is, if anything, lenient). A cross-slot count of how many other slots' reference decodes (`jt9` or `ft8dec`) carry each callsign.

| slot | decode | wrong of 79 | callsign in other slots |
|---|---|---:|---|
| test_37 | R1CBP UV5IW -16 | **4** | 4 / 4 |
| test_27 | RV6ARS E75C -11 | **8** | 12 / 27 |
| test_01 | CQ OZ5VO JO45 | 16 | 5 |
| test_03 | CQ RW6PA LN23 | 16 | 5 |
| test_14 | UR7HN UA3NFG LO28 | 19 | 17 / 8 |
| websdr_test3 | EY8MM YB1BZV OI33 | 23 | 1 / 0 |
| websdr_test9 | KC8MUE V51MA RRR | 27 | 1 / 1 |
| test_34 | IZ7NLM `<...>` R-14 | 38 (not reliable) | 12 |
| test_31 | `<...>` 4U1A -20 | 40 (not reliable) | 20 |
| test_23 | `<...>` IV3KVC JN65 | 48 (not reliable) | 4 |
| test_38 | `<...>` ZY50Y 73 | not packable | 8 |

**Reading.** Seven decodes read **4 to 27 wrong tones of 79, against 45 or more for a wrong message at the same places**, and six of them carry callsigns heard in other slots of the corpus: these are real signals that neither reference decoder listed.
`EY8MM YB1BZV OI33` (23) is the one the repository already counts as debt (`band_baseband.rs`, `S3_DEBT`): the tones say it is a signal, its partner appears nowhere else, so the debt stays a debt. The four messages with a hashed call `<...>` cannot be
checked this way (the scan packs `<...>` as a hash, not the true callsign: the field's symbols read wrong by construction), so their 38-48 mean nothing; their other callsign is heard in 4 to 20 other slots, which is support, not proof.
**None of the 11 reads like noise**; the decodes in neither list, the price named in section 19 (+2 at 30 candidates), are mostly real signals the two references missed. What is **not** shown: that no noise-only slot yields such a decode
(tier C), and the four `<...>` ones by tone.

## 21. 5.4 as built: `decode_block_cells` (2026-11-26)

**What was added** (all `fixed-point`, `Hz8000`; `ft8::decode_block`): `HalfDbCells` (the whole slot's half-dB cells, a byte each: 172 KB at 200-3000 Hz, 184 rows), `coarse_sync_gated_cells` (the 5.2 search with its cells kept: the same candidates as
`coarse_sync_gated`, a test pins it), `decode_block_cells` (the flow of section 18's V1 with the shift threshold of section 19: first try at the grid place, a second at the place the Costas tones point to when the estimate is a quarter cell or more off),
and in the `icap-app` host harness `run_slot_s3_cells` (`icap-profile --s3 --cells`). Nothing existing changed: the u16 spectrogram is bit for bit what it was (the checksum test), `Hz12000` and f32 are untouched, and the S3 flow stays.

**Deviations from the plan.** (1) **The demux writes bytes**: `spectrogram_cells` is generic over the cell it writes (`CellOut`: `u16` as before, `u8` for the half-dB cells), so the half-dB band is made as bytes and, at one band, *is* the store.
The plan kept a `u16` band and packed it; that gave a host peak heap of 509 KB against the S3 flow's 422 KB (a gate failure), the byte demux 258 KB. (2) `coarse_sync_gated_cells` returns `Option` (`None` at another rate) and takes `bands`
like `coarse_sync_gated`; `decode_block_cells` runs one band (more bands read the audio again and cost more: 4 bands 362 M for the search against 152 M). (3) The flow decodes **one candidate at a time**
(`process_candidates_into_with_cs_scratch_tuned_with_fill` on a list of one) so the retry knows which candidate failed, without the experiment's score-as-index trick. (4) The fill and the estimator are not public.

**Ledger** (60 slots, 8 kHz, fixed-point; hits of `jt9`'s 1 338 / extras / `ft8dec` found of 1 026 / found of the union of 1 409 and decodes in neither list; host instructions a slot over 12 busy slots; peak heap the largest of a slot):

| | current S3 flow | step 2: cells, no retry | **cells flow (landed)** |
|---|---|---|---|
| 15 candidates | 761 / 11 / 754 / 769 / 3, 295 M, 422 KB | 753 / 12 / 755 / 762 / 3, 157 M, 258 KB | 754 / 14 / 758 / **765** / 3, **165 M**, 258 KB |
| 30 candidates | 1 009 / 36 / 1 001 / 1 036 / 9, 549 M | 987 / 41 / 1 000 / 1 018 / 10, 300 M | 1 001 / 45 / 1 015 / **1 035** / 11, **382 M** |
| 60 candidates | 1 011 / 37 / 1 001 / 1 039 / 9, 1 003 M | 989 / 42 / 1 000 / 1 021 / 10, 691 M | 1 003 / 46 / 1 015 / 1 038 / 11, 971 M |

Gates of the plan (step 3): instructions at 15 <= 190 M (165) and at 30 <= 410 M (382): **met**; found of the union >= current - 4 at 15 (765 against 769: met, exactly) and - 2 at 30 (1 035 against 1 036): **met**; decodes in neither list <= current + 3
(3 against 3, 11 against 9): **met**; peak heap <= the S3 flow's: **met** (258 against 422 KB). The 22 recordings against `jt9`'s 402: 226 / 286 at 15 / 30 candidates (the S3 flow 230 / 289), extras 3 and 5, all attested or pinned debt
(`Z33Z` and `KC8MUE` attested with their evidence; `EY8MM YB1BZV OI33` stays by-identity debt). The decodes in neither list were checked in section 20.

**Tests** (tier A, mutation-checked; after the `reviewer`'s pass): the store equals the demux's cells for 1, 2, 4 and 7 bands (band column, `first_bin`, and a clamp under 100 caught; a clamp at 180 would not: both sides are the same byte demux, only the range check guards it);
the cells search returns the gated candidates and `None` at 12 kHz (rate gate caught); a clean signal decodes (a fill one **row** off breaks it; one **bin** off does not); the fill reads the transmitted tone at 76 or more of 79 symbols of a clean on-grid signal (a one-row or one-bin shift of the
fill caught); the estimator lands within 0.25 cell of a signal 0.4 cell off in time and frequency (shifts inside `place`, either direction flipped, caught); the retry decodes more than the grid place alone on 80 seeded weak off-grid slots (54 against 46; a flipped `dc` and the threshold caught,
a flipped `dr` only by the estimator test); the retry's mix is the square root of the mixed **powers**, and a place off the slot has no power (no-sqrt and amplitude-mix, and an outside cell read as amplitude 1, caught); the reported frequency and DT of a decode move towards the estimate (the frequency
sign caught at the decode level, both signs by the unit test of `Place::moved`); the tables; the parabola; a retry needs a quarter cell. **The planned noise-only guard was deleted**: 40 seeded noise-only slots decoded nothing under every loosening tried (`q_thresh` 0, a retry threshold of 0), so it
could not fail (CLAUDE.md 4); the precision guard of this flow is the floor test's extras ceiling with the by-identity debt in `icap_profile.rs`, and tier C is still to be run. The clamp in `parab` is redundant (the vertex of a parabola with the centre above both neighbours is within 0.5).

**Findings of the `reviewer` and what was done.** (1) The amplitude differs from `ft8dec`'s: `ft8dec` (`llr = 2`) feeds its max-log LLRs the fourth root of the power (`10^(h / 80)`), this flow gives the staircase the amplitude (`10^(h / 40)`); and `ft8dec`'s `tcal` calibration of the reported DT is not
ported: both now said in the module doc, neither measured against the alternative. (2) The memory text of two public docs described the plan's `u16` band, not the byte demux: corrected (258 KB measured). (3) Test docs that claimed mutations they do not catch were corrected and tests added where a gap was found (above). (4) Experiment figures in code
comments replaced by the landed ones. The "current S3 flow" baseline of the ledger is **295 / 549 / 1 003 M**, measured on the same binary as the cells flow; sections 15-19 have 298 / 558 / 1 022 M from earlier builds (the differences are the builds', not a change of the flow). (5) A cell outside the slot or the
store now has no power (`ft8dec`'s `cellp`: 0), it read as amplitude 1; re-measured: the ledger is unchanged to the decode. (6) Dead work removed (the block-0 `q` computed and dropped); **the plan's block-0 re-rank was not built** (candidates are taken in the search's hard-sync order; the S3 flow goes through `refine_candidates_into_at`),
which only changes which duplicate is reported. (7) The two attested extras (`Z33Z`, `KC8MUE`) are in the list `score()` uses for **every** flow, not only the cells flow: the other flows' guards are loosened by those two messages. (8) The floors (226 / 286, extras <= 5) are the Mac measurement with no margin,
as the S3 floors are (`Tables` use `exp` of the `no_std` build's libm, deterministic across platforms); none of the 5.x floors has run in Linux CI yet.

**`snr_db` is not validated**: on the 22 recordings at 30 candidates it is 11.1 dB below `jt9`'s on average (sd 9.2) for the cells flow and 9.8 dB (sd 8.9) for the S3 flow: neither flow's `compute_snr_db` is calibrated to WSJT-X's, and the cells flow's is about 1.3 dB lower than the S3 flow's.

**Not done**: noise-only slots (tier C); an ESP32 build or instruction count (the cells path replaces the f32 Goertzel by table lookups, integer cell reads, and a `sqrt` of the mixed power on the retry); a board-side producer of the half-dB cells (`stage1_inc`) and the CoreS3's heap budget for 172 KB;
the 60-slot numbers outside the repo (the 38 `20m_busy` slots and `ft8dec` are not vendored, as in section 9); `reviewer` and `scripts/verify.sh` (below, if done).

## 22. A second pass after subtraction on the cells flow: the upside and its cost (host experiment, 2026-11-26)

**What was run** (temporary example in a scratch worktree, not in the tree; kept outside it as `tmp_sic_example.rs`): pass 1 = `decode_block_cells`; each decode's waveform subtracted from a copy of the 8 kHz audio by the repository's exact subtraction
(`subtract_tones_lpf`, `subtractft8.f90`'s algorithm: GFSK reference, low-pass-filtered complex amplitude, FFT-based; an 8 kHz `SubtractCfg`, `lpf_half` 1333); pass 2 (and 3) = `decode_block_cells` on the residual, decodes not already found added. Three ways to subtract:
**a** `subtract_tones_lpf_refine_dt` (probes DT) at the reported place, **b** the frequency refined first (`refine_freq`, +-2 Hz in 0.25 Hz steps) then `refine_dt`, **c** plain `subtract_tones_lpf` at the reported place (the place of the cells flow: 3.125 Hz, 80 ms grid, or the retry's estimate).
60 slots, 8 kHz. **This harness is `mfsk-core` with the `std` build** (the host 16-bit planner), not the `icap-app` build: its pass 1 is **not** the ledger's (union found 767 / 1 050 at 15 / 30 candidates against section 21's 765 / 1 035, and 200 / 584 M instructions against 165 / 382 M; the difference is not explained here),
so only the ratios inside the harness are used. jt9 hits / extras / union found of 1 409 / in neither list; host instructions a slot over 12 busy slots:

| flow (candidates per pass) | jt9 hits | extras | union found | in neither | M instr |
|---|---:|---:|---:|---:|---:|
| 15, pass 1 only | 756 | 15 | 767 | 4 | 200 |
| 15, subtract **c**, 2 passes | **936** | 25 | **952** | 9 | 784 |
| 15, subtract **c**, 3 passes | (not run) | | | | 1 244 |
| 15, subtract a, 2 / 3 passes | 844 / 850 | 20 / 23 | 859 / 868 | 5 / 5 | 1 223 (2 passes) |
| 15, subtract b, 2 passes | 850 | 22 | 865 | 7 | 2 288 |
| 30, pass 1 only | 1 016 | 48 | 1 050 | 14 | 585 |
| 30, pass 1 only, 60 candidates | 1 018 | 50 | 1 053 | 15 | 1 693 |
| 30, subtract **c**, 2 passes | **1 090** | 60 | **1 129** | 21 | 1 812 |
| 30, subtract c, 3 passes | (not run) | | | | 2 813 |
| 30, subtract a, 2 / 3 passes | 1 051 / 1 053 | 59 / 59 | 1 092 / 1 094 | 18 / 18 | |
| 30, subtract b, 2 passes | 1 055 | 63 | 1 096 | 22 | 3 920 |

For reference, `ft8dec` on the same slots: 990 `jt9` hits in one pass, 1 189 in two, 1 204 in three, at 335 / 850 / 1 278 M instructions.

**Where the instructions go (subtract c, host build)**: at 30 candidates pass 1 584 M, the subtraction of its decodes **321 M** (about 18 M a decoded signal: the exact subtraction is two full-length FFTs and a synthesis per signal), the second pass's search and decode **906 M** (more than pass 1: the residual is full of
borderline candidates that reach OSD); at 15 candidates 200 / 226 / 358 M.

**Reading.**
* **The simple subtraction (c) is the best of the three**; refining the DT (a) or the frequency too (b) made the residual *worse* (why is not established: the refinements were written for the 12 kHz decode's places, and nothing here separates a wrong probe from a wrong 8 kHz configuration), and b costs 4-6 times more.
* **At 30 candidates one extra pass buys +74 of `jt9`'s hits (+79 of the union) for 3.1 times the instructions** (584 -> 1 812 M), and 7 more decodes in neither list (14 -> 21, unchecked). `ft8dec`'s second pass buys +199 for 2.5 times. Against the plan's bar (+100 of the union for at most +300 M instructions): **not met**.
* **At 15 candidates the second pass mostly doubles the candidates**: 15 + one pass of 15 gives 936 hits for 784 M, while 30 candidates in one pass give 1 016 for 585 M. At equal cost, more candidates beat a second pass; the second pass only pays once the first is already deep.
* What would have to be true for a board's SIC to pay: a subtraction an order of magnitude cheaper than the exact one (about 2 M a signal: a per-symbol complex amplitude from the Goertzel of the known tones, a synthesised reference removed in a pass over the signal's span, no FFT) *and* a second pass that decodes only the candidates near what was subtracted
(the remaining 906 M is not subtraction's). Neither is built or measured here; with both, a second pass would cost about the cells' demux (76 M) plus a few candidates' decodes, an **estimate**, not a measurement. The subtraction is also where `ft8dec` is more careful (its place comes from the interpolated parabola and its time calibration), which this
experiment's places (the grid's) are not.

**Not done**: a cheap subtraction; a restricted second pass; three-pass numbers (the third pass added 6-9 hits in mode a); the decodes in neither list of the second pass checked (21 at 30 candidates, +7 over pass 1); the board's heap for a residual copy (240 KB at 8 kHz) or a read-while-subtracting design; why this harness's pass 1 differs from the `icap-app` build's; tier C.

## 23. `ft8dec`'s subtraction passes on the cells flow, as built (2026-11-26)

**What this is.** Section 22 measured the repository's WSJT-X-style subtraction by mistake; this section is the port that was asked for: `ft8dec.c` (icap-sdk 1ca9f04, MIT) `sig_corr` (243-290), `sig_sub` (291-330),
`ph_inc` / `ph_at`, `build_wf` (333-380) in `ft8dec_sic.rs` (`ResidualSource`: a `PcmSource` serving the audio minus the signals, which the byte demux reads unchanged, a ring of 8 192 `f32`, the lag of 1 920 samples, no FFT), `parab` / `tcal` and
the place refinement (874-882) in `cells_flow.rs` (`View::refine`), the restricted search (`round_` 826-870: `binok`) in `coarse_lib.rs` (`search_near`, `band_best`'s `allowed`), and the pass loop (984-1010) in `decode_block_cells_sic`.
`decode_block_cells` is `decode_round` over the first pass; nothing existing changed (the u16 demux, the other flows).
**Deviations from `ft8dec`:** the staircase after the spectra is the library's; the search is 5.2's; one band; the slot's gain is read from the audio, not the residual; a pass after the first decodes at most `max_cand / 2` candidates (8 at least), where `ft8dec` decodes `max_cand` (300) every pass (section 23a).

**Ledger** (60 slots, 8 kHz, fixed-point; jt9 hits of 1 338 / extras / found of the union of 1 409 / in neither list; host instructions a slot over 12 busy slots; heap the largest of a slot):

| | 15 candidates | 30 candidates |
|---|---|---|
| 1 pass (the cells flow) | 754 / 14 / 765 / 3, 180 M, 258 KB | 1 001 / 45 / 1 035 / 11, 396 M, 258 KB |
| 2 passes | 959 / 33 / 982 / 10, 595 M, 295 KB | **1 197** / 78 / 1 254 / 21, **1 195 M**, 299 KB |
| 3 passes | 981 / 38 / 1 007 / 12, 1 116 M, 299 KB | 1 209 / 80 / 1 267 / 22, 1 891 M, 302 KB |
(the table above is the **first** build, every candidate decoded in every pass; 23a is the landed one)
| `ft8dec` (its own list, 1 / 2 / 3 passes, 335 / 850 / 1 278 M) | | 990 / 1 189 / 1 204 jt9 hits |

**The plan's bar** (found of the union >= +80 at 30 candidates for <= +250 M, total <= 650 M): **recall met by far (+219), cost not met (+800 M)**; equal to `ft8dec` in recall for 1.4 times its instructions at 2 passes (1 195 against 850 M), 1.5 times at 3 (1 891 against 1 278 M).
**Where the second pass's 800 M goes** (30 candidates; temporary probes): the streaming subtraction and the residual's demux 287 M (the demux about 76, the subtraction about 210: 8 M a signal, no FFT but about 80 instructions a sample for the correlation and the subtraction together),
the restricted scoring 46 M, **decoding the new candidates 466 M**. Cuts measured: the second pass decoding 20 / 15 / 12 / 10 of its candidates: 1 196 / 1 192 / 1 185 / 1 175 hits at 991 / 876 / 824 / 791 M; a hard-sync minimum of 9 / 11: 1 197 / 1 194 hits at 1 089 / 972 M;
**not subtracting the weak signals does not pay** (a floor of -15 dB on the flow's `snr_db` saves 64 M and loses 33 hits, -10 dB loses 127; every decoded signal is worth subtracting). At 15 candidates the second pass takes 15 of its own: 959 hits at 595 M, 922 at 476 M with 8.
Not tried: a cheaper subtraction loop (the sine table's interpolation and the transition zone's per-sample `ph_inc` are the large part), the decode of fewer candidates by a better gate, a pass cap proportional to `max_cand`.

**The 10 new decodes in neither list** (2 passes, 30 candidates; wrong tones of 79 against 45-60 for a wrong message at the same place, section 20's tool; callsigns heard in other slots): `CQ OR18OSB` 9 (11 slots), `CQ 2E0LDW IO70` 13 (17), `F5CCX F4AGZ JN38` 34 (24 / 12): **real**;
`JO1COV PD0MNO JO22` 41, `MM0IMC 4U1A -06` 43: probably real; `EA8PP JH0INP PM96` 44: the message `band_baseband.rs` already keeps as by-identity debt; **`LU5HA UA9TK R-13` 60 and `LU5HA UA9TK R-20` 50 read like noise**
(two slots, one pair of callsigns, different reports: a subtraction ghost is the likely reading, **not established**); `<...> F6DEO/QRP`, `<...> E77VM R-11` not checkable (hashed call). Noise-only slots (tier C) not run: a SIC's risk is exactly such a decode.

**Tests** (tier A, mutation-checked; 8 new: 7 in `ft8dec_sic.rs` after the review's fading test, and 1 in `coarse_lib.rs`): the subtraction removes a clean synthetic signal (the reference: the library's own GFSK synthesis, known tones and place) to under -25 dB and a misplaced one (1.5 Hz, 40 ms) by more than half, two signals 60 Hz apart both, the residual
is the same for read sizes 100 / 777 / 3 072, the phase-step correction pulls 0.6 Hz off to within 0.2 Hz (the GFSK transition, the sine phase, the amplitude interpolation, the lag, serving a sample before it is final, the amplitude scale and the sign of the correction caught);
a strong station hides a weak one 12.5 Hz away that two passes decode and one does not (the `tcal`, either sign of the refined place, `NEAR_CELLS` and no-subtraction mutations caught); one pass decodes `decode_block_cells`'s messages; a decode reports the refined place
(a frequency or DT sign flip caught); the restricted search returns only candidates within the radius, the signal among them, none with no `near` (ignoring `allowed` caught).
**Not done**: noise-only slots; ESP32; a board's ring and the demux fed the residual (`stage1_inc`); the cheap loop; the pass cap; `reviewer`, `scripts/verify.sh`.

### 23a. After the cut: a pass decodes fewer candidates (same day)

(The table below was measured with a hard-sync minimum of 9 next to the cap; the landed code uses `ft8dec`'s 7, which changes the figures by a few hits and instructions: the landed ledger is in 23b.)

**What changed**: a pass after the first decodes at most `max_cand / 2` candidates (8 at least), and only those with a hard-sync count of 9 or more of the 21 Costas symbols (`PASS_CAND_DIV`, `PASS_CAND_MIN`, `PASS_MIN_HS`; the measurements above chose them). The number of passes is the caller's (`passes`; `icap-profile --passes N`).

| | 15 candidates | 20 | 24 | 30 |
|---|---|---|---|---|
| 1 pass: jt9 hits, M instr | 754, 180 | 912, 233 | 979, 280 | 1 001, 396 |
| 2 passes | 922, 488 | 1 099, 593 | 1 162, 701 | **1 192**, **881** |
| 3 passes | 940, 849 | 1 114, 1 043 | 1 175, 1 200 | 1 206, 1 405 |

(union found of 1 409: 30 candidates 1 035 / 1 247 / 1 262; in neither list 11 / 21 / 22; `ft8dec`: 990 / 1 189 / 1 204 hits at 335 / 850 / 1 278 M.) Against the plan's bar the second pass now costs +485 M at 30 candidates (was +800): the recall bar is met (+212 of the union), the +250 M is not;
at 2 passes the flow is 1.04 times `ft8dec`'s instructions for 1 192 against 1 189 hits, at 3 passes 1.10 times for 1 206 against 1 204. **To pass `ft8dec`'s single pass (990 hits, 1 026 in its own list) the cheapest of these is 24 to 30 candidates in one pass (979 / 1 001 hits at 280 / 396 M), or 20 candidates in two (1 099 at 593 M)**; at the CoreS3's 15 candidates
no number of passes gets there (940 at three): the first pass is limited by the candidates it can afford, and a later pass only takes what its subtraction uncovers. The 22 recordings: 275 / 332 of `jt9`'s 402 at 15 / 30 candidates and 2 passes.

### 23b. After the `reviewer`'s pass on the port (same day)

The port was found faithful line for line (`sig_corr`, `sig_sub`, `ph_inc`, `ph_at`, `psc`, `GSTEP`, the lag, the frequency correction, the pass loop, `refine`, the ring against the demux's reads). What was found and done:
* **The amplitude interpolation was less pinned than its docs said**: a ramp removed (`dar = 0`), no recompute at `wi == half` and `k1 = ks` left every test green (a clean constant-amplitude signal makes every correlation alike). A fading-signal test was added (a ramp from 1 to 5, a 0.78 Hz swell of 80 %, a start off the half-symbol grid): it catches `k1` (-20.2 dB against -27.8) and **not** the ramp slope (1.1 dB) or the recompute at the half (no change: the interpolation is not what limits the depth here). Stated in the test.
* **`PASS_MIN_HS` of 9 was measured without the cap**; with the cap it changes the cost by 5 M and the hits by 0. It is now `ft8dec`'s own 7 (`nsync_min`), which rejects nothing here. The cap (`PASS_CAND_DIV`, `PASS_CAND_MIN`) is marked as a divergence from `ft8dec` in the code.
* **Line pointers** into `ft8dec.c` corrected (`sig_corr` 243-279, `sig_sub` 280-319, `build_wf` 321-379, `binok` 833-841, `round_` 830-911).
* **The restricted search's edge**: `ft8dec` skips a candidate not strictly within 9 bins (56.25 Hz) of a new signal, with the frequencies corrected by the pass; the mask of `search_near` is made before the correction and is inclusive (up to 57.8 Hz). A post-check with the corrected frequencies and the strict limit was added (the 22 recordings and the ledger unchanged to the decode).
* **`passes` 1 is not `decode_block_cells` bit for bit**: the same messages, at refined places, 396 against 382 M at 30 candidates (180 against 165 at 15: the place refinement and the signal bookkeeping; the split is not measured). The gain is no longer read from the audio when `passes` is 1. The docs say so.
* **The new decodes in neither list, re-checked on the landed configuration** (2 passes, 30 candidates: **9 new**, 20 in neither list in all): `2E0LDW` 13 and `F5CCX F4AGZ` 34 wrong tones real; `JO1COV PD0MNO` 41 and `MM0IMC 4U1A` 43 probably real; **`LU5HA UA9TK R-13` 60 and `R-20` 50 read like noise**; `F6DEO/QRP` and `E77VM` (hashed calls) not checkable; `EA8PP JH0INP PM96` 44: **attested** now (below). The earlier count of "5 real" was too strong: 2 real, 2 probable.
* **`EA8PP JH0INP PM96`** was classified two ways: tier B (`ft8_real_recordings_vs_jt9.rs`) attests it for `websdr_test7` (`jt9`'s exact text, same 940 Hz and DT -0.6, in `websdr_test5`), `band_baseband.rs` kept it as by-identity debt. It is in `profile::ATTESTED_EXTRAS` with that evidence and out of the SIC floor test's debt; `band_baseband.rs`'s list is left as it is (a message that stops being unexplained is allowed to leave it).
* Landed ledger (60 slots, jt9 hits / union found / in neither, host M instr): 15 candidates 754 / 765 / 3, 180 M; 922 / 941 / 8, 488 M; 939 / 960 / 10, 859 M (1, 2, 3 passes). 20: 912, 233; 1 097, 594; 1 111, 1 055. 24: 979, 280; 1 161, 699; 1 174, 1 218. 30: 1 001 / 1 035 / 11, 396 M; 1 192 / 1 247 / 20, 881 M; 1 206 / 1 262 / 21, 1 443 M.

## 24. The board plan

The CoreS3 side (a 2 560-point esp-dsp plan, the byte producer in `stage1_inc`, the gate on a partially filled store, stage 3 by work stealing from the cells, the subtraction
passes after the slot boundary, the internal-DRAM / PSRAM placement under `SPIRAM_MALLOC_ALWAYSINTERNAL` = 2048) is planned in `S3_CELLS_IMPLEMENTATION_PLAN.md`: what is established
(sections 2-3 there), the steps and their gates, the decisions that are the owner's, and what is not known (the ESP32 time of integer / table code, PSRAM bandwidth, the early emit on a partial store,
OSD on the board, tier C). Nothing there has been run on a board.

## 25. Step 0 of the board plan as built (host, 2026-11-26)

Three commits (`7d14c115`, `d0d39689`, `9f41fe0a`), host only, each with its tier A / B and fixed-point gates and a mutation check of every new test.

* **0a, `CellsWorker`.** The loop body of `process_candidates_with_ap` is `process_candidate_staged`; `CellsWorker` borrows the caller's staging block, BP scratch and tables. Decodes of
  `decode_block_cells` unchanged to the bit; host instructions within +-0.5 % (a host allocator is cheap: the saving is the board's heap traffic and the PSRAM `Box`, **not measured here**). Counting allocator
  (`hosttest/icap-app`): a failing attempt allocates no block and no scratch of its own. **Correction to the plan**: "zero allocations" is false, the shared `engine::llr` scratch (<= 696 B, <= 2 KB: internal on the board) is made per attempt; making it allocation-free is a separate change.
* **0b, a store written a row at a time.** `view_of` borrows the store with `n_time` = the fill, so the scoring needs no new rule. Mutation checks found two gaps in the first tests (the view reading `n_time`; a part dropping its last carrier);
  the tests now use pseudo-random cells past the fill and a signal on the last carrier, and fail both. The search of a complete store: same instructions.
* **0c, 48 -> 8 kHz.** `antialias_taps(src, target)`; 8 kHz spec pass 3 200 / stop 4 000 Hz / 40 dB. 12 kHz outputs pinned by FNV hashes taken from the previous commit. 22 recordings via 12 -> 48 -> 8 kHz: 225 / 285 against the ICAP model's 226 / 286 (extras 2 / 3).
* **Not done: step 0d** (OSD-off ledger of the cells flow and the passes; the early emit at row pair 87 / 85 on a partly filled store). Both are host measurements that need no new library item.

## 26. Where the cells flow's instructions go, against `ft8dec` (Apple M5, 2026-11-27)

`scripts/compare-ft8dec-cells.sh` on the Mac (`/usr/bin/time -l`, 3 vendored recordings, 8 kHz, 30 candidates, OSD off), after the x86 run in `S3_CELLS_IMPLEMENTATION_PLAN.md` section 10 found the cells flow dearer than `ft8dec`:
M instructions, sum of 3: cells 915 (15 candidates) / 1 479 (30); `ft8dec` 444 (30 candidates, OSD off, 1 pass) / 741 (its defaults, 300 candidates, OSD, 1 pass). **The direction is the x86 one, not reversed** (cells 3.3x `ft8dec` at the same knobs; 2.0x its defaults).
Wall time, same audio (ms, 3 recordings, load subtracted): cells 59 (30 cand.), 34 (15) against `ft8dec` 13 (30 cand., OSD off), 32 (defaults): 4.4x, so time is further apart than instructions. A sampled profile of the same runs (OSD off) puts **BP at 60 % of the time** and the search at 17 %, the FFTs at 16 %, the LLR and the cell fill at about 1 %: BP is an instruction-poor, time-rich stage (about 17 % of the instructions, 45 000 a BP iteration on f32 NMS, measured on random LLRs).

**BP calls, counted with a probe (since removed), 30 candidates, 3 recordings:**

| | cells flow | `ft8dec` |
|---|---:|---:|
| candidates that reach BP | 87 | 35 |
| of them decoded / failed | 19 / 68 | 19 / 16 |
| BP calls on a failed candidate | 2.7 | 2.0 |
| iterations a BP call | 29.6 | 24.1 |
| BP iterations in all | 5 666 | 919 |

A decoded candidate costs 1-2 calls and about 10 iterations; **95 % or more of the iterations are failed candidates**. The 6.2x in iterations is 2.5x (candidates tried) x 1.35 (calls a failed one: `a` and `d` at the grid, and again at the retry) x 1.23 (iterations a call: the stall exit rarely fires, a failed call runs all 31 passes; `ft8dec` stops at 25 and exits early). `ft8dec` tries fewer because it takes the top 30 by score, then skips those under 7 hard-sync tones and those within +-2 steps of a decoded candidate; the cells flow takes the top 30 by hard sync, all of which have 6-21. Two guesses ruled out: no failed candidate lies within +-2 rows / +-2 bins of a decoded one (the search's suppression already removed them), and a hard-sync floor of 7 would remove 14 of 208 calls.
**Explained (x86, 2026-11-27):** the x86 record's 179 BP calls on `qso1` at 30 candidates are 63 calls of the staircase (`bp_decode_generic_nms_with_scratch`: the 63 counted here) plus 116 of `bp_decode_generic_kind_with_scratch` (SumProduct), which are the **blind-CQ AP pass** of the host build (`PassCtx::FIRST` has `ap: true`; with `fft-rustfft` every candidate that does not decode goes on to it; the board's build has no such code). **The Mac run above is of the host build too, so its instruction and time ratios (3.3x, 4.4x) include that pass**; with it off (x86, `PassCtx::FIRST.without_ap()`) the cells flow is 1.54x `ft8dec`'s instructions at one pass / 30 candidates (`S3_CELLS_IMPLEMENTATION_PLAN.md` section 10). The BP counts of the table (87 / 35 candidates, 2.7 / 2.0 calls, iterations) are the staircase's and hold for the board. Not re-measured on the Mac with the AP pass off.

**BP iteration cap 30 -> 25 / 24** (24 = `ft8dec`'s 25 passes), 60 slots, OSD on, cells flow: **hits, extras, union found and "in neither list" are identical** at 15 and 30 candidates, 1 and 2 passes (754 / 1 001 / 922 / 1 192 `jt9` hits); instructions at 30 candidates 398.0 -> 387.8 (-2.6 %) / 385.7 M (-3.1 %), at 2 passes 884.9 -> 868.3 / 864.8 M, at 15 candidates -0.6 %. The saving is small because the number of failed BP calls is unchanged. **Not applied** (a change of `DEFAULT_BP_MAX_ITER` for the cells flow only; safe on this corpus, worth 2-3 % of instructions, a little more of BP's time). The levers that matter are the candidates tried (87 against 35) and the calls on a failed candidate (2.7 against 2.0): each costs recall (754 -> 1 001 hits from 15 to 30 candidates) and needs the 60-slot ledger and the board's time before any is chosen. Instruction counts understate BP: the board's time should be predicted from BP iterations, not from instructions.

**The corpus.** `20m_busy` is `kgoba/ft8_lib` `test/wav/20m_busy` (MIT; 38 slots, 76 files with their text), read from a clone at `~/src/ft8_lib` (commit 9fec6ca), **not vendored in this repo**: the 60-slot ledgers are reproducible only with that clone and the references of `jt9` v3.3 beta6 (the 22 recordings are vendored). The design was tuned on the same 60 slots (no held-out set).
**On the board:** a 12-slot replay (`test_01`..`test_12`, about 4.2 MB of the 9 MiB factory partition, one slot at a time from PSRAM, 240 KB at 8 kHz) would give the board's time per BP iteration against these counts; 38 slots do not fit in one image. Not built.


## 27. Where the cells flow's extra instructions are, against `ft8dec`, by function (host x86, board's build, 2026-11-27)

`ft8dec` 1 pass, 30 candidates, OSD off, built with `-fno-inline` so its stages show as functions (210.6 M instructions against 209.7 M inlined, qso3_busy): `build_wf` (the spectrogram: `rfft` 81 M, `rpow` 11 M) 117 M, `find_candidates` (`sync_score`) 62 M, `decode_cand` 25 M (BP 23.6 M). The cells flow on the same slot (`scripts/s3-stage-instr.sh`, 30 candidates, OSD off, no AP): the FFTs 88 M (+ about 16 M of demux and cell fill), `band_best` 64 M, stage 3 144 M (BP 139 M). **The spectrogram and search sides are the same size (ft8dec 179 M, cells 168 M); the whole excess is stage 3's BP, 139 against 24 M.** **Correction (same day, measured slot by slot):** a BP pass does **not** cost the same: `qso3_busy` 139 M in 2 145 passes was about 65 000 instructions a pass by the division used here (139 M of one run over 2 145 passes of another; **the same-run figure is 76 000**, qso1 and qso2 the same, measured later the same day: BP instructions of a run over the passes `BpStats` counted in it), against `ft8dec`'s `bp()` 23.6 M in 488 passes, about 48 500 (qso1 48 800, qso2 48 300; counters of `scripts/probe/ft8dec-bpstat.patch`); the first figure of this paragraph divided the iterations of three recordings by the instructions of one. So the 5.8x of `qso3_busy` was 4.4x in passes and 1.34x (by the same-run figure 1.57x) in the instructions of a pass (the cells flow's NMS pass is dearer than `ft8dec`'s sum-product one: a lever on its own, not examined); the number of iterations is 6.2 times on the three recordings (section 26). The stall exit is the same rule as WSJT-X's in both (`bp.rs` 385, `ft8dec.c` 465).

**Which candidates are tried.** The 90 candidates of qso3_busy, qso1 and qso2 (3 x 30) against `ft8dec`'s `CAND` lines (`FT8_DEBUG`) at the same place (+-6.3 Hz, +-0.2 s): 19 at places `ft8dec` decoded (cells decoded 18), 7 where `ft8dec` tried and failed (cells decoded 1), 4 where its hard sync was under 7, and **60 where `ft8dec` never tried (cells decoded none; scores 8-15, mostly 8-10)**. `ft8dec` takes the top 30 by score from a grid oversampled 2x in time and in frequency, so the neighbours of one signal fill its list and a decoded one skips the rest; the cells flow's 30 are 30 places apart (the search suppresses +-2 bins / +-2 rows) and all go to BP. Three recordings only, and ones where 30 candidates decode what 15 do.

**Yield of a candidate** (`icap-cands` + `scripts/cands-yield.py`: 60 slots = the 22 recordings against `jt9` beta6 and `ft8_lib`'s 38 `20m_busy` against their `.txt`; 1 800 candidates, 975 decodes in the reference, 24 decoded outside it; the corpus the design was tuned on, no held-out set): score 8-10: 266 tried, **0** in the reference; 11-12: 83 tried, 3; 13-15: 104, 15; 16-20: 293, 98; 21-30: 823, 643; over 30: 231, 216. By hard sync: under 9, 172 tried and 1 decoded. By rank in the list (ordered by hard sync): 25-29 yield 4.7 % (14 of 300), 20-24 21.7 %, 0-4 92.7 %. A floor of score 11 would have skipped **266 of the 1 800 tries (14.8 %) and lost none of the 975**; score 12 loses 1, 13 loses 3, 15 loses 11. Skipping by hard sync under 9 loses 1 for 9.6 % of the tries. The floor is `ft8dec`'s `min_score` (8, the same as the cells search's) moved up; **a change of the cells flow's candidate rule: not applied, it needs an Issue and the owner's answer, and a held-out slot set** (the tail ranks 20-29 carry 79 of the 975).

**Passes measured (`BpStats`, same day).** The BP iterations of the table in section 26 and the estimate above (85 500) were by probe and by assumption (30 passes for a failed call, 10 for a decoding one); `BpStats` counts them: **3 516 calls and 76 155 message-update passes** on the 60 slots (21.7 a call), 95.2 % of them on candidates that do not decode. By score: 8-10 11.2 + 9.0 + 7.9 = 28.1 % of the passes for 0 decodes; 11-12 9.4 % for 3; 13-15 11.5 % for 15; 16-20 25.5 % for 98; 21-30 23.7 % for 643; over 30 1.8 % for 216. **A floor of score 11 saves 28.0 % of the passes (21 354) and loses none of 975; 12 saves 33.1 % and loses 1; 13 saves 37.5 % and loses 3; 15 saves 46.0 % and loses 11.** By hard sync: under 9 saves 16.6 % of the passes and loses 1; under 10 saves 32.0 % and loses 2. The tail ranks 25-29 cost 32.6 % of the passes for 14 decodes (1 774 passes a decode against 6.5 for ranks 0-4). `scripts/cands-yield.py` prints these from `icap-cands`' TSV. Not applied.

**A larger population (2026-11-27; `scripts/cands-corpus.py`, `icap-cands`, `scripts/cands-yield.py --by-corpus`).** The 60 slots are the corpus the design was tuned on, and a score floor looked free there. Over **4 780 slots, 143 400 candidates, 6 935 decodes in the reference** (the 60 real slots; the generated tier-C sets: `busy` 420 slots of 10 / 20 / 40 signals and DT offsets, `dense` 200, `sweep` 1 040 and `itu` 3 060 single-signal SNR ladders over AWGN / CCIR / ITU channels; the 200 noise-only slots of `busy` decode nothing and add no phantom): 99.7 % of the 7.4 M passes go to candidates that do not decode (the generated sets are mostly noise candidates; the real ones, 96.6 % and 94.1 %), and **the score floor is not free**. Decodes lost of 6 935: score under 10: 7, under 11: 19, under 12: 35, under 13: 62. They are the sensitivity edge: slots decoded of 20 at -18 dB, `ccir_poor` 6 -> 5 / 4 / 3 for floors 10 / 11 / 12, `itu_lm` 2 -> 2 / 0 / 0, `itu_md` 5 -> 4 / 4 / 4. **A hard-sync floor is**: `hs < 8` (`ft8dec`'s own is 7) loses **0 of 6 935** and changes no slot of the SNR ladders, `hs < 9` loses 4 (2 slots at the edge: `awgn` -21 dB 1 -> 0, `ccir_moderate` -18 dB 6 -> 5). Passes saved by `hs < 8`: 14.1 % on the 22 real recordings, **0.0 % on the 38 busy real slots** (every candidate there has at least 8), 62 % `busy`, 18.8 % `dense`, 73 % `sweep` / `itu`. Rules that lose none of the 6 935: `score < 10 and hs < 10` (saves 30.5 % of the passes on the 22 recordings, 6.2 % on the 38 busy slots, 39.5 % `dense`), `score < 12 and hs < 9` (31.8 / 2.5 / 45.9 %); `score < 11 and hs < 11` loses 6 (all at the edge). **So on a busy real band the BP passes are not on candidates with a low score or low hard sync: most of the 94 % are candidates with hard sync 10-15 that do not decode**, which no floor of these two reaches. Not applied. The generated sets are noise and single-signal slots, not a band; the real slots remain 60, and no real held-out set exists on this machine.

**Where the passes go on a busy real band, and what an early abort of BP would save (2026-11-27; `scripts/bp-abort-sim.py`, `scripts/probe/bp-trace.patch`: a local probe, not committed code).** The 38 busy slots of `ft8_lib`, passes on candidates that do not decode (41 372): hard sync 10 or more with **no listed signal at that place (205 candidates, 47.2 %)**; a listed signal there that no candidate decoded, i.e. a *missed* signal (138 candidates, 33.8 %: BP failing on a real signal, not waste); hard sync under 10 with no signal (17.1 %); the rest 1.9 %. The 22 recordings: 49.3 % are hard sync under 10 and no signal, 25.7 % hard sync 10 or more and no signal, 21.4 % missed. Of the failed candidates with hard sync 10 or more (b: 343), 181 lie over 50 Hz from every decode and 147 within 50 Hz; only 15 are within 12.5 Hz of one, so they are not the neighbour cells of a decoded signal. The ncheck of every BP call (the number of unsatisfied parity checks at each pass) was recorded for all 143 400 candidates (256 864 calls; the passes of the traces, 7 449 550, are `BpStats`' 7 448 386 within 0.02 %) and a rule "abort a call at pass k when ncheck is still above T" simulated; a call that decoded and is aborted counts as a **lost decode** even if another variant would have rescued it (pessimistic). Of 6 935: **(k 6, T 24) saves 10.3 % of the passes and loses 0; (k 15, T 20) 9.5 %, 0; (k 4, T 28) 5.5 %, 0**; with 1-3 lost: (k 4, T 24) 15.8 % / 3, (k 10, T 24) 5.6 % / 0, (k 8, T 20) 19.7 % / 2, (k 10, T 20) 15.7 % / 2, (k 15, T 16) 21.2 % / 2; the 22 recordings and the 38 busy slots alone lose none at (k 15, T 16) and save 20.9 % / 21.1 %, (k 12, T 16) 26.6 % / 28.4 %. Earlier, tighter rules (k 4, T 16: 59.6 % saved, 41 lost) show where the real signals sit: their ncheck is high for the first passes too, so **an abort by ncheck alone saves 10-20 % at no cost, not the 60 % the failing calls' share of the passes would allow**. The WSJT-X stall rule is already in (`ncnt >= 5 and iter >= 10 and ncheck > 15`). Not applied: a change of BP's stopping rule is a divergence from `bpdecode174_91.f90` (Issue and the owner first).

**`ft8dec` at the same knobs, 60 slots (2026-11-27; correction of the paragraphs above).** `ft8dec` (icap-sdk 1ca9f04) at 30 candidates / OSD off / 1 pass / no AP on the same 8 kHz audio (the libraries' resample, dumped by `icap-cands`; debug build for the `CAND` / `dec` lines; counters added to its `bp()` by `scripts/probe/ft8dec-bpstat.patch`, a local probe on the clone), against the cells flow at 30 candidates (`icap-cands`, `BpStats`). The earlier statements that `ft8dec`'s list "is filled by neighbour cells" and that it "decodes fewer" were **not measured**; measured: **candidates that reach BP** 1 132 of 1 800 (63 %; the others were skipped as next to a decoded one, or hard sync under 7) against the cells flow's 1 800; **decodes in the reference** 822 (22 recordings 231, 38 busy 591) against 975 (289, 686), so `ft8dec` is 16 % lower at the same knobs (the three recordings of the first comparison were equal, 19 and 19: they are not the corpus); **BP calls** 1 410 against 3 516 (1.25 a tried candidate against 1.95); **BP passes** 13 929 against 76 155 (9.9 a call against 21.7; `ft8dec`'s cap is 25 passes, the library's 30); **passes per decode in the reference 16.9 against 78.1**. The 5.5x in passes is 1.6 (candidates tried) x 1.6 (calls a candidate) x 2.2 (passes a call), for 19 % more decodes. The calls a candidate is the staircase: the cells flow runs `a` and `d` at the grid and again at the retry, and `d` yields 31 decodes in 837 calls (3.7 %), the retry 14 in 893 (1.6 %) (section above); `ft8dec` makes one BP call a candidate (`llr` 2) and the interpolated retry only on failure. Dropping `d` and the retry would cost at most 45 of 975 decodes (4.6 %) and, since nearly all of those calls fail and run to the cap, about 60 % of the passes; the measured rules are in the list above (no retry at all: -31 % passes, 14 lost; score under 16 without `d` or retry: -29 %, 1 lost).

**The staircase's `d` and the retry, over 4 780 slots and the SNR ladders (2026-11-27; `scripts/probe/staircase-variants.patch` is a local probe, `scripts/staircase-variants-report.py` the report).** Variants of `decode_candidate` (the cells flow, 30 candidates, OSD off, no AP): `base` (`a` and `d` at the grid, then at the retry), `nore` (no retry), `nod` (no variant `d`), `a` (neither), `lowa` (score under 16: `a` only), `dlow16` / `dlow21` (`d` only from score 16 / 21 up; the retry stays). Decodes in the reference of 6 935 and BP passes (the 7.45 M of the base = 100 %): **`nore` 6 751 (-184) 75.1 %; `nod` 6 854 (-81) 50.1 %; `a` 6 658 (-277) 37.6 %; `lowa` 6 854 (-81) 38.7 %; `dlow16` 6 931 (-4) 50.9 %; `dlow21` 6 907 (-28) 50.3 %.** Slots decoded on the SNR ladders (of 1 477): `nore` 1 467, `nod` 1 476, `a` 1 466, `lowa` 1 471, **`dlow16` 1 477, `dlow21` 1 477**: **the retry is what the sensitivity edge needs** (ccir_poor -18 dB 6 -> 4, ccir_good -20 dB 4 -> 3 without it) and `d` is not (one slot, ccir_good -17 dB, without it). The 60 real slots (975 in the reference): `nore` 961 (-14) 65.3 % of the passes; `nod` 945 (-30) 52.1 %; `lowa` 972 (-3) 67.3 %; **`dlow16` 974 (-1) 75.5 %**; `dlow21` 971 (-4) 63.0 %; per corpus `dlow16` passes: 22 recordings 21k of 32k, 38 busy slots 35k of 43k (-19 %), `busy` 358k of 685k, `dense` 166k of 288k. The call-level yields of the section above (`d` 31 decodes in 837 calls, the retry 14 in 893, on the 60 slots) understated both on the larger set (81 and 184). **`dlow16` (`d` only for a candidate of score 16 or more) saves 49 % of the passes over the whole set and 24.5 % on the real slots, loses 4 decodes of 6 935 (1 of 975 real) and no slot of the ladders.** Not applied: it changes the staircase of the cells flow (a divergence: Issue and the owner first); the real slots are the 60 of the tuning set and the ladders and `busy` / `dense` are generated.

**Landed: `d` from score 16 (Issue #765, 2026-11-27).** `CellsWorker::with_d_min_score(CELLS_D_MIN_SCORE = 16.0)`; the library's `decode_block_cells` and `decode_block_cells_sic` set it, `CellsWorker::new` keeps the whole staircase. `scripts/s3-stage-instr.sh` after the change (30 candidates, OSD off, no AP, M instructions; `qso3_busy` / `qso1` / `qso2`): total 312 / 286 / 303 -> **276 / 239 / 249**, stage 3 144 / 119 / 135 -> 108 / 72 / 81 (BP 139 / 115 / 131 -> 103 / 68 / 77); 15 candidates 224 / 230 / 239 -> 220 / 206 / 212; two SIC passes 628 / 484 / 533 -> 575 / 434 / 468; decodes 9 / 4 / 5 (11 / 4 / 5 at two passes) unchanged. The corpus, the measurement and the limits are those of the paragraph above; board time is not measured.

**A pre-BP gate (2026-11-27; `scripts/cands-gate-analysis.py`, `scripts/probe/llr-features.patch`: a local probe).** What the candidate rules (score, hard sync) and an abort by ncheck cannot reach is the failing candidate of hard sync 10-15 (55 % of the real slots' passes). ncheck alone does not separate it: at pass 2 the calls that decode have ncheck under 30 (619 calls) but 223 000 failing calls sit in 10-29 as well; at pass 0 (before any message passes) `ncheck > 46` saves 13.8 % of the passes and loses 9 of 6 935, `> 42` saves 41.8 % and loses 47. **The statistics of `llra` do**, with the score and the hard sync: a logistic model over the first attempt's mean, standard deviation and maximum of |llra|, mean / std, the search score and the hard sync (six inputs, no sort), **trained on the generated sets only** and with its threshold set at the lowest score of any true decode there (so none of the 5 700 true decodes of the generated sets is lost), loses **0 of 974 decodes on the 60 real slots and saves 29.1 % of their BP passes (22 recordings 39.2 %, 38 busy 22.9 %)**; with the score and hard sync alone the same procedure saves 9.9 %, without the hard sync 20.6 %, with the sorted statistics (median, p10, p90) in addition 27.3 % (oracle threshold on the real slots 34.4 %). Leave one generated family out (train on the other three, threshold from them): `dense`, `sweep` and `itu` lose none (41.0 / 91.6 / 89.7 % of their passes saved), `busy` loses 2 of 1 677 (81.9 %). Single statistics are poor (the best saves 10 % of the generated sets' passes, 1-2 % of the real slots', at no loss). This is on top of the staircase's `d` floor (the data are the shipped flow's); the passes of the 60 real slots go from 57.5k to about 41k, `ft8dec`'s 13.9k are then 3.0x away (from 4.1x). **Not applied and not a small change:** a learned gate (six coefficients, a threshold, from generated data) before BP is a new divergence of the cells flow, it can only skip candidates (it cannot create a decode or a phantom), its precision on a real band is the 60 slots' (no held-out real set), and the board's cost (three sums over 174 values and six multiply-adds against a pass of about 65 000 instructions) is predicted, not measured.

**The NMS pass after its flat edge tables (same day, `scripts/bp-instr-per-pass.sh`: BP instructions of an `icap-cands` run over the `BpStats` passes of it, 3 recordings, floor 16 and 0): 76 000 -> 37 600 host instructions a pass** (`ft8dec`'s `bp()` about 48 500: 23.6 M in 488 passes on `qso3_busy`, 14.9 M in 306, 4.7 M in 97), bit for bit the same results (test `the_flat_table_nms_decoder_equals_the_searching_one`). The tables are built at compile time (`LdpcParams::NMS_EDGES`, `edge_arrays`; every index range-checked by compile-time assertions) after a first version that built them in the scratch (+3.4 KB of the CoreS3's internal DRAM a scratch: the review of 86308daa measured -4 588 B of minimum free internal in the bench).

## 28. The OSD fallback on the cells flow: which seeds return, and which attempts are worth it (host, 4 780 slots, 2026-11-27)

**Method.** Local probes (`scripts/probe/osd-attempts.patch`; the analysis is `scripts/osd-gate-fit.py`): every `try_fallback` call of the cells flow with OSD on over the 4 780 slots (22 vendored + 38 `20m_busy` real, generated busy / dense / sweep / itu), 143 400 candidates, **124 765 OSD attempts**; the winning run of each. Generated sets train, the 60 real slots judge.

**Seeds (`OsdSeeds`).** Returns by run (variant, BP snapshot): `a,1` 404 true / 11 not in the reference; `a,2` 44 / 1; `d,1` 110 / 8; `d,2` 20 / 2; **`b`, `c`, `e` (six of the ten runs): 0 / 0**. `OsdSeeds::AD` (runs `a1 a2 d1 d2`) gives the same decodes on all 143 400 candidates (7 442; 7 397 in the reference): host `try_fallback` 319 / 286 / 337 -> 119 / 104 / 122 M instructions (`qso3_busy` / `qso1` / `qso2`), CoreS3 stage 3 with OSD 4.45 / 4.34 / 4.57 s -> **1.78 / 1.65 / 1.72 s** (`ft8-stage-bench`; OSD off 0.62 / 0.49 / 0.47 s). `osd174_91_into` (u32 rows, compile-time columns, caller's memory) is `osd174_91` bit for bit and 10.09 -> 7.26 ms a call on the board (`ft8-osd-bench`). `osd174_91` evaluates 28 words a call on average (2.0 of 91 order-1, 26 of 4 095 pairs pass the `ntheta` gate, 92 % of those stop early), so a call's cost is the 4 095 gate tests, the sort and the elimination, not the search.

**Still not in the window.** AD's OSD adds about 1.2 s to the slot (about 35 attempts at about 33 ms), against a 500 ms budget after the boundary. 578 of the 7 442 decodes (7.8 %) are OSD's, 60 of the 974 on the real slots.

**A gate on the attempt (6 inputs: `|llra|` mean / std / max / mean / std, hard sync, search score, + the BP's unsatisfied checks).** Population matters: OSD succeeds on 5 % of the attempts of the real slots (60 of 1 208) and on 0.4 % of the generated ones (518 of 123 557), so a threshold fitted on the generated sets runs OSD on **70-83 % of the real attempts** while keeping 60 of 60 successes (82 % with llra statistics and hard sync; no better with the BP's checks), against 33-67 % of the generated. 99 % of the generated successes: 70 %.

**A cap per slot (top K of the attempts by the model's score; the OSD pass after the slot's BP work).** Real slots (about 20 attempts a slot): K = 4: 20 % of the attempts, **36 of 60** successes; K = 6: 30 %, 44; K = 8: 40 %, 51; K = 10: 50 %, 56; K = 14: 69 %, 58. The search score alone: K = 6 keeps 36, hard sync alone 41, `mean / std` alone 26 (the model 44). A cap of 6 costs about 0.36 s of the 1.2 s and loses 16 of the real slots' 974 decodes (1.6 %). Not built: it needs the OSD pass to run after the slot's BP work, over a list of pending candidates (a structural change to the flow), and a policy (K, or a time budget) that is the owner's.

**Not done**: the same on the board (the cap is arithmetic on the board's 33 ms an attempt); a model fitted on real slots (60 slots, 60 successes is too few); the cap on a time budget in place of K; OSD later than the slot's window (the auto-QSO's late decodes feed the next TX opportunity).

## 29. The OSD in the SIC (host, 4 780 slots, 30 candidates, seeds `a`, `d`; 2026-11-27)

**Method.** `icap-sic` (`hosttest/icap-app`): the cells flow's `sic_passes` per slot with the OSD (a) off, (b) at once for every failed attempt (`inline`), (c) `round`: `sic_passes_osd`, every round (pass 0 and each later pass) queues the attempts that fail the BP staircase and runs the OSD for the best K (`OsdRank`, section 28), the OSD's decodes joining `known` and so the next pass's subtraction, (d) `final`: the rounds queue, one OSD pass of K on the final residual cells, nothing of it subtracted. Truth = the slot's reference.

| 2 passes | true decodes, all / real 60 | OSD attempts run | OSD's decodes kept, all (of 494) / real (of 71) |
|---|---|---|---|
| OSD off | 7 847 / 1 121 | 0 | |
| at once | 8 341 / 1 192 | all (131 500) | 494 / 71 |
| `round` K = 4 | 8 250 / 1 162 | 22 140 | 403 / 41 |
| `round` K = 6 | 8 287 / 1 171 | 32 442 | 440 / 50 |
| `round` K = 8 | 8 311 / 1 177 | 42 690 | 464 / 56 |
| `final` K = 6 | 8 226 / 1 150 | 28 680 | 379 / 29 |

3 passes: off 7 904 / 1 130, at once 8 404 / 1 206, `round` K = 4 / 6 / 8 8 314 / 8 349 / 8 373 (real 1 175 / 1 183 / 1 189), `final` K = 6 8 276 / 1 156. The BP work is +1 % (an OSD-only attempt runs one BP pass); decodes not in the reference 43 -> 62 (K = 6, 2 passes), 44 -> 64 (3 passes).

**What it says.** The same number of OSD runs (K = 6 a round is 32 k; one pass of 6 at the end is 29 k) keeps 50 of the real slots' 71 OSD decodes when each round's OSD decodes are subtracted and 29 when they are not: the OSD's decodes are signals, and taking them out of the residual lets the next pass search what they hid. The cap per round is not a cap per slot: with 2 passes a slot runs up to 2 K attempts, 6.8 on average over the corpora (12 on `qso3_busy`). On the three recordings (OSD queued / run): 2 passes K = 6 51 / 32 / 41 queued and 12 / 8 / 12 run, K = 8 16 / 10 / 15 run.

**The board (CoreS3, `ft8-sic-bench`, one core, 30 candidates, 2026-11-27; predictions in its module doc, written before the run, all inside the 25 % stop rule).** SIC with the OSD at K = 6 / 8 a round against the `storage` arm (OSD off), seconds, with the decodes:

| | OSD off | `osd6` (predicted) | `osd6` | `osd8` (predicted) | `osd8` |
|---|---|---|---|---|---|
| `qso3_busy`, 2 passes | 4.31 (11) | 4.55-4.80 | **4.94 (13)** | 4.75-4.95 | 5.11 (13) |
| `qso1`, 2 passes | 3.52 (5) | 3.65-3.85 | 3.86 (5) | 3.75-3.95 | 3.94 (5) |
| `qso2`, 2 passes | 3.57 (5) | 3.85-4.05 | 3.97 (5) | 4.0-4.2 | 4.09 (5) |
| `qso3_busy`, 3 passes | 6.64 (11) | 7.0-7.3 | 7.60 (14) | 7.25-7.5 | 7.77 (14) |

The OSD adds 0.63 / 0.34 / 0.40 s at K = 6 (2 passes; 12 / 8 / 12 attempts run on the host): 52 / 43 / 33 ms an attempt against the 36 ms of `ft8-stage-bench`'s `osd pass`; `qso3_busy`'s higher figure is above the prediction by 3-4 % (a guess, not measured: the extra signals its OSD decodes add to the subtraction). Decodes are within one of the host's (`osd8`, `qso3_busy`, 2 passes: 13 against 14).

**Not done**: K per slot rather than per round; the model's ranking on residual cells (fitted on first-pass attempts); the OSD in the SIC under WiFi / USB host, and the cell store and audio the SIC needs held (plan section 14).

