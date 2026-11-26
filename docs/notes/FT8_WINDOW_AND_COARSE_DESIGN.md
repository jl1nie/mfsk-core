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
